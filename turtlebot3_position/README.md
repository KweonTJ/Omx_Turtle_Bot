# turtlebot3_position

## 목적

`turtlebot3_position`은 ROS 2 Humble에서 ESP32 또는 UWB 장치가 계산한 전역
`X`, `Y` 좌표를 받아 TurtleBot3 베이스를 목표점까지 저속으로 이동시키는
`ament_python` 패키지입니다. UWB 위치에 `/odom` yaw를 결합하고, 센서
freshness, enable, safety stop을 확인하면서 짧게 움직인 뒤 새 UWB 표본을
기다리는 stop-and-go 제어를 사용합니다.

구현의 기준은
[Turtlebot_UWB](https://github.com/Jeong-Yun-Kim/Turtlebot_UWB)의 최신 `main`
브랜치에 있는 `turtlebot_uwb_delivery` 패키지입니다. 검증된 좌표 파서,
시리얼 재연결, covariance, stop-and-go 제어, 도착 확인, enable/safety
interlock 및 자동 배송 흐름을 팀 프로젝트 인터페이스에 맞춰 이식했습니다.

이 패키지의 책임은 위치 수신·검증, 현재 전역 pose 발행, 목표점 이동, 도착
판정 및 자동/수동 이동 상태 관리입니다. 주문 앱, ArUco 인식, 매니퓰레이터와
RL 제어, 그리고 최종 속도 중재는 담당하지 않습니다.

## 패키지 구조

```text
turtlebot3_position/
├── config/position.yaml
├── launch/position.launch.py
├── resource/turtlebot3_position
├── turtlebot3_position/
│   ├── __init__.py
│   ├── core.py
│   ├── uwb_serial_node.py
│   ├── position_controller_node.py
│   └── goal_console.py
├── test/
│   ├── test_core.py
│   ├── test_topic_contract.py
│   ├── test_modes.py
│   └── test_colcon.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 노드 역할

| 노드 | 역할 |
|---|---|
| `uwb_serial_node` | pyserial 연결·재연결, UTF-8 입력 처리, `X`, `Y` 파싱, raw/valid 진단, `/odom` yaw freshness 확인, covariance가 포함된 전역 pose 발행 |
| `position_controller_node` | 자동 배송과 수동 목표를 같은 stop-and-go 제어기로 실행하고 enable, safety latch, UWB/odom timeout, 도착 확인, 상태·이벤트·mux용 속도 요청을 관리 |
| `goal_console` | YAML waypoint 이름 또는 직접 좌표를 `PoseStamped` 목표로 변환하고 enable/disable을 발행하는 대화형 수동 시험 도구 |

기본 launch에는 `uwb_serial_node`와 `position_controller_node`만 포함됩니다.
`goal_console`은 별도 터미널에서 필요할 때만 실행합니다.

## 토픽 계약

모든 주요 토픽 이름은 `config/position.yaml`의 ROS 2 parameter로 변경할 수
있습니다.

### 입력

| 토픽 | 타입 | 소비 노드 | 용도 |
|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | serial, controller | 현재 yaw 및 odometry freshness |
| `/turtlebot3_position/goal` | `geometry_msgs/msg/PoseStamped` | controller | 수동 목표 x, y와 선택적 최종 yaw |
| `/turtlebot3_position/enable` | `std_msgs/msg/Bool` | controller | 이동 허용 또는 즉시 비활성화 |
| `/safety_stop` | `std_msgs/msg/Bool` | controller | 모든 이동 정지 및 재출발 latch |
| `/delivery/request` | `std_msgs/msg/String` | controller | 자동 배송 요청 `tower1`, `tower2`, `tower3` |

### 출력

| 토픽 | 타입 | 발행 노드 | 용도 |
|---|---|---|---|
| `/turtlebot3_position/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | serial | meter 단위 UWB 위치, odom yaw, covariance |
| `/turtlebot3_position/status` | `std_msgs/msg/String` | controller | 제어 상태와 상세 원인 |
| `/turtlebot3_position/uwb/valid` | `std_msgs/msg/Bool` | serial | 최근 시리얼 줄의 좌표 유효성 |
| `/turtlebot3_position/uwb/raw` | `std_msgs/msg/String` | serial | 수신한 진단용 원문 |
| `/turtlebot3_control/nav_cmd_vel` | `geometry_msgs/msg/Twist` | controller | `cmd_vel_mux_node`에 전달할 주행 속도 요청 |
| `/turtlebot3_control/base_arrived` | `std_msgs/msg/Bool` | controller | 현재 목표의 안정적 도착 여부, 10 Hz 연속 발행 |
| `/delivery/event` | `std_msgs/msg/String` | controller | 자동 배송 전이, 거부, 완료, 취소 이벤트 |

### `/cmd_vel` 소유권

이 패키지는 실제 `/cmd_vel` publisher를 만들거나 `/cmd_vel` remapping을 하지
않습니다. 주행 출력은 `/turtlebot3_control/nav_cmd_vel`에만 발행합니다. 최종
`/cmd_vel`은 `turtlebot3_control`의 mux 하나가 NAV/PICK/HOLD 상태와 명령
freshness를 함께 판단해 발행해야 합니다. 이렇게 해야 이동 제어와 파지 제어가
동시에 베이스를 구동하지 않습니다.

## 동작 모드

| 구분 | 입력 | 동작 |
|---|---|---|
| 자동 배송 | `/delivery/request`에 `tower1` | pickup → 대기 → tower1 → 대기 → safe |
| 수동 이동 | `goal_console`에서 `tower1` | 현재 위치 → tower1 → `ARRIVED` |
| 수동 직접 좌표 | `goal_console`에서 `x y [yaw_deg]` | 현재 위치 → 해당 좌표 → `ARRIVED` |

### 자동 배송

자동 배송은 `/delivery/request`로만 시작합니다. 허용되는 요청 문자열은
`tower1`, `tower2`, `tower3`입니다. `goal_console`에서 같은 타워 이름을
입력해도 자동 배송을 시작하지 않습니다.

```text
IDLE 또는 SAFE
  → TO_PICKUP
  → WAIT_PICKUP (pickup_wait_sec)
  → TO_TOWER
  → WAIT_DELIVERY (delivery_wait_sec)
  → TO_SAFE
  → SAFE
```

요청은 `IDLE` 또는 `SAFE`에서만 받아들입니다. `TO_PICKUP`, `WAIT_PICKUP`,
`TO_TOWER`, `WAIT_DELIVERY`, `TO_SAFE`, `MANUAL`, `ESTOP`에서는 새 자동
요청을 거부합니다. 안전 정지 중에도 요청을 시작하지 않습니다.

컨트롤러 enable과 센서가 유효한 상태에서 다음처럼 요청할 수 있습니다.

```bash
ros2 topic pub --once /turtlebot3_position/enable \
  std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /delivery/request \
  std_msgs/msg/String \
  "{data: 'tower1'}"
```

### 수동 waypoint와 직접 좌표

수동 콘솔은 설치된 패키지 share 디렉터리의
`config/position.yaml`을 기본으로 읽습니다. 따라서 별도의 `--params-file`
인자 없이 다음 명령만 실행해도 controller와 동일한 초기 waypoint를
사용합니다.

```bash
ros2 run turtlebot3_position goal_console
```

지원 입력은 다음과 같습니다.

```text
UWB goal> tower1
UWB goal> tower2
UWB goal> tower3
UWB goal> pickup
UWB goal> safe
UWB goal> safezone
UWB goal> 1
UWB goal> 2
UWB goal> 3
UWB goal> 0.80 0.93
UWB goal> 0.80 0.93 90
UWB goal> disable
UWB goal> stop
UWB goal> quit
```

`1`, `2`, `3`은 각각 `tower1`, `tower2`, `tower3`의 alias이고,
`safezone`은 `safe`의 alias입니다. `x y`의 단위는 meter, 세 번째
`yaw_deg`는 degree입니다. yaw를 생략하면 최종 방향 요청이 없는 all-zero
quaternion을 발행합니다. `use_goal_yaw: true`일 때만 지정한 최종 yaw 정렬을
수행하며 기본값은 `false`입니다.

`auto_enable: true`이면 콘솔이 목표를 발행한 직후 `enable=true`도
발행합니다. `disable` 또는 `stop`은 `enable=false`를 발행해 자동 배송과
수동 이동을 모두 즉시 중단합니다. `quit` 또는 `exit`는 콘솔을 정상
종료합니다.

수동 goal을 받으면 phase는 `MANUAL`이고 기존 `base_arrived`는 즉시 false가
됩니다. 해당 한 지점에 도착하면 `ARRIVED`로 끝나며 다음 waypoint로 자동
전환하지 않습니다. 따라서 수동 `pickup`은 tower로 이어지지 않고, 수동
`tower2`는 safe로 이어지지 않습니다.

자동 배송 중 수동 goal이 들어오면 0 Twist를 즉시 요청하고 자동 임무의
요청·wait·motion·settle·도착 확인 상태를 초기화한 뒤 `MANUAL`을 우선합니다.
이때 `/delivery/event`에 `mission_cancelled:manual_goal` 이벤트를 발행합니다.
반대로 수동 이동 중의 새 자동 배송 요청은 거부합니다.

## 주요 parameter

### UWB serial과 pose

| parameter | 기본값 | 의미 |
|---|---:|---|
| `port` | `/dev/ttyUSB0` | ESP32/UWB 시리얼 장치 |
| `baud` | `115200` | 시리얼 baud rate |
| `reconnect_sec` | `2.0` | 연결 실패 후 재시도 간격 |
| `frame_id` | `uwb_map` | 전역 UWB 좌표 frame |
| `include_odom_yaw` | `true` | pose orientation에 fresh odom yaw 포함 |
| `odom_timeout_sec` | `1.0` | serial 노드가 yaw를 fresh로 판단하는 시간 |
| `position_variance_x/y` | `0.0121` | pose x/y covariance 대각 원소 |
| `yaw_variance` | `0.04` | fresh yaw covariance |
| `yaw_unavailable_variance` | `9.8696` | yaw를 사용할 수 없을 때 covariance |

입력 줄은 `X=0.875, Y=0.565`와 anchor 거리 뒤에 같은 좌표가 붙는 형식을
지원합니다. 좌표 단위는 meter입니다. `?`, NaN, Inf 또는 숫자가 아닌 값은
pose로 발행하지 않습니다. 장치 연결 실패나 UTF-8 decode 오류가 발생해도
노드는 종료하지 않고 안전하게 진단을 발행한 뒤 재연결 또는 다음 입력을
기다립니다. 장치가 없을 때 가짜 pose를 만들지 않습니다.

### 위치 제어와 안전

| parameter | 기본값 | 의미 |
|---|---:|---|
| `arrival_tolerance` | `0.13` | 목표 도착 거리 허용오차, meter |
| `arrival_confirmations` | `3` | 도착으로 확정할 연속 표본 수 |
| `linear_speed` | `0.055` | drive pulse 선속도, m/s |
| `angular_speed` | `0.28` | turn/final-align 각속도, rad/s |
| `heading_tolerance` | `0.30` | 전진을 허용하는 방향 오차, rad |
| `final_yaw_tolerance` | `0.20` | 선택적 최종 yaw 허용오차, rad |
| `drive_pulse_sec` | `0.45` | 짧은 전진 pulse 시간 |
| `turn_pulse_sec` | `0.35` | 짧은 회전 pulse 시간 |
| `settle_sec` | `1.6` | pulse 후 UWB 안정화 대기 시간 |
| `uwb_timeout_sec` | `2.0` | UWB pose freshness 제한 |
| `odom_timeout_sec` | `1.0` | controller odometry freshness 제한 |
| `pickup_wait_sec` | `2.0` | 자동 배송 pickup 도착 후 대기 |
| `delivery_wait_sec` | `2.0` | 자동 배송 tower 도착 후 대기 |
| `initial_yaw` | `0.0` | odom yaw를 쓰지 않을 때의 시작 yaw |
| `use_odom_yaw` | `true` | 방향 계산에 fresh odometry yaw 요구 |
| `use_goal_yaw` | `false` | 위치 도착 뒤 선택적 최종 yaw 정렬 |

상태 문자열의 기본 prefix는 `WAIT_SENSOR`, `IDLE`, `ROTATE_TO_GOAL`,
`DRIVE`, `FINAL_ALIGN`, `ARRIVED`, `FAULT`, `DISABLED`, `SAFETY_STOP`입니다.
예를 들어 `WAIT_SENSOR:NO_UWB`, `FAULT:ODOM_TIMEOUT`,
`DRIVE:DISTANCE=0.540`, `ARRIVED:MANUAL`처럼 세부 원인이 붙을 수 있습니다.

enable=false, 목표 없음, sensor timeout, safety stop, 도착 상태에서는 항상
0 Twist를 요청합니다. `use_odom_yaw=true`이면 UWB뿐 아니라 odometry가
timeout되어도 이동하지 않습니다. safety_stop=true는 자동·수동 이동을 즉시
중단하고 `base_arrived=false`로 만들며 latch를 설정합니다. safety_stop=false만
발행해서는 다시 움직이지 않고, 해제 후 새 `enable=true`가 필요합니다.

### waypoint 초기 좌표

| 이름 | x (m) | y (m) |
|---|---:|---:|
| `pickup` | 0.80 | 0.93 |
| `tower1` | 0.20 | 0.20 |
| `tower2` | 1.55 | 0.93 |
| `tower3` | 1.55 | 0.90 |
| `safe` | 0.80 | 0.20 |

이 값은 참고 저장소에서 가져온 **초기 시험값**이며 실기 최종 좌표가
아닙니다. 좌표 단위는 meter이고 UWB 원점, 축 방향, anchor 배치와 실제 설치
환경에 맞게 반드시 교정해야 합니다. `position.yaml`의
`position_controller_node`와 `goal_console` waypoint 값을 항상 동일하게
유지하십시오. Python 코드에는 waypoint를 별도로 하드코딩하지 않습니다.

## 빌드와 실행

ROS 2 Humble 환경에서 새 패키지만 빌드합니다.

```bash
cd /home/kjy/omx_turtle_local_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select turtlebot3_position
source /home/kjy/omx_turtle_local_ws/install/setup.bash
```

기본 노드 실행:

```bash
ros2 launch turtlebot3_position position.launch.py
```

기본 launch는 설치된 `config/position.yaml`을 serial과 controller 두 노드에
전달합니다. `goal_console`은 포함하지 않으며 `/cmd_vel` remapping도 없습니다.

하드웨어 없이 core·계약 테스트만 실행할 수 있습니다.

```bash
cd /home/kjy/omx_turtle_local_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select turtlebot3_position
colcon test-result --verbose
```

## 실제 장치 시험 전 주의사항

- waypoint와 UWB 좌표계를 현장에서 교정하기 전에는 로봇을 enable하지
  마십시오.
- 첫 실기 시험은 바퀴를 지면에서 띄우거나 충분한 안전 공간을 확보하고,
  저속 설정과 즉시 사용할 수 있는 safety stop을 확인한 뒤 수행하십시오.
- 이 제어기는 장애물 회피를 제공하지 않으므로 이동 경로에서 사람과 물체를
  제거해야 합니다.
- `/odom` yaw 방향과 UWB 좌표축이 일치하는지 확인하십시오.
- 시리얼 장치가 없으면 노드는 재연결을 기다리고 pose를 발행하지 않습니다.
  controller는 센서 대기 또는 fault 상태에서 0 Twist를 유지합니다.
- 테스트 스위트는 실제 모터를 구동하지 않습니다. 실제 UWB 정확도, odometry
  drift, pulse별 이동량, settle 시간과 반복 도착 오차는 별도의 안전한 실기
  검증이 필요합니다.
