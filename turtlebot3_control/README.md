# turtlebot3_control

실제 TurtleBot3 하드웨어, EEF 카메라·ArUco Vision, 매니퓰레이터 RL 런타임,
베이스 속도 mux와 시작 coordinator를 묶는 프로젝트 통합 패키지다.

## 통합 흐름

1. `hardware.launch.py`가 TurtleBot3와 OpenMANIPULATOR-X controller를 시작한다.
2. EEF 카메라와 `omx_eef_vision`이 marker/object pose를 발행한다.
3. `control_runtime`에 따라 Python 또는 C++ RL 노드 하나를 8초 뒤 시작한다.
4. `cmd_vel_mux_node`는 navigation/PICK/HOLD 명령 중 실제 `/cmd_vel` 발행자를
   하나로 제한한다.
5. `omx_turtle_node`는 pickup arrival와 ArUco visibility가 fresh할 때
   `PICK`을 발행한다.
6. 파지 후 Stay 복귀가 끝나면 mux를 NAV로 돌려 외부 베이스 운반을 허용한다.
7. `base_arrived=false` 출발을 확인한 뒤 배송지의 fresh
   `base_arrived=true`를 받으면 `PLACE`를 발행한다.
8. release와 Stay 복귀가 끝난 `pick place complete`만 최종 DONE으로 본다.

## 런타임 선택

기본값은 기존 Python 구현이다.

```bash
ros2 launch turtlebot3_control omx_turtle.launch.py \
  control_runtime:=python
```

C++ 구현은 명시적으로 선택한다.

```bash
ros2 launch turtlebot3_control omx_turtle.launch.py \
  control_runtime:=cpp
```

허용값은 `python`, `cpp`뿐이다. 두 노드는 같은 이름
`rl_control_node`와 `/arm_controller/joint_trajectory`를 사용하므로 launch가
동시에 시작하지 않도록 조건을 분리한다.

## 주요 launch 인자

| 인자 | 기본값 | 의미 |
|---|---:|---|
| `control_runtime` | `python` | 매니퓰레이터 제어 구현 선택 |
| `start_hardware` | `true` | 실제 하드웨어 bringup |
| `start_rl_control` | `true` | 선택한 RL 노드 시작 |
| `start_eef_vision` | `true` | ArUco Vision 시작 |
| `start_eef_camera_driver` | `true` | `/dev/video0` EEF 카메라 시작 |
| `start_mux` | `true` | `/cmd_vel` mux 시작 |
| `start_coordinator` | `true` | PICK→운반→PLACE coordinator 시작 |
| `require_base_arrived` | `true` | 팔 동작 중 fresh 도착 신호 요구 |
| `require_odom_stopped` | `true` | 팔 동작 중 정지 odometry 요구 |
| `delivery_pose_required` | `false` | PLACE target pose freshness 요구 여부 |
| `start_camera` | `false` | Astra 카메라 비활성 |
| `start_lidar` | `false` | LiDAR 비활성 |

통합 launch는 기존 설정대로 `move_to_stay_pose:=true`를 hardware bringup에
전달하고, 카메라 장착 RPY를 `0.0 1.5708 0.0`으로 전달한다. 실제 팔이
움직일 수 있으므로 실행 전 E-Stop과 작업 공간을 확인한다.

매니퓰레이터만 확인할 때는 주행 관련 노드를 끌 수 있다.

```bash
ros2 launch turtlebot3_control omx_turtle.launch.py \
  control_runtime:=cpp \
  start_mux:=false start_coordinator:=false \
  require_base_arrived:=false require_odom_stopped:=false
```

## 핵심 노드

### cmd_vel_mux_node

- 입력: `/turtlebot3_control/nav_cmd_vel`,
  `/turtlebot3_control/pick_cmd_vel`, mux mode, `/target/base_hold`
- 출력: 실제 `/cmd_vel`
- `base_hold=true`이면 NAV/PICK보다 정지가 우선한다.
- 입력 timeout이면 마지막 속도를 재사용하지 않고 zero를 발행한다.

### omx_turtle_node

- pickup base arrival와 ArUco visible freshness를 검사한다.
- 조건을 통과하면 mux를 PICK으로 바꾸고 `/rl_control/command=PICK`을
  기본 3회 보낸다. `publish_compat_start=true`일 때만 구형 경로의
  `/mp_control/start=true`도 함께 발행한다.
- `grasp complete`는 최종 완료가 아니라 운반 시작 조건이다.
- 외부 navigation은 운반 시작 시 `base_arrived=false`, 배송지 정지 후
  `base_arrived=true`를 주기적으로 발행해야 한다.
- 배송지 재도착 후 `PLACE`를 보내고, `pick place complete`에서만
  DONE/HOLD로 간다.

## 상태 확인

```bash
ros2 topic echo /turtlebot3_control/mux_status
ros2 topic echo /turtlebot3_control/coordinator_status
ros2 topic echo /target/aruco_status
ros2 topic echo /rl_control/status
ros2 topic echo /cmd_vel
```

상세 제어 계약은
[`docs/7. Python 매니퓰레이터 제어 코드 분석.md`](<../docs/7. Python 매니퓰레이터 제어 코드 분석.md>)와
[`docs/8. C++ 매니퓰레이터 제어 구현 및 검증.md`](<../docs/8. C++ 매니퓰레이터 제어 구현 및 검증.md>)를 따른다.
