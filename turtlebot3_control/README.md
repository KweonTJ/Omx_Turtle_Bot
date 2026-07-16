# turtlebot3_control

`turtlebot3_control`은 리더 로봇의 위치 이동과 ArUco 기반 픽앤플레이스를 한 흐름으로 묶기 위한 통합 패키지다.

기존 두 흐름은 그대로 유지하고, 이 패키지에서 제어권을 정리한다.

- `leader_line_follower`: 타 호스트 PC가 준 목표 위치로 리더 로봇 이동
- `mp_control` + `aruco_eef_tracker` + `aruco_mp_bridge`: ArUco 인식 후 매니퓰레이터 파지
- `turtlebot3_control`: 주행 명령과 파지 명령이 동시에 `/cmd_vel`을 건드리지 않도록 mux/coordinator로 제어

## 전체 동작 흐름

1. `aruco_pick_place.launch.py`를 통해 하드웨어 브링업, EEF 카메라, ArUco tracker, MoveIt Servo, `mp_control_node`를 실행한다.
2. `leader_rover_nav_node`가 타 호스트 PC의 `/leader/target_cmd`와 `/leader/global/position`을 받아 `/leader/cmd_vel`을 만든다.
3. `cmd_vel_mux_node`가 주행 중에는 `/leader/cmd_vel`을 실제 `/cmd_vel`로 전달한다.
4. 리더가 목표 위치에 도착하면 `leader_pick_coordinator_node`가 `/leader/nav_feedback`의 `ARRIVED` 또는 `HOLDING` 상태를 확인한다.
5. ArUco marker가 보이면 coordinator가 mux 모드를 `PICK`으로 바꾸고 `/mp_control/start`를 발행한다.
6. `aruco_to_mp_control_bridge`는 `/target/aruco_pose`를 `/target/object_in_base`와 `/target/close_range_ready`로 변환해서 `mp_control_node`에 제공한다.
7. `mp_control_node`가 매니퓰레이터 파지와 handoff를 수행한다.
8. 완료 또는 에러 상태가 감지되면 mux는 `HOLD` 모드로 전환되어 베이스를 정지시킨다.

## 핵심 노드

### cmd_vel_mux_node

실제 `/cmd_vel`을 단일 노드가 발행하도록 만드는 mux 노드다.

입력:

- `/leader/cmd_vel`: rover 주행 명령
- `/turtlebot3_control/pick_cmd_vel`: `mp_control`의 파지 중 베이스 정지/보정 명령
- `/turtlebot3_control/mux_mode`: `NAV`, `PICK`, `HOLD`, `STOP`
- `/target/base_hold`: true일 때 항상 정지 명령 우선

출력:

- `/cmd_vel`: 실제 로봇 베이스 명령
- `/turtlebot3_control/mux_status`: mux 상태 문자열

모드:

- `NAV`: `/leader/cmd_vel`을 `/cmd_vel`로 전달
- `PICK`: `/turtlebot3_control/pick_cmd_vel`을 `/cmd_vel`로 전달
- `HOLD`: 정지
- `STOP`: 정지

### leader_pick_coordinator_node

주행 완료 후에만 파지를 시작하도록 제어하는 노드다.

입력:

- `/leader/nav_feedback`: rover 도착 상태 확인
- `/target/aruco_visible`: ArUco marker 인식 여부
- `/mp_control/status`: 파지 완료/에러 상태 확인

출력:

- `/turtlebot3_control/mux_mode`: mux 모드 전환
- `/mp_control/start`: 파지 시작 신호
- `/turtlebot3_control/coordinator_status`: coordinator 상태 문자열

기본 조건:

- `/leader/nav_feedback.state`가 `ARRIVED` 또는 `HOLDING`
- `/target/aruco_visible`이 fresh 상태
- 두 조건이 만족되면 `/mp_control/start` 발행

## 통합 런치

메인 통합 런치:

```bash
ros2 launch turtlebot3_control leader_rover_aruco_pick_place.launch.py
```

이 런치는 다음 구성을 함께 실행한다.

- `mp_control/launch/aruco_pick_place.launch.py`
- `leader_line_follower/launch/leader_rover.launch.py`
- `aruco_mp_bridge`의 `aruco_to_mp_control_bridge`
- `turtlebot3_control`의 `cmd_vel_mux_node`
- `turtlebot3_control`의 `leader_pick_coordinator_node`

주의할 점:

- 하드웨어 브링업은 `mp_control/launch/aruco_pick_place.launch.py`를 통해 같이 실행된다.
- ArUco bridge의 자동 `/mp_control/start`는 꺼져 있다.
- 파지 시작은 coordinator가 rover 도착 후에만 수행한다.
- `mp_control`의 `base_cmd_vel_topic`은 `/cmd_vel`이 아니라 `/turtlebot3_control/pick_cmd_vel`로 분리되어 있다.

## 실행 스크립트

최종 실행 스크립트:

```bash
~/turtlebot3_ws/src/turtlebot3_control/scripts/run_leader_rover_aruco_pick_place.sh
```

내부 동작:

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_control leader_rover_aruco_pick_place.launch.py
```

런치 인자를 넘길 수도 있다.

```bash
~/turtlebot3_ws/src/turtlebot3_control/scripts/run_leader_rover_aruco_pick_place.sh force_object_x_m:=0.29
```

## 주요 설정 파일

- `config/cmd_vel_mux.yaml`
  - mux 입력/출력 토픽과 timeout 설정
- `config/leader_pick_coordinator.yaml`
  - rover 도착 조건, ArUco visible timeout, `/mp_control/start` 발행 횟수 설정
- `config/mp_control_aruco_integrated_params.yaml`
  - 통합 런치 전용 `mp_control` 설정
  - `base_cmd_vel_topic: /turtlebot3_control/pick_cmd_vel`
- `config/aruco_to_mp_control_bridge_integrated.yaml`
  - 통합 런치 전용 ArUco bridge 설정
  - `publish_start_on_visible: false`

## 상태 확인 명령

현재 mux 상태:

```bash
ros2 topic echo /turtlebot3_control/mux_status --once
```

현재 coordinator 상태:

```bash
ros2 topic echo /turtlebot3_control/coordinator_status --once
```

rover 도착 상태:

```bash
ros2 topic echo /leader/nav_feedback --once
```

ArUco 인식 상태:

```bash
ros2 topic echo /target/aruco_visible --once
ros2 topic echo /target/aruco_pose --once
```

`mp_control` 상태:

```bash
ros2 topic echo /mp_control/status --once
```

실제 베이스 명령:

```bash
ros2 topic echo /cmd_vel --once
```

주행 입력 명령:

```bash
ros2 topic echo /leader/cmd_vel --once
```

파지 쪽 베이스 명령:

```bash
ros2 topic echo /turtlebot3_control/pick_cmd_vel --once
```

## 정상 동작 기준

주행 중:

- `/turtlebot3_control/mux_status`에 `mode=NAV`
- `/leader/cmd_vel`이 `/cmd_vel`로 전달됨
- `/mp_control/start`는 아직 발행되지 않음

목표 도착 후:

- `/leader/nav_feedback.state`가 `ARRIVED` 또는 `HOLDING`
- `/target/aruco_visible`이 true
- coordinator가 `/turtlebot3_control/mux_mode`를 `PICK`으로 변경
- coordinator가 `/mp_control/start` 발행

파지 중:

- `/turtlebot3_control/mux_status`에 `mode=PICK`
- `/target/base_hold=true`이면 베이스는 항상 정지
- `mp_control_node`가 매니퓰레이터와 그리퍼를 제어

완료 또는 에러:

- coordinator가 `DONE` 또는 `ERROR` phase로 전환
- mux는 `HOLD` 모드로 전환
- `/cmd_vel`은 0으로 유지
