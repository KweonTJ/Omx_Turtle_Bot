# OMX TurtleBot Delivery Robot

TurtleBot3 Waffle Pi와 OpenMANIPULATOR-X를 사용하는 ROS 2 Humble 실제 로봇
워크스페이스다. EEF 카메라의 ArUco 자세를 `base_link`로 변환하고, 학습된
PPO residual 정책과 결정론적 기준 경로로 물체를 파지한다.

현재 매니퓰레이터 구현 범위는 **ArUco 인식 → PICK → Stay 운반 자세 →
외부 베이스 이동 → PLACE → Stay 복귀**다. 최신
`arm_delivery_pitch_hold_v4`는 PICK과 PLACE를 함께 학습했다. 베이스
목표 생성과 이동은 외부 navigation 패키지의 책임이며, 이 저장소의
coordinator는 출발·도착 신호를 받아 두 팔 동작을 연결한다.

## 현재 구성

```mermaid
flowchart LR
    CAMERA[EEF USB camera] --> VISION[omx_eef_vision]
    VISION -->|object pose / valid| CONTROL{RL runtime}
    PY[omx_rl_control Python] --> CONTROL
    CPP[omx_rl_control_cpp C++] --> CONTROL
    JOINT[joint_states] --> CONTROL
    CONTROL -->|joint trajectory| ARM[arm_controller]
    CONTROL -->|GripperCommand| GRIPPER[gripper_controller]
    CONTROL -->|base_hold| MUX[turtlebot3_control mux]
```

Python과 C++ 런타임은 선택 관계이며 동시에 실행하지 않는다.

| 패키지 | 역할 | 상태 |
|---|---|---|
| [`omx_eef_vision`](./omx_eef_vision/) | ArUco 검출, 품질 판정, TF, 목표 자세 | 구현 |
| [`omx_rl_control`](./omx_rl_control/) | SB3/PyTorch 기반 Python PPO 런타임 | 구현 |
| [`omx_rl_control_cpp`](./omx_rl_control_cpp/) | TorchScript CUDA/CPU와 Eigen fallback 기반 C++ PPO actor | 구현, 실기 검증 전 |
| [`turtlebot3_control`](./turtlebot3_control/) | 통합 launch, 베이스 mux, 배송 coordinator | 구현 |
| `turtlebot3_manipulation_*` | 하드웨어, ros2_control, URDF, 카메라 TF | upstream 기반 확장 |
| `turtlebot3_position` | UWB 위치·이동 제어 | 패키지 미구현 |

전체 24개 ROS 패키지 중 위 다섯 영역이 프로젝트의 직접 변경 범위이며,
DynamixelSDK와 TurtleBot3 기본 패키지는 기반 의존성이다.

## 기본 모델

| 항목 | 값 |
|---|---|
| 모델 | `arm_delivery_pitch_hold_v4` |
| 관측/행동 | 33 / 4 |
| 행동 | reference + PPO residual 10% |
| 제어 주기 | 50 Hz |
| 자세 유지 | `joint2 + joint3 + joint4 ≈ 0` |
| 지원 명령 | `PICK`, `PLACE` |
| MuJoCo delivery full-tower | 100/100 성공, 충돌 0 |
| MuJoCo delivery sim-to-real robust | 98/100 성공, 충돌 2 |

평가 수치는 MuJoCo 결과이며 실제 로봇 반복 성공률을 의미하지 않는다.

## 빌드

```bash
cd /home/ktj/omx_turtle_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Python 런타임은 Stable-Baselines3와 PyTorch가 설치된 로봇 환경이 필요하다.
C++ 런타임은 LibTorch가 있으면 TorchScript actor를 CUDA 또는 CPU에서
실행하고, LibTorch가 없는 빌드에서는 Eigen CPU actor로 fallback한다.

## 통합 실행

기존 기본값인 Python 런타임:

```bash
ros2 launch turtlebot3_control omx_turtle.launch.py \
  control_runtime:=python
```

C++ 런타임:

```bash
ros2 launch turtlebot3_control omx_turtle.launch.py \
  control_runtime:=cpp \
  cpp_policy_backend:=auto cpp_policy_device:=auto
```

통합 launch 기본값은 실제 하드웨어, EEF 카메라, Vision, RL, mux와
coordinator를 시작한다. 또한 기존 동작대로 bringup에
`move_to_stay_pose:=true`를 전달하므로 실제 로봇에서는 주변과 E-Stop을
확인한 뒤 실행한다. 시작 자세 동작을 변경하는 코드는 이번 작업에 포함하지
않았다.

매니퓰레이터만 점검할 때는 베이스 통합 노드를 끈다.

```bash
ros2 launch turtlebot3_control omx_turtle.launch.py \
  control_runtime:=cpp \
  start_mux:=false start_coordinator:=false \
  require_base_arrived:=false require_odom_stopped:=false
```

## 상태 확인

```bash
ros2 topic echo /target/aruco_status
ros2 topic echo /target/object_pose --once
ros2 topic echo /target/valid
ros2 topic echo /rl_control/status
ros2 control list_controllers
```

`PICK`이나 `valid=true`만으로 정상 동작을 판단하지 않는다. object pose와
ArUco freshness, joint state, base gate, controller와 최종
`/rl_control/status`를 함께 확인한다.

## 문서

개발 순서와 최신 분석은 [docs/README.md](./docs/README.md)에서 시작한다.

- [Python 매니퓰레이터 제어 코드 분석](<./docs/7. Python 매니퓰레이터 제어 코드 분석.md>)
- [C++ 매니퓰레이터 제어 구현 및 검증](<./docs/8. C++ 매니퓰레이터 제어 구현 및 검증.md>)
- [변경 이력](<./docs/9. 변경 이력.md>)
- [배송 정책 v4 실제 로봇 배포](<./docs/10. 배송 정책 v4 실제 로봇 배포.md>)
- [C++ PPO 로봇 제어 코드가 긴 이유](<./docs/11. C++ PPO 로봇 제어 코드가 긴 이유.md>)

## 안전 경계

- 실제 팔은 fake hardware와 모델 checksum·상태 출력을 먼저 확인한다.
- pose, joint, odom 또는 inference timeout 시 계속 명령하지 않는다.
- `/safety_stop=true`에서는 gripper goal을 취소하고 현재 관절 hold를 보낸다.
- 마커 크기, 카메라 보정, EEF 카메라 TF를 실측 없이 추정하지 않는다.
- Python/C++ RL 노드를 동시에 실행해 trajectory publisher를 중복시키지 않는다.
