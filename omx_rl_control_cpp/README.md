# omx_rl_control_cpp

`omx_rl_control` Python 런타임의 PICK/PLACE 경로를 C++17로 옮긴 ROS 2
패키지다. 현재 기본 모델 `arm_delivery_pitch_hold_v4`의 33차원 관측,
256×256 Tanh actor, 기준 경로, joint4 pitch 결합, 속도·가속도 제한과
그리퍼 상태기계를 같은 토픽 계약으로 제공한다.

## 구현 범위

| 항목 | Python | C++ |
|---|---|---|
| 33차원 관측 및 정규화 | 지원 | 지원 |
| deterministic PPO actor | SB3/PyTorch | TorchScript CUDA/CPU + Eigen fallback |
| 기준 waypoint와 numerical IK | 지원 | 지원 |
| PPO 10% residual·EMA·관절 제한 | 지원 | 지원 |
| joint1 물체 방위 정렬 | 지원 | 지원 |
| `joint2 + joint3 + joint4 = 0` pitch 결합 | 지원 | 지원 |
| 그리퍼 Action·timeout·E-Stop | 지원 | 지원 |
| PICK 후 Stay 복귀 | 지원 | 지원 |
| PLACE 후 release·Stay 복귀 | 지원 | 지원 |
| 외부 베이스 운반 중 Stay hold | 지원 | 지원 |

SB3 `policy.zip`은 C++에서 직접 읽지 않는다. 학습 PC에서 동일한 actor를
TorchScript `policy_actor.pt`와 Eigen fallback용 `policy_actor.bin`으로
내보낸다. `cpp_policy_backend=auto`는 LibTorch가 빌드에 포함되면 TorchScript를
우선 사용하고, `cpp_policy_device=auto`는 CUDA가 실제로 사용 가능하면
`cuda:0`, 아니면 CPU를 선택한다. LibTorch 자체가 없으면 Eigen CPU로
fallback한다.

CMake build type을 별도로 지정하지 않으면 이 패키지는 `Release(-O3)`로
빌드한다. `-ffast-math`와 `-march=native`는 정책 패리티와 로봇 간 이식성을
해칠 수 있어 사용하지 않는다. 실행 중에는 actor 추론 500회마다
`mean/p95/p99/max` 지연을 로그로 출력하며 `inference_timing_window`로 표본
크기를 조정할 수 있다.

## 빌드 및 실행

```bash
cd /home/ktj/omx_turtle_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select omx_rl_control omx_rl_control_cpp
source install/setup.bash

ros2 launch omx_rl_control_cpp rl_control_cpp.launch.py
```

CMake는 현재 `python3`에서 PyTorch의 LibTorch 경로를 자동 탐색한다. GPU를
강제 검증할 때는 다음과 같이 실행하며 CUDA가 없으면 시작을 실패시킨다.

```bash
ros2 launch omx_rl_control_cpp rl_control_cpp.launch.py \
  cpp_policy_backend:=torchscript cpp_policy_device:=cuda
```

통합 런치에서는 기본 Python을 유지하며 다음 인자로 C++을 선택한다.

```bash
ros2 launch turtlebot3_control omx_turtle.launch.py \
  control_runtime:=cpp
```

Python과 C++ 노드는 같은 이름과 출력 토픽을 사용하므로 동시에 실행하면
안 된다. 실제 하드웨어 실행 전에는 fake hardware에서 상태와 trajectory를
먼저 확인한다.

## 모델 재내보내기와 패리티

```bash
python3 omx_rl_control_cpp/tools/export_sb3_actor.py \
  --policy omx_rl_control/models/policies/arm_delivery_pitch_hold_v4/policy.zip \
  --output omx_rl_control_cpp/models/arm_delivery_pitch_hold_v4/policy_actor.bin \
  --torchscript-output \
    omx_rl_control_cpp/models/arm_delivery_pitch_hold_v4/policy_actor.pt \
  --metadata omx_rl_control_cpp/models/arm_delivery_pitch_hold_v4/policy_actor.yaml \
  --model-version arm_delivery_pitch_hold_v4
```

모델을 교체할 때는 C++ actor를 다시 내보내고 패리티 검사를 통과해야 한다.

```bash
python3 omx_rl_control_cpp/tools/compare_cpp_policy.py \
  --policy omx_rl_control/models/policies/arm_delivery_pitch_hold_v4/policy.zip \
  --actor omx_rl_control_cpp/models/arm_delivery_pitch_hold_v4/policy_actor.bin \
  --probe ../build/omx_rl_control_cpp/policy_probe
```

현재 v4 actor SHA-256은
`379a38e4866ba2d6f57942f61998ec74f685c50984fc3a0f6e005764987e70d4`다.
259개 관측 Python/C++ 패리티의 최대 절대 오차는
`1.78778711e-07`로 허용값 `2e-6` 이내다.

설계·차이점과 검증 결과는
[`docs/8. C++ 매니퓰레이터 제어 구현 및 검증.md`](<../docs/8. C++ 매니퓰레이터 제어 구현 및 검증.md>)에 기록한다.
