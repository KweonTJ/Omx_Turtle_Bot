# GPU Gazebo server 50회 A/B 결과

> Reference 군은 PPO residual을 0으로 고정한 대리군이며 실제 C++ `mp_control` 실행이 아니다.

| 지표 | Reference only | PPO residual |
|---|---:|---:|
| Cycle 성공 | 27/50 (54.0%) | 28/50 (56.0%) |
| 상태 완료 | 37/50 | 45/50 |
| 완료 후 물리 실패 | 10 | 17 |
| 성공 cycle 평균 | 19.615 s | 20.202 s |
| 성공 cycle 중앙값 | 18.831 s | 19.541 s |
| 성공 cycle p95 | 24.470 s | 25.527 s |
| 완료 XY 오차 중앙값 | 15.9 mm | 18.5 mm |
| GPU active 평균 | 15.4% | 6.6% |
| GPU 최대 | 64% | 35% |
| GPU 최대 메모리 | 520 MiB | 520 MiB |

## Paired 결과

- 두 제어기 모두 성공: 15/50
- Reference만 성공: 12/50
- RL만 성공: 13/50
- 둘 다 실패: 10/50
- 공통 성공에서 RL-reference 평균 시간 차이: +0.691 s
- 공통 성공에서 RL이 빠른 회차: 5/15

## 조건

- 회차마다 Gazebo server, ros2_control, PPO node를 새로 시작했다.
- Gazebo server는 `-s --headless-rendering`, OGRE2 Sensors, NVIDIA PRIME offload를 사용했다.
- PPO 추론은 RTX 4050 `cuda:0`에서 수행했다.
- ODE 물리와 ROS 2 executor는 Gazebo 구조상 CPU에서 수행된다.
- 물리 성공 허용치는 타워 중심 XY 25 mm, 상자 중심 Z 12 mm다.
