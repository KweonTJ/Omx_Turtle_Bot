# v6 ONNX C++ 로컬 검증 및 배포 준비

2026-11-21, 호스트 `/home/ktj/omx_turtle_ws`에서 작업했다. 사용자의 요청에 따라
로봇 배포·실물 구동을 수행하지 않았다. 기준 체크아웃은 `Omx_Turtle_Bot`의
`d5c5dc0`, 참조 리포는 `3D_pereception_Based_Mobile_Manipulator-Autonomous_Navigation_System`의
[`795d0ee0cac048b4289238cb4bdc16ed6a32fd35`](https://github.com/KweonTJ/3D_pereception_Based_Mobile_Manipulator-Autonomous_Navigation_System/tree/795d0ee0cac048b4289238cb4bdc16ed6a32fd35)다.

## 적용한 좌표·모델 계약

- ONNX 입력은 69차원 관측이고 출력은 4차원 관절 보정량이다. 원본 `actor.onnx` SHA-256은
  `fe3656701f104022872ad804ac120496c4b31de8cf9e7c3a2f94250e4ceef00c`이며 변환하거나 재학습하지 않았다.
- 학습 기준 경로에 0.12 residual을 더하고 실제 경과 시간으로 보정한다. 제어 상한은 기본
  0.12 rad/s, 0.3 rad/s², 그리퍼 0.006 m/s다. 오른쪽 회전 중 joint2·3·4는 고정한다.
- [참조 transformer](https://github.com/KweonTJ/3D_pereception_Based_Mobile_Manipulator-Autonomous_Navigation_System/blob/795d0ee0cac048b4289238cb4bdc16ed6a32fd35/mp_control/tools/joint_trajectory_transformer.py)와
  [실물 설정](https://github.com/KweonTJ/3D_pereception_Based_Mobile_Manipulator-Autonomous_Navigation_System/blob/795d0ee0cac048b4289238cb4bdc16ed6a32fd35/mp_control/config/mp_control_real_params.yaml)을
  대조했다. joint3는 동일 피드백을 기준으로 사전 반전·재반전하고 joint4를 추가 보정하지 않는다.
- 이 출력은 ROS trajectory topic 대신 OpenCR 묶음 레지스터로 전송한다. ROS controller를
  함께 실행하지 않는다. 읽기 전용은 모터 레지스터 쓰기 0회다.
- 현재 로봇 URDF 스냅샷의 SHA-256을 확인하고 관절 한계와 planner 한계의 교집합을 사용한다.
  학습·물리 관절 제한을 넘는 목표를 자동 영점 변경으로 통과시키지 않는다.

## 로컬 시험

| 시험 | 결과 |
|---|---|
| Release colcon 빌드 | C++·정책·인지·런타임 및 DynamixelSDK 5개 패키지 통과 |
| 원본 ONNX 973개 검증 벡터 | 최대 절대 오차 5.96066e-7, 허용 오차 1e-5 이내 |
| OpenCV DNN 4.10.0 / 4.5.4 | 두 버전 모두 빌드·973개 출력 비교 통과 |
| 호스트 CPU 추론 2,000회 | 평균 약 12 µs, Jetson 속도 측정 아님 |
| C++ gtest | 14개 통과 |
| Python 인지·기구학·임무·C ABI 통합 | 전용 환경에서 116개 통과, 기존 Torch 선택 시험 1개 제외 |
| 기존 Torch 비교 포함 전체 시험 | 117개 통과 |
| 9단계 × 4 waypoint × 3 주기 | 108개 조건에서 Python 기준식과 C++ 보정·속도·그리퍼 결과 일치 |
| 실제 Dynamixel SDK → 가상 직렬 포트 | 2개 시험 통과: 프로토콜 CRC, 읽기, 묶음 목표, 피드백, 그리퍼 유지 정지 |
| 기본 실행·보정 누락 | 쓰기 차단, 보정 미완료 실행은 직렬 장치를 열기 전 거부 |
| Python 제어 경로 차단 시험 | native 모드가 Python residual/limiter를 호출하지 않고 충돌 검사는 유지함 |
| 독립 C++ watchdog | 제어 정체 뒤 유지·재시도 거부 시험 통과 |

가상 장치의 보정 true 값은 임시 테스트 설정에만 사용했다. 실제 `config/runtime.yaml`의
보정 플래그와 전류 임계값은 변경하지 않았다. 가상 장치 시험은 실제 전류·전압·통신 단절·
충돌·집기 성공을 검증하지 않는다.

로컬 Python 전역 NumPy 2.2.6은 시스템 SciPy 1.8과 ABI가 맞지 않아 그대로 실행할 수 없었다.
전역 환경을 수정하지 않고 `.venv_omx_cpp`에서 NumPy 1.21.5, SciPy 1.8.0, OpenCV 4.10.0,
MuJoCo 3.10.0을 사용했다. 네트워크 패키지 설치가 불가능하여 기존 MuJoCo/OpenGL 설치를
이 가상환경에서 참조한다. 이 호스트 가상환경은 배포 번들에 포함하지 않는다.

```bash
src/omx_turtlebot3_control_cpp/tools/run_local_checks.sh
src/omx_turtlebot3_control_cpp/tools/run_cpp_runtime.sh --help
python3 src/omx_turtlebot3_control_cpp/tools/make_robot_bundle.py --output-dir artifacts
```

## 추후 로봇 배포

번들은 C++/Python 소스·원본 정책·기구학 자산·ArUco·DynamixelSDK 및 파일별 크기·SHA-256
목록을 포함한다. 생성 직후 tar 내부 파일 목록·크기·해시를 모두 대조하며 외부 tar 체크섬도
별도 기록한다. x86 호스트 바이너리, 가상환경, 기존 로그는 포함하지 않는다.

로봇에서는 별도 워크스페이스에 압축을 풀고 ARM64로 다시 빌드한다. 예:

```bash
# 로봇에서 실행할 향후 절차. 이번 작업에서는 실행하지 않았다.
mkdir -p /home/a/omx_v6_cpp_ws
tar -xzf /전송한/omx_v6_cpp_source_고유값.tar.gz -C /home/a/omx_v6_cpp_ws
cd /home/a/omx_v6_cpp_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to omx_turtlebot3_runtime --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run omx_turtlebot3_control_cpp onnx_probe_cpp src/omx_turtlebot3_policy/models/omx_turtlebot3_v6
export OMX_CPP_PYTHON=/home/a/turtlebot3_ws/.venv_omx_runtime/bin/python
src/omx_turtlebot3_control_cpp/tools/run_cpp_runtime.sh --duration 10 --log-dir /tmp/omx_cpp_read_unique
```

기존 ROS·인지·네이티브 개발 의존성과 호환 Python 환경이 필요하다. 전체 실행은 현재 관절·
카메라·마커·실물 장면·접촉/안착 임계값을 확인한 별도 보정 설정을 사용하며, 기존 런타임의
요청·deadman·취소 계약을 유지한다. 이번 완료 범위는 로컬 구현과 소프트웨어 검증이다.
