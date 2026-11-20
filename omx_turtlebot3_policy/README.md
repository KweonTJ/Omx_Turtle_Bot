# omx_turtlebot3 v6 추론 검증 패키지

최신 강화학습 정책을 Jetson에서 검사하는 패키지다. 모터 명령, ROS publisher,
직렬 통신, 자동 실행 런치는 포함하지 않는다. 추론 성공은 실물 픽앤플레이스 성공이 아니다.

2026년 11월 20일 로봇 C++ CPU·PyTorch CPU·CUDA 추론 검사 통과.
최종 모델의 새 시뮬레이션 검사는 10회 중 7회 성공, 3회 관측 만료 정지였다.
배포 내역과 전원 분리 방법은 [검사와 전원 진단](docs/validation_and_power.md)을 참고한다.

## 검사

```bash
source /opt/ros/humble/setup.bash
cd /home/a/turtlebot3_ws
colcon build --packages-select omx_turtlebot3_policy --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run omx_turtlebot3_policy policy_probe_cpp /home/a/turtlebot3_ws/src/omx_turtlebot3_policy/models/omx_turtlebot3_v6
ros2 run omx_turtlebot3_policy policy_probe.py --model-dir /home/a/turtlebot3_ws/src/omx_turtlebot3_policy/models/omx_turtlebot3_v6 --device cpu
# 전원 안정 상태에서 선택적으로 CUDA 추론 검사
ros2 run omx_turtlebot3_policy policy_probe.py --model-dir /home/a/turtlebot3_ws/src/omx_turtlebot3_policy/models/omx_turtlebot3_v6 --device cuda
ros2 run omx_turtlebot3_policy power_snapshot.py
```

Python 검사는 PyTorch가 필요하다. ONNX는 `--backend onnx`로 검사하며 해당 플랫폼의
ONNX Runtime 설치가 필요하다. 설치되지 않은 백엔드로 자동 대체하지 않는다.
모델 디렉터리의 SHA256SUMS를 검사한 뒤 추론한다. C++ 검사는 Eigen만 사용한다.

## 실물 제어 연결 전 조건

- 모델 입력 69개, 출력 4개 보정량. 구형 33개 입력 제어기와 호환되지 않는다.
- ONNX 출력은 목표 관절각이 아니다. 단계별 기준 경로에 제한된 보정량을 더해야 한다.
- 관절 피드백, FK, 실제 카메라 TF와 좌표계, ID 1·2의 40mm ArUco 관측을 연결해야 한다.
- 학습 입력의 양쪽 손가락 접촉과 팔로워 적재면 접촉은 MuJoCo 접촉값이다.
  실물 접촉 센서 또는 검증된 추정기의 동등성이 확보되기 전 임의로 참/거짓을 채우면 안 된다.
- 카메라 내·외부 보정, 경로 충돌 검사, stale 관측 정지, 높이 유지와 회전 종료 판정을 이식해야 한다.
- 모터 전압, 피크 전류, 통신 안정성 및 현장 비상 정지 준비를 확인해야 한다.

기존 로봇 패키지와 설정은 덮어쓰지 않는다. 이 패키지에는 실제 구동을 켜는 옵션이 없다.
