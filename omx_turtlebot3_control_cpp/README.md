# v6 C++ ONNX 제어

`actor.onnx` 원본을 OpenCV DNN CPU로 직접 실행한다. ONNX Runtime나 TorchScript로
자동 대체하지 않는다. 입력 69개와 출력 4개는 그대로이며 원본 모델 파일은 수정하지 않는다.

`control.cpp`는 체크섬·URDF 검증, 추론, 단계별 기준 경로와 residual 결합, 관절 속도·가속도
제한, 회전 중 joint2·3·4 유지, 그리퍼 속도 제한을 담당한다. `opencr.cpp`는 실제 엔코더·
전류·전압·바퀴 정지 검사, 관절 동시 전송과 시간 초과 정지를 담당한다. Python은 ROS 영상,
ArUco, FK/IK·충돌 검사와 단계 전환·집기/안착 증거 판정을 담당하며 C ABI로 호출한다.
`runtime_cpp`에서는 Python의 모터 통신·추론·보정 계산을 호출하지 않는다.

참고 리포: [mp_control](https://github.com/KweonTJ/3D_pereception_Based_Mobile_Manipulator-Autonomous_Navigation_System/tree/795d0ee0cac048b4289238cb4bdc16ed6a32fd35/mp_control).
같은 관절 피드백으로 joint3 사전 반전과 재반전을 수행하고 최종 joint4를 다시 보정하지 않는다.
네 관절을 OpenCR의 200번 묶음 레지스터와 220번 commit으로 함께 전송한다. 이는 원본의
controller 좌표 계약을 프로세스 내부에서 구현한 **직접 OpenCR 경로**다. ROS
`/arm_controller/joint_trajectory_raw` transformer나 `arm_controller`에 publish하지 않는다.
`ros2_control`, 기존 mp_control, Python OpenCR 런타임과 동시에 직렬 장치를 열지 않는다.

```bash
cd /home/ktj/omx_turtle_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to omx_turtlebot3_runtime --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# 하드웨어를 열지 않는 원본 973개 벡터 ONNX 검사
ros2 run omx_turtlebot3_control_cpp onnx_probe_cpp \
  src/omx_turtlebot3_policy/models/omx_turtlebot3_v6
# 인지 의존성이 있는 Python 환경으로 진입점 확인 (하드웨어 불필요)
src/omx_turtlebot3_control_cpp/tools/run_cpp_runtime.sh --help
# 로봇 배포 후 읽기 전용 (모터 쓰기 0회)
src/omx_turtlebot3_control_cpp/tools/run_cpp_runtime.sh --duration 10 --log-dir /tmp/omx_cpp_unique
```

읽기 전용도 실제 OpenCR 장치가 필요하다. 로컬 테스트는 가상 Bus만 사용한다. 원본 런타임과
동일하게 MuJoCo·SciPy·ArUco 등 인지·기하 의존성을 설치해야 한다. Jetson은 시스템 ROS
의존성을 포함한 전용 Python 환경을 `OMX_CPP_PYTHON`으로 지정한다. 로컬 기본값은 워크스페이스의
`.venv_omx_cpp/bin/python`이다. ROS Humble의 Python 3.10, NumPy 1.x, SciPy, MuJoCo 3.10.0,
OpenCV ArUco가 필요하다. 실행 스크립트는 사용자 전역 패키지 혼입을 차단한다. GPU는 사용하지 않는다.

전체 구동은 별도의 검증된 `--config`와 `--execute`, 실제 Stay, 신선한 카메라·관절 피드백,
`/omx_v6/request`와 지속적인 `/omx_v6/deadman`을 요구한다. 기본 보정 플래그·전류 임계값을
수정하지 않는다. C++도 보정 조건을 독립 검사하며 읽기 전용 모드에서는 모터 레지스터 쓰기를
거부한다. native transport의 10ms 주기 감시 스레드는 마지막 명령으로부터 `max_cycle_s`가
지나면 최신 관절 위치를 유지하고 재구동을 잠근다. 그리퍼 목표를 바꾸거나 토크를 끄지 않는다.
프로세스 종료·장치 단절 때의 물리 정지는 현장 검증 대상이다.

ROS, 기구학과 단계 관리는 Python에 남아 있다. 이것을 전체 임무의 완전한 C++ 이식이나
실제 집기 성공으로 해석하지 않는다. 시험 결과와 로봇용 소스 묶음 생성·배포 방법은
[로컬 검증 및 배포 준비](docs/local_validation.md)에 기록한다.
