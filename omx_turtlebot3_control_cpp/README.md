# v6 C++ ONNX 제어

## 최종 실기 배포 상태

`ros_policy_pick_place.launch.py`의 **기존 ROS 하드웨어 + C++ ONNX relay 구성은
실제 로봇 배포와 정상 동작 확인을 완료했다.** 최종 시현 영상은
[메인 README](../README.md#최종-시현-영상)에 있다.
기록된 성공 run은 `lift_near_id0_20261123_0320`이며,
ID 0 집기 → 들어올리기 → 오른쪽 90° 회전 → 내려놓기 → 열기 → Stay를 자동 수행했다.
기본값은 `retract_mode:=lift`, `hold_joint4_during_extension:=false`이며 joint4를 연동한다.
기존 ROS 하드웨어 드라이버가 OpenCR을 소유하고 relay는 직렬 포트를 열지 않는다.
성공 설정과 재현 명령은 [실물 성공 기준](docs/robot_validation_20261123_lift.md)을 따른다.

아래 직접 OpenCR 런타임은 별도 실행 경로이며 자체 보정·시작 조건을 유지한다.

## 추론과 실행 경로

`actor.onnx` 원본을 OpenCV DNN CPU로 직접 실행한다. ONNX Runtime나 TorchScript로
자동 대체하지 않는다. 입력 69개와 출력 4개는 그대로이며 원본 모델 파일은 수정하지 않는다.

Jetson의 OpenCV 4.5.4에서 마지막 Clip이 음수 출력을 0으로 바꾸는 현상을 실측했다.
현재는 원본 ONNX의 `l3` 출력까지 실행하고 마지막 `[-1, 1]` Clip을 C++로 계산한다.
알려진 v6 ONNX 체크섬과 그래프를 확인하며, 초기화할 때 원본 검증 벡터 973개를 반드시
통과해야 한다. 모델·보정 계수·실행 백엔드를 다른 것으로 대체하지 않는다.

`control.cpp`는 체크섬·URDF 검증, 추론, 단계별 기준 경로와 residual 결합, 관절 속도·가속도
제한, 회전 중 joint2·3·4 유지, 그리퍼 속도 제한을 담당한다. `opencr.cpp`는 실제 엔코더·
전류·전압·바퀴 정지 검사, 관절 동시 전송과 시간 초과 정지를 담당한다. Python은 ROS 영상,
ArUco, FK/IK·충돌 검사와 단계 전환·집기/안착 증거 판정을 담당하며 C ABI로 호출한다.
`runtime_cpp`에서는 Python의 모터 통신·추론·보정 계산을 호출하지 않는다.

`stay.cpp`는 시작 시 네 관절을 함께 움직이는 최소 저크 경로와 엔코더 도달 판정을 담당한다.
`mp_control/launch/aruco_pick_place.launch.py`의 Stay 후 제어 시작 순서를 적용했다.
실행 모드는 현장 deadman → 전체 경로 검사 → 현재 위치/저속 프로파일 설정 → 필요한 경우
팔 토크 ON → Stay 이동 → 위치·속도 안정 확인 → 임무 요청 허용 순서다. 기존 런치의 기본
8초 대기 대신 피드백을 확인하며, 바퀴 토크나 바퀴 속도는 설정하지 않는다.

참고 리포: [mp_control](https://github.com/KweonTJ/3D_pereception_Based_Mobile_Manipulator-Autonomous_Navigation_System/tree/795d0ee0cac048b4289238cb4bdc16ed6a32fd35/mp_control).
같은 관절 피드백으로 joint3 사전 반전과 재반전을 수행하고 최종 joint4를 다시 보정하지 않는다.
네 관절을 OpenCR의 200번 묶음 레지스터와 220번 commit으로 함께 전송한다. 이는 원본의
controller 좌표 계약을 프로세스 내부에서 구현한 **직접 OpenCR 경로**다. ROS
`/arm_controller/joint_trajectory_raw` transformer나 `arm_controller`에 publish하지 않는다.
`ros2_control`, 기존 mp_control, Python OpenCR 런타임과 동시에 직렬 장치를 열지 않는다.

`ros_policy_relay_cpp`는 별도의 실물 mp_control 참조 보정 경로다. 이 모드는 직렬 포트를
열지 않고 기존 `ros2_control` 소유자를 유지한다. 원본 ONNX의 bounded residual을 기존
trajectory 세그먼트에 더하며 joint3를 한 번만 반전한다. `mp_control`의
`hold_joint4_during_extension`에 맞춰 raw 전개 명령의 joint4 residual을 0으로 만든다.
handoff 직접 명령의 joint4는 유지한다. 이것은 기존 9단계 IK 런타임과 별도이며 실물 보정
완료를 뜻하지 않는다. ROS 관측 어댑터는 실제 encoder/RGB만 읽고, 제공되지 않는 전류나
접촉을 측정값으로 꾸미지 않는다. 해당 시험에서 정책의 contact/deck bit는 false다.

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

전체 구동은 별도의 검증된 `--config`와 `--execute`, Stay 초기화 성공, 신선한 카메라·관절 피드백,
`/omx_v6/request`와 지속적인 `/omx_v6/deadman`을 요구한다. 기본 보정 플래그·전류 임계값을
수정하지 않는다. C++도 보정 조건을 독립 검사하며 읽기 전용 모드에서는 모터 레지스터 쓰기를
거부한다. native transport의 10ms 주기 감시 스레드는 마지막 명령으로부터 `max_cycle_s`가
지나면 최신 관절 위치를 유지하고 재구동을 잠근다. 그리퍼 목표를 바꾸거나 토크를 끄지 않는다.
프로세스 종료·장치 단절 때의 물리 정지는 현장 검증 대상이다.
토크 OFF 상태의 읽기 전용 진단은 가능하며, 읽기 전용에서는 Stay 이동이나 토크 ON을 하지 않는다.
OpenCR 테이블의 약 20 ms 갱신 사이에 중복 카운터를 읽으면 최대 100 ms 안에서 새 값을
기다린다. 중복 값을 새 피드백으로 재사용하지 않으며 카운터 역행·실제 정지는 거부한다.

직접 OpenCR 런타임의 ROS, 기구학과 단계 관리는 Python에 남아 있다.
이 경로의 로컬 시험 결과와 로봇용 소스 묶음 생성·배포 방법은
[로컬 검증 및 배포 준비](docs/local_validation.md)에 기록한다.

통합 런치 진입점과 현재 남은 실물 조건은
[통합 런치 및 실행 준비 상태](docs/single_launch_readiness.md)를 참고한다.
Jetson 실측 결과와 Stay 초기화 경로는 [실제 로봇 검증 기록](docs/robot_validation_20261122.md)에 있다.
joint4 카메라 고정 수정과 기존 하드웨어에 붙이는 통합 미션 런치는
[호스트 마무리 및 다음 로봇 실행](docs/host_finish_20261122.md)에 기록한다.
