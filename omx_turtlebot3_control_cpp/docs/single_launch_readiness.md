# 통합 런치와 실물 실행 준비 상태

2026-11-22 권한 변경 후 SSH로 Jetson에 접속해 ARM64 빌드·추론·입력을 시험했다.
모델과 소프트웨어 검증은 통과했다. 이어서 기존 mp_control의 Stay 후 제어 시작 순서를
C++ 런타임에 적용했다. 실물 Stay 이동·집기는 아직 검증하지 않았다.
상세 근거는 [실제 로봇 검증 기록](robot_validation_20261122.md)에 있다.

`omx_turtlebot3_runtime/launch/real_robot_cpp.launch.py`는 통합 진입점이다.
선택한 Python 환경에서 C++ ONNX·URDF·충돌 장면 검사를 먼저 실행하고, 성공하면 RGB
카메라와 C++ 추론·OpenCR 런타임을 시작한다. 기존 `ros2_control`, mp_control,
joint trajectory transformer는 이 직접 OpenCR 경로와 함께 실행하지 않는다.

기본 모드는 `read_only`다. `mode:=execute`에서는 필수 실물 보정 7개, 증거 경로,
집기·해제 전류 임계값이 갖춰지지 않으면 프로세스를 만들기 전에 거부한다. 필수 보정 키를
설정에서 지우거나 bool을 전류 숫자로 넣어도 준비 완료로 처리하지 않는다.

기존 `base.launch.py`의 Stay 목표는
`[0.104311, 0.027612, -0.001534, -1.638291]` rad로 모델과 같다.
기존 `mp_control/launch/aruco_pick_place.launch.py`는 hardware bringup을 먼저 띄우고
기본 8초 뒤 제어 노드를 시작한다. 새 런치는 같은 순서를 단일 C++ OpenCR 소유자 안에 구현했다.
`WAIT_DEADMAN → 경로 검사 → MOVING_STAY → READY → 임무 요청 → MISSION`으로 진행한다.
현재 위치를 목표 레지스터에 먼저 넣고 저속 프로파일을 설정한 뒤 필요한 경우 팔 토크만 켠다.
최소 3초의 동시 최소 저크 경로를 속도·가속도 상한에 맞춰 늘린다. 실제 엔코더 위치 오차
0.025 rad 이하·속도 0.03 rad/s 이하가 0.3초 유지되어야 READY가 된다. 단순 지연으로 도달을
가정하지 않는다. READY 대기 중에도 heartbeat·추종 검사를 계속하며 deadman이 만료되면 정지한다.
읽기 전용은 토크 OFF에서도 진단할 수 있으며 모터 쓰기·초기 이동은 하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch omx_turtlebot3_runtime real_robot_cpp.launch.py
# 별도 실물 보정 완료 후 사용할 모드
ros2 launch omx_turtlebot3_runtime real_robot_cpp.launch.py \
  mode:=execute config:=/검증된/실물보정.yaml
```

이것은 프로세스 통합 진입점이다. 임무 요청과 지속적인 현장 deadman 신호를 자동 생성하지
않으므로 런치 명령만으로 로봇이 집기를 시작하지 않는다. 현재 저장소 기본 설정에서는
실행 준비 상태가 false다. 사용자가 별도 보정 YAML도 아직 없다고 확인했다.

로컬에서 직접 실행한 `--check-startup`은 원본 ONNX 추론과 FK/URDF·충돌 장면 검사를
통과했으며 `serial_opened=false`, `motor_writes=0`, `execution_config_ready=false`였다.
런치 옵션 표시와 시작 순서·실패 전파·실행 차단에 대한 테스트도 추가했다.
Stay 초기화와 카운터 처리 수정 후 로컬 C++ 22개, Python 141개 시험이 통과했다. 별도 Torch 검증 벡터
환경 변수를 지정하지 않아 기존 선택 시험 1개는 제외했다. C++ ONNX 검사는 실제로 수행했다.

Jetson의 OpenCV 4.5.4에서 마지막 Clip 출력이 잘리는 문제를 수정했고 973개 벡터의
최대 오차 3.57969e-7을 확인했다. 초기화 시 이 비교를 반드시 수행한다.
Jetson에서도 C++ 14개, Python 130개 시험이 통과했다.

초기 버전의 실제 통합 런치는 모델 검사 후 카메라와 C++ 런타임을 시작했으나 토크 OFF로 중단됐다.
별도 카메라 재시험에서는 RGB 영상 200장과 CameraInfo 199건을 수신했다.
모터 쓰기는 0회였으며 토크 활성화·Stay 이동·집기·운반·안착 성공을 검증한 것은 아니다.

Stay 초기화와 카운터 처리 수정본을 같은 Jetson 검증 워크스페이스에 반영했다.
ARM64 재빌드, C++ 22개·Python 141개 시험이 통과했다(기존 선택 시험 1개 제외).
수정 후 실제 단일 런치는 읽기 전용으로 6초 실행하고 종료 코드 0으로 끝났다.
엔코더 샘플 117건, RGB·CameraInfo 각 177건을 받았고 모터 쓰기는 0회였다.
프레임 동기화 검사에 통과한 영상은 131건이었으며 ID 0·1 자세는 검출되지 않았다.
현재 영점 설정으로 joint2는 계속 약 -2.089 rad여서 제한 밖이다. 실물 이동과 전체 임무는 미검증이다.
