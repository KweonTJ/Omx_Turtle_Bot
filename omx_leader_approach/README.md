# 리더 로봇 호스트 인계 후 평행 정렬과 직진 접근

Depth 카메라나 추가 마커 없이, 호스트의 목표 방향과 로봇 IMU·휠 오도메트리를 사용한다.
그리퍼 RGB 인지 패키지 `omx_eef_vision`은 기존 물체 ArUco ID 1~4의 인지만 담당한다.
이 패키지는 팔을 제어하거나 기존 PPO·ONNX 모델을 교체하지 않는다.

## 정렬 기준과 동작

IMU·오도메트리는 정리대 방향을 스스로 측정하지 못한다. 호스트가 정리대 앞에 로봇을 세우고,
정리대에 정면으로 접근할 `target_yaw_odom`을 **동일한 odom 좌표계의 radian**으로 전달해야 한다.
호스트가 map 좌표계를 사용하면 odom으로 변환한 뒤 전달한다. 목표각 또는 좌표계가 없으면 START를 거절한다.

1. `WAIT_OVERVIEW`: 인지 패키지가 네 물체의 동시 관측을 확인하고, 새 프레임의 네 pose가 모두 유효할 때까지 정지한다.
2. 로봇이 정지하고 IMU와 odometry 시각 차이가 100 ms 이내일 때 두 yaw 좌표계의 차이를 한 번 계산한다.
3. `ALIGN_PARALLEL`: 선속도 0으로 제자리 회전한다. 목표 오차 0.5° 이내와 회전 속도 0.025 rad/s 이하를 새 IMU 샘플 10개 연속 확인한다.
4. `DRIVE_STRAIGHT`: IMU로 목표 방향을 유지하고 odometry 이동량으로 정지한다. 방향 오차가 2° 이상이면 전진을 멈추고 회전만 보정한다.
5. 이동 후 정지 상태를 새 odometry 10개로 확인하면 `READY_FOR_ARM`을 알린다. 별도 모델 제어기가 이 결과를 받아 팔 작업을 수행해야 한다.

기본 전진 거리는 382 mm, 속도는 55 mm/s다. 이는 **몸체 앞끝과 정리대 앞면 간격 500 mm인 지정 시작 위치**를 전제로 한 학습 설정이다.
카메라로 절대 거리나 장애물을 측정하는 접근 제어가 아니다. 정리대 위치가 바뀌면 호스트 시작 위치와 전진 거리를 함께 맞춰야 한다.

IMU 영점 차이는 `imu_goal = wrap(imu_now + wrap(host_yaw_odom - odom_yaw_now))`로 옮긴다.
이는 고정 영점 차이를 처리하는 것이지, IMU 드리프트·휠 슬립을 제거하거나 실제 평행을 보장하는 보정은 아니다.
물체의 실제 파지 좌표는 인지 패키지의 마커 실측 크기, 카메라 보정과 hand-eye TF 보정이 필요하다.

## 토픽 계약

| 방향 | 토픽 | 메시지와 역할 |
|---|---|---|
| 입력 | `/leader_approach/request` | `std_msgs/String` JSON START 또는 STOP |
| 입력 | `/imu_broadcaster/imu` | `sensor_msgs/Imu`, 유효한 orientation·angular_velocity 필수 |
| 입력 | `/diff_drive_controller/odom` | `nav_msgs/Odometry`, frame_id가 설정한 `odom`과 일치 |
| 입력 | `/perception/objects` | `std_msgs/String`, fresh·overview_ready·ID 1~4 pose_valid·usable_2d 검사 |
| 출력 | `/perception/target_request` | 기존 인지 패키지에 SELECT ID 요청 |
| 출력 | `/leader_approach/cmd_vel` | `geometry_msgs/Twist`, mux 입력으로 연결하기 전 제안 명령만 출력 |
| 출력 | `/leader_approach/status` | `std_msgs/String`, 상태·요청 ID·목표각·팔 작업 준비 여부 |

START 예시이며 `0.0`은 실제 정리대 방향에 맞게 호스트가 계산해야 한다.

```json
{"command":"START","request_id":"pick-001","object_id":2,"target_yaw_odom":0.0,"odom_frame":"odom"}
```

정지 요청은 `{"command":"STOP"}`이다. 동일 request_id는 재실행하지 않는다.
진행 중에는 다른 START를 받아 작업을 바꾸지 않는다. 센서가 300 ms 이상 오래되면 명령을 0으로 출력하고 팔 준비 신호도 해제한다.
정렬·전진 시간 초과, 횡방향 오차 20 mm 초과, 목표를 10 mm 넘겨 지나가면 FAULT로 정지한다.
비상 정지와 최종 속도 제한은 기존 로봇 제어·mux 계층에서 계속 담당해야 한다.

## 빌드와 확인

```bash
cd /home/ktj/omx_turtle_ws
colcon build --packages-select omx_leader_approach --symlink-install
source install/setup.bash
ros2 launch omx_leader_approach approach.launch.py
```

launch는 mux 모드를 선택하거나 `/cmd_vel`에 직접 연결하지 않는다.
실기 적용 전에 호스트의 주행 인계, 기존 mux 소유권, odom 프레임, 센서 방향과 타임스탬프를 검증해야 한다.
이 패키지는 실기 구동이 아닌 오프라인 ROS 메시지 시험과 분리된 시뮬레이션으로 검증한다.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/omx_leader_approach:$PYTHONPATH \
  /usr/bin/python3 -m pytest -q src/omx_leader_approach/test
```

학습용 v3는 `/home/ktj/omx_train_ws/configs/omx_turtlebot3_v3.yaml`을 사용한다.
실제 정렬 코어를 학습 scene의 source snapshot으로 복사해 같은 목표각 변환과 안정화 판정을 사용한다.
