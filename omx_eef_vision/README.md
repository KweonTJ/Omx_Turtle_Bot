# OMX EEF Vision

## 새 선반 임무: 인지 전용 4물체 프로필 (2026-10-14)

`shelf_perception.launch.py` / `multi_object_vision_node`를 사용한다.
사용자 지정 ID **1~4**, Dictionary **DICT_4X4_50**, A4 인쇄용 검은 사각형 **40 mm**다.
마커 크기·카메라 TF 실측 확인 플래그는 기본 false이며, 거리 좌표를 제어기에 연결하지 않는다.
아래 기존 단일 마커 launch/설정과 구분한다. 기존 AprilTag 설정은 그대로 보존했다.

- 한 프레임에서 모든 ID, 코너, 픽셀 중심, 중복 여부를 검출한다. CameraInfo가 없어도 2D 검출 가능.
- 보정 해상도/optical frame 일치, 거리·재투영 오차, 촬영 시각 TF를 확인한 마커 중심 자세를 제공한다.
- 처음 4개 동시 인지 3프레임을 확인한 뒤, 요청 ID만 선택한다. 근접 시 다른 ID가 화면 밖으로 나가도
  선택한 ID는 계속 추적한다. 요청 ID의 미검출·중복·영상 지연·TF 실패는 무효 처리한다.
- 목표가 이동하면 현재 검출 위치를 사용한다. ID별 고정 좌표나 오래된 위치를 재사용하지 않는다.
- **인지 결과만 제공한다.** 팔/그리퍼/베이스 제어, 파지점 계산, 경로·충돌·도달성 판단, RL 추론 없음.
- `/target/*`에 출력하지 않는다. 새 OMX 모델이 사용할 관측 계약은 추후 별도로 연결한다.

```bash
source /opt/ros/humble/setup.bash
cd /home/ktj/omx_turtle_ws
colcon build --symlink-install --packages-select omx_eef_vision
source install/setup.bash
ros2 launch omx_eef_vision shelf_perception.launch.py
```

새 터미널에서 인지 대상만 선택:

```bash
ros2 topic pub --once /perception/target_request std_msgs/msg/String \
  '{data: "{\"request_id\":\"scan-001\",\"command\":\"SELECT\",\"object_id\":3}"}'
ros2 topic echo /perception/selected_info
```

해상도/보정, camera-only 드라이버 실행, 토픽 계약, 시험 및 정리대는
[15번 문서](<../docs/15. 그리퍼 다중 물체 인지와 2단 정리대 디지털 트윈.md>)를 참고한다.
인쇄 파일: [01. ArUco ID 1-4 인쇄용.svg](<./markers/01. ArUco ID 1-4 인쇄용.svg>).
100% 인쇄하고 검은 부분 한 변을 재측정한 뒤 설정한다. 물체마다 서로 다른 ID를 한 장씩 부착한다.

## 기존 배송 v4 단일 마커 프로필

OpenMANIPULATOR-X의 EEF USB 카메라에서 ArUco 마커를 검출하고 배송 상자의 6D 자세를 생성하는 ROS 2 패키지다. YOLO나 별도 객체 검출 모델은 사용하지 않는다.

## 처리 흐름

```mermaid
flowchart LR
    IMAGE[EEF Camera Image] --> ARUCO[ArUco Detection]
    INFO[Calibrated CameraInfo] --> PNP[IPPE Square Pose]
    ARUCO --> PNP
    PNP --> QUALITY[크기·깊이·재투영 오차]
    QUALITY --> OFFSET[Marker → Box Offset]
    OFFSET --> TF[Camera → base_link TF]
    TF --> FILTER[연속 검출·점프·EMA 필터]
    FILTER --> TARGET[Object Pose]
```

마커가 영상에 보이는 것만으로 팔을 움직이지 않는다. 카메라 보정, 자세 품질, TF, 작업영역과 연속 검출 조건을 모두 통과한 경우에만 `/target/valid=true`를 발행한다.

## 인터페이스

| 구분 | 토픽 | 타입 | 의미 |
|---|---|---|---|
| 입력 | `/eef_camera/image_raw` | `sensor_msgs/msg/Image` | EEF USB 카메라 영상 |
| 입력 | `/eef_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 보정된 카메라 내부 파라미터 |
| 출력 | `/target/aruco_pose` | `geometry_msgs/msg/PoseStamped` | 카메라 optical frame 기준 원시 마커 자세 |
| 출력 | `/target/object_pose` | `geometry_msgs/msg/PoseStamped` | 필터를 통과한 `base_link` 기준 상자 목표 자세 |
| 출력 | `/target/aruco_visible` | `std_msgs/msg/Bool` | 설정한 마커가 현재 영상에서 검출됨 |
| 출력 | `/target/valid` | `std_msgs/msg/Bool` | 제어기가 목표 자세를 사용해도 됨 |
| 출력 | `/target/aruco_status` | `std_msgs/msg/String` | 대기·거부·검출 상태와 수치 |
| 출력 | `/target/aruco_debug_image` | `sensor_msgs/msg/Image` | 코너, 축과 품질 수치가 표시된 영상 |

`/target/aruco_pose`와 `/target/aruco_visible`은 기존 `aruco_mp_bridge` 호환 출력이다. RL 제어기는 `/target/object_pose`와 `/target/valid`를 사용한다.

## 기준 설정

| 항목 | 기본값 | 확인 사항 |
|---|---:|---|
| Dictionary | `DICT_APRILTAG_36h11` | 실제 인쇄한 마커와 반드시 일치 |
| Marker ID | `0` | 현재 `accept_any_marker=true`이므로 첫 검출 marker 사용 |
| Marker size | `0.04 m` | 실제 YAML 기준, 검은 정사각형 한 변 재실측 필요 |
| 안정 검출 | `3 frames` | 연속 통과 전에는 `valid=false` |
| 최대 재투영 오차 | `8 px` | 높으면 자세가 흔들리므로 거부 |
| 출력 좌표계 | `base_link` | EEF 카메라 TF 필요 |
| 영상 timeout | `1.0 s` | 초과 시 즉시 무효화 |

현재 기본 Dictionary는 `DICT_APRILTAG_36h11`, 크기는 `0.04 m`다. 실제
인쇄물과 다르면 거리 추정도 같은 비율로 틀어지므로
`config/eef_vision.yaml`을 먼저 수정한다. 여러 marker가 보일 수 있는
환경에서는 `accept_any_marker=false`와 고정 ID를 사용한다.

## 상자 중심 보정

ArUco가 주는 위치는 마커 중심이다. 파지 목표가 상자 중심과 다르면 마커 좌표계 기준 변환을 설정한다.

```yaml
object_offset_xyz: [0.0, 0.0, 0.0]
object_offset_rpy: [0.0, 0.0, 0.0]
```

배송 상자는 `6 × 5.5 × 5.5 cm`지만, 마커를 어느 면과 방향으로 부착하는지에 따라 부호가 달라진다. 임의로 `2.75 cm`를 적용하지 말고 `drawFrameAxes`의 축 방향을 확인한 뒤 측정값을 넣는다.

## 실행

EEF 카메라 드라이버와 robot state publisher가 먼저 실행 중이어야 한다. `omx_eef_vision` launch는 카메라 드라이버를 중복 실행하지 않는다.

```bash
cd /home/ktj/omx_turtle_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select omx_eef_vision
source install/setup.bash
ros2 launch omx_eef_vision eef_vision.launch.py
```

기본 카메라 보정 파일은 `turtlebot3_manipulation_bringup/config/eef_usb_camera.yaml`이며 하드웨어 launch가 `v4l2_camera`에 전달한다. `/eef_camera/camera_info`의 `K[0]`, `K[4]`가 0이면 자세를 발행하지 않는다.

## 확인

```bash
ros2 topic hz /eef_camera/image_raw
ros2 topic echo /eef_camera/camera_info --once
ros2 topic echo /target/aruco_status
ros2 topic echo /target/aruco_pose --once
ros2 topic echo /target/object_pose --once
ros2 topic echo /target/valid
```

디버그 영상은 RViz의 Image display 또는 `rqt_image_view /target/aruco_debug_image`로 확인한다. 마커가 보이는데도 자세가 나오지 않으면 status에서 Dictionary, ID, perimeter, depth, reprojection error와 TF 실패를 순서대로 확인한다.

## 안전 원칙

- 마지막 정상 자세를 무기한 재사용하지 않는다.
- 영상·검출 timeout, TF 실패, 작업영역 이탈 시 `valid=false`를 발행한다.
- `accept_any_marker=true`는 장면에 마커가 하나뿐일 때만 사용한다.
- 실제 marker size나 CameraInfo가 틀리면 거리도 같은 비율로 틀어지므로 실측과 캘리브레이션을 생략하지 않는다.
