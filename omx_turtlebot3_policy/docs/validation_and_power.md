# omx_turtlebot3 v6 원격 배포 검사와 전원 분리

2026년 11월 20일, 리더 로봇 `a@192.168.0.11`에 v6 모델과 비구동 추론 검사 패키지를 배포했다.
Jetson에서 모델 추론은 검증했지만 실물 매니퓰레이터 동작은 검증하지 않았다.
현재 모든 보드가 함께 꺼지는 증상은 공통 전원 경로를 먼저 점검해야 한다.

## 배포 경로와 범위

- 원격 소스: `/home/a/turtlebot3_ws/src/omx_turtlebot3_policy`
- 원본 소스: `/home/ktj/omx_turtle_ws/src/omx_turtlebot3_policy`
- 모델: `models/omx_turtlebot3_v6/actor.onnx`
- 보조 형식: `actor.ts`, `actor_weights.txt`, 학습 원본 `policy.zip`
- 체크섬: `models/omx_turtlebot3_v6/SHA256SUMS`, 전송 후 전 항목 일치
- 원본 체크포인트 SHA256: `6d8fc7f6607cc49a654dc577260e61947aa9e1b8fe23f6160330c15e0558c8c1`
- 원격 `colcon build --packages-select omx_turtlebot3_policy` 성공
- 기존 `manipulator_control`과 `leader_runtime.launch.py` 변경은 보존
- 모터 명령, ROS 제어 publisher, 직렬 제어, 자동 실행 런치는 추가하거나 실행하지 않음

## 추론 검사 결과

실제 시뮬레이션 rollout 관측과 추가 시험 벡터를 합친 973개 입력을 원본 PPO와 비교했다.
최대 절대 오차 허용치는 `1e-5`다. 시간은 워밍업 후 단일 입력 추론 평균이며 ROS 주기나
전체 제어 지연이 아니다. CUDA 시간에는 매 반복 동기화가 포함된다.

| 실행 환경 | 최대 절대 오차 | 평균 추론 시간 |
|---|---:|---:|
| 로봇 C++ Eigen CPU | 3.58e-7 | 4.97 µs |
| 로봇 PyTorch 2.10 CPU | 5.96e-7 | 124.77 µs |
| 로봇 PyTorch 2.10 CUDA | 2.98e-7 | 455.74 µs |
| 호스트 ONNX Runtime 1.20.1 CPU | 6.56e-7 | 6.12 µs |

로봇에 ONNX Runtime은 설치돼 있지 않아 ONNX 실행은 호스트에서 검증했다.
CUDA 실행 가능 여부와 작은 신경망에서의 속도 이득은 별개다. 이번 단일 입력 검사에서는
C++ CPU가 가장 빨랐다. GPU에 대한 긴 부하 시험은 하지 않았다.

## 실물 제어를 아직 연결하지 않은 이유

최종 v6 모델의 새 시뮬레이션 검사(seed 8000부터 8009)는 **7회 성공, 3회 실패**였다.
실패 seed 8003, 8004, 8007은 모두 `stale_rgb_target`으로 접근 중 관측 유효기간이 끝나 정지했다.
안전 유효기간을 늘려 성공으로 처리하지 않았다. 이 결과는 실물 성공률이 아니다.

- 구형 로컬 C++ 제어기는 33개 입력과 256×256 은닉층, v6는 69개 입력과 128×128 은닉층이다.
- 출력 4개는 절대 관절각이 아니라 기준 경로에 더하는 제한된 관절 보정량이다.
- 학습 중 사용하는 양쪽 손가락 접촉과 팔로워 적재면 접촉은 MuJoCo 값이다.
  실물에서 검증된 측정·추정 방법 없이 임의로 채울 수 없다.
- 카메라 보정과 관절 피드백, FK, 물체 좌표계, 단계별 경로·충돌·관측 만료 정지를 연결해야 한다.
- 이번 패키지는 추론 검사용이며 실제 구동을 켜는 옵션이 없다.

## 전원 경로와 증상

사용자가 확인한 현재 배선과 증상:

- 어댑터 출력: **12V 5A 60W**
- Jetson도 OpenCR에서 전원을 공급받음
- Astra 연결 시 모터만 멈추는 것이 아니라 **모든 보드의 전원이 꺼짐**
- 이동 전원 계획: UGREEN PB720 20000mAh와 C→DC 케이블

이 조건은 개별 카메라 드라이버 오류보다는 공유 전원 과부하·순간 전압 강하·보호 차단과
일치한다. 단락, 케이블·커넥터 접촉 불량도 배제되지 않았다. 고장 원인 확정에는 어댑터 출력과
Jetson/OpenCR 입력의 부하 시 전압·전류 측정이 필요하다. 전원 차단이 반복되는 상태로 모터
구동을 재현하지 않는다.

## 원격 읽기 전용 확인값

- 보드: Jetson Orin Nano Engineering Reference Developer Kit Super, ARM64, Ubuntu 22.04, ROS 2 Humble
- Jetson 전력 모드: `MAXN_SUPER`, 변경하지 않음
- USB 2.0 허브 `1-2`의 각 포트에 Astra, OpenCR, CP210x, Jieli 카메라가 연결됨
- 허브 기술자는 `Self Powered`, `MaxPower 0mA`다. 내장 허브일 수도 있으며 외부 전원이
  별도로 연결됐다는 증거는 아니다. USB 기술자만으로 배선·공급 여유를 판단하지 않는다.
- Astra 기술자: `Bus Powered`, `MaxPower 500mA`. 실제 전류 측정값이 아니다.
- OpenCR은 `/dev/ttyACM0`에 열거됨. USB 열거는 모터용 12V 전원의 정상 여부를 보장하지 않는다.
- 현재 부팅의 `oc1_event_cnt`, `oc2_event_cnt`, `oc3_event_cnt`: 검사 전후 모두 0
- 당시 Jetson 모듈 VDD_IN: 약 5.096V, 0.976A. **모듈 내부 입력**이며 어댑터 12V나
  로봇 전체·USB·모터 소비전력이 아니다.
- USB audio sample-rate/frequency 오류는 있으나 이 오류 자체는 전원 부족의 증거가 아니다.
- 현재 부팅의 검사 로그에서 Astra/OpenCR의 반복 USB disconnect/reset은 발견되지 않음
- 이전 부팅에서 전원이 끊겼을 때의 전압과 로그는 확보되지 않아 현재 카운터 0으로 과거 차단을 배제할 수 없음

NVIDIA는 VDD_IN을 전체 모듈 전력으로 정의하며, 저전압·평균 과전류·순간 과전류 이벤트
카운터를 별도로 제공한다. [NVIDIA 전력 모니터링 설명](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html)

## 권장 하드웨어 변경

첫 비교 시험은 배터리 전환보다 **전원 분리**를 먼저 한다.

```text
기존 12V 어댑터 ── OpenCR ── 매니퓰레이터와 구동 모터
Jetson 전용 전원 ── Jetson DC 입력 ── USB 카메라와 통신 장치
                       ↑
                 OpenCR에서 받던 전원은 분리
```

1. 모든 전원을 끄고 물체를 제거한 뒤 팔을 안전하게 받친다. 토크가 사라지면 팔이 내려올 수 있다.
2. OpenCR에서 Jetson으로 가는 전원선을 분리하고 Jetson에는 해당 캐리어 보드에 맞는
   정격·극성·잭 규격의 별도 전원을 사용한다. 두 전원 출력을 병렬로 묶지 않는다.
3. USB 데이터 연결은 유지할 수 있지만 전원 역급전이 없는 구성인지 확인한다.
   Jetson Orin Nano 기본 보드의 USB-C는 전원 입력용이 아니다. DC 잭은 5.5×2.5mm다.
   [NVIDIA 하드웨어 안내](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/hardware_layout.html)
4. Astra를 다른 기능에서 쓰지 않는다면 전원을 끈 상태에서 분리한다. 이번 픽앤플레이스는
   그리퍼 RGB 카메라를 사용하며 depth가 필요 없다.
5. Astra가 필요하면 역급전 방지와 충분한 포트 전력이 명시된 별도 전원형 USB 허브를 검토한다.
   원래 상위 전원의 여유가 없는 상태에서 같은 전원에 허브만 추가하는 것으로 해결되지는 않는다.
6. 분리 상태에서 컴퓨터만 켜고 카메라별 연결·스트리밍 순서로 전압과 로그를 비교한다.
   그 다음 현장 비상 정지와 무부하 팔 시험을 진행한다. 정격을 모르는 배선·커넥터·퓨즈는 교체 전 확인한다.

OpenCR 공식 사양은 5V 출력 최대 4A, 12V·5V 출력 합계 55W 이내다.
60W 어댑터를 사용해도 보드 내부 출력 한도가 늘지는 않는다.
[ROBOTIS OpenCR 사양](https://emanual.robotis.com/docs/en/parts/controller/opencr10/)

## PB720과 C→DC 케이블

PB720 제조사 배포 사양의 C1 출력은 12V 3A에서 36W, 15V 3A에서 45W, 20V 5A에서 100W다.
따라서 **12V C→DC 케이블로 로봇 전체를 공급하면 현재 60W 어댑터보다 여유가 줄어든다.**
20000mAh는 용량이며 피크 출력 전류를 보장하지 않는다. 보유 제품 라벨과 다중 포트 사용 시
전력 배분, 케이블 협상 전압·정격·극성을 최종 확인해야 한다.
[UGREEN PB720 제조사 사양표](https://www.lemona.lt/Files/Instrukcijos/TI/En/Pdf/PB720%2025188.pdf)

이동 구성은 PB720을 Jetson 전용으로 쓰고 OpenCR·모터 전원은 분리하는 방향이 우선이다.
단, Jetson 전용이라도 선택한 PD 전압에서의 출력 한도와 보드·카메라 피크 부하를 확인해야 한다.
`MAXN_SUPER`를 그대로 사용해도 충분하다고 보장할 수 없다.

**15V 또는 20V PD 출력을 OpenCR·12V 모터에 직접 연결하지 않는다.**
OpenCR의 보드 입력 허용 범위와 연결된 모터의 허용 전압은 다르다.
고전압 PD로 12V 모터 전원을 만들려면 별도의 검증된 강압·과전류·과전압 보호 및 퓨즈 설계가 필요하다.
[ROBOTIS DYNAMIXEL 전원 전달 안내](https://en.robotis.com/service/forum_view.php?bbs_no=2407965&page=17&save_sca=&save_stx=&sca=&sfl=&stx=)

## 다음 확인

Jetson 독립 전원으로 변경한 뒤 Astra 연결 시 전체 전원 차단이 사라지는지 확인한다.
전압 측정과 무부하 시험을 통과한 다음에만 v6 실물 제어 이식 및 픽앤플레이스 검증을 진행한다.
