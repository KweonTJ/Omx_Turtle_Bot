# 호스트 작업 모니터 검증 기록

2026-10-13~14, Ubuntu/ROS 2 Humble, Qt 5.15.3, RViz 11.2.29.
실제 로봇·학습 정책을 실행하지 않은 UI/브릿지 검사다.

| 검사 | 결과 |
|---|---|
| C++ gtest | 7/7 통과: JSON/중복, 상태, 부분 관절, timeout, proximity, 이미지 검증 |
| Python pytest | 11/11 통과: 브릿지 allowlist·type·loop/중복 거부, 셸 잘못된 인수 거부·정상 launch 인수 |
| colcon 보고 | CTest wrapper 포함 20 tests, errors/failures/skipped 0 |
| Raw / domain 224 | 관절·카메라·상태·ID 수신, 근접 전환, 요청 중복 제거, 송신 중단 후 만료 통과 |
| JPEG / domain 225 | 위 항목 동일 통과 |
| 브릿지 231→230 | JointState payload 동일, cmd_vel/실제 제어 토픽 없음 |
| native GUI / domain 223 | RViz 로봇 재질/가상 동작·영상·관절 그래프 육안 확인, 종료 시 해당 자식 정리 |
| 무입력 / domain 222 | 입력 live 모두 false, requests 0, demo false; 자동 합성 대체 없음 |
| native GUI 발행 목록 | rosout, parameter_events만 발행. 주행·팔·그리퍼 명령 발행 없음 |

원본 실행 아티팩트: `/tmp/leader_monitor_test.hfhr9g9u`,
`/home/ktj/omx_turtle_ws/build/omx_leader_monitor/test_results/omx_leader_monitor`.
재현: `colcon test --packages-select omx_leader_monitor`,
`python3 src/omx_leader_monitor/test/runtime_smoke.py` (workspace source 후).

동봉 XML은 단위 검사 결과이며 `raw.log`, `compressed.log`, `bridge.log`는 격리 통합 시험 프로세스 로그다.
데모 영상은 합성이고 마커 그림도 검출용 유효 ArUco 코드가 아니다.
실카메라/원격 네트워크/선반 충돌/새 ID 선택 제어/실물 파지 성공은 이 시험으로 검증되지 않는다.
