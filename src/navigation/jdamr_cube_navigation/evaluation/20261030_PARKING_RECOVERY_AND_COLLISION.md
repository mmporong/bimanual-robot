# 2026-10-30 주차 입력 복구·도크 중앙 보정·충돌 영역 정리

## 실차 결과와 판정

급수대와 테이블1 주차, 최종 도킹은 사용자 확인 성공이다. 컵배치는 Windows `BiSOFollower is not connected`로 실패했다. 도킹 자동 결과는 confirmed=false였으며 이를 성공으로 덮어쓰지 않았다.

실행 원본은 기존 PC/Pi의 `$HOME/jdamr_data/maps/20261026_manual_160135/runtime/runs/`에 있다. run별 `operator_acceptance.json`은 기존 PC에 기록했다.

| 구간 | run | 근거 |
|---|---|---|
| 급수대 | water_station_20261030_103334_d84000f4 | 사용자 확인 성공, 센서 내부 추정 |
| 테이블1 | table_01_20261030_111336_14293766 | 자동 주차완료·사용자 확인 성공 |
| 도크 중앙보정 실패 | table_01_20261030_112511_d763e062 | 초기 좌우 -0.0421m → 보정후 -0.1016m, 방향 잔차 0.2494rad |
| 최종 도크 | table_01_20261030_112651_18d442fb | 사용자 확인 성공; 라이다 양끝 0.012912/0.013293m, 자동 confirmed=false |

## 수정

1. `rear_box_service.py`: 중앙 보정 곡선의 position-only 2.5cm 수동 handoff를 제거하고 기존 entry_heading_checker로 위치와 방향을 함께 완료한다. 과거 로그에서 곡선이 방향을 마치기 전에 끊긴 증거가 있다. 변경 후 실차 반복정확도는 아직 확인하지 않았다.
2. `depth_box_parking.py`: depth 발행자가 매칭돼 있지만 콜백이 최대 영상 나이의 4배 동안 없으면 CameraInfo/depth 구독을 재생성한다. 현재 500ms 설정에서는 2초다. 기존 관측 누적을 비우고 detected=false 상태를 발행한다. 센서 부재/침묵을 정상 관측으로 만들지 않는다.
3. `astra_tcp_receiver.py`: 같은 수신 프레임에서 depth/CameraInfo를 RGB보다 먼저 발행한다. 대용량 RGB 발행이 depth보다 앞서 대기하는 순서를 제거한다. 실제 전송 지연 감소량은 측정하지 않았다. 500ms 신선도 제한과 latest-only 정책은 유지한다.
4. `restaurant_service.py`: 후진 경로의 정적 지도 검사가 항상 wheel-width 직사각형을 쓰던 것을 runtime stepped footprint로 바꾼다. `reverse_parking.py`의 정적 corridor 검사도 오목한 외곽을 볼록화하지 않고 볼록 조각으로 분해해 각 조각의 이동 영역을 합친다. 바퀴 사이 빈 공간을 차체로 채우던 과대 판정을 줄이며, 회전 중 쓸림 영역은 기존 sagitta 보정을 유지한다. padding이 존재할 때는 기존 보수적 외곽 처리를 유지하며, Nav2 실시간 경로 판정과 Collision Monitor는 유지한다. 별도 `CachedStaticCorridorValidator`의 외곽 처리는 이번에 변경하지 않았다.
5. `reverse_parking.py`, `rear_collision_profile.py`: 충돌 증거에 실제 외곽, 최초 외곽 안/밖, 경계까지 거리, 충돌 예상 자세, 예측시간 및 유효 속도를 기록한다. 다음 현장에서 물체가 실제 외곽에 걸린 것인지 이동 예상 영역인지 구분할 수 있도록 했다.

## 카메라 침묵: 사실과 가설

11:08:22 AST1 first_image_valid 이후에도 11:11~11:12 관측 이력은 status_history=[]/observer_silent였다. 11:12:41 관측 서비스 재시작 후 다음 주차에서 면 검출이 회복됐다. DDS reader 정체는 가설이며 미들웨어 내부 원인은 미확정이다. CameraInfo 순서·QoS 불일치·콜백 예외를 확정 원인이라고 주장하지 않는다. 구독 재생성은 관측된 침묵 상태의 복구 경로이며 실제 재발 상황에서 효과 확인이 남았다.

## Collision Monitor 조회 불일치

짧은 ros2 CLI 조회에서 collision_monitor 미발견 및 /cmd_vel 발행자0을 봤지만, 후속 프로세스 확인과 rclpy GetState는 CM/controller/planner 모두 active였다. journal에는 10:34 StopZone/FootprintApproach 개입이 기록됐다. 따라서 “CM이 꺼져 있었다”는 결론은 근거가 없다. CLI/DDS discovery 차이는 남은 조사 대상이다.

## 라이다 감지와 차체 외곽

사용자가 노트북 선을 라이다 쪽에서 치운 뒤 같은 설정의 후진 경로가 통과했다(급수대와 테이블 이탈). 케이블 영향은 유력하지만 raw scan 대조 없이 단일 원인으로 확정하지 않는다.

회전을 멈춘 live_scan 점 (-0.2660,-0.1986)m은 설정된 차체 다각형 내부였다. 이 사례를 먼 물체에 대한 과도한 여유라고 단정할 수 없다. 소스 footprint_padding은 0이지만 footprint 좌표 자체에는 기존 5mm 여유가 포함되어 있다. 이 작업에서는 실측 없이 그 좌표를 줄이지 않았다. 정적 경로에서 별도로 만들던 큰 직사각형은 제거했다. 회전으로 쓸고 지나갈 영역, 감속/통신·센서 지연 동안의 이동, 센서 신선도 감시는 유지한다.

## 미해결·판정 한계

- 최종 도킹은 약13mm에서 사용자가 성공으로 수용했다. 실행기는 마지막 보정 중 swept-footprint 감지를 받았고 자동 목표 조건은 미충족이었다. 목표 10mm/기존 허용오차를 넓히거나 confirmed를 강제로 true로 만들지 않았다.
- 새 구독 복구와 depth 우선 발행이 실제 Wi-Fi 지연·정지 시간을 얼마나 줄일지는 재연결/실차 근거가 필요하다.
- 중앙 보정 변경 후 한번의 후진으로 끝나는지는 다음 실차에서 확인해야 한다.
- Windows 팔 연결 실패, 실패 주문의 재시도 복원, 서버 IDLE_AT_DOCK와 실제 차체 위치 불일치는 별도 조작/미션 통합 과제다.
- 장애물 감지를 무시하는 우회는 적용하지 않았다. 새 물리 이동은 이 수정 작업에 포함하지 않았다.

## 확인 범위

관측 노드·후면 도킹 참조·reverse_parking·rear_collision_profile 묶음 212개와 후진 정적외곽 회귀 1개가 통과했다. 이는 소프트웨어 확인이며 실차 주차 성공의 대체 근거가 아니다. 6개 변경 모듈의 Python 문법과 diff도 확인했다. 전체 테스트 통과를 뜻하지 않는다.

## 실제 적용 속도 조회

2026-10-30 12:01 KST Pi의 읽기 전용 파라미터 응답 기준이다. 이 작업에서 속도를 변경하지 않았다.

| 항목 | 실제 값 |
|---|---|
| FollowPath 목표 / 최소 접근 속도 | 각각 0.12m/s |
| 곡률 기반 / 비용 기반 RPP 감속 | 모두 false |
| Parking / ParkingReverse 목표 속도 | 각각 0.08m/s |
| velocity_smoother 전진 / 후진 상한 | 0.12 / 0.08m/s |
| SlowdownZone | disabled |
| StopZone / FootprintApproach | enabled |

따라서 통로의 RPP 곡률·비용 감속과 별도 SlowdownZone이 켜져 있어서 느린 상태는 아니다. 면 접근 코드는 continuous table 상한 0.10m/s, 일반 이탈 상한 0.05m/s와 최종 거리 taper를 별도로 사용한다. 각 주행 구간의 지연은 명령·실제 속도·센서 입력 대기·CM 개입 로그를 구분해야 하며, 설정 조회만으로 실제 감속의 원인을 확정하지 않는다.

## Pi 배포

- 2026-10-30 12:27 KST 위 6개 모듈을 기존 활성 소스에 반영했다. Pi에서 문법 확인, 로컬과 SHA256 일치를 확인했다.
- 기존 파일 백업: `$HOME/jdamr_deploy/backups/parking-recovery-20261030_122745/`.
- `jdamr-box-executor.service`, `jdamr-box-observer.service`, 사용자 유닛 `jdamr-astra-tcp.service`를 재시작했다. navigation을 포함한 4개 유닛 active 및 이동 요청·실행 중 spool 항목 없음까지 확인했다. 새 이동 명령은 제출하지 않았다.
- 수신기는 기존 peer 192.168.0.175로 TCP5600 대기 중이다. 새 Windows 192.168.0.157은 SSH 접속만 완료됐고 사용자가 Astra·팔을 아직 연결하지 않았다고 알렸다. 카메라 송신 복구·새 노트북 이식 완료·주행 준비 완료를 뜻하지 않는다.
- 기동 로그에 Fast DDS SHM 포트 잠금 경고가 남았다. 서비스 active만으로 DDS 영상 전달이 정상이라고 판정하지 않는다. 현재 센서 미연결 상태이므로 실제 RGB-D 전달 확인은 연결 후 남아 있다.
