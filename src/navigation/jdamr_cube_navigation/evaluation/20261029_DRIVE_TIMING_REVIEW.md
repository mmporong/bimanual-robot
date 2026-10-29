# 2026-10-29 주행 대기시간·주기·신선도 검토

`fix/amr-continuous-precision-parking`(검토 기준 HEAD `ede1022`)의 실차 주행 경로에서
하드코딩된 대기시간·제한시간·주기·신선도 값을 센서 속도와 Nav2 Jazzy 동작에 대조했다.
근거는 코드, `PROGRESS.md`의 10-29 기록, `20260929_PARKING_FAILURES.md`, Nav2·robot_localization·
geometry2 Jazzy 소스다. 이 PC에는 실차 run 로그가 없으므로 **[확인]**은 코드·소스로 확인한 사실,
**[추정]**은 Pi 로그로 확인해야 하는 가설이다. 실차 효과는 아직 확인하지 않았다.

## 1. 실제 사용 설정

- Nav2 파라미터: `config/new_base_nav2_params.yaml`
  (`restaurant_session.sh` → `restaurant_service.launch.py` → `onboard_nav2_core.launch.py`). [확인]
- odom: EKF(바퀴+바이어스 보정 자이로) 30 Hz, 베이스 드라이버 odom 50 Hz, `cmd_timeout` 0.4 s.
- G4 LiDAR 약 10 Hz(코드 주석 7-10 Hz), Astra RGB 640×480·깊이 320×240 30 Hz, 깊이 관측기 5 Hz.

## 2. 바꾼 값과 근거

| 항목 | 이전 | 변경 | 근거 |
|---|---|---|---|
| 늦게 끝난 새 깊이 결과(나이 > 0.5 s) | 즉시 `table approach depth stale` → 정지·정착(0.5+0.5 s)·재출발 | 소비하지 않고 odom에 고정한 직전 면을 유지. 직전 입력과 직전 면이 `OBSERVER_SILENT_S`(1.5 s) 이내(건강한 가림 고정 제외)이고 CameraInfo가 살아 있을 때만. 영상 TF 대기 중 늦어진 프레임도 같다 | 관측기가 조용할 때는 이미 1.5 s까지 직전 면을 유지했다. 늦은 결과를 조용함보다 나쁘게 처리하던 불일치. 0.5 s 소비 기준은 유지 [확인] |
| `parking_motion_revision`의 회전 판정 | 자이로 표본 1개가 0.02 rad/s를 넘으면 이동 | 0.08 s 연속 또는 0.04 rad/s 초과일 때 이동. 병진·NaN·0이 아닌 `/cmd_vel`은 즉시 | 도크 초기화에서 이미 채택한 0.08 s 규칙. 정지 표본 105개 중 3개가 0.02 rad/s 초과(10-29 기록) [확인] |
| Nav2 FollowPath 오류 102(TF_ERROR) | 재시도 없이 run 종료 | 기존 입력 공백 경로로 1회 재전송. map→odom이 다시 발행된 뒤에만(최대 8 s) | Jazzy는 102에서 즉시 중단하고 `WouldAControllerRecoveryHelp`도 102를 제외한다 [확인]. 0.1115 s 차이는 AMCL τ 1.0 s를 넘어선 약 1.6 s 이상 map→odom 공백을 뜻한다 [추정] |
| 실행기·관측기 TF 수신 | 제어 루프와 같은 단일 스레드(제어 계산 중 /tf 대기열 정체, `can_transform(timeout)` 대기 중 콜백 불가) | 전용 TF 노드·데몬 스레드(`dedicated_tf.DedicatedTfListener`, 노드 이름 재지정 보호) | rclpy Jazzy에서 소유 노드를 spin하지 않아도 TF가 버퍼에 들어오는 것을 확인 [확인] |
| `_direct_motion` 명령 공백 | 기록 없음 | `max_publish_gap_s` 기록 | velocity_smoother `velocity_timeout` 0.5 s를 넘으면 이유 없이 감속 정지한다 |
| 1 cm 테이블 미세 보정 최소 이동 | 0.25 mm 같은 보정도 명령 | 1 mm(`TABLE_FINE_MIN_PROGRESS_M`) 미만은 명령하지 않고 측정값과 함께 종료 | 0.01 m/s에서 정지 지연만 0.7-1.7 mm이며 코드도 1 mm 미만을 진전으로 판정하지 못한다 |

## 3. 유지한 값과 이유

| 값 | 위치 | 판단 |
|---|---|---|
| 깊이·LiDAR 소비 신선도 0.5 s | `box_service.py`, `rear_box_service.py`, `rear_collision_profile.py` | 유지. 10 Hz LiDAR·5 Hz 관측기 기준 여유는 얇지만 오래된 센서로 이동하지 않는 계약이다. 정지 원인이던 늦은 결과만 위 표처럼 처리 |
| `OBSERVER_SILENT_S` 1.5 s | `box_service.py` | 유지. 5 Hz 관측기 7주기, 0.1 m/s에서 최대 15 cm 추정 |
| AMCL `transform_tolerance` 1.0 > costmap 0.7 > RPP 0.5 | `new_base_nav2_params.yaml` | 유지. 순서는 정상이며 AMCL 값은 계약 검사로 고정. RPP만 1.0으로 올리면 costmap 0.7에서 먼저 실패할 수 있다 |
| `controller_frequency` 10 Hz | 같은 파일 | 유지. Pi에서 4-8 Hz 경고가 있지만 정지 상수(`FINAL_STOP_SHORT_M` 등)는 10 Hz에서 측정했다. 낮추기보다 부하를 줄인다 |
| 직접 명령 재발행 20 Hz, 가드 주기 0.1 s, 입력 처리 20 ms | `restaurant_service.py` | 유지. smoother 0.5 s, 베이스 0.4 s timeout 안에 있다 |
| 정지 확인 0.08 s 또는 fresh 2표본, 정착 0.5+0.5 s | 같은 파일 | 유지(이미 10-29 수정) |
| collision monitor `source_timeout` 1.0 s | Nav2 파라미터 | 유지. LiDAR 3주기 이상 |
| RGB 참조 최대 나이 0.5 s, 최소 주기 0.2 s | `box_service.py` | 유지. 전체 재탐색(Pi 약 3.3 s) 결과는 무효지만 다음 영상부터 추적(약 0.11 s)이 이어진다 |
| 정지 자세 확인 `hold_s` 1.0 s, 각속도 0.02 rad/s | 주차 계약·`ParkingHold` | 유지. 최종 확인 계약이므로 바꾸지 않았다. 자이로 단발 잡음으로 1 s 확인이 자주 재시작될 수 있어 Pi odom 잡음 통계 측정 뒤 결정 |

## 4. 측정 후 결정할 후보(코드 변경 없음)

1. **후면 도킹 `rear docking swept footprint occupied` 대기**: 실시간 접근 제한시간(0.6/0.01+5 = 65 s)까지 기다린다.
   기억점이 원인이면 정지 중 사라지지 않는다. 10-20 s 상한 또는 "새 스캔 2장 연속으로 기억점만 차단" 시 종료를
   검토한다. 사람이 뒤에 서 있는 경우의 대기 의도와 함께 결정한다. 확인: run JSONL의
   `rear_box_waiting_for_input`과 `rear_collision_witness.source`.
2. **이동 중 RGB 참조 작업자**: 급수대 참조 작업자가 통로 이동 중에도 전체 탐색을 해 Pi CPU 약 1코어를 쓴다 [추정].
   관측 위치 도착 뒤에 처리 시작하면 부하가 줄지만 첫 일치가 늦어질 수 있다. `top -H`로 카메라 켬/끔 비교 뒤 결정.
3. **AMCL `update_min_d/a` 0.05**: Nav2 기본(0.25/0.2)보다 업데이트가 잦다. `AMCL_QUIET_MAX_*`와 함께 고정돼 있어
   AMCL CPU가 원인으로 확인될 때만 함께 바꾼다.
4. 정지 판정 각속도 불일치: 계약 0.02 rad/s, `_wait_for_input_recovery`·`_refresh_amcl_at_rest` 0.01 rad/s.
   한 표본만 필요해 실패보다 지연을 만든다.

## 5. Pi에서 확인할 기록

| 기록 | 확인 내용 |
|---|---|
| `table_approach_late_frame_skipped`, `straight_drive_stopped`의 `max_publish_gap_s` | 늦은 깊이 결과·명령 공백의 실제 빈도 |
| `nav2_transform_recovery_timeout`, journal의 AMCL "Message Filter dropping", "Control loop missed" | 102의 원인(AMCL 정체·CPU) |
| `box_status_history`의 `processing_duration_s`, `input_age_s` | 관측기 처리시간 |

## 부록. 주행 경로 값 전체 목록

줄 번호는 검토 기준 `ede1022`이다. 판단: 유지 / 변경(2절) / 측정 후(4절).

### A. Python 실행기

| 값 | 위치 | 역할 | 관련 주기 | 판단 |
|---|---|---|---|---|
| 0.5 s | `box_service.py` 1506, 2822-2869 | 깊이 status 신선도(관측·접근) | 관측기 5 Hz | 유지, 늦은 결과 처리만 변경 |
| 0.5 s | `box_service.py` 1071, 1076 | CameraInfo 생존(`_fresh_depth_frame_after`) | 깊이 30 Hz | 유지 |
| 1.5 s | `box_service.py` 123 `OBSERVER_SILENT_S` | 관측기 무응답·면 유지 상한 | 관측기 5 Hz | 유지 |
| 0.5 s / 0.2 s | `box_service.py` 148-149 | RGB 참조 최대 나이 / 최소 주기 | RGB 30 Hz, 추적 약 0.11 s | 유지 |
| 12 s / 30 s | `box_service.py` 1354 / 96 | 관측 창 / 카메라 준비 | Astra 첫 깊이 11.3 s | 유지 |
| 20 ms × 32 | `box_service.py` 2830-2834 | 늦은 status 전 콜백 처리 | — | 유지 |
| ≤ 50 ms | `box_service.py` 2856 | 영상 시각 TF 대기 | EKF 30 Hz | 유지(전용 TF 스레드로 실제 대기 가능) |
| 0.1 s | `restaurant_service.py` 145 `DIRECT_GUARD_PERIOD_S` | 가드 주기 | LiDAR 10 Hz | 유지 |
| 0.05 s | `restaurant_service.py` 1229 | 직접 명령 재발행 20 Hz | smoother 0.5 s, 베이스 0.4 s | 유지, 공백 기록 추가 |
| 20 ms | `restaurant_service.py` 1235 | 제어 주기 입력 처리 | — | 유지 |
| 0.5 + 0.5 s | `restaurant_service.py` 139, 142 | 정착 + 감속 유예 | — | 유지 |
| 0.08 s / 2표본 | `restaurant_service.py` 140-141 | 정지 확인 | odom 30 Hz | 유지 |
| 0.05 s | `restaurant_service.py` 137 `DIRECT_STOP_LATENCY_S` | 조기 정지 리드 | 미측정 | 유지 |
| 65 s | `restaurant_service.py` 1619-1621 | 실시간 접근 이동 제한(후면 대기도 제한) | — | 측정 후(4절 1) |
| 무한 | `restaurant_service.py` 1663-1664 | 깊이 입력 회복 대기(작업 제한시간만) | — | 유지 |
| 10 s | `restaurant_service.py` 2283 | 정지 자세 확인 제한 | — | 유지 |
| 1.0 s / 0.08 s | 계약 `hold_s` / 2285 | 최종 / 중간 정지 유지 | odom 30 Hz | 유지(3절) |
| 0.5 s | 계약 `sample_max_age_s` | 정지 확인 odom·TF 나이 | — | 유지 |
| 5 s | 계약 `observation_timeout_s` | 관측 제한 | — | 유지 |
| 1.5 s | `restaurant_service.py` 2271 | 정지 중 AMCL 갱신 대기 | — | 유지 |
| 0.5 s | `restaurant_service.py` 67 `AMCL_QUIET_MAX_TF_AGE_S` | 조용한 AMCL TF 나이 | — | 유지 |
| 8 s / 1회 | `corridor_route.py` 653, 713 | 입력 공백 회복 | — | 유지(102 재시도가 사용) |
| 20 s × 3 / 2 s | `corridor_route.py` 65-66, 77 | 경로 막힘 대기·확인 | — | 유지 |
| 2.5 s / 15 s | `corridor_route.py` 300, 310 | scan·odom / AMCL 신선도 | — | 유지 |
| 30 s | `restaurant_service.py` 96 | 운영자 연결 | — | 유지 |
| 1.0 s / 3회 | `restaurant_service.py` 91-92 | 후진 경로 재확인 | 전역 costmap 1 Hz | 유지 |
| 0.5 s | `rear_box_service.py` 430, 434; `rear_collision_profile.py` 21 | 스캔 나이 | LiDAR 10 Hz | 유지 |
| 5 s / 1 s, 8프레임 | `rear_box_service.py` 647, 693, 648 | 후면 면 관측 | LiDAR 7-10 Hz | 유지 |
| 0.35 s / 0.5 s | `surface_approach.py` 10-11 | 면 방향 필터 / 접근 시간 | — | 유지 |
| 1 mm | `box_service.py` `TABLE_FINE_MIN_PROGRESS_M` | 미세 보정 진전·최소 이동 | 0.01 m/s | 최소 이동에도 사용(2절) |

### B. Nav2·EKF·센서

| 항목 | 값 | 판단 |
|---|---|---|
| `controller_frequency` / `failure_tolerance` | 10 Hz / 2.0 s(NoValidControl에만 적용) | 유지 |
| progress checker | 10 s 안에 5 cm / 0.10 rad | 유지 |
| RPP `transform_tolerance` | 0.5 s(대기 시간) | 유지 |
| costmap `transform_tolerance` | 0.7 s | 유지 |
| local / global costmap 갱신 | 5 / 1 Hz | 유지 |
| behavior_server `transform_tolerance` | 0.1 s(시각 0 조회 timeout, 나이 기준 아님) | 유지 |
| velocity_smoother | 20 Hz, OPEN_LOOP, `velocity_timeout` 0.5 s | 유지 |
| collision_monitor | `source_timeout` 1.0 s, `transform_tolerance` 0.5 s, `stop_pub_timeout` 2.0 s | 유지 |
| AMCL | τ 1.0 s(계약 고정), `update_min_d/a` 0.05, 입자 500-2000, 빔 60 | 유지(4절 3) |
| EKF | 30 Hz, `sensor_timeout` 0.2 s | 유지 |
| 베이스 드라이버 | odom 50 Hz, `cmd_timeout` 0.4 s | 유지 |
| G4 LiDAR / Astra / 깊이 관측기 | 약 10 Hz / 30 Hz / 5 Hz | — |
