# 새 지도 운영 도구 사본 (2026-09-30 14:10 이후)

이 폴더는 양팔 저장소가 관리하는 현행 출발 도구 소스다. 실행 진입점은 저장소의 `tools/jdamr/jdamr_depart.py`이며, 지도·도크·실행 상태는 `$HOME/jdamr_data/`에 분리한다. 외부 데이터 폴더의 옛 `tools/` 사본을 수정 기준으로 사용하지 않는다. 아래 표는 가져온 운영 기능 기록이며 현행 설치·기본값은 [통합 사용법](../../../../docs/guides/JDAMR_RUNTIME.md)과 CLI·config를 따른다.

| 파일 | 역할 | 로봇 이동 |
|---|---|---|
| `jdamr_depart.py` | `aligned/` 지도와 등록된 도크를 사용하는 출발 도구. 기본 `go table_0N`은 책상으로 이동해 계약의 1 cm까지 주차한 뒤 도크로 복귀한다. 물 받는 곳을 포함할 때는 `--with-via`를 지정한다. 상주 실행기 `jdamr-box-executor`에 요청을 제출하며 실행기가 없으면 한 번 기동한다. 종료 코드와 확인된 도크 도착이 모두 성공해야 사이클을 완료로 처리한다. 이동 전 지도 수신 실패는 현재 위치추정을 유지한 채 실행기만 한 번 재시작하며, 재시도도 실패하면 종료한다. `init`은 등록된 도크에서 위치를 정합하고, 다른 곳에 놓였다면 `init --click`을 사용한다. `recover --seed X Y YAW_DEG [--local-only]`는 별도로 요청하는 전체 복구다. `--resume-at-observation`·`--resume-parked-log`는 남은 구간을 이어가고 `--dock-only`는 도크 복귀만 수행한다. 여러 테이블을 지정하면 도크에서 init 후 다음 사이클을 시작하며 기본 대기는 0초다. `display-start`는 화면만 켜고, PC 녹화는 `display-start --record-diagnostics` 또는 `record-start`로 선택한다. `go --record-diagnostics`는 Pi bag·UDP·부하 기록을 켜고 종료 후 PC로 복사한다. 실행기 코드 변경은 배포 후 실행기를 재시작해 적용한다 | `go`만 |
| `mapping_display_relay.py` | 수동 매핑 중 PC RViz에 실시간 `/map`을 보이게 한 임시 중계. 설치본 `rviz_display_relay`에 `/map`만 더해 ssh 표준입력으로 파이에서 실행한다(파이 쪽은 구독만) | 없음 |
| `dds_probe.py` | 파이에서 새 참가자로 `/map`·`/keepout_filter_mask`·`/tf_static`의 `laser_link`·`/scan`을 실행기와 같은 30 s 안에 받는지 한 줄로 보고한다(구독만, 박스 관측 상태는 수만 기록) | 없음 |
| `box_escape_once.py` | 박스 앞에서 멈춘 로봇에 실행기 `_leave_parked_pose` 후진 이탈만 한 번 실행한다. 앞면은 로봇 방향+측정 거리 또는 로그의 관측 면(단위 법선만 허용). 선택적으로 정지 확인 대기, `--stop-id`로 로그 표지 지정. SIGINT·SIGTERM·SIGHUP이면 후진을 취소한다 | 후진 이탈 |
| `test_jdamr_depart_flow.py` | `jdamr_depart.py`의 출발·복구 흐름을 임시 번들 데이터와 모의 SSH로 확인한다. 실물·외부 지도 폴더 없이 실행한다 | 없음 |
| `analyze_run.py` | 한 실행의 `cycle_events.jsonl`(+ PC 중계 bag)을 단계별 시간·재계획 수·회전 방향 전환·map→odom 보정으로 요약한다 | 없음 |
| `udp_drop_monitor.py` | 파이에서 5 s마다 UDP 수신 넘침을 프로세스별로 기록한다. `go --record-diagnostics`가 실행 동안 transient 유닛으로 띄우고 끝나면 멈춘다(`runs/<run>/udp_drops.jsonl`) | 없음 |
| `imu_axes.py` | `/odom`·`/imu/data_raw`가 든 bag에서 IMU 보드 축을 정한다. z는 자이로와 바퀴 회전율의 기울기, x·y는 바퀴 전진 가속·원심 가속에 대한 회귀. 2026-10-01 결과(보드 z 아래·y 앞·x 왼쪽)가 URDF `imu_joint`다 | 없음 |
| `rotation_truth.py` | 정지 구간으로 감싼 회전(또는 `--turnarounds` 왕복 회전)을 스캔끼리 맞춘 회전량 기준으로 odom·자이로·EKF 회전 오차를 잰다(`--ekf=<bag>`) | 없음 |
| `ekf_replay.sh` | bag의 `/odom`·`/imu/data_raw`를 `imu_bias_relay` + robot_localization에 PC 도메인 89로 흘려 `/odometry/filtered`를 기록한다 | 없음 |
| `imu_check.sh` | 한 run의 `onboard_bag`에 `imu_axes.py` → `ekf_replay.sh` → `rotation_truth.py`를 차례로 돌려 `runs/<run>/imu_check/`에 남긴다. URDF를 파이에 반영한 뒤에는 `ros2 run tf2_ros tf2_echo base_footprint imu_link`로 RPY (180°, 0°, 90°)를 확인한다 | 없음 |
| `test_imu_axes.py` | 보드를 z 아래·y 앞·x 왼쪽으로 단 합성 신호에서 `imu_axes.analyse`가 축을 되찾는지 확인한다 | 없음 |
| `compare_runs.py` | 여러 run 폴더를 같은 기준(첫 목표 수락부터)으로 비교한다: 사이클, 대기점 도착 방향·Spin, 대기점 도착 → 도크 후진, 이탈 → 도크 후진 시간과 odom 회전량, 도크 끝 방향·확인, 면 정렬 시간, 주차. run 폴더의 `run_config.json`(계획기 등 설정)을 함께 적는다 | 없음 |
| `plan_compare.py` | 파이의 살아 있는 세션에서 이동 구간 5개(도크→물 받는 곳, 물 받는 곳→각 테이블, 각 테이블→도크 대기점)를 `GridBased`와 `Lattice`로 계획만 해 길이 비·계획 시간·끝 오차를 비교한다(이동 없음) | 없음 |
| `table_click_capture.py` | PC 전용 도메인 78에서 RViz 2D Pose Estimate 클릭을 목적지 후보로 기록하고 지도 축 0/90/180/270°로 고정해 표시한다 | 없음 |

사용 순서와 결과는 `jdamr_cube_navigation/evaluation/20260929_PARKING_FAILURES.md` §18.
