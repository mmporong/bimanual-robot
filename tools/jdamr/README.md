# JD-AMR 저장소 내부 실행 도구

JD-AMR SLAM·주행 코드의 관리 기준은 이 저장소의 `src/navigation/`이다. 이후 수정은
`bimanual-robot`의 `main`에서 작업 브랜치를 만든 뒤 진행하며, 별도 JD-AMR 저장소나
`$HOME/jdamr_data/.../tools`의 실행 사본을 원본으로 수정하지 않는다.

## 로컬 준비와 빌드

저장소 루트에서 실행한다.

```bash
tools/jdamr/runtime.py bootstrap
tools/jdamr/runtime.py build-pc
```

`bootstrap`은 지도·keepout·목적지 등록부·route 등 누락된 정적 자산만
`$HOME/jdamr_data/map_20260930_manual/aligned/`에 생성한다. 기존 파일이 다르면 유지한다.
번들 값으로 갱신해야 할 때만 `bootstrap --refresh`를 사용하며, 기존 파일은
`.runtime/jdamr/asset_backups/`에 먼저 보존된다.

`build-pc`는 기본적으로 실차 런타임 9개 ROS 패키지를
`.runtime/jdamr/pc_ws/`에 빌드한다. 편입된 모든 ROS 패키지가 필요하면 `--all`을
추가한다. 실행하지 않고 명령만 확인하려면 `--dry-run`을 사용한다.

## Pi 배포

```bash
tools/jdamr/runtime.py plan --host lim@192.168.0.159
tools/jdamr/runtime.py deploy-pi --host lim@192.168.0.159
tools/jdamr/runtime.py deploy-pi --host lim@192.168.0.159 --promote
```

`plan`은 PC 빌드와 Pi 배포 명령을 상태 변경 없이 출력한다. `deploy-pi`의 기본 동작은
Pi의 타임스탬프 stage에 전체 소스를 복사하고 빌드하는 것까지다. `--promote`를 붙여야
현재 `$HOME/jdamr_ws`에 반영한다. 승격은 검증된 stage의 `source/install/tools`를 그대로
보존하고, 안정 경로를 그 절대경로를 가리키는 링크로 교체한다. 기존 슬롯은 Pi의
`$HOME/jdamr_deploy/bimanual_backup_*`에 이동하며 중간 실패 시 원래 슬롯으로 복구한다.
live 경로에서 다시 빌드하지 않으며 서비스 재시작과 로봇 이동도 수행하지 않는다.
기존 원격 정적 자산과 카메라 설정은 기본적으로 유지한다. 번들 파일로 바꿀 때만
`--replace-assets`를 함께 지정하며, 이 파일 교체도 같은 승격 작업에서 백업·복구한다.

## 출발 도구

```bash
tools/jdamr/jdamr_depart.py status
tools/jdamr/jdamr_depart.py prepare
tools/jdamr/jdamr_depart.py session-start
tools/jdamr/jdamr_depart.py init
tools/jdamr/jdamr_depart.py go water_station
```

기본 지도는 `$HOME/jdamr_data/maps/current_manual_mapping.json`에서 선택한다.
다른 포인터는 전역 옵션 `--map-session <전체 경로>` 또는 `JDAMR_MAP_SESSION`으로 지정한다.
`prepare`는 등록 도크·급수대로 실행 자산을 만들고 참조 파일과 함께 Pi에 동기화한다.
`session-start`는 기존 Astra 드라이버 한 개에서 RGB와 깊이를 함께 받도록 설정하고,
Nav2를 이동 전 준비 상태로 기동한다. `init`은 등록 도크를 다시 가르치지 않고 위치를 초기화한다.
충전 중에는 `go`를 실행하지 않는다.

Pi 베이스 서비스는 `jdamr_cube_navigation/real_bringup_imu.launch.py`로 바퀴 속도와
바이어스 보정 자이로를 합친다. 원시 바퀴 odom은 `/wheel/odom`, 보정 IMU는 `/imu/data`,
제어에 쓰는 odom은 EKF의 `/odom`이다. 드라이버의 odom TF 발행은 끄고 EKF만
`odom→base_footprint`를 발행한다. 가속도는 융합하지 않으며 AMCL의 `map→odom` 역할은 유지한다.
기존 `jdamr_cube_bringup/real_bringup.launch.py`는 바퀴 odom 단독 구성으로 남아 있다.
IMU는 회전 추정을 보완하며, 면을 따라 움직인 실제 좌우 위치를 독립 측정하지 않는다.

실행기는 새 지도·도크·급수대 참조의 해시를 대조한 뒤
`src/navigation/scripts/map_20260930_manual/jdamr_depart.py`를 불러온다. 주행·위치초기화
알고리즘의 관리 대상은 이 파일 하나다. `status`는 상태 조회, `init`은 위치초기화,
`go`는 실물 이동 명령이므로 운영 상황에 맞게 사용한다.

기본 `go water_station`은 도크 → 등록 급수대 → 도크를 수행한다.
급수대를 목적지와 경유점으로 중복 방문하지 않는다. 이전 지도의 박스·테이블 경로는
`--legacy-map`으로만 불러온다. 등록 위치가 우선이다. 급수대는 정답 자세에서 촬영됐다는
`goal_binding`이 있는 RGB-D 참조에 한해 `reference_mode: rgbd`를 선택할 수 있다.
컵·물병으로 추정한 목표의 최근 유효 관측 중앙값을 좌우 제어에 연결하고, 깊이 면으로
앞 간격·방향을 보정한다. 최초 유효 관측부터 사용하므로 별도 프레임 대기를 추가하지 않는다.
정답 영상이 관측되는 재접근은 근접 책상 경계 검출을 기다리지 않고 기존 보호 직선 제어로
요청한 거리만큼 후진한 뒤 새 깊이 면을 관측한다. 충돌 감시와 입력·명령 timeout은 유지한다.
목표 연결이 없는 기존 설정의 `rgbd_auxiliary` 모드는 진단용이며, 1번 테이블의
`map_pose_only`는 영상 기준을 사용하지 않는다. 목표는 차체 앞끝 1 cm와 좌우 ±5 mm이고,
영상 제어 적용 뒤 실제 반복 정확도는 아직 확인되지 않았다. `NOT_VALIDATED` 상태는 유지한다.
등록된 도크 방향이 `reverse`이면
고정 도크 앞쪽에서 후진 진입하며, 테이블 주차 목표로 도크 위치를 다시 등록하지 않는다.

기본 책상 주차와 박스 주차는 주차 계약의 `target_front_gap_m`(현재 0.01 m)을
사용한다. 간격을 바꿀 때는 `go water_station --final-gap-m 0.03`처럼 지정한다.
책상 면 정렬 뒤에는 중간 5 cm 정지를 끼우지 않고 최종 간격까지 직진한다.
유효한 관측은 관측 시각의 TF로 갱신하며, 근접 가림 판정 뒤에는 마지막 면을 동결하고
odometry로 남은 거리를 계산한다. 이 경우 최종 값은 새 깊이 실측이 아닌 내부 추정이다.
준비된 세션에서는 상주 실행기에 요청을 제출하고 첫 상태를 즉시 조회한다.
로그의 `SUBMITTED`는 요청 제출이며 실제 이동 시작을 뜻하지 않는다.
`request_submit_s`와 첫 이벤트 경과시간으로 통신·실행기 대기를 구분한다.

이동 전 지도 수신 실패는 위치추정·센서·화면을 유지하고 실행기만 한 번 재시작한다.
재시도도 실패하면 원인을 보고하고 종료한다. `recover` 전체 복구는 별도 명령이며,
앞서 사용하던 화면만 복원하고 화면 실패가 Pi 복구를 중단시키지 않는다.
표시 중계도 `JDAMR_HOST`를 사용한다. PC 녹화가 필요하면
`display-start --record-diagnostics` 또는 `record-start`를 사용한다.

필요한 경로와 접속 정보는 환경변수로 바꿀 수 있다.

| 환경변수 | 기본값 | 용도 |
| --- | --- | --- |
| `JDAMR_HOST` | `lim@192.168.0.159` | Pi SSH 대상 |
| `JDAMR_PI_USER` | `JDAMR_HOST`의 사용자 | Pi systemd 사용자 |
| `JDAMR_PI_HOME` | `/home/lim` | Pi 홈 경로 |
| `JDAMR_PI_WS` | `$JDAMR_PI_HOME/jdamr_ws` | Pi ROS 워크스페이스 |
| `JDAMR_DATA_ROOT` | `$HOME/jdamr_data` | PC 지도·등록부·실행 상태 루트 |
| `JDAMR_MAP_SESSION` | `$HOME/jdamr_data/maps/current_manual_mapping.json` | 현재 수동 지도·도크·급수대 참조 포인터 |
| `JDAMR_RUNTIME_ROOT` | `<저장소>/.runtime/jdamr` | 로컬 빌드·백업 루트 |
| `JDAMR_PC_WS` | `$JDAMR_RUNTIME_ROOT/pc_ws` | PC ROS 빌드 워크스페이스 |

`state.json`, `rviz_initialpose.json`, 실행 로그, bag과 원시 스캔은 저장소 번들에 포함하지
않는다. 초기화와 배포는 기존 상태 및 목적지 등록부를 자동으로 덮어쓰지 않는다.
