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
tools/jdamr/jdamr_depart.py init
tools/jdamr/jdamr_depart.py go table_01
```

실행기는 누락된 정적 자산을 준비한 뒤
`src/navigation/scripts/map_20260930_manual/jdamr_depart.py`를 불러온다. 주행·위치초기화
알고리즘의 관리 대상은 이 파일 하나다. `status`는 상태 조회, `init`은 위치초기화,
`go`는 실물 이동 명령이므로 운영 상황에 맞게 사용한다.

필요한 경로와 접속 정보는 환경변수로 바꿀 수 있다.

| 환경변수 | 기본값 | 용도 |
| --- | --- | --- |
| `JDAMR_HOST` | `lim@192.168.0.159` | Pi SSH 대상 |
| `JDAMR_PI_USER` | `JDAMR_HOST`의 사용자 | Pi systemd 사용자 |
| `JDAMR_PI_HOME` | `/home/lim` | Pi 홈 경로 |
| `JDAMR_PI_WS` | `$JDAMR_PI_HOME/jdamr_ws` | Pi ROS 워크스페이스 |
| `JDAMR_DATA_ROOT` | `$HOME/jdamr_data` | PC 지도·등록부·실행 상태 루트 |
| `JDAMR_RUNTIME_ROOT` | `<저장소>/.runtime/jdamr` | 로컬 빌드·백업 루트 |
| `JDAMR_PC_WS` | `$JDAMR_RUNTIME_ROOT/pc_ws` | PC ROS 빌드 워크스페이스 |

`state.json`, `rviz_initialpose.json`, 실행 로그, bag과 원시 스캔은 저장소 번들에 포함하지
않는다. 초기화와 배포는 기존 상태 및 목적지 등록부를 자동으로 덮어쓰지 않는다.
