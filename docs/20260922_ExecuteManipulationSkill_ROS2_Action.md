# ExecuteManipulationSkill ROS 2 Action 계약과 mock 검증

## 1. 목적과 현재 범위

웹 관제의 `manipulate` 명령을 PLANNED·ACT·HYBRID 실행기에 같은 형식으로 전달하기 위한 ROS 2
Action 계약이다. 현재 구현은 **시뮬레이션 전용 mock**이다. 모터·카메라·Isaac Sim·하드웨어 bridge를
열지 않으며 `dry_run=false` 요청을 거부한다.

구현 파일은 다음과 같다.

- Action IDL: `src/hold_flow_interfaces/action/ExecuteManipulationSkill.action`
- 요청 검증·관제 명령 변환: `src/hold_flow_mission/hold_flow_mission/contract.py`
- mock Action server: `src/hold_flow_mission/hold_flow_mission/mock_action_server.py`
- Action client: `src/hold_flow_mission/hold_flow_mission/action_client.py`
- 관제 명령 예제: `config/simulation/manipulation_command_demo.json`
- 계약 회귀 테스트: `src/hold_flow_mission/test/test_contract.py`

## 2. 요청 계약

Goal은 주문 추적 ID와 조작 실행 계약을 함께 보낸다.

| 필드 | 의미 | 검증 |
|---|---|---|
| `request_id` | 웹·API 요청 추적 ID | 1~128자 식별자 |
| `mission_id` | 관제 Mission ID | 1~128자 식별자 |
| `order_id` | 주문 ID | 1~128자 식별자 |
| `skill_id` | 실행할 조작 단위 | 예: `grasp_cup`, `water_kitchen_full` |
| `control_strategy` | 전체 실행 전략 | `PLANNED_ALL`, `ACT_ALL`, `HYBRID` |
| `phase_ids` | 순서가 있는 phase 목록 | 비어 있거나 중복되면 거부 |
| `phase_backends` | phase별 실행기 | `PLANNED` 또는 `ACT` |
| `checkpoint_ids` | phase별 ACT checkpoint | ACT는 필수, PLANNED는 빈 문자열 |
| `timeout_sec` | Action 전체 제한 시간 | 0보다 큰 유한 실수 |
| `dry_run` | 실물 접근 금지 표시 | mock server는 `true`만 수락 |

세 배열은 인덱스가 하나의 phase 계약을 이룬다. 길이가 다르면 요청을 거부한다.

```text
phase_ids[i]
  + phase_backends[i]
  + checkpoint_ids[i]
```

전략과 backend 조합도 고정한다.

| 전략 | 허용 조합 |
|---|---|
| `PLANNED_ALL` | 모든 phase가 `PLANNED`, checkpoint 없음 |
| `ACT_ALL` | 모든 phase가 `ACT`, 각 checkpoint 필수 |
| `HYBRID` | 한 요청 안에 `PLANNED`와 `ACT`가 모두 존재 |

## 3. feedback과 결과

feedback은 현재 phase, 0부터 시작하는 phase 인덱스, 전체 phase 수, 진행률, 활성 backend와
checkpoint를 보낸다. 완료 feedback은 `progress=1.0`, `status=SUCCEEDED`다.

Result는 다음 정보를 보존한다.

- 요청·Mission·주문·skill ID
- `control_strategy`, phase ID·backend·checkpoint
- ROS 시각의 시작·종료 timestamp
- `success`, `canceled`, `timed_out`
- `failure_stage`, `failure_code`
- `human_intervention_count`

Action 상태와 Result를 함께 해석한다.

| 경우 | ROS Action 상태 | Result |
|---|---|---|
| 정상 완료 | `SUCCEEDED` | `success=true` |
| 실행 실패 | `ABORTED` | `failure_stage`, `failure_code` |
| timeout | `ABORTED` | `timed_out=true`, `failure_code=TIMEOUT` |
| 취소 | `CANCELED` | `canceled=true`, `failure_code=CANCELED` |

## 4. 빌드와 실행

로컬 사용자 Python이 ROS Jazzy의 Python 3.12보다 먼저 잡히는 환경에서는 CMake가 잘못된
인터프리터를 선택할 수 있다. 아래처럼 시스템 Python을 명시한다.

```bash
cd "$HOME/bimanual-robot"
source /opt/ros/jazzy/setup.bash
PATH=/usr/bin:/bin:$PATH colcon build \
  --packages-select hold_flow_interfaces hold_flow_mission \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3.12
source install/setup.bash
```

터미널 1에서 mock server를 시작한다.

```bash
cd "$HOME/bimanual-robot"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run hold_flow_mission mock_manipulation_server \
  --ros-args -p phase_delay_sec:=0.2
```

터미널 2에서 웹 관제가 내보내는 것과 같은 `manipulate` 명령을 Action으로 변환해 보낸다.

```bash
cd "$HOME/bimanual-robot"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run hold_flow_mission manipulation_action_client \
  --command-json config/simulation/manipulation_command_demo.json \
  --request-id REQ-DEMO-001 \
  --mission-id MIS-DEMO-001 \
  --timeout-sec 2.0
```

취소·timeout 검증은 같은 client로 수행한다.

```bash
# 실행 중 취소
ros2 run hold_flow_mission manipulation_action_client \
  --command-json config/simulation/manipulation_command_demo.json \
  --request-id REQ-CANCEL --mission-id MIS-CANCEL \
  --timeout-sec 2.0 --cancel-after-sec 0.03

# phase 실행보다 짧은 timeout
ros2 run hold_flow_mission manipulation_action_client \
  --command-json config/simulation/manipulation_command_demo.json \
  --request-id REQ-TIMEOUT --mission-id MIS-TIMEOUT \
  --timeout-sec 0.03
```

## 5. 이번 검증 결과

ROS 2 Jazzy에서 IDL 생성과 두 패키지 빌드를 통과했다. `/execute_manipulation_skill`에 실제 Action
client를 연결해 다음 세 경로를 확인했다.

- 정상: `SUCCEEDED`, `success=true`, 시작·완료 feedback 수신
- timeout: `ABORTED`, `timed_out=true`, `failure_stage=GRASP_CUP`
- 취소: `CANCELED`, `canceled=true`, `failure_stage=GRASP_CUP`
- 실물 표시 요청: `dry_run=false` Goal 거부

세 결과 모두 request·mission·order ID와 `PLANNED_ALL / PLANNED` 실행 정보를 유지했다. 계약
단위 테스트 7개와 `colcon test`도 통과했다. 이 결과는 ROS 메시지 왕복과 상태 전이 검증이며 조작
성공이나 실물 안전 검증이 아니다.

## 6. 웹 관제 연결 결과

`tools/service_execution_backend.py`가 두 실행 모드를 제공한다.

| 모드 | 주행·충전 | 조작 |
|---|---|---|
| `immediate` | 즉시 성공 mock | 즉시 성공 mock |
| `ros2-mock` | 즉시 성공 mock | `/execute_manipulation_skill` 결과 대기 |
| `ros2-planned-artifact` | 즉시 성공 mock | 해시 검증 Isaac Sim 산출물의 phase 판정 |
| `ros2-planned-session` | 즉시 성공 mock | Unix socket의 상태 보존 executor session |

`ros2-mock` 통합 시험에서 냉수 주문 한 건이 9개 조작 Action을 모두 `SUCCEEDED`로 마치고,
손님 테이블 서빙 뒤 충전소로 복귀해 `IDLE_AT_DOCK`에 도달했다. Action 실행 중 웹에서 주문을
취소했을 때 활성 Goal도 `CANCELED`로 끝났고, 관제 취소 뒤 돌아온 결과는 `superseded=true`로
SQLite에 저장됐다. 이 결과가 이후 phase를 실행하지 않는 것도 확인했다.

mock phase 지연보다 짧은 0.03초 timeout 시험에서는 `ALIGN_KITCHEN`이 두 번 `ABORTED/TIMEOUT`으로
끝났다. 관제 설정의 재시도 횟수를 소진한 뒤 주문은 `ALIGN_KITCHEN:TIMEOUT`으로 실패했고 두
Action 결과가 SQLite command payload에 각각 남았다.

## 7. PLANNED 산출물 Action backend

`planned_artifact_server`는 성공한 Isaac Sim `result.json`을 읽고 다음 항목을 다시 검사한다.

- `task_pass`, 연속 sequence, 지면 접촉, 물붓기·상판 운반·재파지 성공
- 숨은 물체 고정과 실물 접근을 사용하지 않았다는 기록
- 실행 당시 모든 도구와 입력 파일의 SHA-256
- 9개 조작 phase에 대응하는 simulator sample

현재 코드와 해시가 일치하는 `cinematic_wide01/result.json`으로 1번 테이블 냉수 주문을 시험했다.
9개 조작 Action이 같은 artifact SHA를 SQLite에 남기고 주문 성공·충전소 복귀까지 끝났다.
2번 테이블 요청은 지원 범위 밖이므로 두 번 재시도 후
`ALIGN_KITCHEN:ARTIFACT_TABLE_UNSUPPORTED`로 실패했다.

이 backend는 기록된 성공 근거를 관제 계약으로 재생한다. Isaac Sim을 새로 실행하거나 물리 상태를
이어 가지 않는다. 현재 `water_service_mission.py`는 전체 미션을 한 프로세스에서 실행하므로,
phase마다 새로 실행하면 같은 물 서빙을 반복하게 된다. 다음 구현은 Isaac 월드를 한 번만 만들고
Action phase 사이에서 상태를 보존하는 장기 실행 executor다.

```bash
cd "$HOME/bimanual-robot"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ARTIFACT_ROOT="/data/$USER/robot-artifacts"

ros2 run hold_flow_mission planned_artifact_server --ros-args \
  -p result_path:="$ARTIFACT_ROOT/restaurant/cinematic_wide01/result.json" \
  -p repo_root:="$HOME/bimanual-robot"
```

별도 터미널에서 관제를 시작한다.

```bash
cd "$HOME/bimanual-robot"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3.12 tools/service_order_server.py \
  --backend ros2-planned-artifact \
  --port 8767 \
  --state-dir "$HOME/.local/state/bimanual-robot/planned-artifact"
```

## 8. Python ABI 분리와 상태 보존 executor IPC

로컬 Isaac Sim 5.1 환경은 Python 3.11이고 ROS 2 Jazzy의 `rclpy` 확장은 Python 3.12용이다.
Isaac Python에서 `rclpy`를 로드하면 `_rclpy_pybind11` ABI 오류가 발생한다. 한 프로세스에 두
runtime을 억지로 넣지 않고 다음처럼 분리했다.

```text
웹 관제 (Python 3.12)
  -> ExecuteManipulationSkill Action
  -> planned_ipc_server (Python 3.12 / rclpy)
  -> Unix socket · planned_executor_ipc_v1
  -> 장기 실행 executor (Python 3.11 / Isaac Sim)
```

4B.1의 CPU 검증에는 `planned_executor_ipc_mock.py`를 사용한다. 같은 `mission_id`에서 9개 phase의
순서를 기억하고 동일 ID의 완료 결과를 재생한다. 이 mock의 동시 실행 제한은 mission 내부에만
적용되므로 실제 월드 실행기로 사용하지 않는다. 실제 Isaac 연결은 다음 9절의 단일-world executor를 사용한다.

세 터미널에서 아래 순서로 실행한다.

```bash
cd "$HOME/bimanual-robot"
PYTHONPATH="$HOME/bimanual-robot/src/hold_flow_mission" \
  "/data/$USER/conda-envs/leisaac/bin/python" \
  tools/planned_executor_ipc_mock.py \
  --socket /tmp/hold-flow-planned-executor.sock
```

```bash
cd "$HOME/bimanual-robot"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run hold_flow_mission planned_ipc_server --ros-args \
  -p socket_path:=/tmp/hold-flow-planned-executor.sock
```

```bash
cd "$HOME/bimanual-robot"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3.12 tools/service_order_server.py \
  --backend ros2-planned-session \
  --port 8768 \
  --state-dir "$HOME/.local/state/bimanual-robot/planned-session"
```

1번 테이블 냉수 주문에서 9개 phase가 한 session으로 완료되고 충전소까지 복귀했다. 실행 중
취소는 executor의 `CANCELED`, Action의 `CANCELED`, 관제 주문의 `CANCELED`로 이어졌고 늦게 온
결과는 `superseded=true`로 SQLite에 저장됐다. 이 검증에서 executor의
`simulator_accessed`는 false다.

## 9. Isaac 물리 executor (4B.2)

`tools/simulate_restaurant_mobile.py --executor-socket`은 물리 월드를 한 번 초기화하고
웹에서 전달되는 조작 phase를 기다린다. 요청을 기다리는 동안 `world.step()`을 호출하지 않아
컵·병·물의 시뮬레이션 상태와 시간이 유지된다. socket thread는 요청만 등록하며 관절·USD·PhysX는
시뮬레이션 thread만 다룬다. 벽시계 기준 장시간 정지의 실물 안정성을 검증하는 방식은 아니다.

| Action phase | 실제 시뮬레이션 동작과 다음 단계 조건 |
|---|---|
| ALIGN_KITCHEN | 주방에 배치된 기체·물체 초기 안정화. 충전소에서 오는 주행은 포함하지 않음 |
| GRASP_CUP | 왼손 컵 파지·상승, 접촉과 상승 높이 유지 확인 |
| GRASP_BOTTLE | 오른손 병 파지·상승, 접촉과 상승 높이 유지 확인 |
| POUR | 병 기울이기·복원. 전체 입자 중 컵 수용 60% 이상, 외부 유출 5% 이하 |
| RETURN_BOTTLE | 병을 주방에 내려놓고 손가락 접촉 해제·테이블 지지 확인 |
| PLACE_DECK | 선택된 상판 영역에 컵을 놓고 지지·해제·팔 이격 확인 |
| ALIGN_TABLE | **임시로 후진·1번 테이블까지 바퀴 주행·접근·정지를 모두 포함** |
| REGRASP_CUP | 상판 컵 재파지·상승과 접촉 유지 확인 |
| SERVE | 손님 테이블에 놓고 지지·해제 확인, 전체 접지 검사와 결과 저장 후 성공 반환 |

관제 `NAVIGATE_TABLE`은 아직 mock이며 실제 주행은 뒤의 `ALIGN_TABLE`에서 일어난다.
따라서 관제 지도 위치는 논리 위치이지 실시간 시뮬레이터 측정 위치가 아니다. 이를 Nav2 또는
별도 navigation adapter로 옮기는 작업이 남아 있다. 충전소 출발·복귀·충전도 mock이다.

지원 범위는 **1번 테이블·냉수 주문 한 건**이다. 온도 물리 모델이나 두 병 중 선택 기능은 없다.
동일 프로세스의 다른 mission은 `WORLD_OWNED`로 거부한다. 완료·취소·timeout 뒤에는 새 월드가
필요하며, 서버 snapshot만 복원해 물리 상태를 복구할 수 없다. 반복 주문 자동 reset은 미구현이다.
현재 입력 `plate_input10`은 과거 접촉 프록시로, 최신 순정 왼손의 물리 검증을 대신하지 않는다.

### 실행

설치된 로컬 Isaac Sim 5.1/Python 3.11과 ROS Jazzy/Python 3.12를 분리한다. 아래에서
`plate_input10`은 Git에 없는 로컬 시뮬레이션 입력이다. 새 기기는 입력 자산을 별도로 준비해야 한다.
기존 경로의 socket을 자동 삭제하지 않으므로 다른 executor가 쓰는 경로를 재사용하지 않는다.

터미널 1 — Isaac 월드:

```bash
cd "$HOME/bimanual-robot"
export HF_EXECUTOR_SOCKET="/tmp/hold-flow-isaac-$(id -u).sock"
PYTHONPATH="$PWD/src/hold_flow_mission" \
  "/data/$USER/conda-envs/leisaac/bin/python" tools/simulate_restaurant_mobile.py \
  --source-dir "/data/$USER/robot-artifacts/restaurant/plate_input10" \
  --output-dir "/data/$USER/robot-artifacts/restaurant/ipc_$(date +%Y%m%d_%H%M%S)" \
  --headless --mode water-service --duration 320 \
  --executor-socket "$HF_EXECUTOR_SOCKET" --executor-idle-timeout 180
```

터미널 2 — ROS Action bridge:

```bash
cd "$HOME/bimanual-robot"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=73
export PYTHONPATH="$PWD/src/hold_flow_mission:$PYTHONPATH"
export HF_EXECUTOR_SOCKET="/tmp/hold-flow-isaac-$(id -u).sock"
python3 -m hold_flow_mission.planned_ipc_server \
  --ros-args -p socket_path:="$HF_EXECUTOR_SOCKET"
```

터미널 3 — 웹 관제:

```bash
cd "$HOME/bimanual-robot"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=73
export PYTHONPATH="$PWD/src/hold_flow_mission:$PYTHONPATH"
python3 tools/service_order_server.py --backend ros2-planned-session \
  --port 8768 --auto-step-s 0.1 --manipulation-timeout-s 600 \
  --state-dir "$HOME/.local/state/bimanual-robot/isaac-$(date +%Y%m%d_%H%M%S)"
```

socket이 준비된 뒤 `http://127.0.0.1:8768/`에서 **1번 테이블·냉수** 주문을 넣는다.
600초는 phase별 벽시계 timeout이고, 320초는 전체 시뮬레이션 시간 상한이다.
실행 중 수동 phase 전진 API는 차단한다. 영상은 기본으로 저장하지 않는다.

### 증거와 중단

2026-09-22 로컬 `ipc_phase04` 실행은 웹 주문 `ISAAC-PHASE-004`의 9개 Action과 전체 물리 판정을
통과했다. 시뮬레이션 282.942초, 컵 입자 642/918개, 운반 중 추가 입자 손실 0개였다.
`result.json`의 입력·도구 SHA-256은 검증 시점 코드와 일치했다. 웹 관제의 마지막
`IDLE_AT_DOCK`는 논리 상태이며 이 실행에서 충전소까지 물리 주행한 근거가 아니다.
영속 백업은 로컬 `/data/$USER/robot-artifacts/restaurant/ipc_phase04/`에 있다.

- `executor_phases.jsonl`: 요청·미션·주문 ID, 완료 phase, 관측값, 성공/실패 코드.
- `result.json`: 전체 접촉·입자·주행 결과와 입력·실행 코드 해시. 최종 Action 성공은 이 파일 저장 뒤 반환한다.
- 웹 상태 디렉터리의 SQLite `commands.payload_json`: Action result의 `message`에 executor 응답 JSON을 보존한다.
- `simulator_accessed=true`, `executor_kind=isaac_physics`는 mock 결과와 구분한다. 웹은 첫 결과를 받기 전에는 executor 종류를 미확인으로 표시한다.
- 취소 요청 수락과 정지 확인은 다르다. `CANCELING`은 요청 접수, 최종 결과의 `stop_confirmed=true`가 시뮬레이션 루프 종료 확인이다.
- timeout·접촉 실패 뒤에는 같은 월드를 계속 실행하지 않는다. 응답 유실 시 bridge가 취소를 요청하며, 정지 확인이 없으면 성공으로 간주하지 않는다.
- 실행 코드가 변경되었으므로 이전 `cinematic_wide01`의 코드 해시 재생은 현재 checkout에서 불일치할 수 있다. 옛 해시를 수정하지 말고 해당 코드 snapshot 또는 새 실행 근거를 사용한다.

## 10. 다음 연결

1. 완료: 웹 관제 runtime이 조작 Action 결과를 기다리고 성공일 때만 다음 phase로 전이한다.
2. 완료: `ABORTED`, `CANCELED`, timeout과 `superseded`를 SQLite command payload에 저장한다.
3. 완료: 검증된 `water_service_mission.py` 결과를 해시·phase 근거가 있는 PLANNED 산출물 backend로 연결했다.
4. 완료: Python ABI를 분리한 Unix socket 계약에서 phase 순서·중복·취소·timeout과 session 유지를 검증했다.
5. 구현: `water_service_mission.py` 루프를 phase 경계에서 멈추고 재개해 IPC session과 Isaac 월드를 연결했다.
6. Isaac PLANNED executor의 취소·timeout 이후에는 새 월드 초기화가 필요하다. 이후 ACT·hardware bridge의 중단 계약을 별도로 연결한다.
7. ACT backend와 checkpoint loader를 추가해 같은 Action 계약으로 세 전략을 비교한다.

현재는 한 관제 명령을 한 phase Action으로 바꾸고 mock·검증 산출물·단일 Isaac 월드의 결과를
소비한다. 여러 phase를 묶은 HYBRID Goal은 계약 검증 대상이지만 PLANNED↔ACT 전환 실행기는
아직 없다. 충전소 왕복, 여러 테이블·병, 반복 주문 물리 초기화를 다음 단계로 진행한다.
