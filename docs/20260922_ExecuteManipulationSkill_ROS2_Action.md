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

## 8. 다음 연결

1. 완료: 웹 관제 runtime이 조작 Action 결과를 기다리고 성공일 때만 다음 phase로 전이한다.
2. 완료: `ABORTED`, `CANCELED`, timeout과 `superseded`를 SQLite command payload에 저장한다.
3. 완료: 검증된 `water_service_mission.py` 결과를 해시·phase 근거가 있는 PLANNED 산출물 backend로 연결했다.
4. Isaac 월드를 한 번만 띄우고 phase 사이의 물리 상태를 유지하는 PLANNED executor를 만든다.
5. 취소 요청을 PLANNED·ACT executor와 이후 hardware bridge까지 전달한다.
6. ACT backend와 checkpoint loader를 추가해 같은 Action 계약으로 세 전략을 비교한다.

현재는 한 관제 명령을 한 phase Action으로 바꾸고 mock 또는 검증 산출물 결과를 소비한다.
여러 phase를 묶은 HYBRID Goal은 계약 검증 대상이지만, 실시간 PLANNED executor와
PLANNED↔ACT lease 전환 실행기는 아직 없다.
