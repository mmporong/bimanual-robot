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

## 6. 다음 연결

1. 웹 관제 runtime이 `manipulate` 명령을 즉시 성공시키는 대신 Action client 결과를 기다리게 한다.
2. 성공이면 `MissionController.advance(success=true)`를 호출한다.
3. `ABORTED`, `CANCELED`, timeout이면 `failure_code`를 그대로 관제 이벤트와 SQLite에 저장한다.
4. 취소 요청은 mock에서 끝내지 않고 이후 PLANNED·ACT executor와 hardware bridge까지 전달한다.
5. 검증된 `water_service_mission.py` phase를 첫 PLANNED backend로 연결한다.
6. 그다음 ACT backend와 checkpoint loader를 추가해 같은 Action 계약으로 세 전략을 비교한다.

현재는 한 관제 명령을 한 phase Action으로 바꾸는 adapter까지 구현됐다. 여러 phase를 묶은
HYBRID Goal은 계약 검증 대상이지만 실제 PLANNED↔ACT lease 전환 실행기는 아직 없다.
