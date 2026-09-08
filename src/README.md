# src 구현 계약

이 디렉터리는 물 서빙 로봇의 실행 코드를 담는다. 현행 태스크·정책·구현 순서는
[2026-09-07 물 서빙 로봇 회의 결정](../docs/20260907_물서빙로봇_회의결정과_실행범위.md)을
따른다. 과거 붓기안의 MoveIt·로드셀 중심 구조는 현행 구현 기준이 아니다.

현재 실제 ROS 2 패키지는 `hold_flow_description`뿐이다. 아래 나머지 패키지는 책임과
인터페이스를 먼저 고정한 **구현 예정 경계**이며, 빈 패키지를 완료물처럼 만들지 않는다.

## 패키지별 언어와 책임

| 패키지 | 상태 | 주 언어·기술 | 현행 책임 | 착수 조건 |
|---|---|---|---|---|
| `hold_flow_interfaces` | 예정 | ROS IDL | 미션·정렬·policy·물 양·로그 계약 | policy 번호·상태명 확정 |
| `hold_flow_description` | v0.3 정적 검증 완료 | xacro·YAML·URDF | 300 mm·720 mm 모바일 베이스, 혼합 그리퍼 SO-101×2, Astra S·LDS-03 TF | 실측값 반영과 Isaac 동역학 검증 |
| `hold_flow_bringup` | 예정 | Python·YAML | 실물/sim launch profile, lifecycle 실행 순서 | 첫 실행 패키지와 함께 |
| `hold_flow_web` | 예정 | FastAPI·HTML·JavaScript·rclpy | 웹 요청 검증, `ServeDrink` Action client | 요청 JSON 계약 |
| `hold_flow_navigation` | 예정 | Nav2·SLAM Toolbox·AMCL·DWB·Collision Monitor | 지도, station registry, 장애물 회피, 도킹 staging | 실험 공간·station 실측 |
| `hold_flow_perception` | 예정 | Python·RGB-D·YOLO·OpenCV·tf2 | station 정렬, 컵·물통·선반·테이블·액면 인지 | 컵·카메라 POC |
| `hold_flow_motion` | 예정 | C++·Eigen·FollowJointTrajectory | PLANNED phase의 그립·IK·시작·복귀·검증 궤적·붓기 폐루프 | 새 팔 역할·선반 좌표 |
| `hold_flow_safety` | 예정 | C++·rclcpp·tf2 | command lease, 관절 delta, timeout, 정지 | 첫 실물 명령 전에 |
| `hold_flow_hardware` | 예정 | Python→C++ 선택·LeRobot·serial | 좌우 SO-101 포트 단독 소유, 상태·명령 변환 | 포트·서보 변종 감사 |
| `hold_flow_learning` | 예정 | Python·LeRobot 0.6.1·PyTorch·ACT | phase 표시 수집, ACT_ALL·로컬 ACT, rollout, 실패/HIL 데이터 | phase·backend 계약·데이터 게이트 |
| `hold_flow_mission` | 예정 | Python·rclpy | 웹→이동→조작→서빙→도킹 상태기계 | Action mock 통과 |
| `hold_flow_logging` | 예정 | Python·rosbag2·JSON·Parquet | request ID로 미션·episode·실패 연결 | 인터페이스와 함께 |
| `hold_flow_isaac` | importer·정적 계약 존재 | Python·USD·Isaac Sim 6.0·ROS 2 Bridge | v0.3 URDF→USD, Nav2·양팔·센서 SIL, sim/real gap | Isaac 장비에서 USD·접촉·gain 검증 |

## 명령과 데이터 경로

```text
웹 브라우저
  → hold_flow_web
  → ServeDrink Action
  → hold_flow_mission
      ├─ NavigateToPose / DockRobot → Nav2 → base controller
      ├─ AlignToStation → perception
      └─ ExecuteManipulationSkill → phase router
            ├─ backend=PLANNED → motion ┐
            └─ backend=ACT → learning   ├→ command_mux → safety_guard → 양팔 bridge
                                        ┘

Astra S RGB-D → perception → station pose / fill estimate → mission·phase router·logging
joint state·action·strategy·backend·safety event → logging → episode sidecar / failure bank
```

### 소유권 규칙

- 베이스 이동 중에는 Nav2만 베이스 명령 lease를 가진다.
- 팔 조작 중에는 베이스 lease를 해제하고 정지 상태를 확인한다.
- PLANNED·ACT·teleop 중 하나만 팔 명령 lease를 가진다.
- backend 전환은 관절 속도 0, 허용 자세, 남은 ACT chunk 폐기와 기존 lease 해제 뒤에만 한다.
- 좌우 SO-101 시리얼 포트는 `hold_flow_hardware` 한 프로세스만 연다.
- perception은 actuator 명령을 직접 내리지 않고 판정 결과만 반환한다.
- Action 취소는 mission→policy executor→hardware까지 전파한다.

## 먼저 만들 인터페이스

| 인터페이스 | 목적 | 성공 결과에 포함할 것 |
|---|---|---|
| `ServeDrink.action` | 전체 미션 실행 | request ID, final state, error code |
| `AlignToStation.action` | 주방·테이블 정밀 정렬 | pose, quality, timestamp |
| `ExecuteManipulationSkill.action` | PLANNED·ACT·HYBRID 조작 | skill/strategy, phase별 backend·checkpoint, failure stage/code, intervention count |
| `EstimateFillLevel.srv` 또는 Action | 물 양 판정 | estimated mL, class, confidence, cup calibration ID |
| `MissionEvent.msg` | 계층 간 공통 로그 | request ID, stage, event, timestamp |
| `SafetyState.msg` | 실행 허용·정지 상태 | active lease, stop reason, timestamp |

긴 작업에는 Action을 사용한다. 물 양을 한 프레임에서 즉시 읽으면 Service로 시작할 수
있지만, 연속 프레임을 모아 판정하고 취소·feedback이 필요하면 Action으로 올린다.

## 병렬 구현 순서

### policy lane — 팀 1순위

1. policy 1·2 내부 phase와 회의의 `3번 policy` 의미를 확정한다.
2. `PLANNED_ALL / ACT_ALL / HYBRID` preset과 phase별 시작·종료·backend 계약을 IDL로 고정한다.
3. PLANNED_ALL mock·dry-run으로 phase·성공 판정·안전 정지를 연결한다.
4. 주방·테이블 장면을 수동 배치해 phase 표시 LeRobot smoke episode를 수집한다.
5. ACT_ALL과 phase별 ACT 과적합 기준선을 만든다.
6. 같은 scenario matrix에서 세 전략을 비교한다.
7. rollout 실패를 strategy·backend·phase별로 수집하고 확인된 원인만 수정·보강한다.

### 이동·IK lane — 사용자 담당

1. `stations.yaml`에 충전소·주방·테이블 계약을 만든다.
2. SLAM 지도 작성과 AMCL/Nav2 반복 이동을 분리한다.
3. DWB를 첫 controller로 두고 실측 footprint·속도·가감속 한계를 적용한다.
4. LDS-03 obstacle layer와 Collision Monitor 감속·정지 영역을 검증한다.
5. 장애물 회피·재계획 실패 코드와 Nav2 feedback을 기록한다.
6. 새 팔 역할과 선반 frame을 기준으로 IK 시작·복귀 자세를 검증한다.
7. 현행 v0.3 URDF를 Isaac Sim에서 USD로 만들고 같은 인터페이스·접촉·gain을 검증한다.

### 인지 lane

1. Astra S에서 컵·액면 가시성 POC를 한다.
2. YOLO segmentation 라벨과 컵별 보정표를 만든다.
3. `UNDER/TARGET/OVER/SPILL/UNKNOWN` 판정과 confidence를 반환한다.
4. 가림이 실제 주요 실패 원인일 때만 손목 카메라를 검토한다.

### 통합 lane

1. 실제 Nav2·policy 대신 mock Action server로 상태기계를 먼저 통과시킨다.
2. phase router와 세 preset을 mock PLANNED·ACT backend로 검증한다.
3. 각 실제 패키지를 하나씩 mock과 교체한다.
4. backend 전환, Action 결과·취소·timeout·재시도를 검증한다.
5. 마지막에 웹 요청부터 충전 시작까지 request ID 하나로 연결한다.

## 코드 리뷰에서 반드시 확인할 것

- 현재 태스크가 물 서빙 하나인지, 과거 커넥터·음성·범용 태스크가 재진입하지 않았는가
- 단위가 degree/radian, mm/m 중 무엇인지 필드와 코드에서 명시됐는가
- `frame_id`와 timestamp가 있으며 오래된 TF·영상이 거부되는가
- timeout·취소·실제 hold/stop 경로가 있는가
- 베이스와 팔, PLANNED와 ACT의 명령 소유자가 동시에 활성화되지 않는가
- strategy·phase·backend·checkpoint 또는 planner/trajectory/controller 버전이 기록되는가
- 카메라·joint state·action의 실제 fps와 주기 지터를 기록하는가
- policy ID, checkpoint, dataset, calibration, code commit을 연결했는가
- 성공률뿐 아니라 단계별 실패 코드와 사람 개입을 남기는가
- 실물·Isaac Sim·혼합 데이터의 dataset ID와 split이 분리됐는가
- TODO·stub·빈 패키지를 구현 완료로 보고하지 않았는가
