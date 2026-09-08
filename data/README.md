# data — 물 서빙 정책 데이터셋 규격과 인덱스

**이 폴더에는 실데이터를 넣지 않는다.** 에피소드·영상·가중치는 Hugging Face Hub(private)에
두고, 여기에는 저장 위치 인덱스와 데이터를 만드는 계약만 둔다.

현행 태스크와 policy 경계는
[2026-09-07 물 서빙 로봇 회의 결정](../docs/20260907_물서빙로봇_회의결정과_실행범위.md)을
따른다. 커넥터·범용 3태스크·음성 명령 데이터는 현행 데이터셋에 섞지 않는다.

## 1. 저장 위치 인덱스

| 데이터셋 | HF repo | policy | 에피소드 수 | 수집 기간 | 상태 |
|---|---|---|---|---|---|
| (아직 없음) | | | | | |

수집을 시작하면 이 표에 한 줄씩 추가한다. 표에 없는 데이터셋은 팀 기준으로 존재하지 않는
것으로 본다. 현재 실제 데이터셋이 없으므로 스키마 변경에 따른 마이그레이션 대상도 없다.

## 2. 데이터셋 분리

| dataset ID 예시 | 목적 | 포함 범위 |
|---|---|---|
| `water_kitchen_policy1_v1` | 주방 ACT 기준선 | 컵·물통 파지부터 물통 반환·컵 선반 적재 |
| `water_table_policy2_v1` | 테이블 ACT 기준선 | 선반 컵 파지부터 테이블 놓기·팔 복귀 |
| `water_kitchen_policy1_hil_v1` | 실패 교정 데이터 | 사람 개입 전후 문맥 + 개입·성공 복구 구간(구간 경계 표시) |

- Nav2 이동은 ACT 데이터셋에 넣지 않는다.
- policy 1과 policy 2를 한 데이터셋 태스크 문자열만 바꿔 섞지 않는다.
- `policy 3`은 정의되기 전까지 dataset ID를 만들지 않는다.
- 실물·Isaac Sim·혼합 데이터는 repo ID 또는 명시적인 split으로 분리한다.

HIL 원본에서는 실패 직전 문맥과 성공 복구 구간을 함께 보존하되, `control_source`와 구간
경계를 표시한다. 현재 저장소의 양팔 ROS mux 어댑터가 공식 LeRobot HIL 수집 명령과 바로
연결된다는 smoke 근거는 아직 없다.

## 3. 포맷

LeRobot 데이터셋 포맷 **v3.0**을 기준으로 한다. 저장소 기준 LeRobot 버전은 0.6.1이며,
실제 수집 CLI·feature 이름은 설치된 0.6.1에서 다시 확인한 뒤 고정한다.

```text
<dataset_root>/
├── meta/
│   ├── info.json
│   ├── stats.json
│   ├── tasks.parquet
│   └── episodes/…parquet
├── data/chunk-000/file-000.parquet
└── videos/<video_key>/chunk-000/file-000.mp4
```

LeRobot v2와 v3 메타 파일을 같은 repo에 섞지 않는다. 다른 버전으로 받은 데이터는 변환 후
인덱스에 원본·변환 버전을 기록한다.

## 4. 양팔 관측·행동 표현

`bi_so_follower`·`bi_so_leader`의 **키 접두어 방식**을 유지한다.

| 대상 | 키 형태 | 예시 |
|---|---|---|
| 관절 상태·action | `left_*` / `right_*` | `left_shoulder_pan.pos`, `right_gripper.pos` |
| 팔별 카메라 | `left_*` / `right_*` | `left_wrist`, `right_wrist` |
| 공용 카메라 | 접두어 없음 | `top` 또는 `mast` |

- policy 2가 한 팔만 사용해도 반대 팔 키를 제거하지 않는다.
- 첫 기준선의 action은 팔별 관절·그리퍼 **절대 목표 위치**로 고정한다.
- 상대 action이나 Cartesian action을 시험하려면 별도 dataset ID를 사용한다.
- 카메라 key·해상도·fps·crop과 관절 순서는 dataset version 동안 바꾸지 않는다.
- 관측·action timestamp와 실제 제어 주기 지터를 기록한다.

키 규약은 닫혔지만 **본수집 보류 조건은 해제되지 않았다.** `3번 policy` 의미, policy 2
사용 팔, 시작·종료 상태, 컵·선반·물 양 계약이 먼저 확정돼야 한다.

## 5. 수집 조건 sidecar

LeRobot 표준 메타에 없는 수집 조건은 episode index로 조인하는 JSON sidecar로 남긴다.

- 스키마: [schema/episode_metadata.schema.json](schema/episode_metadata.schema.json)
- 예시: [schema/example_episode_meta.json](schema/example_episode_meta.json)
- 검사: `python3 tools/validate_episode_meta.py`
- 실패 분석 스키마: [schema/failure_record.schema.json](schema/failure_record.schema.json)
- 실패 분석 예시: [schema/example_failure_record.json](schema/example_failure_record.json)
- 실패 분석 검사: `python3 tools/validate_failure_record.py`

필수 추적 범위:

| 범주 | 필드 |
|---|---|
| policy | policy ID·version·checkpoint·실행 모드·action 표현 |
| 로봇 | 캘리브레이션 파일 해시·카메라·fps·제어 코드 commit |
| 물체 | 컵·물통·선반·테이블 ID, 질량·표면·시작 영역 |
| 물 양 | 목표·YOLO 추정·ground truth·보정표 ID·판정 class |
| 결과 | 단계별 성공, 관측된 실패 단계·코드, `failure_record_id`, 이상 현상, 사람 개입 횟수 |
| 분할 | train/holdout과 학습 포함 여부·제외 사유 |

## 6. 수집 순서

### 6.1 smoke 수집

목표는 성능이 아니라 파이프라인 검증이다.

1. 물 없이 컵·빈 물통으로 시작한다.
2. 장면·카메라·관절 key를 고정한다.
3. 짧은 에피소드를 수집한다.
4. sidecar 검증기와 LeRobot 로더가 모두 읽는지 확인한다.
5. 영상·상태·action의 시간 정렬을 눈으로 대조한다.
6. 같은 episode에 과적합해 action이 재생되는지 확인한다.

### 6.2 본수집

- 시작 위치·조명·물체 자세를 미리 정한 구간에서 나눠 수집한다.
- 한 episode 안에서 실패를 숨기기 위해 여러 번 처음부터 재시도하지 않는다.
- 조작자 실수나 스톨이 있으면 원본은 failure bank에 보존하고 BC 학습 제외 사유를 남긴다.
- 성공 시연 데이터와 HIL 교정 데이터는 dataset ID를 분리한다.
- policy 1의 실제 물 수집은 건식 파지·붓기 동작과 YOLO stop gate가 먼저 통과한 뒤 한다.

## 7. 실패 데이터 저장과 활용

상세 규약은 [failures/README.md](failures/README.md)를 따른다. 에피소드 sidecar의
`failure_code`는 관측된 증상을 빠르게 집계하기 위한 값이고, 확인된 원인은 별도 실패 분석
레코드에 저장한다.

실패 레코드의 생성·요약·증거 검사는 [`tools/failure_bank.py`](../tools/failure_bank.py)의
`capture`, `summary`, `verify-evidence` 명령을 사용한다.

| failure stage | failure code 예시 | 함께 보존할 것 |
|---|---|---|
| scene | `OBJECT_NOT_FOUND`, `LOW_VISION_CONFIDENCE` | 원본 영상·검출 결과·TF |
| cup grasp | `CUP_GRASP_MISS`, `CUP_SLIP` | 그리퍼·관절·컵 pose |
| jug grasp | `JUG_GRASP_MISS`, `JUG_SLIP` | 그리퍼·관절·물통 pose |
| pour | `UNDER_FILL`, `OVER_FILL`, `SPILL`, `FILL_UNKNOWN` | YOLO mask·추정/실측 mL·붓기 phase |
| return/place | `JUG_RETURN_FAILED`, `SHELF_PLACE_FAILED` | 목표 영역·최종 pose |
| table serve | `TABLE_PLACE_FAILED`, `CUP_TILT_UNSAFE` | 테이블 영역·컵 기울기 |
| runtime | `SERVO_STALL`, `COLLISION_RISK`, `TIMEOUT` | safety state·action window |

실패 분석 레코드에는 다음 순서를 한 `failure_id`로 연결한다.

```text
관측된 증상
  → 실행 조건·근거
  → 원인 가설·기각
  → 확인된 원인
  → 조치
  → 같은 조건 재시험
  → 다른 조건 회귀 확인
  → 학습·평가 데이터 사용 결정
```

- Nav2·TF·캘리브레이션·기구·서보 문제는 시스템을 고친 뒤 같은 checkpoint로 재시험한다.
- 데이터 분포 문제가 다른 계층과 분리된 경우에만 새 성공 시연이나 HIL 복구 데이터를 모은다.
- 실패 에피소드는 BC 성공 시연에 섞지 않고 failure bank·회귀 평가에 사용한다.
- 사람 개입 복구 구간은 별도 HIL dataset ID로 분리한다.
- 원인 미확정 실패는 `unresolved`로 보존하고 추정 원인을 확정값처럼 쓰지 않는다.

## 8. 포함·제외와 holdout

수집한 것을 전부 학습에 넣지 않는다. 기준과 근거는
[에피소드 포함·제외 기준](schema/에피소드_포함제외_기준.md)을 따른다.

- `split`의 `holdout` 포함은 평가 집합에 보존한다는 뜻이며 BC 학습 포함을 뜻하지 않는다.
- BC 학습 포함 여부는 별도 `include`와 제외 사유로 판정한다.
- 자율 rollout 실패 원본은 `include=false`여도 삭제하지 않고 failure bank에 보존해 실패
  분포 집계와 동일 조건·회귀 검증에 사용한다.

평가 조건은 수집 전에 정한다.

- 시작 위치 구역
- 조명 조건
- 컵·물통 배치
- 카메라 가림 조건
- 물 양 목표 구간
- 조작자

팀이 합격 수치를 정하기 전에는 임의의 성공률을 넣지 않는다. 매 평가에서 성공 횟수/전체
횟수, 단계별 실패 수, 사람 개입 횟수, 물 양 오차를 함께 기록한다.

## 9. dataset card

Hub에 올릴 때 [dataset card 템플릿](schema/dataset_card_TEMPLATE.md)을 함께 채운다. 카드에는
policy 경계, 카메라·action key, 물체 버전, 포함·제외 규칙, holdout 조건, 알려진 실패를
반드시 적는다.
