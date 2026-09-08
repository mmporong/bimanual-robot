# 실패 데이터 저장·진단·활용 규약

실패 데이터의 목적은 실패 영상을 모으는 것이 아니라, **관측된 증상과 확인된 원인을
분리하고 수정 뒤 같은 조건에서 다시 검증하는 것**이다. 한 번 성공한 데모보다 어떤 계층의
문제인지 근거로 좁힌 기록을 남긴다.

이 디렉터리에는 실영상·rosbag·trajectory를 커밋하지 않는다. 대용량 근거는 데이터셋과
함께 Hugging Face Hub(private) 또는 팀 artifact storage에 두고, 저장소에는 스키마·작성
예시·집계 규칙만 둔다.

## 1. 저장 단위

`failure_id`는 한 원인 조사를 식별한다. `source.run_id`는 조사를 시작한 대표 실행이며,
같은 조사에 속하는 추가 조작 실행은 각 episode sidecar의 `run_id`와 동일한
`failure_record_id`로 연결한다. 재시험은 `validation`의 run ID와 근거 파일로 연결한다.
추가 실행의 개별 조건·관측을 대표 실행의 `context` 위에 덮어쓰지 않는다.
Nav2 등 sidecar가 없는 추가 발생은 별도 실패 레코드로 기록해 관측과 조건을 보존한다.

따라서 실패 레코드 수는 실패 발생 횟수와 다르다. 발생 빈도·성공률은 성공 실행도 포함한
전체 run 목록을 분모로 계산해야 한다. 현재 집계 도구는 분석 레코드 수만 보고한다.

```text
private-artifacts/
└── failures/
    └── FAIL-YYYYMMDD-NNNN/
        ├── failure.json
        ├── evidence/
        │   ├── before.mp4
        │   ├── run.mcap
        │   ├── nav_arrival.json
        │   ├── joint_action_window.parquet
        │   └── fill_measurement.json
        └── validation/
            ├── same_condition_runs.json
            └── regression_runs.json
```

- `failure.json`: `failure_record.schema.json`을 통과하는 분석 레코드
- `before.mp4`: 실패 직전과 직후가 포함된 영상 구간
- `run.mcap`: 필요한 토픽만 보존한 실행 기록
- `*_window.parquet`: 실패 전후 관측·action·실측 관절값
- `validation/`: 수정 뒤 같은 조건과 다른 조건을 다시 실행한 결과
- 모든 URI는 이동해도 다시 찾을 수 있는 안정 경로를 사용하고, 가능하면 SHA-256을 남긴다.

## 2. 두 종류의 기록을 연결한다

### 에피소드 sidecar

`episode_metadata.schema.json`은 빠른 집계용이다. 실패 여부, 실패 단계, 관측된 failure code,
이상 현상과 `failure_record_id`를 저장한다.

### 실패 분석 레코드

`failure_record.schema.json`은 원인 분석용이다.

```text
관측 observation
→ 실행 조건 context
→ 근거 evidence
→ 가설과 확정 원인 diagnosis
→ 즉시 조치와 수정 response
→ 동일 조건·회귀 재시험 validation
→ 학습·평가 데이터 사용 결정 data_use
```

`CUP_GRASP_MISS`, `UNDER_FILL`, `TIMEOUT`은 관측된 결과다. 이를 곧바로 `ACT 데이터 부족`,
`YOLO 오류`, `서보 고장` 같은 원인으로 기록하지 않는다.

## 3. 필수 캡처 구간

| 경계 또는 단계 | 함께 보존할 데이터 | 구분하려는 원인 |
|---|---|---|
| Nav2 → 조작 시작 | 목표·실제 pose, XY·yaw 오차, covariance, 베이스 속도, TF age, 로컬 정렬량, IK 도달성 | 조작 실패와 도착·정렬 실패 |
| 조작 phase | `control_strategy`·backend·phase, checkpoint 또는 planner·trajectory·controller·config, 영상, 관측/action/관절 window, 그리퍼, inference·계획 시간, 제어 지터, 안전 게이트 | 인지·계획·데이터·정책·실행기·하드웨어 실패 |
| 물 양 판정 | target·YOLO 추정·저울 실측 mL, mask, confidence, 가림, calibration ID, 물통 기울기·시간 | segmentation·보정표·붓기 제어 실패 |
| Isaac Sim ↔ 실물 | URDF commit, 관절 축·방향·한계, 동일 관절값의 말단 pose, 충돌·접촉, 지연 | 모델 오류와 제어·물성 gap |
| 미션 전이 | request/run ID, 상태 진입·종료 시각, Action 결과·취소, timeout, 센서 최신성 | 상태기계·인터페이스 실패 |

원인 판단에 쓰지 않을 데이터는 무작정 늘리지 않는다. 실패 시각 기준의 앞뒤 구간과
clock 기준을 정해 영상·관절·action·센서의 동일 시점을 연결한다. 구간을 잘라도 원본 URI와
시작·종료 오프셋을 남긴다. 로그 보존 기간은 실제 파일 크기와 저장 예산을 측정한 뒤 정한다.

## 4. 실패 처리 순서

1. `run_id`와 `failure_id`를 발급하고 `triage_owner`와 안전 정지 여부를 기록한다.
2. `observation`에는 보인 현상만 적고 `cause_status=untriaged`로 시작한다.
3. 실패 전후 영상·관절/action·Nav2·인지·안전 데이터를 evidence ID로 연결한다.
4. 원인 후보마다 가설을 만들고 `pending / rejected / supported`를 기록한다.
5. 근거가 생긴 경우에만 `cause_status=confirmed`와 `root_cause`를 채운다.
6. 정책 재학습, 보정 수정, station 수정, 기구 수정 중 필요한 조치만 선택한다.
7. 수정한 뒤 같은 조건을 다시 실행하고, 다른 조건의 회귀도 따로 확인한다.
8. 실패 원본을 학습·평가·HIL 중 어디에 사용할지 `data_use`로 결정한다.

원인을 확인하지 못했으면 `unresolved`로 남긴다. `OTHER`나 자유 메모로 억지 결론을 만들지
않는다.

같은 조건에서 반복 실패하면 로직만, 조건별로 성능이 다르면 데이터만 문제라고 단정하지
않는다. 둘 다 원인을 좁히는 단서다. 고정된 정책 편향도 반복 실패를 만들고, 센서·기구
문제도 조건에 따라 달라질 수 있다. 측정값의 유효성을 먼저 확인한 뒤 가설별 대조를 남긴다.

## 5. 학습 데이터와 실패 데이터의 사용 구분

| 데이터 | BC 학습 포함 | 보존·활용 |
|---|---:|---|
| 자율 rollout 실패 원본 | 아니오 | failure bank, 실패 분포, 회귀 평가 |
| 조작자 실수 시연 | 아니오 | 수집 절차 개선 근거 |
| 센서·캘리브레이션 오류 실행 | 아니오 | 시스템 수정과 재시험 근거 |
| 실패 직전 사람이 복구한 HIL 구간 | 별도 데이터셋에서 가능 | recovery 학습, 원본 성공 시연과 분리 |
| 성공했지만 미끄러짐을 복구한 시연 | 기준에 따라 포함 | 복구 행동 분석, 조건 태그 필수 |
| 원인 미확정 실패 | 아니오 | `unresolved` failure bank |

실패가 많다고 곧바로 데이터를 더 모으지 않는다.

- 도착·TF·캘리브레이션·기구·서보 원인이면 시스템을 수정하고 같은 checkpoint 또는 같은
  PLANNED 산출물 버전으로 재시험한다.
- 관측 분포 밖의 초기 상태이면서 시스템 이상이 배제된 경우에만 성공 시연 또는 HIL 복구
  데이터를 보강한다.
- HIL 학습 포함 검증은 확인된 원인층 `policy_data`만 허용한다. navigation·hardware 등
  시스템 원인을 수집 결정 문자열만 바꿔 학습 보강으로 처리할 수 없다.
- 라벨 오류면 재라벨링하고 dataset version을 올린다.
- 실패 원본을 성공 시연 데이터에 섞어 실패 행동을 모방시키지 않는다.

`include_in_training=true`는 실패 원본의 허가가 아니라 검토한 HIL 파생 구간의 사용 결정이다.
복구 구간·제어 주체·성공 확인 근거와 별도 dataset ID를 기록해야 한다. `counterexample`은
실패를 분석하거나 평가할 때의 용도이며 BC의 목표 action으로 자동 편입하지 않는다.

HIL 파생 sidecar의 `training_provenance`에는 원본 전체 길이 `source_duration_s`, 원본 시간축의
사람 개입 구간 `source_intervention_time_range_s`, 그 안에서 추출한 `recovery_time_range_s`를
남긴다. 파생 `outcome.interventions`는 잘라낸 에피소드 시작을 0초로 하며 학습 구간 전체의
사람 제어를 기록한다. 실패 레코드의 `recovery_evidence_ids` 중에는 원본 시간축에서 복구
구간 전체를 포함하는 `time_range_s`가 있는 근거가 필요하다.
원인이 확정되고 HIL 수집 또는 재학습을 결정한 뒤, 담당자가 해당 근거에서 복구 성공을
확인해야 `recovery_verified_success=true`와 실제 제어 주체 `recovery_control_source`를
기록해 학습 포함할 수 있다. 이는 수정 후 동일 조건·회귀 검증인 `validation`과 별개다.
복구 시연으로 재학습하기 전에 문제가 이미 `resolved`일 것을 요구하지는 않는다.

`fill_class=TARGET`은 비전 추정량의 판정이다. 실측과 어긋난 false positive도 실패 데이터로
보존한다. 전체 성공을 주장하려면 최종 단계 결과·흘림 여부·실측 목표량도 성공과 일치해야 한다.

이미 분석·튜닝에 사용한 실패 조건은 회귀 평가에 재사용할 수 있지만, 처음 보는 조건의
일반화 성과라고 발표하지 않는다. 최종 holdout은 수집 전에 세션·장면 조건으로 분리하고,
원본과 그 원본에서 자른 모든 복구 구간은 같은 분할에 둔다. 파생 sidecar의
`training_provenance.source_split`을 필수 기록하고 최상위 `split`과 다르면 검증이 실패한다.
검증기는 두 선언의 일치만 검사한다. 원본 dataset의 실제 분할·영상의 성공 여부까지 조회하거나
판정하지 않으므로, 담당자는 원본 manifest와 근거를 대조한 후 승인한다.

에피소드 검증은 객체 ID 중복, 정책별 필수 역할(주방: cup/jug/shelf, 테이블: cup/shelf/table),
strategy와 phase backend의 일치, phase 시간·순서·시작 상태 출처, ACT checkpoint와 PLANNED
산출물 ID, `water_measurement.cup_id`의 실제 cup 객체 참조도 검사한다. `jsonschema`가 없으면 축소 검증만
수행하고 실패 레코드 참조 대조는 생략하며 종료 코드 3을 반환한다. 이를 검증 PASS로 취급하지 않는다.

## 6. 집계와 포트폴리오 증거

집계할 때 관측 코드와 확인 원인을 따로 센다.

- 단계별 `observed_code` 빈도
- `root_cause.layer`별 빈도와 미확정 비율
- 같은 원인이 발생한 조건 분포
- 조치 종류와 동일 조건 재시험 결과
- 재학습으로 해결된 실패와 시스템 수정으로 해결된 실패의 구분
- 수정 뒤 새로 생긴 회귀 실패

포트폴리오에는 성공 장면만 나열하지 않고, 대표 실패 한 건의 `관측 → 가설 → 기각 근거 →
확정 원인 → 수정 → 재시험`을 타임라인으로 보여준다. 실제 수치가 없는 예시 값은 성과로
사용하지 않는다.

## 7. 파일과 검사

- 실패 레코드 스키마: [`../schema/failure_record.schema.json`](../schema/failure_record.schema.json)
- 가상 작성 예시: [`../schema/example_failure_record.json`](../schema/example_failure_record.json)
- 에피소드 스키마: [`../schema/episode_metadata.schema.json`](../schema/episode_metadata.schema.json)
- 포함·제외 기준: [`../schema/에피소드_포함제외_기준.md`](../schema/에피소드_포함제외_기준.md)
- 수집 입력 예시: [`../schema/example_failure_capture.json`](../schema/example_failure_capture.json)
- 저장·집계 명령: [`../../tools/failure_bank.py`](../../tools/failure_bank.py)

```bash
cd ~/bimanual-robot
python3 tools/validate_failure_record.py
python3 tools/validate_failure_record.py /path/to/failure-record-directory
```

검증에는 `jsonschema`가 필요하다. 미설치로 전체 검증을 못 하는 경우 성공으로 처리하지
않는다. 필드 검사 PASS는 근거 영상의 진위나 로봇의 실제 성공을 증명하지 않는다.

## 8. 실제로 저장하고 사용하는 순서

### 8.1 관측 입력과 근거를 보존한다

수집 입력은 `failure_id`, `triage_owner`, `source`, `observation`, `context`,
`immediate_action` 여섯 필드다. `source`에 run ID·환경을, `observation`에 관측 단계·코드·
요약·판정 주체·심각도를 적는다. `context`에는 실행 당시의 scenario·strategy·phase backend·
코드·캘리브레이션·계측을 넣는다.
현재 PC의 Git HEAD를 실험 당시 commit으로 대신 기록하지 않는다. 모르는 값은 추정하지
않고 빈 매핑으로 남긴 뒤 분석 때 보강한다. `immediate_action`에는 실제 취한 조치만 적는다.

아래는 저장 형식을 확인하는 가상 예시다. 실영상이 아닌 예시 JSON을 근거로 복사하므로
성과나 실기체 시험 기록으로 사용하지 않는다.

```bash
cd ~/bimanual-robot
python3 tools/failure_bank.py capture \
  --input data/schema/example_failure_capture.json \
  --evidence data/schema/example_episode_meta.json \
  --root "$HOME/robot-artifacts/failures"
python3 tools/failure_bank.py verify-evidence "$HOME/robot-artifacts/failures"
python3 tools/failure_bank.py summary "$HOME/robot-artifacts/failures" --include-examples
```

실험에서는 입력을 실제 관측으로 바꾸고 `FAIL-YYYYMMDD-NNNN` ID와 실제 파일을 지정한다.
도구는 원본을 복사해 각 파일의 SHA-256을 기록하고 `open / untriaged / 학습 제외`로 시작한다.
같은 ID가 있으면 덮어쓰지 않는다. 복사 중 오류가 나면 부분 번들을 보존하며 마지막의
`failure.json`이 없는 번들을 완료로 취급하지 않는다. 기존 번들을 확인한 뒤 새 ID로
다시 수집한다. 이 명령은 로봇을 정지시키거나 Hub에 업로드하지 않는다.

### 8.2 원인·조치·재시험을 추가한다

보존한 `failure.json`에 가설과 evidence 참조를 추가한다. 반증된 가설도 삭제하지 않고
`rejected`와 기각 근거를 남긴다. 수정이 코드 변경이면 commit, 캘리브레이션 변경이면 새
calibration ID, 기구 변경이면 부품 버전과 계측 기록을 `response`에 연결한다.

원인이 확인된 상태(`confirmed`)와 수정이 검증된 상태(`resolved`)를 구분한다. 같은 조건
재시험과 다른 조건 회귀가 모두 PASS하고 protocol·판정 근거가 있어야 `resolved`로 닫는다.
run ID 목록만 적는 것으로 실험을 대신하지 않으며, 근거 파일에 각 run의 조건·strategy·
phase backend·checkpoint 또는 PLANNED 산출물·결과·사람 개입 여부와 전체 시도 수를 남긴다.
미실행은 `not_run`, 실패는 `fail`로 유지한다.

```bash
cd ~/bimanual-robot
python3 tools/validate_failure_record.py "$HOME/robot-artifacts/failures"
python3 tools/validate_episode_meta.py "$HOME/robot-artifacts/episode-meta" \
  --failure-record-dir "$HOME/robot-artifacts/failures"
python3 tools/failure_bank.py summary "$HOME/robot-artifacts/failures"
```

집계는 EXAMPLE ID를 기본 제외하고 환경·전략·phase·backend·관측 코드·확정 원인·재시험·
재학습 결정과 미분석/재시험 대기 ID를 출력한다. 중복 ID나 잘못된 레코드가 있으면 부분 집계를 성공으로
출력하지 않는다. 성공 실행의 분모가 없으므로 성공률은 `null`로 남긴다.

### 8.3 검증 뒤 팀에 공유한다

로컬 artifact 경로는 수집 PC의 임시 보관소다. 팀 공유가 승인된 데이터만 private Hub 또는
팀 artifact storage에 옮기고, `data/README.md` 인덱스에 실제 위치와 버전을 남긴다.
번들 내부의 상대 경로·SHA-256을 유지하면 다운로드 후 `verify-evidence`로 원본 일치를
확인할 수 있다. 서명 만료 URL이나 인증 토큰을 레코드에 넣지 않는다. 업로드 자동화와
영상/rosbag 구간 추출은 아직 이 도구의 구현 범위가 아니다.
