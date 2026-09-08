# Dataset Card — <데이터셋 이름>

> Hub에 올릴 때 이 템플릿을 채워 `README.md`로 함께 올린다. 빈 항목을 남기지 않는다 — 모르면 "확인 안 됨"이라고 적는다.

## 요약

| 항목 | 값 |
|---|---|
| repo id | |
| 포맷 버전 | LeRobot v3.0 |
| 태스크 | |
| policy ID·version | |
| control strategy | TELEOP / PLANNED_ALL / ACT_ALL / HYBRID |
| episode scope | full_skill / phase_slice |
| phase·backend | phase별 TELEOP / PLANNED / ACT |
| checkpoint·PLANNED 산출물 | 텔레옵 시연이면 해당 없음 |
| action 표현 | absolute joint / relative joint / Cartesian delta |
| 에피소드 수 | 총 / 학습 포함 / 제외 |
| 총 길이 | 프레임 수, 시간 |
| fps | |
| 로봇 | |
| 팔 구성 | simultaneous / sequential / single |
| 수집 기간 | |
| 조작자 수 | |

## 무엇이 들어 있나

관측 피처와 액션 피처를 표로 적는다. `left_*`·`right_*` 키, 관절 순서, 단위, 카메라
crop을 반드시 명시한다.

| 키 | 형태 | 의미 |
|---|---|---|
| `observation.state` | (D,) | |
| `action` | (D,) | |
| `observation.images.<key>` | HxWx3 | |

## policy·phase 경계

- 시작 상태:
- 포함 동작:
- 종료 상태:
- 외부 정지 조건:
- 반대 팔 상태:
- phase 순서와 각 시작·종료 조건:
- backend 전환 시 정지·lease·ACT chunk 폐기 조건:

policy 1과 policy 2의 경계가 다르면 별도 dataset card를 만든다. 정의되지 않은 policy 3을
기존 데이터와 임의로 합치지 않는다.

## 어떻게 모았나

- 텔레옵 구성 (리더암 / VR / 기타):
- 캘리브레이션 절차와 파일:
- 카메라 배치와 해상도·fps:
- 수집 환경 (장소·조명·배경):
- 컵·물통·선반·테이블 ID와 버전:
- policy/strategy/phase backend/checkpoint 또는 planner·trajectory·controller·config ID:
- phase_slice의 원본 dataset·episode·시간 범위·시작 상태 출처:
- YOLO·컵 보정표 ID:

## 조건 분포

조건별 에피소드 수를 적는다. 이 표가 없으면 나중에 "이 조건에서 성능이 떨어진다"를 데이터 부족 때문인지 알 수 없다.

| 조건 축 | 값 | 에피소드 수 |
|---|---|---|
| 물체 시작 구역 | | |
| 조명 | | |
| 조작자 | | |
| 컵·물통 버전 | | |
| 카메라 가림 | | |
| 목표 물 양 | | |
| phase 시작 상태 출처 | teleop / planned rollout / ACT rollout | |

## 무엇을 뺐나

포함/제외 기준 문서의 번호별 제외 건수와 제외율. 기준 문서: `data/schema/에피소드_포함제외_기준.md`

## 평가셋

- 홀드아웃 축과 그렇게 정한 이유:
- 홀드아웃 에피소드 수:
- **수집 전에 정했는지 여부** (사후 분할이면 그렇게 적는다):
- 분할 그룹(세션·장면 조건)과 원본/복구 구간이 같은 분할에 있는지:
- 분석·튜닝에 이미 사용한 회귀 조건과 아직 사용하지 않은 최종 holdout의 구분:

## 전략별 결과와 실패 분포

PLANNED_ALL·ACT_ALL·HYBRID는 같은 scenario matrix와 holdout을 썼는지 먼저 적는다.

| 전략 | 성공 / 전체 | cycle time | 사람 개입 | 변경 비용·비고 |
|---|---|---|---|---|
| PLANNED_ALL | | | | |
| ACT_ALL | | | | |
| HYBRID | | | | |

| 단계·backend | 성공 / 전체 | 주요 failure code | 사람 개입 횟수 |
|---|---|---|---|
| 컵 파지 | | | |
| 물통 파지 | | | |
| 붓기·물 양 | | | |
| 물통 반환 | | | |
| 선반 또는 테이블 놓기 | | | |

- 목표 대비 물 양 오차 분포:
- `UNDER / OVER / SPILL / UNKNOWN` 건수:
- 안전 정지 건수와 원인:

## 실패 분석 레코드

`failure_code`는 관측된 증상으로 집계하고, 확인된 원인은 `failure_record_id`가 가리키는
별도 레코드에서 집계한다. 원인이 확인되지 않은 건은 `unresolved`로 센다.
실패 에피소드 수와 원인 조사 레코드 수는 별도 열로 구분한다. 한 조사에 여러 실행이
연결될 수 있으므로 레코드 수를 성공률의 분모로 사용하지 않는다. include=false인 실패도
평가 시도 수에 남긴다.

| 항목 | 건수 | 연결된 failure ID 또는 집계 근거 |
|---|---:|---|
| 전체 실패 에피소드 | | |
| 원인 확정 | | |
| 원인 미확정 | | |
| 시스템 수정 후 재시험 | | |
| 데이터 보강·재학습 결정 | | |
| HIL 복구 데이터 전환 | | |

| 확인 원인 계층 | 건수 | 대표 root cause code |
|---|---:|---|
| navigation·localization | | |
| tf·time | | |
| perception·measurement calibration | | |
| ik·mechanical | | |
| policy data·runtime | | |
| control runtime·hardware | | |
| simulation model | | |

- 실패 원본의 기본 저장 위치:
- 실패 레코드 스키마 버전:
- 같은 조건 재시험 protocol:
- 회귀 평가 protocol:
- BC에 포함한 HIL 파생 dataset ID·원본 episode·복구 시작/종료·성공 근거:
- 실패 데이터 중 eval-only·failure bank로 남긴 범위와 이유:

## 알려진 한계

이 데이터로 학습한 정책이 못 할 것으로 예상되는 것을 적는다. 조건 분포에 없는 것은 전부 여기 해당한다.

## 라이선스·출처
