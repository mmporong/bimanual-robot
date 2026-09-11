# 양팔 IL 데이터 수집·Hugging Face 업로드·다른 PC 학습 인계

작성 기준일: 2026-09-11

대상 저장소: `~/bimanual-robot`

LeRobot 기준: `0.6.1`, upstream commit `7e241bd630a3719a56157a497ce5d08f244784f1`

이 문서는 새 팀원이 문서 전체를 AI에게 전달했을 때 환경 확인, 양팔 텔레옵 smoke 수집,
비공개 Hugging Face 업로드, 에피소드 검수, 다른 GPU PC에서 ACT 학습까지 같은 계약으로
진행하게 만드는 실행 원본이다. 이 문서의 명령에 있는 `<...>` 값은 현장에서 확인한 뒤
치환한다. 포트·카메라 번호를 추측해서 실행하지 않는다.

## 0. 이 문서를 받은 AI의 작업 계약

AI는 다음 순서와 경계를 지킨다.

1. `~/bimanual-robot`의 `main`과 작업 트리 상태를 읽고 기존 변경을 보존한다.
2. LeRobot 버전, 패치 상태, 시리얼 by-id, 카메라 by-path, 캘리브레이션 JSON을 읽기 전용으로
   확인한다.
3. 실물 팔이 움직이는 `lerobot-teleoperate`, `lerobot-record`, `lerobot-replay`, rollout은
   운영자가 해당 실행을 명시하고 안전 구역·비상 전원 차단 준비가 확인된 경우에만 시작한다.
4. 최초 수집은 물 없이 2~3개 smoke episode만 기록한다. smoke가 검증되기 전에는 본수집이나
   실제 물 붓기를 시작하지 않는다.
5. 녹화 중 Hub로 프레임을 직접 쓰지 않는다. 로컬 SSD를 원본으로 두고, 프로세스가
   `finalize()`한 완결 데이터만 업로드한다.
6. Hub 저장소는 `private=true`로 만든다. 토큰을 명령, 문서, Git, 로그에 기록하지 않는다.
7. policy 1과 policy 2, smoke와 본수집, 성공 시연과 HIL 복구, 실물과 시뮬레이션 데이터를
   서로 다른 dataset ID로 분리한다. 정의되지 않은 `policy 3`은 수집하지 않는다.
8. 실패 에피소드를 성공 BC 데이터에 넣거나 삭제하지 않는다. sidecar와 failure bank에
   보존하고 포함·제외 판정을 남긴다.
9. 다른 PC 학습은 Hub의 전체 commit SHA를 `dataset.revision`으로 고정한다. 움직이는 `main`
   또는 `v3.0` 데이터 태그만으로 재현성을 주장하지 않는다.
10. 검증 실패 시 원인을 확인하지 않은 반복 실행, 재캘리브레이션, 서보 레지스터 변경,
    공개 업로드로 우회하지 않는다.

### 완료 조건

- 로컬 LeRobot 데이터셋을 다시 열어 첫·중간·마지막 episode의 RGB, state, action, timestamp를
  확인했다.
- 모든 episode sidecar가 전체 JSON Schema 검증을 통과했다.
- 학습 포함·제외 수와 제외 사유가 기록됐다.
- private Hub 데이터셋의 episode 수와 로컬 수가 일치한다.
- Hub dataset commit SHA, 프로젝트 Git commit, LeRobot commit, 캘리브레이션 파일 SHA-256이
  수집 기록에 남았다.
- failure bundle이 있으면 별도 private failure repo의 SHA와 evidence hash 검증이 남았다.
- train·include episode만 가진 파생 dataset과 재집계한 stats의 full SHA가 남았다.
- 다른 PC의 ACT 학습이 그 파생 dataset SHA를 읽고 checkpoint를 생성했다.

## 1. 프로젝트에서 IL이 맡는 범위

IL/ACT는 양팔 조작만 담당한다. SLAM, Nav2 이동, 충전소 복귀, 비상 정지, 최종 성공 판정은
학습 대상이 아니다. 팔 조작 중에는 베이스가 정지하고, 베이스 이동 중에는 팔이 운반·정지
자세를 유지한다.

### policy 1: 주방 양팔 조작

```text
CUP_PICK
→ JUG_PICK
→ MOVE_TO_PREPOUR
→ POUR
→ JUG_RETURN
→ SHELF_PLACE
```

- 왼팔: 컵 파지·지지·선반 배치
- 오른팔: 물통 파지·붓기·원위치
- 첫 smoke는 물 없이 빈 컵·빈 물통으로 한다.
- 고정 task instruction:

```text
Pick up the cup with the left arm, pick up the water jug with the right arm,
pour into the cup, return the jug, and place the cup on the robot shelf.
```

### policy 2: 테이블 서빙 조작

- 왼팔이 선반의 컵을 집어 손님 테이블에 놓는다.
- 오른팔을 사용하지 않아도 양팔 state/action 키는 제거하지 않고 정지 상태로 기록한다.
- 이는 회의 본문의 실제 동작 서술을 따른 **smoke용 임시 기준**이다. 회의 괄호의
  `오른팔 ACT policy2` 표기와 충돌하므로 policy 2 본수집 전에는 팀 책임자가 사용 팔을
  확정해야 한다. 확정 전에는 왼팔 smoke와 오른팔 smoke를 같은 dataset에 섞지 않는다.
- 고정 task instruction:

```text
Pick up the cup from the robot shelf with the left arm and place it on the customer table.
```

Nav2로 주방이나 테이블까지 이동한 구간을 위 데이터셋에 붙이지 않는다.

## 2. 데이터셋 이름과 분리 규칙

Hub ID는 `<hf-owner>/<dataset-name>` 형식이다. 아래 이름을 기준으로 사용하고, 다른 의미의
데이터를 같은 repo에 추가하지 않는다.

| 목적 | dataset name |
|---|---|
| policy 1 smoke | `water-kitchen-full-demo-smoke-v0` |
| policy 1 본수집 | `water-kitchen-full-demo-v1` |
| policy 1 phase 파생 | `water-kitchen-phase-<phase>-v1` |
| policy 2 smoke | `water-table-full-demo-smoke-v0` |
| policy 2 본수집 | `water-table-full-demo-v1` |
| strategy 평가 | `water-kitchen-strategy-eval-v1` |
| HIL 복구 | `water-kitchen-hil-recovery-v1` |

LeRobot 0.6.1은 신규 수집 시 기본적으로 repo ID에 시각 suffix를 붙인다. 팀 이름을 그대로
유지하려면 모든 수집 명령에 `--dataset.no_stamp=true`를 명시한다.

## 3. 새 수집 PC 준비

### 3.1 프로젝트 저장소

```bash
test -d "$HOME/bimanual-robot/.git" || \
  git clone git@github.com:mmporong/bimanual-robot.git "$HOME/bimanual-robot"

git -C "$HOME/bimanual-robot" status --short --branch
test "$(git -C "$HOME/bimanual-robot" branch --show-current)" = "main" || {
  echo "main이 아닌 브랜치이므로 pull을 중단함" >&2
  return 1 2>/dev/null || exit 1
}
test -z "$(git -C "$HOME/bimanual-robot" status --porcelain)" || {
  echo "기존 작업이 있어 pull을 중단함" >&2
  return 1 2>/dev/null || exit 1
}
git -C "$HOME/bimanual-robot" pull --ff-only origin main
```

작업 트리에 기존 변경이 있으면 pull, checkout, reset으로 덮지 말고 변경 경로를 먼저 보고한다.

### 3.2 LeRobot 0.6.1 설치

이미 설치됐다면 새로 설치하지 말고 버전부터 확인한다.

```bash
test -x "$HOME/miniforge3/envs/lerobot/bin/python"
"$HOME/miniforge3/envs/lerobot/bin/python" - <<'PY'
import lerobot
print("lerobot_version=", lerobot.__version__)
print("lerobot_path=", lerobot.__file__)
PY
git -C "$HOME/lerobot" status --short --branch
git -C "$HOME/lerobot" rev-parse HEAD
```

신규 설치 기준은 다음과 같다. 기존 `~/lerobot`이 있으면 삭제하거나 덮어쓰지 않는다.

```bash
git clone https://github.com/huggingface/lerobot.git "$HOME/lerobot"
git -C "$HOME/lerobot" checkout v0.6.1

conda create -y -n lerobot python=3.12
conda activate lerobot
conda install -y ffmpeg -c conda-forge
cd "$HOME/lerobot"
python -m pip install -e ".[core_scripts,feetech]" jsonschema
```

버전 PASS 기준은 Python 패키지 `0.6.1`과 Git commit
`7e241bd630a3719a56157a497ce5d08f244784f1`이다. 프로젝트가 검증한 별도 로컬 commit을
사용한다면 commit SHA와 이유를 수집 기록에 남긴다.

### 3.3 양팔 그리퍼 패치

현행 하드웨어는 왼팔 스톡 그리퍼, 오른팔 ggao50 평행그리퍼다. 오른팔은 LeRobot 기본
그리퍼 보호값으로 닫히지 않을 수 있고, 양팔 wrapper는 좌우 `gripper_protection` 설정을
전달하도록 패치해야 한다.

```bash
cd "$HOME/lerobot"

git apply --reverse --check \
  "$HOME/bimanual-robot/tools/servo/lerobot_gripper_protection.patch" \
  && echo "단일팔 패치: 이미 적용" \
  || git apply "$HOME/bimanual-robot/tools/servo/lerobot_gripper_protection.patch"

git apply --reverse --check \
  "$HOME/bimanual-robot/tools/servo/bi_so_follower_gripper_protection.patch" \
  && echo "양팔 패치: 이미 적용" \
  || git apply "$HOME/bimanual-robot/tools/servo/bi_so_follower_gripper_protection.patch"
```

`git apply`와 reverse check가 모두 실패하면 소스 버전이 다른 것이다. 강제로 고치지 말고
LeRobot commit과 diff를 보고한다. 두 패치는 공식 v0.6.1 tag에 적용 가능한 것을 2026-09-11
확인했다.

## 4. Hugging Face 인증과 로컬 원본 위치

수집 PC에는 dataset write 권한 토큰, 학습 PC에는 private dataset read 권한 토큰만 둔다.
토큰 문자열을 채팅, 셸 기록용 스크립트, `.env`, Git 파일에 넣지 않는다.

팀원이 서로 다른 Hugging Face 계정을 쓴다면 dataset을 팀 organization namespace 아래 만들고
각자 자기 계정의 최소 권한 토큰으로 로그인한다. 개인 namespace의 업로더 토큰을 팀원에게
복사하지 않는다. 수집 전에 학습 담당자의 계정이 private dataset을 읽을 수 있는지 확인한다.

```bash
"$HOME/miniforge3/envs/lerobot/bin/hf" auth login
"$HOME/miniforge3/envs/lerobot/bin/hf" auth whoami
```

학습 담당자 PC의 읽기 권한 smoke는 데이터가 생긴 뒤 다음 명령으로 확인한다.

```bash
"$HOME/miniforge3/envs/lerobot/bin/python" - "<hf-owner>/<dataset-name>" <<'PY'
import sys
from huggingface_hub import HfApi

info = HfApi().dataset_info(sys.argv[1])
print(f"dataset_revision={info.sha}")
print(f"private={info.private}")
PY
```

권한 오류가 나면 업로더의 write 토큰을 공유하지 말고 organization 멤버십과 dataset 접근
권한을 고친다.

세션 변수 예시:

```bash
export HF_OWNER="<hf-owner>"
export DATASET_NAME="water-kitchen-full-demo-smoke-v0"
export DATASET_ID="${HF_OWNER}/${DATASET_NAME}"
export DATASET_ROOT="$HOME/lerobot-data/${DATASET_NAME}"
export LEROBOT_BIN="$HOME/miniforge3/envs/lerobot/bin"
export TASK_INSTRUCTION="Pick up the cup with the left arm, pick up the water jug with the right arm, pour into the cup, return the jug, and place the cup on the robot shelf."
export COLLECTION_MODE="new"
```

policy 2 smoke를 수집할 때는 다음 네 값을 바꾼다. 그 뒤에 나오는 공통 preflight를 실행한다.

```bash
export DATASET_NAME="water-table-full-demo-smoke-v0"
export DATASET_ID="${HF_OWNER}/${DATASET_NAME}"
export DATASET_ROOT="$HOME/lerobot-data/${DATASET_NAME}"
export TASK_INSTRUCTION="Pick up the cup from the robot shelf with the left arm and place it on the customer table."
```

로컬 경로와 Hub repo를 다음 공통 preflight로 확인한다. `new`는 같은 이름이 로컬이나 Hub에
하나라도 있으면 중단하고 새 dataset version을 정한다. `resume`은 완결 메타가 있는 같은 로컬
root와 이미 private인 같은 Hub repo를 모두 요구한다. 기존 public repo에는 절대 이어 쓰지 않는다.

```bash
mkdir -p "$HOME/lerobot-data"
case "$COLLECTION_MODE" in
  new)
    test ! -e "$DATASET_ROOT" || {
      echo "신규 수집 루트가 이미 존재함: $DATASET_ROOT" >&2
      return 1 2>/dev/null || exit 1
    }
    ;;
  resume)
    test -s "$DATASET_ROOT/meta/info.json" || {
      echo "재개할 로컬 meta/info.json이 없음: $DATASET_ROOT" >&2
      return 1 2>/dev/null || exit 1
    }
    ;;
  *)
    echo "COLLECTION_MODE은 new 또는 resume여야 함" >&2
    return 1 2>/dev/null || exit 1
    ;;
esac

"$LEROBOT_BIN/python" - "$DATASET_ID" "$COLLECTION_MODE" <<'PY'
import sys
from huggingface_hub import HfApi
from huggingface_hub.errors import RepositoryNotFoundError

dataset_id, mode = sys.argv[1:]
api = HfApi()
try:
    info = api.dataset_info(dataset_id)
except RepositoryNotFoundError:
    info = None

if mode == "new":
    if info is not None:
        raise SystemExit(f"Hub dataset ID가 이미 존재함: {dataset_id}")
    api.create_repo(dataset_id, repo_type="dataset", private=True, exist_ok=False)
    info = api.dataset_info(dataset_id)
elif mode == "resume":
    if info is None:
        raise SystemExit(f"재개할 Hub dataset이 없음: {dataset_id}")

if info.private is not True:
    raise SystemExit(f"private가 아닌 dataset에는 수집 금지: {dataset_id}")
print(f"hub_preflight=PASS dataset_id={dataset_id} private={info.private}")
PY
```

신규 모드는 녹화 전에 빈 private repo를 먼저 만든다. 이렇게 해야 같은 ID의 기존 public
repo에 자동 업로드되는 일을 막을 수 있다. 녹화가 시작되지 않아 빈 private repo만 남았다면
데이터를 올린 것으로 기록하지 말고, 다음 시도는 `resume`이 아니라 원인 확인 후 같은 ID의
빈 repo를 명시적으로 정리하거나 새 version을 정한다.

`DATASET_ROOT`는 첫 수집부터 명시한다. 재개 시 같은 경로가 필수이며, Hub는 백업·공유 원격이고
수집 중 원본은 로컬 SSD다. LeRobot 0.6.1의 신규 데이터셋 생성기는 root를 직접 만들므로
첫 수집 전에 `DATASET_ROOT` 자체를 `mkdir`로 만들지 않는다.

## 5. 실물 연결값을 읽기 전용으로 확정

### 5.1 시리얼 포트

```bash
ls -l /dev/serial/by-id/
```

`/dev/ttyACM0`처럼 꽂는 순서에 따라 바뀌는 이름을 쓰지 않는다. 다음 네 값을 실제 by-id로
기록한다.

```bash
export LEFT_FOLLOWER_PORT="<왼쪽 팔로워 by-id>"
export RIGHT_FOLLOWER_PORT="<오른쪽 팔로워 by-id>"
export LEFT_LEADER_PORT="<왼쪽 리더 by-id>"
export RIGHT_LEADER_PORT="<오른쪽 리더 by-id>"
```

포트 하나를 두 프로세스가 동시에 열면 안 된다. 텔레옵·녹화 중에는 servo 도구, MoveIt,
다른 LeRobot 프로세스를 함께 실행하지 않는다.

### 5.2 양팔 캘리브레이션

양팔 수집은 단일팔 JSON이 아니라 다음 네 파일을 사용한다.

```text
~/bimanual-robot/calibration/bi_follower/arms_left.json
~/bimanual-robot/calibration/bi_follower/arms_right.json
~/bimanual-robot/calibration/bi_leader/arms_left.json
~/bimanual-robot/calibration/bi_leader/arms_right.json
```

읽기 전용 대조:

```bash
cd "$HOME/bimanual-robot/tools/servo"
"$LEROBOT_BIN/python" servo_check_calibration.py \
  --port "$LEFT_FOLLOWER_PORT" \
  --json "$HOME/bimanual-robot/calibration/bi_follower/arms_left.json"
"$LEROBOT_BIN/python" servo_check_calibration.py \
  --port "$RIGHT_FOLLOWER_PORT" \
  --json "$HOME/bimanual-robot/calibration/bi_follower/arms_right.json"
"$LEROBOT_BIN/python" servo_check_calibration.py \
  --port "$LEFT_LEADER_PORT" \
  --json "$HOME/bimanual-robot/calibration/bi_leader/arms_left.json"
"$LEROBOT_BIN/python" servo_check_calibration.py \
  --port "$RIGHT_LEADER_PORT" \
  --json "$HOME/bimanual-robot/calibration/bi_leader/arms_right.json"
```

네 개가 모두 `결과: 일치`여야 한다. 불일치가 나면 `c`를 눌러 즉석 재캘리브레이션하지 않는다.
이 검사는 offset·min·max·현재 위치만 대조하며 Phase는 검사하지 않는다. 오른쪽 follower의
ggao50 그리퍼(ID 6)는 다음 읽기 전용 검사를 별도로 통과해야 한다.

```bash
cd "$HOME/bimanual-robot/tools/servo"
"$LEROBOT_BIN/python" servo_check_phase.py \
  --port "$RIGHT_FOLLOWER_PORT" \
  --id 6 \
  --expected 76
```

출력이 `Phase=76, expected=76 — 일치`여야 한다. 12 또는 다른 값이면 실물 이동을 시작하지
않고 관측값과 포트·서보 ID를 보고한다. 형제 서보 값을 기준으로 맞추거나 이 도구로 값을
쓰지 않는다. 이 도구는 읽기만 한다.

재캘리브레이션이 필요한 경우
[`20260910_텔레옵_IL_사용법.md`](20260910_텔레옵_IL_사용법.md)의 4절을 따른다.

### 5.3 카메라

```bash
"$LEROBOT_BIN/lerobot-find-cameras" opencv
ls -l /dev/v4l/by-path/ 2>/dev/null || true
```

본수집 v1의 목표 key는 다음과 같다.

| 위치 | 고정 key | 입력 |
|---|---|---|
| 후방 중앙 마스트 | `mast` | RGB 사용, depth는 정렬·로그용이며 첫 ACT 입력에서 제외 |
| 왼손목 | `left_wrist` | RGB |
| 오른손목 | `right_wrist` | RGB |

카메라 key, 해상도 `640×480`, fps `30`, crop과 관절 순서는 dataset version 안에서 바꾸지
않는다. 카메라가 아직 하나뿐이면 한 카메라 smoke dataset은 만들 수 있지만, 3시점 본수집
dataset과 섞지 않는다.

```bash
export MAST_CAMERA="<마스트 카메라 /dev/v4l/by-path/...>"
export LEFT_WRIST_CAMERA="<왼손목 카메라 /dev/v4l/by-path/...>"
export RIGHT_WRIST_CAMERA="<오른손목 카메라 /dev/v4l/by-path/...>"
```

### 5.4 본수집 전에 scenario·holdout 고정

smoke는 파이프라인 확인용이라 이 단계의 평가 표본으로 쓰지 않는다. 본수집을 시작하기 전에
`~/bimanual-robot/data/collection_plans/<dataset-name>.json`을 만들고 프로젝트 Git에 커밋한다.
AI는 다음 필드가 실제 값으로 채워지고 commit SHA가 정해지기 전에는 본수집을 실행하지 않는다.

```json
{
  "dataset_id": "<hf-owner>/water-kitchen-full-demo-v1",
  "plan_version": "v1",
  "created_before_collection": true,
  "split_rule": "scenario_id의 사전 고정 목록",
  "scenarios": [
    {
      "scenario_id": "kitchen-fixed-dry-001",
      "split": "train",
      "planned_episode_count": 5,
      "cup_id": "<실물 ID>",
      "jug_id": "<실물 ID>",
      "station_id": "kitchen",
      "lighting": "indoor_fluorescent"
    },
    {
      "scenario_id": "kitchen-holdout-001",
      "split": "holdout",
      "planned_episode_count": 3,
      "cup_id": "<실물 ID>",
      "jug_id": "<실물 ID>",
      "station_id": "kitchen",
      "lighting": "indoor_fluorescent"
    }
  ]
}
```

- `split`은 성공·실패 결과를 보기 전에 scenario 단위로 정한다.
- 같은 `scenario_id`의 episode를 train과 holdout 양쪽에 넣지 않는다.
- 수집 결과가 나쁘다는 이유로 holdout을 train으로 바꾸지 않는다.
- 각 episode sidecar의 `scenario_id`와 `split`은 이 계획과 일치해야 한다.
- 계획을 바꾸면 이유와 새 commit을 남기고, 이미 본 결과가 있는 표본을 새 holdout으로
  재사용하지 않는다.

## 6. 실물 이동 전 dry teleop 게이트

다음 조건을 운영자와 확인한 뒤에만 실물 텔레옵을 시작한다.

- 팔 작업 반경에 사람, 케이블, 컵, 물통 외 장애물이 없다.
- 전원을 즉시 끌 담당자가 팔 옆에 있다.
- 두 리더 팔과 팔로워 팔의 시작 자세가 비슷하고 그리퍼는 열려 있다.
- 왼팔 스톡 그리퍼에만 `gripper_protection=true`를 사용한다.
- 오른팔 ggao50에는 `gripper_protection=true`를 넣지 않는다.
- 종료 시 토크가 풀려 팔이 떨어질 수 있으므로 받침 위치가 준비됐다.

실행 형식:

```bash
"$LEROBOT_BIN/lerobot-teleoperate" \
  --robot.type=bi_so_follower --robot.id=arms \
  --robot.calibration_dir="$HOME/bimanual-robot/calibration/bi_follower" \
  --robot.left_arm_config.port="$LEFT_FOLLOWER_PORT" \
  --robot.left_arm_config.gripper_protection=true \
  --robot.right_arm_config.port="$RIGHT_FOLLOWER_PORT" \
  --teleop.type=bi_so_leader --teleop.id=arms \
  --teleop.calibration_dir="$HOME/bimanual-robot/calibration/bi_leader" \
  --teleop.left_arm_config.port="$LEFT_LEADER_PORT" \
  --teleop.right_arm_config.port="$RIGHT_LEADER_PORT"
```

정상 기준은 방향이 맞고, 두 팔이 튀지 않으며, 그리퍼가 물체 없이 열림·닫힘을 수행하고,
제어 주기가 지속해서 밀리지 않는 것이다. 이상이 있으면 즉시 종료하고 수집으로 넘어가지 않는다.

## 7. smoke episode 2~3개 수집

### 7.1 수집 명령

아래는 policy 1, RGB 3시점 기준이다. 실제 연결된 장치 경로만 치환한다. 이 명령은 운영자의
명시적 실물 실행 요청과 6절 게이트 통과 뒤 실행한다.

```bash
"$LEROBOT_BIN/lerobot-record" \
  --robot.type=bi_so_follower --robot.id=arms \
  --robot.calibration_dir="$HOME/bimanual-robot/calibration/bi_follower" \
  --robot.left_arm_config.port="$LEFT_FOLLOWER_PORT" \
  --robot.left_arm_config.gripper_protection=true \
  --robot.left_arm_config.cameras="{ wrist: {type: opencv, index_or_path: $LEFT_WRIST_CAMERA, width: 640, height: 480, fps: 30} }" \
  --robot.right_arm_config.port="$RIGHT_FOLLOWER_PORT" \
  --robot.right_arm_config.cameras="{ wrist: {type: opencv, index_or_path: $RIGHT_WRIST_CAMERA, width: 640, height: 480, fps: 30} }" \
  --robot.cameras="{ mast: {type: opencv, index_or_path: $MAST_CAMERA, width: 640, height: 480, fps: 30} }" \
  --teleop.type=bi_so_leader --teleop.id=arms \
  --teleop.calibration_dir="$HOME/bimanual-robot/calibration/bi_leader" \
  --teleop.left_arm_config.port="$LEFT_LEADER_PORT" \
  --teleop.right_arm_config.port="$RIGHT_LEADER_PORT" \
  --dataset.repo_id="$DATASET_ID" \
  --dataset.root="$DATASET_ROOT" \
  --dataset.no_stamp=true \
  --dataset.single_task="$TASK_INSTRUCTION" \
  --dataset.num_episodes=3 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=15 \
  --dataset.fps=30 \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --dataset.push_to_hub=true \
  --dataset.private=true \
  --display_data=true
```

키 동작:

| 키 | 의미 |
|---|---|
| `→` 또는 `n` | 현재 episode를 일찍 끝내고 저장 |
| `←` 또는 `r` | 현재 episode를 버리고 다시 기록 |
| `ESC` 또는 `q` | 전체 녹화를 끝내고 finalize·업로드 |

실패했다고 무조건 `←`로 버리지 않는다. 조작자 실수로 학습 가치가 없는 미완성 buffer는
다시 기록할 수 있지만, 시스템·파지·붓기 실패는 episode로 저장하고 `include=false` sidecar와
failure bank 근거를 남긴다.

### 7.2 업로드가 일어나는 시점

`streaming_encoding=true`는 촬영 중 영상 인코딩이다. Hub 스트리밍 업로드가 아니다.

```text
프레임 수집
→ dataset.save_episode()
→ 녹화 종료
→ dataset.finalize()
→ dataset.push_to_hub(private=true)
```

열려 있는 Parquet·MP4 shard를 별도 프로세스로 업로드하지 않는다. Wi-Fi가 끊겨 종료 업로드가
실패해도 로컬 `DATASET_ROOT`를 삭제하거나 새 이름으로 다시 수집하지 않는다.

## 8. smoke 데이터 검수

### 8.1 로컬 구조와 시각화

```bash
test -s "$DATASET_ROOT/meta/info.json"
find "$DATASET_ROOT" -maxdepth 4 -type f | sort | sed -n '1,120p'

"$LEROBOT_BIN/lerobot-dataset-viz" \
  --repo-id "$DATASET_ID" \
  --root "$DATASET_ROOT" \
  --episode-index 0
```

첫·중간·마지막 episode에서 확인한다.

- `mast`, `left_wrist`, `right_wrist` 영상이 같은 사건을 가리키는가
- 왼팔·오른팔 state와 action의 관절 순서가 고정됐는가
- 그리퍼 열림·닫힘 부호가 실제 영상과 맞는가
- episode 시작·끝에 긴 정지 구간이 없는가
- 목표 30fps 대비 실제 기록 frame 수·drop 비율과 제어 주기 경고가 얼마인가
- 영상·state·action timestamp가 대응하는가

frame drop 합격 임계값은 아직 팀에서 실측으로 승인하지 않았다. 임의의 `5%` 같은 값을
통과 기준으로 만들지 말고, episode별 기대/실제 frame 수, 계산한 비율, LeRobot 주기 초과
경고 횟수를 sidecar의 anomaly·notes와 collection provenance에 기록한다. 경고가 있으면 자동
학습 포함으로 두지 않고 원인 분리 후 팀 기준으로 판정한다.

### 8.2 episode sidecar

LeRobot 표준 메타만으로는 컵·물통·조명·phase·성공·실패·캘리브레이션을 복원할 수 없다.
각 episode에 다음 경로로 sidecar를 만든다.

```text
$DATASET_ROOT/sidecars/episodes/episode_000000.json
$DATASET_ROOT/sidecars/episodes/episode_000001.json
...
```

원본 예시는 `~/bimanual-robot/data/schema/example_episode_meta.json`, 스키마는
`~/bimanual-robot/data/schema/episode_metadata.schema.json`이다. 예시의 `example-*` 값과
가상 수치는 실제 관측값으로 바꾼다. 모르는 값을 추측해 채우지 않는다.

```bash
cd "$HOME/bimanual-robot"
"$LEROBOT_BIN/python" tools/validate_episode_meta.py "$DATASET_ROOT/sidecars/episodes"
```

`jsonschema` 미설치로 축소 검증 경고가 나오면 PASS가 아니다.

```bash
"$HOME/miniforge3/envs/lerobot/bin/python" -m pip install jsonschema
"$HOME/miniforge3/envs/lerobot/bin/python" \
  "$HOME/bimanual-robot/tools/validate_episode_meta.py" \
  "$DATASET_ROOT/sidecars/episodes"
```

검증기가 발견한 JSON만 통과시키고 누락 episode를 놓치지 않도록, LeRobot 총 episode와
sidecar index의 완전성도 별도로 검사한다.

```bash
"$HOME/miniforge3/envs/lerobot/bin/python" - "$DATASET_ROOT" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
info = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))
expected = set(range(info["total_episodes"]))
paths = sorted((root / "sidecars" / "episodes").glob("episode_*.json"))
indices = []
bad_names = []
for path in paths:
    item = json.loads(path.read_text(encoding="utf-8"))
    index = item["episode_index"]
    indices.append(index)
    if path.name != f"episode_{index:06d}.json":
        bad_names.append({"file": path.name, "episode_index": index})
found = set(indices)
missing = sorted(expected - found)
extra = sorted(found - expected)
duplicates = sorted({index for index in indices if indices.count(index) > 1})
if missing or extra or duplicates or bad_names or len(paths) != len(expected):
    raise SystemExit(
        "sidecar 불완전: "
        f"missing={missing}, extra={extra}, duplicates={duplicates}, "
        f"bad_names={bad_names}, files={len(paths)}, episodes={len(expected)}"
    )
print(f"sidecar_complete={len(found)}/{len(expected)}")
PY
```

수집 세션 단위 provenance도 같은 데이터셋에 둔다. AI는 아래 명령의 출력으로
`$DATASET_ROOT/sidecars/collection_session.yaml`을 작성하되 토큰은 넣지 않는다.

```bash
git -C "$HOME/bimanual-robot" rev-parse HEAD
git -C "$HOME/lerobot" rev-parse HEAD
sha256sum \
  "$HOME/bimanual-robot/calibration/bi_follower/arms_left.json" \
  "$HOME/bimanual-robot/calibration/bi_follower/arms_right.json" \
  "$HOME/bimanual-robot/calibration/bi_leader/arms_left.json" \
  "$HOME/bimanual-robot/calibration/bi_leader/arms_right.json"
```

기록 항목은 수집자, 수집 시작·종료 시각, 프로젝트/LeRobot commit, 네 calibration SHA-256,
네 by-id, 세 camera key·by-path, dataset ID, task instruction, fps·해상도, episode 수다.
실패 bundle이 생겼다면 failure repo ID와 해당 full commit SHA도 기록한다.

## 9. 실패 episode 저장과 학습 포함 판정

### 9.1 바로 제외하는 원본

- 최종 태스크 실패
- servo stall, 충돌, 캘리브레이션 불일치
- 제어 주기가 밀린 teleop desync
- frame drop 또는 반복되는 제어 주기 초과 경고
- 조작자 실수 재시도 구간
- 필수 조건 메타데이터 또는 성공 근거 누락
- policy·phase·action 표현·캘리브레이션 추적값 누락

제외는 삭제가 아니다. sidecar에 `include=false`, `exclude_reason`을 기록하고 원본 episode는
failure bank와 회귀 평가에 남긴다. `CUP_GRASP_MISS`, `UNDER_FILL`, `TIMEOUT`은 관측 증상이지
확정 원인이 아니다.

### 9.2 실패 근거 캡처

실제 실패 입력 JSON을 `data/schema/example_failure_capture.json` 형식으로 작성하고, 원본 영상,
관절·action window, 센서 로그를 evidence로 지정한다.

```bash
cd "$HOME/bimanual-robot"
"$LEROBOT_BIN/python" tools/failure_bank.py capture \
  --input <실제_failure_capture.json> \
  --evidence <영상_or_로그_1> <관절_action_로그_2> \
  --root "$HOME/robot-artifacts/failures"

"$LEROBOT_BIN/python" tools/validate_failure_record.py "$HOME/robot-artifacts/failures"
"$LEROBOT_BIN/python" tools/failure_bank.py verify-evidence "$HOME/robot-artifacts/failures"
"$LEROBOT_BIN/python" tools/validate_episode_meta.py "$DATASET_ROOT/sidecars/episodes" \
  --failure-record-dir "$HOME/robot-artifacts/failures"
"$LEROBOT_BIN/python" tools/failure_bank.py summary "$HOME/robot-artifacts/failures"
```

사람이 실패 직전부터 개입해 성공시킨 구간은 원본 성공 시연과 섞지 않고 별도 HIL dataset에
넣는다. 시스템 결함이 원인이면 데이터를 늘리지 말고 시스템을 고친 뒤 같은 조건으로 재시험한다.

### 9.3 실패 bundle을 팀과 공유

`$HOME/robot-artifacts/failures`는 수집 PC 로컬 원본이다. 학습 PC가 성공 BC를 읽는 데는
필요하지 않지만, 다른 팀원이 원인 분석·재시험을 이어가려면 별도 private dataset repo에
동기화한다. 얼굴·이름·화면·장소 정보 등 불필요한 개인정보가 evidence에 없는지 먼저 검토한다.

```bash
export FAILURE_REPO_ID="${HF_OWNER}/water-serving-failure-bank-v1"

"$LEROBOT_BIN/python" - "$FAILURE_REPO_ID" <<'PY'
import sys
from huggingface_hub import HfApi
from huggingface_hub.errors import RepositoryNotFoundError

dataset_id = sys.argv[1]
api = HfApi()
try:
    info = api.dataset_info(dataset_id)
except RepositoryNotFoundError:
    api.create_repo(dataset_id, repo_type="dataset", private=True, exist_ok=False)
    info = api.dataset_info(dataset_id)
if info.private is not True:
    raise SystemExit(f"private가 아닌 failure repo에는 업로드 금지: {dataset_id}")
PY

"$LEROBOT_BIN/hf" upload \
  "$FAILURE_REPO_ID" "$HOME/robot-artifacts/failures" \
  --repo-type dataset \
  --private

export FAILURE_REVISION=$("$LEROBOT_BIN/python" - "$FAILURE_REPO_ID" <<'PY'
import sys
from huggingface_hub import HfApi
print(HfApi().dataset_info(sys.argv[1]).sha)
PY
)
printf 'FAILURE_REPO_ID=%s\nFAILURE_REVISION=%s\n' \
  "$FAILURE_REPO_ID" "$FAILURE_REVISION"
```

다른 PC에서 이어서 분석할 때는 SHA를 고정해 받은 뒤 증거 해시를 다시 검사한다.

```bash
export FAILURE_ROOT="$HOME/robot-artifacts/failures-${FAILURE_REVISION}"
"$LEROBOT_BIN/hf" download "$FAILURE_REPO_ID" \
  --repo-type dataset \
  --revision "$FAILURE_REVISION" \
  --local-dir "$FAILURE_ROOT"
"$LEROBOT_BIN/python" "$HOME/bimanual-robot/tools/failure_bank.py" verify-evidence "$FAILURE_ROOT"
```

episode sidecar의 `failure_record_id`, collection session의 `FAILURE_REPO_ID`와
`FAILURE_REVISION`을 함께 써야 원격에서도 실패 근거를 찾을 수 있다.

## 10. Hub 업로드 확인과 재시도

4절 preflight에서 private Hub repo를 먼저 만든다. 수집이 정상 종료되면 LeRobot이 그 repo로
자동 업로드한다. 8절에서 sidecar와 collection provenance를 추가하고 스키마 검증까지 통과한
뒤에는 아래 명령을 한 번 실행해 최종 로컬 상태를 동기화한다. 최초 자동 업로드가 실패했을
때도 완결된 로컬 dataset에 같은 명령을 쓴다.

```bash
"$LEROBOT_BIN/hf" upload "$DATASET_ID" "$DATASET_ROOT" \
  --repo-type dataset \
  --private
```

같은 호출을 다시 실행하면 이미 올라간 파일과 chunk를 건너뛰어 재개할 수 있다. 단,
`finalize()`되지 않은 수집 디렉터리에는 이 명령을 사용하지 않는다.

원격 commit SHA 확인:

```bash
"$LEROBOT_BIN/python" - "$DATASET_ID" <<'PY'
import sys
from huggingface_hub import HfApi

dataset_id = sys.argv[1]
info = HfApi().dataset_info(dataset_id)
print(f"dataset_id={dataset_id}")
print(f"dataset_revision={info.sha}")
print(f"private={info.private}")
PY
```

`private=True`가 아니면 학습 PC 공유를 진행하지 않는다. Hub에서 episode 수, 영상 key와
로컬 sidecar가 보이는지 확인한다. 수집 시작 뒤에는
`~/bimanual-robot/data/README.md`의 저장 위치 인덱스에 dataset ID, episode 수, 수집 기간,
상태를 기록한다. 아직 만들지 않은 Hub repo를 미리 등록하지 않는다.

LeRobot이 자동 생성하는 dataset card는 첫 push 뒤
`~/bimanual-robot/data/schema/dataset_card_TEMPLATE.md`의 항목으로 보강한다. 마지막 upload 뒤
자동 카드가 다시 갱신될 수 있으므로 최종 dataset release에서 카드 내용을 대조한다.

## 11. 본수집과 같은 dataset 재개

smoke를 본수집 repo로 이름만 바꾸지 않는다. smoke 검증 뒤 본수집용 `DATASET_NAME`과
`DATASET_ROOT`를 새로 만들고 첫 batch를 기록한다. 본수집은 5~10 episode 단위로 종료해서
finalize·upload한다.

같은 dataset에 5개를 추가하는 재개 명령은 7.1절의 로봇·카메라·task 옵션을 그대로 사용하고
다음 항목을 유지한다.

```bash
--dataset.repo_id="$DATASET_ID" \
--dataset.root="$DATASET_ROOT" \
--dataset.no_stamp=true \
--dataset.num_episodes=5 \
--dataset.push_to_hub=true \
--dataset.private=true \
--resume=true
```

`num_episodes=5`는 누적 목표가 아니라 이번 실행에서 추가할 5개다. 재개 전에 다음 항목이 첫
batch와 같은지 검사한다.

- LeRobot commit과 프로젝트 commit
- robot type과 action 표현
- 양팔 calibration 파일 내용과 SHA-256
- 카메라 key, 해상도, fps, crop
- task instruction
- 컵·물통·선반 버전과 scenario matrix

하나라도 의미가 바뀌면 새 dataset version을 만든다.

## 12. 다른 GPU PC에서 ACT 학습

학습 PC에는 실물 팔을 연결하지 않는다. private dataset read 권한, GPU 드라이버, 충분한 SSD
cache와 LeRobot 0.6.1만 필요하다. 신규 학습 PC는 3.2절과 같은 tag를 checkout한 뒤 하드웨어
extra 대신 학습·데이터 시각화 extra를 설치한다.

```bash
cd "$HOME/lerobot"
python -m pip install -e ".[training,dataset_viz]" jsonschema
```

### 12.1 dataset revision 고정

```bash
export HF_OWNER="<hf-owner>"
export DATASET_NAME="water-kitchen-full-demo-v1"
export DATASET_ID="${HF_OWNER}/${DATASET_NAME}"
export LEROBOT_BIN="$HOME/miniforge3/envs/lerobot/bin"

"$LEROBOT_BIN/hf" auth login
"$LEROBOT_BIN/hf" auth whoami

export DATASET_REVISION=$("$LEROBOT_BIN/python" - "$DATASET_ID" <<'PY'
import sys
from huggingface_hub import HfApi
print(HfApi().dataset_info(sys.argv[1]).sha)
PY
)
printf 'DATASET_ID=%s\nDATASET_REVISION=%s\n' "$DATASET_ID" "$DATASET_REVISION"
```

`DATASET_REVISION`은 40자리 full SHA여야 한다. 업로드 때 생성되는 `v3.0` 데이터 태그는
다음 push에서 다시 만들어질 수 있으므로 학습 고정값으로 사용하지 않는다.

### 12.2 학습 전 검수

```bash
export DATASET_AUDIT_ROOT="$HOME/lerobot-data-audit/${DATASET_NAME}-${DATASET_REVISION}"
"$LEROBOT_BIN/hf" download "$DATASET_ID" \
  --repo-type dataset \
  --revision "$DATASET_REVISION" \
  --local-dir "$DATASET_AUDIT_ROOT"

"$LEROBOT_BIN/lerobot-dataset-viz" \
  --repo-id "$DATASET_ID" \
  --root "$DATASET_AUDIT_ROOT" \
  --episode-index 0
```

Hub에서 내려받은 데이터의 feature, 카메라 key, episode 수를 수집 기록과 대조한다. sidecar의
`split=train`과 `include=true`를 모두 만족한 episode만 학습에 사용한다. holdout과 그 원본에서
자른 phase slice를 train에 넣지 않는다. 아래 명령으로 학습 episode 목록을 기계적으로 만든다.

```bash
export TRAIN_EPISODES=$("$LEROBOT_BIN/python" - "$DATASET_AUDIT_ROOT" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1]) / "sidecars" / "episodes"
selected = []
for path in sorted(root.glob("episode_*.json")):
    item = json.loads(path.read_text(encoding="utf-8"))
    if item["split"] == "train" and item["include"] is True:
        selected.append(item["episode_index"])
if not selected:
    raise SystemExit("학습 가능한 train/include=true episode가 없음")
print("[" + ",".join(map(str, selected)) + "]")
PY
)
printf 'TRAIN_EPISODES=%s\n' "$TRAIN_EPISODES"
```

목록을 손으로 다시 입력하지 않는다. 이 목록이 전체 sidecar의 판정과 일치하는지 검토한다.

`--dataset.episodes`만 원본 repo에 적용하면 샘플은 제한되지만 LeRobot 0.6.1의 normalization
통계는 원본 `meta/stats.json`을 읽는다. 그러면 제외 실패와 holdout의 state/action 통계가
학습에 새어 들어간다. 따라서 선택 episode만 가진 파생 train dataset을 만들고 그 과정에서
stats를 다시 집계한다.

```bash
export TRAIN_DATASET_ID="${DATASET_ID}_train"
export TRAIN_BUILD_ROOT="$HOME/lerobot-data-derived/${DATASET_NAME}-${DATASET_REVISION}"
export TRAIN_DATASET_ROOT="$TRAIN_BUILD_ROOT/train"
test ! -e "$TRAIN_BUILD_ROOT" || {
  echo "파생 dataset 경로가 이미 존재함: $TRAIN_BUILD_ROOT" >&2
  return 1 2>/dev/null || exit 1
}

"$LEROBOT_BIN/lerobot-edit-dataset" \
  --repo_id "$DATASET_ID" \
  --root "$DATASET_AUDIT_ROOT" \
  --new_root "$TRAIN_BUILD_ROOT" \
  --operation.type split \
  --operation.splits "{\"train\":$TRAIN_EPISODES}" \
  --push_to_hub false

"$LEROBOT_BIN/python" - \
  "$TRAIN_DATASET_ROOT" "$DATASET_ID" "$DATASET_REVISION" "$TRAIN_EPISODES" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
source_id, source_revision = sys.argv[2:4]
source_episodes = json.loads(sys.argv[4])
info = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))
if info["total_episodes"] != len(source_episodes):
    raise SystemExit("파생 dataset episode 수 불일치")
if not (root / "meta" / "stats.json").is_file():
    raise SystemExit("파생 dataset stats.json 없음")
sidecars = root / "sidecars"
sidecars.mkdir(parents=True, exist_ok=False)
(sidecars / "source_episode_map.json").write_text(
    json.dumps(
        {
            "source_dataset_id": source_id,
            "source_dataset_revision": source_revision,
            "selection": "split=train and include=true",
            "episode_map": [
                {"derived_episode_index": new, "source_episode_index": old}
                for new, old in enumerate(source_episodes)
            ],
        },
        ensure_ascii=False,
        indent=2,
    ) + "\n",
    encoding="utf-8",
)
print(f"derived_dataset=PASS episodes={len(source_episodes)}")
PY
```

`lerobot-edit-dataset split`은 선택 episode를 새 번호로 다시 만들고 해당 episode 통계를
집계해 새 `meta/stats.json`을 쓴다. 원본 dataset은 수정하지 않는다. 파생 repo도 업로드 전에
이름 충돌과 private 여부를 확정한다.

```bash
"$LEROBOT_BIN/python" - "$TRAIN_DATASET_ID" <<'PY'
import sys
from huggingface_hub import HfApi
from huggingface_hub.errors import RepositoryNotFoundError

dataset_id = sys.argv[1]
api = HfApi()
try:
    api.dataset_info(dataset_id)
except RepositoryNotFoundError:
    api.create_repo(dataset_id, repo_type="dataset", private=True, exist_ok=False)
else:
    raise SystemExit(f"파생 Hub dataset ID가 이미 존재함: {dataset_id}")
info = api.dataset_info(dataset_id)
if info.private is not True:
    raise SystemExit(f"파생 dataset이 private가 아님: {dataset_id}")
PY

"$LEROBOT_BIN/hf" upload "$TRAIN_DATASET_ID" "$TRAIN_DATASET_ROOT" \
  --repo-type dataset \
  --private

export TRAIN_DATASET_REVISION=$("$LEROBOT_BIN/python" - "$TRAIN_DATASET_ID" <<'PY'
import sys
from huggingface_hub import HfApi
print(HfApi().dataset_info(sys.argv[1]).sha)
PY
)
printf 'TRAIN_DATASET_ID=%s\nTRAIN_DATASET_REVISION=%s\n' \
  "$TRAIN_DATASET_ID" "$TRAIN_DATASET_REVISION"
```

학습 config에는 원본 dataset ID·SHA, 선택한 원본 episode 목록, 파생 dataset ID·SHA를 모두
보존한다. 파생 dataset SHA가 정해지기 전에는 학습을 시작하지 않는다.

### 12.3 ACT 학습 명령

```bash
export TRAIN_NAME="act_water_kitchen_v1"
export TRAIN_OUTPUT_DIR="$HOME/lerobot-training/outputs/$TRAIN_NAME"

"$LEROBOT_BIN/lerobot-train" \
  --dataset.repo_id="$TRAIN_DATASET_ID" \
  --dataset.revision="$TRAIN_DATASET_REVISION" \
  --policy.type=act \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --output_dir="$TRAIN_OUTPUT_DIR" \
  --job_name="$TRAIN_NAME" \
  --steps=50000 \
  --batch_size=8 \
  --save_freq=10000 \
  --seed=1000 \
  --wandb.enable=false
```

`50000` step과 batch 8은 첫 기준값이지 최적값이 아니다. CUDA OOM이 나면 dataset이나
해상도를 몰래 바꾸지 말고 batch size만 낮춰 새 run ID로 기록한다. 학습 결과에는 다음을
함께 보존한다.

```yaml
dataset_id: <hf-owner>/water-kitchen-full-demo-v1
dataset_revision: <40자리 Hub commit SHA>
train_dataset_id: <hf-owner>/water-kitchen-full-demo-v1_train
train_dataset_revision: <40자리 파생 Hub commit SHA>
source_train_episodes: [<원본 episode index>]
lerobot_commit: 7e241bd630a3719a56157a497ce5d08f244784f1
project_commit: <수집 당시 bimanual-robot commit>
calibration_sha256:
  left_follower: <sha256>
  right_follower: <sha256>
policy: act
steps: 50000
batch_size: 8
seed: 1000
checkpoint: <실제 checkpoint 경로 또는 Hub ID>
```

학습 재개는 수집 재개와 다르다.

```bash
"$LEROBOT_BIN/lerobot-train" \
  --config_path="$TRAIN_OUTPUT_DIR/checkpoints/last/pretrained_model/train_config.json" \
  --resume=true
```

## 13. 학습 후 배포 전 게이트

학습 loss 감소는 실물 성공 근거가 아니다. checkpoint는 다음 순서로 평가한다.

1. 학습에 쓰지 않은 holdout scenario에서 offline 입력·출력 shape를 확인한다.
2. Isaac Sim 또는 명령 차단 상태에서 관절 한계·action 크기를 검사한다.
3. 건식 실물 rollout을 소수 episode로 수행한다.
4. `PLANNED_ALL`, `ACT_ALL`, `HYBRID`를 같은 scenario와 holdout으로 비교한다.
5. 실제 물은 건식 파지·기울임·복귀, YOLO stop gate, 흘림 정지 조건이 통과한 뒤 사용한다.

실물 rollout은 녹화 때와 같은 카메라 key, 관절 순서, calibration과 absolute joint action
표현을 사용해야 한다. ACT와 IK/PLANNED가 동시에 팔 명령을 내리지 않는다.

## 14. 즉시 중단하고 보고할 조건

| 관측 | 처리 |
|---|---|
| 팔이 반대 방향으로 움직임 | 즉시 종료, Phase·calibration·drive mode 확인 |
| calibration mismatch | 즉시 종료, `c` 금지, 네 JSON과 EEPROM 대조 |
| servo stall·과열·소음 증가 | 전원 차단, failure evidence 보존, 재실행 금지 |
| 카메라 key 또는 해상도 변경 | 본수집 중단, 새 dataset version 판단 |
| frame drop·제어 주기 밀림 | 실제 frame 수·drop 비율·주기 경고를 기록, episode 보류 후 CPU·USB·인코더 병목 분리 |
| Hub upload 실패 | 로컬 원본 유지, finalize 여부 확인 후 upload 재시도 |
| sidecar schema 실패 | Hub release·학습 보류, 누락값을 추측하지 않음 |
| 공개 repo로 생성됨 | 추가 업로드 중단, 접근 설정을 private로 변경 후 확인 |
| dataset SHA가 학습 기록과 다름 | 학습 중단, 의도한 revision을 다시 고정 |

## 15. 금지 사항

- 12 V가 켜진 상태에서 서보 커넥터를 빼거나 꽂지 않는다.
- 텔레옵·녹화 중 같은 시리얼 포트를 다른 프로그램에서 열지 않는다.
- 이 버스에서 `lerobot-calibrate`를 즉석 실행하지 않는다.
- ggao50 Phase를 형제 서보 값에 맞추지 않는다.
- 리더·팔로워 방향 문제를 EEPROM Phase 변경으로 임의 해결하지 않는다.
- 열려 있는 dataset 파일을 Hub에 백그라운드 업로드하지 않는다.
- smoke와 본수집, policy 1과 2, 성공 시연과 HIL 복구를 합치지 않는다.
- 실패 원본을 성공 BC 데이터로 바꾸거나 성공률 분모에서 삭제하지 않는다.
- dataset `main`이 바뀐 뒤 예전 학습을 같은 데이터라고 부르지 않는다.
- 토큰, 사람 얼굴·개인정보, 내부 장소 정보가 든 dataset을 public으로 만들지 않는다.

## 16. AI가 마지막에 제출할 인계 결과

AI는 작업을 끝낼 때 다음 표를 실제 값으로 채워 보고한다. 실행하지 않은 항목은 `미실행`으로
쓰고 통과로 표시하지 않는다.

| 항목 | 결과 |
|---|---|
| 프로젝트 commit | |
| LeRobot version·commit | |
| 양팔 패치 상태 | |
| 오른쪽 ggao50 ID 6 Phase | |
| follower/leader by-id 4개 | |
| 카메라 key·by-path·해상도·fps | |
| calibration JSON SHA-256 4개 | |
| collection plan commit·scenario split | |
| dataset ID·local root | |
| task instruction·policy·scope | |
| 수집 episode 총수 | |
| train 포함 / 제외 / holdout | |
| sidecar schema 검증 | |
| failure evidence 검증 | |
| Hub private 여부 | |
| Hub full commit SHA | |
| failure repo ID·full commit SHA | |
| 파생 train dataset ID·full commit SHA | |
| 원본→파생 episode map | |
| 다른 PC 학습 config | |
| 생성 checkpoint | |
| 미실행·blocker | |

## 17. 저장소 안의 원본과 공식 근거

프로젝트 원본:

- [텔레옵·캘리브레이션 상세](20260910_텔레옵_IL_사용법.md)
- [데이터셋 ID·키·수집 계약](../data/README.md)
- [episode sidecar 스키마](../data/schema/episode_metadata.schema.json)
- [에피소드 포함·제외 기준](../data/schema/에피소드_포함제외_기준.md)
- [failure record 스키마](../data/schema/failure_record.schema.json)
- [failure bank 운영](../data/failures/README.md)
- [dataset card 템플릿](../data/schema/dataset_card_TEMPLATE.md)

공식 근거:

- [LeRobot v0.6.1 실물 IL 수집·재개·학습](https://github.com/huggingface/lerobot/blob/7e241bd630a3719a56157a497ce5d08f244784f1/docs/source/il_robots.mdx)
- [LeRobot v0.6.1 record: save, finalize, push 순서](https://github.com/huggingface/lerobot/blob/7e241bd630a3719a56157a497ce5d08f244784f1/src/lerobot/scripts/lerobot_record.py)
- [LeRobotDataset finalize·Hub upload](https://github.com/huggingface/lerobot/blob/7e241bd630a3719a56157a497ce5d08f244784f1/src/lerobot/datasets/lerobot_dataset.py)
- [Hugging Face 인증](https://huggingface.co/docs/huggingface_hub/en/quick-start#authentication)
- [Hugging Face 중단 재개 가능한 폴더 업로드](https://huggingface.co/docs/huggingface_hub/guides/upload)
