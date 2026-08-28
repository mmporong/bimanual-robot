# 팀 협업과 작업 관리

대상 저장소: `mmporong/bimanual-robot`

권장 운영: 작업자 1명, 리뷰어 1명, 강사·멘토는 게이트 검토

2026-08-28에 [@jangjunseo05](https://github.com/jangjunseo05)에게 write 권한 초대를 발송했다. 초대를 수락하면 아래 첫 설정과 Pull Request 절차를 따른다.

## 협업자와 기여자의 차이

- **Collaborator**는 저장소에 쓰기 권한을 받은 사람이다.
- **Contributor**는 실제 커밋이 기본 브랜치에 합쳐져 기여 기록이 남은 사람이다.

팀원을 Collaborator로 초대해도 `Contributors` 목록에 바로 나타나지는 않는다. 팀원이 자신의 GitHub 계정에 연결된 이메일로 커밋하고, 그 커밋이 기본 브랜치 `main`에 합쳐져야 기여자로 집계된다. 반영 뒤 통계가 갱신되기까지 시간이 걸릴 수 있다.

## 팀원 초대

웹에서 초대하는 방법이 가장 간단하다.

1. [mmporong/bimanual-robot](https://github.com/mmporong/bimanual-robot) 저장소를 연다.
2. `Settings`를 누른다.
3. 왼쪽 `Access` 영역에서 `Collaborators`를 연다.
4. `Add people`을 누른다.
5. 팀원의 GitHub 사용자 이름이나 이메일을 검색한다.
6. `Add ... to this repository`를 누른다.
7. 팀원이 이메일이나 GitHub 알림에서 초대를 수락한다.

이 저장소는 개인 계정 소유 저장소다. 개인 저장소의 Collaborator는 코드를 읽고 쓸 수 있으므로 초대 뒤에는 `main` 직접 푸시를 막고 Pull Request로만 합치는 편이 안전하다.

GitHub CLI로 초대하려면 저장소 소유자가 다음 명령을 실행한다.

```bash
gh api --method PUT repos/mmporong/bimanual-robot/collaborators/GITHUB_ID
```

`GITHUB_ID`는 초대할 사람의 GitHub 사용자 이름으로 바꾼다. 초대 전에 입력한 계정이 맞는지 프로필을 확인한다.

공식 문서:

- [Inviting collaborators to a personal repository](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/repository-access-and-collaboration/inviting-collaborators-to-a-personal-repository)
- [Permission levels for a personal account repository](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/repository-access-and-collaboration/permission-levels-for-a-personal-account-repository)
- [Viewing a project's contributors](https://docs.github.com/en/repositories/viewing-activity-and-data-for-your-repository/viewing-a-projects-contributors)

## 첫 설정

팀원은 초대를 수락한 뒤 저장소를 복제하고, GitHub 계정에 연결된 이메일을 이 저장소에만 설정한다.

```bash
git clone git@github.com:mmporong/bimanual-robot.git
cd bimanual-robot
git config user.name "GitHub 표시 이름"
git config user.email "GitHub 계정에 연결된 이메일"
git switch main
git pull --ff-only origin main
```

이메일 공개를 원하지 않으면 GitHub 설정의 `noreply` 주소를 사용한다. 커밋 이메일이 GitHub 계정과 연결되지 않으면 기여 기록이 계정에 연결되지 않는다.

## 브랜치 운영

`main`에는 직접 푸시하지 않는다. Issue 하나마다 작업 브랜치 하나를 만든다.

```bash
git switch main
git pull --ff-only origin main
git switch -c feat/slam-station-approach
```

권장 접두어:

| 접두어 | 용도 | 예시 |
|---|---|---|
| `feat/` | 새 기능·실험 구현 | `feat/gripper-static-poc` |
| `fix/` | 확인된 문제 수정 | `fix/depth-frame-offset` |
| `research/` | 재현 가능한 조사·벤치마크 | `research/act-baseline` |
| `docs/` | 회의록·실험표·설명 | `docs/g0-test-sheet` |

커밋은 한 가지 변경만 담고 한글 Conventional Commit을 사용한다.

```text
feat(slam): 작업대 접근 목표 상태를 기록한다
test(gripper): 병 정적 파지 시험표를 추가한다
docs(plan): G0 통과 조건을 명시한다
```

## Issue를 작업 단위로 사용

Issue 제목에는 게이트와 영역을 함께 적는다.

```text
[G0][hardware] 병과 컵 정적 파지
[G2][data] 저울 유입량 기록 형식
[G3][slam] Nav2 완료 상태와 로컬 정렬 인계
```

모든 Issue에는 다음 다섯 항목이 있어야 한다.

1. 목표: 끝났을 때 무엇이 달라지는가
2. 범위: 이번 Issue에서 할 일과 하지 않을 일
3. 통과 조건: 확인 가능한 완료 기준
4. 증거: 코드, 로그, 표, 사진, 영상 가운데 무엇을 남길지
5. 의존성: 먼저 끝나야 하는 Issue나 하드웨어

큰 게이트는 부모 Issue로 만들고 세부 작업은 Sub-issue로 나눈다. Markdown Tasklist의 고급 추적 기능은 종료됐으므로 새 작업은 GitHub Sub-issues를 우선한다.

권장 라벨:

- 영역: `area:slam`, `area:manipulation`, `area:hardware`, `area:data`, `area:agent`
- 성격: `type:research`, `type:experiment`, `type:implementation`, `type:docs`
- 우선순위: `priority:p0`, `priority:p1`, `priority:p2`
- 상태 보조: `blocked`, `needs-review`

## GitHub Project 보드

개인 프로필의 `Projects`에서 `HOLD THE FLOW` 보드를 만들고 이 저장소의 Issue와 Pull Request를 연결한다. Board 보기를 쓰고 상태는 다음 여섯 개만 둔다.

```text
Backlog → Ready → In progress → Review → Blocked → Done
```

- `Backlog`: 아이디어나 후속 후보
- `Ready`: 통과 조건과 담당자가 정해진 작업
- `In progress`: 실제로 손대는 작업. 한 사람당 하나를 원칙으로 한다.
- `Review`: 코드·문서·실물 증거를 다른 사람이 확인하는 작업
- `Blocked`: 부품, 장비, 선행 Issue 때문에 멈춘 작업
- `Done`: 증거와 리뷰가 모두 남은 작업

마일스톤은 `G0 파지`, `G1 기울이기`, `G2 붓기`, `G3 통합`, `G4 복구`로 나눈다. 봉지 실험은 현재 보드의 `Backlog`에만 둔다.

GitHub Projects는 Issue와 Pull Request 상태를 연결하고 Board·Table·Roadmap 보기를 제공한다.

- [Creating a project](https://docs.github.com/en/issues/planning-and-tracking-with-projects/creating-projects/creating-a-project)
- [Planning and tracking with Projects](https://docs.github.com/en/issues/planning-and-tracking-with-projects)
- [Adding sub-issues](https://docs.github.com/en/issues/tracking-your-work-with-issues/using-issues/adding-sub-issues)

## Pull Request 규칙

1. 작업 시작 전에 Issue를 자신에게 할당한다.
2. Issue 번호를 브랜치와 Pull Request에 연결한다.
3. 변경 내용, 검증 결과, 실물 증거 경로를 Pull Request 본문에 적는다.
4. 작성자가 아닌 팀원 한 명이 리뷰한다.
5. 대화가 해결되고 로컬 검증이 통과하면 `main`에 합친다.
6. 기여자 기록을 보존하려면 기본은 `Rebase and merge`를 사용한다.

`main` 브랜치 보호 권장값:

- Require a pull request before merging
- Require 1 approval
- Dismiss stale approvals when new commits are pushed
- Require conversation resolution before merging
- Block force pushes
- Block branch deletion

설정 위치는 `Settings → Branches → Add branch protection rule`이며 대상 패턴은 `main`이다. 이 저장소에서는 GitHub Actions를 사용하지 않으므로 필수 상태 검사 대신 Pull Request에 로컬 검증 결과를 남긴다.

- [Managing protected branches](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-protected-branches)
- [Managing and standardizing pull requests](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/getting-started/managing-and-standardizing-pull-requests)

## 완료 기준

| 작업 | Done에 필요한 증거 |
|---|---|
| 코드 | 실행 명령, 테스트 결과, 실패 시 로그 |
| 하드웨어 | 부품·설정값, 시험 영상이나 사진, 측정표 |
| 데이터 | 스키마 검증, 샘플 메타데이터, 제외 사유 |
| 리서치 | 원문 링크, 적용 여부, 아직 검증하지 않은 부분 |
| 통합 | 단계별 상태 전이, 실패 위치, 재현 절차 |

논문 결과, 계획 기준, 본인 실험 결과를 같은 칸에 섞지 않는다. 실물 시험 전 항목은 `계획`, 실행했지만 기준을 못 넘긴 항목은 `실패`, 기준을 넘기고 증거가 남은 항목만 `완료`로 표시한다.

## 짧은 일일 공유

매일 10분 동안 Project 보드를 열고 아래 네 가지만 말한다.

```text
오늘 확인한 것:
남긴 증거:
막힌 지점:
다음 행동:
```

말로 끝내지 않고 진행 중인 Issue에 같은 내용을 댓글로 남긴다. 결정이 바뀌면 기존 회의록을 지우지 않고 새 결정 문서를 만든 뒤 README의 기준 문서 링크만 교체한다.
