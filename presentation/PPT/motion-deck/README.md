# motion-deck — 흰 바탕·하늘색 모션 PPT 만들기

MS Team 프로젝트 출정식(2026-10-02) 발표 자료를 만든 방식을 그대로 묶은 키트입니다.
명령 한 줄이면 10쪽짜리 발표 뼈대가 `.pptx`로 만들어집니다. 쪽마다 들어 있는 예시 문구와 사진을 바꾸고 다시 빌드하면 발표 자료가 됩니다.

만들어지는 것

- 발표용 `.pptx` 하나: Pretendard 글꼴 포함, 요소별 등장 애니메이션, 쪽 전환 Morph, 영상 무음 자동 재생
- 쪽마다 미리보기 PNG 한 장: 글자 넘침·겹침을 눈으로 확인하는 용도
- PDF는 만들지 않습니다.

---

## 1. 처음 한 번: 준비

### 1-1. 지원하는 PC

| PC | 방법 |
|---|---|
| Ubuntu 22.04·24.04 | 아래 1-2 명령을 그대로 실행합니다. Ubuntu 24.04에서 확인했습니다. |
| Windows 10·11 | WSL2로 Ubuntu를 설치한 뒤, Ubuntu 창에서 아래 명령을 실행합니다. |
| Mac | 확인하지 못했습니다. Ubuntu 또는 Windows(WSL2) PC를 쓰세요. |

Windows는 PowerShell을 **관리자 권한**으로 열고 아래를 실행한 뒤 재부팅합니다. 재부팅 후 시작 메뉴의 `Ubuntu 24.04`를 열고 사용자 이름과 암호를 정하면 됩니다.

```powershell
wsl --install -d Ubuntu-24.04
```

### 1-2. 필요한 프로그램 설치 (Ubuntu 터미널)

```bash
sudo apt update
sudo apt install -y git curl unzip zip python3-lxml python3-fonttools python3-defusedxml \
  libreoffice-impress poppler-utils ffmpeg xvfb fonts-noto-cjk
```

Node.js는 **20.9 이상**이 필요합니다. Ubuntu 기본 `apt` 판(24.04는 18, 22.04는 12)은 쓸 수 없으니 nvm으로 설치합니다.

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
source ~/.bashrc
nvm install 22
node -v        # v22.x 가 나오면 됩니다
```

### 1-3. 키트 풀기

받은 `motion-deck.zip`을 홈 폴더에 풉니다. 아래 안내는 모두 `~/motion-deck`에 풀었다고 가정합니다.

```bash
unzip ~/motion-deck.zip -d ~
```

Windows에서 zip을 `다운로드` 폴더에 받았다면 Ubuntu 창에서 먼저 복사합니다(`<윈도우 사용자 이름>`은 본인 것으로 바꿉니다).

```bash
cp /mnt/c/Users/<윈도우 사용자 이름>/Downloads/motion-deck.zip ~/
unzip ~/motion-deck.zip -d ~
```

---

## 2. 명령 한 줄로 만들기

```bash
bash ~/motion-deck/scripts/make_deck.sh ~/발표/20261101_중간발표
```

- 처음 실행할 때는 필요한 패키지(pptxgenjs 등), 애니메이션 도구(ppt-master), Pretendard 글꼴을 `~/.cache/motion-deck`에 내려받습니다. 두 번째부터는 내려받지 않습니다.
- 덱 폴더 이름은 자유입니다. 날짜와 행사 이름을 붙이면 찾기 쉽습니다.
- 끝나면 화면 마지막에 아래 경로가 나옵니다.
  - 발표 파일: `~/발표/20261101_중간발표/out/deck.pptx`
  - 미리보기: `~/발표/20261101_중간발표/out/render/s-01.png` … `s-10.png`
- 출정식 27쪽 원본에서 시작하려면 끝에 `kickoff`를 붙입니다(키트에 출정식 사진·영상이 들어 있습니다).

```bash
bash ~/motion-deck/scripts/make_deck.sh ~/발표/20261101_출정식수정 kickoff
```

Windows에서 결과 폴더를 탐색기로 열려면 Ubuntu 창에서 아래처럼 합니다. `deck.pptx`를 PowerPoint로 열면 됩니다.

```bash
cd ~/발표/20261101_중간발표/out && explorer.exe .
```

---

## 3. 내용 바꾸기

### 3-1. 덱 폴더 구조

```
~/발표/20261101_중간발표/
├── deck.config.js   제목, 팀 이름, 색 테마, 로고
├── slides/          쪽 하나 = 파일 하나. 파일 이름 순서가 쪽 순서
│   ├── 01-title.js
│   ├── 02-pipeline.js
│   └── …
├── assets/          사진, 영상, 로고
└── out/             빌드 결과(deck.pptx, render/)
```

### 3-2. starter 10쪽에 들어 있는 것

| 파일 | 쪽 | 바꿀 것 |
|---|---|---|
| `01-title.js` | 표지 | 과정 이름, 두 줄 제목, 한 줄 소개, 날짜, 제품 사진(`robot_w.png`) |
| `02-pipeline.js` | 전체 흐름 | 단계 이름과 설명(배지가 경로를 따라 이동) |
| `03-cards.js` | 해결 방법 | 카드 4장의 제목, 설명, 아이콘 |
| `04-demo.js` | 시연 영상 | 영상(`sample_clip.mp4`), 영상 첫 화면(`sample_poster.jpg`), 큰 숫자, 사실 3줄 |
| `05-metrics.js` | 결과 지표 | 차트 데이터, 지표 타일, 달성 막대 |
| `06-team.js` | 팀과 역할 | 4파트 이름과 담당 |
| `07-part.js` | 파트 상세 | 프로필, 사진(`sample_photo.jpg`), 한 일 |
| `08-table.js` | 시도와 선택 | 비교표 내용(채택한 행은 색으로 표시) |
| `09-next.js` | 다음 단계 | 문제 → 대응 행 |
| `10-closing.js` | 마무리 | 질문 카드, 태그라인, 링크 |

`slides/` 안의 `.js` 파일은 텍스트 편집기(VS Code 등)로 엽니다. Windows에서는 Ubuntu 창에서 `code ~/발표/20261101_중간발표`를 실행하면 VS Code가 열립니다(VS Code의 WSL 확장 필요).

### 3-3. 문구 바꾸기

`txt('문구', x, y, 폭, 높이, {글자 모양}, {애니메이션})`에서 **첫 번째 따옴표 안만** 고칩니다. 줄바꿈은 `\n`입니다.

```js
// 바꾸기 전
txt('완성품 이름을\n두 줄로', 0.75, 1.95, 6.4, 2.25, { size: 58, bold: true, color: C.ink, ls: 1.08 }, { e: 'wipeR', d: 450, t: 0.7 });
// 바꾼 뒤
txt('물 서빙\n양팔 로봇', 0.75, 1.95, 6.4, 2.25, { size: 58, bold: true, color: C.ink, ls: 1.08 }, { e: 'wipeR', d: 450, t: 0.7 });
```

- 쪽 제목은 `header({ … title: '…' })`의 `title`입니다. 길면 30pt에서 20pt까지 자동으로 줄어듭니다.
- 문구가 상자보다 길면 빌드할 때 `FIT WARNINGS`로 알려 줍니다(4장 참고).

### 3-4. 사진·영상 바꾸기

1. 새 파일을 덱 폴더의 `assets/`에 넣습니다.
2. 둘 중 하나를 합니다.
   - 기존 파일과 같은 이름으로 덮어씁니다(예: 시연 영상을 `sample_clip.mp4`로 저장).
   - 쪽 파일에서 파일 이름을 새 이름으로 바꿉니다.
3. 영상은 MP4(H.264)를 씁니다. 영상 첫 화면 그림(`…_poster.jpg`)도 같이 바꿉니다.

```bash
cd ~/발표/20261101_중간발표
ffmpeg -ss 1 -i assets/새영상.mp4 -frames:v 1 -q:v 3 assets/새영상_poster.jpg   # 1초 지점 화면을 첫 화면 그림으로
```

### 3-5. 쪽 추가·삭제·순서 바꾸기

- 추가: 비슷한 쪽 파일을 복사해 이름을 정합니다(예: `07-part.js` → `07b-part2.js`).
- 삭제: 파일을 지웁니다.
- 순서: 파일 이름 앞 번호를 바꿉니다.
- 쪽 번호는 빌드할 때 자동으로 매겨집니다.

### 3-6. 색·팀 이름·로고 (`deck.config.js`)

```js
module.exports = {
  title: '프로젝트 최종 발표',      // 파일 속성의 제목
  author: 'MS Team',
  company: 'MS Team',
  theme: 'sky',                     // sky(기본) | teal | blue
  brand: {
    mark: 'logo_mark_light_sky.png', markW: 0.44, markH: 0.418, alt: 'MS Team 로고',  // 내용 쪽 오른쪽 위
    wordmark: 'logo_h_light_sky.png', wordW: 2.68, wordH: 0.55,                      // 표지·마무리
  },
};
```

- 기본 로고는 MS Team입니다. 다른 팀이면 로고 PNG를 `assets/`에 넣고 파일 이름과 `alt`를 바꿉니다. 로고 없이 만들려면 `brand: { … },` 블록을 지웁니다.
- 색 하나만 바꾸려면 `palette: { main: '1690D6' }`처럼 씁니다(`#` 없이 6자리).

---

## 4. 다시 빌드하고 확인하기

```bash
bash ~/motion-deck/scripts/build.sh ~/발표/20261101_중간발표
```

빌드 출력에서 아래 세 줄을 확인합니다.

| 출력 | 뜻 | 아니면 |
|---|---|---|
| `no fit warnings` | 모든 글자가 상자 안에 들어감 | `FIT WARNINGS:` 아래에 `s4 …`처럼 쪽 번호와 문구가 나옵니다. 문구를 줄이거나 상자를 키웁니다. |
| `partial occlusions: 0` | 도형끼리 일부만 겹쳐 가리는 곳 없음 | 겹친 도형 이름이 나옵니다. 연결선·화살표는 카드를 모두 그린 뒤에 그립니다. |
| `All validations PASSED!` | 파일 구조 검사 통과 | 이 검사는 Anthropic pptx 스킬이 설치된 PC에서만 돕니다. 없으면 `SKIP: …`이 나오고, 기본 구조 검사는 빌드 단계에서 이미 끝났습니다. |

그다음 `out/render/` 안의 PNG를 첫 쪽부터 마지막 쪽까지 모두 열어 봅니다(10쪽 이상이면 `s-01.png`, 9쪽 이하면 `s-1.png`부터).

- 글자가 잘리거나 넘친 곳, 겹친 곳, 가장자리에 너무 붙은 곳, 흐려서 안 읽히는 글자를 봅니다.
- 미리보기는 LibreOffice로 그린 그림입니다. 한글과 영문 사이가 조금 벌어져 보이는 것은 PowerPoint에서는 붙어 보이니 고치지 않습니다.
- 미리보기는 애니메이션이 모두 끝난 상태입니다.

빌드를 빠르게 하는 방법

```bash
NORENDER=1 bash ~/motion-deck/scripts/build.sh ~/발표/20261101_중간발표   # 미리보기 PNG 생략
ONLY=4 bash ~/motion-deck/scripts/build.sh ~/발표/20261101_중간발표       # 4쪽만 out/only.pptx 로
```

애니메이션이 실제로 어떻게 움직이는지 보려면 `out/deck.pptx`를 PowerPoint나 LibreOffice 슬라이드 쇼로 엽니다.

---

## 5. AI(Claude Code·Codex)로 만들기

Claude Code나 Codex를 쓰면 원고만 주고 AI에게 쪽 작성과 검수까지 맡길 수 있습니다.

### 5-1. 스킬 설치 (한 번)

```bash
bash ~/motion-deck/scripts/install_skill.sh
```

`~/.claude/skills/motion-deck`(Claude Code)와 `~/.codex/skills/motion-deck`(Codex) 중 있는 쪽에 복사합니다. 설치 후 **새 세션**을 열어야 인식합니다. 키트를 새 판으로 받으면 다시 실행합니다. 다시 설치하면 설치본을 통째로 바꾸므로, 설치본 안의 파일은 고치지 않습니다.

### 5-2. 요청 예

```
motion-deck 스킬로 PPT 만들어줘.
- 덱 폴더: ~/발표/20261101_중간발표 (starter에서 시작)
- 발표: MS Team 중간 발표, 10분
- 내용: (원고나 메모를 여기에 붙여넣기)
- 사진·영상: ~/사진/시연.mp4, ~/사진/로봇.png
```

- 요청에 **"PPT"** 또는 **"motion-deck"**을 넣습니다. "슬라이드 만들어줘"라고만 하면 다른 방식(웹 슬라이드 등)으로 만들 수 있습니다.
- 기존 PPT·PDF를 이 디자인으로 다시 만들 때는 원본 파일 경로를 같이 알려 줍니다.
- AI가 빌드 결과의 세 줄과 미리보기를 확인한 뒤 보고합니다. 보고에 세 줄이 없으면 확인해 달라고 요청합니다.

설치하지 않고 쓰려면 이렇게 요청합니다.

```
~/motion-deck/SKILL.md 를 읽고 그 순서대로 PPT를 만들어줘. SKILL_DIR은 ~/motion-deck 이야. (이하 내용 동일)
```

---

## 6. 발표 파일 넘기기와 발표할 때

납품 폴더로 정리하려면 아래처럼 합니다. 발표 파일, 글꼴 폴더, 다시 빌드할 원본(`_build/`)이 한곳에 모입니다.

```bash
bash ~/motion-deck/scripts/package.sh ~/발표/20261101_중간발표 ~/발표/20261101_중간발표_납품 MS팀_중간발표
```

- **PowerPoint 2019 이상·Microsoft 365**: 쪽 사이가 Morph로 이어지고, 영상은 반복 재생됩니다.
- **LibreOffice**: Morph 대신 페이드로 넘어가고, 영상은 한 번 재생 후 멈춥니다.
- 한 쪽에서 클릭 한 번이면 그 쪽의 등장 애니메이션이 끝까지 자동으로 이어집니다(대부분 1.5~3.5초).
- 영상은 무음입니다. 소리가 필요하면 PowerPoint에서 영상 선택 → 재생 탭 → 볼륨에서 음소거를 풉니다.
- 다른 PC에서 글꼴이 바뀌어 보이면 `글꼴/` 폴더의 TTF 3개를 설치한 뒤 다시 엽니다.
- **LibreOffice에서 PPTX로 다시 저장하지 않습니다.** Morph, 일부 애니메이션, 포함된 글꼴이 빠집니다. 고칠 것이 있으면 `slides/`를 고쳐 다시 빌드합니다.

---

## 7. 꼭 지킬 것 (실제로 문제가 났던 것들)

1. **움직이는 GIF는 `rounded()`·`cover()`에 넣지 않습니다.** 첫 장면에서 멈춥니다. `img(A('파일.gif'), …)`로 넣거나 MP4로 바꿔 `videoCard`에 넣습니다.
2. **조금 늦게 나타나는 영상은 `videoCard(…, { cover: true, appear: ms })`로 넣습니다.** 덮개가 없으면 LibreOffice에서 쪽이 열리자마자 정지 화면이 보입니다.
3. **연결선·화살표는 카드를 모두 그린 뒤에 그립니다.** 먼저 그리면 다음 카드가 가립니다.
4. **표의 `margin`은 인치 단위입니다**(`[0.04, 0.14, 0.04, 0.14]`). `[0, 8, 0, 8]`처럼 쓰면 표가 깨집니다.
5. **애니메이션 시각 `d`는 쪽이 열린 때부터의 ms입니다.** 앞 효과와의 간격이 아닙니다.
6. 색은 `C.main`, `C.ink`처럼 `C.*` 이름으로만 씁니다. 제목 밑줄, 가장자리 색 띠, 한쪽 테두리 강조는 쓰지 않습니다.
7. 사진·캡처는 `rounded()`로 모서리를 둥글게 합니다. 시연 영상과 실험 화면의 색은 바꾸지 않습니다.
8. 한 쪽의 등장은 1.5~3.5초 안에 끝나게 합니다.

---

## 8. 문제가 생기면

| 화면에 나온 것 | 해결 |
|---|---|
| `Node.js v18… is too old` 또는 `node not found` | 1-2의 nvm 설치를 합니다. 새 터미널에서 `node -v`가 v20.9 이상인지 봅니다. |
| `python3 needs lxml and fonttools` | `sudo apt install python3-lxml python3-fonttools` |
| `motion-deck cache missing` | `bash ~/motion-deck/scripts/setup.sh`를 실행합니다. |
| `refusing: …/slides already exists` | 이미 만든 덱 폴더입니다. 다시 빌드하려면 `build.sh`를 쓰고, 새로 시작하려면 다른 폴더 이름을 씁니다. |
| `SKIP render: soffice/pdftoppm not found` | `sudo apt install libreoffice-impress poppler-utils` 후 다시 빌드합니다. 발표 파일(`deck.pptx`)은 이미 만들어져 있고 미리보기 PNG만 빠진 상태입니다. |
| `starter needs ffmpeg` | `sudo apt install ffmpeg` (starter의 예시 사진·영상을 만들 때 필요합니다) |
| `FIT WARNINGS:` | 나온 쪽 번호의 문구를 줄이거나 상자 폭·높이를 키웁니다. |
| 내려받기 중 실패(`npm`, `git`, `curl`) | 인터넷 연결을 확인하고 같은 명령을 다시 실행합니다. 받은 것은 다시 받지 않습니다. |
| 미리보기의 글자 모양·줄바꿈이 PowerPoint에서 연 발표 파일과 다름 | `setup.sh`를 다시 실행한 뒤 다시 빌드합니다. Pretendard를 내려받아 확인하고, 미리보기가 그 글꼴을 쓰도록 `~/.cache/motion-deck/fonts.conf`를 다시 씁니다. |

---

## 9. 더 자세한 문서

| 문서 | 내용 |
|---|---|
| `~/motion-deck/SKILL.md` | 전체 작업 순서, 쪽 작성 함수 목록(AI가 읽는 문서) |
| `~/motion-deck/references/design.md` | 격자, 색표, 글자 크기, 레이아웃 25종과 참고 코드 위치 |
| `~/motion-deck/references/motion.md` | 애니메이션 종류(`fade`, `up`, `wipeR`, `path`, `pulse` …)와 시간 배분 |
| `~/motion-deck/references/pitfalls.md` | 실제 사고와 원인 |
| `~/motion-deck/references/delivery.md` | 납품 폴더와 `발표 안내.txt` 틀 |
| `~/motion-deck/templates/kickoff/slides/` | 출정식 27쪽 코드(레이아웃 예시가 가장 많음) |

키트 판 정보는 `~/motion-deck/.motion-deck-kit`에 있습니다.

사용한 도구: [pptxgenjs](https://github.com/gitbrent/PptxGenJS)(MIT), [ppt-master](https://github.com/hugohe3/ppt-master)(MIT), [Pretendard](https://github.com/orioncactus/pretendard)(OFL).
