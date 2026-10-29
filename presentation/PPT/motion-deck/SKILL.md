---
name: motion-deck
description: 발표 자료(PPT·슬라이드)를 새로 만들거나 기존 PPT·PDF를 다시 디자인할 때 쓰는 화이트+하늘색 모션 덱 제작 워크플로. 출정식·중간·최종(완성품) 발표, 팀 발표, "발표자료/PPT/슬라이드/덱 만들어줘·리디자인" 요청에 사용. pptxgenjs 엔진으로 Pretendard 내장·네이티브 애니메이션·Morph·영상 자동재생 PPTX를 만들고(PDF는 만들지 않음) LibreOffice 슬라이드 쇼 녹화로 검증한다.
---

# motion-deck — 화이트+하늘색 모션 발표 자료

2026-10-02 MS팀 프로젝트 출정식 덱(27쪽)을 만든 방식을 그대로 묶은 스킬이다. 사용자가 "작성 방법과 템플릿이 좋다,
다음에도 이렇게"라고 확정했다. 디자인·모션·검증 순서를 바꾸지 말고, 바꿀 때는 사용자에게 먼저 묻는다.

명령을 실행하기 전에 셸에서 `SKILL_DIR`부터 정한다(아래 명령은 모두 이 변수를 쓴다).

```bash
SKILL_DIR=~/.claude/skills/motion-deck      # Codex: ~/.codex/skills/motion-deck, 팀 키트: 압축을 푼 폴더(예: ~/motion-deck)
```

## 산출물

- 발표용 `.pptx`: Pretendard 3종 내장, 네이티브 등장 애니메이션, 쪽 전환 Morph(LibreOffice는 페이드), 영상 무음 자동재생
- **PDF는 만들지 않는다**(사용자 지시 2026-10-07). 검수용 PNG 렌더는 `out/render/`에서만 쓴다. 사용자가 그 작업에서 PDF를 명시하면 `out/render/deck_static.pdf`를 쓴다.
- 납품 폴더 `~/Downloads/발표자료/<YYYYMMDD>_<이름>/` — 구성과 `발표 안내.txt`는 [references/delivery.md](references/delivery.md)

## 처음 한 번

```bash
bash "$SKILL_DIR/scripts/setup.sh"   # ~/.cache/motion-deck: npm 패키지, ppt-master(고정 커밋), Pretendard(sha256 확인)
```

ppt-master는 자체 무결성 검사가 있어 파일 일부만 복사하면 실행이 거부된다. 그래서 스킬에 넣지 않고 setup이 받는다.
Node.js는 20.9 이상이어야 한다(sharp 0.35). setup은 렌더용 `fonts.conf`도 만들어, Pretendard가 설치되지 않은 PC에서도 검수 PNG가 Pretendard로 나온다.

준비부터 빌드까지 한 번에: `bash "$SKILL_DIR/scripts/make_deck.sh" <deck_dir> [starter|kickoff]` (이미 있는 덱 폴더면 다시 빌드만 한다).

## 작업 순서

1. **계약**: 산출물·완료 조건·검증 방법·외부 부수효과를 짧게 밝힌다.
2. **원고 확보**: 기존 PPT/PDF가 있으면 `markitdown`으로 문구를 뽑고 원본 렌더로 구성을 본다.
   수치·고유명사·출처·날짜는 원문 그대로 둔다. 없는 수치·사실을 만들지 않고, 추정치는 "추정"으로 표시한다.
   원문을 고친 곳(오탈자, 쪽 번호 참조 등)은 `발표 안내.txt`의 "디자인 외에 바뀐 내용"에 모두 적는다.
3. **프로젝트 생성**: `bash "$SKILL_DIR/scripts/new_deck.sh" <deck_dir> starter|kickoff`
   - `<deck_dir>`는 납품 폴더의 `_build/`로 둔다. 세션 scratchpad는 사라진다.
   - `starter` = 완성품·결과 발표용 10쪽 뼈대, `kickoff` = 출정식 27쪽 원본(재작업은 여기서 시작).
   - `kickoff`의 사진·영상(49MB)은 스킬에 없고 작성자 PC의 `~/Downloads/발표자료/20261002_MS팀_출정식_모션리디자인/_build/assets`를 쓴다.
     팀 키트에는 `templates/kickoff/assets`로 들어 있다. 둘 다 없는 기기에서는 그 폴더를 덱의 `assets/`로 복사하고 `deck.config.js`의 `assetsDir` 줄을 지운다.
   - `starter`의 로고·팀 이름은 MS Team이다. 다른 팀·개인 발표는 `deck.config.js`의 `author`·`company`·`brand`를 바꾸고 로고를 `assets/`에 넣는다. `brand`를 지우면 로고 없이 만든다.
   - **쪽 하나 = `slides/NN-이름.js` 하나**, 파일 이름 순서가 쪽 순서다. 쪽 추가·삭제·순서 변경은 파일을 추가·삭제·이름 변경으로 한다.
     쪽 번호는 엔진이 매긴다. `kickoff`는 10쪽씩 묶인 파일이므로 재작업 때 쪽 단위 파일로 나눠도 된다.
4. **쪽 작성**: 쪽마다 레이아웃 원형을 고르고([references/design.md](references/design.md)의 원형 표) 가장 비슷한 템플릿 쪽을 복사해 고친다.
   모션은 [references/motion.md](references/motion.md) 규칙대로 단다.
5. **빌드**: `bash "$SKILL_DIR/scripts/build.sh" <deck_dir>`
   - 통과 기준: `no fit warnings` · `partial occlusions: 0` · `All validations PASSED!` (렌더 PNG는 STATIC 빌드에서 만든다)
   - post.py가 모든 효과의 시작 시각을 설계값과 대조하고, 어긋나면 빌드를 멈춘다.
6. **렌더 검수**: `out/render/s-NN.png`를 전부 본다. 넘침·겹침·여백 0.3" 미만·저대비·빈 공간 쏠림을 확인한다.
   고친 쪽만 다시 본다. 독립 검수가 가능하면 verifier 서브에이전트에 렌더를 맡긴다.
7. **재생 검수**: 영상·모션 경로·지연 등장·반복 효과가 있는 쪽은 녹화로 확인한다.
   `ONLY=7 bash build.sh <deck_dir>` → `bash "$SKILL_DIR/scripts/lo_show.sh" <deck_dir>/out/only.pptx 10` → 프레임을 1초 간격으로 묶어 본다.
8. **납품**: `bash "$SKILL_DIR/scripts/package.sh" <deck_dir> <납품폴더> <파일 기본이름>` 후 `발표 안내.txt`를 쓴다.
9. **보고**: 결과·전체 경로·검증 근거·남은 제약을 적는다. PowerPoint 실재생은 이 PC에서 확인할 수 없으니 미확인이라고 쓴다.

## 팀원에게 공유

- 팀원용 안내서는 [README.md](README.md)다(설치, 명령 한 줄 생성, 내용 바꾸기, AI로 만들기, 문제 해결).
- 키트 만들기: `bash "$SKILL_DIR/scripts/export_kit.sh" <출력 폴더>` → `<출력 폴더>/motion-deck.zip`. 출정식 자산을 함께 넣고 작성자의 캐시·다른 스킬 없이 돈다.
- 스킬을 고치면 PKM 원본에서 커밋한 뒤 다시 내보낸다. 키트 쪽을 고쳐 되가져오지 않는다.
- 팀원 PC의 Claude Code·Codex에는 키트의 `scripts/install_skill.sh`가 설치한다(키트 표식 `.motion-deck-kit`가 없는 기존 설치본은 덮어쓰지 않는다).

## 쪽 작성 API

```js
'use strict';
const L = require('motion-deck');
const { C, FSB, ML, CR, CW, txt, box, circle, line, iconDot, header, callout, newSlide } = L;

module.exports = async function results() {
  newSlide();                                    // { transition: 'fade' } 은 표지처럼 Morph가 어색한 쪽에만
  header({ num: '04', label: '결과 — 핵심 지표', title: '목표 대비 무엇을 달성했는가', orb: 'c' });
  const a = { e: 'up', d: 300, t: 0.45 };        // e=효과, d=쪽 시작부터 ms(절대 시각), t=초
  box(ML, 2.0, 4, 1.6, { r: 0.12, fill: C.white, line: C.line, shadow: 'soft' }, a);
  await iconDot('LuGauge', ML + 0.6, 2.6, 0.66, C.mainT, C.main, a);  // 같은 a = 한 덩어리로 등장
  txt('지표 이름', ML + 1.1, 2.35, 2.7, 0.5, { size: 21, bold: true, color: C.ink, valign: 'middle' }, a);
  callout('결론', '수치가 말해 주는 한 문장', 6.15, {}, { e: 'up', d: 1350, t: 0.5 });
};
```

- 기본 도형: `txt` `box` `circle` `line` `img` `iconImg` `iconDot`(Lucide 아이콘 이름 `Lu…`).
- 이미지: `rounded(file,w,h)`(모서리 0.14"), `contain`, `cover`.
- 구성 요소: `header` `callout` `tagPill` `videoCard` `coverUntil` `seqLine` `chevrons` `profile` `sec` `bigNum` `bullets` `miniMap` `orbs`.
  시그니처는 `engine/motion-deck.js` 주석에 있다.
- 색은 `C.*`만 쓴다:
  - 주색: `main` `mainD` `mainM` `mainT` `mainT2` `mainT25` `mainT3`
  - 보조: `sub` `subD` `subT`
  - 강조: `acc` `accD` `accT`
  - 글자·바탕: `ink` `text` `muted` `faint` `line` `panel`
  - 테마는 `deck.config.js`의 `theme: 'sky'|'teal'|'blue'`와 `palette`로 바꾼다.
- 차트·표는 pptxgenjs 네이티브로 넣는다(`L.S.slide.addChart/addTable` + `objectName`). 애니메이션은 `L.anim(name, a)`로 단다.

## 꼭 지킬 것 (사고에서 확정 — 전체는 [references/pitfalls.md](references/pitfalls.md))

1. **움직이는 GIF는 `rounded()`·`cover()`에 넣지 않는다.** 결과가 PNG 한 장이라 첫 프레임에서 멈춘다.
   - `img(A('x.gif'), …)`로 원본을 그대로 넣는다. 2026-10-02 LibreOffice 녹화에서 움직이는 것을 확인했다(모서리는 각짐).
   - 또는 MP4로 `videoCard`에 넣는다.
2. **늦게 나타나는 영상은 `videoCard(…, { cover: true, appear })`로 넣는다.**
   - LibreOffice는 영상의 등장 효과를 무시한다. 덮개 없이 넣으면 쪽이 열리자마자 정지 화면으로 보인다.
3. **연결 화살표·커넥터는 카드를 모두 그린 뒤 그린다.** 다음 카드가 가린다(occlusion 검사가 잡는다).
4. **표 `margin`은 인치 단위다**(`[0.04, 0.14, 0.04, 0.14]`). 포인트 값처럼 `[0,8,0,8]`을 넣으면 표가 깨진다.
5. **효과 시각은 쪽 시작 기준 절대 ms로 쓴다.**
   - 간격 변환과 wipe 방향 보정은 post.py가 한다. post.py를 고칠 때 `check_starts`를 빼지 않는다.
6. **LibreOffice 시험은 전용 프로필로만 한다**(build.sh·lo_show.sh가 그렇게 한다).
   - 종료는 실행 파일 이름과 프로필 경로가 모두 맞는 PID로만 한다. `pkill -f`는 쓰지 않는다.
   - 사용자의 Impress 창을 건드리지 않는다.
7. 발표용 파일을 LibreOffice에서 PPTX로 다시 저장하지 않는다. Morph·애니메이션·내장 글꼴이 빠진다. 안내문에도 적는다.
8. 장식 띠, 제목 밑줄, 한쪽 테두리 강조는 쓰지 않는다. 구분은 틴트·그림자·아이콘으로 한다.

## 사용자 기준

- 색: 흰 바탕에 물빛 하늘색(`sky`)이 기본이다. 짙은 블루·다크 배경은 사용자가 "너무 깊다"며 거절했다.
- 사진·캡처는 모서리를 둥글게 한다(`rounded`). 시연 영상·실험 화면의 색은 바꾸지 않는다.
- 로고는 덱 색에 맞춘 판(`logo_*_sky.png`)을 쓴다.
- 모션은 적극적으로 쓰되, 한 쪽의 등장은 1.5~3.5초 안에 끝내고 클릭 한 번으로 쪽 전체가 진행되게 한다.
- (작성자 PC) 진행 중인 재작업 항목은 메모리(`ms-kickoff-deck-rework`)를 먼저 확인한다. 팀 키트에서는 해당 없음.
