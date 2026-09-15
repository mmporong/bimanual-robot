# 도식 원본

README와 설계 문서에 들어가는 설명용 도식의 원본이다. 그림 파일만 고치지 말고
여기 JSON을 고친 뒤 다시 뽑는다. PNG 위에서 라벨을 수정하면 다음 사람이 같은 그림을
이어서 만들 수 없다.

## 목록

| 원본 | 출력 | 쓰는 곳 |
|---|---|---|
| `system-architecture.architecture.json` | `../assets/system-architecture.png` | README 본문 그림 |
| 〃 | `../assets/system-architecture.html` | 그림을 누르면 열리는 조작 도식 |

README는 GitHub에서 스크립트를 실행하지 못하므로 본문에는 PNG를 두고, 그림에 HTML 링크를 건다.
HTML은 [raw.githack.com](https://raw.githack.com)이 GitHub raw 파일을 올바른 형식으로 서빙해 브라우저에서 바로 열린다.

## 다시 뽑기

[archify](https://github.com/tt-a1i/archify)로 검증하고 렌더한다. 설치는 `npx skills add tt-a1i/archify -g`이고,
스킬 루트는 Claude가 `~/.claude/skills/archify`, Codex가 `~/.agents/skills/archify`다.

```bash
SK=~/.claude/skills/archify
JSON=docs/diagrams/system-architecture.architecture.json

node "$SK/bin/archify.mjs" validate architecture "$JSON" --quality showcase
node "$SK/bin/archify.mjs" render   architecture "$JSON" docs/assets/system-architecture.html
python3 ~/physical-ai-lab/tools/figures/archify_to_garden.py \
  docs/assets/system-architecture.html docs/assets/system-architecture.png
```

세 번째 단계는 뷰어 툴바와 조작 버튼을 숨긴 사본을 찍어 본문용 PNG를 만든다. HTML 자체는
그대로 두므로 조작 기능이 사라지지 않는다. 그 도구가 없으면 브라우저에서 HTML을 열고
Export 메뉴로 PNG를 받아도 된다.

**PNG와 HTML을 항상 같이 갱신한다.** 하나만 올리면 그림과 조작 도식이 서로 다른 내용을 보여준다.

## 고칠 때 지킬 것

- **검증 진단이 제시한 좌표값을 그대로 쓴다.** 겹침이나 라벨 충돌이 나면 진단이
  `labelDy -22` 같은 수정값을 함께 준다. 눈대중으로 오프셋을 밀면 다른 진단이 연달아 나온다.
- **주요 노드는 12개 이하로 유지한다.** 더 담아야 하면 도식을 나눈다. 폭이 넓어지면
  1440px 화면에서 글자가 6px 밑으로 떨어져 `composition/desktop-readability`에 걸린다.
- **JSON과 PNG와 HTML을 같은 커밋에 넣는다.** 하나만 올라가면 원본과 그림과 조작 도식이 서로 어긋난다.
- 검증은 `--quality showcase`로 돌린다. 기본 모드에서는 선 교차와 나란한 선 검사가 돌지 않는다.
