# 도식 원본

README와 설계 문서에 들어가는 설명용 도식의 원본이다. 그림 파일만 고치지 말고
여기 JSON을 고친 뒤 다시 뽑는다. PNG 위에서 라벨을 수정하면 다음 사람이 같은 그림을
이어서 만들 수 없다.

## 목록

| 원본 | 출력 | 쓰는 곳 |
|---|---|---|
| `system-architecture.architecture.json` | `../assets/system-architecture.png` | README 「시스템 구조」 |

## 다시 뽑기

[archify](https://github.com/tt-a1i/archify)로 검증하고 렌더한다. 설치는 `npx skills add tt-a1i/archify -g`이고,
스킬 루트는 Claude가 `~/.claude/skills/archify`, Codex가 `~/.agents/skills/archify`다.

```bash
SK=~/.claude/skills/archify
node "$SK/bin/archify.mjs" validate architecture docs/diagrams/system-architecture.architecture.json --quality showcase
node "$SK/bin/archify.mjs" render   architecture docs/diagrams/system-architecture.architecture.json /tmp/sysarch.html
python3 ~/physical-ai-lab/tools/figures/archify_to_garden.py /tmp/sysarch.html docs/assets/system-architecture.png
```

마지막 단계는 뷰어 툴바와 조작 버튼을 숨기고 여백을 정리한다. 그 도구가 없으면
브라우저에서 HTML을 열고 Export 메뉴로 PNG를 받아도 된다.

## 고칠 때 지킬 것

- **검증 진단이 제시한 좌표값을 그대로 쓴다.** 겹침이나 라벨 충돌이 나면 진단이
  `labelDy -22` 같은 수정값을 함께 준다. 눈대중으로 오프셋을 밀면 다른 진단이 연달아 나온다.
- **주요 노드는 12개 이하로 유지한다.** 더 담아야 하면 도식을 나눈다. 폭이 넓어지면
  1440px 화면에서 글자가 6px 밑으로 떨어져 `composition/desktop-readability`에 걸린다.
- **JSON과 PNG를 같은 커밋에 넣는다.** 둘 중 하나만 올라가면 그림과 원본이 어긋난다.
- 검증은 `--quality showcase`로 돌린다. 기본 모드에서는 선 교차와 나란한 선 검사가 돌지 않는다.
