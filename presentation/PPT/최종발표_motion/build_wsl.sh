#!/bin/bash
# WSL에서 실행: 이 폴더(slides/, deck.config.js)를 ~/decks/final_motion 으로 복사해 motion-deck으로 빌드하고,
# 발표 파일(deck.pptx)과 검수용 정적 파일(deck_static.pptx)을 이 폴더의 out/ 으로 되가져온다.
# 미리보기 PNG는 Windows PowerPoint로 렌더한다(이 PC의 WSL에는 LibreOffice가 없음).
#   usage: bash build_wsl.sh            (ONLY=3,7 도 그대로 전달됨)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
KIT=~/motion-deck
DECK=~/decks/final_motion
export PATH="$HOME/.cache/motion-deck/py312/bin:$PATH"   # post.py는 Python 3.12 필요
mkdir -p "$DECK/assets"
rm -rf "$DECK/slides"; cp -r "$HERE/slides" "$DECK/slides"
cp "$HERE/deck.config.js" "$DECK/deck.config.js"
for f in "$KIT"/templates/kickoff/assets/*; do [ -e "$DECK/assets/$(basename "$f")" ] || cp "$f" "$DECK/assets/"; done
[ -d "$HERE/assets" ] && cp -f "$HERE"/assets/* "$DECK/assets/" 2>/dev/null || true
NORENDER=1 bash "$KIT/scripts/build.sh" "$DECK"
NAME=deck; [ -n "${ONLY:-}" ] && NAME=only
source "$KIT/scripts/env.sh"
export DECK_DIR="$DECK"
STATIC=1 node "$KIT/engine/build.js" "$DECK/out/${NAME}_static_raw.pptx" | head -1
python3 "$KIT/engine/post.py" "$DECK/out/${NAME}_static_raw.pptx" "$DECK/out/${NAME}_static_raw.anim.json" "$DECK/out/${NAME}_static.pptx" >/dev/null
mkdir -p "$HERE/out"
cp "$DECK/out/$NAME.pptx" "$DECK/out/${NAME}_static.pptx" "$HERE/out/"
ls -la "$HERE/out"
