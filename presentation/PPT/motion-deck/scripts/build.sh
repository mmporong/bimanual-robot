#!/usr/bin/env bash
# Build a deck: slides -> pptx -> post-process (animations, Morph, video, fonts) -> checks -> PNG renders for review.
#   usage: build.sh [deck_dir]            (default: current dir; needs deck.config.js + slides/)
#   env:   ONLY=3,7   build only those slide positions into out/only.pptx (for slideshow tests)
#          NORENDER=1 skip the PNG renders
#          FONT_SUBSET=1 embed only the characters the deck uses (fits upload limits, e.g. 10 MB)
# outputs: out/deck.pptx (show file), out/render/s-NN.png (static variant: no slideshow covers).
#          The PDF made on the way stays in out/render as an intermediate; decks are delivered without a PDF.
set -euo pipefail
source "$(dirname "$0")/env.sh"
DECK="$(cd "${1:-.}" && pwd)"
export DECK_DIR="$DECK"
[ -d "$DECK/slides" ] || { echo "no slides/ in $DECK" >&2; exit 2; }
OUT="$DECK/out"; mkdir -p "$OUT"
NAME=deck; [ -n "${ONLY:-}" ] && NAME=only

echo "== build ($NAME)"
node "$ENGINE/build.js" "$OUT/${NAME}_raw.pptx"
python3 "$ENGINE/post.py" "$OUT/${NAME}_raw.pptx" "$OUT/${NAME}_raw.anim.json" "$OUT/$NAME.pptx"

echo "== checks"
python3 "$ENGINE/occlusion.py" "$OUT/$NAME.pptx"
VAL="$(ls "$HOME"/.claude/skills/synced/*/pptx/scripts/office/validate.py "$HOME"/.claude/skills/pptx/scripts/office/validate.py 2>/dev/null | head -1 || true)"
if [ -n "$VAL" ] && ! PYTHONPATH="$CACHE/pylib${PYTHONPATH:+:$PYTHONPATH}" python3 -c 'import defusedxml' 2>/dev/null; then
  echo "SKIP: validate.py needs defusedxml (Ubuntu: sudo apt install python3-defusedxml); structure still checked by post.py"
elif [ -n "$VAL" ]; then
  PYTHONPATH="$CACHE/pylib${PYTHONPATH:+:$PYTHONPATH}" python3 "$VAL" "$OUT/$NAME.pptx" 2>&1 | tail -4
else
  echo "SKIP: Anthropic pptx validate.py not installed (structure still checked by post.py)"
fi

if [ -z "${NORENDER:-}" ] && ! { command -v soffice >/dev/null && command -v pdftoppm >/dev/null; }; then
  echo "SKIP render: soffice/pdftoppm not found (Ubuntu: sudo apt install libreoffice-impress poppler-utils)"
elif [ -z "${NORENDER:-}" ]; then
  echo "== static render"
  STATIC=1 node "$ENGINE/build.js" "$OUT/${NAME}_static_raw.pptx" | head -1
  python3 "$ENGINE/post.py" "$OUT/${NAME}_static_raw.pptx" "$OUT/${NAME}_static_raw.anim.json" "$OUT/${NAME}_static.pptx" >/dev/null
  rm -rf "$OUT/render"; mkdir -p "$OUT/render"
  # private LibreOffice profile: never attaches to an Impress window the user has open
  FONTCONFIG_FILE="$MOTION_DECK_FONTCONF" timeout 300 soffice --headless --norestore -env:UserInstallation="file://$OUT/lo_profile_render" \
    --convert-to pdf --outdir "$OUT/render" "$OUT/${NAME}_static.pptx" >/dev/null 2>&1
  pdftoppm -r 60 -png "$OUT/render/${NAME}_static.pdf" "$OUT/render/s"
  echo "renders: $OUT/render ($(ls "$OUT/render"/s-*.png | wc -l) pages)"
fi
echo "show file: $OUT/$NAME.pptx"
