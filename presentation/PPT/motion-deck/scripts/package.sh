#!/usr/bin/env bash
# Copy a verified build into the delivery folder.
#   usage: package.sh <deck_dir> <delivery_dir> <base_name>
#   writes <base_name>.pptx (show file), 글꼴/ (Pretendard TTF + OFL), _build/ (sources + assets). No PDF (user rule 2026-10-07).
#   A different existing <base_name>.pptx or _build/slides is moved to _이전버전/, never deleted.
#   '발표 안내.txt' is written per deck (see references/delivery.md), not by this script.
set -euo pipefail
source "$(dirname "$0")/env.sh"
DECK="$(cd "${1:?deck_dir}" && pwd)"; D="${2:?delivery_dir}"; BASE="${3:?base_name}"
[ -f "$DECK/out/deck.pptx" ] || { echo "build first: out/deck.pptx" >&2; exit 2; }
mkdir -p "$D/글꼴"
for ext in pptx; do
  dst="$D/$BASE.$ext"
  if [ -f "$dst" ] && ! cmp -s "$DECK/out/deck.$ext" "$dst"; then
    mkdir -p "$D/_이전버전"
    mv "$dst" "$D/_이전버전/${BASE}_$(date -r "$dst" +%Y%m%d-%H%M).$ext"
  fi
  cp "$DECK/out/deck.$ext" "$dst"
done
cp "$MOTION_DECK_FONTS"/Pretendard-{Regular,SemiBold,Bold}.ttf "$MOTION_DECK_FONTS/LICENSE.txt" "$D/글꼴/"
B="$(cd "$D" && pwd)/_build"
if [ "$DECK" != "$B" ]; then
  mkdir -p "$B"
  # replace, never merge: a slide deleted from the deck must not survive in _build/slides
  if [ -d "$B/slides" ] && ! diff -rq "$DECK/slides" "$B/slides" >/dev/null 2>&1; then
    mkdir -p "$D/_이전버전"
    mv "$B/slides" "$D/_이전버전/_build_slides_$(date +%Y%m%d-%H%M%S)"
  fi
  [ -d "$B/slides" ] || cp -r "$DECK/slides" "$B/"
  cp "$DECK/deck.config.js" "$B/"
  [ -d "$DECK/assets" ] && cp -r "$DECK/assets" "$B/"
fi
ls -la "$D"; du -sh "$D"
