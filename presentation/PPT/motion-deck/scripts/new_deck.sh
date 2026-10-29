#!/usr/bin/env bash
# Start a deck project from a template.
#   usage: new_deck.sh <deck_dir> [starter|kickoff]
#   starter : 10-slide skeleton for a result/product talk (title, pipeline, cards, demo video, metrics,
#             team, part detail, table, next steps, closing) with generated sample media
#   kickoff : the 27-slide MS팀 출정식 deck (2026-10-02) as delivered; copies templates/kickoff/assets when a
#             team kit bundled them, otherwise its config points at the author's delivery folder (assetsDir)
# Never overwrites: refuses when <deck_dir>/slides already exists.
set -euo pipefail
SKILL_DIR="$(cd "$(dirname "$0")/.." && pwd)"
DECK="${1:?usage: new_deck.sh <deck_dir> [starter|kickoff]}"; KIND="${2:-starter}"
TPL="$SKILL_DIR/templates/$KIND"
[ -d "$TPL" ] || { echo "unknown template: $KIND" >&2; exit 2; }
[ -e "$DECK/slides" ] && { echo "refusing: $DECK/slides already exists" >&2; exit 2; }
[ "$KIND" = starter ] && ! command -v ffmpeg >/dev/null && { echo "starter needs ffmpeg for its sample media (Ubuntu: sudo apt install ffmpeg)" >&2; exit 2; }
mkdir -p "$DECK"
cp -r "$TPL/slides" "$DECK/slides"
cp "$TPL/deck.config.js" "$DECK/deck.config.js"
if [ -d "$TPL/assets" ]; then
  mkdir -p "$DECK/assets"
  for f in "$TPL"/assets/*; do [ -e "$DECK/assets/$(basename "$f")" ] || cp "$f" "$DECK/assets/"; done
fi
if [ "$KIND" = starter ]; then
  # sample media so the skeleton builds; replace with the real photo/clip (same names or edit the slide)
  A="$DECK/assets"
  [ -f "$A/sample_photo.jpg" ] || ffmpeg -loglevel error -f lavfi -i "mandelbrot=size=1600x1000:rate=1" -frames:v 1 -q:v 3 "$A/sample_photo.jpg"
  [ -f "$A/sample_clip.mp4" ] || ffmpeg -loglevel error -f lavfi -i "testsrc2=size=1280x720:rate=30" -t 6 \
    -c:v libx264 -pix_fmt yuv420p -movflags +faststart "$A/sample_clip.mp4"
  [ -f "$A/sample_poster.jpg" ] || ffmpeg -loglevel error -ss 1 -i "$A/sample_clip.mp4" -frames:v 1 -q:v 3 "$A/sample_poster.jpg"
fi
if [ "$KIND" = kickoff ] && [ ! -d "$TPL/assets" ]; then
  KA="$HOME/Downloads/발표자료/20261002_MS팀_출정식_모션리디자인/_build/assets"
  # the 49 MB of photos/videos live in that delivery folder (the author's PC) unless a team kit bundled them
  # into templates/kickoff/assets (export_kit.sh); elsewhere copy them to $DECK/assets and drop assetsDir
  [ -d "$KA" ] || echo "WARN: kickoff assets not found at $KA; copy that _build/assets to $DECK/assets and remove assetsDir from $DECK/deck.config.js"
fi
printf 'node_modules/\nout/\n' > "$DECK/.gitignore"
echo "created $DECK from $KIND"
echo "next: bash \"$SKILL_DIR/scripts/build.sh\" \"$DECK\""
