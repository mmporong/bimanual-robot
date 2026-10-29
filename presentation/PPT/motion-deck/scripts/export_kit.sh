#!/usr/bin/env bash
# Author side: export this skill as a team kit (folder + zip) that teammates unzip and run without the
# author's PKM, caches or other skills. The skill source stays the single source; re-export after changes.
#   usage: export_kit.sh <out_dir>
#   env:   KICKOFF_ASSETS=<dir>  bundle the 출정식 photos/videos into templates/kickoff/assets
#                                (default: the delivery folder below if it exists; KICKOFF_ASSETS= to skip)
# writes <out_dir>/motion-deck/ and <out_dir>/motion-deck.zip; refuses to replace a folder that is not a kit,
# to run from inside a kit or an installed kit copy, and to write into or over this skill folder.
set -euo pipefail
SKILL_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$(realpath -m "${1:?usage: export_kit.sh <out_dir>}")"; SRC="$(realpath "$SKILL_DIR")"
KIT="$OUT/motion-deck"
[ -f "$SKILL_DIR/.motion-deck-kit" ] && { echo "refusing: this is an exported kit; export from the skill source (PKM)" >&2; exit 2; }
case "$OUT/" in "$SRC/"*) echo "refusing: $OUT is inside $SRC" >&2; exit 2;; esac
[ "$(realpath -m "$KIT")" = "$SRC" ] && { echo "refusing: $KIT is the skill source" >&2; exit 2; }
mkdir -p "$OUT"
KA="${KICKOFF_ASSETS-$HOME/Downloads/발표자료/20261002_MS팀_출정식_모션리디자인/_build/assets}"
[ -e "$KIT" ] && [ ! -f "$KIT/.motion-deck-kit" ] && { echo "refusing: $KIT exists and is not a kit" >&2; exit 2; }
rm -rf "$KIT" "$OUT/motion-deck.zip"
rsync -a --exclude .omc --exclude out --exclude node_modules --exclude __pycache__ --exclude '*.pyc' "$SKILL_DIR/" "$KIT/"
if [ -n "$KA" ] && [ -d "$KA" ]; then
  cp -rL "$KA" "$KIT/templates/kickoff/assets"
  sed -i '/^ *assetsDir:/d' "$KIT/templates/kickoff/deck.config.js"   # bundled assets are the ./assets default
  echo "kickoff assets bundled: $(du -sh "$KIT/templates/kickoff/assets" | cut -f1)"
else
  echo "kickoff assets not bundled (starter works; kickoff needs its assets)"
fi
REV="$(git -C "$SKILL_DIR" log -1 --format='%h %cs' -- . 2>/dev/null || echo unknown)"
[ -n "$(git -C "$SKILL_DIR" status --porcelain -- . ':!.omc' 2>/dev/null)" ] && REV="$REV +uncommitted changes"
printf 'motion-deck team kit\nsource: PKM 05_AI_Config/skills/motion-deck @ %s\nexported: %s\n' "$REV" "$(date '+%F %T %z')" > "$KIT/.motion-deck-kit"
(cd "$OUT" && zip -qr motion-deck.zip motion-deck)
echo "kit: $KIT"; echo "zip: $OUT/motion-deck.zip ($(du -h "$OUT/motion-deck.zip" | cut -f1))"
