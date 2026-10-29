#!/usr/bin/env bash
# One command from nothing to a built deck: setup (idempotent) -> new_deck (skipped if slides/ exists) -> build.
#   usage: make_deck.sh <deck_dir> [starter|kickoff]
#   Re-running on an existing deck folder just rebuilds it; edit slides/*.js between runs.
set -euo pipefail
SKILL_DIR="$(cd "$(dirname "$0")/.." && pwd)"
DECK="${1:?usage: make_deck.sh <deck_dir> [starter|kickoff]}"; KIND="${2:-starter}"
echo "== setup"
bash "$SKILL_DIR/scripts/setup.sh"
if [ -d "$DECK/slides" ]; then
  echo "== $DECK already has slides/: rebuilding it (template not copied again)"
else
  echo "== new deck ($KIND)"
  bash "$SKILL_DIR/scripts/new_deck.sh" "$DECK" "$KIND"
fi
bash "$SKILL_DIR/scripts/build.sh" "$DECK"
D="$(cd "$DECK" && pwd)"
cat <<MSG

done
  show file : $D/out/deck.pptx
  previews  : $D/out/render/  (one PNG per slide)
  edit      : $D/slides/*.js  (one file per slide), $D/deck.config.js (title, team, theme, logo)
  rebuild   : bash "$SKILL_DIR/scripts/build.sh" "$D"
MSG
