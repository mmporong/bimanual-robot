# Shared environment for the motion-deck scripts (sourced, not executed).
SKILL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENGINE="$SKILL_DIR/engine"
CACHE="${MOTION_DECK_HOME:-$HOME/.cache/motion-deck}"
export MOTION_DECK_HOME="$CACHE"
export PYTHONDONTWRITEBYTECODE=1
export NODE_PATH="$ENGINE:$CACHE/node/node_modules"
export MOTION_DECK_FONTS="${MOTION_DECK_FONTS:-$CACHE/fonts}"
export PPT_MASTER_SCRIPTS="${PPT_MASTER_SCRIPTS:-$CACHE/ppt-master/skills/ppt-master/scripts}"
source "$SKILL_DIR/scripts/fontconf.sh"; md_fontconf
export MOTION_DECK_FONTCONF="$CACHE/fonts.conf"
[ -d "$CACHE/node/node_modules/pptxgenjs" ] && [ -d "$PPT_MASTER_SCRIPTS" ] && [ -f "$MOTION_DECK_FONTS/Pretendard-Bold.ttf" ] \
  || { echo "motion-deck cache missing: run bash \"$SKILL_DIR/scripts/setup.sh\"" >&2; exit 2; }
