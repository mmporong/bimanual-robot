#!/usr/bin/env bash
# One-time, idempotent setup of the motion-deck cache (~/.cache/motion-deck or $MOTION_DECK_HOME):
#   node/        pptxgenjs · react · react-icons · sharp (npm ci from engine/package-lock.json)
#   ppt-master/  hugohe3/ppt-master pinned commit, sparse: SKILL.md + LICENSE + SPONSORS* + scripts/
#                (its scripts refuse to run without those identity files, so it is fetched, not vendored)
#   fonts/       Pretendard v1.3.9 Regular/SemiBold/Bold TTF + OFL licence, sha256-checked
#   pylib/       defusedxml for the Anthropic pptx validate.py (only if the system python lacks it)
#   fonts.conf   fontconfig file that adds fonts/ for LibreOffice renders
set -euo pipefail
SKILL_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CACHE="${MOTION_DECK_HOME:-$HOME/.cache/motion-deck}"
source "$SKILL_DIR/scripts/fontconf.sh"
PM_COMMIT=44c10ed0bc3a9e1df7a25aa179ae7c26db09469b
FONT_ZIP_URL=https://github.com/orioncactus/pretendard/releases/download/v1.3.9/Pretendard-1.3.9.zip
mkdir -p "$CACHE"

# 1) node packages (sharp 0.35 needs Node >= 20.9; Ubuntu's apt nodejs is 18, so use nvm or NodeSource)
command -v node >/dev/null || { echo "node not found: install Node.js 20.9+ (e.g. nvm install 22)" >&2; exit 1; }
node -e 'const [a,b]=process.versions.node.split(".").map(Number); process.exit(a>20||(a===20&&b>=9)?0:1)' \
  || { echo "Node.js $(node -v) is too old: need 20.9+ (e.g. nvm install 22)" >&2; exit 1; }
if ! cmp -s "$SKILL_DIR/engine/package-lock.json" "$CACHE/node/package-lock.json" || [ ! -d "$CACHE/node/node_modules/pptxgenjs" ]; then
  mkdir -p "$CACHE/node"
  cp "$SKILL_DIR/engine/package.json" "$SKILL_DIR/engine/package-lock.json" "$CACHE/node/"
  (cd "$CACHE/node" && npm ci --no-audit --no-fund --loglevel=error)
fi
echo "node      : $CACHE/node/node_modules ($(node -p "require('$CACHE/node/node_modules/pptxgenjs/package.json').version") pptxgenjs)"

# 2) ppt-master (pinned, sparse)
PM="$CACHE/ppt-master"
if [ "$(git -C "$PM" rev-parse HEAD 2>/dev/null || true)" != "$PM_COMMIT" ]; then
  rm -rf "$PM"
  git init -q "$PM"
  git -C "$PM" remote add origin https://github.com/hugohe3/ppt-master.git
  git -C "$PM" config core.sparseCheckout true
  printf '%s\n' /skills/ppt-master/SKILL.md /skills/ppt-master/LICENSE /skills/ppt-master/SPONSORS.md \
    /skills/ppt-master/SPONSORS_CN.md /skills/ppt-master/requirements.txt /skills/ppt-master/scripts/ > "$PM/.git/info/sparse-checkout"
  git -C "$PM" fetch -q --depth 1 --filter=blob:none origin "$PM_COMMIT"
  git -C "$PM" -c advice.detachedHead=false checkout -q FETCH_HEAD
fi
echo "ppt-master: $PM ($(git -C "$PM" rev-parse --short HEAD))"

# 3) Pretendard TTF (the exact files whose metrics engine/metrics.json was measured from)
FONTS="$CACHE/fonts"
check_fonts() {
  (cd "$FONTS" 2>/dev/null && sha256sum -c --quiet - >/dev/null 2>&1) <<'SUMS'
6d0af5258997aec7354a6e340fc2325ba321c410ca48b3af858c8c3d6e92a324  Pretendard-Regular.ttf
5e1c548732af70873103066c16e1369b9a8a871f0b38c321a1d5bc73e43cea2d  Pretendard-SemiBold.ttf
c16b88c670d23e83fa1170c954cbc4822d3b8dad3c3cde15d798a94b43d97985  Pretendard-Bold.ttf
SUMS
}
if ! check_fonts; then
  mkdir -p "$FONTS"
  TMPZ="$(mktemp "$CACHE/pretendard.XXXXXX.zip")"
  curl -fsSL --retry 2 -o "$TMPZ" "$FONT_ZIP_URL"
  unzip -o -q -j "$TMPZ" public/static/alternative/Pretendard-Regular.ttf public/static/alternative/Pretendard-SemiBold.ttf \
    public/static/alternative/Pretendard-Bold.ttf LICENSE.txt -d "$FONTS"
  rm -f "$TMPZ"
  check_fonts || { echo "Pretendard checksum mismatch in $FONTS" >&2; exit 1; }
fi
# LibreOffice renders read fonts through fontconfig, not from the pptx: point it at these TTFs so the
# review PNGs use Pretendard on PCs where it is not installed (otherwise Noto CJK changes line breaks)
md_fontconf
echo "fonts     : $FONTS (Pretendard v1.3.9, sha256 ok; renders via $CACHE/fonts.conf)"

# 4) python
python3 -c 'import lxml, fontTools' 2>/dev/null || { echo "python3 needs lxml and fonttools (Ubuntu: sudo apt install python3-lxml python3-fonttools)" >&2; exit 1; }
if ! python3 -c 'import defusedxml' 2>/dev/null && [ ! -d "$CACHE/pylib/defusedxml" ]; then
  # only the optional Anthropic validate.py needs it; a PC without pip still builds
  python3 -m pip install -q --target "$CACHE/pylib" defusedxml 2>/dev/null \
    || echo "WARN: defusedxml missing (Ubuntu: sudo apt install python3-defusedxml); build.sh skips validate.py"
fi
DX=missing; python3 -c 'import defusedxml' 2>/dev/null && DX=system || { [ -d "$CACHE/pylib/defusedxml" ] && DX="$CACHE/pylib"; }
echo "python    : lxml fontTools ok; defusedxml $DX"
command -v ffmpeg >/dev/null || echo "WARN: ffmpeg not found (new_deck.sh starter needs it for sample media; also slideshow checks)"
for t in soffice pdftoppm; do command -v "$t" >/dev/null || echo "WARN: $t not found (build.sh skips the PNG renders)"; done
command -v Xvfb >/dev/null || echo "WARN: Xvfb not found (lo_show.sh slideshow recording needs it)"
echo "setup ok"
