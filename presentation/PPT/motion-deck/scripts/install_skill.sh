#!/usr/bin/env bash
# Team kit only: copy this kit into ~/.claude/skills/motion-deck and ~/.codex/skills/motion-deck so Claude Code
# and Codex can run it as the "motion-deck" skill. Re-run after updating the kit.
#   usage: install_skill.sh            (installs for whichever of ~/.claude and ~/.codex exist)
# Refuses to replace a motion-deck skill that was not installed from a kit (no .motion-deck-kit marker),
# e.g. the author's own copy that is synced from another source. A kit-installed copy is replaced as a whole,
# so edit the kit (or the deck folders), not the installed copy.
set -euo pipefail
KIT="$(cd "$(dirname "$0")/.." && pwd)"
[ -f "$KIT/.motion-deck-kit" ] || { echo "not a team kit (no .motion-deck-kit in $KIT): nothing installed" >&2; exit 2; }
n=0; skipped=0
for home in "$HOME/.claude" "$HOME/.codex"; do
  [ -d "$home" ] || continue
  dst="$home/skills/motion-deck"
  if [ "$(cd "$dst" 2>/dev/null && pwd -P)" = "$(cd "$KIT" && pwd -P)" ]; then echo "already here: $dst"; n=$((n+1)); continue; fi
  if [ -e "$dst" ] && [ ! -f "$dst/.motion-deck-kit" ]; then
    echo "skip $dst: an existing motion-deck skill that did not come from a kit" >&2; skipped=$((skipped+1)); continue
  fi
  mkdir -p "$home/skills"
  # stage outside skills/ so an interrupted copy never shows up as a second skill
  tmp="$home/.motion-deck-install.tmp"; rm -rf "$tmp"; cp -r "$KIT" "$tmp"; rm -rf "$dst"; mv "$tmp" "$dst"
  echo "installed: $dst"; n=$((n+1))
done
[ "$n" -gt 0 ] || { [ "$skipped" -gt 0 ] && echo "nothing installed (see skip above)" >&2 || echo "neither ~/.claude nor ~/.codex exists: install Claude Code or Codex first" >&2; exit 1; }
echo "start a new Claude Code / Codex session, then ask e.g. \"motion-deck으로 PPT 만들어줘\""
