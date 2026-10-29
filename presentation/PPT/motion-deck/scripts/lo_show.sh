#!/usr/bin/env bash
# Record a LibreOffice slideshow of a pptx on a private Xvfb display (4 fps PNG frames) to check motion and video.
#   usage: lo_show.sh deck.pptx [seconds=10] [frames_dir=<pptx dir>/show_frames]
#   tip:   ONLY=7 build.sh first, then record out/only.pptx so the slide under test opens first
# Uses its own LibreOffice profile so it never attaches to the user's running Impress window.
set -u
FC="${MOTION_DECK_HOME:-$HOME/.cache/motion-deck}/fonts.conf"; [ -f "$FC" ] && export FONTCONFIG_FILE="$FC"  # Pretendard for the show
PPTX="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"; SECS="${2:-10}"; OUT="${3:-$(dirname "$PPTX")/show_frames}"; DISP=:97
W="$(dirname "$PPTX")"
mkdir -p "$OUT"; rm -f "$OUT"/f[0-9]*.png
Xvfb $DISP -screen 0 1600x900x24 -nolisten tcp >/dev/null 2>&1 & XPID=$!
sleep 1
# record from before LibreOffice starts so the opening of the slide (entrance build) is captured
timeout $((SECS+15)) ffmpeg -loglevel error -f x11grab -video_size 1600x900 -framerate 4 -i $DISP -t "$((SECS+8))" "$OUT/f%03d.png" & FPID=$!
DISPLAY=$DISP soffice --norestore --nologo -env:UserInstallation="file://$W/lo_profile" --show "$PPTX" >/dev/null 2>&1 & SPID=$!
wait $FPID
# stop only LibreOffice processes started with this private profile (match on executable name + profile path)
LOPIDS=$(ps -eo pid=,comm=,args= | awk -v prof="$W/lo_profile" '($2=="soffice.bin" || $2=="oosplash") && index($0, prof) {print $1}')
[ -n "$LOPIDS" ] && kill $LOPIDS 2>/dev/null; sleep 1
kill $XPID 2>/dev/null; sleep 1
LEFT=$(ps -eo pid=,comm=,args= | awk -v prof="$W/lo_profile" '($2=="soffice.bin" || $2=="oosplash") && index($0, prof) {print $1}')
[ -n "$LEFT" ] && echo "WARN: test soffice still alive: $LEFT" || echo "test LibreOffice stopped"
kill -0 $XPID 2>/dev/null && echo "WARN: Xvfb alive" || echo "Xvfb stopped"
ls "$OUT" | wc -l
