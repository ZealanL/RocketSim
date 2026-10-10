#!/usr/bin/env bash
# vis-shot.sh — deterministic screenshot harness for the vis viewer.
#
# Ensures a nested Xephyr display exists, runs a command on it, captures the
# root window with xwd, converts to PNG via tools/xwd2png.py.
#
# Usage:
#   tools/vis-shot.sh [-d DISPLAY] [-g GEOM] [-w WAIT_SECS] <out.png> -- <cmd...>
#
# Example (prove the harness with a trivial client):
#   tools/vis-shot.sh /tmp/probe.png -- xterm -e 'sleep 5'
# Example (viewer + sim):
#   tools/vis-shot.sh -w 8 /tmp/shot.png -- ./target/debug/vis_viewer 39201
#
# The Xephyr server is left running for reuse across shots.
set -euo pipefail

DISPLAY_NUM=":99"
GEOM="1280x720"
WAIT_SECS="3"

while getopts "d:g:w:" opt; do
  case "$opt" in
    d) DISPLAY_NUM="$OPTARG" ;;
    g) GEOM="$OPTARG" ;;
    w) WAIT_SECS="$OPTARG" ;;
    *) echo "usage: $0 [-d DISPLAY] [-g GEOM] [-w WAIT] <out.png> -- <cmd...>" >&2; exit 2 ;;
  esac
done
shift $((OPTIND - 1))

if [ "$#" -lt 3 ]; then
  echo "usage: $0 [-d DISPLAY] [-g GEOM] [-w WAIT] <out.png> -- <cmd...>" >&2
  exit 2
fi
OUT="$1"; shift
if [ "$1" != "--" ]; then echo "expected -- before command" >&2; exit 2; fi
shift

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! DISPLAY="$DISPLAY_NUM" xwininfo -root >/dev/null 2>&1; then
  echo "vis-shot: starting Xephyr on $DISPLAY_NUM ($GEOM)..."
  Xephyr "$DISPLAY_NUM" -screen "$GEOM" -nolisten tcp >/tmp/xephyr-"${DISPLAY_NUM#:}".log 2>&1 &
  for _ in $(seq 1 50); do
    DISPLAY="$DISPLAY_NUM" xwininfo -root >/dev/null 2>&1 && break
    sleep 0.1
  done
  DISPLAY="$DISPLAY_NUM" xwininfo -root >/dev/null 2>&1 \
    || { echo "vis-shot: Xephyr failed (see /tmp/xephyr-${DISPLAY_NUM#:}.log)" >&2; exit 1; }
fi

echo "vis-shot: running: $* (DISPLAY=$DISPLAY_NUM)"
env -u WAYLAND_DISPLAY DISPLAY="$DISPLAY_NUM" "$@" &
CMD_PID=$!
sleep "$WAIT_SECS"

XWD_TMP="$(mktemp /tmp/vis-shot-XXXXXX.xwd)"
DISPLAY="$DISPLAY_NUM" xwd -root -out "$XWD_TMP"
python3 "$REPO_ROOT/tools/xwd2png.py" "$XWD_TMP" "$OUT"
rm -f "$XWD_TMP"

if kill -0 "$CMD_PID" 2>/dev/null; then
  kill "$CMD_PID" 2>/dev/null || true
  wait "$CMD_PID" 2>/dev/null || true
  echo "vis-shot: command stopped (was still running)"
fi
echo "vis-shot: saved $OUT"
