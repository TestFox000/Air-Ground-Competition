#!/bin/bash
# ============================================================================
# cam_record.sh - record LRCP AR0234 USB camera on dzsmart (Jetson Orin Nano)
# via GStreamer. MJPG source -> [H.264 (x264enc) | MJPG passthrough].
#
# Usage:
#   cam_record.sh start [-d DEV] [-r WxH] [-f FPS] [-t SECS] [-o DIR] [--mjpeg] [--mp4]
#   cam_record.sh stop
#   cam_record.sh status
#
# Defaults: /dev/video0 1280x720@30 unlimited, H.264 .mp4, ~/Videos/recordings
#   --mjpeg   zero-CPU MJPEG passthrough into .mkv (crash-safe container)
#   --mp4     force .mp4 container (qtmux); clean stop finalizes the index
# ============================================================================
set -u
CACHE="$HOME/.cache"; mkdir -p "$CACHE"
PIDFILE="$CACHE/cam_record.pid"
META="$CACHE/cam_record.meta"
DEV="/dev/video0"; W=1280; H=720; FPS=30; DUR=0
OUT="$HOME/Videos/recordings"; MODE=h264; CONT=mp4

alive() { [ -f "$1" ] && kill -0 "$(cat "$1")" 2>/dev/null; }

do_stop() {
  if ! alive "$PIDFILE"; then echo "not running"; rm -f "$PIDFILE" "$META"; exit 0; fi
  PID=$(cat "$PIDFILE"); FILE=$(cut -d'|' -f2 "$META" 2>/dev/null || echo "")
  echo "sending SIGINT (EOS) to $PID ..."
  kill -INT "$PID" 2>/dev/null
  for i in $(seq 1 60); do kill -0 "$PID" 2>/dev/null || break; sleep 0.5; done
  if kill -0 "$PID" 2>/dev/null; then echo "force kill"; kill -9 "$PID" 2>/dev/null; fi
  rm -f "$PIDFILE" "$META"
  [ -n "$FILE" ] && echo "recorded file: $FILE ($(du -h "$FILE" | cut -f1))"
  echo "stopped"
}

do_status() {
  if ! alive "$PIDFILE"; then
    echo "status: NOT running"
    ls -lht "$OUT" 2>/dev/null | head -6
    exit 0
  fi
  PID=$(cat "$PIDFILE")
  START=$(cut -d'|' -f1 "$META"); FILE=$(cut -d'|' -f2 "$META"); MODE_M=$(cut -d'|' -f3 "$META")
  NOW=$(date +%s); EL=$((NOW - START))
  printf "status: RUNNING pid=%s mode=%s elapsed=%ds\n" "$PID" "$MODE_M" "$EL"
  printf "file: %s (%s)\n" "$FILE" "$(du -h "$FILE" 2>/dev/null | cut -f1)"
  echo "--- log tail ---"; tail -n 4 "${FILE}.log" 2>/dev/null
}

[ $# -lt 1 ] && { echo "usage: $0 start|stop|status"; exit 1; }
CMD="$1"; shift

case "$CMD" in
  stop) do_stop; exit 0 ;;
  status) do_status; exit 0 ;;
esac

while [ $# -gt 0 ]; do
  case "$1" in
    -d) DEV="$2"; shift 2;;
    -r) W="${2%x*}"; H="${2#*x}"; shift 2;;
    -f) FPS="$2"; shift 2;;
    -t) DUR="$2"; shift 2;;
    -o) OUT="$2"; shift 2;;
    --mjpeg) MODE=mjpeg; shift;;
    --mp4) CONT=mp4; shift;;
    *) echo "unknown arg: $1"; exit 1;;
  esac
done

if alive "$PIDFILE"; then echo "ERROR: already running (pid $(cat "$PIDFILE")). stop first."; exit 1; fi
[ -e "$DEV" ] || { echo "ERROR: $DEV not found"; exit 1; }
mkdir -p "$OUT"
AVAIL=$(df -BG --output=avail "$OUT" 2>/dev/null | tail -1 | tr -dc '0-9')
[ "${AVAIL:-0}" -lt 2 ] && { echo "ERROR: <2GB free in $OUT"; exit 1; }
TS=$(date +%Y%m%d_%H%M%S)
EXT=$([ "$CONT" = mp4 ] && [ "$MODE" = h264 ] && echo mp4 || echo mkv)
FILE="$OUT/cam_${TS}.${EXT}"
LOG="$FILE.log"

if [ "$MODE" = h264 ]; then
  [ "$W" = 1280 ] && BR=6000 || BR=8000
  PIPE="v4l2src device=$DEV ! image/jpeg,width=$W,height=$H,framerate=$FPS/1 ! jpegdec ! videoconvert ! x264enc speed-preset=ultrafast bitrate=$BR key-int-max=$((FPS*2)) ! video/x-h264,profile=baseline"
  [ "$EXT" = mp4 ] && MUX=qtmux || MUX=matroskamux
else
  PIPE="v4l2src device=$DEV ! image/jpeg,width=$W,height=$H,framerate=$FPS/1"
  MUX=matroskamux
fi

RUNNER="gst-launch-1.0 -q -e $PIPE ! $MUX ! filesink location=$FILE"
[ "$DUR" -gt 0 ] 2>/dev/null && RUNNER="timeout -s INT $DUR $RUNNER"

echo "pipeline: $RUNNER"
setsid bash -c "$RUNNER" > "$LOG" 2>&1 < /dev/null &
NEWPID=$!
sleep 2
if ! kill -0 "$NEWPID" 2>/dev/null; then echo "ERROR: gst exited immediately:"; cat "$LOG"; exit 1; fi
echo "$NEWPID" > "$PIDFILE"
echo "$(date +%s)|$FILE|$MODE" > "$META"
echo "STARTED pid=$NEWPID mode=$MODE dev=$DEV ${W}x${H}@${FPS} out=$FILE"
