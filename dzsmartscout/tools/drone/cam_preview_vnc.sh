#!/bin/bash
# ============================================================================
# cam_preview_vnc.sh - open/close the live camera window on the VNC desktop (DISPLAY :1)
#
# Revised 2026-10-18 (Lead):
#   - default sink is ximagesink (VNC has no XVideo; xvimagesink cannot open a window there)
#   - added --status / --stop
#   - documents the verified 1920x1080@30 MJPEG pipeline
#
# Usage (recommended: pipe from Windows, nothing is written on the drone):
#   Get-Content -Raw tools/drone/cam_preview_vnc.sh | ssh duzhong@<ip> bash -s -- [options]
#   (no option)   open the live preview
#   --stop        close the preview
#   --status      show preview/recording state
#   --test        probe formats only (camera must be free)
#   --xv          force xvimagesink (local X11 only)
#   e.g.  ssh ... bash -s -- --stop
# ============================================================================
set +u

DEV="/dev/video0"
MODE="on"
SINK="ximagesink"
for a in "$@"; do
  case "$a" in
    --stop)   MODE="stop" ;;
    --status) MODE="status" ;;
    --test)   MODE="test" ;;
    --xv)     SINK="xvimagesink" ;;
    /dev/video*) DEV="$a" ;;
  esac
done

# NOTE: pids() matches ONLY preview windows (ximagesink/xvimagesink); recording is never matched
pids() { pgrep -f "gst-launch.*v4l2src.*(xvimagesink|ximagesink)" 2>/dev/null; }
recs() { pgrep -f "gst-launch.*v4l2src.*(x264enc|filesink)" 2>/dev/null; }

case "$MODE" in
  status)
    P=$(pids)
    R=$(recs)
    if [ -n "$R" ]; then
      echo "RECORDING: PID(s) $(echo $R | tr '\n' ' ') (--stop will NOT kill it)"
      pgrep -af "gst-launch.*filesink" 2>/dev/null | head -1 | cut -c1-170
    fi
    if [ -n "$P" ]; then
      echo "PREVIEW RUNNING: PID(s) $(echo $P | tr '\n' ' ')"
      ps -o pid,etime,pcpu,args -p $(echo $P | tr ' ' ',') 2>/dev/null | sed -n '2,4p'
      echo "log: /tmp/cam_preview.log"; tail -3 /tmp/cam_preview.log 2>/dev/null
    else
      echo "PREVIEW: not running (no preview window)"
    fi
    exit 0 ;;
  stop)
    P=$(pids)
    if [ -z "$P" ]; then echo "NOT running (nothing to stop)"; exit 0; fi
    echo "stopping PID(s) $(echo $P | tr '\n' ' ')"
    kill -TERM $P 2>/dev/null; sleep 1
    P2=$(pids); [ -n "$P2" ] && kill -9 $P2 2>/dev/null
    if [ -z "$(pids)" ]; then echo "STOPPED"; else echo "FAILED: still running"; exit 1; fi
    exit 0 ;;
esac

export DISPLAY=:1
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

if [ ! -e "$DEV" ]; then
  echo "ERROR: $DEV not found (camera unplugged?)"; ls -l /dev/video* 2>/dev/null || echo "  (none)"; exit 2
fi

echo "device : $DEV"
echo "DISPLAY: $DISPLAY   XAUTHORITY: $XAUTHORITY"

# --- candidate formats (first one verified on 2026-10-18) --------------------
CANDS=(
  "MJPG 1920x1080@30|image/jpeg,width=1920,height=1080,framerate=30/1|jpegdec"
  "MJPG 1280x720@30|image/jpeg,width=1280,height=720,framerate=30/1|jpegdec"
  "MJPG 640x480@30|image/jpeg,width=640,height=480,framerate=30/1|jpegdec"
  "YUY2 1280x720@30|video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1|"
  "YUY2 640x480@30|video/x-raw,format=YUY2,width=640,height=480,framerate=30/1|"
)

WIN_LABEL=""; WIN_CAPS=""; WIN_DEC=""
for cand in "${CANDS[@]}"; do
  IFS='|' read -r label caps dec <<< "$cand"
  if [ -n "$dec" ]; then chain="$dec ! videoconvert"; else chain="videoconvert"; fi
  if timeout 20 gst-launch-1.0 -q v4l2src device="$DEV" num-buffers=30 ! "$caps" ! $chain ! fakesink sync=false >/dev/null 2>&1; then
    WIN_LABEL="$label"; WIN_CAPS="$caps"; WIN_DEC="$dec"; echo "  OK    $label"; break
  else
    echo "  FAIL  $label"
  fi
done

if [ -z "$WIN_LABEL" ]; then
  echo; echo "RESULT: no working format. If the camera is busy (preview/recording running), stop it first: --stop / cam_record.sh stop"
  exit 3
fi
echo; echo "RESULT: working format = $WIN_LABEL"
[ "$MODE" = "test" ] && { echo "(--test: window not opened)"; exit 0; }

if [ -n "$WIN_DEC" ]; then
  PIPE="v4l2src device=$DEV ! $WIN_CAPS ! $WIN_DEC ! videoconvert ! $SINK sync=false force-aspect-ratio=true"
else
  PIPE="v4l2src device=$DEV ! $WIN_CAPS ! videoconvert ! $SINK sync=false force-aspect-ratio=true"
fi
echo "pipeline:"; echo "  $PIPE"; echo

if [ -n "$(pids)" ]; then
  echo "NOTE: preview already running (PID $(pids | tr '\n' ' ')), not opening another; use --stop first"
  exit 0
fi

setsid gst-launch-1.0 -q $PIPE >/tmp/cam_preview.log 2>&1 < /dev/null &
sleep 4
if [ -n "$(pids)" ]; then
  echo "STARTED: gst-launch PID(s) $(pids | tr '\n' ' ')"
  echo "Look at VNC desktop :1 - a live window should be visible. log: /tmp/cam_preview.log"
else
  echo "FAILED. log tail:"; tail -20 /tmp/cam_preview.log 2>/dev/null; exit 4
fi
