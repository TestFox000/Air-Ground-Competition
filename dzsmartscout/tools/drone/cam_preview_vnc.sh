#!/bin/bash
# ============================================================================
# cam_preview_vnc.sh - show a live camera picture on the VNC desktop (DISPLAY :1)
#
# ZERO INSTALLS: uses only GStreamer plugins already present on the drone
#   (v4l2src, jpegdec, nvjpegdec, nvvidconv, videoconvert, xvimagesink).
# Writes no file on the drone and changes no configuration.
#
# Usage:
#   ./cam_preview_vnc.sh [device] [--test]
#     device   defaults to /dev/video0
#     --test   only probe which format works, do not open a window
#
# From Windows (no file left on the drone):
#   Get-Content -Raw tools/drone/cam_preview_vnc.sh |
#     ssh -4 duzhong@<drone-ip> bash -s -- /dev/video0
# ============================================================================
set +u

DEV="/dev/video0"
TEST_ONLY=0
for a in "$@"; do
  case "$a" in
    --test) TEST_ONLY=1 ;;
    /dev/video*) DEV="$a" ;;
  esac
done

export DISPLAY=:1
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

if [ ! -e "$DEV" ]; then
  echo "ERROR: $DEV does not exist. Camera not plugged in?"
  echo "Available:"; ls -l /dev/video* 2>/dev/null || echo "  (none)"
  exit 2
fi

echo "device : $DEV"
echo "DISPLAY: $DISPLAY   XAUTHORITY: $XAUTHORITY"
echo

# --- candidate capture caps, best first -------------------------------------
# label|v4l2 caps|decoder chain
CANDS=(
  "MJPG 1920x1080@30|image/jpeg,width=1920,height=1080,framerate=30/1|jpegdec"
  "MJPG 1280x720@30|image/jpeg,width=1280,height=720,framerate=30/1|jpegdec"
  "MJPG 1280x720@60|image/jpeg,width=1280,height=720,framerate=60/1|jpegdec"
  "MJPG 640x480@30|image/jpeg,width=640,height=480,framerate=30/1|jpegdec"
  "YUY2 1280x720@30|video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1|"
  "YUY2 640x480@30|video/x-raw,format=YUY2,width=640,height=480,framerate=30/1|"
  "auto-negotiate|||decodebin"
)

WIN_LABEL=""
WIN_CAPS=""
WIN_DEC=""

echo "--- probing capture formats (30 frames each, no window) ---"
for c in "${CANDS[@]}"; do
  IFS='|' read -r label caps dec <<< "$c"

  if [ -z "$caps" ]; then
    # auto-negotiate branch
    if timeout 20 gst-launch-1.0 -q v4l2src device="$DEV" num-buffers=30 \
         ! "$dec" ! fakesink sync=false >/dev/null 2>&1; then
      WIN_LABEL="$label"; WIN_CAPS=""; WIN_DEC="$dec"
      echo "  OK    $label"
      break
    else
      echo "  FAIL  $label"
    fi
    continue
  fi

  if [ -n "$dec" ]; then
    chain="$dec ! videoconvert"
  else
    chain="videoconvert"
  fi

  if timeout 20 gst-launch-1.0 -q v4l2src device="$DEV" num-buffers=30 \
       ! "$caps" ! "$chain" ! fakesink sync=false >/dev/null 2>&1; then
    WIN_LABEL="$label"; WIN_CAPS="$caps"; WIN_DEC="$dec"
    echo "  OK    $label"
    break
  else
    echo "  FAIL  $label"
  fi
done

if [ -z "$WIN_LABEL" ]; then
  echo
  echo "RESULT: no format worked. Camera may be a CSI device needing nvarguscamerasrc,"
  echo "        or the node needs a permission/driver check. Run cam_detect.sh."
  exit 3
fi

echo
echo "RESULT: working format = $WIN_LABEL"

if [ "$TEST_ONLY" = "1" ]; then
  echo "(--test given: not opening a window)"
  exit 0
fi

# --- build the real pipeline and put it on the VNC desktop ------------------
if [ -n "$WIN_DEC" ]; then
  PIPE="v4l2src device=$DEV ! $WIN_CAPS ! $WIN_DEC ! videoconvert ! xvimagesink sync=false"
else
  PIPE="v4l2src device=$DEV ! $WIN_CAPS ! videoconvert ! xvimagesink sync=false"
fi
[ -z "$WIN_CAPS" ] && PIPE="v4l2src device=$DEV ! $WIN_DEC ! videoconvert ! xvimagesink sync=false"

echo "pipeline:"
echo "  $PIPE"
echo

# already showing something? report it, do not stack windows
if pgrep -f "xvimagesink" >/dev/null 2>&1; then
  echo "NOTE: an xvimagesink process is already running (PID $(pgrep -f xvimagesink | tr '\n' ' '))."
  echo "      Kill it first if you want a fresh window:  pkill -f xvimagesink"
fi

setsid gst-launch-1.0 -q $PIPE >/tmp/cam_preview.log 2>&1 < /dev/null &
sleep 4

if pgrep -f "xvimagesink" >/dev/null 2>&1; then
  echo "STARTED: gst-launch PID(s) $(pgrep -f xvimagesink | tr '\n' ' ')"
  echo "Look at the VNC desktop (:1) - a window should show the live camera."
  echo "Log: /tmp/cam_preview.log"
else
  echo "FAILED to start. Log tail:"
  tail -20 /tmp/cam_preview.log 2>/dev/null
  exit 4
fi
