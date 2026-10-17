#!/bin/bash
# ============================================================================
# cam_detect.sh - detect a freshly plugged camera and enumerate its abilities.
# READ-ONLY: writes nothing, starts nothing, installs nothing.
#
# Usage (from Windows, no file left on the drone):
#   Get-Content -Raw tools/drone/cam_detect.sh |
#     ssh -4 duzhong@<drone-ip> bash -s
# ============================================================================

echo "=== HOST / UPTIME ==="
hostname; uptime -p 2>/dev/null; uptime -s 2>/dev/null

echo
echo "=== /dev/video* ==="
if ls /dev/video* >/dev/null 2>&1; then
  ls -l /dev/video*
else
  echo "NONE  <-- camera not detected"
fi

echo
echo "=== /sys/class/video4linux (device names) ==="
found=0
for d in /sys/class/video4linux/video*; do
  [ -e "$d" ] || continue
  found=1
  n=$(cat "$d/name" 2>/dev/null)
  i=$(cat "$d/index" 2>/dev/null)
  echo "$(basename "$d")  name='$n'  index=$i"
done
[ "$found" = 0 ] && echo "none"

echo
echo "=== GStreamer device monitor (caps per device) ==="
if command -v gst-device-monitor-1.0 >/dev/null 2>&1; then
  gst-device-monitor-1.0 Video 2>&1 | head -80
else
  echo "gst-device-monitor-1.0 MISSING"
fi

echo
echo "=== USB tree (look for a new UVC device) ==="
lsusb 2>/dev/null || echo "lsusb MISSING"

echo
echo "=== kernel messages (camera / uvc) ==="
dmesg 2>/dev/null | grep -iE 'uvc|usb .*new|video[0-9]|camera' | tail -20 \
  || echo "dmesg not readable without root"

echo
echo "=== relevant kernel modules ==="
lsmod 2>/dev/null | grep -iE 'uvc|videodev|tegra.*vi|imx|ov[0-9]' || echo "none matched"

echo
echo "=== DONE ==="
