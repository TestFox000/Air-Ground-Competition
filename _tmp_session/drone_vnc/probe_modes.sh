#!/bin/bash
for m in 1920x1200 1920x1080 1600x1200 1280x960 1280x800 1280x720 2560x1440 1024x768; do
  W=${m%x*}; H=${m#*x}
  if timeout 6 gst-launch-1.0 -q v4l2src device=/dev/video0 num-buffers=3 ! image/jpeg,width=$W,height=$H,framerate=30/1 ! fakesink >/dev/null 2>&1; then
    echo "OK   $m"
  else
    echo "FAIL $m"
  fi
done
