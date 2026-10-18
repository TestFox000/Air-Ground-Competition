#!/usr/bin/env bash
set -u
export DISPLAY=:1
export XAUTHORITY=/home/duzhong/.Xauthority
pkill -x gst-launch-1.0 2>/dev/null
sleep 1
ls -l /home/duzhong/dzuav/engines_d13
cd /tmp/dzuav_view || exit 1
setsid -f ./build/viewer /dev/video0 \
  /home/duzhong/dzuav/engines_d13/hbb_detector_d13_fp16.engine \
  /home/duzhong/dzuav/engines_d13/cell_classifier_v4d_fp16.engine \
  1280x720@30 > /tmp/dzuav_view.log 2>&1 < /dev/null
sleep 6
echo ---PROC---
pgrep -a viewer
echo ---LOG---
tail -12 /tmp/dzuav_view.log
