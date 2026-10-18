#!/bin/bash
pkill -x dzuav_view_1280 || true
pkill -x dzuav_view || true
sleep 1
cd /tmp/dzuav_view
DISPLAY=:1 VIEW_W=1280 VIEW_H=1280 setsid ./dzuav_view_1280 > run_1280.log 2>&1 < /dev/null &
sleep 10
echo "--- procs"
pgrep -a dzuav_view || true
echo "--- window"
DISPLAY=:1 xwininfo -root -tree | grep -i -A2 "dzuav live" || true
echo "--- latest"
ls -la /tmp/dzuav_view/latest.jpg || true
echo "--- log"
tail -5 run_1280.log || true
echo "--- shot"
DISPLAY=:1 xwd -root -out /tmp/dzuav_view/root.xwd 2>/dev/null && ls -la /tmp/dzuav_view/root.xwd || true
