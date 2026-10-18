#!/bin/bash
pkill -x dzuav_view_1280 || true
pkill -x dzuav_view_sq || true
sleep 1
cd /tmp/dzuav_view
DISPLAY=:1 VIEW_W=1280 VIEW_H=1280 CAP_W=1920 CAP_H=1200 CAP_FPS=30 setsid ./dzuav_view_sq > run_sq.log 2>&1 < /dev/null &
sleep 15
echo "--- procs"
pgrep -a dzuav_view_sq || true
echo "--- window"
DISPLAY=:1 xwininfo -root -tree | grep -i -A2 "dzuav live" || true
echo "--- latest"
ls -la /tmp/dzuav_view/latest_sq.jpg 2>/dev/null || true
echo "--- log"
tail -6 run_sq.log || true
echo "--- shot"
DISPLAY=:1 xwd -root -out /tmp/dzuav_view/root_sq.xwd 2>/dev/null && ls -la /tmp/dzuav_view/root_sq.xwd || true
