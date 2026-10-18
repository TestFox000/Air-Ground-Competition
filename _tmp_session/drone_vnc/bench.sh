#!/bin/bash
cd /tmp/dzuav_view
pkill -x dzuav_view_sq || true
pkill -x dzuav_view_perf || true
sleep 1
echo "=== TEST A: capture only (no inference)"
DISPLAY=:1 NO_INFER=1 VIEW_W=1280 VIEW_H=1280 CAP_W=1920 CAP_H=1200 CAP_FPS=30 setsid ./dzuav_view_perf > run_cap.log 2>&1 < /dev/null &
sleep 12
pkill -x dzuav_view_perf || true
sleep 1
grep STAT run_cap.log | tail -3
echo "=== TEST B: with inference"
DISPLAY=:1 NO_INFER=0 VIEW_W=1280 VIEW_H=1280 CAP_W=1920 CAP_H=1200 CAP_FPS=30 setsid ./dzuav_view_perf > run_inf.log 2>&1 < /dev/null &
sleep 6
timeout 5 tegrastats --interval 1000 2>&1 | tail -4
sleep 10
grep STAT run_inf.log | tail -3
pkill -x dzuav_view_perf || true
sleep 1
echo "=== restart normal viewer"
DISPLAY=:1 VIEW_W=1280 VIEW_H=1280 CAP_W=1920 CAP_H=1200 CAP_FPS=30 setsid ./dzuav_view_sq > run_sq.log 2>&1 < /dev/null &
sleep 6
pgrep -a dzuav_view || true
