#!/bin/bash
L=/home/duzhong/robot_logs
echo "=== GIMBAL PITCH LINES ==="
grep -n -E 'pitch' $L/5_gimbal.log 2>/dev/null | head -14
echo "=== GIMBAL TAIL ==="
tail -6 $L/5_gimbal.log 2>/dev/null
echo "=== PATH TAIL ==="
tail -8 $L/7_path.log 2>/dev/null
echo "=== DET TAIL ==="
tail -4 $L/4_det.log 2>/dev/null
echo "=== JUDGMENT TAIL ==="
tail -4 $L/6_judgment.log 2>/dev/null
echo "=== ERRORS ==="
grep -c -E 'SAFE_STOP|Traceback|ERROR' $L/7_path.log 2>/dev/null
echo "=== PROGRESS MARKER ==="
grep -o -E '到点 [0-9]+/[0-9]+ [A-N][0-9]+' $L/7_path.log 2>/dev/null | tail -3
grep -c 'FINISH' $L/7_path.log 2>/dev/null
