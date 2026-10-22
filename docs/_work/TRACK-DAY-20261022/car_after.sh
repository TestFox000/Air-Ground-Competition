#!/bin/bash
LOG=/home/duzhong/r3_field_20261021/logs
echo "=== DRIVE SECTION ==="
sed -n '236,300p' "$LOG/7_path_aim.log"
echo "=== GIMBAL TAIL ==="
tail -25 "$LOG/5_gimbal.log"
echo "=== JUDGMENT TAIL ==="
tail -12 "$LOG/6_judgment.log"
echo "=== FINISH BANNER ==="
grep -n -A12 '【任务结束】' "$LOG/7_path_aim.log" | tail -16
