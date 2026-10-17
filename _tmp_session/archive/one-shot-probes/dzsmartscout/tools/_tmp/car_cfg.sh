#!/bin/bash
CFG=/home/duzhong/dzrobot/config/robot.yaml
echo "=== 关键配置 ==="
grep -nE 'auto_start:|auto_start_delay:|mission_reset:|shoot_distance_target:|shoot_distance_max:' "$CFG" | head -10
echo
echo "=== mission_blocks 所在行 ==="
grep -n 'mission_blocks:' "$CFG" | head -3
echo
echo "=== 航点 token 数 ==="
awk '/mission_blocks:/{f=1} f{print} f&&/\]/{exit}' "$CFG" | tr ',' '\n' | grep -cE '"[A-Z]'
echo
echo "=== install 包数 ==="
ls -1 /home/duzhong/dzrobot/install 2>/dev/null | wc -l
echo
echo "=== 磁盘 ==="
df -h / | tail -1
echo
echo "=== 车侧 dzros.env ==="
cat /home/duzhong/dzrobot/dzros.env 2>/dev/null | grep -v '^#' | grep -v '^$'
