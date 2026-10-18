#!/bin/bash
# 用法: bash /tmp/rec.sh <输出csv> <秒数>
set +u
cd /home/duzhong/dzrobot || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
source ./dzros.env
OUT="${1:-/tmp/rec.csv}"
SEC="${2:-20}"
echo "recording $SEC s -> $OUT"
python3 /tmp/log_aim.py "$OUT" "$SEC"
wc -l "$OUT"
