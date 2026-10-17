#!/bin/bash
# t19-G1 取证：备份目录与 hub 文件的时间戳（无人机时钟），用于解释“目录名比部署后还晚”的观感
UB=/home/duzhong/dzlink_tools
echo "drone_now=$(date '+%Y-%m-%dT%H:%M:%S.%N %z')"
echo
echo "=== t18 备份目录本身 ==="
stat -c '%y  %n' "$UB"/_backups/t18_20261012_200918
echo "=== 目录内 4 个副本（cp -p 保留源 mtime）==="
stat -c '%y  %n' "$UB"/_backups/t18_20261012_200918/*.py
echo "=== 当前 hub 4 个文件（= t18 部署后的 mtime）==="
stat -c '%y  %n' "$UB"/dzlink_supervisor.py "$UB"/dz_send_waypoint.py \
                "$UB"/test_dzlink_supervisor.py "$UB"/test_dz_send_waypoint.py
echo "=== 上一次部署（t16）的备份目录，作对照 ==="
stat -c '%y  %n' "$UB"/_backups/t16_* 2>/dev/null | head -3
echo "### done"
