#!/bin/bash
# 最终状态快照（发布交接前的对照基线）
set +u
source /opt/ros/humble/setup.bash
LOG=/tmp/dzlink_dzsmart.log

echo "=== 时间 ==="; date +%F_%T

echo "=== 信标速率（20s 新增，单一 2Hz 发布者应约 40 条） ==="
c1=$(grep -c 'HELLO from dzsmart' "$LOG"); sleep 20; c2=$(grep -c 'HELLO from dzsmart' "$LOG")
echo "20s 新增 $((c2 - c1)) 条"
echo "对端 duzhong 累计（本轮新日志）: $(grep -c 'HELLO from duzhong' "$LOG")"

echo "=== 常驻进程 ==="
pgrep -af pc2_bridge | cut -c1-72
pgrep -af livox_ros_driver2_node | cut -c1-72
pgrep -af beacon.sh

echo "=== 关键话题速率 ==="
timeout 14 stdbuf -oL ros2 topic hz /livox/points 2>&1 | tail -2
timeout 14 stdbuf -oL ros2 topic hz /livox/imu 2>&1 | tail -2
timeout 10 stdbuf -oL ros2 topic hz /dzlink/seq 2>&1 | tail -2

echo "=== 桥 STAT（subs_* = 已匹配订阅者数，可用于反向发现取证） ==="
grep STAT /tmp/dzlink_pc2_bridge.log | tail -2

echo "=== 保全的证据日志（第一轮+第二轮，156 条对端） ==="
wc -l /tmp/dzlink_dzsmart.round1-round2.log
echo DONE
