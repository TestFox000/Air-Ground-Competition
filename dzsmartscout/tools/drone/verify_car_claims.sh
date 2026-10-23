#!/bin/bash
# 无人机侧独立核验脚本（只读；为车侧交付的声明做旁证）
# 用法: bash /tmp/verify_car_claims.sh
LOG=/tmp/dzlink_dzsmart.log
echo "=== 1) 本机时间 ==="
date '+%F %T %Z'

echo "=== 2) livox 驱动与桥接进程 ==="
pgrep -af livox_ros_driver2_node || echo "(livox: none)"
pgrep -af dzlink_livox_bridge || echo "(bridge: none)"
pgrep -af beacon.sh || echo "(beacon: none)"

echo "=== 3) 信标日志概况 ==="
if [ -f "$LOG" ]; then
  ls -l "$LOG" | awk '{print "size="$5" mtime="$6" "$7" "$8}'
  echo "--- 统计 ---"
  echo "duzhong 总条数: $(grep -c 'HELLO from duzhong' "$LOG")"
  echo "dzsmart 总条数: $(grep -c 'HELLO from dzsmart' "$LOG")"
  echo "--- 对端(duzhong)首条与末条(带行号) ---"
  grep -n 'HELLO from duzhong' "$LOG" | head -2
  grep -n 'HELLO from duzhong' "$LOG" | tail -2
  echo "--- 对端消息的行号分段(每 100 行一桶，看是否有第二个突发窗口) ---"
  grep -n 'HELLO from duzhong' "$LOG" | awk -F: '{b=int($1/100)*100; c[b]++} END {for (k in c) print k"-"k+99": "c[k]}' | sort -n
  echo "--- 日志行的原始格式样本 ---"
  sed -n '1p;1151p;1200p' "$LOG"
else
  echo "(无信标日志: $LOG 不存在)"
fi

echo "=== 4) 当前 DDS 域内可见节点（含远程！） ==="
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzpx4/install/setup.bash" 2>/dev/null
timeout 15 ros2 node list 2>&1 | head -30

echo "=== 5) 两个 livox 话题的端点统计（看有无远程订阅者） ==="
timeout 15 ros2 topic info /livox/imu --verbose 2>&1 | grep -E 'Node name|Node namespace|Topic type|GID|Reliability|Durability|count|Publisher|Subscription' | head -40
echo "--- lidar ---"
timeout 15 ros2 topic info /livox/lidar 2>&1 | head -10

echo "=== 6) 网卡与组播加入情况 ==="
ip -4 -o addr show | awk '{print $2, $4}'
echo "--- mDNS 组播组成员 (239.255.0.1) ---"
ip -o maddr show 2>/dev/null | grep -E '239\.255\.0\.1|eth|wlan' | head
echo "=== DONE ==="
