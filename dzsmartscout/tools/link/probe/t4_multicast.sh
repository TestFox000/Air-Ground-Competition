#!/bin/bash
# t4 组播转发能力测试（AP 是否放过 DDS 发现用的组播）
# 用法: t4_multicast.sh recv [秒]   /   t4_multicast.sh send [次数]
MODE="${1:-recv}"
DUR="${2:-20}"
source /opt/ros/humble/setup.bash
echo "== host=$(hostname) mode=$MODE =="
if [ "$MODE" = "recv" ]; then
  echo "== 监听 ros2 multicast（默认 225.0.0.1:49150）${DUR}s =="
  timeout "$DUR" ros2 multicast receive 2>&1
  echo "== recv exit=$? （124=超时无包）=="
else
  sleep 1
  for i in 1 2 3; do
    echo "-- send #$i --"
    ros2 multicast send 2>&1
    sleep 1
  done
fi
echo "### t4_multicast done"
