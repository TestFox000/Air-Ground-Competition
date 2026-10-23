#!/bin/bash
# 无人机侧：后台启动 dzlink 桥接（脱离 SSH 会话，独立存活）
# 用法: bash /tmp/start_bridge_bg.sh
set -u
pkill -f dzlink_livox_bridge.py 2>/dev/null
sleep 1

setsid nohup bash -lc 'source /opt/ros/humble/setup.bash && source $HOME/dzpx4/install/setup.bash && exec python3 /tmp/dzlink_livox_bridge.py --ros-args -p seq_rate:=50.0' \
  > /tmp/dzlink_bridge.log 2>&1 < /dev/null &

sleep 8
echo "=== 进程 ==="
pgrep -af dzlink_livox_bridge.py || echo "(未运行)"
echo "=== 日志前 25 行 ==="
head -25 /tmp/dzlink_bridge.log
echo "=== 日志尾部 ==="
tail -5 /tmp/dzlink_bridge.log
