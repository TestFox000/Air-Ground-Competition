#!/bin/bash
# 续期字符串信标（beacon.sh 启动时会截断自己的日志，故先保全旧日志）
set +u
echo "=== 1) 保全既有证据日志 ==="
if [ -f /tmp/dzlink_dzsmart.log ]; then
  cp -f /tmp/dzlink_dzsmart.log /tmp/dzlink_dzsmart.round1-round2.log
  wc -l /tmp/dzlink_dzsmart.round1-round2.log
  echo "对端条数: $(grep -c 'HELLO from duzhong' /tmp/dzlink_dzsmart.round1-round2.log)"
else
  echo "(无旧日志)"
fi

echo "=== 2) 终止旧信标（整个进程组，避免残留 pub/echo） ==="
OLD=$(pgrep -f "beacon.sh 3600 dzsmart" | head -1)
if [ -n "$OLD" ]; then
  PGID=$(ps -o pgid= -p "$OLD" | tr -d ' ')
  echo "旧信标 PID=$OLD PGID=$PGID"
  kill -TERM -"$PGID" 2>/dev/null && echo "已终止进程组" || echo "终止失败"
else
  echo "(未找到旧信标)"
fi
sleep 3
echo "--- 残留 dzlink_hello 相关进程（应为空） ---"
pgrep -af "dzlink_hello" | grep -v pgrep || echo "(无残留)"

echo "=== 3) 启动新信标：7200s（2 小时） ==="
setsid nohup bash /tmp/beacon.sh 7200 dzsmart > /tmp/dzlink_beacon_round3.log 2>&1 < /dev/null &
sleep 8
pgrep -af beacon.sh | head -3

echo "=== 4) /dzlink_hello 发布者数量（必须为 1，否则会双倍计数） ==="
source /opt/ros/humble/setup.bash
timeout 12 ros2 topic info /dzlink_hello 2>&1 | head -4

echo "=== 5) 新信标日志与统计 ==="
head -5 /tmp/dzlink_beacon_round3.log
echo "--- 新日志条数 ---"
grep -c "HELLO from dzsmart" /tmp/dzlink_dzsmart.log 2>/dev/null

echo "=== 6) 当前全量话题与进程 ==="
timeout 12 ros2 topic list -t 2>/dev/null | grep -iE "livox|dzlink"
pgrep -af "pc2_bridge|livox_ros_driver2_node" | cut -c1-70
echo "=== DONE ==="
