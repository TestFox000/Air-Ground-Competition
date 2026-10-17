#!/bin/bash
# 清理旧信标残留子进程 -> 重启单一信标 -> 校验发布者/订阅者各为 1
set +u
echo "=== 1) 精确终止残留的旧信标子进程 ==="
for p in 6981 6983 6999 7000; do
  if kill -0 "$p" 2>/dev/null; then kill -TERM "$p" 2>/dev/null && echo "TERM $p"; fi
done
sleep 3
for p in 6981 6983 6999 7000; do
  if kill -0 "$p" 2>/dev/null; then kill -KILL "$p" 2>/dev/null && echo "KILL(兜底) $p"; fi
done
sleep 2

echo "=== 2) 停掉刚起的信标组，并清掉任何 dzlink_hello 残留 ==="
NEW=$(pgrep -f "beacon.sh 7200 dzsmart" | head -1)
if [ -n "$NEW" ]; then
  PG=$(ps -o pgid= -p "$NEW" | tr -d ' ')
  echo "停 PID=$NEW PGID=$PG"
  kill -TERM -"$PG" 2>/dev/null
  sleep 3
  kill -KILL -"$PG" 2>/dev/null
fi
sleep 1
for p in $(pgrep -f dzlink_hello); do kill -KILL "$p" 2>/dev/null && echo "KILL 残留 $p"; done
sleep 2
echo "--- 启动前残留检查（应为“干净”） ---"
pgrep -af dzlink_hello | grep -v pgrep || echo "(干净)"

echo "=== 3) 启动全新单一信标 7200s ==="
setsid nohup bash /tmp/beacon.sh 7200 dzsmart > /tmp/dzlink_beacon_round3.log 2>&1 < /dev/null &
sleep 12

echo "=== 4) 校验：/dzlink_hello 发布者与订阅者各应为 1 ==="
source /opt/ros/humble/setup.bash
timeout 15 ros2 topic info /dzlink_hello 2>&1 | head -4

echo "=== 5) 自收速率校验（再等 12s，2Hz 应约 24 条） ==="
sleep 12
echo "本机 dzsmart 条数: $(grep -c 'HELLO from dzsmart' /tmp/dzlink_dzsmart.log)"
echo "对端 duzhong 条数: $(grep -c 'HELLO from duzhong' /tmp/dzlink_dzsmart.log)"
pgrep -af beacon.sh | head -2
echo "=== 6) 三个常驻进程确认 ==="
pgrep -af "pc2_bridge|livox_ros_driver2_node|beacon.sh" | cut -c1-72
echo "=== DONE ==="
