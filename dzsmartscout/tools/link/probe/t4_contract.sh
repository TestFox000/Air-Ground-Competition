#!/bin/bash
# t4 第三轮取证：车侧部署的 dzlink 契约（QoS 真值）+ 域号进入 start_all.sh 的路径（只读）
WS="$HOME/dzrobot"
echo "=== A. 车侧 contract.py：话题 + QoS 常量定义 ==="
C="$WS/src/dzbrain/dzbrain/io/contract.py"
if [ -r "$C" ]; then
  echo "-- QoS 常量定义 --"
  grep -nE "^QOS_|^_QOS|QoSProfile|ReliabilityPolicy|HistoryPolicy|DurabilityPolicy|depth=" "$C" | sed 's/^/  /'
  echo "-- waypoint_cmd / drone_in / drone_out 三行 + 上下文 --"
  grep -n -A3 -E '"waypoint_cmd"|"drone_in"|"drone_out"' "$C" | sed 's/^/  /'
else
  echo "  (无 $C)"
fi

echo
echo "=== B. QoS 常量各自的取值（逐个定义块）==="
[ -r "$C" ] && sed -n '1,60p' "$C" | sed 's/^/  /'

echo
echo "=== C. 域号如何进入 start_all.sh（当前那份）==="
S="$WS/start_all.sh"
if [ -r "$S" ]; then
  grep -n "dzros.env\|ROS_DOMAIN_ID\|DZ_ROS_DOMAIN" "$S" | sed 's/^/  /'
  echo "  -- 上述行前后 4 行 --"
  L=$(grep -n "dzros.env" "$S" | head -1 | cut -d: -f1)
  [ -n "$L" ] && sed -n "$((L-4)),$((L+4))p" "$S" | sed 's/^/  /'
  echo "  -- start_all.sh 的 mtime / md5 --"
  stat -c '  %n  size=%s mtime=%y' "$S"
  md5sum "$S" | sed 's/^/  /'
else
  echo "  (无 $S)"
fi

echo
echo "=== D. 车侧 .bashrc 118 那一段的上下文 ==="
sed -n '118,132p' "$HOME/.bashrc" | cat -n | sed 's/^/  /'

echo
echo "=== E. robot.yaml 的 drone 段 + 相关时效参数（车侧真值）==="
Y="$WS/config/robot.yaml"
[ -r "$Y" ] && grep -nE "^\s*(drone|enabled|topic_in|topic_out|heartbeat_offline_s|info_stale_s|bridge_info_wait_s|auto_start|auto_start_delay_s|auto_plan)" "$Y" | sed 's/^/  /'

echo
echo "=== F. 车侧消费端 drone.py 的 ack / car_state 真实字段（双向对账要用）==="
D="$WS/src/dzbrain/dzbrain/tasks/drone.py"
if [ -r "$D" ]; then
  grep -nE '"ack"|"car_state"|"seq"|"ok"|"type"|json.dumps' "$D" | head -40 | sed 's/^/  /'
fi

echo
echo "=== G. /dzlink 话题的发布者声明（谁能写）==="
grep -rn "drone_out\|car_to_drone" "$WS/src/dzbrain/dzbrain" 2>/dev/null | grep -v "\.pyc" | head -20 | sed 's/^/  /'

echo "### t4_contract done on $(hostname)"
