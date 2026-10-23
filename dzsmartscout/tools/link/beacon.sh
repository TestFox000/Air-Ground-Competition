#!/usr/bin/env bash
# 无人机 <-> 小车  ROS2 直连双向探测（只做网络收发，不改任何配置）
#
# 用法:  bash beacon.sh [持续秒数] [本机标识]
# 例:    bash beacon.sh 60 dzsmart
#        bash beacon.sh 60 duzhong
#
# 原理: 本机一边以 2Hz 发布 /dzlink_hello，一边订阅同一话题。
#       收到「不是自己发的」HELLO => 跨机 DDS 直连成立。
DUR="${1:-60}"
SELF="${2:-$(hostname)}"
TOPIC=/dzlink_hello
LOG="/tmp/dzlink_${SELF}.log"

for d in /opt/ros/*/setup.bash; do [ -r "$d" ] && source "$d"; done
[ -r "$HOME/dzpx4/install/setup.bash" ] && source "$HOME/dzpx4/install/setup.bash"
[ -r "$HOME/dzrobot/install/setup.bash" ] && source "$HOME/dzrobot/install/setup.bash"

echo "== 本机标识: $SELF   持续: ${DUR}s   话题: $TOPIC =="
echo "== ROS_DISTRO=${ROS_DISTRO:-unset}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW=${RMW_IMPLEMENTATION:-default} =="
echo "== 本机地址: $(hostname -I 2>/dev/null) =="
: > "$LOG"

SRC=$(echo "$SELF" | tr -cd 'A-Za-z0-9_-')
SELF_TAG="HELLO from ${SRC}"

# 1) 持续发布信标（多跑 8 秒，保证监听端始终有源）
timeout $((DUR + 8)) ros2 topic pub -r 2 "$TOPIC" std_msgs/msg/String "{data: '${SELF_TAG}'}" >/dev/null 2>&1 &
PUB_PID=$!
sleep 2

# 2) 持续监听（行缓冲，保证收尾统计不丢数据）
if command -v stdbuf >/dev/null 2>&1; then
  timeout "$DUR" stdbuf -oL -eL ros2 topic echo "$TOPIC" std_msgs/msg/String >"$LOG" 2>&1
else
  timeout "$DUR" ros2 topic echo "$TOPIC" std_msgs/msg/String >"$LOG" 2>&1
fi
kill $PUB_PID 2>/dev/null
wait $PUB_PID 2>/dev/null
sleep 1

TOTAL=$(grep -c "HELLO from " "$LOG" 2>/dev/null || true)
OWN=$(grep -c "$SELF_TAG" "$LOG" 2>/dev/null || true)
TOTAL=${TOTAL:-0}; OWN=${OWN:-0}
PEER=$((TOTAL - OWN))

echo "== 收到本机信标: ${OWN} 条（本机自发自收正常性检查）"
echo "== 收到对端信标: ${PEER} 条"
if [ "$PEER" -gt 0 ]; then
  echo "== 对端样本:"
  grep "HELLO from " "$LOG" | grep -v "$SELF_TAG" | tail -3 | sed 's/^/   /'
  echo "==> RESULT: LINK_OK  与对端 ROS2/DDS 直连成功"
else
  echo "==> RESULT: LINK_FAIL  未收到对端消息（本机自发自收 ${OWN} 条，说明本机 ROS2 正常）"
fi
echo "== 完整日志: $LOG"
