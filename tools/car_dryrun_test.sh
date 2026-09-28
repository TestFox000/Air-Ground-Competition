#!/usr/bin/env bash
# 车端「干跑」联调：真的把 car_node 跑起来（dry_run=true），喂一帧假情报，
# 看它算出什么路线。**不会让车动**：
#   * car_node 在 dry_run 下只发 /car/route_status，绝不发 /nav/waypoint
#   * 不启动任何渡众节点，不碰串口，不碰相机
#   * 结束时只 kill 我们自己起的那个 PID（不会误杀别人的节点）
#
# 用法：python tools/remote_run.py --host <IP> --user duzhong --script tools/car_dryrun_test.sh

WS="${AG_WS:-/tmp/ag_ws}"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null

hr() { echo "----------------------------------------------------"; }

cleanup() {
  if [ -n "${NODE_PID:-}" ] && kill -0 "$NODE_PID" 2>/dev/null; then
    kill -INT "$NODE_PID" 2>/dev/null
    sleep 1
    kill -0 "$NODE_PID" 2>/dev/null && kill -9 "$NODE_PID" 2>/dev/null
    echo "  已停止 car_node (pid $NODE_PID)"
  fi
}
trap cleanup EXIT

echo "==================== 车端干跑联调（不动车） ===================="
date
echo

hr; echo "## 0. 前置检查：确认车里没有别人在跑关键节点（避免互相干扰）"
timeout 10 ros2 node list 2>/dev/null | sed 's/^/  /' || true
echo "  （以上为当前节点；本项目只启动 car_node，dry_run=true）"
echo

hr; echo "## 1. 启动 car_node（dry_run=true，只算不发）"
timeout 300 ros2 run ag_car car_node --ros-args \
  -p dry_run:=true \
  -p min_votes:=2 \
  -p min_mines:=1 \
  -p require_target_face:=true \
  -p start_cell:=N10 \
  -p goal_cell:=N11 > /tmp/ag_car_node.log 2>&1 &
NODE_PID=$!
echo "  pid=$NODE_PID，日志 /tmp/ag_car_node.log"
sleep 5
if ! kill -0 "$NODE_PID" 2>/dev/null; then
  echo "  [FAIL] 节点启动即退出，日志："
  tail -20 /tmp/ag_car_node.log | sed 's/^/    /'
  exit 1
fi
echo "  [OK] 节点在跑"
echo

hr; echo "## 2. 喂两帧情报（min_votes=2，同内容、不同 seq）"
PUB() {
  timeout 20 ros2 topic pub --once /link/field_intel ag_interfaces/msg/FieldIntelMsg \
    "{mines: ['C3', 'G8', 'B2'], obstacles: ['E5'], bridge_a: 'K4', bridge_b: 'K9',
      target_cells: ['D2', 'E2'], target_face: 'blue', seq: $1}" >/dev/null 2>&1
}
PUB 1; sleep 1; PUB 2; sleep 3
echo "  已发送 seq=1,2（蓝面=敌方）"
echo

hr; echo "## 3. 抓 car_node 算出的路线（/car/route_status）"
echo "-- 完整状态 JSON --"
timeout 15 ros2 topic echo --once /car/route_status --field data 2>/dev/null | sed 's/^/  /'
echo

hr; echo "## 4. 反证：dry_run 下**一条路线命令都没发**"
echo "-- 监听 /nav/waypoint 8 秒（dry_run 应收到 0 条）--"
got=$(timeout 8 ros2 topic echo --once /nav/waypoint --field data 2>/dev/null)
if [ -z "$got" ]; then
  echo "  [OK] 8 秒内没有收到任何 /nav/waypoint 消息 —— 车不会动"
else
  echo "  [FAIL] 竟然收到了: $got"
fi
echo "  --（注：/nav/waypoint 的 Publisher count=1 是节点创建了发布器，不代表发过消息）--"
echo

hr; echo "## 5. 节点日志（dry_run 提示与路线）"
grep -E "car_node 就绪|路线|dry_run|敌我" /tmp/ag_car_node.log | head -10 | sed 's/^/  /'
echo

echo "==================== 结束（节点将自动停止） ===================="
