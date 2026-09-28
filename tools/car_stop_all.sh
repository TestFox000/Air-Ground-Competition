#!/usr/bin/env bash
# 【紧急停止】把车上"能让车动 / 能开火"的节点全部停掉。
#
# 用途：车跑飞了、别人还在用、或者你要走了但不确定有没有残留 —— 先跑这个。
# 只停**明确的进程名**（不用模糊通配），不会碰别人的程序。
#
# 用法：python tools/remote_run.py --host <IP> --user duzhong --script tools/car_stop_all.sh

# 注意：`ros2 launch` 是壳，子节点在它的进程组里；这里按**可执行文件路径**逐个停，
# 所以壳和子进程都会命中（比 kill 壳更可靠）。
PROC_PATS=(
  "/opt/ros/humble/bin/ros2 launch dzactuator"
  "/opt/ros/humble/lib/tf2_ros/static_transform_publisher"
  "/lib/dzactuator/dzactuator_node"
  "det_node/lib/det_node/det_node"
  "usb_cam_node_exe"
  "car_localization/path_executor.py"
  "car_localization/serial_reader"
  "dzrknn_dataprocess"
  "ag_car/car_node"
  "ag_car/aim_guard_node"
  "ag_perception/vision_node"
  "ag_perception/intel_fusion_node"
  "ag_bridge/ground_link_node"
  "ag_bridge/air_link_node"
  "ag_bridge/fake_intel_node"
)

echo "==================== 紧急停止：车上所有能动的节点 ===================="
date
echo

echo "## 0. 现在有哪些相关进程"
for p in "${PROC_PATS[@]}"; do
  ps -eo pid,cmd | grep -F "$p" | grep -v grep | sed 's/^/  /'
done
echo "  （以上为空则本来就没在跑）"
echo

echo "## 1. 发 SIGINT（让节点自己正常退出，它会写日志/落盘）"
stopped=0
for p in "${PROC_PATS[@]}"; do
  pids=$(ps -eo pid,cmd | grep -F "$p" | grep -v grep | awk '{print $1}' | tr '\n' ' ')
  if [ -n "$pids" ]; then
    echo "  停止 [$p]: $pids"
    kill -INT $pids 2>/dev/null
    stopped=$((stopped+1))
  fi
done
[ "$stopped" -eq 0 ] && echo "  （没有需要停的）"
sleep 3
echo

echo "## 2. 仍在跑的强制 SIGKILL"
for p in "${PROC_PATS[@]}"; do
  pids=$(ps -eo pid,cmd | grep -F "$p" | grep -v grep | awk '{print $1}' | tr '\n' ' ')
  [ -n "$pids" ] && { echo "  强制停止 [$p]: $pids"; kill -9 $pids 2>/dev/null; }
done
sleep 1
echo

echo "## 3. 复核"
LEFT="$(for p in "${PROC_PATS[@]}"; do ps -eo pid,cmd | grep -F "$p" | grep -v grep; done)"
if [ -z "$LEFT" ]; then
  echo "  [OK] 没有残留进程"
else
  echo "  [FAIL] 仍有残留："; echo "$LEFT" | sed 's/^/    /'
fi
echo
echo "## 4. 位置（确认车停住了）"
source /opt/ros/humble/setup.bash 2>/dev/null
timeout 10 ros2 topic echo --once /odom/localization --field pose.pose.position 2>/dev/null \
  | sed 's/^/  /' || echo "  （定位话题已停/无数据）"
echo "==================== 结束 ===================="
