#!/usr/bin/env bash
# 把官方的瞄准节点 `dzrknn_dataprocess` 编译到**我们自己的 /tmp 工作空间**。
#
# 背景（车端实测）：`~/dzrobot/install` 的 60 个包里**没有 dzrknn_dataprocess** ——
# 只有源码、没编译过。于是：
#   * `ros2 launch dzactuator bringup_all.launch.py` 在当前车上会缺一个节点
#   * 云台扫描 / 瞄准 / 开火这条链根本起不来
#
# 所以在这里单独把它编出来。做法：
#   从 ~/dzrobot/src **只读复制**源码到 /tmp/ag_ws/src，在那里 colcon build。
#   **完全不写厂家的 build/install/log，车保持原样。**
#
# 编完之后就可以这样启动（配合我们的开火闸门）：
#   source /tmp/ag_ws/install/setup.bash
#   ros2 run dzrknn_dataprocess dzactuator_node --ros-args -r offset_center:=/offset_center_guarded

set -o pipefail
WS="${AG_WS:-/tmp/ag_ws}"
SRC="$HOME/dzrobot/src/dzrknn_dataprocess"
LOG="/tmp/ag_gimbal_build.log"

echo "==================== 编译官方瞄准节点（到 /tmp，不动厂家目录） ===================="
date
echo

echo "## 1. 复制源码（只读源 -> /tmp）"
if [ ! -d "$SRC" ]; then echo "!! 找不到 $SRC"; exit 1; fi
rm -rf "$WS/src/dzrknn_dataprocess"
mkdir -p "$WS/src"
cp -r "$SRC" "$WS/src/dzrknn_dataprocess"
rm -rf "$WS/src/dzrknn_dataprocess/build" "$WS/src/dzrknn_dataprocess/install" \
       "$WS/src/dzrknn_dataprocess/log"
echo "  已复制到 $WS/src/dzrknn_dataprocess"
ls "$WS/src/dzrknn_dataprocess" | sed 's/^/    /'
echo

echo "## 2. colcon build --packages-select dzrknn_dataprocess"
source /opt/ros/humble/setup.bash 2>/dev/null
cd "$WS" || exit 1
colcon build --packages-select dzrknn_dataprocess --symlink-install > "$LOG" 2>&1
RC=$?
echo "  退出码 = $RC"
echo "-- 摘要 --"
grep -E "Starting|Finished|Failed|Summary|error|Error" "$LOG" | tail -20 | sed 's/^/  /'
echo

echo "## 3. 结果"
if [ -x "$WS/install/dzrknn_dataprocess/lib/dzrknn_dataprocess/dzactuator_node" ]; then
  echo "  [OK] 可执行文件已生成："
  ls -l "$WS/install/dzrknn_dataprocess/lib/dzrknn_dataprocess/" | sed 's/^/    /'
else
  echo "  [FAIL] 没生成可执行文件，日志尾部："
  tail -30 "$LOG" | sed 's/^/    /'
fi
echo

echo "## 4. 厂家的目录有没有被碰过（应该完全没变）"
echo "  ~/dzrobot/install 包数: $(ls "$HOME/dzrobot/install" | wc -l)（开工前也是这个数）"
ls -d "$HOME/dzrobot/install/dzrknn_dataprocess" 2>/dev/null && \
  echo "  ⚠️ 厂家目录里出现了 dzrknn_dataprocess（不该发生）" || echo "  [OK] 厂家 install 里仍然没有它（我们没写进去）"
echo "  完整日志: $LOG"
echo "==================== 结束 ===================="
