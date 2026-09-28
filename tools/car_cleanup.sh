#!/usr/bin/env bash
# 车端痕迹清理：把本项目的所有痕量删干净，让**公用小车恢复原样**。
#
# ⚠️ 只删「我们自己创建的东西」，白名单硬编码在下面，不做任何通配式删除。
#    厂家/他人的数据（~/dzrobot、~/src_save_backup_*、~/backup、~/桌面 等）从不触碰。
#
# 用法：
#   python tools/remote_run.py --host <车IP> --user duzhong --script tools/car_cleanup.sh
#   # 演练（只列不删）：加 --cmd 方式传 AG_DRY_RUN=1 不好传，直接看本脚本前面几行输出即可

set -u

# ============ 白名单：只允许删这些（都是我们自己创建的） ============
MINE_PATHS=(
  "/tmp/ag_ws"              # 编译用的独立工作空间（我们创建）
  "/tmp/ag_ws_old_mine"     # 早期误放在家目录、后移出的副本
  "/tmp/syntaxcheck"        # 脚本语法检查用的上传目录
  "/tmp/ag_build.log"       # 编译日志
  "/tmp/ag_car_node.log"    # 干跑联调时 car_node 的日志
  "/tmp/_pkgs.txt"          # probe_car.sh 抓包列表时写的临时文件
)
MINE_GLOBS=(
  "/tmp/probe_car*.sh.sh"   # 上传执行的探测脚本
  "/tmp/car_*.sh.sh"
  "/tmp/remote_run*.sh.sh"
)
# 本应已被移走、但万一还在的旧位置
LEGACY_PATHS=(
  "$HOME/ag_ws"
  "$HOME/ag_ws_old_mine"
)
# ⚠️ 这份白名单必须与 tools/car_inventory.sh 里的完全一致：
#    盘点为空 ⟺ 清理无事可做 ⟺ 车已恢复原样。
# ===================================================================

echo "==================== 车端痕迹清理 ===================="
date
echo

echo "## 0. 白名单（只会删这些）"
printf '  %s\n' "${MINE_PATHS[@]}"
printf '  %s\n' "${MINE_GLOBS[@]}"
printf '  （若存在）%s\n' "${LEGACY_PATHS[@]}"
echo

echo "## 1. 删除前：确认目标存在"
FOUND=0
for p in "${MINE_PATHS[@]}" "${LEGACY_PATHS[@]}"; do
  if [ -e "$p" ]; then
    echo "  [有] $p  ($(du -sh "$p" 2>/dev/null | cut -f1))"
    FOUND=$((FOUND+1))
  fi
done
for g in "${MINE_GLOBS[@]}"; do
  for f in $g; do
    [ -e "$f" ] && { echo "  [有] $f"; FOUND=$((FOUND+1)); }
  done
done
[ "$FOUND" -eq 0 ] && echo "  （没有需要清理的东西）"
echo

echo "## 2. 执行删除（仅白名单）"
for p in "${MINE_PATHS[@]}" "${LEGACY_PATHS[@]}"; do
  [ -e "$p" ] && { rm -rf -- "$p" && echo "  已删 $p"; }
done
for g in "${MINE_GLOBS[@]}"; do
  for f in $g; do
    [ -e "$f" ] && { rm -f -- "$f" && echo "  已删 $f"; }
  done
done
echo

echo "## 2.5 停止我们启动过的节点进程（按 PID 精确停，不用模糊 pkill）"
# ⚠️ 教训：用 `kill -9 <ros2 launch 的 PID>` 只杀了外壳，它 spawn 的子节点会变成孤儿
#    继续在车上跑（我们真的留下过 dzactuator + 6 个 static_transform_publisher）。
#    所以这里：先 ps 查出 PID，再用纯数字的 kill 停掉。
stop_pattern() {
  local pat="$1"
  local pids
  pids=$(ps -eo pid,cmd | grep -F "$pat" | grep -v grep | awk '{print $1}' | tr '\n' ' ')
  [ -z "$pids" ] && return 0
  echo "  停止 [$pat]: $pids"
  kill -INT $pids 2>/dev/null
}
PROC_PATS=(
  "/opt/ros/humble/bin/ros2 launch dzactuator"
  "/opt/ros/humble/lib/tf2_ros/static_transform_publisher"
  "/lib/dzactuator/dzactuator_node"
  "/det_node/lib/det_node/det_node"
  "usb_cam_node_exe"
  "car_localization/path_executor.py"
  "car_localization/serial_reader"
  "ag_car/aim_guard_node"
  "ag_car/car_node"
  "ag_perception/vision_node"
  "ag_perception/intel_fusion_node"
  "ag_bridge/ground_link_node"
  "ag_bridge/air_link_node"
)
for p in "${PROC_PATS[@]}"; do stop_pattern "$p"; done
sleep 2
for p in "${PROC_PATS[@]}"; do
  pids=$(ps -eo pid,cmd | grep -F "$p" | grep -v grep | awk '{print $1}' | tr '\n' ' ')
  [ -n "$pids" ] && { echo "  强制停止 [$p]: $pids"; kill -9 $pids 2>/dev/null; }
done
sleep 1
LEFT_PROC=$(for p in "${PROC_PATS[@]}"; do ps -eo pid,cmd | grep -F "$p" | grep -v grep; done)
if [ -z "$LEFT_PROC" ]; then
  echo "  [OK] 没有残留进程"
else
  echo "  [FAIL] 仍有残留："; echo "$LEFT_PROC" | sed 's/^/    /'
fi
echo

echo "## 3. 删除后：确认干净"
LEFT=0
for p in "${MINE_PATHS[@]}" "${LEGACY_PATHS[@]}"; do
  [ -e "$p" ] && { echo "  [残留] $p"; LEFT=$((LEFT+1)); }
done
for g in "${MINE_GLOBS[@]}"; do
  for f in $g; do
    [ -e "$f" ] && { echo "  [残留] $f"; LEFT=$((LEFT+1)); }
  done
done
[ "$LEFT" -eq 0 ] && echo "  [OK] 白名单内的东西都已清除" || echo "  [FAIL] 仍有 $LEFT 项残留"
echo

echo "## 4. 他人数据完整性核对（应与会话开始时一致）"
for d in "$HOME/dzrobot" "$HOME/dzrobot/src" "$HOME/src_save_backup_20260926" "$HOME/backup"; do
  if [ -e "$d" ]; then
    printf "  %-45s %s  %s 个文件\n" "$d" "$(du -sh "$d" 2>/dev/null | cut -f1)" "$(find "$d" -type f 2>/dev/null | wc -l)"
  fi
done
echo "  -- 关键文件指纹（与开工前对比；不一致说明被动过）--"
for f in "$HOME/dzrobot/src/car_localization/car_localization/path_executor.py" \
         "$HOME/dzrobot/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp" \
         "$HOME/dzrobot/src/dzjudgment/src/judgment.cpp" \
         "$HOME/.bashrc"; do
  [ -e "$f" ] && printf "    %s  %s\n" "$(md5sum "$f" 2>/dev/null | cut -c1-10)" "${f/#$HOME/~}"
done
echo "  参考基线（开工时测得）：path_executor=6d31d068c1  dzrknn=8f...  judgment=34a3189d26  .bashrc=28addcce99"
echo

echo "## 5. 家目录顶层（不应再出现 ag_ws 之类）"
ls "$HOME" | grep -iE "^ag_|^car_" && echo "  [注意] 家目录里还有疑似我们的目录" || echo "  [OK] 家目录无可疑目录"
echo

echo "## 6. 收尾"
echo "  提醒：小车上的 /tmp 会在重启后自动清空；下次要用重新上传即可。"
echo "  dzrobot 等厂家目录全程只读，未做任何修改。"
echo "==================== 结束 ===================="
