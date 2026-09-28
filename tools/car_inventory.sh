#!/usr/bin/env bash
# 车端「我留下了什么」盘点（只读，不删任何东西）。
#
# 用途：这台车是**公用**的，别人在上面有数据。我们做任何事之前先盘点、
# 结束之前必须对照本脚本的输出把痕迹清干（见 tools/car_cleanup.sh）。
#
# 只读，不会修改任何文件。

hr() { echo "----------------------------------------------------"; }

echo "==================== 车端痕迹盘点（只读） ===================="
date
echo "主机: $(hostname)   运行时长: $(uptime -p 2>/dev/null)"
echo

# ⚠️ 下面这份白名单必须与 tools/car_cleanup.sh 完全一致：
#    盘点列表为空 ⟺ 清理脚本无事可做 ⟺ 车已恢复原样。
MINE_PATHS=(
  "/tmp/ag_ws"
  "/tmp/ag_ws_old_mine"
  "/tmp/syntaxcheck"
  "/tmp/ag_build.log"
  "/tmp/ag_car_node.log"
  "/tmp/_pkgs.txt"
)
MINE_GLOBS=(
  "/tmp/probe_car*.sh.sh"
  "/tmp/car_*.sh.sh"
  "/tmp/remote_run*.sh.sh"
)
LEGACY_PATHS=(
  "$HOME/ag_ws"
  "$HOME/ag_ws_old_mine"
)

hr; echo "## 1. 我创建的东西（可安全删除；白名单同 car_cleanup.sh）"
TOTAL=0
for p in "${MINE_PATHS[@]}" "${LEGACY_PATHS[@]}"; do
  if [ -e "$p" ]; then
    echo "  [有] $p  ($(du -sh "$p" 2>/dev/null | cut -f1), $(find "$p" -type f 2>/dev/null | wc -l) 个文件)"
    TOTAL=$((TOTAL+1))
  fi
done
for g in "${MINE_GLOBS[@]}"; do
  for f in $g; do
    [ -e "$f" ] && { echo "  [有] $f"; TOTAL=$((TOTAL+1)); }
  done
done
if [ "$TOTAL" -eq 0 ]; then
  echo "  [OK] 白名单内没有任何残留 —— 车已是原样"
else
  echo "  共 $TOTAL 项；收工时跑 tools/car_cleanup.sh 清掉"
fi
echo

hr; echo "## 2.5 残留进程（应该没有；有的话说明有节点没被正确停掉）"
# ⚠️ 教训：ros2 launch 被 kill 后，它 spawn 的子节点会变孤儿继续跑。
#    所以盘点必须包含进程，不能只看文件。
PROC_PATS=(
  "ros2 launch dzactuator"
  "static_transform_publisher"
  "dzactuator_node"
  "/det_node"
  "usb_cam_node_exe"
  "path_executor.py"
  "serial_reader"
  "aim_guard_node"
  "car_node"
  "vision_node"
  "ground_link_node"
  "air_link_node"
)
FOUND_PROC=0
for p in "${PROC_PATS[@]}"; do
  lines=$(ps -eo pid,lstart,cmd | grep -F "$p" | grep -v grep)
  if [ -n "$lines" ]; then
    echo "  [有] $p"
    echo "$lines" | sed 's/^/       /'
    FOUND_PROC=$((FOUND_PROC+1))
  fi
done
[ "$FOUND_PROC" -eq 0 ] && echo "  [OK] 没有任何本项目/官方节点在跑"
echo

hr; echo "## 3. 厂家/他人数据 —— 只允许读，未经批准不动"
for d in "$HOME/dzrobot" "$HOME/dzrobot/src" "$HOME/src_save_backup_20260926" "$HOME/backup"; do
  if [ -e "$d" ]; then
    printf "%-45s %s  %s 个文件\n" "$d" "$(du -sh "$d" 2>/dev/null | cut -f1)" "$(find "$d" -type f 2>/dev/null | wc -l)"
  fi
done
echo "-- 关键文件指纹（用于事后核对没被动过）--"
for f in "$HOME/dzrobot/src/car_localization/car_localization/path_executor.py" \
         "$HOME/dzrobot/src/rknn_pt/rknn_pt/src/det/postprocess.cc" \
         "$HOME/dzrobot/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp" \
         "$HOME/dzrobot/src/dzjudgment/src/judgment.cpp" \
         "$HOME/.bashrc"; do
  [ -e "$f" ] && printf "%s  %s\n" "$(md5sum "$f" 2>/dev/null | cut -c1-10)" "${f/#$HOME/~}"
done
echo

hr; echo "## 4. 家目录顶层（看有没有我多出来的东西）"
ls -la "$HOME" | grep -vE "^total|^\." | awk '{print "  " $6, $7, $8, $9}'
echo

hr; echo "## 5. 磁盘"
df -h / | tail -1
echo

echo "==================== 结束 ===================="
