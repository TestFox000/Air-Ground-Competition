#!/usr/bin/env bash
# 现场排查工具：**ROS2 的节点日志不在 stdout，而在 ~/.ros/log/**。
#
# 为什么需要这个工具（踩过的坑）：
#   `path_executor.py` 是 Python 节点，它的 INFO 日志走 Humble 的 spdlog 后端，
#   同时写到 console 和 `~/.ros/log/<node>_<pid>_<epoch>.log`。
#   而我们用 SSH 跑它时 console 是管道（非 TTY）→ C 层全缓冲 → 进程被 kill 时
#   缓冲整体丢失 —— 于是"日志像没打一样"。真正的日志一直在 ~/.ros/log/ 里。
#   现场任何"节点没反应"的排查，都应该先来这里看。
#
# 用法：python tools/remote_run.py --host <IP> --user duzhong --script tools/car_ros_logs.sh
#       MINUTES=600 python tools/remote_run.py ... --script tools/car_ros_logs.sh   （看最近 10 小时）

MIN="${MINUTES:-180}"
PAT="${PATTERN:-Odometry reset|ARRIVED|Navigating|Path Executor Node Initialized|stop_signal|Shoot stop|re-anchored|error|Error|Traceback}"

echo "==================== 车端 ROS 日志（最近 ${MIN} 分钟） ===================="
date
echo "日志目录: $HOME/.ros/log"
echo "过滤: $PAT"
echo

echo "## 1. 最近写过的日志文件（按时间倒序，前 15 个）"
find "$HOME/.ros/log" -name "*.log" -mmin "-$MIN" 2>/dev/null | head -15 | while read -r f; do
  printf "  %s  %s  %s 行\n" "$(date -r "$f" '+%m-%d %H:%M:%S')" "$(du -h "$f" | cut -f1)" "$(wc -l < "$f")"
  echo "     $f"
done
echo

echo "## 2. 关键事件（按文件分组）"
find "$HOME/.ros/log" -name "*.log" -mmin "-$MIN" 2>/dev/null | while read -r f; do
  hits=$(grep -cE "$PAT" "$f" 2>/dev/null)
  if [ "${hits:-0}" -gt 0 ]; then
    echo "===== $(basename "$f")  ($hits 条) ====="
    grep -E "$PAT" "$f" | tail -15 | sed 's/^/  /'
    echo
  fi
done

echo "## 3. 全局搜索今天的 reset / ARRIVED（不限 mmin，防止时钟差异）"
grep -h -E "Odometry reset|ARRIVED! dist" "$HOME/.ros/log"/*.log 2>/dev/null | tail -12 | sed 's/^/  /'
echo

echo "==================== 结束 ===================="
