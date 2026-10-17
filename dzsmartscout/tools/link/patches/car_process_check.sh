#!/bin/bash
# ==============================================================================
#  只读：确认车上有没有残留节点在跑
#
#  ★ 为什么按【可执行名 comm】过滤而不是命令行 args：
#    用 `pgrep -f 'ros2 launch'` 时，ssh 会话自己的 `bash -c "...ros2 launch..."`
#    会被匹配到，永远返回 1 —— 本轮实测踩过，虚惊一场。
#    comm 只有可执行文件名（bash/grep/ps 都叫自己的名字），不会自匹配。
# ==============================================================================
set -u
echo "=== 可疑节点进程（comm 过滤）==="
ps -eo pid,etimes,comm --no-headers \
  | grep -E 'ros2|python3|det_node|usb_cam|static_transf|chrome|rviz' || echo "(无)"
echo
echo "=== 受管 PID 文件数（start_all.sh 的 stop 会清空）==="
ls -1 "$HOME/robot_logs/pids/" 2>/dev/null | wc -l
echo
echo "=== 8080 网页台是否在听 ==="
ss -lntp 2>/dev/null | grep ':8080' || echo "(没有监听 8080)"
echo
echo "=== robot_logs 最近 6 项 ==="
ls -lt --time-style=+%Y-%m-%d_%H:%M:%S "$HOME/robot_logs/" 2>/dev/null | head -6
