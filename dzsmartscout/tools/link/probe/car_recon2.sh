#!/bin/bash
# Read-only: how start_all.sh handles 4_det, and where run logs live.
echo "=== A. start_all.sh 第 45-75 行（4_det 特殊处理）==="
sed -n '45,75p' "$HOME/dzrobot/start_all.sh"
echo
echo "=== B. det_node 现在能不能解析 ==="
. /opt/ros/humble/setup.bash
. "$HOME/dzrobot/install/setup.bash" 2>&1 | head -3
ros2 pkg prefix det_node 2>&1 | head -3
echo "--- 顺便看另外两个 ---"
ros2 pkg prefix fake_amcl_node 2>&1 | head -2
ros2 pkg prefix openslam_gmapping 2>&1 | head -2
echo
echo "=== C. 运行日志目录 ==="
ls -lt "$HOME/robot_logs/" 2>/dev/null | head -12
echo
echo "=== D. 上次 7_path 日志尾部 ==="
tail -5 "$HOME/robot_logs/7_path.log" 2>/dev/null || echo "(没有 7_path.log)"
