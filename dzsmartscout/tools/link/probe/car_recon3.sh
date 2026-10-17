#!/bin/bash
# Read-only, CORRECT env sourcing (no pipe around source).
echo "=== A. 正确加载工作区 ==="
. /opt/ros/humble/setup.bash
. "$HOME/dzrobot/install/setup.bash"
echo "AMENT_PREFIX_PATH 条目数: $(echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | wc -l)"
echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | head -12
echo
echo "=== B. 关键包能否解析 ==="
for p in det_node fake_amcl_node car_localization openslam_gmapping dzactuator dzjudgment dzrknn_dataprocess; do
  printf "%-24s %s\n" "$p" "$(ros2 pkg prefix "$p" 2>&1 | head -1)"
done
echo
echo "=== C. localization.launch.py 起了哪些节点 ==="
L=$(find "$HOME/dzrobot/install/car_localization" -name 'localization.launch.py' 2>/dev/null | head -1)
echo "文件: $L"
grep -n -E "Node\(|package=|executable=|name=" "$L" 2>/dev/null | head -30
echo
echo "=== D. 谁发布 /odom/localization ==="
grep -rn "odom/localization" "$HOME/dzrobot/install/car_localization/lib/python3.10/site-packages/car_localization/" 2>/dev/null | head -10
echo
echo "=== E. path_executor 订阅了什么 ==="
grep -n -E "create_subscription|nav/waypoint" "$HOME/dzrobot/install/car_localization/lib/python3.10/site-packages/car_localization/path_executor.py" 2>/dev/null | head -15
