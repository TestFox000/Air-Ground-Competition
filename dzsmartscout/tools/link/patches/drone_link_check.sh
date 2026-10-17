#!/bin/bash
# ==============================================================================
#  无人机侧：确认能看到小车（域号取 dzlink.env）
#  判据：/nav/waypoint 的 Subscription count 必须是 1（车侧 path_executor 的订阅）
#  用法：bash drone_link_check.sh
# ==============================================================================
set -u
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
if [ -f "$HOME/dzlink_tools/dzlink.env" ]; then
  # shellcheck disable=SC1091
  . "$HOME/dzlink_tools/dzlink.env"
fi
echo "=== 本机域号 ==="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-未设}  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-未设}"
echo
echo "=== /nav/waypoint 端点（判据 Subscription count: 1）==="
timeout 25 ros2 topic info /nav/waypoint -v 2>&1 | head -32
echo
echo "=== 无人机看到的节点 ==="
timeout 25 ros2 node list 2>&1 | head -20
echo
echo "=== 小车回传话题（旧核不发布，仅确认话题是否存在）==="
timeout 20 ros2 topic list 2>&1 | grep -i 'dzlink\|nav/\|Laser' | head -10
