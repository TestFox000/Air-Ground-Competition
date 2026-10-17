#!/bin/bash
# t4_topicinfo.sh —— 在某台机器上取 ROS 图 + 指定话题的端点信息（只读）
# 用法: bash t4_rosenv.sh bash /tmp/t4_topicinfo.sh /dzlink/drone_to_car /nav/waypoint
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -r "$HOME/dzrobot/install/setup.bash" ] && source "$HOME/dzrobot/install/setup.bash"
[ -r "$HOME/dzrobot/dzros.env" ] && . "$HOME/dzrobot/dzros.env"
[ -r "$HOME/dzlink_tools/dzlink.env" ] && . "$HOME/dzlink_tools/dzlink.env"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-118}"

echo "### host=$(hostname) ip=$(hostname -I | awk '{print $1}') domain=${ROS_DOMAIN_ID} time=$(date -Is)"
echo "### node list"
timeout 12 ros2 node list 2>&1 | sed 's/^/  /'
echo "### topic list"
timeout 12 ros2 topic list 2>&1 | sed 's/^/  /'
for t in "$@"; do
  echo "### topic info -v $t"
  timeout 15 ros2 topic info -v "$t" 2>&1 | sed 's/^/  /'
done
echo "### t4_topicinfo done"
