#!/bin/bash
# 无人机侧发送器包装：自动 source ROS + dzlink.env，再原样转发参数
source /opt/ros/humble/setup.bash
if [ -f "$HOME/dzlink_tools/dzlink.env" ]; then
  source "$HOME/dzlink_tools/dzlink.env"
fi
echo "DOMAIN=[${ROS_DOMAIN_ID:-unset}]"
exec python3 "$HOME/dzlink_tools/dz_send_waypoint.py" "$@"
