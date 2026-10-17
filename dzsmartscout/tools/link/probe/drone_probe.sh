#!/bin/bash
# Drone-side probe wrapper. Domain comes from DZ_DOMAIN (default 118).
export ROS_DOMAIN_ID="${DZ_DOMAIN:-118}"
. /opt/ros/humble/setup.bash
exec python3 /tmp/dz_drone_send.py "$@"
