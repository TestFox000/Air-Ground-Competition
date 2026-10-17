set +e
source /opt/ros/humble/setup.bash
source ~/dzlink_tools/dzlink.env
echo "D=[$ROS_DOMAIN_ID]"
echo "=== TOPICS (car side, cross-machine) ==="
timeout 25 ros2 topic list 2>/dev/null | sort
echo
echo "=== INFO ==="
for t in /cmd_vel /nav/cmd_vel /motion_state /nav/motion_state /pursuitAngle /odom /nav/waypoint; do
  echo "--- $t"
  timeout 12 ros2 topic info "$t" 2>&1 | head -5
done
echo "=== DONE ==="
