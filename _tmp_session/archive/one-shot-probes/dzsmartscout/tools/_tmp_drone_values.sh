set +e
source /opt/ros/humble/setup.bash
source ~/dzlink_tools/dzlink.env
echo "=== /nav/manual_override VALUE ==="
timeout 12 ros2 topic echo /nav/manual_override --once 2>&1 | head -5
echo "=== /nav/manual_override -v ==="
timeout 20 ros2 topic info /nav/manual_override -v 2>&1 | head -22
echo
echo "=== /nav/motion_state VALUE (best_effort) ==="
timeout 12 ros2 topic echo /nav/motion_state --once --qos-reliability best_effort 2>&1 | head -5
echo
echo "=== /pursuitAngle VALUE (best_effort) ==="
timeout 12 ros2 topic echo /pursuitAngle --once --qos-reliability best_effort 2>&1 | head -12
echo
echo "=== /pursuitAngle -v (who publishes) ==="
timeout 25 ros2 topic info /pursuitAngle -v 2>&1 | grep -E "Publisher count|Subscription count|Node name|Reliability" | head -24
echo
echo "=== /aim/status VALUE ==="
timeout 12 ros2 topic echo /aim/status --once 2>&1 | head -5
echo
echo "=== /move_base/stop_signal VALUE ==="
timeout 12 ros2 topic echo /move_base/stop_signal --once 2>&1 | head -5
echo "=== DONE ==="
