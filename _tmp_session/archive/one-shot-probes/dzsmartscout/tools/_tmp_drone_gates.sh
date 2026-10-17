set +e
source /opt/ros/humble/setup.bash
source ~/dzlink_tools/dzlink.env
echo "=== INFO ==="
for t in /navi_cmd /cmd_vel /move_base/stop_signal /StopCmd /logicontrol /current_vel_state /nav/motion_state /vehicle_Drivestate /nav/scan_request /nav/manual_override /CameMo_Conl; do
  echo "--- $t"
  timeout 12 ros2 topic info "$t" 2>&1 | head -4
done
echo
echo "=== HZ /navi_cmd ==="
timeout 10 ros2 topic hz /navi_cmd 2>&1 | head -3
echo "=== HZ /cmd_vel ==="
timeout 10 ros2 topic hz /cmd_vel 2>&1 | head -3
echo "=== HZ /pursuitAngle ==="
timeout 10 ros2 topic hz /pursuitAngle 2>&1 | head -3
echo
echo "=== ECHO /nav/motion_state ==="
timeout 12 ros2 topic echo /nav/motion_state --once 2>&1 | head -6
echo "=== ECHO /current_vel_state ==="
timeout 12 ros2 topic echo /current_vel_state --once 2>&1 | head -10
echo "=== ECHO /vehicle_Drivestate ==="
timeout 12 ros2 topic echo /vehicle_Drivestate --once 2>&1 | head -8
echo "=== ECHO /StopCmd ==="
timeout 12 ros2 topic echo /StopCmd --once 2>&1 | head -6
echo "=== DONE ==="
