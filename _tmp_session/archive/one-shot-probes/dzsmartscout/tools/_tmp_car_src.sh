set +e
cd ~/dzrobot/src/car_localization/car_localization 2>/dev/null || { echo "NO SRC DIR"; exit 0; }
echo "=== FILE ==="
ls -l path_executor.py
echo "=== PUBLISHERS ==="
grep -n "create_publisher" path_executor.py | head -30
echo "=== DRIVE TOPIC NAMES ==="
grep -n "cmd_vel\|navi_cmd\|/nav/waypoint\|StopCmd\|stop_signal\|motion_state" path_executor.py | head -40
echo "=== MISSIONS / GATES (keywords) ==="
grep -n "def follow_route_command\|def _start_mission_scan\|def mission\|def _mission\|while \|wait_for\|shoot_wait\|timeout" path_executor.py | head -40
echo "=== DONE ==="
