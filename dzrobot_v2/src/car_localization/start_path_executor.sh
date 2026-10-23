#!/bin/bash
#
# Start path executor with a simple waypoint command
#
# Usage:
#   ./start_path_executor.sh [waypoint]
#   ./start_path_executor.sh "M10,y"
#

WAYPOINT="${1:-M10,y}"

echo "=========================================="
echo "Starting Path Executor"
echo "Waypoint: $WAYPOINT"
echo "=========================================="

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ~/dzrobot/install/setup.bash

# Generate test path first
echo "Generating test path..."
python3 ~/dzrobot/src/car_localization/car_localization/generate_path.py /tmp/test_path.txt

# Start path executor in background
echo "Starting path executor node..."
python3 ~/dzrobot/src/car_localization/car_localization/path_executor.py &
EXECUTOR_PID=$!

# Wait for node to start
sleep 3

# Send waypoint command
echo "Sending waypoint command: $WAYPOINT"
ros2 topic pub /nav/waypoint std_msgs/msg/String "{data: '$WAYPOINT'}" --once

echo ""
echo "Path executor is running (PID: $EXECUTOR_PID)"
echo "Press Ctrl+C to stop"

# Wait for the executor process
wait $EXECUTOR_PID