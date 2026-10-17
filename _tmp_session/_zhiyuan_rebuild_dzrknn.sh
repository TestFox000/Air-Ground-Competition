#!/bin/bash
source /opt/ros/humble/setup.bash >/dev/null 2>&1
cd /home/duzhong/dzrobot
echo "=== BUILD START $(date +%H:%M:%S) ==="
colcon build --packages-select dzrknn_dataprocess 2>&1 | tail -6
echo "=== BUILD END $(date +%H:%M:%S) ==="
ls -l --time-style=+%m-%d_%H:%M install/dzrknn_dataprocess/lib/dzrknn_dataprocess/dzactuator_node
echo -n "binary_has_pitch_symbol_count="
strings install/dzrknn_dataprocess/lib/dzrknn_dataprocess/dzactuator_node 2>/dev/null | grep -c scan_pitch_offset_deg
