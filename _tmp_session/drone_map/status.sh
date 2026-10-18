#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/duzhong/dzpx4/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
echo '### VEHICLE_STATUS'
timeout 8 ros2 topic echo /fmu/out/vehicle_status --qos-reliability best_effort --once 2>&1 | grep -E 'arming_state|nav_state|failsafe|pre_flight' | head -10
echo '### LOCAL_POS'
timeout 8 ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>&1 | grep -E '^x:|^y:|^z:|xy_valid|z_valid' | head -8
echo '### GLOBAL_MAP_TOPIC'
timeout 8 ros2 topic info /global_map 2>&1 | head -4
echo '### GLOBAL_LOC_NODE'
timeout 8 ros2 node info /global_localization 2>&1 | sed -n '1,36p'
echo '### LOCALIZATION_SAMPLE'
timeout 8 ros2 topic echo /localization --once 2>&1 | grep -E 'position:|x:|y:|z:' | head -9
echo '### RECORDER'
pgrep -af 'dzmap/rec.py' || echo 'not running'
echo '### END'
