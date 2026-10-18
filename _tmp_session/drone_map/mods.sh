#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
echo '### MODIFICATIONS_HEAD'
sed -n '1,110p' /home/duzhong/dzpx4/src/FAST_LIO_LOCALIZATION_ROS2/MODIFICATIONS.md
echo '### LASERMAPPING_1130_1190'
sed -n '1130,1190p' /home/duzhong/dzpx4/src/FAST_LIO_LOCALIZATION_ROS2/src/laserMapping.cpp
echo '### END'
