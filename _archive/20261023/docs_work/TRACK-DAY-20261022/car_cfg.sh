#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/duzhong/dzrobot/install/setup.bash
F=/home/duzhong/r3_field_20261021/config/robot_r3.yaml
echo ===YAW-SPIN===
grep -n -i 'yaw_\|spin\|pivot\|turn_' "$F" | head -40
echo ===AIM===
grep -n -A2 '^aim:' "$F" | head -40
grep -n 'lead\|fire_box\|fire_dz\|fire_only\|lost_' "$F" | head -30
echo ===MISSION-SHOOT===
grep -n 'shoot_timeout\|auto_start\|scan_mode' "$F" | head -20
echo ===INSTALLED-LOC===
python3 -c "import car_localization.serial_reader as m; print('MOD:',m.__file__); print('START:',m.START_X,m.START_Y,'VER:',m.NODE_VERSION)"
