set +e
echo "== autostart_agup.sh: domain/agent lines =="
grep -n -E 'ROS_DOMAIN_ID|MicroXRCE|agent' /home/duzhong/dzuav_deploy/air_ground/tools/autostart_agup.sh 2>&1 | head -40
echo "== start_drone.sh agent lines =="
grep -n -E 'ROS_DOMAIN_ID|MicroXRCE|agent' /home/duzhong/start_drone.sh 2>&1 | head -30
echo "== watchdog section =="
sed -n '1,80p' /home/duzhong/dzuav_deploy/air_ground/tools/autostart_agup.sh 2>&1
echo "== fmu data on domain 0 =="
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=0 timeout 8 ros2 topic hz /fmu/out/vehicle_local_position 2>&1 | head -6
ROS_DOMAIN_ID=0 timeout 8 ros2 topic echo --once /fmu/out/vehicle_local_position 2>&1 | head -25
echo "== node list domain 0 =="
ROS_DOMAIN_ID=0 timeout 8 ros2 node list 2>&1 | head
