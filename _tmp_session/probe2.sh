set +e
echo "== agent proc env =="
tr '\0' '\n' < /proc/3193/environ 2>&1 | grep -E 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|FASTRTPS|ROS_VERSION|PWD=|LD_LIBRARY_PATH=' | head -20
echo "== agent cmdline =="
tr '\0' ' ' < /proc/3193/cmdline; echo
echo "== agent start time =="
ps -o pid,lstart,etime,cmd -p 3193 2>&1
echo "== drone_logs =="
ls -lt /home/duzhong/drone_logs/ 2>&1 | head -15
echo "== agent log tail =="
tail -60 /home/duzhong/drone_logs/agent_watchdog_agent.log 2>&1
echo "== watchdog script? =="
grep -rl 'agent_watchdog' /home/duzhong --include=*.sh 2>/dev/null | head -5
crontab -l 2>&1 | head -20
systemctl --user list-units --type=service --no-pager 2>/dev/null | grep -i -E 'agent|uxrce|px4|ros' | head
echo "== fmu on domain 118 =="
source /opt/ros/humble/setup.bash; source ~/dzpx4/install/setup.bash 2>/dev/null
ROS_DOMAIN_ID=118 timeout 8 ros2 topic list 2>&1 | grep -c fmu
echo "== fmu on domain 0 =="
ROS_DOMAIN_ID=0 timeout 8 ros2 topic list 2>&1 | grep fmu | head -10
echo "== all topics domain 0 (head) =="
ROS_DOMAIN_ID=0 timeout 8 ros2 topic list 2>&1 | head -20
