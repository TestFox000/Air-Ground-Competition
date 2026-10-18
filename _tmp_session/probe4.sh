set +e
echo "== watchdog source (head 70) =="
sed -n '1,70p' /home/duzhong/dzuav_deploy/air_ground/tools/orin_agent_watchdog.py 2>&1
echo "== 17:24 agent log =="
cat /home/duzhong/drone_logs/20261017_172451_agent.log 2>&1 | head -30
echo "== node_agent.log session lines =="
grep -a -i -E 'session|client|create' /home/duzhong/drone_logs/node_agent.log 2>/dev/null | tail -8
echo "== node_agent.log size/tail =="
ls -l /home/duzhong/drone_logs/node_agent.log; tail -6 /home/duzhong/drone_logs/node_agent.log 2>&1
echo "== autostart.log tail =="
tail -12 /home/duzhong/drone_logs/autostart.log 2>&1
echo "== px4_msgs dirs =="
ls -d /home/duzhong/*px4* 2>/dev/null; find /home/duzhong -maxdepth 4 -type d -name px4_msgs 2>/dev/null | head -5
echo "== wdog log tail =="
tail -20 /home/duzhong/drone_logs/node_wdog.log 2>&1
echo "== agent_watchdog.log tail =="
tail -8 /home/duzhong/drone_logs/agent_watchdog.log 2>&1
