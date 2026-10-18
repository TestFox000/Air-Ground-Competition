set +e
PID=$(pgrep -x MicroXRCEAgent | head -1)
echo "== agent pid=$PID =="
if [ -n "$PID" ]; then
  echo "-- io t0 --"; awk '{print}' /proc/$PID/io | head -6
  sleep 8
  echo "-- io t1 (after 8s) --"; awk '{print}' /proc/$PID/io | head -6
fi
echo "== last non-init lines of node_agent.log =="
tail -600 /home/duzhong/drone_logs/node_agent.log | sed 's/\x1b\[[0-9;]*m//g' | grep -v 'TermiosAgentLinux\|logger setup' | tail -25
echo "== timestamps of init restarts in node_agent.log (last 6) =="
grep -a 'TermiosAgentLinux' /home/duzhong/drone_logs/node_agent.log | sed 's/\x1b\[[0-9;]*m//g' | awk -F'[][]' '{print $2}' | tail -6
echo "== any delete/timeout/error lines =="
grep -a -i -E 'delete|timeout|error|fail|reconnect' /home/duzhong/drone_logs/node_agent.log | tail -10
