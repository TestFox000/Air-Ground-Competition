set +e
P=$(pgrep -x MicroXRCEAgent | head -1); echo "pid=$P"
echo "== cat /proc/io =="; head -8 /proc/$P/io 2>&1
echo "== passive byte count on ttyACM0 for 5s (competes with agent, agent restarts anyway) =="
timeout 5 dd if=/dev/ttyACM0 bs=4096 count=64 iflag=nonblock 2>/dev/null | wc -c
echo "== agent log tail after that =="
sed 's/\x1b\[[0-9;]*m//g' /home/duzhong/drone_logs/agent_watchdog_agent.log | tail -4
echo "== usb device dir =="
ls -l --time-style=+%F_%T /sys/bus/usb/devices/ 2>/dev/null | grep -E '1-2|1-1' | head
echo "== usb dev mtime =="
stat -c '%n %y' /sys/bus/usb/devices/1-2.1 2>&1
echo "== boot/uptime =="; uptime -s; uptime -p
echo "== last create_publisher timestamp (epoch) =="
grep -a 'create_publisher' /home/duzhong/drone_logs/node_agent.log | tail -1 | sed 's/\x1b\[[0-9;]*m//g'
python3 - <<'EOF'
import datetime, subprocess
ts=1792216267.608
print("last create_publisher ->", datetime.datetime.fromtimestamp(ts).strftime('%F %T'))
EOF
