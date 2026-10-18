set +e
PID=$(pgrep -f 'MicroXRCEAgent serial' | head -1)
echo "agent pid=$PID"
echo "== agent io snapshot 1 =="; grep -E 'rchar|wchar|syscr|syscw' /proc/$PID/io
echo "== agent fds =="; ls -l /proc/$PID/fd 2>/dev/null | head
sleep 6
echo "== agent io snapshot 2 (after 6s) =="; grep -E 'rchar|wchar|syscr|syscw' /proc/$PID/io
echo "== agent still alive? =="; ps -o pid,etime,stat,cmd -p $PID 2>&1
echo "== who has ttyACM* open =="
for p in $(pgrep -f 'MicroXRCE|watchdog|mavlink|px4'); do echo "-- pid $p: $(tr '\0' ' ' < /proc/$p/cmdline)"; ls -l /proc/$p/fd 2>/dev/null | grep -E 'ttyACM|ttyUSB'; done
echo "== ttyACM1 traffic sample (2s) =="
timeout 2 dd if=/dev/ttyACM1 bs=256 count=4 2>/dev/null | wc -c
echo "== ttyACM0 device path =="
readlink -f /sys/class/tty/ttyACM0/device 2>&1
cat /sys/class/tty/ttyACM0/device/interface 2>&1
echo "== usb device power/state =="
cat /sys/bus/usb/devices/1-2.1/authorized 2>&1; cat /sys/bus/usb/devices/1-2.1/manufacturer /sys/bus/usb/devices/1-2.1/product 2>&1
echo "== recent agent log lines with time =="
tail -3 /home/duzhong/drone_logs/agent_watchdog_agent.log
