set +e
echo "== blocking 6s read of ttyACM0 =="
timeout 6 cat /dev/ttyACM0 2>/dev/null | wc -c
echo "== lsusb -t =="
lsusb -t 2>&1
echo "== px4 usb ids =="
cat /sys/bus/usb/devices/1-2.1/idVendor /sys/bus/usb/devices/1-2.1/idProduct /sys/bus/usb/devices/1-2.1/bNumInterfaces /sys/bus/usb/devices/1-2.1/speed 2>&1
echo "== which iface is cdc_acm =="
for i in 1-2.1:1.0 1-2.1:1.1; do echo -n "$i: "; readlink -f /sys/bus/usb/devices/$i/driver 2>/dev/null | xargs -r basename; done
echo "== tty ttyACM0 iface =="
ls -l /sys/class/tty/ttyACM0/device 2>&1 | head -2
echo "== agent log create count total (whole file) =="
sed 's/\x1b\[[0-9;]*m//g' /home/duzhong/drone_logs/agent_watchdog_agent.log | grep -c 'create_publisher'
echo "== last 3 watchdog lines =="
tail -3 /home/duzhong/drone_logs/agent_watchdog.log
