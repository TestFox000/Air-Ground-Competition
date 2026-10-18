#!/bin/bash
echo "=== tegrastats (4s)"
timeout 4 tegrastats --interval 1000 2>&1 | tail -5
echo "=== cpufreq"
for i in 0 1 2 3 4 5; do echo -n "cpu$i "; cat /sys/devices/system/cpu/cpu$i/cpufreq/scaling_cur_freq 2>/dev/null; done
echo "=== gpu"
cat /sys/class/devfreq/17000000.gpu/cur_freq 2>/dev/null
cat /sys/class/devfreq/17000000.gpu/load 2>/dev/null
echo "=== thermal"
for z in /sys/devices/virtual/thermal/thermal_zone*/temp; do echo -n "$z "; cat $z 2>/dev/null; done
echo "=== usb errors"
dmesg 2>/dev/null | grep -i -E "video0|uvc|usb.*error|bandwidth" | tail -10
echo "=== camera fps from HUD"
ls -la /tmp/dzuav_view/latest_sq.jpg 2>/dev/null
