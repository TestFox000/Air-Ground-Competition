#!/bin/bash
echo "=== uname"; uname -a
echo "=== nproc/mem"; nproc; free -m | head -3
echo "=== uptime"; uptime
echo "=== top"; top -bn1 | head -18
echo "=== top procs"; ps -eo pid,pcpu,pmem,comm,args --sort=-pcpu | head -14
echo "=== tools"; which tegrastats nvpmodel jetson_clocks 2>/dev/null
echo "=== nvpmodel"; nvpmodel -q 2>&1 | head -5
echo "=== gpu load"; cat /sys/devices/gpu.0/load 2>/dev/null; ls /sys/class/devfreq 2>/dev/null
echo "=== devfreq gpu"; for f in /sys/class/devfreq/*gpu*/cur_freq /sys/class/devfreq/*gpu*/max_freq; do echo -n "$f "; cat $f 2>/dev/null; done
echo "=== cpufreq"; cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor; cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null
echo "=== drone procs"; pgrep -af "dzuav|fastlio|livox|usb_cam|start_drone|ros2|MicroXRCE" | head -20
echo "=== ros nodes"; timeout 8 bash -c "source /opt/ros/humble/setup.bash 2>/dev/null; ros2 node list 2>/dev/null" | head -20
