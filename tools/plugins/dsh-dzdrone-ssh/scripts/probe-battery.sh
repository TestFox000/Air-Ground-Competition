#!/bin/sh
# Battery-source probe for dzsmart (Jetson Orin Nano). READ-ONLY.
#
# The drone's battery does NOT live on the board (no /sys/class/power_supply,
# no fuel-gauge module, no battery DT node - it is USB-C/5V powered). It lives
# in the PX4 flight controller, surfaced over uXRCE-DDS as
# /fmu/out/battery_status (px4_msgs/msg/BatteryStatus: remaining 0..1, voltage_v).
# That topic exists only while the FC is plugged in and MicroXRCEAgent runs.
#
# Run locally via:
#   Get-Content -Raw scripts/probe-battery.sh | ssh duzhong@dzsmart.local "bash -s"
echo "=== DZB_BEGIN ==="
echo "host: $(hostname)  kernel: $(uname -r)"
echo "--- board power supply (expect: empty, no on-board battery) ---"
ls -1 /sys/class/power_supply/ 2>/dev/null | sed 's/^/  /' || echo "  (none)"
echo "--- INA3221 rails (supply voltage, not a battery gauge) ---"
for d in /sys/class/hwmon/hwmon*; do
  [ "$(cat "$d/name" 2>/dev/null)" = "ina3221" ] || continue
  for f in "$d"/in*_label "$d"/in*_input; do
    [ -e "$f" ] && echo "  $(basename "$f")=$(cat "$f" 2>/dev/null)"
  done
done
echo "--- flight-controller serial device (absent = no FC) ---"
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | sed 's/^/  /' || echo "  (none)"
echo "--- PX4 battery over uXRCE-DDS (present only with FC) ---"
if [ -f /opt/ros/humble/setup.bash ]; then
  bash -c '. /opt/ros/humble/setup.bash 2>/dev/null; timeout 6 ros2 topic info /fmu/out/battery_status 2>/dev/null' | sed 's/^/  /' || echo "  (no /fmu/out/battery_status)"
  bash -c '. /opt/ros/humble/setup.bash 2>/dev/null; timeout 6 ros2 topic echo /fmu/out/battery_status --once 2>/dev/null' | grep -E 'remaining:|voltage_v:' | sed 's/^/  /' || echo "  (echo empty)"
else
  echo "  (no /opt/ros/humble)"
fi
echo "=== DZB_END ==="
