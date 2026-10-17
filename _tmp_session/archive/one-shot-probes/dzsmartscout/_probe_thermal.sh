echo "--- types"
cat /sys/class/thermal/thermal_zone*/type
echo "--- temps"
cat /sys/class/thermal/thermal_zone*/temp
echo "--- cpu line"
awk '/^cpu /{print $2,$3,$4,$5,$6,$7,$8,$9,$10}' /proc/stat
echo "--- uptime"
cat /proc/uptime
