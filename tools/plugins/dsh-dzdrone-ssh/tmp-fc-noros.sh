echo DZH_BEGIN
awk '/^cpu /{print $2,$3,$4,$5,$6,$7,$8,$9,$10; exit}' /proc/stat
sleep 1
awk '/^cpu /{print $2,$3,$4,$5,$6,$7,$8,$9,$10; exit}' /proc/stat
for z in /sys/class/thermal/thermal_zone*; do n=$(cat "$z/type" 2>/dev/null) || continue; t=$(cat "$z/temp" 2>/dev/null) || t=$(cat "$z/temp" 2>/dev/null) || continue; printf "temp_%s=%s\n" "$n" "$t"; done
awk '{printf "uptimeSec=%s\n", $1}' /proc/uptime
awk '/^btime/{printf "bootEpoch=%s\n", $2}' /proc/stat
awk '{printf "load1=%s load5=%s load15=%s\n", $1, $2, $3}' /proc/loadavg
if true; then printf 'batteryFc=present\n'; if [ -f /opt/ros/humble/setup.bash.NOPE ]; then o=$(bash -c '. /opt/ros/humble/setup.bash 2>/dev/null; timeout 3 ros2 topic echo /fmu/out/battery_status --once 2>/dev/null' | awk '/^[[:space:]]*remaining:/{r=$2+0} /^[[:space:]]*voltage_v:/{v=$2+0} END{if(r>=0 && r<=1 && v>0) printf "batteryPercent=%s batteryVoltage=%s\n", r*100, v}'); if [ -n "$o" ]; then printf '%s\n' "$o"; else printf 'batteryNote=dds_silent\n'; fi; else printf 'batteryNote=no_ros\n'; fi; else printf 'batteryFc=absent\n'; fi
echo DZH_END