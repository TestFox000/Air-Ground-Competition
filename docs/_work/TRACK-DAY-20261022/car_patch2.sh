#!/bin/bash
LOG=/home/duzhong/r3_field_20261021/logs
DEST=/home/duzhong/dzrobot/install/car_localization/lib/python3.10/site-packages/car_localization/serial_reader.py
echo "=== PATCH2 ==="
python3 - "$DEST" <<'PYEOF'
import sys
dest = sys.argv[1]
src = open(dest, encoding='utf-8').read()
OLD = """        # Quaternion carries yaw_rel so path_executor's
        # RAW_YAW_TO_MAP_YAW_OFFSET = pi/2 produces correct map yaw.
        half = yaw_rel / 2.0"""
NEW = """        # Quaternion carries yaw_rel so path_executor's
        # RAW_YAW_TO_MAP_YAW_OFFSET = pi/2 produces correct map yaw.
        # ---- R3 field patch 2026-10-22 (part 2) ----
        # Keep raw(+pi/2)=map_yaw valid AFTER a reset re-bases dr_yaw_off.
        # dzbrain computes map yaw = raw + pi/2; serial_reader's own map
        # yaw is yaw_rel + dr_yaw_off, so publish (map_yaw - pi/2).
        # Without this, reset:E11,x left dzbrain showing yaw=+80deg while
        # the car physically faced +x (0deg) - observed 2026-10-22.
        pub_yaw = normalize_angle(yaw_rel + self.dr_yaw_off - math.pi / 2.0)
        half = pub_yaw / 2.0"""
n = src.count(OLD)
print('marker2 count =', n)
if n == 1:
    open(dest, 'w', encoding='utf-8').write(src.replace(OLD, NEW, 1))
    print('PATCH2 WRITTEN')
elif n == 0:
    print('PATCH2 ALREADY PRESENT (skip)')
else:
    sys.exit(3)
PYEOF
[ $? -ne 0 ] && { echo "PATCH2 FAILED"; exit 1; }
python3 -m py_compile "$DEST" && echo "PY_COMPILE2 OK"
echo "=== RESTART ==="
pkill -TERM -f 'car_localization localization.launch.py' 2>/dev/null
pkill -TERM -f 'serial_reader_node' 2>/dev/null
pkill -TERM -f 'dzbrain.main --config /home/duzhong/r3_field_20261021' 2>/dev/null
sleep 3
cd /home/duzhong/dzrobot
source /opt/ros/humble/setup.bash
source /home/duzhong/dzrobot/dzros.env
source /home/duzhong/dzrobot/install/setup.bash
setsid ros2 launch car_localization localization.launch.py > "$LOG/3_localization_aim.log" 2>&1 &
sleep 4
PYTHONUNBUFFERED=1 PYTHONIOENCODING=utf-8 setsid python3 -u -m dzbrain.main --config /home/duzhong/r3_field_20261021/config/robot_r3.yaml --map /home/duzhong/dzrobot/config/map.yaml > "$LOG/7_path_aim.log" 2>&1 &
sleep 14
echo "=== RESET ==="
timeout 6 ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:E11,x'}" 2>&1 | tail -1
sleep 2
echo "=== LOC ==="
grep -n 'reset:' "$LOG/3_localization_aim.log" | tail -3
tail -3 "$LOG/3_localization_aim.log"
echo "=== BRAIN ==="
grep -n '复位 belief\|阶段' "$LOG/7_path_aim.log" | tail -5
grep -n '\[T\]' "$LOG/7_path_aim.log" | tail -3
