#!/bin/bash
set -u
DEST=/home/duzhong/dzrobot/install/car_localization/lib/python3.10/site-packages/car_localization/serial_reader.py
LOG=/home/duzhong/r3_field_20261021/logs
BAK="$DEST.bak_r3reset_20261022"
echo "=== PATCH ==="
cp -a "$DEST" "$BAK" 2>/dev/null || true
python3 - "$DEST" <<'PYEOF'
import sys
dest = sys.argv[1]
src = open(dest, encoding='utf-8').read()
OLD = '''        self.get_logger().info(
            f"[{NODE_VERSION}] reset: DR direction re-anchored, "
            f"map_yaw -> {math.degrees(target_yaw):+.0f}deg "
            f"(was {math.degrees(cur):+.0f}deg)")'''
NEW = OLD + '''
        # ---- R3 field patch 2026-10-22: re-anchor DR POSITION too ----
        # Why: path_executor is NOT running in the R3 stack (it fights
        # dzbrain for the chassis), so its snap logic is unavailable.
        # Documented reset semantics = "the car is AT block, FACING head,
        # right now"; without this the belief stays frozen at the
        # hard-coded N10 start (2026-10-22: belief stuck at (0.525,0.175)
        # while the car was physically placed at E11).
        try:
            blk = parts[0].strip().upper()
            col0 = ord(blk[0]) - ord('A')
            row = int(blk[1:])
            self.dr_x_m = (11.5 - row) * BLOCK_SIZE
            self.dr_y_m = (13.5 - col0) * BLOCK_SIZE
            self.prev_px = None
            self.prev_py = None
            self.get_logger().info(
                f"[{NODE_VERSION}] reset: DR position re-anchored to "
                f"{blk} -> ({self.dr_x_m:.3f}, {self.dr_y_m:.3f}) m")
        except (ValueError, IndexError) as exc:
            self.get_logger().error(
                f"reset: bad block spec {parts[0]!r}: {exc}")
'''
n = src.count(OLD)
print('marker count =', n)
if n != 1:
    sys.exit(3)
open(dest, 'w', encoding='utf-8').write(src.replace(OLD, NEW, 1))
print('PATCH WRITTEN')
PYEOF
if [ $? -ne 0 ]; then echo "PATCH FAILED -> abort"; exit 1; fi
python3 -m py_compile "$DEST" && echo "PY_COMPILE OK"
echo "=== RESTART localization + dzbrain ==="
pkill -TERM -f 'car_localization localization.launch.py' 2>/dev/null
pkill -TERM -f 'serial_reader_node' 2>/dev/null
pkill -TERM -f 'dzbrain.main --config /home/duzhong/r3_field_20261021' 2>/dev/null
sleep 3
cd /home/duzhong/dzrobot
source /opt/ros/humble/setup.bash
source /home/duzhong/dzrobot/dzros.env
source /home/duzhong/dzrobot/install/setup.bash
setsid ros2 launch car_localization localization.launch.py > "$LOG/3_localization_aim.log" 2>&1 &
echo $! > "$LOG/3_localization_aim.pid"
sleep 3
PYTHONUNBUFFERED=1 PYTHONIOENCODING=utf-8 setsid python3 -u -m dzbrain.main --config /home/duzhong/r3_field_20261021/config/robot_r3.yaml --map /home/duzhong/dzrobot/config/map.yaml > "$LOG/7_path_aim.log" 2>&1 &
echo $! > "$LOG/7_path_aim.pid"
echo "spawned: loc=$(cat $LOG/3_localization_aim.pid) brain=$(cat $LOG/7_path_aim.pid)"
echo "=== WAIT 16s ==="
sleep 16
echo "=== PROC ==="
pgrep -af 'serial_reader_node|dzbrain.main|localization.launch' | head
echo "=== LOC TAIL ==="
tail -6 "$LOG/3_localization_aim.log"
echo "=== BRAIN STATE ==="
grep -n '阶段\|安全\|就绪\|SAFE_STOP\|复位' "$LOG/7_path_aim.log" | tail -12
echo "=== BRAIN TAIL ==="
tail -6 "$LOG/7_path_aim.log"
