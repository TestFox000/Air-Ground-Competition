set +e
echo "=== CWD ==="
cd ~/dzrobot && pwd
echo "=== AUTOSTART (config/robot.yaml) ==="
grep -n "auto_start" config/robot.yaml | head -5
echo "=== LOG DIRS ==="
for d in ~/dzrobot/log ~/robot_logs ~/dzrobot/robot_logs; do
  if [ -d "$d" ]; then echo "--- $d ---"; ls -l "$d"/7_path.log "$d"/3_localization.log 2>/dev/null; fi
done
echo "=== BANNER COUNT in 7_path.log ==="
for f in ~/dzrobot/log/7_path.log ~/robot_logs/7_path.log; do
  if [ -f "$f" ]; then echo "$f : >>> count = $(grep -c '>>>' "$f")"; fi
done
echo "=== 7_path.log TAIL ==="
tail -c 400 ~/dzrobot/log/7_path.log 2>/dev/null
tail -c 400 ~/robot_logs/7_path.log 2>/dev/null
echo
echo "=== POSE SAMPLES (last 3) ==="
tail -600 ~/robot_logs/3_localization.log 2>/dev/null | grep -o "pos:([^)]*)[^|]*yaw:[+-]*[0-9]*" | tail -3
tail -600 ~/dzrobot/log/3_localization.log 2>/dev/null | grep -o "pos:([^)]*)[^|]*yaw:[+-]*[0-9]*" | tail -3
echo "=== DONE ==="
