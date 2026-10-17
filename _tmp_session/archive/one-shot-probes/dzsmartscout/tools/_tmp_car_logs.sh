set +e
echo "=== 7_path.log FULL (bytes: $(wc -c < ~/robot_logs/7_path.log)) ==="
cat -A ~/robot_logs/7_path.log | sed 's/\$$//' | head -40
echo
echo "=== 7_path.log as-is ==="
cat ~/robot_logs/7_path.log
echo
echo "=== localization log line sample ==="
tail -1 ~/robot_logs/3_localization.log
echo "=== pos count ==="
grep -c "pos:(" ~/robot_logs/3_localization.log
echo "=== pos FIRST 3 ==="
grep -o "pos:([^)]*)[^|]*" ~/robot_logs/3_localization.log | head -3
echo "=== pos LAST 3 ==="
grep -o "pos:([^)]*)[^|]*" ~/robot_logs/3_localization.log | tail -3
echo "=== distinct pos values (all) ==="
grep -o "pos:([^)]*)" ~/robot_logs/3_localization.log | sort | uniq -c | sort -rn | head -20
echo "=== yaw distinct ==="
grep -o "yaw:[+-]*[0-9]*" ~/robot_logs/3_localization.log | sort | uniq -c | sort -rn | head -20
echo "=== other logs mtime ==="
ls -l --time-style=+%H:%M:%S ~/robot_logs/ | head -20
echo "=== DONE ==="
