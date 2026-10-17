set +e
echo "== A) locate log dir =="
ls -d ~/dzrobot/log/robot_logs ~/robot_logs ~/dzrobot/robot_logs 2>/dev/null
D=$(find ~ -maxdepth 4 -type d -name robot_logs 2>/dev/null | head -1)
echo "LOGDIR=$D"
ls -la "$D" 2>/dev/null | tail -20

echo
echo "== B) 3_localization.log line count =="
wc -l "$D/3_localization.log" 2>/dev/null

echo
echo "== C) localization tail 30 =="
tail -n 30 "$D/3_localization.log" 2>/dev/null

echo
echo "== D) localization pos/block lines (last 80) =="
grep -anE "pos:|block:|reset|drift" "$D/3_localization.log" 2>/dev/null | tail -80

echo
echo "== E) 7_path.log tail 90 =="
tail -n 90 "$D/7_path.log" 2>/dev/null

echo
echo "== F) locate py sources =="
find ~/dzrobot ~/dzrobot_v2 -maxdepth 8 -name path_executor.py -o -maxdepth 8 -name serial_reader.py 2>/dev/null | head -10

echo
echo "== G) stack running? =="
pgrep -af "path_executor|serial_reader" 2>/dev/null | head -5
echo DONE
