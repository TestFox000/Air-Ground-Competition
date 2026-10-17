set +e
D=/home/duzhong/robot_logs
S=/home/duzhong/dzrobot/install/car_localization/lib/python3.10/site-packages/car_localization/serial_reader.py
P=/home/duzhong/dzrobot/install/car_localization/lib/python3.10/site-packages/car_localization/path_executor.py

echo "===== 1) FULL 7_path.log ====="
cat "$D/7_path.log"

echo
echo "===== 2) block sequence (dwell counts at 5Hz) ====="
grep -a "pos:" "$D/3_localization.log" | sed -E 's/.*block: ([A-Z][0-9]+).*/\1/' | uniq -c | tail -60

echo
echo "===== 3) trajectory at 0.1-block resolution (counts) ====="
grep -a "pos:" "$D/3_localization.log" | sed -E 's/.*pos: \(([-0-9.]+), ([-0-9.]+)\).*/\1 \2/' | awk '{printf "%.1f %.1f\n", $1, $2}' | uniq -c | tail -70

echo
echo "===== 4) serial_reader constants ====="
grep -nE "BLOCK_SIZE|3290|3236|TICK|PER_|WHEEL|0\.073|dr_x_m *=|dr_y_m *=|^def |^    def " "$S" | head -60

echo
echo "===== 5) serial_reader reset func ====="
sed -n '150,205p' "$S"

echo
echo "===== 6) serial_reader dr update + pos print ====="
grep -n "pos:" "$S"
sed -n '205,265p' "$S"

echo
echo "===== 7) path_executor arrival/threshold hits ====="
grep -nE "arrival_threshold|snap|0\.073|arrived|arriv|到达|停车" "$P" | head -70
echo DONE
