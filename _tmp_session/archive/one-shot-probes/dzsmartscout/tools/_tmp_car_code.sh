set +e
S=/home/duzhong/dzrobot/install/car_localization/lib/python3.10/site-packages/car_localization/serial_reader.py
P=/home/duzhong/dzrobot/install/car_localization/lib/python3.10/site-packages/car_localization/path_executor.py

echo "===== A) serial_reader.py lines 1-70 (constants) ====="
sed -n '1,70p' "$S"

echo
echo "===== B) serial_reader.py lines 93-150 (init) ====="
sed -n '93,150p' "$S"

echo
echo "===== C) path_executor.py lines 620-665 (threshold + calib comment) ====="
sed -n '620,665p' "$P"

echo
echo "===== D) path_executor.py lines 2296,2400 (arrival branch + drift metric) ====="
sed -n '2296,2400p' "$P"

echo
echo "===== E) odom rate check: timestamps of last 12 pos lines ====="
grep -a "pos:" /home/duzhong/robot_logs/3_localization.log | tail -12 | sed -E 's/.*\[([0-9]+\.[0-9]+)\].*/\1/'

echo
echo "===== F) how many odom callbacks vs printed (log_counter%10) ====="
grep -an "log_counter" "$S" | head -10
echo DONE
