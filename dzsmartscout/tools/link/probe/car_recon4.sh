#!/bin/bash
# Read-only: which core does start_all.sh step 7 actually run?
echo "=== A. start_all.sh 第 185-215 行（第 7 步选择逻辑）==="
sed -n '185,215p' "$HOME/dzrobot/start_all.sh"
echo
echo "=== B. 车上有 dzbrain 吗 ==="
. /opt/ros/humble/setup.bash
. "$HOME/dzrobot/install/setup.bash"
printf "ros2 pkg prefix dzbrain: %s\n" "$(ros2 pkg prefix dzbrain 2>&1 | head -1)"
python3 -c "import dzbrain, os; print('import dzbrain OK ->', os.path.dirname(dzbrain.__file__))" 2>&1 | head -3
echo "--- src 下有什么 ---"
ls "$HOME/dzrobot/src/" 2>/dev/null
echo "--- 有没有 dzbrain 目录 ---"
find "$HOME/dzrobot" -maxdepth 4 -type d -name dzbrain 2>/dev/null | head
echo
echo "=== C. 旧核是否订阅 B 通道 /dzlink/drone_to_car ==="
grep -rn "dzlink" "$HOME/dzrobot/install/car_localization/lib/python3.10/site-packages/car_localization/path_executor.py" 2>/dev/null | head -5
echo "(空 = 旧核不吃 B 通道)"
echo
echo "=== D. LEGACY_PATH 开关是否存在 ==="
grep -n "LEGACY_PATH\|dzbrain" "$HOME/dzrobot/start_all.sh" | head -10
