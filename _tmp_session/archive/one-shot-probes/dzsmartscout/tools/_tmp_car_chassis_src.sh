set +e
cd ~/dzrobot
echo "=== dzactuator sources ==="
find . -maxdepth 7 \( -name "*.cpp" -o -name "*.py" -o -name "*.hpp" -o -name "*.h" \) -path "*dzactuator*" 2>/dev/null | head -20
echo "=== callback_movebase_angle / Pursuit ==="
grep -rn "movebase_angle\|Pursuit\|pursuit" --include=*.cpp --include=*.hpp --include=*.py . 2>/dev/null | grep -v "^./build/" | head -25
echo
echo "=== cam_viewer sources ==="
find . -maxdepth 7 -iname "*cam*view*" 2>/dev/null | head -10
echo "=== web api endpoints ==="
grep -rn "route('/api\|@app.route\|add_url_rule" --include=*.py . 2>/dev/null | grep -v "^./build/" | head -25
echo "=== DONE ==="
