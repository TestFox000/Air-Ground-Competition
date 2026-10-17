set +e
cd ~/dzrobot/src/car_localization/car_localization
echo "=== HEADER 1-58 ==="
sed -n '1,58p' path_executor.py
echo
echo "=== PUBLISHERS 770-840 ==="
sed -n '770,840p' path_executor.py
echo
echo "=== GATES / VALID FLAGS ==="
grep -n "msg_valid\|Drivestate\|self.enabled\|not self\.\|return$" path_executor.py | grep -i "valid\|drive\|enable\|manual\|override\|stop" | head -30
echo
echo "=== TICK FUNCTION DEFS ==="
grep -n "^    def \|^def " path_executor.py | sed -n '1,80p'
echo "=== DONE ==="
