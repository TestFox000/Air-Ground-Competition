set +e
cd ~/robot_logs
echo "=== 1_chassis.log TAIL 30 ==="
tail -30 1_chassis.log
echo
echo "=== 5_gimbal.log TAIL 25 ==="
tail -25 5_gimbal.log
echo
echo "=== 5b_aimshim.log (full) ==="
cat 5b_aimshim.log
echo
echo "=== 6_judgment.log (full) ==="
cat 6_judgment.log
echo
echo "=== 8_camview.log (full) ==="
cat 8_camview.log
echo
echo "=== pids dir ==="
ls -l pids/
echo "=== DONE ==="
