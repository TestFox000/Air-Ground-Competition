set +e
P=/home/duzhong/dzrobot/install/car_localization/lib/python3.10/site-packages/car_localization/path_executor.py
echo "===== reset handler (960-1030) ====="
sed -n '960,1030p' "$P"
echo
echo "===== constants ====="
grep -nE "^LINEAR_SPEED|^DRIFT_WARN_M|SHOOT_STOP_RADIUS *=|SHOOT_OVERSHOOT_TOL *=|^APPROACH|LINEAR_SPEED *=|DRIFT_WARN_M *=" "$P" | head -20
echo
echo "===== route load: where does it re-anchor? (grep snap_x_off assignments) ====="
grep -n "snap_x_off *=" "$P"
echo
echo "===== config key format uses ====="
grep -nE "_CFG\.get\(" "$P" | head -25
echo DONE
