set +e
echo "===== drone bash history: sender invocations ====="
grep -a "dz_send_waypoint" ~/.bash_history 2>/dev/null | tail -40
echo
echo "===== drone: dzlink_tools dir ====="
ls -la ~/dzlink_tools/ 2>/dev/null
echo
echo "===== drone: any sender logs ====="
ls -la /tmp/*.log 2>/dev/null | tail -10
echo
echo "===== drone: current env ====="
echo "DOMAIN=[${ROS_DOMAIN_ID:-unset}]"
echo DONE
