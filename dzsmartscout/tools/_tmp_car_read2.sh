#!/bin/bash
# read-only diagnostics: run ON the drone, reads the car over ssh. No writes anywhere.
CAR=10.226.211.188
SSH="sshpass -p duzhong ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 -o LogLevel=ERROR duzhong@$CAR"
echo "=== 1) drone ==="
hostname
echo "=== 2) car ping ==="
ping -c1 -W2 $CAR >/dev/null 2>&1 && echo "CAR PING OK" || echo "CAR PING FAIL"
echo "=== 3) neighbours ==="
ip neigh 2>/dev/null | head -10
echo "=== 4) log dir ==="
$SSH 'ls -la ~/dzrobot/log/robot_logs/ 2>/dev/null | tail -18'
echo "=== 5) 7_path.log tail 40 ==="
$SSH 'tail -n 40 ~/dzrobot/log/robot_logs/7_path.log 2>/dev/null'
echo "=== 6) localization tail 3 ==="
$SSH 'tail -n 3 ~/dzrobot/log/robot_logs/3_localization.log 2>/dev/null'
echo "=== 7) reset token support ==="
$SSH 'grep -rn "reset:" ~/dzrobot/install --include=*.py 2>/dev/null | head -20'
echo "=== 8) config keys ==="
$SSH 'grep -n "mission_reset\|auto_start\|arrival\|stop_dist\|tolerance" ~/dzrobot/config/robot.yaml 2>/dev/null | head -20'
echo "=== 9) running ==="
$SSH 'pgrep -af path_executor | head -5'
echo "DONE"
