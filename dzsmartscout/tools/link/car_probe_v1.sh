#!/bin/bash
# 车端只读体检 v1 —— 不改任何文件
echo "===== 1. identity ====="
hostname; uptime; date
echo "===== 2. ROS domain ====="
echo "DZ_ROS_DOMAIN=$DZ_ROS_DOMAIN ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
grep -E "^DZ_ROS_DOMAIN|^export ROS_DOMAIN_ID" ~/dzrobot/dzros.env 2>/dev/null
echo "===== 3. stack processes ====="
ps -eo pid,etime,cmd | grep -E "dzbrain|path_executor|cam_viewer|usb_cam|serial_reader|shot_cam" | grep -v grep
echo "(none above = stack not running)"
echo "===== 4. key params (config/robot.yaml) ====="
grep -nE "^[[:space:]]*(auto_start|auto_start_delay_s|cruise_speed_mps|linear_speed|accel_up_mps2|accel_down_mps2|mission_reset|ticksPerMeter|ticksPer2PI|drift_stop|max_shots)" ~/dzrobot/config/robot.yaml 2>/dev/null | head -25
echo "===== 5. mission_blocks token count ====="
python3 - <<'PY'
import re
p = "/home/duzhong/dzrobot/config/robot.yaml"
try:
    s = open(p, encoding="utf-8").read()
except Exception as e:
    print("read fail:", e); raise SystemExit
m = re.search(r"mission_blocks:\s*\n((?:\s+-\s+.*\n)+)", s)
if m:
    toks = re.findall(r"-\s*([A-Za-z0-9_:*@,.\-]+)", m.group(1))
    print("tokens =", len(toks))
    print("route  =", " ".join(toks))
else:
    print("mission_blocks block NOT FOUND")
PY
echo "===== 6. log files ====="
ls -lt ~/dzrobot/log/*.log 2>/dev/null | head -6
echo "===== 7. battery / disk ====="
cat /sys/class/power_supply/*/capacity 2>/dev/null | tr '\n' ' '; echo
df -h / | tail -1
echo "===== 8. devices ====="
ls /dev/video0 /dev/ttyACM* 2>&1 | head -5
