#!/bin/bash
# 车侧只读体检（不写任何文件）
echo "=== 主机 / 开机时长 ==="
hostname; uptime | head -1; date
echo
echo "=== 8 步栈进程 ==="
pgrep -af 'path_executor_node|serial_reader_node|dzactuator|dzjudgment|det_node|cam_viewer|usb_cam_node|dzrknn' | cut -c1-120
echo
echo "=== 受管 PID 文件数 ==="
ls -1 ~/dzrobot/log/pids 2>/dev/null | wc -l
echo
echo "=== 域号来源 ==="
grep -h 'ROS_DOMAIN_ID' ~/.bashrc ~/dzrobot/start_all.sh ~/dzrobot/dzros.env 2>/dev/null | head -8
echo
echo "=== 8080 网页台 ==="
curl -s --max-time 5 http://127.0.0.1:8080/api/status | head -c 400
echo
echo "=== 电池电压（底盘自报）==="
for t in /PowerVoltage /Battery_Percentage; do
  printf '%s = ' "$t"
  timeout 6 curl -s "http://127.0.0.1:8080/api/read?topic=$t" 2>/dev/null | head -c 200
  echo
done
echo
echo "=== 定位 ==="
timeout 6 curl -s http://127.0.0.1:8080/api/status 2>/dev/null | head -c 200
echo
echo "=== 7_path.log 尾部 ==="
tail -6 ~/dzrobot/log/7_path.log 2>/dev/null
echo
echo "=== auto_start 配置 ==="
grep -n -E '^\s*auto_start' ~/dzrobot/config/robot.yaml 2>/dev/null | head -3
