#!/bin/bash
# 诊断：日志尾 + 进程数 + yaml 关键键
set +u
echo "=== 日志尾 30 行 ==="
tail -30 /home/duzhong/robot_logs/5_gimbal.log
echo "=== 进程数 ==="
ps -ef | grep -c '[d]zrknn'
echo "=== yaml 关键键 ==="
python3 - <<'PY'
import yaml
p = '/home/duzhong/dzrobot/config/robot.yaml'
d = yaml.safe_load(open(p))
import sys
sec = d.get('/dzrknn_dataprocess') or {}
ros = sec.get('ros__parameters') or {}
print('yaml 解析 OK, 参数个数 =', len(ros))
for k in ('track_mode', 'v1_kd', 'v1_kp', 'pid_enable', 'pid_kd', 'pid_kp', 'pid_ki'):
    print(' ', k, '=', ros.get(k, '<缺失>'))
PY
