#!/usr/bin/env bash
# 只读探针：查小车当前配置/标定/ROS 栈状态（不写任何文件）
set -u
echo "########## 0. 基本信息 ##########"
hostname; date; uptime | head -1

echo
echo "########## 1. home 目录结构 ##########"
ls -la ~ | head -30

echo
echo "########## 2. 工作区存在性 ##########"
for d in ~/dzrobot ~/dzrobot_v2 ~/_backups ~/_quarantine_data ~/robot_logs; do
  if [ -d "$d" ]; then echo "存在: $d  ($(du -sh "$d" 2>/dev/null | cut -f1))"; else echo "缺失: $d"; fi
done

echo
echo "########## 3. 标定值（grep 原文） ##########"
grep -nE 'ticksPer(Meter|2PI)' ~/dzrobot/config/robot.yaml 2>/dev/null || echo "(未找到 ~/dzrobot/config/robot.yaml)"

echo
echo "########## 4. 配置语义解析 ##########"
if [ -f ~/dzrobot/config/robot.yaml ]; then
python3 - <<'PY'
import yaml
try:
    d = yaml.safe_load(open('/home/duzhong/dzrobot/config/robot.yaml'))
    a = d.get('/dzactuator', {}).get('ros__parameters', {})
    p = d.get('path_executor', {}).get('ros__parameters', {})
    print('ticksPerMeter :', a.get('ticksPerMeter'))
    print('ticksPer2PI   :', a.get('ticksPer2PI'))
    print('waypoints     :', len(p.get('mission_blocks', [])))
    print('auto_start    :', p.get('auto_start'))
    print('linear_speed  :', p.get('linear_speed'))
    print('obstacles     :', p.get('obstacle_blocks'))
    print('bridge_blocks :', (p.get('bridge_blocks') or ['<无>'])[:3], '... 共', len(p.get('bridge_blocks') or []))
    print('mission_blocks:')
    for i, b in enumerate(p.get('mission_blocks', [])):
        print(f'   {i:2d} {b}')
except Exception as e:
    print('解析失败:', e)
PY
else
  echo "(跳过)"
fi

echo
echo "########## 5. 地图配置 ##########"
cat ~/dzrobot/config/map.yaml 2>/dev/null || echo "(无)"

echo
echo "########## 6. 配置指纹 ##########"
md5sum ~/dzrobot/config/robot.yaml ~/dzrobot/config/map.yaml 2>/dev/null

echo
echo "########## 7. ROS 栈状态 ##########"
source /opt/ros/humble/setup.bash 2>/dev/null
timeout 10 ros2 node list 2>&1 | head -25 || echo "(ros2 node list 超时/失败)"

echo
echo "########## 8. 相关进程 ##########"
ps -eo pid,stat,pcpu,etime,cmd --sort=-pcpu 2>/dev/null | grep -E 'dzactuator|dzjudgment|dzrknn|path_exec|car_local|rknn_pt|cam_viewer|ros2' | grep -v grep | head -15 || echo "(无匹配进程)"

echo
echo "########## 9. 日志目录 ##########"
ls -lt ~/robot_logs 2>/dev/null | head -12 || echo "(无 robot_logs)"

echo
echo "########## 10. 平行工作区 ~/dzrobot_v2 摘要 ##########"
if [ -d ~/dzrobot_v2 ]; then
  ls -la ~/dzrobot_v2 | head -20
  echo "--- src 包 ---"; ls ~/dzrobot_v2/src 2>/dev/null | head -20
  echo "--- config ---"; ls -l ~/dzrobot_v2/config 2>/dev/null | head -10
else
  echo "(无 ~/dzrobot_v2)"
fi

echo
echo "########## 探针结束（全程只读） ##########"
