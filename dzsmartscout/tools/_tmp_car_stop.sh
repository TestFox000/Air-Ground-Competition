#!/bin/bash
# 只读诊断：在无人机上执行，SSH 进小车读日志与源码。零写入。
CAR=10.226.211.188
SSH="sshpass -p duzhong ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 -o LogLevel=ERROR duzhong@$CAR"

echo "=== A) 日志目录时间戳 ==="
$SSH 'ls -la ~/dzrobot/log/robot_logs/ 2>/dev/null | tail -20'

echo
echo "=== B) 定位日志：行数 + 末尾 25 行 ==="
$SSH 'L=~/dzrobot/log/robot_logs/3_localization.log; wc -l $L; echo "--- tail 25 ---"; tail -n 25 $L'

echo
echo "=== C) 定位日志：所有含 pos/block 的行（最近 60 条）==="
$SSH 'L=~/dzrobot/log/robot_logs/3_localization.log; grep -anE "pos:|block:|漂移|reset|锚" $L | tail -60'

echo
echo "=== D) 7_path.log 尾部 80 行 ==="
$SSH 'tail -n 80 ~/dzrobot/log/robot_logs/7_path.log'

echo
echo "=== E) 找源码文件 ==="
$SSH 'find ~/dzrobot/install -name "path_executor.py" -o -name "serial_reader.py" 2>/dev/null'

echo "DONE"
