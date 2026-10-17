#!/bin/bash
L=/home/duzhong/robot_logs
echo "=== 终点横幅/完赛 ==="
grep -n -E 'FINISH|完赛|终点|横幅|DRIVE.*N5|到达终点' $L/7_path.log 2>/dev/null | tail -20
echo "=== 射击停站片段(F3 附近 发/中/开火) ==="
grep -n -E 'F3|开火|发[0-9]/中|允许射击|射击超时|Shoot' $L/7_path.log 2>/dev/null | tail -40
echo "=== 开火指令下发证据 ==="
grep -n -E 'LaserShot|开火|fire|shot|shoot' $L/6_judgment.log 2>/dev/null | tail -20
