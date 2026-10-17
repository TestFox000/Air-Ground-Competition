#!/bin/bash
# t4 车侧副作用盘点（只读）：本次会话到底动了什么
CUT="2026-10-12 19:09:00"
echo "--- 1) ~/.ros 下本会话新增文件数（ROS 自动日志，非项目文件）---"
find /home/duzhong/.ros -type f -newermt "$CUT" 2>/dev/null | wc -l
echo "--- 2) ~/.ros/log 下本会话新增目录 ---"
find /home/duzhong/.ros/log -maxdepth 1 -newermt "$CUT" 2>/dev/null | head -15
echo "--- 3) ~/dzrobot 下本会话被改/新建的文件（install/build/log 之外）---"
find /home/duzhong/dzrobot -type f -newermt "$CUT" \
     -not -path "*/install/*" -not -path "*/build/*" -not -path "*/log/*" 2>/dev/null | head -20
echo "(空 = 项目目录一字未动)"
echo "--- 4) ~ 下（排除缓存/ROS日志/robot_logs/项目 build·install）本会话被改的文件 ---"
find /home/duzhong -xdev -type f -newermt "$CUT" \
     -not -path "*/.cache/*" -not -path "*/.ros/*" -not -path "*/robot_logs/*" \
     -not -path "*/dzrobot/install/*" -not -path "*/dzrobot/build/*" 2>/dev/null | head -30
echo "--- 5) /tmp 里属于 t4 的探针数量与占用 ---"
ls -1 /tmp/t4_* 2>/dev/null | wc -l
du -sh /tmp 2>/dev/null
echo "--- 6) 有没有任何 systemd/cron/自启被改（应为空）---"
find /etc/systemd /home/duzhong/.config/autostart -newermt "$CUT" 2>/dev/null | head
echo "### done"
