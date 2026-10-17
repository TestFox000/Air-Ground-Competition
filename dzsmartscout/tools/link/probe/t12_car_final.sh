#!/bin/bash
# t12 收尾取证：车侧冷态与零改动 + /tmp 探针清单（只读）
CUT="${1:-2026-10-12 19:35:00}"
echo "=== 1) 车侧：t12 探针进程（应为空）==="
ps -eo pid,cmd | grep -F '/tmp/t12_' | grep -v grep || echo "(无)"
echo "=== 2) 车侧：车栈是否仍冷态 ==="
pgrep -af 'dzbrain|path_executor|usb_cam|serial_reader' || echo "(无 dzbrain/path_executor → 冷态保持)"
echo "=== 3) 车侧：/tmp 下 t12 相关文件 ==="
ls -l --time-style=+%Y-%m-%dT%H:%M:%S /tmp/t12_* 2>/dev/null || echo "(无)"
echo "=== 4) 车侧：~/dzrobot 本次是否被改（应为空）==="
find /home/duzhong/dzrobot -type f -newermt "$CUT" \
     -not -path '*/install/*' -not -path '*/build/*' -not -path '*/log/*' 2>/dev/null | head
echo "(空 = 未被触碰)"
echo "=== 5) 车侧：关键文件 md5（与 t4 基线对照）==="
md5sum /home/duzhong/dzrobot/start_all.sh /home/duzhong/dzrobot/config/robot.yaml \
       /home/duzhong/dzrobot/dzros.env /home/duzhong/.bashrc 2>/dev/null
echo "=== 6) 车侧：dzrobot_v2 是否存在于本机（本任务要求不碰）==="
ls -d /home/duzhong/dzrobot_v2 2>/dev/null || echo "(车上没有 dzrobot_v2 目录)"
echo "### done"
