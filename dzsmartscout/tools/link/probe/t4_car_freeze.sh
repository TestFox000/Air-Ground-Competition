#!/bin/bash
# t4_car_freeze.sh —— 车侧「零改动」基线快照（只读）
#   阶段 1（开始）：bash t4_car_freeze.sh begin
#   阶段 2（结束）：bash t4_car_freeze.sh end
# 两条命令都会把快照写到 /tmp/t4_freeze_<mode>.txt，便于逐项 diff。
MODE="${1:-begin}"
OUT="/tmp/t4_freeze_${MODE}.txt"
{
  echo "### mode=$MODE host=$(hostname) time=$(date -Is)"
  echo "### 关键文件 md5"
  for f in "$HOME/dzrobot/start_all.sh" "$HOME/dzrobot/config/robot.yaml" \
           "$HOME/dzrobot/dzros.env" "$HOME/.bashrc" \
           "$HOME/dzrobot/src/dzbrain/dzbrain/io/contract.py" \
           "$HOME/dzrobot/src/dzbrain/dzbrain/tasks/drone.py"; do
    [ -r "$f" ] && md5sum "$f" || echo "MISSING $f"
  done
  echo "### ~/dzrobot 顶层（名字/大小/mtime，排序稳定）"
  ls -l --time-style=+%Y-%m-%dT%H:%M:%S "$HOME/dzrobot" | tail -n +2 | awk '{print $5, $6, $7}' | sort
  echo "### 全盘 HOME 下文件数（排除 .cache/.ros/log 这类易变目录）"
  find "$HOME" -xdev -type f \
       -not -path "*/.cache/*" -not -path "*/.ros/log/*" -not -path "*/robot_logs/*" \
       -not -path "*/build/*" -not -path "*/install/*" -not -path "*/log/*" 2>/dev/null | wc -l
  echo "### /tmp 下属于本次会话的探针"
  ls -l --time-style=+%Y-%m-%dT%H:%M:%S /tmp/t4_* /tmp/dzlink* 2>/dev/null
} > "$OUT" 2>&1
echo "== 快照写入 $OUT =="
cat "$OUT"
