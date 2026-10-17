#!/bin/bash
# t4 收尾：列出并清理本任务在 /tmp 起的 t4 探针进程（只杀我们自己的，按 /tmp/t4_ 路径匹配）
echo "=== 残留的 t4 探针进程 ==="
ps -eo pid,etime,cmd | grep -F "/tmp/t4_" | grep -v grep || echo "(无)"
echo
echo "=== 清理（只匹配 /tmp/t4_*.py 的 python3 探针，不碰任何项目进程）==="
for p in $(ps -eo pid,cmd | grep -F "/tmp/t4_" | grep -v grep | awk '{print $1}'); do
  cmd=$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)
  case "$cmd" in
    *"/tmp/t4_"*.py*) echo "  kill $p  ($cmd)"; kill "$p" 2>/dev/null ;;
    *) echo "  跳过 $p（不是 t4 探针）: $cmd" ;;
  esac
done
sleep 2
echo
echo "=== 清理后复查 ==="
ps -eo pid,cmd | grep -F "/tmp/t4_" | grep -v grep || echo "(干净)"
echo
echo "=== 确认没碰项目进程：dzbrain / path_executor / dzbrain 状态 ==="
pgrep -af "dzbrain|path_executor" || echo "(车栈确实没起，冷态保持)"
echo "### done"
