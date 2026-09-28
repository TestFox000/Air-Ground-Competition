#!/usr/bin/env bash
# 车端脚本语法检查：把本仓库所有 .sh 拉过去过一遍 `bash -n`。
# 本机没有 bash（Windows），所以只能借车端的 bash 校验；检查完不执行、不留痕。
#
# 用法：
#   for s in $(ls tools/*.sh); do python tools/remote_run.py --host <IP> --user duzhong --put "$s" --dest "/tmp/syntaxcheck/$(basename $s)"; done
#   python tools/remote_run.py --host <IP> --user duzhong --script tools/car_syntaxcheck.sh

FAIL=0
echo "==================== 车端脚本语法检查 ===================="
for f in /tmp/syntaxcheck/*.sh; do
  [ -e "$f" ] || { echo "  （/tmp/syntaxcheck 为空）"; break; }
  if out=$(bash -n "$f" 2>&1); then
    echo "  [OK]   $(basename "$f")"
  else
    echo "  [FAIL] $(basename "$f")"
    echo "$out" | sed 's/^/         /'
    FAIL=$((FAIL+1))
  fi
done
echo
echo "  失败 $FAIL 个"
echo "==================== 结束 ===================="
exit "$FAIL"
