#!/usr/bin/env bash
# 用契约校验器扫一遍车端现存的 .rknn 模型（只读）。
#
# 目的有两个：
#   1) 独立确认"车端在用的模型输出确实是 [1,9,8400]"（不靠源码注释，靠模型文件本身）
#   2) 顺便验证我们自己的契约检查器（rknn_io 解析器）在真实文件上能跑通
#
# 用法：
#   python tools/remote_run.py --host <IP> --user duzhong --put tools/ml --dest /tmp/ag_ws/tools_ml
#   python tools/remote_run.py --host <IP> --user duzhong --script tools/ml/car_check_models.sh

MODEL_DIR="${MODEL_DIR:-$HOME/dzrobot/src/rknn_pt/model}"
CHECKER="${CHECKER:-/tmp/ag_ws/tools_ml/verify_contract.py}"

echo "==================== 车端模型契约扫描（只读） ===================="
date
echo "模型目录: $MODEL_DIR"
echo "校验器  : $CHECKER"
echo

if [ ! -f "$CHECKER" ]; then
  echo "!! 先把 tools/ml 上传到 /tmp/ag_ws/tools_ml"
  exit 1
fi

PASS=0; FAIL=0
for f in "$MODEL_DIR"/*.rknn; do
  [ -e "$f" ] || continue
  echo "================================================================"
  echo "## $(basename "$f")  ($(du -h "$f" | cut -f1), md5 $(md5sum "$f" | cut -c1-8))"
  if python3 "$CHECKER" --model "$f" --nc 2 --nkpt 1 --kpt-dim 3; then
    PASS=$((PASS+1))
  else
    FAIL=$((FAIL+1))
  fi
  echo
done

echo "==================== 汇总 ===================="
echo "  符合 [1, 3, 640, 640] -> [1, 9, 8400] 契约: $PASS 个"
echo "  不符合（多半是旧版多输出模型）: $FAIL 个"
echo "  结论：只有 9 通道单输出的模型才能直接给现在这版 det_node 用。"
echo "==================== 结束 ===================="
