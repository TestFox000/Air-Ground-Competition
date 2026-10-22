#!/usr/bin/env bash
# 机上验收（短版，限 2 分钟）：报错路径 + 契约自检 + 后台短跑启动（2026-10-21）
set -u
cd "$HOME/dzuav_deploy/air_ground" || exit 9
R=src/ag_integrate/ag_integrate/video_recon.py
echo "===== 1) 视频不存在：期望 rc=2 ====="
python3 "$R" /tmp/nope.mp4 >/tmp/v1.out 2>&1
echo "RC1=$? OUT=$(tail -2 /tmp/v1.out | head -1)"
echo "===== 2) --debug：期望真堆栈 ====="
python3 "$R" /tmp/nope.mp4 --debug >/tmp/v2.out 2>&1
echo "TRACE=$(grep -c Traceback /tmp/v2.out) FNF=$(grep -c FileNotFoundError /tmp/v2.out)"
echo "===== 3) 7 类旧模型：期望拒绝（11 通道） ====="
python3 "$R" "$(ls -t "$HOME"/Videos/recordings/*.mp4 | head -1)" \
    --max-frames 1 --d13 "$HOME/dzuav/models/hbb_detector_7cls_direct_20261015.onnx" \
    >/tmp/v3.out 2>&1
echo "RC3=$? HIT=$(grep -c '11 通道' /tmp/v3.out)"
echo "===== 4) 后台短跑启动（无模型参数） ====="
bash tools/orin_recon_bg.sh
