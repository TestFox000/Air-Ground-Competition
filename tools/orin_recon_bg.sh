#!/usr/bin/env bash
# 机上后台短跑：链路易断，用 nohup 脱手 + 日志轮询（2026-10-21）
set -u
cd "$HOME/dzuav_deploy/air_ground" || exit 9
OUT=/tmp/ag_recon_bg
LOG=/tmp/ag_recon_bg.log
rm -rf "$OUT" "$LOG"
VIDEO=${VIDEO:-$(ls -t "$HOME"/Videos/recordings/*.mp4 | head -1)}
MAXF=${MAXF:-60}
STRIDE=${STRIDE:-15}
nohup python3 -u src/ag_integrate/ag_integrate/video_recon.py "$VIDEO" \
    --out "$OUT" --stride "$STRIDE" --max-frames "$MAXF" \
    > "$LOG" 2>&1 &
echo "PID=$! VID=$VIDEO MAXF=$MAXF STRIDE=$STRIDE LOG=$LOG"
