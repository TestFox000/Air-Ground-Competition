#!/usr/bin/env bash
# D13 13类检测 + v4d 格号引擎 侦察栈（MJPG UVC 修复版）
# 用法: orin_run_dzuav_d13.sh camera [dev] [WxH@FPS]
#       orin_run_dzuav_d13.sh image <img> [outdir]
#       orin_run_dzuav_d13.sh video <mp4> [outdir]
set -eu
BIN=$HOME/dzuav_deploy/air_ground/onboard/dzuav_mjpg/build/dzuav
ENGINES=$HOME/dzuav/engines
HBB=$ENGINES/hbb_detector_d13_fp16.engine
CELL=$ENGINES/cell_classifier_v4d_fp16.engine
PORT=50000
mode=camera
[ $# -ge 1 ] && mode=$1
case "$mode" in
 camera)
   dev=/dev/video0; [ $# -ge 2 ] && dev=$2
   spec=1280x720@30; [ $# -ge 3 ] && spec=$3
   exec "$BIN" "uvc:$dev" "$HBB" "$CELL" "$PORT" "$spec" ;;
 image)
   img=$2; out=""; [ $# -ge 3 ] && out=$3
   if [ -n "$out" ]; then exec "$BIN" "$img" "$HBB" "$CELL" "$PORT" "$out"; else exec "$BIN" "$img" "$HBB" "$CELL" "$PORT"; fi ;;
 video)
   vid=$2; out=""; [ $# -ge 3 ] && out=$3
   if [ -n "$out" ]; then exec "$BIN" "$vid" "$HBB" "$CELL" "$PORT" "$out"; else exec "$BIN" "$vid" "$HBB" "$CELL" "$PORT"; fi ;;
 *) echo "usage: $0 {camera|image|video} [arg] [spec|outdir]"; exit 1 ;;
esac
