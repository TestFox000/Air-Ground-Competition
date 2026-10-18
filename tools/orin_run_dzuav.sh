#!/usr/bin/env bash
# Orin 上启动 dzuav 侦察栈（相机实时 / 离线图片 / 离线视频）
# 用法：
#   bash tools/orin_run_dzuav.sh camera              # USB 相机实时（默认 /dev/video0, 1920x1080@30）
#   bash tools/orin_run_dzuav.sh camera /dev/video1  # 指定相机设备
#   bash tools/orin_run_dzuav.sh image <图片路径>     # 离线图片推理
#   bash tools/orin_run_dzuav.sh video <视频路径>     # 离线视频推理
#
# 引擎默认用 ~/dzuav/engines/，UDP 端口默认 50000。

set -euo pipefail

DZUAV_DIR="${DZUAV_DIR:-$HOME/ag_test/dzuav}"   # 我们的编译目录（不碰原 ~/dzuav）
ENGINES="${ENGINES:-$HOME/dzuav/engines}"        # 原机引擎目录
HBB_ENGINE="${HBB_ENGINE:-$ENGINES/hbb_detector_fp16.engine}"
CELL_ENGINE="${CELL_ENGINE:-$ENGINES/cell_classifier_fp16.engine}"
UDP_PORT="${UDP_PORT:-50000}"

BIN="$DZUAV_DIR/build/dzuav"

if [ ! -f "$BIN" ]; then
    echo "[错误] 找不到 dzuav 二进制: $BIN"
    echo "       请先编译：cd $DZUAV_DIR && mkdir -p build && cd build && cmake .. && make -j\$(nproc)"
    exit 1
fi

if [ ! -f "$HBB_ENGINE" ] || [ ! -f "$CELL_ENGINE" ]; then
    echo "[错误] 找不到引擎文件"
    echo "  HBB:  $HBB_ENGINE"
    echo "  CELL: $CELL_ENGINE"
    exit 1
fi

mode="${1:-camera}"

case "$mode" in
    camera)
        dev="${2:-/dev/video0}"
        size_fps="${3:-1920x1080@30}"
        echo "[启动] USB 相机实时: $dev, $size_fps, UDP:$UDP_PORT"
        echo "       Ctrl+C 停止"
        exec "$BIN" "uvc:$dev" "$HBB_ENGINE" "$CELL_ENGINE" "$UDP_PORT" "$size_fps"
        ;;
    image)
        img="${2:-}"
        if [ -z "$img" ]; then
            echo "用法: $0 image <图片路径>"
            exit 1
        fi
        if [ ! -f "$img" ]; then
            echo "[错误] 图片不存在: $img"
            exit 1
        fi
        echo "[推理] 图片: $img"
        exec "$BIN" "$img" "$HBB_ENGINE" "$CELL_ENGINE" "$UDP_PORT"
        ;;
    video)
        vid="${2:-}"
        if [ -z "$vid" ]; then
            echo "用法: $0 video <视频路径>"
            exit 1
        fi
        if [ ! -f "$vid" ]; then
            echo "[错误] 视频不存在: $vid"
            exit 1
        fi
        echo "[推理] 视频: $vid"
        exec "$BIN" "$vid" "$HBB_ENGINE" "$CELL_ENGINE" "$UDP_PORT"
        ;;
    *)
        echo "用法: $0 {camera|image|video} [参数]"
        echo "  camera [设备] [WxH@FPS]   USB 相机实时（默认 /dev/video0 1920x1080@30）"
        echo "  image <图片路径>            离线图片推理"
        echo "  video <视频路径>            离线视频推理"
        exit 1
        ;;
esac
