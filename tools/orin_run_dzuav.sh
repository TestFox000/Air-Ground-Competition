#!/usr/bin/env bash
# Orin 上启动 dzuav 侦察栈（相机实时 / 离线图片 / 离线视频）
# 用法：
#   bash tools/orin_run_dzuav.sh camera              # USB 相机实时（默认 /dev/video0, 1920x1080@30）
#   bash tools/orin_run_dzuav.sh camera /dev/video1  # 指定相机设备
#   bash tools/orin_run_dzuav.sh image <图片路径>     # 离线图片推理
#   bash tools/orin_run_dzuav.sh video <视频路径>     # 离线视频推理
#
# 默认值 = canonical（13 类部署线），不要再改回 7 类：
#   二进制   <仓库>/onboard/dzuav/build/dzuav   （orin_deploy --verify 编译产物）
#   检测引擎 hbb_detector_d13_fp16.engine       （13 类 D13）
#   格号引擎 cell_classifier_v4d_fp16.engine    （v4d）
# ⚠️ 历史坑（2026-10-20 查出）：这里曾默认 $HOME/ag_test/dzuav + hbb_detector_fp16
#    + cell_classifier_fp16，于是一键启动拉起的是 9 月的 7 类老二进制 + 7 类引擎，
#    输出类别索引与 13 类协议错位（障碍被当蓝靶），而界面上看不出任何异常。
#    可用 DZUAV_DIR / HBB_ENGINE / CELL_ENGINE 环境变量显式覆盖。

set -euo pipefail

_REPO="$(cd "$(dirname "$0")/.." && pwd)"
DZUAV_DIR="${DZUAV_DIR:-$_REPO/onboard/dzuav}"    # 仓库内编译目录（部署即用）
ENGINES="${ENGINES:-$HOME/dzuav/engines}"          # 机上引擎目录
HBB_ENGINE="${HBB_ENGINE:-$ENGINES/hbb_detector_d13_fp16.engine}"
CELL_ENGINE="${CELL_ENGINE:-$ENGINES/cell_classifier_v4d_fp16.engine}"
UDP_PORT="${UDP_PORT:-50000}"

BIN="$DZUAV_DIR/build/dzuav"

if [ ! -f "$BIN" ]; then
    echo "[错误] 找不到 dzuav 二进制: $BIN"
    echo "       请先编译：bash tools/orin_build_verify.sh（或 orin_deploy.py --verify）"
    exit 1
fi

if [ ! -f "$HBB_ENGINE" ] || [ ! -f "$CELL_ENGINE" ]; then
    echo "[错误] 找不到引擎文件"
    echo "  HBB:  $HBB_ENGINE"
    echo "  CELL: $CELL_ENGINE"
    echo "  本脚本默认走 13 类 canonical 引擎（d13 + v4d）；换机/刷机后需先生成。"
    exit 1
fi

# 把实际使用的三件套亮出来：VNC 上看一眼就能确认没跑错版本
echo "[配置] bin    = $BIN"
echo "[配置] hbb    = $(basename "$HBB_ENGINE")"
echo "[配置] cell   = $(basename "$CELL_ENGINE")"
echo "[配置] udp    = $UDP_PORT"

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
