#!/bin/bash
# ============================================================================
# cam_shot.sh —— 「真的出图了吗」的硬证据（第二步，本包的核心判据）
#
# 做两件事：
#   1) 抓 1 帧写成 JPEG（存在则说明 UVC 取流链路是通的）
#   2) 再抓 1 帧缩成 8x8 灰度，算 min/max/avg —— 区分「有流但全黑」和「真的出画」
#
# 只读承诺：只在 /tmp 下写这一个 JPEG 和一个 .gray 临时文件，不碰系统其它任何东西。
#
# 用法：
#   bash cam_shot.sh                        # 自动选设备，输出 /tmp/cam_shot.jpg
#   bash cam_shot.sh /dev/video0 /tmp/a.jpg
#
# 退出码：0 正常 / 2 无设备 / 3 画面全黑 / 4 全白 / 5 无可用格式 / 6 取流失败
# ============================================================================
set +u
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=cam_lib.sh
. "$DIR/cam_lib.sh"

DEV=""
OUT=""
for a in "$@"; do
  case "$a" in
    /dev/video*) DEV="$a" ;;
    *)           OUT="$a" ;;
  esac
done
OUT="${OUT:-/tmp/cam_shot.jpg}"

if ! cam_have gst-launch-1.0; then
  echo "ERROR: 机上没有 gst-launch-1.0，本脚本无法工作（可改用 ROS 路线: cam_up.sh --ros）"
  exit 6
fi

# ---- 1. 选设备 --------------------------------------------------------------
if [ -z "$DEV" ]; then
  echo "--- 自动选设备（跳过 metadata 节点，逐个试格式阶梯）---"
  DEV="$(cam_first_working_dev)" || {
    echo "RESULT: 没有任何 /dev/video* 能取到流。"
    echo "        先跑 cam_env.sh 看节点是否存在；节点存在但取不到流 = 格式/权限/驱动问题。"
    exit 2
  }
  echo "选中: $DEV  ($(cam_node_name "$DEV"))"
fi
[ -e "$DEV" ] || { echo "ERROR: $DEV 不存在"; exit 2; }

# ---- 2. 选格式 --------------------------------------------------------------
echo
echo "--- 探测可用格式 ---"
CAPSINFO="$(cam_probe_format "$DEV")" || {
  echo "RESULT: $DEV 存在但所有候选格式都取不到流。"
  echo "        排查：权限(是否在 video 组) / 是否被别的进程占用(pgrep -af v4l2) / 相机是否要外供电源"
  exit 5
}
IFS='|' read -r KIND CAPS W H FPS <<EOF
$CAPSINFO
EOF
echo "可用格式: kind=$KIND  caps=${CAPS:-<自动协商>}  ${W}x${H}@${FPS}"

# ---- 3. 抓帧 ----------------------------------------------------------------
echo
echo "--- 连抓 8 帧、留最后一帧 -> $OUT （等自动曝光收敛，单帧容易偏暗）---"
if cam_grab_jpeg "$DEV" "$OUT" "$KIND" "$CAPS" 8; then
  SZ="$(stat -c%s "$OUT" 2>/dev/null)"
  echo "OK: $(ls -l "$OUT")"
  echo "    大小 ${SZ} 字节，JPEG 头 FFD8FF 校验通过"
  cam_have file && file "$OUT"
  cam_have identify && identify "$OUT" 2>/dev/null
else
  echo "RESULT: 取流成功但落盘失败/不是合法 JPEG —— 很可能是编码链问题，把上面 kind 记下来。"
  exit 6
fi

# ---- 4. 亮度统计（全黑/全白 vs 正常）---------------------------------------
echo
echo "--- 亮度统计（8x8 GRAY8 取样，同样取收敛后的最后一帧）---"
THUMB="${OUT%.jpg}.gray"
RC=0   # 默认成功：JPEG 已经落盘就是硬证据，亮度取样失败不该把结论翻成失败
if cam_grab_gray "$DEV" "$KIND" "$CAPS" 8 "$THUMB"; then
  STATS="$(od -An -tu1 -v "$THUMB" | awk '
    { for (i=1;i<=NF;i++) { v=$i; if (n==0 || v<mn) mn=v; if (v>mx) mx=v; s+=v; n++ } }
    END { if (n>0) printf "%d %d %.1f %d", mn, mx, s/n, n }')"
  set -- $STATS
  MN="$1"; MX="$2"; AVG="$3"; NPX="$4"
  echo "min=$MN  max=$MX  avg=$AVG  取样像素=$NPX"
  if [ "${MX:-0}" -le 8 ]; then
    echo "判定: **全黑** —— 有数据流，但画面没有光。查：镜头盖/光圈、曝光参数、"
    echo "      以及这台相机是不是需要在别的分辨率下才有画面（换 cam_shot.sh 的阶梯）。"
    RC=3
  elif [ "${MN:-0}" -ge 247 ]; then
    echo "判定: **全白** —— 严重过曝或信号异常（也可能镜头正对强光）。"
    RC=4
  else
    echo "判定: **正常** —— 画面存在明暗层次，相机确实在工作。"
    RC=0
  fi
else
  echo "(取样失败，不阻塞前面的 JPEG 结论)"
fi

echo
echo "=== 结论 ==="
echo "设备: $DEV   格式: ${W}x${H}@${FPS} kind=$KIND"
echo "图片: $OUT   （把这个文件拷回 Windows 打开，或用 read_image 让我直接看）"
exit $RC
