#!/usr/bin/env bash
# =============================================================================
# Orin 上「一条命令复现视频侦察 → 路径规划」（VNC / 桌面双击或终端直接跑）
#
# 用法（机上，仓库根目录或任意位置）：
#     bash tools/orin_demo_recon.sh              # 有已存态势就秒级复现，没有就提示怎么跑全量
#     bash tools/orin_demo_recon.sh --full       # 从视频全量重跑（stride 5，约 20 分钟，后台执行）
#     STATE=/tmp/xx/field_state.json bash tools/orin_demo_recon.sh   # 指定其它态势
#
# 为什么要这个脚本：整条链有 4 个前置件（Python cv2 / 13 类模型 / 中文字体 /
# 已存态势），少一个都只在最后一步报错。这里把它们一次查完并说清修法，
# 现场（或演示时）不需要记任何参数。
# =============================================================================
set -u
REPO="${REPO:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$REPO" || { echo "[失败] 找不到仓库根目录: $REPO"; exit 9; }
R="src/ag_integrate/ag_integrate/video_recon.py"
STATE="${STATE:-/tmp/ag_recon_bg/field_state.json}"

say()  { printf '\n\e[1;36m==> %s\e[0m\n' "$*"; }
ok()   { printf '\e[1;32m[通过]\e[0m %s\n' "$*"; }
warn() { printf '\e[1;33m[警告]\e[0m %s\n' "$*"; }
bad()  { printf '\e[1;31m[失败]\e[0m %s\n' "$*"; }

# ---------------------------------------------------------------- 前置检查
say "1/4 前置检查（少了任何一项都会在这里说清修法）"
PY=python3
if $PY -c 'import cv2' 2>/dev/null; then
    ok "cv2 $($PY -c 'import cv2;print(cv2.__version__)') 可用"
else
    bad "cv2 不可用 —— 跑一次：bash tools/orin_fix_pyvision.sh --smoke"
    exit 1
fi
if $PY -c 'from PIL import Image' 2>/dev/null; then
    ok "Pillow 可用（渲染 route_render.png）"
else
    warn "缺 Pillow：pip3 install --user pillow（仅影响渲染图）"
fi
FONT=$(ls /usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc 2>/dev/null || true)
[[ -n $FONT ]] && ok "中文字体在位（$FONT）" || warn "无中文字体：渲染图会退化为英文标签"

$PY - "$REPO" <<'PY'
import sys
sys.path.insert(0, sys.argv[1] + "/src/ag_integrate")
from ag_integrate import video_recon as vr
print(f"[信息] 模型搜索顺序: {vr.D13_SEARCH[0]} …")
PY
MODEL=$(ls "$HOME"/dzuav/models/hbb_detector_d13*.onnx 2>/dev/null | head -1 || true)
[[ -n $MODEL ]] && ok "13 类 d13 模型在位（$MODEL）" \
                || warn "~/dzuav/models 下没找到 hbb_detector_d13*.onnx，默认定位会失败，用 --d13 指定"

# ---------------------------------------------------------------- 秒级复现
say "2/4 复现路径规划（不重跑识别，秒级）"
if [[ ! -f $STATE ]]; then
    warn "没有已存态势：$STATE"
    echo "     先跑一次完整侦察（约 20 分钟）：bash tools/orin_demo_recon.sh --full"
    echo "     或指定其它态势：STATE=/path/field_state.json bash tools/orin_demo_recon.sh"
    exit 0
fi
OUTDIR=$(dirname "$STATE")
echo "     态势: $STATE"
$PY tools/orin_replan_from_state.py "$STATE" "$OUTDIR"
rc=$?
[[ $rc -eq 0 ]] && ok "规划完成" || { bad "规划失败（rc=$rc）"; exit "$rc"; }

# ---------------------------------------------------------------- 产物
say "3/4 产物（VNC 里可直接看）"
for f in route_plan.json route_render.png field_state.json; do
    [[ -f $OUTDIR/$f ]] && printf '     %-20s %s\n' "$f" "$(ls -lh "$OUTDIR/$f" | awk '{print $5}')"
done
echo "     看图: eog $OUTDIR/route_render.png   （或 xdg-open $OUTDIR/route_render.png）"
echo "     看命令: $PY -c \"import json;d=json.load(open('$OUTDIR/route_plan.json'));print(d['car_cmds'])\""

# ---------------------------------------------------------------- 全量重跑
if [[ "${1:-}" == "--full" ]]; then
    say "4/4 从视频全量重跑（stride 5 / 约 20 分钟，后台执行）"
    VIDEO=$(ls -t "$HOME"/Videos/recordings/*.mp4 2>/dev/null | head -1 || true)
    [[ -n $VIDEO ]] || { bad "~/Videos/recordings 下没有 mp4"; exit 1; }
    echo "     视频: $VIDEO"
    MAXF=0 STRIDE=5 bash tools/orin_recon_bg.sh
    echo "     看进度: tail -f /tmp/ag_recon_bg.log"
    echo "     跑完后重跑本脚本（不带 --full）即从新态势出规划"
else
    say "4/4 需要重算识别时"
    echo "     全量（推荐，19 分钟）: bash tools/orin_demo_recon.sh --full"
    echo "     快速冒烟（60 帧，只看前 35% 时长，会漏目标）:"
    echo "         MAXF=60 STRIDE=15 bash tools/orin_recon_bg.sh"
fi
