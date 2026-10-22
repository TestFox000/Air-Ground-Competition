#!/usr/bin/env bash
# =============================================================================
# 通用视频侦察入口：给任意视频 → 识别（13 类）→ 格号反推 → 路线规划 → 渲染图
#
# 用法（机上任意位置）：
#     tools/orin_video_recon.sh                       # 列出机上视频，输入序号选一个
#     tools/orin_video_recon.sh <视频路径>             # 直接指定视频（mp4/mkv/avi…）
#     tools/orin_video_recon.sh <视频> --stride 5      # 想快：采样帧距（默认 1=逐帧最全）
#     tools/orin_video_recon.sh <视频> --force         # 不用缓存态势，强制重跑识别
#     tools/orin_video_recon.sh <视频> --replay-safe   # 静态回放：不排射击停靠（无 '*')
#     tools/orin_video_recon.sh --replay-last          # 秒级：用最近一次的态势重出规划
#
# 缓存语义（这是"通用"的关键）：按 <视频名>-<大小>-<时间戳>-s<stride> 建缓存目录，
# 同一视频同一参数跑第二次时复用**已落盘的逐帧识别结果**（frames_meta.json），
# 用**当前**门控/裁决规则重算态势并重出规划（秒级）——门控修过的地方不会因为
# 缓存而继续用旧结论。换视频/换 stride 自然是新目录，不会串味。--force 忽略缓存。
#
# 关于耗时与质量：默认 stride=1 **逐帧**推理，约 2.2 秒/帧（2220 帧≈80 分钟）——
# 测试环境不拿采样换时间。调大 stride 就是采样漏检，不是"快一点的同结果"：
# 同一段视频实测 stride 15 只剩 3 雷、stride 5 出 8 雷（=官方基线 8）。
# =============================================================================
set -u
REPO="${REPO:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$REPO" || { echo "[失败] 找不到仓库: $REPO"; exit 9; }
PY=python3
R="src/ag_integrate/ag_integrate/video_recon.py"
CACHE_ROOT="${CACHE_ROOT:-/tmp/ag_recon}"
VIDEO_DIR="${VIDEO_DIR:-$HOME/Videos/recordings}"

say()  { printf '\n\e[1;36m==> %s\e[0m\n' "$*"; }
ok()   { printf '\e[1;32m[通过]\e[0m %s\n' "$*"; }
warn() { printf '\e[1;33m[警告]\e[0m %s\n' "$*"; }
bad()  { printf '\e[1;31m[失败]\e[0m %s\n' "$*"; }

# 秒级复用：识别结果（frames_meta.json）已在，用当前门控重算态势 + 重出规划。
# 这是"门控改了但缓存没重跑"的唯一正确做法——与重跑识别共用同一份聚合代码。
reagg() {   # $1 = 缓存目录
    if [[ -f "$1/frames_meta.json" ]]; then
        $PY -u "$R" --reaggregate "$1" "${EXTRA[@]}"
    else
        warn "该缓存缺 frames_meta.json（旧版本产物），退回只重跑规划层——"
        warn "态势仍是旧识别结论；要套用新门控请加 --force 重跑一次识别"
        [[ ${#EXTRA[@]} -gt 0 ]] && warn "旧重跑器不支持 ${EXTRA[*]}，本次忽略"
        $PY tools/orin_replan_from_state.py "$1/field_state.json" "$1"
    fi
}

STRIDE=1; FORCE=0; VIDEO=""; OUT=""; REPLAY_LAST=0; REPLAY_SAFE=0; EXTRA=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --stride) STRIDE="${2:-1}"; shift 2 ;;
        --force)  FORCE=1; shift ;;
        --out)    OUT="${2:-}"; shift 2 ;;
        --replay-safe) REPLAY_SAFE=1; EXTRA+=(--replay-safe); shift ;;
        --replay-last) REPLAY_LAST=1; shift ;;
        -h|--help) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) VIDEO="$1"; shift ;;
    esac
done

pick_video() {
    local files=()
    while IFS= read -r f; do files+=("$f"); done < <(
        find "$VIDEO_DIR" "$HOME" -maxdepth 2 -type f \
             \( -name '*.mp4' -o -name '*.mkv' -o -name '*.avi' -o -name '*.mov' \) \
             -size +100k -printf '%T@ %p\n' 2>/dev/null \
        | sort -rn | head -12 | cut -d' ' -f2-)
    [[ ${#files[@]} -gt 0 ]] || { bad "没找到视频（$VIDEO_DIR 下没有 >100k 的 mp4/mkv），直接给路径：$0 <视频>"; return 1; }
    echo "可用视频（新的在前）："
    local i=1
    for f in "${files[@]}"; do
        printf '  %2d) %-58s %s\n' "$i" "$(basename "$f")" "$(du -h "$f" | cut -f1)"
        i=$((i + 1))
    done
    [[ -t 0 ]] || { bad "非交互环境：请直接给视频路径（$0 <视频>）"; return 1; }
    local n; read -r -p "输入序号（或直接粘贴路径）: " n
    if [[ "$n" =~ ^[0-9]+$ ]] && (( n >= 1 && n <= ${#files[@]} )); then
        VIDEO="${files[$((n - 1))]}"
    else
        VIDEO="$n"
    fi
}

# ---------------------------------------------------------------- 前置检查
say "前置检查"
$PY -c 'import cv2' 2>/dev/null && ok "cv2 $($PY -c 'import cv2;print(cv2.__version__)')" \
    || { bad "cv2 不可用 → 先跑 bash tools/orin_fix_pyvision.sh --smoke"; exit 1; }
ls "$HOME"/dzuav/models/hbb_detector_d13*.onnx >/dev/null 2>&1 \
    && ok "13 类 d13 模型在位" \
    || warn "~/dzuav/models 下没找到 hbb_detector_d13*.onnx（可用 --d13 指定，见 video_recon.py --help）"

# ---------------------------------------------------------------- --replay-last
if [[ $REPLAY_LAST == 1 ]]; then
    LAST=$(ls -dt "$CACHE_ROOT"/*/ 2>/dev/null | while read -r d; do
               [[ -f "$d/field_state.json" ]] && { echo "${d%/}"; break; }; done)
    [[ -n $LAST ]] || { bad "没有缓存态势（$CACHE_ROOT 下没有 field_state.json）——先跑一次：$0 <视频>"; exit 1; }
    OUT="$LAST"                        # 后面摘要/看图都以 OUT 为准，这里必须补上
    say "秒级重出规划（缓存态势：$LAST）"
    reagg "$LAST" || exit $?
    PNG="$LAST/route_render.png"
else
    [[ -n $VIDEO ]] || { pick_video || exit 1; }
    if [[ ! -f $VIDEO ]]; then
        # 允许只给文件名（自动在录像目录里找）
        CAND=$(ls "$VIDEO_DIR/$VIDEO" 2>/dev/null || true)
        [[ -n $CAND ]] && VIDEO="$CAND" || { bad "视频不存在: $VIDEO"; exit 1; }
    fi
    SZ=$(stat -c%s "$VIDEO"); MT=$(stat -c%Y "$VIDEO")
    KEY="$(basename "${VIDEO%.*}")-${SZ}-${MT}-s${STRIDE}"
    OUT="${OUT:-$CACHE_ROOT/$KEY}"
    say "视频: $VIDEO"
    echo "     采样帧距: $STRIDE    缓存目录: $OUT"
    if [[ -f "$OUT/field_state.json" && $FORCE == 0 ]]; then
        ok "该视频已有识别结果（同尺寸/同帧距）——直接用当前门控重算态势（秒级）"
        echo "     要重跑逐帧识别: 加 --force"
        reagg "$OUT" || exit $?
    else
        [[ $FORCE == 1 ]] && warn "--force：忽略缓存，重跑识别"
        echo "     开始识别（默认逐帧最全；每 20 帧打印一次进度，预计时间见下一行）"
        echo "     想中途离开又不丢进度：另开终端跑，或 Ctrl+Z 后 bg（本脚本不写死后台）"
        mkdir -p "$OUT"
        $PY -u "$R" "$VIDEO" --out "$OUT" --stride "$STRIDE" "${EXTRA[@]}" || {
            bad "识别/规划失败（看上方 [失败] 原因）"; exit 1; }
    fi
    PNG="$OUT/route_render.png"
fi

# ---------------------------------------------------------------- 结果摘要
say "结果"
$PY - "$OUT" <<'PY'
import json, sys
from pathlib import Path
out = Path(sys.argv[1])
st = json.loads((out / "field_state.json").read_text(encoding="utf-8"))
pl = json.loads((out / "route_plan.json").read_text(encoding="utf-8"))
kv = st.get("frames_grid_ok"), st.get("frames_sampled")
n_mines = len(st["mines_video"])
print(f"  网格拟合帧 {kv[0]}/{kv[1]}   雷 {st['mines_video']}"
      f"（{n_mines}/8 颗，官方基线 8）{'' if n_mines >= 8 else '  ⚠ 可能漏检'}")
print(f"  K列箭头 {st['passable_video']}   障碍 {st['obstacles_video']}   靶 {st['targets_video']}")
if st.get("targets_reclassified_as_bucket"):
    print(f"  靶裁决: 静靶→桶 {st['targets_reclassified_as_bucket']}（场地真值口径，"
          f"要保留静靶用 --allow-static-targets）")
print(f"  路线 {pl['route_len_cells']} 格 / 压缩 {len(pl['compact'])} 点 / 转向 {pl['turns']} 次")
print(f"  车命令: {pl['car_cmds']['route'][:110]}{'…' if len(pl['car_cmds']['route'])>110 else ''}")
print(f"  射击停靠: {pl['car_cmds']['shoot_cells']}   {pl['check']}")
PY
echo "     产物: $OUT/{field_state,route_plan,frames_meta}.json + route_render.png"

if [[ -f $PNG ]]; then
    if [[ -n "${DISPLAY:-}" ]] && command -v eog >/dev/null; then
        echo "     打开渲染图……（关掉看图窗口即回到终端）"
        eog "$PNG" 2>/dev/null || echo "     看图: $PNG"
    else
        echo "     看图: $PNG   （或在 VNC 桌面里双击它）"
    fi
fi
