#!/usr/bin/env bash
# =============================================================================
# 通用视频侦察入口：给任意视频 → 识别（13 类）→ 格号反推 → 路线规划 → 渲染图
#
# 用法（机上任意位置）：
#     tools/orin_video_recon.sh                       # 列出机上视频，输入序号选一个
#     tools/orin_video_recon.sh <视频路径>             # 直接指定视频（mp4/mkv/avi…）
#     tools/orin_video_recon.sh <视频> --stride 15      # 更稀：漏检（实测只剩 3/8 雷）
#     tools/orin_video_recon.sh <视频> --force         # 不用缓存态势，强制重跑识别
#     tools/orin_video_recon.sh <视频> --replay-safe   # 静态回放：不排射击停靠（无 '*')
#     tools/orin_video_recon.sh --replay-last          # 秒级：用最近一次的态势重出规划
#
# 缓存语义（这是"通用"的关键）：按 <视频名>-<大小>-<时间戳>-s<stride> 建缓存目录，
# 同一视频同一参数跑第二次时复用**已落盘的逐帧识别结果**（frames_meta.json），
# 用**当前**门控/裁决规则重算态势并重出规划（秒级）——门控修过的地方不会因为
# 缓存而继续用旧结论。换视频/换 stride 自然是新目录，不会串味。--force 忽略缓存。
#
# 关于采样与质量：默认 stride=5 是**实测最优点**（同一段视频 8/8=官方基线），
# 不是"越密越好"也不是"越快越好"：stride 15 漏检到 3 雷；stride 1 逐帧反而
# 14 雷（7 个多余，弱误检累积到 >=3 票后混入，幻影消除还会误杀真雷 E2）。
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

# 失败原因 → 可执行处理建议（日志由 tee 落盘，成功路径不打印这些）
analyze_log() {   # $1 = 日志路径
    local log="$1"
    echo "     完整日志: $log"
    if grep -q 'cv2 不可用' "$log" 2>/dev/null; then
        echo "     原因: Python cv2 环境缺失/损坏"
        echo "     处理: bash tools/orin_fix_pyvision.sh --smoke"
    elif grep -qE '未找到.*(d13|v4d|模型)|模型.*(缺失|找不到)' "$log" 2>/dev/null; then
        echo "     原因: 模型缺失或路径不对"
        echo "     处理: ls ~/dzuav/models/ 确认 d13+v4d；或用 --d13/--v4d 明确指定"
    elif grep -qE '视频不存在|打不开视频|无法解码|0 字节' "$log" 2>/dev/null; then
        echo "     原因: 视频路径/编码问题"
        echo "     处理: ls -l <视频> 看大小；ffprobe <视频> 看编码；转 h264 mp4 再试"
    elif grep -q '没有缓存' "$log" 2>/dev/null; then
        echo "     原因: 该视频还没跑过识别，--replay-last 无缓存可复用"
        echo "     处理: 去掉 --replay-last，先跑一次完整侦察"
    elif grep -q '未预期的记号' "$log" 2>/dev/null; then
        echo "     原因: 脚本在运行中被覆盖（bash 按字节偏移读脚本）"
        echo "     处理: 重新部署脚本后再跑；部署长跑脚本要用「上传 .new → mv」原子替换"
    elif grep -qE 'MemoryError|Killed' "$log" 2>/dev/null; then
        echo "     原因: 内存/CPU 不足（逐帧 stride=1 代价高）"
        echo "     处理: 用默认 stride 5，或关掉其它大进程再跑"
    else
        echo "     处理: 把上面的 [失败] 与日志一起发出；或加 --debug 看堆栈"
    fi
}

# 秒级复用：识别结果（frames_meta.json）已在，用当前门控重算态势 + 重出规划。
# 这是"门控改了但缓存没重跑"的唯一正确做法——与重跑识别共用同一份聚合代码。
reagg() {   # $1 = 缓存目录
    local dir="$1" log rc=0
    log="$dir/reagg.log"
    if [[ -f "$dir/frames_meta.json" ]]; then
        $PY -u "$R" --reaggregate "$dir" "${EXTRA[@]}" 2>&1 | tee "$log"
        rc=${PIPESTATUS[0]}
        if [[ $rc -ne 0 ]]; then
            bad "重算失败（退出码 $rc）"
            analyze_log "$log"
            return "$rc"
        fi
    else
        warn "该缓存缺 frames_meta.json（旧版本产物），退回只重跑规划层——"
        warn "态势仍是旧识别结论；要套用新门控请加 --force 重跑一次识别"
        [[ ${#EXTRA[@]} -gt 0 ]] && warn "旧重跑器不支持 ${EXTRA[*]}，本次忽略"
        $PY tools/orin_replan_from_state.py "$dir/field_state.json" "$dir" || {
            bad "规划层重跑失败（看上方报错）"; return 1; }
    fi
}

STRIDE=5; FORCE=0; VIDEO=""; OUT=""; REPLAY_LAST=0; REPLAY_SAFE=0; EXTRA=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --stride) STRIDE="${2:-5}"; shift 2 ;;
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
# /tmp 爆满时 JSON 会写半截，表现为"产物损坏"，比直接报磁盘满更难查
FREE_MB=$(df -Pm "$CACHE_ROOT" 2>/dev/null | awk 'NR==2{print $4}')
if [[ -n ${FREE_MB:-} && $FREE_MB -lt 200 ]]; then
    warn "/tmp 仅剩 ${FREE_MB}MB（<200MB）——清理 /tmp/ag_recon 或旧日志后再跑"
fi

# 同一产物目录同时只允许一个进程（两个识别抢写会把产物写坏）
lock_dir() {
    command -v flock >/dev/null 2>&1 || return 0
    mkdir -p "$1" 2>/dev/null || true
    exec 9>"$1/.lock" 2>/dev/null || return 0
    flock -n 9 || {
        bad "同一产物目录已有进程在跑：$1"
        echo "     处理：等它结束，或用 --out 换一个目录"
        return 1
    }
}

# ---------------------------------------------------------------- --replay-last
if [[ $REPLAY_LAST == 1 ]]; then
    # "最近一次"取**正式结果**：优先 stride 5（实测最优点）。2026-10-22 实测：
    # 目录按 mtime 取最新会选到刚跑过的 stride1 反例（14 雷），路线直接错。
    pick_cache() {   # $1 = glob（如 '*-s5/'），取最新且带 field_state.json 的目录
        ls -dt "$CACHE_ROOT"/$1 2>/dev/null | while read -r d; do
            [[ -f "$d/field_state.json" ]] && { echo "${d%/}"; break; }
        done
    }
    LAST=$(pick_cache '*-s5/')
    SRC="stride5（实测最优）"
    if [[ -z ${LAST:-} ]]; then
        LAST=$(pick_cache '*/')
        SRC="最新一份（无 stride5 缓存）"
    fi
    [[ -n ${LAST:-} ]] || { bad "没有缓存态势（$CACHE_ROOT 下没有 field_state.json）——先跑一次：$0 <视频>"; exit 1; }
    OUT="$LAST"                        # 后面摘要/看图都以 OUT 为准，这里必须补上
    lock_dir "$OUT" || exit 1
    say "秒级重出规划（缓存态势：$LAST）"
    echo "     采用缓存: $SRC"
    reagg "$LAST" || exit $?
    PNG="$LAST/route_render.png"
else
    [[ -n $VIDEO ]] || { pick_video || exit 1; }
    if [[ ! -f $VIDEO ]]; then
        # 允许只给文件名（自动在录像目录里找）
        CAND=$(ls "$VIDEO_DIR/$VIDEO" 2>/dev/null || true)
        [[ -n $CAND ]] && VIDEO="$CAND" || {
            bad "视频不存在: $VIDEO"
            echo "     常见原因：文件名带空格/中文未加引号，或不在 $VIDEO_DIR"
            echo "     处理：ls '$VIDEO_DIR' 看实际文件名；或直接给绝对路径"
            exit 1; }
    fi
    [[ -s $VIDEO ]] || { bad "视频是空文件（0 字节）: $VIDEO"; echo "     处理：重新拷贝录像"; exit 1; }
    SZ=$(stat -c%s "$VIDEO"); MT=$(stat -c%Y "$VIDEO")
    KEY="$(basename "${VIDEO%.*}")-${SZ}-${MT}-s${STRIDE}"
    OUT="${OUT:-$CACHE_ROOT/$KEY}"
    lock_dir "$OUT" || exit 1
    say "视频: $VIDEO"
    echo "     采样帧距: $STRIDE    缓存目录: $OUT"
    if [[ -f "$OUT/field_state.json" && $FORCE == 0 ]]; then
        ok "该视频已有识别结果（同尺寸/同帧距）——直接用当前门控重算态势（秒级）"
        echo "     要重跑逐帧识别: 加 --force"
        reagg "$OUT" || exit $?
    else
        [[ $FORCE == 1 ]] && warn "--force：忽略缓存，重跑识别"
        echo "     开始识别（默认 stride 5；每 20 帧打印一次进度，预计时间见下一行）"
        echo "     想中途离开又不丢进度：另开终端跑，或 Ctrl+Z 后 bg（本脚本不写死后台）"
        mkdir -p "$OUT"
        LOG="$OUT/run.log"
        $PY -u "$R" "$VIDEO" --out "$OUT" --stride "$STRIDE" "${EXTRA[@]}" 2>&1 | tee "$LOG"
        RC=${PIPESTATUS[0]}
        if [[ $RC -ne 0 ]]; then
            bad "识别/规划失败（退出码 $RC）"
            analyze_log "$LOG"
            exit "$RC"
        fi
    fi
    PNG="$OUT/route_render.png"
fi

# ---------------------------------------------------------------- 结果摘要
say "结果"
# 摘要逻辑单独成 tools/orin_print_summary.py：shell 里嵌 Python heredoc 一旦
# 引号/转义错一个字符就整段崩（实测报「未预期的记号 "("」），且难复测。
$PY "$REPO/tools/orin_print_summary.py" "$OUT" || {
    bad "摘要生成失败：产物缺失或 JSON 损坏（目录 $OUT）"
    echo "     处理：加 --force 重跑一次识别，别用这份产物"
    exit 1
}
echo "     产物: $OUT/{field_state,route_plan,frames_meta}.json + route_render.png"

if [[ -f $PNG ]]; then
    if [[ -n "${DISPLAY:-}" ]] && command -v eog >/dev/null; then
        echo "     打开渲染图……（关掉看图窗口即回到终端）"
        eog "$PNG" 2>/dev/null || echo "     看图: $PNG"
    else
        echo "     看图: $PNG   （或在 VNC 桌面里双击它）"
    fi
fi
