#!/usr/bin/env bash
# =============================================================================
# L4 相机验收 —— 把 orin_build_verify.sh 够不到的那条路径验掉（在 Orin 上跑）
#
# 为什么单开一级：orin_build_verify.sh 的 L0~L3 全都不碰相机。
#   L2 是无参数运行看 usage，L3 是 `dzuav <图片> <hbb> <cell>` 走**离线图片**分支。
#   于是 main.cpp 里 runCamera() 那一整条路径——spec 解析、pipeline 三级回退、
#   协商尺寸对账、断流有界重开、SIGINT 优雅停机——**一次都没被验过**。
#   2026-10-20 复查发现的四条机载缺陷全在这条路径上，改完之后 L0~L3 照样全绿。
#   本脚本就是把这四条钉住的 L4，也是把「已写待验」升级成「已修」的唯一凭据。
#
# 分级（A~C 不需要相机，D~E 需要）:
#   A  前置：二进制在位且是 13 类；引擎按 canonical 名在位
#   B  退出码语义表：2=用法错误 / 3=打不开输入，逐条对齐 main.cpp 的 return
#   C  spec 严格性：坏 spec 必须 rc=2 拒绝，好 spec 必须原样回显
#   D  真相机实测：协商尺寸对账 + 真出帧计数 + 实测 fps 对 180s 硬时限的预算
#   E  断流重开演练（--drain，需手在 USB 线上）：拔线 -> 有界重开 -> 恢复或 rc=4
#
# 用法:
#   tools/orin_camera_smoke.sh                       # A~D（默认 /dev/video0 1920x1080@30）
#   tools/orin_camera_smoke.sh --no-camera           # 只跑 A~C（相机没接也能验 spec 严格性）
#   tools/orin_camera_smoke.sh --drain               # A~D 之后追加 E（提示拔插 USB）
#   tools/orin_camera_smoke.sh --dev /dev/video1 --spec 1280x720@30
#   tools/orin_camera_smoke.sh --seconds 40 --out /tmp/ag_evidence
#
# 环境变量:
#   DZUAV_DIR / ENGINES / HBB_ENGINE / CELL_ENGINE  同 orin_run_dzuav.sh（不另抄一份）
#   ALLOW_RESCALE=1   协商尺寸 != 请求尺寸时只警告不判失败（默认判失败：内参按帧
#                     分辨率缩放，尺寸一变定位就系统性偏移，而 L1~L3 都不碰内参）
#
# 证据: 全量日志 + 结论表落到 --out 目录（默认 /tmp/ag_camera_smoke_<时间戳>），
#       跑完用 tools/remote_run.py --get 拉回本地，随 进度/ 一起归档。
# =============================================================================

# ⚠️ 故意不加 -e：本脚本一半的断言就是「命令必须以非零退出」，set -e 会在
#    第一个**预期失败**上把整个验收脚本自己干掉（orin_bringup.sh 同理用 -uo）。
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
RUN_SH="$REPO/tools/orin_run_dzuav.sh"
DZUAV_DIR="${DZUAV_DIR:-$REPO/onboard/dzuav}"
BIN="$DZUAV_DIR/build/dzuav"

DEV="/dev/video0"
SPEC="1920x1080@30"
SECONDS_TO_RUN=25
DO_DRAIN=0
NO_CAMERA=0
OUT=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-camera) NO_CAMERA=1; shift ;;
        --drain)     DO_DRAIN=1; shift ;;
        --dev)       DEV="$2"; shift 2 ;;
        --spec)      SPEC="$2"; shift 2 ;;
        --seconds)   SECONDS_TO_RUN="$2"; shift 2 ;;
        --out)       OUT="$2"; shift 2 ;;
        -h|--help)
            awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0"
            exit 0 ;;
        *) echo "未知参数: $1（-h 查看用法）" >&2; exit 2 ;;
    esac
done

OUT="${OUT:-/tmp/ag_camera_smoke_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
LOG="$OUT/camera_smoke.log"      # 全量原始输出（证据）
REPORT="$OUT/camera_smoke.txt"   # 结论表（人看的）
: >"$LOG"; : >"$REPORT"

if [[ -t 1 ]]; then
    C_STEP=$'\e[1;36m'; C_OK=$'\e[1;32m'; C_WARN=$'\e[1;33m'
    C_ERR=$'\e[1;31m'; C_OFF=$'\e[0m'
else
    C_STEP=''; C_OK=''; C_WARN=''; C_ERR=''; C_OFF=''
fi

NPASS=0; NFAIL=0; NWARN=0; LAST_RC=0
step()  { printf '%s\n' "${C_STEP}==>${C_OFF} $*" | tee -a "$REPORT"; }
pass()  { printf '%s\n' "${C_OK}[通过]${C_OFF} $*" | tee -a "$REPORT"; NPASS=$((NPASS+1)); }
warn()  { printf '%s\n' "${C_WARN}[警告]${C_OFF} $*" | tee -a "$REPORT"; NWARN=$((NWARN+1)); }
fail()  { printf '%s\n' "${C_ERR}[失败]${C_OFF} $*" | tee -a "$REPORT"; NFAIL=$((NFAIL+1)); }
die()   { printf '%s\n' "${C_ERR}[中止]${C_OFF} $*" | tee -a "$REPORT" >&2; exit 1; }
note()  { printf '%s\n' "       $*" | tee -a "$REPORT"; }

# run_expect <标签> <期望rc(逗号分隔可多个)> <命令...>
# 把 stdout/stderr 一起追加进 $LOG（证据），rc 与期望集合比对。
# ⚠️ 进来第一件事就是 mark_log 打点：后面的 grep_log/count_log 只看**这一次**
#    的输出。不打点的话 tail_log 会从头读整个日志，于是"无参数那次打过的
#    usage:"会被后面每一次 grep_log "usage:" 命中 —— 断言看着在验，其实验的是
#    上一轮的残留，典型的假绿。
run_expect() {
    local label="$1" want="$2"; shift 2
    local rc tmp
    mark_log
    tmp=$(mktemp)
    echo "----- $label : $* -----" >>"$LOG"
    "$@" >>"$LOG" 2>"$tmp"
    rc=$?
    cat "$tmp" >>"$LOG"; rm -f "$tmp"
    echo "----- rc=$rc -----" >>"$LOG"
    if [[ ",$want," == *",$rc,"* ]]; then
        pass "$label（rc=$rc，期望 $want）"
    else
        fail "$label：rc=$rc，期望 $want（详见 $LOG）"
    fi
    LAST_RC=$rc
    return 0
}

# grep_log <模式> —— 在**本次**日志片段里找。片段边界由 mark_log 打点。
MARK_BYTES=0
mark_log() { MARK_BYTES=$(wc -c <"$LOG"); }
tail_log() { tail -c +"$((MARK_BYTES + 1))" "$LOG"; }
grep_log() { tail_log | grep -a -- "$1" >/dev/null 2>&1; }
count_log() { tail_log | grep -ac -- "$1" 2>/dev/null || true; }

# 占位引擎路径：C 级负例/正例都在 parseSpec 或 open camera 阶段就返回了，
# **根本走不到加载引擎**（main.cpp runCamera 的顺序是 parseSpec -> 候选 pipeline
# -> open -> 试读 -> 才 new HbbDetector）。用占位路径有两个好处：
#   ① 没装引擎也能验 spec 严格性（刚编译完就能跑，不必等 trtexec 转 40 分钟）
#   ② 它本身就是判据——若 parseSpec 不严格，rc 就不是 2 而是 3（打不开设备），
#      负例会当场红给你看。
FAKE_HBB="/nonexistent_ag_smoke/hbb.engine"
FAKE_CELL="/nonexistent_ag_smoke/cell.engine"
NODEV="uvc:/dev/nonexistent_ag_smoke"

# ===================================================================== A 前置
step "A 前置检查（二进制 / 13 类 / 引擎）"
[[ -x "$BIN" ]] || die "找不到 dzuav 二进制: $BIN —— 先 bash tools/orin_build_verify.sh"
pass "二进制在位: $BIN"

# 类别名表可直接在二进制里查：13 类含 uav_end/paper，7 类旧版含 mine/arrow。
# 2026-10-20 的真实事故形态就是「一键启动指向了 9 月的 7 类老二进制」，
# 界面上一切正常，只有类别索引与 13 类协议错位。
grep -qa uav_end "$BIN" || die "$BIN 不是 13 类（未见类别名 uav_end）—— 疑似 7 类旧版，跑起来会与 13 类协议错位（障碍被当蓝靶）"
grep -qa paper   "$BIN" || die "$BIN 未见 13 类类别名 paper"
pass "二进制为 13 类（含 uav_end / paper 类别名）"

# 引擎名不另抄一份：orin_run_dzuav.sh 是唯一真源（它的默认值同时被
# orin_convert_engines.sh 的产出名、orin_build_verify.sh 的 CANON_*_ENGINE、
# preflight.py 的 check_orin 共用，tests/test_tools_smoke.py 三方对锁）。
# 这里把它那三行赋值原文 eval 进来 —— 验收脚本与实跑口径不可能分叉。
if [[ ! -f "$RUN_SH" ]]; then
    die "缺少 $RUN_SH —— 引擎口径的唯一真源不在，拒绝用猜的引擎名出验收结论"
fi
eval "$(grep -aE '^(ENGINES|HBB_ENGINE|CELL_ENGINE)=' "$RUN_SH")"
[[ -n "${HBB_ENGINE:-}" && -n "${CELL_ENGINE:-}" ]] \
    || die "从 $RUN_SH 解析引擎默认值失败 —— 该脚本的变量定义行改了形状，本脚本的 eval 需要同步"
note "实跑口径: hbb=$(basename "$HBB_ENGINE")  cell=$(basename "$CELL_ENGINE")"
if [[ -f "$HBB_ENGINE" && -f "$CELL_ENGINE" ]]; then
    pass "canonical 引擎在位（d13 + v4d）"
else
    warn "canonical 引擎缺失（$HBB_ENGINE / $CELL_ENGINE）—— D 级会起不来；换机/刷机后先跑 orin_convert_engines.sh"
fi

# ================================================================== B 退出码
step "B 退出码语义表（对齐 main.cpp 的 return，不需要相机）"
run_expect "无参数 -> usage" "2" "$BIN"
grep_log "usage:" && pass "  usage 文本在位" || fail "  rc=2 但没打 usage:（L2 判据不一致）"

run_expect "argc=3（缺 cell 引擎）-> usage" "2" "$BIN" "$NODEV" "$FAKE_HBB"
grep_log "usage:" && pass "  argc<4 的边界成立" || fail "  argc=3 没有落进 usage 分支"

# 端口校验在 source 分支之前（main.cpp:536 早于 :541），所以给什么源都验得到，
# 且**走不到加载引擎**——占位引擎路径在这里同样安全。
# 每一个坏端口都要顺带验报错文案：只挑一个验的话，改坏另一条路径上的文案不会
# 被发现（本脚本首版就是这么漏的——变异测试把"文案改名"跑成了静默通过）。
# 现场排障是按文案搜日志的，文案一变就搜不到，等于报错白打。
for p in 99999 0 -1 abc ""; do
    run_expect "坏 UDP 端口 '$p' -> 用法错误" "2" "$BIN" "$NODEV" "$FAKE_HBB" "$FAKE_CELL" "$p"
    grep_log "invalid udp port:" \
        && pass "  报错点名 invalid udp port" \
        || fail "  rc=$LAST_RC 但没打 invalid udp port —— 文案变了，现场按文案搜不到"
done

# ⚠️ 不测 "unknown camera source:"：那条分支在 main.cpp 里**不可达**。
#    runCamera 只有一个调用点（main.cpp:544），而调用点前面已经用
#    `source.rfind("argus:",0)==0 || source.rfind("uvc:",0)==0` 过滤过一遍，
#    runCamera 内部那个 else 永远进不去。真传 "foo:/dev/video0" 会掉进**离线
#    图片分支**去构造 HbbDetector，占位引擎路径直接抛未捕获异常（SIGABRT），
#    得不到任何有意义的判据。那个 else 是自由函数的防御性守卫，留着不删；
#    但别在它上面挂验收断言 —— 那会是一条永远验不到的假绿。

# 省略 argv[5] 时应回落到默认 spec（main.cpp:543）
run_expect "省略 spec -> 回落默认" "3" "$BIN" "$NODEV" "$FAKE_HBB" "$FAKE_CELL" "50000"
grep_log "^camera spec: 1920x1080@30\$" \
    && pass "  默认 spec 为 1920x1080@30（与 orin_run_dzuav.sh / orin_bringup.sh 的 DZUAV_SPEC 同口径）" \
    || fail "  省略 spec 时没有回落到 1920x1080@30 —— 三处默认值口径分叉了"

# 离线分支的 rc=3 走的是 HbbDetector 构造**之后**的 imread/VideoCapture，
# 所以这条要真引擎；引擎不在位就跳过而不是判失败（跳过的原因写清楚，
# 静默跳过等于给出一份看着全绿、其实少验了一条的结论）。
if [[ -f "$HBB_ENGINE" && -f "$CELL_ENGINE" ]]; then
    run_expect "不存在的图片 -> 3" "3" "$BIN" "/nonexistent_ag_smoke.jpg" "$HBB_ENGINE" "$CELL_ENGINE" "50000"
else
    note "跳过「不存在的图片 -> 3」：需要真引擎（$HBB_ENGINE 不在位）"
fi

# ================================================================ C spec 严格
step "C spec 严格性（parseSpec 回写比对；不需要相机，也不需要引擎）"
# 负例：任何多余字符 / 非规范写法都必须被拒。这条是 2026-10-20 修掉的缺陷——
# 原实现 sscanf 读到 "1920x1080@30xyz" 会心满意足地拿 1920/1080/30 然后放行，
# 写错一个字符就以为在跑请求的 caps，实际 pipeline 按别的参数出帧。
BAD_SPECS=(
    "1920x1080@30xyz"     # 尾部多余字符
    "1920x1080@30 "       # 尾部空格（bringup 里最容易混进来的一种）
    "0x0@30"              # 零尺寸
    "1920x1080@"          # 缺帧率值
    "1920x1080 30"        # 缺 @
    "1920x1080@-5"        # 负帧率（f<0 被拒；f==0 是**合法**的一档，见下）
    "01920x1080@30"       # 前导零（非规范写法）
    "1920x1080@30@30"     # 重复段
    ""                    # 空串
)
for s in "${BAD_SPECS[@]}"; do
    run_expect "坏 spec 被拒: '${s}'" "2" "$BIN" "$NODEV" "$FAKE_HBB" "$FAKE_CELL" "50000" "$s"
    if grep_log "invalid camera spec:"; then
        pass "  报错点名 invalid camera spec"
    else
        fail "  rc=$LAST_RC 但没打 invalid camera spec —— 拒的理由不明，现场会查错方向"
    fi
    # 拒了就不该再回显 camera spec：main.cpp 的顺序是 parseSpec 成功**之后**才打印。
    # 若两者同时出现，现场日志里就并存着「spec 非法」和「camera spec: 某值」两条
    # 互相矛盾的事实，读日志的人会挑对自己有利的那条看。
    if grep_log "^camera spec: "; then
        fail "  spec 被拒却仍回显 camera spec: $(tail_log | grep -a -m1 '^camera spec: ')"
    fi
done

# 正例：合法写法必须**通过 parseSpec** 并把规范化后的值回显。
# 判据是「rc=3 而不是 2」+「camera spec: 逐字等于期望」——用不存在的设备，
# 于是它会停在 open camera 的 rc=3，正好证明 parseSpec 已经放行。
# 表里用 "输入|期望回显" 两段：省略 @fps 时 parseSpec 不写 fps，沿用调用方
# 带进来的 30，所以回显与输入**不同**，必须分列，不能一律要求逐字相等。
# 无 fps 那两条故意用 1280x720 / 640x480 而不是 1920x1080：runCamera 的初值
# 恰好是 1920x1080@30，用它会分不清"宽高真被写进去了"还是"碰巧等于初值"。
GOOD_SPECS=(
    "1920x1080@30|1920x1080@30"   # 规范写法，逐字回显
    "1280x720@60|1280x720@60"
    "640x480@30|640x480@30"
    "1920x1080@0|1920x1080@0"     # fps=0 是合法的一档 = 不约束帧率（两个 pipeline
                                  # builder 都是 `if (fps > 0)` 才塞 framerate=）
    "1920x1080|1920x1080@30"      # 省略 @fps -> 沿用 runCamera 初值 30
    "1280x720|1280x720@30"        # 同上，且证明宽高确实被写进去了
    "640x480|640x480@30"
)
for pair in "${GOOD_SPECS[@]}"; do
    s="${pair%%|*}"; want="${pair##*|}"
    run_expect "好 spec 放行: '$s'" "3" "$BIN" "$NODEV" "$FAKE_HBB" "$FAKE_CELL" "50000" "$s"
    if [[ $LAST_RC == 2 ]]; then
        fail "  合法 spec '$s' 被 parseSpec 拒了（rc=2）"
        continue
    fi
    if grep_log "^camera spec: $want\$"; then
        if [[ "$s" == "$want" ]]; then
            pass "  回显逐字一致: camera spec: $want"
        else
            pass "  '$s' -> camera spec: $want（省略 @fps 时沿用调用方默认 30）"
        fi
    else
        fail "  没有回显 'camera spec: $want'（输入 '$s'）—— 回写比对或 has_fps 分支行为变了"
    fi
done

# ================================================================ D 真相机
CAM_RC=""
if [[ $NO_CAMERA == 1 ]]; then
    step "D 真相机实测 —— --no-camera 指定，跳过"
    note "A~C 已覆盖 spec 严格性与退出码语义；协商尺寸对账/断流重开必须接相机才能验。"
else
    step "D 真相机实测（$DEV, $SPEC, ${SECONDS_TO_RUN}s）"
    [[ -e "$DEV" ]] || warn "相机设备 $DEV 不存在（--dev 指定；接了相机再跑才有 D 级结论）"

    DLOG="$OUT/dzuav_camera.log"
    mark_log
    echo "----- D: orin_run_dzuav.sh camera $DEV $SPEC -----" >>"$LOG"
    # 后台起 + kill -INT + wait 取真实 rc：
    #   ① 用 orin_run_dzuav.sh 而不是直接调 $BIN，引擎名才不会与实跑口径分叉；
    #      它内部是 exec，所以 bash 的 pid 就是 dzuav 的 pid，信号能直达。
    #   ② 必须发 SIGINT 而不是 SIGKILL：main.cpp 装了 handleSignal（只置
    #      g_running 标志），走这条路才会打 "stopped after N processed frames"
    #      并以 rc=0 退出——优雅停机本身就是要验的一条。
    bash "$RUN_SH" camera "$DEV" "$SPEC" >"$DLOG" 2>&1 &
    DPID=$!
    sleep "$SECONDS_TO_RUN"
    if kill -0 "$DPID" 2>/dev/null; then
        kill -INT "$DPID" 2>/dev/null
        # 给 5s 收尾；还在就 SIGKILL 兜底（不能把 dzuav 留在机上占着相机）
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            kill -0 "$DPID" 2>/dev/null || break
            sleep 0.5
        done
        kill -0 "$DPID" 2>/dev/null && { kill -KILL "$DPID" 2>/dev/null; warn "SIGINT 后 5s 未退出，已 SIGKILL —— 优雅停机不成立"; }
    else
        warn "dzuav 在 ${SECONDS_TO_RUN}s 内自己退了 —— 见下方 rc"
    fi
    wait "$DPID"; CAM_RC=$?
    cat "$DLOG" >>"$LOG"
    echo "----- rc=$CAM_RC -----" >>"$LOG"

    # -- D1 spec 原样回显（runCamera 的第一件事）
    if grep_log "^camera spec: "; then
        pass "D1 camera spec 已回显: $(tail_log | grep -a -m1 '^camera spec: ')"
    else
        fail "D1 没有 camera spec 行 —— runCamera 未走到 parseSpec 之后"
    fi

    # -- D2 pipeline 三级回退：报告最终用的是哪条（MJPEG / RAW / RAW+降档）
    if grep_log "^camera opened with: "; then
        pass "D2 相机已打开: $(tail_log | grep -a -m1 '^camera opened with: ' | cut -c1-110)"
        note "尝试过的 pipeline 共 $(count_log '^opening camera: ') 条（回退次数越多说明首选 caps 越不通）"
    else
        fail "D2 相机没打开（无 camera opened with 行）"
        grep_log "cannot open camera" && note "报错: $(tail_log | grep -a -m1 'cannot open camera' | cut -c1-160)"
    fi

    # -- D3 协商尺寸对账：这是 2026-10-20 新加的告警，第一次上机必须人工确认。
    #      内参按帧分辨率缩放（MapLocalizer::scaledIntrinsics），GStreamer 悄悄
    #      降档就会让定位系统性偏移，而 L1~L3 全都不碰内参 —— 全绿。
    NEG=$(tail_log | grep -a -m1 '^negotiated frame size: ' | awk '{print $4}')
    if [[ -z "$NEG" ]]; then
        fail "D3 没有 negotiated frame size 行 —— 新增的协商尺寸自检没生效"
    else
        REQ="${SPEC%@*}"
        note "D3 请求 $REQ / 协商到 $NEG"
        if grep_log "WARNING: negotiated "; then
            if [[ "${ALLOW_RESCALE:-0}" == "1" ]]; then
                warn "D3 协商尺寸 != 请求尺寸（ALLOW_RESCALE=1，只警告）：定位精度已不可信，必须修 pipeline 或按 $NEG 重新标定"
            else
                fail "D3 协商尺寸 != 请求尺寸：$(tail_log | grep -a -m1 'WARNING: negotiated ' | cut -c1-160)"
                note "    确认可接受再 ALLOW_RESCALE=1 重跑；否则改 pipeline 或用 tools/calibrate_camera.py 按 $NEG 重标"
            fi
        else
            [[ "$NEG" == "$REQ" ]] \
                && pass "D3 协商尺寸 == 请求尺寸（$NEG），未触发 WARNING" \
                || fail "D3 协商 $NEG != 请求 $REQ，但**没有**打 WARNING —— 告警判据失效"
        fi
    fi

    # -- D4 真出帧：只"打开成功"不算数（not-negotiated 时 isOpened 也可能是 true）
    NFRAMES=$(count_log 'localized=')
    NEMPTY=$(count_log '^empty frame')
    if [[ "${NFRAMES:-0}" -gt 0 ]]; then
        FPS=$(awk -v n="$NFRAMES" -v s="$SECONDS_TO_RUN" 'BEGIN{printf "%.2f", n/s}')
        pass "D4 真出帧 $NFRAMES 帧 / ${SECONDS_TO_RUN}s ≈ ${FPS} fps"
        # 180s 硬时限的预算：实测帧率决定一局能采到多少张情报。
        BUDGET=$(awk -v f="$FPS" 'BEGIN{printf "%d", f*180}')
        note "    按此帧率，180s 硬时限内约可处理 $BUDGET 帧"
        awk -v f="$FPS" 'BEGIN{exit !(f<10)}' \
            && warn "    实测 ${FPS} fps 远低于 spec 里的 30 —— 所有按 30fps 推的时序预算（扫描分区/悬停时长/快门窗口）都要按实测值重算"
        NLOC1=$(count_log 'localized=1')
        note "    其中定位成功 localized=1 共 ${NLOC1:-0} 帧"
    else
        fail "D4 ${SECONDS_TO_RUN}s 内一帧都没处理（localized= 计数为 0，empty frame 计数 ${NEMPTY:-0}）"
    fi

    # -- D5 优雅停机
    if grep_log "^stopped after "; then
        pass "D5 优雅停机: $(tail_log | grep -a -m1 '^stopped after ')"
    else
        fail "D5 SIGINT 后没有 stopped after 行 —— g_running 检查或信号处理断了"
    fi
    if [[ "$CAM_RC" == "0" ]]; then
        pass "D5 退出码 0（SIGINT -> rc=0）"
    else
        fail "D5 退出码 $CAM_RC（期望 0）：4=断流重开耗尽 5=连续帧异常超限 3=打不开相机"
    fi

    # -- D6 不该出现的故障行
    D6_BAD=0
    for pat in "camera reopen failed" "camera stream did not recover" \
               "too many consecutive frame errors" "frame processing error"; do
        if grep_log "$pat"; then
            fail "D6 日志出现「$pat」$(count_log "$pat") 次：$(tail_log | grep -a -m1 "$pat" | cut -c1-140)"
            D6_BAD=1
        fi
    done
    [[ $D6_BAD == 0 ]] && pass "D6 无重开失败 / 无帧异常 / 无连续错误超限"
fi

# ============================================================ E 断流重开演练
if [[ $DO_DRAIN == 1 && $NO_CAMERA != 1 ]]; then
    step "E 断流重开演练（有界重试 3 次 + 退避 + 试读判据）"
    # 这一级要人在场按回车、再物理拔线。没有 tty 时 read 会立刻返回（不是挂住，
    # 而是直接跳过等待），脚本就在操作手还没走到机器跟前时把 60s 窗口开跑了 ——
    # 演练白做，还会得出一份"没触发重开"的假失败。所以非交互直接中止。
    [[ -t 0 ]] || die "--drain 需要交互终端（要按回车、要拔线）：在机上的 tmux/VNC 里跑，不要经 SSH exec 单条命令"
    note "这一级**只能靠物理拔线**触发：脚本没法在不影响相机的前提下模拟 USB 掉电。"
    note "操作：看到提示后拔掉 $DEV 的 USB 线，数到 15 再插回。dzuav 应在 300 空帧"
    note "      （约 10s）后开始重开，最多 3 次，退避 200/400/600ms。"
    note "补充：恢复后 reopen_attempts 应归零（下一次独立掉线重新获得 3 次额度）。"
    note "      这条日志里看不出来，要靠**同一局内拔第二次**才验得到 —— 有工夫就拔两次。"
    printf '%s' "准备好后按回车开始（Ctrl-C 取消）... "
    read -r _ || die "读取确认失败"

    ELOG="$OUT/dzuav_drain.log"
    mark_log
    bash "$RUN_SH" camera "$DEV" "$SPEC" >"$ELOG" 2>&1 &
    EPID=$!
    printf '%s\n' ">>> 现在拔掉 USB 线，15 秒后插回 <<<"
    sleep 60
    kill -INT "$EPID" 2>/dev/null
    sleep 2
    kill -0 "$EPID" 2>/dev/null && kill -KILL "$EPID" 2>/dev/null
    wait "$EPID"; ERC=$?
    cat "$ELOG" >>"$LOG"
    echo "----- rc=$ERC -----" >>"$LOG"

    NATTEMPT=$(count_log 'camera stream lost; reopening pipeline')
    if [[ "${NATTEMPT:-0}" -gt 0 ]]; then
        pass "E1 触发了断流重开，共 $NATTEMPT 次尝试"
        tail_log | grep -a 'reopening pipeline\|reopen failed' | tail -6 | while read -r l; do note "  $l"; done
        if [[ "${NATTEMPT}" -gt 3 ]]; then
            fail "E2 重开尝试超过 3 次（$NATTEMPT）—— 上限失效，会无限重试到硬时限烧完"
        else
            pass "E2 重开次数在上限内（$NATTEMPT ≤ 3）"
        fi
        NFAILR=$(count_log 'camera reopen failed')
        if grep_log 'camera stream did not recover'; then
            # 3 次耗尽后必须 rc=4 响亮退出——这正是要验的行为。相机本身没恢复
            # 不算代码缺陷（多半是供电/线缆），但这次演练也就拿不到「重开后重新
            # 出帧」的证据，得再来一次，所以记警告而不是通过。
            if [[ "$ERC" == "4" ]]; then
                pass "E3 重开耗尽后以 rc=4 非零退出（不是无声空转把 180s 硬时限烧完）"
                warn "E3 但相机没恢复：本次未取得「重开后重新出帧」证据 —— 查 USB 供电/线缆后再 --drain 一次"
            else
                fail "E3 重开耗尽却不是 rc=4（实到 $ERC）—— 重试上限的兜底退出没生效"
            fi
        elif [[ "${NFAILR:-0}" -lt "${NATTEMPT:-0}" ]]; then
            pass "E3 相机已恢复（$NATTEMPT 次尝试中失败 ${NFAILR} 次，之后重新出帧）"
            note "  全程累计处理 $(count_log 'localized=') 帧"
        else
            fail "E3 $NATTEMPT 次重开全部失败且未以 rc=4 退出（rc=$ERC）—— 会无声空转到硬时限"
        fi
    else
        fail "E1 60s 内没有触发任何重开（无 camera stream lost 行）—— 拔线没生效，或空帧阈值 300 在该帧率下要等更久"
        note "    300 空帧 × usleep(33ms) ≈ 10s；实测 5fps 时空帧累积更慢，可把拔线时长加到 30s 重试"
    fi
elif [[ $DO_DRAIN == 1 ]]; then
    warn "E 断流重开演练需要相机，--no-camera 下跳过"
fi

# ==================================================================== 结论
echo | tee -a "$REPORT"
echo "==================== L4 相机验收结论 ====================" | tee -a "$REPORT"
printf '通过 %d / 失败 %d / 警告 %d\n' "$NPASS" "$NFAIL" "$NWARN" | tee -a "$REPORT"
printf '证据   %s\n' "$OUT" | tee -a "$REPORT"
printf '全量日志 %s\n' "$LOG" | tee -a "$REPORT"
printf '拉回本地 python tools/remote_run.py --host <Orin IP> --user <用户> --get %s --to <本地目录>\n' "$OUT" | tee -a "$REPORT"
if [[ $NFAIL -eq 0 ]]; then
    printf '%s\n' "结论：L4 通过。机载 main.cpp 的相机路径改动可标记为「已修」" | tee -a "$REPORT"
    printf '%s\n' "      （L1/L2 由 orin_build_verify.sh 出；L3 需 --run <engines> <测试图>）" | tee -a "$REPORT"
    echo "=========================================================" | tee -a "$REPORT"
    exit 0
fi
printf '%s\n' "结论：L4 **未**通过 —— 不要据此起飞。逐条看上面的 [失败]，全量输出在 $LOG" | tee -a "$REPORT"
echo "=========================================================" | tee -a "$REPORT"
exit 1
