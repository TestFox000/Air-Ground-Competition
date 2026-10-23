#!/usr/bin/env bash
# =============================================================================
# 机载一键入口（仓库单源）。桌面 ~/start.sh 只是一层薄壳：
#     exec bash <repo>/tools/orin_menu.sh "$@"
# 这样改菜单/改报错只改本文件，不必重刷机器；也不再往桌面写 6KB 的 heredoc
# 脚本副本（2026-10-22：副本里混着 bash+Python heredoc，引号错一个字符就整盘崩）。
#
# 用法：
#     bash tools/orin_menu.sh              # 交互菜单
#     bash tools/orin_menu.sh up|up-send|accept|stop|recon|recon-replay|plan|diag
#     bash tools/orin_menu.sh broadcast-start|broadcast-stop
# =============================================================================
set -u

REPO="$(cd "$(dirname "$0")/.." && pwd)"
PARAMS="$REPO/src/ag_integrate/config/match_params.yaml"
OUT=/tmp/ag_plan_live
LOGDIR=/tmp/ag_menu_logs
mkdir -p "$LOGDIR" 2>/dev/null || true

say()  { printf '\n\e[1;36m==> %s\e[0m\n' "$*"; }
ok()   { printf '\e[1;32m[通过]\e[0m %s\n' "$*"; }
warn() { printf '\e[1;33m[警告]\e[0m %s\n' "$*"; }
bad()  { printf '\e[1;31m[失败]\e[0m %s\n' "$*"; }
note() { printf '      %s\n' "$*"; }

pause() { [ -t 0 ] && { echo; read -r -p "按回车返回菜单…" _ || true; }; }
have()  { command -v "$1" >/dev/null 2>&1; }

# ---------------------------------------------------------------- 启动前检查
preflight() {
    echo "--- 启动前检查 ---"
    if [ ! -e /dev/video0 ]; then
        warn "没有检测到相机 /dev/video0：dzuav 窗口会立即退出，"
        note "规划窗口会一直等不到情报（其余窗口正常）。这是预期现象，不是故障。"
        note "接上相机后重跑本项即可；离线侦察（菜单 8）不需要相机。"
    else
        ok "相机 /dev/video0 存在"
    fi
    local bin="$REPO/onboard/dzuav/build/dzuav"
    if [ -f "$bin" ] && grep -qa uav_end "$bin"; then
        ok "dzuav 二进制为 13 类"
    elif [ -f "$bin" ]; then
        bad "dzuav 二进制不是 13 类（未见 uav_end）——请勿继续"
        note "处理：bash tools/orin_build_verify.sh"
    else
        bad "缺少 dzuav 二进制"
        note "处理：bash tools/orin_build_verify.sh"
    fi
    local e
    for e in hbb_detector_d13_fp16 cell_classifier_v4d_fp16; do
        [ -f "$HOME/dzuav/engines/$e.engine" ] \
            && ok "引擎 $e" \
            || { bad "缺少引擎 $e.engine（13 类部署线要求 d13 + v4d）"; \
                 note "处理：核对 tools/orin_deploy.py 的模型推送，勿用 7 类旧引擎"; }
    done
    if tmux has-session -t agup 2>/dev/null; then
        note "· 全栈已在运行（tmux 会话 agup 存在），本次启动会先停掉再重开"
    fi
    echo "-------------------"
}

# ---------------------------------------------------------------- 启动/停止
start_stack() {   # $1 = dry | send
    local mode="${1:-dry}" log rc=0
    log="$LOGDIR/up_$(date +%H%M%S).log"
    preflight
    if [ "$mode" = "send" ]; then
        echo "⚠ 即将【实发】车命令（0x4003），路线会直接给车执行。"
        read -r -p "确认车端已就绪？输入 yes 继续: " a || a=""
        [ "$a" = "yes" ] || { echo "已取消。"; return 0; }
        PLAN_SEND=1 bash "$REPO/tools/orin_bringup.sh" up --no-teleop 2>&1 | tee "$log"
        rc=${PIPESTATUS[0]}
    else
        bash "$REPO/tools/orin_bringup.sh" up --no-teleop 2>&1 | tee "$log"
        rc=${PIPESTATUS[0]}
    fi
    echo
    if [ "$rc" -ne 0 ]; then
        bad "全栈【未启动】（orin_bringup.sh 退出码 $rc，完整日志：$log）"
        local dead
        dead=$(ls /tmp/agup_agup/*.rc 2>/dev/null | xargs -r -n1 basename \
               | sed 's/\.rc$//' | tr '\n' ' ')
        [ -n "${dead:-}" ] && echo "未起来的窗口：$dead"
        if grep -q '相机设备 .* 不存在' "$log"; then
            echo "原因：相机未接（dzuav 会立即退出，其余窗口不受影响）"
            echo "影响：实时感知链不可用；**离线视频侦察（菜单 8 / 9）照常可用**"
            echo "处理：接上相机后重跑 1；或 DZUAV_CAM=/dev/videoN 指定"
        elif grep -q '飞控串口 .* 不存在' "$log"; then
            echo "原因：飞控串口缺失（px4 窗口起不来 → 遥测全无）"
            echo "处理：检查飞控 USB；ls -l /dev/serial/by-id/ 认 NxtPX4；"
            echo "      或用 PX4_DEV=/dev/ttyACMx 显式指定"
        elif grep -q '会话外已在跑' "$log"; then
            echo "原因：会话外有残留进程（autostart 看门狗/旧栈没退干净）。上面已列出 pid 与命令行。"
            echo "处理 A（推荐）先停残留再起："
            echo "    pkill -f '[o]rin_agent_watchdog.py' ; pkill -x MicroXRCEAgent"
            echo "    ~/start.sh up"
            echo "  ⚠ 停之前确认它接的是飞控：ls -l /dev/serial/by-id/（NxtPX4 才是飞控）"
            echo "处理 B 确认要共存："
            echo "    ALLOW_STRAY=1 tools/orin_bringup.sh up --no-teleop"
        elif grep -qE '死亡|未.*就绪' "$log"; then
            echo "原因：有窗口启动后退出。"
            echo "处理：tools/orin_bringup.sh status  看哪个窗口的 rc；"
            echo "      tmux attach -t agup 进对应窗口看报错（Ctrl+B 再按 d 退出）"
        else
            echo "处理：看日志 $log；或 tools/orin_bringup.sh status"
        fi
        return 1
    fi
    ok "已启动：4=验收  5=进 tmux  6=看产物  7=停止"
}

# ---------------------------------------------------------------- 体检
diag() {
    say "一键体检 $(date '+%F %T')"
    echo "[磁盘 /tmp]  $(df -h /tmp 2>/dev/null | tail -1)"
    echo "[内存]       $(free -h 2>/dev/null | sed -n '2p')"
    local cams
    cams=$(ls /dev/video* 2>/dev/null | tr '\n' ' ')
    echo "[相机]       ${cams:-无 /dev/video*（相机未接）}"
    echo "[USB 相机]   $(lsusb 2>/dev/null | grep -iE '0bda:5883|camera' || echo '未见（0bda:5883 为已知相机）')"
    echo "[飞控串口]   $(ls -l /dev/serial/by-id/ 2>/dev/null | sed -n '2,6p' | tr '\n' ' ')"
    echo "[相关进程]（含当前全栈；不属于 agup 的才是残留）"
    pgrep -a -f 'MicroXRCEAgent|px4_odom_convert|fastlio_mapping|livox_ros_driver2_node|orin_agent_watchdog' \
        2>/dev/null | grep -v pgrep | sed 's/^/             /' || echo "             无"
    local bin="$REPO/onboard/dzuav/build/dzuav"
    if [ -f "$bin" ] && grep -qa uav_end "$bin"; then
        echo "[dzuav]      13 类 ✓"
    else
        echo "[dzuav]      ✗ 缺失或不是 13 类 → bash tools/orin_build_verify.sh"
    fi
    echo "[引擎]       $(ls "$HOME"/dzuav/engines/*.engine 2>/dev/null | xargs -r -n1 basename | tr '\n' ' ')"
    echo "[栈窗口]"
    tmux list-windows -t agup -F '             #{window_name}' 2>/dev/null || echo "             未启动"
    echo "[Python]     $(python3 -V 2>&1)  cv2: $(python3 -c 'import cv2;print(cv2.__version__)' 2>/dev/null || echo 不可用)"
    echo "[WiFi]       $(nmcli -t -f GENERAL.STATE,IP4.ADDRESS connection show TCL-842553 2>/dev/null | tr '\n' ' ')"
    echo "[地址]       $(ip -4 -o addr show scope global | awk '{print $2": "$4}' | tr '\n' '  ')"
    echo "[广播]       $(pgrep -af '[o]rin_broadcast_route' | head -1 || echo 未运行)"
    echo "[最近缓存]"
    ls -dt /tmp/ag_recon/*/ 2>/dev/null | head -3 | sed 's/^/             /' || echo "             无"
    echo "[最近日志]   $LOGDIR"
}

# ---------------------------------------------------------------- 产物/侦察
show_plan() {
    local latest
    latest=$(ls -dt /tmp/ag_plan* /tmp/ag_recon/*/ 2>/dev/null | while read -r d; do
                 [ -f "$d/route_plan.json" ] && { echo "${d%/}"; break; }; done)
    if [ -z "${latest:-}" ]; then
        warn "还没有 route_plan.json（规划窗口还没产出过，或侦察还没跑成功）"
        note "看一次：菜单 8 视频侦察；看实时栈：菜单 1 后等情报"
        return 1
    fi
    echo "最新产物目录: $latest"
    ls -l "$latest" 2>/dev/null
    python3 "$REPO/tools/orin_print_summary.py" "$latest"
    echo "--- planned_path.txt（前 300 字节）---"
    head -c 300 "$latest/planned_path.txt" 2>/dev/null || echo "(无 planned_path.txt)"
    echo
}

show_render() {
    local png
    png=$(ls -t /tmp/ag_recon/*/route_render.png 2>/dev/null | head -1 || true)
    [ -n "$png" ] || { warn "没找到 route_render.png（识别可能没跑完）"; return 0; }
    echo
    echo "渲染图: $png"
    if have eog; then
        eog "$png" 2>/dev/null || true
    elif have xdg-open; then
        xdg-open "$png" 2>/dev/null || true
    else
        note "没有 eog/xdg-open，手动打开上面的路径即可"
    fi
}

analyze_recon_failure() {   # $1 = 日志
    local log="$1"
    echo "完整日志：$log"
    if grep -q 'cv2 不可用' "$log"; then
        echo "原因：Python cv2 环境缺失/损坏。"
        echo "处理：bash tools/orin_fix_pyvision.sh --smoke"
    elif grep -qE '未找到.*(d13|v4d|模型)|模型.*(缺失|找不到)' "$log"; then
        echo "原因：模型缺失或路径不对。"
        echo "处理：ls ~/dzuav/models/ 确认 d13+v4d；或用 --d13/--v4d 显式指定"
    elif grep -qE '视频不存在|打不开视频|无法解码|0 字节' "$log"; then
        echo "原因：视频路径/编码问题。"
        echo "处理：ls -l <视频> 看大小；ffprobe <视频> 看编码；转 h264 mp4 再试"
    elif grep -q '没有缓存' "$log"; then
        echo "原因：该视频还没跑过识别（--replay-last 无缓存可复用）。"
        echo "处理：不加 --replay-last，先跑一次完整侦察"
    elif grep -q '未预期的记号' "$log"; then
        echo "原因：脚本在运行中被覆盖（bash 按字节偏移读脚本）。"
        echo "处理：重新部署后再跑；以后用「上传 .new → mv」原子替换"
    elif grep -qE 'MemoryError|Killed' "$log"; then
        echo "原因：内存不足（逐帧 stride=1 很吃内存/CPU）。"
        echo "处理：用默认 stride 5；或关掉其它大进程再跑"
    else
        echo "处理：把上面的日志一起发出来；或加 --debug 看堆栈"
    fi
}

recon_video() {   # 参数原样透传给 orin_video_recon.sh
    local log rc=0
    log="$LOGDIR/recon_$(date +%H%M%S).log"
    bash "$REPO/tools/orin_video_recon.sh" "$@" 2>&1 | tee "$log"
    rc=${PIPESTATUS[0]}
    if [ "$rc" -ne 0 ]; then
        bad "视频侦察失败（退出码 $rc）"
        analyze_recon_failure "$log"
        return "$rc"
    fi
    show_render
}

# ---------------------------------------------------------------- 广播
broadcast_latest_plan() {
    # 优先用 stride5（实测最优）的路线；没有才退到最新一份
    local p
    p=$(ls -dt /tmp/ag_recon/*/route_plan.json 2>/dev/null | grep -- '-s5/' | head -1)
    [ -n "${p:-}" ] || p=$(ls -dt /tmp/ag_recon/*/route_plan.json 2>/dev/null | head -1)
    echo "${p:-}"
}

broadcast_start() {
    local plan log
    plan=$(broadcast_latest_plan)
    if [ -z "${plan:-}" ]; then
        warn "没有可广播的 route_plan.json"
        note "先跑菜单 8（视频侦察）或等机载规划产出"
        return 1
    fi
    if pgrep -f '[o]rin_broadcast_route.py' >/dev/null 2>&1; then
        warn "已有广播实例在跑；先停再起"
        note "停止：pkill -f orin_broadcast_route.py"
        return 1
    fi
    log=/tmp/ag_broadcast.log
    setsid python3 -u "$REPO/tools/orin_broadcast_route.py" --plan "$plan" --hz 2 \
        >"$log" 2>&1 </dev/null &
    sleep 2
    if pgrep -f '[o]rin_broadcast_route.py' >/dev/null 2>&1; then
        ok "0x4003 持续广播已启动（2Hz，路线：$plan）"
        note "抓包：sudo tcpdump -i any -n udp port 50000"
        note "日志：$log    停止：pkill -f orin_broadcast_route.py"
    else
        bad "广播进程没能启动（日志：$log）"
        tail -n 8 "$log" 2>/dev/null
        return 1
    fi
}

broadcast_stop() {
    if pkill -f '[o]rin_broadcast_route.py' 2>/dev/null; then
        ok "已停止 0x4003 广播"
    else
        note "当前没有广播实例在跑"
    fi
}

# ---------------------------------------------------------------- 菜单
menu() {
    while true; do
        cat <<'TXT'

╔════════════════════════════════════════════════════════╗
║              机载一键入口（输入后回车）                 ║
╠════════════════════════════════════════════════════════╣
║   1  启动全栈（规划只落盘，不发车命令）                  ║
║   2  启动全栈 并 实发车命令（⚠ 会驱动车，需二次确认）    ║
║   3  只启机载规划（前台运行，Ctrl+C 停止）               ║
║   4  机载规划验收（自检全链，约 40 秒）                  ║
║   5  进入 tmux 看各窗口（Ctrl+B 切窗，再按 d 退出）      ║
║   6  看最新规划产物                                      ║
║   7  停止全栈                                            ║
║   8  视频侦察：选视频 → 识别 + 规划 + 出图（任意视频）    ║
║   9  秒级重出最近一次（复用识别结果，按当前门控重算）    ║
║   d  一键体检（相机/引擎/串口/残留进程/栈/网络/缓存）     ║
║   b  启停 0x4003 持续广播（测试源，给别人抓包）           ║
║   0  退出                                                ║
╚════════════════════════════════════════════════════════╝
TXT
        read -r -p "请选择 [0-9/d/b]: " n || exit 0
        case "${n:-}" in
            1) start_stack dry; pause ;;
            2) start_stack send; pause ;;
            3) python3 -u "$REPO/tools/orin_plan_sidecar.py" --params "$PARAMS" \
                   --out "$OUT" --plan-mode on-change ;;
            4) bash "$REPO/tools/orin_accept_plan.sh"; pause ;;
            5) tmux attach -t agup 2>/dev/null || warn "全栈未启动（先选 1）"; pause ;;
            6) show_plan; pause ;;
            7) bash "$REPO/tools/orin_bringup.sh" down; pause ;;
            8) recon_video; pause ;;
            9) recon_video --replay-last; pause ;;
            d|D) diag; pause ;;
            b|B) if pgrep -f '[o]rin_broadcast_route.py' >/dev/null 2>&1; then
                     broadcast_stop
                 else
                     broadcast_start
                 fi
                 pause ;;
            0) exit 0 ;;
            *) echo "只接受 0-9 / d / b。" ;;
        esac
    done
}

# 非交互入口（桌面图标/~/start.sh 传参）：执行完一律回到菜单。
case "${1:-menu}" in
    up)             start_stack dry; pause; menu ;;
    up-send)        start_stack send; pause; menu ;;
    accept)         bash "$REPO/tools/orin_accept_plan.sh"; pause; menu ;;
    stop)           bash "$REPO/tools/orin_bringup.sh" down; pause; menu ;;
    recon)          recon_video; pause; menu ;;
    recon-replay)   recon_video --replay-last; pause; menu ;;
    plan)           python3 -u "$REPO/tools/orin_plan_sidecar.py" --params "$PARAMS" \
                        --out "$OUT" --plan-mode on-change ;;
    diag)           diag; pause; menu ;;
    broadcast-start) broadcast_start; pause; menu ;;
    broadcast-stop)  broadcast_stop; pause; menu ;;
    menu|*)         menu ;;
esac
