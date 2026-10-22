#!/usr/bin/env bash
# =============================================================================
# 机载桌面一键入口安装器（在 Orin 上跑一次即可）
#
# 目的：VNC 桌面前的人不需要记路径与长命令——装完桌面出现三个图标：
#       启动机载全栈 / 机载规划验收 / 停止机载全栈    +  一份《机载操作指南》
# 只写 $HOME 下的 start.sh 与 Desktop/，不动仓库任何源文件。
#
# 用法:  bash tools/orin_install_desktop.sh
# =============================================================================
set -u

REPO="$HOME/dzuav_deploy/air_ground"
DESK="$HOME/Desktop"
GUIDE="$DESK/机载操作指南.txt"

ok()   { echo "[通过] $*"; }
fail() { echo "[失败] $*" >&2; exit 1; }

[ -d "$REPO" ] || fail "找不到机载仓库 $REPO"
mkdir -p "$DESK"

# ---------------------------------------------------------------- ~/start.sh
cat > "$HOME/start.sh" <<'START'
#!/usr/bin/env bash
# 机载一键入口（VNC 桌面用）。菜单里每条都对应仓库里的一条正式命令，无隐藏动作。
set -u
REPO="$HOME/dzuav_deploy/air_ground"
PARAMS="$REPO/src/ag_integrate/config/match_params.yaml"
OUT=/tmp/ag_plan_live

pause() { [ -t 0 ] && { echo; read -r -p "按回车返回菜单…" _ || true; }; }

# 启动前的人话检查：把"会静默跑错/跑不起来"的条件先摆到屏幕上。
preflight() {
    echo "--- 启动前检查 ---"
    if [ ! -e /dev/video0 ]; then
        echo "⚠ 没有检测到相机 /dev/video0：dzuav 窗口起来后会立即退出，"
        echo "  规划窗口会一直等不到情报（其余窗口正常）。这是预期现象，不是故障。"
    else
        echo "✓ 相机 /dev/video0 存在"
    fi
    local bin="$REPO/onboard/dzuav/build/dzuav"
    if [ -f "$bin" ] && grep -qa uav_end "$bin"; then
        echo "✓ dzuav 二进制为 13 类"
    elif [ -f "$bin" ]; then
        echo "✗ dzuav 二进制不是 13 类（未见 uav_end）——请勿继续，先跑 orin_build_verify.sh"
    else
        echo "✗ 缺少 dzuav 二进制，先跑 bash tools/orin_build_verify.sh"
    fi
    for e in hbb_detector_d13_fp16 cell_classifier_v4d_fp16; do
        [ -f "$HOME/dzuav/engines/$e.engine" ] \
            && echo "✓ 引擎 $e" \
            || echo "✗ 缺少引擎 $e.engine（13 类部署线要求 d13 + v4d）"
    done
    if tmux has-session -t agup 2>/dev/null; then
        echo "· 全栈已在运行（tmux 会话 agup 存在），本次启动会先停掉再重开"
    fi
    echo "-------------------"
}

start_stack() {   # $1 = dry | send
    preflight
    if [ "$1" = "send" ]; then
        echo "⚠ 即将【实发】车命令（0x4003），路线会直接给车执行。"
        read -r -p "确认车端已就绪？输入 yes 继续: " a
        [ "$a" = "yes" ] || { echo "已取消。"; return; }
        PLAN_SEND=1 bash "$REPO/tools/orin_bringup.sh" up --no-teleop
    else
        bash "$REPO/tools/orin_bringup.sh" up --no-teleop
    fi
    echo
    echo "已启动。下一步可：4=验收  5=进 tmux 看各窗口  6=看规划产物  7=停止"
}

show_plan() {
    local latest
    latest=$(ls -dt /tmp/ag_plan* 2>/dev/null | head -1)
    [ -n "${latest:-}" ] || { echo "还没有产物目录（规划窗口还没产出过）"; return; }
    echo "最新产物目录: $latest"
    ls -l "$latest" 2>/dev/null
    if [ -s "$latest/route_plan.json" ]; then
        echo "--- route_plan.json 摘要 ---"
        grep -m1 '"reset"'       "$latest/route_plan.json"
        grep -m1 '"route"'       "$latest/route_plan.json"
        grep -m1 'route_len_cells' "$latest/route_plan.json"
        grep -m1 '排雷覆盖'       "$latest/route_plan.json"
    else
        echo "（暂无 route_plan.json —— 门控还没放行：需要识别到 ≥1 颗雷 + 靶面）"
    fi
    echo "--- planned_path.txt ---"
    head -c 300 "$latest/planned_path.txt" 2>/dev/null
}

menu() {
    while true; do
        cat <<'TXT'

╔══════════════════════════════════════════════════════╗
║              机载一键入口（输入数字后回车）            ║
╠══════════════════════════════════════════════════════╣
║   1  启动全栈（规划只落盘，不发车命令）                ║
║   2  启动全栈 并 实发车命令（⚠ 会驱动车，需二次确认）  ║
║   3  只启机载规划（前台运行，Ctrl+C 停止）             ║
║   4  机载规划验收（自检全链，约 40 秒）                ║
║   5  进入 tmux 看各窗口（Ctrl+B 切窗，再按 d 退出）    ║
║   6  看最新规划产物                                    ║
║   7  停止全栈                                          ║
║   8  视频侦察：选视频 → 识别 + 规划 + 出图（任意视频）  ║
║   9  秒级重出最近一次（复用识别结果，按当前门控重算）  ║
║   0  退出                                              ║
╚══════════════════════════════════════════════════════╝
TXT
        read -r -p "请选择 [0-9]: " n || exit 0
        case "${n:-}" in
            1) start_stack dry; pause ;;
            2) start_stack send; pause ;;
            3) python3 -u "$REPO/tools/orin_plan_sidecar.py" --params "$PARAMS" \
                   --out "$OUT" --plan-mode on-change ;;
            4) bash "$REPO/tools/orin_accept_plan.sh"; pause ;;
            5) tmux attach -t agup ;;
            6) show_plan; pause ;;
            7) bash "$REPO/tools/orin_bringup.sh" down; pause ;;
            8) recon_video; pause ;;
            9) recon_replay; pause ;;
            0) exit 0 ;;
            *) echo "只接受 0-9 的数字。" ;;
        esac
    done
}

# 视频侦察：离线把"视频 → 态势 → 路线 → 车命令 → 渲染图"跑一遍（通用入口）。
# 8 = 给任意视频跑识别+规划（无参数时列出机上视频让你选）；9 = 秒级重出最近一次的规划。
recon_video() {
    bash "$REPO/tools/orin_video_recon.sh"
    show_render
}
recon_replay() {
    bash "$REPO/tools/orin_video_recon.sh" --replay-last
    show_render
}
show_render() {
    local png
    png=$(ls -t /tmp/ag_recon/*/route_render.png 2>/dev/null | head -1 || true)
    [ -n "$png" ] || return 0
    echo
    echo "渲染图: $png（正在打开…关掉即回到本菜单）"
    eog "$png" 2>/dev/null || xdg-open "$png" 2>/dev/null || true
}

# 图标/命令行动作：**执行完一律回到菜单**，不停在一个死界面上。
# 2026-10-20 反馈：原先 accept/stop 跑完 pause 就退出，用户在 VNC 上
# 按完回车看到一片空白，以为"没有菜单"——菜单必须是操作中枢。
case "${1:-menu}" in
    up)      start_stack dry; pause; menu ;;
    up-send) start_stack send; pause; menu ;;
    accept)  bash "$REPO/tools/orin_accept_plan.sh"; pause; menu ;;
    stop)    bash "$REPO/tools/orin_bringup.sh" down; pause; menu ;;
    recon)   recon_video; pause; menu ;;
    recon-replay) recon_replay; pause; menu ;;
    plan)    python3 -u "$REPO/tools/orin_plan_sidecar.py" --params "$PARAMS" \
                 --out "$OUT" --plan-mode on-change ;;
    menu|*)  menu ;;
esac
START
chmod +x "$HOME/start.sh"
ok "已安装 ~/start.sh（菜单式一键入口）"

# ---------------------------------------------------------------- 桌面图标
mk_item() {  # mk_item <文件名> <显示名> <参数>
    cat > "$DESK/$1" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=$2
Comment=空地协同侦排挑战赛 · 机载自主规划
Terminal=true
Exec=$HOME/start.sh $3
Icon=utilities-terminal
Categories=Utility;
EOF
    chmod +x "$DESK/$1"
    gio set "$DESK/$1" metadata::trusted true 2>/dev/null || true
}

mk_item "ag-plan-start.desktop"  "① 启动机载全栈"     "up"
mk_item "ag-plan-accept.desktop" "② 机载规划验收"     "accept"
mk_item "ag-plan-stop.desktop"   "③ 停止机载全栈"     "stop"
mk_item "ag-recon.desktop"       "④ 视频侦察（出路线图）" "recon"
mk_item "ag-plan-menu.desktop"   "更多操作（菜单）"   "menu"
ok "已安装桌面图标：启动 / 验收 / 停止 / 视频侦察 / 更多操作（双击即执行，执行完自动回菜单）"

# ---------------------------------------------------------------- 桌面 .sh（可直接运行）
# 用户要求"sh 化"：桌面除 .desktop 外再放一个**真 .sh**，双击/终端都能直接跑，
# 不必记仓库路径。收尾带 pause，避免双击跑完窗口就关掉看不到结果。
cat > "$DESK/视频侦察.sh" <<'SH'
#!/usr/bin/env bash
# 桌面直跑入口：任意视频 → 识别 → 路线规划 → 渲染图
#   ./视频侦察.sh                    列出机上视频、选一个跑（默认 stride 1 逐帧）
#   ./视频侦察.sh ~/a.mp4 --stride 5    # 想快（可能漏目标；默认逐帧最全）
#   ./视频侦察.sh --replay-last      秒级：用最近一次的识别结果重出规划
set -u
REPO="$HOME/dzuav_deploy/air_ground"
cd "$REPO" || { echo "找不到仓库: $REPO"; read -r -p "回车关闭…" _; exit 9; }
bash tools/orin_video_recon.sh "$@"
echo
read -r -p "按回车返回菜单…" _ || true
exec "$HOME/start.sh" menu
SH
chmod +x "$DESK/视频侦察.sh"
ok "已安装 ~/Desktop/视频侦察.sh（可直接双击/终端运行，跑完自动回菜单）"

# ---------------------------------------------------------------- 操作指南
cat > "$GUIDE" <<'TXT'
机载操作指南（VNC 桌面前照着做即可）
=====================================

【一句话】桌面上双击「① 启动机载全栈」→ 它会先做启动前检查、然后直接拉起全栈，
         拉起后**自动显示数字菜单**（4=验收 5=看窗口 6=看产物 7=停止 8=视频侦察 0=退出）。
         如果你在任何时候找不到菜单：打开终端敲  ~/start.sh  回车即可。

〇、只想看「视频 → 路径规划」（不需要起全栈、不需要相机）——**通用，任意视频**
  1. 双击桌面「④ 视频侦察（出路线图）」或「视频侦察.sh」
     · 无参数时它**列出机上视频**让你输序号（也会认你直接粘的路径）
     · 选完就跑：识别 → 格号反推 → 路线 → 车命令 → 渲染图，跑完自动打开图
     · 前置件（cv2 / 13 类模型 / 中文字体）缺哪项它会说清修法
  2. 秒级重出规划：菜单输入 9（或 ./视频侦察.sh --replay-last）
     · 复用上次识别结果，只重跑规划层，几秒出图（调参数、看路线时用这个）
  3. 终端等价命令（VNC 里可手敲）：
        cd ~/dzuav_deploy/air_ground
        ./tools/orin_video_recon.sh                    # 选视频跑
        ./tools/orin_video_recon.sh ~/a.mp4 --stride 5   # 想快（可能漏目标）
        ./tools/orin_video_recon.sh --replay-last      # 秒级
  4. 采样帧距（--stride）怎么选：
        1  = 默认，**逐帧最全**（测试环境不省时间；2220 帧约 80 分钟）
        调大 = 拿采样换时间，**不是"快一点的同结果"**：同一段视频实测 stride15
                只剩 3 雷、stride 5 出 8 雷（=官方基线 8）——短暂可见的雷会被漏掉
        同一视频同参数跑第二次会自动复用逐帧识别结果，并用**当前门控**重算态势
        （秒级；门控修过也不会继续拿旧结论）
  ⚠ 渲染图上的 D8* 是移动靶的“确认位”（视频里看到的是红面=友方）：静态回放要手动
     去掉这个 '*'，在飞链路由 fire_guard 复判（红面/未辨自动禁火）。

一、开机后要做的事（每次上电）
  1. 双击「① 启动机载全栈」
     · 先看它打印的「启动前检查」：相机/二进制是否 13 类/引擎是否 d13+v4d
     · 然后拉起：感知(dzuav) / 机载规划(plan) / 雷达 / SLAM定位 / 飞控桥 / 控制 / 航线
     · 规划窗口默认「只落盘，不发车命令」——安全档
  2. 菜单里输入 4 回车 → 机载规划验收（约 40 秒）
     · 一路 [通过] 并以「机载规划验收通过」结尾即为正常
     · 它会自检：栈版本、规划进程起没起、同机广播投递、矩阵/门控/路线/三份产物

二、怎么看正在发生什么
  · 菜单输入 5（或终端敲 tmux attach -t agup）
    → 进入 10 个窗口，看名字：
        dzuav    相机识别+定位（打印 localized / landmarks）
        plan     机载规划（打印场地矩阵 + 路线）
        fastlio  定位   px4 飞控桥   control 控制   path 航线
    切窗口：Ctrl+B 松开后按窗口名首字母或数字；退出看板：Ctrl+B 再按 d
  · 【先看这两行，确认没跑错版本】dzuav 窗口开头会打印：
        [配置] bin    = .../onboard/dzuav/build/dzuav
        [配置] hbb    = hbb_detector_d13_fp16.engine
        [配置] cell   = cell_classifier_v4d_fp16.engine
    若出现 hbb_detector_fp16 / cell_classifier_fp16（没有 d13/v4d），说明跑的是
    7 类旧栈，类别编号会与协议错位（障碍可能被当蓝靶）——立即停止并告知负责人。

三、规划产物在哪
  · /tmp/ag_plan_live/   （菜单输入 6 可看摘要）
      cell_matrix.json   场地矩阵 + 精确坐标
      route_plan.json    路线 + 车命令 + 校验结论
      planned_path.txt   执行路线格序列

四、真的要让车动起来（比赛日）
  1. 先确认：车端已上线、已切薄接收（car_command_source=onboard）
  2. 菜单输入 2（启动全栈并实发；会要求你输入 yes 二次确认）
     ⚠ 这一步发出的路线会直接给车执行，务必先做过验收（步骤一·2）
  3. 回滚：菜单输入 1（重新以只落盘方式启动）

五、出问题怎么停
  · 双击「③ 停止机载全栈」，或菜单输入 7
  · 或终端敲：tools/orin_bringup.sh down

六、常见现象对照
  · plan 窗口一直只有空矩阵、没有路线 → 门控没放行：需要识别到「≥1 颗雷 + 看清靶面」
  · dzuav 窗口 localized=0 且 landmarks 很少 → 相机视野内地标记不够（相机/高度问题，
    不是规划问题）：PnP 至少要 4 个格号且都要是内点
  · 验收脚本报「规划进程未能启动」 → UDP:50000 被别的进程占了（例如另一个规划进程还在跑）
TXT
ok "已写入桌面《机载操作指南.txt》"

echo
echo "==================== 安装完成 ===================="
echo "桌面图标：① 启动机载全栈 / ② 机载规划验收 / ③ 停止全栈 / ④ 视频侦察 / 更多操作（菜单）"
echo "          （双击即执行；执行完自动回到数字菜单）"
echo "找不到菜单时：打开终端敲  ~/start.sh  回车"
echo "文本入口：~/start.sh（也支持 up / accept / stop / recon / recon-full / plan）"
echo "操作指南：~/Desktop/机载操作指南.txt"
echo "=================================================="
