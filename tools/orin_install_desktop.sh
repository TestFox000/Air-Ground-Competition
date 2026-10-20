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

==================== 机载一键入口 ====================
  1  启动全栈（感知+规划+雷达+飞控；规划只落盘不发车命令）
  2  启动全栈 并 实发车命令（⚠ 命令会上车，确认车端已就绪再选）
  3  只启机载规划（不起雷达/飞控；终端调试用，Ctrl+C 停止）
  4  机载规划验收（自检：进程→投递→矩阵→门控→路线→三份产物）
  5  看规划日志（接进 tmux 会话 agup，Ctrl+B 再按数字切窗口）
  6  看最新规划产物（route_plan.json 摘要）
  7  停止全栈
  0  退出
=====================================================
TXT
        read -r -p "输入编号后回车: " n || exit 0
        case "${n:-}" in
            1) bash "$REPO/tools/orin_bringup.sh" up --no-teleop; pause ;;
            2) PLAN_SEND=1 bash "$REPO/tools/orin_bringup.sh" up --no-teleop; pause ;;
            3) python3 -u "$REPO/tools/orin_plan_sidecar.py" --params "$PARAMS" \
                   --out "$OUT" --plan-mode on-change ;;
            4) bash "$REPO/tools/orin_accept_plan.sh"; pause ;;
            5) tmux attach -t agup ;;
            6) show_plan; pause ;;
            7) bash "$REPO/tools/orin_bringup.sh" down; pause ;;
            0) exit 0 ;;
            *) ;;
        esac
    done
}

case "${1:-menu}" in
    menu)        menu ;;
    accept)      bash "$REPO/tools/orin_accept_plan.sh"; pause ;;
    stop)        bash "$REPO/tools/orin_bringup.sh" down; pause ;;
    plan)        python3 -u "$REPO/tools/orin_plan_sidecar.py" --params "$PARAMS" \
                     --out "$OUT" --plan-mode on-change ;;
    *)           menu ;;
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

mk_item "ag-plan-start.desktop"  "启动机载全栈"   "menu"
mk_item "ag-plan-accept.desktop" "机载规划验收"   "accept"
mk_item "ag-plan-stop.desktop"   "停止机载全栈"   "stop"
ok "已安装桌面图标：启动机载全栈 / 机载规划验收 / 停止机载全栈"

# ---------------------------------------------------------------- 操作指南
cat > "$GUIDE" <<'TXT'
机载操作指南（VNC 桌面前照着做即可）
=====================================

【一句话】桌面上双击「启动机载全栈」→ 在菜单里选 1 → 之后双击「机载规划验收」看是否全绿。

一、开机后要先做的事（每次上电）
  1. 双击桌面「启动机载全栈」 → 菜单里输入 1 回车
     · 这一步会拉起：感知(dzuav) / 机载规划(plan) / 雷达 / SLAM定位 / 飞控桥 / 控制 / 航线
     · 规划窗口默认「只落盘，不发车命令」——安全档
  2. 双击「机载规划验收」 → 看到一路 [通过] 与「机载规划验收通过」即为正常
     · 它会自检：规划进程起没起、同机广播投递成不成、矩阵/门控/路线/三份产物在不在

二、怎么看正在发生什么
  · 双击「启动机载全栈」→ 菜单输入 5（或终端敲 tmux attach -t agup）
    → 进入 10 个窗口，看名字：
        dzuav    相机识别+定位（打印 localized / landmarks）
        plan     机载规划（打印场地矩阵 + 路线）
        fastlio  定位   px4 飞控桥   control 控制   path 航线
    切窗口：Ctrl+B 松开后按窗口名首字母或数字；退出看板：Ctrl+B 再按 d

三、规划产物在哪
  · /tmp/ag_plan_live/   （菜单输入 6 可看摘要）
      cell_matrix.json   场地矩阵 + 精确坐标
      route_plan.json    路线 + 车命令 + 校验结论
      planned_path.txt   执行路线格序列

四、真的要让车动起来（比赛日）
  1. 先确认：车端已上线、已切薄接收（car_command_source=onboard）
  2. 双击「启动机载全栈」→ 菜单输入 2（=PLAN_SEND=1，实发 0x4003 车命令）
     ⚠ 这一步发出的路线会直接给车执行，务必先做过验收（步骤一·2）
  3. 回滚：菜单输入 1（回到只落盘不发）

五、出问题怎么停
  · 双击「停止机载全栈」→ 整栈停掉
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
echo "桌面图标：启动机载全栈 / 机载规划验收 / 停止机载全栈"
echo "文本入口：~/start.sh（菜单式；也支持 ~/start.sh accept / stop / plan）"
echo "操作指南：~/Desktop/机载操作指南.txt"
echo "=================================================="
