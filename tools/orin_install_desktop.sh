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
GUIDE="$DESK/操作指南.txt"

ok()   { echo "[通过] $*"; }
fail() { echo "[失败] $*" >&2; exit 1; }

[ -d "$REPO" ] || fail "找不到机载仓库 $REPO"
mkdir -p "$DESK"

# ---------------------------------------------------------------- ~/start.sh
# 薄壳：只做「找到仓库 → 交给 tools/orin_menu.sh」，菜单与报错逻辑全部留在
# 仓库源文件里。改菜单不用重刷机器，也不会出现"桌面副本与源码漂移"。
# 原子替换：正在菜单里的会话仍读旧 inode，不会因截断报 unexpected EOF。
cat > "$HOME/.start.sh.new" <<'START'
#!/usr/bin/env bash
# 机载一键入口（薄壳）。真正的菜单与全部诊断逻辑在 tools/orin_menu.sh。
set -u
REPO="$HOME/dzuav_deploy/air_ground"
if [ ! -d "$REPO" ]; then
    echo "[失败] 找不到机载仓库: $REPO"
    echo "  先在电脑上跑 tools/orin_deploy.py 部署，或检查 Type-C 挂载"
    [ -t 0 ] && read -r -p "回车退出…" _
    exit 9
fi
if [ ! -f "$REPO/tools/orin_menu.sh" ]; then
    echo "[失败] 仓库缺少 tools/orin_menu.sh（部署不完整或旧版本）"
    echo "  处理：重新部署仓库，再跑 bash tools/orin_install_desktop.sh"
    [ -t 0 ] && read -r -p "回车退出…" _
    exit 9
fi
exec bash "$REPO/tools/orin_menu.sh" "$@"
START
chmod +x "$HOME/.start.sh.new"
mv -f "$HOME/.start.sh.new" "$HOME/start.sh"
ok "已安装 ~/start.sh（薄壳 → tools/orin_menu.sh，改菜单无需重刷桌面）"

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

mk_item "ag-plan-start.desktop"  "① 启动全栈"     "up"
mk_item "ag-plan-accept.desktop" "② 规划验收"     "accept"
mk_item "ag-plan-stop.desktop"   "③ 停止全栈"     "stop"
mk_item "ag-recon.desktop"       "④ 视频侦察"     "recon"
mk_item "ag-diag.desktop"        "⑤ 一键体检"     "diag"
mk_item "ag-plan-menu.desktop"   "更多操作"       "menu"
ok "已安装桌面图标：启动 / 验收 / 停止 / 视频侦察 / 更多操作（双击即执行，执行完自动回菜单）"

# ---------------------------------------------------------------- 桌面 .sh（可直接运行）
# 用户要求"sh 化"：桌面除 .desktop 外再放一个**真 .sh**，双击/终端都能直接跑，
# 不必记仓库路径。收尾带 pause，避免双击跑完窗口就关掉看不到结果。
cat > "$DESK/.视频侦察.sh.new" <<'SH'
#!/usr/bin/env bash
# 桌面直跑入口：任意视频 → 识别 → 路线规划 → 渲染图
#   ./视频侦察.sh                    列出机上视频、选一个跑（默认 stride 5 实测最优点）
#   ./视频侦察.sh ~/a.mp4 --stride 15   # 更稀：会漏雷（实测只剩 3/8）
#   ./视频侦察.sh --replay-last      秒级：用最近一次的识别结果重出规划
set -u
REPO="$HOME/dzuav_deploy/air_ground"
cd "$REPO" || { echo "找不到仓库: $REPO"; read -r -p "回车关闭…" _; exit 9; }
bash tools/orin_video_recon.sh "$@"
echo
read -r -p "按回车返回菜单…" _ || true
exec "$HOME/start.sh" menu
SH
chmod +x "$DESK/.视频侦察.sh.new"
mv -f "$DESK/.视频侦察.sh.new" "$DESK/视频侦察.sh"
ok "已安装 ~/Desktop/视频侦察.sh（可直接双击/终端运行，跑完自动回菜单）"

# ---------------------------------------------------------------- 操作指南
cat > "$GUIDE" <<'TXT'
机载操作指南（VNC 桌面前照着做即可；图标名已按桌面显示宽度缩写）
=====================================

【一句话】桌面上双击「① 启动全栈」→ 它会先做启动前检查、然后直接拉起全栈，
         拉起后**自动显示数字菜单**（4=验收 5=看窗口 6=看产物 7=停止 8=视频侦察
         d=一键体检 b=启停广播 0=退出）。
         如果你在任何时候找不到菜单：打开终端敲  ~/start.sh  回车即可。
         菜单与所有报错处理逻辑都在仓库 tools/orin_menu.sh——改菜单只改那里，
         ~/start.sh 只是薄壳，不必重刷桌面。

〇、只想看「视频 → 路径规划」（不需要起全栈、不需要相机）——**通用，任意视频**
  1. 双击桌面「④ 视频侦察」或「视频侦察.sh」
     · 无参数时它**列出机上视频**让你输序号（也会认你直接粘的路径）
     · 选完就跑：识别 → 格号反推 → 路线 → 车命令 → 渲染图，跑完自动打开图
     · 前置件（cv2 / 13 类模型 / 中文字体）缺哪项它会说清修法
  2. 秒级重出规划：菜单输入 9（或 ./视频侦察.sh --replay-last）
     · 复用上次识别结果，只重跑规划层，几秒出图（调参数、看路线时用这个）
  3. 终端等价命令（VNC 里可手敲）：
        cd ~/dzuav_deploy/air_ground
        ./tools/orin_video_recon.sh                    # 选视频跑
        ./tools/orin_video_recon.sh ~/a.mp4 --stride 15   # 更稀：会漏雷
        ./tools/orin_video_recon.sh --replay-last      # 秒级
  4. 采样帧距（--stride）怎么选：
        5  = 默认，**实测最优点**（同一段视频 8/8 = 官方雷基线）
        15 = 太稀：只剩 3 雷（采样漏检）
        1  = 太密：14 雷（7 个多余）——弱误检累积到 3 票后混入，幻影消除还会误杀真雷
        ！不是"越密越好"也不是"越快越好"，两端都会掉精度
        同一视频同参数跑第二次会自动复用识别结果，并用**当前门控**重算态势
        （秒级；门控修过也不会继续拿旧结论）
  ⚠ 渲染图上的 D8* 是移动靶的“确认位”。**静态回放**（没有实时红蓝复判）请加
     `--replay-safe`，脚本会在机器级剔掉该停靠位（别手改路线）；在飞链路仍有
     fire_guard 复判（红面/未辨自动禁火）。

一、开机后要做的事（每次上电）
  1. 双击「① 启动机载全栈」
     · 先看它打印的「启动前检查」：相机/二进制是否 13 类/引擎是否 d13+v4d
     · 然后拉起：感知(dzuav) / 机载规划(plan) / 雷达 / SLAM定位 / 飞控桥 / 控制 / 航线
     · 规划窗口默认「只落盘，不发车命令」——安全档
     · 若最后报「全栈未启动 · 会话外已在跑 MicroXRCEAgent」：那是 autostart 的
       看门狗已经占了飞控链路。先停它再重来：
           pkill -f orin_agent_watchdog.py ; pkill -x MicroXRCEAgent
       停之前用 ls -l /dev/serial/by-id/ 认一下：**NxtPX4 才是飞控**
       （脚本已改为按 by-id 自动选串口，不再写死 ttyACM0）
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
rm -f "$DESK/机载操作指南.txt" 2>/dev/null || true   # 旧长名会被桌面截断，改为短名
ok "已写入桌面《操作指南.txt》"

echo
echo "==================== 安装完成 ===================="
echo "桌面图标：① 启动全栈 / ② 规划验收 / ③ 停止全栈 / ④ 视频侦察 / ⑤ 一键体检 / 更多操作"
echo "          （双击即执行；执行完自动回到数字菜单）"
echo "找不到菜单时：打开终端敲  ~/start.sh  回车"
echo "文本入口：~/start.sh（也支持 up / up-send / accept / stop / recon / recon-replay / plan / diag / broadcast-start / broadcast-stop）"
echo "操作指南：~/Desktop/操作指南.txt"
echo "=================================================="
