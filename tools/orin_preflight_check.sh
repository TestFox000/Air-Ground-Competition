#!/usr/bin/env bash
# =============================================================================
# 起飞前 30 秒体检：多实例 / 定位链健康 / 飞控链路 / 环境——全绿才允许起飞。
#
# 背景（2026-10-26 悬停失控事故，见项目记忆 hover-incident-20261026）：
#   双 px4_odom_convert / 双 localization 同发关键话题会撕扯 EKF2 触发复位
#   风暴，起飞即失控；numpy 2.x 会杀定位 Python 侧；墙钟 1970 使日志失真。
#   本工具把这些教训变成硬判据：任何一项 FAIL 就不要解锁。
#
# 用法（Orin 上，栈已由 orin_bringup.sh up 启动并沉降 2~3 分钟后）:
#   bash tools/orin_preflight_check.sh
# 退出码 0=全绿可飞；1=有红项（输出会写明每一项怎么排查）。
# =============================================================================
set -uo pipefail

SESSION=agup
STATE_DIR=/tmp/agup_${SESSION}
PASS=0; FAIL=0

ok()   { printf '\e[1;32m[PASS]\e[0m %s\n' "$*"; PASS=$((PASS+1)); }
bad()  { printf '\e[1;31m[FAIL]\e[0m %s\n' "$*"; FAIL=$((FAIL+1)); }
head_() { printf '\e[1;36m==>\e[0m %s\n' "$*"; }

# 节点名单与 orin_bringup.sh 同口径（bash 包装行不算节点本体）
PROCS_X=(MicroXRCEAgent px4_odom_convert px4_imu_bridge fastlio_mapping
         livox_ros_driver2_node dzuav px4_control_node path_follow_node
         sensor_combined_to_imu)
PROCS_F=(global_localization[.]py transform_fusion[.]py
         global_map_publisher[.]py orin_plan_sidecar[.]py)

head_ "1) 栈存活"
if tmux has-session -t "$SESSION" 2>/dev/null; then
    ok "tmux 会话 $SESSION 存在（$(tmux list-windows -t $SESSION 2>/dev/null | wc -l) 窗）"
else
    bad "tmux 会话 $SESSION 不存在——先 bash tools/orin_bringup.sh up"
fi
deaths=$(cat "$STATE_DIR"/*.rc 2>/dev/null | wc -l)
if (( deaths == 0 )); then ok "无窗口死亡（.rc 全空）"
else
    bad "有 $deaths 个窗口已退出（ls $STATE_DIR/*.rc 看是谁；tmux attach -t $SESSION 排查）"
fi

head_ "2) 单实例（多实例=两路同发关键话题，撕扯 EKF）"
inst_bad=0
for n in "${PROCS_X[@]}"; do
    c=$(pgrep -xc "$n" 2>/dev/null || true)
    (( ${c:-0} > 1 )) && { bad "$n 有 $c 个实例"; inst_bad=1; }
done
for n in "${PROCS_F[@]}"; do
    c=$(pgrep -af "$n" 2>/dev/null | grep -cv bash || true)
    (( ${c:-0} > 1 )) && { bad "$n 有 $c 个实例"; inst_bad=1; }
done
(( inst_bad == 0 )) && ok "全部关键节点单实例"
(( inst_bad == 1 )) && bad "处理：tools/check_stack_health.py --kill-dup，或 orin_bringup.sh down 后重新 up"

head_ "3) 定位→EKF 链路健康（px4_odom_convert 最新 health 行）"
H=$(tmux capture-pane -t "$SESSION:odom" -p -J -S -600 2>/dev/null | grep health | tail -1)
if [[ -z $H ]]; then
    bad "读不到 health 行（odom 窗口无输出？栈可能没起）"
else
    val() { grep -oE "$1=[-0-9.]+" <<<"$H" | head -1 | cut -d= -f2; }
    chk() { # 名字 实际值 判OK的awk条件 阈值描述
        local n=$1 v=$2 cond=$3 desc=$4 verdict
        verdict=$(awk -v v="$v" -v c="$cond" 'BEGIN{print (v "" != "" && v c) ? 1 : 0}')
        if [[ $verdict == 1 ]]; then ok "$n = $v"; else bad "$n = ${v:-缺}（要求 $desc）"; fi
    }
    chk aligned   "$(val aligned)"   "==1"          "1"
    chk ekf_valid "$(val ekf_valid)" "==1"          "1"
    chk slam_hz   "$(val slam_hz)"   ">=30"         "≥30 Hz"
    chk storm     "$(val storm)"     "==0"          "0（复位风暴中=禁飞）"
    chk jump_max  "$(val jump_max_m)" "<=0.05"      "≤0.05 m"
    chk resets    "$(val ekf_reset_60s)" "<=2"      "≤2 次/分"
    if [[ $(val aligned) != 1 ]]; then
        printf '      提示：刚起栈需 2~3 分钟收敛，aligned=0 时先等再重跑本体检\n'
    fi
fi

head_ "4) 话题频率（/localization ≥30Hz，飞控 local_position 在发）"
# ROS setup 脚本在 set -u 下会因未绑定环境变量直接退出——source 前后局部放开
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzpx4/install/setup.bash" 2>/dev/null
set -u
hz=$(timeout 8 ros2 topic hz /localization 2>/dev/null | grep -oE 'average rate: [0-9.]+' | head -1 | awk '{print $3}')
if awk -v h="${hz:-0}" 'BEGIN{exit !(h>=30)}'; then ok "/localization ${hz} Hz"
else bad "/localization ${hz:-无} Hz（<30：FAST-LIO/global_loc 链路降级或未收敛）"; fi
# 注意不能用 grep -q：提前关管道会让 ros2 收 SIGPIPE 退出 141，pipefail 下整条
# 管道被误判失败（2026-10-26 实测 85Hz 健康话题被本项误报红）。
lpz=$(timeout 12 ros2 topic hz /fmu/out/vehicle_local_position 2>/dev/null \
      | grep -oE 'average rate: [0-9.]+' | head -1 | awk '{print $3}')
if awk -v h="${lpz:-0}" 'BEGIN{exit !(h>0)}'; then
    ok "飞控 /fmu/out/vehicle_local_position ${lpz} Hz（uXRCE 链路活）"
else
    bad "飞控 local_position 无输出（查 px4 窗口 MicroXRCEAgent 与飞控供电/串口）"
fi

head_ "5) 环境"
now=$(date +%s 2>/dev/null || echo 0)
if (( now > 1735689600 )); then ok "墙钟 $(date '+%F %T')"; else bad "墙钟未对（1970）——sudo date -s 对时"; fi
np=$(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null)
if [[ $np == 1.26.* ]]; then ok "numpy $np"; else bad "numpy ${np:-?}（须 1.26.x，2.x 会杀定位 Python 侧——bash tools/orin_fix_pyvision.sh）"; fi

echo
if (( FAIL == 0 )); then
    printf '\e[1;32m================ 体检全绿（%d 项），可以起飞 ================\e[0m\n' "$PASS"
    exit 0
else
    printf '\e[1;31m================ %d 项红，禁止起飞（按上面各行排查） ================\e[0m\n' "$FAIL"
    exit 1
fi
