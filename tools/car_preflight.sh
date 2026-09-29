#!/usr/bin/env bash
# 发车前置体检（**只读 + 短暂起底盘/定位验证**，不发任何控制命令）
#
# 为什么要有它：实地跑一局之前，最容易出事的三件都不在"我们的代码"里 ——
#   ① 官方栈残缺（`det_node` 只剩空壳 / `dzactuator` 根本没装）→ 底盘/相机起不来
#   ② **定位读不到** → 车不知道自己在哪，`/nav/waypoint` 一发就是盲跑
#   ③ 车上已经有别人在跑的节点 → 两套东西抢串口/话题，现象是"用着好好的突然不行"
# 这三件一次查清，并直接给出下一步该敲什么命令。
#
# 用法：python car.py preflight <车IP>
# 环境变量：AG_SKIP_HARDWARE=1   只做静态检查（不起底盘/定位，秒级返回）

set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null
[ -f "${AG_WS:-/tmp/ag_ws}/install/setup.bash" ] && source "${AG_WS:-/tmp/ag_ws}/install/setup.bash" 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1
SKIP_HW="${AG_SKIP_HARDWARE:-0}"

FAIL=0
PGRPS=""
start() {  # start <名字> <命令行...>
  local name="$1"; shift
  setsid "$@" > "/tmp/ag_pre_${name}.log" 2>&1 &
  PGRPS="$PGRPS $!"
}
cleanup() {
  for pg in $PGRPS; do kill -INT -"$pg" 2>/dev/null; done
  sleep 1
  for pg in $PGRPS; do kill -9 -"$pg" 2>/dev/null; done
  pkill -9 -f "${AG_WS:-/tmp/ag_ws}/install/" 2>/dev/null
  return 0
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }
say_fail() { FAIL=1; echo "  [FAIL] $*"; }
say_ok()   { echo "  [OK] $*"; }

echo "==================== 发车前置体检 ===================="
date
echo "  主机: $(hostname)   工作区: ${AG_WS:-/tmp/ag_ws}"
echo

hr; echo "## 0. 车上现在有没有别人在跑（两套节点会互相抢）"
NODES="$(timeout 10 ros2 node list 2>/dev/null)"
if [ -n "$NODES" ]; then
  echo "$NODES" | sed 's/^/    /'
  echo "  [注意] ROS 图不是空的 —— 确认不是别人在用；否则先停掉再体检："
  echo "         python car.py stop <车IP>"
else
  say_ok "ROS 图是空的"
fi
echo

hr; echo "## 1. 静态检查"
PKG_LIST="$(ros2 pkg list 2>/dev/null)"
echo "  -- 官方包（缺了跑不动） --"
for p in car_localization dzactuator dzjudgment; do
  if echo "$PKG_LIST" | grep -qx "$p"; then say_ok "$p"; else say_fail "缺 $p → python car.py restore <车IP>"; fi
done
echo "  -- 我们的包（缺了就是 /tmp 被清过） --"
for p in ag_car ag_bridge ag_referee; do
  if echo "$PKG_LIST" | grep -qx "$p"; then say_ok "$p"; else say_fail "缺 $p → python car.py deploy <车IP>"; fi
done
echo "  -- 视觉（可选，只有要用相机才必须） --"
for p in det_node dzrknn_dataprocess; do
  if echo "$PKG_LIST" | grep -qx "$p"; then say_ok "$p"; else echo "  [--] 缺 $p（不用视觉就不影响发车）"; fi
done

echo "  -- 关键源码是否存在 --"
PE="$HOME/dzrobot/src/car_localization/car_localization/path_executor.py"
[ -f "$PE" ] && say_ok "path_executor.py" || say_fail "找不到 $PE（执行路线的那一段没了）"
# ⚠️ 2026-09-29 实跑真因：源码存在 ≠ 起得来。
#    path_executor.py 里 `from car_localization.map_utils import ...`，而 map_utils
#    **只存在于源码目录**（官方装好的包里没有它）。直接 `python3 <源码>/path_executor.py`
#    会 ModuleNotFoundError 启动即崩 —— 表现为 /nav/waypoint 的订阅者是 0，
#    决策层命令发得出去、但**没人执行**，车纹丝不动，且链路上没有任何报错。
#    修法：把源码目录加进 PYTHONPATH。体检必须拦住它，否则每次都要重新踩一遍。
echo "  -- 路线执行器能不能真起来（不只是『文件在不在』） --"
PE_SRC="$HOME/dzrobot/src/car_localization"
if PYTHONPATH="$PE_SRC:${PYTHONPATH}" python3 -c "import car_localization.map_utils" 2>/dev/null; then
  say_ok "map_utils 可导入（源码里有、装好的包里没有）"
  case ":${PYTHONPATH}:" in
    *":$PE_SRC:"*) say_ok "本会话已带 PYTHONPATH=$PE_SRC（执行器能起来）" ;;
    *) echo "  [!] 本会话尚未导出 PYTHONPATH=$PE_SRC —— **起 path_executor 的脚本必须先导出它**，"
       echo "      否则执行器启动即崩、车不动，而链路上不会有任何报错" ;;
  esac
else
  say_fail "car_localization.map_utils 导不进来 → 执行器起不来 → 车不动。修法：export PYTHONPATH=\"$PE_SRC:\$PYTHONPATH\""
fi
echo "  -- 相机设备 --"
ls /dev/video* 2>/dev/null | sed 's/^/      /' || echo "      （没有 /dev/video*）"
echo "  -- 日志目录可写？ --"
LD="${AG_LOG_DIR:-/tmp/ag_logs}"
if mkdir -p "$LD" 2>/dev/null && touch "$LD/.w" 2>/dev/null; then
  rm -f "$LD/.w"; say_ok "$LD 可写（黑匣子日志会落这里）"
else
  say_fail "$LD 不可写（复盘就没数据了）"
fi
echo "  -- 磁盘 --"
df -h / | tail -1 | sed 's/^/      /'
echo

if [ "$SKIP_HW" = "1" ]; then
  hr; echo "## 2. 硬件项（定位）—— 已按 AG_SKIP_HARDWARE=1 跳过"
  echo "  ⚠️ 没验定位就等于没验最关键的一项；真发车前务必不带这个变量再跑一次"
else
  hr; echo "## 2. 硬件项：起底盘 + 定位，验「定位到底能不能读」（最关键的检查）"
  start actuator ros2 launch dzactuator bringup.launch.py
  sleep 12
  grep -E "serial port opened|can not open" /tmp/ag_pre_actuator.log 2>/dev/null | tail -2 | sed 's/^/      /' \
    || echo "      （底盘日志里没看到串口信息）"
  start localization ros2 launch car_localization localization.launch.py
  sleep 10
  echo "  -- 定位节点日志尾部 --"
  tail -6 /tmp/ag_pre_localization.log 2>/dev/null | sed 's/^/      /'

  echo
  echo "  -- 相关话题（不猜名字，先看真实存在哪些） --"
  TOPICS="$(ros2 topic list 2>/dev/null)"
  echo "$TOPICS" | grep -iE "odom|localiz|pose|amcl" | sed 's/^/      /' || echo "      （一个都没有）"

  echo
  echo "  -- 逐个验活 --"
  LOC_OK=""
  for t in /odom/localization /odom /localization/odom /amcl_pose /robot_pose; do
    if echo "$TOPICS" | grep -qx "$t"; then
      RATE="$(timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' | tr -s ' ')"
      if [ -n "$RATE" ]; then
        say_ok "$t 有数据（$RATE）"
        [ "$t" = "/odom/localization" ] && LOC_OK=1
        timeout 8 ros2 topic echo --once "$t" 2>/dev/null | head -8 | sed 's/^/          /'
      else
        echo "  [--] $t 话题在，但 8 秒内没数据（没收敛 / 没启动）"
      fi
    fi
  done
  [ -z "$LOC_OK" ] && say_fail "发车要用的 /odom/localization 没有可用数据 —— **现在不能真发车**"
fi

hr; echo "## 结论"
if [ "$FAIL" = "0" ]; then
  echo "  [OK] 体检通过，可以往下走。建议顺序："
  echo "     python car.py move <车IP> --with-vision          # 先干跑，看路线对不对（车不动）"
  echo "     python car.py move <车IP> --really-move --run-s 60   # ★确认无误、有人在场再真发车★"
  echo "     python car.py logs <车IP>                        # 跑完取日志出复盘"
else
  echo "  [FAIL] 有上面标 [FAIL] 的项，按它给的命令修完再跑（**先别真发车**）"
fi
echo "==================== 结束（本脚本启动的节点已停止） ===================="
