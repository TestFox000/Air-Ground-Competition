#!/bin/bash
# ============================================================
#  小车一键启动 / 停止 / 重启
#     ~/dzrobot/start_all.sh            启动全部节点
#     ~/dzrobot/start_all.sh stop       停止全部节点
#     ~/dzrobot/start_all.sh restart    重启全部节点
#
#  启动顺序：底盘 -> 相机 -> 定位 -> 视觉 -> 云台 -> 循迹 -> 网页弹窗
#  循迹节点启动后会在 mission.auto_start_delay_s(默认10s) 后自动下发内置路线
#
#  ★ 自检只在发车前 dzbrain 自己的准备窗口内做（mission.auto_start_delay_s=10：
#     就绪日志 / 定位与安全初始化 / 出发前路线加载校验）；起栈本身不再跑额外门禁。
#     出车体检（可选，人工按需）可另跑：  ~/dzrobot/tools/healthcheck.sh
# ============================================================
set -u

# ★ 本回合起过的节点台账 "名字:pid"（WB-22 存活检查用）。
#   显式声明为空数组：在 `set -u` 下对未声明变量做 `+=` 会抛 "unbound variable"。
SPAWNED=()

WS=/home/duzhong/dzrobot
LOGDIR=$HOME/robot_logs
PIDDIR=$LOGDIR/pids
mkdir -p "$LOGDIR" "$PIDDIR"

# 配置中心（全车唯一改参数的地方）。可用 ROBOT_CONFIG 临时指向另一份做 A/B 对比。
CONFIG="${ROBOT_CONFIG:-$WS/config/robot.yaml}"

# GUI 窗口(视觉识别画面)输出到哪个显示：
#   :1 = VNC 桌面  <- 默认。想通过 VNC 观察识别画面就用这个
#   :0 = 板子物理 HDMI 显示器
# 两者是隔离的 X 服务器，窗口不能跨屏，必须起对。
# 可临时覆盖：GUI_DISPLAY=:0 ./start_all.sh restart
GUI_DISPLAY="${GUI_DISPLAY:-:1}"

# ==============================================================================
#  进程管理
# ==============================================================================
#  ★ 为什么不再用 `pkill -f <名字>`：
#    ① 模糊匹配容易误伤（能匹配到别的进程，甚至匹配到正在跑本脚本的 shell）；
#    ② 实测在某些执行环境下 pkill 会【漏杀】—— 进程属主正确、kill -9 有效，
#       pkill 就是匹配不到。残留后果很严重，已经出过两次事故：
#         · 12 个 static_transform_publisher 残留 = 多套底盘栈抢 TF
#         · 相机抢不到 /dev/video0 直接 abort(exit -6) → 浏览器"无信号"
#
#  ★ 现在改成【进程组精确回收】：
#      启动时用 setsid 让每个节点自成进程组，stop 时对【整个进程组】发信号，
#      ros2 launch 派生的所有子进程会被一并回收。既精确又彻底。
# ==============================================================================

spawn() {                       # spawn <日志名> <命令...>
  local name="$1"; shift
  local extra=()
  # 视觉节点的 OpenCV 窗口要显式指定 DISPLAY，否则窗口会开到物理显示器上，
  # VNC 里永远看不到识别画面。
  if [ "$name" = "4_det" ]; then
    extra=(env "DISPLAY=$GUI_DISPLAY")
  fi
  setsid "${extra[@]}" "$@" > "$LOGDIR/${name}.log" 2>&1 &
  local pid=$!
  echo "$pid" > "$PIDDIR/${name}.pid"
  # ★ 记进"本回合起过的节点"台账，供收尾的存活检查用（WB-22）。
  #   为什么不在这里立刻 `kill -0`：节点刚起来时可能还在 exec／加载库的中途，
  #   立刻判会把「正在启动」误报成「启动即退出」。统一放到收尾（那时各步都已 sleep 过）。
  SPAWNED+=("${name}:${pid}")
  printf "        pid=%-7s log=%s\n" "$pid" "$LOGDIR/${name}.log"
}

kill_group() {                  # kill_group <pid>  先温和后强制，回收整个进程组
  local pid="$1"
  [ -z "$pid" ] && return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
  local i
  for i in 1 2 3 4 5 6; do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.5
  done
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
}

cleanup_strays() {
  # 兜底：清掉"属于本项目工作空间"但不在 PID 记录里的残留进程。
  # 判别依据是【工作空间绝对路径】—— 精确、不会误伤系统进程。
  local _self=$$ _pid _cmd _n=0
  for _p in /proc/[0-9]*; do
    _pid=${_p#/proc/}
    [ "$_pid" = "$_self" ] && continue
    _cmd=$(tr '\0' ' ' < "$_p/cmdline" 2>/dev/null) || continue
    case "$_cmd" in
      *"$WS"*) ;;
      *) continue ;;
    esac
    # 别把正在执行本脚本的 shell 杀掉
    case "$_cmd" in */bin/bash*|*/bin/sh*) continue ;; esac
    kill -9 "$_pid" 2>/dev/null && _n=$((_n+1))
  done
  [ "$_n" -gt 0 ] && echo "  已清理 $_n 个上次遗留的残留进程"
  return 0
}

stop_all() {
  echo "停止全部节点..."
  local n=0
  if [ -d "$PIDDIR" ]; then
    for f in "$PIDDIR"/*.pid; do
      [ -e "$f" ] || continue
      local pid
      pid=$(cat "$f" 2>/dev/null)
      if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill_group "$pid"
        n=$((n + 1))
      fi
      rm -f "$f"
    done
  fi
  cleanup_strays
  sleep 1
  echo "已停止 $n 个受管节点。"
}

# ==============================================================================
#  ★ ROS 域号：钉在这里，不再依赖 ~/.bashrc（2026-10-07 无人机↔小车联调事故）
#    · 交互式终端起栈              → 读 ~/.bashrc → 118
#    · systemd / cron / ssh 一行命令 → 不读 ~/.bashrc → 默认 0
#    两种起法域号不同，症状一模一样且【一个错都不报】：
#      ping 通、SSH 通、网页台 :8080 通，但 ROS 死活看不见对方。
#    ★ 改域号只改一个文件：$WS/dzros.env（无人机侧对应 ~/dzlink_tools/dzlink.env）
#    外部已显式 export 的 ROS_DOMAIN_ID 仍优先（临时 A/B 用），但启动横幅会报警。
# ==============================================================================
if [ -f "$WS/dzros.env" ]; then . "$WS/dzros.env"; fi    # ★ 全车唯一真相源
export DZ_ROS_DOMAIN="${DZ_ROS_DOMAIN:-118}"             # 兜底：上面文件缺失时才用它（2026-10-29 补 export：子进程要核对）
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$DZ_ROS_DOMAIN}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# ---- stop / restart 需要在 stop_all 之前就把 ROS 环境准备好（cleanup_strays 用不到，但保持顺序清晰）----
# 注意：ROS 的 setup.bash 内部会展开未定义变量 AMENT_TRACE_SETUP_FILES
#       （/opt/ros/humble/setup.bash 第 8 行），与脚本开头的 `set -u` 冲突，
#       会让脚本刚启动就报"未绑定的变量"并直接退出。
#       所以在 source 期间临时关闭 nounset，之后立即恢复。
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

case "${1:-}" in
  stop)    stop_all; exit 0 ;;
  restart) stop_all; sleep 2 ;;
esac

stop_all   # 先清场，避免残留进程抢话题

# ---- 配置中心 YAML 可解析守卫（瞬时、不算自检）：解析失败或顶层裸键会让相机/视觉/云台
#      三节点 abort，所以起栈前先报出来（不是门禁，只提示不拦截） ----
echo "配置中心: $CONFIG"
if [ -f "$CONFIG" ]; then
  if python3 -c "import yaml,sys; yaml.safe_load(open(sys.argv[1],encoding='utf-8'))" "$CONFIG" 2>/dev/null; then
    echo "        ✅ 可解析"
  else
    echo "        ❌ 解析失败 —— 各节点会回退到代码内置默认值，请检查 YAML 语法！"
  fi
else
  echo "        ❌ 文件不存在 —— 各节点会回退到代码内置默认值"
fi
echo ""

# ---- 链路自检：域号不一致是最阴的静默故障（对端不同域时 ROS 一个错都不报）----
echo "ROS 链路域号: ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY  (车标准域 $DZ_ROS_DOMAIN)"
if [ "$ROS_DOMAIN_ID" != "$DZ_ROS_DOMAIN" ]; then
  echo "        ⚠️  域号偏离标准值 —— 无人机侧必须跟着改 ~/dzlink_tools/dzlink.env，否则互相看不见"
fi

# ==============================================================================
#  ★ 车端无本地地图（2026-10-31）—— 起栈前只报一件事：规划依据来自无人机 field_layout
# ------------------------------------------------------------------------------
#  main.py 的分派（main.py:_layout_mode_enabled）：不传 --map ⇒ 布局模式，先起无人机线程
#  等 field_layout；没收到布局就不规划、不发车（绝不用本地地图/旧 route.tokens 兜底）。
#  为什么这里不再传 --map：config/map.yaml 已从工作树与车端删除；历史写法恒传 --map 却指向
#  一个不存在的文件 = 谎报意图；更要命的是谁把旧图塞回车上，就会**静默**切成文件模式。
#  ⚠️ 本段必须留在"节点 7"那段**之前**（出车信息越早看到越好），且**不得**插进节点 7 的
#     启动行内部（tests/test_deploy.py ⑥ 按结构锚点钉死该块的形状）。
# ==============================================================================
if [ -f "$WS/config/map.yaml" ]; then
  echo "⚠️⚠️ 发现残留地图文件 $WS/config/map.yaml"
  echo "        → 起栈不传 --map，仍是布局模式；但请确认这张图不该存在：rm -f $WS/config/map.yaml"
else
  echo "地图：无本地地图（布局模式）—— 规划依据由无人机 field_layout 下发"
fi
echo ""

# ---- 自检已收敛到发车前 auto_start_delay_s=10 准备窗口（dzbrain 就绪横幅），起栈路径不再跑任何门禁 ----

echo ""
echo "1/7 底盘串口 + TF + IMU ..."
spawn 1_chassis ros2 launch dzactuator bringup.launch.py
sleep 3

echo "2/7 相机 ..."
spawn 2_camera ros2 launch dzactuator camera.launch.py
sleep 3

echo "3/7 定位 (serial_reader -> /odom/localization) ..."
spawn 3_localization ros2 launch car_localization localization.launch.py
sleep 3

echo "4/7 视觉检测 (识别画面窗口 -> 显示 $GUI_DISPLAY) ..."
spawn 4_det ros2 launch det_node det_node.launch.py
sleep 3

echo "5/7 云台自瞄 ..."
spawn 5_gimbal ros2 launch dzrknn_dataprocess dzrknn_dataprocess.launch.py
sleep 2

# ★ 原 5b 步「自瞄状态兼容转发 (Int32 -> UInt8)」已随浏览器控制台一并下线。
#   它存在的唯一理由是网页端按 UInt8 订阅 /aim/status，而云台发的是 Int32；
#   网页端没了，这个 shim 就只剩一个没人订阅的转发节点 —— 白占一个进程和一份
#   DDS 带宽。**注意 /aim/status 本身没变**：dzbrain(contract.py) 直接订阅
#   Int32 原话题，不经过这个 shim，循迹不受影响。

# ★★★ 射击驱动 —— 少了这一步，开火就是"发进了空话题" ★★★
#   链路：dzrknn_dataprocess --/LaserShot_Command(UInt8=1)--> dzjudgment
#         --> 串口 /dev/dzjudgment 写 6 字节帧 B5 5B 03 21 01 25 --> 激光模块
#   参考《案例9：云台控制与射击》：dzshooting.launch.py 必须同时启动
#   dzrknn_dataprocess（云台控制）+ dzjudgment（上位机与车载裁判系统通讯）。
#   以前只起了前者，导致 ros2 topic info /LaserShot_Command 显示
#   Publisher 3 / Subscription 0 —— 云台算得好好的开火指令全进了空话题。
echo "6/7 射击驱动 (dzjudgment -> /dev/dzjudgment) ..."
spawn 6_judgment ros2 launch dzjudgment judgment.launch.py
sleep 2

echo "7/7 循迹（新决策层 dzbrain）..."
# ★2026-10-07 换血：默认走**新决策层** dzbrain（单进程 8 任务线程：定位 / 安全 / 循迹 /
#   调度 / 云台打击 / 误差修正 / 无人机 / 遥测），取代旧的 path_executor 单体（2622 行）。
# ★2026-10-31 旧核整块删除（用户指令）：path_executor.py / console_scripts 入口 / LEGACY_PATH
#   回退分支 / px 级 mission_blocks 路线全部移除 —— 车端只有一条循迹实现。
# ★★ 必须无缓冲 ★★：ROS2 Python 节点的日志走 stdout，
#   而 stdout 重定向到文件时是【块缓冲】(4KB 才 flush) —— 不加 -u 时
#   7_path.log 会长时间 0 字节，现场 tail -f 看不到任何日志，
#   极易误判成"循迹节点没起来/卡死"（2026-10-03 实测踩到，进程其实在跑）。
spawn 7_path env PYTHONUNBUFFERED=1 PYTHONIOENCODING=utf-8 \
  python3 -u -m dzbrain.main --config "$CONFIG" --force
sleep 2

# ★ 浏览器控制台已下线（2026-10-09）——原第 8 步 cam_viewer :8080 整块移除。
#   它不是"一个网页"：/api/param 是全车唯一的调参通道（白名单/类型校验/.bak 回滚
#   全在 param_spec.py 里），/api/rc* 的遥控器还兼着【接管仲裁 + 四层失效保护】
#   （rc_control.py，负责按住才动 / 数值钳制 / 两层看门狗），start_all.sh 因此不再拉起它。
#   ⚠️ 由此产生的真实缺口：**遥控器没有替代品了**。原先唯一的可视化遥控入口就是
#      这个网页；底盘 1.0s 无指令强制停车仍作为最底层兜底保留，但"人接管"这条路
#      现在只能自己写节点。起栈前请确认这点可接受。
#   调参改走 config/robot.yaml（healthcheck.sh 第 3 节查配置一致性）。
echo "（已跳过原第 8 步：浏览器控制台已下线）"

echo ""
# ==============================================================================
#  ★★ WB-22 · 收尾存活检查（**在打印"启动完成"之前**，2026-11-01）
# ------------------------------------------------------------------------------
#  为什么需要：spawn 是 `setsid ... &` 后台起进程，**不 wait 也不看 $?**；
#  全文没有 `set -e`。于是 dzbrain 因危险配置被引擎闸门拒绝（rc=7）时，
#  起栈脚本**跑完 rc 仍是 0**，而下面那句"全部启动完成"照样打印 ——
#  运维会直接去看云台/视觉日志，**没人回头确认 dzbrain 到底在不在**。
#  闸门做对了一半，那一半被这一层吞了。
#
#  判据（只加观察、**不改任何启动参数**）：
#    · 全部存活 ⇒ rc 0，措辞保持"全部启动完成"（成功路径一字不改）；
#    · 任一节点已死 ⇒ rc **非 0**，逐条点名「节点 X 启动后即退出」+ 给日志路径，
#      并且**不再打印**"全部启动完成"。
#
#  ⚠️ 为什么用 `kill -0` 而不是 `wait <pid>`：这些进程是 setsid 起的、与脚本
#     脱离进程组，`wait` 只能等直接子进程且会**阻塞到节点退出**（节点本该常驻）
#     ⇒ 必须用非阻塞的存活探测。实测（2026-11-01，开发机）：`( exit 1 ) &` 起的
#     进程两秒后 `kill -0` 正确判 DEAD（即子进程已退出、不是僵尸）。
#     ⚠️ **未在本机验证 `setsid` 分支**：开发机没有 `setsid`（`which setsid` 为空，
#        直接调用 rc=127）⇒ 本机跑不到车上那条路径。车上语义（setsid 后 $! 为
#        setsid 自身、其子为节点）**未实测**，故本卡只保证"记录了 spawn 返回的 pid
#        并在收尾查它是否存活"这一层；若车上 `setsid` 使 $! 与节点 pid 不一致，
#        该判据会失效——这一条留给真机验证，不在此声称已验证。
# ==============================================================================
DEAD_NODES=()
if [ "${#SPAWNED[@]}" -gt 0 ]; then
  for _entry in "${SPAWNED[@]}"; do
    _name="${_entry%%:*}"
    _pid="${_entry##*:}"
    if ! kill -0 "$_pid" 2>/dev/null; then
      DEAD_NODES+=("${_name}:${_pid}")
    fi
  done
fi

echo ""
echo "=========================================================="
if [ "${#DEAD_NODES[@]}" -gt 0 ]; then
  echo " ❌ 启动未完成：${#DEAD_NODES[@]}/${#SPAWNED[@]} 个节点启动后立即退出"
  for _entry in "${DEAD_NODES[@]}"; do
    _name="${_entry%%:*}"
    _pid="${_entry##*:}"
    echo "     · 节点 ${_name} 启动后即退出（pid=${_pid}，见 ${LOGDIR}/${_name}.log）"
  done
  echo ""
  echo "   日志尾部（先看这几行，多半是配置/依赖问题）："
  for _entry in "${DEAD_NODES[@]}"; do
    _name="${_entry%%:*}"
    echo "     ── ${_name} ──"
    tail -8 "$LOGDIR/${_name}.log" 2>/dev/null | sed 's/^/        /'
  done
  echo ""
  echo "  其余节点仍在运行；停止全部: $0 stop"
  echo "=========================================================="
  exit 1
fi
echo " 全部启动完成。日志目录: $LOGDIR"
echo ""
echo "   tail -f $LOGDIR/5_gimbal.log        # 云台自瞄  ★最关键"
echo "   tail -f $LOGDIR/4_det.log           # 视觉检测  ★看模型是否加载成功"
echo "   tail -f $LOGDIR/6_judgment.log      # 射击驱动  ★看开火指令是否下发"
echo "   tail -f $LOGDIR/7_path.log          # 循迹      ★看是否自动出发"
echo "   tail -f $LOGDIR/1_chassis.log       # 底盘串口"
echo ""
echo "  出车体检(可选):    $WS/tools/healthcheck.sh"
echo "  狙击链路自检:      ros2 topic info /LaserShot_Command   # Subscription 必须是 1"
echo "  自瞄数值实时监控:  $WS/monitor_aim.sh"
echo "  一键停止:          $0 stop"
echo "=========================================================="
