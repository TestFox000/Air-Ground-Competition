#!/bin/bash
# ============================================================
#  小车一键启动 / 停止 / 重启
#     ~/dzrobot/start_all.sh            启动全部节点
#     ~/dzrobot/start_all.sh stop       停止全部节点
#     ~/dzrobot/start_all.sh restart    重启全部节点
#
#  启动顺序：底盘 -> 相机 -> 定位 -> 视觉 -> 云台 -> 循迹 -> 网页弹窗
#  循迹节点启动后会在 AUTO_START_DELAY(默认8s) 后自动下发内置路线
#
#  ★ 出车前建议先跑一次体检：  ~/dzrobot/tools/healthcheck.sh
#     它会把"参数改了不起作用"这类静默故障提前揪出来。
# ============================================================
set -u

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
  # 网页弹窗服务可能不在 $WS 路径下启动（比如手动 cd 到 tools 里跑）
  for _p in $(ss -lntp 2>/dev/null | grep ':8080' | grep -oP '(?<=pid=)[0-9]+'); do
    kill -9 "$_p" 2>/dev/null
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
  if ss -lntp 2>/dev/null | grep -q ':8080'; then
    echo "  ⚠️  8080 仍被占用 —— 网页弹窗可能起不来，可手动处理后再试"
  fi
  pkill -f "chromium-browser.*127.0.0.1:8080" 2>/dev/null
  echo "已停止 $n 个受管节点。"
}

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

# ---- 配置中心自检：静默失败是所有"改了参数不生效"问题的根源，启动时先报出来 ----
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

echo "1/8 底盘串口 + TF + IMU ..."
spawn 1_chassis ros2 launch dzactuator bringup.launch.py
sleep 3

echo "2/8 相机 ..."
spawn 2_camera ros2 launch dzactuator camera.launch.py
sleep 3

echo "3/8 定位 (serial_reader -> /odom/localization) ..."
spawn 3_localization ros2 launch car_localization localization.launch.py
sleep 3

echo "4/8 视觉检测 (识别画面窗口 -> 显示 $GUI_DISPLAY) ..."
spawn 4_det ros2 launch det_node det_node.launch.py
sleep 3

echo "5/8 云台自瞄 ..."
spawn 5_gimbal ros2 launch dzrknn_dataprocess dzrknn_dataprocess.launch.py
sleep 2

# ★★★ 射击驱动 —— 少了这一步，开火就是"发进了空话题" ★★★
#   链路：dzrknn_dataprocess --/LaserShot_Command(UInt8=1)--> dzjudgment
#         --> 串口 /dev/dzjudgment 写 6 字节帧 B5 5B 03 21 01 25 --> 激光模块
#   参考《案例9：云台控制与射击》：dzshooting.launch.py 必须同时启动
#   dzrknn_dataprocess（云台控制）+ dzjudgment（上位机与车载裁判系统通讯）。
#   以前只起了前者，导致 ros2 topic info /LaserShot_Command 显示
#   Publisher 3 / Subscription 0 —— 云台算得好好的开火指令全进了空话题。
echo "6/8 射击驱动 (dzjudgment -> /dev/dzjudgment) ..."
spawn 6_judgment ros2 launch dzjudgment judgment.launch.py
sleep 2

echo "7/8 循迹 (启动后约 8 秒自动出发) ..."
spawn 7_path python3 "$WS/src/car_localization/car_localization/path_executor.py"
sleep 2

LAN_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "8/8 摄像头网页弹窗 + 自动打开浏览器 ..."
spawn 8_camview python3 "$WS/tools/cam_viewer.py" --port 8080
sleep 3

# ★自动弹出浏览器观察端（显示在 GUI_DISPLAY 指定的那块屏上）
#   默认开：省得每次手动开浏览器。想关掉就  OPEN_BROWSER=0 ./start_all.sh restart
#   用 --app= 模式：没有地址栏和标签页，看起来就像个原生监视窗口。
OPEN_BROWSER="${OPEN_BROWSER:-1}"
if [ "$OPEN_BROWSER" = "1" ]; then
  # 先等一下 viewer 真的起来，免得浏览器打开时是"无法连接"
  for _i in 1 2 3 4 5 6 7 8 9 10; do
    if curl -s -o /dev/null --max-time 1 "http://127.0.0.1:8080/api/status"; then break; fi
    sleep 1
  done
  if command -v chromium-browser >/dev/null 2>&1; then
    DISPLAY="$GUI_DISPLAY" nohup chromium-browser \
        --app="http://127.0.0.1:8080" --start-maximized \
        --no-first-run --no-default-browser-check \
        > "$LOGDIR/8_browser.log" 2>&1 &
    echo "       浏览器(chromium) -> 显示 $GUI_DISPLAY"
  elif command -v firefox >/dev/null 2>&1; then
    DISPLAY="$GUI_DISPLAY" nohup firefox "http://127.0.0.1:8080" \
        > "$LOGDIR/8_browser.log" 2>&1 &
    echo "       浏览器(firefox) -> 显示 $GUI_DISPLAY"
  else
    echo "       ⚠️  未找到浏览器，请手动打开 http://127.0.0.1:8080"
  fi
fi

echo ""
echo "=========================================================="
echo " 全部启动完成。日志目录: $LOGDIR"
echo ""
echo "  ★ 摄像头实时画面(浏览器打开):"
echo "        http://${LAN_IP:-<板子IP>}:8080"
echo ""
echo "   tail -f $LOGDIR/5_gimbal.log        # 云台自瞄  ★最关键"
echo "   tail -f $LOGDIR/4_det.log           # 视觉检测  ★看模型是否加载成功"
echo "   tail -f $LOGDIR/6_judgment.log      # 射击驱动  ★看开火指令是否下发"
echo "   tail -f $LOGDIR/7_path.log          # 循迹      ★看是否自动出发"
echo "   tail -f $LOGDIR/8_camview.log       # 网页弹窗服务"
echo "   tail -f $LOGDIR/1_chassis.log       # 底盘串口"
echo ""
echo "  浏览器观察端:      http://127.0.0.1:8080  (已自动打开)"
echo "  出车体检:          $WS/tools/healthcheck.sh"
echo "  狙击链路自检:      ros2 topic info /LaserShot_Command   # Subscription 必须是 1"
echo "  自瞄数值实时监控:  $WS/monitor_aim.sh"
echo "  一键停止:          $0 stop"
echo "=========================================================="
