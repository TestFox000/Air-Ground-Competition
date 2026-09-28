#!/usr/bin/env bash
# 实机验证 · 开火闸门（**零位移、零发射**）
#
# 只启动：usb_cam + det_node + aim_guard_node
# **不启动** dzrknn_dataprocess（那才会扫云台并开火）、不启动 dzjudgment（串口开火通道）
#
# 做法：**双向对照**，不依赖"当前镜头前是蓝还是红"：
#   阶段 A：白名单 = 画面里当前的类别 -> 期望过滤流里**出现**目标（放行）
#   阶段 B：白名单 = 另一个类别        -> 期望过滤流**恒为全 0**（拦截）
#   两个方向都对，才说明闸门真的在做判断，而不是把链路掐死或全放行。
#   注意：判定必须在**闸门运行期间**采样（闸门一停，过滤流就没有发布者了）。
#
# ⚠️ 参数用**参数文件**传，别用 `-p allow_classes:=1`：
#    后者会被 YAML 解析成整数，而该参数声明为字符串 -> 节点直接崩
#    （真机踩过：InvalidParameterTypeException ... 'INTEGER', expecting 'STRING'）。
#
# ⚠️ 停止节点要连子进程一起停（pgrep -P），否则 ros2 launch 的子节点会变孤儿常驻。
#
# ⚠️ 跑之前先确认相机：本脚本依赖"画面里有个目标"。相机崩了就会跑出看不懂的判定，
#    所以第 1 步会检查 /image_raw 帧率并快速失败；也可先跑
#    tools/car_camera_check.sh 单独验相机。

set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
WS="${AG_WS:-/tmp/ag_ws}"
source "$WS/install/setup.bash" 2>/dev/null
[ -z "${DISPLAY:-}" ] && [ -e /tmp/.X11-unix/X0 ] && \
  export DISPLAY=:0 XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1

PIDS=()
GUARD=""

stop_tree() {   # 递归停掉进程树（ros2 launch 的子节点必须一起停）
  local pid="$1" child
  [ -z "$pid" ] && return 0
  for child in $(pgrep -P "$pid" 2>/dev/null); do stop_tree "$child"; done
  kill -INT "$pid" 2>/dev/null
}

cleanup() {
  [ -n "$GUARD" ] && kill -INT "$GUARD" 2>/dev/null
  for pid in "${PIDS[@]}"; do stop_tree "$pid"; done
  sleep 2
  local child
  [ -n "$GUARD" ] && kill -9 "$GUARD" 2>/dev/null
  for pid in "${PIDS[@]}"; do
    for child in $(pgrep -P "$pid" 2>/dev/null); do kill -9 "$child" 2>/dev/null; done
    kill -9 "$pid" 2>/dev/null
  done
  echo "  已停止本次启动的节点（含子进程）"
}
trap cleanup EXIT

hr() { echo "----------------------------------------------------"; }

# `--field data` 返回 numpy repr：array('i', [113, 55, 1, 80, ...])
# 去掉包装与标点，压成 "113 55 1 80 ..."；第 3 个数就是类别号。
sample() {
  # ⚠️ `ros2 topic echo` 会在每帧后打一行 `---` 分隔符，必须去掉：
  # 否则 is_blank 的 `^0( 0)*$` 永远匹配不上，"被拦成全 0"会被误判成放行。
  timeout 15 ros2 topic echo --once "$1" --field data 2>/dev/null \
    | sed "s/array('i', *\[*//; s/array('d', *\[*//; s/[])]//g" \
    | grep -v -- '^---$' \
    | tr -d ',' | tr '\n' ' ' | tr -s ' ' | sed 's/^ *//; s/ *$//'
}
is_blank() { [ -z "$1" ] || echo "$1" | grep -qE "^0( 0)*$"; }

# 采样 n 次，返回「出现过非零目标」的次数
count_hits() {
  local topic="$1" n="$2" i v hits=0 samples=""
  for i in $(seq 1 "$n"); do
    v=$(sample "$topic")
    samples="$samples|$v"
    is_blank "$v" || hits=$((hits+1))
    sleep 1
  done
  echo "$hits"
}

start_guard() {   # $1 = allow_classes 值
  cat > /tmp/ag_guard_params.yaml <<EOF
aim_guard_node:
  ros__parameters:
    input_topic: "/offset_center"
    output_topic: "/offset_center_guarded"
    class_blue: 1
    class_red: 2
    allow_classes: "$1"
    enable_fire: true
    publish_blocked_log_period_s: 0.0
EOF
  timeout 600 ros2 run ag_car aim_guard_node --ros-args \
    --params-file /tmp/ag_guard_params.yaml > "/tmp/ag_guard_$1.log" 2>&1 &
  GUARD=$!
  sleep 6
  if kill -0 $GUARD 2>/dev/null; then
    echo "  [OK] 闸门已启动（白名单=\"$1\"）"
  else
    echo "  [FAIL] 闸门启动失败："
    head -12 "/tmp/ag_guard_$1.log" | sed 's/^/    /'
    GUARD=""
    return 1
  fi
}

stop_guard() {
  [ -n "$GUARD" ] && kill -INT "$GUARD" 2>/dev/null
  sleep 2
  [ -n "$GUARD" ] && kill -9 "$GUARD" 2>/dev/null
  GUARD=""
  sleep 1
}

echo "==================== 开火闸门实机验证（零位移/零发射） ===================="
date
echo

hr; echo "## 0. 现有节点（若已有别人在跑，需先确认）"
timeout 12 ros2 node list 2>/dev/null | sed 's/^/  /'
echo

hr; echo "## 1. 起相机 + 检测（不产生任何动作）"
timeout 600 ros2 launch dzactuator camera.launch.py > /tmp/ag_b_cam.log 2>&1 &
PIDS+=($!); sleep 8
timeout 600 ros2 launch dzactuator det_node.launch.py > /tmp/ag_b_det.log 2>&1 &
PIDS+=($!); sleep 12
grep -q "Can't initialize GTK backend" /tmp/ag_b_det.log && echo "  ⚠️ det_node 因 GTK 崩了（DISPLAY 未生效）"
grep -q "process has died" /tmp/ag_b_cam.log 2>/dev/null && echo "  ⚠️ usb_cam 崩了（真机踩过 exit code -6 / SIGABRT）"

# ⚠️ 先确认相机真的有图像，再往下跑。
# 2026-09-27 有一次 usb_cam 崩了，det_node 只会无限刷 "waiting for /image_raw"，
# 于是下面第 2 步读不到类别、第 5 步判定退化成莫名其妙的 SKIP/FAIL ——
# 白白跑两分钟还得出个看不懂的结论。与其事后猜，不如在这里快速失败。
CAM_HZ=$(timeout 20 ros2 topic hz /image_raw 2>/dev/null | grep -m1 "average rate")
if [ -z "$CAM_HZ" ]; then
  echo "  [FAIL] /image_raw 没有帧 —— 相机没起、崩了，或话题名不对"
  echo "         先单独跑：python tools/remote_run.py --host <IP> --user duzhong \\"
  echo "                   --script tools/car_camera_check.sh"
  exit 1
fi
echo "  相机: $CAM_HZ"
echo

hr; echo "## 2. 画面里现在是什么目标"
RAW=""
for i in 1 2 3 4 5 6; do
  RAW=$(sample /offset_center)
  is_blank "$RAW" || break
  sleep 1
done
echo "  原始（/offset_center）: $RAW"
CLS=$(echo "$RAW" | awk '{print $3}')
echo "  类别 cls = ${CLS:-?}   （1=蓝靶=敌方  2=红靶=友方）"
case "$CLS" in
  1) OTHER=2 ;;
  2) OTHER=1 ;;
  *) OTHER=2; echo "  ⚠️ 没读到有效类别，判定阶段会 SKIP" ;;
esac
echo

hr; echo "## 3. 阶段 A：白名单 = 当前类别（${CLS:-?}）→ 期望**放行**"
if start_guard "${CLS:-1}"; then
  A_HITS=$(count_hits /offset_center_guarded 6)
  echo "  闸门运行中采样 6 次，过滤流出现目标 $A_HITS 次"
  echo "-- 闸门日志 --"; tail -3 "/tmp/ag_guard_${CLS:-1}.log" | sed 's/^/  /'
  stop_guard
fi
echo

hr; echo "## 4. 阶段 B：白名单 = 另一个类别（$OTHER）→ 期望**拦截**"
if start_guard "$OTHER"; then
  B_HITS=$(count_hits /offset_center_guarded 6)
  echo "  闸门运行中采样 6 次，过滤流出现目标 $B_HITS 次（期望 0）"
  echo "-- 闸门日志（应有拦截记录）--"; grep -E "拦截" "/tmp/ag_guard_$OTHER.log" | tail -3 | sed 's/^/  /'
  stop_guard
fi
echo

hr; echo "## 5. 判定"
FAIL=0
if [ -z "$CLS" ]; then
  echo "  [SKIP] 画面里没有有效目标，无法判定（换一张对着靶标的画面再跑）"
else
  if [ "${A_HITS:-0}" -gt 0 ]; then
    echo "  [OK] 白名单含该类别 -> 目标被放行（$A_HITS/6 次）"
  else
    echo "  [FAIL] 白名单含该类别，却一次也没放行"; FAIL=1
  fi
  if [ "${B_HITS:-1}" -eq 0 ]; then
    echo "  [OK] 白名单不含该类别 -> 过滤流恒为「无目标」（0/6 次）"
    echo "       => 官方瞄准节点即使运行也锁不到它，不可能误击友方"
  else
    echo "  [FAIL] 白名单不含该类别，却仍被放行 $B_HITS/6 次"; FAIL=1
  fi
fi
echo

hr; echo "## 6. 反证：全程没有动作"
echo "  -- /CameMo_Conl（云台指令；dzrknn 未启动，应无发布者）--"
timeout 10 ros2 topic info /CameMo_Conl 2>/dev/null | sed 's/^/  /' || echo "  话题不存在（符合预期）"
echo "  -- /odom/localization（本轮没起底盘）--"
timeout 10 ros2 topic echo --once /odom/localization --field pose.pose.position 2>/dev/null | tr '\n' ' ' | sed 's/^/  /' || echo "  无发布者（符合预期）"
echo
echo "  结论: $([ "$FAIL" -eq 0 ] && echo '闸门在真机上双向验证通过' || echo '有失败项，见上')"
echo "==================== 结束（节点将自动停止） ===================="
exit "$FAIL"
