#!/usr/bin/env bash
# 开火闸门自检（**零位移、零发射，且不需要相机 / 官方视觉栈**）
#
# 为什么要有这个：
#   `car_stage2b_guard_proof.sh` 依赖"镜头前有个真靶标"，而官方视觉栈
#   （dzactuator + det_node）在公用车上并不总是装好的 —— 2026-09-28 就遇到过
#   dzactuator 没安装、`det_node` 里连可执行文件都没有，相机根本起不来。
#   但闸门本身跟相机无关：它只消费 `/offset_center`、产出 `/offset_center_guarded`。
#   所以这里**直接合成一帧**喂进去，就能在真机 ROS 环境里做双向对照。
#
# 做法（双向对照，缺一不可）：
#   阶段 A：白名单 = 帧里的类别 -> 期望**原样放行**
#   阶段 B：白名单 = 另一个类别 -> 期望**过滤成全 0**
#   两个方向都对，才说明闸门在真的判断，而不是掐死链路或全放行。
#
# 用法：python tools/remote_run.py --host <IP> --user duzhong --script tools/car_guard_selftest.sh

# ⚠️ 不要在这里用 `set -u`：ROS 的 setup.bash 会引用未定义变量，
#    非交互 shell 下会直接退出（实测：整份脚本一行输出都没有，只返回 1）。
set -o pipefail
WS="${AG_WS:-/tmp/ag_ws}"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null
export RCUTILS_LOGGING_BUFFERED_STREAM=1

GUARD=""
cleanup() {
  [ -n "$GUARD" ] && kill -INT "$GUARD" 2>/dev/null
  sleep 2
  [ -n "$GUARD" ] && kill -9 "$GUARD" 2>/dev/null
  echo "  已停止闸门"
}
trap cleanup EXIT

hr() { echo "----------------------------------------------------"; }

# 帧格式 [dx, dy, cls, w, h, kx, ky, kconf*100]，第 3 个是类别（1=蓝=敌 2=红=友）
FRAME="[13, 55, 1, 80, 76, 12, 202, 63]"

# ⚠️ 顺序很重要：**先挂监听，再发帧**。
# 反过来（先发后听）时消息早就发完了，`echo --once` 会一直等下一条、超时返回空，
# 看起来就像"闸门没转发"—— 实测踩过，白白查了半天。
listen_guarded() {   # $1 = 输出文件
  rm -f "$1"
  ( timeout 12 ros2 topic echo --once /offset_center_guarded --field data \
      > "$1" 2>/dev/null ) &
  sleep 1
}

read_guarded() {     # $1 = 输出文件
  # `ros2 topic echo` 会在每帧后打一行 `---` 分隔符，不去掉的话
  # "全 0 = 被拦"的判据永远匹配不上（实测：明明拦住了却判成放行）。
  sed "s/array('i', *\[*//; s/array('d', *\[*//; s/[])]//g" "$1" 2>/dev/null \
    | grep -v -- '^---$' \
    | tr -d ',' | tr '\n' ' ' | tr -s ' ' | sed 's/^ *//; s/ *$//'
}

start_guard() {   # $1 = allow_classes
  cat > /tmp/ag_guard_selftest.yaml <<EOF
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
  timeout 120 ros2 run ag_car aim_guard_node --ros-args \
    --params-file /tmp/ag_guard_selftest.yaml > "/tmp/ag_guard_self_$1.log" 2>&1 &
  GUARD=$!
  sleep 6
  kill -0 "$GUARD" 2>/dev/null && { echo "  [OK] 闸门已启动（allow_classes=\"$1\"）"; return 0; }
  echo "  [FAIL] 闸门没起来："; head -8 "/tmp/ag_guard_self_$1.log" | sed 's/^/    /'
  return 1
}

stop_guard() { [ -n "$GUARD" ] && kill -INT "$GUARD" 2>/dev/null; sleep 2; GUARD=""; }

echo "==================== 开火闸门自检（合成帧，零位移零发射） ===================="
date
echo

hr; echo "## 0. 现有节点"
timeout 10 ros2 node list 2>/dev/null | sed 's/^/  /'
echo

FAIL=0

hr; echo "## 1. 阶段 A：白名单=1（帧里 cls=1）→ 期望放行"
if start_guard "1"; then
  listen_guarded /tmp/ag_guard_A.txt
  timeout 20 ros2 topic pub -r 5 --times 8 /offset_center std_msgs/msg/Int32MultiArray \
    "{data: $FRAME}" > /dev/null 2>&1
  sleep 2
  A=$(read_guarded /tmp/ag_guard_A.txt)
  echo "  过滤后: ${A:-（无输出）}"
  if [ -z "$A" ]; then
    echo "  [FAIL] 过滤流没有输出（闸门没转发？）"; FAIL=1
  elif echo "$A" | grep -qE "^0( 0)*$"; then
    echo "  [FAIL] 白名单含该类别却被拦截"; FAIL=1
  else
    echo "  [OK] 放行（类别被判为蓝面=敌方）"
  fi
  echo "  -- 闸门日志 --"; tail -2 /tmp/ag_guard_self_1.log 2>/dev/null | sed 's/^/    /'
  stop_guard
fi
echo

hr; echo "## 2. 阶段 B：白名单=2（帧里仍是 cls=1）→ 期望拦截成全 0"
if start_guard "2"; then
  listen_guarded /tmp/ag_guard_B.txt
  timeout 20 ros2 topic pub -r 5 --times 8 /offset_center std_msgs/msg/Int32MultiArray \
    "{data: $FRAME}" > /dev/null 2>&1
  sleep 2
  B=$(read_guarded /tmp/ag_guard_B.txt)
  echo "  过滤后: ${B:-（无输出）}"
  if echo "$B" | grep -qE "^0( 0)*$"; then
    echo "  [OK] 被拦成「无目标」—— 官方瞄准节点锁不到它，不可能误击"
  else
    echo "  [FAIL] 白名单不含该类别却仍放行：${B:-（空）}"; FAIL=1
  fi
  echo "  -- 闸门日志（应看到拦截记录）--"
  grep -E "拦截" /tmp/ag_guard_self_2.log 2>/dev/null | tail -2 | sed 's/^/    /'
  stop_guard
fi
echo

hr; echo "## 3. 判定"
echo "  结论: $([ "$FAIL" -eq 0 ] && echo '闸门在真机上双向验证通过' || echo '有失败项，见上')"
echo "==================== 结束（节点将自动停止） ===================="
exit "$FAIL"
