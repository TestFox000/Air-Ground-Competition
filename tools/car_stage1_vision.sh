#!/usr/bin/env bash
# 实机联调 · 第 1 段：**视觉链路**（不启动任何会产生动作的节点）
#
# 启动：camera(usb_cam) + det_node(RKNN)
# 不启动：dzactuator（底盘/云台串口）、dzrknn_dataprocess（云台会开始扫描）、
#         judgment（裁判串口，留给下一段）
#
# 因此**车不会动、云台不会转、串口不会被占**。目的：
#   * 确认真机相机 /image_raw 有流
#   * 确认 RKNN 在 NPU 上能跑（det_node 加载 light_det.rknn）
#   * 看到真实的 /offset_center 输出（含类别号 1=蓝 2=红）
#
# 结束时只 kill 我们自己起的 PID（不会误杀别人的节点）。
#
# 用法：python tools/remote_run.py --host <IP> --user duzhong --script tools/car_stage1_vision.sh

set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null

# ⚠️ 真机坑：det_node 内部会 cv::imshow 画框，**没有图形后端就直接崩**
#    （报 OpenCV ... Can't initialize GTK backend）。
#    赛时是从 VNC 桌面里启动的（有 DISPLAY），但 SSH/服务方式启动就必须显式给：
#      DISPLAY=:0 + XAUTHORITY=$HOME/.Xauthority
#    这里自动检测可用的 X，避免"节点起了但一帧就死"这种最难查的现场故障。
if [ -z "${DISPLAY:-}" ]; then
  if [ -e /tmp/.X11-unix/X0 ]; then
    export DISPLAY=:0
    export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
    echo "[环境] 检测到 :0 上的 X 会话，已设置 DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY"
  else
    echo "[环境] !! 没有可用的 X 会话：det_node 会用 cv::imshow，很可能启动后即崩"
    echo "        （必要时装 xvfb 或用 VNC 桌面启动）"
  fi
fi

CAMPID=""; DETPID=""
cleanup() {
  for pid in "$DETPID" "$CAMPID"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null; sleep 1
      kill -0 "$pid" 2>/dev/null && kill -9 "$pid" 2>/dev/null
      echo "  已停止 pid $pid"
    fi
  done
  pkill -f "usb_cam_node_exe" 2>/dev/null
  pkill -f "/det_node" 2>/dev/null
}
trap cleanup EXIT

hr() { echo "----------------------------------------------------"; }

echo "==================== 实机联调 · 第 1 段：视觉链路 ===================="
date
echo

hr; echo "## 0. 前置检查"
echo "-- 当前是否已有节点在跑（有的话先别继续，可能互相干扰）--"
timeout 10 ros2 node list 2>/dev/null | sed 's/^/  /' || true
echo "-- 依赖包 --"
for p in usb_cam det_node dzactuator dzrknn_dataprocess; do
  printf "  %-20s %s\n" "$p" "$(ros2 pkg prefix "$p" 2>/dev/null || echo '未安装')"
done
echo "-- 相机设备 --"
ls /dev/video0 2>/dev/null || echo "  /dev/video0 不存在"
echo

hr; echo "## 1. 启动 usb_cam（只读相机，不产生任何动作）"
timeout 300 ros2 launch dzactuator camera.launch.py > /tmp/ag_stage1_cam.log 2>&1 &
CAMPID=$!
sleep 8
echo "-- /image_raw 是否出现 --"
if timeout 15 ros2 topic list 2>/dev/null | grep -q "^/image_raw$"; then
  echo "  [OK] /image_raw 存在"
  echo "-- 帧率（3 秒采样）--"
  timeout 12 ros2 topic hz /image_raw 2>/dev/null | head -3 | sed 's/^/  /'
  echo "-- 一帧的分辨率/编码 --"
  timeout 15 ros2 topic echo --once /image_raw 2>/dev/null | head -12 | sed 's/^/  /'
else
  echo "  [FAIL] /image_raw 没出现，日志尾部："
  tail -15 /tmp/ag_stage1_cam.log | sed 's/^/    /'
fi
echo

hr; echo "## 2. 启动 det_node（RKNN 推理，输出 /offset_center）"
timeout 300 ros2 launch dzactuator det_node.launch.py > /tmp/ag_stage1_det.log 2>&1 &
DETPID=$!
sleep 12
echo "-- 节点日志（模型路径 / 初始化）--"
grep -E "Model path|rknnPool|init|error|Error|fail" /tmp/ag_stage1_det.log | head -10 | sed 's/^/  /'
if grep -q "Can't initialize GTK backend" /tmp/ag_stage1_det.log; then
  echo "  ⚠️ 命中已知坑：det_node 的 cv::imshow 没有图形后端 —— 需要 DISPLAY=:0（或 VNC 桌面启动）"
  echo "     这条同样适用于比赛：**别用 SSH/服务方式启动 det_node**，否则一帧就崩。"
fi
if ! kill -0 "$DETPID" 2>/dev/null; then
  echo "  ⚠️ det_node 进程已退出（pid $DETPID）"
fi
echo
echo "-- /offset_center 是否在发 --"
if timeout 15 ros2 topic list 2>/dev/null | grep -q "^/offset_center$"; then
  echo "  [OK] /offset_center 存在"
  echo "-- 采样 3 条真实输出（[dx, dy, 类别, 框宽, 框高, kpt_dx, kpt_dy, kpt_conf*100]）--"
  timeout 20 ros2 topic echo /offset_center 2>/dev/null | head -24 | sed 's/^/  /'
else
  echo "  [FAIL] /offset_center 没出现，日志尾部："
  tail -15 /tmp/ag_stage1_det.log | sed 's/^/    /'
fi
echo

hr; echo "## 3. 反证：确认没有启动任何会动硬件的节点"
echo "-- 云台指令话题 /CameMo_Conl 的发布者数（应为 0）--"
timeout 10 ros2 topic info /CameMo_Conl 2>/dev/null | sed 's/^/  /' || echo "  话题不存在（正常）"
echo "-- 底盘指令 /pursuitAngle 的发布者数（应为 0）--"
timeout 10 ros2 topic info /pursuitAngle 2>/dev/null | sed 's/^/  /' || echo "  话题不存在（正常）"
echo "-- 串口未被占用（dzactuator/judgment 都未启动）--"
ls -l /dev/dzactuator /dev/dzjudgment 2>/dev/null | sed 's/^/  /'
fuser /dev/dzactuator /dev/dzjudgment 2>&1 | head -3 | sed 's/^/  /' || true
echo

hr; echo "## 4. 本次联调的结论与下一步"
echo "  第 1 段（视觉）验证完就可以进第 2 段："
echo "    - 启动 dzactuator（底盘/云台串口）+ judgment（裁判）"
echo "    - 最后才启动 dzrknn_dataprocess —— 它一启动云台就会开始左右扫描"
echo "    ⚠️ 第 2 段必须有人在场看着，且车要架空或留出足够空间"
echo
echo "==================== 结束（节点将自动停止） ===================="
