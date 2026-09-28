#!/usr/bin/env bash
# 上车前 / 联调前先单独验一下**相机**。
#
# 为什么要有这个脚本：2026-09-27 的一次联调里 `usb_cam_node` 以 SIGABRT 崩掉，
# 而整个链路是"静默"的 —— det_node 只会无限刷 "waiting for /image_raw"，
# 我们这边也安静得不发一帧检测。现场表现永远只是"什么都检测不到"，
# 真正的原因却在链路上游三跳之外，只能靠翻节点日志才发现。
# 所以：**任何要用到视觉的联调，先把这一项单独验掉**。
#
# 用法（零动作：只起相机与检测，不碰云台/裁判/底盘/开火）：
#   python tools/remote_run.py --host <IP> --user duzhong --script tools/car_camera_check.sh
#
# 退出码：0 = 相机健康；1 = 没有图像或没有检测。

set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "${AG_WS:-/tmp/ag_ws}/install/setup.bash" 2>/dev/null
# det_node 内部 cv::imshow，没有 DISPLAY 会一帧就崩
[ -z "${DISPLAY:-}" ] && [ -e /tmp/.X11-unix/X0 ] && \
  export DISPLAY=:0 XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1

PIDS=()
cleanup() {
  local pid child
  for pid in "${PIDS[@]}"; do
    for child in $(pgrep -P "$pid" 2>/dev/null); do kill -INT "$child" 2>/dev/null; done
    kill -INT "$pid" 2>/dev/null
  done
  sleep 2
  for pid in "${PIDS[@]}"; do
    for child in $(pgrep -P "$pid" 2>/dev/null); do kill -9 "$child" 2>/dev/null; done
    kill -9 "$pid" 2>/dev/null
  done
  echo "  已停止本脚本启动的节点（含子进程）"
}
trap cleanup EXIT

hr() { echo "----------------------------------------------------"; }
echo "==================== 相机健康检查（零动作） ===================="
date
echo

hr; echo "## 0. 现状：有没有别人在跑"
timeout 12 ros2 node list 2>/dev/null | sed 's/^/  /'
echo

hr; echo "## 1. 起相机 + 检测"
timeout 180 ros2 launch dzactuator camera.launch.py > /tmp/ag_chk_cam.log 2>&1 &
PIDS+=($!); sleep 8
timeout 180 ros2 launch dzactuator det_node.launch.py > /tmp/ag_chk_det.log 2>&1 &
PIDS+=($!); sleep 12

echo "-- 相机节点日志尾部 --"
tail -4 /tmp/ag_chk_cam.log 2>/dev/null | sed 's/^/  /'
if grep -q "process has died" /tmp/ag_chk_cam.log 2>/dev/null; then
  echo "  [FAIL] usb_cam 进程死掉了（真机踩过：exit code -6 / SIGABRT）"
  echo "         常见原因：设备被别的进程占着、USB 带宽或供电不足。先看 dmesg 与 /dev/video*"
fi
if grep -q "Can't initialize GTK backend" /tmp/ag_chk_det.log 2>/dev/null; then
  echo "  [FAIL] det_node 因 GTK 崩了（DISPLAY 未生效）—— 走 VNC 或先 export DISPLAY=:0"
fi
echo

hr; echo "## 2. /image_raw 帧率（数 5 秒）"
HZ=$(timeout 20 ros2 topic hz /image_raw 2>/dev/null | grep -m1 "average rate" | sed 's/^/  /')
if [ -z "$HZ" ]; then
  echo "  [FAIL] 5 秒内 /image_raw 没有任何一帧"
  echo "         -> 相机没起 / 崩了 / 话题名不对。先确认 `ros2 node list` 里有 usb_cam_node"
  RATE=0
else
  echo "$HZ"
  RATE=$(echo "$HZ" | grep -oE "[0-9]+\.[0-9]+" | head -1)
fi
echo

hr; echo "## 3. 是否能出检测结果（/offset_center）"
RAW=$(timeout 20 ros2 topic echo --once /offset_center --field data 2>/dev/null \
      | sed "s/array('i', *\[*//; s/array('d', *\[*//; s/[])]//g" \
      | tr -d ',' | tr '\n' ' ' | tr -s ' ' | sed 's/^ *//; s/ *$//')
echo "  /offset_center: ${RAW:-（无数据）}"
CLS=$(echo "$RAW" | awk '{print $3}')
echo "  类别 cls = ${CLS:-?}   （1=蓝靶=敌方  2=红靶=友方）"
echo

hr; echo "## 4. 结论"
OK=1
if [ "${RATE:-0}" = "0" ] || [ -z "$RATE" ]; then
  echo "  [FAIL] 相机没有稳定出图 —— 先修这个，视觉相关的一切都无从谈起"
  OK=0
else
  echo "  [OK] /image_raw 有帧（约 ${RATE} Hz）"
fi
if [ -z "$CLS" ] || [ "$CLS" = "0" ]; then
  echo "  [注意] 画面里当前没有有效目标（不影响相机本身，但闸门/瞄准类联调需要靶标）"
else
  echo "  [OK] 读到检测类别 $CLS"
fi
echo
echo "  结论: $([ "$OK" -eq 1 ] && echo '相机健康，可以继续视觉类联调' || echo '相机不健康，先别跑视觉联调')"
echo "==================== 结束（节点将自动停止） ===================="
exit "$([ "$OK" -eq 1 ] && echo 0 || echo 1)"
