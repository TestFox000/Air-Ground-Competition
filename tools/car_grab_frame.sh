#!/usr/bin/env bash
# 从车端相机抓**一帧**回来，让"人（或 AI）能真的看见现场"。
#
# 为什么需要它：车端的定位是编码器+IMU 死推算，车的相机又没有"认网格/读格号"的模型
# —— 所以"场地上现在长什么样"（桥洞在哪、水桶在哪、车朝哪）**谁也答不上来**，
# 只能靠猜。2026-10-01 就因为猜，把路线第一个点选成了桥洞格 K9。
# 抓一帧回来看一眼，比猜十次便宜。
#
# 零动作：只起相机（camera.launch.py）。**不碰云台、不碰底盘、不开火、不发任何 /nav/waypoint。**
#
# 用法：
#   python tools/remote_run.py --host <IP> --user duzhong --script tools/car_grab_frame.sh
#   python tools/remote_run.py --host <IP> --user duzhong --get /tmp/ag_frame --to logs/car/frame
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
# det_node 之类会 cv::imshow，没有 DISPLAY 一帧就崩；这里只抓图，但要防其它节点被带上
[ -z "${DISPLAY:-}" ] && [ -e /tmp/.X11-unix/X0 ] && \
  export DISPLAY=:0 XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1

OUT=/tmp/ag_frame
rm -rf "$OUT"; mkdir -p "$OUT"
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
  echo "  已停止相机（只停了本脚本起的进程）"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

echo "==================== 抓一帧（零动作：只起相机） ===================="
date; echo

hr; echo "## 0. 起相机"
timeout 120 ros2 launch dzactuator camera.launch.py > /tmp/ag_frame_cam.log 2>&1 &
PIDS+=($!)
sleep 12
if grep -q "process has died" /tmp/ag_frame_cam.log 2>/dev/null; then
  echo "  [FAIL] 相机进程死了（usb_cam SIGABRT 是踩过的坑：USB 带宽/供电/被占用）"
  tail -6 /tmp/ag_frame_cam.log | sed 's/^/    /'
  exit 1
fi
tail -3 /tmp/ag_frame_cam.log 2>/dev/null | sed 's/^/    /'
echo

hr; echo "## 1. 订阅 /image_raw 抓第一帧"
# ⚠️ 不用 `ros2 topic echo`：
#   ① image 数据量大，echo 会打印一大坨 base64/数组，毫无意义；
#   ② 这些话题是 BEST_EFFORT，CLI 默认 RELIABLE 订阅 → QoS 不匹配，一条都收不到；
#   ③ 反复 kill 会把 ros2 daemon 搞坏（之后 CLI 直接 !rclpy.ok() 崩）。
#    所以内嵌 rclpy 订阅器，自己解码存图。
cat > /tmp/ag_grab.py <<'PY'
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image

out = sys.argv[1]
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)

got = []
rclpy.init()
node = Node("ag_grab_frame")


def cb(m):
    if not got:
        got.append(m)


node.create_subscription(Image, "/image_raw", cb, qos)
t0 = time.time()
while not got and time.time() - t0 < 40:
    rclpy.spin_once(node, timeout_sec=0.2)

if not got:
    print("NO_FRAME：40 s 内 /image_raw 一帧都没有（相机没起/话题名不对）")
    rclpy.shutdown()
    sys.exit(2)

m = got[0]
enc = (m.encoding or "").lower()
h, w, step = m.height, m.width, m.step
print("encoding=%s  %dx%d  step=%d  bytes=%d" % (m.encoding, w, h, step, len(m.data)))

# 原始字节 + 元信息一并落盘：万一车上没有 cv2，可以取回本地再解码（不用二次上车）
raw = bytes(m.data)
with open(os.path.join(out, "frame.raw"), "wb") as f:
    f.write(raw)
with open(os.path.join(out, "frame.meta.txt"), "w") as f:
    f.write("encoding=%s\nwidth=%d\nheight=%d\nstep=%d\n" % (m.encoding, w, h, step))

try:
    import cv2

    buf = np.frombuffer(raw, dtype=np.uint8)
    if enc in ("yuv422", "yuyv", "yuv422_yuy2", "yuv422_yuyv"):
        img = buf.reshape((h, w, 2))
        bgr = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
    elif enc in ("rgb8", "bgr8"):
        img = buf.reshape((h, step))[:, : w * 3].reshape((h, w, 3))
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img
    elif enc in ("mono8", "8uc1"):
        img = buf.reshape((h, step))[:, :w]
        bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        raise RuntimeError("未知编码 " + repr(m.encoding))
    p = os.path.join(out, "frame.jpg")
    cv2.imwrite(p, bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    print("已存 frame.jpg  %s  %d 字节" % (bgr.shape, os.path.getsize(p)))
except Exception as e:  # noqa: BLE001
    print("cv2 解码失败（原始字已存 frame.raw，可本地解）: %s: %s" % (type(e).__name__, e))

rclpy.shutdown()
PY
timeout 70 python3 -u /tmp/ag_grab.py "$OUT" 2>&1 | sed 's/^/    /'
echo

hr; echo "## 2. 产物"
ls -l "$OUT" 2>/dev/null | sed 's/^/    /'
echo
echo "  取回：python tools/remote_run.py --host <IP> --user duzhong --get $OUT --to logs/car/frame"
echo "==================== 结束 ===================="
