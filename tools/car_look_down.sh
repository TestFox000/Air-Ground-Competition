#!/usr/bin/env bash
# 让车**低头看地面**并抓帧：用来确认"正前方那几格是什么格号、地面有没有网格线"。
#
# 为什么需要：车是贴地前视相机，平视时只看得到 1 m 外的一小片斜视地面；
# 而"车现在到底在第几格、朝哪个方向走不会撞桥洞/水桶"这件事，
# 只有低头看到格号才能定。2026-10-01 就是因为看不见，把路线第一个点选成了桥洞格 K9。
#
# 安全性：只动**云台**，不动底盘、不开火。
#   官方接口文档明确：`CameMo_Conl` 只改云台，**不解锁驱动**。
#
# 用法：
#   python tools/remote_run.py --host <IP> --user duzhong --script tools/car_look_down.sh
#   python tools/remote_run.py --host <IP> --user duzhong --get /tmp/ag_look --to logs/car/look
#
# 环境变量：
#   AG_PITCH_A=1750  第一个俯仰刻度（零位 2047；两个方向各拍一张，省得猜正负）
#   AG_PITCH_B=2350  第二个俯仰刻度
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
[ -z "${DISPLAY:-}" ] && [ -e /tmp/.X11-unix/X0 ] && \
  export DISPLAY=:0 XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1

PA="${AG_PITCH_A:-1750}"   # 约 -26°（相对零位 2047）
PB="${AG_PITCH_B:-2350}"   # 约 +26°
YA="${AG_YAW_A:-2047}"     # 水平：2047 = 正前；±90° = ±1024 刻度（11.378 刻度/度）
YB="${AG_YAW_B:-2047}"
ZERO=2047
OUT=/tmp/ag_look
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
  echo "  已停止（只停了本脚本起的进程）"
}
trap cleanup EXIT
hr() { echo "----------------------------------------------------"; }

echo "==================== 低头看地面（只动云台，不动底盘） ===================="
date; echo

hr; echo "## 0. 起底盘串口（云台走它）+ 相机"
# 云台由 dzactuator 经串口驱动，必须先起它；起它本身不会让车动（没有速度指令）。
timeout 150 ros2 launch dzactuator bringup.launch.py > /tmp/ag_look_act.log 2>&1 &
PIDS+=($!)
sleep 12
grep -E "serial port opened|can not open" /tmp/ag_look_act.log 2>/dev/null | tail -2 | sed 's/^/    /'
timeout 150 ros2 launch dzactuator camera.launch.py > /tmp/ag_look_cam.log 2>&1 &
PIDS+=($!)
sleep 10
echo

# 云台指令发布器（内嵌 rclpy，不用 ros2 CLI：CLI 依赖 daemon，反复 kill 会把它搞坏）
cat > /tmp/ag_gimbal.py <<'PY'
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

pitch = int(sys.argv[1])
yaw = int(sys.argv[2]) if len(sys.argv) > 2 else 2047
rclpy.init()
n = Node("ag_gimbal_look")
pub = n.create_publisher(Int32MultiArray, "/CameMo_Conl", 10)
time.sleep(0.8)
m = Int32MultiArray()
# [俯仰刻度, 水平刻度, 速度0, 速度1]；零位 2047，11.378 刻度/度
m.data = [pitch, yaw, 300, 300]
for _ in range(8):
    pub.publish(m)
    time.sleep(0.1)
n.destroy_node()
rclpy.shutdown()
print("  /CameMo_Conl <- pitch=%d yaw=%d" % (pitch, yaw))
PY

cat > /tmp/ag_grab2.py <<'PY'
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image

out, tag = sys.argv[1], sys.argv[2]
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
got = []
rclpy.init()
n = Node("ag_grab2")
n.create_subscription(Image, "/image_raw", lambda m: got.append(m) if not got else None, qos)
t0 = time.time()
while not got and time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.2)
if not got:
    print("  [FAIL] %s: 30 s 没抓到帧" % tag)
    rclpy.shutdown()
    sys.exit(2)
m = got[0]
raw = bytes(m.data)
with open(os.path.join(out, "frame_%s.raw" % tag), "wb") as f:
    f.write(raw)
try:
    import cv2
    buf = np.frombuffer(raw, dtype=np.uint8)
    enc = (m.encoding or "").lower()
    if enc in ("yuv422", "yuyv", "yuv422_yuy2", "yuv422_yuyv"):
        bgr = cv2.cvtColor(buf.reshape((m.height, m.width, 2)), cv2.COLOR_YUV2BGR_YUYV)
    elif enc in ("rgb8", "bgr8"):
        img = buf.reshape((m.height, m.step))[:, : m.width * 3].reshape((m.height, m.width, 3))
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img
    else:
        raise RuntimeError("编码 " + repr(m.encoding))
    p = os.path.join(out, "frame_%s.jpg" % tag)
    cv2.imwrite(p, bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    print("  [OK] %s -> %s" % (tag, os.path.basename(p)))
except Exception as e:  # noqa: BLE001
    print("  [注意] %s: cv2 解码失败(%s)，已存 raw" % (tag, e))
rclpy.shutdown()
PY

hr; echo "## 1. 按给定姿态拍照（a/b 各一张）"
for pose in "a:$PA:$YA" "b:$PB:$YB"; do
  tag="$(echo "$pose" | cut -d: -f1)"
  pitch="$(echo "$pose" | cut -d: -f2)"
  yaw="$(echo "$pose" | cut -d: -f3)"
  echo "  -- 姿态 $tag: 俯仰 $pitch / 水平 $yaw（零位 $ZERO）--"
  timeout 40 python3 -u /tmp/ag_gimbal.py "$pitch" "$yaw" 2>&1 | sed 's/^/  /'
  sleep 2
  timeout 60 python3 -u /tmp/ag_grab2.py "$OUT" "$tag" 2>&1 | sed 's/^/  /'
done

hr; echo "## 2. 云台复位到零位 $ZERO"
timeout 40 python3 -u /tmp/ag_gimbal.py "$ZERO" 2>&1 | sed 's/^/  /'
echo
ls -l "$OUT" 2>/dev/null | sed 's/^/    /'
echo
echo "  取回：python tools/remote_run.py --host <IP> --user duzhong --get $OUT --to logs/car/look"
echo "==================== 结束 ===================="
