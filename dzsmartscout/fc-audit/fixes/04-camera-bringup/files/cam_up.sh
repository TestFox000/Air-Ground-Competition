#!/bin/bash
# ============================================================================
# cam_up.sh —— 点亮相机（两种模式，**二选一**）
#
#   默认        GStreamer MJPG 预览窗口 —— 高分辨率，给人看（VNC 桌面上开窗）
#   --ros       v4l2_camera_node      —— 给 ROS 用，发布 /image_raw
#
# 为什么必须二选一：同一个 UVC 节点同一时刻只能被一个进程取流。
#
# ⚠ 关键限制（实测踩到的坑，见 APPLY.md 第三节）：
#     v4l2_camera 0.6.2 **不支持 MJPG**。传 pixel_format:=MJPG 会让节点直接
#     abort（cv_bridge: Unrecognized image encoding []），所以 --ros 模式
#     强制走 YUYV，并且只接受下表里真实存在的 YUYV 组合。
#
# 用法：
#   bash cam_up.sh                          # 自动选设备，MJPG 1920x1200 预览
#   bash cam_up.sh /dev/video0 1280x720     # 指定设备与分辨率预览
#   bash cam_up.sh --ros                    # ROS 节点，YUYV 640x480@30
#   bash cam_up.sh --ros 1280x720 10        # ROS 节点，YUYV 1280x720@10
# ============================================================================
set +u
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=cam_lib.sh
. "$DIR/cam_lib.sh"

DEV=""
GEO=""
FPS=""
MODE="preview"
for a in "$@"; do
  case "$a" in
    --ros)     MODE="ros" ;;
    --preview) MODE="preview" ;;
    /dev/video*) DEV="$a" ;;
    [0-9]*x[0-9]*) GEO="$a" ;;
    [0-9]*)    FPS="$a" ;;
  esac
done

if ! cam_have gst-launch-1.0; then
  echo "ERROR: 没有 gst-launch-1.0，本脚本无法工作"
  exit 6
fi

# ---- 选设备 -----------------------------------------------------------------
if [ -z "$DEV" ]; then
  echo "--- 自动选设备（跳过 metadata 节点）---"
  DEV="$(cam_first_working_dev)" || {
    echo "ERROR: 没有能取到流的 /dev/video*。先跑 cam_env.sh 看节点情况。"
    exit 2
  }
  echo "选中: $DEV  ($(cam_node_name "$DEV"))"
fi

IFS='|' read -r KIND CAPS W H FPS_DET <<EOF
$(cam_probe_format "$DEV")
EOF
[ -n "$KIND" ] || { echo "ERROR: $DEV 取不到流"; exit 5; }

# ============================================================================
# 模式一：GStreamer 预览（MJPG 优先，画面最好看）
# ============================================================================
if [ "$MODE" = "preview" ]; then
  RES="${GEO:-${W}x${H}}"
  PW="${RES%x*}"
  PH="${RES#*x}"
  RATE="${FPS:-$FPS_DET}"
  [ "$RATE" = "0" ] && RATE=30

  if [ -z "${DISPLAY:-}" ]; then
    if DISPLAY=:1 xdpyinfo >/dev/null 2>&1; then
      DISPLAY=:1
    else
      echo "ERROR: 没有可用 X 显示（DISPLAY 空且 :1 不可达）。无头机器请改用:"
      echo "       bash cam_shot.sh        # 抓帧存图，不需要显示器"
      exit 7
    fi
  fi
  export DISPLAY
  echo "显示: DISPLAY=$DISPLAY"
  echo "取流: $DEV  MJPG ${PW}x${PH}@${RATE}"

  LOG=/tmp/cam_preview.log
  setsid nohup gst-launch-1.0 v4l2src "device=$DEV" \
    ! "image/jpeg,width=$PW,height=$PH,framerate=$RATE/1" \
    ! jpegdec ! videoconvert ! xvimagesink sync=false \
    > "$LOG" 2>&1 < /dev/null &

  sleep 4
  if pgrep -f "v4l2src device=$DEV" >/dev/null 2>&1; then
    echo
    echo "已启动。窗口应该出现在 VNC 桌面上（默认尺寸就是分辨率大小）。"
    echo "停止:  pkill -f 'v4l2src device=$DEV'"
    echo "日志:  $LOG"
  else
    echo
    echo "启动失败，日志尾部："
    tail -12 "$LOG"
    echo "提示: 若报 'Device or resource busy'，说明设备已被占用（ROS 节点在跑？）"
    echo "      pgrep -af v4l2_camera_node"
  fi
  exit 0
fi

# ============================================================================
# 模式二：ROS 节点（v4l2_camera，**只能 YUYV**）
# ============================================================================
RES="${GEO:-640x480}"
RW="${RES%x*}"
RH="${RES#*x}"
RATE="${FPS:-30}"

if ! cam_have ros2; then
  # shellcheck disable=SC1091
  [ -f /opt/ros/humble/setup.bash ] && . /opt/ros/humble/setup.bash
fi
if ! cam_have ros2; then
  echo "ERROR: 找不到 ros2，--ros 模式不可用（预览模式不受影响）"
  exit 8
fi

# YUYV 档位校验：这些是从 gst-device-monitor 实测枚举出来的真实组合
MAXRATE="$(cam_yuyv_maxrate "$RW" "$RH")"
if [ -z "$MAXRATE" ]; then
  echo "ERROR: YUYV 不支持 ${RW}x${RH}。可用组合（YUYV 是原始格式，帧率远低于 MJPG）："
  cam_yuyv_table | sed 's/^/       /'
  echo "      常用推荐: 640x480@30（控制回路）或 1280x720@10（看得清）"
  exit 9
fi
if [ "$RATE" -gt "$MAXRATE" ]; then
  echo "警告: YUYV ${RW}x${RH} 最高 ${MAXRATE} fps，已从 ${RATE} 降为 ${MAXRATE}"
  RATE="$MAXRATE"
fi

pgrep -f v4l2_camera_node >/dev/null 2>&1 && {
  echo "注意: 已有 v4l2_camera_node 在跑，先停掉再启（同一设备只能一个取流者）:"
  echo "      pkill -f v4l2_camera_node"
  exit 10
}

LOG=/tmp/cam_ros.log
echo "启动: YUYV ${RW}x${RH}@${RATE}  (pixel_format 只能是 YUYV —— MJPG 会让节点 abort)"
setsid nohup ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="$DEV" \
  -p pixel_format:=YUYV \
  -p image_size:="[$RW,$RH]" \
  -p time_per_frame:="[1,$RATE]" \
  -p output_encoding:=rgb8 \
  -p camera_frame_id:=camera_optical_frame \
  > "$LOG" 2>&1 < /dev/null &

sleep 7
if pgrep -f v4l2_camera_node >/dev/null 2>&1; then
  echo
  echo "已启动。验证："
  echo "  ros2 topic list | grep -i image"
  echo "  ros2 topic hz /image_raw"
  echo "  DISPLAY=:1 python3 /tmp/cam_ros_view.py /image_raw 1280 800   # VNC 里看画面"
  echo "  python3 /tmp/cam_ros_snap.py                                   # 无头抓帧存 PNG"
  echo "停止:  pkill -f v4l2_camera_node"
else
  echo
  echo "启动失败，日志尾部（MJPG 相关报错见 APPLY.md 第三节）："
  tail -15 "$LOG"
fi
