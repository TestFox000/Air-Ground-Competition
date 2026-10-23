#!/bin/bash
# ============================================================================
# cam_lib.sh —— cam_env.sh / cam_up.sh / cam_shot.sh 共用的探测逻辑
#
# 只依赖 GStreamer。**不依赖 v4l2-ctl**（实测机上 v4l-utils 未安装）。
#
# 设计要点：
#   1) 格式阶梯（ladder）逐个试，每个只抓 1 帧，谁通用谁。这样不需要事先知道
#      相机的能力，也不需要解析 gst-device-monitor 的文本输出。
#   2) UVC 相机常见双节点（video0=取流, video1=metadata），metadata 节点要跳过，
#      否则永远取不到流。
#   3) 同一个 UVC 节点同一时刻只能被一个进程取流，所以"预览"和"ROS 节点"必须二选一。
# ============================================================================

# 阶梯：kind|caps|W|H|FPS      （kind 决定后面怎么解码）
cam_cands() {
  cat <<'EOF'
mjpg|image/jpeg,width=1920,height=1200,framerate=30/1|1920|1200|30
mjpg|image/jpeg,width=1920,height=1080,framerate=30/1|1920|1080|30
mjpg|image/jpeg,width=1280,height=960,framerate=30/1|1280|960|30
mjpg|image/jpeg,width=1280,height=720,framerate=30/1|1280|720|30
mjpg|image/jpeg,width=1280,height=720,framerate=60/1|1280|720|60
mjpg|image/jpeg,width=640,height=480,framerate=30/1|640|480|30
yuyv|video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1|1280|720|30
yuyv|video/x-raw,format=YUY2,width=640,height=480,framerate=30/1|640|480|30
auto||0|0|0
EOF
}

# 取流节点名（用于跳过 metadata 节点）
cam_node_name() {
  local dev="$1" base
  base="$(basename "$dev")"
  cat "/sys/class/video4linux/$base/name" 2>/dev/null
}

# 判断是否 metadata 节点
cam_is_metadata() {
  case "$(cam_node_name "$1")" in
    *etadata*) return 0 ;;
  esac
  return 1
}

# 探测某设备的可用格式；成功则回显 "kind|caps|W|H|FPS"
cam_probe_format() {
  local dev="$1" kind caps w h fps pipe
  while IFS='|' read -r kind caps w h fps; do
    [ -z "$kind" ] && continue
    if [ -z "$caps" ]; then
      pipe=(v4l2src "device=$dev" num-buffers=1 ! decodebin ! fakesink sync=false)
    else
      pipe=(v4l2src "device=$dev" num-buffers=1 ! "$caps" ! fakesink sync=false)
    fi
    if timeout 25 gst-launch-1.0 -q "${pipe[@]}" >/dev/null 2>&1; then
      echo "$kind|$caps|$w|$h|$fps"
      return 0
    fi
  done < <(cam_cands)
  return 1
}

# 自动选出第一个能真正取到流的节点
cam_first_working_dev() {
  local d caps
  for d in /dev/video*; do
    [ -e "$d" ] || continue
    if cam_is_metadata "$d"; then
      echo "  skip $d ($(cam_node_name "$d"))" >&2
      continue
    fi
    if caps="$(cam_probe_format "$d")"; then
      echo "$d"
      return 0
    fi
  done
  return 1
}

# 抓 N 帧、**保留最后一帧**写成 JPEG。参数：dev out kind caps [count]
#
# 为什么要连抓而不是 num-buffers=1：
#   UVC 开流瞬间自动曝光还没收敛，**单帧往往偏暗甚至全黑**。
#   实测同一台相机：单帧抓取 avg≈38；连续取流收敛后 mean≈140。
#   所以先抓一串，丢掉前面的，只留最后一帧。
# 返回 0 = 文件存在且是合法 JPEG（FFD8FF）
cam_grab_jpeg() {
  local dev="$1" out="$2" kind="$3" caps="$4" count="${5:-8}" chain="" tmpdir last
  case "$kind" in
    mjpg) chain="" ;;                                        # MJPG 帧本身就是 JPEG，直接落盘
    yuyv) chain="videoconvert ! jpegenc" ;;
    auto) chain="decodebin ! videoconvert ! jpegenc" ;;
  esac
  rm -f "$out"
  tmpdir="$(mktemp -d)" || return 1
  if [ -z "$caps" ]; then
    timeout 45 gst-launch-1.0 -q v4l2src "device=$dev" num-buffers="$count" ! $chain \
      ! multifilesink "location=$tmpdir/f%02d.jpg" >/dev/null 2>&1
  else
    timeout 45 gst-launch-1.0 -q v4l2src "device=$dev" num-buffers="$count" ! "$caps" ! $chain \
      ! multifilesink "location=$tmpdir/f%02d.jpg" >/dev/null 2>&1
  fi
  last="$(ls -1 "$tmpdir"/f*.jpg 2>/dev/null | sort | tail -1)"
  [ -n "$last" ] && mv "$last" "$out"
  rm -rf "$tmpdir"
  [ -s "$out" ] || return 1
  head -c 3 "$out" | od -An -tx1 | grep -qi 'ff d8 ff'
}

# 抓 N 帧、保留最后一帧缩成 8x8 GRAY8（同样是为了等自动曝光收敛）
# 参数：dev kind caps [count] [out]
cam_grab_gray() {
  local dev="$1" kind="$2" caps="$3" count="${4:-8}" out="${5:-/tmp/cam_thumb.gray}"
  local chain tmpdir last
  chain="$(cam_gray_chain "$kind")"
  rm -f "$out"
  tmpdir="$(mktemp -d)" || return 1
  if [ -z "$caps" ]; then
    timeout 45 gst-launch-1.0 -q v4l2src "device=$dev" num-buffers="$count" ! $chain \
      ! video/x-raw,format=GRAY8,width=8,height=8 \
      ! multifilesink "location=$tmpdir/g%02d.gray" >/dev/null 2>&1
  else
    timeout 45 gst-launch-1.0 -q v4l2src "device=$dev" num-buffers="$count" ! "$caps" ! $chain \
      ! video/x-raw,format=GRAY8,width=8,height=8 \
      ! multifilesink "location=$tmpdir/g%02d.gray" >/dev/null 2>&1
  fi
  last="$(ls -1 "$tmpdir"/g*.gray 2>/dev/null | sort | tail -1)"
  [ -n "$last" ] && mv "$last" "$out"
  rm -rf "$tmpdir"
  [ -s "$out" ]
}

# 给"预览/ROS/缩略图"用的解码段
cam_decode_chain() {   # $1=kind
  case "$1" in
    mjpg) echo "jpegdec" ;;
    yuyv) echo "" ;;
    auto) echo "decodebin" ;;
    *)    echo "" ;;
  esac
}

cam_gray_chain() {     # $1=kind   （接在 caps 之后，用于 8x8 灰度缩略图）
  case "$1" in
    mjpg) echo "jpegdec ! videoconvert ! videoscale" ;;
    yuyv) echo "videoconvert ! videoscale" ;;
    auto) echo "decodebin ! videoconvert ! videoscale" ;;
    *)    echo "videoconvert ! videoscale" ;;
  esac
}

cam_have() { command -v "$1" >/dev/null 2>&1; }

# ---------------------------------------------------------------------------
# YUYV 真实档位表（从 gst-device-monitor 实测枚举，非推测）
# 为什么单独列：v4l2_camera 只吃原始格式，而 YUYV 的帧率远低于 MJPG，
# 传一个不存在的组合会让节点起不来。
# ---------------------------------------------------------------------------
cam_yuyv_table() {
  cat <<'EOF'
1920x1200 5
1920x1080 5
896x1200 5
1600x1200 5
1280x960 5
1024x768 10
1280x720 10
960x720 15
800x600 20
640x480 30
320x240 120
EOF
}

# 查某分辨率在 YUYV 下的最高帧率；不支持则输出空
cam_yuyv_maxrate() {
  local want="$1x$2"
  cam_yuyv_table | awk -v w="$want" '$1==w {print $2; found=1} END{if(!found) exit 0}'
}
