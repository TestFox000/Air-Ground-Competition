#!/bin/bash
# ============================================================================
# cam_env.sh —— 相机「能不能用」的只读体检（第一步）
#
# 只读承诺：不创建/修改/删除文件，不装软件包，不改配置，不起常驻进程。
#          只有 gst-device-monitor-1.0 这个枚举工具会被调用（瞬时退出）。
#
# 用法：  bash cam_env.sh
#
# 判据：  本脚本回答「有没有设备节点、是什么驱动、什么权限、工具链够不够」。
#          能不能出图由 cam_shot.sh 回答。
# ============================================================================
set +u
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=cam_lib.sh
. "$DIR/cam_lib.sh"

echo "=== 1. 主机 ==="
hostname
cat /proc/device-tree/model 2>/dev/null && echo
uname -srm
if [ -f /etc/os-release ]; then . /etc/os-release; echo "OS: $PRETTY_NAME"; fi

echo
echo "=== 2. 视频设备节点 ==="
if ls /dev/video* >/dev/null 2>&1; then
  ls -l /dev/video*
else
  echo "NONE  <-- 没有任何 /dev/video*"
  echo "          物理层原因（按可能性排序）：USB 口/线缆、相机供电、相机自身故障、"
  echo "          或这是 CSI(MIPI) 相机（那样不会以 /dev/video* 形式出现，要走 nvargus）"
fi

echo
echo "=== 3. 每个节点的驱动名（区分 UVC / metadata / CSI）==="
found=0
for d in /sys/class/video4linux/video*; do
  [ -e "$d" ] || continue
  found=1
  n="$(cat "$d/name" 2>/dev/null)"
  printf '%-8s name=%s\n' "$(basename "$d")" "$n"
  case "$n" in *etadata*) echo "          ^ metadata 节点，不是取流节点（cam_lib.sh 会自动跳过）" ;; esac
done
[ "$found" = 0 ] && echo "(无)"

echo
echo "=== 4. USB 层（UVC 相机应出现在这里）==="
lsusb 2>/dev/null || echo "lsusb 不可用"

echo
echo "=== 5. 内核模块 ==="
lsmod 2>/dev/null | grep -iE '^uvcvideo|^videodev' || echo "(未见 uvcvideo —— 若相机是 CSI/MIPI 走 tegra_camera + nvargus，属正常)"

echo
echo "=== 6. 权限（能否免 sudo 取流）==="
id
echo "用户组: $(id -nG)"
if id -nG | tr ' ' '\n' | grep -qx video; then
  echo "OK: 在 video 组内"
else
  echo "注意: 不在 video 组 —— 若取流报 Permission denied，需手动执行:"
  echo "      sudo usermod -aG video \$USER   （然后重新登录）"
fi

echo
echo "=== 7. 工具链 ==="
for t in gst-launch-1.0 gst-inspect-1.0 gst-device-monitor-1.0 v4l2-ctl ffmpeg python3 ros2; do
  printf '%-24s' "$t"
  cam_have "$t" && echo "OK" || echo "MISSING"
done
echo "说明: 本包只强依赖 gst-launch-1.0；v4l2-ctl/ffmpeg 缺了不影响。"

echo
echo "=== 8. GStreamer 关键插件 ==="
if cam_have gst-inspect-1.0; then
  for p in v4l2src jpegdec jpegenc videoconvert videoscale xvimagesink autovideosink nvvidconv nvjpegdec nvarguscamerasrc; do
    printf '%-18s' "$p"
    gst-inspect-1.0 "$p" >/dev/null 2>&1 && echo "OK" || echo "no"
  done
else
  echo "gst-inspect-1.0 MISSING —— 没有 GStreamer 就只能走 ROS 路线"
fi

echo
echo "=== 9. 设备能力（用 GStreamer 枚举，替代缺失的 v4l2-ctl）==="
if cam_have gst-device-monitor-1.0; then
  timeout 30 gst-device-monitor-1.0 Video 2>&1 | head -120
else
  echo "gst-device-monitor-1.0 MISSING"
fi

echo
echo "=== 10. ROS 2 相机相关包（有 ROS 才看）==="
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  . /opt/ros/humble/setup.bash >/dev/null 2>&1
  [ -f "$HOME/dzpx4/install/setup.bash" ] && . "$HOME/dzpx4/install/setup.bash" >/dev/null 2>&1
  ros2 pkg list 2>/dev/null | grep -iE 'camera|image|v4l2|usb_cam|cv_bridge' | sort \
    || echo "(ros2 可用，但无相机相关包 -> 走纯 GStreamer 路线)"
else
  echo "(此机没有 /opt/ros/humble —— 走纯 GStreamer 路线，本包完全支持)"
fi

echo
echo "=== 11. 图形显示（要开预览窗口才需要）==="
echo "DISPLAY=${DISPLAY:-<未设置>}"
pgrep -a 'Xtigervnc|Xorg|Xwayland' 2>/dev/null | head -5 || echo "(无 X 会话: 只能存图验证，不能开窗)"
if [ -n "${DISPLAY:-}" ] && cam_have xdpyinfo; then
  xdpyinfo 2>/dev/null | grep -E 'dimensions|depth of root'
fi

echo
echo "=== 12. 下一步 ==="
if ls /dev/video* >/dev/null 2>&1; then
  echo "有设备节点 -> 接着跑:  bash cam_shot.sh        # 抓一帧存图，判定是否真出图"
else
  echo "没有设备节点 -> 先解决物理层：优先插 USB3 口、换一根 USB 线、确认相机供电；"
  echo "                插好后重跑本脚本，应能看到新增的 /dev/video*。"
fi
echo "=== DONE ==="
