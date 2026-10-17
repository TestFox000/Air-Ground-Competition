#!/bin/bash
# Camera reconnaissance for dzsmart (Jetson Orin Nano).
# READ-ONLY: no file writes, no process starts, no package installs.
# Run locally via:  Get-Content -Raw tools/drone/cam_recon.sh | ssh duzhong@dzsmart.local bash -s

echo "=== HOST ==="
hostname; uname -r; cat /etc/nv_tegra_release 2>/dev/null | head -1

echo "=== VIDEO DEVICES (/dev/video*) ==="
ls -l /dev/video* 2>/dev/null || echo "NONE"

echo "=== V4L2 DEVICE DETAIL ==="
if command -v v4l2-ctl >/dev/null 2>&1; then
  v4l2-ctl --list-devices 2>&1
else
  echo "v4l2-ctl MISSING (package v4l-utils not installed)"
fi

echo "=== USB DEVICES ==="
lsusb 2>/dev/null || echo "lsusb MISSING"

echo "=== RUNNING: lidar / vnc / rviz ==="
pgrep -a -f livox_ros_driver2 | head -3
pgrep -a Xtigervnc | head -3
pgrep -a rviz2 | head -3

echo "=== ROS 2 PACKAGES (camera/image related) ==="
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source "$HOME/dzpx4/install/setup.bash" >/dev/null 2>&1
ros2 pkg list 2>/dev/null | grep -iE 'camera|image|usb_cam|v4l2|gscam|video' | sort

echo "=== CLI TOOLS ==="
for t in ffmpeg gst-launch-1.0 v4l2-ctl rqt_image_view ros2 nvargus-daemon; do
  printf '%-20s ' "$t"
  command -v "$t" 2>/dev/null || echo "MISSING"
done

echo "=== GSTREAMER PLUGINS (camera/v4l2/jpeg) ==="
if command -v gst-inspect-1.0 >/dev/null 2>&1; then
  gst-inspect-1.0 2>/dev/null | grep -iE 'v4l2src|jpegdec|nvvidconv|xvimagesink|v4l2src' | head -20
else
  echo "gst-inspect-1.0 MISSING"
fi

echo "=== JETSON CSI / ARGUS ==="
ls -l /dev/video* 2>/dev/null | wc -l
ls /sys/class/video4linux 2>/dev/null || echo "no video4linux class"
systemctl is-active nvargus-daemon 2>/dev/null || echo "nvargus-daemon not active"

echo "=== VNC SESSION ==="
pgrep -a Xtigervnc | head -2
DISPLAY=:1 xdpyinfo 2>/dev/null | grep -E 'dimensions|depth of root' || echo "DISPLAY :1 not queryable"

echo "=== APT: candidate camera packages ==="
apt-cache policy ros-humble-v4l2-camera ros-humble-usb-cam ros-humble-image-view ros-humble-rqt-image-view v4l-utils 2>/dev/null | grep -E '^[a-z]|Candidate' | head -30

echo "=== DONE ==="
