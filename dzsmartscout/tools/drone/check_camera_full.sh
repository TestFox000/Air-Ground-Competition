#!/bin/bash
echo "=== HOSTNAME ==="
hostname
echo "=== UPTIME ==="
uptime
echo ""
echo "=== LSUSB TREE ==="
lsusb -t
echo ""
echo "=== LSUSB (all devices) ==="
lsusb
echo ""
echo "=== ALL /dev/video* ==="
ls -la /dev/video* 2>&1
echo ""
echo "=== ALL /dev/media* ==="
ls -la /dev/media* 2>&1
echo ""
echo "=== /dev/v4l* ==="
ls -la /dev/v4l* 2>&1
echo ""
echo "=== /sys/class/video4linux2/ ==="
ls -la /sys/class/video4linux2/ 2>&1
echo ""
echo "=== /sys/class/media/ ==="
ls -la /sys/class/media/ 2>&1
echo ""
echo "=== DEVICE TREE (camera/sensor/csi) ==="
find /proc/device-tree -name '*camera*' -o -name '*sensor*' -o -name '*imx*' -o -name '*ov56*' -o -name '*ar02*' -o -name '*csi*' 2>/dev/null
echo "(end device tree search)"
echo ""
echo "=== DMESG USB + CAMERA ==="
dmesg | grep -iE 'uvc|video|camera|imx|ov56|ar02|v4l2|usb .*connect|usb .*disconnect|new full speed|new high speed|new super speed|cdc_acm|ch34' 2>&1
echo "(end dmesg)"
echo ""
echo "=== /dev/ttyACM* ==="
ls -la /dev/ttyACM* 2>&1
echo ""
echo "=== USB SYSFS (all devices) ==="
for d in /sys/bus/usb/devices/*/; do
  if [ -f "${d}idVendor" ]; then
    v=$(cat "${d}idVendor" 2>/dev/null)
    p=$(cat "${d}idProduct" 2>/dev/null)
    prod=$(cat "${d}product" 2>/dev/null)
    mfr=$(cat "${d}manufacturer" 2>/dev/null)
    maxpower=$(cat "${d}bMaxPower" 2>/dev/null)
    echo "${v}:${p}  mfr=[${mfr}]  prod=[${prod}]  maxPower=${maxpower}"
  fi
done
echo ""
echo "=== KERNEL MODULES (camera related) ==="
lsmod | grep -iE 'uvcvideo|videobuf|v4l2|nvcsi|camera|imx|ov56|tegra' 2>&1
echo "(end lsmod)"
echo ""
echo "=== V4L2-CTL if installed ==="
which v4l2-ctl 2>&1 && v4l2-ctl --list-devices 2>&1 || echo "v4l2-ctl not installed"
echo ""
echo "=== GST INSPECT camera elements ==="
gst-inspect-1.0 --exists nvarguscamerasrc 2>&1 && echo "nvarguscamerasrc: exists" || echo "nvarguscamerasrc: not found"
gst-inspect-1.0 --exists v4l2src 2>&1 && echo "v4l2src: exists" || echo "v4l2src: not found"
echo ""
echo "=== DONE ==="
