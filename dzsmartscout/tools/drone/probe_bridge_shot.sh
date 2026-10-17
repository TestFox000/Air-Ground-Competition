#!/bin/bash
# 探测：雷达速率 / dzlink 桥资产 / 截图与 GL 手段（只读）
set +u

echo "=== A. livox rate ==="
bash /tmp/check_lidar_rate.sh 2>&1 | head -32

echo
echo "=== B. dzlink_ws on drone ==="
ls -la /home/duzhong/dzlink_ws/ 2>/dev/null
echo "-- install bin --"
find /home/duzhong/dzlink_ws/install -maxdepth 4 -type f -name pc2_bridge 2>/dev/null
echo "-- build bin --"
ls -la /home/duzhong/dzlink_ws/build/dzlink_pc2_bridge/pc2_bridge 2>/dev/null
echo "-- install setup.bash --"
ls -la /home/duzhong/dzlink_ws/install/setup.bash 2>/dev/null
echo "-- src --"
ls -la /home/duzhong/dzlink_ws/src/dzlink_pc2_bridge/src/ 2>/dev/null

echo
echo "=== C. screenshot / GL tools ==="
command -v glxgears import magick convert xwdtopnm pnmtopng ffmpeg scrot maim gnome-screenshot xwd

echo
echo "=== D. python capture libs ==="
python3 - <<'EOF'
for m in ['Xlib', 'mss', 'pyautogui']:
    try:
        __import__(m)
        print(m, 'OK')
    except Exception:
        print(m, 'MISSING')
try:
    from PIL import Image, ImageGrab
    print('PIL.ImageGrab OK')
    print('xwd support:', Image.registered_extensions().get('.xwd', 'none'))
except Exception as e:
    print('PIL MISSING', e)
EOF

echo
echo "=== E. gnome-screenshot test on :1 ==="
DISPLAY=:1 timeout 25 gnome-screenshot -f /tmp/shot_test.png 2>&1 | head -5
ls -la /tmp/shot_test.png 2>/dev/null || echo "(no png produced)"

echo
echo "=== F. xwd test on :1 ==="
DISPLAY=:1 timeout 25 xwd -root -silent -out /tmp/shot_test.xwd 2>&1 | head -3
ls -la /tmp/shot_test.xwd 2>/dev/null || echo "(no xwd produced)"

echo "=== DONE ==="
