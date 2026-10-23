#!/bin/bash
# 解锁 VNC :1 的 xfce4 锁屏，并让桌面保持可用（供 RViz 长期显示点云）
# 升级阶梯，每一步都用真实截图判定：查询 -> deactivate -> XTEST 输密码 -> 杀掉锁屏进程
set +u

XWD=/tmp/unlock_probe.xwd

stat_of() {
  DISPLAY=:1 timeout 20 xwd -root -silent -out "$XWD" 2>/dev/null
  python3 - "$XWD" <<'PY'
import sys
sys.path.insert(0, '/tmp')
import numpy as np
from xwd2png import parse_xwd
rgb, info = parse_xwd(sys.argv[1])
u = len(np.unique(rgb.reshape(-1, 3), axis=0))
m = float(rgb.reshape(-1, 3).mean())
print('    unique_colors=%d mean_rgb=%.2f  ->  %s' % (u, m, 'BLACK(仍被遮挡)' if m < 1.0 else 'VISIBLE(有内容)'))
PY
}

echo "=== 1. screensaver state ==="
DISPLAY=:1 timeout 10 xfce4-screensaver-command -q 2>&1 || echo "(command failed)"

echo
echo "=== 2. try deactivate ==="
DISPLAY=:1 timeout 10 xfce4-screensaver-command -d 2>&1 || echo "(deactivate failed)"
sleep 2
DISPLAY=:1 timeout 10 xfce4-screensaver-command -q 2>&1
echo "  capture after deactivate:"
stat_of

echo
echo "=== 3. if still black: type password via XTEST ==="
python3 - <<'PY'
import sys
sys.path.insert(0, '/tmp')
import numpy as np
from xwd2png import parse_xwd
rgb, _ = parse_xwd('/tmp/unlock_probe.xwd')
print('    pre-check mean=%.2f' % float(rgb.reshape(-1, 3).mean()))
PY
DISPLAY=:1 timeout 30 python3 /tmp/xtest_type.py :1 duzhong --enter 2>&1
sleep 3
echo "  capture after typing:"
stat_of

echo
echo "=== 4. if still black: kill the locker process ==="
pkill -f xfce4-screensaver 2>/dev/null && echo "  killed xfce4-screensaver" || echo "  (no xfce4-screensaver running)"
sleep 2
echo "  capture after kill:"
stat_of

echo
echo "=== 5. disable auto-lock/blanking so the desktop stays visible ==="
xfconf-query -c xfce4-screensaver -p /saver/enabled -s false 2>&1 || echo "(xfconf saver/enabled failed)"
xfconf-query -c xfce4-screensaver -p /lock/enabled -s false 2>&1 || echo "(xfconf lock/enabled failed)"
xfconf-query -c xfce4-screensaver -p /saver/idle-activation/enabled -s false 2>&1 || echo "(xfconf idle failed)"
echo "-- verify --"
xfconf-query -c xfce4-screensaver -p /saver/enabled 2>&1
xfconf-query -c xfce4-screensaver -p /lock/enabled 2>&1

echo
echo "=== 6. rviz windows now ==="
DISPLAY=:1 timeout 10 xwininfo -root -tree 2>/dev/null | grep -i -B1 -A2 rviz | head -20

echo "=== DONE ==="
