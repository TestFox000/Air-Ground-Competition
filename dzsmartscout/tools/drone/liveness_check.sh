#!/bin/bash
# 活性证明：隔 3 秒抓两帧 VNC :1，比较 RViz 渲染区的像素差
# 若两帧不同 => 画面在实时刷新（而不是一张静止截图/冻结窗口）
set +u

DISPLAY=:1 timeout 25 xwd -root -silent -out /tmp/f1.xwd 2>/dev/null
sleep 3
DISPLAY=:1 timeout 25 xwd -root -silent -out /tmp/f2.xwd 2>/dev/null

python3 - <<'PY'
import sys
sys.path.insert(0, '/tmp')
import numpy as np
from xwd2png import parse_xwd

a, ia = parse_xwd('/tmp/f1.xwd')
b, ib = parse_xwd('/tmp/f2.xwd')
print('frame size:', ia['width'], 'x', ia['height'])

d = np.abs(a.astype(np.int16) - b.astype(np.int16)).sum(axis=2)
print('whole-frame changed pixels(>8): %d (%.2f%%)' % ((d > 8).sum(), 100.0 * (d > 8).sum() / d.size))

# RViz 渲染区（左侧 Displays 面板 430px 与顶栏 90px 之外）
roi = d[95:1130, 440:1900]
changed = (roi > 8).sum()
print('RViz ROI changed pixels(>8): %d / %d = %.2f%%' % (changed, roi.size, 100.0 * changed / roi.size))
print('RViz ROI mean abs diff: %.2f' % roi.mean())
print('VERDICT:', 'LIVE (画面在刷新)' if changed > 2000 else 'STATIC (画面没变，可疑)')
PY
