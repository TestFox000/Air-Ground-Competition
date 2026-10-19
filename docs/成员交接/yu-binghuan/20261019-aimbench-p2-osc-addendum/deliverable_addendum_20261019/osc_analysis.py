# -*- coding: utf-8 -*-
"""rail@267 基线振荡结构定量：频率、增长包络、相位关系。"""
import csv
import math

path = (r'C:\Users\29212\Documents\deepseek-harness\default-workspace\_cloud\bench'
        r'\runs\rail_beta02_267_nolead_213155728\data.csv')
ref, fb = [], []
with open(path, encoding='utf-8') as f:
    for r in csv.reader(f):
        if r[0] == 't':
            continue
        if r[1] == 'ref':
            ref.append((float(r[0]), float(r[2])))          # (t, target_deg)
        elif r[1] == 'fb':
            fb.append((float(r[0]), (float(r[3]) - 2047) / 11.378))  # yaw tick -> deg

# 50Hz 误差序列 e(t) = target - gimbal
errs = []
j = 0
for t, ang in fb:
    while j < len(ref) - 1 and ref[j][0] < t:
        j += 1
    tgt = ref[min(j, len(ref) - 1)][1]
    errs.append((t, tgt - ang))
errs.sort()

# 振荡频率：稳态段(t>6s)过零间隔
tail = [e for e in errs if e[0] > 6.0]
zc = [i for i in range(1, len(tail)) if tail[i][1] * tail[i - 1][1] < 0]
if len(zc) > 2:
    periods = [(tail[i][0] - tail[k][0]) * 2 for i, k in zip(zc[1:], zc[:-1])]
    periods = [p for p in periods if 0.2 < p < 6]
    med_p = sorted(periods)[len(periods) // 2]
    print(f'过零 {len(zc)} 次 → 振荡周期中位 {med_p:.2f}s = {1/med_p:.2f}Hz '
          f'（纯延迟失稳理论频率 1/(2×0.267)={1/0.534:.2f}Hz）')

# 增长包络：逐 3s 窗的 |e| 峰值
print('包络(|e|峰值/3s窗):')
for i in range(0, 27, 3):
    w = [abs(e) for t, e in errs if i <= t < i + 3]
    if w:
        print(f'  [{i:2d}-{i+3:2d}s) max {max(w):5.2f}°  rms {math.sqrt(sum(x*x for x in w)/len(w)):5.2f}°')

# 分段：早期(收敛前) vs 稳态后
early = [abs(e) for t, e in errs if t < 4]
grown = [abs(e) for t, e in errs if 18 < t < 28]
print(f'早期(0-4s) |e| rms {math.sqrt(sum(x*x for x in early)/len(early)):.2f}°  '
      f'| 成熟期(18-28s) rms {math.sqrt(sum(x*x for x in grown)/len(grown)):.2f}°')
