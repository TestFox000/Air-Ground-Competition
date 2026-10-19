# -*- coding: utf-8 -*-
"""合成靶台架验收分析：平滑律的指令连续性 / 反馈跟随 / 误差收敛。"""
import csv
import math
import statistics as st

path = r'C:\Users\29212\Desktop\赛事临时\v1\.ssh2-tmp\synth_bench.csv'
rows = []
with open(path, encoding='utf-8') as f:
    for r in csv.reader(f):
        if r and r[0] != 't':
            rows.append((float(r[0]), r[1], float(r[2]), float(r[3]), float(r[4]), float(r[5])))
T = rows[-1][0]
off = [r for r in rows if r[1] == 'off']
cmd = [r for r in rows if r[1] == 'cmd']
fb = [r for r in rows if r[1] == 'fb']
print(f'span {T:.1f}s  off={len(off)} cmd={len(cmd)} ({len(cmd)/T:.1f}Hz) fb={len(fb)}')

# ---- 指令连续性（核心指标：卡顿=长冻结+大跳变）----
yb = [(r[0], r[3]) for r in cmd]
same = sum(1 for i in range(1, len(yb)) if yb[i][1] == yb[i-1][1])
chg = [(yb[i][0], yb[i][1] - yb[i-1][1]) for i in range(1, len(yb)) if yb[i][1] != yb[i-1][1]]
run = cur = 1
for i in range(1, len(yb)):
    run = run + 1 if yb[i][1] == yb[i-1][1] else 1
    cur = max(cur, run)
print(f'指令连续性: 相邻相同 {same}/{len(yb)-1}（{same/(len(yb)-1)*100:.0f}%，含到位后合理恒定）')
print(f'  变化 {len(chg)} 次 | 步长中位 {st.median([abs(c[1]) for c in chg]):.1f} tick '
      f'P95 {sorted(abs(c[1]) for c in chg)[int(len(chg)*0.95)]:.0f} max {max(abs(c[1]) for c in chg):.0f} tick')
print(f'  最长冻结 {cur} 条 ≈ {cur/(len(cmd)/T)*1000:.0f}ms（旧律实测冻结 12000ms）')

# ---- 反馈跟随 ----
fy = [(r[0], r[3]) for r in fb]
pos = [p for _, p in fy]
ts = [t for t, _ in fy]
vels = []
for i in range(1, len(fy)):
    dt = ts[i] - ts[i-1]
    if dt > 1e-3:
        vels.append((pos[i] - pos[i-1]) / dt)
nz = [v for v in vels if abs(v) > 5]
flips = sum(1 for i in range(1, len(nz)) if nz[i] * nz[i-1] < 0)
print(f'反馈 yaw: [{min(pos):.0f}, {max(pos):.0f}] tick（扫幅 {max(pos)-min(pos):.0f}，应≈±128 跟随靶）')
print(f'  非零速度样本 {len(nz)}/{len(vels)} | 速度符号翻转 {flips} 次（来回摆动的自然翻转，中位间隔 '
      f'{(T/max(flips,1))*1000:.0f}ms）')

# ---- 指令-反馈跟踪误差 ----
errs = []
ci = 0
for i in range(len(fy)):
    while ci < len(yb) - 1 and yb[ci][0] < ts[i]:
        ci += 1
    errs.append(abs(yb[min(ci, len(yb)-1)][1] - pos[i]))
print(f'指令-反馈偏差: 中位 {st.median(errs):.0f} tick max {max(errs):.0f}（>50=追不上）')

# ---- 视觉误差（合成靶 dx 与真实 det 帧混流；只统计有靶帧）----
sdx = [r[2] for r in off if r[4] == 1]
if sdx:
    print(f'有靶帧 dx: [{min(sdx):.0f},{max(sdx):.0f}] 中位 {st.median(sdx):.0f} '
          f'|dy| 中位 {st.median(abs(r[3]) for r in off if r[4] == 1):.0f} px')
    ib = sum(1 for r in off if r[4] == 1 and abs(r[2]) <= 18 and abs(r[3]) <= 18)
    print(f'  开火框内(|dx|&|dy|≤18) {ib}/{len(sdx)} = {ib/len(sdx)*100:.0f}%')
