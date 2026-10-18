# -*- coding: utf-8 -*-
import re
p = r"D:/User/Desktop/比赛存档/v1/_tmp_session/t28_run/3_localization.log"
pat = re.compile(r"pos: \(([-0-9.]+), ([-0-9.]+)\) \| block: (\S+) \| yaw: ([-+0-9.]+)deg \[(\w+)\]")
rows=[]
for line in open(p, encoding='utf-8', errors='replace'):
    m = pat.search(line)
    if m:
        ts = re.search(r"\[(\d+\.\d+)\]", line)
        rows.append((float(ts.group(1)) if ts else 0.0, float(m.group(1)), float(m.group(2)), m.group(3), float(m.group(4)), m.group(5)))
print("rows", len(rows))
prev=None
shown=0
for r in rows:
    if prev:
        d = ((r[1]-prev[1])**2 + (r[2]-prev[2])**2) ** 0.5
        if d > 0.8 and shown < 25:
            print(f"JUMP dt={r[0]-prev[0]:.2f}s d={d:.2f}m  ({prev[1]:.2f},{prev[2]:.2f}){prev[3]}[{prev[5]}] -> ({r[1]:.2f},{r[2]:.2f}){r[3]}[{r[5]}] yaw {prev[4]:.0f}->{r[4]:.0f}")
            shown += 1
    prev = r
# print around first big jump
print("--- first 5 and last 5 ---")
for r in rows[:3]+rows[-3:]:
    print(f"t={r[0]:.1f} pos=({r[1]:.2f},{r[2]:.2f}) block={r[3]} yaw={r[4]:.0f} [{r[5]}]")
