# -*- coding: utf-8 -*-
"""解析 run_log.txt 的最终地图与航点, 定点核对挡板线画在哪、路径是否真越线。"""
import os, re, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mission_planner as M

_log = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "run_log.txt")
lines = open(_log, encoding="utf-8").read().splitlines()

# --- 1) 定位报告段 ---
fi = next(i for i, l in enumerate(lines) if l.startswith("最终地图"))
wpt_line = next(l for l in lines[fi:] if l.startswith("全流程线路"))
print("航点行: ", wpt_line)

# --- 2) 定点解析最终地图: 每行格式 " R  c1 c2 ... c11"(每格1字符+1空格, 挡板行中间多一个|) ---
rows = {}
for l in lines[fi:]:
    m = re.match(r"^ ([A-N])  (.*)$", l)
    if m:
        rows[m.group(1)] = m.group(2)
print("\n--- 渲染出的挡板线位置(定点解析) ---")
drawn_boundaries = set()
for letter, joined in rows.items():
    j = joined.find("|")
    if j == -1:
        print(f"  行{letter}: 无挡板线")
        continue
    before = j // 2          # 线前有 j//2 个格子
    print(f"  行{letter}: '|'在字符位{j} -> 线画在第{before}格与第{before+1}格之间 "
          f"(格内容: {' '.join(joined[0::2])!r})")
    drawn_boundaries.add(before)

# --- 3) 代码事实对照 ---
print("\n--- 代码事实 ---")
print(f"  真实约束 barrier_blocks_cross: 禁止第{M.BARRIER_COLS[0]}列↔第{M.BARRIER_COLS[1]}列 (D~N 行)")
print(f"  ASCII 渲染: parts.insert({max(M.BARRIER_COLS)-1},'|') -> 画在第{M.BARRIER_COLS[0]}|{M.BARRIER_COLS[1]}列之间")
print(f"  PNG 渲染(map_render): b_col={max(M.BARRIER_COLS)-1} -> 第{M.BARRIER_COLS[0]}|{M.BARRIER_COLS[1]}列之间")
print(f"  解析到的实际画线位置: 第{sorted(drawn_boundaries)}|{sorted(x+1 for x in drawn_boundaries)}列之间")

# --- 4) 航点逐边核对 ---
cells = re.findall(r"([A-N]\d{1,2})(?:\((?:踩雷|停靠|打击|终点|桥洞)\))?", wpt_line)
rcs = [M.name2rc(c) for c in cells]
full = [rcs[0]]
for a, b in zip(rcs, rcs[1:]):
    dr = (b[0] > a[0]) - (b[0] < a[0])
    dc = (b[1] > a[1]) - (b[1] < a[1])
    cur = a
    while cur != b:
        cur = (cur[0] + dr, cur[1] + dc)
        full.append(cur)

bad_real = [(M.rc2name(a), M.rc2name(b)) for a, b in zip(full, full[1:])
            if M.barrier_blocks_cross(a, b)]
lo, hi = M.ROW_LETTERS.index(M.BARRIER_LO_ROW), M.ROW_LETTERS.index(M.BARRIER_HI_ROW)
k = min(drawn_boundaries)  # 画线在第 k|k+1 列之间(1 基)
bad_drawn = [(M.rc2name(a), M.rc2name(b)) for a, b in zip(full, full[1:])
             if lo <= min(a[0], b[0]) and max(a[0], b[0]) <= hi
             and {a[1], b[1]} == {k - 1, k}]

print(f"\n--- 路径核对 ({len(cells)} 航点, 展开 {len(full)} 格) ---")
print(f"  违反真实挡板(第{M.BARRIER_COLS[0]}|{M.BARRIER_COLS[1]}列)的边: {len(bad_real)} 条 {bad_real}")
print(f"  穿过画出的线(第{k}|{k+1}列)的边: {len(bad_drawn)} 条 {bad_drawn}")
if bad_real:
    print("\n结论: 路径确实压了真挡板!")
elif k != min(M.BARRIER_COLS):
    print("\n结论: 路径未跨真挡板, 但画线与禁跨约束位置不一致(渲染仍偏格)!")
else:
    print("\n结论: 图文一致——挡板线画在第6|7列, 禁跨约束也在第6|7列, 路径零跨越。")
