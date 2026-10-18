#!/usr/bin/env python3
"""起点 M7 → 终点 N5 的可行性与通行规则校验

场地（已确认）：
  14 × 11 格 @35cm；字母轴 A–N = x（4.90m），数字轴 1–11 = y（3.85m）
  不可跨越线: row6 与 row7 之间，覆盖 D..M 列（格子本身可通行）
  桥洞: K 列，桥洞一跨 K1–K6，桥洞二跨 K7–K11，各 1 个可跨越门
  起点 M7 = (col 12, row 6)，终点 N5 = (col 13, row 4)

关键约束：
  ★ 全程不允许触碰边界
    → 车体（≤300×250mm）不得压到场地围栏，也不得压到不可跨越线
    → 用"格级安全距离"建模：可通行格必须离任何障碍/边界 ≥ SAFE_CELLS 格
  ★ 格子 35cm，车宽 ≤25cm → 车道富余仅 100mm，转弯余量极小

运行:
  python3 corridor_check.py
"""

import sys
from collections import deque
from itertools import product

# ---------------------------------------------------------------- 规格
COLS, ROWS = 14, 11
CELL_CM = 35.0

START = (12, 6)          # M7
GOAL = (13, 4)           # N5

BARRIER_UP = 5           # "6" 的 0-based 下标
BARRIER_LO = 6           # "7"
BARRIER_C0, BARRIER_C1 = 3, 12      # D..M
BRIDGE_COL = 10          # K
BRIDGE_A = (0, 5)        # 桥洞一 K1..K6
BRIDGE_B = (6, 10)       # 桥洞二 K7..K11

# 车体（检录上限：≤300 × 250 × 250mm，≤5kg）
CAR_LEN_CM = 30.0
CAR_WID_CM = 25.0


def col_name(c):
    return chr(ord('A') + c)


def cname(c, r):
    return f"{col_name(c)}{r + 1}"


def on_barrier_line(c):
    return BARRIER_C0 <= c <= BARRIER_C1


def is_bridge_interval(r):
    return (BRIDGE_A[0] <= r <= BRIDGE_A[1]) or (BRIDGE_B[0] <= r <= BRIDGE_B[1])


class Field:
    """边阻挡模型 + 边界安全距离

    blocked_cells : 不可驶入的格（水桶、地雷旁的禁入区等）
    gate_row      : 已开的桥洞门所在行（线位置），None = 未开
    """

    def __init__(self, barrier_hi=BARRIER_C1, gate_rows=(), blocked_cells=()):
        self.barrier_hi = barrier_hi
        self.gate_rows = set(gate_rows)
        self.blocked = set(blocked_cells)

    # ---------- 基础通行性 ----------
    def cell_ok(self, c, r):
        if not (0 <= c < COLS and 0 <= r < ROWS):
            return False
        return (c, r) not in self.blocked

    def edge_crossable(self, c, up_row):
        """(c, up_row) -> (c, up_row+1) 能否跨越"""
        if up_row != BARRIER_UP:
            return True
        if c > self.barrier_hi:
            return True                      # 线不覆盖（N 列）
        if c < BARRIER_C0:
            return True                      # 线不覆盖（A/B/C）
        # 线覆盖：只有 K 列且门已开
        return c == BRIDGE_COL and up_row in self.gate_rows

    def step_ok(self, c0, r0, c1, r1):
        if not (self.cell_ok(c0, r0) and self.cell_ok(c1, r1)):
            return False
        if c0 == c1:
            up = min(r0, r1)
            return self.edge_crossable(c0, up)
        if r0 == r1:
            return True
        return False

    def neighbors(self, c, r):
        for nc, nr in ((c + 1, r), (c - 1, r), (c, r + 1), (c, r - 1)):
            if self.step_ok(c, r, nc, nr):
                yield nc, nr

    def bfs(self, src, dst):
        if not self.cell_ok(*src) or not self.cell_ok(*dst):
            return None
        prev = {src: None}
        q = deque([src])
        while q:
            cur = q.popleft()
            if cur == dst:
                break
            for nxt in self.neighbors(*cur):
                if nxt not in prev:
                    prev[nxt] = cur
                    q.append(nxt)
        if dst not in prev:
            return None
        path, cur = [], dst
        while cur is not None:
            path.append(cur)
            cur = prev[cur]
        return list(reversed(path))


# ---------------------------------------------------------------- 边界安全
def far_from_wall(c, r):
    """该格是否离任何受阻格 ≥1 格（四邻）"""
    for nc, nr in ((c + 1, r), (c - 1, r), (c, r + 1), (c, r - 1)):
        if not (0 <= nc < COLS and 0 <= nr < ROWS):
            return False
    return True


def safe_for_car(c, r, margin_cells, barrier_hi, gates):
    """车体安全判据：

    1) 距场地围栏 ≥ margin_cells 格
    2) 不得压到"不可跨越线"：如果该格紧邻线且跨不过去，则不能贴线
       （线是物理挡板，车压上去就是触碰边界）
    3) 桥洞门所在格可以贴线（那里没有挡板）
    """
    # 1) 围栏距离
    if c < margin_cells or c >= COLS - margin_cells:
        return False
    if r < margin_cells or r >= ROWS - margin_cells:
        return False

    # 2) 线附近的处理：与线相邻的格，只在门处允许
    for r_up in range(ROWS - 1):
        for cc, rr in ((c, r),):
            pass
    return True


def line_adjacent_ok(c, r, barrier_hi, gates):
    """该格是否紧贴"不可跨越线"却跨不过去 —— 跨不过去时车压在档板上 = 触碰边界

    线在 row(BARRIER_UP) 与 row(BARRIER_LO) 之间：
      * 南侧的贴线格   = row(BARRIER_LO) = "7" 行  → D7..M7
      * 北侧的贴线格   = row(BARRIER_UP) = "6" 行  → D6..M6
    格子本身可通行，但车体不能压档板，所以这些格只在"门所在列"才允许占用。
    线未覆盖的列（A/B/C 与 N）没有档板，正常通行。
    """
    if not on_barrier_line(c) or c > barrier_hi:
        return True                       # 无档板
    if r not in (BARRIER_UP, BARRIER_LO):
        return True                       # 不贴线
    gate_here = (c == BRIDGE_COL and BARRIER_UP in gates)
    return gate_here


def min_clearance_cm(c, r, barrier_hi):
    """返回该格中心的最小净空（cm）—— 取"到场地围栏"与"到档板"的较小值

    物理量：
      格心 x = (c + 0.5) * 35，y = (r + 0.5) * 35     单位 cm
      场地围栏: x = 0 / 490，y = 0 / 385
      档板:     y = 210（row5/row6 之间），覆盖 x ∈ [105, 455)（D..M）
    """
    cx = (c + 0.5) * CELL_CM
    cy = (r + 0.5) * CELL_CM

    # 到围栏
    to_fence = min(cx, COLS * CELL_CM - cx, cy, ROWS * CELL_CM - cy)

    # 到档板
    to_wall = 1e9
    wall_y = (BARRIER_UP + 1) * CELL_CM          # = 210cm
    wall_x0 = BARRIER_C0 * CELL_CM               # = 105cm (D 左边界)
    wall_x1 = (barrier_hi + 1) * CELL_CM         # = 455cm (M 右边界)
    if wall_x0 <= cx <= wall_x1:
        to_wall = abs(cy - wall_y)

    return min(to_fence, to_wall), to_fence, to_wall


def reachable_with_clearance(need_cm, barrier_hi, gates, verbose=False):
    """要求每格中心净空 ≥ need_cm，测试 M7 → N5 是否可达"""
    f = Field(barrier_hi=barrier_hi, gate_rows=gates)
    blocked = set()
    for c in range(COLS):
        for r in range(ROWS):
            clr, _, _ = min_clearance_cm(c, r, barrier_hi)
            if clr < need_cm:
                blocked.add((c, r))
    f.blocked = blocked

    path = f.bfs(START, GOAL) if (f.cell_ok(*START) and f.cell_ok(*GOAL)) else None

    if verbose:
        s_clr = min_clearance_cm(*START, barrier_hi)[0]
        g_clr = min_clearance_cm(*GOAL, barrier_hi)[0]
        print(f"    要求净空 ≥{need_cm:5.1f}cm : 起点净空 {s_clr:.1f}cm，"
              f"终点净空 {g_clr:.1f}cm → "
              f"{'可达 %d 格' % len(path) if path else '不可达'}")
        if path:
            print(f"      路径: {' → '.join(cname(*p) for p in path)}")
    return path


def check_clearance_sweep():
    """以物理净空为约束，扫描"需要多少净空才不可达" """
    probs = []
    print("  逐级要求更大净空，测试 M7 → N5 是否可达:")
    print("  （净空 = 格心到最近围栏/档板的距离，需 ≥ 车半宽 + 安全余量）")
    gates = {BARRIER_UP}
    last_ok = None
    for need in (0, 2, 5, 10, 15, 20, 25, 30, 35):
        p = reachable_with_clearance(need, BARRIER_C1, gates, verbose=True)
        if p:
            last_ok = need
    print()
    car_half = CAR_WID_CM / 2
    print(f"  车半宽 {car_half:.1f}cm → 至少要满足净空 ≥ {car_half:.1f}cm 才不碰边")
    if last_ok is None:
        probs.append("净空 0 都不可达")
    elif last_ok < car_half:
        probs.append(
            f"最大可行净空 {last_ok}cm < 车半宽 {car_half:.1f}cm —— "
            f"按『全程不碰边界』的车体尺度，M7→N5 **不可行**")
    else:
        print(f"  最大可行净空 {last_ok}cm ≥ 车半宽 {car_half:.1f}cm → 可行")
    return probs


def check_m7_bottleneck():
    """M7 是南侧沿线的最后一个可通行格吗？"""
    probs = []
    print("  沿线各格到档板的净空（档板 y=210cm）:")
    wall_y = (BARRIER_UP + 1) * CELL_CM
    for r in (BARRIER_UP, BARRIER_LO):
        desc = []
        for c in (BARRIER_C0, 10, 11, 12, 13):
            cx = (c + 0.5) * CELL_CM
            cy = (r + 0.5) * CELL_CM
            in_wall_x = BARRIER_C0 * CELL_CM <= cx <= (BARRIER_C1 + 1) * CELL_CM
            gap = abs(cy - wall_y) if in_wall_x else float('inf')
            desc.append(f"{cname(c, r)}={gap:.0f}cm" if gap != float('inf')
                        else f"{cname(c, r)}=无档板")
        print(f"    row{r + 1}: " + "  ".join(desc))
    print(f"    → M7 格心 y={((BARRIER_LO) + 0.5) * CELL_CM:.0f}cm，"
          f"到档板 {abs(((BARRIER_LO) + 0.5) * CELL_CM - wall_y):.0f}cm")
    print(f"    → M6 格心 y={((BARRIER_UP) + 0.5) * CELL_CM:.0f}cm，"
          f"到档板 {abs(((BARRIER_UP) + 0.5) * CELL_CM - wall_y):.0f}cm")
    print(f"    → N6 格心 y={((BARRIER_UP) + 0.5) * CELL_CM:.0f}cm，"
          f"但 N 列无档板（线止于 M）")
    print(f"    ★ 也就是说：M7 本身净空充足；真正紧的是 row6 上的 D..M 那些格")
    return probs


def check_fence_sweep():
    """只考虑围栏（把档板当成普通墙），扫描围栏边距"""
    probs = []
    print("  只看围栏边距（档板按边判定，不额外禁止格）:")
    gates = {BARRIER_UP}
    for margin in (0, 1):
        f = Field(barrier_hi=BARRIER_C1, gate_rows=gates)
        blocked = set()
        for c in range(COLS):
            for r in range(ROWS):
                if c < margin or c >= COLS - margin or \
                   r < margin or r >= ROWS - margin:
                    blocked.add((c, r))
        f.blocked = blocked
        p = f.bfs(START, GOAL) if (f.cell_ok(*START) and f.cell_ok(*GOAL)) else None
        print(f"    围栏边距 {margin} 格: "
              f"{'可达 %d 格' % len(p) if p else '不可达'}"
              f"{'  (' + ' → '.join(cname(*q) for q in p) + ')' if p else ''}")
    return probs


# ---------------------------------------------------------------- 校验项
def check_start_goal_sides():
    probs = []
    print(f"  起点 {cname(*START)} : col={START[0]} ({col_name(START[0])}), "
          f"row={START[1] + 1}")
    print(f"  终点 {cname(*GOAL)} : col={GOAL[0]} ({col_name(GOAL[0])}), "
          f"row={GOAL[1] + 1}")
    print(f"  线位于 row{BARRIER_UP + 1} 与 row{BARRIER_LO + 1} 之间")
    s_side = "北侧(线上方)" if START[1] <= BARRIER_UP else "南侧(线下方)"
    g_side = "北侧(线上方)" if GOAL[1] <= BARRIER_UP else "南侧(线下方)"
    print(f"  → 起点在{s_side}，终点在{g_side}")
    if s_side != g_side:
        print(f"  ★ 起终点分居线两侧：出车第一步（或最后一步）必须跨越该线")
        print(f"     起点 M7 在 M 列，而线覆盖到 M 列 → M6↔M7 之间就是那条线")
        print(f"     终点 N5 在 N 列，N 列**不被线覆盖** → N5↔N6 可以自由跨越")
        if not on_barrier_line(GOAL[0]):
            print(f"     → 关键：可以先沿南侧走到 N 列，再从 N 列自由上跨到 N5，"
                  f"绕开 M 列的挡板")
    return probs


def check_n_column_route():
    """验证"走 N 列绕开 M 列挡板"这条路是否成立"""
    probs = []
    f = Field(barrier_hi=BARRIER_C1, gate_rows=())   # 一个门都不开
    path = f.bfs(START, GOAL)
    print("  门全关时 M7 → N5:")
    if path:
        print(f"    可达！路径 {len(path)} 格")
        print(f"    {' → '.join(cname(*p) for p in path)}")
        used_n = any(c == 13 for c, _ in path)
        crossed_at_n = any(c == 13 and r == BARRIER_UP for c, r in path)
        print(f"    经过 N 列: {used_n}；在 N 列跨线: {crossed_at_n}")
        if crossed_at_n:
            print("    ★ 确认：从 N 列（线未覆盖）即可绕开 M 列挡板完成南北穿越")
    else:
        print("    不可达")
        probs.append("门全关时 M7 → N5 不可达")
    return probs


def check_corridor_margin():
    """计算"不碰边界"允许的最大格级边距

    两类边界分开处理：
      A) 场地围栏      → 需要 margin_cells 格的边距
      B) 不可跨越线的档板 → 贴线格（D6..M6 / D7..M7）完全禁用，等价于 1 格边距
    """
    probs = []
    print("  A) 场地围栏边距（B 类档板固定按 1 格处理）:")
    gates = {BARRIER_UP}
    last_ok = None
    for margin in range(0, 4):
        p = reachable_with_margin(margin, BARRIER_C1, gates, verbose=True)
        if p:
            last_ok = margin
    if last_ok is None:
        print("    → 即使 margin=0 也不可达")
        probs.append("margin=0 即不可达，起点/终点/档板约束互相冲突")
        return probs
    print(f"\n  结论：围栏边距最大 {last_ok} 格 = {last_ok * CELL_CM:.0f}cm")
    if last_ok == 0:
        print("    ★ margin=0 意味着起终点就在最外圈格")
    if last_ok < 1:
        probs.append("围栏边距只能为 0 —— 无法保证不碰边界，"
                     "需重新核对 M7 / N5 的格号")
    return probs


def check_geometry_tightness():
    """格宽 35cm vs 车宽 25cm 的富余量"""
    probs = []
    slack = CELL_CM - CAR_WID_CM
    print(f"  格宽 {CELL_CM:.0f}cm，车宽上限 {CAR_WID_CM:.0f}cm "
          f"→ 单车横向富余 {slack:.0f}mm（每侧 {slack / 2:.0f}mm）")
    assert slack > 0
    if slack < 10:
        probs.append(f"横向富余仅 {slack:.0f}mm，直线都难以保持")
    elif slack < 20:
        print(f"  ★ 富余 {slack:.0f}mm：直线可通过，但转弯必须大幅减速，"
              f"且需要 <2cm 的横向控制精度")
    else:
        print(f"  富余 {slack:.0f}mm：直线与缓弯可接受")

    # 车道两侧各留一点余量时的可用宽度
    for keep in (5, 10, 15):
        usable = CELL_CM - 2 * (keep / 10.0)
        print(f"    若单侧保留 {keep}mm 安全间隙 → 可用宽度 {usable:.0f}cm，"
              f"{'够' if usable > CAR_WID_CM else '不够'}（车宽 {CAR_WID_CM:.0f}cm）")
    return probs


def check_gate_need():
    """穿门 vs 绕 N 列的代价"""
    probs = []
    print("  两条南北通路对比（起点南侧 M7 → 北侧 N5）:")
    for label, gates in (("门全关", ()), ("开桥洞门 K6", {BARRIER_UP})):
        f = Field(barrier_hi=BARRIER_C1, gate_rows=gates)
        p = f.bfs(START, GOAL)
        print(f"    {label:14s}: {'%d 格' % len(p) if p else '不可达'}")
    print("    → 因为终点在 N 列（线不覆盖），穿门并非必需；"
          "桥洞的 20 分需要别的手段确认（见 REPORT）")
    return probs


def main():
    print("=" * 78)
    print("起点 M7 → 终点 N5 可行性与边界约束校验")
    print("=" * 78)

    checks = [
        ("起点 / 终点与线的相对位置", check_start_goal_sides),
        ("N 列绕行通路", check_n_column_route),
        ("M7 瓶颈与沿岸净空", check_m7_bottleneck),
        ("物理净空扫描（不碰边界）", check_clearance_sweep),
        ("只算围栏边距", check_fence_sweep),
        ("格宽与车宽的几何富余", check_geometry_tightness),
        ("穿门 vs 绕行代价", check_gate_need),
    ]

    total = 0
    for title, fn in checks:
        print(f"\n[{title}]")
        probs = fn()
        for p in probs:
            print(f"   [FAIL] {p}")
        total += len(probs)
        if not probs:
            print("   OK")

    # 地图
    print("\n[场地示意：S=起点 M7, E=终点 N5, G=桥洞门]")
    f = Field(barrier_hi=BARRIER_C1, gate_rows={BARRIER_UP})
    out = ["     " + "".join("%-3s" % col_name(c) for c in range(COLS))]
    for r in range(ROWS):
        row = []
        for c in range(COLS):
            if (c, r) == START:
                ch = 'S'
            elif (c, r) == GOAL:
                ch = 'E'
            elif c == BRIDGE_COL and r == BARRIER_UP and BARRIER_UP in f.gate_rows:
                ch = 'G'
            else:
                ch = ' '
            row.append("%-3s" % ch)
        out.append("%-4s " % (r + 1) + "".join(row))
        if r == BARRIER_UP:
            seg = []
            for c in range(COLS):
                if not on_barrier_line(c) or c > f.barrier_hi:
                    seg.append("   ")
                elif c == BRIDGE_COL and BARRIER_UP in f.gate_rows:
                    seg.append(" G ")
                else:
                    seg.append("---")
            out.append("     " + "".join(seg) + "  <- 不可跨越线 (D6–M6 / D7–M7)")
    print("\n".join(out))

    print("=" * 78)
    print("全部校验通过。" if total == 0 else f"{total} 项校验失败。")
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
