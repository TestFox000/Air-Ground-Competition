#!/usr/bin/env python3
"""FieldGrid 参考实现 —— 场地拓扑校验

规格（与队伍确认）：
  场地 4m × 5m，栅格 35cm
  字母轴 A..N = 14 格（x，4.90m）；数字轴 1..11 = 11 格（y，3.85m）

  不可跨越线: D6–M6 与 D7–M7 之间存在一条不可跨越的线，
              **但这些格子本身可通行**（线在格与格之间，不在格内）。
              桥洞大小与格一致，并与格线重合。
  桥洞:      K 列，桥洞一跨 K1–K6、桥洞二跨 K7–K11；
              每个桥洞恰有一个可跨越的"门"。

  本脚本把两种"线"的语义都跑一遍，并明确报告哪一种自洽 —— 这决定了
  车端路径规划必须知道什么信息：

    [语义 A] 线只禁止"纵向跨越"（D6 ↛ D7）
              → 从 D 列横移到 A/B/C 或 N 列再纵向即可绕过，
                此时桥洞不是唯一通路，"10 分协同分"可以被绕开拿到。
    [语义 B] 线是真正的隔离带：跨过 row6→row7 必须经过桥洞门
              → 桥洞是唯一近路，与"桥洞 10 分"的设置相符。

运行:
  python3 field_grid_check.py
"""

import sys
from collections import deque

# ---------------------------------------------------------------- 规格
COLS = 14          # A..N
ROWS = 11          # 1..11
CELL = 0.35        # m

BARRIER_UP = 5     # "6" 的 0-based 下标（线的上侧行）
BARRIER_LO = 6     # "7" 的 0-based 下标（线的下侧行）
BARRIER_C0 = 3     # D
BARRIER_C1 = 12    # M（不含 N）

BRIDGE_COL = 10    # K
BRIDGE_A = (0, 5)  # 桥洞一：K1–K6
BRIDGE_B = (6, 10) # 桥洞二：K7–K11

FREE, BLOCKED, MINE = 0, 1, 2


def col_name(c):
    return chr(ord('A') + c)


def cname(c, r):
    return f"{col_name(c)}{r + 1}"


def on_line(c):
    """该列是否被那条线覆盖"""
    return BARRIER_C0 <= c <= BARRIER_C1


def bridge_interval(row):
    if BRIDGE_A[0] <= row <= BRIDGE_A[1]:
        return 'A'
    if BRIDGE_B[0] <= row <= BRIDGE_B[1]:
        return 'B'
    return None


class Grid:
    """cells[r][c] 为格内通行性；cross_ok[c] 为"该列能否跨过那条线"

    cross_ok 语义由语义 A/B 决定：
      A: 只对 K 列按门设置，其它列 True（格内可走，线只挡纵向边）
      B: 非 K 列且被线覆盖的列 False（真正隔离）
    """

    def __init__(self, semantics='B'):
        self.cells = [[FREE] * COLS for _ in range(ROWS)]
        self.semantics = semantics
        self.gate_row = None      # 已开的门（K 列上的跨越位置）

    def open_gate(self, row):
        if bridge_interval(row) is None:
            return False
        self.gate_row = row
        return True

    def passable(self, c, r):
        return 0 <= c < COLS and 0 <= r < ROWS and self.cells[r][c] in (FREE, MINE)

    def cross_allowed(self, c, r_from_barrier_up):
        """从 (c, BARRIER_UP) 跨到 (c, BARRIER_LO) 是否允许"""
        if not on_line(c):
            return True                     # 线不覆盖：A/B/C 与 N
        if self.semantics == 'A':
            return True                     # 语义 A：线只挡纵向，横向可绕
        # 语义 B：真正隔离，只有 K 列的门能跨
        if c != BRIDGE_COL:
            return False
        if self.gate_row is None:
            return False
        return self.gate_row == BARRIER_UP

    def edge_blocked_by_line(self, c, r_from, r_to):
        """判断 (c, r_from)→(c, r_to) 这条纵向边是否被那条线封死

        线在 row6↔row7 之间，覆盖 D..M 列。
        线覆盖的所有列（含 K 列）默认封死；只有门的边开放。
        """
        up, lo = (r_from, r_to) if r_to > r_from else (r_to, r_from)
        if not (up == BARRIER_UP and lo == BARRIER_LO):
            return False                    # 不是那条线所在的位置
        if not on_line(c):
            return False                    # A/B/C 与 N：线不覆盖
        if self.semantics == 'A':
            return False                    # 语义 A：线只挡"就地纵向"，可靠横移绕
        # 语义 B：线覆盖的列全部封死，只有门这一格例外
        return not (c == BRIDGE_COL and self.gate_row == BARRIER_UP)

    def neighbors(self, c, r):
        out = []
        for nc, nr in ((c + 1, r), (c - 1, r), (c, r + 1), (c, r - 1)):
            if not self.passable(nc, nr):
                continue
            if nc == c and self.edge_blocked_by_line(c, r, nr):
                continue
            out.append((nc, nr))
        return out

    def reach(self, src):
        if not self.passable(*src):
            return set()
        seen = {src}
        q = deque([src])
        while q:
            cur = q.popleft()
            for nxt in self.neighbors(*cur):
                if nxt not in seen:
                    seen.add(nxt)
                    q.append(nxt)
        return seen

    def bfs(self, src, dst):
        if not self.passable(*src) or not self.passable(*dst):
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

    def crossed(self, reach, from_row=0):
        """从 from_row 出发，是否到达了线下方"""
        return any(r > BARRIER_UP for _, r in reach)


def check_dimensions():
    probs = []
    w, h = COLS * CELL, ROWS * CELL
    print(f"  栅格: {COLS} × {ROWS} = {COLS * ROWS} 格")
    print(f"  字母轴 A..N : {COLS} × {CELL} = {w:.2f} m")
    print(f"  数字轴 1..11: {ROWS} × {CELL} = {h:.2f} m")
    if abs(w - 4.9) > 0.05 or abs(h - 3.85) > 0.05:
        probs.append(f"尺寸异常 {w:.2f} × {h:.2f}")
    return probs


def check_line_extent():
    probs = []
    print(f"  线位置: row{BARRIER_UP + 1} 与 row{BARRIER_LO + 1} 之间")
    print(f"  线覆盖列: {col_name(BARRIER_C0)}..{col_name(BARRIER_C1)}"
          f"（共 {BARRIER_C1 - BARRIER_C0 + 1} 列）")
    free_cols = [col_name(c) for c in range(COLS) if not on_line(c)]
    print(f"  线不覆盖: {', '.join(free_cols)}")
    print(f"  → {col_name(BARRIER_C0)}{BARRIER_UP + 1}–"
          f"{col_name(BARRIER_C1)}{BARRIER_UP + 1} 与 "
          f"{col_name(BARRIER_C0)}{BARRIER_LO + 1}–"
          f"{col_name(BARRIER_C1)}{BARRIER_LO + 1} 之间有线，"
          f"但这些格子可通行")
    if BARRIER_C1 != 12:
        probs.append("线右端应为 M（下标 12）")
    return probs


def check_bridges():
    probs = []
    n_a = BRIDGE_A[1] - BRIDGE_A[0] + 1
    n_b = BRIDGE_B[1] - BRIDGE_B[0] + 1
    print(f"  桥洞一 跨 K1–K6 : {n_a} 格")
    print(f"  桥洞二 跨 K7–K11: {n_b} 格")
    print("  合法跨越边只有一条：K6→K7（线的位置），两种桥洞共用这一个缺口")
    print("  → 「6 选 1 / 5 选 1」的含义需场地确认，见下方问题清单")
    if (n_a, n_b) != (6, 5):
        probs.append(f"桥洞格数 ({n_a},{n_b}) 与预期 (6,5) 不符")
    return probs


def check_semantics():
    """对比两种语义，并量化「穿门」与「绕行」的代价差

    注意：线只到 M 列，所以 N 列本来就是合法绕行通路。
    因此正确的判据不是"门全关时能否跨线"，而是：
      穿门代价 vs 绕行代价 —— 差多少格。
    """
    probs = []
    print("  线只覆盖 D..M，因此 N 列与 A/B/C 列本来就是合法绕行通路。")
    print("  正确的判据是代价差：\n")

    for sem in ('A', 'B'):
        g_none = Grid(sem)
        # 门全关：从 K6 上方到 K7 下方
        p_detour = g_none.bfs((BRIDGE_COL, BARRIER_UP), (BRIDGE_COL, BARRIER_LO))

        g_gate = Grid(sem)
        g_gate.open_gate(BARRIER_UP)
        p_gate = g_gate.bfs((BRIDGE_COL, BARRIER_UP), (BRIDGE_COL, BARRIER_LO))

        desc = '线只禁纵向跨越' if sem == 'A' else '线是真正隔离带'
        print(f"  [语义 {sem}] {desc}")
        if p_gate:
            print(f"    穿门 (K6→K7)          : {len(p_gate)} 格")
        else:
            print(f"    穿门 (K6→K7)          : 不可达")
        if p_detour:
            print(f"    门全关时绕行          : {len(p_detour)} 格")
            if p_detour and p_gate:
                print(f"    → 穿门省 {len(p_detour) - len(p_gate)} 格")
            # 绕行经过哪些列
            cols = sorted({c for c, _ in p_detour})
            print(f"    → 绕行经过列: {', '.join(col_name(c) for c in cols)}")
        else:
            print(f"    门全关时绕行          : 不可达（门是唯一通路）")
        print()

    # 语义 B 下，绕行必须经过线未覆盖的列（A/B/C 或 N）
    gb = Grid('B')
    p = gb.bfs((BRIDGE_COL, BARRIER_UP), (BRIDGE_COL, BARRIER_LO))
    if p:
        used = {c for c, _ in p}
        if not (used & {0, 1, 2, 13}):
            probs.append("语义 B 的绕行未经过线未覆盖的列 —— 模型有误")
        else:
            print(f"  [检查] 语义 B 绕行经过未覆盖列: "
                  f"{', '.join(col_name(c) for c in sorted(used & {0, 1, 2, 13}))} ✓")
    else:
        print("  [检查] 语义 B 下门全关时无法绕行（门是唯一通路）")

    print("\n  ★ 结论：两种语义下「门」都只是近路，不是唯一通路；")
    print("    因为规则里的线止于 M 列，N 列天然可绕。")
    print("    请确认：裁判是否认可「从 N 列绕过」也算完成穿越任务？")
    return probs


def check_gate_candidates():
    """逐格测试：把门开在 K 列各位置，能否实现跨线"""
    probs = []
    print("  候选门位测试（语义 B，门必须落在线上才有意义）:")
    ok = []
    for row in range(ROWS - 1):
        g = Grid('B')
        g.open_gate(row)
        r = g.reach((BRIDGE_COL, 0))
        if g.crossed(r):
            ok.append(row)
    print(f"    能实现跨线的门位: {[cname(BRIDGE_COL, r) for r in ok]}")
    if ok != [BARRIER_UP]:
        print(f"    → 只有线的位置 ({cname(BRIDGE_COL, BARRIER_UP)}) 才是有效门位")
    print(f"    桥洞一区间 K1–K6 中有效门位: "
          f"{[cname(BRIDGE_COL, r) for r in ok if r in range(BRIDGE_A[0], BRIDGE_A[1] + 1)]}")
    print(f"    桥洞二区间 K7–K11 中有效门位: "
          f"{[cname(BRIDGE_COL, r) for r in ok if r in range(BRIDGE_B[0], BRIDGE_B[1] + 1)]}")
    return probs


def demo_dump():
    g = Grid('B')
    g.open_gate(BARRIER_UP)
    for name in ('D3', 'G5', 'L2', 'E8', 'J9', 'A4', 'C9', 'M10', 'K2'):
        c, r = ord(name[0]) - 65, int(name[1:]) - 1
        g.cells[r][c] = MINE
    out = ["     " + "".join("%-3s" % col_name(c) for c in range(COLS))]
    for r in range(ROWS):
        row = []
        for c in range(COLS):
            if g.cells[r][c] == MINE:
                row.append("%-3s" % 'm')
            elif c == BRIDGE_COL and r == BARRIER_UP and g.gate_row == BARRIER_UP:
                row.append("%-3s" % 'G')
            else:
                row.append("%-3s" % ' ')
        out.append("%-4s " % (r + 1) + "".join(row))
        if r == BARRIER_UP:
            seg = []
            for c in range(COLS):
                if not on_line(c):
                    seg.append("   ")
                elif c == BRIDGE_COL:
                    seg.append(" G ")     # 门
                else:
                    seg.append("---")
            out.append("     " + "".join(seg) + "  <- 线 (D6–M6 / D7–M7)，G = 桥洞门")
    return "\n".join(out)


def main():
    print("=" * 78)
    print("FieldGrid 场地拓扑校验")
    print("=" * 78)

    checks = [
        ("尺寸", check_dimensions),
        ("不可跨越线的范围", check_line_extent),
        ("桥洞区间", check_bridges),
        ("候选门位", check_gate_candidates),
        ("两种线语义对照", check_semantics),
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

    print("\n[场地示意]")
    print(demo_dump())

    print("=" * 78)
    print("全部校验通过。" if total == 0 else f"{total} 项校验失败。")
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
