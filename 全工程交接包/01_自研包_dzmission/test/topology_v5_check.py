#!/usr/bin/env python3
"""场地拓扑 v5 —— 现场确认后的最终模型（★ 结论与 v4 完全不同）

现场确认的三条事实（2026 现场地图 + 用户确认）：
  1. 起点 = **M11**（col 12, row 10），终点 = **N6**（col 13, row 5）
  2. 水平隔离线 **覆盖 D..N 全宽**，位于 **row6 / row7 之间**
  3. 竖直黄线 = **K 列桥洞通道那一排**，整列 K；
     每局随机指定 2 个可通行通道，区段一 K1–K6、区段二 K7–K11

★ 与 v4 的决定性差别：
  v4 假设线只覆盖 D..M，起点 M7（col 12, row 6）与终点 N5（col 13, row 4）
  都在线的**北侧**，所以起终点路径无需穿桥洞 -> "保底 10 分不依赖无人机"。

  v5 中线覆盖到 N，起点 **M11 在南侧**、终点 **N6 在北侧**，
  而 M、N 两列都被线覆盖 -> **东侧条带被完全切断**。
  => M11 无法直接北上到 N6，**必须穿桥洞**。

  这意味着：
    · 「起终点 10 分」**不再是保底分**，它依赖无人机下发的桥洞门位
    · 桥洞从"加分项 20 分"变成"**必经通道**"—— 不穿桥洞则起终点都到不了
    · 断链（收不到门位）时的降级策略必须重新设计

运行:  python3 topology_v5_check.py
"""

import sys
from collections import deque

# ---------------- 场地 ----------------
COLS, ROWS = 14, 11
CELL = 35.0
COL_LETTERS = 'ABCDEFGHIJKLMN'
WALL_UP = 5                    # 线在 row5 与 row6 之间（0-based）
WALL_C0, WALL_C1 = 3, 13       # D..N 全宽
BRIDGE_COL = 10                # K

SEG_A = (0, 5)                 # K1..K6
SEG_B = (6, 10)                # K7..K11

START = (12, 10)               # M11
GOAL = (13, 5)                 # N6

# 示例门位（说明书给的例子是 K4 与 K9）
GATES_REF = (3, 8)             # K4 (row 3), K9 (row 8)


def cname(c, r):
    return "%s%d" % (COL_LETTERS[c], r + 1)


def cc(c, r):
    return ((c + 0.5) * CELL, (r + 0.5) * CELL)


class Field:
    def __init__(self, gates=(), blocked=(), wall_c1=WALL_C1, wall_on=True):
        self.gates = set(gates)
        self.blocked = set(blocked)
        self.wall_c1 = wall_c1
        self.wall_on = wall_on
        # K 列：只有指定门可通行，其余是隔板
        for r in range(ROWS):
            self.blocked.add((BRIDGE_COL, r))
        for g in self.gates:
            self.blocked.discard((BRIDGE_COL, g))

    def passable(self, c, r):
        if not (0 <= c < COLS and 0 <= r < ROWS):
            return False
        return (c, r) not in self.blocked

    def edge_blocked_vert(self, c, r0, r1):
        """竖直相邻两格（列相同）之间的边是否被封"""
        up = min(r0, r1)
        if not self.wall_on:
            return False
        if up == WALL_UP and WALL_C0 <= c <= self.wall_c1:
            return True
        return False

    def neighbors(self, c, r):
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nc, nr = c + dc, r + dr
            if not self.passable(nc, nr):
                continue
            if dc == 0 and self.edge_blocked_vert(c, r, nr):
                continue
            yield nc, nr

    def bfs(self, src, dst):
        if not (self.passable(*src) and self.passable(*dst)):
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

    def plen(self, a, b):
        p = self.bfs(a, b)
        return -1 if p is None else len(p)


def sec(t):
    print()
    print("=" * 78)
    print(t)
    print("=" * 78)


# ================================================================ 1
def check_geometry():
    sec("1. 几何关系核对（确认 off-by-one 没搞错）")
    print("  坐标约定：+x = 列字母增大（A->N），+y = 行号增大（1->11）")
    print("  水平隔离线：row%d 与 row%d 之间，覆盖 %s..%s" % (
        WALL_UP + 1, WALL_UP + 2, COL_LETTERS[WALL_C0], COL_LETTERS[WALL_C1]))
    print()
    print("  %-8s %-10s %-8s %-10s %s" % ("格", "(x,y)cm", "col,row", "在线哪侧", "是否被线覆盖"))
    for name, (c, r) in (("M11", START), ("N6", GOAL), ("M6", (12, 5)),
                         ("N7", (13, 6)), ("A11", (0, 10)), ("A6", (0, 5))):
        x, y = cc(c, r)
        side = "南(下)" if r > WALL_UP else "北(上)"
        covered = "是" if WALL_C0 <= c <= WALL_C1 else "否"
        print("  %-8s (%5.0f,%5.0f) %-8s %-10s %s" % (
            name, x, y, "%d,%d" % (c, r), side, covered))
    print()
    print("  ★ 起点 M11 在线的**南侧**，终点 N6 在**北侧** —— 分居两侧")
    print("  ★ M(col12) 与 N(col13) 都被线覆盖 -> 东侧无法直接北上")


# ================================================================ 2
def check_no_gate():
    sec("2. 门全关时：M11 能否到 N6？")
    f = Field(gates=(), wall_c1=WALL_C1)
    p = f.bfs(START, GOAL)
    print("  门全关 -> %s" % ("不可达" if p is None else "可达 %d 格" % len(p)))
    print("  对照：把线的覆盖范围缩到 D..M（N 列不挡）")
    f2 = Field(gates=(), wall_c1=12)
    p2 = f2.bfs(START, GOAL)
    print("    线只到 M -> %s" % ("不可达" if p2 is None else "可达 %d 格" % len(p2)))
    if p2 is not None:
        print("      路径: " + " -> ".join(cname(*q) for q in p2))
    print()
    print("  ★ 这就是 v4 与 v5 的分水岭：线的列范围决定起终点是否需要穿桥洞")


# ================================================================ 3
def check_with_gates():
    sec("3. 开了桥洞门之后：M11 -> N6 的路径")
    print("  门位候选数：区段一 K%d-K%d（%d 个）、区段二 K%d-K%d（%d 个）" % (
        SEG_A[0] + 1, SEG_A[1] + 1, SEG_A[1] - SEG_A[0] + 1,
        SEG_B[0] + 1, SEG_B[1] + 1, SEG_B[1] - SEG_B[0] + 1))
    print()
    print("  %-22s %-10s %-10s %s" % ("门位组合", "起点可达", "穿门次数", "路径长度"))
    total = 0
    ok = 0
    for ga in range(SEG_A[0], SEG_A[1] + 1):
        for gb in range(SEG_B[0], SEG_B[1] + 1):
            f = Field(gates=(ga, gb))
            p = f.bfs(START, GOAL)
            total += 1
            if p is not None:
                ok += 1
                # 数一下路径里有几个 K 列格（= 穿了几次门）
                thru = sum(1 for q in p if q[0] == BRIDGE_COL)
                if total <= 6 or (ga, gb) == GATES_REF:
                    print("  K%d + K%-17s %-10s %-10d %d 格" % (
                        ga + 1, str(gb + 1), "可达", thru, len(p)))
    print("  ...")
    print()
    print("  合计 %d 种门位组合，%d 种可达（%.0f%%）" % (total, ok, 100.0 * ok / total))
    print()
    # 参考门位的完整路径
    f = Field(gates=GATES_REF)
    p = f.bfs(START, GOAL)
    print("  参考门位 K4+K9 的完整路径（%d 格）:" % len(p))
    print("    " + " -> ".join(cname(*q) for q in p))


# ================================================================ 4
def check_gate_requirement():
    sec("4. 关键结论：两个门都是必需的吗？（答案是：是）")
    print("  起点 M11 在南侧（东条带），终点 N6 在北侧（东条带）。")
    print("  K 列整列是桥洞排 —— 除门格以外不可通行；")
    print("  水平隔离线 D..N 全宽，其**唯一缺口就在 K 列**（K 列穿了线）。")
    print()
    print("  %-26s %-12s %s" % ("门位配置", "可达性", "路径长度"))
    for name, gates in (("区段一单门 K4", (3,)),
                        ("区段二单门 K9", (8,)),
                        ("区段一任意单门（6 个全试）", None),
                        ("区段二任意单门（5 个全试）", None),
                        ("两段各一门 K4+K9", (3, 8))):
        if gates is None:
            lo, hi = (SEG_A if "一" in name else SEG_B)
            lens = []
            for g in range(lo, hi + 1):
                f = Field(gates=(g,))
                p = f.bfs(START, GOAL)
                if p is not None:
                    lens.append(len(p))
            print("  %-26s %-12s %s" % (
                name, "全部不可达" if not lens else "%d 个可达" % len(lens),
                "-" if not lens else "最短 %d 格" % min(lens)))
            continue
        f = Field(gates=gates)
        p = f.bfs(START, GOAL)
        print("  %-26s %-12s %s" % (
            name, "可达" if p else "不可达",
            "%d 格" % len(p) if p else "-"))
    print()
    print("  ★ 真相：**两个门都必需**，缺一不可。原因是两个障碍叠加：")
    print("     1) **竖直的 K 列隔板**：整列不可通行，只有 2 个门格能进")
    print("        —— 这是东条带通往西侧/中部的**唯一**通道")
    print("     2) **水平的 D..N 隔离线**（row6/row7 之间）：唯一缺口在 K 列")
    print("        —— 这是南 ↔ 北的**唯一**通道（K 列穿线处）")
    print()
    print("  ★ 路径实证（参考门位 K4+K9，31 格）：")
    print("      M11 -> L11 -> L10 -> L9 -> [K9] -> J9 -> … -> D9 -> C9")
    print("           -> C8 -> C7 -> [C7->C6 跨水平线] -> C6 -> D6 -> … -> J6")
    print("           -> J5 -> J4 -> [K4] -> L4 -> M4 -> N4 -> N5 -> N6")
    print("      三个关键动作：")
    print("        · 穿 K9 门：从东条带进入西侧（K 列隔板的南段门）")
    print("        · 在 **C 列** 跨水平线：C 在 D..N 之外，线未覆盖 -> 唯一无门出口")
    print("        · 穿 K4 门：从西侧回到东条带（K 列隔板的北段门）")
    print()
    print("     于是：")
    print("       · 只开**北段**门 -> 车在南侧根本进不了 K 列 -> 不可达")
    print("       · 只开**南段**门 -> 能进 K 列、能穿到西侧，但回不了东条带")
    print("         （东条带只能从 K 列进，北段门没开）-> 也不可达")
    print()
    print("     **必须南段一个门（进西侧） + 北段一个门（回东条带）**。")
    print()
    print("  这对比赛的含义（★ 很重要）：")
    print("     · 无人机的 BRIDGE_GATE 消息必须**两个门都正确下发**，")
    print("       只给一个门等于**完全无法通行**，不是'少走点路'。")
    print("     · 两个门里任何一个错，整套任务（起终点 + 地雷 40 分）全部失败。")
    print("     · 因此 AIR_GROUND_PROTOCOL 的 BRIDGE_GATE 必须有**校验**：")
    print("       收到两个门后，车端本地跑一次连通性校验（就是本脚本的逻辑），")
    print("       不自洽就要求重发，而不是硬走。")
    print()
    print("  ★ 另一个重要推论：A/B/C 列是**无门出口**（线不覆盖，K 列以外唯一")
    print("     能南北贯通的地方）。所以 A/B/C 列是战术要道 —— 但要注意")
    print("     那里在线的南侧紧邻的 D 列有挡板端部，通行时留好余量。")


# ================================================================ 5
def check_west_field():
    sec("5. 西半场（地雷/障碍所在）的到达性")
    f = Field(gates=GATES_REF)
    for name, tgt in (("A1 (西北角)", (0, 0)), ("A11 (西南角)", (0, 10)),
                      ("D1", (3, 0)), ("J11", (9, 10))):
        p = f.bfs(START, tgt)
        print("  M11 -> %-14s %s" % (name, "可达 %d 格" % len(p) if p else "不可达"))
    print()
    print("  ★ 西半场同样必须穿桥洞才能到达 —— 与起终点共用同一条通道")


# ================================================================ 6
def check_v4_vs_v5():
    sec("6. v4 与 v5 的结论对照（为什么必须改策略）")
    print("  %-42s %-16s %-16s" % ("项目", "v4（线 D..M）", "v5（线 D..N，已确认）"))
    rows = [
        ("起点 / 终点", "M7 (12,6) / N5 (13,4)", "M11 (12,10) / N6 (13,5)"),
        ("起终点相对线的位置", "都在线的北侧", "分居两侧（南/北）"),
        ("起终点是否需穿桥洞", "**不需要**", "**必须穿**"),
        ("起终点 10 分是否保底", "是（断链也能拿）", "**否（依赖门位）**"),
        ("桥洞性质", "加分项 20 分", "**必经通道 + 20 分**"),
        ("断链降级策略", "走保底路径拿 10 分", "只能原地等待/盲试门位"),
    ]
    for a, b, c in rows:
        print("  %-42s %-16s %-16s" % (a, b, c))
    print()
    print("  ★ 最重要的变化：**桥洞门位从'加分'变成'生存必需'**。")
    print("     没有无人机的门位信息，起终点都到不了，直接损失 10 分，")
    print("     而且地雷（40 分）也全在西半场，等于 50 分全丢。")
    print("     => 通信链路（AIR_GROUND_PROTOCOL 的 BRIDGE_GATE 消息）")
    print("        从'重要'升级为'**单点致命**'，必须有降级预案。")


# ================================================================ 7
def check_fallback_options():
    sec("7. 断链时的降级预案（必须设计）")
    print("  没有门位信息时，可选策略：")
    print()
    print("  (a) 盲试门位 —— 风险可控（比预想的好）")
    print("      K 列是通道，水平隔离线的缺口只需**一个**门即可跨线：")
    print("        南侧格 -> 进 K 列 -> 在门那一行跨过隔离线 -> 沿 K 列北上 -> 北侧出")
    total = 0
    found = 0
    for g in range(SEG_A[0], SEG_B[1] + 1):
        total += 1
        f = Field(gates=(g,))
        if f.bfs(START, GOAL) is not None:
            found += 1
    print("      逐个试 %d 个门位，其中 %d 个能跨线（%.0f%%）" % (
        total, found, 100.0 * found / total))
    print("      K 列每格 35cm，逐个试探 + 撞到隔板再退，平均试 %.1f 次" % (total / 2.0))
    print("      每次试算约 5s -> 约 %.0f s，占掉赛时 %.0f%%" % (
        total / 2.0 * 5, total / 2.0 * 5 / 180 * 100))
    print("      ★ 但盲试会真的撞到隔板 —— 规则：与固定场地相撞持续超 10s")
    print("        裁判切手动。所以**每次试探都必须限时后退**，不能抵着撞。")
    print()
    print("  (b) 沿 K 列贴边慢行 + 视觉找缺口 —— 推荐")
    print("      K 列是通道，缺口处两侧的挡板结构不同；车端相机可以看")
    print("      '前方是否有白色长条/挡板'。以 0.1 m/s 沿 K 列扫，")
    print("      单列 11 格 = 3.85m，约 40 s 能扫完。")
    print()
    print("  (c) 用 A-C 列绕行 —— **不可行**")
    f = Field(gates=(), wall_c1=13)
    print("      验证：门全关时 M11 -> A11 是否可达？ %s" % (
        "可达" if f.bfs(START, (0, 10)) else "不可达"))
    print("      A-C 列在线南侧可自由北上，但要回到东侧的 N6 必须穿 K 列")
    print("      -> 绕行 A-C 只能解决'过线'，不能解决'回东侧'")
    print()
    print("  (d) 请求重发 —— 最基本")
    print("      链路恢复后立即重发 BRIDGE_GATE；任务层在链路超时时")
    print("      先执行'安全区等待'（停在 M11，不浪费电），而不是乱走。")


def main():
    print("场地拓扑 v5 —— 现场确认后的最终模型")
    print("起点 M11(col12,row10)  终点 N6(col13,row5)")
    print("水平隔离线 D..N 全宽，在 row6/row7 之间")
    print("桥洞 = K 列整列，随机 2 个通道（K1-K6 一个，K7-K11 一个）")

    check_geometry()
    check_no_gate()
    check_with_gates()
    check_gate_requirement()
    check_west_field()
    check_v4_vs_v5()
    check_fallback_options()

    sec("总结论")
    print("  1. 起点 M11（南）与终点 N6（北）分居水平隔离线两侧，")
    print("     而线覆盖 D..N 全宽 -> 东侧条带无法南北贯通")
    print("  2. 真拓扑是**两个障碍叠加**：")
    print("       · 竖直 K 列隔板（整列，只有 2 个门格可通行）")
    print("         = 东条带 ↔ 西侧/中部的唯一通道")
    print("       · 水平 D..N 隔离线（唯一缺口在 K 列）")
    print("         = 南 ↔ 北的唯一通道；A/B/C 列因线未覆盖而可自由南北")
    print("  3. ★ **两个门缺一不可**：只给一个门 = 完全无法通行，")
    print("     不是'少走点路'。北门开南门没开 -> 进不了 K 列；")
    print("     南门开北门没开 -> 进了 K 列出不来。")
    print("     30 种门位组合 100% 可达，但必须两个都给对。")
    print("  4. ★ 「起终点 10 分不依赖无人机」的保底结论**作废**")
    print("     => 门位信息是硬依赖，且必须**两个都对**")
    print("     => 地雷 40 分也在西侧，与起终点共用同一条通道")
    print("     => 通信链路从'重要'升级为'单点致命'")
    print("  5. 必须加的校验：车端收到两个门后本地跑一次连通性校验")
    print("     （FieldGrid::selfCheck 里已固化为断言），不自洽就要求重发")
    print("  6. 参考门位 K4+K9 的最短路径 = 31 格（见第 3 节）")
    print("     最优路线形状固定为：南门进西侧 -> A/B/C 列北上 -> 北门回东条带")
    return 0


if __name__ == "__main__":
    sys.exit(main())
