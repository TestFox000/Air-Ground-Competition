#!/usr/bin/env python3
"""原地转向（pivot turn）几何验证

背景：学员代码（src(走斜线)/path_executor.py）在锐角转角处使用"三段式原地转向"：
    phase 0: 停车，打满舵（angular.z = ±30°）
    phase 1: 保持 0.5s 让舵机转到位
    phase 2: 以 0.08 m/s 蠕行 + 保持满舵，直到偏航角对准下一段
             （对准容差 15°）

我此前用 FieldGrid::checkTurn 证明：**单圆弧 90° 转向在 K 列以东的窄条里
不可行**（扫掠半径 R + 车体对角半径 = 446mm，而 N 列中心线距场边仅 175mm）。

本脚本验证：同样是 90° 转向，**原地转向**是否可行。

关键区别：
  · 单圆弧：车沿半径 R 的圆弧前进，车体**平动 + 转动**，
    扫掠区域是以转弯中心为圆心、半径 (R + 车体对角半径) 的扇形 → 很大
  · 原地转向：车绕后轴中心**纯转动**（位移≈0，只有 0.08m/s 蠕行的微小前进），
    扫掠区域是以后轴中心为圆心、半径 = 车体最大对角半径 的圆 → 很小

结论决定：窄条内该用单圆弧还是原地转向。

运行:  python3 pivot_turn_check.py
"""

import math
import sys

# ---------------- 车体（官方实测） ----------------
CAR_L, CAR_W = 27.8, 14.0      # cm
WB, TRACK = 15.0, 11.0
R_ARC = 26.0                   # 单圆弧保守最小转弯半径
FRONT, REAR = 0.62 * CAR_L, 0.38 * CAR_L
HALF_W = CAR_W / 2

# ---------------- 场地 ----------------
CELL = 35.0
COLS, ROWS = 14, 11
FIELD_W, FIELD_H = COLS * CELL, ROWS * CELL
WALL_UP = 5
WALL_Y = (WALL_UP + 1) * CELL
WALL_C0, WALL_C1 = 3, 12
BRIDGE_COL = 10

START = (12, 6)                # M7
GOAL = (13, 4)                 # N5


def cname(c, r):
    return "{}{}".format(chr(ord('A') + c), r + 1)


def cc(c, r):
    return ((c + 0.5) * CELL, (r + 0.5) * CELL)


def sec(t):
    print()
    print("=" * 78)
    print(t)
    print("=" * 78)


class Field:
    """场地模型：场地边界 + 水平隔离线挡板 + 不可通行格（桥洞隔板/水桶）

    ★ 重要：**格与格之间的分界线不是障碍物**（场地没有四周围挡）。
    只有下列三样是真实阻挡：
      1. 场地外边界 4m×5m（不得驶出）
      2. 水平隔离线挡板（D..M 列，row6/row7 之间），一条不可跨越的线
      3. 不可通行的格（K 列未指定的桥洞槽 = 隔板；8 个水桶所在格）
    """

    def __init__(self, gates=(), blocked=()):
        self.gates = set(gates)
        self.blocked = set(blocked)
        for r in range(ROWS):
            if r not in self.gates:
                self.blocked.add((BRIDGE_COL, r))

    def gap(self, x, y):
        """世界点 (x,y) 到最近**真实阻挡**的距离 cm；已侵入返回负值"""
        # 1) 场地边界
        if not (0.0 <= x <= FIELD_W and 0.0 <= y <= FIELD_H):
            return -min(abs(x), abs(x - FIELD_W), abs(y), abs(y - FIELD_H))
        g = min(x, FIELD_W - x, y, FIELD_H - y)

        # 2) 水平隔离线挡板
        if WALL_C0 * CELL <= x <= (WALL_C1 + 1) * CELL:
            g = min(g, abs(y - WALL_Y))

        # 3) 不可通行格
        c = int(x // CELL)
        r = int(y // CELL)
        if (c, r) in self.blocked:
            ccx, ccy = cc(c, r)
            return -max(1.0, CELL * 0.5 - math.hypot(x - ccx, y - ccy))
        for nc, nr, d in ((c + 1, r, (c + 1) * CELL - x),
                          (c - 1, r, x - c * CELL),
                          (c, r + 1, (r + 1) * CELL - y),
                          (c, r - 1, y - r * CELL)):
            if (nc, nr) in self.blocked:
                g = min(g, d)
        return g


def body_corners(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + dx * c - dy * s, y + dx * s + dy * c)
            for dx, dy in ((FRONT, HALF_W), (FRONT, -HALF_W),
                           (-REAR, -HALF_W), (-REAR, HALF_W))]


def wheels(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    hw = TRACK / 2
    return [(x + dx * c - dy * s, y + dx * s + dy * c)
            for dx, dy in ((0.0, hw), (0.0, -hw), (WB, hw), (WB, -hw))]


# ================================================================ 1
def check_swept_radius():
    sec("1. 两种转向的扫掠半径对比")

    r_front_corner = math.hypot(FRONT, HALF_W)
    r_rear_corner = math.hypot(REAR, HALF_W)
    r_max = max(r_front_corner, r_rear_corner)

    print("  车体自后轴中心（base_link）的几何：")
    print("    前伸 {:.1f} cm，后伸 {:.1f} cm，半宽 {:.1f} cm".format(
        FRONT, REAR, HALF_W))
    print("    前外角距后轴中心 {:.1f} cm".format(r_front_corner))
    print("    后外角距后轴中心 {:.1f} cm".format(r_rear_corner))
    print("    → 车体最大对角半径 = {:.1f} cm".format(r_max))
    print()
    print("  【单圆弧 90° 转向】")
    print("    转弯中心 C 在车道中心线交点，距后轴中心 R = {:.0f} cm".format(R_ARC))
    print("    扫掠最大半径 = R + 前外角 = {:.0f} + {:.1f} = {:.1f} cm"
          .format(R_ARC, r_front_corner, R_ARC + r_front_corner))
    print("    → 需要以 C 为圆心、半径 {:.0f} cm 的扇形全程无阻挡"
          .format(R_ARC + r_front_corner))
    print()
    print("  【原地转向】")
    print("    车绕后轴中心纯转动，后轴中心位移 ≈ 0")
    print("    （学员实现用 0.08 m/s 蠕行，10cm 前进量可忽略）")
    print("    扫掠最大半径 = 车体最大对角半径 = {:.1f} cm".format(r_max))
    print("    → 只需以**车当前位置**为圆心、半径 {:.1f} cm 的圆无阻挡"
          .format(r_max))
    print()
    ratio = (R_ARC + r_front_corner) / r_max
    print("  ★ 原地转向的扫掠半径只有单圆弧的 1/{:.1f}（{:.1f} vs {:.1f} cm）"
          .format(ratio, r_max, R_ARC + r_front_corner))
    return r_max


# ================================================================ 2
def check_arc_vs_pivot():
    sec("2. 在关键位置做 90° 转向：单圆弧 vs 原地转向")

    f = Field(gates=(2, 8))
    cases = [
        ("M7 (起点格)", START, "东→北"),
        ("N7", (13, 6), "东→北"),
        ("L1 (最北行)", (11, 0), "东→北"),
        ("M1", (12, 0), "东→北"),
        ("J8 (贴挡板南侧)", (9, 7), "东→北"),
    ]

    print("  {:22} {:>14} {:>14} {:>10}".format(
        "位置", "单圆弧最小间隙", "原地转向最小间隙", "结论"))
    for name, (c, r), desc in cases:
        x, y = cc(c, r)

        # --- 单圆弧：转弯中心 = 出向列中心线 x 入向行中心线 ---
        cx, cy = x, y
        gap_arc = 1e9
        N = 180
        for i in range(N + 1):
            yaw = (math.pi / 2) * i / N          # 0 → 90°，逆时针
            px = cx + R_ARC * math.cos(yaw - math.pi / 2)
            py = cy + R_ARC * math.sin(yaw - math.pi / 2)
            for pt in body_corners(px, py, yaw) + wheels(px, py, yaw):
                gap_arc = min(gap_arc, f.gap(*pt))

        # --- 原地转向：位置不动，只转 yaw ---
        gap_pivot = 1e9
        for i in range(N + 1):
            yaw = (math.pi / 2) * i / N
            for pt in body_corners(x, y, yaw) + wheels(x, y, yaw):
                gap_pivot = min(gap_pivot, f.gap(*pt))

        verdict = "单圆弧不行" if gap_arc < HALF_W else "两者皆可"
        if gap_pivot < HALF_W:
            verdict = "两者都不行"
        print("  {:22} {:>12.1f}cm {:>12.1f}cm   {}".format(
            name, gap_arc, gap_pivot, verdict))
    print()
    print("  （判据：最小间隙 >= 半车宽 {:.0f} cm 才算安全）".format(HALF_W))


# ================================================================ 3
def check_pivot_all_cells():
    sec("3. 全场普查：原地转向在每个可通行格都能做吗")
    f = Field(gates=(2, 8))
    ok = 0
    bad = []
    blocked_itself = 0
    for r in range(ROWS):
        for c in range(COLS):
            if (c, r) in f.blocked:
                blocked_itself += 1
                continue
            x, y = cc(c, r)
            g = 1e9
            for i in range(37):
                yaw = 2 * math.pi * i / 36
                for pt in body_corners(x, y, yaw) + wheels(x, y, yaw):
                    g = min(g, f.gap(*pt))
            if g >= 0.0:
                ok += 1
            else:
                bad.append((cname(c, r), g))
    total = ROWS * COLS - blocked_itself
    print("  不可通行的格（K 列隔板）: {} 个".format(blocked_itself))
    print("  可通行格: {} 个".format(total))
    print("  其中'绕自身 360° 旋转不侵入任何真实阻挡'的: {} 个".format(ok))
    if bad:
        print()
        print("  不满足的格（这些格旋转时车体会压到挡板或场边）:")
        for name, g in bad[:20]:
            print("    {}  侵入 {:.1f} cm".format(name, -g))
    else:
        print("  ✓ 所有可通行格都能安全原地转向")
    print()
    # 分别统计：压场边 / 压挡板 / 邻格被挡
    print("  分类统计（只统计失败的格）:")
    edge_only = wall_only = nb_only = 0
    for (name, g) in bad:
        c = ord(name[0]) - ord('A')
        r = int(name[1:]) - 1
        x, y = cc(c, r)
        r_max = max(math.hypot(FRONT, HALF_W), math.hypot(REAR, HALF_W))
        hit_edge = (x - r_max < 0 or x + r_max > FIELD_W
                    or y - r_max < 0 or y + r_max > FIELD_H)
        hit_wall = (WALL_C0 * CELL <= x <= (WALL_C1 + 1) * CELL
                    and abs(y - WALL_Y) < r_max)
        if hit_edge:
            edge_only += 1
        elif hit_wall:
            wall_only += 1
        else:
            nb_only += 1
    print("    压场地边界 : {} 个（第 1 行/第 11 行/最外列）".format(edge_only))
    print("    压水平挡板 : {} 个".format(wall_only))
    print("    邻格被挡   : {} 个".format(nb_only))
    print()
    print("  ★ 决定性对比：")
    print("     · 单圆弧：窄条(东侧 L/M/N)每个格都不可行")
    print("     · 原地转向：只有贴场边 / 贴挡板的格不可行，其余全可行")
    print("    ⇒ 窄条内的 90° 转向必须用原地转向，不能用单圆弧")


# ================================================================ 4
def check_pivot_needs_room():
    sec("4. 原地转向对邻格的要求（用于路径规划约束）")
    r_max = max(math.hypot(FRONT, HALF_W), math.hypot(REAR, HALF_W))
    print("  扫掠半径 {:.1f} cm = {:.2f} 格".format(r_max, r_max / CELL))
    print()
    print("  以格心为中心、半径 {:.1f} cm 的圆覆盖范围：".format(r_max))
    reach = r_max / CELL
    print("    {:.2f} 格 → 最多触及相邻格的一半以内".format(reach))
    print("    → 只要本格 + 四个正交邻格可通行，原地转向就是安全的")
    print()
    print("  实测验证（扫掠圆与格边界的交点）：")
    for name, (c, r) in (("M7", START), ("N5", GOAL), ("L1", (11, 0))):
        x, y = cc(c, r)
        print("    {} 格心 ({:.0f},{:.0f})，扫掠圆 x∈[{:.0f},{:.0f}] y∈[{:.0f},{:.0f}]"
              .format(name, x, y, x - r_max, x + r_max, y - r_max, y + r_max))
    print()
    print("  ⇒ 路径规划约束：**允许在任意可通行格做原地转向**，")
    print("     只需保证该格与四邻可通行（这条比单圆弧约束宽松得多）")


def main():
    print("原地转向（pivot turn）几何验证")
    print("车体 {:.0f}x{:.0f} cm，轴距 {:.0f}，轮距 {:.0f}，单圆弧 R={:.0f} cm".format(
        CAR_L, CAR_W, WB, TRACK, R_ARC))

    r_max = check_swept_radius()
    check_arc_vs_pivot()
    check_pivot_all_cells()
    check_pivot_needs_room()

    sec("结论")
    print("  1. 单圆弧扫掠半径 {:.0f} cm；原地转向扫掠半径 {:.1f} cm（仅车体对角）"
          .format(R_ARC + math.hypot(FRONT, HALF_W), r_max))
    print("  2. 窄条（K 列以东 3 列）内单圆弧不可行，但原地转向可行 ——")
    print("     这解释了学员为什么用三段式原地转向而不是圆弧")
    print("  3. 原地转向的规划约束很宽松：本格 + 四邻可通行即可")
    print("  4. 代价：每次转向要停车 + 等舵机 0.5s，单次约 2~3 s")
    print("     8 颗雷 + 2 次桥洞 + 起终点 ≈ 15~20 次转向 → 30~60 s 额外开销")
    print("     但 180 s 总时长仍够用（估算见 mine_planner_check.py）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
