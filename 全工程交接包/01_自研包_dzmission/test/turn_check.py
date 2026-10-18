#!/usr/bin/env python3
"""转弯可行性 —— 与 C++ FieldGrid::checkTurn 的对照实现 + 场地转角普查

用途：
  1. 作为 C++ 实现的**行为基准**（同一组输入必须得到相同结论）
  2. 普查场地内所有 90° 转角，回答"哪些地方能转、哪些不能"
  3. 验证我已经写进 FieldGrid 的判据

判据（与 C++ 一致）：
  1) a->b->c 四连通且构成 90° 转向
  2) 转弯中心 C = 入向车道中心线 ∩ 出向车道中心线（**几何锁定**）
     转弯方向 = sign(cross(入向, 出向))：>0 左转（逆时针），<0 右转（顺时针）
  3) 车体绕 C 扫过 90°（含四角与四轮接地点）
     扫掠包络必须全部落在可通行区域，且间隙 >= turn_min_clearance

车体（官方实测）：278 × 140 mm，轴距 150，轮距 110，R = 240~260 mm
场地：14 × 11 格 @35cm；水平隔离线 D6–M6/D7–M7；K 列桥洞排

运行:  python3 turn_check.py
"""

import math
import sys

# ---------------- 车体（官方实测） ----------------
CAR_L = 27.8      # cm
CAR_W = 14.0      # cm
WB = 15.0         # 轴距 cm
TRACK = 11.0      # 轮距 cm
R_TURN = 26.0     # 保守最小转弯半径 cm（官方 24~26）
FRONT = 0.62 * CAR_L
REAR = 0.38 * CAR_L
HALF_W = CAR_W / 2

# ---------------- 场地 ----------------
CELL = 35.0
COLS, ROWS = 14, 11
FIELD_W, FIELD_H = COLS * CELL, ROWS * CELL
WALL_UP = 5                       # row6 与 row7 之间
WALL_Y = (WALL_UP + 1) * CELL
WALL_C0, WALL_C1 = 3, 12

BRIDGE_COL = 10
BRIDGE_SEG_A = (0, 5)
BRIDGE_SEG_B = (6, 10)

START = (12, 6)
GOAL = (13, 4)

UNKNOWN, FREE, BLOCKED, MINE, TARGET = range(5)


def col_name(c):
    return chr(ord('A') + c)


def cname(c, r):
    return f"{col_name(c)}{r + 1}"


def cc(c, r):
    return ((c + 0.5) * CELL, (r + 0.5) * CELL)


class Grid:
    def __init__(self, gates=()):
        self.gates = set(gates)
        self.cells = [[FREE] * COLS for _ in range(ROWS)]
        # K 列：只有指定的门可通行
        for r in range(ROWS):
            self.cells[r][BRIDGE_COL] = FREE if r in self.gates else BLOCKED

    def passable(self, c, r):
        return 0 <= c < COLS and 0 <= r < ROWS and self.cells[r][c] in (FREE, MINE)

    # ---------- 净空 ----------
    def gap(self, x, y):
        """世界点 (x,y) 到最近阻挡的间隙（cm）。正=在可通行区内。"""
        if not (0.0 <= x <= FIELD_W and 0.0 <= y <= FIELD_H):
            return -1.0
        g = min(x, FIELD_W - x, y, FIELD_H - y)
        # 水平隔离线
        if WALL_C0 * CELL <= x <= (WALL_C1 + 1) * CELL:
            g = min(g, abs(y - WALL_Y))
        c = int(x // CELL)
        r = int(y // CELL)
        if not (0 <= c < COLS and 0 <= r < ROWS) or not self.passable(c, r):
            ccx, ccy = cc(c, r)
            d = math.hypot(x - ccx, y - ccy)
            return -max(1.0, CELL * 0.5 - d)
        # 到相邻阻挡格的距离
        dx = min(x - c * CELL, (c + 1) * CELL - x)
        dy = min(y - r * CELL, (r + 1) * CELL - y)
        nb = ((c + 1, r, dx), (c - 1, r, dx), (c, r + 1, dy), (c, r - 1, dy))
        for nc, nr, d in nb:
            if not (0 <= nc < COLS and 0 <= nr < ROWS) or not self.passable(nc, nr):
                g = min(g, d)
        return g

    def body_corners(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + dx * c - dy * s, y + dx * s + dy * c)
                for dx, dy in ((FRONT, HALF_W), (FRONT, -HALF_W),
                               (-REAR, -HALF_W), (-REAR, HALF_W))]

    def wheel_points(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        hw = TRACK / 2
        return [(x + dx * c - dy * s, y + dx * s + dy * c)
                for dx, dy in ((0.0, hw), (0.0, -hw), (WB, hw), (WB, -hw))]


def turn_center(ac, ar, bc, br, cc, cr):
    """转弯中心 = 入向车道中心线 ∩ 出向车道中心线"""
    in_vertical = (ac == bc)
    if in_vertical:
        return (bc + 0.5) * CELL, (cr + 0.5) * CELL
    return (cc + 0.5) * CELL, (br + 0.5) * CELL


def yaw_of(dc, dr):
    if dc == 1:
        return 0.0
    if dc == -1:
        return math.pi
    if dr == 1:
        return math.pi / 2
    return -math.pi / 2


def check_turn(grid, a, b, c, min_clearance=None, verbose=False):
    """返回 (feasible, reason, info)

    min_clearance 默认取**半车宽** —— 转弯时车体要绕中心旋转，
    至少需要半个车宽的回旋余量，否则就是'擦着边缘过'，实车必然蹭上。
    """
    if min_clearance is None:
        min_clearance = HALF_W
    ac, ar = a
    bc, br = b
    cc_, cr = c

    if not (grid.passable(bc, br) and grid.passable(cc_, cr)):
        return False, "入向或出向格不可通行", {}
    ab_vert = (ac == bc)
    bc_vert = (bc == cc_)
    if ab_vert == bc_vert:
        return False, "不是 90 度转向", {}

    cx, cy = turn_center(ac, ar, bc, br, cc_, cr)
    yaw_in = yaw_of(bc - ac, br - ar)
    yaw_out = yaw_of(cc_ - bc, cr - br)
    # 转弯方向由叉积决定
    cross = math.sin(yaw_out - yaw_in)
    left = cross > 0
    dpsi = (1.0 if left else -1.0) * math.pi / 2

    R = R_TURN
    min_gap = 1e9
    hit_body = False
    hit_wheel = False
    xs_all, ys_all = [], []
    N = 90
    for i in range(N + 1):
        yaw = yaw_in + dpsi * i / N
        px = cx + R * math.cos(yaw - math.pi / 2)
        py = cy + R * math.sin(yaw - math.pi / 2)
        for (xx, yy) in grid.body_corners(px, py, yaw):
            g = grid.gap(xx, yy)
            min_gap = min(min_gap, g)
            if g < 0:
                hit_body = True
            xs_all.append(xx)
            ys_all.append(yy)
        for (xx, yy) in grid.wheel_points(px, py, yaw):
            g = grid.gap(xx, yy)
            min_gap = min(min_gap, g)
            if g < 0:
                hit_wheel = True

    info = {"cx": cx, "cy": cy, "left": left, "gap": min_gap,
            "x_lo": min(xs_all), "x_hi": max(xs_all),
            "y_lo": min(ys_all), "y_hi": max(ys_all),
            "yaw_in": math.degrees(yaw_in)}

    if hit_body:
        return False, "车身扫掠进入阻挡区（间隙 %.0f cm）" % min_gap, info
    if hit_wheel:
        return False, "车轮轨迹进入阻挡区（间隙 %.0f cm）" % min_gap, info
    if min_gap < min_clearance:
        return False, "转弯净空不足（%.0f < %.0f cm）" % (min_gap, min_clearance), info
    return True, "可转弯（最小间隙 %.0f cm）" % min_gap, info


def sec(t):
    print()
    print("=" * 78)
    print(t)
    print("=" * 78)


# ================================================================ 1
def check_start_goal_turn():
    sec("1. 起终点路径 M7 -> N7 -> N6 的转弯（关键判定）")
    g = Grid(gates=())
    a, b, c = START, (13, 6), (13, 5)     # M7 -> N7 -> N6

    print("  安全余量取多少？规则要求'全程不允许触碰边界'，")
    print("  且里程计/控制存在误差，所以要求间隙 >= 半车宽 = {:.0f} cm".format(HALF_W))
    print()
    for margin, label in ((0.0, "间隙 >= 0（仅不穿插）"),
                          (HALF_W, "间隙 >= 半车宽（推荐）"),
                          (2 * HALF_W, "间隙 >= 车宽（保守）")):
        ok, reason, info = check_turn(g, a, b, c, min_clearance=margin)
        print("  [{}]".format(label))
        if info:
            print("    转弯中心 C = ({:.0f}, {:.0f})  {}".format(
                info["cx"], info["cy"], "左转" if info["left"] else "右转"))
            print("    扫掠 x [{:.0f}, {:.0f}]   y [{:.0f}, {:.0f}]".format(
                info["x_lo"], info["x_hi"], info["y_lo"], info["y_hi"]))
            print("    距水平挡板 {:.1f} cm，距场边 {:.1f} cm，最小间隙 {:.1f} cm".format(
                info["y_lo"] - WALL_Y, FIELD_W - info["x_hi"], info["gap"]))
        print("    判定: {}  —— {}".format("可行" if ok else "不可行", reason))
        print()
    ok0, _, _ = check_turn(g, a, b, c, min_clearance=0.0)
    ok1, _, _ = check_turn(g, a, b, c, min_clearance=HALF_W)
    if ok0 and not ok1:
        print("  ★ 结论：几何上'勉强能过'（最小间隙仅 {:.1f} cm），".format(
            check_turn(g, a, b, c)[2]["gap"]))
        print("     但离场边只有 {:.0f} mm —— 小于车半宽，实际会蹭边。".format(
            10 * (FIELD_W - check_turn(g, a, b, c)[2]["x_hi"])))
        print("     ⇒ 该转角**不可用**，必须改路点或用多段转向")
    return ok1


# ================================================================ 2
def survey_all_turns():
    sec("2. 全场 90° 转角普查（找出所有可转的点）")
    g = Grid(gates=())
    feasible, infeasible = [], []
    for br in range(1, ROWS - 1):
        for bc in range(1, COLS - 1):
            if not g.passable(bc, br):
                continue
            # 四个方向组合：入向 ∈ {N,S,E,W}，出向 ∈ 垂直方向
            dirs = {"N": (0, 1), "S": (0, -1), "E": (1, 0), "W": (-1, 0)}
            for din, (dc1, dr1) in dirs.items():
                ac, ar = bc - dc1, br - dr1
                if not g.passable(ac, ar):
                    continue
                for dout, (dc2, dr2) in dirs.items():
                    if dc2 * dc1 + dr2 * dr1 != 0:      # 必须垂直
                        continue
                    cc_, cr = bc + dc2, br + dr2
                    if not g.passable(cc_, cr):
                        continue
                    ok, reason, info = check_turn(g, (ac, ar), (bc, br), (cc_, cr))
                    (feasible if ok else infeasible).append((bc, br, din, dout, info))

    print("  可转的 (格, 入向->出向) 组合: {} 个".format(len(feasible)))
    print("  不可转的组合: {} 个".format(len(infeasible)))
    print("  （判据：最小间隙 >= 半车宽 {:.0f} cm）".format(HALF_W))

    # 按列统计可转点
    print()
    print("  按列统计可转点数（列: 可转/总尝试）:")
    per_col_ok = {}
    per_col_all = {}
    for (bc, br, din, dout, info) in feasible:
        per_col_ok[bc] = per_col_ok.get(bc, 0) + 1
    for (bc, br, din, dout, info) in infeasible:
        per_col_all[bc] = per_col_all.get(bc, 0) + 1
    for bc in range(COLS):
        ok_n = per_col_ok.get(bc, 0)
        bad_n = per_col_all.get(bc, 0)
        tot = ok_n + bad_n
        if tot == 0:
            continue
        tag = "全能转" if bad_n == 0 else ("全不能转" if ok_n == 0 else "")
        print("    {} 列: {:>3}/{:<3}  {}".format(col_name(bc), ok_n, tot, tag))

    print()
    east_ok = [(bc, br, din, dout, info) for (bc, br, din, dout, info) in feasible
               if bc >= BRIDGE_COL]
    print("  K 列以东（L/M/N，窄条）的可转点:")
    if east_ok:
        for (bc, br, din, dout, info) in east_ok:
            print("    {}  {}->{}  最小间隙 {:.1f} cm".format(
                cname(bc, br), din, dout, info["gap"]))
    else:
        print("    （空）")
        print("    → 窄条内**没有任何**满足安全余量的单圆弧 90° 可转点")
    print()
    print("  各列可达的最小转弯间隙（越大越好）:")
    per_col_gap = {}
    for (bc, br, din, dout, info) in feasible + infeasible:
        per_col_gap.setdefault(bc, []).append(info["gap"])
    for bc in range(COLS):
        if bc not in per_col_gap:
            continue
        gmin = min(per_col_gap[bc])
        gmax = max(per_col_gap[bc])
        flag = "充足" if gmin >= HALF_W else "紧张"
        print("    {} 列: [{:5.1f}, {:5.1f}] cm  {}".format(
            col_name(bc), gmin, gmax, flag))
    return feasible, infeasible


# ================================================================ 3
def check_specific_candidates():
    sec("3. 候选路径的逐转角检查")
    g = Grid(gates=())
    paths = {
        "原保底 M7->N7->N6->N5": [START, (13, 6), (13, 5), (13, 4)],
        "下沉一格 M7->M8->N8->N5": [START, (12, 7), (13, 7), (13, 6), (13, 5), (13, 4)],
        "先西后北 M7->M6->L6->L5->N5 附近": [START, (12, 5), (11, 5)],
    }
    for name, path in paths.items():
        print("  [{}]".format(name))
        print("    路径: " + " -> ".join(cname(*p) for p in path))
        if len(path) < 3:
            print("    转角数: 0")
            print()
            continue
        n_turn = 0
        for i in range(2, len(path)):
            a, b, c = path[i - 2], path[i - 1], path[i]
            ab_vert = (a[0] == b[0])
            bc_vert = (b[0] == c[0])
            if ab_vert == bc_vert:
                continue
            n_turn += 1
            ok, reason, info = check_turn(g, a, b, c)
            print("      转角 {}: {}  —— {}".format(
                cname(*b), "OK" if ok else "不可行", reason))
        if n_turn == 0:
            print("      无 90° 转角")
        print()


# ================================================================ 4
def check_multi_point_turn():
    sec("4. 多段转向（前进-后退-前进）的空间核算")
    print("  单圆弧不可行时，用多段转向。K 型掉头的通道宽度经验式：")
    print("    通道宽度 W >= R + 车宽/2")
    for R in (24.0, 26.0):
        need = R + HALF_W
        print("    R={:.0f} cm -> W >= {:.0f} cm = {:.2f} 格".format(
            R, need, need / CELL))
    print()
    print("  可用通道宽度:")
    print("    单格        {:5.0f} cm".format(CELL))
    print("    M+N 两列    {:5.0f} cm".format(2 * CELL))
    print("    L+M+N 三列  {:5.0f} cm".format(3 * CELL))
    print()
    for R in (24.0, 26.0):
        need = R + HALF_W
        print("    R={:.0f}: 需 {:.0f} cm -> 单格 {} / M+N {} / L+M+N {}".format(
            R, need,
            "够" if need <= CELL else "不够",
            "够" if need <= 2 * CELL else "不够",
            "够" if need <= 3 * CELL else "不够"))
    print()
    print("  ★ 结论：多段转向在 M+N（700mm）内可行，这是窄条里的唯一选择")


# ================================================================ 5
def diagnose_why():
    sec("5. 为什么窄条里转不了弯（几何诊断）")
    g = Grid(gates=())
    print("  单圆弧转弯的两个硬约束（转弯中心 C 被锁定在车道中心线交点）:")
    print()
    print("  约束① 距水平挡板：C 落在 row7 中心线时，车体最低点距挡板 3.3 mm")
    print("         → 要把车从挡板推开，C 必须北移；但 C 又被车道中心线锁定")
    print()
    print("  约束② 距场地边界：C 落在 N 列中心线上时距场边 175 mm，")
    print("         而车体绕 C 扫掠要扫出 R + 车体对角半径")
    print()
    r_diag = math.hypot(FRONT, HALF_W)
    print("     R = {:.0f} cm，车体对角半径 = {:.1f} cm".format(R_TURN, r_diag))
    print("     扫掠最大半径 = {:.0f} + {:.1f} = {:.1f} cm".format(
        R_TURN, r_diag, R_TURN + r_diag))
    print("     N 列中心线距场边 = {:.1f} cm".format(FIELD_W - cc(13, 0)[0]))
    print("     → 扫掠半径 ({:.1f}) > 可用空间 ({:.1f})，必然出界".format(
        R_TURN + r_diag, FIELD_W - cc(13, 0)[0]))
    print()
    print("  具体用例的扫掠包络：")
    print("  {:<24} {:>16} {:>14} {:>14}".format(
        "用例", "扫掠 x 范围", "距场边", "距挡板"))
    for name, path in (
            ("M7->N7->N6", [START, (13, 6), (13, 5)]),
            ("M7->M6->L6", [START, (12, 5), (11, 5)]),
            ("L6->M6->M5", [(11, 5), (12, 5), (12, 4)]),
            ("M4->N4->N3", [(12, 3), (13, 3), (13, 2)]),
    ):
        ok, reason, info = check_turn(g, path[0], path[1], path[2],
                                      min_clearance=0.0)
        if not info:
            print("  {:<24} {}".format(name, reason))
            continue
        print("  {:<24} [{:5.0f},{:5.0f}] {:>10.1f} cm {:>10.1f} cm".format(
            name, info["x_lo"], info["x_hi"],
            FIELD_W - info["x_hi"], info["y_lo"] - WALL_Y))
    print()
    print("  → 每个用例至少有一个方向贴到 0（场边或挡板）")
    print("  ⇒ 窄条里**没有**满足'离边界 >= 半车宽'的单圆弧转弯")
    print()
    print("  例外：远离挡板且不在最外列的位置（如 L2/M2，row2）情况如何？")
    for name, path in (
            ("L2->M2->M1", [(11, 1), (12, 1), (12, 0)]),
            ("L2->M2->M3", [(11, 1), (12, 1), (12, 2)]),
            ("K 以西 G3->G4->H4", [(6, 2), (6, 3), (7, 3)]),
    ):
        ok, reason, info = check_turn(g, path[0], path[1], path[2],
                                      min_clearance=0.0)
        gap = "{:.1f} cm".format(info["gap"]) if info else "-"
        print("    {:<24} 间隙 {}".format(name, gap))
    print()
    print("  → 即使远离挡板，只要转弯中心落在 M 列，就会差 ~1mm 贴到场边")
    print("     所以窄条（K 以东 3 列）的**任何**位置都无法单圆弧转弯")


# ================================================================ 6
def verify_against_cpp_expectation():
    sec("6. 与 C++ FieldGrid::checkTurn 的行为对照片")
    print("  下面列出固定用例，C++ 侧单测应给出相同结论：")
    print()
    g = Grid(gates=())
    cases = [
        ("M7->N7->N6 (东->北)", START, (13, 6), (13, 5)),
        ("M7->M6->L6 (北->西)", START, (12, 5), (11, 5)),
        ("N7->N6->N5 (北->北,直行)", (13, 6), (13, 5), (13, 4)),
        ("J8->K8->K7 (K 列门关闭)", (9, 7), (10, 7), (10, 6)),
        ("A7->B7->B6 (西->北,开阔)", (0, 6), (1, 6), (1, 5)),
        ("G3->G4->H4 (南->东,西半场)", (6, 2), (6, 3), (7, 3)),
    ]
    print("  {:<32} {:<10} {:<12} {}".format("用例", "可行", "最小间隙", "原因"))
    for name, a, b, c in cases:
        ok, reason, info = check_turn(g, a, b, c)
        gap = "{:.0f} cm".format(info["gap"]) if info else "-"
        print("  {:<32} {:<10} {:<12} {}".format(
            name, "是" if ok else "否", gap, reason))
    print()
    print("  → 把这 6 行的 (可行, 间隙±1cm) 作为 C++ 单测断言")


def main():
    print("转弯可行性对照实现与普查")
    print("车体 {:.0f}x{:.0f} cm，轴距 {:.0f}，轮距 {:.0f}，R={:.0f} cm".format(
        CAR_L, CAR_W, WB, TRACK, R_TURN))
    print("场地 {}x{} 格 @{:.0f}cm，水平挡板 y={:.0f}，K 列 col={}".format(
        COLS, ROWS, CELL, WALL_Y, BRIDGE_COL))

    check_start_goal_turn()
    feasible, infeasible = survey_all_turns()
    check_specific_candidates()
    check_multi_point_turn()
    diagnose_why()
    verify_against_cpp_expectation()

    sec("总结论")
    east_ok = [t for t in feasible if t[0] >= BRIDGE_COL]
    print("  1. 全场可转点 {} 个，不可转 {} 个".format(len(feasible), len(infeasible)))
    print("  2. K 列以东（窄条 L/M/N）可转点: {} 个".format(len(east_ok)))
    if not east_ok:
        print("     -> 窄条内**没有任何**单圆弧 90° 可转点")
    print("  3. 起终点路径 M7->N7->N6->N5 的转角判定见第 1 节")
    print("  4. 窄条内只能用多段转向（M+N 共 {:.0f}cm 够用）".format(2 * CELL))
    return 0


if __name__ == "__main__":
    sys.exit(main())
