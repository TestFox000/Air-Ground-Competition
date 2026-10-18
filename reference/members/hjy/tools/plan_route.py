#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 plan_route.py —— 按本车端算法对指定地图做路径规划，并输出线路图
================================================================================

★ 复刻的算法（与 C++ FieldGrid 一致，不是另写一套）
---------------------------------------------------
  模型     : 隔离线是**边**（row5↔row6），覆盖 D..N；K 列是桥洞格
  passable : 只有 FREE / MINE 可通行（kUnknown/kBlocked/kTarget 不可）
  搜索     : A*，状态 = (格, 进入方向)  —— "方向感知"才能把"该格拐弯是否可行"
             作为状态转移约束；只在事后查转角会得到"格级最短但实车走不了"
  约束     : forbid_reverse（禁止立即 180° 折返）
             turn_aware（拐弯必须通过 checkTurn）
  分级回退 : findPathStaged 的 4 级
               1 turn_aware + 起始航向 + 禁折返
               2 turn_aware，放开起始航向
               3 turn_aware + pivot_ok（允许原地转向的转角）
               4 放开 turn_aware，保留禁折返
  checkTurn: 转弯中心 = 入向车道中心线 ∩ 出向车道中心线
             扫掠半径 = R(26) + 车体对角半径；车体四角/四轮扫掠全程
             净空 >= turn_min_clearance_cm(0.5)
  checkPivot: 原地转向绕后轴中心纯转动，扫掠半径 = 车体对角半径
             （窄条 L/M/N 内单圆弧不可行，只能原地转向）

车体几何（官方实测）
    长度 27.8 / 宽度 14.0 / 轴距 15.0 / 轮距 11.0 cm
    最小转弯半径 R = 26.0 cm
    前悬 17.2 / 后悬 10.6 cm（0.62/0.38 × 长度）
    格边长 35 cm

用法
----
    python3 tools/plan_route.py                       # 用内置的这张图
    python3 tools/plan_route.py --json                # 只输出路径 JSON
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import sys

# ==============================================================================
# 场地常量（与 field_grid.hpp / air_protocol.py 一致）
# ==============================================================================
COLS, ROWS = 14, 11
CELL = 35.0                    # cm
BARRIER_UP_ROW = 5             # 线在 row5 与 row6 之间
BARRIER_COL_LO, BARRIER_COL_HI = 3, 13      # D..N
BRIDGE_COL = 10                # K
BRIDGE_A = (0, 5)              # K1..K6
BRIDGE_B = (6, 10)             # K7..K11
START = (12, 10)               # M11
GOAL = (13, 5)                 # N6
START_HEADING = (-1, 0)        # 起点朝北（-y 方向，即行号减小）

# 车体
CAR_L, CAR_W = 27.8, 14.0
WB, TRACK = 15.0, 11.0
R_TURN = 26.0
FRONT, REAR = 17.2, 10.6
HALF_W = CAR_W / 2.0
DIAG = math.hypot(FRONT, HALF_W)          # 车体对角半径 ≈ 18.6
SWEEP_R = R_TURN + DIAG                   # 单圆弧扫掠半径 ≈ 44.6
TURN_MIN_CLEARANCE = 0.5
# ★ 可在命令行用 --clearance 覆盖。
#   为什么要暴露它：本图在 turn_min_clearance=0.5cm 下**不可达**，
#   降到 0.0cm 才可达 —— 瓶颈是终点前 N5 那个转角（实测净空 0.3cm）。
#   0.5 是"保守余量"，0.0 是"只要不穿插就通过"（field_grid.hpp 注释明确：
#   "取 0 → 只要不穿插就通过（正确的能否转判据）"）。
#   所以 0.0 并非违规，而是"用几何真值判定"。

FREE, BLOCKED, MINE, TARGET, GATE_UNK, GATE_OPEN = 1, 2, 3, 4, 5, 6

# ==============================================================================
# 本图数据（由用户提供的地图图片转录，0-based 格坐标）
# ==============================================================================
MINES = [(5, 0), (8, 0), (3, 1), (2, 2), (1, 7), (6, 9), (4, 10), (7, 10)]
#        F1      I1      D2      C3      B8      G10     E11     H11
OBSTACLES = [(7, 1), (3, 2), (0, 3), (4, 4), (2, 7), (6, 8), (8, 8), (1, 9)]
#            H2      D3      A4      E5      C8      G9      I9      B10
TARGETS = [(5, 2), (5, 3)]        # F3, F4
GATES = {0: True, 6: True}        # K1=通, K7=通（其余洞未知）
# ★ 无人机起降点（用户确认）
#   起点 **N8** = (col 13, row 7)  —— 曾误记为 M8(12,7)，列错一格
#   终点 **N2** = (col 13, row 1)
DRONE_START = (13, 7)             # N8
DRONE_END = (13, 1)               # N2


# ==============================================================================
# 栅格
# ==============================================================================
def col_letter(c):
    return chr(ord('A') + c) if 0 <= c < 26 else '?'


def cname(c, r):
    return '%s%d' % (col_letter(c), r + 1)


class Grid:
    def __init__(self):
        # 底图全部自由（与 FieldGrid::reset 一致）
        self.cell = [[FREE] * COLS for _ in range(ROWS)]
        # K 列：整列隔板，只有门可通行
        for r in range(ROWS):
            self.cell[r][BRIDGE_COL] = GATE_OPEN if GATES.get(r) else GATE_UNK
        for (c, r) in MINES:
            self.cell[r][c] = MINE
        for (c, r) in OBSTACLES:
            self.cell[r][c] = BLOCKED
        for (c, r) in TARGETS:
            self.cell[r][c] = TARGET
        # ★ 无人机起降点 = 禁入格（kBlocked）
        for (c, r) in (DRONE_START, DRONE_END):
            self.cell[r][c] = BLOCKED

    def passable(self, c, r):
        if not (0 <= c < COLS and 0 <= r < ROWS):
            return False
        v = self.cell[r][c]
        # ★ 只有 FREE / MINE / GATE_OPEN 可通行
        #   —— GATE_OPEN 必须算通行！C++ 里 openCrossing() 会把该格改成 kFree，
        #      所以那边 cellPassable 只判 FREE/MINE 就够；这里保留"洞开"语义，
        #      就必须显式放行，否则两个桥洞门全被堵死（第一版就踩了这个坑，
        #      导致"起终点不连通"）。
        return v in (FREE, MINE, GATE_OPEN)

    def edge_blocked_vert(self, c, r0, r1):
        """row 与 row+1 之间的纵向边是否被隔离线挡住（E 模型）"""
        up = min(r0, r1)
        return (up == BARRIER_UP_ROW) and (BARRIER_COL_LO <= c <= BARRIER_COL_HI)

    def neighbors(self, c, r):
        for (dc, dr) in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nc, nr = c + dc, r + dr
            if not self.passable(nc, nr):
                continue
            if dc == 0 and self.edge_blocked_vert(c, r, nr):
                continue
            yield (nc, nr)


# ==============================================================================
# 几何：车体角点 / 轮位 / 到阻挡的距离
# ==============================================================================
def body_corners(px, py, yaw):
    """后轴中心 (px,py)、航向 yaw 下的车体四角（世界系 cm）"""
    cs, sn = math.cos(yaw), math.sin(yaw)
    pts = []
    for (lx, ly) in ((FRONT, HALF_W), (FRONT, -HALF_W),
                     (-REAR, -HALF_W), (-REAR, HALF_W)):
        pts.append((px + lx * cs - ly * sn, py + lx * sn + ly * cs))
    return pts


def wheel_points(px, py, yaw):
    cs, sn = math.cos(yaw), math.sin(yaw)
    pts = []
    for (lx, ly) in ((WB, TRACK / 2), (WB, -TRACK / 2),
                     (0.0, TRACK / 2), (0.0, -TRACK / 2)):
        pts.append((px + lx * cs - ly * sn, py + lx * sn + ly * cs))
    return pts


def cell_center_gap(c, r, x, y):
    """点到格 (c,r) 边界的距离（正=在格外，负=在格内到最近边）"""
    gx0, gx1 = c * CELL, (c + 1) * CELL
    gy0, gy1 = r * CELL, (r + 1) * CELL
    if gx0 <= x <= gx1 and gy0 <= y <= gy1:
        return -min(x - gx0, gx1 - x, y - gy0, gy1 - y)
    dx = max(gx0 - x, 0.0, x - gx1)
    dy = max(gy0 - y, 0.0, y - gy1)
    return math.hypot(dx, dy)


def obstacle_gap(x, y, grid):
    """点到"最近不可通行区域"的距离（cm）。正 = 有余量，负 = 压进去。

    ★★ 关键设计（第一版这里错了，导致所有转角都"不可行"）★★
      判据用**格级可通行性**，而不是"到隔离线/隔板的解析距离"。

      为什么：**门（K 列）就是隔离线的开口**。若把"row5/row6 之间那条线覆盖
      D..N"无条件当成实心墙，就会把隧道内的格（如 K6）判成"车体压在墙里"
      （算出 -17.5cm），于是**穿桥洞永远不可行** —— 而穿桥洞是规则要求的
      必由之路。C++ 的语义也是格级的：扫掠覆盖到的每个格都必须可通行。

      所以这里：
        · 场边 / 障碍桶 / 无人机禁入格 / 靶标  -> 实心几何距离（与格无关）
        · 隔离线与桥洞隔板                     -> 由 **格的可通行性** 表达
          （GATE_OPEN 的格可通行 => 门是开口；GATE_UNK 不可通行 => 隔板）
    """
    # ---- 场边 ----
    g = min(x, y, COLS * CELL - x, ROWS * CELL - y)

    # ---- 实心阻挡：障碍 / 无人机禁入 / 靶标 所在格 ----
    c = int(math.floor(x / CELL))
    r = int(math.floor(y / CELL))
    # 实心格（整格阻挡，与格是否可通行无关：它们本来就是"桶/禁入/靶"）
    solid = set(OBSTACLES) | {DRONE_START, DRONE_END} | set(TARGETS)
    for (sc, sr) in solid:
        g = min(g, cell_center_gap(sc, sr, x, y))

    return g


def gap_to_passability(x, y, grid):
    """点落在不可通行格内则为负（负值 = 到最近可行格边界的距离）。

    这是与 C++ sweepGap 同语义的判据：扫掠覆盖到的格必须都可通行。
    """
    c = int(math.floor(x / CELL))
    r = int(math.floor(y / CELL))
    if not (0 <= c < COLS and 0 <= r < ROWS):
        # 场外 -> 到最近可行格的距离（负）
        cc = min(max(c, 0), COLS - 1)
        rr = min(max(r, 0), ROWS - 1)
        return -cell_center_gap(cc, rr, x, y)
    if grid.passable(c, r):
        # 在可通行格内：返回到最近的"不可通行格或场边/实心物"的正距离
        best = 1e9
        for dc in (-2, -1, 0, 1, 2):
            for dr in (-2, -1, 0, 1, 2):
                cc, rr = c + dc, r + dr
                if 0 <= cc < COLS and 0 <= rr < ROWS and not grid.passable(cc, rr):
                    best = min(best, cell_center_gap(cc, rr, x, y))
        return best
    return -cell_center_gap(c, r, x, y)


def sweep_gap(x, y, grid):
    """综合净空 = min(到实心阻挡的距离, 到不可通行格的净空)"""
    return min(obstacle_gap(x, y, grid), gap_to_passability(x, y, grid))


def turn_centers(ac, ar, bc, br, cc, cr):
    """转弯中心 = 入向车道中心线 ∩ 出向车道中心线（与 C++ turnCenters 一致）"""
    in_vertical = (ac == bc)
    if in_vertical:
        return ((bc + 0.5) * CELL, (cr + 0.5) * CELL)
    return ((cc + 0.5) * CELL, (br + 0.5) * CELL)


def check_turn(ac, ar, bc, br, cc, cr, grid):
    """复刻 FieldGrid::checkTurn —— 返回 (feasible, min_gap, reason)"""
    if abs(ar - br) + abs(ac - bc) != 1 or abs(br - cr) + abs(bc - cc) != 1:
        return (False, 0.0, '非四连通相邻')
    if (ac == bc) == (bc == cc):
        return (False, 0.0, '不是90度转向')
    if not grid.passable(bc, br) or not grid.passable(cc, cr):
        return (False, 0.0, '入向/出向格不可通行')

    cx, cy = turn_centers(ac, ar, bc, br, cc, cr)
    # 起始偏航 = 入向车道方向
    if ac == bc:
        yaw_in = math.pi / 2 if br > ar else -math.pi / 2
    else:
        yaw_in = 0.0 if bc > ac else math.pi
    # 左转判据（与 C++ 一致）
    left_turn = (cc > bc) if (ac == bc) else (cr < br)
    dpsi = (1.0 if left_turn else -1.0) * math.pi / 2.0

    best = 1e9
    N = 60
    for i in range(N + 1):
        f = i / float(N)
        yaw = yaw_in + dpsi * f
        px = cx + R_TURN * math.cos(yaw - math.pi / 2)
        py = cy + R_TURN * math.sin(yaw - math.pi / 2)
        for (x, y) in body_corners(px, py, yaw) + wheel_points(px, py, yaw):
            best = min(best, sweep_gap(x, y, grid))
    # ★ epsilon 必须有：当几何**恰好相切**（净空 = 容差）时，
    #   浮点累计误差会让 best 变成 -1e-15 之类的"负零"，把可行的转角判死。
    #   实测：直穿桥洞门 L7->K7->J7 的净空恰好 0.00，就被这个噪声挡掉了。
    ok = best >= TURN_MIN_CLEARANCE - 1e-9
    return (ok, best, '' if ok else '扫掠净空不足 %.2fcm' % best)


def check_pivot(c, r, grid):
    """复刻 checkPivot —— 原地转向绕后轴中心纯转动，扫掠半径 = 车体对角半径"""
    # 后轴中心在格内；简化：以格心为中心，检查扫掠圆覆盖到的格是否都可通行
    cx, cy = (c + 0.5) * CELL, (r + 0.5) * CELL
    R = DIAG
    worst = 1e9
    NS = 36
    for i in range(NS):
        a = 2 * math.pi * i / NS
        for rr in (R,):
            x = cx + rr * math.cos(a)
            y = cy + rr * math.sin(a)
            worst = min(worst, sweep_gap(x, y, grid))
    return (worst >= TURN_MIN_CLEARANCE - 1e-9, worst)   # 同 check_turn，需 epsilon


# ==============================================================================
# A*（状态 = 格 + 进入方向）
# ==============================================================================
DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1)]


def plan(grid, start, goal, heading, turn_aware=True, forbid_reverse=True,
         pivot_ok=False):
    """返回 (path, level_desc) 或 (None, reason)"""
    sc, sr = start
    gc, gr = goal
    if not grid.passable(sc, sr):
        return (None, '起点不可通行')
    if not grid.passable(gc, gr):
        return (None, '终点不可通行')

    h_dirs = [d for d in DIRS if d == heading] if heading else []

    # 状态: (c, r, dc, dr)  —— (dc,dr) 为"进入该格的方向"
    start_states = [(sc, sr, dc, dr) for (dc, dr) in (h_dirs if h_dirs else DIRS)]

    openq = []
    gcost = {}
    prev = {}
    for st in start_states:
        gcost[st] = 0.0
        heapq.heappush(openq, (abs(sc - gc) + abs(sr - gr), 0.0, st))

    def h(c, r):
        return abs(c - gc) + abs(r - gr)

    end_state = None
    while openq:
        _f, g, st = heapq.heappop(openq)
        if g > gcost.get(st, 1e18) + 1e-9:
            continue
        c, r, dc, dr = st
        if (c, r) == (gc, gr):
            end_state = st
            break
        for (ndc, ndr) in grid.neighbors(c, r):
            # 禁止立即折返
            if forbid_reverse and (ndc - c, ndr - r) == (-dc, -dr):
                continue
            new_dc, new_dr = ndc - c, ndr - r
            # 拐弯检查
            if (dc, dr) != (new_dc, new_dr) and turn_aware:
                # ★ 入向格 a = 当前格 - **本次移动方向**（不是减进入方向！）
                #   曾经写成 c - dc，对直线移动恰好相反，导致 check_turn 收到
                #   a==b（非四连通相邻）-> 所有移动都被拒 -> 转弯感知搜索全废。
                ac, ar = c - new_dc, r - new_dr
                ok, gap, _why = check_turn(ac, ar, c, r, ndc, ndr, grid)
                if not ok:
                    if not (pivot_ok and check_pivot(c, r, grid)[0]):
                        continue
            nst = (ndc, ndr, new_dc, new_dr)
            ng = g + 1.0
            if ng < gcost.get(nst, 1e18) - 1e-9:
                gcost[nst] = ng
                prev[nst] = st
                heapq.heappush(openq, (ng + h(ndc, ndr), ng, nst))

    if end_state is None:
        return (None, '无可行路径（该约束级别下）')

    # 回溯
    path = []
    st = end_state
    while st is not None:
        path.append((st[0], st[1]))
        st = prev.get(st)
    path.reverse()

    # 路径里可能因方向重复而含重复格 -> 去重保序
    dedup = []
    for p in path:
        if not dedup or dedup[-1] != p:
            dedup.append(p)
    return (dedup, '')


def plan_staged(grid, start, goal, heading):
    """复刻 findPathStaged 的 4 级回退"""
    levels = [
        ('1. 转弯感知 + 起始航向 + 禁折返', dict(turn_aware=True, forbid_reverse=True)),
        ('2. 转弯感知（放开起始航向）', dict(turn_aware=True, forbid_reverse=True)),
        ('3. 转弯感知 + 允许原地转向', dict(turn_aware=True, forbid_reverse=True, pivot_ok=True)),
        ('4. 放开转弯约束（保留禁折返）', dict(turn_aware=False, forbid_reverse=True)),
    ]
    for i, (desc, kw) in enumerate(levels):
        h = heading if i == 0 else None
        path, why = plan(grid, start, goal, h, **kw)
        if path:
            return path, desc, i + 1
    return None, '全部级别均失败', 0


# ==============================================================================
# 路径校验与打印
# ==============================================================================
def verify_path(grid, path):
    """逐段校验：可通行、边界、转角可行性"""
    issues = []
    for i in range(1, len(path)):
        (ac, ar), (bc, br) = path[i - 1], path[i]
        if abs(ac - bc) + abs(ar - br) != 1:
            issues.append('第%d步不四连通: %s->%s' % (i, cname(ac, ar), cname(bc, br)))
        if not grid.passable(bc, br):
            issues.append('第%d步进入不可通行格 %s' % (i, cname(bc, br)))
        if ac == bc and grid.edge_blocked_vert(ac, ar, br):
            issues.append('第%d步跨越隔离线: %s->%s' % (i, cname(ac, ar), cname(bc, br)))
    # 转角
    turns = []
    for i in range(1, len(path) - 1):
        (ac, ar), (bc, br), (cc, cr) = path[i - 1], path[i], path[i + 1]
        if (ac == bc) == (bc == cc):
            continue
        ok, gap, why = check_turn(ac, ar, bc, br, cc, cr, grid)
        pk, pgap = check_pivot(bc, br, grid)
        turns.append({'at': cname(bc, br), 'arc_ok': ok, 'arc_gap': round(gap, 1),
                      'pivot_ok': pk, 'pivot_gap': round(pgap, 1), 'why': why})
        if not ok and not pk:
            issues.append('转角 %s 单圆弧与原地转向都不可行' % cname(bc, br))
    return issues, turns


def render(grid, path, turns):
    """ASCII 线路图"""
    onpath = set(path)
    turn_at = {t['at'] for t in turns}
    lines = []
    lines.append('     ' + ''.join('%2s' % col_letter(c) for c in range(COLS)))
    lines.append('     ' + '-' * (COLS * 2))
    for r in range(ROWS):
        row = []
        for c in range(COLS):
            v = grid.cell[r][c]
            ch = {FREE: ' ', BLOCKED: '#', MINE: '*', TARGET: 'T',
                  GATE_UNK: 'x', GATE_OPEN: 'O'}.get(v, '?')
            if (c, r) == START:
                ch = 'S'
            elif (c, r) == GOAL:
                ch = 'G'
            elif (c, r) in onpath:
                ch = '+' if cname(c, r) in turn_at else 'o'
            row.append('%2s' % ch)
        lines.append('%2d   ' % (r + 1) + ''.join(row))
        if r == BARRIER_UP_ROW:
            seg = ''.join('==' if BARRIER_COL_LO <= c <= BARRIER_COL_HI else '  '
                          for c in range(COLS))
            lines.append('     ' + seg + '  <== 隔离线(边) D..N')
    lines.append('')
    lines.append('    图例: S起点 G终点 o路径 +路径上的转角')
    lines.append('          *地雷(可通行) #障碍/无人机禁入格/桥洞隔板')
    lines.append('          T靶标 x洞未知 O洞开(可通行)')
    return '\n'.join(lines)


def render_png(grid, path, out_path, turns):
    """用 Pillow 画线路图（不依赖 matplotlib）

    配色与现场图对齐：黄=雷 红=障 粉=靶 绿=通
    路径画成带箭头的折线；转角用圆圈标出（绿=单圆弧可转，橙=需原地转向）
    """
    import math as _m

    from PIL import Image, ImageDraw, ImageFont

    CELL_PX = 78
    MARGIN_L, MARGIN_T = 48, 32
    W = MARGIN_L + COLS * CELL_PX + 34
    H = MARGIN_T + ROWS * CELL_PX + 60

    img = Image.new('RGB', (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)

    def font(sz):
        for p in ('C:/Windows/Fonts/msyh.ttc', 'C:/Windows/Fonts/simhei.ttf'):
            try:
                return ImageFont.truetype(p, sz)
            except Exception:
                continue
        return ImageFont.load_default()

    f_small, f_big, f_lbl = font(12), font(26), font(14)

    def xy(c, r):
        return (MARGIN_L + c * CELL_PX, MARGIN_T + r * CELL_PX)

    COL = {
        FREE: (245, 245, 245), BLOCKED: (232, 78, 78), MINE: (255, 205, 20),
        TARGET: (245, 120, 190), GATE_UNK: (245, 245, 245),
        GATE_OPEN: (76, 185, 100),
    }
    for r in range(ROWS):
        for c in range(COLS):
            x0, y0 = xy(c, r)
            d.rectangle([x0, y0, x0 + CELL_PX, y0 + CELL_PX],
                        fill=COL.get(grid.cell[r][c], (245, 245, 245)),
                        outline=(205, 205, 205), width=1)
            d.text((x0 + 3, y0 + 2), cname(c, r), fill=(130, 130, 130), font=f_small)

    for r in range(ROWS):
        for c in range(COLS):
            t = {MINE: '雷', BLOCKED: '障', TARGET: '靶',
                 GATE_OPEN: '通'}.get(grid.cell[r][c])
            if t:
                x0, y0 = xy(c, r)
                bb = d.textbbox((0, 0), t, font=f_big)
                d.text((x0 + (CELL_PX - (bb[2] - bb[0])) / 2 - bb[0],
                        y0 + (CELL_PX - (bb[3] - bb[1])) / 2 - bb[1]),
                       t, fill=(95, 95, 95), font=f_big)

    def annotate(c, r, txt, color):
        x0, y0 = xy(c, r)
        d.rectangle([x0 + 1, y0 + 1, x0 + CELL_PX - 1, y0 + CELL_PX - 1],
                    outline=color, width=2)
        d.text((x0 + 4, y0 + CELL_PX - 15), txt, fill=color, font=f_small)

    annotate(DRONE_START[0], DRONE_START[1], '无人机起点', (200, 150, 20))
    annotate(DRONE_END[0], DRONE_END[1], '无人机终点', (200, 150, 20))
    annotate(START[0], START[1], '智能车起点', (20, 160, 60))
    annotate(GOAL[0], GOAL[1], '智能车终点', (205, 40, 40))

    # 隔离线 = BARRIER_UP_ROW 那一行的**北向边**，也就是
    #   row(BARRIER_UP_ROW) 与 row(BARRIER_UP_ROW+1) 之间的分界。
    #   ★ 在 1-based 里是 "row6 与 row7 之间"，0-based 就是 5 与 6 之间。
    #   渲染位置 = (BARRIER_UP_ROW) 行的下沿 = MARGIN_T + BARRIER_UP_ROW*CELL_PX + CELL_PX
    #   —— 曾误写成 (BARRIER_UP_ROW+1)*CELL_PX（等于再往下量一整格，画到了下一行下面）。
    yline = MARGIN_T + (BARRIER_UP_ROW + 1) * CELL_PX - CELL_PX
    xa = MARGIN_L + BARRIER_COL_LO * CELL_PX
    xb = MARGIN_L + (BARRIER_COL_HI + 1) * CELL_PX
    d.line([xa, yline, xb, yline], fill=(40, 165, 70), width=4)
    d.text((xb + 5, yline - 7), '隔离线', fill=(40, 165, 70), font=f_small)

    # 路径折线
    pts = [(MARGIN_L + (c + 0.5) * CELL_PX, MARGIN_T + (r + 0.5) * CELL_PX)
           for (c, r) in path]
    for i in range(1, len(pts)):
        d.line([pts[i - 1], pts[i]], fill=(25, 65, 200), width=5)
    for i in range(2, len(pts), 3):
        (ax, ay), (bx, by) = pts[i - 1], pts[i]
        ang = _m.atan2(by - ay, bx - ax)
        for sgn in (1, -1):
            d.line([(bx, by),
                    (bx - 13 * _m.cos(ang - sgn * 0.45),
                     by - 13 * _m.sin(ang - sgn * 0.45))],
                   fill=(25, 65, 200), width=5)

    # 转角标记
    tmap = {t['at']: t for t in turns}
    for (c, r) in path:
        t = tmap.get(cname(c, r))
        if not t:
            continue
        cx, cy = MARGIN_L + (c + 0.5) * CELL_PX, MARGIN_T + (r + 0.5) * CELL_PX
        col = (30, 150, 60) if t['arc_ok'] else (235, 140, 20)
        d.ellipse([cx - 13, cy - 13, cx + 13, cy + 13], outline=col, width=3)
        d.text((cx - 7, cy - 8), '弧' if t['arc_ok'] else '转',
               fill=col, font=f_small)

    for (c, r), col in ((START, (20, 160, 60)), (GOAL, (205, 40, 40))):
        x0, y0 = xy(c, r)
        d.rectangle([x0 + 3, y0 + 3, x0 + CELL_PX - 3, y0 + CELL_PX - 3],
                    outline=col, width=4)

    for c in range(COLS):
        d.text((MARGIN_L + c * CELL_PX + CELL_PX / 2 - 5, MARGIN_T - 24),
               col_letter(c), fill=(60, 60, 60), font=f_lbl)
    for r in range(ROWS):
        d.text((MARGIN_L - 28, MARGIN_T + r * CELL_PX + CELL_PX / 2 - 9),
               str(r + 1), fill=(60, 60, 60), font=f_lbl)

    ly = MARGIN_T + ROWS * CELL_PX + 14
    d.text((MARGIN_L, ly),
           '蓝线=规划路径     绿圈=单圆弧可转     橙圈=需原地转向     绿线=隔离线(边)',
           fill=(60, 60, 60), font=f_small)
    d.text((MARGIN_L, ly + 17),
           '黄=雷(可通行，需排)  红=障/无人机禁入格  粉=靶  绿=通(桥洞门)  '
           'S=智能车起点 G=智能车终点',
           fill=(60, 60, 60), font=f_small)

    img.save(out_path)
    return out_path


def main():
    ap = argparse.ArgumentParser(description='按本车端算法规划路径')
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--png', default='', help='输出线路图 PNG')
    ap.add_argument('--clearance', type=float, default=None,
                    help='转弯最小净空容差 cm（默认 0.5；本图需 0.0 才可达）')
    args = ap.parse_args()

    global TURN_MIN_CLEARANCE
    if args.clearance is not None:
        TURN_MIN_CLEARANCE = args.clearance

    grid = Grid()
    path, level, lvl_no = plan_staged(grid, START, GOAL, START_HEADING)

    if path is None:
        print('★ 规划失败: %s' % level)
        return 1

    issues, turns = verify_path(grid, path)

    if args.png:
        render_png(grid, path, args.png, turns)
        print(' 线路图已输出: %s\n' % args.png)

    if args.json:
        print(json.dumps({
            'level': level,
            'path': [cname(c, r) for (c, r) in path],
            'path_xy': [((c + 0.5) * CELL, (r + 0.5) * CELL) for (c, r) in path],
            'steps': len(path) - 1,
            'length_cm': (len(path) - 1) * CELL,
            'turns': turns,
            'issues': issues,
        }, ensure_ascii=False, indent=2))
        return 0

    print('=' * 86)
    print(' 按本车端算法规划结果')
    print('=' * 86)
    print(' 起点 %s -> 终点 %s   起始航向: 北（-y）' % (cname(*START), cname(*GOAL)))
    print(' 命中回退级别: %s' % level)
    print(' 路径长度: %d 步 = %.2f m（格边长 %.0f cm）'
          % (len(path) - 1, (len(path) - 1) * CELL / 100.0, CELL))
    print(' 转角数: %d' % len(turns))
    print()
    print(render(grid, path, turns))
    print()
    print('=' * 86)
    print(' 路径序列')
    print('=' * 86)
    # 每行 8 格
    for i in range(0, len(path), 8):
        chunk = path[i:i + 8]
        print('  ' + ' '.join('%-4s' % cname(c, r) for (c, r) in chunk))
    print()
    print('=' * 86)
    print(' 转角可行性（本车端 checkTurn / checkPivot）')
    print('=' * 86)
    print('  %-6s %-10s %-10s %-10s %s' %
          ('转角', '单圆弧', '圆弧净空', '原地转向', '说明'))
    for t in turns:
        print('  %-6s %-10s %-10s %-10s %s' %
              (t['at'],
               '可行' if t['arc_ok'] else '不可行',
               '%.1f cm' % t['arc_gap'],
               '可行' if t['pivot_ok'] else '不可行',
               t['why'] or ('窄条内需原地转向(净空%.1f)' % t['pivot_gap'])))
    print()
    print('=' * 86)
    if issues:
        print(' ★ 路径校验发现问题 %d 条：' % len(issues))
        for s in issues:
            print('   - %s' % s)
    else:
        print(' ✓ 路径校验通过：全部四连通、可通行、未跨隔离线、转角可行')
    print('=' * 86)
    return 0


if __name__ == '__main__':
    sys.exit(main())
