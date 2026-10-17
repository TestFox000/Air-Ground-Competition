#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
全局路径规划器   global_planner.py
================================================================================
输入：起点、终点、8 个雷、8 个障、2 个靶标、2 个桥洞（合计 20 个坐标点）
输出：最优访问路径序列（可直接喂给 path_executor 的 /nav/waypoint）

--------------------------------------------------------------------------------
三层结构，逐层收窄搜索空间 —— 这样既做到"雷的全排列穷举"，
又不会因为 20 个点的全排列（20! ≈ 2.4e18）而算不出来。
--------------------------------------------------------------------------------

  第 1 层  拓扑网格与约束建模
      14(列 A~N) x 11(行 1~11) 二维通行数组
      · 8 个"障"占格不可通行
      · ★挡板：第 6 行与第 7 行之间、D~N 列强制不可通行
        也就是说 6<->7 行的跨越【只有 A/B/C 列】能走
      · 2 个靶标：本身可通行，但规划时把它周围的空格作为"候选射击点"

  第 2 层  任务顺序规划
      · 对 8 个雷做【全排列枚举】(8! = 40320)，按代价排序取前 N 名
        代价 = 曼哈顿距离 + 方向惩罚 + 障碍惩罚 + 跨区惩罚
      · 再对入围的前 N 名，用"最优插入"把 2 靶 + 2 桥洞插进序列
        （4 个点插入 14 个位置，做全枚举代价可接受）

  第 3 层  阿克曼约束路径规划（A*）
      状态 = (行, 列, 朝向)，动作集 = { 前进, 倒车, 左转90°, 右转90° }
      · 普通 A* 只有位置，规划出的路径车拐不过去（阿克曼不能原地转）
      · 加入朝向维度后，转向代价被显式计入，倒车也成为一个可选的"动作"，
        于是"倒车 + 转向"的组合路径会被自动找出来
      · 输出保持【网格点中心】序列 —— 直角转弯交给 path_executor 里
        现成的 PIVOT 三段式原地转向逻辑处理（不重复造轮子）

--------------------------------------------------------------------------------
用法
--------------------------------------------------------------------------------
  # 用配置中心里的实车坐标直接规划
  ./global_planner.py --config ~/dzrobot/config/robot.yaml

  # 手动指定 20 个点
  ./global_planner.py \\
      --start M10 --end N5 \\
      --mines A1 F4 C5 J5 E7 B9 J9 D10 \\
      --obstacles B2 D3 A5 G6 E9 B10 H10 F11 \\
      --targets F1 G1 --bridges K7 K1

  # 只看规划结果，不打印 A* 细节
  ./global_planner.py --quiet

  # 把结果写成可直接下发的路线字符串
  ./global_planner.py --emit-route
================================================================================
"""
from __future__ import annotations

import argparse
import heapq
import itertools
import math
import os
import sys
from typing import Dict, List, Optional, Sequence, Set, Tuple

# =============================================================================
#  地图常量（与 path_executor.py 严格一致，改一处必须改两处）
# =============================================================================
COL_LETTERS = "ABCDEFGHIJKLMN"      # 14 列，A(西) .. N(东)
MAP_COLS = 14
MAP_ROWS = 11                       # 行 1(远) .. 行 11(近)
BLOCK_SIZE = 0.35                   # 格边长(m)

# ★挡板约束：第 6 行与第 7 行之间，横跨 D~N 列，强制不可通行。
#   含义是"跨越 6<->7 行"这个动作在 D~N 列被禁止 —— 不是禁止整列。
#   所以上下半场之间只有 A/B/C 三列能穿行。
BARRIER_ROW_HI = 6                  # 挡板上侧行号
BARRIER_ROW_LO = 7                  # 挡板下侧行号
BARRIER_COL_FROM = "D"              # 从 D 列开始封死（含 D）

# 通行为 1，不可通行为 0
PASSABLE, BLOCKED = 1, 0

# 阿克曼动作代价。倒车比前进贵（更慢、更难控制），转向有固定代价。
COST_FORWARD = 1.0
COST_TURN = 1.6                     # 一次 90° 原地/行进转向
COST_BACKWARD = 1.9                 # 倒车一格
COST_BACKWARD_TURN = 2.4            # 倒车中转向（比单纯倒车更贵）

# 第 2 层任务顺序的代价权重（单位：格）
W_DIRECTION = 0.55                  # 方向惩罚：连续同向行进有惯性优势，掉头代价高
W_OBSTACLE = 2.5                    # 障碍惩罚：路线贴障碍越近越危险
W_CROSS_ZONE = 4.0                  # 跨区惩罚：穿越挡板缺口很贵，应尽量避免来回穿
OBSTACLE_AVOID_R = 1                # 障碍影响半径（格）
TOP_K_LEI_ORDERS = 240             # 雷序初筛后进入第 2 阶段精算的条数
SHOOT_CAND_KEEP = 2                 # 每个靶标保留的候选射击点数（按"顺路"排序后取前几个）

# 方向索引：0=北(行号减小) 1=西(列号减小) 2=南(行号增大) 3=东(列号增大)
DIRS = [(-1, 0), (0, -1), (1, 0), (0, 1)]
DIR_NAME = ["北(+X)", "西(+Y)", "南(-X)", "东(-Y)"]
DIR_ARROW = ["↑", "←", "↓", "→"]


# =============================================================================
#  坐标工具
# =============================================================================
def parse_block(name: str) -> Tuple[int, int]:
    """'F4' -> (row=4, col=5)。同时兼容 'R:F4' / 'F4*' / 'F4,y' 这些后缀形式。"""
    s = str(name).strip().upper()
    if s.startswith("R:"):
        s = s[2:]
    s = s.rstrip("*")
    if "," in s:
        s = s.split(",", 1)[0]
    if len(s) < 2 or s[0] not in COL_LETTERS or not s[1:].isdigit():
        raise ValueError("非法坐标点: %r" % name)
    row = int(s[1:])
    col = COL_LETTERS.index(s[0])
    if not (1 <= row <= MAP_ROWS):
        raise ValueError("行号越界: %r (应为 1~%d)" % (name, MAP_ROWS))
    return row, col


def block_name(row: int, col: int) -> str:
    """(row, col) -> 'F4'"""
    return "%s%d" % (COL_LETTERS[col], row)


def block_to_meters(row: int, col: int) -> Tuple[float, float]:
    """与 path_executor.block_to_meters 完全一致。"""
    return ((10.5 - (row - 1)) * BLOCK_SIZE, (13.5 - col) * BLOCK_SIZE)


def manhattan(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# =============================================================================
#  第 1 层：拓扑网格与约束建模
# =============================================================================
class Grid:
    """14x11 通行网格 + 挡板跨越约束。"""

    def __init__(self, obstacles: Sequence[str]):
        # 通行数组：1=可通行，0=障碍
        self.cell = [[PASSABLE] * MAP_COLS for _ in range(MAP_ROWS + 1)]  # 下标直接用行号 1..11
        self.barrier_col_start = COL_LETTERS.index(BARRIER_COL_FROM)
        self.obstacles: Set[Tuple[int, int]] = set()

        for ob in obstacles:
            r, c = parse_block(ob)
            self.cell[r][c] = BLOCKED
            self.obstacles.add((r, c))

    # ---------- 基础查询 ----------
    def in_bounds(self, r: int, c: int) -> bool:
        return 1 <= r <= MAP_ROWS and 0 <= c < MAP_COLS

    def passable(self, r: int, c: int) -> bool:
        return self.in_bounds(r, c) and self.cell[r][c] == PASSABLE

    def barrier_cross_ok(self, r_from: int, r_to: int, c: int) -> bool:
        """跨越挡板检查：6<->7 行之间只有 A/B/C 列（列号 < 3）能过。"""
        if {r_from, r_to} == {BARRIER_ROW_HI, BARRIER_ROW_LO}:
            return c < self.barrier_col_start
        return True

    def zone_of(self, r: int) -> int:
        """0 = 上半场(行 1~6)，1 = 下半场(行 7~11)。用于跨区惩罚。"""
        return 0 if r <= BARRIER_ROW_HI else 1

    # ---------- 阿克曼动作集 ----------
    def actions(self, r: int, c: int, h: int):
        """产出 ((r2, c2, h2), cost, action_name)。

        动作集刻意做成"阿克曼能执行"的：
          · 前进一格（沿当前朝向）
          · 倒车一格（朝向后方的反方向）—— 这是阿克曼车唯一的"倒退"手段
          · 左转 / 右转 90°（原地不动改变朝向）—— 实车由 PIVOT 三段式完成
        """
        # 前进
        dr, dc = DIRS[h]
        r2, c2 = r + dr, c + dc
        if self.passable(r2, c2) and self.barrier_cross_ok(r, r2, c2):
            yield (r2, c2, h), COST_FORWARD, "前进"

        # 倒车（沿当前朝向的反方向后退一格，车头朝向不变）
        r2, c2 = r - dr, c - dc
        if self.passable(r2, c2) and self.barrier_cross_ok(r, r2, c2):
            yield (r2, c2, h), COST_BACKWARD, "倒车"

        # 左右转 90°（位置不变，只改朝向）
        for turn in (-1, +1):
            h2 = (h + turn) % 4
            yield (r, c, h2), COST_TURN, ("左转" if turn < 0 else "右转")

    # ---------- 障碍距离势场（供第 2 层代价评估用） ----------
    def obstacle_penalty(self, r: int, c: int) -> float:
        """离障碍越近代价越高；半径 OBSTACLE_AVOID_R 内线性衰减。"""
        best = OBSTACLE_AVOID_R + 1
        for (orow, ocol) in self.obstacles:
            d = max(abs(orow - r), abs(ocol - c))     # 切比雪夫距离（含斜向）
            if d < best:
                best = d
        if best > OBSTACLE_AVOID_R:
            return 0.0
        return W_OBSTACLE * (OBSTACLE_AVOID_R + 1 - best) / (OBSTACLE_AVOID_R + 1)

    def line_obstacle_penalty(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """两点直线上采样若干点，累加障碍势场。"""
        steps = max(abs(b[0] - a[0]), abs(b[1] - a[1])) or 1
        total = 0.0
        for i in range(steps + 1):
            t = i / steps
            r = int(round(a[0] + (b[0] - a[0]) * t))
            c = int(round(a[1] + (b[1] - a[1]) * t))
            total += self.obstacle_penalty(r, c)
        return total / (steps + 1)


# =============================================================================
#  第 3 层：阿克曼约束 A*
# =============================================================================
def astar(grid: Grid, start: Tuple[int, int, int], goal_cell: Tuple[int, int],
          max_expand: int = 60000):
    """带朝向的 A*。返回 (路径, 总代价)。

    路径 = [(r, c, h, action), ...]，action 是到达该状态所用的动作名。
    和普通 A* 的区别：状态里带朝向 h，所以"这个格子从哪个方向进来"是区分开的 ——
    这正是阿克曼车必须的，因为能否顺利拐下一个弯取决于当前车头朝向。
    """
    if not grid.passable(*goal_cell):
        return None, float("inf")

    def heuristic(r, c, h):
        # 距离下界 + 至少还要转几次弯的下界（乐观估计，保证 A* 最优性）
        d = manhattan((r, c), goal_cell)
        # 朝向若能直接指向目标则无需转向；否则至少转一次
        dr, dc = DIRS[h]
        need = 0 if (dr * (goal_cell[0] - r) + dc * (goal_cell[1] - c)) > 0 else 1
        return d * COST_FORWARD + need * COST_TURN * 0.5

    open_h = [(heuristic(*start), 0.0, start, None)]
    gscore: Dict[Tuple[int, int, int], float] = {start: 0.0}
    came: Dict[Tuple[int, int, int], Tuple[Optional[Tuple[int, int, int]], str]] = {start: (None, "起点")}
    expanded = 0

    while open_h:
        _, g, cur, _ = heapq.heappop(open_h)
        if g > gscore.get(cur, float("inf")) + 1e-9:
            continue
        if (cur[0], cur[1]) == goal_cell:
            # 回溯
            path = []
            node = cur
            while node is not None:
                prev, act = came[node]
                path.append((node[0], node[1], node[2], act))
                node = prev
            path.reverse()
            return path, g

        expanded += 1
        if expanded > max_expand:
            return None, float("inf")

        r, c, h = cur
        for nxt, cost, act in grid.actions(r, c, h):
            ng = g + cost
            if ng < gscore.get(nxt, float("inf")) - 1e-9:
                gscore[nxt] = ng
                came[nxt] = (cur, act)
                heapq.heappush(open_h, (ng + heuristic(*nxt), ng, nxt, None))

    return None, float("inf")


def path_to_waypoints(path, start_heading: int = 0, force_keep=None):
    """把 A* 状态路径压成航点序列。

    返回 [(block, heading, action), ...]，并标记哪些段需要倒车。
    倒车的判定：某一步的【移动方向】与当时车头朝向相反 —— 就是倒车。
    压实规则：同一朝向的连续前进/倒车合并成一段，只在转向处切分。

    ★ force_keep：这些格子【绝不能】被压缩掉。
      踩过的坑：两个靶标的射击点可能是相邻格（例如 F1 和 F2），
      压在一条直线上就会被当成"中间点"删掉 —— 结果车里少停一次、
      少打一个靶。所以射击点/桥洞必须强制保留。
    """
    force_keep = set(force_keep or ())
    if not path:
        return []

    out = []           # [(row, col, heading, action)]
    for i, (r, c, h, act) in enumerate(path):
        if i == 0:
            out.append((r, c, h, "起点"))
            continue
        pr, pc, ph, _ = path[i - 1]
        moved = (r, c) != (pr, pc)
        if not moved:
            # 原地转向：不产出航点（实车由 PIVOT 完成），但记录朝向
            if out:
                out[-1] = (out[-1][0], out[-1][1], h, out[-1][3])
            continue
        # 判断这一步是前进还是倒车
        step_dir = (r - pr, c - pc)
        face_dir = DIRS[ph]
        is_back = (step_dir[0] * face_dir[0] + step_dir[1] * face_dir[1]) < 0
        out.append((r, c, ph, "倒车" if is_back else "前进"))

    # 压掉中间点，只保留"方向发生变化的拐点"
    if len(out) <= 2:
        return out
    keep = [out[0]]
    for i in range(1, len(out) - 1):
        prev, cur, nxt = out[i - 1], out[i], out[i + 1]
        d1 = (cur[0] - prev[0], cur[1] - prev[1])
        d2 = (nxt[0] - cur[0], nxt[1] - cur[1])
        same_move = (d1[0] * d2[1] - d1[1] * d2[0]) == 0 and (d1[0] * d2[0] + d1[1] * d2[1]) > 0
        same_mode = (prev[3] == cur[3] == nxt[3])
        must_keep = (cur[0], cur[1]) in force_keep
        if must_keep or not (same_move and same_mode):
            keep.append(cur)
    keep.append(out[-1])
    return keep


def convert_uturn_to_reverse(grid: Grid, waypoints):
    """把 180° 原地掉头自动改写成【倒车段】。

    ★ 为什么必须这么做（不是优化，是安全修正）：
      实车的原地转向（PIVOT）是"打死舵 + 慢慢前进"，走的是圆弧，不是原地旋转。
      转弯半径约 wheelbase/tan(30°) ≈ 0.27m，所以一次 180° 掉头会横向扫出约
      0.54m —— 比一格(0.35m)还宽。在地图角落（比如 A1）做这件事必然甩出边界。

      而阿克曼底盘本来就有更好的选择：**倒车**。倒着退过去即可，
      车头朝向完全不用变，零横向摆动，也不需要额外的空间。
      global_planner 的 A* 其实已经支持倒车动作，只是它按纯代价比较时
      选了"两次 90° 转向 + 前进"（8.2）而不是"全程倒车"（9.5）——
      代价模型低估了 180° 掉头的真实风险。这里做一次定向修正。

    判定条件（三者同时满足才改写，避免误伤正常的 90° 拐弯）：
      ① 折返角 > 150°（接近正反向）
      ② 前后两段共线（同一行或同一列，即纯直线折返）
      ③ 两段都是"前进"（倒车再折返的情况不管）
    """
    if len(waypoints) < 3:
        return list(waypoints), 0

    out = list(waypoints)
    n_fix = 0
    for i in range(1, len(out) - 1):
        pr, pc, ph, pact = out[i - 1]
        cr, cc, ch, cact = out[i]
        nr, nc, nh, nact = out[i + 1]

        # ③ 只处理两段都是前进的
        if pact != "前进" or cact != "前进":
            continue
        d1 = (cr - pr, cc - pc)
        d2 = (nr - cr, nc - cc)
        if d1 == (0, 0) or d2 == (0, 0):
            continue
        # ② 共线：叉积为 0 且点积为负（正反向）
        cross = d1[0] * d2[1] - d1[1] * d2[0]
        dot = d1[0] * d2[0] + d1[1] * d2[1]
        if cross != 0 or dot >= 0:
            continue
        # ① 角度接近 180°（共线反向必然满足，这里保留一个显式判据便于将来放宽）
        import math as _m
        ang = _m.degrees(_m.atan2(abs(cross), dot))
        if ang < 150.0:
            continue

        # 改写：把"从 C 出发去 N"这一段改成倒车
        # （车头朝向不变，倒着退到 N —— 与 R: 前缀的语义完全一致）
        out[i + 1] = (nr, nc, nh, "倒车")
        n_fix += 1

    return out, n_fix


# =============================================================================
#  第 2 层：任务顺序规划
# =============================================================================
def route_cost(grid: Grid, seq: Sequence[Tuple[int, int]],
               heads: Optional[Sequence[int]] = None) -> Tuple[float, list]:
    """评估一条访问序列的代价。

    代价 = 曼哈顿距离 + 方向惩罚 + 障碍惩罚 + 跨区惩罚

      · 方向惩罚：连续两段方向一致说明"顺路"，代价低；掉头则说明绕了远路
      · 障碍惩罚：航线上的障碍势场累加，逼规划器远离障碍桶
      · 跨区惩罚：跨越挡板缺口（6<->7 行）非常贵 —— 缺口只有 A/B/C 三列，
        来回穿会显著增加风险与里程，所以要尽量避免
    """
    total = 0.0
    detail = []
    prev_dir = None

    for i in range(len(seq) - 1):
        a, b = seq[i], seq[i + 1]
        d = manhattan(a, b)
        # 方向惩罚
        dr, dc = b[0] - a[0], b[1] - a[1]
        cur_dir = None
        if abs(dr) >= abs(dc) and dr != 0:
            cur_dir = 0 if dr < 0 else 2
        elif dc != 0:
            cur_dir = 1 if dc < 0 else 3
        dir_pen = 0.0
        if prev_dir is not None and cur_dir is not None:
            if cur_dir != prev_dir:
                # 掉头最贵，转 90° 次之
                diff = abs(cur_dir - prev_dir)
                dir_pen = W_DIRECTION * (2.0 if diff == 2 else 1.0)
        prev_dir = cur_dir if cur_dir is not None else prev_dir

        # 障碍惩罚
        obs_pen = grid.line_obstacle_penalty(a, b) * max(1, d)

        # 跨区惩罚
        cross_pen = 0.0
        if grid.zone_of(a[0]) != grid.zone_of(b[0]):
            cross_pen = W_CROSS_ZONE
            # 过界列越靠西越安全（缺口只有 A/B/C，B 列余量最大）
            if b[1] >= grid.barrier_col_start:
                cross_pen *= 3.0

        seg = d + dir_pen + obs_pen + cross_pen
        total += seg
        detail.append(seg)

    return total, detail


def greedy_insert(grid: Grid, base: List[Tuple[int, int]],
                  inserts: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """把 inserts 依序逐点插到【当前代价最低的位置】。

    ★ 这里刻意用贪心而不是全排列：4 个点全排列x位置组合 = 24x15 种/点，
      再乘上 240 条候选雷序，就是 40 万次代价评估，实测要跑几分钟 ——
      对一个"出车前跑一次"的工具来说太慢了。
      改贪心后降到约 1/20 的评估量，而由于靶/桥洞只有 4 个点、
      位置约束又很强（桥洞就在 K 列、靶在角落），贪心解与最优解基本一致。
      真正需要穷举的"8 个雷的顺序"仍然在阶段 1 全排列枚举，没有打折。
    """
    cur = list(base)
    for pt in inserts:
        best_i, best_c = 0, float("inf")
        for i in range(len(cur) + 1):
            c, _ = route_cost(grid, cur[:i] + [pt] + cur[i:])
            if c < best_c:
                best_c, best_i = c, i
        cur = cur[:best_i] + [pt] + cur[best_i:]
    return cur


def candidate_shoot_points(grid: Grid, target: Tuple[int, int]) -> List[Tuple[int, int]]:
    """靶标周围坐标作为候选射击点。

    射击点必须满足：
      · 可通行（不是障碍、不出界）
      · 与靶标相邻（切比雪夫距离 <= 1，即八邻域）—— 太远云台视场覆盖不到
      · 不是靶标本身（车不能骑在靶上）
    若八邻域全被占，则退化为"曼哈顿距离 2"的点，保证一定有解。
    """
    tr, tc = target
    cands = []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            r, c = tr + dr, tc + dc
            if grid.passable(r, c):
                cands.append((r, c))
    if cands:
        return cands

    for r in range(1, MAP_ROWS + 1):
        for c in range(MAP_COLS):
            if grid.passable(r, c) and manhattan((r, c), (tr, tc)) == 2:
                cands.append((r, c))
    return cands


# ★ 射击点的"朝向代价"权重（readme【九】第28条）。
#   同列 = 0.0 ：靶在车头正前/正后，车头正对靶，云台几乎不用转角 → 最优
#   同行 = 0.8 ：靶在车身侧面，云台要转约 90°
#   对角 = 2.4 ：又慢又容易超出云台 ±90° 行程 → 最差
#   数值来源是"车体转向 + 云台转角"的粗略总和，量纲与下面的格数代价可比。
SHOOT_ALIGN_SAME_COL = 0.0
SHOOT_ALIGN_SAME_ROW = 0.8
SHOOT_ALIGN_DIAGONAL = 2.4


def shoot_alignment_penalty(cd: Tuple[int, int], target: Tuple[int, int]) -> float:
    """候选射击点相对靶标的朝向代价（越小越优先）。

    ★ 这条权重是 2026-10-03 补上的（修 #75）：原先排序只看"离雷序折线多远"，
      完全没考虑云台要转多少角度，于是把【对角】点(0.495m)排到了【同列】点(0.35m)
      前面 —— 而对角点其实既超出射击距离上限、又最难瞄。readme 第28/29条
      早就写明"优先同列，示例最佳点就是 G10 而不是对角的 H10"。
    """
    if cd[1] == target[1]:
        return SHOOT_ALIGN_SAME_COL      # 同列：车头正对靶
    if cd[0] == target[0]:
        return SHOOT_ALIGN_SAME_ROW      # 同行：靶在侧面
    return SHOOT_ALIGN_DIAGONAL          # 对角：最差


def shoot_distance_m(cd: Tuple[int, int], target: Tuple[int, int]) -> float:
    """两格中心之间的距离(m)。正交相邻 = 0.35；对角相邻 = 0.35*√2 ≈ 0.495。"""
    return math.hypot(cd[0] - target[0], cd[1] - target[1]) * BLOCK_SIZE


def _rank_shoot_points(grid: Grid, target: Tuple[int, int],
                       lei_order: Sequence[Tuple[int, int]],
                       dist_min: float = 0.20, dist_max: float = 0.45):
    """把靶标的候选射击点按"综合代价"排序。

    代价 = 顺路代价(到雷序折线的最近距离)
         + 障碍贴边代价
         + ★朝向代价(同列/同行/对角，见上方权重说明)

    ★ 距离硬约束：候选点与靶标的中心距必须落在 [dist_min, dist_max]
      （真源 = config/robot.yaml 的 shoot_distance_min / shoot_distance_max，
      由 main 读进来传入，不在这里另写一份），超出直接淘汰。
      正交相邻 0.35m 合格，对角相邻 0.495m 超限 —— 这正是 #75 选错的直接原因。
    """
    cands = candidate_shoot_points(grid, target)
    if not cands or not lei_order:
        return cands

    # ---- 距离硬过滤 ----
    in_range = [cd for cd in cands
                if dist_min - 1e-6 <= shoot_distance_m(cd, target) <= dist_max + 1e-6]
    # 全被过滤掉（靶贴边等极端布局）就退回原候选集 —— 宁可超限也要有解，
    # 否则 plan_tour 会因为拿不到射击点而整条路线作废。
    pool = in_range or cands

    scored = []
    for cd in pool:
        cost = min(manhattan(cd, p) for p in lei_order)   # 顺路
        cost += grid.obstacle_penalty(*cd)                  # 别贴着障碍打
        cost += shoot_alignment_penalty(cd, target)        # ★朝向（主权重）
        scored.append((cost, cd))
    scored.sort(key=lambda x: x[0])
    return [cd for _, cd in scored]


def plan_tour(grid: Grid, start: Tuple[int, int], end: Tuple[int, int],
              mines: List[Tuple[int, int]], targets: List[Tuple[int, int]],
              bridges: List[Tuple[int, int]], verbose: bool = True,
              shoot_limits: Tuple[float, float] = (0.20, 0.45)):
    """主规划流程。返回 (最优序列, 元信息)。"""

    # ---- 阶段 1：8 个雷的全排列枚举（8! = 40320）----
    if verbose:
        n = math.factorial(len(mines))
        print("  [阶段1] 雷序全排列枚举：%d 个雷 -> %d 种排列" % (len(mines), n))

    scored = []
    for perm in itertools.permutations(mines):
        seq = [start] + list(perm) + [end]
        c, _ = route_cost(grid, seq)
        scored.append((c, perm))
    scored.sort(key=lambda x: x[0])

    if verbose:
        print("         初筛完成，取前 %d 名进入精算" % TOP_K_LEI_ORDERS)
        print("         最优雷序(初筛)：%s  代价=%.1f" %
              (" ".join(block_name(*p) for p in scored[0][1]), scored[0][0]))

    # ---- 阶段 2：把 2 靶 + 2 桥洞最优插入，再结合候选射击点 ----
    if verbose:
        print("  [阶段2] 靶/桥洞最优插入 + 射击点选择")

    best = None
    for rank, (base_cost, perm) in enumerate(scored[:TOP_K_LEI_ORDERS]):
        if verbose and rank and rank % 60 == 0:
            print("         已精算 %d/%d 条候选雷序..." % (rank, TOP_K_LEI_ORDERS))

        # 每个靶标只留最靠近该雷序折线的 SHOOT_CAND_KEEP 个射击点 ——
        # 八邻域最多 8 个候选，全组合是 64 种；按"是否顺路"筛掉绕远的即可。
        cand_sets = [_rank_shoot_points(grid, t, perm, shoot_limits[0],
                                        shoot_limits[1])[:SHOOT_CAND_KEEP]
                     for t in targets]
        if any(not cs for cs in cand_sets):
            continue

        for combo in itertools.product(*cand_sets):
            # 靶与桥洞的先后顺序不固定，两种次序都试一遍（比全排列省 12 倍）
            for order in (list(combo) + list(bridges),
                          list(bridges) + list(combo)):
                seq = greedy_insert(grid, [start] + list(perm), order) + [end]
                c, _ = route_cost(grid, seq)
                if best is None or c < best[0]:
                    best = (c, seq, perm, combo)

    if best is None:
        return None, {}

    cost, seq, lei_order, shoot_pts = best
    meta = {
        "cost": cost,
        "lei_order": lei_order,
        "shoot_points": shoot_pts,
        "targets": targets,
        "bridges": bridges,
    }
    return seq, meta


# =============================================================================
#  序列 -> 阿克曼路径 -> 路线字符串
# =============================================================================
def build_full_path(grid: Grid, seq: Sequence[Tuple[int, int]],
                    start_heading: int = 0, verbose: bool = True):
    """对相邻访问点逐段跑 A*，拼成完整的阿克曼路径。"""
    seg_paths = []
    heading = start_heading
    total_cost = 0.0

    for i in range(len(seq) - 1):
        a, b = seq[i], seq[i + 1]
        p, c = astar(grid, (a[0], a[1], heading), b)
        if p is None:
            if verbose:
                print("  ⚠️  A* 失败: %s -> %s" % (block_name(*a), block_name(*b)))
            continue
        total_cost += c
        seg_paths.append(p)
        heading = p[-1][2]

    # 拼接（去掉重复的接缝状态）
    full = []
    for p in seg_paths:
        if full and p and full[-1][:3] == p[0][:3]:
            full.extend(p[1:])
        else:
            full.extend(p)

    return full, total_cost


def path_to_route_string(grid: Grid, waypoints, shoot_points, bridges,
                         final_heading: Optional[str] = None) -> str:
    """航点 -> path_executor 可解析的路线字符串。

    输出约定（与 follow_route_command 完全兼容）：
        'B5'       普通航点
        'R:B5'     ★倒车航点（车倒着走到 B5）
        'B5*'      射击停站（车停在这里开火）
        ',x' / ',y' 末尾强制终点朝向

    ★ '*' 打在【射击点】上，而不是靶标本身。
      原因是车不可能骑到靶标上开火 —— 射击点是靶标旁边那个可通行的格子。
      规划器已经为每个靶标选好了射击点（见 candidate_shoot_points）。
    """
    shoot_cells = set(shoot_points)
    marked = set()          # ★只在【首次】到达时标 '*'
    toks = []

    for (r, c, h, act) in waypoints:
        name = block_name(r, c)
        # 踩过的坑：A* 路径可能两次经过同一个射击点（例如先西行到 A1 打点，
        # 再东行回 K1 时又路过 F1）。若每次经过都标 '*'，车会在同一个位置
        # 停两次、重复开火 —— 既浪费弹药，又会卡住路线进度。
        if (r, c) in shoot_cells and (r, c) not in marked:
            name += "*"
            marked.add((r, c))
        if act == "倒车":
            name = "R:" + name
        toks.append(name)

    route = " ".join(toks)
    if final_heading in ("x", "y"):
        route += "," + final_heading
    return route


# =============================================================================
#  配置中心读取
# =============================================================================
def _find_map_file(cfg_path):
    """地图文件位置（不能放在 robot.yaml 里 —— ROS params 文件结构约束）。"""
    cands = []
    if os.environ.get('ROBOT_MAP'):
        cands.append(os.environ['ROBOT_MAP'])
    if cfg_path:
        cands.append(os.path.join(os.path.dirname(os.path.abspath(cfg_path)),
                                  'map.yaml'))
    for c in cands:
        if c and os.path.exists(c):
            return c
    return None


def load_map_file(path: str):
    """从独立的 map.yaml 读地图要素。"""
    import yaml
    with open(path, encoding='utf-8') as fp:
        d = yaml.safe_load(fp) or {}
    m = d.get('map') or d
    return {
        "start": m.get("start"),
        "end": m.get("end"),
        "mines": list(m.get("mines") or []),
        "obstacles": list(m.get("obstacles") or []),
        "targets": list(m.get("targets") or []),
        "bridges": list(m.get("bridges") or []),
    }


def load_from_config(path: str):
    """从 robot.yaml 读取地图要素。

    当前配置中心只存了路线（mission_blocks），没有单独的分类要素表，
    所以这里从路线里反推：带 '*' 的是靶标，其余按需由调用方补充。
    如果配置里新增了 map: 段（推荐），则优先使用它。
    """
    import yaml
    with open(path, encoding="utf-8") as fp:
        d = yaml.safe_load(fp) or {}

    # 优先：显式的 map 段（旧写法，已不推荐 —— 会破坏 ROS params 文件结构）
    m = d.get("map") or {}
    if m:
        return {
            "start": m.get("start"),
            "end": m.get("end"),
            "mines": list(m.get("mines") or []),
            "obstacles": list(m.get("obstacles") or []),
            "targets": list(m.get("targets") or []),
            "bridges": list(m.get("bridges") or []),
        }

    # ★ 推荐：独立的 config/map.yaml
    mf = _find_map_file(path)
    if mf:
        return load_map_file(mf)

    # 回退：从路线反推
    p = (d.get("path_executor") or {}).get("ros__parameters", {}) or {}
    blocks = [str(b) for b in (p.get("mission_blocks") or [])]
    targets = [b.rstrip("*") for b in blocks if b.endswith("*")]
    return {
        "start": str(p.get("mission_reset", "M10,y")).split(",")[0],
        "end": blocks[-1] if blocks else None,
        "mines": [], "obstacles": [], "targets": targets, "bridges": [],
    }


def load_shoot_limits(cfg_path):
    """读射击距离约束（真源 = config/robot.yaml 的 path_executor 段）。

    单独抽一个函数而不是塞进 load_from_config()，是为了让"地图要素"与
    "规划约束"两类来源保持分离，也避免动到既有调用方。
    读失败时回退到与 robot.yaml 一致的默认值（现场安全优先）。
    """
    try:
        import yaml
        with open(cfg_path, encoding='utf-8') as fp:
            d = yaml.safe_load(fp) or {}
        p = (d.get('path_executor') or {}).get('ros__parameters', {}) or {}
        return (float(p.get('shoot_distance_min', 0.20)),
                float(p.get('shoot_distance_max', 0.45)))
    except Exception:
        return (0.20, 0.45)


# =============================================================================
#  CLI
# =============================================================================
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="全局路径规划器（14x11 网格 + 挡板约束 + 阿克曼 A*）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.expanduser("~/dzrobot/config/robot.yaml"),
                    help="配置中心路径（读取地图要素）")
    ap.add_argument("--start", help="起点，如 M10")
    ap.add_argument("--end", help="终点，如 N5")
    ap.add_argument("--mines", nargs="*", default=[], help="8 个雷")
    ap.add_argument("--obstacles", nargs="*", default=[], help="8 个障碍")
    ap.add_argument("--targets", nargs="*", default=[], help="2 个靶标")
    ap.add_argument("--bridges", nargs="*", default=[], help="2 个桥洞")
    ap.add_argument("--start-heading", default="n",
                    choices=["n", "w", "s", "e"], help="起点车头朝向")
    ap.add_argument("--emit-route", action="store_true",
                    help="只输出路线字符串（便于管道传给 ros2 topic pub）")
    ap.add_argument("--quiet", action="store_true", help="少打印")
    args = ap.parse_args(argv)

    verbose = not args.quiet

    # ---- 取地图要素：命令行优先，缺的从配置中心补 ----
    cfg = {}
    if os.path.exists(args.config):
        try:
            cfg = load_from_config(args.config)
        except Exception as exc:
            print("  ⚠️  读取配置中心失败: %s" % exc, file=sys.stderr)

    start_s = args.start or cfg.get("start")
    end_s = args.end or cfg.get("end")
    mines_s = args.mines or cfg.get("mines") or []
    obs_s = args.obstacles or cfg.get("obstacles") or []
    tgt_s = args.targets or cfg.get("targets") or []
    brg_s = args.bridges or cfg.get("bridges") or []

    if not start_s or not end_s:
        print("❌ 缺少起点或终点（命令行 --start/--end，或配置中心 map 段）",
              file=sys.stderr)
        return 2
    if not mines_s:
        print("❌ 缺少雷点（--mines）。规划必须知道要访问哪些雷。", file=sys.stderr)
        return 2

    # ---- 第 1 层：建模 ----
    grid = Grid(obs_s)
    start = parse_block(start_s)
    end = parse_block(end_s)
    mines = [parse_block(x) for x in mines_s]
    targets = [parse_block(x) for x in tgt_s]
    bridges = [parse_block(x) for x in brg_s]

    if not args.emit_route:
        print("=" * 78)
        print("  全局路径规划器")
        print("=" * 78)
        print("  起点 %s  终点 %s  车头 %s" %
              (block_name(*start), block_name(*end), args.start_heading.upper()))
        print("  雷  %d: %s" % (len(mines), " ".join(block_name(*m) for m in mines)))
        print("  障  %d: %s" % (len(obs_s), " ".join(obs_s) if obs_s else "(无)"))
        print("  靶  %d: %s" % (len(targets), " ".join(block_name(*t) for t in targets)))
        print("  通  %d: %s" % (len(bridges), " ".join(block_name(*b) for b in bridges)))
        print("  挡板: 第%d~%d行之间、%s 列以东封死；只有 A/B/C 列能穿行"
              % (BARRIER_ROW_HI, BARRIER_ROW_LO, BARRIER_COL_FROM))
        print()

    # ---- 第 2 层：任务顺序 ----
    # 射击距离约束来自配置中心（不另写一份，见 load_shoot_limits 注释）
    shoot_limits = load_shoot_limits(args.config)
    if verbose:
        print("  射击距离窗 : %.2f ~ %.2f m" % shoot_limits)

    seq, meta = plan_tour(grid, start, end, mines, targets, bridges,
                          verbose=verbose, shoot_limits=shoot_limits)
    if seq is None:
        print("❌ 未找到可行序列", file=sys.stderr)
        return 1

    # ---- 第 3 层：阿克曼路径 ----
    heading_map = {"n": 0, "w": 1, "s": 2, "e": 3}
    full, path_cost = build_full_path(grid, seq, heading_map[args.start_heading],
                                      verbose=verbose)
    # ★★ 所有"任务点"都必须出现在航点序列里，不能被直线压缩吃掉。
    #   压缩规则(path_to_waypoints)会把"同向 + 同模式"的连续点合并成一条直线段，
    #   只在方向变化处保留拐点 —— 落在这条直线上的中间格会被直接删掉。
    #   踩过的坑（2026-10-03 修 C8 时发现）：原先 force_keep 只放了射击点和桥洞，
    #   结果 8 个雷里有 4 个（H1 / G2 / C4 / B5）恰好落在直线段上被吃掉，
    #   规划器内部算出的最优雷序是全覆盖的，输出的路线却只覆盖一半 —— 那些雷会被漏踩，
    #   而且【不报任何错】，极具迷惑性。
    #   所以雷点必须和射击点/桥洞同等对待；起点/终点本来就会被保留，一并写上更保险。
    shoot_pts = list(meta["shoot_points"])
    must_keep = set(shoot_pts) | set(bridges) | set(mines) | {start, end}
    wps = path_to_waypoints(full, heading_map[args.start_heading],
                            force_keep=must_keep)

    # ★把 180° 原地掉头改写成倒车段（安全修正，理由见函数注释）
    wps, n_uturn = convert_uturn_to_reverse(grid, wps)
    if n_uturn and not args.quiet:
        print()
        print("  ⚠️  检测到 %d 处 180° 原地掉头，已自动改写为【倒车段】：" % n_uturn)
        print("     原因：原地转向走圆弧，180° 会横向扫出约 0.54m（> 一格 0.35m），")
        print("           在地图角落必然甩出边界。倒车则零横向摆动，更安全。")

    route = path_to_route_string(grid, wps, shoot_pts, bridges)

    if args.emit_route:
        print(route)
        return 0

    # ---- 报告 ----
    print()
    print("-" * 78)
    print("  第 2 层 · 访问顺序")
    print("-" * 78)
    print("  序列代价 : %.1f" % meta["cost"])
    print("  访问序列 : %s" % " ".join(block_name(*p) for p in seq))
    print("  靶标射击点: %s" %
          "  ".join("%s->%s" % (block_name(*t), block_name(*s))
                    for t, s in zip(meta["targets"], meta["shoot_points"])))

    print()
    print("-" * 78)
    print("  第 3 层 · 阿克曼路径（%d 个航点，A* 代价 %.1f）" % (len(wps), path_cost))
    print("-" * 78)
    print("   #   点位   朝向      动作")
    for i, (r, c, h, act) in enumerate(wps):
        mark = "  ★倒车" if act == "倒车" else ""
        print("  %2d   %-5s  %-8s  %s%s" %
              (i + 1, block_name(r, c), DIR_NAME[h], act, mark))

    n_back = sum(1 for w in wps if w[3] == "倒车")
    print()
    print("-" * 78)
    print("  路线字符串（可直接下发）")
    print("-" * 78)
    print("  " + route)
    print()
    print("  统计: 航点 %d 个 | 倒车段 %d 个 | 挡板穿越 %d 次" %
          (len(wps), n_back,
           sum(1 for i in range(len(seq) - 1)
               if grid.zone_of(seq[i][0]) != grid.zone_of(seq[i + 1][0]))))
    print()
    print("  下发命令:")
    print("    ros2 topic pub /nav/waypoint std_msgs/msg/String \\")
    print("        \"{data: '%s'}\" --once" % route)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
