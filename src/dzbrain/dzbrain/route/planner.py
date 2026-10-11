# -*- coding: utf-8 -*-
"""路线规划器 —— **换一张 `map.yaml` 就能自动生成路线**，不再手写 token。

用户诉求原话："我只要把地图点位（map.yaml）替换掉就行了，小车会自己算路径然后跑起来。"
本模块就是那个"自己算"的部分；它只读 `map.yaml` 的六个字段
（`start/end/mines/obstacles/targets/bridges`）+ `Config` 里的几个约束值，
**不复制任何一份地图要素**（真源唯一性，见 `validate.py` 记录的 K13 教训）。

--------------------------------------------------------------------------------
赛道模型（`config/map.yaml` 顶部注释 + `robot.yaml` 的 `route` / `bridge_order`）
--------------------------------------------------------------------------------
整个 K 列是一座桥、每格都是一个桥洞；第 6/7 行之间有一条横跨 D~N 列的隔板，
只有 A/B/C 列能上下穿行。于是合法路线只能是一条 **U 形**：

    start（下半场，桥东侧）→ 穿下半场桥洞（**向西**）→ 压雷/避障/打靶 →
    绕隔板北上（A/B/C 列）→ 穿上半场桥洞（**向东**）→ end（上半场，桥东侧）

本模块把这个 U 形结构**显式建模**，不指望 A* 自己摸出来：

1. 按行号把必访点分到上/下半场。`map.bridges` 只是"本局识图结果"，
   真正的候选集合是 `Config` 的 `route.cross_lower/upper_candidates`
   （每局随机、由无人机情报选格），缺省才回落到 `map.bridges`，再缺省才用整列。
2. 顺序固定为 `start → 下半场节点 → K(lower) → 上半场节点 → K(upper) → end`，
   结构上保证 `validate.py` 的 R007（每半场**恰好**穿一次、方向正确）不可能被破坏
   —— 不靠"搜出来碰巧对"。
3. 半场内顺序用"最近邻 + 2-opt"，代价来自**带朝向的 A\\***；而且**朝向本身也参与优化**
   （见下节）—— 这条是必需的，否则会出现"到达桥洞时车头朝东、下一步却要往西"的
   180° 折返。

--------------------------------------------------------------------------------
为什么顺序优化里必须同时优化"朝向"
--------------------------------------------------------------------------------
阿克曼车不能横移，所以"从哪个方向进某个格"直接决定下一段能不能顺畅出发。
第一版只按"格到格的最短代价"排序，结果生成出 `I10 -> I9 -> K9 -> J9`：车从东边
开进 K9 后**原地 180° 折返**再往东走 —— 一条能过校验、但现场很蠢的路线。
现在把朝向作为状态的一部分：

* `cost[(a, b)]` = 从 a 的**四个朝向**出发、到达 b 的**四个朝向**的最小代价（顺序轮用）；
* `h_cost[(a, b)][ha][hb]` = 带朝向的精确代价（朝向轮用）。

两者交替优化（顺序 → 朝向 → 顺序 …，最多 3 轮），起点朝向按 `mission.reset` 锁死。
2-opt 反转子段时，两端的朝向选择会自动重新评估（成本函数里每次都取当前朝向）。

--------------------------------------------------------------------------------
三个必须显式处理的坑（都有现场依据）
--------------------------------------------------------------------------------
① **K 列只在桥洞行可穿越**。`validate.py` 的 R007 要求"每半场**恰好**一次穿越 K 列，
   所以规划期就把"在非桥洞行进出 K 列"直接拉黑。否则 A* 会为了抄近路从
   `I10 -> K10 -> J10` 蹭一下 K 列 —— 那会多算一次穿越，行为上也是撞侧壁。
   同时拉黑"K 列内上下挪格"（R009：桥洞只有单格宽）与"K 邻列斜切"（R009 的 10° 上限）。

② **180° 折返必须改写成倒车段**（`R:` 前缀）。
   实车原地转向是"打死舵 + 慢慢前进"，走的是圆弧；180° 掉头会横向扫出约 0.54 m
   —— 比一格(0.35 m)还宽，在地图角落必然甩出边界。倒车零横向摆动，是阿克曼车处理
   死角的正确解法，也是现有手写路线里那 3 段 `R:` 的来历。
   编译阶段检测"前进→前进 且折返角 > 150°"并改写，随后 `assert_legal` **断言**
   结果里不再存在任何"同模式 180° 折返"；改写后车头朝向**重算**
   （倒车掉头与原地掉头的车头正好相反，弄错会让终点姿态与云台基准全错）。

③ **射击停站必须落在真能打到靶的位置**。
   距离判据（`0.20~0.45 m`）**复用现有函数** `validate._euc`
   （= `shoot_planner.dist_m` = `route_check` 那条规则的算法），不另写一套；
   候选点 = 各靶格的 4 个正交邻居（`targets` 是**相连两格** = 一处轨道靶区，
   所以"一个点同时服务两格"的位置天然在候选里 —— 手写路线的 `G10` 正是这种点）。

--------------------------------------------------------------------------------
与现有校验器的关系
--------------------------------------------------------------------------------
本模块**不复制、不放宽**任何规则：结构判据直接复用 `validate.py` 的
`_crossing_at_column` / `_barrier_crossing` / `_euc` / `_stop_ok`，
朝向与代价语义对齐 `tools/global_planner.py`。生成结果在
`tools/plan_route.py --check` 里再跑一遍 `tools/route_check.py`（0 error 才算通过）。
两套校验器口径若冲突，本模块**如实抛错并贴出规则原话**，绝不擅自放宽。
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

from ..core import geometry as geo
from ..core.config import Config
from . import validate as V
from .model import Route, RouteBuilder, Waypoint
from .parser import PIVOT_SUFFIX as _PIVOT_SUFFIX, parse_reset, parse_token

__all__ = [
    "PlannerError",
    "PlanResult",
    "MapSpec",
    "Grid",
    "plan_route",
    "load_map",
    "map_section",
    "assert_legal",
    "assert_legal_no_reverse",
    "check_route_structure",
    "shoot_candidates",
    "stop_distance_m",
    "turn_angle_deg",
    "route_tokens",
    "pivot_vertices",
    "mark_pivots",
    "DIRS",
    "DEFAULT_MAX_SECONDS",
    "PIVOT_MARK_DEG",
    "PIVOT_PENALTY",
    "MINE_PRESS_TOL_M",
    "MINE_TURN_PENALTY",
    "straighten_mines",
]

# --------------------------------------------------------------------------- #
#  常量
# --------------------------------------------------------------------------- #
#: 朝向索引：0=北(行号减小) 1=西(列号减小) 2=南(行号增大) 3=东(列号增大)。
#: 与 `tools/global_planner.py` 的 `DIRS` 逐字一致 —— 现场语义已经这么用；
#: 换算成 `geometry` 的地图 yaw：0/2 → `HEADING_X`、1/3 → `HEADING_Y`。
DIRS: tuple[tuple[int, int], ...] = ((-1, 0), (0, -1), (1, 0), (0, 1))
TURN_LEFT, TURN_RIGHT = -1, +1
HEAD_NAMES = ("北(行号减小)", "西(列号减小)", "南(行号增大)", "东(列号增大)")

#: 动作编号（`flood` 回传，用于判断某一段是否倒车）
ACT_START, ACT_FORWARD, ACT_BACKWARD, ACT_LEFT, ACT_RIGHT = 9, 0, 1, 2, 3

#: 阿克曼动作代价：倒车比前进贵（更慢、更难控制），转向有固定代价。
COST_FORWARD = 1.0
COST_BACKWARD = 2.2
COST_TURN = 1.4

#: 障碍安全势场：切比雪夫距离 1 / 2 两档加罚，逼路线离水桶远一点。
#: 罚值刻意"软"而不是拉黑 —— 现场 A6/C4 这类雷点本身就贴着障碍，
#: 硬拉黑会直接无解；软罚只影响"绕不绕得开"的取舍。
OBSTACLE_DANGER_RADIUS = 2
OBSTACLE_DANGER_W1 = 0.9
OBSTACLE_DANGER_W2 = 0.4
#: 隔板相邻行的额外罚（缺口只有 A/B/C 三列，别贴着隔板擦）
BARRIER_NEAR_COST = 0.15

#: 编译阶段的倒车改写阈值（度）：>150° 才算"折返"，正常 90° 拐弯绝不动它。
UTURN_REVERSE_DEG = 150.0

#: ★ 折返点（`:PIVOT`）的判定阈值（度）。取值 = `pursuit.pivot_angle_deg`
#: （`core/config.py` 默认 135°，真车已实测完成），判据与 `pursuit.path.corner_turn_deg`
#: 一致（顶点处**车头**需转过的角度）：相邻两段有倒车时车头一度都不用转，不算折返。
#: 现场无倒车路线实测只有两处：`F8` / `D6`（都是"开进死格再原路退出"的压雷死角）。
PIVOT_MARK_DEG = 135.0
#: ★ TSP/顺序搜索里的**折返惩罚**（等效代价单位）：
#: 访问顺序里一处"进去再原路退出来"的顶点按 5.0 计（= 5 格前进代价）。
#: 为什么需要：无倒车模式下这种访问必然让车在顶点原地掉头（PIVOT 实测 ~10.8 s、
#: 横向扫出 ~0.54 m > 一格 0.35 m），比多绕两格贵得多 ——
#: 只按 A* 里程排序会把它当成"顺路"，于是折返点变成路线里最贵的动作却没被计价。
PIVOT_PENALTY = 5.0

#: ★★ 雷心必须被**车心**压过（用户 2026-10-11 口径）：参考折线到每个雷格中心的
#: 最近距离 ≤ 该容差（米）。实现见 `straighten_mines()` —— 把雷格做成"进—雷—出"
#: 三点直线的**中点**，纯追踪于是沿直线穿过雷心；不这么做的后果有实测依据：
#: ① 90° 拐角处的雷被切角"擦过"（t8 报告 §3.5：C5 差 3.1 cm、I8 差 2.7 cm）；
#: ② 180° 折返顶点上的雷更糟：`pursuit.pivot_dist_m=0.25` 让车在距顶点 0.25 m 处
#:    就掉头，车心**根本到不了**雷心（A9 实测差 12.7 cm）。
MINE_PRESS_TOL_M = 0.03

#: ★ 顺序层"雷格不得作为拐点"的惩罚权重（等效代价单位）：90° 拐点记 1 份、180° 记 8 份。
#: 1.0 份 ≈ 1 格前进代价 —— 权衡"多绕一格"与"雷被切角擦过"，后者要严重得多
#: （真车漏压雷 = 直接丢分），所以取 2.0 让排序优先满足压雷。
MINE_TURN_PENALTY = 2.0

#: 单次规划的时间预算（秒）。超了抛 `PlannerError`，绝不静默给半成品路线。
DEFAULT_MAX_SECONDS = 0.8
#: "顺序 ↔ 朝向"交替优化的轮数上限
MAX_TOUR_ROUNDS = 3
#: 2-opt 的收敛上限（防病态输入吃光时间预算）
MAX_2OPT_PASSES = 6
#: 相邻航点最大间距（格）。超过就插中间点：`validate.py` 的 R011 在 2.5 格处告警，
#: 而且 1.4 m 的长直线段在纯追踪下会切角，插点同时解决这两件事。
MAX_HOP_BLOCKS = 2
#: ★ 出车前校验器 `tools/route_check.py` 的**实际接受窗口**（该文件第 579 / 615-618 行）：
#:     ``SHOOT_DIST_MIN - 0.03 <= d <= SHOOT_DIST_MAX + 0.06``
#: 即 0.17 ~ 0.51 m。现场手写路线的 `G10*` 正是靠这个容差才算"同时覆盖 G11 与 H11"
#: （`G10`→`G11` 0.35 m、`G10`→`H11` 0.495 m > 0.45 m 的标称上限）。
#: 本规划器把它作为**唯一真相的接受判据**（显式写出，不是偷偷放宽）：规划器自己不发明
#: 阈值，接受窗口由"最终会被执行的那条校验规则"决定。
ROUTE_CHECK_TOL_LO = 0.03
ROUTE_CHECK_TOL_HI = 0.06

_CROSS_COL = geo.COL_LETTERS.index(V.CROSS_COLUMN)


class PlannerError(RuntimeError):
    """规划失败（无解 / 与校验器冲突 / 超出时间预算）。**绝不静默降级。**"""


# --------------------------------------------------------------------------- #
#  地图
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class MapSpec:
    """`map.yaml` 的 `map:` 段（只取规划需要的字段，**不复制**要素真源）。"""

    name: str
    start: str
    end: str
    mines: tuple[str, ...]
    obstacles: tuple[str, ...]
    targets: tuple[str, ...]
    bridges: tuple[str, ...]
    barrier_col_from: str = V.BARRIER_COL_FROM
    barrier_row_hi: int = V.BARRIER_ROW_HI
    barrier_row_lo: int = V.BARRIER_ROW_LO
    source: str = ""


def map_section(map_cfg: Any) -> dict:
    """取 `map:` 段 —— 直接复用 `validate._map_section`（口径必须与校验器一致）。"""
    return V._map_section(map_cfg)                     # noqa: SLF001  同包既有实现


def _as_cells(value: Any) -> tuple[str, ...]:
    """列表归一化（大写、去空、保序）—— 复用 `validate._cells` 的规则。"""
    return tuple(V._cells({"value": value}, "value"))  # noqa: SLF001


def _checked(cells: Iterable[str], group: str) -> tuple[str, ...]:
    for cell in cells:
        try:
            geo.block_to_index(cell)
        except ValueError as exc:
            raise PlannerError(f"地图 {group} 里的格名非法: {exc}") from exc
    return tuple(cells)


def load_map(map_cfg: Any, *, source: str = "") -> MapSpec:
    """`map.yaml` 原始 dict（或已展平 dict / `Config`）-> `MapSpec`。

    缺失/非法一律抛 `PlannerError`，**绝不猜一个默认起点** —— 起点错了整条路线都错位。
    """
    merged = map_section(map_cfg)
    if not merged:
        raise PlannerError("地图里没有 map: 段（或为空），无法规划")

    start = str(merged.get("start") or "").strip().upper()
    end = str(merged.get("end") or "").strip().upper()
    if not start or not end:
        raise PlannerError("地图缺少 map.start / map.end")
    _checked((start, end), "start/end")

    barrier_col = str(merged.get("barrier_col_from") or V.BARRIER_COL_FROM).strip().upper()
    if barrier_col not in geo.COL_LETTERS:
        raise PlannerError(f"map.barrier_col_from 非法: {barrier_col!r}")

    def _int(key: str, fallback: int) -> int:
        try:
            return int(merged.get(key, fallback))
        except (TypeError, ValueError):
            return fallback

    return MapSpec(
        name=str(merged.get("name") or ""), start=start, end=end,
        mines=_checked(_as_cells(merged.get("mines")), "mines"),
        obstacles=_checked(_as_cells(merged.get("obstacles")), "obstacles"),
        targets=_checked(_as_cells(merged.get("targets")), "targets"),
        bridges=_checked(_as_cells(merged.get("bridges")), "bridges"),
        barrier_col_from=barrier_col,
        barrier_row_hi=_int("barrier_row_hi", V.BARRIER_ROW_HI),
        barrier_row_lo=_int("barrier_row_lo", V.BARRIER_ROW_LO),
        source=source,
    )


# --------------------------------------------------------------------------- #
#  栅格与代价
# --------------------------------------------------------------------------- #
class Grid:
    """通行栅格：障碍占格 + 隔板跨越约束 + K 列桥洞约束 + 障碍安全势场。

    坐标一律用 ``(row, col)``：``row`` 是 1-based 行号，``col`` 是 0-based 列索引
    （与 `geometry.block_to_index` 的返回值口径对齐）。
    """

    def __init__(self, spec: MapSpec, bridge_rows: Iterable[int] = (),
                 *, allow_reverse: bool = True) -> None:
        self.spec = spec
        #: 无倒车模式（真车底盘不支持倒车）：`_moves` 不再产出倒车动作，
        #: 180° 折返 = 正向开进死格、原地转向（2×COST_TURN）后正向开出 ——
        #: 顶点处由 TrackerTask 自动 PIVOT。默认 True = 旧行为（允许倒车）。
        self.allow_reverse = bool(allow_reverse)
        self.blocked: set[tuple[int, int]] = set()
        for cell in spec.obstacles:
            row_idx, col_idx = geo.block_to_index(cell)
            self.blocked.add((row_idx + 1, col_idx))
        self.barrier_col = geo.COL_LETTERS.index(spec.barrier_col_from)
        self.barrier_hi = spec.barrier_row_hi
        self.barrier_lo = spec.barrier_row_lo
        self.bridge_rows = {int(r) for r in bridge_rows}
        self._danger = self._build_danger()

    # ---------------- 查询 ----------------
    def in_bounds(self, row: int, col: int) -> bool:
        return 1 <= row <= geo.MAP_ROWS and 0 <= col < geo.MAP_COLS

    def passable(self, row: int, col: int) -> bool:
        return self.in_bounds(row, col) and (row, col) not in self.blocked

    def barrier_cross_ok(self, row_from: int, row_to: int, col: int) -> bool:
        """跨越第 hi/lo 行之间的隔板：**只有列索引 < barrier_col_from 的列**能过。"""
        if {row_from, row_to} == {self.barrier_hi, self.barrier_lo}:
            return col < self.barrier_col
        return True

    def zone_of(self, row: int) -> int:
        """0 = 上半场（row <= barrier_row_hi），1 = 下半场。"""
        return 0 if row <= self.barrier_hi else 1

    def move_ok(self, row: int, col: int, nrow: int, ncol: int) -> bool:
        """一步是否合法。

        四条判据（每条都对应一条现场硬约束）：
         ① 目标格可通行（障碍不可通行）；
         ② 不非法跨越第 hi/lo 行隔板（只有 A/B/C 列能上下穿行）；
         ③ K 列内不上下挪格（R009：桥洞单格宽，必撞侧壁）；
         ④ **只在桥洞行进出 K 列**（R007：每半场恰好一次穿越；非桥洞行进出会多算穿越，
            行为上也是撞桥墩）。K 邻列的斜切同时被本条拦掉（R009 的 10° 上限）。
        """
        if not self.passable(nrow, ncol):
            return False
        if not self.barrier_cross_ok(row, nrow, ncol):
            return False
        drow, dcol = nrow - row, ncol - col
        if col == _CROSS_COL and ncol == _CROSS_COL and drow != 0:
            return False
        crosses_cross_col = (col - _CROSS_COL) * (ncol - _CROSS_COL) <= 0 and col != ncol
        if crosses_cross_col:
            row_at_cross = nrow if ncol == _CROSS_COL else row
            if row_at_cross not in self.bridge_rows:
                return False
        return True

    def danger(self, row: int, col: int) -> float:
        return self._danger[row][col]

    def _build_danger(self) -> list[list[float]]:
        grid = [[0.0] * geo.MAP_COLS for _ in range(geo.MAP_ROWS + 1)]
        for (orow, ocol) in self.blocked:
            for drow in range(-OBSTACLE_DANGER_RADIUS, OBSTACLE_DANGER_RADIUS + 1):
                for dcol in range(-OBSTACLE_DANGER_RADIUS, OBSTACLE_DANGER_RADIUS + 1):
                    row, col = orow + drow, ocol + dcol
                    if not self.in_bounds(row, col):
                        continue
                    cheb = max(abs(drow), abs(dcol))
                    if cheb == 0:
                        continue
                    value = OBSTACLE_DANGER_W1 if cheb == 1 else OBSTACLE_DANGER_W2
                    if value > grid[row][col]:
                        grid[row][col] = value
        for row in (self.barrier_hi, self.barrier_lo):
            if 1 <= row <= geo.MAP_ROWS:
                for col in range(geo.MAP_COLS):
                    grid[row][col] += BARRIER_NEAR_COST
        return grid


# --------------------------------------------------------------------------- #
#  带朝向的搜索（Dijkstra 洪泛）
# --------------------------------------------------------------------------- #
def _moves(grid: Grid, row: int, col: int, head: int):
    """``((row, col, head), cost, action)``：前进 / 倒车 / 左右转，阿克曼可执行的动作集。

    `grid.allow_reverse=False`（无倒车模式）时**不产出倒车动作**：真车底盘不支持倒车
    （负速度方向位被 STM32 无视），所以路线里任何依赖倒车的路径都必须改用
    "正向开进 + 原地转向 + 正向开出"（顶点处由 TrackerTask PIVOT）。
    """
    drow, dcol = DIRS[head]
    nrow, ncol = row + drow, col + dcol
    if grid.move_ok(row, col, nrow, ncol):
        yield (nrow, ncol, head), COST_FORWARD + grid.danger(nrow, ncol), ACT_FORWARD
    if not grid.allow_reverse:
        yield (row, col, (head + TURN_LEFT) % 4), COST_TURN, ACT_LEFT
        yield (row, col, (head + TURN_RIGHT) % 4), COST_TURN, ACT_RIGHT
        return
    brow, bcol = row - drow, col - dcol
    if grid.move_ok(row, col, brow, bcol):
        yield (brow, bcol, head), COST_BACKWARD + grid.danger(brow, bcol), ACT_BACKWARD
    yield (row, col, (head + TURN_LEFT) % 4), COST_TURN, ACT_LEFT
    yield (row, col, (head + TURN_RIGHT) % 4), COST_TURN, ACT_RIGHT


def flood(grid: Grid, source: tuple[int, int, int]):
    """从 ``(row, col, heading)`` 出发的**一次** Dijkstra 洪泛。

    :return: ``(dist, came, actions, settled)``；``dist[(row, col, head)]`` = 最短代价，
        ``came`` 是前驱状态，``actions[(row, col, head)]`` = 到达该状态用的动作，
        ``settled`` = 实际弹出的状态数（搜索代价的读数，供日志/断言用）。

    ★ 为什么要洪泛而不是逐对 A*：代价矩阵要"一个点到其它所有点 × 四个到达朝向"，
      逐对 A* 是 16 次搜索/对，19 个点就是 2700+ 次 —— 直接吃光时间预算。
      状态空间只有 11×14×4 = 616，一次洪泛就能拿到全部朝向的距离，
      而且**仍然是最优解**（Dijkstra，不做启发式剪枝）。
    """
    counter = 0
    settled = 0
    heap: list[tuple[float, int, tuple[int, int, int]]] = [(0.0, counter, source)]
    dist: dict[tuple[int, int, int], float] = {source: 0.0}
    came: dict[tuple[int, int, int], tuple[int, int, int] | None] = {source: None}
    actions: dict[tuple[int, int, int], int] = {source: ACT_START}

    while heap:
        value, _, cur = heapq.heappop(heap)
        if value > dist.get(cur, float("inf")) + 1e-9:
            continue
        settled += 1
        row, col, head = cur
        for nxt, cost, action in _moves(grid, row, col, head):
            nv = value + cost
            if nv < dist.get(nxt, float("inf")) - 1e-9:
                dist[nxt] = nv
                came[nxt] = cur
                actions[nxt] = action
                counter += 1
                heapq.heappush(heap, (nv, counter, nxt))
    return dist, came, actions, settled


def _reconstruct(dist: dict, came: dict, actions: dict, goal: tuple[int, int, int]
                 ) -> tuple[list[tuple[int, int, int, int]], float]:
    """从洪泛结果里取出一条最短路径（目标朝向固定）。"""
    if goal not in dist:
        return [], float("inf")
    path: list[tuple[int, int, int, int]] = []
    node: tuple[int, int, int] | None = goal
    while node is not None:
        path.append((node[0], node[1], node[2], actions.get(node, ACT_START)))
        node = came[node]
    path.reverse()
    return path, dist[goal]


def _heading_of(drow: int, dcol: int, fallback: int) -> int:
    """位移 -> 朝向索引；零位移返回 fallback。"""
    if drow < 0:
        return 0
    if drow > 0:
        return 2
    if dcol < 0:
        return 1
    if dcol > 0:
        return 3
    return fallback


def _path_reverse(path: Sequence[tuple[int, int, int, int]]) -> list[bool]:
    """逐步路径 -> 每一步是否倒车（``back[i]`` 指到达点 i 的那一步）。"""
    return [action == ACT_BACKWARD for (_, _, _, action) in path]


# --------------------------------------------------------------------------- #
#  判据（复用 validate 的口径）
# --------------------------------------------------------------------------- #
def stop_distance_m(block_a: str, block_b: str) -> float:
    """两格中心的欧氏距离（米）。

    **复用** `validate._euc` —— 它与 `tools/shoot_planner.dist_m`、
    `tools/route_check.py` 的射击点规则是同一个算法（`BLOCK_SIZE × 格距`）。
    """
    return V._euc(Waypoint.at_block(block_a), block_b)      # noqa: SLF001


def turn_angle_deg(start: tuple[int, int], mid: tuple[int, int],
                   end: tuple[int, int]) -> float:
    """折返角（度）：进入段与离开段的夹角。0° = 直行，180° = 原地掉头。"""
    d1 = (mid[0] - start[0], mid[1] - start[1])
    d2 = (end[0] - mid[0], end[1] - mid[1])
    n1, n2 = math.hypot(*d1), math.hypot(*d2)
    if n1 < 1e-9 or n2 < 1e-9:
        return 0.0
    cosv = max(-1.0, min(1.0, (d1[0] * d2[0] + d1[1] * d2[1]) / (n1 * n2)))
    return math.degrees(math.acos(cosv))


def shoot_candidates(spec: MapSpec, grid: Grid, limits: tuple[float, float]) -> list[str]:
    """靶区的候选射击点：各靶格的**正交邻居**里、距离落在窗口内的可通行格。

    格距 0.35 m 时正交相邻恰为 0.35 m（窗口 0.20~0.45 内），对角 0.495 m 超限
    —— 几何决定的，与 `tools/shoot_planner.py` 的结论一致。
    `targets` 是**相连两格 = 一处轨道靶区**，所以候选取各靶格邻居的并集：
    一个能同时服务两格的点（如 `G10` 到 `G11`/`H11` 各 0.35 m）天然被包含。
    返回按格名排序（确定性）。
    """
    lo, hi = accept_window(limits)
    blocked = set(spec.obstacles)
    out: set[str] = set()
    for target in spec.targets:
        row_idx, col_idx = geo.block_to_index(target)
        row = row_idx + 1
        for drow, dcol in DIRS:
            nrow, ncol = row + drow, col_idx + dcol
            if not grid.passable(nrow, ncol):
                continue
            block = geo.index_to_block(nrow - 1, ncol)
            if block in blocked:
                continue
            if lo - 1e-9 <= stop_distance_m(block, target) <= hi + 1e-9:
                out.add(block)
    return sorted(out)


def accept_window(limits: tuple[float, float]) -> tuple[float, float]:
    """规划器采用的射击点接受窗口 = `tools/route_check.py` 的容差窗口。

    见 `ROUTE_CHECK_TOL_LO/HI` 的说明：规划器不自创阈值，接受窗口跟随"真正会执行的那条
    校验规则"。返回值仍是 ``(lo, hi)`` 升序。
    """
    lo, hi = (limits if limits[0] <= limits[1] else (limits[1], limits[0]))
    return lo - ROUTE_CHECK_TOL_LO, hi + ROUTE_CHECK_TOL_HI


def covering_stops(spec: MapSpec, pool: Sequence[str],
                   limits: tuple[float, float]) -> dict[str, tuple[str, ...]]:
    """每个候选射击点**能覆盖到哪些靶格**（`block -> (靶格, ...)`），按接受窗口判。

    ★ 为什么必须"全覆盖"：`targets` 是**相连两格 = 一处轨道靶区**，
    `tools/route_check.py` 会逐格判覆盖（"靶覆盖 N/M"）。只覆盖一格就是 "1/2 失败"
    —— 本轮实测踩过：射击点 `I11` 到 `H11` 0.35 m 合格、到 `G11` 0.70 m 超限。
    `validate.py` 的 R006 只要求"至少覆盖一个"，口径更松；这里按**更严的那个**来，
    否则过得了 dzbrain 预检、过不了出车前的 `route_check`。
    """
    lo, hi = accept_window(limits)
    return {block: tuple(t for t in spec.targets
                         if V._stop_ok(Waypoint.at_block(block), t, lo, hi))  # noqa: SLF001
            for block in pool}


# --------------------------------------------------------------------------- #
#  编译：A* 路径 -> Route
# --------------------------------------------------------------------------- #
@dataclass
class _Stop:
    """编译期内部结构：一个航点。

    ★ 不变量：``reverse[i] == True`` ⇔ **段 (i-1 → i) 是倒车段**。
      这与 `validate.py` / `route_check.py` 的读数口径完全一致
      （它们判的是"第 i 个航点是否由倒车到位"）。
    ``heading`` = **到达**该点时的车头；``depart`` = **从该点出发**时的车头
    （末点为 -1）。倒车段"车头不变"，所以 `reverse[i]` 时 `depart[i-1] == depart[i]`。
    """

    row: int
    col: int
    block: str
    heading: int = 0            # 到达时的车头朝向索引（0北/1西/2南/3东）
    depart: int = -1            # 出发时的车头朝向（末点 = -1）
    reverse: bool = False       # 段 (i-1 -> i) 倒车到位
    shoot: bool = False
    tag: str = ""               # start / end / mine / lower_bridge / upper_bridge / shoot
    #: `:PIVOT` 折返点声明（该顶点处车头需转过 ≥ `PIVOT_MARK_DEG`）。**纯声明。**
    pivot: bool = False


def _collapse(path: Sequence[tuple[int, int, int, int]],
              keep: set[tuple[int, int]]) -> list[tuple[int, int, int, bool]]:
    """逐步路径 -> ``[(row, col, face, back), ...]``。

    保留：起点、终点、`keep` 里的强制点（雷 / 桥洞 / 射击点）、**方向变化的拐点**、
    以及"步进模式变化"的拐点（前进↔倒车）。`face` 是车头**实际**指向
    （倒车时与行进方向相反），`back` 是"到达该点的那一步是否倒车"。

    ★ `keep` 必须包含全部雷点：`global_planner` 踩过的坑 —— 落在直线上的中间格会被
      压缩掉，规划器内部算的是全覆盖、输出的路线却漏一半，而且**不报任何错**。
    """
    if not path:
        return []
    moving = _path_reverse(path)

    steps: list[tuple[int, int, int, bool]] = []
    for i, (row, col, head, action) in enumerate(path):
        if i == 0:
            steps.append((row, col, head, False))
            continue
        prow, pcol = path[i - 1][0], path[i - 1][1]
        if (row, col) == (prow, pcol):
            if steps:
                steps[-1] = (steps[-1][0], steps[-1][1], head, steps[-1][3])
            continue
        direction = _heading_of(row - prow, col - pcol, head)
        face = (direction + 2) % 4 if moving[i] else direction
        steps.append((row, col, face, moving[i]))

    if len(steps) <= 2:
        return steps

    kept: list[tuple[int, int, int, bool]] = [steps[0]]
    for i in range(1, len(steps) - 1):
        prev, cur, nxt = steps[i - 1], steps[i], steps[i + 1]
        d1 = (cur[0] - prev[0], cur[1] - prev[1])
        d2 = (nxt[0] - cur[0], nxt[1] - cur[1])
        straight = (d1[0] * d2[1] - d1[1] * d2[0]) == 0 and (d1[0] * d2[0] + d1[1] * d2[1]) > 0
        same_mode = prev[3] == cur[3] == nxt[3]
        if (cur[0], cur[1]) in keep or not (straight and same_mode):
            kept.append(cur)
    kept.append(steps[-1])

    out: list[tuple[int, int, int, bool]] = []
    for i, (row, col, face, back) in enumerate(kept):
        if i + 1 < len(kept):
            nrow, ncol = kept[i + 1][0], kept[i + 1][1]
            direction = _heading_of(nrow - row, ncol - col, face)
            face = (direction + 2) % 4 if kept[i + 1][3] else direction
        out.append((row, col, face, back))
    return out


def _split_long_hops(stops: list[_Stop], grid: Grid,
                     max_blocks: int = MAX_HOP_BLOCKS) -> list[_Stop]:
    """在过长的相邻航点之间插入中间点（每格一个，只在直线段上）。

    ★ 为什么要插：`validate.py` 的 R011 在 "> 2.5 格" 处告警，而 1.4 m 以上的长直线
      在纯追踪下会切角（现场过桥洞尤其危险）。插点后段长恒 <= `max_blocks` 格，
      R011 一条都不出现，循迹也更贴格心。
    中间点**只从 A* 已经验证过可通行的格子里取**（沿直线插），所以不会踩障碍。
    """
    if len(stops) < 2:
        return stops
    out: list[_Stop] = [stops[0]]
    for cur in stops[1:]:
        prev = out[-1]
        drow, dcol = cur.row - prev.row, cur.col - prev.col
        steps = max(abs(drow), abs(dcol))
        straight = drow == 0 or dcol == 0
        if steps > max_blocks and straight:
            srow = 0 if drow == 0 else (1 if drow > 0 else -1)
            scol = 0 if dcol == 0 else (1 if dcol > 0 else -1)
            for k in range(1, steps):
                row, col = prev.row + srow * k, prev.col + scol * k
                block = geo.index_to_block(row - 1, col)
                if not grid.passable(row, col) or block in out[-1].block:
                    continue
                if out and (row, col) == (out[-1].row, out[-1].col):
                    continue
                out.append(_Stop(row, col, block, heading=cur.heading, reverse=cur.reverse))
            out[-1].reverse = cur.reverse
        out.append(cur)
    return out


def _fix_uturn_reverse(stops: list[_Stop]) -> int:
    """把"前进 → 前进"的 180° 折返改写成倒车段（与 `global_planner` 同一策略）。

    判定（避免误伤正常 90° 拐弯）：
      ① 折返角 > `UTURN_REVERSE_DEG`；② 两段都不是倒车段；③ 下一格可通行。
    改写 = 给**到达段**打 `reverse`（段 i → i+1 变成倒车到位）：车头保持不变、
    倒着退过去，零横向摆动。

    为什么必须改：原地 180° 掉头横向扫出约 0.54 m > 一格 0.35 m，在地图角落必出界；
    这也是现有手写路线里 `R:F11` / `R:B6` / `R:D6` 三段倒车的来历。
    """
    fixed = 0
    for i in range(1, len(stops) - 1):
        prev, cur, nxt = stops[i - 1], stops[i], stops[i + 1]
        if cur.reverse or nxt.reverse:
            continue            # 已经带倒车段（车头与行进方向不一致），不再改写
        if turn_angle_deg((prev.row, prev.col), (cur.row, cur.col),
                          (nxt.row, nxt.col)) <= UTURN_REVERSE_DEG:
            continue
        # ★ "车头朝西开进死胡同、再倒着退出来" = 进与出**两段都是倒车**：
        #   倒车段的车头保持不变，所以 `cur -> nxt` 的位移方向仍与车头一致时，
        #   必须把这**两段**都标成倒车 —— 只标一段就会让车头在中途凭空翻 180°。
        #   与现场手写路线的 `F10 F11 E11 R:F11 F10` 逐字同形。
        cur.reverse = True
        nxt.reverse = True
        fixed += 1
    return fixed


def _merge_duplicate_stops(stops: list[_Stop]) -> list[_Stop]:
    """合并相邻同格航点，**并保持 `reverse` 不变量**（两个同格点不可能同时倒车到位）。

    `_collapse` 偶尔会产出 `[F11(reverse=False), F11(reverse=True)]` 这种"同格两点"：
    前者是到达点、后者是出发段倒车的标记。保留两个会让 `RouteBuilder` 的去重把
    `reverse=True` 丢掉（它只保留前一个点），随后 `assert_legal` 就会报
    "E11 处 180° 折返" —— 这正是本轮踩过的坑。这里显式合并：同格时**保留反向标记为
    False 的那个到达语义**（`reverse[i]=True` 只表示"段 (i-1→i) 倒车"），
    射击/桥洞 tag 一并合并。
    """
    out: list[_Stop] = []
    for stop in stops:
        if out and (out[-1].row, out[-1].col) == (stop.row, stop.col):
            prev = out[-1]
            prev.reverse = prev.reverse and stop.reverse
            prev.shoot = prev.shoot or stop.shoot
            prev.tag = prev.tag or stop.tag
            continue
        out.append(stop)
    return out


def mark_pivots(stops: list[_Stop], *,
                min_deg: float = PIVOT_MARK_DEG) -> list[tuple[int, str, float]]:
    """给折返顶点打上 `:PIVOT` 声明，返回 ``[(索引, 格名, 角度°), ...]``。

    判据与 `pivot_vertices()` **逐字相同**（只是作用在编译期的 `_Stop` 上）：
    折返角 ≥ `min_deg` 且相邻两段都不是倒车段 ⇒ `stop.pivot = True`。

    ★ 调用时机：必须在 `_merge_duplicate_stops` **之后**（否则索引会变），
      且只在**无倒车模式**下调用 —— 允许倒车模式下折返由 `R:` 完成，
      不应出现 `:PIVOT` 声明（这也是 `--allow-reverse` 与卡 A 基线逐字节一致的结构性保证）。
    ★ `:PIVOT` 只是声明：不打车、不停车、不改 `pursuit/**` 与 `tracker.py` 的行为。
    """
    marked: list[tuple[int, str, float]] = []
    for i in range(1, len(stops) - 1):
        prev, cur, nxt = stops[i - 1], stops[i], stops[i + 1]
        if cur.reverse or nxt.reverse:
            continue
        angle = turn_angle_deg((prev.row, prev.col), (cur.row, cur.col),
                               (nxt.row, nxt.col))
        if angle >= min_deg - 1e-9:
            cur.pivot = True
            marked.append((i, cur.block, angle))
    return marked


def _refresh_headings(stops: list[_Stop]) -> None:
    """重算每个航点记的**车头实际朝向**（= 到达该点时车头指向哪边）。

    唯一一条物理规则：**倒车段车头不变**（车头背着目标、倒着退过去）。于是

    * 前进到达 p → p 的车头 = 行进方向；
    * 倒车到达 p → p 的车头 = 行进方向的**反方向**（车头背着 p，倒着退进来了）。

    这条规则对"前进段起点"同时也是"下一段出发方向"（车确实朝那边开），
    对"倒车段起点"则给出背离目标的车头 —— 与现场手写路线的 `F10 F11 E11 R:F11 F10`
    逐字同形（`R:F11` 那段就是"车头朝西、倒着往东退"）。

    ★ 调用时机：必须在 `_fix_uturn_reverse` **之后**。折返改写会翻转某一段的 `reverse`
      （车头随之翻 180°），先算后改就会把终点姿态与云台基准朝向写错。

    注意：`RouteBuilder` 给中间点自动补的 yaw 是"离开段方向"，在倒车段上会与车头差
    180°（现场手写路线也一样）；`tracker` 判断倒车用的是
    `path.segment_is_reverse()` 而不是 yaw，所以 yaw 只对**终点姿态**有意义 ——
    这正是本函数要保证 `_final_heading` 正确的原因。
    """
    for i in range(1, len(stops)):
        prev, cur = stops[i - 1], stops[i]
        direction = _heading_of(cur.row - prev.row, cur.col - prev.col, cur.heading)
        cur.heading = (direction + 2) % 4 if cur.reverse else direction
    # 出发车头：前进 = 行进方向；倒车 = 行进方向的反方向（车头背着目标退过去）。
    #   —— 倒车段"车头不变"，所以 `reverse[i+1]` 时 `depart[i]` 必须等于 `heading[i]`。
    for i in range(len(stops) - 1):
        cur, nxt = stops[i], stops[i + 1]
        direction = _heading_of(nxt.row - cur.row, nxt.col - cur.col, cur.heading)
        cur.depart = (direction + 2) % 4 if nxt.reverse else direction
    stops[-1].depart = -1


def _final_heading(stops: Sequence[_Stop]) -> int | None:
    """末点航向（供 `RouteBuilder.fix_final_yaw`）= 到达末点时的车头（已含倒车翻转）。"""
    return stops[-1].heading if stops else None


def _cell(block: str) -> tuple[int, int]:
    row_idx, col_idx = geo.block_to_index(block)
    return row_idx + 1, col_idx


def _to_route(stops: Sequence[_Stop], *, name: str, crossing: dict,
              final_heading: int | None = None) -> Route:
    """`_Stop` 序列 -> `Route`（经 `RouteBuilder` 补朝向）。

    ★ `drop_consecutive_duplicates=False`：相邻同格点必须走 `_merge_duplicate_stops`
      （它知道 `reverse` 的语义），`RouteBuilder` 的内置去重会把 `reverse=True` 丢掉。
    """
    builder = RouteBuilder(name)
    for stop in stops:
        x, y = geo.block_to_meters(stop.block)
        yaw = geo.HEADING_Y if stop.heading in (1, 3) else geo.HEADING_X
        token = (("R:" if stop.reverse else "") + stop.block
                 + ("*" if stop.shoot else "")
                 + (_PIVOT_SUFFIX if stop.pivot else ""))
        builder.add(Waypoint(block=stop.block, x=x, y=y, yaw=yaw,
                             reverse=stop.reverse, shoot=stop.shoot,
                             pivot=stop.pivot, token=token))
    fix_final = None
    if final_heading is not None:
        fix_final = geo.HEADING_Y if final_heading in (1, 3) else geo.HEADING_X
    route = builder.build(drop_consecutive_duplicates=False, fix_final_yaw=fix_final)
    return Route(route.waypoints, name=name, crossing=dict(crossing))


def route_tokens(route: Route, *, final_heading: bool = True) -> list[str]:
    """`Route` -> token 列表（下发 `/nav/waypoint` / 回写 `robot.yaml`）。

    后缀顺序与 `parser.normalize_token` 的约定一致：
    `R:` 前缀 → 格名 → `*` → `:PIVOT` → `,x|,y`。
    """
    tokens: list[str] = []
    last = len(route.waypoints) - 1
    for idx, wp in enumerate(route.waypoints):
        token = ("R:" if wp.reverse else "") + wp.block + ("*" if wp.shoot else "")
        if wp.pivot:
            token += _PIVOT_SUFFIX
        if final_heading and idx == last:
            token += ",y" if abs(wp.yaw - geo.HEADING_Y) < 1e-6 else ",x"
        tokens.append(token)
    return tokens


# --------------------------------------------------------------------------- #
#  自检
# --------------------------------------------------------------------------- #
def check_route_structure(route: Route, spec: MapSpec, grid: Grid,
                          limits: tuple[float, float]) -> list[str]:
    """规划器自检（返回问题列表，空 = 通过）。**只报不改。**

    这些是"规划器必须自己保证"的结构约束；完整校验仍以 `validate.validate_route`
    与 `tools/route_check.py` 为准（见 `tools/plan_route.py --check`）。
    """
    problems: list[str] = []
    blocks = route.blocks()
    if not blocks:
        return ["路线为空"]

    if blocks[0] != spec.start:
        problems.append(f"起点不是 map.start={spec.start}（实际 {blocks[0]}）")
    if blocks[-1] != spec.end:
        problems.append(f"终点不是 map.end={spec.end}（实际 {blocks[-1]}）")

    mine_set, obs_set = set(spec.mines), set(spec.obstacles)
    miss = sorted(mine_set - set(blocks))
    if miss:
        problems.append(f"雷漏压 {' '.join(miss)}")
    on_obs = sorted(obs_set & set(blocks))
    if on_obs:
        problems.append(f"路线踩到障碍格 {' '.join(on_obs)}")
    u_cols = {geo.COL_LETTERS.index(c) for c in V.U_TURN_COLUMNS}
    if not any(geo.block_to_index(b)[1] in u_cols for b in blocks):
        problems.append(f"路线完全没有经过 {'/'.join(V.U_TURN_COLUMNS)} 列（没绕过隔板）")

    runs: list[tuple[str, str, tuple]] = []
    for a, b in zip(blocks, blocks[1:]):
        hit = V._crossing_at_column(a, b)               # noqa: SLF001
        if hit is not None:
            runs.append((a, b, hit))
    lower = [r for r in runs if V.LOWER_HALF_ROWS[0] <= round(r[2][0]) <= V.LOWER_HALF_ROWS[1]]
    upper = [r for r in runs if V.UPPER_HALF_ROWS[0] <= round(r[2][0]) <= V.UPPER_HALF_ROWS[1]]
    if len(lower) != 1:
        problems.append(f"下半场应恰好穿洞 1 次，实际 {len(lower)} 次")
    elif sum(r[2][3] for r in lower) >= 0:
        problems.append("下半场穿洞方向不是**向西**（R007：L* -> K* -> J*）")
    if len(upper) != 1:
        problems.append(f"上半场应恰好穿洞 1 次，实际 {len(upper)} 次")
    elif sum(r[2][3] for r in upper) <= 0:
        problems.append("上半场穿洞方向不是**向东**（R007：J* -> K* -> L*）")
    for a, b, hit in runs:
        if hit[1] > V.MAX_CROSS_ANGLE_DEG + 1e-9:
            problems.append(f"{a}->{b} 斜切入洞 {hit[1]:.1f}°"
                            f"（R009 上限 {V.MAX_CROSS_ANGLE_DEG:.0f}°）")

    for a, b in zip(blocks, blocks[1:]):
        hit = V._barrier_crossing(a, b)                 # noqa: SLF001
        if hit and hit[0]:
            name = geo.COL_LETTERS[min(geo.MAP_COLS - 1, max(0, int(round(hit[1]))))]
            problems.append(f"{a}->{b} 非法跨越第 {grid.barrier_hi}/{grid.barrier_lo} 行隔板"
                            f"（交点列 {name}，只有 {'/'.join(V.U_TURN_COLUMNS)} 列能过）")

    stops = route.shoot_points()
    if spec.targets:
        if len(stops) != 1:
            problems.append(f"射击停站应恰好 1 个，实际 {len(stops)}")
        for _, wp in stops:
            # ★ 判据比 `validate.py` 的 R006 更严：**每个**靶格都要被这个停站覆盖。
            #   理由见 `covering_stops` —— route_check 是逐格判的，只覆盖一格会
            #   报「靶覆盖 1/2」。窗口用 route_check 的接受窗口（含其容差）。
            lo, hi = accept_window(limits)
            for tgt in spec.targets:
                if not V._stop_ok(wp, tgt, lo, hi):                 # noqa: SLF001
                    problems.append(
                        f"射击停站 {wp.block} 到靶 {tgt} 的距离 "
                        f"{V._euc(wp, tgt):.3f} m 不在窗口 "
                        f"{lo:.2f}~{hi:.2f} m（route_check 会判覆盖不足）")
    elif stops:
        problems.append("地图没有 targets，路线里却有射击停站")

    for a, b in zip(blocks, blocks[1:]):
        ax, ay = geo.block_to_meters(a)
        bx, by = geo.block_to_meters(b)
        length = math.hypot(bx - ax, by - ay)
        if length < V.MIN_SEGMENT_M:
            problems.append(f"段 {a}->{b} 长度 {length:.4f} m < {V.MIN_SEGMENT_M} m（重复点？）")
        elif length / geo.BLOCK_SIZE > V.MAX_SEGMENT_BLOCKS:
            problems.append(f"段 {a}->{b} 长 {length / geo.BLOCK_SIZE:.2f} 格"
                            f" > {V.MAX_SEGMENT_BLOCKS} 格（R011）")

    reverse_m = total_m = 0.0
    for i, (a, b) in enumerate(zip(blocks, blocks[1:])):
        ax, ay = geo.block_to_meters(a)
        bx, by = geo.block_to_meters(b)
        seg = math.hypot(bx - ax, by - ay)
        total_m += seg
        if route.waypoints[i + 1].reverse:
            reverse_m += seg
    if total_m > 1e-9 and reverse_m / total_m > V.MAX_REVERSE_RATIO:
        problems.append(f"倒车占比 {reverse_m / total_m * 100:.1f}%"
                        f" > R014 的 {V.MAX_REVERSE_RATIO * 100:.0f}%（现场硬约束）")
    return problems


def assert_legal(route: Route) -> None:
    """硬性合法性断言：token 可解析 + 无相邻重复点 + **同模式 180° 折返为 0**。

    最后一条是本规划器最关键的不变量：180° 折返必须以 `R:`（倒车）出现，
    否则实车只能靠原地转向掉头，横向扫出 0.54 m > 一格，直接出界。
    """
    for idx, wp in enumerate(route.waypoints):
        token = ("R:" if wp.reverse else "") + wp.block + ("*" if wp.shoot else "")
        info = parse_token(token)
        if info["block"] != wp.block or info["reverse"] != wp.reverse:
            raise PlannerError(f"第 {idx} 个航点 {token!r} 与模型不一致")
    blocks = route.blocks()
    for a, b in zip(blocks, blocks[1:]):
        if a == b:
            raise PlannerError(f"相邻重复航点 {a}")

    for i in range(1, len(route.waypoints) - 1):
        prev, cur, nxt = route.waypoints[i - 1], route.waypoints[i], route.waypoints[i + 1]
        # 只要相邻两段里有倒车段，就不算"原地掉头"（车头没掉过来）——与
        # route_check 的 _is_rev_vertex 同一判据。
        if prev.reverse or cur.reverse:
            continue
        angle = _angle_of(prev, cur, nxt)
        if angle > UTURN_REVERSE_DEG:
            raise PlannerError(f"第 {i} 个航点 {cur.block} 处存在同模式 {angle:.0f}° 折返"
                               f"（未改写为倒车段）")


def _angle_of(a: Waypoint, b: Waypoint, c: Waypoint) -> float:
    d1, d2 = (b.x - a.x, b.y - a.y), (c.x - b.x, c.y - b.y)
    n1, n2 = math.hypot(*d1), math.hypot(*d2)
    if n1 < 1e-9 or n2 < 1e-9:
        return 0.0
    cosv = max(-1.0, min(1.0, (d1[0] * d2[0] + d1[1] * d2[1]) / (n1 * n2)))
    return math.degrees(math.acos(cosv))


def pivot_vertices(route: Route, *, min_deg: float = PIVOT_MARK_DEG
                   ) -> list[tuple[int, str, float]]:
    """折返顶点清单 ``[(索引, 格名, 折返角°), ...]``（升序）。

    判据与循迹层**逐条对齐**（这不是另发明一套）：
      * 角度 = `pursuit.path.corner_turn_deg` = 顶点处**车头**要转过的角度；
        无倒车模式下与 `corner_angle_deg`（行进方向夹角）同值；
      * 阈值 = `pursuit.pivot_angle_deg`（`PIVOT_MARK_DEG` = 135°，只做声明）；
      * 相邻两段中**任一段是倒车段**就不算折返（`route_check._is_rev_vertex` 同一判据）：
        倒车段的车头一度都不用转，那里不会触发 PIVOT。
    """
    out: list[tuple[int, str, float]] = []
    wps = route.waypoints
    for i in range(1, len(wps) - 1):
        prev, cur, nxt = wps[i - 1], wps[i], wps[i + 1]
        if cur.reverse or nxt.reverse:              # 相邻段有倒车 → 车头不用掉头
            continue
        angle = _angle_of(prev, cur, nxt)
        if angle >= min_deg - 1e-9:
            out.append((i, cur.block, angle))
    return out


def assert_legal_no_reverse(route: Route) -> None:
    """无倒车模式的硬性合法性断言（`plan_route(allow_reverse=False)` 内必调）。

    四条不变量：
      ① 每个航点的 token 可解析，且 `block` / `reverse` / `pivot` 与模型一致；
      ② 相邻航点不重复；
      ③ **0 段 `R:`** —— 真车底盘不支持倒车（负速度方向位被 STM32 无视，车以 |v|
         朝车头方向前进），出现一段 `R:` 就整条路线报废（旧行为：不允许）；
      ④ **折返点声明完整、且不虚报**：每个折返角 ≥ `PIVOT_MARK_DEG`（= 循迹层
         `pursuit.pivot_angle_deg`，判据同 `pursuit.path.corner_turn_deg`）的顶点
         都必须带 `:PIVOT`；反之带 `:PIVOT` 的顶点必须真的是折返点。

    第 ④ 条就是"合法性自检"的负向控制入口：把某处 `:PIVOT` 去掉即抛 `PlannerError`。
    `:PIVOT` 本身**不改变**循迹层行为（`pursuit/**` / `tasks/tracker.py` 不读该字段），
    这里断言的是"声明与实际几何必须一致"，不是"照着声明去执行"。
    """
    for idx, wp in enumerate(route.waypoints):
        token = ("R:" if wp.reverse else "") + wp.block + ("*" if wp.shoot else "")
        if wp.pivot:
            token += _PIVOT_SUFFIX
        info = parse_token(token)
        if (info["block"] != wp.block or info["reverse"] != wp.reverse
                or info["pivot"] != wp.pivot):
            raise PlannerError(f"第 {idx} 个航点 {token!r} 与模型不一致")
        if wp.reverse:
            raise PlannerError(
                f"无倒车模式却出现倒车段：第 {idx} 个航点 {token!r}"
                f"（真车底盘不支持倒车，负速度方向位被 STM32 无视）")
    blocks = route.blocks()
    for a, b in zip(blocks, blocks[1:]):
        if a == b:
            raise PlannerError(f"相邻重复航点 {a}")

    # ---- ④ 折返点声明的一致性（完整性 + 不虚报）----
    need = {i: (block, deg) for i, block, deg in pivot_vertices(route)}
    declared = {i: wp.block for i, wp in enumerate(route.waypoints) if wp.pivot}
    missing = sorted(set(need) - set(declared))
    if missing:
        detail = "; ".join(f"#{i} {need[i][0]} {need[i][1]:.0f}°" for i in missing)
        raise PlannerError(
            f"折返点缺少 {_PIVOT_SUFFIX} 声明：{detail}"
            f"（折返角 ≥ {PIVOT_MARK_DEG:.0f}° 的顶点必须显式声明 —— "
            f"该处车头要转 {need[missing[0]][1]:.0f}°，无倒车模式下车只能原地掉头）")
    extra = sorted(set(declared) - set(need))
    if extra:
        detail = "; ".join(f"#{i} {declared[i]}" for i in extra)
        raise PlannerError(
            f"虚报 {_PIVOT_SUFFIX}：{detail}（这些顶点的折返角 "
            f"< {PIVOT_MARK_DEG:.0f}°，或相邻段是倒车段 —— 车头一度都不用转）")


# --------------------------------------------------------------------------- #
#  规划结果
# --------------------------------------------------------------------------- #
@dataclass
class PlanResult:
    """规划结果（含全部证据数值，便于直接贴日志 / 出 JSON）。"""

    route: Route
    tokens: list[str]
    stops: tuple["_Stop", ...] = ()
    crossing: dict = field(default_factory=dict)
    shoot_stop: str = ""
    shoot_distance_m: float = 0.0
    shoot_targets: tuple[str, ...] = ()
    cost: float = 0.0
    expanded: int = 0
    elapsed_ms: float = 0.0
    reverse_segments: int = 0
    uturn_fixes: int = 0
    warnings: tuple[str, ...] = ()
    map_name: str = ""
    map_source: str = ""
    start: str = ""
    end: str = ""
    polyline_m: float = 0.0
    start_heading: str = ""
    #: 本次规划是否**允许倒车**（`allow_reverse`；False = 车上默认的无倒车模式）
    allow_reverse: bool = True
    #: ★ 折返点清单 ``[(航点索引, 格名, 折返角°), ...]`` —— 与 `:PIVOT` 声明一一对应
    pivots: tuple[tuple[int, str, float], ...] = ()
    #: 折返惩罚（等效代价单位/处）；0.0 = 未启用
    pivot_penalty: float = 0.0
    #: "雷格不得作拐点"的惩罚权重（等效代价单位）；0.0 = 未启用
    mine_turn_penalty: float = 0.0
    #: ★ 逐雷穿越轴 ``[(雷格名, 轴编号0北1西2南3东, 是否找到可行轴), …]`` ——
    #: `ok=False` 表示该雷找不到直线穿越轴（会被切角擦过，需人工核对地图）
    mine_axes: tuple[tuple[str, int, bool], ...] = ()
    #: ★【自诊断】逐雷"进点前共线格数" ``[(雷格名, 格数), …]`` —— 见 `_mine_aligned_runs`。
    #: `<2` 的雷在纯追踪下会被切角（车心压不到雷心），plan_route 会另附一条 warning。
    mine_runs: tuple[tuple[str, int], ...] = ()
    #: 顺序代价（含折返惩罚）—— 供"加惩罚前 vs 后"对比用
    tour_cost: float = 0.0

    @property
    def pivot_count(self) -> int:
        return len(self.pivots)

    def as_dict(self) -> dict:
        return {
            "map": self.map_name, "map_file": self.map_source,
            "start": self.start, "end": self.end,
            "start_heading": self.start_heading,
            "tokens": list(self.tokens), "token_count": len(self.tokens),
            "crossing": dict(self.crossing),
            "shoot_stop": self.shoot_stop,
            "shoot_distance_m": round(self.shoot_distance_m, 4),
            "shoot_targets": list(self.shoot_targets),
            "cost": round(self.cost, 4),
            "polylength_m": round(self.polyline_m, 4),
            "reverse_segments": self.reverse_segments,
            "uturn_fixes": self.uturn_fixes,
            "expanded": self.expanded,
            "elapsed_ms": round(self.elapsed_ms, 3),
            "warnings": list(self.warnings),
            "allow_reverse": self.allow_reverse,
            "pivot_penalty": round(self.pivot_penalty, 4),
            "mine_turn_penalty": round(self.mine_turn_penalty, 4),
            "mine_axes": [{"block": b, "axis": a, "ok": o} for b, a, o in self.mine_axes],
            "mine_runs": [{"block": b, "run": r} for b, r in self.mine_runs],
            "tour_cost": round(self.tour_cost, 4),
            "pivot_count": self.pivot_count,
            "pivots": [{"index": i, "block": b, "deg": round(d, 1)}
                       for i, b, d in self.pivots],
        }

    def summary(self) -> str:
        shoot = (f"射击停站 {self.shoot_stop} 到靶 {self.shoot_distance_m:.3f} m"
                 f"（{'/'.join(self.shoot_targets)}）" if self.shoot_stop else "无射击停站")
        pivots = (" ".join(f"{b} {d:.0f}°" for _, b, d in self.pivots) or "无")
        lines = [
            f"路线[{self.route.name}] {len(self.tokens)} token / {self.polyline_m:.2f} m"
            f" | 穿洞 下={self.crossing.get('lower', '?')} 上={self.crossing.get('upper', '?')}"
            f" | {shoot}",
            f"  A* 代价 {self.cost:.2f} / 展开 {self.expanded} 次 / 耗时 {self.elapsed_ms:.1f} ms"
            f" | 倒车段 {self.reverse_segments}（折返改写 {self.uturn_fixes}）"
            f" | 无倒车模式 {'是' if not self.allow_reverse else '否'}",
            f"  折返点 {self.pivot_count} 处（{_PIVOT_SUFFIX} 声明）：{pivots}"
            f" | 顺序代价 {self.tour_cost:.2f}（折返惩罚 {self.pivot_penalty:.1f}/处）",
            "  " + " ".join(self.tokens),
        ]
        lines.extend(f"  ⚠️ {w}" for w in self.warnings)
        return "\n".join(lines)


# --------------------------------------------------------------------------- #
#  顺序 + 朝向联合优化
# --------------------------------------------------------------------------- #
class _Tour:
    """顺序与朝向的联合优化器 + 段路径缓存。

    数据：
      ``order_cost[(a, b)]``（顺序轮用；四个朝向对里的最小值，是真实代价的**下界**）
      ``head_cost[(a, b)][ha][hb]``（朝向轮用；带朝向的精确代价）

    ★ 代价全部来自"每个点一次 Dijkstra 洪泛"（见 `flood`），不是逐对 A* ——
      后者在 19 个点上要 2700+ 次搜索，会直接顶爆 `DEFAULT_MAX_SECONDS`。
    """

    def __init__(self, grid: Grid, deadline: float,
                 pivot_penalty: float = 0.0, record_dirs: bool = True,
                 mine_cells: Iterable[tuple[int, int]] = (),
                 mine_turn_penalty: float = 0.0) -> None:
        self.grid = grid
        self.deadline = deadline
        #: ★ 折返惩罚（等效代价单位/处）。0.0 = 关闭（允许倒车模式的默认值，
        #: 也是"加惩罚前 vs 后"对比实验的对照组）。
        self.pivot_penalty = max(0.0, float(pivot_penalty))
        #: 是否记录"每对访问点最短腿的起步/末步朝向"——折返惩罚与
        #: `straighten_mines()`（雷心直线穿越）都要用它，故默认开启。
        self.record_dirs = bool(record_dirs)
        #: ★ 雷格集合 + **雷格不得作为拐点**的惩罚权重：顺序里若某个雷格处有转角
        #: （90° 记 1 份、180° 记 8 份），按该权重计价。理由同 `MINE_PRESS_TOL_M`：
        #: 雷格一旦是拐点，纯追踪就会切角/掉头，车心压不到雷心 —— 这一项让
        #: "顺序 ↔ 朝向"优化主动把雷格排到**直线上**，而不是事后补救。
        self.mine_cells = set(mine_cells)
        self.mine_turn_penalty = max(0.0, float(mine_turn_penalty))
        self._floods: dict[tuple[int, int], tuple[dict, dict, dict, int]] = {}
        self.settled = 0
        self.head_cost: dict[tuple[tuple[int, int], tuple[int, int]],
                             list[list[float]]] = {}
        self.order_cost: dict[tuple[tuple[int, int], tuple[int, int]], float] = {}
        #: 每对访问点之间"最短腿"的**起步朝向**与**末步朝向**（用于判折返）
        self._exit_dir: dict[tuple[tuple[int, int], tuple[int, int]], int] = {}
        self._entry_dir: dict[tuple[tuple[int, int], tuple[int, int]], int] = {}

    # ---------------- 洪泛与查询 ----------------
    def _flood_of(self, cell: tuple[int, int]) -> tuple[dict, dict, dict, int]:
        hit = self._floods.get(cell)
        if hit is None:
            if time.perf_counter() > self.deadline:
                raise PlannerError("规划超时（代价洪泛阶段）")
            hit = flood(self.grid, (cell[0], cell[1], 0))
            self._floods[cell] = hit
            self.settled += hit[3]
        return hit

    def cost(self, a: tuple[int, int], b: tuple[int, int], ha: int, hb: int) -> float:
        """带朝向的精确代价 `(a, ha) -> (b, hb)`（洪泛里直接读，O(1)）。"""
        if a == b:
            return 0.0 if ha == hb else COST_TURN
        dist = self._flood_of(a)[0]
        return dist.get((b[0], b[1], hb), float("inf"))

    def path(self, a: tuple[int, int], b: tuple[int, int], ha: int, hb: int
             ) -> tuple[list[tuple[int, int, int, int]], float]:
        """取一段最短路径（同一 (起,终,朝向对) 只算一次）。"""
        if a == b:
            return ([(a[0], a[1], ha, ACT_START)], 0.0) if ha == hb else ([], float("inf"))
        dist, came, actions, _ = self._flood_of(a)
        return _reconstruct(dist, came, actions, (b[0], b[1], hb))

    # ---------------- 代价表 ----------------
    def build(self, cells: Sequence[tuple[int, int]]) -> None:
        """预建两套代价表（每个点一次洪泛 + O(点数²) 的查表）。"""
        ordered = sorted(set(cells))
        for cell in ordered:
            self._flood_of(cell)
        for i, a in enumerate(ordered):
            for b in ordered[i + 1:]:
                table = [[self.cost(a, b, ha, hb) for hb in range(4)]
                         for ha in range(4)]
                self.head_cost[(a, b)] = table
                self.head_cost[(b, a)] = [[table[hb][ha] for hb in range(4)]
                                          for ha in range(4)]
                value = min(min(row) for row in table)
                self.order_cost[(a, b)] = self.order_cost[(b, a)] = value
        # ★ 折返惩罚与"雷心直线穿越"都要用"每对点之间最短腿的起步/末步朝向"
        #   （见 `_record_leg_dirs`）
        if self.record_dirs or self.pivot_penalty > 0.0:
            for a in ordered:
                for b in ordered:
                    if a != b:
                        self._record_leg_dirs(a, b)

    def _record_leg_dirs(self, a: tuple[int, int], b: tuple[int, int]) -> None:
        """记下 `a -> b` **最短腿**的起步朝向与末步朝向（只在末步/起步方向判折返）。

        为什么这么判：无倒车模式里"车在顶点掉头"的物理表现就是**到达方向与离开方向
        正好相反**（原路退回）。这里取的是纯里程最短腿（朝向对里代价最小者，平局按
        (ha, hb) 升序 → 确定性），所以它是顺序层的**下界估计**，不会高估也不会漏判
        那些"必须以折返结束/开始"的访问。
        """
        table = self.head_cost.get((a, b))
        if table is None:
            return
        best: tuple[float, int, int] | None = None
        for ha in range(4):
            for hb in range(4):
                value = table[ha][hb]
                if not math.isfinite(value):
                    continue
                if best is None or value < best[0] - 1e-12:
                    best = (value, ha, hb)
        if best is None:
            return
        path, _ = self.path(a, b, best[1], best[2])
        dirs: list[int] = []
        for prev, cur in zip(path, path[1:]):
            if (cur[0], cur[1]) != (prev[0], prev[1]):
                dirs.append(_heading_of(cur[0] - prev[0], cur[1] - prev[1], -1))
        dirs = [d for d in dirs if d >= 0]
        if dirs:
            self._exit_dir[(a, b)] = dirs[0]
            self._entry_dir[(a, b)] = dirs[-1]

    def leg_cost(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        """`a -> b` 的**顺序层代价**（四个朝向对里的最小值，与 `order_cost` 同口径）。

        新插入的格子（`straighten_mines` 的进点/出点）不在 `order_cost` 里，
        这里按需洪泛并取最小，保证"选哪条穿越轴"的判决与顺序优化用同一把尺子。
        """
        if a == b:
            return 0.0
        hit = self.order_cost.get((a, b))
        if hit is not None:
            return hit
        best = float("inf")
        for ha in range(4):
            for hb in range(4):
                value = self.cost(a, b, ha, hb)
                if value < best:
                    best = value
        return best

    def reversals(self, seq: Sequence[tuple[int, int]]) -> int:
        """访问顺序里的**折返次数**：`a -> b -> c` 且"进 b 的末步朝向"与"出 b 的起步朝向"相反。"""
        count = 0
        for i in range(1, len(seq) - 1):
            entry = self._entry_dir.get((seq[i - 1], seq[i]))
            leave = self._exit_dir.get((seq[i], seq[i + 1]))
            if entry is None or leave is None:
                continue
            if (entry - leave) % 4 == 2:                 # 正相反 = 原路折返
                count += 1
        return count

    def penalty_of(self, seq: Sequence[tuple[int, int]]) -> float:
        """顺序里"不该有的动作"的总代价：折返（180°）+ **雷格当拐点**。"""
        total = 0.0
        if self.pivot_penalty > 0.0:
            total += self.pivot_penalty * self.reversals(seq)
        if self.mine_turn_penalty > 0.0 and self.mine_cells:
            for i in range(1, len(seq) - 1):
                if seq[i] not in self.mine_cells:
                    continue
                entry = self._entry_dir.get((seq[i - 1], seq[i]))
                leave = self._exit_dir.get((seq[i], seq[i + 1]))
                if entry is None or leave is None:
                    continue
                delta = (entry - leave) % 4
                if delta == 0:
                    continue                    # 直线穿越 = 合格
                total += self.mine_turn_penalty * (8.0 if delta == 2 else 1.0)
        return total

    # ---------------- 顺序 ----------------
    def order_cost_total(self, seq: Sequence[tuple[int, int]]) -> float:
        """整条顺序的代价（含惩罚项）。**用 `leg_cost`**，所以对 `straighten_mines`
        插进来的新格子也成立 —— 否则 `seq_cost` 读不到预建表会返回 `inf`，
        报告里就会出现"顺序代价 inf"这种假读数（本轮踩过）。
        """
        total = self.penalty_of(seq)
        for a, b in zip(seq, seq[1:]):
            value = self.leg_cost(a, b)
            if not math.isfinite(value):
                return float("inf")
            total += value
        return total

    def seq_cost(self, seq: Sequence[tuple[int, int]]) -> float:
        total = self.penalty_of(seq)
        for a, b in zip(seq, seq[1:]):
            value = self.order_cost.get((a, b), float("inf"))
            if not math.isfinite(value):
                return float("inf")
            total += value
        return total

    def nearest(self, first: tuple[int, int],
                nodes: Sequence[tuple[int, int]]) -> list[tuple[int, int]]:
        """最近邻。**确定性**：距离相同时按格名序，绝不依赖 set/dict 迭代顺序。"""
        remaining = sorted(set(nodes))
        out: list[tuple[int, int]] = []
        cur = first
        while remaining:
            best = min(remaining,
                       key=lambda n: (self.order_cost.get((cur, n), float("inf")),
                                      geo.index_to_block(n[0] - 1, n[1])))
            remaining.remove(best)
            out.append(best)
            cur = best
        return out

    def two_opt(self, seq: list[tuple[int, int]],
                *, keep_ends: bool = True) -> list[tuple[int, int]]:
        """2-opt（子段反转）直到不再改进或达到轮数上限。

        ★ `keep_ends=True`（默认）时**只反转内部子段**：起点/终点是任务硬约束
          （`mission.reset` 锚在起点、终点要停在 `map.end`），2-opt 不该动它们。
          旧实现允许反转含首/末的子段，一旦代价函数变化就会把起点挪走，
          只能在 `plan_route` 末尾靠"内部错误：优化后起点/终点被挪动"兜住 ——
          那是把约束写在断言里而不是算法里，本轮踩到后改为按约束搜索。
        """
        if len(seq) < 3:
            return seq
        i_lo = 1 if keep_ends else 0
        i_hi = len(seq) - 1 if keep_ends else len(seq) - 1
        j_hi = len(seq) - 1 if keep_ends else len(seq)
        improved, passes = True, 0
        while improved and passes < MAX_2OPT_PASSES:
            improved, passes = False, passes + 1
            best = self.seq_cost(seq)
            for i in range(i_lo, i_hi):
                for j in range(i + 2, j_hi):
                    candidate = seq[:i] + list(reversed(seq[i:j + 1])) + seq[j + 1:]
                    cost = self.seq_cost(candidate)
                    if cost < best - 1e-9:
                        seq, best, improved = candidate, cost, True
        return seq

    # ---------------- 朝向 ----------------
    def pair_cost(self, a: tuple[int, int], b: tuple[int, int], ha: int, hb: int) -> float:
        if a == b:
            return 0.0 if ha == hb else COST_TURN
        table = self.head_cost.get((a, b))
        if table is None:
            return self.cost(a, b, ha, hb)
        return table[ha][hb]

    def route_cost(self, order: Sequence[tuple[int, int]], heads: Sequence[int]) -> float:
        return (sum(self.pair_cost(order[i], order[i + 1], heads[i], heads[i + 1])
                    for i in range(len(order) - 1))
                + self.penalty_of(order))

    def best_headings(self, order: Sequence[tuple[int, int]],
                      seed: Sequence[int] | None = None,
                      fixed: dict[tuple[int, int], int] | None = None) -> list[int]:
        """给定顺序，最小化 `Σ pair_cost(i, i+1)` 的朝向序列（坐标下降）。

        起点朝向通常被 `mission.reset` 锁死（放在 `fixed` 里）；终点朝向自由
        （可在终点原地转向，代价由 `pair_cost` 的 `COST_TURN` 体现）。
        """
        fixed = dict(fixed or {})
        heads: list[int] = []
        for idx, cell in enumerate(order):
            if cell in fixed:
                heads.append(fixed[cell])
            elif seed is not None and idx < len(seed):
                heads.append(seed[idx] % 4)
            else:
                heads.append(0)
        best = self.route_cost(order, heads)
        improved, passes = True, 0
        while improved and passes < MAX_2OPT_PASSES:
            improved, passes = False, passes + 1
            for idx, cell in enumerate(order):
                if cell in fixed:
                    continue
                keep = heads[idx]
                for head in range(4):
                    if head == keep:
                        continue
                    heads[idx] = head
                    value = self.route_cost(order, heads)
                    if value < best - 1e-9:
                        best, keep, improved = value, head, True
                heads[idx] = keep
        return heads

    def polish(self, order: list[tuple[int, int]], heads: Sequence[int],
               rounds: int = 2) -> tuple[list[tuple[int, int]], list[int], float]:
        """顺序 ↔ 朝向交替优化 —— 这正是"相邻两段的到达/出发朝向必须一致"的那一环。"""
        best_order, best_heads = list(order), list(heads)
        best = self.route_cost(best_order, best_heads)
        for _ in range(max(0, rounds)):
            order2 = self.two_opt(list(best_order))
            heads2 = self.best_headings(order2, seed=best_heads)
            value = self.route_cost(order2, heads2)
            if value < best - 1e-9:
                best_order, best_heads, best = order2, heads2, value
            else:
                break
        return best_order, best_heads, best


# --------------------------------------------------------------------------- #
#  主入口
# --------------------------------------------------------------------------- #
def _limits(cfg: Config | None) -> tuple[float, float]:
    """射击距离窗口。真源 = `Config`（`path_executor.shoot_distance_min/max`），
    缺省值直接用 `validate.DEFAULT_SHOOT_DISTANCE_*`（不硬编码第二份默认值）。"""
    lo = float(V._cfg_get(cfg, "path_executor.shoot_distance_min",     # noqa: SLF001
                          V.DEFAULT_SHOOT_DISTANCE_MIN_M))
    hi = float(V._cfg_get(cfg, "path_executor.shoot_distance_max",     # noqa: SLF001
                          V.DEFAULT_SHOOT_DISTANCE_MAX_M))
    return (lo, hi) if lo <= hi else (hi, lo)


def _cross_pools(cfg: Config | None, spec: MapSpec) -> tuple[list[str], list[str]]:
    """上下半场穿洞候选格。优先级（与 `validate.py` 的口径一致）：

    ① `Config` 的 `route.cross_lower/upper_candidates`（每局随机、无人机情报选格）
    ② `map.bridges`（本局现场识图结果）—— **非空时与①取交集**：换一张地图就必须用那张
       地图的桥洞，否则会拿上一张图的候选格去撞新图的桥墩（本轮实测踩过：换成
       `bridges: [K1,K7]` 的新图后，`robot.yaml` 的 `K9/K4` 仍然被当成候选，
       路线会横穿非桥洞的 K 列）。
    ③ 交集为空或都没给 → 退到"整列 K 的对应半场"（**不写死任何一格**）
    """
    def split(items: Iterable[str]) -> tuple[list[str], list[str]]:
        lower, upper = [], []
        for cell in items:
            try:
                row_idx, col_idx = geo.block_to_index(str(cell))
            except ValueError:
                continue
            if col_idx != _CROSS_COL:
                continue
            row = row_idx + 1
            if V.LOWER_HALF_ROWS[0] <= row <= V.LOWER_HALF_ROWS[1]:
                lower.append(str(cell))
            elif V.UPPER_HALF_ROWS[0] <= row <= V.UPPER_HALF_ROWS[1]:
                upper.append(str(cell))
        return lower, upper

    lower, upper = split(V._as_list(V._cfg_get(                  # noqa: SLF001
        cfg, "route.cross_lower_candidates", None)))
    upper = upper + split(V._as_list(V._cfg_get(                 # noqa: SLF001
        cfg, "route.cross_upper_candidates", None)))[1]
    b_lower, b_upper = split(spec.bridges)
    if b_lower and lower:
        lower = [c for c in lower if c in set(b_lower)] or list(b_lower)
    if b_upper and upper:
        upper = [c for c in upper if c in set(b_upper)] or list(b_upper)
    lower = lower or b_lower
    upper = upper or b_upper
    if not lower:
        lower = [f"{V.CROSS_COLUMN}{r}"
                 for r in range(V.LOWER_HALF_ROWS[0], V.LOWER_HALF_ROWS[1] + 1)]
    if not upper:
        upper = [f"{V.CROSS_COLUMN}{r}"
                 for r in range(V.UPPER_HALF_ROWS[0], V.UPPER_HALF_ROWS[1] + 1)]
    return sorted(set(lower)), sorted(set(upper))


def _default_cross(cfg: Config | None, pool: Sequence[str], half: str) -> str:
    preferred = str(V._cfg_get(cfg, f"route.cross_{half}_default", "") or "")   # noqa: SLF001
    preferred = preferred.strip().upper()
    if preferred and preferred in pool:
        return preferred
    return sorted(pool)[0]


def plan_route(map_cfg: Any, cfg: Config | None = None, *,
               cross_lower: str | None = None, cross_upper: str | None = None,
               max_seconds: float = DEFAULT_MAX_SECONDS,
               allow_reverse: bool = True,
               pivot_penalty: float | None = None,
               mine_turn_penalty: float | None = None,
               name: str = "planner", source: str = "") -> PlanResult:
    """**核心入口**：一张地图 -> 一条可直接下发的路线。

    :param map_cfg: `config/map.yaml` 的原始 dict（`{'map': {...}}`）；也接受 `MapSpec`
    :param cfg: `core.config.Config`；提供射击窗口、穿洞候选集合、起点朝向（`mission.reset`）
    :param cross_lower/cross_upper: 指定穿洞格（通常由无人机情报给出）；
        缺省用 `route.cross_*_default`；不在候选集合里则抛 `PlannerError`
    :param max_seconds: 时间预算；超出抛 `PlannerError`
    :param allow_reverse: **False = 无倒车模式**（车上 dzbrain 的默认用法）。
        真车底盘不支持倒车（实测：负速度方向位被 STM32 无视），所以无倒车模式：
        ① 搜索动作集里**没有倒车**（`Grid.allow_reverse`）；② **跳过** `_fix_uturn_reverse`
        （不把 180° 折返改写成 `R:`）；③ 给每个折返顶点打 `:PIVOT` 声明（`mark_pivots`）；
        ④ 合法性断言换成 `assert_legal_no_reverse`（0 段 `R:` + 折返声明完整且不虚报）。
        True = 旧行为（`R:` 倒车段处理死角），**默认值不变**（签名向后兼容）。
    :param pivot_penalty: 顺序搜索的折返惩罚（等效代价单位/处）。`None`（缺省）=
        **跟随配置** `planner.pivot_penalty`（键缺失 → 0.0，行为与卡 A 基线一致）。
    :param mine_turn_penalty: 顺序搜索里"**雷格不得作为拐点**"的惩罚权重。
        `None`（缺省）= 跟随配置 `planner.mine_turn_penalty`（键缺失 → `MINE_TURN_PENALTY`）。
        每压一个雷都要多 2 格路（进点/出点），这一项保证"宁可多绕一格，也不让雷被切角擦过"。
    :raises PlannerError: 无解 / 结构约束矛盾 / 超时 / 折返声明不一致（**绝不静默降级**）
    """
    t0 = time.perf_counter()
    spec = map_cfg if isinstance(map_cfg, MapSpec) else load_map(map_cfg, source=source)
    limits = _limits(cfg)
    warnings: list[str] = []
    deadline = t0 + max(0.05, float(max_seconds))
    if pivot_penalty is None:
        pivot_penalty = float(V._cfg_get(cfg, "planner.pivot_penalty", 0.0) or 0.0)   # noqa: SLF001
    pivot_penalty = max(0.0, float(pivot_penalty))
    if mine_turn_penalty is None:
        mine_turn_penalty = float(V._cfg_get(                                          # noqa: SLF001
            cfg, "planner.mine_turn_penalty", MINE_TURN_PENALTY) or 0.0)
    mine_turn_penalty = max(0.0, float(mine_turn_penalty))

    start_cell, end_cell = _cell(spec.start), _cell(spec.end)

    # ---------------- 穿洞格（先定，再建带桥洞约束的栅格） ----------------
    lower_pool, upper_pool = _cross_pools(cfg, spec)
    lower_cross = (cross_lower or _default_cross(cfg, lower_pool, "lower")).strip().upper()
    upper_cross = (cross_upper or _default_cross(cfg, upper_pool, "upper")).strip().upper()
    for cell, pool, half in ((lower_cross, lower_pool, "下半场"),
                             (upper_cross, upper_pool, "上半场")):
        if cell not in pool:
            raise PlannerError(f"{half}穿洞格 {cell} 不在候选集合（{' '.join(pool)}）里"
                               f" —— 候选来自 route.cross_lower/upper_candidates 与 "
                               f"map.bridges")
        row_idx, col_idx = geo.block_to_index(cell)
        if col_idx != _CROSS_COL:
            raise PlannerError(f"穿洞格 {cell} 不在 {V.CROSS_COLUMN} 列上")
    cross_lower_cell, cross_upper_cell = _cell(lower_cross), _cell(upper_cross)
    if cross_lower_cell == cross_upper_cell:
        raise PlannerError(f"下半场与上半场选了同一个桥洞 {lower_cross}（必须各穿一次）")

    grid = Grid(spec, bridge_rows=(cross_lower_cell[0], cross_upper_cell[0]),
                allow_reverse=allow_reverse)
    if not grid.passable(*start_cell):
        raise PlannerError(f"起点 {spec.start} 是障碍格，无法出发")
    if not grid.passable(*end_cell):
        raise PlannerError(f"终点 {spec.end} 是障碍格，无法到达")
    for cell, name, half in ((cross_lower_cell, lower_cross, "下半场"),
                             (cross_upper_cell, upper_cross, "上半场")):
        if not grid.passable(*cell):
            raise PlannerError(f"{half}穿洞格 {name} 是障碍格，无法通行")

    # ---------------- 起点朝向（真源：mission.reset，如 "N10,y"） ----------------
    start_heading = 1                                  # 默认朝西（列号减小）
    reset_spec = str(V._cfg_get(cfg, "mission.reset", "") or "")      # noqa: SLF001
    if reset_spec:
        try:
            reset_block, reset_yaw = parse_reset(reset_spec)
            if reset_block == spec.start:
                start_heading = 1 if abs(reset_yaw - geo.HEADING_Y) < 1e-6 else 0
            else:
                warnings.append(f"mission.reset={reset_spec} 的起点 {reset_block} 与地图起点 "
                                f"{spec.start} 不一致，起点朝向按向西处理")
        except ValueError:
            warnings.append(f"mission.reset={reset_spec!r} 无法解析，起点朝向按向西处理")
    drow, dcol = DIRS[start_heading]
    if not grid.move_ok(start_cell[0], start_cell[1],
                        start_cell[0] + drow, start_cell[1] + dcol):
        start_heading = 0
        warnings.append(f"起点 {spec.start} 朝西不可行（前方是障碍/出界/非桥洞行的 K 列），"
                        f"起点朝向改为朝 X+")

    # ---------------- U 形结构可行性 ----------------
    # 下半场：起点在桥东 -> 穿 K(lower) 向西 -> 收站；上半场：收站 -> 穿 K(upper) 向东 -> 终点。
    # 于是两条必要条件是"起点/终点必须在各自桥洞的东侧"，否则必然要穿两次（违反 R007）。
    if start_cell[1] < _CROSS_COL:
        raise PlannerError(
            f"起点 {spec.start}（列索引 {start_cell[1]}）不在下半场桥洞 {lower_cross}"
            f"（列索引 {_CROSS_COL}）以东 —— 与「每半场只穿一次桥洞」（validate R007）"
            f"矛盾：开到西侧再回东侧起点就必须穿两次。请检查地图或穿洞候选")
    if end_cell[1] < _CROSS_COL:
        raise PlannerError(
            f"终点 {spec.end}（列索引 {end_cell[1]}）不在上半场桥洞 {upper_cross}"
            f"（列索引 {_CROSS_COL}）以东 —— 与「每半场只穿一次桥洞」（validate R007）"
            f"矛盾。请检查地图或穿洞候选")

    # ---------------- 必访点：雷按半场分组 ----------------
    mines_lower, mines_upper = [], []
    for mine in spec.mines:
        cell = _cell(mine)
        if not grid.passable(*cell):
            raise PlannerError(f"雷 {mine} 落在障碍格上（地图自相矛盾），无法压雷")
        (mines_lower if grid.zone_of(cell[0]) == 1 else mines_upper).append(cell)

    shoot_pool: list[str] = []
    if spec.targets:
        pool = shoot_candidates(spec, grid, limits)
        if not pool:
            raise PlannerError(f"靶 {' '.join(spec.targets)} 周围没有落在窗口 "
                               f"{limits[0]:.2f}~{limits[1]:.2f} m 内的可通行射击点")
        cov = covering_stops(spec, pool, limits)
        shoot_pool = [b for b in pool if len(cov[b]) == len(spec.targets)]
        if not shoot_pool:
            detail = "; ".join(f"{b}->{'/'.join(cov[b]) or '无'}" for b in pool)
            raise PlannerError(
                f"靶区 {' '.join(spec.targets)} 找不到**一个**能同时覆盖全部 "
                f"{len(spec.targets)} 格靶的射击停站点（route_check 会判「靶覆盖 "
                f"{len(spec.targets) - 1}/{len(spec.targets)}」）。候选与覆盖：{detail}。"
                f"若这是现场实际布局，请确认 targets 是否写成了不相邻的两格，"
                f"或射击窗口 {limits[0]:.2f}~{limits[1]:.2f} m 是否需要现场标定")

    # ---------------- 顺序 + 朝向联合优化 ----------------
    #   ★ "雷格不得作拐点"的惩罚同样只在无倒车模式下生效（理由同上：允许倒车 = 旧行为）
    mine_cells = set() if allow_reverse else {_cell(m) for m in spec.mines}
    tour = _Tour(grid, deadline, pivot_penalty=pivot_penalty,
                 mine_cells=mine_cells, mine_turn_penalty=mine_turn_penalty)
    tour.build([start_cell] + mines_lower + mines_upper
               + [cross_lower_cell, cross_upper_cell, end_cell]
               + [_cell(b) for b in shoot_pool])

    lower_seq = tour.two_opt(tour.nearest(start_cell, mines_lower + [cross_lower_cell])) \
        if mines_lower else [cross_lower_cell]
    if cross_lower_cell not in lower_seq:              # 2-opt 不会丢点，纯防御
        lower_seq.append(cross_lower_cell)
    upper_nodes = mines_upper + [cross_upper_cell]
    upper_seq = tour.two_opt(tour.nearest(lower_seq[-1], upper_nodes)) if upper_nodes \
        else [cross_upper_cell]
    order = [start_cell] + lower_seq + upper_seq + [end_cell]

    heads = tour.best_headings(order, fixed={start_cell: start_heading})
    order, heads, _ = tour.polish(order, heads, rounds=MAX_TOUR_ROUNDS - 1)
    if order[0] != start_cell or order[-1] != end_cell:
        raise PlannerError("内部错误：优化后起点/终点被挪动")
    heads[0] = start_heading                          # 起点朝向由 mission.reset 锁死

    # ---------------- 射击停站：插到"绕路代价最小"的位置 ----------------
    shoot_block = ""
    if shoot_pool:
        order, heads, shoot_block = _insert_shoot(tour, order, heads, shoot_pool)

    # ---------------- ★ 雷心直线穿越（用户 2026-10-11：车心必须压过每个雷心）----------------
    #   把每个雷格补成"进点—雷格—出点"三点直线的中点；插点后顺序变了，朝向必须重算。
    #   ★ 只在**无倒车模式**下做：那是车上真正跑的路线（真车底盘不支持倒车）；
    #     `allow_reverse=True` 保持旧行为，避免"允许倒车"这条诊断路径的输出被顺带改掉
    #     （也符合卡 A/卡 C「允许倒车 = 旧行为」的语义）。
    mine_axes: list[tuple[str, int, bool]] = []
    if not allow_reverse:
        order, mine_axes = straighten_mines(order, spec, grid, tour)
        heads = tour.best_headings(order, fixed={order[0]: start_heading})
        for _block, _axis, _ok in mine_axes:
            if not _ok:
                warnings.append(
                    f"雷 {_block} 找不到可行的直线穿越轴（进/出点被障碍挡住或落在 K 列上）"
                    f"—— 该雷仍会被切角擦过，需人工核对地图")

    # ---------------- 编译成航点 ----------------
    keep = {start_cell, end_cell, cross_lower_cell, cross_upper_cell}
    keep |= set(mines_lower) | set(mines_upper)
    if shoot_block:
        keep.add(_cell(shoot_block))

    stops: list[_Stop] = [_Stop(start_cell[0], start_cell[1], spec.start,
                                heading=start_heading, tag="start")]
    cost = 0.0
    for i in range(len(order) - 1):
        a, b = order[i], order[i + 1]
        path, seg_cost = tour.path(a, b, heads[i], heads[i + 1])
        if not path:
            raise PlannerError(f"无解：{geo.index_to_block(a[0] - 1, a[1])} -> "
                               f"{geo.index_to_block(b[0] - 1, b[1])}"
                               f"（出发朝向 {HEAD_NAMES[heads[i]]} / "
                               f"到达朝向 {HEAD_NAMES[heads[i + 1]]}）")
        cost += seg_cost
        for (row, col, face, back) in _collapse(path, keep):
            if (row, col) == (stops[-1].row, stops[-1].col):
                stops[-1].reverse = back
                stops[-1].heading = face
                continue
            block = geo.index_to_block(row - 1, col)
            stops.append(_Stop(row, col, block, heading=face, reverse=back,
                               tag="mine" if block in set(spec.mines) else "path"))
        if time.perf_counter() > deadline:
            raise PlannerError(f"规划超时（>{max_seconds:.2f} s），未生成完整路线")

    if (stops[0].row, stops[0].col) != start_cell:
        raise PlannerError("内部错误：路线首点不是起点")
    if (stops[-1].row, stops[-1].col) != end_cell:
        raise PlannerError("内部错误：路线末点不是终点")

    # ---------------- 长段插点 + 标记 + 180° 折返改写 ----------------
    stops = _split_long_hops(stops, grid)
    for stop in stops:
        if (stop.row, stop.col) == cross_lower_cell:
            stop.tag = "lower_bridge"
        elif (stop.row, stop.col) == cross_upper_cell:
            stop.tag = "upper_bridge"
    if shoot_block:
        for stop in stops:
            if stop.block == shoot_block:
                stop.shoot = True
                stop.tag = "shoot"
                break
        else:                                   # pragma: no cover - 防御性
            raise PlannerError(f"内部错误：射击点 {shoot_block} 不在编译后的路线里")

    uturn_fixes = _fix_uturn_reverse(stops) if allow_reverse else 0
    stops = _merge_duplicate_stops(stops)
    # ★ 折返改写会改变"哪一段在倒车"，所以必须**在改写之后**重算车头朝向
    #   （原地掉头与倒车掉头的车头正好相反，弄错会让终点姿态与云台基准全错）。
    #   无倒车模式没有改写，但朝向重算对"正向开进死格再正向开出"同样必需
    #   （顶点处 180° 换向由 TrackerTask PIVOT 完成，航点序列本身保持前进语义）。
    _refresh_headings(stops)

    # ★ `:PIVOT` 折返点声明：**只在无倒车模式**下打（允许倒车模式的折返由 `R:` 完成，
    #   声明它反而自相矛盾，且会让 `--allow-reverse` 输出偏离卡 A 基线）。
    #   标记必须在 `_merge_duplicate_stops` 之后（索引已冻结）。
    marked: list[tuple[int, str, float]] = []
    if not allow_reverse:
        marked = mark_pivots(stops)

    # ★【自诊断】逐雷"雷前共线格数"：<2 的雷在纯追踪下会被切角，车心压不到雷心。
    #   只报一条汇总 warning（每个雷一行会把启动日志淹掉），数据同时进 `PlanResult.mine_runs`。
    mine_runs: list[tuple[str, int]] = []
    if not allow_reverse:
        mine_runs = _mine_aligned_runs(stops, spec)
        short = [(b, r) for b, r in mine_runs if r < 2]
        if short:
            warnings.append(
                "雷心压过风险：%d/%d 个雷的进点前共线不足 2 格（%s）—— "
                "纯追踪要 ~2 格（0.7 m）才贴回参考线，这些雷会被切角，"
                "车心到雷心实测可差 3~8 cm（t8/t11 口径）。"
                "要真正消除需要改访问顺序（会额外增加 PIVOT/里程），"
                "当前按『不动路线』处理，仅提示。"
                % (len(short), len(mine_runs),
                   " ".join("%s=%d" % (b, r) for b, r in short)))

    crossing = {"lower": lower_cross, "upper": upper_cross}
    route = _to_route(stops, name=name, crossing=crossing,
                      final_heading=_final_heading(stops))

    problems = check_route_structure(route, spec, grid, limits)
    if problems:
        raise PlannerError("规划结果未通过结构自检：\n  - " + "\n  - ".join(problems))
    if allow_reverse:
        assert_legal(route)
    else:
        assert_legal_no_reverse(route)

    tokens = route_tokens(route)
    shoot_at = next((wp.block for _, wp in route.shoot_points()), "")
    shoot_dist = 0.0
    shoot_targets: tuple[str, ...] = ()
    if shoot_at:
        acc_lo, acc_hi = accept_window(limits)
        pairs = sorted((stop_distance_m(shoot_at, t), t) for t in spec.targets)
        in_window = [(d, t) for d, t in pairs if acc_lo - 1e-9 <= d <= acc_hi + 1e-9]
        shoot_dist, _ = in_window[0] if in_window else pairs[0]
        shoot_targets = tuple(t for d, t in in_window if abs(d - shoot_dist) < 1e-6)
    polyline = sum(math.hypot(route.waypoints[i + 1].x - route.waypoints[i].x,
                              route.waypoints[i + 1].y - route.waypoints[i].y)
                   for i in range(len(route.waypoints) - 1))

    return PlanResult(
        route=route, tokens=tokens, stops=tuple(stops), crossing=crossing,
        shoot_stop=shoot_at, shoot_distance_m=float(shoot_dist),
        shoot_targets=shoot_targets, cost=cost, expanded=tour.settled,
        elapsed_ms=(time.perf_counter() - t0) * 1000.0,
        reverse_segments=sum(1 for wp in route.waypoints if wp.reverse),
        uturn_fixes=uturn_fixes, warnings=tuple(warnings), map_name=spec.name,
        map_source=spec.source, start=spec.start, end=spec.end,
        polyline_m=polyline, start_heading=HEAD_NAMES[start_heading],
        allow_reverse=bool(allow_reverse), pivots=tuple(marked),
        pivot_penalty=float(pivot_penalty),
        mine_turn_penalty=float(mine_turn_penalty), mine_axes=tuple(mine_axes),
        mine_runs=tuple(mine_runs),
        tour_cost=tour.order_cost_total(order),
    )


def _insert_shoot(tour: _Tour, order: list[tuple[int, int]], heads: Sequence[int],
                  pool: Sequence[str]) -> tuple[list[tuple[int, int]], list[int], str]:
    """把射击停站插到"绕路代价最小"的位置，返回 (新顺序, 新朝向, 选中的射击格)。

    绕路代价 = `m(prev,stop) + m(stop,next) - m(prev,next)`（不新增任何阈值），
    再加上**折返惩罚的增量**（插入点可能把一处顺路访问变成"进去再退出来"），
    然后用朝向轮重算一次朝向。平局用 (绕路代价, 插入位置, 格名) 全序决定 —— **确定性**。
    """
    positions = range(1, len(order))
    best: tuple[float, int, str] | None = None
    base_penalty = tour.penalty_of(order)
    for block in pool:
        cell = _cell(block)
        if cell in order:
            detour, pos = 0.0, order.index(cell)
        else:
            scored = []
            for i in positions:
                prev_cell, next_cell = order[i - 1], order[i]
                value = (tour.order_cost.get((prev_cell, cell), float("inf"))
                         + tour.order_cost.get((cell, next_cell), float("inf"))
                         - tour.order_cost.get((prev_cell, next_cell), 0.0))
                if math.isfinite(value) and tour.pivot_penalty > 0.0:
                    trial = list(order)
                    trial.insert(i, cell)
                    value += tour.penalty_of(trial) - base_penalty
                scored.append((value, i))
            detour, pos = min(scored, key=lambda item: (item[0], item[1]))
        if not math.isfinite(detour):
            continue
        key = (detour, pos, block)
        if best is None or key < best:
            best = key
    if best is None:
        raise PlannerError("没有可用的射击停站点（全部绕路代价无穷）")

    _, pos, block = best
    cell = _cell(block)
    new_order = list(order)
    if cell not in new_order:
        new_order.insert(pos, cell)
    new_heads = tour.best_headings(new_order, seed=None, fixed={new_order[0]: heads[0]})
    return new_order, new_heads, block


# --------------------------------------------------------------------------- #
#  ★ 雷心直线穿越（用户 2026-10-11 口径：车心必须压过每个雷格中心）
# --------------------------------------------------------------------------- #
def _axis_pass_ok(grid: Grid, cell: tuple[int, int], d: tuple[int, int]
                  ) -> tuple[bool, tuple[int, int], tuple[int, int]]:
    """沿方向 `d` 能否做"进点—雷—出点"直线穿越，返回 (可行, 进点, 出点)。

    三条硬约束（每条都对应一条既有规则，不新造判据）：
      ① 进/出点必须可通行（障碍不可通行）；
      ② 进/出点**不能在 K 列**：否则会平白多一次 K 列穿行（R007 要求每半场恰好一次）；
      ③ 进/出两步不得非法跨越第 6/7 行隔板（只有 A/B/C 列能过，R003）。

    ★ 曾经还要求"雷前第 2 格（起跑点）也通行"并把它插进航点序列，实测**更差**：
      A* 会先绕到起跑点再折回，PIVOT 从 1 个涨到 5 个、里程 19.95→23.45 m。
      车心到雷心的真值偏差改用"速度/前视"扫描来压（t11 报告 §A3），不靠插起跑点。
    """
    m_in = (cell[0] - d[0], cell[1] - d[1])
    m_out = (cell[0] + d[0], cell[1] + d[1])
    for c in (m_in, m_out):
        if not grid.passable(*c):
            return False, m_in, m_out
        if c[1] == _CROSS_COL:
            return False, m_in, m_out
        if not grid.barrier_cross_ok(cell[0], c[0], c[1]):
            return False, m_in, m_out
    return True, m_in, m_out


def _dir_turn_cost(from_dir: int, to_dir: int) -> float:
    """朝向索引 `from_dir -> to_dir` 的转角代价：0° = 0，90° = 1，180° = 100（几乎禁止）。

    180° 之所以重罚：无倒车模式下它必然是一次原地 PIVOT（实测 ~10.8 s、横向扫出 ~0.54 m），
    而且 `pursuit.pivot_dist_m=0.25` 让车在距顶点 0.25 m 处就掉头 —— 只要 180° 落在
    雷格**旁边那一格**，同样会把雷心的压过精度毁掉。所以选轴时必须把"新插点处会不会
    出现折返"一并计进去。
    """
    delta = (int(from_dir) - int(to_dir)) % 4
    if delta == 0:
        return 0.0
    if delta == 2:
        return 100.0
    return 1.0


def _aligned_run_before(tour: "_Tour", prev_cell: tuple[int, int] | None,
                        m_in: tuple[int, int], axis: int) -> int:
    """进点 `m_in` **之前**还有几格是沿 `axis` 共线的（0/1/2，2 表示"够长"）。

    为什么必须够长：纯追踪在 90° 转向后要 ~2 格（0.7 m）才重新贴回参考线。若雷格
    前面只有 1 格直线（即转向点就在进点处），车心到雷心实测能差 **14~19 cm**
    （t11 第一版实测 B3 0.1409 / F3 0.1933 m）——比"在雷格上切角擦过"（2.7~3.1 cm）更糟。
    所以"雷前 ≥2 格共线"是压雷的必要条件，不是可选项。
    """
    if prev_cell is None:
        return 0
    table = tour.head_cost.get((prev_cell, m_in))
    best: tuple[float, int, int] | None = None
    if table is not None:
        for ha in range(4):
            for hb in range(4):
                value = table[ha][hb]
                if not math.isfinite(value):
                    continue
                if best is None or value < best[0] - 1e-12:
                    best = (value, ha, hb)
    if best is None:
        return 0
    path, _cost = tour.path(prev_cell, m_in, best[1], best[2])
    dirs: list[int] = []
    for prev, cur in zip(path, path[1:]):
        if (cur[0], cur[1]) != (prev[0], prev[1]):
            dirs.append(_heading_of(cur[0] - prev[0], cur[1] - prev[1], -1))
    run = 0
    for d in reversed(dirs):
        if d == axis:
            run += 1
        else:
            break
    return run


def _mine_aligned_runs(stops: Sequence["_Stop"], spec: MapSpec) -> list[tuple[str, int]]:
    """【自诊断】逐雷量"雷格前有几格是共线的"（编译后路线口径，单位 = 格）。

    口径：从雷格往前数，**方向不变**的连续格数（含紧邻的那一格）。
    `1` = 转向点就压在雷前那一格上 —— 纯追踪此时还在切角，车心压不到雷心；
    `>= 2` = 车进雷格之前已经有 ~0.7 m 直线，够贴回参考线。

    为什么放在规划器里：这条是**真车丢分的直接原因**（漏压雷 = 直接丢分），
    但以前只有跑完仿真才知道（t8 §3.5 / t11 §A3 都是事后测量）。现在规划器自己
    把风险报出来（`PlanResult.mine_runs` + 一条 warning），新地图不用先上车就能看出
    哪几个雷压不实。判据本身不发明阈值：`MINE_PRESS_TOL_M` 与 t11 的实测口径一致。
    """
    cells = [(stop.row, stop.col) for stop in stops]
    out: list[tuple[str, int]] = []
    for name in spec.mines:
        cell = _cell(name)
        if cell not in cells:
            out.append((name, 0))
            continue
        idx = cells.index(cell)
        if idx == 0 or idx >= len(cells) - 1:
            out.append((name, 0))
            continue
        step_in = (cell[0] - cells[idx - 1][0], cell[1] - cells[idx - 1][1])
        step_out = (cells[idx + 1][0] - cell[0], cells[idx + 1][1] - cell[1])
        if step_in != step_out or step_in == (0, 0):
            out.append((name, 0))          # 雷格是拐点（不是直线穿越）→ 必然切角
            continue
        run = abs(step_in[0]) + abs(step_in[1])
        k = idx - 1
        while k > 0:
            prev = (cells[k][0] - cells[k - 1][0], cells[k][1] - cells[k - 1][1])
            if prev != step_in:
                break
            run += abs(prev[0]) + abs(prev[1])
            k -= 1
        out.append((name, run))
    return out


def straighten_mines(order: list[tuple[int, int]], spec: MapSpec, grid: Grid,
                     tour: "_Tour"
                     ) -> tuple[list[tuple[int, int]], list[tuple[str, int, bool]]]:
    """把每个雷格变成"进—雷—出"三点直线的**中点**，返回 (新顺序, 穿越轴清单)。

    为什么必须动顺序而不是只动编译：雷格上的转角是**拓扑**决定的 —— 只要雷格是
    访问顺序里的一个拐点，纯追踪就会在它上面切角（实测 2.7~3.1 cm）；若是 180°
    折返点，车更是在距顶点 0.25 m 处就掉头（`pursuit.pivot_dist_m`），车心压不到雷心
    （A9 实测差 12.7 cm）。这里在顺序里给雷格补上**进点**与**出点**（各离雷格一格、
    与雷格共线），于是编译出的折线在雷格处是**直线**：

        …, 进点, 雷格, 出点, …        （进点 → 雷格 → 出点 共线）

    轴的选择（确定性）：对每个可行轴算"进点处的转角 + 出点处的转角"（180° 记 100、
    90° 记 1、直行记 0，见 `_dir_turn_cost`），取代价最小者；平局取轴编号小者。
    三条硬约束见 `_axis_pass_ok()`。

    :return: ``(新顺序, [(雷格名, 轴编号(0北1西2南3东), 是否找到可行轴), …])``
        —— 找不到可行轴时**如实记 False**，由 `plan_route` 报 warning，绝不假装压到了。
    """
    mines = {_cell(m) for m in spec.mines}
    out: list[tuple[int, int]] = []
    axes: list[tuple[str, int, bool]] = []
    for idx, cell in enumerate(order):
        if cell not in mines:
            out.append(cell)
            continue
        block = geo.index_to_block(cell[0] - 1, cell[1])
        prev_cell = order[idx - 1] if idx > 0 else None
        next_cell = order[idx + 1] if idx + 1 < len(order) else None
        entry = tour._entry_dir.get((prev_cell, cell)) if prev_cell else None   # noqa: SLF001
        leave = tour._exit_dir.get((cell, next_cell)) if next_cell else None    # noqa: SLF001
        chosen: tuple[tuple[int, float, int], tuple[int, int], tuple[int, int]] | None = None
        for axis in range(4):
            ok, m_in, m_out = _axis_pass_ok(grid, cell, DIRS[axis])
            if not ok:
                continue
            # ① 这个轴会不会在进点/出点**制造 180° 折返**（新 PIVOT，必须优先避免）
            bad = 0
            if prev_cell is not None and prev_cell != m_in:
                tour._record_leg_dirs(prev_cell, m_in)                          # noqa: SLF001
                arrive = tour._entry_dir.get((prev_cell, m_in))                 # noqa: SLF001
                if arrive is not None and (int(arrive) - axis) % 4 == 2:
                    bad = 1
            if next_cell is not None and next_cell != m_out:
                tour._record_leg_dirs(m_out, next_cell)                         # noqa: SLF001
                depart = tour._exit_dir.get((m_out, next_cell))                 # noqa: SLF001
                if depart is not None and (axis - int(depart)) % 4 == 2:
                    bad = 1
            # ② 同分时比**实际路径代价**（进点→雷格→出点 各一格 = 2 格，可忽略不计），
            #    这把"绕远路去压雷"和"走近路但要多打一次舵"放在同一把尺子上量。
            cost = 0.0
            if prev_cell is not None and prev_cell != m_in:
                cost += tour.leg_cost(prev_cell, m_in)
            if next_cell is not None and next_cell != m_out:
                cost += tour.leg_cost(m_out, next_cell)
            # ③ 按 (是否制造 180°, 实际代价, 轴编号) 取最优 —— 全序、确定性
            key = (bad, round(cost, 6), axis)
            if chosen is None or key < chosen[0]:
                chosen = (key, m_in, m_out)
        if chosen is None:
            axes.append((block, -1, False))
            out.append(cell)
            continue
        (_bad, _cost, axis), m_in, m_out = chosen
        # 进点 → 雷格 → 出点（三点共线）
        if m_in != prev_cell and (not out or out[-1] != m_in):
            out.append(m_in)
        out.append(cell)
        if m_out != next_cell:
            out.append(m_out)
        axes.append((block, axis, True))
    # 去重相邻重复点（插点可能与原顺序里的邻居重合）
    dedup: list[tuple[int, int]] = []
    for c in out:
        if not dedup or dedup[-1] != c:
            dedup.append(c)
    return dedup, axes
