# -*- coding: utf-8 -*-
"""WB-4 参数化地图合成器（卡 `WB-4-SWEEP-20261031` · tier-1 验证台配套）。

## 一句话

**给一个种子，造出 N 张「合法且带参考解」的地图**，让"能规划"从孤例变成分布。

```
PYTHONPATH=src/dzbrain python tools/map_synth.py --seed 20261031 --count 30 \
        --out ../artifacts/synth_maps/ --selfcheck
```

每张图落两个文件（**成对**，缺一不可）：

| 文件 | 内容 | 为什么必须有 |
|---|---|---|
| `<名>.yaml` | `map:` 段，`sim.parse_map_yaml` / `yaml.safe_load` 两种解析器都吃 | 喂 planner / route_check / sim |
| `<名>.ref.txt` | **参考解**路线（一行，空白分隔） | 没有它就无法区分「规划器漏解」与「这张图本来就无解」 |

## 三条合法性判据（不是"看起来像张图"）

1. **全格可通行**：除 `obstacles` / `targets` 外的每一格都在 `geometry` 场界内（11×14）；
2. **雷/靶/障落合法格**：格名能过 `geo.block_to_index`，雷不进 K 列（桥洞单格宽，
   列内挪格即撞侧壁，见 `physics.k_cells_sealed_along_column`），靶是**跨格板**
   （两个正交相邻格）；
3. **存在一条人工可行路线** ⇒ 本文件的 `build_reference()` 产出 `ref.txt`，
   且它必须过 `route.validate.validate_route()` 的 **0 error** 与 `tools/route_check.py`
   的 **rc=0**（`--selfcheck` 逐张实跑）。

**判据 3 不成立 ⇒ 该图判「生成非法」（`rc=3` 的 `INVALID` 计数），绝不并入
「规划器无解」** —— 这是本轮反复出现的"假归因"（卡面负例 3）。

## 为什么参考解不交给规划器算

参考解必须是**规划器的前提**，不能是它的产物 —— 否则「规划器有解」就成了循环论证。
所以 `build_reference()` 是**独立的**构造：按半场分区 + 逐雷最近邻 + 一次西穿 / 一次东穿
串起来，每一腿用 `planner.Grid.move_ok()`（**生产栅格**，不另写一套通行判据）做 BFS。
它可能比规划器产物更长 —— 那是故意的：它只需要"存在"，不需要"最优"。

## 逐位可复现（本卡契约）

- 自备 LCG（`DeterministicLcg`），**不**用 `random` 模块 —— 同一 `seed` 同一序列已有先例
  （`tools/sim_world.py::DeterministicNoise` 同款做法与同款理由）；
- 落盘文本里**没有**时间戳 / 路径 / PID；
- 同一 seed 两次跑：全部文件 sha256 逐位相同（卡面 verify 第 2 条）。

## 隔板行**不**参与随机（实测发现，2026-11-01）

卡面写"变隔板行"，但实测 `route/validate.py:225` 的 `_barrier_crossing()` 与
`tools/route_check.py:698` 的 `barrier_centerline()` 读的都是 `core.geometry` 的
**全局常量** `BARRIER_ROW_HI/LO`，**不读地图里的 `map.barrier_row_hi/lo/col_from`**
（只有 `planner.load_map()` 和 `route_check.TrajectoryFieldGate` 读地图声明）。
⇒ 地图声明与校验器口径一改就分叉：同一张图，规划器按声明的墙算、校验器按常量的墙判。
所以本合成器的隔板行/列**固定为现场值**，随机化只落在「桥洞行 / 雷位 / 靶板 / 障碍」上
（这四类**都被**校验器从地图读，不会分叉）。`--vary-barrier` 保留为**诊断开关**，
它会把这张图同时交给两个口径，专门用来复现上面这条不一致（见 `--selfcheck` 输出的
`BARRIER_DECLARED_NOT_HONORED` 行）。

## 退出码

| 码 | 含义 |
|---|---|
| 0 | 全部图合法且参考解通过校验（`--selfcheck` 时逐张实跑 route_check） |
| 2 | 参数/环境问题（`--out` 不可写、`yaml` 缺失等） |
| 3 | 有图被判「生成非法」（重试次数用尽）；清单已落盘，逐张带 `INVALID` 理由 |

## 刻意不做

- 不碰 `config/**` 与 `src/dzbrain/dzbrain/**`（冻结区，零写入）；本工具**只读调用**。
- 不 commit / 不 push / 不部署。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

# --------------------------------------------------------------------------- #
#  sys.path：`src/dzbrain` 在前，`tools` 在后（`tools/dzbrain.py` 会遮住真包）
# --------------------------------------------------------------------------- #
_THIS = pathlib.Path(__file__).resolve()
_TOOLS = _THIS.parent
_REPO = _TOOLS.parent
_SRC = _REPO / "src" / "dzbrain"

for _entry in (str(_TOOLS), str(_REPO)):
    while _entry in sys.path:
        sys.path.remove(_entry)
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
sys.path.append(str(_TOOLS))

from dzbrain.core import geometry as geo                      # noqa: E402
from dzbrain.core.config import Config                        # noqa: E402
from dzbrain.route import planner as PL                       # noqa: E402
from dzbrain.route import validate as V                       # noqa: E402
from dzbrain.route.model import Route, Waypoint, make_route   # noqa: E402

#: 现场固定量（**不随机**，理由见文件头「隔板行不参与随机」）
BARRIER_ROW_HI = geo.BARRIER_ROW_HI
BARRIER_ROW_LO = geo.BARRIER_ROW_LO
BARRIER_COL_FROM = geo.BARRIER_COL_FROM
START_BLOCK = "N10"          # 现场摆位约定（里程计开机硬锚点，见 validate R002 的 warn）
END_BLOCK = "N5"

#: 桥洞列（U 形赛道唯一的两处过洞所在列）与东西两侧分区
CROSS_COL = geo.COL_LETTERS.index(V.CROSS_COLUMN)      # K ⇒ 10
EAST_COLS = tuple(range(CROSS_COL + 1, geo.MAP_COLS))  # L, M, N ⇒ 11..13
WEST_COLS = tuple(range(0, CROSS_COL))                 # A..J ⇒ 0..9

#: 一张图的重试预算（放置 → 构造参考解 → 校验 全过才算合法）
MAX_ATTEMPTS = 40

#: 退出码
RC_OK, RC_USAGE, RC_INVALID = 0, 2, 3


# --------------------------------------------------------------------------- #
#  确定性随机源（自备 LCG，理由同 tools/sim_world.py::DeterministicNoise）
# --------------------------------------------------------------------------- #
class DeterministicLcg:
    """64 位 LCG。同一 seed ⇒ 同一序列；跨 Python 版本可复现（不依赖 `random` 实现细节）。"""

    __slots__ = ("_state",)
    _A = 6364136223846793005
    _C = 1442695040888963407
    _M = 1 << 64

    def __init__(self, seed: int) -> None:
        self._state = (int(seed) * 2862933555777941757 + 3037000493) % self._M

    def next_u64(self) -> int:
        self._state = (self._A * self._state + self._C) % self._M
        return self._state

    def randint(self, lo: int, hi: int) -> int:
        """闭区间 `[lo, hi]`（`hi <= lo` 时返回 `lo`，不抛异常 —— 调用方先算好范围）。"""
        if hi <= lo:
            return lo
        return lo + self.next_u64() % (hi - lo + 1)

    def shuffled(self, items):
        """Fisher–Yates（固定用 `next_u64`，因此与 `random.shuffle` 的序列**不**相同也不需要相同）。"""
        out = list(items)
        for i in range(len(out) - 1, 0, -1):
            j = self.next_u64() % (i + 1)
            out[i], out[j] = out[j], out[i]
        return out


# --------------------------------------------------------------------------- #
#  格 ↔ (行, 列)：**统一用 1-based 行**，与 planner `_cell()` 同口径
# --------------------------------------------------------------------------- #
def cell_of(block: str) -> tuple[int, int]:
    """格名 -> `(行 1-based, 列 0-based)`（与 `planner._cell()` 完全同口径）。"""
    row_idx, col_idx = geo.block_to_index(block)
    return row_idx + 1, col_idx


def block_of(cell: tuple[int, int]) -> str:
    return geo.index_to_block(cell[0] - 1, cell[1])


def _half_of(row: int) -> str:
    """0 = 上半场（`row <= barrier_hi`），1 = 下半场 —— 与 `planner.Grid.zone_of` 同口径。"""
    return "upper" if row <= BARRIER_ROW_HI else "lower"


# --------------------------------------------------------------------------- #
#  一版地图的"声明"（生成阶段全部随机量都在这一个 frozen 结构里）
# --------------------------------------------------------------------------- #
class MapDecl:
    """一张待产地图的全部随机量（写盘前还是可变的，生成器逐次尝试时会被替换）。"""

    def __init__(self, *, name: str, mines: list[str], obstacles: list[str],
                 targets: list[str], bridges: list[str], seed: int, index: int) -> None:
        self.name = name
        self.mines = list(mines)
        self.obstacles = list(obstacles)
        self.targets = list(targets)       # 恰好 2 格，**正交相邻** = 一块跨格靶板
        self.bridges = list(bridges)       # 恰好 2 格：`[下桥洞, 上桥洞]`，都在 K 列
        self.seed = seed
        self.index = index

    # ---------------- 派生 ----------------
    def as_cfg(self) -> dict:
        """转成 `validate_route()` 认的 `map_cfg` 形状（`{'map': {...}}`）。"""
        return {"map": {
            "name": self.name,
            "start": START_BLOCK,
            "end": END_BLOCK,
            "barrier_col_from": BARRIER_COL_FROM,
            "barrier_row_hi": BARRIER_ROW_HI,
            "barrier_row_lo": BARRIER_ROW_LO,
            "mines": list(self.mines),
            "obstacles": list(self.obstacles),
            "targets": list(self.targets),
            "bridges": list(self.bridges),
        }}

    # ★★ 桥洞的"上/下"必须**按行号**判，不能按列表顺序 —— 实测踩到：
    #   现场 `map_field7_t24.yaml` / `map_sixth_20261017.yaml` 写的是
    #   `bridges: ["K5", "K7"]`（**上半场在前**），而本工具生成的合成图是
    #   `[K<下>, K<上>]`（下半场在前）。第一版按 `bridges[0]==lower` 取值，
    #   于是在**真实夹具图**上把 P1 的出口算成"上半场 K5"，参考解当场自相矛盾
    #   （进 P1 是下半场、出 P1 是上半场 ⇒ 必然撞隔板或漏穿洞）。
    #   地图的 `bridges` 顺序是**没有语义**的（只是要素清单），任何按下标取
    #   "哪个是上半场"的代码都是错的。
    @property
    def lower_bridge(self) -> str:
        return sorted(self.bridges, key=lambda b: -cell_of(b)[0])[0]

    @property
    def upper_bridge(self) -> str:
        return sorted(self.bridges, key=lambda b: cell_of(b)[0])[0]

    def spec(self) -> PL.MapSpec:
        return PL.load_map(self.as_cfg(), source=f"<synth:{self.name}>")


# --------------------------------------------------------------------------- #
#  生成：随机量 → MapDecl
# --------------------------------------------------------------------------- #
def _placeable_cells(*, forbid: set[tuple[int, int]] = frozenset(),
                     allow_cols=None, allow_rows=None,
                     exclude_cross_col: bool = True) -> list[tuple[int, int]]:
    """可放要素的格（**保序**：按 `index_to_block` 的字典序，确定性）。"""
    out: list[tuple[int, int]] = []
    for row in range(1, geo.MAP_ROWS + 1):
        for col in range(geo.MAP_COLS):
            if exclude_cross_col and col == CROSS_COL:
                continue                      # K 列 = 桥洞，列内挪格即撞侧壁
            if allow_cols is not None and col not in allow_cols:
                continue
            if allow_rows is not None and row not in allow_rows:
                continue
            if (row, col) in forbid:
                continue
            out.append((row, col))
    return out


#: 过洞走廊（每次穿洞必经的 J/K/L 三格 × 两个桥洞行）—— 由 `generate()` 计算，
#: 见其中"过洞走廊必须空着"的说明。这里单列一个名字，方便测试直接引用。
def corridor_cells(lower_row: int, upper_row: int) -> set[tuple[int, int]]:
    cells: set[tuple[int, int]] = set()
    for row in (lower_row, upper_row):
        for col in (WEST_COLS[-1], CROSS_COL, EAST_COLS[0]):
            cells.add((row, col))
    return cells


def _pick_targets(rng: DeterministicLcg, *, forbid: set[tuple[int, int]] = frozenset(),
                  upper_only: bool = True) -> list[str] | None:
    """造一块**跨格靶板**（两格正交相邻，现场是 35×70 cm 一块板）。

    为什么必须两格：`planner.Grid` 把**每个**靶格都拉黑（不可压过），而
    `shoot_candidates()` 的窗口是按**板心**（两格心中点）算的 —— 单格靶板会让
    「板心」退化成格心，窗口 0.78~1.02 m 里没有格心可落（格距 0.35 的整数倍里
    只有 0.00/0.35/0.70/1.05…，0.70 差 0.08 m 贴不上，1.05 又超上限）⇒ 直接无解。
    现场 0.90±0.12 这个窗口本身就是"两格板"的几何产物。
    """
    rows = range(1, BARRIER_ROW_HI + 1) if upper_only else range(BARRIER_ROW_LO, geo.MAP_ROWS + 1)
    vertical = rng.next_u64() % 2 == 0
    if vertical:                     # 竖向两格：同列、相邻行
        picks = []
        for row in list(rows)[:-1]:
            for col in (c for c in range(geo.MAP_COLS) if c != CROSS_COL):
                if row + 1 in rows and (row, col) not in forbid and (row + 1, col) not in forbid:
                    picks.append([(row, col), (row + 1, col)])
    else:                            # 横向两格：同行、相邻列
        picks = []
        for row in rows:
            for col in range(geo.MAP_COLS - 1):
                if col == CROSS_COL or col + 1 == CROSS_COL:
                    continue
                if (row, col) in forbid or (row, col + 1) in forbid:
                    continue
                picks.append([(row, col), (row, col + 1)])
    if not picks:
        return None
    pair = picks[rng.next_u64() % len(picks)]
    return [block_of(p) for p in pair]


def generate(seed: int, index: int, *, attempt: int = 0,
             barrier_variation: bool = False) -> MapDecl:
    """第 `index` 张图（**同 seed 同 index ⇒ 逐位同一张图**）。

    随机化的四类量（都被校验器**从地图**读取，不会分叉）：

    | 量 | 范围 | 出处 |
    |---|---|---|
    | 桥洞行 | 下 8~11 / 上 2~5（K 列） | `validate` R008 本半场行段 |
    | 雷 | 6~10 颗，全图非 K 列 | R005 必须全部压过 |
    | 障 | 4~10 个 | R004 不可踩 + 净空判据 |
    | 靶 | 1 块跨格板（上半场） | R006/R013 射击窗口 |

    `barrier_variation=True` 时**额外**把隔板行也随机化 —— 这是**诊断**开关，
    会把「规划器按声明 / 校验器按常量」的分叉当场造出来（见文件头）。
    """
    # 每张图的随机流 = 种子 × 序号 × 尝试号混合，**互不干扰**
    #   （第 k 张不依赖第 k-1 张怎么生成；第 k 张的第 a 次尝试也不依赖别的尝试）
    rng = DeterministicLcg((seed * 1000003) ^ (index * 2654435761) ^ (attempt * 40503))

    lower_row = rng.randint(BARRIER_ROW_LO, geo.MAP_ROWS)     # 下桥洞行 7~11
    upper_row = rng.randint(1, BARRIER_ROW_HI)                # 上桥洞行 1~6
    bridges = [f"{V.CROSS_COLUMN}{lower_row}", f"{V.CROSS_COLUMN}{upper_row}"]

    start_cell, end_cell = cell_of(START_BLOCK), cell_of(END_BLOCK)

    # ★★ 过洞走廊**必须空着**（实测踩到的一次真 bug，见 make_one 的报错原文）：
    #   东西两次穿洞都是 `J{row} -> K{row} -> L{row}`，而 `L{row}` 是落到东侧的**唯一**入口
    #   —— 若恰好是靶板格，参考解就"压过靶板"，route_check 直接 ❌（而 `validate_route`
    #   **不查**"航点压靶板"，只有 route_check 查 ⇒ 这条只在自检里才会暴露）。
    #   ⇒ 把三格（J/K/L）× 两个桥洞行 全部钉进 forbid，靶/雷/障都不许落在上面。
    corridor = corridor_cells(lower_row, upper_row)

    targets = _pick_targets(rng, forbid=corridor)
    if targets is None:
        targets = ["D5", "D6"]

    forbid = {start_cell, end_cell} | corridor
    for t in targets:
        forbid.add(cell_of(t))
    pool = _placeable_cells(forbid=forbid)

    n_mines = rng.randint(6, 10)
    n_obs = rng.randint(4, 10)
    shuffled = rng.shuffled(pool)
    mines = [block_of(c) for c in shuffled[:n_mines]]
    obstacles = [block_of(c) for c in shuffled[n_mines:n_mines + n_obs]]

    decl = MapDecl(name=_map_name(seed, index), mines=mines, obstacles=obstacles,
                   targets=list(targets), bridges=bridges, seed=seed, index=index)
    decl.barrier_variation = bool(barrier_variation)
    return decl


def _map_name(seed: int, index: int) -> str:
    return f"synth_s{seed}_n{index:03d}"


# --------------------------------------------------------------------------- #
#  参考解：**独立于规划器**的可行路线构造
# --------------------------------------------------------------------------- #
def _grid_for(decl: MapDecl):
    """带本图两个桥洞的**生产栅格**（通行判据全部来自 `planner.Grid.move_ok`）。"""
    spec = decl.spec()
    rows = [cell_of(decl.lower_bridge)[0], cell_of(decl.upper_bridge)[0]]
    return spec, PL.Grid(spec, bridge_rows=rows, allow_reverse=False)


def _bfs(grid: PL.Grid, a: tuple[int, int], b: tuple[int, int],
         *, no_cross: bool = False) -> list[tuple[int, int]] | None:
    """`a -> b` 的最短格路（只问 `grid.move_ok()`，不另写通行判据）。

    邻居序固定 `(北, 西, 南, 东)` ⇒ 同图同起点必得同一条路（确定性）。

    `no_cross=True` 时**禁止**任何跨 K 列的移动 —— 这是 R007 的硬要求：
    「下半场应**恰好** 1 次穿越 K 列 / 上半场应恰好 1 次」。本合成器的参考解
    在**每个半场分区内部**串腿时必须用 `no_cross=True`，否则 BFS 会为了省两格
    直接从 L 列跳到 K 列再跳回来，造出 3 次穿洞（实测：`field7 + N2` 的参考解
    自己就吃了 2 条 R007，而"这条路线人工可行"这件事本身是真的 —— 只是 BFS
    挑了一条不该挑的省路）。
    """
    if a == b:
        return [a]
    order = ((1, 0), (0, -1), (-1, 0), (0, 1))
    prev: dict[tuple[int, int], tuple[int, int] | None] = {a: None}
    queue = [a]
    head = 0
    while head < len(queue):
        cur = queue[head]
        head += 1
        for dr, dc in order:
            nxt = (cur[0] + dr, cur[1] + dc)
            if nxt in prev:
                continue
            if not grid.passable(*nxt):
                continue
            if no_cross and cur[1] != nxt[1] and CROSS_COL in (cur[1], nxt[1]):
                continue                      # 一步跨过 K 列 = 一次穿洞，分区内不许
            # `move_ok` 的前置断言要求正交相邻 —— BFS 只产出单位步，天然成立
            if not grid.move_ok(cur[0], cur[1], nxt[0], nxt[1]):
                continue
            prev[nxt] = cur
            if nxt == b:
                queue = [a]
                head = len(queue)       # 直接收工（BFS 首达即最短）
                break
            queue.append(nxt)
    if b not in prev:
        return None
    out: list[tuple[int, int]] = []
    cur: tuple[int, int] | None = b
    while cur is not None:
        out.append(cur)
        cur = prev[cur]
    return list(reversed(out))


def _nearest_order(grid: PL.Grid, start: tuple[int, int],
                   stops: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """逐站**最近邻**排序（并列时按 (行,列) 升序 ⇒ 确定性）。

    为什么不用最优顺序：参考解只需要"存在"，不需要"最优"。最近邻在
    8 颗雷的规模上已经足够短，且**没有**任何搜索预算问题。
    """
    remaining = sorted(stops)
    cur = start
    out: list[tuple[int, int]] = []
    while remaining:
        best = None
        best_len = None
        for cand in remaining:
            leg = _bfs(grid, cur, cand, no_cross=True)
            key = (len(leg) if leg else 10 ** 9, cand)
            if best_len is None or key < best_len:
                best, best_len = cand, key
        out.append(best)
        remaining.remove(best)
        cur = best
    return out


def _walk_phase(grid: PL.Grid, entry: tuple[int, int],
                mines: list[tuple[int, int]], exit_cell: tuple[int, int],
                *, tail_stop: tuple[int, int] | None = None
                ) -> list[tuple[int, int]] | None:
    """一个半场分区内的串行腿：`entry -> 各雷(最近邻) -> [tail_stop] -> exit`。

    `tail_stop`（= 本区的射击停站）**排在雷之后、exit 之前**：射击停站必须是
    路线上真实到过的一格（`validate` R006/R013 判的是"某个 `*` 停站点到靶的距离"），
    所以参考解必须**经过**它，不能只在旁边绕过去。
    """
    route = [entry]
    cur = entry
    for stop in _nearest_order(grid, entry, mines):
        leg = _bfs(grid, cur, stop, no_cross=True)
        if leg is None:
            return None
        route += leg[1:]
        cur = stop
    if tail_stop is not None and tail_stop != cur:
        leg = _bfs(grid, cur, tail_stop, no_cross=True)
        if leg is None:
            return None
        route += leg[1:]
        cur = tail_stop
    leg = _bfs(grid, cur, exit_cell, no_cross=True)
    if leg is None:
        return None
    route += leg[1:]
    return route


def _up_column(grid: PL.Grid) -> int | None:
    """A/B/C 列里第一条**上下两行都通行**的列（隔板缺口所在列）。"""
    for col in range(0, geo.COL_LETTERS.index(BARRIER_COL_FROM)):
        if grid.passable(BARRIER_ROW_LO, col) and grid.passable(BARRIER_ROW_HI, col):
            return col
    return None


def _shoot_stop(spec: PL.MapSpec, grid: PL.Grid, cfg: Config | None) -> str | None:
    """射击停站候选（**复用生产** `planner.shoot_candidates()`，不另算窗口）。"""
    accept = None
    if cfg is not None:
        mid = cfg.get("route.shoot_distance_m")
        tol = cfg.get("route.shoot_distance_tol_m")
        if mid is not None and tol is not None:
            accept = (float(mid), float(tol))
    pool = PL.shoot_candidates(spec, grid, accept)
    return pool[0] if pool else None


def build_reference(decl: MapDecl, cfg: Config | None = None) -> tuple[list[str] | None, str]:
    """造参考解。返回 `(token 列表, 失败理由)`；成功时理由为空串。

    结构（与现场 U 形赛道同构，`validate` R007/R012 逐条对着写）：

    ```
    P1 下半场东侧(L~N) 含起点N10 → 走到 L{下桥洞行}
    西穿: L{bl} → K{bl} → J{bl}
    P2 下半场西侧(A~J) 逐雷 → 走到 (第7行, A/B/C 列某列)
    北上: (7,c) → (6,c)                     ← 唯一一处跨隔板
    P3 上半场西侧(A~J) 逐雷 → 走到 J{上桥洞行}
    东穿: J{bu} → K{bu} → L{bu}
    P4 上半场东侧(L~N) 逐雷 → 停站(带 *) → 终点N5
    ```
    """
    spec, grid = _grid_for(decl)
    if not grid.passable(*cell_of(START_BLOCK)) or not grid.passable(*cell_of(END_BLOCK)):
        return None, "起/终点格不可通行"
    for m in decl.mines:
        if not grid.passable(*cell_of(m)):
            return None, f"雷 {m} 落在不可通行格"
    if not spec.targets:
        return None, "没有靶"
    stop_block = _shoot_stop(spec, grid, cfg)
    if not stop_block:
        return None, "没有任何射击停站候选（靶板窗口内无可通行格）"
    stop_rc = cell_of(stop_block)

    # ---- 雷按半场分区（K 列不放雷，所以只有四块）----
    bl = cell_of(decl.lower_bridge)[0]
    bu = cell_of(decl.upper_bridge)[0]
    buckets: dict[str, list[tuple[int, int]]] = {"P1": [], "P2": [], "P3": [], "P4": []}
    for m in decl.mines:
        rc = cell_of(m)
        half = _half_of(rc[0])
        east = rc[1] in EAST_COLS
        key = ("P4" if half == "upper" else "P1") if east else ("P3" if half == "upper" else "P2")
        buckets[key].append(rc)

    # ---- P1：起点 → 逐雷 → L{bl} ----
    route = _walk_phase(grid, cell_of(START_BLOCK), buckets["P1"], (bl, EAST_COLS[0]))
    if route is None:
        return None, "P1(下半场东侧) 无法串成可行腿（障碍堵死或雷不可达）"
    # ---- 西穿（K 列单格：只在桥洞行进出，R007/R009）----
    for col in (CROSS_COL, WEST_COLS[-1]):
        route.append((bl, col))
    # ---- P2：J{bl} → 逐雷 → 第7行缺口列 ----
    upcol = _up_column(grid)
    if upcol is None:
        return None, "A/B/C 列没有任何一列能上穿隔板"
    phase2 = _walk_phase(grid, (bl, WEST_COLS[-1]), buckets["P2"], (BARRIER_ROW_LO, upcol))
    if phase2 is None:
        return None, "P2(下半场西侧) 无法串成可行腿"
    route += phase2[1:]
    # ---- 北上（唯一一次跨隔板，落在 A/B/C 列）----
    route.append((BARRIER_ROW_HI, upcol))
    # ---- P3：上半场西侧 → 逐雷 → J{bu} ----
    phase3 = _walk_phase(grid, (BARRIER_ROW_HI, upcol), buckets["P3"], (bu, WEST_COLS[-1]),
                         tail_stop=(None if stop_rc[1] in EAST_COLS else stop_rc))
    if phase3 is None:
        return None, "P3(上半场西侧) 无法串成可行腿"
    route += phase3[1:]
    # ---- 东穿 ----
    for col in (CROSS_COL, EAST_COLS[0]):
        route.append((bu, col))
    # ---- P4：L{bu} → 逐雷 → **停站** → 终点 ----
    #   ★ 停站排在本区：若把 `stop_rc` 放在别的区，那条腿就变成"绕进去再绕出来"，
    #     很容易把整条参考解拖长一倍；实测把停站按"离哪个入口近"归区才是短的那条。
    #   ★ 停站归区**按列**（不看"离谁近"）：上半场西侧走 P3、东侧走 P4 —— 一条腿的
    #     入口与出口都是该区自己的格，把别区的格塞进来就必然多绕一整个半场。
    stop_in_p4 = stop_rc[1] in EAST_COLS
    phase4 = _walk_phase(grid, (bu, EAST_COLS[0]), buckets["P4"], cell_of(END_BLOCK),
                         tail_stop=(stop_rc if stop_in_p4 else None))
    if phase4 is None:
        return None, "P4(上半场东侧) 无法串成可行腿"
    route += phase4[1:]

    # 去重相邻同格（起点可能被两条腿各带一次）
    dedup: list[tuple[int, int]] = []
    for rc in route:
        if dedup and dedup[-1] == rc:
            continue
        dedup.append(rc)
    if stop_rc not in dedup:
        return None, f"射击停站 {stop_block} 不在参考解走廊上"

    tokens = [block_of(rc) for rc in dedup]
    # 停站打 `*`（token 语法：`<格>*`）
    for i, tok in enumerate(tokens):
        if tok == stop_block and i > 0:
            tokens[i] = f"{tok}*"
            break
    # 终点朝向后缀（现场 `N5,y` 是合法终点朝向声明，见 parser._CANON_TAIL_RE）
    tokens[-1] = f"{tokens[-1]},y" if not tokens[-1].endswith("*") else f"{tokens[-1]},y"
    return tokens, ""


## ★★ 不做「同向压缩」—— 这是实测踩出来的，不是保守选择
#
# 第一版曾把连续同向的格压成两端（"直段只留拐点"，与 parser 的语义一致），
# 结果 3 张图里 2 张**立刻**被自己的校验器判红：
#   · `R004 第 12 个航点 L4 踩到障碍格`（压缩把绕过障碍的那一格删了）；
#   · `R005 雷覆盖 5/7，漏 D1 H4`（压缩把必须**压过**的雷格删了）。
# 根因是同一个：**"几何上同向"不等于"这一路上的格可以被跳过"** ——
# 雷必须逐格经过（R005 按格心判），障碍绕行往往是"同向三步再拐回"，
# 压缩后折线依然直达、但经过的格不再出现在 token 里，而校验器/规划器对
# 雷的覆盖判据是**几何折线**、对障碍的判据也是折线 —— 巧的是两者都会变。
# ⇒ 本合成器**输出逐格路线**（一格一 token）。代价：token 数从 ~60 涨到 ~400；
#   收益：没有任何一条判据依赖"压缩是否安全"这个隐含假设。R011 的段长上限
#   （2.5 格）对逐格路线永远不触发，所以也不会引入新的 warn。


def reference_route(decl: MapDecl, tokens: list[str]) -> Route:
    """token 列表 → `Route`（走生产 `make_route`，朝向按"离开段方向"补齐）。"""
    wps = []
    for tok in tokens:
        block = tok.rstrip("*").split(",")[0]
        wps.append(Waypoint.at_block(block, shoot=tok.endswith("*"),
                                     token=tok))
    return make_route(wps, name=f"{decl.name}.ref")


def validate_reference(decl: MapDecl, tokens: list[str], cfg: Config | None) -> list:
    """参考解的**生产校验**（`validate_route`，与 route_check 同一套规则）。"""
    route = reference_route(decl, tokens)
    return V.validate_route(route, map_cfg=decl.as_cfg(), cfg=cfg)


# --------------------------------------------------------------------------- #
#  合法化：生成 → 参考解 → 校验，全过才算一张图
# --------------------------------------------------------------------------- #
def make_one(seed: int, index: int, cfg: Config | None, *,
             barrier_variation: bool = False) -> tuple[MapDecl | None, list[str], str]:
    """第 `index` 张图。返回 `(决定, 参考解 token, 失败理由)`；失败时决定为 None。

    ⚠️ **不许**为了让它过就把判据放宽（卡面：不得为求绿放宽断言）。
    这里只做两件事：换一次随机（重试），或如实判「生成非法」。
    """
    reason = ""
    for attempt in range(MAX_ATTEMPTS):
        decl = generate(seed, index, attempt=attempt)
        tokens, why = build_reference(decl, cfg)
        if not tokens:
            reason = why
            continue
        issues = validate_reference(decl, tokens, cfg)
        errors = [i for i in issues if i.level == "error"]
        if errors:
            reason = f"参考解校验 {len(errors)} error：{errors[0].code} {errors[0]}"
            continue
        return decl, tokens, ""
    return None, [], reason or f"{MAX_ATTEMPTS} 次尝试均失败"


# --------------------------------------------------------------------------- #
#  落盘（**字节确定**：无时间戳、无绝对路径、固定键序）
# --------------------------------------------------------------------------- #
YAML_HEADER = """\
# 合成地图（WB-4 map_synth）—— 本文件由工具生成，**不要手改**。
# 生成器: dzrobot_v2/tools/map_synth.py
# 种子/序号: seed={seed} index={index}
# 合法性: 全格可通行 + 雷/靶/障落合法格 + 存在参考解（同名 .ref.txt）
#
# 消费者: planner.load_map / sim.parse_map_yaml / tools/route_check.py(--map via ROBOT_MAP)
"""


def render_yaml(decl: MapDecl) -> str:
    """规范 YAML 文本。**两种解析器都要吃**：`yaml.safe_load` 与 `sim.parse_map_yaml`。

    后者是自己手写的行解析器（只认 `map:` 段 2 空格缩进 + 行内 `[..]` 列表），
    所以这里固定用**两空格缩进 + 行内列表**，不用块状列表、不写多行字符串。
    """
    def inline(items) -> str:
        return "[" + ", ".join(f'"{x}"' for x in items) + "]"

    body = [
        f"map:",
        f'  name: "{decl.name}"',
        f'  start: "{START_BLOCK}"',
        f'  end: "{END_BLOCK}"',
        f'  barrier_col_from: "{BARRIER_COL_FROM}"',
        f"  barrier_row_hi: {BARRIER_ROW_HI}",
        f"  barrier_row_lo: {BARRIER_ROW_LO}",
        f"  # 雷({len(decl.mines)})：必须全部压过（R005）",
        f"  mines:      {inline(decl.mines)}",
        f"  # 障({len(decl.obstacles)})：圆柱，不可通行且遮挡视线（R004 + 净空判据）",
        f"  obstacles:  {inline(decl.obstacles)}",
        f"  # 靶(2)：跨格靶板（正交相邻两格，车体不可压过）",
        f"  targets:    {inline(decl.targets)}",
        f"  # 通(2)：下半场 {decl.lower_bridge} + 上半场 {decl.upper_bridge}",
        f"  bridges:    {inline(decl.bridges)}",
        "",
    ]
    head = YAML_HEADER.format(seed=decl.seed, index=decl.index)
    return head + "\n".join(body)


def render_tokens(tokens: list[str], *, width: int = 78) -> str:
    """参考解路线文件：**一行**（`_read_token_file` 认空白分隔，一行最省事且字节稳定）。"""
    return " ".join(tokens) + "\n"


# --------------------------------------------------------------------------- #
#  自检（逐张实跑 tools/route_check.py —— 复用生产校验器，不另造口径）
# --------------------------------------------------------------------------- #
def run_route_check(map_path: pathlib.Path, tokens: list[str],
                    cfg_path: pathlib.Path, *,
                    trajectory: bool = False,
                    max_nodes: int | None = None,
                    sample_step: str | None = None) -> tuple[int, str]:
    """跑一次生产校验器。`trajectory=True` 走**完整口径**（连续轨迹编译 + 审计）。

    ★ 卡 WB-19：`--trajectory` **强制要求显式 `--map`**（实测：只给 `ROBOT_MAP`
      环境变量会得到 `--trajectory requires an explicit --map` + rc=2）。
      所以 trajectory 模式下必须同时传 `--map`，**不能**只靠环境变量 —— 这是
      第一次探针 30/30 全 rc=2 的直接原因，写在这里免得下次再踩。
    """
    import os
    import subprocess
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"
    if trajectory:
        # 显式 --map 优先；同时清掉环境变量，避免两处口径打架
        env.pop("ROBOT_MAP", None)
        cmd = [sys.executable, str(_TOOLS / "route_check.py"),
               "--trajectory", "--map", str(map_path),
               "--route", " ".join(tokens), "--config", str(cfg_path)]
        if max_nodes is not None:
            cmd += ["--max-nodes", str(max_nodes)]
        if sample_step is not None:
            cmd += ["--sample-step", sample_step]
    else:
        env["ROBOT_MAP"] = str(map_path)
        cmd = [sys.executable, str(_TOOLS / "route_check.py"),
               "--route", " ".join(tokens), "--config", str(cfg_path)]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300,
                              cwd=str(_REPO), env=env, encoding="utf-8", errors="replace")
    except subprocess.TimeoutExpired:
        return 1, "route_check.py 超时（300s）"
    return proc.returncode, (proc.stdout or "") + (proc.stderr or "")


#: 校验器对"未证明"的**自述**标记（卡 WB-19 的机器判据，不许靠人读措辞）。
UNPROVEN_MARKER = "not a proof of map infeasibility"


def classify_trajectory(map_path: pathlib.Path, tokens: list[str],
                        cfg_path: pathlib.Path) -> dict:
    """按**完整口径**（`--trajectory`）分三档：`PASS` / `UNPROVEN` / `FAIL`。

    ★ 三档判据（全部机器可判，不靠人读措辞）：
      · `PASS`     —— rc == 0（穷实通过）
      · `UNPROVEN` —— rc != 0 **且** 输出含 `not a proof of map infeasibility`
                      **且** 提高 `--max-nodes`(11×) 后**仍不通过**（证明不是预算问题）
      · `FAIL`     —— rc != 0 **且** 免责声明缺失，**且** 细化 `--sample-step`(4×) 后仍失败
                      （证明不是采样间距问题）

    ★ 为什么必须"换参数仍不变"才算：只凭报错文本判档，等于把"机器当时多忙/参数多紧"
      当成图的固有性质 —— 这与 WB-4 已修过的"墙钟超时 ≠ 无解"是同一类错误。
    """
    rc, txt = run_route_check(map_path, tokens, cfg_path, trajectory=True)
    rec = {"rc": rc, "unproven_marker": UNPROVEN_MARKER in txt,
           "reason": next((l.strip() for l in txt.splitlines()
                           if l.strip().startswith("ERROR")), "")[:160]}
    if rc == 0:
        rec["state"] = "PASS"
        return rec
    if rec["unproven_marker"]:
        rc2, _ = run_route_check(map_path, tokens, cfg_path, trajectory=True,
                                 max_nodes=18000 * 11)
        rec["retry_max_nodes_rc"] = rc2
        # 提预算就过 ⇒ 这条本不该叫"未证明"，判据失效必须显式暴露（不是静默降级）
        rec["state"] = "UNPROVEN" if rc2 != 0 else "UNPROVEN_BUDGET_SENSITIVE"
        return rec
    rc3, _ = run_route_check(map_path, tokens, cfg_path, trajectory=True,
                             sample_step="0.0005")
    rec["retry_sample_step_rc"] = rc3
    rec["state"] = "FAIL" if rc3 != 0 else "FAIL_SAMPLE_SENSITIVE"
    return rec


def declared_barrier_not_honored(map_path: pathlib.Path, cfg_path: pathlib.Path) -> str | None:
    """★ 复现「地图声明隔板行被校验器忽略」这条实测不一致（诊断，不参与生成）。

    做法：把地图里的 `barrier_row_hi` 改成 `barrier_row_hi-1`（墙往上挪一行），
    再问 `validate._barrier_crossing()` 是否按**声明**判。参考答案：两行都读
    `geo.BARRIER_ROW_HI` 常量 ⇒ 改声明**不改变**判定 ⇒ 返回一行证据。
    """
    import yaml
    doc = yaml.safe_load(map_path.read_text(encoding="utf-8")) or {}
    section = dict(doc.get("map") or {})
    claimed = int(section.get("barrier_row_hi", BARRIER_ROW_HI))
    probe_up = block_of((BARRIER_ROW_HI - 1, 0))       # 行 hi-1 的 A 列
    probe_lo = block_of((BARRIER_ROW_HI, 0))           # 行 hi 的 A 列
    hit = V._barrier_crossing(probe_up, probe_lo)      # noqa: SLF001  只读调用，不改它
    declared_wall_between = (BARRIER_ROW_HI, BARRIER_ROW_HI + 1)
    if hit is None:
        return None
    blocked, _cd, rd = hit
    return (f"BARRIER_DECLARED_NOT_HONORED 地图声明 barrier_row_hi={claimed} "
            f"(墙在 {declared_wall_between[0]}/{declared_wall_between[1]} 行之间)，"
            f"但 _barrier_crossing({probe_up}->{probe_lo}) 仍按常量 "
            f"{geo.BARRIER_ROW_HI}/{geo.BARRIER_ROW_LO} 判（交点行 {rd:.2f}，"
            f"blocked={blocked}）⇒ 声明被忽略")


# --------------------------------------------------------------------------- #
#  主流程
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="WB-4 参数化地图合成器（给定 seed 逐位可复现 + 参考解 + 生产校验）")
    ap.add_argument("--seed", type=int, default=20261031)
    ap.add_argument("--count", type=int, default=30, help="生成张数")
    ap.add_argument("--out", default="../artifacts/synth_maps/", help="输出目录（不存在则新建）")
    ap.add_argument("--selfcheck", action="store_true",
                    help="逐张实跑 tools/route_check.py（复用生产校验器）")
    ap.add_argument("--trajectory", action="store_true",
                    help="★ WB-19：自检走**完整口径**（route_check.py --trajectory，"
                         "连续轨迹编译+审计）并分三档 PASS/UNPROVEN/FAIL。"
                         "不带本开关 = legacy 口径，只能报『生成合法』，**不能**声称图可跑")
    ap.add_argument("--config", default=None, help="配置中心（缺省 config/robot.yaml）")
    ap.add_argument("--vary-barrier", action="store_true",
                    help="诊断开关：把隔板行也随机化（会造出「声明 vs 校验器」分叉）")
    ap.add_argument("--json", action="store_true", help="末行输出 MAPSYNTH_RESULT_JSON")
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg_path = pathlib.Path(args.config) if args.config else (_REPO / "config" / "robot.yaml")
    cfg = Config.load(str(cfg_path)) if cfg_path.is_file() else None
    out_dir = pathlib.Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    entries: list[dict] = []
    invalid: list[dict] = []
    for index in range(args.count):
        decl, tokens, why = make_one(args.seed, index, cfg,
                                     barrier_variation=args.vary_barrier)
        if decl is None:
            invalid.append({"index": index, "reason": why})
            print(f"  ❌ 生成非法 #{index}: {why}")
            continue
        map_path = out_dir / f"{decl.name}.yaml"
        ref_path = out_dir / f"{decl.name}.ref.txt"
        map_path.write_text(render_yaml(decl), encoding="utf-8")
        ref_path.write_text(render_tokens(tokens), encoding="utf-8")
        entry = {"index": index, "name": decl.name,
                 "map": map_path.name, "ref": ref_path.name,
                 "mines": len(decl.mines), "obstacles": len(decl.obstacles),
                 "targets": len(decl.targets), "bridges": decl.bridges,
                 "ref_tokens": len(tokens)}
        entries.append(entry)
        print(f"  ✅ {decl.name}: 雷{len(decl.mines)} 障{len(decl.obstacles)} "
              f"靶{len(decl.targets)} 通{'/'.join(decl.bridges)} "
              f"参考解 {len(tokens)} token")

    checks: list[dict] = []
    barrier_note: str | None = None
    rc = RC_OK
    if args.selfcheck:
        mode = "trajectory（完整口径，分三档）" if args.trajectory else "legacy（只判生成合法）"
        print("\n=== 自检：逐张过 tools/route_check.py（生产校验器）· %s ===" % mode)
        for entry in entries:
            map_path = out_dir / entry["map"]
            tokens = (out_dir / entry["ref"]).read_text(encoding="utf-8").split()
            if args.trajectory:
                # ★ 完整口径：三档分类（PASS / UNPROVEN / FAIL）
                cls = classify_trajectory(map_path, tokens, cfg_path)
                state = cls["state"]
                rec = {"name": entry["name"], "state": state, "rc": cls["rc"],
                       "trajectory_reason": cls["reason"],
                       "unproven_marker": cls["unproven_marker"]}
                if "retry_max_nodes_rc" in cls:
                    rec["retry_max_nodes_rc"] = cls["retry_max_nodes_rc"]
                if "retry_sample_step_rc" in cls:
                    rec["retry_sample_step_rc"] = cls["retry_sample_step_rc"]
                checks.append(rec)
                mark = {"PASS": "✅", "UNPROVEN": "◻", "FAIL": "❌"}.get(state, "⚠")
                print(f"  {mark} {entry['name']}: {state}  rc={cls['rc']}"
                      + (f"  {cls['reason'][:90]}" if cls["reason"] else ""))
                # ★ 只有 PASS 才不改 rc；UNPROVEN **不算通过**但也不算"图非法"，
                #   所以单独计数、不把 rc 打成 INVALID（否则会掩盖"未证明"与"已证伪"的区别）
                if state.startswith("UNPROVEN"):
                    rc = RC_INVALID if rc == RC_OK else rc
                elif state != "PASS":
                    rc = RC_INVALID
            else:
                code, text = run_route_check(map_path, tokens, cfg_path)
                errs = [ln.strip() for ln in text.splitlines() if "❌" in ln]
                checks.append({"name": entry["name"], "rc": code,
                               "state": "LEGACY_UNCHECKED",
                               "errors": len(errs),
                               "first_error": (errs[0] if errs else "")})
                mark = "✅" if code == RC_OK else "❌"
                print(f"  {mark} {entry['name']}: rc={code} error行={len(errs)}"
                      f"  （legacy：**未**验证轨迹可编译）"
                      + (f"  首条: {errs[0][:90]}" if errs else ""))
                if code != RC_OK:
                    rc = RC_INVALID
        barrier_note = declared_barrier_not_honored(out_dir / entries[0]["map"], cfg_path) \
            if entries else None
        if barrier_note:
            print("  ⚠️ " + barrier_note)

    # 清单（字节确定：键序固定、无时间戳、**不含**自检结果）
    #
    # ★ 为什么自检结果单独一个文件（实测踩到）：卡面 verify 第 2 条要求
    #   「同 seed 两次跑，产物目录逐位相同」，而第一次目录是用 `--selfcheck` 造的、
    #   第二次不一定带 —— 把自检结论并进 MANIFEST 会让**同一份地图**因为"跑没跑自检"
    #   产生不同的清单字节，确定性判据当场假红（实测 61 个文件里恰有 MANIFEST.json 不符）。
    #   ⇒ MANIFEST 只记"生成了什么"（纯输入的确定性产物），自检结论落 SELFCHECK.json。
    manifest = {"schema": "dzrobot-map-synth/1", "seed": int(args.seed),
                "count": int(args.count), "generated": len(entries),
                "invalid": invalid, "maps": entries}
    (out_dir / "MANIFEST.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    if args.selfcheck:
        buckets: dict[str, list[str]] = {}
        for c in checks:
            buckets.setdefault(c.get("state") or ("PASS" if c.get("rc") == 0
                                                  else "LEGACY_UNCHECKED"), []).append(c["name"])
        report = {"schema": "dzrobot-map-synth-selfcheck/1", "seed": int(args.seed),
                  "count": int(args.count),
                  "mode": ("trajectory" if args.trajectory else "legacy"),
                  "checks": checks,
                  # ★ 卡 WB-19：按档分批计数。`unproven` 与 `deterministic_fail` **必须并列**——
                  #   若把 FAIL 也塞进 unproven，`invalid` 会永远为空，
                  #   「每张图都过校验器」这条 acceptance 将永远看不到红。
                  "pass": buckets.get("PASS", []),
                  "unproven": buckets.get("UNPROVEN", []),
                  "deterministic_fail": buckets.get("FAIL", []),
                  "legacy_unchecked": buckets.get("LEGACY_UNCHECKED", []),
                  "counts": {k: len(v) for k, v in buckets.items()},
                  "barrier_note": barrier_note}
        if args.trajectory:
            report["note"] = ("可行率只对 PASS 图成立；UNPROVEN 与 FAIL **均不进**"
                              "可行率分子/分母，二者单列。legacy 模式只能报『生成合法』，"
                              "不能声称图可跑。")
        (out_dir / "SELFCHECK.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")

    ok_n = len(entries)
    print(f"\n共 {ok_n}/{args.count} 张生成合法"
          + (f"，{len(invalid)} 张生成非法" if invalid else "") + "")
    if checks:
        if args.trajectory:
            n_pass = sum(1 for c in checks if c.get("state") == "PASS")
            n_unp = sum(1 for c in checks if str(c.get("state", "")).startswith("UNPROVEN"))
            n_fail = sum(1 for c in checks
                         if str(c.get("state", "")).startswith("FAIL"))
            print(f"轨迹三档：PASS {n_pass} / UNPROVEN {n_unp} / FAIL {n_fail}"
                  f"（共 {len(checks)} 张，**完整口径** route_check.py --trajectory）")
            print("  ⚠️ 可行率只对 PASS 图成立；UNPROVEN（未证明）与 FAIL（已证伪）"
                  "均不进可行率分子/分母")
        else:
            print(f"legacy 自检 {sum(1 for c in checks if c['rc'] == 0)}/{len(checks)} 张 rc=0"
                  "  ⚠️ legacy 口径**未**验证轨迹可编译 ⇒ 这**不是**「图合法」的证据")
    if args.json:
        line = dict(schema="dzrobot-map-synth/1", seed=int(args.seed), count=int(args.count),
                    generated=ok_n, invalid=len(invalid), out=str(out_dir))
        print("MAPSYNTH_RESULT_JSON " + json.dumps(line, ensure_ascii=False, sort_keys=True))
    return rc


if __name__ == "__main__":
    sys.exit(main())
