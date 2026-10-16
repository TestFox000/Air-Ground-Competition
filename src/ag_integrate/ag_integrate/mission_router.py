"""任务路由器：世界态势 -> 绕障的有序格序列。

输入 world_model.WorldSnapshot，输出有序格号（tuple[str]），交给 car_adapter。

运动模型（对齐官方 path_executor 路线语法"直线连接、可对角"）：
    - 8 邻域移动：正交 1.0 格、对角 √2 格；对角移动要求两侧正交格均非障碍（防切角）；
    - 转向代价：阿克曼底盘（轴距 150mm、最小转弯半径≈0.34m），每 45° 转向
      ≈ 0.27m 弧长 ≈ 0.76 格，默认 turn_45=0.75（设 0 则退化为纯最短路）；
    - 朝向：start_heading / final_heading 支持 'x'（东）/ 'y'（北），
      对应车端 reset 断言朝向与路线末尾 ",x/,y"；中途朝向自动链式传递。

做法（近似全局最优）：
    - 障碍格为 blocked；必经点 = 雷 + 可通行桥洞格 + 蓝色敌靶格（开过去才能打击）；
    - 各必经点间用朝向感知 Dijkstra（状态 = 格 + 航向）求最省代价路径作边权，
      Held-Karp 位压 DP 求访问顺序（边权对"进入朝向"取乐观界，≤12 点毫秒级）；
    - 枚举所有达到乐观最优的顺序，逐段按真实朝向链拼接、顺路避让已踩格，
      取（真实代价, 重复踩格）最小者；
    - 不可达必经点剔除（跳过）；终点不可达报错。

中场挡板：默认不建模；现场证实"只能从可通行格穿越半场"后传入 cross_gates
（允许穿越的格）。桥洞一格宽，穿越中线的移动限定正交且贴着 gate 格。
"""

from __future__ import annotations

import heapq
import warnings
from itertools import count, product
from math import gcd

from .field_frame import CellRef, ROWS, COLS, LETTERS
from .world_model import WorldSnapshot

_SQRT2 = 2.0 ** 0.5
_INF = float("inf")
_EPS = 1e-6

# 航向索引（顺时针，0=正东）：车端 'x'=东(0)、'y'=北(6)
_DIRS = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))
_HEADING_IDX = {"x": 0, "y": 6}
_ALL_HEADINGS = tuple(range(8))

# 中场横向挡板：位于第 6/7 行的边界线上（row5↔row6 之间），
# 向上占第6行底部1/4、向下占第7行顶部1/4，总高半格。
# 第6行、第7行格子本身可通行，但跨边界的竖向移动被阻挡，桥洞缺口除外。
# 2026-10-15 终裁：挡板实体在 D7~N7 格带（第 6/7 行边界）——车上官方地图真源
# + 现场核实（I6→I7/K6→K7/L6→L7 全封死）双重确认，mid_boundary=6。
# 注意：旧"决定性证据"（goal=N6+heading=y 只在 =5 可行）建立在旧门模型上，
# 与现场物理冲突的归一问题见 docs/分区规划与感知约束-20261015.md 问题1。
_MID_BOUNDARY = 6  # 唯一默认值来源：row5(第6行) ↔ row6(第7行) 之间

# 横向挡板覆盖 D(3)~N(13) 列，A、B、C 列无挡板可自由穿越。
# 桥洞列由 passable（ARROW）动态确定，每局随机。
_BARRIER_COLS: frozenset[int] = frozenset(range(3, COLS))  # D..N

# ---- 静态几何表（模块级一次性构建）：整数编码 (格, 航向) 状态空间 ----
# 格索引 ci = row*COLS + col；状态索引 si = ci*8 + h。
# Dijkstra 内循环只碰整数与预构边表，不再逐邻居构造 CellRef/元组哈希
# （2026-10-14 沙箱审计：旧实现单次 ~60ms，其中大半是 CellRef.__init__/__hash__）。
_N_CELLS = ROWS * COLS
_CELLS: tuple[CellRef, ...] = tuple(
    CellRef(c, r) for r in range(ROWS) for c in range(COLS)
)
# 8×8 航向差（几个 45°）查表
_TURNS: tuple[tuple[int, ...], ...] = tuple(
    tuple(min(abs(a - b), 8 - abs(a - b)) for b in range(8)) for a in range(8)
)


def _build_edges() -> tuple[tuple[tuple[int, int, float, bool, int, int, int, int], ...], ...]:
    """_EDGES[ci] = ((n_ci, nh, step, diag, o1_ci, o2_ci, dr, n_row), ...)

    o1/o2 为对角移动防切角的两侧正交格（非对角时为 -1）；dr/n_row 供中场
    挡板跨界判定（mid_boundary 是运行期参数，不能烧进表里）。
    """
    edges = []
    for r in range(ROWS):
        for c in range(COLS):
            lst = []
            for nh, (dc, dr) in enumerate(_DIRS):
                nc, nr = c + dc, r + dr
                if not (0 <= nc < COLS and 0 <= nr < ROWS):
                    continue
                diag = dc != 0 and dr != 0
                o1 = r * COLS + nc if diag else -1   # (nc, r)
                o2 = nr * COLS + c if diag else -1   # (c, nr)
                lst.append((nr * COLS + nc, nh, _SQRT2 if diag else 1.0,
                            diag, o1, o2, dr, nr))
            edges.append(tuple(lst))
    return tuple(edges)


_EDGES = _build_edges()


def _turn_units(h1: int, h2: int) -> int:
    """两个航向之间相差几个 45°。"""
    d = abs(h1 - h2)
    return min(d, 8 - d)


def _zone_of(cell: CellRef, mid_boundary: int) -> str:
    """格号所属功能区（2026-10-15 现场确认的三区划分）。

    西区 = A~C 通长走廊（自由穿行）；东南区 = D~N 第 1~6 行；东北区 = D~N 第 7~11 行。
    """
    if cell.col <= 2:
        return "西"
    return "东南" if cell.row < mid_boundary else "东北"


def _crosses_mid(row_a: int, row_b: int, boundary: int) -> bool:
    """一条竖向边是否跨越中场分界（boundary 指上半区行数，如 6 = row5↔row6 之间）。"""
    return (row_a < boundary <= row_b) or (row_b < boundary <= row_a)


def _dijkstra(
    start: CellRef,
    blocked: frozenset[CellRef],
    cross_gates: frozenset[CellRef] | None,
    mid_boundary: int,
    entry_headings: tuple[int, ...],
    turn_45: float,
    allow_diagonal: bool,
    terminal: CellRef | None = None,
    cross_penalty: float = 0.0,
    tunnel_axes: frozenset[int] | None = None,
    tunnel_req: dict[int, int] | None = None,
) -> tuple[dict, dict]:
    """(格, 航向) 状态空间的 Dijkstra；返回 (dist, prev)。

    cross_penalty：每次跨越挡板边界附加的代价当量（默认 0=关闭，向后兼容），
    分区策略软约束——"一区扫完再转场"天然优于反复跨区。
    tunnel_axes：桥洞隧道格集合——进出必须同行正交（沿隧道轴东西向）、
    洞内禁止转向；一格宽桥洞斜进斜出 = 撞桥边（阿克曼底盘物理约束）。
    tunnel_req：隧道单向约束 {格索引: 行驶朝向}（0=向东 4=向西），
    通行方向=桥顶箭头指向，逆向入洞禁边。依据 docs/分区规划与感知约束-20261015.md。

    entry_headings 里的朝向以 0 代价起步（自由进入 = 不罚第一步转向）。
    terminal 指定时，该格成为终点态：进入即停、不许穿过再离开
    （避免“路过目标又折返”的路径）。

    实现：整数态 (si = ci*8+h) + 预构边表 + 位图障碍，出口处物化为
    原 {(CellRef, h): ...} 字典接口（消费方 _arrival_state/_trace_states 不变）。
    """
    blocked_bits = bytearray(_N_CELLS)
    for c in blocked:
        blocked_bits[c.row * COLS + c.col] = 1
    gates_bits: bytearray | None = None
    if cross_gates is not None:
        gates_bits = bytearray(_N_CELLS)
        for c in cross_gates:
            gates_bits[c.row * COLS + c.col] = 1
    term_ci = terminal.row * COLS + terminal.col if terminal is not None else -1
    start_ci = start.row * COLS + start.col

    dist_i = [_INF] * (_N_CELLS * 8)
    prev_i = [-1] * (_N_CELLS * 8)
    tick = count()
    pq: list[tuple[float, int, int]] = []
    for h in entry_headings:
        si = start_ci * 8 + h
        dist_i[si] = 0.0
        heapq.heappush(pq, (0.0, next(tick), si))
    push, pop = heapq.heappush, heapq.heappop
    while pq:
        d, _, si = pop(pq)
        if dist_i[si] < d - 1e-12:
            continue
        ci = si >> 3
        if ci == term_ci:
            continue  # 终点态：不再向外扩展
        h = si & 7
        row = ci // COLS
        for nci, nh, step, diag, o1, o2, dr, nrow in _EDGES[ci]:
            if blocked_bits[nci]:
                continue
            if diag:
                if not allow_diagonal:
                    continue
                # 防切角：对角移动要求两侧正交格均非障碍
                if blocked_bits[o1] or blocked_bits[o2]:
                    continue
            if (
                dr and gates_bits is not None
                and ((row < mid_boundary <= nrow) or (nrow < mid_boundary <= row))
            ):
                # 桥洞一格宽：只能正交穿越，且必须贴着 gate 格
                if diag or not (gates_bits[ci] or gates_bits[nci]):
                    continue
            # 桥洞隧道轴向约束：斜进斜出/洞内转向=撞桥边（物理必修）
            if tunnel_axes is not None:
                if nci in tunnel_axes and dr != 0:
                    continue            # 进出隧道格只允许同行正交（东西向）
                if ci in tunnel_axes:
                    rev = (h + 4) % 8
                    if dr != 0 or (nh != h and nh != rev):
                        continue        # 出洞只许直行穿洞或原路倒出（倒车=4×45°转向代价）
            # 隧道单向：逆向入洞禁边（规则方向=桥顶箭头指向）
            if tunnel_req is not None:
                req_h = tunnel_req.get(nci, -1)
                if req_h >= 0 and nh != req_h:
                    continue
            # 穿越挡板边界附加代价：分区策略软约束（docs/分区规划与感知约束）
            extra = 0.0
            if cross_penalty and dr and (
                    (row < mid_boundary <= nrow) or (nrow < mid_boundary <= row)):
                extra = cross_penalty
            nd = d + step + _TURNS[h][nh] * turn_45 + extra
            nsi = nci * 8 + nh
            if nd < dist_i[nsi] - 1e-12:
                dist_i[nsi] = nd
                prev_i[nsi] = si
                push(pq, (nd, next(tick), nsi))

    dist: dict[tuple[CellRef, int], float] = {}
    prev: dict[tuple[CellRef, int], tuple[CellRef, int]] = {}
    for si in range(_N_CELLS * 8):
        dd = dist_i[si]
        if dd < _INF:
            st = (_CELLS[si >> 3], si & 7)
            dist[st] = dd
            p = prev_i[si]
            if p >= 0:
                prev[st] = (_CELLS[p >> 3], p & 7)
    return dist, prev


def _arrival_state(
    dist: dict, cell: CellRef, want: int | None
) -> tuple[float, tuple[CellRef, int] | None]:
    """到达 cell 的最省（代价, 终态）；want 指定到达航向。"""
    if want is not None:
        return dist.get((cell, want), _INF), (cell, want)
    best = (_INF, None)
    for h in _ALL_HEADINGS:
        d = dist.get((cell, h), _INF)
        if d < best[0]:
            best = (d, (cell, h))
    return best


def _trace_states(prev: dict, end_state: tuple[CellRef, int]) -> list[CellRef]:
    cells: list[CellRef] = []
    state: tuple[CellRef, int] | None = end_state
    while state is not None:
        cells.append(state[0])
        state = prev.get(state)
    cells.reverse()
    return cells


def _cached_dijkstra(
    cache: dict | None,
    start: CellRef,
    blocked: frozenset[CellRef],
    cross_gates: frozenset[CellRef] | None,
    mid_boundary: int,
    entry_headings: tuple[int, ...],
    turn_45: float,
    allow_diagonal: bool,
    terminal: CellRef | None = None,
    cross_penalty: float = 0.0,
    tunnel_axes: frozenset[int] | None = None,
    tunnel_req: dict[int, int] | None = None,
) -> tuple[dict, dict]:
    """_dijkstra 的缓存封装：键 = (起点, 进入朝向集, 终点态, 障碍集)。

    cross_gates/mid_boundary/turn_45/allow_diagonal 在一次 plan_route 调用内
    不变，由调用方保证；避让重算的 blocked 会变，故必须进键。
    cross_penalty/tunnel_axes/tunnel_req 同属本约定（一次调用内不变），不进键。
    """
    key = (start, entry_headings, terminal, blocked)
    if cache is not None and key in cache:
        return cache[key]
    result = _dijkstra(start, blocked, cross_gates, mid_boundary, entry_headings, turn_45, allow_diagonal, terminal=terminal,
                       cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
    if cache is not None:
        cache[key] = result
    return result


def _route_leg(
    frm: CellRef,
    to: CellRef,
    blocked: frozenset[CellRef],
    used: set[CellRef],
    cross_gates: frozenset[CellRef] | None,
    mid_boundary: int,
    entry: tuple[int, ...],
    want_arrival: int | None,
    turn_45: float,
    allow_diagonal: bool,
    cache: dict | None = None,
    cross_penalty: float = 0.0,
    tunnel_axes: frozenset[int] | None = None,
    tunnel_req: dict[int, int] | None = None,
) -> tuple[list[CellRef] | None, int | None, float]:
    """frm→to 单段路由：优先走避开已踩格子的等代价路径。返回 (cells, 到达航向, 代价)。

    性能（2026-10-14 沙箱审计修复）：等最优顺序枚举最多 256 条，每条 9+ 段，
    逐段两次 Dijkstra 曾导致单局规划 30s+（ARM 车端会放大到分钟级，直接
    吃掉 180s 计时）。三处提速：
      1. Dijkstra 结果按 (frm, entry, terminal, blocked) 缓存——跨顺序/跨组合
         大量重复；中间段一律用全场图（terminal=None），使同一起点+进入朝向的
         所有段共享同一次计算；
      2. want_arrival=None 时不指定 terminal 是安全的：最省到达态的回溯链
         不可能中途先穿过 to（穿过意味着存在更便宜的到达，与"最省"矛盾），
         只有末段 h_final 约束才需要终点态防折返；
      3. 自由路径若本就零重复踩格，跳过避让重算（多数段的实际情况）。
    """
    if frm == to:
        h0 = entry[0] if len(entry) == 1 else (want_arrival if want_arrival is not None else 0)
        return [frm], h0, 0.0

    term = to if want_arrival is not None else None
    d_free, p_free = _cached_dijkstra(cache, frm, blocked, cross_gates, mid_boundary, entry, turn_45, allow_diagonal, terminal=term,
                                      cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
    cost, end = _arrival_state(d_free, to, want_arrival)
    if end is None or cost >= _INF:
        return None, None, _INF

    avoid_cells = used - {frm, to}
    free_cells = _trace_states(p_free, end)
    if avoid_cells and not (set(free_cells) & avoid_cells):
        # 自由路径没有踩任何已踩格：避让重算的结果必然相同，直接返回
        return free_cells, end[1], cost

    avoid = frozenset(blocked | avoid_cells)
    if avoid != blocked:
        d_av, p_av = _cached_dijkstra(cache, frm, avoid, cross_gates, mid_boundary, entry, turn_45, allow_diagonal, terminal=term,
                                  cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
        c_av, e_av = _arrival_state(d_av, to, want_arrival)
        if e_av is not None and c_av <= cost + _EPS:
            return _trace_states(p_av, e_av), e_av[1], c_av
    return free_cells, end[1], cost


def _flatten(segments: list[list[CellRef]]) -> tuple[str, ...]:
    names: list[str] = []
    for seg in segments:
        for cell in seg:
            name = cell.name
            if not names or names[-1] != name:
                names.append(name)
    return tuple(names)


def _reconstruct_order(parent: list[list[int]], mask: int, last: int) -> list[int]:
    """按 DP 父指针回溯出一条最优访问顺序（保底用）。"""
    order: list[int] = []
    while last != -1:
        order.append(last)
        prev = parent[mask][last]
        mask ^= 1 << last
        last = prev
    order.reverse()
    return order


def _enumerate_optimal_orders(
    dp: list[list[float]],
    W: list[list[float]],
    dgoal: list[float],
    n: int,
    best_cost: float,
    call_cap: int = 20000,
    order_cap: int = 256,
) -> list[list[int]]:
    """枚举所有达到（乐观）最优代价的访问顺序（带上限，防对称场景组合爆炸）。"""
    orders: list[list[int]] = []
    budget = [call_cap]

    def rec(mask: int, last: int, suffix: tuple[int, ...]) -> None:
        if budget[0] <= 0 or len(orders) >= order_cap:
            return
        budget[0] -= 1
        if mask == (1 << last):  # 回到初始态：该顺序从起点直接过来
            orders.append([last, *suffix])
            return
        pmask = mask ^ (1 << last)
        row = dp[pmask]
        target = dp[mask][last]
        for i in range(n):
            if not (pmask >> i) & 1:
                continue
            if row[i] < _INF and abs(row[i] + W[i + 1][last + 1] - target) <= _EPS:
                rec(pmask, i, (last, *suffix))
                if len(orders) >= order_cap:
                    return

    full_mask = (1 << n) - 1
    for last in range(n):
        if dp[full_mask][last] < _INF and abs(dp[full_mask][last] + dgoal[last + 1] - best_cost) <= _EPS:
            rec(full_mask, last, ())
    return orders


def _assemble(
    order: list[int],
    nodes: list[CellRef],
    goal: CellRef,
    blocked: frozenset[CellRef],
    cross_gates: frozenset[CellRef] | None,
    mid_boundary: int,
    entry0: tuple[int, ...],
    h_final: int | None,
    turn_45: float,
    allow_diagonal: bool,
    cache: dict | None = None,
    fallback_any_heading: bool = False,
    cross_penalty: float = 0.0,
    tunnel_axes: frozenset[int] | None = None,
    tunnel_req: dict[int, int] | None = None,
) -> tuple[tuple[str, ...] | None, float]:
    """按给定顺序逐段拼接（真实朝向链 + 顺路避让），返回 (路线, 真实总代价)。

    fallback_any_heading：终点朝向约束（h_final）物理不可达时
    （例：终点格是口袋入口，朝向到达须先踩终点），允许以任意朝向进终点——
    放弃终点朝向分，保住完赛。严格模式（False）下不可达即整段返回 None。
    """
    used: set[CellRef] = {nodes[0]}
    segments: list[list[CellRef]] = []
    total = 0.0
    cur, entry = nodes[0], entry0
    for i in order:
        w = nodes[i + 1]
        seg, h, c = _route_leg(cur, w, blocked, used, cross_gates, mid_boundary, entry, None, turn_45, allow_diagonal, cache,
                               cross_penalty, tunnel_axes, tunnel_req)
        if seg is None:
            return None, _INF
        segments.append(seg)
        used.update(seg)
        total += c
        cur, entry = w, (h,)
    seg, _h, c = _route_leg(cur, goal, blocked, used, cross_gates, mid_boundary, entry, h_final, turn_45, allow_diagonal, cache,
                               cross_penalty, tunnel_axes, tunnel_req)
    if seg is None and fallback_any_heading and h_final is not None:
        seg, _h, c = _route_leg(cur, goal, blocked, used, cross_gates, mid_boundary, entry, None, turn_45, allow_diagonal, cache,
                               cross_penalty, tunnel_axes, tunnel_req)
    if seg is None:
        return None, _INF
    segments.append(seg)
    total += c
    return _flatten(segments), total


def _plan_core(
    start: CellRef,
    goal: CellRef,
    blocked: frozenset[CellRef],
    required: dict[str, CellRef],
    cross_gates: frozenset[CellRef] | None,
    mid_boundary: int,
    entry0: tuple[int, ...],
    h_final: int | None,
    turn_45: float,
    allow_diagonal: bool,
    cache: dict | None = None,
    cross_penalty: float = 0.0,
    tunnel_axes: frozenset[int] | None = None,
    tunnel_req: dict[int, int] | None = None,
) -> tuple[tuple[str, ...], float]:
    """给定障碍集与必经点，输出（最优路线, 真实总代价）。"""
    dist0, _ = _cached_dijkstra(cache, start, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal,
                     cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
    if _arrival_state(dist0, goal, None)[0] >= _INF:
        raise ValueError("goal cell unreachable from start")
    # 注意：可达性检查必须按"任意朝向"——朝向到达不可行由拼装层降级兜底，
    # 若在此按 h_final 判不可达，会把"丢 5 分朝向"误升级成"整局崩溃"。

    # 剔除不可达必经点（与旧语义一致：跳过）；nodes[0] 恒为起点。
    # 但不再静默：逐格告警并按功能分区标注——整区被跳过大概率是唯一通道
    # （桥洞/走廊口）被障碍误检堵死（交界格误检是 documented 顽疾）。
    reachable_required = []
    dropped = []
    for c in required.values():
        (reachable_required if _arrival_state(dist0, c, None)[0] < _INF else dropped).append(c)
    if dropped:
        by_zone: dict[str, list[str]] = {}
        for c in dropped:
            by_zone.setdefault(_zone_of(c, mid_boundary), []).append(c.name)
        detail = "；".join(f"{z}区[{','.join(sorted(ns))}]" for z, ns in sorted(by_zone.items()))
        warnings.warn(
            f"{len(dropped)} 个必经点不可达被跳过：{detail}。",
            RuntimeWarning, stacklevel=3,
        )
    nodes = [start] + reachable_required
    n = len(nodes) - 1
    if n == 0:
        seg, _h, _c = _route_leg(start, goal, blocked, set(), cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal, cache,
                             cross_penalty, tunnel_axes, tunnel_req)
        return _flatten([seg]), _c

    # 每个节点做一次自由进入朝向的 Dijkstra，得两两边权（乐观界）。
    dists = [dist0] + [
        _cached_dijkstra(cache, c, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal,
                          cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)[0]
        for c in nodes[1:]
    ]
    W = [[_INF] * (n + 1) for _ in range(n + 1)]
    for i in range(n + 1):
        W[i][i] = 0.0
        for j in range(n + 1):
            if j != i:
                W[i][j] = _arrival_state(dists[i], nodes[j], None)[0]
    dgoal = [_arrival_state(dists[i], goal, h_final)[0] for i in range(n + 1)]
    if h_final is not None and all(d >= _INF for d in dgoal):
        # 终点朝向到达全局不可行（口袋地形）：TSP 排序改用任意朝向界，
        # 朝向降级由拼装层兜底——否则 dgoal 全 INF 会导致回溯出空顺序，
        # 退化成"起点直奔终点、跳过全部雷"的灾难路线。
        dgoal = [_arrival_state(dists[i], goal, None)[0] for i in range(n + 1)]

    # Held-Karp：dp[mask][i] = 从起点出发、恰好经过 mask 集合内的必经点、
    # 当前停在必经点 i 的最少总代价（必经点编号 0..n-1 对应 nodes[1..n]）。
    full = 1 << n
    dp = [[_INF] * n for _ in range(full)]
    parent = [[-1] * n for _ in range(full)]
    for i in range(n):
        dp[1 << i][i] = W[0][i + 1]
    for mask in range(1, full):
        row = dp[mask]
        for i in range(n):
            cur = row[i]
            if cur >= _INF or not (mask >> i) & 1:
                continue
            base = W[i + 1]
            for j in range(n):
                if (mask >> j) & 1:
                    continue
                nm = mask | (1 << j)
                nd = cur + base[j + 1]
                if nd < dp[nm][j] - 1e-12:
                    dp[nm][j] = nd
                    parent[nm][j] = i

    best_cost, best_last = _INF, -1
    for i in range(n):
        c = dp[full - 1][i] + dgoal[i + 1]
        if c < best_cost:
            best_cost, best_last = c, i

    def assemble(order: list[int]) -> tuple[tuple[str, ...] | None, float]:
        return _assemble(order, nodes, goal, blocked, cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal, cache,
                         cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)

    # 保底顺序 + 枚举其余等代价顺序，取（真实代价, 重复踩格）最小者。
    # 拼接数硬上限：对称场景下等代价顺序可达 256 条，每条的避让重算是
    # 剩余耗时大头；64 次足以覆盖真实的并列分支（实测场景 <10 次即收敛）。
    best_route: tuple[str, ...] | None = None
    best_cost_true: float = _INF
    best_dup = 1 << 30
    candidates = [_reconstruct_order(parent, full - 1, best_last)]
    candidates += _enumerate_optimal_orders(dp, W, dgoal, n, best_cost)
    for order in candidates[:64]:
        route, cost_true = assemble(order)
        if route is None:
            continue
        dup = len(route) - len(set(route))
        key_better = cost_true < best_cost_true - _EPS or (
            abs(cost_true - best_cost_true) <= _EPS and dup < best_dup
        )
        if best_route is None or key_better:
            best_route, best_cost_true, best_dup = route, cost_true, dup
        # 真实代价达到乐观下界且零重复，不可能更好了
        if best_cost_true <= best_cost + _EPS and best_dup == 0:
            break
    if best_route is None and h_final is not None:
        # 终点朝向到达物理不可行（如终点格是口袋入口，朝向到达须先踩终点）。
        # 降级：任意朝向进终点——放弃终点朝向分，保住完赛（崩溃 = 整局 0 分）。
        warnings.warn(
            f"goal {goal.name} 以朝向到达不可行，降级为任意朝向进终点"
            "（终点朝向 5 分不保）",
            RuntimeWarning, stacklevel=3,
        )
        for order in candidates[:64]:
            route, cost_true = _assemble(
                order, nodes, goal, blocked, cross_gates, mid_boundary,
                entry0, h_final, turn_45, allow_diagonal, cache,
                fallback_any_heading=True,
                cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req,
            )
            if route is None:
                continue
            dup = len(route) - len(set(route))
            key_better = cost_true < best_cost_true - _EPS or (
                abs(cost_true - best_cost_true) <= _EPS and dup < best_dup
            )
            if best_route is None or key_better:
                best_route, best_cost_true, best_dup = route, cost_true, dup
    assert best_route is not None  # 保底顺序必然可拼接（终点朝向不可达时已降级）
    return best_route, best_cost_true


_ORTHO_STEPS = ((1, 0), (-1, 0), (0, 1), (0, -1))
_SHOOT_COMBO_CAP = 64  # 蓝靶射击位组合枚举上限，超过则退化为逐靶就近


def _shoot_candidates(
    target: CellRef,
    blocked: frozenset[CellRef],
    target_cells: dict[str, CellRef],
) -> list[CellRef]:
    """固定靶的候选停靠射击格：正交相邻格（车端正对靶心开火最准），
    排除障碍/桥洞墙/其他靶格。"""
    out = []
    for dc, dr in _ORTHO_STEPS:
        c = CellRef(target.col + dc, target.row + dr)
        if not (0 <= c.col < COLS and 0 <= c.row < ROWS):
            continue
        if c in blocked or c.name in target_cells:
            continue
        out.append(c)
    return sorted(out, key=lambda c: c.name)


def _shoot_candidates_moving(
    t: "TargetRef",
    blocked: frozenset[CellRef],
) -> list[CellRef]:
    """移动靶的候选停靠射击格：轨道两端在蓝面朝向一侧的正交邻格。

    车停这类格上，靶移动到正对端点时恰好正交相邻（距离 1 格），
    车头正对靶心开火最准；红面朝向另一侧，不可能误伤。
    轨道水平（两端同行）时 blue_face='S'(row+1)/'N'(row-1)；
    垂直（两端同列）时 'E'(col+1)/'W'(col-1)。
    """
    a, b = t.track
    if a.row == b.row:  # 水平轨道
        dr = 1 if t.blue_face == "S" else -1
        cells = (CellRef(a.col, a.row + dr), CellRef(b.col, b.row + dr))
    else:               # 垂直轨道
        dc = 1 if t.blue_face == "E" else -1
        cells = (CellRef(a.col + dc, a.row), CellRef(b.col + dc, b.row))
    out = []
    for c in cells:
        if not (0 <= c.col < COLS and 0 <= c.row < ROWS):
            continue
        if c in blocked:
            continue
        out.append(c)
    return sorted(out, key=lambda c: c.name)


def plan_route(
    snapshot: WorldSnapshot,
    start: CellRef,
    goal: CellRef,
    cross_gates: frozenset[CellRef] | None = None,
    mid_boundary: int = _MID_BOUNDARY,
    *,
    start_heading: str | None = None,
    final_heading: str | None = None,
    turn_45: float = 0.75,
    allow_diagonal: bool = True,
    crossing_penalty: float = 0.0,
    tunnel_dirs: dict[str, str] | None = None,
) -> tuple[str, ...]:
    """输出有序格号；转向按 turn_45/45° 计费，起点/终点朝向可约束。

    crossing_penalty：每次跨越挡板边界的附加代价（格当量，默认 0=关闭）。
    tunnel_dirs：桥洞隧道方向约束 {格号: 'E'/'W'/'EW'}——规则方向=桥顶箭头
    指向（E=西向东）；'EW'=方向未知时仍强制轴向（东西直行、洞内不转向）。
    默认 None=对 snapshot.passable 的全部隧道格自动施加 'EW' 轴向约束
    （物理必修：斜进一格宽桥洞会撞桥边）；传 {} 可完全关闭（遗留行为）。

    标靶格（红/蓝）物理上立着靶体，一律不可踩；每个蓝色敌方靶必须在
    其正交邻格停靠射击（车头正对靶心，UDP 朝向对准即打得准），
    停靠格作为必经点参与全局排序；红靶只挡路、不停靠、不打。
    """
    if start_heading is not None and start_heading not in _HEADING_IDX:
        raise ValueError(f"start_heading 只支持 'x'/'y'，收到 {start_heading!r}")
    if final_heading is not None and final_heading not in _HEADING_IDX:
        raise ValueError(f"final_heading 只支持 'x'/'y'，收到 {final_heading!r}")
    entry0 = (_HEADING_IDX[start_heading],) if start_heading else _ALL_HEADINGS
    h_final = _HEADING_IDX[final_heading] if final_heading else None

    # 隧道约束解析：方向 'E'/'W'（单向）或 'EW'（双向但强制轴向）。
    # 默认对所有 passable 隧道格强制 'EW'（物理上斜进必撞边，无需等规则方向）。
    tunnel_req: dict[int, int] | None = None
    tunnel_axes: frozenset[int] | None = None
    if tunnel_dirs is None:
        axes = {c.row * COLS + c.col for c in snapshot.passable}
        tunnel_axes = frozenset(axes) if axes else None
    elif tunnel_dirs:
        axes = set()
        tunnel_req = {}
        for name, d in tunnel_dirs.items():
            cn = name.strip().upper()
            dd = str(d).strip().upper()
            if dd not in ("E", "W", "EW"):
                raise ValueError(f"tunnel_dirs[{name!r}] 只支持 'E'/'W'/'EW'，收到 {d!r}")
            cell = CellRef(LETTERS.index(cn[0]), int(cn[1:]) - 1)
            if not (0 <= cell.row < ROWS and 0 <= cell.col < COLS):
                raise ValueError(f"tunnel_dirs 格号越界: {name!r}")
            ci = cell.row * COLS + cell.col
            axes.add(ci)
            if dd != "EW":
                tunnel_req[ci] = 0 if dd == "E" else 4
        tunnel_axes = frozenset(axes)
        if not tunnel_req:
            tunnel_req = None
    else:
        tunnel_axes = None     # 显式 {} = 全关（遗留行为，不推荐）

    cross_penalty = crossing_penalty  # 内部链路简称（对外参数 crossing_penalty）

    # 桥洞列推导：passable（ARROW）所在列即桥洞列。
    # 桥洞列物理上每格都有缺口，但规则上只有带箭头的格子允许通过，
    # 其余格视为墙。箭头格朝向挡板边界的一侧设1格引道（穿越后出口），
    # 另一侧不设引道——保证桥洞列南北不连通，形成两个独立半场。
    # 箭头位置每场随机。多个箭头之间不默认连通。
    passable_cells = tuple(snapshot.passable)
    passable_names = {c.name for c in passable_cells}
    bridge_cols = {c.col for c in passable_cells}
    bridge_open: set[CellRef] = set()       # 桥洞列中可通行的格
    bridge_walls: set[CellRef] = set()
    for col in bridge_cols:
        open_rows: set[int] = set()
        for c in passable_cells:
            if c.col != col:
                continue
            open_rows.add(c.row)            # 箭头格本身（隧道口）
            # 2026-10-15 现场模型修正：桥体其余格全部是结构体（不可踩），
            # 不再设"引道"——引道概念属于旧的"桥洞格南北穿越中线"模型，
            # 实测隧道是东西向穿越桥体（docs/分区规划与感知约束-20261015.md）。
        for row in range(ROWS):
            c = CellRef(col, row)
            if row in open_rows:
                bridge_open.add(c)
            else:
                bridge_walls.add(c)
    bridge_walls = frozenset(bridge_walls)
    # 标靶格一律不可踩（物理靶体）；移动靶的轨道两端同样不可踩（轨道本体）。
    target_cells = {t.cell.name: t.cell for t in snapshot.targets}
    track_cells = {c.name: c for t in snapshot.targets if t.track for c in t.track}
    blocked = (
        frozenset(snapshot.obstacles) | bridge_walls
        | frozenset(target_cells.values()) | frozenset(track_cells.values())
    )
    # 桥洞可通行格优先于桥洞墙（同一格不可能既是缺口又是墙）。
    blocked = frozenset(c for c in blocked if c.name not in passable_names)
    # 起点/终点格免疫封锁：车物理上就站在起点，终点是固定方框——
    # 这两格上的"障碍"只能是误检（边缘误检是 documented 顽疾），
    # 信任它会让规划器在自家脚下看到一堵墙。
    blocked = frozenset(c for c in blocked if c != start and c != goal)

    # 中场横向挡板的穿越缺口：
    # 1. 挡板范围外的列（A、B、I~N）可自由穿越边界；
    # 2. 桥洞列中位于边界附近的可通行格（箭头+引道）可穿越。
    # 若调用方未显式指定 cross_gates，则自动推导。
    if cross_gates is None:
        gates: set[CellRef] = set()
        for col in range(COLS):
            if col not in _BARRIER_COLS:
                for row in (mid_boundary - 1, mid_boundary):
                    gates.add(CellRef(col, row))
        # 桥洞列中边界附近的可通行格
        for c in bridge_open:
            if c.row in (mid_boundary - 1, mid_boundary):
                gates.add(c)
        if gates:
            cross_gates = frozenset(gates)

    # 雷为普通必经点；蓝靶的必经点是其停靠射击格（每靶一组候选）。
    # cache 在本次 plan_route 内共享：blocked/cross_gates 已定型，
    # 等最优顺序枚举与射击位组合会大量重复同一段 Dijkstra。
    cache: dict = {}

    base_required: dict[str, CellRef] = {}
    for c in snapshot.mines:
        base_required.setdefault(c.name, c)
    # 可通行桥洞格也是必经点：规则里"桥洞通道协同穿梭"每段 10 分（一区域 5 选 1、
    # 二区域 6 选 1，每区只计一次）——无人机读到的箭头格不穿过去就是白丢分。
    # 此前代码漏了这条（docstring 写了但实现没加），2026-10-15 随机图复盘发现补上。
    for c in snapshot.passable:
        base_required.setdefault(c.name, c)

    cand_lists: list[list[CellRef]] = []
    for t in snapshot.targets:
        if not t.is_blue:
            continue   # 红面可见（友方）或红靶：不打不停靠
        if t.track is not None:
            cands = _shoot_candidates_moving(t, blocked)
        else:
            cands = _shoot_candidates(t.cell, blocked, target_cells)
        if cands:
            cand_lists.append(cands)
        # 无候选 = 该靶被围死无处停靠，跳过（与不可达必经点语义一致）

    # 射击位候选按"起点可达性"预过滤：不可达候选直接移出组合枚举。
    # 否则某组合的候选不可达会被当成"该靶无可停靠"静默跳过，
    # 可能挤掉另一组合中可达候选的路线——白丢整组打击分。
    dist0 = None
    if cand_lists:
        dist0, _ = _cached_dijkstra(cache, start, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal,
                     cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
        cand_lists = [
            [c for c in cl if _arrival_state(dist0, c, None)[0] < _INF]
            for cl in cand_lists
        ]
        cand_lists = [cl for cl in cand_lists if cl]

    if not cand_lists:
        route, _ = _plan_core(start, goal, blocked, base_required, cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal, cache,
                              cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
        return route

    total = 1
    for cl in cand_lists:
        total *= len(cl)
    if total <= _SHOOT_COMBO_CAP:
        combos: list[tuple[CellRef, ...]] = list(product(*cand_lists))
    else:
        # 退化：逐靶选离起点最近的候选（大场景保护，实际靶数少不会触发）
        combos = [tuple(
            min(cl, key=lambda c: (_arrival_state(dist0, c, None)[0], c.name)) for cl in cand_lists
        )]

    best_route: tuple[str, ...] | None = None
    best_cost = _INF
    best_dup = 1 << 30
    for combo in combos:
        req = dict(base_required)
        for c in combo:
            req.setdefault(c.name, c)
        try:
            route, cost = _plan_core(start, goal, blocked, req, cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal, cache,
                                     cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
        except ValueError:
            continue
        dup = len(route) - len(set(route))
        better = cost < best_cost - _EPS or (abs(cost - best_cost) <= _EPS and dup < best_dup)
        if best_route is None or better:
            best_route, best_cost, best_dup = route, cost, dup
    if best_route is None:
        raise ValueError("goal cell unreachable from start")
    return best_route


def _norm_dir(a: tuple[int, int], b: tuple[int, int]) -> tuple[int, int]:
    """a→b 位移归一化到最简整数方向，用于共线判断。"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    g = gcd(abs(dx), abs(dy)) or 1
    return (dx // g, dy // g)


def compress_route(
    route: tuple[str, ...],
    keep: frozenset[str] | set[str] = frozenset(),
) -> tuple[str, ...]:
    """共线压缩：删掉"进入方向 == 离开方向"的中间格。

    车端按"相邻格号之间连直线段"行驶（官方协议"直线连接、可对角"），
    同向共线的中间格删掉后物理轨迹完全不变，命令 token 更短。
    永不删除：首尾格、所有拐点（方向变化处）、keep 中的格
    （雷/桥洞/靶等必经点与射击格必须保留为显式途经点）。
    """
    if len(route) < 3:
        return tuple(route)
    keep = frozenset(k.strip().upper() for k in keep)

    def pos(name: str) -> tuple[int, int]:
        return LETTERS.index(name[0]), int(name[1:]) - 1

    out: list[str] = [route[0]]
    for i in range(1, len(route) - 1):
        cur = route[i]
        if cur not in keep and _norm_dir(pos(route[i - 1]), pos(cur)) == _norm_dir(pos(cur), pos(route[i + 1])):
            continue  # 同向共线中间格：直线段自然经过其格心，无需单列
        out.append(cur)
    out.append(route[-1])
    return tuple(out)
