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

# 中场横向挡板：位于第6行与第7行的边界线上（row5↔row6 之间），
# 向上占第6行底部1/4、向下占第7行顶部1/4，总高半格。
# 第6行、第7行格子本身可通行，但跨边界的竖向移动被阻挡，桥洞缺口除外。
_MID_BOUNDARY = 6  # row5(第6行) ↔ row6(第7行) 之间


def _turn_units(h1: int, h2: int) -> int:
    """两个航向之间相差几个 45°。"""
    d = abs(h1 - h2)
    return min(d, 8 - d)


def _crosses_mid(row_a: int, row_b: int, boundary: int) -> bool:
    """一条竖向边是否跨越中场分界（boundary 指上半区行数，如 5 = row4↔row5 之间）。"""
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
) -> tuple[dict, dict]:
    """(格, 航向) 状态空间的 Dijkstra；返回 (dist, prev)。

    entry_headings 里的朝向以 0 代价起步（自由进入 = 不罚第一步转向）。
    terminal 指定时，该格成为终点态：进入即停、不许穿过再离开
    （避免“路过目标又折返”的路径）。
    """
    dist: dict[tuple[CellRef, int], float] = {}
    prev: dict[tuple[CellRef, int], tuple[CellRef, int]] = {}
    tick = count()
    pq: list[tuple[float, int, CellRef, int]] = []
    for h in entry_headings:
        dist[(start, h)] = 0.0
        heapq.heappush(pq, (0.0, next(tick), start, h))
    while pq:
        d, _, cur, h = heapq.heappop(pq)
        state = (cur, h)
        if dist.get(state, _INF) < d - 1e-12:
            continue
        if terminal is not None and cur == terminal:
            continue  # 终点态：不再向外扩展
        for nh, (dc, dr) in enumerate(_DIRS):
            diag = dc != 0 and dr != 0
            if diag and not allow_diagonal:
                continue
            nxt = CellRef(cur.col + dc, cur.row + dr)
            if not (0 <= nxt.col < COLS and 0 <= nxt.row < ROWS) or nxt in blocked:
                continue
            # 防切角：对角移动要求两侧正交格均非障碍
            if diag and (
                CellRef(cur.col + dc, cur.row) in blocked
                or CellRef(cur.col, cur.row + dr) in blocked
            ):
                continue
            if cross_gates is not None and dr != 0 and _crosses_mid(cur.row, nxt.row, mid_boundary):
                # 桥洞一格宽：只能正交穿越，且必须贴着 gate 格
                if diag or (cur not in cross_gates and nxt not in cross_gates):
                    continue
            nd = d + (_SQRT2 if diag else 1.0) + _turn_units(h, nh) * turn_45
            nstate = (nxt, nh)
            if nd < dist.get(nstate, _INF) - 1e-12:
                dist[nstate] = nd
                prev[nstate] = state
                heapq.heappush(pq, (nd, next(tick), nxt, nh))
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
) -> tuple[list[CellRef] | None, int | None, float]:
    """frm→to 单段路由：优先走避开已踩格子的等代价路径。返回 (cells, 到达航向, 代价)。"""
    if frm == to:
        h0 = entry[0] if len(entry) == 1 else (want_arrival if want_arrival is not None else 0)
        return [frm], h0, 0.0
    d_free, p_free = _dijkstra(frm, blocked, cross_gates, mid_boundary, entry, turn_45, allow_diagonal, terminal=to)
    cost, end = _arrival_state(d_free, to, want_arrival)
    if end is None or cost >= _INF:
        return None, None, _INF
    avoid = frozenset(blocked | (used - {frm, to}))
    if avoid != blocked:
        d_av, p_av = _dijkstra(frm, avoid, cross_gates, mid_boundary, entry, turn_45, allow_diagonal, terminal=to)
        c_av, e_av = _arrival_state(d_av, to, want_arrival)
        if e_av is not None and c_av <= cost + _EPS:
            return _trace_states(p_av, e_av), e_av[1], c_av
    return _trace_states(p_free, end), end[1], cost


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
) -> tuple[tuple[str, ...] | None, float]:
    """按给定顺序逐段拼接（真实朝向链 + 顺路避让），返回 (路线, 真实总代价)。"""
    used: set[CellRef] = {nodes[0]}
    segments: list[list[CellRef]] = []
    total = 0.0
    cur, entry = nodes[0], entry0
    for i in order:
        w = nodes[i + 1]
        seg, h, c = _route_leg(cur, w, blocked, used, cross_gates, mid_boundary, entry, None, turn_45, allow_diagonal)
        if seg is None:
            return None, _INF
        segments.append(seg)
        used.update(seg)
        total += c
        cur, entry = w, (h,)
    seg, _h, c = _route_leg(cur, goal, blocked, used, cross_gates, mid_boundary, entry, h_final, turn_45, allow_diagonal)
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
) -> tuple[tuple[str, ...], float]:
    """给定障碍集与必经点，输出（最优路线, 真实总代价）。"""
    dist0, _ = _dijkstra(start, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal)
    if _arrival_state(dist0, goal, h_final)[0] >= _INF:
        raise ValueError("goal cell unreachable from start")

    # 剔除不可达必经点（与旧语义一致：跳过）；nodes[0] 恒为起点。
    nodes = [start] + [c for c in required.values() if _arrival_state(dist0, c, None)[0] < _INF]
    n = len(nodes) - 1
    if n == 0:
        seg, _h, _c = _route_leg(start, goal, blocked, set(), cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal)
        return _flatten([seg]), _c

    # 每个节点做一次自由进入朝向的 Dijkstra，得两两边权（乐观界）。
    dists = [dist0] + [
        _dijkstra(c, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal)[0]
        for c in nodes[1:]
    ]
    W = [[_INF] * (n + 1) for _ in range(n + 1)]
    for i in range(n + 1):
        W[i][i] = 0.0
        for j in range(n + 1):
            if j != i:
                W[i][j] = _arrival_state(dists[i], nodes[j], None)[0]
    dgoal = [_arrival_state(dists[i], goal, h_final)[0] for i in range(n + 1)]

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
        return _assemble(order, nodes, goal, blocked, cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal)

    # 保底顺序 + 枚举其余等代价顺序，取（真实代价, 重复踩格）最小者。
    best_route: tuple[str, ...] | None = None
    best_cost_true: float = _INF
    best_dup = 1 << 30
    candidates = [_reconstruct_order(parent, full - 1, best_last)]
    candidates += _enumerate_optimal_orders(dp, W, dgoal, n, best_cost)
    for order in candidates:
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
    assert best_route is not None  # 保底顺序必然可拼接
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
) -> tuple[str, ...]:
    """输出有序格号；转向按 turn_45/45° 计费，起点/终点朝向可约束。

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

    # 桥洞墙自动推导：passable 是无人机识别到的带箭头可通行格（ARROW）。
    # 它们所在的列即桥洞列。桥洞是挡板上的缺口，小车需从引道驶入：
    # ARROW 格本身可通行，其上下各一格也视为引道可通行；列中其余格才是墙。
    # 若同一列有多个 ARROW，它们之间的整段连通。通行格位置每局随机。
    passable_cells = tuple(snapshot.passable)
    passable_names = {c.name for c in passable_cells}
    bridge_cols = {c.col for c in passable_cells}
    bridge_walls: set[CellRef] = set()
    for col in bridge_cols:
        col_arrows = sorted(c.row for c in passable_cells if c.col == col)
        lo, hi = col_arrows[0] - 1, col_arrows[-1] + 1
        lo = max(lo, 0); hi = min(hi, ROWS - 1)
        for row in range(ROWS):
            c = CellRef(col, row)
            if c.name in passable_names:
                continue
            if lo <= row <= hi:
                continue   # ARROW 及引道、多 ARROW 之间连通段
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

    # 中场横向挡板的桥洞缺口：passable 格即为允许穿越边界的位置。
    # 若调用方未显式指定 cross_gates，则自动用 passable 格作为缺口。
    if cross_gates is None and passable_cells:
        cross_gates = frozenset(passable_cells)

    # 雷为普通必经点；蓝靶的必经点是其停靠射击格（每靶一组候选）。
    base_required: dict[str, CellRef] = {}
    for c in snapshot.mines:
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

    if not cand_lists:
        route, _ = _plan_core(start, goal, blocked, base_required, cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal)
        return route

    total = 1
    for cl in cand_lists:
        total *= len(cl)
    if total <= _SHOOT_COMBO_CAP:
        combos: list[tuple[CellRef, ...]] = list(product(*cand_lists))
    else:
        # 退化：逐靶选离起点最近的候选（大场景保护，实际靶数少不会触发）
        dist0, _ = _dijkstra(start, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal)
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
            route, cost = _plan_core(start, goal, blocked, req, cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal)
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
