# mr_nav —— 由 mission_router.py 拆出（图搜索与路线内核：Dijkstra/状态回溯/Held-Karp序/核心规划）。与原文件逐行一致（AST 校验过）。
from __future__ import annotations

import heapq
import warnings
from itertools import count, product
from math import gcd

from .field_frame import CellRef, ROWS, COLS, LETTERS
from .fire_guard import blue_targets_hit, decide_fire
from .world_model import WorldSnapshot
from .mr_board import _CELLS, _EDGES, _IDIAG, _INEDGES, _MID_BOUNDARY, _N_CELLS, _SIDES, _TURNS, _crosses_mid, _turn_units, _zone_of
from .mr_params import _ALL_HEADINGS, _BIG_INT, _DIRS, _EPS, _GEAR_SWITCH, _HK_HEADING_MAX, _INF, _NEED_MM, _REV_MULT, _SPACE_W, _TSP_EXACT_MAX_NODES, _TURN_GRADE, _TURN_GUARD

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
    turn90_guard: bool = False,
    solids: dict[int, list[float]] | None = None,
    reverse: bool = False,
    rev_mult: float = _REV_MULT,
    gear_switch: float = _GEAR_SWITCH,
) -> tuple[dict, dict, dict]:
    """(格, 航向[, 挡位]) 状态空间的 Dijkstra；返回 (dist, prev, aux)。

    aux[(cell, h)] = (累计转向 45° 单位数, 累计步数)——同代价平局裁决用：
    代价相同时偏好更平直（转向少）再步数少的路线（步数/姿态平衡）。

    cross_penalty：每次跨越挡板边界附加的代价当量（默认 0=关闭，向后兼容），
    分区策略软约束——"一区扫完再转场"天然优于反复跨区。
    tunnel_axes：桥洞隧道格集合——进出必须同行正交（沿隧道轴东西向）、
    洞内禁止转向；一格宽桥洞斜进斜出 = 撞桥边（阿克曼底盘物理约束）。
    tunnel_req：隧道单向约束 {格索引: 行驶朝向}（0=向东 4=向西），
    通行方向=桥顶箭头指向，逆向入洞禁边。依据 docs/分区规划与感知约束-20261015.md。

    entry_headings 里的朝向以 0 代价起步（自由进入 = 不罚第一步转向）。
    terminal 指定时，该格成为终点态：进入即停、不许穿过再离开
    （避免“路过目标又折返”的路径）。

    2026-10-25 综合规划三项（默认全关 = 行为与旧版逐字节一致）：
      turn90_guard：90° 弯内切对角格被桶/挡板占用则删边（原模块级
        _TURN_GUARD 的运行时化；模块级开关仍生效，两者取或）。
      solids：实体挡板探测表（{格: 8 向 mm}）。给出时转弯可用空间按
        内侧正交 + 内切对角两向探测取小（桶=225、开阔=575、挡板=探测值），
        avail < need（_NEED_MM 物理判据）的边直接删除——45° 贴板、
        90° 贴桶这类"格占用模型看不见"的几何债在搜索内出清。
      reverse：倒车挡位原语。状态扩为 (格, 朝向, F/R)：R 态走镜像边
        （倒车遍历同一条物理弧，边级过滤同判），距离按 rev_mult 倍、
        换挡（F↔R 原地）按 gear_switch 计价。三点掉头由此自然涌现，
        替代旧"180° 转向按倒车计 ×1"的代价口径假设。

    平局裁决（2026-10-17 步数-姿态平衡）：代价、转向、步数三者沿路径
    可加且非负 → (代价, 转向, 步数) 字典序 Dijkstra 仍最优。turn_45=0
    （纯距离最短）时所有同距离形态全平局，本裁决保证取"转向最少 →
    步数最少"的最平直形态，不出锯齿路线；turn_45>0 时转向已计价，
    裁决只在精确补偿平局（如 Δ转向=4×0.75 ↔ Δ正交步=3）中生效。

    实现：整数态 (si = ci*8+h [+ N8 偏移= R 挡]) + 预构边表 + 位图障碍，
    出口处物化为原 {(CellRef, h): ...} 字典接口（消费方 _arrival_state/
    _trace_states 不变；挡位是搜索内部维度，同 (格,朝向) 双挡位取更优）。
    """
    blocked_bits = bytearray(_N_CELLS)
    for c in blocked:
        blocked_bits[c.row * COLS + c.col] = 1
    gates_bits: bytearray | None = None
    if cross_gates is not None:
        gates_bits = bytearray(_N_CELLS)
        for c in cross_gates:
            gates_bits[c.row * COLS + c.col] = 1
    # 进洞邻格（隧道格东西两侧一格）集合：dzbrain 实测依据（L10→L9 横向进=124 拍蹭墙、
    # M10→M9→L9→K9 沿轴进=0 拍）——桥室冗余墙向进洞侧突出 ~0.135m，换行（含对角）
    # 进出该格的车身余量仅 ~2cm。2026-10-24 移植为边级硬规则：禁换行触碰进洞邻格。
    entry_nbr: frozenset[int] | None = None
    if tunnel_axes:
        _en = set()
        for _ci in tunnel_axes:
            _r, _c = divmod(_ci, COLS)
            for _c2 in (_c - 1, _c + 1):
                if 0 <= _c2 < COLS:
                    _en.add(_r * COLS + _c2)
        entry_nbr = frozenset(_en) if _en else None
    term_ci = terminal.row * COLS + terminal.col if terminal is not None else -1
    start_ci = start.row * COLS + start.col

    guard_on = turn90_guard or _TURN_GUARD
    solid_on = solids is not None
    # 实体挡板行表：全格列表（无挡板的格 = None），热路径一次下标取行，
    # 避免 dict.get 哈希 + min(genexpr)（实测占增强模式 60%+ 耗时）。
    solid_rows: list[list[float] | None] = [None] * _N_CELLS
    if solid_on:
        for _ci, _row8 in solids.items():
            solid_rows[_ci] = _row8
    n8 = _N_CELLS * 8
    nst = n8 * 2 if reverse else n8
    dist_i = [_INF] * nst
    turns_i = [_BIG_INT] * nst   # 累计转向 45° 单位（平局裁决）
    steps_i = [_BIG_INT] * nst    # 累计步数（平局裁决）
    prev_i = [-1] * nst
    settled = bytearray(nst)
    tick = count()
    pq: list[tuple[float, int, int, int, int]] = []
    for h in entry_headings:
        si = start_ci * 8 + h
        dist_i[si] = 0.0
        turns_i[si] = 0
        steps_i[si] = 0
        heapq.heappush(pq, (0.0, 0, 0, next(tick), si))
    push, pop = heapq.heappush, heapq.heappop
    _grade, _sw = _TURN_GRADE, _SPACE_W   # 热路径预取（转角分级，V1019 移植）
    _sides, _need, _dirs = _SIDES, _NEED_MM, _DIRS

    while pq:
        d, t, s, _, si = pop(pq)
        if settled[si]:
            continue          # 字典序堆：首次出堆即最优，后续为陈旧副本
        settled[si] = 1
        if dist_i[si] < d - 1e-12:
            continue
        in_r = reverse and si >= n8
        core = si - n8 if in_r else si
        ci = core >> 3
        if ci == term_ci:
            continue  # 终点态：不再向外扩展
        h = core & 7
        row = ci // COLS
        if not in_r:
            # ---- F 挡：前进边（旧行为原样 + 守卫/实体挡板增强）----
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
                # 进洞邻格禁换行（2026-10-24，dzbrain 实测移植）：换行/对角进出
                # 进洞邻格 = 蹭桥室冗余墙（余量 ~2cm）；沿行（东西向）进出不受限。
                if entry_nbr is not None and dr and (ci in entry_nbr or nci in entry_nbr):
                    continue
                _tu = _TURNS[h][nh]
                if _tu and (guard_on or solid_on):
                    # 转弯空间判定：90° 守卫（旧口径：内切对角格被占/越界删边）
                    # 与实体挡板（新口径：内侧正交+内切对角探测，avail<need 删边）。
                    # 45° 快速通道：桶邻格 225 ≥ need 146 恒可行，只有实体挡板
                    # 能否决——本格无实体行的 45° 弯直接放行，不进探测循环。
                    if solid_on:
                        _srow = solid_rows[ci]
                        if _tu != 1 or _srow is not None:
                            av = 575.0
                            for dd in _sides[h][nh]:
                                av2 = _srow[dd] if _srow is not None else 575.0
                                if av2 >= 575.0:
                                    _cc2 = ci % COLS + _dirs[dd][0]
                                    _cr2 = row + _dirs[dd][1]
                                    if 0 <= _cr2 < ROWS and 0 <= _cc2 < COLS \
                                            and blocked_bits[_cr2 * COLS + _cc2]:
                                        av2 = 225.0
                                if av2 < av:
                                    av = av2
                            if av < _need[_tu]:
                                continue
                    elif guard_on and _tu == 2:
                        _cr = row + _dirs[h][1] + _dirs[nh][1]
                        _cc = ci % COLS + _dirs[h][0] + _dirs[nh][0]
                        if not (0 <= _cr < ROWS and 0 <= _cc < COLS):
                            continue
                        if blocked_bits[_cr * COLS + _cc]:
                            continue
                # 穿越挡板边界附加代价：分区策略软约束（docs/分区规划与感知约束）
                extra = 0.0
                if cross_penalty and dr and (
                        (row < mid_boundary <= nrow) or (nrow < mid_boundary <= row)):
                    extra = cross_penalty
                # 转角分级代价（V1019 移植）：90°×2、135°+×10（见 _TURN_GRADE 注释）。
                # _grade/_sw 循环外预取：内循环热路径尽量只碰局部名（求解器路径
                # O(n²) 次规划对每一点开销都敏感，见 test_worst_case_solver_planning_speed）。
                nd = d + step + _tu * turn_45 * _grade[_tu] + extra
                if _sw > 0.0 and _tu >= 2:
                    # 空间缺口代价：内切对角格被桶/挡板占用 → avail 仅 225mm
                    # （半格 175 + 桶边 50）；空着 → 575mm。与 turn_feasibility_v2 同判据
                    # （2026-10-25：实体挡板格优先取探测值，与守卫同一口径）。
                    _idg = _IDIAG[h][nh]
                    _av = 575.0
                    if _idg >= 0:
                        if solid_on:
                            _sr = solid_rows[ci]
                            if _sr is not None:
                                _av = _sr[_idg]
                        if _av >= 575.0:
                            _cc2 = ci % COLS + _dirs[_idg][0]
                            _cr2 = row + _dirs[_idg][1]
                            if 0 <= _cr2 < ROWS and 0 <= _cc2 < COLS \
                                    and blocked_bits[_cr2 * COLS + _cc2]:
                                _av = 225.0
                    nd += _sw * max(0.0, _need[_tu] - _av)
                nt = t + _tu
                nst_ = s + 1
                nsi = nci * 8 + nh
                cur_d = dist_i[nsi]
                # 主序：代价更小直接胜出；同代价平局比 (转向, 步数) 字典序
                better = nd < cur_d - 1e-12 or (
                    abs(nd - cur_d) <= 1e-12
                    and (nt, nst_) < (turns_i[nsi], steps_i[nsi])
                )
                if better:
                    dist_i[nsi] = nd
                    turns_i[nsi] = nt
                    steps_i[nsi] = nst_
                    prev_i[nsi] = si
                    push(pq, (nd, nt, nst_, next(tick), nsi))
            if reverse and gear_switch > 0.0:
                # F→R 换挡（原地：停车+换挡）
                nsi = n8 + ci * 8 + h
                nd = d + gear_switch
                if nd < dist_i[nsi] - 1e-12:
                    dist_i[nsi] = nd
                    turns_i[nsi] = t
                    steps_i[nsi] = s
                    prev_i[nsi] = si
                    push(pq, (nd, t, s, next(tick), nsi))
        else:
            # ---- R 挡：倒车镜像边 (ci,h,R)→(p,h_dep,R) ----
            # 底层前进边 (p,h_dep)→(ci,h)：同一条物理弧，边级过滤（防切角/
            # 桥门/隧道/进洞邻格）按该前进边同判；转向代价 = h_dep→h 在 p 处
            # 的转弯（镜像几何：同一片扫掠空间）。
            if gear_switch > 0.0:
                nsi = ci * 8 + h          # R→F 换挡
                nd = d + gear_switch
                if nd < dist_i[nsi] - 1e-12:
                    dist_i[nsi] = nd
                    turns_i[nsi] = t
                    steps_i[nsi] = s
                    prev_i[nsi] = si
                    push(pq, (nd, t, s, next(tick), nsi))
            for p_ci, trav, step, diag, o1, o2, dr, prow in _INEDGES[ci]:
                if trav != h or blocked_bits[p_ci]:
                    continue              # 倒车行驶方向 = 车尾方向（trav 必须 = 车头 h）
                if diag:
                    if not allow_diagonal:
                        continue
                    if blocked_bits[o1] or blocked_bits[o2]:
                        continue
                if (
                    dr and gates_bits is not None
                    and ((prow < mid_boundary <= row) or (row < mid_boundary <= prow))
                ):
                    if diag or not (gates_bits[p_ci] or gates_bits[ci]):
                        continue
                if tunnel_axes is not None:
                    if ci in tunnel_axes and dr != 0:
                        continue          # 镜像：倒车进入隧道格 ci 也只许同行正交
                    if p_ci in tunnel_axes:
                        prev_in_tunnel = True
                    else:
                        prev_in_tunnel = False
                else:
                    prev_in_tunnel = False
                if tunnel_req is not None:
                    req_h = tunnel_req.get(ci, -1)
                    if req_h >= 0 and h != req_h:
                        continue          # 镜像：倒车进入 ci 的行驶方向受单向约束
                if entry_nbr is not None and dr and (p_ci in entry_nbr or ci in entry_nbr):
                    continue
                extra = 0.0
                if cross_penalty and dr and (
                        (prow < mid_boundary <= row) or (row < mid_boundary <= prow)):
                    extra = cross_penalty
                for h_dep in range(8):
                    if prev_in_tunnel:
                        rev = (h_dep + 4) % 8
                        if h != h_dep and h != rev:
                            continue      # 镜像：隧道格上的倒车出弧只许直退/原路倒出
                    _tu = _TURNS[h_dep][h]
                    if _tu and (guard_on or solid_on):
                        # 同 F 挡口径（含 45° 无挡板快速通道），探测格 = 弧的
                        # 转弯发生格 p（镜像几何），方向对 = (h_dep → h)。
                        if solid_on:
                            _srow = solid_rows[p_ci]
                            if _tu != 1 or _srow is not None:
                                av = 575.0
                                for dd in _sides[h_dep][h]:
                                    av2 = _srow[dd] if _srow is not None else 575.0
                                    if av2 >= 575.0:
                                        _cc2 = p_ci % COLS + _dirs[dd][0]
                                        _cr2 = prow + _dirs[dd][1]
                                        if 0 <= _cr2 < ROWS and 0 <= _cc2 < COLS \
                                                and blocked_bits[_cr2 * COLS + _cc2]:
                                            av2 = 225.0
                                    if av2 < av:
                                        av = av2
                                if av < _need[_tu]:
                                    continue
                        elif guard_on and _tu == 2:
                            _cr = prow + _dirs[h_dep][1] + _dirs[h][1]
                            _cc = p_ci % COLS + _dirs[h_dep][0] + _dirs[h][0]
                            if not (0 <= _cr < ROWS and 0 <= _cc < COLS):
                                continue
                            if blocked_bits[_cr * COLS + _cc]:
                                continue
                    nd = d + step * rev_mult + _tu * turn_45 * _grade[_tu] + extra
                    nt = t + _tu
                    nst_ = s + 1
                    nsi = n8 + p_ci * 8 + h_dep
                    cur_d = dist_i[nsi]
                    better = nd < cur_d - 1e-12 or (
                        abs(nd - cur_d) <= 1e-12
                        and (nt, nst_) < (turns_i[nsi], steps_i[nsi])
                    )
                    if better:
                        dist_i[nsi] = nd
                        turns_i[nsi] = nt
                        steps_i[nsi] = nst_
                        prev_i[nsi] = si
                        push(pq, (nd, nt, nst_, next(tick), nsi))

    dist: dict[tuple[CellRef, int], float] = {}
    prev: dict[tuple[CellRef, int], tuple[CellRef, int]] = {}
    aux: dict[tuple[CellRef, int], tuple[int, int]] = {}
    _best_ts: dict[tuple[CellRef, int], tuple[int, int]] = {}
    for si in range(nst):
        dd = dist_i[si]
        if dd >= _INF:
            continue
        core = si - n8 if (reverse and si >= n8) else si
        st = (_CELLS[core >> 3], core & 7)
        ts = (turns_i[si], steps_i[si])
        cur = dist.get(st)
        if cur is None or (dd, ts) < (cur, _best_ts[st]):
            dist[st] = dd
            aux[st] = ts
            _best_ts[st] = ts
            p = prev_i[si]
            if p >= 0:
                pc = p - n8 if (reverse and p >= n8) else p
                prev[st] = (_CELLS[pc >> 3], pc & 7)
    return dist, prev, aux
def _arrival_state(
    dist: dict, cell: CellRef, want: int | None,
    aux: dict | None = None,
) -> tuple[float, tuple[CellRef, int] | None]:
    """到达 cell 的最省（代价, 终态）；want 指定到达航向。

    多个到达朝向同代价时用 aux[(cell,h)]=(累计转向, 步数) 平局裁决：
    取更平直（转向少）再步数少的到达态——等时路线里姿态更稳、
    航段更少（步数-姿态平衡，2026-10-17）。
    """
    if want is not None:
        return dist.get((cell, want), _INF), (cell, want)
    best = (_INF, None)
    best_key: tuple[int, int] = (_BIG_INT, _BIG_INT)
    for h in _ALL_HEADINGS:
        d = dist.get((cell, h), _INF)
        if d >= _INF:
            continue
        key = aux.get((cell, h), (_BIG_INT, _BIG_INT)) if aux is not None else (_BIG_INT, _BIG_INT)
        if (
            d < best[0] - 1e-12
            or (abs(d - best[0]) <= 1e-12 and key < best_key)
        ):
            best = (d, (cell, h))
            best_key = key
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
    turn90_guard: bool = False,
    solids: dict[int, list[float]] | None = None,
    solid_key: tuple | None = None,
    reverse: bool = False,
    rev_mult: float = _REV_MULT,
    gear_switch: float = _GEAR_SWITCH,
) -> tuple[dict, dict, dict]:
    """_dijkstra 的缓存封装：键 = (起点, 进入朝向集, 终点态, 障碍集, 增强项)。

    cross_gates/mid_boundary/turn_45/allow_diagonal 在一次 plan_route 调用内
    不变，由调用方保证；避让重算的 blocked 会变，故必须进键。
    cross_penalty/tunnel_axes/tunnel_req 同属本约定（一次调用内不变），不进键。
    2026-10-25 增强项（turn90_guard/solid_key/reverse/rev_mult/gear_switch）
    一次调用内同样不变，但会改变同一 (起点,障碍) 下的结果，必须进键——
    solids 本体是 dict 不可哈希，进键的是它的排序元组投影 solid_key。
    """
    key = (start, entry_headings, terminal, blocked,
           turn90_guard, solid_key, reverse, rev_mult, gear_switch)
    if cache is not None and key in cache:
        return cache[key]
    result = _dijkstra(start, blocked, cross_gates, mid_boundary, entry_headings, turn_45, allow_diagonal, terminal=terminal,
                       cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req,
                       turn90_guard=turn90_guard, solids=solids,
                       reverse=reverse, rev_mult=rev_mult, gear_switch=gear_switch)
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
    face_idx: int | None = None,
    turn90_guard: bool = False,
    solids: dict[int, list[float]] | None = None,
    solid_key: tuple | None = None,
    reverse: bool = False,
    rev_mult: float = _REV_MULT,
    gear_switch: float = _GEAR_SWITCH,
) -> tuple[list[CellRef] | None, int | None, float]:
    """frm→to 单段路由：优先走避开已踩格子的等代价路径。返回 (cells, 到达航向, 代价)。

    2026-10-25 增强项（turn90_guard/solids/reverse 等）原样透传给底层
    Dijkstra——搜索内判可行性，不再只靠事后体检。

    face_idx：射击停靠段的正对朝向（_DIRS 索引）。给出时按
    "到达朝向 + 停靠转身"联合最省选择到达态（朝向零转向准则：
    绕一格正对到达优于直进后原地 90° 转身），代价含转身。

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
    _enh = dict(turn90_guard=turn90_guard, solids=solids, solid_key=solid_key,
                reverse=reverse, rev_mult=rev_mult, gear_switch=gear_switch)
    d_free, p_free, a_free = _cached_dijkstra(cache, frm, blocked, cross_gates, mid_boundary, entry, turn_45, allow_diagonal, terminal=term,
                                      cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req, **_enh)

    def _pick(dist_map, aux_map) -> tuple[float, tuple[CellRef, int] | None]:
        if face_idx is not None and want_arrival is None:
            # 停靠段：全部到达朝向按"路程+停靠转身"联合计价取最省
            # （朝向零转向准则：绕一格正对到达优于直进后原地 90° 转身）
            best = None
            best_end: tuple[CellRef, int] | None = None
            best_key: tuple[int, int] = (_BIG_INT, _BIG_INT)
            for hh in _ALL_HEADINGS:
                c0 = dist_map.get((to, hh), _INF)
                if c0 >= _INF:
                    continue
                c1 = c0 + _TURNS[hh][face_idx] * turn_45
                # 同代价平局：行进过程更平直（转向少）/步数少的到达态优先
                key = aux_map.get((to, hh), (_BIG_INT, _BIG_INT)) if aux_map is not None else (_BIG_INT, _BIG_INT)
                if (
                    best is None or c1 < best - 1e-12
                    or (abs(c1 - best) <= 1e-12 and key < best_key)
                ):
                    best = c1
                    best_key = key
                    best_end = (to, hh)
            if best is not None and best_end is not None:
                return best, best_end
        return _arrival_state(dist_map, to, want_arrival, aux_map)

    cost, end = _pick(d_free, a_free)
    if end is None or cost >= _INF:
        return None, None, _INF

    avoid_cells = used - {frm, to}
    free_cells = _trace_states(p_free, end)
    if avoid_cells and not (set(free_cells) & avoid_cells):
        # 自由路径没有踩任何已踩格：避让重算的结果必然相同，直接返回
        return free_cells, end[1], cost

    avoid = frozenset(blocked | avoid_cells)
    if avoid != blocked:
        d_av, p_av, _ = _cached_dijkstra(cache, frm, avoid, cross_gates, mid_boundary, entry, turn_45, allow_diagonal, terminal=term,
                                  cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req, **_enh)
        c_av, e_av = _pick(d_av, None)
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
    face_req: dict[str, int] | None = None,
    align_degraded: set[str] | None = None,
    turn90_guard: bool = False,
    solids: dict[int, list[float]] | None = None,
    solid_key: tuple | None = None,
    reverse: bool = False,
    rev_mult: float = _REV_MULT,
    gear_switch: float = _GEAR_SWITCH,
) -> tuple[tuple[str, ...] | None, float]:
    """按给定顺序逐段拼接（真实朝向链 + 顺路避让），返回 (路线, 真实总代价)。

    2026-10-25 增强项（turn90_guard/solids/reverse 等）透传给每一段
    _route_leg——段与段之间朝向链已真实传递，增强项保证段内几何一致。

    face_req：射击停靠格的"正对朝向"要求（格名 -> _DIRS 朝向索引）。
    靶向对齐（2026-10-23）：停靠段**强制先试**"到达朝向 = 正对朝向"（一次
    到位）——官方 path_executor 只跟折线、不知道靶在哪，开火瞬间的车头
    朝向只能由进段几何保证；终点态约束下的 Dijkstra 自动绕/曲进对齐
    （"逐渐调整"的几何形态由此涵盖：进段可多折几手 45° 小弯逐步转过来），
    到达即正对、无停靠转身，代价口径也更真实。对齐到达不可行（对侧进段
    被堵）才退回"代价择优 + 停靠转身"假设（残余偏差交执行器停转-爬行/
    云台消化），并把格名记入 align_degraded 供上层一次性告警。

    fallback_any_heading：终点朝向约束（h_final）物理不可达时
    （例：终点格是口袋入口，朝向到达须先踩终点），允许以任意朝向进终点——
    放弃终点朝向分，保住完赛。严格模式（False）下不可达即整段返回 None。
    """
    used: set[CellRef] = {nodes[0]}
    _aenh = dict(turn90_guard=turn90_guard, solids=solids, solid_key=solid_key,
                 reverse=reverse, rev_mult=rev_mult, gear_switch=gear_switch)
    segments: list[list[CellRef]] = []
    total = 0.0
    cur, entry = nodes[0], entry0
    for i in order:
        w = nodes[i + 1]
        fidx = face_req.get(w.name) if face_req else None
        seg = None
        h = None
        c = _INF
        if fidx is not None:
            # 一次到位：终点态约束（到达朝向=正对朝向）的进段。
            seg, h, c = _route_leg(cur, w, blocked, used, cross_gates, mid_boundary,
                                   entry, fidx, turn_45, allow_diagonal, cache,
                                   cross_penalty, tunnel_axes, tunnel_req, **_aenh)
        if seg is None:
            # 普通段（fidx=None）的主路径；射击段则是"逐渐调整"兜底——对齐
            # 到达不可行（对侧进段被堵），退回代价择优 + 停靠转身假设，并
            # 记录降级格供上层一次性告警。
            seg, h, c = _route_leg(cur, w, blocked, used, cross_gates, mid_boundary,
                                   entry, None, turn_45, allow_diagonal, cache,
                                   cross_penalty, tunnel_axes, tunnel_req,
                                   face_idx=fidx, **_aenh)
            if seg is not None and fidx is not None and align_degraded is not None:
                align_degraded.add(w.name)
        if seg is None:
            return None, _INF
        segments.append(seg)
        used.update(seg)
        total += c
        # 射击停靠后车头已转向正对朝向（打完后从该朝向继续行驶）
        cur = w
        entry = (face_req[w.name],) if (face_req and w.name in face_req) else (h,)
    seg, _h, c = _route_leg(cur, goal, blocked, used, cross_gates, mid_boundary, entry, h_final, turn_45, allow_diagonal, cache,
                               cross_penalty, tunnel_axes, tunnel_req, **_aenh)
    if seg is None and fallback_any_heading and h_final is not None:
        seg, _h, c = _route_leg(cur, goal, blocked, used, cross_gates, mid_boundary, entry, None, turn_45, allow_diagonal, cache,
                               cross_penalty, tunnel_axes, tunnel_req, **_aenh)
    if seg is None:
        return None, _INF
    segments.append(seg)
    total += c
    return _flatten(segments), total
def _classify_blocked(cell: CellRef, diag: dict | None) -> str:
    """封锁格归类（诊断用）：障碍/靶体/轨道/桥墙/封锁。"""
    if diag:
        if cell.name in diag.get("obstacles", ()):
            return "障碍"
        if cell.name in diag.get("targets", ()):
            return "靶体"
        if cell.name in diag.get("tracks", ()):
            return "轨道"
        if cell.col in diag.get("bridge_cols", ()):
            return "桥墙"
    return "封锁"
def _unreachable_detail(start: CellRef, goal: CellRef, dist0: dict,
                        blocked: frozenset[CellRef],
                        cross_gates: frozenset[CellRef] | None,
                        mid_boundary: int, diag: dict | None) -> str:
    """不可达诊断（2026-10-17 修复）：起点连通域规模 + 封锁点分类 +
    开放桥门——现场拿到报错就知道该查哪块板/哪个桶。

    封锁点 = 与起点连通域 8 邻接的 blocked 格；连通域取自 Dijkstra
    dist0（任意朝向可达即算）。"""
    reach = {c for (c, _h), d in dist0.items() if d < _INF} or {start}
    sealing: dict[str, set[str]] = {}
    for c in reach:
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                if dc == dr == 0:
                    continue
                col, row = c.col + dc, c.row + dr
                if not (0 <= col < COLS and 0 <= row < ROWS):
                    continue
                n = CellRef(col, row)
                if n in reach or n not in blocked:
                    continue
                sealing.setdefault(_classify_blocked(n, diag), set()).add(n.name)
    parts = [f"goal {goal.name} 不可达：起点 {start.name} 连通域仅 {len(reach)} 格"]
    for kind in ("障碍", "靶体", "轨道", "桥墙", "封锁"):
        if sealing.get(kind):
            names = sorted(sealing[kind])
            parts.append(f"{kind}[{','.join(names[:8])}{'…' if len(names) > 8 else ''}]")
    if diag:
        open_gates = sorted(diag.get("open_gates", ()))
        if open_gates:
            parts.append(f"桥门开放[{','.join(open_gates)}]")
        elif diag.get("bridge_cols"):
            parts.append("桥门全部关闭（箭头格缺失或被实物占用）")
    if cross_gates is not None:
        parts.append(f"中线穿越口 {len(cross_gates)} 格")
    parts.append("排查：门格是否被桶占用、箭头格是否漏检、障碍是否误检堵死唯一通道")
    return "；".join(parts)
def _greedy_orders(W: list[list[float]], n: int) -> list[list[int]]:
    """多起点最近邻访问顺序（Held-Karp 超限时用）。

    W 下标与 DP 同口径：0=起点，k+1=必经点 k。每条 O(n^2)，n 条共 O(n^3)，
    n=20 也是微秒级。多起点比单起点稳——单起点在"起点位于图边缘"时容易
    一路走回头路，而这里只能拼一次（不像精确解有等代价顺序可枚举）。
    """
    orders: list[list[int]] = []
    for first in range(n):
        unvisited = set(range(n))
        unvisited.discard(first)
        order = [first]
        cur = first
        while unvisited:
            nxt = min(unvisited, key=lambda j: (W[cur + 1][j + 1], j))
            order.append(nxt)
            unvisited.discard(nxt)
            cur = nxt
        orders.append(order)
    return orders
def _held_karp_orders(W: list[list[float]], dgoal: list[float],
                      n: int) -> list[list[int]]:
    """Held-Karp 精确排序 -> 候选访问顺序（保底顺序 + 全部等代价顺序）。

    dp[mask][i] = 从起点出发、恰好经过 mask 内必经点、当前停在 i 的最少代价。
    必经点编号 0..n-1 对应 nodes[1..n]；W/dgoal 下标 0 = 起点。
    调用方须先保证 n <= _TSP_EXACT_MAX_NODES：状态数是 2^n × n。
    """
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
    if best_last < 0:
        # 全集没有任何可行顺序（单向隧道等方向约束把所有排列都堵死）。
        # 此时 best_last=-1 会回溯出**空顺序**，拼装成"起点直奔终点、
        # 跳过全部雷与桥洞"——正是最坏的静默灾难，改走贪心候选并告警。
        warnings.warn(
            f"{n} 个必经点无可行访问顺序（乐观代价全为不可达），退化为"
            "最近邻顺序——检查单向隧道方向是否标反",
            RuntimeWarning, stacklevel=4,
        )
        return _greedy_orders(W, n)
    return ([_reconstruct_order(parent, full - 1, best_last)]
            + _enumerate_optimal_orders(dp, W, dgoal, n, best_cost))
def _held_karp_orders_heading(
    cache: dict | None,
    nodes: list[CellRef],
    goal: CellRef,
    entry0: tuple[int, ...],
    h_final: int | None,
    blocked, cross_gates, mid_boundary, turn_45, allow_diagonal,
    cross_penalty, tunnel_axes, tunnel_req, aenh: dict,
) -> list[list[int]]:
    """朝向传递 Held-Karp（2026-10-25）：边权带 (进入朝向, 到达朝向) 二元组。

    旧口径边权 = 自由进入朝向的乐观界——必经点拼接处可能出现"上一段到达
    朝向逼出下一段急弯/U 形回头"的形态，拼装层只能被动消化。本 DP 把到达
    朝向放进状态 (mask, last, h)：
        dp[mask][i][h] = 恰好经过 mask、停在 i、**以朝向 h 到达**的最少代价；
        转移 = dp + W[i][h][j][h2]（i 处不停车，到达朝向即出发朝向）；
        终局 = dp[full][i][h] + dgoal(i, h)（h_final 约束在 goal 段生效）。
    每节点 8 次固定进入朝向 Dijkstra（共享 _cached_dijkstra 缓存）。
    返回候选顺序（保底一条）；全不可行返回 []（调用方退回乐观界候选）。
    """
    m = len(nodes)              # 0 = 起点，1..n = 必经点
    n = m - 1
    INF = _INF
    targets = list(nodes) + [goal]
    # Wh[i][h_in][j][h_out]：i 以 h_in 进入、j 以 h_out 到达的段代价；j=m 是 goal
    Wh: list[list[list[list[float]]]] = [[None] * 8 for _ in range(m)]
    for i in range(m):
        for h_in in range(8):
            dist, _, _ = _cached_dijkstra(
                cache, nodes[i], blocked, cross_gates, mid_boundary, (h_in,),
                turn_45, allow_diagonal, cross_penalty=cross_penalty,
                tunnel_axes=tunnel_axes, tunnel_req=tunnel_req, **aenh)
            Wh[i][h_in] = [
                [dist.get((t, h_out), INF) for h_out in range(8)] for t in targets
            ]
    full = 1 << n
    dp = [[[INF] * 8 for _ in range(n)] for _ in range(full)]
    par: list[list[list[tuple[int, int] | None]]] = [
        [[None] * 8 for _ in range(n)] for _ in range(full)
    ]
    mask_alive = bytearray(full)          # 有有限态的 mask 快速跳过（热路径）
    for j in range(n):
        for h_out in range(8):
            dp[1 << j][j][h_out] = min(
                Wh[0][h0][j + 1][h_out] for h0 in entry0)
        mask_alive[1 << j] = 1
    for mask in range(1, full):
        if not mask_alive[mask]:
            continue
        dpm = dp[mask]
        free_k = [k for k in range(n) if not (mask >> k) & 1]
        for i in range(n):
            if not (mask >> i) & 1:
                continue
            base = dpm[i]
            Wi = Wh[i + 1]
            for h in range(8):
                cur = base[h]
                if cur >= INF:
                    continue
                Wih = Wi[h]
                for k in free_k:
                    row = Wih[k + 1]
                    nm = mask | (1 << k)
                    dpk = dp[nm][k]
                    park = par[nm][k]
                    for h2 in range(8):
                        w = row[h2]
                        if w >= INF:
                            continue
                        nd = cur + w
                        if nd < dpk[h2] - 1e-12:
                            dpk[h2] = nd
                            park[h2] = (i, h)
                            mask_alive[nm] = 1
    best_cost, best_end = INF, None
    for i in range(n):
        dgh = Wh[i + 1]          # 从 i 出发的边权：[h_in][m=goal][h_out]
        for h in range(8):
            cur = dp[full - 1][i][h]
            if cur >= INF:
                continue
            if h_final is not None:
                g = dgh[h][m][h_final]
            else:
                g = min(dgh[h][m])
            if g >= INF:
                continue
            c = cur + g
            if c < best_cost:
                best_cost, best_end = c, (i, h)
    if best_end is None:
        return []
    order: list[int] = []
    mask, (i, h) = full - 1, best_end
    while i >= 0:
        order.append(i)
        p = par[mask][i][h]
        if p is None:
            break
        mask ^= 1 << i
        i, h = p
    order.reverse()
    return [order]
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
    face_req: dict[str, int] | None = None,
    diag: dict | None = None,
    align_degraded: set[str] | None = None,
    turn90_guard: bool = False,
    solids: dict[int, list[float]] | None = None,
    solid_key: tuple | None = None,
    reverse: bool = False,
    rev_mult: float = _REV_MULT,
    gear_switch: float = _GEAR_SWITCH,
    heading_prop: bool = False,
) -> tuple[tuple[str, ...], float]:
    """给定障碍集与必经点，输出（最优路线, 真实总代价）。
    diag：不可达诊断上下文（障碍/靶/轨道名集 + 桥列/开放门），由 plan_route 提供。
    align_degraded：靶向对齐降级格收集集（见 _assemble），None 表示不需要收集。
    2026-10-25 增强项：turn90_guard/solids/reverse 进每层搜索；heading_prop
    在 n ≤ _HK_HEADING_MAX 时用朝向传递 Held-Karp 选序（见 _held_karp_orders_heading）。"""
    _aenh = dict(turn90_guard=turn90_guard, solids=solids, solid_key=solid_key,
                 reverse=reverse, rev_mult=rev_mult, gear_switch=gear_switch)
    dist0, _, _ = _cached_dijkstra(cache, start, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal,
                     cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req, **_aenh)
    if _arrival_state(dist0, goal, None)[0] >= _INF:
        raise ValueError(_unreachable_detail(
            start, goal, dist0, blocked, cross_gates, mid_boundary, diag))
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
                             cross_penalty, tunnel_axes, tunnel_req, **_aenh)
        if seg is None and h_final is not None:
            # 终点朝向到达不可行（口袋地形）：降级任意朝向进终点
            warnings.warn(
                f"goal {goal.name} 以朝向到达不可行，降级为任意朝向进终点"
                "（终点朝向 5 分不保）",
                RuntimeWarning, stacklevel=3,
            )
            seg, _h, _c = _route_leg(start, goal, blocked, set(), cross_gates, mid_boundary, entry0, None, turn_45, allow_diagonal, cache,
                                 cross_penalty, tunnel_axes, tunnel_req, **_aenh)
        if seg is None:
            raise ValueError(_unreachable_detail(
                start, goal, dist0, blocked, cross_gates, mid_boundary, diag))
        return _flatten([seg]), _c

    # 每个节点做一次自由进入朝向的 Dijkstra，得两两边权（乐观界）。
    # aux 供等代价边权平局裁决（姿态更平直的到达态优先）。
    _maps = [
        _cached_dijkstra(cache, c, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal,
                          cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req, **_aenh)
        for c in [start] + nodes[1:]
    ]
    dists = [m[0] for m in _maps]
    auxs = [m[2] for m in _maps]
    W = [[_INF] * (n + 1) for _ in range(n + 1)]
    for i in range(n + 1):
        W[i][i] = 0.0
        for j in range(n + 1):
            if j != i:
                W[i][j] = _arrival_state(dists[i], nodes[j], None, auxs[i])[0]
    dgoal = [_arrival_state(dists[i], goal, h_final)[0] for i in range(n + 1)]
    if h_final is not None and all(d >= _INF for d in dgoal):
        # 终点朝向到达全局不可行（口袋地形）：TSP 排序改用任意朝向界，
        # 朝向降级由拼装层兜底——否则 dgoal 全 INF 会导致回溯出空顺序，
        # 退化成"起点直奔终点、跳过全部雷"的灾难路线。
        dgoal = [_arrival_state(dists[i], goal, None)[0] for i in range(n + 1)]

    # 访问顺序候选：n 在上限内走 Held-Karp 精确解，超限退化为最近邻。
    if n > _TSP_EXACT_MAX_NODES:
        warnings.warn(
            f"必经点 {n} 个超过精确 TSP 上限 {_TSP_EXACT_MAX_NODES}，退化为"
            "最近邻顺序（路线次优）——检查感知是否把大片格误判成必经点",
            RuntimeWarning, stacklevel=3,
        )
        candidates = _greedy_orders(W, n)
    else:
        candidates = _held_karp_orders(W, dgoal, n)
        if heading_prop and 1 < n <= _HK_HEADING_MAX:
            # 朝向传递候选置前（拼接循环按序取前 64 条）：它按"真实朝向链"
            # 选序，拼接真实代价不劣于乐观界口径；失败/超限时静默退回旧候选。
            # 内部 Dijkstra 用轻量边模型（关倒车挡）：本 DP 只是候选顺序生成器，
            # 最终定序与可行性一律由 _assemble 的全模型真实代价裁决——倒车态
            # 让单次 Dijkstra 翻倍（实测 2ms→6ms），候选层省掉不损正确性。
            try:
                horders = _held_karp_orders_heading(
                    cache, nodes, goal, entry0, h_final, blocked, cross_gates,
                    mid_boundary, turn_45, allow_diagonal, cross_penalty,
                    tunnel_axes, tunnel_req,
                    {k: v for k, v in _aenh.items() if k != "reverse"})
            except Exception:
                horders = []
            if horders:
                seen = {tuple(c) for c in candidates}
                candidates = horders + [c for c in candidates if tuple(c) not in seen]

    def assemble(order: list[int]) -> tuple[tuple[str, ...] | None, float]:
        return _assemble(order, nodes, goal, blocked, cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal, cache,
                         cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req, face_req=face_req,
                         align_degraded=align_degraded, **_aenh)

    # 保底顺序 + 枚举其余等代价顺序，取（真实代价, 重复踩格, 姿态形态）最小者。
    # 形态键 = (总转向 45° 单位, 总步数)：等时代价里更平直、航段更少者优先
    # （步数-姿态平衡，2026-10-17）。拼接数硬上限：对称场景下等代价顺序
    # 可达 256 条；64 次足以覆盖真实的并列分支（实测场景 <10 次即收敛）。
    best_route: tuple[str, ...] | None = None
    best_cost_true: float = _INF
    best_dup = 1 << 30
    best_shape: tuple[int, int] = (_BIG_INT, _BIG_INT)
    for order in candidates[:64]:
        route, cost_true = assemble(order)
        if route is None:
            continue
        dup = len(route) - len(set(route))
        shape = _route_shape(route)
        key_better = cost_true < best_cost_true - _EPS or (
            abs(cost_true - best_cost_true) <= _EPS
            and (dup, shape) < (best_dup, best_shape)
        )
        if best_route is None or key_better:
            best_route, best_cost_true, best_dup, best_shape = route, cost_true, dup, shape
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
                face_req=face_req, align_degraded=align_degraded, **_aenh,
            )
            if route is None:
                continue
            dup = len(route) - len(set(route))
            shape = _route_shape(route)
            key_better = cost_true < best_cost_true - _EPS or (
                abs(cost_true - best_cost_true) <= _EPS
                and (dup, shape) < (best_dup, best_shape)
            )
            if best_route is None or key_better:
                best_route, best_cost_true, best_dup, best_shape = route, cost_true, dup, shape
    if best_route is None:
        # 所有候选顺序都拼不出完整路线。原来是 `assert best_route is not None`：
        # `python -O` 会把 assert 整条删掉，None 便顺着 plan_route 流进
        # ensure_mine_center_pass 再到 commands.route，现场表现为"车收到一条
        # 莫名其妙的路线"而不是一个报错。抛 ValueError 则能被 plan_route 的
        # 射击位组合循环 `except ValueError: continue` 接住，换下一组合重试。
        raise ValueError(
            f"{len(candidates[:64])} 条候选访问顺序均无法拼接完整路线："
            f"start={start.name} goal={goal.name} 必经点={n} "
            f"障碍={len(blocked)}。终点本身可达（前置检查已过），"
            "多半是必经点之间的朝向/隧道约束把每一段都堵死；"
            "排查单向桥洞方向是否标反、必经点是否落在不可踩格上"
        )
    return best_route, best_cost_true
_ORTHO_STEPS = ((1, 0), (-1, 0), (0, 1), (0, -1))
def _pos(name: str) -> tuple[int, int]:
    """格号 -> (col, row)（0 基）。"""
    return LETTERS.index(name[0]), int(name[1:]) - 1
# 单位位移 -> 航向索引（_route_shape 用；只覆盖 8 邻域单位步）
_DIR_INDEX: dict[tuple[int, int], int] = {d: i for i, d in enumerate(_DIRS)}
def _route_shape(route: tuple[str, ...]) -> tuple[int, int]:
    """路线形态键 (总转向 45° 单位数, 总步数)——等代价平局裁决用。

    从相邻格位移推航向链、统计相邻航向差。首步的起步转向不计入
    （起点朝向对各候选一致，口径公平）。射击停靠格上的原地转身
    不在几何航向链里，但同局各候选的停靠集相同，不影响比较结论。
    """
    turns = 0
    prev_h = -1
    for i in range(1, len(route)):
        a, b = _pos(route[i - 1]), _pos(route[i])
        h = _DIR_INDEX.get((b[0] - a[0], b[1] - a[1]), -1)
        if prev_h >= 0 and h >= 0 and h != prev_h:
            turns += _TURNS[prev_h][h]
        if h >= 0:
            prev_h = h
    return turns, max(len(route) - 1, 0)
def ensure_mine_center_pass(
    route: tuple[str, ...],
    mine_names: frozenset[str] | set[str],
    blocked: frozenset[CellRef],
    mid_boundary: int = _MID_BOUNDARY,
    cross_gates: frozenset[CellRef] | None = None,
    tunnel_names: frozenset[str] | None = None,
) -> tuple[str, ...]:
    """雷格过心约束：>45° 转向处的雷格补一格"过心穿越点"。

    排雷判据是"任意轮中心进入雷标 12cm 圆"（docs/执行层深优化-20261016
    问题 3）。executor 的 arrival_threshold ≈ 0.157m：雷格处于路线拐点时，
    车在距雷心 15.7cm 处即宣布到点并切向下一段，物理轨迹切角——轮中心
    可能从未进入 12cm 圆，排雷失败。直线途经的雷格天然过心（纯追踪贴线），
    45° 及以下拐角由拐角保护（lookahead 钳顶点）自然贴顶点过弯（主证明
    路线实测），只有 >45° 拐角触发 PIVOT 停转-爬行、顶点不再保证到达
    ——本函数只修 >45°（PIVOT 阈值同口径）。

    修复：雷格处转向 >45° 时，沿进入方向在雷后补一格过心穿越点——雷格
    变成直线段中间点（必过心），转向推迟到过心穿越点（普通格，无判分风险）。

    补点等效改写两条边（雷→补点、补点→原后继），必须逐条过 Dijkstra
    同款边界规则，任何一条不过就放弃补点（该雷退化为拐点，尽力而为）：
    - 越界 / 落在障碍格上；
    - 补点或雷格落在 K 列（桥体结构 + 车端 BRIDGE_BLOCKS 强制直行
      保护：在 K 列补点 = 以非轴向进桥/在保护点上转向，SIL 实测引发
      直行接管后失控出图）；
    - 对角边两侧正交格须非障碍（防切角）；
    - 跨挡板边界：只许正交竖向且端点有 gate（无 gate 建模时不查）——
      典型反例 K7 贴挡板雷，向北补点会穿挡板，必须拒绝；
    - 补点/原后继落在桥洞隧道格时只许东西向进出（洞内转向=撞桥边）。
    补点→原后继只接受 8 邻接几何（保持路线逐格 8 邻接约定，fuzz 不变量
    锁死）；骑士步/2 格长弦一概拒绝。

    逐格左→右扫描：补点本身也可能是雷（连环雷），其"实际进入方向"以补点
    为前驱计算，天然收敛（每次补点只增加一格）。
    """
    if len(route) < 3:
        return tuple(route)
    mines = {m.strip().upper() for m in mine_names}
    if not mines:
        return tuple(route)
    blocked_names = {c.name for c in blocked}
    gates_names = ({g.name for g in cross_gates} if cross_gates is not None
                   else None)
    tunnel = tunnel_names or frozenset()
    # 桥体列（K，col 10）：场地固定结构，2026-10-15 现场核实。
    _BRIDGE_COL = LETTERS.index("K")

    def _cell_ok(c: CellRef) -> bool:
        return (0 <= c.col < COLS and 0 <= c.row < ROWS
                and c.name not in blocked_names)

    def _diag_sides_ok(a: CellRef, b: CellRef) -> bool:
        """对角边 a→b 的防切角：两侧正交格均须在界内且非障碍。"""
        dc, dr = b.col - a.col, b.row - a.row
        for s in (CellRef(a.col + dc, a.row), CellRef(a.col, a.row + dr)):
            if not _cell_ok(s):
                return False
        return True

    def _leg_ok(a: CellRef, b: CellRef) -> bool:
        """单条 8 邻接边 a→b 过 Dijkstra 同款边界规则。"""
        if not (_cell_ok(a) and _cell_ok(b)):
            return False
        dc, dr = b.col - a.col, b.row - a.row
        diag = dc != 0 and dr != 0
        if diag and not _diag_sides_ok(a, b):
            return False
        # 跨挡板：正交竖向 + 端点是 gate（无 gate 建模时挡板不存在，不查）
        if (cross_gates is not None and dr != 0
                and _crosses_mid(a.row, b.row, mid_boundary)):
            if diag:
                return False
            if not (a.name in gates_names or b.name in gates_names):
                return False
        # 隧道格：只许东西向进出（含洞内直行），竖向=撞桥边
        if dr != 0 and (a.name in tunnel or b.name in tunnel):
            return False
        return True

    work = [c.strip().upper() for c in route]
    out: list[str] = [work[0]]
    for i in range(1, len(work)):
        cur = work[i]
        if cur in mines and i + 1 < len(work):
            mine = CellRef(*_pos(cur))
            nxt = CellRef(*_pos(work[i + 1]))
            d_in = _norm_dir(_pos(out[-1]), _pos(cur))
            d_out = _norm_dir(_pos(cur), _pos(work[i + 1]))
            # 只有相邻单步方向的转向角才有定义；跨格跳线不做补点
            if (d_in in _DIRS and d_out in _DIRS and d_in != (0, 0)
                    and d_in != d_out
                    and _turn_units(_DIRS.index(d_in), _DIRS.index(d_out)) >= 2
                    and mine.col != _BRIDGE_COL):
                ov = CellRef(mine.col + d_in[0], mine.row + d_in[1])
                if not _cell_ok(ov) or ov.col == _BRIDGE_COL:
                    out.append(cur)
                    continue
                # 补点→原后继只接受 8 邻接几何（保持路线逐格 8 邻接约定，
                # fuzz 不变量锁死）；骑士步/2 格长弦一概拒绝（该雷退化为拐点）
                vc, vr = nxt.col - ov.col, nxt.row - ov.row
                mid_ok = (abs(vc) <= 1 and abs(vr) <= 1 and (vc, vr) != (0, 0))
                if mid_ok and _leg_ok(mine, ov) and _leg_ok(ov, nxt):
                    out.append(cur)              # 雷格（直线段中间点）
                    out.append(ov.name)          # 过心穿越点：转向推迟到这
                    continue
        out.append(cur)
    return tuple(out)
def _norm_dir(a: tuple[int, int], b: tuple[int, int]) -> tuple[int, int]:
    """a→b 位移归一化到最简整数方向，用于共线判断。"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    g = gcd(abs(dx), abs(dy)) or 1
    return (dx // g, dy // g)
