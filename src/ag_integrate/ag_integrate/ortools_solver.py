"""联合求解器：任务排序 + 预算取舍一步到位（全线升级方案 U1）。

现行管线是两步启发式（Held-Karp 全任务排序 → 超软预算整砍射击位），
本模块把它升级为**联合最优**：哪些任务做、哪些任务舍、按什么顺序做，
在"总耗时 ≤ 预算"约束下最大化得分（同分比用时 → 得分字典序优先、耗时最小次之）。

双后端（同输入同输出语义，均为精确解）：
  · "dp"（默认血线安全路径）：纯 Python 子集枚举 + Held-Karp DP，
    零原生依赖、任何机器可跑——本任务规模（≤14 节点）毫秒级出精确解；
  · "cp-sat"：OR-Tools CP-SAT AddCircuit 建模。⚠️ 实测部分 Windows 机器上
    ortools 原生库导入成功但求解段错误（进程级崩溃无法 try/except 捕获），
    故启用前必须通过 `cp_sat_healthy()` 的**子进程健康探测**（崩溃只死探针，
    不死主进程）。backend="auto" = 探测通过用 CP-SAT，否则 DP。

设计要点（对齐 docs/全线升级方案-20261018.md U1 与血线哲学）：
  · drop_policy="shooting_only"（默认）：只有射击位可舍——保持 pipeline 现行
    "少排雷是人的决策，不是规划器的"哲学，只是把"整砍"细化成"按需砍"；
  · drop_policy="any_task"：雷也可舍（V3 升级口径），远程低效雷让位近程任务；
  · 降级契约：求解不可用/不可行 → 相应 status 返回，order/dropped 为空、
    est=-1，调用方退回现行启发式——与"全程无人工干预"红线一致。

用法：
    from ag_integrate.ortools_solver import VisitTask, solve_visit_plan
    plan = solve_visit_plan(start, goal, tasks, leg_s=..., time_budget_s=165.0)
    plan.order / plan.dropped / plan.est_seconds / plan.est_score / plan.status
"""

from __future__ import annotations

import subprocess
import sys
from dataclasses import dataclass
from typing import Callable

# CP-SAT 整数缩放（时间/得分统一 ×100，精度 0.01）；DP 后端直接用浮点
_SCALE = 100
# CP-SAT 词典序权重：得分严格优先于耗时
_BIG = 10 ** 9
# DP 后端节点上限（2^n × n 状态；14 与 mission_router._TSP_EXACT_MAX_NODES 对齐）
_DP_MAX_NODES = 14

STATUS_OPTIMAL = "optimal"
STATUS_FEASIBLE = "feasible"
STATUS_NO_OR_TOOLS = "no_or_tools"
STATUS_INFEASIBLE = "infeasible"
STATUS_TOO_MANY_NODES = "too_many_nodes"

_INF = float("inf")


@dataclass(frozen=True)
class VisitTask:
    cell: str          # 格号，如 "C3"
    kind: str          # "mine" | "shoot"
    score: float       # 得分贡献（雷=5；射击位=命中期望分）
    service_s: float   # 停靠/过雷减速附加秒（进入该节点即计入）


@dataclass(frozen=True)
class VisitPlan:
    order: tuple[str, ...]     # 保留任务的格号，按访问顺序（start/goal 不含）
    dropped: tuple[str, ...]   # 舍弃任务格号
    est_seconds: float         # 总耗时（含 service；求解器口径）；-1 = 未求解
    est_score: float           # 保留任务总得分
    status: str                # optimal / feasible / no_or_tools / infeasible / too_many_nodes
    backend: str = ""          # dp / cp-sat（实际使用的后端）


# ---------------------------------------------------------------- 健康探测
_healthy: bool | None = None


def cp_sat_healthy(*, force_recheck: bool = False) -> bool:
    """OR-Tools CP-SAT 是否可安全使用（子进程探测，结果进程内缓存）。

    探针跑一个平凡模型：导入失败 / 求解失败 / **原生段错误**（退出码
    0xC0000005 等）都判不健康——崩溃只发生在子进程里，主进程不受影响。
    """
    global _healthy
    if _healthy is None or force_recheck:
        code = (
            "import sys\n"
            "from ortools.sat.python import cp_model\n"
            "m = cp_model.CpModel(); x = m.NewBoolVar('x'); m.Maximize(x)\n"
            "s = cp_model.CpSolver()\n"
            "sys.exit(0 if s.Solve(m) in (cp_model.OPTIMAL, cp_model.FEASIBLE) else 3)\n"
        )
        try:
            r = subprocess.run([sys.executable, "-c", code],
                               capture_output=True, timeout=60)
            _healthy = r.returncode == 0
        except Exception:
            _healthy = False
    return _healthy


# ---------------------------------------------------------------- 主入口
def solve_visit_plan(
    start_cell: str,
    goal_cell: str,
    tasks: tuple[VisitTask, ...] | list[VisitTask],
    *,
    leg_s: Callable[[str, str], float],
    time_budget_s: float,
    drop_policy: str = "shooting_only",
    backend: str = "auto",
    solver_time_limit_s: float = 2.0,
) -> VisitPlan:
    """在 time_budget_s 内最大化得分的访问序 + 舍弃集（精确解）。

    leg_s(a, b)：a→b 的纯旅行秒（不含 service）；不可达返回 float("inf")。
    backend：auto（探测通过用 cp-sat，否则 dp）/ dp / cp-sat。
    """
    tasks = tuple(tasks)
    if drop_policy not in ("shooting_only", "any_task"):
        raise ValueError(f"drop_policy 非法：{drop_policy!r}")
    if backend not in ("auto", "dp", "cp-sat"):
        raise ValueError(f"backend 非法：{backend!r}")

    if backend == "cp-sat" or (backend == "auto" and cp_sat_healthy()):
        plan = _solve_cp_sat(start_cell, goal_cell, tasks, leg_s=leg_s,
                             time_budget_s=time_budget_s, drop_policy=drop_policy,
                             solver_time_limit_s=solver_time_limit_s)
        if plan.status != STATUS_NO_OR_TOOLS:
            return plan if plan.backend else _replace_backend(plan, "cp-sat")
        # cp-sat 不可用（未安装）：auto 落到 dp；显式 cp-sat 则如实返回
        if backend == "cp-sat":
            return plan
    return _solve_dp(start_cell, goal_cell, tasks, leg_s=leg_s,
                     time_budget_s=time_budget_s, drop_policy=drop_policy)


def _replace_backend(plan: VisitPlan, backend: str) -> VisitPlan:
    from dataclasses import replace
    return replace(plan, backend=backend)


# ---------------------------------------------------------------- DP 后端
def _solve_dp(
    start_cell: str,
    goal_cell: str,
    tasks: tuple[VisitTask, ...],
    *,
    leg_s: Callable[[str, str], float],
    time_budget_s: float,
    drop_policy: str,
) -> VisitPlan:
    """子集枚举 + Held-Karp：对每个"保留子集"求最优访问时序，再按
    （得分最大, 耗时最小）选子集——与 CP-SAT 词典序目标完全同语义。"""
    n = len(tasks)
    if n == 0:
        direct = leg_s(start_cell, goal_cell)
        if direct == _INF or direct > time_budget_s:
            return VisitPlan((), (), -1.0, 0.0, STATUS_INFEASIBLE, "dp")
        return VisitPlan((), (), direct, 0.0, STATUS_OPTIMAL, "dp")
    if n > _DP_MAX_NODES:
        return VisitPlan((), (), -1.0, 0.0, STATUS_TOO_MANY_NODES, "dp")

    # 必经任务（shooting_only 下雷不可舍）
    mandatory = frozenset(
        k for k, t in enumerate(tasks) if drop_policy == "shooting_only" and t.kind == "mine"
    )

    # 预计算 legs（含终点）
    legs = [[leg_s(a.cell, b.cell) for b in tasks] for a in tasks]
    leg_start = [leg_s(start_cell, t.cell) for t in tasks]
    leg_goal = [leg_s(t.cell, goal_cell) for t in tasks]

    full = 1 << n
    # dp[mask][last] = 从 start 顺序访问 mask（含 last）的最小总耗时（含 service）
    dp = [[_INF] * n for _ in range(full)]
    parent = [[-1] * n for _ in range(full)]
    for k in range(n):
        if leg_start[k] != _INF:
            dp[1 << k][k] = leg_start[k] + tasks[k].service_s
    for mask in range(full):
        row = dp[mask]
        for i in range(n):
            t_i = row[i]
            if t_i == _INF or not (mask >> i) & 1:
                continue
            for j in range(n):
                if (mask >> j) & 1:
                    continue
                lj = legs[i][j]
                if lj == _INF:
                    continue
                nm = mask | (1 << j)
                cand = t_i + lj + tasks[j].service_s
                if cand < dp[nm][j]:
                    dp[nm][j] = cand
                    parent[nm][j] = i

    # 逐子集结算：完成时间 = dp[mask][last] + leg(last, goal)
    best: tuple[float, float, int, int] | None = None  # (score, -time, mask, last)
    for mask in range(full):
        if not mandatory <= frozenset(
            k for k in range(n) if (mask >> k) & 1
        ):
            continue
        score = sum(tasks[k].score for k in range(n) if (mask >> k) & 1)
        best_total = _INF
        best_last = -1
        for last in range(n):
            if dp[mask][last] == _INF or leg_goal[last] == _INF:
                continue
            total = dp[mask][last] + leg_goal[last]
            if total < best_total:
                best_total, best_last = total, last
        if mask == 0:
            direct = leg_s(start_cell, goal_cell)
            best_total, best_last = (direct, -1) if direct != _INF else (_INF, -1)
        if best_total == _INF or best_total > time_budget_s:
            continue
        key = (score, -best_total)
        if best is None or key > (best[0], -best[1]):
            best = (score, best_total, mask, best_last)

    if best is None:
        return VisitPlan((), (), -1.0, 0.0, STATUS_INFEASIBLE, "dp")
    score, total, mask, last = best
    # 回溯顺序
    order: list[int] = []
    cur_mask, cur_last = mask, last
    while cur_last != -1:
        order.append(cur_last)
        p = parent[cur_mask][cur_last]
        cur_mask ^= 1 << cur_last
        cur_last = p
    order.reverse()
    dropped = tuple(tasks[k].cell for k in range(n) if not (mask >> k) & 1)
    return VisitPlan(
        order=tuple(tasks[k].cell for k in order),
        dropped=dropped,
        est_seconds=total,
        est_score=score,
        status=STATUS_OPTIMAL,
        backend="dp",
    )


# ---------------------------------------------------------------- CP-SAT 后端
def _solve_cp_sat(
    start_cell: str,
    goal_cell: str,
    tasks: tuple[VisitTask, ...],
    *,
    leg_s: Callable[[str, str], float],
    time_budget_s: float,
    drop_policy: str,
    solver_time_limit_s: float,
) -> VisitPlan:
    n = len(tasks)
    if n == 0:
        direct = leg_s(start_cell, goal_cell)
        if direct == _INF or direct > time_budget_s:
            return VisitPlan((), (), -1.0, 0.0, STATUS_INFEASIBLE, "cp-sat")
        return VisitPlan((), (), direct, 0.0, STATUS_OPTIMAL, "cp-sat")

    try:
        from ortools.sat.python import cp_model
    except ImportError:
        return VisitPlan((), (), -1.0, 0.0, STATUS_NO_OR_TOOLS, "")

    # 节点编号：0=start（depot），1..n=任务，n+1=goal
    START, GOAL = 0, n + 1
    names = [start_cell] + [t.cell for t in tasks] + [goal_cell]

    # 弧成本（含 service(j)；goal 的 service=0）；不可达弧不建 = 禁用。
    # goal→start 闭合弧零成本：AddCircuit 要一个环，路径 start→…→goal 靠它闭合。
    leg_cost: dict[tuple[int, int], int] = {}
    for i in range(n + 2):
        for j in range(n + 2):
            if i == GOAL and j == START:
                leg_cost[(i, j)] = 0          # 闭合弧（必须先于 j==START 的排除判断）
                continue
            if i == j:
                continue                      # 自环在 drop 段统一处理
            if j == START:
                continue                      # 不进 start
            if i == GOAL:
                continue                      # goal 只出不进（除闭合弧）
            travel = leg_s(names[i], names[j])
            if travel == _INF:
                continue
            service = tasks[j - 1].service_s if j != GOAL else 0.0
            leg_cost[(i, j)] = int(round((travel + service) * _SCALE))

    m = cp_model.CpModel()
    arc_lit = {ij: m.NewBoolVar(f"arc_{ij[0]}_{ij[1]}") for ij in leg_cost}
    drop_lit: dict[int, object] = {}
    for k, t in enumerate(tasks, start=1):
        if drop_policy == "any_task" or t.kind != "mine":
            drop_lit[k] = m.NewBoolVar(f"drop_{k}")

    circuit_arcs = [(i, j, lit) for (i, j), lit in arc_lit.items()]
    for k in range(1, n + 1):
        if k in drop_lit:
            circuit_arcs.append((k, k, drop_lit[k]))
        else:
            must_false = m.NewBoolVar(f"must_{k}")
            m.Add(must_false == 0)
            circuit_arcs.append((k, k, must_false))
    m.AddCircuit(circuit_arcs)

    time_expr = sum(lit * leg_cost[ij] for ij, lit in arc_lit.items())
    score_terms = []
    for k, t in enumerate(tasks, start=1):
        if k in drop_lit:
            kept = m.NewBoolVar(f"kept_{k}")
            m.Add(kept == 1 - drop_lit[k])
            score_terms.append(kept * int(round(t.score * _SCALE)))
        else:
            score_terms.append(int(round(t.score * _SCALE)))
    score_expr = sum(score_terms)

    m.Add(time_expr <= int(round(time_budget_s * _SCALE)))
    m.Maximize(score_expr * _BIG - time_expr)   # 得分字典序优先，耗时次之

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = solver_time_limit_s
    solver.parameters.num_search_workers = 1    # 确定性：同输入同输出
    code = solver.Solve(m)
    if code not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return VisitPlan((), (), -1.0, 0.0, STATUS_INFEASIBLE, "cp-sat")

    succ: dict[int, int] = {}
    for (i, j), lit in arc_lit.items():
        if solver.Value(lit):
            succ[i] = j
    order: list[str] = []
    cur = succ.get(START)
    while cur is not None and cur != GOAL:
        order.append(tasks[cur - 1].cell)
        cur = succ.get(cur)
    dropped = [t.cell for k, t in enumerate(tasks, start=1)
               if k in drop_lit and solver.Value(drop_lit[k])]

    return VisitPlan(
        order=tuple(order),
        dropped=tuple(dropped),
        est_seconds=solver.Value(time_expr) / _SCALE,
        est_score=solver.Value(score_expr) / _SCALE,
        status=STATUS_OPTIMAL if code == cp_model.OPTIMAL else STATUS_FEASIBLE,
        backend="cp-sat",
    )
