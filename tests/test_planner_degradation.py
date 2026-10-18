"""规划器退化路径与"assert 不得进生产代码"的回归测试。

锁住三处已经修掉、但很容易被改回去的行为：

1. **Held-Karp 无上限**：DP 状态数是 2^n × n，本机实测 n=14 → 0.17s、
   n=16 → 0.86s，n=20 约 4 分钟且几百 MB。感知一旦把大片格误判成必经点，
   规划线程就地卡死 = 整局 0 分。现在超过 _TSP_EXACT_MAX_NODES 退化为
   多起点最近邻。退化只许改**顺序**，不许丢任务——雷一个都不能少。

2. **best_last < 0 的静默空路线**：全集无可行顺序时回溯出空 order，
   拼装结果变成"起点直奔终点、跳过全部雷与桥洞"，且没有任何报错。
   这是最坏的一种坏：车看起来在正常跑，分全丢。

3. **生产代码里的 assert**：`python -O` 会把 assert 整条删掉。原先
   `assert best_route is not None` 在 -O 下消失，None 顺着 plan_route
   流进 commands.route，现场表现为"车收到一条莫名其妙的路线"而不是报错。
"""
from __future__ import annotations

import ast
import time
from pathlib import Path

import pytest

from ag_integrate.field_frame import CellRef
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType
from ag_integrate.world_model import WorldModel
from ag_integrate.mission_router import (
    _TSP_EXACT_MAX_NODES,
    _greedy_orders,
    _held_karp_orders,
    plan_route,
)

REPO = Path(__file__).resolve().parent.parent
_INF = float("inf")


def _obj(cell: CellRef, t: ObjectType) -> IntelObject:
    from ag_integrate.field_frame import cell_to_world
    x, y = cell_to_world(cell)
    return IntelObject(t, x, y, 0.35, 0.35)


def _snapshot(objects) -> "WorldSnapshot":  # noqa: F821
    return WorldModel().update(IntelFrame(1, tuple(objects)))


# ------------------------------------------------------- 贪心候选本身的正确性


@pytest.mark.parametrize("n", [1, 2, 5, 14, 20])
def test_greedy_orders_are_full_permutations(n):
    """退化路径产出的是**完整排列**，不是截断/空列表。

    n=20 也在里面：这正是精确解会跑到分钟级的规模，退化路径必须微秒级
    且照样给出访问每一个必经点的顺序。
    """
    W = [[float(abs(i - j)) for j in range(n + 1)] for i in range(n + 1)]
    orders = _greedy_orders(W, n)
    assert len(orders) == n                     # 多起点：每个起点一条
    for o in orders:
        assert sorted(o) == list(range(n)), f"顺序 {o} 不是 0..{n-1} 的排列"


def test_greedy_orders_is_fast_at_scale():
    """n=20 的贪心必须远快于同规模精确解（后者约 4 分钟）。"""
    n = 20
    W = [[float((i * 7 + j * 3) % 11) for j in range(n + 1)] for i in range(n + 1)]
    t0 = time.monotonic()
    _greedy_orders(W, n)
    assert time.monotonic() - t0 < 0.5


# ------------------------------------------- 无可行顺序时不许退化成空路线


def test_held_karp_infeasible_falls_back_to_greedy_not_empty():
    """全 INF 代价矩阵 = 没有任何可行访问顺序（单向隧道方向标反等）。

    回归点：best_last 停在 -1，_reconstruct_order 会回溯出**空**顺序，
    拼出来就是"直奔终点、跳过全部雷与桥洞"。必须改走贪心候选并告警。
    """
    n = 4
    W = [[_INF] * (n + 1) for _ in range(n + 1)]
    dgoal = [_INF] * (n + 1)

    with pytest.warns(RuntimeWarning, match="无可行访问顺序"):
        orders = _held_karp_orders(W, dgoal, n)

    assert orders, "退化后仍然给了空候选——正是当初的静默灾难"
    assert len(orders) == n
    for o in orders:
        assert sorted(o) == list(range(n))


def test_held_karp_feasible_still_exact():
    """有可行解时不能被上面的兜底路径带偏：仍要走精确 DP。

    4 个点排成一条线，最优顺序唯一（0,1,2,3），任何贪心起点都该收敛到它。
    """
    n = 4
    W = [[0.0] * (n + 1) for _ in range(n + 1)]
    for i in range(n + 1):
        for j in range(n + 1):
            W[i][j] = float(abs(i - j))
    dgoal = [float(n - i) for i in range(n + 1)]

    orders = _held_karp_orders(W, dgoal, n)
    assert orders
    assert [0, 1, 2, 3] in orders


# ------------------------------------------------- 端到端：超限必须退化不卡死


def _many_mines(count: int) -> list[CellRef]:
    """铺 count 个雷，避开 K 列（K 列 PAPER 是桥洞箭头，不是雷）。"""
    cells = []
    for r in range(11):
        for c in range(14):
            if c == 10:
                continue
            cells.append(CellRef(c, r))
            if len(cells) == count:
                return cells
    return cells


def test_route_degrades_when_required_exceeds_exact_cap():
    """必经点超过 _TSP_EXACT_MAX_NODES：告警 + 秒级出解 + **一个雷都不许丢**。"""
    mines = _many_mines(_TSP_EXACT_MAX_NODES + 2)      # 16 个
    assert len(mines) == _TSP_EXACT_MAX_NODES + 2
    s = _snapshot([_obj(c, ObjectType.PAPER) for c in mines])

    t0 = time.monotonic()
    with pytest.warns(RuntimeWarning, match="超过精确 TSP 上限"):
        route = plan_route(s, CellRef(13, 9), CellRef(13, 4))   # N10 -> N5
    elapsed = time.monotonic() - t0

    assert elapsed < 10.0, f"退化没生效，规划耗时 {elapsed:.1f}s（精确解会到分钟级）"
    assert route[0] == "N10" and route[-1] == "N5"
    missing = [c.name for c in mines if c.name not in route]
    assert not missing, f"退化把任务丢了：{missing}（次优顺序可以接受，丢雷不行）"


def test_route_under_cap_still_uses_exact_solver():
    """上限以内不许触发退化告警——否则等于精确解白写了。"""
    mines = _many_mines(4)
    s = _snapshot([_obj(c, ObjectType.PAPER) for c in mines])
    import warnings as _w
    with _w.catch_warnings(record=True) as caught:
        _w.simplefilter("always")
        route = plan_route(s, CellRef(13, 9), CellRef(13, 4))
    degraded = [x for x in caught if "超过精确 TSP 上限" in str(x.message)]
    assert not degraded, f"n={len(mines)} 不该退化: {[str(x.message) for x in degraded]}"
    assert route[0] == "N10" and route[-1] == "N5"
    for c in mines:
        assert c.name in route


# --------------------------------------------- assert 不得出现在生产代码里


def _production_python_files():
    # tools/ml 是训练时脚本（工作站跑、不上机、不存在 -O 场景），
    # 2026-10-18 合并分支起明确豁免；src/tools/onboard 的其余部分全量扫描。
    for sub in ("src", "tools", "onboard"):
        root = REPO / sub
        if not root.is_dir():
            continue
        for p in root.rglob("*.py"):
            if "__pycache__" in p.parts:
                continue
            if p.relative_to(REPO).as_posix().startswith("tools/ml/"):
                continue
            yield p


def test_no_assert_statements_in_production_code():
    """生产代码里不许有 assert。

    `python -O` / `PYTHONOPTIMIZE=1` 会把 assert 整条删除。本仓真出过这个
    问题：`assert best_route is not None` 守在一条**可达**的失败路径上，
    -O 下守卫消失，None 一路流进 commands.route，车收到一条莫名其妙的
    路线而不是一个异常。要校验就用显式 raise。
    """
    offenders = []
    for p in _production_python_files():
        try:
            tree = ast.parse(p.read_text(encoding="utf-8"), filename=str(p))
        except SyntaxError as exc:          # 顺带当语法检查
            offenders.append(f"{p.relative_to(REPO).as_posix()}: SyntaxError {exc}")
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.Assert):
                offenders.append(
                    f"{p.relative_to(REPO).as_posix()}:{node.lineno}")
    assert not offenders, (
        "以下位置用了 assert 做运行期校验（-O 下会被删掉，改成 raise）：\n  "
        + "\n".join(offenders))
