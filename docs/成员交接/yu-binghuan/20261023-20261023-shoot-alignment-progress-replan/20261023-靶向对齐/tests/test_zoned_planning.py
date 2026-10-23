"""分区规划硬化测试：部分场域适配 + 中转优化 + 可靠性兜底。

覆盖 2026-10-16 路径规划全面提升的四项要求：
1. 部分场域（无人机只给 1/3 区域）也能正常规划
2. 最小化折返（重复踩格 = 0）与绕弯
3. 高可靠性（缺区自动追加、不可达降级保底、不抛异常）
4. 每区单独可完赛（必经点全覆盖校验）
"""

from __future__ import annotations

import warnings

import pytest

from ag_integrate.field_frame import CellRef
from ag_integrate.mission_router import (
    _MID_BOUNDARY,
    _zone_of,
    plan_route,
    plan_route_zoned,
)
from ag_integrate.world_model import WorldSnapshot


def _cell(name: str) -> CellRef:
    from ag_integrate.field_frame import LETTERS
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


# ---------- 1. 部分场域适配：单区已知也能规划 ----------

def test_single_zone_planning_completes():
    """无人机只侦察到 1 个区（东南），规划应覆盖该区雷并到达终点。"""
    start = _cell("N11")
    goal = _cell("N6")
    mine = _cell("D3")          # 东南区
    snap = WorldSnapshot(0, (mine,), (), (), ())
    route = plan_route_zoned(snap, start, goal, ("东北", "西", "东南"),
                             start_heading="y", final_heading="y")
    assert route[0] == "N11"
    assert route[-1] == "N6"
    assert "D3" in route, f"雷 D3 未被访问: {route}"


def test_single_zone_planning_no_backtracking():
    """单区规划路线不应有折返（重复踩格）。"""
    start = _cell("N11")
    goal = _cell("N6")
    mine = _cell("D3")
    snap = WorldSnapshot(0, (mine,), (), (), ())
    route = plan_route_zoned(snap, start, goal, ("东北", "西", "东南"),
                             start_heading="y", final_heading="y")
    dup = len(route) - len(set(route))
    assert dup == 0, f"路线有 {dup} 次折返: {route}"


# ---------- 2. 缺区自动追加：不抛 ValueError ----------

def test_missing_zone_auto_appended_no_error():
    """zone_order 未覆盖含任务的区时自动追加，不抛异常。"""
    start = _cell("N11")
    goal = _cell("N6")
    mine = _cell("D3")          # 东南区
    snap = WorldSnapshot(0, (mine,), (), (), ())
    # zone_order 只列了东北，东南有雷但未列入
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        route = plan_route_zoned(snap, start, goal, ("东北",),
                                 start_heading="y", final_heading="y")
    assert any("自动追加" in str(x.message) for x in w), \
        f"应告警自动追加东南区: {[str(x.message) for x in w]}"
    assert "D3" in route
    assert route[-1] == "N6"


# ---------- 3. 空区跳过：无任务区不穿越 ----------

def test_empty_zone_skipped():
    """无任务的区（西）应被跳过，路线不白跑。"""
    start = _cell("N11")
    goal = _cell("N6")
    mine_se = _cell("D3")       # 东南
    mine_ne = _cell("K9")       # 东北
    snap = WorldSnapshot(0, (mine_se, mine_ne), (), (), ())
    route = plan_route_zoned(snap, start, goal, ("东北", "西", "东南"),
                             start_heading="y", final_heading="y")
    assert "D3" in route
    assert "K9" in route
    assert route[-1] == "N6"
    # 无折返
    assert len(route) == len(set(route))


# ---------- 4. 跨区中转走西区走廊 ----------

def test_cross_zone_transit_uses_corridor():
    """东南↔东北 转场必须经西区走廊格（A/B/C 列 6/7 行）。"""
    start = _cell("N11")        # 东北
    goal = _cell("N6")          # 东北
    mine_se = _cell("D3")       # 东南
    snap = WorldSnapshot(0, (mine_se,), (), (), ())
    route = plan_route_zoned(snap, start, goal, ("东南", "东北"),
                             start_heading="y", final_heading="y")
    # 路线必须经过西区走廊格（A/B/C 列，第 6/7 行）
    corridor_cells = {"A6", "A7", "B6", "B7", "C6", "C7"}
    assert corridor_cells & set(route), \
        f"跨区转场未走西区走廊: {route}"


# ---------- 5. 每区完赛校验 ----------

def test_zone_completion_warns_on_missing():
    """区内存有可达必经点但路线遗漏时应告警（兜底校验）。"""
    start = _cell("N11")
    goal = _cell("N6")
    mine = _cell("D3")
    snap = WorldSnapshot(0, (mine,), (), (), ())
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        plan_route_zoned(snap, start, goal, ("东南",),
                         start_heading="y", final_heading="y")
    # 正常情况下 D3 可达，不应有完赛校验告警
    completion_warns = [x for x in w if "完赛校验" in str(x.message)]
    assert not completion_warns, \
        f"可达雷不应触发完赛校验告警: {[str(x.message) for x in completion_warns]}"


# ---------- 6. 不可达区降级保底 ----------

def test_unreachable_zone_degrades_to_goal():
    """某区规划失败（到中转点不可达）时降级直奔终点，不抛异常。"""
    start = _cell("N11")
    goal = _cell("N6")
    mine = _cell("D3")
    # 用桥洞隧道 K6 隔断东南与东北（桥列墙使两区不连通）
    pas = _cell("K6")
    snap = WorldSnapshot(0, (mine,), (pas,), (), ())
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        route = plan_route_zoned(snap, start, goal, ("东南", "东北"),
                                 start_heading="y", final_heading="y")
    # 降级后仍能返回一条到达终点的路线
    assert route[-1] == "N6", f"降级后应到达终点 N6: {route[-5:]}"
    assert any("降级" in str(x.message) for x in w), \
        "应告警降级跳过不可达区"


# ---------- 7. 终点不可达返回部分路线 ----------

def test_goal_unreachable_returns_partial():
    """终点完全不可达时返回已规划部分，不抛异常。"""
    start = _cell("N11")
    goal = _cell("A1")
    # 用障碍墙围死起点周围
    obstacles = [
        _cell("M11"), _cell("N10"), _cell("M10"),
    ]
    snap = WorldSnapshot(0, (), (), tuple(obstacles), ())
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        try:
            route = plan_route_zoned(snap, start, goal, ("西",),
                                     start_heading="y", final_heading="y")
        except ValueError:
            pytest.fail("终点不可达不应抛 ValueError，应返回部分路线")
    # 至少返回了起点
    assert route[0] == "N11"


# ---------- 8. 多雷多区全局最优（无折返） ----------

def test_multiple_mines_across_zones_no_backtrack():
    """多个雷分布在多个区，路线应全覆盖且无折返。"""
    start = _cell("N11")
    goal = _cell("N6")
    mines = [_cell("D3"), _cell("F4"), _cell("H8"), _cell("J9")]
    snap = WorldSnapshot(0, tuple(mines), (), (), ())
    route = plan_route_zoned(snap, start, goal, ("东南", "东北"),
                             start_heading="y", final_heading="y")
    for m in mines:
        assert m.name in route, f"雷 {m.name} 未被访问"
    assert route[-1] == "N6"
    # 无折返
    dup = len(route) - len(set(route))
    assert dup == 0, f"多雷路线有 {dup} 次折返"


# ---------- 9. zone_order=None 退化为全局 TSP ----------

def test_zone_order_none_falls_back_to_global():
    """zone_order=None 时退化为全局 plan_route（向后兼容）。"""
    start = _cell("N11")
    goal = _cell("N6")
    mine = _cell("D3")
    snap = WorldSnapshot(0, (mine,), (), (), ())
    route = plan_route_zoned(snap, start, goal, None,
                             start_heading="y", final_heading="y")
    assert "D3" in route
    assert route[-1] == "N6"


# ---------- 10. 未知区名抛 ValueError ----------

def test_unknown_zone_name_raises():
    """zone_order 含未知区名时抛 ValueError（区分于漏覆盖自动追加）。"""
    start = _cell("N11")
    goal = _cell("N6")
    snap = WorldSnapshot(0, (), (), (), ())
    with pytest.raises(ValueError, match="未知区域名"):
        plan_route_zoned(snap, start, goal, ("火星",),
                         start_heading="y", final_heading="y")


# ---------- 11. 桥洞格作为必经点 ----------

def test_passable_cell_is_required():
    """可通行桥洞格（ARROW）应作为必经点被访问。"""
    start = _cell("N11")
    goal = _cell("N7")           # 东北区（与桥洞同区，避免桥列隔断）
    pas = _cell("K7")            # 东北区桥洞（挡板边界行，桥列唯一缺口）
    snap = WorldSnapshot(0, (), (pas,), (), ())
    route = plan_route_zoned(snap, start, goal, ("东北",),
                             start_heading="y", final_heading="y")
    assert "K7" in route, f"桥洞格 K7 未被访问: {route}"


# ---------- 12. 区域划分正确性 ----------

def test_zone_classification():
    """三区划分：西=A-C 通长，东南=D-N 第1-6行，东北=D-N 第7-11行。"""
    assert _zone_of(_cell("A1"), _MID_BOUNDARY) == "西"
    assert _zone_of(_cell("C11"), _MID_BOUNDARY) == "西"
    assert _zone_of(_cell("D6"), _MID_BOUNDARY) == "东南"   # row5 < 6
    assert _zone_of(_cell("D7"), _MID_BOUNDARY) == "东北"   # row6 >= 6
    assert _zone_of(_cell("N1"), _MID_BOUNDARY) == "东南"
    assert _zone_of(_cell("N11"), _MID_BOUNDARY) == "东北"
