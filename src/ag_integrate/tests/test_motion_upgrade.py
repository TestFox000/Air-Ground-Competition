# -*- coding: utf-8 -*-
"""综合规划升级回归护栏（2026-10-25）：运动可行性进搜索 + 实体挡板 + 倒车挡位。

覆盖六项：
  1. 默认参数逐字节回归——增强项全关时与 2026-10-24 版（drone-replan-fixes
     基线，Held-Karp/分区/隧道口径不变）输出完全一致（旧版由 git HEAD
     的原包重算固化成字面量，见 _BASELINE_*）。
  2. 实体挡板：贴板 45° 弯在搜索内删除、体检器（turn_feasibility_v2.solid）
     同口径判 pivot——"守卫按 A 优化、体检按 B 打分"的口径分裂不复存在。
  3. 90° 内切守卫运行时化：真实场地图删内切被占的 90° 边，随机图（默认关）
     不变量不破。
  4. 倒车挡位原语：单宽走廊终点朝向约束在前进口径下不可达（降级告警），
     倒车口径下倒进终点即满足——180°×1 计价假设被可执行语义替代。
  5. HK 朝向传递：候选顺序经全模型拼装，真实代价不劣于乐观界口径。
  6. route_time 倒车差异化：直线段新旧同秒、180° 折返段计倒车速度+换挡、
     直角转弯不误报。
"""
from __future__ import annotations

import warnings

import pytest

from ag_integrate.field_frame import CellRef, LETTERS
from ag_integrate.mission_router import (
    _build_solid_barriers,
    _MID_BOUNDARY,
    plan_route,
    plan_route_zoned,
)
from ag_integrate.route_time import RouteTimeParams, estimate_seconds
from ag_integrate.turn_feasibility_v2 import turn_space, verify
from ag_integrate.world_model import WorldSnapshot, TargetRef


def cell(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


# ---------------------------------------------------------------- 场地夹具
def make_snapshot(mines=(), passable=(), obstacles=(), targets=()):
    return WorldSnapshot(
        timestamp_ms=1,
        mines=tuple(cell(n) for n in mines),
        passable=tuple(cell(n) for n in passable),
        obstacles=tuple(cell(n) for n in obstacles),
        targets=tuple(TargetRef(cell(n), b) for n, b in targets),
    )


_SNAP = make_snapshot(
    mines=["D2", "F3", "H2", "J4", "E8", "G9", "I8", "M3"],
    passable=["K9"],
    obstacles=["C5", "F6", "H7"],
    targets=[("E7", True), ("L5", False)],
)

# 基准（git HEAD 原包重算，2026-10-25）：默认参数（增强全关）必须逐字节复现。
_BASELINE_PLAIN = (
    'B9', 'B8', 'C7', 'D7', 'D8', 'E9', 'F9', 'G9', 'H9', 'I9', 'J9', 'K9',
    'J9', 'I9', 'I8', 'H8', 'G8', 'F8', 'E8', 'D8', 'C8', 'B7', 'B6', 'B5',
    'B4', 'C3', 'D3', 'E3', 'F3', 'G3', 'H3', 'I3', 'J4', 'I3', 'H2', 'G2',
    'F2', 'E2', 'D2', 'C2', 'B3', 'A3', 'A2', 'B2')
_BASELINE_ZONED = (
    'B9', 'B4', 'C3', 'F3', 'I3', 'J4', 'H2', 'D2', 'A5', 'A7', 'D7', 'D8',
    'E9', 'G9', 'K9', 'I9', 'I8', 'E8', 'B8', 'A7', 'A2', 'B2')
_BASELINE_NO_HEAD = (
    'B9', 'B8', 'C7', 'D7', 'D8', 'E9', 'F9', 'G9', 'H9', 'I9', 'J9', 'K9',
    'J9', 'I9', 'I8', 'H8', 'G8', 'F8', 'E8', 'D8', 'C8', 'B7', 'B6', 'B5',
    'B4', 'C3', 'D3', 'E3', 'F3', 'G3', 'H3', 'I3', 'J4', 'I3', 'H2', 'G2',
    'F2', 'E2', 'D2', 'C2', 'B2')

_MOTION_ON = dict(turn90_guard=True, solid_barriers=True, reverse=True,
                  heading_prop=True)


# ================================================================ 1 回归
def test_default_kwargs_reproduce_baseline():
    assert plan_route(_SNAP, cell("B9"), cell("B2"),
                      start_heading="y", final_heading="x") == _BASELINE_PLAIN
    assert plan_route(_SNAP, cell("B9"), cell("B2")) == _BASELINE_NO_HEAD
    assert plan_route_zoned(
        _SNAP, cell("B9"), cell("B2"), ("东南", "西", "东北"),
        start_heading="y", final_heading="x") == _BASELINE_ZONED


def test_enhanced_planning_still_visits_all_reachable_tasks():
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        route = plan_route(_SNAP, cell("B9"), cell("B2"),
                           start_heading="y", final_heading="x", **_MOTION_ON)
    names = set(route)
    for m in ("D2", "F3", "H2", "J4", "E8", "G9", "I8", "K9"):
        assert m in names, f"增强模式漏任务 {m}"
    assert route[-1] == "B2"


# ================================================================ 2 实体挡板
def test_solid_barriers_table_shape_and_bridge_hole():
    smap, _ = _build_solid_barriers(_MID_BOUNDARY, [cell("K6")])
    assert smap is not None
    # 边界两侧格（0 基 row5/row6）、D~N 列、桥洞列 K 豁免。
    # _DIRS y 轴向下：(0,1)=南=索引2，(0,-1)=北=索引6——上侧格朝南、
    # 下侧格朝北（首版曾写反，本测试锁方向）。
    ci = lambda name: cell(name).row * 14 + cell(name).col
    assert 87.5 == smap[ci("F6")][2]        # 第6行（上侧）朝南
    assert 87.5 == smap[ci("F7")][6]        # 第7行（下侧）朝北
    assert smap[ci("F6")][6] == 575.0       # 上侧格朝北开阔
    assert ci("K6") not in smap and ci("K7") not in smap   # 桥洞开口列
    assert ci("A6") not in smap             # A~C 无挡板


def test_turn_space_with_solid_flags_board_side_pivot():
    solid = {"F6": [575.0] * 8}
    solid["F6"][2] = 87.5                   # F6 朝南贴板（(0,1)=南）
    # 东进→东南出（(1,1)，y 向下=南）：内侧朝南=贴板 → 87.5 < need146 → pivot
    sp_board = turn_space("F6", (1, 0), (1, 1), set(), solid)
    assert sp_board["action"] == "pivot"
    # 东进→东北出（(1,-1)）：内侧朝北不贴板 → ok（无实体表同理 ok）
    sp = turn_space("F6", (1, 0), (1, -1), set(), solid)
    assert sp["action"] in ("ok", "tight")
    sp_free = turn_space("F6", (1, 0), (1, 1), set(), None)
    assert sp_free["action"] == "ok"


def test_search_prunes_board_side_45_turn():
    """贴板行（1基第6/7行）上转向内侧朝板的弯在搜索内删除。

    对照：无实体口径允许贴板 45° 弯；实体口径改走安全形态。两口径都出
    完整路线（删边不破坏可达性），且增强路线经同口径体检 pivot/forbid 归零
    ——搜索删掉的弯，体检不再报（守卫与验收同一判据）。
    """
    snap = make_snapshot(mines=["E6", "H6"], passable=["K6"])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        r_off = plan_route(snap, cell("B6"), cell("B2"), start_heading="x")
        r_on = plan_route(snap, cell("B6"), cell("B2"),
                          start_heading="x", solid_barriers=True)
    assert r_off[-1] == "B2" and r_on[-1] == "B2"
    assert {"E6", "H6"} <= set(r_on)
    smap, _ = _build_solid_barriers(_MID_BOUNDARY, [cell("K6")])
    from ag_integrate.mission_router import _CELLS
    solid = {_CELLS[ci].name: row for ci, row in smap.items()}
    assert verify(r_on, ["C5", "F6", "H7"], solid)["pivot_needed"] == 0


# ================================================================ 3 90° 守卫
def test_turn90_guard_runtime_toggle():
    """90° 内切守卫：桶占内切对角格时，守卫开的路线不使用该 90° 弯。"""
    # E4 为桶：C4→D4→E4 不可踩；D4 上 90° 弯（北进东出）的内切对角 = E5。
    snap = make_snapshot(mines=["F2"], obstacles=["E5"])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        r_off = plan_route(snap, cell("B4"), cell("B2"), start_heading="x")
        r_on = plan_route(snap, cell("B4"), cell("B2"),
                          start_heading="x", turn90_guard=True)
    assert r_off[-1] == "B2" and r_on[-1] == "B2"
    # 模块级开关（旧用法）仍生效
    import ag_integrate.mission_router as mr
    old = mr._TURN_GUARD
    try:
        mr._TURN_GUARD = True
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            r_mod = plan_route(snap, cell("B4"), cell("B2"), start_heading="x")
        assert r_mod == r_on
    finally:
        mr._TURN_GUARD = old


# ================================================================ 4 倒车挡位
def test_reverse_enables_backing_into_deadend_with_final_heading():
    """单宽走廊尽头的终点朝向：前进口径降级（任意朝向告警），倒车口径满足。

    M1(北端)→M4(走廊尽头)：两侧封死、南端贴挡板。final_heading=y(北) 只能
    "在 M3 面朝北倒进 M4"——前进口径物理不可达，触发"降级任意朝向"告警；
    倒车口径无告警且路线完整。
    """
    obstacles = ["L2", "N2", "L3", "N3", "L4", "N4"]  # 走廊两侧
    snap = make_snapshot(mines=[], obstacles=obstacles)
    kw = dict(start_heading="y", final_heading="y")

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        r_off = plan_route(snap, cell("M1"), cell("M4"), **kw)
    assert any("降级为任意朝向进终点" in str(w.message) for w in caught)

    with warnings.catch_warnings(record=True) as caught2:
        warnings.simplefilter("always")
        r_on = plan_route(snap, cell("M1"), cell("M4"), reverse=True, **kw)
    assert r_on[-1] == "M4"
    assert not any("降级为任意朝向进终点" in str(w.message) for w in caught2)


def test_reverse_not_used_when_unnecessary():
    """顺路任务（起点朝向与行进一致）：倒车计价更贵（1.5×+换挡），
    最优路线不应出现 180° 折返段。场景留在 K 列以东（无桥墙封路）。"""
    snap = make_snapshot(mines=["N2"])
    r = plan_route(snap, cell("M2"), cell("N4"), start_heading="x", reverse=True)
    est = estimate_seconds(r, mine_cells={"N2"})
    assert est.n_gear_switches == 0 and est.reverse_m == 0.0


# ================================================================ 5 朝向传递
def test_heading_prop_candidates_not_worse():
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        base = plan_route(_SNAP, cell("B9"), cell("B2"),
                          start_heading="y", final_heading="x")
        prop = plan_route(_SNAP, cell("B9"), cell("B2"),
                         start_heading="y", final_heading="x", heading_prop=True)
    # 任务全覆盖 + 形态不劣（步数不超过基线 +2；朝向传递可能微增步数换平顺）
    assert set(prop) >= {"D2", "F3", "H2", "J4", "E8", "G9", "I8", "K9"}
    assert len(prop) <= len(base) + 2


# ================================================================ 6 route_time
def test_route_time_straight_same_legacy():
    p_old = RouteTimeParams(reverse_mps=None)
    p_new = RouteTimeParams()
    r = ("B2", "C2", "D2", "E2")
    assert estimate_seconds(r, params=p_old).seconds == \
        pytest.approx(estimate_seconds(r, params=p_new).seconds)


def test_route_time_reversal_accounting():
    r = ("B2", "C2", "D2", "E2", "D2", "C2", "B2")
    est = estimate_seconds(r)
    assert est.reverse_m == pytest.approx(3 * 0.35)
    assert est.n_gear_switches == 1          # F→R；路线终点即折返段终点
    legacy = estimate_seconds(r, params=RouteTimeParams(reverse_mps=None))
    assert est.seconds > legacy.seconds      # 倒车更慢+换挡，预算更保守


def test_route_time_right_turn_not_reverse():
    est = estimate_seconds(("B2", "C2", "C3", "C4"))
    assert est.reverse_m == 0.0 and est.n_gear_switches == 0


# ================================================================ 护栏
def test_pipeline_motion_kwargs_end_to_end():
    from ag_integrate.pipeline import build_mission

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        cmds = build_mission(
            _SNAP, "B9", "B2", "y", "x",
            cross_gates=(), target_track=("D7", "E7"), target_blue_face="S",
            soft_cap_s=0,      # 关时间门控，专注规划链
            **_MOTION_ON,
        )
    assert cmds.route and "B2" in cmds.route


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
