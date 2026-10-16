"""mission_router 鲁棒性回归：fuzz 逼出来的三个真 bug 的定点守护。

1. 终点朝向不可达 → 降级任意朝向完赛（旧代码断言崩溃 = 整局 0 分）；
2. 起/终点格对封锁免疫（这两格上的障碍报告只能是误检）；
3. 比赛配置自洽守护：goal=N6 + final_heading=y 在近乎空地图上必须出解
   （终点朝向不可达时按规则 1 降级）。
   注：本文曾把 mid_boundary=5 当"决定性证据"——2026-10-15 终裁推翻：
   挡板实体在第 6/7 行之界（车上官方地图真源 + 现场核实双证），
   归一问题见 docs/分区规划与感知约束-20261015.md 问题1。
"""

import pytest

from ag_integrate.field_frame import CellRef, LETTERS
from ag_integrate.mission_router import plan_route
from ag_integrate.world_model import WorldSnapshot


def _c(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


def _snap(mines=(), passable=(), obstacles=(), targets=()):
    return WorldSnapshot(0, tuple(mines), tuple(passable), tuple(obstacles), tuple(targets))


def test_match_config_self_consistent():
    """比赛配置（N11→N6，双朝向 y）在近乎空地图上必须出解。

    终点朝向不可达时按模块级降级策略放弃朝向、完赛优先（不许崩溃）。
    """
    route = plan_route(
        _snap(mines=(_c("E6"),)), _c("N11"), _c("N6"),
        start_heading="y", final_heading="y",
    )
    assert route[0] == "N11" and route[-1] == "N6"
    assert "E6" in route


def test_goal_heading_infeasible_degrades_not_crashes():
    """口袋地形：终点朝向到达须先踩终点——必须降级完赛，不允许断言崩溃。

    场景来自 fuzz 首案（种子 20261014 rand0）：障碍 M7 + K 列墙把 N6 以南
    围成口袋，朝北到达 N6 物理不可行。
    （2026-10-15 mb=6 几何适配：门移到 6/7 边界的 K7，L7→H7 保持东侧封锁，
    否则门格东侧引道被堵、场景从"朝向不可达"变成"整格不可达"。）
    """
    mines = ["A9", "J7", "A6", "D11", "I5", "A11", "M11", "H9"]
    obstacles = ["G7", "H7", "A4", "E6", "M7", "C10", "I9", "F10"]
    snap = _snap(
        mines=[_c(n) for n in mines],
        passable=[_c("K7")],   # mb=6 几何：门在 6/7 边界上（K5 是 mb=5 时代的位置）
        obstacles=[_c(n) for n in obstacles],
    )
    with pytest.warns(RuntimeWarning, match="降级"):
        route = plan_route(snap, _c("N11"), _c("N6"),
                           start_heading="y", final_heading="y")
    assert route[0] == "N11" and route[-1] == "N6"
    for m in mines:
        assert m in route   # 降级只是放弃终点朝向，雷一颗不能少


def test_goal_cell_obstacle_report_ignored():
    """误检把终点格标成障碍：免疫（终点是固定方框），照常出解不撂挑子。"""
    snap = _snap(obstacles=[_c("N11")])   # N11 是本次调用的终点
    route = plan_route(snap, _c("N10"), _c("N11"))
    assert route == ("N10", "N11")


def test_start_cell_obstacle_report_ignored():
    """误检把起点格标成障碍：车就站在那儿，照常规划。"""
    snap = _snap(mines=[_c("E6")], obstacles=[_c("N11")])
    route = plan_route(snap, _c("N11"), _c("N6"),
                       start_heading="y", final_heading="y")
    assert route[0] == "N11" and "E6" in route
