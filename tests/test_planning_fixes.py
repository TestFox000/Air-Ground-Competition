# -*- coding: utf-8 -*-
"""规划器 2026-10-17 三项修复的回归测试：

A. 桥门被实物占用 → 门失效 + 告警（旧逻辑静默豁免，路线从桶上开过去）
B. 终点不可达 → 报错附连通性诊断（封锁点分类 + 开放桥门，而非裸字符串）
C. compact_and_adapt 原子 API → 射击停靠格不可能被压缩掉

场景全部取自 2026-10-17 实拍场地复盘：
- K9 门区放桶（照片格位 ±1）——旧逻辑无视桶直接穿门
- 仅 K4 单门时东南区被封死——旧报错 "goal cell unreachable from start" 无从排查
- 压缩丢 D9 射击格——adapt 拿到的压缩路线里没有停靠格，'*' 丢失
"""

from __future__ import annotations

import warnings

import pytest

from ag_integrate.field_frame import CellRef
from ag_integrate.world_model import TargetRef, WorldSnapshot
from ag_integrate.mission_router import plan_route
from ag_integrate.car_adapter import adapt, compact_and_adapt, mission_keep

LETTERS = "ABCDEFGHIJKLMN"


def cell(n: str) -> CellRef:
    return CellRef(LETTERS.index(n[0]), int(n[1:]) - 1)


def snap(mines=(), passable=("K4", "K9"), obstacles=(), targets=()):
    return WorldSnapshot(0, tuple(cell(c) for c in mines),
                         tuple(cell(c) for c in passable),
                         tuple(cell(c) for c in obstacles),
                         tuple(targets))


MOVING = (TargetRef(cell("D8"), True, track=(cell("D8"), cell("E8")), blue_face="S"),)


# ---------------------------------------------------------------- 修复 A
def test_gate_blocked_by_obstacle_disables_gate():
    """K9 门被桶占 + 起点在东南区：该区物理封死（K9 是南区唯一桥门）——
    旧逻辑会无视桶直接穿门"成功"；修复后必须报错且诊断点名 K9 障碍。"""
    s = snap(mines=("E3", "I2"), obstacles=("K9",))
    with pytest.warns(RuntimeWarning, match="K9"):
        with pytest.raises(ValueError) as ei:
            plan_route(s, cell("N11"), cell("N6"), start_heading="y", final_heading="y")
    msg = str(ei.value)
    assert "K9" in msg and "障碍" in msg, "诊断必须点名被占门格"


def test_gate_blocked_routes_via_other_gate_from_reachable_side():
    """K9 被占但起点在西区：改走 K4 完赛，K9 绝不出现在路线中。"""
    s = snap(mines=("E3",), obstacles=("K9",))
    with pytest.warns(RuntimeWarning, match="K9"):
        route = plan_route(s, cell("A1"), cell("N6"))
    assert "K9" not in route, "被桶占用的门格不得出现在路线中"
    assert "K4" in route, "另一扇门必须被穿越"


def test_gate_blocked_by_target_disables_gate():
    s = snap(passable=("K4", "K9"), targets=(TargetRef(cell("K9"), True),))
    with pytest.warns(RuntimeWarning, match="K9"):
        with pytest.raises(ValueError) as ei:
            plan_route(s, cell("N11"), cell("N6"))
    assert "K9" in str(ei.value)


def test_all_gates_blocked_bridge_becomes_wall():
    """两扇门都被占：K 列整列成墙 → 东南区封死 → 报错且诊断为桥门全关。"""
    s = snap(passable=("K4", "K9"), obstacles=("K4", "K9"))
    with pytest.warns(RuntimeWarning), pytest.raises(ValueError) as ei:
        plan_route(s, cell("N11"), cell("N6"))
    assert "桥门全部关闭" in str(ei.value)


def test_unblocked_gate_unaffected():
    """无占用时行为不变：双门可走、无告警。"""
    s = snap(mines=("E3",))
    with warnings.catch_warnings():
        warnings.simplefilter("error")          # 任何告警都会让测试失败
        route = plan_route(s, cell("N11"), cell("N6"))
    assert route


# ---------------------------------------------------------------- 修复 B
def test_unreachable_diagnostic_names_blockers():
    """东南区被封（仅 K4 门）：报错必须点名封锁类别与开放门。"""
    s = snap(passable=("K4",))                 # K9 未开放
    with pytest.raises(ValueError) as ei:
        plan_route(s, cell("N11"), cell("N6"))
    msg = str(ei.value)
    assert "不可达" in msg
    assert "桥墙" in msg, "应指出桥体墙格封锁"
    assert "K4" in msg, "应列出唯一开放门"
    assert "排查" in msg, "应给出排查建议"


def test_unreachable_diagnostic_names_obstacles():
    """障碍封死通道时按障碍类别点名。"""
    # 用障碍把 N 列终点围死
    s = snap(passable=("K4", "K9"), obstacles=("M5", "M6", "M7", "N5", "N7"))
    with pytest.raises(ValueError) as ei:
        plan_route(s, cell("N11"), cell("N6"))
    assert "障碍" in str(ei.value)
    assert "M5" in str(ei.value) or "N5" in str(ei.value)


# ---------------------------------------------------------------- 修复 C
def test_compact_and_adapt_keeps_shoot_cells():
    """压缩不得丢射击停靠格：D9 在完整路线中共线，naive 压缩会删掉它。"""
    s = snap(mines=("E3", "E2"), targets=MOVING)
    route = plan_route(s, cell("N11"), cell("N6"), start_heading="y", final_heading="y")
    compact, cmds = compact_and_adapt(s, route, "N11", "y", "y")
    assert cmds.shoot_cells, "射击格不得为空"
    assert all(c in compact for c in cmds.shoot_cells), "射击格必须在压缩路线中"
    assert "*" in cmds.route, "车端命令必须带 '*' 开火标记"


def test_mission_keep_covers_all_mission_cells():
    s = snap(mines=("E3",), passable=("K4", "K9"), targets=MOVING)
    keep = mission_keep(s, ("D9", "E9", "N11"))
    assert {"E3", "K4", "K9", "D8", "E8", "D9", "E9"} <= keep


def test_naive_adapt_on_overcompressed_route_was_the_bug():
    """固化旧陷阱作为对照：把 D9/E9 从压缩路线剔除后 adapt 静默丢射击——
    这正是 compact_and_adapt 要消灭的 bug 模式。"""
    s = snap(mines=("E3", "E2"), targets=MOVING)
    route = plan_route(s, cell("N11"), cell("N6"), start_heading="y", final_heading="y")
    compact, cmds = compact_and_adapt(s, route, "N11", "y", "y")
    broken = tuple(c for c in compact if c not in ("D9", "E9"))
    cmds_broken = adapt(s, broken, "N11", "y", "y")
    assert not cmds_broken.shoot_cells, "前提校验：旧用法确实会丢射击格"
    assert cmds.shoot_cells, "新 API 不丢"
