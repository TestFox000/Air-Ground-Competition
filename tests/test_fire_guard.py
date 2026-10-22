# -*- coding: utf-8 -*-
"""fire_guard 禁火闸门独立回归（2026-10-22，docs/标靶打击优化方案-20261020.md §五）。

此前禁火逻辑的覆盖混在 router/robustness 系测试里，动 fire_guard 前无独立
锚点（10-20 分析挂的风险项）。本文件把闸门语义单独立住：

  1. 固定靶：正交邻蓝 → 允许；邻红 → 拒；蓝红夹缝 → 拒（宁可不打）；
     无靶 → 拒；对角不算邻。
  2. 移动靶（全场仅一个）：蓝面可见 → 轨道端点蓝面侧邻格允许；
     红面可见 → 一律禁火（不论停轨道何处）。
  3. 发现态（红蓝未辨 is_blue=None）→ 一律禁火（fail-safe）。

fire_guard 是"误击友方 −5/次"的最后一道离线闸门，语义变更必须先红这里。
"""
from __future__ import annotations

from ag_integrate.field_frame import LETTERS, CellRef
from ag_integrate.fire_guard import (
    blue_targets_hit,
    decide_fire,
    neighbor_names,
)
from ag_integrate.world_model import TargetRef, WorldSnapshot


def cell(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


def snap(*targets: TargetRef) -> WorldSnapshot:
    return WorldSnapshot(timestamp_ms=0, mines=(), passable=(),
                         obstacles=(), targets=tuple(targets))


TRACK = (cell("D7"), cell("E7"))   # 裁判地图轨道（水平，两端同行）


# ---------------- 固定靶 ----------------

def test_fixed_blue_adjacent_allows():
    s = snap(TargetRef(cell("D7"), True))
    d = decide_fire(s, "D8")
    assert d.allowed and d.reason == "enemy_blue_adjacent"


def test_fixed_red_adjacent_denies():
    s = snap(TargetRef(cell("D7"), False))
    d = decide_fire(s, "D8")
    assert d.denied and d.reason == "friendly_red_adjacent"


def test_mixed_blue_red_gap_denies():
    # 夹缝：停靠格同时正交邻到蓝靶与红靶——宁可不打也不能冒误伤友方的险
    s = snap(TargetRef(cell("C5"), True), TargetRef(cell("E5"), False))
    d = decide_fire(s, "D5")
    assert d.denied and d.reason == "mixed_targets_risk"


def test_no_target_adjacent_denies():
    d = decide_fire(snap(), "D5")
    assert d.denied and d.reason == "no_target_adjacent"


def test_diagonal_target_not_adjacent():
    # 对角格不算正交邻：蓝靶在 D7，停 E8 打不到
    s = snap(TargetRef(cell("D7"), True))
    assert blue_targets_hit(s, "E8") == ()
    assert decide_fire(s, "E8").denied


def test_unclassified_target_denies():
    # 发现态（红蓝未辨）：fail-safe 禁火
    s = snap(TargetRef(cell("D7"), None))
    assert decide_fire(s, "D8").denied


def test_blue_targets_hit_returns_fixed_target():
    t = TargetRef(cell("D7"), True)
    assert blue_targets_hit(snap(t), "D6") == (t,)


# ---------------- 移动靶（轨道几何判定） ----------------

def test_moving_blue_face_s_allows_endpoint_side_cells():
    # 蓝面朝南（S=行+1）：轨道端点 D7/E7 南侧的 D8/E8 是合法开火格
    s = snap(TargetRef(cell("D7"), True, track=TRACK, blue_face="S"))
    for shoot in ("D8", "E8"):
        d = decide_fire(s, shoot)
        assert d.allowed, f"{shoot} 应允许（蓝面侧端点邻格）: {d.reason}"
    # 北侧（红面朝向）不是合法开火格
    assert decide_fire(s, "D6").denied


def test_moving_red_face_visible_denies_everywhere():
    # 红面可见（友方）：一律禁火——不论停在轨道何处（含蓝面侧端点邻格）
    s = snap(TargetRef(cell("D7"), False, track=TRACK, blue_face="S"))
    for shoot in ("D8", "E8", "D6", "E6", "D7"):
        assert decide_fire(s, shoot).denied, f"{shoot} 不应放行"


def test_moving_unclassified_track_denies():
    # 发现态移动靶：fail-safe 禁火
    s = snap(TargetRef(cell("D7"), None, track=TRACK, blue_face="S"))
    assert decide_fire(s, "D8").denied


# ---------------- 邻格工具 ----------------

def test_neighbor_names_boundary():
    # A1 在场角：只向右/向下有邻格，不越界（顺序是实现细节，按集合断言）
    assert set(neighbor_names("A1")) == {"A2", "B1"}
    assert len(neighbor_names("G6")) == 4
