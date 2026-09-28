"""桥洞占格推理与校验测试。

依据：说明书 V2 图 7/图 8 的通行标识出现在 K4 / K9；
规则原文说两处桥洞分别「5 选 1」与「6 选 1」，而 K 列共 11 行 = 5 + 6。
"""

import pytest

from ag_core.bridge_zone import (
    DEFAULT_COL,
    DEFAULT_SPLIT,
    BridgeZones,
    check_bridge_pair,
    pick_passable,
    render_zones,
    zone_cells_map,
)
from ag_core.grid_map import FieldGrid


def test_default_zones_split_5_and_6():
    z = BridgeZones()
    assert len(z.zone_a) == 5
    assert len(z.zone_b) == 6
    assert len(z.zone_a) + len(z.zone_b) == 11        # 恰好等于行数
    assert z.zone_a[0] == "K1" and z.zone_a[-1] == "K5"
    assert z.zone_b[0] == "K6" and z.zone_b[-1] == "K11"


def test_document_examples_fall_in_expected_zones():
    """说明书里给的 K4 / K9 必须分别落在 A 段 / B 段，否则我们的推断站不住。"""
    z = BridgeZones()
    assert z.zone_of("K4") == "a"
    assert z.zone_of("K9") == "b"


def test_zone_of_outside():
    z = BridgeZones()
    assert z.zone_of("K3") == "a"
    assert z.zone_of("D4") is None
    assert z.zone_of("L4") is None
    assert z.zone_of("") is None


@pytest.mark.parametrize("cell", ["k4", " K4 ", "k4"])
def test_zone_of_is_case_and_space_tolerant(cell):
    assert BridgeZones().zone_of(cell) == "a"


def test_custom_zone_layout():
    """现场若核实桥洞不在 K 列，改两个参数即可。"""
    z = BridgeZones(col="J", split=6, rows=11)
    assert z.zone_of("J6") == "a"
    assert z.zone_of("J7") == "b"
    assert len(z.zone_a) == 6 and len(z.zone_b) == 5
    assert z.zone_of("K4") is None


# ---------------------------------------------------------------- 校验
def test_valid_pair_passes():
    r = check_bridge_pair("K4", "K9", grid=FieldGrid())
    assert r.ok is True
    assert r.zone_a == "a" and r.zone_b == "b"
    assert r.level == "ok"


def test_swapped_pair_still_ok():
    """无人机把两处桥洞的顺序报反了不算错（区域自会纠正）。"""
    r = check_bridge_pair("K9", "K4")
    assert r.ok is True
    assert (r.zone_a, r.zone_b) == ("b", "a")


def test_same_zone_is_flagged():
    r = check_bridge_pair("K2", "K4")
    assert r.ok is False
    assert any("同一区域" in p for p in r.problems)


def test_out_of_zone_is_flagged():
    r = check_bridge_pair("D4", "K9")
    assert r.ok is False
    assert any("不在推断的桥洞区域内" in p for p in r.problems)


def test_same_cell_is_flagged():
    r = check_bridge_pair("K4", "K4")
    assert r.ok is False
    assert any("同一格" in p for p in r.problems)


def test_out_of_field_is_flagged():
    r = check_bridge_pair("K14", "K9", grid=FieldGrid())
    assert r.ok is False
    assert any("超出场地范围" in p or "无法解析" in p for p in r.problems)


def test_missing_one_side_is_flagged():
    r = check_bridge_pair("K4", None)
    assert r.ok is False
    assert any("缺失" in p for p in r.problems)


def test_garbage_cell_is_flagged():
    r = check_bridge_pair("???", "K9")
    assert r.ok is False


def test_strict_mode_changes_level_only():
    """默认只告警；只有现场确认了布点后才切成严格模式。"""
    loose = check_bridge_pair("D4", "K9", strict=False)
    strict = check_bridge_pair("D4", "K9", strict=True)
    assert loose.ok is False and strict.ok is False
    assert loose.level == "warn"
    assert strict.level == "reject"


def test_message_is_readable():
    r = check_bridge_pair("K4", "K9")
    assert "桥洞情报正常" in r.message()
    r2 = check_bridge_pair("D4", "K9")
    assert "D4" in r2.message()


# ---------------------------------------------------------------- 现场辅助
def test_pick_passable_returns_zone_cells():
    assert list(pick_passable(which="a"))[0] == "K1"
    assert len(list(pick_passable(which="b"))) == 6


def test_zone_cells_map():
    m = zone_cells_map()
    assert set(m) == {"a", "b"}
    assert m["a"][0] == "K1"


def test_render_zones_marks_choice():
    text = render_zones(highlight=["K4", "K9"])
    assert "K4" in text and "<== 本次可通行" in text
    assert text.count("<== 本次可通行") == 2
    assert DEFAULT_COL == "K" and DEFAULT_SPLIT == 5


def test_zones_have_no_overlap():
    z = BridgeZones()
    assert not (set(z.zone_a) & set(z.zone_b))
