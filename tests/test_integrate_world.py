"""ag_integrate field_frame / world_model 测试。"""

import pytest

from ag_integrate.field_frame import CellRef, cell_to_world, world_to_cell
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType, build, parse
from ag_integrate.world_model import WorldModel


@pytest.mark.parametrize(
    "cell",
    [CellRef(0, 0), CellRef(13, 10), CellRef(10, 5), CellRef(3, 0)],
)
def test_world_cell_roundtrip(cell):
    x, y = cell_to_world(cell)
    assert world_to_cell(x, y) == cell


def test_world_to_cell_rejects_outliers():
    # 收紧吸附容差：偏离格心 0.4 个格距应被拒
    x = 0.065 + 0.35 * 2.4
    with pytest.raises(ValueError):
        world_to_cell(x, 0.295, max_residual=0.3)
    with pytest.raises(ValueError):
        world_to_cell(10.0, 10.0)


def obj_at(cell: CellRef, obj_type: ObjectType) -> IntelObject:
    x, y = cell_to_world(cell)
    return IntelObject(obj_type, x, y, 0.35, 0.35)


def test_world_model_classifies_and_dedups():
    m1, m2 = CellRef(3, 10), CellRef(1, 6)     # D11, B7
    a1 = CellRef(10, 8)                         # K9
    o1 = CellRef(5, 5)                          # F6
    blue, red = CellRef(3, 1), CellRef(4, 1)    # D2, E2

    frame = IntelFrame(
        1234,
        (
            obj_at(m1, ObjectType.MINE),
            obj_at(m1, ObjectType.MINE),        # 重复，去重
            obj_at(m2, ObjectType.MINE),
            obj_at(a1, ObjectType.ARROW),
            obj_at(o1, ObjectType.OBSTACLE),
            obj_at(blue, ObjectType.TARGET_BLUE),
            obj_at(red, ObjectType.TARGET_RED),
            obj_at(CellRef(7, 7), ObjectType.CAR),
            obj_at(CellRef(7, 8), ObjectType.CELL_LABEL),
        ),
    )

    model = WorldModel()
    assert not model.ready
    snap = model.update(frame)
    assert model.ready
    assert snap.timestamp_ms == 1234
    assert {c.name for c in snap.mines} == {"D11", "B7"}
    assert {c.name for c in snap.passable} == {"K9"}
    assert {c.name for c in snap.obstacles} == {"F6"}
    assert {t.cell.name for t in snap.targets} == {"D2", "E2"}
    assert snap.has_enemy
    assert {(t.cell.name, t.is_blue) for t in snap.targets} == {("D2", True), ("E2", False)}


def test_pipeline_from_bytes():
    """协议字节 -> 解析 -> 世界模型 端到端。"""
    cell = CellRef(0, 0)
    raw = build(IntelFrame(99, (obj_at(cell, ObjectType.MINE),)))
    model = WorldModel()
    snap = model.update(parse(raw))
    assert snap.mines[0].name == "A1"
