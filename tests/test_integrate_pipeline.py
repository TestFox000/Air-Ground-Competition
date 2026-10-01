"""pipeline：门控 + 从态势直接组装车命令。"""

from ag_integrate.field_frame import CellRef, cell_to_world
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType
from ag_integrate.world_model import WorldModel
from ag_integrate.pipeline import evaluate_gate, build_mission


def obj(cell, t):
    x, y = cell_to_world(cell)
    return IntelObject(t, x, y, 0.35, 0.35)


def snapshot(objects):
    return WorldModel().update(IntelFrame(1, tuple(objects)))


def test_gate_decisions():
    # 没有雷 -> 门不开
    s = snapshot((obj(CellRef(3, 1), ObjectType.TARGET_BLUE),))
    assert not evaluate_gate(s, min_mines=1, require_face=True).ok
    # 有雷但无敌我判决 -> face_unknown
    s = snapshot((obj(CellRef(3, 10), ObjectType.MINE),))
    assert evaluate_gate(s, 1, require_face=True).reason == "face_unknown"
    assert evaluate_gate(s, 1, require_face=False).ok
    # 雷 + 靶 -> 开门
    s = snapshot(
        (
            obj(CellRef(3, 10), ObjectType.MINE),
            obj(CellRef(3, 1), ObjectType.TARGET_BLUE),
        )
    )
    gate = evaluate_gate(s, 1, True)
    assert gate.ok


def test_build_mission_full():
    s = snapshot(
        (
            obj(CellRef(3, 10), ObjectType.MINE),       # D11
            obj(CellRef(10, 8), ObjectType.ARROW),      # K9
            obj(CellRef(5, 5), ObjectType.OBSTACLE),    # F6
            obj(CellRef(3, 1), ObjectType.TARGET_BLUE),  # D2 敌
        )
    )
    cmds = build_mission(s, "N10", "N11", "y", "y")
    assert cmds.reset == "reset:N10,y"
    tokens = cmds.route.split()
    assert tokens[0] == "N10"
    assert tokens[-1] == "N11,y"
    # 敌靶格带 *，且射击点集合恰为 D2
    assert cmds.shoot_cells == ("D2",)
    assert "D2*" in tokens
    assert "F6" not in tokens


def test_build_mission_cross_gates_from_names():
    s = snapshot(())
    # 格名字符串透传：F6 为唯一可穿越点，A1 -> A11 可达且经过 F6
    cmds = build_mission(s, "A1", "A11", "y", None, cross_gates=("F6",))
    tokens = cmds.route.split()
    assert tokens[0] == "A1"
    assert tokens[-1] == "A11"
    assert "F6" in tokens
