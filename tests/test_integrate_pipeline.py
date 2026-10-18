"""pipeline：门控 + 从态势直接组装车命令。"""

from ag_integrate.field_frame import CellRef, cell_to_world
from ag_integrate.fire_guard import neighbor_names
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
    s = snapshot((obj(CellRef(3, 10), ObjectType.PAPER),))
    assert evaluate_gate(s, 1, require_face=True).reason == "face_unknown"
    assert evaluate_gate(s, 1, require_face=False).ok
    # 雷 + 靶 -> 开门
    s = snapshot(
        (
            obj(CellRef(3, 10), ObjectType.PAPER),
            obj(CellRef(3, 1), ObjectType.TARGET_BLUE),
        )
    )
    gate = evaluate_gate(s, 1, True)
    assert gate.ok


def test_build_mission_full():
    s = snapshot(
        (
            obj(CellRef(3, 10), ObjectType.PAPER),       # D11
            obj(CellRef(10, 8), ObjectType.PAPER),      # K9
            obj(CellRef(5, 5), ObjectType.OBSTACLE),    # F6
            obj(CellRef(3, 1), ObjectType.TARGET_BLUE),  # D2 敌
        )
    )
    cmds = build_mission(s, "N10", "N11", "y", "x")  # N11 最底行，只能东/西驶入
    # 上线的是**车端坐标架**的 token（两架差 90°，car_commands 负责翻译）：
    # 本机 'y'(向第1行) -> 车端 'x'；本机 'x'(A→N) -> 车端 '-y'。
    # 映射本身由 tests/test_heading_frame.py 从两个包的格号->坐标函数推导校验。
    assert cmds.reset == "reset:N10,x"
    tokens = cmds.route.split()
    assert tokens[0] == "N10"
    assert tokens[-1] == "N11,-y"
    # 敌靶格 D2 不可踩；车在其正交邻格停靠开火（C2/D3/E2/D1 均为合法邻格，
    # 具体选哪个取决于路径规划，如 D7-N7 挡板会改变最优通路）
    assert "D2" not in tokens
    d2_neighbors = set(neighbor_names("D2"))
    assert len(cmds.shoot_cells) == 1
    assert cmds.shoot_cells[0] in d2_neighbors
    assert f"{cmds.shoot_cells[0]}*" in tokens
    assert "F6" not in tokens


def test_build_mission_cross_gates_from_names():
    s = snapshot(())
    # 格名字符串透传：F6 为唯一可穿越点，A1 -> A11 可达且经过 F6
    cmds = build_mission(s, "A1", "A11", "y", None, cross_gates=("F6",))
    tokens = cmds.route.split()
    assert tokens[0] == "A1"
    assert tokens[-1] == "A11"
    assert "F6" in tokens


def test_build_mission_turn_balance_plumbed_through():
    """步数-姿态平衡参数透传：build_mission 的 turn_45/allow_diagonal
    必须一路传到 plan_route_zoned/plan_route（配置层 match_params.yaml ->
    integrate_node -> build_mission -> 路由层整链生效）。"""
    s = snapshot(())

    def _moves(route_tokens):
        cells = [t.rstrip("*").rstrip(",x").rstrip(",y") for t in route_tokens]
        cells = [t for t in cells if t and t[0] in "ABCDEFGHIJKLMN"]
        return [
            (ord(b[0]) - ord(a[0]), int(b[1:]) - int(a[1:]))
            for a, b in zip(cells, cells[1:])
        ]

    # 默认（allow_diagonal=True）：E2->G4 两步对角直达
    cmds = build_mission(s, "E2", "G4", "x", None)
    moves = _moves(cmds.route.split())
    assert sum(1 for dc, dr in moves if dc and dr) == 2

    # allow_diagonal=False 透传关闭对角：退化为 4 步纯正交
    cmds_orth = build_mission(s, "E2", "G4", "x", None, allow_diagonal=False)
    moves_orth = _moves(cmds_orth.route.split())
    assert len(moves_orth) == 4
    assert all((dc == 0) != (dr == 0) for dc, dr in moves_orth)

    # turn_45=0 透传生效：纯最短距 = 两步对角、中途零转向（形态键 (0, 2)；
    # _route_shape 不计首步起步转向——各候选口径一致，比较公平性不变）
    from ag_integrate.mission_router import _route_shape
    cmds_t0 = build_mission(s, "E2", "G4", "x", None, turn_45=0.0)
    cells = [t.rstrip("*").rstrip(",x").rstrip(",y") for t in cmds_t0.route.split()]
    cells = [t for t in cells if t and t[0] in "ABCDEFGHIJKLMN"]
    assert _route_shape(tuple(cells)) == (0, 2)


def test_inject_track_only_hits_targets_on_track():
    """移动靶轨道先验只注入落在轨道上的靶：固定红靶不触发全局禁火，
    轨道上的蓝靶按轨道几何停靠（D9/E9 蓝面侧正对端点）。"""
    s = snapshot(
        (
            obj(CellRef(0, 5), ObjectType.PAPER),           # A6
            obj(CellRef(3, 7), ObjectType.TARGET_BLUE),    # D8：轨道上
            obj(CellRef(13, 1), ObjectType.TARGET_RED),    # N2：固定红靶
        )
    )
    cmds = build_mission(s, "A1", "A11", "y", None,
                         target_track=("D8", "E8"), target_blue_face="S")
    tokens = cmds.route.split()
    stars = [t.rstrip("*") for t in tokens if t.endswith("*")]
    # 移动靶按轨道几何停靠：D9/E9 之一带 *（固定红靶 N2 若被误注入轨道
    # 会触发全局禁火，这里能打出 * 即证明未注入）
    assert any(c in stars for c in ("D9", "E9"))
    assert "N2" not in tokens     # 固定红靶格不可踩


def test_inject_track_leaves_fixed_blue_on_fixed_geometry():
    """不在轨道上的固定蓝靶保持固定语义：停靠格仍是其正交邻格，
    不被轨道几何带偏；与移动靶各自命中、各记一次。"""
    s = snapshot(
        (
            obj(CellRef(0, 5), ObjectType.PAPER),           # A6
            obj(CellRef(3, 7), ObjectType.TARGET_BLUE),    # D8：移动靶
            obj(CellRef(13, 8), ObjectType.TARGET_BLUE),   # N9：固定蓝靶
        )
    )
    cmds = build_mission(s, "A1", "A11", "y", None,
                         target_track=("D8", "E8"), target_blue_face="S")
    stars = [t.rstrip("*") for t in cmds.route.split() if t.endswith("*")]
    assert len(cmds.shoot_cells) == 2
    assert any(c in stars for c in ("D9", "E9"))            # 移动靶轨道停靠
    assert any(c in stars for c in ("M9", "N8", "N10"))     # 固定靶邻格停靠


def test_parse_name_boundary_validation():
    """_parse_name 边界校验：非法格号（字母越界/行号越界/空串）抛 ValueError，
    在规划入口就炸，不留到路由深处。"""
    import pytest
    s = snapshot(())
    with pytest.raises(ValueError):
        build_mission(s, "A1", "A11", "y", None, cross_gates=("Z9",))
    with pytest.raises(ValueError):
        build_mission(s, "A1", "A11", "y", None, cross_gates=("F99",))
    with pytest.raises(ValueError):
        build_mission(s, "A1", "A11", "y", None, cross_gates=("",))
