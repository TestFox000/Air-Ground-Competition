"""机载自主规划旁挂进程 + 0x4003 车命令帧契约测试。

覆盖三件事：
  1. 0x4003 编解码：build/parse 往返 + 各类损坏帧一律拒绝（宁错不发）；
  2. PlannerCore 规划核心：态势 -> 门控 -> 规划 -> 三份产物（无 socket，可离线跑）；
  3. 车端 onboard 薄接收：0x4003 -> /nav/waypoint（走真实收包路径 + tick），
     且 local 模式必须不消费 0x4003（规划来源互斥）。

零硬件：不碰真实网络（udp_port=0），不碰 dzuav、不碰相机。
"""
from __future__ import annotations

import importlib.util
import json
import struct
import sys
from pathlib import Path

import pytest

import _node_stub_utils as stub

REPO = Path(__file__).resolve().parents[1]


def load_module(name: str, file: Path):
    spec = importlib.util.spec_from_file_location(name, file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


sidecar = load_module("orin_plan_sidecar", REPO / "tools" / "orin_plan_sidecar.py")

from ag_integrate.integrate_node import IntegrateNode  # noqa: E402
from ag_integrate.intel_protocol import (  # noqa: E402
    MAGIC, SUFFIX, VERSION, CarCommandFrame, CarPose, IntelFrame, IntelObject,
    ObjectType, build_car_command, crc16, detect_msg_id, parse_car_command,
)

_LETTERS = "ABCDEFGHIJKLMN"


# ---------------------------------------------------------------- 构造用例数据

def _at(cell: str, t: ObjectType, size: float = 0.35) -> IntelObject:
    col = _LETTERS.index(cell[0])
    row = int(cell[1:]) - 1
    return IntelObject(t, 0.065 + col * 0.35, 0.295 + (10 - row) * 0.35, size, size)


def mine_at(cell: str) -> IntelObject:
    """雷 = 非 K 列 PAPER（落格判语义）。"""
    return _at(cell, ObjectType.PAPER)


def blue_at(cell: str) -> IntelObject:
    return _at(cell, ObjectType.TARGET_BLUE, 0.3)


def obstacle_at(cell: str) -> IntelObject:
    return _at(cell, ObjectType.OBSTACLE)


def frame_of(*objects: IntelObject, ts: int = 1) -> IntelFrame:
    return IntelFrame(ts, tuple(objects))


def _raw(ts: int, msg_id: int, declared: int, body: bytes, crc_ok: bool = True) -> bytes:
    """手搓一帧（用于构造损坏帧）：结构同 build_car_command，可注入 CRC 错误。"""
    out = bytearray()
    out += MAGIC
    out.append(VERSION)
    out += struct.pack("<QHI", ts, msg_id, declared)
    out += body
    body_end = 19 + len(body)
    checksum = crc16(memoryview(out)[4:body_end])
    out += struct.pack("<H", checksum if crc_ok else checksum ^ 0xFFFF)
    out += SUFFIX
    return bytes(out)


def _cmd_body(reset: bytes, route: bytes) -> bytes:
    return struct.pack("<H", len(reset)) + reset + struct.pack("<H", len(route)) + route


# ================================================================ 1. 协议契约

def test_car_command_roundtrip_and_msg_id():
    frame = CarCommandFrame(1712345678, "reset:N10,x", "N10 M9 D2* E2,-y")
    data = build_car_command(frame)
    assert detect_msg_id(data) == 0x4003
    assert parse_car_command(data) == frame


def test_car_command_allows_empty_reset():
    """reset 空串合法：本局不发锚定命令（publish_reset=false 的车端形态）。"""
    data = build_car_command(CarCommandFrame(1, "", "A5"))
    assert parse_car_command(data).reset == ""


def test_car_command_build_rejects_empty_route_and_bad_reset():
    with pytest.raises(ValueError):
        build_car_command(CarCommandFrame(1, "", "   "))
    with pytest.raises(ValueError):
        build_car_command(CarCommandFrame(1, "N10,x", "A5"))   # reset 缺前缀


def test_car_command_rejects_wrong_msg_id():
    """0x4001 情报帧喂给 parse_car_command 必须拒绝（不得跨类型误解析）。"""
    intel = _raw(1, 0x4001, 4, struct.pack("<H", 1))
    with pytest.raises(ValueError):
        parse_car_command(intel)


def test_car_command_rejects_bad_crc():
    body = _cmd_body(b"reset:A1,x", b"A5")
    with pytest.raises(ValueError):
        parse_car_command(_raw(1, 0x4003, len(body), body, crc_ok=False))


def test_car_command_rejects_truncated_and_bad_suffix():
    body = _cmd_body(b"reset:A1,x", b"A5")
    good = _raw(1, 0x4003, len(body), body)
    with pytest.raises(ValueError):
        parse_car_command(good[:-1])
    with pytest.raises(ValueError):
        parse_car_command(good[:-4] + b"\x00\xfe\xfe\xfe")


def test_car_command_rejects_non_ascii_and_nul():
    with pytest.raises(ValueError):
        parse_car_command(_raw(1, 0x4003, len(_cmd_body(b"", "路线".encode("utf-8"))),
                               _cmd_body(b"", "路线".encode("utf-8"))))
    with pytest.raises(ValueError):
        parse_car_command(_raw(1, 0x4003, len(_cmd_body(b"", b"A5\x00")),
                               _cmd_body(b"", b"A5\x00")))


def test_car_command_rejects_empty_route_on_parse():
    body = _cmd_body(b"reset:A1,x", b"")
    with pytest.raises(ValueError):
        parse_car_command(_raw(1, 0x4003, len(body), body))


def test_car_command_rejects_length_mismatch():
    body = _cmd_body(b"reset:A1,x", b"A5")
    with pytest.raises(ValueError):
        parse_car_command(_raw(1, 0x4003, len(body) + 3, body))


# ================================================================ 2. 规划核心

def make_core(tmp_path: Path, plan_mode: str = "once", **cfg_kw):
    cfg = sidecar.PlanConfig(**cfg_kw) if cfg_kw else sidecar.PlanConfig()
    return sidecar.PlannerCore(cfg, tmp_path, plan_mode=plan_mode)


def test_route_cells_strips_shoot_and_heading_token():
    assert sidecar.route_cells("N10 M9 D2* E2,-y") == ["N10", "M9", "D2", "E2"]
    assert sidecar.route_cells("A5") == ["A5"]


def test_core_holds_until_gate_ok(tmp_path):
    """只有靶没有雷：门控不放行，不得产出路线（但仍可落矩阵）。"""
    core = make_core(tmp_path)
    core.feed_frame(frame_of(blue_at("F5")))
    assert core.step() is None
    assert not (tmp_path / "route_plan.json").exists()
    assert (tmp_path / "cell_matrix.json").exists()


def test_core_matrix_signature_handles_out_of_field_and_in_cell_same_kind(tmp_path):
    """回归：同类目标一个出界(None) + 一个落格(int) 时，矩阵签名不得崩。

    2026-10-20 机上实跑踩到：`sorted((kind, col, row))` 遇到 None 与 int 混比
    抛 TypeError，_write_matrix 直接把 sidecar 进程打死（真机上表现为
    「矩阵在，规划不推进」）。出界点取 x=-1.0（世界 x 取值范围 0.065~4.615）。
    """
    core = make_core(tmp_path)
    out_of_field = IntelObject(ObjectType.BUCKET_RED, -1.0, 6.5, 0.35, 0.35)
    in_cell = _at("F5", ObjectType.BUCKET_RED)
    for _ in range(2):        # 融合需 confirm=2 帧
        core.feed_frame(frame_of(out_of_field, in_cell))
    core.step()               # 不得抛异常
    core.write_matrix()

    matrix = json.loads((tmp_path / "cell_matrix.json").read_text(encoding="utf-8"))
    cells = [s["cell"] for s in matrix["spots"]]
    assert None in cells and any(c for c in cells)   # 出界与落格同时保留


def test_core_step_does_not_raise_on_matrix_failure(tmp_path, monkeypatch):
    """回归：矩阵落盘抛异常时 step() 不得把异常甩出去（真机曾整进程被打死）。

    2026-10-20 机上实跑：`_write_matrix` 的 TypeError 直接终结 sidecar 进程，
    现场表现是"矩阵在、规划不推进"。外壳已把决策拍包进异常兜底，这里锁住
    "核心自身也不越过异常边界"这一契约：写出异常不能挡住门控与规划。
    """
    core = make_core(tmp_path)
    for _ in range(2):
        core.feed_frame(frame_of(mine_at("D2"), blue_at("F5")))

    def boom(*_a, **_k):
        raise TypeError("simulated matrix failure")

    monkeypatch.setattr(core, "_write_matrix", boom)
    route = core.step()                      # 不得抛异常
    assert route, "矩阵落盘失败不应挡住门控与规划"
    assert (tmp_path / "route_plan.json").exists()


def test_core_plans_and_writes_three_artifacts(tmp_path):
    core = make_core(tmp_path)
    # 矩阵融合需 confirm=2 帧才确认目标（单帧幽灵过滤）：喂两帧
    for _ in range(2):
        core.feed_frame(frame_of(mine_at("D2"), blue_at("F5")))
    route = core.step()
    assert route, "门控通过后必须出路线"
    assert core.last_reset.startswith("reset:")

    plan = json.loads((tmp_path / "route_plan.json").read_text(encoding="utf-8"))
    assert plan["planner"] == "onboard_sidecar"
    assert plan["car_cmds"]["reset"] == core.last_reset
    assert plan["car_cmds"]["route"] == route
    assert plan["check"]["排雷覆盖"] == "全部覆盖"
    assert plan["check"]["障碍避让"] == "通过"
    assert plan["route_len_cells"] == len(plan["route"]) >= 2

    matrix = json.loads((tmp_path / "cell_matrix.json").read_text(encoding="utf-8"))
    grid = matrix["grid"]
    assert len(grid) == 11 and len(grid[0]) == 14
    assert grid[1][3] == "M"      # D2：雷（列 D=3，行 2 -> 下标 1）
    assert grid[4][5] == "B"      # F5：蓝靶（列 F=5，行 5 -> 下标 4）

    # planned_path.txt = 执行路线格序列
    assert (tmp_path / "planned_path.txt").read_text(encoding="utf-8").split() \
        == plan["route"]


def test_core_route_covers_all_mines(tmp_path):
    core = make_core(tmp_path)
    core.feed_frame(frame_of(mine_at("M9"), mine_at("D2"), blue_at("F5")))
    route = core.step()
    cells = sidecar.route_cells(route)
    assert "M9" in cells and "D2" in cells


def test_core_once_mode_does_not_replan(tmp_path):
    """once 模式（默认，对齐 yaml replan_on_update=false）：出过路线不再重算。"""
    core = make_core(tmp_path, plan_mode="once")
    core.feed_frame(frame_of(mine_at("D2"), blue_at("F5")))
    assert core.step()
    core.feed_frame(frame_of(mine_at("D2"), mine_at("A2"), blue_at("F5")))
    assert core.step() is None
    assert core.plan_count == 1


def test_core_on_change_mode_replans_when_route_blocked(tmp_path):
    """on-change 模式（重规划开启/离线回放）：路线上的格被确认障碍后必须重规划。

    障碍需连续 2 帧确认（canonical 默认 N=2），所以同一份障碍喂两帧。
    """
    core = make_core(tmp_path, plan_mode="on-change")
    base = (mine_at("D2"), blue_at("F5"))
    core.feed_frame(frame_of(*base))
    first = core.step()
    assert first

    cells = sidecar.route_cells(first)
    protected = {"N10", "N5", "D2", "F5"}
    blocked = next((c for c in cells[1:-1] if c not in protected), cells[len(cells) // 2])

    core.feed_frame(frame_of(*base, obstacle_at(blocked)))
    core.feed_frame(frame_of(*base, obstacle_at(blocked)))
    second = core.step()
    assert second, "路线被障碍封堵后必须重规划出新路线"
    assert second != first
    assert blocked not in sidecar.route_cells(second)
    assert core.plan_count == 2


def test_core_uses_car_pose_nearest_axis_for_reset_assertion(tmp_path):
    """断言集合只有 {x=东, y=北}：车头朝东（yaw=0）时断言必须吸附到 x。"""
    core = make_core(tmp_path, heading_bias_source="udp", start_heading="y")
    core.feed_car_pose(CarPose(1, 1.0, 2.0, 0.0))     # 朝东 → 最近轴 x
    assert sidecar.effective_start_heading(core.cfg, core._fresh_car_yaw()) == "x"


def test_core_ignores_stale_car_pose(tmp_path):
    core = make_core(tmp_path, heading_bias_source="udp", start_heading="y",
                     car_pose_ttl_s=0.01)
    core.feed_car_pose(CarPose(1, 1.0, 2.0, 0.0), recv_mono=0.0)
    assert core._fresh_car_yaw() is None                   # 过期 = 没有观测
    assert sidecar.effective_start_heading(core.cfg, core._fresh_car_yaw()) == "y"


def test_config_from_canonical_yaml_roundtrip():
    """yaml -> PlanConfig 必须解出 canonical 值（防止出现第二份默认值副本）。"""
    raw = sidecar.load_params(REPO / "src" / "ag_integrate" / "config" / "match_params.yaml")
    cfg = sidecar.config_from_params(raw)
    assert (cfg.start_cell, cfg.goal_cell) == ("N10", "N5")
    assert (cfg.start_heading, cfg.final_heading) == ("y", "y")
    assert cfg.mid_boundary == 6
    assert cfg.obstacle_confirm_frames == 2
    assert cfg.target_track == ("D7", "E7")
    assert cfg.udp_port == 50000
    # 机载规划默认「出一次路线不再重算」，与 yaml 的 replan_on_update 同口径
    assert cfg.replan_on_update is False


# ================================================================ 3. 车端薄接收

def make_node(monkeypatch, **overrides) -> IntegrateNode:
    params = dict(
        udp_port=0, dry_run=False, publish_reset=True,
        start_cell="N10", goal_cell="N5", start_heading="y", final_heading="y",
        matrix_log=False, log_enabled=False,
    )
    params.update(overrides)
    return stub.make_node(monkeypatch, IntegrateNode, **params)


def test_onboard_mode_forwards_car_command(monkeypatch):
    node = make_node(monkeypatch, car_command_source="onboard")
    try:
        node._process_packet(
            build_car_command(CarCommandFrame(1, "reset:N10,x", "N10 M9,-y")))
        stub.tick(node)
        assert stub.published_data(node, "/nav/waypoint") == ["reset:N10,x", "N10 M9,-y"]
    finally:
        node.destroy_node()


def test_onboard_mode_does_not_republish_identical_route(monkeypatch):
    node = make_node(monkeypatch, car_command_source="onboard")
    try:
        node._process_packet(
            build_car_command(CarCommandFrame(1, "reset:N10,x", "N10 M9,-y")))
        stub.tick(node)
        stub.tick(node)
        assert len(stub.published_data(node, "/nav/waypoint")) == 2
    finally:
        node.destroy_node()


def test_local_mode_ignores_car_command(monkeypatch):
    """规划来源互斥：local 模式（默认）不得消费机上 0x4003。"""
    node = make_node(monkeypatch)     # 默认 car_command_source="local"
    try:
        node._process_packet(
            build_car_command(CarCommandFrame(1, "reset:N10,x", "N10 M9,-y")))
        stub.tick(node)
        assert stub.published_data(node, "/nav/waypoint") == []
    finally:
        node.destroy_node()


def test_onboard_mode_ignores_bad_car_command(monkeypatch):
    """坏 0x4003（CRC 错）必须丢弃：绝不让未校验的命令上底盘。"""
    node = make_node(monkeypatch, car_command_source="onboard")
    try:
        bad = bytearray(build_car_command(
            CarCommandFrame(1, "reset:N10,x", "N10 M9,-y")))
        bad[20] ^= 0xFF                       # 破坏 body 首字节（reset_len 高位）
        node._process_packet(bytes(bad))
        stub.tick(node)
        assert stub.published_data(node, "/nav/waypoint") == []
    finally:
        node.destroy_node()


def test_onboard_mode_rejects_unknown_source(monkeypatch):
    with pytest.raises(ValueError):
        make_node(monkeypatch, car_command_source="cloud")
