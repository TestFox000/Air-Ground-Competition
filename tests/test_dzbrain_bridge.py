"""dzbrain_bridge 测试：UDP 0x4001 -> dzbrain JSON 话题桥接。

不碰真实网络：udp_port=0（绑定临时端口，永远收不到包），
直接向 _process_packet() 喂帧、tick() 触发定时器、deliver() 喂车端回报，
断言发到 /dzlink/drone_to_car 的 JSON 内容。

时间源：ros_stub.FakeClock 每次 now() 推进 0.1s，节点每 tick 只读一次，
所以 tick 序号即时间（第 k 拍 t=1.1+0.1*(k-1) 秒），周期断言据此排拍。
"""

import json
import types

import ros_stub  # noqa: F401  必须先于节点模块导入

import pytest

from ag_integrate.dzbrain_bridge import DzbrainBridgeNode
from ag_integrate.intel_protocol import (
    IntelFrame, IntelObject, ObjectType, build,
)

_LETTERS = "ABCDEFGHIJKLMN"


def obj_at(obj_type: ObjectType, cell: str) -> IntelObject:
    """在格心构造一个目标（世界坐标公式与 field_frame 一致）。"""
    col = _LETTERS.index(cell[0])
    row = int(cell[1:]) - 1
    x = 0.065 + col * 0.35
    y = 0.295 + (10 - row) * 0.35
    return IntelObject(obj_type, x, y, 0.35, 0.35)


def feed(node: DzbrainBridgeNode, *objects: IntelObject, ts: int = 1) -> None:
    """走真实收包路径喂一帧 0x4001：字节编码 -> 解析 -> 入队。"""
    node._process_packet(build(IntelFrame(ts, tuple(objects))))


def published_json(node: DzbrainBridgeNode) -> list[dict]:
    return [json.loads(m.data)
            for m in node.topic("/dzlink/drone_to_car").published]


def by_type(node: DzbrainBridgeNode, msg_type: str) -> list[dict]:
    return [p for p in published_json(node) if p.get("type") == msg_type]


@pytest.fixture(autouse=True)
def _overrides():
    """基线参数覆盖（fixture 里设置，避免 import 期污染全局 _OVERRIDES）。"""
    ros_stub.set_overrides(udp_port=0)
    yield
    ros_stub.clear_overrides()


def make_node(**overrides) -> DzbrainBridgeNode:
    params = dict(udp_port=0)
    params.update(overrides)
    ros_stub.set_overrides(**params)
    return DzbrainBridgeNode()


# ---------------- 黄金报文 ----------------

def test_golden_frame_messages():
    """黄金报文：ARROW(K9)+雷(A2)+障(B4)+蓝靶(E5) 一帧喂入，
    发布到 drone_to_car 的 JSON 逐字段核对。"""
    node = make_node()
    try:
        feed(node,
             obj_at(ObjectType.PAPER, "K9"),
             obj_at(ObjectType.PAPER, "A2"),
             obj_at(ObjectType.OBSTACLE, "B4"),
             obj_at(ObjectType.TARGET_BLUE, "E5"))
        node.tick()

        bridge = by_type(node, "bridge_passable")
        assert len(bridge) == 1
        b = bridge[0]
        assert b["type"] == "bridge_passable"
        assert b["half"] == "lower"          # K9 在第 9 行 → 下半场
        assert b["block"] == "K9"
        assert b["confidence"] == 1.0
        assert isinstance(b["ts"], (int, float))
        assert isinstance(b["seq"], int) and b["seq"] >= 1

        verify = by_type(node, "field_verify")
        assert len(verify) == 1
        assert verify[0]["type"] == "field_verify"
        assert verify[0]["mines"] == ["A2"]
        assert verify[0]["obstacles"] == ["B4"]

        zone = by_type(node, "target_zone")
        assert len(zone) == 1
        assert zone[0]["type"] == "target_zone"
        assert "E5" in zone[0]["blocks"]
    finally:
        node.destroy_node()


def test_upper_half_arrow_reports_upper():
    """上半场 ARROW（K5，第 5 行）→ half="upper"。"""
    node = make_node()
    try:
        feed(node, obj_at(ObjectType.PAPER, "K5"))
        node.tick()
        bridge = by_type(node, "bridge_passable")
        assert len(bridge) == 1
        assert bridge[0]["half"] == "upper"
        assert bridge[0]["block"] == "K5"
    finally:
        node.destroy_node()


def test_half_boundary_rows():
    """半场分界：第 6 行→upper、第 7 行→lower（挡板在 6/7 行界）。"""
    node = make_node()
    try:
        feed(node, obj_at(ObjectType.PAPER, "K6"),
             obj_at(ObjectType.PAPER, "K7"))
        node.tick()
        got = {b["block"]: b["half"] for b in by_type(node, "bridge_passable")}
        assert got == {"K6": "upper", "K7": "lower"}
    finally:
        node.destroy_node()


# ---------------- 心跳 ----------------

def test_heartbeat_period_and_increasing_seq():
    """heartbeat 以配置周期恒定发（无情报也发），seq 严格递增。"""
    node = make_node(heartbeat_period_s=0.3)
    try:
        for _ in range(10):       # 每 tick 推进 0.1s：t=1.1 .. 2.0
            node.tick()
        beats = by_type(node, "heartbeat")
        # t=1.1 首发（下一拍 1.4），之后 1.4/1.7/2.0 各一条
        assert len(beats) == 4
        for b in beats:
            assert b["type"] == "heartbeat"
            assert isinstance(b["ts"], (int, float))
            assert isinstance(b["seq"], int)
        seqs = [b["seq"] for b in beats]
        assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)
        # 无情报：只有心跳，没有别的报文
        assert {p["type"] for p in published_json(node)} == {"heartbeat"}
    finally:
        node.destroy_node()


# ---------------- 变化触发 vs 周期刷新 ----------------

def test_change_triggers_immediate_and_refresh_periodic():
    """同一情报不连刷；情报变化立即发；无变化到刷新周期补发。"""
    node = make_node(refresh_period_s=0.5)
    try:
        feed(node, obj_at(ObjectType.PAPER, "K9"))
        node.tick()                        # t=1.1：首发 K9
        assert len(by_type(node, "bridge_passable")) == 1

        # 同一情报再来一帧：签名未变，不连刷
        feed(node, obj_at(ObjectType.PAPER, "K9"))
        node.tick()                        # t=1.2
        node.tick()                        # t=1.3
        assert len(by_type(node, "bridge_passable")) == 1

        # 情报变化（桥洞格移动 K9->K8）：不等刷新周期，立即发
        feed(node, obj_at(ObjectType.PAPER, "K8"))
        node.tick()                        # t=1.4
        bridges = by_type(node, "bridge_passable")
        assert len(bridges) == 2
        assert bridges[-1]["block"] == "K8"

        # 无变化但超过刷新周期（0.5s）：t=1.9 补发一次同内容
        for _ in range(6):                 # t=1.5 .. 2.0
            node.tick()
        bridges = by_type(node, "bridge_passable")
        assert len(bridges) == 3
        assert bridges[-1]["block"] == "K8"
    finally:
        node.destroy_node()


def test_target_zone_includes_red_and_blue():
    """target_zone 红+蓝靶都报（车上与本地地图核对用）。"""
    node = make_node()
    try:
        feed(node,
             obj_at(ObjectType.TARGET_BLUE, "E5"),
             obj_at(ObjectType.TARGET_RED, "F6"))
        node.tick()
        zone = by_type(node, "target_zone")
        assert len(zone) == 1
        assert set(zone[0]["blocks"]) == {"E5", "F6"}
    finally:
        node.destroy_node()


def test_empty_target_zone_not_sent():
    """无靶时 target_zone 整条不发（空 blocks 会触发 DroneTask 缺字段告警）；
    field_verify 空列表照发（"全场干净"是有效情报）。"""
    node = make_node()
    try:
        feed(node, obj_at(ObjectType.PAPER, "A2"))
        node.tick()
        assert by_type(node, "target_zone") == []
        verify = by_type(node, "field_verify")
        assert len(verify) == 1
        assert verify[0]["mines"] == ["A2"]
        assert verify[0]["obstacles"] == []
    finally:
        node.destroy_node()


# ---------------- 车端回报（/dzlink/car_to_drone） ----------------

def test_car_to_drone_ack_and_bad_messages_never_crash():
    """喂 ack/car_state/坏 JSON/非对象/非字符串：不抛异常，坏消息 warn。"""
    node = make_node()
    try:
        def deliver_raw(data):
            node.deliver("/dzlink/car_to_drone",
                         types.SimpleNamespace(data=data))

        deliver_raw('{"type":"ack","of":3,"ts":1.0}')
        deliver_raw('{"type":"car_state","block":"N10","yaw_deg":90.0,'
                    '"phase":"scout","ts":1.0,"seq":2}')
        deliver_raw("not json{")            # 非法 JSON
        deliver_raw("[1,2,3]")              # 合法 JSON 但非对象
        deliver_raw('{"type":"mystery"}')   # 未知类型
        deliver_raw(123)                    # 非字符串

        # 节点存活：照常发心跳
        node.tick()
        assert by_type(node, "heartbeat")

        records = node.logger.records
        debugs = [m for lvl, m in records if lvl == "debug"]
        warns = [m for lvl, m in records if lvl == "warning"]
        assert any("ack" in m for m in debugs)
        assert any("车端状态" in m for m in debugs)
        assert any("非法 JSON" in m for m in warns)
        assert any("不是 JSON 对象" in m for m in warns)
        assert any("未知车端消息类型" in m for m in warns)
        assert any("非字符串" in m for m in warns)
    finally:
        node.destroy_node()


# ---------------- UDP 坏帧 ----------------

def test_bad_crc_frame_rejected_node_survives():
    """UDP 坏帧（CRC 错）：拒收、记告警、不崩；后续好帧照常处理。"""
    node = make_node()
    try:
        good = build(IntelFrame(1, (obj_at(ObjectType.PAPER, "K9"),)))
        bad = bytearray(good)
        bad[-6] ^= 0xFF      # 破坏 CRC（suffix 前 2 字节是 CRC）
        node._process_packet(bytes(bad))
        node.tick()
        assert by_type(node, "bridge_passable") == []
        assert any("丢弃坏帧" in m for _lvl, m in node.logger.records)

        # 节点存活：好帧照常桥接
        node._process_packet(good)
        node.tick()
        assert len(by_type(node, "bridge_passable")) == 1
    finally:
        node.destroy_node()


# ---------------- 报文兼容性金线（对照 DroneTask 解析逻辑） ----------------

def test_wire_compatibility_with_drone_task_parser():
    """逐条对照 dzbrain DroneTask 的解析逻辑（字段名/类型/取值域）。

    真源：reference/car_src_20261015/dzbrain/dzbrain/tasks/drone.py。
    这里任何一项不符，车端那边就会计 missing_fields 丢消息——
    本测试是我们发出报文与车端解析器之间的金线。
    """
    node = make_node()
    try:
        feed(node,
             obj_at(ObjectType.PAPER, "K9"),
             obj_at(ObjectType.PAPER, "K5"),
             obj_at(ObjectType.PAPER, "A2"),
             obj_at(ObjectType.OBSTACLE, "B4"),
             obj_at(ObjectType.TARGET_BLUE, "E5"),
             obj_at(ObjectType.TARGET_RED, "F6"))
        node.tick()
        msgs = published_json(node)
        assert msgs, "节点必须发出报文"

        for payload in msgs:
            # DroneTask._handle_raw：非字符串/非法 JSON/非对象才丢弃——
            # json.loads 已成功，只验 type 白名单（其余记"未知类型"）
            assert payload["type"] in (
                "heartbeat", "bridge_passable", "target_zone", "field_verify")

        # _on_bridge_passable：half 归一化后必须 ∈ {lower,upper}，block 非空
        bridges = by_type(node, "bridge_passable")
        assert len(bridges) == 2            # 上下半场各一条
        for b in bridges:
            assert b["half"] in ("lower", "upper")
            assert isinstance(b["block"], str) and b["block"].strip()
            assert b["block"] == b["block"].upper()
            # confidence 走 _as_float：必须是 float() 可接受的数
            assert isinstance(b["confidence"], (int, float))
            assert 0.0 <= b["confidence"] <= 1.0
            # seq 触发 _maybe_ack 回执：整数序号
            assert isinstance(b["seq"], int) and b["seq"] >= 1
            assert isinstance(b["ts"], (int, float))

        # _on_target_zone：blocks 必须是非空 list，条目经 _norm_blocks
        # （str.strip().upper() 非空，畸形条目计 missing_fields）
        zone = by_type(node, "target_zone")
        assert len(zone) == 1
        blocks = zone[0]["blocks"]
        assert isinstance(blocks, list) and blocks
        assert all(isinstance(x, str) and x.strip() for x in blocks)
        assert all(x == x.upper() for x in blocks)

        # _on_field_verify：mines/obstacles 至少一键在；在的键必须是 list
        verify = by_type(node, "field_verify")
        assert len(verify) == 1
        v = verify[0]
        assert "mines" in v or "obstacles" in v
        for key in ("mines", "obstacles"):
            if key in v:
                assert isinstance(v[key], list)
                assert all(isinstance(x, str) and x.strip() for x in v[key])

        # _on_heartbeat 只读 type；seq 触发 ack，ts 为秒时间戳
        for h in by_type(node, "heartbeat"):
            assert h["type"] == "heartbeat"
            assert isinstance(h["seq"], int)
            assert isinstance(h["ts"], (int, float))
    finally:
        node.destroy_node()


# ---------------- 启动自检 / 优雅关闭 ----------------

def test_udp_bind_failure_raises_at_startup(monkeypatch):
    """UDP 端口绑定失败：构造函数当场抛错（fail-fast，同 integrate_node
    范式），而不是收包线程静默死亡、节点假装就绪。"""
    import socket as _socket
    import types as _types

    import ag_integrate.dzbrain_bridge as m

    class _BadSock:
        def setsockopt(self, *a):
            pass

        def bind(self, *a):
            raise OSError("address already in use")

        def close(self):
            pass

    fake_socket_mod = _types.SimpleNamespace(
        AF_INET=_socket.AF_INET, SOCK_DGRAM=_socket.SOCK_DGRAM,
        SOL_SOCKET=_socket.SOL_SOCKET, SO_REUSEADDR=_socket.SO_REUSEADDR,
        timeout=_socket.timeout, socket=lambda *a, **k: _BadSock(),
    )
    monkeypatch.setattr(m, "socket", fake_socket_mod)
    with pytest.raises(OSError):
        make_node()


def test_destroy_node_stops_receive_thread():
    """优雅关闭：destroy_node 停收包线程、关 socket。"""
    node = make_node()
    assert node._thread.is_alive()
    node.destroy_node()
    assert not node._thread.is_alive()


# ---------------- 黑匣子事件 ----------------

def test_blackbox_events(tmp_path):
    """黑匣子关键事件：启动 session.begin/node.args + 发报 dzbrain.send
    （EV 表无 dzbrain 桥专用常量，字面量事件名见 dzbrain_bridge 注释）。"""
    from ag_core.blackbox import read_events
    node = make_node(log_dir=str(tmp_path))
    try:
        feed(node, obj_at(ObjectType.PAPER, "K9"))
        node.tick()
    finally:
        node.destroy_node()
    events, bad = read_events(tmp_path)
    assert bad == 0
    kinds = {e.ev for e in events}
    assert "session.begin" in kinds and "node.args" in kinds
    sends = [e for e in events if e.ev == "dzbrain.send"]
    assert sends and any("bridge_passable" in e.get("body", "") for e in sends)
