"""dzbrain_bridge 测试：UDP 0x4001 -> dzbrain JSON 话题桥接（13 类协议）。

不碰真实网络：udp_port=0（绑定临时端口，永远收不到包），直接向
_process_packet() 喂帧、tick() 触发定时器、inject_str() 喂车端回报，
断言发到 /dzlink/drone_to_car 的 JSON 内容。

13 类协议（canonical）：雷 = 非 K 列 PAPER，桥洞箭头 = K 列 PAPER
（落格判语义，见 intel_protocol.ObjectType）。桥的 WorldModel 用默认
obstacle_confirm_frames=2（单帧幻影障碍不进态势），所以含障碍的场景
喂两帧确认——这也是"多帧共识"语义的一部分。

时间源：_node_stub_utils.tick() 每拍推进假时钟 0.1s，第 k 拍 t=k*0.1s，
周期断言（心跳/刷新）据此排拍。
"""

import json

import pytest

import _node_stub_utils as stub
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
    """走真实收包路径喂一帧 0x4001：字节编码 -> 解析 -> 态势 -> 入队。"""
    node._process_packet(build(IntelFrame(ts, tuple(objects))))


def published_json(node: DzbrainBridgeNode) -> list[dict]:
    return [json.loads(m.data)
            for m in stub.pub(node, "/dzlink/drone_to_car").sent]


def by_type(node: DzbrainBridgeNode, msg_type: str) -> list[dict]:
    return [p for p in published_json(node) if p.get("type") == msg_type]


def make_node(monkeypatch, **overrides) -> DzbrainBridgeNode:
    params = dict(udp_port=0, log_enabled=False)
    params.update(overrides)
    return stub.make_node(monkeypatch, DzbrainBridgeNode, **params)


# ---------------- 黄金报文 ----------------

def test_golden_frame_messages(monkeypatch):
    """黄金报文：桥洞(K9 PAPER)+雷(A2 PAPER)+障(B4)+蓝靶(E5) 一帧喂入，
    发布到 drone_to_car 的 JSON 逐字段核对。障碍喂两帧（默认 N=2 确认）。"""
    node = make_node(monkeypatch)
    try:
        for _ in range(2):
            feed(node,
                 obj_at(ObjectType.PAPER, "K9"),    # K 列 → 桥洞
                 obj_at(ObjectType.PAPER, "A2"),    # 非 K 列 → 雷
                 obj_at(ObjectType.OBSTACLE, "B4"),
                 obj_at(ObjectType.TARGET_BLUE, "E5"))
        stub.tick(node)

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


def test_upper_half_arrow_reports_upper(monkeypatch):
    """上半场桥洞（K5，第 5 行）→ half="upper"。"""
    node = make_node(monkeypatch)
    try:
        feed(node, obj_at(ObjectType.PAPER, "K5"))
        stub.tick(node)
        bridge = by_type(node, "bridge_passable")
        assert len(bridge) == 1
        assert bridge[0]["half"] == "upper"
        assert bridge[0]["block"] == "K5"
    finally:
        node.destroy_node()


def test_half_boundary_rows(monkeypatch):
    """半场分界：第 6 行→upper、第 7 行→lower（挡板在 6/7 行界）。"""
    node = make_node(monkeypatch)
    try:
        feed(node, obj_at(ObjectType.PAPER, "K6"),
             obj_at(ObjectType.PAPER, "K7"))
        stub.tick(node)
        got = {b["block"]: b["half"] for b in by_type(node, "bridge_passable")}
        assert got == {"K6": "upper", "K7": "lower"}
    finally:
        node.destroy_node()


# ---------------- 心跳 ----------------

def test_heartbeat_period_and_increasing_seq(monkeypatch):
    """heartbeat 以配置周期恒定发（无情报也发），seq 严格递增。"""
    node = make_node(monkeypatch, heartbeat_period_s=0.3)
    try:
        for _ in range(10):       # 每拍推进 0.1s：t=0.1 .. 1.0
            stub.tick(node)
        beats = by_type(node, "heartbeat")
        # t=0.1 首发（下一拍 0.4），之后 0.4/0.7/1.0 各一条
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

def test_change_triggers_immediate_and_refresh_periodic(monkeypatch):
    """同一情报不连刷；情报变化立即发；无变化到刷新周期补发。"""
    node = make_node(monkeypatch, refresh_period_s=0.5)
    try:
        feed(node, obj_at(ObjectType.PAPER, "K9"))
        stub.tick(node)                        # t=0.1：首发 K9
        assert len(by_type(node, "bridge_passable")) == 1

        # 同一情报再来一帧：签名未变，不连刷
        feed(node, obj_at(ObjectType.PAPER, "K9"))
        stub.tick(node)                        # t=0.2
        stub.tick(node)                        # t=0.3
        assert len(by_type(node, "bridge_passable")) == 1

        # 情报变化（桥洞格移动 K9->K8）：不等刷新周期，立即发
        feed(node, obj_at(ObjectType.PAPER, "K8"))
        stub.tick(node)                        # t=0.4
        bridges = by_type(node, "bridge_passable")
        assert len(bridges) == 2
        assert bridges[-1]["block"] == "K8"

        # 无变化但超过刷新周期（0.5s）：t=0.9 补发一次同内容
        for _ in range(6):                     # t=0.5 .. 1.0
            stub.tick(node)
        bridges = by_type(node, "bridge_passable")
        assert len(bridges) == 3
        assert bridges[-1]["block"] == "K8"
    finally:
        node.destroy_node()


def test_target_zone_includes_red_and_blue(monkeypatch):
    """target_zone 红+蓝靶都报（车上与本地地图核对用）。"""
    node = make_node(monkeypatch)
    try:
        feed(node,
             obj_at(ObjectType.TARGET_BLUE, "E5"),
             obj_at(ObjectType.TARGET_RED, "F6"))
        stub.tick(node)
        zone = by_type(node, "target_zone")
        assert len(zone) == 1
        assert set(zone[0]["blocks"]) == {"E5", "F6"}
    finally:
        node.destroy_node()


def test_empty_target_zone_not_sent(monkeypatch):
    """无靶时 target_zone 整条不发（空 blocks 会触发 DroneTask 缺字段告警）；
    field_verify 空列表照发（"全场干净"是有效情报）。"""
    node = make_node(monkeypatch)
    try:
        feed(node, obj_at(ObjectType.PAPER, "A2"))
        stub.tick(node)
        assert by_type(node, "target_zone") == []
        verify = by_type(node, "field_verify")
        assert len(verify) == 1
        assert verify[0]["mines"] == ["A2"]
        assert verify[0]["obstacles"] == []
    finally:
        node.destroy_node()


# ---------------- 车端回报（/dzlink/car_to_drone） ----------------

def test_car_to_drone_ack_and_bad_messages_never_crash(monkeypatch):
    """喂 ack/car_state/坏 JSON/非对象/非字符串：不抛异常，坏消息 warn。"""
    node = make_node(monkeypatch)
    try:
        stub.inject_str(node, "/dzlink/car_to_drone",
                        '{"type":"ack","of":3,"ts":1.0}')
        stub.inject_str(node, "/dzlink/car_to_drone",
                        '{"type":"car_state","block":"N10","yaw_deg":90.0,'
                        '"phase":"scout","ts":1.0,"seq":2}')
        stub.inject_str(node, "/dzlink/car_to_drone", "not json{")   # 非法 JSON
        stub.inject_str(node, "/dzlink/car_to_drone", "[1,2,3]")     # 非对象
        stub.inject_str(node, "/dzlink/car_to_drone",
                        '{"type":"mystery"}')                        # 未知类型
        stub.inject_str(node, "/dzlink/car_to_drone", 123)           # 非字符串

        # 节点存活：照常发心跳
        stub.tick(node)
        assert by_type(node, "heartbeat")

        log = node.get_logger()
        debugs = log.text(level="debug")
        warns = log.text(level="warning")
        assert any("ack" in m for m in debugs)
        assert any("车端状态" in m for m in debugs)
        assert any("非法 JSON" in m for m in warns)
        assert any("不是 JSON 对象" in m for m in warns)
        assert any("未知车端消息类型" in m for m in warns)
        assert any("非字符串" in m for m in warns)
    finally:
        node.destroy_node()


# ---------------- UDP 坏帧 ----------------

def test_bad_crc_frame_rejected_node_survives(monkeypatch):
    """UDP 坏帧（CRC 错）：拒收、记告警、不崩；后续好帧照常处理。"""
    node = make_node(monkeypatch)
    try:
        good = build(IntelFrame(1, (obj_at(ObjectType.PAPER, "K9"),)))
        bad = bytearray(good)
        bad[-6] ^= 0xFF      # 破坏 CRC（suffix 前 2 字节是 CRC）
        node._process_packet(bytes(bad))
        stub.tick(node)
        assert by_type(node, "bridge_passable") == []
        assert any("丢弃坏帧" in m for m in node.get_logger().text())

        # 节点存活：好帧照常桥接
        node._process_packet(good)
        stub.tick(node)
        assert len(by_type(node, "bridge_passable")) == 1
    finally:
        node.destroy_node()


# ---------------- 报文兼容性金线（对照 DroneTask 解析逻辑） ----------------

def test_wire_compatibility_with_drone_task_parser(monkeypatch):
    """逐条对照 dzbrain DroneTask 的解析逻辑（字段名/类型/取值域）。

    真源：reference/car_src_20261015/dzbrain/dzbrain/tasks/drone.py。
    这里任何一项不符，车端那边就会计 missing_fields 丢消息——
    本测试是我们发出报文与车端解析器之间的金线。
    """
    node = make_node(monkeypatch)
    try:
        for _ in range(2):   # 障碍 B4 需两帧确认（默认 N=2）
            feed(node,
                 obj_at(ObjectType.PAPER, "K9"),
                 obj_at(ObjectType.PAPER, "K5"),
                 obj_at(ObjectType.PAPER, "A2"),
                 obj_at(ObjectType.OBSTACLE, "B4"),
                 obj_at(ObjectType.TARGET_BLUE, "E5"),
                 obj_at(ObjectType.TARGET_RED, "F6"))
        stub.tick(node)
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
        def bind(self, *a):
            raise OSError("address already in use")

        def close(self):
            pass

    fake_socket_mod = _types.SimpleNamespace(
        AF_INET=_socket.AF_INET, SOCK_DGRAM=_socket.SOCK_DGRAM,
        timeout=_socket.timeout, socket=lambda *a, **k: _BadSock(),
    )
    monkeypatch.setattr(m, "socket", fake_socket_mod)
    with pytest.raises(OSError):
        make_node(monkeypatch)


def test_destroy_node_stops_receive_thread(monkeypatch):
    """优雅关闭：destroy_node 停收包线程、关 socket。"""
    node = make_node(monkeypatch)
    assert node._thread.is_alive()
    node.destroy_node()
    assert not node._thread.is_alive()


def test_period_params_must_be_positive(monkeypatch):
    """heartbeat_period_s/refresh_period_s <=0：构造即抛——非法周期会让
    心跳/刷新每拍连发刷爆话题（对齐 integrate_node 启动自检范式；
    2026-10-18 合并分支从 main 移植该 fail-fast 校验）。"""
    with pytest.raises(ValueError):
        make_node(monkeypatch, heartbeat_period_s=0.0)
    with pytest.raises(ValueError):
        make_node(monkeypatch, heartbeat_period_s=-0.5)
    with pytest.raises(ValueError):
        make_node(monkeypatch, refresh_period_s=0.0)


# ---------------- UDP 互斥防线（刻意不设 SO_REUSEADDR，重复 bind 即 fail-fast） ----------------

def _hold_udp_port() -> tuple:
    """占住一个 UDP 端口（通配地址，与节点 bind("", port) 同架），返回 (sock, port)。"""
    import socket as _socket
    hold = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
    hold.bind(("", 0))
    return hold, hold.getsockname()[1]


def test_second_bridge_same_port_fails_fast(monkeypatch):
    """同一 UDP 端口已被占时第二个桥构造即抛——"绝不同时跑"靠 bind
    fail-fast 兜底（Linux 下 SO_REUSEADDR 会让双方重复 bind 且各收一份副本，
    双决策核心并行驱动车；UDP 无 TIME_WAIT，去掉它重启不受阻）。"""
    hold, port = _hold_udp_port()
    try:
        with pytest.raises(OSError):
            make_node(monkeypatch, udp_port=port)
    finally:
        hold.close()


def test_bridge_and_integrate_node_mutual_exclusion(monkeypatch):
    """跨节点互斥：integrate_node 在跑时 dzbrain_bridge 绑同一端口即抛
    （两节点同抢 UDP:50000，决策核心只能有一个）。"""
    import socket as _socket

    from ag_integrate.integrate_node import IntegrateNode

    probe = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
    probe.bind(("", 0))
    port = probe.getsockname()[1]
    probe.close()
    n1 = stub.make_node(monkeypatch, IntegrateNode, udp_port=port,
                        matrix_log=False, log_enabled=False)
    try:
        with pytest.raises(OSError):
            make_node(monkeypatch, udp_port=port)   # 第二决策核心 → 当场炸
    finally:
        n1.destroy_node()


# ---------------- 黑匣子事件 ----------------

def test_blackbox_events(monkeypatch, tmp_path):
    """黑匣子关键事件：启动 session.begin/node.args + 发报 dzbrain.send
    （EV 表无 dzbrain 桥专用常量，字面量事件名见 dzbrain_bridge 注释）。"""
    from ag_core.blackbox import read_events
    node = make_node(monkeypatch, log_enabled=True, log_dir=str(tmp_path))
    try:
        feed(node, obj_at(ObjectType.PAPER, "K9"))
        stub.tick(node)
    finally:
        node.destroy_node()
    events, bad = read_events(tmp_path)
    assert bad == 0
    kinds = {e.ev for e in events}
    assert "session.begin" in kinds and "node.args" in kinds
    sends = [e for e in events if e.ev == "dzbrain.send"]
    assert sends and any("bridge_passable" in e.get("body", "") for e in sends)
