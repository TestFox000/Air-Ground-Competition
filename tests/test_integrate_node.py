"""integrate_node 线程交接层测试：收包线程 -> 最新帧队列 -> timer 决策。

不碰真实网络：udp_port=0（绑定临时端口，永远收不到包），
直接向 _submit() 投递 WorldSnapshot，再 tick 定时器断言发布内容。
"""

import ros_stub  # noqa: F401  必须先于 integrate_node 导入

import math

import pytest

from ag_integrate.integrate_node import IntegrateNode
from ag_integrate.intel_protocol import (
    CarPose, IntelFrame, IntelObject, ObjectType, build, build_car_pose,
)
from ag_integrate.world_model import WorldModel

@pytest.fixture(autouse=True)
def _overrides():
    """基线参数覆盖：在 fixture 里设置，避免模块 import 期就污染全局 _OVERRIDES
    （收集期副作用会让本文件之外的直接构造 IntegrateNode 的测试继承这里的值）。"""
    ros_stub.set_overrides(
        udp_port=0, dry_run=False, publish_reset=True,
        start_cell="N10", goal_cell="N11", start_heading="y", final_heading="x",
    )
    yield
    ros_stub.clear_overrides()


def mine_at(cell: str) -> IntelObject:
    letters = "ABCDEFGHIJKLMN"
    col = letters.index(cell[0])
    row = int(cell[1:]) - 1
    x = 0.065 + col * 0.35
    y = 0.295 + (10 - row) * 0.35
    return IntelObject(ObjectType.MINE, x, y, 0.35, 0.35)


def blue_target_at(cell: str) -> IntelObject:
    letters = "ABCDEFGHIJKLMN"
    col = letters.index(cell[0])
    row = int(cell[1:]) - 1
    x = 0.065 + col * 0.35
    y = 0.295 + (10 - row) * 0.35
    return IntelObject(ObjectType.TARGET_BLUE, x, y, 0.3, 0.3)


def obstacle_at(cell: str) -> IntelObject:
    letters = "ABCDEFGHIJKLMN"
    col = letters.index(cell[0])
    row = int(cell[1:]) - 1
    x = 0.065 + col * 0.35
    y = 0.295 + (10 - row) * 0.35
    return IntelObject(ObjectType.OBSTACLE, x, y, 0.35, 0.35)


def snapshot_of(*objects: IntelObject):
    model = WorldModel()
    return model.update(IntelFrame(1, tuple(objects)))


def make_node(**overrides) -> IntegrateNode:
    params = dict(
        udp_port=0, dry_run=False, publish_reset=True,
        # N11 是最底行，物理上无法朝北进入 -> 终点朝向用 x（东向驶入可达）
        start_cell="N10", goal_cell="N11", start_heading="y", final_heading="x",
    )
    params.update(overrides)
    ros_stub.set_overrides(**params)
    return IntegrateNode()


def test_latest_frame_wins_and_publishes_reset_then_route():
    node = make_node()
    try:
        # 两帧先后入队但未消费：决策只应看到最新一帧（E2 的雷）
        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5")))
        node._submit(snapshot_of(mine_at("E2"), blue_target_at("F5")))
        node.tick()

        pub = node.topic("/nav/waypoint")
        assert len(pub.published) == 2            # reset + route
        assert pub.published[0].data == "reset:N10,y"
        route = pub.published[1].data
        assert "E2" in route and "D2" not in route
        # 蓝靶 F5 不可踩；停靠格是其正交邻格（E5/F4/F6/G5 之一）带 *
        assert "F5" not in route
        stars = [t.rstrip("*") for t in route.split() if t.endswith("*")]
        assert stars and all(s in ("E5", "F4", "F6", "G5") for s in stars)
        assert route.endswith(",x")    # 终点朝向（最底行只能东/西驶入）
    finally:
        node.destroy_node()


def test_gate_hold_then_release_logs_once_each():
    node = make_node()
    try:
        # 无雷：暂不发车，不发布
        node._submit(snapshot_of(blue_target_at("F5")))
        node.tick()
        node.tick()
        assert node.topic("/nav/waypoint").published == []
        holds = [m for lvl, m in node.logger.records if "暂不发车" in m]
        assert len(holds) == 1          # 同一原因只记一次

        # 补上雷后放行
        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5")))
        node.tick()
        published = [m.data for m in node.topic("/nav/waypoint").published]
        assert published[0] == "reset:N10,y"
        assert "D2" in published[1]
    finally:
        node.destroy_node()


def test_dry_run_never_publishes():
    node = make_node(dry_run=True)
    try:
        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5")))
        node.tick()
        assert node.topic("/nav/waypoint").published == []
        assert any("[dry_run]" in m for lvl, m in node.logger.records)
    finally:
        node.destroy_node()


def test_duplicate_route_not_republished():
    node = make_node()
    try:
        snap = snapshot_of(mine_at("D2"), blue_target_at("F5"))
        node._submit(snap)
        node.tick()
        n_after_first = len(node.topic("/nav/waypoint").published)
        assert n_after_first == 2      # reset + route

        # 相同态势再来一帧：路线未变，不重复发布
        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5")))
        node.tick()
        assert len(node.topic("/nav/waypoint").published) == 2
    finally:
        node.destroy_node()


def test_replan_disabled_keeps_first_route():
    """replan_on_update=False（默认）：首条路线发出后，态势变化不重发。"""
    node = make_node()
    try:
        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5")))
        node.tick()
        assert len(node.topic("/nav/waypoint").published) == 2

        # 新识别到一颗雷：路线本应变化，但开关关 -> 不重发
        node._submit(snapshot_of(mine_at("D2"), mine_at("E2"), blue_target_at("F5")))
        node.tick()
        assert len(node.topic("/nav/waypoint").published) == 2
    finally:
        node.destroy_node()


def test_replan_enabled_republishes_changed_route():
    """replan_on_update=True：态势变化导致路线变化时自动重发新路线。"""
    node = make_node(replan_on_update=True)
    try:
        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5")))
        node.tick()
        first_route = node.topic("/nav/waypoint").published[1].data
        assert len(node.topic("/nav/waypoint").published) == 2

        # 新识别到一颗雷：路线变化 -> 重发（reset 不重复发）
        node._submit(snapshot_of(mine_at("D2"), mine_at("E2"), blue_target_at("F5")))
        node.tick()
        published = node.topic("/nav/waypoint").published
        assert len(published) == 3          # reset + 旧路线 + 新路线
        assert "E2" in published[2].data
        assert published[2].data != first_route
    finally:
        node.destroy_node()


def test_replan_guard_far_obstacle_no_replan():
    """重规划守卫：远处新障碍（不在路线 ±1 格内）不触发重算，路线不重发。"""
    node = make_node(replan_on_update=True)
    try:
        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5")))
        node.tick()
        assert len(node.topic("/nav/waypoint").published) == 2

        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5"),
                                 obstacle_at("A11")))   # 远角，与路线无关
        node.tick()
        assert len(node.topic("/nav/waypoint").published) == 2
    finally:
        node.destroy_node()


def test_replan_guard_route_obstacle_replans():
    """重规划守卫：新障碍落在路线上（雷格被堵）→ 触发重算并重发新路线。"""
    node = make_node(replan_on_update=True)
    try:
        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5")))
        node.tick()
        first_route = node.topic("/nav/waypoint").published[1].data
        assert "D2" in first_route

        # D2 雷位被障碍报告覆盖 -> 该雷不可达被剔除，路线必变
        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5"),
                                 obstacle_at("D2")))
        node.tick()
        published = node.topic("/nav/waypoint").published
        assert len(published) == 3
        assert published[2].data != first_route
    finally:
        node.destroy_node()


def test_car_pose_udp_frame_sets_true_yaw():
    """msg_id=0x4002 车位姿帧：接收并存储车的真实朝向。"""
    node = make_node(heading_bias_source="udp")
    try:
        assert node._car_true_yaw is None
        # 构造一帧 0x4002：车真实朝向 0°（x 方向）
        data = build_car_pose(CarPose(1, 1.0, 2.0, 0.0))
        node._process_packet(data)
        assert node._car_true_yaw == 0.0
    finally:
        node.destroy_node()


def test_external_yaw_picks_nearest_axis():
    """有外部朝向时，reset 断言选最近坐标轴。"""
    node = make_node(heading_bias_source="udp", start_heading="y")
    try:
        # 外部观测 yaw=0°（x 方向），断言 'y' 应修正为 'x'
        node._car_true_yaw = 0.0
        assert node._effective_start_heading() == "x"

        # 外部观测 yaw=90°，断言 'y' 保持
        node._car_true_yaw = math.pi / 2
        assert node._effective_start_heading() == "y"
    finally:
        node.destroy_node()


def test_no_udp_source_ignores_car_yaw():
    """heading_bias_source='none'（默认）时，不使用外部朝向。"""
    node = make_node(heading_bias_source="none", start_heading="y")
    try:
        node._car_true_yaw = 0.0  # 即使有 yaw 也忽略
        assert node._effective_start_heading() == "y"
    finally:
        node.destroy_node()


# ---------------- 朝向对齐（align 模式） ----------------

def test_align_mode_waits_until_aligned():
    """align 模式：门控通过后启动对齐，对齐完成前不发路线。"""
    node = make_node(heading_bias_source="align", start_heading="y")
    try:
        snap = snapshot_of(mine_at("D2"), blue_target_at("F5"))
        node._submit(snap)
        node._car_true_yaw = math.radians(158.6)  # 偏 68.6°
        node.tick()   # 启动对齐，但未完成

        # 对齐中：不发 reset/route，只发 cmd_vel
        assert node.topic("/nav/waypoint").published == []
        cmdvel = node.topic("/cmd_vel").published
        assert cmdvel and cmdvel[-1].angular.z != 0.0
    finally:
        node.destroy_node()


def test_align_completes_then_publishes_route():
    """align 模式：yaw 收敛到目标后，连续 settle 完成，发 reset+route。"""
    node = make_node(heading_bias_source="align", start_heading="y",
                     heading_align_settle_ticks=2)
    try:
        snap = snapshot_of(mine_at("D2"), blue_target_at("F5"))
        node._submit(snap)
        target = math.pi / 2  # 'y' = 90°

        # 第一帧：偏 68.6°，启动对齐
        node._car_true_yaw = math.radians(158.6)
        node.tick()
        assert node._align_state == "aligning"
        assert node.topic("/nav/waypoint").published == []

        # 后续 tick：yaw 已在目标容差内
        node._car_true_yaw = target
        for _ in range(5):
            node.tick()
        assert node._align_state == "aligned"

        # 对齐完成后再 tick 一次（带态势）触发路线发布
        node._submit(snap)
        node.tick()
        pub = node.topic("/nav/waypoint").published
        assert len(pub) == 2
        assert pub[0].data == "reset:N10,y"
    finally:
        node.destroy_node()


def test_align_publishes_zero_cmd_vel_when_within_tolerance():
    """yaw 在容差内时，发零角速度（停止）并累计 settle。"""
    node = make_node(heading_bias_source="align", start_heading="y")
    try:
        node._car_true_yaw = math.pi / 2  # 正好是目标
        node._start_heading_alignment()
        node._do_heading_alignment()
        last = node.topic("/cmd_vel").published[-1]
        assert last.angular.z == 0.0
        assert node._align_settle_count == 1
    finally:
        node.destroy_node()


def test_align_rotates_in_correct_direction():
    """真实朝向偏左/偏右时，角速度符号指向目标方向。"""
    node = make_node(heading_bias_source="align", start_heading="y")
    target = math.pi / 2
    try:
        # 真实 0°（东），目标 90°（北）：应逆时针（正角速度）
        node._car_true_yaw = 0.0
        node._start_heading_alignment()
        node._do_heading_alignment()
        assert node.topic("/cmd_vel").published[-1].angular.z > 0.0

        # 真实 180°，目标 90°：应顺时针（负角速度）
        node._car_true_yaw = math.pi
        node._do_heading_alignment()
        assert node.topic("/cmd_vel").published[-1].angular.z < 0.0
    finally:
        node.destroy_node()


def test_align_no_yaw_keeps_stopped():
    """未收到车位姿帧时，不发角速度（等待）。"""
    node = make_node(heading_bias_source="align", start_heading="y")
    try:
        node._car_true_yaw = None
        node._start_heading_alignment()
        node._do_heading_alignment()
        assert node.topic("/cmd_vel").published[-1].angular.z == 0.0
        assert node._align_state == "aligning"  # 仍在等
    finally:
        node.destroy_node()


def test_external_yaw_west_facing_picks_y_not_x():
    """朝西的车（yaw=180°）：最近轴是 y(90°差)，不是 x(180°差)。

    旧实现 `yaw∈[45°,135°)→y 否则 x` 在此场景错选 x，等效重新引入
    画圆根因（断言与真实朝向差 135°+）。
    """
    node = make_node(heading_bias_source="udp", start_heading="x")
    try:
        node._car_true_yaw = math.pi          # 正西
        assert node._effective_start_heading() == "y"
        node._car_true_yaw = math.radians(200)
        assert node._effective_start_heading() == "y"
        node._car_true_yaw = math.radians(270)  # 正南 -> x
        assert node._effective_start_heading() == "x"
    finally:
        node.destroy_node()


def test_align_timeout_fails_then_fallback_publishes():
    """align 超时分支：超时 -> failed -> 退回 udp 最近轴断言，仍照常发车。

    （总比完全不动好；FakeClock 每次 now() 推进 0.1s，timeout=0.5 几拍即触发。）
    """
    node = make_node(heading_bias_source="align", start_heading="y",
                     heading_align_timeout=0.5)
    try:
        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5")))
        node._car_true_yaw = 0.0   # 无人机观测车头朝东(0°)，与目标 'y' 差 90°
        node.tick()                # 启动对齐，转向中
        assert node._align_state == "aligning"
        assert node.topic("/nav/waypoint").published == []

        # 持续 tick 直到超时：状态置 failed，发零角速度停车，记 warning
        for _ in range(8):
            node.tick()
        assert node._align_state == "failed"
        assert any("朝向对齐超时" in m for _lvl, m in node.logger.records)
        assert node.topic("/cmd_vel").published[-1].angular.z == 0.0

        # 下一帧态势：failed 分支不再重对齐，退回最近轴断言照常发车
        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5")))
        node.tick()
        published = [m.data for m in node.topic("/nav/waypoint").published]
        assert published[0] == "reset:N10,x"   # 观测 0° -> 最近轴 'x'（而非目标 'y'）
        assert len(published) == 2
    finally:
        node.destroy_node()


def test_align_starts_on_car_pose_before_gate_passes():
    """对齐与侦察并行：0x4002 车位姿一到即启动对齐，不等情报门控（min_mines）。

    180s 硬顶下，对齐（最长 heading_align_timeout 秒）不能压在发车关键路径上。
    """
    node = make_node(heading_bias_source="align", start_heading="y", min_mines=8)
    try:
        assert node._align_state is None
        node._car_true_yaw = math.radians(158.6)   # 无人机已看到车
        node.tick()                                # 无任何态势帧（门控不过）
        assert node._align_state == "aligning"     # 但对齐已启动
        assert node.topic("/cmd_vel").published    # 且在发角速度
        assert node.topic("/nav/waypoint").published == []  # 路线仍等门控
    finally:
        node.destroy_node()


# ---------------- 启动自检 / 端口绑定（fail-fast） ----------------

def test_invalid_start_params_raise_at_startup():
    """非法 start_cell/goal_cell/start_heading：构造即抛错，不留到赛中才炸。"""
    with pytest.raises(ValueError):
        make_node(start_cell="N12")
    with pytest.raises(ValueError):
        make_node(goal_cell="O1")
    with pytest.raises(ValueError):
        make_node(start_heading="z")
    # 前导零/小写归一化后是合法格号，不炸
    node = make_node(start_cell="n10")
    node.destroy_node()


def test_udp_bind_failure_raises_at_startup(monkeypatch):
    """UDP 端口绑定失败：构造函数当场抛错（节点起不来、问题暴露），
    而不是收包线程静默死亡、节点假装就绪。"""
    import socket as _socket
    import types as _types

    import ag_integrate.integrate_node as m

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


# ---------------- 规划异常保护 ----------------

def test_build_mission_failure_keeps_node_alive(tmp_path):
    """build_mission 抛错（goal 不可达）：记 ROS 日志 + node.error 事件、
    不发路线、保持门控等下拍；节点存活，态势恢复后正常发车。"""
    from ag_core.blackbox import EV, read_events
    # 终点换成 M11：N10 是本测试的默认起点，对封锁免疫（上游规则：
    # 起/终点格上的障碍报告必为误检），堵它永远堵不出"不可达"。
    node = make_node(log_dir=str(tmp_path), goal_cell="M11")
    try:
        # 终点 M11 的三个入口 L11/N11/M10 全堵 -> build_mission 抛 ValueError
        node._submit(snapshot_of(
            mine_at("D2"), blue_target_at("F5"),
            obstacle_at("L11"), obstacle_at("N11"), obstacle_at("M10"),
        ))
        node.tick()
        assert node.topic("/nav/waypoint").published == []
        assert any("规划失败" in m for _lvl, m in node.logger.records)
        events, _bad = read_events(tmp_path)
        errs = [e for e in events if e.ev == EV.NODE_ERROR]
        assert errs and errs[-1].get("where") == "build_mission"

        # 节点存活：障碍消失的下拍恢复正常发车
        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5")))
        node.tick()
        published = [m.data for m in node.topic("/nav/waypoint").published]
        assert published[0] == "reset:N10,y"
        assert len(published) == 2
    finally:
        node.destroy_node()


# ---------------- 复盘事件 ----------------

def test_session_begin_and_reset_publish_events(tmp_path):
    """启动写 session.begin（含 dry_run/publish_reset/start/goal）；
    发 reset 命令时写 route.publish kind="reset"。"""
    from ag_core.blackbox import EV, read_events
    node = make_node(log_dir=str(tmp_path))
    try:
        events, _bad = read_events(tmp_path)
        begins = [e for e in events if e.ev == EV.SESSION_BEGIN]
        assert len(begins) == 1
        b = begins[0]
        assert b.get("dry_run") is False
        assert b.get("publish_reset") is True
        assert b.get("start") == "N10" and b.get("goal") == "N11"

        node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5")))
        node.tick()
        events, _bad = read_events(tmp_path)
        pubs = [e for e in events if e.ev == EV.ROUTE_PUBLISH]
        assert [e.get("kind") for e in pubs] == ["reset", "route"]
        assert pubs[0].get("cmd") == "reset:N10,y"
    finally:
        node.destroy_node()


def test_healthy_session_diagnose_has_no_critical(tmp_path):
    """端到端：真实节点跑一局健康局（收 0x4001 帧 -> 门控放行 -> 发 reset+route），
    产出的事件流跑 diagnose 必须零 critical 误报。

    这是"消费侧对齐现役栈"的定盘星：规则若还在引用已删除的旧事件
    （intel.stable / car_node），这里会立刻冒出假 critical。
    """
    from ag_core.blackbox import EV, read_events
    from ag_core.log_diagnosis import diagnose, summarize
    node = make_node(log_dir=str(tmp_path))
    try:
        # 走真实收包路径：解析 -> INTEL_RECV（限流首次必放行）-> 入队
        node._process_packet(build(IntelFrame(1, (mine_at("D2"), blue_target_at("F5")))))
        node.tick()
        assert len(node.topic("/nav/waypoint").published) == 2
    finally:
        node.destroy_node()

    events, bad = read_events(tmp_path)
    assert bad == 0
    kinds = {e.ev for e in events}
    assert {EV.SESSION_BEGIN, EV.NODE_ARGS, EV.INTEL_RECV,
            EV.PLAN_BUILT, EV.ROUTE_PUBLISH} <= kinds
    m = summarize(events)
    assert m["intel_recv"] == 1 and m["reset_sent"] == 1 and m["route_sent"] == 1
    assert m["dry_run"] is False and m["publish_reset"] is True
    assert m["shoot_stops"]          # plan.built 的 shoot= 被正确读出
    criticals = [f for f in diagnose(events) if f.severity == "critical"]
    assert criticals == [], [f"{f.code}: {f.title}" for f in criticals]


def test_red_veto_ttl_param_wired_to_world_model():
    """red_veto_ttl_s 节点参数透传到世界模型（红靶粘性否决 TTL）。"""
    node = make_node(red_veto_ttl_s=9.5)
    try:
        assert node.model.red_veto_ttl_s == 9.5
    finally:
        node.destroy_node()


# ---------------- 黑匣子软依赖（ag_core 缺失） ----------------

def test_node_survives_when_ag_core_missing():
    """ag_core 缺失时占位命名空间兜底：收包/决策路径里 EV.XXX 实参求值
    不再 NameError，节点照常完成发车，黑匣子事件静默丢弃。"""
    import importlib
    import sys

    import ag_integrate.integrate_node as m

    saved = sys.modules.get("ag_core.blackbox")
    sys.modules["ag_core.blackbox"] = None   # 触发 import 处的 ImportError
    try:
        importlib.reload(m)
        assert not m._HAS_BB
        assert m.EV.NODE_ARGS == "" and m.AIM_BLOCK_WHY.GATE_OFF == ""

        ros_stub.set_overrides(
            udp_port=0, dry_run=False, publish_reset=True,
            start_cell="N10", goal_cell="N11", start_heading="y", final_heading="x",
        )
        node = m.IntegrateNode()
        try:
            assert node.bb is None
            # 坏帧路径（EV.LINK_DROP 作为实参求值）不炸
            good = build(IntelFrame(1, (mine_at("D2"),)))
            bad = bytearray(good)
            bad[-6] ^= 0xFF    # 破坏 CRC，触发 LINK_DROP 分支
            node._process_packet(bytes(bad))
            # 正常发车路径照常工作
            node._submit(snapshot_of(mine_at("D2"), blue_target_at("F5")))
            node.tick()
            assert len(node.topic("/nav/waypoint").published) == 2
        finally:
            node.destroy_node()
    finally:
        if saved is None:
            del sys.modules["ag_core.blackbox"]
        else:
            sys.modules["ag_core.blackbox"] = saved
        importlib.reload(m)   # 恢复真实 EV，不污染其它测试
