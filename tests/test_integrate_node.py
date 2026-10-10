"""integrate_node 线程交接层测试：收包线程 -> 最新帧队列 -> timer 决策。

不碰真实网络：udp_port=0（绑定临时端口，永远收不到包），
直接向 _submit() 投递 WorldSnapshot，再 tick 定时器断言发布内容。
"""

import ros_stub  # noqa: F401  必须先于 integrate_node 导入

import math

import pytest

from ag_integrate.integrate_node import IntegrateNode
from ag_integrate.intel_protocol import (
    CarPose, IntelFrame, IntelObject, ObjectType, build_car_pose,
)
from ag_integrate.world_model import WorldModel

ros_stub.set_overrides(
    udp_port=0, dry_run=False, publish_reset=True,
    start_cell="N10", goal_cell="N11", start_heading="y", final_heading="x",
)


@pytest.fixture(autouse=True)
def _clear_overrides():
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
