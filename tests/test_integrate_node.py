"""integrate_node 线程交接层测试：收包线程 -> 最新帧队列 -> timer 决策。

不碰真实网络：udp_port=0（绑定临时端口，永远收不到包），
直接向 _submit() 投递 WorldSnapshot，再 tick 定时器断言发布内容。
"""

import ros_stub  # noqa: F401  必须先于 integrate_node 导入

import pytest

from ag_integrate.integrate_node import IntegrateNode
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType
from ag_integrate.world_model import WorldModel

ros_stub.set_overrides(
    udp_port=0, dry_run=False, publish_reset=True,
    start_cell="N10", goal_cell="N11", start_heading="y", final_heading="y",
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
        start_cell="N10", goal_cell="N11", start_heading="y", final_heading="y",
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
        assert "F5*" in route          # 蓝敌格允许停靠开火
        assert route.endswith(",y")    # 终点朝向
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
