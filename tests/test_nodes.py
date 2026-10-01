"""节点层集成测试（用 ros_stub 替身，无需 ROS 环境）。

覆盖以前测不到的部分：参数读取、话题接线、回调状态机、发布出去的消息内容。
"""

import json

import numpy as np
import pytest

import ros_stub  # noqa: F401  必须在导入节点模块之前安装替身
from ag_core.homography import field_corners, homography_from_correspondences
from ag_interfaces.msg import (  # noqa: E402
    Detection,
    DetectionArray,
    FieldIntelMsg,
    GimbalCommand,
    RefereeStateMsg,
)
from std_msgs.msg import Int32, String, UInt8MultiArray  # noqa: E402

from ag_bridge.air_link_node import AirLinkNode  # noqa: E402
from ag_car.car_node import CarNode  # noqa: E402
from ag_chassis.chassis_node import ChassisNode  # noqa: E402
from ag_bridge.fake_intel_node import FakeIntelNode  # noqa: E402
from ag_bridge.ground_link_node import GroundLinkNode  # noqa: E402
from ag_mission.mission_node import MissionNode  # noqa: E402
from ag_mission.target_node import TargetNode  # noqa: E402
from ag_perception.intel_fusion_node import IntelFusionNode  # noqa: E402
from ag_referee.referee_node import RefereeNode  # noqa: E402


# ------------------------------------------------------------------ 辅助
def intel(seq, mines=("C3", "G8"), obstacles=("E5",), bridge_a="K4", bridge_b="K9"):
    return FieldIntelMsg(
        mines=list(mines),
        obstacles=list(obstacles),
        bridge_a=bridge_a,
        bridge_b=bridge_b,
        target_cells=["D2", "E2"],
        target_face="blue",
        seq=seq,
    )


def det(cls=0, score=0.95, cx=320.0, cy=240.0, kc=0.9, box=20.0):
    return Detection(
        cls=cls, score=score,
        x1=cx - box, y1=cy - box, x2=cx + box, y2=cy + box,
        kpt_x=cx, kpt_y=cy, kpt_conf=kc,
    )


def arr(*dets):
    return DetectionArray(detections=list(dets))


# ------------------------------------------------------------------ mission_node
def test_mission_node_ignores_single_frame():
    n = MissionNode()
    n.deliver("/link/field_intel", intel(1))
    assert n.topic("/mission/plan").published == []


def test_mission_node_plans_after_consistent_frames():
    n = MissionNode()
    n.deliver("/link/field_intel", intel(1))
    n.deliver("/link/field_intel", intel(2))
    published = n.topic("/mission/plan").published
    assert len(published) == 1
    data = json.loads(published[-1].data)
    assert data["tasks"][0]["type"] == "leave_start"
    assert data["tasks"][-1]["type"] == "goal"
    assert data["score"] > 0
    assert data["time_s"] <= 180.0


def test_mission_node_out_of_order_frame_dropped():
    n = MissionNode()
    n.deliver("/link/field_intel", intel(5))
    n.deliver("/link/field_intel", intel(3))  # 乱序，应被丢弃
    assert n.voter.frame_count == 1
    assert n.topic("/mission/plan").published == []


def test_mission_node_publishes_waypoint_and_advances():
    n = MissionNode()
    n.deliver("/link/field_intel", intel(1))
    n.deliver("/link/field_intel", intel(2))

    wp = json.loads(n.topic("/mission/waypoints").published[-1].data)
    assert wp["index"] == 0
    assert wp["type"] == "leave_start"
    assert wp["path"] == ["N10"]   # 车端实测默认摆放位 N10

    n.deliver("/mission/task_done", String(data="LEAVE_START"))
    wp2 = json.loads(n.topic("/mission/waypoints").published[-1].data)
    assert wp2["index"] == 1
    assert wp2["type"] != "leave_start"


def test_mission_node_ignores_unrelated_done_signal():
    n = MissionNode()
    n.deliver("/link/field_intel", intel(1))
    n.deliver("/link/field_intel", intel(2))
    before = n._index
    n.deliver("/mission/task_done", String(data="Z9"))
    assert n._index == before


def test_mission_node_keeps_done_tasks_on_replan():
    n = MissionNode()
    n.deliver("/link/field_intel", intel(1))
    n.deliver("/link/field_intel", intel(2))
    n.deliver("/mission/task_done", String(data="LEAVE_START"))

    # 新情报到达（新增一颗雷），已执行的 LEAVE_START 必须仍在计划首位
    n.deliver("/link/field_intel", intel(3, mines=("C3", "G8", "J12")))
    n.deliver("/link/field_intel", intel(4, mines=("C3", "G8", "J12")))
    plan = json.loads(n.topic("/mission/plan").published[-1].data)
    assert plan["tasks"][0]["type"] == "leave_start"
    assert n._index == 1


def test_mission_node_status_topic():
    n = MissionNode()
    n.tick()
    status = json.loads(n.topic("/mission/status").published[-1].data)
    assert status["total"] == 0
    assert status["confident"] is False


# ------------------------------------------------------------------ target_node
def test_target_node_needs_consensus_before_fire():
    n = TargetNode()
    for _ in range(2):
        n.deliver("/perception/detections", arr(det()))
    cmds = n.topic("/gimbal/command").published
    assert cmds[-1].fire is False
    assert "投票" in cmds[-1].reason

    n.deliver("/perception/detections", arr(det()))
    assert n.topic("/gimbal/command").published[-1].fire is True


def test_target_node_never_fires_on_red():
    n = TargetNode()
    for _ in range(5):
        n.deliver("/perception/detections", arr(det(cls=1)))
    cmds = n.topic("/gimbal/command").published
    assert all(c.fire is False for c in cmds)
    assert any("友方" in c.reason or "禁止" in c.reason for c in cmds)


def test_target_node_respects_fire_cooldown():
    n = TargetNode()
    for _ in range(3):
        n.deliver("/perception/detections", arr(det()))
    assert n.topic("/gimbal/command").published[-1].fire is True
    n.deliver("/perception/detections", arr(det()))
    last = n.topic("/gimbal/command").published[-1]
    assert last.fire is False
    assert "冷却" in last.reason


def test_target_node_syncs_referee_counters():
    n = TargetNode()
    n.deliver("/referee/state", RefereeStateMsg(ammo=6, shots=4, enemy_count=1))
    assert n.policy.shots_fired == 4
    assert n.policy.enemy_count == 1
    assert n.policy.ammo_left == 6


def test_target_node_disabled_fire_only_aims():
    ros_stub.set_overrides(enable_fire=False)
    try:
        n = TargetNode()
        for _ in range(4):
            n.deliver("/perception/detections", arr(det()))
        cmds = n.topic("/gimbal/command").published
        assert all(c.fire is False for c in cmds)
        assert any(c.yaw_delta != 0.0 or c.pitch_delta != 0.0 for c in cmds) is False
    finally:
        ros_stub.clear_overrides()


# ------------------------------------------------------------------ referee_node
def test_referee_node_publishes_state_and_counts_shots():
    n = RefereeNode()
    assert n.simulate is True  # Windows/CI 无串口，必然退化
    n.tick()
    st = n.topic("/referee/state").published[-1]
    assert st.ammo == 10 and st.shots == 0

    n.deliver("/gimbal/command", GimbalCommand(fire=True, reason="test"))
    n.tick()
    st = n.topic("/referee/state").published[-1]
    assert st.shots == 1 and st.ammo == 9


def test_referee_node_blocks_rapid_fire():
    n = RefereeNode()
    n.deliver("/gimbal/command", GimbalCommand(fire=True))
    n.deliver("/gimbal/command", GimbalCommand(fire=True))  # 冷却内，应被拒
    assert n.state.shots == 1


def test_referee_node_ignores_non_fire_command():
    n = RefereeNode()
    n.deliver("/gimbal/command", GimbalCommand(fire=False))
    assert n.state.shots == 0


# ------------------------------------------------------------------ intel_fusion_node
def _pixel_homography():
    src = np.array([[0.0, 0.0], [1100.0, 0.0], [1100.0, 1400.0], [0.0, 1400.0]])
    return homography_from_correspondences(src, field_corners(11, 14, 0.35))


def test_intel_fusion_node_maps_detections_to_cells():
    H = _pixel_homography()
    ros_stub.set_overrides(homography=list(H.flatten()), min_votes=1, publish_period_s=1.0)
    try:
        n = IntelFusionNode()
        assert n.grid.has_homography
        # 像素 (250, 260) 落在 C3 格内
        n.deliver("/perception/mines", arr(det(score=0.9, cx=250.0, cy=250.0, kc=0.0, box=5.0)))
        n.tick()
        out = n.topic("/air/field_intel").published[-1]
        assert out.mines == ["C3"]
        assert out.seq == 1
    finally:
        ros_stub.clear_overrides()


def test_intel_fusion_node_splits_target_classes():
    H = _pixel_homography()
    ros_stub.set_overrides(homography=list(H.flatten()), min_votes=1)
    try:
        n = IntelFusionNode()
        n.deliver("/perception/detections", arr(det(cls=1, cx=250.0, cy=250.0, kc=0.0)))
        n.tick()
        out = n.topic("/air/field_intel").published[-1]
        assert out.target_face == "red"
        assert out.target_cells == ["C3"]
    finally:
        ros_stub.clear_overrides()


def test_intel_fusion_node_without_homography_publishes_nothing():
    ros_stub.set_overrides(homography=[0.0] * 9, min_votes=1, only_when_ready=True)
    try:
        n = IntelFusionNode()
        n.deliver("/perception/mines", arr(det(score=0.9)))
        n.tick()
        assert n.topic("/air/field_intel").published == []
        assert any("单应矩阵无效" in r[1] for r in n.logger.records)
    finally:
        ros_stub.clear_overrides()


# ------------------------------------------------------------------ 链路节点
def _free_port() -> int:
    import socket

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_fake_intel_node_publishes_intel_and_detections():
    ros_stub.set_overrides(publish_detections=True, intel_topic="/link/field_intel")
    try:
        n = FakeIntelNode()
        n.tick()
        out = n.topic("/link/field_intel").published[-1]
        assert out.seq == 1
        assert out.bridge_a == "K4"
        dets = n.topic("/perception/detections").published[-1]
        assert len(dets.detections) == 1
        assert dets.detections[0].cls == 0
    finally:
        ros_stub.clear_overrides()


def test_fake_intel_node_without_detections_has_no_publisher():
    ros_stub.set_overrides(publish_detections=False)
    try:
        n = FakeIntelNode()
        n.tick()
        assert "/perception/detections" not in n.publishers
    finally:
        ros_stub.clear_overrides()


def test_link_nodes_roundtrip_over_udp():
    """机载端 -> UDP -> 车端，端到端验证情报一帧不丢、字段不串。"""
    pa, pb = _free_port(), _free_port()
    ros_stub.set_overrides(bind_port=pa, peer_ip="127.0.0.1", peer_port=pb,
                           intel_topic="/air/field_intel")
    try:
        air = AirLinkNode()
    finally:
        ros_stub.clear_overrides()
    ros_stub.set_overrides(bind_port=pb, peer_ip="127.0.0.1", peer_port=pa)
    try:
        ground = GroundLinkNode()
    finally:
        ros_stub.clear_overrides()

    try:
        air.deliver("/air/field_intel", intel(7, mines=("C3", "G8")))
        ground.tick()
        got = ground.topic("/link/field_intel").published
        assert len(got) == 1
        assert got[-1].mines == ["C3", "G8"]
        assert got[-1].seq == 7
        assert ground.topic("/link/link_stats").published  # 定时器有输出
    finally:
        air.link.close()
        ground.link.close()


# ------------------------------------------------------------------ chassis_node
def _wp(done=False, cell="C3", eta=3.0, index=1, type_="mine"):
    return String(data=json.dumps(
        {"done": done, "index": index, "type": type_, "cell": cell,
         "path": ["B2", "B3", cell], "score": 5, "eta_s": eta}))


def test_chassis_node_executes_and_reports():
    ros_stub.set_overrides(time_scale=100.0, tick_period_s=0.1)
    try:
        n = ChassisNode()
        n.deliver("/mission/waypoints", _wp())
        assert n.link.busy
        n.tick()
        done = n.topic("/mission/task_done").published
        assert done and done[-1].data == "C3"
        assert n.link.state.status == "arrived"
    finally:
        ros_stub.clear_overrides()


def test_chassis_node_stops_on_done_flag():
    ros_stub.set_overrides(time_scale=100.0)
    try:
        n = ChassisNode()
        n.deliver("/mission/waypoints", _wp())
        n.deliver("/mission/waypoints", _wp(done=True, cell=""))
        assert n.link.state.status == "aborted"
    finally:
        ros_stub.clear_overrides()


def test_chassis_node_ignores_bad_json():
    ros_stub.set_overrides(time_scale=100.0)
    try:
        n = ChassisNode()
        n.deliver("/mission/waypoints", String(data="not-json"))
        assert n.link.busy is False
        assert any("不是合法 JSON" in r[1] for r in n.logger.records)
    finally:
        ros_stub.clear_overrides()


def test_chassis_node_publishes_state():
    ros_stub.set_overrides(time_scale=100.0)
    try:
        n = ChassisNode()
        n.tick()
        st = json.loads(n.topic("/chassis/state").published[-1].data)
        assert st["status"] == "idle"
    finally:
        ros_stub.clear_overrides()


def test_chassis_node_nav2_mode_publishes_goal():
    ros_stub.set_overrides(mode="nav2", time_scale=100.0, goal_topic="/goal_pose",
                           frame_id="map", origin_x=1.0, origin_y=2.0, cell_size=0.35)
    try:
        n = ChassisNode()
        n.deliver("/mission/waypoints", _wp(cell="C3"))
        goals = n.topic("/goal_pose").published
        assert len(goals) == 1
        g = goals[-1]
        assert g.header.frame_id == "map"
        # C3 -> 栅格 (2.5*0.35, 2.5*0.35) + 原点 (1,2)
        assert g.pose.position.x == pytest.approx(1.0 + 2.5 * 0.35, abs=1e-6)
        assert g.pose.position.y == pytest.approx(2.0 + 2.5 * 0.35, abs=1e-6)
    finally:
        ros_stub.clear_overrides()


def test_chassis_node_direct_mode_publishes_no_goal():
    ros_stub.set_overrides(mode="direct", time_scale=100.0)
    try:
        n = ChassisNode()
        n.deliver("/mission/waypoints", _wp(cell="C3"))
        assert n.topic("/goal_pose").published == []
    finally:
        ros_stub.clear_overrides()


def test_referee_node_topic_source_receives_json():
    ros_stub.set_overrides(source="topic", state_json_topic="/referee/state_json",
                           shot_json_topic="/referee/shot_cmd")
    try:
        n = RefereeNode()
        n.deliver("/referee/state_json", String(
            data=json.dumps({"ammo": 6, "shots": 4, "enemy_count": 2,
                             "friend_count": 0, "illuminated": 0x10, "hp": 9})))
        assert n.state.ammo == 6
        assert n.state.enemy_count == 2
        n.tick()
        st = n.topic("/referee/state").published[-1]
        assert st.ammo == 6 and st.enemy_count == 2
    finally:
        ros_stub.clear_overrides()


def test_referee_node_topic_source_publishes_shot_json():
    ros_stub.set_overrides(source="topic", min_shot_interval_s=0.01)
    try:
        n = RefereeNode()
        n.deliver("/referee/state_json", String(data=json.dumps({"ammo": 10})))
        n.deliver("/gimbal/command", GimbalCommand(fire=True, reason="test"))
        cmds = n.topic("/referee/shot_cmd").published
        assert cmds and json.loads(cmds[-1].data)["cmd"] == "shot"
    finally:
        ros_stub.clear_overrides()


def test_referee_node_topic_source_ignores_bad_json():
    ros_stub.set_overrides(source="topic")
    try:
        n = RefereeNode()
        n.deliver("/referee/state_json", String(data="{bad"))
        assert any("不是合法 JSON" in r[1] for r in n.logger.records)
    finally:
        ros_stub.clear_overrides()


def test_chassis_node_real_mode_falls_back_with_warning():
    """实机模式但协议未对接时，应回退为计时推进而不是崩掉。"""
    ros_stub.set_overrides(simulate=False, time_scale=100.0, tick_period_s=0.1)
    try:
        n = ChassisNode()
        n.deliver("/mission/waypoints", _wp())
        n.tick()  # _send_path 抛错 -> 回退
        assert any("回退" in r[1] for r in n.logger.records)
        assert n.link.simulate is True
    finally:
        ros_stub.clear_overrides()


# ------------------------------------------------------------------ referee_node (dz 实机模式)
def test_referee_node_dz_mode_uses_official_topics():
    """实机上裁判串口由 dzjudgment 负责，本节点只转接话题、不碰串口。"""
    ros_stub.set_overrides(source="dz")
    try:
        n = RefereeNode()
    finally:
        ros_stub.clear_overrides()
    assert n.link.port is None                # 不开串口
    n.deliver("/HpAndHitmsg", UInt8MultiArray(data=[6, 10, 0x02, 4, 6]))
    n.tick()
    out = n.topic("/referee/state").published[-1]
    assert (out.ammo, out.shots, out.illuminated, out.hp) == (6, 4, 0x02, 10)
    assert n.target_hp == 6


def test_referee_node_dz_mode_short_frame_ignored():
    ros_stub.set_overrides(source="dz")
    try:
        n = RefereeNode()
    finally:
        ros_stub.clear_overrides()
    n.deliver("/HpAndHitmsg", UInt8MultiArray(data=[1, 2]))
    assert n.topic("/referee/state").published == []
    assert any("5 个字段" in r[1] for r in n.logger.records)


def test_referee_node_dz_mode_fire_publishes_pulse():
    """开火 = 在官方话题上发 1 再发 0（与 dzrknn_dataprocess 行为一致）。"""
    ros_stub.set_overrides(source="dz")
    try:
        n = RefereeNode()
    finally:
        ros_stub.clear_overrides()
    n.deliver("/HpAndHitmsg", UInt8MultiArray(data=[6, 10, 0x10, 0, 6]))
    n.deliver("/gimbal/command", GimbalCommand(fire=True, reason="test"))
    pulses = [m.data for m in n.topic("/LaserShot_Command").published]
    assert pulses[-2:] == [1, 0]


def test_referee_node_dz_mode_stops_when_out_of_ammo():
    ros_stub.set_overrides(source="dz", min_shot_interval_s=0.0)
    try:
        n = RefereeNode()
    finally:
        ros_stub.clear_overrides()
    n.deliver("/HpAndHitmsg", UInt8MultiArray(data=[6, 10, 0x10, 3, 0]))
    n.deliver("/gimbal/command", GimbalCommand(fire=True, reason="no ammo"))
    assert n.topic("/LaserShot_Command").published == []
    assert any("拒绝" in r[1] for r in n.logger.records)


# ------------------------------------------------------------------ car_node
def _car(**kw):
    ros_stub.set_overrides(**kw)
    try:
        return CarNode()
    finally:
        ros_stub.clear_overrides()


def _feed(node, times=2, **intel_kw):
    for i in range(1, times + 1):
        node.deliver("/link/field_intel", intel(i, **intel_kw))


def test_car_node_dry_run_publishes_nothing():
    """第一次上车必须能"只算不发"，这是默认值。"""
    n = _car()
    _feed(n)
    assert n.topic("/nav/waypoint").published == []
    assert n.last_route  # 但路线确实算出来了


def test_car_node_publishes_reset_then_route():
    n = _car(dry_run=False)
    _feed(n)
    sent = [m.data for m in n.topic("/nav/waypoint").published]
    assert sent[0] == "reset:N10,y"          # 先锚定里程计
    assert sent[-1].startswith("reset") is False
    assert "D2*" in sent[-1]                  # 打击点带 '*'
    assert sent[-1].endswith(",y")            # 终点朝向


def test_car_node_reset_sent_only_once():
    n = _car(dry_run=False)
    _feed(n)
    n.replan()
    n.replan()
    resets = [m.data for m in n.topic("/nav/waypoint").published if m.data.startswith("reset:")]
    assert len(resets) == 1


def test_car_node_does_not_repeat_same_route():
    n = _car(dry_run=False)
    _feed(n)
    before = len(n.topic("/nav/waypoint").published)
    n.replan()
    n.replan()
    assert len(n.topic("/nav/waypoint").published) == before


def test_car_node_waits_until_face_decided():
    """敌我未判决时不要发车（否则可能白跑一趟，也可能错过该打的靶）。"""
    n = _car(dry_run=False)
    for i in (1, 2):
        m = intel(i)
        m.target_cells = []
        m.target_face = ""
        n.deliver("/link/field_intel", m)
    assert n.topic("/nav/waypoint").published == []
    assert any("敌我尚未判决" in r[1] for r in n.logger.records)


def test_car_node_waits_for_min_mines():
    n = _car(dry_run=False, min_mines=3)
    for i in (1, 2):
        m = intel(i, mines=("C3", "G8"))
        n.deliver("/link/field_intel", m)
    assert n.topic("/nav/waypoint").published == []
    assert any("颗雷" in r[1] for r in n.logger.records)


def test_car_node_friendly_target_still_sends_route_without_star():
    """⚠️ 关键回归：靶是**友方（红面）**时也必须发车，只是不带 '*'。

    早期版本用「至少 1 个打击点」当出发条件，遇到友方靶会永远不发车 ——
    白丢排雷/桥洞/终点分。这个测试就是锁死这个行为。
    """
    n = _car(dry_run=False)
    for i in (1, 2):
        m = intel(i)
        m.target_face = "red"
        m.target_cells = ["D2", "E2"]
        n.deliver("/link/field_intel", m)
    sent = n.topic("/nav/waypoint").published
    assert sent, "友方靶时也必须出车"
    route = sent[-1].data
    assert "*" not in route                 # 绝不停靠射击
    assert n.face == "red"
    assert n.last_route == route


def test_car_node_enemy_target_marks_shoot_stop():
    n = _car(dry_run=False)
    _feed(n)                                # 默认 target_face="blue"
    route = n.topic("/nav/waypoint").published[-1].data
    assert route.count("*") == 1            # 弹药只够一个停靠点


def test_car_node_max_shoot_stops_is_enforced():
    n = _car(dry_run=False, max_shoot_stops=0)
    _feed(n)
    route = n.topic("/nav/waypoint").published[-1].data
    assert "*" not in route
    assert any("只保留前 0 个" in r[1] for r in n.logger.records)


def test_car_node_drops_shoot_stop_when_time_is_short():
    """往返时间已经很紧时，放弃 15 分打击分，优先保证回终点与计时。"""
    n = _car(dry_run=False, budget_s=70.0, goal_margin_s=10.0, speed_mps=0.35)
    _feed(n)
    route = n.topic("/nav/waypoint").published[-1].data
    assert "*" not in route
    assert any("时间不够" in r[1] for r in n.logger.records)


def test_car_node_keeps_shoot_stop_when_budget_allows():
    n = _car(dry_run=False, budget_s=180.0, shoot_wait_budget_s=30.0)
    _feed(n)
    assert n._fits_shoot_wait() is True
    assert "*" in n.topic("/nav/waypoint").published[-1].data


def test_car_node_accepts_expected_bridge_cells():
    """默认情报里的 K4 / K9 落在推断出的两处桥洞内，不应告警。"""
    n = _car(dry_run=False)
    _feed(n)                       # bridge_a=K4 bridge_b=K9
    assert n.bridge_check is not None and n.bridge_check.ok is True
    assert not any("桥洞情报可疑" in r[1] for r in n.logger.records)


def test_car_node_warns_on_suspicious_bridge_but_still_drives():
    """桥洞格号可疑时默认只告警（布点还是推断值，不能因此不发车）。"""
    n = _car(dry_run=False)
    for i in (1, 2):
        m = intel(i, bridge_a="D4", bridge_b="K9")   # D4 不在推断区域内
        n.deliver("/link/field_intel", m)
    assert n.topic("/nav/waypoint").published, "默认模式应照常发车"
    assert any("桥洞情报可疑" in r[1] for r in n.logger.records)


def test_car_node_strict_bridge_zone_blocks():
    n = _car(dry_run=False, strict_bridge_zone=True)
    for i in (1, 2):
        m = intel(i, bridge_a="D4", bridge_b="K9")
        n.deliver("/link/field_intel", m)
    assert n.topic("/nav/waypoint").published == []
    assert any("拒绝发车" in r[1] for r in n.logger.records)


def test_car_node_route_cells_are_all_legal():
    """发出去的路线必须能被车端 path_executor 解析（14 列 x 11 行）。"""
    from ag_car.route_builder import CarGrid

    n = _car(dry_run=False)
    _feed(n)
    route = n.topic("/nav/waypoint").published[-1].data
    g = CarGrid()
    for tok in route.replace(",y", "").replace(",x", "").split():
        g.parse(tok)  # 非法格号会抛 RouteError


def test_car_node_status_json():
    n = _car(dry_run=False)
    _feed(n)
    n.deliver("/shoot_count", Int32(data=7))
    n.tick()
    st = json.loads(n.topic("/car/route_status").published[-1].data)
    assert st["shots"] == 7
    assert st["mines"] == ["C3", "G8"]
    assert st["target_cells"] == ["D2", "E2"]
    assert st["reset_sent"] is True
    # 现场排查用：为什么标/没标 '*'
    assert st["face_decided"] == "blue"
    assert st["shoot_stops"] == ["D2"]
    assert st["fits_shoot_wait"] is True
    assert st["drive_eta_s"] > 0


def test_car_node_route_includes_bfs_intermediate_cells():
    """绕障 BFS 路径的中间格必须下发，不能只发任务格。

    纯追踪在相邻路点间走**直线**，而水桶障碍立在格中央：只发任务格，
    车就会直线横穿障碍格（比如 C3->K4 一路扫过 8 格）。
    旧的 build_command 只拼 t.cell.name，把 planner 算好的 t.path 丢掉了。
    """
    n = _car(dry_run=False)
    # 一颗雷 C3：路线从 N10 出发，第一格必须是 N10 的相邻格，
    # 而不是旧实现那样直接跳到十几格外的任务格
    _feed(n, mines=("C3",), obstacles=(), bridge_a=None, bridge_b=None)
    sent = [m.data for m in n.topic("/nav/waypoint").published]
    route = sent[-1]
    toks = route.replace(",y", "").split()
    assert toks[0] in ("N9", "M10"), f"路线的第一格应是 N10 的相邻格，实际: {toks[0]}（{route}）"
    assert "C3" in toks
    assert "D2*" in route
    assert route.endswith(",y")


if __name__ == "__main__":
    pytest.main([__file__])
