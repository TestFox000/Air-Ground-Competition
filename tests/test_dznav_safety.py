# -*- coding: utf-8 -*-
"""dznav 地面车安全回归测试。

这一组测试的存在理由：整改前 dznav（唯一会**物理移动**、能撞障/出图/误击
的模块）测试覆盖为零，而规划器有 15 个测试文件。以下每条都对应一个已在
代码里定位到具体行的失效路径，不是泛泛的"健壮性检查"。

覆盖：
  P0-3  命令出口 NaN/inf → 满舵（Python min/max 的 NaN 语义）
  P0-4  SIGTERM 不展开栈 → 归零失效
  P1-5  传感链路断流盲跑（原先只有 180s nav_timeout 兜底）
  P1-5  越界/急停闩锁语义、reset 解除边界
  P1-5  配置未知键静默丢弃、yaml 与 DEFAULTS 键漂移
"""

from __future__ import annotations

import math
import signal
from pathlib import Path

import pytest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Int32, String, UInt8

from dznav.contract import (FIELD_X_MAX, OOB_MARGIN, TOPIC_ESTOP, TOPIC_IMU,
                            TOPIC_ODOM_RAW, TOPIC_PURSUIT, TOPIC_SHOOT_COUNT,
                            TOPIC_STOP_SIGNAL, TOPIC_WAYPOINT)
from dznav.executor_node import (DEFAULTS, DzNavNode, _deep_update,
                                 _sigterm_handler, _zero_and_stop)

NAN = float("nan")
INF = float("inf")


# ---------------------------------------------------------------- 测试夹具
def _imu(yaw_rel: float, stamp: float | None = None) -> Imu:
    """由 yaw_rel 构造单位四元数（绕 Z）。"""
    m = Imu()
    m.orientation.x = 0.0
    m.orientation.y = 0.0
    m.orientation.z = math.sin(yaw_rel / 2.0)
    m.orientation.w = math.cos(yaw_rel / 2.0)
    if stamp is not None:
        m.header.stamp.sec = int(stamp)
        m.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
    return m


def _odom(x: float, y: float, stamp: float | None = None) -> Odometry:
    m = Odometry()
    m.pose.pose.position.x = x
    m.pose.pose.position.y = y
    if stamp is not None:
        m.header.stamp.sec = int(stamp)
        m.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
    return m


def _feed(node, yaw_rel: float = 0.0, ox: float = 0.0, oy: float = 0.0,
          stamp: float | None = None):
    """在当前节点时钟下注入一帧新鲜 odom + imu（喂饱链路看门狗）。"""
    if stamp is None:
        stamp = node.get_clock().now().nanoseconds * 1e-9
    node.inject(TOPIC_ODOM_RAW, _odom(ox, oy, stamp))
    node.inject(TOPIC_IMU, _imu(yaw_rel, stamp))


def _armed(node, route: str = "M9,y") -> None:
    """把节点推到 ACTIVE：喂传感器 → reset 重锚 → 下发路线。"""
    _feed(node)
    node.inject(TOPIC_WAYPOINT, String(data="reset:N10,y"))
    node.inject(TOPIC_WAYPOINT, String(data=route))


def _cmd(node) -> tuple[float, float]:
    last = node.publishers[TOPIC_PURSUIT].last
    return (last.linear.x, last.angular.z)


# =============================================================== P0-3 NaN
def test_python_nan_clamp_is_full_lock_canary():
    """金丝雀：记录 Python min/max 的 NaN 语义，说明为什么必须有出口闸。

    如果有人"简化"掉 executor_node._publish 或 pursuit._track 里的
    isfinite 检查，这条测试本身仍然通过，但下面几条会失败——保留它是
    为了给未来的读者留下因果链，而不是一句"别删"。
    """
    limit = 30.0
    # min(30.0, nan) -> 30.0（首参非 NaN 时返回首参），不是 nan！
    assert min(limit, NAN) == limit
    # 于是"钳位"把 NaN 变成了正向满舵
    assert max(-limit, min(limit, NAN)) == limit
    assert max(-limit, min(limit, NAN)) != 0.0


@pytest.mark.parametrize("bad", [NAN, INF, -INF])
def test_publish_rejects_nonfinite_and_latches_estop(bad):
    node = DzNavNode()
    node._publish(0.25, bad)
    assert _cmd(node) == (0.0, 0.0), "非有限命令必须落成 (0,0)"
    assert node.state == "ESTOP"
    assert node._estop_reason == "nan"
    assert node.publishers[TOPIC_STOP_SIGNAL].last.data == 0, \
        "急停必须收回云台放行位，否则进程还活着但云台继续开火"


@pytest.mark.parametrize("bad", [NAN, INF, -INF])
def test_publish_rejects_nonfinite_speed(bad):
    node = DzNavNode()
    node._publish(bad, 5.0)
    assert _cmd(node) == (0.0, 0.0)
    assert node.state == "ESTOP"


def test_publish_clamps_overrange_steer_without_estop():
    """幅值超限是可恢复的上游 bug，钳位 + 告警即可，不该把车锁死。"""
    node = DzNavNode()
    node._publish(0.2, 99.0)
    v, steer = _cmd(node)
    assert v == pytest.approx(0.2)
    assert steer == pytest.approx(DEFAULTS["servo_limit_deg"])
    assert node.state == "STANDBY"
    assert node.get_logger().text("warning", "超出行程")


def test_publish_clamps_overspeed():
    node = DzNavNode()
    node._publish(9.9, 0.0)
    assert _cmd(node)[0] == pytest.approx(DEFAULTS["v_max"])


def test_tracker_returns_zero_command_on_nan_pose():
    """定位崩掉时跟踪器必须停车，不能把 NaN 往下传。"""
    node = DzNavNode()
    _armed(node)
    assert node.tracker is not None
    cmd, _ev = node.tracker.update(NAN, 0.2, 0.0, 0.02, 0.0)
    assert cmd.v == 0.0 and cmd.steer_deg == 0.0
    cmd, _ev = node.tracker.update(0.2, INF, 0.0, 0.02, 0.0)
    assert cmd.v == 0.0 and cmd.steer_deg == 0.0


def test_tracker_rejects_nonpositive_dt():
    node = DzNavNode()
    _armed(node)
    cmd, _ev = node.tracker.update(0.5, 0.2, 0.0, 0.0, 0.0)
    assert cmd.v == 0.0 and cmd.steer_deg == 0.0


def test_control_publishes_zero_when_tracker_emits_nan(monkeypatch):
    """端到端：即便跟踪器漏出 NaN，出口闸也必须拦住并闩锁。"""
    node = DzNavNode()
    _armed(node)
    from dznav.pursuit import Command

    monkeypatch.setattr(node.tracker, "update",
                        lambda *a, **k: (Command(0.3, NAN), []))
    node._control()
    assert _cmd(node) == (0.0, 0.0)
    assert node.state == "ESTOP"
    assert node._estop_reason == "nan"
    assert node.tracker is None, "急停必须丢弃路线，杜绝 resume 到旧 tracker"


# =============================================================== P0-4 退出
def test_sigterm_handler_raises_systemexit():
    """SIGTERM 必须转成 SystemExit，否则 finally 里的归零不会执行。

    dznav_deploy.py:143,158 用 pkill（SIGTERM）收车；SIGTERM 的默认处置
    是直接终止、不展开 Python 栈。
    """
    with pytest.raises(SystemExit):
        _sigterm_handler(signal.SIGTERM, None)


def test_zero_and_stop_publishes_zero_throttle_and_revokes_fire():
    node = DzNavNode()
    node.publishers[TOPIC_STOP_SIGNAL].sent.append(UInt8(data=1))
    moving = Twist()
    moving.linear.x = 0.3
    moving.angular.z = 12.0
    node.publishers[TOPIC_PURSUIT].sent.append(moving)
    _zero_and_stop(node)
    assert _cmd(node) == (0.0, 0.0)
    assert node.publishers[TOPIC_STOP_SIGNAL].last.data == 0
    # BEST_EFFORT/depth=1 有丢包可能，重发是廉价保险
    assert len(node.publishers[TOPIC_PURSUIT].sent) >= 3


# =============================================================== P1-5 链路
def test_link_watchdog_latches_estop_while_moving():
    node = DzNavNode()
    _armed(node)
    assert node.state == "ACTIVE"
    node.get_clock().advance(DEFAULTS["link_timeout"] + 0.1)
    node._control()
    assert node.state == "ESTOP"
    assert node._estop_reason == "link"
    assert _cmd(node) == (0.0, 0.0)
    assert node.publishers[TOPIC_STOP_SIGNAL].last.data == 0


def test_link_watchdog_survives_on_fresh_input():
    node = DzNavNode()
    _armed(node)
    for _ in range(20):
        _feed(node)
        node.get_clock().advance(0.02)
        node._control()
        assert node.state != "ESTOP", "喂了新鲜数据不该跳闸"


def test_link_watchdog_silent_in_standby():
    """待命期传感器没接不该把车锁死——否则上电顺序变成硬约束。"""
    node = DzNavNode()
    node.get_clock().advance(60.0)
    node._control()
    assert node.state == "STANDBY"


def test_watchdog_timeout_is_far_below_nav_timeout():
    """看门狗必须比 nav_timeout 短一个量级以上，否则等于没有。"""
    assert DEFAULTS["link_timeout"] * 20 < DEFAULTS["nav_timeout"]


# =============================================================== 急停闩锁
def test_estop_topic_latches_and_survives_reset():
    """/estop=True 只能由 /estop=False 解除。

    让 reset: 能解除操作员急停是 fail-dangerous 的：现场慌乱中一条重锚
    命令就会悄悄放行走错方向的车。
    """
    node = DzNavNode()
    node.inject(TOPIC_ESTOP, Bool(data=True))
    assert node.state == "ESTOP"
    assert node._estop_reason == "cmd"
    _feed(node)
    node.inject(TOPIC_WAYPOINT, String(data="reset:N10,y"))
    assert node.state == "ESTOP", "reset 不得解除 /estop 闩锁"
    assert node.get_logger().text("error", "/estop 仍闩锁")
    node.inject(TOPIC_ESTOP, Bool(data=False))
    assert node.state == "STANDBY"


def test_estop_topic_blocks_new_route():
    node = DzNavNode()
    _feed(node)
    node.inject(TOPIC_ESTOP, Bool(data=True))
    node.inject(TOPIC_WAYPOINT, String(data="reset:N10,y"))
    node.inject(TOPIC_WAYPOINT, String(data="M9,y"))
    assert node.state == "ESTOP"
    assert node.tracker is None


def test_estop_subscription_uses_default_volatile_qos():
    """S3：/estop 订阅必须是默认 QoS（VOLATILE durability）。

    旧代码用 TRANSIENT_LOCAL 订阅：与任何默认 VOLATILE 发布者
    （ros2 topic pub /estop、上位机脚本）durability 不兼容 = 零投递，
    急停端到端不成立。闩锁由 dznav 软件层自持（_estop_cmd，见上面两条
    闩锁测试），不依赖 DDS 滞留，订阅侧因此不需要也不许再用
    TRANSIENT_LOCAL。
    """
    node = DzNavNode()
    sub = node.sub_for(TOPIC_ESTOP)
    # 传 int 深度 = rclpy 默认 QoS（RELIABLE + VOLATILE + KEEP_LAST）。
    # 桩下 QoSProfile 是 auto-vivify 替身，getattr 探 durability 恒真，
    # 所以这里认"int 深度"这一可判据形态，不认对象属性。
    assert isinstance(sub.qos, int), \
        "急停订阅应传默认深度（如 10），不得再构造 TRANSIENT_LOCAL QoSProfile"


def test_estop_volatile_subscription_receives_default_publisher_real_dds():
    """真实 QoS 语义端到端：默认 VOLATILE 发布者的 /estop 必须送达。

    桩环境没有 DDS 传输语义，这条在桩下 skip 而不是假装通过——
    S3 的失效（TRANSIENT_LOCAL 订阅收不到 VOLATILE 发布者）只有在
    真 DDS 下才能证伪/证真。
    """
    rclpy = pytest.importorskip("rclpy")
    if getattr(rclpy, "__file__", None) is None:
        pytest.skip("桩 rclpy 无 DDS 传输语义，不假装通过")
    import time as _time

    rclpy.init(args=None)
    node = DzNavNode()
    pub_node = rclpy.create_node("estop_test_pub")
    pub = pub_node.create_publisher(Bool, TOPIC_ESTOP, 10)  # 默认 VOLATILE
    try:
        deadline = _time.monotonic() + 5.0
        while _time.monotonic() < deadline and not node._estop_cmd:
            pub.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=0.1)
        assert node._estop_cmd, "默认 QoS 发布者的 /estop=True 必须送达并闩锁"
        while _time.monotonic() < deadline and node._estop_cmd:
            pub.publish(Bool(data=False))
            rclpy.spin_once(node, timeout_sec=0.1)
        assert not node._estop_cmd, "/estop=False 必须解除闩锁"
    finally:
        node.destroy_node()
        pub_node.destroy_node()
        rclpy.shutdown()


def test_reset_clears_link_latch():
    node = DzNavNode()
    _armed(node)
    node.get_clock().advance(5.0)
    node._control()
    assert node.state == "ESTOP" and node._estop_reason == "link"
    _feed(node)
    node.inject(TOPIC_WAYPOINT, String(data="reset:N10,y"))
    assert node.state == "STANDBY"
    # reset 必须重开一个完整看门狗窗口：重新放行后第一拍不许立刻再跳闸
    node.inject(TOPIC_WAYPOINT, String(data="M9,y"))
    assert node.state == "ACTIVE"
    node._control()
    assert node.state == "ACTIVE"


def test_out_of_bounds_latches_and_reset_clears():
    node = DzNavNode()
    _armed(node)
    node.loc.x = FIELD_X_MAX + OOB_MARGIN + 0.5
    _feed(node)
    node._control()
    assert node.state == "ESTOP"
    assert node._estop_reason == "oob"
    node.inject(TOPIC_WAYPOINT, String(data="reset:N10,y"))
    assert node.state == "STANDBY"


# =============================================================== 放行门禁
def test_route_refused_without_sensors():
    """没有 IMU 就没有航向，跟踪器的 e_th 是垃圾——绝不能放行。"""
    node = DzNavNode()
    node.inject(TOPIC_WAYPOINT, String(data="M9,y"))
    assert node.state == "STANDBY"
    assert node.tracker is None
    assert node.get_logger().text("error", "尚未收到")


def test_route_refused_with_only_one_sensor():
    node = DzNavNode()
    node.inject(TOPIC_IMU, _imu(0.0, 1.0))
    node.inject(TOPIC_WAYPOINT, String(data="M9,y"))
    assert node.state == "STANDBY"
    assert node.get_logger().text("error", "/odom")


def test_route_accepted_after_both_sensors_seen():
    node = DzNavNode()
    _armed(node)
    assert node.state == "ACTIVE"
    assert node.tracker is not None


# =============================================================== 输入卫生
def test_nonfinite_odom_position_dropped():
    node = DzNavNode()
    node.inject(TOPIC_ODOM_RAW, _odom(NAN, 0.0, 1.0))
    assert node._odom_seen is False
    node.inject(TOPIC_ODOM_RAW, _odom(0.0, 0.0, 2.0))
    assert node._odom_seen is True


def test_out_of_order_odom_dropped():
    node = DzNavNode()
    node.inject(TOPIC_ODOM_RAW, _odom(0.0, 0.0, 10.0))
    node.inject(TOPIC_ODOM_RAW, _odom(1.0, 1.0, 5.0))
    assert node._odom_last_stamp == pytest.approx(10.0), \
        "旧戳帧必须被丢弃，否则里程会来回跳"


def test_out_of_order_imu_dropped():
    node = DzNavNode()
    node.inject(TOPIC_IMU, _imu(0.0, 10.0))
    node.inject(TOPIC_IMU, _imu(1.0, 4.0))
    assert node._imu_last_stamp == pytest.approx(10.0)
    assert node.loc.yaw_rel == pytest.approx(0.0)


def test_zero_quaternion_imu_dropped():
    node = DzNavNode()
    m = Imu()
    m.orientation.x = 0.0
    m.orientation.y = 0.0
    m.orientation.z = 0.0
    m.orientation.w = 0.0
    m.header.stamp.sec = 1
    node.inject(TOPIC_IMU, m)
    assert node._imu_seen is False, "|q|=0 无法解出航向，必须丢弃"


def test_missing_stamp_degrades_gracefully():
    """戳为 0/缺失时退化成不做乱序检查，不能因此丢帧。"""
    node = DzNavNode()
    node.inject(TOPIC_IMU, _imu(0.3))
    node.inject(TOPIC_IMU, _imu(0.4))
    assert node._imu_seen is True
    assert node._imu_last_stamp is None
    assert node.loc.yaw_rel == pytest.approx(0.4)


# =============================================================== 时钟
def test_dt_comes_from_real_clock():
    node = DzNavNode()
    node.get_clock().advance(0.05)
    node._control()
    assert node._dt == pytest.approx(0.05, abs=1e-6)


def test_dt_clamped_after_process_stall():
    """进程被挂起 5s 后，绝不能用 dt=5 让加速度限幅失效。"""
    node = DzNavNode()
    node.get_clock().advance(5.0)
    node._control()
    assert node._dt == pytest.approx(DEFAULTS["dt_max"])


def test_dt_falls_back_to_nominal_on_zero_or_negative():
    node = DzNavNode()
    node._control()                       # 未推进时钟 → dt == 0
    assert node._dt == pytest.approx(0.02)


def test_stall_triggers_link_watchdog_not_silent_recovery():
    """挂起后 dt 被钳住，但墙上时钟照走 → 看门狗必须跳闸。"""
    node = DzNavNode()
    _armed(node)
    node.get_clock().advance(5.0)
    node._control()
    assert node.state == "ESTOP"
    assert node._estop_reason == "link"


# =============================================================== 配置
def test_deep_update_warns_on_unknown_keys():
    logs: list[str] = []
    base = {"v_max": 0.3}
    _deep_update(base, {"v_max": 0.4, "alat_max": 0.9}, log=logs.append)
    assert base == {"v_max": 0.4}
    assert logs and "alat_max" in logs[0], \
        "拼错的键必须喊出来，静默丢弃会让现场调参变成玄学"


def test_deep_update_silent_without_logger():
    base = {"a": 1}
    assert _deep_update(base, {"a": 2, "b": 3}) == {"a": 2}


def test_shipped_yaml_keys_match_defaults():
    """dznav_default.yaml 的键必须与 DEFAULTS 一一对应。

    双向检查：yaml 多出来的键会被静默忽略（现场调参变玄学），
    DEFAULTS 多出来的键意味着 yaml 不再是完整的参数清单。
    """
    yaml = pytest.importorskip("yaml")
    path = (Path(__file__).resolve().parents[1] / "src" / "dznav" / "config"
            / "dznav_default.yaml")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert not sorted(set(data) - set(DEFAULTS)), \
        f"yaml 有 DEFAULTS 不认识的键: {sorted(set(data) - set(DEFAULTS))}"
    assert not sorted(set(DEFAULTS) - set(data)), \
        f"yaml 缺 DEFAULTS 里的键: {sorted(set(DEFAULTS) - set(data))}"


def test_yaml_loaded_via_env(monkeypatch, tmp_path):
    pytest.importorskip("yaml")
    cfg = tmp_path / "c.yaml"
    cfg.write_text("v_max: 0.11\nlink_timeout: 0.9\nbogus_key: 1\n",
                   encoding="utf-8")
    monkeypatch.setenv("DZ_NAV_CONFIG", str(cfg))
    node = DzNavNode()
    assert node.cfg["v_max"] == pytest.approx(0.11)
    assert node.cfg["link_timeout"] == pytest.approx(0.9)
    assert node.get_logger().text("warning", "bogus_key")


def test_missing_config_file_is_reported(monkeypatch, tmp_path):
    monkeypatch.setenv("DZ_NAV_CONFIG", str(tmp_path / "nope.yaml"))
    node = DzNavNode()
    assert node.get_logger().text("error", "不存在")
    assert node.cfg["v_max"] == DEFAULTS["v_max"]


# =============================================================== 其它
def test_shoot_count_tracked():
    node = DzNavNode()
    node.inject(TOPIC_SHOOT_COUNT, Int32(data=7))
    assert node.shoot_count == 7


def test_bad_reset_syntax_rejected():
    node = DzNavNode()
    node.inject(TOPIC_WAYPOINT, String(data="reset:ZZ9,q"))
    assert node.get_logger().text("error", "reset 格式错")


def test_localization_belief_published_every_tick():
    node = DzNavNode()
    node._control()
    node._control()
    assert len(node.publishers["/odom/localization"].sent) == 2


# =============================================================== 2026-10-18 评审整改回归
def test_auto_mission_fires_once_and_ignores_retrigger(monkeypatch):
    """S1 回归：自动发车只触发一次（起栈 20s 内周期定时器重触发不得重发）。

    旧代码 create_timer(auto_delay, _auto_mission) 是无守卫周期定时器：
    DZ_AUTO_START=1 时每 8s 重建 tracker 重发内置路线——行驶中急停重启、
    覆盖上游 replan 路线、打断射击/ARC。
    """
    monkeypatch.setenv("DZ_AUTO_START", "1")
    node = DzNavNode(overrides={"builtin_route": "M9,y", "auto_delay": 8.0})
    _feed(node)
    node.inject(TOPIC_WAYPOINT, String(data="reset:N10,y"))
    node._auto_mission()                  # t=8s 第一次触发：发车
    assert node.state == "ACTIVE"
    # 上游 replan 改路线（比赛中的动态重规划），随后定时器第 2、3 次触发
    node.inject(TOPIC_WAYPOINT, String(data="M8,y"))
    tracker_after_replan = node.tracker
    assert tracker_after_replan is not None
    node._auto_mission()                  # t=16s
    node._auto_mission()                  # t=24s
    assert node.tracker is tracker_after_replan, \
        "自动发车重发会重建 tracker、覆盖 replan 后的路线"
    assert len(node.get_logger().text("warning", "自动发车")) == 1


def test_reset_during_motion_publishes_zero_frames():
    """S2 回归：行驶中 reset → 静默前连发零 Twist，下一拍命令即 0。

    转入待命静默后，dzactuator 要等 CMD_WATCHDOG_S=1.0s 无指令才强制
    停车，旧命令会继续执行 ~1s（0.30m/s 下 ~30cm 盲滑，图缘足够出格）。
    reset 必须主动把零命令顶进看门狗窗口。
    """
    node = DzNavNode()
    _armed(node)
    node._control()
    v, s = _cmd(node)
    assert v != 0.0 or s != 0.0, "前置：车确实在动"
    pub = node.publishers[TOPIC_PURSUIT]
    pub.clear()
    node.inject(TOPIC_WAYPOINT, String(data="reset:N10,y"))
    assert node.state == "STANDBY"
    assert len(pub.sent) >= 3, "BEST_EFFORT/depth=1 有丢包可能，须重发"
    assert all(m.linear.x == 0.0 and m.angular.z == 0.0 for m in pub.sent)
    # 下一拍（STANDBY 静默）不再发非零命令：执行器看到的最后命令是 0
    node._control()
    assert pub.last.linear.x == 0.0 and pub.last.angular.z == 0.0
    assert node.publishers[TOPIC_STOP_SIGNAL].last.data == 0


def test_pending_anchor_dropped_on_route_switch():
    """M1 回归：换路线必须作废旧路线的出弯延迟锚点。

    旧代码 _start_route 直接覆盖 tracker，_pending_anchor 跨路线残留：
    新 tracker 的 s_prog 越过旧 s_target 时，信念被拉向已废弃路径上的点。
    """
    node = DzNavNode()
    _armed(node, "M9,y")
    node._pending_anchor = 0.0            # 旧路线遗留锚点（下一拍即触发）
    node.inject(TOPIC_WAYPOINT, String(data="M8,y"))
    assert node._pending_anchor is None, "换路线必须丢弃旧锚点"
    anchored = []
    orig_anchor = node.loc.anchor
    node.loc.anchor = lambda *a, **k: (anchored.append(a),
                                       orig_anchor(*a, **k))[1]
    for _ in range(10):
        _feed(node)
        node._control()
    assert not anchored, "旧锚点不得在新路线上作祟"


def test_load_route_starts_from_belief_pose(tmp_path):
    """M2 回归：load: 起点必须是当前信念位姿（米制）。

    旧代码用 START_BLOCK（格单位）当米用，起点被错钉到 (1.5, 0.5)m
    而非信念位置——整段参考路径从错误位置生成。
    """
    node = DzNavNode()
    _feed(node)
    node.inject(TOPIC_WAYPOINT, String(data="reset:K5,x"))   # 信念不在 N10
    p = tmp_path / "route.txt"
    p.write_text("1.0,1.0,0\n2.0,2.0,0\n", encoding="utf-8")
    node.inject(TOPIC_WAYPOINT, String(data=f"load:{p}"))
    assert node.tracker is not None
    sx, sy, _ = node.loc.pose
    w0 = node.tracker.path.waypoints[0]
    assert (w0.x, w0.y) == pytest.approx((sx, sy), abs=1e-9)


def test_shoot_skip_event_logs_error_with_block_and_deviation():
    """M4：射击停站掠过必须 error 级可见（含格号与偏差）。"""
    node = DzNavNode()
    _armed(node)
    node._handle_event({"ev": "shoot_skipped", "idx": 1,
                        "label": "E9", "d": 0.21})
    msgs = node.get_logger().text("error", "掠过未触发")
    assert msgs and "E9" in msgs[0] and "0.21" in msgs[0]


def test_new_route_revokes_stop_signal():
    """M5：新路线开头必须收回云台放行位。

    路线中途换路线（replan/手动改单）时若不清 stop_signal，云台会以为
    仍在停站窗口，对着行进中的车继续开火。
    """
    node = DzNavNode()
    _feed(node)
    node.publishers[TOPIC_STOP_SIGNAL].sent.append(UInt8(data=1))  # 残留放行位
    node.inject(TOPIC_WAYPOINT, String(data="M9,y"))
    assert node.state == "ACTIVE"
    assert node.publishers[TOPIC_STOP_SIGNAL].last.data == 0


def test_nav_timeout_finish_does_not_release_gimbal():
    """M5：nav_timeout 强制收车不是到点，不得发 stop_signal=1。

    超时点不一定是射击位，放行等于让云台对空地开火。
    """
    node = DzNavNode()
    _armed(node)
    node.nav_start = node._t - DEFAULTS["nav_timeout"] - 1.0
    _feed(node)                           # 喂饱链路看门狗，隔离变量
    node._control()
    assert node.state == "ARRIVED"
    sigs = [m.data for m in node.publishers[TOPIC_STOP_SIGNAL].sent]
    assert 1 not in sigs, "超时收车放行云台 = 对空地开火"


def test_final_arc_probe_samples_arc_envelope():
    """M7 回归：终点朝向机动探针必须查弧包络中段，不只看出/入口两点。

    判别场景：西图缘 (0.10, 1.0)m 从 +y 转向 -x——出/入口都在 10cm
    容差带内（旧两点探针判可行），但满舵弧中段 x≈-0.17m 明显出图，
    新探针必须判不可行；同起点转向 +x（弧扫向图内）必须判可行。
    """
    node = DzNavNode()
    _armed(node, "M11 N11,x")
    # 把终点机动探针摆到西图缘（直接改末途经点 + 重锚信念位姿）
    node.tracker.path.waypoints[-1].x = 0.10
    node.tracker.path.waypoints[-1].y = 1.0
    node.final_heading = math.pi                   # 目标 -x（朝图外）
    node.loc.reset(0.10, 1.0, math.pi / 2)         # 当前朝 +y
    assert not node._final_arc_feasible(), "弧中段出图 17cm 必须判不可行"
    node.final_heading = 0.0                       # 目标 +x（弧扫向图内）
    assert node._final_arc_feasible()
    # 边界钉死：主证明场景 N11,x（-y→+x）弧中段 y≈-0.097m，在 10cm 贴边
    # 容差内必须放行——该场景靠终点机动把终点误差收进 0.35m
    node2 = DzNavNode()
    _armed(node2, "M11 N11,x")
    node2.final_heading = 0.0
    node2.loc.reset(0.175, 0.175, -math.pi / 2)
    assert node2._final_arc_feasible(), "N11,x 贴边弧（9.7cm）不得误伤"


def test_shoot_budget_matches_field_tuning():
    """S4：射击参数对齐现役调参（3 发/20s）。

    旧值 10 发/150s 抄自 legacy 过期 docstring：全场只有 10 发弹，
    每站等 10 发意味着第二站起永远干等到超时；150s 超时违背现役
    SHOOT_TIMEOUT=20s 的"宁可漏打不可卡死"原则。
    """
    assert DEFAULTS["shoot_target"] == 3
    assert DEFAULTS["shoot_timeout"] == pytest.approx(20.0)
    # 预算约束：每站发数 × 停站数 ≤ 弹药总量（全场 10 发）
    assert DEFAULTS["shoot_target"] * 3 <= 10
