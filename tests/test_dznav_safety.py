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
                            TOPIC_STOP_SIGNAL, TOPIC_WAYPOINT, wrap_pi)
from dznav.executor_node import (DEFAULTS, DzNavNode, ESTOP_QOS,
                                 ESTOP_QOS_VOLATILE, _deep_update,
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


def test_estop_subscribed_with_transient_local_qos():
    """TRANSIENT_LOCAL：晚启动的节点也要立刻收到已闩锁的急停。"""
    node = DzNavNode()
    sub = node.sub_for(TOPIC_ESTOP)
    assert sub.qos is ESTOP_QOS
    assert getattr(ESTOP_QOS, "durability", None) is not None, \
        "急停订阅必须声明 durability，否则 late-join 收不到闩锁状态"


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


# =============================================================== U2 循迹对标
def test_track_mode_switch_default_off():
    """U2 跟踪律开关：默认必须仍是 stanley（不改变现役行为），
    overrides 可切 pure_pursuit 且穿透到 TrackerConfig。"""
    node = DzNavNode()
    assert DEFAULTS["track_mode"] == "stanley"
    assert node.cfg["track_mode"] == "stanley"
    assert node.tcfg.track_mode == "stanley"
    node2 = DzNavNode(overrides={"track_mode": "pure_pursuit"})
    assert node2.tcfg.track_mode == "pure_pursuit"


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


# ============================================== 射击停站：弹药全局 vs 目标每站
class _FakeTracker:
    def __init__(self):
        self.resumed = 0

    def resume(self):
        self.resumed += 1


def _park_at_shoot_stop(node, *, count_at_arrival: int, now: float = 100.0):
    """把节点摆到"刚进射击停站"的那一刻，绕过路径跟踪只测等待逻辑。"""
    node.state = "SHOOT_WAIT"
    node.tracker = _FakeTracker()
    node.shoot_count = count_at_arrival
    node.shoot_at_arrival = count_at_arrival
    node.shoot_wait_start = now
    node._t = now


def test_shoot_wait_releases_when_global_ammo_exhausted():
    """第二个 '*' 停靠点在弹药打空后必须**立刻**放行，不能干等 shoot_timeout。

    事故链：shoot_target 是**每停靠点**增量（yaml 原注释就写着"每停站发弹数"），
    而弹药是全局物理量（10 发、不可补）。car_commands.py:103 给 shoot_cells 里
    **每一个**格子都打 '*'，所以路线可以有多个停靠点。第一站打空 10 发后，
    第二站的 (shoot_count - shoot_at_arrival) 永远是 0，追不上 shoot_target。
    而 SHOOT_WAIT 在 _control() 里提前 return，nav_timeout 兜底根本轮不到它
    —— 车原地干等满 shoot_timeout。旧值 150.0s 对 180s 赛时等于直接输掉比赛。
    """
    node = DzNavNode()
    _park_at_shoot_stop(node, count_at_arrival=int(DEFAULTS["ammo_total"]))
    node._shoot_wait_step()          # 刚到站，一发未打
    assert node.state == "ACTIVE", "弹药已空还死等，会把赛时烧光"
    assert node.tracker.resumed == 1
    assert node.get_logger().text("warning", "弹药已打空")


def test_shoot_wait_still_honours_per_stop_target_when_ammo_remains():
    """弹药充足时仍按每站目标数等待——修弹药洞不能顺手废掉正常语义。"""
    node = DzNavNode()
    _park_at_shoot_stop(node, count_at_arrival=0)
    node.shoot_count = int(DEFAULTS["shoot_target"]) - 1
    node._shoot_wait_step()
    assert node.state == "SHOOT_WAIT", "差一发就放行，等于没打满"
    node.shoot_count = int(DEFAULTS["shoot_target"])
    node._shoot_wait_step()
    assert node.state == "ACTIVE"


def test_shoot_wait_releases_when_final_ammo_spent():
    """最后一站把余弹打完即放行，不必等满 shoot_target。"""
    node = DzNavNode()
    _park_at_shoot_stop(node, count_at_arrival=int(DEFAULTS["ammo_total"]) - 3)
    node.shoot_count = int(DEFAULTS["ammo_total"])   # 只剩 3 发，全打完
    node._shoot_wait_step()
    assert node.state == "ACTIVE"


def test_shoot_timeout_is_bounded_well_below_match_budget():
    """shoot_timeout 必须远小于赛时，否则一次等不到弹就烧掉整局。

    SHOOT_WAIT 不受 nav_timeout 兜底（_control() 提前 return），所以
    shoot_timeout 是这段死等的**唯一**上限，必须自己守住量级。
    """
    assert DEFAULTS["shoot_timeout"] * 4 < DEFAULTS["nav_timeout"], (
        f"shoot_timeout={DEFAULTS['shoot_timeout']}s 相对 "
        f"nav_timeout={DEFAULTS['nav_timeout']}s 太大：路线里出现多个 '*' "
        f"停靠点时，死等会吃光赛时")


def test_shoot_timeout_covers_firing_all_ammo():
    """反向约束：shoot_timeout 也不能小到打不完弹就放行。

    车上 robot.yaml（2026-10-19 实测档）shot_cooldown_ms=800，打空 10 发
    物理上就要 8s。timeout 小于它 = 每局都"射击超时，继续路线"，白丢打击分。
    """
    SHOT_COOLDOWN_S = 0.8      # docs/成员交接/.../deliverable_20261019/robot.yaml:197
    need = DEFAULTS["ammo_total"] * SHOT_COOLDOWN_S
    assert DEFAULTS["shoot_timeout"] >= need, (
        f"shoot_timeout={DEFAULTS['shoot_timeout']}s < 打空全部弹药所需 {need}s")


def test_shipped_yaml_shoot_params_match_defaults():
    """yaml 里的弹药/超时三键必须与 DEFAULTS 一致，防单边改漂移。"""
    yaml = pytest.importorskip("yaml")
    path = (Path(__file__).resolve().parent.parent / "src" / "dznav" / "config"
            / "dznav_default.yaml")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    for k in ("shoot_target", "shoot_timeout", "ammo_total", "nav_timeout"):
        assert float(data[k]) == float(DEFAULTS[k]), f"{k}: yaml={data[k]} DEFAULTS={DEFAULTS[k]}"


# =================================================== 急停 QoS 兼容性
def test_estop_also_subscribed_with_volatile_qos():
    """除 TRANSIENT_LOCAL 外必须还有一条 VOLATILE reader。

    DDS Durability 兼容规则：writer 提供的等级必须 >= reader 请求的等级。
    TRANSIENT_LOCAL 的 reader 遇上 VOLATILE 的 writer **不匹配**——端点不建立、
    一条都收不到。/estop 的发布方在本仓之外，可能是官方车端节点
    （contract.py 注为 TRANSIENT_LOCAL），也可能是操作员手敲 `ros2 topic pub`
    （默认 VOLATILE）。只订 TRANSIENT_LOCAL 就是拿急停赌对端实现。

    注意：node.inject() 直接调回调，**绕过整个 DDS 层**，所以现有那组急停
    闩锁测试无论 QoS 怎么配都会通过——它们证明回调逻辑对，证明不了消息到得了。
    """
    node = DzNavNode()
    subs = [s for s in node.subscriptions if s.topic == TOPIC_ESTOP]
    durabilities = {getattr(s.qos, "durability", None) for s in subs}
    assert len(subs) >= 2, f"/estop 只订了 {len(subs)} 条，对端 VOLATILE 时收不到"
    assert ESTOP_QOS.durability in durabilities, "缺 late-join 补发闩锁的 TL reader"
    assert ESTOP_QOS_VOLATILE.durability in durabilities, \
        "缺兼容 VOLATILE 发布方的 reader——`ros2 topic pub` 打不进急停"
    for s in subs:
        # 绑定方法每次取属性都新建对象，只能比 __func__，不能用 is。
        assert s.callback.__func__ is DzNavNode._cb_estop, \
            "两条 reader 必须共用同一个闩锁回调"


def test_estop_latch_fires_through_volatile_reader():
    """经由 VOLATILE 那条 reader 送进来的急停也必须闩锁。"""
    node = DzNavNode()
    vol = [s for s in node.subscriptions
           if s.topic == TOPIC_ESTOP and s.qos is ESTOP_QOS_VOLATILE]
    assert vol, "没有 VOLATILE reader"
    _feed(node)
    vol[0].callback(Bool(data=True))
    assert node.state == "ESTOP"
    assert node._estop_reason == "cmd"
    vol[0].callback(Bool(data=False))
    assert node.state == "STANDBY"


# =================================================== 自动发车：一次性语义
def test_auto_mission_fires_exactly_once(monkeypatch):
    """DZ_AUTO_START=1 时 builtin_route 只能下发一次。

    rclpy 的 create_timer 是**周期**定时器。修复前 _auto_mission 没有任何闸，
    auto_delay=8.0 意味着每 8 秒重发一次路线：_cb_waypoint 重建 Tracker（航点
    served 全清零）、state 拉回 ACTIVE、nav_start 归零。180s 赛时里重发约 22 次，
    车每次只跑出 8s，永远到不了终点；射击停靠点反复重触发；而 nav_timeout 因为
    每次都被重新计时，连兜底都失效。
    """
    monkeypatch.setenv("DZ_AUTO_START", "1")
    node = DzNavNode(overrides={"builtin_route": "M9,y"})
    _feed(node)
    for _ in range(5):          # 模拟周期定时器连跳 5 次
        node._auto_mission()
    fired = node.get_logger().text("warning", "自动发车")
    assert len(fired) == 1, f"自动发车被触发 {len(fired)} 次，路线会被反复重发"


def test_auto_mission_timer_is_canceled_after_firing(monkeypatch):
    """发车后周期定时器必须被 cancel，别留一个永久空转的回调。"""
    monkeypatch.setenv("DZ_AUTO_START", "1")
    node = DzNavNode(overrides={"builtin_route": "M9,y"})
    assert node._auto_timer is not None, "DZ_AUTO_START=1 + builtin_route 未注册定时器"
    _feed(node)
    node._auto_mission()
    assert node._auto_timer.cancel_count == 1
    assert node._auto_timer.canceled is True


def test_auto_mission_timer_absent_when_disabled(monkeypatch):
    """默认（DZ_AUTO_START 未设）不得注册自动发车定时器——实车调试安全约定。"""
    monkeypatch.delenv("DZ_AUTO_START", raising=False)
    node = DzNavNode(overrides={"builtin_route": "M9,y"})
    assert node._auto_timer is None
    assert node._auto_started is False


def test_auto_mission_timer_absent_without_builtin_route(monkeypatch):
    """开了 DZ_AUTO_START 但没配路线：不得注册定时器（否则空发一条空路线）。"""
    monkeypatch.setenv("DZ_AUTO_START", "1")
    node = DzNavNode()
    assert node.cfg["builtin_route"] == ""
    assert node._auto_timer is None


# ============================================== fillet 切角 vs 17.5cm 失格线
def test_arc_fallback_deg_respects_dq_deviation_line():
    """arc_fallback_deg 不得超过"fillet 切角仍在失格线内"的最大转角。

    fillet 偏差 dev = R·(1/cos(θ/2) − 1)，而 R 会被物理下限 r_phys 顶住
    （refpath.py 的 `if r < r_phys: r = r_phys`），于是 dev 随 θ 单调上升。
    出厂参数 servo_mode=ideal / steer_limit_deg=30 / wheelbase=0.157 给
    r_phys = 0.157/tan30° = 0.2719m，代入 dev=0.175 解得 θ = 105.05°。

    旧值 125.0 意味着 105°~125° 的弯走 fillet，切角最大 31.7cm —— 远超
    17.5cm 失格线，参考线会从障碍/靶体上切过去。这条测试把"配置值必须跟
    物理半径自洽"钉住：改了 steer_limit_deg / wheelbase / servo_mode 而忘了
    重算 arc_fallback_deg，当场就红。
    """
    from dznav.refpath import DQ_DEVIATION_M, max_fillet_deg
    from dznav.servo import ServoModel

    servo = ServoModel(steer_limit_deg=DEFAULTS["servo_limit_deg"],
                       wheelbase=DEFAULTS["wheelbase"],
                       mode=DEFAULTS["servo_mode"])
    r_phys = 1.0 / servo.kappa_max(1.0)
    limit = max_fillet_deg(r_phys, DQ_DEVIATION_M)
    assert DEFAULTS["arc_fallback_deg"] <= limit + 1e-6, (
        f"arc_fallback_deg={DEFAULTS['arc_fallback_deg']}° 超过物理上限 "
        f"{limit:.2f}°（r_phys={r_phys:.4f}m，失格线 {DQ_DEVIATION_M}m）："
        f"这个角度区间内的弯会生成失格级切角")


def test_max_fillet_deg_matches_hand_computed_value():
    """max_fillet_deg 的公式本身要对：出厂参数下应解出 ≈105.05°。

    手算复核（不依赖被测函数）：dev = r·(1/cos(θ/2)−1) = 0.175，r = 0.2719
    → cos(θ/2) = 1/(1+0.175/0.2719) = 0.60849 → θ/2 = 52.52° → θ = 105.05°。
    """
    from dznav.refpath import max_fillet_deg

    r_phys = 0.157 / math.tan(math.radians(30.0))
    assert abs(r_phys - 0.2719) < 5e-4
    assert abs(max_fillet_deg(r_phys) - 105.05) < 0.05


def test_shipped_yaml_arc_fallback_matches_defaults():
    """yaml 与 DEFAULTS 的 arc_fallback_deg 必须同值，防单边改漂移。

    既有 test_shipped_yaml_keys_match_defaults 只比**键集合**，不比值；
    这个参数一旦两边不一致，现场调了 yaml 却按 DEFAULTS 跑（或反之），
    失格线约束就悄悄失效。
    """
    yaml = pytest.importorskip("yaml")
    path = (Path(__file__).resolve().parent.parent / "src" / "dznav" / "config"
            / "dznav_default.yaml")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert float(data["arc_fallback_deg"]) == float(DEFAULTS["arc_fallback_deg"]), (
        f"arc_fallback_deg: yaml={data['arc_fallback_deg']} "
        f"DEFAULTS={DEFAULTS['arc_fallback_deg']}")


# ============================== 相邻 fillet 切点重叠（路径自交，F3 未修只告警）
def _servo_from_defaults():
    from dznav.servo import ServoModel
    return ServoModel(steer_limit_deg=DEFAULTS["servo_limit_deg"],
                      wheelbase=DEFAULTS["wheelbase"],
                      mode=DEFAULTS["servo_mode"])


def test_consecutive_90_corners_on_cell_leg_overlap_is_warned():
    """出厂参数下，同一格边(0.35m)上相邻两个 90° 弯的切点必然重叠 —— 必须告警。

    手算：r_phys = 0.157/tan30° = 0.2719m；90° 弯切点长 = r_phys·tan45°
    = 0.2719m。两个弯共享一条腿 → 需 0.5439m > 0.35m（连对角腿 0.495m 也不够）。
    于是 straight_len 归零、两弧互相重叠、参考线自交，project 会投到错误一侧，
    cte 冲高触发 pursuit._need_rebuild；rebuild_limit(3) 耗尽后 forced arrived，
    任务在场中中止。

    正解（按比例同时缩小共享腿上的两个 fillet，或把顶点转 ARC）是路径几何设计
    变更，没有 SIL 不敢动；这里钉住的是"它至少不能再是静默的"。
    """
    from dznav.contract import BLOCK
    from dznav.refpath import PathOptions, Waypoint, build_reference

    assert abs(BLOCK - 0.35) < 1e-9, "本测试的数值反例基于格边 0.35m"
    r_phys = 1.0 / _servo_from_defaults().kappa_max(1.0)
    need = 2.0 * r_phys * math.tan(math.radians(45.0))
    assert need > BLOCK, f"反例失效：两弯只需 {need:.3f}m <= 腿长 {BLOCK}m"

    msgs: list = []
    wps = [Waypoint(0.0, 0.0), Waypoint(BLOCK, 0.0),
           Waypoint(BLOCK, BLOCK), Waypoint(2 * BLOCK, BLOCK)]
    build_reference(wps, _servo_from_defaults(), PathOptions(),
                    warn=msgs.append)
    hit = [m for m in msgs if "重叠" in m]
    assert hit, f"切点重叠未告警；实际告警={msgs}"


def test_well_spaced_corners_do_not_trigger_overlap_warning():
    """告警不能对合法几何误报，否则现场会把它当噪声关掉。

    腿长 1.2m 时两弯只需 0.544m，余量充足 → 不得出现"重叠"告警。
    """
    from dznav.refpath import PathOptions, Waypoint, build_reference

    msgs: list = []
    wps = [Waypoint(0.0, 0.0), Waypoint(1.2, 0.0),
           Waypoint(1.2, 1.2), Waypoint(2.4, 1.2)]
    build_reference(wps, _servo_from_defaults(), PathOptions(),
                    warn=msgs.append)
    assert not [m for m in msgs if "重叠" in m], msgs


def test_dev_budget_floor_breach_is_warned():
    """dev_cap 被物理半径顶穿时必须告警：出厂参数下这是常态而非边缘情况。

    90° 弯的 r_dev = 0.08/(1/cos45°−1) = 0.193m < r_phys = 0.272m，所以
    `if r < r_phys: r = r_phys` 每个 90° 弯都会走，dev_cap=0.08 这个"预算"
    从未真正生效（实际偏差 11.3cm）。原代码只在 b.critical 时才喊，非关键
    弯全程静默。
    """
    from dznav.refpath import PathOptions, Waypoint, build_reference

    msgs: list = []
    wps = [Waypoint(0.0, 0.0), Waypoint(1.2, 0.0),
           Waypoint(1.2, 1.2), Waypoint(2.4, 1.2)]
    build_reference(wps, _servo_from_defaults(), PathOptions(),
                    warn=msgs.append)
    assert [m for m in msgs if "顶穿" in m], msgs


# ================================ rebuild 必须沿用配置过的 PathOptions（F-opts）
def test_rebuild_reuses_configured_path_options():
    """_rebuild 曾硬编码 PathOptions(dev_cap=0.08, dev_cap_critical=0.03)。

    后果：运维在 YAML 里调过的 dev_cap / dev_cap_critical / arc_fallback_deg
    只在首条路径上生效，第一次重建就被静默丢回 dataclass 默认值，日志里毫无
    痕迹。arc_fallback_deg 尤其要命——丢回默认等于失格线约束换了个值。
    """
    from dznav.refpath import PathOptions, Waypoint, build_reference
    from dznav.pursuit import Tracker, TrackerConfig

    opts = PathOptions(dev_cap=0.15, dev_cap_critical=0.05,
                       arc_fallback_deg=100.0)
    wps = [Waypoint(0.0, 0.0), Waypoint(1.2, 0.0),
           Waypoint(1.2, 1.2), Waypoint(2.4, 1.2)]
    tr = Tracker(build_reference(wps, _servo_from_defaults(), opts),
                 _servo_from_defaults(), TrackerConfig())

    assert tr._rebuild(0.0, 0.0, 0.0, 0.0, []) is True
    assert tr.path.opts is not None, "重建后的路径丢了 opts"
    assert tr.path.opts.dev_cap == 0.15
    assert tr.path.opts.dev_cap_critical == 0.05
    assert tr.path.opts.arc_fallback_deg == 100.0


def test_build_reference_records_opts_on_path():
    """opts 必须挂在 ReferencePath 上，否则 _rebuild 无从沿用。"""
    from dznav.refpath import PathOptions, Waypoint, build_reference

    opts = PathOptions(dev_cap=0.11)
    p = build_reference([Waypoint(0.0, 0.0), Waypoint(1.0, 0.0)],
                        _servo_from_defaults(), opts)
    assert p.opts is opts
    # 不传 opts 时也要落一个实例，不能留 None（_rebuild 依赖它）
    p2 = build_reference([Waypoint(0.0, 0.0), Waypoint(1.0, 0.0)],
                         _servo_from_defaults())
    assert p2.opts is not None and p2.opts.dev_cap == PathOptions().dev_cap


# ============================ _pending_anchor 是弧长，换路径必须作废（F9）
def test_pending_anchor_cleared_on_rebuild_event():
    """rebuild 把 s_prog 归零并换了全新几何，旧弧长必须当场作废。

    不清的话：s_prog 从 0 重新爬，爬到旧锚点数值那一刻
    `s_prog - _pending_anchor` 恰好很小（重建后 v_cmd=0 起步，单周期只走几厘米），
    0.20m 新鲜度窗口拦不住 —— 会在一个跟"出弯收敛区"毫无关系的位置注入
    anchor，污染定位。
    """
    node = DzNavNode()
    _armed(node)
    node._pending_anchor = 1.234
    node._handle_event({"ev": "rebuild", "to_idx": 3})
    assert node._pending_anchor is None


def test_pending_anchor_cleared_on_abort_and_new_route():
    """_abort_route / _start_route 都不走对方，两处都得单独清。

    _abort_route 只把 tracker 置 None；下一条路线 _start_route 直接覆盖
    tracker，于是旧弧长活到新路径上，在新几何里被当成出弯锚点消费。
    """
    node = DzNavNode()
    _armed(node)
    node._pending_anchor = 2.0
    node._abort_route()
    assert node._pending_anchor is None

    node._pending_anchor = 2.0
    _armed(node, route="M9,y")     # 走 _start_route
    assert node._pending_anchor is None


def test_arrived_forced_flag_reset_on_new_route():
    """放弃收车的标记不能带到下一条路线上，否则复盘时误判成败。"""
    node = DzNavNode()
    _armed(node)
    node._handle_event({"ev": "arrived", "d": 2.5, "forced": True})
    assert node.arrived_forced is True
    _armed(node, route="M9,y")
    assert node.arrived_forced is False


# ====================== forced arrived 不是到达：不得 FINAL_ARC / 不得锚终点（F10）
def test_forced_arrival_does_not_run_final_arc_or_anchor_goal():
    """rebuild_limit 耗尽后 pursuit 发的 "arrived" 是**放弃**，d 无界。

    旧代码 `elif ev == "arrived": self._on_arrived(e.get("d", 0.0))` 丢掉了
    forced 标记，于是"车还在场中、距终点 2.5m"被当成正常到点：
      1) 若 final_heading 偏差 >12° 就进 FINAL_ARC，在场中画满舵弧；
      2) 弧结束后 loc.anchor(终点坐标, blend=0.7) —— belief 传送到终点格；
      3) _finish() 以 INFO 宣告 ARRIVED，复盘看不出是失败。
    """
    node = DzNavNode()
    _armed(node, route="M9,y")
    goal = node.tracker.path.waypoints[-1]
    before = node.loc.pose[:2]

    node.final_heading = wrap_pi(node.loc.th + math.radians(90.0))
    node._handle_event({"ev": "arrived", "d": 2.5, "forced": True})

    assert node.state == "ARRIVED"          # 仍是闩锁停车态
    assert node.arrived_forced is True
    assert node._final_arc is None, "放弃收车却进了终点朝向机动"
    after = node.loc.pose[:2]
    assert math.hypot(after[0] - before[0], after[1] - before[1]) < 1e-9, (
        f"belief 被传送：{before} -> {after}（终点 {goal.x:.2f},{goal.y:.2f}）")
    assert node.get_logger().text("error", "跟踪放弃"), "放弃必须用 error 喊出来"


def test_normal_arrival_still_runs_final_arc():
    """反向对照：正常到点（非 forced）该走的终点机动不能被一并禁掉。"""
    node = DzNavNode()
    _armed(node, route="M9,y")
    node.final_heading = wrap_pi(node.loc.th + math.radians(90.0))
    node._handle_event({"ev": "arrived", "d": 0.05})
    assert node.state == "FINAL_ARC"
    assert node._final_arc is not None
    assert node.arrived_forced is False


# ==================== 终点锚定必须先证明车真在终点（F8）
def test_goal_anchor_max_m_derivation():
    """可锚上限 = 失格线 / blend：注入误差 blend·e 单靠自己不得破 17.5cm。

    blend_arc=0.7 → 0.25m。做成随增益反解，改 blend_arc 时上限自动跟着走。
    """
    from dznav.executor_node import goal_anchor_max_m
    from dznav.refpath import DQ_DEVIATION_M

    assert abs(goal_anchor_max_m(0.7) - 0.25) < 1e-9
    assert abs(goal_anchor_max_m(1.0) - DQ_DEVIATION_M) < 1e-9
    assert goal_anchor_max_m(0.0) > 0.0        # 不得除零


def _enter_final_arc(node, off: float):
    """把车摆到"距终点 off 米、朝向已转到位"的状态，进入 FINAL_ARC。"""
    goal = node.tracker.path.waypoints[-1]
    node.final_heading = wrap_pi(node.loc.th + math.radians(30.0))
    node._final_arc = {"sign": 1.0, "target": node.loc.th,
                       "x": goal.x, "y": goal.y, "t0": node._t}
    # 直接改 belief 位置，模拟"入场时就偏了 off 米"
    node.loc.x = goal.x + off
    node.loc.y = goal.y
    return goal


def test_final_arc_skips_position_anchor_when_far_from_goal():
    """满舵弧只改朝向不改位置，偏 0.40m 入场就偏 0.40m 出场。

    _final_arc_feasible() 只检查弧几何在图内，从不检查车离终点多远；
    旧代码无条件 anchor(终点, blend=0.7)，等于把 belief 硬拉 0.28m。
    """
    node = DzNavNode()
    _armed(node, route="M9,y")
    goal = _enter_final_arc(node, off=0.40)

    node._final_arc_step()      # err=0 → done，走锚定分支

    x, y, _th = node.loc.pose
    assert abs(x - goal.x) > 0.30, (
        f"位置被拉向终点：x={x:.3f} 终点={goal.x:.3f}（应保留真实偏差）")
    assert node.get_logger().text("warning", "只锚朝向不锚位置")


def test_final_arc_anchors_when_actually_at_goal():
    """反向对照：真在终点附近（偏 5cm）时锚定必须照常发生，否则丢抗漂移。"""
    node = DzNavNode()
    _armed(node, route="M9,y")
    goal = _enter_final_arc(node, off=0.05)

    node._final_arc_step()

    x, y, _th = node.loc.pose
    assert abs(x - goal.x) < 0.05, f"近终点却没锚定：x={x:.3f} {goal.x:.3f}"


def test_final_arc_timeout_does_not_fabricate_heading():
    """12s 超时说明弧**没转到**目标朝向，此时锚航向同样是编造。

    旧代码 `if done or timeout:` 两个分支共用同一句 anchor(target)，超时也
    把 belief 航向拉向一个从未到达的 target，残差可能几十度。
    """
    node = DzNavNode()
    _armed(node, route="M9,y")
    goal = node.tracker.path.waypoints[-1]
    node._final_arc = {"sign": 1.0, "target": wrap_pi(node.loc.th + 1.0),
                       "x": goal.x, "y": goal.y, "t0": node._t - 99.0}
    th_before = node.loc.th

    node._final_arc_step()      # err=1.0rad 未到位，但已超时

    assert node.state == "ARRIVED"
    assert abs(wrap_pi(node.loc.th - th_before)) < 1e-9, (
        f"超时却把航向锚向未到达的 target：{th_before:.3f} -> {node.loc.th:.3f}")
    assert node.get_logger().text("error", "终点朝向机动超时")
