# -*- coding: utf-8 -*-
"""dznav 软件在环回归：全链路真码对照（trae-glm 从 preserve/kimi/wip-20261014 移植）。

链路 A（dznav）：raw /odom + /imu_data -> DzNavNode（真码）-> pursuitAngle
  -> DzActuatorSim -> ChassisSim（真值）。
链路 B（基线）：SerialReaderSim -> 发学员 path_executor 真码
  （reference/car_src_fxxy_20261017/car_localization_rect，importlib 只读加载）
  -> pursuitAngle -> 同一 DzActuatorSim/ChassisSim。

保真边界同 tests/sim_car.py：验证"契约 + 控制逻辑"，不做动力学参数辨识
（理想定位、无滑移、速度瞬时到位）。两链路传感器同等理想化 —— 度量的是
控制架构差异，不是传感器差异。

主证明场景 = test_sim_tracking.py::test_align_then_route_arrives 的
图缘终点几何（N10 起步 68.6° 摆位偏差 → align → E9* 射击停站 →
N11,x 图缘终点）：legacy 现役版在该场景东向出图越界急停（xfail 固化），
dznav 用"终点顶点 ARC 机动"替代"回退插点"应直接通过。

移植注记（2026-10-18）：kimi 原版写于自带 ros_stub 环境（DzNavNode 绑
ros_stub.FakeNode：BUS 直通 + SIM 时钟 + tick()）。canonical 有两套桩：
根 conftest 的 fake rclpy 与 tests/ros_stub.py。文件名带 sim_tracking 前缀
是**收集顺序契约**：import ros_stub 会整体覆盖 sys.modules 里的 rclpy，
本文件必须排在 test_integrate_node.py（需要根桩 FakeClock.advance）等
之后、紧跟 test_sim_tracking.py（同款先例）；全套跑时 DzNavNode 已被
test_dznav_safety.py 先 import 而绑根桩（无 tick/inject、publisher 是
.sent、发布不经 BUS），单跑本文件时 ros_stub 先装、DzNavNode 绑
ros_stub.FakeNode（kimi 原生语义）。下方 _deliver/_fire/_pub_msgs/
_bind_bridge/_flush_pursuit 适配层让两条绑定路径行为一致：消息一律直投
节点订阅回调；根桩绑定时节点时钟改挂 SIM、pursuitAngle 手动桥回 BUS
（actuator 才收得到）。
"""

from __future__ import annotations

import importlib.util
import math
import os
import types
from pathlib import Path

import pytest

import ros_stub  # noqa: F401  必须先于被测节点模块导入（单文件跑时绑 ros_stub 桩）
from ros_stub import BUS, SIM, Odometry, String, Twist
from std_msgs.msg import Int32, UInt8
from sensor_msgs.msg import Imu

import sim_car
from sim_car import ChassisSim, DzActuatorSim, SerialReaderSim

from dznav.contract import (TOPIC_IMU, TOPIC_ODOM_RAW, TOPIC_PURSUIT,
                            TOPIC_SHOOT_COUNT, TOPIC_STOP_SIGNAL,
                            TOPIC_WAYPOINT, block_to_meters)
from dznav.executor_node import DzNavNode

ROOT = Path(__file__).resolve().parents[1]
DT = 0.02

# 发学员基线执行器真码（吸收快照，只读加载；gitignore 不入库）
_PE_PATH = (ROOT / "reference" / "car_src_fxxy_20261017" / "car_localization_rect"
            / "car_localization" / "path_executor.py")


def load_fxxy_baseline():
    """只读加载学员 path_executor 真码（BaselineWorld 链路 B 用）。

    环境隔离与 sim_car.load_path_executor_module 同款：DZ_AUTO_START=0
    强制待命；ROBOT_CONFIG 指向不存在路径，防止本机 ~/dzrobot/config/
    robot.yaml 的真机参数渗进仿真。
    """
    os.environ["DZ_AUTO_START"] = "0"
    os.environ["ROBOT_CONFIG"] = str(
        ROOT / "tests" / "_nonexistent_robot_config.yaml")
    spec = importlib.util.spec_from_file_location("fxxy_pe", _PE_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


#: 主证明路线（图缘终点 + 射击停站 + 斜线段 + 直角弯混合）
MAIN_ROUTE = ("N10 M9 L8 K8 J8 I8 H9 G9 F9 E9* F10 G11 H11 I11 "
              "J11 K11 L11 M11 N11,x")
#: S 弯路线（右左右右左直角弯序列；首段 +y 与 reset 断言朝向一致）
SBEND_ROUTE = "N10 K10 K7 G7 G4 M4 M7,y"


def _quat(msg, yaw):
    msg.orientation.z = math.sin(yaw / 2.0)
    msg.orientation.w = math.cos(yaw / 2.0)
    return msg


# ---------------------------------------------------------------- 桩适配层
def _root_bound(node) -> bool:
    """True=节点绑根 conftest 桩（_FakeNode）；False=绑 tests/ros_stub 桩。

    按基类名判 MRO：两套桩的类名分别是 _FakeNode / FakeNode。不能用
    hasattr 探方法（2026-10-18 两栖化后 ros_stub 也有 inject/.sent，
    特征撞车），也不能信 sys.modules（importlib 缓存决定绑定，看实例最真）。
    """
    return not any(cls.__name__ == "FakeNode" for cls in type(node).__mro__)


def _deliver(node, topic: str, msg) -> None:
    """把消息直投节点的订阅回调（两种桩绑定通用，绕过 BUS）。"""
    if _root_bound(node):             # 根桩：inject
        node.inject(topic, msg)
    else:                             # ros_stub：deliver
        node.deliver(topic, msg)


def _fire(node) -> None:
    """触发节点全部定时器（= kimi 原版 node.tick() 的语义）。"""
    if _root_bound(node):             # 根桩 timers=[(period, cb)]
        for _period, cb in node.timers:
            cb()
    else:                             # ros_stub timers=[FakeTimer]
        node.tick()


def _pub_msgs(node, topic: str) -> list:
    """取发布器历史消息（两栖桩都有 .sent：根桩原生/ros_stub 别名）。"""
    return node.publishers[topic].sent


def _bind_bridge(world) -> None:
    """根桩绑定时让节点与仿真世界接通（ros_stub 绑定时是 no-op）。

    - 时钟：根桩 FakeClock.now() 每次 +0.1s 自推进、与 SIM.t 脱钩，而
      _control 的 dt 取真实时钟差（限幅/看门狗都吃它）——遮蔽 get_clock
      改读全局仿真时钟，与 kimi 原版 FakeClock(SIM) 语义一致。
    - pursuitAngle：根桩 FakePublisher.publish 不经 BUS，执行器收不到；
      每拍 tick 后手动把新命令 dispatch 回 BUS（actuator 订阅在 rig 上）。
    """
    node = world.node
    world._root_stub = _root_bound(node)
    if not world._root_stub:
        return
    clock = types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(nanoseconds=int(SIM.t * 1e9)))
    node.get_clock = lambda: clock   # 实例属性遮蔽，仅限本仿真
    node._wall_prev = 0.0            # 构造期根桩时钟残留，矫正到仿真 0 点
    world._pu_k = 0


def _flush_pursuit(world) -> None:
    """根桩绑定专用：pursuitAngle 新命令桥回 BUS。"""
    if not getattr(world, "_root_stub", False):
        return
    pub = world.node.publishers[TOPIC_PURSUIT]
    for msg in pub.sent[world._pu_k:]:
        BUS.dispatch("pursuitAngle", msg)
    world._pu_k = len(pub.sent)


class _World:
    """公共骨架：底盘/执行器/对齐/指标。子类喂不同的"车端栈"。"""

    def __init__(self, start_cell="N10", true_yaw_deg=158.6,
                 theta_boot_deg=90.0):
        BUS.reset()
        SIM.enable(0.0)
        os.environ["DZ_AUTO_START"] = "0"
        x0, y0 = block_to_meters(start_cell)
        self.chassis = ChassisSim(x0, y0, math.radians(true_yaw_deg))
        self.actuator = DzActuatorSim()
        self.theta_boot = math.radians(theta_boot_deg)
        self.rig = ros_stub.FakeNode("sil_rig")
        self.rig.create_subscription(
            Twist, "/cmd_vel", lambda m: self.actuator.on_cmd_vel(m, SIM.t), 10)
        self.rig.create_subscription(
            Twist, "pursuitAngle",
            lambda m: self.actuator.on_pursuit(m, SIM.t), 10)
        self.cv_pub = self.rig.create_publisher(Twist, "/cmd_vel", 10)
        # 基线链路（B）仍走 BUS：学员 path_executor 是 ros_stub 节点
        self.wp_pub = self.rig.create_publisher(String, "/nav/waypoint", 10)
        self.shoot_pub = self.rig.create_publisher(Int32, "/shoot_count", 10)
        # 指标
        self.t = 0.0
        self.trace = []            # (t, x, y, th, v, steer)
        self.route_sent_t = None
        self.arrived_t = None
        self._shots_k = 0
        self._next_shot_t = 0.0
        self.stop_signals = []

    def _mark_arrived(self):
        if self.arrived_t is None:
            self.arrived_t = self.t

    # ---- 对齐：闭环把真实车转到目标朝向（heading_calibration 的理想化）
    def align_to(self, target_deg: float, tol_deg: float = 1.5,
                 budget_s: float = 30.0):
        target = math.radians(target_deg)
        while self.t < budget_s:
            err = sim_car.wrap_pi(target - self.chassis.theta)
            if abs(err) <= math.radians(tol_deg):
                break
            m = Twist()
            m.linear.x = 0.10        # 慢速弧转：控制漂移（真机 align 同理）
            m.angular.z = 1.2 * (1.0 if err > 0 else -1.0)
            self.cv_pub.publish(m)
            self.step()
        m = Twist()
        self.cv_pub.publish(m)
        self.step()
        assert abs(sim_car.wrap_pi(target - self.chassis.theta)) \
            <= math.radians(tol_deg + 1.0)

    def send(self, cmd: str):
        self.wp_pub.publish(String(data=cmd))
        if self.route_sent_t is None and not cmd.startswith("reset:"):
            self.route_sent_t = self.t

    def step(self):
        t = SIM.advance(DT)
        self.t = t
        self._feed_sensors(t)
        self._shoot_feeder(t)
        self.actuator.step(t)
        v, steer = self.actuator.out_v, self.actuator.out_steer_deg
        self.chassis.step(v, steer, DT)
        self.trace.append((t, self.chassis.x, self.chassis.y,
                           self.chassis.theta, v, steer))
        return t

    def _feed_sensors(self, t):
        raise NotImplementedError

    def _in_shoot_wait(self) -> tuple[bool, int]:
        raise NotImplementedError

    def _shoot_feeder(self, t):
        waiting, at_arrival = self._in_shoot_wait()
        if not waiting:
            self._shots_k = 0
            self._next_shot_t = 0.0
            return
        if self._next_shot_t == 0.0:
            self._next_shot_t = t + 0.25
        if t >= self._next_shot_t - 1e-9 and self._shots_k < 10:
            self._shots_k += 1
            self._next_shot_t = t + 0.25
            self.shoot_pub.publish(Int32(data=at_arrival + self._shots_k))

    def run_until(self, cond, budget_s: float) -> bool:
        end = self.t + budget_s
        while not cond() and self.t < end - 1e-9:
            self.step()
        return bool(cond())

    # ---- 指标
    def lateral_stats(self, blocks) -> dict:
        """真值轨迹对理想折线（途经格心连线）的横向偏差统计。

        统计窗口：发车 → 到点（终点朝向机动属于收车动作，不计入）。
        """
        pts = [block_to_meters(b) for b in blocks]
        segs = list(zip(pts[:-1], pts[1:]))

        def seg_d(px, py, a, b):
            dx, dy = b[0] - a[0], b[1] - a[1]
            tt = max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy)
                              / (dx * dx + dy * dy)))
            return math.hypot(px - a[0] - tt * dx, py - a[1] - tt * dy)

        t_end = self.arrived_t if self.arrived_t is not None else self.t
        devs = []
        for rec in self.trace:
            if self.route_sent_t is None or rec[0] < self.route_sent_t:
                continue
            if rec[0] > t_end:
                continue
            d = min(seg_d(rec[1], rec[2], a, b) for a, b in segs)
            devs.append(d)
        if not devs:
            return {"rms": float("inf"), "max": float("inf"), "n": 0}
        return {"rms": math.sqrt(sum(d * d for d in devs) / len(devs)),
                "max": max(devs), "n": len(devs)}


class DzNavWorld(_World):
    """链路 A：dznav 节点真码（融合+跟踪一体）。消息直投节点回调。"""

    def __init__(self, node_overrides: dict | None = None, **kw):
        super().__init__(**kw)
        self.node = DzNavNode(overrides=node_overrides)
        _bind_bridge(self)

    def send(self, cmd: str):
        # dznav 侧直投节点订阅回调（两种桩绑定通用，不依赖 BUS 注册）
        _deliver(self.node, TOPIC_WAYPOINT, String(data=cmd))
        if self.route_sent_t is None and not cmd.startswith("reset:"):
            self.route_sent_t = self.t

    def _feed_sensors(self, t):
        m = Odometry()
        m.pose.pose.position.x = self.chassis.x
        m.pose.pose.position.y = self.chassis.y
        _deliver(self.node, TOPIC_ODOM_RAW, m)
        yaw_rel = sim_car.wrap_pi(self.chassis.theta - self.theta_boot)
        _deliver(self.node, TOPIC_IMU, _quat(Imu(), yaw_rel))
        _fire(self.node)         # 触发 50Hz 控制（含信念发布）
        _flush_pursuit(self)     # 根桩绑定：pursuitAngle 桥回 BUS
        # 到点时刻标记（终点朝向机动前，供横向统计截断）
        if self.node.tracker is not None and self.node.tracker.arrived:
            self._mark_arrived()

    def _shoot_feeder(self, t):
        # 云台弹数回报直投 dznav（基线链路仍走 BUS）
        waiting, at_arrival = self._in_shoot_wait()
        if not waiting:
            self._shots_k = 0
            self._next_shot_t = 0.0
            return
        if self._next_shot_t == 0.0:
            self._next_shot_t = t + 0.25
        if t >= self._next_shot_t - 1e-9 and self._shots_k < 10:
            self._shots_k += 1
            self._next_shot_t = t + 0.25
            _deliver(self.node, TOPIC_SHOOT_COUNT,
                     Int32(data=at_arrival + self._shots_k))

    def _in_shoot_wait(self):
        sigs = _pub_msgs(self.node, TOPIC_STOP_SIGNAL)
        if sigs and self.stop_signals != sigs:
            self.stop_signals = list(sigs)
        return (self.node.state == "SHOOT_WAIT",
                int(self.node.shoot_at_arrival))

    @property
    def arrived(self) -> bool:
        return (self.node.tracker is not None
                and self.node.tracker.arrived)


class BaselineWorld(_World):
    """链路 B：发学员 serial_reader 语义 + path_executor 真码（走 BUS）。"""

    def __init__(self, **kw):
        super().__init__(**kw)
        self._pe_mod = load_fxxy_baseline()
        self.serial = SerialReaderSim(self.theta_boot,
                                      theta_true0=self.chassis.theta)
        self.loc_pub = self.rig.create_publisher(
            Odometry, "/odom/localization", 10)
        self.pe = self._pe_mod.PathExecutorNode()

    def _feed_sensors(self, t):
        self.loc_pub.publish(self.serial.make_odom())
        self.pe.control_loop()    # 50Hz 调用（原 20Hz 状态机，多调无害）
        if getattr(self.pe, "arrived", False):
            self._mark_arrived()
        # 基线 control_loop 后推进传感器（与 TrackingWorld 顺序等价）
        self.serial.step(self.actuator.out_v, DT, self.chassis.theta)

    def _in_shoot_wait(self):
        return (getattr(self.pe, "mode", "") == "shoot_wait",
                int(getattr(self.pe, "shoot_count_at_arrival", 0)))

    @property
    def arrived(self) -> bool:
        return bool(getattr(self.pe, "arrived", False))


def _blocks_of(route: str) -> list:
    return [tok.upper().rstrip("*") for tok in route.replace(",", " ").split()
            if tok and tok[0] in "ABCDEFGHIJKLMN"]


@pytest.fixture(autouse=True)
def _sim_cleanup():
    """SIL 全局态收尾：停用总线/时钟，避免冻结后续测试的 FakeClock
    （曾致 test_integrate_node 的 align 超时用例卡在 aligning）。"""
    yield
    ros_stub.BUS.disable()
    ros_stub.SIM.disable()


# ================================================================ 场景
class TestDzNavSil:

    def test_standby_silent_no_motion(self):
        """待命静默：无路线不发 pursuitAngle、车不动（align 可随时做）。"""
        w = DzNavWorld()
        for _ in range(250):        # 5s
            w.step()
        assert w.node.state == "STANDBY"
        assert len(_pub_msgs(w.node, TOPIC_PURSUIT)) == 0
        assert w.chassis.dist_traveled == pytest.approx(0.0, abs=1e-9)
        # 信念流常开（track_diag 依赖）
        assert len(_pub_msgs(w.node, "/odom/localization")) >= 200

    def test_reset_reanchor(self):
        w = DzNavWorld()
        w.align_to(90.0)
        w.send("reset:N10,y")
        w.step()
        x, y, th = w.node.loc.pose
        ex, ey = block_to_meters("N10")
        assert (x, y) == pytest.approx((ex, ey), abs=0.01)
        assert th == pytest.approx(math.pi / 2, abs=0.02)

    def test_align_then_route_arrives(self):
        """主证明：图缘终点几何（legacy xfail 场景）dznav 必须到点不出图。

        68.6° 摆位偏差 → align（弧线转动，物理上必带 ~0.3m 位移漂移）
        → reset 断言 → E9* 射击停站 → N11,x 图缘终点。
        legacy 现役版此场景：终点回退插点被图缘钳位 → PIVOT 过冲 →
        东向出图 1.7m+ → 越界急停（docs/SIL发现-PIVOT过冲-20261016.md）。
        dznav 断言：到点、全程不出图（真值包络）、射击停站全流程。
        """
        w = DzNavWorld()
        w.align_to(90.0)
        w.send("reset:N10,y")
        w.send(MAIN_ROUTE)
        # 跑到完整收车（ARRIVED 含终点朝向机动完成/跳过）
        ok = w.run_until(lambda: w.node.state in ("ARRIVED", "ESTOP"),
                         budget_s=150.0)
        assert w.node.state != "ESTOP", "图缘终点场景不得越界急停"
        assert ok and w.arrived, "必须到点"
        fx, fy = block_to_meters("N11")
        d = math.hypot(w.chassis.x - fx, w.chassis.y - fy)
        # 终点误差被 align 弧线漂移主导（68.6° 旋转弧长 ~0.27m，物理必然；
        # legacy 同场景历史口径 0.26m）。图缘朝向机动不可行时保持朝向收车。
        assert d < 0.35, f"终点误差 {d:.3f}m"
        # 真值位置全程不得远离开图（legacy 失败形态是出图 1m+ 狂奔；
        # align 弧线漂移允许贴边小越界，用 0.40m 包络刻画）
        from dznav.contract import FIELD_X_MAX, FIELD_Y_MAX
        for rec in w.trace:
            if w.route_sent_t is not None and rec[0] < w.route_sent_t:
                continue
            assert -0.40 <= rec[1] <= FIELD_X_MAX + 0.40, \
                f"t={rec[0]:.1f} 真值 x 出图 {rec[1]:.2f}"
            assert -0.40 <= rec[2] <= FIELD_Y_MAX + 0.40, \
                f"t={rec[0]:.1f} 真值 y 出图 {rec[2]:.2f}"
        # 射击停站走全流程：stop_signal 1 -> 0，弹数推进
        sigs = [int(m.data) for m in _pub_msgs(w.node, TOPIC_STOP_SIGNAL)]
        assert 1 in sigs and 0 in sigs
        logs = " ".join(r[1] for r in w.node.get_logger().records)
        assert "ARRIVED" in logs

    def test_mine_pass_through(self):
        """雷格过心：直段雷格，轮心到雷心最近距离 <= 6cm（排雷触发半径）。

        净起步（初始朝向即断言朝向，无 align 漂移）——严格横向精度场景。
        """
        route = "N10 K10 K7 G7 G4,y"
        mines = ["M10", "L10", "K9", "K8", "I7", "H7", "G6", "G5"]
        w = DzNavWorld(true_yaw_deg=90.0)
        w.align_to(90.0)
        w.send("reset:N10,y")
        w.send(route)
        ok = w.run_until(lambda: w.arrived or w.node.state == "ESTOP",
                         budget_s=120.0)
        assert ok and w.arrived and w.node.state != "ESTOP"
        for m in mines:
            mx, my = block_to_meters(m)
            dmin = min(math.hypot(r[1] - mx, r[2] - my) for r in w.trace)
            assert dmin <= 0.06, f"雷格 {m} 过心失败: {dmin:.3f}m"

    def test_s_bend_closed_loop(self):
        """左右弯全程闭环：无重建、无 ARC 机动、横向 < 17.5cm（净起步）。"""
        w = DzNavWorld(true_yaw_deg=90.0)
        w.align_to(90.0)
        w.send("reset:N10,y")
        w.send(SBEND_ROUTE)
        ok = w.run_until(lambda: w.node.state in ("ARRIVED", "ESTOP"),
                         budget_s=120.0)
        assert ok and w.arrived and w.node.state != "ESTOP"
        tr = w.node.tracker
        assert tr.rebuilds == 0, "S 弯不应触发重建"
        modes = [c.mode for c in tr.path.corners if c.idx > 0]
        assert modes and all(m == "fillet" for m in modes), modes
        stats = w.lateral_stats(_blocks_of(SBEND_ROUTE))
        assert stats["max"] < 0.175, f"横向峰值 {stats['max']:.3f}m"
        # 终点朝向机动收敛到 +y（非图缘，可行）
        assert abs(sim_car.wrap_pi(w.chassis.theta - math.pi / 2)) \
            < math.radians(20.0)

    def test_target_behind_recovery(self):
        """目标在身后：对齐弧自救，不画大回环、不越界。"""
        w = DzNavWorld(true_yaw_deg=180.0)
        w.align_to(180.0)          # 朝 -x（西）
        w.send("reset:N7,-x")
        w.send("N7 N4,y")          # 首段 +x 在正后方
        ok = w.run_until(lambda: w.arrived or w.node.state == "ESTOP",
                         budget_s=90.0)
        assert w.node.state != "ESTOP"
        assert ok and w.arrived
        logs = " ".join(r[1] for r in w.node.get_logger().records)
        assert "身后" in logs, "应先做对齐弧"


# ================================================================ 对照
class TestBaselineCompare:

    @pytest.mark.parametrize("route", [MAIN_ROUTE, SBEND_ROUTE],
                             ids=["main", "sbend"])
    def test_dznav_beats_baseline(self, route):
        """净起步同协议双栈对照：dznav 横向精度不劣于发学员基线。"""
        if not _PE_PATH.exists():
            pytest.skip("reference/car_src_fxxy_20261017 未取回"
                        "（gitignore 不入库的车端官方源码）")
        blocks = _blocks_of(route)

        wA = DzNavWorld(true_yaw_deg=90.0)
        wA.align_to(90.0)
        wA.send("reset:N10,y")
        wA.send(route)
        okA = wA.run_until(lambda: wA.arrived or wA.node.state == "ESTOP",
                           budget_s=150.0)
        assert okA and wA.arrived and wA.node.state != "ESTOP"
        sA = wA.lateral_stats(blocks)
        tA = wA.t - (wA.route_sent_t or 0.0)

        wB = BaselineWorld(true_yaw_deg=90.0)
        wB.align_to(90.0)
        wB.send("reset:N10,y")
        wB.send(route)
        okB = wB.run_until(lambda: wB.arrived, budget_s=200.0)
        sB = wB.lateral_stats(blocks)
        tB = wB.t - (wB.route_sent_t or 0.0)

        print(f"\n[{route[:24]}...] dznav: arrive={okA} t={tA:.1f}s "
              f"rms={sA['rms']*100:.1f}cm max={sA['max']*100:.1f}cm")
        print(f"[{route[:24]}...] base : arrive={okB} t={tB:.1f}s "
              f"rms={sB['rms']*100:.1f}cm max={sB['max']*100:.1f}cm")
        if okB:
            assert sA["rms"] <= sB["rms"] + 1e-9, \
                f"dznav RMS 应优于基线: {sA['rms']:.3f} vs {sB['rms']:.3f}"
            assert sA["max"] <= sB["max"] + 0.005, \
                f"dznav 峰值应不劣于基线: {sA['max']:.3f} vs {sB['max']:.3f}"
