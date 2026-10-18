# -*- coding: utf-8 -*-
"""dznav 软件在环回归：全链路真码对照。

链路 A（dznav）：raw /odom + /imu_data -> DzNavNode（真码，ros_stub 桩化）
  -> pursuitAngle -> DzActuatorSim -> ChassisSim（真值）。
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
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_stub  # noqa: E402  必须先于节点模块导入
from ros_stub import BUS, SIM, Odometry, String, Twist  # noqa: E402
from std_msgs.msg import Int32, UInt8  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

import sim_car  # noqa: E402
from sim_car import ChassisSim, DzActuatorSim, SerialReaderSim  # noqa: E402

from dznav.contract import block_to_meters  # noqa: E402
from dznav.executor_node import DzNavNode  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
DT = 0.02

# 发学员基线执行器真码（吸收快照，只读加载）
_PE_PATH = (ROOT / "reference" / "car_src_fxxy_20261017" / "car_localization_rect"
            / "car_localization" / "path_executor.py")


def load_fxxy_baseline():
    os.environ.setdefault("DZ_AUTO_START", "0")
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
        self.wp_pub = self.rig.create_publisher(String, "/nav/waypoint", 10)
        self.cv_pub = self.rig.create_publisher(Twist, "/cmd_vel", 10)
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
    """链路 A：dznav 节点真码（融合+跟踪一体）。"""

    def __init__(self, node_overrides: dict | None = None, **kw):
        super().__init__(**kw)
        self.node = DzNavNode(overrides=node_overrides)
        self.odom_pub = self.rig.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.rig.create_publisher(Imu, "/imu_data", 10)

    def _feed_sensors(self, t):
        m = Odometry()
        m.pose.pose.position.x = self.chassis.x
        m.pose.pose.position.y = self.chassis.y
        self.odom_pub.publish(m)
        yaw_rel = sim_car.wrap_pi(self.chassis.theta - self.theta_boot)
        self.imu_pub.publish(_quat(Imu(), yaw_rel))
        self.node.tick()          # 触发 50Hz 控制（含信念发布）
        # 到点时刻标记（终点朝向机动前，供横向统计截断）
        if self.node.tracker is not None and self.node.tracker.arrived:
            self._mark_arrived()

    def _in_shoot_wait(self):
        sigs = self.node.pub_stop.published
        if sigs and self.stop_signals != sigs:
            self.stop_signals = list(sigs)
        return (self.node.state == "SHOOT_WAIT",
                int(self.node.shoot_at_arrival))

    @property
    def arrived(self) -> bool:
        return (self.node.tracker is not None
                and self.node.tracker.arrived)


class BaselineWorld(_World):
    """链路 B：发学员 serial_reader 语义 + path_executor 真码。"""

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
        assert len(w.node.pub_cmd.published) == 0
        assert w.chassis.dist_traveled == pytest.approx(0.0, abs=1e-9)
        # 信念流常开（track_diag 依赖）
        assert len(w.node.pub_loc.published) >= 200

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
        sigs = [int(m.data) for m in w.node.pub_stop.published]
        assert 1 in sigs and 0 in sigs
        logs = " ".join(r[1] for r in w.node.logger.records)
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
        logs = " ".join(r[1] for r in w.node.logger.records)
        assert "身后" in logs, "应先做对齐弧"


# ================================================================ 对照
class TestBaselineCompare:

    @pytest.mark.parametrize("route", [MAIN_ROUTE, SBEND_ROUTE],
                             ids=["main", "sbend"])
    def test_dznav_beats_baseline(self, route):
        """净起步同协议双栈对照：dznav 横向精度不劣于发学员基线。"""
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


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
