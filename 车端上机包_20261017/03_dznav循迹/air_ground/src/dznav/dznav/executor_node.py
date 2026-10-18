# -*- coding: utf-8 -*-
"""dznav 执行节点：定位融合 + 路径跟踪一体化，替代官方
serial_reader + path_executor 组合（契约兼容，可一键切换/回滚）。

话题契约（与发学员/现役版完全一致，见 dznav/contract.py）：
  订阅 /nav/waypoint  reset:N10,y | load:/path | 'M10 L9 K7,y'（* 射击格）
  订阅 /odom          dzactuator 原始里程（只取步长距离）
  订阅 /imu_data      IMU 航向
  订阅 /shoot_count   累计弹数
  发布 /odom/localization  信念位姿（格；yaw+π/2=地图航向）
  发布 pursuitAngle         linear.x=m/s，angular.z=舵机角(度)，BEST_EFFORT
  发布 /move_base/stop_signal  1=放行云台瞄准射击

行为差异（结构更优，均向后兼容）：
1. 待命静默：无路线时不发 pursuitAngle（不像 legacy 一启动就 20Hz 保活
   把 dzactuator 钉死模式2）—— 朝向对齐（align）可在 dznav 启动后任意
   时刻进行，两阶段启动不再是硬约束；配置 keepalive_idle=true 可还原
   legacy 行为。
2. 终点朝向用"顶点 ARC 机动"（低速满舵 + yaw 停止 + 软锚点），
   不做回退插点 —— 根治图缘顶点过冲出图（docs/SIL发现-PIVOT过冲）。
3. 到点半径收紧到 12cm（legacy 15.7cm > 排雷触发半径 6cm 是实锤缺陷）。
4. 越界急停沿用 0.60m 余量，但为闩锁态：reset 前 refuses to move。

环境变量：
  DZ_AUTO_START=0   待命模式（默认，实车调试安全约定）
  DZ_AUTO_START=1   起栈 auto_delay 秒后自动跑 config.builtin_route（比赛用）
  DZ_NAV_CONFIG     yaml 覆盖路径（可选）
"""

from __future__ import annotations

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8, Int32
from sensor_msgs.msg import Imu

from .contract import (BLOCK, block_to_meters, HEADING_YAWS, RE_HEADING,
                       RE_BLOCK, in_field, OOB_MARGIN, TOPIC_WAYPOINT,
                       TOPIC_ODOM_RAW, TOPIC_IMU, TOPIC_ODOM_LOC,
                       TOPIC_PURSUIT, TOPIC_STOP_SIGNAL, TOPIC_SHOOT_COUNT,
                       wrap_pi)
from .fusion import Localizer
from .pursuit import Tracker, TrackerConfig
from .refpath import (ReferencePath, Waypoint, build_reference, PathOptions)
from .servo import ServoModel

CMD_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

DEFAULTS = {
    # servo
    "servo_mode": "ideal",
    "servo_limit_deg": 30.0,
    "wheelbase": 0.157,
    "steer_trim_deg": 0.0,
    # tracker
    "v_max": 0.30,
    "a_lat_max": 0.22,
    # mission
    "nav_timeout": 180.0,
    "shoot_target": 10,
    "shoot_timeout": 150.0,
    "blend_anchor": 0.5,       # 途经点位置锚增益
    "blend_th": 0.25,          # 途经点航向锚增益（小于位置：防跟踪滞后污染）
    "anchor_delay_m": 0.35,    # 出弯后再锚（跟踪收敛区，测量干净）
    "blend_arc": 0.7,
    "keepalive_idle": False,
    "auto_delay": 8.0,
    "builtin_route": "",
    # path opts
    "dev_cap": 0.08,
    "dev_cap_critical": 0.03,
    "arc_fallback_deg": 125.0,
}

#: 起点约定（N10 格心，对齐原版 serial_reader START）
START_BLOCK = (1.5, 0.5)


def _deep_update(base: dict, extra: dict) -> dict:
    for k, v in (extra or {}).items():
        if k in base:
            base[k] = v
    return base


class DzNavNode(Node):
    def __init__(self, overrides: dict | None = None):
        super().__init__("dznav_node")
        cfg = dict(DEFAULTS)
        cfg_path = os.environ.get("DZ_NAV_CONFIG", "")
        if cfg_path and os.path.isfile(cfg_path):
            try:
                import yaml
                with open(cfg_path, "r", encoding="utf-8") as f:
                    _deep_update(cfg, yaml.safe_load(f) or {})
            except Exception as e:  # noqa: BLE001 - 配置坏不致命
                self.get_logger().warn(f"配置加载失败({e})，用默认值")
        if overrides:
            _deep_update(cfg, overrides)
        self.cfg = cfg

        self.servo = ServoModel(steer_limit_deg=cfg["servo_limit_deg"],
                                wheelbase=cfg["wheelbase"],
                                mode=cfg["servo_mode"])
        tcfg = TrackerConfig(v_max=cfg["v_max"], a_lat_max=cfg["a_lat_max"],
                             steer_trim_deg=cfg["steer_trim_deg"])
        self.tcfg = tcfg
        self.popts = PathOptions(dev_cap=cfg["dev_cap"],
                                 dev_cap_critical=cfg["dev_cap_critical"],
                                 arc_fallback_deg=cfg["arc_fallback_deg"])
        self.loc = Localizer(start_block=START_BLOCK)

        # ---- 通信 ----
        self.create_subscription(Odometry, TOPIC_ODOM_RAW,
                                 self._cb_odom, 10)
        self.create_subscription(Imu, TOPIC_IMU, self._cb_imu, 10)
        self.create_subscription(String, TOPIC_WAYPOINT,
                                 self._cb_waypoint, 10)
        self.create_subscription(Int32, TOPIC_SHOOT_COUNT,
                                 self._cb_shoot, 10)
        self.pub_loc = self.create_publisher(Odometry, TOPIC_ODOM_LOC, 10)
        self.pub_cmd = self.create_publisher(Twist, TOPIC_PURSUIT, CMD_QOS)
        self.pub_stop = self.create_publisher(UInt8, TOPIC_STOP_SIGNAL, 10)

        # ---- 状态机 ----
        self.state = "STANDBY"    # STANDBY|ACTIVE|SHOOT_WAIT|FINAL_ARC|ARRIVED|ESTOP
        self.tracker: Tracker | None = None
        self.final_heading: float | None = None
        self.route_label = ""
        self.nav_start = None
        self.shoot_at_arrival = 0
        self.shoot_wait_start = None
        self.shoot_count = 0
        self._final_arc = None
        self._has_odom = False
        self._last_cmd = (0.0, 0.0)
        self._t = 0.0
        self._dt = 0.02
        self._pending_anchor = None    # 出弯延迟锚点 (s_target,)

        self.create_timer(0.02, self._control)
        self.create_timer(3.0, self._check_pubs)

        auto = os.environ.get("DZ_AUTO_START", "0") == "1"
        if auto and cfg.get("builtin_route"):
            self.create_timer(float(cfg["auto_delay"]), self._auto_mission)

        self.get_logger().info(
            f"dznav 就绪 [待命] servo={cfg['servo_mode']} "
            f"v_max={cfg['v_max']} a_lat={cfg['a_lat_max']}")

    # ================================================================ 回调
    def _cb_odom(self, msg):
        p = msg.pose.pose.position
        self.loc.on_odom_raw(p.x, p.y)
        self._has_odom = True

    def _cb_imu(self, msg):
        q = msg.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.loc.on_imu(yaw, self._t)

    def _cb_shoot(self, msg):
        self.shoot_count = int(msg.data)

    def _cb_waypoint(self, msg):
        data = (msg.data or "").strip()
        try:
            if data.startswith("reset:"):
                self._cmd_reset(data[6:])
            elif data.startswith("load:"):
                self._cmd_load(data[5:].strip())
            else:
                self._cmd_route(data)
        except Exception as e:  # noqa: BLE001 - 命令解析错不致死
            self.get_logger().error(f"命令处理失败: {e}")

    # ================================================================ 命令
    def _cmd_reset(self, body: str):
        parts = [p.strip() for p in body.split(",")]
        if len(parts) < 2 or not RE_BLOCK.match(parts[0]) \
                or not RE_HEADING.match(parts[1].lower()):
            self.get_logger().error(f"reset 格式错: {body!r}")
            return
        x, y = block_to_meters(parts[0])
        th = HEADING_YAWS[parts[1].lower()]
        self._abort_route()
        self.loc.reset(x, y, th)
        self.state = "STANDBY"
        self.get_logger().info(
            f"reset 重锚 ({x:.2f},{y:.2f})m yaw={math.degrees(th):+.0f}deg")

    def _cmd_load(self, path: str):
        wps = [Waypoint(*START_BLOCK, label="START")]
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                p = [float(v) for v in line.split(",")]
                if len(p) >= 3:
                    wps.append(Waypoint(p[0], p[1]))
        if len(wps) < 2:
            self.get_logger().error(f"路径文件无有效点: {path}")
            return
        self._start_route(wps, None, path)

    def _cmd_route(self, data: str):
        import re as _re
        tokens = [t for t in _re.split(r"[\s,]+", data) if t]
        blocks, heads = [], []
        heading = None
        for tok in tokens:
            t = tok.lower()
            if RE_HEADING.match(t):
                heading = t
            elif RE_BLOCK.match(tok.upper()):
                m = RE_BLOCK.match(tok.upper())
                blocks.append((m.group(1) + m.group(2), bool(m.group(3))))
            else:
                self.get_logger().error(f"非法 token: {tok!r}")
                return
        if not blocks:
            self.get_logger().error(f"路线无途经格: {data!r}")
            return
        x0, y0, _th = self.loc.pose
        wps = [Waypoint(x0, y0, label="START")]
        for name, shoot in blocks:
            bx, by = block_to_meters(name)
            if not wps[1:] and math.hypot(x0 - bx, y0 - by) < 0.02:
                continue       # 首格即当前位（对齐原版跳过语义）
            wps.append(Waypoint(bx, by, label=name, shoot_stop=shoot))
        if len(wps) < 2:
            self.get_logger().error("首格即当前位且无后续途经格")
            return
        self._start_route(wps, heading, data)

    def _start_route(self, wps, heading, label):
        warns = []
        path = build_reference(wps, self.servo, self.popts,
                               warn=lambda m: warns.append(m))
        for w in warns:
            self.get_logger().warn(w)
        self.tracker = Tracker(path, self.servo, self.tcfg)
        self.final_heading = HEADING_YAWS[heading] if heading else None
        self.route_label = label
        self.state = "ACTIVE"
        self.nav_start = self._t
        self.tracker.set_travel(self.loc.raw_travel)
        # 首段在身后（>100°）→ 先对齐弧，避免画大回环（越界兜底前自救）
        th0 = path.samples[0][3]
        x0, y0, th_now = self.loc.pose
        e0 = wrap_pi(th0 - th_now)
        if abs(e0) > math.radians(100.0):
            from .pursuit import _AlignCorner
            self.tracker._enter_arc(_AlignCorner(1.0 if e0 > 0 else -1.0),
                                    x0, y0, th_now, self._t, [],
                                    at_vertex=False, target_th=th0)
            self.get_logger().warn(
                f"首段方向在身后({math.degrees(e0):+.0f}deg)，先对齐弧")
        self.get_logger().info(
            f"路线启动: {label} 长度{path.length:.2f}m "
            f"角数{sum(1 for c in path.corners if c.mode != 'pass')} "
            f"终点朝向={heading or '保持'}")

    def _abort_route(self):
        self.tracker = None
        self._final_arc = None

    def _auto_mission(self):
        route = self.cfg.get("builtin_route", "")
        self.get_logger().warn(f"自动发车（DZ_AUTO_START=1）: {route}")
        msg = String()
        msg.data = route
        self._cb_waypoint(msg)

    # ================================================================ 主循环
    def _control(self):
        self._t += self._dt
        self.loc.tick(self._t)
        # 信念发布（50Hz，/odom/localization 契约）
        self.pub_loc.publish(self.loc.make_odom(Odometry))

        if self.state == "ESTOP":
            self._publish(0.0, 0.0)
            return
        if self.state in ("STANDBY",):
            if self.cfg.get("keepalive_idle"):
                self._publish(0.0, 0.0)
            return
        if self.state == "ARRIVED":
            self._publish(0.0, 0.0)
            return

        # 越界闩锁
        x, y, th = self.loc.pose
        if not in_field(x, y, OOB_MARGIN):
            self.state = "ESTOP"
            self.get_logger().error(
                f"越界急停 @({x:.2f},{y:.2f}) —— reset:BLOCK,HEAD 后恢复")
            self._publish(0.0, 0.0)
            return

        if self.state == "SHOOT_WAIT":
            self._publish(0.0, 0.0)
            self._shoot_wait_step()
            return
        if self.state == "FINAL_ARC":
            self._final_arc_step()
            return

        # ACTIVE
        if self.nav_start is not None and \
                self._t - self.nav_start > self.cfg["nav_timeout"]:
            self.get_logger().error("路线超时，强制收车")
            self._finish()
            return
        if self.tracker is None:
            return
        self.tracker.set_travel(self.loc.raw_travel)
        cmd, events = self.tracker.update(x, y, th, self._dt, self._t)
        for e in events:
            self._handle_event(e)
            if self.state != "ACTIVE":
                break
        # 出弯延迟锚点：跟踪收敛后测量才干净。
        # 新鲜度窗口：ARC 机动/停站会让 s_prog 冻结后一次跳过目标点，
        # 跳过 0.2m 以上视为过期测量，丢弃（SIL 实锤：过期锚点落在 ARC
        # 顶点航向不连续区，曾注入 -33° 假航向修正）。
        if (self._pending_anchor is not None and self.tracker is not None
                and self.tracker.s_prog >= self._pending_anchor):
            if self.tracker.s_prog - self._pending_anchor <= 0.20:
                px, py, pth, _k = self.tracker.path.at_s(
                    self._pending_anchor)
                self.loc.anchor(px, py, pth, blend=self.cfg["blend_anchor"],
                                blend_th=self.cfg["blend_th"])
            self._pending_anchor = None
        if self.state == "ACTIVE":
            self._publish(cmd.v, cmd.steer_deg)

    # ---- 事件 ----
    def _handle_event(self, e: dict):
        ev = e["ev"]
        if ev == "pass_wp":
            # 延迟锚点：过途经点后 anchor_delay_m 再锚（出弯收敛区）。
            # 弧顶/弯中锚定会被跟踪滞后污染（SIL 实测逐弯累积 0.4m）。
            # ARC 机动顶点存在航向不连续（入腿/出腿方向跳变），
            # 锚点落在其 ±0.35m 邻域时推到机动出口之后。
            path = self.tracker.path
            i = min(e["idx"], len(path.waypoints) - 1)
            s_t = min(path.vertex_s[i] + self.cfg["anchor_delay_m"],
                      path.length)
            for c in path.corners:
                if c.mode == "arc" and abs(s_t - path.vertex_s[c.idx]) < 0.35:
                    s_t = max(s_t, path.vertex_s[c.idx] + 0.40)
                    break
            self._pending_anchor = max(self._pending_anchor or 0.0, s_t)
        elif ev == "shoot_stop":
            wps = self.tracker.path.waypoints
            w = wps[min(e["idx"], len(wps) - 1)]
            self.loc.anchor(w.x, w.y, self.loc.th, blend=0.3, blend_th=0.0)
            self.state = "SHOOT_WAIT"
            self.shoot_at_arrival = self.shoot_count
            self.shoot_wait_start = self._t
            sig = UInt8()
            sig.data = 1
            self.pub_stop.publish(sig)
            self.get_logger().info(
                f"射击停站 {w.label}: 等 {self.cfg['shoot_target']} 发")
        elif ev == "arrived":
            self._on_arrived(e.get("d", 0.0))
        elif ev == "arc_exit":
            self.loc.anchor(e["x"], e["y"], e["th"],
                            blend=self.cfg["blend_arc"],
                            blend_th=self.cfg["blend_th"])
        elif ev == "rebuild":
            self.get_logger().warn(
                f"偏差过大重建路线 → wp#{e['to_idx']}（{self.tracker.rebuilds}"
                f"/{self.tcfg.rebuild_limit}）")
        elif ev == "align_done":
            self.loc.anchor(self.loc.x, self.loc.y, e["th"], blend=0.5)

    def _on_arrived(self, d):
        fw = self.tracker.path.waypoints[-1]
        if self.final_heading is not None:
            err = wrap_pi(self.final_heading - self.loc.th)
            if abs(err) > math.radians(12.0) and self._final_arc_feasible():
                self._final_arc = {
                    "sign": 1.0 if err > 0 else -1.0,
                    "target": self.final_heading,
                    "x": fw.x, "y": fw.y,
                    "t0": self._t,
                }
                self.state = "FINAL_ARC"
                self.get_logger().info(
                    f"到点(d={d:.2f}m)，终点朝向机动 "
                    f"{math.degrees(err):+.0f}deg")
                return
            if abs(err) > math.radians(12.0):
                self.get_logger().warn(
                    "终点朝向机动不可行（图缘），保持当前朝向收车")
        self._finish()

    def _final_arc_feasible(self) -> bool:
        fw = self.tracker.path.waypoints[-1]
        th = self.final_heading
        ex = fw.x + 0.15 * math.cos(th)
        ey = fw.y + 0.15 * math.sin(th)
        err = wrap_pi(th - self.loc.th)
        side = 1.0 if err > 0 else -1.0
        mx = fw.x + 0.30 * side * (-math.sin(th))
        my = fw.y + 0.30 * side * (math.cos(th))
        return in_field(ex, ey, 0.10) and in_field(mx, my, 0.05)

    def _final_arc_step(self):
        a = self._final_arc
        err = wrap_pi(a["target"] - self.loc.th)
        v = self.tcfg.arc_v
        steer = a["sign"] * (self.servo.steer_limit_deg - 2.0)
        done = a["sign"] * err <= math.radians(self.tcfg.arc_lead_deg)
        timeout = self._t - a.get("t0", self._t) > 12.0
        if done or timeout:
            self.loc.anchor(a["x"], a["y"], a["target"], blend=0.7,
                            blend_th=self.cfg["blend_th"])
            self._final_arc = None
            self._finish()
            return
        self._publish(v, steer)

    def _shoot_wait_step(self):
        done = (self.shoot_count - self.shoot_at_arrival
                >= int(self.cfg["shoot_target"]))
        timeout = self._t - self.shoot_wait_start > self.cfg["shoot_timeout"]
        if done or timeout:
            sig = UInt8()
            sig.data = 0
            self.pub_stop.publish(sig)
            self.get_logger().info(
                "射击完成" if done else "射击超时，继续路线")
            self.tracker.resume()
            self.state = "ACTIVE"
            self.nav_start = self._t     # 停站不占路线预算

    def _finish(self):
        self.state = "ARRIVED"
        self._publish(0.0, 0.0)
        sig = UInt8()
        sig.data = 1
        self.pub_stop.publish(sig)
        x, y, th = self.loc.pose
        self.get_logger().info(
            f"ARRIVED ({x:.2f},{y:.2f})m yaw={math.degrees(th):+.0f}deg "
            f"stop_signal=1")

    # ---- 工具 ----
    def _publish(self, v: float, steer_deg: float):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(steer_deg)
        self.pub_cmd.publish(m)
        self._last_cmd = (v, steer_deg)

    def _check_pubs(self):
        try:
            n = self.count_publishers(TOPIC_ODOM_LOC)
            if n > 1:
                self.get_logger().error(
                    f"/odom/localization 有 {n} 个发布者！"
                    "serial_reader 还在跑？位姿会被污染，先停掉它")
        except Exception:  # noqa: BLE001 - 桩环境无总线
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DzNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        m = Twist()
        node.pub_cmd.publish(m)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
