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
5. 【2026-10-17 评审整改】命令出口 isfinite 双检 + 急停闩锁：上游任何
   NaN 都会被 pursuit.py 的 max(-L, min(L, nan)) 放大成 +L 满舵（Python
   语义实锤），现改为落 (0,0) 并闩锁 ESTOP。
6. 【同上】传感器链路看门狗（link_timeout，默认 0.5s）：盲跑不再依赖
   180s nav_timeout 兜底；仅在"会动"的状态下跳闸，待命期不误伤。
7. 【同上】SIGTERM 显式归零：dznav_deploy.py 用 pkill 收车，SIGTERM 默认
   处置不展开 Python 栈，原 finally 归零根本不会执行。
8. 【同上】控制周期 dt 取真实时钟：循环被饿死时加速度限幅不再失效。

环境变量：
  DZ_AUTO_START=0   待命模式（默认，实车调试安全约定）
  DZ_AUTO_START=1   起栈 auto_delay 秒后自动跑 config.builtin_route（比赛用）
  DZ_NAV_CONFIG     yaml 覆盖路径（可选）
"""

from __future__ import annotations

import math
import os
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8, Int32, Bool
from sensor_msgs.msg import Imu

from .contract import (BLOCK, block_to_meters, HEADING_YAWS, RE_HEADING,
                       RE_BLOCK, in_field, OOB_MARGIN, TOPIC_WAYPOINT,
                       TOPIC_ODOM_RAW, TOPIC_IMU, TOPIC_ODOM_LOC,
                       TOPIC_PURSUIT, TOPIC_STOP_SIGNAL, TOPIC_SHOOT_COUNT,
                       TOPIC_ESTOP, wrap_pi)
from .fusion import Localizer
from .pursuit import Tracker, TrackerConfig
from .refpath import (ReferencePath, Waypoint, build_reference, PathOptions)
from .servo import ServoModel

CMD_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
#: 急停用 TRANSIENT_LOCAL：晚启动的节点也能立刻收到已闩锁的急停状态。
ESTOP_QOS = QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST)

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
    # ---- 安全（2026-10-17 评审整改新增）----
    "link_timeout": 0.5,       # /odom 或 /imu_data 断流多久跳闸（仅运动态）
    "dt_max": 0.20,            # 单周期 dt 上限：防挂起后一次性大跳
}

#: 起点约定（N10 格心，对齐原版 serial_reader START）
START_BLOCK = (1.5, 0.5)

#: 会真正输出非零速度的状态：链路看门狗只在这些状态下跳闸。
_MOVING_STATES = frozenset({"ACTIVE", "SHOOT_WAIT", "FINAL_ARC"})


def _deep_update(base: dict, extra: dict, log=None) -> dict:
    """就地覆盖 base 中已存在的键。未知键告警而非静默丢弃。

    静默丢弃是实锤运维坑：yaml 里把 a_lat_max 写成 alat_max，配置看起来
    生效了、实际跑的是默认值，现场无从发现。
    """
    unknown = []
    for k, v in (extra or {}).items():
        if k in base:
            base[k] = v
        else:
            unknown.append(k)
    if unknown and log is not None:
        log(f"配置含未知键，已忽略: {sorted(unknown)}")
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
                    _deep_update(cfg, yaml.safe_load(f) or {},
                                 log=self.get_logger().warning)
                self.get_logger().info(f"配置已载入: {cfg_path}")
            except Exception as e:  # noqa: BLE001 - 配置坏不致命
                self.get_logger().warning(f"配置加载失败({e})，用默认值")
        elif cfg_path:
            # 指了路径但文件不存在：几乎一定是部署脚本传错，必须喊出来。
            self.get_logger().error(
                f"DZ_NAV_CONFIG 指向的文件不存在: {cfg_path} —— 用默认值")
        if overrides:
            _deep_update(cfg, overrides, log=self.get_logger().warning)
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
        self.create_subscription(Bool, TOPIC_ESTOP, self._cb_estop, ESTOP_QOS)
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
        self._last_cmd = (0.0, 0.0)
        self._t = 0.0
        self._dt_nominal = 0.02
        self._dt = self._dt_nominal
        self._pending_anchor = None    # 出弯延迟锚点 (s_target,)

        # ---- 安全（2026-10-17 评审整改）----
        #: ESTOP 原因。"cmd" 只能由 /estop=False 解除，reset: 不可解除；
        #: 其余（nan/oob/link）可由 reset: 解除，与既有越界闩锁语义一致。
        self._estop_reason = ""
        self._estop_cmd = False        # /estop 话题闩锁位（跨 reset 保持）
        self._wall_prev = self._now_s()
        self._odom_last_arrival = None   # 本地时钟，仅用于断流判定
        self._imu_last_arrival = None
        self._odom_last_stamp = None     # 消息头时间戳，仅用于乱序剔除
        self._imu_last_stamp = None
        self._imu_seen = False
        self._odom_seen = False

        self.create_timer(self._dt_nominal, self._control)
        self.create_timer(3.0, self._check_pubs)

        auto = os.environ.get("DZ_AUTO_START", "0") == "1"
        if auto and cfg.get("builtin_route"):
            self.create_timer(float(cfg["auto_delay"]), self._auto_mission)

        self.get_logger().info(
            f"dznav 就绪 [待命] servo={cfg['servo_mode']} "
            f"v_max={cfg['v_max']} a_lat={cfg['a_lat_max']} "
            f"link_timeout={cfg['link_timeout']}s")

    # ---- 时钟 ----
    def _now_s(self) -> float:
        """本地单调秒（节点时钟）。不用消息头时间戳：车队无 NTP 同步，
        跨设备比较墙上时钟会得出荒谬的新鲜度结论。"""
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_s(msg) -> float | None:
        """取消息头时间戳（秒）。戳缺失/为 0/类型异常一律返回 None，
        调用方据此退化为"不做乱序检查"——宁可不检查，也不能丢帧。"""
        try:
            st = msg.header.stamp
            v = st.sec + st.nanosec * 1e-9
        except (AttributeError, TypeError, ValueError):
            return None
        return v if v > 0.0 else None

    # ================================================================ 回调
    def _cb_odom(self, msg):
        stamp = self._stamp_s(msg)
        # 乱序剔除：只与"已接受的最新戳"比较，不碰墙上时钟，
        # 因此对无 NTP 同步/重启回跳完全免疫。戳为 0 或缺失时自动退化为不检查。
        if stamp is not None and self._odom_last_stamp is not None \
                and stamp < self._odom_last_stamp:
            return
        p = msg.pose.pose.position
        if not (math.isfinite(p.x) and math.isfinite(p.y)):
            self.get_logger().error(
                f"/odom 位置非有限值 ({p.x},{p.y})，丢弃该帧", throttle_duration_sec=2.0)
            return
        self._odom_last_stamp = stamp
        self._odom_last_arrival = self._now_s()
        self._odom_seen = True
        self.loc.on_odom_raw(p.x, p.y)

    def _cb_imu(self, msg):
        stamp = self._stamp_s(msg)
        if stamp is not None and self._imu_last_stamp is not None \
                and stamp < self._imu_last_stamp:
            return
        q = msg.orientation
        n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if not math.isfinite(n2) or n2 <= 1e-6:
            self.get_logger().error(
                f"/imu_data 四元数非法 (|q|^2={n2})，丢弃该帧",
                throttle_duration_sec=2.0)
            return
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._imu_last_stamp = stamp
        self._imu_last_arrival = self._now_s()
        self._imu_seen = True
        self.loc.on_imu(yaw, self._t)

    def _cb_shoot(self, msg):
        self.shoot_count = int(msg.data)

    def _cb_estop(self, msg):
        """外部急停。True=闩锁停车（reset: 不可解除，必须显式发 False）。

        设计取舍：让 reset: 能解除操作员的急停是 fail-dangerous 的
        —— 现场慌乱中一条重锚命令就会悄悄放行走错方向的车。
        """
        if bool(msg.data):
            if not self._estop_cmd:
                self.get_logger().error("/estop=True 收到，急停闩锁")
            self._estop_cmd = True
            self._enter_estop("cmd")
        else:
            if self._estop_cmd:
                self.get_logger().warning("/estop=False 收到，解除急停闩锁")
            self._estop_cmd = False
            if self.state == "ESTOP" and self._estop_reason == "cmd":
                self._estop_reason = ""
                self.state = "STANDBY"

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
        if self._estop_cmd:
            self.state = "ESTOP"
            self._estop_reason = "cmd"
            self.get_logger().error(
                f"reset 重锚 ({x:.2f},{y:.2f})m，但 /estop 仍闩锁 —— "
                "需先发 /estop=False 才能放行")
            return
        cleared = self._estop_reason
        self._estop_reason = ""
        # 链路看门狗重新给一个完整窗口：reset 前若刚好断流，
        # 不应在放行的第一个周期就立刻再跳闸。
        now = self._now_s()
        self._odom_last_arrival = now if self._odom_seen else None
        self._imu_last_arrival = now if self._imu_seen else None
        self.state = "STANDBY"
        self.get_logger().info(
            f"reset 重锚 ({x:.2f},{y:.2f})m yaw={math.degrees(th):+.0f}deg"
            + (f"，解除 {cleared} 急停闩锁" if cleared else ""))

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
        blocks = []
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
        # 放行前传感器门禁：没有 IMU 就没有航向，跟踪器的 e_th 是垃圾；
        # 没有里程就没有弧长，纯跟踪的 lookahead 也是垃圾。旧版直接放行，
        # 结果是"路线启动了、车按错误航向满舵跑"，现场极难归因。
        if self._estop_cmd:
            self.get_logger().error(
                f"拒绝路线 {label}: /estop 仍闩锁，需先发 /estop=False")
            return
        missing = [n for n, ok in (("/imu_data", self._imu_seen),
                                   ("/odom", self._odom_seen)) if not ok]
        if missing:
            self.get_logger().error(
                f"拒绝路线 {label}: 尚未收到 {missing}，无定位输入")
            return
        warns = []
        path = build_reference(wps, self.servo, self.popts,
                               warn=lambda m: warns.append(m))
        for w in warns:
            self.get_logger().warning(w)
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
            self.get_logger().warning(
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
        self.get_logger().warning(f"自动发车（DZ_AUTO_START=1）: {route}")
        msg = String()
        msg.data = route
        self._cb_waypoint(msg)

    # ================================================================ 主循环
    def _control(self):
        # dt 取真实时钟差而非假定 50Hz：循环被 CPU 饿死时，
        # pursuit.py 的 v_cmd += clamp(dv, ±accel*dt) 会按错误的 dt 放行
        # 过大的速度阶跃，加速度限幅形同虚设。
        now_wall = self._now_s()
        dt = now_wall - self._wall_prev
        self._wall_prev = now_wall
        if not math.isfinite(dt) or dt <= 0.0:
            dt = self._dt_nominal        # 时钟异常：退回标称步长
        elif dt > self.cfg["dt_max"]:
            # 进程被挂起过。不要用一个巨大的 dt 让限幅失效，
            # 也不要假装时间没走 —— 交给链路看门狗（墙上时钟）去跳闸。
            dt = self.cfg["dt_max"]
        self._dt = dt
        self._t += dt
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

        # 链路看门狗：只在"会动"的状态下跳闸（待命期传感器抖动不误伤）。
        # 旧版唯一的兜底是 nav_timeout=180s —— 盲跑三分钟才收车，
        # 在 3.85x4.90m 的场地上等于必然出图/撞障。
        if self.state in _MOVING_STATES:
            stale = self._link_stale(now_wall)
            if stale:
                self._enter_estop("link",
                                  f"传感链路断流 {stale} > "
                                  f"{self.cfg['link_timeout']}s，急停")
                return

        # 越界闩锁
        x, y, th = self.loc.pose
        if not in_field(x, y, OOB_MARGIN):
            self._enter_estop("oob",
                              f"越界急停 @({x:.2f},{y:.2f})m")
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
            self.get_logger().warning(
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
                self.get_logger().warning(
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
    def _link_stale(self, now: float) -> str:
        """返回断流的输入名（空串=链路健康）。用本地到达时刻判定，
        不比较消息头时间戳——车队无 NTP，跨设备墙上时钟不可信。"""
        lim = float(self.cfg["link_timeout"])
        bad = []
        if self._imu_last_arrival is None \
                or (now - self._imu_last_arrival) > lim:
            bad.append("/imu_data")
        if self._odom_last_arrival is None \
                or (now - self._odom_last_arrival) > lim:
            bad.append("/odom")
        return "+".join(bad)

    def _enter_estop(self, reason: str, detail: str = "") -> None:
        """闩锁急停：归零 + 收回云台放行位 + 丢弃路线。

        丢弃路线是有意的——ESTOP 后唯一的出路是 reset: 重锚，
        留着一个 tracker 只会让"resume 到旧路线"这种幽灵行为有机可乘。
        """
        transition = (self.state != "ESTOP"
                      or self._estop_reason != reason)
        self.state = "ESTOP"
        self._estop_reason = reason
        self._abort_route()
        self._publish(0.0, 0.0)
        sig = UInt8()
        sig.data = 0
        self.pub_stop.publish(sig)
        if transition:
            hint = ("/estop=False 解除" if reason == "cmd"
                    else "reset:BLOCK,HEAD 后恢复")
            self.get_logger().error(
                f"ESTOP[{reason}] {detail} —— {hint}")

    def _publish(self, v: float, steer_deg: float):
        """最后一道闸：命令出口前做有限性与幅值双检。

        为什么必须在这里再检一次——pursuit.py:180 的钳位写法是
            steer = max(-L, min(L, steer))
        CPython 的 min/max 遇 NaN 返回**第一个实参**，实测：
            min(30.0, nan) -> 30.0；max(-30.0, 30.0) -> 30.0
        即上游任何一个 NaN 都会被"钳位"成 **+L 满舵**，而速度项照常，
        车会以最高速打死方向。这不是理论问题：refpath/fusion 里任何一处
        除零、asin 越界或 atan2(0,0) 都能产出 NaN。
        """
        v = float(v)
        steer_deg = float(steer_deg)
        if not (math.isfinite(v) and math.isfinite(steer_deg)):
            self._enter_estop(
                "nan", f"命令含非有限值 v={v} steer={steer_deg}，落 (0,0)")
            self._last_cmd = (0.0, 0.0)
            return
        lim = float(self.servo.steer_limit_deg)
        if abs(steer_deg) > lim:
            self.get_logger().warning(
                f"舵角 {steer_deg:+.1f}deg 超出行程 ±{lim:.1f}deg，已钳位"
                "（上游钳位失效，请查 pursuit/refpath）",
                throttle_duration_sec=2.0)
            steer_deg = lim if steer_deg > 0.0 else -lim
        v_lim = float(self.cfg["v_max"])
        if abs(v) > v_lim:
            self.get_logger().warning(
                f"速度 {v:+.3f}m/s 超出 v_max={v_lim:.3f}，已钳位",
                throttle_duration_sec=2.0)
            v = v_lim if v > 0.0 else -v_lim
        m = Twist()
        m.linear.x = v
        m.angular.z = steer_deg
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


def _sigterm_handler(_signum, _frame):
    """SIGTERM 的默认处置是"直接终止"，不展开 Python 栈，所以 finally
    里的归零根本不会执行 —— 而 dznav_deploy.py:137,152 正是用 pkill
    （SIGTERM）收车的。抛 SystemExit 把控制权交回 finally。

    注：dzactuator 自带 ACTIVITY_TIMEOUT=0.5s，pursuitAngle 断流 0.5s
    会自行失活，所以这不是"车永不停"，而是"车还会带着最后一次的舵角
    再跑 ~15cm，且 stop_signal 停留在 1（云台继续开火）"。
    """
    raise SystemExit(0)


def _zero_and_stop(node) -> None:
    """归零油门 + 收回云台放行位。

    pursuitAngle 是 BEST_EFFORT/depth=1，单发有丢包可能，重发几次是
    廉价保险；stop_signal 必须回 0，否则进程死了云台还认为可以开火。
    """
    sig = UInt8()
    sig.data = 0
    for _ in range(3):
        try:
            node.pub_cmd.publish(Twist())
            node.pub_stop.publish(sig)
        except Exception:  # noqa: BLE001 - 退出路径上不能抛
            break
        time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = DzNavNode()

    try:
        signal.signal(signal.SIGTERM, _sigterm_handler)
    except (ValueError, OSError):   # 非主线程/平台不支持：不致命
        node.get_logger().warning("无法注册 SIGTERM 处理器，退出归零可能失效")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        node.get_logger().warning("收到 SIGTERM，归零收车")
    finally:
        _zero_and_stop(node)
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
