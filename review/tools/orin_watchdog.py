#!/usr/bin/env python3
"""机载自治守卫（watchdog）：出了问题机上自己介入，不等地面、不等链路。

动机（2026-10-16 审查/实飞日志驱动）：
  下午的 OFFBOARD 退出×3 证明"地面看见 → 人按键 → SSH → 机上发令"这条链太慢、
  且受 NCM 链路抖动制约。本进程常驻机上，直接订阅飞控话题做判定，
  介入命令机上本地发出（DDS 延迟毫秒级），不经过笔记本。

介入分级（保守优先，全部留痕）：
  LAND  走 px4_control 安全通道（状态机守门：空中拒 DISARM 那套防护都在）
  KILL  直发 /fmu/in/vehicle_command（400 p1=0 p2=21196 强制断解锁=电机停转，
        空中=坠机，默认关闭，必须 --enable-kill 显式授权）

规则（armed+空中才介入；每条规则有持续判定+冷却；detailed 见 WatchdogRules）：
  BLIND          遥测断流 >1.5s（armed 却看不见飞机）→ LAND
  OFFBOARD_LOST  空中掉出 OFFBOARD 且非 LAND/RTL 持续 1.5s → LAND
                 （--no-land-on-offboard-loss 可关；飞手接管场景想留人工就关）
  POS_JUMP       位置连续 2 帧水平跳变 >2m（EKF 被 SLAM 跳变污染征兆）→ LAND
  TILT_RUNAWAY   |roll|/|pitch|>60° 持续 0.8s → KILL（仅 --enable-kill；否则只告警）
  BATT_CRIT      空中电量 <10% → LAND（加固 PX4 自身电池 failsafe）
  FAILSAFE       failsafe 置位持续 0.5s → LAND（加固，PX4 自己也在动作）

用法（Orin 上，一般由 tmux 窗口常驻）：
    python3 orin_watchdog.py                    # LAND 级介入开启，KILL 关闭
    python3 orin_watchdog.py --enable-kill      # 授权 KILL（倾覆狂奔时电机停转）
    python3 orin_watchdog.py --dry-run          # 只判定+记录，不发任何命令
    python3 orin_watchdog.py --check            # 打印订阅注册情况后退出（部署自检）

只依赖 rclpy + px4_msgs/px4_control + 标准库。所有判定与介入写 JSONL
（~/ag_flight_logs/watchdog/<会话>/watchdog-<pid>.jsonl，事件级 fsync）。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path

NODE_NAME = "flight_wd"

NAV_OFFBOARD, NAV_RTL, NAV_LAND, NAV_PRECLAND = 14, 5, 8, 10
LANDING_STATES = (NAV_LAND, NAV_RTL, NAV_PRECLAND)


# ---------------------------------------------------------------- 纯规则引擎（可单测）

class WatchdogRules:
    """无人机自治介入规则状态机（纯逻辑，不碰 ROS；照 UavRuleEngine 范式）。

    每拍喂一个快照 dict（键可缺省）：
      t: float               当前时刻（monotonic 或 epoch，同一钟即可）
      armed: bool|None       vehicle_status.arming_state==2
      nav_state: int|None
      failsafe: bool|None
      z: float|None          NED 高度（空中 <-0.2 视为 airborne）
      roll_deg/pitch_deg: float|None
      batt_remaining: float|None  0..1
      last_seen_t: float|None     最近一次任一遥测消息时刻（断流判定用）
      pos_jump: bool|None    本帧水平跳变超阈（ROS 壳按相邻帧算好传入）

    返回动作列表 [{"action": "LAND"|"KILL"|"ALERT", "rule": str, "reason": str}]。
    """

    AIRBORNE_Z = -0.2        # NED：z 小于此值视为离地

    def __init__(self, enable_kill: bool = False, land_on_offboard_loss: bool = True,
                 blind_timeout_s: float = 1.5, offboard_loss_sustain_s: float = 1.5,
                 jump_streak_need: int = 2, tilt_kill_deg: float = 60.0,
                 tilt_sustain_s: float = 0.8, batt_crit: float = 0.10,
                 failsafe_sustain_s: float = 0.5, action_cooldown_s: float = 10.0):
        self.enable_kill = enable_kill
        self.land_on_offboard_loss = land_on_offboard_loss
        self.blind_timeout_s = blind_timeout_s
        self.offboard_loss_sustain_s = offboard_loss_sustain_s
        self.jump_streak_need = jump_streak_need
        self.tilt_kill_deg = tilt_kill_deg
        self.tilt_sustain_s = tilt_sustain_s
        self.batt_crit = batt_crit
        self.failsafe_sustain_s = failsafe_sustain_s
        self.action_cooldown_s = action_cooldown_s
        self._last_action: dict[str, float] = {}
        self._since: dict[str, float] = {}
        self._jump_streak = 0
        self._fired_this_flight: set[str] = set()   # 每段飞行每规则只介入一次

    # ---- 内部 ----
    def _cooled(self, rule: str, t: float) -> bool:
        return t - self._last_action.get(rule, -1e18) >= self.action_cooldown_s

    def _sustained(self, key: str, t: float, cond: bool, need: float) -> bool:
        if not cond:
            self._since.pop(key, None)
            return False
        since = self._since.setdefault(key, t)
        return t - since >= need

    def _fire(self, actions, action, rule, t, reason, once_per_flight=True):
        if once_per_flight and rule in self._fired_this_flight:
            return
        if not self._cooled(rule, t):
            return
        self._last_action[rule] = t
        self._fired_this_flight.add(rule)
        actions.append({"action": action, "rule": rule, "reason": reason})

    # ---- 主入口 ----
    def update(self, s: dict) -> list[dict]:
        t = float(s.get("t", time.monotonic()))
        armed = bool(s.get("armed"))
        if not armed:
            # 落地/上锁：清飞行内锁存、持续计时与冷却——下一段飞行重新武装，
            # 上一段飞过的介入不能压制下一段的介入（冷却也按飞行计）
            self._fired_this_flight.clear()
            self._since.clear()
            self._jump_streak = 0
            self._last_action.clear()
            return []
        z = s.get("z")
        airborne = z is not None and z < self.AIRBORNE_Z
        actions: list[dict] = []

        # BLIND：armed 且曾看见过、现在断流（从未见过=栈还在起，不介入）
        last_seen = s.get("last_seen_t")
        if airborne and last_seen is not None \
                and t - last_seen > self.blind_timeout_s:
            self._fire(actions, "LAND", "BLIND", t,
                       f"遥测断流 {t - last_seen:.1f}s（armed 空中看不见飞机）")

        # OFFBOARD_LOST：空中掉出 OFFBOARD 且不在降落类状态
        nav = s.get("nav_state")
        obl_cond = airborne and nav is not None \
            and nav not in (NAV_OFFBOARD, *LANDING_STATES)
        if self.land_on_offboard_loss and self._sustained(
                "obl", t, obl_cond, self.offboard_loss_sustain_s):
            self._fire(actions, "LAND", "OFFBOARD_LOST", t,
                       f"空中掉出 OFFBOARD（nav_state={nav}）持续 "
                       f"{self.offboard_loss_sustain_s:.1f}s，接管加固为降落")

        # POS_JUMP：相邻帧水平跳变（ROS 壳算好 pos_jump 传入）
        self._jump_streak = self._jump_streak + 1 if s.get("pos_jump") else 0
        if airborne and self._jump_streak >= self.jump_streak_need:
            self._fire(actions, "LAND", "POS_JUMP", t,
                       f"位置连续 {self._jump_streak} 帧跳变超阈——"
                       f"定位可能被污染（SLAM 跳变/EV 拒收），先落地")

        # TILT_RUNAWAY：倾角超限持续（倾覆狂奔前停电机；未授权 KILL 时只告警）
        r, p = s.get("roll_deg"), s.get("pitch_deg")
        tilt = None
        if r is not None and p is not None:
            tilt = max(abs(r), abs(p))
        tilt_cond = airborne and tilt is not None and tilt > self.tilt_kill_deg
        if self._sustained("tilt", t, tilt_cond, self.tilt_sustain_s):
            if self.enable_kill:
                self._fire(actions, "KILL", "TILT_RUNAWAY", t,
                           f"倾角 {tilt:.0f}° 持续 {self.tilt_sustain_s:.1f}s"
                           f"（阈 {self.tilt_kill_deg:.0f}°）——电机停转止损")
            else:
                self._fire(actions, "ALERT", "TILT_RUNAWAY", t,
                           f"倾角 {tilt:.0f}° 超 {self.tilt_kill_deg:.0f}° "
                           f"持续超限（--enable-kill 未开，只告警）")

        # BATT_CRIT
        rem = s.get("batt_remaining")
        if airborne and rem is not None and rem < self.batt_crit:
            self._fire(actions, "LAND", "BATT_CRIT", t,
                       f"电量 {rem * 100:.0f}% < {self.batt_crit * 100:.0f}%（空中）")

        # FAILSAFE
        if self._sustained("fs", t, bool(s.get("failsafe")) and airborne,
                           self.failsafe_sustain_s):
            self._fire(actions, "LAND", "FAILSAFE", t,
                       "failsafe 置位持续——加固为降落（PX4 自身 failsafe 之外的双保险）")
        return actions


# ---------------------------------------------------------------- ROS 壳（Orin 上运行）

def quat_to_euler_deg(q0, q1, q2, q3):
    roll = math.degrees(math.atan2(2.0 * (q0 * q1 + q2 * q3),
                                   1.0 - 2.0 * (q1 * q1 + q2 * q2)))
    s = max(-1.0, min(1.0, 2.0 * (q0 * q2 - q3 * q1)))
    pitch = math.degrees(math.asin(s))
    yaw = math.degrees(math.atan2(2.0 * (q0 * q3 + q1 * q2),
                                  1.0 - 2.0 * (q2 * q2 + q3 * q3)))
    return roll, pitch, yaw


class WatchdogNode:
    """rclpy 壳：订阅 → 喂快照给 WatchdogRules → 机上本地发介入命令。

    与 rclpy 解耦的判定全在 WatchdogRules；这里只做 IO 与防御。
    """

    JUMP_WINDOW_S = 0.25     # 相邻帧间隔小于它才算"跳变"（采样间隔过长不算）

    def __init__(self, args):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        self.args = args
        self.rules = WatchdogRules(
            enable_kill=args.enable_kill,
            land_on_offboard_loss=not args.no_land_on_offboard_loss,
            blind_timeout_s=args.blind_timeout_s,
            jump_streak_need=2, tilt_kill_deg=args.tilt_kill_deg,
            tilt_sustain_s=args.tilt_sustain_s, batt_crit=args.batt_crit)
        self.rclpy = rclpy
        self.node = Node(NODE_NAME)
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 快照原料
        self.st: dict = {"armed": None, "nav_state": None, "failsafe": None,
                         "z": None, "roll_deg": None, "pitch_deg": None,
                         "batt_remaining": None}
        self.last_seen_t: float | None = None
        self._last_pos = None          # (t, x, y)
        self._pos_jump = False         # 本拍消费的跳变标志
        self._pub_lock = threading.Lock()
        self._stop = threading.Event()

        # 落盘
        log_dir = Path(args.log_dir).expanduser() / args.session
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_file = log_dir / f"watchdog-{os.getpid()}.jsonl"
        self.fh = open(self.log_file, "a", encoding="utf-8", buffering=1)

        # 订阅（缺类型只警告，守卫本体不垮）
        self._subs_ok: list[str] = []
        try:
            import px4_msgs.msg as px4_msg
        except ImportError:
            px4_msg = None
            self.node.get_logger().warn("缺 px4_msgs：飞控话题订阅全部跳过")
        try:
            import px4_control.msg as px4_control_msg
        except ImportError:
            px4_control_msg = None
            self.node.get_logger().warn("缺 px4_control.msg：LAND 通道不可用")

        if px4_msg is not None:
            for topic, cls_name, cb in (
                    ("/fmu/out/vehicle_status", "VehicleStatus", self._on_status),
                    ("/fmu/out/vehicle_local_position", "VehicleLocalPosition",
                     self._on_pos),
                    ("/fmu/out/vehicle_attitude", "VehicleAttitude", self._on_att),
                    ("/fmu/out/battery_status", "BatteryStatus", self._on_batt)):
                try:
                    self.node.create_subscription(getattr(px4_msg, cls_name),
                                                  topic, cb, qos_be)
                    self._subs_ok.append(topic)
                except Exception as e:  # noqa: BLE001
                    self.node.get_logger().warn(f"跳过 {topic}: {e}")

        # 介入命令发布端
        self.pub_ctrl = None
        if px4_control_msg is not None:
            self.pub_ctrl = self.node.create_publisher(
                px4_control_msg.ControlCommand, "/px4_control/command", 10)
        self.pub_vc = None
        if px4_msg is not None:
            self.pub_vc = self.node.create_publisher(
                px4_msg.VehicleCommand, "/fmu/in/vehicle_command", 10)

        self.node.create_timer(0.1, self._tick)   # 10Hz 判定拍
        self._emit("wd.ready", enable_kill=args.enable_kill,
                   dry_run=args.dry_run, subs=list(self._subs_ok))
        self.node.get_logger().info(
            f"watchdog 启动 -> {self.log_file}（kill={'开' if args.enable_kill else '关'}"
            f"{' DRY-RUN' if args.dry_run else ''}）")

    # ---- 订阅回调：只更新快照，不做判定 ----
    def _touch(self):
        self.last_seen_t = time.monotonic()

    def _on_status(self, m):
        self._touch()
        self.st["armed"] = bool(getattr(m, "arming_state", None) == 2)
        self.st["nav_state"] = getattr(m, "nav_state", None)
        self.st["failsafe"] = bool(getattr(m, "failsafe", False))

    def _on_pos(self, m):
        self._touch()
        x, y = getattr(m, "x", None), getattr(m, "y", None)
        z = getattr(m, "z", None)
        self.st["z"] = float(z) if z is not None and math.isfinite(z) else None
        now = time.monotonic()
        if x is not None and y is not None and math.isfinite(x) and math.isfinite(y):
            if self._last_pos is not None:
                pt, px, py = self._last_pos
                dt = now - pt
                if 0 < dt < self.JUMP_WINDOW_S:
                    if math.hypot(x - px, y - py) > self.args.jump_threshold_m:
                        self._pos_jump = True
            self._last_pos = (now, x, y)

    def _on_att(self, m):
        self._touch()
        q = getattr(m, "q", None)
        if q is not None and len(q) >= 4:
            r, p, _y = quat_to_euler_deg(q[0], q[1], q[2], q[3])
            self.st["roll_deg"], self.st["pitch_deg"] = r, p

    def _on_batt(self, m):
        self._touch()
        rem = getattr(m, "remaining", None)
        self.st["batt_remaining"] = \
            float(rem) if rem is not None and math.isfinite(rem) else None

    # ---- 判定拍（10Hz，定时器驱动：没消息也要能发现断流）----
    def _tick(self):
        snap = dict(self.st)
        snap["t"] = time.monotonic()
        snap["last_seen_t"] = self.last_seen_t
        snap["pos_jump"] = self._pos_jump
        self._pos_jump = False
        try:
            actions = self.rules.update(snap)
        except Exception as e:  # noqa: BLE001 - 判定异常绝不能杀掉守卫
            self.node.get_logger().error(f"规则引擎异常: {e}")
            return
        for a in actions:
            self._act(a)

    # ---- 介入执行 ----
    def _act(self, a: dict):
        action, rule, reason = a["action"], a["rule"], a["reason"]
        lvl = "error" if action == "KILL" else "warn"
        self._emit("wd.action", lvl=lvl, action=action, rule=rule, reason=reason)
        self.node.get_logger().warn(f"[介入] {action}（{rule}）：{reason}")
        if self.args.dry_run:
            self._emit("wd.suppressed", action=action, rule=rule,
                       reason="dry-run 未实际发送")
            return
        threading.Thread(target=self._burst, args=(action,), daemon=True).start()

    def _burst(self, action: str):
        """连发 10 帧 @20Hz（DDS 发现竞态/单帧丢失兜底；命令幂等）。"""
        try:
            if action == "KILL":
                if self.pub_vc is None:
                    return
                import px4_msgs.msg as px4_msg
                msg = px4_msg.VehicleCommand()
                msg.command, msg.param1, msg.param2 = 400, 0.0, 21196.0
                msg.target_system, msg.target_component = 1, 1
                msg.source_system, msg.source_component = 1, 1
                msg.from_external = True
                pub = self.pub_vc
            else:  # LAND
                if self.pub_ctrl is None:
                    return
                import px4_control.msg as px4_control_msg
                msg = px4_control_msg.ControlCommand()
                msg.cmd = 4     # CMD_LAND（px4_control 状态机守门）
                pub = self.pub_ctrl
            with self._pub_lock:
                for _ in range(10):
                    if hasattr(msg, "timestamp"):
                        msg.timestamp = int(time.time() * 1e6)
                    pub.publish(msg)
                    time.sleep(0.05)
            self._emit("wd.sent", action=action, frames=10)
        except Exception as e:  # noqa: BLE001
            self.node.get_logger().error(f"介入发送失败({action}): {e}")
            self._emit("wd.send_fail", lvl="error", action=action, error=str(e))

    # ---- 落盘 ----
    def _emit(self, ev: str, lvl: str = "info", **fields):
        rec = {"ts": round(time.time(), 3), "mono": round(time.monotonic(), 3),
               "node": NODE_NAME, "ev": ev, "lvl": lvl,
               "session": self.args.session}
        rec.update(fields)
        try:
            self.fh.write(json.dumps(rec, ensure_ascii=False,
                                     allow_nan=False) + "\n")
            self.fh.flush()
            if lvl in ("warn", "error"):
                os.fsync(self.fh.fileno())
        except (OSError, ValueError) as e:  # noqa: BLE001
            self.node.get_logger().error(f"落盘失败({ev}): {e}")

    # ---- 运行 ----
    def spin(self):
        def on_sig(_s, _f):
            self._stop.set()
        signal.signal(signal.SIGINT, on_sig)
        signal.signal(signal.SIGTERM, on_sig)
        last_hb = 0.0
        while not self._stop.is_set():
            self.rclpy.spin_once(self.node, timeout_sec=0.2)
            now = time.monotonic()
            if now - last_hb >= 3.0:
                last_hb = now
                print(f"[{NODE_NAME}] armed={self.st['armed']} "
                      f"nav={self.st['nav_state']} z={self.st['z']} "
                      f"seen={None if self.last_seen_t is None else round(now - self.last_seen_t, 1)}",
                      flush=True)
        self._emit("wd.exit", force=True)
        try:
            os.fsync(self.fh.fileno())
            self.fh.close()
        except Exception:  # noqa: BLE001
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description="机载自治守卫：出问题机上自己介入")
    ap.add_argument("--enable-kill", action="store_true",
                    help="授权 KILL（倾覆狂奔时电机立即停转=坠机止损；默认关闭）")
    ap.add_argument("--dry-run", action="store_true",
                    help="只判定+记录，不实际发任何命令")
    ap.add_argument("--no-land-on-offboard-loss", action="store_true",
                    help="掉出 OFFBOARD 时不自动降落（飞手接管演练时用这个）")
    ap.add_argument("--blind-timeout-s", type=float, default=1.5,
                    help="遥测断流介入阈值秒（默认 1.5）")
    ap.add_argument("--jump-threshold-m", type=float, default=2.0,
                    help="位置跳变阈值米（默认 2.0，需连续 2 帧）")
    ap.add_argument("--tilt-kill-deg", type=float, default=60.0,
                    help="倾覆判定角度（默认 60°）")
    ap.add_argument("--tilt-sustain-s", type=float, default=0.8,
                    help="倾覆持续秒数（默认 0.8）")
    ap.add_argument("--batt-crit", type=float, default=0.10,
                    help="空中紧急电量（默认 0.10）")
    ap.add_argument("--log-dir", type=Path,
                    default=Path("~/ag_flight_logs/watchdog"))
    ap.add_argument("--session", default=time.strftime("%Y%m%d_%H%M%S"))
    ap.add_argument("--check", action="store_true",
                    help="只打印订阅注册情况后退出（部署自检）")
    args = ap.parse_args()

    import rclpy
    rclpy.init()
    wd = WatchdogNode(args)
    if args.check:
        subs = [s.topic_name for s in wd.node.subscriptions]
        print(f"已注册 {len(subs)} 个订阅:")
        for t in sorted(subs):
            print(f"  {t}")
        print(f"介入通道: px4_control={'通' if wd.pub_ctrl else '缺'} "
              f"vehicle_command={'通' if wd.pub_vc else '缺'}")
        print(f"介入授权: LAND=开 KILL={'开' if args.enable_kill else '关'}"
              f"{' DRY-RUN' if args.dry_run else ''}")
        print(f"日志文件: {wd.log_file}")
        rclpy.try_shutdown()
        return 0
    try:
        wd.spin()
    finally:
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
