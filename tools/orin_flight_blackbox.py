#!/usr/bin/env python3
"""机载飞行黑匣子：把 PX4 飞控 + FAST-LIO + 控制通道全程留痕成 JSONL。

为什么需要它：无人机飞控不稳定，出事后必须能回答"当时到底发生了什么"。
  * 现有 orin_log_capture.py 只录 dzuav 视觉 stdout，飞控数据完全不覆盖；
  * px4_control 的 RCLCPP_INFO 只在 tmux 里滚走，坠机断电即丢。

本脚本在 **Orin 上**运行（Python3 + rclpy，ROS2 自带，无第三方依赖），三件事：
  1. 【看】订阅飞控遥测（位置/姿态/状态/电池）、SLAM 位姿、控制指令，
     全量/限频写入结构化 JSONL（与车端 ag_core.blackbox 同款字段：ts/mono/seq/node/ev/lvl）；
  2. 【录】坠机安全——行级 flush + 关键事件(命令/failsafe/状态翻转)立即 fsync，
     断电最多丢最后 2 秒的普通遥测，命令审计零丢失；
  3. 【传】--udp-mirror 把每条已落盘事件实时镜像给地面台 flight_console.py。

事件一览（node=flight_bb）:
    flight.pos      NED 位置/速度/航向（限频，默认 5Hz）
    flight.att      姿态四元数+欧拉角（限频，默认 2Hz）
    flight.status   解锁/导航模式/failsafe（变更即写，另每秒 1 条保底心跳）
    flight.batt     电池电量/电压（限频，默认 0.2Hz）
    slam.odom       FAST-LIO 原始位姿（限频，默认 5Hz）
    fmu.cmd         发往飞控的 VehicleCommand（命令审计，全量 + fsync）
    fmu.target      px4_control 下发的轨迹目标（限频，默认 5Hz）
    ctrl.cmd        /px4_control/command 审计（teleop 键 + 地面台都在这留痕，全量）
    goto.state      GOTO 开关翻转
    bb.hb           每 3s 健康事件（含各话题实测 Hz）

用法（Orin 上，一般由 flight_console.py 自动拉起，无需手动）:
    python3 orin_flight_blackbox.py --log-dir ~/ag_flight_logs
    python3 orin_flight_blackbox.py --udp-mirror 192.168.50.100:7447
    python3 orin_flight_blackbox.py --list-topics   # 只打印话题订阅情况后退出

字段防御说明：px4_msgs 各版本字段有差异（如 BatteryStatus 电压名），一律
getattr 兜底取值；消息类型缺失时跳过该话题并打警告，绝不因单个话题拖死黑匣子。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import sys
import time
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
except ImportError:  # 无 ROS 的环境（地面笔记本）也要能 --help / 被单测加载
    rclpy = None

    class Node:  # type: ignore[no-redef]  占位基类：main() 会在初始化前响亮拦下
        def __init__(self, *_a, **_kw):
            raise SystemExit(
                "缺少 rclpy：本脚本必须在 Orin 的 ROS2 环境里运行（source /opt/ros/*/setup.bash）")

NODE_NAME = "flight_bb"

# ---- PX4 状态翻译（值来自 PX4 v1.14 vehicle_status.msg；未收录的显示原始数字） ----
ARMING_NAMES = {1: "STANDBY", 2: "ARMED"}
NAV_NAMES = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "MISSION", 4: "LOITER",
    5: "RTL", 7: "TAKEOFF", 8: "LAND", 10: "PRECLAND", 11: "ORBIT",
    14: "OFFBOARD", 17: "LANDENGFAIL", 18: "RTGS",
}
# ControlCommand.msg（px4_control）命令常量
CTRL_NAMES = {
    1: "OFFBOARD", 2: "ARM", 3: "DISARM", 4: "LAND",
    5: "MOVE", 6: "YAW", 7: "GOTO_ON", 8: "GOTO_OFF",
}
# VehicleCommand 常用命令码（px4_control_node 用到 + 应急）
VC_NAMES = {176: "DO_SET_MODE", 21: "NAV_LAND", 400: "ARM_DISARM"}


def _f(v):
    """float 化 + 有限性过滤：NaN/Inf → None（JSON 里不允许 NaN，且 NaN 本身就是坏数据）。"""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _b(v):
    """bool/uint8 转可读 int。"""
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def parse_udp_mirror(text: str) -> tuple[str, int]:
    """解析 --udp-mirror 的 IP:PORT。缺冒号/端口非数字要响亮报错——
    旧写法 rpartition(':') 缺冒号时 int('') 抛 ValueError 裸栈，现场看不懂。"""
    host, sep, port = text.rpartition(":")
    if not sep or not host or not port.isdigit():
        raise SystemExit(f"--udp-mirror 需要 IP:PORT 形式（如 192.168.50.100:7447），"
                         f"收到: {text!r}")
    return host, int(port)


def reclaim_disk(log_root: Path, min_free_mb: float, keep_session: str) -> list[str]:
    """磁盘水位防护：剩余空间低于 min_free_mb 时，从最老会话目录开始删
    （当前会话除外），返回删掉的会话名。会话名是时间戳，名字序即新旧序。"""
    import shutil
    removed: list[str] = []
    if min_free_mb <= 0 or not log_root.is_dir():
        return removed
    while shutil.disk_usage(log_root).free < min_free_mb * 1024 * 1024:
        victims = sorted(p for p in log_root.iterdir()
                         if p.is_dir() and p.name != keep_session)
        if not victims:
            break
        shutil.rmtree(victims[0], ignore_errors=True)
        removed.append(victims[0].name)
    return removed


def quat_to_euler(q0, q1, q2, q3):
    """四元数(w,x,y,z) → (roll, pitch, yaw)，弧度。与 PX4 机体约定一致。"""
    try:
        q0, q1, q2, q3 = float(q0), float(q1), float(q2), float(q3)
    except (TypeError, ValueError):
        return None, None, None
    roll = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
    s = max(-1.0, min(1.0, 2.0 * (q0 * q2 - q3 * q1)))
    pitch = math.asin(s)
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return roll, pitch, yaw


class FlightBlackbox(Node):
    def __init__(self, args):
        super().__init__(NODE_NAME)
        self.args = args
        self.seq = 0
        self.counts: dict[str, int] = {}      # 事件名 -> 条数（健康行用）
        self.start_mono = time.monotonic()

        # ---- 落盘文件 ----
        log_root = Path(args.log_dir).expanduser()
        # 磁盘水位检查：机上存储有限，低于阈值时从最老会话日志开始腾位
        # （当前会话永不删），删了哪些留痕到 bb.disk_reclaim
        removed = reclaim_disk(log_root, args.min_free_mb, args.session)
        log_dir = log_root / args.session
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_file = log_dir / f"flight-{os.getpid()}.jsonl"
        self.fh = open(self.log_file, "a", encoding="utf-8", buffering=1)
        self.last_fsync = time.monotonic()
        if removed:
            self.get_logger().warn(f"磁盘水位不足，已删最老会话日志: {removed}")
            self._emit("bb.disk_reclaim", lvl="warn", removed=removed)

        # ---- UDP 镜像 ----
        self.mirror = None
        if args.udp_mirror:
            self.mirror = parse_udp_mirror(args.udp_mirror)
            self.mirror_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.mirror_sock.setblocking(False)

        # ---- 限频器：per 话题 ----
        self.gates: dict[str, float] = {}

        # ---- 状态变更检测 ----
        self.last_status_sig = None
        self.last_goto = None

        self._register_all()
        self._emit("bb.ready", pid=os.getpid(), log_file=str(self.log_file),
                   udp_mirror=args.udp_mirror or "", argv=" ".join(sys.argv))
        self.get_logger().info(f"飞行黑匣子启动 -> {self.log_file}")

    # ------------------------------------------------------------------ 订阅注册
    def _gate_ok(self, key: str, rate_hz: float) -> bool:
        """限频门：rate_hz<=0 表示不限频；返回 True 表示本条放行。"""
        if rate_hz <= 0:
            return True
        now = time.monotonic()
        if now - self.gates.get(key, 0.0) >= 1.0 / rate_hz:
            self.gates[key] = now
            return True
        return False

    def _sub(self, topic, type_mod, type_name, qos, cb):
        """防御性订阅：类型缺失/注册失败只警告，不拖死黑匣子。"""
        try:
            cls = getattr(type_mod, type_name)
            self.create_subscription(cls, topic, cb, qos)
            return True
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"跳过话题 {topic}: {type(e).__name__}: {e}")
            return False

    def _register_all(self):
        qos_px4 = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_ctrl = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_latch = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        r = self.args
        px4_msgs = nav_msgs = std_msgs = px4_control = None

        # 消息类在 .msg 子模块（2026-10-16 实机踩坑：顶层 import 成功但 getattr 全失败）
        try:
            import px4_msgs.msg as px4_msgs
        except ImportError:
            self.get_logger().warn("缺 px4_msgs：飞控话题全部跳过（source 工作空间了吗？）")
        try:
            import nav_msgs.msg as nav_msgs
        except ImportError:
            pass
        try:
            import std_msgs.msg as std_msgs
        except ImportError:
            pass
        try:
            import px4_control.msg as px4_control
        except ImportError:
            self.get_logger().warn("缺 px4_control.msg：/px4_control/command 审计跳过")

        if px4_msgs:
            self._sub("/fmu/out/vehicle_local_position", px4_msgs, "VehicleLocalPosition",
                      qos_px4, self._on_local_pos)
            self._sub("/fmu/out/vehicle_attitude", px4_msgs, "VehicleAttitude",
                      qos_px4, self._on_attitude)
            self._sub("/fmu/out/vehicle_status", px4_msgs, "VehicleStatus",
                      qos_px4, self._on_status)
            self._sub("/fmu/out/battery_status", px4_msgs, "BatteryStatus",
                      qos_px4, self._on_battery)
            self._sub("/fmu/in/vehicle_command", px4_msgs, "VehicleCommand",
                      qos_px4, self._on_fmu_cmd)
            self._sub("/fmu/in/trajectory_setpoint", px4_msgs, "TrajectorySetpoint",
                      qos_px4, self._on_target)
        if nav_msgs:
            self._sub("/Odometry", nav_msgs, "Odometry", qos_px4, self._on_slam_odom)
        if std_msgs:
            self._sub("/px4_control/goto_enabled", std_msgs, "Bool", qos_latch, self._on_goto)
        if px4_control:
            self._sub("/px4_control/command", px4_control, "ControlCommand",
                      qos_ctrl, self._on_ctrl_cmd)

    # ------------------------------------------------------------------ 各话题回调
    def _on_local_pos(self, m):
        if not self._gate_ok("pos", self.args.rate_pos):
            return
        g = lambda k: getattr(m, k, None)  # noqa: E731
        xy_valid, z_valid = _b(g("xy_valid")), _b(g("z_valid"))
        self._emit("flight.pos", x=_f(g("x")), y=_f(g("y")), z=_f(g("z")),
                   vx=_f(g("vx")), vy=_f(g("vy")), vz=_f(g("vz")),
                   yaw=_f(g("heading")), xy_valid=xy_valid, z_valid=z_valid)

    def _on_attitude(self, m):
        if not self._gate_ok("att", self.args.rate_att):
            return
        q = getattr(m, "q", None)
        if not q or len(q) < 4:
            return
        roll, pitch, yaw = quat_to_euler(q[0], q[1], q[2], q[3])
        self._emit("flight.att", q=[_f(v) for v in q[:4]],
                   roll=_f(roll), pitch=_f(pitch), yaw=_f(yaw))

    def _on_status(self, m):
        """状态变化必须零延迟落盘（含 fsync），同时每秒 1 条保底心跳。"""
        g = lambda k: getattr(m, k, None)  # noqa: E731
        arming = _b(g("arming_state"))
        nav = _b(g("nav_state"))
        failsafe = _b(g("failsafe"))
        sig = (arming, nav, failsafe)
        changed = sig != self.last_status_sig
        if not changed and not self._gate_ok("status", 1.0):
            return
        first = self.last_status_sig is None
        prev = self.last_status_sig
        self.last_status_sig = sig
        lvl = "warn" if changed and not first else "info"
        self._emit("flight.status", lvl=lvl, force=True,
                   arming=arming, arming_name=ARMING_NAMES.get(arming, f"s{arming}"),
                   nav=nav, nav_name=NAV_NAMES.get(nav, f"nav{nav}"),
                   failsafe=failsafe,
                   prev=None if first else {
                       "arming": prev[0], "nav": prev[1], "failsafe": prev[2]})
        if changed and not first:
            self.get_logger().warn(f"状态翻转: {prev} -> {sig}")

    def _on_battery(self, m):
        if not self._gate_ok("batt", self.args.rate_batt):
            return
        g = lambda k: getattr(m, k, None)  # noqa: E731
        # 各版本字段名不同：新版本 voltage_v，老版本 voltage_a
        volt = _f(g("voltage_v"))
        if volt is None:
            volt = _f(g("voltage_a"))
        rem = _f(g("remaining"))
        lvl = "error" if rem is not None and rem < 0.15 else "info"
        self._emit("flight.batt", lvl=lvl, remaining=rem, voltage=volt,
                   warning=_b(g("warning")))

    def _on_slam_odom(self, m):
        if not self._gate_ok("slam", self.args.rate_slam):
            return
        p = getattr(getattr(m, "pose", None), "pose", None)
        if p is None:
            return
        self._emit("slam.odom",
                   x=_f(p.position.x), y=_f(p.position.y), z=_f(p.position.z),
                   qw=_f(p.orientation.w), qx=_f(p.orientation.x),
                   qy=_f(p.orientation.y), qz=_f(p.orientation.z))

    def _on_fmu_cmd(self, m):
        """发往飞控的底层命令——命令审计，全量+立即 fsync。"""
        g = lambda k: getattr(m, k, None)  # noqa: E731
        cmd = _b(g("command"))
        lvl = "warn"
        if cmd == 400 and _f(g("param1")) == 0.0 and _f(g("param2")) == 21196.0:
            lvl = "error"  # KILL（电机立即停转）
        self._emit("fmu.cmd", lvl=lvl, force=True, command=cmd,
                   name=VC_NAMES.get(cmd, f"vc{cmd}"),
                   p1=_f(g("param1")), p2=_f(g("param2")), p3=_f(g("param3")),
                   p4=_f(g("param4")), p7=_f(g("param7")))

    def _on_target(self, m):
        if not self._gate_ok("target", self.args.rate_target):
            return
        pos = getattr(m, "position", None)
        vel = getattr(m, "velocity", None)
        yaw = _f(getattr(m, "yaw", None))
        rec = {"target_x": None, "target_y": None, "target_z": None, "yaw": yaw}
        if pos and len(pos) >= 3:
            rec.update(target_x=_f(pos[0]), target_y=_f(pos[1]), target_z=_f(pos[2]))
        if vel and len(vel) >= 3:
            rec.update(tvz=_f(vel[2]))  # 着陆自管下降速度（其余多为 NaN）
        self._emit("fmu.target", **rec)

    def _on_goto(self, m):
        v = _b(getattr(m, "data", None))
        if v is None or v == self.last_goto:
            return
        self.last_goto = v
        self._emit("goto.state", lvl="warn", force=True, enabled=v)

    def _on_ctrl_cmd(self, m):
        """teleop 键 / 地面台指令统一审计——不管 px4_control 拒没拒绝，这里都留痕。"""
        g = lambda k: getattr(m, k, None)  # noqa: E731
        cmd = _b(g("cmd"))
        self._emit("ctrl.cmd", lvl="warn", force=True, cmd=cmd,
                   name=CTRL_NAMES.get(cmd, f"cmd{cmd}"),
                   dx=_f(g("dx")), dy=_f(g("dy")), dz=_f(g("dz")))

    # ------------------------------------------------------------------ 落盘+镜像
    def _emit(self, ev: str, lvl: str = "info", force: bool = False, **fields):
        self.seq += 1
        rec = {
            "ts": round(time.time(), 3),
            "mono": round(time.monotonic(), 3),
            "seq": self.seq,
            "node": NODE_NAME,
            "ev": ev,
            "lvl": lvl,
            "session": self.args.session,
        }
        rec.update(fields)
        self.counts[ev] = self.counts.get(ev, 0) + 1
        try:
            line = json.dumps(rec, ensure_ascii=False, separators=(",", ":"),
                              allow_nan=False)
            self.fh.write(line + "\n")
            self.fh.flush()
            now = time.monotonic()
            if force or now - self.last_fsync >= 2.0:
                os.fsync(self.fh.fileno())
                self.last_fsync = now
        except (OSError, ValueError) as e:  # noqa: BLE001  写盘/编码失败绝不拖死节点
            self.get_logger().error(f"落盘失败({ev}): {e}")
            return
        if self.mirror:
            try:
                self.mirror_sock.sendto((line + "\n").encode("utf-8", "replace"),
                                        self.mirror)
            except (OSError, BlockingIOError):  # noqa: BLE001
                pass  # 镜像丢了不影响落盘

    # ------------------------------------------------------------------ 健康行
    def health_line(self) -> str:
        el = time.monotonic() - self.start_mono
        hz = lambda k: round(self.counts.get(k, 0) / el, 1) if el > 0 else 0.0  # noqa: E731
        sig = self.last_status_sig or (None, None, None)
        return (f"[{NODE_NAME}] up={el:.0f}s pos={hz('flight.pos')}Hz att={hz('flight.att')}Hz "
                f"slam={hz('slam.odom')}Hz cmd={self.counts.get('fmu.cmd', 0)} "
                f"armed={(sig[0] or 0) == 2} nav={NAV_NAMES.get(sig[1], sig[1])} "
                f"failsafe={bool(sig[2])} ev={self.seq} -> {self.log_file}")

    def run(self):
        stopping = {"flag": False}

        def on_sig(signum, frame):  # noqa: ARG001
            stopping["flag"] = True

        signal.signal(signal.SIGINT, on_sig)
        signal.signal(signal.SIGTERM, on_sig)

        last_hb = 0.0
        while rclpy_ok() and not stopping["flag"]:
            rclpy.spin_once(self, timeout_sec=0.2)
            now = time.monotonic()
            if now - last_hb >= 3.0:
                last_hb = now
                print(self.health_line(), flush=True)
                self._emit("bb.hb", counts=dict(self.counts),
                           up=round(now - self.start_mono, 1))
        self.close()

    def close(self):
        try:
            self._emit("bb.exit", force=True, counts=dict(self.counts))
            os.fsync(self.fh.fileno())
            self.fh.close()
        except Exception:  # noqa: BLE001
            pass
        print(f"[{NODE_NAME}] 已停止，日志: {self.log_file}", flush=True)


def rclpy_ok() -> bool:
    try:
        return rclpy.ok()
    except Exception:  # noqa: BLE001
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description="机载飞行黑匣子（PX4+SLAM+控制通道 -> JSONL）")
    ap.add_argument("--log-dir", type=Path, default=Path("~/ag_flight_logs"),
                    help="日志根目录（默认 ~/ag_flight_logs）")
    ap.add_argument("--session", default=time.strftime("%Y%m%d_%H%M%S"),
                    help="会话名（默认启动时间戳）")
    ap.add_argument("--udp-mirror", default=None, metavar="IP:PORT",
                    help="把每条已落盘事件镜像到地面台（flight_console.py 的监听端口）")
    ap.add_argument("--rate-pos", type=float, default=5.0, help="位置记录频率 Hz（默认 5）")
    ap.add_argument("--rate-att", type=float, default=2.0, help="姿态记录频率 Hz（默认 2）")
    ap.add_argument("--rate-slam", type=float, default=5.0, help="SLAM 里程计频率 Hz（默认 5）")
    ap.add_argument("--rate-target", type=float, default=5.0, help="控制目标频率 Hz（默认 5）")
    ap.add_argument("--rate-batt", type=float, default=0.2, help="电池记录频率 Hz（默认 0.2）")
    ap.add_argument("--min-free-mb", type=float, default=500.0,
                    help="磁盘剩余低于此值时删最老会话日志腾位（默认 500MB，0=关闭）")
    ap.add_argument("--list-topics", action="store_true",
                    help="只打印订阅注册情况后退出（部署自检用）")
    args = ap.parse_args()

    if rclpy is None:
        raise SystemExit(
            "缺少 rclpy：本脚本必须在 Orin 的 ROS2 环境里运行（source /opt/ros/*/setup.bash）")
    if args.udp_mirror:
        parse_udp_mirror(args.udp_mirror)   # 提前响亮报错，别等节点起来才炸

    rclpy.init()
    node = FlightBlackbox(args)
    if args.list_topics:
        names = sorted(getattr(s, "topic_name", str(s)) for s in node.subscriptions)
        print(f"已注册 {len(names)} 个订阅:")
        for t in names:
            print(f"  {t}")
        print(f"日志文件: {node.log_file}")
        node.close()
        rclpy.try_shutdown()
        return 0
    try:
        node.run()
    finally:
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
