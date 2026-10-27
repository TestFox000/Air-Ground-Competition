# uavlink.sampler_src —— 由 tools/uav_link.py 拆出（机载采样器源码串与启动命令）。与原文件逐行一致（AST 校验过）。
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import socket
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path
from uavlink.config import DZROS_ENV, ORIN_WS_SETUPS, ROS_SETUP, SAMPLER_REMOTE

# ---------------------------------------------------------------- 机载采样器（内嵌源码，SFTP 上传）

SAMPLER_SRC = r'''#!/usr/bin/env python3
"""uav_link 机载采样器（由 tools/uav_link.py 自动上传到 /tmp/uav_sampler.py，勿手改）。

三种用法：
  默认        枚举话题 → 按候选清单订阅 → stdout 流式打 NDJSON（每行一帧）。
  --do ACT    一次性命令（arm/disarm/takeoff/land/return/kill）：按内置映射表
              向发现的 vehicle_command 话题连发 10 帧后退出。
  --print-caps 只枚举打印发现的话题与命令通道，然后退出（笔记本端排障用）。

只依赖 rclpy + 标准库；无 rclpy 时输出 {"error":...} 并非零退出。
话题名无法离线核实，全部运行期自描述发现（get_topic_names_and_types）。
"""
import argparse
import json
import math
import sys
import time

# 与笔记本端 tools/uav_link.py 同源（改了一边记得同步另一边）
TOPIC_CANDIDATES = (
    ("attitude_target", ("vehicle_attitude_setpoint", "attitude_target")),
    ("attitude", ("vehicle_attitude",)),
    ("rates", ("vehicle_angular_velocity",)),
    ("position", ("vehicle_local_position",)),
    ("battery", ("battery_status",)),
    ("status", ("vehicle_status",)),
    ("estimator", ("estimator_status",)),
    ("imu", ("sensor_combined",)),
)
RATE_CAPS_HZ = {"attitude": 10.0, "position": 5.0}
RATE_DEFAULT_HZ = 1.0
COMMAND_MAP = {
    "arm":     {"cmd": 400, "params": {"param1": 1.0}, "ctrl": 2},
    "disarm":  {"cmd": 400, "params": {"param1": 0.0}, "ctrl": 3},
    "takeoff": {"cmd": 22, "params": {"param7": 2.0}, "ctrl": 2},
    "land":    {"cmd": 21, "params": {}, "ctrl": 4},
    "return":  {"cmd": 20, "params": {}, "ctrl": None},
    "kill":    {"cmd": 400, "params": {"param1": 0.0, "param2": 21196.0},
                "ctrl": None},
}
COMMAND_TOPIC_HINT = "vehicle_command"
# px4_control 状态机防护的安全通道（M1）：发现且动作有 ctrl 码时优先走它
# （空中拒 DISARM、地面才 ARM），找不到才直发 vehicle_command；
# kill/return 无对应码（ctrl=None），恒直发。
SAFE_CHANNEL_HINT = "px4_control/command"
TMUX_SESSION = "agup"


def emit(obj):
    print(json.dumps(obj), flush=True)


def quat_to_euler_deg(w, x, y, z):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def choose_qos(reliabilities):
    # 有 best_effort 发布就必须同档订阅（track_diag 真车踩过的坑）；
    # 空列表 = DDS 发现未跑完的竞态——订阅端 best_effort 永远兼容，reliable 会静默
    for r in reliabilities:
        if str(r).lower() == "best_effort":
            return "best_effort"
    return "best_effort" if not reliabilities else "reliable"


def pick_qos(node, topic):
    """按发布端实际 QoS 选订阅档（get_publishers_info_by_topic 自描述）。"""
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rels = []
    try:
        for info in node.get_publishers_info_by_topic(topic):
            r = info.qos_profile.reliability
            # ReliabilityPolicy: 0=SYSTEM_DEFAULT 1=RELIABLE 2=BEST_EFFORT
            rels.append("best_effort" if int(r) == 2 else "reliable")
    except Exception:
        pass
    if choose_qos(rels) == "best_effort":
        return QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    return QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)


def classify_topic(topic):
    name = topic.rsplit("/", 1)[-1] if "/" in topic else topic
    for channel, cands in TOPIC_CANDIDATES:
        if any(c in name or c in topic for c in cands):
            return channel
    return None


def _g(obj, name, default=None):
    """宽松取属性：没有就 default（话题字段无法离线核实，全靠运行期运气）。"""
    try:
        return getattr(obj, name, default)
    except Exception:
        return default


def slim(channel, msg):
    """按通道抽取精简字段（只取确实存在的属性，宁缺毋错）。"""
    if channel == "status":
        arming = _g(msg, "arming_state")
        return {"armed": bool(arming == 2) if arming is not None else None,
                "arming_state": arming,
                "nav_state": _g(msg, "nav_state"),
                "failsafe": bool(_g(msg, "failsafe", False))}
    if channel == "attitude":
        q = _g(msg, "q")
        if q is not None and len(q) >= 4:
            return {"rpy_deg": quat_to_euler_deg(q[0], q[1], q[2], q[3])}
        return {}
    if channel == "attitude_target":
        # vehicle_attitude_setpoint: roll_body/pitch_body/yaw_body (rad)
        r, p, y = _g(msg, "roll_body"), _g(msg, "pitch_body"), _g(msg, "yaw_body")
        if r is not None and p is not None and y is not None:
            return {"rpy_deg": [math.degrees(r), math.degrees(p), math.degrees(y)]}
        qd = _g(msg, "q_d")
        if qd is not None and len(qd) >= 4:
            return {"rpy_deg": quat_to_euler_deg(qd[0], qd[1], qd[2], qd[3])}
        return {}
    if channel == "rates":
        xyz = _g(msg, "xyz")
        if xyz is not None and len(xyz) >= 3:
            return {"dps": [math.degrees(v) for v in xyz[:3]]}
        return {}
    if channel == "position":
        out = {}
        if _g(msg, "x") is not None:
            out["xyz"] = [_g(msg, "x"), _g(msg, "y"), _g(msg, "z")]
        if _g(msg, "vx") is not None:
            out["vel"] = [_g(msg, "vx"), _g(msg, "vy"), _g(msg, "vz")]
        return out
    if channel == "battery":
        return {"voltage": _g(msg, "voltage_v", _g(msg, "voltage")),
                "remaining": _g(msg, "remaining")}
    if channel == "estimator":
        # estimator_status*/estimator_status_flags 版本差异大：把 *_flags 数值全收
        # 留痕。⚠️ S2 审查修正：ok 只由 filter_fault_flags / innovation_check_flags
        # 决定（非零=坏）；solution_status_flags 是"哪些状态输出有效"的位掩码
        # （健康飞行恒非零），绝不参与零判定——旧逻辑"全部为零=健康"恰好搞反，
        # 真机上健康飞行会全程误报 EKF_DEGRADED。两类坏 flag 都不存在时 ok=None
        # （信息不足，笔记本端自然不评 EKF 规则）。与笔记本端
        # estimator_flags_ok 同源（改一边同步另一边）。
        flags = {}
        for name in ("filter_fault_flags", "innovation_check_flags",
                     "solution_status_flags", "control_mode_flags", "fault_flags"):
            v = _g(msg, name)
            if isinstance(v, (int, float)):
                flags[name] = int(v)
        out = {"flags": flags}
        bad = [flags[n] for n in ("filter_fault_flags", "innovation_check_flags")
               if n in flags]
        out["ok"] = all(v == 0 for v in bad) if bad else None
        return out
    if channel == "imu":
        out = {}
        clip = 0
        found = False
        for name in ("accelerometer_clipping", "accel_clipping", "clipping"):
            v = _g(msg, name)
            if v is None:
                continue
            try:
                clip += sum(int(x) for x in v)
                found = True
            except TypeError:
                clip += int(v)
                found = True
        if found:
            out["clip"] = clip
        a = _g(msg, "accelerometer_m_s2")
        if a is not None and len(a) >= 3:
            out["accel"] = [float(a[0]), float(a[1]), float(a[2])]
        return out
    return {}


def discover(node):
    """自描述枚举：返回 (全部话题名列表, {channel: (topic, type_str)})。"""
    names_types = node.get_topic_names_and_types()
    found = sorted(n for n, _ in names_types)
    matched = {}
    for name, types in names_types:
        ch = classify_topic(name)
        if ch and ch not in matched and types:
            matched[ch] = (name, types[0])
    return found, matched


def find_command_channel(found_topics):
    for t in found_topics:
        if COMMAND_TOPIC_HINT in t:
            return t
    return None


def choose_command_route(action, found_topics):
    # 命令路径分路（M1，与笔记本端 uav_link.choose_command_route 同源，
    # 改一边同步另一边）：有 px4_control 安全通道且动作有对应码 → 优先走
    # 状态机防护；否则直发 vehicle_command；都没有 → None（响亮失败）
    if COMMAND_MAP[action].get("ctrl") is not None:
        for t in found_topics:
            if SAFE_CHANNEL_HINT in t:
                return ("px4_control", t)
    ch = find_command_channel(found_topics)
    return ("fmu_direct", ch) if ch is not None else None


def read_dzuav_health():
    """dzuav 健康通道：先 orin_log_capture 的 jsonl，再 tmux capture-pane；没有 None。"""
    import glob
    import os
    import re
    # ① orin_log_capture 输出目录的最新 jsonl 尾行
    try:
        pats = ("/tmp/ag_logs/*/dzuav-*.jsonl",
                os.path.expanduser("~/ag_logs/*/dzuav-*.jsonl"))
        files = []
        for p in pats:
            files.extend(glob.glob(p))
        if files:
            newest = max(files, key=os.path.getmtime)
            # M3：只读文件尾 64KB 找最后一条 JSON 行——旧版每秒整读 jsonl，
            # 文件涨到几百 MB 后每秒全量扫描纯属浪费
            size = os.path.getsize(newest)
            with open(newest, "rb") as fh:
                fh.seek(max(0, size - 65536))
                tail = fh.read().decode("utf-8", "replace")
            last = None
            for line in tail.splitlines():
                line = line.strip()
                if line.startswith("{"):
                    last = line
            if last:
                obj = json.loads(last)
                return {"src": "jsonl", "file": newest,
                        "localized": obj.get("localized"),
                        "landmarks": obj.get("landmarks"),
                        "counts": obj.get("counts")}
    except Exception:
        pass
    # ② tmux capture-pane 抓 dzuav 窗口最近输出里的健康行
    try:
        import subprocess
        out = subprocess.run(
            ["tmux", "capture-pane", "-pt", f"{TMUX_SESSION}:dzuav", "-S", "-40"],
            capture_output=True, text=True, timeout=3).stdout
        health = None
        for line in out.splitlines():
            m = re.search(r"localized=(\d+)\s+landmarks=(\d+)", line)
            if m:
                health = {"src": "tmux", "localized": bool(int(m.group(1))),
                          "landmarks": int(m.group(2)), "raw": line.strip()[:200]}
        return health
    except Exception:
        return None


class RateLimiter:
    def __init__(self, caps, default_hz):
        self.caps = dict(caps)
        self.default_hz = float(default_hz)
        self._last = {}

    def allow(self, key, t, changed=False):
        if changed:
            self._last[key] = t
            return True
        cap = self.caps.get(key, self.default_hz)
        # cap<=0 除零守卫（与笔记本端 RateLimiter 同步）：period=0 = 不限速
        period = 1.0 / cap if cap > 0 else 0.0
        last = self._last.get(key)
        if last is None or t - last >= period - 1e-9:
            self._last[key] = t
            return True
        return False


def make_node():
    """导入 rclpy 并建节点；失败原样报回笔记本（{"error":...} 非零退出）。"""
    try:
        import rclpy
        from rclpy.node import Node
    except Exception as e:
        emit({"error": "rclpy 导入失败: %s: %s" % (type(e).__name__, e)})
        sys.exit(3)
    rclpy.init()
    return rclpy, Node("uav_link_sampler")


def get_msg_class(type_str):
    """按发现的消息类型字符串动态取类（px4_msgs 在 dzpx4 工作空间里）。"""
    try:
        from rosidl_runtime_py.utilities import get_message
        return get_message(type_str)
    except Exception:
        pkg, _, name = type_str.replace("/msg/", "/").split("/")
        mod = __import__(pkg + ".msg", fromlist=[name])
        return getattr(mod, name)


def run_stream(args):
    rclpy, node = make_node()
    found, matched = discover(node)
    emit({"hello": "uav_sampler", "topics_found": found,
          "matched": {ch: t for ch, (t, _ty) in matched.items()}})
    # 等 DDS 发现对端发布端：立刻订阅的话 pick_qos 拿不到 publisher info，
    # 会落到默认档（2026-10-16 实机竞态：话题在、数据零）
    time.sleep(2.0)

    latest = {}          # channel -> slim 后的最新数据
    last_status = [None] # status on-change 对照

    for ch, (topic, type_str) in matched.items():
        try:
            cls = get_msg_class(type_str)
        except Exception as e:
            emit({"error": "消息类型 %s 导入失败（%s），跳过该通道" % (type_str, e)})
            continue
        qos = pick_qos(node, topic)

        def cb(msg, ch=ch):
            latest[ch] = slim(ch, msg)

        node.create_subscription(cls, topic, cb, qos)

    limiter = RateLimiter(RATE_CAPS_HZ, RATE_DEFAULT_HZ)
    t0 = time.time()

    def tick():
        now = time.time()
        for ch, (topic, _ty) in matched.items():
            if ch not in latest:
                continue
            data = latest[ch]
            changed = (ch == "status" and data != last_status[0])
            if limiter.allow(ch, now, changed=changed):
                if ch == "status":
                    last_status[0] = dict(data)
                emit({"t": now, "topic": topic, "ch": ch,
                      "hz": RATE_CAPS_HZ.get(ch, RATE_DEFAULT_HZ), "data": data})
        if not args.no_dzuav and limiter.allow("dzuav", now):
            emit({"t": now, "topic": "dzuav.health", "ch": "dzuav",
                  "hz": 1.0, "data": read_dzuav_health()})

    node.create_timer(0.05, tick)
    emit({"t": time.time(), "topic": "_ready", "ch": "meta", "hz": 0.0,
          "data": {"elapsed_boot": round(time.time() - t0, 2)}})
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_caps(_args):
    _rclpy, node = make_node()
    found, matched = discover(node)
    caps = {
        "topics_found": found,
        "matched": {ch: t for ch, (t, _ty) in matched.items()},
        "command_channel": find_command_channel(found),
        "command_actions": sorted(COMMAND_MAP),
    }
    emit({"hello": "uav_sampler", "topics_found": found})
    emit({"caps": caps})
    node.destroy_node()
    _rclpy.shutdown()


def run_do(args):
    rclpy, node = make_node()
    found, matched = discover(node)
    emit({"hello": "uav_sampler", "topics_found": found,
          "matched": {ch: t for ch, (t, _ty) in matched.items()}})
    route = choose_command_route(args.do, found)
    if route is None:
        # 响亮失败：列出实际发现的话题，绝不盲发
        emit({"ctl": args.do, "ok": False,
              "detail": "未发现命令通道（期望话题名含 %r 或 %r）；实际发现: %s"
                        % (SAFE_CHANNEL_HINT, COMMAND_TOPIC_HINT,
                           ", ".join(found) or "（空）")})
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(4)
    via, channel = route
    spec = COMMAND_MAP[args.do]
    # 找到该话题的类型（命令通道一定在枚举里，否则上面已退出）
    type_str = None
    for name, types in node.get_topic_names_and_types():
        if name == channel and types:
            type_str = types[0]
    try:
        cls = get_msg_class(type_str)
    except Exception as e:
        emit({"ctl": args.do, "ok": False,
              "detail": "命令消息类型 %s 导入失败: %s" % (type_str, e)})
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(5)
    msg = cls()
    if via == "px4_control":
        # 安全通道：px4_control/msg/ControlCommand（uint8 cmd, float32 dx/dy/dz），
        # 状态机守门（ARM 需地面+Offboard+定位有效、DISARM 空中拒绝、LAND 按状态机）
        fields = [("cmd", spec["ctrl"]), ("dx", 0.0), ("dy", 0.0), ("dz", 0.0)]
        detail = ("已向 %s 连发 10 帧 ControlCommand cmd=%d"
                  "（px4_control 状态机防护通道）" % (channel, spec["ctrl"]))
        if args.do == "takeoff":
            detail += "；起飞高度由机载 takeoff_height 参数定（--alt 不生效）"
    else:
        # 直发飞控。该动作有安全通道映射但没找到时：执行前醒目提示（M1）——
        # 绕过 px4_control 状态机防护，空中 DISARM 被拒等保护全部失效。
        if spec["ctrl"] is not None:
            emit({"warn": "未发现 px4_control 安全通道（期望话题名含 %r）——"
                          "ctl %s 将直发飞控 %s，绕过状态机防护，"
                          "确认当前高度/姿态安全"
                          % (SAFE_CHANNEL_HINT, args.do, channel)})
        params = dict(spec["params"])
        if args.do == "takeoff":
            params["param7"] = float(args.alt)
        # 宽松赋值：字段存在才写（px4_msgs 版本差异靠这个扛）
        fields = [("command", spec["cmd"]), ("target_system", 1),
                  ("target_component", 1), ("source_system", 1),
                  ("source_component", 1), ("from_external", True),
                  ("timestamp", int(time.time() * 1e6))] + list(params.items())
        detail = ("已向 %s 连发 10 帧 VehicleCommand cmd=%d params=%s（直发飞控）"
                  % (channel, spec["cmd"], params))
    for name, val in fields:
        if hasattr(msg, name):
            try:
                setattr(msg, name, type(getattr(msg, name))(val)
                        if name != "timestamp" else int(val))
            except (TypeError, ValueError):
                setattr(msg, name, val)
    pub = node.create_publisher(cls, channel, 10)
    time.sleep(0.3)   # 等 DDS 发现对端，首帧不丢
    for _ in range(10):  # 连发 10 帧 @20Hz，抗单帧丢失
        if hasattr(msg, "timestamp"):
            msg.timestamp = int(time.time() * 1e6)
        pub.publish(msg)
        time.sleep(0.05)
    time.sleep(0.2)
    emit({"ctl": args.do, "ok": True, "detail": detail})
    node.destroy_node()
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description="uav_link 机载采样器")
    ap.add_argument("--do", default=None, choices=sorted(COMMAND_MAP),
                    help="一次性命令模式")
    ap.add_argument("--alt", type=float, default=2.0, help="takeoff 目标高度（米）")
    ap.add_argument("--print-caps", action="store_true", help="只打印发现的话题/命令通道")
    ap.add_argument("--no-dzuav", action="store_true", help="不并 dzuav 健康通道")
    args = ap.parse_args()
    if args.print_caps:
        run_caps(args)
    elif args.do:
        run_do(args)
    else:
        run_stream(args)


if __name__ == "__main__":
    main()
'''
def sampler_launch_cmd(extra: str = "") -> str:
    """机载采样器启动命令（bash -lc 非交互 shell，不读 ~/.bashrc）。

    先 source ~/dzrobot/dzros.env（机载 ROS_DOMAIN_ID=118 等域配置，不存在则静默跳过），
    再 source humble 与工作空间（px4_msgs 在 dzpx4 工作空间）；漏掉 dzros.env 会落在
    默认域 0，永远看不到机载 118 域的节点（track_diag 同款教训）。
    ⚠️ 外层 bash -lc 单引号包裹，内层不得再出现单引号。
    """
    ws = " ".join(f"[ -f {s} ] && source {s} || true;" for s in ORIN_WS_SETUPS)
    return (f"bash -lc '[ -f {DZROS_ENV} ] && source {DZROS_ENV} || true; "
            f"source {ROS_SETUP}; {ws} "
            f"setsid python3 -u {SAMPLER_REMOTE}{extra}'")
