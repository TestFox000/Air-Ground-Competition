# -*- coding: utf-8 -*-
"""无人机地面站：看得见（遥测）、控得住（调控）、查得回（黑匣子 + 复盘）。

针对"飞控很不稳定，需要知道无人机正在发生什么"的现场需求：
  run     只读遥测 + 黑匣子：SSH 到机载电脑（Jetson Orin Nano），上传内嵌采样器
          （rclpy 自描述枚举话题），NDJSON 流回笔记本逐行落盘（黑匣子），
          2Hz 终端仪表盘 + 规则引擎实时告警（FAILSAFE/EKF/姿态发散/速率饱和/
          振动削波/低电/dzuav 静默/链路滞后）。Ctrl-C 优雅退出并写收尾统计。
  ctl     调控支配（全部显式闸门，默认什么都不做）：
          arm/takeoff/stack-restart 必须 --yes；disarm/land/return/kill
          免 --yes 但醒目警告 + 3s 倒计时（--yes 跳过）；stack-status 只读。
          找不到预期命令通道时响亮失败（列出实际发现的话题），绝不盲发。
  logs    取回日志：dzuav jsonl、ROS log 近况、tmux 各窗口 capture-pane，
          打包到 logs/uav_link/<时间戳>/ 并列清单。
  report  离线复盘：读一个黑匣子 jsonl，按 严重/注意/信息 分级输出
          （armed 时间线、failsafe 事件、姿态/角速度统计、姿态-目标偏差、
          vibe clip、链路空洞、dzuav 健康摘要）。

现场架构（已核实，勿重查）：
  Orin 上官方栈 dzpx4：Livox MID360 → FASTLIO2 → PX4（MicroXRCEAgent DDS 桥，
  **不是裸 MAVLink**），另有我们的 dzuav 侦察进程；tools/orin_bringup.sh 用
  tmux 编排（会话 agup，窗口 dzuav/livox/fastlio/px4/odom/imu/control/path/teleop）。
  ROS 域号真源 ~/dzrobot/dzros.env（默认 118）——所有机载 ROS 命令必须先 source 它。
  PX4 经 XRCE 的话题通常是 /fmu/out/*，但具体话题名无法离线验证，采样器自描述发现。

用法:
  set AG_ORIN_PASS=******
  python tools/uav_link.py run --host 10.15.166.60            # 遥测 + 黑匣子
  python tools/uav_link.py ctl stack-status                   # 只读看栈状态
  python tools/uav_link.py ctl arm --yes                      # 解锁（高风险）
  python tools/uav_link.py ctl land                           # 3s 倒计时后降落
  python tools/uav_link.py ctl --print-caps                   # 看机载发现的命令通道
  python tools/uav_link.py logs                               # 取回日志打包
  python tools/uav_link.py report logs/uav_link/20261016_120000.jsonl

依赖: paramiko（仅笔记本端）；Orin 侧只用 rclpy + 标准库。
"""
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

# ---------------------------------------------------------------- 现场契约（已核实，勿改）

ORIN_USER_DEFAULT = "nvidia"          # 机载默认用户（--orin-user 可改）
DZROS_ENV = "~/dzrobot/dzros.env"     # ROS 域号真源（默认 118），不 source 就跨不到机载域
ROS_SETUP = "/opt/ros/humble/setup.bash"
ORIN_WS_SETUPS = ("~/dzrobot/install/setup.bash", "~/dzpx4/install/setup.bash")
TMUX_SESSION = "agup"                 # orin_bringup.sh 编排的全栈会话
TMUX_WINDOWS = ("dzuav", "livox", "fastlio", "px4", "odom", "imu",
                "control", "path", "teleop")
ORIN_REPO = "~/dzuav_deploy/air_ground"   # Orin 上仓库路径（orin_deploy.py REMOTE_ROOT）

# 通道候选清单：话题名无法离线核实，采样器运行期自描述枚举后按子串匹配。
# ⚠️ 顺序敏感：attitude_target 必须在 attitude 前（setpoint 名字包含 attitude）。
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

# 每话题限速：attitude 10Hz、position 5Hz、status/其他 1Hz（status 另有 on-change 直通）
RATE_CAPS_HZ = {"attitude": 10.0, "position": 5.0}
RATE_DEFAULT_HZ = 1.0

# 命令映射表（数据驱动）：action → MAVLink VehicleCommand。
# cmd 编号：400=COMPONENT_ARM_DISARM 22=NAV_TAKEOFF 21=NAV_LAND
#          20=NAV_RETURN_TO_LAUNCH 185=DO_FLIGHTTERMINATION（电机立即停转）
COMMAND_MAP = {
    "arm":     {"cmd": 400, "params": {"param1": 1.0}, "desc": "解锁（电机可转）"},
    "disarm":  {"cmd": 400, "params": {"param1": 0.0}, "desc": "上锁"},
    "takeoff": {"cmd": 22, "params": {"param7": 2.0},
                "desc": "起飞到 --alt 米（param7，默认 2.0）"},
    "land":    {"cmd": 21, "params": {}, "desc": "原地降落"},
    "return":  {"cmd": 20, "params": {}, "desc": "返航（Return-to-Launch）"},
    "kill":    {"cmd": 185, "params": {"param1": 1.0},
                "desc": "急停（flight termination，电机立即停转，会炸机！）"},
}
# 栈生命周期动作（tmux / orin_bringup.sh，不走 ROS）
STACK_ACTIONS = ("stack-status", "stack-restart")
ALL_ACTIONS = tuple(COMMAND_MAP) + STACK_ACTIONS

COMMAND_TOPIC_HINT = "vehicle_command"   # 期望 /fmu/in/vehicle_command

# 安全闸门：高风险动作必须 --yes；次高风险免 --yes 但倒计时 3s
YES_REQUIRED = {"arm", "takeoff", "stack-restart"}
COUNTDOWN_ACTIONS = {"disarm", "land", "return", "kill"}
COUNTDOWN_S = 3

SAMPLER_REMOTE = "/tmp/uav_sampler.py"
HOST_CACHE = Path("logs") / "uav_link" / "uav_link.local.env"   # 上次 --host 缓存


# ---------------------------------------------------------------- 纯函数：四元数 / QoS / 限速 / 话题分类

def quat_to_euler_deg(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """四元数 (w,x,y,z) → (roll, pitch, yaw) 度，ZYX 内旋顺序（航空惯例）。

    PX4 vehicle_attitude 的 q 字段即 [w,x,y,z]。pitch 接近 ±90° 时万向锁，
    asin 输入截断到 ±1 防数值越界。
    """
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def choose_qos(reliabilities) -> str:
    """按发布端 QoS 选订阅档：有 best_effort 就 best_effort，否则 reliable。

    ⚠️ track_diag 真车踩过的坑：DDS 里 reliable 订阅收不到 best_effort 发布，
    反向可以；PX4 经 XRCE 的 /fmu/out/* 大多是 best_effort，盲订 reliable 会
    永远收不到。reliabilities 为标签串列表（"best_effort"/"reliable"/...），
    空列表按 reliable（ROS 发布端默认即 reliable）。
    """
    for r in reliabilities:
        if str(r).lower() == "best_effort":
            return "best_effort"
    return "reliable"


class RateLimiter:
    """每话题限速器（per-topic cap）。changed=True 直通（状态类话题 on-change）。"""

    def __init__(self, caps: dict | None = None, default_hz: float = RATE_DEFAULT_HZ):
        self.caps = dict(caps or RATE_CAPS_HZ)
        self.default_hz = float(default_hz)
        self._last: dict[str, float] = {}

    def allow(self, key: str, t: float, changed: bool = False) -> bool:
        if changed:
            self._last[key] = t
            return True
        cap = self.caps.get(key, self.default_hz)
        period = 1.0 / cap if cap > 0 else 0.0
        last = self._last.get(key)
        if last is None or t - last >= period - 1e-9:
            self._last[key] = t
            return True
        return False


def classify_topic(topic: str) -> str | None:
    """话题名 → 通道名（attitude/rates/position/battery/status/estimator/imu/...）。

    按 TOPIC_CANDIDATES 顺序首个子串命中即返回；都不中返回 None。
    """
    name = topic.rsplit("/", 1)[-1] if "/" in topic else topic
    for channel, cands in TOPIC_CANDIDATES:
        if any(c in name or c in topic for c in cands):
            return channel
    return None


def find_command_channel(found_topics) -> str | None:
    """从采样器发现的话题清单里找命令通道（含 vehicle_command 者）；找不到 None。"""
    for t in found_topics:
        if COMMAND_TOPIC_HINT in str(t):
            return str(t)
    return None


def render_channel_failure(action: str, found_topics) -> str:
    """找不到命令通道时的响亮失败文本（列出实际发现的话题帮助用户改映射）。"""
    lines = [
        f"[失败] ctl {action}：机载没有发现命令通道（期望话题名含 "
        f"{COMMAND_TOPIC_HINT!r}），**拒绝盲发**。",
        "采样器实际发现的话题：",
    ]
    lines += [f"  - {t}" for t in found_topics] or ["  （一个话题都没发现——"
                                                  "MicroXRCEAgent 可能没起，先 ctl stack-status 看栈）"]
    lines.append("排查：① MicroXRCEAgent 窗口(px4)是否活着；② dzros.env 域号是否一致；"
                 "③ 话题名变了就改 COMMAND_TOPIC_HINT/采样器候选清单。")
    return "\n".join(lines)


def render_command_map() -> str:
    """命令映射表文本（ctl --print-caps / 帮助排障用，数据驱动渲染 COMMAND_MAP）。"""
    lines = ["命令映射表（数据驱动，改 COMMAND_MAP 即可换命令/参数）："]
    for action, spec in COMMAND_MAP.items():
        params = " ".join(f"{k}={v}" for k, v in spec["params"].items()) or "（无参数）"
        lines.append(f"  {action:<8s} -> VehicleCommand cmd={spec['cmd']:<4d} "
                     f"{params:<14s} {spec['desc']}")
    lines.append(f"  stack-status / stack-restart -> tmux 会话 {TMUX_SESSION} "
                 f"+ {ORIN_REPO}/tools/orin_bringup.sh（不走 ROS）")
    lines.append(f"命令通道期望话题名含 {COMMAND_TOPIC_HINT!r}"
                 "（实际以采样器 --print-caps 发现为准）")
    return "\n".join(lines)


# ---------------------------------------------------------------- 纯函数：NDJSON 采样行解析 / 黑匣子读写

def _float_or_none(v) -> float | None:
    try:
        return float(v) if v is not None else None
    except (TypeError, ValueError):
        return None


def parse_sampler_line(line: str) -> dict | None:
    """解析采样器一行 NDJSON；坏行/非 JSON 返回 None（容错跳过）。

    返回规范形：
      {"kind": "hello",  "topics_found": [...]}
      {"kind": "error",  "error": "..."}                      采样器主动报错
      {"kind": "sample", "t": float|None, "topic": str, "ch": str|None,
       "hz": float|None, "data": dict}
      {"kind": "ctl",    "action": str, "ok": bool, "detail": str}
    """
    line = line.strip()
    if not line or not line.startswith("{"):
        return None
    try:
        obj = json.loads(line)
    except json.JSONDecodeError:
        return None
    if not isinstance(obj, dict):
        return None
    if obj.get("error"):
        return {"kind": "error", "error": str(obj["error"])}
    if "hello" in obj:
        tf = obj.get("topics_found")
        return {"kind": "hello", "hello": str(obj["hello"]),
                "topics_found": [str(x) for x in tf] if isinstance(tf, list) else []}
    if obj.get("src") == "ctl" or obj.get("ctl") is not None:
        return {"kind": "ctl",
                "action": str(obj.get("action") or obj.get("ctl") or ""),
                "ok": bool(obj.get("ok")),
                "detail": str(obj.get("detail", ""))}
    if "topic" in obj:
        data = obj.get("data")
        topic = str(obj["topic"])
        return {"kind": "sample",
                "t": _float_or_none(obj.get("t")),
                "topic": topic,
                "ch": str(obj["ch"]) if obj.get("ch") else classify_topic(topic),
                "hz": _float_or_none(obj.get("hz")),
                "data": data if isinstance(data, dict) else {}}
    return None


class BlackboxWriter:
    """黑匣子写入器：每行 NDJSON，盖本机接收时刻 recv，逐行 flush（断电也尽量少丢）。

    ⚠️ 必须 newline="\\n"：Windows 默认会把 \\n 翻译成 \\r\\n（remote_run.py 踩过的坑）。
    """

    def __init__(self, path, append: bool = False):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, "a" if append else "w",
                        encoding="utf-8", newline="\n")

    def write(self, row: dict, recv: float | None = None) -> None:
        out = {"recv": time.time() if recv is None else float(recv)}
        out.update(row)
        self._fh.write(json.dumps(out, ensure_ascii=False) + "\n")
        self._fh.flush()

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:  # noqa: BLE001 - 收尾阶段异常不掩盖主流程
            pass


def read_blackbox(path) -> list[dict]:
    """读黑匣子 jsonl → 原始行列表；坏行容错跳过（文件可能被 Ctrl-C 截尾）。"""
    rows: list[dict] = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or not line.startswith("{"):
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(obj, dict):
                rows.append(obj)
    return rows


# ---------------------------------------------------------------- 纯函数：安全闸门 / 倒计时

def check_gate(action: str, yes: bool) -> str | None:
    """安全闸门：返回 None=放行；否则返回拒绝原因（调用方打印后退出）。

    arm/takeoff/stack-restart 属高风险，必须显式 --yes——默认什么都不做。
    """
    if action in YES_REQUIRED and not yes:
        return (f"动作 {action} 属高风险，必须显式加 --yes 才会执行"
                f"（默认什么都不做）。确认无人机周围安全后重试。")
    return None


def countdown(action: str, seconds: int = COUNTDOWN_S, yes: bool = False,
              out=print, sleep=time.sleep) -> None:
    """次高风险动作执行前的醒目警告 + 逐秒倒计时；--yes 跳过（out/sleep 可注入便于测试）。"""
    if yes or action not in COUNTDOWN_ACTIONS:
        return
    out("!" * 60)
    out(f"!! 即将执行 {action}（{COMMAND_MAP[action]['desc']}）")
    out(f"!! {seconds} 秒内 Ctrl-C 可中止")
    out("!" * 60)
    for i in range(seconds, 0, -1):
        out(f"   {i} …")
        sleep(1.0)


# ---------------------------------------------------------------- 主机解析 / 密码 / TCP 探测

def load_cached_host(path=HOST_CACHE) -> str | None:
    """从缓存文件读上次 --host（uav_link.local.env，形如 UAV_HOST=...）。"""
    try:
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line.startswith("UAV_HOST="):
                v = line.split("=", 1)[1].strip()
                return v or None
    except OSError:
        pass
    return None


def save_cached_host(host: str, path=HOST_CACHE) -> None:
    """记住本次 --host，下次免输（纯便利缓存，不是秘密）。"""
    try:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(f"UAV_HOST={host}\n", encoding="utf-8", newline="\n")
    except OSError:
        pass


def resolve_host(args) -> str | None:
    """--host 优先，其次上次缓存；都没有返回 None（调用方给清晰报错）。"""
    host = getattr(args, "host", None)
    if host:
        return host
    return load_cached_host(getattr(args, "host_cache", HOST_CACHE))


def read_password(args) -> str:
    """密码优先级：--password > AG_ORIN_PASS 环境变量 > 交互提问。

    ⚠️ 与 tools/remote_run.py 同一坑：没有交互终端时 getpass 会**静默死等**
    （看着像"卡死"），这里直接报错并说明该给什么。
    """
    pw = getattr(args, "password", None) or os.environ.get("AG_ORIN_PASS")
    if pw:
        return pw
    import getpass
    if not sys.stdin.isatty():
        raise SystemExit(
            "缺少机载端密码（当前没有交互终端，无法弹出密码提问）：\n"
            "  ① 设环境变量：set AG_ORIN_PASS=<密码>\n"
            "  ② 或命令行传入：--password <密码>"
        )
    return getpass.getpass(f"{args.orin_user}@{args.host} 的密码: ")


def probe_ssh(host: str, port: int = 22, timeout: float = 3.0) -> str | None:
    """连接前 TCP 探测：端口不通返回清晰报错文本，通则 None。

    省得在 paramiko 的长超时里干等，还把"无人机没开机/不在网段"误判成 SSH 故障。
    """
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return None
    except OSError as e:
        return (f"SSH 端口不可达 {host}:{port}（{type(e).__name__}: {e}）——"
                f"查无人机是否开机、笔记本是否同网段、IP 是否变了")


# ---------------------------------------------------------------- 告警规则引擎（纯逻辑、可测）

class UavRuleEngine:
    """无人机遥测告警规则状态机（照 track_diag RuleEngine 范式：冷却 + 持续判定）。

    update(t, st)：st 为遥测快照 dict（键都可缺省，缺了对应规则自然不评）：
      failsafe: bool            vehicle_status.failsafe
      estimator_ok: bool        estimator 类 flag 全零=True
      att_deg: (r,p,y)          姿态角（度）
      att_target_deg: (r,p,y)   姿态目标（度，有才评 ATT_DIVERGE）
      rate_dps: (x,y,z)         角速度（度/秒）
      clip_count: int           sensor_combined 加速度计削波累计计数
      battery_remaining: float  0..1
      dzuav_t: float|None       最近一次 dzuav 健康行时刻（本机钟）
    note_sample(t, topic, hz)：每条采样行到达时调用（LINK_LAG 用）。
    命中返回 [{"rule","t","msg"}]。
    """

    COOLDOWN_S = 5.0        # 每条规则命中后的冷却
    EKF_SUSTAIN_S = 2.0     # EKF_DEGRADED 需持续时长
    ATT_DIVERGE_DEG = 15.0  # ATT_DIVERGE 偏差阈值
    ATT_SUSTAIN_S = 1.0
    RATE_SAT_DPS = 300.0    # RATE_SAT 角速度限幅阈值（侦察机持续 300°/s 必异常）
    RATE_SUSTAIN_S = 1.0
    BAT_LOW_FRAC = 0.20     # BAT_LOW 剩余电量阈值
    DZUAV_SILENT_S = 5.0    # DZUAV_SILENT 静默阈值
    LINK_LAG_RATIO = 2.0    # LINK_LAG：采样间隔 p95 > 期望 × 此倍数
    LINK_WINDOW_S = 5.0     # LINK_LAG 观察窗
    LINK_MIN_SAMPLES = 10   # LINK_LAG 至少这么多间隔样本才评
    LINK_FAST_HZ = 5.0      # 只有声明 hz >= 此值的快通道才评 LINK_LAG

    MESSAGES = {
        "FAILSAFE": "飞控 FAILSAFE 触发——立即接管/降落，查触发原因",
        "EKF_DEGRADED": "EKF 估计器 flag 异常——定位/姿态估计退化，查估计器状态",
        "ATT_DIVERGE": "姿态与目标偏差过大——跟踪回路异常，查角速率/振动/调参",
        "RATE_SAT": "角速度接近限幅——可能在甩，查振动/调参/机械",
        "VIBE_CLIP": "加速度计削波计数上升——振动过大，查桨/机架松动/减震",
        "BAT_LOW": "电池剩余 <20%——尽快降落",
        "DZUAV_SILENT": "dzuav 健康 5s 无更新——侦察进程可能挂了",
        "LINK_LAG": "遥测链路滞后——采样间隔 p95 超期望 2 倍，查 Wi-Fi/串口桥",
    }

    def __init__(self) -> None:
        self.expect_dzuav = True
        self.start_t: float | None = None
        self.cond_since: dict[str, float] = {}
        self.last_fired: dict[str, float] = {}
        self.counts: Counter = Counter()
        self._last_clip: int | None = None
        self._link: dict[str, deque] = {}   # topic -> deque[(t, interval, expected)]

    # ---- 内部工具（与 track_diag RuleEngine 同款）----
    def _cooled(self, rule: str, t: float) -> bool:
        return t - self.last_fired.get(rule, -1e18) >= self.COOLDOWN_S

    def _fire(self, rule: str, t: float, extra: str = "") -> dict:
        self.last_fired[rule] = t
        self.counts[rule] += 1
        return {"rule": rule, "t": t, "msg": self.MESSAGES[rule] + extra}

    def _sustained(self, rule: str, t: float, cond: bool, sustain: float) -> bool:
        """条件持续 sustain 秒且过冷却 → 应触发（触发后重新计时）。"""
        if not cond:
            self.cond_since.pop(rule, None)
            return False
        since = self.cond_since.setdefault(rule, t)
        if t - since >= sustain and self._cooled(rule, t):
            self.cond_since[rule] = t
            return True
        return False

    # ---- 链路采样间隔跟踪 ----
    def note_sample(self, t: float, topic: str, hz: float | None) -> None:
        """记录快通道采样间隔（LINK_LAG 原料）。hz 为采样器声明的限速档位。"""
        if not hz or hz < self.LINK_FAST_HZ:
            return
        dq = self._link.setdefault(topic, deque())
        if dq:
            interval = t - dq[-1][0]
            if interval > 0:
                dq.append((t, interval, 1.0 / hz))
        else:
            dq.append((t, 0.0, 1.0 / hz))   # 首样本只占位（interval=0 不参与统计）
        while dq and t - dq[0][0] > self.LINK_WINDOW_S:
            dq.popleft()

    def _link_lag_ratio(self, t: float) -> tuple[str, float] | None:
        """窗口内 p95 间隔 / 期望周期 的最大比值（>1 即滞后）；样本不足 None。"""
        worst = None
        for topic, dq in self._link.items():
            pts = [p for p in dq if p[1] > 0 and t - p[0] <= self.LINK_WINDOW_S + 1e-9]
            if len(pts) < self.LINK_MIN_SAMPLES:
                continue
            ints = sorted(p[1] for p in pts)
            p95 = _percentile(ints, 95)
            expected = pts[-1][2]
            if expected > 0:
                ratio = p95 / expected
                if worst is None or ratio > worst[1]:
                    worst = (topic, ratio)
        return worst

    # ---- 主入口 ----
    def update(self, t: float, st: dict) -> list[dict]:
        alerts: list[dict] = []
        if self.start_t is None:
            self.start_t = t

        # FAILSAFE：failsafe=true 立即报（冷却内不重复）
        if st.get("failsafe") and self._cooled("FAILSAFE", t):
            alerts.append(self._fire("FAILSAFE", t))

        # EKF_DEGRADED：estimator flag 异常持续 2s
        ok = st.get("estimator_ok")
        if self._sustained("EKF_DEGRADED", t, ok is False, self.EKF_SUSTAIN_S):
            alerts.append(self._fire("EKF_DEGRADED", t))

        # ATT_DIVERGE：姿态与目标偏差 >15° 持续 1s（无目标则不评）
        att = st.get("att_deg")
        tgt = st.get("att_target_deg")
        dev = None
        if att is not None and tgt is not None:
            dev = max(abs(_wrap_deg(a - b)) for a, b in zip(att, tgt))
        if self._sustained("ATT_DIVERGE", t,
                           dev is not None and dev > self.ATT_DIVERGE_DEG,
                           self.ATT_SUSTAIN_S):
            alerts.append(self._fire("ATT_DIVERGE", t, f"（偏差 {dev:.1f}°）"))

        # RATE_SAT：|角速度| 超限持续 1s
        rate = st.get("rate_dps")
        peak = max((abs(v) for v in rate), default=None) if rate else None
        if self._sustained("RATE_SAT", t,
                           peak is not None and peak > self.RATE_SAT_DPS,
                           self.RATE_SUSTAIN_S):
            alerts.append(self._fire("RATE_SAT", t, f"（峰值 {peak:.0f}°/s）"))

        # VIBE_CLIP：削波计数上升即报（计数器回落=飞控重启，静默重设基线）
        clip = st.get("clip_count")
        if clip is not None:
            if self._last_clip is not None and clip > self._last_clip \
                    and self._cooled("VIBE_CLIP", t):
                alerts.append(self._fire(
                    "VIBE_CLIP", t, f"（{self._last_clip}→{clip}）"))
            self._last_clip = clip

        # BAT_LOW：剩余 <20%（电平型，冷却内不重复）
        rem = st.get("battery_remaining")
        if rem is not None and rem < self.BAT_LOW_FRAC \
                and self._cooled("BAT_LOW", t):
            alerts.append(self._fire("BAT_LOW", t, f"（剩余 {rem * 100:.0f}%）"))

        # DZUAV_SILENT：健康 5s 无更新（从未见过也算，自 start_t 起计）
        if self.expect_dzuav:
            last = st.get("dzuav_t")
            last = last if last is not None else self.start_t
            if t - last > self.DZUAV_SILENT_S and self._cooled("DZUAV_SILENT", t):
                alerts.append(self._fire("DZUAV_SILENT", t))

        # LINK_LAG：快通道采样间隔 p95 > 期望 2 倍
        lag = self._link_lag_ratio(t)
        if lag is not None and lag[1] > self.LINK_LAG_RATIO \
                and self._cooled("LINK_LAG", t):
            alerts.append(self._fire(
                "LINK_LAG", t, f"（{lag[0]} p95 间隔为期望 {lag[1]:.1f} 倍）"))
        return alerts


def _wrap_deg(a: float) -> float:
    """角度归一化到 (-180, 180]。"""
    return (a + 180.0) % 360.0 - 180.0


def _percentile(sorted_vals, pct: float) -> float:
    """已排序序列的分位数（线性插值，纯标准库，免得为 report 引 numpy）。"""
    if not sorted_vals:
        raise ValueError("空序列无分位数")
    if len(sorted_vals) == 1:
        return float(sorted_vals[0])
    rank = pct / 100.0 * (len(sorted_vals) - 1)
    lo = int(rank)
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = rank - lo
    return float(sorted_vals[lo] * (1 - frac) + sorted_vals[hi] * frac)


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
    "arm":     {"cmd": 400, "params": {"param1": 1.0}},
    "disarm":  {"cmd": 400, "params": {"param1": 0.0}},
    "takeoff": {"cmd": 22, "params": {"param7": 2.0}},
    "land":    {"cmd": 21, "params": {}},
    "return":  {"cmd": 20, "params": {}},
    "kill":    {"cmd": 185, "params": {"param1": 1.0}},
}
COMMAND_TOPIC_HINT = "vehicle_command"
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
    # 有 best_effort 发布就必须同档订阅，否则 reliable（track_diag 真车踩过的坑）
    for r in reliabilities:
        if str(r).lower() == "best_effort":
            return "best_effort"
    return "reliable"


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
        # estimator_status*/estimator_status_flags 版本差异大：把 *_flags 数值全收，
        # ok = 全部为零（一个都不存在时 ok=None，笔记本端自然不评 EKF 规则）
        flags = {}
        for name in ("filter_fault_flags", "innovation_check_flags",
                     "solution_status_flags", "control_mode_flags", "fault_flags"):
            v = _g(msg, name)
            if isinstance(v, (int, float)):
                flags[name] = int(v)
        out = {"flags": flags}
        out["ok"] = all(v == 0 for v in flags.values()) if flags else None
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
            last = None
            with open(newest, "r", encoding="utf-8", errors="replace") as fh:
                for line in fh:
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
        last = self._last.get(key)
        if last is None or t - last >= 1.0 / cap - 1e-9:
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
    channel = find_command_channel(found)
    if channel is None:
        # 响亮失败：列出实际发现的话题，绝不盲发
        emit({"ctl": args.do, "ok": False,
              "detail": "未发现命令通道（期望话题名含 %r）；实际发现: %s"
                        % (COMMAND_TOPIC_HINT, ", ".join(found) or "（空）")})
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(4)
    spec = COMMAND_MAP[args.do]
    # 找到该话题的类型（vehicle_command 一定在枚举里，否则上面已退出）
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
    params = dict(spec["params"])
    if args.do == "takeoff":
        params["param7"] = float(args.alt)
    # 宽松赋值：字段存在才写（px4_msgs 版本差异靠这个扛）
    for name, val in [("command", spec["cmd"]), ("target_system", 1),
                      ("target_component", 1), ("source_system", 1),
                      ("source_component", 1), ("from_external", True),
                      ("timestamp", int(time.time() * 1e6))] + list(params.items()):
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
    emit({"ctl": args.do, "ok": True,
          "detail": "已向 %s 连发 10 帧 VehicleCommand cmd=%d params=%s"
                    % (channel, spec["cmd"], params)})
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


# ---------------------------------------------------------------- 遥测状态聚合（仪表盘 + 规则引擎共用）

class TelemetryState:
    """把采样行滚成当前快照。feed() 顺带产出规则引擎输入 dict。"""

    def __init__(self) -> None:
        self.status: dict | None = None      # armed/nav_state/failsafe
        self.att_deg: tuple | None = None    # (r,p,y) 度
        self.att_target_deg: tuple | None = None
        self.rate_dps: tuple | None = None
        self.pos: dict | None = None         # xyz/vel
        self.battery: dict | None = None
        self.estimator_ok: bool | None = None
        self.clip_count: int | None = None
        self.dzuav: dict | None = None
        self.dzuav_t: float | None = None    # 本机钟：最近一次 dzuav 健康行
        self.last_lag_ms: float | None = None
        self.counts: Counter = Counter()     # 每通道消息计数

    @staticmethod
    def _num3(v):
        if isinstance(v, (list, tuple)) and len(v) == 3:
            try:
                return float(v[0]), float(v[1]), float(v[2])
            except (TypeError, ValueError):
                return None
        return None

    def feed(self, t_recv: float, row: dict) -> None:
        """row 为 parse_sampler_line 的 sample 规范形。"""
        ch = row.get("ch")
        data = row.get("data") or {}
        self.counts[ch or "?"] += 1
        if row.get("t") is not None:
            self.last_lag_ms = max(0.0, (t_recv - row["t"]) * 1000.0)
        if ch == "status":
            self.status = data
        elif ch == "attitude":
            self.att_deg = self._num3(data.get("rpy_deg")) or self.att_deg
        elif ch == "attitude_target":
            self.att_target_deg = self._num3(data.get("rpy_deg")) or self.att_target_deg
        elif ch == "rates":
            self.rate_dps = self._num3(data.get("dps")) or self.rate_dps
        elif ch == "position":
            self.pos = data
        elif ch == "battery":
            self.battery = data
        elif ch == "estimator":
            if data.get("ok") is not None:
                self.estimator_ok = bool(data["ok"])
        elif ch == "imu":
            if data.get("clip") is not None:
                self.clip_count = int(data["clip"])
        elif ch == "dzuav":
            self.dzuav = data or None
            self.dzuav_t = t_recv   # 通道行到达即算活跃（data 可为 null）

    def engine_input(self) -> dict:
        rem = None
        if self.battery:
            rem = _float_or_none(self.battery.get("remaining"))
        return {"failsafe": bool(self.status.get("failsafe")) if self.status else None,
                "estimator_ok": self.estimator_ok,
                "att_deg": self.att_deg,
                "att_target_deg": self.att_target_deg,
                "rate_dps": self.rate_dps,
                "clip_count": self.clip_count,
                "battery_remaining": rem,
                "dzuav_t": self.dzuav_t}


def render_dashboard(st: TelemetryState, active: str, now: float | None = None) -> str:
    """2Hz 终端仪表盘（单行滚动，Windows GBK 控制台安全）。"""
    now = time.time() if now is None else now
    s = st.status or {}
    armed = "ARMED" if s.get("armed") else ("disarmed" if st.status else "--")
    nav = s.get("nav_state")
    fs = " FAILSAFE!" if s.get("failsafe") else ""
    rpy = ("r%+05.1f p%+05.1f y%+05.1f" % st.att_deg) if st.att_deg else "姿态--"
    rate = ("ω%.0f/%.0f/%.0f°/s" % st.rate_dps) if st.rate_dps else "ω--"
    if st.pos and st.pos.get("xyz"):
        x, y, z = st.pos["xyz"]
        pos = f"pos({x:+.1f},{y:+.1f},{z:+.1f})m"
        if st.pos.get("vel"):
            vx, vy, vz = st.pos["vel"]
            pos += f" vel({vx:+.1f},{vy:+.1f},{vz:+.1f})"
    else:
        pos = "pos--"
    if st.battery and st.battery.get("voltage") is not None:
        rem = st.battery.get("remaining")
        bat = f"{st.battery['voltage']:.1f}V"
        bat += f" {rem * 100:.0f}%" if rem is not None else ""
    else:
        bat = "bat--"
    lag = f"{st.last_lag_ms:.0f}ms" if st.last_lag_ms is not None else "--"
    if st.dzuav:
        dz = f"dzuav:loc={int(bool(st.dzuav.get('localized')))} lm={st.dzuav.get('landmarks')}"
    else:
        dz = "dzuav:无数据"
    return (f"{time.strftime('%H:%M:%S', time.localtime(now))} "
            f"{armed}{fs} nav={nav} | {rpy} | {rate} | {pos} | {bat} | "
            f"链路 {lag} | {dz} | 告警: {active}")


# ---------------------------------------------------------------- SSH 链路（笔记本端）

class OrinLink(threading.Thread):
    """SSH 遥测链路：上传采样器 → setsid 流式跑 → NDJSON 入队。

    远程用 setsid 自成进程组，stop() 时 pkill 整组回收（docs 里有孤儿进程教训）。
    启动命令见 sampler_launch_cmd()：dzros.env（域配置）与 setup.bash 都必须
    显式 source，否则跨不到机载 ROS 域。
    """

    def __init__(self, host: str, user: str, password: str, port: int = 22,
                 sampler_args: str = "") -> None:
        super().__init__(daemon=True, name="uav-link-orin")
        self.host, self.user, self.password, self.port = host, user, password, port
        self.sampler_args = sampler_args
        self.queue: queue.Queue = queue.Queue()
        self.stop_evt = threading.Event()
        self.error: str | None = None
        self._cli = None

    def run(self) -> None:  # pragma: no cover - 需要真实无人机，本地不测
        try:
            import paramiko
        except ImportError:
            self.error = "缺少 paramiko，请先执行: python -m pip install paramiko"
            return
        try:
            cli = paramiko.SSHClient()
            cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            cli.connect(hostname=self.host, port=self.port, username=self.user,
                        password=self.password, timeout=30, banner_timeout=30,
                        auth_timeout=30)
            self._cli = cli
            sftp = cli.open_sftp()
            with sftp.open(SAMPLER_REMOTE, "w") as f:
                f.write(SAMPLER_SRC)
            sftp.close()
            _, stdout, stderr = cli.exec_command(
                sampler_launch_cmd(self.sampler_args), get_pty=False)
            for line in iter(stdout.readline, ""):
                if self.stop_evt.is_set():
                    break
                obj = parse_sampler_line(line)
                if obj is not None:
                    self.queue.put((time.time(), obj))
            if not self.stop_evt.is_set():
                code = stdout.channel.recv_exit_status()
                err = stderr.read().decode("utf-8", errors="replace")[:500]
                self.error = f"采样器退出（码 {code}）: {err.strip()}"
        except Exception as e:  # noqa: BLE001 - 网络层异常原样上报
            if not self.stop_evt.is_set():
                self.error = f"{type(e).__name__}: {e}"

    def stop(self) -> None:  # pragma: no cover - 需要真实无人机，本地不测
        self.stop_evt.set()
        try:
            if self._cli is not None:
                # setsid 让采样器自成进程组，pkill 整组回收，不留孤儿
                self._cli.exec_command(f"pkill -f {SAMPLER_REMOTE}", timeout=5)
        except Exception:  # noqa: BLE001
            pass
        try:
            if self._cli is not None:
                self._cli.close()
        except Exception:  # noqa: BLE001
            pass


def _connect(host: str, user: str, password: str, port: int = 22, timeout: float = 30.0):
    """建立一次性 SSH 连接（ctl/logs 用；run 走 OrinLink 线程）。"""
    import paramiko
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(hostname=host, port=port, username=user, password=password,
                timeout=timeout, banner_timeout=timeout, auth_timeout=timeout)
    return cli


def _upload_sampler(cli) -> None:  # pragma: no cover - 需要真实无人机
    sftp = cli.open_sftp()
    try:
        with sftp.open(SAMPLER_REMOTE, "w") as f:
            f.write(SAMPLER_SRC)
    finally:
        sftp.close()


def _exec_collect(cli, cmd: str, timeout: float = 60.0):  # pragma: no cover
    """执行远程命令收全量输出，返回 (退出码, stdout, stderr)。"""
    _, stdout, stderr = cli.exec_command(cmd, timeout=timeout, get_pty=False)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return stdout.channel.recv_exit_status(), out, err


def _link_preflight(args) -> tuple[str, str] | tuple[None, None]:
    """ctl/logs 共用的前置：解析主机 → TCP 探测 → 密码。失败打印并返回 (None, None)。"""
    host = resolve_host(args)
    if not host:
        print("[失败] 未给 --host 且无上次缓存（先跑一次 --host <IP> 成功后会记住）",
              file=sys.stderr)
        return None, None
    bad = probe_ssh(host, getattr(args, "port", 22))
    if bad:
        print(f"[失败] {bad}", file=sys.stderr)
        return None, None
    pw = read_password(args)
    save_cached_host(host)
    return host, pw


# ---------------------------------------------------------------- run 主模式（只读遥测 + 黑匣子）

def _summary_lines(state: TelemetryState, counts: Counter, span_s: float,
                   n_rows: int) -> list[str]:
    """收尾统计：消息计数/丢包估计/各告警命中数。"""
    lines = ["==== 收尾统计 ===="]
    lines.append(f"时长 {span_s:.1f}s，黑匣子 {n_rows} 行")
    per = ", ".join(f"{k}x{v}" for k, v in sorted(state.counts.items())) or "无"
    lines.append(f"采样消息: {per}")
    # 丢包估计：按 attitude 声明 10Hz 名义对比实收
    att = state.counts.get("attitude", 0)
    if att and span_s > 1.0:
        expect = span_s * RATE_CAPS_HZ["attitude"]
        lines.append(f"丢包估计: attitude 实收 {att} / 名义 {expect:.0f}"
                     f"（≈{max(0.0, 1 - att / expect) * 100:.0f}% 缺口，链路/飞控双端都可能）")
    lines.append("告警命中: " + (", ".join(f"{k}x{v}" for k, v in counts.items())
                                 if counts else "无"))
    return lines


def cmd_run(args) -> int:  # pragma: no cover - 主流程需要真实无人机，纯逻辑已单测
    host = resolve_host(args)
    if not host:
        print("[失败] 未给 --host 且无上次缓存。用法: "
              "python tools/uav_link.py run --host <Orin IP>", file=sys.stderr)
        return 2
    bad = probe_ssh(host, args.port)
    if bad:
        print(f"[失败] {bad}", file=sys.stderr)
        return 4
    pw = read_password(args)
    save_cached_host(host)

    sampler_args = " --no-dzuav" if args.no_dzuav else ""
    link = OrinLink(host, args.orin_user, pw, args.port, sampler_args)
    link.start()
    print(f"[机载] 已发起 SSH 遥测 {args.orin_user}@{host} …（Ctrl-C 优雅退出）")

    out_path = Path(args.out) if args.out else \
        Path("logs") / "uav_link" / (time.strftime("%Y%m%d_%H%M%S") + ".jsonl")
    box = BlackboxWriter(out_path)
    print(f"[黑匣子] {out_path}")

    state = TelemetryState()
    engine = UavRuleEngine()
    engine.expect_dzuav = not args.no_dzuav
    alerts: list[dict] = []
    n_rows = 0
    hello_shown = False
    err_shown = False
    last_print = 0.0
    t_start = time.time()
    try:
        while True:
            t = time.time()
            latest_rows = []
            while True:
                try:
                    latest_rows.append(link.queue.get_nowait())
                except queue.Empty:
                    break
            for t_recv, obj in latest_rows:
                n_rows += 1
                box.write(obj, recv=t_recv)
                if obj["kind"] == "hello" and not hello_shown:
                    hello_shown = True
                    tf = obj["topics_found"]
                    print(f"[机载] 采样器发现 {len(tf)} 个话题: "
                          + (", ".join(tf[:12]) + (" …" if len(tf) > 12 else "")
                             if tf else "（空——MicroXRCEAgent 可能没起）"))
                elif obj["kind"] == "error":
                    if not err_shown:
                        err_shown = True
                        print(f"[机载] 采样器报错：{obj['error']}")
                elif obj["kind"] == "sample":
                    state.feed(t_recv, obj)
                    engine.note_sample(t_recv, obj.get("topic", "?"),
                                       obj.get("hz"))
            if link.error and not err_shown:
                err_shown = True
                print(f"[机载] 链路异常：{link.error}")

            fired = engine.update(t, state.engine_input())
            for a in fired:
                print(f"[告警] {a['rule']}: {a['msg']}")
                box.write({"src": "alert", "rule": a["rule"], "msg": a["msg"]},
                          recv=t)
            alerts.extend(fired)

            if t - last_print >= 0.5:   # 2Hz 仪表盘
                last_print = t
                active = ",".join(sorted({a["rule"] for a in alerts
                                          if t - a["t"] <= 5.0})) or "无"
                print(render_dashboard(state, active, now=t))
            if link.error and link.queue.empty():
                break   # 链路已死且无积余，退出循环走收尾
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n[退出] Ctrl-C，回收远程进程组…")
    finally:
        link.stop()
        link.join(timeout=5)
        span = time.time() - t_start
        lines = _summary_lines(state, engine.counts, span, n_rows)
        for ln in lines:
            print(ln)
        box.write({"summary": lines})
        box.close()
        print(f"[黑匣子已收尾] {out_path}")
    return 0


# ---------------------------------------------------------------- ctl 调控支配（全部显式闸门）

def cmd_ctl(args) -> int:  # pragma: no cover - 远端交互不测，闸门/映射纯逻辑已单测
    if not args.action and not args.print_caps:
        print("[失败] 缺动作：uav_link.py ctl <" + "/".join(ALL_ACTIONS) + ">\n"
              + render_command_map(), file=sys.stderr)
        return 2
    action = args.action
    if action:
        # 安全闸门：高风险没 --yes 直接拒；次高风险醒目警告 + 倒计时
        deny = check_gate(action, args.yes)
        if deny:
            print(f"[拒绝] {deny}", file=sys.stderr)
            return 2
        countdown(action, yes=args.yes)

    host, pw = _link_preflight(args)
    if not host:
        return 4
    box = BlackboxWriter(Path("logs") / "uav_link" / "ctl_receipts.jsonl", append=True)
    try:
        cli = _connect(host, args.orin_user, pw, args.port)
    except Exception as e:  # noqa: BLE001 - 网络层异常原样报出便于现场判断
        print(f"[失败] SSH 连接失败: {type(e).__name__}: {e}", file=sys.stderr)
        return 4

    try:
        if args.print_caps:
            _upload_sampler(cli)
            rc, out, err = _exec_collect(cli, sampler_launch_cmd(" --print-caps"),
                                         timeout=60)
            sys.stdout.write(out)
            if err.strip():
                sys.stderr.write("--- stderr ---\n" + err[:500])
            print()
            print(render_command_map())
            return 0 if rc == 0 else 1

        if action in COMMAND_MAP:
            _upload_sampler(cli)
            extra = f" --do {action}"
            if action == "takeoff":
                extra += f" --alt {args.alt}"
            rc, out, err = _exec_collect(cli, sampler_launch_cmd(extra), timeout=90)
            ctl_row = None
            found_topics: list[str] = []
            for line in out.splitlines():
                obj = parse_sampler_line(line)
                if not obj:
                    continue
                if obj["kind"] == "hello":
                    found_topics = obj["topics_found"]
                elif obj["kind"] == "ctl":
                    ctl_row = obj
                elif obj["kind"] == "error":
                    print(f"[机载] {obj['error']}", file=sys.stderr)
            ok = bool(ctl_row and ctl_row["ok"])
            detail = ctl_row["detail"] if ctl_row else \
                f"采样器未回执（退出码 {rc}）: {err.strip()[:300]}"
            if ok:
                print(f"[完成] ctl {action}：{detail}")
            else:
                print(render_channel_failure(action, found_topics) if not ctl_row
                      else f"[失败] ctl {action}：{detail}", file=sys.stderr)
            box.write({"src": "ctl", "action": action, "ok": ok, "detail": detail})
            return 0 if ok else 5

        # ---- 栈生命周期（tmux / orin_bringup.sh，不走 ROS）----
        script = f"{ORIN_REPO}/tools/orin_bringup.sh"
        if action == "stack-status":
            remote = (f"if [ -f {script} ]; then bash {script} status; "
                      f"else tmux has-session -t {TMUX_SESSION} 2>/dev/null && "
                      f"tmux list-windows -t {TMUX_SESSION} "
                      f"-F '#{{window_name}} #{{pane_current_command}}' "
                      f"|| echo 'tmux 会话 {TMUX_SESSION} 不存在'; fi")
        else:  # stack-restart（已过 --yes 闸门）
            remote = (f"if [ -f {script} ]; then bash {script} down; sleep 2; "
                      f"bash {script} up; "
                      f"else echo 'Orin 上未找到 {script}（先 orin_deploy 同步仓库）'; "
                      f"exit 6; fi")
        # exec_command 本就过登录 shell，if/&&/|| 直接可用（不再套 bash -lc：
        # remote 内含单引号，外层再单引号包裹会被截断）
        rc, out, err = _exec_collect(cli, remote, timeout=120)
        sys.stdout.write(out)
        if err.strip():
            sys.stderr.write("--- stderr ---\n" + err[:500])
        box.write({"src": "ctl", "action": action, "ok": rc == 0,
                   "detail": out.strip()[:300]})
        return 0 if rc == 0 else 1
    finally:
        box.close()
        cli.close()


# ---------------------------------------------------------------- logs 取回日志

# 机载端暂存脚本（单行分号串，bash -lc 单引号包裹，内层不得有单引号）：
# dzuav jsonl 最近 5 个 + ROS log 最近 3 次打包 + tmux 各窗口 capture-pane。
_LOGS_STAGE_CMD = (
    "ST=/tmp/uav_logs_stage; rm -rf $ST; mkdir -p $ST; "
    "ls -t /tmp/ag_logs/*/dzuav-*.jsonl ~/ag_logs/*/dzuav-*.jsonl 2>/dev/null "
    "| head -5 | while read f; do cp \"$f\" $ST/ 2>/dev/null || true; done; "
    "if [ -d ~/.ros/log ]; then (cd ~/.ros/log && ls -t | head -3 "
    "| tar czf $ST/ros_log_recent.tgz -T -) 2>/dev/null || true; fi; "
    + " ".join(f"tmux capture-pane -pt {TMUX_SESSION}:{w} -S -300 "
               f"> $ST/tmux_{w}.txt 2>/dev/null || true;" for w in TMUX_WINDOWS)
    + " ls -la $ST"
)


def _sftp_get_tree(cli, remote: str, local: Path) -> int:  # pragma: no cover
    """递归取回远程目录到本地，返回文件数。"""
    import stat as stat_mod
    sftp = cli.open_sftp()
    n = 0
    try:
        def walk(r: str, l: Path) -> None:
            nonlocal n
            l.mkdir(parents=True, exist_ok=True)
            for e in sftp.listdir_attr(r):
                rp, lp = f"{r}/{e.filename}", l / e.filename
                if stat_mod.S_ISDIR(e.st_mode):
                    walk(rp, lp)
                else:
                    sftp.get(rp, str(lp))
                    n += 1
        st = sftp.stat(remote)
        if stat_mod.S_ISDIR(st.st_mode):
            walk(remote, local)
        else:
            local.parent.mkdir(parents=True, exist_ok=True)
            sftp.get(remote, str(local))
            n = 1
    finally:
        sftp.close()
    return n


def cmd_logs(args) -> int:  # pragma: no cover - 需要真实无人机
    host, pw = _link_preflight(args)
    if not host:
        return 4
    out_dir = Path(args.out) if args.out else \
        Path("logs") / "uav_link" / time.strftime("%Y%m%d_%H%M%S")
    try:
        cli = _connect(host, args.orin_user, pw, args.port)
    except Exception as e:  # noqa: BLE001
        print(f"[失败] SSH 连接失败: {type(e).__name__}: {e}", file=sys.stderr)
        return 4
    try:
        rc, out, err = _exec_collect(cli, f"bash -lc '{_LOGS_STAGE_CMD}'",
                                     timeout=120)
        if rc != 0:
            print(f"[警告] 机载暂存脚本退出码 {rc}（继续尽量取回）: "
                  f"{err.strip()[:300]}", file=sys.stderr)
        n = _sftp_get_tree(cli, "/tmp/uav_logs_stage", out_dir)
    finally:
        cli.close()
    # 清单与大小
    print(f"[已取回] {n} 个文件 -> {out_dir}")
    total = 0
    for p in sorted(out_dir.rglob("*")):
        if p.is_file():
            sz = p.stat().st_size
            total += sz
            print(f"  {p.relative_to(out_dir)}  {sz / 1024:.1f} KiB")
    print(f"  合计 {total / 1024:.1f} KiB")
    return 0


# ---------------------------------------------------------------- report 离线复盘

SEVERITY_LABEL = {"critical": "严重", "warn": "注意", "info": "信息"}
_SEVERITY_RANK = {"critical": 0, "warn": 1, "info": 2}


def _row_t(row: dict) -> float | None:
    """黑匣子行的时间：采样器 t 优先，退化本机 recv。

    ⚠️ 必须显式判 None 不能用 or：t=0.0 是合法时刻，or 会把它当空值退化到 recv。
    """
    t = _float_or_none(row.get("t"))
    return t if t is not None else _float_or_none(row.get("recv"))


def build_report(rows: list[dict]) -> dict:
    """黑匣子行 → 复盘数据（纯函数，可测）。

    输入为 read_blackbox 的原始行（含 recv/topic/ch/data 与 ctl/alert/hello 行）。
    """
    rep: dict = {
        "n_rows": len(rows),
        "per_ch": Counter(),
        "span_s": 0.0,
        "topics_found": [],
        "armed_timeline": [],     # [(t, armed, nav_state)]（含首帧）
        "mode_switches": [],      # [(t, nav_state)]
        "failsafe_events": [],    # [(t, nav_state)]（False→True 跳变）
        "att_abs_roll": [], "att_abs_pitch": [], "rate_max": [],
        "att_divergence": [],     # [(t_start, t_end, max_deg)]，阈值同 ATT_DIVERGE
        "vibe_clip_total": 0,
        "link_gaps": [],          # [(t_start, t_end, dur)]，>2s 无消息段
        "battery": {"min_remaining": None, "min_voltage": None},
        "dzuav": {"rows": 0, "null_rows": 0},
        "ctl_receipts": [],       # [{"action","ok","detail"}]
        "alert_counts": Counter(),
    }
    t0 = t1 = None
    last_status: tuple | None = None        # (armed, nav_state, failsafe)
    last_att = last_tgt = None
    div_seg = None                          # [t_start, t_end, max_deg]
    last_clip = None
    prev_sample_t = None
    for row in rows:
        if row.get("src") == "ctl" or row.get("action") and "topic" not in row:
            if row.get("src") == "ctl":
                rep["ctl_receipts"].append(
                    {"action": row.get("action"), "ok": bool(row.get("ok")),
                     "detail": str(row.get("detail", ""))[:200]})
            continue
        if "hello" in row:
            tf = row.get("topics_found")
            if isinstance(tf, list):
                rep["topics_found"] = [str(x) for x in tf]
            continue
        if row.get("src") == "alert" and row.get("rule"):
            rep["alert_counts"][str(row["rule"])] += 1
            continue
        if "summary" in row or "topic" not in row:
            continue
        # ---- 采样行 ----
        t = _row_t(row)
        topic = str(row["topic"])
        ch = str(row["ch"]) if row.get("ch") else classify_topic(topic) or "?"
        data = row.get("data") if isinstance(row.get("data"), dict) else {}
        rep["per_ch"][ch] += 1
        if t is not None:
            t0 = t if t0 is None else min(t0, t)
            t1 = t if t1 is None else max(t1, t)
            if prev_sample_t is not None and t - prev_sample_t > 2.0:
                rep["link_gaps"].append((prev_sample_t, t, t - prev_sample_t))
            prev_sample_t = t

        if ch == "status":
            cur = (bool(data.get("armed")), data.get("nav_state"),
                   bool(data.get("failsafe")))
            if t is not None:
                if last_status is None or cur[0] != last_status[0]:
                    rep["armed_timeline"].append((t, cur[0], cur[1]))
                if last_status is not None and cur[1] != last_status[1]:
                    rep["mode_switches"].append((t, cur[1]))
                if cur[2] and (last_status is None or not last_status[2]):
                    rep["failsafe_events"].append((t, cur[1]))
            last_status = cur
        elif ch == "attitude":
            rpy = TelemetryState._num3(data.get("rpy_deg"))
            if rpy:
                rep["att_abs_roll"].append(abs(rpy[0]))
                rep["att_abs_pitch"].append(abs(rpy[1]))
                last_att = rpy
        elif ch == "attitude_target":
            tgt = TelemetryState._num3(data.get("rpy_deg"))
            if tgt:
                last_tgt = tgt
        elif ch == "rates":
            dps = TelemetryState._num3(data.get("dps"))
            if dps:
                rep["rate_max"].append(max(abs(v) for v in dps))
        elif ch == "imu":
            clip = data.get("clip")
            if isinstance(clip, (int, float)):
                if last_clip is not None and clip > last_clip:
                    rep["vibe_clip_total"] += int(clip - last_clip)
                last_clip = int(clip)
        elif ch == "battery":
            rem, vol = _float_or_none(data.get("remaining")), \
                _float_or_none(data.get("voltage"))
            b = rep["battery"]
            if rem is not None:
                b["min_remaining"] = rem if b["min_remaining"] is None \
                    else min(b["min_remaining"], rem)
            if vol is not None:
                b["min_voltage"] = vol if b["min_voltage"] is None \
                    else min(b["min_voltage"], vol)
        elif ch == "dzuav":
            rep["dzuav"]["rows"] += 1
            if not data:
                rep["dzuav"]["null_rows"] += 1

        # 姿态-目标偏差时段（>15°，阈值与实时规则一致）
        if t is not None and last_att is not None and last_tgt is not None:
            dev = max(abs(_wrap_deg(a - b)) for a, b in zip(last_att, last_tgt))
            if dev > UavRuleEngine.ATT_DIVERGE_DEG:
                if div_seg is None:
                    div_seg = [t, t, dev]
                else:
                    div_seg[1], div_seg[2] = t, max(div_seg[2], dev)
            elif div_seg is not None:
                rep["att_divergence"].append(tuple(div_seg))
                div_seg = None
    if div_seg is not None:
        rep["att_divergence"].append(tuple(div_seg))
    if t0 is not None and t1 is not None:
        rep["span_s"] = t1 - t0
    return rep


def _stat_line(vals) -> str:
    """mean/p95/max 一行（vals 非空）。"""
    s = sorted(vals)
    return (f"mean={sum(s) / len(s):.2f} p95={_percentile(s, 95):.2f} "
            f"max={s[-1]:.2f}")


def build_findings(rep: dict) -> list[dict]:
    """复盘数据 → 分级结论（严重/注意/信息，照 ag_core log_diagnosis 分级）。"""
    fs: list[dict] = []

    def add(sev: str, title: str, detail: str = "") -> None:
        fs.append({"severity": sev, "title": title, "detail": detail})

    n_fs = len(rep["failsafe_events"])
    if n_fs:
        times = "、".join(f"t={t:.1f}s" for t, _ in rep["failsafe_events"][:5])
        add("critical", f"FAILSAFE 事件 {n_fs} 次",
            f"发生点：{times}——查触发前后姿态/电池/链路")
    b = rep["battery"]
    if b["min_remaining"] is not None:
        if b["min_remaining"] < 0.10:
            add("critical", f"电池电量极低（最低 {b['min_remaining'] * 100:.0f}%）",
                "飞控不稳可能只是饿的——先换满电电池再排查")
        elif b["min_remaining"] < 0.20:
            add("warn", f"电池电量偏低（最低 {b['min_remaining'] * 100:.0f}%）")
    if rep["link_gaps"]:
        longest = max(g[2] for g in rep["link_gaps"])
        add("warn", f"链路空洞 {len(rep['link_gaps'])} 段（>2s 无消息，最长 "
                    f"{longest:.1f}s）",
            "空洞内的飞控行为完全不可见——查 Wi-Fi 信号/MicroXRCEAgent 重启")
    if rep["vibe_clip_total"]:
        add("warn", f"加速度计削波累计 +{rep['vibe_clip_total']}",
            "振动已达削波——查桨叶损伤/机架松动/减震座；振动是姿态发散常见根因")
    if rep["att_divergence"]:
        worst = max(s[2] for s in rep["att_divergence"])
        add("warn", f"姿态-目标偏差 >15° 共 {len(rep['att_divergence'])} 段"
                    f"（最大 {worst:.1f}°）",
            "姿态跟踪回路异常——结合角速度峰值与削波看是振动还是调参")
    if rep["rate_max"] and max(rep["rate_max"]) > UavRuleEngine.RATE_SAT_DPS:
        add("warn", f"角速度峰值 {max(rep['rate_max']):.0f}°/s 超 "
                    f"{UavRuleEngine.RATE_SAT_DPS:.0f}°/s",
            "机体在甩——先查振动，再看速率环调参")
    dz = rep["dzuav"]
    if dz["rows"] and dz["null_rows"] == dz["rows"]:
        add("warn", "dzuav 健康通道全程无数据",
            "侦察进程可能没起——tmux 看 dzuav 窗口，或 orin_log_capture 是否在场")
    if not rep["armed_timeline"]:
        add("info", "全程未见 vehicle_status 采样行",
            "话题名变了或 MicroXRCEAgent 没起——对照 topics_found 检查")
    if not rep["per_ch"]:
        add("info", "黑匣子里没有采样行", "可能 run 刚启动就退出，或采样器在报错")
    return sorted(fs, key=lambda f: _SEVERITY_RANK[f["severity"]])


def render_report(rep: dict) -> list[str]:
    """复盘数据 → 打印行（严重/注意/信息 分级输出）。"""
    lines = ["===== 黑匣子复盘 ====="]
    per = ", ".join(f"{k}x{v}" for k, v in sorted(rep["per_ch"].items())) or "无"
    lines.append(f"[信息] 时长 {rep['span_s']:.1f}s，采样行 "
                 f"{sum(rep['per_ch'].values())} / 总行 {rep['n_rows']}（{per}）")
    if rep["topics_found"]:
        lines.append(f"[信息] 采样器发现 {len(rep['topics_found'])} 个话题: "
                     + ", ".join(rep["topics_found"][:8])
                     + (" …" if len(rep["topics_found"]) > 8 else ""))
    # armed/模式时间线
    if rep["armed_timeline"]:
        tl = "；".join(f"t={t:.1f}s {'ARMED' if a else 'disarmed'}(nav={n})"
                       for t, a, n in rep["armed_timeline"][:10])
        lines.append(f"[信息] armed 时间线: {tl}")
    if rep["mode_switches"]:
        sw = "；".join(f"t={t:.1f}s nav→{n}" for t, n in rep["mode_switches"][:10])
        lines.append(f"[信息] 模式切换: {sw}")
    # 姿态/角速度统计
    if rep["att_abs_roll"]:
        lines.append(f"[信息] |roll|(°): {_stat_line(rep['att_abs_roll'])}")
        lines.append(f"[信息] |pitch|(°): {_stat_line(rep['att_abs_pitch'])}")
    if rep["rate_max"]:
        lines.append(f"[信息] 角速度峰值(°/s): {_stat_line(rep['rate_max'])}")
    if rep["ctl_receipts"]:
        rc = "；".join(f"{r['action']}({'成' if r['ok'] else '败'})"
                       for r in rep["ctl_receipts"][:10])
        lines.append(f"[信息] ctl 回执 {len(rep['ctl_receipts'])} 条: {rc}")
    if rep["alert_counts"]:
        ac = ", ".join(f"{k}x{v}" for k, v in rep["alert_counts"].items())
        lines.append(f"[信息] 实时告警命中: {ac}")
    # 分级结论（严重 → 注意 → 信息）
    findings = build_findings(rep)
    if findings:
        lines.append("---- 诊断结论 ----")
        for f in findings:
            line = f"[{SEVERITY_LABEL[f['severity']]}] {f['title']}"
            if f["detail"]:
                line += f"——{f['detail']}"
            lines.append(line)
    else:
        lines.append("[信息] 未发现异常模式（不代表飞控没问题，只是这份日志里没有）")
    return lines


def cmd_report(args) -> int:
    p = Path(args.blackbox)
    if not p.is_file():
        print(f"[失败] 黑匣子不存在: {p}", file=sys.stderr)
        return 2
    rows = read_blackbox(p)
    if not rows:
        print(f"[失败] 黑匣子为空或无有效行: {p}", file=sys.stderr)
        return 1
    print(f"[黑匣子] {p}")
    rep = build_report(rows)
    for ln in render_report(rep):
        print(ln)
    return 0


# ---------------------------------------------------------------- CLI

def _add_link_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--host", default=None,
                   help="Orin IP（给过一次就缓存到 uav_link.local.env，之后可省）")
    p.add_argument("--orin-user", default=ORIN_USER_DEFAULT,
                   help=f"机载用户名（默认 {ORIN_USER_DEFAULT}）")
    p.add_argument("--port", type=int, default=22)
    p.add_argument("--password", default=None,
                   help="机载密码；更推荐 AG_ORIN_PASS 环境变量")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="uav_link.py",
        description="无人机地面站：看得见（遥测）、控得住（调控）、查得回（黑匣子+复盘）"
                    "（详见文件头说明）")
    sub = ap.add_subparsers(dest="cmd")

    rn = sub.add_parser("run", help="只读遥测 + 黑匣子 + 实时告警（默认）")
    _add_link_args(rn)
    rn.add_argument("--out", default=None,
                    help="黑匣子路径（默认 logs/uav_link/<时间戳>.jsonl）")
    rn.add_argument("--no-dzuav", action="store_true",
                    help="不并 dzuav 健康通道（也不评 DZUAV_SILENT）")
    rn.set_defaults(func=cmd_run)

    ct = sub.add_parser("ctl", help="调控支配（默认什么都不做，高风险要 --yes）")
    ct.add_argument("action", nargs="?", choices=ALL_ACTIONS,
                    help="arm/disarm/takeoff/land/return/kill/stack-status/stack-restart")
    ct.add_argument("--yes", action="store_true",
                    help="高风险动作确认闸门；对倒计时动作跳过 3s 倒计时")
    ct.add_argument("--alt", type=float, default=2.0,
                    help="takeoff 目标高度（米，默认 2.0）")
    ct.add_argument("--print-caps", action="store_true",
                    help="打印采样器实际发现的话题/命令通道 + 命令映射表后退出")
    _add_link_args(ct)
    ct.set_defaults(func=cmd_ctl)

    lg = sub.add_parser("logs", help="取回机载日志（dzuav jsonl/ROS log/tmux 窗口）")
    _add_link_args(lg)
    lg.add_argument("--out", default=None,
                    help="本地目录（默认 logs/uav_link/<时间戳>/）")
    lg.set_defaults(func=cmd_logs)

    rp = sub.add_parser("report", help="离线复盘黑匣子（严重/注意/信息 分级）")
    rp.add_argument("blackbox", help="黑匣子 jsonl 路径")
    rp.set_defaults(func=cmd_report)
    return ap


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    # 默认子命令是 run：`uav_link.py --host ...` 等价 `uav_link.py run --host ...`
    subs = {"run", "ctl", "logs", "report"}
    if argv and argv[0] not in subs and argv[0] not in ("-h", "--help"):
        if argv[0].startswith("-"):
            argv.insert(0, "run")
        else:
            print(f"未知子命令: {argv[0]}（应为 run / ctl / logs / report）",
                  file=sys.stderr)
            return 2
    elif not argv:
        argv = ["run"]
    ap = build_parser()
    args = ap.parse_args(argv)
    if not getattr(args, "func", None):
        ap.print_help()
        return 0
    # ⚠️ 同 remote_run.py：Windows 控制台默认 GBK，先开容错防止特殊符号崩整条命令
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(errors="replace")
        except Exception:  # noqa: BLE001
            pass
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
