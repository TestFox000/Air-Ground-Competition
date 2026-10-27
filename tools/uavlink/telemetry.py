# uavlink.telemetry —— 由 tools/uav_link.py 拆出（采样行解析、话题分类、遥测快照与终端仪表盘）。与原文件逐行一致（AST 校验过）。
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
from uavlink.config import COMMAND_MAP, COMMAND_TOPIC_HINT, ORIN_REPO, RATE_CAPS_HZ, RATE_DEFAULT_HZ, SAFE_CHANNEL_HINT, TMUX_SESSION, TOPIC_CANDIDATES

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
    """按发布端 QoS 选订阅档：有 best_effort 就 best_effort。

    ⚠️ track_diag 真车踩过的坑：DDS 里 reliable 订阅收不到 best_effort 发布，
    反向可以（writer 可靠性 ≥ reader 即兼容）——所以订阅端在拿不准时必须按
    best_effort。空列表 = DDS 发现未跑完的竞态（2026-10-16 实机：
    local_position/vehicle_status 被订成 reliable 全程静默），同样按 best_effort。
    reliabilities 为标签串列表（"best_effort"/"reliable"/...）。
    """
    for r in reliabilities:
        if str(r).lower() == "best_effort":
            return "best_effort"
    return "best_effort" if not reliabilities else "reliable"
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
def choose_command_route(action: str, found_topics) -> tuple[str, str] | None:
    """命令路径分路（M1，与机载采样器 run_do 同源逻辑，改一边同步另一边）：

      有 /px4_control/command 且该动作有对应 ControlCommand 码
          → ("px4_control", 话题)   优先走状态机防护的安全通道
      否则有 vehicle_command
          → ("fmu_direct", 话题)    直发飞控（绕过防护，执行前醒目提示）
      都没有 → None（响亮失败，绝不盲发）
    """
    if COMMAND_MAP[action].get("ctrl") is not None:
        for t in found_topics:
            if SAFE_CHANNEL_HINT in str(t):
                return ("px4_control", str(t))
    ch = find_command_channel(found_topics)
    return ("fmu_direct", ch) if ch is not None else None
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
    lines.append(f"命令通道分层：arm/disarm/takeoff/land 优先走 "
                 f"{SAFE_CHANNEL_HINT!r}（px4_control 状态机防护）；找不到才直发 "
                 f"{COMMAND_TOPIC_HINT!r}（执行前醒目提示）；kill/return 恒直发")
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
      {"kind": "warn",   "msg": "..."}                        采样器醒目提示（直发警告等）
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
    if obj.get("warn"):
        return {"kind": "warn", "msg": str(obj["warn"])}
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
# ---------------------------------------------------------------- 告警规则引擎（纯逻辑、可测）

def estimator_flags_ok(flags: dict) -> bool | None:
    """estimator flags 字典 → 健康判定（S2，与机载采样器 slim("estimator") 同源）。

    ok 只由 filter_fault_flags / innovation_check_flags 决定（非零=坏）。
    ⚠️ solution_status_flags 是"哪些状态输出有效"的位掩码——健康飞行恒非零，
    它绝不参与零判定（旧逻辑"全部 flags 为零=健康"恰好搞反，真机上每次健康
    飞行都会全程轰炸 EKF_DEGRADED 误报，2026-10-17 审查修正）。
    两类坏 flag 一个都不存在时返回 None（信息不足，调用方自然不评 EKF 规则）。
    """
    bad = [flags[n] for n in ("filter_fault_flags", "innovation_check_flags")
           if isinstance(flags.get(n), (int, float))]
    if not bad:
        return None
    return all(v == 0 for v in bad)
def _wrap_deg(a: float) -> float:
    """角度归一化到 [-180, 180)。"""
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
        self.dzuav_t: float | None = None    # 本机钟：最近 data 非空的 dzuav 健康行
        self.dzuav_null_since: float | None = None  # data 持续 null 的起始时刻
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
            # S2：采样器给了 flags 字典就笔记本端重算 ok（estimator_flags_ok：
            # solution_status_flags 不参与零判定）；老采样器只有 ok 字段时沿用
            flags = data.get("flags")
            if isinstance(flags, dict) and flags:
                ok = estimator_flags_ok(flags)
                if ok is not None:
                    self.estimator_ok = ok
            elif data.get("ok") is not None:
                self.estimator_ok = bool(data["ok"])
        elif ch == "imu":
            if data.get("clip") is not None:
                self.clip_count = int(data["clip"])
        elif ch == "dzuav":
            # S1：只有 data 非空才算 dzuav 活跃（采样器每秒都发 dzuav.health 行，
            # 即使 data=null——若行到达即刷新，dzuav 真挂了 DZUAV_SILENT 永不触发）；
            # data 持续 null 的起始时刻另记，给 DZUAV_NULL 独立告警用
            if data:
                self.dzuav = data
                self.dzuav_t = t_recv
                self.dzuav_null_since = None
            else:
                self.dzuav = None
                if self.dzuav_null_since is None:
                    self.dzuav_null_since = t_recv

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
                "dzuav_t": self.dzuav_t,
                "dzuav_null_since": self.dzuav_null_since}
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
