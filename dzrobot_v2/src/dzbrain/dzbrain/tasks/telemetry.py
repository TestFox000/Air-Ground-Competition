# -*- coding: utf-8 -*-
r"""TelemetryTask —— 遥测（2 Hz，架构 §3.1 的 T8）。

职责只有两件（**不做任何决策**，也不驱动物理量）：

1. 把散落在 Blackboard 上的一页纸状态汇总成 `core.state.Telemetry`，
   以 `P_TELEMETRY[TELEMETRY_KEY]` 落盘 —— 供网页/地面站/结束横幅/复盘读取；
2. 按 `telemetry.period_s`（默认 0.5 s = 2 Hz）周期打印**一行**紧凑状态，
   便于 `tail -f` 与"车跑完看一眼日志"。

## ★ 为什么不能整键覆盖 `P_TELEMETRY`

`P_TELEMETRY` 是**约定共享**的 dict（`tasks/tracker.py` 的 docstring 第 53 行写明），
里面已经有别人的子键：`tracker_cmd`（TrackerTask）、`mission_state`（MissionTask）。
本任务只做 read-modify-write 自己的两个子键，绝不覆盖整键：
  · `TELEMETRY_KEY`       → `Telemetry` 快照（类型安全，字段固定）
  · `TELEMETRY_LINE_KEY`  → 最近打印过的那一行文本（日志与快照可对拍）
这条纪律也是测试项："遥测任务不写任何其它键"。

## 数据来源（全部只读 `cfg.get` 一次，tick 内不再碰 Config）

| 字段 | 来源 |
|---|---|
| phase / block / x / y / yaw_deg | `P_MISSION_PHASE` / `P_POSE` |
| seg_index / s / speed / steer_deg | `P_ROUTE_INDEX` / `P_ROUTE_S` / `P_TELEMETRY[tracker_cmd]` |
| aim / target / shots / hits / ammo | `P_AIM`（`AimState`）/ `P_TARGET`（`TargetState`） |
| safety_ok | `P_SAFETY`（`SafetyState`） |
| drone | `P_DRONE`（`DroneState`）+ `bridge_of()` 的过期判据 |
| route_len / extra.stop_point / extra.fix_est | `P_ROUTE` / `P_STOP_POINT` / `P_FIX_EST` |

★ 缺键一律"显示为未知"（空串 / 默认值），绝不因为某个任务还没起就崩或停印。

## 铁律

- `tick()` 非阻塞、无 I/O（`log` 是注入的可调用对象，测试里就是收集器）；
- **不 import rclpy**；不直接调 `time.time()/time.monotonic()`（走注入的 `clock`）；
- `render_line()` 是**纯函数**（同一个 Telemetry 永远同一行文本）→ 可单测。

⚠️ SCHEMA 缺键：无（`telemetry.period_s` 已在 `core/config.py` 的 SCHEMA 里）。
"""

from __future__ import annotations

import json
from typing import Any, Callable

from ..core import geometry as geo
from ..core.blackboard import (P_AIM, P_DRONE, P_FIX_EST, P_MISSION_PHASE, P_POSE,  # noqa: F401
                               P_ROUTE, P_ROUTE_INDEX, P_ROUTE_S, P_SAFETY, P_STOP_POINT,
                               P_TARGET, P_TELEMETRY, Blackboard)
from ..core.config import Config
from ..core.events import EventBus
from ..core.logging import Throttle
from ..core.state import AimState, DroneState, FixState, SafetyState, Telemetry
from ..core.task import TaskThread
from ..io.contract import AimStatus
from ..route.model import Route

__all__ = ["TelemetryTask", "TELEMETRY_KEY", "TELEMETRY_LINE_KEY", "render_line",
           "build_telemetry"]

#: `P_TELEMETRY` 内属于本任务的子键（★ 别人的子键一个都不许动）
TELEMETRY_KEY = "telemetry"          # → core.state.Telemetry 快照
TELEMETRY_LINE_KEY = "telemetry_line"  # → str：最近打印的那一行

#: `TrackerTask` 的子键（读它拿速度/舵角；只读不写）
TRACKER_CMD_KEY = "tracker_cmd"


# --------------------------------------------------------------------------- #
#  纯函数：渲染
# --------------------------------------------------------------------------- #
def render_line(tel: Telemetry | None) -> str:
    """一页纸 → **一行**紧凑文本（纯函数：无 I/O、无时钟、无全局状态）。

    形如：`[T] N10 DRIVE (0.525,0.175) yaw0.0 v0.150 st+0.0 | 段4 s1.23/34 | 瞄IDLE 靶none 发0/中0 弹- | 安全OK | 无人机offline(信0)`

    ★ 字段顺序刻意固定为"位置 → 阶段 → 控制 → 进度 → 打击 → 安全 → 链路"，
      便于 `grep`/`awk` 与人工扫读（旧代码日志的痛点是同一信息每次换个位置）。
    """
    if tel is None:
        return "[T] (无遥测)"
    ammo = "-" if tel.ammo is None else str(tel.ammo)
    parts = [
        f"[T] {tel.block or '??'} {tel.phase or 'IDLE'} "
        f"({tel.x:.3f},{tel.y:.3f}) yaw{tel.yaw_deg:.1f} "
        f"v{tel.speed:.3f} st{tel.steer_deg:+.1f}",
        f"段{tel.seg_index} s{tel.s:.2f}/{tel.route_len}",
        f"瞄{tel.aim} 靶{tel.target} 发{tel.shots}/中{tel.hits} 弹{ammo}",
        f"安全{'OK' if tel.safety_ok else 'STOP'}",
        f"无人机{tel.drone}",
    ]
    return " | ".join(parts)


def build_telemetry(bb: Blackboard, *, now: float, drone_stale_s: float = 30.0) -> Telemetry:
    """把 Blackboard 上的散件汇总成 `Telemetry`（纯读取；缺键=未知，不抛异常）。

    与 `TelemetryTask.tick()` 分开放，是为了让"怎么汇总"能在没有任务线程的情况下单测。
    """
    pose = bb.get(P_POSE)
    x = float(pose.x) if isinstance(pose, geo.Pose) else 0.0
    y = float(pose.y) if isinstance(pose, geo.Pose) else 0.0
    yaw_deg = geo.deg(pose.yaw) if isinstance(pose, geo.Pose) else 0.0
    block = pose.block() if isinstance(pose, geo.Pose) else ""

    phase = str(bb.get(P_MISSION_PHASE) or "")

    seg_raw = bb.get(P_ROUTE_INDEX)
    seg_index = int(seg_raw) if isinstance(seg_raw, (int, float)) else -1
    s_raw = bb.get(P_ROUTE_S)
    s = float(s_raw) if isinstance(s_raw, (int, float)) else 0.0

    route = bb.get(P_ROUTE)
    route_len = len(route) if isinstance(route, Route) else 0

    snap = bb.get(P_TELEMETRY)
    cmd = snap.get(TRACKER_CMD_KEY) if isinstance(snap, dict) else None
    speed = 0.0
    steer_deg = 0.0
    if isinstance(cmd, dict):
        speed = _num(cmd.get("speed"))
        steer_deg = _num(cmd.get("steer_deg"))

    aim = bb.get(P_AIM)
    aim_name = AimStatus.name(aim.status) if isinstance(aim, AimState) else "IDLE"
    shots = int(aim.shots_fired) if isinstance(aim, AimState) else 0
    hits = int(aim.hits) if isinstance(aim, AimState) else 0
    ammo = aim.ammo if isinstance(aim, AimState) else None

    target = bb.get(P_TARGET)
    target_text = _target_text(target)

    safety = bb.get(P_SAFETY)
    safety_ok = True if safety is None else (
        bool(safety.ok) if isinstance(safety, SafetyState) else True)

    drone = bb.get(P_DRONE)
    drone_text = _drone_text(drone, now=now, stale_s=drone_stale_s)

    stop = bb.get(P_STOP_POINT)
    fix = bb.get(P_FIX_EST)
    extra: dict[str, Any] = {}
    if stop is not None:
        extra["stop_point"] = {
            "index": int(getattr(stop, "index", -1)),
            "block": str(getattr(stop, "block", "")),
            "kind": str(getattr(stop, "kind", "")),
            "phase": str(getattr(stop, "phase", "")),
        }
    if isinstance(fix, FixState):
        extra["fix_est"] = {"scale": fix.scale, "samples": fix.anchor_count,
                            "max_drift_m": fix.max_drift_m}
    if isinstance(safety, SafetyState) and safety.reasons:
        extra["safety_reasons"] = list(safety.reasons)

    return Telemetry(
        phase=phase, block=block, x=x, y=y, yaw_deg=yaw_deg,
        speed=speed, steer_deg=steer_deg,
        seg_index=seg_index, s=s, route_len=route_len,
        aim=aim_name, target=target_text, shots=shots, hits=hits, ammo=ammo,
        safety_ok=safety_ok, drone=drone_text, extra=extra,
    )


def _num(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _target_text(target: Any) -> str:
    """靶状态一行摘要：`none` / `seen` / `s0.42v-30.0`（有轨道估计时）。"""
    if target is None or not bool(getattr(target, "seen", False)):
        return "none"
    s_rail = getattr(target, "s_rail", None)
    v_rail = _num(getattr(target, "v_rail", 0.0))
    if isinstance(s_rail, (int, float)):
        return f"s{float(s_rail):.2f}v{v_rail:+.1f}"
    return "seen"


def _drone_text(drone: Any, *, now: float, stale_s: float) -> str:
    """`offline` / `online` / `online:lower=K9`（+ `!过期` / `!差异N`）。"""
    if not isinstance(drone, DroneState):
        return "offline"
    if not drone.online:
        return "offline"
    text = "online"
    halves = [h for h in ("lower", "upper")
              if drone.bridge_of(h, now=now, stale_s=stale_s) is not None]
    if halves:
        text += ":" + ",".join(f"{h}={drone.bridge_of(h, now=now, stale_s=stale_s)}"
                               for h in halves)
    elif drone.bridge_block is not None:
        text += "!过期"
    if drone.field_diff:
        text += f"!差异{len(drone.field_diff)}"
    return text


# --------------------------------------------------------------------------- #
#  任务
# --------------------------------------------------------------------------- #
class TelemetryTask(TaskThread):
    """汇总 + 周期打印（2 Hz，只写 `P_TELEMETRY` 的两个子键）。"""

    def __init__(self, bridge, bb: Blackboard, bus: EventBus, cfg: Config, *,
                 log: Callable[[str], None] = print, group=None,
                 clock: Callable[[], float] | None = None) -> None:
        super().__init__("TelemetryTask", 2.0, bus=bus, blackboard=bb, logger=log)
        self.bridge = bridge
        self.cfg = cfg
        self.group = group
        if clock is None:
            import time as _time
            clock = _time.monotonic
        self.clock = clock

        # 配置一次读齐（tick 里不再碰 cfg）
        self.period_s = max(0.05, float(cfg.get("telemetry.period_s") or 0.5))
        self.drone_stale_s = float(cfg.get("drone.info_stale_s") or 30.0)
        self._rate_limit_s = float(cfg.get("logging.rate_limit_s") or 1.0)

        self.ticks = 0
        self.prints = 0
        self.last_line = ""
        self.latest: Telemetry | None = None
        self._next_at: float = 0.0
        self._warn = Throttle(self._rate_limit_s)

    # ------------------------------------------------------------------ #
    #  生命周期
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        self._next_at = self.clock() + self.period_s     # 第一拍不印（避免启动瞬间刷屏）
        self._log(f"遥测：{1.0 / self.period_s:.1f} Hz（{self.period_s:g}s/条）")

    def teardown(self) -> None:
        self._log(f"退出：{self.ticks} 拍 / 打印 {self.prints} 条 / 最后一行：{self.last_line}")

    # ------------------------------------------------------------------ #
    #  单拍
    # ------------------------------------------------------------------ #
    def tick(self) -> None:
        self.ticks += 1
        now = self.clock()
        if now < self._next_at:
            return                                   # ★ 节流：不是每拍都汇总/打印
        self._next_at = now + self.period_s
        try:
            tel = build_telemetry(self.bb, now=now, drone_stale_s=self.drone_stale_s)
        except Exception as exc:                     # noqa: BLE001  汇总失败不许杀线程
            if self._warn.allow("build", now):
                self._log(f"⚠️ 遥测汇总失败（本拍跳过）：{type(exc).__name__}: {exc}")
            return
        self.latest = tel
        line = render_line(tel)
        self.last_line = line
        self._write(tel, line, now)
        self.prints += 1
        self._log(line)

    # ------------------------------------------------------------------ #
    #  只读接口（测试用）
    # ------------------------------------------------------------------ #
    def snapshot(self) -> Telemetry | None:
        """最近一次落盘的快照（与 `P_TELEMETRY[TELEMETRY_KEY]` 相同对象）。"""
        return self.latest

    def json_line(self) -> str:
        """最近一行的 JSON 版（给将来网页/地面站用；纯格式化，无副作用）。"""
        tel = self.latest
        if tel is None:
            return "{}"
        return json.dumps({
            "phase": tel.phase, "block": tel.block, "x": tel.x, "y": tel.y,
            "yaw_deg": tel.yaw_deg, "speed": tel.speed, "steer_deg": tel.steer_deg,
            "seg_index": tel.seg_index, "s": tel.s, "route_len": tel.route_len,
            "aim": tel.aim, "target": tel.target, "shots": tel.shots,
            "hits": tel.hits, "ammo": tel.ammo, "safety_ok": tel.safety_ok,
            "drone": tel.drone, "extra": tel.extra,
        }, ensure_ascii=False, separators=(",", ":"))

    # ------------------------------------------------------------------ #
    #  内部
    # ------------------------------------------------------------------ #
    def _write(self, tel: Telemetry, line: str, now: float) -> None:
        """★ read-modify-write 自己的两个子键，别人的子键原样保留。"""
        snap = self.bb.get(P_TELEMETRY)
        data = dict(snap) if isinstance(snap, dict) else {}
        data[TELEMETRY_KEY] = tel
        data[TELEMETRY_LINE_KEY] = line
        self.bb.set(P_TELEMETRY, data, source=self.name, ts=now)
