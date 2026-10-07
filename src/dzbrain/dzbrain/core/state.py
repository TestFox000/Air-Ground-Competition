# -*- coding: utf-8 -*-
"""跨任务共享的状态类型（全部 frozen，可安全放进 Blackboard）。

为什么集中定义：Tracker / Mission / Aim / Safety 之间共享的每一个状态，
如果各写一份，字段名与语义就会漂移（旧代码 90 个状态字段平铺在 `__init__` 里就是这个下场）。
"""

from __future__ import annotations

from dataclasses import dataclass, field


# --------------------------------------------------------------------------- #
#  安全
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class SafetyState:
    """安全状态。任一任务读它决定"能不能动"。"""

    ok: bool = True
    reasons: tuple[str, ...] = ()
    since: float = 0.0
    stops: int = 0                  # 累计强制停车次数（遥测用）

    def with_reason(self, reason: str, ts: float, *, ok: bool = False) -> "SafetyState":
        reasons = tuple(dict.fromkeys(self.reasons + (reason,)))
        return SafetyState(ok=ok, reasons=reasons, since=ts, stops=self.stops + (0 if ok else 1))


# --------------------------------------------------------------------------- #
#  停站点（射击 / 扫描）
# --------------------------------------------------------------------------- #
class StopKind:
    SHOOT = "SHOOT"
    SCAN = "SCAN"


class StopPhase:
    SETTLE = "SETTLE"       # 等底盘停稳
    ACQUIRE = "ACQUIRE"     # 等云台锁定目标
    FIRE = "FIRE"           # 开火窗口
    DONE = "DONE"           # 本点结束，放行


@dataclass(frozen=True)
class StopPointState:
    """当前停站点的上下文（Mission 写，Tracker/Aim/遥测读）。"""

    index: int = -1                    # 航点索引
    block: str = ""
    kind: str = StopKind.SHOOT
    phase: str = StopPhase.SETTLE
    group: str | None = None           # SHOOT@ 候选链组号
    started_at: float = 0.0
    phase_started_at: float = 0.0
    shots_at_arrival: int = 0          # 到站时累计发数快照（判断本点打了几发）
    shots_needed: int = 3
    detail: str = ""                   # 最近一次判定原因（日志/遥测）

    @property
    def active(self) -> bool:
        return self.index >= 0 and self.phase != StopPhase.DONE


# --------------------------------------------------------------------------- #
#  目标与云台
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class TargetState:
    """移动靶观测与估计（AimTask 写，Mission/遥测读）。"""

    seen: bool = False
    dx: int = 0
    dy: int = 0
    cls: int = 0
    box_w: int = 0
    seen_at: float = 0.0
    lost_since: float = 0.0
    # 轨道运动估计（1-D：s 为轨道归一化位置 0..1，v 为 px/s 或 m/s 由实现约定）
    s_rail: float | None = None
    v_rail: float = 0.0
    samples: int = 0
    notes: str = ""


@dataclass(frozen=True)
class AimState:
    """云台与开火状态（AimTask 写）。"""

    status: int = 0                 # contract.AimStatus
    fire_gate: bool = False
    lead_px: float = 0.0
    shots_fired: int = 0            # 本任务发出的开火许可次数
    hits: int = 0                   # 裁判确认命中数
    ammo: int | None = None
    cooled_until: float = 0.0
    last_reason: str = ""


# --------------------------------------------------------------------------- #
#  无人机
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class DroneState:
    online: bool = False
    last_heartbeat: float = 0.0
    bridge_half: str | None = None      # "lower" | "upper"
    bridge_block: str | None = None     # 例 "K9"
    bridge_confidence: float = 0.0
    bridge_ts: float = 0.0
    field_diff: tuple[str, ...] = ()    # 靶区/雷/障核对差异
    messages: int = 0
    last_type: str = ""

    def bridge_of(self, half: str, *, now: float, stale_s: float) -> str | None:
        """取某半场的可通行桥洞格；过期或半场不匹配则 None。"""
        if self.bridge_half != half or self.bridge_block is None:
            return None
        if now - self.bridge_ts > stale_s:
            return None
        return self.bridge_block


# --------------------------------------------------------------------------- #
#  误差修正
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class FixState:
    """误差修正量（P3 的 FixTask 写，LocalizationTask 应用）。

    P1 阶段恒为全 0（无修正），但接口先固定，避免 P3 改架构。
    """

    dx: float = 0.0
    dy: float = 0.0
    dyaw: float = 0.0
    scale: float = 1.0              # 里程比例修正（1.0 = 不改）
    anchor_count: int = 0
    last_anchor: str = ""
    drift_m: float = 0.0            # 最近一段的漂移量
    max_drift_m: float = 0.0

    def corrected(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        from ..core import geometry as geo
        return x + self.dx, y + self.dy, geo.normalize_angle(yaw + self.dyaw)


# --------------------------------------------------------------------------- #
#  遥测（一页纸状态，供网页/日志/复盘）
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Telemetry:
    phase: str = ""
    block: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw_deg: float = 0.0
    speed: float = 0.0
    steer_deg: float = 0.0
    seg_index: int = -1
    s: float = 0.0
    route_len: int = 0
    aim: str = "IDLE"
    target: str = "none"
    shots: int = 0
    hits: int = 0
    ammo: int | None = None
    safety_ok: bool = True
    drone: str = "offline"
    extra: dict = field(default_factory=dict)
