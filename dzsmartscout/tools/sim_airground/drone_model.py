# -*- coding: utf-8 -*-
"""无人机天眼模型：U 形航线（与小车同向）+ K 列直飞侦察腿 + 2.5 格感知半径。

## 航线设计（用户四点要求 + 队长补充 B + 本轮参数 ①③⑤）

正式仿真参数（队长 2026-10 最新口径，**取代旧假设**）：
- 高度 1.5 m、感知半径 2.5 格（0.875 m）、飞行速度 **0.5 m/s**；
- 桥洞开口**每场随机**：入口 K7~K11 随机一格、出口 K1~K6 随机一格（默认格 K10/K5 只是
  "范围合法"兜底，盲冲命中概率 1/30）→ 侦察从"校验"升级为"必需"；
- 桥顶箭头 = 画了箭头的垫片（与地雷同物理形态），YOLO 口径 → **不再要求贴桥直飞**，
  进入揭示圆即读到（旧"正下方 2.0~2.5 m"口径被⑤取代）。

航点序列（49 点，全在格心连线上，总长 ≈ 17.15 m，0.5 m/s ≈ 34.3 s）：

    N8 → M8 → L8 → K8                                   （起飞，向东区→K 列，与小车下半场同向）
    K8 → K9 → K10 → K11                                 （K 列侦察腿·南向：3.5 s 内揭示 K9/K10/K11）
    K11 → K10 → K9 → K8 → K7 → K6 → K5 → K4 → K3 → K2 → K1（K 列侦察腿·北向全列：8.4 s 揭示 K5、11.2 s 揭示 K1）
    K1 → K2 → K3 → K4 → K5 → K6 → K7 → K8 → K9 → J9 → I9 → H9 → G9 → F9 → E9 → D9 → C9 → B9 → A9（下半场西向腿，行 9）
    A9 → A8 → A7 → A6 → A5 → A4 → A3                     （西侧北向腿，沿 A 列）
    A3 → B3 → C3 → D3 → E3 → F3 → G3 → H3 → I3 → J3 → K3 → L3 → M3 → M2（上半场东向腿，行 3 → 降落 M2）

设计依据（逐条）：
1. U 形开口朝东、与小车行进同向（下半场西向、上半场东向），不受雷/障/桥洞约束（飞行）。
2. 侦察腿**南向先行**：入口格是 K7~K11 随机，其中 K11 离 K8 最远（1.05 m，超出 2.5 格
   半径）——南向先行把 K11 的揭示时刻从 ~12 s 提前到 **4.2 s**，直接压缩"车到桥前等待
   新鲜情报"的最坏等待；出口格 K1~K6 由北向全列腿在 ≤11.2 s 内全部揭示。
3. 下半场腿取**行 9**：半径 2.5 格覆盖行 7~11 → 下半场全部关键格（雷 D8/F9/I10/E11、
   障 B7/C9/E10/G11、靶 B11/C11、入口 K7~K11）——行 10 腿会漏 B7。
4. 上半场腿取**行 3**：覆盖行 1~5 → 上半场全部关键格（雷 B1/G2/H3/A4、障 D1/F4/A5/H5、
   出口 K1~K6；K6 离行 3 有 1.05 m 超出半径，由侦察腿北向段在 7.7 s 揭示）。
5. 桥顶箭头垫片（⑤）：进入揭示圆即被 YOLO 读到方向（理想置信度 1.0），时间 = 该格揭示时刻。

## 感知模型

- 静态要素（雷/障/靶/箭头垫片）：扫过即**永久揭示**。
- 桥洞开口：动态情报——揭示后带新鲜度，车侧 30 s 过期回落（t6 BridgeIntel 口径）；
  无人机降落悬停后按 ``reemit_s`` 周期重发续期。
- 揭示在时间上**连续**采样（航线段上每 0.01 s 一步）：感知帧率 ~30 fps 远高于
  仿真 dt=0.02 s，离散化误差 < 2 cm，可忽略。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .geo import block_to_meters
from .scenarios import DroneParams, Scenario

#: U 形航线（含 K 列侦察腿）航点序列 —— 报告 §U 形航线几何 与本节 docstring 同源。
DRONE_WAYPOINTS: tuple[str, ...] = (
    "N8", "M8", "L8", "K8",
    "K9", "K10", "K11",
    "K10", "K9", "K8", "K7", "K6", "K5", "K4", "K3", "K2", "K1",
    "K2", "K3", "K4", "K5", "K6", "K7", "K8", "K9",
    "J9", "I9", "H9", "G9", "F9", "E9", "D9", "C9", "B9", "A9",
    "A8", "A7", "A6", "A5", "A4", "A3",
    "B3", "C3", "D3", "E3", "F3", "G3", "H3", "I3", "J3", "K3", "L3", "M3", "M2",
)


@dataclass
class RevealEvent:
    """一次揭示事件（静态永久 / 动态带新鲜度）。"""

    t: float            # 揭示时刻（航线时间，不含起飞延迟与链路延迟）
    cell: str
    kind: str           # "mine" | "obstacle" | "target" | "bridge"


@dataclass
class ArrowRead:
    """一次桥顶箭头垫片读取（⑤：YOLO 口径，揭示即读，不再要求贴桥直飞）。"""

    t: float            # = 垫片格揭示时刻
    cell: str           # 桥洞格（垫片贴在哪个桥洞上）
    direction: str      # 真值方向（E/W）
    confidence: float


@dataclass
class DroneTimeline:
    """无人机整段飞行的揭示/箭头/报文时间线（纯确定性）。"""

    params: DroneParams
    waypoints: tuple[str, ...] = DRONE_WAYPOINTS
    total_len_m: float = 0.0
    flight_end_t: float = 0.0
    reveals: list[RevealEvent] = field(default_factory=list)
    arrow_reads: list[ArrowRead] = field(default_factory=list)
    reveal_time: dict[str, float] = field(default_factory=dict)   # 格名 → 首揭示时刻
    #: 报文发射表：(send_t, payload dict)。发射 = 揭示事件的首次报告 + 心跳 + 周期性重发。
    messages: list[tuple[float, dict]] = field(default_factory=list)

    def position(self, t: float) -> tuple[float, float] | None:
        """航线时间 t 的无人机位置（米）；t<0 → None；已降落且不悬停 → None。"""
        if t < 0.0:
            return None
        if not self.params.hover_after_land and t > self.flight_end_t:
            return None
        pts = self._pts
        if t >= self.flight_end_t:
            return pts[-1]                      # 悬停 M2
        import bisect
        arc = min(t * self.params.speed_mps, self._cum[-1])   # ★ 时间 → 弧长（速度档位）
        i = bisect.bisect_right(self._cum, arc) - 1
        if i < 0:
            return pts[0]
        if i >= len(pts) - 1:
            return pts[-1]
        s0, s1 = self._cum[i], self._cum[i + 1]
        frac = 0.0 if s1 <= s0 else (arc - s0) / (s1 - s0)
        (x0, y0), (x1, y1) = pts[i], pts[i + 1]
        return x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac

    @property
    def _pts(self) -> list[tuple[float, float]]:
        return [block_to_meters(b) for b in self.waypoints]

    @property
    def _cum(self) -> list[float]:
        pts = self._pts
        cum = [0.0]
        for a, b in zip(pts, pts[1:]):
            cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        return cum


def build_timeline(scenario: Scenario) -> DroneTimeline:
    """由情景参数构造无人机时间线（揭示/箭头；报文在 ``emit_messages``）。"""
    from .geo import block_to_meters
    p = scenario.drone
    tl = DroneTimeline(params=p)
    tl.total_len_m = tl._cum[-1]
    tl.flight_end_t = tl.total_len_m / p.speed_mps

    key_cells = set(scenario.truth.mines) | set(scenario.truth.obstacles) \
        | set(scenario.truth.targets) | set(scenario.truth.bridges)
    tl._key_cells = sorted(key_cells)

    radius_m = p.reveal_radius_cells * 0.35
    t = 0.0
    dt = 0.01
    while t <= tl.flight_end_t + 1e-9:
        pos = tl.position(t)
        if pos is None:
            break
        px, py = pos
        for cell in tl._key_cells:
            if cell in tl.reveal_time:
                continue
            cx, cy = block_to_meters(cell)
            if math.hypot(px - cx, py - cy) <= radius_m + 1e-9:
                tl.reveal_time[cell] = round(t, 4)
        t += dt
    for cell in sorted(tl.reveal_time):
        tl.reveals.append(RevealEvent(tl.reveal_time[cell], cell, _cell_kind(cell, scenario)))
    tl.reveals.sort(key=lambda e: e.t)
    tl.unrevealed = sorted(set(key_cells) - set(tl.reveal_time))

    # 箭头垫片（⑤）：进入揭示圆即读到方向（YOLO 理想置信度 1.0）
    for cell, direction in scenario.arrows.items():
        if cell in tl.reveal_time:
            tl.arrow_reads.append(ArrowRead(tl.reveal_time[cell], cell, direction, 1.0))
    tl.arrow_reads.sort(key=lambda r: r.t)
    return tl


def _cell_kind(cell: str, scenario: Scenario) -> str:
    if cell in scenario.truth.mines:
        return "mine"
    if cell in scenario.truth.obstacles:
        return "obstacle"
    if cell in scenario.truth.targets:
        return "target"
    if cell in scenario.truth.bridges:
        return "bridge"
    return "?"


def emit_messages(scenario: Scenario, timeline: DroneTimeline, horizon_s: float) -> list[tuple[float, dict]]:
    """报文发射表（航线时间）：揭示事件首次报告 + 心跳 1 Hz + 全量重发每 ``reemit_s``。

    ★ 载荷逐条按 t2 协议构造（``dzsmartscout/tools/perception/protocol.py`` 只读 import）：
    - ``bridge_passable``：half=lower/upper、block、confidence（**这是车侧采纳门的唯一权威**）；
    - ``target_zone``：targets 全部揭示后发一条（空表会被车侧记 missing_fields ⇒ 不发空表）；
    - ``field_verify``：雷/障分组（揭示一批发一批）；
    - ``heartbeat``：1 Hz（车侧 3 s 超时）。
    - 桥顶箭头（⑤）：现协议无此类型（未知类型被车侧忽略）→ **只进仿真台账，不发车**（改进建议出处）。
    """
    from perception import protocol as _protocol
    build_bridge_passable = _protocol.build_bridge_passable
    build_field_verify = _protocol.build_field_verify
    build_heartbeat = _protocol.build_heartbeat
    build_target_zone = _protocol.build_target_zone
    p = scenario.drone
    msgs: list[tuple[float, dict]] = []
    seq = 0

    mines = sorted({e.cell for e in timeline.reveals if e.kind == "mine"})
    obstacles = sorted({e.cell for e in timeline.reveals if e.kind == "obstacle"})
    targets = sorted({e.cell for e in timeline.reveals if e.kind == "target"})
    if mines:
        seq += 1
        msgs.append((timeline.reveal_time[mines[-1]],
                     build_field_verify(mines=mines, seq=seq, ts=timeline.reveal_time[mines[-1]])))
    if obstacles:
        seq += 1
        msgs.append((timeline.reveal_time[obstacles[-1]],
                     build_field_verify(obstacles=obstacles, seq=seq,
                                        ts=timeline.reveal_time[obstacles[-1]])))
    if targets:
        seq += 1
        msgs.append((timeline.reveal_time[targets[-1]],
                     build_target_zone(targets, seq=seq, ts=timeline.reveal_time[targets[-1]])))

    first_bridge: dict[str, float] = {}
    for e in timeline.reveals:
        if e.kind != "bridge":
            continue
        half = bridge_half(e.cell, scenario)
        if half is None or half in first_bridge:
            continue
        first_bridge[half] = e.t
        seq += 1
        msgs.append((e.t, build_bridge_passable(half, e.cell, confidence=1.0,
                                                ts=e.t, seq=seq)))
    by_half = {}
    for cell in scenario.truth.bridges:
        half = bridge_half(cell, scenario)
        if half:
            by_half[half] = cell

    hb_period = 1.0 / p.heartbeat_hz
    t = 0.0
    while t <= horizon_s:
        seq += 1
        msgs.append((t, build_heartbeat(ts=t, seq=seq)))
        t += hb_period

    t = p.reemit_s
    while t <= horizon_s:
        for half, first_t in first_bridge.items():
            if first_t > t:
                continue
            seq += 1
            msgs.append((t, build_bridge_passable(half, by_half[half], confidence=1.0,
                                                  ts=t, seq=seq)))
        if mines:
            seq += 1
            msgs.append((t, build_field_verify(mines=mines, seq=seq, ts=t)))
        if obstacles:
            seq += 1
            msgs.append((t, build_field_verify(obstacles=obstacles, seq=seq, ts=t)))
        if targets:
            seq += 1
            msgs.append((t, build_target_zone(targets, seq=seq, ts=t)))
        t += p.reemit_s
    msgs.sort(key=lambda m: m[0])
    timeline.messages = msgs
    return msgs


def bridge_half(cell: str, scenario: Scenario) -> str | None:
    """桥洞格 → 半场名（行 7~11=lower / 1~6=upper，与车侧 CROSS_LOWER/UPPER_ROWS 同口径）。"""
    from .geo import block_to_index
    try:
        row_idx, _ = block_to_index(cell)
    except ValueError:
        return None
    row = row_idx + 1
    if 7 <= row <= 11:
        return "lower"
    if 1 <= row <= 6:
        return "upper"
    return None
