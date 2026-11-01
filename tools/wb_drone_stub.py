# -*- coding: utf-8 -*-
"""WB-3 无人机**步进 stub**（卡 `WB-3-INCIDENTS-20261031`；用户裁决 9 的收窄形态）。

## 它是什么 / 不是什么

| 是 | 不是 |
|---|---|
| 一个**按段吐 `field_layout` 报文**的脚本化发报机 | 天眼模型 / 侦察精度 / 覆盖率优化 |
| 能配三种节奏（一次给全图 / 逐段给 / 中途断供） | 航线规划、视觉识别、云台对准 |
| 断供时用来验证车端"不无限 WAIT_INFO"（B 区裁决） | 任何飞控 / 链路 / 通讯时延建模 |

## 为什么只做这么点（这条是本卡唯一可能被质疑"做得太少"的地方）

`DroneTask` 消费的全部内容就是**一条 JSON 报文**（`tasks/drone.py::_handle_raw`）：
`{type:"field_layout", version, start, end, known[], mines[], obstacles[], targets[],
 bridges[], barrier_*, target_cell?, segment_id?}`。车端拿到它只做三件事：
校验 → 写 `P_FIELD_LAYOUT` → `main.plan_route_from_field_layout()` 规划。

**天眼精度/覆盖率不进循迹闭环**：它们只影响"无人机多久报一次 known"，
而 known 的**数量**已经被"逐段给 vs 一次给全"这两个节奏参数穷尽了。
再往上建模（扫描轨迹、遮挡、丢检率）是给一个**不影响判据**的量加自由度 ——
台账 D-026 子约束「未校准扫参禁止」正是这条：没有实车标定数据的参数，建了就是拟合噪声。

## 与真机的关系（这条要写清楚，否则会被误读成"仿真了无人机"）

真机 `field_layout` 由无人机地面站/机载程序按扫描覆盖产出；本 stub **不**模拟那个过程，
它只是一条**可复现的报文序列**。所以 `transfer` 分级是"仅结构"：它证明的是
**车端对报文时序的响应**（不无限等、unknown 当可通行、版本单调），不是"无人机能扫多准"。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field

#: 报文 schema 版本（车端 `tasks/drone.py::FIELD_LAYOUT_VERSION` 认 2）
LAYOUT_VERSION = 2

#: 三种节奏
PACING_ALL_AT_ONCE = "all-at-once"   # 开局一次给全图（= 静态场地）
PACING_PER_SEGMENT = "per-segment"   # 每段只给"这一段要走的那几格"
PACING_CUTOFF = "cutoff"             # 给到第 N 段就**断供**（验证车端不无限等）

PACINGS: tuple[str, ...] = (PACING_ALL_AT_ONCE, PACING_PER_SEGMENT, PACING_CUTOFF)


@dataclass
class DroneStub:
    """按段吐 `field_layout` 的发报机。

    ★ 它是**纯函数式**的：`messages_at(t)` 只依赖 `t` 与自身状态，不依赖墙钟/随机数
      ⇒ 同一条时间线跑两次逐位相同（这是"可重放"的前提，与台架同一口径）。

    ★ 版本单调递增：车端 `_on_field_layout` 会**拒收** `version < 当前` 的迟到旧报
      （`layout_stale+=1`）。stub 如果乱发版本号，夹具就会测出"车端拒收了布局"这种
      与案例无关的现象 —— 那不是被控对象，是我的发报机在骗人。
    """

    #: 每段要走的格（按顺序；车端会把这些格并进 known）
    segments: list[list[str]] = field(default_factory=list)
    #: 全图格（`all-at-once` 与 `cutoff` 用；空 = 用 segments 的并集）
    all_cells: list[str] = field(default_factory=list)
    #: 起点 / 终点格
    start: str = "N10"
    end: str = "N5"
    #: 雷 / 障 / 靶
    mines: list[str] = field(default_factory=list)
    obstacles: list[str] = field(default_factory=list)
    targets: list[str] = field(default_factory=list)
    #: 通口格（不可为空表：车端 `_layout_cells(..., allow_empty=False)` 会整份拒收）
    bridges: list[str] = field(default_factory=lambda: ["K5", "K7"])
    barrier_col_from: str = "D"
    barrier_row_hi: int = 6
    barrier_row_lo: int = 7
    #: 节奏
    pacing: str = PACING_PER_SEGMENT
    #: 每段间隔秒（`per-segment` / `cutoff` 用）
    segment_period_s: float = 8.0
    #: `cutoff` 模式下给到第几段就停（从 0 数）
    cutoff_after_segments: int = 1
    #: 目标停站格（可为空串）
    target_cell: str = ""
    #: 已发出几条（诊断）
    emitted: int = 0

    def __post_init__(self) -> None:
        if self.pacing not in PACINGS:
            raise ValueError(f"未登记的节奏 {self.pacing!r}（可选 {list(PACINGS)}）")
        if not self.bridges:
            raise ValueError("bridges 不能为空（车端 allow_empty=False ⇒ 整份拒收）")

    # ------------------------------------------------------------------ #
    #  节奏
    # ------------------------------------------------------------------ #
    def segment_index_at(self, t: float) -> int:
        """`t` 秒时应该已经给到第几段（从 0 数）。"""
        if self.pacing == PACING_ALL_AT_ONCE:
            return len(self.segments) - 1
        if self.segment_period_s <= 0.0:
            return len(self.segments) - 1
        idx = int(t // self.segment_period_s)
        upper = len(self.segments) - 1
        if self.pacing == PACING_CUTOFF:
            upper = min(upper, max(0, int(self.cutoff_after_segments)))
        return max(0, min(idx, upper))

    @property
    def stalled(self) -> bool:
        """`cutoff` 模式给完了吗（给完 = 之后车端应转 WAIT_INFO，不是无限空等）。"""
        return self.pacing == PACING_CUTOFF and len(self.segments) > 0

    def known_at(self, t: float) -> list[str]:
        """`t` 秒时这份报文的 `known`（**只增不减**，与真机"扫描覆盖只增"同语义）。

        ★★ 一条协议硬约束（`tasks/drone.py::normalize_field_layout` 实测）：
          v2 要求 **特征 ⊆ known** —— `mines`/`obstacles`/`targets` 里出现一个
          "还没观测到"的格，车端就**整份拒收**（`layout_rejects+=1`）。
          这不是可以绕过的坑：真机同样是"扫到那一格才敢报它的雷"。
          ⇒ 本 stub 只把**已经 known 的**特征报出去。想让某个雷立刻生效，
            就把它所在的格放进第 1 段（`segments[0]`）。
          第一版 stub 无视这条，四条报文全被车端拒收 —— 而拒收日志措辞
          "layout_rejects" 看起来像是在测"车端 bug"，实际是我的发报机不合协议。
        """
        if not self.segments:
            return list(self.all_cells)
        upto = self.segment_index_at(t) + 1
        cells: list[str] = []
        for seg in self.segments[:upto]:
            for c in seg:
                if c not in cells:
                    cells.append(c)
        return cells

    def _known_subset(self, cells: list[str], known: list[str]) -> list[str]:
        """只留 `known` 里的特征（协议要求 `特征 ⊆ known`，否则整份拒收）。"""
        ok = set(known)
        return [c for c in cells if c.strip().upper() in ok]

    # ------------------------------------------------------------------ #
    #  报文
    # ------------------------------------------------------------------ #
    def layout_at(self, t: float) -> dict:
        """`t` 秒时应当发出的那份 `field_layout`（形状 = `normalize_field_layout` 认的）。"""
        known = self.known_at(t)
        target = self.target_cell if self.target_cell.strip().upper() in set(known) else ""
        return {
            "type": "field_layout",
            "version": LAYOUT_VERSION,
            "ts": round(float(t), 3),
            "start": self.start,
            "end": self.end,
            "known": known,
            "mines": self._known_subset(self.mines, known),
            "obstacles": self._known_subset(self.obstacles, known),
            "targets": self._known_subset(self.targets, known),
            "bridges": list(self.bridges),
            "barrier_col_from": self.barrier_col_from,
            "barrier_row_hi": int(self.barrier_row_hi),
            "barrier_row_lo": int(self.barrier_row_lo),
            "target_cell": target,
            "segment_id": self.segment_index_at(t) + 1,
        }

    def messages_at(self, t: float, *, last_t: float | None = None) -> list[str]:
        """`(last_t, t]` 之间应当发出的报文（JSON 字符串；可直接 `bridge.push_drone()`）。

        ★ `last_t=None` = 首帧：给一条（`version` 从 1 起，之后逐条 +1）。
        """
        idx = self.segment_index_at(t)
        prev = -1 if last_t is None else self.segment_index_at(last_t)
        if prev >= idx and last_t is not None:
            return []                            # 本拍没有新段 ⇒ 不发新报（不做心跳洪泛）
        out: list[str] = []
        for step in range(prev + 1, idx + 1):
            layout = self.layout_at(float(t))
            layout["version"] = LAYOUT_VERSION + step     # 单调递增（拒收更小版本）
            layout["segment_id"] = step + 1
            out.append(json.dumps(layout, ensure_ascii=False, sort_keys=True))
            self.emitted += 1
        return out

    def describe(self) -> str:
        tail = (f" / 断供于第 {self.cutoff_after_segments + 1} 段"
                if self.pacing == PACING_CUTOFF else "")
        return (f"无人机步进 stub：节奏 {self.pacing} / {len(self.segments)} 段 / "
                f"间隔 {self.segment_period_s:g}s / 通口 {self.bridges}{tail}")


# --------------------------------------------------------------------------- #
#  与车端闭环：把 stub 接到 `Bridge.take_drone_messages`
# --------------------------------------------------------------------------- #
def pump(stub: DroneStub, bridge, now: float, *, last_t: float | None = None) -> int:
    """把 `stub` 在本拍应发的报文推进桥的无人机收件箱。返回推了几条。"""
    msgs = stub.messages_at(now, last_t=last_t)
    for msg in msgs:
        bridge.push_drone(msg)
    return len(msgs)


__all__ = ["DroneStub", "pump", "PACINGS", "PACING_ALL_AT_ONCE", "PACING_PER_SEGMENT",
           "PACING_CUTOFF", "LAYOUT_VERSION"]
