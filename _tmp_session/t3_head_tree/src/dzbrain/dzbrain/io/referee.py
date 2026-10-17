# -*- coding: utf-8 -*-
"""裁判系统回传帧解析（命中 / 挨打 / 弹药）—— 纯逻辑，无 ROS 依赖，可离线测。

===============================================================================
★ 为什么会有这个模块：一条"声明了但没人生产"的闭环
===============================================================================
`io/contract.py` 原先声明订阅 `/referee/hit`、`/referee/ammo`；
`docs/新版架构设计.md` §五 计划由 `dzjudgment` 侧解析 0x07 帧后发布它们。
**但车端从未实现这两个话题**（全仓库检索：除 dzbrain 自己外没有任何生产者）。
于是 `AimTask` 的"命中即停火"与"弹药对账"在真车上恒为空转 —— 典型的静默失败：
`take_hits()` 永远返回 0，看代码却一切正常。

★ 处置：直接消费车端**已经在发**的真实话题 `/HpAndHitmsg`
（`src/dzjudgment/src/judgment.cpp:193-203` `publish_referee_state()` 发布；
`case 0x07` 收帧即发 + 1 Hz 心跳），在纯 Python 侧解析。三个好处：
  · 不改任何 C++ 硬件包、不需要车端重编译；
  · 离线可测 —— 本模块喂字节就能测，不需要 ROS、不需要车；
  · 真源唯一 —— 车端发什么我们就解析什么，不再依赖"计划中的话题"。

===============================================================================
★ 帧格式（真源：judgment.cpp:195-201 的 push_back 顺序 + judgment.h:87-91 初值）
===============================================================================
| 下标 | 字段           | 语义                                          | 初值 |
|------|----------------|-----------------------------------------------|------|
| [0]  | targethp       | **对方血量** —— "我们打中靶"的唯一真信号       | 0    |
| [1]  | self_hp        | 己方血量 —— "我们挨打"                         | 0    |
| [2]  | Bing_hit       | 被击打方位 0x01左 0x02前 0x03右 0x04后 0x10未被 | 0x10 |
| [3]  | Shooting_Count | 硬件侧累计射击次数                             | 0    |
| [4]  | ammo           | 剩余弹药                                       | 10   |

★★ 两处**必须记牢的语义纠正**（旧文档写错了，直接照抄会打飞）：
 1. **命中判据是 `targethp` 下降，不是 `Bing_hit`**。`Bing_hit` 是"我方被谁打了"的
    方位（见 judgment.h:89 注释）；拿它当命中确认，会把"我们挨打"记成"我们打中"，
    于是**每挨一发就停火一次**，而真正打中靶时反而毫无反应。
 2. **心跳与实帧的值相同**。`Shooting_Count`/`ammo` 不变时，1 Hz 心跳会重复推完全
    相同的 5 个字节。因此计数**必须基于状态变化**（HP 下降量），绝不能按"收到帧"
    计数 —— 否则心跳会把命中数刷成天文数字（10 发真命中 × 心跳 = 每小时 3600 次）。

★ 判据与边界：
  · 帧长 < 5 → 记 `malformed` 并**丢弃**，绝不猜字段（对应 C++ 侧 `dataLength < 6`
    的越界守卫，judgment.cpp:330-339；那边防的是读越界栈内存，这边防的是记账污染）；
  · 首帧只建立基线、不计命中（否则开机时 targethp 会被当成"从 0 掉到 X"）；
  · `targethp` **下降 N** → 记 N 次命中；
  · `targethp` **上升** → 新一局/复位：重建基线、不计命中、`resets += 1`；
  · `self_hp` 下降 → `self_hits += 1`；`Bing_hit` 只**记录方位**，不参与计数
    （否则"挨一发"会被 self_hp 与 Bing_hit 各记一次 = 双计）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

#: 期望帧长（judgment.cpp 固定 push 5 个字节）
FRAME_LEN = 5

#: 字节下标（与 judgment.cpp:197-201 的注释逐条对应）
IDX_TARGET_HP = 0
IDX_SELF_HP = 1
IDX_BING_HIT = 2
IDX_SHOTS_HW = 3
IDX_AMMO = 4

#: `Bing_hit` 的"未被击打"取值（judgment.h:89）
BING_NOT_HIT = 0x10

#: 方位码 → 中文（日志/遥测可读性）
BING_NAMES: dict[int, str] = {
    0x01: "左", 0x02: "前", 0x03: "右", 0x04: "后", BING_NOT_HIT: "未被击打",
}


@dataclass(frozen=True)
class RefereeFrame:
    """一帧解析结果（不可变，可安全跨线程传）。"""

    targethp: int
    self_hp: int
    bing_hit: int
    shots_hw: int
    ammo: int
    at: float

    @property
    def bing_name(self) -> str:
        return BING_NAMES.get(self.bing_hit, f"0x{self.bing_hit:02X}")

    @property
    def being_hit(self) -> bool:
        """是否正被击打（方位码不是"未被击打"）。"""
        return self.bing_hit != BING_NOT_HIT


@dataclass(frozen=True)
class RefereeDelta:
    """相对上一帧的**变化量** —— 只有这里的东西才允许进账本。"""

    hits: int = 0        # 打中靶次数（= targethp 下降量）
    self_hits: int = 0   # 我方挨打次数（= self_hp 下降量）
    reset: bool = False  # 新一局/复位（targethp 上升）
    frame: RefereeFrame | None = None

    def __bool__(self) -> bool:
        return bool(self.hits or self.self_hits or self.reset)


class RefereeTracker:
    """把 `/HpAndHitmsg` 字节流变成"命中 / 挨打 / 弹药"事件。

    线程模型：`feed()` 由 ROS executor 线程调用；`state`/`ammo`/`hits_total` 等只读
    属性由任务线程读取。写只发生在 `feed()` 内、且只在方法返回前落定，GIL 下单个属性
    读取不会读到半个对象 —— 与 `ros_io` 里其它缓存同级的约定（真正的复合读取走
    `RosBridge._lock`）。
    """

    def __init__(self) -> None:
        self.frames = 0            # 合法帧计数
        self.malformed = 0         # 长度不足/非法帧计数
        self.resets = 0            # targethp 上升次数（新一局/复位）
        self.hits_total = 0        # 累计命中
        self.self_hits_total = 0   # 累计挨打
        self._last: RefereeFrame | None = None
        self.last_seen_at = 0.0

    # ------------------------------------------------------------------ #
    #  只读
    # ------------------------------------------------------------------ #
    @property
    def state(self) -> RefereeFrame | None:
        return self._last

    @property
    def valid_seen(self) -> bool:
        """是否至少收到过一帧合法帧。

        `ros_io` 用它决定是否退回到"非真源"的 `/referee/hit`、`/referee/ammo`
        Int32 话题（那两个话题车端当前无生产者，只作未来桥接的兜底）。
        """
        return self._last is not None

    @property
    def ammo(self) -> int | None:
        """剩余弹药（未收到帧时为 None = **未知**，绝不假设成某个数）。"""
        return None if self._last is None else self._last.ammo

    def age(self, now: float) -> float:
        """距最后一帧的秒数（inf = 从未收到）。"""
        return float("inf") if not self.last_seen_at else float(now) - self.last_seen_at

    # ------------------------------------------------------------------ #
    #  主入口
    # ------------------------------------------------------------------ #
    def feed(self, data: Sequence[int], now: float) -> RefereeDelta:
        """喂一帧 `HpAndHitmsg.data`；返回相对上一帧的变化量。

        非法帧返回**空 delta**（`frame=None`）并累加 `malformed` —— 宁可少记一笔，
        也不拿残缺字节去污染命中账本。
        """
        if data is None or len(data) < FRAME_LEN:
            self.malformed += 1
            return RefereeDelta()
        try:
            vals = [int(v) & 0xFF for v in list(data)[:FRAME_LEN]]
        except (TypeError, ValueError):
            self.malformed += 1
            return RefereeDelta()

        frame = RefereeFrame(
            targethp=vals[IDX_TARGET_HP],
            self_hp=vals[IDX_SELF_HP],
            bing_hit=vals[IDX_BING_HIT],
            shots_hw=vals[IDX_SHOTS_HW],
            ammo=vals[IDX_AMMO],
            at=float(now),
        )
        prev = self._last
        self._last = frame
        self.last_seen_at = float(now)
        self.frames += 1

        if prev is None:                     # 首帧：只建基线，不计命中
            return RefereeDelta(frame=frame)

        hits = max(0, prev.targethp - frame.targethp)        # 下降 = 我们打中靶
        self_hits = max(0, prev.self_hp - frame.self_hp)     # 下降 = 我们挨打
        reset = frame.targethp > prev.targethp               # 上升 = 新一局/复位
        if reset:
            self.resets += 1
        self.hits_total += hits
        self.self_hits_total += self_hits
        return RefereeDelta(hits=hits, self_hits=self_hits, reset=reset, frame=frame)

    # ------------------------------------------------------------------ #
    #  排障
    # ------------------------------------------------------------------ #
    def describe(self) -> str:
        st = self._last
        if st is None:
            return (f"裁判回传：无合法帧（frames={self.frames} malformed={self.malformed}）"
                    f"—— 裁判系统未开启/未连接？")
        return (f"裁判回传：对方HP {st.targethp} / 我方HP {st.self_hp} / "
                f"方位 {st.bing_name} / 硬件发数 {st.shots_hw} / 弹药 {st.ammo} "
                f"（命中 {self.hits_total} 挨打 {self.self_hits_total} "
                f"复位 {self.resets} 坏帧 {self.malformed}）")
