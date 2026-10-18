# -*- coding: utf-8 -*-
r"""布局下发节奏（步进循环协议 v2 §4.4 / §4.5）：变化即发 + 每秒心跳重发。

**不直接发 DDS**：本模块只给"什么时候该发哪一份布局"的纯计算/无 I/O 状态机，
由 T29 仿真台或机侧发送器决定走 ROS/DDS 还是写 JSONL。

两条硬语义（协议原文）：
1. 布局**有变化就发**（version 递增）；
2. 即使没有变化，也**每秒重发最新版**（version/segment_id 不变），
   防止"2 分钟停发"类问题；车端按版本去重。

`heartbeat_schedule()` 是纯函数：输入变化时刻与截止时刻，输出 `[(t, version_index)]`；
`LayoutFeed` 是同语义的有状态便利类（`push(layout)` + `poll(now)`）。
"""

from __future__ import annotations

import json
from typing import Any, Iterable

#: 默认心跳周期（协议 §4.4："每秒重发最新版"）
DEFAULT_HEARTBEAT_S = 1.0
_EPS = 1e-9


def heartbeat_schedule(change_times: Iterable[float], t_end: float, *,
                       interval_s: float = DEFAULT_HEARTBEAT_S) -> list[tuple[float, int]]:
    """纯函数：变化时刻序列 -> `[(发送时刻, 变化序号)]`。

    * 每个变化时刻**立即发**一次；
    * 变化之间每 `interval_s` 秒重发**最新**那一版（同序号）；
    * 同一时刻只保留一条（变化优先于恰好撞上的周期重发）；
    * 只输出 `t <= t_end` 的条目。
    """
    if interval_s <= 0:
        raise ValueError("interval_s 必须 > 0")
    times = sorted({round(float(t), 6) for t in change_times})
    times = [t for t in times if t <= t_end + _EPS]
    out: list[tuple[float, int]] = []
    for i, ct in enumerate(times):
        out.append((ct, i))
        next_ct = times[i + 1] if i + 1 < len(times) else None
        t = ct + interval_s
        while t <= t_end + _EPS and (next_ct is None or t < next_ct - _EPS):
            out.append((round(t, 6), i))
            t += interval_s
    seen: set[float] = set()
    deduped: list[tuple[float, int]] = []
    for t, idx in out:
        if t in seen:
            continue
        seen.add(t)
        deduped.append((t, idx))
    return deduped


def heartbeat_messages(layouts: Iterable[dict], *, t_end: float | None = None,
                       interval_s: float = DEFAULT_HEARTBEAT_S) -> list[tuple[float, dict]]:
    """纯函数：布局序列 -> `[(发送时刻, 布局dict)]`（变化 + 周期重发）。

    `t_end` 缺省 = 最后一份布局的 `ts`（不补发到布局之后）。
    """
    ordered = sorted((dict(x) for x in layouts), key=lambda x: float(x.get("ts") or 0.0))
    if not ordered:
        return []
    changes = [float(x.get("ts") or 0.0) for x in ordered]
    end = changes[-1] if t_end is None else float(t_end)
    return [(t, ordered[idx]) for t, idx in
            heartbeat_schedule(changes, end, interval_s=interval_s)]


def to_json(layout: dict) -> str:
    """规范 JSON（字段顺序稳定、无空格；与机侧发送器同口径）。"""
    return json.dumps(layout, ensure_ascii=False, separators=(",", ":"))


class LayoutFeed:
    """"变化即发 + 每秒重发"的无 I/O 状态机（T29 可直接喂虚拟时钟）。

    `poll(now)` 返回要发的布局 dict；无内容可发返回 `None`。
    `push(layout)` 只接受**更新版本**（旧版本被忽略，返回 False）。
    """

    def __init__(self, *, interval_s: float = DEFAULT_HEARTBEAT_S) -> None:
        self.interval_s = float(interval_s)
        self.layout: dict | None = None
        self.version = -1
        self.last_sent: float | None = None
        self.sent_version = -1
        self.sent_count = 0

    def push(self, layout: dict) -> bool:
        version = int(layout.get("version") or 0)
        if self.layout is not None and version <= self.version:
            return False
        self.layout = dict(layout)
        self.version = version
        return True

    def poll(self, now: float) -> dict | None:
        if self.layout is None:
            return None
        changed = self.version != self.sent_version
        due = self.last_sent is None or (now - self.last_sent) >= self.interval_s - _EPS
        if not changed and not due:
            return None
        self.last_sent = float(now)
        self.sent_version = self.version
        self.sent_count += 1
        return dict(self.layout)


__all__ = ["DEFAULT_HEARTBEAT_S", "heartbeat_schedule", "heartbeat_messages",
           "to_json", "LayoutFeed"]
