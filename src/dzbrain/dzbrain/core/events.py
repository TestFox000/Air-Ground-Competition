# -*- coding: utf-8 -*-
"""事件总线 —— 任务线程之间**唯一**的低频解耦通道。

设计要点（对应架构设计 §3.2）：
- 发布非阻塞：任一任务线程发布事件绝不等待消费者（消费者慢→丢事件并计数，不拖慢控制回路）。
- 每消费者独立队列：一个线程卡住不影响别人。
- 事件不可变：带来源、时间戳、序号，便于日志/复盘。
- 禁止用事件传高频数据（位姿、视觉帧）——那些走 Blackboard。
"""

from __future__ import annotations

import itertools
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable

# --------------------------------------------------------------------------- #
#  事件类型（字符串常量，避免拼写错误扩散）
# --------------------------------------------------------------------------- #
# 任务/路线
EV_MISSION_STARTED = "MISSION_STARTED"
EV_MISSION_FINISHED = "MISSION_FINISHED"
EV_ROUTE_LOADED = "ROUTE_LOADED"
EV_WAYPOINT_REACHED = "WAYPOINT_REACHED"
EV_ARRIVED_STOP_POINT = "ARRIVED_STOP_POINT"          # 到达射击/扫描停站点
EV_STOP_POINT_DONE = "STOP_POINT_DONE"                # 该停站点流程结束，放行
EV_OVERSHOOT_RECOVERED = "OVERSHOOT_RECOVERED"        # 开过头后补触发停站
# 打击
EV_TARGET_FIRST_SEEN = "TARGET_FIRST_SEEN"
EV_TARGET_LOST = "TARGET_LOST"
EV_FIRE_ALLOWED = "FIRE_ALLOWED"
EV_FIRE_BLOCKED = "FIRE_BLOCKED"
EV_HIT_CONFIRMED = "HIT_CONFIRMED"
EV_AMMO_CHANGED = "AMMO_CHANGED"
# 定位/修正
EV_POSE_ANCHORED = "POSE_ANCHORED"                    # 到位吸附
EV_DRIFT_WARNING = "DRIFT_WARNING"
EV_FIX_APPLIED = "FIX_APPLIED"                        # 误差修正量已应用
# 无人机
EV_DRONE_ONLINE = "DRONE_ONLINE"
EV_DRONE_OFFLINE = "DRONE_OFFLINE"
EV_BRIDGE_INFO = "BRIDGE_INFO"                        # 桥洞可通行情报
EV_FIELD_VERIFY = "FIELD_VERIFY"                      # 靶区/雷/障核对结果
# 安全
EV_SAFETY_STOP = "SAFETY_STOP"
EV_SAFETY_CLEAR = "SAFETY_CLEAR"
EV_TASK_FAULT = "TASK_FAULT"                          # 某任务线程异常/心跳丢失


@dataclass(frozen=True)
class Event:
    """不可变事件。"""

    type: str
    source: str = ""
    data: dict = field(default_factory=dict)
    ts: float = field(default_factory=time.time)
    seq: int = 0


class Subscription:
    """订阅句柄：轮询式取事件（任务线程在自己的 tick 里 drain，不引入额外线程）。"""

    def __init__(self, bus: "EventBus", name: str, maxsize: int = 256) -> None:
        self._bus = bus
        self.name = name
        self._q: "queue.Queue[Event]" = queue.Queue(maxsize=maxsize)
        self.dropped = 0

    def poll(self, max_items: int = 32) -> list[Event]:
        """取最多 max_items 条，非阻塞。"""
        out: list[Event] = []
        for _ in range(max_items):
            try:
                out.append(self._q.get_nowait())
            except queue.Empty:
                break
        return out

    def drain(self, handler: Callable[[Event], None], max_items: int = 32) -> int:
        """取事件并逐条回调，返回处理条数。"""
        events = self.poll(max_items)
        for ev in events:
            handler(ev)
        return len(events)

    def _offer(self, ev: Event) -> None:
        try:
            self._q.put_nowait(ev)
        except queue.Full:
            self.dropped += 1

    def unsubscribe(self) -> None:
        self._bus._remove(self)


class EventBus:
    """进程内事件总线（线程安全）。"""

    def __init__(self) -> None:
        self._subs: dict[str, dict[str, Subscription]] = {}
        self._lock = threading.RLock()
        self._seq = itertools.count(1)
        self.published = 0

    def subscribe(self, name: str, types: list[str] | None = None, maxsize: int = 256) -> Subscription:
        """按类型订阅；types=None 或 [] 表示订阅全部。"""
        sub = Subscription(self, name, maxsize=maxsize)
        with self._lock:
            for t in (types or ["*"]):
                self._subs.setdefault(t, {})[name] = sub
        return sub

    def publish(self, ev_type: str, source: str = "", **data: Any) -> Event:
        """发布事件（非阻塞，永不抛异常）。"""
        ev = Event(type=ev_type, source=source, data=dict(data), seq=next(self._seq))
        with self._lock:
            targets = list(self._subs.get(ev_type, {}).values())
            if "*" in self._subs:
                targets.extend(s for s in self._subs["*"].values()
                               if s.name not in {t.name for t in targets})
        self.published += 1
        for sub in targets:
            sub._offer(ev)
        return ev

    def _remove(self, sub: Subscription) -> None:
        with self._lock:
            for bucket in self._subs.values():
                bucket.pop(sub.name, None)

    def stats(self) -> dict:
        with self._lock:
            return {
                "published": self.published,
                "subscribers": {t: list(b) for t, b in self._subs.items()},
                "dropped": {s.name: s.dropped
                            for b in self._subs.values() for s in b.values()},
            }
