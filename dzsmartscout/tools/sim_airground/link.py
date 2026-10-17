# -*- coding: utf-8 -*-
"""dzlink（DDS）通信代价注入：0.06 s 量级延迟 + 抖动 + 可配丢包（队长补充 C）。

- 延迟队列思路沿用 ``dzbrain.tools.sim`` 的 ``cmd_latency_s`` 队列（队列空时语义不同：
  情报链路队列空 = 无新情报，**不重放**旧情报）。
- 同发布者 FIFO：BEST_EFFORT 下 ROS2 单发布者保序，乱序只在多发布者间；本仿真单发布者
  （无人机）→ **不建模乱序**，抖动只改变投递时刻（如实记录）。
- 丢包：每报文独立同分布 Bernoulli(drop_p)，种子固定 → 可复现。
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from .scenarios import LinkParams


@dataclass
class DeliveredMessage:
    send_t: float        # 航线发送时刻
    recv_t: float        # 车侧接收时刻（joint 时间 = send + takeoff_delay + 链路延迟）
    payload: dict


@dataclass
class LinkStats:
    sent: int = 0
    delivered: int = 0
    dropped: int = 0
    delay_sum: float = 0.0


class Link:
    """确定性链路：``apply(messages, takeoff_delay)`` → 投递表（按接收时刻排序）。"""

    def __init__(self, params: LinkParams):
        self.params = params
        self._rng = random.Random(params.seed)

    def apply(self, messages: list[tuple[float, dict]],
              takeoff_delay_s: float = 0.0) -> tuple[list[DeliveredMessage], LinkStats]:
        stats = LinkStats()
        out: list[DeliveredMessage] = []
        for send_t, payload in messages:
            stats.sent += 1
            if self._rng.random() < self.params.drop_p:
                stats.dropped += 1
                continue
            delay = self.params.latency_s + self._rng.uniform(0.0, self.params.jitter_s)
            recv_t = send_t + takeoff_delay_s + delay
            out.append(DeliveredMessage(send_t, recv_t, payload))
            stats.delivered += 1
            stats.delay_sum += delay
        out.sort(key=lambda m: m.recv_t)
        return out, stats
