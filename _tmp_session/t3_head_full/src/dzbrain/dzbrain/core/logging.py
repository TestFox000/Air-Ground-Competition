# -*- coding: utf-8 -*-
"""日志工具 —— 统一横幅、限流、结构化事件流。

为什么单独一个模块：旧代码的日志散落各处、无统一前缀、高频分支靠人肉节流（readme 里
多处"只报一次"手写标志位）。这里提供三件事：横幅、限流器、事件行（便于复盘）。

注意：车端日志是**主要证据来源**（"绝不静默报成功"），所以：
- 状态迁移一律留痕（进入/退出/原因）
- 限流只压重复项，不吞首次与异常
- 异常一律带上下文（谁、什么条件、数值）
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable

WIDTH = 78


def banner(lines: list[str] | str, char: str = "=", log: Callable[[str], None] = print) -> None:
    """打印带框横幅。"""
    if isinstance(lines, str):
        lines = [lines]
    log(char * WIDTH)
    for line in lines:
        log(line)
    log(char * WIDTH)


def kv_lines(data: dict[str, Any], indent: int = 2) -> list[str]:
    """把 dict 渲染成对齐的多行文本（用于启动横幅）。"""
    if not data:
        return []
    width = max(len(str(k)) for k in data)
    pad = " " * indent
    return [f"{pad}{str(k).ljust(width)} : {v}" for k, v in data.items()]


@dataclass
class Throttle:
    """同类日志限流：同一 key 在 interval 秒内只放行一次（首次必放行）。"""

    interval: float = 1.0
    _last: dict[str, float] = field(default_factory=dict)

    def allow(self, key: str, now: float | None = None) -> bool:
        now = time.time() if now is None else now
        last = self._last.get(key)
        if last is None or (now - last) >= self.interval:
            self._last[key] = now
            return True
        return False

    def reset(self, key: str | None = None) -> None:
        if key is None:
            self._last.clear()
        else:
            self._last.pop(key, None)


class EventLog:
    """结构化事件行：`[EV] ts=... task=... type=... k=v ...`。

    设计目的：复盘时用 grep 就能拉出时间线，不依赖人工描述的日志措辞。
    """

    def __init__(self, log: Callable[[str], None] = print) -> None:
        self._log = log
        self.records: list[str] = []

    def write(self, task: str, ev_type: str, **fields: Any) -> str:
        parts = [f"[EV] ts={time.time():.3f}", f"task={task}", f"type={ev_type}"]
        for k, v in fields.items():
            parts.append(f"{k}={v}")
        line = " ".join(parts)
        self.records.append(line)
        self._log(line)
        return line


class FaultGuard:
    """一次性日志/动作守卫（替代散落各处的 `logged_xxx` 标志位）。"""

    def __init__(self) -> None:
        self._fired: set[str] = set()

    def first(self, key: str) -> bool:
        """首次调用返回 True，之后 False。"""
        if key in self._fired:
            return False
        self._fired.add(key)
        return True

    def clear(self, key: str | None = None) -> None:
        if key is None:
            self._fired.clear()
        else:
            self._fired.discard(key)
