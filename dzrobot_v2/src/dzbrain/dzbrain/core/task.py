# -*- coding: utf-8 -*-
"""任务线程基类 —— "每个任务点一个线程" 的落地骨架。

每个任务 = 一个独立线程 + 固定频率 + 独立状态机 + 独立失效兜底。

框架保证：
1. **固定频率**：按绝对时基排下一拍（补偿执行耗时，避免累积漂移）；超时记 overrun。
2. **异常隔离**：单拍异常不杀线程；连续异常超阈值 → 进入 FAULT 并上报（SafetyTask 会据此停车）。
3. **心跳**：`last_tick_at / ticks / errors / overruns` 随时可查（SafetyTask 的输入）。
4. **优雅停止**：`stop()` 置位 + join，`teardown()` 保证被调用。

约束（写任务时必读）：
- `tick()` 必须**非阻塞**、**无长 I/O**、**不持锁等待别的任务**；重活放到独立进程（RKNN/底盘/云台）。
- 任务之间**只能**通过 Blackboard（高频状态）与 EventBus（低频事件）通信，禁止直接互相调用。
"""

from __future__ import annotations

import enum
import threading
import time
import traceback
from dataclasses import dataclass
from typing import Callable

from .blackboard import Blackboard
from .events import EV_TASK_FAULT, EventBus


class TaskState(str, enum.Enum):
    INIT = "INIT"
    RUNNING = "RUNNING"
    DEGRADED = "DEGRADED"      # 有异常但仍在跑
    FAULT = "FAULT"            # 连续异常超阈值，已停止 tick
    STOPPED = "STOPPED"


@dataclass
class TaskHealth:
    """任务健康度快照（SafetyTask / 遥测消费）。"""

    name: str
    rate_hz: float
    state: str = TaskState.INIT.value
    ticks: int = 0
    errors: int = 0
    consecutive_errors: int = 0
    overruns: int = 0
    max_tick_ms: float = 0.0
    last_tick_at: float = 0.0
    last_error: str = ""
    started_at: float = 0.0

    def age(self, now: float | None = None) -> float:
        now = time.time() if now is None else now
        return float("inf") if not self.last_tick_at else now - self.last_tick_at


class TaskThread(threading.Thread):
    """任务线程基类。子类实现 setup/tick/teardown。"""

    #: 连续异常达到此值即判定 FAULT
    max_consecutive_errors = 5

    def __init__(
        self,
        name: str,
        rate_hz: float,
        *,
        bus: EventBus | None = None,
        blackboard: Blackboard | None = None,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        super().__init__(name=name, daemon=True)
        if rate_hz <= 0:
            raise ValueError(f"{name}: rate_hz 必须 > 0，收到 {rate_hz}")
        self.name = name
        self.rate_hz = float(rate_hz)
        self.period = 1.0 / float(rate_hz)
        self.bus = bus or EventBus()
        self.bb = blackboard or Blackboard()
        self._log = logger or (lambda msg: print(f"[{self.name}] {msg}", flush=True))
        self._stop_event = threading.Event()
        self._health = TaskHealth(name=name, rate_hz=self.rate_hz)
        self._health_lock = threading.Lock()

    # ------------------------------------------------------------------ #
    #  子类接口
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        """线程启动前调用一次：订阅话题/事件、建发布者。异常将导致任务直接 FAULT。"""

    def tick(self) -> None:
        """单拍逻辑。子类**必须**实现。"""
        raise NotImplementedError

    def teardown(self) -> None:
        """退出时调用一次：发布零指令、释放资源。必须尽量不抛异常。"""

    def fault_hook(self, exc: BaseException) -> None:
        """进入 FAULT 前的最后机会（例如发布停车指令）。"""

    # ------------------------------------------------------------------ #
    #  运行
    # ------------------------------------------------------------------ #
    def run(self) -> None:  # noqa: D401
        h = self._health
        with self._health_lock:
            h.started_at = time.time()
            h.state = TaskState.RUNNING.value
        try:
            self.setup()
        except Exception as exc:  # 启动失败 = 直接 FAULT
            self._fail("setup", exc)
            return

        self._log(f"启动：{self.rate_hz:g} Hz")
        next_deadline = time.perf_counter()
        while not self._stop_event.is_set():
            t0 = time.perf_counter()
            try:
                self.tick()
                with self._health_lock:
                    h.ticks += 1
                    h.consecutive_errors = 0
                    h.last_tick_at = time.time()
            except Exception as exc:  # 单拍异常隔离
                with self._health_lock:
                    h.errors += 1
                    h.consecutive_errors += 1
                    h.last_error = f"{type(exc).__name__}: {exc}"
                    h.last_tick_at = time.time()
                    h.state = TaskState.DEGRADED.value
                    consec = h.consecutive_errors
                self._log(f"tick 异常 #{consec}: {h.last_error}")
                if consec >= self.max_consecutive_errors:
                    self._fail("tick", exc)
                    break
            dt_ms = (time.perf_counter() - t0) * 1000.0
            with self._health_lock:
                if dt_ms > h.max_tick_ms:
                    h.max_tick_ms = dt_ms
                if dt_ms > self.period * 1000.0:
                    h.overruns += 1

            next_deadline += self.period
            sleep_s = next_deadline - time.perf_counter()
            if sleep_s > 0:
                self._stop_event.wait(sleep_s)
            else:
                # 落后太多：重置时基，避免"追赶"式连拍
                next_deadline = time.perf_counter()
                with self._health_lock:
                    h.overruns += 1

        try:
            self.teardown()
        except Exception as exc:
            self._log(f"teardown 异常: {type(exc).__name__}: {exc}")
        with self._health_lock:
            if h.state != TaskState.FAULT.value:
                h.state = TaskState.STOPPED.value
        self._log("已停止")

    def _fail(self, where: str, exc: BaseException) -> None:
        with self._health_lock:
            self._health.state = TaskState.FAULT.value
            self._health.last_error = f"{type(exc).__name__}: {exc}"
        self._log(f"FAULT（{where}）：{type(exc).__name__}: {exc}\n"
                  + "".join(traceback.format_exception_only(type(exc), exc)).strip())
        try:
            self.fault_hook(exc)
        except Exception:
            pass
        self.bus.publish(EV_TASK_FAULT, source=self.name,
                         task=self.name, where=where, error=str(exc))
        self._stop_event.set()

    def stop(self, timeout: float | None = 2.0) -> bool:
        """请求停止并（可选）等待退出。返回是否已退出。"""
        self._stop_event.set()
        if self.is_alive():
            self.join(timeout=timeout)
        return not self.is_alive()

    @property
    def stopping(self) -> bool:
        return self._stop_event.is_set()

    def health(self) -> TaskHealth:
        with self._health_lock:
            return TaskHealth(**vars(self._health))


class TaskGroup:
    """任务集合：统一启动/停止/健康汇总。"""

    def __init__(self, bus: EventBus, blackboard: Blackboard,
                 logger: Callable[[str], None] | None = None) -> None:
        self.bus = bus
        self.bb = blackboard
        self._tasks: list[TaskThread] = []
        self._log = logger or (lambda msg: print(msg, flush=True))

    def add(self, task: TaskThread) -> TaskThread:
        self._tasks.append(task)
        return task

    def start_all(self) -> None:
        for t in self._tasks:
            t.start()
        self._log(f"任务组启动：{len(self._tasks)} 个线程 -> "
                  + ", ".join(f"{t.name}@{t.rate_hz:g}Hz" for t in self._tasks))

    def stop_all(self, timeout: float = 2.0) -> list[str]:
        """逆序停止（后启动的先停），返回未优雅退出的任务名。"""
        stuck: list[str] = []
        for t in reversed(self._tasks):
            if not t.stop(timeout=timeout):
                stuck.append(t.name)
        return stuck

    def health(self) -> dict[str, TaskHealth]:
        return {t.name: t.health() for t in self._tasks}

    def faulty(self) -> list[str]:
        return [t.name for t in self._tasks if t.health().state == TaskState.FAULT.value]

    def describe(self) -> list[dict]:
        now = time.time()
        out = []
        for t in self._tasks:
            h = t.health()
            out.append({
                "name": h.name, "state": h.state, "rate_hz": h.rate_hz,
                "ticks": h.ticks, "errors": h.errors, "overruns": h.overruns,
                "age_s": round(h.age(now), 3), "max_tick_ms": round(h.max_tick_ms, 2),
                "last_error": h.last_error,
            })
        return out
