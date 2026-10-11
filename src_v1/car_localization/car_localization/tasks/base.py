"""tasks/base.py — TaskThread 基类：固定频率循环 + 异常隔离 + 心跳 + 优雅停止。

对应 V2 core/task.py 的设计模式，但独立于 rclpy/V2 框架。
与 V1 语义对应：V1 的 control_loop 是单个 20Hz 定时器回调；
V2 中每个任务是独立的 TaskThread 子类，各自以固定频率运行 tick()。

关键设计：
  - run() 用绝对时基安排下一拍（clock time, non-sleep-skew）
  - 异常不崩线程：连续异常超过 MAX_CONSECUTIVE_ERRORS 才停
  - _stopped Event 用于停止（显式不与 threading.Thread._stop 冲突）
  - 可嵌套 stop()：线程安全
"""

import threading
import time
import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple


class TaskState(Enum):
    INIT = 'INIT'
    RUNNING = 'RUNNING'
    DEGRADED = 'DEGRADED'
    FAULT = 'FAULT'
    STOPPED = 'STOPPED'


@dataclass
class TaskHealth:
    """任务健康快照。"""
    name: str
    rate_hz: float
    state: TaskState
    ticks: int
    errors: int
    consecutive_errors: int
    last_tick_time: float
    last_error: str = ''


class TaskThread(threading.Thread):
    """可继承任务线程基类。

    用法：
      class MyTask(TaskThread):
          def tick(self):
              # 业务逻辑
              pass

      task = MyTask(name='my_task', rate_hz=20)
      task.start()
      # ...
      task.stop()

    注意：子类 _stopped 属性用 self._stopped（Event），
    不要与 threading.Thread._stop 冲突。
    """

    MAX_CONSECUTIVE_ERRORS: int = 10  # 连续异常阈值 -> FAULT

    def __init__(
        self,
        name: str,
        rate_hz: float,
        logger: Optional[Any] = None,
    ):
        """初始化 TaskThread。

        Args:
            name: 任务名称（线程名）
            rate_hz: tick 频率（Hz）
            logger: 日志对象（可选，None 时用 print）
        """
        super().__init__(name=name, daemon=True)
        self._task_name: str = name
        self._rate_hz: float = rate_hz
        self._period: float = 1.0 / rate_hz if rate_hz > 0 else 0.05
        self._log: Any = logger
        self._log_impl: Callable[..., None] = self._default_logger

        # ---- 停止控制 ----
        # 使用 Event 而非 threading.Thread._stop（显式避名）
        self._stopped: threading.Event = threading.Event()
        self._stopped.clear()

        # ---- 状态 ----
        self._state: TaskState = TaskState.INIT
        self._lock: threading.Lock = threading.Lock()
        self._ticks: int = 0
        self._errors: int = 0
        self._consecutive_errors: int = 0
        self._last_tick_time: float = 0.0
        self._last_error: str = ''

        # setup / teardown 保护
        self._setup_done: bool = False

    def _default_logger(self, msg: str, level: str = 'info') -> None:
        """默认日志：用 print 模拟。"""
        tag = level.upper() if level else 'INFO'
        print(f"[{tag}][{self._task_name}] {msg}")

    def log(self, msg: str, level: str = 'info') -> None:
        """日志输出。"""
        if self._log is not None:
            self._log(msg)
        else:
            self._log_impl(msg, level)

    # ── 子类可覆写 ──────────────────────────────────────────

    def setup(self) -> None:
        """初始化钩子（start 后立即调用）。"""
        pass

    def tick(self) -> None:
        """每拍业务逻辑（子类必须覆写）。"""
        raise NotImplementedError(f"{self._task_name}.tick() not implemented")

    def teardown(self) -> None:
        """停止前的收尾钩子。"""
        pass

    def fault_hook(self, exc: Exception) -> None:
        """连续异常超限时的钩子（默认 log）。"""
        self.log(f"FAULT: {exc}", level='error')

    # ── 停止 ────────────────────────────────────────────────

    def stop(self, timeout: Optional[float] = None) -> bool:
        """请求停止线程。

        Args:
            timeout: 等待线程退出的超时秒数（None = 不等待）

        Returns:
            True = 线程已退出，False = 超时
        """
        self._stopped.set()  # 通知 run() 退出
        if timeout is not None:
            self.join(timeout)
            return not self.is_alive()
        return True

    # ── 健康 ────────────────────────────────────────────────

    def health(self) -> TaskHealth:
        """获取健康快照（线程安全）。"""
        with self._lock:
            return TaskHealth(
                name=self._task_name,
                rate_hz=self._rate_hz,
                state=self._state,
                ticks=self._ticks,
                errors=self._errors,
                consecutive_errors=self._consecutive_errors,
                last_tick_time=self._last_tick_time,
                last_error=self._last_error,
            )

    def is_faulted(self) -> bool:
        """是否进入 FAULT 状态。"""
        return self._state == TaskState.FAULT

    def is_running(self) -> bool:
        """是否正常运行中。"""
        return self._state == TaskState.RUNNING

    # ── run() ───────────────────────────────────────────────

    def run(self) -> None:
        """主循环（模板方法）。

        setup → 循环(tick) → teardown
        频率控制用绝对时基，每 tick 计算下一拍目标时间。
        """
        try:
            self._state = TaskState.RUNNING
            self.setup()
            self._setup_done = True

            # 绝对时基安排
            next_tick = time.monotonic()

            while not self._stopped.is_set():
                # 等距到下一拍
                now = time.monotonic()
                sleep_seconds = next_tick - now
                if sleep_seconds > 0:
                    # 用很短间隔等待停止信号
                    self._stopped.wait(min(sleep_seconds, 0.1))
                    if self._stopped.is_set():
                        break
                    now = time.monotonic()

                # 更新 next_tick（以 0 为误差累积边界）
                while next_tick <= now:
                    next_tick += self._period

                # 执行 tick
                try:
                    self.tick()
                    with self._lock:
                        self._ticks += 1
                        self._consecutive_errors = 0
                        self._last_tick_time = now
                except Exception as e:
                    with self._lock:
                        self._errors += 1
                        self._consecutive_errors += 1
                        self._last_error = str(e)
                        self._last_tick_time = now
                    self.log(f"tick error ({self._consecutive_errors}): {e}",
                             level='error')

                    # 连续异常超限 -> FAULT
                    if self._consecutive_errors >= self.MAX_CONSECUTIVE_ERRORS:
                        self._state = TaskState.FAULT
                        self.fault_hook(e)
                        break

        except Exception as e:
            self._state = TaskState.FAULT
            self._last_error = str(e)
            self.log(f"Task {self._task_name} FATAL: {e}", level='error')
        finally:
            self._state = TaskState.STOPPED
            try:
                self.teardown()
            except Exception as e:
                self.log(f"teardown error: {e}", level='error')
