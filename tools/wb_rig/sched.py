# -*- coding: utf-8 -*-
"""虚拟时钟 + 定序调度（卡 `WB-2-RIG-20261031`）。

## 为什么不用 `thread.start()`（这是本卡最关键的一个决定）

生产任务是 `core.task.TaskThread`，`run()` 里的节拍是 `time.perf_counter()` + `Event.wait()`。
直接起线程会得到**两个不可接受**的后果：

1. **不可复现**：线程唤醒顺序由 OS 调度器决定，同一命令两次跑的 `steps.csv` 必然不同
   ⇒ 卡面"同命令两次跑 sha256 逐位相同"这条验收**永远过不了**；
2. **跑不动**：24.85 m @0.35 m/s ≈ 71 s 墙钟。压测多图（wb-4）会变成"一小时跑十张图"。

所以本台架**不起线程**，而是按"定序 + 虚拟时钟"直接调**生产任务的 `setup()`/`tick()`/
`teardown()` 本尊**。这**不是**第二份状态机：

| 担心 | 事实 |
|---|---|
| "你不就是在自己实现调度吗" | 调度不是状态机。状态机是 `tick()` 里的决策（Localization 的吸附闸、Tracker 的投影/停站、Mission 的阶段迁移），这些**一个字节都没重写**，调的就是生产函数 |
| "那你怎么保证频率对" | `RATES` 取自生产类的 `rate_hz`（`LocalizationTask=20`、`TrackerTask=50`、`SafetyTask=20`、`MissionTask=10`），不是抄来的常数；`test_wb2_rig_invariants.py` 有一条断言把它们与 `cls(...).rate_hz` 对拍 |
| "虚拟时钟不会让代码走不到某条分支吗" | 会 —— 所有用到 `time.monotonic()` 的地方都改走注入时钟（生产类**本来就**支持 `clock=` 形参，`bench` 与 `tests/fakes.py` 也用它）。唯二不可注入的是 `TaskThread.run()` 的**节拍**（本类接管）与任务健康的 `last_tick_at`（本类按虚拟时钟回填，见 `_mirror_health`） |

★ `_mirror_health` 为什么必须存在：`SafetyTask` 的检查①/③读的是
  `group.health()` 里的 `last_tick_at`（生产上由 `TaskThread.run()` 每一拍写墙钟）。
  我们不用 `run()`，所以必须按**虚拟时钟**补写，否则宽度 8 s 的启动宽限过后，
  每拍都会判 `pose_stalled:Localization 心跳 infs` → 锁存 SAFE_STOP → 车一动不动。
  这是一条**真实的坑**（`tasks/safety.py:298-311` 的注释记着实车血证），
  不补写就会在本台架上复现一次"假故障"，然后把整条线误判成"台架不可用"。
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field

#: 世界模型（真值积分 + 里程计生成）的积分步长（s）。取 Tracker 周期的 1/5：
#: 真值是**连续**物理量，但控制链是离散的；步长越小越接近"车在动"而不是"车在跳"，
#: 且 0.002 s 让 20 Hz 定位重算之间仍有足够的分辨率（0.004 s 会让 50 Hz 控制看到阶梯）。
WORLD_DT_S = 0.002

#: 采样档（Hz）：odom / Localization / Tracker。默认**取真车值**。
#: ⚠️ 这三档是"可配置"的（卡面要求），但**默认值不许单挡化**——
#: 曾经有人把所有任务压到 50 Hz 图省事，结果 Localization 每拍重算吸附闸，
#: 触发条件与现场 20 Hz 完全不同，指标整体不可迁移。
SAMPLE_HZ: dict[str, float] = {"odom": 100.0, "localization": 20.0, "tracker": 50.0}


@dataclass
class VirtualClock:
    """可注入的单调时钟（`now()` 单调不减，与 `time.monotonic` 同契约）。

    ★ 用 `t0 + n*dt` 而不是 `t += dt` 累加：浮点累加会让第 5 万拍的时刻带上
      1e-9 量级的尾差（实测 `time_s = 99.99999999763531`），而 `s`/`dt` 又会把这尾巴
      带进指标的最后几位 —— 逐位复现虽然还成立（同样的累加序列），但**人读的数字是脏的**。
      用整数步数 + 一次乘法则两者都干净。
    """

    t0: float = 1000.0
    _n: int = 0
    _dt: float = 0.001

    def __post_init__(self) -> None:
        self._n = 0
        self._dt = 0.001

    def now(self) -> float:
        return float(self.t0) + self._n * self._dt

    def advance(self, dt: float) -> float:
        if dt < 0.0:
            raise ValueError(f"虚拟时钟不许倒退（dt={dt!r}）")
        start = self.now()
        self._dt = float(dt)
        self._n = int(round((start - float(self.t0)) / self._dt)) + 1
        # 换步长（罕见）时保持连续：以新 dt 重算步数
        return self.now()

    @property
    def elapsed(self) -> float:
        return round(self.now() - float(self.t0), 9)


@dataclass
class Slot:
    """一个任务在调度表里的位置。"""

    name: str
    task: object          # TaskThread 实例（生产类）
    rate_hz: float
    next_due: float
    ticks: int = 0

    @property
    def period(self) -> float:
        return 1.0 / float(self.rate_hz)


class Scheduler:
    """定序调度：虚拟时钟推进 → 按到期时刻**升序**跑 `tick()`。

    ★ 定序规则（三条，缺一条就不确定）：
      1. 同一时刻多个任务到期 ⇒ 按**登记顺序**（`order` 形参），不按名字排序 ——
         登记顺序就是生产 `build_tasks()` 的顺序（Localization → Safety → Tracker →
         Mission），换个顺序会让"同一拍谁先看到谁的结果"变掉，指标跟着变；
      2. 每个任务用**自己的** `next_due` 累加（`+= period`），不做"追赶"（落后就重置），
         与 `TaskThread.run()` 的 `next_deadline` 语义一致；
      3. `dt` 迭代用固定 `WORLD_DT_S`，不用墙钟差值 —— 这是逐位可复现的**唯一**来源。
    """

    def __init__(self, clock: VirtualClock) -> None:
        self.clock = clock
        self._slots: list[Slot] = []
        self.steps = 0
        #: 实际调用 tick() 的总数（= 全部任务拍数和），跑完与 `expected_ticks()` 对拍
        self.calls = 0
        #: 上一步**真的跑过**的任务名集合（供外部"同拍配对"用：例如信念↔真值必须
        #: 在 Localization 刚写过 `P_POSE` 的那一步比较，否则量的是时间错位）。
        self.last_ran: frozenset[str] = frozenset()

    # ------------------------------------------------------------------ #
    #  装配
    # ------------------------------------------------------------------ #
    def add(self, task: object, *, rate_hz: float | None = None) -> Slot:
        """登记一个任务。`rate_hz` 缺省取任务自己的 `rate_hz`（生产真源）。"""
        hz = float(rate_hz if rate_hz is not None else getattr(task, "rate_hz"))
        if hz <= 0.0:
            raise ValueError(f"{getattr(task, 'name', task)!r}: rate_hz 必须 > 0，收到 {hz!r}")
        slot = Slot(name=str(getattr(task, "name", type(task).__name__)), task=task,
                    rate_hz=hz, next_due=self.clock.now())
        self._slots.append(slot)
        return slot

    @property
    def slots(self) -> tuple[Slot, ...]:
        return tuple(self._slots)

    def setup_all(self) -> None:
        """按登记顺序调 `setup()`（与 `TaskGroup.start_all()` 的先后一致）。"""
        for s in self._slots:
            s.task.setup()

    def teardown_all(self) -> list[str]:
        """逆序调 `teardown()`（与 `TaskGroup.stop_all()` 一致：后启动的先停）。"""
        stuck: list[str] = []
        for s in reversed(self._slots):
            try:
                s.task.teardown()
            except Exception as exc:                     # noqa: BLE001  teardown 不许炸整轮
                stuck.append(f"{s.name}: {type(exc).__name__}: {exc}")
        return stuck

    # ------------------------------------------------------------------ #
    #  运行
    # ------------------------------------------------------------------ #
    def run_step(self) -> bool:
        """推进 `WORLD_DT_S` 并跑掉所有到期任务。返回"本步是否真有任务被调用"。"""
        self.clock.advance(WORLD_DT_S)
        self.steps += 1
        now = self.clock.now()
        due = [(s.next_due, i) for i, s in enumerate(self._slots) if s.next_due <= now + 1e-12]
        if not due:
            self.last_ran = frozenset()
            return False
        heapq.heapify(due)
        ran_names: list[str] = []
        while due:
            _, idx = heapq.heappop(due)
            s = self._slots[idx]
            if s.next_due > self.clock.now() + 1e-12:
                continue
            self._tick_one(s)
            ran_names.append(s.name)
            # 累加而非追赶：落后超过一个周期就重置时基（同 TaskThread.run 的 else 分支）
            s.next_due += s.period
            if s.next_due < self.clock.now() - s.period:
                s.next_due = self.clock.now()
            if s.next_due <= self.clock.now() + 1e-12:
                heapq.heappush(due, (s.next_due, idx))
        self.last_ran = frozenset(ran_names)
        return bool(ran_names)

    def _tick_one(self, s: Slot) -> None:
        """跑一拍**生产** `tick()`，并按虚拟时钟回填任务健康台账。"""
        s.task.tick()
        s.ticks += 1
        self.calls += 1
        self._mirror_health(s.task)

    @staticmethod
    def _mirror_health(task: object) -> None:
        """按虚拟时钟写 `_health`（`TaskThread.run()` 在生产上每拍做这件事）。

        ★ 不写会怎样：`SafetyTask` 读 `group.health()` 的 `last_tick_at`，
          真实时间是墙钟而虚拟时钟停在 1000 s 附近 ⇒ `age` 恒为正无穷 ⇒
          启动宽限（8 s 虚拟秒）一过就 `pose_stalled` → SAFE_STOP 锁存 → 车不动。
        只回填三个字段，**不**动 `last_error`/`overruns`（那是真故障台账）。
        """
        lock = getattr(task, "_health_lock", None)
        health = getattr(task, "_health", None)
        clock = getattr(task, "clock", None) or getattr(task, "_clock", None)
        if health is None or clock is None:
            return
        now = float(clock())
        writer = (lambda: setattr(health, "last_tick_at", now)) if lock is None else None
        if writer is not None:
            writer()
            return
        with lock:
            health.last_tick_at = now
            if health.consecutive_errors == 0 and not health.last_error:
                health.state = "RUNNING"

    def expected_ticks(self, seconds: float) -> dict[str, int]:
        """给定时长下各任务的理论拍数（测试对拍"没漏拍/没多拍"）。"""
        return {s.name: int(round(seconds * s.rate_hz)) for s in self._slots}


__all__ = ["VirtualClock", "Slot", "Scheduler", "WORLD_DT_S", "SAMPLE_HZ"]
