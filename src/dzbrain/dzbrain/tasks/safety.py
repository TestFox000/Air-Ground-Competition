# -*- coding: utf-8 -*-
"""SafetyTask —— 安全看门狗（20 Hz，架构 §3.1 的 T7）★ **唯一有权强制停车者**。

为什么必须是"唯一"：旧版 `path_executor` 把越界急停写在**控制分支内部**（survey §6.2），
结果 PIVOT 蠕动分支（2422–2468，0.08 m/s、无超时）**根本没有边界检查** ——
"跑出地图 12 m 仍未停"就是这么发生的。新版的立场：

    刹车决策只在这一处做，判据只读 Blackboard/事件，与"车正在执行哪个状态机"完全无关。
    因此不存在"某个分支忘了检查"这回事。

五道检查（每项独立判据 + 独立限流日志；任一触发即强制停车）：
  ① 定位失联：`bridge.pose_age()` 为 inf 或 P_ODOM_AGE > `safety.pose_timeout_s`
     （另加）：Localization 心跳本身也不能停滞 —— 否则"它不再写 P_ODOM_AGE"会被误读成健康。
  ② 越界：`not geo.in_bounds(x, y, pursuit.map_safety_margin_m)`（★ 顶层生效，修旧版缺口）。
  ③ 任务健康：`group` 里任一任务 FAULT 或心跳年龄 > `safety.task_stall_s`。
  ④ 收到 `EV_TASK_FAULT`（订阅事件；覆盖"任务在 group 之外/事件早于本拍"的情形）。
  ⑤ 遥控接管：`bridge.manual_override()` 为真 **不算故障**（这是合法状态，人比机器优先），
     只记录"当前处于遥控接管"；里程计超时/越界在接管期间**照样报警**（那时更危险）。

恢复语义（★ 刻意不做自动恢复）：所有条件解除后**不自动恢复**，必须显式 `clear_safety()`。
理由：抖动场景下自动恢复会变成"刹车—起步—刹车"的反复启停，比停着更危险；
"谁清的场、什么时候清的"必须留痕（`clear_safety()` 打横幅 + 发 `EV_SAFETY_CLEAR`）。

铁律：`tick()` 非阻塞、无长 I/O、不 import rclpy、不调 `time.time()/time.monotonic()`
（一律用注入的 `clock()`）。
"""

from __future__ import annotations

import math
from typing import Callable

from ..core import geometry as geo
from ..core.blackboard import P_ODOM_AGE, P_POSE, P_SAFETY, Blackboard
from ..core.config import Config
from ..core.events import (EV_SAFETY_CLEAR, EV_SAFETY_STOP, EV_TASK_FAULT,  # noqa: F401
                           EventBus, Subscription)
from ..core.logging import Throttle, banner
from ..core.state import SafetyState
from ..core.task import TaskGroup, TaskState, TaskThread

__all__ = ["SafetyTask"]

_INF = float("inf")
#: 任务健康年龄的时钟偏斜容差（秒）：允许"心跳时刻比现在略晚"而不误报停滞
_CLOCK_SKEW_S = 0.05


class SafetyTask(TaskThread):
    """把"该不该动"收敛成一个布尔量 `P_SAFETY.ok`，并把强制停车权留在自己手里。"""

    def __init__(self, bridge, bb: Blackboard, bus: EventBus, cfg: Config, *,
                 log: Callable[[str], None] = print, group: TaskGroup | None = None,
                 clock: Callable[[], float] = None) -> None:
        from time import monotonic as _monotonic
        super().__init__("Safety", 20.0, bus=bus, blackboard=bb, logger=log)
        self.bridge = bridge
        self.cfg = cfg
        self.group = group
        self.clock = _monotonic if clock is None else clock

        # 参数一次读齐
        self.pose_timeout_s = float(cfg.get("safety.pose_timeout_s") or 1.0)
        self.task_stall_s = float(cfg.get("safety.task_stall_s") or 1.0)
        self.brake_on_fault = bool(cfg.get("safety.brake_on_fault"))
        self.map_margin_m = float(cfg.get("pursuit.map_safety_margin_m") or 0.0)
        #: 启动宽限：从未收到任何定位数据时的容忍时间（超过才判"定位失联"）。
        #: 为什么不复用 `safety.pose_timeout_s`：那个是"数据流断了多久算丢"（运行期语义），
        #: 而这里是"系统还没跑起来"（启动期语义），两者含义不同、不该共用一个键。
        self._startup_grace_s = float(cfg.get("safety.startup_grace_s") or 3.0)
        self._warn = Throttle(float(cfg.get("logging.rate_limit_s") or 1.0))

        # 状态
        self._reasons: list[str] = []        # 当前生效的原因（**有序**，首次触发顺序）
        self.stopped = False                 # 是否处于"安全停车"锁存态（恢复必须显式 clear）
        self._last_kind: str = ""            # 上一次变化的类型（reason|all_clear|auto_recover）
        self._manual_last: bool | None = None
        self._sub: Subscription | None = None
        self._fault_reasons: dict[str, str] = {}   # 事件来源 → 原因串

        # 计数器（测试直接断言，不解析日志）
        self.ticks = 0
        self.brakes = 0
        self.forced_stops = 0
        self.stop_events = 0
        self.clear_events = 0
        self.fault_events_seen = 0
        self._first_data_at: float | None = None    # 第一次见到定位数据的时刻
        self._started_at = self.clock()             # 构造时刻 → 启动宽限基准
        self._degraded = False                      # "等数据中"（DEGRADED，不是 SAFE_STOP）
        #: ★ 本进程内是否**见过两个来源都健康的定位**（启动期与运行期的分界）。
        #  为什么需要它（实车实测 2026-10-09 20:1x，Orange Pi）：两个任务对"有没有定位"
        #  的判断**天然差一拍** ——
        #    · Localization 先 tick：它自己还没拿到里程计 → 把 `P_ODOM_AGE` 写成 `inf`
        #    · Safety 后 tick：桥此刻**已经**收到帧（`pose_age` 有限）
        #  ⇒ 旧判据 `never_had_pose = pose_age==inf and (odom_age is None or inf)` 为假，
        #    于是绕开启动宽限、直接落到"`elif pose_age == _INF or odom_age == _INF`"
        #    那条**既无宽限也无日志**的分支 → `pose_lost` → 锁存 SAFE_STOP；
        #    而安全是**刻意不自动恢复**的 ⇒ 车一动不动。
        #  日志特征（可据此一眼认出）：只有一条 `启动宽限（0.0x/3.0s）`，
        #  **没有** `❌ 启动宽限期已过`，却出现了 `pose_lost`。
        self._ever_had_pose = False
        #: `P_ODOM_AGE=inf` 的起始时刻（None = 当前不是 inf）→ 配合持续性判据，见 `_check_pose`。
        self._odom_inf_since: float | None = None

        # 恢复后立刻又触发 → 记 auto_recover；连续 N 次视为"抖动"，打横幅
        self.auto_recover_count = 0
        self.auto_recover_limit = 5
        self.last_recover_at = 0.0

    @property
    def reasons(self) -> tuple[str, ...]:
        return tuple(self._reasons)

    @property
    def degraded(self) -> bool:
        """是否处于"等定位数据中"（DEGRADED）——**不是**安全停车，但也不该发车。"""
        return self._degraded

    # ------------------------------------------------------------------ #
    #  生命周期
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        self._sub = self.bus.subscribe(self.name, [EV_TASK_FAULT])
        banner([
            "SafetyTask 启动（20 Hz）★ 唯一强制停车者",
            f"  定位超时      : {self.pose_timeout_s:.2f} s",
            f"  地图安全余量  : ±{self.map_margin_m:.2f} m  {geo.map_bounds(self.map_margin_m)}",
            f"  任务停滞判定  : {self.task_stall_s:.2f} s",
            f"  故障刹车      : {'ON' if self.brake_on_fault else 'OFF（只报不刹）'}",
            f"  任务组        : {'已注入 ' + str(len(self.group.health())) + ' 个' if self.group else '未注入 → 跳过检查③'}",
        ], log=self._log)

    def tick(self) -> None:
        self.ticks += 1
        now = self.clock()
        self._pump_events()
        pose = self.bb.get(P_POSE)
        if not isinstance(pose, geo.Pose):
            pose = None

        reasons = self._evaluate(pose, now)

        if reasons:
            self._update(reasons, now)
        elif self.stopped:
            # ★ 条件解除**不自动恢复**：只记一次"其实已可恢复"，等人来 clear
            self.auto_recover_count += 1
            self.last_recover_at = now
            if self._warn.allow("auto_recover", now):
                self._log(f"安全条件已全部解除，但**不自动恢复**（第 {self.auto_recover_count} 拍）"
                          f"—— 需显式 clear_safety()。解除前原因：{list(self._reasons)}")
            if self.auto_recover_count == self.auto_recover_limit:
                banner(["⚠️ SafetyTask：安全条件反复解除又触发（抖动）",
                        f"  已 {self.auto_recover_count} 次；本轮不做自动恢复，",
                        "  请检查定位/边界/任务健康后再 clear_safety()。"], char="!", log=self._log)
        else:
            # ★ 健康时必须**至少写一次** P_SAFETY（下游 Tracker/Mission 要 "ok 才许动"）。
            #   P1 §2 把 P_SAFETY 的读方列为"全部"：若键一直不存在，读方要么 KeyError
            #   要么各自 default=True —— 那就等于"安全系统没上线"，属于静默失效。
            #   所以启动第一拍立即写一条 OK；此后健康期不重复写（黑盒写入量不该被 20 Hz 灌满）。
            cur = self.bb.get(P_SAFETY)
            if not isinstance(cur, SafetyState) or not cur.ok:
                st = self._state(ok=True, now=now)
                self.bb.set(P_SAFETY, st, source=self.name, ts=now)
                self._log(f"安全系统就绪：P_SAFETY.ok=True"
                          f"（越界余量 ±{self.map_margin_m:.2f} m / 定位超时 "
                          f"{self.pose_timeout_s:.2f} s）")

    def teardown(self) -> None:
        if self._sub is not None:
            self._sub.unsubscribe()
            self._sub = None
        self._log(f"安全任务退出：{self.ticks} 拍 / 锁存中={self.stopped}"
                  f" / 累计刹车 {self.brakes} 次 / 停止事件 {self.stop_events} 条"
                  f" / 当前原因 {list(self._reasons)}")

    # ------------------------------------------------------------------ #
    #  对外：显式清场（唯一的恢复入口）
    # ------------------------------------------------------------------ #
    def clear_safety(self, operator: str = "manual", source: str = "operator") -> SafetyState:
        """显式清除安全停车。**不在** tick 里自动调用，只能由人或上层任务调用。

        返回清除后的 SafetyState；若当前无锁存、无原因 → 幂等返回，不重复发事件。
        """
        now = self.clock()
        if not self.stopped and not self._reasons:
            self._log(f"clear_safety({operator})：当前无安全故障，忽略（幂等）")
            return self._state(ok=True, reasons=(), now=now)
        cleared = list(self._reasons)
        self._reasons = []
        self._fault_reasons.clear()
        was_stopped = self.stopped
        self.stopped = False
        self._last_kind = "all_clear"
        st = self._state(ok=True, now=now)
        self.bb.set(P_SAFETY, st, source=self.name, ts=now)
        self.bus.publish(EV_SAFETY_CLEAR, source=self.name, operator=operator,
                         cleared=tuple(cleared), was_stopped=was_stopped)
        self.clear_events += 1
        banner([f"✅ 安全停车已清除（{operator}）",
                f"  清除的原因: {cleared if cleared else '（无）'}",
                f"  锁存前状态: {'SAFE_STOP' if was_stopped else 'OK'}",
                "  ★ 本次清除后 Tracker 可恢复驱动，请确认现场安全。"], char="*", log=self._log)
        return st

    # 旧版命名兼容（P1 文档写的是 clear_safety()，这里不额外提供别名，避免接口漂移）

    # ------------------------------------------------------------------ #
    #  五道检查
    # ------------------------------------------------------------------ #
    def _evaluate(self, pose: geo.Pose | None, now: float) -> list[str]:
        out: list[str] = []

        # ⑤ 遥控接管：不是故障，只记录（人比机器优先）
        try:
            manual = bool(self.bridge.manual_override())
        except Exception as exc:                       # 桥接口异常不能拖垮安全线程
            manual = False
            if self._warn.allow("manual_err", now):
                self._log(f"manual_override() 异常，按未接管处理：{type(exc).__name__}: {exc}")
        if manual != self._manual_last:
            self._log(f"遥控接管：{'ON（人接管底盘，安全判据继续生效）' if manual else 'OFF（交回自动）'}")
            self._manual_last = manual

        # ① 定位失联（两个来源，任一异常即报：桥的年龄 + Localization 的心跳）
        # ★ 注意区分"键不存在(None)" 与 "值是 inf"：
        #   `float(None)` 抛 TypeError，若把两者都当 inf，则**冷启动第一拍**
        #   （P_ODOM_AGE 还没人写、但 bridge.pose_age() 明明是 0.0）就会被误判成
        #   "定位失联"并白白刹一次车。键缺失时退化为"以 bridge 为准"。
        odom_raw = self.bb.get(P_ODOM_AGE)
        odom_age = None if odom_raw is None else self._finite(odom_raw)
        raw_pose_age = self._safe_call(self.bridge.pose_age)
        # 桥接口取不到 age（异常/缺失）不能当成"新数据"：按最坏情况处理，宁可误报
        pose_age = _INF if raw_pose_age is None else self._finite(raw_pose_age)
        heart_age = self._heartbeat_age("Localization", now)

        # ★ 启动宽限：**从未收到过任何定位数据** ≠ **跑着跑着定位丢了**。
        #   ⚠️ 判据不能要求"`P_ODOM_AGE` 键不存在" —— 这条真实路径会漏掉
        #   （**实车实测 2026-10-09 19:05，Orange Pi**）：
        #     · Localization **先 tick 了一拍**，把 `P_ODOM_AGE` 写成 `inf`
        #       （它自己此刻也还没拿到里程计 —— ROS2 发现还没完成）
        #     · Safety 随后判 `odom_age is None` 为假 ⇒ 落到 `elif pose_age == _INF`
        #       ⇒ **完全没有宽限**，直接 `pose_lost` 并锁存 SAFE_STOP
        #     · 而安全是**刻意不自动恢复**的 ⇒ 车一动不动（这正是"起不来"的第二个原因）
        #   ⇒ 判据改为"**桥没有帧** 且 `P_ODOM_AGE` 缺失或为 inf"（= 两种写法都算"从未有数据"）。
        never_had_pose = pose_age == _INF and (odom_age is None or odom_age == _INF)  # noqa: F841 (保留给日志排查)
        # ★ 桥有**新鲜位姿** ⇒ 记下"本进程见过定位"（此后任何失联都是运行期真故障）。
        #   为什么以桥为准而不是"两来源都健康"：`P_ODOM_AGE` 是 **Localization 自报**的
        #   心跳，而它与本任务读的是**同一个桥** —— "桥有帧、它却写 inf" 必然只是
        #   两个任务**差一拍**（≤ 1/20Hz = 50ms），把它算成"没见过定位"会让宽限永远
        #   续不上、真故障时反而漏报。
        if pose_age != _INF:
            self._ever_had_pose = True
        # ★ `P_ODOM_AGE=inf` 的**持续性**判据（实车 2026-10-09 20:1x 血证）：
        #   单拍 inf 是上面那种必然竞态；**持续**超过 `pose_timeout_s` 才是真故障。
        #   旧写法"任一来源 inf 即致命"会在启动/节点单独重启时**立刻锁存 SAFE_STOP**，
        #   而安全刻意不自动恢复 ⇒ 车一动不动（auto_start 也拉不动）。
        if odom_age == _INF:
            if self._odom_inf_since is None:
                self._odom_inf_since = now
        else:
            self._odom_inf_since = None
        odom_inf_long = (self._odom_inf_since is not None
                         and (now - self._odom_inf_since) > self.pose_timeout_s)
        # ★ 启动期 = **本进程内从未见过新鲜位姿**（不是"此刻某一来源是 inf"）。
        if not self._ever_had_pose:
            elapsed = now - self._started_at
            if elapsed <= self._startup_grace_s:
                self._degraded = True
                if self._warn.allow("no_pose_data", now):
                    self._log(f"启动宽限（{elapsed:.2f}"
                              f"/{self._startup_grace_s:.1f}s）：仍未收到任何定位数据，"
                              f"暂不判失联（pose_age={pose_age!r}，P_ODOM_AGE={odom_raw!r}）")
                return out                          # 只跳过检查①，③④照常
            self._degraded = False                   # 宽限过后不是"降级"，是明确故障
            if self._warn.allow("no_pose_data_end", now):
                self._log(f"❌ 启动宽限期（{self._startup_grace_s:.1f}s）已过，"
                          f"仍未收到任何定位数据 → 按失联处理")
            out.append("pose_lost:里程计无数据（age=inf）")
        elif pose_age == _INF or odom_inf_long:
            out.append("pose_lost:里程计无数据（age=inf）")
        elif pose_age > self.pose_timeout_s:
            out.append(f"pose_stale:里程计 {pose_age:.2f}s > {self.pose_timeout_s:.2f}s")
        elif odom_age is not None and odom_age != _INF and odom_age > self.pose_timeout_s:
            # ★ 必须排掉 inf：`inf > timeout` 恒真，会把"从未/丢失"误报成"陈旧 1 秒"
            #   （实测被这条二次锁存：`pose_stale:P_ODOM_AGE infs > 1.00s`）。
            #   inf 由上面的 `odom_inf_long`（持续性）单独负责。
            out.append(f"pose_stale:P_ODOM_AGE {odom_age:.2f}s > {self.pose_timeout_s:.2f}s")
        # ★ 启动竞态豁免（t27 实车 2026-10-11 15:15 血证）：Safety 首拍可能早于
        #   Localization 首拍 ⇒ _heartbeat_age 因 last_tick_at==0 返回 inf ⇒
        #   `inf > 2*timeout` 恒真 ⇒ 首拍即 pose_stalled 锁存 SAFE_STOP（且不自动恢复）。
        #   上面的启动宽限救不了这条：它判"桥从未有帧"，而桥由 serial_reader 提前喂 ⇒
        #   恒不触发（t27 日志实证「启动宽限」0 命中）。与 _check_group 的 init_grace
        #   同口径：从未 tick + 仍在宽限期内 ⇒ 不判；宽限过后仍从未 tick 照判
        #   （不削弱真故障语义，宽限不是免死金牌）。
        if heart_age is not None and heart_age > self.pose_timeout_s * 2.0:
            if heart_age == _INF and (now - self._started_at) <= self._startup_grace_s:
                if self._warn.allow("localization_init_grace", now):
                    self._log(f"检查①宽限：Localization 从未 tick（heart_age=inf），"
                              f"宽限 {self._startup_grace_s:.1f}s 内不判停滞")
            else:
                out.append(f"pose_stalled:Localization 心跳 {heart_age:.2f}s 未更新")
        if pose_age != _INF or odom_age is not None:
            self._first_data_at = now                # 见过定位数据了
            self._degraded = False

        # ② 越界（★ 顶层唯一实施点；旧版分散两处且 PIVOT 分支漏检）
        if pose is not None and not geo.in_bounds(pose.x, pose.y, self.map_margin_m):
            x_min, x_max, y_min, y_max = geo.map_bounds(self.map_margin_m)
            out.append(f"out_of_bounds:({pose.x:.3f}, {pose.y:.3f}) 超出 "
                       f"x[{x_min:.3f},{x_max:.3f}] y[{y_min:.3f},{y_max:.3f}]")

        # ③ 任务健康（group 未注入 → 跳过并说明）
        out.extend(self._check_group(now))

        # ④ EV_TASK_FAULT 事件
        out.extend(sorted(self._fault_reasons.values()))

        return out

    def _check_group(self, now: float) -> list[str]:
        """检查③：任一任务 FAULT 或心跳停滞。

        ★ **不检查自己**。看门狗盯自己有两个必然的坏结果：
        1. 自己的 `last_tick_at` 永远是"上一拍"，age ≥ 一个周期 + 偏斜容差。
           只要 `task_stall_s` 不大于周期（默认 1.0 s vs 0.05 s 虽然够，但配置一改就翻车），
           Safety 会把自己判成"停滞"并永久锁在 SAFE_STOP；
        2. 逻辑上也不可能 —— 看门狗死了就没人看门狗，自检通不出这个信息
           （该由外层 supervisor / systemd 负责）。
        首拍更明显：那时自己的 last_tick_at 还是 0.0（任务刚 RUNNING、本拍尚未结束），
        按"从未 tick 过 = 从未证明活着"会被判 inf 而误刹。
        """
        if self.group is None:
            if self._warn.allow("no_group", now):
                self._log("未注入 TaskGroup → **跳过检查③（任务健康/停滞）**；"
                          "检查①②④照常生效")
            return []
        out: list[str] = []
        for h in self.group.health().values():
            if h.name == self.name:
                if self._warn.allow("self_skip", now):
                    self._log(f"检查③跳过自己（{self.name}）：看门狗不盯自己，"
                              f"自身存活由外层 supervisor 负责")
                continue
            if h.state == TaskState.FAULT.value:
                out.append(f"task_fault:{h.name} state=FAULT err={h.last_error or '-'}")
                continue
            # ★ 冷启动宽限：任务组是**顺序 start** 的，Safety 起来时后面的线程可能还没
            #   开始跑（ticks=0、**last_tick_at 还是 0**）。此时判"心跳停滞"
            #   会把刚启动的系统直接锁死（bench 实测：Tracker/Mission 被判 stalled →
            #   刹车 + 不自动恢复）。
            #   ⚠️ 宽限条件**不能要求 `state == INIT`**：`TaskThread.start()` 是先把状态
            #   置成 RUNNING、再进循环跑第一拍 —— 于是"已 RUNNING 但从未 tick"的任务拿不到
            #   宽限，`age = _INF` 直接被判停滞。**周期最慢的两个天然中招**
            #   （DroneTask 5Hz / TelemetryTask 2Hz 的首拍要来 0.2~0.5s 之后）。
            #   ★ 实车实测（2026-10-09 18:57，Orange Pi 上 `python3 -m dzbrain.main`）：
            #       🛑 task_stalled:DroneTask 心跳 infs > 1.00s（ticks=0 state=RUNNING）
            #     而安全是**刻意不自动恢复**的 ⇒ 新核一上电就永久锁在 SAFE_STOP、车一动不动。
            #     （这条 bench 抓不到：bench 的起线程时序不同。）
            #   ⇒ 判据改为"**从未 tick 过**（last_tick_at=0）+ 仍在宽限期内"，与 state 无关。
            #   原语义仍保留：只要有 last_tick_at（说明它活过），心跳变旧就是真停滞 ——
            #   哪怕系统刚起来（否则"宽限期"会变成"启动后 3 秒内可以静默死掉"）。
            if (not h.last_tick_at
                    and (now - self._started_at) <= self._startup_grace_s):
                if self._warn.allow(f"init_grace_{h.name}"):
                    self._log(f"检查③宽限：{h.name} 从未 tick（ticks=0 state={h.state}），"
                              f"宽限 {self._startup_grace_s:.1f}s 内不判停滞")
                continue
            age = self._task_age(h, now)
            if age is not None and age > self.task_stall_s:
                out.append(f"task_stalled:{h.name} 心跳 {age:.2f}s > {self.task_stall_s:.2f}s"
                           f"（ticks={h.ticks} state={h.state}）")
        return out

    @staticmethod
    def _task_age(h, now: float) -> float | None:
        """任务心跳年龄。用注入时钟算，保证测试能快进（TaskHealth.age() 走墙上时钟）。"""
        if not h.last_tick_at:
            return _INF          # 从未 tick 过 = 从未证明自己活着
        return (now - h.last_tick_at) + _CLOCK_SKEW_S

    def _heartbeat_age(self, name: str, now: float) -> float | None:
        """某个任务的心跳年龄；查询不到（不在 group 里）返回 None。"""
        if self.group is None:
            return None
        h = self.group.health().get(name)
        if h is None:
            return None
        return self._task_age(h, now)

    # ------------------------------------------------------------------ #
    #  状态迁移
    # ------------------------------------------------------------------ #
    def _update(self, reasons: list[str], now: float) -> None:
        """把"当前原因集合"落成 P_SAFETY + 事件 + 刹车。

        ★ 两条铁律，都踩过坑：
        1. **刹车每拍都发**，不能被日志限流短路 —— 底盘有 1 s 看门狗，
           "发一次就算"会在故障持续期间被底盘放行。所以限流只能包住 `_log`，
           绝不能 `return` 掉刹车（写成 `elif not allow(): return` 就是这个错）。
        2. **同一原因集合只发一条 EV_SAFETY_STOP**；集合变化（新增/解除）才补发。
        """
        new = tuple(dict.fromkeys(reasons))            # 去重且保序（首次触发的顺序）
        prev = tuple(self._reasons)
        changed = new != prev
        if changed:
            added = [r for r in new if r not in prev]
            removed = [r for r in prev if r not in new]
            self._reasons = list(new)
            self._last_kind = "reason"
            self.bus.publish(EV_SAFETY_STOP, source=self.name, reasons=new,
                             added=tuple(added), removed=tuple(removed),
                             first=not self.stopped)
            self.stop_events += 1
            self._log(f"🛑 安全停车：{len(new)} 项原因"
                      + (f"（新增 {added}）" if added else "")
                      + (f"（解除 {removed}）" if removed else ""))
            for r in new:
                self._log(f"    · {r}")
        elif self._warn.allow("still_stopped", now):
            self._log(f"… 仍在安全停车中：{list(new)}")     # 每 rate_limit_s 提一次，不刷屏

        first = not self.stopped
        if first:
            self.stopped = True
            self.forced_stops += 1
            self.auto_recover_count = 0

        st = self._state(ok=False, now=now)
        self.bb.set(P_SAFETY, st, source=self.name, ts=now)

        if first:
            self._log(f"★ 强制停车（第 {self.forced_stops} 次）："
                      f"{'已发零速指令' if self.brake_on_fault else 'brake_on_fault=False，未发指令'}")
        if self.brake_on_fault:
            # ★ 每拍都发：底盘有 1 s 看门狗，安全态必须持续压住（不是"发一次就算"）
            self._safe_call(self.bridge.publish_brake)
            self.brakes += 1

    def _state(self, *, ok: bool, now: float) -> SafetyState:
        """组装 P_SAFETY。

        `since` 语义 = "本状态从何时开始"：
        - 已经处于故障（prev.ok is False）→ 保留原 since，不因每拍重算而跳动；
        - 首次进入故障 / 恢复 → 取当前时刻。
        """
        prev = self.bb.get(P_SAFETY)
        if isinstance(prev, SafetyState) and (prev.ok is False) == (ok is False):
            since = prev.since
        else:
            since = now
        return SafetyState(ok=ok, reasons=tuple(self._reasons) if not ok else (),
                           since=since, stops=self.forced_stops)

    # ------------------------------------------------------------------ #
    #  事件
    # ------------------------------------------------------------------ #
    def _pump_events(self) -> None:
        if self._sub is None:
            return
        for ev in self._sub.poll(32):
            if ev.type == EV_TASK_FAULT:
                self.on_task_fault(ev)

    def on_task_fault(self, ev) -> None:
        """收到 `EV_TASK_FAULT`（任务线程连续异常/启动失败）。"""
        source = str(ev.data.get("task") or ev.source or "?")
        where = ev.data.get("where", "?")
        error = ev.data.get("error", "")
        self.fault_events_seen += 1
        self._fault_reasons[source] = f"task_fault_event:{source} where={where} err={error}"
        self._log(f"收到 TASK_FAULT：{source}（{where}）err={error}")

    # ------------------------------------------------------------------ #
    #  小工具
    # ------------------------------------------------------------------ #
    @staticmethod
    def _finite(value) -> float:
        """转 float；None/NaN/inf 一律返回 inf（= 异常，交给调用方判）。"""
        try:
            out = float(value)
        except (TypeError, ValueError):
            return _INF
        return out if not math.isnan(out) else _INF

    @staticmethod
    def _safe_call(fn):
        """调桥接口并吞异常（返回值异常 → None；接口不存在 → None）。"""
        try:
            return fn()
        except Exception:
            return None
