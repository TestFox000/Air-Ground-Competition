# -*- coding: utf-8 -*-
"""WB-3 事件注入 API + 案例夹具运行器（卡 `WB-3-INCIDENTS-20261031` · tier-1）。

## 一句话

把「现场翻车**怎么发生的**」变成**可注入、可判读、可复跑**的东西：

```
PYTHONPATH=src/dzbrain python tools/wb_events.py --list-incidents
PYTHONPATH=src/dzbrain python tools/wb_events.py --scenario incident:I05 --arm realistic --json
PYTHONPATH=src/dzbrain python tools/wb_events.py --scenario incident:I05 --no-events --json   # 负例
```

## 本模块**不**造台架（这条决定了它与 wb-2 的关系）

台架（世界模型 × 生产任务线程 × 执行器链 × 虚拟时钟）**已经**在
`tools/sim_rig.py` + `tools/sim_world.py` + `tools/wb_rig/` 里了。本模块只做三件事：

| 做 | 不做 |
|---|---|
| 在**三个缝**上注入事件（观测 / 执行器 / 世界） | 不写第二份推进、不写第二份投影（判据①，`test_wb3_incidents.py` §1 有守卫） |
| 读夹具 JSON → 解析成一次 `sim_rig.run()` 的入参 | 不复制地图/路线快照（走 `wb_rig.scenarios` 的具名解析，取不到就报错） |
| 把"症状"写成**机器判据**并当场判红绿 | 不解析日志文本当指标（口径同 `wb_rig.counters`） |

## 三个缝（为什么是这三个、而不是改 `sim_rig.py`）

`sim_rig.run()` 内部自己构造了 `World`、`FakeBridge`、`Actuator`。卡面把
`tools/sim_rig.py` / `tools/sim_world.py` 的主体结构划在 wb-core 的写域（本卡只调用），
所以事件注入**不能**靠给 `run()` 加形参，只能在这三个对象上做**可撤销的包装**：

| 缝 | 真实现象 | 包装方式 | 为什么这个缝是对的 |
|---|---|---|---|
| **观测**（桥） | reset 单帧 yaw 跳变、位姿单帧突跳、位姿丢样 | `fakes.FakeBridge` 子类：`set_pose` 可吞帧、`pose()` 可改一帧 | 这三件事真车上就是"`/odom/localization` 报了这么一帧"，下游 `LocalizationTask` 一个字节都不该改 |
| **执行器** | 指令延迟阶跃 | `wb_rig.actuator.Actuator` 子类：`command()` 里改 `latency_s` | 延迟是**链路属性**，不是控制器决策 |
| **世界** | 里程比例突变（标定漂移/换轮径） | `sim_world.World` 子类：`step()` 前换 `injection` | 比例误差是**被控对象**属性；换臂档才是"注入"，突变是"现场发生的事" |

★ 每个包装都调用**父类原方法**（`super().step()` / `super().pose()` / `super().command()`），
  绝不自算位置：判据①（不得有第二份推进/投影）在 `test_wb3_incidents.py` §1 里对**本文件**
  也做源码级扫描。

## 事件语法（人可读、机可判）

```text
yaw-jump=35@8.0           # t=8.0 s 那一帧位姿的 yaw 突然 +35°（单帧，下一帧就复原）
pose-step=0.30@12.0       # t=12.0 s 那一帧位姿沿**当前航向的垂直方向**突跳 0.30 m
pose-step-x=-0.68@5.5     # 同上，但沿世界 x 轴（负号 = 向 -x）
drop=1.5@10.0             # t=10.0 s 起**连续 1.5 s 不再有任何位姿帧**（定位丢样）
scale-step=0.35@5.0       # t=5.0 s 起里程比例误差**变成** +35%（绝对值，不是增量）
latency-step=0.30@6.0     # t=6.0 s 起指令延迟变成 0.30 s（阶跃）
```

同一个夹具里可以写多条；`at_t` 指**虚拟时钟**秒（从 0 起算，与 `sim_rig --seconds` 同一时基）。

## 退出码

| 码 | 含义 |
|---|---|
| 0 | 症状按夹具预期**出现**（positive）或**消失**（`--no-events` 负例） |
| 1 | 症状与预期不符（夹具没在测它声称要测的东西） |
| 2 | 输入/夹具问题（夹具不存在、语法错、场景解析失败） |
| 4 | 台架自身失真（`sim_rig.run` 抛异常 / 任务 FAULT） |
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import pathlib
import sys
from contextlib import contextmanager
from dataclasses import dataclass, field

# --------------------------------------------------------------------------- #
#  sys.path：与 `tools/sim_rig.py` **同一套顺序**（src/dzbrain、tests 在前，tools 在后）
#  ★ 顺序是判据级的：`tools/dzbrain.py` 是个同名脚本，会把真包 `dzbrain` 整个遮住。
# --------------------------------------------------------------------------- #
_THIS = pathlib.Path(__file__).resolve()
_TOOLS = _THIS.parent
_REPO = _TOOLS.parent
_SRC = _REPO / "src" / "dzbrain"
_TESTS = _SRC / "tests"
sys.path[:] = [p for p in sys.path if os.path.abspath(p or os.curdir) != str(_TOOLS)]
sys.path.insert(0, str(_TESTS))
sys.path.insert(0, str(_SRC))
sys.path.append(str(_TOOLS))

import sim_rig                                                            # noqa: E402
import wb_rig.actuator as ACT                                             # noqa: E402
import wb_rig.arms as ARMS                                                # noqa: E402
import wb_rig.scenarios as SCN                                            # noqa: E402
from dzbrain.core import geometry as geo                                  # noqa: E402
from sim_world import World                                              # noqa: E402
from wb_rig.actuator import Actuator                                      # noqa: E402

#: 本模块的契约前缀（**区分于** `TRACKBENCH_RESULT_JSON` 与 `SIM_RESULT_JSON`：三个 schema 并存）
INCIDENT_PREFIX = "WB3_INCIDENT_JSON"
#: 夹具 schema 号。别的号一律拒绝（宁可红，不做兼容猜测）
FIXTURE_SCHEMA = "dzrobot-incident/1"
#: 夹具目录（相对本文件：tools/ → repo/src/dzbrain/tests/fixtures/incidents）
INCIDENTS_DIR = str(_REPO / "src" / "dzbrain" / "tests" / "fixtures" / "incidents")

RC_OK = 0
RC_MISMATCH = 1
RC_INPUT = 2
RC_FAULT = 4

#: 允许的事件类型（写错类型的夹具直接报错，不静默忽略 —— 静默忽略 = 夹具什么都没测）
#:
#: | kind | 卡面/现场说法 | 作用缝 | 值单位 |
#: |---|---|---|---|
#: | `yaw-jump` | reset 单帧 yaw 跳变 | 观测（桥 `pose()`） | 度（正 = 逆时针） |
#: | `pose-step` | 单帧位姿突跳（沿**航向垂直**方向） | 观测 | 米 |
#: | `pose-step-x` / `pose-step-y` | 同上，沿世界轴 | 观测 | 米（可负） |
#: | `drop` | 位姿超时丢样（卡面写 `drop-s`） | 观测（桥 `set_pose` 吞帧） | **秒**（持续时长） |
#: | `scale-step` | 里程比例突变 | 世界（`World.injection`） | 比例（0.45 = +45%） |
#: | `latency-step` | 指令延迟阶跃 | 执行器（`Actuator.latency_s`） | 秒 |
EVENT_KINDS: tuple[str, ...] = ("yaw-jump", "pose-step", "pose-step-x", "pose-step-y",
                                "drop", "scale-step", "latency-step")

#: 卡面简写（不带 `@t`）的默认时刻（秒）。取第 8 秒：realistic 档那时已经跑到第 3~4 段，
#: 既不是起步（起步期任何注入都会被启动宽限吞掉），也不是尾段（尾段的症状无从区分）。
DEFAULT_AT_T = 8.0


# --------------------------------------------------------------------------- #
#  事件
# --------------------------------------------------------------------------- #
class EventError(ValueError):
    """事件语法错误 / 未登记类型。**必须显式抛**，不许静默丢弃。"""


@dataclass(frozen=True)
class Event:
    """一条已解析的事件。`at_t` 是**虚拟时钟秒**（从 0 起算）。"""

    kind: str
    at_t: float
    value: float
    raw: str = ""
    #: 原串里**是否显式给了 `@t`**（没给 = 用了卡面简写，由 `default_at_t` 补）
    had_at: bool = True

    @staticmethod
    def parse(text: str, *, default_at_t: float = DEFAULT_AT_T) -> "Event":
        """`kind=value@t` → `Event`。语法错误抛 `EventError`（带原串，便于夹具作者定位）。

        ★ 卡面三种简写（**不带 `@t`**，用 `default_at_t` 补时刻）：
            --inject yaw-jump=35      → yaw-jump=35@8.0
            --inject drop-s=1.5       → drop=1.5@8.0
            --inject pose-step=0.30   → pose-step=0.30@8.0
          `drop-s` 是卡面的写法（"秒"），内部存的 kind 是 `drop`；别名在这里吸收，
          不让调用方记两套名字。
        """
        raw = str(text or "").strip()
        if not raw:
            raise EventError("空事件")
        if "=" not in raw:
            raise EventError(f"事件语法应为 `kind=value[@t]`，收到 {raw!r}")
        kind, _, rest = raw.partition("=")
        kind = kind.strip()
        if kind == "drop-s":                      # 卡面写法 → 内部 kind
            kind = "drop"
        if "@" in rest:
            value_s, _, t_s = rest.partition("@")
            try:
                at_t = float(t_s)
            except ValueError as exc:
                raise EventError(f"事件的时刻不是数字：{raw!r}（{exc}）") from exc
        else:
            value_s, at_t = rest, float(default_at_t)
        if kind not in EVENT_KINDS:
            raise EventError(f"未登记的事件类型 {kind!r}（已登记 {list(EVENT_KINDS)}）：{raw!r}")
        try:
            value = float(value_s)
        except ValueError as exc:
            raise EventError(f"事件的 value 不是数字：{raw!r}（{exc}）") from exc
        if at_t < 0.0:
            raise EventError(f"事件时刻不许为负：{raw!r}")
        if kind in ("drop", "scale-step", "latency-step") and value < 0.0:
            raise EventError(f"{kind} 的值不许为负：{raw!r}")
        return Event(kind=kind, at_t=at_t, value=value, raw=raw, had_at=("@" in rest))

    def describe(self) -> str:
        return self.raw or f"{self.kind}={self.value}@{self.at_t}"


def parse_events(items) -> list[Event]:
    """列表 → 事件表（按 `at_t` 升序；同刻按书写顺序，保证确定性）。"""
    out = [Event.parse(x) for x in (items or [])]
    out.sort(key=lambda e: (e.at_t,))
    return out


# --------------------------------------------------------------------------- #
#  驱动器：把事件表变成"对三个缝的副作用"
# --------------------------------------------------------------------------- #
@dataclass
class EventDriver:
    """事件驱动器（**只作用于观测/执行器/被控对象，不作用于决策**）。

    ★ 时间基：`at_t` 是**相对秒**（从台架开跑那一刻算）。台架的 `VirtualClock` 从
      1000.0 s 起（`wb_rig.sched.VirtualClock.t0`），所以**不能**拿绝对时刻去比 ——
      第一版就栽在这里：`yaw-jump=35@8.0` 在 t=0 就判"到点"，全部事件挤在第一拍生效，
      于是每条夹具看起来都能复现（其实测的是"我把所有注入一次性砸在起步"）。
      ⇒ `t0` 由 `install()` 在第一次观测时钉死，之后一律比 `now - t0`。

    ★ 幂等性：每条事件**只生效一次**（按**下标**记账，不按字符串 —— 两条同文事件必须
      各自生效）。这样"单帧跳变"真的是单帧，而不是"从 t 开始一直偏着"。
    """

    events: list[Event] = field(default_factory=list)
    #: 时间基（虚拟秒）。`None` = 还没钉死，第一帧观测时取当时的 `now`
    t0: float | None = None
    #: 已生效事件（`(相对秒, 原始串)`；进契约行，让人看到"这一轮到底注入了什么、什么时候"）
    fired: list[tuple[float, str]] = field(default_factory=list)
    _fired_idx: set[int] = field(default_factory=set)
    #: 当前生效的里程比例误差 / 指令延迟（`scale-step` / `latency-step` 的落点）
    scale_err: float | None = None
    latency_s: float | None = None
    #: 丢样窗口：[start, end]（**绝对虚拟秒**）
    drop_windows: list[tuple[float, float]] = field(default_factory=list)
    #: 待生效的**单帧**位姿变换（按事件下标排队，由桥 `pose()` 取走一条）
    pending_pose: dict[int, tuple[str, float]] = field(default_factory=dict)

    # ------------------------------ 时间推进 ------------------------------ #
    def rel(self, now: float) -> float:
        """把绝对虚拟时钟换算成"从开跑算起"的相对秒。"""
        if self.t0 is None:
            self.t0 = float(now)
        return float(now) - float(self.t0)

    def _take(self, now: float, kinds: tuple[str, ...]) -> list[tuple[int, Event]]:
        """取走"到点且属于 `kinds`"的事件（每条只取一次）。"""
        t = self.rel(now)
        due: list[tuple[int, Event]] = []
        for i, ev in enumerate(self.events):
            if ev.kind not in kinds or i in self._fired_idx:
                continue
            if ev.at_t > t + 1e-9:
                continue
            due.append((i, ev))
        return due

    def _mark(self, i: int, ev: Event, now: float) -> None:
        self._fired_idx.add(i)
        self.fired.append((round(self.rel(now), 6), ev.describe()))

    @property
    def fired_text(self) -> list[str]:
        return [text for _t, text in self.fired]

    @property
    def active(self) -> bool:
        return bool(self.events)

    # ------------------------------ 世界缝 ------------------------------ #
    def on_world(self, world: World, now: float) -> None:
        """`World.step()` 之前调用：里程比例突变。"""
        for i, ev in self._take(now, ("scale-step",)):
            self._mark(i, ev, now)
            self.scale_err = float(ev.value)
            world.injection = dataclasses.replace(world.injection,
                                                  speed_scale_err=float(ev.value))

    # ------------------------------ 执行器缝 ------------------------------ #
    def on_command(self, act: Actuator, now: float) -> None:
        """`Actuator.command()` 里调用：指令延迟阶跃。"""
        for i, ev in self._take(now, ("latency-step",)):
            self._mark(i, ev, now)
            self.latency_s = float(ev.value)
            act.latency_s = float(ev.value)

    # ------------------------------ 观测缝 ------------------------------ #
    def on_odom(self, world: World, now: float) -> None:
        """`World.odom_frame()` 之后调用：把"单帧位姿事件"排进队、把丢样窗口记下来。"""
        for i, ev in self._take(now, ("yaw-jump", "pose-step", "pose-step-x", "pose-step-y")):
            self._mark(i, ev, now)
            self.pending_pose[i] = (ev.kind, float(ev.value))
        for i, ev in self._take(now, ("drop",)):
            self._mark(i, ev, now)
            self.drop_windows.append((float(now), float(now) + float(ev.value)))

    def suppress_pose(self, now: float) -> bool:
        """当前是否处于丢样窗口（`FakeBridge.set_pose` 据此吞帧）。"""
        for start, end in self.drop_windows:
            if start <= now + 1e-9 < end:
                return True
        return False

    def take_pose_patch(self, pose, now: float):
        """给桥 `pose()` 用：取走**一条**待生效的单帧变换并作用到这一帧。"""
        if not self.pending_pose:
            return pose
        idx = sorted(self.pending_pose)[0]
        kind, value = self.pending_pose.pop(idx)
        if kind == "yaw-jump":
            return geo.Pose(pose.x, pose.y, geo.normalize_angle(pose.yaw + geo.rad(value)))
        if kind == "pose-step":                # 沿**当前航向的垂直方向**（重锚/横漂的样子）
            perpendicular = pose.yaw + _PI_OVER_2
            dx = value * _cos(perpendicular)
            dy = value * _sin(perpendicular)
            return geo.Pose(pose.x + dx, pose.y + dy, pose.yaw)
        if kind == "pose-step-x":
            return geo.Pose(pose.x + value, pose.y, pose.yaw)
        if kind == "pose-step-y":
            return geo.Pose(pose.x, pose.y + value, pose.yaw)
        return pose

    # ------------------------------ 诊断 ------------------------------ #
    def describe(self) -> str:
        if not self.events:
            return "无事件注入"
        return "事件：" + "；".join(e.describe() for e in self.events)


_PI_OVER_2 = 1.5707963267948966


def _cos(v: float) -> float:
    import math
    return math.cos(v)


def _sin(v: float) -> float:
    import math
    return math.sin(v)


# --------------------------------------------------------------------------- #
#  三个缝的包装（**可撤销**：退出上下文即还原，跑完一个夹具不留痕）
# --------------------------------------------------------------------------- #
#: 当前生效的驱动器（包装类看到它）。★ 只在 `install()` 上下文内非空；
#:   用模块级单例而不是给包装类塞构造参数，是因为 `sim_rig.run()` 内部自己 `new` 对象，
#:   我们**不许**改它的构造签名（那是 wb-core 的写域）。
_ACTIVE_DRIVER: EventDriver | None = None


class _EventWorld(World):
    """`World` 子类：只加**两个调用点**，物理/推进全部走父类。"""

    def step(self, dt: float, cmd_v: float, cmd_steer: float, now: float):
        drv = _ACTIVE_DRIVER
        if drv is not None:
            drv.on_world(self, now)
        return super().step(dt, cmd_v, cmd_steer, now)

    def odom_frame(self, now: float, *, rng_refresh: bool = False):
        sample = super().odom_frame(now, rng_refresh=rng_refresh)
        drv = _ACTIVE_DRIVER
        if drv is not None:
            drv.on_odom(self, now)
        return sample

    def take_pose_patch(self, pose, now: float):
        """桥拿不到驱动器时，走世界（两条路径都在，避免依赖构造顺序）。"""
        drv = _ACTIVE_DRIVER
        return drv.take_pose_patch(pose, now) if drv is not None else pose


class _EventActuator(Actuator):
    """`Actuator` 子类：只在 `command()` 入口加一个"改延迟"的调用点。"""

    def command(self, now: float, v: float, steer_deg: float) -> None:
        drv = _ACTIVE_DRIVER
        if drv is not None:
            drv.on_command(self, now)
        return super().command(now, v, steer_deg)


@contextmanager
def install(driver: EventDriver | None):
    """把三个缝装上（退出即还原）。`driver=None` = 装一个空驱动器（等价不注入，用于负例）。"""
    global _ACTIVE_DRIVER
    import fakes
    prev_driver = _ACTIVE_DRIVER
    prev_world = sim_rig.World
    prev_bridge = fakes.FakeBridge

    class _Bridge(prev_bridge):                    # noqa: N801  局部类，只为这次上下文
        """`FakeBridge` 子类：`set_pose` 可吞帧（丢样）、`pose()` 可改**一帧**（跳变）。"""

        def set_pose(self, x: float, y: float, yaw: float = 0.0) -> None:
            drv = _ACTIVE_DRIVER
            if drv is not None and drv.suppress_pose(self.clock.now()):
                return                             # 丢样窗口：真机上就是"这一帧没来"
            return super().set_pose(x, y, yaw)

        def pose(self):
            p = super().pose()
            drv = _ACTIVE_DRIVER
            if drv is None or p is None:
                return p
            return drv.take_pose_patch(p, self.clock.now())

    _ACTIVE_DRIVER = driver if driver is not None else EventDriver()
    sim_rig.World = _EventWorld
    fakes.FakeBridge = _Bridge
    try:
        yield
    finally:
        sim_rig.World = prev_world
        fakes.FakeBridge = prev_bridge
        _ACTIVE_DRIVER = prev_driver


# --------------------------------------------------------------------------- #
#  夹具：加载 + 校验
# --------------------------------------------------------------------------- #
class FixtureError(ValueError):
    """夹具缺失/非法。显式抛（不许回落默认图、不许猜）。"""


@dataclass(frozen=True)
class Incident:
    """一个案例夹具（**全部字段来自 JSON**，本模块不写死任何案例内容）。"""

    id: str
    slug: str
    title: str
    symptom: str
    root_cause: str
    sources: tuple[str, ...]
    map_path: str
    route_path: str | None
    scenario_name: str | None
    arm: str
    seconds: float
    events: tuple[str, ...]
    arm_patch: dict
    derived: dict
    criteria: tuple[dict, ...]
    negative: dict

    def describe(self) -> str:
        ev = ("；|".join(self.events)) if self.events else "（无事件）"
        return (f"[{self.id}] {self.title}\n"
                f"     症状：{self.symptom}\n"
                f"     根因：{self.root_cause}\n"
                f"     出处：{'；'.join(self.sources)}\n"
                f"     场景：{self.scenario_name or self.map_path} / 档 {self.arm} / "
                f"{self.seconds:g}s | {ev}")


def incidents_dir() -> str:
    return INCIDENTS_DIR


def _load_json(path: str) -> dict:
    try:
        with open(path, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError) as exc:
        raise FixtureError(f"夹具读不动 {path}（{type(exc).__name__}: {exc}）") from exc


def _resolve_incident_path(rel: str) -> str:
    """夹具内的相对路径 → 绝对路径，并**保证仍在夹具目录内**（防 `../` 越界）。"""
    base = os.path.abspath(INCIDENTS_DIR)
    cand = os.path.abspath(os.path.join(base, str(rel)))
    if os.path.commonpath([cand, base]) != base:
        raise FixtureError(f"夹具路径越界（不在 {base} 内）: {rel!r} → {cand}")
    if not os.path.isfile(cand):
        raise FixtureError(f"夹具文件不存在: {cand}（登记的是 {rel!r}）")
    return cand


def load(name: str) -> Incident:
    """`I05` / `I05-PIVOT-LOOP` / `incident:I05` → `Incident`。未知 ⇒ `FixtureError`。"""
    key = str(name or "").strip()
    if key.lower().startswith("incident:"):
        key = key.split(":", 1)[1].strip()
    if not key:
        raise FixtureError("夹具名是空的")
    base = os.path.abspath(INCIDENTS_DIR)
    if not os.path.isdir(base):
        raise FixtureError(f"夹具目录不存在: {base}")
    files = sorted(f for f in os.listdir(base) if f.endswith(".json"))
    hit = None
    for fn in files:
        doc = _load_json(os.path.join(base, fn))
        if str(doc.get("schema") or "") != FIXTURE_SCHEMA:
            raise FixtureError(f"夹具 schema 不符：期望 {FIXTURE_SCHEMA!r}，"
                               f"实得 {doc.get('schema')!r}（{fn}）")
        ident = str(doc.get("id") or "").strip().upper()
        slug = str(doc.get("slug") or "").strip().upper()
        if key.upper() in (ident, slug):
            hit = doc
            break
    if hit is None:
        raise FixtureError(f"未知夹具 {name!r}；已登记 {[s for s in registered_slugs()]}")

    scenario_name = hit.get("scenario")
    if scenario_name:
        sc = SCN.resolve(str(scenario_name))         # 具名场景：**复用** WB-1 的快照登记
        map_path, route_path = sc.map_path, sc.route_path
    else:
        map_path = _resolve_incident_path(str(hit.get("map") or ""))
        route_rel = hit.get("route")
        route_path = _resolve_incident_path(str(route_rel)) if route_rel else None
    if not map_path:
        raise FixtureError(f"夹具 {key!r} 没有可用地图（scenario 与 map 都为空）")

    arm = str(hit.get("arm") or "realistic")
    if arm not in ARMS.ARMS:
        raise FixtureError(f"夹具 {key!r} 的 arm={arm!r} 未登记（可选 {sorted(ARMS.ARMS)}）")
    events = tuple(str(x) for x in (hit.get("events") or []))
    for ev in events:                                # 早失败：夹具里写坏的事件不要等到第 40 s
        Event.parse(ev)
    crit = hit.get("criteria") or []
    if not isinstance(crit, list) or not crit:
        raise FixtureError(f"夹具 {key!r} 没有机器判据（criteria 为空 ⇒ 无法判红绿）")
    return Incident(
        id=str(hit.get("id") or ""), slug=str(hit.get("slug") or ""),
        title=str(hit.get("title") or ""), symptom=str(hit.get("symptom") or ""),
        root_cause=str(hit.get("root_cause") or ""),
        sources=tuple(str(x) for x in (hit.get("sources") or [])),
        map_path=str(map_path), route_path=(str(route_path) if route_path else None),
        scenario_name=(str(scenario_name) if scenario_name else None),
        arm=arm, seconds=float(hit.get("seconds") or 120.0), events=events,
        arm_patch=dict(hit.get("arm_patch") or {}),
        derived=dict(hit.get("derived") or {}),
        criteria=tuple(crit),
        negative=dict(hit.get("negative") or {}),
    )


def registered_slugs() -> list[str]:
    """已登记夹具的 slug（排序固定，便于 `--list-incidents` 与测试断言）。"""
    base = os.path.abspath(INCIDENTS_DIR)
    if not os.path.isdir(base):
        return []
    out = []
    for fn in sorted(f for f in os.listdir(base) if f.endswith(".json")):
        doc = _load_json(os.path.join(base, fn))
        out.append(str(doc.get("slug") or doc.get("id") or fn))
    return out


def all_incidents() -> list[Incident]:
    return [load(s) for s in registered_slugs()]


# --------------------------------------------------------------------------- #
#  派生量（从逐拍行算；夹具在 `derived` 里声明它要哪几个）
# --------------------------------------------------------------------------- #
def _derive(rows: list[dict], spec: dict) -> dict[str, object]:
    """按夹具声明计算派生量。**未登记 kind 一律抛**（宁可红，不静默给 0）。

    ★ 为什么这些量要从**逐拍行**算而不是从指标键：现场那 16 条里有好几条的症状是
      **时间序列形态**（"s 冻住但车还在动"、"每拍翻转 N 次"、"一步 +12.8 m"），
      它们在单点指标里根本看不见 —— 单点 RMS 会把"卡在同一处 40 s"平均成一个好看的数。
    """
    out: dict[str, object] = {"rows": len(rows)}
    for name, cfg in (spec or {}).items():
        kind = str((cfg or {}).get("kind") or "")
        if kind == "xt_in_band":                       # 横向误差落在 [lo,hi] 的拍数
            lo, hi = float(cfg.get("lo", -1e9)), float(cfg.get("hi", 1e9))
            out[name] = float(sum(1 for r in rows if lo <= float(r["xt"]) <= hi))
        elif kind == "min_dist_cell":                  # 真值离某个格心的最小距离
            cx, cy = geo.block_to_meters(str(cfg["cell"]).upper())
            out[name] = (min((abs(float(r["truth_x"]) - cx) ** 2
                              + abs(float(r["truth_y"]) - cy) ** 2) ** 0.5 for r in rows)
                         if rows else 1e9)
        elif kind == "max_dist_cell":                  # 真值离某个格心的最大距离
            cx, cy = geo.block_to_meters(str(cfg["cell"]).upper())
            out[name] = (max((abs(float(r["truth_x"]) - cx) ** 2
                              + abs(float(r["truth_y"]) - cy) ** 2) ** 0.5 for r in rows)
                         if rows else 0.0)
        elif kind == "ticks_phase":                    # 处于某阶段的拍数
            ph = str(cfg["phase"])
            out[name] = float(sum(1 for r in rows if str(r["phase"]) == ph))
        elif kind == "max_v_cmd":                      # 下发速度峰值（限速被绕过的观测量）
            out[name] = max((abs(float(r["cmd_v"])) for r in rows), default=0.0)
        elif kind == "cmd_negative_eaten":              # 负速度被吃掉的直接观测量
            out[name] = float(sum(1 for r in rows
                                  if float(r["cmd_v"]) < -1e-9 and float(r["exec_v"]) >= -1e-9))
        elif kind == "s_frozen_moving":                 # ★"s 冻住但车还在动"（I01/I03）
            #   `road_moved_m` 扣掉"车真的停住"的情形：只看真值位移 > `min_move_m` 的拍。
            min_move = float(cfg.get("min_move_m", 0.010))
            min_ds = float(cfg.get("min_ds_m", 0.002))
            out[name] = float(sum(
                1 for a, b in zip(rows, rows[1:])
                if (abs(float(b["truth_x"]) - float(a["truth_x"])) ** 2
                    + abs(float(b["truth_y"]) - float(a["truth_y"])) ** 2) ** 0.5 >= min_move
                and abs(float(b["s"]) - float(a["s"])) < min_ds))
        elif kind == "s_jump_max":                      # ★单拍弧长最大跳变（I02 的 +12.824 m）
            out[name] = max((abs(float(b["s"]) - float(a["s"]))
                             for a, b in zip(rows, rows[1:])), default=0.0)
        elif kind == "cmd_flip":                        # ★命令符号翻转次数（I04 的 601 次/拍）
            flips = 0
            prev = 0.0
            for r in rows:
                v = float(r["cmd_v"])
                if abs(v) < 1e-9:
                    continue
                if prev != 0.0 and (v > 0.0) != (prev > 0.0):
                    flips += 1
                prev = v
            out[name] = float(flips)
        elif kind == "belief_jump_max":                 # ★信念单拍最大位移（I11 的 0.303 m）
            out[name] = max((((float(b["belief_x"]) - float(a["belief_x"])) ** 2
                              + (float(b["belief_y"]) - float(a["belief_y"])) ** 2) ** 0.5
                             for a, b in zip(rows, rows[1:]) if _both_finite(a, b)),
                            default=0.0)
        elif kind == "per_second":                      # 某计数器 / 时长（速率化）
            src = str(cfg["metric"])
            dur = float(cfg.get("duration_s") or 0.0) or 1.0
            out[name] = float(out.get(src) or 0.0) / dur
        else:
            raise FixtureError(f"未登记的派生量 kind={kind!r}（夹具 {name!r}）")
    return out


def _both_finite(a: dict, b: dict) -> bool:
    import math
    for k in ("belief_x", "belief_y"):
        for r in (a, b):
            v = r.get(k)
            if v is None or (isinstance(v, float) and not math.isfinite(v)):
                return False
    return True


def _too_big(value) -> bool:
    import math
    return isinstance(value, float) and not math.isfinite(value)


_OPS = {
    "lt": lambda a, b: a < b,
    "le": lambda a, b: a <= b,
    "gt": lambda a, b: a > b,
    "ge": lambda a, b: a >= b,
    "eq": lambda a, b: a == b,
}


def _judge(crit: dict, values: dict) -> dict:
    """判一条判据。缺失的键**判红**（不是跳过）—— 缺指标等于测不了。

    ★ 两类判据：
      · 数值类（`phase_final` 除外）：走 `_OPS` 的数值比较；
      · **字符串类**（`phase_final` 这种枚举）：只允许 `eq`，逐字比较。
        为什么不让 `gt` 也能比字符串：`"SAFE_STOP" > "DRIVE"` 在 Python 里是**字典序**，
        能"通过"但毫无意义 —— 那是把判据写成彩票。
    """
    name = str(crit.get("name") or crit.get("metric") or "")
    metric = str(crit.get("metric") or "")
    op = str(crit.get("op") or "")
    target = crit.get("value")
    actual = values.get(metric)
    if metric not in values:
        return {"name": name, "metric": metric, "op": op, "value": target,
                "actual": None, "pass": False, "why": "指标缺失（台架没给这个键）"}
    if op not in _OPS:
        return {"name": name, "metric": metric, "op": op, "value": target,
                "actual": actual, "pass": False, "why": f"未登记的比较符 {op!r}"}
    if isinstance(actual, str) or isinstance(target, str):
        if op != "eq":
            return {"name": name, "metric": metric, "op": op, "value": target,
                    "actual": actual, "pass": False,
                    "why": "字符串指标只允许 eq（字典序比较会给出无意义的通过）"}
        ok = str(actual) == str(target)
    else:
        try:
            ok = bool(_OPS[op](float(actual), float(target)))
        except (TypeError, ValueError):
            return {"name": name, "metric": metric, "op": op, "value": target,
                    "actual": actual, "pass": False, "why": "非数值，无法比较"}
    return {"name": name, "metric": metric, "op": op, "value": target,
            "actual": actual, "pass": ok}


# --------------------------------------------------------------------------- #
#  跑一个夹具
# --------------------------------------------------------------------------- #
def _arm_for(inc: Incident, *, arm_name: str | None = None) -> ARMS.ArmProfile:
    base = ARMS.get(arm_name or inc.arm)
    if not inc.arm_patch:
        return base
    known = {f.name for f in dataclasses.fields(ARMS.ArmProfile)}
    bad = [k for k in inc.arm_patch if k not in known]
    if bad:
        raise FixtureError(f"夹具 {inc.id} 的 arm_patch 有未登记字段 {bad}（可选 {sorted(known)}）")
    return dataclasses.replace(base, name=f"{base.name}+{inc.id}", **inc.arm_patch)


def _ad_hoc_incident(*, base_scenario: str) -> Incident:
    """只给 `--inject` 时的**自建夹具**：拿一个具名场景，判据 = "事件全部按点生效"。

    ★ 这不违反"夹具必须来自现场案例"：卡面把 `--inject` 三个入口列为**API 可用性**
      验收项（`yaw-jump=<deg>` / `drop-s=<s>` / `pose-step=<m>`），它测的是**注入 API
      本身**能不能生效，不是某条案例的症状。案例夹具走 `--scenario incident:<ID>`，
      它们各自的判据来自现场文档 —— 两条路分开，谁也不替谁背书。
    """
    sc = SCN.resolve(base_scenario)
    return Incident(
        id="INJECT", slug="INJECT-API", title="事件注入 API 冒烟",
        symptom="（API 冒烟，不是现场案例）", root_cause="（同上）",
        sources=("卡面 `--inject` 三个入口",),
        map_path=sc.map_path, route_path=sc.route_path,
        scenario_name=base_scenario, arm="ideal", seconds=30.0, events=(),
        arm_patch={}, derived={},
        criteria=({"name": "事件全部按点生效", "metric": "events_fired_n",
                   "op": "eq", "value": 1.0},),
        negative={})


def run_incident(inc: Incident, *, arm_name: str | None = None,
                 events_on: bool = True, seconds: float | None = None,
                 seed: int = 20261031, quiet: bool = True,
                 patch: dict | None = None, scenario_name: str | None = None) -> dict:
    """跑一次夹具，返回结果 + 判据 + 契约行（JSON 可序列化）。

    `patch` / `scenario_name` 是给**负例臂**用的：负例要能换一个注入档或换一张图跑，
    而不是只能"关事件"——因为有几条案例（几何切角类）本来就不是事件造成的，
    它们的负例必须是"**改掉那一条几何/那一个开关**"（见各夹具 `negative` 段）。
    """
    arm = _arm_for(inc, arm_name=arm_name)
    if patch:
        known = {f.name for f in dataclasses.fields(ARMS.ArmProfile)}
        bad = [k for k in patch if k not in known]
        if bad:
            raise FixtureError(f"负例 patch 有未登记字段 {bad}（可选 {sorted(known)}）")
        arm = dataclasses.replace(arm, **patch)
    scen_name = scenario_name or inc.scenario_name
    scenario = SCN.resolve(scen_name) if scen_name else None
    if scenario is None:
        # 夹具自带的图/路线：仍走 `wb_rig.scenarios.Scenario`（同一套结构，不另建类型）
        scenario = SCN.Scenario(name=f"incident:{inc.id}", map_path=inc.map_path,
                                route_path=inc.route_path,
                                origin=f"WB-3 夹具 {inc.slug}", ledger="WB-3")
    if not scenario.route_path:
        raise FixtureError(f"夹具 {inc.id} 没有路线快照 ⇒ 闭环台架跑不了（只做布局模式是 wb-4）")
    driver = EventDriver(events=parse_events(inc.events)) if events_on else EventDriver()
    # ★ 执行器用**事件版**子类（`latency-step` 才能生效）；默认延迟与台架同源
    act = _EventActuator(latency_s=ACT.DEFAULT_LATENCY_S)
    out_dir = str(_REPO / "artifacts" / "wb3_incidents"
                  / f"{inc.slug}_{arm.name}_{'inject' if events_on else 'negative'}")
    with install(driver):
        result = sim_rig.run(scenario=scenario, arm=arm,
                             seconds=float(seconds if seconds is not None else inc.seconds),
                             seed=int(seed), actuator=act, field_modeled=True,
                             out_dir=out_dir,
                             log=(lambda msg: None) if quiet else None)
    m = result["metrics"]
    values: dict[str, object] = dict(m.values)
    values.update(_derive(result["rows"], inc.derived))
    # ★ `events_fired_n` 是注入 API 的**直接观测量**（几条事件真的按点生效了）：
    #   它不在生产指标里，只能由驱动器自己给。缺它的话"API 可用"这条验收就无从机器判。
    values["events_fired_n"] = float(len(driver.fired))
    crit = [_judge(c, values) for c in inc.criteria]
    reproduced = all(c["pass"] for c in crit)
    payload = {
        "schema": "dzrobot-wb3-incident/1",
        "incident": inc.id, "slug": inc.slug, "title": inc.title,
        "arm": arm.name, "events_on": bool(events_on),
        "events": [e.describe() for e in driver.events],
        "events_fired": list(driver.fired_text),
        "events_fired_at": [[t, txt] for t, txt in driver.fired],
        "t0_s": driver.t0,
        "seconds": float(inc.seconds if seconds is None else seconds),
        "scenario": {"name": scenario.name, "map": os.path.basename(scenario.map_path),
                     "route": os.path.basename(scenario.route_path or "")},
        "symptom_seen": bool(reproduced),
        "criteria": crit,
        "values": {k: (float(v) if isinstance(v, (int, float)) and not isinstance(v, bool)
                       else v)
                   for k, v in values.items() if not isinstance(v, (list, tuple, dict))},
        "faulty": result["faulty"], "phase": result["phase"],
        "negative": None,
    }
    return {"payload_payload": payload, "rows": result["rows"], "metrics": m,
            "fired": list(driver.fired_text), "reproduced": reproduced,
            "criteria": crit, "values": values, "faulty": result["faulty"]}


def contract_line(payload: dict) -> str:
    return f"{INCIDENT_PREFIX} " + json.dumps(payload, ensure_ascii=False,
                                              sort_keys=True, default=str)


# --------------------------------------------------------------------------- #
#  CLI
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="wb_events",
        description="WB-3 现场翻车案例夹具运行器 + 事件注入 API",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--scenario", default=None,
                    help="夹具：`incident:I05` / `I05` / `I05-PIVOT-LOOP`；"
                         "给了 --inject 时可省略（走 `--base-scenario` 的裸台架）")
    ap.add_argument("--base-scenario", default="field7",
                    help="只做 --inject 时用的具名场景（默认 field7）")
    ap.add_argument("--inject", default=None, action="append",
                    help=f"卡面简写（可重复）：`yaw-jump=<deg>` / `drop-s=<s>` / "
                         f"`pose-step=<m>` / `scale-step=<ratio>` / `latency-step=<s>`；"
                         f"不带 `@t` 时按 {DEFAULT_AT_T:g}s 生效")
    ap.add_argument("--arm", default=None, choices=sorted(ARMS.ARMS),
                    help="覆盖夹具自带的注入档（默认用夹具写的）")
    ap.add_argument("--seconds", type=float, default=None, help="覆盖仿真时长")
    ap.add_argument("--seed", type=int, default=20261031, help="噪声种子")
    ap.add_argument("--no-events", dest="no_events", action="store_true",
                    help="★负例：装空驱动器 —— 事件一条不生效（症状必须消失）")
    ap.add_argument("--no-negative", dest="no_negative", action="store_true",
                    help="只跑正例，不自动跑夹具声明的负例")
    ap.add_argument("--json", dest="json_line", action="store_true",
                    help=f"末行追加 `{INCIDENT_PREFIX} {{...}}` 契约行")
    ap.add_argument("--list-incidents", action="store_true", help="列出全部夹具后退出")
    ap.add_argument("--event", default=None, action="append",
                    help="临时加一条事件（可重复）；给了它就**不用**夹具自带事件表")
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.list_incidents:
        print(f"WB-3 案例夹具（{INCIDENTS_DIR}）：")
        for slug in registered_slugs():
            try:
                print("  " + load(slug).describe())
            except (FixtureError, SCN.ScenarioError) as exc:
                print(f"  [{slug}] ❌ 夹具不可用：{exc}")
        print(f"  共 {len(registered_slugs())} 条")
        return RC_OK
    if not args.scenario and not args.inject:
        build_parser().print_help()
        return RC_INPUT

    try:
        if args.scenario:
            inc = load(args.scenario)
        else:
            inc = _ad_hoc_incident(base_scenario=args.base_scenario)
        if args.event:                                    # `--event` 全串（带 @t）
            inc = dataclasses.replace(inc, events=tuple(args.event))
        elif args.inject:                                 # `--inject` 卡面简写
            inc = dataclasses.replace(
                inc, events=tuple(Event.parse(x) for x in args.inject))
            inc = dataclasses.replace(inc, events=tuple(e.raw for e in inc.events))
            # 自建夹具的判据是"事件真的按点生效了"，而不是某条案例的症状
            if not args.scenario:
                inc = dataclasses.replace(inc, criteria=(
                    {"name": "事件全部按点生效", "metric": "events_fired_n", "op": "eq",
                     "value": float(len(args.inject))},))
        arm_name = args.arm
    except (FixtureError, SCN.ScenarioError, EventError) as exc:
        print(f"❌ 输入失败：{exc}", file=sys.stderr)
        return RC_INPUT

    if args.inject and args.scenario:
        # 夹具已有判据，`--inject` 只是替换事件表 ⇒ 判据照旧
        pass

    try:
        pos = run_incident(inc, arm_name=arm_name, events_on=not args.no_events,
                           seconds=args.seconds, seed=args.seed)
    except (FixtureError, SCN.ScenarioError, KeyError) as exc:
        print(f"❌ 输入失败：{exc}", file=sys.stderr)
        return RC_INPUT
    except Exception as exc:                              # noqa: BLE001  台架自身失真
        print(f"❌ 台架异常（不许当结果用）：{type(exc).__name__}: {exc}", file=sys.stderr)
        import traceback
        traceback.print_exc(limit=3)
        return RC_FAULT

    payload = pos["payload_payload"]
    negative = None
    rc = RC_OK
    if pos["faulty"]:
        print(f"❌ 生产任务 FAULT：{pos['faulty']}", file=sys.stderr)
        rc = RC_FAULT
    # ★ 判正例的**唯一**开关是"这一轮是不是故意关掉注入的负例"。但**自建夹具**
    #   （`--inject`，slug=INJECT-API）本身就是正例：它的判据是"事件按点生效"，
    #   用 `--no-events` 跑它就是在**做**那条负例，必须照判、必须红。
    ad_hoc = inc.slug == "INJECT-API"
    if not args.no_events or ad_hoc:
        if not pos["reproduced"]:
            rc = rc or RC_MISMATCH
        if not args.no_negative and inc.negative and not args.event:
            try:
                neg = run_incident(inc,
                                   arm_name=inc.negative.get("arm") or inc.arm,
                                   events_on=bool(inc.negative.get("events_on", False)),
                                   seconds=args.seconds, seed=args.seed,
                                   patch=dict(inc.negative.get("arm_patch") or {}) or None,
                                   scenario_name=inc.negative.get("scenario"))
            except Exception as exc:                      # noqa: BLE001
                print(f"❌ 负例跑不动：{type(exc).__name__}: {exc}", file=sys.stderr)
                return RC_FAULT
            negative = {"arm": neg["payload_payload"]["arm"],
                        "events_on": neg["payload_payload"]["events_on"],
                        "scenario": neg["payload_payload"]["scenario"]["name"],
                        "desc": str(inc.negative.get("desc") or ""),
                        "symptom_seen": bool(neg["reproduced"]),
                        "values": {c["metric"]: neg["values"].get(c["metric"])
                                   for c in pos["criteria"]}}
            if neg["reproduced"]:
                # 负例也复现 ⇒ 症状不是注入造成的（这正是"夹具在测别的东西"）
                negative["ok"] = False
                rc = rc or RC_MISMATCH
            else:
                negative["ok"] = True
            payload["negative"] = negative

    if not args.json_line:
        print(f"== WB-3 案例夹具 · {inc.id} {inc.title} ==")
        print(f"   {inc.describe()}")
        print(f"   正例：symptom_seen={pos['reproduced']}  events_fired={pos['fired']}")
        for c in pos["criteria"]:
            print(f"     {'✅' if c['pass'] else '❌'} {c['name']:<28s} "
                  f"{c['metric']} {c['op']} {c['value']}  实得 {c['actual']}")
        if negative is not None:
            print(f"   负例（{negative['arm']} / events_on={negative['events_on']}）："
                  f"symptom_seen={negative['symptom_seen']}  "
                  f"{'✅ 症状已消失' if negative['ok'] else '❌ 症状仍在 ⇒ 夹具在测别的东西'}")
    if args.json_line:
        print(contract_line(payload))
    return rc


if __name__ == "__main__":      # pragma: no cover
    sys.exit(main())
