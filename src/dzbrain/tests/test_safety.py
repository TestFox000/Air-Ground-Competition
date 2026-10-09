# -*- coding: utf-8 -*-
r"""SafetyTask 测试（纯脚本，无 pytest）。

运行：python src\dzbrain\tests\test_safety.py

覆盖（P1接口约定.md §3.4 + 任务书逐条）：
  1. 正常状态 → P_SAFETY.ok == True 且不刹车
  2. 定位失联（pose_frozen / P_ODOM_AGE 大 / 位姿缺失）→ 刹车 ≥1、ok=False、原因含 pose
  3. 越界（x=-0.5，margin 0.6 → 出界）→ 刹车且 ok=False
  4. 任务 FAULT（FakeTask 真的进 FAULT）→ 刹车
  5. 任务心跳停滞 → 刹车
  6. 遥控接管 → **不**产生安全故障（人比机器优先）
  7. clear_safety() → ok=True 且发过 EV_SAFETY_CLEAR
  8. 同一原因连续 50 拍只发 1 次 EV_SAFETY_STOP（限流生效）
  ★ 越界只有 Safety 一处生效：Localization 把越界位姿照实写入 P_POSE（不拦），
     Safety 才刹车 —— 这就是"唯一有权强制停车者"的可执行断言。
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:                                    # 车端/CI 控制台默认不是 UTF-8，中文会乱码
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

from dzbrain.core import geometry as geo                                      # noqa: E402
from dzbrain.core.blackboard import (P_ODOM_AGE, P_POSE, P_SAFETY,            # noqa: E402
                                     P_ROUTE, Blackboard)
from dzbrain.core.config import Config                                        # noqa: E402
from dzbrain.core.events import (EV_SAFETY_CLEAR, EV_SAFETY_STOP,             # noqa: E402
                                 EV_TASK_FAULT, EventBus)
from dzbrain.core.state import SafetyState                                    # noqa: E402
from dzbrain.core.task import TaskGroup, TaskState, TaskThread                # noqa: E402
from dzbrain.route.model import Waypoint, make_route                          # noqa: E402
from dzbrain.tasks.localization import LocalizationTask                       # noqa: E402
from dzbrain.tasks.safety import SafetyTask                                   # noqa: E402
from fakes import FakeBridge, FakeClock                                       # noqa: E402
from harness import check, finish, section                                  # noqa: E402

INF = float("inf")


# --------------------------------------------------------------------------- #
#  脚手架
# --------------------------------------------------------------------------- #
class SafetyProbe(SafetyTask):
    """把"实际发出的停车指令次数"记进 P_SAFETY.stops（测试断言用）。

    生产实现里 `stops` 来自"进入安全停车的次数"；测试需要断言"刹车 ≥1 次"这个**事实**，
    所以在子类里让计数以 `brakes` 为准 —— 这样计数不依赖具体是哪条代码路径写 P_SAFETY。
    签名必须与基类逐字一致（`reasons`/`now` 都是关键字且基类两种调用形态都有）。
    """

    def _state(self, *, ok: bool, now: float, reasons: tuple[str, ...] | None = None) -> SafetyState:
        st = super()._state(ok=ok, now=now)
        return SafetyState(ok=st.ok, reasons=st.reasons, since=st.since,
                           stops=self.brakes + (1 if self.stopped else 0))


class FakeTask(TaskThread):
    """可控假任务：tick 可以"正常"或"必然抛异常"（用于造 FAULT / 心跳）。"""

    def __init__(self, name: str, *, failing: bool, **kw) -> None:
        super().__init__(name=name, rate_hz=20.0, **kw)
        self.failing = failing
        self.n = 0

    def tick(self) -> None:
        self.n += 1
        if self.failing:
            raise RuntimeError("故意故障")


class LiveBridge(FakeBridge):
    """/odom/localization 的替身：位置固定，但**每拍都在发**（age 随真实时间走）。

    ★ 为什么必须另做一个：`FakeBridge._pose_ts` 在 `set_pose()` 时冻结，
      于是随着 FakeClock 快进，`pose_age()` 会一路涨到超过 `pose_timeout_s`
      —— 那是**测试脚手架**造成的假故障，不是被测行为。
      真车上话题是按 20 Hz 持续到达的，age 恒为"距上一帧"。这里如实建模，
      于是"时间推进"与"数据新鲜度"两件事在测试里不再互相污染。
    """

    def __init__(self, clock: FakeClock) -> None:
        super().__init__(clock)
        self.last_msg_t = clock.now()

    def pose_age(self) -> float:
        if self._pose is None or self.pose_frozen:
            return float("inf")
        return self.clock.now() - self.last_msg_t


def build(overrides: dict | None = None, *, with_pose: bool = True,
          group=None, margin: float = 0.6, timeout: float = 1.0, stall: float = 1.0,
          blackboard: Blackboard | None = None, bus: EventBus | None = None):
    clock = FakeClock()
    bridge = LiveBridge(clock)
    bb = blackboard if blackboard is not None else Blackboard()
    bus = bus if bus is not None else EventBus()
    cfg = Config.from_dict({
        "route": {"tokens": ["N10", "M10"]},
        "pursuit": {"map_safety_margin_m": margin},
        "safety": {"pose_timeout_s": timeout, "task_stall_s": stall,
                   "brake_on_fault": True},
        "logging": {"rate_limit_s": 1.0},
        "mission": {"auto_start": False},
        **(overrides or {}),
    })
    if with_pose:
        bridge.set_pose_at_block("N10")
        # ★ P1接口约定 §2：Safety 越界检查读的是 `P_POSE`（不直接读 bridge），
        #   生产者是 LocalizationTask。测试里等价地播一次种，否则第 ① 项
        #   "定位失联"会把"没人写过位姿"也算进去 —— 那样测不出越界路径。
        bb.set(P_POSE, bridge.pose(), source="seed", ts=clock.now())
    # ★ 关键脚手架细节：任务与 bridge 必须共用**同一个 FakeClock**。
    #   若各拿一个（都从 1000.0 起），SafetyTask 读 bridge.pose_age() 会得到
    #   `bridge.clock.now() - bridge._pose_ts`，而 pose_ts 是 bridge 自己的时钟时刻 ——
    #   两边一旦不同步就会被误判成"定位失联"（这个坑我在本地踩过一次）。
    task = SafetyProbe(bridge, bb, bus, cfg, log=lambda m: None, group=group,
                       clock=clock.now)
    task._clock = clock          # 测试脚手架：保留 FakeClock 本体以便快进时间
    task.setup()                 # 真线程由 run() 调 setup；测试里手工调（订阅 EV_TASK_FAULT）
    return task, bridge, bb, bus, clock


def publish_pose(bridge, bb, clock, x: float, y: float, yaw: float = 0.0) -> None:
    """模拟 LocalizationTask 的一拍：改 bridge 位置 + 把新信念位姿写进 P_POSE。

    ★ Safety 不读 bridge.pose()，它读 `P_POSE`（P1 §2）。测试里必须照这个分工走，
      否则会测出"越界没被拦住"的假结论。
    ★ 同时写 `P_ODOM_AGE`（有限值）：Localization 每拍都会写它，而 Safety 的启动宽限
      判据是"**键从未被写过** + pose_age=inf"。只播 P_POSE 不播 P_ODOM_AGE，
      断流场景会被宽限吞掉，测不到运行期失联。
    """
    bridge.set_pose(x, y, yaw)
    bb.set(P_POSE, bridge.pose(), source="seed", ts=clock.now())
    bb.set(P_ODOM_AGE, 0.0, source="seed", ts=clock.now())


def tick_n(task, n: int, *, dt: float = 0.0) -> None:
    """连打 n 拍；dt > 0 时每拍推进时钟（模拟 20 Hz 的真实时间流逝）。"""
    for _ in range(n):
        task.tick()
        if dt:
            task._clock.advance(dt)


def tick_live(task, n: int, *, dt: float = 0.05) -> None:
    """带时间推进的连拍，且每拍都有新里程计到达（贴近真车 20 Hz 话题）。"""
    for _ in range(n):
        task.bridge.last_msg_t = task._clock.now()
        task.tick()
        task._clock.advance(dt)


section("① 正常状态：ok=True、不刹车、无原因")
task, bridge, bb, bus, clock = build()
sub_stop = bus.subscribe("t1", [EV_SAFETY_STOP])
task.tick()
st = bb.get(P_SAFETY)
check("写入了 P_SAFETY", isinstance(st, SafetyState))
check("ok == True", st.ok is True, f"got {st!r}")
check("reasons 为空", st.reasons == (), f"got {st.reasons!r}")
check("刹车次数 0", bridge.brakes == 0, f"got {bridge.brakes}")
check("未发 EV_SAFETY_STOP", len(sub_stop.poll()) == 0)
check("task.stopped == False", task.stopped is False)
tick_live(task, 20)                        # 1 秒真实时间（20 拍 × 0.05 s）
check("20 拍（含 1 s 时间流逝）仍不刹车", bridge.brakes == 0, f"got {bridge.brakes}")
check("20 拍后 ok 仍为 True", bb.get(P_SAFETY).ok is True)
check("里程计一直新鲜（age 仅为本拍间隔 0.05 s）", bridge.pose_age() < 0.1,
      f"got {bridge.pose_age()}")

section("★ ①' 启动宽限：冷启动第一拍不得误刹（两个任务独立起线程的真实竞态）")
# 复现实测缺陷：Safety 的第一拍可能早于 Localization 的第一次写入。
# 那时 bridge.pose_age()==inf、P_ODOM_AGE **键还不存在**（= 从未收到过数据）。
# 若不区分，这一拍会判"定位失联"，而本任务**故意不做自动恢复** → 整车被永久锁在
# SAFE_STOP，必须人工 clear_safety() 才能动。必须只进 DEGRADED：不刹车、不锁存。
task, bridge, bb, bus, clock = build(with_pose=False, stall=5.0)
check("确认此刻 P_ODOM_AGE 键不存在", bb.get(P_ODOM_AGE) is None)
check("确认此刻 bridge 也没有位姿", bridge.pose() is None)
task._started_at = clock.now()                            # 宽限基准 = 现在
task.tick()
st = bb.get(P_SAFETY)
check("宽限期内 ok 仍为 True（不误刹）", st.ok is True, f"got {st!r}")
check("宽限期内不刹车", bridge.brakes == 0, f"got {bridge.brakes}")
check("宽限期内不锁存", task.stopped is False)
check("但标记为 DEGRADED（不是在正常发车）", task.degraded is True)
check("P_SAFETY 仍被写出来（下游能读到 ok）", isinstance(st, SafetyState))

# ★★ 实车实测的**第二个**洞（2026-10-09 19:05，Orange Pi 上 python3 -m dzbrain.main）：
#   上面那条只覆盖"P_ODOM_AGE 键还不存在"。真实路径往往是：
#     Localization **先 tick 了一拍**，把 P_ODOM_AGE 写成 **inf**（它自己也没拿到里程计，
#     ROS2 发现还没完成）⇒ 旧判据 `odom_age is None` 为假 ⇒ **完全不进宽限** ⇒
#     落到 `elif pose_age == _INF` ⇒ pose_lost + 锁存 SAFE_STOP（绝不自动恢复）⇒ 车不动。
task, bridge, bb, bus, clock = build(with_pose=False, stall=5.0)
task._started_at = clock.now()
bb.set(P_ODOM_AGE, float("inf"), "test")      # ★ 关键：键**存在**但值是 inf
task.tick()
st = bb.get(P_SAFETY)
check("★P_ODOM_AGE=inf（键存在）时同样进宽限：不误刹", st.ok is True, f"got {st!r}")
check("★P_ODOM_AGE=inf 时不刹车（否则车永远不动）", bridge.brakes == 0, f"got {bridge.brakes}")
check("★P_ODOM_AGE=inf 时不锁存（stopped=False）", task.stopped is False)
check("★确实进了 DEGRADED（不是在正常发车）", task.degraded is True)
clock.advance(4.0)                            # 超过 startup_grace_s(3.0)
task.tick()
st = bb.get(P_SAFETY)
check("★宽限过后仍无数据 → 判失联（宽限不是免死金牌）", st.ok is False, f"got {st!r}")
check("原因含 pose_lost", any("pose_lost" in r for r in st.reasons), f"got {st.reasons!r}")

task, bridge, bb, bus, clock = build(with_pose=False, stall=5.0,
                                     overrides={"safety": {"startup_grace_s": 1.0}})
task._started_at = clock.now()
task.tick()
check("宽限期内不刹车（startup_grace_s=1.0）", bridge.brakes == 0, f"got {bridge.brakes}")
check("1000.99 s 时仍未越界（边界确认）", (clock.now() - task._started_at) < 1.0)
task._clock.advance(1.5)                                  # 超过宽限
task.tick()
st = bb.get(P_SAFETY)
check("宽限过后仍无数据 → 判失联", st.ok is False, f"got {st!r}")
check("原因含 pose_lost", any("pose_lost" in r for r in st.reasons), f"got {st.reasons!r}")
check("宽限过后刹车 ≥ 1", bridge.brakes >= 1, f"got {bridge.brakes}")
check("DEGRADED 标记被清掉", task.degraded is False)

# 反向：一旦见过数据，再断流就是**运行期**失联，必须立刻停车（不受宽限保护）
task, bridge, bb, bus, clock = build(with_pose=False, stall=5.0)
task._started_at = clock.now()
task.tick()                                               # 第一拍：宽限期内，不刹
check("第一拍（无数据）不刹", bridge.brakes == 0, f"got {bridge.brakes}")
check("第一拍处于 DEGRADED", task.degraded is True)
publish_pose(bridge, bb, clock, 0.525, 0.175)             # 定位上线
task.tick()
check("见到数据后 ok", bb.get(P_SAFETY).ok is True)
check("见到数据后 DEGRADED 解除", task.degraded is False, f"got {task.degraded}")
bridge.pose_frozen = True                                 # 跑着跑着断流
task.tick()
check("运行期断流立刻停车（不受宽限保护）", bb.get(P_SAFETY).ok is False)
check("运行期断流刹车 ≥ 1", bridge.brakes >= 1, f"got {bridge.brakes}")
check("运行期断流原因含 pose_lost",
      any("pose_lost" in r for r in bb.get(P_SAFETY).reasons), f"{bb.get(P_SAFETY).reasons!r}")

section("★ ①'' Safety 不得把自己判成『停滞』（看门狗不盯自己）")
task, bridge, bb, bus, clock = build(stall=1.0)
group_self = TaskGroup(bus, bb, logger=lambda m: None)
group_self.add(task)
task.group = group_self
task.tick()
check("组里只有自己时不会 task_stalled",
      not any("task_stalled" in r for r in bb.get(P_SAFETY).reasons),
      f"got {bb.get(P_SAFETY).reasons!r}")
check("组里只有自己时 ok == True", bb.get(P_SAFETY).ok is True, f"got {bb.get(P_SAFETY)!r}")
task._clock.advance(5.0)          # 时间过去 5 s：自己的 last_tick_at 会显旧
task.tick()
check("5 s 后仍不把自己判成停滞",
      not any(f"task_stalled:{task.name}" in r for r in bb.get(P_SAFETY).reasons),
      f"got {bb.get(P_SAFETY).reasons!r}")

section("② 定位失联（bridge.pose_frozen）→ 刹车 + ok=False + 原因含 pose")
task, bridge, bb, bus, clock = build()
sub_stop = bus.subscribe("t2", [EV_SAFETY_STOP])
task.tick()
check("先确认健康（不刹车）", bridge.brakes == 0 and bb.get(P_SAFETY).ok is True)
bb.set(P_ODOM_AGE, 0.0, source="seed", ts=clock.now())   # 模拟 Localization 已在写
bridge.pose_frozen = True                 # pose() 仍返回旧对象，但 pose_age() == inf
task.tick()
st = bb.get(P_SAFETY)
check("ok == False", st.ok is False, f"got {st!r}")
check("原因里含 pose", any("pose" in r for r in st.reasons), f"got {st.reasons!r}")
check("原因文本 = 里程计无数据", "age=inf" in st.reasons[0], f"got {st.reasons[0]!r}")
check("刹车 ≥ 1 次", bridge.brakes >= 1, f"got {bridge.brakes}")
check("发过 EV_SAFETY_STOP（1 条）", len(sub_stop.poll()) == 1)
tick_live(task, 10)                        # 故障持续 0.5 s（10 拍 × 0.05 s）
check("持续故障期间每拍都发零速（11 拍 = 11 次，底盘 1s 看门狗）", bridge.brakes == 11,
      f"got {bridge.brakes}")
check("P_SAFETY.stops 记录了实际刹车数", bb.get(P_SAFETY).stops >= 1,
      f"got {bb.get(P_SAFETY).stops}")

section("②b 定位失联（P_ODOM_AGE 大 / 键缺失）→ 同样刹车")
task, bridge, bb, bus, clock = build()
bb.set(P_ODOM_AGE, 5.0, source="test")    # 5 s > pose_timeout 1.0 s
task.tick()
check("P_ODOM_AGE=5 即判失联", bb.get(P_SAFETY).ok is False)
check("原因含 pose_stale", any("pose_stale" in r for r in bb.get(P_SAFETY).reasons),
      f"got {bb.get(P_SAFETY).reasons!r}")
check("刹车 ≥ 1", bridge.brakes >= 1)

task2, bridge2, bb2, bus2, clock2 = build()
task2.tick()
check("先健康", bb2.get(P_SAFETY).ok is True)
bb2.set(P_ODOM_AGE, INF, source="test")
task2.tick()
check("P_ODOM_AGE=inf 即判失联", bb2.get(P_SAFETY).ok is False, f"{bb2.get(P_SAFETY)!r}")
check("刹车 ≥ 1", bridge2.brakes >= 1)

section("③ 越界：x=-0.5、margin=0.6 → 出界 → 刹车且 ok=False")
task, bridge, bb, bus, clock = build(margin=0.6)
sub_stop = bus.subscribe("t3", [EV_SAFETY_STOP])
publish_pose(bridge, bb, clock, -0.5, 0.175)   # x_min = 0.175-0.6 = -0.425 > -0.5
check("确认这个坐标确实在地图外", not geo.in_bounds(-0.5, 0.175, 0.6))
check("x 下界 = -0.425（0.175 - 0.6）", geo.map_bounds(0.6)[0] == -0.425,
      f"got {geo.map_bounds(0.6)[0]!r}")
check("确认它离边界只差 0.075 m（不是离谱值）", abs((-0.425) - (-0.5) - 0.075) < 1e-12,
      f"got {abs(-0.425 - (-0.5))!r}")
task.tick()
st = bb.get(P_SAFETY)
check("ok == False", st.ok is False, f"got {st!r}")
check("原因含 out_of_bounds", any("out_of_bounds" in r for r in st.reasons),
      f"got {st.reasons!r}")
check("刹车 ≥ 1 次", bridge.brakes >= 1, f"got {bridge.brakes}")
check("发过 EV_SAFETY_STOP", len(sub_stop.poll()) >= 1)

# 四个方向的边界对称验证
for (x, y, where) in [(-0.5, 0.5, "x 下界"), (4.4, 0.5, "x 上界"),
                      (0.5, -0.5, "y 下界"), (0.5, 5.4, "y 上界")]:
    t, br, b, bu, ck = build(margin=0.6)
    publish_pose(br, b, ck, x, y)
    t.tick()
    check(f"{where} ({x}, {y}) 被判越界", b.get(P_SAFETY).ok is False,
          f"got {b.get(P_SAFETY).reasons!r}")
    check(f"{where} 原因确为 out_of_bounds",
          any("out_of_bounds" in r for r in b.get(P_SAFETY).reasons),
          f"got {b.get(P_SAFETY).reasons!r}")
# 界内在余量范围内 → 不报
t, br, b, bu, ck = build(margin=0.6)
publish_pose(br, b, ck, -0.40, 0.175)     # x_min=-0.425 → -0.40 仍在界内
t.tick()
check("界内（-0.40 > -0.425）不报越界", b.get(P_SAFETY).ok is True,
      f"got {b.get(P_SAFETY).reasons!r}")
check("界内不刹车", br.brakes == 0)

section("★ 越界只有 Safety 一处生效（Localization 照实写，不拦）")
bus_o = EventBus()
bb_o = Blackboard()
clock_o = FakeClock()
bridge_o = FakeBridge(clock_o)
cfg_o = Config.from_dict({"route": {"tokens": ["N10", "M10"]},
                          "pursuit": {"map_safety_margin_m": 0.6},
                          "safety": {"pose_timeout_s": 1.0, "task_stall_s": 1.0},
                          "fix": {}})
bb_o.set(P_ROUTE, make_route([Waypoint.at_block("N10"), Waypoint.at_block("M10")]),
         source="test")
loc = LocalizationTask(bridge_o, bb_o, bus_o, cfg_o, log=lambda m: None, clock=clock_o.now)
bridge_o.set_pose(-0.5, 0.175)
loc.tick()
check("Localization 照实把越界位姿写进 P_POSE（不拦）",
      bb_o.get(P_POSE) is not None and abs(bb_o.get(P_POSE).x - (-0.5)) < 1e-12,
      f"got {bb_o.get(P_POSE)!r}")
check("Localization 没有把越界当异常（仍报有限 age）",
      bb_o.get(P_ODOM_AGE) == 0.0, f"got {bb_o.get(P_ODOM_AGE)!r}")
check("Localization 绝不刹车（刹车权只在 Safety）", bridge_o.brakes == 0,
      f"got {bridge_o.brakes}")

safety_o = SafetyProbe(bridge_o, bb_o, bus_o, cfg_o, log=lambda m: None,
                       clock=clock_o.now)
safety_o.tick()
check("加上 Safety 之后同一份数据立刻被拦（刹车 ≥ 1）", bridge_o.brakes >= 1,
      f"got {bridge_o.brakes}")
check("同一份数据 P_SAFETY.ok == False", bb_o.get(P_SAFETY).ok is False)
check("原因就是 out_of_bounds", any("out_of_bounds" in r for r in bb_o.get(P_SAFETY).reasons),
      f"got {bb_o.get(P_SAFETY).reasons!r}")

section("④ 任务 FAULT：FakeTask 真的进 FAULT → 刹车")
bus_g = EventBus()
bb_g = Blackboard()
group = TaskGroup(bus_g, bb_g, logger=lambda m: None)
faulty = FakeTask("TrackerFake", failing=True, bus=bus_g, blackboard=bb_g)
healthy = FakeTask("MissionFake", failing=False, bus=bus_g, blackboard=bb_g)
group.add(faulty)
group.add(healthy)

# 先让"健康任务"证明自己活着（否则它也会被判定为停滞，混掉本用例要测的 FAULT 路径）
healthy.tick()
with healthy._health_lock:
    healthy._health.last_tick_at = 1000.0        # FakeClock 起点 = "刚刚"
check("健康任务的心跳被设为当前时刻", healthy.health().last_tick_at == 1000.0)

# 把故障任务的 5 次连续异常走完 → 状态 FAULT（与 TaskThread.run 的判 FAULT 路径一致）
faulty.failing = True
faulty.bus = EventBus()                          # 隔离：不让 ④ 的 FAULT 事件污染 ④b
for _ in range(FakeTask.max_consecutive_errors):
    try:
        faulty.tick()
    except RuntimeError:
        pass
    with faulty._health_lock:
        faulty._health.errors += 1
        faulty._health.consecutive_errors += 1
        faulty._health.last_tick_at = 1000.0
        if faulty._health.consecutive_errors >= FakeTask.max_consecutive_errors:
            faulty._health.state = TaskState.FAULT.value
check("故障任务状态确为 FAULT", faulty.health().state == TaskState.FAULT.value,
      f"got {faulty.health().state}")
check("健康任务状态不是 FAULT", healthy.health().state != TaskState.FAULT.value,
      f"got {healthy.health().state}")

task, bridge, bb, bus, clock = build(group=group, stall=5.0, blackboard=bb_g, bus=bus_g)
task.tick()
st = bb_g.get(P_SAFETY)
check("任务 FAULT 被检出 → ok=False", st is not None and st.ok is False, f"got {st!r}")
check("原因含 task_fault", st is not None and any("task_fault" in r for r in st.reasons),
      f"got {getattr(st, 'reasons', None)!r}")
check("原因里点名了 TrackerFake",
      st is not None and any("TrackerFake" in r for r in st.reasons),
      f"got {getattr(st, 'reasons', None)!r}")
check("刹车 ≥ 1 次", bridge.brakes >= 1, f"got {bridge.brakes}")
check("P_SAFETY 写在 group 共用的黑盒上", bb_g.get(P_SAFETY) is not None)

section("④b 收到 EV_TASK_FAULT 事件 → 刹车（订阅路径）")
task, bridge, bb, bus, clock = build()
task.tick()
check("先健康", bb.get(P_SAFETY).ok is True)
bus.publish(EV_TASK_FAULT, source="AimTask", task="AimTask", where="tick",
            error="RuntimeError: 炸了")
task.tick()
st = bb.get(P_SAFETY)
check("事件路径也触发停车", st.ok is False, f"got {st!r}")
check("原因含 task_fault_event", any("task_fault_event" in r for r in st.reasons),
      f"got {st.reasons!r}")
check("原因里带来源任务名", any("AimTask" in r for r in st.reasons), f"got {st.reasons!r}")
check("计数 fault_events_seen == 1", task.fault_events_seen == 1,
      f"got {task.fault_events_seen}")
check("刹车 ≥ 1", bridge.brakes >= 1)
# 重复事件不重复加原因
bus.publish(EV_TASK_FAULT, source="AimTask", task="AimTask", where="tick", error="又炸了")
task.tick()
check("同一任务重复故障不产生重复原因", len(bb.get(P_SAFETY).reasons) == 1,
      f"got {bb.get(P_SAFETY).reasons!r}")

section("⑤ 任务心跳停滞 → 刹车")
bus_g2 = EventBus()
bb_g2 = Blackboard()
group2 = TaskGroup(bus_g2, bb_g2, logger=lambda m: None)
stuck = FakeTask("StuckFake", failing=False, bus=bus_g2, blackboard=bb_g2)
group2.add(stuck)
with stuck._health_lock:
    stuck._health.last_tick_at = 1000.0          # = FakeClock 起点（"刚刚 tick 过"）
check("先有正常心跳（= 注入时钟当前时刻）", stuck.health().last_tick_at == 1000.0)
task, bridge, bb, bus, clock = build(group=group2, stall=1.0, blackboard=bb_g2,
                                     bus=bus_g2)
with stuck._health_lock:
    stuck._health.last_tick_at = 1000.0          # = FakeClock 当前时刻（新鲜）
task.tick()
check("心跳新鲜时不报停滞", bb_g2.get(P_SAFETY).ok is True,
      f"got {bb_g2.get(P_SAFETY)!r}")
with stuck._health_lock:
    stuck._health.last_tick_at = 1000.0 - 10.0   # 心跳停在 10 s 前
task.tick()
st = bb_g2.get(P_SAFETY)
check("心跳 10 s > task_stall 1 s → 停车", st.ok is False, f"got {st!r}")
check("原因含 task_stalled", any("task_stalled" in r for r in st.reasons),
      f"got {st.reasons!r}")
check("原因里带年龄（约 10.05 s，含 0.05 时钟偏斜容差）",
      any("10.0" in r or "10.1" in r for r in st.reasons), f"got {st.reasons!r}")
check("刹车 ≥ 1", bridge.brakes >= 1)

section("⑤c ★RUNNING 但从未 tick 的任务：宽限期内不得判停滞（实车启动竞态）")
#  ★ 实车实测（2026-10-09 18:57，Orange Pi 上 `python3 -m dzbrain.main`）：
#      🛑 安全停车：task_stalled:DroneTask 心跳 infs > 1.00s（ticks=0 state=RUNNING）
#    而安全是**刻意不自动恢复**的 ⇒ 新核一上电就永久锁在 SAFE_STOP、**车一动不动**
#    （这正是"车跑不起来"的真正原因，bench 抓不到：bench 的起线程时序不同）。
#  根因：宽限条件要求 `state == INIT`，而 `TaskThread.start()` 是**先把状态置 RUNNING、
#  再进循环跑第一拍** —— 周期最慢的 DroneTask(5Hz)/TelemetryTask(2Hz) 首拍天然来得晚，
#  正好落在"已 RUNNING、ticks=0、last_tick_at=0"的窗口里 ⇒ `age = _INF` ⇒ 判停滞。
#  修法：宽限判据只看"**从未 tick 过** + 在宽限期内"，与 state 无关。
bus_g3 = EventBus()
bb_g3 = Blackboard()
group3 = TaskGroup(bus_g3, bb_g3, logger=lambda m: None)
slow = FakeTask("SlowFake", failing=False, bus=bus_g3, blackboard=bb_g3)
group3.add(slow)
with slow._health_lock:
    slow._health.state = TaskState.RUNNING.value     # ★ 线程已启动…
    slow._health.last_tick_at = 0.0                  # …但第一拍还没跑（真实竞态）
    slow._health.ticks = 0
task, bridge, bb, bus, clock = build(group=group3, stall=1.0, blackboard=bb_g3, bus=bus_g3)
task._started_at = clock.now()                       # 宽限基准 = 现在
task.tick()
st = bb_g3.get(P_SAFETY)
check("★RUNNING 且从未 tick：宽限期内 ok 仍为 True（不误刹）", st.ok is True, f"got {st!r}")
check("★不刹车（刹车会让车永远不动）", bridge.brakes == 0, f"brakes={bridge.brakes}")
check("★原因里没有 task_stalled:SlowFake",
      not any("task_stalled:SlowFake" in r for r in st.reasons), f"got {st.reasons!r}")
clock.advance(2.0)                                   # 累计 2.0 s：仍在 startup_grace_s(3.0) 内
task.tick()
st = bb_g3.get(P_SAFETY)
check("宽限期内（+2s）仍不判**任务停滞**（此时可能因别的检查而 ok=False，只看停滞原因）",
      not any("task_stalled" in r for r in st.reasons), f"got {st.reasons!r}")
clock.advance(1.5)                                   # 累计 3.5 s > 宽限 3.0 s
task.tick()
st = bb_g3.get(P_SAFETY)
check("★宽限过后仍从未 tick → 判停滞（宽限不是免死金牌）", st.ok is False, f"got {st!r}")
check("原因含 task_stalled:SlowFake",
      any("task_stalled:SlowFake" in r for r in st.reasons), f"got {st.reasons!r}")

section("⑤b group=None → 跳过检查④并说明；其余检查照常")
task, bridge, bb, bus, clock = build(group=None)
check("group 确实没注入", task.group is None)
task.tick()
check("group=None 时正常状态仍为 ok", bb.get(P_SAFETY).ok is True)
bridge.set_pose(-0.5, 0.175)
bb.set(P_POSE, bridge.pose(), source="seed", ts=clock.now())   # 模拟 Localization 落盘
task.tick()
check("group=None 不影响越界检查", bb.get(P_SAFETY).ok is False,
      f"got {bb.get(P_SAFETY)!r}")
task.clear_safety()
check("可正常清场", bb.get(P_SAFETY).ok is True)

section("⑥ 遥控接管 → 不产生安全故障（人比机器优先）；但越界照报")
logs: list[str] = []
task, bridge, bb, bus, clock = build()
task._log = logs.append
sub6 = bus.subscribe("t6", [EV_SAFETY_STOP])
task._manual_last = None                  # 从"未知"开始 → 首次必然记录一条状态日志
bridge.set_manual_override(True)
tick_n(task, 5)
check("接管期间 ok 仍为 True", bb.get(P_SAFETY).ok is True, f"got {bb.get(P_SAFETY)!r}")
check("接管不产生原因", bb.get(P_SAFETY).reasons == (), f"got {bb.get(P_SAFETY).reasons!r}")
check("接管不刹车", bridge.brakes == 0, f"got {bridge.brakes}")
check("接管不发 EV_SAFETY_STOP", len(sub6.poll()) == 0)
check("★ 但**记录**了接管状态（不是静默）",
      any("遥控接管" in m for m in logs), f"logs={logs}")
check("只在状态变化时记录（5 拍 1 条）",
      sum(1 for m in logs if "遥控接管" in m) == 1, f"logs={logs}")
bridge.set_manual_override(False)
task.tick()
check("交回自动后仍 ok", bb.get(P_SAFETY).ok is True)
check("解除接管不刹车", bridge.brakes == 0)
check("解除接管也留痕（共 2 条）", sum(1 for m in logs if "遥控接管" in m) == 2,
      f"logs={logs}")

# 接管期间越界仍必须报（那时更危险）
task2, bridge2, bb2, bus2, clock2 = build()
bridge2.set_manual_override(True)
publish_pose(bridge2, bb2, clock2, -0.5, 0.175)
task2.tick()
check("接管期间越界照样停车", bb2.get(P_SAFETY).ok is False, f"got {bb2.get(P_SAFETY)!r}")
check("接管期间越界照样刹车", bridge2.brakes >= 1)

section("⑦ clear_safety() → ok=True 且发过 EV_SAFETY_CLEAR（且不自动恢复）")
task, bridge, bb, bus, clock = build()
sub_clear = bus.subscribe("t7", [EV_SAFETY_CLEAR])
publish_pose(bridge, bb, clock, -0.5, 0.175)
task.tick()
check("先进入停车", bb.get(P_SAFETY).ok is False and task.stopped is True)
check("刹车 1 次", bridge.brakes == 1, f"got {bridge.brakes}")

# ★ 不自动恢复：把车挪回界内，故障未清 → 仍然 ok=False；但不该继续发新的 STOP 事件
sub7 = bus.subscribe("t7b", [EV_SAFETY_STOP])
publish_pose(bridge, bb, clock, 0.525, 0.175)
tick_n(task, 10)
check("条件解除后**不自动恢复**（仍 ok=False）", bb.get(P_SAFETY).ok is False,
      f"got {bb.get(P_SAFETY)!r}")
check("不自动恢复期间不发新的 EV_SAFETY_STOP", len(sub7.poll()) == 0)
check("auto_recover_count 在累加（已记录'其实可恢复'）", task.auto_recover_count >= 1,
      f"got {task.auto_recover_count}")
check("仍在锁存态", task.stopped is True)
check("刹车已停止（不再每拍发零速）", bridge.brakes == 1, f"got {bridge.brakes}")

st = task.clear_safety()
check("clear_safety 返回 ok=True", st.ok is True, f"got {st!r}")
check("P_SAFETY 被写为 ok=True", bb.get(P_SAFETY).ok is True)
check("P_SAFETY.reasons 已清空", bb.get(P_SAFETY).reasons == ())
check("发过 EV_SAFETY_CLEAR（1 条）", len(sub_clear.poll()) == 1)
check("clear_events == 1", task.clear_events == 1)
check("stopped 已解除", task.stopped is False)
check("清场后仍不自动踩油门（刹车计数不再增长）",
      (tick_n(task, 5) or bridge.brakes) == 1, f"got {bridge.brakes}")
check("清场后继续 tick 保持 ok", bb.get(P_SAFETY).ok is True)

# 幂等：没有故障时再清一次不重复发事件
before = task.clear_events
task.clear_safety()
check("无故障时 clear 幂等（不重复发事件）", task.clear_events == before,
      f"got {task.clear_events}")

section("⑧ 同一原因连续 50 拍只发 1 次 EV_SAFETY_STOP（限流生效）")
bus8 = EventBus()
bb8 = Blackboard()
group8 = TaskGroup(bus8, bb8, logger=lambda m: None)
grp_task = FakeTask("GrpFake", failing=False, bus=bus8, blackboard=bb8)
group8.add(grp_task)
with grp_task._health_lock:
    grp_task._health.last_tick_at = 1000.0       # 心跳新鲜（age=0.05 s < stall 1.0 s）
task, bridge, bb, bus, clock = build(stall=1.0, group=group8, blackboard=bb8, bus=bus8)
sub8 = bus.subscribe("t8", [EV_SAFETY_STOP])
publish_pose(bridge, bb, clock, 0.525, 0.175)    # 先有定位数据（否则进启动宽限）
bridge.pose_frozen = True                        # 然后断流
tick_n(task, 50)
events = sub8.poll()
check("50 拍只发 1 条 EV_SAFETY_STOP", len(events) == 1, f"got {len(events)}")
check("stop_events == 1", task.stop_events == 1, f"got {task.stop_events}")
check("但 50 拍都在刹车（安全态持续压住）", bridge.brakes == 50, f"got {bridge.brakes}")
if events:
    check("事件带 reasons 元组", isinstance(events[0].data.get("reasons"), tuple),
          f"{events[0].data}")
    check("事件 first=True（首次进入停车）", events[0].data.get("first") is True,
          f"{events[0].data}")

# 原因集合变化 → 必须再发一条（限流只压重复项，不吞新原因）
# ★ 关键：先把 pose_lost 解除（pose_frozen 置回 False），否则"位姿仍失联"这条原因
#   会让状态机看不出变化 —— 那样测的就不是"新增越界"了。
# ★ 也不推进时钟：推进会让 P_ODOM_AGE 变旧、多出一条 pose_stale，污染本用例。
bridge.pose_frozen = False
publish_pose(bridge, bb, clock, -0.5, 0.175)     # 有新鲜位姿，但位置越界
task.tick()
ev2 = sub8.poll()
check("原因集合变化时补发一条（新增越界）", len(ev2) == 1, f"got {len(ev2)}")
if ev2:
    check("新增原因里有 out_of_bounds",
          any("out_of_bounds" in r for r in ev2[0].data.get("reasons", ())),
          f"{ev2[0].data}")
    check("added 里恰好只有这 1 项新增", len(ev2[0].data.get("added", ())) == 1,
          f"{ev2[0].data}")
    check("removed 里记录了被解除的 pose_lost",
          any("pose_lost" in r for r in ev2[0].data.get("removed", ())), f"{ev2[0].data}")
check("stop_events 累计为 2", task.stop_events == 2, f"got {task.stop_events}")
check("两拍都刹车（累计 51）", bridge.brakes == 51, f"got {bridge.brakes}")

section("⑨ 抖动熔断：反复'解除又触发'达上限打横幅（仍不自动恢复）")
task, bridge, bb, bus, clock = build()
publish_pose(bridge, bb, clock, 0.525, 0.175)      # 先有定位数据（否则进启动宽限）
bridge.pose_frozen = True
task.tick()
check("先停车", bb.get(P_SAFETY).ok is False)
bridge.pose_frozen = False
tick_n(task, task.auto_recover_limit + 2)
check("抖动计数达到上限", task.auto_recover_count >= task.auto_recover_limit,
      f"got {task.auto_recover_count}")
check("仍然没有自动恢复", bb.get(P_SAFETY).ok is False, f"got {bb.get(P_SAFETY)!r}")
check("闸门标语未误报（不影响 ok/reasons 语义）",
      bb.get(P_SAFETY).reasons != (), f"got {bb.get(P_SAFETY).reasons!r}")

section("⑩ brake_on_fault=False → 只报不刹（配置生效）")
task, bridge, bb, bus, clock = build(overrides={"safety": {"brake_on_fault": False}})
sub10 = bus.subscribe("t10", [EV_SAFETY_STOP])
publish_pose(bridge, bb, clock, 0.525, 0.175)
bridge.pose_frozen = True
tick_n(task, 3)
check("ok 仍被判 False", bb.get(P_SAFETY).ok is False)
check("但不发零速指令", bridge.brakes == 0, f"got {bridge.brakes}")
check("仍然发布 EV_SAFETY_STOP（让上层知道）", len(sub10.poll()) == 1)

finish()
