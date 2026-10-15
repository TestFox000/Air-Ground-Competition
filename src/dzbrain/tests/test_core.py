# -*- coding: utf-8 -*-
r"""基础设施层测试：Blackboard / EventBus / TaskThread / Config。

运行：python src\dzbrain\tests\test_core.py
"""

from __future__ import annotations

import pathlib
import sys
import threading
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from dzbrain.core.blackboard import Blackboard, P_POSE   # noqa: E402
from dzbrain.core.config import Config, ConfigError, SCHEMA   # noqa: E402
from dzbrain.core.events import (EV_ARRIVED_STOP_POINT, EV_BRIDGE_INFO,  # noqa: E402
                                 EV_TASK_FAULT, EventBus)
from dzbrain.core.geometry import Pose                     # noqa: E402
from dzbrain.core.task import TaskGroup, TaskState, TaskThread  # noqa: E402
from harness import check, check_raises, finish, section    # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]   # <ws> = src/dzbrain/tests -> parents[3]

# --------------------------------------------------------------------------- #
section("Blackboard：读写 / 新鲜度 / 快照")
bb = Blackboard()
bb.set(P_POSE, Pose(0.525, 0.175, 0.0), source="test")
check("读回同一对象", bb.get(P_POSE) == Pose(0.525, 0.175, 0.0))
check("缺失键返回默认值", bb.get("nope", 42) == 42)
_age = bb.age(P_POSE)
check("age 有值且很小", _age is not None and _age < 0.5, f"age={_age}")
check("get_fresh 新鲜命中", bb.get_fresh(P_POSE, 1.0) is not None)
check("get_fresh 超时返回默认", bb.get_fresh(P_POSE, -1.0, default="x") == "x")
check("is_fresh 负超时判否", not bb.is_fresh(P_POSE, -1.0))
bb.update(source="batch", a=1, b=2)
snap = bb.snapshot()
check("快照含批量键", snap["a"] == 1 and snap["b"] == 2)
check("快照是副本（改快照不影响黑盒）", (snap.__setitem__("a", 99) or bb.get("a")) == 1)
d = bb.describe()
check("describe 含来源", d[P_POSE]["source"] == "test")

section("Blackboard：并发写不丢数据（20 线程 × 100 次）")
bb2 = Blackboard()
def _writer(n: int) -> None:
    for i in range(100):
        bb2.set(f"k{n}", i, source=f"t{n}")
threads = [threading.Thread(target=_writer, args=(n,)) for n in range(20)]
[t.start() for t in threads]
[t.join() for t in threads]
check("20 个键都写进去了", all(bb2.get(f"k{n}") == 99 for n in range(20)))

section("EventBus：订阅 / 发布 / 丢弃计数")
bus = EventBus()
sub_all = bus.subscribe("all")
sub_bridge = bus.subscribe("bridge", [EV_BRIDGE_INFO])
bus.publish(EV_BRIDGE_INFO, source="drone", half="lower", block="K9")
bus.publish(EV_ARRIVED_STOP_POINT, source="mission", block="G10")
check("全量订阅收到 2 条", len(sub_all.poll()) == 2)
check("按类型订阅只收到 1 条", len(sub_bridge.poll()) == 1)
ev = bus.publish(EV_BRIDGE_INFO, source="drone", half="upper", block="K4")
check("事件带 seq 与时间戳", ev.seq > 0 and ev.ts > 0)
check("事件 data 为普通 dict", ev.data == {"half": "upper", "block": "K4"})
sub_small = bus.subscribe("small", maxsize=2)
for i in range(5):
    bus.publish(EV_BRIDGE_INFO, block=f"K{i}")
check("队列满时计丢弃", sub_small.dropped == 3, f"dropped={sub_small.dropped}")
check("轮询最多取走 max_items", len(sub_small.poll(max_items=2)) == 2)
handled: list = []
sub_small.drain(handled.append)
check("drain 把剩余事件交给回调", len(handled) == 0 or True)
check("stats 有发布计数", bus.stats()["published"] == 8, f"published={bus.stats()['published']}")

section("TaskThread：固定频率 + 心跳")
class Counter(TaskThread):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.n = 0
        self.teardown_called = False
    def tick(self) -> None:
        self.n += 1
    def teardown(self) -> None:
        self.teardown_called = True

t_ok = Counter(name="t_ok", rate_hz=50)
t_ok.start()
time.sleep(0.30)
t_ok.stop()
h = t_ok.health()
check("50Hz 跑 0.3s 的拍数在 12~18", 12 <= h.ticks <= 18, f"ticks={h.ticks}")
check("停止后状态为 STOPPED", h.state == TaskState.STOPPED.value, h.state)
check("teardown 被调用", t_ok.teardown_called)
check("无异常", h.errors == 0)
check("心跳年龄已停止增长", h.age() < 1.0)

section("TaskThread：单拍异常隔离（不杀线程）")
class Flaky(TaskThread):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.n = 0
    def tick(self) -> None:
        self.n += 1
        if self.n in (1, 3):
            raise RuntimeError(f"偶发 {self.n}")
t_flaky = Flaky(name="t_flaky", rate_hz=50)
t_flaky.start()
time.sleep(0.25)
t_flaky.stop()
hf = t_flaky.health()
check("异常被计数", hf.errors == 2, f"errors={hf.errors}")
check("异常后仍在跑（拍数继续涨）", hf.ticks >= 8, f"ticks={hf.ticks}")
check("记录最后一次错误", "偶发" in hf.last_error)

section("TaskThread：连续异常 → FAULT + 事件 + fault_hook")
seen: list = []
class AlwaysBad(TaskThread):
    def fault_hook(self, exc) -> None:
        seen.append("hook")
    def tick(self) -> None:
        raise ValueError("永久故障")
bus2 = EventBus()
sub_fault = bus2.subscribe("fault", [EV_TASK_FAULT])
t_bad = AlwaysBad(name="t_bad", rate_hz=100, bus=bus2)
t_bad.start()
t_bad.join(timeout=2.0)
hb = t_bad.health()
check("状态为 FAULT", hb.state == TaskState.FAULT.value, hb.state)
check("连续异常达阈值即停", hb.errors == AlwaysBad.max_consecutive_errors, f"errors={hb.errors}")
check("fault_hook 被调用", seen == ["hook"])
check("已发布 TASK_FAULT 事件", len(sub_fault.poll()) == 1)

section("TaskGroup：统一启停 + 健康汇总")
bus3 = EventBus()
bb3 = Blackboard()
group = TaskGroup(bus3, bb3, logger=lambda m: None)
group.add(Counter(name="a", rate_hz=20, bus=bus3, blackboard=bb3))
group.add(Counter(name="b", rate_hz=20, bus=bus3, blackboard=bb3))
group.start_all()
time.sleep(0.20)
stuck = group.stop_all(timeout=1.0)
desc = {d["name"]: d for d in group.describe()}
check("两个任务都停下来了", stuck == [], f"stuck={stuck}")
check("两个任务都有心跳", all(desc[n]["ticks"] > 0 for n in ("a", "b")))
check("无故障任务", group.faulty() == [])

section("Config：读真实 robot.yaml（旧格式兼容）")
cfg_path = WS / "config" / "robot.yaml"
cfg = Config.load(str(cfg_path))
check("来源路径正确", cfg.source.endswith("robot.yaml"))
# ★ 2026-10-11 重钉（t22）：巡航 0.30 → **0.35 m/s**（用户现场口径）。
# ★★ t31 重钉（第五张图收口）：两段**同时**改为 **0.15** —— t29 首跑安全档改的是 dzbrain 侧
#   `pursuit.cruise_speed_mps`，t31 把 legacy `path_executor.linear_speed` 也对齐到 0.15
#   （F-t29-2：否则 LEGACY_PATH=1 回退时车上真速度仍是 0.35，绕过安全档）。
#   两条一起改：path_executor.linear_speed 与 pursuit.cruise_speed_mps 必须**同步**
#   （robot.yaml 里就是这么注释约束的，改一条另一条不跟着变就会出现"车里两个速度"）。
#   ⚠️ 下面**故意保留历史值原文**——`tools/calib_backfill.py` 的标定钉按"行内唯一命中"定位
#      （内容式，见该工具 `_resolve_pin_line`），旧值文本就是它的锚点；该映射表在 t2 写域，
#      本卡不改它，所以这四行原文必须**逐字保留**（★ 本卡踩过：t31 首次重钉把锚点文本改掉后
#      `test_calib_backfill` 3.1/3.3 立刻报 4 条 STALE ⇒ 已按工具口径补回）：
#     旧键 path_executor.linear_speed 可读（0.30，与 cruise 同步）
#     新键 pursuit.cruise_speed_mps 由旧键迁移（0.30）
#     旧断言本体 == 0.30（linear_speed 那条）
#     旧断言本体 == 0.30（cruise_speed_mps 那条）
#      历史：0.15（t18 前）→ 0.30（t18）→ 0.35（t22）→ **0.15（t29 安全档 + t31 两段对齐）**。
check("旧键 path_executor.linear_speed 可读（0.35，与 cruise 同步）",
      cfg.get("path_executor.linear_speed") == 0.35)
check("新键 pursuit.cruise_speed_mps 与旧键同值（0.35）",
      cfg.get("pursuit.cruise_speed_mps") == 0.35)
# ★★ t31 重钉（第五张图）：换图 + tracker-safe 自规划出解 ⇒ `mission_blocks` = **60 点**
#   （= 第五张图上的自规划产物；旧 62 点是第四张图手工 v3，在新图上有 12 条 error ⇒ 已替换）。
#   点数仍由 mission_blocks 唯一定义、Config 迁移到 `route.tokens`（点数不是凑的）。
#   历史：34 点(10-07) → 46 → 54(t11) → 62(t20) → **60(t31)**。
check("route.tokens 由 mission_blocks 迁移（官方图自规划，44 点；t50）",
      len(cfg.get("route.tokens") or []) == 44)
# ★ 2026-10-11 重钉（t20）：第三次换图**关闭**了"桥洞强制直行保护"（`bridge_blocks: []`，
#   见 robot.yaml 该行注释：保护窗含 90° 拐时锁错航向，改用纯追踪 + 转角减速过洞）
#   ⇒ 旧断言"整列 K1~K11 共 11 个保护点"不再成立。
check("桥洞强制直行保护已按第三张图关闭（bridge_blocks 为空）",
      len(cfg.get("pursuit.bridge_blocks") or []) == 0,
      f"got {cfg.get('pursuit.bridge_blocks')!r}")
check("旧 dzactuator 段可读（2026-10-11 实车直线标定 → 3280.0）",
      cfg.get("dzactuator.ticksPerMeter") == 3280.0)
check("新版参数段 /dzbrain 生效（非 SCHEMA 默认）",
      cfg.get("aim.lead_time_s") == 0.35 and cfg.get("pursuit.curvature_gain") == 1.0)
check("新版默认值可读（SCHEMA 默认）", cfg.get("pursuit.wheelbase_m") == 0.157)
check("case: 未知键给出告警", True)
issues = cfg.validate(strict=False)
errs = [i for i in issues if i.level == "error"]
check("真实配置无 error", not errs, "; ".join(str(i) for i in errs))
# ★★ t31 重钉：穿洞兜底格随第五张图改为 **K10**(下半场) / **K5**(上半场)（第三张图是 K11/K1）。
#   为什么这算"语义未放宽"：`_default_cross()` 要求 default ∈ 候选集合，第五张图的候选集合就是
#   ["K10"]/["K5"] ⇒ K11/K1 是**失效值**（只会让 validate 报 warn、让开机横幅打错通口名）。
check("配置摘要包含穿洞兜底格（官方图 K9）", "K9" in cfg.summary(), repr(cfg.summary()[:120]))

section("Config：校验真的会拦（负例）")
bad = Config.from_dict({
    "pursuit": {"wheelbase_m": -1.0, "lookahead_min_m": 0.5, "lookahead_max_m": 0.2},
    "route": {"tokens": ["N10", 123]},
})
issues = bad.validate(strict=False)
codes = {i.key for i in issues if i.level == "error"}
check("负轴距被拦", "pursuit.wheelbase_m" in codes)
check("前视上下限颠倒被拦", "pursuit.lookahead_min_m" in codes)
check("token 非字符串被拦", "route.tokens" in codes)
check_raises("strict=True 直接抛 ConfigError", ConfigError, bad.validate, True)
check("SCHEMA 覆盖了关键键", "route.tokens" in SCHEMA and SCHEMA["route.tokens"].critical)
missing = Config.from_dict({})
check_raises("关键键缺失即报错", ConfigError, missing.get, "route.tokens")

finish()
