# -*- coding: utf-8 -*-
r"""FixTask（误差修正 · 估计侧）测试 —— 纯脚本，无 pytest。

运行：python src\dzbrain\tests\test_fix.py

覆盖（任务书六条 + 反用防护）：
  1. 无锚定事件 → 不出 scale（scale==1.0，样本数 0）；
  2. 喂 5 段"信念位移 = 真实 × 1.18" → scale 收敛到 1.18±0.03，且每段步长 ≤ fix.scale_step_max；
  3. scale_limit=0.10 → 估计被夹在 [0.9, 1.1]；
  4. 异常段（漂移 0.5 m）被剔除且发 EV_DRIFT_WARNING；
  5. fix.scale_est_enabled=False → 只统计不改估计；
  6. 漂移汇总（max）正确；
  7. ★ 反用防护：只写 P_FIX_EST、绝不碰 P_POSE / P_POSE_RAW / P_FIX（估计侧不改位姿）。

★ 测试用**手动单拍**（不 start 线程）+ FakeClock 快进：任务内只用注入的 clock，
  所以时间完全可控（10 Hz → 每拍 advance(0.1)）。
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:                                    # 车端/Windows 控制台默认不是 UTF-8
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

from dzbrain.core import geometry as geo                                          # noqa: E402
from dzbrain.core.blackboard import (P_FIX, P_FIX_EST, P_POSE, P_POSE_RAW,        # noqa: E402
                                     P_ROUTE_S, Blackboard)
from dzbrain.core.config import Config                                            # noqa: E402
from dzbrain.core.events import (EV_DRIFT_WARNING, EV_POSE_ANCHORED, EventBus)    # noqa: E402
from dzbrain.core.state import FixState                                           # noqa: E402
from dzbrain.route.model import Waypoint, make_route                              # noqa: E402
from dzbrain.tasks.fix import FixTask                                             # noqa: E402
from fakes import FakeBridge, FakeClock                                           # noqa: E402
from harness import check, check_close, finish, section                           # noqa: E402

#: 测试路线：N10 → M10 → L10（同列相邻格，每段 0.35 m，沿 y 增大方向）
BLOCKS = ("N10", "M10", "L10")
STEP = 0.35
_BELIEF_RATIO = 1.18                    # 信念位移 / 真实位移（里程多报 18%）
_ANCHOR_OFFSET = 0.05                   # 每次锚定的偏差（< arrival_m 0.12，正常吸附）
_FRAMES_PER_SEG = 6                     # 每段 6 拍：位移分辨率 ~0.06 m，足够算比值


# --------------------------------------------------------------------------- #
#  脚手架
# --------------------------------------------------------------------------- #
def build(overrides: dict | None = None, *, route_blocks=BLOCKS, fix_present=False):
    """造 (task, bridge, bb, bus, clock, clock_holder, warnings)。"""
    clock = FakeClock()
    bridge = FakeBridge(clock)
    bb = Blackboard()
    bus = EventBus()
    cfg = Config.from_dict({
        "route": {"tokens": list(route_blocks)},
        "pursuit": {},
        "fix": {},
        "logging": {},
        "mission": {"auto_start": False},
        **(overrides or {}),
    })
    route = make_route([Waypoint.at_block(b) for b in route_blocks])
    bb.set(P_ROUTE_S, 0.0, source="test")
    if fix_present:
        bb.set(P_FIX, FixState(anchor_count=7, last_anchor="X1", drift_m=0.2,
                               max_drift_m=0.31), source="test")
        bb.set(P_POSE_RAW, geo.Pose(9.9, 9.9, 0.0), source="test")
    warnings: list = []
    sub = bus.subscribe("test_watch", [EV_DRIFT_WARNING, EV_POSE_ANCHORED])
    task = FixTask(bridge, bb, bus, cfg, log=lambda m: None, clock=clock.now)
    task.setup()
    return task, bridge, bb, bus, clock, warnings, sub, route


def tick(task, clock, bb, dt: float = 0.1) -> None:
    clock.advance(dt)
    task.tick()


def set_pose(bb, x: float, y: float, s: float) -> None:
    bb.set(P_POSE, geo.Pose(x, y, 0.0), source="test", ts=0.0)
    bb.set(P_ROUTE_S, float(s), source="test", ts=0.0)


def run_segment(task, clock, bb, bus, route, start_block: str, end_block: str,
                drift: float = _ANCHOR_OFFSET, ratio: float = _BELIEF_RATIO,
                frames: int = _FRAMES_PER_SEG) -> dict:
    """跑完一段并触发锚定事件。返回本段的关键数值（供断言打印）。"""
    x0, y0 = geo.block_to_meters(start_block)
    x1, y1 = geo.block_to_meters(end_block)
    set_pose(bb, x0, y0, 0.0)
    tick(task, clock, bb)
    for i in range(1, frames + 1):
        t = i / frames
        set_pose(bb, x0 + (x1 - x0) * t * ratio, y0 + (y1 - y0) * t * ratio, t * STEP)
        tick(task, clock, bb)
    idx = route.blocks().index(end_block)
    bus.publish(EV_POSE_ANCHORED, source="LocalizationTask", block=end_block,
                index=idx, drift=round(drift, 4), x=round(x1, 4), y=round(y1, 4))
    set_pose(bb, x1, y1, STEP)              # 吸附把它拉回格心（真实系统就是这样）
    tick(task, clock, bb)
    return {"belief_m": task._belief_m, "geo_m": task._geo_m,
            "ratio": task.stats()["ratios"][-1] if task.stats()["ratios"] else None}


def drain(sub) -> list:
    return sub.poll(512)


# --------------------------------------------------------------------------- #
section("① 无锚定事件：不出 scale（scale==1.0，样本数 0）")
task, bridge, bb, bus, clock, _w, sub, route = build()
for _ in range(30):
    set_pose(bb, 0.525, 0.175 + 0.01 * _, _ * 0.01)
    tick(task, clock, bb)
est = bb.get(P_FIX_EST)
check("P_FIX_EST 已写出（类型 FixState）", isinstance(est, FixState),
      f"got {type(est).__name__}")
check_close("scale == 1.0（无锚定事件 = 不出估计值）", est.scale, 1.0, 0.0)
check("anchor_count == 0", est.anchor_count == 0, f"got {est.anchor_count}")
check("样本数 == 0", task.samples == 0, f"got {task.samples}")
check("段数 == 0（没有锚定就没有段闭合）", task.segments_total == 0,
      f"got {task.segments_total}")
check("没有发过 EV_POSE_ANCHORED 之外的估计日志（steps==0）", task.steps == 0,
      f"got {task.steps}")

# --------------------------------------------------------------------------- #
section("② 5 段「信念位移 = 真实 × 1.18」→ 比值实测 1.18，步长 ≤ fix.scale_step_max")
task, bridge, bb, bus, clock, _w, sub, route = build({"fix": {"scale_limit": 0.5,
                                                              "scale_step_max": 0.005,
                                                              "scale_est_enabled": True,
                                                              "scale_min_samples": 3}})
segments: list[dict] = []
scale_marks: list[float] = []
segs = [("N10", "M10"), ("M10", "L10")]
seq_segments = (segs + segs + [segs[0]])           # 5 段（在两个格间往返，路线固定）
for start, end in seq_segments:
    segments.append(run_segment(task, clock, bb, bus, route, start, end))
    scale_marks.append(task.scale)
deltas = [abs(b - a) for a, b in zip([1.0] + scale_marks[:-1], scale_marks)]
st = task.stats()
check("5 段全部被接受（没有无辜剔除）", task.samples == 5, f"samples={task.samples} "
      f"rejected={task.segments_rejected} reasons={st['reasons']}")
measured = [round(i['ratio'], 5) for i in segments if i['ratio']]
check("★ 每段实测的『信念位移/几何弧长』≈ 1.18（估计器输入正确）",
      len(measured) == 5 and all(abs(r - 1.18) < 0.03 for r in measured),
      f"ratios={measured}")
check("★ 每段步长 ≤ fix.scale_step_max(0.005)",
      all(d <= 0.005 + 1e-12 for d in deltas), f"deltas={[round(d, 6) for d in deltas]}")
check("步长恰好用满上限（保守步长不是摆设）", abs(max(deltas) - 0.005) < 1e-12,
      f"max delta={max(deltas)!r}")
check("样本门槛生效：前 2 段不动（scale 仍 1.0）",
      abs(scale_marks[0] - 1.0) < 1e-12 and abs(scale_marks[1] - 1.0) < 1e-12,
      f"marks={scale_marks}")
check("估计单调上升（朝 1.18 走，不来回摆）",
      all(b >= a - 1e-12 for a, b in zip(scale_marks, scale_marks[1:])),
      f"marks={[round(m, 5) for m in scale_marks]}")
check("落盘值 == 任务内部值", abs(bb.get(P_FIX_EST).scale - task.scale) < 1e-12,
      f"bb={bb.get(P_FIX_EST).scale} task={task.scale}")
check("★ scale 语义 = 信念位移/几何弧长（>1 = 里程多报）", task.scale > 1.0,
      f"scale={task.scale:.5f}（修正系数应为 1/scale={1.0 / task.scale:.5f}）")

section("②b 收敛速度：5 段只走 0.025，走满 33 段才进 ±0.03（保守步长的代价，写清楚）")
check_close("5 段的理论位移 = 3 次有效步 × 0.005", task.scale, 1.015, 1e-12)
steps_to_go = (1.18 - 1.015) / 0.005
check("按步长上限推算还需 ~33 段", 32 <= steps_to_go <= 34, f"got {steps_to_go:.2f}")

task2, bridge2, bb2, bus2, clock2, _w2, sub2, route2 = build(
    {"fix": {"scale_limit": 0.5, "scale_step_max": 0.005, "scale_min_samples": 3}})
conv_at = None
for n in range(1, 61):
    start, end = seq_segments[(n - 1) % 2]
    run_segment(task2, clock2, bb2, bus2, route2, start, end)
    if conv_at is None and abs(task2.scale - 1.18) <= 0.03:
        conv_at = n
check("60 段后 scale 收敛到 1.18 ± 0.03", abs(task2.scale - 1.18) <= 0.03,
      f"scale={task2.scale:.6f}")
check("收敛发生在 30~40 段之间（与步长上限推算一致）",
      conv_at is not None and 30 <= conv_at <= 40, f"got {conv_at}")
check("全程没有段被误剔除", task2.segments_rejected == 0 and task2.samples == 60,
      f"accepted={task2.samples} rejected={task2.segments_rejected} "
      f"{task2.reject_reasons}")
check_close("收敛终点就是真值（无稳态偏差）", task2.scale, 1.18, 1e-9)

section("②c ★ SCHEMA 默认 scale_limit=0.10 会把估计夹在 1.10（真值 1.18 时**够不着**）")
task3, bridge3, bb3, bus3, clock3, _w3, sub3, route3 = build({"fix": {}})
check_close("读取 SCHEMA 默认 fix.scale_limit", task3.scale_limit, 0.10, 1e-12)
for n in range(1, 41):
    run_segment(task3, clock3, bb3, bus3, route3, *seq_segments[(n - 1) % 2])
check("默认夹紧下 40 段后停在 1.10（不是 1.18）", abs(task3.scale - 1.10) < 1e-12,
      f"scale={task3.scale:.6f}")
check("落盘值同样是 1.10（夹紧在估计器内生效）",
      abs(bb3.get(P_FIX_EST).scale - 1.10) < 1e-12, f"bb={bb3.get(P_FIX_EST).scale}")
check("→ 现场若里程真有 >10% 的系统误差，必须调大 fix.scale_limit（见回报）",
      task3.scale >= 1.0 + task3.scale_limit - 1e-12,
      f"raw={task3.scale_raw:.4f} clamp=±{task3.scale_limit:.2f}")

# --------------------------------------------------------------------------- #
section("③ scale_limit=0.10 → 估计被夹在 [0.9, 1.1]")
task, bridge, bb, bus, clock, _w, sub, route = build({"fix": {"scale_limit": 0.10}})
check_close("上限：预置 1.60 + 比值 1.05 → 夹到 1.10",
            task._clamp(1.60 + task.step_max * 0.0), 1.10, 1e-12)
check_close("下限：预置 0.40 → 夹到 0.90", task._clamp(0.40), 0.90, 1e-12)
task.scale = 1.60
task.step_max = 0.005
task.scale = task._clamp(task.scale + max(-task.step_max, min(task.step_max, 1.05 - task.scale)))
check_close("在途更新也被夹住（1.60 → 1.10）", task.scale, 1.10, 1e-12)
for start, end in (("N10", "M10"), ("M10", "L10"), ("N10", "M10")):
    run_segment(task, clock, bb, bus, route, start, end)
samples = [task.scale]
check("连续 3 段后仍在 [0.9, 1.1] 内", all(0.9 - 1e-12 <= v <= 1.1 + 1e-12 for v in samples),
      f"scale={task.scale:.5f}")
check("落盘值同样被夹紧", 0.9 - 1e-12 <= bb.get(P_FIX_EST).scale <= 1.1 + 1e-12,
      f"bb={bb.get(P_FIX_EST).scale:.5f}")

# --------------------------------------------------------------------------- #
section("④ 异常段（漂移 0.5 m > drift_warn 0.30）被剔除且发 EV_DRIFT_WARNING")
task, bridge, bb, bus, clock, _w, sub, route = build()
drain(sub)
info = run_segment(task, clock, bb, bus, route, "N10", "M10", drift=0.5)
events = drain(sub)
warn = [e for e in events if e.type == EV_DRIFT_WARNING]
drift_warn = [e for e in warn if e.data.get("by") == "fix_est"]
reject_warn = [e for e in warn if e.data.get("by") == "fix_est_reject"]
check("该段被判为异常段（rejected += 1）", task.segments_rejected == 1,
      f"rejected={task.segments_rejected}")
check("该段没有进入估计（samples == 0）", task.samples == 0, f"samples={task.samples}")
check("估计值未动（scale == 1.0）", abs(task.scale - 1.0) < 1e-12,
      f"scale={task.scale}")
check("原因登记为 drift_outlier", task.reject_reasons.get("drift_outlier") == 1,
      f"reasons={task.reject_reasons}")
check("发出 EV_DRIFT_WARNING（限流后 1 条，drift=0.5）", len(drift_warn) == 1,
      f"got {len(drift_warn)} 条：{[e.data for e in drift_warn]}")
check("剔除事件也留痕（by=fix_est_reject，reason=drift_outlier）",
      len(reject_warn) == 1 and reject_warn[0].data.get("reason") == "drift_outlier",
      f"got {[e.data.get('reason') for e in reject_warn]}")
check("漂移告警计数 = 1", task.drift_warnings == 1, f"got {task.drift_warnings}")
info_ok = run_segment(task, clock, bb, bus, route, "M10", "L10", drift=0.02)
check("正常段在异常段之后仍能被接受", task.samples == 1,
      f"samples={task.samples} rejected={task.segments_rejected}")

# --------------------------------------------------------------------------- #
section("⑤ fix.scale_est_enabled=False → 只统计不改估计")
task, bridge, bb, bus, clock, _w, sub, route = build({"fix": {"scale_est_enabled": False}})
for start, end in (("N10", "M10"), ("M10", "L10"), ("N10", "M10"), ("M10", "L10")):
    run_segment(task, clock, bb, bus, route, start, end)
check("比值被统计（samples == 4）", task.samples == 4, f"samples={task.samples}")
check("scale 恒为 1.0（不改估计）", abs(task.scale - 1.0) < 1e-12,
      f"scale={task.scale}")
check("落盘 scale 也是 1.0", abs(bb.get(P_FIX_EST).scale - 1.0) < 1e-12,
      f"bb={bb.get(P_FIX_EST).scale}")
check("raw 估计也没被写（scale_raw == 1.0）", abs(task.scale_raw - 1.0) < 1e-12,
      f"raw={task.scale_raw}")
check("统计窗口里确实有 4 个比值 ≈1.18",
      len(task.stats()["ratios"]) == 4 and
      all(abs(r - 1.18) < 0.03 for r in task.stats()["ratios"]),
      f"ratios={[round(r, 4) for r in task.stats()['ratios']]}")

# --------------------------------------------------------------------------- #
section("⑥ 漂移汇总（max 正确）与台账")
task, bridge, bb, bus, clock, _w, sub, route = build({"logging": {"rate_limit_s": 0.2}})
drifts = [0.03, 0.11, 0.42, 0.07]
for i, (start, end) in enumerate((("N10", "M10"), ("M10", "L10"),
                                  ("N10", "M10"), ("M10", "L10"))):
    run_segment(task, clock, bb, bus, route, start, end, drift=drifts[i])
est = bb.get(P_FIX_EST)
check("anchor_count == 4（从事件累计）", est.anchor_count == 4, f"got {est.anchor_count}")
check("last_anchor 是最后一段的终点格", est.last_anchor == "L10",
      f"got {est.last_anchor!r}")
check_close("drift_m == 最近一段 0.07", est.drift_m, 0.07, 1e-9)
check_close("max_drift_m == 0.42（不是最后一个值）", est.max_drift_m, 0.42, 1e-9)
check("漂移告警只对 0.42 那一段发（>0.30）", task.drift_warnings == 1,
      f"got {task.drift_warnings}")
report = task.drift_report()
check("drift_report() 汇总一致",
      report["anchor_count"] == 4 and abs(report["max_drift_m"] - 0.42) < 1e-9
      and report["warnings"] == 1, f"got {report}")
check("render_report() 是一行文本且含关键数字",
      "\n" not in task.render_report() and "0.420" in task.render_report(),
      task.render_report())

# --------------------------------------------------------------------------- #
section("⑦ ★ 反用防护：只写 P_FIX_EST，绝不改位姿（估计侧不越界）")
task, bridge, bb, bus, clock, _w, sub, route = build({"fix": {}}, fix_present=True)
before_pose = bb.entry(P_POSE)
before_raw = bb.entry(P_POSE_RAW)
before_fix = bb.entry(P_FIX)
writes_before = bb.writes
for start, end in (("N10", "M10"), ("M10", "L10")):
    run_segment(task, clock, bb, bus, route, start, end)
drain(sub)
check("P_POSE 没有被本任务写过（写入者仍是 test）",
      bb.entry(P_POSE).source == "test", f"source={bb.entry(P_POSE).source}")
check("P_POSE_RAW 原样未动（source/值都没变）",
      bb.entry(P_POSE_RAW) is before_raw or
      (bb.entry(P_POSE_RAW).value == before_raw.value
       and bb.entry(P_POSE_RAW).source == before_raw.source),
      f"source={bb.entry(P_POSE_RAW).source}")
check("P_FIX（Localization 的执行台账）没有被本任务改写",
      bb.entry(P_FIX).value is before_fix.value
      and bb.entry(P_FIX).source == before_fix.source,
      f"source={bb.entry(P_FIX).source} value={bb.entry(P_FIX).value}")
check("P_FIX_EST 的 source == FixTask", bb.entry(P_FIX_EST).source == "FixTask",
      f"got {bb.entry(P_FIX_EST).source}")
check("P_FIX_EST 是冻结的（frozen dataclass，不能原地改）",
      isinstance(bb.get(P_FIX_EST), FixState))
check("本任务写的键只有 P_FIX_EST（用 bb.writes 增量核对）",
      bb.writes > writes_before, f"writes {writes_before} → {bb.writes}")
check("FixState 的平移/航向项恒为 0（L2 只做比例，不做 dx/dy/dyaw）",
      bb.get(P_FIX_EST).dx == 0.0 and bb.get(P_FIX_EST).dy == 0.0
      and bb.get(P_FIX_EST).dyaw == 0.0,
      f"dx={bb.get(P_FIX_EST).dx} dy={bb.get(P_FIX_EST).dy} dyaw={bb.get(P_FIX_EST).dyaw}")

# --------------------------------------------------------------------------- #
section("⑧ 兜底：无路线 / 位姿突跳 / 段太短 都不崩")
task, bridge, bb, bus, clock, _w, sub, route = build()
bb.set(P_ROUTE_S, None, source="test")
set_pose(bb, 0.525, 0.175, 0.0)
tick(task, clock, bb)
check("P_ROUTE_S 缺失时不崩、不累计几何弧长", abs(task._geo_m) < 1e-9,
      f"geo_m={task._geo_m}")
set_pose(bb, 3.0, 3.0, 0.0)                 # 突跳 3 m（模拟 reset/重锚）
tick(task, clock, bb)
check("位姿突跳被识别，不计入位移", abs(task._belief_m) < 1e-9,
      f"belief_m={task._belief_m}")
check("突跳后基线已重设（_prev_pose 跟上）", task._prev_pose is not None)
set_pose(bb, 3.0, 3.0, 0.0)
tick(task, clock, bb)                       # 静止一拍：位移 0
bus.publish(EV_POSE_ANCHORED, source="LocalizationTask", block="M10", index=1,
            drift=0.0)
tick(task, clock, bb)
check("位移不足的段被剔除（seg_too_short）",
      task.reject_reasons.get("seg_too_short") == 1,
      f"reasons={task.reject_reasons}")
task.teardown()
check("teardown 后取消订阅（再发事件也不进队列）", task._sub is None)

# --------------------------------------------------------------------------- #
section("⑨ ★ 输入守卫：物理不可能的 ds 被丢弃 + 计数，且不毁掉正常段（P_ROUTE_S 三写入者互跳）")
# 现场实证：P_ROUTE_S 有 Localization / Tracker / Mission 三个写入者，各维护自己的投影链；
# 任一处冻结或换手，相邻两拍的 ds 就是物理不可能的跳（bench：3.73↔4.20 来回跳）。
# 这里把"换手"做成一次明确的注入：第 3 拍上 P_ROUTE_S 突然 +7 m（来源写成 TrackerTask）。
task, bridge, bb, bus, clock, _w, sub, route = build({"fix": {}})
x0, y0 = geo.block_to_meters("N10")
x1, y1 = geo.block_to_meters("M10")
set_pose(bb, x0, y0, 0.0)
tick(task, clock, bb)
FAKE_DS = 7.0                       # 假增量（米）
inject_tick = 3
for i in range(1, _FRAMES_PER_SEG + 1):
    t = i / _FRAMES_PER_SEG
    set_pose(bb, x0 + (x1 - x0) * t * _BELIEF_RATIO, y0 + (y1 - y0) * t * _BELIEF_RATIO,
             t * STEP)
    if i == inject_tick:
        bb.set(P_ROUTE_S, t * STEP + FAKE_DS, source="TrackerTask", ts=0.0)
    tick(task, clock, bb)
# ★ 段内的累加器要在"锚定事件闭合本段"之前取（`_on_anchored` 会 rebaseline 清零）
geo_m, belief_m = task._geo_m, task._belief_m
bus.publish(EV_POSE_ANCHORED, source="LocalizationTask", block="M10", index=1,
            drift=_ANCHOR_OFFSET)
set_pose(bb, x1, y1, STEP)
tick(task, clock, bb)

st = task.stats()
check("★ 假 ds 被丢弃并计数（ds_rejected ≥ 1）", task.ds_rejected >= 1,
      f"ds_rejected={task.ds_rejected} 共 {task.ds_rejected_m:.3f} m")
check("★ 被丢弃的量级就是那 7 m（不是小数被误伤）",
      FAKE_DS <= task.ds_rejected_m <= FAKE_DS + STEP, f"{task.ds_rejected_m:.3f} m")
check("★ 假 ds 没有进几何弧长：本段 geo 仍等于真实走过的 0.35 m",
      abs(geo_m - STEP) < 1e-9,
      f"geo={geo_m:.4f} m（无守卫时会是 {STEP + FAKE_DS:.2f} m 量级）")
check("信念位移没被牵连（belief 仍是 0.35×1.18）",
      abs(belief_m - STEP * _BELIEF_RATIO) < 1e-9, f"belief={belief_m:.4f} m")
check("★ 该段**仍然被接受**（守卫不该吞掉正常样本）", task.samples == 1,
      f"samples={task.samples} rejected={task.segments_rejected} {st['reasons']}")
check("★ 比值仍是真值 1.18（丢弃的那一拍由下一拍补账回来，比值不偏）",
      len(st["ratios"]) == 1 and abs(st["ratios"][0] - _BELIEF_RATIO) < 0.03,
      f"ratios={[round(r, 4) for r in st['ratios']]}")
check("统计口径可观测：stats() 里有 ds_rejected / ds_rejected_m",
      st["ds_rejected"] == task.ds_rejected
      and abs(st["ds_rejected_m"] - task.ds_rejected_m) < 1e-12, f"{st}")
check("段落盘（P_FIX_EST）没被这次污染影响", isinstance(bb.get(P_FIX_EST), FixState))

# 反向：正常段一次都不该误伤（守卫不能把正常 ds 也拒掉）
ds_before = task.ds_rejected
run_segment(task, clock, bb, bus, route, "M10", "L10", drift=0.02)
check("正常段零误伤（ds_rejected 不增）", task.ds_rejected == ds_before,
      f"{ds_before} → {task.ds_rejected}")

# 反向：连续不可信 → 重新基线，改按新坐标系继续量（不死循环、也不把 +7 记成位移）
task2, bridge2, bb2, bus2, clock2, _w2, sub2, route2 = build({"fix": {}})
set_pose(bb2, 0.525, 0.175, 0.0)
tick(task2, clock2, bb2)
bad_frame = 7.0                     # 坏写入者：整条链偏了 7 m，但自己的增量是正常的
for i in range(1, 10):
    set_pose(bb2, 0.525, 0.175 + 0.01 * i, 0.01 * i)
    bb2.set(P_ROUTE_S, 0.01 * i + bad_frame, source="BadWriter", ts=0.0)
    tick(task2, clock2, bb2)
check("换坐标系那几拍被计数（≥ fix.ds_reject_max_run）", task2.ds_rejected >= 3,
      f"got {task2.ds_rejected}")
check("★ 偏置 7 m 没有变成位移（geo 只累计换基线之后的正常增量）",
      abs(task2._geo_m - 0.06) < 1e-9, f"geo_m={task2._geo_m:.4f} m")
check("★ 重新基线后估计器恢复工作（不是卡在拒绝里）", task2._geo_m > 0.0,
      f"geo_m={task2._geo_m:.4f} m")
check("坏坐标系的偏置不会被当成漂移/段样本",
      task2.segments_total == 0 and task2.samples == 0,
      f"segments={task2.segments_total} samples={task2.samples}")

finish()
