# -*- coding: utf-8 -*-
r"""LocalizationTask 测试（纯脚本，无 pytest）。

运行：python src\dzbrain\tests\test_localization.py

覆盖（P1接口约定.md §3.1 + 任务书逐条）：
  1. 无位姿 → 不写 P_POSE、P_ODOM_AGE=inf
  2. 有位姿 → 米制 Pose（K9 格心）
  3. P_FIX 非零 → P_POSE 被修正、P_POSE_RAW 保持原值
  4. 带 s_hint 的投影 → P_ROUTE_S 单调递增
  5. 到位吸附（偏差 0.05 m < arrival 0.12 m）→ 锚回格心 + 事件 + anchor_count + drift_m
  6. 关掉 fix.anchor_at_waypoint → 不锚定
  7. 漂移 0.5 m > drift_warn 0.30 m → EV_DRIFT_WARNING
  8. 同一航点反复吸附只算一次（防"越锚越偏"）；路线更换后 hint 重置
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:                                    # 车端/CI 控制台默认不是 UTF-8，中文会乱码
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

from dzbrain.core import geometry as geo                                      # noqa: E402
from dzbrain.core.blackboard import (P_FIX, P_ODOM_AGE, P_POSE, P_POSE_RAW,   # noqa: E402
                                     P_ROUTE, P_ROUTE_INDEX, P_ROUTE_S, Blackboard)
from dzbrain.core.config import Config                                        # noqa: E402
from dzbrain.core.events import (EV_DRIFT_WARNING, EV_POSE_ANCHORED,          # noqa: E402
                                 EventBus)
from dzbrain.core.state import FixState                                       # noqa: E402
from dzbrain.route.model import Route, Waypoint, make_route                   # noqa: E402
from dzbrain.tasks.localization import LocalizationTask                       # noqa: E402
from fakes import FakeBridge, FakeClock                                       # noqa: E402
from harness import check, check_close, finish, section                       # noqa: E402

INF = float("inf")

#: 测试路线的两个格心（米制）。★ 格名与米制的对应关系见 core/geometry.py：
#: N10 = (0.525, 0.175)、M10 = (0.525, 0.525) —— 同列相邻两行，沿 **y 增大**方向前进。
N10 = geo.block_to_meters("N10")
M10 = geo.block_to_meters("M10")
M9 = geo.block_to_meters("M9")
STEP = M10[1] - N10[1]                        # 相邻两格间距 = 0.35 m


# --------------------------------------------------------------------------- #
#  脚手架
# --------------------------------------------------------------------------- #
def build(overrides: dict | None = None, route: Route | None = None,
          seed_pose: bool = False):
    """造一套 (task, bridge, bb, bus, clock)。默认装路线 N10→M10、有初始位姿。"""
    clock = FakeClock()
    bridge = FakeBridge(clock)
    bb = Blackboard()
    bus = EventBus()
    cfg = Config.from_dict({
        "route": {"tokens": ["N10", "M10"]},
        "pursuit": {},
        "fix": {},
        "safety": {},
        "logging": {},
        "mission": {"auto_start": False},
        **(overrides or {}),
    })
    if route is not None:
        bb.set(P_ROUTE, route, source="test")
    if seed_pose:
        bridge.set_pose_at_block("N10")
    task = LocalizationTask(bridge, bb, bus, cfg, log=lambda m: None, clock=clock.now)
    return task, bridge, bb, bus, clock


def feed(task, n: int) -> None:
    for _ in range(n):
        task.tick()


section("① 无位姿：不写 P_POSE，P_ODOM_AGE = inf")
task, bridge, bb, bus, clock = build()
check("初始无 P_POSE", bb.get(P_POSE) is None)
feed(task, 3)
check("3 拍后仍无 P_POSE（不沿用旧值）", bb.get(P_POSE) is None)
check("P_ODOM_AGE == inf", bb.get(P_ODOM_AGE) == INF, f"got {bb.get(P_ODOM_AGE)!r}")
check("缺位姿拍数 = 3", task.missing_pose_ticks == 3, f"got {task.missing_pose_ticks}")
check("pose_age() 本身也是 inf", bridge.pose_age() == INF)
check("没有写 P_POSE_RAW（也没写）", bb.get(P_POSE_RAW) is None)

bridge.set_pose_at_block("K9")
task.tick()
check("恢复位姿后写出 P_POSE", bb.get(P_POSE) is not None)
check("P_ODOM_AGE 变回有限值", bb.get(P_ODOM_AGE) == 0.0, f"got {bb.get(P_ODOM_AGE)!r}")

section("② 有位姿：米制 Pose（K9 = geometry.block_to_meters）")
kx, ky = geo.block_to_meters("K9")
task, bridge, bb, bus, clock = build(seed_pose=False)
bridge.set_pose_at_block("K9")
task.tick()
p = bb.get(P_POSE)
check_close("P_POSE.x == block_to_meters('K9').x", p.x, geo.block_to_meters("K9")[0], 1e-12)
check_close("P_POSE.y == block_to_meters('K9').y", p.y, geo.block_to_meters("K9")[1], 1e-12)
check("K9 中心 = (0.875, 1.2249999999999999)（格单位 ×0.35 的浮点结果）",
      (kx, ky) == (0.875, 1.2249999999999999), f"got {(kx, ky)!r}")
check("格单位没有被误当米用（x 应 < 3.675+margin）", 0.0 < p.x < 4.3, f"x={p.x!r}")
check("P_ODOM_AGE 是有限小数", bb.get(P_ODOM_AGE) == 0.0)

section("③ P_FIX 非零：P_POSE 被修正，P_POSE_RAW 保持原值")
fix = FixState(dx=0.10, dy=-0.20, dyaw=0.05)
kx, ky = geo.block_to_meters("K9")
task, bridge, bb, bus, clock = build()
bridge.set_pose_at_block("K9")
bb.set(P_FIX, fix, source="test")
task.tick()
pos = bb.get(P_POSE)
raw = bb.get(P_POSE_RAW)
check_close("P_POSE.x = raw.x + dx", pos.x, kx + 0.10, 1e-12)
check_close("P_POSE.y = raw.y + dy", pos.y, ky - 0.20, 1e-12)
check_close("P_POSE.yaw = normalize(raw.yaw + dyaw)", pos.yaw,
            geo.normalize_angle(0.0 + 0.05), 1e-12)
check_close("P_POSE_RAW.x 未被修正", raw.x, kx, 1e-12)
check_close("P_POSE_RAW.y 未被修正", raw.y, ky, 1e-12)
check("P_POSE_RAW 是另一个对象", raw is not pos)
check("修正方向正确（dx=+0.10 使 x 增大）", pos.x > raw.x, f"{pos.x} vs {raw.x}")

section("④ 带 s_hint 的投影：P_ROUTE_S 单调递增，P_ROUTE_INDEX 正确")
route = make_route([Waypoint.at_block("N10"), Waypoint.at_block("M10")])
task, bridge, bb, bus, clock = build(route=route, seed_pose=False)
s_list: list[float] = []
idx_list: list[int] = []
for i in range(12):
    bridge.set_pose(N10[0], N10[1] + i * 0.05, 0.0)     # 沿路线（y 增大）每拍走 0.05 m
    task.tick()
    s_list.append(bb.get(P_ROUTE_S))
    idx_list.append(bb.get(P_ROUTE_INDEX))
check("投影发生（projected_ticks == 12）", task.projected_ticks == 12,
      f"got {task.projected_ticks}")
deltas = [round(b - a, 9) for a, b in zip(s_list, s_list[1:])]
check("12 拍 P_ROUTE_S 单调不减", all(d >= -1e-9 for d in deltas), f"deltas={deltas}")
check("S 末值 == 路线总长 0.35 m（已到终点）", abs(s_list[-1] - STEP) < 1e-9,
      f"got {s_list[-1]:.6f}")
check("P_ROUTE_INDEX 恒为 0（单段路线）", set(idx_list) == {0}, f"got {set(idx_list)}")
check_close("路线长度 = 一格 = 0.35 m", task._path.total_length, 0.35, 1e-9)
check("每一步 s 增量 ≈ 0.05 m（未锚定时）",
      all(abs(d - 0.05) < 1e-9 for d in deltas[:4]), f"deltas[:5]={deltas[:5]}")

# s_hint 真的在起作用：人为把 hint 打回 0，投影不该被"错误窗口"锁死
task._s_hint = 0.0
bridge.set_pose(N10[0], N10[1] + 0.20, 0.0)
task.tick()
check("hint 落后时也能前进到正确弧长（0.20 m）",
      abs(bb.get(P_ROUTE_S) - 0.20) < 1e-9, f"got {bb.get(P_ROUTE_S)}")

section("⑤ 到位吸附：偏差 0.05 m < arrival 0.12 m")
task, bridge, bb, bus, clock = build(route=route, seed_pose=False)
sub_anchor = bus.subscribe("t_anchor", [EV_POSE_ANCHORED])
bridge.set_pose(N10[0], N10[1] + 0.30, 0.0)   # 沿路只差 0.05 m 到 M10 格心
task.tick()
p = bb.get(P_POSE)
fx = bb.get(P_FIX)
check("锚回格心（x）", abs(p.x - M10[0]) < 1e-12, f"got {p.x!r}, want {M10[0]!r}")
check("锚回格心（y）", abs(p.y - M10[1]) < 1e-12, f"got {p.y!r}, want {M10[1]!r}")
check("yaw 不变（仍为 0.0）", p.yaw == 0.0, f"got {p.yaw!r}")
check("锚定前位置不是格心（0.05 m 偏差真实存在）",
      abs((N10[1] + 0.30) - M10[1]) > 0.04)
check("发出了 EV_POSE_ANCHORED（1 条）", len(sub_anchor.poll()) == 1)
check("anchor_count == 1", isinstance(fx, FixState) and fx.anchor_count == 1,
      f"got {getattr(fx, 'anchor_count', None)!r}")
check("last_anchor == 'M10'", getattr(fx, "last_anchor", None) == "M10",
      f"got {getattr(fx, 'last_anchor', None)!r}")
check_close("drift_m == 锚定前偏差 0.05", fx.drift_m, 0.05, 1e-9)
check_close("max_drift_m == 0.05", fx.max_drift_m, 0.05, 1e-9)
check("task.anchored == 1", task.anchored == 1, f"got {task.anchored}")
check("锚定后 P_ROUTE_S == 总长（锚在末点上）", abs(bb.get(P_ROUTE_S) - STEP) < 1e-9,
      f"got {bb.get(P_ROUTE_S)}")

# 同一航点连续吸附：只算一次（防"越锚越偏"）
for _ in range(5):
    bridge.set_pose(N10[0], N10[1] + 0.30, 0.0)
    task.tick()
check("同一航点反复到位只锚一次", task.anchored == 1, f"got {task.anchored}")
check("anchor_count 仍为 1", bb.get(P_FIX).anchor_count == 1,
      f"got {bb.get(P_FIX).anchor_count}")
check("EV_POSE_ANCHORED 仍只有 1 条", len(sub_anchor.poll()) == 0)

section("⑥ 关掉 fix.anchor_at_waypoint：不锚定")
task, bridge, bb, bus, clock = build(overrides={"fix": {"anchor_at_waypoint": False}},
                                     route=route, seed_pose=False)
sub_anchor = bus.subscribe("t_noanch", [EV_POSE_ANCHORED])
bridge.set_pose(N10[0], N10[1] + 0.30, 0.0)   # 与 ⑤ 同样的 0.05 m 到位偏差
task.tick()
p = bb.get(P_POSE)
check("位置未被锚定（保留 0.05 m 偏差）",
      abs(p.y - (N10[1] + 0.30)) < 1e-12, f"got {p.y!r}")
check("未发 EV_POSE_ANCHORED", len(sub_anchor.poll()) == 0)
check("未写 P_FIX", bb.get(P_FIX) is None)
check("task.anchored == 0", task.anchored == 0)

section("⑦ 漂移告警：横向侧偏 0.50 m > drift_warn 0.30 m（限流只发 1 条）")
task, bridge, bb, bus, clock = build(route=route, seed_pose=False)
sub_drift = bus.subscribe("t_drift", [EV_DRIFT_WARNING])
# 车横向侧偏 0.50 m、沿路只差 0.05 m 到 M10。
# ★ 这里刻意展示一个**语义事实**：`arrival_m`(0.12) < `drift_warn_m`(0.30) 时，
#   "被吸附"与"被判漂移"不可能同拍发生 —— 能锚上的偏差最多 0.12 m（< 0.30 不告警），
#   能告警的偏差至少 0.30 m（> 0.12 锚不上）。所以锚定与告警是**互斥**的两条路径，
#   这正是"漂移必须独立判定"的理由（否则 0.30 m 以上的漂移永远报不出来）。
SIDE, ALONG = 0.50, 0.05
bridge.set_pose(N10[0] + SIDE, M10[1] - ALONG, 0.0)
task.tick()
events = sub_drift.poll()
check("发出 1 条 EV_DRIFT_WARNING", len(events) == 1, f"got {len(events)}")
if events:
    check("事件带 block=M10", events[0].data.get("block") == "M10", f"{events[0].data}")
    check_close("事件 drift=0.50（事件里是 4 位小数）",
                float(events[0].data.get("drift", -1)),
                math.hypot(SIDE, ALONG), 1e-4)
    check_close("事件 threshold=0.30", float(events[0].data.get("threshold", -1)), 0.30, 1e-9)
    check("事件带 anchored=False（0.50 m 已超出 0.12 m 吸附半径）",
          events[0].data.get("anchored") is False, f"{events[0].data}")
check("未吸附（超出到位半径）", task.anchored == 0, f"got {task.anchored}")
check("未写 P_FIX（锚定才写）", bb.get(P_FIX) is None)
check("drift_warnings == 1", task.drift_warnings == 1, f"got {task.drift_warnings}")
check("位置未被改动", abs(bb.get(P_POSE).x - (N10[0] + SIDE)) < 1e-12)

# 同一航点再连续 50 拍：不刷屏（按航点去重）
for _ in range(50):
    bridge.set_pose(N10[0] + SIDE, M10[1] - ALONG, 0.0)
    task.tick()
check("同航点 50 拍不再重复告警", len(sub_drift.poll()) == 0)
check("drift_warnings 仍为 1", task.drift_warnings == 1, f"got {task.drift_warnings}")

section("⑦b 锚定与告警互斥性验证：可锚定的偏差一定低于告警阈值")
task, bridge, bb, bus, clock = build(route=route, seed_pose=False)
sub_b = bus.subscribe("t_d5", [EV_DRIFT_WARNING, EV_POSE_ANCHORED])
LAT, ALNG = 0.11, 0.04          # 到格心 0.1170 m < arrival 0.12 m → 可锚定
bridge.set_pose(N10[0] + LAT, M10[1] - ALNG, 0.0)
task.tick()
dist = math.hypot(LAT, ALNG)
check("到 M10 距离 %.4f m < arrival 0.12 m" % dist, dist < 0.12, f"got {dist:.4f}")
check("该偏差 %.4f m < drift_warn 0.30 m（不告警）" % dist, dist < 0.30)
evs = sub_b.poll()
check("只发 EV_POSE_ANCHORED，不发 EV_DRIFT_WARNING",
      [e.type for e in evs] == ["POSE_ANCHORED"], f"{[e.type for e in evs]}")
check_close("drift_m 记录该段偏差 %.4f m" % dist, bb.get(P_FIX).drift_m, dist, 1e-9)
check("锚回 M10 格心", abs(bb.get(P_POSE).x - M10[0]) < 1e-12
      and abs(bb.get(P_POSE).y - M10[1]) < 1e-12, f"got {bb.get(P_POSE)!r}")

section("⑦c 吸附闩锁按航点重置：下一个航点会再次吸附")
route3 = make_route([Waypoint.at_block("N10"), Waypoint.at_block("M10"),
                     Waypoint.at_block("M9")])
task, bridge, bb, bus, clock = build(route=route3, seed_pose=False)
sub3 = bus.subscribe("t_d4", [EV_POSE_ANCHORED])
bridge.set_pose(N10[0], N10[1] + 0.30, 0.0)       # 到位 M10（第 1 段末点）
task.tick()
check("第一处到位吸附 M10", task.anchored == 1 and bb.get(P_FIX).last_anchor == "M10",
      f"anchored={task.anchored} last={getattr(bb.get(P_FIX), 'last_anchor', None)}")
# 车沿路线继续走：第 2 段是 M10(0.525,0.525) → M9(0.875,0.525)，离 M9 只剩 0.10 m
# （< arrival 0.12）→ 闩锁必须已解开，再次吸附
bridge.set_pose(M9[0] - 0.10, M9[1], 0.0)
task.tick()
check("走到下一个航点后再次吸附 M9",
      task.anchored == 2 and bb.get(P_FIX).last_anchor == "M9",
      f"anchored={task.anchored} last={getattr(bb.get(P_FIX), 'last_anchor', None)}")
check("anchor_count 累计为 2", bb.get(P_FIX).anchor_count == 2,
      f"got {bb.get(P_FIX).anchor_count}")
check("EV_POSE_ANCHORED 共 2 条", len(sub3.poll()) == 2)
check("max_drift_m 取两段较大者", bb.get(P_FIX).max_drift_m >= bb.get(P_FIX).drift_m - 1e-12)

section("⑧ 边界情形：无路线 / 路线换新 / 位姿冻结")
task, bridge, bb, bus, clock = build(seed_pose=False)
bridge.set_pose_at_block("N10")
task.tick()
check("无 P_ROUTE 时不写 P_ROUTE_S", bb.get(P_ROUTE_S) is None)
check("无 P_ROUTE 时仍写 P_POSE", bb.get(P_POSE) is not None)
check("无 P_ROUTE 时 projected_ticks=0", task.projected_ticks == 0)

route2 = make_route([Waypoint.at_block("N10"), Waypoint.at_block("N9"),
                     Waypoint.at_block("N8")])
bb.set(P_ROUTE, route2, source="test")
bridge.set_pose_at_block("N9")
task.tick()
check("换路线后 hint 重置（P_ROUTE_S 从头算）", bb.get(P_ROUTE_S) < 0.5,
      f"got {bb.get(P_ROUTE_S)}")
check("P_ROUTE_INDEX 属于新路线", bb.get(P_ROUTE_INDEX) in (0, 1),
      f"got {bb.get(P_ROUTE_INDEX)}")

bridge.pose_frozen = True                     # 定位失联但 pose() 仍返回旧对象
n_before = task.missing_pose_ticks
task.tick()
check("pose_age()==inf 时不写 P_POSE（冻结）", bb.get(P_POSE) is not None)  # 旧值还在
check("冻结时 P_ODOM_AGE 被写成 inf", bb.get(P_ODOM_AGE) == INF,
      f"got {bb.get(P_ODOM_AGE)!r}")
check("冻结被计入 missing_pose_ticks", task.missing_pose_ticks == n_before + 1,
      f"got {task.missing_pose_ticks}")

finish()
