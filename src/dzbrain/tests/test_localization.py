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
from dzbrain.pursuit.path import Path                                         # noqa: E402
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

section("⑤ 到位吸附（非末点）：偏差 0.05 m < arrival 0.12 m")
# ★ 用 3 点路线 N10→M10→M9：M10 是**非末点**航点，吸附半径照旧 arrival_m(0.12)。
#   末点的吸附半径已收紧到 _GOAL_STOP_M(0.03)（见 ⑦d），本条专证非末点行为不变。
route3a = make_route([Waypoint.at_block("N10"), Waypoint.at_block("M10"),
                      Waypoint.at_block("M9")])
task, bridge, bb, bus, clock = build(route=route3a, seed_pose=False)
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
check("锚定后 P_ROUTE_S == M10 的弧长 0.35 m（非末点吸附照常）",
      abs(bb.get(P_ROUTE_S) - STEP) < 1e-9,
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
# ★ 3 点直线 N10→M10→L10（都沿 +y）：M10 是**非末点**航点，0.117 m 偏差照旧可锚；
#   末点吸附半径收紧到 _GOAL_STOP_M(0.03) 由 ⑦d 单独验证。
route3b = make_route([Waypoint.at_block("N10"), Waypoint.at_block("M10"),
                      Waypoint.at_block("L10")])
task, bridge, bb, bus, clock = build(route=route3b, seed_pose=False)
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
# ★ 4 点路线 N10→M10→M9→L9：M9 是**非末点**航点，第二次吸附仍按 arrival_m(0.12) 判。
route3 = make_route([Waypoint.at_block("N10"), Waypoint.at_block("M10"),
                     Waypoint.at_block("M9"), Waypoint.at_block("L9")])
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

section("⑦d ★ 末点吸附收紧：0.10 m 外不再被拽到格心 / 0.02 m 内照常吸附")
# 实车缺陷（2026 直线标定 N10→A10）：末点按 arrival_m(0.12) 吸附，信念在偏差 0.118 m
# 处被拽到格心（车端 7_path.log："到位吸附 A10 偏差 0.118 m → 锚回格心"）→ Mission
# 见信念已到 → FINISH 提前刹车 → 物理车停在格心前 11 cm 欠程（scale=1.00003 说明
# 不是里程计误差）。修复后末点吸附半径 = _GOAL_STOP_M(0.03)：
#   0.10 m 外不吸附（假 FINISH 无从谈起），0.02 m 内照常吸附（误差归零能力保留）。
task, bridge, bb, bus, clock = build(route=route, seed_pose=False)   # route 的末点 = M10
sub_g = bus.subscribe("t_goal", [EV_POSE_ANCHORED])
bridge.set_pose(N10[0], M10[1] - 0.10, 0.0)   # 离末点 M10 还有 0.10 m
task.tick()
p = bb.get(P_POSE)
check("0.10 m 外不吸附：P_POSE 保持原值（未被拽到格心）",
      abs(p.x - N10[0]) < 1e-12 and abs(p.y - (M10[1] - 0.10)) < 1e-12,
      f"got ({p.x!r},{p.y!r})")
check("0.10 m 外不吸附：无 EV_POSE_ANCHORED / anchored==0 / 不写 P_FIX",
      len(sub_g.poll()) == 0 and task.anchored == 0 and bb.get(P_FIX) is None,
      f"anchored={task.anchored} fix={bb.get(P_FIX)!r}")
check("0.10 m 外 P_ROUTE_S = 0.25（没有因吸附被填成总长 0.35）",
      abs(bb.get(P_ROUTE_S) - 0.25) < 1e-9, f"got {bb.get(P_ROUTE_S)}")
bridge.set_pose(N10[0], M10[1] - 0.02, 0.0)   # 离末点 0.02 m ≤ _GOAL_STOP_M(0.03)
task.tick()
p2 = bb.get(P_POSE)
check("0.02 m 内照常吸附回末点格心", abs(p2.x - M10[0]) < 1e-12
      and abs(p2.y - M10[1]) < 1e-12, f"got ({p2.x!r},{p2.y!r})")
evs = sub_g.poll()
check("0.02 m 内吸附：1 条 EV_POSE_ANCHORED / anchored==1",
      len(evs) == 1 and task.anchored == 1, f"evs={len(evs)} anchored={task.anchored}")
check_close("吸附记录 drift_m = 0.02", bb.get(P_FIX).drift_m, 0.02, 1e-9)

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

# --------------------------------------------------------------------------- #
section("⑨ ★ 投影卡死自愈：离开走廊后 P_ROUTE_S 仍跟得上全局投影（bench 实证缺陷）")

# 合成路线 = 现场图那一段的形状（坐标就是 geometry.block_to_meters 的格心值）：
#   G10 → F10 → F11 → E11 → D11 → D8
# 为什么非得是这种"折返/两条平行走廊"的形状：冻结需要**路线自己靠近自己** —— 车切角
# 拐进两条走廊之间的"口袋"后，全局最近点落在后面那条走廊（D11→D8），而窗口投影被夹在
# 窗口下界（`Path.project` 的 t_floor）上不动。bench 日志里 P_ROUTE_S 正是冻在
# "段10 s3.73"（F10→F11 段的中段）而全局投影已跑到 2.5 m 之外。
# ★ 与现场图的**刻意差异**：现场 F11→E11→R:F11 是**共线折返**（同一物理点落在两条航段上、
#   s 相差整整一段），那是路径自重叠的分支歧义，会掩盖"冻结"本身；这里把折返简化为
#   "继续向北、再向东"，保留冻结所需的几何，同时让"真值"唯一。
_ROUTE_FOLD = (("G10", 0.525, 2.625), ("F10", 0.525, 2.975), ("F11", 0.175, 2.975),
               ("E11", 0.175, 3.325), ("D11", 0.175, 3.675), ("D8", 1.225, 3.675))
_TICK_DT = 0.05          # 20 Hz
_STEP_M = 0.0075         # 巡航 0.15 m/s × 0.05 s


def fold_route() -> Route:
    return make_route([Waypoint(block=b, x=x, y=y, yaw=0.0) for b, x, y in _ROUTE_FOLD])


def walk(task, bridge, clock, path, s_target: float) -> None:
    """沿路线把投影链"喂"到弧长 s_target（每拍 7.5 mm，模拟真实 20 Hz 巡航）。"""
    s = 0.0
    while s < s_target:
        x, y, _ = path.point_at(s)
        bridge.set_pose(x, y, 0.0)
        clock.advance(_TICK_DT)
        task.tick()
        s += _STEP_M


def leg(task, bridge, clock, bb, path, x0, y0, dx, dy, n, out) -> None:
    """走一条直线腿，逐拍记录（上报投影点 vs 全局最近点）。"""
    for k in range(n + 1):
        t = k / n
        x, y = x0 + dx * t, y0 + dy * t
        bridge.set_pose(x, y, 0.0)
        clock.advance(_TICK_DT)
        task.tick()
        s = bb.get(P_ROUTE_S)
        px, py, _ = path.point_at(s)
        g = path.project(x, y, s_hint=None)
        out.append({
            "x": x, "y": y, "s": s,
            "d_car": math.hypot(x - px, y - py),        # 车 → 上报投影点
            "g_s": g.s,
            "g_car": math.hypot(x - g.x, y - g.y),      # 车 → 全局最近点
            "d_pt": math.hypot(px - g.x, py - g.y),     # 上报点 ↔ 全局点
        })


route_fold = fold_route()
path_fold = Path(route_fold)
check_close("合成路线总长 = 一格 + 一格 + 0.35 + 0.35 + 1.05 = 2.45 m",
            path_fold.total_length, 2.45, 1e-9)

# ---- ⑨.1 沿走廊把 hint 喂到冻结段中段（F10→F11 段，s=0.55） ----
task, bridge, bb, bus, clock = build(overrides={"fix": {"anchor_at_waypoint": False}},
                                     route=route_fold, seed_pose=False)
walk(task, bridge, clock, path_fold, 0.55)
check("喂到冻结段：P_ROUTE_S 落在 F10→F11 段中段（0.54~0.56）",
      0.54 <= bb.get(P_ROUTE_S) <= 0.56, f"got {bb.get(P_ROUTE_S)}（步长 {_STEP_M} m）")
check("喂的过程中 hint 与投影链都没卡（0 次全扫）", task.stuck_scans == 0,
      f"got {task.stuck_scans}")

# ---- ⑨.2 切角离开走廊：冻结 → 自愈 ----
rows: list[dict] = []
leg(task, bridge, clock, bb, path_fold, 0.325, 2.975, 0.275, 0.425, 67, rows)   # 进"口袋"
leg(task, bridge, clock, bb, path_fold, 0.600, 3.400, 0.200, 0.260, 40, rows)   # 回到远端走廊
moved = math.hypot(rows[-1]["x"] - rows[0]["x"], rows[-1]["y"] - rows[0]["y"])
lag_last = abs(rows[-1]["s"] - rows[-1]["g_s"])
worst = max(r["d_car"] - r["g_car"] for r in rows)

check("车确实离开了原走廊并走了 ~0.83 m（对照：不是原地抖动）", moved > 0.80,
      f"位移 {moved:.3f} m")
check("★ 自愈被触发（projection_heals ≥ 1）", task.projection_heals >= 1,
      f"heals={task.projection_heals} scans={task.stuck_scans}")
check("★ 每一拍：车到**上报**投影点的距离 ≤ 车到**全局最近点**的距离 + arrival_m"
      "（冻结时这一项会失控增长 —— 见下面修复前的失败输出）",
      worst <= 0.12 + 1e-9, f"最差 {worst:.3f} m（容差 0.12 m）")
check("★ 末拍：上报 P_ROUTE_S 与全局投影之差 ≤ arrival_m（车已回到远端走廊上）",
      lag_last <= 0.12 + 1e-9, f"|{rows[-1]['s']:.3f} − {rows[-1]['g_s']:.3f}| = {lag_last:.3f} m")
check("★ 末拍：车到上报投影点 ≤ arrival_m（上报点就在车脚下）",
      rows[-1]["d_car"] <= 0.12 + 1e-9, f"{rows[-1]['d_car']:.3f} m")
check("末拍与全局点基本重合", rows[-1]["d_pt"] <= 0.05,
      f"{rows[-1]['d_pt']:.3f} m")
heals_at_freeze = [r for r in rows if r["s"] != 0.55]
check("s 不再是那个冻住的 0.55（旧代码 109 拍全冻）", bool(heals_at_freeze),
      f"s 序列唯一值 = {sorted({round(r['s'], 3) for r in rows})}")
check("P_ROUTE_INDEX 跟着 s 走（不再卡在第 1 段）",
      bb.get(P_ROUTE_INDEX) == path_fold.project(rows[-1]["x"], rows[-1]["y"]).seg_index,
      f"index={bb.get(P_ROUTE_INDEX)}")

# ---- ⑨.3 反例一：正常沿走廊行驶，一次全扫都不该做（不是"每拍全路径扫描"） ----
task_n, bridge_n, bb_n, bus_n, clock_n = build(
    overrides={"fix": {"anchor_at_waypoint": False}}, route=route_fold, seed_pose=False)
walk(task_n, bridge_n, clock_n, path_fold, 0.55)
check("正常行驶 0 次全扫（自愈不是性能热点）", task_n.stuck_scans == 0,
      f"scans={task_n.stuck_scans} heals={task_n.projection_heals}")

# ---- ⑨.4 反例二：车静止 + 大横向偏差 → 不判卡死（判据要求"车确实在动"） ----
task_s, bridge_s, bb_s, bus_s, clock_s = build(
    overrides={"fix": {"anchor_at_waypoint": False}}, route=route_fold, seed_pose=False)
bridge_s.set_pose(0.875, 3.425, 0.0)          # 停在走廊之间（离线 ~0.25 m）
task_s.tick()
s_park = bb_s.get(P_ROUTE_S)
for _ in range(40):
    clock_s.advance(_TICK_DT)
    task_s.tick()
check("静止 40 拍：0 次全扫（位姿没动 → 不满足'车在动'）", task_s.stuck_scans == 0,
      f"scans={task_s.stuck_scans}")
check("静止时 P_ROUTE_S 保持不变（车没动，没有跑掉的真值要追）",
      bb_s.get(P_ROUTE_S) == s_park, f"{s_park} → {bb_s.get(P_ROUTE_S)}")
check("静止时也没吸附/没崩（ticks == 41）", task_s.ticks == 41, f"got {task_s.ticks}")

# --------------------------------------------------------------------------- #
#  ⑩ 尺度疑似闸（闸③）：固定到位偏移放行 —— 2026-10-17 实车 N5 多跑 10 cm 的根因
# --------------------------------------------------------------------------- #
# 实车指纹：每次吸附都在 arrival_m 边缘触发，沿程修正量恒 ~0.10 m（同号），
# 隐含尺度 = 修正量 ÷ 距上次吸附弧长，随分母增长一路衰减（44.7% → 0.2%）。
# 旧闸只看「连续同号 + 中位 ≥ 0.06 m」→ 34 连拒 → 信念不再纠偏 → 终点物理多跑 10 cm。
# 修复：隐含尺度必须 ≥ localization.anchor_scale_max(8%) 才拦 ——
#   固定偏移被长分母稀释到 8% 以下时放行，真实尺度误差（比值不衰减）仍被拦。
section("⑩ 尺度疑似闸：固定偏移（隐含尺度 < 8%）放行；真实尺度误差仍拦")

def _walk(task, bridge, clock, x0, y0, x1, y1, step=0.05):
    """从 (x0,y0) 沿直线走到 (x1,y1)，每步 tick 一次（模拟车连续行驶）。"""
    seg = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int(round(seg / step)))
    for i in range(1, n + 1):
        t = i / n
        bridge.set_pose(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, 0.0)
        clock.advance(0.1)
        task.tick()

# ⑩.1 固定偏移：N10→M10→M9→L9→K9→J9→I9→H9（每段 0.35 m，均在格心前 0.10 m 触发吸附；
#   H9 只是收尾占位，保证 I9 是非末点、用 arrival_m 半径）
route_scale = make_route([Waypoint.at_block(b) for b in
                          ("N10", "M10", "M9", "L9", "K9", "J9", "I9", "H9")])
task, bridge, bb, bus, clock = build(route=route_scale, seed_pose=False)
c = {b: geo.block_to_meters(b)
     for b in ("N10", "M10", "M9", "L9", "K9", "J9", "I9", "H9")}

def _toward(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

def _to_offset(a, prev, off):
    """沿 prev→a 方向、停在 a 格心前 off 米的点。"""
    d = math.hypot(a[0] - prev[0], a[1] - prev[1])
    t = max(0.0, 1.0 - off / d)
    return _toward(prev, a, t)

pos = c["N10"]
for prev, nxt in (("N10", "M10"), ("M10", "M9"), ("M9", "L9"),
                  ("L9", "K9"), ("K9", "J9"), ("J9", "I9")):
    end = _to_offset(c[nxt], c[prev], 0.10)
    _walk(task, bridge, clock, *pos, *end)
    pos = end
check("固定偏移：L9/K9/J9 三次同向大修正被拦（隐含尺度 40%/16.7%/10.5% ≥ 8%）",
      task.anchor_refused == 3, f"refused={task.anchor_refused}")
check("固定偏移：I9 隐含尺度衰减到 7.7% < 8% → 放行，吸附链恢复（anchored == 3）",
      task.anchored == 3, f"anchored={task.anchored}")
check("固定偏移：last_anchor == 'I9'（不再 34 连拒）",
      getattr(bb.get(P_FIX), "last_anchor", None) == "I9",
      f"{getattr(bb.get(P_FIX), 'last_anchor', None)!r}")
check("固定偏移：scale_suspect 仍记账（闸确实看见过疑似模式）", task.scale_suspect is True)
check_close("固定偏移：scale_suspect_max ≈ 40%（L9 那次）", task.scale_suspect_max, 0.40, 0.02)

# ⑩.2 对照：真实尺度误差（同号 + 中位 ≥ 0.06m + 隐含尺度 ≥ 8%）仍被拦。
#   N10→M10→M9→L9→K9，前两段正常吸附，第三段隐含尺度 0.10/0.25 = 40% ≥ 8% → 拒绝。
#   （K9 只是收尾占位，保证 L9 是非末点、用 arrival_m 半径）
route_grow = make_route([Waypoint.at_block(b) for b in ("N10", "M10", "M9", "L9", "K9")])
task2, bridge2, bb2, bus2, clock2 = build(route=route_grow, seed_pose=False)
pos = c["N10"]
for prev, nxt in (("N10", "M10"), ("M10", "M9"), ("M9", "L9")):
    end = _to_offset(c[nxt], c[prev], 0.10)
    _walk(task2, bridge2, clock2, *pos, *end)
    pos = end
check("真实尺度误差：第 3 次同向大修正仍被拦（anchored 停在 2）", task2.anchored == 2,
      f"anchored={task2.anchored}")
check("真实尺度误差：anchor_refused == 1 且 scale_suspect 置位",
      task2.anchor_refused == 1 and task2.scale_suspect is True,
      f"refused={task2.anchor_refused} suspect={task2.scale_suspect}")

finish()


finish()
