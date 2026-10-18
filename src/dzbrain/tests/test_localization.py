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
  9. 投影卡死自愈（离开走廊后 P_ROUTE_S 跟得上全局投影）
 10. 尺度疑似闸（吸附掩盖尺度误差的可见化）
 11. ★ 2026-10-17 14:59 实车 +12.82 m 投影误锁的回归：大跳跃必须"新投影贴在车上"
 12. ★★ 航向完整性守卫（yaw 率超运动学上限 ⇒ 判故障）：14:59:29~40 真实片段回放必须报
     （比投影跳变早 5.2 s），正常 90° 转弯/静止小漂移不得误报，遥控接管中的转动只记账
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
from dzbrain.core.blackboard import (P_FIX, P_MOTION_LIMIT, P_ODOM_AGE, P_POSE,  # noqa: E402
                                     P_POSE_RAW, P_ROUTE, P_ROUTE_INDEX, P_ROUTE_S,
                                     P_SAFETY, Blackboard)
from dzbrain.core.config import Config, SCHEMA                                # noqa: E402
from dzbrain.core.events import (EV_DRIFT_WARNING, EV_POSE_ANCHORED,          # noqa: E402
                                 EV_TASK_FAULT, EventBus)
from dzbrain.core.state import FixState                                       # noqa: E402
from dzbrain.pursuit.path import Path                                         # noqa: E402
from dzbrain.route.model import Route, Waypoint, make_route                   # noqa: E402
from dzbrain.route.parser import parse_mission                                # noqa: E402
from dzbrain.tasks.localization import LocalizationTask                       # noqa: E402
from dzbrain.tasks.safety import SafetyTask                                   # noqa: E402
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


# --------------------------------------------------------------------------- #
#  ⑪ ★ 2026-10-17 14:59 实车 +12.82 m 投影误锁：大跳跃必须"新投影贴在车上"才可信
# --------------------------------------------------------------------------- #
# 真实事故（_tmp_session/run_20261017_1504/7_path_utf8.txt，UTF-8 行号）：
#   行 207 TrackerTask：「投影跟丢自愈：窗口内最近点离线 0.70 m，全路线重投影 → 段40 s=14.11」
#   行 210 Localization：「投影卡死自愈：s 冻在 1.290（第 3 段，离线 0.78 m）→ 全路线重投影得
#          14.110（第 40 段，离线 0.62 m，+12.820 m）」
# 为什么是"误锁"：车在 K9 桥洞蹭墙偏航后驶向 +x，投影链一直认为 s≈1.29（第 3~4 段）；
# 当车走到 u≈4.65（两条路线腿之间）时，终点段 K5→L5 离线 0.62 m 只比当前段 0.78 m
# "近 0.16 m" ⇒ 全路线重投影就把 s 瞬移到 12.7 m 之外。控制器的前视点随后指到终点段上，
# 转角减速层同时失效（0.12→0.35 m/s），车直线冲出边界。
#
# 本用例喂**事故时刻的真实里程计位姿**（3_localization.log 的 pos，格单位×0.35 m），
# 断言修复后单拍 s 不再被瞬移；并当场把可信闸关掉，证明"关掉就红"。
INCIDENT_TOKENS = ("N10 M10 M9 L9 K9 J9 J10 I10 H10 G10 F10 E10 E9 D9 C9 B9 B8 B7 B6 "
                   "A6 A5 A4 A3 A2 B2 B1 C1 D1 D2 D3* E3 F3 G3 G4 G5 G6 H6 I6 I5 J5 "
                   "K5 L5 M5 N5,y").split()
#: 2026-10-17 15:00:32~40 的真实位姿（t 从 serial_reader 首帧算起，5 Hz 采样）
INCIDENT_POSES = [
    (0.911, 1.105), (0.945, 1.113), (0.980, 1.113), (1.015, 1.240),
    (1.043, 1.296), (1.065, 1.295), (1.099, 1.192), (1.133, 1.153),
    (1.181, 1.126), (1.242, 1.109), (1.288, 1.105), (1.330, 1.106),
    (1.372, 1.109), (1.414, 1.109), (1.459, 1.109), (1.505, 1.113),
    (1.547, 1.113), (1.592, 1.113), (1.620, 1.113), (1.662, 1.116),
]


def _replay_incident(disable_gate: bool) -> tuple[list[float], object]:
    """把事故位姿按顺序喂进 LocalizationTask，返回 (每拍 P_ROUTE_S, task)。"""
    route = parse_mission(list(INCIDENT_TOKENS))
    task, bridge, bb, bus, clock = build(
        overrides={"fix": {"anchor_at_waypoint": False}}, route=route, seed_pose=False)
    if disable_gate:                      # 关掉可信闸 = 修复前行为（带牙对照）
        task.stuck_trust_dist_m = float("inf")
    out: list[float] = []
    for x, y in INCIDENT_POSES:
        bridge.set_pose(x, y, 0.0)
        clock.advance(0.2)                # 真实里程计 5 Hz
        task.tick()
        out.append(float(bb.get(P_ROUTE_S)))
    return out, task


s_ok, task_ok = _replay_incident(disable_gate=False)
step_max = max(abs(b - a) for a, b in zip(s_ok, s_ok[1:]))
check("修复后：单拍 P_ROUTE_S 最大变化 < 0.5 m（旧行为 12.82 m）", step_max < 0.5,
      f"max step = {step_max:.3f} m；s 序列 = {[round(v, 3) for v in s_ok]}")
check("修复后：末拍 s 仍在第 3/4 段附近（< 2.0 m），没被瞬移到 14.11",
      s_ok[-1] < 2.0, f"got {s_ok[-1]:.3f}")
check("修复后：拒绝计数 ≥ 1（闸确实被触发，不是「恰好没判卡死」）",
      task_ok.stuck_heal_refused >= 1, f"refused={task_ok.stuck_heal_refused}")

s_old, task_old = _replay_incident(disable_gate=True)
old_max = max(s_old)
check("★ 带牙对照：关掉可信闸（= 修复前）后，同一段位姿确实被瞬移到 > 10 m",
      old_max > 10.0, f"max s = {old_max:.3f}；序列 = {[round(v, 3) for v in s_old]}")
check("★ 带牙对照：修复前那一跳 ≈ +12.8 m（与 7_path 行 210 的 +12.820 m 同量级）",
      11.0 < old_max < 15.0, f"max s = {old_max:.3f}")

# --------------------------------------------------------------------------- #
#  ⑫ ★★ 航向完整性守卫（2026-10-17 14:59 K9 事故：yaw 120.3°/s @ 0.112 m/s = 5.08× 上限）
# --------------------------------------------------------------------------- #
section("⑫ 航向完整性守卫：真实片段回放必须报故障；正常 90° 转弯不得误报")

# ---- ⑫.0 SCHEMA 登记（阈值不写死在代码里；现场可经 robot.yaml 覆盖）----
_yaw_keys = (
    "localization.yaw_guard", "localization.yaw_kin_margin",
    "localization.yaw_rate_floor_dps", "localization.yaw_abs_floor_dps",
    "localization.yaw_static_dps", "localization.yaw_min_speed_mps",
    "localization.yaw_bad_ticks", "localization.yaw_plausible_speed_mps",
    "localization.yaw_max_dt_s", "localization.yaw_imu_mismatch_ratio",
)
_cfg0 = Config.from_dict({})
_missing = [k for k in _yaw_keys if k not in SCHEMA]
check("10 个 localization.yaw_* 阈值全部登记进 SCHEMA", not _missing, f"missing={_missing}")
check("SCHEMA 默认值可读到：yaw_guard=True", _cfg0.get("localization.yaw_guard") is True)
check("SCHEMA 默认值可读到：yaw_static_dps=45°/s、yaw_bad_ticks=3",
      _cfg0.get("localization.yaw_static_dps") == 45.0
      and _cfg0.get("localization.yaw_bad_ticks") == 3,
      f"{_cfg0.get('localization.yaw_static_dps')!r}/{_cfg0.get('localization.yaw_bad_ticks')!r}")

# ---- ⑫.1 真实片段：3_localization.log:152-185（5 Hz，14:59:28.6~40.6）----
# 字段 = (米制 x, 米制 y, 地图 yaw/度)；pos 原为格单位（×0.35 转米），yaw 是 serial_reader
# 打印的 [imu] 地图 yaw（已含 +90° 偏置，与 bridge.pose() 同口径）。
# 前 3.6 s 是正常沿 K9 直道（角速度 0~30°/s）；14:59:31.8 起 yaw 90°→−78°。
YAW_EVENT_FRAMES = [
    (0.710, 0.595, 38), (0.728, 0.609, 38), (0.745, 0.623, 39), (0.763, 0.637, 41),
    (0.777, 0.651, 43), (0.801, 0.675, 48), (0.837, 0.717, 57), (0.865, 0.773, 67),
    (0.878, 0.816, 72), (0.892, 0.858, 76), (0.899, 0.899, 82), (0.903, 0.938, 85),
    (0.906, 0.973, 86), (0.906, 1.008, 88), (0.910, 1.046, 89), (0.910, 1.085, 90),
    (0.913, 1.162, 83), (0.927, 1.232, 71), (0.955, 1.281, 54), (0.980, 1.302, 32),
    (1.001, 1.313, 16), (1.022, 1.316, -4), (1.043, 1.309, -24), (1.064, 1.298, -44),
    (1.074, 1.277, -67), (1.081, 1.256, -78), (1.085, 1.235, -72), (1.092, 1.214, -74),
    (1.099, 1.194, -76), (1.106, 1.172, -58), (1.123, 1.158, -34), (1.141, 1.148, -30),
    (1.158, 1.137, -30), (1.179, 1.127, -26),
]

#: 停车后静止期 yaw 漂移片段：3_localization.log:585-600（15:00:55.2~58.4，5 Hz）。
#: 真实位置全程冻结在 (12.28,4.95) 格 = (4.298,1.7325) m，yaw 16°→131°→98°（峰值 170°/s）。
YAW_STATIC_FRAMES = [
    (4.298, 1.7325, 16), (4.298, 1.7325, 35), (4.298, 1.7325, 52), (4.298, 1.7325, 62),
    (4.298, 1.7325, 65), (4.298, 1.7325, 55), (4.298, 1.7325, 48), (4.298, 1.7325, 47),
    (4.298, 1.7325, 60), (4.298, 1.7325, 94), (4.298, 1.7325, 118), (4.298, 1.7325, 131),
    (4.298, 1.7325, 129), (4.298, 1.7325, 123), (4.298, 1.7325, 116), (4.298, 1.7325, 109),
]


def _replay_frames(frames, *, guard: bool = True, manual: bool = False,
                   with_route: bool = True, imu_dps=None):
    """把 5 Hz 位姿帧喂进 LocalizationTask（每帧 dt=0.2 s），返回 (task, bridge, bb, events, trip_t)。"""
    route = parse_mission(list(INCIDENT_TOKENS)) if with_route else None
    task, bridge, bb, bus, clock = build(
        overrides={"fix": {"anchor_at_waypoint": False}}, route=route, seed_pose=False)
    task.yaw_guard_enabled = bool(guard)
    if manual:
        bridge.set_manual_override(True)
    if imu_dps is not None:                  # 模拟"桥有 IMU"的能力（诊断分支用）
        bridge.imu = lambda: (0.0, 0.0, math.radians(imu_dps), 0.0, 0.0, 0.0)
        bridge.imu_age = lambda: 0.0
        task._imu_fn = bridge.imu
        task._imu_age_fn = bridge.imu_age
    sub = bus.subscribe("yaw_guard", [EV_TASK_FAULT], maxsize=64)
    events, trip_t = [], None
    for i, (x, y, yaw_deg) in enumerate(frames):
        bridge.set_pose(x, y, math.radians(yaw_deg))
        clock.advance(0.2)                    # 真实里程计/serial 打印 5 Hz
        task.tick()
        events.extend(sub.poll(64))
        if task.yaw_untrusted and trip_t is None:
            trip_t = 0.2 * i
    return task, bridge, bb, events, trip_t


# ---- ⑫.2 正例：真实事故片段必须判故障，且早于投影跳变（38.2 s）与越界（49.2 s）----
task_y, bridge_y, bb_y, ev_y, trip_y = _replay_frames(YAW_EVENT_FRAMES)
check("真实片段：判「航向失去可信」（yaw_untrusted=True）", task_y.yaw_untrusted is True)
check("真实片段：故障在窗口内触发（trip ≤ 5.0 s = 14:59:33.6；投影跳变在 38.2 s）",
      trip_y is not None and trip_y <= 5.0, f"trip_t={trip_y}")
check("真实片段：也不是前 3.4 s 正常段的误报（trip ≥ 3.4 s）",
      trip_y is not None and trip_y >= 3.4, f"trip_t={trip_y}")
check("真实片段：连续 3 个新帧超限才判（yaw_bad_samples ≥ 3）",
      task_y.yaw_bad_samples >= 3, f"bad={task_y.yaw_bad_samples}")
check("真实片段：33 个新帧样本（5 Hz 帧频；同一帧重复读不算样本）",
      task_y.yaw_guard_samples == len(YAW_EVENT_FRAMES) - 1,
      f"samples={task_y.yaw_guard_samples}")
check("真实片段：原因 = 运动学上限（yaw_rate_over_kinematic）",
      task_y._yaw_reason == "yaw_rate_over_kinematic", f"reason={task_y._yaw_reason!r}")
check("真实片段：见过的峰值角速度 ≈ 120°/s（与裁决书 120.3°/s 同量级）",
      100.0 < task_y.yaw_max_rate_dps < 140.0,
      f"max_rate={task_y.yaw_max_rate_dps:.1f}°/s")

lim_y = bb_y.get(P_MOTION_LIMIT)
check("真实片段：P_MOTION_LIMIT 请求 0 速（既有停车通路）",
      isinstance(lim_y, dict) and lim_y.get("speed_mps") == 0.0
      and lim_y.get("yaw_untrusted") is True,
      f"limit={lim_y}")
check("真实片段：EV_TASK_FAULT 只发 1 条（故障通路，不是每拍刷）",
      sum(1 for e in ev_y if e.type == EV_TASK_FAULT) == 1,
      f"events={[e.type for e in ev_y]}")
check("真实片段：故障载荷带 where=heading_untrusted",
      any(e.data.get("where") == "heading_untrusted" for e in ev_y))

# ★ 带牙对照：关掉守卫（= 修复前行为）后，同一段位姿一条故障都报不出来。
task_nog, _, bb_nog, ev_nog, trip_nog = _replay_frames(YAW_EVENT_FRAMES, guard=False)
check("★ 带牙对照：关掉守卫后同一片段不判故障、不发事件、不限速",
      task_nog.yaw_untrusted is False and not ev_nog and bb_nog.get(P_MOTION_LIMIT) is None,
      f"untrusted={task_nog.yaw_untrusted} events={len(ev_nog)}")

# ---- ⑫.3 反例一：事故前 3.4 s 正常段（14:59:28.6~32.0，18 帧）零误报 ----
#      注：第 19 帧（14:59:32.6）已经进入异常（85°/s > 82°/s 上限），所以只取前 18 帧。
task_ok, _, _, ev_ok, _ = _replay_frames(YAW_EVENT_FRAMES[:18])
check("反例（事故前正常段 18 帧）：0 个超限样本、不判故障",
      task_ok.yaw_untrusted is False and task_ok.yaw_bad_samples == 0
      and task_ok.yaw_guard_samples == 17,
      f"bad={task_ok.yaw_bad_samples} samples={task_ok.yaw_guard_samples} "
      f"max_rate={task_ok.yaw_max_rate_dps:.1f}")


# ---- ⑫.4 反例二：正常 90° 转弯（Ackermann 极限：0.35 m/s @ 舵角 30° = 73.7°/s）----
def _arc_frames(v: float = 0.35, steer_deg: float = 30.0, n: int = 80,
                dt: float = 0.05) -> list:
    """自行车模型积分出一条圆弧的位姿帧（20 Hz，tick 与帧同频）。"""
    gain = math.tan(math.radians(steer_deg)) / 0.157
    x = y = yaw = 0.0
    out = []
    for _ in range(n):
        yaw += v * gain * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        out.append((x, y, math.degrees(yaw)))
    return out


task_turn, _, bb_turn, ev_turn, _ = _replay_frames(_arc_frames(), with_route=False)
check("反例（正常 90° 转弯、0.35 m/s、满舵 30° = 73.7°/s）：不误报",
      task_turn.yaw_untrusted is False and task_turn.yaw_bad_samples == 0,
      f"bad={task_turn.yaw_bad_samples} samples={task_turn.yaw_guard_samples}")
check("反例（正常 90° 转弯）：P_MOTION_LIMIT 未被触发",
      bb_turn.get(P_MOTION_LIMIT) is None, f"limit={bb_turn.get(P_MOTION_LIMIT)}")

# ---- ⑫.5 反例三：静止期的小幅 yaw 漂移（7.5°/s < 静止上限 45°/s）不误报 ----
_drift_small = [(1.0, 1.0, 30.0 + 1.5 * i) for i in range(20)]
task_dr, _, _, ev_dr, _ = _replay_frames(_drift_small, with_route=False)
check("反例（静止 + 7.5°/s 慢漂）：不误报",
      task_dr.yaw_untrusted is False and task_dr.yaw_bad_samples == 0,
      f"bad={task_dr.yaw_bad_samples}")

# ---- ⑫.6 正例二：静止期被「转」（事故 15:00:57 的 170°/s 片段）→ 判故障 ----
task_st, _, bb_st, ev_st, trip_st = _replay_frames(YAW_STATIC_FRAMES, with_route=False)
check("静止期 yaw 170°/s：判「静止期航向变化」故障",
      task_st.yaw_untrusted is True and task_st._yaw_reason == "yaw_static_rotation",
      f"reason={task_st._yaw_reason!r} trip_t={trip_st}")
check("静止期故障：在片段前 1 s 内触发（前三帧 95/85/50°/s）",
      trip_st is not None and trip_st <= 1.0, f"trip_t={trip_st}")

# ---- ⑫.7 区分「手动旋转」：遥控接管中同样的转动只记账、不判故障 ----
#   ★ 真实片段的位置 (4.298,1.73) 已在安全框外（越界余量 ±0.6），边界护栏会独立发
#     boundary_stop；为把「航向判据」单独隔离出来，这里只把**位置**挪到框内，
#     **yaw 序列仍是 15:00:55~58 的真实值**。
YAW_STATIC_FRAMES_IN_BOUNDS = [(1.0, 1.0, yaw) for _, _, yaw in YAW_STATIC_FRAMES]
task_man, _, bb_man, ev_man, _ = _replay_frames(YAW_STATIC_FRAMES_IN_BOUNDS, manual=True)
check("遥控接管中静止转动：不判故障（人比机器优先）",
      task_man.yaw_untrusted is False and task_man.yaw_manual_ticks >= 3,
      f"manual_ticks={task_man.yaw_manual_ticks}")
check("遥控接管中静止转动：不发 EV_TASK_FAULT、不限速",
      not ev_man and bb_man.get(P_MOTION_LIMIT) is None,
      f"events={len(ev_man)} limit={bb_man.get(P_MOTION_LIMIT)}")

# ---- ⑫.8 IMU 交叉核对（诊断分支）：pose 判超限但 IMU 说没转 ⇒ 「链内部矛盾」----
task_im, _, _, ev_im, _ = _replay_frames(YAW_EVENT_FRAMES, with_route=False, imu_dps=0.0)
check("IMU 交叉核对：pose 超限而 IMU |gz|≈0 → 判「位姿/航向链内部不一致」",
      task_im.yaw_untrusted is True and task_im._yaw_reason == "yaw_pose_imu_mismatch"
      and task_im.yaw_imu_mismatch >= 1,
      f"reason={task_im._yaw_reason!r} mismatch={task_im.yaw_imu_mismatch}")
task_is, _, _, ev_is, _ = _replay_frames(YAW_EVENT_FRAMES, with_route=False, imu_dps=200.0)
check("IMU 交叉核对：IMU 也报高速转动 → 仍判运动学超限（不误标成矛盾）",
      task_is.yaw_untrusted is True and task_is._yaw_reason == "yaw_rate_over_kinematic"
      and task_is.yaw_imu_mismatch == 0,
      f"reason={task_is._yaw_reason!r} mismatch={task_is.yaw_imu_mismatch}")

# ---- ⑫.9 复用既有 fault 通路：EV_TASK_FAULT → SafetyTask 锁存 SAFE_STOP（不新造机制）----
task_sf, bridge_sf, bb_sf, bus_sf, clock_sf = build(
    overrides={"fix": {"anchor_at_waypoint": False}},
    route=parse_mission(list(INCIDENT_TOKENS)), seed_pose=False)
safety = SafetyTask(bridge_sf, bb_sf, bus_sf,
                    Config.from_dict({"safety": {}, "pursuit": {}, "logging": {}}),
                    log=lambda m: None, clock=clock_sf.now)
safety.setup()
for x, y, yaw_deg in YAW_EVENT_FRAMES:
    bridge_sf.set_pose(x, y, math.radians(yaw_deg))
    clock_sf.advance(0.2)
    task_sf.tick()
safety.tick()
check("复用 fault 通路：SafetyTask 收到 task_fault_event:Localization 并锁存",
      safety.stopped is True
      and any("task_fault_event:Localization" in r for r in safety.reasons),
      f"stopped={safety.stopped} reasons={safety.reasons}")
_p_safe = bb_sf.get(P_SAFETY)
check("复用 fault 通路：P_SAFETY.ok=False 且已发零速指令",
      _p_safe is not None and _p_safe.ok is False and bridge_sf.brakes >= 1,
      f"ok={getattr(_p_safe, 'ok', None)} brakes={bridge_sf.brakes}")

# --------------------------------------------------------------------------- #
#  ⑬ 折返/共线投影（2026-10-18 10:14 实车烟测：N10 M10 R:N10 在折返处 s 锁死 0.70）
# --------------------------------------------------------------------------- #
section("⑬ 折返共线投影：motion_heading + 换载重置 + 倒车方向 + 真漂移不削弱")

SMOKE_TOKENS = ["N10", "M10", "R:N10"]      # 烟测路线：3 点 / 0.70 m，最后一段倒车折返
#: 7_path.log 的 [T] 真实位姿（15 个去重采样点，yaw=度）；车从 N10 东行到 M10 前 2.6 cm。
SMOKE_POSES = [
    (0.525, 0.175, 90.5), (0.524, 0.253, 89.7), (0.524, 0.288, 89.6),
    (0.524, 0.357, 89.1), (0.524, 0.392, 89.0), (0.525, 0.461, 87.8),
    (0.526, 0.482, 87.3), (0.526, 0.499, 87.7), (0.526, 0.499, 87.8),
    (0.526, 0.499, 87.7), (0.526, 0.499, 87.8), (0.526, 0.499, 87.9),
    (0.526, 0.499, 88.0), (0.526, 0.499, 87.9), (0.526, 0.499, 88.0),
]


def _fold_s(x: float, y: float, phase: str = "out") -> float:
    """折返路线的几何真值弧长（与投影链无关）。out=去程，back=倒车回程。"""
    n10y, m10y = geo.block_to_meters("N10")[1], geo.block_to_meters("M10")[1]
    if phase == "out":
        return max(0.0, min(y - n10y, 0.35))
    return 0.35 + max(0.0, min(m10y - y, 0.35))


def _fold_rig(motion: bool = True):
    task, bridge, bb, bus, clock = build(
        overrides={"fix": {"anchor_at_waypoint": True}},
        route=parse_mission(list(SMOKE_TOKENS)), seed_pose=False)
    task.motion_heading_enabled = bool(motion)
    return task, bridge, bb, bus, clock


def _fold_replay(frames, phases, motion: bool = True):
    """按 20 Hz（每个采样点 10 拍）回放，返回 (task, bb, 最大误差, [(pose,s,truth)…])."""
    task, bridge, bb, bus, clock = _fold_rig(motion)
    worst = 0.0
    rows = []
    for idx, (x, y, yaw_deg) in enumerate(frames):
        for _ in range(10):
            bridge.set_pose(x, y, math.radians(yaw_deg))
            clock.advance(0.05)
            task.tick()
        s = bb.get(P_ROUTE_S)
        truth = _fold_s(x, y, phases[idx])
        worst = max(worst, abs(s - truth))
        rows.append((x, y, s, truth))
    return task, bb, worst, rows


# ---- ⑬.1 正例：真实烟测位姿回放（旧行为：首拍 s=0.700、末拍 0.483、drift_stop）----
task_f, bb_f, worst_f, rows_f = _fold_replay(SMOKE_POSES, ["out"] * len(SMOKE_POSES))
s_first, s_last = rows_f[0][2], rows_f[-1][2]
check("真实烟测回放：首拍 s=0.000（旧行为 0.700 = 提前锁定终点格）",
      abs(s_first) < 0.02, f"first_s={s_first:.3f}")
check("真实烟测回放：末拍 s ≤ 0.37（旧行为 0.483 回程段；真值 0.324）",
      s_last <= 0.37, f"last_s={s_last:.3f}")
check("真实烟测回放：全程 |s-真值| ≤ 0.075（含到位吸附 ≤arrival_m 的前拉）",
      worst_f <= 0.075, f"worst={worst_f:.3f}")
check("真实烟测回放：0 漂移告警（旧行为 1 次 / 偏差 0.301>0.300）",
      task_f.drift_warnings == 0, f"warnings={task_f.drift_warnings}")
check("真实烟测回放：P_MOTION_LIMIT 未被触发（旧行为 drift→drift_stop）",
      bb_f.get(P_MOTION_LIMIT) is None, f"limit={bb_f.get(P_MOTION_LIMIT)}")
check("真实烟测回放：方向拒绝闸确实工作过（不是'恰好没触发'）",
      task_f.direction_rejects >= 1 and task_f.motion_heading_ticks >= 1,
      f"rejects={task_f.direction_rejects} ticks={task_f.motion_heading_ticks}")

# ---- ⑬.2 ★带牙对照：关掉 motion_heading = 回退旧行为，必须红 ----
task_of, bb_of, worst_of, rows_of = _fold_replay(SMOKE_POSES, ["out"] * len(SMOKE_POSES),
                                                motion=False)
lim_of = bb_of.get(P_MOTION_LIMIT)
check("★带牙对照：关掉 motion_heading 后首拍 s=0.700（旧吸收态复现）",
      rows_of[0][2] > 0.5, f"first_s={rows_of[0][2]:.3f}")
# ★ 2026-10-18：新的漂移升级门限（drift_warn + 0.05、按"全路线离线"判）之后，
#   旧行为不再升级到 drift_stop（那是**修对了**：这里的"漂移"同样是投影落后引起的假信号）。
#   带牙改钉"假漂移告警 + 末拍仍落在回程侧 0.483"，motion_heading 的作用由 §⑬.1 的 s 断言覆盖。
check("★带牙对照：关掉 motion_heading 后仍报假漂移、末拍落在回程侧（旧行为）",
      task_of.drift_warnings >= 1 and rows_of[-1][2] > 0.45,
      f"warnings={task_of.drift_warnings} last_s={rows_of[-1][2]:.3f} limit={lim_of}")

# ---- ⑬.3 倒车段方向符号：一出（y↑）一回（y↓，车头不变）----
n_drive = 88
y = geo.block_to_meters("N10")[1]
_frames, _phases = [], []
for _ in range(n_drive):
    y = min(geo.block_to_meters("M10")[1], y + 0.08 * 0.05)
    _frames.append((geo.block_to_meters("N10")[0], y, 90.0))
    _phases.append("out")
for _ in range(n_drive):
    y = max(geo.block_to_meters("N10")[1], y - 0.08 * 0.05)
    _frames.append((geo.block_to_meters("N10")[0], y, 90.0))
    _phases.append("back")

task_rev, bb_rev, worst_rev, rows_rev = _fold_replay(_frames, _phases)
_s_seq = [r[2] for r in rows_rev]
check("倒车回程：s 单调不减（倒车段 s 仍沿路线前进，不倒退）",
      all(b >= a - 1e-9 for a, b in zip(_s_seq, _s_seq[1:])),
      f"min step={min(b - a for a, b in zip(_s_seq, _s_seq[1:])):+.4f}")
check("倒车回程：末拍 s≈0.70（走到路线终点）",
      abs(_s_seq[-1] - 0.70) <= 0.05, f"end_s={_s_seq[-1]:.3f}")
check("倒车回程：|s-真值| ≤ 0.125（仅到位吸附那一拍的前拉 ≤arrival_m）",
      worst_rev <= 0.125, f"worst={worst_rev:.3f}")
check("倒车回程：0 漂移告警 / 无限速（折返不被误判成漂移）",
      task_rev.drift_warnings == 0 and bb_rev.get(P_MOTION_LIMIT) is None)

# ---- ⑬.4 换载重置：上一趟终点 s=0.70 不得带进新路线 ----
task_rl, bridge_rl, bb_rl, bus_rl, clock_rl = _fold_rig()
for x, y2, yaw_deg in _frames:                 # 先跑完一整趟（末拍 s=0.70）
    bridge_rl.set_pose(x, y2, math.radians(yaw_deg))
    clock_rl.advance(0.05)
    task_rl.tick()
s_prev = bb_rl.get(P_ROUTE_S)
bb_rl.set(P_ROUTE, parse_mission(list(SMOKE_TOKENS)), source="test-reload")  # 换载（新对象）
bridge_rl.set_pose(0.525, 0.175, math.pi / 2)
clock_rl.advance(0.05)
task_rl.tick()
s_new = bb_rl.get(P_ROUTE_S)
check("换载重置：上一趟 s=0.70 没有带进新路线（首拍回到 0.0）",
      s_prev >= 0.65 and abs(s_new) < 0.02, f"prev={s_prev:.3f} new={s_new:.3f}")

# ---- ⑬.5 真漂移护栏不得削弱：横向偏 0.40 m 直行仍要报漂移并停车 ----
_drift_frames = [(geo.block_to_meters("N10")[0] + 0.40,
                  geo.block_to_meters("N10")[1] + 0.08 * 0.05 * i, 90.0) for i in range(60)]
task_d2, bb_d2, _, _ = _fold_replay(_drift_frames, ["out"] * 60)
_lim_d2 = bb_d2.get(P_MOTION_LIMIT)
check("真漂移 0.40 m：仍报漂移告警（护栏未被本轮修复削弱）",
      task_d2.drift_warnings >= 1, f"warnings={task_d2.drift_warnings}")
check("真漂移 0.40 m：仍下发 drift/drift_stop 限速",
      isinstance(_lim_d2, dict) and _lim_d2.get("reason") in ("drift", "drift_stop"),
      f"limit={_lim_d2}")

# --------------------------------------------------------------------------- #
#  ⑭ G4「按方向自愈」带牙回归（verifier C1：§⑬ 正例回放 stuck_scans=0，方向自愈分支无牙）
# --------------------------------------------------------------------------- #
section("⑭ G4 按方向自愈：冻在错误支时自愈必须回去程侧；关掉方向则落回程侧")

#: 冻结态复现台架：用公共 tick() 驱动，再把旧 bug 的**结果**（hint 冻在 0.70 + 上一拍投影
#: 已失效）注入回去，逼出 _heal_stuck 的方向裁决分支。
#: ★ 为什么直接注入 _s_hint/_last_proj/_last_seg 而不是纯回放：
#:   生产上制造这个冻结态的 bug（首拍 s=0.700）已被 §⑬ 修掉，回放再也走不到卡死分支；
#:   而 _heal_stuck 的方向分支只在这种状态下被调用 —— 不注入就无法给该分支上牙。
#:   注入的是旧 bug 的**状态结果**，不是 _heal_stuck 的入参；断言只看自愈结果落在哪一侧。
def _heal_branch_replay(motion: bool = True):
    task, bridge, bb, bus, clock = build(
        overrides={"fix": {"anchor_at_waypoint": True}},
        route=parse_mission(list(SMOKE_TOKENS)), seed_pose=False)
    task.motion_heading_enabled = bool(motion)
    for (x, y, yaw_deg) in SMOKE_POSES[:3]:      # 先走 3 个真实采样点，建立可信 travel_heading
        for _ in range(10):
            bridge.set_pose(x, y, math.radians(yaw_deg))
            clock.advance(0.05)
            task.tick()
    s_before = float(bb.get(P_ROUTE_S))
    # ---- 注入旧 bug 产生的冻结态（hint=0.70=回程段末点；上一拍投影作废，方向闸无从拒绝）----
    task._last_proj = None
    task._s_hint = 0.70
    task._last_seg = 1
    seq = []
    for y in (0.318, 0.348, 0.378, 0.408, 0.438, 0.468, 0.498):
        bridge.set_pose(0.525, y, math.pi / 2)   # 车继续向北（去程方向）走
        clock.advance(0.05)
        task.tick()
        seq.append(float(bb.get(P_ROUTE_S)))
    return task, s_before, seq


task_hm, s_before_hm, seq_hm = _heal_branch_replay(motion=True)
task_nm, s_before_nm, seq_nm = _heal_branch_replay(motion=False)
s_hm, s_nm = seq_hm[-1], seq_nm[-1]

check("自愈前置：方向开启时正常投影 s≈0.113（证明 travel_heading 已建立，不是空值兜底）",
      abs(s_before_hm - 0.113) <= 0.02, f"s_before={s_before_hm:.3f}")
check("方向自愈确实被触发（stuck_scans == 1 且 projection_heals == 1）",
      task_hm.stuck_scans == 1 and task_hm.projection_heals == 1,
      f"scans={task_hm.stuck_scans} heals={task_hm.projection_heals}")
check("方向自愈结果落在去程侧（s≈0.35 一侧，不得跳回 0.497 的回程段）",
      abs(s_hm - 0.35) <= 0.02, f"final_s={s_hm:.3f} seq={[round(x, 3) for x in seq_hm]}")
_healed_hm = [v for v in seq_hm if v <= 0.60]        # 去掉自愈触发前仍冻在 0.70 的样本
check("方向自愈后 s 只到 ≤0.36（没有落到回程段 0.497）",
      bool(_healed_hm) and max(_healed_hm) <= 0.36,
      f"healed_s={[round(x, 3) for x in _healed_hm]}")

# ★ 带牙对照：同一冻结态 + 同一位姿序列，关掉 motion_heading ⇒ 旧连续性裁决落回程段 0.497
check("★带牙对照：关掉方向后同样触发自愈（stuck_scans == 1）",
      task_nm.stuck_scans == 1, f"scans={task_nm.stuck_scans}")
check("★带牙对照：关掉方向后自愈落回回程侧 s≈0.497（旧行为）",
      abs(s_nm - 0.497) <= 0.01, f"final_s={s_nm:.3f} seq={[round(x, 3) for x in seq_nm]}")
check("★带牙对照：开/关方向的终值差 >0.10 m（该分支对结果有实质影响）",
      (s_nm - s_hm) > 0.10, f"on={s_hm:.3f} off={s_nm:.3f} diff={s_nm - s_hm:+.3f}")

# --------------------------------------------------------------------------- #
#  ⑮ 漂移升级判据：倒车/共线窗口的「投影落后」不得假停；真横向漂移仍要停
# --------------------------------------------------------------------------- #
section("⑮ 倒车窗口投影落后：不得假 drift_stop；0.40 m 真漂移仍 drift_stop")

#: field7 仿真**成功跑完**的 41 token 路线（artifacts/sim/...ld018；与 sim_t29 报告同源）。
FIELD7_TOKENS = ["N10", "M10", "L10", "K10", "J10", "I10", "H10", "H8", "R:H9", "G9", "F9*", "E9", "E8", "C8", "C7", "C6", "C5", "C4", "C3", "C2", "A2", "A1", "B1", "C1", "C3", "E3", "F3", "R:F5", "R:F4", "F3", "F2", "G2", "H2", "I2", "J2", "J4", "K4", "L4", "M4", "N4", "N5,x"]
#: 真值轨迹 t=7.0~12.6 s 的 5 Hz 采样（artifacts/sim/1792291186306_field7_route_ld018/traj.csv
#: 的 truth_x/truth_y/truth_yaw）。这一段覆盖 R:H9 折返入口：仿真真值 s 在 t=9.08 从 2.354
#: 跳到 3.245（同一点上的两条共线段弧长差 0.89 m），车端窗口投影只能慢慢追。
FIELD7_LAG_POSES = [(0.5258, 2.098, 1.52), (0.5276, 2.1219, 1.4624), (0.531, 2.1457, 1.3845), (0.537, 2.1712, 1.288), (0.5534, 2.2149, 1.1163), (0.5772, 2.2552, 0.9439), (0.6076, 2.2911, 0.7707), (0.6443, 2.3214, 0.5956), (0.6867, 2.3454, 0.416), (0.7351, 2.3621, 0.2274), (0.7906, 2.3696, 0.0211), (0.8345, 2.3677, -0.0989), (0.8754, 2.3634, -0.0943), (0.9164, 2.3613, 0.0234), (0.9574, 2.3651, 0.175), (0.9974, 2.375, 0.3266), (1.0354, 2.3908, 0.4781), (1.0705, 2.4122, 0.6293), (1.0984, 2.4352, 0.7625), (1.1222, 2.4608, 0.8912), (1.1422, 2.4885, 1.0169), (1.1581, 2.5177, 1.1392), (1.17, 2.5476, 1.2576), (1.178, 2.5776, 1.372), (1.1825, 2.6072, 1.482), (1.1836, 2.6359, 1.5876), (1.1819, 2.6633, 1.6885), (1.1777, 2.6891, 1.7848), (1.1714, 2.7132, 1.8765)]


def _lag_stop_replay():
    """把 5 Hz 真值帧 ×4 拍（= 20 Hz）喂进 LocalizationTask，记录限速原因与 drift_run 峰值。

    这一段在修复前会：车贴线正常走（全路线离线仅 0.304~0.308 m，属于折返入口的正常切角），
    但"到下一航点"的距离 >0.30 连续 ≥8 拍 ⇒ drift_run 升级 ⇒ drift_stop。
    """
    task, bridge, bb, bus, clock = build(
        overrides={}, route=parse_mission(list(FIELD7_TOKENS)), seed_pose=False)
    reasons, max_run = [], 0
    for (x, y, yaw) in FIELD7_LAG_POSES:
        for _ in range(4):
            bridge.set_pose(x, y, yaw)
            clock.advance(0.05)
            task.tick()
            lim = bb.get(P_MOTION_LIMIT)
            if isinstance(lim, dict):
                reasons.append(lim.get("reason"))
            max_run = max(max_run, task.drift_run)
    return task, bb, reasons, max_run


task_lag, bb_lag, reasons_lag, max_run_lag = _lag_stop_replay()
check("预置：落后窗口确实触发了漂移判定（drift_warnings ≥1，否则本用例没走到判据）",
      task_lag.drift_warnings >= 1, f"warnings={task_lag.drift_warnings}")
check("预置：116 拍全部参与投影（20 Hz × 29 帧）",
      task_lag.projected_ticks == len(FIELD7_LAG_POSES) * 4,
      f"projected={task_lag.projected_ticks}")
check("倒车落后窗口：drift_run 不得升级到 8 拍（正常切角 0.308 < 0.35 门限）",
      max_run_lag < 8, f"max_drift_run={max_run_lag}")
check("倒车落后窗口：不得出现 drift / drift_stop 限速请求",
      not any(r in ("drift", "drift_stop") for r in reasons_lag),
      f"reasons={sorted(set(reasons_lag))}")
check("倒车落后窗口：最终 P_MOTION_LIMIT 为 None（未被假停锁住）",
      bb_lag.get(P_MOTION_LIMIT) is None, f"limit={bb_lag.get(P_MOTION_LIMIT)}")

# ★ 对照（另一侧带牙）：0.40 m 真横向漂移必须仍然 drift_stop —— 门限 0.35 不放过真漂移
_real_drift = [(geo.block_to_meters("N10")[0] + 0.40,
                geo.block_to_meters("N10")[1] + 0.08 * 0.05 * i, 90.0) for i in range(60)]
task_rd, bb_rd, _, _ = _fold_replay(_real_drift, ["out"] * 60)
_lim_rd = bb_rd.get(P_MOTION_LIMIT)
check("★真漂移 0.40 m：仍然 drift_stop（不能因为本次修复被放过）",
      task_rd.drift_warnings >= 1 and isinstance(_lim_rd, dict)
      and _lim_rd.get("reason") == "drift_stop",
      f"warnings={task_rd.drift_warnings} limit={_lim_rd}")

finish()
