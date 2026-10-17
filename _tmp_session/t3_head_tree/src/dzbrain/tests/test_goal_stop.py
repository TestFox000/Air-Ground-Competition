# -*- coding: utf-8 -*-
"""终点停车紧判据测试（纯脚本，无 pytest）。

运行：python src\\dzbrain\\tests\\test_goal_stop.py

缺陷背景（实车实测）：车跑到终点 A10 时在离格心 ~11 cm 处刹车报 FINISH。
根因是终点判据链用了 `arrival_m`(0.12 m)：
  1. Localization 对末点也按 0.12 m 半径吸附 → 信念在 0.118 m 处被瞬移到格心；
  2. Mission `_check_finish` 用 `s + arrival_m >= s_at_waypoint(last)` → 吸附后立即 FINISH。
修复（对齐 `tools/sim.py` 的 `_GOAL_STOP_M = 0.03`）：
  · mission：末点到点/终点判据全部收紧到 `_GOAL_STOP_M`，中间航点不变；
  · localization：末点吸附触发半径收紧到 0.03 m，中间航点仍 0.12 m。

覆盖：
  1. mission：信念在末点 0.11 m 处（沿程未到）→ 不 FINISH、不锁存；
     在 0.02 m 处 → FINISH 锁存（刹车 + 事件恰好 1 次）；
  2. localization：末点 0.11 m 处不吸附、信念不被瞬移；0.02 m 处吸附且信念=末点格心；
  3. 回归哨兵：中间航点仍按 0.12 m 半径正常吸附（localization）/ 判到达（mission）。

全过 exit 0；任一失败 exit 1。
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

try:                                    # 车端/CI 控制台默认不是 UTF-8，中文会乱码
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass

from harness import check, check_close, finish, section               # noqa: E402

from dzbrain.core import geometry as geo                              # noqa: E402
from dzbrain.core.blackboard import (P_FIX, P_POSE, P_ROUTE, P_SAFETY,  # noqa: E402
                                     Blackboard)
from dzbrain.core.config import Config                                # noqa: E402
from dzbrain.core.events import (EV_MISSION_FINISHED, EV_POSE_ANCHORED,  # noqa: E402
                                 EV_WAYPOINT_REACHED, EventBus)
from dzbrain.core.state import SafetyState                            # noqa: E402
from dzbrain.io.contract import MissionPhase                          # noqa: E402
from dzbrain.route.model import Waypoint, make_route                  # noqa: E402
from dzbrain.tasks.localization import LocalizationTask               # noqa: E402
from dzbrain.tasks.mission import MissionTask                         # noqa: E402
from fakes import FakeBridge, FakeClock                               # noqa: E402

#: 测试路线：N10 → M10 → L10（同一列、沿 +y 直行，段长各 0.35 m）。
#: 末点 L10 是普通航点（非停站点），由 FINISH 处理 —— 与缺陷现场一致。
N10 = geo.block_to_meters("N10")
M10 = geo.block_to_meters("M10")
L10 = geo.block_to_meters("L10")
TOKENS = ["N10", "M10", "L10"]
LAST_IDX = len(TOKENS) - 1

GOAL_STOP_M = 0.03     # 与 mission/localization 的 `_GOAL_STOP_M` 同值（验收基准）
ARRIVAL_M = 0.12       # 非末点航点的既有到位半径（回归哨兵基准）


# --------------------------------------------------------------------------- #
#  脚手架
# --------------------------------------------------------------------------- #
def mission_rig():
    """一台 Mission 车：FakeBridge + Blackboard + MissionTask + 事件记录。"""
    clock = FakeClock()
    bridge = FakeBridge(clock)
    bb = Blackboard()
    bus = EventBus()
    logs: list[str] = []
    cfg = Config.from_dict({
        "mission": {"auto_start": True, "auto_start_delay_s": 0.1, "reset": "N10,y",
                    "scan_mode": "continuous"},
        "pursuit": {},
        "logging": {"rate_limit_s": 1.0},
        "route": {"tokens": list(TOKENS)},
    })
    task = MissionTask(bridge, bb, bus, cfg, log=logs.append, clock=clock.now)
    sub = bus.subscribe("test_goal", [EV_WAYPOINT_REACHED, EV_MISSION_FINISHED], maxsize=4096)
    events: list = []
    bb.set(P_SAFETY, SafetyState(ok=True), "test")
    bridge.set_pose(N10[0], N10[1], geo.HEADING_Y)
    bb.set(P_POSE, geo.Pose(N10[0], N10[1], geo.HEADING_Y), "test")
    task.setup()
    task.tick()

    def tick(dt: float = 0.1) -> None:
        task.tick()
        events.extend(sub.poll(4096))
        clock.advance(dt)

    def set_pose(x: float, y: float) -> None:
        bridge.set_pose(x, y, geo.HEADING_Y)
        bb.set(P_POSE, geo.Pose(x, y, geo.HEADING_Y), "test")

    def count(ev: str) -> int:
        return sum(1 for e in events if e.type == ev)

    rig = type("Rig", (), {})()
    rig.task, rig.bridge, rig.bb, rig.bus = task, bridge, bb, bus
    rig.tick, rig.set_pose, rig.count = tick, set_pose, count
    rig.logs = logs
    return rig


def loc_build():
    """一套 Localization 车：路线 N10→M10→L10，未喂位姿。"""
    clock = FakeClock()
    bridge = FakeBridge(clock)
    bb = Blackboard()
    bus = EventBus()
    cfg = Config.from_dict({
        "route": {"tokens": list(TOKENS)},
        "pursuit": {}, "fix": {}, "safety": {}, "logging": {},
        "mission": {"auto_start": False},
    })
    route = make_route([Waypoint.at_block(t) for t in TOKENS])
    bb.set(P_ROUTE, route, source="test")
    task = LocalizationTask(bridge, bb, bus, cfg, log=lambda m: None, clock=clock.now)
    sub = bus.subscribe("t_goal_anchor", [EV_POSE_ANCHORED])
    return task, bridge, bb, bus, sub


def feed_loc(task, bridge, y: float, yaw: float = 0.0) -> None:
    bridge.set_pose(L10[0], y, yaw)     # 沿路线（+y 直行）喂位姿
    task.tick()


# --------------------------------------------------------------------------- #
#  1. mission：末点紧判据
# --------------------------------------------------------------------------- #
section("1. mission 终点紧判据：0.11 m 不 FINISH，0.02 m FINISH 锁存")
rig = mission_rig()
for _ in range(3):                      # 快进到 auto_start（delay 0.1 s）→ 路线载入、DRIVE
    rig.tick()
check("1.0 前置：路线已载入、阶段 DRIVE",
      rig.bb.get(P_ROUTE) is not None and rig.task.phase == MissionPhase.DRIVE,
      f"phase={rig.task.phase!r}")

# ---- 1a 中间航点哨兵：M10 在 0.11 m 处仍按 0.12 m 判到达 ----
rig.set_pose(M10[0], M10[1] - 0.11)
rig.tick()
check("1a.1 中间航点 M10 在 0.11 m 处仍判到达（0.12 m 半径不变）",
      rig.task._passed[1] is True, f"passed={rig.task._passed}")
check("1a.2 末点 L10 未被连坐判到达（0.11 m 处不越权）",
      rig.task._passed[LAST_IDX] is False, f"passed={rig.task._passed}")
check("1a.3 阶段仍在 DRIVE（没 FINISH）", rig.task.phase == MissionPhase.DRIVE,
      f"phase={rig.task.phase!r}")

# ---- 1b 末点在 0.11 m 处（沿程未到）→ 不 FINISH ----
rig.set_pose(L10[0], L10[1] - 0.11)
for _ in range(5):
    rig.tick()
ok_1b = check("1b.1 末点 0.11 m 处：`_passed[last]` 仍为 False（信念没被瞬移、判据收紧）",
              rig.task._passed[LAST_IDX] is False, f"passed={rig.task._passed}")
ok_1b &= check("1b.2 末点 0.11 m 处：阶段不是 FINISH", rig.task.phase != MissionPhase.FINISH,
               f"phase={rig.task.phase!r}")
ok_1b &= check("1b.3 末点 0.11 m 处：没有 EV_MISSION_FINISHED",
               rig.count(EV_MISSION_FINISHED) == 0,
               f"count={rig.count(EV_MISSION_FINISHED)}")
ok_1b &= check("1b.4 末点 0.11 m 处：没有发刹车", rig.bridge.brakes == 0,
               f"brakes={rig.bridge.brakes}")

# ---- 1c 末点在 0.02 m 处 → FINISH 锁存 ----
rig.set_pose(L10[0], L10[1] - 0.02)
rig.tick()
check("1c.1 末点 0.02 m 处：阶段 FINISH", rig.task.phase == MissionPhase.FINISH,
      f"phase={rig.task.phase!r}")
check("1c.2 发过刹车", rig.bridge.brakes >= 1, f"brakes={rig.bridge.brakes}")
check("1c.3 EV_MISSION_FINISHED 恰好 1 次", rig.count(EV_MISSION_FINISHED) == 1,
      f"count={rig.count(EV_MISSION_FINISHED)}")
fin_before, brk_before = rig.count(EV_MISSION_FINISHED), rig.bridge.brakes
for _ in range(10):
    rig.tick()
check("1c.4 再跑 10 拍：FINISH 锁存（事件/刹车不重复）",
      rig.count(EV_MISSION_FINISHED) == fin_before and rig.bridge.brakes == brk_before,
      f"finished={rig.count(EV_MISSION_FINISHED)} brakes={rig.bridge.brakes}")

# --------------------------------------------------------------------------- #
#  2. localization：末点吸附收紧
# --------------------------------------------------------------------------- #
section("2. localization 末点吸附：0.11 m 不吸附，0.02 m 吸附到格心")
task, bridge, bb, bus, sub = loc_build()

# ---- 2a 中间航点哨兵：M10 在 0.11 m 处仍按 0.12 m 半径吸附 ----
for y in (0.225, 0.275, 0.325, 0.375, 0.415):
    feed_loc(task, bridge, y)
p = bb.get(P_POSE)
ok_2a = check("2a.1 中间航点 M10 在 0.11 m 处仍吸附（0.12 m 半径不变）",
              task.anchored == 1 and bb.get(P_FIX) is not None
              and bb.get(P_FIX).last_anchor == "M10",
              f"anchored={task.anchored} fix={bb.get(P_FIX)!r}")
check_close("2a.2 信念被锚回 M10 格心（x）", p.x, M10[0], 1e-12)
check_close("2a.3 信念被锚回 M10 格心（y）", p.y, M10[1], 1e-12)

# ---- 2b 末点在 0.11 m 处（沿程未到）→ 不吸附、信念不被瞬移 ----
for y in (0.525, 0.575, 0.625, 0.675, 0.725, 0.765):
    feed_loc(task, bridge, y)
p = bb.get(P_POSE)
ok_2b = check("2b.1 末点 0.11 m 处：不吸附（anchored 仍为 1，只有 M10 那次）",
              task.anchored == 1, f"anchored={task.anchored}")
ok_2b &= check_close("2b.2 末点 0.11 m 处：信念 x 不被瞬移（保持 0.11 m 偏差）",
                     p.x, L10[0], 1e-12)
ok_2b &= check_close("2b.3 末点 0.11 m 处：信念 y 不被瞬移（= L10.y - 0.11）",
                     p.y, L10[1] - 0.11, 1e-12)
evs_b = sub.poll()
ok_2b &= check("2b.4 没有为末点发 EV_POSE_ANCHORED（只有 M10 那 1 条）",
               len(evs_b) == 1, f"events={len(evs_b)}")

# ---- 2c 末点在 0.02 m 处 → 吸附且信念 = 末点格心 ----
feed_loc(task, bridge, L10[1] - 0.02)
p = bb.get(P_POSE)
check("2c.1 末点 0.02 m 处：吸附发生（anchored == 2）", task.anchored == 2,
      f"anchored={task.anchored}")
check_close("2c.2 信念 x == L10 格心 x", p.x, L10[0], 1e-12)
check_close("2c.3 信念 y == L10 格心 y", p.y, L10[1], 1e-12)
evs_c = sub.poll()
check("2c.4 末点吸附事件 block == L10",
      any(e.data.get("block") == "L10" for e in evs_c), f"events={len(evs_c)}")

# --------------------------------------------------------------------------- #
#  3. 回归哨兵：末点 0.11 m 场景下，中间航点推进/吸附半径仍是 0.12 m
# --------------------------------------------------------------------------- #
section("3. 回归哨兵汇总")
check("3.1 mission：中间航点 M10 在 0.11 m 处判到达（0.12 m 半径不变）",
      rig.task._passed[1] is True)
check("3.2 localization：中间航点 M10 在 0.11 m 处吸附（0.12 m 半径不变）",
      bool(ok_2a))
check("3.3 末点判据半径已收紧：0.11 m 时 mission 不 FINISH / localization 不吸附",
      bool(ok_1b) and bool(ok_2b))

finish()
