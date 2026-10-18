# -*- coding: utf-8 -*-
r"""D1 回归：车→机 car_state 必须回报 **StepLoop 的段状态**（P_CAR_STATE）。

运行：python src\dzbrain\tests\test_car_state_report.py

背景（2026-10-18 实车首跑）：drone.py 只读 P_MISSION_PHASE（MissionTask 的
DRIVE/FINISH），而段级真相在 P_CAR_STATE（StepLoop 的 SEGMENT_DONE/WAIT_INFO/…）。
机端按 phase 判"这段完成"，于是永远等不到 → 检查点不推进（现场只能临时改机端）。

断言：
  1. 无 P_CAR_STATE（旧行为/仿真）→ phase 仍取 P_MISSION_PHASE；
  2. 有 P_CAR_STATE → phase/segment_id/target_cell/stop_kind 随报文上报；
  3. 同一 phase 换段（SEGMENT_DONE→SEGMENT_DONE 但 segment_id 变）也要发；
  4. P_CAR_STATE 非法（非 dict / 无 phase）→ 回退旧口径且不崩。
"""

from __future__ import annotations

import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:                                    # 车端/Windows 控制台默认不是 UTF-8
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

from dzbrain.core import geometry as geo                                   # noqa: E402
from dzbrain.core.blackboard import (Blackboard, P_MISSION_PHASE, P_POSE)  # noqa: E402
from dzbrain.core.config import Config                                     # noqa: E402
from dzbrain.core.events import EventBus                                   # noqa: E402
from dzbrain.tasks.drone import DroneTask                                  # noqa: E402
from fakes import FakeBridge, FakeClock                                    # noqa: E402
from harness import check, finish, section                                 # noqa: E402

#: 与 dzbrain.main.P_CAR_STATE 同值（main → tasks 单向依赖，测试里写字面量即可）
P_CAR_STATE = "car_state"


def build():
    clock = FakeClock()
    bridge = FakeBridge(clock)
    bb = Blackboard()
    bus = EventBus()
    cfg = Config.from_dict({
        "drone": {"enabled": True, "car_state_period_s": 0.7},
        "mission": {"auto_start": False},
        "logging": {"rate_limit_s": 1.0},
    })
    bb.set(P_POSE, geo.Pose(*geo.block_to_meters("N10"), yaw=geo.HEADING_Y), source="test")
    bb.set(P_MISSION_PHASE, "DRIVE", source="test")
    task = DroneTask(bridge, bb, bus, cfg, log=lambda m: None, clock=clock.now)
    task.setup()
    return task, bridge, bb, bus, clock


def feed(task, clock, n: int = 1, dt: float = 0.2) -> None:
    for _ in range(n):
        clock.advance(dt)
        task.tick()


def last(bridge) -> dict:
    return json.loads(bridge.drone_out[-1]) if bridge.drone_out else {}


# --------------------------------------------------------------------------- #
section("① 无 P_CAR_STATE（仿真/旧行为）：phase 取 P_MISSION_PHASE=DRIVE")
task, bridge, bb, bus, clock = build()
feed(task, clock)
p = last(bridge)
check("发出过 car_state", p.get("type") == "car_state", f"{p.get('type')!r}")
check("phase == 'DRIVE'（旧口径不变）", p.get("phase") == "DRIVE", f"{p.get('phase')!r}")
check("旧字段仍在（block/yaw_deg/ts/seq）",
      all(k in p for k in ("block", "yaw_deg", "ts", "seq")), f"keys={sorted(p)}")

# --------------------------------------------------------------------------- #
section("② 有 P_CAR_STATE → 段级字段随报文上报（D1 根因）")
bb.set(P_CAR_STATE, {
    "phase": "SEGMENT_DONE", "segment_id": 3, "known_count": 4,
    "target_cell": "E5", "stop_kind": "path", "block": "E5",
    "reason": "", "version": 2,
}, source="StepLoop")
feed(task, clock)
p = last(bridge)
check("phase 改为 StepLoop 的 'SEGMENT_DONE'", p.get("phase") == "SEGMENT_DONE",
      f"{p.get('phase')!r}")
check("segment_id == 3", p.get("segment_id") == 3, f"{p.get('segment_id')!r}")
check("target_cell == 'E5'", p.get("target_cell") == "E5", f"{p.get('target_cell')!r}")
check("stop_kind == 'path'（T32 停站类型）", p.get("stop_kind") == "path",
      f"{p.get('stop_kind')!r}")
check("known_count 一并上报", p.get("known_count") == 4, f"{p.get('known_count')!r}")
check("type 仍是 car_state（机端解析不变）", p.get("type") == "car_state")

# --------------------------------------------------------------------------- #
section("③ 同一 phase 换段（segment_id 变）也要发 —— 去重键含段号")
n_before = len(bridge.drone_out)
bb.set(P_CAR_STATE, {
    "phase": "SEGMENT_DONE", "segment_id": 4, "known_count": 4,
    "target_cell": "F5", "stop_kind": "shoot", "block": "F5",
}, source="StepLoop")
feed(task, clock)
check("多发了一条（同 phase 不同段不被去重）", len(bridge.drone_out) > n_before,
      f"{n_before} → {len(bridge.drone_out)}")
p = last(bridge)
check("新段号 4 / 目标 F5 / 停站 shoot",
      (p.get("segment_id"), p.get("target_cell"), p.get("stop_kind")) == (4, "F5", "shoot"),
      f"{p}")

# --------------------------------------------------------------------------- #
section("④ P_CAR_STATE 非法 → 回退旧口径，不崩")
for bad in ("not-a-dict", 42, {"no_phase": True}, {}):
    bb.set(P_CAR_STATE, bad, source="test")
    bb.set(P_MISSION_PHASE, "WAIT_INFO", source="test")
    feed(task, clock, n=4)                      # 越过 0.7 s 周期，确保会发
    p = last(bridge)
    check(f"非法 {bad!r} 时 phase 回退 'WAIT_INFO'", p.get("phase") == "WAIT_INFO",
          f"{p.get('phase')!r}")

finish()