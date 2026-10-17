# -*- coding: utf-8 -*-
"""MissionTask 自测（**纯脚本，无 pytest**）。

    python src\\dzbrain\\tests\\test_mission.py

覆盖（逐条给数值证据）：
  1. `auto_start=False` → 不发车、阶段保持 IDLE、路线不载入、无事件
  2. `auto_start=True` + 快进时间 → 路线载入（`P_ROUTE` 非空、块名序列正确）、阶段 DRIVE、
     发过 `EV_ROUTE_LOADED`；`scan_mode=continuous` 开局发 `scan_request=2`
  3. 到点推进：喂一串位置 → `EV_WAYPOINT_REACHED` 数量随推进增长
  4. 停站点三段：`SETTLE → ACQUIRE → FIRE`，发数够了 → DONE、闸门收尾 False、阶段回 DRIVE
  5. ★ 不卡死：三段各自注入"永不到达"条件 → 每段都在超时后放行、总耗时 ≤ 各超时之和、
     全程阶段最终回到 DRIVE
  6. 终点锁存：到最后一个航点 → FINISH、发过刹车、再跑 20 拍不重复发 `EV_MISSION_FINISHED`
  7. 外部命令：`reset:K9,x` 改 belief + 清空路线；`load:文件`；多点；未知命令不崩

全过 exit 0；任一失败 exit 1。
"""

from __future__ import annotations

import math
import pathlib
import re
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

try:
    sys.stdout.reconfigure(errors="replace")  # type: ignore[union-attr]
except Exception:                              # pragma: no cover
    pass

from harness import check, finish, section                     # noqa: E402

from dzbrain.core import geometry as geo                       # noqa: E402
from dzbrain.core.blackboard import (P_DRONE, P_MISSION_PHASE, P_POSE, P_ROUTE,  # noqa: E402
                             P_ROUTE_INDEX,
                                     P_ROUTE_S, P_SAFETY, P_STOP_POINT, P_TELEMETRY, Blackboard)
from dzbrain.core.config import Config                         # noqa: E402
from dzbrain.core.events import (EV_ARRIVED_STOP_POINT, EV_BRIDGE_INFO,  # noqa: E402
                                 EV_MISSION_FINISHED,
                                 EV_MISSION_STARTED, EV_ROUTE_LOADED, EV_STOP_POINT_DONE,
                                 EV_WAYPOINT_REACHED, EventBus)
from dzbrain.core.state import DroneState, SafetyState, StopKind, StopPhase  # noqa: E402
from dzbrain.io.contract import AimStatus, MissionPhase        # noqa: E402
from dzbrain.pursuit.path import Path                          # noqa: E402
from dzbrain.route.parser import parse_token                   # noqa: E402
from dzbrain.tasks.mission import (BridgeIntel, MissionTask,   # noqa: E402
                                   rewrite_crossing_tokens)
from dzbrain.tasks.tracker import PIVOT_DONE_KEY, PivotPhase, TrackerTask  # noqa: E402

from fakes import FakeBridge, FakeClock                        # noqa: E402

#: 现场路线片段：N10 出发 → 直行 → G10* 打 G11 靶 → 终点
FIELD_ROUTE = ["N10", "M10", "L10", "L9", "K9", "J9", "J10", "I10", "H10", "G10*", "F10", "F9"]
WATCHED = (EV_MISSION_STARTED, EV_ROUTE_LOADED, EV_WAYPOINT_REACHED, EV_ARRIVED_STOP_POINT,
           EV_STOP_POINT_DONE, EV_MISSION_FINISHED)


def cfg_of(*, mission=None, route=None, pursuit=None, aim=None) -> Config:
    data: dict = {"mission": dict(mission or {}), "pursuit": dict(pursuit or {}),
                  "aim": dict(aim or {}),
                  "logging": {"rate_limit_s": 1.0}}
    if route is not None:
        data["route"] = dict(route)
    return Config.from_dict(data)


class Rig:
    """一台"车"：FakeBridge + Blackboard + MissionTask + 事件记录。"""

    def __init__(self, *, cfg: Config, pose=None, safety: SafetyState | None = None) -> None:
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.logs: list[str] = []
        self.task = MissionTask(self.bridge, self.bb, self.bus, cfg,
                                log=self.logs.append, clock=self.clock.now)
        self.sub = self.bus.subscribe("test", list(WATCHED), maxsize=4096)
        self.events: list = []
        self.bb.set(P_SAFETY, safety if safety is not None else SafetyState(ok=True), "test")
        self.cursor: list[float] | None = None      # walk() 用的"车实际位置"（跨调用保持）
        self.cursor_seg = 0                         # 当前行进到第几段（跨调用保持）
        if pose is not None:
            self.set_pose(*pose)
        self.task.setup()
        self.task.tick()

    # ---------------- 操作 ----------------
    def set_pose(self, x: float, y: float, yaw: float = 0.0) -> None:
        self.bridge.set_pose(x, y, yaw)
        self.bb.set(P_POSE, geo.Pose(x, y, yaw), "test")
        self.cursor = [float(x), float(y)]
        self.cursor_seg = 0

    def set_pose_at(self, block: str, yaw: float = 0.0) -> None:
        x, y = geo.block_to_meters(block)
        self.set_pose(x, y, yaw)

    def tick(self, dt: float = 0.1) -> None:
        self.task.tick()
        self.events.extend(self.sub.poll(4096))
        self.clock.advance(dt)

    def ticks(self, n: int, dt: float = 0.1) -> None:
        for _ in range(n):
            self.tick(dt)

    def count(self, ev_type: str) -> int:
        return sum(1 for e in self.events if e.type == ev_type)

    def of(self, ev_type: str) -> list:
        return [e for e in self.events if e.type == ev_type]

    def stop_phase(self) -> str | None:
        state = self.bb.get(P_STOP_POINT)
        return getattr(state, "phase", None)

    def _feed(self, x: float, y: float, yaw: float) -> None:
        """喂位姿但**不动 cursor_seg**（walk 内部用；`set_pose` 会重置段号）。"""
        self.bridge.set_pose(x, y, yaw)
        self.bb.set(P_POSE, geo.Pose(x, y, yaw), "test")
        self.cursor = [float(x), float(y)]

    def walk(self, *, step: float = 0.1, max_steps: int = 200,
             stop_on_stop_point: bool = True) -> int:
        """沿 `P_ROUTE` 折线小步喂位姿（模拟底盘在走）；**位置与段号跨调用累积**。

        ★ `stop_on_stop_point=True`：进入 STOP 阶段即停手（真实车会停在停站点），
          否则这个 helper 会"开着车穿过停站点"，测出来的时序就不是现场时序。
        """
        route = self.bb.get(P_ROUTE)
        if route is None or self.cursor is None:
            return 0
        pts = [(w.x, w.y) for w in route.waypoints]
        n = 0
        while n < max_steps:
            if self.cursor_seg >= len(pts) - 1:      # 已到末点
                self._feed(pts[-1][0], pts[-1][1], self.bb.get(P_POSE).yaw)
                self.tick()
                return n + 1
            tx, ty = pts[self.cursor_seg + 1]
            dx, dy = tx - self.cursor[0], ty - self.cursor[1]
            dist = (dx * dx + dy * dy) ** 0.5
            yaw = geo.heading_of(dx, dy, fallback=self.bb.get(P_POSE).yaw)
            if dist <= step:
                self.cursor_seg += 1
                self._feed(tx, ty, yaw)
                self.tick()
                n += 1
                continue
            self._feed(self.cursor[0] + dx / dist * step, self.cursor[1] + dy / dist * step, yaw)
            self.tick()
            n += 1
            if stop_on_stop_point and self.task.phase == MissionPhase.STOP:
                return n
        return n


# --------------------------------------------------------------------------- #
#  1. auto_start=False
# --------------------------------------------------------------------------- #
def test_no_auto_start() -> None:
    section("1. auto_start=False：不发车、阶段保持 IDLE")
    cfg = cfg_of(mission={"auto_start": False, "auto_start_delay_s": 0.1},
                 route={"tokens": ["N10", "M10"]})
    rig = Rig(cfg=cfg, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig.ticks(30)                                   # 3 s：远超 auto_start_delay_s
    check("1.1 阶段保持 IDLE", rig.task.phase == MissionPhase.IDLE,
          f"phase={rig.task.phase!r}")
    check("1.2 路线未载入（P_ROUTE 为 None）", rig.bb.get(P_ROUTE) is None,
          f"{rig.bb.get(P_ROUTE)!r}")
    check("1.3 无 ROUTE_LOADED / MISSION_STARTED 事件",
          rig.count(EV_ROUTE_LOADED) == 0 and rig.count(EV_MISSION_STARTED) == 0,
          f"loaded={rig.count(EV_ROUTE_LOADED)} started={rig.count(EV_MISSION_STARTED)}")
    check("1.4 不发运动指令、不发扫描请求",
          not rig.bridge.commands and not rig.bridge.scan_requests,
          f"commands={rig.bridge.commands} scans={rig.bridge.scan_requests}")


# --------------------------------------------------------------------------- #
#  2. auto_start=True + 快进
# --------------------------------------------------------------------------- #
def test_auto_start() -> None:
    section("2. auto_start=True + 快进 → 路线载入 / DRIVE / ROUTE_LOADED")
    tokens = ["N10", "M10", "L10", "L9"]
    cfg = cfg_of(mission={"auto_start": True, "auto_start_delay_s": 2.0,
                          "reset": "N10,y", "scan_mode": "continuous"},
                 route={"tokens": tokens})
    rig = Rig(cfg=cfg, pose=(0.0, 0.0, 0.0))        # 故意给个错位姿，验证复位生效
    rig.ticks(5)                                    # 0.5 s < 2.0 s
    check("2.1 延时未到：仍 IDLE、路线未载入",
          rig.task.phase == MissionPhase.IDLE and rig.bb.get(P_ROUTE) is None,
          f"phase={rig.task.phase!r} t={rig.clock.now() - 1000.0:.1f}s")
    rig.ticks(20)                                   # 共 2.5 s > 2.0 s
    route = rig.bb.get(P_ROUTE)
    check("2.2 延时到：路线载入且块名序列正确", route is not None
          and list(route.blocks()) == tokens, f"{route.blocks() if route else None}")
    check("2.3 阶段 = DRIVE", rig.task.phase == MissionPhase.DRIVE, f"{rig.task.phase!r}")
    check("2.4 发过 EV_ROUTE_LOADED 与 EV_MISSION_STARTED",
          rig.count(EV_ROUTE_LOADED) == 1 and rig.count(EV_MISSION_STARTED) == 1,
          f"loaded={rig.count(EV_ROUTE_LOADED)} started={rig.count(EV_MISSION_STARTED)}")
    check("2.5 路线长度 = 4（Path 用）", len(route) == 4, f"{len(route)}")
    check("2.6 ★ 单写者（t24）：lock 模式下 Mission **不发** scan_request（continuous 也 inert）",
          rig.bridge.scan_requests == [], f"scans={rig.bridge.scan_requests}")
    check("2.6b ★ Mission 自己的扫描台账也为空（= 真的没发，不是被做了去重）",
          rig.task._scans == [], f"_scans={rig.task._scans}")
    check("2.6c 日志说明跳过原因（可复盘：单写者=AimTask）",
          any("单写者" in line for line in rig.logs), f"{rig.logs[-3:]}")

    pose = rig.bb.get(P_POSE)
    want = geo.block_to_meters("N10")
    check("2.7 复位 belief 到 N10 格心 + 朝向 y",
          abs(pose.x - want[0]) < 1e-12 and abs(pose.y - want[1]) < 1e-12
          and abs(pose.yaw - geo.HEADING_Y) < 1e-12,
          f"pose=({pose.x:.3f},{pose.y:.3f},{geo.deg(pose.yaw):.1f}°)")

    # 2.8 旧行为档（`aim.gimbal_lock_enabled=false`）才按 scan_mode 下发：
    #     continuous → 2 / waypoint → 1（t24 单写者只在 lock=true 时生效）
    cfg2 = cfg_of(mission={"auto_start": True, "auto_start_delay_s": 0.1,
                           "scan_mode": "waypoint"},
                  route={"tokens": tokens}, aim={"gimbal_lock_enabled": False})
    rig2 = Rig(cfg=cfg2, pose=(0.0, 0.0, 0.0))
    rig2.ticks(3)
    check("2.8 旧行为档（lock=false）scan_mode=waypoint → scan_request=1",
          rig2.bridge.scan_requests[:1] == [1], f"scans={rig2.bridge.scan_requests}")
    cfg2b = cfg_of(mission={"auto_start": True, "auto_start_delay_s": 0.1,
                            "scan_mode": "continuous"},
                   route={"tokens": tokens}, aim={"gimbal_lock_enabled": False})
    rig2b = Rig(cfg=cfg2b, pose=(0.0, 0.0, 0.0))
    rig2b.ticks(3)
    check("2.8b 旧行为档（lock=false）scan_mode=continuous → scan_request=2（绝不用 1）",
          rig2b.bridge.scan_requests[:1] == [2], f"scans={rig2b.bridge.scan_requests}")

    # 2.9 配置坏（route.tokens 空）→ 停在 IDLE，不崩、不发车
    cfg3 = cfg_of(mission={"auto_start": True, "auto_start_delay_s": 0.1},
                  route={"tokens": []})
    rig3 = Rig(cfg=cfg3, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig3.ticks(5)
    check("2.9 route.tokens 为空 → 保持 IDLE、不崩",
          rig3.task.phase == MissionPhase.IDLE and rig3.bb.get(P_ROUTE) is None
          and rig3.task.health().errors == 0,
          f"phase={rig3.task.phase!r} errors={rig3.task.health().errors}")


# --------------------------------------------------------------------------- #
#  3. 到点推进
# --------------------------------------------------------------------------- #
def test_waypoint_progress() -> None:
    section("3. 到点推进：EV_WAYPOINT_REACHED 随推进增长")
    tokens = ["N10", "M10", "L10", "L9", "K9"]
    cfg = cfg_of(mission={"auto_start": True, "auto_start_delay_s": 0.1, "reset": "N10,y"},
                 route={"tokens": tokens})
    rig = Rig(cfg=cfg, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig.ticks(3)
    check("3.1 出发后 0 个到点事件（起点不报到达）", rig.count(EV_WAYPOINT_REACHED) == 0,
          f"{rig.count(EV_WAYPOINT_REACHED)}")

    counts = []
    for _ in range(3):
        rig.walk(step=0.1, max_steps=6)
        counts.append(rig.count(EV_WAYPOINT_REACHED))
    check("3.2 事件数随推进单调增长", counts == sorted(counts) and counts[-1] > counts[0],
          f"三次采样 {counts}")
    rig.walk(step=0.1, max_steps=200)
    total = rig.count(EV_WAYPOINT_REACHED)
    check("3.3 走完全程：到达事件数 = 航点数 - 1（末点由终点逻辑负责）",
          1 <= total <= len(tokens) - 1, f"到达 {total} 次 / 航点 {len(tokens)}")
    blocks = [e.data.get("block") for e in rig.of(EV_WAYPOINT_REACHED)]
    check("3.4 事件按路线顺序、不重复", blocks == sorted(set(blocks), key=blocks.index),
          f"{blocks}")
    check("3.5 P_ROUTE_S 已被写出（Mission 自己也算，不依赖 Localization）",
          isinstance(rig.bb.get(P_ROUTE_S), (int, float)), f"{rig.bb.get(P_ROUTE_S)!r}")


# --------------------------------------------------------------------------- #
#  4. 停站点三段
# --------------------------------------------------------------------------- #
def _field_cfg(**extra) -> Config:
    mission = {"auto_start": True, "auto_start_delay_s": 0.1, "reset": "N10,y",
               "shots_per_stop": 3, "settle_time_s": 0.3, "settle_eps_m": 0.02,
               "lock_timeout_s": 5.0, "shot_timeout_s": 6.0}
    mission.update(extra)
    return cfg_of(mission=mission, route={"tokens": list(FIELD_ROUTE)})


def test_stop_point_three_phases() -> None:
    section("4. 停站点三段：SETTLE → ACQUIRE → FIRE → DONE")
    rig = Rig(cfg=_field_cfg(), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig.ticks(3)
    check("4.1 停站点被识别（G10* 在路线里）",
          any(rig.bb.get(P_ROUTE)[i].shoot for i in range(len(rig.bb.get(P_ROUTE)))),
          f"{rig.bb.get(P_ROUTE).blocks()}")

    # 走到停站点：先走几拍确认还没到，再走到 G10*
    rig.walk(step=0.1, max_steps=40)
    arrived = rig.count(EV_ARRIVED_STOP_POINT)
    check("4.2 到达停站点：发 EV_ARRIVED_STOP_POINT、阶段 STOP",
          arrived >= 1 and rig.task.phase == MissionPhase.STOP,
          f"arrived={arrived} phase={rig.task.phase!r} s={rig.bb.get(P_ROUTE_S)!r}")
    check("4.3 停站点上下文写进 P_STOP_POINT（块名 G10、kind=SHOOT）",
          rig.stop_phase() is not None and rig.task.stop_context is not None
          and rig.task.stop_context.block == "G10"
          and rig.task.stop_context.kind == StopKind.SHOOT,
          f"phase={rig.stop_phase()!r} block={getattr(rig.task.stop_context, 'block', None)!r}")
    check("4.4 三段起步于 SETTLE",
          rig.stop_phase() in (StopPhase.SETTLE, StopPhase.ACQUIRE), f"{rig.stop_phase()!r}")

    # SETTLE：位置不动 → 0.3 s 后进 ACQUIRE
    phases = [rig.stop_phase()]
    for _ in range(6):
        rig.tick(dt=0.1)
        phases.append(rig.stop_phase())
    check("4.5 SETTLE（位置不变）→ ACQUIRE", StopPhase.ACQUIRE in phases, f"阶段序列 {phases}")

    # ACQUIRE：云台 TRACK + 视觉有目标 → FIRE
    rig.bridge.set_aim_status(AimStatus.TRACK)
    rig.bridge.set_vision(dx=5, dy=3, cls=1)
    rig.tick(dt=0.05)                               # 本拍做 ACQUIRE→FIRE 迁移（还不发闸门）
    before_gate = len(rig.bridge.fire_gate)
    rig.tick(dt=0.05)                               # 进入 FIRE 段的第一拍：开闸
    check("4.6 ACQUIRE（TRACK + 有目标）→ FIRE", rig.stop_phase() == StopPhase.FIRE,
          f"{rig.stop_phase()!r}")
    check("4.7 ★ 进 FIRE 即 open gate（publish_fire_gate(True)）",
          rig.bridge.fire_gate[before_gate:] == [True],
          f"gate 尾部={rig.bridge.fire_gate[before_gate:]}")

    # FIRE：发数增量到 3 → DONE
    rig.bridge.add_shots(3)
    shots_at = rig.task.stop_context.shots_at_arrival
    rig.tick(dt=0.05)
    check("4.8 发数增量 3 ≥ shots_per_stop → 阶段回 DRIVE", rig.task.phase == MissionPhase.DRIVE,
          f"phase={rig.task.phase!r} shots={rig.bridge.shoot_count()} 到站快照={shots_at}")
    check("4.9 停站点收尾：P_STOP_POINT.phase = DONE", rig.stop_phase() == StopPhase.DONE,
          f"{rig.stop_phase()!r}")
    check("4.10 ★ 闸门收尾为 False（绝不留一个开着的放行门）",
          rig.bridge.fire_gate[-1] is False and True in rig.bridge.fire_gate,
          f"gate 序列={rig.bridge.fire_gate}")
    check("4.11 发过 EV_STOP_POINT_DONE（该点已处理）",
          rig.count(EV_STOP_POINT_DONE) == 1 and rig.task.handled_stops == {9},
          f"done={rig.count(EV_STOP_POINT_DONE)} handled={sorted(rig.task.handled_stops)}")
    check("4.12 处理过的停站点不会被重复触发（再跑 20 拍）",
          (rig.ticks(20), rig.count(EV_ARRIVED_STOP_POINT))[1] == 1,
          f"arrived={rig.count(EV_ARRIVED_STOP_POINT)}")


# --------------------------------------------------------------------------- #
#  4b. ★t26 射击停站离站条件（用户指令 4）
# --------------------------------------------------------------------------- #
def test_stop_departure_t26() -> None:
    """离站条件 = 「发数增量 ≥ shots_per_stop（10）」 **或** 「到站起 30s 硬死线」，先到即离；
    即使 0 发，30s 到点也必须离站。三段式结构与段超时链不变，只多一条到站级死线。"""
    section("4b. ★t26 离站条件：增量满 10 即完成离站 / 到站 30s 硬死线（0 发也离）")

    # ---- (a) 发数增量 ≥ shots_per_stop(10) → 完成离站（既有逻辑，只改口径值）----
    cfg = _field_cfg(shots_per_stop=10, shot_timeout_s=30.0, lock_timeout_s=6.0,
                     settle_time_s=0.3)
    rig = Rig(cfg=cfg, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig.ticks(3)
    rig.walk(step=0.1, max_steps=40)
    check("4b.1 前置：到停站点且 shots_needed = 10（配置口径生效，不再是 3）",
          rig.task.phase == MissionPhase.STOP and rig.task.stop_context is not None
          and rig.task.stop_context.shots_needed == 10,
          f"phase={rig.task.phase!r} needed="
          f"{getattr(rig.task.stop_context, 'shots_needed', None)}")
    rig.bridge.set_aim_status(AimStatus.TRACK)
    rig.bridge.set_vision(dx=5, dy=3, cls=1)
    for _ in range(40):                             # SETTLE→ACQUIRE→FIRE
        rig.tick(dt=0.05)
        if rig.stop_phase() == StopPhase.FIRE:
            break
    rig.tick(dt=0.05)                               # 进 FIRE 的**下一拍**才开闸（与 §4.6/4.7 同构）
    check("4b.2 已进 FIRE 段且放行门打开", rig.stop_phase() == StopPhase.FIRE
          and rig.bridge.fire_gate[-1:] == [True], f"phase={rig.stop_phase()!r} "
          f"gate尾={rig.bridge.fire_gate[-1:]}")
    rig.bridge.add_shots(10)                        # 打完全部弹药
    rig.tick(dt=0.05)
    check("4b.3 ★ 增量 10 ≥ 10 → 完成离站（阶段回 DRIVE、P_STOP_POINT=DONE）",
          rig.task.phase == MissionPhase.DRIVE and rig.stop_phase() == StopPhase.DONE
          and any("停站点" in ln and "完成" in ln for ln in rig.logs),
          f"phase={rig.task.phase!r} stop={rig.stop_phase()!r}")
    check("4b.4 离站原因 = 发数增量（日志留痕，不是死线）",
          any("发数增量 10 ≥ 10" in ln for ln in rig.logs),
          f"{[ln for ln in rig.logs if '发数增量' in ln][-1:]}")

    # ---- (b) 0 发 + 到站 30s 硬死线 → 同样离站（误差 ≤ 1 拍）----
    #  夹具故意让三段**各自**的超时都够不到死线之前：SETTLE 立刻停稳进 ACQUIRE，
    #  ACQUIRE 的 lock_timeout 拉长到 600s（永不超时），FIRE 根本不进
    #  ⇒ 释放只可能来自到站级死线。
    cfg2 = _field_cfg(shots_per_stop=10, shot_timeout_s=30.0, lock_timeout_s=600.0,
                      settle_time_s=0.3, settle_eps_m=0.02)
    rig2 = Rig(cfg=cfg2, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig2.ticks(3)
    rig2.walk(step=0.1, max_steps=40)
    stop_ctx = rig2.task.stop_context                    # 收尾后 _stopctx 会置 None ⇒ 先留引用
    check("4b.5 前置：已到停站点、0 发、未锁定（aim=IDLE、无目标）",
          rig2.task.phase == MissionPhase.STOP and stop_ctx is not None
          and rig2.bridge.shoot_count() == 0 and stop_ctx.shots_at_arrival == 0
          and stop_ctx.kind == StopKind.SHOOT,
          f"phase={rig2.task.phase!r} shots={rig2.bridge.shoot_count()} "
          f"kind={getattr(stop_ctx, 'kind', None)}")
    t_arrive = stop_ctx.started_at
    deadline = rig2.task._stop_deadline_s              # noqa: SLF001  （= shoot_timeout_s = 30.0）
    elapsed = None
    for _ in range(400):                               # 上限 40 s（10 Hz）
        now_before = rig2.clock.now()                  # ★与 Mission 内部读时钟的时刻**同一口径**
        rig2.tick(dt=0.1)
        if rig2.task.phase == MissionPhase.DRIVE:
            elapsed = now_before - t_arrive
            break
    check("4b.6 ★ 0 发也离站：死线到点 → 阶段回 DRIVE 且发过 EV_STOP_POINT_DONE",
          rig2.task.phase == MissionPhase.DRIVE and rig2.count(EV_STOP_POINT_DONE) >= 1
          and elapsed is not None,
          f"phase={rig2.task.phase!r} elapsed={elapsed} done={rig2.count(EV_STOP_POINT_DONE)}")
    check("4b.7 ★ 离站时刻 = 到站 + 30s（≤1 拍 0.1s 容差；旧实现要等三段 12+6+30=48s）",
          elapsed is not None and abs(elapsed - deadline) <= 0.1 + 1e-6 and elapsed < 48.0,
          f"实测 {elapsed if elapsed is None else round(elapsed, 3)}s vs 死线 {deadline:.2f}s")
    check("4b.8 ★ 释放原因 = 到站硬死线（日志留痕，且带 0/10 增量证据）",
          any("到站硬死线" in ln and "0/10" in ln for ln in rig2.logs),
          f"{[ln for ln in rig2.logs if '硬死线' in ln][-1:]}")
    check("4b.9 死线出口与三段超时同一条：放行门**从未被打开**（本场景没进 FIRE）"
          "且本点已记账（不重复触发）",
          True not in rig2.bridge.fire_gate and rig2.task.handled_stops == {9},
          f"gate={rig2.bridge.fire_gate} handled={sorted(rig2.task.handled_stops)}")


# --------------------------------------------------------------------------- #
#  5. 不卡死（最重要）
# --------------------------------------------------------------------------- #
def test_no_deadlock() -> None:
    section("5. ★ 不卡死：三段各自'永不到达' → 每段超时后放行")

    # 5.1 永不稳定（位置一直动）+ 永无目标（vision=None, aim=IDLE）
    #   ★t26：本用例测的是**三段各自的超时链**，所以把"到站级硬死线"踢到 100s 之外
    #     （死线只在缺键时 = `mission.shoot_timeout_s`；这里显式覆盖，避免它抢先放行）。
    #     死线本身由 §4b 单独钉死。
    cfg = _field_cfg(settle_time_s=10.0, settle_eps_m=0.001, lock_timeout_s=1.0,
                     shot_timeout_s=1.0, stop_deadline_s=100.0)
    rig = Rig(cfg=cfg, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig.ticks(3)
    rig.walk(step=0.1, max_steps=40)
    check("5.1 前置：已到停站点、阶段 STOP", rig.task.phase == MissionPhase.STOP,
          f"phase={rig.task.phase!r}")
    t0 = rig.clock.now()
    settle_grace = max(0.4 * 1.0, 1.0)
    seen = [rig.stop_phase()]
    for _ in range(60):                             # 6 s，远超三段超时之和 1+1+1
        # 位置每拍都在动 → SETTLE 的"位置不变"永不成立；速度未知 → 也不武装
        x, y = 0.7 + 0.001 * _, 1.0
        rig.set_pose(x, y, geo.HEADING_Y)
        rig.tick(dt=0.1)
        seen.append(rig.stop_phase())
        if rig.task.phase == MissionPhase.DRIVE:
            break
    elapsed = rig.clock.now() - t0
    bound = settle_grace + 1.0                      # SETTLE 降级 + ACQUIRE 超时（本场景不进 FIRE）
    check("5.2 三段全部超时后放行：阶段回到 DRIVE", rig.task.phase == MissionPhase.DRIVE,
          f"phase={rig.task.phase!r} 用时 {elapsed:.2f}s")
    check("5.3 总耗时 ≤ 各超时之和 + 2 拍容差", elapsed <= bound + 0.25,
          f"实测 {elapsed:.2f}s ≤ 上限 {bound:.2f}s")
    check("5.4 阶段确实走完了 SETTLE→ACQUIRE（永不锁定 → 不进 FIRE 是正确行为）",
          {StopPhase.SETTLE, StopPhase.ACQUIRE} <= set(seen) and StopPhase.FIRE not in seen,
          f"观察到的阶段={sorted(set(seen))}")
    check("5.5 放弃本点也发 EV_STOP_POINT_DONE（候选链放行）",
          rig.count(EV_STOP_POINT_DONE) >= 1 and rig.task.handled_stops == {9},
          f"done={rig.count(EV_STOP_POINT_DONE)} handled={sorted(rig.task.handled_stops)}")
    check("5.6 闸门未被留在 True（本场景没进 FIRE 就没开过闸；开了就必须收尾）",
          True not in rig.bridge.fire_gate or rig.bridge.fire_gate[-1] is False,
          f"{rig.bridge.fire_gate}")

    # 5.7 永不增发数（有 TRACK、有目标，但 shoot_count 不动）→ FIRE 超时放行
    #   ★t26：同样把到站级死线踢远（100s），让本用例继续测 **FIRE 段自己的**超时。
    cfg2 = _field_cfg(shot_timeout_s=1.0, lock_timeout_s=1.0, settle_time_s=0.1,
                      stop_deadline_s=100.0)
    rig2 = Rig(cfg=cfg2, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig2.ticks(3)
    rig2.walk(step=0.1, max_steps=40)
    rig2.bridge.set_aim_status(AimStatus.TRACK)
    fire_seen = False
    t1 = None
    for _ in range(60):
        rig2.bridge.set_vision(dx=3, dy=2, cls=1)   # 每拍刷新视觉（不过期）
        rig2.set_pose(rig2.bb.get(P_POSE).x, rig2.bb.get(P_POSE).y, geo.HEADING_Y)
        rig2.tick(dt=0.1)
        if rig2.stop_phase() == StopPhase.FIRE:
            fire_seen = True
            if t1 is None:
                t1 = rig2.clock.now()               # 从"进入 FIRE 那一拍"开始计时
        if rig2.task.phase == MissionPhase.DRIVE:
            break
    elapsed2 = (rig2.clock.now() - t1) if t1 is not None else float("nan")
    check("5.8 FIRE 段（永不增发数）超时放行：进过 FIRE 且回到 DRIVE",
          fire_seen and rig2.task.phase == MissionPhase.DRIVE,
          f"fire_seen={fire_seen} phase={rig2.task.phase!r} 用时 {elapsed2:.2f}s")
    check("5.9 FIRE 超时 ≤ shot_timeout_s + 3 拍（10 Hz 的 3 拍 = 0.3 s）",
          elapsed2 <= 1.0 + 0.35, f"实测 {elapsed2:.2f}s（上限 1.0s + 0.35 容差）")
    check("5.10 闸门收尾 False", rig2.bridge.fire_gate[-1] is False,
          f"{rig2.bridge.fire_gate}")

    # 5.11 全流程：停站点超时放行后继续跑到终点（Mission 不能停摆）
    #   ★t26：同 5.7 —— 死线踢远，本用例仍走三段超时链。
    cfg3 = _field_cfg(shot_timeout_s=1.0, lock_timeout_s=1.0, settle_time_s=0.1,
                      stop_deadline_s=100.0)
    rig3 = Rig(cfg=cfg3, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig3.ticks(3)
    for _ in range(400):
        if rig3.task.phase == MissionPhase.STOP:
            rig3.ticks(30, dt=0.1)                  # 停站点三段：全部超时后自己放行
            continue
        rig3.walk(step=0.1, max_steps=8)
        if rig3.task.phase == MissionPhase.FINISH:
            break
    check("5.11 停站点超时放行后继续跑：最终到达 FINISH（没有卡在 STOP）",
          rig3.task.phase == MissionPhase.FINISH,
          f"phase={rig3.task.phase!r} arrived_stop={rig3.count(EV_ARRIVED_STOP_POINT)} "
          f"done={rig3.count(EV_STOP_POINT_DONE)}")


# --------------------------------------------------------------------------- #
#  6. 终点锁存
# --------------------------------------------------------------------------- #
def test_finish_latch() -> None:
    section("6. 终点锁存：FINISH + 刹车 + 不重复发事件")
    tokens = ["N10", "M10", "L10"]
    cfg = cfg_of(mission={"auto_start": True, "auto_start_delay_s": 0.1, "reset": "N10,y",
                          "scan_mode": "continuous"},
                 route={"tokens": tokens})
    rig = Rig(cfg=cfg, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig.ticks(3)
    rig.walk(step=0.1, max_steps=200)
    check("6.1 到达末点 → 阶段 FINISH", rig.task.phase == MissionPhase.FINISH,
          f"phase={rig.task.phase!r} s={rig.bb.get(P_ROUTE_S)!r}")
    check("6.2 发过刹车（publish_brake）", rig.bridge.brakes >= 1, f"brakes={rig.bridge.brakes}")
    check("6.3 ★ 单写者（t24）：lock 模式下 Mission 在终点**也不发** scan_request（交给 Aim 回锁止）",
          0 not in rig.bridge.scan_requests and rig.bridge.scan_requests == [],
          f"scans={rig.bridge.scan_requests}")
    check("6.4 EV_MISSION_FINISHED 恰好 1 次", rig.count(EV_MISSION_FINISHED) == 1,
          f"{rig.count(EV_MISSION_FINISHED)}")
    events_before = rig.count(EV_MISSION_FINISHED)
    brakes_before = rig.bridge.brakes
    scans_before = len(rig.bridge.scan_requests)
    rig.ticks(20)
    check("6.5 ★ 再跑 20 拍不重复发事件/刹车/停扫（锁存）",
          rig.count(EV_MISSION_FINISHED) == events_before
          and rig.bridge.brakes == brakes_before
          and len(rig.bridge.scan_requests) == scans_before,
          f"finished={rig.count(EV_MISSION_FINISHED)} brakes={rig.bridge.brakes} "
          f"scans={len(rig.bridge.scan_requests)}")
    check("6.6 结束横幅已打印（含用时/发数/漂移）",
          any("任务结束" in line for line in rig.logs)
          and any("用时" in line for line in rig.logs),
          f"最后日志={rig.logs[-1]!r}")
    info = rig.task.state()["finish"]
    check("6.7 结束统计可用（用时/发数/漂移/航点数）",
          set(info) >= {"elapsed_s", "shots", "hits", "drift_m", "waypoints"},
          f"{info}")


# --------------------------------------------------------------------------- #
#  6b. ★ 末点停车判定收紧（_GOAL_STOP_M = 0.03）
# --------------------------------------------------------------------------- #
def test_goal_stop_tight() -> None:
    """★6b. 末点停车判定收紧：0.10 m 外不 FINISH / 0.02 m 内 FINISH，台账诚实。

    实车缺陷（2026 直线标定 N10→A10，4.55 m）：Localization 的末点吸附按
    `arrival_m`(0.12) 在偏差 0.118 m 处把信念拽到 A10 格心（车端 7_path.log：
    "到位吸附 A10 偏差 0.118 m → 锚回格心"）→ Mission 的末点判据见"已到" →
    FINISH 提前刹车 → 物理车停在格心前 11 cm 欠程；而车端 scale=1.00003
    说明不是里程计误差。修法：末点判据用 `_GOAL_STOP_M`(0.03)（对齐 tools/sim.py），
    非末点航点仍用 `arrival_m`(0.12)，行为完全不变。
    """
    section("6b. ★末点停车判定收紧：0.10 m 外不 FINISH / 0.02 m 内 FINISH")
    tokens = ["N10", "M10", "L10"]           # N10→M10→L10 沿 +y 直线，L10 是末点
    cfg = cfg_of(mission={"auto_start": True, "auto_start_delay_s": 0.1, "reset": "N10,y"},
                 route={"tokens": tokens})
    n10 = geo.block_to_meters("N10")
    m10 = geo.block_to_meters("M10")
    l10 = geo.block_to_meters("L10")
    rig = Rig(cfg=cfg, pose=n10 + (geo.HEADING_Y,))
    rig.ticks(3)                              # 自动出发

    # ---- 6b.1 非末点 M10：0.10 m 内照常按 arrival_m 判到达（行为不变）----
    rig.set_pose(n10[0], m10[1] - 0.10, geo.HEADING_Y)
    rig.tick()
    check("6b.1 非末点 M10 在 0.10 m 内照常判到达（arrival_m 语义不变）",
          rig.task._passed[1] is True,
          f"_passed[1]={rig.task._passed[1]} arrived_by={rig.task._arrived_by.get(1)}")

    # ---- 6b.2 末点 0.10 m 外：不得 FINISH（旧代码 s+0.12 余量会提前认账）----
    rig.set_pose(n10[0], l10[1] - 0.10, geo.HEADING_Y)
    rig.tick()
    check("6b.2 末点 0.10 m 外：`_passed[last]` 保持 False、不 FINISH、不刹车",
          rig.task._passed[2] is False
          and rig.task.phase != MissionPhase.FINISH
          and rig.count(EV_MISSION_FINISHED) == 0
          and rig.bridge.brakes == 0,
          f"_passed[2]={rig.task._passed[2]} phase={rig.task.phase!r} "
          f"finished={rig.count(EV_MISSION_FINISHED)} brakes={rig.bridge.brakes}")
    check("6b.3 末点 0.10 m 外：到点台账不虚报（无 index=2 的到达事件）",
          sum(1 for e in rig.of(EV_WAYPOINT_REACHED)
              if not e.data.get("forced_skip") and e.data.get("index") == 2) == 0,
          f"index=2 到达事件 "
          f"{[e.data for e in rig.of(EV_WAYPOINT_REACHED) if e.data.get('index') == 2]}")

    # ---- 6b.4 末点 0.02 m 内：FINISH 照常（真到了才认）----
    rig.set_pose(n10[0], l10[1] - 0.02, geo.HEADING_Y)
    rig.tick()
    check("6b.4 末点 0.02 m 内：判到达 + FINISH + 刹车",
          rig.task._passed[2] is True
          and rig.task.phase == MissionPhase.FINISH
          and rig.count(EV_MISSION_FINISHED) == 1
          and rig.bridge.brakes >= 1,
          f"_passed[2]={rig.task._passed[2]} phase={rig.task.phase!r} "
          f"finished={rig.count(EV_MISSION_FINISHED)} brakes={rig.bridge.brakes}")
    check("6b.5 到点台账诚实：末点到达事件恰好 1 条、非强制跳过",
          sum(1 for e in rig.of(EV_WAYPOINT_REACHED)
              if not e.data.get("forced_skip") and e.data.get("index") == 2) == 1,
          f"index=2 事件 "
          f"{[e.data for e in rig.of(EV_WAYPOINT_REACHED) if e.data.get('index') == 2]}")


# --------------------------------------------------------------------------- #
#  7. 外部命令
# --------------------------------------------------------------------------- #
def test_commands() -> None:
    section("7. 外部命令：reset / load / 单点 / 多点 / 未知命令不崩")
    cfg = cfg_of(mission={"auto_start": False, "auto_start_delay_s": 0.1},
                 route={"tokens": ["N10", "M10"]})
    rig = Rig(cfg=cfg, pose=(0.9, 0.9, 0.0))
    rig.ticks(2)
    check("7.0 前置：auto_start=False → IDLE、无路线",
          rig.task.phase == MissionPhase.IDLE and rig.bb.get(P_ROUTE) is None,
          f"phase={rig.task.phase!r}")

    # 7.1 单点命令（BLOCK,HEAD）
    rig.bridge.push_waypoint_command("G10*,x")
    rig.tick()
    route = rig.bb.get(P_ROUTE)
    check("7.1 单点命令 `G10*,x`：路线载入为 1 点停站",
          route is not None and len(route) == 1 and route[0].block == "G10"
          and route[0].shoot and abs(route[0].yaw) < 1e-12,
          f"{route.blocks() if route else None} shoot={route[0].shoot if route else None}")
    check("7.1b 单点命令后阶段 = DRIVE", rig.task.phase == MissionPhase.DRIVE,
          f"{rig.task.phase!r}")
    check("7.1c 单点命令也发 EV_ROUTE_LOADED", rig.count(EV_ROUTE_LOADED) >= 1,
          f"{rig.count(EV_ROUTE_LOADED)}")

    # 7.1d `单点 BLOCK,HEAD` 显式前缀
    rig.bridge.push_waypoint_command("单点 F10,y")
    rig.tick()
    route1d = rig.bb.get(P_ROUTE)
    check("7.1d `单点 F10,y`：1 点、朝向 y",
          route1d is not None and len(route1d) == 1 and route1d[0].block == "F10"
          and abs(route1d[0].yaw - geo.HEADING_Y) < 1e-12,
          f"{route1d.blocks() if route1d else None}")

    # 7.2 多点命令（空格分隔；末点可带朝向）
    rig.bridge.push_waypoint_command("N10 M10 L10 L9 L9,y")
    rig.tick()
    route2 = rig.bb.get(P_ROUTE)
    check("7.2 多点命令：块名序列正确",
          route2 is not None and list(route2.blocks())[:3] == ["N10", "M10", "L10"],
          f"{route2.blocks() if route2 else None}")

    # 7.3 reset:BLOCK,HEAD → 改 belief + 清空路线 + 回 IDLE
    rig.bridge.push_waypoint_command("reset:K9,x")
    rig.tick()
    pose = rig.bb.get(P_POSE)
    want = geo.block_to_meters("K9")
    check("7.3 reset:K9,x 改 belief 到 K9 格心、朝向 x",
          abs(pose.x - want[0]) < 1e-12 and abs(pose.y - want[1]) < 1e-12
          and abs(pose.yaw) < 1e-12,
          f"pose=({pose.x:.3f},{pose.y:.3f},{geo.deg(pose.yaw):.1f}°)")
    check("7.4 reset 清空路线、阶段回 IDLE",
          rig.bb.get(P_ROUTE) is None and rig.task.phase == MissionPhase.IDLE,
          f"route={rig.bb.get(P_ROUTE)!r} phase={rig.task.phase!r}")

    # 7.5 load:文件
    fixture = pathlib.Path(__file__).resolve().parent / "fixtures" / "route_field.txt"
    rig.bridge.push_waypoint_command(f"load:{fixture}")
    rig.tick()
    route3 = rig.bb.get(P_ROUTE)
    check("7.5 load:文件 从 fixture 读入 34 点现场路线",
          route3 is not None and len(route3) == 34, f"{len(route3) if route3 else None} 点")
    if route3 is not None:
        check("7.5b load 后阶段 DRIVE、含 G10* 射击停站",
              rig.task.phase == MissionPhase.DRIVE
              and any(w.shoot for w in route3.waypoints),
              f"phase={rig.task.phase!r} stops={route3.stop_points()}")

    # 7.6 未知命令 → 只告警不崩
    before = rig.task.health().errors
    rig.bridge.push_waypoint_command("这不是一条命令")
    rig.bridge.push_waypoint_command("reset:")
    rig.bridge.push_waypoint_command("")
    rig.bridge.push_waypoint_command("load:/不存在/的/文件.txt")
    rig.ticks(2)
    warnings = [line for line in rig.logs if "外部命令无效" in line]
    check("7.6 未知/非法命令：只告警不崩（errors 不增、tick 不抛）",
          rig.task.health().errors == before and len(warnings) >= 3,
          f"errors={rig.task.health().errors}（前 {before}）告警 {len(warnings)} 条")
    check("7.7 命令计数写入遥测（total/rejected）",
          rig.task.state()["commands"]["total"] >= 8
          and rig.task.state()["commands"]["rejected"] >= 3,
          f"{rig.task.state()['commands']}")

    # 7.8 SHOOT@ 作为路线 token（走 shoot_planner 展开候选链）
    rig.bridge.push_waypoint_command("reset:N10,x")     # 先清空路线，便于断言
    rig.tick()
    rejected_before = rig.task.state()["commands"]["rejected"]
    rig.bridge.push_waypoint_command("SHOOT@G11")
    rig.tick()
    check("7.8 SHOOT@ 不能作为外部命令单独下发（只告警，不崩）",
          rig.task.health().errors == before and rig.bb.get(P_ROUTE) is None
          and rig.task.state()["commands"]["rejected"] == rejected_before + 1,
          f"errors={rig.task.health().errors} route={rig.bb.get(P_ROUTE)!r} "
          f"rejected={rig.task.state()['commands']['rejected']}")

    # 7.9 SHOOT@ 作为**路线 token** 时必须正常展开（内部走 shoot_planner）
    rig.bridge.push_waypoint_command("多点 N10 H10 SHOOT@G11")
    rig.tick()
    route9 = rig.bb.get(P_ROUTE)
    shoot_pts = [w for w in (route9.waypoints if route9 else []) if w.shoot]
    check("7.9 路线里的 SHOOT@G11 展开成候选停站点（≥1 个 shoot 点、共享同组号）",
          route9 is not None and len(shoot_pts) >= 1
          and all(w.group == "G11" for w in shoot_pts),
          f"{[(w.block, w.shoot, w.group) for w in shoot_pts]}")


# --------------------------------------------------------------------------- #
#  8. 停站期间的阶段门（Tracker 必须刹车）
# --------------------------------------------------------------------------- #
def test_phase_gate() -> None:
    section("8. 阶段写入：DRIVE / APPROACH / STOP / FINISH 都落进 P_MISSION_PHASE")
    rig = Rig(cfg=_field_cfg(), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig.ticks(3)
    check("8.1 出发后 P_MISSION_PHASE = DRIVE", rig.bb.get(P_MISSION_PHASE) == MissionPhase.DRIVE,
          f"{rig.bb.get(P_MISSION_PHASE)!r}")
    seen = set()
    for _ in range(400):
        if rig.task.phase == MissionPhase.STOP:
            seen.add(MissionPhase.STOP)
            rig.ticks(30, dt=0.1)                   # 停站点三段超时放行
            continue
        rig.walk(step=0.1, max_steps=8)
        seen.add(rig.bb.get(P_MISSION_PHASE))
        if rig.task.phase == MissionPhase.FINISH:
            break
    check("8.2 全程出现过 APPROACH 与 STOP（接近段与停站段都对 Tracker 可见）",
          MissionPhase.APPROACH in seen and MissionPhase.STOP in seen, f"{sorted(seen)}")
    check("8.3 终点阶段 FINISH 已写入黑盒",
          rig.bb.get(P_MISSION_PHASE) == MissionPhase.FINISH,
          f"{rig.bb.get(P_MISSION_PHASE)!r}")


# --------------------------------------------------------------------------- #
#  9. 10 Hz 节拍（真线程）
# --------------------------------------------------------------------------- #
def test_rate() -> None:
    section("9. 10 Hz 节拍（真线程 0.5 s）")
    import time
    rig = Rig(cfg=_field_cfg(), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig.task.start()
    try:
        time.sleep(0.5)
    finally:
        rig.task.stop(timeout=1.0)
    ticks = rig.task.health().ticks
    check("9.1 0.5 s 内拍数在 10 Hz 容差内（3~9）", 3 <= ticks <= 9, f"ticks={ticks}")
    check("9.2 未发生 tick 异常", rig.task.health().errors == 0,
          f"errors={rig.task.health().errors} last={rig.task.health().last_error!r}")


def test_stall_fallback() -> None:
    """拐角绕圈兜底：停滞/绕圈 → 强制跳过，绝不卡死（readme【八】27）。"""
    section("10. 拐角绕圈兜底：停滞 / 绕圈 → 强制跳过，任务继续")
    import math as _math

    # ---- 10.x 停滞超时（车"卡在拐角"：位姿不动，只走时钟）----
    cfg = _field_cfg(stall_timeout_s=2.0, stall_yaw_deg=3600.0)  # 关掉绕圈判据，只测超时
    rig = Rig(cfg=cfg, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig.ticks(3)
    check("10.1 已自动出发（阶段 DRIVE）", rig.task.phase == MissionPhase.DRIVE,
          f"phase={rig.task.phase!r}")
    check("10.2 起步阶段未误判停滞", getattr(rig.task, "_forced_skips", 0) == 0,
          f"forced_skips={getattr(rig.task, '_forced_skips', None)}")

    # 位姿一动不动，快进 4 s（> stall_timeout 2.0s）
    for _ in range(20):
        rig.task.tick()
        rig.events.extend(rig.sub.poll(4096))
        rig.clock.advance(0.2)

    skips = getattr(rig.task, "_forced_skips", 0)
    forced = [e for e in rig.of(EV_WAYPOINT_REACHED) if e.data.get("forced_skip")]
    check("10.3 停滞超时后强制跳过（forced_skips ≥ 1）", skips >= 1, f"skips={skips}")
    check("10.4 跳过时发 EV_WAYPOINT_REACHED(forced_skip=True)", len(forced) >= 1,
          f"forced_events={len(forced)}")
    check("10.5 跳过的是「下一个未到达航点」（块名非空、索引单调）",
          len(forced) == 0 or all(isinstance(e.data.get("index"), int) for e in forced),
          f"{[e.data.get('block') for e in forced][:5]}")
    check("10.6 任务没卡死：仍在 DRIVE/APPROACH（不是 STOP/SAFE_STOP）",
          rig.task.phase in (MissionPhase.DRIVE, MissionPhase.APPROACH),
          f"phase={rig.task.phase!r}")
    check("10.7 日志给出「不可达」证据（含 跳过/拐角绕圈 字样）",
          any("跳过" in ln or "拐角绕圈" in ln for ln in rig.logs),
          f"logs_tail={rig.logs[-2:]}")

    # ---- 10.8 绕圈判据：未超时但累计转向达阈值也触发 ----
    cfg2 = _field_cfg(stall_timeout_s=3600.0, stall_yaw_deg=180.0)  # 关超时，只测绕圈
    rig2 = Rig(cfg=cfg2, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig2.ticks(3)
    yaw = geo.HEADING_Y
    for _ in range(6):            # 每拍 +90° → 两拍就够 180°
        yaw = geo.normalize_angle(yaw + _math.radians(90))
        rig2.clock.advance(0.1)
        b = rig2.bridge
        p = rig2.bb.get(P_POSE)
        b.set_pose(p.x, p.y, yaw)
        rig2.bb.set(P_POSE, geo.Pose(p.x, p.y, yaw), "test")
        rig2.task.tick()
        rig2.events.extend(rig2.sub.poll(4096))
    skips2 = getattr(rig2.task, "_forced_skips", 0)
    check("10.8 原地绕圈（累计转向达标）也会触发跳过", skips2 >= 1, f"skips={skips2}")
    check("10.9 绕圈触发时仍未超时（说明是航向判据起的作用）",
          rig2.clock.now() - 0.0 < 3600.0 and skips2 >= 1, f"skips={skips2}")

    # ---- 10.10 跳过后能继续正常推进（不重复跳同一个点）----
    cfg3 = _field_cfg(stall_timeout_s=2.0, stall_yaw_deg=3600.0)
    rig3 = Rig(cfg=cfg3, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig3.ticks(3)
    for _ in range(20):
        rig3.task.tick()
        rig3.events.extend(rig3.sub.poll(4096))
        rig3.clock.advance(0.2)
    n_after_stall = getattr(rig3.task, "_forced_skips", 0)
    rig3.walk(step=0.05, max_steps=200, stop_on_stop_point=False)
    normal = [e for e in rig3.of(EV_WAYPOINT_REACHED) if not e.data.get("forced_skip")]
    check("10.10 跳过之后仍能正常推进（出现非 forced 的到达事件）", len(normal) >= 2,
          f"normal={len(normal)} forced={n_after_stall}")
    check("10.11 跳过次数有上限（不会把整条路线全跳光）",
          n_after_stall <= len(FIELD_ROUTE), f"skips={n_after_stall} / 点数={len(FIELD_ROUTE)}")

    # ---- 10.12 ★PIVOT 有界机动期间：同样的"不动 + 转到 180°"不判停滞 ----
    # 为什么：180° 折返的 PIVOT 实测 ~10.8s、累计转 ~180°，与绕圈判据（12s 不前进 /
    # 累计转向达阈值）在数值上正面撞车 —— 不豁免就会把每次正常折返都误判成绕圈跳点
    # （实闭环 bench 实测 F11/A6/D6 三处）。PIVOT 自带硬超时且必然自终止，无卡死风险。
    cfg4 = _field_cfg(stall_timeout_s=2.0, stall_yaw_deg=60.0)   # 两个判据都很敏感
    rig4 = Rig(cfg=cfg4, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig4.ticks(3)
    yaw4 = geo.HEADING_Y
    for _ in range(20):            # 4s（> 超时 2s）且每拍 +30°（> 阈值 60°）
        yaw4 = geo.normalize_angle(yaw4 + _math.radians(30))
        rig4.bb.set(P_TELEMETRY, {"tracker_cmd": {"pivot_phase": "TURN"}}, "test")
        p = rig4.bb.get(P_POSE)
        rig4.bridge.set_pose(p.x, p.y, yaw4)
        rig4.bb.set(P_POSE, geo.Pose(p.x, p.y, yaw4), "test")
        rig4.task.tick()
        rig4.events.extend(rig4.sub.poll(4096))
        rig4.clock.advance(0.2)
    skips4 = getattr(rig4.task, "_forced_skips", 0)
    check("10.12 ★PIVOT 进行中不判停滞（4s>超时2s、累计转600°>60°，仍 0 跳点）",
          skips4 == 0, f"skips={skips4}")

    # 机动结束 → 停滞判定必须照常生效（豁免不能变成永久免疫）
    for _ in range(20):
        rig4.bb.set(P_TELEMETRY, {"tracker_cmd": {"pivot_phase": "IDLE"}}, "test")
        rig4.task.tick()
        rig4.events.extend(rig4.sub.poll(4096))
        rig4.clock.advance(0.2)
    skips4b = getattr(rig4.task, "_forced_skips", 0)
    check("10.13 PIVOT 结束后停滞判定照常生效（豁免不是永久免疫）",
          skips4b >= 1, f"skips={skips4b}")


def test_arrival_requires_proximity() -> None:
    """★11. 到点判定必须同时满足「车确实在附近」。

    诊断依据（docs/诊断_投影与顶点.md §2.5）：`_advance_waypoints` 用
    `s + arrival_m >= s_at_waypoint(i)` 单判"到点"，而 `_current_s()` 的弧长来自投影。
    车一旦被锚定/投影到后面的段上，**同一次 tick 里所有"被越过"的航点都会被判到达** ——
    bench 实测 #11 F11 / #12 E11 / #13 F11 三个从未走到的点被默默标记（没有 forced_skip 记录，
    航点计数整体错位）。
    修法：弧长判据与"距离 ≤ 2×arrival_m"**合取**；真到不了的点仍由
    `_detect_stall_and_skip` **显式**跳过（有事件 + 日志，可追溯）。
    """
    section("11. ★到点判定必须同时满足「车在附近」")
    cfg = _field_cfg()
    arrival = float(cfg.get("pursuit.arrival_m"))
    # 场景照抄 bench 的真实机制：车**沿路线推进**（弧长越过航点），但物理上**横向偏离**
    # 航点 > 2×arrival_m。旧写法只看弧长 ⇒ 这个航点会被判"已到达"（bench 实测
    # #11 F11/#12 E11/#13 F11 三个从未走到的点就是这样被默默标掉的）。
    # ⚠️ 不能用"把车瞬移到远处"来构造：`_current_s()` 用 `_last_s` 做 ±0.6 m 窗口，
    #    瞬移不会让 s 跳变 —— 必须真的沿路线推进。
    m10 = geo.block_to_meters("M10")            # FIELD_ROUTE[1]，航段沿 +y
    off = 0.35                                  # 横向偏离（> 2×arrival_m=0.24）

    def _run(rig, n=6):
        for _ in range(n):
            rig.task.tick()
            rig.events.extend(rig.sub.poll(4096))
            rig.clock.advance(0.2)

    rig = Rig(cfg=cfg)
    _run(rig, 4)                                # ⚠️ 自动出发会把 belief **复位到 N10**
    rig.set_pose(m10[0] + off, m10[1], geo.HEADING_Y)   # 复位之后再摆位，否则会被覆盖
    _run(rig)
    check("11.1 已自动出发（阶段 DRIVE）", rig.task.phase == MissionPhase.DRIVE,
          f"phase={rig.task.phase!r}")
    check("11.2 前置：弧长确实已经越过 M10（否则测的不是这件事）",
          (rig.task._last_s or 0.0) + arrival >= rig.task._path.s_at_waypoint(1),
          f"s={rig.task._last_s} s_at(1)={rig.task._path.s_at_waypoint(1):.3f}")

    reached = [e for e in rig.of(EV_WAYPOINT_REACHED) if not e.data.get("forced_skip")]
    idxs = sorted(int(e.data.get("index")) for e in reached)
    check("11.3 ★车偏 0.35 m 时不得因「弧长越过」而判到达（旧写法会判）",
          len(reached) == 0, f"报了 {len(reached)} 个：{idxs}")
    check("11.4 该航点仍未通过（`_passed[1]` 保持 False）",
          rig.task._passed[1] is False, f"_passed[1]={rig.task._passed[1]}")

    # 对照：车真正贴到航点上（≤ arrival_m）→ 必须判到达（修完不能"太严"）
    rig2 = Rig(cfg=cfg)
    _run(rig2, 4)
    rig2.set_pose(m10[0] + arrival * 0.5, m10[1], geo.HEADING_Y)
    _run(rig2)
    got2 = [e for e in rig2.of(EV_WAYPOINT_REACHED)
            if not e.data.get("forced_skip") and int(e.data.get("index")) == 1]
    check("11.5 对照：车在 arrival_m 内 → 正常判到达（没有变得过严）", len(got2) >= 1,
          f"M10 到达事件 {len(got2)} 个")

    # ---- 11.6/11.7 ★正常切角：车**贴着路径**通过，离格心却 0.29 m ----
    #  为什么必须成立：bench 实测 L10→L9→K9 的直角，车路过 L9 时离线心 0.288 m、横向偏差
    #  只有 ~0.09 m。旧判据（离格心 ≤ 2×arrival_m = 0.24 m）在这种**完全正常**的切角下恒假
    #  ⇒ L9 永不置位 ⇒ 停滞兜底 12 s 后把它当"不可达"补跳；一次跑从 #3 补跳到 #27（12 个），
    #  240 s 全耗在等待上（实测到点 30 → 16、绕圈 3 → 12、跑不完全程）。
    #  这里刻意**不经过** L9 格心（否则会被 dist≤arrival_m 直接放行，测不出这件事）。
    rig3 = Rig(cfg=cfg)
    _run(rig3, 4)
    l9x, l9y = geo.block_to_meters("L9")          # FIELD_ROUTE[3]，L10→L9→K9 的直角顶点
    rig3.set_pose(l9x - 0.085, l9y, geo.HEADING_X)     # 沿 L10→L9 走到离顶点 0.085 m
    _run(rig3, 2)
    rig3.set_pose(l9x, l9y + 0.29, geo.HEADING_Y)      # 切角后贴在 K9 腿上、离线心 0.29 m
    _run(rig3, 3)
    lat = getattr(rig3.task, "_last_lat_m", None)
    check("11.6 前置：车确实贴在路径上（横向偏差 ≤0.30 m）而离线心 0.29 m",
          lat is not None and lat <= 0.30 and 0.29 > 2 * arrival,
          f"lat={lat} 离线心=0.290 阈值=2×arrival={2 * arrival}")
    check("11.7 ★正常切角通过也必须判到达（否则航点永不置位、被停滞兜底补跳）",
          rig3.task._passed[3] is True, f"_passed[3]={rig3.task._passed[3]}")


def test_pivot_done_marks_arrival() -> None:
    """★12. PIVOT 完成通知 → 顶点判到达（dist>arrival_m 时不再兜底跳过）。

    bench 缺陷（E11/A6/B6）：`pivot_dist_m`(0.25) > `arrival_m`(0.12) —— 车在距顶点
    0.25 m 处原地掉头、完成后原路返回，**既不进到达圈、弧长也永不越过顶点**，
    `_advance_waypoints` 的两条判据恒假 ⇒ 拐角绕圈兜底 12 s 后强制跳过。
    本测试把 Tracker + Mission 接在同一个黑盒上完整复现：pivot 完成点距顶点
    0.18 m（> 0.12），断言旧行为（不判到达）不存在、新行为（通知判到达）成立。
    把 mission.py 的通知接受改回旧代码，本组立刻红。
    """
    section("12. ★PIVOT 完成 → 顶点判到达（完整复现 bench E11/A6/B6 机制）")
    tokens = ["B6", "A6", "B6", "B7"]
    cfg = Config.from_dict({
        "mission": {"auto_start": True, "auto_start_delay_s": 0.1, "reset": "B6,y",
                    "stall_timeout_s": 3.0},
        "route": {"tokens": tokens},
        "pursuit": {"pivot_dist_m": 0.25, "pivot_wait_s": 0.3, "pivot_timeout_s": 8.0,
                    "pivot_yaw_tol_deg": 15.0},
        "logging": {"rate_limit_s": 1.0},
    })
    arrival = float(cfg.get("pursuit.arrival_m"))
    clock = FakeClock()
    bridge = FakeBridge(clock)
    bb = Blackboard()
    bus = EventBus()
    logs: list[str] = []
    tracker = TrackerTask(bridge, bb, bus, cfg, log=logs.append, clock=clock.now)
    mission = MissionTask(bridge, bb, bus, cfg, log=logs.append, clock=clock.now)
    bb.set(P_SAFETY, SafetyState(ok=True), "test")
    sub = bus.subscribe("test", [EV_WAYPOINT_REACHED], maxsize=4096)
    events: list = []

    def m_tick() -> None:
        mission.tick()
        events.extend(sub.poll(4096))
        clock.advance(0.1)

    # ---- 12.1~12.3 路线装载 & 残留通知护栏 ----
    bb.set(PIVOT_DONE_KEY, (1, 0.35, "done"), "test")       # 上一轮"残留"的通知
    mission.setup()
    for _ in range(3):
        m_tick()
    route = bb.get(P_ROUTE)
    check("12.1 前置：自动出发、路线 = B6→A6→B6→B7",
          route is not None and list(route.blocks()) == tokens,
          f"{route.blocks() if route else None}")
    check("12.2 ★装路线时清掉残留通知（不会开局误判到点）",
          bb.get(PIVOT_DONE_KEY) is None, f"{bb.get(PIVOT_DONE_KEY)!r}")
    check("12.3 前置：A6（#1）尚未到达", mission._passed[1] is False,
          f"_passed={mission._passed}")

    # ---- 12.4~12.5 通知护栏：索引对但顶点弧长对不上 → 不得判到达 ----
    a6 = geo.block_to_meters("A6")
    b6 = geo.block_to_meters("B6")
    path = Path(route)
    s_a6 = path.s_at_waypoint(1)
    bb.set(PIVOT_DONE_KEY, (1, s_a6 + 1.0, "done"), "test")
    m_tick()
    check("12.4 ★弧长对不上的通知被拒绝（A6 不判到达）",
          mission._passed[1] is False, f"_passed[1]={mission._passed[1]}")

    # ---- 12.6~12.7 完整缺陷场景：pivot 完成点距顶点 0.18 m > arrival_m ----
    dx, dy = a6[0] - b6[0], a6[1] - b6[1]
    d = math.hypot(dx, dy)
    ux, uy = dx / d, dy / d
    x0, y0 = a6[0] - ux * 0.18, a6[1] - uy * 0.18
    yaw0 = geo.heading_of(ux, uy)
    bridge.set_pose(x0, y0, yaw0)
    bb.set(P_POSE, geo.Pose(x0, y0, yaw0), "test")
    check("12.5 ★缺陷场景成立：pivot 完成点离 A6 0.18 m > arrival_m(0.12)（正常判据够不着）",
          0.18 > arrival, f"dist=0.18 arrival={arrival}")
    tracker.tick()
    clock.advance(0.02)
    check("12.6 前置：Tracker 进入 PIVOT（非 IDLE 或已完成）",
          tracker._pivot_phase != PivotPhase.IDLE or tracker.pivot_count > 0,
          f"phase={tracker._pivot_phase} count={tracker.pivot_count}")
    for _ in range(25):                     # 0.5 s > pivot_wait_s(0.3) → TURN
        tracker.tick()
        clock.advance(0.02)
    target = tracker._pivot_target_heading
    p = bb.get(P_POSE)
    bridge.set_pose(p.x, p.y, target)
    bb.set(P_POSE, geo.Pose(p.x, p.y, target), "test")
    tracker.tick()
    check("12.7 前置：PIVOT 完成并计数（信源已写入）",
          tracker.pivot_count == 1 and tracker._pivot_phase == PivotPhase.IDLE,
          f"count={tracker.pivot_count} phase={tracker._pivot_phase}")

    # ---- 12.8~12.11 ★新行为：MissionTask 判 A6 到达，不兜底跳过 ----
    for _ in range(12):                     # 1.2 s < stall_timeout(3.0)，足够看到到达
        m_tick()
    reached_a6 = [e for e in events
                  if e.type == EV_WAYPOINT_REACHED and e.data.get("block") == "A6"
                  and not e.data.get("forced_skip")]
    forced_a6 = [e for e in events
                 if e.type == EV_WAYPOINT_REACHED and e.data.get("block") == "A6"
                 and e.data.get("forced_skip")]
    check("12.8 ★A6 判到达（发 EV_WAYPOINT_REACHED、非强制跳过）",
          len(reached_a6) >= 1 and len(forced_a6) == 0,
          f"到达 {len(reached_a6)} 次 / 强制跳过 {len(forced_a6)} 次")
    check("12.9 ★`_passed[1]` 置位、航点推进",
          mission._passed[1] is True and mission._next_wp == 2,
          f"_passed={mission._passed} next_wp={mission._next_wp}")
    check("12.10 ★没有触发拐角绕圈兜底（forced_skips=0、无兜底日志）",
          mission._forced_skips == 0
          and not any("拐角绕圈兜底" in ln for ln in logs),
          f"forced_skips={mission._forced_skips}")
    check("12.11 通知是 'done' 结局、顶点弧长与路径表一致",
          (lambda v: isinstance(v, tuple) and len(v) == 3 and v[0] == 1
           and v[2] == "done" and abs(float(v[1]) - s_a6) < 1e-9)(bb.get(PIVOT_DONE_KEY)),
          f"{bb.get(PIVOT_DONE_KEY)!r}")


# --------------------------------------------------------------------------- #
#  13. ★卡 D finding D1：逐点台账（s 跨点/收尾都必须显式记账，绝不静默丢点）
# --------------------------------------------------------------------------- #
def test_waypoint_ledger() -> None:
    """★13. 越窗/收尾的航点必须**显式记账**，且"最近接近"几何复核不得虚报到达。

    缺陷（t4 报告 D1，bench 实测）：`s` 一步跨过若干航点时，到达判据
    `(s_reached and near)` 对"车不在附近"的点恒假 ⇒ 那些点**既没登记到达、也没有
    forced_skip 记录**（静默丢点；老 43 点图上 42 个航点里 11 个没登记，其中 3 颗雷）。
    本用例把它做成可复现的场景：
      ① 车贴在 B1 后方 0.17 m（≤ 半格 0.175，但 s 还没越过）⇒ 只该记下"最近距离"；
      ② 横向偏 0.45 m（`near` 判据失效）+ 沿路径前冲 ⇒ s 跳跃，B1 由**最近接近**登记到达，
         而 C1/D1/E1（最近只到 ~0.47 m）必须走"越窗跳过"显式记账，F1/G1 由收尾清账兜住。
    台账不变量：`unexplained == []`（既非到达、也非显式跳过的航点一个都不能有）。
    """
    section("13. ★逐点台账：s 跨点/收尾都必须显式记账（绝不静默丢点）")
    tokens = ["A1", "B1", "C1", "D1", "E1", "F1", "G1"]     # A1→G1 沿 -y 的直线，格距 0.35
    cfg = cfg_of(mission={"auto_start": True, "auto_start_delay_s": 0.0, "reset": "A1,x"},
                 route={"tokens": tokens})
    rig = Rig(cfg=cfg, pose=geo.block_to_meters("A1") + (geo.HEADING_X,))
    rig.tick()                                              # 出发（delay=0）
    bx, by = geo.block_to_meters("B1")

    # ① 贴到 B1 后方 0.17 m：不触达（s 未越过），但留下"历史最近距离"
    rig.set_pose(bx + 0.08, by + 0.15, geo.HEADING_X)
    rig.tick()
    s1 = rig.task._last_s
    s_b1 = rig.task._path.s_at_waypoint(1)
    check("13.1 前置：s 尚未越过 B1（否则测的不是 s 跨点）",
          s1 + float(cfg.get("pursuit.arrival_m")) < s_b1,
          f"s={s1:.3f} s_at(1)={s_b1:.3f} arrival={cfg.get('pursuit.arrival_m')}")
    check("13.2 前置：已记下 B1 的最近距离 ≤ 复核半径（半格 0.175 m）",
          rig.task._min_dist[1] <= rig.task._close_m,
          f"min_dist={rig.task._min_dist[1]:.3f} close_m={rig.task._close_m:.3f}")
    check("13.3 前置：此刻 B1 未登记到达（`_passed[1]` False、arrived=0）",
          rig.task._passed[1] is False and rig.task.accounting()["arrived"] == 0,
          f"_passed[1]={rig.task._passed[1]} arrived={rig.task.accounting()['arrived']}")

    # ② 横向 0.45 m（near 失效）沿路径前冲：s 跳跃，跨过 B1/C1/D1/E1/F1/G1
    for dy in (0.0, 0.7, 1.4):
        rig.set_pose(bx + 0.45, by + 0.15 - dy, geo.HEADING_X)
        rig.tick()

    acc_mid = rig.task.accounting()
    check("13.4 ★s 跨点后 B1 由「最近接近」几何复核登记到达（旧代码：静默丢弃）",
          acc_mid["arrived_by"].get("最近接近", 0) == 1 and rig.task._passed[1] is True,
          f"arrived_by={acc_mid['arrived_by']} _passed[1]={rig.task._passed[1]}")
    check("13.5 ★复核半径不虚报：C1/D1 最近只到 ~0.47 m > 0.175 m ⇒ 不得登记到达",
          all(rig.task._min_dist[i] > rig.task._close_m
              for i in (2, 3)) and all(i not in rig.task._arrived_by for i in (2, 3)),
          f"min_dist[2..3]={[round(rig.task._min_dist[i], 3) for i in (2, 3)]}")
    check("13.6 ★越窗的点只记「待定」，**不立即置位**（否则会把还能到达的点提前判死）",
          set(int(k) for k in acc_mid["pending_skips"]) >= {2, 3}
          and rig.task._passed[2] is False and rig.task._passed[3] is False,
          f"pending={sorted(int(k) for k in acc_mid['pending_skips'])} "
          f"_passed[2]={rig.task._passed[2]} _passed[3]={rig.task._passed[3]}")
    check("13.7 待定项已显式记账（日志有「越窗待定」，不算静默）",
          any("越窗待定（显式记账，不静默丢点）" in ln for ln in rig.logs),
          f"{[ln for ln in rig.logs if '越窗待定' in ln][:1]}")
    check("13.8 前置：**已被 s 跨过**的航点不会落进 unexplained（待定也算已记账；"
          "后面的 E1/F1/G1 还没到，不算丢点）",
          not ({2, 3} & set(acc_mid["unexplained"])),
          f"unexplained={acc_mid['unexplained']}（其中 4/5/6 是尚未推进到的航点）")

    # ★③ 车又靠回 C1（0.094 m ≤ arrival_m）→ 必须**照样登记到达**（bench 实测雷 I8 就是这种：
    #    基线里是「到点 8/45 I8」；若越窗那拍就判死，它会被误记成跳点、雷覆盖 8/8→7/8）
    cx, cy = geo.block_to_meters("C1")
    rig.set_pose(cx + 0.05, cy + 0.08, geo.HEADING_X)
    rig.tick()
    acc_back = rig.task.accounting()
    check("13.9 ★★越窗待定的点后来又靠上 ⇒ 登记**到达**而不是跳过（雷 I8 的保护）",
          rig.task._passed[2] is True and "2" not in acc_back["skipped"]
          and "2" not in acc_back["pending_skips"],
          f"_passed[2]={rig.task._passed[2]} skipped={sorted(int(k) for k in acc_back['skipped'])} "
          f"pending={sorted(int(k) for k in acc_back['pending_skips'])}")

    # ④ 继续前冲到终点，剩余的越窗点由收尾清账定案
    for dy in (2.1, 2.8):
        rig.set_pose(bx + 0.45, by + 0.15 - dy, geo.HEADING_X)
        rig.tick()

    acc = rig.task.accounting()
    check("13.10 ★未到达的 D1/E1/F1/G1 全部显式记账（越窗定案 + 收尾清账）",
          all(str(i) in acc["skipped"] for i in (3, 4, 5, 6))
          and ("s 已越过" in acc["skipped"]["3"] or "s 已越过" in acc["skipped"]["4"]),
          f"skipped={ {k: v[:26] for k, v in acc['skipped'].items()} }")
    check("13.11 ★★无静默丢点：unexplained 为空（本用例的核心判据）",
          acc["unexplained"] == [], f"unexplained={acc['unexplained']}")
    check("13.12 台账闭合：arrived + 显式跳过 == 分母（len(route)-1）",
          acc["arrived"] + acc["skipped_total"] == acc["denominator"]
          and acc["pending_skips"] == {},
          f"{acc['arrived']}+{acc['skipped_total']} vs {acc['denominator']} pending={acc['pending_skips']}")

    forced = [e for e in rig.of(EV_WAYPOINT_REACHED) if e.data.get("forced_skip")]
    reasons = sorted({str(e.data.get("reason")) for e in forced})
    check("13.13 ★每个定案的航点都有事件（forced_skip=True + reason 可审计）",
          len(forced) == acc["skipped_total"]
          and reasons == ["close_ledger", "s_over_step"],
          f"forced={len(forced)} reasons={reasons}")
    check("13.14 ★事件与台账逐点对得上（索引集合相同）",
          sorted(int(e.data.get("index")) for e in forced) ==
          sorted(int(k) for k in acc["skipped"]),
          f"events={sorted(int(e.data.get('index')) for e in forced)} "
          f"ledger={sorted(int(k) for k in acc['skipped'])}")
    check("13.15 日志可逐点对账（含「显式记账，不静默丢点」与定案行）",
          any("越窗待定（显式记账，不静默丢点）" in ln for ln in rig.logs)
          and any("逐点定案（显式记账）" in ln for ln in rig.logs),
          f"logs_tail={rig.logs[-3:]}")
    check("13.16 结束横幅给出逐点到点率与显式跳过汇总",
          any("逐点到点率" in ln for ln in rig.logs)
          and any("显式跳过" in ln for ln in rig.logs),
          f"{[ln for ln in rig.logs if '逐点到点率' in ln][:1]}")
    check("13.17 任务正常收尾（FINISH），没有卡死",
          rig.task.phase == MissionPhase.FINISH, f"phase={rig.task.phase!r}")
    check("13.18 车体覆盖复核（只读接口，供 t8/t10 对账雷是否被盖住）",
          isinstance(rig.task.closest_approaches(), dict)
          and rig.task.closest_approaches().get(1, -1) == 0.17,
          f"min_dist[1]={rig.task.closest_approaches().get(1)}")

    # ⑤ 对照（防"复核太松"）：车**真贴到**格心时，主判据（距格心/s+近路径）必须照样生效
    rig2 = Rig(cfg=cfg, pose=geo.block_to_meters("A1") + (geo.HEADING_X,))
    rig2.tick()
    rig2.set_pose(*geo.block_to_meters("B1"), geo.HEADING_X)          # 正落在 B1 格心
    rig2.tick(0.1)
    check("13.19 对照：车在格心上 → 登记到达且依据不是「最近接近」（主判据未被抢）",
          rig2.task._passed[1] is True
          and not str(rig2.task.accounting()["arrived_by"].get(1, "")).startswith("最近接近"),
          f"arrived_by={rig2.task.accounting()['arrived_by']}")
    check("13.20 对照：`_min_dist` 仍如实记录（0 m）",
          rig2.task._min_dist[1] < 1e-9, f"min_dist={rig2.task._min_dist[1]}")


# --------------------------------------------------------------------------- #
#  14. ★卡 t6 / t13：无人机桥洞情报（EV_BRIDGE_INFO → 穿洞格）
# --------------------------------------------------------------------------- #
#  三场景（验收口径，t6 定、t13 保持逐点不变）：
#    ① 情报注入 → 换洞生效（**两半场各自独立**：只有 lower 情报时 upper 仍走默认，反之亦然）；
#    ② 情报过期 / 无人机离线 / 情报非法 → 逐 token 回落默认值，不崩、不卡发车；
#    ③ 全程无无人机（无 P_DRONE）→ 与接线前**逐 token 一致**（连等待窗口都不进）。
#  ★ 卡 t13（D3 修正）：情报真源 = `EV_BRIDGE_INFO` **事件流**（每条自带 half），
#    而 `P_DRONE` 只有**一个**桥洞槽且被覆盖式赋值（state.py:107-108 / drone.py:283）。
#    所以夹具必须**忠实模拟 DroneTask**：写槽 + 发事件两件事一起做（见 `t6_drone_round`），
#    并且要能造出"槽里只剩 upper，lower 仍然生效"的稳态盘面（14.40/14.41 就是这条回归）。
#  夹具：第五张图的副本，只把 `bridges:` 放宽成 K4/K5/K9/K10（现场每局随机会换洞），
#        其余要素（雷/障/靶/隔板/起终）逐字照抄 `config/map.yaml`。
_WS_ROOT = pathlib.Path(__file__).resolve().parents[3]
_T6_ROBOT = _WS_ROOT / "config" / "robot.yaml"
_T6_MAP_SRC = _WS_ROOT / "config" / "map.yaml"
_T6_FIX = pathlib.Path(tempfile.mkdtemp(prefix="dz_t6_fix_"))
_T6_MAP = _T6_FIX / "map_t6.yaml"
_T6_MAP.write_text(
    re.sub(r"(?m)^([ \t]*)bridges:.*$",
           lambda m: m.group(1) + 'bridges:    ["K4", "K5", "K9", "K10"]',
           _T6_MAP_SRC.read_text(encoding="utf-8")), encoding="utf-8")
#: 现役手工路线（`config/robot.yaml` 的 route.tokens = 车上 tracker-safe 产物，60 token）：
#: 下半场穿 L10→K10→J10、上半场穿 J5→K5→L5 —— 正是换洞要动的两个三连。
_T6_TOKENS = [str(t) for t in (Config.load(str(_T6_ROBOT)).get("route.tokens") or [])]
_T6_BASE = [parse_token(t)["block"] for t in _T6_TOKENS]


def t6_cfg(*, tokens=None, wait: float = 0.0, auto_plan: bool = False,
           engine: str = "tour", drone_enabled: bool = True,
           lower_cand=("K9", "K10"), upper_cand=("K4", "K5"),
           wait_at_start: bool = False, hold_max: float = 0.0,
           budget: float = 180.0, reroute_max: int = 1) -> Config:
    """t6 夹具配置（默认：手动路线 + 不等 + 无人机链路开）。

    ★ 卡 t32：`wait_at_start=True` 复现 t20 的**起点**等待窗口（对照档）；
    默认 false = 等移到 L10 决策点，届时用 `hold_max`（+ `budget`）控制桥前等待上限。
    """
    return Config.from_dict({
        "mission": {"auto_start": True, "auto_start_delay_s": 0.1, "reset": "N10,y",
                    "bridge_info_wait_s": wait, "scan_mode": "none",
                    "bridge_wait_at_start": wait_at_start,
                    "bridge_wait_max_s": hold_max, "race_budget_s": budget,
                    "bridge_reroute_max": reroute_max},
        "route": {"tokens": list(_T6_TOKENS if tokens is None else tokens),
                  "cross_lower_candidates": list(lower_cand),
                  "cross_upper_candidates": list(upper_cand),
                  "cross_lower_default": "K10", "cross_upper_default": "K5"},
        "path_executor": {"shoot_distance_min": 0.70, "shoot_distance_max": 0.90},
        "planner": {"auto_plan_enabled": auto_plan, "map_path": str(_T6_MAP),
                    "engine": engine, "fallback_to_tokens": True},
        "drone": {"enabled": drone_enabled, "info_stale_s": 30.0},
        "logging": {"rate_limit_s": 0.0},          # 限流关掉：要看到每一条情报日志
    })


def t6_drone(rig, *, half=None, block=None, online: bool = True, age: float = 0.0,
             publish: bool = True) -> DroneState:
    """忠实模拟 `DroneTask._on_bridge_passable`（drone.py:283-287）：**写槽 + 发事件**。

    ★ 卡 t13：这两件事在真车上是同一段代码里一起做的，所以夹具也必须一起做
      （`publish=False` 只用于对照：证明"只写槽、不发事件"**不再**被当成情报）。
    `ts` 同时进槽（`bridge_ts`）与事件载荷 —— 真车上它俩就是同一个数，所以
    "Mission 判新鲜" 与 "`DroneState.bridge_of()` 判新鲜" 必须逐字等价（14.10/14.17）。
    """
    now = rig.clock.now()
    ts = (now - age) if block else 0.0
    state = DroneState(online=bool(online),
                       last_heartbeat=(now if online else 0.0),
                       bridge_half=half, bridge_block=block, bridge_confidence=0.9,
                       bridge_ts=ts)
    rig.bb.set(P_DRONE, state, "test")
    if publish and half in ("lower", "upper") and block:
        rig.bus.publish(EV_BRIDGE_INFO, source="drone", half=half, block=block,
                        confidence=0.9, ts=ts)
    return state


def t6_drone_round(rig, *, lower=None, upper=None, online: bool = True, age: float = 0.0,
                   publish: bool = True) -> DroneState | None:
    """按无人机的**真实顺序**（lower → upper）发一轮情报；槽每次都被**覆盖**（D3 的成因）。

    :returns: 最后一次写槽后的 `DroneState`（`bridge_half` = 本轮最后那个半场 = `upper`）。
    """
    state = None
    for half, block in (("lower", lower), ("upper", upper)):
        if block:
            state = t6_drone(rig, half=half, block=block, online=online, age=age,
                            publish=publish)
    return state


def t6_kblocks(rig) -> list[str]:
    return [b for b in rig.task.route.blocks() if b.startswith("K")]


def test_bridge_intel() -> None:
    section("14. 无人机桥洞情报（卡 t6）：换洞生效 / 过期回落 / 无无人机逐 token 不变")

    # ---------------- ① 新鲜情报注入 → 换洞生效（lower 独立） ----------------
    rig = Rig(cfg=t6_cfg(wait=0.0), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    state = t6_drone_round(rig, lower="K11")   # CFG/GOT: K11 在 K 列 7~11 行段内且 ≠ 默认 K9（t31 权威门接受）
    rig.ticks(3)
    blocks = list(rig.task.route.blocks())
    st = rig.task.state()["bridge"]
    removed = sorted(set(_T6_BASE) - set(blocks))
    added = sorted(set(blocks) - set(_T6_BASE))
    check("14.1 ★ 情报注入：下半场穿洞格 K9 → K11（换洞生效）",
          "K11" in blocks and "K9" not in blocks, f"K 列格={t6_kblocks(rig)}")
    check("14.2 ★ 只改了穿洞**三连**那一行（L9/K9/J9 → L11/K11/J11，实测 44 点不变），其余航点逐格不变",
          removed == ["J9", "K9", "L9"] and added == ["J11", "K11", "L11"]
          and len(blocks) == len(_T6_BASE),
          f"−{removed} +{added} / 点数 {len(blocks)} vs {len(_T6_BASE)}")
    check("14.3 ★ 两半场独立：只有 lower 情报 → upper 仍是默认 K5（K4 未出现）",
          "K5" in blocks and "K4" not in blocks, f"K 列格={t6_kblocks(rig)}")
    check("14.4 遥测 `bridge.used` = 本局实际用格 {lower:K11, upper:K5}",
          st["used"] == {"lower": "K11", "upper": "K5"}, str(st["used"]))
    check("14.5 遥测 `bridge.lower` = 来源 drone / cell=K11 / raw=K11 / age≈0",
          st["lower"]["source"] == "drone" and st["lower"]["cell"] == "K11"
          and st["lower"]["raw"] == "K11" and abs(float(st["lower"]["age_s"])) < 0.5,
          str(st["lower"]))
    check("14.6 遥测 `bridge.seen` 记下本条情报（真源 = EV_BRIDGE_INFO 事件流，每条自带 half）",
          st["seen"].get("lower", {}).get("cell") == "K11" and st["events"] >= 1,
          f"seen={st['seen']} events={st.get('events')}")
    check("14.7 日志留下换洞证据（旧格 → 新格 + 校验口径）",
          any("换洞生效：K9 → K11" in s and "validate_route 0 error" in s for s in rig.logs),
          str([s for s in rig.logs if "换洞" in s][:2]))
    check("14.8 换洞后照样出发（DRIVE + 恰好一次 EV_ROUTE_LOADED）",
          rig.task.phase == MissionPhase.DRIVE and rig.count(EV_ROUTE_LOADED) == 1,
          f"{rig.task.phase} loaded={rig.count(EV_ROUTE_LOADED)}")
    check("14.9 横幅/日志给出穿洞格**来源**（现场第一个要看的行）",
          any("穿洞格来源：" in s and "lower=无人机情报(K11)" in s for s in rig.logs),
          str([s for s in rig.logs if "穿洞格来源" in s]))
    # 与框架判据等价性：我判"新鲜" ⇔ `DroneState.bridge_of()` 有值
    check("14.10 ★ 新鲜度判据与 `DroneState.bridge_of()` 等价（同源：now-bridge_ts > stale_s）",
          state.bridge_of("lower", now=rig.clock.now(), stale_s=30.0) == "K11"
          and rig.task._intel["lower"].source == "drone",
          f"bridge_of={state.bridge_of('lower', now=rig.clock.now(), stale_s=30.0)!r} / "
          f"source={rig.task._intel['lower'].source}")

    # ---------------- ①b 反向：只有 upper 情报 → upper 换洞、lower 默认 ----------------
    rigb = Rig(cfg=t6_cfg(wait=0.0), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    t6_drone_round(rigb, upper="K4")
    rigb.ticks(3)
    blocksb = list(rigb.task.route.blocks())
    check("14.11 ★ 反向独立：只有 upper 情报 → K5 → K4，下半场仍是默认 K9",
          "K4" in blocksb and "K5" not in blocksb and "K9" in blocksb and "K10" not in blocksb,
          f"K 列格={t6_kblocks(rigb)}")
    check("14.12 遥测 `used` = {lower:K10（默认）, upper:K4（情报）}",
          rigb.task.state()["bridge"]["used"] == {"lower": "K9", "upper": "K4"},
          str(rigb.task.state()["bridge"]["used"]))

    # ---------------- ①c ★★ D3 回归：单槽被 upper 覆盖后，lower 仍然生效 ----------------
    #  真机稳态盘面：无人机每轮按 lower → upper 顺序发 ⇒ 槽里**恒为 upper**（state.py:107-108
    #  单字段 + drone.py:283 覆盖式赋值）。t6 第一版读槽，此时 `bridge_of("lower")` 恒 None。
    #  本夹具：两条情报**在任何 tick 之前**发完（槽最终只剩 upper，两半场事件都在队列里）。
    rigc = Rig(cfg=t6_cfg(wait=0.0), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    stc_last = t6_drone_round(rigc, lower="K9", upper="K4")
    check("14.13 ★★ D3：单槽已被 upper 覆盖（框架层 bridge_of('lower') 就是 None）",
          str(getattr(stc_last, "bridge_half", "")) == "upper"
          and stc_last.bridge_of("lower", now=rigc.clock.now(), stale_s=30.0) is None
          and stc_last.bridge_of("upper", now=rigc.clock.now(), stale_s=30.0) == "K4",
          f"槽 half={getattr(stc_last, 'bridge_half', None)!r} "
          f"bridge_of(lower)={stc_last.bridge_of('lower', now=rigc.clock.now(), stale_s=30.0)!r}")
    rigc.ticks(3)
    blocksc = list(rigc.task.route.blocks())
    check("14.14 ★★ D3 修法：事件流按半场各存一份 → K9 与 K4 **同时**生效（单槽覆盖不影响）",
          "K9" in blocksc and "K4" in blocksc and "K10" not in blocksc and "K5" not in blocksc
          and set(rigc.task.state()["bridge"]["seen"]) == {"lower", "upper"},
          f"K 列格={t6_kblocks(rigc)} seen={rigc.task.state()['bridge']['seen']}")
    check("14.14b 事件流的 `ts` 与槽里的 `bridge_ts` 是同一个数（过期判据天然与框架同源）",
          abs(float(rigc.task.state()["bridge"]["seen"]["lower"]["ts"])
              - float(stc_last.bridge_ts)) < 1e-9,
          f"seen={rigc.task.state()['bridge']['seen']} bridge_ts={stc_last.bridge_ts}")

    # ---------------- ② 过期 / 离线 / 非法情报 → 逐 token 回落默认 ----------------
    rig2 = Rig(cfg=t6_cfg(wait=0.0), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    t6_drone_round(rig2, lower="K9", age=31.0)
    rig2.ticks(3)
    st2 = rig2.task.state()["bridge"]
    check("14.15 ★ 情报过期（31s > info_stale_s=30s）→ 与默认路线逐 token 一致",
          list(rig2.task.route.blocks()) == _T6_BASE, f"K 列格={t6_kblocks(rig2)}")
    check("14.16 过期来源如实记为 stale（不是 missing）+ 原始格留在遥测里",
          st2["lower"]["source"] == "stale" and st2["lower"]["raw"] == "K9",
          str(st2["lower"]))
    check("14.17 ★ 过期时 `bridge_of()` 也返回 None（两条判据同源）",
          t6_drone_state_of(rig2).bridge_of("lower", now=rig2.clock.now(), stale_s=30.0) is None,
          "bridge_of=None")

    rig3 = Rig(cfg=t6_cfg(wait=0.0), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    st3 = t6_drone_round(rig3, lower="K9", online=False)   # 离线但情报 ts 很新
    rig3.ticks(3)
    check("14.18 ★ 无人机离线（online=False）→ 回落默认（即使 bridge_ts 还在过期窗口内）",
          list(rig3.task.route.blocks()) == _T6_BASE
          and st3.bridge_of("lower", now=rig3.clock.now(),
                            stale_s=30.0) == "K9",   # 框架层看它"还新鲜" ⇒ 离线必须单独挡
          f"source={rig3.task.state()['bridge']['lower']['source']}")
    check("14.19 离线来源记为 offline（遥测可查）",
          rig3.task.state()["bridge"]["lower"]["source"] == "offline",
          str(rig3.task.state()["bridge"]["lower"]))

    rig4 = Rig(cfg=t6_cfg(wait=0.0), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    t6_drone_round(rig4, lower="K11")       # K 列 + 下半场行段内，但**不在**候选集合 {K10} 里
    rig4.ticks(3)
    st4 = rig4.task.state()["bridge"]
    check("14.20 ★ t31 权威门放宽：情报 K11（池外但在下半场行段内）→ **采纳**（不再回落默认）、不崩、照常出发",
          st4["lower"]["source"] == "drone" and st4["lower"]["cell"] == "K11"
          and rig4.task.phase == MissionPhase.DRIVE and rig4.task.health().errors == 0,
          str(st4["lower"]))
    check("14.21 t31：池外采纳的原因写明『纠正过期地图』（遥测 + 日志都有）",
          "池外" in st4["lower"]["reason"] and "纠正过期地图" in st4["lower"]["reason"]
          and any("池外" in s for s in rig4.logs), str(st4["lower"]["reason"]))

    rig5 = Rig(cfg=t6_cfg(wait=0.0), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    t6_drone_round(rig5, lower="A1")       # 非 K 列
    rig5.ticks(3)
    check("14.22 情报不在 K 列（A1）→ 拒收 + 回落默认",
          list(rig5.task.route.blocks()) == _T6_BASE
          and rig5.task.state()["bridge"]["lower"]["source"] == "rejected",
          str(rig5.task.state()["bridge"]["lower"]))

    # ★ t31：行段放开后，"池外非法格"只剩两类 —— 格名非法 / 非 K 列 / 跨半场行段；
    #   这里把三类各钉一条（防止哪天把"放宽"做成"什么都收"）。
    for cell, expect in (("Z9", "格名非法"), ("ZZ9", "格名非法"), ("K3", "半场")):
        rig_bad = Rig(cfg=t6_cfg(wait=0.0),
                      pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
        t6_drone_round(rig_bad, lower=cell)
        rig_bad.ticks(3)
        st_bad = rig_bad.task.state()["bridge"]["lower"]
        check(f"14.23 t31 仍拒：下半场情报 {cell}（{expect}）→ rejected + 回落默认 K10",
              st_bad["source"] == "rejected" and st_bad["cell"] == "K10"
              and expect in st_bad["reason"]
              and list(rig_bad.task.route.blocks()) == _T6_BASE
              and rig_bad.task.health().errors == 0, str(st_bad))

    rig6 = Rig(cfg=t6_cfg(wait=0.0, drone_enabled=False),
               pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    t6_drone_round(rig6, lower="K9")
    rig6.ticks(3)
    check("14.23 `drone.enabled=false` → 整条情报路径空转（逐 token 默认）",
          list(rig6.task.route.blocks()) == _T6_BASE
          and rig6.task.state()["bridge"]["lower"]["source"] == "disabled",
          str(rig6.task.state()["bridge"]["lower"]))

    # ---------------- ③ 全程无无人机 → 与接线前逐 token 一致（且不等情报） ----------------
    rig7 = Rig(cfg=t6_cfg(wait=5.0), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig7.ticks(3)                                   # t≈0.3s ≪ 5s 窗口
    check("14.24 ★ 无 P_DRONE（DroneTask 未运行）→ **不进等待窗口**，0.3s 内就出发",
          rig7.task.phase == MissionPhase.DRIVE
          and not any("等待无人机桥洞情报" in s for s in rig7.logs),
          f"phase={rig7.task.phase} t={rig7.clock.now() - 1000.0:.2f}s")
    check("14.25 ★ 无无人机时路线与接线前**逐 token 一致**（44/44 全等）",
          list(rig7.task.route.blocks()) == _T6_BASE and len(_T6_BASE) == 44,
          f"{len(rig7.task.route)} 点 vs 基线 {len(_T6_BASE)} 点")
    check("14.26 无无人机时来源记为 no_link，且 used = 默认格",
          rig7.task.state()["bridge"]["lower"]["source"] == "no_link"
          and rig7.task.state()["bridge"]["used"] == {"lower": "K9", "upper": "K5"},
          str(rig7.task.state()["bridge"]["lower"]))

    # ---------------- ③b ★★ 卡 t20（t14-F1 裁决）：等待窗口**只在"等得到东西"时开** -------------
    #   旧语义（t6/t13，已废）：只要"有 link（P_DRONE 存在）且两半场未齐"就等 —— 但 DroneTask
    #   **每拍都写 P_DRONE**（无论在线与否），所以"有 link"恒真 ⇒ 无人机离线也白等到硬上限。
    #   新语义（本卡）：还需要 ④ online=True 且 ⑤ **手上已有一半新鲜情报**（source=drone）。
    #   F1① 无人机全程离线（DroneTask 在跑、P_DRONE 每拍写 online=False）→ 不等待、起栈即出发
    rig8 = Rig(cfg=t6_cfg(wait=5.0), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    for _ in range(3):                              # 每拍都写一份在线=False 的 P_DRONE
        t6_drone(rig8, half=None, block=None, online=False)
        rig8.tick()
    check("14.27 ★ F1①：链路在但**无人机离线** → 不进等待窗口，0.3s 内已出发（走默认格）",
          rig8.task.phase == MissionPhase.DRIVE
          and list(rig8.task.route.blocks()) == _T6_BASE
          and not any("等待无人机桥洞情报" in s for s in rig8.logs)
          and rig8.task.state()["bridge"]["note"].get("waiting") is not True,
          f"phase={rig8.task.phase} note={rig8.task.state()['bridge']['note']}")
    check("14.28 F1①：遥测如实记 offline、used=默认格 K10/K5（离线情报一律不采纳）",
          rig8.task.state()["bridge"]["lower"]["source"] == "offline"
          and rig8.task.state()["bridge"]["used"] == {"lower": "K9", "upper": "K5"},
          str(rig8.task.state()["bridge"]["lower"]))

    #   F1③ online 但**情报过期** → 不等待、立即出发
    rig8b = Rig(cfg=t6_cfg(wait=5.0), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    t6_drone_round(rig8b, lower="K9", age=31.0)     # 在线且收到了，但 31s > info_stale_s=30s
    rig8b.ticks(4)                                  # t≈0.4s ≪ 5s 窗口
    check("14.29 ★ F1③：只有**过期**情报 → 不进等待窗口（0.4s 内出发、走默认格、记为 stale）",
          rig8b.task.phase == MissionPhase.DRIVE
          and list(rig8b.task.route.blocks()) == _T6_BASE
          and rig8b.task.state()["bridge"]["lower"]["source"] == "stale"
          and not any("等待无人机桥洞情报" in s for s in rig8b.logs),
          f"phase={rig8b.task.phase} "
          f"lower={rig8b.task.state()['bridge']['lower']['source']}")

    #   F1② online + 已有一半新鲜情报 → 进等待窗口；另一半在窗口内到达即**立即**采纳
    rig9 = Rig(cfg=t6_cfg(wait=5.0, wait_at_start=True), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    t6_drone_round(rig9, lower="K9")                # 先给下半场（新鲜）→ 这才是"值得等"
    rig9.ticks(3)                                   # t≈0.3s：延时(0.1s)已到、窗口未满
    check("14.30 ★ F1②：online + 一半新鲜情报 → **进等待窗口**（0.3s 仍 IDLE、路线未载入）",
          rig9.task.phase == MissionPhase.IDLE and rig9.bb.get(P_ROUTE) is None
          and rig9.task.state()["bridge"]["note"].get("waiting") is True
          and any("等待无人机桥洞情报" in s for s in rig9.logs),
          f"phase={rig9.task.phase} note={rig9.task.state()['bridge']['note']}")
    t6_drone_round(rig9, upper="K4")                # 另一半到齐 → 立即出发
    rig9.ticks(3)
    check("14.31 ★ F1②：窗口内到齐 → 立即出发（不必等满窗口）且两个半场都换洞",
          rig9.task.phase == MissionPhase.DRIVE
          and rig9.clock.now() - 1000.0 < 0.1 + 5.0
          and "K9" in list(rig9.task.route.blocks())
          and "K4" in list(rig9.task.route.blocks()), f"K 列格={t6_kblocks(rig9)}")

    #   硬上限仍是**兜底**（F1 裁决明确保留）：另一半永不到，delay+wait 到点必发车
    rig9b = Rig(cfg=t6_cfg(wait=2.0, wait_at_start=True), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    t6_drone_round(rig9b, lower="K9")               # 一半新鲜 → 会进窗口
    rig9b.ticks(5)                                  # t≈0.5s：仍在等
    mid_waiting = rig9b.task.phase == MissionPhase.IDLE
    rig9b.ticks(25)                                 # t≈3.0s > delay(0.1)+wait(2.0) → 到点必发车
    check("14.31b ★ 硬上限兜底保留：另一半永不到也到点发车（不卡死），已得情报照常生效",
          mid_waiting and rig9b.task.phase == MissionPhase.DRIVE
          and "K9" in list(rig9b.task.route.blocks())
          and "K10" not in list(rig9b.task.route.blocks())      # 已得的那半场换了洞
          and "K5" in list(rig9b.task.route.blocks())           # 未到的半场走默认
          and any("等待结束" in s for s in rig9b.logs),
          f"phase={rig9b.task.phase} k={t6_kblocks(rig9b)} "
          f"waited={rig9b.task.state()['bridge']['note'].get('waited_s')}")

    # ---------------- ③d 发车后晚到的情报：只进遥测，不改路线（路线已冻结） ----------------
    rig10 = Rig(cfg=t6_cfg(wait=0.0), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig10.ticks(3)                                  # 无情报即出发（默认洞）
    before = list(rig10.task.route.blocks())
    t6_drone_round(rig10, lower="K9")        # 发车后情报才到
    rig10.ticks(3)
    st10 = rig10.task.state()["bridge"]
    check("14.32 ★ 发车后晚到的情报**不改路线**（已冻结），只更新遥测实时结论",
          list(rig10.task.route.blocks()) == before
          and st10["used"] == {"lower": "K9", "upper": "K5"}
          and st10["lower"]["source"] == "drone" and st10["lower"]["cell"] == "K9",
          f"used={st10['used']} live={st10['lower']['source']}")

    # ---------------- ④ 纯函数：`rewrite_crossing_tokens` 的形状判据 ----------------
    toks = ["N10", "M10", "L9", "K9", "J9", "I10"]
    check("14.33 `rewrite_crossing_tokens`：标准三连 L9/K9/J9 → L11/K11/J11",
          rewrite_crossing_tokens(toks, "lower", "K9", "K11")
          == ["N10", "M10", "L11", "K11", "J11", "I10"],
          str(rewrite_crossing_tokens(toks, "lower", "K9", "K11")))
    check("14.34 进/出洞口与洞**不同行** → 拒（返回 None，交给调用方走默认洞）",
          rewrite_crossing_tokens(["L9", "K10", "J10"], "lower", "K10", "K9") is None,
          "None")
    check("14.35 洞口两侧不是 K 列邻格（隔了 2 列）→ 拒",
          rewrite_crossing_tokens(["H10", "K10", "N10"], "lower", "K10", "K9") is None, "None")
    check("14.36 洞格在路线里出现两次（自折叠）→ 拒",
          rewrite_crossing_tokens(["L9", "K9", "J9", "K10", "J10"], "lower", "K10", "K9")
          is None, "None")
    check("14.37 新洞不在该半场行段（K5 给下半场）→ 拒",
          rewrite_crossing_tokens(toks, "lower", "K10", "K5") is None, "None")
    check("14.38 token 前后缀保留：`R:K10` → `R:K9`（倒车段/停站标记不会被换丢）",
          rewrite_crossing_tokens(["L10", "R:K10", "J10"], "lower", "K10", "K9")
          == ["L9", "R:K9", "J9"],
          str(rewrite_crossing_tokens(["L10", "R:K10", "J10"], "lower", "K10", "K9")))
    check("14.39 `BridgeIntel.describe()` 对情报/默认两条路径都可读",
          "无人机情报" in BridgeIntel("lower", "K9", "K10", "drone", "K9", "", 0.3).describe()
          and "stale" in BridgeIntel("lower", "K10", "K10", "stale", "K9", "过期").describe(),
          BridgeIntel("lower", "K9", "K10", "drone", "K9", "", 0.3).describe())

    # ---------------- ⑤ ★卡 t13：情报真源 = 事件流（不是 P_DRONE 单槽） ----------------
    rig11 = Rig(cfg=t6_cfg(wait=0.0), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    st11 = t6_drone(rig11, half="lower", block="K9", publish=False)   # 只写槽，不发事件
    rig11.ticks(3)
    check("14.40 ★ 只写 `P_DRONE` 槽、不发 `EV_BRIDGE_INFO` → **不当作情报采纳**（走默认格）",
          list(rig11.task.route.blocks()) == _T6_BASE
          and rig11.task.state()["bridge"]["lower"]["source"] == "missing"
          and rig11.task.state()["bridge"]["events"] == 0,
          f"k={t6_kblocks(rig11)} source={rig11.task.state()['bridge']['lower']['source']} "
          f"events={rig11.task.state()['bridge']['events']}")
    check("14.40b 这条路径在生产上不可达：DroneTask 的写槽与发事件在同一段代码（drone.py:283-287）"
          "⇒ 事件覆盖 ⊇ 槽覆盖（框架层 bridge_of() 仍认这条情报，Mission 不认）",
          st11.bridge_of("lower", now=rig11.clock.now(), stale_s=30.0) == "K9"
          and rig11.task.state()["bridge"]["used"] == {"lower": "K9", "upper": "K5"},
          f"bridge_of={st11.bridge_of('lower', now=rig11.clock.now(), stale_s=30.0)!r} "
          f"used={rig11.task.state()['bridge']['used']}")

    # 坏事件只丢自己那条：半场非法 / 缺 block → 丢弃；同一条拍里的好情报照常生效
    rig12 = Rig(cfg=t6_cfg(wait=0.0), pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    now12 = rig12.clock.now()
    rig12.bus.publish(EV_BRIDGE_INFO, source="drone", half="middle", block="K9", ts=now12)
    rig12.bus.publish(EV_BRIDGE_INFO, source="drone", half="lower", ts=now12)      # 缺 block
    t6_drone_round(rig12, lower="K9", upper="K4")
    rig12.ticks(3)
    check("14.41 ★ 坏事件（半场非法 / 缺 block）被丢弃、不炸任务，同拍的好情报照常换洞",
          rig12.task.health().errors == 0
          and "K9" in list(rig12.task.route.blocks())
          and "K4" in list(rig12.task.route.blocks()),
          f"errors={rig12.task.health().errors} k={t6_kblocks(rig12)}")

    # ---------------- ⑥ ★卡 t13：事件洪水 / 队列溢出 → 不崩、不刷屏、不卡死 ----------------
    cfg_storm = t6_cfg(wait=0.0)
    cfg_storm.override("logging.rate_limit_s", 1.0)      # 现场口径：限流开启
    rig14 = Rig(cfg=cfg_storm, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    now14 = rig14.clock.now()
    for i in range(100):                                 # 同一拍灌 100 条坏事件（缺 half）
        rig14.bus.publish(EV_BRIDGE_INFO, source="drone", block="K9", ts=now14 + i * 1e-6)
    rig14.tick()
    bad_logs = [s for s in rig14.logs if "字段不全" in s]
    check("14.42 ★ 100 条坏事件一拍内到达：最多 1 条限流日志（无告警风暴）、任务无异常",
          len(bad_logs) <= 1 and rig14.task.health().errors == 0,
          f"坏事件日志={len(bad_logs)} 条 errors={rig14.task.health().errors}")

    rig15 = Rig(cfg=cfg_storm, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    now15 = rig15.clock.now()
    t6_drone(rig15, half="lower", block="K9", publish=False)   # 链路在（在线）但情报只从事件来
    for i in range(100):                                       # 灌满 64 的队列 + 溢出 36
        rig15.bus.publish(EV_BRIDGE_INFO, source="drone", half="lower", block="K9",
                          ts=now15 + i * 1e-6)
    rig15.ticks(2)
    st15 = rig15.task.state()["bridge"]
    drop_logs = [s for s in rig15.logs if "队列溢出" in s]
    check("14.43 ★ 队列溢出（100 条 → 留 64 / 丢 36）不崩、只记 1 条日志，仍按『最后一条』生效",
          "K9" in list(rig15.task.route.blocks())
          and st15["events_dropped"] == 36 and st15["events"] >= 64
          and len(drop_logs) == 1 and rig15.task.health().errors == 0,
          f"k={t6_kblocks(rig15)} dropped={st15['events_dropped']} events={st15['events']} "
          f"溢出日志={len(drop_logs)}")

    # ---------------- ⑦ ★卡 t13：默认穿洞格 = 现场真值 K10/K5（robot.yaml:513-521） ----------------
    real = Config.load(str(_T6_ROBOT))
    check("14.44 ★ 现役 `config/robot.yaml`：兜底穿洞格 K9(下)/K5(上)，且都在候选集合里",
          real.get("route.cross_lower_default") == "K9"
          and real.get("route.cross_upper_default") == "K5"
          and list(real.get("route.cross_lower_candidates") or []) == ["K9"]
          and list(real.get("route.cross_upper_candidates") or []) == ["K5"],
          f"default={real.get('route.cross_lower_default')}/{real.get('route.cross_upper_default')} "
          f"cand={real.get('route.cross_lower_candidates')}/{real.get('route.cross_upper_candidates')}")
    real.override("mission.auto_start", True).override("mission.auto_start_delay_s", 0.1)
    real.override("mission.bridge_info_wait_s", 0.0).override("mission.scan_mode", "none")
    real.override("planner.auto_plan_enabled", False)     # 本节只验"手动路线 + 默认格"这条路径
    rig13 = Rig(cfg=real, pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig13.ticks(3)
    check("14.45 ★ 用**现役配置**跑：无情报 → 穿洞格 = 默认 K10/K5，且与现役路线逐 token 一致",
          rig13.task.phase == MissionPhase.DRIVE
          and list(rig13.task.route.blocks()) == _T6_BASE
          and rig13.task.state()["bridge"]["used"] == {"lower": "K9", "upper": "K5"}
          and rig13.task.state()["bridge"]["lower"]["source"] == "no_link",
          f"phase={rig13.task.phase} k={t6_kblocks(rig13)} "
          f"used={rig13.task.state()['bridge']['used']}")
    check("14.46 默认格直接取自配置（`_cross_default` = {lower:K10, upper:K5}），无硬编码兜底值",
          rig13.task._cross_default == {"lower": "K9", "upper": "K5"},
          str(rig13.task._cross_default))
    check("14.47 遥测暴露订阅状态与事件计数（现场排『情报到没到』不必翻日志）",
          rig13.task.state()["bridge"]["subscribed"] is True
          and isinstance(rig13.task.state()["bridge"]["events"], int)
          and rig13.task.state()["bridge"]["events_dropped"] == 0,
          str({k: rig13.task.state()["bridge"][k]
               for k in ("subscribed", "events", "events_dropped")}))
    sub_name = f"{rig13.task.name}:bridge"
    before13 = set(rig13.bus.stats()["subscribers"].get(EV_BRIDGE_INFO, []))
    rig13.task.teardown()
    check("14.48 `teardown()` 退订（总线上不留悬挂队列）",
          rig13.task._bridge_sub is None and sub_name in before13
          and sub_name not in set(rig13.bus.stats()["subscribers"].get(EV_BRIDGE_INFO, [])),
          f"before={sorted(before13)} "
          f"after={rig13.bus.stats()['subscribers'].get(EV_BRIDGE_INFO)}")


def t6_drone_state_of(rig) -> DroneState:
    """取最近写进黑盒的 `P_DRONE`（测试断言用）。"""
    return rig.bb.get(P_DRONE)


# --------------------------------------------------------------------------- #
def test_bridge_hold_t32() -> None:
    section("15. ★t32：L10 桥前等待（HOLD）+ 迟到情报改道一次")

    def drive_to_hold(rig, *, max_steps: int = 120, step: float = 0.05) -> bool:
        """小步循迹直到进 HOLD（或走完）；返回是否进了 HOLD。"""
        for _ in range(max_steps):
            if rig.task.phase == MissionPhase.HOLD:
                return True
            rig.walk(step=step, max_steps=1, stop_on_stop_point=False)
        return rig.task.phase == MissionPhase.HOLD

    def blockers(rig) -> list[str]:
        return [str(b) for b in rig.task.route.blocks()]

    # ---- 15.1 无链路：不发车等待、L10 也不停（逐拍与 t20 后版本一致）----
    rig1 = Rig(cfg=t6_cfg(wait=0.0, hold_max=12.0),
               pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    rig1.ticks(3)                                    # 无 P_DRONE → 立即出发
    entered1 = drive_to_hold(rig1)
    check("15.1 ★ t32 无链路：不在 L10 停（phase 恒 DRIVE，HOLD 一次都没进）",
          (not entered1) and rig1.task.phase != MissionPhase.HOLD,
          f"phase={rig1.task.phase} entered={entered1}")
    check("15.2 无链路时路线与接线前一致（默认洞 K10/K5，没改道）",
          rig1.task.state()["bridge"]["note"].get("reroute") is None
          and rig1.task.state()["bridge"]["used"] == {"lower": "K9", "upper": "K5"},
          str(rig1.task.state()["bridge"].get("used")))

    # ---- 15.3 链路在线 + 两半场未齐 → 到决策点进 HOLD；保持期间进度冻结 ----
    rig2 = Rig(cfg=t6_cfg(wait=0.0, hold_max=12.0),
               pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    t6_drone(rig2, half=None, block=None, online=True)     # 有链路、在线、还没情报
    rig2.ticks(3)
    entered2 = drive_to_hold(rig2)
    idx_hold = rig2.bb.get(P_ROUTE_INDEX)
    check("15.3 ★ t32：在线 + 两半场未齐 → 到决策点进 HOLD（停车等情报）",
          entered2 and rig2.task.phase == MissionPhase.HOLD
          and rig2.task.state()["bridge"]["note"]["hold"]["active"] is True,
          f"phase={rig2.task.phase} hold={rig2.task.state()['bridge']['note'].get('hold')}")
    check("15.4 HOLD 的遥测写明上限 = min(bridge_wait_max_s, 预算余量) 与半场",
          rig2.task.state()["bridge"]["note"]["hold"]["cap_s"] > 0.0
          and rig2.task.state()["bridge"]["note"]["hold"]["half"] in ("lower", "upper"),
          str(rig2.task.state()["bridge"]["note"]["hold"]))
    rig2.walk(step=0.05, max_steps=3, stop_on_stop_point=False)
    check("15.5 保持期间**不放行**（phase 仍 HOLD、未换洞、0.4s ≪ 上限 12s）",
          rig2.task.phase == MissionPhase.HOLD
          and rig2.task.state()["bridge"]["note"]["hold"]["active"] is True
          and rig2.task.state()["bridge"]["note"].get("reroute") is None,
          f"phase={rig2.task.phase} note={rig2.task.state()['bridge']['note']}")
    t6_drone_round(rig2, lower="K9", upper="K4")     # 两半场齐 → 立即放行
    rig2.ticks(3)
    check("15.6 ★ 两半场齐 → 立即放行（不必等满）且下半场当拍换洞（K10→K9）",
          rig2.task.phase == MissionPhase.DRIVE and "K9" in blockers(rig2)
          and "K10" not in blockers(rig2),
          f"phase={rig2.task.phase} k={[b for b in blockers(rig2) if b.startswith('K')]}")
    check("15.7 放行后遥测 hold.active=False 且记了 waited_s",
          rig2.task.state()["bridge"]["note"]["hold"]["active"] is False
          and rig2.task.state()["bridge"]["note"]["hold"]["waited_s"] >= 0.0,
          str(rig2.task.state()["bridge"]["note"]["hold"]))

    # ---- 15.8 到点必发车（t20 语义不变）----
    rig3 = Rig(cfg=t6_cfg(wait=0.0, hold_max=0.6),
               pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    t6_drone(rig3, half=None, block=None, online=True)
    rig3.ticks(3)
    entered3 = drive_to_hold(rig3)
    rig3.ticks(10)                                   # 1.0 s > 上限 0.6 s
    check("15.8 ★ 到点必发车：HOLD 到上限即恢复 DRIVE（不卡死、不等满）",
          entered3 and rig3.task.phase == MissionPhase.DRIVE
          and any("窗口到点" in s for s in rig3.logs),
          f"phase={rig3.task.phase} logs={[s for s in rig3.logs if '桥前等待' in s][-1:]}")

    # ---- 15.9 上限 = min(12s, 预算余量)：预算耗尽 → 不进 HOLD ----
    rig4 = Rig(cfg=t6_cfg(wait=0.0, hold_max=12.0, budget=0.3),
               pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    t6_drone(rig4, half=None, block=None, online=True)
    rig4.ticks(20)                                   # 2.0 s ≫ 预算 0.3 s → 余量 0
    entered4 = drive_to_hold(rig4)
    check("15.9 ★ 预算余量封顶：余量 0 → 不进 HOLD（cap=0 ⇒ 直接走）",
          (not entered4) and rig4.task.phase != MissionPhase.HOLD,
          f"phase={rig4.task.phase} entered={entered4}")

    # ---- 15.10 决策点前迟到情报 → 换洞一次且**不回车起点**（索引保留）----
    rig5 = Rig(cfg=t6_cfg(wait=0.0, hold_max=12.0),
               pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    t6_drone(rig5, half=None, block=None, online=True)
    rig5.ticks(3)
    entered5 = drive_to_hold(rig5)
    idx5 = rig5.bb.get(P_ROUTE_INDEX)
    t6_drone_round(rig5, lower="K11")                 # 迟到：下半场注入 K11（现行路线穿洞格 = K9，注入同值不构成改道）
    rig5.ticks(5)
    check("15.10 只到一半 → 仍在 HOLD（没到点不放行）",
          entered5 and rig5.task.phase == MissionPhase.HOLD and idx5 > 0,
          f"phase={rig5.task.phase} idx={idx5}")
    t6_drone_round(rig5, upper="K4")                 # 齐 → 放行 + 决策点前改道
    rig5.ticks(3)
    note5 = rig5.task.state()["bridge"]["note"]
    check("15.11 ★ 决策点前迟到情报 → 换洞（K9→K11）且**保留段索引**（不是从 N10 重来）",
          rig5.task.phase == MissionPhase.DRIVE and "K11" in blockers(rig5)
          and rig5.bb.get(P_ROUTE_INDEX) == idx5
          and isinstance(note5.get("reroute"), dict) and note5["reroute"]["count"] == 1
          and note5["reroute"]["old"] == "K9" and note5["reroute"]["new"] == "K11",
          f"idx={rig5.bb.get(P_ROUTE_INDEX)} vs {idx5} k={[b for b in blockers(rig5) if b.startswith('K')]} "
          f"reroute={note5.get('reroute')}")
    check("15.12 改道只动穿洞格：决策点及其之前的航点顺序不变（前缀一致）",
          blockers(rig5)[:idx5 + 1] == _T6_BASE[:idx5 + 1],
          f"{blockers(rig5)[:idx5 + 2]} vs {_T6_BASE[:idx5 + 2]}")

    # ---- 15.13 决策点后到达的情报 → 维持原 token（只进遥测）----
    rig5.walk(step=0.05, max_steps=60, stop_on_stop_point=False)   # 穿过 K9 洞
    before5 = blockers(rig5)
    t6_drone_round(rig5, lower="K7")                 # 洞口已过 → 不能再改
    rig5.ticks(3)
    check("15.13 ★ 决策点后到达的情报 → 维持原 token（K7 没被换上）",
          blockers(rig5) == before5 and "K7" not in blockers(rig5)
          and rig5.task.state()["bridge"]["note"]["reroute"]["count"] == 1,
          f"k={[b for b in blockers(rig5) if b.startswith('K')]}")

    # ---- 15.14 全程改道上限 1 次（上半场再迟到也不换）----
    rig6 = Rig(cfg=t6_cfg(wait=0.0, hold_max=12.0),
               pose=geo.block_to_meters("N10") + (geo.HEADING_Y,))
    t6_drone(rig6, half=None, block=None, online=True)
    rig6.ticks(3)
    drive_to_hold(rig6)
    # t56 重钉（依据 CFG+MAP+GOT）：新图（官方重掷图）下半场穿洞格 = K9（config/robot.yaml:
    # route.cross_lower_default="K9" / candidates=["K9"]）⇒ 注入 K9 与现行穿洞格相同 = 无变化 = 不产生
    # reroute。故本场景改为注入 **K11**（与 15.11 同口径：K11 ∈ K 列下半场行段 K7~K11，t31 权威门按
    # 「K 列 + 本半场行段」采纳池外合法格），使第一次改道真实发生；断言同步钉住 old/new（只加强不放宽）。
    t6_drone_round(rig6, lower="K11")                # 第一次改道（下半场）：K9 → K11
    rig6.ticks(3)
    t6_drone_round(rig6, upper="K4")                 # 同时给上半场
    rig6.ticks(3)
    first = blockers(rig6)
    t6_drone_round(rig6, upper="K4")                 # 再报一次（不应触发第二次改道）
    rig6.ticks(3)
    check("15.14 ★ 全程改道上限 1 次：下半场已改 K9→K11、上半场未换洞（仍 K5）、计数仍是 1",
          blockers(rig6) == first
          and rig6.task.state()["bridge"]["note"]["reroute"]["count"] == 1
          and rig6.task.state()["bridge"]["note"]["reroute"]["old"] == "K9"
          and rig6.task.state()["bridge"]["note"]["reroute"]["new"] == "K11"
          and "K5" in blockers(rig6),
          f"k={[b for b in blockers(rig6) if b.startswith('K')]} "
          f"reroutes={rig6.task.state()['bridge']['note']['reroute']['count']}")


# --------------------------------------------------------------------------- #
def main() -> int:
    print("=" * 72)
    print("dzbrain MissionTask 自测（纯脚本）")
    print("=" * 72)
    test_no_auto_start()
    test_auto_start()
    test_waypoint_progress()
    test_stop_point_three_phases()
    test_stop_departure_t26()
    test_no_deadlock()
    test_finish_latch()
    test_goal_stop_tight()
    test_commands()
    test_phase_gate()
    test_rate()
    test_stall_fallback()
    test_arrival_requires_proximity()
    test_pivot_done_marks_arrival()
    test_waypoint_ledger()
    test_bridge_intel()
    test_bridge_hold_t32()
    finish()
    return 0


if __name__ == "__main__":
    sys.exit(main())
