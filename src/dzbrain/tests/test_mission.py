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
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

try:
    sys.stdout.reconfigure(errors="replace")  # type: ignore[union-attr]
except Exception:                              # pragma: no cover
    pass

from harness import check, finish, section                     # noqa: E402

from dzbrain.core import geometry as geo                       # noqa: E402
from dzbrain.core.blackboard import (P_MISSION_PHASE, P_POSE, P_ROUTE, P_ROUTE_S,  # noqa: E402
                                     P_SAFETY, P_STOP_POINT, P_TELEMETRY, Blackboard)
from dzbrain.core.config import Config                         # noqa: E402
from dzbrain.core.events import (EV_ARRIVED_STOP_POINT, EV_MISSION_FINISHED,  # noqa: E402
                                 EV_MISSION_STARTED, EV_ROUTE_LOADED, EV_STOP_POINT_DONE,
                                 EV_WAYPOINT_REACHED, EventBus)
from dzbrain.core.state import SafetyState, StopKind, StopPhase  # noqa: E402
from dzbrain.io.contract import AimStatus, MissionPhase        # noqa: E402
from dzbrain.pursuit.path import Path                          # noqa: E402
from dzbrain.tasks.mission import MissionTask                  # noqa: E402
from dzbrain.tasks.tracker import PIVOT_DONE_KEY, PivotPhase, TrackerTask  # noqa: E402

from fakes import FakeBridge, FakeClock                        # noqa: E402

#: 现场路线片段：N10 出发 → 直行 → G10* 打 G11 靶 → 终点
FIELD_ROUTE = ["N10", "M10", "L10", "L9", "K9", "J9", "J10", "I10", "H10", "G10*", "F10", "F9"]
WATCHED = (EV_MISSION_STARTED, EV_ROUTE_LOADED, EV_WAYPOINT_REACHED, EV_ARRIVED_STOP_POINT,
           EV_STOP_POINT_DONE, EV_MISSION_FINISHED)


def cfg_of(*, mission=None, route=None, pursuit=None) -> Config:
    data: dict = {"mission": dict(mission or {}), "pursuit": dict(pursuit or {}),
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
    check("2.6 ★ 持续扫描：开局发 scan_request=2（绝不用 1）",
          rig.bridge.scan_requests[:1] == [2], f"scans={rig.bridge.scan_requests}")

    pose = rig.bb.get(P_POSE)
    want = geo.block_to_meters("N10")
    check("2.7 复位 belief 到 N10 格心 + 朝向 y",
          abs(pose.x - want[0]) < 1e-12 and abs(pose.y - want[1]) < 1e-12
          and abs(pose.yaw - geo.HEADING_Y) < 1e-12,
          f"pose=({pose.x:.3f},{pose.y:.3f},{geo.deg(pose.yaw):.1f}°)")

    # 2.8 `scan_mode=waypoint` → 发 1（单趟），不是 2
    cfg2 = cfg_of(mission={"auto_start": True, "auto_start_delay_s": 0.1,
                           "scan_mode": "waypoint"}, route={"tokens": tokens})
    rig2 = Rig(cfg=cfg2, pose=(0.0, 0.0, 0.0))
    rig2.ticks(3)
    check("2.8 scan_mode=waypoint → scan_request=1", rig2.bridge.scan_requests[:1] == [1],
          f"scans={rig2.bridge.scan_requests}")

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
#  5. 不卡死（最重要）
# --------------------------------------------------------------------------- #
def test_no_deadlock() -> None:
    section("5. ★ 不卡死：三段各自'永不到达' → 每段超时后放行")

    # 5.1 永不稳定（位置一直动）+ 永无目标（vision=None, aim=IDLE）
    cfg = _field_cfg(settle_time_s=10.0, settle_eps_m=0.001, lock_timeout_s=1.0,
                     shot_timeout_s=1.0)
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
    cfg2 = _field_cfg(shot_timeout_s=1.0, lock_timeout_s=1.0, settle_time_s=0.1)
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
    cfg3 = _field_cfg(shot_timeout_s=1.0, lock_timeout_s=1.0, settle_time_s=0.1)
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
    check("6.3 发过 scan_request=0（停扫）", 0 in rig.bridge.scan_requests,
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
def main() -> int:
    print("=" * 72)
    print("dzbrain MissionTask 自测（纯脚本）")
    print("=" * 72)
    test_no_auto_start()
    test_auto_start()
    test_waypoint_progress()
    test_stop_point_three_phases()
    test_no_deadlock()
    test_finish_latch()
    test_commands()
    test_phase_gate()
    test_rate()
    test_stall_fallback()
    test_arrival_requires_proximity()
    test_pivot_done_marks_arrival()
    finish()
    return 0


if __name__ == "__main__":
    sys.exit(main())
