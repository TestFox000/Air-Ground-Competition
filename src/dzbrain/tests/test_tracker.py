# -*- coding: utf-8 -*-
"""TrackerTask 自测（**纯脚本，无 pytest**）。

    python src\\dzbrain\\tests\\test_tracker.py

覆盖（逐条给数值证据；每条都注明它防的是哪个旧毛病）：
  1. 前置门五种条件**逐条**断言"只刹车、不发运动指令"
  2. 正常态：单拍喂位姿 → 非零 linear.x、|舵角| ≤ max_steer_deg、路线弧长被回写
  3. 桥洞保护：穿 K9 的段被锁航向 ±8° + 0.10 m/s；
     ★ 普通段（K 列附近但**不在保护航段内**）必须走纯追踪 —— 证明没有旧版"K 列附近全域锁航向"
  4. 倒车段：linear_x < 0，`motion_state` 只记到一次 True（状态变化才发）
  5. 接近停站点降速：G10* 前 0.5 m → 速度低于巡航
  6. Path 缓存：连续 100 拍不重建（`id()` 断言）
  7. 保护集合按航段索引预计算，且**只在路线变化时重建**

全过 exit 0；任一失败 exit 1。
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

# ⚠️ Windows 控制台默认 GBK：编不出的字符降级成替换字符，保证测试结果不被编码问题掩盖
try:
    sys.stdout.reconfigure(errors="replace")  # type: ignore[union-attr]
except Exception:                              # pragma: no cover
    pass

from harness import check, finish, section                     # noqa: E402

from dzbrain.core import geometry as geo                       # noqa: E402
from dzbrain.core.blackboard import (P_MISSION_PHASE, P_MOTION_LIMIT, P_POSE, P_ROUTE,  # noqa: E402
                                     P_ROUTE_INDEX, P_ROUTE_S, P_SAFETY, Blackboard)
from dzbrain.core.config import Config                         # noqa: E402
from dzbrain.core.events import EventBus                       # noqa: E402
from dzbrain.core.state import SafetyState                     # noqa: E402
from dzbrain.io.contract import MissionPhase                   # noqa: E402
from dzbrain.pursuit.path import Path                          # noqa: E402
from dzbrain.route.parser import parse_mission                 # noqa: E402
from dzbrain.tasks.tracker import TRACKER_CMD_KEY, TrackerTask  # noqa: E402

from fakes import FakeBridge, FakeClock                        # noqa: E402

# --------------------------------------------------------------------------- #
#  脚手架
# --------------------------------------------------------------------------- #
#: 现场 K 列整列：桥洞保护点（配置驱动，代码里没有"K 列"特判）
K_COLUMN = [f"K{r}" for r in range(1, 12)]


def cfg_of(*, bridge_blocks=None, rate_limit_s: float = 1.0, **pursuit) -> Config:
    """配置：`pursuit.*` 可覆盖；桥洞点默认给整列 K1~K11（= 现场 robot.yaml）。

    `logging.rate_limit_s` 默认 1.0 s（与 SCHEMA 一致），测试 1.5b 用它验证拦停日志限流。
    """
    data = {"pursuit": dict(pursuit), "logging": {"rate_limit_s": rate_limit_s}}
    if bridge_blocks is not None:
        data["pursuit"]["bridge_blocks"] = list(bridge_blocks)
    else:
        data["pursuit"]["bridge_blocks"] = list(K_COLUMN)
    return Config.from_dict(data)


class Rig:
    """一台"车"：FakeBridge + Blackboard + TrackerTask。"""

    def __init__(self, *, cfg: Config | None = None, route=None, pose=None,
                 phase: str = MissionPhase.DRIVE, safety: SafetyState | None = None) -> None:
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.cfg = cfg or cfg_of()
        self.logs: list[str] = []
        self.task = TrackerTask(self.bridge, self.bb, self.bus, self.cfg,
                                log=self.logs.append, clock=self.clock.now)
        self.bb.set(P_SAFETY, safety if safety is not None else SafetyState(ok=True), "test")
        self.bb.set(P_MISSION_PHASE, phase, "test")
        if route is not None:
            self.bb.set(P_ROUTE, route, "test")
        if pose is not None:
            self.set_pose(*pose)

    # ---------------- 操作 ----------------
    def set_pose(self, x: float, y: float, yaw: float) -> None:
        """喂位姿：**两侧都写** —— bridge 供 `pose_age` 语义，blackboard 供 `P_POSE` 决策。"""
        self.bridge.set_pose(x, y, yaw)
        self.bb.set(P_POSE, geo.Pose(x, y, yaw), "test")

    def set_pose_at(self, block: str, yaw: float = 0.0) -> None:
        self.bridge.set_pose_at_block(block, yaw)
        x, y = geo.block_to_meters(block)
        self.bb.set(P_POSE, geo.Pose(x, y, yaw), "test")

    def set_block_pose(self, block: str, yaw: float = 0.0) -> None:
        self.set_pose_at(block, yaw)

    def ticks(self, n: int = 1, dt: float = 0.0) -> None:
        for _ in range(n):
            self.task.tick()
            if dt:
                self.clock.advance(dt)

    def cmds(self):
        return list(self.bridge.commands)

    def moved(self):
        return self.bridge.moved_commands()

    def cmd(self) -> dict:
        return self.task.telemetry()


def route_of(*blocks):
    from dzbrain.route.model import make_route, Waypoint
    return make_route([Waypoint.at_block(b) for b in blocks])


def gate_case(name: str, rig: Rig, *, expect_reason: str = "") -> None:
    """门条件公共断言：**只刹车、不发运动指令**，且原因是可读的。"""
    rig.ticks(2)
    m = rig.moved()
    ok = (not m) and rig.bridge.brakes >= 1 and all(c == (0.0, 0.0) for c in rig.cmds())
    check(f"{name}：只刹车、无运动指令", ok,
          f"brakes={rig.bridge.brakes} 指令={rig.cmds()} 运动指令={m}")
    info = rig.cmd()
    if expect_reason:
        check(f"{name}：原因含 {expect_reason!r}", expect_reason in str(info.get("reason", "")),
              f"reason={info.get('reason')!r}")


# --------------------------------------------------------------------------- #
#  1. 前置门（逐条）
# --------------------------------------------------------------------------- #
def test_gates() -> None:
    section("1. 前置门：五种条件逐条" + "只刹车、不发运动指令")

    # 1.1 无位姿
    rig = Rig(route=route_of("N10", "M10"))
    gate_case("1.1 无 P_POSE", rig, expect_reason="无位姿")
    check("1.1b 无位姿时连 P_ROUTE_INDEX 都不写（不拿旧值做决策）",
          rig.bb.get(P_ROUTE_INDEX) is None, f"{rig.bb.get(P_ROUTE_INDEX)!r}")

    # 1.2 无路线
    rig = Rig(pose=(0.6, 0.2, 0.0))
    gate_case("1.2 无 P_ROUTE", rig, expect_reason="无路线")

    # 1.3 阶段 IDLE（以及其余五个明令刹车的阶段）
    for phase in (MissionPhase.IDLE, MissionPhase.STOP, MissionPhase.SHOOT,
                  MissionPhase.WAIT_SCAN, MissionPhase.FINISH, MissionPhase.SAFE_STOP):
        rig = Rig(route=route_of("N10", "M10"), pose=(0.6, 0.2, 0.0), phase=phase)
        gate_case(f"1.3 阶段 {phase}", rig, expect_reason=phase)

    # 1.4 安全非 OK（★ 非 SAFE 时 Tracker 必须刹车）
    rig = Rig(route=route_of("N10", "M10"), pose=(0.6, 0.2, 0.0),
              safety=SafetyState(ok=False, reasons=("定位失联",)))
    gate_case("1.4 P_SAFETY.ok=False", rig, expect_reason="安全不 OK")
    check("1.4b 原因里带上了安全原因", "定位失联" in str(rig.cmd().get("reason")),
          f"reason={rig.cmd().get('reason')!r}")

    # 1.5 遥控接管
    rig = Rig(route=route_of("N10", "M10"), pose=(0.6, 0.2, 0.0))
    rig.bridge.set_manual_override(True)
    gate_case("1.5 manual_override=True", rig, expect_reason="遥控接管")

    # 1.5b 同一原因限流：连续 50 拍（跨 1 s 窗口）只打一条日志，但每拍都发刹车
    rig.ticks(50, dt=0.02)                          # 先跨过 1 s 窗口
    rig.logs.clear()
    brakes_before = rig.bridge.brakes
    rig.ticks(50, dt=0.02)                          # 窗口内 1 s：只应放行 1 条
    blocked_logs = [line for line in rig.logs if line.startswith("刹车：")]
    check("1.5b 同一拦停原因限流（1 s 窗口内 1 条日志），但每拍都发刹车",
          len(blocked_logs) == 1 and (rig.bridge.brakes - brakes_before) == 50,
          f"日志 {len(blocked_logs)} 条 / 本段刹车 {rig.bridge.brakes - brakes_before} 次")

    # 1.6 空路线（长度 0）同样拦
    rig = Rig(route=route_of("N10", "M10"), pose=(0.6, 0.2, 0.0))
    rig.bb.set(P_ROUTE, None, "test")
    gate_case("1.6 路线被清空", rig, expect_reason="无路线")


# --------------------------------------------------------------------------- #
#  2. 正常态
# --------------------------------------------------------------------------- #
def test_normal() -> None:
    section("2. 正常态：非零速度 + 舵角限幅 + 弧长回写")
    cfg = cfg_of(cruise_speed_mps=0.15, max_steer_deg=30.0)
    rig = Rig(cfg=cfg, route=route_of("N10", "M10", "L10", "L9", "K9"), pose=(0.6, 0.2, 0.0))
    rig.ticks(1)
    cmd = rig.cmd()
    check("2.1 发出非零 linear.x", cmd.get("speed", 0.0) > 0.0,
          f"v={cmd.get('speed')!r} reason={cmd.get('speed_reason')!r}")
    check("2.2 |舵角| ≤ max_steer_deg", abs(cmd.get("steer_deg", 0.0)) <= 30.0,
          f"steer={cmd.get('steer_deg')!r}°")
    check("2.3 段索引/弧长已回写黑盒（与投影结果同值）",
          rig.bb.get(P_ROUTE_INDEX) == cmd.get("seg_index")
          and abs(float(rig.bb.get(P_ROUTE_S)) - float(cmd.get("s"))) < 1e-12,
          f"index={rig.bb.get(P_ROUTE_INDEX)} s={rig.bb.get(P_ROUTE_S)!r}")
    check("2.4 遥测写进 P_TELEMETRY[TRACKER_CMD_KEY]",
          isinstance(rig.bb.get("telemetry"), dict)
          and TRACKER_CMD_KEY in rig.bb.get("telemetry"), f"{list((rig.bb.get('telemetry') or {}))}")

    # 2.5 对准直路（在首段上、正好落后前视距离 0.18 m）→ 舵角 = 0、速度 = 巡航
    rig2 = Rig(cfg=cfg, route=route_of("N10", "M10", "L10"),
               pose=(0.525, 0.355, geo.HEADING_Y))
    rig2.ticks(1)
    cmd2 = rig2.cmd()
    check("2.5 对准直路：舵角 = 0 且速度 = 巡航",
          abs(cmd2.get("steer_deg", 1.0)) < 1e-9 and abs(cmd2.get("speed", 0.0) - 0.15) < 1e-9,
          f"steer={cmd2.get('steer_deg')!r}° v={cmd2.get('speed')!r} m/s")

    # 2.5b 同一路线但横向偏 0.15 m → 舵角显著非零（证明舵角真的在跟随横向偏差）
    rig2c = Rig(cfg=cfg, route=route_of("N10", "M10", "L10"),
                pose=(0.675, 0.355, geo.HEADING_Y))     # 横向偏 0.15 m
    rig2c.ticks(1)
    check("2.5b 横向偏 0.15 m → 舵角显著非零（不是恒 0 的死控制器）",
          abs(rig2c.cmd().get("steer_deg", 0.0)) > 10.0,
          f"steer={rig2c.cmd().get('steer_deg')!r}°（对准时 {rig2.cmd().get('steer_deg')!r}°）")

    # 2.6 大偏差 → 被硬限幅到 max_steer_deg（不是无限大）
    rig3 = Rig(cfg=cfg, route=route_of("N10", "M10", "L10"), pose=(0.6, 1.4, 0.0))
    rig3.ticks(1)
    check("2.6 横向大偏差 → 舵角被限到 max_steer_deg(30°)",
          abs(rig3.cmd().get("steer_deg", 0.0)) == 30.0,
          f"steer={rig3.cmd().get('steer_deg')!r}°")


# --------------------------------------------------------------------------- #
#  3. 桥洞保护（航段索引，不是几何距离）
# --------------------------------------------------------------------------- #
def test_bridge_align() -> None:
    section("3. 桥洞直行保护：穿 K9 段锁航向 ±8° / 普通段走纯追踪")
    cfg = cfg_of(cruise_speed_mps=0.15, align_kp=12.0, align_steer_limit_deg=8.0,
                 align_straight_speed_mps=0.10, align_straight_before=2.0,
                 align_straight_after=1.0)

    # 3.1 穿 K9：段索引 1（L9→K9）在保护集合内
    #     路线用 4 点（末段也必须存在），保护集合 = [1-2, 1+1] ∩ [0, 2] = {0,1,2}
    route = route_of("L9", "K9", "J9", "J10")
    rig = Rig(cfg=cfg, route=route)
    rig.set_pose(0.875, 0.70, geo.HEADING_Y)          # K9 前一格、朝向正东
    rig.ticks(1)
    segs = rig.task.last_align_segments
    cmd = rig.cmd()
    check("3.1 保护航段集合 = {0,1,2}（航点 1=K9，before=2/after=1 展开后钳到 [0,末段]）",
          segs == frozenset({0, 1, 2}), f"segs={sorted(segs)}")
    check("3.2 命中保护段：align=True、速度 = align_straight_speed_mps(0.10)、舵角 0（朝向已对）",
          cmd.get("align") is True and abs(cmd.get("speed", 0.0) - 0.10) < 1e-12
          and abs(cmd.get("steer_deg", 1.0)) < 1e-9,
          f"align={cmd.get('align')} v={cmd.get('speed')!r} steer={cmd.get('steer_deg')!r}°")

    # 3.3 航向偏 +20°（车头偏左）→ e = 段方向 - 车朝向 = -20° → 舵角 = clamp(-24, ±8) = -8
    rig.set_pose(0.875, 0.70, geo.HEADING_Y + geo.rad(20.0))
    rig.ticks(1)
    cmd = rig.cmd()
    check("3.3 航向偏 20° → 舵角被限在 -align_steer_limit_deg(8°)（未限幅需求 = -24°）",
          abs(cmd.get("steer_deg", 0.0) + 8.0) < 1e-9,
          f"e=-20°，20×12/10 = 24° → 输出 {cmd.get('steer_deg')!r}°")

    # 3.4 反向偏 20°（车头偏右）→ +8°（正=左转契约）
    rig.set_pose(0.875, 0.70, geo.HEADING_Y - geo.rad(20.0))
    rig.ticks(1)
    check("3.4 反向偏 20° → +8°（正=左转契约）",
          abs(rig.cmd().get("steer_deg", 0.0) - 8.0) < 1e-9, f"{rig.cmd().get('steer_deg')!r}°")

    # 3.5 ★ 关键反例：不在保护航段内 → 必须走纯追踪，不是"K 列附近全域锁航向"
    #     路线 D5→D1 沿列直下（离 K 列 6 格），车与路线平行、横向偏 0.15 m：
    #     · align 模式在此姿态下 e=0 → 舵角 0°、速度 0.10 m/s
    #     · 纯追踪看到 0.15 m 横偏 → 舵角 ≈ 21°、速度 0.15 m/s
    #     两者数值差距极大，断言不会因浮点噪声假过。
    route2 = route_of("D5", "D4", "D3", "D2", "D1")
    rig2 = Rig(cfg=cfg, route=route2)
    rig2.set_pose(2.425, 2.50, geo.rad(-90.0))         # 与 D 列平行、横向偏 0.15 m
    rig2.ticks(1)
    cmd2 = rig2.cmd()
    check("3.5 普通段：align=False（不套用 ±8° 限幅）", cmd2.get("align") is False
          and rig2.task.last_align_segments == frozenset(),
          f"align={cmd2.get('align')} segs={sorted(rig2.task.last_align_segments)}")
    check("3.6 普通段：横向偏 0.15 m → 舵角 > 8°（align 模式在此姿态下会输出 0°）",
          abs(cmd2.get("steer_deg", 0.0)) > 8.0,
          f"steer={cmd2.get('steer_deg')!r}°（align 模式会输出 0°）")
    check("3.7 普通段：速度 0.15 ≠ align 0.10（没有把普通段也压成直行速度）",
          abs(cmd2.get("speed", 0.0) - 0.10) > 1e-9 and abs(cmd2.get("speed", 0.0) - 0.15) < 1e-9,
          f"v={cmd2.get('speed')!r} reason={cmd2.get('speed_reason')!r}")

    # 3.8 保护结合集只算一次：同一路线 50 拍不重建
    route3 = route_of("L9", "K9", "J9", "J10")
    rig3 = Rig(cfg=cfg, route=route3)
    rig3.set_pose(0.875, 0.70, geo.HEADING_Y)
    rig3.ticks(1)
    key_before = rig3.task._align_build_key
    segs_before = rig3.task.last_align_segments
    rig3.ticks(50)
    check("3.8 保护集合 50 拍不重建（key 与内容都不变）",
          rig3.task._align_build_key == key_before and rig3.task.last_align_segments == segs_before,
          f"key 不变={rig3.task._align_build_key == key_before}")

    # 3.9 路线变化 → 必须重建
    rig3.bb.set(P_ROUTE, route2, "test")
    rig3.set_pose(2.425, 2.50, geo.rad(-90.0))
    rig3.ticks(1)
    check("3.9 路线变化 → 保护集合重建为空（新路线不穿洞）",
          rig3.task.last_align_segments == frozenset(),
          f"segs={sorted(rig3.task.last_align_segments)}")

    # 3.10 没有配置 bridge_blocks → 不做桥洞保护（必须显式配置，见 SCHEMA 说明）
    cfg_none = cfg_of(bridge_blocks=[], cruise_speed_mps=0.15)
    rig4 = Rig(cfg=cfg_none, route=route_of("L9", "K9", "J9", "J10"))
    rig4.set_pose(0.875, 0.70, geo.HEADING_Y)
    rig4.ticks(1)
    check("3.10 bridge_blocks 为空 → 保护集合空、按纯追踪跑（不猜 K 列）",
          rig4.task.last_align_segments == frozenset() and rig4.cmd().get("align") is False
          and abs(rig4.cmd().get("speed", 0.0) - 0.15) < 1e-9,
          f"v={rig4.cmd().get('speed')!r} align={rig4.cmd().get('align')}")


# --------------------------------------------------------------------------- #
#  4. 倒车段
# --------------------------------------------------------------------------- #
def test_reverse() -> None:
    section("4. 倒车段：linear_x < 0 且 motion_state 只在变化时发")
    from dzbrain.route.model import make_route, Waypoint
    cfg = cfg_of(cruise_speed_mps=0.15, reverse_speed_ratio=0.6, reverse_steer_sign=1.0)
    route = make_route([Waypoint.at_block("B5"), Waypoint.at_block("B6", reverse=True),
                        Waypoint.at_block("B7", reverse=True)])
    rig = Rig(cfg=cfg, route=route)
    rig.set_pose(2.1, 4.375, geo.rad(180.0))        # 车头朝 -x（段方向），倒着往 B6 走
    rig.ticks(1)
    cmd = rig.cmd()
    check("4.1 倒车段：linear_x < 0", cmd.get("speed", 0.0) < 0.0,
          f"v={cmd.get('speed')!r} m/s reverse={cmd.get('reverse')}")
    check("4.2 倒车速度 = -(巡航 × reverse_speed_ratio) = -0.09（下界 min_speed 保护）",
          abs(cmd.get("speed", 0.0) + 0.09) < 1e-12,
          f"v={cmd.get('speed')!r}，期望 -0.09")
    check("4.3 reverse 标记来自路线数据（`path.segment_is_reverse`）",
          cmd.get("reverse") is True, f"{cmd.get('reverse')}")

    rig.ticks(9)
    check("4.4 motion_state 只发一次（倒车状态未变化）",
          rig.bridge.motion_states == [True], f"{rig.bridge.motion_states}")
    check("4.5 后续 9 拍速度依旧为负（不是只第一拍倒车）",
          all(c[0] < 0 for c in rig.moved()) and len(rig.moved()) == 10,
          f"共 {len(rig.moved())} 拍，速度集合={sorted({round(c[0], 4) for c in rig.moved()})}")

    # 4.6 回到前进段 → 状态变化 → 再发一次 False
    rig.bb.set(P_ROUTE, route_of("B7", "B5"), "test")
    rig.set_pose(1.65, 4.375, geo.rad(180.0))
    rig.ticks(1)
    check("4.6 切回前进段 → motion_state 再发一次 False（仅变化时发）",
          rig.bridge.motion_states == [True, False], f"{rig.bridge.motion_states}")
    check("4.7 前进段速度为正",
          bool(rig.moved()) and rig.moved()[-1][0] > 0.0, f"{rig.moved()[-1:] }")


# --------------------------------------------------------------------------- #
#  5. 接近停站点降速
# --------------------------------------------------------------------------- #
def test_stop_approach() -> None:
    section("5. 接近停站点降速（未完成的停站点才算）")
    from dzbrain.route.model import make_route, Waypoint
    cfg = cfg_of(cruise_speed_mps=0.15, approach_m=0.60, stop_approach_speed_gain=0.30)
    route = make_route([Waypoint.at_block("G10", shoot=True), Waypoint.at_block("G11")])
    rig = Rig(cfg=cfg, route=route)
    rig.set_pose(0.525, 2.275, geo.rad(180.0))       # G10 前 0.35 m（= H10 格心）
    rig.ticks(1)
    cmd = rig.cmd()
    check("5.1 距未完成停站点 0.35 m → 速度低于巡航",
          cmd.get("speed", 1.0) < 0.15, f"v={cmd.get('speed')!r} < 0.15（reason={cmd.get('speed_reason')!r}）")
    check("5.2 限速主因 = stop_approach",
          cmd.get("speed_reason") == "stop_approach", f"{cmd.get('speed_reason')!r}")

    # 5.3 停站点已在身后（proj.seg_index 之后没有未完成停站点）→ 不减速
    rig2 = Rig(cfg=cfg, route=route)
    rig2.set_pose(0.525, 3.10, geo.rad(180.0))       # 已过 G10、朝 G11 走
    rig2.ticks(1)
    cmd2 = rig2.cmd()
    check("5.4 停站点在身后 → 不因它减速（`_dist_to_stop` 只算未完成的）",
          cmd2.get("speed_reason") != "stop_approach",
          f"reason={cmd2.get('speed_reason')!r} v={cmd2.get('speed')!r}")


# --------------------------------------------------------------------------- #
#  6. Path 缓存
# --------------------------------------------------------------------------- #
def test_path_cache() -> None:
    section("6. Path 缓存：连续 100 拍不重建")
    rig = Rig(route=route_of("N10", "M10", "L10", "L9", "K9"), pose=(0.6, 0.2, 0.0))
    rig.ticks(1)
    first = rig.task.path
    ids = {id(rig.task.path)}
    for i in range(100):
        rig.set_pose(0.6 + (i % 5) * 0.01, 0.2, 0.0)
        rig.ticks(1)
        ids.add(id(rig.task.path))
    check("6.1 100 拍后 Path 实例仍是同一个（id 唯一）",
          len(ids) == 1 and rig.task.path is first,
          f"id 集合大小={len(ids)}")
    check("6.2 每拍都真的发了指令（不是因为没算所以没发）",
          len(rig.moved()) == 101, f"{len(rig.moved())} 拍运动指令")

    # 6.3 路线换掉 → 必须重建（缓存不能跨路线复用）
    other = route_of("N10", "M10", "L10", "L9", "K9")
    rig.bb.set(P_ROUTE, other, "test")
    rig.ticks(1)
    check("6.4 路线变化 → Path 重建（新对象）", rig.task.path is not first,
          f"identity 变化={rig.task.path is not first}")


# --------------------------------------------------------------------------- #
#  7. 50 Hz 节拍（真线程）
# --------------------------------------------------------------------------- #
def test_rate() -> None:
    section("7. 50 Hz 节拍（真线程 0.4 s）")
    import time
    rig = Rig(route=route_of("N10", "M10", "L10", "L9"), pose=(0.9, 0.9, 0.0))
    rig.task.start()
    try:
        time.sleep(0.4)
    finally:
        rig.task.stop(timeout=1.0)
    ticks = rig.task.health().ticks
    check("7.1 0.4 s 内拍数在 50 Hz 容差内（14~26）", 14 <= ticks <= 26, f"ticks={ticks}")
    check("7.2 至少发过一次指令", len(rig.moved()) >= 10, f"{len(rig.moved())}")
    check("7.3 未发生 tick 异常", rig.task.health().errors == 0,
          f"errors={rig.task.health().errors} last={rig.task.health().last_error!r}")


# --------------------------------------------------------------------------- #
# --------------------------------------------------------------------------- #
#  8. ★自重叠折返：命令符号不得每拍翻转（实闭环 bench 的 F8/B8/B6 停滞）
# --------------------------------------------------------------------------- #
def test_overlap_no_flipflop() -> None:
    section("8. ★折返压线处投影不得来回倒带（bench 的 F8/B8/B6 停滞）")
    # 实闭环 bench（--bench --drive --seconds 240）在 F11 折返簇实测：命令在
    # +0.120/-0.080 之间**每拍翻转**、段号在两条共线段之间来回跳、车毫米级抖动，
    # 12s 后触发「拐角绕圈兜底」强制跳点（F8/B8/B6 三处，全在同一段自重叠折返上）。
    #
    # 根因：去程段与回程段**完全共线** ⇒ 同一物理点到两条段等距（横向偏移相同），
    # 而两条候选的 s 相差整整一段。旧代码在等距时**先比行进方向**，而方向又由命令
    # 决定 ⇒ 自指环：命令符号 → 运动方向 → 选哪条段 → is_reverse → 命令符号。
    # 修法：等距时**先比与上拍投影的连续性**（弧长不跳变），方向只在连续性也分不出
    # 时使用（典型就是折返顶点：两条候选的 s 相同）。
    #
    # 本组用「bench 同款积分模型」驱动**真实 TrackerTask**，在 B6→A6→R:B6 上跑 24 s。
    cfg = cfg_of(pivot_enabled=True, bridge_blocks=[])
    route = parse_mission(["B6", "A6", "R:B6", "R:D6"])
    a6 = geo.block_to_meters("A6")
    b6 = geo.block_to_meters("B6")
    ux, uy = a6[0] - b6[0], a6[1] - b6[1]
    dist = math.hypot(ux, uy)
    ux, uy = ux / dist, uy / dist
    rig = Rig(cfg=cfg, route=route, pose=(b6[0], b6[1], geo.heading_of(ux, uy)))
    wheelbase = float(cfg.get("pursuit.wheelbase_m"))
    dt = 0.02
    signs: list[int] = []
    s_track: list[float] = []
    for _ in range(1200):                       # 24 s @ 50 Hz
        rig.task.tick()
        v, steer = rig.bridge.commands[-1]
        signs.append(0 if abs(v) < 1e-9 else (1 if v > 0 else -1))
        p = rig.bb.get(P_POSE)
        yaw = p.yaw + (v / wheelbase) * math.tan(math.radians(steer)) * dt
        x = p.x + v * math.cos(yaw) * dt
        y = p.y + v * math.sin(yaw) * dt
        rig.set_pose(x, y, yaw)
        rig.clock.advance(dt)
        s_now = rig.bb.get(P_ROUTE_S)
        s_track.append(float(s_now) if isinstance(s_now, (int, float)) else -1.0)

    def _flips(seq: list[int]) -> int:
        n, prev = 0, 0
        for sg in seq:
            if sg and prev and sg != prev:
                n += 1
            if sg:
                prev = sg
        return n

    check("8.1 ★折返走完：s 到达回程末端（≥0.66 m，全程 0.70 m）",
          max(s_track) >= 0.66, f"max s={max(s_track):.3f}")
    check("8.2 ★终点回到 B6 一侧（没有越走越远）",
          abs(s_track[-1] - max(s_track)) < 0.35 or s_track[-1] >= 0.60,
          f"末 s={s_track[-1]:.3f} max={max(s_track):.3f}")
    check("8.3 ★命令符号翻转 ≤150 次/24s（修复前 601 次=每拍翻转）",
          _flips(signs) <= 150, f"{_flips(signs)} 次")
    check("8.4 ★最后 4 秒已收敛（≤5 次翻转，不是一直在抖）",
          _flips(signs[-200:]) <= 5, f"尾 200 拍翻转 {_flips(signs[-200:])} 次")
    check("8.5 ★倒车段折返顶点不进入 PIVOT（车头无需转，倒着原路退回）",
          sum(1 for x in rig.logs if "进入 PIVOT" in x) == 0 and rig.task.pivot_count == 0,
          f"进入 {sum(1 for x in rig.logs if '进入 PIVOT' in x)} 次 / 完成 {rig.task.pivot_count} 次")
    check("8.6 ★确实发出了倒车指令（倒车段入口'推一把'生效）",
          any(sg < 0 for sg in signs),
          f"速度集合={sorted({round(c[0], 3) for c in rig.bridge.commands})}")


def test_arc_floor_no_oscillation() -> None:
    """★9. 弧长地板：倒车段↔正向段交界处不得来回振荡（现场 F11 簇）。

    实闭环 bench 与确定性探针都复现过同一故障：车倒车回到 F11（seg12 `E11→R:F11`
    倒车段结束、seg13 正向去 F10）后**永久振荡**，机制是三重叠加：
      · 两条候选（倒车段末端 / 正向段起点）的 s 都在顶点附近、Δs 几乎相同
        ⇒ 第 ① 顺位"连续性裁决"分不出来；
      · 于是落到第 ② 顺位"行进方向"，而方向由命令符号决定、命令符号又由 `is_rev`
        决定 ⇒ 自指环；
      · 此时车**真的**在抖（0.08~0.15 m/s × 0.02 s = 1.6~3 mm/拍），**大于**行进方向
        的锁存门限 1 mm ⇒ 连"实测方向"也每拍跟着翻。
    探针实测 30 s 内 **715 次**命令符号翻转、位置冻在毫米级；bench 里表现为
    「拐角绕圈兜底」每 12 s 跳一个点（到点 30 → 17）。

    修法：投影加**弧长地板**（已经走过的顶点不允许退回去）—— 与仿真
    `_project_with_floor` 同语义，把"顶点之前那条段"整个排除出搜索窗口，环就断了。
    """
    section("9. ★弧长地板：倒车段↔正向段交界处不得振荡（F11 簇）")
    cfg = cfg_of(pivot_enabled=True, bridge_blocks=[])
    tokens = ["F10", "F11", "E11", "R:F11", "F10", "F8", "B8", "B6"]
    p0 = geo.block_to_meters("F10")
    p1 = geo.block_to_meters("F11")
    ux, uy = p1[0] - p0[0], p1[1] - p0[1]
    d = math.hypot(ux, uy)
    ux, uy = ux / d, uy / d
    rig = Rig(cfg=cfg, route=parse_mission(tokens),
              pose=(p0[0], p0[1], geo.heading_of(ux, uy)))
    wheelbase = float(cfg.get("pursuit.wheelbase_m"))
    dt = 0.02
    signs: list[int] = []
    segs: list[int] = []
    s_track: list[float] = []
    for _ in range(1500):                      # 30 s @ 50 Hz
        rig.task.tick()
        v, steer = rig.bridge.commands[-1]
        signs.append(0 if abs(v) < 1e-9 else (1 if v > 0 else -1))
        seg_now = rig.bb.get(P_ROUTE_INDEX)
        segs.append(int(seg_now) if isinstance(seg_now, (int, float)) else -1)
        s_now = rig.bb.get(P_ROUTE_S)
        s_track.append(float(s_now) if isinstance(s_now, (int, float)) else -1.0)
        p = rig.bb.get(P_POSE)
        yaw = p.yaw + (v / wheelbase) * math.tan(math.radians(steer)) * dt
        x = p.x + v * math.cos(yaw) * dt
        y = p.y + v * math.sin(yaw) * dt
        rig.set_pose(x, y, yaw)
        rig.clock.advance(dt)

    def _flips(seq: list[int]) -> int:
        n, prev = 0, 0
        for sg in seq:
            if sg and prev and sg != prev:
                n += 1
            if sg:
                prev = sg
        return n

    check("9.1 ★沿程单调推进到接近末端（≥2.30 m，全长 2.45 m）",
          max(s_track) >= 2.30, f"max s={max(s_track):.3f}")
    check("9.2 ★命令符号翻转 ≤5 次/30s（无地板时 715 次）",
          _flips(signs) <= 5, f"{_flips(signs)} 次")
    check("9.3 ★段号不来回跳（切换次数 ≤ 段数 7）",
          sum(1 for i in range(1, len(segs)) if segs[i] != segs[i - 1]) <= 7,
          f"切换={sum(1 for i in range(1, len(segs)) if segs[i] != segs[i-1])}")
    check("9.4 ★末 5 秒已收敛（≤2 次翻转）", _flips(signs[-250:]) <= 2,
          f"尾 250 拍翻转 {_flips(signs[-250:])} 次")
    check("9.5 倒车段确实走过（出现过反向指令）", any(sg < 0 for sg in signs),
          f"速度集合={sorted({round(c[0], 3) for c in rig.bridge.commands})}")


def test_projection_relock() -> None:
    """★10. 投影「跟丢」必须能自愈 —— 窗口/地板不得把错误的 s 变成自洽不动点。

    bench 实测的后果（本轮抓到的最严重一条）：tracker 的 s 冻在第 16 段 **7.70**，
    而车实际已在第 32 段 **16.30**（同一个信念位姿、Localization 与 Tracker 两个写入者
    差 **8.6 m**）；tracker 于是追着幻影目标开 ——
    虚拟小车**最大横向偏差 4.101 m**、只走到第 18/34 点、`定位` 侧还报了 4 次"投影卡死自愈"。

    机制：`s_hint` 每拍跟随上一拍的 s ⇒ 窗口（±0.6 m，外加地板）把错误的 s
    变成**自洽的不动点**，车走多远都拉不回来。仿真的 `_project_with_floor` 一直有
    `release_dist` 出口，实车代码此前没有。

    本组构造同样的跟丢：先在顶点处把地板抬起来（投影链"锚"在前段），再把车**远距离
    挪到路线后段**（模拟锚定/位姿跳变打断窗口链），断言投影能全路线重投影恢复。
    """
    section("10. ★投影跟丢自愈（窗口/地板不得把错的 s 变成不动点）")
    cfg = cfg_of(pivot_enabled=False, bridge_blocks=[])
    tokens = ["F10", "F11", "E11", "R:F11", "F10", "F8", "B8", "B6"]
    route = parse_mission(tokens)
    path = Path(route)
    p0 = geo.block_to_meters("F10")
    p1 = geo.block_to_meters("F11")
    ux, uy = p1[0] - p0[0], p1[1] - p0[1]
    d = math.hypot(ux, uy)
    ux, uy = ux / d, uy / d
    rig = Rig(cfg=cfg, route=route, pose=(p0[0], p0[1], geo.heading_of(ux, uy)))
    rig.ticks(40)                                  # 先开一段，让投影链/地板建立起来
    s_before = rig.bb.get(P_ROUTE_S)
    check("10.1 前置：已建立投影（s 有值且在地板之前）",
          isinstance(s_before, (int, float)) and float(s_before) < 1.0, f"s={s_before}")

    # ★ 远距离瞬移到路线**后段**（B6 = 末点），模拟锚定/位姿跳变把窗口链打断
    b6 = geo.block_to_meters("B6")
    rig.set_pose(b6[0], b6[1], geo.heading_of(-1.0, 0.0))
    rig.ticks(3)
    s_after = rig.bb.get(P_ROUTE_S)
    dist_after = getattr(rig.task, "_last_cmd", {}).get("dist_to_path")
    check("10.2 ★跟丢后 s 恢复到后段（≥2.0 m，路线全长 2.45 m）",
          isinstance(s_after, (int, float)) and float(s_after) >= 2.0,
          f"跟丢前 s={s_before} → 跟丢后 s={s_after}")
    check("10.3 ★恢复后离线很小（≤0.15 m；不恢复时会 ≥1 m）",
          isinstance(dist_after, (int, float)) and float(dist_after) <= 0.15,
          f"dist_to_path={dist_after}")
    check("10.4 自愈计数器被记录（可观测，便于上车排障）",
          getattr(rig.task, "projection_relocks", 0) >= 1,
          f"projection_relocks={getattr(rig.task, 'projection_relocks', None)}")
    check("10.5 自愈日志可查（含「跟丢」字样）",
          any("跟丢" in ln for ln in rig.logs), f"logs_tail={rig.logs[-2:]}")


def test_motion_limit_wiring() -> None:
    """★11. 限速请求接线（卡 t21）：`P_MOTION_LIMIT` 必须真的钳到**指令速度**。

    分工：读侧实现在 `pursuit/controller.py`（卡 t20：`motion_limit_speed_limit` +
    `compute_command(..., bb=...)`）；**本组验的是"接线"** —— Tracker 把 `self.bb` 传进
    `compute_command`，于是 Localization 的限速/停机请求真能生效。

    ★ 负向控制（证明断言有牙）：把 `tasks/tracker.py` 里 `compute_command(...)` 的
      `bb=self.bb` 注释掉，本组 **11.2 / 11.3 / 11.6 必红**（实测 78/81、exit 1；
      11.1/11.1b/11.4/11.5 仍绿 —— 它们是"无信号 no-op"的正向保证，本就不依赖接线）。
      实测输出：`_tmp_session/t21/neg_control_broken.txt`。
    """
    section("11. 限速请求接线：P_MOTION_LIMIT 钳速 + 无信号 no-op")
    cfg = cfg_of(cruise_speed_mps=0.30, max_steer_deg=30.0)
    # 路线要足够长：否则末点落在 `pursuit.approach_m` 窗口内，速度由 `stop_approach` 主导，
    # 就看不出"巡航 0.30 被钳到 0.08"这件事了（第一版踩过：实测 0.156/stop_approach）。
    route = ("N10", "M10", "L10", "K10", "J10")
    pose = (0.525, 0.355, geo.HEADING_Y)      # 对准首段直路（与 2.5 同构）
    keys = ("speed", "steer_deg", "speed_reason")

    def trip(ticks: int = 5, payload=...):
        """独立一台车跑 `ticks` 拍；`payload` 省略 = 键根本不存在。"""
        rig = Rig(cfg=cfg, route=route_of(*route), pose=pose)
        if payload is not ...:
            rig.bb.set(P_MOTION_LIMIT, payload, "Localization")
        rig.ticks(ticks)
        return rig

    # 11.1 无信号两种形态（键不存在 / 键=None）逐位一致
    base = trip()
    none_rig = trip(payload=None)
    b0, n0 = base.cmd(), none_rig.cmd()
    check("11.1 ★键不存在 vs 值=None：三条命令字段逐位一致（无信号 no-op）",
          tuple(b0.get(k) for k in keys) == tuple(n0.get(k) for k in keys),
          f"{[b0.get(k) for k in keys]} vs {[n0.get(k) for k in keys]}")
    check("11.1b 无信号时速度 = 巡航 0.30（前置事实，reason=cruise）",
          abs(float(b0.get("speed", -1)) - 0.30) < 1e-9 and b0.get("speed_reason") == "cruise",
          f"v={b0.get('speed')!r} reason={b0.get('speed_reason')!r}")

    # 11.2 ★接线生效：0.08 的限速必须把 0.30 的巡航钳下来
    lim = trip(payload={"speed_mps": 0.08, "reason": "drift", "drift_run": 5})
    c1 = lim.cmd()
    check("11.2 ★有信号 0.08 → 指令速度被钳到 0.08（接线生效）",
          abs(float(c1.get("speed", -1)) - 0.08) < 1e-9 and c1.get("speed_reason") == "motion_limit",
          f"无信号 {b0.get('speed')!r} → 有信号 {c1.get('speed')!r} "
          f"reason={c1.get('speed_reason')!r}")

    # 11.3 ★停机请求 0.0 必须真给 0（不被 pursuit.min_speed_mps 蠕动下限顶回）
    lim.bb.set(P_MOTION_LIMIT, {"speed_mps": 0.0, "reason": "boundary_stop"}, "Localization")
    lim.ticks(1)
    c2 = lim.cmd()
    check("11.3 ★停机请求 0.0 → 指令速度 0.0（reason=motion_limit）",
          c2.get("speed") == 0.0 and c2.get("speed_reason") == "motion_limit",
          f"v={c2.get('speed')!r}（min_speed={cfg.get('pursuit.min_speed_mps')}）"
          f" reason={c2.get('speed_reason')!r}")

    # 11.4 撤回（写 None）→ 回到巡航（接线置空即恢复，不会残留限速）
    lim.bb.set(P_MOTION_LIMIT, None, "Localization")
    lim.ticks(1)
    c3 = lim.cmd()
    check("11.4 撤回信号（None）→ 回到巡航 0.30",
          abs(float(c3.get("speed", -1)) - 0.30) < 1e-9,
          f"v={c3.get('speed')!r} reason={c3.get('speed_reason')!r}")

    # 11.5 ★非法载荷（半个信号）→ 与无信号逐位一致，绝不把车锁死
    bad = trip(payload={"speed_mps": "x", "reason": "drift"})
    c4 = bad.cmd()
    check("11.5 ★非法载荷 → 三条字段逐位等于无信号（不锁死）",
          tuple(c4.get(k) for k in keys) == tuple(b0.get(k) for k in keys),
          f"{[c4.get(k) for k in keys]} vs {[b0.get(k) for k in keys]}")

    # 11.6 倒车段按绝对值钳且保号（Tracker 的 reverse 分支也要受限速管）
    from dzbrain.route.model import make_route, Waypoint
    rev_cfg = cfg_of(cruise_speed_mps=0.15, reverse_speed_ratio=0.6, reverse_steer_sign=1.0)
    rev = Rig(cfg=rev_cfg,
              route=make_route([Waypoint.at_block("B5"), Waypoint.at_block("B6", reverse=True),
                                Waypoint.at_block("B7", reverse=True)]))
    rev.set_pose(2.1, 4.375, geo.rad(180.0))          # 与 §4 同构：倒着往 B6 走，倒车速度 -0.09
    rev.bb.set(P_MOTION_LIMIT, {"speed_mps": 0.05, "reason": "drift"}, "Localization")
    rev.ticks(1)
    c5 = rev.cmd()
    check("11.6 倒车：|v| ≤ 0.05 且保号（负）",
          -0.05 - 1e-12 <= float(c5.get("speed", 0.0)) < 0.0,
          f"v={c5.get('speed')!r} reason={c5.get('speed_reason')!r}")


def main() -> int:
    print("=" * 72)
    print("dzbrain TrackerTask 自测（纯脚本）")
    print("=" * 72)
    test_gates()
    test_normal()
    test_bridge_align()
    test_reverse()
    test_stop_approach()
    test_path_cache()
    test_overlap_no_flipflop()
    test_arc_floor_no_oscillation()
    test_projection_relock()
    test_motion_limit_wiring()
    test_rate()
    finish()
    return 0


if __name__ == "__main__":
    sys.exit(main())
