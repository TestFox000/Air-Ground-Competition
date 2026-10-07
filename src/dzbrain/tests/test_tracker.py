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
from dzbrain.core.blackboard import (P_MISSION_PHASE, P_POSE, P_ROUTE, P_ROUTE_INDEX,  # noqa: E402
                                     P_ROUTE_S, P_SAFETY, Blackboard)
from dzbrain.core.config import Config                         # noqa: E402
from dzbrain.core.events import EventBus                       # noqa: E402
from dzbrain.core.state import SafetyState                     # noqa: E402
from dzbrain.io.contract import MissionPhase                   # noqa: E402
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
    test_rate()
    finish()
    return 0


if __name__ == "__main__":
    sys.exit(main())
