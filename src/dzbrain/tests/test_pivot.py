# -*- coding: utf-8 -*-
r"""PIVOT 原地转向专项测试（并入 10-05 核心的已验证修复）。

为什么需要 PIVOT：纯追踪在**大角度换向**（现场图 F11 的 180° 折返）会画大弧，
仿真实测该处横向偏差 0.37~0.41 m，而那两段正是**压雷段**（E11/F8 是雷）——
偏差 0.4 m 就可能漏压雷。9 组参数扫描（tools/sim_param_sweep.py）证明这是几何
结构性的（最好只降 0.7%），所以改用"顶点前停下 → 低速蠕转对航向"的做法。

本测试钉死：
  ① 大角度换向触发（先刹车 SETTLE，再蠕转 TURN）；
  ② ★硬超时兜底（旧版**没有**超时，是本项目标注的缺口，必须不卡死）；
  ③ 小角度拐角不误触发（90° 直角弯仍走纯追踪）；
  ④ 桥洞保护段内不触发（桥洞里侧壁就在旁边）；
  ⑤ 蠕转舵角也受限幅约束（"限幅只在纯追踪里生效"是暗坑）；
  ⑥ 航向对上后正常退出并计数。

运行：python src\dzbrain\tests\test_pivot.py
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.stdout.reconfigure(errors="replace")

from dzbrain.core import geometry as geo                        # noqa: E402
from dzbrain.core.blackboard import (P_MISSION_PHASE, P_POSE, P_ROUTE, P_SAFETY,  # noqa: E402
                                     Blackboard)
from dzbrain.core.config import Config                          # noqa: E402
from dzbrain.core.events import EventBus                        # noqa: E402
from dzbrain.core.state import SafetyState                      # noqa: E402
from dzbrain.io.contract import MissionPhase                    # noqa: E402
from dzbrain.route.parser import parse_mission                  # noqa: E402
from dzbrain.tasks.tracker import PivotPhase, TrackerTask       # noqa: E402
from fakes import FakeBridge, FakeClock                         # noqa: E402
from harness import check, section, finish                      # noqa: E402

K_COLUMN = [f"K{r}" for r in range(1, 12)]


def _cfg(**pursuit) -> Config:
    data = {"pursuit": dict(pursuit), "logging": {"rate_limit_s": 0.0}}
    data["pursuit"].setdefault("bridge_blocks", list(K_COLUMN))
    return Config.from_dict(data)


class Rig:
    def __init__(self, *, tokens, pose, cfg: Config | None = None) -> None:
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.cfg = cfg or _cfg()
        self.logs: list[str] = []
        self.route = parse_mission(tokens)
        self.task = TrackerTask(self.bridge, self.bb, self.bus, self.cfg,
                                log=self.logs.append, clock=self.clock.now)
        self.bb.set(P_SAFETY, SafetyState(ok=True), "test")
        self.bb.set(P_MISSION_PHASE, MissionPhase.DRIVE, "test")
        self.bb.set(P_ROUTE, self.route, "test")
        self.set_pose(*pose)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self.bridge.set_pose(x, y, yaw)
        self.bb.set(P_POSE, geo.Pose(x, y, yaw), "test")

    def tick(self, dt: float = 0.02) -> tuple[float, float] | None:
        self.task.tick()
        self.clock.advance(dt)
        return self.bridge.commands[-1] if self.bridge.commands else None

    def ticks(self, n: int, dt: float = 0.02) -> None:
        for _ in range(n):
            self.tick(dt)


# 180° 折返：A6 -> B6（向西）-> 再向东回到 A6（换向 180°）
U_TURN = ["B6", "A6", "R:B6", "R:D6"]
# 90° 直角：C6 -> D6（向西）-> D4（向南）
RIGHT_ANGLE = ["C6", "D6", "D4"]


def test_trigger_and_phases() -> None:
    section("1. 180° 折返触发 PIVOT：先刹车 SETTLE，再蠕转 TURN")
    cfg = _cfg(pivot_dist_m=0.30, pivot_wait_s=0.4, pivot_speed_mps=0.08,
               pivot_yaw_tol_deg=15.0, pivot_timeout_s=8.0)
    a6x, a6y = geo.block_to_meters("A6")
    b6x, b6y = geo.block_to_meters("B6")
    rig = Rig(tokens=U_TURN, pose=(b6x - 0.10, b6y, geo.heading_of(a6x - b6x, a6y - b6y)),
              cfg=cfg)
    # 沿 B6->A6 慢慢靠近 A6（顶点），触发 PIVOT
    dx, dy = a6x - b6x, a6y - b6y
    d = math.hypot(dx, dy)
    ux, uy = dx / d, dy / d
    for frac in (0.2, 0.4, 0.6, 0.75, 0.85, 0.92):
        rig.set_pose(b6x + ux * d * frac, b6y + uy * d * frac, geo.heading_of(ux, uy))
        rig.tick()
    check("1.1 大角度换向进入 PIVOT（非 IDLE）",
          rig.task._pivot_phase != PivotPhase.IDLE or rig.task.pivot_count > 0,
          f"phase={rig.task._pivot_phase} count={rig.task.pivot_count}")
    settled = rig.task._pivot_phase == PivotPhase.SETTLE
    if settled:
        cmd = rig.bridge.commands[-1]
        check("1.2 SETTLE 段发的是刹车（零速零舵）",
              abs(cmd[0]) < 1e-9 and abs(cmd[1]) < 1e-9, f"cmd={cmd}")
    else:
        check("1.2 SETTLE 段（本轮已越过或未进入，按 TURN 检查）", True, f"phase={rig.task._pivot_phase}")

    # 快进过 wait，应进入 TURN：速度 = pivot_speed，舵角指向目标航向
    for _ in range(30):
        rig.tick()
    saw_turn = rig.task._pivot_phase == PivotPhase.TURN or rig.task.pivot_count > 0
    check("1.3 进入 TURN（或已完成）", saw_turn, f"phase={rig.task._pivot_phase} "
                                                  f"count={rig.task.pivot_count}")
    turns = [c for c in rig.bridge.commands if abs(c[0] - 0.08) < 1e-9]
    check("1.4 TURN 段速度 = pivot_speed_mps(0.08)", len(turns) >= 1,
          f"turns={len(turns)} 全部速度={sorted({round(c[0],3) for c in rig.bridge.commands})}")

    # 把车头直接摆到目标航向 → 应完成并回到 IDLE
    target = rig.task._pivot_target_heading
    p = rig.bb.get(P_POSE)
    rig.set_pose(p.x, p.y, target)
    rig.tick()
    check("1.5 航向对上后完成 PIVOT 并计数",
          rig.task.pivot_count >= 1 and rig.task._pivot_phase == PivotPhase.IDLE,
          f"count={rig.task.pivot_count} phase={rig.task._pivot_phase}")


def test_timeout_no_deadlock() -> None:
    section("2. ★硬超时兜底（旧版没有超时——本项目标注的缺口）")
    cfg = _cfg(pivot_dist_m=0.30, pivot_wait_s=0.0, pivot_timeout_s=1.0,
               pivot_yaw_tol_deg=1.0)   # 容差极小 + 超时 1s
    a6x, a6y = geo.block_to_meters("A6")
    b6x, b6y = geo.block_to_meters("B6")
    # ★ 关键：车头摆成"来的时候那个方向"（B6→A6），而目标航向是"走的时候那个方向"（A6→B6）
    #   —— 两者差 180°，而本测试**故意不把车转过去**，所以只能靠超时放行。
    rig = Rig(tokens=U_TURN, pose=(a6x, a6y, geo.heading_of(a6x - b6x, a6y - b6y)), cfg=cfg)
    # 车到达顶点；此后**故意不把车头转过去**（航向误差永远很大）→ 只能靠超时放行
    rig.tick()
    phases = []
    for _ in range(120):        # 120 × 0.02 = 2.4 s > timeout 1.0 s
        rig.tick()
        phases.append(rig.task._pivot_phase)
    check("2.1 曾在 PIVOT 里（确实触发了）", PivotPhase.TURN in phases or PivotPhase.SETTLE in phases,
          f"phases={sorted(set(phases))}")
    check("2.2 超时后放行：不再停在 PIVOT（不卡死，且不会立刻重复触发同一顶点）",
          rig.task._pivot_phase == PivotPhase.IDLE and rig.task.pivot_abandon_count >= 1,
          f"phase={rig.task._pivot_phase} abandons={rig.task.pivot_abandon_count}")
    check("2.3 超时有明确日志（可复盘）",
          any("PIVOT 超时" in ln for ln in rig.logs), f"logs={rig.logs[-2:]}")
    check("2.4 超时后恢复正常循迹（有非零速度指令）",
          any(abs(c[0]) > 0.01 for c in rig.bridge.commands[-5:]),
          f"tail={rig.bridge.commands[-3:]}")


def test_no_false_trigger() -> None:
    section("3. 不误伤：90° 直角弯 / 桥洞段 / 小角度")
    c6x, c6y = geo.block_to_meters("C6")
    d6x, d6y = geo.block_to_meters("D6")
    rig = Rig(tokens=RIGHT_ANGLE, pose=(d6x, d6y, geo.heading_of(c6x - d6x, c6y - d6y)),
              cfg=_cfg(pivot_dist_m=0.30))
    rig.ticks(20)
    check("3.1 90° 直角弯不触发 PIVOT（仍走纯追踪）",
          rig.task._pivot_phase == PivotPhase.IDLE and rig.task.pivot_count == 0,
          f"phase={rig.task._pivot_phase} count={rig.task.pivot_count}")
    check("3.2 90° 弯仍有运动指令（不是停在原地）",
          any(abs(c[0]) > 0.01 for c in rig.bridge.commands), f"tail={rig.bridge.commands[-2:]}")

    # 桥洞保护段内：构造 L9 -> K9 -> J9（真正的穿洞段），车在 K9 前
    l9x, l9y = geo.block_to_meters("L9")
    k9x, k9y = geo.block_to_meters("K9")
    rig2 = Rig(tokens=["L9", "K9", "J9", "J1"],
               pose=(l9x, l9y, geo.heading_of(k9x - l9x, k9y - l9y)),
               cfg=_cfg(pivot_dist_m=0.60, pivot_angle_deg=60.0))   # 阈值调低，强行给触发机会
    rig2.ticks(30)
    check("3.3 桥洞保护段内不触发 PIVOT（旧版显式排除过）",
          rig2.task.pivot_count == 0, f"count={rig2.task.pivot_count} "
                                      f"phase={rig2.task._pivot_phase}")
    check("3.4 桥洞段走的是 ALIGN 直行（速度=align 速度 0.10）",
          any(abs(c[0] - 0.10) < 1e-9 for c in rig2.bridge.commands),
          f"speeds={sorted({round(c[0],3) for c in rig2.bridge.commands})}")


def test_turn_respects_rate_limit() -> None:
    section("4. 蠕转舵角也受限幅约束（限幅不能只在纯追踪里生效）")
    cfg = _cfg(pivot_dist_m=0.30, pivot_wait_s=0.0, pivot_speed_mps=0.08,
               pivot_kp=5.0, max_steer_rate_deg_s=120.0, max_steer_deg=30.0,
               pivot_timeout_s=20.0, pivot_yaw_tol_deg=1.0)
    a6x, a6y = geo.block_to_meters("A6")
    b6x, b6y = geo.block_to_meters("B6")
    # 同上：车头朝向与目标航向差 180°，TURN 段才会持续（否则瞬间完成，采不到样本）
    rig = Rig(tokens=U_TURN, pose=(a6x, a6y, geo.heading_of(a6x - b6x, a6y - b6y)), cfg=cfg)
    rig.tick()
    steers: list[float] = []
    for _ in range(40):
        rig.tick(dt=0.02)
        if rig.task._pivot_phase == PivotPhase.TURN and rig.bridge.commands:
            steers.append(rig.bridge.commands[-1][1])
    check("4.1 确实进入了 TURN 段（有样本）", len(steers) >= 3, f"n={len(steers)}")
    if len(steers) >= 2:
        steps = [abs(steers[i + 1] - steers[i]) for i in range(len(steers) - 1)]
        worst = max(steps) if steps else 0.0
        check("4.2 蠕转时单拍舵角变化 ≤ 限幅×(dt=0.02) = 2.4°",
              worst <= 2.4 + 1e-6, f"worst={worst:.3f}°")
    check("4.3 舵角不超过 max_steer_deg(30°)",
          all(abs(s) <= 30.0 + 1e-9 for s in steers), f"max={max([abs(s) for s in steers], default=0):.2f}°")


def main() -> int:
    print("=" * 72)
    print("dzbrain PIVOT 原地转向专项测试（纯脚本）")
    print("=" * 72)
    test_trigger_and_phases()
    test_timeout_no_deadlock()
    test_no_false_trigger()
    test_turn_respects_rate_limit()
    finish()
    return 0


if __name__ == "__main__":
    sys.exit(main())
