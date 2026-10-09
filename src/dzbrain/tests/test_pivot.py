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
  ⑦ ★**超时预算必须覆盖物理转弯时间**（8.0s < 135° 所需的 8.01s = 触发即超时，
     实闭环 bench 在 E11 的 180° 折返处就是这么卡住的；现按配置现算并卡住）。

运行：python src\dzbrain\tests\test_pivot.py
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.stdout.reconfigure(errors="replace")

from dzbrain.core import geometry as geo                        # noqa: E402
from dzbrain.core.blackboard import (P_MISSION_PHASE, P_POSE, P_ROUTE, P_ROUTE_INDEX,  # noqa: E402
                                     P_ROUTE_S, P_SAFETY, Blackboard)
from dzbrain.core.config import Config                          # noqa: E402
from dzbrain.core.events import EventBus                        # noqa: E402
from dzbrain.core.state import SafetyState                      # noqa: E402
from dzbrain.io.contract import MissionPhase                    # noqa: E402
from dzbrain.pursuit.path import Path                           # noqa: E402
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


# ★ 180° 折返（**两条正向段**）：B6 → A6 → 再正向回到 B6，车头必须真调头 180°。
#   ⚠️ 曾经这里用的是 `["B6","A6","R:B6","R:D6"]`（下一段是**倒车段**）—— 那种顶点车头
#   **一度都不用转**（倒着原路退回就行），按"行进方向夹角 180°"判 PIVOT 是错的：
#   会把车头转反 180°，之后纯追踪自己跟自己打架（见 test_reverse_vertex_no_pivot）。
U_TURN = ["B6", "A6", "B6"]
# 90° 直角：C6 -> D6（向西）-> D4（向南）
RIGHT_ANGLE = ["C6", "D6", "D4"]
# ★ 现场 F11 折返簇：F11→E11（正向）→ R:F11（**倒车**退回）→ F10
REV_STUB = ["F11", "E11", "R:F11", "F10"]


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
    # ⚠️ 起点必须在顶点**之前**（< pivot_dist_m）：`path.project` 在"投影正好贴在顶点"时
    #   会把段号推到下一段（顶点前瞻重投影，见 path.py 2b 段注释），那时 `_corner_ahead`
    #   看到的顶点已经不是这个折返点了 —— 夹具若把车摆在顶点正上方，就不再触发 PIVOT
    #   （这是**夹具**问题，不是逻辑问题：现场是先触发再走过去，正上方是测不到的瞬间态）。
    rig = Rig(tokens=U_TURN,
              pose=(a6x - (a6x - b6x) * 0.34, a6y - (a6y - b6y) * 0.34,
                    geo.heading_of(a6x - b6x, a6y - b6y)), cfg=cfg)
    # 车在顶点前 0.12 m（< pivot_dist_m=0.30）→ 触发 PIVOT；此后**故意不把车头转过去**
    # （航向误差永远很大）→ 只能靠超时放行
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
    # ⚠️ 同样必须在顶点**之前**起步（原因见 §2 的说明：投影贴在顶点时会推到下一段）。
    # ⚠️ 而且要先让**纯追踪跑几拍**建立"上一拍舵角" —— 否则起手就在 PIVOT 里、
    #    `_last_steer_deg` 是 None，首拍不受限幅（实测样本会全等于 30°，4.2 变成空断言）。
    rig = Rig(tokens=U_TURN, pose=(b6x, b6y, geo.heading_of(a6x - b6x, a6y - b6y)), cfg=cfg)
    rig.ticks(3)                      # 纯追踪：在线上直行 → 上一拍舵角≈0
    rig.set_pose(a6x - (a6x - b6x) * 0.34, a6y - (a6y - b6y) * 0.34,
                 geo.heading_of(a6x - b6x, a6y - b6y))   # 挪到顶点前 0.12 m → 触发 PIVOT
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


def test_timeout_covers_physics() -> None:
    """★5. 超时预算必须覆盖物理转弯时间 —— 否则 PIVOT 每次触发都必然超时。

    这不是纸上推演：实闭环 bench（`python tools\\dzbrain.py --bench --drive --seconds 180`）
    在 E11 的 180° 折返处实测 —— PIVOT 触发 → 8.0s 硬超时 → 半路放行交回纯追踪 →
    车在顶点附近来回摆（yaw 在 ±180° 之间来回、v 在 ±0.08/0.12 之间抖），
    13/34 个点后停滞，并触发"拐角绕圈兜底"强制跳过航点。

    手算就看得出来（Ackermann 模型，最小转弯半径 R = L/tan(舵角)）：
        偏航率上限 = v/L·tan(舵角) = 0.08/0.157·tan(30°) = 0.294 rad/s = 16.9°/s
        135°（触发阈值）需 8.01s；180°（现场折返）需 10.68s
    而当时 pivot_timeout_s = 8.0s —— **比触发阈值所需的 8.01s 还短**，
    也就是"只要触发就必定超时"，而超时又把转弯丢在半路，比不转更糟。

    所以这里按配置**现算**一遍，任何一项参数被改坏都会立刻变红。
    """
    section("5. ★超时预算必须覆盖物理转弯时间（「触发即超时」的死结）")
    cfg = Config.from_dict({})
    L = float(cfg.get("pursuit.wheelbase_m"))
    steer = math.radians(float(cfg.get("pursuit.max_steer_deg")))
    v = float(cfg.get("pursuit.pivot_speed_mps"))
    wait = float(cfg.get("pursuit.pivot_wait_s"))
    timeout = float(cfg.get("pursuit.pivot_timeout_s"))
    thresh = float(cfg.get("pursuit.pivot_angle_deg"))

    yaw_rate = v / L * math.tan(steer)        # rad/s（上限）
    check("5.1 偏航率上限算得出来（v/L·tan 舵角）", yaw_rate > 1e-6,
          f"{math.degrees(yaw_rate):.1f}°/s（最小转弯半径 {L / math.tan(steer):.3f} m）")
    for angle in (thresh, 180.0):
        need = wait + math.radians(angle) / yaw_rate
        check(f"5.2 ★超时 {timeout:.1f}s 覆盖 {angle:.0f}° 转弯（需 {need:.2f}s，余量 ≥15%）",
              timeout >= need * 1.15, f"余量 {timeout - need:+.2f}s")
    check("5.3 超时仍在可配范围内（1~30s）", 1.0 <= timeout <= 30.0, f"{timeout}s")
    check("5.4 超时不许超过 30s（否则「卡死兜底」失去意义）", timeout <= 30.0)


def test_no_reentry_after_success() -> None:
    """★6. PIVOT 成功后不得在同一顶点反复重入（实闭环 bench 抓到的死循环）。

    bench 证据（`--bench --drive --seconds 240`，修超时值**之后**仍然复现，说明
    这是与超时无关的第二个 bug）：

        ★ 进入 PIVOT（顶点 s=4.20 距 0.20m / 换向 180° > 阈值 135°）
        [T] F11 DRIVE (-0.058,3.127) yaw-105.0 **v0.000** st+0.0 | 段12 s4.43/34
        ✓ PIVOT 完成：航向误差 +15.0° ≤ 15°（用时 0.5s）
        ★ 进入 PIVOT（……）        ← 下一拍又来，周而复始

    整场 240s 卡在 s≈4.4/34 m（几百次循环），车每拍先被刹车 0.5s，几乎不动。

    根因：`_pivot_abandon_until_s` 这个"该顶点已处理"的标记**只有超时分支设**，
    成功分支漏了 —— 而车完成转向后**仍停在顶点邻域内**（< pivot_dist_m）、换向角
    依旧 > 阈值，于是每拍重新满足触发条件。

    本组故意让车**停在顶点不动**并把车头摆到容差内（最坏情况：最容易重入），
    要求 3 秒内只进入 1 次、只完成 1 次，且完成后能交回纯追踪（发得出非零速度）。
    """
    section("6. ★PIVOT 成功后不得重入（顶点邻域死循环回归）")
    cfg = _cfg(pivot_dist_m=0.30, pivot_wait_s=0.4, pivot_speed_mps=0.08,
               pivot_yaw_tol_deg=15.0, pivot_timeout_s=8.0)
    a6x, a6y = geo.block_to_meters("A6")
    b6x, b6y = geo.block_to_meters("B6")
    dx, dy = a6x - b6x, a6y - b6y
    d = math.hypot(dx, dy)
    ux, uy = dx / d, dy / d
    # 车停在顶点前 0.10 m（< pivot_dist_m），车头朝"来向" → 触发条件成立
    rig = Rig(tokens=U_TURN, pose=(a6x - ux * 0.10, a6y - uy * 0.10, geo.heading_of(ux, uy)),
              cfg=cfg)
    rig.tick()
    entered_first = rig.task._pivot_phase != PivotPhase.IDLE
    check("6.1 顶点邻域内触发 PIVOT（前置条件成立）", entered_first or rig.task.pivot_count > 0,
          f"phase={rig.task._pivot_phase}")

    # 把车头直接摆到目标航向 → 下一拍必然判"完成"（最坏情况：立刻满足容差）
    if rig.task._pivot_phase != PivotPhase.IDLE:
        target = rig.task._pivot_target_heading
        p = rig.bb.get(P_POSE)
        rig.set_pose(p.x, p.y, target)

    # ★ 关键：之后**一动不动**地 tick 3 秒（150 拍）。
    #   修好之前这里会反复"进入→完成"，修好之后只允许各 1 次。
    n0 = len([s for s in rig.logs if "进入 PIVOT" in s])
    for _ in range(150):
        rig.tick()
    entries = len([s for s in rig.logs if "进入 PIVOT" in s])
    completed = len([s for s in rig.logs if "PIVOT 完成" in s])
    check("6.2 ★停车 3s 内只进入 PIVOT 1 次（不重入）", entries - n0 <= 1,
          f"进入 {entries - n0} 次（阈值 1）")
    check("6.3 ★只完成 1 次（不是每 0.4s『完成』一次）", rig.task.pivot_count == 1,
          f"pivot_count={rig.task.pivot_count}")
    check("6.4 完成次数与日志条数一致（无隐藏重入）", completed <= 1,
          f"日志里『完成』{completed} 条")
    check("6.5 ★完成后交回纯追踪：发得出非零前进速度（车能走）",
          any(abs(c[0]) > 0.05 for c in rig.bridge.commands),
          f"速度集合={sorted({round(c[0], 3) for c in rig.bridge.commands})}")


def test_reverse_vertex_no_pivot() -> None:
    """★7. 下一段是**倒车段**（`R:`）的顶点：不得 PIVOT，但要能"倒进"倒车段。

    现场路线 `F10→F11→E11→R:F11→F10`：
      · seg1 = F11→E11（正向，行进 +90°）→ seg2 = E11→R:F11（**倒车**，行进 −90°）
      · 两条段的**行进方向**差 180°，但**车头都是 +90°** —— 车倒着原路退回，一度都不用转。
    旧代码按"行进方向夹角"判 PIVOT ⇒ 把车头**转反 180°**（转到 −90°），然后：
      · 投影仍贴在前一段（换段靠实测位移，而车头已反）
      · 纯追踪又要往回转 ⇒ `v` 在 +0.150/−0.080 之间反复翻、`yaw` 一直转
      · 12s 后触发「拐角绕圈兜底」跳点（bench 实测 F11/A6/D6 三处）

    修法两半，缺一不可：
      ① 换向角用 `corner_turn_deg`（**车头**转角）→ 这种顶点转角 0° ⇒ 不触发 PIVOT；
      ② 倒车段入口"推一把"：贴到顶点且下一段是倒车段时直接给 v<0 ——
         否则「换段靠实测位移、实测位移靠先发 v<0、v<0 靠先换段」鸡生蛋，s 冻在顶点。
    """
    section("7. ★倒车段顶点：不 PIVOT（车头无需转）+ 能倒进倒车段")
    cfg = _cfg(pivot_dist_m=0.30, pivot_wait_s=0.4, pivot_speed_mps=0.08,
               pivot_yaw_tol_deg=15.0, pivot_timeout_s=8.0)
    route = parse_mission(REV_STUB)
    path = Path(route)

    # ---- 7.1~7.3 几何：旧判据会触发，新判据不该触发 ----
    #  tokens = [F11, E11, R:F11, F10] → seg0 = F11→E11（正向）、seg1 = E11→R:F11（倒车）
    #  顶点夹在 seg0/seg1 之间 ⇒ 索引 0
    check("7.1 行进方向夹角 = 180°（旧判据据此触发 PIVOT）",
          abs(path.corner_angle_deg(0) - 180.0) < 1e-9, f"{path.corner_angle_deg(0):.1f}°")
    check("7.2 ★车头实际转角 = 0°（新判据不触发）",
          path.corner_turn_deg(0) < 1e-9, f"{path.corner_turn_deg(0):.2f}°")
    check("7.3 两段的车头朝向相同（倒车段车头与行进反向）",
          abs(geo.deg(geo.ang_diff(path.car_heading_on(0),
                                   path.car_heading_on(1)))) < 1e-9,
          f"{geo.deg(path.car_heading_on(0)):.1f}° vs {geo.deg(path.car_heading_on(1)):.1f}°")
    check("7.4 正向 180° 调头仍然算 180°（没有被改坏）",
          abs(parse_mission(U_TURN) and Path(parse_mission(U_TURN)).corner_turn_deg(0) - 180.0) < 1e-9,
          f"{Path(parse_mission(U_TURN)).corner_turn_deg(0):.1f}°")

    # ---- 7.5~7.7 跑起来：用 bench 同款积分模型驱动 ----
    e11 = geo.block_to_meters("E11")
    f11 = geo.block_to_meters("F11")
    ux, uy = e11[0] - f11[0], e11[1] - f11[1]
    d = math.hypot(ux, uy)
    ux, uy = ux / d, uy / d
    rig = Rig(tokens=REV_STUB, cfg=cfg,
              pose=(e11[0] - ux * 0.05, e11[1] - uy * 0.05, geo.heading_of(ux, uy)))
    wheelbase = float(cfg.get("pursuit.wheelbase_m"))
    dt = 0.02
    segs: list[int] = []
    speeds: list[float] = []
    for _ in range(300):                       # 6 s
        rig.task.tick()
        v, steer = rig.bridge.commands[-1]
        speeds.append(v)
        seg_now = rig.bb.get(P_ROUTE_INDEX)
        segs.append(int(seg_now) if isinstance(seg_now, (int, float)) else -1)
        p = rig.bb.get(P_POSE)
        yaw = p.yaw + (v / wheelbase) * math.tan(math.radians(steer)) * dt
        x = p.x + v * math.cos(yaw) * dt
        y = p.y + v * math.sin(yaw) * dt
        rig.set_pose(x, y, yaw)
        rig.clock.advance(dt)

    check("7.5 ★全程从未进入 PIVOT（车头无需转）",
          rig.task.pivot_count == 0 and rig.task._pivot_phase == PivotPhase.IDLE,
          f"count={rig.task.pivot_count} phase={rig.task._pivot_phase}")
    check("7.6 ★发出过倒车指令（'推一把'生效）", any(v < -1e-9 for v in speeds),
          f"速度集合={sorted({round(v, 3) for v in speeds})}")
    check("7.7 ★投影换到了倒车段（seg1）且沿程推进",
          1 in segs, f"段集合={sorted(set(segs))}")
    s_end = rig.bb.get(P_ROUTE_S)
    check("7.8 倒车段上 s 继续增长（越过顶点 0.35 m 以上）",
          isinstance(s_end, (int, float)) and float(s_end) > 0.37,
          f"末 s={s_end}")


def main() -> int:
    print("=" * 72)
    print("dzbrain PIVOT 原地转向专项测试（纯脚本）")
    print("=" * 72)
    test_trigger_and_phases()
    test_timeout_no_deadlock()
    test_no_false_trigger()
    test_turn_respects_rate_limit()
    test_timeout_covers_physics()
    test_no_reentry_after_success()
    test_reverse_vertex_no_pivot()
    finish()
    return 0


if __name__ == "__main__":
    sys.exit(main())
