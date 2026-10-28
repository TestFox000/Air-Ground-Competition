# -*- coding: utf-8 -*-
"""P2-T1 段完成劫持闸（MISSION-GATE-20261029）—— `MissionTask._segment_completion_gate`。

现场证据（真车 2026-10-28，段1「N10→L8」）：
    段在 **0.02 s** 就报完成、到点航点 **0/4** —— 全路线重投影把 `s` 一脚推到段末
    1.40 m，`_check_finish` 只看 `s` 与 `_passed[last]` 就锁存 FINISH ⇒ 段完成被
    "投影异常"劫持。本闸要求运动段（段长 > 0.30 m）带最小行驶证据：
    行驶距离 ≥ 段长×0.5 **或** 用时 ≥ 2 s；不满足 ⇒ 打「段完成疑似劫持，继续等待」、
    保持不推进（不锁存 FINISH / 不报段完成）。

三条验收用例（卡面原文）：
    ① 0.02 s 假完成被拦且不推进；
    ② 正常段（约 1 m / 3 s）放行；
    ③ 0.2 m 短段 0.5 s 也放行（短段豁免）。

外加一条**反证**（证明闸是唯一变量）：同一份劫持现场、只把闸关掉
（`mission.segment_gate_min_s: 0`）⇒ FINISH 立刻锁存。

跑法：
    cd dzrobot_v2 && python -B -X utf8 src/dzbrain/tests/test_p2_t1_segment_hijack_gate.py
"""
from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

try:                                   # Windows 控制台 GBK：编不出的字符降级，别把测试炸掉
    sys.stdout.reconfigure(errors="replace")
except Exception:                      # pragma: no cover
    pass

from harness import check, check_close, finish, section                 # noqa: E402

from dzbrain.core import geometry as geo                                # noqa: E402
from dzbrain.core.blackboard import (P_POSE, P_ROUTE, P_SAFETY,         # noqa: E402
                                     Blackboard)
from dzbrain.core.config import Config                                 # noqa: E402
from dzbrain.core.events import EV_MISSION_FINISHED, EventBus           # noqa: E402
from dzbrain.core.state import SafetyState                             # noqa: E402
from dzbrain.io.contract import MissionPhase                           # noqa: E402
from dzbrain.route.model import Route, Waypoint                        # noqa: E402
from dzbrain.tasks.mission import MissionTask                          # noqa: E402

from fakes import FakeBridge, FakeClock                                # noqa: E402

#: 现场那一段的航点：N10 → M10 → L10 → L8（3 段 × 0.35 + 0.35 = 1.40 m）
SEG_TOKENS = ["N10", "M10", "L10", "L8"]


def cfg_of(*, gate: dict | None = None, auto_start: bool = True) -> Config:
    return Config.from_dict({
        "mission": {"auto_start": auto_start, "auto_start_delay_s": 0.0,
                    "reset": "N10,y", **(gate or {})},
        "pursuit": {"arrival_m": 0.12},
        "logging": {"rate_limit_s": 0.0},
        "route": {"tokens": SEG_TOKENS},
    })


class Rig:
    """一台"停着不动的车"：FakeBridge + Blackboard + MissionTask（手动单拍）。

    位姿只在 `set_pose` 时给一次 —— 本文件三条用例都靠"车有没有真走"来区分，
    所以**不许**在 tick 里偷偷推进位姿（那是"正常段"用例自己的事，见 `walk`）。
    """

    def __init__(self, *, cfg: Config) -> None:
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.logs: list[str] = []
        self._sub = self.bus.subscribe("p2", [EV_MISSION_FINISHED], maxsize=1024)
        self.events: list = []
        self.task = MissionTask(self.bridge, self.bb, self.bus, cfg,
                               log=self.logs.append, clock=self.clock.now)
        self.bb.set(P_SAFETY, SafetyState(ok=True), "test")
        self.set_pose_at("N10")
        self.task.setup()

    def set_pose_at(self, block: str) -> None:
        x, y = geo.block_to_meters(block)
        self.bridge.set_pose(x, y, geo.HEADING_Y)
        self.bb.set(P_POSE, geo.Pose(x, y, geo.HEADING_Y), "test")

    def tick(self, dt: float = 0.1) -> None:
        self.task.tick()
        self.events.extend(self._sub.poll(1024))
        self.clock.advance(dt)

    def ticks(self, n: int, dt: float = 0.1) -> None:
        for _ in range(n):
            self.tick(dt)

    def finishes(self) -> int:
        return sum(1 for e in self.events if e.type == EV_MISSION_FINISHED)

    def start_step_segment(self) -> None:
        """走**真实段入口**（`request_segment` → tick 开头 `_process_segment_requests`）。

        为什么要用段入口而不是 route.tokens 直跑：本闸只对步进循环的段生效
        （`_segment_active`）—— "段完成"是段的概念，整段一次性任务没有完成单。
        """
        res = self.task.request_segment(SEG_TOKENS, segment_id=1, target_cell="L8",
                                        known_count=30, source="test", stop_kind="end")
        assert res.get("accepted"), res
        self.task.tick()                    # 本拍处理段请求：载入 + DRIVE + _started_at

    def install_custom_route(self, route: Route, *, as_segment: bool = True) -> None:
        """装一条**非格心**坐标的路线（0.2 m 短段只能按米造，格名之间没有这么短的段）。"""
        self.task._install_route(route, source="test-short")
        self.task._started_at = self.clock.now()
        if as_segment:
            self.task._segment_active = True          # 视作步进循环的一个段
            self.task._segment_id = 7
        self.task._start_mission()

    def fake_current_s(self, s: float) -> None:
        """★ 复现现场故障态：投影（`s`）说"到段末了"，车其实一步没走。

        真车那一趟就是这样：全路线重投影把 s 推到 1.40 m。这里直接替换
        `_current_s`（判据的唯一输入），比伪造 Path 更贴近"投影异常"这一事实，
        也不会连带改动 `_advance_waypoints` 之外的任何东西。
        """
        self.task._current_s = lambda: float(s)      # type: ignore[method-assign]


# --------------------------------------------------------------------------- #
#  ① 0.02 s 假完成：拦下且不推进（= 卡面第一条）
# --------------------------------------------------------------------------- #
def case_hijack_blocked() -> None:
    section("① 劫持：1.40 m 段在 0.02 s 报完成 + 一步没走 → 拦下且不推进")
    rig = Rig(cfg=cfg_of())
    rig.start_step_segment()
    route = rig.bb.get(P_ROUTE)
    check("①.0 路线已载入且段长 1.40 m（现场同态）",
          route is not None and abs(route.path_length() - 1.40) < 1e-6,
          f"len={route.path_length() if route else None}")
    check("①.0b 阶段 DRIVE（不是 IDLE，否则 _check_finish 直接早退，测不到闸）",
          rig.task.phase == MissionPhase.DRIVE, f"phase={rig.task.phase!r}")

    rig.fake_current_s(route.path_length())          # 投影异常：s 一脚到段末
    rig.ticks(2, dt=0.01)                            # 0.02 s

    check("①.1 没有锁存 FINISH（0.02 s 的假完成被拦）",
          rig.task._finish_latched is False, f"latched={rig.task._finish_latched}")
    check("①.2 阶段不是 FINISH", rig.task.phase != MissionPhase.FINISH,
          f"phase={rig.task.phase!r}")
    check("①.3 没发 EV_MISSION_FINISHED（任务没被结算）", rig.finishes() == 0,
          f"got {rig.finishes()}")
    check("①.4 没有段完成回报（_segment_done 仍为空 ⇒ 步进循环不推进到下一段）",
          rig.task._segment_done is None, f"{rig.task._segment_done!r}")
    check("①.5 日志写明「段完成疑似劫持，继续等待」（现场一眼可见）",
          any("段完成疑似劫持，继续等待" in line for line in rig.logs),
          f"最后三行={rig.logs[-3:]}")
    check("①.6 计数可观测：segment_gate_holds >= 1", rig.task.segment_gate_holds >= 1,
          f"holds={rig.task.segment_gate_holds}")
    check("①.7 行驶证据如实：一步没走 ⇒ _travel_m == 0",
          abs(rig.task._travel_m) < 1e-9, f"travel={rig.task._travel_m:.4f} m")
    check("①.8 用时证据如实：0.02 s < 2 s（两条判据都不满足才算劫持）",
          rig.task._started_at is not None
          and (rig.clock.now() - rig.task._started_at) < 2.0,
          f"elapsed={rig.clock.now() - (rig.task._started_at or 0):.3f}s")


# --------------------------------------------------------------------------- #
#  ①b 反证：同一现场只把闸关掉 ⇒ FINISH 立刻锁存（闸是唯一变量）
# --------------------------------------------------------------------------- #
def case_hijack_with_gate_off() -> None:
    section("①b 反证：同一劫持现场，闸关掉（segment_gate_min_s=0）⇒ FINISH 锁存")
    rig = Rig(cfg=cfg_of(gate={"segment_gate_min_s": 0.0}))
    rig.start_step_segment()
    route = rig.bb.get(P_ROUTE)
    rig.fake_current_s(route.path_length())
    rig.ticks(2, dt=0.01)
    check("①b.1 闸关 ⇒ 照旧锁存 FINISH（说明 ① 的拦截来自闸，不是别的门）",
          rig.task._finish_latched is True, f"latched={rig.task._finish_latched}")
    check("①b.2 闸关 ⇒ segment_gate_holds 恒为 0", rig.task.segment_gate_holds == 0,
          f"holds={rig.task.segment_gate_holds}")


# --------------------------------------------------------------------------- #
#  ② 正常段（约 1 m / 3 s）：放行
# --------------------------------------------------------------------------- #
def case_normal_segment_passes() -> None:
    section("② 正常段：车真的走完 1.40 m（约 2.8 s @0.5 m/s）→ 放行且结算")
    rig = Rig(cfg=cfg_of())
    rig.start_step_segment()
    route = rig.bb.get(P_ROUTE)

    # 沿折线小步喂位姿（0.05 m/拍 @10 Hz = 0.5 m/s ⇒ 1.40 m 约 2.8 s）
    pts = [(w.x, w.y) for w in route.waypoints]
    cur = [pts[0][0], pts[0][1]]
    seg = 0
    steps = 0
    while seg < len(pts) - 1 and steps < 400:
        tx, ty = pts[seg + 1]
        dx, dy = tx - cur[0], ty - cur[1]
        dist = (dx * dx + dy * dy) ** 0.5
        if dist <= 0.05:
            cur = [tx, ty]
            seg += 1
        else:
            cur = [cur[0] + dx / dist * 0.05, cur[1] + dy / dist * 0.05]
        rig.bridge.set_pose(cur[0], cur[1], geo.heading_of(dx, dy, fallback=geo.HEADING_Y))
        rig.bb.set(P_POSE, geo.Pose(cur[0], cur[1], geo.HEADING_Y), "test")
        rig.tick(0.1)
        steps += 1
        if rig.task._finish_latched:
            break
    elapsed = rig.clock.now() - (rig.task._started_at or 0.0)
    check("②.1 已结算 FINISH", rig.task._finish_latched is True,
          f"latched={rig.task._finish_latched} steps={steps} travel={rig.task._travel_m:.3f}")
    check("②.2 发了 1 条 EV_MISSION_FINISHED", rig.finishes() == 1,
          f"got {rig.finishes()}")
    check("②.3 闸**没有**拦（segment_gate_holds == 0）", rig.task.segment_gate_holds == 0,
          f"holds={rig.task.segment_gate_holds}")
    check("②.4 行驶距离 ≈ 段长（0.3 m 容差内）",
          abs(rig.task._travel_m - route.path_length()) <= 0.30,
          f"travel={rig.task._travel_m:.3f} m / 段长={route.path_length():.2f} m")
    check("②.5 用时 ≥ 2 s（该段本来就要几秒；这条也是放行依据之一）",
          elapsed >= 2.0, f"elapsed={elapsed:.2f}s")


# --------------------------------------------------------------------------- #
#  ③ 0.2 m 短段 0.5 s：豁免放行
# --------------------------------------------------------------------------- #
def case_short_segment_exempt() -> None:
    section("③ 短段：0.20 m 段在 0.5 s 报完成 → 短段豁免，放行")
    rig = Rig(cfg=cfg_of(auto_start=False))          # 不自动跑，自己装短路线
    x0, y0 = geo.block_to_meters("N10")
    short = Route((
        Waypoint(block="N10", x=x0, y=y0, yaw=geo.HEADING_Y, token="N10"),
        Waypoint(block="N10+", x=x0, y=y0 + 0.20, yaw=geo.HEADING_Y, token="N10+"),
    ), name="short-0.20m")
    check("③.0 短段长度 0.20 m ≤ 豁免门 0.30 m",
          short.path_length() <= rig.task._seg_gate_min_m,
          f"len={short.path_length():.3f} / gate={rig.task._seg_gate_min_m}")
    rig.install_custom_route(short)
    rig.fake_current_s(short.path_length())
    # ★ 只 tick 一拍就要看到锁存：Mission 到 FINISH 的**下一拍**会复位回 IDLE
    #   （_reset_for_next_run 会把 _finish_latched 清掉），多 tick 会把证据洗掉。
    rig.tick(0.1)
    latched_at = rig.task._finish_latched
    elapsed_at = rig.clock.now() - (rig.task._started_at or 0.0)
    check("③.1 短段放行：FINISH 已锁存（0.1 s，车一步没走）", latched_at is True,
          f"latched={latched_at} holds={rig.task.segment_gate_holds}")
    check("③.2 闸没有拦（segment_gate_holds == 0）", rig.task.segment_gate_holds == 0,
          f"holds={rig.task.segment_gate_holds}")
    check("③.3 用时 0.1 s < 2 s、行驶 0 ⇒ 放行只能来自短段豁免",
          rig.task._travel_m < 0.02 and elapsed_at < 2.0,
          f"travel={rig.task._travel_m:.3f} m / elapsed={elapsed_at:.2f}s")
    rig.ticks(4, dt=0.1)                             # 0.5 s 总时长：到 FINISH 后回 IDLE
    check("③.4 只结算一次（EV_MISSION_FINISHED == 1，没被反复结算）",
          rig.finishes() == 1, f"got {rig.finishes()}")


# --------------------------------------------------------------------------- #
#  ④ 闸的配置默认值（卡面要求"写在现有配置结构里、给合理默认值"）
# --------------------------------------------------------------------------- #
def case_defaults() -> None:
    section("④ 默认值：0.30 m 豁免门 / 0.5 里程比 / 2.0 s 用时（缺键即生效）")
    cfg = Config.from_dict({"mission": {}, "pursuit": {"arrival_m": 0.12},
                            "logging": {"rate_limit_s": 0.0}})
    rig = Rig(cfg=cfg)
    check_close("④.1 segment_gate_min_m 默认 0.30", rig.task._seg_gate_min_m, 0.30, 1e-12)
    check_close("④.2 segment_gate_ratio 默认 0.50", rig.task._seg_gate_ratio, 0.50, 1e-12)
    check_close("④.3 segment_gate_min_s 默认 2.00", rig.task._seg_gate_min_s, 2.0, 1e-12)
    check("④.4 坏值不崩（字符串回落默认）",
          Rig(cfg=cfg_of(gate={"segment_gate_ratio": "abc"})).task._seg_gate_ratio == 0.50)


for name, fn in (("① 劫持", case_hijack_blocked),
                 ("①b 反证", case_hijack_with_gate_off),
                 ("② 正常段", case_normal_segment_passes),
                 ("③ 短段", case_short_segment_exempt),
                 ("④ 默认值", case_defaults)):
    try:
        fn()
    except Exception as exc:                     # noqa: BLE001
        import traceback
        traceback.print_exc()
        check(f"{name} 用例自身未抛异常", False, f"{type(exc).__name__}: {exc}")

finish()
