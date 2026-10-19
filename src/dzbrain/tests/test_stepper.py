# -*- coding: utf-8 -*-
"""车端步进循环 T27 自测（**纯脚本，无 pytest**）。

    python src\dzbrain\tests\test_stepper.py

覆盖（docs/step_loop.md §3 的逐条验收）：
  1. 布局 v2 校验：known/target_cell/segment_id；特征 ⊆ known；v1 布局保持旧行为
  2. 段规划：未知格（不在 known）=不可通行、target 不在 known 拒收、tracker-safe
     "策略性转弯降级为代价"在本图不再无解（N10→N5 能出路线）
  3. 规划失败 → WAIT_INFO（不退出、不规划、不出发、只刹车）
  4. 收到新版布局立即重规划（WAIT_INFO → PLANNING → DRIVING）
  5. **换段只在段边界**：DRIVING 中到新版不换路线，段末才用新布局重规划
  6. 段完成回报：P_CAR_STATE 的 phase=SEGMENT_DONE / segment_id / known_count /
     target_cell / block；MissionTask.segment_status().done 同源
  7. WAIT_INFO 静止不触发假完赛（阶段/事件都不动）
  8. WAIT_INFO 无超时（默认 0）与可配超时（>0）
  9. 到 end 格 → FINISH（含主循环退出码口径）
 10. 运行期 belief 连续落入未知格 → SAFE_STOP（停段 + 刹车）
 11. run_live 接线：v2 布局走步进分支（不再"规划一次"），v1 判定不变
 12. ★T32 段末射击停站语义：目标格命中射击停站 ⇒ 末 token 带 `*` / MissionTask 进 SHOOT /
     stop_kind（shoot|mine|path|end）随 P_CAR_STATE 上报；终点/雷格/普通格不标 `*`

全过 exit 0；任一失败 exit 1。
"""

from __future__ import annotations

import math
import os
import pathlib
import sys
import types

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

try:
    sys.stdout.reconfigure(errors="replace")  # type: ignore[union-attr]
except Exception:                              # pragma: no cover
    pass

from harness import check, finish, section                     # noqa: E402

from dzbrain.core import geometry as geo                       # noqa: E402
from dzbrain.core.blackboard import (Blackboard, P_MISSION_PHASE, P_ODOM_AGE,  # noqa: E402
                                     P_POSE, P_POSE_RAW, P_ROUTE, P_SAFETY, P_STOP_POINT)
from dzbrain.io import contract as CT                          # noqa: E402
from dzbrain.io import ros_io as RIO                           # noqa: E402
from dzbrain.tasks.localization import LocalizationTask         # noqa: E402
from dzbrain.core.config import Config                         # noqa: E402
from dzbrain.core.events import EV_MISSION_FINISHED, EventBus  # noqa: E402
from dzbrain.core.state import SafetyState, StopKind, StopPhase  # noqa: E402
from dzbrain.io.contract import AimStatus, MissionPhase        # noqa: E402
from dzbrain.tasks.mission import MissionTask, plan_segment_route  # noqa: E402
from dzbrain import main as M                                  # noqa: E402

from fakes import FakeBridge, FakeClock                        # noqa: E402

#: 真实图上的"东侧走廊"：N10 出发向西一格/两格（不需要穿桥洞，规划确定）
N10 = geo.block_to_meters("N10")
HEAD_Y = geo.HEADING_Y


def cfg_of(*, wait_s: float = 0.0, engine: str = "tracker-safe",
           reset: str = "N10,y") -> Config:
    return Config.from_dict({
        "planner": {"auto_plan_enabled": True, "engine": engine},
        "mission": {"auto_start": False, "reset": reset},
        # ★T32：射击停站窗口用**现役 config/robot.yaml** 的值（0.90 ± 0.12 ⇒ [0.78,1.02]），
        #   不用 SCHEMA 的 0.80±0.10 旧默认 —— 段末射击停站判据是 T21 真源。
        "route": {"shoot_distance_m": 0.90, "shoot_distance_tol_m": 0.12},
        "step": {"wait_timeout_s": wait_s},
        "logging": {"rate_limit_s": 1.0},
    })


def layout_of(*, known, target="", version=1, segment_id=1, start="N10", end="N5",
              mines=(), obstacles=(), targets=(), bridges=("K7", "K5")) -> dict:
    return {"type": "field_layout", "version": version, "ts": 0.0,
            "start": start, "end": end, "known": list(known),
            "mines": list(mines), "obstacles": list(obstacles),
            "targets": list(targets), "bridges": list(bridges),
            "barrier_col_from": "D", "barrier_row_hi": 6, "barrier_row_lo": 7,
            "target_cell": target, "segment_id": segment_id}


class Rig:
    """一台"步进车"：FakeBridge + 真 MissionTask + StepLoop（无 ROS、无车）。"""

    def __init__(self, *, cfg: Config | None = None) -> None:
        self.cfg = cfg or cfg_of()
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.logs: list[str] = []
        self.bb.set(P_SAFETY, SafetyState(ok=True), "test")
        self.bridge.set_pose_at_block("N10", yaw=HEAD_Y)
        self.mission = MissionTask(self.bridge, self.bb, self.bus, self.cfg,
                                   log=self.logs.append, clock=self.clock.now)
        self.mission.setup()
        self.mission.tick()
        self.loop = M.StepLoop(self.cfg, self.bb, self.bridge, self.mission,
                               bus=self.bus, log=self.logs.append, clock=self.clock.now)
        self.car_states: list[dict] = []
        self.finished_events = 0
        self._route_obj = None
        self._wp_idx = 1
        self._sub = self.bus.subscribe("stepper-test", [EV_MISSION_FINISHED], maxsize=4096)

    # ---------------- 布局 ----------------
    def start(self, layout: dict) -> None:
        self.bb.set(M.P_FIELD_LAYOUT, layout, "test")
        loaded = M.load_step_layout(self.bb)
        assert loaded is not None, "测试布局没过 normalize_step_layout"
        self.loop.start(loaded)

    def feed(self, layout: dict) -> None:
        self.bb.set(M.P_FIELD_LAYOUT, layout, "test")

    # ---------------- 位姿/驾驶 ----------------
    def _set_pose(self, x: float, y: float, yaw: float = HEAD_Y) -> None:
        self.bridge.set_pose(x, y, yaw)
        self.bb.set(P_POSE, geo.Pose(x, y, yaw), "test")

    def _follow_route(self, step_m: float = 0.05) -> None:
        """沿当前 P_ROUTE 走一步（测试替身：直接喂位姿，不跑 tracker）。"""
        pose = self.bb.get(P_POSE)
        route = self.bb.get(P_ROUTE)
        if pose is None or route is None:
            return
        if route is not self._route_obj:
            self._route_obj = route
            pts = [(w.x, w.y) for w in route.waypoints]
            best = (1e9, 1)
            for i in range(1, len(pts)):
                d = math.hypot(pts[i][0] - pose.x, pts[i][1] - pose.y)
                if d < best[0]:
                    best = (d, i)
            self._wp_idx = best[1]
        if self.mission.phase not in (MissionPhase.DRIVE, MissionPhase.APPROACH,
                                      MissionPhase.STOP):
            return
        pts = [(w.x, w.y) for w in route.waypoints]
        idx = min(self._wp_idx, len(pts) - 1)
        tx, ty = pts[idx]
        dx, dy = tx - pose.x, ty - pose.y
        dist = math.hypot(dx, dy)
        if dist <= step_m:
            self._set_pose(tx, ty)
            if idx < len(pts) - 1:
                self._wp_idx = idx + 1
        else:
            self._set_pose(pose.x + dx / dist * step_m, pose.y + dy / dist * step_m)

    def step(self, n: int = 1, *, drive: bool = False, step_m: float = 0.05) -> None:
        for _ in range(n):
            if drive:
                self._follow_route(step_m)
            self.loop.tick()
            self.mission.tick()
            snap = self.bb.get(M.P_CAR_STATE)
            if isinstance(snap, dict):
                self.car_states.append(dict(snap))
            self.clock.advance(0.1)
            self.finished_events += len(self._sub.poll(256))

    def drive_until_done(self, *, max_steps: int = 1500) -> dict | None:
        for _ in range(max_steps):
            self.step(drive=True)
            snap = self.bb.get(M.P_CAR_STATE)
            if isinstance(snap, dict) and snap.get("phase") == M.STEP_PHASE_SEGMENT_DONE:
                return snap
        return None

    def done_states(self) -> list[dict]:
        return [s for s in self.car_states if s.get("phase") == M.STEP_PHASE_SEGMENT_DONE]

    def logs_with(self, needle: str) -> list[str]:
        return [ln for ln in self.logs if needle in ln]

    def close(self) -> None:
        try:
            self.mission.teardown()
        except Exception:                          # noqa: BLE001
            pass


# =========================================================================== #
#  1. 布局 v2 校验 / v1 兼容
# =========================================================================== #
section("1. 布局 v2 归一化：known/target_cell/segment_id 入模，v1 保持旧行为")
v2 = layout_of(known=["N10", "M10", "L10"], target="L10", segment_id=42)
norm, why = M.normalize_step_layout(v2)
check("1.1 合法 v2 → known/target_cell/segment_id 全部保留",
      norm is not None and norm["known"] == ["N10", "M10", "L10"]
      and norm["target_cell"] == "L10" and norm["segment_id"] == 42,
      f"{norm} / {why!r}")
check("1.2 v2 布局走步进循环（_step_mode_enabled=True）",
      M._step_mode_enabled(norm) is True)
v1 = layout_of(known=[])
v1.pop("known")
v1.pop("target_cell")
v1.pop("segment_id")
v1n, why1 = M.normalize_step_layout(v1)
check("1.3 v1（无 known）→ 归一化后仍**不补 known 键**（旧'整图已知'语义）+ 判为非步进",
      v1n is not None and "known" not in v1n and M._step_mode_enabled(v1n) is False,
      f"{sorted(v1n or {})} / {why1!r}")
bad_known = layout_of(known=["N10"], target="N10", mines=["A2"])
bad_norm, bad_why = M.normalize_step_layout(bad_known)
check("1.4 特征不在 known 里 → 整份拒收（特征 ⊆ known）",
      bad_norm is None and "known" in bad_why, f"{bad_norm} / {bad_why!r}")
bad_target = layout_of(known=["N10"], target="Z9")
bt_norm, bt_why = M.normalize_step_layout(bad_target)
check("1.5 target_cell 非法 → 整份拒收",
      bt_norm is None and "target_cell" in bt_why, f"{bt_norm} / {bt_why!r}")
half_v2 = layout_of(known=[], target="M10")
half_v2.pop("known")
hv_norm, hv_why = M.normalize_step_layout(half_v2)
check("1.6 半 v2（有 target_cell 无 known）→ 拒收（不能把目标悄悄丢掉）",
      hv_norm is None and "known" in hv_why, f"{hv_norm} / {hv_why!r}")
# =========================================================================== #
#  2. 段规划：未知格=不可通行（但不是物理障碍）
# =========================================================================== #
section("2. 段规划：未知格不可通行 / target 必须在 known / tracker-safe 策略性转弯降级")
cfg = cfg_of()
p_ok = M.plan_segment_from_layout(
    cfg, norm, from_cell="N10", target_cell="L10", heading_dir=None)
check("2.1 known 覆盖时能出段（N10→L10 = [N10, M10, L10]）",
      p_ok.ok and p_ok.cells == ["N10", "M10", "L10"],
      f"{p_ok.error} / {p_ok.cells}")
check("2.2 段路线全部落在 known 内（unknown 双保险自检）",
      p_ok.ok and all(c in set(norm["known"]) for c in p_ok.cells), str(p_ok.cells))
blocked = layout_of(known=["N10", "L10"], target="L10")
p_block = M.plan_segment_from_layout(cfg, M.normalize_step_layout(blocked)[0],
                                     from_cell="N10", target_cell="L10")
check("2.3 中间格 M10 未知 ⇒ 无解（绝不穿未知格）",
      not p_block.ok and "段规划无解" in p_block.error, f"{p_block.error}")
p_target = M.plan_segment_from_layout(cfg, norm, from_cell="N10", target_cell="A1")
check("2.4 target 不在 known ⇒ 拒收（'不在 known' 写进原因）",
      not p_target.ok and "不在 known" in p_target.error, p_target.error)

all_cells = M.all_map_cells()
full = layout_of(known=all_cells, target="N5", start="N10", end="N5",
                 mines=["A2", "F4", "H4", "G5", "E8", "B10", "D11", "I11"],
                 obstacles=["B1", "D1", "F3", "I6", "H7", "A8", "C9", "E9"],
                 targets=["D5", "D6"])
full_n = M.normalize_step_layout(full)[0]
p_full = M.plan_segment_from_layout(cfg, full_n, from_cell="N10", target_cell="N5")
check("2.5 ★ tracker-safe 段规划在本图**仍有解**（整线引擎会因隔板转弯无解：N10→N5 32 格）",
      p_full.ok and len(p_full.cells) >= 20 and "M7" in p_full.cells,
      f"{p_full.error} / {len(p_full.cells)} 格")
check("2.6 策略性转弯已降级并在 warnings 里留痕（M7/I7 两处）",
      p_full.ok and any("策略性转弯" in w for w in p_full.warnings),
      str(p_full.warnings))
check("2.7 段规划显式带引擎与耗时（日志/遥测可读）",
      p_full.engine == "tracker-safe" and p_full.elapsed_ms >= 0.0
      and p_full.route_len_m > 0.0,
      f"{p_full.engine} / {p_full.elapsed_ms:.1f}ms / {p_full.route_len_m:.2f}m")

# 纯几何单测：起点车头与首段相反时，允许"原地转向"重规划并留警告
raw_map = {"map": {"start": "N10", "end": "M10", "mines": [], "obstacles": [],
                   "targets": [], "bridges": ["K7", "K5"],
                   "barrier_col_from": "D", "barrier_row_hi": 6, "barrier_row_lo": 7}}
# 把 N10/M10 之外全禁掉（只剩一条 N10→M10 的走廊）；车头朝反方向 (0,1) ⇒ 首步必是 180°
only_corridor = [c for c in all_cells if c not in ("N10", "M10")]
p_back = plan_segment_route(raw_map, cfg, from_cell="N10", target_cell="M10",
                            heading_dir=(0, 1), blocked_extra=only_corridor)
check("2.8 起点朝向与唯一可行首步相反 ⇒ 允许原地转向重规划（不再永久 WAIT_INFO）",
      p_back.ok and p_back.cells == ["N10", "M10"]
      and any("原地转向" in w for w in p_back.warnings),
      f"{p_back.error} / {p_back.cells} / {p_back.warnings}")

# =========================================================================== #
#  3. 规划失败 → WAIT_INFO（不退出、不规划、不出发、只刹车）
# =========================================================================== #
section("3. 规划失败 → WAIT_INFO：不退出、不规划、不出发、只刹车")
rig = Rig()
rig.start(blocked)
rig.step(2)
check("3.1 规划失败 ⇒ 相位 WAIT_INFO（不抛、不退出）",
      rig.loop.phase == M.STEP_PHASE_WAIT_INFO, rig.loop.phase)
check("3.2 WAIT_INFO 原因写明'段规划失败'",
      "段规划失败" in rig.loop.status()["reason"], rig.loop.status()["reason"])
check("3.3 任务线程仍在 IDLE、P_ROUTE 为空（一步都没发）",
      rig.mission.phase == MissionPhase.IDLE and rig.bb.get(P_ROUTE) is None,
      f"{rig.mission.phase} / route={rig.bb.get(P_ROUTE)}")
check("3.4 WAIT_INFO 进入时发过刹车（停车保持位姿）", rig.bridge.brakes >= 1,
      f"brakes={rig.bridge.brakes}")
rig.step(60)
check("3.5 无新布局时一直等（6s 后仍 WAIT_INFO；无超时配置）",
      rig.loop.phase == M.STEP_PHASE_WAIT_INFO and rig.loop.status()["replans"] == 1,
      f"{rig.loop.phase} / replans={rig.loop.status()['replans']}")

# =========================================================================== #
#  4. 收到新版布局 → 立即重规划
# =========================================================================== #
section("4. 收到新版布局立即重规划（WAIT_INFO → PLANNING → DRIVING）")
rig.feed(layout_of(known=["N10", "M10", "L10"], target="L10", version=2, segment_id=2))
rig.step(1)
check("4.1 新版布局到达的那一拍立刻转 PLANNING（不等 5s 日志周期）",
      rig.loop.phase == M.STEP_PHASE_PLANNING, rig.loop.phase)
rig.step(1)
check("4.2 下一拍完成段规划并下发（DRIVING）",
      rig.loop.phase == M.STEP_PHASE_DRIVING, rig.loop.phase)
route = rig.bb.get(P_ROUTE)
check("4.3 执行路线 = N10→M10→L10（新 known 立即生效）",
      route is not None and [w.block for w in route.waypoints] == ["N10", "M10", "L10"],
      str([w.block for w in route.waypoints] if route else None))
check("4.4 重规划计数 +1、目标格换成本段 target",
      rig.loop.status()["replans"] == 2 and rig.loop.status()["target_cell"] == "L10",
      str(rig.loop.status()))

# =========================================================================== #
#  5. ★ 换段只在段边界
# =========================================================================== #
section("5. ★ 换段只在段边界：DRIVING 中到新版布局不改路线")
before_route = rig.bb.get(P_ROUTE)
before_ids = id(before_route)
rig.feed(layout_of(known=["N10", "M10", "L10", "L9", "N9", "M9"],
                   target="N9", version=3, segment_id=3))
rig.step(3)
check("5.1 DRIVING 中收到新版布局：仍执行本段旧路线（Route 对象未换）",
      rig.bb.get(P_ROUTE) is before_route and id(rig.bb.get(P_ROUTE)) == before_ids,
      f"route id {id(rig.bb.get(P_ROUTE))} vs {before_ids}")
check("5.2 段内不更新 segments_started（仍是第 1 段）",
      rig.loop.status()["segments_started"] == 1, str(rig.loop.status()))
check("5.3 日志明确留痕：新布局'段边界再重规划'",
      bool(rig.logs_with("段边界再重规划")), str(rig.logs_with("段边界再重规划")[:1]))
done1 = rig.drive_until_done()
done_info = rig.mission.segment_status()["done"] or {}
check("5.4 第 1 段在段边界完成（停 L10）",
      done1 is not None and done1["block"] == "L10", str(done1))
rig.step(1)                                    # SEGMENT_DONE → 段边界判定
check("5.5 段边界发现新布局 ⇒ 转 PLANNING（不是 WAIT_INFO）",
      rig.loop.phase == M.STEP_PHASE_PLANNING, rig.loop.phase)
rig.step(1)                                    # 规划第 2 段
route2 = rig.bb.get(P_ROUTE)
check("5.6 第 2 段按新布局下发（目标 N9）",
      rig.loop.status()["segments_started"] == 2 and route2 is not None
      and route2.waypoints[-1].block == "N9",
      f"{rig.loop.status()['segments_started']} / "
      f"{[w.block for w in route2.waypoints] if route2 else None}")

# =========================================================================== #
#  6. 段完成回报字段
# =========================================================================== #
section("6. 段完成回报：P_CAR_STATE(phase=SEGMENT_DONE, segment_id, known_count, target_cell, block)")
done_states = rig.done_states()
last_done = done_states[-1] if done_states else {}
check("6.1 P_CAR_STATE 出现过 phase=SEGMENT_DONE", bool(done_states))
check("6.2 回报字段齐全且值正确（segment_id=2 / target=L10 / block=L10 / known=3）",
      last_done.get("segment_id") == 2 and last_done.get("target_cell") == "L10"
      and last_done.get("block") == "L10" and last_done.get("known_count") == 3,
      str(last_done))
seg_done = done_info          # 段 2 完成时抓的；段 3 起段时 request_segment 会清空旧 done
check("6.3 MissionTask.segment_status().done 与 P_CAR_STATE 同源（段号/目标/格）",
      seg_done.get("segment_id") == 2 and seg_done.get("target_cell") == "L10"
      and seg_done.get("block") == "L10" and seg_done.get("pose_block") == "L10",
      str(seg_done))
check("6.4 段完成单含用时/航点数（现场台账可读）",
      seg_done.get("elapsed_s", -1) >= 0.0 and seg_done.get("waypoints") == 3,
      str({k: seg_done.get(k) for k in ("elapsed_s", "waypoints")}))
check("6.5 DRIVING 阶段也回报当前 target/segment/known",
      all(("phase" in s and "segment_id" in s and "target_cell" in s
           and "known_count" in s) for s in rig.car_states),
      f"{len(rig.car_states)} 份 P_CAR_STATE")

# =========================================================================== #
#  7. WAIT_INFO 静止不触发假完赛
# =========================================================================== #
section("7. WAIT_INFO 静止不触发假完赛（阶段/事件都不动）")
# 把第 2 段跑完 → 段末 WAIT_INFO；然后原地不动 40 拍
done2 = rig.drive_until_done()
check("7.1 第 2 段完成（停 N9）", done2 is not None and done2["block"] == "N9", str(done2))
rig.step(1)
check("7.2 段末转 WAIT_INFO", rig.loop.phase == M.STEP_PHASE_WAIT_INFO, rig.loop.phase)
ev_before = rig.finished_events
phase_before = rig.mission.phase
rig.step(40)                                   # 4s 静止
check("7.3 静止 4s：步进相位仍 WAIT_INFO、任务线程仍 IDLE（没有假完赛）",
      rig.loop.phase == M.STEP_PHASE_WAIT_INFO and rig.mission.phase == MissionPhase.IDLE,
      f"{rig.loop.phase} / {rig.mission.phase}（之前 {phase_before}）")
check("7.4 没有新增 EV_MISSION_FINISHED（段末那一发之外不再发）",
      rig.finished_events == ev_before, f"{ev_before} → {rig.finished_events}")
check("7.5 静止期间 P_CAR_STATE 只有 WAIT_INFO（没有 FINISH/DRIVING 抖动）",
      {s["phase"] for s in rig.car_states[-10:]} == {M.STEP_PHASE_WAIT_INFO},
      str({s["phase"] for s in rig.car_states[-10:]}))
rig.close()

# =========================================================================== #
#  8. WAIT_INFO 无超时 vs 可配超时
# =========================================================================== #
section("8. WAIT_INFO 超时语义：默认 0 = 无限；>0 才超时")
rig8 = Rig(cfg=cfg_of(wait_s=0.0))
rig8.start(blocked)
rig8.step(2)
check("8.1 进入 WAIT_INFO", rig8.loop.phase == M.STEP_PHASE_WAIT_INFO, rig8.loop.phase)
rig8.step(600)                                 # 60s
check("8.2 step.wait_timeout_s=0 ⇒ 60s 仍 WAIT_INFO（无上限）",
      rig8.loop.phase == M.STEP_PHASE_WAIT_INFO, rig8.loop.phase)
check("8.3 每 5s 一条 WAIT_INFO 日志（60s 内 ≥ 10 条）",
      len(rig8.logs_with("WAIT_INFO 已")) >= 10, str(len(rig8.logs_with("WAIT_INFO 已"))))
rig8.close()

rig9 = Rig(cfg=cfg_of(wait_s=5.0))
rig9.start(blocked)
rig9.step(2)
check("8.4 配置超时 5s：进入 WAIT_INFO", rig9.loop.phase == M.STEP_PHASE_WAIT_INFO)
rig9.step(80)                                  # 8s
check("8.5 超过 5s ⇒ WAIT_INFO_TIMEOUT（显式退出，不静默挂住）",
      rig9.loop.phase == M.STEP_PHASE_WAIT_TIMEOUT, rig9.loop.phase)
why9 = rig9.loop.run(stop_flag={"stop": False}, sleep=lambda _s: None)
check("8.6 主循环退出原因 = timeout（run_live 映射退出码 4）", why9 == "timeout", why9)
rig9.close()

# =========================================================================== #
#  9. 到 end 格 → FINISH
# =========================================================================== #
section("9. 到 end 格 → FINISH")
rig10 = Rig()
rig10.start(layout_of(known=["N10", "M10"], target="M10", version=1, segment_id=9,
                      start="N10", end="M10"))
done10 = rig10.drive_until_done()
check("9.1 段完成（目标 = end = M10）",
      done10 is not None and done10["block"] == "M10", str(done10))
rig10.step(1)
check("9.2 段完成后判定 target == end ⇒ 步进相位 FINISH",
      rig10.loop.phase == M.STEP_PHASE_FINISH, rig10.loop.phase)
fin_state = rig10.bb.get(M.P_CAR_STATE) or {}
check("9.3 P_CAR_STATE 最终 phase=FINISH、block=M10",
      fin_state.get("phase") == M.STEP_PHASE_FINISH and fin_state.get("block") == "M10",
      str(fin_state))
why10 = rig10.loop.run(stop_flag={"stop": False}, sleep=lambda _s: None)
check("9.4 步进主循环返回 finish（run_live 退出码 0）", why10 == "finish", why10)
rig10.close()

# =========================================================================== #
# 10. 运行期 belief 落入未知格 → SAFE_STOP
# =========================================================================== #
section("10. 运行期 belief 连续落入未知格 → SAFE_STOP（停段 + 刹车）")
rig11 = Rig()
rig11.start(layout_of(known=["N10", "M10", "L10"], target="L10", version=1, segment_id=1))
rig11.step(2)
check("10.1 前置：段已下发、任务线程 DRIVING",
      rig11.loop.phase == M.STEP_PHASE_DRIVING
      and rig11.mission.phase in (MissionPhase.DRIVE, MissionPhase.APPROACH),
      f"{rig11.loop.phase} / {rig11.mission.phase}")
brakes_before = rig11.bridge.brakes
# N9 不在本段 known 里（明确的未知格），且贴着路线起点 —— 位姿不会"瞬移到终点"
# 让 Mission 先判完赛（真实场景里 belief 不会瞬移，本用例要测的是运行期守卫）。
rig11._set_pose(*geo.block_to_meters("N9"))
rig11.step(4)
check("10.2 连续 3 拍未知格 ⇒ 步进相位 SAFE_STOP", rig11.loop.phase == M.STEP_PHASE_SAFE_STOP,
      rig11.loop.phase)
check("10.3 已请求停段（P_ROUTE 清空、任务线程回 IDLE）",
      rig11.bb.get(P_ROUTE) is None and rig11.mission.phase == MissionPhase.IDLE,
      f"route={rig11.bb.get(P_ROUTE)} / {rig11.mission.phase}")
check("10.4 SAFE_STOP 发了刹车", rig11.bridge.brakes > brakes_before,
      f"{brakes_before} → {rig11.bridge.brakes}")
check("10.5 P_CAR_STATE 回报 SAFE_STOP（现场能看出来为什么停）",
      (rig11.bb.get(M.P_CAR_STATE) or {}).get("phase") == M.STEP_PHASE_SAFE_STOP,
      str(rig11.bb.get(M.P_CAR_STATE)))
check("10.6 SAFE_STOP 不会自动恢复（再跑 20 拍仍 SAFE_STOP）",
      (rig11.step(20) or True) and rig11.loop.phase == M.STEP_PHASE_SAFE_STOP,
      rig11.loop.phase)
rig11.close()

# =========================================================================== #
# 11. run_live 接线（替身，不碰 ROS）：v2 走步进，v1 判定不变
# =========================================================================== #
section("11. run_live 接线：v2 布局 → StepLoop；不再'规划一次'")
import dzbrain.io.ros_io as _rz                                          # noqa: E402


class _FakeLiveBridge:
    """run_live 的 RosBridge 替身（NO ROS）。

    · READY 控制位姿就绪门（pose()/pose_age()）；
    · LINK 控制"图上端点是否在"（endpoint_counts 的 odom 发布方）；
    · ENDPOINTS 可逐话题覆盖 (pubs, subs)，用于构造缺端点场景。
    """

    READY = True
    LINK = True
    ENDPOINTS: dict = {}

    def __init__(self, *a, **k):
        self.shutdown_brakes: list[bool] = []
        self.spun = 0
        self._ready = bool(self.READY)
        self._link = bool(self.LINK)

    def spin_async(self, threads: int = 4):
        self.spun += 1

    def shutdown(self, *, brake: bool = True):
        self.shutdown_brakes.append(brake)
        return 0

    # ---- 启动自检 / 位姿就绪门接口（与 RosBridge 同名同义）----
    def subscription_topics(self):
        return list(RIO.required_subscription_topics().values())

    def endpoint_counts(self, topic):
        """启动契约用：话题 -> (publishers, subscribers)。ENDPOINTS 覆盖优先。"""
        if topic in self.ENDPOINTS:
            return self.ENDPOINTS[topic]
        if topic == "/odom/localization":
            return (1 if self._link else 0), 1
        return 1, 1

    def topic_counts(self):
        return {"odom": {"publishers": 1 if self._link else 0, "subscribers": 1},
                "drone_in": {"publishers": 1, "subscribers": 1},
                "pursuit_angle": {"publishers": 1, "subscribers": 1}}

    def pose(self):
        return geo.Pose(*geo.block_to_meters("N10"), geo.HEADING_Y) if self._ready else None

    def pose_age(self):
        return 0.05 if self._ready else float("inf")

    def publish_brake(self):
        return None


class _FakeLiveDrone:
    def __init__(self, *a, **k):
        self.layout_updates = 0
        self.layout_rejects = 0

    def start(self):
        pass

    def stop(self, timeout: float | None = 2.0):
        return True

    def health(self):
        return types.SimpleNamespace(ticks=0, errors=0)


class _FakeMission:
    def __init__(self):
        self.name = "MissionTask"
        self.reset_specs: list[str] = []

    def reset_pose(self, spec: str) -> None:
        self.reset_specs.append(spec)


class _FakeGroup:
    def __init__(self):
        self._tasks: list = [_FakeMission()]
        self.started = False

    def start_all(self):
        self.started = True

    def stop_all(self, timeout: float = 2.0):
        return []


_saved = {name: getattr(M, name) for name in (
    "install_stop_signals", "build_tasks", "start_banner", "finish_report",
    "_start_layout_listener", "_wait_for_field_layout", "_run_step_loop",
    "plan_route_from_field_layout")}
_rz_saved = _rz.RosBridge
try:
    _rz.RosBridge = _FakeLiveBridge
    M.install_stop_signals = lambda *a, **k: []
    M.finish_report = lambda *a, **k: None
    M.start_banner = lambda *a, **k: None
    M._start_layout_listener = lambda *a, **k: _FakeLiveDrone()
    step_layout = M.normalize_step_layout(
        layout_of(known=["N10", "M10", "L10"], target="L10", version=1))[0]
    M._wait_for_field_layout = lambda *a, **k: dict(step_layout)
    calls: dict = {"build": 0, "skip": None, "step": 0, "one_shot": 0}

    def _fake_build(*a, **kw):
        calls["build"] += 1
        calls["skip"] = kw.get("skip")
        return _FakeGroup(), []

    def _fake_one_shot(*a, **k):
        calls["one_shot"] += 1
        return types.SimpleNamespace(ok=True, tokens=["N10", "N5"], route_len_m=1.0,
                                     engine="tour", elapsed_ms=1.0)

    def _fake_step_loop(loop, *, stop_flag, alive=None, log=None):
        calls["step"] += 1
        calls["loop"] = loop
        return "finish"

    M.build_tasks = _fake_build
    M.plan_route_from_field_layout = _fake_one_shot
    M._run_step_loop = _fake_step_loop
    cfg_live = Config.from_dict({"planner": {"auto_plan_enabled": True},
                                 "route": {"tokens": ["N10", "N5,y"]},
                                 # §11 只验接线：位姿门关闭（位姿门由 §13 单独钉）
                                 "step": {"pose_ready_wait_s": 0.0}})
    rc = M.run_live(cfg_live, log=lambda s: None)
    check("11.1 v2 布局 → run_live 走步进循环（_run_step_loop 调 1 次）",
          calls["step"] == 1, str(calls))
    check("11.2 不再调用 plan_route_from_field_layout（不'规划一次'）",
          calls["one_shot"] == 0, str(calls))
    check("11.3 MissionTask 的 auto_start 被关掉（段由步进器显式下发）",
          cfg_live.get("mission.auto_start") is False,
          str(cfg_live.get("mission.auto_start")))
    check("11.4 任务组仍用 skip={'drone'} 构造、且真的启动了",
          calls["build"] == 1 and calls["skip"] == {"drone"}, str(calls))
    check("11.5 步进循环启动时按 mission.reset 复位 belief 一次",
          bool(calls["loop"].status()) and calls["loop"].phase in (
              M.STEP_PHASE_PLANNING, M.STEP_PHASE_WAIT_LAYOUT),
          calls["loop"].phase)
    check("11.6 正常收尾 exit 0", rc == 0, f"rc={rc}")
finally:
    _rz.RosBridge = _rz_saved
    for _name, _orig in _saved.items():
        setattr(M, _name, _orig)

check("11.7 v1 布局（无 known）仍判旧行为（步进分支不启用）",
      M._step_mode_enabled({"start": "N10", "end": "N5", "mines": [],
                            "obstacles": [], "targets": [], "bridges": ["K7"]}) is False)

# =========================================================================== #
# 12. ★T32 段末射击停站语义（目标格 = 射击格 ⇒ 末 token 带 * / stop_kind 上报）
# =========================================================================== #
section("12. ★T32 段末射击停站：target 命中射击格 → 末 token 带 *、MissionTask 进 SHOOT")

# 场景图 = T29 台架真值图（2026-10-18 现场图）：靶 D7+E7（板心 (1.575, 3.5)），
# 现役窗口 [0.78,1.02] 内 C9 = 0.875 m（覆盖 D7 0.783 / E7 0.990），F9 同样合格。
SIM_MINES = ["B1", "A2", "I2", "C3", "E3", "G5", "I8", "H9"]
SIM_OBSTACLES = ["D5", "I6", "A7", "F8", "B9", "F10", "H10", "A11"]
SIM_TARGETS = ["D7", "E7"]
SIM_BRIDGES = ("K4", "K10")


def sim_layout(*, target: str, version: int = 1, segment_id: int = 1) -> dict:
    """全 known 的 T29 台架图布局（带 targets，才能判"段末是不是射击停站"）。"""
    return layout_of(known=M.all_map_cells(), target=target, version=version,
                     segment_id=segment_id, mines=SIM_MINES, obstacles=SIM_OBSTACLES,
                     targets=SIM_TARGETS, bridges=SIM_BRIDGES)


from dzbrain.route import planner as _PL                          # noqa: E402

cfg12 = cfg_of()
lay_shoot = sim_layout(target="C9")
n_shoot = M.normalize_step_layout(lay_shoot)[0]
p_shoot = M.plan_segment_from_layout(cfg12, n_shoot, from_cell="H9", target_cell="C9")
check("12.1 ★ 目标格命中射击停站 ⇒ 末 token = 'C9*'（T29 端到端 0 停站的根因）",
      p_shoot.ok and p_shoot.tokens[-1] == "C9*" and p_shoot.cells[-1] == "C9",
      f"{p_shoot.error} / tokens={p_shoot.tokens}")
check("12.2 stop_kind='shoot'、到靶板中心 0.875 m（现役窗口 [0.78,1.02] 内）",
      p_shoot.stop_kind == "shoot"
      and 0.78 - 1e-9 <= p_shoot.stop_distance_m <= 1.02 + 1e-9
      and abs(p_shoot.stop_distance_m - 0.875) < 1e-6,
      f"{p_shoot.stop_kind} / {p_shoot.stop_distance_m:.4f} m")
_spec12 = _PL.load_map(M._segment_map_cfg(n_shoot, "H9", "C9"))
_cov12 = _PL.covering_stops(_spec12, ["C9"], _PL._shoot_accept(cfg12))   # noqa: SLF001
check("12.3 判据与 T21 同源（planner.covering_stops）：C9 覆盖 D7+E7 两格靶板",
      sorted(_cov12.get("C9", ())) == ["D7", "E7"], str(_cov12))

p_end = M.plan_segment_from_layout(cfg12, n_shoot, from_cell="H9", target_cell="N5")
check("12.4 target=N5（布局 end）⇒ 不标 *、stop_kind='end'（终点绝不停站打靶）",
      p_end.ok and p_end.tokens[-1] == "N5" and not any("*" in t for t in p_end.tokens)
      and p_end.stop_kind == "end",
      f"{p_end.error} / {p_end.tokens[-2:]} / {p_end.stop_kind}")
p_mine = M.plan_segment_from_layout(cfg12, n_shoot, from_cell="H9", target_cell="I8")
check("12.5 target=雷格 I8 ⇒ 不标 *、stop_kind='mine'（压过即可，不额外停站）",
      p_mine.ok and p_mine.tokens[-1] == "I8" and not any("*" in t for t in p_mine.tokens)
      and p_mine.stop_kind == "mine",
      f"{p_mine.error} / {p_mine.tokens} / {p_mine.stop_kind}")
p_path = M.plan_segment_from_layout(cfg12, n_shoot, from_cell="C9", target_cell="D9")
check("12.6 普通路径目标 D9（板心 0.63 m 在窗口外）⇒ stop_kind='path'、无 *",
      p_path.ok and p_path.stop_kind == "path" and not any("*" in t for t in p_path.tokens),
      f"{p_path.error} / {p_path.tokens} / {p_path.stop_kind}")
check("12.7 空段分支（from==target，不经过规划器）也如实分类：C9→shoot / N5→end",
      (M.classify_segment_target(cfg12, n_shoot, "C9") == "shoot"
       and M.classify_segment_target(cfg12, n_shoot, "N5") == "end"
       and M.classify_segment_target(cfg12, n_shoot, "D9") == "path"),
      str([M.classify_segment_target(cfg12, n_shoot, c) for c in ("C9", "N5", "D9")]))

# ---- MissionTask：末点带 * ⇒ 真的进三段式停站并开火 ----
rig12 = Rig(cfg=cfg_of(reset="H9,x"))
rig12.start(lay_shoot)
rig12.bridge.set_aim_status(AimStatus.TRACK)    # 云台已锁定（ACQUIRE 的前置）
rig12.step(3)                                   # PLANNING → 段下发 → 路线载入
route12 = rig12.bb.get(P_ROUTE)
check("12.8 执行路线末航点 = C9 且带 shoot 标记（'C9*' 真的落进 Waypoint）",
      route12 is not None and route12.waypoints[-1].block == "C9"
      and route12.waypoints[-1].shoot is True
      and route12.waypoints[-1].token == "C9*",
      f"{[w.token for w in route12.waypoints] if route12 else None}")
phases12: list[str] = []
mission_phases12: list[str] = []
stop_seen: dict = {}
fired12 = False
done12: dict | None = None
for _ in range(900):
    rig12.bridge.set_vision(dx=0, dy=0, cls=1, box_w=45, box_h=55)   # 目标持续可见
    rig12.step(1, drive=True)
    _sp = rig12.bb.get(P_STOP_POINT)
    _ph = getattr(_sp, "phase", None)
    # P_STOP_POINT 的**初值**是默认 StopPointState(phase=SETTLE)：只在真进停站后采信
    if _ph and (rig12.mission.stop_context is not None or _ph == StopPhase.DONE):
        if not phases12 or phases12[-1] != _ph:
            phases12.append(_ph)
    mission_phases12.append(rig12.mission.phase)
    _ctx = rig12.mission.stop_context
    if _ctx is not None and not stop_seen:
        stop_seen = {"block": _ctx.block, "kind": _ctx.kind, "index": _ctx.index}
    if rig12.mission.phase == MissionPhase.STOP and not fired12:
        rig12.bridge.add_shots(3)               # 模拟"打进 3 发"（shots_per_stop 默认 3）
        fired12 = True
    _snap = rig12.bb.get(M.P_CAR_STATE)
    if isinstance(_snap, dict) and _snap.get("phase") == M.STEP_PHASE_SEGMENT_DONE:
        done12 = dict(_snap)
        break
check("12.9 ★ MissionTask 进三段式停站：SETTLE→ACQUIRE→FIRE→DONE 依次走完",
      phases12[:4] == [StopPhase.SETTLE, StopPhase.ACQUIRE, StopPhase.FIRE, StopPhase.DONE],
      str(phases12))
check("12.10 停站上下文 = C9 / SHOOT（不是普通航点跑完）",
      stop_seen.get("block") == "C9" and stop_seen.get("kind") == StopKind.SHOOT,
      str(stop_seen))
check("12.11 开火门真的开过、且收尾为关（发数增量达标即离站）",
      True in rig12.bridge.fire_gate and rig12.bridge.fire_gate[-1] is False,
      f"fire_gate 首次 {rig12.bridge.fire_gate[:6]} … 末态 {rig12.bridge.fire_gate[-2:]}")
check("12.12 ★ 打完才判段完成：P_CAR_STATE(SEGMENT_DONE) 带 stop_kind='shoot' / target=C9",
      done12 is not None and done12.get("phase") == M.STEP_PHASE_SEGMENT_DONE
      and done12.get("stop_kind") == "shoot" and done12.get("target_cell") == "C9"
      and done12.get("block") == "C9",
      str(done12))
_info12 = rig12.mission.segment_status()
check("12.13 MissionTask 完成单同样带 stop_kind（与 P_CAR_STATE 同源，机端可核对）",
      (_info12.get("done") or {}).get("stop_kind") == "shoot"
      and _info12.get("stop_kind") == "shoot",
      f"status={_info12.get('stop_kind')!r} done={(_info12.get('done') or {}).get('stop_kind')!r}")
check("12.14 StepLoop 状态与日志留痕：停站 shoot + '段末射击停站：C9'",
      rig12.loop.status()["stop_kind"] == "shoot"
      and bool(rig12.logs_with("段末射击停站：C9"))
      and any("C9（SHOOT）" in ln for ln in rig12.logs),
      f"{rig12.loop.status()['stop_kind']!r} / {rig12.logs_with('段末射击停站：C9')[:1]}")
rig12.close()

# ---- 雷格段：不标 *、不停站，但 stop_kind 如实报 mine ----
rig13 = Rig(cfg=cfg_of(reset="H9,x"))
rig13.start(sim_layout(target="I8"))
rig13.step(3)
route13 = rig13.bb.get(P_ROUTE)
check("12.15 雷格段路线末航点 I8 **无** shoot 标记（不会停站开火）",
      route13 is not None and route13.waypoints[-1].block == "I8"
      and route13.waypoints[-1].shoot is False,
      f"{[w.token for w in route13.waypoints] if route13 else None}")
done13: dict | None = None
entered_stop13 = False
for _ in range(600):
    rig13.step(1, drive=True)
    if rig13.mission.phase == MissionPhase.STOP:
        entered_stop13 = True
    _snap = rig13.bb.get(M.P_CAR_STATE)
    if isinstance(_snap, dict) and _snap.get("phase") == M.STEP_PHASE_SEGMENT_DONE:
        done13 = dict(_snap)
        break
check("12.16 雷格段完成：P_CAR_STATE stop_kind='mine'、且**从未**进 STOP 三段式",
      done13 is not None and done13.get("stop_kind") == "mine" and not entered_stop13
      and done13.get("block") == "I8",
      f"{done13} / entered_stop={entered_stop13}")
rig13.close()

# ---- 终点段：stop_kind='end'、不标 *、不停站 ----
rig14 = Rig(cfg=cfg_of(reset="H9,x"))
rig14.start(sim_layout(target="N5"))
done14: dict | None = None
entered_stop14 = False
for _ in range(3000):
    rig14.step(1, drive=True)
    if rig14.mission.phase == MissionPhase.STOP:
        entered_stop14 = True
    _snap = rig14.bb.get(M.P_CAR_STATE)
    if isinstance(_snap, dict) and _snap.get("phase") == M.STEP_PHASE_SEGMENT_DONE:
        done14 = dict(_snap)
        break
rig14.step(1)
check("12.17 终点段：stop_kind='end'、block=N5、FINISH，且从未进 STOP（终点不是打靶点）",
      done14 is not None and done14.get("stop_kind") == "end"
      and done14.get("block") == "N5" and not entered_stop14
      and rig14.loop.phase == M.STEP_PHASE_FINISH,
      f"{done14} / phase={rig14.loop.phase} / entered_stop={entered_stop14}")
rig14.close()

# ---- 空段（车已在目标格）：不走规划器也要如实报 shoot ----
rig15 = Rig(cfg=cfg_of(reset="C9,x"))
rig15.start(sim_layout(target="C9"))
rig15.step(2)
_done15 = [s for s in rig15.car_states
           if s.get("phase") == M.STEP_PHASE_SEGMENT_DONE]
check("12.18 空段（车已在 C9）也报 stop_kind='shoot'：机端不会把'已到位待打靶'当普通到达",
      bool(_done15) and _done15[-1].get("stop_kind") == "shoot"
      and _done15[-1].get("target_cell") == "C9"
      and "空段" in str(_done15[-1].get("reason")),
      str(_done15[-1] if _done15 else None))
rig15.close()

# =========================================================================== #
# 13. ★2026-10-18 现场回归：ROS 图自检 / 位姿就绪门 / 运行期位姿看门狗
# =========================================================================== #
section("13. ★现场 pose_stale 回归防线：订阅契约 / 唯一 bridge / odom 覆盖信念 / 无 odom⇒exit5")

_req = RIO.required_subscription_topics()
_topic_odom = CT.topic("odom")
check("13.1 订阅契约：/odom/localization 在必订清单里（QoS=best_effort depth=10，与旧构建逐字相同）",
      _req.get("odom") == "/odom/localization"
      and _topic_odom.name == "/odom/localization"
      and _topic_odom.qos.reliability == "best_effort"
      and _topic_odom.qos.depth == 10,
      f"{_req.get('odom')} / {_topic_odom.qos}")

cfg13 = cfg_of()
_clock13 = FakeClock()
bridge13 = FakeBridge(_clock13)
bb13, bus13 = Blackboard(), EventBus()
group13, skipped13 = M.build_tasks(bridge13, bb13, bus13, cfg13,
                                   log=lambda _s: None, skip={"drone"})
names13 = [t.name for t in group13._tasks]
check("13.2 布局模式装配：Localization/Safety/Tracker/Mission 都在组内（skip 只跳 drone）",
      {"Localization", "Safety", "TrackerTask", "MissionTask"} <= set(names13)
      and "DroneTask" not in names13,
      f"{names13} / skipped={skipped13}")
_holders = [t for t in group13._tasks if hasattr(t, "bridge")]
check("13.3 ★ 组内任务拿到的是同一个 bridge 实例（无二次构造 → 不会丢订阅）",
      bool(_holders) and all(t.bridge is bridge13 for t in _holders),
      f"{len(_holders)} 个任务持有 bridge；全部同一实例="
      f"{all(t.bridge is bridge13 for t in _holders)}")

# ---- 13.4 注入一帧 odom ⇒ P_POSE 被覆盖（钉死"P_POSE 从未写过"那类回归）----
_clock14 = FakeClock()
bridge14 = FakeBridge(_clock14)
bb14, bus14 = Blackboard(), EventBus()
bb14.set(P_SAFETY, SafetyState(ok=True), "test")
_seed14 = geo.Pose(*geo.block_to_meters("N10"), geo.HEADING_Y)      # 种子 yaw=90°
bb14.set(P_POSE, _seed14, "test")
bridge14.set_pose_at_block("N10", yaw=0.0)                           # 注入一帧 odom（yaw=0）
loc14 = LocalizationTask(bridge14, bb14, bus14, cfg13, log=lambda _s: None)
loc14.setup()
loc14.tick()
_pose14 = bb14.get(P_POSE)
check("13.4 ★ 注入一帧 odom ⇒ P_POSE 被写入且 belief 从种子(90°)被覆盖成 odom(0°)",
      isinstance(_pose14, geo.Pose) and abs(_pose14.yaw) < 1e-9
      and abs(_pose14.x - geo.block_to_meters("N10")[0]) < 1e-9,
      f"种子 yaw=90.0° → P_POSE yaw="
      f"{geo.deg(_pose14.yaw) if isinstance(_pose14, geo.Pose) else 'None'}")
check("13.5 P_POSE_RAW / P_ODOM_AGE 同时落盘（排障能分辨原始帧与信念）",
      bb14.get(P_POSE_RAW) is not None and bb14.get(P_ODOM_AGE) is not None,
      f"raw={bb14.get(P_POSE_RAW)} age={bb14.get(P_ODOM_AGE)}")

# ---- 13.6 无 odom ⇒ 位姿就绪门 False + 打印三条照抄命令 ----
_cfg_gate = cfg_of().override("step.pose_ready_wait_s", 0.3)
_logs13: list[str] = []
_ok_none = M._wait_for_pose_ready(FakeBridge(FakeClock()), _cfg_gate,
                                  stop_flag={"stop": False},
                                  log=_logs13.append, sleep=lambda _s: None)
_text13 = "\n".join(_logs13)
check("13.6 ★ 无 odom ⇒ 位姿就绪门 False，且打印 topic info / node list / /proc/self/environ 三条命令",
      _ok_none is False and "位姿就绪门超时" in _text13
      and "ros2 topic info" in _text13 and "ros2 node list" in _text13
      and "/proc/self/environ" in _text13,
      _text13[-260:])

# ---- 13.7 env 自检 ----
_errors13, _warns13 = RIO.env_problems({"ROS_DOMAIN_ID": "0", "DZ_ROS_DOMAIN": "118",
                                        "ROS_LOCALHOST_ONLY": "0", "RMW_IMPLEMENTATION": ""})
check("13.7 ★ 域号分叉（ROS_DOMAIN_ID=0 vs DZ_ROS_DOMAIN=118）⇒ env 自检报 error",
      bool(_errors13) and "不一致" in _errors13[0], str(_errors13))
_errors_ok, _warns_ok = RIO.env_problems({"ROS_DOMAIN_ID": "118", "DZ_ROS_DOMAIN": "118",
                                          "ROS_LOCALHOST_ONLY": "0", "RMW_IMPLEMENTATION": ""})
_errors_none, _warns_none = RIO.env_problems({"ROS_DOMAIN_ID": "", "DZ_ROS_DOMAIN": "",
                                              "ROS_LOCALHOST_ONLY": "", "RMW_IMPLEMENTATION": ""})
check("13.8 域号一致 ⇒ 无 error；没 source dzros.env ⇒ 只告警（不阻断 test/bench）",
      _errors_ok == [] and bool(_warns_none) and _errors_none == [],
      f"ok_errs={_errors_ok} none_warns={_warns_none}")

# ---- 13.9 运行期位姿看门狗：DRIVING 中断流 ⇒ ROS_LOST ----
rig15 = Rig(cfg=cfg_of().override("step.odom_watch_s", 0.3))
rig15.start(layout_of(known=["N10", "M10", "L10"], target="M10", version=1, segment_id=1))
rig15.step(3)
_phase15 = rig15.loop.phase
rig15.step(40)                                  # 冻结位姿：pose_age 单调增长
check("13.9 ★ DRIVING 中 /odom 断流 ⇒ 运行期看门狗判 ROS_LOST（不再半死跑）",
      _phase15 == M.STEP_PHASE_DRIVING and rig15.loop.phase == M.STEP_PHASE_ROS_LOST
      and bool(rig15.logs_with("运行期位姿看门狗")),
      f"{_phase15} → {rig15.loop.phase}")
_why15 = rig15.loop.run(stop_flag={"stop": False}, sleep=lambda _s: None)
check("13.10 ROS_LOST 后主循环返回 ros_lost（run_live 映射退出码 5）",
      _why15 == "ros_lost", str(_why15))
rig15.close()

# ---- 13.11/13.12 run_live 端到端：缺 odom ⇒ exit 5；odom 就绪 ⇒ 正常进步进循环 ----


def _run_live_fake(ready: bool, *, wait_s: float = 0.3, link: bool = True,
                   endpoints: dict | None = None, subscriptions: list | None = None):
    """用 NO-ROS 替身跑一遍 run_live 的布局分支（返回 rc / 步进调用次数 / 日志）。

    link=False ⇒ 图上 odom 端点缺失（契约层）；ready=False 且 link=True ⇒ 端点齐但无数据（位姿门）。
    subscriptions 给定时覆盖"本节点已创建订阅"清单（用于构造装配缺失场景）。
    """
    calls = {"step": 0}
    logs: list[str] = []
    names = ("install_stop_signals", "build_tasks", "start_banner", "finish_report",
             "_start_layout_listener", "_wait_for_field_layout", "_run_step_loop")
    saved = {name: getattr(M, name) for name in names}
    saved_bridge = _rz.RosBridge

    class _Bridge(_FakeLiveBridge):
        READY = ready
        LINK = link
        ENDPOINTS = dict(endpoints or {})

        def subscription_topics(self):
            if subscriptions is not None:
                return list(subscriptions)
            return super().subscription_topics()

    def _step(loop, *, stop_flag, alive=None, log=None):
        calls["step"] += 1
        return "finish"

    try:
        _rz.RosBridge = _Bridge
        M.install_stop_signals = lambda *a, **k: []
        M.finish_report = lambda *a, **k: None
        M.start_banner = lambda *a, **k: None
        M._start_layout_listener = lambda *a, **k: _FakeLiveDrone()
        _lay = M.normalize_step_layout(
            layout_of(known=["N10", "M10"], target="M10"))[0]
        M._wait_for_field_layout = lambda *a, **k: dict(_lay)
        M.build_tasks = lambda *a, **k: (_FakeGroup(), [])
        M._run_step_loop = _step
        cfg = Config.from_dict({"planner": {"auto_plan_enabled": True},
                                "route": {"tokens": ["N10", "N5,y"]},
                                "step": {"pose_ready_wait_s": wait_s}})
        rc = M.run_live(cfg, log=logs.append)
    finally:
        _rz.RosBridge = saved_bridge
        for name, value in saved.items():
            setattr(M, name, value)
    return rc, calls, logs


rc_c, calls_c, logs_c = _run_live_fake(False, link=False)
check("13.11 ★ 图上缺 odom 端点 ⇒ 启动契约拦截（退出码 6，不是泛泛 5）、不进步进循环",
      rc_c == 6 and calls_c["step"] == 0
      and any("启动契约未通过" in s for s in logs_c),
      f"rc={rc_c} step_calls={calls_c['step']}")
rc_p, calls_p, logs_p = _run_live_fake(False, link=True)
check("13.12 端点齐但无 odom 数据 ⇒ 位姿就绪门 exit 5（与契约 6 分层定位）",
      rc_p == 5 and calls_p["step"] == 0
      and any("位姿就绪门超时" in s for s in logs_p),
      f"rc={rc_p} step_calls={calls_p['step']}")
rc_ok, calls_ok, logs_ok = _run_live_fake(True)
check("13.13 同一装配下 odom 就绪 ⇒ 位姿门放行、进入步进循环并正常收尾",
      rc_ok == 0 and calls_ok["step"] == 1
      and any("位姿就绪门通过" in s for s in logs_ok),
      f"rc={rc_ok} step_calls={calls_ok['step']}")
# =========================================================================== #
# 14. ★缺口 A/E：启动契约（sub/pub 对账，exit 6）+ 布局新鲜度（step.layout_ttl_s）
# =========================================================================== #
section("14. ★启动契约 sub/pub 对账（exit 6）+ 布局新鲜度 layout_stale")

# ---- 14.1 正常图：契约通过，required 行 odom(sub)/pursuit_angle(pub) 全绿 ----
_good = _FakeLiveBridge()
_rows_logs: list[str] = []
_ok14, _rows14 = M.startup_contract(_good, "live", log=_rows_logs.append)
_req14 = {r["key"]: r for r in _rows14}
check("14.1 正常图 ⇒ startup_contract ok；odom(sub)/pursuit_angle(pub) 两行 required 且 ok",
      _ok14 and _req14["odom"]["ok"] and _req14["odom"]["blocking"]
      and _req14["pursuit_angle"]["ok"] and _req14["pursuit_angle"]["blocking"]
      and "启动契约" in "\n".join(_rows_logs),
      f"ok={_ok14} rows={len(_rows14)}")

# ---- 14.2 缺 pub 端点（底盘没订阅 pursuitAngle）⇒ 契约 False ----
class _NoActuatorBridge(_FakeLiveBridge):
    ENDPOINTS = {"pursuitAngle": (1, 0)}


_ok_bad, _rows_bad = M.startup_contract(_NoActuatorBridge(), "live")
_bad14 = {r["key"]: r for r in _rows_bad}
check("14.2 ★ pursuitAngle 订阅方 0（底盘没听）⇒ 契约 False + 该行 ❌/blocking/note 可读",
      _ok_bad is False and _bad14["pursuit_angle"]["ok"] is False
      and _bad14["pursuit_angle"]["blocking"] is True
      and "订阅方 0" in _bad14["pursuit_angle"]["note"],
      str(_bad14["pursuit_angle"]))

# ---- 14.3 本节点没建 odom 订阅（装配层）⇒ 契约 False ----
class _NoSubBridge(_FakeLiveBridge):
    def subscription_topics(self):
        return [t for t in super().subscription_topics() if t != "/odom/localization"]


_ok_ns, _rows_ns = M.startup_contract(_NoSubBridge(), "live")
_ns14 = {r["key"]: r for r in _rows_ns}
check("14.3 ★ 本节点未创建 /odom/localization 订阅（装配层）⇒ 契约 False + note 指明",
      _ok_ns is False and _ns14["odom"]["ok"] is False
      and "未创建订阅" in _ns14["odom"]["note"], str(_ns14["odom"]))

# ---- 14.4 DZ_SKIP_CONTRACT=1 ⇒ 整体跳过（bench/离线联调）----
_old_skip14 = os.environ.get(M.CONTRACT_SKIP_ENV)
os.environ[M.CONTRACT_SKIP_ENV] = "1"
try:
    _ok_skip, _rows_skip = M.startup_contract(_NoActuatorBridge(), "live")
finally:
    if _old_skip14 is None:
        os.environ.pop(M.CONTRACT_SKIP_ENV, None)
    else:
        os.environ[M.CONTRACT_SKIP_ENV] = _old_skip14
check("14.4 DZ_SKIP_CONTRACT=1 ⇒ 契约整体跳过（图不全也 ok，供 bench）",
      _ok_skip is True and "跳过" in _rows_skip[0]["note"], str(_rows_skip[:1]))

# ---- 14.5 布局过期 ⇒ WAIT_INFO(layout_stale)，一次都不规划 ----
rig14 = Rig(cfg=cfg_of().override("step.layout_ttl_s", 0.5))
rig14.start(layout_of(known=["N10", "M10", "L10"], target="M10", version=1, segment_id=1))
rig14.clock.advance(2.0)                 # 布局在手上放了 2s > ttl 0.5s
rig14.step(2)
_st14 = rig14.loop.status()
check("14.5 ★ 布局过期（2s > ttl 0.5s）⇒ WAIT_INFO(reason=layout_stale)，一次都不规划",
      rig14.loop.phase == M.STEP_PHASE_WAIT_INFO
      and _st14["reason"].startswith("layout_stale")
      and _st14["replans"] == 0 and _st14["layout_stale"] is True
      and rig14.bb.get(P_ROUTE) is None,
      f"{rig14.loop.phase} / {_st14['reason'][:56]} / replans={_st14['replans']}")

# ---- 14.6 新布局（version 递增）到达 ⇒ 计时清零、恢复规划 ----
rig14.feed(layout_of(known=["N10", "M10", "L10"], target="M10", version=2, segment_id=2))
rig14.step(3)
_st14b = rig14.loop.status()
check("14.6 ★ 新布局 v2 到达 ⇒ 恢复规划并下发（DRIVING / replans=1 / stale=False）",
      rig14.loop.phase == M.STEP_PHASE_DRIVING and _st14b["replans"] == 1
      and rig14.bb.get(P_ROUTE) is not None
      and _st14b["layout_stale"] is False,
      f"{rig14.loop.phase} replans={_st14b['replans']} stale={_st14b['layout_stale']}")
rig14.close()

# ---- 14.7 ttl=0 ⇒ 关闭（默认闸可回滚）----
rig14b = Rig(cfg=cfg_of().override("step.layout_ttl_s", 0.0))
rig14b.start(layout_of(known=["N10", "M10", "L10"], target="M10"))
rig14b.clock.advance(30.0)
rig14b.step(3)
check("14.7 step.layout_ttl_s=0 ⇒ 关闭新鲜度门（放 30s 仍照常规划）",
      rig14b.loop.status()["layout_stale"] is False
      and rig14b.loop.phase in (M.STEP_PHASE_DRIVING, M.STEP_PHASE_SEGMENT_DONE),
      f"{rig14b.loop.phase} / stale={rig14b.loop.status()['layout_stale']}")
rig14b.close()

# ---- 14.8 新鲜度只挡"新规划"，不影响已下发段执行 ----
rig14c = Rig(cfg=cfg_of().override("step.layout_ttl_s", 0.3))
rig14c.start(layout_of(known=["N10", "M10", "L10"], target="M10"))
rig14c.step(2)                           # 规划完成 → DRIVING
rig14c.clock.advance(5.0)                # 运行中布局变旧
rig14c.step(1, drive=True)
check("14.8 布局过期只挡新规划：DRIVING 中照常执行当前段（不停车、不判失败）",
      rig14c.loop.phase == M.STEP_PHASE_DRIVING
      and rig14c.loop.status()["layout_stale"] is True,
      f"{rig14c.loop.phase} / stale={rig14c.loop.status()['layout_stale']}")
rig14c.close()
finish()
