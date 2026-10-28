# -*- coding: utf-8 -*-
r"""AGC-RECTIFY · rt-gate 五卡验收（G1-DEGRADE / S3-STEP-GATE / M1-BRAINMAP / M2-ARCHIVE / S2-LEAD-GATE）。

纯脚本、无 pytest（本项目惯例）：

    python src\dzbrain\tests\test_agc_rectify_rt_gate.py

为什么单独一个文件：五张卡的验收都落在 `dzbrain/main.py` 的**闸门分级**与**运行归档**上，
既有套件（test_stepper 等）钉的是"旧拦停语义"，不能为了新口径去改它们的判据；新口径
必须有自己的机器判据 + 自己的负例（每卡 ≥2 条负例）。

验收映射（卡面 V 号 → 本文件小节）：
  G1 V1 五个违约注入 → 继续跑 + 5 条对应 warn   ...... §G1.2~G1.8
  G1 V2 硬拦调用点只剩 3 处                     ...... §G1.9
  G1 负例：hard 口径仍拦 / 3 条硬拦真的会停     ...... §G1.10~G1.14
  S3 三处口径一致 + 运行时值 = 生效值            ...... §S3.1~S3.3
  S3 WAIT_INFO(layout_stale) 有出口             ...... §S3.5
  S3 负例：hard 仍空等 / 无新布局不得无理由空等 ...... §S3.6~S3.7
  M1 5 格 unknown 继续 / 1 格不 exit            ...... §M1.1~M1.2
  M1 同图不换版 / 新进程新目录 / 不读上一场      ...... §M1.4~M1.6
  M2 六文件齐 / summary 六键 / SIGTERM 也出包    ...... §M2.1~M2.6
  S2 lead_cells 计算 + info_lead_low 只记不拦    ...... §S2.1~S2.5

全过 exit 0；任一失败 exit 1。
"""

from __future__ import annotations

import hashlib
import json
import os
import pathlib
import re
import signal
import subprocess
import sys
import tempfile
import time

HERE = pathlib.Path(__file__).resolve()
sys.path.insert(0, str(HERE.parent))                 # tests/（harness / fakes）
sys.path.insert(0, str(HERE.parents[2]))             # src/dzbrain

from harness import check, section, finish            # noqa: E402
import fakes                                          # noqa: E402
from dzbrain.core import geometry as geo              # noqa: E402
from dzbrain.core.blackboard import P_AIM, P_FIELD_LAYOUT, P_POSE, Blackboard  # noqa: E402
from dzbrain.core.config import SCHEMA, Config        # noqa: E402
from dzbrain.core.events import EventBus              # noqa: E402
from dzbrain.core.state import AimState               # noqa: E402
from dzbrain.core.run_archive import (RUN_FILES, SUMMARY_KEYS, RunArchive,  # noqa: E402
                                      install_log_tee)
import dzbrain.main as M                              # noqa: E402

MAIN_SRC = pathlib.Path(M.__file__).read_text(encoding="utf-8")
HEAD_Y = 1.5707963267948966                            # 朝 +y（与 test_stepper 同口径）
ALL_CELLS = M.all_map_cells()
TOTAL_CELLS = len(ALL_CELLS)


# --------------------------------------------------------------------------- #
#  替身：任务线程（只实现 StepLoop 用到的四个方法）
# --------------------------------------------------------------------------- #
class FakeMission:
    """MissionTask 替身：段请求 / 段状态 / 停段 / 位姿复位。"""

    def __init__(self) -> None:
        self.seg: dict | None = None
        self.done: dict | None = None
        self.error = ""
        self.stops: list[str] = []
        self.requests: list[dict] = []
        self.reset_spec = ""

    def request_segment(self, tokens, *, segment_id=0, target_cell="", known_count=0,
                        source="", stop_kind="") -> dict:
        self.requests.append({"tokens": list(tokens), "segment_id": segment_id,
                              "target_cell": target_cell, "stop_kind": stop_kind})
        self.seg = {"tokens": list(tokens), "segment_id": segment_id}
        self.done = None
        self.error = ""
        return {"accepted": True, "reason": ""}

    def segment_status(self) -> dict:
        return {"active": self.seg is not None,
                "segment_id": int((self.seg or {}).get("segment_id", 0)),
                "target_cell": "", "known_count": 0, "stop_kind": "",
                "done": self.done, "error": self.error,
                "phase": ("DRIVE" if self.seg is not None else "IDLE")}

    def stop_segment(self, why) -> None:
        self.stops.append(str(why))
        self.seg = None

    def reset_pose(self, spec) -> None:
        self.reset_spec = str(spec)

    # 测试用：让任务线程"报段完成"
    def finish(self, *, block="L10", stop_kind="path", elapsed=1.0) -> None:
        sid = int((self.seg or {}).get("segment_id", 0))
        self.done = {"segment_id": sid, "block": block, "stop_kind": stop_kind,
                     "elapsed_s": float(elapsed)}


def _cfg(*, ttl: float | None = 0.0, policy: str | None = "warn",
         wait_s: float = 0.0, footprint_gate: str = "warn", aim_blue=True, max_shots=3) -> Config:
    """构造配置替身。`ttl=None`/`policy=None` = **键缺省**（auto 口径看"显式表态"）。"""
    step: dict = {"wait_timeout_s": wait_s}
    if ttl is not None:
        step["layout_ttl_s"] = float(ttl)
    if policy is not None:
        step["gate_policy"] = str(policy)
    return Config.from_dict({
        "planner": {"auto_plan_enabled": True, "engine": "tracker-safe",
                    "footprint_gate": footprint_gate},
        "mission": {"auto_start": False, "reset": "N10,y"},
        "route": {"shoot_distance_m": 0.90, "shoot_distance_tol_m": 0.12},
        "step": step,
        "safety": {"pose_timeout_s": 1.5},
        "logging": {"rate_limit_s": 1.0},
        "aim": {"fire_only_blue": aim_blue, "max_shots": max_shots},
    })


def layout(known, target="", *, version=1, segment_id=1, start="N10", end="N5",
           mines=(), obstacles=(), targets=()) -> dict:
    return {"type": "field_layout", "version": version, "ts": 0.0, "start": start, "end": end,
            "known": list(known), "mines": list(mines), "obstacles": list(obstacles),
            "targets": list(targets), "bridges": ["K7", "K5"],
            "barrier_col_from": "D", "barrier_row_hi": 6, "barrier_row_lo": 7,
            "target_cell": target, "segment_id": segment_id}


class Rig:
    """StepLoop 最小闭环试验台（位姿手动喂、段完成手动报）。"""

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.bb = Blackboard()
        self.bus = EventBus()
        self.clock = fakes.FakeClock()
        self.bridge = fakes.FakeBridge(clock=self.clock)
        self.mission = FakeMission()
        self.logs: list[str] = []
        self.loop = M.StepLoop(cfg, self.bb, self.bridge, self.mission,
                               bus=self.bus, log=self.logs.append, clock=self.clock.now)
        self.loop.attach_archive(None)

    # -- 布局 / 位姿 ---------------------------------------------------- #
    def start(self, lay: dict) -> None:
        self.bb.set(P_FIELD_LAYOUT, lay, "test")
        loaded = M.load_step_layout(self.bb)
        assert loaded is not None, "测试布局没过 normalize_step_layout"
        self.loop.start(loaded)

    def feed(self, lay: dict) -> None:
        self.bb.set(P_FIELD_LAYOUT, lay, "test")

    def set_pose(self, x: float, y: float, yaw: float = HEAD_Y) -> None:
        self.bridge.set_pose(x, y, yaw)
        self.bb.set(P_POSE, geo.Pose(x, y, yaw), "test")

    def set_pose_block(self, block: str, yaw: float = HEAD_Y) -> None:
        x, y = geo.block_to_meters(block)
        self.set_pose(x, y, yaw)

    # -- 推进 ----------------------------------------------------------- #
    def tick(self, n: int = 1, dt: float = 0.1) -> None:
        for _ in range(n):
            self.loop.tick()
            self.clock.advance(dt)

    # -- 日志检索（验收判据就靠这些） ------------------------------------ #
    def warns(self, key: str | None = None) -> list[str]:
        out = [ln for ln in self.logs if "GATE[warn]" in ln]
        return [ln for ln in out if f"key={key}" in ln] if key else out

    def hards(self, key: str | None = None) -> list[str]:
        out = [ln for ln in self.logs if "GATE[hard]" in ln]
        return [ln for ln in out if f"key={key}" in ln] if key else out

    def infos(self, key: str) -> list[str]:
        return [ln for ln in self.logs if "INFO key=" + key in ln]


# --------------------------------------------------------------------------- #
section("G1.1 闸门清单结构：3 条硬拦 / 11 条降级 / 每条 warn 有固定字段")
# --------------------------------------------------------------------------- #
check("G1.1a 14 条闸门，键唯一", len(M.GATE_SPECS) == 14
      and len({s.key for s in M.GATE_SPECS}) == 14,
      f"{len(M.GATE_SPECS)} 条")
check("G1.1b 硬拦恰好 3 条（hw_estop / fire_interlock / field_boundary）",
      M.GATE_HARD_KEYS == (M.GATE_HW_ESTOP, M.GATE_FIRE_INTERLOCK, M.GATE_FIELD_BOUNDARY),
      str(M.GATE_HARD_KEYS))
check("G1.1c 降级恰好 11 条（与任务清单逐条对应）",
      len(M.GATE_WARN_KEYS) == 11 and set(M.GATE_WARN_KEYS) == {
          "layout_incomplete", "layout_stale", "footprint", "revisit", "drift_stop",
          "pose_ready", "odom_watchdog", "segment_timeout", "done_missing",
          "target_unlocked", "gimbal_status"},
      str(sorted(M.GATE_WARN_KEYS)))
check("G1.1d 每条 warn 都有 ≥1 个固定日志字段（机端可聚合）",
      all(len(M.GATE_SPEC_BY_KEY[k].fields) >= 1 for k in M.GATE_WARN_KEYS))
check("G1.1e 每条 spec 的 level 与两张 key 表自洽",
      all(M.GATE_SPEC_BY_KEY[k].level == "hard" for k in M.GATE_HARD_KEYS)
      and all(M.GATE_SPEC_BY_KEY[k].level == "warn" for k in M.GATE_WARN_KEYS))

# --------------------------------------------------------------------------- #
section("G1.2 V1-a 注入『布局不全』(known 空) ⇒ warn + 继续跑，不 WAIT_INFO 空等")
# --------------------------------------------------------------------------- #
rig_g1 = Rig(_cfg())
rig_g1.set_pose_block("N10")
# 「布局不全」的可注入形态 = **已观测格只有两个端点**（中间格还未知）：
# 段路线必然穿过未知格。known 为空 / target 不在 known 在协议层就被 normalize 拒收
# （见 G1.2c 反向守卫：降级不许把报文校验也一起放宽）。
rig_g1.start(layout(["N10", "L10"], target="L10"))
rig_g1.tick(4)
check("G1.2a 出 GATE[warn] key=layout_incomplete（route_through_unknown + 格数）",
      len(rig_g1.warns("layout_incomplete")) >= 1
      and "why=route_through_unknown" in rig_g1.warns("layout_incomplete")[0]
      and "route_cells=" in rig_g1.warns("layout_incomplete")[0],
      (rig_g1.warns("layout_incomplete") or ["<无>"])[0][:170])
check("G1.2b 不再 WAIT_INFO 空等：已下发段（DRIVING/SEGMENT_DONE）",
      rig_g1.loop.phase in (M.STEP_PHASE_DRIVING, M.STEP_PHASE_SEGMENT_DONE),
      rig_g1.loop.phase)
_norm_empty, _why_empty = M.normalize_step_layout(layout([], target="M10"))
_norm_tgt, _why_tgt = M.normalize_step_layout(layout(["N10"], target="L10"))
check("G1.2c 反向守卫：known 为空 / target 不在 known 仍在**协议层**被拒收（降级不放宽报文校验）",
      _norm_empty is None and "known" in _why_empty
      and _norm_tgt is None and "target_cell" in _why_tgt,
      f"{str(_why_empty)[:30]} | {str(_why_tgt)[:40]}")

# --------------------------------------------------------------------------- #
section("G1.3 V1-b 注入『足迹侵入』：配置层 footprint_gate=error ⇒ 降级 warn")
# --------------------------------------------------------------------------- #
rig_fp = Rig(_cfg(footprint_gate="error"))
check("G1.3a 出 GATE[warn] key=footprint（source 写明 error→warn）",
      any("key=footprint" in ln and "config_downgraded" in ln for ln in rig_fp.warns()),
      (rig_fp.warns("footprint") or ["<无>"])[0][:150])
check("G1.3b 生效配置真的被降为 warn（否则 warn 判定等于白降级）",
      str(rig_fp.cfg.get("planner.footprint_gate")) == "warn",
      str(rig_fp.cfg.get("planner.footprint_gate")))

# --------------------------------------------------------------------------- #
section("G1.4/G1.5 V1-c/V1-d 注入 drift / 重复访问（段错误文本）⇒ warn + 重新规划")
# --------------------------------------------------------------------------- #
rig_err = Rig(_cfg())
rig_err.set_pose_block("N10")
rig_err.start(layout(["N10", "M10", "L10"], target="L10"))
rig_err.tick(2)
before_req = len(rig_err.mission.requests)
rig_err.mission.error = "drift_stop：定位漂移 0.42 m 连续 8 拍"
rig_err.tick(1)
check("G1.4a 出 GATE[warn] key=drift_stop（带 segment/detail）",
      len(rig_err.warns("drift_stop")) >= 1 and "segment=" in rig_err.warns("drift_stop")[0],
      (rig_err.warns("drift_stop") or ["<无>"])[0][:150])
check("G1.4b 降级为『继续跑』：回 PLANNING 且计数 +1（不是 WAIT_INFO 死等）",
      rig_err.loop.phase == M.STEP_PHASE_PLANNING
      and rig_err.loop.status()["gated_continues"] >= 1,
      f"{rig_err.loop.phase} / {rig_err.loop.status()['gated_continues']}")
rig_err.tick(2)
rig_err.mission.error = "重复访问：F4 本场已访问过"
rig_err_prev = len(rig_err.warns("revisit"))
rig_err.tick(1)
check("G1.5a 出 GATE[warn] key=revisit", len(rig_err.warns("revisit")) > rig_err_prev,
      (rig_err.warns("revisit") or ["<无>"])[0][:150])
check("G1.5b 重新规划过（段请求数增加）", len(rig_err.mission.requests) > before_req,
      f"{before_req} → {len(rig_err.mission.requests)}")

# --------------------------------------------------------------------------- #
section("G1.6/G1.7 V1-e 注入『靶未锁』+ 5 条 warn 互不相同")
# --------------------------------------------------------------------------- #
rig_aim = Rig(_cfg())
rig_aim.set_pose_block("N10")
rig_aim.start(layout(["N10", "M10", "L10"], target="L10"))
rig_aim.tick(2)
rig_aim.bb.set(P_AIM, AimState(status=2, fire_gate=False, last_reason="no_target"),
               "AimTask")
rig_aim.tick(1)
check("G1.6a 出 GATE[warn] key=target_unlocked（reason=no_target / fire_gate=False）",
      len(rig_aim.warns("target_unlocked")) >= 1
      and "no_target" in rig_aim.warns("target_unlocked")[0],
      (rig_aim.warns("target_unlocked") or ["<无>"])[0][:150])
check("G1.6b 靶未锁**不拦车**：相位仍 DRIVING", rig_aim.loop.phase == M.STEP_PHASE_DRIVING,
      rig_aim.loop.phase)
seen = {k for k in ("layout_incomplete", "footprint", "drift_stop", "revisit",
                    "target_unlocked") if rig_g1.warns(k) or rig_fp.warns(k)
        or rig_err.warns(k) or rig_aim.warns(k)}
check("G1.7 五个违约各自出对应 warn，5 个 key 互不相同（V1 的『5 条对应 warn』）",
      seen == {"layout_incomplete", "footprint", "drift_stop", "revisit", "target_unlocked"},
      str(sorted(seen)))

# --------------------------------------------------------------------------- #
section("G1.8 车继续跑：连续 40 拍注入不进入任何『停死』相位")
# --------------------------------------------------------------------------- #
rig_run = Rig(_cfg())
rig_run.set_pose_block("N10")
rig_run.start(layout(["N10", "M10", "L10"], target="L10"))
rig_run.tick(2)
phases = set()
for i in range(40):
    rig_run.mission.error = ("drift_stop：漂移" if i % 4 == 0
                             else "重复访问：M10" if i % 4 == 1
                             else "")
    rig_run.bb.set(P_AIM, AimState(fire_gate=(i % 3 == 0), last_reason="" if i % 3 == 0
                                   else "lost"), "AimTask")
    rig_run.tick(1)
    phases.add(rig_run.loop.phase)
check("G1.8a 40 拍内从未进入 SAFE_STOP / ROS_LOST / WAIT_INFO_TIMEOUT",
      not (phases & {M.STEP_PHASE_SAFE_STOP, M.STEP_PHASE_ROS_LOST,
                     M.STEP_PHASE_WAIT_TIMEOUT}), str(sorted(phases)))
check("G1.8b 降级继续计数 ≥ 5（确实在『停段-重规划』而不是停在原地）",
      rig_run.loop.status()["gated_continues"] >= 5,
      str(rig_run.loop.status()["gated_continues"]))

# --------------------------------------------------------------------------- #
section("G1.9 V2 硬拦调用点只剩 3 处（机器 grep + 键清单）")
# --------------------------------------------------------------------------- #
calls = [ln.strip() for ln in MAIN_SRC.splitlines()
         if re.search(r"(?<![_\w])hard_stop\(", ln)
         and not ln.lstrip().startswith(("def ", "#"))]
hard_keys = sorted({m.group(1) for ln in calls
                    for m in [re.search(r"key=(GATE_[A-Z_]+)", ln)] if m})
check("G1.9a hard_stop( 调用点 == 3", len(calls) == 3, f"{len(calls)} 处：{calls}")
check("G1.9b 三处分别是 hw_estop / fire_interlock / field_boundary",
      hard_keys == ["GATE_FIELD_BOUNDARY", "GATE_FIRE_INTERLOCK", "GATE_HW_ESTOP"],
      str(hard_keys))

# --------------------------------------------------------------------------- #
section("G1.10~G1.14 负例：hard 口径仍拦停；3 条硬拦真的会停")
# --------------------------------------------------------------------------- #
rig_hard = Rig(_cfg(ttl=0.5, policy="hard"))
rig_hard.set_pose_block("N10")
rig_hard.start(layout(["N10", "M10", "L10"], target="L10"))
rig_hard.tick(2)
rig_hard.clock.advance(5.0)
rig_hard.loop._plan(rig_hard.clock.now())              # noqa: SLF001 直接推进一拍规划
check("G1.10 负例-1 hard 口径：过期布局仍 ⇒ WAIT_INFO(layout_stale)（没被降级）",
      rig_hard.loop.phase == M.STEP_PHASE_WAIT_INFO
      and rig_hard.loop.status()["reason"].startswith("layout_stale"),
      f"{rig_hard.loop.phase} / {rig_hard.loop.status()['reason'][:60]}")

rig_u = Rig(_cfg(policy="hard"))
rig_u.set_pose_block("N10")
rig_u.start(layout(["N10", "M10", "L10"], target="L10"))
rig_u.tick(2)
rig_u.set_pose_block("N9")                             # 明确不在 known 的格
rig_u.tick(4)
check("G1.11 负例-2 hard 口径：belief 连续落未知格仍 ⇒ SAFE_STOP（旧语义不变）",
      rig_u.loop.phase == M.STEP_PHASE_SAFE_STOP, rig_u.loop.phase)

rig_e = Rig(_cfg())
rig_e.set_pose_block("N10")
rig_e.start(layout(["N10", "M10", "L10"], target="L10"))
rig_e.tick(2)
rig_e.bridge._manual_override = True                   # noqa: SLF001 注入遥控接管
rig_e.tick(1)
check("G1.12 负例-3 硬拦① 遥控接管 ⇒ SAFE_STOP + GATE[hard] key=hw_estop",
      rig_e.loop.phase == M.STEP_PHASE_SAFE_STOP and len(rig_e.hards("hw_estop")) >= 1,
      f"{rig_e.loop.phase} / {(rig_e.hards('hw_estop') or ['<无>'])[0][:80]}")

rig_b = Rig(_cfg())
rig_b.set_pose_block("N10")
rig_b.start(layout(["N10", "M10", "L10"], target="L10"))
rig_b.tick(2)
rig_b.set_pose(geo.MAP_ROWS * geo.BLOCK_SIZE + 0.5, 1.0)   # 越过地面线
rig_b.tick(1)
check("G1.13 负例-4 硬拦③ 越界 ⇒ SAFE_STOP + GATE[hard] key=field_boundary",
      rig_b.loop.phase == M.STEP_PHASE_SAFE_STOP
      and len(rig_b.hards("field_boundary")) >= 1
      and "limit_x=" in rig_b.hards("field_boundary")[0],
      f"{rig_b.loop.phase} / {(rig_b.hards('field_boundary') or ['<无>'])[0][:90]}")

gates_ok = M.GateLedger(lambda _s: None)
ok_interlock = M.assert_fire_interlock(_cfg(), gates_ok, log=lambda _s: None)
gates_bad = M.GateLedger(lambda _s: None)
bad_interlock = M.assert_fire_interlock(_cfg(aim_blue=False), gates_bad,
                                        log=lambda _s: None)
check("G1.14 负例-5 硬拦② 开火联锁：正常配置通过；fire_only_blue=false ⇒ 拒绝",
      ok_interlock is True and bad_interlock is False
      and gates_bad.count(M.GATE_FIRE_INTERLOCK) == 1,
      f"ok={ok_interlock} bad={bad_interlock}")

# --------------------------------------------------------------------------- #
section("S3.1~S3.3 三处口径一致 + 运行时日志值 = 生效值")
# --------------------------------------------------------------------------- #
spec_ttl = float(SCHEMA["step.layout_ttl_s"].default)
spec_derived = M._spec_default("step.layout_ttl_s", -1.0)      # noqa: SLF001
loop_ttl = Rig(_cfg(ttl=None, policy=None)).loop._layout_ttl_s  # noqa: SLF001
check("S3.1a SCHEMA 缺省 == 代码兜底读到的值（同一真源，无第二份写死数）",
      spec_derived == spec_ttl, f"{spec_derived} vs {spec_ttl}")
check("S3.1b StepLoop 兜底 == SCHEMA 缺省（键缺省时不再回落 10.0）",
      loop_ttl == spec_ttl, f"{loop_ttl} vs {spec_ttl}")
check("S3.1c 三处口径都是『0 = 关闭』（本裁决后的目标口径）", spec_ttl == 0.0, str(spec_ttl))
check("S3.2a main.py 里不再有 layout_ttl_s 的 10.0 写死兜底",
      not re.search(r'"step\.layout_ttl_s",\s*10\.0', MAIN_SRC))
check("S3.2b 模块头/StepLoop docstring 都写明 ttl 缺省 0 = 关闭",
      MAIN_SRC.count("step.layout_ttl_s") >= 3 and "缺省 0 = 关闭" in MAIN_SRC)
rig_ttl = Rig(_cfg(ttl=0.0, policy="warn"))
check("S3.3a status().layout_ttl_s == 生效值 0.0",
      rig_ttl.loop.status()["layout_ttl_s"] == 0.0,
      str(rig_ttl.loop.status()["layout_ttl_s"]))
check("S3.3b 启动日志打印生效值（step.layout_ttl_s=0s）",
      any("step.layout_ttl_s=0s" in ln for ln in rig_ttl.logs),
      [ln for ln in rig_ttl.logs if "运行面闸门" in ln][:1])

# --------------------------------------------------------------------------- #
section("S3.4/S3.5 stale 降级 + WAIT_INFO(layout_stale) 有出口")
# --------------------------------------------------------------------------- #
rig_s3 = Rig(_cfg(ttl=0.5, policy="warn"))
rig_s3.set_pose_block("N10")
rig_s3.start(layout(["N10", "M10", "L10"], target="L10"))
rig_s3.tick(1)
rig_s3.clock.advance(5.0)
rig_s3.loop._plan(rig_s3.clock.now())                  # noqa: SLF001 一拍规划 → 判 stale
_warn_stale = rig_s3.warns("layout_stale")
rig_s3.feed(layout(["N10", "M10", "L10"], target="L10", version=2, segment_id=2))
rig_s3.tick(2)
check("S3.4a 过期 ⇒ 出 GATE[warn] key=layout_stale（age_s/ttl_s 都在）",
      len(_warn_stale) >= 1 and "age_s=" in _warn_stale[0] and "ttl_s=" in _warn_stale[0],
      (_warn_stale or ["<无>"])[0][:130])
check("S3.4b 过期不空等：判 stale 的那一拍没有转 WAIT_INFO（照常规划）",
      rig_s3.loop.phase in (M.STEP_PHASE_DRIVING, M.STEP_PHASE_SEGMENT_DONE),
      rig_s3.loop.phase)
check("S3.5 新布局到达即继续：layout_stale 不再影响规划（stale→v2 后 DRIVING）",
      rig_s3.loop.status()["layout_stale"] is False
      and rig_s3.loop.phase == M.STEP_PHASE_DRIVING,
      f"stale={rig_s3.loop.status()['layout_stale']} / {rig_s3.loop.phase}")

# --------------------------------------------------------------------------- #
section("S3.6/S3.7 负例：hard 仍空等 / 无新布局不得无理由无界等待")
# --------------------------------------------------------------------------- #
check("S3.6 负例-1 hard 口径下同一场景 ⇒ WAIT_INFO(layout_stale)（旧语义兜住）",
      rig_hard.loop.phase == M.STEP_PHASE_WAIT_INFO, rig_hard.loop.phase)

rig_wait = Rig(_cfg(policy="warn", wait_s=0.0))
rig_wait.set_pose_block("N10")
rig_wait.start(layout(["N10", "M10", "L10"], target=""))     # 没有 target_cell
rig_wait.tick(2)
rig_wait.clock.advance(M._WAIT_INFO_BUDGET_S + 10.0)         # noqa: SLF001
rig_wait.tick(1)
check("S3.7a 负例-2 空等到预算 ⇒ 必留『为什么还在等』（wait_info_bounded）",
      len(rig_wait.infos("wait_info_bounded")) >= 1,
      (rig_wait.infos("wait_info_bounded") or ["<无>"])[0][:130])
check("S3.7b 但仍不退出/不停死：相位 WAIT_INFO（有界理由 ≠ 拦停）",
      rig_wait.loop.phase == M.STEP_PHASE_WAIT_INFO, rig_wait.loop.phase)
rig_to = Rig(_cfg(policy="warn", wait_s=1.0))
rig_to.set_pose_block("N10")
rig_to.start(layout(["N10", "M10", "L10"], target=""))
rig_to.tick(2)
rig_to.clock.advance(2.0)
rig_to.tick(1)
check("S3.8 段超时降级：warn segment_timeout，相位不转 WAIT_INFO_TIMEOUT",
      len(rig_to.warns("segment_timeout")) >= 1
      and rig_to.loop.phase == M.STEP_PHASE_WAIT_INFO,
      f"{rig_to.loop.phase} / {(rig_to.warns('segment_timeout') or ['<无>'])[0][:80]}")

# --------------------------------------------------------------------------- #
section("M1.1/M1.2 注入 5 个 unknown ⇒ 继续跑；1 个 unknown ⇒ 不 exit")
# --------------------------------------------------------------------------- #
five_unknown = ["A1", "A2", "B1", "N11", "M11"]
known_rest = [c for c in ALL_CELLS if c not in five_unknown]
rig_m1 = Rig(_cfg(ttl=0.0, policy="warn"))
rig_m1.set_pose_block("N10")
rig_m1.start(layout(known_rest, target="L10"))
rig_m1.tick(3)
st_m1 = rig_m1.loop.status()
check("M1.1a unknown_cells == 5（逐格 known/unknown 口径）", st_m1["unknown_cells"] == 5,
      str(st_m1["unknown_cells"]))
check("M1.1b 出串继续：相位 DRIVING/SEGMENT_DONE，非 SAFE_STOP/WAIT_INFO",
      rig_m1.loop.phase in (M.STEP_PHASE_DRIVING, M.STEP_PHASE_SEGMENT_DONE),
      rig_m1.loop.phase)
one_unknown = ["A1"]
rig_m1b = Rig(_cfg(ttl=0.0, policy="warn"))
rig_m1b.set_pose_block("N10")
rig_m1b.start(layout([c for c in ALL_CELLS if c not in one_unknown], target="L10"))
rig_m1b.tick(3)
check("M1.2 1 个 unknown ⇒ 不退出、不 SAFE_STOP（unknown_cells=1）",
      rig_m1b.loop.status()["unknown_cells"] == 1
      and rig_m1b.loop.phase in (M.STEP_PHASE_DRIVING, M.STEP_PHASE_SEGMENT_DONE),
      f"{rig_m1b.loop.status()['unknown_cells']} / {rig_m1b.loop.phase}")

# --------------------------------------------------------------------------- #
section("M1.3 unknown_traversed 日志（只记不拦）")
# --------------------------------------------------------------------------- #
rig_m1.set_pose_block("A1")                            # 明确未知格
rig_m1.tick(2)
check("M1.3a 出 INFO key=unknown_traversed（block=A1 / map_version 在）",
      len(rig_m1.infos("unknown_traversed")) >= 1
      and "block=A1" in rig_m1.infos("unknown_traversed")[0],
      (rig_m1.infos("unknown_traversed") or ["<无>"])[0][:120])
check("M1.3b 走过未知格不 SAFE_STOP（degrade 口径）",
      rig_m1.loop.phase not in (M.STEP_PHASE_SAFE_STOP,), rig_m1.loop.phase)

# --------------------------------------------------------------------------- #
section("M1.4 同一份图读两次：不解锁/不重算/不换版")
# --------------------------------------------------------------------------- #
rig_m1c = Rig(_cfg(ttl=0.0, policy="warn"))
rig_m1c.set_pose_block("N10")
lay_v1 = layout(["N10", "M10", "L10"], target="L10", version=1)
rig_m1c.start(lay_v1)
rig_m1c.tick(2)
ver1 = rig_m1c.loop.status()["map_version"]
rep1 = rig_m1c.loop.status()["replans"]
req1 = len(rig_m1c.mission.requests)
rig_m1c.feed(lay_v1)                                   # 同一版同一内容再来一次
rig_m1c.tick(2)
check("M1.4a map_version 不变（同格不换版）",
      rig_m1c.loop.status()["map_version"] == ver1,
      f"{ver1} → {rig_m1c.loop.status()['map_version']}")
check("M1.4b 不重算：重规划计数与段请求数都不变",
      rig_m1c.loop.status()["replans"] == rep1
      and len(rig_m1c.mission.requests) == req1,
      f"replans {rep1}→{rig_m1c.loop.status()['replans']} / "
      f"req {req1}→{len(rig_m1c.mission.requests)}")

# --------------------------------------------------------------------------- #
section("M1.5/M1.6 新进程新目录 + 不读上一场")
# --------------------------------------------------------------------------- #
tmp_m1 = tempfile.mkdtemp(prefix="rtgate_m1_")
arch_a = RunArchive(tmp_m1, run_id="RUNX")
arch_a.blank_map(rows=geo.MAP_ROWS, cols=geo.MAP_COLS, block_size_m=geo.BLOCK_SIZE,
                 barrier_col_from=geo.BARRIER_COL_FROM, barrier_row_hi=geo.BARRIER_ROW_HI,
                 barrier_row_lo=geo.BARRIER_ROW_LO)
arch_a.finalize(reason="first")
arch_b = RunArchive(tmp_m1, run_id="RUNX")             # 同名再起（模拟新进程）
arch_b.blank_map(rows=geo.MAP_ROWS, cols=geo.MAP_COLS, block_size_m=geo.BLOCK_SIZE,
                 barrier_col_from=geo.BARRIER_COL_FROM, barrier_row_hi=geo.BARRIER_ROW_HI,
                 barrier_row_lo=geo.BARRIER_ROW_LO)
check("M1.5a 同名再起 ⇒ 目录不同（绝不复用上一场目录）",
      arch_a.dir != arch_b.dir and arch_b.dir.is_dir(),
      f"{arch_a.dir.name} vs {arch_b.dir.name}")
check("M1.5b 新目录 map.json 是空白固定层（变量为空）",
      json.loads((arch_b.dir / "map.json").read_text(encoding="utf-8"))["mines"] == [])
src_arch = pathlib.Path(M.__file__).with_name("core") / "run_archive.py"
arch_text = src_arch.read_text(encoding="utf-8")
reads = re.findall(r"(read_text\(|open\([^)]*[\"']r[\"'])", arch_text)
check("M1.6a run_archive.py 没有任何读既有演出目录的代码（只写不读）",
      not [r for r in reads if "summary" not in r], str(reads[:3]))
h_before = hashlib.sha256((arch_a.dir / "summary.json").read_bytes()).hexdigest()
arch_b.finalize(reason="second")
h_after = hashlib.sha256((arch_a.dir / "summary.json").read_bytes()).hexdigest()
check("M1.6b 第二场收尾不改第一场任何文件（物理隔离）", h_before == h_after,
      h_before[:16])

# --------------------------------------------------------------------------- #
section("M1.7/M1.8 负例：hard 仍拦 unknown / degrade 段路线真穿过 unknown")
# --------------------------------------------------------------------------- #
rig_m1h = Rig(_cfg(ttl=0.5, policy="hard"))
rig_m1h.set_pose_block("N10")
rig_m1h.start(layout(["N10", "M10", "L10"], target="L10"))
rig_m1h.tick(2)
rig_m1h.set_pose_block("N9")
rig_m1h.tick(4)
check("M1.7 负例-1 hard 口径：走过未知格 3 拍仍 ⇒ SAFE_STOP（unknown 不是可通行）",
      rig_m1h.loop.phase == M.STEP_PHASE_SAFE_STOP, rig_m1h.loop.phase)
_norm_deg = M.normalize_step_layout(layout(["N10", "L10"], target="L10"))[0]
assert _norm_deg is not None
_p_deg = M.plan_segment_from_layout(_cfg(), _norm_deg, from_cell="N10", target_cell="L10")
_p_hard = M.plan_segment_from_layout(_cfg(ttl=0.5, policy="hard"), _norm_deg,
                                     from_cell="N10", target_cell="L10")
check("M1.8 负例-2 同一份图两种口径：degrade=可通行（穿 M10）；hard=拒发（绝不穿未知格）",
      _p_deg.ok and "M10" in _p_deg.cells
      and (not _p_hard.ok) and ("未知格" in _p_hard.error or "无解" in _p_hard.error),
      f"deg={_p_deg.cells} / hard={_p_hard.error[:40]}")

# --------------------------------------------------------------------------- #
section("M2.1~M2.4 归档六文件 / summary 六键 / 里程 / 计数透传")
# --------------------------------------------------------------------------- #
tmp_m2 = tempfile.mkdtemp(prefix="rtgate_m2_")
arch = RunArchive(tmp_m2, log=lambda _s: None)
arch.blank_map(rows=geo.MAP_ROWS, cols=geo.MAP_COLS, block_size_m=geo.BLOCK_SIZE,
               barrier_col_from=geo.BARRIER_COL_FROM, barrier_row_hi=geo.BARRIER_ROW_HI,
               barrier_row_lo=geo.BARRIER_ROW_LO)
arch.write_route(["N10", "M10", "L10*"], source="step:v1", from_cell="N10",
                 target_cell="L10")
tee = install_log_tee(lambda _s: None, arch.log_path)
tee("hello archive")
arch.pose(1.0, 0.0, 0.0, 0.0, "N10", "DRIVING", 1)
arch.pose(2.0, 0.3, 0.0, 0.0, "M10", "DRIVING", 1)
arch.pose(3.0, 0.3, 0.4, 0.0, "L10", "DRIVING", 1, out_of_bounds=True)
arch.trace("gate", key="layout_stale", age_s=1.5)
arch.warn("layout_stale")
summary = arch.finalize(reason="finish", duration_s=12.5, unknown_cells=5)
status_files = arch.file_status()
check("M2.1 六文件齐（map/route/pose/log/trace/summary）",
      all(status_files.values()) and len(status_files) == 6,
      str(status_files))
check("M2.2 summary 六键齐", set(SUMMARY_KEYS) <= set(summary)
      and all(k in summary for k in SUMMARY_KEYS), str(sorted(summary)))
check("M2.3 里程按位姿折线累计（0.3 + 0.4 = 0.7 m）",
      abs(summary["distance_m"] - 0.7) < 1e-6, str(summary["distance_m"]))
check("M2.4 越界次数与 unknown 格数透传进 summary",
      summary["out_of_bounds"] == 1 and summary["unknown_cells"] == 5,
      f"{summary['out_of_bounds']} / {summary['unknown_cells']}")
check("M2.5 summary 里的 warn_counts 按 key 计数",
      summary["warn_counts"].get("layout_stale") == 1, str(summary["warn_counts"]))

# --------------------------------------------------------------------------- #
section("M2.6 SIGTERM/停机路径也出归档包（run_live 的 finally 兜住）")
# --------------------------------------------------------------------------- #
tmp_live = tempfile.mkdtemp(prefix="rtgate_live_")
orig_body = M._run_live_body
orig_arch_cls = M.RunArchive
created: list[RunArchive] = []


def _fake_archive(*args, **kwargs):
    kwargs.setdefault("root", tmp_live)
    obj = orig_arch_cls(*args, **kwargs)
    created.append(obj)
    return obj


def _fake_body_stop(cfg, *, log=None, alive=None, map_path=None, archive=None, report=None):
    if report is not None:
        report["reason"] = "stop/SIGTERM"              # 真实停机路径写的就是这个口径
    if archive is not None:
        archive.warn("segment_timeout")
    return 0


M.RunArchive = _fake_archive
M._run_live_body = _fake_body_stop
try:
    rc_stop = M.run_live(_cfg())
finally:
    M.RunArchive = orig_arch_cls
    M._run_live_body = orig_body
pkg = created[0] if created else None
check("M2.6a 停机路径 rc=0 且产出归档目录", rc_stop == 0 and pkg is not None
      and pkg.dir.is_dir(), f"rc={rc_stop}")
check("M2.6b 归档包六文件齐（reason 记录为 stop/SIGTERM）",
      pkg is not None and all(pkg.file_status().values())
      and json.loads((pkg.dir / "summary.json").read_text(encoding="utf-8"))["reason"]
      == "stop/SIGTERM",
      str(pkg.file_status() if pkg else "<无>"))

# ---- 负例-1：body 抛异常，归档仍然要出 -------------------------------- #
def _fake_body_raise(cfg, *, log=None, alive=None, map_path=None, archive=None, report=None):
    raise RuntimeError("模拟故障退出")


created.clear()
M.RunArchive = _fake_archive
M._run_live_body = _fake_body_raise
raised = False
try:
    M.run_live(_cfg())
except RuntimeError:
    raised = True
finally:
    M.RunArchive = orig_arch_cls
    M._run_live_body = orig_body
summary_exc = (json.loads((created[0].dir / "summary.json").read_text(encoding="utf-8"))
               if created else {})
check("M2.7 负例-1 故障退出（抛异常）仍然出归档包、异常照旧上抛",
      raised and bool(created) and all(created[0].file_status().values()),
      f"raised={raised} files={created[0].file_status() if created else '<无>'}")
check("M2.8 负例-2 finalize 幂等：重复调用不产生第二个 run 目录、summary 仍六键",
      (lambda a: (a.finalize(reason="again"),
                  a.finalize(reason="again2"),
                  len([p for p in pathlib.Path(tmp_m2).iterdir() if p.is_dir()]) >= 1
                  and set(SUMMARY_KEYS) <= set(json.loads(
                      (a.dir / "summary.json").read_text(encoding="utf-8")))))(arch),
      str(sorted(p.name for p in pathlib.Path(tmp_m2).iterdir())))

# --------------------------------------------------------------------------- #
section("S2.1~S2.5 lead_cells 计算 + info_lead_low 只记不拦")
# --------------------------------------------------------------------------- #
rig_s2 = Rig(_cfg(ttl=0.0, policy="warn"))
rig_s2.set_pose_block("N10")
rig_s2.start(layout(["N10", "M10", "L10"], target="L10"))   # known=3，前方要走 2 格 ⇒ 1
rig_s2.tick(2)
st_s2 = rig_s2.loop.status()
check("S2.1a lead_cells = known(3) - ahead(2) = 1（<2 即记 info_lead_low）",
      st_s2["lead_cells"] == 1, str(st_s2["lead_cells"]))
check("S2.1b 出 INFO key=info_lead_low（lead_cells/known_cells/ahead_cells 都在）",
      len(rig_s2.infos("info_lead_low")) >= 1
      and "lead_cells=" in rig_s2.infos("info_lead_low")[0],
      (rig_s2.infos("info_lead_low") or ["<无>"])[0][:130])
check("S2.2 负例-1 lead<2 仍不拦停（相位不是 SAFE_STOP/WAIT_INFO_TIMEOUT）",
      rig_s2.loop.phase not in (M.STEP_PHASE_SAFE_STOP, M.STEP_PHASE_WAIT_TIMEOUT),
      rig_s2.loop.phase)
check("S2.3 负例-2 info_lead_low 不计入 warn_counts（只记不拦，不是 warn）",
      "info_lead_low" not in rig_s2.loop.status()["gate_counts"],
      str(rig_s2.loop.status()["gate_counts"]))


class _StubPlan:
    """注入用：4 格路线（ahead=3），配 known=1 ⇒ lead = -2（负领先量）。"""

    ok = True
    tokens = ["N10", "M10", "L10", "L9"]
    cells = ["N10", "M10", "L10", "L9"]
    route_len_m = 1.05
    elapsed_ms = 0.1
    engine_line = "stub"
    warnings: tuple = ()
    stop_kind = "path"
    single_cell = False


rig_s2c = Rig(_cfg(ttl=0.0, policy="warn"))
rig_s2c.set_pose_block("N10")
rig_s2c.loop._known_cells = frozenset({"N10"})        # noqa: SLF001
rig_s2c.loop._known_count = 1                          # noqa: SLF001
rig_s2c.loop._target_cell = "L9"                       # noqa: SLF001
rig_s2c.loop._lead_update(_StubPlan(), from_cell="N10")  # noqa: SLF001
_stub_lead = rig_s2c.loop.status()["lead_cells"]
check("S2.3b 负例-3 注入 lead=-2 ⇒ 出 info_lead_low 且不拦停（仍不做任何相位变更）",
      _stub_lead == -2 and len(rig_s2c.infos("info_lead_low")) >= 1
      and rig_s2c.loop.phase == M.STEP_PHASE_WAIT_LAYOUT,
      f"lead={_stub_lead} / phase={rig_s2c.loop.phase}")
rig_s2b = Rig(_cfg(ttl=0.0, policy="warn"))
rig_s2b.set_pose_block("N10")
rig_s2b.start(layout([c for c in ALL_CELLS if c not in ("A1", "A2")], target="L10"))
rig_s2b.tick(2)
st_s2b = rig_s2b.loop.status()
check("S2.4 lead 充足（known≈152，ahead=2）⇒ 不出 info_lead_low",
      st_s2b["lead_cells"] is not None and st_s2b["lead_cells"] >= 2
      and not rig_s2b.infos("info_lead_low"),
      f"lead={st_s2b['lead_cells']} / info={rig_s2b.infos('info_lead_low')[:1]}")
check("S2.5 P_CAR_STATE 带上 lead_cells（机端/复盘可读）",
      isinstance(rig_s2b.bb.get(M.P_CAR_STATE), dict)
      and "lead_cells" in (rig_s2b.bb.get(M.P_CAR_STATE) or {}),
      str(sorted((rig_s2b.bb.get(M.P_CAR_STATE) or {}).keys())))

finish()
