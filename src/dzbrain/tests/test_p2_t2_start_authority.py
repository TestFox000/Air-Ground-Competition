# -*- coding: utf-8 -*-
"""P2-T2 起点口径收敛（MISSION-GATE-20261029）：layout.start 是唯一权威。

现场证据（真车 2026-10-28）：布局 start=L7、步进器 belief 复位 N10、odom 实停 L7
=> 段1 四个航点横向偏差**清一色 0.350 m**（整格偏移）。三值各说各话，投影/到点全失真。

本文件钉住两件事（都是卡面验收项）：
  1. `DroneTask` 收到带 `start` 的布局（v1 与 v2）时：打印三值对照
     （layout.start / belief 当前格 / odom 当前格），两两偏差 > 半格(0.175 m) ⇒ WARNING，
     并把 belief 写到 layout.start 格心（只写一次，不搬正在跑的车）；不带 start 的报文
     照旧整份拒收且不崩。
  2. `MissionTask.reset_pose`（StepLoop 启动复位那条路径）：有布局时以布局起点为准，
     没有布局（仿真/bench/离线）时逐位保持旧语义 —— 这就是"以布局为唯一权威写 belief"
     在**主链**上的落地（否则 main 的 `_seed_pose` 会用配置里的旧默认盖掉布局真值）。

跑法：
    cd dzrobot_v2 && python -B -X utf8 src/dzbrain/tests/test_p2_t2_start_authority.py
"""
from __future__ import annotations

import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:                      # pragma: no cover
    pass

from harness import check, check_close, finish, section                  # noqa: E402

from dzbrain.core import geometry as geo                                 # noqa: E402
from dzbrain.core.blackboard import (P_FIELD_LAYOUT, P_POSE, P_POSE_RAW,  # noqa: E402
                                     P_SAFETY, Blackboard)
from dzbrain.core.config import Config                                  # noqa: E402
from dzbrain.core.events import EventBus                                # noqa: E402
from dzbrain.core.state import SafetyState                              # noqa: E402
from dzbrain.tasks.drone import START_AUTHORITY_TOL_M, DroneTask         # noqa: E402
from dzbrain.tasks.mission import MissionTask                           # noqa: E402

from fakes import FakeBridge, FakeClock                                 # noqa: E402


def layout_payload(*, start: str, version: int = 2, known=None, ts: float = 100.0) -> dict:
    """造一份合法 field_layout（v2 带 known；version=1 且不给 known 即 v1 形态）。"""
    payload = {
        "type": "field_layout", "version": version, "ts": ts,
        "start": start, "end": "N5",
        "mines": ["H1"], "obstacles": ["D1"], "targets": ["F11"],
        "bridges": ["K7", "K5"],
    }
    if known is not None:
        payload["known"] = list(known)
    return payload


class DroneRig:
    """一台"车 + 无人机链路"，位姿可分别给 belief(P_POSE) 与 odom(P_POSE_RAW)。"""

    def __init__(self, *, belief_at: str = "N10", odom_at: str | None = None) -> None:
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.logs: list[str] = []
        cfg = Config.from_dict({"drone": {"enabled": True, "heartbeat_timeout_s": 3.0,
                                          "info_stale_s": 30.0},
                                "mission": {"auto_start": False},
                                "logging": {"rate_limit_s": 0.0}})
        self.bb.set(P_SAFETY, SafetyState(ok=True), "test")
        self.bb.set(P_POSE, geo.Pose(*geo.block_to_meters(belief_at), geo.HEADING_Y), "test")
        if odom_at is not None:
            self.bb.set(P_POSE_RAW, geo.Pose(*geo.block_to_meters(odom_at), geo.HEADING_Y),
                        "test")
        self.task = DroneTask(self.bridge, self.bb, self.bus, cfg, log=self.logs.append,
                              clock=self.clock.now, local_map=None)
        self.task.setup()

    def feed(self, payload: dict) -> None:
        self.bridge.push_drone(json.dumps(payload, ensure_ascii=False))
        self.clock.advance(0.2)
        self.task.tick()

    def belief_cell(self) -> str:
        pose = self.bb.get(P_POSE)
        return geo.meters_to_block(pose.x, pose.y) if pose is not None else ""

    def belief_xy(self) -> tuple[float, float] | None:
        pose = self.bb.get(P_POSE)
        return None if pose is None else (float(pose.x), float(pose.y))

    def lines_with(self, text: str) -> list[str]:
        return [line for line in self.logs if text in line]


# --------------------------------------------------------------------------- #
#  ① 三值不一致（belief/odom 都在 N10，布局说 M10）⇒ WARNING + belief 跟布局
# --------------------------------------------------------------------------- #
def case_mismatch_warns_and_seeds() -> None:
    section("① 三值不一致：belief/odom=N10、layout.start=M10 → WARNING 且 belief ← M10")
    rig = DroneRig(belief_at="N10", odom_at="N10")
    check("①.0 半格门 = 0.175 m（卡面口径）", abs(START_AUTHORITY_TOL_M - 0.175) < 1e-12,
          f"{START_AUTHORITY_TOL_M}")
    rig.feed(layout_payload(start="M10"))
    cells = [line for line in rig.logs if "起点口径：" in line]
    check("①.1 打印了三值对照（layout.start / belief / odom 同一行）",
          bool(cells) and "layout.start=M10" in cells[-1] and "belief=N10" in cells[-1]
          and "odom=N10" in cells[-1], f"{cells[-1] if cells else None}")
    check("①.2 两两偏差都算出来了（layout↔belief / layout↔odom / belief↔odom）",
          bool(cells) and cells[-1].count("cm") >= 3, f"{cells[-1] if cells else None}")
    warn = rig.lines_with("起点口径不一致")
    check("①.3 偏差 0.350 m > 半格 ⇒ WARNING 提示人工确认",
          len(warn) == 1 and "35.0 cm" in warn[0] and "布局为真值" in warn[0],
          f"{warn}")
    check("①.4 belief 已按 layout.start 写入（== M10 格心，米级对齐）",
          rig.belief_cell() == "M10"
          and rig.belief_xy() == geo.block_to_meters("M10"),
          f"belief={rig.belief_cell()} {rig.belief_xy()}")
    check("①.5 只写一次（start_authority_seeded=True）",
          rig.task.start_authority_seeded is True)
    check("①.6 不是阻断：布局照常接受（updates=1 / rejects=0）",
          rig.task.layout_updates == 1 and rig.task.layout_rejects == 0,
          f"{rig.task.layout_updates}/{rig.task.layout_rejects}")
    check("①.7 统计快照可对账（start_authority_last）",
          rig.task.start_authority_last.get("start") == "M10"
          and rig.task.start_authority_last.get("belief_cell") == "N10"
          and abs(rig.task.start_authority_last.get("worst_m", 0.0) - 0.35) < 1e-6,
          f"{rig.task.start_authority_last}")


# --------------------------------------------------------------------------- #
#  ② 三值一致 ⇒ 不 WARNING、不搬车
# --------------------------------------------------------------------------- #
def case_consistent_no_warning() -> None:
    section("② 三值一致（都在 N10）→ 不 WARNING、belief 原地不动")
    rig = DroneRig(belief_at="N10", odom_at="N10")
    rig.feed(layout_payload(start="N10"))
    check("②.1 没有 WARNING", rig.lines_with("起点口径不一致") == [],
          f"{rig.lines_with('起点口径不一致')}")
    check("②.2 start_authority_warnings == 0", rig.task.start_authority_warnings == 0)
    check("②.3 belief 仍在 N10 格心（0 位移）",
          rig.belief_cell() == "N10" and rig.belief_xy() == geo.block_to_meters("N10"),
          f"belief={rig.belief_cell()} {rig.belief_xy()}")
    check("②.4 仍然一次性写了权威（幂等：同格无害）",
          rig.task.start_authority_seeded is True)


# --------------------------------------------------------------------------- #
#  ③ v1 与 v2 两种输入都不崩（卡面原文）
# --------------------------------------------------------------------------- #
def case_v1_and_v2_no_crash() -> None:
    section("③ v1（无 known）与 v2（带 known）两种输入都不崩，都按布局起点落 belief")
    r1 = DroneRig(belief_at="N10")
    r1.feed(layout_payload(start="L10", version=1))          # v1：无 known
    check("③.1 v1 接受成功（rejects=0）且 belief ← L10",
          r1.task.layout_rejects == 0 and r1.belief_cell() == "L10",
          f"rejects={r1.task.layout_rejects} belief={r1.belief_cell()}")
    check("③.2 v1 归一化后没有 known 键（旧路径语义不变，不被本卡改动）",
          "known" not in (r1.bb.get(P_FIELD_LAYOUT) or {}),
          f"{r1.bb.get(P_FIELD_LAYOUT)}")

    r2 = DroneRig(belief_at="N10")
    known = ["N10", "M10", "L10", "N9", "M9", "L9", "K7", "K5", "N5", "H1", "D1", "F11"]
    r2.feed(layout_payload(start="M10", version=2, known=known, ts=200.0))
    check("③.3 v2 接受成功且 belief ← M10",
          r2.task.layout_rejects == 0 and r2.belief_cell() == "M10",
          f"rejects={r2.task.layout_rejects} belief={r2.belief_cell()}")
    check("③.4 v2 的 known 照常落盘（本卡没动接收语义）",
          (r2.bb.get(P_FIELD_LAYOUT) or {}).get("known") == known,
          f"{len((r2.bb.get(P_FIELD_LAYOUT) or {}).get('known') or [])} 格")


# --------------------------------------------------------------------------- #
#  ④ 不带 start：整份拒收，不崩、不写 belief
# --------------------------------------------------------------------------- #
def case_missing_start_rejected() -> None:
    section("④ 报文不带 start → 整份拒收（不崩、不碰 belief）")
    rig = DroneRig(belief_at="N10", odom_at="N10")
    bad = layout_payload(start="M10")
    bad.pop("start")
    rig.feed(bad)
    check("④.1 拒收计数 +1", rig.task.layout_rejects == 1, f"{rig.task.layout_rejects}")
    check("④.2 没写权威（start_authority_seeded 仍 False）",
          rig.task.start_authority_seeded is False)
    check("④.3 belief 一动不动（仍在 N10）", rig.belief_cell() == "N10",
          f"{rig.belief_cell()}")
    check("④.4 没有起点口径日志（拒收的帧不该产生口径结论）",
          rig.lines_with("起点口径：") == [], f"{rig.lines_with('起点口径：')}")


# --------------------------------------------------------------------------- #
#  ⑤ 后续布局版本不搬车（车可能正在走）
# --------------------------------------------------------------------------- #
def case_no_reseed_on_newer_version() -> None:
    section("⑤ 后到的布局改 start ⇒ 只报对照/WARNING，**不搬**正在跑的车")
    rig = DroneRig(belief_at="N10", odom_at="N10")
    # known 必须覆盖 mines/obstacles/targets（v2 要求"特征 ⊆ known"，否则整份拒收）
    base_known = ["N10", "M10", "H1", "D1", "F11", "K7", "K5", "N5"]
    rig.feed(layout_payload(start="M10", version=2, known=base_known, ts=100.0))
    check("⑤.0 首份布局已把 belief 放到 M10", rig.belief_cell() == "M10",
          f"{rig.belief_cell()}")
    rig.feed(layout_payload(start="L10", version=3,
                            known=base_known + ["L10"], ts=300.0))
    check("⑤.1 新版本照常接受", rig.task.layout_updates == 2, f"{rig.task.layout_updates}")
    check("⑤.2 belief 没有被第二次搬走（仍在 M10）", rig.belief_cell() == "M10",
          f"{rig.belief_cell()}")
    check("⑤.3 但对照照打、WARNING 累计（现场能看见 odom 与布局分叉）",
          len(rig.lines_with("起点口径：")) == 2
          and rig.task.start_authority_warnings >= 2,
          f"lines={len(rig.lines_with('起点口径：'))} warn={rig.task.start_authority_warnings}")


# --------------------------------------------------------------------------- #
#  ⑥ Mission 侧：reset_pose 以布局 start 为权威（主链落地）
# --------------------------------------------------------------------------- #
def mission_rig(*, layout_start: str | None, reset_spec: str = "N10,y"):
    clock = FakeClock()
    bridge = FakeBridge(clock)
    bb = Blackboard()
    bus = EventBus()
    logs: list[str] = []
    cfg = Config.from_dict({"mission": {"auto_start": False, "reset": reset_spec},
                            "pursuit": {"arrival_m": 0.12},
                            "logging": {"rate_limit_s": 0.0}})
    bb.set(P_SAFETY, SafetyState(ok=True), "test")
    bb.set(P_POSE, geo.Pose(0.0, 0.0, 0.0), "test")
    if layout_start is not None:
        bb.set(P_FIELD_LAYOUT, {"start": layout_start, "end": "N5"}, "test")
    task = MissionTask(bridge, bb, bus, cfg, log=logs.append, clock=clock.now)
    task.setup()
    return task, bb, logs


def case_mission_reset_prefers_layout() -> None:
    section("⑥ Mission.reset_pose：有布局 ⇒ 用布局起点；无布局 ⇒ 逐位保持旧语义")
    # 6a 有布局：配置/步进器说 N10，布局说 M10 ⇒ 以布局为准
    task, bb, logs = mission_rig(layout_start="M10")
    task.reset_pose("N10,y")
    pose = bb.get(P_POSE)
    check("⑥.1 belief == 布局起点 M10 格心（不是复位串里的 N10）",
          (float(pose.x), float(pose.y)) == geo.block_to_meters("M10"),
          f"pose=({pose.x:.3f},{pose.y:.3f})")
    check("⑥.2 朝向沿用复位串的 ,y（车头姿态与格位无关）",
          abs(pose.yaw - geo.HEADING_Y) < 1e-12, f"yaw={geo.deg(pose.yaw):.1f}°")
    check("⑥.3 打了 WARNING（把配置与布局的分歧摆到现场）",
          any("以**布局为真值**复位 belief" in line for line in logs), f"{logs[-3:]}")

    # 6b 无布局：旧语义（N10），一个字符都不改
    task2, bb2, logs2 = mission_rig(layout_start=None)
    task2.reset_pose("N10,y")
    pose2 = bb2.get(P_POSE)
    check("⑥.4 无布局 ⇒ 仍是 N10（旧语义逐位不变：仿真/bench/离线不受影响）",
          (float(pose2.x), float(pose2.y)) == geo.block_to_meters("N10"),
          f"pose=({pose2.x:.3f},{pose2.y:.3f})")
    check("⑥.5 无布局 ⇒ 没有 WARNING",
          not any("以**布局为真值**复位 belief" in line for line in logs2), f"{logs2[-3:]}")

    # 6c 复位串本来就等于布局起点：不 WARNING
    task3, bb3, logs3 = mission_rig(layout_start="N10")
    task3.reset_pose("N10,y")
    check("⑥.6 复位串与布局一致 ⇒ 不 WARNING（只有真分歧才报）",
          not any("以**布局为真值**复位 belief" in line for line in logs3), f"{logs3[-3:]}")


for name, fn in (("① 不一致", case_mismatch_warns_and_seeds),
                 ("② 一致", case_consistent_no_warning),
                 ("③ v1/v2", case_v1_and_v2_no_crash),
                 ("④ 缺 start", case_missing_start_rejected),
                 ("⑤ 不搬车", case_no_reseed_on_newer_version),
                 ("⑥ Mission 侧", case_mission_reset_prefers_layout)):
    try:
        fn()
    except Exception as exc:                     # noqa: BLE001
        import traceback
        traceback.print_exc()
        check(f"{name} 用例自身未抛异常", False, f"{type(exc).__name__}: {exc}")

finish()
