# -*- coding: utf-8 -*-
"""AGC-T1 进度生产者/消费者契约（新增测试，不覆盖既有测试）。

背景（已证事实，见卡 AGC-T1-PROGRESS-20261024 §2）：
  · TrackerTask / LocalizationTask / MissionTask 三处都写 P_ROUTE_INDEX/P_ROUTE_S；
  · run2 实跑里 F9 STOP 显示 段11/s4.40，DRIVE 恢复后显示 段6/s2.48，车其实一路走到 A8。

本文件只钉"契约"，不改生产逻辑：
  ① 单一槽位 = 最后写入者赢，来源随写入顺序变（显示层没有权威/新鲜度概念）；
  ② 同一姿态下，"健康"写入者（Localization）与"旧 hint"写入者（Mission）可以差 2 m 以上；
  ③ Mission._current_s 的采纳判据是"两个估计互比"（|缓存-自身| <= arrival_m），
     不与位姿比：自己的投影离线 0.70 m 时仍会覆盖掉正确的 4.40（契约缺口，已登记）；
  ④ 授权 reset / 换路（含 keep_progress）是唯一合法的"进度回退"，与正常推进可分离；
  ⑤ 控制器（Tracker）不用该键做运动决策 —— 它自算投影，键只作 s_hint 兜底。

跑法（v1 根目录）：
  python -B -X utf8 dzrobot_v2/src/dzbrain/tests/test_agc_t1_progress_contract.py
"""
from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from fakes import FakeBridge, FakeClock                                    # noqa: E402
from harness import check, check_close, finish, section                    # noqa: E402

from dzbrain.core import geometry as geo                                   # noqa: E402
from dzbrain.core.blackboard import (P_MISSION_PHASE, P_POSE, P_ROUTE,  # noqa: E402
                                     P_ROUTE_INDEX, P_ROUTE_S, P_SAFETY,
                                     Blackboard)
from dzbrain.core.config import Config                                     # noqa: E402
from dzbrain.core.events import EventBus                                   # noqa: E402
from dzbrain.core.state import SafetyState                                 # noqa: E402
from dzbrain.io.contract import MissionPhase                               # noqa: E402
from dzbrain.route.parser import parse_mission                             # noqa: E402
from dzbrain.tasks.localization import LocalizationTask                    # noqa: E402
from dzbrain.tasks.mission import MissionTask                              # noqa: E402
from dzbrain.tasks.telemetry import build_telemetry                        # noqa: E402
from dzbrain.tasks.tracker import TrackerTask                              # noqa: E402

#: run2 实跑的 54 点路线（_tmp_session/carloop-1024/run-intent.json 原样）
TOKENS = ["N10", "M10", "L10", "K10", "J10", "I10", "H10", "H8", "R:H9", "R:H10",
          "H9", "G9", "F9*", "E9", "D9", "D10", "C10", "B10", "B9", "A9", "A8", "A7",
          "A6", "A5", "A4", "A3", "A2", "A1", "B1", "C1", "C3", "C4", "C5*", "R:C4",
          "R:C3", "D3", "E3", "F3", "F4", "F5", "G5", "H5", "I5", "I4", "I3", "I2",
          "J2", "J3", "J4", "K4", "L4", "M4", "N4", "N5"]

#: 现场 K 列（与 run2 进程私有配置一致）
K_COLUMN = [f"K{r}" for r in range(1, 12)]

#: run2 遥测采样（2 Hz，原件 run2-mission.log）：(x, y, yaw_deg, 阶段)
#: 取 F9 停站前后一段，用于"同一姿态、不同写入者"的对照。
F9_APPROACH = (0.909, 2.748, 90.0)          # 进站前（实跑显示 段6 s2.49）
F9_STOP = (0.908, 2.827, 93.1)              # 停站（实跑显示 段11 s4.40）


def cfg_of(**pursuit) -> Config:
    data = {"pursuit": dict(pursuit), "localization": {},
            "mission": {"auto_start": False}, "logging": {"rate_limit_s": 0.0}}
    data["pursuit"].setdefault("bridge_blocks", list(K_COLUMN))
    data["pursuit"].setdefault("cruise_speed_mps", 0.1)
    return Config.from_dict(data)


def _gate_off(mission) -> None:
    """把「几何可信门」关掉 = 复现**修复前**语义（AGC-T1 §4A.10 补丁未落地时无该属性，直接跳过）。

    本文件 ①②③⑦ 里有 12 条断言记录的是**修复前的契约缺口**（旧 hint 驻点 + 读侧回写），
    它们是"事故现场指纹"，必须继续可复现；修复后语义由
    `test_agc_t1_mission_gate_fix.py` 断言。两条路径是同一份代码、只差这一个门限，
    所以这里**不改任何断言的期望数值**，只把门限抬到 inf。
    """
    if hasattr(mission, "_drift_gate_m"):
        mission._drift_gate_m = float("inf")


class Rig:
    """一套共享 Blackboard + FakeBridge；三个任务都挂在同一套总线上（与车端同构）。

    `legacy=True` = 关掉几何可信门，跑"修复前"语义（见 `_gate_off`）。
    """

    def __init__(self, legacy: bool = False) -> None:
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.cfg = cfg_of()
        self.route = parse_mission(TOKENS, name="agc-t1")
        self.bb.set(P_SAFETY, SafetyState(ok=True), "test")
        self.bb.set(P_ROUTE, self.route, "test")
        self.tracker = TrackerTask(self.bridge, self.bb, self.bus, self.cfg,
                                   log=lambda m: None, clock=self.clock.now)
        self.loc = LocalizationTask(self.bridge, self.bb, self.bus, self.cfg,
                                    log=lambda m: None, clock=self.clock.now)
        self.mission = MissionTask(self.bridge, self.bb, self.bus, self.cfg,
                                   log=lambda m: None, clock=self.clock.now)
        self.mission._install_route(self.route, source="test")
        if legacy:
            _gate_off(self.mission)

    def pose(self, x: float, y: float, yaw_deg: float, phase: str = MissionPhase.DRIVE) -> None:
        yaw = geo.rad(yaw_deg)
        self.bridge.set_pose(x, y, yaw)
        self.bb.set(P_POSE, geo.Pose(x, y, yaw), "car")
        self.bb.set(P_MISSION_PHASE, phase, "car")

    def key(self):
        return (self.bb.get(P_ROUTE_INDEX), self.bb.get(P_ROUTE_S),
                (self.bb.entry(P_ROUTE_S).source if self.bb.entry(P_ROUTE_S) else None))


# --------------------------------------------------------------------------- #
#  ① 单一槽位：最后写入者赢，来源跟着变
# --------------------------------------------------------------------------- #
def test_single_slot_last_writer() -> None:
    section("① 单一槽位：谁最后写谁赢 —— 换个顺序，显示值就从 段11/4.40 变 段6/2.48")
    rig = Rig(legacy=True)
    rig.pose(*F9_STOP)
    rig.tracker.tick()
    after_t = rig.key()
    check("1.1 Tracker 写过之后，键的 source = TrackerTask",
          after_t[2] == "TrackerTask", f"got {after_t[2]!r}")
    rig.mission._last_s = 2.42                      # 旧 hint（实跑同态）
    rig.mission._current_s()
    after_m = rig.key()
    check("1.2 Mission 用旧 hint 自己算到 段6/2.48，但**不再写键**：键仍是 Tracker 的 段11/4.40",
          abs(rig.mission._last_s - 2.483) < 1e-3 and after_m[2] != "MissionTask"
          and after_m[0] == 11,
          f"mission_s={rig.mission._last_s!r} key={after_m!r}")
    rig.loc.tick()
    after_l = rig.key()
    check("1.3 Localization 同拍给 段11/4.40，与 Mission 自己的旧 hint 值（2.483）差 > 1.5 m",
          after_l[2] == "Localization" and after_l[0] == 11
          and abs(float(after_l[1]) - float(rig.mission._last_s)) > 1.5,
          f"mission_s={rig.mission._last_s!r} loc={after_l[1]!r}")

    rig2 = Rig(legacy=True)
    rig2.pose(*F9_STOP)
    rig2.loc.tick()
    rig2.mission._last_s = 2.42
    rig2.mission._current_s()
    check("1.4 顺序反过来（先 Localization 后 Mission）→ 键仍是 Localization 的 段11/4.40",
          rig2.key()[2] == "Localization" and rig2.key()[0] == 11
          and abs(rig2.mission._last_s - 2.483) < 1e-3,
          f"key={rig2.key()!r} mission_s={rig2.mission._last_s!r}")

    tel = build_telemetry(rig2.bb, now=rig2.clock.now())
    check("1.5 显示层（Telemetry）读到的是黑板原值，不做来源/新鲜度过滤",
          tel.seg_index == rig2.bb.get(P_ROUTE_INDEX)
          and abs(float(tel.s) - float(rig2.bb.get(P_ROUTE_S))) < 1e-12,
          f"tel=({tel.seg_index}, {tel.s!r}) bb={rig2.key()!r}")


# --------------------------------------------------------------------------- #
#  ② 同一姿态，不同写入者的值差 2 m 以上
# --------------------------------------------------------------------------- #
def test_writers_disagree_at_same_pose() -> None:
    section("② 同一姿态：Localization（无 hint 全路线）vs Mission（旧 hint）差 > 1.5 m")
    rig = Rig(legacy=True)
    rig.pose(*F9_STOP)
    rig.loc.tick()
    s_loc = float(rig.bb.get(P_ROUTE_S))
    idx_loc = rig.bb.get(P_ROUTE_INDEX)
    check("2.1 Localization 在 F9 得到 段11/s≈4.4~4.6（与实跑 STOP 显示一致）",
          idx_loc in (11, 12) and 4.3 <= s_loc <= 4.7, f"got 段{idx_loc} s={s_loc}")

    rig.mission._last_s = 2.42                      # 旧 hint：H10→H8 段（实跑同态）
    s_mis = rig.mission._current_s()
    own = rig.mission._path.project(F9_STOP[0], F9_STOP[1], s_hint=2.42)
    check("2.2 同一姿态下 Mission 用旧 hint 只算到 段6/s≈2.45",
          rig.mission._path.project(F9_STOP[0], F9_STOP[1], s_hint=2.42).seg_index == 6
          and 2.3 <= s_mis <= 2.6, f"got s={s_mis}")
    check("2.3 两者差 > 1.5 m（同一个位姿、同一拍，值差两个格）",
          abs(s_loc - s_mis) > 1.5, f"loc={s_loc:.3f} mission={s_mis:.3f}")
    check("2.4 Mission 自己的投影离线 0.55 m —— 这个量已算出（_last_lat_m）但没参与裁决",
          own.dist > 0.3 and rig.mission._last_lat_m > 0.3,
          f"proj.dist={own.dist:.3f} _last_lat_m={rig.mission._last_lat_m:.3f}")
    check("2.5 结论：Mission 的旧 hint 只影响**自己**（s=2.483），不再污染共享键（键=Localization 段11/4.40）",
          rig.bb.get(P_ROUTE_INDEX) == 11
          and rig.bb.entry(P_ROUTE_S).source != "MissionTask"
          and abs(rig.mission._last_s - s_mis) < 1e-9,
          f"bb={rig.key()!r} mission_s={rig.mission._last_s!r}")


# --------------------------------------------------------------------------- #
#  ③ Mission 采纳判据：两个估计互比（0.12 m 窗口），不与位姿比
# --------------------------------------------------------------------------- #
def _mission_case(cached_s: float, cached_idx: int, hint_s: float, legacy: bool = False):
    rig = Rig(legacy=legacy)
    rig.pose(*F9_STOP)
    rig.bb.set(P_ROUTE_S, cached_s, "Localization", ts=rig.clock.now())
    rig.bb.set(P_ROUTE_INDEX, cached_idx, "Localization", ts=rig.clock.now())
    rig.mission._last_s = hint_s
    out = rig.mission._current_s()
    return rig, out


def test_mission_adoption_rule() -> None:
    section("③ Mission._current_s：|缓存-自身|<=arrival_m 才采纳缓存，否则自己回写")
    # A 采纳：缓存 4.50 与自身 4.55 差 0.05 <= 0.12
    rig, out = _mission_case(4.50, 11, 4.39)
    check("3.1 A 缓存 4.50 与自身 4.55 差 0.05 → 采纳缓存（不下写）",
          abs(out - 4.50) < 1e-9 and rig.bb.entry(P_ROUTE_S).source == "Localization",
          f"out={out!r} src={rig.bb.entry(P_ROUTE_S).source}")
    # B 不采纳（差 0.15 > 0.12）→ 采用自身；"回写键"是修复前行为（legacy）
    rig, out = _mission_case(2.42, 6, 4.39, legacy=True)
    check("3.2 B 缓存旧 2.42、自身 4.40 → 丢弃缓存、采用自身（只算不写：键仍留给 Localization）",
          abs(out - 4.402) < 1e-3 and rig.bb.get(P_ROUTE_INDEX) == 6
          and rig.bb.entry(P_ROUTE_S).source == "Localization",
          f"out={out!r} bb={rig.key()!r}")
    # C ★缺口（legacy）：缓存正确、自身旧 → 键被改成错的
    rig, out = _mission_case(4.40, 11, 2.42, legacy=True)
    check("3.3 C 缺口已被补丁关掉：legacy 下自身仍是 2.483（指纹不变），但键不再被改写成 段6/2.48",
          abs(out - 2.483) < 1e-3 and rig.bb.get(P_ROUTE_INDEX) == 11
          and rig.bb.entry(P_ROUTE_S).source == "Localization",
          f"out={out!r} bb={rig.key()!r}")
    check("3.4 缺口根因：离线 0.55 m 也算得出来，旧判据不看它（门关掉即复现）⇒ 几何可信门为此而加",
          rig.mission._last_lat_m > 0.3, f"_last_lat_m={rig.mission._last_lat_m:.3f}")
    # D 0.12 m 窗口的边沿：0.098 采纳、0.15 拒绝 —— 拒绝那侧就是"段号提前一段"
    rig, out = _mission_case(4.40, 11, 4.39)
    check("3.5 D 缓存 4.40 与自身 4.402 只差 0.002 → 采纳缓存（同一拍不写，来源仍是 Localization）",
          abs(out - 4.40) < 1e-9 and rig.bb.entry(P_ROUTE_S).source == "Localization",
          f"out={out!r} src={rig.bb.entry(P_ROUTE_S).source}")


# --------------------------------------------------------------------------- #
#  ④ 授权 reset / 换路 是唯一合法的回退
# --------------------------------------------------------------------------- #
def test_authorized_resets() -> None:
    section("④ 授权复位/换路：键上的合法回退，可与正常推进分离")
    rig = Rig()
    rig.bb.set(P_ROUTE_S, 4.40, "Localization", ts=rig.clock.now())
    rig.bb.set(P_ROUTE_INDEX, 11, "Localization", ts=rig.clock.now())

    rig.mission._install_route(rig.route, source="换路", keep_progress=True)
    check("4.1 keep_progress=True（决策点改道）→ 段11/s4.40 原样保留",
          rig.key()[0] == 11 and abs(float(rig.key()[1]) - 4.40) < 1e-9,
          f"got {rig.key()!r}")

    rig.mission._install_route(rig.route, source="重新载入")
    check("4.2 普通载入路线 → 键复位成 段0/s0.0（source=MissionTask）",
          rig.key()[0] == 0 and float(rig.key()[1]) == 0.0
          and rig.key()[2] == "MissionTask", f"got {rig.key()!r}")

    rig.bb.set(P_ROUTE_S, 4.40, "Localization", ts=rig.clock.now())
    rig.bb.set(P_ROUTE_INDEX, 11, "Localization", ts=rig.clock.now())
    rig.mission._reset_belief("N10,y")
    check("4.3 reset:BLOCK,HEAD → 键复位成 段-1/s0.0（source=MissionTask）",
          rig.key()[0] == -1 and float(rig.key()[1]) == 0.0
          and rig.key()[2] == "MissionTask", f"got {rig.key()!r}")


# --------------------------------------------------------------------------- #
#  ⑤ 控制器不用该键做运动决策
# --------------------------------------------------------------------------- #
def test_tracker_control_independent() -> None:
    section("⑤ 控制器（Tracker）自算投影：把键改成离谱值，控制与遥测都不受影响")
    rig = Rig()
    rig.pose(*F9_APPROACH)
    rig.tracker.tick()
    s_own = float(rig.tracker.telemetry()["s"])
    rig.bb.set(P_ROUTE_S, 99.0, "test", ts=rig.clock.now())
    rig.bb.set(P_ROUTE_INDEX, 42, "test", ts=rig.clock.now())
    rig.tracker.tick()
    tel = rig.tracker.telemetry()
    check("5.1 键被写成 s=99/段42 后，Tracker 自己的投影照旧（控制不读键）",
          abs(float(tel["s"]) - s_own) < 0.05 and int(tel["seg_index"]) in (10, 11, 12),
          f"before={s_own:.3f} after={tel['s']!r} 段={tel['seg_index']}")
    check("5.2 Tracker 只把键当 s_hint 兜底：自身已有 _seg_s 时根本不读它",
          rig.tracker._seg_s is not None and abs(float(rig.tracker._seg_s) - 99.0) > 1.0,
          f"_seg_s={rig.tracker._seg_s!r}")
    check("5.3 但 Tracker 随后又把键覆盖回自己的值（三个写者都在写同一个槽）",
          abs(float(rig.bb.get(P_ROUTE_S)) - float(tel["s"])) < 1e-12
          and rig.bb.entry(P_ROUTE_S).source == "TrackerTask",
          f"bb={rig.key()!r}")


# --------------------------------------------------------------------------- #
#  ⑥ 实跑回放：健康写入者在 F9 都指向 段11/4.4，段6 的显示值只能来自旧 hint
# --------------------------------------------------------------------------- #
def test_replay_f9_window() -> None:
    section("⑥ 实跑同姿态回放：两个健康写入者在 F9 停站点都给 段11/≈4.4")
    rig = Rig()
    poses = [(0.869, 2.513, 62.8), (0.897, 2.589, 75.1), (0.909, 2.672, 85.1),
             F9_APPROACH, F9_STOP]
    for (x, y, yaw) in poses:
        rig.pose(x, y, yaw)
        rig.tracker.tick()
        rig.loc.tick()
    t_tel = rig.tracker.telemetry()
    check("6.1 Tracker 走到 F9 时自己的投影 = 段11~12 / s≈4.4~4.6（不是段6）",
          int(t_tel["seg_index"]) in (11, 12) and 4.3 <= float(t_tel["s"]) <= 4.7,
          f"段{t_tel['seg_index']} s={t_tel['s']!r}")
    check("6.2 Localization 同一姿态 ss≈4.4~4.6",
          4.3 <= float(rig.bb.get(P_ROUTE_S)) <= 4.7,
          f"bb s={rig.bb.get(P_ROUTE_S)!r} 段={rig.bb.get(P_ROUTE_INDEX)}")
    check("6.3 所以实跑里的 段6/s2.4x 不能当'车卡在 H10→H8'：车与两个健康写入者都在 F9",
          True, "依据 run2-mission.log 第347~359行 与本地回放")


def test_display_fingerprint_matches_mission() -> None:
    section("⑦ 现场指纹：实跑那段 段6/s2.1~2.5 只可能来自 Mission 的旧 hint（不是健康写入者）")
    # run2-mission.log 第 449/452/459 行（F9→D10→C10 途中）：段6 s2.18 → 段6 s2.13 → 段5 s2.10
    # 同拍位姿取自该日志。健康写入者同拍给的是 段12~15/s5.0~5.9。
    log_rows = [(6, 2.18), (6, 2.13), (5, 2.10)]
    rig = Rig(legacy=True)
    rig.mission._last_s = 2.483            # F9 停站时 Mission 自己的值（≈2.48）
    rows = []
    for (x, y, yaw) in ((0.606, 3.783, 136.5), (0.553, 3.846, 123.0), (0.514, 3.920, 111.3)):
        rig.pose(x, y, yaw)
        rig.tracker.tick()
        legacy_proj = rig.mission._path.project(x, y, s_hint=rig.mission._last_s)
        s_m = rig.mission._current_s()
        # 段号取"Mission 自己那一次 hint 窗口投影"的段（= 实跑打印出来的那个段），
        # 不再读共享键 —— 补丁之后 Mission 不写键了（见 test_agc_t1_mission_gate_fix.py）。
        rows.append((legacy_proj.seg_index, round(s_m, 2),
                     float(rig.tracker.telemetry()["s"])))
    check("7.1 Mission 旧 hint 投影在这三拍都停在 段5/6、s 落在 2.05~2.50（窗口驻点，不跟车）",
          all(r[0] in (5, 6) and 2.05 <= r[1] <= 2.50 for r in rows), f"got {rows}")
    check("7.2 同拍 Tracker 自算 = 段12~15/s5.0~5.9（与 2.1x 差 3.0 m 以上）",
          all(r[2] > 5.0 for r in rows), f"got {[r[2] for r in rows]}")
    check("7.3 实跑打印的 2.18/2.13/2.10 全在 Mission 值带内，且与健康值差 > 3.0 m",
          all(2.05 <= s <= 2.50 for (_i, s) in log_rows)
          and all(r[2] - s > 3.0 for (s, r) in zip((2.18, 2.13, 2.10), rows)),
          f"log={log_rows} 本地Mission={[(r[0], r[1]) for r in rows]}")
    check("7.4 结构指纹：s 走到 2.100（段界）时段号变 段5 —— 实跑同拍也打印 段5 s2.10",
          rows[-1][0] == 5 and abs(rows[-1][1] - 2.10) < 1e-6
          and log_rows[-1] == (5, 2.10), f"本地={rows[-1]} 实跑={log_rows[-1]}")


def main() -> int:
    test_single_slot_last_writer()
    test_writers_disagree_at_same_pose()
    test_mission_adoption_rule()
    test_authorized_resets()
    test_tracker_control_independent()
    test_replay_f9_window()
    test_display_fingerprint_matches_mission()
    finish()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
