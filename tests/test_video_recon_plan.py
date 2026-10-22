# -*- coding: utf-8 -*-
"""video_recon 的规划落盘回归：桥门"按段回退默认值"。

背景（2026-10-21 机上实测事故）：桥洞规则是 K 列分两段（K1~K6 / K7~K11）
各有且只有一个箭头。视频只拍到一个段的箭头时，原实现 `passable or 默认`
不回退，另一段整段判墙 → 起点连通域被切断 → `plan_route` 抛
"goal N5 不可达"，整条视频侦察在最后一步失败。

这里锁定三件事：
  1. 逐段合并语义（有则用检出、无则补默认）；
  2. 只有 K10 的实测态势也能出路线（且 K4 标注为默认来源）；
  3. 路线满足既有验收口径：雷全覆盖、不穿障碍。
"""
from __future__ import annotations

import json

import pytest

from ag_integrate import video_recon as vr


def _state(mines, passable, obstacles, red=(), blue=()):
    return {
        "mines_video": list(mines),
        "passable_video": list(passable),
        "obstacles_video": list(obstacles),
        "targets_video": {"red": list(red), "blue": list(blue)},
        "car_start_video": [], "car_end_video": [], "uav_marks_video": [],
        "votes_detail": {}, "coverage": {}, "phantom_log": {},
    }


@pytest.mark.parametrize("detected, expect_gates, expect_default", [
    ([], ["K4", "K9"], ["K4", "K9"]),          # 全空 → 两段都默认
    (["K10"], ["K4", "K10"], ["K4"]),          # 只拍到上段 → 下段默认
    (["K2"], ["K2", "K9"], ["K9"]),            # 只拍到下段 → 上段默认
    (["K4", "K10"], ["K4", "K10"], []),        # 两段都拍到 → 无需默认
])
def test_merge_passable_by_segment(detected, expect_gates, expect_default):
    gates, from_default = vr.merge_passable_with_default(detected)
    assert gates == expect_gates
    assert from_default == expect_default


def test_merge_passable_ignores_garbage():
    """坏名字（非 K 列/非数字/越界）一律忽略，宁可用默认。"""
    gates, from_default = vr.merge_passable_with_default(
        ["K", "Kx", "K0", "K12", "A5", None, 7])
    assert gates == ["K4", "K9"]
    assert from_default == ["K4", "K9"]


def test_plan_succeeds_on_single_gate_video(tmp_path):
    """机上实测态势（只检出 K10）：必须能出路线，且标注 K4 来自默认。"""
    state = _state(
        mines=["F5", "G9"], passable=["K10"],
        obstacles=["C9", "F8", "H11"], red=["D5"], blue=["B7"])
    plan = vr.plan_from_state(state, tmp_path)

    assert plan["route_len_cells"] > 0
    assert plan["source"]["passable"] == ["K4", "K10"]
    assert plan["source"]["passable_from_default"] == ["K4"]
    # 既有验收口径：所有雷都被路线覆盖、且不穿任何障碍格
    route = set(plan["route"])
    assert set(state["mines_video"]) <= route
    assert not (set(state["obstacles_video"]) & route)
    # 三产物里的两件（渲染图依赖 PIL/CJK 字体，单独由冒烟脚本覆盖）
    assert (tmp_path / "route_plan.json").exists()
    assert json.loads((tmp_path / "route_plan.json").read_text(encoding="utf-8"))


# ---------------------------------------------------------------- 单格单标签
def test_conflict_resolved_by_votes():
    """票多者胜：B7 桶 31 票 vs 蓝靶 5 票 → 判为障碍，靶被剔除。

    这是实测中自相矛盾：不裁决时 B7 同时是蓝靶和障碍，
    规划器找不到可停靠射击邻格 → 车命令里一个 '*' 都没有。
    """
    roles = {"mine": [], "passable": [], "obstacle": ["B7"],
             "target_red": [], "target_blue": ["B7", "D11"]}
    votes = {"obstacle": {"B7": 31}, "target_blue": {"B7": 5, "D11": 5},
             "mine": {}, "target_red": {}}
    out, conflicts = vr.resolve_cell_conflicts(roles, votes)

    assert out["obstacle"] == ["B7"]
    assert out["target_blue"] == ["D11"]        # B7 被剔除，D11 保留
    assert len(conflicts) == 1
    assert conflicts[0]["cell"] == "B7" and conflicts[0]["winner"] == "obstacle"


def test_conflict_tie_favors_target():
    """平票时按语义优先级：靶 > 桶/障 > 雷。"""
    roles = {"mine": [], "passable": [], "obstacle": ["C5"],
             "target_red": ["C5"], "target_blue": []}
    votes = {"obstacle": {"C5": 6}, "target_red": {"C5": 6},
             "mine": {}, "target_blue": {}}
    out, conflicts = vr.resolve_cell_conflicts(roles, votes)
    assert out["target_red"] == ["C5"]
    assert out["obstacle"] == []
    assert conflicts[0]["winner"] == "target_red"


def test_passable_gate_never_arbitrated():
    """K 列桥门不参与裁决——门格被占用是规划器自己的告警路径。"""
    roles = {"mine": [], "passable": ["K5"], "obstacle": ["K5"],
             "target_red": [], "target_blue": []}
    votes = {"obstacle": {"K5": 99}, "mine": {}, "target_red": {},
             "target_blue": {}}
    out, conflicts = vr.resolve_cell_conflicts(roles, votes)
    assert out["passable"] == ["K5"]
    assert out["obstacle"] == ["K5"]
    assert conflicts == []


# ---------------------------------------------------------------- 靶先验裁决
def test_arbitrate_static_targets_as_buckets():
    """全场唯一靶=移动双靶：别处检出的"静靶"一律改判为桶。

    2026-10-21 用户核对裁图后给出的场地真值：`D5`/`B7`/`D11` 都是**桶**；
    检测框尺寸与"障碍桶"检出同级（22~25% 帧宽），而真靶是立式薄板。
    """
    blue, red, buckets, obs = vr.arbitrate_targets(["B7", "D11"], ["D5", "I6"])
    assert (blue, red) == ([], [])
    assert buckets == ["B7", "D11", "D5", "I6"]     # 字典序（D11 < D5）
    assert obs == {}


def test_arbitrate_track_cell_is_target_observation():
    """轨道（D7~E7）±1 格内的检出 = 移动靶本体观测（**不是**误检，不许剔除）。

    2026-10-21 用户真值：E7 就是靶（移动双靶本体），红面=友方。
    邻域 = 切比雪夫距离 ≤1：D7/E7 周围 C5~F7 共 12 格（含 C7、D8、E8）。
    """
    blue, red, buckets, obs = vr.arbitrate_targets(["E7"], ["E8", "A1"])
    assert obs == {"E7": "blue", "E8": "red"}
    assert buckets == ["A1"]                # 邻域外 → 按静靶改判为桶
    assert (blue, red) == ([], [])


def test_arbitrate_red_wins_same_cell():
    """同格红蓝冲突 → 红优先：红=友方，误判成蓝会去误击友方（−5），反向最多少打。"""
    _, _, _, obs = vr.arbitrate_targets(["E7"], ["E7"])
    assert obs == {"E7": "red"}


def test_arbitrate_can_be_disabled():
    """练习场真摆了静靶时，--allow-static-targets 关掉"静靶→桶"，静靶原样保留。

    但轨道本体的观测判定不受开关影响（那是裁判硬件先验）。
    """
    blue, red, buckets, obs = vr.arbitrate_targets(
        ["B7"], ["E7"], static_is_bucket=False)
    assert blue == ["B7"] and red == []     # B7 保留为静靶
    assert buckets == []
    assert obs == {"E7": "red"}             # 轨道观测仍生效


def test_plan_red_face_track_target_keeps_two_stage_attack(tmp_path):
    """端到端：静靶并入障碍、轨道靶保留**两段式打击位**，红面观测如实留痕。

    2026-10-21 用户两次纠正后的口径：
      ① "e7 靶子，但红色靶面为队友" → 红面不能打；
      ② "这里面没有标靶打击" → 计划里必须留打击环节。
    两者同时满足的形态 = 两段式：计划排蓝面侧确认/射击停靠位（`*` 在），
    实弹由在飞 fire_guard 按实时画面复判（红面/未辨 → 禁火）。
    """
    state = _state(mines=["F5"], passable=["K10"], obstacles=["C9"],
                   red=["D5", "E7"], blue=["B7", "D11"])
    plan = vr.plan_from_state(state, tmp_path)

    src = plan["source"]
    assert src["targets_reclassified_as_bucket"] == ["B7", "D11", "D5"]
    assert src["targets_track_observed"] == {"E7": "red"}
    assert "两段式" in src["target_attack_mode"]
    snap = plan["snapshot"]
    assert snap["targets"] == ["轨道靶@E7（可打击=是；视频观测红面(友)）"], snap["targets"]
    assert set(["B7", "D11", "D5"]).issubset(set(snap["obstacles"]))
    assert "E7" not in snap["obstacles"], "靶本体（轨道格）不该被当桶"
    # 打击环节必须在：蓝面侧停靠位 + 命令里的 '*'
    assert plan["car_cmds"]["shoot_cells"], "两段式必须排确认/射击停靠位"
    assert "*" in plan["car_cmds"]["route"]
    # 红面观测必须显式提醒（静态回放会误击）
    assert "两段式" in plan["check"]["友方面禁火"]
    assert "手动去掉" in plan["check"]["友方面禁火"]
    # 轨道本体不可通行（靶格）；停靠位在蓝面侧（轨道端点的南邻）
    assert "E7" not in plan["route"] and "D7" not in plan["route"]
    assert set(plan["car_cmds"]["shoot_cells"]) <= {"D8", "E8"}


def test_plan_blue_face_track_target_allows_shooting(tmp_path):
    """反向守卫：轨道靶观测到蓝面（敌）时同样排射击停靠，且无红面提醒。"""
    state = _state(mines=["F5"], passable=["K10"], obstacles=["C9"],
                   red=[], blue=["E7"])
    plan = vr.plan_from_state(state, tmp_path)
    assert plan["snapshot"]["targets"] == ["轨道靶@E7（可打击=是；视频观测蓝面(敌)）"]
    assert plan["car_cmds"]["shoot_cells"], "蓝面敌靶应派射击停靠"
    assert "*" in plan["car_cmds"]["route"]
    assert "友方面禁火" not in plan["check"]


def test_plan_without_track_observation_uses_referee_prior(tmp_path):
    """视频没拍到靶：按裁判先验补轨道靶（蓝面朝南），照常排射击位。"""
    state = _state(mines=["F5"], passable=["K10"], obstacles=["C9"],
                   red=[], blue=[])
    plan = vr.plan_from_state(state, tmp_path)
    assert plan["snapshot"]["targets"] == ["轨道靶@D7（视频未观测，用裁判先验）"]
    assert plan["source"]["targets_track_observed"] == {}
    assert "*" in plan["car_cmds"]["route"]


def test_plan_reports_mine_shortfall_against_baseline(tmp_path):
    """"排雷覆盖✓"只对已检出的雷负责；检出不足 8 颗时必须显式标出漏检风险。

    2026-10-22 实测：通用入口用 stride 15 + 旧门控跑某段视频只检出 2 颗雷，check 仍显示
    "排雷覆盖: 全部覆盖"，用户看渲染图当场问"这航点缺雷"。基线是 field_rules.MAX_MINES=8。
    """
    state = _state(mines=["A2", "C3"], passable=["K10"], obstacles=["C9"],
                   red=[], blue=[])
    plan = vr.plan_from_state(state, tmp_path)
    note = plan["check"]["排雷覆盖"]
    assert note.startswith("全部覆盖")
    assert "2/8" in note and "漏检" in note, note


def test_plan_does_not_warn_when_baseline_met(tmp_path):
    """检出满 8 颗时不该出现漏检提示（避免长期告警变成噪声）。"""
    state = _state(mines=["A2", "B1", "C3", "D4", "E3", "F5", "G9", "H8"],
                   passable=["K10"], obstacles=["C9"], red=[], blue=[])
    plan = vr.plan_from_state(state, tmp_path)
    assert "漏检" not in plan["check"]["排雷覆盖"], plan["check"]


def test_plan_warns_when_mines_exceed_baseline(tmp_path):
    """检出超过官方基线 8 颗时同样必须告警：多出来的一定是误检。"""
    state = _state(mines=["A2", "B1", "C3", "D4", "E2", "E3", "F5", "G3", "G9", "I2"],
                   passable=["K10"], obstacles=["C9"], red=[], blue=[])
    plan = vr.plan_from_state(state, tmp_path)
    note = plan["check"]["排雷覆盖"]
    assert "10" in note and "误检" in note, note


def _meta(frames: int, q, conf: float, labels=("B3", "B5", "B7")) -> list[dict]:
    """构造逐帧识别明细：同一目标连续 N 帧落在 q（连续格坐标）。"""
    return [{"frame": i, "grid_ok": True, "labels": list(labels),
             "objs": [{"cls": "paper", "conf": conf, "q": list(q)}]}
            for i in range(frames)]


def test_aggregate_keeps_strong_vote_low_confidence_mine():
    """≥3 帧独立同意的雷不被子框阈值的均值置信度门卡掉（2026-10-22 G9 事故）。

    放宽单框阈值(0.22→0.15)会拉低命中的均值置信度，若均值门仍对"票数刚好够"
    的格子生效，就会把真雷一起卡掉——同一段视频上一版反而检得到。3 票已足够
    作为独立证据，均值门只该拦 2 票的弱格。
    """
    st = vr.aggregate_state(_meta(3, (1.0, 4.0), 0.336))
    assert "B5" in st["mines_video"], st["votes_detail"]["paper"]

    st_weak = vr.aggregate_state(_meta(2, (1.0, 4.0), 0.336))
    assert "B5" not in st_weak["mines_video"], st_weak["votes_detail"]["paper"]


def test_reaggregate_matches_full_run_on_saved_meta(tmp_path):
    """--reaggregate 用 frames_meta 重算，必须与完整跑一遍的态势一致（单点派生）。"""
    meta = (_meta(3, (1.0, 4.0), 0.60, labels=("B3", "B5", "B7"))
            + _meta(2, (6.0, 2.0), 0.55, labels=("G1", "G3", "G5")))
    (tmp_path / "frames_meta.json").write_text(
        json.dumps(meta, ensure_ascii=False), encoding="utf-8")
    st = vr.aggregate_state(meta, video="x.mp4", stride=5)
    (tmp_path / "field_state.json").write_text(
        json.dumps(st, ensure_ascii=False), encoding="utf-8")
    assert vr.main(["--reaggregate", str(tmp_path)]) == 0
    plan = json.loads((tmp_path / "route_plan.json").read_text(encoding="utf-8"))
    assert "B5" in plan["snapshot"]["mines"] and "G3" in plan["snapshot"]["mines"]
