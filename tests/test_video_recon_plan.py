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

    这是 2026-10-21 冠军配置实测的自相矛盾：不裁决时 B7 同时是蓝靶和障碍，
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
def test_reclassify_static_targets_as_buckets():
    """全场唯一靶=移动双靶：别处检出的"静靶"一律改判为桶。

    2026-10-21 机上物证：B7/D11 格内是蓝色圆桶、D5 是红色圆桶、I6 同级圆物，
    检测框尺寸与"障碍桶"检出同级（22~25% 帧宽），而真靶是立式薄板；
    与 2026-10-18 审计"该分类头会把蓝桶误判为蓝靶"同款。
    """
    blue, red, buckets, conflict = vr.reclassify_targets(
        ["B7", "D11"], ["D5", "I6"])
    assert (blue, red) == ([], [])
    assert buckets == ["B7", "D11", "D5", "I6"]     # 字典序（D11 < D5）
    assert conflict == []


def test_reclassify_track_neighborhood_is_referee_conflict():
    """轨道（D7~E7）±1 格内的检出 = 移动靶误投影：剔除并留痕，不当静靶也不当桶。

    邻域 = 切比雪夫距离 ≤1：D7/E7 周围 C5~F7 共 12 格（含 C7、D8、E8）。
    """
    blue, red, buckets, conflict = vr.reclassify_targets(["E7"], ["E8", "A1"])
    assert conflict == ["E7", "E8"]
    assert buckets == ["A1"]                # 邻域外 → 按静靶改判为桶
    assert (blue, red) == ([], [])


def test_reclassify_can_be_disabled():
    """练习场真摆了静靶时，--allow-static-targets 关掉裁决，静靶原样保留。"""
    blue, red, buckets, conflict = vr.reclassify_targets(
        ["B7"], ["E7"], static_is_bucket=False)
    assert blue == ["B7"] and red == []     # B7 保留
    assert buckets == []
    assert conflict == ["E7"]               # 轨道邻域仍剔除（裁判硬件先验）


def test_plan_reclassifies_and_injects_mobile_prior(tmp_path):
    """端到端：静靶并入障碍、轨道检出剔除、补回移动靶先验、留痕写入 route_plan。

    修复前（2026-10-21 机上全量跑的实况）：5 个"靶"全是桶，且因为 targets 非空
    而完全不注入移动靶先验 → 车不会去打移动靶，渲染图看起来"靶一堆、桶没了"。
    """
    state = _state(mines=["F5"], passable=["K10"], obstacles=["C9"],
                   red=["D5", "E7"], blue=["B7", "D11"])
    plan = vr.plan_from_state(state, tmp_path)

    src = plan["source"]
    assert src["targets_reclassified_as_bucket"] == ["B7", "D11", "D5"]
    assert src["targets_reclassified_referee_conflict"] == ["E7"]
    snap = plan["snapshot"]
    assert "蓝@D7（轨道靶）" in snap["targets"], snap["targets"]
    assert set(["B7", "D11", "D5"]).issubset(set(snap["obstacles"]))
    assert "E7" not in snap["obstacles"]
    assert plan["car_cmds"]["shoot_cells"], "移动靶先验没有生成停靠/确认格"
    assert "*" in plan["car_cmds"]["route"], "车命令丢了开火标记"
    # 裁决后的态势不允许出现"靶格即障碍格"的自相矛盾
    tgt_cells = {t.split("@")[1].replace("（轨道靶）", "") for t in snap["targets"]}
    assert not (tgt_cells & set(snap["obstacles"]))
