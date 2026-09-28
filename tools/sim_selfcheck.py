#!/usr/bin/env python3
"""赛前离线自检：不出机房、不用 ROS，跑一遍完整「侦察 -> 规划 -> 执行 -> 打击」闭环。

用途：
    * 改了速度/预算/投票参数后，先看 180 s 够不够、能拿多少分
    * 验证「多帧投票」能否扛住误检噪声
    * CI 里当冒烟测试跑（退出码非 0 即失败）

    python tools/sim_selfcheck.py
    python tools/sim_selfcheck.py --speed 0.45 --noise-ratio 0.4 --min-votes 3
"""

from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path
from typing import List, Optional

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
# ROS2 的 Python 包是 src/<pkg>/<pkg>/ 两层，所以每个包目录都要单独入 path
# （与 tests/conftest.py 同一套规则；少加一个就会出现 ModuleNotFoundError）
for _p in [SRC] + sorted(p for p in SRC.iterdir() if p.is_dir()):
    sys.path.insert(0, str(_p))

from ag_core.grid_map import CellID, FieldGrid  # noqa: E402
from ag_core.link_protocol import FieldIntel  # noqa: E402
from ag_core.mission_planner import MissionPlanner, TaskType  # noqa: E402
from ag_core.postprocess import Detection  # noqa: E402
from ag_core.target_policy import TargetPolicy  # noqa: E402
from ag_mission.field_fusion import IntelVoteConfig, IntelVoter  # noqa: E402

DEFAULT_MINES = ["B2", "C3", "D4", "E5", "F6", "G7", "H8", "I9"]
DEFAULT_OBSTACLES = ["A5", "J7"]
NOISE_POOL = ["A9", "K2", "D11"]


def gen_frames(mines, obstacles, bridge_a, bridge_b, rounds, noise_ratio, seed) -> List[FieldIntel]:
    """模拟无人机逐帧揭示 + 偶发误检。"""
    rng = random.Random(seed)
    frames: List[FieldIntel] = []
    seq = 0
    for _ in range(rounds):
        for k in range(2, len(mines) + 1):
            seq += 1
            cur = list(mines[:k])
            if rng.random() < noise_ratio:
                cur.append(rng.choice(NOISE_POOL))
            frames.append(FieldIntel(mines=sorted(cur), obstacles=list(obstacles),
                                     bridge_a=bridge_a, bridge_b=bridge_b,
                                     target_cells=["D2", "E2"], target_face="blue",
                                     ts=float(seq), seq=seq))
        # 侦察完成后持续发全量帧
        for _ in range(6):
            seq += 1
            frames.append(FieldIntel(mines=list(mines), obstacles=list(obstacles),
                                     bridge_a=bridge_a, bridge_b=bridge_b,
                                     target_cells=["D2", "E2"], target_face="blue",
                                     ts=float(seq), seq=seq))
    return frames


def run_mission(frames, mines, start, goal, speed, budget, margin, min_votes, ratio, ttl,
                confirm_frames: int = 5):
    """喂帧直到情报连续 confirm_frames 帧不再变化，才认为可以出发。

    这一点很关键：情报在无人机侦察过程中是逐帧增长的，
    过早出发会拿着残缺情报排雷，直接损失分值。
    """
    grid = FieldGrid()
    planner = MissionPlanner(grid, CellID.parse(start), CellID.parse(goal),
                             speed_mps=speed, budget_s=budget, goal_margin_s=margin)
    voter = IntelVoter(IntelVoteConfig(min_votes=min_votes, min_ratio=ratio, ttl_s=ttl))
    used_frames = 0
    last_key = None
    stable_run = 0

    for fr in frames:
        used_frames += 1
        if not voter.push(fr, now=fr.ts):
            continue
        if not voter.is_confident:
            continue
        st = voter.stable
        planner.set_intel(mines=st.mines, obstacles=st.obstacles,
                          bridge_a=st.bridge_a, bridge_b=st.bridge_b,
                          target_cell=st.target_cells[0] if st.target_cells else None)
        key = (tuple(st.mines), st.bridge_a, st.bridge_b)
        stable_run = stable_run + 1 if key == last_key else 1
        last_key = key
        if stable_run >= confirm_frames:
            break

    plan = planner.full_plan()
    planned = {c.name for c in planner.mines if isinstance(c, CellID)}
    false_cells = sorted(planned - set(mines))
    return {
        "frames_used": used_frames,
        "plan": plan,
        "score": planner.estimate_score(plan),
        "time": planner.estimate_time(plan),
        "mines_done": sum(1 for t in plan if t.type == TaskType.MINE),
        "bridges_done": sum(1 for t in plan if t.type in (TaskType.BRIDGE_A, TaskType.BRIDGE_B)),
        "target_done": sum(1 for t in plan if t.type == TaskType.TARGET),
        "false_cells": false_cells,
    }


def run_strike(max_frames: int = 30) -> dict:
    """模拟敌我识别链路：连续帧确认蓝面 -> 开火 -> 命中计数。"""
    policy = TargetPolicy(conf_thres=0.60, consensus_frames=3, img_w=640, img_h=480)
    det = Detection(cls=0, score=0.95, x1=300, y1=220, x2=340, y2=260,
                    kpts=np.array([[320.0, 240.0, 0.9]], dtype=np.float32))
    fired = 0
    for _ in range(max_frames):
        d = policy.update([det])
        if d.action.value == "fire":
            policy.on_shot_result(hit=True, on_enemy=True)
            fired += 1
            policy._history.clear()  # 开火后重新积累一致性（对应节点里的射击冷却）
        if policy.target_destroyed or policy.ammo_left <= 0:
            break
    return {
        "fired": fired,
        "enemy_count": policy.enemy_count,
        "friend_count": policy.friend_count,
        "score": policy.expected_score,
        "ammo_left": policy.ammo_left,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="赛前离线自检")
    ap.add_argument("--mines", default=",".join(DEFAULT_MINES))
    ap.add_argument("--obstacles", default=",".join(DEFAULT_OBSTACLES))
    ap.add_argument("--bridge-a", default="K4")
    ap.add_argument("--bridge-b", default="K9")
    ap.add_argument("--start", default="N10")
    ap.add_argument("--goal", default="N11")
    ap.add_argument("--speed", type=float, default=0.55)
    ap.add_argument("--budget", type=float, default=180.0)
    ap.add_argument("--margin", type=float, default=12.0)
    ap.add_argument("--min-votes", type=int, default=2)
    ap.add_argument("--ratio", type=float, default=0.5)
    ap.add_argument("--ttl", type=float, default=20.0)
    ap.add_argument("--rounds", type=int, default=2)
    ap.add_argument("--noise-ratio", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=20260927)
    ap.add_argument("--confirm-frames", type=int, default=5,
                    help="情报连续多少帧不再变化才出发")
    args = ap.parse_args()

    mines = [m.strip().upper() for m in args.mines.split(",") if m.strip()]
    obstacles = [o.strip().upper() for o in args.obstacles.split(",") if o.strip()]

    frames = gen_frames(mines, obstacles, args.bridge_a, args.bridge_b,
                        args.rounds, args.noise_ratio, args.seed)
    m = run_mission(frames, mines, args.start, args.goal, args.speed, args.budget,
                    args.margin, args.min_votes, args.ratio, args.ttl, args.confirm_frames)
    s = run_strike()

    # 注意：planner 里的 TARGET 任务已按 15 分计入行驶得分（= 打击满分），
    # 打击链只在没能打满 3 次计数 / 误击时才产生差额。
    strike_delta = s["score"] - 15
    total = m["score"] + min(0, strike_delta)
    print("=" * 62)
    print("情报融合")
    print(f"  帧数: {len(frames)}，规划触发于第 {m['frames_used']} 帧")
    print(f"  误报格号: {', '.join(m['false_cells']) or '无'}")
    print("任务执行")
    print(f"  排雷 {m['mines_done']}/{len(mines)}  桥洞 {m['bridges_done']}/2  打击任务 {m['target_done']}/1")
    print(f"  预估耗时: {m['time']:.1f} s / {args.budget:.0f} s")
    print(f"  行驶得分: {m['score']}")
    print("打击链路")
    print(f"  开火 {s['fired']} 次，敌计数 {s['enemy_count']}，友计数 {s['friend_count']}，余弹 {s['ammo_left']}")
    print(f"  打击得分: {s['score']}（行驶得分里的 TARGET 任务已按 15 分计入）")
    print("-" * 62)
    print(f"预估总分: {total}  (满分 5+5+8*5+2*10+15 = 85)")
    print("=" * 62)

    failures: List[str] = []
    if m["time"] > args.budget:
        failures.append(f"耗时 {m['time']:.1f}s 超过预算 {args.budget:.0f}s")
    if m["false_cells"]:
        failures.append(f"情报出现误报: {m['false_cells']}")
    if m["mines_done"] == 0:
        failures.append("一颗雷都没排进计划")
    if s["friend_count"] > 0:
        failures.append("误击友方")
    if m["target_done"] and s["enemy_count"] < 3:
        failures.append(f"计划里含打击任务，但敌计数只有 {s['enemy_count']}/3")

    if failures:
        for f in failures:
            print(f"[FAIL] {f}")
        return 1
    print("[PASS] 闭环自检通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
