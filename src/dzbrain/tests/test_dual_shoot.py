# -*- coding: utf-8 -*-
"""CAR-DUAL-20261024: full tour output must contain two real firing stops.

Run: python -B src/dzbrain/tests/test_dual_shoot.py
Uses the production planner and route_check; no vehicle/ROS access or writes.
"""
from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent))

from dzbrain.core.config import Config
from dzbrain.route import planner as P
from dzbrain.route import parser
from dzbrain.tasks.mission import plan_route_from_config
from harness import check, check_raises, finish, run_case, section

MAP = HERE / "fixtures" / "map_carloop_20261024.yaml"
RAW = yaml.safe_load(MAP.read_text(encoding="utf-8"))
CFG = Config.load(str(ROOT / "config" / "robot.yaml"))
SPEC = P.load_map(RAW)
ACCEPT = P._shoot_accept(CFG)


def plan(raw=RAW, objective="legacy"):
    return P.plan_route(raw, CFG, cross_lower="K10", cross_upper="K4",
                        allow_reverse=False, objective=objective, max_seconds=10.0)


def candidates():
    section("真实 t27 地图候选与几何")
    pool = P.shoot_candidates(SPEC, P.Grid(SPEC), ACCEPT)
    check("首选合法候选 C5/F9；排除雷 F5 与障碍 C9",
          pool[:2] == ["C5", "F9"] and "F5" not in pool and "C9" not in pool, str(pool))
    check("F9 的最佳对侧是 C5", P.second_shoot_stop(SPEC, P.Grid(SPEC), "F9", ACCEPT) == "C5")
    check("两站各覆盖全靶且异侧", not P.check_shoot_stops(["F9", "C5"], SPEC, ACCEPT))
    check("两站到板心均 0.875 m", all(abs(P.shoot_stop_distance(b, SPEC) - 0.875) < 1e-9
                                               for b in ("F9", "C5")))


def complete_routes():
    section("完整 tour 直接产物：不依赖 Mission 或 CLI 事后补星")
    # Before-change geometry hashes from planner-before.json, 2026-10-24.
    # Only firing flags may change here; engine/objective/route shape must not.
    baselines = {
        "legacy": (62, 22.75, "29a9a4dd46f730c53dddbf82255cc6ed213a27081506381b699771e12f1ea269"),
        "tasks": (63, 23.45, "7e84e67df4c2e172256d37df199993c30a691204adf73e321c4cb527e91fd659"),
    }
    for objective, (count, distance, geometry_hash) in baselines.items():
        result = plan(objective=objective)
        blocks = result.shoot_blocks()
        parsed = [wp.block for _, wp in parser.parse_mission(result.tokens).shoot_points()]
        check(f"{objective}: 两站直接落星且不降级", blocks == ["F9", "C5"] and not result.degraded,
              f"stars={blocks}, degraded={result.degraded}")
        check(f"{objective}: token/route/stop/task/metadata 同源",
              parsed == blocks == [s.block for s in result.stops if s.shoot]
              == list(result.tasks_stops) == [result.shoot_stop, result.shoot_stop2])
        check(f"{objective}: 第二站距离有实际值", abs(result.shoot_distance_m2 - 0.875) < 1e-9)
        check(f"{objective}: 两个不同合法异侧点均覆盖全靶",
              not P.check_shoot_stops(blocks, SPEC, ACCEPT))
        geometry = " ".join(t.replace("*", "") for t in result.tokens)
        digest = hashlib.sha256(geometry.encode()).hexdigest()
        check(f"{objective}: 不变更路线几何/目标函数",
              len(result.tokens) == count and abs(result.polyline_m - distance) < 1e-8
              and digest == geometry_hash,
              f"{len(result.tokens)} token, {result.polyline_m:.2f} m, hash={digest}")
        closed, marked, _ = P.ensure_second_shoot_stop(result.tokens, RAW, CFG)
        check(f"{objective}: 已有合法双站收口幂等", closed == result.tokens and marked == "")
        if objective == "legacy":
            env = dict(os.environ, ROBOT_MAP=str(MAP), PYTHONUTF8="1",
                       PYTHONIOENCODING="utf-8", PYTHONDONTWRITEBYTECODE="1")
            proc = subprocess.run([sys.executable, "-B", str(ROOT / "tools" / "route_check.py"),
                                   "--config", str(ROOT / "config" / "robot.yaml"),
                                   "--route", " ".join(result.tokens)],
                                  cwd=str(ROOT), env=env, capture_output=True, text=True,
                                  encoding="utf-8", errors="replace", timeout=60)
            check("真实图产物通过现有 route_check", proc.returncode == 0,
                  f"exit={proc.returncode}" if proc.returncode == 0 else proc.stdout + proc.stderr)


def closeout():
    section("收口检验不以星号数量冒充双站")
    for tokens in (["F9*", "F5*"], ["F9*", "C9*"], ["F9*", "F9*"],
                   ["F9*", "C5*", "F9*"]):
        check_raises("拒绝非法/重复/多余射击站 " + " ".join(tokens), P.PlannerError,
                     P.ensure_second_shoot_stop, tokens, RAW, CFG)
    same_side = copy.deepcopy(RAW)
    same_side["map"]["obstacles"].remove("C9")
    check_raises("两个合法但同侧的点仍拒绝", P.PlannerError,
                 P.ensure_second_shoot_stop, ["F9*", "C9*"], same_side, CFG)
    tokens = ["F9*", "C6", "R:C5:PIVOT", "C4"]
    closed, marked, _ = P.ensure_second_shoot_stop(tokens, RAW, CFG)
    check("单站只补合法对侧星，保留倒车/PIVOT与顺序",
          closed == ["F9*", "C6", "R:C5*:PIVOT", "C4"] and marked == "C5", str(closed))
    check("只验局部分段不强制两站",
          not P.check_shoot_stops(["F9"], SPEC, ACCEPT, require_pair=False))
    out, marked, note = P.ensure_second_shoot_stop(["F9*", "E9"], RAW, CFG)
    check("对侧不在局部路径时明确报告，不擅自改路",
          out == ["F9*", "E9"] and not marked and "不在本路线上" in note, note)
    check("坏格名返回诊断，不泄漏坐标转换异常",
          bool(P.check_shoot_stops(["F9", "BAD"], SPEC, ACCEPT)))
    check_raises("非法单星不能在补标时变成非法双站成功", P.PlannerError,
                 P.ensure_second_shoot_stop, ["F5*", "F9"], RAW, CFG)
    check("单点远站不被强制补站或拒绝",
          P.ensure_second_shoot_stop(["D11*"], RAW, CFG)[0] == ["D11*"])


def entrypoints():
    section("真实 Mission 与 CLI 入口保留原生双站及元数据")
    result = plan()
    cfg = Config.load(str(ROOT / "config" / "robot.yaml"))
    cfg.override("planner.engine", "tour")
    cfg.override("planner.allow_reverse", False)
    cfg.override("planner.max_seconds", 10.0)
    outcome = plan_route_from_config(cfg, map_path=str(MAP), cross_lower="K10", cross_upper="K4")
    check("Mission 入口与原生产物双站一致",
          outcome.ok and outcome.tokens == result.tokens
          and list(outcome.shoot_stops) == result.shoot_blocks()
          and outcome.second_stop_added == "", outcome.error or str(outcome.shoot_stops))
    env = dict(os.environ, PYTHONUTF8="1", PYTHONIOENCODING="utf-8",
               PYTHONDONTWRITEBYTECODE="1")
    proc = subprocess.run([sys.executable, "-B", str(ROOT / "tools" / "plan_route.py"),
                           "--config", str(ROOT / "config" / "robot.yaml"), "--map", str(MAP),
                           "--engine", "tour", "--objective", "legacy", "--no-reverse",
                           "--cross-lower", "K10", "--cross-upper", "K4", "--max-seconds", "10",
                           "--check", "--validate", "--json", "--quiet"],
                          cwd=str(ROOT), env=env, capture_output=True, text=True,
                          encoding="utf-8", errors="replace", timeout=60)
    check("CLI 真实图通过 route_check 与 validate_route", proc.returncode == 0,
          f"exit={proc.returncode}" if proc.returncode == 0 else proc.stdout + proc.stderr)
    payload = json.loads(proc.stdout)
    data = payload["plans"][0]
    check("CLI JSON 双站/距离/任务字段与原生产物一致",
          all(data[k] == result.as_dict()[k] for k in (
              "tokens", "shoot_stops", "shoot_stop", "shoot_stop2", "shoot_distance_m",
              "shoot_distance_m2", "second_stop_added", "tasks_stops")), str(data["shoot_stops"]))


def missing_second():
    section("完整规划无法兑现对侧站：单站必须显式降级（不许静默成功）")
    one_side = copy.deepcopy(RAW)
    one_side["map"]["obstacles"].append("C5")
    spec = P.load_map(one_side)
    check("负例确实没有可用对侧点", not P.second_shoot_stop(spec, P.Grid(spec), "F9", ACCEPT))
    # ★RECT-PLAN-FIX-20261026：原断言（before）
    #     check_raises("legacy 完整规划拒绝单站假成功", P.PlannerError, plan, one_side)
    #   期望：单站 -> 抛 PlannerError（整趟判死）。
    #   现断言（after）：**意图不变**（单站不得静默成功），机制从"抛错"换成"显式降级"：
    #     degraded=True + degraded_reason 命中三分支之一 + 进 warnings + 路线里确实只有 1 个星。
    single = plan(one_side)
    check("legacy 单站不得静默成功：degraded=True 且理由进 warnings",
          bool(single.degraded) and bool(single.degraded_reason)
          and any(w.startswith("单站降级") for w in single.warnings),
          f"degraded={single.degraded}, reason={single.degraded_reason}")
    check("降级文案命中三分支之一，且点名被排除的对侧格",
          "没有可入池的合法候选" in single.degraded_reason
          and "C5（障碍格）" in single.degraded_reason,
          single.degraded_reason)
    check("单站就是单站：路线里恰好 1 个星（不是伪造双站）",
          [t for t in single.tokens if t.endswith("*")] == ["F9*"],
          str([t for t in single.tokens if t.endswith("*")]))
    degraded = plan(one_side, "tasks")
    check("tasks 保留既有降级语义且带原因", degraded.degraded and bool(degraded.degraded_reason),
          f"degraded={degraded.degraded}, reason={degraded.degraded_reason}")
    check("无靶局部分段不被双站门拦截",
          P.ensure_second_shoot_stop(["N10", "M10"], {"map": {**RAW["map"], "targets": []}}, CFG)[0]
          == ["N10", "M10"])


if __name__ == "__main__":
    for name, case in (("candidates", candidates), ("complete_routes", complete_routes),
                       ("closeout", closeout), ("entrypoints", entrypoints),
                       ("missing_second", missing_second)):
        run_case(name, case)
    finish()
