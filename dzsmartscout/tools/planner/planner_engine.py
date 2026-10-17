#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""planner_engine.py —— 「车侧现在用的是哪个规划引擎」的单一口径读取

为什么必须单独抽出来（t60 收口轮 · 队长实锤）：
  `tools/plan_route.py` 的 `--engine` **默认是 `tour`**，而实车自动规划入口
  `tasks/mission.py:906 plan_route_from_config` 是**从 config 读** `planner.engine`
  （第五张图 = `tracker-safe`，`robot.yaml:586`）→ 走 `plan_route_tracker_safe()`。
  两处不读同一个键 ⇒ 会出现"**机算 40 token（tour）/ 车跑 44 token（tracker-safe）**"的错位，
  两条路线都能过 route_check，静默且致命。

本模块的口径：**用车上同一份代码读同一个键** —— `dzbrain.core.config.Config`
（从 vendor 树导入，与 `mission.py:399 cfg.get("planner.engine", DEFAULT_PLAN_ENGINE)` 同源），
外加与 `mission.py:391 resolve_plan_engine` 相同的"未知值→回退默认"保护。
"""
from __future__ import annotations

import pathlib
import sys

ENGINE_TOUR = "tour"
ENGINE_TRACKER_SAFE = "tracker-safe"
PLAN_ENGINES = (ENGINE_TOUR, ENGINE_TRACKER_SAFE)
DEFAULT_PLAN_ENGINE = ENGINE_TOUR


def _load_config(vendor_ws: pathlib.Path):
    """用 vendor 树里的 `dzbrain.core.config.Config` 读配置（与车上同代码）。"""
    src = str(vendor_ws / "src" / "dzbrain")
    if src not in sys.path:
        sys.path.insert(0, src)
    from dzbrain.core.config import Config          # noqa: PLC0415
    return Config


def read_engine(vendor_ws: pathlib.Path, source_ws: pathlib.Path | None = None) -> dict:
    """返回 {engine, tracker_max_seconds, max_seconds, warnings, source_values}
    —— engine 来自 `planner.engine`；未知值按 `resolve_plan_engine` 的口径回退默认并告警。"""
    warnings: list[str] = []
    Config = _load_config(vendor_ws)
    cfg = Config.load(str(vendor_ws / "config" / "robot.yaml"))
    raw = cfg.get("planner.engine", DEFAULT_PLAN_ENGINE)
    engine = str(raw).strip()
    if engine not in PLAN_ENGINES:
        warnings.append("planner.engine=%r 不在 %s 内（与 mission.resolve_plan_engine 同口径）"
                        "→ 回退 %s" % (raw, list(PLAN_ENGINES), DEFAULT_PLAN_ENGINE))
        engine = DEFAULT_PLAN_ENGINE
    out = {
        "engine": engine,
        "raw": raw,
        "tracker_max_seconds": cfg.get("planner.tracker_max_seconds", None),
        "max_seconds": cfg.get("planner.max_seconds", None),
        "allow_reverse": cfg.get("planner.allow_reverse", None),
        "map_path": cfg.get("planner.map_path", None),
        "fallback_to_tokens": cfg.get("planner.fallback_to_tokens", None),
        "auto_plan_enabled": cfg.get("planner.auto_plan_enabled", None),
        "config_path": str(vendor_ws / "config" / "robot.yaml"),
        "warnings": warnings,
    }
    if source_ws is not None:
        cfg2 = Config.load(str(source_ws / "config" / "robot.yaml"))
        out["source_engine"] = str(cfg2.get("planner.engine", DEFAULT_PLAN_ENGINE))
    return out


def read_mission_blocks(config_path: pathlib.Path) -> list[str]:
    """从 robot.yaml 里递归找 `mission_blocks`（车侧"无外部路线时的板载回退路线"）。"""
    import yaml
    data = yaml.safe_load(pathlib.Path(config_path).read_text(encoding="utf-8")) or {}
    found: list[str] = []

    def walk(node):
        nonlocal found
        if isinstance(node, dict):
            for k, v in node.items():
                if str(k) == "mission_blocks" and isinstance(v, (list, tuple)):
                    found = [str(x).strip() for x in v if str(x).strip()]
                walk(v)
        elif isinstance(node, (list, tuple)):
            for v in node:
                walk(v)

    walk(data)
    return found
