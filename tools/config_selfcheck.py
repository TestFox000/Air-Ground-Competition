# -*- coding: utf-8 -*-
r"""配置 + 路线自检（本地即可跑，不需要 ROS、不需要车）。

用途：
  1. 出发前确认 config/robot.yaml 能被新版读通、关键键来自 `/dzbrain` 段而非默认值；
  2. 确认路线与 config/map.yaml 一致（雷 8/8、障不踩、两处穿洞、U 形绕行、射击点在距离窗内）；
  3. 一眼看清"当前配置到底是哪张图、哪个标定"。

用法：
    python tools\config_selfcheck.py
    python tools\config_selfcheck.py --config config\robot.yaml --map config\map.yaml
退出码：0 = 无 error；1 = 有 error（可直接用在部署前门禁里）；
        3 = --config-only：配置通过但**空间证据未校验**，不是发车放行，调用方必须看 RESULT_JSON。

★ 为什么单列 3：仓库里的静态 map.yaml 已删（D-023 每局真图）时，本工具不能假装空间已校验，
  也不能逼操作者为了拿绿去还原旧图；独立退出码让"配置可用"与"可发车"永远不是同一件事。

★ 布局模式（2026-10-26 增）：静态图缺失 **且** `planner.auto_plan_enabled=true` 时，路线由无人机
  `field_layout` 在运行期规划 —— 这不是 `MAP_INPUT_ERROR`。本工具按 `main.py::_layout_mode_enabled`
  的同一定义判定，分别给出 LAYOUT-MODE（exit 3，配置可用但空间未校验）与 MAP_INPUT_ERROR（exit 1，
  真的缺输入）。**判据必须与车端一致**，否则预检永远是假红、放行门失效。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve()
_WS = _HERE.parents[1]
sys.path.insert(0, str(_WS / "src" / "dzbrain"))

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass

import yaml  # noqa: E402

from dzbrain.core.config import Config, ConfigError  # noqa: E402
from dzbrain.route.parser import parse_mission  # noqa: E402
from dzbrain.route.validate import _map_section, summary, validate_route  # noqa: E402

CHECK_KEYS = (
    "pursuit.wheelbase_m",
    "pursuit.max_steer_rate_deg_s",
    "pursuit.curvature_gain",
    "pursuit.cruise_speed_mps",
    "pursuit.map_safety_margin_m",
    "route.cross_lower_candidates",
    "route.cross_upper_candidates",
    "route.cross_lower_default",
    "route.cross_upper_default",
    "mission.auto_start",
    "mission.shots_per_stop",
    "aim.lead_time_s",
    "aim.fire_policy",
    "aim.lost_frames_to_rescan",
    "fix.anchor_at_waypoint",
    "safety.startup_grace_s",
    "pursuit.drift_seg_gate_m",
)


def _yaml_mapping(path: str, label: str) -> dict:
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not data:
        raise ValueError(f"{label} YAML must be a nonempty mapping")
    return data


def _map_evidence(data: dict) -> dict:
    # Missing fields are UNKNOWN, not proof that the observed world is empty.
    if "map" in data and (not isinstance(data["map"], dict) or not data["map"]):
        raise ValueError("map section must be a nonempty mapping")
    section = _map_section(data)
    required = ("start", "end", "mines", "obstacles", "targets", "bridges")
    missing = [key for key in required if key not in section]
    if missing:
        raise ValueError("map evidence missing fields: " + ", ".join(missing))
    for key in ("start", "end"):
        if not isinstance(section[key], str) or not section[key]:
            raise ValueError(f"map.{key} must be a cell string")
    for key in ("mines", "obstacles", "targets", "bridges"):
        value = section[key]
        if not isinstance(value, list) or not all(isinstance(cell, str) for cell in value):
            raise ValueError(f"map.{key} must be an explicit list of cell strings")
    # 桥洞不能是空表：车端布局协议 allow_empty=False（tasks/drone.py:242）会直接拒收，
    # 静态图却印"通过"= 让操作者以为路线可走。空表在这里就是错，不是"这局没桥洞"。
    if not section["bridges"]:
        raise ValueError("map.bridges must not be empty (car layout protocol rejects empty bridges)")
    return section


def _input_failure(kind: str, path: str, exc: Exception) -> int:
    # Missing spatial evidence is UNKNOWN, never a skipped check that implies GO.
    print(f"[{kind}] {path}: {exc}", file=sys.stderr)
    print("RESULT_JSON=" + json.dumps({
        "ok": False, "launch_go": False, "status": "NO-GO",
        "route_status": "UNKNOWN", "error": kind, "path": str(path),
    }, ensure_ascii=False))
    return 1


def _layout_mode_enabled(cfg: Config, map_path: str) -> bool:
    """与 `main.py::_layout_mode_enabled` **同一定义**（工具侧镜像，防口径分叉）。

    · 地图文件存在 ⇒ False（文件模式，空间证据可校验）；
    · 地图文件不存在 ⇒ 看 `planner.auto_plan_enabled`：
      True = 布局模式（路线由 field_layout 运行期生成，静态图缺失是设计形态）；
      False = 真的缺输入（仍按 MAP_INPUT_ERROR 判红）。
    """
    if map_path and pathlib.Path(map_path).is_file():
        return False
    return bool(cfg.get("planner.auto_plan_enabled", False))


def _run() -> int:
    ap = argparse.ArgumentParser(description="dzbrain 配置 + 路线自检")
    ap.add_argument("--config", default=str(_WS / "config" / "robot.yaml"))
    ap.add_argument("--map", dest="map_path", default=str(_WS / "config" / "map.yaml"))
    ap.add_argument("--quiet", action="store_true", help="只打印问题")
    ap.add_argument("--config-only", action="store_true",
                    help="只校验配置；地图/路线记为 UNKNOWN，退出码 3（不是发车放行）")
    args = ap.parse_args()

    try:
        _yaml_mapping(args.config, "config")
        cfg = Config.load(args.config)
    except (OSError, UnicodeError, yaml.YAMLError, ConfigError, ValueError) as exc:
        return _input_failure("CONFIG_INPUT_ERROR", args.config, exc)
    cfg_issues = [i for i in cfg.validate(strict=False) if i.level != "info"]
    cfg_errors = [str(i) for i in cfg_issues if i.level == "error"]
    if cfg_errors:
        return _input_failure("CONFIG_VALIDATION_ERROR", args.config,
                              ConfigError("; ".join(cfg_errors)))
    if args.config_only:
        for issue in cfg_issues:
            print(f"   {issue}")
        warns = [i for i in cfg_issues if i.level == "warn"]
        print("-" * 72)
        print("[空间证据] 未校验（--config-only）→ route_status=UNKNOWN，launch_go=false")
        print(f"结论：配置 error 0 / warn {len(warns)} —— 仅配置可用；"
              "不是发车放行（地图/路线未校验）")
        print("RESULT_JSON=" + json.dumps({
            "ok": True, "launch_go": False, "status": "CONFIG-ONLY",
            "route_status": "UNKNOWN", "error": None, "path": str(args.config),
            "spatial_checked": False, "warn": len(warns),
        }, ensure_ascii=False))
        return 3
    # 布局模式：静态地图缺失是设计形态，不是输入错误（镜像 main.py 的 _layout_mode_enabled）。
    # 判据同 main.py：地图文件不存在 **且** planner.auto_plan_enabled=true。
    if _layout_mode_enabled(cfg, args.map_path):
        for issue in cfg_issues:
            print(f"   {issue}")
        warns = [i for i in cfg_issues if i.level == "warn"]
        print("-" * 72)
        print(f"[布局模式] 地图文件不存在：{args.map_path}")
        print("           planner.auto_plan_enabled=true ⇒ 路线由 field_layout 在运行期规划；")
        print("           静态地图缺失属设计形态，**不是**输入错误。")
        print(f"结论：配置 error 0 / warn {len(warns)} —— 仅配置可用；"
              "空间证据未校验（布局模式）→ 不是发车放行")
        print("RESULT_JSON=" + json.dumps({
            "ok": True, "launch_go": False, "status": "LAYOUT-MODE",
            "route_status": "UNKNOWN", "error": None, "path": str(args.config),
            "spatial_checked": False, "warn": len(warns), "layout_mode": True,
        }, ensure_ascii=False))
        return 3
    try:
        map_cfg = _yaml_mapping(args.map_path, "map")
        map_section = _map_evidence(map_cfg)
    except (OSError, UnicodeError, yaml.YAMLError, ValueError) as exc:
        return _input_failure("MAP_INPUT_ERROR", args.map_path, exc)

    if not args.quiet:
        print("=" * 72)
        print(f"配置文件 : {cfg.source}")
        print(f"地图文件 : {args.map_path}   图名 = {map_section.get('name')!r}")
        print("=" * 72)
        print("关键键（新版本口径）:")
        for key in CHECK_KEYS:
            print(f"  {key:34} = {cfg.get(key)!r}")
        print("底盘标定:")
        print(f"  dzactuator.ticksPerMeter            = {cfg.get('dzactuator.ticksPerMeter')!r}")
        print(f"  dzactuator.ticksPer2PI              = {cfg.get('dzactuator.ticksPer2PI')!r}")
        tokens = cfg.get("route.tokens") or []
        print(f"路线: {len(tokens)} 点 | 射击停站 {[t for t in tokens if t.endswith('*')]} | "
              f"倒车 {[t for t in tokens if t.startswith('R:')]}")
        print("-" * 72)

    empty_features = [k for k in ("mines", "obstacles", "targets") if map_section.get(k) == []]
    for key in empty_features:
        print(f"[显式空集合] map.{key} = [] —— 这是\"本局没有该特征\"的主张，不是缺省；"
              "若实际场上有，任务会漏压/漏打。")
    print(f"[配置校验] {len(cfg_issues)} 条非 info 问题：")
    for i in cfg_issues:
        print(f"   {i}")

    try:
        route = parse_mission(cfg.get("route.tokens") or [])
        issues = validate_route(route, map_cfg=map_cfg, cfg=cfg)
    except (ConfigError, ValueError) as exc:
        return _input_failure("ROUTE_INPUT_ERROR", args.config, exc)
    print("-" * 72)
    print("[路线校验]")
    print(summary(issues))

    errors = [i for i in cfg_issues if i.level == "error"] + \
             [i for i in issues if i.level == "error"]
    # ★ 配置校验的 warn 也必须进结论行 —— 只看路线 warn 会让屏幕上的
    #   "20 条非 info"与"warn 0 ✅"自相矛盾（假绿）。
    warn = [i for i in cfg_issues if i.level == "warn"] + \
           [i for i in issues if i.level == "warn"]
    print("-" * 72)
    warn_total = len(warn) + len(empty_features)
    if errors:
        verdict = "  ❌ 不可发车"
    elif warn_total:
        # G1 口径是 error 0 ∧ warn 0；有 warn 就只能是"有条件"，不能印成通过。
        verdict = "  ⚠ 有条件通过（有 warn：必须逐条判读，禁止只看退出码）"
    else:
        verdict = "  ✅ 通过"
    print(f"结论：error {len(errors)} / warn {warn_total}" + verdict)
    return 1 if errors else 0


def main() -> int:
    """门禁入口：任何未预料异常都必须变成结构化 NO-GO，绝不以 traceback 结束。

    为什么：本工具是发车前最后一道离线闸。崩溃的进程退出码虽然也是非零，
    但没有 RESULT_JSON、没有分类原因，调用方只能靠"看屏幕"判断——这正是假绿的来源。
    """
    try:
        return _run()
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001  闸门必须失败闭合
        return _input_failure("INTERNAL_ERROR", "<internal>", exc)


if __name__ == "__main__":
    raise SystemExit(main())
