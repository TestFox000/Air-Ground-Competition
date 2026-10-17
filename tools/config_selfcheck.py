# -*- coding: utf-8 -*-
r"""配置 + 路线自检（本地即可跑，不需要 ROS、不需要车）。

用途：
  1. 出发前确认 config/robot.yaml 能被新版读通、关键键来自 `/dzbrain` 段而非默认值；
  2. 确认路线与 config/map.yaml 一致（雷 8/8、障不踩、两处穿洞、U 形绕行、射击点在距离窗内）；
  3. 一眼看清"当前配置到底是哪张图、哪个标定"。

用法：
    python tools\config_selfcheck.py
    python tools\config_selfcheck.py --config config\robot.yaml --map config\map.yaml
退出码：0 = 无 error；1 = 有 error（可直接用在部署前门禁里）。
"""

from __future__ import annotations

import argparse
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

from dzbrain.core.config import Config  # noqa: E402
from dzbrain.route.parser import parse_mission  # noqa: E402
from dzbrain.route.validate import summary, validate_route  # noqa: E402

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


def main() -> int:
    ap = argparse.ArgumentParser(description="dzbrain 配置 + 路线自检")
    ap.add_argument("--config", default=str(_WS / "config" / "robot.yaml"))
    ap.add_argument("--map", dest="map_path", default=str(_WS / "config" / "map.yaml"))
    ap.add_argument("--quiet", action="store_true", help="只打印问题")
    args = ap.parse_args()

    cfg = Config.load(args.config)
    map_cfg = yaml.safe_load(open(args.map_path, encoding="utf-8")) or {}

    if not args.quiet:
        print("=" * 72)
        print(f"配置文件 : {cfg.source}")
        print(f"地图文件 : {args.map_path}   图名 = {map_cfg.get('map', {}).get('name')!r}")
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

    cfg_issues = [i for i in cfg.validate(strict=False) if i.level != "info"]
    print(f"[配置校验] {len(cfg_issues)} 条非 info 问题：")
    for i in cfg_issues:
        print(f"   {i}")

    route = parse_mission(cfg.get("route.tokens") or [])
    issues = validate_route(route, map_cfg=map_cfg, cfg=cfg)
    print("-" * 72)
    print("[路线校验]")
    print(summary(issues))

    errors = [i for i in cfg_issues if i.level == "error"] + \
             [i for i in issues if i.level == "error"]
    # ★ t7 复核修正（2026-10-17）：配置校验的 warn 也要进结论行——此前只看路线 warn，
    #   屏幕上"20 条非 info"与"warn 0 ✅"自相矛盾（假绿）。
    warn = [i for i in cfg_issues if i.level == "warn"] + \
           [i for i in issues if i.level == "warn"]
    print("-" * 72)
    print(f"结论：error {len(errors)} / warn {len(warn)}"
          + ("  ❌ 不可发车" if errors else "  ✅ 通过"))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
