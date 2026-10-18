#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read-only enumeration (t13): shoot-stop candidates on both board sides."""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.join(HERE, "..", "dzrobot_v2")
sys.path.insert(0, os.path.join(WS, "src", "dzbrain"))

import yaml  # noqa: E402

from dzbrain.core import geometry as geo  # noqa: E402
from dzbrain.core.config import Config  # noqa: E402
from dzbrain.route import planner as P  # noqa: E402


def main() -> int:
    cfg = Config.load(os.path.join(WS, "config", "robot.yaml"))
    with open(os.path.join(WS, "config", "map.yaml"), encoding="utf-8") as f:
        map_cfg = yaml.safe_load(f.read()) or {}
    spec = P.load_map(map_cfg)
    grid = P.Grid(spec, allow_reverse=False)
    ctr = P.plate_center_m(spec)
    nx, ny = P.plate_bisector_axis_m(spec)
    print("targets   :", spec.targets)
    print("obstacles :", spec.obstacles)
    print("mines     :", spec.mines)
    print("plate ctr : (%.4f, %.4f)  normal=(%.4f, %.4f)" % (ctr[0], ctr[1], nx, ny))
    print("window    :", P.shoot_distance_window())
    rows = []
    for b in P.shoot_candidates(spec, grid):
        x, y = geo.block_to_meters(b)
        off = (x - ctr[0]) * nx + (y - ctr[1]) * ny
        rows.append((b, math.hypot(x - ctr[0], y - ctr[1]), off))
    print("\ncandidates %d (ranked, bisector first):" % len(rows))
    for b, d, off in rows:
        side = "+" if off > 1e-6 else ("-" if off < -1e-6 else "0")
        print("  %-3s d=%.4f  off=%+.4f side=%s" % (b, d, off, side))
    pos = [r for r in rows if r[2] > 1e-6]
    neg = [r for r in rows if r[2] < -1e-6]
    print("\nside + : %d   best=%s" % (len(pos), pos[0][0] if pos else "none"))
    print("side - : %d   best=%s" % (len(neg), neg[0][0] if neg else "none"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
