# -*- coding: utf-8 -*-
r"""拐角质量参数扫描（本地、不碰车）：找"最差拐角横向偏差"最小的参数组。

背景：现场图路线在 `F11`（180° 折返）与 `F10→F8` 段实测横向峰值 0.37~0.41 m，
远高于直段的 0.04 m —— 这是「切角」类问题（旧 10-05 核心用 PIVOT + 切角修复处理）。
新版先不急着加状态机：先用仿真器把**纯参数**能做到的极限测出来。

输出：每个组合的 completed / RMS / MAX / 末点误差 / 时长 / 长度比 + 最差段。

运行：python tools\sim_param_sweep.py
"""

from __future__ import annotations

import pathlib
import sys

WS = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src" / "dzbrain"))
sys.stdout.reconfigure(errors="replace")

from dzbrain.core.config import Config                      # noqa: E402
from dzbrain.route.parser import parse_mission              # noqa: E402
from dzbrain.tools import sim as S                          # noqa: E402

ROUTE = WS / "src" / "dzbrain" / "tests" / "fixtures" / "route_field.txt"
TOKENS = [ln.strip() for ln in ROUTE.read_text(encoding="utf-8").splitlines()
          if ln.strip() and not ln.strip().startswith("#")]
route = parse_mission(TOKENS)

#: 扫描空间（只动与"切角/拐角"直接相关的键，其它保持出厂）
GRID = [
    ("出厂", {}),
    ("前视下限 0.14", {"pursuit.lookahead_min_m": 0.14}),
    ("前视下限 0.22", {"pursuit.lookahead_min_m": 0.22}),
    ("顶点钳制 0.10", {"pursuit.vertex_clamp_m": 0.10}),
    ("顶点钳制 0.10 + 前视 0.14", {"pursuit.vertex_clamp_m": 0.10,
                                   "pursuit.lookahead_min_m": 0.14}),
    ("横向加速度 0.25", {"pursuit.lat_acc_max_mps2": 0.25}),
    ("横向加速度 0.25 + 前视 0.14", {"pursuit.lat_acc_max_mps2": 0.25,
                                     "pursuit.lookahead_min_m": 0.14}),
    ("curvature_gain 0.7", {"pursuit.curvature_gain": 0.7}),
    ("curvature_gain 1.3", {"pursuit.curvature_gain": 1.3}),
]

print("=" * 100)
print("拐角质量参数扫描（现场图 34 点，出厂误差模型：零注入 + 锚定开）")
print("=" * 100)
print(f"{'组合':<28}{'完成':<5}{'RMS':>7}{'MAX':>8}{'末点':>8}{'时长':>8}{'长度比':>8}")
print("-" * 100)
best = None
for label, over in GRID:
    cfg = Config.load(str(WS / "config" / "robot.yaml"))
    for k, v in over.items():
        cfg.override(k, v)
    res = S.run_sim(route, cfg, S.SimConfig())
    m = res.metrics
    print(f"{label:<28}{('是' if m['completed'] else '否'):<5}"
          f"{m['cross_track_rms_m']:>7.4f}{m['cross_track_max_m']:>8.4f}"
          f"{m['final_error_m']:>8.4f}{m['time_s']:>8.1f}{m['path_len_ratio']:>8.3f}")
    key = (not m["completed"], round(m["cross_track_max_m"], 4), round(m["time_s"], 1))
    if best is None or key < best[0]:
        best = (key, label, m)

print("-" * 100)
print(f"★ 按『完成 → 最差横向 → 时长』排序最优: {best[1]}")
mb = best[2]
print(f"  RMS {mb['cross_track_rms_m']:.4f} / MAX {mb['cross_track_max_m']:.4f} / "
      f"末点 {mb['final_error_m']:.4f} / {mb['time_s']:.1f}s / 长度比 {mb['path_len_ratio']:.3f}")
print("\n注：横向误差物理下限约 0.10 m（最小转弯半径 0.272 m vs 90° 弯 0.35 m 段长，"
      "见 docs/新版架构设计.md §7.1）；MAX 主要由 180° 折返段 F11 与 F10→F8 贡献。")
