# -*- coding: utf-8 -*-
r"""坐标系回归测试 —— 保证新旧实现坐标完全一致（错了全盘错位，必须先钉死）。

运行：python src\dzbrain\tests\test_geometry.py
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from dzbrain.core import geometry as geo          # noqa: E402
from harness import check, check_close, check_raises, finish, section  # noqa: E402

# --------------------------------------------------------------------------- #
section("坐标锚点（与旧 block_to_meters 逐字一致）")
# 旧代码注释里的三个基准点：N11 中心 (0.175, 0.175)、A1 中心 (3.675, 4.725)、起点 N10 = 格 (1.5, 0.5)
x, y = geo.block_to_meters("N11")
check_close("N11 中心 x", x, 0.175)
check_close("N11 中心 y", y, 0.175)
x, y = geo.block_to_meters("A1")
check_close("A1 中心 x", x, 3.675)
check_close("A1 中心 y", y, 4.725)
x, y = geo.block_to_meters("N10")
check_close("N10 中心 x", x, 0.525)
check_close("N10 中心 y", y, 0.175)
u, v = geo.block_to_units("N10")
check_close("N10 格单位 u", u, 1.5, 1e-12)
check_close("N10 格单位 v", v, 0.5, 1e-12)
check_close("格单位 ↔ 米制往返", geo.units_to_meters(*geo.meters_to_units(0.525, 0.175))[0], 0.525, 1e-12)
check_close("格边 BLOCK_SIZE", geo.BLOCK_SIZE, 0.35)

section("格名 ↔ 索引 ↔ 米制 双向一致（全 154 格）")
bad = []
for col_idx in range(geo.MAP_COLS):
    for row_idx in range(geo.MAP_ROWS):
        block = geo.index_to_block(row_idx, col_idx)
        mx, my = geo.block_to_meters(block)
        if geo.meters_to_block(mx, my) != block:
            bad.append(block)
check("154 格 米制→格名 往返全对", not bad, f"失败 {bad[:5]}")
check_raises("非法列字母 Z3", ValueError, geo.block_to_index, "Z3")
check_raises("非法行号 K99", ValueError, geo.block_to_index, "K99")
check_raises("空串", ValueError, geo.block_to_index, "")
check("小写也可解析 k9", geo.block_to_index("k9") == geo.block_to_index("K9"))

section("角度工具")
check_close("normalize_angle(3π/2)", geo.normalize_angle(3 * math.pi / 2), -math.pi / 2)
check_close("normalize_angle(-3π/2)", geo.normalize_angle(-3 * math.pi / 2), math.pi / 2)
check_close("ang_diff(10°, 350°)", geo.ang_diff(math.radians(10), math.radians(350)),
            math.radians(20), 1e-12)
check_close("heading_token_to_yaw('y')", geo.heading_token_to_yaw("y"), math.pi / 2)
check_close("heading_token_to_yaw('x')", geo.heading_token_to_yaw("x"), 0.0)
check_close("heading_token_to_yaw('Y')", geo.heading_token_to_yaw("Y"), math.pi / 2)
check_close("heading_of 朝 +y", geo.heading_of(0.0, 1.0), math.pi / 2)

section("四元数")
qx, qy, qz, qw = geo.yaw_to_quaternion(math.radians(90))
check_close("yaw 90 → qz", qz, math.sin(math.pi / 4))
check_close("yaw 90 → qw", qw, math.cos(math.pi / 4))
back = geo.quaternion_to_yaw(qx, qy, qz, qw)
check_close("四元数往返 yaw", back, math.pi / 2, 1e-12)

section("地图边界与越界判定")
x_min, x_max, y_min, y_max = geo.map_bounds()
check("无余量边界", (x_min, x_max, y_min, y_max) == (0.175, 3.675, 0.175, 4.725))
check("N11 在界内", geo.in_bounds(*geo.block_to_meters("N11")))
check("地图外 +∞ 不在界内", not geo.in_bounds(99.0, 99.0))
check("余量 0.6 后 (-0.4, 0.0) 在界内", geo.in_bounds(-0.4, 0.0, margin=0.6))
check("余量 0.6 后 (-0.5, 0.0) 出界", not geo.in_bounds(-0.5, 0.0, margin=0.6))

section("Pose")
p = geo.Pose(0.525, 0.175, math.pi / 2)
check("Pose.block() = N10", p.block() == "N10")
dx, dy, dist = p.offset_to(0.875, 0.175)
check_close("offset_to dx", dx, 0.35)
check_close("offset_to 距离", dist, 0.35)
check_close("translated 保持 yaw", p.translated(0.1, 0.2).yaw, math.pi / 2)
check_close("rotated 折绕", geo.Pose(0, 0, geo.normalize_angle(math.pi - 0.1)).rotated(0.2).yaw,
            geo.normalize_angle(math.pi + 0.1), 1e-12)
check("Pose 不可变（frozen）", p.__dataclass_params__.frozen)

section("K 列整列（桥洞保护的格号来源）")
k_blocks = [f"K{r}" for r in range(1, 12)]
check("K1~K11 全部解析成功", all(geo.block_to_index(b)[1] == geo.COL_LETTERS.index("K")
                                 for b in k_blocks))
check("K 列 x 随行号单调", all(geo.block_to_meters(f"K{r}")[0] > geo.block_to_meters(f"K{r+1}")[0]
                              for r in range(1, 11)))

finish()
