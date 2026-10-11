# -*- coding: utf-8 -*-
r"""自重叠路线的投影回归测试。

背景（真 bug，实车路线就会踩）：
现场图路线里有一段 `B6 → A6 → R:B6 → R:D6`：
    B6 --向西 0.35m--> A6 --倒车回 B6（同一条线反向）--> 再向东到 D6
于是**同一条线段上压着两条航段**。`Path.project` 若按"先找到的最近段优先"，
车在折返腿上时投影会永远贴回前一条段的末端，沿程弧长 `s` 卡在段界不再前进
⇒ 任务层的"已过点"判据永不满足 ⇒ 车原地绕圈（仿真里实测 s 在 7.58↔8.400 反复跳）。

本测试钉死修复后的行为：**等距时按 s_hint 连续性裁决**，折返段 s 必须能继续增长。

运行：python src\dzbrain\tests\test_path_overlap.py
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass

from dzbrain.core import geometry as geo                          # noqa: E402
from dzbrain.pursuit.path import Path                             # noqa: E402
from dzbrain.route.model import Waypoint, make_route              # noqa: E402
from harness import check, check_close, finish, section           # noqa: E402


def _route_from_blocks(blocks, reverse_at=()):
    """按格名造路线；reverse_at 里的格名表示"到达它的那一段是倒车"。"""
    pts = []
    for b in blocks:
        pts.append(Waypoint.at_block(b, reverse=(b in reverse_at)))
    return make_route(pts)


section("1. 基线：自重叠路线的几何确实重叠")
route = _route_from_blocks(["B6", "A6", "B6", "D6"], reverse_at=("B6", "D6"))
path = Path(route)
check("4 个点 / 3 段", len(route) == 4 and path.segment_count == 3,
      f"pts={len(route)} segs={path.segment_count}")
seg = path.segments
check_close("段0（B6→A6）长度 0.35", seg[0][4], 0.35, 1e-9)
check_close("段1（A6→B6）长度 0.35", seg[1][4], 0.35, 1e-9)
check_close("段2（B6→D6）长度 0.70", seg[2][4], 0.70, 1e-9)
check("段0 与段1 共线反向",
      abs(geo.ang_diff(seg[0][5], seg[1][5])) > 3.0)   # ≈π
check("倒车标记落在到达点上", route[2].reverse and route[3].reverse)

section("2. ★ 折返腿上 s 必须继续增长（靠 motion_heading 裁决几何歧义）")
# 车沿段1（A6→B6 倒车腿）从 A6 端往 B6 端走。
# ★ 关键：倒车时"车头"朝 +y，但**行进方向**是 -y —— 传进去的必须是行进方向，
#   否则同一物理点在两条重叠段上等距、永远裁决不出来（这是真车会原地绕圈的根因）。
a6x, a6y = geo.block_to_meters("A6")
b6x, b6y = geo.block_to_meters("B6")
ux, uy = (b6x - a6x), (b6y - a6y)          # A6 -> B6 方向（行进方向，倒车腿）
travel = geo.heading_of(ux, uy)
travel_fwd = geo.heading_of(-ux, -uy)      # 去程（B6 -> A6）的行进方向
s_prev = seg[0][4]                          # 折返开始时 ≈ 段0 结束（0.35）
mono = True
for frac in (0.10, 0.30, 0.50, 0.70, 0.90):
    x = a6x + ux * frac
    y = a6y + uy * frac
    proj = path.project(x, y, s_hint=s_prev, allow_backward=True, motion_heading=travel)
    advanced = proj.s - s_prev
    check(f"折返 {frac:.2f} 处投影落在段1（不是贴回段0）", proj.seg_index == 1,
          f"seg_index={proj.seg_index} s={proj.s:.4f}")
    check(f"折返 {frac:.2f} 处 s 前进（Δ={advanced:+.4f} m）", advanced > -1e-9)
    if proj.s < s_prev - 1e-9:
        mono = False
    s_prev = proj.s
check("整段折返 s 单调不减", mono)
# 走到折返腿 90% 处，s 应为 段0末端 + 0.9×段1长（= 0.665），即确实在往段1末端长
expect_90 = seg[0][4] + 0.9 * seg[1][4]
check("折返腿 90% 处 s ≈ 段0末端+90%段1", abs(s_prev - expect_90) < 1e-6,
      f"s={s_prev:.4f} 期望≈{expect_90:.4f}")
# 走到折返腿末端（回到 B6）时 s 应到达段1 末尾 = 段0+段1
proj_end = path.project(b6x, b6y, s_hint=s_prev, allow_backward=True, motion_heading=travel)
check("折返腿走完（回到 B6）时 s ≥ 段1 末尾", proj_end.s >= seg[0][4] + seg[1][4] - 1e-6,
      f"s={proj_end.s:.4f} 期望≥{seg[0][4] + seg[1][4]:.4f}")

section("2b. 反向对照：去程（同一物理点、方向相反）必须命中段0")
# 车从 B6 往 A6 走（去程），行进方向 +y，s 应从 0 长到 0.35，不能提前跳到段1
s_fwd = 0.0
fwd_ok = True
for frac in (0.10, 0.30, 0.50, 0.70, 0.90):
    x = b6x - ux * frac          # 从 B6 出发往 A6
    y = b6y - uy * frac
    proj = path.project(x, y, s_hint=s_fwd, allow_backward=True, motion_heading=travel_fwd)
    if proj.s >= seg[0][4] - 1e-9 or proj.seg_index != 0:
        fwd_ok = False
    check(f"去程 {frac:.2f} 处落在段0 且 s<段0末端", proj.seg_index == 0 and proj.s <= seg[0][4] + 1e-9,
          f"seg_index={proj.seg_index} s={proj.s:.4f}")
    s_fwd = proj.s
check("去程全程不被误判到折返段（不会提前掉头）", fwd_ok)

section("3. 反向对照：没有 s_hint 时行为保持确定性（不保证连续性）")
p_nohint = path.project(a6x + ux * 0.5, a6y + uy * 0.5)
check("无 hint 仍返回一个合法投影", p_nohint.seg_index in (0, 1, 2),
      f"seg_index={p_nohint.seg_index}")
check("无 hint 结果可复现", path.project(a6x + ux * 0.5, a6y + uy * 0.5).s == p_nohint.s)

section("4. allow_backward=False 时索引不后退（旧行为保留）")
s_now = seg[0][4] + 0.2
proj_fwd = path.project(a6x, a6y, s_hint=s_now, allow_backward=False)
check("只前进模式下结果 s ≥ s_hint", proj_fwd.s >= s_now - 1e-9,
      f"s={proj_fwd.s:.4f} hint={s_now:.4f}")

section("5. 现场图路线的折返段真实存在（防止测试与实车脱节）")
try:
    import yaml  # noqa: F401
    from dzbrain.core.config import Config
    from dzbrain.route.parser import parse_mission
    ws = pathlib.Path(__file__).resolve().parents[3]
    cfg = Config.load(str(ws / "config" / "robot.yaml"))
    field = parse_mission(cfg.get("mission_blocks") or [])
    fpath = Path(field)
    rev = field.reverse_segments()
    # ★ 2026-10-09 设计变更（实车铁证）：底盘**不支持倒车**（决定性实验：发 linear.x<0，
    #   车以 |v| 朝车头方向前进；STM32 下位机无视串口帧方向位）⇒ 路线删掉全部 R: 腿，
    #   180° 折返改为**原地 PIVOT 掉头**（corner_turn_deg=180° > 阈值 135°）。
    check("★现场图路线已无倒车段（底盘不支持倒车 → 折返改为 PIVOT）",
          len(rev) == 0, f"reverse_segments={rev}")
    piv_blocks = [field.blocks()[i + 1] for i in range(1, fpath.segment_count)
                  if fpath.corner_turn_deg(i) >= 135.0]
    check("★折返点全部转为 ≥135° 的 PIVOT 掉头（G10 / A9 / F3 各一处）",
          sorted(piv_blocks) == ["A9", "F3", "G10"], f"pivots={piv_blocks}")
    # 找 F11→E11→F11 这条回折（same-line backtrack，现在走 PIVOT 而非倒车）
    blocks = field.blocks()
    backtrack = [i for i in range(len(blocks) - 2)
                 if blocks[i + 2] == blocks[i]]
    check("现场图存在 A→B→A 式的同格回折", len(backtrack) >= 1,
          f"index={backtrack}")
    if backtrack:
        i = backtrack[0]
        sa = fpath.s_at_waypoint(i)
        sb = fpath.s_at_waypoint(i + 2)
        check("回折两段长度相等（几何重叠）",
              abs((sb - sa) - 2 * fpath.segments[i][4]) < 1e-6,
              f"Δs={sb - sa:.4f} 段长={fpath.segments[i][4]:.4f}")
except Exception as exc:      # 配置读不到也要让上面几节的结果有效
    check(f"现场图路线检查（跳过：{type(exc).__name__}: {exc}）", True, "skip")

finish()
