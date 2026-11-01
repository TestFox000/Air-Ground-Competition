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
    # ★ 2026-10-31 重钉（第七张图 / 布局模式，v4.4.0）：车端删除本地地图（`config/map.yaml`）
    #   后 `robot.yaml` 不再手写路线 —— `route.tokens` 只剩占位空表（SCHEMA 默认 `[]`），
    #   路线改由无人机 `field_layout` 运行期规划。**现场路线本体改由固定夹具承载**
    #   （`route_field7_t24.txt`，68 token，与删除前 robot.yaml 逐字一致）——
    #   与 test_planner / test_physics / test_sim / test_mission 同一口径：
    #   本节验的是「现场路线的几何」，不依赖它写在哪个文件里。
    #   （旧代码 `parse_mission(cfg.get("route.tokens") or [])` 在零 token 下必抛
    #     「Path 至少需要 2 个航点」⇒ 本节整段报异常。）
    cfg = Config.load(str(ws / "config" / "robot.yaml"))
    check("布局模式互锁（planner.auto_plan_enabled=true / map_path 空串）⇒ 现场路线取自夹具",
          cfg.get("planner.auto_plan_enabled") is True
          and (cfg.get("planner.map_path") or "") == "",
          f"auto_plan={cfg.get('planner.auto_plan_enabled')!r} "
          f"map_path={cfg.get('planner.map_path')!r}")
    fixture = ws / "src" / "dzbrain" / "tests" / "fixtures" / "route_field7_t24.txt"
    tokens = fixture.read_text(encoding="utf-8").split()
    check("现场路线夹具 route_field7_t24.txt = 68 token（第七图）",
          len(tokens) == 68, f"{len(tokens)} 个")
    field = parse_mission(tokens)
    fpath = Path(field)
    rev = field.reverse_segments()
    # ★ 2026-10-09 设计变更（实车铁证）：底盘**不支持倒车**（决定性实验：发 linear.x<0，
    #   车以 |v| 朝车头方向前进；STM32 下位机无视串口帧方向位）⇒ 路线删掉全部 R: 腿，
    #   180° 折返改为**原地 PIVOT 掉头**（corner_turn_deg=180° > 阈值 135°）。
    check("★现场图路线已无倒车段（底盘不支持倒车 → 折返改为 PIVOT）",
          len(rev) == 0, f"reverse_segments={rev}")
    # ★ 2026-10-31 重钉（第七图）：第六图靠「外圈绕行」把 180° 折返消掉（0 处折返顶点）；
    #   第七图把回折**加回来**了（H4→I4→H4 / H4→G4→H4 / A2→A1→A2），而底盘仍不能倒车
    #   ⇒ 全部用 `:PIVOT` 表达（原地掉头）。真正的不变量是**第二条**：≥135° 的顶点必须
    #   带 :PIVOT —— 不带就是让车硬拐 180°（实车会绕圈跳点，见 test_route §11 的 PIVOT 契约）。
    sharp = [i for i in range(1, fpath.segment_count) if fpath.corner_turn_deg(i) >= 135.0]
    check("★现场图路线有 3 处 180° 折返顶点（第七图 H4→I4 / H4→G4 / A2→A1 回折）",
          len(sharp) == 3 and all(abs(fpath.corner_turn_deg(i) - 180.0) < 1e-9 for i in sharp),
          f"顶点段号={sharp}；转角={[round(fpath.corner_turn_deg(i), 1) for i in sharp]}")
    unpivot = [(i, field.waypoints[i + 1].token) for i in sharp
               if not field.waypoints[i + 1].pivot]
    check("★每一处 ≥135° 折返都带 :PIVOT 声明（不能倒车 ⇒ 只能原地掉头）",
          unpivot == [], f"未声明 PIVOT 的折返={unpivot}")
    # 回折腿 = A→B→A 同格回折（几何上两段压同一条直线）——本文件的 s_hint 修复就是为它写的。
    # 第七图恰好 3 处，与上面 3 个 PIVOT 顶点一一对应；两条断言互相钉住「声明」与「几何」。
    blocks = field.blocks()
    backtrack = [i for i in range(len(blocks) - 2) if blocks[i + 2] == blocks[i]]
    check("★现场图路线存在 3 处 A→B→A 同格回折，且与 PIVOT 顶点一一对应",
          backtrack == sharp, f"同格回折={backtrack}；PIVOT 顶点段号={sharp}")
    overlap_bad = [(i, round(fpath.segments[i][4], 4), round(fpath.segments[i + 1][4], 4))
                   for i in backtrack
                   if abs(fpath.segments[i][4] - fpath.segments[i + 1][4]) >= 1e-6]
    check("★回折两段长度相等（同一条线压两条航段 ⇒ 投影歧义，必须靠 s_hint 裁决）",
          overlap_bad == [], f"{overlap_bad}")
    # ★ 修复本体在**真实第七图路线**上复验（照 §2/§2b 的同一调用式）：
    #   回折压线时两段完全共线、同一物理点到两段**等距**，位置本身无法裁决
    #   ⇒ 只能靠 `motion_heading`（实际行进方向）+ `s_hint` 连续性。回程腿必须命中返回段、
    #   s 单调增长到 `顶点 + 段长`；去程腿（反向对照）必须仍命中去程段、s 不越过顶点。
    stuck = []
    for i in backtrack:
        seg = fpath.segments[i + 1]                    # 回程段 mid -> 出发格
        ux, uy = seg[2] - seg[0], seg[3] - seg[1]
        travel = geo.heading_of(ux, uy)
        s_prev = fpath.s_at_waypoint(i + 1)            # 折返顶点弧长
        for frac in (0.15, 0.45, 0.75, 0.95):
            pr = fpath.project(seg[0] + ux * frac, seg[1] + uy * frac,
                               s_hint=s_prev, motion_heading=travel)
            expect_s = fpath.s_at_waypoint(i + 1) + frac * seg[4]
            if pr.seg_index != i + 1 or pr.s < s_prev - 1e-9 or abs(pr.s - expect_s) >= 1e-6:
                stuck.append((i, frac, pr.seg_index, round(pr.s, 4), round(expect_s, 4)))
            s_prev = pr.s
    check("★回程腿上投影命中返回段且 s 单调前进（修复本体，现场路线实测）",
          stuck == [], f"{stuck}")
    bad_fwd = []
    for i in backtrack:
        seg = fpath.segments[i]                        # 去程段 出发格 -> mid
        ux, uy = seg[2] - seg[0], seg[3] - seg[1]
        travel = geo.heading_of(ux, uy)
        apex = fpath.s_at_waypoint(i + 1)
        s_prev = fpath.s_at_waypoint(i)
        for frac in (0.15, 0.45, 0.75, 0.95):
            pr = fpath.project(seg[0] + ux * frac, seg[1] + uy * frac,
                               s_hint=s_prev, motion_heading=travel)
            if pr.seg_index != i or pr.s >= apex - 1e-9:
                bad_fwd.append((i, frac, pr.seg_index, round(pr.s, 4)))
            s_prev = pr.s
    check("★反向对照：去程腿仍命中去程段、不提前跳到返回段（不会提前掉头）",
          bad_fwd == [], f"{bad_fwd}")
except Exception as exc:      # 配置读不到 = 测试环境坏了，必须红，绝不允许静默跳过（2026-10-11 假绿教训）
    check(f"现场图路线检查失败（{type(exc).__name__}: {exc}）", False, "异常")

finish()
