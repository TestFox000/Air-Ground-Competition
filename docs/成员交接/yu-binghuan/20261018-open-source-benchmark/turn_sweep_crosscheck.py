# -*- coding: utf-8 -*-
"""G1 转弯扫掠对拍：规划层模型（拐点起步弧）vs 执行层几何（提前切角弧）。

背景（2026-10-18 开源吸收 G1）：
  Fields2Cover 对拍发现的口径疑点 —— 两层弧几何可能不是同一条弧：
  - 规划层 mission_router._turn_sweep_offsets：弧【从拐点格中心起步】，
    圆心 = 拐点 + R*u_in 法向侧，弧凸向【入向延长侧】（冲过拐点再转弯）；
    R = 0.17/tan(35°) = 0.243m（现场标定 35° 满舵）。
  - 执行层 path_executor.start_turn：车在距拐点 r_corner = R_TURN 处
    【提前切入】满舵弧（yaw 闭环停，lead 6°），圆心 = 拐点 - R*u_in + R*n_side，
    弧凸向【转弯内侧/来路侧】（切角绕过拐点），R = 0.17/tan(30°) = 0.294m
    （MAX_STEER_DEG=30 servo 限位）。

真值源 A（保真 import）：ag_integrate.mission_router 的
  _turn_sweep_offsets（整格墙判定）/_turn_sweep_points（亚格圆实体判定）。
真值源 B（独立复刻执行层）：切角弧采样 + 与规划层同款膨胀/距离口径。

对拍输出（56 个 8 向转向组合全扫描）：
  1. 漏检格 = 执行弧扫到但规划不查的格 —— 是否恒属「来路安全集」
     （来路格 + 对角进入的防切角正交格，路线本身已保证无实体）
  2. 错杀格 = 规划查但执行弧根本不去的格 —— 可行转向被误禁的保守面
  3. 半径口径：规划 0.243(35°) vs 执行 0.294(30°) 双口径对比
  4. 亚格实体（桶 r=0.125m / 靶 r=0.0125m）：两弧触发集差异
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

SRC = Path(r"c:\Users\29212\Documents\Trae-CN\default"
           r"\路径规划交接_20261017\src\ag_integrate")
sys.path.insert(0, str(SRC))

from ag_integrate.mission_router import (  # noqa: E402
    _DIRS, _turn_sweep_offsets, _turn_sweep_points,
    CELL_M, HALF_WIDTH_M, R_MIN_M,
)

# ---- 实车常量（对齐 path_executor 实码） --------------------------------
WHEELBASE_M = 0.17
MAX_STEER_DEG_EXEC = 30.0            # servo limit used by dztrack
R_EXEC_M = WHEELBASE_M / math.tan(math.radians(MAX_STEER_DEG_EXEC))  # 0.2944m
TURN_YAW_LEAD_DEG = 6.0              # yaw 提前停（保守对拍用 0，敏感性用 6）
BUCKET_RADIUS_M = 0.125              # 障碍桶等效圆半径（现场 2026-10-18）
TARGET_RADIUS_M = 0.0125             # 移动靶薄板等效圆半径

R_PLAN_CELLS = R_MIN_M / CELL_M      # 0.694 格（35°）
R_EXEC_CELLS = R_EXEC_M / CELL_M     # 0.841 格（30°）
HALF_W_CELLS = HALF_WIDTH_M / CELL_M  # 0.286 格


def _dir_xy(h: int) -> tuple[float, float]:
    """航向 -> 显示坐标单位向量（X=col, Y=-row，北=+Y）。

    注意 +0.0 规范化：-float(dr) 在 dr=0 时产生 -0.0，会让 180° 掉头的
    cross = -0.0 → atan2(-0.0, -1) = -π（误判右转）；执行层 start_turn
    用路径坐标差算 ang（+0.0）得 +π 左转。此处与执行层对齐。
    """
    dc, dr = _DIRS[h]
    return float(dc), -float(dr) + 0.0


def _approach_safe_cells(h_in: int) -> set[tuple[int, int]]:
    """来路安全集（相对拐点的格偏移）：路线保证无实体的格。

    - 正交进入：来路 1 格。
    - 对角进入：来路对角格 + 防切角的两个正交格（mission_router
      对角移动要求 o1/o2 非障碍，路线生成层已保证）。
    """
    dc, dr = _DIRS[h_in]
    if dc != 0 and dr != 0:
        return {(-dc, -dr), (-dc, 0), (0, -dr)}
    return {(-dc, -dr)}


def _exit_safe_cells(h_out: int) -> set[tuple[int, int]]:
    """出路安全集：拐点之后路线保证无实体的格。

    - 出向格（下一航点）恒安全；
    - 出向为对角时，防切角的两个正交侧格同样由路线生成层保证。
    """
    dc, dr = _DIRS[h_out]
    if dc != 0 and dr != 0:
        return {(dc, dr), (dc, 0), (0, dr)}
    return {(dc, dr)}


def _route_safe_cells(h_in: int, h_out: int) -> set[tuple[int, int]]:
    return _approach_safe_cells(h_in) | _exit_safe_cells(h_out)


def _cell_center_xy(dc: int, dr: int) -> tuple[float, float]:
    """格中心相对拐点的显示坐标（X=col, Y=-row）。"""
    return float(dc), -float(dr)


def exec_arc_geometry(h_in: int, h_out: int, r_cells: float,
                      lead_deg: float = 0.0):
    """执行层切角弧：返回 (圆心xy, 起始圆心角, 带符号扫描角, 转向符号)。

    几何（path_executor.start_turn/turn_update 实码复刻）：
      提前量 = r_corner = R_TURN（单格 0.35m 航段下 min(R_TURN, 0.9*0.35)=R_TURN）
      圆心   = 拐点 - R*u_in + R*n_side（n_side = 转弯侧法向）
      起点   = 入线切点 = 拐点 - R*u_in
      扫角   = |转向角| - lead（yaw-based stop；保守对拍 lead 可取 0）
      180° 特判：turn_sign = +1（左），对齐 start_turn 的 ang=π>0。
    """
    uix, uiy = _dir_xy(h_in)
    uox, uoy = _dir_xy(h_out)
    # 2026-10-18 镜像执行层修复：path_executor.start_turn 已加 cross+0.0 归一
    # （掉头一律左转）。原式 0.0*(-1)-1.0*0.0 = -0.0 → atan2(-0.0,-1)=-π
    # 会把北↔南掉头建成右转——与修复后的执行层失真。+0.0 同步归一。
    cross = uix * uoy - uiy * uox + 0.0
    dot = uix * uox + uiy * uoy
    theta = math.atan2(cross, dot)           # 带符号转向角
    if theta == 0.0:
        return None
    sign = 1.0 if theta > 0 else -1.0        # 左/右（180° 掉头恒左，对齐实码）
    # 转弯侧法向：左转取 u_in 左法向，右转取右法向
    if sign > 0:
        nsx, nsy = -uiy, uix
    else:
        nsx, nsy = uiy, -uix
    cx = -r_cells * uix + r_cells * nsx
    cy = -r_cells * uiy + r_cells * nsy
    t1 = (-r_cells * uix, -r_cells * uiy)    # 入线切点
    a0 = math.atan2(t1[1] - cy, t1[0] - cx)
    sweep = abs(theta) - math.radians(lead_deg)
    sweep = max(0.0, sweep)
    return (cx, cy, a0, sign * sweep)


def exec_sweep_offsets(h_in: int, h_out: int, r_cells: float,
                       half_w: float, lead_deg: float = 0.0,
                       n_samp: int = 256) -> set[tuple[int, int]]:
    """执行弧膨胀格集（墙判定口径与 _turn_sweep_offsets 同款）。"""
    geo = exec_arc_geometry(h_in, h_out, r_cells, lead_deg)
    if geo is None:
        return set()
    cx, cy, a0, dth = geo
    out: set[tuple[int, int]] = set()
    for i in range(n_samp + 1):
        a = a0 + dth * i / n_samp
        px = cx + r_cells * math.cos(a)
        py = cy + r_cells * math.sin(a)
        for dc in range(-3, 4):
            for dr in range(-3, 4):
                if (dc, dr) in out:
                    continue
                gy = -dr
                ddx = max(0.0, abs(px - dc) - 0.5)
                ddy = max(0.0, abs(py - gy) - 0.5)
                if ddx * ddx + ddy * ddy <= half_w * half_w:
                    out.add((dc, dr))
    return out


def exec_sweep_dists(h_in: int, h_out: int, r_cells: float,
                     lead_deg: float = 0.0,
                     n_samp: int = 256) -> dict[tuple[int, int], float]:
    """执行弧到邻近格中心的最小距离（格单位，亚格实体判定口径同款）。"""
    geo = exec_arc_geometry(h_in, h_out, r_cells, lead_deg)
    if geo is None:
        return {}
    cx, cy, a0, dth = geo
    best: dict[tuple[int, int], float] = {}
    for i in range(n_samp + 1):
        a = a0 + dth * i / n_samp
        px = cx + r_cells * math.cos(a)
        py = cy + r_cells * math.sin(a)
        for dc in range(-3, 4):
            for dr in range(-3, 4):
                gy = -dr
                d = math.hypot(px - dc, py - gy)
                if d < best.get((dc, dr), 9.9):
                    best[(dc, dr)] = d
    return best


def wall_set_plan(h_in: int, h_out: int, r_cells: float) -> set:
    return set(_turn_sweep_offsets(h_in, h_out, r_cells, HALF_W_CELLS))


def entity_hits_plan(h_in: int, h_out: int, r_cells: float,
                     r_ent_m: float) -> set:
    """规划弧触发的亚格实体格集（dist*CELL < half_w + r_ent）。"""
    thresh = (HALF_WIDTH_M + r_ent_m) / CELL_M
    return {(dc, dr) for dc, dr, d in _turn_sweep_points(h_in, h_out, r_cells)
            if d < thresh}


def entity_hits_exec(h_in: int, h_out: int, r_cells: float,
                     r_ent_m: float, lead_deg: float = 0.0) -> set:
    thresh = (HALF_WIDTH_M + r_ent_m) / CELL_M
    return {k for k, d in exec_sweep_dists(h_in, h_out, r_cells,
                                           lead_deg).items()
            if d < thresh}


def cells_str(cells) -> str:
    if not cells:
        return "-"
    return ",".join(f"{dc:+d}{dr:+d}" for dc, dr in sorted(cells))


def main() -> None:
    print(f"半径口径：规划 R={R_MIN_M:.3f}m({R_PLAN_CELLS:.3f}格, 35°)  "
          f"执行 R={R_EXEC_M:.3f}m({R_EXEC_CELLS:.3f}格, 30°)  "
          f"半宽={HALF_W_CELLS:.3f}格")
    print(f"{'入向':>4s} {'出向':>4s} | {'漏检':>16s} {'错杀':>22s}")
    print("-" * 72)

    danger_total = 0
    rows = []
    for h_in in range(8):
        for h_out in range(8):
            if h_in == h_out:
                continue
            s_plan = wall_set_plan(h_in, h_out, R_PLAN_CELLS)
            s_exec = exec_sweep_offsets(h_in, h_out, R_EXEC_CELLS,
                                        HALF_W_CELLS, lead_deg=0.0)
            safe = _route_safe_cells(h_in, h_out)
            miss = s_exec - s_plan - safe          # 真漏检（危险）
            miss_safe = (s_exec - s_plan) & safe   # 漏检但属路线安全集（无害）
            overkill = s_plan - s_exec             # 错杀（保守面损失）
            turn_deg = _turn_units_deg(h_in, h_out)
            if miss:
                danger_total += 1
            rows.append({
                "h_in": h_in, "h_out": h_out, "turn_deg": turn_deg,
                "miss_danger": sorted(miss),
                "miss_route_safe": sorted(miss_safe),
                "overkill": sorted(overkill),
                "n_exec": len(s_exec), "n_plan": len(s_plan),
            })
            if miss or overkill:
                print(f"{_DIRS[h_in]}->{_DIRS[h_out]} "
                      f"{turn_deg:4.0f}° | 漏检{cells_str(miss):>14s}"
                      f"{('(安全:' + cells_str(miss_safe) + ')') if miss_safe else '':>12s}"
                      f" 错杀{cells_str(overkill)}")

    print("-" * 72)
    print(f"组合总数 56，存在【真漏检】的组合数 = {danger_total}")
    by_deg = {}
    for r in rows:
        by_deg.setdefault(r["turn_deg"], [0, 0, 0])
        by_deg[r["turn_deg"]][0] += 1
        if r["miss_danger"]:
            by_deg[r["turn_deg"]][1] += 1
        by_deg[r["turn_deg"]][2] += len(r["overkill"])
    print("按转角统计 [组合数, 真漏检组合数, 错杀格总数]:")
    for deg in sorted(by_deg):
        n, m, ok = by_deg[deg]
        print(f"  {deg:3.0f}°: 组合{n} 真漏检{m} 错杀{ok}格")

    # ---- 实际路线核查：漏检/错杀 × 实体分布 ------------------------------
    print("\n== 实际路线核查（route_plan.json × field_state.json）==")
    route_check()

    # ---- 落盘 -----------------------------------------------------------
    out = {
        "r_plan_m": R_MIN_M, "r_exec_m": R_EXEC_M,
        "half_w_m": HALF_WIDTH_M,
        "danger_combos": danger_total,
        "combos": rows,
    }
    dst = Path(__file__).with_name("G1_转弯扫掠对拍结果.json")
    dst.write_text(json.dumps(out, ensure_ascii=False, indent=1),
                   encoding="utf-8")
    print(f"\n结果已落盘 {dst.name}")


def _parse_cell(name: str) -> tuple[int, int]:
    """格号 -> (col, row)。"""
    core = name.strip().rstrip("*,xyXY")
    return ord(core[0].upper()) - 65, int(core[1:]) - 1


def route_check() -> None:
    """对真实路线逐拐点核查：真漏检格是否撞实体 / 错杀格是否损失机会。"""
    import glob
    base = Path(r"c:\Users\29212\Documents\Trae-CN\default")
    combos = {}
    for h_in in range(8):
        for h_out in range(8):
            if h_in == h_out:
                continue
            s_plan = wall_set_plan(h_in, h_out, R_PLAN_CELLS)
            s_exec = exec_sweep_offsets(h_in, h_out, R_EXEC_CELLS,
                                        HALF_W_CELLS)
            safe = _route_safe_cells(h_in, h_out)
            combos[(h_in, h_out)] = (
                s_exec - s_plan - safe,     # 真漏检
                s_plan - s_exec,             # 错杀
            )
    files = sorted(glob.glob(str(base / "检测召回AB" / "**" / "route_plan.json"),
                             recursive=True))
    total_turns = total_hits = total_lost = 0
    for f in files[:8]:
        rf = Path(f)
        j = json.loads(rf.read_text(encoding="utf-8"))
        route = j.get("route") or j.get("cells") or j.get("route_cells")
        fs = json.loads((rf.with_name("field_state.json"))
                        .read_text(encoding="utf-8"))
        # 实体分布（墙 = 桥列非通行格；桶/靶 = 视频 obstacles/targets）
        ent = {}
        for c in fs.get("obstacles_video", []):
            ent[_parse_cell(c)] = "桶"
        for c in fs.get("targets_video", {}).get("red", []) + \
                fs.get("targets_video", {}).get("blue", []):
            ent[_parse_cell(c)] = "靶"
        bridge = {_parse_cell(c)[0] for c in fs.get("passable_video", [])}
        for cc, tag in ent.items():
            pass
        # 桥墙：桥列非通行格（不含通行格本身）
        for col in bridge:
            for rrow in range(11):
                cell = (col, rrow)
                name = f"{chr(65 + col)}{rrow + 1}"
                if name not in fs.get("passable_video", []) \
                        and cell not in ent:
                    ent.setdefault(cell, "墙")
        if not route:
            continue
        route = [str(c) for c in route]
        n_hit = n_lost = n_turns = 0
        for i in range(1, len(route) - 1):
            c0, c1, c2 = (route[i - 1], route[i], route[i + 1])
            p0, p1, p2 = _parse_cell(c0), _parse_cell(c1), _parse_cell(c2)
            u1 = (p1[0] - p0[0], p1[1] - p0[1])
            u2 = (p2[0] - p1[0], p2[1] - p1[1])
            if u1 == u2:
                continue
            h_in, h_out = _DIRS.index(u1), _DIRS.index(u2)
            miss, overkill = combos[(h_in, h_out)]
            n_turns += 1
            for dc, dr in miss:
                tgt = (p1[0] + dc, p1[1] + dr)
                if tgt in ent:
                    n_hit += 1
                    print(f"  [{rf.parent.name}] {c1} "
                          f"{_turn_units_deg(h_in, h_out):.0f}° "
                          f"真漏检撞{ent[tgt]}格 {chr(65 + tgt[0])}{tgt[1] + 1}")
            for dc, dr in overkill:
                tgt = (p1[0] + dc, p1[1] + dr)
                if tgt not in ent:
                    n_lost += 1
        total_turns += n_turns
        total_hits += n_hit
        total_lost += n_lost
        print(f"[{rf.parent.name}] 拐点{n_turns} 真漏检撞实体{n_hit} "
              f"错杀空格(机会损失){n_lost}")
    print(f"合计：拐点{total_turns} 漏检撞实体{total_hits} "
          f"错杀空格{total_lost}")


def _turn_units_deg(h1: int, h2: int) -> float:
    d = abs(h1 - h2)
    return 45.0 * min(d, 8 - d)


if __name__ == "__main__":
    main()
