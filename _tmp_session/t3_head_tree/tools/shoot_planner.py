#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
最佳射击点自主计算   shoot_planner.py
================================================================================
角色：给定靶标位置，算出"车应该停在哪一格开火"，并按优劣排序。

--------------------------------------------------------------------------------
为什么需要它
--------------------------------------------------------------------------------
地图坐标只有 0.35m 的粒度，而命中要求是 30cm±10cm —— 光靠地图对不准。
所以定位分工是：
    地图坐标  -> 把车带到靶标附近的【粗定位】
    视觉云台  -> 在 30cm±10cm 内做【精瞄】

本模块负责"粗定位"这一环：挑出满足几何约束的候选停车格，
并给它们打分排序。这样即使第一候选点因为现场情况（靶纸歪了、地面有障碍）
打不中，还能自动换到第二、第三候选点重试 —— **宁可漏打，不可卡死**。

--------------------------------------------------------------------------------
三层筛选
--------------------------------------------------------------------------------
    ① 几何层：距离约束 + 可通行约束
    ② 视线层：Bresenham 连线，剔除被障碍/挡板遮住的点
    ③ 可达层：阿克曼可达性（有没有可通的接近方向）
最后用评分函数排序：距离误差 + 云台偏转角代价 + 路径绕行代价

--------------------------------------------------------------------------------
用法
--------------------------------------------------------------------------------
    # 单靶测试
    ./shoot_planner.py --targets F1 G1 --obstacles B2 D3 A5 G6 --pose M10

    # 从配置中心读
    ./shoot_planner.py --config ~/dzrobot/config/robot.yaml --targets F1 G1

    # 只输出机器可读结果（供 path_executor 用）
    ./shoot_planner.py --targets F1 --json
================================================================================
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import Dict, List, Optional, Sequence, Set, Tuple

# =============================================================================
#  地图常量（与 path_executor.py / global_planner.py 严格一致）
# =============================================================================
COL_LETTERS = "ABCDEFGHIJKLMN"
MAP_COLS = 14
MAP_ROWS = 11
BLOCK_SIZE = 0.35                   # 格边长(m)

# 挡板：第 6 行与第 7 行之间、D~N 列封死（跨越该行界的动作被禁止）
BARRIER_ROW_HI = 6
BARRIER_ROW_LO = 7
BARRIER_COL_FROM = "D"

PASSABLE, BLOCKED = 1, 0


# =============================================================================
#  坐标工具
# =============================================================================
def parse_block(name: str) -> Tuple[int, int]:
    """'F4' -> (row=4, col=5)。兼容 R: 前缀 / * 后缀 / ,x 后缀。"""
    s = str(name).strip().upper()
    if s.startswith("R:"):
        s = s[2:]
    if s.endswith("*"):
        s = s[:-1]
    if "@" in s:
        s = s.split("@", 1)[0]
    if ":" in s:
        s = s.split(":", 1)[0]
    if "," in s:
        s = s.split(",", 1)[0]
    s = s.strip()
    if len(s) < 2 or s[0] not in COL_LETTERS or not s[1:].isdigit():
        raise ValueError("非法坐标点: %r" % name)
    row = int(s[1:])
    col = COL_LETTERS.index(s[0])
    if not (1 <= row <= MAP_ROWS):
        raise ValueError("行号越界: %r (应为 1~%d)" % (name, MAP_ROWS))
    return row, col


def block_name(row: int, col: int) -> str:
    return "%s%d" % (COL_LETTERS[col], row)


def cell_to_meters(row: int, col: int) -> Tuple[float, float]:
    """格中心 -> 米制坐标。与 path_executor.block_to_meters 完全一致。"""
    return ((10.5 - (row - 1)) * BLOCK_SIZE, (13.5 - col) * BLOCK_SIZE)


def dist_m(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    """两个格中心之间的真实欧氏距离(m)。"""
    ax, ay = cell_to_meters(*a)
    bx, by = cell_to_meters(*b)
    return math.hypot(bx - ax, by - ay)


# =============================================================================
#  地图栅格
# =============================================================================
class MapGrid:
    """14x11 栅格：障碍占格 + 挡板跨越约束。

    cell 用 [row][col] 直接索引（row 从 1 开始，多留一行方便下标与行号一致）。
    """

    def __init__(self, obstacles: Sequence[str] = (),
                 barrier_blocked: bool = True):
        # 1 = 可通行，0 = 障碍
        self.cell = [[PASSABLE] * MAP_COLS for _ in range(MAP_ROWS + 1)]
        self.barrier_col_start = COL_LETTERS.index(BARRIER_COL_FROM)
        self.barrier_blocked = barrier_blocked
        self.obstacles: Set[Tuple[int, int]] = set()

        for ob in obstacles:
            try:
                r, c = parse_block(ob)
            except ValueError:
                continue
            self.cell[r][c] = BLOCKED
            self.obstacles.add((r, c))

    # ---------- 基础 ----------
    def in_bounds(self, r: int, c: int) -> bool:
        return 1 <= r <= MAP_ROWS and 0 <= c < MAP_COLS

    def passable(self, r: int, c: int) -> bool:
        return self.in_bounds(r, c) and self.cell[r][c] == PASSABLE

    def blocks_los(self, r: int, c: int) -> bool:
        """视线阻挡：障碍占格会挡住视线。"""
        return not self.passable(r, c)

    def barrier_cross_ok(self, r_from: int, r_to: int, c: int) -> bool:
        """跨挡板检查：6<->7 行之间只有 A/B/C 列能过。"""
        if not self.barrier_blocked:
            return True
        if {r_from, r_to} == {BARRIER_ROW_HI, BARRIER_ROW_LO}:
            return c < self.barrier_col_start
        return True


# =============================================================================
#  ① 视线检查（Bresenham）
# =============================================================================
def has_line_of_sight(p1: Tuple[int, int], p2: Tuple[int, int],
                      map_grid: MapGrid) -> bool:
    """Bresenham 直线连通性检查。

    返回 True = 两点之间视线通畅（没有被障碍挡住、也没有非法穿越挡板）。

    ★ 为什么打靶必须查视线：障碍桶/挡板挡在车与靶之间时，
      云台即使转到正确角度也看不到靶（脉冲激光同理会被挡）。
      地图上"能停"不等于"能打"，这一步把"能停但不能打"的点提前剔掉。

    p1 / p2 为 (row, col) 栅格坐标，可传靶标或候选点；顺序无关。
    """
    r0, c0 = p1
    r1, c1 = p2

    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dr - dc

    r, c = r0, c0
    prev_r = r0
    # 迭代上界：格子数是有限的，加个保险防止意外的死循环
    for _ in range(MAP_ROWS * MAP_COLS * 4):
        # 端点本身不检查（候选点/靶标所在的格子允许是"靶标"这种东西，
        # 但障碍占格在筛选阶段就已经被 passable 挡掉了）
        if (r, c) != (r0, c0) and (r, c) != (r1, c1):
            if map_grid.blocks_los(r, c):
                return False
            # 跨挡板检查：相邻两个采样格如果横跨 6<->7 行，看列是否允许
            if not map_grid.barrier_cross_ok(prev_r, r, c):
                return False
        if (r, c) == (r1, c1):
            return True
        prev_r = r
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return False        # 走不到终点（理论上不会）


# =============================================================================
#  ② 候选点生成
# =============================================================================
def _ring_cells(center: Tuple[int, int], radius: int) -> List[Tuple[int, int]]:
    """取以 center 为中心、切比雪夫距离恰好 = radius 的一圈格子。"""
    cr, cc = center
    out = []
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            if max(abs(dr), abs(dc)) != radius:
                continue
            out.append((cr + dr, cc + dc))
    return out


def get_shoot_candidates(target_cells: Sequence[Tuple[int, int]],
                         map_grid: MapGrid,
                         dist_min: float = 0.20,
                         dist_max: float = 0.45,
                         scan_rings: int = 2,
                         verbose: bool = False) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
    """为每个靶标算出候选射击点池。

    返回 {靶标格: [候选格, ...]}，候选格按"离理想距离近"初步排序。

    筛选顺序（严格 -> 宽松，保证一定不会返回空）：
      ① 距离约束  dist_min <= d <= dist_max   (d = 格中心到靶心的真实欧氏距离)
      ② 可通行    候选格本身不是障碍、不出界
      ③ LOS遮档   has_line_of_sight(候选, 靶心)
      ④ 阿克曼可达 至少要有一个可通的"接近方向"，否则车开不进去
      ⑤ 兜底      若以上全被筛掉，放宽距离约束取最近的几个可通行点
                  —— 战术原则"宁可漏打，不可卡死"，绝不能因为挑不出点就不停车

    ★ 关于 0.35m 格距下只剩 4 个候选：
      格距 0.35m 时，正交相邻 = 0.35m（在 0.20~0.45 内 ✓），
      对角相邻 = 0.495m（超出 ✗），隔一格 = 0.70m（超出 ✗）。
      所以严格筛选下每个靶标正好有 4 个候选（上下左右）。
      这是几何决定的，不是 bug —— 4 个方向足够覆盖"哪边好停"的选择。
    """
    result: Dict[Tuple[int, int], List[Tuple[int, int]]] = {}

    for tgt in target_cells:
        strict: List[Tuple[int, int]] = []
        relaxed: List[Tuple[float, Tuple[int, int]]] = []

        for radius in range(1, scan_rings + 1):
            for cand in _ring_cells(tgt, radius):
                r, c = cand
                # ② 可通行（候选格本身 + 不能就是靶标自己）
                if not map_grid.passable(r, c):
                    continue
                d = dist_m(cand, tgt)
                # ③ 视线遮挡
                if not has_line_of_sight(cand, tgt, map_grid):
                    continue
                # ④ 阿克曼可达性：至少要有一个正交方向能作为接近方向
                if not _ackermann_reachable(cand, map_grid):
                    continue

                if dist_min <= d <= dist_max:
                    strict.append(cand)      # 严格满足距离约束
                else:
                    # 先记下来。若同靶标所有候选都没通过严格约束，
                    # 就用"离靶最近"的几个兜底（宁可漏打，不可卡死）。
                    relaxed.append((d, cand))

        if not strict:
            # ⑤ 兜底：放宽距离约束，取最近的几个
            relaxed.sort(key=lambda x: x[0])
            strict = [cd for _, cd in relaxed[:4]]
            if verbose and strict:
                print("    ⚠️  靶 %s：严格距离约束内无候选，"
                      "已放宽距离取最近 %d 个（宁可漏打，不可卡死）"
                      % (block_name(*tgt), len(strict)))

        # 初步排序：离理想距离越近越靠前（后面的综合评分会再重排）
        ideal = (dist_min + dist_max) / 2.0
        strict.sort(key=lambda cd: abs(dist_m(cd, tgt) - ideal))
        result[tgt] = strict

        if verbose:
            print("    靶 %s -> 候选 %d 个: %s" % (
                block_name(*tgt), len(strict),
                " ".join("%s(%.2fm)" % (block_name(*cd), dist_m(cd, tgt))
                         for cd in strict)))

    return result


def _ackermann_reachable(cand: Tuple[int, int], map_grid: MapGrid) -> bool:
    """阿克曼可达性粗筛：候选格至少要有【两个正交方向】中的一个是通的。

    为什么是"两个方向"而不是"任意一个"：
      阿克曼车不能横移，要停到某个格子必须能"开进去"。如果候选格四面
      只有一条通道，那它要么是死胡同（进去出不来），要么需要倒车掉头 ——
      在比赛里都是高风险动作。要求至少两个正交方向可通，能过滤掉这类点。
    """
    r, c = cand
    open_dirs = 0
    for (dr, dc) in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        nr, nc = r + dr, c + dc
        if map_grid.passable(nr, nc) and map_grid.barrier_cross_ok(r, nr, nc):
            open_dirs += 1
    return open_dirs >= 2


# =============================================================================
#  ③ 候选点评分
# =============================================================================
def calculate_shoot_score(candidate: Tuple[int, int],
                          target_center: Tuple[int, int],
                          current_car_pose,
                          dist_target: float = 0.30,
                          dist_sigma: float = 0.10,
                          w_distance: float = 1.0,
                          w_gimbal: float = 0.6,
                          w_detour: float = 0.5,
                          w_axis: float = 1.5,
                          map_grid: Optional[MapGrid] = None
                          ) -> Tuple[float, Dict[str, float]]:
    """给候选射击点打分。**分数越低越好。**

    三项代价：
      ① 距离误差  |d - dist_target| / dist_sigma
         越接近理想射击距离越好。太近云台视场压不住靶、太远精度差。
      ② 云台偏转角代价  |偏转角| / 90
         车停住时，从"车头朝向"转到"对准靶"要花时间。
         偏角越大，锁定越慢，还可能超出云台行程 —— 所以要惩罚。
      ③ 路径绕行代价  额外路程 / 1 格
         从当前位姿到这个候选点要绕多远。直线可达时约为 0。
      ④ ★轴向对齐代价（权重最高）
         与靶同列 = 0（车头正对靶，云台几乎不用转）
         与靶同行 = 0.8（要转约 90°）
         对角     = 2.4（转角最大，且吃 ±90° 行程）

    current_car_pose 支持三种形式：
      · (row, col)                      仅位置
      · (row, col, heading_rad)         位置 + 车头朝向(rad)
      · dict 含 row/col(或 x/y 米制) 与 yaw
    """
    cr, cc, cyaw = _normalize_pose(current_car_pose, map_grid)

    # ---- ① 距离误差 ----
    d = dist_m(candidate, target_center)
    dist_cost = abs(d - dist_target) / max(dist_sigma, 1e-6)

    # ---- ② 云台偏转角代价 ----
    tx, ty = cell_to_meters(*target_center)
    cx, cy = cell_to_meters(*candidate)
    # 车在候选点上、要看向靶标的方向（地图坐标系）
    need_yaw = math.atan2(ty - cy, tx - cx)
    yaw_err = abs(_norm_angle(need_yaw - cyaw))
    # 超过 180° 不可能，取最短转向
    if yaw_err > math.pi:
        yaw_err = 2 * math.pi - yaw_err
    gimbal_cost = math.degrees(yaw_err) / 90.0

    # ---- ③ 路径绕行代价 ----
    straight = dist_m(candidate, (cr, cc))
    # 直线穿不过去就按"要绕一圈"估，用切比雪夫距离当绕行上界
    if map_grid is not None and not has_line_of_sight(candidate, (cr, cc), map_grid):
        cheb = max(abs(candidate[0] - cr), abs(candidate[1] - cc))
        straight = max(straight, cheb * BLOCK_SIZE)
    detour_cost = straight / BLOCK_SIZE

    # ---- ④ ★轴向对齐代价（本次新增，最关键的一项）----
    #  为什么单列这一项：云台水平扫描只有 ±90°。
    #    · 射击点与靶【同一列】（车头正对靶，南北向）-> 云台几乎不用转，最优
    #    · 射击点与靶【同一行】（车头侧对靶，东西向）-> 要转约 90°，次优
    #    · 射击点与靶【对角】                        -> 转角最大，最差
    #  实测教训：原来只按"距离+云台偏角+绕行"评分，结果在 F11/G11 这张图上
    #  选出了 H11（靶的东侧同行）—— 但车是从北边沿列开过来的，停在正上方
    #  （G10）才是"转个弯就到、车头正对靶"的最优解。用户明确指出应以 G10 为准。
    tr, tc = target_center
    cr2, cc2 = candidate
    if cc2 == tc:
        axis_cost, axis_tag = 0.0, "同列(正对)"
    elif cr2 == tr:
        axis_cost, axis_tag = 0.8, "同行(侧对)"
    else:
        axis_cost, axis_tag = 2.4, "对角"

    total = (w_distance * dist_cost
             + w_gimbal * gimbal_cost
             + w_detour * detour_cost
             + w_axis * axis_cost)

    return total, {
        "distance_m": d,
        "dist_cost": dist_cost,
        "gimbal_deg": math.degrees(yaw_err),
        "gimbal_cost": gimbal_cost,
        "detour_m": straight,
        "detour_cost": detour_cost,
        "axis": axis_tag,
        "axis_cost": axis_cost,
    }


def _norm_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _normalize_pose(pose, map_grid) -> Tuple[int, int, float]:
    """把各种位姿写法统一成 (row, col, yaw_rad)。"""
    if pose is None:
        return 1, 0, 0.0
    if isinstance(pose, dict):
        if "row" in pose and "col" in pose:
            return int(pose["row"]), int(pose["col"]), float(pose.get("yaw", 0.0))
        if "x" in pose and "y" in pose:
            r, c = meters_to_cell(pose["x"], pose["y"])
            return r, c, float(pose.get("yaw", 0.0))
        return 1, 0, 0.0
    if isinstance(pose, str):
        r, c = parse_block(pose)
        return r, c, 0.0
    if isinstance(pose, (tuple, list)):
        if len(pose) >= 3:
            return int(pose[0]), int(pose[1]), float(pose[2])
        if len(pose) == 2:
            return int(pose[0]), int(pose[1]), 0.0
    return 1, 0, 0.0


def meters_to_cell(x: float, y: float) -> Tuple[int, int]:
    """米制 -> 最近格（与 block_to_meters 互逆）。"""
    row = int(round(10.5 - x / BLOCK_SIZE)) + 1
    col = int(round(13.5 - y / BLOCK_SIZE))
    row = max(1, min(MAP_ROWS, row))
    col = max(0, min(MAP_COLS - 1, col))
    return row, col


# =============================================================================
#  顶层：为多个靶标规划射击点序列
# =============================================================================
def plan_shoot_points(targets: Sequence[str],
                      map_grid: MapGrid,
                      current_pose=None,
                      dist_target: float = 0.30,
                      dist_min: float = 0.20,
                      dist_max: float = 0.45,
                      w_distance: float = 1.0,
                      w_gimbal: float = 0.6,
                      w_detour: float = 0.5,
                      keep_per_target: int = 3,
                      verbose: bool = False) -> List[dict]:
    """为每个靶标算出排序后的射击点列表。

    返回 [{target, target_name, candidates:[{cell,name,dist,score,detail}], ...}, ...]

    ★ 为什么每个靶要保留多个候选（keep_per_target）：
      "最多 3 发弹、失败自动换点"需要备胎。只算一个点的话，
      那个点因为现场原因打不中，这个靶就彻底废了。
    """
    tgt_cells = [parse_block(t) for t in targets]
    pools = get_shoot_candidates(tgt_cells, map_grid,
                                 dist_min=dist_min, dist_max=dist_max,
                                 verbose=verbose)

    out = []
    pose = current_pose
    for tgt in tgt_cells:
        cands = pools.get(tgt, [])
        scored = []
        for cd in cands:
            score, detail = calculate_shoot_score(
                cd, tgt, pose,
                dist_target=dist_target, w_distance=w_distance,
                w_gimbal=w_gimbal, w_detour=w_detour, map_grid=map_grid)
            scored.append((score, cd, detail))
        scored.sort(key=lambda x: x[0])

        keep = scored[:keep_per_target] if scored else []
        out.append({
            "target": tgt,
            "target_name": block_name(*tgt),
            "candidates": [{
                "cell": cd,
                "name": block_name(*cd),
                "dist": round(detail["distance_m"], 3),
                "score": round(sc, 3),
                "detail": {k: (round(v, 3) if isinstance(v, float) else v)
                           for k, v in detail.items()},
            } for sc, cd, detail in keep],
        })

        if verbose:
            print("  靶 %s 最佳射击点:" % block_name(*tgt))
            for i, (sc, cd, dt) in enumerate(keep):
                print("    #%d %-4s  d=%.2fm  云台偏角=%+.0fdeg  绕行=%.2fm  评分=%.2f"
                      % (i + 1, block_name(*cd), dt["distance_m"],
                         dt["gimbal_deg"], dt["detour_m"], sc))
            if not keep:
                print("    ❌ 无可用候选点")

    return out


# =============================================================================
#  CLI
# =============================================================================
def load_config_map(path: str) -> dict:
    """从配置中心读取地图要素与射击参数。"""
    import yaml
    with open(path, encoding="utf-8") as fp:
        d = yaml.safe_load(fp) or {}
    m = d.get("map") or {}
    # ★ 地图要素优先从独立的 config/map.yaml 读
    _mf = os.path.join(os.path.dirname(os.path.abspath(path)), "map.yaml")
    if not m and os.path.exists(_mf):
        try:
            with open(_mf, encoding="utf-8") as _fp:
                m = (yaml.safe_load(_fp) or {}).get("map") or {}
        except Exception:
            m = {}
    p = (d.get("path_executor") or {}).get("ros__parameters", {}) or {}
    g = (d.get("dzrknn_dataprocess") or {}).get("ros__parameters", {}) or {}
    return {
        "obstacles": list(m.get("obstacles") or []),
        "targets": list(m.get("targets") or []),
        "start": m.get("start") or p.get("mission_reset", "M10,y").split(",")[0],
        "shoot_distance_target": float(p.get("shoot_distance_target", 0.30)),
        "shoot_distance_min": float(p.get("shoot_distance_min", 0.20)),
        "shoot_distance_max": float(p.get("shoot_distance_max", 0.45)),
        "shoot_score_w_gimbal": float(p.get("shoot_score_w_gimbal", 0.6)),
        "shoot_score_w_detour": float(p.get("shoot_score_w_detour", 0.5)),
        "keep_per_target": int(p.get("shoot_candidates_keep", 3)),
        "fire_only_blue": bool(g.get("fire_only_blue", True)),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="最佳射击点自主计算（栅格候选 + 视线检查 + 综合评分）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config",
                    default=os.path.expanduser("~/dzrobot/config/robot.yaml"))
    ap.add_argument("--targets", nargs="*", default=[], help="靶标，如 F1 G1")
    ap.add_argument("--obstacles", nargs="*", default=[], help="障碍，如 B2 D3")
    ap.add_argument("--pose", default=None,
                    help="车当前位置：格名(M10) 或 'row,col,yaw_deg'")
    ap.add_argument("--dist-target", type=float, default=None, help="理想射击距离(m)")
    ap.add_argument("--dist-min", type=float, default=None)
    ap.add_argument("--dist-max", type=float, default=None)
    ap.add_argument("--keep", type=int, default=None, help="每个靶保留几个候选")
    ap.add_argument("--json", action="store_true", help="只输出 JSON")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    cfg = {}
    if os.path.exists(args.config):
        try:
            cfg = load_config_map(args.config)
        except Exception as exc:
            print("  ⚠️  读配置失败: %s" % exc, file=sys.stderr)

    targets = args.targets or cfg.get("targets") or []
    obstacles = args.obstacles or cfg.get("obstacles") or []
    if not targets:
        print("❌ 未指定靶标（--targets，或配置中心 map.targets）", file=sys.stderr)
        return 2

    d_target = args.dist_target if args.dist_target is not None \
        else cfg.get("shoot_distance_target", 0.30)
    d_min = args.dist_min if args.dist_min is not None \
        else cfg.get("shoot_distance_min", 0.20)
    d_max = args.dist_max if args.dist_max is not None \
        else cfg.get("shoot_distance_max", 0.45)
    keep = args.keep if args.keep is not None else cfg.get("keep_per_target", 3)

    grid = MapGrid(obstacles)
    pose = None
    if args.pose:
        if "," in args.pose:
            parts = [float(x) for x in args.pose.split(",")]
            pose = (int(parts[0]), int(parts[1]),
                    math.radians(parts[2]) if len(parts) > 2 else 0.0)
        else:
            pose = args.pose
    elif cfg.get("start"):
        pose = cfg["start"]

    if not args.json:
        print("=" * 74)
        print("  最佳射击点自主计算")
        print("=" * 74)
        print("  靶标     : %s" % " ".join(targets))
        print("  障碍     : %s" % (" ".join(obstacles) if obstacles else "(无)"))
        print("  当前位姿 : %s" % (pose if pose else "(未指定)"))
        print("  距离约束 : %.2f ~ %.2f m（理想 %.2f m）" % (d_min, d_max, d_target))
        print("  格距     : %.2f m（正交相邻=%.2f，对角=%.2f）"
              % (BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE * math.sqrt(2)))
        print()

    plans = plan_shoot_points(targets, grid, current_pose=pose,
                              dist_target=d_target, dist_min=d_min,
                              dist_max=d_max, keep_per_target=keep,
                              verbose=not args.quiet and not args.json)

    if args.json:
        print(json.dumps([{
            "target": p["target_name"],
            "candidates": [{"name": c["name"], "dist": c["dist"],
                            "score": c["score"]} for c in p["candidates"]],
        } for p in plans], ensure_ascii=False))
        return 0

    print()
    print("-" * 74)
    print("  下发建议")
    print("-" * 74)
    for p in plans:
        if not p["candidates"]:
            print("  靶 %s : ❌ 无可用射击点" % p["target_name"])
            continue
        best = p["candidates"][0]
        chain = " -> ".join(c["name"] for c in p["candidates"])
        print("  靶 %-4s : 首选 %s (%.2fm)   备选链 %s"
              % (p["target_name"], best["name"], best["dist"], chain))
    print()
    print("  路线写法（射击点用 :SHOOT_STOP 标签）:")
    for p in plans:
        if p["candidates"]:
            print("     %s:SHOOT_STOP" % p["candidates"][0]["name"])
    print()
    print("  或直接用自动模式（让 path_executor 自己挑）:")
    for p in plans:
        print("     SHOOT@%s" % p["target_name"])
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
