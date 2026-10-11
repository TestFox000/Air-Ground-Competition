"""pursuit/lookahead.py — 前视点查找，含顶点钳制和退化保护。

从 path_executor.py V1 L895-970 find_lookahead_point 拆出：
  - 纯函数（输入 Path + Projection + 前视距离 → 输出 LookaheadPoint）
  - 弯道顶点钳制（V1 L927-944）：20°以上弯角且距顶点 <4cm → 夹紧到顶点
  - 跨段前视（V1 L946-968）：沿弧长前进直至前视距离耗尽
  - 退化保护（新增）：dist_to_vertex==0 时不做顶点钳制（防前视塌缩）

使用方式：
  point = find_lookahead_point(path, proj, lookahead_dist, ...)
  # point.x, point.y 为瞄准点坐标

设计模式：纯函数，无副作用。
"""

import math
from dataclasses import dataclass
from typing import Optional

from .path import Path, Projection


@dataclass(frozen=True)
class LookaheadPoint:
    """前视瞄准点。"""
    x: float               # 瞄准点 x 坐标
    y: float               # 瞄准点 y 坐标
    seg_index: int         # 瞄准点所在段索引
    s: float               # 瞄准点弧长
    clamped_to_vertex: bool = False   # 是否被夹紧到顶点（弯道保护）


def find_lookahead_point(
    path: Path,
    proj: Projection,
    lookahead_dist: float,
    corner_angle_rad: float = math.radians(20.0),
    clamp_margin: float = 0.04,
) -> LookaheadPoint:
    """沿路线上距投影 point 前视距离处的瞄准点（纯函数）。

    完全复制 V1 find_lookahead_point L895-970 的语义：
      1. 弯道检测（20°）→ 最后 4cm 夹紧到顶点
      2. 前视距在段内 → 段内前进
      3. 段内不足以跨段 → 循环跨段
      4. 路径总长不足 → 直指终点

    Args:
        path: 弧长路径
        proj: 当前投影点
        lookahead_dist: 前视距离（米）
        corner_angle_rad: 弯道角阈值（弧度，默认 20°）
        clamp_margin: 夹紧余量（米，默认 0.04m）

    Returns:
        LookaheadPoint 对象
    """
    # 退化保护：路径过短或无投影
    n_seg = path.num_segments
    n_wp = path._n
    if n_wp < 1:
        return LookaheadPoint(x=0.0, y=0.0, seg_index=0, s=0.0)
    if n_seg < 1:
        # 只有一个点 → 返回该点
        p = path._point(0)
        return LookaheadPoint(x=p[0], y=p[1], seg_index=0, s=0.0)

    seg_idx = proj.seg_index

    # 当前段信息
    if seg_idx < n_seg:
        seg = path.segments[seg_idx]
        seg_x0, seg_y0 = seg[0], seg[1]
        seg_x1, seg_y1 = seg[2], seg[3]
        seg_len = seg[4]  # len
    else:
        seg_idx = n_seg - 1
        seg = path.segments[seg_idx]
        seg_x0, seg_y0 = seg[0], seg[1]
        seg_x1, seg_y1 = seg[2], seg[3]
        seg_len = seg[4]

    # 段内剩余距离（从投影点到段末端）
    left = proj.dist_to_vertex  # (1.0 - t) * seg_len

    # ── 弯道检测 + 顶点夹紧 ──────────────────────────────────
    # V1 L926-944: corner_ahead detection + clamp at last 4cm
    corner_ahead = False
    if seg_idx + 1 < n_seg:  # 下一条段存在
        # 计算两段夹角（V1 L928-938）
        v1x = seg_x1 - seg_x0  # d1x
        v1y = seg_y1 - seg_y0  # d1y
        next_seg = path.segments[seg_idx + 1]
        v2x = next_seg[2] - next_seg[0]  # p3['x'] - p2['x']
        v2y = next_seg[3] - next_seg[1]  # p3['y'] - p2['y']
        cross = v1x * v2y - v1y * v2x
        dot = v1x * v2x + v1y * v2y
        ang = abs(math.atan2(cross, dot))
        if ang > corner_angle_rad:
            corner_ahead = True

    # V1 L940-944: corner + left < 0.04 → 瞄准点 = 顶点
    if corner_ahead and left < clamp_margin:
        return LookaheadPoint(
            x=seg_x1, y=seg_y1,
            seg_index=seg_idx,
            s=proj.s + left,
            clamped_to_vertex=True,
        )

    # ── 沿段前进 ────────────────────────────────────────────
    # V1 L946-952: 前视距完全在当前段内
    remaining = lookahead_dist

    if seg_len > 1e-9 and left >= remaining:
        # 从投影点沿段方向前进 remaining
        ux = (seg_x1 - seg_x0) / seg_len
        uy = (seg_y1 - seg_y0) / seg_len
        return LookaheadPoint(
            x=proj.x + ux * remaining,
            y=proj.y + uy * remaining,
            seg_index=seg_idx,
            s=proj.s + remaining,
            clamped_to_vertex=False,
        )

    remaining -= left  # 当前段全部贡献

    # ── 跨段 ────────────────────────────────────────────────
    # V1 L957-968: 逐段消耗前视预算
    j = seg_idx + 1
    while j < n_seg and remaining > 1e-9:
        seg_j = path.segments[j]
        s_jx0, s_jy0 = seg_j[0], seg_j[1]
        s_jx1, s_jy1 = seg_j[2], seg_j[3]
        s_jlen = seg_j[4]
        if s_jlen > 1e-9 and s_jlen >= remaining:
            ux = (s_jx1 - s_jx0) / s_jlen
            uy = (s_jy1 - s_jy0) / s_jlen

            # 本段弧长起点
            s_at_seg_start = path.s_at_waypoint(j)

            return LookaheadPoint(
                x=s_jx0 + ux * remaining,
                y=s_jy0 + uy * remaining,
                seg_index=j,
                s=s_at_seg_start + remaining,
                clamped_to_vertex=False,
            )
        remaining -= s_jlen
        j += 1

    # ── 路径总长不足前视距 → 直指终点 ──────────────────────
    # V1 L969-970
    last_idx = path.num_segments  # 最后一个点 = 段数（因为段从 0 到 n-1，起点为航点）
    last_point = path._point(last_idx)
    return LookaheadPoint(
        x=last_point[0], y=last_point[1],
        seg_index=last_idx - 1,
        s=path.total_length,
        clamped_to_vertex=False,
    )


def find_lookahead_point_v1_dict(
    path_points: list,
    seg: dict,
    lookahead_dist: float,
) -> dict:
    """V1 find_lookahead_point 的纯函数版本（独立于 self）。

    完全复制 V1 L895-970 逻辑，输入无状态参数，输出 V1 格式 dict。

    Args:
        path_points: V1 格式航点列表
        seg: V1 格式最近段 dict {'idx','t','px','py','dist'}
        lookahead_dist: 前视距离

    Returns:
        V1 格式 dict {'x','y','z','qx','qy','qz','qw'}
    """
    pts = path_points
    if len(pts) < 2:
        return pts[-1] if pts else None

    def _pt(x, y):
        return {'x': x, 'y': y, 'z': 0.0,
                'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}

    CORNER_ANGLE_RAD = math.radians(20.0)

    remaining = lookahead_dist
    idx = seg['idx']
    p1 = pts[idx]
    p2 = pts[idx + 1]
    seg_len = math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
    left = (1.0 - seg['t']) * seg_len if seg_len > 1e-9 else 0.0

    # Corner check
    corner_ahead = False
    if idx + 2 < len(pts):
        d1x = p2['x'] - p1['x']
        d1y = p2['y'] - p1['y']
        p3 = pts[idx + 2]
        d2x = p3['x'] - p2['x']
        d2y = p3['y'] - p2['y']
        cross = d1x * d2y - d1y * d2x
        dot = d1x * d2x + d1y * d2y
        ang = abs(math.atan2(cross, dot))
        if ang > CORNER_ANGLE_RAD:
            corner_ahead = True

    if corner_ahead and left < 0.04:
        return p2

    if seg_len > 1e-9 and left >= remaining:
        ux = (p2['x'] - p1['x']) / seg_len
        uy = (p2['y'] - p1['y']) / seg_len
        return _pt(seg['px'] + ux * remaining,
                   seg['py'] + uy * remaining)

    remaining -= left

    j = idx + 1
    while j < len(pts) - 1 and remaining > 1e-9:
        p1 = pts[j]
        p2 = pts[j + 1]
        sl = math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
        if sl > 1e-9 and sl >= remaining:
            ux = (p2['x'] - p1['x']) / sl
            uy = (p2['y'] - p1['y']) / sl
            return _pt(p1['x'] + ux * remaining,
                       p1['y'] + uy * remaining)
        remaining -= sl
        j += 1

    return pts[-1]
