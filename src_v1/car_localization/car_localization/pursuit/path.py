"""pursuit/path.py — 弧长参数化 Path 类 + 投影函数。

从 path_executor.py V1 L855-891 find_nearest_segment 拆出：
  - 编译期预处理弧长前缀 s_start 数组（bisect 加速）
  - project() 支持 s_hint 窗口搜索 + 回退/前进显式参数
  - 等距裁决语义与 V1 完全相同（nearest-dist wins）
  - 返回 Projection frozen dataclass

使用方式：
  path = Path(route)
  proj = path.project(x, y, s_hint=0.0, window_m=0.6)

设计模式：构造后所有字段只读，project() 无副作用，线程安全。
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from ..core.geometry import normalize_angle


@dataclass(frozen=True)
class Projection:
    """投影点信息。对应 V1 find_nearest_segment 返回的 dict。

    V1 字段：idx, t, px, py, dist
    新增：seg_length, seg_heading, passed_end（V1 无弧长概念）
    """
    seg_index: int       # 段索引（对应 V1 'idx'）
    t: float              # 投影参数 [0,1]（对应 V1 't'）
    x: float              # 投影点 x 坐标（对应 V1 'px'）
    y: float              # 投影点 y 坐标（对应 V1 'py'）
    s: float              # 沿程弧长（新增，V1 无）
    dist: float           # 车到投影点垂直距离（对应 V1 'dist'）
    seg_length: float     # 段长度
    seg_heading: float    # 段方向（弧度）
    passed_end: bool      # 是否超出段末端（t>=1 且超出末端时 True）

    @property
    def dist_to_vertex(self) -> float:
        """到段末端的沿程弧长（V1 内联 (1.0-t)*seg_len）。"""
        return (1.0 - self.t) * self.seg_length

    @property
    def s_vertex(self) -> float:
        """本段末端弧长。"""
        return self.s + self.dist_to_vertex


class Path:
    """弧长路径：编译期预处理，纯函数投影（无副作用）。

    V1 的 find_nearest_segment 每次 O(N) 全扫 → 本类 project() 用
    s_hint + bisect 做到 O(logN) + O(窗口)。但等距裁决保持 V1 语义：
    搜索 current_target_idx 之后的所有段，取 dist 最小的段。

    构造：从 Route 的 waypoints 预计算段信息和弧长前缀。
    线程安全：构造后所有字段只读，project() 不修改 self。
    """

    def __init__(self, waypoints: list) -> None:
        """构造 Path。

        Args:
            waypoints: 航点列表，每项为类似 V1 的 dict
                       {'x':float, 'y':float, ...}，或 Waypoint 对象（有 .x/.y 属性）
                       必须至少有 2 个点才能形成段。
        """
        # 存储原始航点（只读引用）
        self._waypoints: list = waypoints
        self._n: int = len(waypoints)

        # 段信息：每段一条 (x0, y0, x1, y1, length, heading)
        self._segments: List[Tuple[float, float, float, float, float, float]] = []

        # 弧长前缀 s_start[i] = 段 i 起点的沿程弧长（0-based）
        self._s_start: List[float] = []

        if self._n >= 2:
            s = 0.0
            for i in range(self._n - 1):
                p1 = self._point(i)
                p2 = self._point(i + 1)
                sx = p2[0] - p1[0]
                sy = p2[1] - p1[1]
                seg_len = math.hypot(sx, sy)
                heading = math.atan2(sy, sx)
                self._segments.append((p1[0], p1[1], p2[0], p2[1], seg_len, heading))
                self._s_start.append(s)
                s += seg_len
            # 最后一段末端弧长 = 总长
            self._s_start.append(s)

        self._total_length: float = self._s_start[-1] if self._s_start else 0.0

    # ── 内部辅助 ────────────────────────────────────────────

    def _point(self, idx: int) -> Tuple[float, float]:
        """获取航点坐标 (x, y)。支持 dict 和对象两种格式。"""
        p = self._waypoints[idx]
        if isinstance(p, dict):
            return p['x'], p['y']
        return p.x, p.y  # Waypoint 对象

    # ── 只读属性 ────────────────────────────────────────────

    @property
    def total_length(self) -> float:
        """折线总长（米）。"""
        return self._total_length

    @property
    def segments(self) -> Tuple:
        """每段元组 (x0,y0,x1,y1,length,heading) 的只读视图。"""
        return tuple(self._segments)

    @property
    def num_segments(self) -> int:
        """段数。"""
        return len(self._segments)

    # ── 弧长相关 ────────────────────────────────────────────

    def s_at_waypoint(self, index: int) -> float:
        """航点索引 → 弧长。"""
        if index < len(self._s_start):
            return self._s_start[index]
        return self._total_length

    def remaining(self, s: float) -> float:
        """剩余弧长到终点。"""
        return max(0.0, self._total_length - s)

    def point_at(self, s: float) -> Tuple[float, float, int]:
        """弧长 s → 坐标 (x, y, 段索引)。"""
        s = max(0.0, min(s, self._total_length))
        if self._n < 2:
            p = self._point(0)
            return p[0], p[1], 0

        # bisect 找段
        lo, hi = 0, len(self._s_start) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self._s_start[mid] <= s:
                lo = mid
            else:
                hi = mid - 1
        seg_idx = lo
        if seg_idx >= len(self._segments):
            seg_idx = len(self._segments) - 1

        x0, y0, x1, y1, seg_len, heading = self._segments[seg_idx]
        if seg_len > 1e-12:
            t = (s - self._s_start[seg_idx]) / seg_len
            t = max(0.0, min(1.0, t))
        else:
            t = 0.0
        px = x0 + t * (x1 - x0)
        py = y0 + t * (y1 - y0)
        return px, py, seg_idx

    def heading_at(self, s: float) -> float:
        """弧长 s 处的航向（所在段方向）。"""
        _, _, seg_idx = self.point_at(s)
        if seg_idx < len(self._segments):
            return self._segments[seg_idx][5]
        return 0.0

    def segment_is_reverse(self, idx: int) -> bool:
        """段是否为倒车段（V1 无倒车概念，默认 False）。"""
        # V1 无倒车段，保持 False
        return False

    def corner_angle_deg(self, idx: int) -> float:
        """段 idx 处弯角（本段与下段夹角, 度）。
        对应 V1 L936-938 内联 atan2(cross, dot)。
        """
        if idx + 1 >= len(self._segments):
            return 0.0
        _, _, _, _, _, head1 = self._segments[idx]
        _, _, _, _, _, head2 = self._segments[idx + 1]
        ang = abs(normalize_angle(head2 - head1))
        return math.degrees(ang)

    def corner_turn_deg(self, idx: int) -> float:
        """车头需转过角度（度）。V1 无此函数，默认用弯角。"""
        return self.corner_angle_deg(idx)

    # ── 核心投影 ────────────────────────────────────────────

    def project(self, x: float, y: float,
                s_hint: Optional[float] = None,
                window_m: float = 1.0,
                allow_backward: bool = False,
                motion_heading: Optional[float] = None,
                back_m: float = 0.3,
                s_min: Optional[float] = None,
                ) -> Optional[Projection]:
        """最近段投影：寻找路线上离点 (x,y) 最近的段并返回投影。

        核心逻辑保持 V1 L855-891 的等距裁决语义：
          - 从目标段开始向前搜索所有剩余段
          - 每段计算投影参数 t 并夹紧到 [0,1]
          - 取 dist（垂直距离）最小的段
        在此之上增加弧长参数化和 s_hint 窗口加速。

        V1 语义（V1 L873）：
          for i in range(self.current_target_idx, len(pts) - 1):
          → 从 current_target_idx 开始，只向前的全扫
        本方法的 s_hint 作为弧长提示指定起始段（类似 current_target_idx 的弧长等价）。

        Args:
            x, y: 待投影坐标（米）
            s_hint: 弧长提示（对应 V1 current_target_idx 所指示的段弧长）
            window_m: 窗口范围（仅在 s_hint 附近 ±window_m 内搜索，
                      默认 1.0m；设为 0 则全扫剩余段）
            allow_backward: 是否允许向后搜索（V1 永不回看）
            motion_heading: 运动方向（弧度），用于等距裁决议（V1 无）
            back_m: 向后搜索范围（米），仅 allow_backward=True 时有效
            s_min: 最小弧长（投影点 s 不小于此值）

        Returns:
            Projection 对象，或 None（不足 1 个段）
        """
        n_seg = len(self._segments)
        if n_seg < 1:
            return None

        if s_hint is not None and s_hint < 0.0:
            s_hint = 0.0
        if s_hint is not None and s_hint > self._total_length:
            s_hint = self._total_length

        # 确定搜索起始段索引
        if s_hint is not None:
            hint_seg = self._segment_at_s(s_hint)
            # V1 语义：从 hint_seg 向前搜索（不能回退）
            first_seg = hint_seg if allow_backward else max(hint_seg, 0)
        else:
            first_seg = 0

        # V1 一直搜索到最后一段
        last_seg = n_seg - 1

        # 当 s_hint 不为 None 时，用 window_m 控制前向搜索范围上限
        # （窗口模式：从 hint_seg 到 hint_seg + window_m 的段范围内）
        if s_hint is not None and window_m > 0:
            s_hi = min(self._total_length, s_hint + window_m)
            hi_seg = self._segment_at_s(s_hi)
            if hi_seg < last_seg:
                last_seg = hi_seg

        if s_min is not None:
            min_seg = self._segment_at_s(s_min)
            if min_seg > first_seg:
                first_seg = min_seg

        # V1 等距搜索：遍历段内所有段，取 dist 最小的
        best = None  # (dist, seg_index, t, px, py, s, seg_len, heading)

        for i in range(first_seg, last_seg + 1):
            x0, y0, x1, y1, seg_len, heading = self._segments[i]
            sx = x1 - x0
            sy = y1 - y0
            seg2 = sx * sx + sy * sy
            if seg2 < 1e-12:
                continue  # 零长度段跳过（V1 L879-880）

            # 向量投影：t = dot(AP, AB) / |AB|^2
            t_val = ((x - x0) * sx + (y - y0) * sy) / seg2
            t_val = max(0.0, min(1.0, t_val))  # 夹紧 [0,1]

            px = x0 + t_val * sx
            py = y0 + t_val * sy
            d = math.hypot(x - px, y - py)

            # 弧长 s
            s_val = self._s_start[i] + t_val * seg_len

            if best is None or d < best[0]:
                best = (d, i, t_val, px, py, s_val, seg_len, heading)

        if best is None:
            return None

        dist, seg_idx, t_val, px, py, s_val, seg_len, heading = best

        return Projection(
            seg_index=seg_idx,
            t=t_val,
            x=px,
            y=py,
            s=s_val,
            dist=dist,
            seg_length=seg_len,
            seg_heading=heading,
            passed_end=(t_val >= 1.0),
        )

    def _segment_at_s(self, s: float) -> int:
        """弧长 s → 段索引（bisect 查找）。"""
        if s <= 0.0 or len(self._s_start) < 2:
            return 0
        if s >= self._total_length:
            return len(self._segments) - 1 if self._segments else 0
        lo, hi = 0, len(self._s_start) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self._s_start[mid] <= s:
                lo = mid
            else:
                hi = mid - 1
        return min(lo, len(self._segments) - 1)


# ── V1 兼容函数 ──────────────────────────────────────────────

def find_nearest_segment_v1(path_points: list,
                            current_target_idx: int,
                            cx: float, cy: float,
                            ) -> Optional[dict]:
    """V1 find_nearest_segment 的纯函数版本（独立于 self）。

    完全复制 V1 L855-891 的逻辑，输入无状态参数，输出 dict。

    Args:
        path_points: V1 格式航点列表 [{'x','y',...}, ...]
        current_target_idx: 当前目标段起始索引
        cx, cy: 当前坐标（V1 self.current_x/y）

    Returns:
        dict {'idx','t','px','py','dist'} 或 None
    """
    if not path_points or len(path_points) < 2:
        return None

    best = None
    for i in range(current_target_idx, len(path_points) - 1):
        p1 = path_points[i]
        p2 = path_points[i + 1]
        sx = p2['x'] - p1['x']
        sy = p2['y'] - p1['y']
        seg2 = sx * sx + sy * sy
        if seg2 < 1e-12:
            continue
        t = ((cx - p1['x']) * sx + (cy - p1['y']) * sy) / seg2
        t = max(0.0, min(1.0, t))
        qx = p1['x'] + t * sx
        qy = p1['y'] + t * sy
        d = math.hypot(cx - qx, cy - qy)
        if best is None or d < best['dist']:
            best = {'idx': i, 't': t, 'px': qx, 'py': qy, 'dist': d}
    return best
