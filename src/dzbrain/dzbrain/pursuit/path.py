# -*- coding: utf-8 -*-
"""路径几何：把 `Route` 编译成**折线 + 弧长前缀**，提供投影 / 弧长 / 段属性查询。

旧实现的问题（docs/survey_path_executor.md §2.2、§8.1、§8.2）：
- `find_nearest_segment()` 每拍 O(N) **只向前**全扫；路线一密（文件路径 320+ 点）就是热点；
- 索引只前进无法回退，"车在倒车（`R:`）而索引在前进"这一情形没有处理；
- 转角几何在 3 处各写一遍、阈值还不一样（20° / 45° / 45°）。

本模块的对策：
- 弧长前缀 `_s_start` + `bisect` → 给 `s_hint` 时只为**窗口内**的段算距离（O(窗口)）；
- `allow_backward` 显式区分"只前进（旧行为）"与"允许回退（倒车段必需）"；
- 转角几何收敛到 `corner_angle_deg()` 一处（策略阈值仍由 Config 决定）。

★ 不变量
1. `project().s` 是**弧长**（从路线首点算起），`t ∈ [0, 1]` 是段内参数，`dist` 是到折线的距离。
2. `allow_backward=False` 时必有 `result.s >= s_hint`（索引绝不后退 = 旧行为）。
3. `passed_end=True` 仅当"最近段就是末段"且该段原始投影参数 > 1（即车已跑过末点）。
4. 段索引 `i` 的语义 = 航点 `i → i+1`；段属性（倒车等）挂在**终点航点**上，
   与 `Route.reverse_segments()` 的既有约定一致。

⚠️ 退化段（相邻同格点 → 长度 0）在投影时被跳过：`RouteBuilder.build()` 已去重，
   但手工构造的 `Route` 仍可能出现，故保留防御而不是让它除零。
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

from ..core import geometry as geo
from ..route.model import Route, Waypoint

__all__ = ["Projection", "Path"]

_EPS = 1e-9          # 段长/参数比较容差（米）
#: 自重叠段（路线折返压在同一条线上）的等距容差，单位 m²。
#: 两条段到车的距离差在这个平方容差内就视为"几何上等距"，改由 s_hint 连续性裁决 ——
#: 见 `Path.project` 的说明（现场图 B6→A6→R:B6→R:D6 就是这种结构）。
_TIE_D2_M2 = 1e-6


@dataclass(frozen=True)
class Projection:
    """车当前位置在路径上的投影（不可变，可跨线程共享）。"""

    seg_index: int          # 命中的段索引（= 航点 i → i+1）
    t: float                # 段内参数 [0, 1]
    x: float                # 投影点（米制）
    y: float
    s: float                # 投影点的弧长（从路线首点算起）
    dist: float             # 车到折线的距离
    seg_length: float       # 命中段长度
    seg_heading: float      # 命中段航向（弧度）
    passed_end: bool        # 车已跑过末点

    @property
    def s_vertex(self) -> float:
        """本段**末端顶点**的弧长（= 下一航点所在的弧长位置）。"""
        return self.s + self.seg_length * (1.0 - self.t)

    @property
    def dist_to_vertex(self) -> float:
        """沿路径到本段末端顶点还剩多少米（转角钳制/转角减速都用它）。"""
        return self.seg_length * (1.0 - self.t)


class Path:
    """一条不可变折线路径：`Path(route)` 一次编译，之后只读查询。

    ★ 线程安全：构造后所有字段只读，`project()` 无副作用（可多线程并发调用）。
    """

    def __init__(self, route: Route) -> None:
        points: tuple[Waypoint, ...] = tuple(route.waypoints)
        if len(points) < 2:
            raise ValueError(f"Path 至少需要 2 个航点才能构成路径（当前 {len(points)} 个）")

        segments: list[tuple[float, float, float, float, float, float]] = []
        s_start: list[float] = []
        acc = 0.0
        for a, b in zip(points, points[1:]):
            dx, dy = b.x - a.x, b.y - a.y
            length = math.hypot(dx, dy)
            # 零向量段用起点朝向兜底（heading_of 自带 fallback）
            segments.append((a.x, a.y, b.x, b.y, length,
                             geo.heading_of(dx, dy, fallback=a.yaw)))
            s_start.append(acc)
            acc += length

        self._route = route
        self._points = points
        self._segments = tuple(segments)
        self._s_start = tuple(s_start)
        self._total = acc

        # 停站候选（射击/扫描）+ 末点：控制器只遍历这几个索引，不做全表扫描
        stops = [i for i, w in enumerate(points) if w.is_stop_point]
        last = len(points) - 1
        if last not in stops:
            stops.append(last)
        self._stop_indices = tuple(sorted(stops))

    # ------------------------------------------------------------------ #
    #  只读属性
    # ------------------------------------------------------------------ #
    @property
    def total_length(self) -> float:
        """路径总长（米）。"""
        return self._total

    @property
    def segments(self) -> tuple[tuple[float, float, float, float, float, float], ...]:
        """每段 `(x0, y0, x1, y1, length, heading)`。"""
        return self._segments

    @property
    def segment_count(self) -> int:
        return len(self._segments)

    @property
    def route(self) -> Route:
        return self._route

    @property
    def waypoints(self) -> tuple[Waypoint, ...]:
        return self._points

    @property
    def stop_indices(self) -> tuple[int, ...]:
        """需要减速/停车的航点索引（射击停站、扫描停站、末点），升序。"""
        return self._stop_indices

    # ------------------------------------------------------------------ #
    #  弧长查询
    # ------------------------------------------------------------------ #
    def _seg_index_at(self, s: float) -> int:
        """弧长 s 落在哪一段（边界归**后**一段；越界夹紧）。"""
        i = bisect.bisect_right(self._s_start, s) - 1
        if i < 0:
            return 0
        if i > len(self._segments) - 1:
            return len(self._segments) - 1
        return i

    def point_at(self, s: float) -> tuple[float, float, int]:
        """弧长 s 处的点 -> (x, y, seg_index)。s 越界自动夹到 [0, total_length]。"""
        if s < 0.0:
            s = 0.0
        elif s > self._total:
            s = self._total
        i = self._seg_index_at(s)
        x0, y0, x1, y1, length, _ = self._segments[i]
        if length <= _EPS:
            return x0, y0, i
        t = (s - self._s_start[i]) / length
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, i

    def heading_at(self, s: float) -> float:
        """弧长 s 所在段的航向（弧度）。"""
        return self._segments[self._seg_index_at(s)][5]

    def s_at_waypoint(self, index: int) -> float:
        """航点 index 处的弧长（越界夹紧；index == 航点数-1 → 总长）。"""
        if index <= 0:
            return 0.0
        if index >= len(self._segments):
            return self._total
        return self._s_start[index]

    def remaining(self, s: float) -> float:
        """从弧长 s 到终点还剩多少米（负值夹到 0）。"""
        left = self._total - s
        return left if left > 0.0 else 0.0

    # ------------------------------------------------------------------ #
    #  段属性
    # ------------------------------------------------------------------ #
    def segment_is_reverse(self, seg_index: int) -> bool:
        """该段是否倒车（`R:` 标记挂在**终点航点**上，与旧 `_is_reverse_segment` 同语义）。"""
        if seg_index < 0 or seg_index >= len(self._segments):
            return False        # ⚠️ 越界防御式返回 False，不外抛（控制环不能因索引错位停摆）
        return bool(self._points[seg_index + 1].reverse)

    def segment_speed_scale(self, seg_index: int) -> float:
        """该段限速系数：取两端点 `speed_scale` 的**较小值**（限速挂在哪端都生效，失败偏安全）。"""
        if seg_index < 0 or seg_index >= len(self._segments):
            return 1.0
        a = self._points[seg_index].speed_scale
        b = self._points[seg_index + 1].speed_scale
        return float(min(a, b))

    def corner_angle_deg(self, seg_index: int) -> float:
        """本段与下一段的夹角（度，0 = 共线，180 = 原路折返）。末段无下一段 → 0。"""
        if seg_index < 0 or seg_index + 1 >= len(self._segments):
            return 0.0
        h0 = self._segments[seg_index][5]
        h1 = self._segments[seg_index + 1][5]
        return abs(math.degrees(geo.normalize_angle(h1 - h0)))

    # ------------------------------------------------------------------ #
    #  投影（核心）
    # ------------------------------------------------------------------ #
    def project(self, x: float, y: float, *, s_hint: float | None = None,
                window_m: float = 0.60, allow_backward: bool = True,
                motion_heading: float | None = None,
                back_m: float | None = None) -> Projection:
        """把车位置投影到折线上。

        ★ `s_hint` 给定时只在 `[s_hint - window_m, s_hint + window_m]` 内搜索
          （`allow_backward=False` 时下界收到 `s_hint`）—— 复杂度只与窗口内段数有关，
          避免"路线自交 → 误锁到另一条边"。
        ★ `allow_backward=False` 时索引只前进（旧行为，防倒退误锁）；
          倒车段（`R:`）必须用 `True`，否则车往后走时投影点不肯后退。
        ★ `back_m`：**单拍内投影点允许向后跳的最大距离**（默认 None = 用 `window_m`，
          即旧行为）。为什么需要：折返压线时去程段与回程段**完全共线**，同一个物理点
          （尤其两段的公共端点，如 B6）到两条段**等距**，而两条候选的 s 相差整整一段
          （0.0 vs 0.70）。若允许向后搜 0.6 m，s 就会在 0.0↔0.70 之间跳；跳完 hint 也跟着
          跳，而 ±window_m 的窗口又**回不到**刚跳走的那一支 ⇒ 两个吸引子互锁，
          连命令符号一起每拍翻转（实闭环 bench 与探针实测：601 次翻转、位置毫米级抖动、
          12s 后触发"拐角绕圈兜底"强制跳点）。
          正常运动每拍只走几毫米（0.12 m/s × 0.02 s = 2.4 mm），15 cm 的余量绰绰有余；
          被这条限制挡住的只有"一拍跳一整段"这种病态情况。
        ★ 末点之后：最近段 = 末段且原始参数 > 1 → `t = 1`、`s = total_length`、`passed_end=True`。
        ★★ `motion_heading`：**实际行进方向**（地图系弧度；倒车时是车头反向）。
          路线折返压在同一条线上时（现场图 B6→A6→R:B6 就是），同一物理点落在两条
          航段上**等距**，位置本身无法裁决 —— 唯一的判据是"车正在往哪边走"：
          往前一次走 ⇒ 命中前一条段；倒回来 ⇒ 命中后一条段。不传该参数时退化为
          "等距取更接近 s_hint 的候选"（无 hint 则取先找到的，保持确定性）。
        """
        n = len(self._segments)
        s_start = self._s_start
        if window_m < 0.0:      # ⚠️ 负窗口会把区间反转发散，直接当 0 处理
            window_m = 0.0

        # ---- 1. 确定搜索窗口的段区间（bisect，O(log N)） ----
        t_floor = 0.0
        if s_hint is None:
            i_first, i_last = 0, n - 1
        else:
            hint = s_hint if 0.0 <= s_hint <= self._total else (
                0.0 if s_hint < 0.0 else self._total)
            back = window_m if back_m is None else max(0.0, float(back_m))
            lo = (hint - back) if allow_backward else hint
            if lo < 0.0:
                lo = 0.0
            hi = hint + window_m
            if hi > self._total:
                hi = self._total
            i_first = self._seg_index_at(lo)
            i_last = bisect.bisect_right(s_start, hi) - 1
            if i_last < i_first:
                i_last = i_first
            elif i_last > n - 1:
                i_last = n - 1
            # ★ 窗口下界必须在**弧长**上生效，不能只按"段"生效：
            #   搜索区间是按段取的（`i_first = seg_index_at(lo)`），若不对首段做 t 下限，
            #   首段会被**整段**搜索 —— 于是 `back_m=0.15` 也可能返回 0.3~0.6 m 之后的
            #   候选（共线折返的两条段正好都落在同一个首段区间里），限位形同虚设。
            #   实测：不加这一钳制时 `back_m` 完全挡不住 s 在 0.202↔0.499 之间倒带。
            seg_len = self._segments[i_first][4]
            if seg_len > _EPS:
                t_floor = (lo - s_start[i_first]) / seg_len
                if t_floor < 0.0:
                    t_floor = 0.0
                elif t_floor > 1.0:
                    t_floor = 1.0

        # ---- 2. 窗口内找最近点（用平方距离比较，省 sqrt） ----
        # ★ 自重叠段的裁决（现场图 B6→A6→R:B6→R:D6 就是这种：折返压在同一条线上）：
        #   两条段到车的距离完全相等（都为 0），若仍按"先找到的优先"，投影会永远贴回
        #   前一条段的末端 ⇒ 沿程弧长 s 卡在段界上不再前进（仿真实测：s 在 7.58↔8.400
        #   之间反复跳，任务推进判据永不满足，车原地绕圈）。
        #   ★ 但"位置"本身裁决不了：同一物理点在两条重叠段上**完全等距**。
        #     真正的判据是 `motion_heading`（车正在往哪边走）：往前一趟 ⇒ 命中前一条段，
        #     倒回来 ⇒ 命中后一条段。取不到方向时退化为"更接近 s_hint 的候选"。
        best_i = -1
        best_t = 0.0
        best_d2 = float("inf")
        best_t_raw = 0.0
        best_s = 0.0
        best_ds = float("inf")          # |s - s_hint|
        tie_tol = _TIE_D2_M2
        for i in range(i_first, i_last + 1):
            x0, y0, x1, y1, length, heading_i = self._segments[i]
            if length <= _EPS:
                continue
            ux = (x1 - x0) / length
            uy = (y1 - y0) / length
            t_raw = ((x - x0) * ux + (y - y0) * uy) / length
            t = 0.0 if t_raw < 0.0 else (1.0 if t_raw > 1.0 else t_raw)
            if i == i_first and t < t_floor:
                t = t_floor
            px = x0 + (x1 - x0) * t
            py = y0 + (y1 - y0) * t
            d2 = (x - px) * (x - px) + (y - py) * (y - py)
            s_i = s_start[i] + length * t
            if best_i < 0:
                take = True
            elif d2 < best_d2 - tie_tol:
                take = True
            elif abs(d2 - best_d2) <= tie_tol:
                # ---- 等距（两条段几何重叠）裁决 ----
                # ★★ 顺序很关键：**先比"与上拍投影的连续性"（弧长），再比行进方向**。
                #   旧顺序（方向优先）在自重叠折返段上会形成**闭环振荡**：
                #     命令符号 → 实际运动方向 → 等距裁决选哪条段 → segment_is_reverse
                #     → 命令符号 …… 首尾相接。实闭环 bench 与探针实测（B6→A6→R:B6）：
                #     命令每拍翻转（24s 内 601 次）、段号在两条共线段间来回跳、
                #     车在毫米级抖动、12s 后触发"拐角绕圈兜底"强制跳点。
                #   连续性给出的是**稳定不动点**：两条段完全共线时，位置与瞬时方向都
                #   裁决不了，唯一有意义的语义是"沿路线连续推进"（弧长不跳变）。
                #   方向只在连续性也分不出时使用 —— 典型场景正是**折返顶点**：
                #   两条候选的 s 相同（都是顶点），这时才必须靠"车正在往哪边走"裁决。
                take = False
                if s_hint is not None and abs(abs(s_i - hint) - best_ds) > 1e-9:
                    take = abs(s_i - hint) < best_ds              # ① 连续性优先
                elif motion_heading is not None:
                    cos_new = math.cos(heading_i - motion_heading)
                    cos_best = math.cos(self._segments[best_i][5] - motion_heading)
                    take = cos_new > cos_best + 1e-12             # ② 连续性分不出 → 比方向
                elif s_hint is None:
                    take = i > best_i                             # ③ 无提示 → 取更靠后的段
                # ④ 连续性完全并列、又没有方向信息 → 保持先找到的那条（稳定的旧行为）
            else:
                take = False
            if take:
                best_d2 = d2
                best_i = i
                best_t = t
                best_t_raw = t_raw
                best_s = s_i
                best_ds = abs(s_i - hint) if s_hint is not None else 0.0

        if best_i < 0:      # 整条路径都是零长度退化段
            best_i = 0
            best_t = 0.0
            best_t_raw = 0.0
            x0, y0 = self._segments[0][0], self._segments[0][1]
            best_d2 = (x - x0) * (x - x0) + (y - y0) * (y - y0)

        x0, y0, x1, y1, length, heading = self._segments[best_i]
        px = x0 + (x1 - x0) * best_t
        py = y0 + (y1 - y0) * best_t
        s = s_start[best_i] + length * best_t
        passed_end = (best_i == n - 1) and (best_t_raw > 1.0)
        return Projection(seg_index=best_i, t=best_t, x=px, y=py, s=s,
                          dist=math.sqrt(best_d2), seg_length=length,
                          seg_heading=heading, passed_end=passed_end)

    def __repr__(self) -> str:      # pragma: no cover - 仅日志用
        return (f"Path(segments={len(self._segments)}, "
                f"length={self._total:.3f}m, name={self._route.name!r})")
