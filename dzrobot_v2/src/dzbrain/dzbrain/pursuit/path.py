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
#: 顶点前瞻重投影的越顶步长（米）：把二次投影的下界抬到顶点**之后**这幺多。
#: 取值理由：必须 > 浮点噪声（否则"更接近 s_hint"的裁决会把上一段端点又选回来 ——
#: 实测步长 1e-3 时上一段 d²=0 仍然胜出），又必须 ≪ 段长 0.35 m（否则二次投影会
#: 跳过下一段本身）。1 mm 同时满足两条，且与本模块的等距容差同量级
#: （sqrt(_TIE_D2_M2) = 1e-3 m）。
_VERTEX_STEP_M = 1e-3


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
        """本段与下一段的夹角（度，0 = 共线，180 = 原路折返）。末段无下一段 → 0。

        ⚠️ 这是两条段的**行进方向**夹角，不是车头要转的角度 —— 需要后者请用
        `corner_turn_deg()`（下一段是倒车段时两者差 180°，见其注释）。
        """
        if seg_index < 0 or seg_index + 1 >= len(self._segments):
            return 0.0
        h0 = self._segments[seg_index][5]
        h1 = self._segments[seg_index + 1][5]
        return abs(math.degrees(geo.normalize_angle(h1 - h0)))

    def car_heading_on(self, seg_index: int) -> float:
        """在 `seg_index` 这段上行驶时，**车头**该朝的方向（弧度）。

        ★ 与 `heading_at()` 的区别：那是**行进方向**。倒车段的车头与行进方向相反 ——
          现场路线 `F10→F11→E11→R:F11→F10` 里 seg1（F11→E11，行进 +90°）与 seg2
          （E11→R:F11，倒车、行进 −90°）**行进方向差 180°，但车头都是 +90°**：
          车倒着原路退回，一度都不用转。
        """
        if seg_index < 0 or seg_index >= len(self._segments):
            return 0.0
        h = self._segments[seg_index][5]
        return geo.normalize_angle(h + math.pi) if self.segment_is_reverse(seg_index) else h

    def corner_turn_deg(self, seg_index: int) -> float:
        """车头在顶点处**实际需要转过的角度**（0~180°）。末段无下一段 → 0。

        ★★ 这是本项目一个真 bug 的来源：旧代码在 PIVOT 判据里用 `corner_angle_deg`
          （**行进方向**夹角）。折返压线时下一段若是倒车段（`R:`），行进方向差 180°
          而**车头一度都不用转**（直接倒回去）⇒ PIVOT 被误触发，还把车头**转反 180°**，
          之后纯追踪自己跟自己打架。实闭环 bench 实测（现场 F11 簇）：
          `v` 在 +0.150/−0.080 之间反复翻、`yaw` 一直转、每 12s 触发一次「绕圈跳点」，
          而正确行为是**保持车头、倒车退回**。
        """
        if seg_index < 0 or seg_index + 1 >= len(self._segments):
            return 0.0
        return abs(math.degrees(geo.normalize_angle(
            self.car_heading_on(seg_index + 1) - self.car_heading_on(seg_index))))

    # ------------------------------------------------------------------ #
    #  投影（核心）
    # ------------------------------------------------------------------ #
    def project(self, x: float, y: float, *, s_hint: float | None = None,
                window_m: float = 0.60, allow_backward: bool = True,
                motion_heading: float | None = None,
                back_m: float | None = None,
                s_min: float | None = None) -> Projection:
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
        ★★ `s_min`：**弧长地板**（单调推进的硬下界）——已经走过的顶点不允许再退回去。
          请参见 `_search_window()` 里的长注释：它断的是"倒车段↔正向段交界处"的自指环。
        ★★ 顶点前瞻重投影：最优点**正好贴在当前段末端顶点**（`t ≥ 1`）且还有下一段时，
          内部 hint 会推到 `s_at_waypoint(i+1) + 1e-3` 再投影一次（最多一次），采用"不比
          第一遍更差"的那个结果 —— 因为纯追踪在顶点上必须用**下一段**算前视，返回顶点本身
          会让前视点塌缩到车身上（舵角恒 0、车直行漂出走廊，详见函数内 2b 段注释）。
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

        # ---- 1. 确定搜索窗口的段区间 + 弧长下界（bisect，O(log N)） ----
        # 抽成闭包是为"顶点前瞻重投影"（见本函数末尾）复用 —— 两遍投影必须**逐字同一套**
        # 窗口/等距裁决规则，否则第二遍会引入与第一遍不一致的语义。
        hint_ref: list[float | None] = [s_hint]      # 当前用于"连续性裁决"的参考弧长

        def _search_window() -> tuple[int, int, float]:
            """(i_first, i_last, t_floor)。窗口下界必须在**弧长**上生效，不能只按"段"生效。"""
            hint = hint_ref[0]
            if hint is None:
                return 0, n - 1, 0.0
            h = hint if 0.0 <= hint <= self._total else (0.0 if hint < 0.0 else self._total)
            back = window_m if back_m is None else max(0.0, float(back_m))
            lo = (h - back) if allow_backward else h
            # ★★ 弧长地板：**已经走过的顶点不允许再退回去**（与仿真的 `_project_with_floor`
            #    同语义）。为什么必须要：倒车段→正向段的交界处（现场 F11：seg12 E11→R:F11
            #    倒车结束、seg13 正向去 F10），两条候选的 s 都在顶点附近、Δs 几乎相同 ⇒
            #    "连续性裁决"分不出来，只能落到"行进方向"；而方向由命令符号决定、
            #    命令符号又由 `is_rev` 决定 ⇒ 自指环复现。而且此时车真的在来回抖
            #    （v=±0.08~0.15 × 0.02 s = 1.6~3 mm/拍），**大于**行进方向的锁存门限，
            #    于是"实测方向"也每拍跟着翻 —— 探针实测 30 s 内 715 次符号翻转、位置冻住。
            #    地板把"顶点之前的那条段"整个排除出搜索窗口，环就断了。
            if s_min is not None and float(s_min) > lo:
                lo = float(s_min)
            if lo < 0.0:
                lo = 0.0
            hi = h + window_m
            if hi > self._total:
                hi = self._total
            i0 = self._seg_index_at(lo)
            i1 = bisect.bisect_right(s_start, hi) - 1
            if i1 < i0:
                i1 = i0
            elif i1 > n - 1:
                i1 = n - 1
            # ★ 搜索区间是按段取的（`i_first = seg_index_at(lo)`），若不对首段做 t 下限，
            #   首段会被**整段**搜索 —— 于是 `back_m=0.15` 也可能返回 0.3~0.6 m 之后的
            #   候选（共线折返的两条段正好都落在同一个首段区间里），限位形同虚设。
            #   实测：不加这一钳制时 `back_m` 完全挡不住 s 在 0.202↔0.499 之间倒带。
            seg_len = self._segments[i0][4]
            floor = 0.0
            if seg_len > _EPS:
                floor = (lo - s_start[i0]) / seg_len
                if floor < 0.0:
                    floor = 0.0
                elif floor > 1.0:
                    floor = 1.0
            return i0, i1, floor

        # ---- 2. 窗口内找最近点（用平方距离比较，省 sqrt） ----
        # ★ 自重叠段的裁决（现场图 B6→A6→R:B6→R:D6 就是这种：折返压在同一条线上）：
        #   两条段到车的距离完全相等（都为 0），若仍按"先找到的优先"，投影会永远贴回
        #   前一条段的末端 ⇒ 沿程弧长 s 卡在段界上不再前进（仿真实测：s 在 7.58↔8.400
        #   之间反复跳，任务推进判据永不满足，车原地绕圈）。
        #   ★ 但"位置"本身裁决不了：同一物理点在两条重叠段上**完全等距**。
        #     真正的判据是 `motion_heading`（车正在往哪边走）：往前一趟 ⇒ 命中前一条段，
        #     倒回来 ⇒ 命中后一条段。取不到方向时退化为"更接近 s_hint 的候选"。
        def _nearest() -> tuple[int, float, float, float, float]:
            """(best_i, best_t, best_t_raw, best_d2, best_s)；整条路径全退化段时返回 -1。"""
            i0, i1, t_floor = _search_window()
            hint = hint_ref[0]
            b_i = -1
            b_t = 0.0
            b_t_raw = 0.0
            b_d2 = float("inf")
            b_s = 0.0
            b_ds = float("inf")         # |s - s_hint|
            tie_tol = _TIE_D2_M2
            for i in range(i0, i1 + 1):
                x0, y0, x1, y1, length, heading_i = self._segments[i]
                if length <= _EPS:
                    continue
                ux = (x1 - x0) / length
                uy = (y1 - y0) / length
                t_raw = ((x - x0) * ux + (y - y0) * uy) / length
                t = 0.0 if t_raw < 0.0 else (1.0 if t_raw > 1.0 else t_raw)
                if i == i0 and t < t_floor:
                    t = t_floor
                px = x0 + (x1 - x0) * t
                py = y0 + (y1 - y0) * t
                d2 = (x - px) * (x - px) + (y - py) * (y - py)
                s_i = s_start[i] + length * t
                if b_i < 0:
                    take = True
                elif d2 < b_d2 - tie_tol:
                    take = True
                elif abs(d2 - b_d2) <= tie_tol:
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
                    if hint is not None and abs(abs(s_i - hint) - b_ds) > 1e-9:
                        take = abs(s_i - hint) < b_ds              # ① 连续性优先
                    elif motion_heading is not None:
                        cos_new = math.cos(heading_i - motion_heading)
                        cos_best = math.cos(self._segments[b_i][5] - motion_heading)
                        take = cos_new > cos_best + 1e-12          # ② 连续性分不出 → 比方向
                    elif hint is None:
                        take = i > b_i                             # ③ 无提示 → 取更靠后的段
                    # ④ 连续性完全并列、又没有方向信息 → 保持先找到的那条（稳定的旧行为）
                else:
                    take = False
                if take:
                    b_d2 = d2
                    b_i = i
                    b_t = t
                    b_t_raw = t_raw
                    b_s = s_i
                    b_ds = abs(s_i - hint) if hint is not None else 0.0
            return b_i, b_t, b_t_raw, b_d2, b_s

        best_i, best_t, best_t_raw, best_d2, best_s = _nearest()
        # ★ passed_end 只能由**第一遍**（调用方给的窗口）裁决：二次投影把窗口刻意推到
        #   顶点之后，若让第二遍重算，末段越界的语义会被改写。
        passed_end = (best_i == n - 1) and (best_t_raw > 1.0)

        # ---- 2b. ★顶点前瞻重投影（车"正好站在直角顶点上"时不许把投影钉在上一段） ----
        #  现场图 F10（wp10，s=3.500）是三条段等距的直角顶点：seg9 到顶点、seg10 从顶点
        #  向西、seg11 从顶点向北。车被 Localization 钉到格心 (0.525, 2.975) 后，三条段
        #  的 d² **完全相等**（全 = 0.015876），等距裁决按"s 更接近 s_hint"把 seg9 的
        #  端点 t=1.000 留了下来。而纯追踪在顶点上必须用**下一段**算前视：
        #  返回"顶点本身"会让 find_lookahead_point 的顶点钳制把前视点钳到车自己身上
        #  ⇒ 弦长 0 ⇒ 舵角恒 +0.0° ⇒ 车直行漂出走廊（离线复现：2.5 s 内 seg9 s 不动、
        #  dist 0.000→0.300、舵角恒 0、v 恒 +0.120）。
        #  判据收得很紧：**只**在最优点确实贴在当前段末端（t ≥ 1）、且还有下一段时，把
        #  内部 hint 推到 `s_at_waypoint(i+1) + _VERTEX_STEP_M` 再投影**一次**（绝不递归、
        #  绝不循环）。`back_m = _VERTEX_STEP_M` 让下界正好落在顶点上 —— 顶点之后那些段
        #  的 t=0 候选能进来，上一段的端点被窗口下界排除，1 mm 的浮点噪声也被覆盖。
        #  ★ 保守取舍：只有当第二遍**不比第一遍更差**（dist ≤ 原 dist + 数值容差）时才采用。
        #    反例（L 形路径、车沿上一段方向已冲过顶点 0.2~0.5 m）：此时最近点是上一段的
        #    延长线，第二遍会更差 ⇒ 保留第一遍，既有语义一字不改。
        #    没有这条，`allow_backward=False` 的"只前进"与末点越界语义都会被改写。
        if best_i + 1 < n and best_t >= 1.0 and best_t_raw <= 1.0 + _EPS:
            s_vertex = s_start[best_i] + self._segments[best_i][4]
            # 第一遍的 s 必须**确实落在顶点上**：`allow_backward=False` 时它可能被窗口
            # 下界钳在别处，那种情形与"站在顶点"无关。
            if abs(best_s - s_vertex) <= _VERTEX_STEP_M:
                hint_ref[0] = s_vertex + _VERTEX_STEP_M
                saved_back = back_m
                back_m = _VERTEX_STEP_M
                try:
                    i2, t2, t_raw2, d2, s2 = _nearest()
                finally:
                    back_m = saved_back
                if i2 >= 0 and d2 <= best_d2 + _VERTEX_STEP_M * _VERTEX_STEP_M:
                    best_i, best_t, best_t_raw, best_d2, best_s = i2, t2, t_raw2, d2, s2

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
        return Projection(seg_index=best_i, t=best_t, x=px, y=py, s=s,
                          dist=math.sqrt(best_d2), seg_length=length,
                          seg_heading=heading, passed_end=passed_end)

    def __repr__(self) -> str:      # pragma: no cover - 仅日志用
        return (f"Path(segments={len(self._segments)}, "
                f"length={self._total:.3f}m, name={self._route.name!r})")
