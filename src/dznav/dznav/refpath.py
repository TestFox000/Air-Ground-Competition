# -*- coding: utf-8 -*-
"""参考路径生成：格点折线 -> 曲率受限的稠密中心线。

对原版（发学员/现役）的拔高点 1：弯道从"模式机 + 开环满舵弧 +
弯后硬吸附"进化为"圆角 fillet + 全程闭环跟踪"。每个内顶点按
外侧偏差预算反解圆角半径 R，并夹到单侧物理曲率极限
（servo.kappa_max，支持左右不对称档案）：

    R = min( R_dev(θc, dev_cap), R_phys(方向), 0.45 * 邻段可用长 )

其中 R_dev = dev_cap / (1/cos(θc/2) - 1)。90° 弯在 dev_cap=8cm 下
R≈0.19 m，理想舵机 κ_max=5.06 (R=0.198) 恰好覆盖 —— 全程闭环可行。
θc ≥ arc_fallback_deg（默认 135°，掉头类）物理上无法闭环跟住，
回落为 ARC 机动（低速满舵 + yaw 停止 + 出口软锚点），即原版机制的
收编保留：它是例外路径而非主路径。

关键途经点（雷格）默认 dev_cap_critical=3cm 的更紧预算；若该弯
几何上给不出（所需曲率超物理极限），照常 fillet 但打 WARN —— 排雷
半径 6cm 的硬约束应由规划层保证"雷在直段上"，这里只兜底告警。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .contract import wrap_pi
from .servo import ServoModel

#: 直行判定：转角小于此值视作共线（压缩）
STRAIGHT_THROUGH_RAD = 0.12


@dataclass
class Waypoint:
    x: float
    y: float
    label: str = ""
    shoot_stop: bool = False          # '*' 射击停站
    critical: bool = False            # 关键途经点（雷格）：收紧偏差预算
    served: bool = False              # 停站已服务（防复驶入半径内反复触发）


@dataclass
class Corner:
    """内顶点弯道记录。mode: fillet | arc | pass（直行穿越）。"""
    idx: int                          # 对应 waypoints 下标
    mode: str
    turn_sign: float                  # +1 左(CCW) / -1 右(CW)
    angle: float                      # 转角幅值 rad
    radius: float = 0.0               # fillet 半径
    dev: float = 0.0                  # fillet 对顶点最大偏差 m
    exit_x: float = 0.0               # arc 机动出口
    exit_y: float = 0.0
    exit_th: float = 0.0


@dataclass
class PathOptions:
    dev_cap: float = 0.08             # 普通弯顶点偏差预算 m
    dev_cap_critical: float = 0.03    # 关键途经点预算 m
    arc_fallback_deg: float = 125.0   # 超过此转角走 ARC 机动
    sample_step: float = 0.02         # 稠密采样步长 m
    arc_exit_len: float = 0.15        # ARC 出口锚点距顶点距离 m


@dataclass
class ReferencePath:
    samples: list = field(default_factory=list)   # [(s, x, y, th, k)]
    corners: list = field(default_factory=list)
    waypoints: list = field(default_factory=list) # 原始 Waypoint 序列
    vertex_s: list = field(default_factory=list)  # 各途经点对应 s
    length: float = 0.0

    # ---------------------------------------------------------------- 投影
    def project(self, x: float, y: float, s_hint: float,
                back: float = 0.30, fwd: float = 0.60) -> dict:
        """位姿投影到中心线（窗口搜索）。

        返回 {s, cte, th, k, ahead}；cte>0 = 车在路径左侧。
        窗口内找不到样本时退回全局最近（恢复模式兜底）。
        """
        ss = self.samples
        lo = self._idx_at(s_hint - back)
        hi = self._idx_at(s_hint + fwd)
        best_i, best_d = None, float("inf")
        for i in range(lo, hi + 1):
            d = math.hypot(x - ss[i][1], y - ss[i][2])
            if d < best_d:
                best_d, best_i = d, i
        # 冷启动/大偏差：车在窗口外（如首拍即深入路径），回退全局最近
        if best_i is None or best_d > 0.35:
            best_i = self._nearest_global(x, y)
        s, px, py, th, k = ss[best_i]
        # 投影到相邻段精化（前或后，取更近者）
        if best_i + 1 < len(ss):
            s, cte, th, k = self._proj_seg(ss[best_i], ss[best_i + 1], x, y,
                                           s, th, k)
        elif best_i > 0:
            s, cte, th, k = self._proj_seg(ss[best_i - 1], ss[best_i], x, y,
                                           s, th, k)
        else:
            cte = 0.0
        return {"s": s, "cte": cte, "th": th, "k": k,
                "ahead": self.length - s}

    def _proj_seg(self, a: tuple, b: tuple, x: float, y: float,
                  s_fb: float, th_fb: float, k_fb: float):
        s0, x0, y0, th0, k0 = a
        s1, x1, y1, th1, k1 = b
        dx, dy = x1 - x0, y1 - y0
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-12:
            return s_fb, 0.0, th_fb, k_fb
        t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / seg2))
        qx, qy = x0 + t * dx, y0 + t * dy
        th = th0 + wrap_pi(th1 - th0) * t
        k = k0 + (k1 - k0) * t
        # cte：切向左法线 (-ty, tx)；点积>0 = 左侧
        tx, ty = math.cos(th), math.sin(th)
        cte = -(x - qx) * ty + (y - qy) * tx
        return s0 + t * (s1 - s0), cte, th, k

    def _idx_at(self, s: float) -> int:
        ss = self.samples
        if not ss:
            return 0
        if s <= 0.0:
            return 0
        if s >= self.length:
            return len(ss) - 1
        # 样本近似等距：按比例初猜后线性校正
        i = int(s / self.length * (len(ss) - 1))
        i = max(0, min(len(ss) - 1, i))
        while i > 0 and ss[i][0] > s:
            i -= 1
        while i < len(ss) - 1 and ss[i + 1][0] < s:
            i += 1
        return i

    def _nearest_global(self, x: float, y: float) -> int:
        best_i, best_d = 0, float("inf")
        for i, (_s, px, py, _th, _k) in enumerate(self.samples):
            d = math.hypot(x - px, y - py)
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def kappa_max_window(self, s: float, dist: float) -> float:
        """[s, s+dist] 窗口内 |κ| 峰值（限速用）。"""
        lo, hi = self._idx_at(s), self._idx_at(s + dist)
        return max(abs(self.samples[i][4]) for i in range(lo, hi + 1))

    def at_s(self, s: float) -> tuple[float, float, float, float]:
        """按弧长取参考 (x, y, th, k)（线性插值；供前馈取 κ 用）。"""
        ss = self.samples
        if not ss:
            return (0.0, 0.0, 0.0, 0.0)
        if s <= 0.0:
            return ss[0][1:]
        if s >= self.length:
            return ss[-1][1:]
        i = self._idx_at(s)
        if i + 1 >= len(ss):
            return ss[-1][1:]
        s0, x0, y0, th0, k0 = ss[i]
        s1, x1, y1, th1, k1 = ss[i + 1]
        if s1 == s0:
            return (x0, y0, th0, k0)
        t = (s - s0) / (s1 - s0)
        return (x0 + t * (x1 - x0), y0 + t * (y1 - y0),
                th0 + wrap_pi(th1 - th0) * t, k0 + t * (k1 - k0))


# ---------------------------------------------------------------- 构建
def build_reference(waypoints: list, servo: ServoModel,
                    opts: PathOptions | None = None,
                    warn=None) -> ReferencePath:
    """Waypoint 序列 -> ReferencePath。waypoints[0] 为当前车位姿起点。"""
    opts = opts or PathOptions()
    warn = warn or (lambda *_: None)

    # 1) 去重共线点（保标志）
    wps: list[Waypoint] = [waypoints[0]]
    for w in waypoints[1:]:
        if math.hypot(w.x - wps[-1].x, w.y - wps[-1].y) < 1e-6:
            wps[-1].shoot_stop = wps[-1].shoot_stop or w.shoot_stop
            wps[-1].critical = wps[-1].critical or w.critical
            if w.label:
                wps[-1].label = w.label
        else:
            wps.append(w)

    corners: list[Corner] = [Corner(0, "pass", 0.0, 0.0)]
    for i in range(1, len(wps) - 1):
        a, b, c = wps[i - 1], wps[i], wps[i + 1]
        d1x, d1y = b.x - a.x, b.y - a.y
        d2x, d2y = c.x - b.x, c.y - b.y
        l1 = math.hypot(d1x, d1y)
        l2 = math.hypot(d2x, d2y)
        if l1 < 1e-9 or l2 < 1e-9:
            corners.append(Corner(i, "pass", 0.0, 0.0))
            continue
        cross = d1x * d2y - d1y * d2x
        dot = d1x * d2x + d1y * d2y
        ang = math.atan2(abs(cross), dot)          # 转角幅值 (0, pi]
        sign = 1.0 if cross > 0 else -1.0
        if ang < STRAIGHT_THROUGH_RAD:
            corners.append(Corner(i, "pass", sign, ang))
            continue
        if math.degrees(ang) >= opts.arc_fallback_deg:
            ex = b.x + opts.arc_exit_len * d2x / l2
            ey = b.y + opts.arc_exit_len * d2y / l2
            corners.append(Corner(i, "arc", sign, ang,
                                  exit_x=ex, exit_y=ey,
                                  exit_th=math.atan2(d2y, d2x)))
            continue
        # fillet 半径：偏差预算/邻腿长给上限，物理曲率给下限（R >= R_phys）。
        # 预算给不出（R_dev < R_phys）时抬到物理半径：偏差超出预算但仍在
        # 17.5cm 失格线内；关键途经点（雷格）额外告警交规划层纠偏。
        cap = opts.dev_cap_critical if b.critical else opts.dev_cap
        denom = 1.0 / math.cos(ang / 2.0) - 1.0
        r_dev = cap / denom if denom > 1e-9 else cap / 1e-9
        r_phys = 1.0 / max(0.05, servo.kappa_max(sign))
        r = min(r_dev, 0.45 * l1, 0.45 * l2)
        if r < r_phys:
            r = r_phys
            if b.critical:
                warn(f"wp#{i} {b.label}: 转角{math.degrees(ang):.0f}° 偏差预算"
                     f"{cap*100:.0f}cm 超物理半径 {r_phys:.2f}m，按物理极限"
                     f" fillet（偏差≈{r*denom*100:.0f}cm，规划层应避免雷格处转弯）")
        corners.append(Corner(i, "fillet", sign, ang, radius=r,
                              dev=r * denom))

    # 2) 逐段铺几何点（先不计 s），最后按真实累计距离生成弧长坐标。
    #    tangent[i] = R_i * tan(ang_i/2)：fillet 角在其前后两段腿上
    #    各占用一个切点段；相邻两角在同一段腿上的占用之和 < 0.9*腿长，
    #    中间直线恒存在（R 已被 0.45*邻腿长 夹过）。
    tangent = [0.0] * len(wps)
    for c0 in corners:
        if c0.mode == "fillet":
            tangent[c0.idx] = c0.radius * math.tan(c0.angle / 2.0)

    pts: list[list] = [[wps[0].x, wps[0].y, None, 0.0]]   # th 待回填

    for i in range(len(wps) - 1):
        a, b = wps[i], wps[i + 1]
        seg_len = math.hypot(b.x - a.x, b.y - a.y)
        ux, uy = (b.x - a.x) / seg_len, (b.y - a.y) / seg_len
        th_seg = math.atan2(uy, ux)
        t_start = tangent[i] if (i > 0 and corners[i].mode == "fillet") else 0.0
        c_next = corners[i + 1] if i + 1 < len(corners) else None
        t_end = tangent[i + 1] if (c_next and c_next.mode == "fillet") else 0.0
        straight_len = max(0.0, seg_len - t_start - t_end)
        # 直线段：j 从 1 起（j=0 点 = 上一弧出口/上一顶点，已入列）
        n = max(1, int(straight_len / opts.sample_step) + 1)
        for j in range(1, n + 1):
            d = t_start + straight_len * j / n
            pts.append([a.x + ux * d, a.y + uy * d, th_seg, 0.0])
        # 下一顶点圆弧（入口切点已作为直线段末点入列）
        if c_next is not None and c_next.mode == "fillet":
            _sample_fillet(pts, wps, corners, tangent, i + 1, opts)

    # 3) 起点航向回填 + 弧长坐标累计
    if len(pts) >= 2 and pts[0][2] is None:
        pts[0][2] = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
    ss: list[tuple] = []
    s = 0.0
    ss.append((0.0, pts[0][0], pts[0][1], pts[0][2], pts[0][3]))
    for j in range(1, len(pts)):
        ds = math.hypot(pts[j][0] - pts[j - 1][0], pts[j][1] - pts[j - 1][1])
        s += ds
        ss.append((s, pts[j][0], pts[j][1], pts[j][2], pts[j][3]))

    # 4) 途经点触发 s：取离各途经点最近的样本
    vertex_s = [0.0] * len(wps)
    for i, w in enumerate(wps):
        best_d, best_s = float("inf"), ss[-1][0]
        for s0, x, y, _th, _k in ss:
            d = math.hypot(x - w.x, y - w.y)
            if d < best_d:
                best_d, best_s = d, s0
        vertex_s[i] = best_s

    return ReferencePath(samples=ss, corners=corners, waypoints=wps,
                         vertex_s=vertex_s, length=s)


def _sample_fillet(pts, wps, corners, tangent, i, opts):
    """在顶点 i 铺圆弧样本（入口切点已由直线段末点入列）。"""
    a, b, c = wps[i - 1], wps[i], wps[i + 1]
    co = corners[i]
    R = co.radius
    u1x = (b.x - a.x) / max(1e-9, math.hypot(b.x - a.x, b.y - a.y))
    u1y = (b.y - a.y) / max(1e-9, math.hypot(b.x - a.x, b.y - a.y))
    u2x = (c.x - b.x) / max(1e-9, math.hypot(c.x - b.x, c.y - b.y))
    u2y = (c.y - b.y) / max(1e-9, math.hypot(c.x - b.x, c.y - b.y))
    t = tangent[i]
    t1x, t1y = b.x - u1x * t, b.y - u1y * t          # 入口切点
    t2x, t2y = b.x + u2x * t, b.y + u2y * t          # 出口切点
    # 圆心：入口切点的法向侧（左转取左法线）
    if co.turn_sign > 0:
        nx, ny = -u1y, u1x
    else:
        nx, ny = u1y, -u1x
    cx, cy = t1x + R * nx, t1y + R * ny
    a0 = math.atan2(t1y - cy, t1x - cx)
    a1 = math.atan2(t2y - cy, t2x - cx)
    # 带符号扫角归一到 (0, 2*pi]，方向由 turn_sign 决定
    sweep_signed = co.turn_sign * (a1 - a0)
    while sweep_signed <= 1e-9:
        sweep_signed += 2.0 * math.pi
    while sweep_signed > 2.0 * math.pi + 1e-9:
        sweep_signed -= 2.0 * math.pi
    n = max(2, int(R * sweep_signed / opts.sample_step) + 1)
    for j in range(1, n + 1):
        ang = a0 + co.turn_sign * sweep_signed * j / n
        px, py = cx + R * math.cos(ang), cy + R * math.sin(ang)
        th = ang + co.turn_sign * math.pi / 2.0
        pts.append([px, py, wrap_pi(th), co.turn_sign / R])
