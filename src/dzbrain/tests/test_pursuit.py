# -*- coding: utf-8 -*-
"""pursuit 核心自测（**纯脚本，无 pytest**）。

    python src\\dzbrain\\tests\\test_pursuit.py

覆盖（对齐 docs/P0接口约定.md §3/§4，并逐条给数值证据）：
  1. 坐标一致性（geometry 唯一真源；与旧公式逐位一致）
  2. Path.project() 直线投影 / 末点之后 passed_end
  3. 索引窗口：500 点蛇形路径，窗口搜索 ≡ 全量搜索，且 < 100 µs/次（打印实测）
  4. allow_backward=False 只前进 / True 可回退（倒车段必需）
  5. find_lookahead_point：直线前进、跨段累加、转角顶点钳制、末点不越界
  6. compute_command 舵角：α=0、教科书式（因子 2）、gain=0.5=旧行为、限幅、速率限幅、倒车翻号
  7. speed_profile：曲率降速、停站降速、转角降速、下界、speed_scale、量化补偿
  8. brake_command：零 Twist = 刹车
  9. 与旧公式对拍（gain=0.5、ld=0.15、α=10°）

全过 exit 0；任一失败 exit 1。
"""

from __future__ import annotations

import math
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

# ⚠️ Windows 控制台默认 GBK，遇到编不出的字符会抛 UnicodeEncodeError 把测试打断：
#    这里降级成替换字符，保证"测试结果"永远不被编码问题掩盖。
try:
    sys.stdout.reconfigure(errors="replace")  # type: ignore[union-attr]
except Exception:                              # pragma: no cover
    pass

from dzbrain.core import geometry as geo                                              # noqa: E402
from dzbrain.core.config import Config, curve_speed_limit, lookahead_distance         # noqa: E402
from dzbrain.pursuit.controller import brake_command, compute_command, speed_profile  # noqa: E402
from dzbrain.pursuit.lookahead import find_lookahead_point                            # noqa: E402
from dzbrain.pursuit.path import Path                                                 # noqa: E402
from dzbrain.route.model import Route, RouteBuilder, Waypoint                         # noqa: E402

# --------------------------------------------------------------------------- #
#  极简断言框架
# --------------------------------------------------------------------------- #
_PASS = 0
_FAIL = 0
_FAILED: list[str] = []
MEASURED: dict[str, float] = {}


def ok(name: str, cond: bool, detail: str = "") -> None:
    """打印 PASS/FAIL 行；detail 里放数值证据。"""
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print(f"PASS  {name}" + (f"   [{detail}]" if detail else ""))
    else:
        _FAIL += 1
        _FAILED.append(name)
        print(f"FAIL  {name}   [{detail}]")


def close(name: str, got: float, want: float, tol: float = 1e-9, extra: str = "") -> None:
    """浮点断言：打印实测/期望/残差。"""
    ok(name, abs(got - want) <= tol,
       f"实测 {got!r} vs 期望 {want!r}，Δ={got - want:.3e} ≤ {tol:g}" + (f"，{extra}" if extra else ""))


def sec(title: str) -> None:
    print()
    print(f"--- {title} ---")


# --------------------------------------------------------------------------- #
#  测试脚手架
# --------------------------------------------------------------------------- #
def cfg_of(**pursuit) -> Config:
    """只覆盖 pursuit.* 的配置；其余键走 SCHEMA 默认值（= 现场 robot.yaml 的缺省）。"""
    return Config.from_dict({"pursuit": dict(pursuit)})


def route_of(points, name: str = "test", reverses=None) -> Route:
    """(x, y) 列表 → Route（块名只是标签，坐标由测试直接给）。"""
    b = RouteBuilder(name)
    rev = set(reverses or ())
    for i, (x, y) in enumerate(points):
        b.add(Waypoint(block=f"P{i}", x=float(x), y=float(y), yaw=0.0, reverse=(i in rev)))
    return b.build()


def straight(length: float = 5.0) -> Path:
    return Path(route_of([(0.0, 0.0), (length, 0.0)], name="straight"))


def serpentine(rows: int = 50, cols: int = 10, step: float = 0.35,
               row_gap: float = 0.70) -> Route:
    """蛇形（boustrophedon）路径：500 点 / 499 段 / 191.8 m，用于窗口与性能测试。"""
    pts = []
    for r in range(rows):
        seq = range(cols) if r % 2 == 0 else range(cols - 1, -1, -1)
        for c in seq:
            pts.append((c * step, r * row_gap))
    return route_of(pts, name=f"serpentine{rows}x{cols}")


def time_us(fn, n: int, runs: int = 3) -> float:
    """fn 单次调用耗时（µs），取 runs 批的最小值以压掉调度噪声。"""
    for _ in range(50):          # 预热
        fn()
    best = float("inf")
    for _ in range(runs):
        t0 = time.perf_counter_ns()
        for _ in range(n):
            fn()
        best = min(best, (time.perf_counter_ns() - t0) / n / 1000.0)
    return best


# --------------------------------------------------------------------------- #
#  1. 坐标一致性
# --------------------------------------------------------------------------- #
def test_coordinates() -> None:
    sec("1. 坐标一致性（geometry 唯一真源）")
    x10, y10 = geo.block_to_meters("N10")
    # ⚠️ 十进制字面量 0.525 与 (10.5-9)*0.35 差 1 ULP：块心 x = 0.5249999999999999
    #    （旧 block_to_meters 同公式 → 逐位一致，故按 1e-12 断言而不是 == 字面量）
    ok("1.1 block_to_meters('N10') = (0.525, 0.175)",
       abs(x10 - 0.525) < 1e-12 and abs(y10 - 0.175) < 1e-12,
       f"实测 ({x10!r}, {y10!r})，Δx={x10 - 0.525:+.3e}（1 ULP）")
    ok("1.2 与旧公式 (10.5-row)*0.35 / (13.5-col)*0.35 逐位一致",
       geo.block_to_meters("N10") == ((10.5 - 9) * 0.35, (13.5 - 13) * 0.35)
       and geo.block_to_meters("K9") == ((10.5 - 8) * 0.35, (13.5 - 10) * 0.35))
    xa, ya = geo.block_to_meters("A1")
    ok("1.3 block_to_meters('A1') = (3.675, 4.725) 精确相等",
       (xa, ya) == (3.675, 4.725), f"实测 ({xa!r}, {ya!r})")
    xn, yn = geo.block_to_meters("N11")
    ok("1.4 block_to_meters('N11') = (0.175, 0.175) 精确相等",
       (xn, yn) == (0.175, 0.175), f"实测 ({xn!r}, {yn!r})")
    u, v = geo.block_to_units("N10")
    ok("1.5 N10 格单位 = (1.5, 0.5)（与 /odom/localization 起点约定一致）",
       abs(u - 1.5) < 1e-12 and abs(v - 0.5) < 1e-12, f"实测 ({u!r}, {v!r})")

    # Path 必须直接吃 geometry 的输出，不得另写换算
    p = Path(route_of([geo.block_to_meters("N10"), geo.block_to_meters("K9")], name="n10-k9"))
    n10, k9 = geo.block_to_meters("N10"), geo.block_to_meters("K9")
    want = math.hypot(k9[0] - n10[0], k9[1] - n10[1])
    close("1.6 Path 段长 = 两格心距离（N10→K9）", p.total_length, want, 1e-12,
          extra="Path 复用 geometry，不另写换算")


# --------------------------------------------------------------------------- #
#  2. Path.project() 基础语义
# --------------------------------------------------------------------------- #
def test_project_basic() -> None:
    sec("2. Path.project()：直线投影 / 弧长 / 末点之后")
    path = straight(2.0)
    ok("2.0 直线路径 1 段 / 总长 2.0 m",
       path.segment_count == 1 and path.total_length == 2.0,
       f"segments={path.segment_count} 总长={path.total_length!r}")

    p = path.project(0.5, 0.3)
    ok("2.1 投影 seg_index/t 正确", p.seg_index == 0 and abs(p.t - 0.25) < 1e-12,
       f"seg={p.seg_index} t={p.t!r}")
    ok("2.2 投影 s/dist/seg_length/seg_heading 正确",
       abs(p.s - 0.5) < 1e-12 and abs(p.dist - 0.3) < 1e-12
       and p.seg_length == 2.0 and abs(p.seg_heading) < 1e-12,
       f"s={p.s!r} dist={p.dist!r} len={p.seg_length!r} heading={p.seg_heading!r}")
    ok("2.3 投影点坐标 = 垂足", (p.x, p.y) == (0.5, 0.0), f"({p.x!r}, {p.y!r})")
    ok("2.4 未过末点 → passed_end=False", p.passed_end is False)

    p2 = path.project(1.0, -0.2)
    ok("2.5 路径另一侧（负横向）投影", abs(p2.t - 0.5) < 1e-12 and abs(p2.dist - 0.2) < 1e-12
       and p2.passed_end is False, f"t={p2.t!r} dist={p2.dist!r}")

    p3 = path.project(2.5, 0.0)      # 末点之后 0.5 m
    ok("2.6 末点之后：t=1 / s=总长 / passed_end=True",
       p3.t == 1.0 and p3.s == 2.0 and p3.seg_index == 0 and p3.passed_end is True,
       f"t={p3.t!r} s={p3.s!r} seg={p3.seg_index} passed_end={p3.passed_end}")
    close("2.7 末点之后 dist = 到末点的距离", p3.dist, 0.5, 1e-12)

    p4 = path.project(-0.5, 0.1)     # 起点之前
    ok("2.8 起点之前：t=0 / s=0 / 不置 passed_end",
       p4.t == 0.0 and p4.s == 0.0 and p4.passed_end is False,
       f"t={p4.t!r} s={p4.s!r} passed_end={p4.passed_end}")
    close("2.9 起点之前 dist = 到首点的距离", p4.dist, math.hypot(0.5, 0.1), 1e-12)

    x, y, si = path.point_at(0.75)
    ok("2.10 point_at(s) 与 project 互逆 + remaining/heading_at",
       (x, y, si) == (0.75, 0.0, 0) and abs(path.remaining(0.75) - 1.25) < 1e-12
       and path.heading_at(0.75) == 0.0 and path.remaining(2.5) == 0.0,
       f"point_at(0.75)=({x!r},{y!r},{si}) remaining={path.remaining(0.75)!r}")

    # 折线：第二段上的投影必须落到 seg_index=1，且 s 含第一段长度
    lpath = Path(route_of([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], name="L"))
    pl = lpath.project(0.9, 0.6)
    ok("2.11 折线第二段投影（跨段弧长累加）",
       pl.seg_index == 1 and abs(pl.s - 1.6) < 1e-12 and abs(pl.t - 0.6) < 1e-12
       and abs(pl.seg_heading - math.pi / 2) < 1e-12,
       f"seg={pl.seg_index} s={pl.s!r} t={pl.t!r} heading={pl.seg_heading!r}")
    ok("2.12 corner_angle_deg：90° 拐角 / 直线 0° / 末段 0°",
       abs(lpath.corner_angle_deg(0) - 90.0) < 1e-9 and lpath.corner_angle_deg(1) == 0.0
       and path.corner_angle_deg(0) == 0.0,
       f"L[0]={lpath.corner_angle_deg(0)!r}° L[1]={lpath.corner_angle_deg(1)!r}° "
       f"直线[0]={path.corner_angle_deg(0)!r}°")
    ok("2.13 Path 与 Route.path_length() 一致",
       abs(lpath.total_length - lpath.route.path_length()) < 1e-12,
       f"Path={lpath.total_length!r} Route={lpath.route.path_length()!r}")

    raised = False
    try:
        Path(route_of([(0.0, 0.0)], name="single"))
    except ValueError:
        raised = True
    ok("2.13b 少于 2 个航点 → ValueError（不静默造一条空路径）", raised,
       "Route 单点 → Path 拒绝构造")

    pv = path.project(1.0, 0.0)
    ok("2.14 Projection.s_vertex / dist_to_vertex（转角减速与钳制共用）",
       abs(pv.s_vertex - 2.0) < 1e-12 and abs(pv.dist_to_vertex - 1.0) < 1e-12,
       f"s_vertex={pv.s_vertex!r} dist_to_vertex={pv.dist_to_vertex!r}")


# --------------------------------------------------------------------------- #
#  3. 索引窗口 + 性能
# --------------------------------------------------------------------------- #
def test_index_window_and_perf() -> None:
    sec("3. 索引窗口：500 点蛇形路径，窗口搜索 == 全量搜索，且 < 100 us")
    path = Path(serpentine())
    ok("3.1 蛇形路径规模", len(path.waypoints) == 500 and path.segment_count == 499
       and abs(path.total_length - 191.8) < 1e-9,
       f"航点 {len(path.waypoints)} / 段 {path.segment_count} / 总长 {path.total_length:.4f} m")

    s_probe = path.s_at_waypoint(255) + 0.17      # 段中（离顶点 0.17 m），避免边界歧义
    qx, qy, _ = path.point_at(s_probe)
    query = (qx, qy + 0.05)                        # 侧向偏 0.05 m（行距 0.70 → 全局唯一最近）

    full = path.project(*query)
    ok("3.2 全量搜索命中预期弧长与横向距离",
       abs(full.s - s_probe) < 1e-9 and abs(full.dist - 0.05) < 1e-12,
       f"实测 s={full.s!r}（s_probe={s_probe!r}）dist={full.dist!r}")

    for off in (0.0, +0.10, -0.10, +0.55, -0.55):
        win = path.project(*query, s_hint=s_probe + off)
        same = (win.seg_index == full.seg_index and abs(win.t - full.t) < 1e-12
                and abs(win.s - full.s) < 1e-12 and abs(win.dist - full.dist) < 1e-12
                and abs(win.x - full.x) < 1e-12 and abs(win.y - full.y) < 1e-12
                and win.passed_end == full.passed_end)
        ok(f"3.3 窗口搜索(hint{off:+.2f} m) 与全量搜索一致", same,
           f"Δs={win.s - full.s:.2e} Δt={win.t - full.t:.2e} Δdist={win.dist - full.dist:.2e}")

    us_win = time_us(lambda: path.project(*query, s_hint=s_probe), 2000)
    us_full = time_us(lambda: path.project(*query), 200, runs=1)
    MEASURED["us_win"] = us_win
    MEASURED["us_full"] = us_full
    ok("3.4 窗口投影耗时 < 100 us/次", us_win < 100.0,
       f"实测 {us_win:.2f} us/次（499 段路径，仅扫窗口内约 {int(2 * 0.60 / 0.35) + 1} 段）")
    ok("3.5 窗口搜索 ≪ 全量搜索（O(窗口) 生效）", us_win * 5.0 < us_full,
       f"窗口 {us_win:.2f} us vs 全量 {us_full:.2f} us（快 {us_full / us_win:.0f} 倍）")

    # allow_backward=False 的窗口下界必须仍是 s_hint（不能因"含 s_hint 所在段"而漏到段首）
    w0 = path.project(*query, s_hint=s_probe, window_m=0.0)
    ok("3.6 window_m=0（只搜本段）仍能命中",
       abs(w0.s - full.s) < 1e-12, f"s={w0.s!r}")


# --------------------------------------------------------------------------- #
#  4. 允许后退 vs 只前进
# --------------------------------------------------------------------------- #
def test_allow_backward() -> None:
    sec("4. allow_backward：False=只前进（旧行为）/ True=可回退（倒车段必需）")
    path = Path(serpentine())
    s_now = path.s_at_waypoint(255) + 0.17
    bx, by, _ = path.point_at(s_now - 0.5)         # 车实际在后方 0.5 m
    query = (bx, by + 0.05)

    p_fwd = path.project(*query, s_hint=s_now, allow_backward=False)
    p_bwd = path.project(*query, s_hint=s_now, allow_backward=True)

    ok("4.1 allow_backward=False → 索引不后退（s ≥ s_hint）", p_fwd.s >= s_now - 1e-9,
       f"s={p_fwd.s!r} s_hint={s_now!r}（差 {p_fwd.s - s_now:+.3e}）")
    close("4.2 allow_backward=False → 停在窗口下界（最优允许点=s_hint）", p_fwd.s, s_now, 1e-9)
    close("4.3 allow_backward=True → 正确回退到车所在弧长", p_bwd.s, s_now - 0.5, 1e-9,
          extra="倒车段（R:）没有这条就会永远锁在前方，投影点不跟车")
    ok("4.4 两种模式的 seg 都在窗口内且 dist 合理",
       abs(p_fwd.dist - math.hypot(0.5, 0.05)) < 1e-9 and abs(p_bwd.dist - 0.05) < 1e-12,
       f"fwd.dist={p_fwd.dist!r}（到 s_hint 处）bwd.dist={p_bwd.dist!r}（垂距）")

    # 倒车段标记：段属性挂在终点航点（与 Route.reverse_segments 同语义）
    rpath = Path(route_of([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], name="rev", reverses={1}))
    ok("4.5 segment_is_reverse：R: 段 True，其它/越界 False",
       rpath.segment_is_reverse(0) is True and rpath.segment_is_reverse(1) is False
       and rpath.segment_is_reverse(-1) is False and rpath.segment_is_reverse(9) is False,
       f"seg0={rpath.segment_is_reverse(0)} seg1={rpath.segment_is_reverse(1)} "
       f"越界={rpath.segment_is_reverse(9)}")
    ok("4.6 Route.reverse_segments() 与 Path 判定一致",
       rpath.route.reverse_segments() == [0], f"{rpath.route.reverse_segments()}")

    # 倒车实测：同一条倒车轨迹，两种模式各自的 s 演化（这是"必须允许后退"的直接证据）
    sp = straight(5.0)
    s_fwd, s_bwd = 2.0, 2.0
    trace_fwd, trace_bwd = [], []
    for x in (2.0, 1.5, 1.0, 0.5):
        p_f = sp.project(x, 0.0, s_hint=s_fwd, allow_backward=False)
        p_b = sp.project(x, 0.0, s_hint=s_bwd, allow_backward=True)
        s_fwd, s_bwd = p_f.s, p_b.s
        trace_fwd.append(round(s_fwd, 3))
        trace_bwd.append(round(s_bwd, 3))
    ok("4.7 车倒退 1.5 m 时 allow_backward=False 的 s 被锁死，True 才跟得上",
       trace_fwd == [2.0, 2.0, 2.0, 2.0] and trace_bwd == [2.0, 1.5, 1.0, 0.5],
       f"只前进 s={trace_fwd}；可回退 s={trace_bwd}")


# --------------------------------------------------------------------------- #
#  5. 前视点
# --------------------------------------------------------------------------- #
def test_lookahead() -> None:
    sec("5. find_lookahead_point：前进 / 跨段 / 转角顶点钳制 / 末点不越界")
    cfg = cfg_of()

    # 5a 直线段：前视点位于投影点前方 ld
    path = straight(5.0)
    proj = path.project(1.0, 0.1)
    lp = find_lookahead_point(path, proj, 0.3, cfg)
    close("5.1 直线：前视点在投影点前方 ld=0.3", lp.s, proj.s + 0.3, 1e-12)
    ok("5.2 直线：点坐标/段号/未钳制",
       abs(lp.x - 1.3) < 1e-12 and lp.y == 0.0 and lp.seg_index == 0
       and lp.clamped_to_vertex is False,
       f"({lp.x!r}, {lp.y!r}) seg={lp.seg_index} clamped={lp.clamped_to_vertex}")

    # 5b 跨段累加：L 形路径 (0,0)->(0.2,0)->(0.2,0.5)
    lpath = Path(route_of([(0.0, 0.0), (0.2, 0.0), (0.2, 0.5)], name="L"))
    pl = lpath.project(0.1, 0.0)
    lp2 = find_lookahead_point(lpath, pl, 0.30, cfg)
    ok("5.3 跨段：ld=0.3 从 s=0.1 累加进第二段（s=0.4, seg=1）",
       abs(lp2.s - 0.4) < 1e-12 and lp2.seg_index == 1 and abs(lp2.y - 0.2) < 1e-12
       and lp2.clamped_to_vertex is False,
       f"s={lp2.s!r} seg={lp2.seg_index} ({lp2.x!r},{lp2.y!r})（距顶点 0.1 m > 0.04 → 不钳制）")

    # 5c 转角顶点钳制：距 90° 顶点 0.02 m < vertex_clamp_m(0.04)
    pc = lpath.project(0.18, -0.02)
    ok("5.3b 前置：投影落在第一段末端 0.02 m 处、夹角 90°",
       pc.seg_index == 0 and abs(pc.dist_to_vertex - 0.02) < 1e-12
       and abs(lpath.corner_angle_deg(0) - 90.0) < 1e-9,
       f"seg={pc.seg_index} dist_to_vertex={pc.dist_to_vertex!r} 夹角={lpath.corner_angle_deg(0)}°")
    lp3 = find_lookahead_point(lpath, pc, 0.30, cfg)
    ok("5.4 转角钳制：clamped_to_vertex=True 且点 = 顶点(0.2, 0)",
       lp3.clamped_to_vertex is True and (lp3.x, lp3.y) == (0.2, 0.0)
       and abs(lp3.s - 0.2) < 1e-12 and lp3.seg_index == 0,
       f"clamped={lp3.clamped_to_vertex} ({lp3.x!r},{lp3.y!r}) s={lp3.s!r} seg={lp3.seg_index}")

    # 5d 末点不越界
    short = Path(route_of([(0.0, 0.0), (0.2, 0.0)], name="short"))
    ps = short.project(0.1, 0.0)
    lp4 = find_lookahead_point(short, ps, 0.30, cfg)       # 0.1+0.3 > 0.2
    ok("5.5 末点钳制：前视点 = 最后一点，s = 总长，不越界",
       (lp4.x, lp4.y) == (0.2, 0.0) and lp4.s == short.total_length
       and lp4.s <= short.total_length and lp4.clamped_to_vertex is False,
       f"({lp4.x!r},{lp4.y!r}) s={lp4.s!r} total={short.total_length!r}")
    lp5 = find_lookahead_point(short, ps, 0.0, cfg)
    ok("5.6 ld=0 → 退化为投影点本身", abs(lp5.s - ps.s) < 1e-12 and lp5.clamped_to_vertex is False,
       f"s={lp5.s!r}")

    # 5e 顶点钳制的阈值边界：距顶点 0.05 m > 0.04 → 不钳制
    pc2 = lpath.project(0.15, -0.02)               # 距顶点 0.05 m
    lp6 = find_lookahead_point(lpath, pc2, 0.30, cfg)
    ok("5.7 距顶点 0.05 m > vertex_clamp_m(0.04) → 不钳制（阈值真的生效）",
       lp6.clamped_to_vertex is False and abs(lp6.s - (pc2.s + 0.30)) < 1e-12,
       f"dist_to_vertex={pc2.dist_to_vertex!r} clamped={lp6.clamped_to_vertex} s={lp6.s!r}")
    cfg_noclamp = cfg_of(vertex_clamp_m=0.10)
    lp7 = find_lookahead_point(lpath, pc2, 0.30, cfg_noclamp)
    ok("5.8 vertex_clamp_m=0.10 → 同一位置变成钳制（参数走 Config，不硬编码）",
       lp7.clamped_to_vertex is True and (lp7.x, lp7.y) == (0.2, 0.0),
       f"clamped={lp7.clamped_to_vertex} ({lp7.x!r},{lp7.y!r})")


# --------------------------------------------------------------------------- #
#  6. 舵角
# --------------------------------------------------------------------------- #
def test_steer() -> None:
    sec("6. compute_command 舵角：教科书式 / 限幅 / 速率限幅 / 倒车")
    path = straight(5.0)

    # 6a α = 0 → δ ≈ 0
    p0 = path.project(1.0, 0.0)
    c0 = compute_command(geo.Pose(1.0, 0.0, 0.0), path, p0, cfg_of())
    ok("6.1 对准前视点（α=0）→ steer_deg == 0", c0.steer_deg == 0.0 and c0.curvature == 0.0,
       f"steer={c0.steer_deg!r} curvature={c0.curvature!r} speed={c0.linear_x!r}")

    # 6b α = 20°、ld = 0.3、L = 0.157、gain = 1.0 → δ = atan(2L·sin20°/0.3)
    cfg20 = cfg_of(wheelbase_m=0.157, lookahead_min_m=0.30, lookahead_max_m=0.30,
                   curvature_gain=1.0)
    ok("6.2 前置：ld 由巡航速度与 min/max 决定 = 0.30",
       abs(lookahead_distance(cfg20.get("pursuit.cruise_speed_mps"), cfg20) - 0.30) < 1e-12,
       f"ld={lookahead_distance(0.15, cfg20)!r}")
    off = 0.30 * math.tan(math.radians(20.0))       # 横向偏置 → α=20°
    pose20 = geo.Pose(1.0, -off, 0.0)
    proj20 = path.project(pose20.x, pose20.y)
    cmd20 = compute_command(pose20, path, proj20, cfg20)
    want20 = math.degrees(math.atan(2.0 * 0.157 * math.sin(math.radians(20.0)) / 0.30))
    close("6.3 α=20°/ld=0.3/gain=1.0 → δ = degrees(atan(2·0.157·sin20°/0.3))",
          cmd20.steer_deg, want20, 1e-6, extra=f"解析值 {want20:.9f}°")
    ok("6.4 场景确为左前方 20°（车体系方位角）",
       abs(math.degrees(math.atan2(off, 0.30)) - 20.0) < 1e-9,
       f"α={math.degrees(math.atan2(off, 0.30)):.9f}° off={off!r}")

    # 6c gain = 0.5 → 逐位等于"无因子 2"的旧公式
    cfg_half = cfg_of(wheelbase_m=0.157, lookahead_min_m=0.30, lookahead_max_m=0.30,
                      curvature_gain=0.5)
    cmd_half = compute_command(pose20, path, proj20, cfg_half)
    want_half = math.degrees(math.atan(0.157 * math.sin(math.radians(20.0)) / 0.30))
    close("6.5 gain=0.5 → δ = degrees(atan(0.157·sin20°/0.3))（旧行为等价）",
          cmd_half.steer_deg, want_half, 1e-9, extra=f"解析值 {want_half:.9f}°")
    ok("6.6 gain 只缩放转向：1.0 的结果严格大于 0.5，且曲率不受 gain 影响",
       cmd20.steer_deg > cmd_half.steer_deg and cmd20.curvature == cmd_half.curvature,
       f"δ(gain1)={cmd20.steer_deg:.6f}° > δ(gain0.5)={cmd_half.steer_deg:.6f}°；"
       f"κ={cmd20.curvature!r}（与 gain 解耦）")

    # 6d 硬限幅：横向偏 1.0 m（ld=0.18 默认）→ 未限幅需求 ≈ 60° > 30°
    pose_far = geo.Pose(1.0, -1.0, 0.0)
    proj_far = path.project(pose_far.x, pose_far.y)
    cfg_def = cfg_of()
    ld_def = lookahead_distance(cfg_def.get("pursuit.cruise_speed_mps"), cfg_def)
    alpha_far = math.atan2(1.0, ld_def)
    raw_far = math.degrees(math.atan(2.0 * 0.157 * math.sin(alpha_far) / ld_def))
    cmd_far = compute_command(pose_far, path, proj_far, cfg_def)
    ok("6.7 极端 α：|steer_deg| ≤ max_steer_deg(30) 且确实被限到 30",
       abs(cmd_far.steer_deg) <= cfg_def.get("pursuit.max_steer_deg")
       and cmd_far.steer_deg == 30.0 and raw_far > 30.0,
       f"未限幅需求 {raw_far:.4f}° → 输出 {cmd_far.steer_deg!r}°（α={math.degrees(alpha_far):.2f}°）")
    cmd_far_neg = compute_command(geo.Pose(1.0, 1.0, 0.0), path, path.project(1.0, 1.0), cfg_def)
    ok("6.8 右侧目标 → 负舵角（正=左转契约）", cmd_far_neg.steer_deg == -30.0,
       f"{cmd_far_neg.steer_deg!r}°")

    # 6e 速率限幅：0 → 30°，dt=0.02，120°/s ⇒ 单拍 ≤ 2.4°
    c1 = compute_command(pose_far, path, proj_far, cfg_def, last_steer_deg=0.0, dt=0.02)
    ok("6.9 速率限幅：首拍 0→30° 只走 2.4°（120°/s × 0.02 s）",
       abs(c1.steer_deg - 2.4) < 1e-9 and abs(c1.steer_deg - 0.0) <= 120.0 * 0.02 + 1e-9,
       f"steer={c1.steer_deg!r}° Δ={c1.steer_deg - 0.0!r}° ≤ {120.0 * 0.02!r}°")
    c2 = compute_command(pose_far, path, proj_far, cfg_def, last_steer_deg=c1.steer_deg, dt=0.02)
    ok("6.10 第二拍继续爬（2.4 → 4.8°）",
       abs(c2.steer_deg - 4.8) < 1e-9, f"steer={c2.steer_deg!r}°")
    c3 = compute_command(pose_far, path, proj_far, cfg_def, last_steer_deg=29.0, dt=0.02)
    ok("6.11 接近上限时被硬限幅夹住（29 + 2.4 → 30 而不是 31.4）",
       c3.steer_deg == 30.0, f"steer={c3.steer_deg!r}°")
    c4 = compute_command(pose_far, path, proj_far, cfg_def, last_steer_deg=None)
    ok("6.12 首拍（last_steer_deg=None）不做速率限幅", c4.steer_deg == 30.0,
       f"steer={c4.steer_deg!r}°")

    # 6f 倒车：速度取负 × ratio、舵角 × reverse_steer_sign
    cfg_rev = cfg_of(wheelbase_m=0.157, lookahead_min_m=0.30, lookahead_max_m=0.30,
                     curvature_gain=1.0, reverse_steer_sign=-1.0)
    cmd_rev = compute_command(pose20, path, proj20, cfg_rev, reverse=True)
    ok("6.13 倒车：舵角符号翻转（reverse_steer_sign=-1）",
       abs(cmd_rev.steer_deg + cmd20.steer_deg) < 1e-12 and cmd_rev.steer_deg < 0.0,
       f"前进 {cmd20.steer_deg:.6f}° vs 倒车 {cmd_rev.steer_deg:.6f}°")
    ok("6.14 倒车：速度取负并乘 reverse_speed_ratio(0.6)，|v| ≥ min_speed",
       cmd_rev.linear_x < 0.0 and cmd_rev.reverse is True
       and abs(cmd_rev.linear_x + max(0.08, cmd20.linear_x * 0.6)) < 1e-12,
       f"前进 v={cmd20.linear_x!r} 倒车 v={cmd_rev.linear_x!r}（ratio=0.6）")
    ok("6.15 reverse_steer_sign=-1 只影响倒车，不影响前进",
       abs(cmd20.steer_deg - want20) < 1e-6 and cmd20.reverse is False,
       f"同 cfg 前进 {cmd20.steer_deg:.6f}° 未翻号")

    # 6g 倒车段由路线驱动（R:），调用方不传 reverse 也自动识别
    rpath = Path(route_of([(0.0, 0.0), (5.0, 0.0)], name="rev", reverses={1}))
    rproj = rpath.project(1.0, -off)
    cmd_seg = compute_command(geo.Pose(1.0, -off, 0.0), rpath, rproj, cfg_rev)
    ok("6.16 R: 倒车段：显式 reverse=False 也自动倒车（路线数据为准）",
       cmd_seg.reverse is True and cmd_seg.linear_x < 0.0 and cmd_seg.steer_deg < 0.0,
       f"reverse={cmd_seg.reverse} v={cmd_seg.linear_x!r} δ={cmd_seg.steer_deg:.6f}°")


# --------------------------------------------------------------------------- #
#  7. 速度剖面
# --------------------------------------------------------------------------- #
def test_speed_profile() -> None:
    sec("7. speed_profile：曲率/停站/转角降速 + 下界 + speed_scale + 量化补偿")
    cfg = cfg_of()
    v0, r0 = speed_profile(curvature=0.0, dist_to_stop=None, corner_dist=None,
                           speed_scale=1.0, cfg=cfg)
    close("7.1 基准：无约束 = 巡航速度", v0, 0.15, 1e-12)
    ok("7.2 基准限速主因 = cruise", r0 == "cruise", f"reason={r0!r}")

    v_curve, r_curve = speed_profile(curvature=20.0, dist_to_stop=None, corner_dist=None,
                                     speed_scale=1.0, cfg=cfg)
    ok("7.3 曲率大 → 速度下降（v = sqrt(0.35/20) = 0.132）",
       v_curve < v0 and abs(v_curve - math.sqrt(0.35 / 20.0)) < 1e-12 and r_curve == "curve",
       f"v={v_curve!r} m/s < {v0!r}（reason={r_curve!r}）")

    v_floor, r_floor = speed_profile(curvature=2000.0, dist_to_stop=None, corner_dist=None,
                                     speed_scale=1.0, cfg=cfg)
    ok("7.4 极端曲率 → 仍不低于 min_speed_mps(0.08)",
       v_floor == 0.08 and r_floor == "min_speed",
       f"v={v_floor!r}（未夹时为 {math.sqrt(0.35 / 2000.0)!r}）reason={r_floor!r}")

    v_stop, r_stop = speed_profile(curvature=0.0, dist_to_stop=0.30, corner_dist=None,
                                   speed_scale=1.0, cfg=cfg)
    ok("7.5 接近停站点 → 速度下降（v = 0.30 × d = 0.09）",
       v_stop < v0 and abs(v_stop - 0.09) < 1e-12 and r_stop == "stop_approach",
       f"v={v_stop!r} m/s（d=0.30 m）reason={r_stop!r}")
    v_far, r_far = speed_profile(curvature=0.0, dist_to_stop=0.55, corner_dist=None,
                                 speed_scale=1.0, cfg=cfg)
    ok("7.6 停站点还远（0.55 m）→ 不减速（0.30×0.55=0.165 > 巡航）",
       v_far == 0.15 and r_far == "cruise", f"v={v_far!r} reason={r_far!r}")

    v_corner, r_corner = speed_profile(curvature=0.0, dist_to_stop=None, corner_dist=0.20,
                                       speed_scale=1.0, cfg=cfg)
    ok("7.7 接近尖角（0.20 m < corner_slow_dist_m 0.45）→ 压到 corner_speed_mps 0.12",
       v_corner == 0.12 and r_corner == "corner", f"v={v_corner!r} reason={r_corner!r}")
    v_corner_far, r_corner_far = speed_profile(curvature=0.0, dist_to_stop=None,
                                               corner_dist=0.50, speed_scale=1.0, cfg=cfg)
    ok("7.8 尖角还远（0.50 m > 0.45）→ 不减速",
       v_corner_far == 0.15 and r_corner_far == "cruise", f"v={v_corner_far!r}")

    v_scale, _ = speed_profile(curvature=0.0, dist_to_stop=None, corner_dist=None,
                               speed_scale=0.8, cfg=cfg)
    close("7.9 speed_scale 生效（巡航 ×0.8 = 0.12）", v_scale, 0.12, 1e-12)
    v_scale0, _ = speed_profile(curvature=0.0, dist_to_stop=None, corner_dist=None,
                                speed_scale=0.0, cfg=cfg)
    ok("7.10 speed_scale=0 → 仍受 min_speed 下界保护", v_scale0 == 0.08,
       f"v={v_scale0!r}")

    cfg_comp = cfg_of(speed_quant_comp=1.043)
    v_comp, _ = speed_profile(curvature=0.0, dist_to_stop=None, corner_dist=None,
                              speed_scale=1.0, cfg=cfg_comp)
    close("7.11 speed_quant_comp 生效（0.15 × 1.043）", v_comp, 0.15 * 1.043, 1e-12)

    v_multi, r_multi = speed_profile(curvature=20.0, dist_to_stop=0.20, corner_dist=0.10,
                                     speed_scale=1.0, cfg=cfg)
    ok("7.12 多层同时作用取最慢（min(0.15, 0.132, 0.08, 0.12) = 0.08）",
       abs(v_multi - 0.08) < 1e-12, f"v={v_multi!r} reason={r_multi!r}")

    ok("7.13 曲率限速函数与 Config 一致（curve_speed_limit）",
       abs(curve_speed_limit(20.0, cfg) - math.sqrt(0.35 / 20.0)) < 1e-12
       and curve_speed_limit(0.0, cfg) == float("inf"),
       f"κ=20 → {curve_speed_limit(20.0, cfg)!r}；κ=0 → inf")

    # 路线驱动的 speed_scale（Waypoint.speed_scale）确实进到命令里
    r = RouteBuilder("slow")
    r.add(Waypoint(block="P0", x=0.0, y=0.0, yaw=0.0))
    r.add(Waypoint(block="P1", x=5.0, y=0.0, yaw=0.0, speed_scale=0.5))
    spath = Path(r.build())
    cmd_slow = compute_command(geo.Pose(1.0, 0.0, 0.0), spath, spath.project(1.0, 0.0), cfg)
    ok("7.14 航点 speed_scale=0.5 → 命令速度被压到 0.08（0.075 受下界保护）",
       abs(cmd_slow.linear_x - 0.08) < 1e-12 and cmd_slow.speed_reason == "min_speed",
       f"v={cmd_slow.linear_x!r} reason={cmd_slow.speed_reason!r}")

    # 终点前减速（末点也在 stop_indices 里）
    cmd_end = compute_command(geo.Pose(4.85, 0.0, 0.0), spath,
                              spath.project(4.85, 0.0), cfg)
    ok("7.15 接近末点 → 减速（d=0.15 m → 0.30×0.15=0.045 → 下界 0.08）",
       cmd_end.linear_x == 0.08 and cmd_end.speed_reason in ("stop_approach", "min_speed"),
       f"v={cmd_end.linear_x!r} reason={cmd_end.speed_reason!r}")


# --------------------------------------------------------------------------- #
#  8. 刹车
# --------------------------------------------------------------------------- #
def test_brake() -> None:
    sec("8. brake_command：零 Twist = 刹车（下位机契约）")
    cfg = cfg_of()
    b = brake_command(cfg)
    ok("8.1 linear_x == 0（零速）", b.linear_x == 0.0, f"linear_x={b.linear_x!r}")
    ok("8.2 steer_deg == 0（不许带舵角停车，否则轮子停在打死姿态）",
       b.steer_deg == 0.0, f"steer_deg={b.steer_deg!r}")
    ok("8.3 curvature == 0 / reverse False / reason='brake'",
       b.curvature == 0.0 and b.reverse is False and b.speed_reason == "brake",
       f"curvature={b.curvature!r} reverse={b.reverse} reason={b.speed_reason!r}")


# --------------------------------------------------------------------------- #
#  9. 与旧行为对拍
# --------------------------------------------------------------------------- #
def test_legacy_parity() -> None:
    sec("9. 与旧行为对拍：gain=0.5、ld=0.15、L=0.157、α=10°")
    # 旧公式（survey §2.3）：δ = degrees(atan(L·sinα / max(dist, 0.10)))，dist = 前视弦长
    # 新公式（P0 §4）：δ = degrees(atan(gain·2L·sinα / max(ld, floor)))
    #   gain = 0.5 时 0.5×2L == L（IEEE754 精确），于是"分母相同 ⇒ 逐位相同"。
    LD = 0.15
    L = 0.157
    cfg_old = cfg_of(wheelbase_m=L, lookahead_min_m=LD, lookahead_max_m=LD, curvature_gain=0.5)
    ok("9.1 前置：ld 被配置成 0.15（旧硬编码值）",
       abs(lookahead_distance(0.15, cfg_old) - LD) < 1e-12,
       f"ld={lookahead_distance(0.15, cfg_old)!r}")

    path = straight(5.0)
    off = LD * math.tan(math.radians(10.0))        # 横向偏置 → α = 10°
    pose = geo.Pose(1.0, -off, 0.0)
    proj = path.project(pose.x, pose.y)
    tgt = find_lookahead_point(path, proj, lookahead_distance(0.15, cfg_old), cfg_old)
    alpha_meas = math.atan2(tgt.y - pose.y, tgt.x - pose.x)     # yaw=0 ⇒ 车体系=世界系
    ok("9.2 前置：场景 α = 10°（实测）",
       abs(math.degrees(alpha_meas) - 10.0) < 1e-9,
       f"α={math.degrees(alpha_meas):.12f}° off={off!r} 前视点=({tgt.x!r},{tgt.y!r})")

    cmd = compute_command(pose, path, proj, cfg_old)
    want_old = math.degrees(math.atan(L * math.sin(alpha_meas) / LD))
    close("9.3 gain=0.5 -> 与旧公式 degrees(atan(L·sinα/0.15)) 数值一致（容差 1e-9）",
          cmd.steer_deg, want_old, 1e-9, extra=f"旧公式值 {want_old:.12f}°")
    want_old_lit = math.degrees(math.atan(L * math.sin(math.radians(10.0)) / LD))
    close("9.4 与 α 取字面量 10° 的旧公式一致（容差 1e-6）",
          cmd.steer_deg, want_old_lit, 1e-6, extra=f"{want_old_lit:.12f}°")

    cfg_new = cfg_of(wheelbase_m=L, lookahead_min_m=LD, lookahead_max_m=LD, curvature_gain=1.0)
    cmd_new = compute_command(pose, path, proj, cfg_new)
    want_new = math.degrees(math.atan(2.0 * L * math.sin(alpha_meas) / LD))
    close("9.5 gain=1.0 -> 教科书式（多出的因子 2 真的生效）", cmd_new.steer_deg, want_new, 1e-9,
          extra=f"{want_new:.12f}°（≈ 旧值 {want_old:.6f}° 的 1.94 倍）")
    ok("9.6 新旧关系：gain=1.0 的舵角严格大于 gain=0.5（同 α 同 ld）",
       cmd_new.steer_deg > cmd.steer_deg,
       f"{cmd_new.steer_deg:.9f}° > {cmd.steer_deg:.9f}°")

    # 倒车时"舵角同式"：只是符号系数，公式本体不变
    cfg_rev = cfg_of(wheelbase_m=L, lookahead_min_m=LD, lookahead_max_m=LD,
                     curvature_gain=0.5, reverse_steer_sign=1.0)
    cmd_rev = compute_command(pose, path, proj, cfg_rev, reverse=True)
    ok("9.7 倒车舵角同式（reverse_steer_sign=1.0 时与前进完全同值）",
       cmd_rev.steer_deg == cmd.steer_deg and cmd_rev.linear_x == -max(0.08, cmd.linear_x * 0.6),
       f"倒车 δ={cmd_rev.steer_deg!r}° == 前进 δ={cmd.steer_deg!r}°；v={cmd_rev.linear_x!r}")


# --------------------------------------------------------------------------- #
#  汇总
# --------------------------------------------------------------------------- #
def main() -> int:
    print("=" * 72)
    print("dzbrain pursuit 核心自测（纯脚本）")
    print("=" * 72)
    t0 = time.perf_counter()
    test_coordinates()
    test_project_basic()
    test_index_window_and_perf()
    test_allow_backward()
    test_lookahead()
    test_steer()
    test_speed_profile()
    test_brake()
    test_legacy_parity()
    dt = time.perf_counter() - t0

    us_win = MEASURED.get("us_win", float("nan"))
    us_full = MEASURED.get("us_full", float("nan"))
    print()
    print("=" * 72)
    print(f"断言：PASS {_PASS} / FAIL {_FAIL}（共 {_PASS + _FAIL} 条，耗时 {dt:.2f} s）")
    print(f"投影实测（499 段蛇形，s_hint 窗口）：{us_win:.2f} us/次")
    print(f"投影实测（同路径，s_hint=None 全量）：{us_full:.2f} us/次"
          f"（窗口快 {us_full / us_win:.0f} 倍）")
    print("注：block_to_meters('N10').x = 0.5249999999999999，与十进制字面量 0.525 差 1 ULP；"
          "与旧公式逐位一致（见 1.2）")
    if _FAIL:
        print(f"失败用例：{', '.join(_FAILED)}")
        print("结果：FAILED")
        return 1
    print("结果：全部通过 (ALL PASS)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
