# -*- coding: utf-8 -*-
"""物理几何基础自测（t22）—— 用户 2026-10-11 现场口径。

覆盖：
 1. 车体与扫掠半径（含与 `pursuit.*` 同源的最小转弯半径）
 2. **压雷判据**：车心 3.5 cm 圆 ∩ 0.12×0.12 垫片（含边界：0.095 压到 / 0.096 压不到）
 3. 障碍圆柱净空（直段 0.215 / 拐角按扫掠圆 0.3029）
 4. 挡板 2 cm 细条（中心线 x=1.75、长 3.85 m、门槛 0.0775）+ 桥洞 K 处精确抠段
 5. `config/robot.yaml` 与 `core/config.py` 的键值一致（数值只许在 config 里出现一次）
 6. **对拍** `tools/route_check.py` 的同一套公式（用户侧工具刻意不 import dzbrain，
    两份实现必须不分叉 —— 这条断言是唯一的防分叉机制）

运行：python src\\dzbrain\\tests\\test_physics.py
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve()
_TESTS = _HERE.parent


def _find_src() -> pathlib.Path:
    for parent in _HERE.parents:
        cand = parent / "src" / "dzbrain"
        if (cand / "dzbrain" / "__init__.py").is_file():
            return cand
    return _HERE.parent


_SRC = _find_src()
for _p in (str(_SRC), str(_TESTS)):
    if _p not in sys.path:
        sys.path.insert(0, _p)
_WS = _SRC.parents[1]

from harness import check, check_close, check_raises, section, finish   # noqa: E402

from dzbrain.core.config import Config                                  # noqa: E402
from dzbrain.core import geometry as geo                                # noqa: E402
from dzbrain.route import physics as P                                  # noqa: E402

_CFG = Config.load(str(_WS / "config" / "robot.yaml"))
L = _CFG.get("vehicle.length_m")
W = _CFG.get("vehicle.width_m")
R_COV = _CFG.get("vehicle.coverage_radius_m")
R_OBS = _CFG.get("field.obstacle_radius_m")
PAD = _CFG.get("field.mine_pad_m")
B_W = _CFG.get("field.barrier_width_m")
MARGIN = _CFG.get("planner.obstacle_margin_m")


def _load_route_check():
    """用户侧工具按模块加载（与本文件同一套公式的**第二份实现**，用于对拍）。"""
    spec = importlib.util.spec_from_file_location("rc_for_physics", _WS / "tools" / "route_check.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


RC = _load_route_check()


# --------------------------------------------------------------------------- #
#  1. 车体与扫掠半径
# --------------------------------------------------------------------------- #
def test_vehicle_and_sweep() -> None:
    section("1. 车体（0.28 × 0.135）与扫掠半径")
    check_close("车体半对角线 = √(0.14²+0.0675²) = 0.15542 m",
                P.vehicle_half_diagonal_m(L, W), 0.15542, 1e-5)
    sweep, turn_r = P.sweep_radius(length_m=L, width_m=W,
                                   wheelbase_m=_CFG.get("pursuit.wheelbase_m"),
                                   max_steer_deg=_CFG.get("pursuit.max_steer_deg"))
    check_close("扫掠半径 = 半对角线", sweep, 0.15542, 1e-5)
    check_close("最小转弯半径 R = 轴距/tan(30°) = 0.27193 m", turn_r, 0.27193, 1e-5)
    check("★ 扫掠半径(0.1554) 远大于车半宽(0.0675)：拐角判据不能用半宽",
          sweep > W / 2.0 + 0.08, f"sweep={sweep:.4f} half_w={W/2:.4f}")
    check("R_min(0.272) > 半格(0.175)（这就是「到不了格心」的几何根因）",
          turn_r > 0.35 / 2.0, f"R={turn_r:.4f}")
    check_raises("非法舵角上限（0°）抛 ValueError", ValueError,
                 P.sweep_radius, max_steer_deg=0.0)


# --------------------------------------------------------------------------- #
#  2. 压雷判据（车心 3.5 cm 圆 ∩ 0.12 垫片）
# --------------------------------------------------------------------------- #
def test_mine_criterion() -> None:
    section("2. 压雷判据：车心 3.5 cm 圆 ∩ 0.12×0.12 垫片")
    mx, my = 0.875, 1.225                      # K9 格心（现场雷也有在格心的）
    check("车心正好在垫片中心 ⇒ 距离 0、判压到",
          P.mine_clearance_m(mx, my, mx, my, mine_pad_m=PAD) == 0.0
          and P.mine_pressed(mx, my, mx, my, coverage_radius_m=R_COV, mine_pad_m=PAD))
    # 轴向：垫片半宽 0.06 ⇒ 0.095 处距离恰为 0.035（阈值上）
    check_close("轴向偏 0.095 m ⇒ 到垫片 0.0350 m（阈值边界）",
                P.mine_clearance_m(mx + 0.095, my, mx, my, mine_pad_m=PAD), 0.035, 1e-9)
    check("轴向偏 0.095 m ⇒ **判压到**（≤ 0.035）",
          P.mine_pressed(mx + 0.095, my, mx, my, coverage_radius_m=R_COV, mine_pad_m=PAD))
    check("轴向偏 0.096 m ⇒ **判没压到**（> 0.035）",
          not P.mine_pressed(mx + 0.096, my, mx, my, coverage_radius_m=R_COV, mine_pad_m=PAD))
    # 进入垫片内：距离 0
    check("车心在垫片内（偏 0.05 m）⇒ 距离 0、判压到",
          P.mine_clearance_m(mx + 0.05, my, mx, my, mine_pad_m=PAD) == 0.0
          and P.mine_pressed(mx + 0.05, my, mx, my, coverage_radius_m=R_COV, mine_pad_m=PAD))
    # ★ 与"到格心 ≤ 3.5 cm"的错口径对照（本模块存在的理由）
    dx, dy = 0.09, 0.0
    naive = math.hypot(dx, dy) <= R_COV
    proper = P.mine_pressed(mx + dx, my + dy, mx, my, coverage_radius_m=R_COV, mine_pad_m=PAD)
    check("★ 反例：轴向 0.09 m 处两者结论相反（贴边压到，但离格心 9 cm）",
          proper and not naive,
          f"到垫片 {P.mine_clearance_m(mx+dx, my, mx, my, mine_pad_m=PAD):.4f} m 判压到；"
          f"到格心 {math.hypot(dx,dy):.4f} m > {R_COV} 会被错口径判成没压到")
    # 线段口径
    check("横穿垫片的航段 ⇒ 距离 0",
          P.mine_clearance_segment_m(mx - 0.3, my, mx + 0.3, my, mx, my, mine_pad_m=PAD) == 0.0)
    check_close("平行擦过（横向偏 0.09 m）⇒ 段到垫片 0.03 m",
                P.mine_clearance_segment_m(mx - 0.3, my + 0.09, mx + 0.3, my + 0.09,
                                           mx, my, mine_pad_m=PAD), 0.03, 1e-9)
    check("平行擦过（横向偏 0.10 m）⇒ 0.04 m > 0.035 ⇒ 段判**没压到**",
          P.mine_clearance_segment_m(mx - 0.3, my + 0.10, mx + 0.3, my + 0.10,
                                     mx, my, mine_pad_m=PAD) > R_COV)


# --------------------------------------------------------------------------- #
#  3. 障碍圆柱净空
# --------------------------------------------------------------------------- #
def test_obstacle_clearance() -> None:
    section("3. 障碍圆柱（r=0.1275，柱心在格心）净空")
    req_straight = P.required_obstacle_center_distance_m(obstacle_radius_m=R_OBS,
                                                        body_radius_m=W / 2.0,
                                                        margin_m=MARGIN)
    req_corner = P.required_obstacle_center_distance_m(
        obstacle_radius_m=R_OBS,
        body_radius_m=P.vehicle_half_diagonal_m(L, W), margin_m=MARGIN)
    check_close("直段门槛 = 0.1275+0.0675+0.02 = 0.215 m", req_straight, 0.215, 1e-9)
    check_close("拐角门槛 = 0.1275+0.15542+0.02 = 0.3029 m", req_corner, 0.30292, 1e-5)
    check("拐角门槛比直段严 0.088 m（扫掠半径 − 车半宽）",
          req_corner - req_straight > 0.08, f"{req_corner:.4f} - {req_straight:.4f}")
    ox, oy = 0.525, 2.975
    check_close("点到柱心距离 = 欧氏距离",
                P.obstacle_clearance_point_m(ox + 0.3, oy + 0.4, ox, oy), 0.5, 1e-12)
    check_close("段到柱心：整段离柱心 0.35 m ⇒ 0.35",
                P.obstacle_clearance_segment_m(ox - 0.35, oy + 0.35, ox + 0.35, oy + 0.35,
                                               ox, oy), 0.35, 1e-12)
    check_close("段端点正压柱心外侧 ⇒ 交于柱心竖线上仍算垂直距离",
                P.obstacle_clearance_segment_m(ox - 0.05, oy + 0.05, ox + 0.05, oy - 0.05,
                                               ox, oy), 0.0, 1e-12)


# --------------------------------------------------------------------------- #
#  4. 挡板 2 cm 细条
# --------------------------------------------------------------------------- #
def test_barrier_criterion() -> None:
    section("4. 挡板（第 6/7 行之间、D..N、2 cm 细条）")
    cf, ct = P.parse_col_span(_CFG.get("field.barrier_cols"))
    check("`field.barrier_cols` 解析成 (D, N)", (cf, ct) == ("D", "N"), f"{(cf, ct)}")
    x, y0, y1 = P.barrier_centerline(col_from=cf, col_to=ct, row_hi=6, row_lo=7)
    check_close("中心线 x = 1.75 m", x, 1.75, 1e-12)
    check_close("y 覆盖 0.00 ~ 3.85 m（11 列 × 0.35）", y1 - y0, 3.85, 1e-12)
    check("`parse_col_span` 也吃单列写法", P.parse_col_span("K") == ("K", "K"),
          str(P.parse_col_span("K")))
    req = P.required_barrier_distance_m(barrier_width_m=B_W, car_width_m=W)
    check_close("触线门槛 = 0.01 + 0.0675 = 0.0775 m", req, 0.0775, 1e-12)
    # 行 7 格心（x=1.575）到中心线 1.75 ⇒ 0.175 m > 0.0775 ⇒ 安全
    check_close("行 7 格心上行驶 ⇒ 0.175 m（安全）",
                P.barrier_distance_point_m(1.575, 1.0, x=x, y0=y0, y1=y1), 0.175, 1e-12)
    check("贴到中心线 0.05 m ⇒ 触线（< 0.0775）",
          P.barrier_distance_point_m(x + 0.05, 1.0, x=x, y0=y0, y1=y1) < req)
    check("中心线上（x=1.75）⇒ 0 m ⇒ 触线",
          P.barrier_distance_point_m(x, 1.0, x=x, y0=y0, y1=y1) == 0.0)
    check("挡板**端点之外**（y=4.025 即 C 列心）⇒ 0.175 m（走缺口不算触线）",
          abs(P.barrier_distance_point_m(x, 4.025, x=x, y0=y0, y1=y1) - 0.175) < 1e-12)
    # 桥洞豁免：精确抠段
    pieces = P.segment_outside_band(x, 4.025, x, 1.0, band_y=1.225, band_half=0.175)
    check("桥洞带（K 列 y=1.225±0.175）内的部分被抠掉 ⇒ 剩 2 截",
          len(pieces) == 2, str(pieces))
    check("抠完的第一截止于带的上沿 y=1.400",
          abs(pieces[0][3] - 1.400) < 1e-9, f"y={pieces[0][3]:.4f}")
    check("带内（y=1.225）的点不会被留成「待检」段",
          all(not (1.05 < p[1] < 1.40 and 1.05 < p[3] < 1.40) for p in pieces), str(pieces))


# --------------------------------------------------------------------------- #
#  5. config 键值单一真源
# --------------------------------------------------------------------------- #
def test_config_values() -> None:
    section("5. `config/robot.yaml` + SCHEMA：数值只出现一次")
    want = {"vehicle.length_m": 0.28, "vehicle.width_m": 0.135,
            "vehicle.coverage_radius_m": 0.035, "field.obstacle_radius_m": 0.1275,
            "field.mine_pad_m": 0.12, "field.barrier_width_m": 0.02,
            "planner.obstacle_margin_m": 0.02, "pursuit.cruise_speed_mps": 0.15,
            "path_executor.linear_speed": 0.15, "dzactuator.ticksPer2PI": 2680.0,
            "dzactuator.ticksPerMeter": 3280.0,
            # ★ 卡 t11：现场卷尺实测的参考点偏移（后轴中心 → 车体几何中心）
            "vehicle.ref_offset_m": 0.075}
    for k, v in want.items():
        check_close(f"{k} = {v}", _CFG.get(k), v, 1e-12)
    check("`field.barrier_cols` = D..N", str(_CFG.get("field.barrier_cols")) == "D..N",
          repr(_CFG.get("field.barrier_cols")))
    issues = [i for i in _CFG.validate(strict=False) if i.level == "error"]
    check("真实配置无 error（新键已进 SCHEMA）", not issues, "; ".join(str(i) for i in issues))
    check("新键可读（SCHEMA 注册过，不是「只有 YAML 有、代码读不到」）",
          all(_CFG.get(k) is not None for k in want), "8 个新键 + 3 个旧键全部可读")
    # ★ 卡 t11：**YAML 真源 ↔ SCHEMA 默认值**必须一致，且要按 YAML 里的**段前缀键路径**读得到。
    #   为什么单列这一条：`vehicle.*` 在 YAML 里写在 `/dzactuator: ros__parameters: vehicle:`，
    #   flatten 之后是 `dzactuator.vehicle.*`；而 `cfg.get("vehicle.length_m", 0.28)` 这种带非 None
    #   默认值的写法在键不在 `_flat` 时**直接返回默认值**（读不到现场改动）—— 偏移这个量必须
    #   确保走的是"读得到 YAML"的那条路（`tasks/mission.py: vehicle_geometry()`）。
    import yaml                                                    # noqa: PLC0415
    _doc = yaml.safe_load((_WS / "config" / "robot.yaml").read_text(encoding="utf-8"))
    _veh = _doc["/dzactuator"]["ros__parameters"]["vehicle"]
    check("YAML 里 `dzactuator.vehicle.ref_offset_m` 与 SCHEMA 默认值逐位一致（单一真源）",
          abs(float(_veh["ref_offset_m"]) - float(_CFG.get("vehicle.ref_offset_m"))) < 1e-12
          and "dzactuator.vehicle.ref_offset_m" in list(_CFG.keys()),
          f"yaml={_veh['ref_offset_m']} / SCHEMA={_CFG.get('vehicle.ref_offset_m')}")


# --------------------------------------------------------------------------- #
#  6. 对拍 route_check（两份实现不许分叉）
# --------------------------------------------------------------------------- #
def test_cross_check_route_check() -> None:
    section("6. 对拍 `tools/route_check.py`：同一套公式、两份实现")
    check("route_check 读到的物理常量与本模块一致（8 项）",
          abs(RC.VEHICLE_LENGTH_M - L) < 1e-12 and abs(RC.VEHICLE_WIDTH_M - W) < 1e-12
          and abs(RC.COVERAGE_RADIUS_M - R_COV) < 1e-12 and abs(RC.OBSTACLE_RADIUS_M - R_OBS) < 1e-12
          and abs(RC.MINE_PAD_M - PAD) < 1e-12 and abs(RC.BARRIER_WIDTH_M - B_W) < 1e-12
          and abs(RC.OBSTACLE_MARGIN_M - MARGIN) < 1e-12
          and str(RC.BARRIER_COLS) == str(_CFG.get("field.barrier_cols")),
          f"{RC.VEHICLE_LENGTH_M}/{RC.VEHICLE_WIDTH_M}/{RC.COVERAGE_RADIUS_M}/"
          f"{RC.OBSTACLE_RADIUS_M}/{RC.MINE_PAD_M}/{RC.BARRIER_WIDTH_M}/"
          f"{RC.OBSTACLE_MARGIN_M}/{RC.BARRIER_COLS}")
    check("挡板中心线一致",
          RC.barrier_centerline() == P.barrier_centerline(col_from="D", col_to="N"),
          f"{RC.barrier_centerline()} vs {P.barrier_centerline(col_from='D', col_to='N')}")
    check_close("车体半对角线一致（route_check 的函数名同义）",
                RC.vehicle_half_diagonal_m(L, W), P.vehicle_half_diagonal_m(L, W), 1e-12)
    # ★ 用**现役路线的真实几何**做 8 个雷的逐一对拍（比构造折线有意义得多）：
    #   现场路线是"雷心直线穿越"⇒ 每个雷的"路线↔垫片"距离应为 **0**。
    import yaml
    from dzbrain.core import geometry as geo
    from dzbrain.route.parser import parse_mission
    mines = [str(m) for m in
             ((yaml.safe_load((_WS / "config" / "map.yaml").read_text(encoding="utf-8")) or {})
              .get("map") or {}).get("mines") or []]
    live = [str(t) for t in _CFG.get("route.tokens") or []]
    pts = [geo.block_to_meters(b) for b in parse_mission(live).blocks()]
    check("现役路线的雷表非空（对拍样本）", len(mines) == 8, f"{len(mines)} 个雷")
    for mine in mines:
        a = RC.mine_clearance_by_route(pts, mine)
        mx, my = RC.block_to_meters(mine)
        b = min(P.mine_clearance_segment_m(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1],
                                           mx, my, mine_pad_m=PAD)
                for i in range(len(pts) - 1))
        check_close(f"雷 {mine}：两份实现一致且 = 0（雷心直线穿越）", a, b, 1e-12)
        check(f"雷 {mine} 被车心 3.5 cm 圆压到（距离 {a:.4f} ≤ {R_COV}）", a <= R_COV + 1e-12)
    dx, dy = 0.11, 0.03
    a = RC._phys_seg_rect_dist(0.0, 0.0, 1.0, 0.0, 0.5, 0.0, PAD / 2)
    b = P.segment_rect_distance_m(0.0, 0.0, 1.0, 0.0, 0.5, 0.0, PAD / 2)
    check_close("段↔矩形（横穿）两份实现都是 0", a, b, 1e-12)
    check("横穿垫片的段距离 = 0（不是「离最近的角还有 0.06」）", a == 0.0, f"got {a}")
    a2 = RC._phys_seg_rect_dist(0.0, 0.09, 1.0, 0.09, 0.5, 0.0, PAD / 2)
    b2 = P.segment_rect_distance_m(0.0, 0.09, 1.0, 0.09, 0.5, 0.0, PAD / 2)
    check_close("段↔矩形（平行擦过 0.03）两份实现一致", a2, b2, 1e-12)
    check("抠段（桥洞豁免）两份实现一致",
          len(RC.seg_outside_band(1.75, 4.025, 1.75, 1.0, 1.225, 0.175))
          == len(P.segment_outside_band(1.75, 4.025, 1.75, 1.0, band_y=1.225, band_half=0.175))
          == 2)


# --------------------------------------------------------------------------- #
#  7. 用户 2026-10-11 追加口径（船长转达）：靶板 35×35 / K 列纵向封死 / 挡板含 K 段
# --------------------------------------------------------------------------- #
def test_amended_field_spec() -> None:
    section("7. 追加口径：靶板 35×35（格级=物理精确）/ K 列纵向封死 / 挡板 D..N 含 K 段")
    check_close("靶板半边长 = 半格 0.175 m（35×35cm 与格对齐）", P.board_half_m(), 0.175, 1e-12)
    check("★ 靶板与格逐个重合 ⇒ 规划器的「靶格不可通行」格级挡块 = 物理精确（澄清③）",
          abs(P.board_half_m() - P.CELL_M / 2.0) < 1e-12, "半格口径")
    req_s = P.required_target_body_clearance_m(length_m=L, width_m=W, at_corner=False)
    req_c = P.required_target_body_clearance_m(length_m=L, width_m=W, at_corner=True)
    check_close("压靶（车体）直段门槛 = 车半宽 0.0675 m", req_s, 0.0675, 1e-12)
    check_close("压靶（车体）拐角门槛 = 车体半对角线 0.15542 m", req_c, 0.15542, 1e-5)
    check("车体压到靶板（间隙 0.05 < 0.0675）⇒ 判 overlap",
          P.target_body_overlap(0.05, length_m=L, width_m=W))
    check("车体没压到靶板（间隙 0.20 > 0.1554 拐角门槛）⇒ 不 overlap",
          not P.target_body_overlap(0.20, length_m=L, width_m=W, at_corner=True))
    check("★ K 列纵向相邻格全部封死（澄清②）—— 由 validate 的 R009「列内」硬拦兑现",
          P.k_cells_sealed_along_column() is True, "见 physics.k_cells_sealed_along_column 说明")
    from dzbrain.route import validate as V
    from dzbrain.route.parser import parse_mission as _pm
    import yaml as _yaml
    _map = _yaml.safe_load((_WS / "config" / "map.yaml").read_text(encoding="utf-8"))
    _iss = V.validate_route(_pm(["A8", "B8", "C8", "D8", "E9", "I9", "J9", "K9", "K10", "L10"]),
                            map_cfg=_map, cfg=_CFG)
    check("K9→K10（列内挪一格）被 R009「列内」硬拦（与澄清②一致）",
          any(i.code == "R009" and "列内" in i.msg for i in _iss),
          str([i.msg for i in _iss if i.code == "R009"]))
    # 挡板：D..N 含 K 段、无豁免 ⇒ 现场路线仍 0 风险（实测 0.175 ≥ 0.0775）
    from dzbrain.core import geometry as _geo
    _live_pts = [_geo.block_to_meters(b) for b in _pm(
        [str(t) for t in _CFG.get("route.tokens") or []]).blocks()]
    _bmin, _bidx = RC.barrier_touch_min(_live_pts)
    check(f"挡板（D..N 含 K 段、**无豁免**）：现役路线最近 {_bmin:.4f} m ≥ 0.0775 m",
          _bmin >= 0.0775 - 1e-9, f"航段#{_bidx}")
    # 车体压靶（追加判据）在现役路线上的实测值
    _req_s = RC.VEHICLE_WIDTH_M / 2.0
    _req_c = RC.vehicle_half_diagonal_m(RC.VEHICLE_LENGTH_M, RC.VEHICLE_WIDTH_M)
    _bs, _bt = 9e9, 9e9
    for _t in [str(x) for x in (_yaml.safe_load((_WS / "config" / "map.yaml")
                                                .read_text(encoding="utf-8")))["map"]["targets"]]:
        _tx, _ty = _geo.block_to_meters(_t)
        for _i in range(len(_live_pts) - 1):
            _bs = min(_bs, P.segment_rect_distance_m(_live_pts[_i][0], _live_pts[_i][1],
                                                     _live_pts[_i + 1][0], _live_pts[_i + 1][1],
                                                     _tx, _ty, P.board_half_m()))
        for _i in range(1, len(_live_pts) - 1):
            _bt = min(_bt, P.point_rect_distance_m(_live_pts[_i][0], _live_pts[_i][1],
                                                   _tx, _ty, P.board_half_m()))
    check(f"现役路线车体压靶：直段 {_bs:.4f} ≥ {_req_s:.4f} m（车半宽）",
          _bs >= _req_s - 1e-9, f"{_bs:.4f}")
    check(f"现役路线车体压靶：拐角 {_bt:.4f} ≥ {_req_c:.4f} m（半对角线，余量 {_bt - _req_c:+.4f}）",
          _bt >= _req_c - 1e-9, f"{_bt:.4f}")


def test_ref_offset_footprint() -> None:
    """★ 卡 t11：**参考点偏移**感知的车体足迹（`body_*`）与实测状态钉死。"""
    section("7. 参考点偏移（后轴中心 → 车体几何中心 +7.5 cm）")
    OFF = float(_CFG.get("vehicle.ref_offset_m"))
    check_close("SCHEMA 默认偏移 = 现场实测 +7.5 cm", OFF, 0.075, 1e-12)
    check_close("默认偏移常量 = 0.0（不读配置的调用方行为不变）", P.DEFAULT_REF_OFFSET_M, 0.0, 1e-12)

    # ---- 几何：中心沿车头方向平移 ----
    check("offset=0 时车体中心 == 参考点（老行为）",
          P.body_center(1.234, 5.678, 0.9, offset_m=0.0) == (1.234, 5.678), "同点")
    check_close("yaw=0、offset=+7.5cm ⇒ 中心 x 前移 0.075",
                P.body_center(1.0, 2.0, 0.0, offset_m=OFF)[0], 1.075, 1e-12)
    check_close("yaw=90°、offset=+7.5cm ⇒ 中心 y 前移 0.075",
                P.body_center(1.0, 2.0, math.pi / 2, offset_m=OFF)[1], 2.075, 1e-12)
    check("offset=0 的四角 == 以参考点为心的旋转矩形（与 sim.body_corners 口径一致）",
          all(abs(a[0] - b[0]) < 1e-12 and abs(a[1] - b[1]) < 1e-12
              for a, b in zip(P.body_corners(0.0, 0.0, 0.0),
                              ((0.14, 0.0675), (0.14, -0.0675),
                               (-0.14, -0.0675), (-0.14, 0.0675)))),
          str(P.body_corners(0.0, 0.0, 0.0)))
    check("offset=+7.5cm 的四角 == 整体沿 +x 平移 0.075（yaw=0）",
          all(abs(a[0] - (b[0] + 0.075)) < 1e-12 and abs(a[1] - b[1]) < 1e-12
              for a, b in zip(P.body_corners(0.0, 0.0, 0.0, offset_m=OFF),
                              P.body_corners(0.0, 0.0, 0.0))), "平移")

    # ---- 多边形判据（SAT / 最近距离）自洽 ----
    check_close("重叠深度：两块 0.2 方块错开 0.15 ⇒ 0.05",
                P.polygon_overlap_depth(P.corner_poly(0, 0, 0.1, 0.1),
                                        P.corner_poly(0.15, 0, 0.1, 0.1)), 0.05, 1e-12)
    check("分离 ⇒ 重叠深度恒 0（不能用「中心在不在里面」代替）",
          P.polygon_overlap_depth(P.corner_poly(0, 0, 0.1, 0.1),
                                  P.corner_poly(0.3, 0, 0.1, 0.1)) == 0.0, "0")
    check_close("分离 ⇒ 最近距离 0.10",
                P.polygon_distance_m(P.corner_poly(0, 0, 0.1, 0.1),
                                     P.corner_poly(0.3, 0, 0.1, 0.1)), 0.10, 1e-12)
    _rect_c = P.body_corners(2.0, 2.0, 0.0)
    _edges_min = min(P.segment_rect_distance_m(_rect_c[_i][0], _rect_c[_i][1],
                                               _rect_c[(_i + 1) % 4][0],
                                               _rect_c[(_i + 1) % 4][1],
                                               2.0, 3.0, 0.175) for _i in range(4))
    check("与既有基元不分叉：offset=0 的 body↔矩形 ==「车体 4 边 → segment_rect_distance」",
          abs(P.body_rect_clearance_m(2.0, 2.0, 0.0, rect_cx=2.0, rect_cy=3.0,
                                      rect_half_x=0.175, rect_half_y=0.175)
              - _edges_min) < 1e-12, f"{_edges_min:.4f}")

    # ---- ★ 实测状态钉死（真值轨迹里挡板最紧/首个侵入那一拍，见 _tmp_session/t11/out/）
    #  tracker-safe @0.30 的第 1466 拍：x=1.53733, y=1.65956, yaw=43.21°（J7 拐角切内）
    _x, _y, _yaw = 1.5373257425306674, 1.6595578010558152, math.radians(43.21046949591248)
    _strip = P.corner_poly(1.75, (0.0 + 3.85) / 2.0, 0.02 / 2.0, 3.85 / 2.0)
    _p0 = P.body_corners(_x, _y, _yaw, length_m=L, width_m=W, offset_m=0.0)
    _p1 = P.body_corners(_x, _y, _yaw, length_m=L, width_m=W, offset_m=OFF)
    check_close("同一拍：offset=0 ⇒ 挡板余量 +0.0544 m（碰不到）",
                P.polygon_distance_m(_p0, _strip), 0.0544, 1e-4)
    check_close("同一拍：offset=+7.5cm ⇒ 车体角点侵入挡板 0.24 mm（深度）",
                P.polygon_overlap_depth(_p1, _strip), 0.000243, 2e-5)
    check("翻转偏移符号（-7.5 cm）⇒ 余量变大（证明符号约定没写反）",
          P.polygon_distance_m(P.body_corners(_x, _y, _yaw, length_m=L, width_m=W,
                                              offset_m=-OFF), _strip) > 0.05, "远离挡板")
    check_close("body_barrier_clearance_m 与多边形判据同源（同一拍 offset=0）",
                P.body_barrier_clearance_m(_x, _y, _yaw, length_m=L, width_m=W, offset_m=0.0),
                0.0544, 1e-4)

    # ---- 射击停站 F4 正对靶板（E4+E5 合并 35×70）：offset 把它从"差 3.5 cm"变成"压上" ----
    _fx, _fy, _fyaw = 2.625, 2.975, math.pi / 2          # F4 格心，朝 +y（正对靶板）
    _kw = dict(rect_cx=2.45, rect_cy=3.325, rect_half_x=0.35, rect_half_y=0.175,
               length_m=L, width_m=W)
    check_close("F4 停站 offset=0 ⇒ 车头离靶板 +0.0350 m",
                P.body_rect_clearance_m(_fx, _fy, _fyaw, offset_m=0.0, **_kw), 0.035, 1e-9)
    check("F4 停站 offset=+7.5cm ⇒ 车头压进靶板（叠合深度 = 0.04 m）",
          abs(P.polygon_overlap_depth(
              P.body_corners(_fx, _fy, _fyaw, length_m=L, width_m=W, offset_m=OFF),
              P.corner_poly(2.45, 3.325, 0.35, 0.175)) - 0.04) < 1e-9, "0.04 m")

    # ---- 障碍：车体矩形 ↔ 圆柱（t5 口径：老口径漏判"斜着蹭柱面"） ----
    _cx, _cy = geo.block_to_meters("C8")
    check_close("柱心在车体内 ⇒ 间隙 = −r（真重叠，不是 0）",
                P.body_obstacle_clearance_m(_cx, _cy, 0.0, _cx, _cy,
                                            obstacle_radius_m=R_OBS,
                                            length_m=L, width_m=W, offset_m=OFF),
                -R_OBS, 1e-12)
    _g_axis = P.body_obstacle_clearance_m(_cx, _cy + 0.5, math.pi / 2, _cx, _cy,
                                          obstacle_radius_m=R_OBS, length_m=L, width_m=W,
                                          offset_m=OFF)
    _g_diag = P.body_obstacle_clearance_m(_cx, _cy + 0.5, math.pi / 4, _cx, _cy,
                                          obstacle_radius_m=R_OBS, length_m=L, width_m=W,
                                          offset_m=OFF)
    check("斜置车体的柱面间隙比正对更小（旋转角点更靠外，老口径看不到）",
          _g_diag < _g_axis, f"{_g_diag:.4f} < {_g_axis:.4f}")


def main() -> None:
    print("=" * 74)
    print("dzbrain 物理几何基础自测（t22：0.28×0.135 车体 / r=0.1275 障碍 / 0.12 垫片 / 2cm 挡板）")
    print(f"python {sys.version.split()[0]}   工作区 {_WS}")
    print("=" * 74)
    test_vehicle_and_sweep()
    test_mine_criterion()
    test_obstacle_clearance()
    test_barrier_criterion()
    test_config_values()
    test_cross_check_route_check()
    test_amended_field_spec()
    test_ref_offset_footprint()
    finish()


if __name__ == "__main__":
    main()
