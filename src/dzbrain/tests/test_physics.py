# -*- coding: utf-8 -*-
"""物理几何基础自测（t22）—— 用户 2026-10-11 现场口径。

覆盖：
 1. 车体与扫掠半径（含与 `pursuit.*` 同源的最小转弯半径）
 2. **压雷判据**：车心 3.5 cm 圆 ∩ 0.12×0.12 垫片（含边界：0.095 压到 / 0.096 压不到）
 3. 障碍圆柱净空（直段 0.2325 / 拐角按扫掠圆 0.3113）
 4. 挡板 2 cm 细条（中心线 x=1.75、长 3.85 m、门槛 0.095）+ 桥洞 K 处精确抠段
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
    section("1. 车体（0.28 × 0.17）与扫掠半径")
    check_close("车体半对角线 = √(0.14²+0.085²) = 0.16378 m",
                P.vehicle_half_diagonal_m(L, W), 0.16378, 1e-5)
    sweep, turn_r = P.sweep_radius(length_m=L, width_m=W,
                                   wheelbase_m=_CFG.get("pursuit.wheelbase_m"),
                                   max_steer_deg=_CFG.get("pursuit.max_steer_deg"))
    check_close("扫掠半径 = 半对角线", sweep, 0.16378, 1e-5)
    check_close("最小转弯半径 R = 轴距/tan(30°) = 0.27193 m", turn_r, 0.27193, 1e-5)
    # ★ 2026-10-27 真值：车宽 0.135→0.17 后 sweep−半宽 = 0.1638−0.085 = 0.0788
    #   （旧断言写死 `> W/2 + 0.08` 会以 1.2 mm 之差失败）。**意图不变**（扫掠圆远大于半宽，
    #   拐角不能用半宽），判据改成**比值**，避免再被绝对毫米数钉死。
    check("★ 扫掠半径(0.1638) 远大于车半宽(0.085)：拐角判据不能用半宽",
          sweep >= 1.8 * (W / 2.0), f"sweep={sweep:.4f} half_w={W/2:.4f} ratio={sweep/(W/2):.2f}")
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
    check_close("直段门槛 = 0.1275+0.085+0.02 = 0.2325 m", req_straight, 0.2325, 1e-9)
    check_close("拐角门槛 = 0.1275+0.16378+0.02 = 0.3113 m", req_corner, 0.31128, 1e-5)
    # 意图：拐角门槛必须**显著**更严。真值差 = 0.16378 − 0.085 = 0.0788 m
    # （旧写死 `> 0.08` 在车宽 0.17 下差 1.2 mm 即失败）。
    check("拐角门槛比直段严 0.0788 m（扫掠半径 − 车半宽）",
          req_corner - req_straight > 0.07, f"{req_corner:.4f} - {req_straight:.4f}")
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
    check_close("触线门槛 = 0.01 + 0.085 = 0.095 m", req, 0.095, 1e-12)
    # 行 7 格心（x=1.575）到中心线 1.75 ⇒ 0.175 m > 0.095 ⇒ 安全
    check_close("行 7 格心上行驶 ⇒ 0.175 m（安全）",
                P.barrier_distance_point_m(1.575, 1.0, x=x, y0=y0, y1=y1), 0.175, 1e-12)
    check("贴到中心线 0.05 m ⇒ 触线（< 0.095）",
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
    want = {"vehicle.length_m": 0.28, "vehicle.width_m": 0.17,
            "vehicle.coverage_radius_m": 0.035, "field.obstacle_radius_m": 0.1275,
            "field.mine_pad_m": 0.12, "field.barrier_width_m": 0.02,
            "planner.obstacle_margin_m": 0.02, "pursuit.cruise_speed_mps": 0.35,
            "path_executor.linear_speed": 0.35, "dzactuator.ticksPer2PI": 2680.0,
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
    check_close("压靶（车体）直段门槛 = 车半宽 0.085 m", req_s, 0.085, 1e-12)
    check_close("压靶（车体）拐角门槛 = 车体半对角线 0.16378 m", req_c, 0.16378, 1e-5)
    check("车体压到靶板（间隙 0.05 < 0.085）⇒ 判 overlap",
          P.target_body_overlap(0.05, length_m=L, width_m=W))
    check("车体没压到靶板（间隙 0.20 > 0.1638 拐角门槛）⇒ 不 overlap",
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
    # 挡板：D..N 含 K 段、无豁免 ⇒ 现场路线仍 0 风险（实测 0.175 ≥ 0.095）
    from dzbrain.core import geometry as _geo
    _live_pts = [_geo.block_to_meters(b) for b in _pm(
        [str(t) for t in _CFG.get("route.tokens") or []]).blocks()]
    _bmin, _bidx = RC.barrier_touch_min(_live_pts)
    check(f"挡板（D..N 含 K 段、**无豁免**）：现役路线最近 {_bmin:.4f} m ≥ 0.095 m",
          _bmin >= 0.095 - 1e-9, f"航段#{_bidx}")
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
                              ((0.14, 0.085), (0.14, -0.085),
                               (-0.14, -0.085), (-0.14, 0.085)))),
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


def test_trajectory_field_audit() -> None:
    """The compiler and the verifier share physics, not acceptance shortcuts."""
    from dataclasses import replace
    from dzbrain.tools.sim import FieldSpec
    from dzbrain.route.parser import parse_mission
    from dzbrain.pursuit.path import Path
    import random
    section("9. 连续轨迹物理门禁：车体净空 / 严格雷心 / 宽相位对拍")
    field = FieldSpec(mines=('M10',), obstacles=('F9',), targets=('D7', 'E7'),
                      bridges=('K10', 'K4'))
    gate = RC.TrajectoryFieldGate(field, _CFG)
    rng = random.Random(20261025)
    mismatch = []
    for _ in range(500):
        x, y, yaw = rng.uniform(-0.1, 4), rng.uniform(-0.1, 5), rng.uniform(-math.pi, math.pi)
        inspected = gate.inspect(x, y, yaw)
        expected = (inspected['obstacle_surface_m'] >= MARGIN and
                    all(value > 1e-9 for key, value in inspected.items()
                        if key not in ('obstacle_surface_m', 'obstacle_center_m')) and
                    all(not (abs(y-by) <= geo.BLOCK_SIZE/2 and abs(x-bx) <= geo.BLOCK_SIZE/2)
                        or abs(x-bx) <= geo.BLOCK_SIZE/4 for bx, by in gate.openings) and
                    (abs(math.cos(yaw)) <= 1e-6 or all(P.body_rect_clearance_m(
                        x, y, yaw, rect_cx=bx, rect_cy=by, rect_half_x=geo.BLOCK_SIZE/2,
                        rect_half_y=geo.BLOCK_SIZE/2, **gate.body) > 1e-9 for bx, by in gate.openings)))
        if gate(x, y, yaw) != expected:
            mismatch.append((x, y, yaw))
    check("AABB只筛远物体：500随机姿态与完整物理复核逐项一致", not mismatch, str(mismatch[:1]))
    ox, oy = geo.block_to_meters('F9')
    check("障碍中心距0.25<0.30只告警，车体侧面净空达标可通过",
          gate(ox, oy+0.25, 0.0))
    check("障碍中心距同为0.25，车头ref_offset侵入则拒绝",
          not gate(ox-0.25, oy, 0.0))
    check("非有限姿态拒绝", not gate(float('nan'), 1, 0) and not gate(1, 1, float('inf')))
    route = parse_mission(['N10', 'M10,y'])
    path = Path(route)
    clean = RC.TrajectoryFieldGate(replace(field, obstacles=(), targets=()), _CFG)
    samples = RC._sample_path(path, 0.01, continuous=False)
    shifted = [replace(q, x=q.x+0.05) for q in samples]
    result = RC.trajectory_metrics(route, shifted, [0, path.total_length], clean.field, clean,
                                   mandatory_indices=(0, 1), curvature_bound=0.0)
    check("严格雷心判据与旧垫片覆盖分别报告：横偏5cm=严格0/1、垫片1/1",
          result['strict_mine_covered'] == 0 and result['pad_mine_covered'] == 1)
    check_close("原顶点误差仍报告5cm，不能用轨迹自身零误差替代",
                result['waypoint_error_max_m'], 0.05, 1e-9)
    check_close("雷任务必经点按映射事件的真车心计误差",
                result['mandatory_error_max_m'], math.hypot(0.05, _CFG.get('vehicle.ref_offset_m')), 1e-9)
    check_close("轨迹对原折线偏离独立保留",
                result['trajectory_to_source_polyline_max_m'], 0.05, 1e-9)
    from dzbrain.route.model import Route
    short_route = Route((route[1], replace(route[1], y=route[1].y+0.03)))
    short_path = Path(short_route)
    reference_only = RC.trajectory_metrics(short_route, RC._sample_path(short_path, 0.001, continuous=False),
        [0, short_path.total_length], clean.field, clean, curvature_bound=0.0)
    check("后轴压雷不冒充几何车心：rear 1/1但body 0/1分别保留",
          reference_only['strict_mine_covered'] == 1 and reference_only['strict_body_center_mine_covered'] == 0)
    bridge_route = parse_mission(['L10', 'K10', 'J10,y'])
    bridge_path = Path(bridge_route)
    bridge_samples = RC._sample_path(bridge_path, 0.01, continuous=False)
    bridge_report = RC.trajectory_metrics(bridge_route, bridge_samples,
        [bridge_path.s_at_waypoint(i) for i in range(3)], clean.field, clean, curvature_bound=0.0)
    check("0.35m桥洞沿轴直穿可验证净空",
          bridge_report['clearance_certified'] and bridge_report['bridge_axis_error_max_rad'] < 1e-9)
    bx, by = geo.block_to_meters('K10')
    check("桥洞横向偏离9cm即使矩形仍有墙余量也拒绝", not clean(bx+0.09, by, math.pi/2))
    # ★ 2026-10-27 真值：净开口 0.33、车宽 0.17 ⇒ 横向余量恰为 **0.080 m**
    #   ⇒ 偏 8.0 cm 正好**贴壁**（clearance=0，判不通过），"刚好过"是 7.9 cm。
    #   旧断言"8cm 安全可通过"是"整格无墙 0.175 + 车宽 0.135"时代的产物，意图不变只换真值。
    check("桥洞横向偏离7.9cm且精确矩形安全可通过", clean(bx+0.079, by, math.pi/2))
    check("桥洞横向偏离8.0cm=真值余量边界 ⇒ 贴壁不可通过", not clean(bx+0.08, by, math.pi/2))
    check("参考点尚未进洞但车头已进洞，提前转弯必须拒绝",
          not clean(bx, by-geo.BLOCK_SIZE/2-0.1, math.pi/2+0.1))
    bad = [replace(q, yaw=q.yaw+0.1) for q in bridge_samples]
    tilted = RC.trajectory_metrics(bridge_route, bad,
        [bridge_path.s_at_waypoint(i) for i in range(3)], clean.field, clean, curvature_bound=0.0)
    check("桥洞横摆有独立对正误差，不能仅凭参考点没撞墙宣称通过",
          tilted['bridge_axis_error_max_rad'] > 0.09)
    jumped = [replace(q, reverse=i > len(samples)//2) for i, q in enumerate(samples)]
    jump_report = RC.trajectory_metrics(route, jumped, [0, path.total_length], clean.field, clean,
                                        curvature_bound=0.0)
    check("cusp车身若跳转180度，不得发连续扫掠净空证书",
          jump_report['body_yaw_step_excess_rad'] > 3.0 and not jump_report['clearance_certified'])
    check("末点朝向逗号保留", RC._trajectory_tokens('N10 M10,y') == ['N10', 'M10,y'])


def test_trajectory_cli() -> None:
    import json
    import tempfile
    import subprocess
    import os
    section("10. 现有route_check CLI：显式地图、双图、失败结构化")
    with tempfile.TemporaryDirectory(prefix='trajectory-physics-') as directory:
        folder = pathlib.Path(directory)
        maps = [folder/'one.yaml', folder/'two.yaml']
        for map_path, extra in zip(maps, ('[]', '[A1]')):
            map_path.write_text('map:\n  mines: [M10]\n  obstacles: '+extra+
                '\n  targets: []\n  bridges: [K10, K4]\n', encoding='utf-8')
        env = dict(os.environ, PYTHONIOENCODING='utf-8', PYTHONUTF8='1')
        cmd = [sys.executable, str(_WS/'tools/route_check.py'), '--trajectory',
               '--config', str(_WS/'config/robot.yaml'), '--route', 'N10 M10,y',
               '--json', '--max-nodes', '10']
        proc = subprocess.run(cmd+['--map', str(maps[0]), '--map', str(maps[1])],
                              capture_output=True, text=True, encoding='utf-8', env=env, timeout=30)
        lines = [line for line in proc.stdout.splitlines() if line.startswith('TRAJECTORY_RESULT_JSON ')]
        report = json.loads(lines[-1].split(' ', 1)[1]) if lines else {}
        check("同一CLI/参数接受不同布局两图并输出严格JSON", proc.returncode == 0 and
              report.get('ok') and len(report.get('reports', [])) == 2, proc.stderr)
        check("CLI同时保留旧折线指标和新轨迹指标", all(
              'polyline' in r and 'trajectory' in r for r in report.get('reports', [])))
        from dzbrain.tools.sim import SimConfig, resolve_initial_yaw, DESIGN_START_YAW
        from dzbrain.route.parser import parse_mission
        crossed = parse_mission(['N10', 'N9,x'])
        yaw, source = resolve_initial_yaw(crossed, SimConfig())
        check("真实N10起姿由共享规则给+Y，不能跟首段+X洗平",
              abs(yaw-DESIGN_START_YAW) < 1e-12 and abs(yaw-crossed[0].heading_to(crossed[1])) > 1.0)
        check("CLI记录真实起姿且候选在同一车身角起步", all(
              abs(r['initial_pose']['yaw']-DESIGN_START_YAW) < 1e-12 and
              r['trajectory']['initial_body_yaw_error_rad'] < 1e-9 for r in report['reports']))
        before, after = folder/'before.csv', folder/'after.csv'
        before.write_text('file,exit\ntest_old.py,1\ntest_same.py,0\n', encoding='utf-8')
        after.write_text('file,exit\ntest_old.py,0\ntest_same.py,0\n', encoding='utf-8')
        partial = subprocess.run(cmd+['--map', str(maps[0]), '--simulate', '--sim-max-steps', '5',
            '--output-dir', str(folder/'artifacts'), '--baseline-suites', str(before), '--candidate-suites', str(after)],
            capture_output=True, text=True, encoding='utf-8', env=env, timeout=30)
        partial_batch = json.loads(next(line.split(' ', 1)[1] for line in partial.stdout.splitlines()
                                       if line.startswith('TRAJECTORY_RESULT_JSON ')))
        partial_report = partial_batch['reports'][0]
        comparison = partial_report['simulation']
        check("--simulate复用两臂且未完赛RMS不能证提升", partial.returncode == 1 and
              not comparison['rms_comparison_eligible'] and
              all(not arm['completed'] for arm in comparison['arms'].values()))
        check("两臂真值pose经过当前精确车体场地观察", all(
              'strict_body_center_mine_covered' in arm['observed_field'] and
              'sampled_region_violations' in arm['observed_field'] for arm in comparison['arms'].values()))
        check("复用现有CSV/SVG保存器，路径真实落盘", all(
              pathlib.Path(path).is_file() for arm in comparison['arms'].values() for path in arm['artifacts'].values()))
        check("既有suite CSV只读合表保留哈希和修复红数", partial_batch['suite_comparison']['fixed_red'] == ['test_old.py']
              and partial_batch['suite_comparison']['new_red'] == []
              and len(partial_batch['suite_comparison']['snapshots']['baseline']['sha256']) == 64)
        maps[0].write_text('map:\n  mines: [M10]\n  obstacles: [N10]\n'
                           '  targets: []\n  bridges: [K10, K4]\n', encoding='utf-8')
        failed = subprocess.run(cmd+['--map', str(maps[0])], capture_output=True, text=True,
                                encoding='utf-8', env=env, timeout=30)
        failure = json.loads(next(line.split(' ', 1)[1] for line in failed.stdout.splitlines()
                                 if line.startswith('TRAJECTORY_RESULT_JSON ')))
        check("不可编译输出失败JSON及非零退出码", failed.returncode == 1 and not failure['ok'] and
              failure['reports'][0]['errors'], failed.stderr)
        check("有限搜索失败不等同世界无解", 'not a proof' in failure['reports'][0]['conclusion'])
        unplanned = subprocess.run([sys.executable, str(_WS/'tools/route_check.py'), '--trajectory',
            '--map', str(maps[0]), '--json'], capture_output=True, text=True,
            encoding='utf-8', env=env, timeout=30)
        planning_failure = json.loads(next(line.split(' ', 1)[1] for line in unplanned.stdout.splitlines()
                                           if line.startswith('TRAJECTORY_RESULT_JSON ')))
        check("原planner输入拒绝也输出结构化失败（不补造起终点）",
              unplanned.returncode == 2 and planning_failure['reports'][0]['stage'] == 'input_or_planning')


def main() -> None:
    print("=" * 74)
    test_trajectory_field_audit()
    test_trajectory_cli()
    if '--trajectory-only' in sys.argv:
        finish()
        return
    print("dzbrain 物理几何基础自测（t22：0.28×0.17 车体 / r=0.1275 障碍 / 0.12 垫片 / 2cm 挡板）")
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
