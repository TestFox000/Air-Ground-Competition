# -*- coding: utf-8 -*-
"""物理几何基础（t22）—— **纯函数**：不读文件、不 import ROS、不产生 Issue。

用户 2026-10-11 现场口径（**数值全部来自 `config/robot.yaml` / `core/config.py` 的 SCHEMA**，
本模块只提供**公式**与默认形参，绝不把现场数值烘焙进调用方）：

| 物件 | 现场几何 | 判据 |
|---|---|---|
| **车体** | 长 0.28 m × 宽 0.135 m 的矩形；另有"车心假设圆"半径 **0.035 m** | 压雷用那个 3.5 cm 圆；转弯用扫掠圆（半对角线） |
| **障碍** | 半径 **0.1275 m** 的圆柱，柱心在障碍格格心 | 车心距柱心 ≥ `r + 车半宽 + margin`（拐角处车半宽取扫掠半径） |
| **雷** | **0.12 × 0.12 m 垫片**，中心在雷格格心 | 车心 3.5 cm 圆与垫片**产生过重叠** ⇔ 车心到垫片最近距离 ≤ 0.035 m |
| **挡板** | 第 6/7 行之间、**D..N 列**，长 11×0.35 = **3.85 m**、厚 **0.02 m** 细条（中心线在 x=1.75） | 车心到中心线 ≥ `0.01 + 0.0675 = 0.0775 m`（桥洞 K 处豁免，沿用通口逻辑） |

★★ 三个容易搞错的地方（本模块存在的理由）：
1. **压雷不能用"车心到雷心 ≤ 3.5 cm"**：垫片是 0.12 m 见方的**方块**，"贴边压过"时
   车心离雷心可以到 `0.06 + 0.035 = 0.095 m` 仍算压到 —— 差一个垫片半宽（0.06 m）量级。
   正确判据是 **点到矩形的最近距离**（见 `point_rect_distance_m`）。
2. **转弯时不能用"车半宽"当扫掠半径**：车体绕顶点转 90° 时，车身外包圆半径是
   **半对角线** `√((0.28/2)²+(0.135/2)²) = 0.1554 m`（远大于 0.0675）；顶点邻着障碍/挡板时
   必须按扫掠圆算。
3. **挡板是 2 cm 细条而不是"一整格"**：格带口径（旧 `tools/route_check.py` 的
   `BARRIER_MIN_MARGIN = 0.10` 从缺口西缘算）会**高估**挡板占位，把"其实过得去"的路线判红；
   细条口径的门槛是 `挡板半厚 0.01 + 车半宽 0.0675 = 0.0775 m`。

⚠️ 门禁口径在 `tools/route_check.py`（用户侧脚本，**刻意不 import dzbrain**：车端没装包也要能单跑）
  —— 它按**同一套公式**复算，并由 `src/dzbrain/tests/test_physics.py` 的**对拍断言**钉住
  两份实现不许分叉。规划器/仿真侧请直接 import 本模块。
⚠️ 卡 t11 起本模块还提供**参考点偏移感知**的车体足迹（`body_corners()` / `body_*_clearance_m()`，
  真源 = `config/robot.yaml: vehicle.ref_offset_m`，现场实测 +7.5 cm）。**默认偏移 0.0**，
  所以既有调用方行为不变；而 `tools/route_check.py` 那份副本**不含**偏移口径（它读不到
  `vehicle.ref_offset_m`）——它的挡板/障碍判据只等价于 `offset=0` 的情形，这一点必须在
  报告里说明（t11：+7.5 cm 时挡板余量从 +7.6 mm 变成 −5.2 mm 的侵入）。
⚠️ 本模块**不做**语义判断（该不该压雷、绕不绕路），只回答"几何上有没有碰上"。
"""

from __future__ import annotations

import math

from ..core import geometry as geo

__all__ = [
    "CELL_M",
    "point_segment_distance_m",
    "point_rect_distance_m",
    "segment_rect_distance_m",
    "vehicle_half_diagonal_m",
    "sweep_radius",
    "mine_clearance_m",
    "mine_clearance_segment_m",
    "mine_pressed",
    "obstacle_clearance_point_m",
    "obstacle_clearance_segment_m",
    "required_obstacle_center_distance_m",
    "barrier_centerline",
    "barrier_distance_point_m",
    "barrier_distance_segment_m",
    "required_barrier_distance_m",
    "parse_col_span",
    "segment_outside_band",
    "k_cells_sealed_along_column",
    "board_half_m",
    "required_target_body_clearance_m",
    "target_body_overlap",
    "body_overlaps_rect_centered",
    # ★★ 卡 t11：**参考点偏移**感知的车体足迹（净空唯一真源）
    "DEFAULT_REF_OFFSET_M",
    "body_center",
    "body_corners",
    "corner_poly",
    "polygon_overlap_depth",
    "polygon_distance_m",
    "body_obstacle_clearance_m",
    "body_barrier_clearance_m",
    "body_rect_clearance_m",
]

#: 格边长（= `geometry.BLOCK_SIZE`，只在这里取一次，避免各处重复写 0.35）
CELL_M = geo.BLOCK_SIZE


# --------------------------------------------------------------------------- #
#  基本几何：点↔线段、点↔矩形、线段↔矩形
# --------------------------------------------------------------------------- #
def point_segment_distance_m(px: float, py: float,
                             ax: float, ay: float, bx: float, by: float) -> float:
    """点 P 到线段 AB 的**最近距离**（投影钳位到 [0,1]，退化线段按点处理）。"""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 <= 1e-18:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / L2
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def point_rect_distance_m(px: float, py: float, cx: float, cy: float, half: float) -> float:
    """点 P 到**轴对齐矩形**（心 (cx,cy)、半边长 `half`）的最近距离；在矩形内 = 0。

    ★ 这就是**压雷判据**要用的距离（垫片是正方形，不是圆）：只在两个轴上都超出时才计入，
      因此"贴着垫片侧边压过"（车心在垫片外侧 0.06 m 内）会被正确判成压到。
    """
    ox = abs(px - cx) - half
    oy = abs(py - cy) - half
    if ox <= 0.0 and oy <= 0.0:
        return 0.0
    return math.hypot(max(ox, 0.0), max(oy, 0.0))


def segment_rect_distance_m(ax: float, ay: float, bx: float, by: float,
                            cx: float, cy: float, half: float) -> float:
    """线段 AB 到轴对齐矩形（心 (cx,cy)、半边长 `half`）的最近距离。

    ★ 两步（缺一不可）：
      ① **相交则 0** —— 用 Liang–Barsky 平板裁剪判定"段与矩形有没有公共点"；
         ⚠️ 少了这一步，"从矩形中间穿过去"会被算成"离最近的角还有 half 那么远"（错）。
      ② 不相交时：两个**凸**集的最近点对必取在某一方的**顶点**上
         ⇒ `min(4 个角到线段, 两端点到矩形)` 是**精确**解（不是近似、不是采样）。
    """
    if _segment_rect_intersects(ax, ay, bx, by, cx, cy, half):
        return 0.0
    corners = ((cx - half, cy - half), (cx + half, cy - half),
               (cx + half, cy + half), (cx - half, cy + half))
    return min(min(point_segment_distance_m(qx, qy, ax, ay, bx, by) for qx, qy in corners),
               point_rect_distance_m(ax, ay, cx, cy, half),
               point_rect_distance_m(bx, by, cx, cy, half))


def _segment_rect_intersects(ax: float, ay: float, bx: float, by: float,
                             cx: float, cy: float, half: float) -> bool:
    """线段与轴对齐矩形是否相交（含相切）—— Liang–Barsky 平板裁剪。"""
    lo_x, hi_x = cx - half, cx + half
    lo_y, hi_y = cy - half, cy + half
    dx, dy = bx - ax, by - ay
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, ax - lo_x), (dx, hi_x - ax), (-dy, ay - lo_y), (dy, hi_y - ay)):
        if abs(p) <= 1e-18:
            if q < 0.0:
                return False                              # 平行且在板外
            continue
        r = q / p
        if p < 0.0:
            if r > t1:
                return False
            if r > t0:
                t0 = r
        else:
            if r < t0:
                return False
            if r < t1:
                t1 = r
    return t0 <= t1


# --------------------------------------------------------------------------- #
#  车体：扫掠半径
# --------------------------------------------------------------------------- #
def vehicle_half_diagonal_m(length_m: float = 0.28, width_m: float = 0.135) -> float:
    """车体**半对角线**（= 车身外包圆半径）= `√((L/2)² + (W/2)²)`。

    现场值：`√(0.14² + 0.0675²) = 0.15542 m`。
    """
    return math.hypot(length_m / 2.0, width_m / 2.0)


def sweep_radius(*, length_m: float = 0.28, width_m: float = 0.135,
                 wheelbase_m: float = 0.157, max_steer_deg: float = 30.0
                 ) -> tuple[float, float]:
    """``(扫掠半径, 最小转弯半径)``。

    * **扫掠半径** = 车体半对角线（上一条）—— 顶点转向时车身的最大外包半径；
    * **最小转弯半径** `R = wheelbase / tan(max_steer_deg)` —— 与 `pursuit.*` 同源
      （现场 `0.157/tan30° = 0.2719 m`，这也是"到不了格心"的几何根因，见 test_sim 8.14）。

    ★ 两个数**都要**：净空判据用前者（车身占位），"能不能贴着走"的可行性判断用后者。
    """
    if not (0.0 < max_steer_deg < 90.0):
        raise ValueError(f"max_steer_deg 必须在 (0,90) 内，收到 {max_steer_deg!r}")
    return vehicle_half_diagonal_m(length_m, width_m), wheelbase_m / math.tan(
        math.radians(max_steer_deg))


# --------------------------------------------------------------------------- #
#  雷：0.12×0.12 垫片
# --------------------------------------------------------------------------- #
def mine_clearance_m(px: float, py: float, mine_x: float, mine_y: float,
                     *, mine_pad_m: float = 0.12) -> float:
    """车心 P 到**雷垫片**（边长 `mine_pad_m` 的正方形）的最近距离（m）。"""
    return point_rect_distance_m(px, py, mine_x, mine_y, mine_pad_m / 2.0)


def mine_clearance_segment_m(ax: float, ay: float, bx: float, by: float,
                             mine_x: float, mine_y: float, *,
                             mine_pad_m: float = 0.12) -> float:
    """**航段** AB 到雷垫片的最近距离（m）—— 整段上的最小"车心↔垫片"距离。"""
    return segment_rect_distance_m(ax, ay, bx, by, mine_x, mine_y, mine_pad_m / 2.0)


def mine_pressed(px: float, py: float, mine_x: float, mine_y: float, *,
                 coverage_radius_m: float = 0.035, mine_pad_m: float = 0.12) -> bool:
    """车心 3.5 cm 圆与垫片**产生过重叠**吗（现场压雷判据）。

    等价判据：``车心到垫片最近距离 ≤ coverage_radius_m``。
    """
    return mine_clearance_m(px, py, mine_x, mine_y,
                            mine_pad_m=mine_pad_m) <= coverage_radius_m + 1e-12


# --------------------------------------------------------------------------- #
#  障碍：半径 0.1275 的圆柱（柱心在格心）
# --------------------------------------------------------------------------- #
def obstacle_clearance_point_m(px: float, py: float, ox: float, oy: float) -> float:
    """车心 P 到**障碍柱心**的距离（m）—— 判据是它 ≥ `柱半径 + 车半宽 + margin`。"""
    return math.hypot(px - ox, py - oy)


def obstacle_clearance_segment_m(ax: float, ay: float, bx: float, by: float,
                                 ox: float, oy: float) -> float:
    """航段 AB 到**障碍柱心**的最近距离（m）。"""
    return point_segment_distance_m(ox, oy, ax, ay, bx, by)


def required_obstacle_center_distance_m(*, obstacle_radius_m: float = 0.1275,
                                        body_radius_m: float = 0.0675,
                                        margin_m: float = 0.02) -> float:
    """净空判据的门槛：`柱半径 + 车身半径 + 余量`。

    直段用 `body_radius_m = 车半宽 = 0.0675` ⇒ **0.215 m**；
    拐角顶点用 `body_radius_m = 扫掠半径 = 0.1554` ⇒ **0.3029 m**。
    """
    return obstacle_radius_m + body_radius_m + margin_m


# --------------------------------------------------------------------------- #
#  挡板：第 6/7 行之间、D..N 列的 2 cm 细条
# --------------------------------------------------------------------------- #
def parse_col_span(spec: str) -> tuple[str, str]:
    """把 ``"D..N"`` / ``"D-N"`` / ``"D"`` 解析成 ``(起始列, 结束列)``。

    ★ 配置里写 `field.barrier_cols: "D..N"` —— 一个键说清"挡板覆盖哪些列"，
      不额外造列表键（与 `config/map.yaml` 的 `barrier_col_from` 单一真源思路一致）。
    """
    s = str(spec).strip().upper().replace("..", "-").replace("—", "-")
    if not s:
        raise ValueError("空列范围")
    if "-" in s:
        a, _, b = s.partition("-")
        return a.strip(), b.strip()
    ch = s[0]
    return ch, ch


def barrier_centerline(*, col_from: str = "D", col_to: str = "N",
                       row_hi: int = 6, row_lo: int = 7) -> tuple[float, float, float]:
    """挡板**中心线**（`x` 恒定，`y` 一段）→ ``(x, y0, y1)``。

    取第 6 行与第 7 行格心的中点：现场 `x = ((10.5-5)*0.35 + (10.5-6)*0.35)/2 = 1.75`；
    `y` 覆盖 D..N 两端的**外缘**（列心 ± 半格）：`0.0 ~ 3.85`（= 11 列 × 0.35 m ✓ 与现场口径一致）。
    """
    x_hi = (10.5 - (row_hi - 1)) * CELL_M
    x_lo = (10.5 - (row_lo - 1)) * CELL_M
    c0 = geo.COL_LETTERS.index(col_from)
    c1 = geo.COL_LETTERS.index(col_to)
    y_a = (13.5 - c0) * CELL_M + CELL_M / 2.0
    y_b = (13.5 - c1) * CELL_M - CELL_M / 2.0
    return (x_hi + x_lo) / 2.0, min(y_a, y_b), max(y_a, y_b)


def barrier_distance_point_m(px: float, py: float, *, x: float = 1.75,
                             y0: float = 0.0, y1: float = 3.85) -> float:
    """车心 P 到挡板**中心线**的距离（m）—— 判据 ≥ `挡板半厚 + 车半宽 = 0.0775`。"""
    return point_segment_distance_m(px, py, x, y0, x, y1)


def barrier_distance_segment_m(ax: float, ay: float, bx: float, by: float, *,
                               x: float = 1.75, y0: float = 0.0, y1: float = 3.85) -> float:
    """航段 AB 到挡板中心线的最近距离（m）。**注意**：桥洞 K 处要先用
    `segment_outside_band()` 抠掉豁免段再调用本函数。"""
    return _seg_seg_distance(ax, ay, bx, by, x, y0, x, y1)


def required_barrier_distance_m(*, barrier_width_m: float = 0.02,
                                car_width_m: float = 0.135) -> float:
    """不触线的门槛：`挡板半厚 + 车半宽 = 0.01 + 0.0675 = 0.0775 m`。

    ★★ 2026-10-11 用户现场澄清（以澄清为准）：挡板是 **385 cm × 2 cm** 的细条，
      第 6/7 行之间、**D..N 列（含 K 段）**——即**挡板在 K 列处也是连续的，没有通口**；
      "通格 K1/K11 横向贯通"说的是 K 列**格与格之间**的纵向封死关系（见下），
      不是挡板上的洞。所以判据就是"车心到中心线 ≥ 0.0775 m，**不得跨越**"，
      **不需要任何桥洞豁免**（`segment_outside_band()` 仍作为通用工具保留，本判据不再调用）。
    """
    return barrier_width_m / 2.0 + car_width_m / 2.0


# --------------------------------------------------------------------------- #
#  K 列桥室（用户 2026-10-11 澄清②）
# --------------------------------------------------------------------------- #
def k_cells_sealed_along_column() -> bool:
    """K 列**纵向相邻格之间全部封死**（每对相邻 K 格之间都有挡板）。

    ★ 这条不是"新判据"，而是**解释既有判据的物理根据**：
      `validate.py` 的 R009「K 列内行驶」（`K9 -> K10` 这类沿列挪一格）与
      `tools/route_check.py` 的「桥洞列内行驶」之所以是**硬错**，就是因为每一对相邻
      K 格之间的壁是实体；而"通格 K1/K11 横向贯通"只说明**这两格**可以从东西方向穿过
      （U 形赛道的两处过洞），**不**意味着列内可以纵向移动。
    """
    return True


# --------------------------------------------------------------------------- #
#  靶板（35×35 cm/格、中心在格心 —— 用户 2026-10-11 澄清③④）
# --------------------------------------------------------------------------- #
def board_half_m() -> float:
    """靶板半边长 = **半格**（0.175 m）。

    ★★ 澄清③：靶板是 **35×35 cm**、格心对齐 ⇒ 棋盘上的"靶格"与靶板**逐个重合**
      ⇒ 规划器侧那条「靶格不可通行」的**格级挡块恰好就是物理精确**，不需要再做形状级建模
      （那边保持不动，只补注释）。本条判据只负责**路线几何**这一侧：车体不许与靶板 overlap。
    """
    return CELL_M / 2.0


def required_target_body_clearance_m(*, length_m: float = 0.28, width_m: float = 0.135,
                                     at_corner: bool = False) -> float:
    """车体不压靶板所需的最小**间隙**（折线/顶点到靶板矩形的距离）。

    * **直段**：车身横向占位 = 车半宽 `0.0675 m`；
    * **拐角**：车绕顶点转向时车身外包圆 = 车体半对角线 `0.1554 m`（≥ 半宽 0.088 m）。

    ⇒ 判据：``间隙 ≥ 本函数返回值``；等价于"车体 28×13.5 不与 35×35 靶板 overlap"。
    """
    return vehicle_half_diagonal_m(length_m, width_m) if at_corner else width_m / 2.0


def target_body_overlap(clearance_to_board_m: float, *, length_m: float = 0.28,
                        width_m: float = 0.135, at_corner: bool = False) -> bool:
    """给"折线/顶点到靶板矩形的间隙"，判定**车体是否压到靶板**（澄清④）。"""
    return clearance_to_board_m < required_target_body_clearance_m(
        length_m=length_m, width_m=width_m, at_corner=at_corner) - 1e-12


def body_overlaps_rect_centered(px: float, py: float, cx: float, cy: float, *, half: float,
                                length_m: float = 0.28, width_m: float = 0.135) -> bool:
    """**保守**版本：把车体当"以车心为心、半径 = 半对角线"的圆，判它是否与靶板矩形相交。

    ⚠️ 这只是给"只有一个点（例如某个航点）"的场景用的粗判（比真实的 28×13.5 矩形更保守）；
      航段请用 `segment_rect_distance_m()` + `target_body_overlap()` 那一对精确判据。
    """
    return point_rect_distance_m(px, py, cx, cy, half) < vehicle_half_diagonal_m(
        length_m, width_m) - 1e-12


def _seg_seg_distance(ax: float, ay: float, bx: float, by: float,
                      cx: float, cy: float, dx: float, dy: float) -> float:
    """两条线段的最短距离（相交 = 0）。用"端点对另一线段"取最小 —— 对两条线段成立
    的唯一例外是**交叉**，先判掉即可。"""
    if _segments_intersect(ax, ay, bx, by, cx, cy, dx, dy):
        return 0.0
    return min(point_segment_distance_m(ax, ay, cx, cy, dx, dy),
               point_segment_distance_m(bx, by, cx, cy, dx, dy),
               point_segment_distance_m(cx, cy, ax, ay, bx, by),
               point_segment_distance_m(dx, dy, ax, ay, bx, by))


def _segments_intersect(ax: float, ay: float, bx: float, by: float,
                        cx: float, cy: float, dx: float, dy: float) -> bool:
    def cross(ox, oy, px, py, qx, qy) -> float:
        return (px - ox) * (qy - oy) - (py - oy) * (qx - ox)
    d1 = cross(cx, cy, dx, dy, ax, ay)
    d2 = cross(cx, cy, dx, dy, bx, by)
    d3 = cross(ax, ay, bx, by, cx, cy)
    d4 = cross(ax, ay, bx, by, dx, dy)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def segment_outside_band(ax: float, ay: float, bx: float, by: float, *,
                         band_y: float, band_half: float
                         ) -> list[tuple[float, float, float, float]]:
    """把线段 AB 抠掉"落在 `|y − band_y| ≤ band_half` 带内"的部分，返回剩余子线段。

    ★ 用途：**桥洞豁免**。挡板在 K 列有通口（现场 `bridges: K1/K11` 那一列），
      车在 K 列带内穿行是**合法的**；但带外的部分照样不许贴线 ⇒ 必须精确抠段，
      不能"两端点有一个在带内就整段豁免"（那会把"斜着蹭过洞口外侧"放过去）。
    """
    out: list[tuple[float, float, float, float]] = []
    lo, hi = band_y - band_half, band_y + band_half
    dy = by - ay
    if abs(dy) <= 1e-15:                                  # 水平段：整段要么在带内要么在外
        if ay < lo - 1e-12 or ay > hi + 1e-12:
            out.append((ax, ay, bx, by))
        return out
    ta = (lo - ay) / dy
    tb = (hi - ay) / dy
    cut_lo, cut_hi = min(ta, tb), max(ta, tb)
    c0 = max(0.0, cut_lo)                                 # 带 ∩ [0,1] = [c0, c1]
    c1 = min(1.0, cut_hi)
    if c0 >= c1:                                          # 不相交（或只擦一点）⇒ 整段保留
        out.append((ax, ay, bx, by))
        return out
    for (u, v) in ((0.0, c0), (c1, 1.0)):                 # 带外的前后两截
        if v - u > 1e-12:
            out.append((ax + u * (bx - ax), ay + u * dy,
                        ax + v * (bx - ax), ay + v * dy))
    return out


# --------------------------------------------------------------------------- #
#  ★★ 卡 t11：参考点偏移感知的车体足迹（"净空唯一真源"补齐）
# --------------------------------------------------------------------------- #
#  背景（现场卷尺实测，2026-10-11）：
#    后轴中心→车尾沿 6.5 cm、后轴中心→车头沿 21.5 cm、车身总长 28 cm
#    ⇒ 车体**几何中心**在参考点（= 里程计/仿真积分点 = **后轴中心**）**前方 7.5 cm**。
#    `config/robot.yaml: vehicle.ref_offset_m` 是唯一真源；`dzactuator.cpp` 与
#    `tools/sim.py: step_bicycle` 都按"后轴"积分（`X += v·cos(yaw)`），所以这个偏移
#    是**真实几何**，不是建模花招。
#
#  为什么必须落在本模块：本模块是净空/碰撞的**唯一真源**（`tools/route_check.py` 那份
#  是等价的用户侧副本，由 `test_physics.py` 对拍钉住）。t5 实测：把车体矩形沿车头方向
#  平移 +3.5 cm 就让现役路线"顶到挡板"，+7.5 cm 报 35~38 拍侵入 ⇒ 偏移必须能进真源，
#  否则所有"净空够不够"的结论都默认了 `offset=0`（= 假设车体中心与后轴重合，**与实测矛盾**）。
#
#  兼容性铁律：`offset_m` 默认 `DEFAULT_REF_OFFSET_M = 0.0` ⇒ 不读配置的老调用方
#  （planner / sim / route_check / 既有测试）行为**逐字节不变**；只有显式传值的调用方
#  才进入"真实几何"口径。
#: 参考点(后轴中心) → 车体几何中心的纵向偏移默认值（米，正 = 中心在参考点前方）
DEFAULT_REF_OFFSET_M = 0.0


def body_center(px: float, py: float, yaw: float, *,
                offset_m: float = DEFAULT_REF_OFFSET_M) -> tuple[float, float]:
    """参考点 `(px,py,yaw)` → 车体**几何中心**（沿车头方向前移 `offset_m`）。"""
    return (px + offset_m * math.cos(yaw), py + offset_m * math.sin(yaw))


def body_corners(px: float, py: float, yaw: float, *,
                 length_m: float = 0.28, width_m: float = 0.135,
                 offset_m: float = DEFAULT_REF_OFFSET_M
                 ) -> tuple[tuple[float, float], ...]:
    """车体矩形 4 角（地图系，逆时针；中心 = `body_center()`）。

    `offset_m=0.0`（默认）时退化为"以参考点为心的轴对齐/旋转矩形" —— 与
    `tools/sim.py: body_corners()`（把矩形中心放在积分点上）**完全一致**。
    """
    cx, cy = body_center(px, py, yaw, offset_m=offset_m)
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = length_m / 2.0, width_m / 2.0
    return tuple((cx + dx * c - dy * s, cy + dx * s + dy * c)
                 for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)))


def corner_poly(cx: float, cy: float, half_x: float, half_y: float
                ) -> tuple[tuple[float, float], ...]:
    """轴对齐矩形 4 角（心 + 两个半边长）—— 挡板细条/靶板矩形都用它构造。"""
    return ((cx - half_x, cy - half_y), (cx + half_x, cy - half_y),
            (cx + half_x, cy + half_y), (cx - half_x, cy + half_y))


def _poly_axes(poly) -> list[tuple[float, float]]:
    """凸多边形的所有边法线（SAT 候选轴）。"""
    out: list[tuple[float, float]] = []
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy)
        if L > 1e-12:
            out.append((-dy / L, dx / L))
    return out


def _poly_proj(poly, ax: tuple[float, float]) -> tuple[float, float]:
    vals = [p[0] * ax[0] + p[1] * ax[1] for p in poly]
    return min(vals), max(vals)


def polygon_overlap_depth(a, b) -> float:
    """两**凸**多边形的重叠深度（SAT/MTV；`> 0` = 真重叠，`0` = 不重叠）。

    ★ 与 `tools/sim.py` / t5lib 的 `sat_overlap_depth` 同式：只要有一条分离轴就返回 0，
      否则返回最短穿透深度 —— "**碰上了没有**"必须用这个判据，不能用"中心在不在里面"。
    """
    best = float("inf")
    for ax in _poly_axes(a) + _poly_axes(b):
        lo_a, hi_a = _poly_proj(a, ax)
        lo_b, hi_b = _poly_proj(b, ax)
        ov = min(hi_a, hi_b) - max(lo_a, lo_b)
        if ov <= 0.0:
            return 0.0
        best = min(best, ov)
    return best


def polygon_distance_m(a, b) -> float:
    """两**凸**多边形的最近距离（重叠 = 0）。凸集的最近点必在"顶点-边"对上 ⇒ 精确。"""
    if polygon_overlap_depth(a, b) > 0.0:
        return 0.0
    best = float("inf")
    for poly_a, poly_b in ((a, b), (b, a)):
        n, m = len(poly_a), len(poly_b)
        for i in range(n):
            ax, ay = poly_a[i]
            bx, by = poly_a[(i + 1) % n]
            for j in range(m):
                cx, cy = poly_b[j]
                dx, dy = poly_b[(j + 1) % m]
                best = min(best, _seg_seg_distance(ax, ay, bx, by, cx, cy, dx, dy))
    return best


def _point_in_poly(px: float, py: float, poly) -> bool:
    inside = False
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        if (y0 > py) != (y1 > py):
            x_at = x0 + (py - y0) * (x1 - x0) / (y1 - y0)
            if x_at > px:
                inside = not inside
    return inside


def body_obstacle_clearance_m(px: float, py: float, yaw: float, ox: float, oy: float, *,
                              obstacle_radius_m: float = 0.1275,
                              length_m: float = 0.28, width_m: float = 0.135,
                              offset_m: float = DEFAULT_REF_OFFSET_M) -> float:
    """车体矩形表面 → **圆柱障碍表面**的间隙（米；`≤ 0` = 真重叠）。

    比老口径（"车心到柱心 ≥ r + 车半宽 + 余量"）严格：它算的是旋转车体的真实角点，
    所以"斜着蹭柱面"不会被漏掉（t5 实测：老口径报 0 拍，本口径报 253 拍真重叠）。
    """
    poly = body_corners(px, py, yaw, length_m=length_m, width_m=width_m, offset_m=offset_m)
    if _point_in_poly(ox, oy, poly):
        return -obstacle_radius_m
    d = min(point_segment_distance_m(ox, oy, poly[i][0], poly[i][1],
                                     poly[(i + 1) % len(poly)][0],
                                     poly[(i + 1) % len(poly)][1])
            for i in range(len(poly)))
    return d - obstacle_radius_m


def body_barrier_clearance_m(px: float, py: float, yaw: float, *,
                             barrier_width_m: float = 0.02,
                             barrier_x: float = 1.75,
                             barrier_y0: float = 0.0, barrier_y1: float = 3.85,
                             length_m: float = 0.28, width_m: float = 0.135,
                             offset_m: float = DEFAULT_REF_OFFSET_M) -> float:
    """车体矩形表面 → **挡板细条表面**的间隙（米；重叠 = 0）。

    挡板是 `barrier_width_m` 厚的细条（`x = barrier_x` 恒定，`y ∈ [y0, y1]`），
    所以判据是"两个矩形的最近距离"，**不是**"车心到中心线 ≥ 半厚 + 车半宽"（后者把
    旋转中的车身当成半宽细条，t5 实测两口径差 13 mm）。
    """
    poly = body_corners(px, py, yaw, length_m=length_m, width_m=width_m, offset_m=offset_m)
    strip = corner_poly(barrier_x, (barrier_y0 + barrier_y1) / 2.0,
                        barrier_width_m / 2.0, (barrier_y1 - barrier_y0) / 2.0)
    return polygon_distance_m(poly, strip)


def body_rect_clearance_m(px: float, py: float, yaw: float, *,
                          rect_cx: float, rect_cy: float,
                          rect_half_x: float, rect_half_y: float,
                          length_m: float = 0.28, width_m: float = 0.135,
                          offset_m: float = DEFAULT_REF_OFFSET_M) -> float:
    """车体矩形表面 → **任意轴对齐矩形实体**表面（靶板/桥室侧壁都用它）的间隙。

    重叠 = 0；`rect_half_*` 是半边长（跨格靶板 35×70 cm ⇒ 半边长 0.175/0.35）。
    """
    poly = body_corners(px, py, yaw, length_m=length_m, width_m=width_m, offset_m=offset_m)
    return polygon_distance_m(poly, corner_poly(rect_cx, rect_cy, rect_half_x, rect_half_y))
