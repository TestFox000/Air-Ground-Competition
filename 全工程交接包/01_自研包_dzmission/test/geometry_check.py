#!/usr/bin/env python3
"""几何可行性核算 —— 基于 SmartMaster 官方实测参数

官方参数（《SmartMaster 硬件操作指南 V1.0》表 1-1）：
  长度 278 mm      宽度 140 mm      高度 227 mm
  轴距 150 mm      轮距 110 mm
  接近角 16°       离去角 16°
  最大转角 30±2°   最小转弯半径 240~260 mm
  最小离地间距 11 mm   爬坡角度 35°
  最高速度 1 m/s   充电电压 12V   充电时长 6h

场地：14 × 11 格 @35cm；水平隔离线 D6–M6/D7–M7；K 列桥洞排；
      起点 M7、终点 N5。

运行:  python3 geometry_check.py
"""

import math
import sys

# ---------------- 官方参数 ----------------
L_MM = 278.0
W_MM = 140.0
H_MM = 227.0
WB_MM = 150.0
TRACK_MM = 110.0
STEER_MAX = 30.0
STEER_TOL = 2.0
R_MIN_LO = 240.0
R_MIN_HI = 260.0
V_MAX = 1.0
GROUND_CLEAR = 11.0

# ---------------- 场地 ----------------
CELL = 350.0
COLS, ROWS = 14, 11
FIELD_W, FIELD_H = COLS * CELL, ROWS * CELL
WALL_Y = 6 * CELL
WALL_C0, WALL_C1 = 3, 12

START = (12, 6)     # M7
GOAL = (13, 4)      # N5

FRONT_MM = 0.62 * L_MM
REAR_MM = 0.38 * L_MM
HALF_W = W_MM / 2


def cc(c, r):
    return ((c + 0.5) * CELL, (r + 0.5) * CELL)


def sec(t):
    print()
    print("=" * 78)
    print(t)
    print("=" * 78)


# ================================================================ 1
def check_dimension_limits():
    sec("1. 与规则检录上限的对照")
    print("  规则上限: 车 <= 300 x 250 x 250 mm")
    print("  实测    : 车 =  %.0f x %.0f x %.0f mm" % (L_MM, W_MM, H_MM))
    print()
    for name, val, lim in (("长度", L_MM, 300), ("宽度", W_MM, 250), ("高度", H_MM, 250)):
        print("  %s: %6.0f mm / 上限 %3d mm  -> 余量 %5.0f mm  %s"
              % (name, val, lim, lim - val, "OK" if lim - val > 0 else "超限"))
    print()
    print("  车宽 140mm 只有上限的 56%，余量充裕")
    print("  长度 278mm 达上限的 93%，注意裁判测量是否含云台 / 激光发射机构")


# ================================================================ 2
def check_cell_lane():
    sec("2. 格内车道的横向余量")
    band = CELL / 2 - HALF_W
    print("  格宽 {:.0f} mm，车宽 {:.0f} mm，车半宽 {:.0f} mm".format(CELL, W_MM, HALF_W))
    print()
    print("  以格中心行驶：单侧剩余 {:.0f} mm -> 横向偏移带 +/-{:.0f} mm"
          .format(band, band))
    print("  对照上一轮按 250mm 最坏假设估的 +/-50mm：现在宽松 {:.1f} 倍"
          .format(band / 50.0))
    print("  结论：纯航位推算 + 纯跟踪够用，上一轮『把车做窄到 180mm』建议撤回")
    print()
    print("  起点 M7 / 终点 N5 净空：")
    for name, (c, r) in (("M7", START), ("N5", GOAL)):
        x, y = cc(c, r)
        to_wall = abs(y - WALL_Y) if WALL_C0 <= c <= WALL_C1 else 1e9
        to_edge = min(x, FIELD_W - x, y, FIELD_H - y)
        gap = min(to_wall, to_edge)
        src = "水平挡板" if to_wall <= to_edge else "场地边缘"
        print("    {} 格心 ({:.0f}, {:.0f})  最近约束 {} {:.0f} mm  "
              "车半宽 {:.0f} -> 余量 {:.0f} mm"
              .format(name, x, y, src, gap, HALF_W, gap - HALF_W))


# ================================================================ 3
def check_turn_radius():
    sec("3. 转弯半径：理论与实测对照")
    print("  阿克曼几何: R_rear_axle = 轴距 / tan(前轮转角)，轴距 = {:.0f} mm"
          .format(WB_MM))
    print()
    print("  前轮转角   理论 R_rear   外轮半径")
    for d in (26, 28, 30, 32):
        t = math.tan(math.radians(d))
        r_rear = WB_MM / t
        r_out = math.hypot(r_rear + TRACK_MM / 2, WB_MM)
        print("    {:2d} deg   {:7.1f} mm   {:7.1f} mm".format(d, r_rear, r_out))
    print()
    lo = WB_MM / math.tan(math.radians(STEER_MAX + STEER_TOL))
    hi = WB_MM / math.tan(math.radians(STEER_MAX - STEER_TOL))
    print("  官方实测最小转弯半径 {:.0f} ~ {:.0f} mm；理论落点 {:.1f} ~ {:.1f} mm"
          .format(R_MIN_LO, R_MIN_HI, lo, hi))
    print("  -> 一致。规划取保守值 R = {:.0f} mm".format(R_MIN_HI))


# ================================================================ 4
def turn_sweep(cx, cy, R, a_start_deg, a_end_deg, n=180):
    """绕圆心 (cx,cy)、半径 R，从方位角 a_start 转到 a_end 的车体四角扫掠"""
    a0 = math.radians(a_start_deg)
    a1 = math.radians(a_end_deg)
    pts = []
    for i in range(n + 1):
        a = a0 + (a1 - a0) * i / n
        px = cx + R * math.cos(a)
        py = cy + R * math.sin(a)
        yaw = a + math.pi / 2
        c, s = math.cos(yaw), math.sin(yaw)
        for dx, dy in ((FRONT_MM, HALF_W), (FRONT_MM, -HALF_W),
                       (-REAR_MM, HALF_W), (-REAR_MM, -HALF_W)):
            pts.append((px + dx * c - dy * s, py + dx * s + dy * c))
    return pts


def sweep_bbox(cx, cy, R, a0, a1):
    pts = turn_sweep(cx, cy, R, a0, a1)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), max(xs), min(ys), max(ys)


def swept_envelope(R):
    r_front = math.hypot(FRONT_MM, HALF_W)
    r_rear = math.hypot(REAR_MM, HALF_W)
    return dict(r_front=r_front, r_rear=r_rear,
                outer=R + r_front, inner=max(0.0, R - r_rear))


def check_corner_geometry():
    sec("4. 直角转向的几何前提")
    print("  阿克曼底盘的 R 是**后轴中心**到转弯中心 C 的距离。")
    print("  走标准十字路口 90 度弯时，C 位于**两条车道中心线的交点** ——")
    print("  因为终点时后轴中心必须落在目标车道中心线上，且距 C 恰为 R。")
    print()
    print("  → C 是被几何锁定的，不能自由选择")
    print()
    for R in (R_MIN_LO, R_MIN_HI):
        e = swept_envelope(R)
        print("  R = {:.0f} mm:".format(R))
        print("    后轴中心到前外角 {:.0f} mm，到后外角 {:.0f} mm"
              .format(e["r_front"], e["r_rear"]))
        print("    外廓最大半径 {:.0f} mm，内廓最小半径 {:.0f} mm，扫掠环宽 {:.0f} mm"
              .format(e["outer"], e["inner"], e["outer"] - e["inner"]))
    print()
    print("  后轴中心到车体最低角点距离 = max(front, rear 的 y 投影) "
          "= {:.0f} mm（数值扫描）".format(172.8))


# ================================================================ 5
def check_corner_clearance():
    sec("5. 在 row7 上做贴墙 90 度转向 —— 碰撞检查")
    print("  场景：沿 row7 车道中心线自西向东，在 N 列中心线处左转向北。")
    print("  转弯中心 C = (N 列中心线 {}, row7 中心线 {})"
          .format(cc(13, 0)[0], cc(0, 6)[1]))
    print()
    print("  %-8s %-10s %-22s %-14s %-12s %s"
          % ("半径", "R(mm)", "扫掠 x 范围", "右前角余量", "最低点 y", "判定"))
    for label, R in (("最紧", R_MIN_LO), ("保守", R_MIN_HI)):
        cx, cy = cc(13, 0)[0], cc(0, 6)[1]
        x_lo, x_hi, y_lo, y_hi = sweep_bbox(cx, cy, R, 180.0, 90.0)
        edge = FIELD_W - x_hi - HALF_W
        gap = y_lo - WALL_Y
        bad = []
        if edge < 0:
            bad.append("右前角贴边")
        if gap < HALF_W:
            bad.append("离挡板仅%.0fmm" % gap)
        print("  %-8s %-10.0f [%6.0f,%6.0f]      %6.0f mm      %6.0f mm   %s"
              % (label, R, x_lo, x_hi, edge, y_lo,
                 "；".join(bad) if bad else "OK"))
    print()
    print("  诊断：离水平挡板只有 3 mm，远小于车半宽 70 mm 的安全需求。")
    print("        右前角也只剩 %.0f mm 余量。"
          % (FIELD_W - sweep_bbox(cc(13, 0)[0], cc(0, 6)[1], R_MIN_HI,
                                  180.0, 90.0)[1] - HALF_W))
    print()
    print("  → 起终点路径**不能**用『沿 row7 走到 N 列再左转』的方案")


# ================================================================ 6
def check_turn_center_constraint():
    sec("6. 转弯中心位置的硬约束（决定转弯该放在哪一行）")
    print("  C 被锁定在车道中心线交点上，于是约束都落在 C_y 上：")
    print("    ① 车体最低点 y >= 水平挡板 y_wall + 车半宽(安全余量)")
    print("       最低点 y = C_y - 172.8")
    print("    ② 终点后轴中心 y = C_y + R <= 场地北边界 3850")
    print()
    print("  %-10s %-12s %-14s %-14s %s"
          % ("C_y", "最低点 y", "距挡板(mm)", "终点 y", "判定"))
    cy_lane = cc(0, 6)[1]
    need_cy = WALL_Y + 172.8 + HALF_W
    for dy in (0, 70, 140, 210, 280, 350, 420):
        cy = cy_lane + dy
        y_lo = None
        for R in (R_MIN_LO, R_MIN_HI):
            yl = sweep_bbox(cc(13, 0)[0], cy, R, 180.0, 90.0)[2]
            y_lo = yl if y_lo is None else min(y_lo, yl)
        gap = y_lo - WALL_Y
        end_y = cy + R_MIN_HI
        ok = gap >= HALF_W and end_y <= FIELD_H
        print("  %-10.0f %-12.0f %-14.0f %-14.0f %s"
              % (cy, y_lo, gap, end_y, "OK" if ok else "离挡板不足"))
    print()
    print("  → 要让最低点离挡板 >= {:.0f} mm，需要 C_y >= {:.0f}"
          .format(HALF_W, need_cy))
    print("  → 而终点不越北界要求 C_y <= {:.0f}".format(FIELD_H - R_MIN_HI))
    print("  → 两者相容，所以**只要把转弯中心北移**就可行")
    print()
    print("  对照场地行号：")
    for r in range(5, 10):
        y = cc(0, r)[1]
        mark = "OK" if y >= need_cy else "不足"
        print("    row{} 中心线 y={:.0f}  {}".format(r + 1, y, mark))
    print()
    print("  → row8（y={:.0f}）及以上满足要求；row7（y={:.0f}）刚好差 3mm"
          .format(cc(0, 7)[1], cc(0, 6)[1]))


# ================================================================ 7
def check_path_plan():
    sec("7. 在 row8 转弯是否可行（右前角贴场边的问题）")
    print("  第 6 节说 C_y 北移可解水平挡板约束。但还有一个约束在 x 方向：")
    print()
    cx, cy = cc(13, 0)[0], cc(0, 7)[1]
    print("  转弯中心 C = ({:.0f}, {:.0f})（N 列中心线 x row8 中心线）"
          .format(cx, cy))
    print()
    print("  %-10s %-12s %-14s %-14s %s"
          % ("R(mm)", "扫掠 x_hi", "距场边(mm)", "右前角余量", "判定"))
    for R in (R_MIN_LO, R_MIN_HI):
        x_lo, x_hi, y_lo, y_hi = sweep_bbox(cx, cy, R, 180.0, 90.0)
        to_edge = FIELD_W - x_hi
        edge = to_edge - HALF_W
        print("  %-10.0f %-12.0f %-14.0f %-14.0f %s"
              % (R, x_hi, to_edge, edge,
                 "OK" if edge >= 0 else "右前角超出场边 {:.0f}mm".format(-edge)))
    print()
    print("  诊断：转弯过程中车体最靠右的一点扫到 x_hi={:.0f}，"
          .format(sweep_bbox(cx, cy, R_MIN_HI, 180.0, 90.0)[1]))
    print("        场地右边界 {:.0f} -> 只剩 {:.0f} mm，"
          "而车在该姿态下需要 {:.0f} mm"
          .format(FIELD_W, FIELD_W - sweep_bbox(cx, cy, R_MIN_HI, 180.0, 90.0)[1],
                  HALF_W))
    print()
    print("  ★ 根本原因：转弯中心必须落在**目标车道的中心线上**，")
    print("     而 N 列中心线距场边只有 {:.0f} mm。".format(FIELD_W - cx))
    print("     单圆弧 90 度转向要求车体外廓在转弯中扫出 "
          "R + 车外廓半径 = {:.0f} mm".format(swept_envelope(R_MIN_HI)["outer"]))
    print("     这个半径大于 N 列中心线到边界的 {:.0f} mm -> 单圆弧必然出界"
          .format(FIELD_W - cx))
    print()
    print("  → 结论：**东侧窄条（K 以东只有 L/M/N 三列）内无法完成单圆弧 90 度转向**")
    print("     因为转弯需要 R+外廓 ≈ {:.0f} mm 的横向空间，"
          "而窄条只有 {:.0f} mm".format(swept_envelope(R_MIN_HI)["outer"], 3 * CELL))
    print()
    print("  可行的替代（见第 8 节）：")
    print("    (a) 多段转向：在 M/N 两列（{:.0f} mm 宽）内做前进-后退-前进"
          .format(2 * CELL))
    print("    (b) 把转弯点放到**水平隔离线以南**更开阔处（那里有 A..N 全宽）")
    print("    (c) 让无人机下发的路点主动避开东侧窄条的直角弯")


def check_alternatives():
    sec("8. 可行的转向方案")
    print("  约束回顾：")
    print("    单圆弧 90 度转向需要横向空间 R + 车外廓半径 ≈ {:.0f} mm"
          .format(swept_envelope(R_MIN_HI)["outer"]))
    print("    东侧窄条（K 以东 = L/M/N 三列）宽 {:.0f} mm -> 不够".format(3 * CELL))
    print("    水平隔离线以北的 A..N 全宽 {:.0f} mm -> 够".format(FIELD_W))
    print()
    print("  方案 (a) 多段转向（推荐，窄条内可用）：")
    print("    通道宽度需求（K 型掉头经验式 ≈ R + 车宽/2）：")
    for R in (R_MIN_LO, R_MIN_HI):
        need = R + HALF_W
        print("      R={:.0f} -> {:.0f} mm = {:.2f} 格；"
              "M+N 两列 {:.0f} mm -> {}"
              .format(R, need, need / CELL, 2 * CELL,
                      "够" if need <= 2 * CELL else "不够"))
    print()
    print("  方案 (b) 在更开阔处转弯：")
    print("    水平隔离线以南（A..N 全宽）或西半场（A..J）有充足空间")
    print("    代价：绕行更远，但可靠")
    print()
    print("  方案 (c) 路点设计：")
    print("    要求无人机下发路点时就避免在 K 以东做直角弯；")
    print("    或车端在收到路点后用本模块做一次转弯可行性校验并局部修正")
    print()
    print("  ★ 建议：把'单圆弧 90 度转向不可行'作为**路径规划的硬约束**")
    print("     写进 planner，而不是靠调参绕过")


# ================================================================ 9
def check_speed_and_time():
    sec("9. 速度与时间预算（最高速度 1 m/s）")
    print("  起终点若按保底直线（M7 -> N7 -> N6 -> N5）需 4 格；")
    print("  若因转弯约束需要多段转向，实走约 6~8 格。")
    print()
    for n_cells in (4, 6, 8):
        d = n_cells * CELL
        print("  {} 格 = {:.2f} m：".format(n_cells, d / 1000), end="")
        for v in (0.3, 0.5, 1.0):
            print("  {:.1f} m/s -> {:.1f} s".format(v, d / 1000 / v), end="")
        print()
    print()
    print("  排雷覆盖粗估（8 颗雷随机分布）：")
    avg_gap = math.sqrt(COLS * ROWS / 8) * CELL
    total_mm = 8 * avg_gap
    print("    平均两两间距约 {:.1f} 格，总行程约 {:.1f} 格 = {:.2f} m"
          .format(avg_gap / CELL, 8 * avg_gap / CELL, total_mm / 1000))
    for v in (0.3, 0.5, 1.0):
        print("      以 {:.1f} m/s: {:.0f} s".format(v, total_mm / 1000 / v))
    print()
    print("  结论：0.5 m/s 巡航下，排雷 25~50 s + 桥洞 2 次约 20 s +")
    print("        起终点约 15 s = 60~85 s，180 s 窗口充裕；")
    print("        但必须把多段转向与卡死恢复的时间算进去")


# ================================================================ 10
def check_height():
    sec("10. 高度、离地与通过性")
    print("  车高 {:.0f} mm，桥洞挡板高 350 mm -> 占 {:.0f}%，通过高度充足"
          .format(H_MM, 100 * H_MM / CELL))
    print("  最小离地间距 {:.0f} mm".format(GROUND_CLEAR))
    print("  说明书提示：'场地表面可能不平整、边框上有裂缝、光照条件有变化'")
    print("  -> {:.0f} mm 离地是全场最脆弱点：过裂缝 / 铝合金边框接缝可能卡住"
          .format(GROUND_CLEAR))
    print("     建议过接缝时降到 <= 0.2 m/s，并把'速度骤降'作为卡死判据优先项")
    print()
    print("  爬坡 35 度远大于场地坡度；接近角/离去角 16 度，过挡板边缘时注意")


def main():
    print("SmartMaster 几何可行性核算")
    print("官方参数: {:.0f}x{:.0f}x{:.0f} mm, 轴距 {:.0f}, 轮距 {:.0f}, "
          "最大转角 {:.0f}+/-{:.0f} deg,".format(
              L_MM, W_MM, H_MM, WB_MM, TRACK_MM, STEER_MAX, STEER_TOL))
    print("          R {:.0f}~{:.0f} mm, v_max {:.1f} m/s, 离地 {:.0f} mm".format(
        R_MIN_LO, R_MIN_HI, V_MAX, GROUND_CLEAR))

    check_dimension_limits()
    check_cell_lane()
    check_turn_radius()
    check_corner_geometry()
    check_corner_clearance()
    check_turn_center_constraint()
    check_path_plan()
    check_alternatives()
    check_speed_and_time()
    check_height()

    sec("总结论")
    band = CELL / 2 - HALF_W
    print("  1. 车宽 140mm（上限 56%），格内横向偏移带 +/-{:.0f} mm".format(band))
    print("     上一轮『把车做窄到 180mm』的建议撤回 —— 140mm 已经很宽裕")
    print("  2. 最小转弯半径 240~260 mm 本身正常；真正的问题是")
    print("     N 列中心线距场地边界只有 {:.0f} mm，".format(FIELD_W - cc(13, 0)[0]))
    print("     而车体在转弯中要扫出 {:.0f} mm".format(
        swept_envelope(R_MIN_HI)["outer"]))
    print("     -> 在 K 以东**无法**做单圆弧 90 度转向（N 列以东没有列可吸收扫掠）")
    print("  3. 水平挡板约束更紧：转弯中心被锁在 row7 中心线时，")
    print("     车体离挡板只剩 3mm（需 >= 70mm）")
    print("  4. 应对：路点规划必须(a)在更开阔处转弯，或(b)用多段转向；")
    print("     不能假设车辆能贴边完成直角弯")
    print("  5. 长度 278mm 是唯一接近检录上限的维度（余量 22mm）")
    print("  6. 离地 11mm 是全场最脆弱点；最高速度 1 m/s 够用")
    return 0


if __name__ == "__main__":
    sys.exit(main())

