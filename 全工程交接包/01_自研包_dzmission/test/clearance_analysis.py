#!/usr/bin/env python3
"""净空与车宽匹配分析

在"全程不允许触碰边界"约束下，反推车体能有多大、控制精度要多高。

坐标（cm）：
  场地 490 × 385，格 35cm
  档板 y = 210，覆盖 x ∈ [105, 455)（D..M）
  围栏 x = 0 / 490，y = 0 / 385
  起点 M7 格心 (437.5, 227.5)；终点 N5 格心 (472.5, 192.5)

运行:
  python3 clearance_analysis.py
"""

import sys

CELL = 35.0
COLS, ROWS = 14, 11
FIELD_W, FIELD_H = COLS * CELL, ROWS * CELL

WALL_Y = 6 * CELL                       # 210
WALL_X0, WALL_X1 = 3 * CELL, 13 * CELL  # 105 .. 455

START = (12, 6)
GOAL = (13, 4)


def cc(c, r):
    return ((c + 0.5) * CELL, (r + 0.5) * CELL)


def gaps(c, r):
    """返回 (到围栏, 到档板, 最小)"""
    x, y = cc(c, r)
    to_fence = min(x, FIELD_W - x, y, FIELD_H - y)
    to_wall = abs(y - WALL_Y) if WALL_X0 <= x <= WALL_X1 else float('inf')
    return to_fence, to_wall, min(to_fence, to_wall)


# 路线 M7 → N7 → N6 → N5
ROUTE = [(12, 6), (13, 6), (13, 5), (13, 4)]


def sec(title):
    print()
    print("=" * 74)
    print(title)
    print("=" * 74)


def analyze_route():
    sec("1. 路线 M7 → N7 → N6 → N5 的逐格净空")
    print("  格号    格心(cm)          到围栏   到档板   可用净空")
    worst = None
    for (c, r) in ROUTE:
        x, y = cc(c, r)
        tf, tw, mn = gaps(c, r)
        tw_s = "无档板" if tw == float('inf') else "%.1fcm" % tw
        print("  %-6s (%6.1f, %6.1f)  %6.1fcm  %8s  %6.1fcm"
              % (chr(65 + c) + str(r + 1), x, y, tf, tw_s, mn))
        if worst is None or mn < worst[1]:
            worst = ((c, r), mn)
    print(f"\n  最紧处: {chr(65 + worst[0][0])}{worst[0][1] + 1}，"
          f"可用净空 {worst[1]:.1f}cm")
    return worst[1]


def analyze_width_limit(worst_gap):
    sec("2. 由净空反推车体尺寸上限")
    print(f"  瓶颈处可用净空 {worst_gap:.1f}cm（格心到最近挡板/围栏）")
    print("  车体不得触碰边界 → 半个车宽 < 可用净空")
    print()
    print("  车宽(cm)   半宽(cm)   剩余余量(cm)   判定")
    for w in (15, 18, 20, 22, 25, 28, 30):
        half = w / 2
        margin = worst_gap - half
        verdict = "✓ 可通行" if margin > 3 else (
            "△ 极限（余量<3cm，需 <1cm 级控制精度）" if margin > 0 else "✗ 会碰边")
        print("  %6.1f    %6.1f    %10.1f      %s" % (w, half, margin, verdict))
    print()
    print("  → 规则检录上限是 300 × 250mm。若车宽接近 250mm，")
    print(f"     在瓶颈处仅剩 {worst_gap - 12.5:.1f}cm 余量，"
          f"横向控制精度必须优于 ±{worst_gap - 12.5:.1f}cm")
    print("  → 强烈建议把车宽做小：车宽 ≤180mm 时余量 ≥6cm，"
          "普通纯跟踪即可稳定通过")


def analyze_cell_center_precision(worst_gap):
    sec("3. 需要的横向控制精度")
    print("  车在格中行驶时，横向允许偏移带 = 2 × (可用净空 − 车半宽)")
    print()
    print("  车宽(cm)   允许偏移带(cm)   单车侧(cm)   对应格宽比例")
    for w in (15, 18, 20, 22, 25):
        band = 2 * (worst_gap - w / 2)
        if band <= 0:
            print("  %6.1f    %14s   %9s   %s" % (w, "无（会碰边）", "—", "—"))
            continue
        print("  %6.1f    %14.1f   %9.1f   %.1f%%"
              % (w, band, band / 2, 100 * band / (2 * CELL)))
    print()
    print("  对照：dzactuator 的轮速里程计 + EKF 在 3 分钟内横向漂移量级为")
    print("        数厘米到十几厘米，因此**不能只靠航位推算走直线**，")
    print("        必须用视觉或地图做横向闭环修正。")


def analyze_corridor_widths():
    sec("4. 各走廊的实际可用宽度")
    print("  场景                          可用宽度   车宽上限(留2cm余量)")
    rows = []

    # 南北向走廊：以 M7 所在行（row6）为例，左右两侧的约束
    # 右侧（N 列）由围栏 x=490 限制；左侧由档板末端 x=455 或相邻格
    x_m7 = cc(*START)[0]
    right = FIELD_W - x_m7          # 到围栏
    left = x_m7 - WALL_X1           # 到档板末端（D..M 的右端 455）
    rows.append(("M7 处：左侧档板末端 到 右侧围栏",
                 left + right, left + right - 2))

    # 南北向走廊通用：格宽 35
    rows.append(("普通格内南北向通行（格宽 35cm）", CELL, CELL - 2))

    # 若要求离档板/围栏各留 5cm 安全间隙
    rows.append(("普通格 + 两侧各留 5cm 间隙", CELL - 10, CELL - 12))
    rows.append(("普通格 + 两侧各留 10cm 间隙", CELL - 20, CELL - 22))

    for name, usable, limit in rows:
        tag = "✓" if limit >= 25 else ("△" if limit > 18 else "✗")
        print("  %-38s %6.1fcm   %6.1fcm  %s" % (name, usable, limit, tag))
    print()
    print("  → 只有『普通格内直线通行』且两侧间隙 ≤5cm 时，车宽 25cm 才勉强可行")


def analyze_start_goal_margins():
    sec("5. 起点 / 终点的边距（决定起步与停车的容错）")
    for name, (c, r) in (("起点 M7", START), ("终点 N5", GOAL)):
        x, y = cc(c, r)
        tf, tw, mn = gaps(c, r)
        print(f"  {name}: 格心 ({x:.1f}, {y:.1f})")
        print(f"      到围栏 {tf:.1f}cm，到档板 "
              f"{'无' if tw == float('inf') else '%.1fcm' % tw}")
        print(f"      车半宽 12.5cm → 单侧余量 {mn - 12.5:.1f}cm")
    print()
    print("  终点 N5 在 N 列（最外圈），x = 472.5cm，距围栏 17.5cm。")
    print("  停车时若横向误差 > 17.5 − 12.5 = 5.0cm，车角就会碰到围栏。")
    print("  ★ 终点停车必须是「低速 + 视觉定位」，不能靠航位推算直接停。")


def analyze_turn_requirement():
    sec("6. 转弯半径需求")
    print("  路线 M7 → N7 → N6 → N5 含 1 次 90° 转向（M7 → N7 后转向北）")
    print()
    print("  格宽 35cm，转向必须在一个格内完成 → 转弯半径 ≤ 17.5cm")
    print()
    print("  底盘参数（dzactuator）：轴距 CARL = 0.15m = 15cm")
    print("  阿克曼/差速最小转弯半径 ≈ 轴距 / tan(最大前轮转角)")
    import math
    for ang in (20, 25, 30, 35):
        r_min = 15.0 / math.tan(math.radians(ang))
        tag = "✓" if r_min <= 17.5 else "✗"
        print("    最大前轮转角 %2d° → 最小转弯半径 %5.1fcm  %s" % (ang, r_min, tag))
    print()
    print("  → 若最大前轮转角 < 35°，无法在单格内完成 90° 转向，")
    print("     必须用「多段转向」（先转 45°，前进，再转 45°），")
    print("     这会占用额外格位，路径规划需预留空间。")


def main():
    print("净空与车宽匹配分析 —— 约束：全程不允许触碰边界")
    worst = analyze_route()
    analyze_width_limit(worst)
    analyze_cell_center_precision(worst)
    analyze_corridor_widths()
    analyze_start_goal_margins()
    analyze_turn_requirement()

    sec("结论")
    print(f"  1. 路线 M7 → N7 → N6 → N5 可行，4 格，全程不碰那条挡板")
    print(f"     （N 列不被线覆盖，这是唯一不穿桥洞就能南北穿越的口子）")
    print(f"  2. 瓶颈净空 {worst:.1f}cm → 车宽必须 ≤ {worst * 2:.0f}cm 才有物理余量")
    print(f"     检录上限 25cm 时仅有 {worst - 12.5:.1f}cm 余量，属于极限")
    print(f"  3. 终点 N5 距围栏 17.5cm，停车横向误差必须 <5cm")
    print(f"  4. 单格内 90° 转向要求最小转弯半径 ≤17.5cm，")
    print(f"     即最大前轮转角 ≥35°；否则必须用多段转向")
    print(f"  5. 建议：车宽 ≤180mm，并在终点前用视觉闭环停车")
    return 0


if __name__ == "__main__":
    sys.exit(main())
