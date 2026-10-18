# -*- coding: utf-8 -*-
"""执行层掉头侧修复验证：path_executor.start_turn 的 cross+0.0 归一化。

复刻 start_turn 的 ang/turn_sign 数学（格中心坐标 0.35m 间距），对照
修复前（原式）/修复后（+0.0）在 8 向 × {180°, 45°, 90°, 135°} 全组合的
turn_sign 输出：
  - 180° 掉头：修复前北↔南右转/东↔西左转（浮点 -0.0 传播偶然），
    修复后恒左转（确定）；
  - 非 180° 组合：+0.0 为空操作，零变化（交叉验证修复的手术精确性）。
"""
import math

CELL = 0.35


def make_pts(d1, d2, base=(2.0, 2.0)):
    b = (base[0] + d1[0] * CELL, base[1] + d1[1] * CELL)
    c = (b[0] + d2[0] * CELL, b[1] + d2[1] * CELL)
    return base, b, c


def turn_sign(fixed: bool, a, b, c) -> int:
    """复刻 start_turn：0=直通(abs(ang)<0.15)，+1=左，-1=右。"""
    d1x, d1y = b[0] - a[0], b[1] - a[1]
    d2x, d2y = c[0] - b[0], c[1] - b[1]
    cross = d1x * d2y - d1y * d2x
    if fixed:
        cross = cross + 0.0
    ang = math.atan2(cross, d1x * d2x + d1y * d2y)
    if abs(ang) < 0.15:
        return 0
    return 1 if ang > 0 else -1


DIRS = {"东": (1, 0), "东北": (1, 1), "北": (0, 1), "西北": (-1, 1),
        "西": (-1, 0), "西南": (-1, -1), "南": (0, -1), "东南": (1, -1)}
names = list(DIRS)

if __name__ == "__main__":
    print("== 180° 掉头（4 组对向）：修复前 → 修复后 ==")
    n_uturn_changed = 0
    for i, n1 in enumerate(names):
        n2 = names[(i + 4) % 8]
        a, b, c = make_pts(DIRS[n1], DIRS[n2])
        s0 = turn_sign(False, a, b, c)
        s1 = turn_sign(True, a, b, c)
        changed = "  ← 符号归一" if s0 != s1 else ""
        if s0 != s1:
            n_uturn_changed += 1
        side0 = {1: "左", -1: "右", 0: "直通"}[s0]
        side1 = {1: "左", -1: "右", 0: "直通"}[s1]
        print(f"  {n1}→{n2}: {side0} → {side1}{changed}")

    print("\n== 非 180° 组合（45/90/135 全部）：应零变化 ==")
    n_other_changed = 0
    for i, n1 in enumerate(names):
        for k in (1, 2, 3):
            n2 = names[(i + k) % 8]
            a, b, c = make_pts(DIRS[n1], DIRS[n2])
            s0 = turn_sign(False, a, b, c)
            s1 = turn_sign(True, a, b, c)
            if s0 != s1:
                n_other_changed += 1
                print(f"  !! 意外变化: {n1}→{n2}")
    print(f"  非 180° 变化组合 = {n_other_changed}")

    ok = n_uturn_changed == 2 and n_other_changed == 0
    # 期望：修复前恰有 2 组右转（北→南、西→东）——同格 180° 掉头从不同
    # 方向进入竟转不同侧，正是 -0.0 传播偶然性的体现；修复后 8 组恒左。
    # 非 180° 组合 +0.0 为空操作，零变化 = 修复的手术精确性。
    print(f"\n结论：掉头右转→左归一 {n_uturn_changed}/2 组、非 180° 零影响 —— "
          f"{'✓ 通过' if ok else '✗ 未通过'}")
