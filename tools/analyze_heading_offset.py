#!/usr/bin/env python3
"""从探针 dump 里复现"恒定朝向偏差"那张表（`docs/朝向偏差根因-20261001.md`）。

用法：
    python tools/analyze_heading_offset.py logs/_pe_review/dump.txt

dump 格式（`tools/car_probe_turn.sh` 内嵌订阅器落盘）：
    twist <t> <linear.x> <angular.z>
    odom  <t> <x> <y> <yaw_deg>

它做两件事：
  1. 打出"定位上报的 yaw"随时间的变化（这是 raw_map_yaw 的输入，**不是**执行器的 belief）
  2. 按 path_executor 的复位逻辑推算执行器的 belief，并与"实际走的方向"对比

⚠️ 使用前必读（2026-10-01 的局限，别当结论用）：
    表里"实际走的方向"是由**同一路 yaw 积分出的位置**反算的，而"相信的 yaw"来自同一路 yaw
    再加一个常数偏移 —— 因此两者之差**按定义就是常数**，它**不能自证**"车的真实朝向差了 68.6°"。
    真正的判据是物理实验：把车原地转 ~69° 重跑，看是否直线到目标。
    （详见 docs/接手说明.md §3.2、docs/朝向偏差根因-20261001.md）
"""
from __future__ import annotations

import math
import sys


def load(path: str):
    """读 dump：返回 (twist, odom) 两个列表；各元素为元组 (t, ...)。"""
    tw, od = [], []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            p = line.split()
            if len(p) == 4 and p[0] == "twist":
                tw.append((float(p[1]), float(p[2]), float(p[3])))
            elif len(p) == 5 and p[0] == "odom":
                od.append((float(p[1]), float(p[2]), float(p[3]), float(p[4])))
    return tw, od


def main(argv=None) -> int:
    # Windows 控制台默认 GBK：中文一打印就 UnicodeEncodeError（且报错只出现在子进程里）
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        print(__doc__)
        return 2
    path = argv[0]
    tw, od = load(path)
    if not od:
        print("  [FAIL] dump 里没有 odom 样本（采样没接上？见 tools/car_probe_turn.sh 头注）")
        return 1

    print("=== 定位上报的 yaw 随时间（raw_map_yaw 的输入，不是执行器的 belief）===")
    t = 0.0
    while t < od[-1][0]:
        a = min(od, key=lambda o: abs(o[0] - t))
        print("  t=%5.1fs  pos(%6.2f,%6.2f)  上报 yaw %+7.1f°" % (a[0], a[1], a[2], a[3]))
        t += 2.0

    # 复位发生在直行开始前（约 t=15s，车开始动的 t=17s 之前）
    # 找复位时刻：位置开始变化前最后一个稳定帧
    i0 = 0
    for i in range(1, len(od)):
        if math.dist((od[i][1], od[i][2]), (od[0][1], od[0][2])) > 0.05:
            i0 = i
            break
    print("\n车开始移动: t=%.1fs  此前的上报 yaw（≈复位时刻的值 S0）: %+.1f°"
          % (od[i0][0], od[i0 - 1][3]))
    S0 = od[i0 - 1][3]

    print("\n=== 执行器的 belief 推算 ===")
    print("  belief = 上报yaw + 90(RAW偏移) + snap，其中 snap = 90 - S0")
    print("  即 belief = 上报yaw + 180 - S0")
    print()
    print("   t     上报yaw    执行器belief    实际走的方向     belief vs 实际")
    for t in range(19, 34, 2):
        a = min(od, key=lambda o: abs(o[0] - t))
        b = min(od, key=lambda o: abs(o[0] - (t + 0.6)))
        dx, dy = b[1] - a[1], b[2] - a[2]
        if math.hypot(dx, dy) < 0.02:
            continue
        move = math.degrees(math.atan2(dy, dx))
        belief = a[3] + 180.0 - S0
        belief = (belief + 180) % 360 - 180
        diff = (move - belief + 180) % 360 - 180
        print("  %4.1f   %+7.1f°    %+8.1f°      %+8.1f°        %+7.1f°"
              % (t, a[3], belief, move, diff))
    print()
    print("  注意：最后一列恒定是**构造性质**（两列同源），不能当作真实朝向差的证据。")
    print("     决定性判据是物理实验：把车原地转 ~69 度重跑，看是否直线到目标。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
