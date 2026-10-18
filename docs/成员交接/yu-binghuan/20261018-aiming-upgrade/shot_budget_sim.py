# -*- coding: utf-8 -*-
"""10 发弹量预算仿真：给定总弹量、每靶连发上限 k、单发命中率 p，
算「能把几个目标打全」以及「弹匣在多长时间内被耗光」。

现实约束（代码）：max_shots=10 全程硬上限；max_shots_per_target=3；
shot_cooldown_ms=500；**无命中反馈**（只上报已发射数 /shoot_count）⇒ 每个锁定
都会把 k 发打满，不管有没有打中。
"""
from __future__ import annotations

import random

SHOTS_TOTAL = 10
COOLDOWN_S = 0.5


def magazine_lifetime(k, n_targets, cooldown=COOLDOWN_S):
    """无命中反馈：每个目标打满 k 发；弹匣耗光所需时间(s)。"""
    used = min(SHOTS_TOTAL, k * n_targets)
    return used * cooldown, used


def p_all_hit(k, n_targets, p, trials=20000, seed=1):
    """蒙特卡洛：依次交战 n_targets 个目标，每目标最多 k 发（命中即停或打满都试）。
    返回 (至少把 min(n_targets, 可交战数) 个目标打中的概率, 平均用弹)。"""
    rng = random.Random(seed)
    cover = SHOTS_TOTAL // k                    # 弹量允许交战的目标数
    n = min(n_targets, cover)
    ok = 0
    used_sum = 0
    for _ in range(trials):
        shots = 0
        allhit = True
        for _t in range(n):
            hit = False
            for _s in range(k):
                if shots >= SHOTS_TOTAL:
                    break
                shots += 1
                if rng.random() < p:
                    hit = True
                    break                        # 命中即停
            if not hit:
                allhit = False
                break
        used_sum += shots
        if allhit:
            ok += 1
    return ok / trials, used_sum / trials


if __name__ == "__main__":
    print(f"总弹量={SHOTS_TOTAL} 冷却={COOLDOWN_S}s（无命中反馈⇒每锁定打满 k 发）\n")
    print("=== A. 弹匣寿命 & 可交战目标数 ===")
    print(f"{'每靶k发':>7} {'可交战目标数':>12} {'弹匣耗光时间(s)':>16}")
    for k in (1, 2, 3, 5):
        t, used = magazine_lifetime(k, 4)
        print(f"{k:7d} {SHOTS_TOTAL // k:12d} {t:16.1f}")

    print("\n=== B. 打全 N 个目标的概率（单发命中率 p 下，命中即停）===")
    print(f"{'p':>5} {'k':>3} | " + " ".join(f"N={n:>2}" for n in (2, 3, 4)))
    for p in (0.3, 0.6, 0.9):
        for k in (3, 2, 1):
            cells = []
            for n in (2, 3, 4):
                pr, _ = p_all_hit(k, n, p)
                cells.append(f"{pr:5.0%}")
            print(f"{p:5.1f} {k:3d} | " + " ".join(cells))

    print("\n注: p=0.3 ≈ 未标 laser_offset / 无明显提前量（打偏）；"
          "p=0.9 ≈ 超前+offset 标定到位。")
