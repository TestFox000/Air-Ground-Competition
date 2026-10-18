# -*- coding: utf-8 -*-
"""开火链「兑现率」离线仿真 —— 量化"条件满足"到"真的打出子弹"的概率。

开火需【全部同时成立】且连续 hold 帧（fire_hold_frames），每发后进冷却（shot_cooldown_ms）：
    target_confirmed  : 连续确认帧
    in_fire_zone      : |dx_filt(+lead) - laser_offset| <= fire_dz（动态框按框宽）
    aim_ready         : 两键(use_external_fire_gate) ? gate&ready : 画面稳定(|Δe|<=stable_thr)
    ammo/cooldown     : 弹药与冷却

模型（残差 = 滤波后瞄准偏差，AR(1)，对齐 EMA α）：
    e_t = α·x_t + (1-α)·e_{t-1},  x_t = bias + 噪声
    bias        = 移动靶未超前时的稳态滞后(px)，超前生效≈0；静止靶≈0
    Δe 判据     = α·(视在运动 + 噪声差) —— 用于"画面稳定"判据
"""
from __future__ import annotations

import random

FPS = 30.0
V_FRAME = 12.0 * 10.7 / 30.0      # 移动靶每帧视在位移(px) ≈4.27
ALPHA = 0.3                        # ema_alpha
STABLE_THR = 1.5                   # fire_stable_px_per_frame
COOLDOWN = 0.5                     # shot_cooldown_ms=500
SIGMA = 2.0                        # 检测+滤波后抖动(px)


def run(box, hold, two_key, lead_on, target="moving", T=10.0, seed=1):
    rng = random.Random(seed)
    n = int(T * FPS)
    bias = 0.0 if (target == "static" or lead_on) else 9.0   # 未超时的移动靶稳态滞后
    apparent = 0.0 if (target == "static" or lead_on) else V_FRAME
    e = bias
    hold_cnt, fires, last, t_first = 0, 0, -1e9, None
    for k in range(n):
        t = k / FPS
        e = ALPHA * (bias + rng.gauss(0.0, SIGMA)) + (1 - ALPHA) * e
        de = ALPHA * (apparent + rng.gauss(0.0, SIGMA * 1.4))
        ok = abs(e) <= box
        if not two_key:
            ok = ok and abs(de) <= STABLE_THR
        hold_cnt = hold_cnt + 1 if ok else 0
        if hold_cnt >= hold and (t - last) >= COOLDOWN:
            fires += 1
            last = t
            hold_cnt = 0
            if t_first is None:
                t_first = t
    return fires, t_first


def p_at_least_one(box, hold, two_key, lead_on, target, T=3.0, trials=400):
    hit = 0
    for s in range(trials):
        _, t1 = run(box, hold, two_key, lead_on, target, T=30.0, seed=s)
        if t1 is not None and t1 <= T:
            hit += 1
    return hit / trials


if __name__ == "__main__":
    print(f"参数: V_FRAME={V_FRAME:.2f}px α={ALPHA} stable_thr={STABLE_THR} "
          f"cooldown={COOLDOWN}s σ={SIGMA}px\n")

    print("=== A. 现状（框=3.5px, hold=5, 两键关, 无超前）===")
    for tg in ("static", "moving"):
        f, t1 = run(3.5, 5, False, False, tg)
        print(f"  {tg:7s} 10s 内开火 {f} 发；3s 内至少 1 发概率 "
              f"{p_at_least_one(3.5,5,False,False,tg):.0%}")

    print("\n=== B. 只改开火框（hold=5, 两键关, 无超前）===")
    print(f"{'框(px)':>8} " + "  ".join(f"{tg:>8s}" for tg in ("static", "moving")))
    for box in (3.5, 5.0, 6.0, 10.0, 18.0):
        cells = []
        for tg in ("static", "moving"):
            f, _ = run(box, 5, False, False, tg)
            cells.append(f"{f:>8d}")
        print(f"{box:8.1f} " + "  ".join(cells))

    print("\n=== C. 开两键（绕过'画面稳定'）+ 超前（框=6px）===")
    print(f"{'hold':>6} {'两键':>5} {'超前':>5} | {'静止靶 10s发数':>14} {'移动靶 10s发数':>14}"
          f" {'移动靶 3s内≥1发':>16}")
    for hold in (5, 3, 2, 1):
        for two_key, lead in ((True, False), (True, True), (False, True)):
            fs, _ = run(6.0, hold, two_key, lead, "static")
            fm, _ = run(6.0, hold, two_key, lead, "moving")
            pm = p_at_least_one(6.0, hold, two_key, lead, "moving")
            print(f"{hold:6d} {str(two_key):>5} {str(lead):>5} | {fs:14d} {fm:14d} {pm:16.0%}")
