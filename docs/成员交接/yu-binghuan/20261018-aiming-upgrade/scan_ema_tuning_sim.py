# -*- coding: utf-8 -*-
"""扫描速度(patrol_speed_ratio) 与 EMA(ema_alpha) 离线定值仿真。

Part 1 锁定成功率/时长 vs 扫描速度
    扫描匀速往复（每趟 180°，±90°），目标在视野(FOV)内每帧以概率 p 被检出，
    需连续 confirm 帧同类别才"锁定"（target_confirm_frames）。一趟扫过目标时
    可见帧数 n = FOV/speed × fps；用 DP 精确算"至少一次连续 confirm 命中"概率。
Part 2 EMA 滞后 vs 噪声
    数值测一阶低通对斜坡的**群延迟(帧)**与**噪声衰减**，换算成运动中的瞄准残差(px)。
"""
from __future__ import annotations

import math
import random

MAX_TICK_S = 3700.0          # gimbal_max_speed_
TICKS_PER_DEG = 11.378
FPS = 30.0
FOV_DEG = 65.0               # 相机视场（实测约 60~70°）
CONFIRM = 8                  # target_confirm_frames_
PASS_DEG = 180.0             # 单趟扫幅（-90..+90）
PASSES_PER_CYCLE = 2         # 一趟"左"+一趟"右"
MAX_CYCLES = 3               # scan_continuous_cycles_
PX_PER_DEG = 0.94 * TICKS_PER_DEG
OMEGA_DEG_S = 12.0           # 标靶匀速段角速度（实测口径）


def speed_deg_s(ratio: float) -> float:
    return MAX_TICK_S * ratio / TICKS_PER_DEG


def p_run_at_least_once(n: int, p: float, k: int) -> float:
    """n 次独立试验(成功概率 p)中，至少出现一次"连续 k 次成功"的概率。"""
    if n <= 0:
        return 0.0
    if k <= 0:
        return 1.0
    # f[i] = 还没成功、且当前连续成功数为 i (<k) 的概率
    f = [0.0] * k
    f[0] = 1.0
    for _ in range(n):
        g = [0.0] * k
        for i in range(k):
            if f[i] == 0.0:
                continue
            g[0] += f[i] * (1 - p)              # 失败 → 归零
            if i + 1 >= k:
                pass                            # 成功且达标 → 进入"已成功"吸收态(不计入 f)
            else:
                g[i + 1] += f[i] * p
        f = g
    return 1.0 - sum(f)


def part1():
    print("=" * 78)
    print("Part 1  锁定成功率 / 时长  vs  扫描速度（FOV=%.0f° fps=%.0f 连续需%d帧）"
          % (FOV_DEG, FPS, CONFIRM))
    print("=" * 78)
    ratios = (0.35, 0.30, 0.25, 0.20, 0.15)
    ps = (0.3, 0.5, 0.7, 0.9)
    print(f"{'ratio':>6} {'速度°/s':>8} {'单趟可见帧':>10} {'单趟t(s)':>8}"
          f"{'周期t(s)':>9} | " + " ".join(f"P(p={p:.1f})" for p in ps))
    for r in ratios:
        s = speed_deg_s(r)
        n = int(FOV_DEG / s * FPS)
        t_pass = PASS_DEG / s
        t_cycle = t_pass * PASSES_PER_CYCLE
        cells = []
        for p in ps:
            pp = p_run_at_least_once(n, p, CONFIRM)          # 单趟锁定概率
            total = 1 - (1 - pp) ** (MAX_CYCLES * PASSES_PER_CYCLE)
            cells.append(f"{total:7.1%}")
        print(f"{r:6.2f} {s:8.1f} {n:10d} {t_pass:8.2f} {t_cycle:9.2f} | "
              + " ".join(cells))
    print("\n注: 周期t = 扫完一个来回(±90° 两趟)的时间；P 为 3 个周期(6 趟)内锁定概率。")


def ema_lag_noise(alpha: float, n=400, slope=1.0, noise=0.3, seed=7):
    """数值测一阶低通(EMA)对斜坡的群延迟(帧) 与 输出噪声 std。"""
    rng = random.Random(seed)
    x = [slope * i + rng.gauss(0, noise) for i in range(n)]
    y = []
    s = None
    for v in x:
        s = v if s is None else alpha * v + (1 - alpha) * s
        y.append(s)
    half = n // 2
    d = (sum(x[half:]) - sum(y[half:])) / (n - half) / slope     # 延迟(帧)
    # 去线性趋势后取残差 std（对斜坡直接取 y² 是错的）
    idx = list(range(half, n))
    ys = y[half:]
    m = len(ys)
    sx = sum(idx)
    sy = sum(ys)
    sxx = sum(i * i for i in idx)
    sxy = sum(idx[k] * ys[k] for k in range(m))
    det = m * sxx - sx * sx
    b = (m * sxy - sx * sy) / det
    a0 = (sy - b * sx) / m
    resid = [ys[k] - (a0 + b * idx[k]) for k in range(m)]
    out_noise = math.sqrt(sum(r * r for r in resid) / m)
    return d, out_noise


def part2():
    print("\n" + "=" * 78)
    print("Part 2  EMA 滞后 vs 噪声（对斜坡+噪声，数值测量）")
    print("=" * 78)
    print(f"{'alpha':>6} {'滞后(帧)':>9} {'滞后(ms)':>9} {'残留噪声(|deg|)':>15}"
          f"{'运动残差(px)':>13} {'合成代价*':>10}")
    best = None
    for a in (0.2, 0.3, 0.4, 0.5, 0.6, 0.7):
        d_frames, noise_out = ema_lag_noise(a)
        d_ms = d_frames / FPS * 1000.0
        aim_err_px = OMEGA_DEG_S * (d_frames / FPS) * PX_PER_DEG
        noise_px = noise_out * PX_PER_DEG
        cost = aim_err_px + noise_px            # 残差(结构) + 噪声(抖动) 的合成
        if best is None or cost < best[0]:
            best = (cost, a, d_frames, aim_err_px, noise_px)
        print(f"{a:6.1f} {d_frames:9.2f} {d_ms:9.1f} {noise_out:15.3f} "
              f"{aim_err_px:13.2f} {cost:10.2f}")
    _, a, d, ae, np_ = best
    print(f"\n==> 合成代价最小的 alpha ≈ {a}  (滞后 {d:.2f}帧, 运动残差 {ae:.1f}px, "
          f"噪声 {np_:.1f}px)；现用 0.3。")
    print("注: 运动残差 = omega×滞后×px/deg（运动中系统性偏差）；噪声 = 抖动项。")


if __name__ == "__main__":
    part1()
    part2()
