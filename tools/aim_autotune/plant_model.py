#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云台自瞄 PID 的被控对象模型 + 离线扫参（不依赖 ROS，可在 PC 上跑）。

模型依据（全部来自 dzrknn_dataprocess.cpp 实读，非教科书假设）：
  * 控制律：u = Kp*e + Kd*d(e)/dt + ∫Ki*e*dt      （u 单位 ticks/s，e 单位 px）
             Δθ = u*dt  → 云台目标位置增量；另有 |u|≤cap(2000) 与 min_step 闸
  * 执行器：CameMo_Conl = [Position_0, Position_1, Speed_0, Speed_1]，
            位置+速度协议 → 用"一阶趋近 + 速度限幅"建模
  * 视觉  ：offset_center 30 Hz，端到端延迟约 0.26 s（源码注释实测口径）
  * 几何  ：dx ≈ -a * Δθ（a = px/tick，由 P1 开环阶跃实测）

用法：
  python3 plant_model.py --sweep                     # 扫参，输出前 10 组
  python3 plant_model.py --eval 12,1.0,1.8 3,0.3,0.6 # 对比指定参数
"""
import argparse
import math
import random
from dataclasses import dataclass


@dataclass
class Plant:
    a: float = 0.35          # px / tick（正：云台正转 → 图像左移，符号由实测定）
    tau_servo: float = 0.08  # s，伺服一阶时间常数（P1 实测定）
    delay: float = 0.26      # s，视觉+链路端到端延迟（源码注释口径）
    vmax: float = 2000.0     # ticks/s，trackSpeedCap
    deadzone: float = 5.0    # px，track_deadzone_px
    dt: float = 1.0 / 30.0   # s，视觉帧间隔
    noise_px: float = 0.5    # px，检测噪声（P1 实测定）
    d_alpha: float = 0.3     # 微分一阶低通


def simulate(plant: Plant, kp, ki, kd, dx0=60.0, duration=6.0, seed=0):
    """按 C++ pidStep 的语义逐帧推进，返回 (t, dx, u) 三条曲线。"""
    rng = random.Random(seed)
    nd = max(1, int(round(plant.delay / plant.dt)))
    n = int(duration / plant.dt)

    p = 0.0                 # 云台实际位置(ticks)
    p_cmd = 0.0             # 目标位置(ticks)
    speed_cmd = 0.0
    dx_true = dx0           # 真实像素误差
    hist = [dx0] * nd       # 视觉延迟队列
    e1 = None
    integ = 0.0
    dfilt = 0.0
    ts, dxs, us = [], [], []

    for k in range(n):
        t = k * plant.dt
        dx_meas = hist[0]       # 延迟后的测量
        e_raw = dx_meas
        e = 0.0 if abs(e_raw) <= plant.deadzone else math.copysign(abs(e_raw) - plant.deadzone, e_raw)

        # ---- pidStep ----
        if e1 is None:
            e1 = e
        d_raw = (e - e1) / plant.dt
        dfilt = plant.d_alpha * dfilt + (1.0 - plant.d_alpha) * d_raw
        u_pd = kp * e + kd * dfilt
        if abs(u_pd) < 0.95 * plant.vmax:
            integ += ki * e * plant.dt
            integ = max(-600.0, min(600.0, integ))
        u = u_pd + integ
        e1 = e

        # ---- 位置增量（min_step 闸：|Δθ|<1 不下发）----
        inc = u * plant.dt
        if abs(inc) >= 1.0 and abs(u) > 1e-9:
            p_cmd += inc
            speed_cmd = min(plant.vmax, max(0.0, abs(u)))
        # ---- 伺服：一阶趋近 + 速度限幅 ----
        err = p_cmd - p
        step = err / plant.tau_servo * plant.dt
        step = max(-speed_cmd * plant.dt if speed_cmd > 0 else 0.0,
                   min(speed_cmd * plant.dt if speed_cmd > 0 else 0.0, step))
        p += step
        # ---- 几何 ----
        dx_true = dx0 - plant.a * p
        dx_true += rng.gauss(0, plant.noise_px)
        hist.append(dx_true)
        dx_out = hist.pop(0)
        ts.append(t)
        dxs.append(dx_out)
        us.append(u)
    return ts, dxs, us


def metrics(ts, dxs, tol=5.0, hold=0.5):
    """判定指标：调节时间/超调/IAE/振荡次数/稳态 RMS。"""
    dt = ts[1] - ts[0]
    x0 = dxs[0]
    # 调节时间：此后一直 |dx|<=tol 的第一个时刻
    settle = None
    win = int(hold / dt)
    for i in range(len(dxs)):
        if all(abs(v) <= tol for v in dxs[i:i + win]) and i + win <= len(dxs):
            settle = ts[i]
            break
    # 超调（反向穿过 0 的最大幅度 / 初始误差）
    sign0 = 1.0 if x0 >= 0 else -1.0
    over = 0.0
    crossed = False
    for v in dxs:
        if v * sign0 < 0:
            crossed = True
        if crossed:
            over = max(over, -v * sign0)
    over_pct = 100.0 * over / max(abs(x0), 1e-6)
    iae = sum(abs(v) for v in dxs) * dt
    # 振荡：0.3 s 之后 dx 穿越 0 的次数
    zc = 0
    prev = dxs[0]
    for t, v in zip(ts, dxs):
        if t > 0.3 and prev != 0 and v * prev < 0:
            zc += 1
        prev = v
    tail = dxs[-int(2.0 / dt):]
    rms = math.sqrt(sum(v * v for v in tail) / max(len(tail), 1))
    return dict(settle=settle, overshoot_pct=over_pct, iae=iae, zero_cross=zc, rms_tail=rms)


def score(m):
    """越小越好；硬约束用大罚分。"""
    s = 0.0
    s += (m["settle"] if m["settle"] is not None else 6.0)
    s += 0.02 * m["iae"]
    s += 0.02 * m["overshoot_pct"]
    s += 0.08 * m["zero_cross"]
    s += 0.05 * m["rms_tail"]
    if m["overshoot_pct"] > 25:
        s += 2.0
    if m["settle"] is None:
        s += 3.0
    if m["zero_cross"] > 6:
        s += 1.5
    return s


def kp_stability_bound(plant: Plant):
    """积分器+纯延迟的相位裕度判据：a*Kp*θ < π/2 ⇒ Kp < 1.57/(a·θ)。"""
    return math.pi / 2.0 / (plant.a * plant.delay)


def sweep(plant: Plant, dx0=60.0):
    kps = [0.5, 0.8, 1.2, 1.8, 2.5, 3.5, 5.0, 7.0, 10.0, 14.0]
    kis = [0.0, 0.2, 0.5, 1.0, 2.0]
    kds = [0.0, 0.3, 0.6, 1.0, 1.8, 3.0]
    rows = []
    for kp in kps:
        for ki in kis:
            for kd in kds:
                ts, dxs, _ = simulate(plant, kp, ki, kd, dx0=dx0, duration=5.0, seed=1)
                m = metrics(ts, dxs)
                rows.append((score(m), kp, ki, kd, m))
    rows.sort(key=lambda r: r[0])
    return rows

def main():
    ap = argparse.ArgumentParser(description="云台自瞄 PID 被控对象模型与离线扫参")
    ap.add_argument("--a", type=float, default=0.35, help="px/tick（P1 实测）")
    ap.add_argument("--tau", type=float, default=0.08, help="伺服时间常数 s（P1 实测）")
    ap.add_argument("--delay", type=float, default=0.26, help="端到端延迟 s")
    ap.add_argument("--dx0", type=float, default=60.0, help="初始像素误差")
    ap.add_argument("--eval", type=str, default="", help="逗号分隔的 kp,ki,kd 组，用 ; 分组")
    ap.add_argument("--sweep", action="store_true")
    args = ap.parse_args()

    p = Plant(a=args.a, tau_servo=args.tau, delay=args.delay)
    print("=== 模型参数 ===")
    print("a=%.3f px/tick  tau=%.3f s  delay=%.3f s  vmax=%.0f tick/s  deadzone=%.1f px"
          % (p.a, p.tau_servo, p.delay, p.vmax, p.deadzone))
    print("理论 Kp 稳定上界（积分器+纯延迟，相位裕度 0）: %.2f" % kp_stability_bound(p))

    if args.eval:
        print("\n=== 指定参数对比 ===")
        print("%-16s %8s %8s %8s %8s %8s %8s" % ("kp,ki,kd", "settle", "over%", "IAE", "ZC", "rms", "score"))
        for grp in args.eval.split(";"):
            parts = [float(x) for x in grp.strip().split(",")]
            if len(parts) != 3:
                continue
            ts, dxs, _ = simulate(p, *parts, dx0=args.dx0, duration=5.0, seed=1)
            m = metrics(ts, dxs)
            print("%-16s %8s %8.1f %8.2f %8d %8.2f %8.2f" % (
                "%.2f,%.2f,%.2f" % tuple(parts),
                ("%.2f" % m["settle"]) if m["settle"] is not None else ">5",
                m["overshoot_pct"], m["iae"], m["zero_cross"], m["rms_tail"], score(m)))

    if args.sweep:
        rows = sweep(p, dx0=args.dx0)
        print("\n=== 扫参前 10（按 score 升序）===")
        print("%-16s %8s %8s %8s %8s %8s %8s" % ("kp,ki,kd", "settle", "over%", "IAE", "ZC", "rms", "score"))
        for sc, kp, ki, kd, m in rows[:10]:
            print("%-16s %8s %8.1f %8.2f %8d %8.2f %8.2f" % (
                "%.2f,%.2f,%.2f" % (kp, ki, kd),
                ("%.2f" % m["settle"]) if m["settle"] is not None else ">5",
                m["overshoot_pct"], m["iae"], m["zero_cross"], m["rms_tail"], sc))
        print("\n注：本模型只用于【缩小候选集】，最终以实车 P3 闭环验收为准。")


if __name__ == "__main__":
    main()
