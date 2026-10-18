# -*- coding: utf-8 -*-
"""CTE 转向修正 离线仿真 + 增益扫掠：为 CtePidController 定标默认增益。

纯 python，无 ROS 依赖。模型（小角度双积分器 + 恒定扰动）：
    psi_dot = (v / L) * tan(steer_cmd + BIAS)   # BIAS=实车恒定机械左偏(未建模)
    e_dot   = v * sin(psi)
以「前置点纯追踪≈0」隔离 CTE 修正环，初始偏差 10cm。
意义：纯P 对双积分器是【无阻尼振荡】，且对恒定左偏留下【稳态误差】——
      正是 D(阻尼) 与 I(消稳态) 要解决的问题。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

PKG = Path(r"c:\Users\29212\Documents\Trae-CN\default\fxxy_extract\发学员"
           r"\src(走直角)\car_localization\car_localization")
sys.path.insert(0, str(PKG))
from cte_pid import CtePidController  # noqa: E402

V, L, DT, T = 0.32, 0.17, 0.05, 6.0
E0 = 0.10
MAX_STEER = 30.0
BIAS = 2.0      # deg，恒定机械左偏（对应实车要靠 STEER_TRIM 压的偏差）


def simulate(e0, updater):
    e, psi = e0, 0.0
    hist = []
    t = 0.0
    while t < T:
        steer = max(-MAX_STEER, min(MAX_STEER, updater(e)))
        psi += (V / L) * math.tan(math.radians(steer + BIAS)) * DT
        e += V * math.sin(psi) * DT
        hist.append((t, e, steer))
        t += DT
    return hist


def metrics(hist, tol=0.005):
    es = [h[1] for h in hist]
    n = len(es)
    last = max(1, int(1.0 / DT))
    rms_all = math.sqrt(sum(x * x for x in es) / n)
    rms_last = math.sqrt(sum(x * x for x in es[-last:]) / len(es[-last:]))
    overshoot = max(0.0, -min(es))
    settle = next((hist[i][0] for i in range(n)
                   if all(abs(x) < tol for x in es[i:])), None)
    return (max(abs(x) for x in es), overshoot, settle,
            max(abs(h[2]) for h in hist), rms_all, rms_last)


def line(name, m):
    st = "不收敛" if m[2] is None else f"{m[2]:.2f}s"
    print(f"{name:26s} 峰值{m[0]*100:6.2f} 超调{m[1]*100:6.2f} "
          f"RMS全程{m[4]*100:6.2f} RMS末段{m[5]*100:6.2f}cm 稳定{st:>7s} 舵{m[3]:4.0f}deg")


if __name__ == "__main__":
    print(f"模型 v={V} L={L} dt={DT} 初始偏差{E0*100:.0f}cm 恒定左偏{BIAS}deg\n")
    print("--- 基线：改动前纯P(Kp=115) ---")
    line("纯P Kp=115", metrics(simulate(E0, lambda e: -e * 115.0)))

    print("\n--- 扫掠 CTE-PID（Kp=115）---")
    best = None
    for ki in (0, 20, 50, 100):
        for kd in (10, 30, 60, 100):
            c = CtePidController(kp=115.0, ki=ki, kd=kd, deadzone=0.005,
                                 integral_limit=0.05, error_threshold=0.05)
            m = metrics(simulate(E0, lambda e: c.update(e, DT)))
            st_v = 9.9 if m[2] is None else m[2]
            score = m[5] * 100 + m[1] * 20 + m[4] * 50 + (st_v * 0.5)
            if best is None or score < best[0]:
                best = (score, ki, kd, m)
            if ki in (0, 50) and kd in (30, 60):
                line(f"  ki={ki:3d} kd={kd:3d}", m)

    _, ki, kd, m = best
    print(f"\n==> 扫掠最优: ki={ki} kd={kd} deadzone=0.005")
    line("   扫掠最优", m)

    print("\n--- 出厂默认 CtePidController()（不传参） ---")
    c = CtePidController()
    line("   出厂默认", metrics(simulate(E0, lambda e: c.update(e, DT))))
