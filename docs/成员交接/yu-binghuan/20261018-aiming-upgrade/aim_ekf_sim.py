# -*- coding: utf-8 -*-
"""瞄准链路状态估计 A/B：0.6s 长窗最小二乘（现状） vs EKF（吸收 RM 自瞄做法）。

合成实测标靶运动（匀速往返 3.4s + 端停 0.2s）+ 观测噪声 + 丢帧，比较：
  · ω 估计误差（度/s）
  · 超前瞄准落点误差（px）：链路延迟 t_ahead=0.26s 后的预测位置 vs 真值
    分三个口径：全程 / 匀速段 / 丢帧期 —— 直接对应"打不中"（开火框 3.5~18px）

现状口径对齐实车：当前角 θ 用 EMA(alpha=0.3)（C++ 就是这么做的），
两种方法只差【角速度估计方式】。另加一行"完美 ω（用真值）"作为预测上界对照。
"""
from __future__ import annotations

import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "dzrknn_dataprocess"))
from target_ekf import TargetEkf  # noqa: E402

PX_PER_DEG = 0.94 * 11.378      # ≈10.7 px/deg
DT = 1.0 / 30.0
T_AHEAD = 0.26
V_TRUE = 12.0
MOVE_S, DWELL_S = 3.4, 0.2
HALF = MOVE_S + DWELL_S
AMP = V_TRUE * MOVE_S / 2.0
NOISE_DEG = 0.3
T_END = 22.0
EMA_A = 0.3
DROP_AT = (6.0, 12.0, 18.0)
DROP_LEN = 0.4
STEADY_MARGIN = 0.4     # 匀速段判定：前后各 0.4s 速度不变才算"纯匀速"


def truth(t):
    k = int(t // HALF)
    tau = t - k * HALF
    lo, hi = -AMP, AMP
    if k % 2 == 0:
        return (lo + V_TRUE * tau, V_TRUE) if tau <= MOVE_S else (hi, 0.0)
    return (hi - V_TRUE * tau, -V_TRUE) if tau <= MOVE_S else (lo, 0.0)


def gen(rng):
    samples = []
    t = 0.0
    while t < T_END:
        th, w = truth(t)
        dropped = any(d0 <= t < d0 + DROP_LEN for d0 in DROP_AT) or rng.random() < 0.10
        z = None if dropped else th + rng.gauss(0.0, NOISE_DEG)
        samples.append([t, z, th, w, dropped, False])
        t += DT
    # 标定"纯匀速"样本：±STEADY_MARGIN 内速度符号与大小都不变
    for s in samples:
        t, _, _, w = s[0], s[1], s[2], s[3]
        if abs(w) < 1e-6:
            continue
        ok = True
        for m in range(1, int(STEADY_MARGIN / DT) + 1):
            for tt in (t - m * DT, t + m * DT):
                if abs(truth(tt)[1] - w) > 1e-6:
                    ok = False
        s[5] = ok
    return samples


def _ls_slope(buf, t, win=0.6, min_pts=8):
    seg = []
    for (ti, zi) in reversed(buf):
        if t - ti > win:
            break
        if seg and (seg[-1][0] - ti) > 0.15:
            break
        seg.append((ti, zi))
    if len(seg) < min_pts:
        return None
    n = len(seg)
    t0 = seg[0][0]
    ts = [ti - t0 for ti, _ in seg]
    zs = [zi for _, zi in seg]
    st, sz = sum(ts), sum(zs)
    stt = sum(x * x for x in ts)
    stz = sum(ts[i] * zs[i] for i in range(n))
    det = n * stt - st * st
    return (n * stz - st * sz) / det if abs(det) > 1e-9 else None


def run(seq, mode, use_true_omega=False, **ekf_kw):
    buf, ema, last_t, ekf = [], None, None, TargetEkf(**ekf_kw)
    out = []
    for s in seq:
        t, z, th, w, dropped, steady = s
        if mode == "ls":
            if z is not None:
                buf.append((t, z))
                ema = z if ema is None else EMA_A * z + (1 - EMA_A) * ema
            omega = _ls_slope(buf, t)
            theta_est = ema
            if use_true_omega:
                omega = w if omega is not None else None
        else:
            if last_t is not None:
                ekf.predict(t - last_t)
            last_t = t
            if z is not None:
                ekf.update(z)
            theta_est, omega = ekf.theta, ekf.omega
        out.append((t, theta_est, omega, th, w, dropped, steady))
    return out


def metrics(res):
    def rms(a):
        return math.sqrt(sum(x * x for x in a) / len(a)) if a else float("nan")
    w_err, lead_all, lead_steady, lead_drop = [], [], [], []
    for t, th_e, om, th_t, w_t, dropped, steady in res:
        if om is None or th_e is None:
            continue
        if abs(w_t) > 1e-6:
            w_err.append(om - w_t)
        e_px = abs(th_e + om * T_AHEAD - truth(t + T_AHEAD)[0]) * PX_PER_DEG
        lead_all.append(e_px)
        if steady:
            lead_steady.append(e_px)
        if dropped:
            lead_drop.append(e_px)
    return (rms(w_err), rms(lead_all), rms(lead_steady), rms(lead_drop))


def line(name, m):
    print(f"{name:<26s} {m[0]:8.3f} {m[1]:9.2f}px {m[2]:10.2f}px {m[3]:10.2f}px")


if __name__ == "__main__":
    rng = random.Random(20261018)
    seq = gen(rng)
    print(f"{'方法':<26s} {'ω误差':>8s} {'全程':>9s} {'匀速段':>10s} {'丢帧期':>10s}")
    line_depth = "    " + " " * 8 + " " * 9 + " " * 10 + " " * 10
    line("0.6s长窗LS+EMA(现状)", metrics(run(seq, "ls")))
    line("  └ 完美ω(上界对照)", metrics(run(seq, "ls", use_true_omega=True)))

    print("\n--- 扫掠 EKF 参数 ---")
    best = None
    for qw in (6.0, 20.0, 60.0, 200.0):
        for rt in (0.05, 0.10, 0.30):
            mm = metrics(run(seq, "ekf", q_theta=0.1, q_omega=qw, r_theta=rt))
            score = mm[2] + mm[3] * 0.3
            if best is None or score < best[0]:
                best = (score, qw, rt, mm)
    _, qw, rt, mm = best
    print(f"   最优: q_theta=0.1 q_omega={qw} r_theta={rt}")
    line(f"EKF(最优)", mm)
    line("EKF(出厂默认)", metrics(run(seq, "ekf")))
