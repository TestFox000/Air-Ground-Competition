#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
 sim_selftest.py —— 仿真器的**模型自检**

★ 为什么必须做这一步：
   第一版仿真跑出"学员版抖动 108°"—— 这在物理上不可能
   （云台行程只有 ±180°，而真实系统能正常工作）。
   凡是仿真结果超出物理常识，**先怀疑模型，再怀疑被控对象**。

本自检用几个**已知答案**的用例来验证被控对象模型：

  用例1: 纯比例控制，增益很小 -> 必须单调收敛、无超调
  用例2: 纯比例控制，增益适中 -> 可以超调，但必须收敛
  用例3: 增益过大         -> 必然发散（验证模型确实能表现不稳定）
  用例4: 无延迟 + 增益 1.0 -> 一步到位（验证"位置伺服"语义）
  用例5: 位置指令 = 反馈 + 增量（学员版核心机制）在**无延迟**下必须收敛

如果用例 1/2/4/5 任一失败，说明被控对象或仿真循环写错了，
此时任何"抖动有多大"的结论都不可信。
"""

from __future__ import annotations

import math
import sys

CENTER = 2047.0
TICK_PER_DEG = 11.378
MAX_SPEED = 3700.0


class Plant:
    def __init__(self, dt, max_speed=MAX_SPEED):
        self.pos = CENTER
        self.dt = dt
        self.max_speed = max_speed

    def step(self, cmd, speed):
        cmd_q = float(int(round(cmd)))
        sp = min(abs(speed), self.max_speed)
        move = sp * self.dt
        err = cmd_q - self.pos
        if abs(err) <= move:
            self.pos = cmd_q
        else:
            self.pos += math.copysign(move, err)
        return self.pos


def run_p_control(kp, delay_frames, n=600, dt=1.0 / 100.0,
                  target_deg=15.0, speed_fn=None):
    """纯比例控制器 + 可选延迟

    speed_fn(err_ticks) -> speed；默认 speed = kp * |err|
    """
    target = CENTER + target_deg * TICK_PER_DEG
    plant = Plant(dt)
    buf = [0.0] * delay_frames
    hist = []
    for i in range(n):
        err_true = target - plant.pos
        if delay_frames:
            buf.append(err_true)
            err = buf.pop(0)
        else:
            err = err_true
        sp = speed_fn(err) if speed_fn else abs(kp * err)
        plant.step(plant.pos + err, sp)     # cmd = 当前位置 + 误差 = 目标
        hist.append(plant.pos)
    return target, hist


def report(name, ok, detail=''):
    print('  [%s] %-46s %s' % ('OK' if ok else 'XX', name, detail))
    return ok


def main():
    print('=' * 84)
    print(' 仿真器模型自检 —— 用已知答案验证"云台位置伺服"模型')
    print('=' * 84)
    print(' 被控对象: 位置伺服 + 速率饱和 %.0f tick/s, dt=10ms' % MAX_SPEED)
    print(' 行程: 3700 tick/s = %.1f °/s（关键：单步最多走 %.2f°）'
          % (MAX_SPEED / TICK_PER_DEG, MAX_SPEED * 0.01 / TICK_PER_DEG))
    print()

    allok = True

    # ---------------- 用例1: 小增益应渐近收敛（误差随时间单调减小）----------------
    print('用例1: 纯比例 Kp=0.5（小增益）—— 期望**渐近收敛**（非一步到位）')
    target, hist = run_p_control(kp=0.5, delay_frames=0, n=1500)
    # 用小增益时误差按几何级数衰减，末误差不会恰好为 0，只看趋势
    e0 = abs(hist[0] - target) / TICK_PER_DEG
    e1 = abs(hist[100] - target) / TICK_PER_DEG
    e2 = abs(hist[-1] - target) / TICK_PER_DEG
    rev = sum(1 for i in range(1, len(hist) - 1)
              if (hist[i] - hist[i - 1]) * (hist[i + 1] - hist[i]) < 0)
    allok &= report('小增益渐近收敛且无超调',
                    e2 < e1 < e0 and rev == 0,
                    '%.2f° -> %.2f° -> %.2f°, 反转 %d 次' % (e0, e1, e2, rev))

    # ---------------- 用例2: 中等增益无延迟应收敛且无残差 ----------------
    print()
    print('用例2: 纯比例 Kp=8 —— 无延迟时必须收敛')
    target, hist = run_p_control(kp=8, delay_frames=0)
    final_err = abs(hist[-1] - target) / TICK_PER_DEG
    allok &= report('无延迟中等增益收敛', final_err < 0.5,
                    '末误差 %.3f°' % final_err)

    # ---------------- 用例3: 有延迟 -> 有界极限环（不是发散！）----------------
    print()
    print('用例3: Kp=8 + 267ms 延迟 —— 期望出现**有界振荡(极限环)**')
    print('        判据：振荡有界且在 [0.5°, 60°] 内（超出说明模型不合理）')
    target, hist = run_p_control(kp=8, delay_frames=8, n=1500)
    tail = hist[-400:]
    amp = (max(tail) - min(tail)) / TICK_PER_DEG
    # 有界：末段幅度不应继续增长
    first_half = hist[600:1000]
    amp_early = (max(first_half) - min(first_half)) / TICK_PER_DEG
    bounded = amp < 60.0
    stable_amp = abs(amp - amp_early) / max(amp, 1e-9) < 0.5
    allok &= report('延迟产生有界极限环', bounded and stable_amp,
                    '末段峰峰 %.1f°（早期 %.1f°）' % (amp, amp_early))

    # ---------------- 用例4: 速率限制的单步位移必须正确 ----------------
    print()
    print('用例4: 速率限制 —— 单步位移必须 = speed*dt（而非一步到位）')
    target, hist = run_p_control(kp=1e9, delay_frames=0, n=200)
    step_deg = (hist[0] - CENTER) / TICK_PER_DEG
    expect = MAX_SPEED * (1.0 / 100.0) / TICK_PER_DEG
    allok &= report('单步位移符合速率限制',
                    abs(step_deg - expect) < 0.05,
                    '实测 %.2f°/步, 理论 %.2f°/步' % (step_deg, expect))

    # ---------------- 用例5: 学员版核心机制（无延迟）必须收敛 ----------------
    print()
    print('用例5: 学员版核心机制 cmd = fb + eff/2，**无延迟**必须收敛')
    target = CENTER + 15.0 * TICK_PER_DEG
    plant = Plant(1.0 / 100.0)
    filt = None
    prev_filt = 0.0
    dz = 12.0
    kp, kd = 8.0, 3.0
    sp_min, sp_max = 100.0, 2220.0
    filt_init = False
    abs_last = 0.0
    cmd = CENTER
    hist = []
    for i in range(1500):
        err_px = (target - plant.pos) / TICK_PER_DEG * 9.24   # px
        if not filt_init:
            filt = err_px
            prev_filt = err_px
            abs_last = abs(err_px)
            filt_init = True
        filt = 0.3 * err_px + 0.7 * filt
        eff = 0.0
        if abs(filt) > dz:
            eff = filt - math.copysign(dz, filt)
        v = filt - prev_filt
        prev_filt = filt
        if eff != 0.0:
            cmd = plant.pos + eff / 2.0 + v * 0.5
        eff_now = 0.0
        if abs(filt) > dz:
            eff_now = filt - math.copysign(dz, filt)
        if eff_now == 0.0:
            sp = 0.0
        else:
            ae = abs(filt)
            k_, d_ = kp, kd
            if ae < 20.0:
                k_ *= 0.5
                d_ *= 2.0
            sp = k_ * ae + d_ * (ae - abs_last)
            sp = max(sp_min, min(sp_max, sp))
        abs_last = abs(filt)
        plant.step(cmd, sp)
        hist.append(plant.pos)
    amp_tail = (max(hist[-100:]) - min(hist[-100:])) / TICK_PER_DEG
    final_err = abs(hist[-1] - target) / TICK_PER_DEG
    allok &= report('学员版机制无延迟收敛', amp_tail < 1.0 and final_err < 2.0,
                    '末段峰峰 %.2f°, 末误差 %.2f°' % (amp_tail, final_err))

    print()
    print('=' * 84)
    if allok:
        print(' ✓ 模型自检全部通过 —— 可以用它比较两个实现')
    else:
        print(' ✗ 模型自检失败 —— 必须先修仿真器，否则"抖动多大"的结论不可信')
    print('=' * 84)
    return 0 if allok else 1


if __name__ == '__main__':
    sys.exit(main())
