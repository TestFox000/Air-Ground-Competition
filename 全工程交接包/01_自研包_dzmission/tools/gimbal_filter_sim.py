#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 gimbal_filter_sim.py —— 云台 PID 抖动仿真：学员版 vs 本机版
================================================================================

目的
----
数值回答三个问题：
  1. 学员版（dzrknn_dataprocess）的实际抖动有多大？
  2. 本机版（dzmission/TargetAimer）的抖动有多大？
  3. PID 该怎么调？

被控对象模型
------------
云台是**位置伺服 + 速率限制**：

    if |pos - cmd| 很大:  向 cmd 移动 speed*dt   （速率饱和）
    else:                 到达 cmd                （到位停住）

关键参数（代码常量 / 现场实测）：

    学员版: yaw_ticks_per_deg = 11.378, gimbal_max_speed = 3700 tick/s
    本机版: yaw_ticks_per_deg = 11.111, gimbal_max_speed = 3700 tick/s

★ 视觉延迟（抖动的主因，双方都有）
    流水线 4 帧 @ 30fps = 133 ms
    ★ 纯延迟对控制回路等效于巨大相位滞后，是高增益振荡的根源。

★ 两个曾把结论带偏的建模坑（本文件已修，见各自注释）
    坑1: 把延迟也加到"反馈位置"上。真实系统里 camemo_feedback 是实时的，
         只有视觉误差走流水线。加错会造出"假性发散"（曾算出 108°）。
    坑2: 延迟队列按"控制周期"定长。视觉 30Hz、控制 100Hz 时，队列每 3 步
         才吃到一个新值 -> 真实延迟变成设定的 3 倍。必须按**时间戳**取帧。

用法
----
    python3 tools/gimbal_filter_sim.py            # 默认对比
    python3 tools/gimbal_filter_sim.py --sweep    # 参数扫描
    python3 tools/sim_selftest.py                 # 先跑模型自检
"""

from __future__ import annotations

import argparse
import math
import sys

# ==============================================================================
# 常量
# ==============================================================================
TICK_PER_DEG_STUDENT = 11.378
TICK_PER_DEG_MINE = 11.111
CENTER_TICK = 2047
GIMBAL_MAX_SPEED = 3700        # tick/s（硬件上限，两者相同）

# 视觉：像素 <-> 角度
#   640 宽画面、水平视场约 60° -> f ≈ 554 px，约 9.24 px/°
#   ★ 参数化：它决定"像素误差 -> 角度误差"的比例，直接影响回路增益
PX_PER_DEG_DEFAULT = 9.24

VISION_DELAY_MS = 133          # 4 帧流水线 @30fps（★ 曾误按 15fps 写成 267，
                               #   那正好落在稳定性边界之外，导致假振荡结论）
VISION_HZ = 30.0               # 视觉实际帧率


class Plant:
    """云台位置伺服：向 cmd 移动，受速率限制；整数刻度量化"""

    def __init__(self, tick_per_deg, max_speed, dt):
        self.pos = float(CENTER_TICK)
        self.tick_per_deg = tick_per_deg
        self.max_speed = float(max_speed)
        self.dt = dt

    def step(self, cmd, speed_cmd):
        cmd_q = float(int(round(cmd)))          # 下位机是整数刻度
        eff_speed = min(abs(speed_cmd), self.max_speed)
        err = cmd_q - self.pos
        move = eff_speed * self.dt
        if abs(err) <= move:
            self.pos = cmd_q                    # 到位
        else:
            self.pos += math.copysign(move, err)
        return self.pos


class VisionDelay:
    """固定延迟（按**时间戳**，鲁棒）

    ★ 坑2：定长队列会随"控制频率/视觉频率"的比值放大真实延迟。
      这里存 (时刻, 值)，取不晚于 (now - delay) 的最后一帧。
    """

    def __init__(self, delay_s):
        self.delay_s = float(delay_s)
        self.buf = []

    def push(self, t, v):
        self.buf.append((t, v))
        cutoff = t - self.delay_s * 3.0
        while len(self.buf) > 2 and self.buf[0][0] < cutoff:
            self.buf.pop(0)

    def get(self, t):
        want = t - self.delay_s
        best = None
        for (ts, v) in self.buf:
            if ts <= want:
                best = v
            else:
                break
        if best is None and self.buf:
            best = self.buf[0][1]
        return 0.0 if best is None else best


# ==============================================================================
# 学员版控制器（视觉帧 30Hz 更新位置；主循环 100Hz 更新速度）
# ==============================================================================
class StudentController:
    def __init__(self, kp=8.0, kd=3.0, deadzone_px=12, fine_zone_px=20,
                 fine_kp_scale=0.5, fine_kd_scale=2.0,
                 speed_min=100.0, speed_limit=777.0,
                 ema_alpha=0.3, predict_gain=0.5,
                 px_per_deg=PX_PER_DEG_DEFAULT,
                 tick_per_deg=TICK_PER_DEG_STUDENT):
        self.kp, self.kd = float(kp), float(kd)
        self.dz = float(deadzone_px)
        self.fine_zone = float(fine_zone_px)
        self.fine_kp_scale, self.fine_kd_scale = fine_kp_scale, fine_kd_scale
        self.speed_min = float(speed_min)
        # 代码里 patrolSpeed()=3700*0.35=1295，再乘 track_speed_max_ratio_(0.6)
        #   -> 上限 777 tick/s，只有硬件上限的 21%
        self.speed_limit = float(speed_limit)
        self.alpha, self.predict_gain = float(ema_alpha), float(predict_gain)
        self.px_per_deg = px_per_deg
        self.tick_per_deg = tick_per_deg

        self.filt = 0.0
        self.filt_init = False
        self.prev_filt = 0.0
        self.abs_last = 0.0
        self.cmd_pos = float(CENTER_TICK)

    def reset(self):
        self.filt_init = False
        self.prev_filt = 0.0
        self.abs_last = 0.0
        self.cmd_pos = float(CENTER_TICK)

    def tick(self, err_px, fb_pos, do_position_update):
        cmd_pos = self.cmd_pos

        if do_position_update:
            if not self.filt_init:
                self.filt = err_px
                self.prev_filt = err_px
                self.abs_last = abs(err_px)
                self.filt_init = True
            self.filt = self.alpha * err_px + (1.0 - self.alpha) * self.filt

            eff = 0.0
            if abs(self.filt) > self.dz:
                eff = self.filt - math.copysign(self.dz, self.filt)
            v = self.filt - self.prev_filt
            self.prev_filt = self.filt

            # 位置增量 = 有效偏差的一半 + 预判（基准是**当前**反馈位置）
            if eff != 0.0:
                cmd_pos = fb_pos + eff / 2.0 + v * self.predict_gain

        eff_now = 0.0
        if abs(self.filt) > self.dz:
            eff_now = self.filt - math.copysign(self.dz, self.filt)

        if eff_now == 0.0:
            sp = 0.0
        else:
            abs_err = abs(self.filt)
            kp, kd = self.kp, self.kd
            if abs_err < self.fine_zone:
                kp *= self.fine_kp_scale
                kd *= self.fine_kd_scale
            d_abs = abs_err - self.abs_last
            sp = kp * abs_err + kd * d_abs
            sp = max(self.speed_min, min(self.speed_limit, sp))
        self.abs_last = abs(self.filt)

        self.cmd_pos = cmd_pos
        return cmd_pos, sp


# ==============================================================================
# 本机版控制器（tick 20Hz，位置与速度同频）
# ==============================================================================
class MineController:
    def __init__(self, motor_kp=1000.0, max_speed=3700.0,
                 vision_step_gain=0.3, fast_step_gain=0.7,
                 max_step=20, fast_max_step=80,
                 fast_error_threshold=40,
                 vision_deadband=3, vision_smooth_alpha=0.4, fast_alpha=0.85,
                 add_kd=0.0, speed_cap=None, px_per_deg=PX_PER_DEG_DEFAULT,
                 tick_per_deg=TICK_PER_DEG_MINE):
        self.motor_kp = float(motor_kp)
        self.max_speed = float(max_speed)
        self.step_gain, self.fast_step_gain = vision_step_gain, fast_step_gain
        self.max_step, self.fast_max_step = max_step, fast_max_step
        self.fast_thr = fast_error_threshold
        self.deadband = vision_deadband
        self.alpha, self.fast_alpha = vision_smooth_alpha, fast_alpha
        self.add_kd = add_kd
        self.speed_cap = float(speed_cap) if speed_cap else float(max_speed)
        self.px_per_deg = px_per_deg
        self.tick_per_deg = tick_per_deg
        self.filt = 0.0
        self.filt_init = False
        self.prev_filt = 0.0

    def reset(self):
        self.filt_init = False
        self.prev_filt = 0.0

    def tick(self, err_px, fb_pos):
        a = abs(err_px)
        fast = a > self.fast_thr
        alpha = self.fast_alpha if fast else self.alpha
        gain = self.fast_step_gain if fast else self.step_gain
        mx = self.fast_max_step if fast else self.max_step

        if not self.filt_init:
            self.filt = err_px
            self.prev_filt = err_px
            self.filt_init = True
        else:
            self.filt = alpha * err_px + (1.0 - alpha) * self.filt

        ex = self.filt
        if abs(ex) < self.deadband:
            ex = 0.0
        step = int(round(ex * gain))
        step = max(-mx, min(mx, step))
        cmd_pos = fb_pos + step

        err_ticks = cmd_pos - fb_pos
        sp = abs(self.motor_kp * err_ticks)
        if self.add_kd != 0.0:
            d_px = self.filt - self.prev_filt
            sp += self.add_kd * abs(d_px) * (self.tick_per_deg / self.px_per_deg)
        sp = min(sp, self.speed_cap)
        self.prev_filt = self.filt
        return cmd_pos, sp


# ==============================================================================
# 仿真主循环
# ==============================================================================
def simulate(ctrl, plant, delay, target_pos_tick, duration_s,
             ctrl_dt, px_per_deg):
    """返回轨迹 [(t, target_tick, gimbal_pos, err_true_px, speed)]

    ★ 坑1：延迟只作用在**视觉误差**上；`cmd = fb + inc` 的 fb 是**实时**反馈。
    """
    dt = ctrl_dt
    n = int(duration_s / dt)
    t = 0.0
    traj = []
    vis_accum = 0.0

    def to_px(d_tick):
        return (d_tick / ctrl.tick_per_deg) * px_per_deg

    for _i in range(n):
        fb_now = plant.pos
        true_px = to_px(target_pos_tick - fb_now)

        vis_accum += dt
        if vis_accum >= 1.0 / VISION_HZ - 1e-9:
            vis_accum = 0.0
            pos_update = True
            delay.push(t, true_px)
        else:
            pos_update = False
        vis_px = delay.get(t)

        if isinstance(ctrl, StudentController):
            cmd, sp = ctrl.tick(vis_px, fb_now, pos_update)
        else:
            cmd, sp = ctrl.tick(vis_px, fb_now)

        plant.step(cmd, sp)
        t += dt
        traj.append((t, target_pos_tick, plant.pos, true_px, sp))

    return traj


def analyze(traj, label, tick_per_deg):
    if not traj:
        return {}
    target = traj[0][1]
    start = traj[0][2]
    k = int(len(traj) * 0.7)
    tail = traj[k:] or traj

    pos_tail = [p for (_t, _tg, p, _e, _s) in tail]
    ptp_deg = (max(pos_tail) - min(pos_tail)) / tick_per_deg
    mean_err = sum(abs(p - target) for p in pos_tail) / len(pos_tail) / tick_per_deg
    max_err = max(abs(p - target) for p in pos_tail) / tick_per_deg

    all_pos = [p for (_t, _tg, p, _e, _s) in traj]
    direction = 1.0 if target >= start else -1.0
    if direction > 0:
        overshoot = max(0.0, (max(all_pos) - target) / tick_per_deg)
    else:
        overshoot = max(0.0, (target - min(all_pos)) / tick_per_deg)

    reversals = 0
    prev_sign = 0
    for j in range(1, len(tail)):
        d = tail[j][2] - tail[j - 1][2]
        s = (d > 0) - (d < 0)
        if s != 0:
            if prev_sign != 0 and s != prev_sign:
                reversals += 1
            prev_sign = s

    return {'label': label, 'ptp_deg': ptp_deg, 'overshoot_deg': overshoot,
            'mean_err_deg': mean_err, 'max_err_deg': max_err,
            'reversals': reversals}


def print_result(r):
    if not r:
        return
    print('  %-32s 抖动 %7.2f°  超调 %6.2f°  稳态偏差 %5.2f°  反转 %3d 次'
          % (r['label'], r['ptp_deg'], r['overshoot_deg'],
             r['mean_err_deg'], r['reversals']))


def run_scenario(name, target_delta_deg=3.0, duration=8.0,
                 student_kw=None, mine_kw=None, quiet=False):
    target_tick = CENTER_TICK + target_delta_deg * TICK_PER_DEG_STUDENT
    sdt = 1.0 / 100.0
    p1 = Plant(TICK_PER_DEG_STUDENT, GIMBAL_MAX_SPEED, sdt)
    c1 = StudentController(**(student_kw or {}))
    d1 = VisionDelay(VISION_DELAY_MS / 1000.0)
    tr1 = simulate(c1, p1, d1, target_tick, duration, sdt, c1.px_per_deg)
    r1 = analyze(tr1, '学员版 (Kp=%.1f Kd=%.1f 死区%.0f)'
                 % (c1.kp, c1.kd, c1.dz), TICK_PER_DEG_STUDENT)

    mdt = 1.0 / 20.0
    p2 = Plant(TICK_PER_DEG_MINE, GIMBAL_MAX_SPEED, mdt)
    c2 = MineController(**(mine_kw or {}))
    d2 = VisionDelay(VISION_DELAY_MS / 1000.0)
    tr2 = simulate(c2, p2, d2, target_tick, duration, mdt, c2.px_per_deg)
    r2 = analyze(tr2, '本机版 (motor_kp=%.0f)' % c2.motor_kp, TICK_PER_DEG_MINE)

    if not quiet:
        print('=' * 100)
        print(' 场景: %s   靶标角度偏差 %.1f°   视觉延迟 %d ms   %.2f px/°'
              % (name, target_delta_deg, VISION_DELAY_MS, c1.px_per_deg))
        print('=' * 100)
        print_result(r1)
        print_result(r2)
    return r1, r2


def sweep_kd(kd_list):
    print()
    print('=' * 100)
    print(' 学员版 Kd 扫描（Kp=8 固定，靶标偏 3°）—— 微分阻尼的作用')
    print('=' * 100)
    print('  %-8s %-12s %-12s %-12s %s'
          % ('Kd', '抖动(峰峰)', '超调', '稳态偏差', '方向反转'))
    for kd in kd_list:
        r1, _r2 = run_scenario('sweep', 3.0, quiet=True,
                               student_kw={'kp': 8.0, 'kd': float(kd)})
        print('  %-8.1f %-12.2f %-12.2f %-12.2f %d'
              % (kd, r1['ptp_deg'], r1['overshoot_deg'],
                 r1['mean_err_deg'], r1['reversals']))


def sweep_kp(kp_list):
    print()
    print('=' * 100)
    print(' 学员版 Kp 扫描（Kd=3 固定，靶标偏 3°）')
    print('=' * 100)
    print('  %-8s %-12s %-12s %-12s %s'
          % ('Kp', '抖动(峰峰)', '超调', '稳态偏差', '方向反转'))
    for kp in kp_list:
        r1, _r2 = run_scenario('sweep', 3.0, quiet=True,
                               student_kw={'kp': float(kp), 'kd': 3.0})
        tag = '   <== 当前值' if kp == 8 else ''
        print('  %-8.1f %-12.2f %-12.2f %-12.2f %d%s'
              % (kp, r1['ptp_deg'], r1['overshoot_deg'],
                 r1['mean_err_deg'], r1['reversals'], tag))


def sweep_deadzone():
    print()
    print('=' * 100)
    print(' 学员版 死区扫描（Kp=8, Kd=3，靶标偏 3°）—— 死区对残余抖动的作用')
    print('=' * 100)
    print('  %-10s %-12s %-12s %-12s %s'
          % ('死区(px)', '抖动(峰峰)', '超调', '稳态偏差', '方向反转'))
    for dz in (3, 6, 9, 12, 16, 20, 25):
        r1, _r2 = run_scenario('sweep', 3.0, quiet=True,
                               student_kw={'kp': 8.0, 'kd': 3.0,
                                           'deadzone_px': float(dz)})
        tag = '   <== 当前值' if dz == 12 else ''
        print('  %-10d %-12.2f %-12.2f %-12.2f %d%s'
              % (dz, r1['ptp_deg'], r1['overshoot_deg'],
                 r1['mean_err_deg'], r1['reversals'], tag))


def sweep_mine():
    print()
    print('=' * 100)
    print(' 本机版 motor_kp 扫描（其余默认，靶标偏 3°）—— 当前 1000')
    print('=' * 100)
    print('  %-10s %-12s %-12s %-12s %s'
          % ('motor_kp', '抖动(峰峰)', '超调', '稳态偏差', '方向反转'))
    for kp in (5, 10, 20, 30, 50, 80, 100, 200, 500, 1000):
        _r1, r2 = run_scenario('sweep', 3.0, quiet=True,
                               mine_kw={'motor_kp': float(kp)})
        tag = '   <== 当前值' if kp == 1000 else ''
        print('  %-10.0f %-12.2f %-12.2f %-12.2f %d%s'
              % (kp, r2['ptp_deg'], r2['overshoot_deg'],
                 r2['mean_err_deg'], r2['reversals'], tag))

    print()
    print('=' * 100)
    print(' 本机版 死区 + 速度上限（motor_kp=50）')
    print('=' * 100)
    print('  %-10s %-10s %-12s %-12s %s'
          % ('死区(px)', '速度上限', '抖动(峰峰)', '超调', '方向反转'))
    for db in (3, 6, 9, 12):
        for cap in (777, 1500, 3700):
            _r1, r2 = run_scenario('sweep', 3.0, quiet=True,
                                   mine_kw={'motor_kp': 50.0,
                                            'vision_deadband': db,
                                            'speed_cap': float(cap)})
            print('  %-10d %-10d %-12.2f %-12.2f %d'
                  % (db, cap, r2['ptp_deg'], r2['overshoot_deg'],
                     r2['reversals']))


def main():
    p = argparse.ArgumentParser(description='云台 PID 抖动仿真')
    p.add_argument('--sweep', action='store_true')
    p.add_argument('--delta-deg', type=float, default=3.0)
    args = p.parse_args()

    print('=' * 100)
    print(' 云台 PID 抖动仿真  ——  学员版(dzrknn_dataprocess) vs 本机版(dzmission)')
    print('=' * 100)
    print(' 被控对象 : 位置伺服 + 速率饱和 %d tick/s = %.1f °/s（单步 %.2f°）'
          % (GIMBAL_MAX_SPEED, GIMBAL_MAX_SPEED / TICK_PER_DEG_STUDENT,
             GIMBAL_MAX_SPEED / 100.0 / TICK_PER_DEG_STUDENT))
    print(' 视觉延迟 : %d ms（流水线 4 帧 @30fps）' % VISION_DELAY_MS)
    print(' 换算     : %.2f px/°（参数化）' % PX_PER_DEG_DEFAULT)
    print(' 学员版速度上限: patrolSpeed(1295) x 0.6 = 777 tick/s = 硬件的 21%')
    print()

    run_scenario('靶标偏 %.1f°' % args.delta_deg, args.delta_deg)
    print()
    run_scenario('靶标偏 1.0°（跟踪中微调）', 1.0)

    if args.sweep:
        sweep_kp([2, 4, 6, 8, 10, 12, 16, 20])
        sweep_kd([0, 1, 2, 3, 5, 8, 12, 20])
        sweep_deadzone()
        sweep_mine()

    return 0


if __name__ == '__main__':
    sys.exit(main())
