# -*- coding: utf-8 -*-
"""离线回归：平滑律（EKF+100Hz 斜坡）vs 旧律（30Hz 台阶 PID），267ms 视觉延迟。
场景：轨道靶 ±100px/s 往返（0.7m 距离、140px/m 口径），检测噪声 σ=2px，30Hz 视觉。
指标：稳态 RMS 误差 / 超调 / 指令连续性（相邻指令不变的占比）/ 开火框内时间占比。
"""
import math
import random

random.seed(42)

# ---- 公共参数（与 robot.yaml / 代码一致）----
FPS = 30.0
DT_V = 1.0 / FPS
LAT = 0.267          # 视觉链路延迟 s
KPT = 0.94           # px/tick（tick 与 px 在本模型同纲）
EMA = 0.45
DZ = 3.0             # 死区 px
KP, KI, KD = 10.0, 0.5, 2.0
PREDICT = 0.2
ALPHA, BETA = 0.6, 0.2
T_LOOK = 0.267       # EKF 前瞻 = vision_latency_ms
K_SP, FF, SLEW = 8.0, 1.0, 2000.0
INNER_HZ = 100.0
DT_I = 1.0 / INNER_HZ
TAU_ACT = 0.04       # 执行器一阶响应 40ms
V_CAP = 1880.0       # px/s（2000 tick/s × 0.94）
FIRE_BOX = 18.0

def target_pos(t):
    # 轨道靶往返：半程 0.9s、±100px/s
    p = 100.0 * math.sin(2.0 * math.pi * t / 3.6)
    return p

def run_old_law(t_end=14.0):
    g = 0.0                 # 云台实际位置（图像坐标）
    e_ema = None
    i_state = 0.0
    d_prev_eff = 0.0
    prev_cmd = None
    errs, cmds, same_cnt, in_box = [], [], 0, 0
    n_box = 0
    vis_log = []            # (t_capture, e_true)
    frame_idx = 0
    t = 0.0
    next_v = 0.0
    while t < t_end:
        # 内环 100Hz：执行器逼近指令
        if prev_cmd is not None:
            step = (prev_cmd - g) * DT_I / TAU_ACT
            step = max(-V_CAP * DT_I, min(V_CAP * DT_I, step))
            g += step
        # 视觉 30Hz：用 267ms 前的真值 + 噪声
        if t >= next_v:
            e_true = target_pos(t - LAT) - (g if prev_cmd is None else g)
            meas = e_true + random.gauss(0, 2.0)
            if e_ema is None:
                e_ema = meas
            e_prev = e_ema
            e_ema = EMA * meas + (1 - EMA) * e_ema
            v_est = e_ema - e_prev            # px/帧
            eff = e_ema - math.copysign(DZ, e_ema) if abs(e_ema) > DZ else 0.0
            u = KP * eff + KI * (i_state + eff * DT_V) + KD * (eff - d_prev_eff) / DT_V
            i_state += eff * DT_V
            d_prev_eff = eff
            inc = (u * DT_V + v_est * PREDICT) / KPT * KPT  # 增量(px)
            prev_cmd = g + inc
            e_now = target_pos(t) - g
            errs.append(e_now)
            n_box += 1
            in_box += (abs(e_now) <= FIRE_BOX)
            if hasattr(run_old_law, '_last_cmd'):
                if run_old_law._last_cmd == prev_cmd:
                    same_cnt += 1
                cmds.append(abs(prev_cmd - run_old_law._last_cmd))
            run_old_law._last_cmd = prev_cmd
            frame_idx += 1
            next_v += DT_V
        t += DT_I
    del run_old_law._last_cmd
    return _metrics('旧律(30Hz台阶PID)', errs, same_cnt, in_box, n_box, cmds, settle=4.0, times=[i * DT_V for i in range(len(errs))])

def run_smooth_law(t_end=14.0):
    g = 0.0
    x = v = 0.0
    ekf_init = False
    sp = None
    yaw_des = 0.0
    prev_cmd = None
    last_vis_t = None
    errs, cmds, same_cnt, in_box, n_box = [], [], 0, 0, 0
    t = 0.0
    next_v = 0.0
    while t < t_end:
        # 视觉 30Hz：EKF 更新 + 外环锚定
        if t >= next_v:
            e_true = target_pos(t - LAT) - g
            meas = e_true + random.gauss(0, 2.0)
            if last_vis_t is None:
                dt_v = 1 / FPS
            else:
                dt_v = max(1e-3, t - last_vis_t)
            last_vis_t = t
            if not ekf_init:
                x, v, ekf_init = meas, 0.0, True
            else:
                pred = x + v * dt_v
                res = meas - pred
                x = pred + ALPHA * res
                v += BETA * res / dt_v
            yaw_des = g + (x + v * T_LOOK) / KPT
            e_now = target_pos(t) - g
            errs.append(e_now)
            n_box += 1
            in_box += (abs(e_now) <= FIRE_BOX)
            next_v += DT_V
        # 内环 100Hz：斜坡逼近
        if ekf_init:
            if sp is None:
                sp = g
            step = (FF * v / KPT + K_SP * (yaw_des - sp)) * DT_I
            step = max(-SLEW * DT_I, min(SLEW * DT_I, step))
            sp += step
            prev_cmd_val = sp
            if hasattr(run_smooth_law, '_last_cmd'):
                if run_smooth_law._last_cmd == prev_cmd_val:
                    same_cnt += 1
                cmds.append(abs(prev_cmd_val - run_smooth_law._last_cmd))
            run_smooth_law._last_cmd = prev_cmd_val
            step_g = (sp - g) * DT_I / TAU_ACT
            step_g = max(-V_CAP * DT_I, min(V_CAP * DT_I, step_g))
            g += step_g
        t += DT_I
    del run_smooth_law._last_cmd
    return _metrics('平滑律(EKF+100Hz斜坡)', errs, same_cnt, in_box, n_box, cmds, settle=4.0, times=[i * DT_V for i in range(len(errs))])

def _metrics(name, errs, same_cnt, in_box, n_box, cmds, settle, times):
    # 稳态段：4s 之后
    tail = [abs(e) for e, t in zip(errs, times) if t >= settle]
    rms = math.sqrt(sum(e * e for e in tail) / max(len(tail), 1))
    mx = max(tail) if tail else 0
    same_ratio = same_cnt / max(same_cnt + len(cmds), 1)
    mean_cmd = sum(cmds) / max(len(cmds), 1)
    box_ratio = in_box / max(n_box, 1)
    print(f'{name}')
    print(f'  稳态|误差| RMS {rms:6.2f}px  max {mx:6.2f}px   开火框内 {box_ratio*100:5.1f}%')
    print(f'  指令连续性: 相邻相同占比 {same_ratio*100:5.1f}%  平均跳变 {mean_cmd:6.3f}px')
    return rms

print('=== 267ms 延迟 · ±100px/s 轨道靶 · σ=2px ===')
run_old_law()
run_smooth_law()
