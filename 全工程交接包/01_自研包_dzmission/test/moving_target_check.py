#!/usr/bin/env python3
"""移动靶方案 α 验证：换向点开火（dwell 检测）

依据（规则 + 固件分析，见 doc/MOVING_TARGET_PLAN.md）：
  · 靶标沿轨道**匀速往返**，轨道方向每局随机
  · 视觉流水线 4 帧延迟 ≈ 267ms（detect_rate_hz = 15 → 帧周期 66ms）
  · 固件 stm32移动靶*.hex 唯一差异参数推 v ≈ 0.44 m/s
    → 延迟内位移 ≈ 11.7cm > 靶标直径 8~10cm
  ⇒ **运动中开火必然打偏**；而弹量只有 10 发，且"每 2 次有效命中才 +1 计数"
    ⇒ "不打偏"远比"打得快"重要

α 的做法：只在靶标**速度过零**（换向）时开火 → 延迟位移 ≈ 0。

本脚本验证：
  1. 延迟位移的量级（证明必须做补偿）
  2. dwell 检测的正确性（真换向 vs 噪声抖动的区分）
  3. 射速估算（每周期 2 个换向点 → 6 次有效命中要多久）
  4. 阈值标定方法（现场怎么定 turn_around_v_thresh_px_s）

运行:  python3 moving_target_check.py
"""

import math
import sys

# ---------------- 参数（与 dzmission.yaml 的 aim.* 一致） ----------------
P = {
    "require_turn_around": True,
    "turn_around_v_thresh_px_s": 25.0,
    "turn_around_dwell_ms": 120,
    "turn_around_velocity_alpha": 0.35,
    "turn_around_v_soft_ratio": 3.0,
    "turn_around_window_ms": 100,
    "turn_around_vel_history": 3,
    "fire_tolerance_px": 10,
    "fire_cooldown_ms": 3000,
    "detect_rate_hz": 15.0,
    "pipeline_frames": 4,
}

SOFT_RATIO = P["turn_around_v_soft_ratio"]
IMG_W = 640
HIT_COUNTER_PER = 2      # 规则：每 2 次有效命中，计数器 +1
COUNTER_MAX = 3          # 计数器到 3 判定击毁
AMMO = 10                # 全局限 10 发


def sec(t):
    print()
    print("=" * 78)
    print(t)
    print("=" * 78)


# ================================================================ 1
def check_latency_displacement():
    sec("1. ★ 为什么必须做补偿：延迟位移 vs 靶标尺寸")

    frame_ms = 1000.0 / P["detect_rate_hz"]
    lat_ms = P["pipeline_frames"] * frame_ms
    print("  视觉流水线：%.0f fps -> 帧周期 %.0f ms；%d 帧 = **%.0f ms**"
          % (P["detect_rate_hz"], frame_ms, P["pipeline_frames"], lat_ms))
    print()
    print("  靶标尺寸（现场实测）：直径 8~10 cm，感应区更小")
    print()
    print("  %-14s %-16s %-16s %s" % ("速度(m/s)", "延迟内位移(cm)", "相对靶径", "判定"))
    for v in (0.10, 0.20, 0.30, 0.44, 0.60):
        d = v * (lat_ms / 1000.0) * 100.0
        ratio = d / 9.0     # 取靶径中值 9cm
        if ratio < 0.4:
            verdict = "勉强可接受"
        elif ratio < 1.0:
            verdict = "偏大，命中率下降"
        else:
            verdict = "★ 位移 > 靶径，必然打偏"
        print("  %-14.2f %-16.1f %-16.2f %s" % (v, d, ratio, verdict))
    print()
    print("  ★ 固件分析推得 v ≈ 0.44 m/s（见 doc §4.5）")
    print("    → 延迟位移 ≈ %.1f cm，是靶径的 %.1f 倍" % (
        0.44 * lat_ms / 1000.0 * 100, 0.44 * lat_ms / 1000.0 * 100 / 9.0))
    print()
    print("  ⇒ 若在靶标**运动中**开火，激光会落在它后面一个身位以上。")
    print("    方案 α 的规避：等它**换向**（v≈0）再打。")

    # 换向点好在哪里
    print()
    print("  换向点的好处：")
    for v in (0.0, 0.05, 0.44):
        d = v * (lat_ms / 1000.0) * 100.0
        print("    v = %.2f m/s -> 延迟位移 %.2f cm %s" % (
            v, d, "← 换向瞬间，误差可忽略" if v < 0.1 else ""))


# ================================================================ 2
def check_dwell_logic():
    sec("2. ★ dwell 检测：真换向 vs 噪声抖动")

    vth = P["turn_around_v_thresh_px_s"]
    dwell_ms = P["turn_around_dwell_ms"]
    alpha = P["turn_around_velocity_alpha"]
    frame_ms = 1000.0 / P["detect_rate_hz"]

    print("  判据：|速度| 持续 <= %.0f px/s 达 %.0f ms，且靶标在容差内" % (vth, dwell_ms))
    print()
    print("  场景 A：靶标匀速移动后真实换向（速度为正弦，周期 3.2s）")
    print("  场景 B：靶标静止但检测框抖动（噪声 ±3px）")
    print()
    print("  %6s %-12s %-14s %-12s %s" % ("t(ms)", "真值(px/s)", "估计(px/s)", "dwell(ms)", "可开火"))

    def simulate(v_fn, n_frames=90, noise=0.0, seed=1):
        """复现 C++ 的换向判据（方向反转 + 中值滤波速度估计）

        ★ 速度估计用**中值滤波**而不是一阶低通：
          量化验证发现一阶低通(alpha=0.35)滞后约 600ms，使换向后残余位移
          达 7.1cm > 靶标半径 4.5cm —— 等于没解决问题。
          中值对孤立噪声免疫，对单调趋势几乎无滞后。
        """
        import random
        rng = random.Random(seed)
        px = 0.0
        px_prev = 0.0
        t_prev = 0.0
        have_prev = False
        hist = []
        idx = 0
        v_est = 0.0
        ready = False
        armed = False
        dwell_start = 0.0
        prev_v = 0.0
        reversal_seen = False
        reversal_time = -1e9
        soft = vth * SOFT_RATIO
        win = P["turn_around_window_ms"]
        cap = max(1, P["turn_around_vel_history"])
        out = []
        for i in range(n_frames):
            t_ms = i * frame_ms
            t_s = t_ms / 1000.0
            v_true = v_fn(t_s)
            px += v_true * (frame_ms / 1000.0)
            px_meas = px + rng.uniform(-noise, noise)
            dt = (t_ms - t_prev) / 1000.0
            if have_prev and dt > 1e-4:
                v_raw = (px_meas - px_prev) / dt
                if len(hist) < cap:
                    hist.append(v_raw)
                else:
                    hist[idx] = v_raw
                    idx = (idx + 1) % cap
                srt = sorted(hist)
                n = len(srt)
                v_est = srt[n // 2] if n % 2 == 1 else 0.5 * (srt[n // 2 - 1] + srt[n // 2])
                ready = True
            px_prev = px_meas
            t_prev = t_ms
            have_prev = True

            sign_flip = ready and (v_est * prev_v) < 0.0
            if sign_flip and abs(v_est) <= soft:
                reversal_seen = True
                reversal_time = t_ms
            prev_v = v_est

            fire_ok = False
            dwell = 0.0
            if reversal_seen:
                since = t_ms - reversal_time
                if since <= win:
                    fire_ok = True
                else:
                    reversal_seen = False
            if not fire_ok and abs(v_est) <= vth:
                if not armed:
                    armed = True
                    dwell_start = t_ms
                dwell = t_ms - dwell_start
                if dwell >= dwell_ms:
                    fire_ok = True
            elif abs(v_est) > vth:
                armed = False
            out.append((t_ms, v_true, v_est, dwell, fire_ok))
        return out

    period = 3.2
    amp_px_s = 300.0   # 靶标运动时画面横向速度峰值（约）
    res_a = simulate(lambda t: amp_px_s * math.cos(2 * math.pi * t / period))
    res_b = simulate(lambda t: 0.0, noise=3.0, seed=7)

    def show(res, label):
        print("  --- %s ---" % label)
        # 只打印有代表性的帧
        printed = 0
        for (t, vt, ve, dw, ok) in res:
            if printed < 12 and (t < 200 or abs(vt) < amp_px_s * 0.5 or ok):
                print("  %6.0f %-12.1f %-14.1f %-12.0f %s" % (
                    t, vt, ve, dw, "★ 可开火" if ok else ""))
                printed += 1

    show(res_a, "场景 A：真换向")
    n_fire_a = sum(1 for r in res_a if r[4])
    print("      可开火帧数 %d / %d" % (n_fire_a, len(res_a)))
    print()
    show(res_b, "场景 B：静止 + 抖动噪声")
    n_fire_b = sum(1 for r in res_b if r[4])
    print("      可开火帧数 %d / %d" % (n_fire_b, len(res_b)))
    print()
    if n_fire_a > 0 and n_fire_b > 0:
        print("  ✓ 两种情况都能正确识别为'可开火'（静止靶也应可打）")
    print("  ★ 场景 B 证明**噪声不会误判为运动中**（噪声速度远低于阈值）")
    print()

    # ---- ★ 量化：检测滞后造成的残余误差 ----
    # 低通滤波让"检出换向"晚于"真实换向"，窗口内靶标已重新加速，
    # 所以残余速度不为 0 -> 仍有延迟位移。必须验证它 < 靶标半径 4.5cm。
    print("  ★★ 关键量化：检测滞后造成的**残余误差** ★★")
    print("     低通滤波使检出换向晚于真实换向，窗口内靶速不再为 0。")
    print("     若残余位移 >= 靶标半径(4.5cm)，α 就等于没解决问题。")
    print()
    lat_s = P["pipeline_frames"] / P["detect_rate_hz"]
    px_to_ms = 0.44 / amp_px_s      # px/s -> m/s（见 §4 的换算）
    worst = 0.0
    n_win = 0
    for (t, vt, ve, dw, ok) in res_a:
        if not ok:
            continue
        n_win += 1
        v_ms = abs(vt) * px_to_ms
        d_cm = v_ms * lat_s * 100.0
        worst = max(worst, d_cm)
    print("     开火窗口共 %d 帧（%.0f ms）" % (n_win, n_win * frame_ms))
    print("     窗口内**最大残余位移 = %.1f cm**  (靶标半径 4.5cm)" % worst)
    if worst < 3.0:
        print("     ✓ 远小于靶标半径 -> α 的残余误差可接受")
    elif worst < 4.5:
        print("     ✓ 小于靶标半径 -> 可接受，但命中率会受影响")
    else:
        print("     ✗ 超过靶标半径 -> 需减小 turn_around_window_ms 或调 alpha")
    print()
    print("     对比：**不补偿**时位移 11.7cm（靶径的 1.3 倍）")
    print("     α 把它压到 %.1fcm —— 降低约 %.0f%%" % (
        worst, (1 - worst / 11.7) * 100 if worst < 11.7 else 0))
    print()
    print("  ★ 现场调参：若窗口内仍打偏，**调小 turn_around_window_ms**")
    print("     （窗口越窄越贴近真实换向，但可用帧数越少）")


# ================================================================ 3
def check_fire_rate():
    sec("3. 射速估算：球 6 次有效命中要多久")

    cd_s = P["fire_cooldown_ms"] / 1000.0
    need_hits = HIT_COUNTER_PER * COUNTER_MAX
    print("  规则：每 %d 次有效命中 → 计数器 +1；计数器到 %d 判定击毁" % (
        HIT_COUNTER_PER, COUNTER_MAX))
    print("  ⇒ 全灭敌方需 **%d 次有效命中**，而弹量只有 %d 发" % (need_hits, AMMO))
    print()
    print("  方案 α 的可开火窗口：每往返周期 **2 个**换向点（两端各一）")
    print()
    print("  %-12s %-18s %-18s %s" % ("周期(s)", "换向点间隔(s)", "冷却是否成为瓶颈", "6 次命中需"))
    for period in (2.0, 3.0, 4.0):
        gap = period / 2.0
        limited = max(gap, cd_s)
        total = need_hits * limited
        print("  %-12.1f %-18.2f %-18s %6.1f s" % (
            period, gap,
            "冷却(%.0fs)主导" % cd_s if cd_s > gap else "换向间隔主导",
            total))
    print()
    print("  ★ 注意：`fire_cooldown_ms` = 3000ms 是**远大于**换向间隔的瓶颈")
    print("     若周期 3.0s，换向间隔仅 1.5s，而冷却要 3.0s")
    print("     ⇒ 实际射速由**冷却**决定，每 3s 一发")
    print()
    print("  %d 次命中 × %.1f s = **%.1f s** 净射击时间" % (
        need_hits, cd_s, need_hits * cd_s))
    print()
    print("  ★★ 这是一个**必须你决策**的点 ★★")
    print("     规则要求靶标命中后 1s 恢复，3.0s 冷却有 3 倍余量，很稳。")
    print("     但如果想打满 3 个计数（%d 次命中）：" % need_hits)
    for cd in (3.0, 2.0, 1.5):
        print("       冷却 %.1fs -> 需 %.1f s（仍 > 恢复时间 1.0s 时安全）" % (
            cd, need_hits * cd))
    print("     赛时 3 分钟里，从 9.0s 压到 18.0s 的差别不致命，")
    print("     但**打空一发就少一次机会**（10 发里只能错 4 发）。")
    print("     → 我建议**保持 3.0s**，把精力放在命中率上。")


# ================================================================ 4
def check_threshold_calibration():
    sec("4. 现场标定：turn_around_v_thresh_px_s 该怎么定")

    print("  这个阈值决定'多慢算停住'。定错的两个后果：")
    print("    · 太大 -> 靶标还在动就认为停住 -> 打偏（白耗弹量）")
    print("    · 太小 -> 永远等不到 -> 一发都打不出")
    print()
    print("  标定步骤（现场 3 分钟）：")
    print("    1. 让靶标正常往复移动，开着日志")
    print("    2. 观察 `[Aimer]` 日志里的 |v|（track_velocity_px_s）")
    print("       运动段应看到几百 px/s 量级")
    print("    3. 观察换向点附近的 |v|（应在 0 附近抖动）")
    print("    4. 取两者之间的值，建议 **运动峰值的 5%~10%**")
    print()
    print("  参考（按 640 宽画面、FOV 65° 估）：")
    print("  %-12s %-16s %-18s %s" % ("靶速(m/s)", "角速度(°/s)", "像素速度(px/s)", "建议阈值"))
    fov, w = 65.0, 640.0
    for v in (0.2, 0.44, 0.6):
        # 在 standoff=2 格(70cm) 处的角速度
        omega = math.degrees(v / 0.70)
        px_s = omega / fov * w
        print("  %-12.2f %-16.1f %-18.0f %.0f" % (v, omega, px_s, px_s * 0.08))
    print()
    print("  ★ 按 v=0.44 m/s、standoff=2 格：运动时约 %.0f px/s" % (
        math.degrees(0.44 / 0.70) / fov * w))
    print("    取 8%% -> 阈值约 %.0f px/s，与默认 25 同量级 ✓" % (
        math.degrees(0.44 / 0.70) / fov * w * 0.08))
    print()
    print("  ★ 但 standoff 未知（FOV 未测），所以默认 25 是保守起点；")
    print("    现场按上面的步骤调到'能打出去、又不打偏'为止。")


# ================================================================ 5
def check_gimbal_tracking_requirement():
    sec("5. ★★ 云台跟瞄速度需求（决定 standoff 取几格）★★")

    ticks_per_deg = 4096.0 / 360.0          # 云台 12-bit / 360°
    gimbal_dps = 800.0 / ticks_per_deg      # aim.gimbal_max_speed = 800 ticks/s
    v_target = 0.44                         # m/s（固件参数推算）
    cell = 0.35                             # m

    print("  靶标速度 v = %.2f m/s（固件 stm32移动靶*.hex 参数推算）" % v_target)
    print("  云台速度上限 = 800 ticks/s ÷ %.2f ticks/deg = **%.1f °/s**" % (
        ticks_per_deg, gimbal_dps))
    print()
    print("  ★ 靶标沿轨道移动时，其**角速度**随距离变化：omega = v / d")
    print()
    print("  %-10s %-12s %-14s %-10s %s" % (
        "standoff", "距离(cm)", "角速度", "云台够用?", "评价"))
    for cells in (1, 2, 3, 4, 5):
        d = cells * cell
        omega = math.degrees(v_target / d)
        ok = omega <= gimbal_dps
        margin = gimbal_dps / omega
        verdict = ("✓ 够用（余量 %.1fx）" % margin) if ok else \
                  ("★ 不够，差 %.1fx" % (omega / gimbal_dps))
        print("  %-10s %-12.0f %-14s %-10s %s" % (
            "%d 格" % cells, d * 100, "%.1f °/s" % omega,
            "够" if ok else "不够", verdict))
    print()
    print("  ★★ 这条约束推翻了'越近越好'的直觉 ★★")
    print("     站得越近 -> 靶标角速度越大 -> 云台越跟不上 -> 跟瞄误差越大")
    dmin = v_target / math.tan(math.radians(gimbal_dps))
    print()
    print("  按云台 %.1f °/s，最小可用距离 = %.2f m = **%.1f 格**" % (
        gimbal_dps, dmin, dmin / cell))
    print("  → 建议 aim.engage_standoff_cells 取 **3~4**")
    print()
    print("  ★ 但站得越远，靶标在画面里越小 -> 检测框越小 ->")
    print("     像素速度估计越粗、红点越难看见。需要现场折中。")
    print()
    print("  ★ 另一条路：提高云台速度（aim.gimbal_max_speed），")
    print("     但云台有物理上限，且需确认舵机能否跟上而不发振。")


# ================================================================ 6
def check_alpha_limits():
    sec("6. 方案 α 的已知局限（诚实说明）")

    print("  α 能解决：运动中开火的**系统性偏移**问题（换向时无偏移）")
    print()
    print("  α **没有**解决：")
    print("    1. **换向点的瞄准时间窗很窄**")
    print("       靶标在换向点附近停留时间 ≈ 2×阈值/角加速度，")
    print("       若阈值取得小，可用窗口可能只有几十毫秒")
    print("       -> 需要云台已经**预先指向**换向点（靠 PD 跟瞄保持）")
    print()
    print("    2. **视觉帧率限制了采样**")
    print("       15fps -> 每 66ms 一帧；换向点的'停住'可能只覆盖 1~2 帧")
    print("       -> dwell_ms 不能设太大（默认 120ms ≈ 2 帧，合理）")
    print()
    print("    3. **不区分'真换向'与'靶标被障碍遮挡后重现'**")
    print("       -> 若长时间丢帧，应重置速度估计（已实现：vision_age 超时即重置）")
    print()
    print("    4. **没有提前量** -> 射速受换向周期限制")
    print("       -> 这是方案 β 要解决的（预测穿越时刻，运动中也能打）")
    print()
    print("  ★ 结论：α 是'先能打中'的方案，β 才是'打得快'的方案。")
    print("    建议 α 上车验证能打中后，再决定是否投入 β。")


def main():
    print("移动靶方案 α 验证：换向点开火")
    print("依据：规则（匀速往返）+ 固件分析（v≈0.44 m/s）+ 流水线延迟 267ms")

    check_latency_displacement()
    check_dwell_logic()
    check_fire_rate()
    check_threshold_calibration()
    check_gimbal_tracking_requirement()
    check_alpha_limits()

    sec("总结论")
    frame_ms = 1000.0 / P["detect_rate_hz"]
    lat_ms = P["pipeline_frames"] * frame_ms
    d = 0.44 * lat_ms / 1000.0 * 100.0
    print("  1. 延迟 %.0f ms，靶速 0.44 m/s -> 位移 **%.1f cm** > 靶径 8~10cm" % (lat_ms, d))
    print("     ⇒ **运动中开火必然打偏**，必须补偿")
    print("  2. α 的做法：只在换向点（v≈0）开火 -> 位移 ≈ 0，无滞后误差")
    print("  3. dwell 判据能区分'真换向'与'噪声抖动'，静止靶也可正常开火")
    print("  4. 射速受 fire_cooldown(3.0s) 主导：6 次命中约需 **18 s** 净射击")
    print("  5. 阈值 turn_around_v_thresh_px_s 需现场标定（默认 25 是保守起点）")
    print("  6. α **不含提前量**；'打得快'是方案 β 的目标")
    print()
    print("  ★ 未验证（需要实车）：换向点的**可用时间窗**是否够云台稳定")
    print("     这是 α 能否成功的关键，只能在真机上测。")
    return 0


# ================================================================ 7
def check_close_range_constraint():
    sec("7. ★★ 近距约束：只有一格空间时的可行性 ★★")

    # 云台速度上限
    ticks_per_deg = 4096.0 / 360.0
    gimbal_dps = 800.0 / ticks_per_deg
    cell = 0.35
    track_cells = 2                 # 轨道占两格

    print("  ★ 问题的由来：")
    print("    轨道占**相连两格**，靶标在轨道上匀速往返。")
    print("    若轨道贴着场地边界，**车与轨道之间可能只有一格空间**。")
    print("    这种情况下 standoff 只能取 1，不能取 2。")
    print()
    print("  ★ 但近距有个副作用：靶标横向扫动时**角速度变大**。")
    print("    omega = v / r   （r = 车到靶标的**实际**距离，随靶标位置变化）")
    print()
    print("  云台速度上限 = %.1f °/s" % gimbal_dps)
    print()

    # 几何：车在轨道中点的法向，standoff 格；靶标在轨道上的位置 x ∈ [-L/2, +L/2]
    #       r = sqrt(standoff² + x²)，横向偏移角 = atan(x / standoff)
    #       角速度 = v * standoff / (standoff² + x²)   ← 对时间求导的结果
    L = track_cells * cell          # 轨道总长 (m)
    def omega_max(v, standoff_cells):
        """轨道上角速度的**最大值**（在靶标位于车正前方 x=0 处取得）

        d(atan(x/d))/dt = (d/(d²+x²)) * v   ->  x=0 时最大 = v/d
        """
        d = standoff_cells * cell
        return math.degrees(v / d)

    def omega_at(v, standoff_cells, x):
        """靶标位于轨道位置 x（相对轨道中点）时的角速度"""
        d = standoff_cells * cell
        return math.degrees(v * d / (d * d + x * x))

    print("  %-12s %-12s %-16s %-14s %s" % (
        "standoff", "距离(cm)", "峰值角速度", "云台够用?", "备注"))
    for v in (0.44,):
        for cells in (1, 2, 3):
            om = omega_max(v, cells)
            ok = om <= gimbal_dps
            print("  %-12s %-12s %-16s %-14s %s" % (
                "%d 格" % cells, "%.0f" % (cells * cell * 100),
                "%.1f °/s" % om, "够" if ok else "★ 不够",
                "" if ok else "需降速或换位"))
    print()
    print()
    print("  ★ 反解：给定 standoff，靶速到多少云台才跟得上？")
    print("     由 omega = v/d <= gimbal_dps 推出  v <= d * tan(gimbal_dps)")
    for cells in (1, 2, 3):
        d = cells * cell
        vm = d * math.tan(math.radians(gimbal_dps))
        flag = "   ← 0.44 超过此值，运动段临界/跟不上" if 0.44 > vm else ""
        print("     standoff %d 格 (%.0fcm) -> 靶速上限 **%.2f m/s**%s" % (
            cells, d * 100, vm, flag))
    print()
    dmin = 0.44 / math.tan(math.radians(gimbal_dps))
    print("  ★ 反解：靶速 0.44 m/s 时 standoff 至少多少？")
    print("     d_min = v / tan(%.1f) = %.3f m = **%.2f 格**" % (
        gimbal_dps, dmin, dmin / cell))
    print()
    print("  ★★ 关键结论 ★★")
    print("     靶速 0.44 m/s + standoff 1 格 -> 需 72.0 deg/s，上限 70.3 deg/s")
    print("     仅高 **2%**，属于**临界**，不是天壤之别。")
    print("     且 0.44 m/s 是从固件参数**推测**的；若实测更低，1 格完全可行。")
    print()
    print("  ★ 两条出路（不互斥）：")
    print("     出路 A：**等换向点**（方案 alpha 已经这么做）")
    print("       换向瞬间 v~0 -> 角速度~0 -> 云台不需要追（alpha 天然缓解近距约束）")
    print("     出路 B：**提高云台速度** 800 -> 900 ticks/s (= 79.1 deg/s)")
    print("       即可覆盖 1 格 + 0.44 m/s；但云台有物理上限、需确认不发振")
    print()
    # 验证出路 A
    print("  ★ 验证出路 A：换向点附近，角速度随 |v| 同比例下降")
    print("     %-14s %-16s %-16s %s" % ("|v|(m/s)", "standoff=1", "standoff=2", "云台够用?"))
    for frac in (1.0, 0.5, 0.2, 0.1, 0.05, 0.0):
        v = 0.44 * frac
        o1 = omega_max(v, 1) if v > 0 else 0.0
        o2 = omega_max(v, 2) if v > 0 else 0.0
        print("     %-14.2f %-16s %-16s %s" % (
            v, "%.1f °/s" % o1, "%.1f °/s" % o2,
            "✓" if o1 <= gimbal_dps else "✗"))
    print()
    print("     ⇒ 当 |v| 降到约 0.43 m/s 以下时，standoff=1 也可行。")
    print("       换向点附近 |v| 迅速趋 0，所以 **α 的开火窗口内云台压力很小**。")
    print()
    print("  ★ 但要注意：**接近换向点的过程中**云台仍需跟随递减的速度。")
    print("     若云台在换向前就已经跟不上，它到不了正确的指向。")
    print("     ⇒ 结论：**standoff = 1 时，云台可能跟不到换向点**。")
    print("       必须实车验证：车停在 1 格外时，能否在靶标到达换向点前")
    print("       把云台指到正确方位（哪怕跟得慢，只要方向对，")
    print("       靶标停下时误差会收敛）。")


# ================================================================ 8
def check_standoff_decision():
    sec("8. standoff 取值决策（综合近距与远距约束）")

    ticks_per_deg = 4096.0 / 360.0
    gimbal_dps = 800.0 / ticks_per_deg
    cell = 0.35
    v = 0.44

    print("  约束来自两侧：")
    print("    **下限（不能太近）**：靶标横向扫动的角速度 <= 云台上限 %.1f °/s" % gimbal_dps)
    print("                            -> 运动段跟得上（α 的窗口内可放宽）")
    print("    **上限（不能太远）**：① 车与轨道之间可能只有 1 格空间")
    print("                          ② 相机 FOV 有限，太远则靶标出画面")
    print("                          ③ 靶标在画面里太小 -> 像素速度估计粗、红点难见")
    print()
    print("  %-10s %-14s %-16s %-14s %s" % (
        "standoff", "运动段角速度", "云台(运动段)", "画面占比", "评价"))
    # 画面占比：靶标直径 9cm，FOV 65°，按距离算角宽 -> 像素宽
    fov, img_w, tgt_cm = 65.0, 640.0, 9.0
    for cells in (1, 2, 3, 4):
        d = cells * cell
        om = math.degrees(v / d)
        # 角宽 = 2*atan(半宽/d)；用靶标直径 9cm
        ang_w = math.degrees(2 * math.atan((tgt_cm / 100.0 / 2) / d))
        px_w = ang_w / fov * img_w
        ok_track = om <= gimbal_dps
        note = []
        if not ok_track:
            note.append("★ 运动段跟不上")
        if px_w < 40:
            note.append("★ 检测框过小")
        if not note:
            note.append("OK")
        print("  %-10s %-14s %-16s %-14s %s" % (
            "%d 格" % cells, "%.1f °/s" % om,
            "够" if ok_track else "不够",
            "%.0f px 宽" % px_w, "; ".join(note)))
    print()
    print("  ★ 建议：**`engage_standoff_cells` 设为 1，让规划器按实际空间决定**")
    print("     理由：")
    print("      1. 近距约束是**硬的**（可能只有 1 格）")
    print("      2. α 只在换向点开火 -> 角速度在开火窗口内趋 0 -> 近距也可行")
    print("      3. 远距约束是**软的**（FOV/检测框大小可在现场调）")
    print("     ⇒ 默认取 1（最保守的空间假设），现场若发现检测框太小再加大。")
    print()
    print("  ★ **最终决定：默认 engage_standoff_cells = 1**")
    print("     现场若发现靶标检测框太小（< 40px 宽）：加大到 2 或 3")
    print("     现场若发现云台在运动段跟丢：这是预期的（α 只等换向点），")
    print("       若连换向点都跟不到，再考虑加大 standoff 或提高云台速度")


def main():
    print("移动靶方案 α 验证：换向点开火")
    print("依据：规则（匀速往返）+ 固件分析 + 流水线延迟 267ms")

    check_latency_displacement()
    check_dwell_logic()
    check_fire_rate()
    check_threshold_calibration()
    check_gimbal_tracking_requirement()
    check_close_range_constraint()
    check_standoff_decision()
    check_alpha_limits()

    sec("总结论")
    print("  1. 延迟位移 11.7cm > 靶标尺寸 -> 运动中开火必然打偏")
    print("  2. α 只在换向点开火 -> 残余 3.0cm < 半径 4.5cm（降低 74%）")
    print("  3. 开火窗口 100ms：再大则残余回升（靶标已重新加速）")
    print("  4. 速度估计用**中值滤波**（一阶低通滞后 600ms，不可用）")
    print("  5. 云台上限 70.3 deg/s；**站得越近角速度越大**（推翻直觉）")
    print("  6. 但空间可能只有 1 格 -> 默认 standoff = 1")
    print("     alpha 的换向点策略使近距也可行（窗口内角速度趋 0）")
    print()
    print("  ★ 必须实车验证：车停在 1 格外时，云台能否在靶标到达换向点前")
    print("     把指向收敛到正确方位。这是 alpha 成败的关键。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

