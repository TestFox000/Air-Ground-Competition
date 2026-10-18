#!/usr/bin/env python3
"""TargetAimer 瞄准律验证 —— 借鉴学员工程后的三个新机制

来源：《发学员/src(走斜线)》与《src(走直角)》的 dzrknn_dataprocess 差异分析。
两版都用、且我的实现原先没有的三个机制：

  ① 丢帧宽限（按时间）：目标刚消失时"保持不动"等视觉，而不是立刻重扫。
     学员原话："Grace period: if the target was seen recently, HOLD position
     and wait for the (slow) image pipeline instead of resuming the sweep.
     This is why the gimbal used to scan forever past a nearby target:
     the sweep outran the detection latency, so the target was never
     reported while the gimbal was actually pointing at it."

  ② 开火保持窗口 vs 对准容差（分层）：
     对准容差决定"何时进入开火态"，保持窗口决定"开火态下漂多远仍继续开火"。
     学员取值：对准 5px / 保持 15px。防止边缘抖动导致开火↔停火反复切换。

  ③ 开火间隔必须匹配靶标恢复时间（规则：命中后 1s 恢复）。

本脚本验证这三条在**时序上**是否正确，特别是边界条件。

运行:  python3 aimer_check.py
"""

import sys

# ---------------- 参数（与 dzmission.yaml 的 aim.* 一致） ----------------
P = {
    "scan_mode": "step",
    "scan_dwell_ms": 300,
    "scan_hold_ms": 200,
    "scan_sweep_rate_dps": 12.0,
    "scan_yaw_min_deg": -90.0,
    "scan_yaw_max_deg": 90.0,
    "lost_grace_ms": 600,          # ★ 时间宽限
    "lost_frames_to_rescan": 20,   # 帧计数兜底
    "fire_tolerance_px": 10,       # 进入开火态
    "fire_hold_px": 15,            # 开火态下继续开火
    "fire_debounce_frames": 3,
    "fire_cooldown_ms": 3000,      # ★ 匹配靶标 1s 恢复
    "vision_timeout_ms": 500,
    "detect_rate_hz": 15.0,        # 检测帧率（优化后默认 15）
}

TARGET_RECOVERY_S = 1.0    # 规则：命中后靶标自动恢复时间
TOTAL_AMMO = 10            # 规则：全局限 10 发


def sec(t):
    print()
    print("=" * 76)
    print(t)
    print("=" * 76)


# ================================================================ 1
def check_grace_period():
    sec("1. ★ 丢帧宽限期（按时间）—— 防止'扫过目标却报没看到'")

    grace_s = P["lost_grace_ms"] / 1000.0
    frame_period_ms = 1000.0 / P["detect_rate_hz"]
    pipeline_frames = 4           # det_node 要攒满 threadNum_ 帧才出结果
    pipeline_ms = pipeline_frames * frame_period_ms

    print("  视觉流水线延迟推导：")
    print("    检测帧率        %.0f fps -> 帧周期 %.0f ms" % (
        P["detect_rate_hz"], frame_period_ms))
    print("    流水线深度      %d 帧（det_node 攒满才 get）" % pipeline_frames)
    print("    => 端到端延迟   %.0f ms" % pipeline_ms)
    print("    宽限期配置      %d ms" % P["lost_grace_ms"])
    print("    余量倍数        %.2f x" % (P["lost_grace_ms"] / pipeline_ms))
    print()
    if P["lost_grace_ms"] >= pipeline_ms:
        print("  ✓ 宽限期 >= 流水线延迟，目标短时丢帧时云台保持不动，")
        print("    视觉结果回来后能立即接上，不会扫过头。")
    else:
        print("  ✗ 宽限期 < 流水线延迟！云台会在检测帧回来之前就重新扫描，")
        print("    这正是学员注释描述的失败模式。需增大 lost_grace_ms。")
    print()
    print("  时序仿真（目标在第 3 帧后消失，第 6 帧重新出现）：")
    print()
    print("  %6s %-12s %-14s %s" % ("t(ms)", "视觉", "状态", "说明"))
    t = 0.0
    last_seen = 0.0
    ever_seen = False
    state = "SCANNING"
    for i in range(24):
        t = i * frame_period_ms
        visible = (i < 3) or (i >= 6)
        if visible:
            last_seen = t
            ever_seen = True
            if state in ("SCANNING", "IDLE", "COOLDOWN"):
                state = "TRACKING"
        else:
            since = (t - last_seen) / 1000.0 if ever_seen else 1e9
            if since < grace_s:
                state = "TRACKING"      # 宽限期内保持
            else:
                state = "SCANNING"      # 超时才重扫
        if i < 9 or i % 4 == 0:
            print("  %6.0f %-12s %-14s %s" % (
                t, "有目标" if visible else "无目标", state,
                "宽限保持" if (not visible and state == "TRACKING") else ""))
    print()
    print("  ★ 关键：第 3~5 帧（无目标）期间状态仍是 TRACKING（保持不动），")
    print("     所以第 6 帧目标回来时能直接接上，**不会重新扫**。")


# ================================================================ 2
def check_hold_window():
    sec("2. ★ 开火保持窗口 vs 对准容差（分层容差防抖）")

    tol = P["fire_tolerance_px"]
    hold = P["fire_hold_px"]
    print("  对准容差 fire_tolerance_px = %d px  -> 进入开火态" % tol)
    print("  保持窗口 fire_hold_px      = %d px  -> 开火态下继续开火" % hold)
    print()
    print("  %-14s %-12s %-16s %s" % ("|cx|(px)", "是否在对准内", "是否在保持内", "行为"))

    cases = [0, 5, tol, tol + 1, 12, hold, hold + 1, 25]
    for cx in cases:
        in_tol = cx <= tol
        in_hold = cx <= hold
        if in_tol:
            act = "对准 -> 可开火"
        elif in_hold:
            act = "仅在已开火时维持开火"
        else:
            act = "★ 超出保持窗口 -> 暂停开火并微调"
        print("  %-14d %-12s %-16s %s" % (
            cx, "是" if in_tol else "否", "是" if in_hold else "否", act))
    print()
    print("  单阈值 vs 分层阈值的对比（目标在阈值附近抖动）：")
    print()
    print("  %6s %-12s %-22s %s" % ("t(ms)", "|cx|", "单阈值(10px)", "分层(10/15px)"))
    jitter = [3, 8, 12, 14, 11, 9, 13, 7, 12, 14]
    single_fires = 0
    layered_fires = 0
    single_state = "TRACKING"
    layered_state = "TRACKING"
    for i, cx in enumerate(jitter):
        t = i * 66.0
        # 单阈值：只要超出 10px 就停火
        if cx <= tol:
            single_state = "FIRING"
            single_fires += 1
        else:
            single_state = "PAUSE"
        # 分层：已开火时看 15px
        if cx <= tol or (layered_state == "FIRING" and cx <= hold):
            layered_state = "FIRING"
            layered_fires += 1
        else:
            layered_state = "PAUSE"
        print("  %6.0f %-12d %-22s %s" % (t, cx, single_state, layered_state))
    print()
    print("  开火窗口占比：单阈值 %d/%d，分层 %d/%d" % (
        single_fires, len(jitter), layered_fires, len(jitter)))
    if layered_fires > single_fires:
        print("  ✓ 分层阈值减少了'开火↔停火'抖动，弹量利用率更高")
        print("    （全局限 %d 发，频繁切换会浪费）" % TOTAL_AMMO)
    else:
        print("  两者相当（该抖动序列未触发差异）")


# ================================================================ 3
def check_fire_interval():
    sec("3. ★ 开火间隔必须匹配靶标恢复时间")

    cd = P["fire_cooldown_ms"] / 1000.0
    print("  规则：靶标被命中后 **%.1f s 自动恢复**；任一侧累计 3 次则整体击毁。" % TARGET_RECOVERY_S)
    print("  当前 fire_cooldown_ms = %d ms (%.1f s)" % (P["fire_cooldown_ms"], cd))
    print()
    print("  %-28s %-14s %s" % ("配置", "间隔(s)", "判定"))
    for name, ms in (("原配置（我最初的错误）", 500),
                     ("学员两版", 3000),
                     ("当前配置", P["fire_cooldown_ms"]),
                     ("过于保守", 6000)):
        s = ms / 1000.0
        if s < TARGET_RECOVERY_S:
            verdict = "✗ 快于恢复时间，部分命中无效（浪费弹量）"
        elif s > 5000:
            verdict = "△ 过于保守，3 次命中要 %.1f s" % (s * 3)
        else:
            verdict = "✓ 合理"
        print("  %-28s %-14.1f %s" % (name, s, verdict))
    print()
    print("  击毁一个目标需要 3 次有效命中：")
    for s in (0.5, 3.0, 6.0):
        print("    间隔 %.1f s -> 需 %.1f s（不含瞄准时间）" % (s, s * 3))
    print()
    if cd >= TARGET_RECOVERY_S:
        print("  ✓ 当前 %.1f s >= 恢复时间 %.1f s，每次命中都有效" % (cd, TARGET_RECOVERY_S))
    else:
        print("  ✗ 当前 %.1f s < 恢复时间 %.1f s，会打出无效命中！" % (cd, TARGET_RECOVERY_S))
    print()
    print("  ★ 补充：学员用 3.0 s 还有另一个好处 —— 可以观察'受击计数'是否")
    print("     真的 +1（裁判板反馈），据此判断是否命中，而不是盲目连发。")


# ================================================================ 4
def check_scan_modes():
    sec("4. 两种扫描模式的覆盖能力对比")

    span = P["scan_yaw_max_deg"] - P["scan_yaw_min_deg"]
    frame_period_s = 1.0 / P["detect_rate_hz"]

    # step 模式
    dirs = 8
    dwell_s = P["scan_dwell_ms"] / 1000.0
    step_total = dirs * dwell_s
    step_per_dwell = span / (dirs - 1)
    frames_per_dwell = dwell_s / frame_period_s

    # sweep 模式
    rate = P["scan_sweep_rate_dps"]
    sweep_sweep_s = span / rate
    hold_s = P["scan_hold_ms"] / 1000.0
    levels = 3
    sweep_total = (sweep_sweep_s + hold_s * 2) * levels
    deg_per_frame = rate * frame_period_s
    frames_per_target = 0.0
    # 目标在视野内停留时间 = FOV / rate；假设水平 FOV 65°
    fov = 65.0
    frames_per_target = (fov / rate) / frame_period_s

    print("  扫程 ±%.0f° = %.0f°；检测 %.0f fps（帧周期 %.0f ms）" % (
        abs(P["scan_yaw_min_deg"]), span, P["detect_rate_hz"],
        frame_period_s * 1000))
    print()
    print("  %-14s %-16s %-16s %s" % ("模式", "扫完一遍", "每档/每帧转过", "目标可见帧数"))
    print("  %-14s %-16s %-16s %.1f" % (
        "step", "%.1f s" % step_total, "%.1f°/档" % step_per_dwell,
        frames_per_dwell))
    print("  %-14s %-16s %-16s %.1f" % (
        "sweep", "%.1f s" % sweep_total, "%.1f°/帧" % deg_per_frame,
        frames_per_target))
    print()
    print("  ★ step 模式：每档停 %.0f ms，期间有 %.0f 帧检测。" % (
        P["scan_dwell_ms"], frames_per_dwell))
    print("     优点：扫得快（%.1f s 一遍），适合已知大致方位。" % step_total)
    print("     风险：**档与档之间是跳变的**，若目标在档位之间，两档都")
    print("           看不到它。且 300ms 停留对 4 帧流水线（%.0f ms）" % (
        4 * frame_period_s * 1000))
    print("           偏紧 —— 刚好够，没有余量。")
    print()
    print("  ★ sweep 模式：连续扫，目标只要在 FOV 内就会被看到 %.1f 帧。" % frames_per_target)
    print("     优点：**无覆盖盲区**，是学员两版采用的做法。")
    print("     代价：扫一遍要 %.1f s，赛时 180s 内最多扫 %.0f 遍。" % (
        sweep_total, 180 / sweep_total))
    print()
    print("  结论：")
    print("    · 若靶标方位大致已知（例如无人机下发了 TARGET_LIST）-> step 更快")
    print("    · 若完全不知道靶在哪 -> sweep 更可靠（无盲区）")
    print("    · 两者已做成参数 scan_mode，现场可切换")


# ================================================================ 5
def check_not_borrowed():
    sec("5. 明确**不借鉴**的两条（附理由）")

    print("  ✗ 1. 一步死区律（one-step deadbeat，走斜线版）")
    print("       算法：target = current_feedback + pixel_offset * K")
    print("       K 的推导：units/pixel = 11.38 * FOV(65°) / 640 ≈ 1.15")
    print("       不采纳的理由：")
    print("         · FOV=65° 是**假设值，未实测验证**")
    print("         · 量级检查：100px 偏移 × 1.15 = 115 units ≈ 10°")
    print("           以 4 帧流水线延迟（267ms @15fps）看，一次命令走 10°")
    print("           必然过冲")
    print("         · **作者自己也没启用**：STOP_ON_SIGHT 设为 false，")
    print("           且保留了增量式 CaremaMontorControl() —— 说明是实验分支")
    print()
    print("  ✗ 2. 激光红点闭环（两版都有）")
    print("       做法：用相机检测**激光红点**，以红点在画面中心作为命中确认。")
    print("       暂不可用的理由：")
    print("         · 本模型是**单类**（postprocess.h: OBJ_CLASS_NUM = 1），")
    print("           DetectionBox 里没有 class_id，无法区分'红点'与'靶标'")
    print("         · 要启用必须先重训模型（加红点类别）")
    print("       价值：如果将来模型支持多类，这比'靶标在中心'更可靠，")
    print("             因为它直接确认了激光落点。")
    print()
    print("  ✓ 另外确认：我的 TargetAimer 是单一 tick() 驱动，")
    print("     不存在学员'走直角'版的问题 —— 那里回调与 100Hz 循环")
    print("     **同时写 moveBaseControl 结构体**，是数据竞争。")
    print("     我保留'回调只更新状态、tick() 负责下发电机指令'的纪律。")


def main():
    print("TargetAimer 瞄准律验证 —— 借鉴学员工程后的三个新机制")
    print("来源：dzrknn_dataprocess 两版差异分析（走斜线 612 行 / 走直角 480 行）")

    check_grace_period()
    check_hold_window()
    check_fire_interval()
    check_scan_modes()
    check_not_borrowed()

    sec("总结论")
    cd = P["fire_cooldown_ms"] / 1000.0
    grace_s = P["lost_grace_ms"] / 1000.0
    pipeline_ms = 4 * 1000.0 / P["detect_rate_hz"]
    ok1 = P["lost_grace_ms"] >= pipeline_ms
    ok2 = P["fire_hold_px"] > P["fire_tolerance_px"]
    ok3 = cd >= TARGET_RECOVERY_S
    print("  ① 丢帧宽限 %d ms %s（流水线延迟 %.0f ms，余量 %.2fx）" % (
        P["lost_grace_ms"], "✓" if ok1 else "✗", pipeline_ms,
        P["lost_grace_ms"] / pipeline_ms))
    print("  ② 分层容差 对准 %d px / 保持 %d px %s" % (
        P["fire_tolerance_px"], P["fire_hold_px"], "✓" if ok2 else "✗"))
    print("  ③ 开火间隔 %.1f s vs 靶标恢复 %.1f s %s" % (
        cd, TARGET_RECOVERY_S, "✓" if ok3 else "✗"))
    print("  ④ 扫描模式已参数化（step / sweep），现场可切换")
    print("  ⑤ 一步死区律与红点闭环**未采纳**，理由见第 5 节")
    return 0 if (ok1 and ok2 and ok3) else 1


if __name__ == "__main__":
    sys.exit(main())
