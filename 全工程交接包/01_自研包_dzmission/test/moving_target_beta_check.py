#!/usr/bin/env python3
"""方案 β 验证：移动靶提前量射击

与 α 的区别（重要）：
  α  等靶标【换向点】（速度过零）再开火 -> 延迟位移约等于 0，安全但射速低
  β  在靶标【匀速运动时】开火，把延迟造成的位移【提前补偿掉】
     -> 射速高，但补偿必须准，否则打空（弹量只有 10 发）

★ β 的核心洞察：
  激光与相机【同轴】（都在云台上，Q14 已确认）。所以：
    1. 云台指向决定激光方向 —— 要命中，云台必须指向【未来命中点】
    2. 相机画面只用来【估计靶标运动】，不决定命中
  因此提前量的物理量 = 【从画面那一帧到激光真正打出去的时间】 x 靶标速度

延迟链（每段都要算，漏一段就偏一段）：
  T_exposure   相机曝光 + 图像传输
  T_pipeline   det_node 流水线（4 帧，见 compute_audit）
  T_servo      云台从当前位置转到目标位置
  T_fire       开火指令到激光实际出光
  ------------------------------------------------
  T_total      上列之和
  提前量(像素) = 靶标像素速度(px/s) x T_total

运行:  python3 moving_target_beta_check.py
"""

import math
import sys

# ---------------- 参数（与 dzmission.yaml 的 aim.* 一致） ----------------
P = {
    "detect_rate_hz": 15.0,
    "pipeline_frames": 4,
    "fire_cooldown_ms": 3000,
    "fire_tolerance_px": 10,
    "fire_hold_px": 15,
    "turn_around_v_thresh_px_s": 25.0,
    # β 新增
    "lead_enable": True,
    "lead_exposure_ms": 33.0,      # 1/30s 曝光 + 传输
    "lead_servo_ms": 60.0,         # 云台小角度调整典型耗时（实测标定）
    "lead_fire_ms": 20.0,          # 开火指令 -> 实际出光
    "lead_velocity_frames": 5,     # 速度估计窗口（帧）
    "lead_min_confidence": 3,      # 至少几帧有效观测才启用提前量
    "lead_aim_tolerance_px": 8,    # 云台【已指向瞄准点】的容差
}

IMG_W = 640
HIT_PER_COUNTER = 2      # 规则：每 2 次有效命中 +1 计数
COUNTER_MAX = 3
AMMO = 10


def sec(t):
    print()
    print("=" * 78)
    print(t)
    print("=" * 78)


def latency_breakdown(p=None):
    p = p or P
    frame_ms = 1000.0 / p["detect_rate_hz"]
    pipe_ms = p["pipeline_frames"] * frame_ms
    total = (pipe_ms + p["lead_exposure_ms"] +
             p["lead_servo_ms"] + p["lead_fire_ms"])
    return {
        "exposure": p["lead_exposure_ms"],
        "pipeline": pipe_ms,
        "servo": p["lead_servo_ms"],
        "fire": p["lead_fire_ms"],
        "total": total,
    }


# ================================================================ 1
def check_latency_chain():
    sec("1. ★ 延迟链分解（提前量必须补偿的总时间）")

    lat = latency_breakdown()
    print("  %-36s %8s" % ("环节", "耗时(ms)"))
    print("  " + "-" * 48)
    print("  %-36s %8.0f" % ("相机曝光 + 图像传输", lat["exposure"]))
    print("  %-36s %8.0f" % ("det_node 流水线（4 帧）", lat["pipeline"]))
    print("  %-36s %8.0f" % ("云台转到瞄准点（小角度）", lat["servo"]))
    print("  %-36s %8.0f" % ("开火指令 -> 激光出光", lat["fire"]))
    print("  " + "-" * 48)
    print("  %-36s %8.0f   <== T_total" % ("合计", lat["total"]))
    print()

    miss = lat["total"] - lat["pipeline"]
    print("  ★ 只算流水线（= α 只回避不算补偿）会【少补偿 %.0f ms】，" % miss)
    print("     即少补偿 %.0f%%。" % (100.0 * miss / lat["total"]))
    print("     这段差值在靶速 0.44 m/s 时相当于 %.1f cm。" % (
        0.44 * miss / 1000.0 * 100))
    print("     靶标直径只有 9 cm —— 漏这一段就会打偏。")
    print()
    print("  ★ 现场标定方法（三个新增量都需实测）：")
    print("     - lead_servo_ms ：让云台做小角度阶跃，看 camemo_feedback 到位时间")
    print("     - lead_fire_ms  ：发一次开火，用高速摄像或声音测延迟")
    print("     - lead_exposure_ms：查相机驱动配置的曝光时间")
    print()
    print("  ★ 若不确定：先把 lead_servo_ms 设 0，只补偿流水线延迟，")
    print("     然后按脱靶方向逐步加大 —— 这是安全的标定顺序。")


# ================================================================ 2
def check_lead_offset():
    sec("2. ★ 提前量随靶速与距离的变化")

    lat = latency_breakdown()
    t = lat["total"] / 1000.0
    fov, w, d = 65.0, 640.0, 0.70
    print("  T_total = %.0f ms" % lat["total"])
    print()
    print("  (A) 提前量随靶速变化（standoff 2 格 = 70cm、FOV 65、宽 640）")
    print("  %-14s %-18s %-16s %s" % ("靶速(m/s)", "像素速度(px/s)", "提前量(px)", "相对容差"))
    print("  " + "-" * 62)
    for v in (0.1, 0.22, 0.44, 0.6):
        omega = math.degrees(v / d)
        px_s = omega / fov * w
        lead_px = px_s * t
        print("  %-14.2f %-18.0f %-16.0f %.1f 倍（容差 %dpx）" % (
            v, px_s, lead_px, lead_px / P["fire_tolerance_px"],
            P["fire_tolerance_px"]))
    print()
    lead44 = (math.degrees(0.44 / d) / fov * w) * t
    print("  ★ 靶速 0.44 m/s 时提前量约 %.0f px，是容差(%dpx)的 %.1f 倍。" % (
        lead44, P["fire_tolerance_px"], lead44 / P["fire_tolerance_px"]))
    print("     不补偿绝无可能命中 —— 这解释了为什么 α 必须等换向点。")
    print()
    print("  (B) 提前量随距离变化（同一靶速下，站得越远像素提前量越小）")
    print("  %-12s %-14s %-18s %s" % ("standoff", "距离(cm)", "像素速度(px/s)", "提前量(px)"))
    print("  " + "-" * 62)
    for cells in (1, 2, 3, 4):
        dd = cells * 0.35
        omega = math.degrees(0.44 / dd)
        px_s = omega / fov * w
        print("  %-12s %-14.0f %-18.0f %.0f" % (
            "%d 格" % cells, dd * 100, px_s, px_s * t))
    print()
    print("  ★ 注意：提前量的【物理量】（cm）与距离无关，= v x T_total = %.1f cm。" % (
        0.44 * t * 100))
    print("     但【像素量】随距离变化。所以参数存 T_total（物理时间），")
    print("     由观测到的 v(px/s) 现算提前量 —— 像素速度是可直接观测的量。")


# ================================================================ 3
def check_firing_logic():
    sec("3. ★ 开火时机：β 的正确判据")

    print("  ★ 常见错误：跟踪靶标 + 每帧把云台指向预测位置，然后开火")
    print("     问题：瞄准点本身在移动，云台永远追不上（有 T_total 的滞后）。")
    print("     结果：云台一直在【追】一个移动的点，稳态误差 = v x T_total。")
    print()
    print("  ★★ β 的正确做法：【预瞄固定点 + 穿越触发】★★")
    print("     1. 由观测到的靶标运动，算出它的【运动轴线】与速度")
    print("     2. 在轴线上选一个【固定瞄准点】（通常取画面中心对应的靶面点）")
    print("     3. 云台【预先转到】该瞄准点并稳定（不跟踪靶标）")
    print("     4. 预测靶标【到达瞄准点】的时刻 t_hit")
    print("     5. 在 t_fire = t_hit - T_total 时刻开火")
    print()
    print("  好处：云台几乎不动（只在换向时重选瞄准点），")
    print("        命中点由【开火时机】控制，而不是由云台跟踪精度控制。")
    print("        这比跟踪+补偿稳定得多 —— 因为不需要高带宽跟踪。")
    print()
    print("  ★ 一个额外好处：云台不动 -> 也不会有云台运动带来的画面抖动,")
    print("     视觉估计更稳。")
    print()
    print("  %-12s %-16s %-18s %s" % ("靶速(m/s)", "半周期(s)", "冷却限制", "有效开火机会"))
    print("  " + "-" * 68)
    cd = P["fire_cooldown_ms"] / 1000.0
    for v in (0.22, 0.44, 0.6):
        half = 0.35 / v
        limited = max(half, cd)
        print("  %-12.2f %-16.2f %-18s %s" % (
            v, half, "%.1fs" % cd,
            "每 %.1fs 一发（冷却主导）" % limited if cd > half
            else "每 %.1fs 一发" % half))
    print()
    print("  ★ 关键：冷却 3.0s 与半周期相当或更长。")
    print("     若半周期 0.8s（v=0.44），冷却 3.0s 意味着靶标经过瞄准点多次")
    print("     才轮到开火一发。所以 β 的射速上限由【冷却】决定，不是瞄准。")


# ================================================================ 4
def check_pixel_velocity_estimation():
    sec("4. ★ 像素速度估计（β 的输入质量决定命中率）")

    print("  β 依赖【靶标像素速度】的准确性。误差来源：")
    print()
    print("  %-30s %-18s %s" % ("误差源", "量级", "缓解方式"))
    print("  " + "-" * 72)
    rows = [
        ("检测框抖动（正负 2px）", "正负 30 px/s",
         "中值滤波（α 已实现，3 帧窗口）"),
        ("靶标换向附近", "速度定义不成立",
         "改用 α 的 dwell 判据（不信任速度）"),
        ("靶标被遮挡或丢帧", "速度估计跳变",
         "丢帧时清空速度历史（已实现 vision_age 重置）"),
        ("透视：靶标沿轨道移动", "像素速度非线性",
         "只在靶标接近画面中心时信任速度估计"),
    ]
    for (a, b, c) in rows:
        print("  %-30s %-18s %s" % (a, b, c))
    print()
    print("  ★ 结论：β 【只在靶标接近画面中心时信任速度估计】。")
    print("     在画面边缘（视角大）时透视使像素速度失真 -> 不开火。")
    print("     这天然与【预瞄固定瞄准点】吻合：瞄准点就取画面中心。")
    print()
    print("  ★ 置信度门槛：至少 lead_min_confidence(=%d) 帧有效观测才启用。" %
          P["lead_min_confidence"])
    print("     观测不足时【退回 α 的换向点策略】（安全兜底）。")


# ================================================================ 5
def check_alpha_beta_combined():
    sec("5. ★★ α + β 组合策略（推荐的上线形态）")

    print("  两者不是替代关系，而是【互补】：")
    print()
    print("  %-24s %-24s %s" % ("场景", "用哪个", "理由"))
    print("  " + "-" * 72)
    rows = [
        ("靶标速度可估计且较大", "β（提前量）", "射速高，补偿掉 T_total 位移"),
        ("靶标在换向点附近", "α（dwell）", "速度定义不成立，等停更稳"),
        ("靶标接近画面中心", "β", "像素速度接近真实，补偿准"),
        ("靶标在画面边缘", "α 或不开火", "透视使像素速度失真"),
        ("速度估计置信度不足", "α（兜底）", "观测不足时用保守策略"),
        ("已连续 N 发未命中", "退回 α", "说明补偿有系统偏差"),
    ]
    for (a, b, c) in rows:
        print("  %-24s %-24s %s" % (a, b, c))
    print()
    print("  ★ 实现形态：同一套联锁里加一个【策略选择】，而不是两套代码。")
    print("     lead_enable 打开 -> 优先 β；条件不满足 -> 自动退回 α。")
    print("     这样即使 β 标定不准，也【不会比 α 更差】（有兜底）。")


# ================================================================ 6
def check_risk():
    sec("6. β 的风险（诚实说明）")

    print("  1. 标定误差会直接变成脱靶")
    print("     T_total 差 50ms，靶速 0.44 m/s 时脱靶 2.2cm；")
    print("     靶标半径 4.5cm -> 还在容差内，但余量只剩一半。")
    print()
    print("  2. 云台响应时间不稳定")
    print("     lead_servo_ms 取决于角度大小与舵机状态，不是常数。")
    print("     小角度快、大角度慢 -> 用固定值一定有残差。")
    print()
    print("  3. 弹量风险（最重要）")
    print("     全局限 10 发。若 β 补偿不准，会连续打空；")
    print("     而 α 每次都在 v 约等于 0 时打，几乎不会空。")
    print("     所以【必须有连续 N 发未命中就退回 α 的机制】。")
    print()
    print("  4. 无法离线验证")
    print("     以上全是纸面推导。T_servo / T_fire 只能实测。")
    print("     我无法验证 β 在实车上是否真的更准。")
    print()

    lat = latency_breakdown()
    cd = P["fire_cooldown_ms"] / 1000.0
    need = HIT_PER_COUNTER * COUNTER_MAX
    print("  ★★ 定量对比（冷却 3.0s 下，%d 次有效命中耗时）★★" % need)
    print("  %-10s %-28s %s" % ("策略", "开火间隔下限", "净耗时"))
    print("  " + "-" * 58)
    print("  %-10s %-28s %.1f s" % ("α", "max(换向间隔, 3.0s)", need * cd))
    print("  %-10s %-28s %.1f s" % ("β", "max(穿越间隔, 3.0s)", need * cd))
    print("  -> 【相同】（都被冷却限制）")
    print()
    print("  ★★ 所以 β 的真正价值不是【打得快】，而是：")
    print("     - 不依赖靶标恰好停在换向点（若靶标不停，α 就打不了）")
    print("     - 靶标行程中任何位置都能打 -> 开火窗口更连续")
    print()
    print("  ★★ 我的建议 ★★")
    print("     先上车验证 α 能否打中。")
    print("       - 若 α 能打中：β 的边际收益有限（射速被冷却卡住）")
    print("       - 若 α 打不中（靶标换向太快/窗口太窄）：再上 β")
    print("     所以本轮我把 β 实现成【可选 + 有兜底】，而不是替换 α。")


def main():
    print("方案 β 验证：移动靶提前量射击")
    print("核心：激光与相机同轴 -> 云台指向决定命中；提前量 = v(px/s) x T_total")

    check_latency_chain()
    check_lead_offset()
    check_firing_logic()
    check_pixel_velocity_estimation()
    check_alpha_beta_combined()
    check_risk()

    sec("总结论")
    lat = latency_breakdown()
    fov, w, d = 65.0, 640.0, 0.70
    lead44 = (math.degrees(0.44 / d) / fov * w) * lat["total"] / 1000.0
    print("  1. T_total = %.0f ms（比只算流水线多 %.0f ms，不可忽略）" % (
        lat["total"], lat["total"] - lat["pipeline"]))
    print("  2. 靶速 0.44 m/s 时提前量约 %.0f px = 容差的 %.1f 倍" % (
        lead44, lead44 / P["fire_tolerance_px"]))
    print("  3. 正确做法是【预瞄固定点 + 穿越触发】，不是【跟踪 + 补偿】")
    print("     后者要求云台高带宽跟踪移动瞄准点，稳态误差 = v x T_total")
    print("  4. 射速上限由 3.0s 冷却决定 —— α 与 β 差别不大")
    print("     β 的真正价值：不依赖靶标停在换向点，开火窗口更连续")
    print("  5. 必须有【连续 N 发未命中退回 α】的兜底（弹量只有 10 发）")
    print("  6. 三个新增标定量（servo/fire/exposure）只能实测")
    print()
    print("  => 实现形态：α+β 同一套联锁 + 策略选择 + 兜底，")
    print("     而不是用 β 替换 α。这样 β 标定不准也不会比 α 差。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
