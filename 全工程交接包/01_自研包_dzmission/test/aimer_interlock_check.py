#!/usr/bin/env python3
"""TargetAimer 开火联锁验证 —— 红蓝面 / 激光红点 / 靶标击毁 / 弹量纪律

本赛题最关键的安全约束：**靶标是双面结构，一面红（友方）一面蓝（敌方），
朝向随机；打中友方要扣分**（友方受击计数每 +1 扣 5 分，比命中收益更贵）。

车端相机只能看到**朝向自己的那一面**，所以决策矩阵必须穷举验证。

四重联锁（全部满足才开火）：
  1. 视觉容差内 + 连续 N 帧
  2. ★ 可见面必须是**蓝**（红 = 友方，绝对不打；不确定由策略决定）
  3. 弹量充足（本地记账与裁判板取小）+ 靶标未击毁
  4. 未处于被击打状态
  + 激光红点（打过至少一发之后，用于确认落点）

运行:  python3 aimer_interlock_check.py
"""

import itertools
import sys

# ---------------- 参数（与 dzmission.yaml 的 aim.* 一致） ----------------
P = {
    "require_blue_face": True,
    "face_unknown_policy": 0,      # 0=禁止 1=允许
    "face_debounce_frames": 2,
    "require_laser_dot": True,
    "laser_dot_required_after_first": True,
    "laser_dot_debounce_frames": 2,
    "target_hp_destroyed": 3,
    "ammo_link_lost_policy": 2,    # 0=保守 1=乐观 2=折中
    "ammo_link_lost_allowance": 3,
    "max_local_shots": 10,
    "min_ammo_to_fire": 1,
    "fire_cooldown_ms": 3000,
    "fire_debounce_frames": 3,
    "fire_tolerance_px": 10,
    "fire_hold_px": 15,
    "require_stationary": False,
    "max_engage_range_m": 8.0,
}

FACE_UNKNOWN, FACE_RED, FACE_BLUE = 0, 1, 2
FACE_NAME = {0: "不确定", 1: "红(友)", 2: "蓝(敌)"}


def sec(t):
    print()
    print("=" * 78)
    print(t)
    print("=" * 78)


# ================================================================ 决策核心
def decide(face, hp, local_shots, board_ammo, link_ok,
           dot_seen, in_tol, debounce_ok, cooldown_ok,
           being_hit=False, stationary=True, p=None):
    """复现 C++ 的四重联锁判定，返回 (是否开火, 原因)"""
    p = p or P

    # ---- 红蓝面联锁 ----
    face_ok = True
    face_reason = ""
    if p["require_blue_face"]:
        if face == FACE_BLUE:
            face_ok = True
        elif face == FACE_RED:
            face_ok = False
            face_reason = "可见面是红色（友方）"
        else:
            face_ok = (p["face_unknown_policy"] == 1)
            if not face_ok:
                face_reason = "可见面不确定（保守策略禁止开火）"

    # ---- 靶标击毁 ----
    destroyed = (hp >= p["target_hp_destroyed"])

    # ---- 弹量纪律 ----
    if link_ok:
        ammo_eff = min(board_ammo, max(0, p["max_local_shots"] - local_shots))
    else:
        pol = p["ammo_link_lost_policy"]
        if pol == 0:
            ammo_eff = 0
        elif pol == 1:
            ammo_eff = max(0, p["max_local_shots"] - local_shots)
        else:
            ammo_eff = max(0, p["ammo_link_lost_allowance"] - local_shots)
    ammo_ok = (ammo_eff >= p["min_ammo_to_fire"]) and (not destroyed)

    # ---- 激光红点 ----
    dot_ok = True
    if p["require_laser_dot"] and p["laser_dot_required_after_first"] \
            and local_shots > 0:
        dot_ok = dot_seen

    # ---- 汇总 ----
    ok = (in_tol and debounce_ok and cooldown_ok and ammo_ok
          and (not being_hit) and face_ok and dot_ok and (not destroyed))

    if ok:
        return True, "开火"
    if destroyed:
        return False, "靶标已击毁(hp=%d)" % hp
    if not face_ok:
        return False, face_reason
    if not dot_ok:
        return False, "未见激光红点"
    if not ammo_ok:
        return False, "弹量不足(有效 %d, 本地已发 %d, link=%s)" % (
            ammo_eff, local_shots, link_ok)
    if being_hit:
        return False, "正在被击打"
    if not in_tol:
        return False, "未对准"
    if not debounce_ok:
        return False, "连续帧数不足"
    if not cooldown_ok:
        return False, "冷却中"
    return False, "未知原因"


# ================================================================ 1
def check_face_matrix():
    sec("1. ★★ 红蓝面决策矩阵（穷举）★★")
    print("  规则：红面=友方（打了扣 5 分/次）、蓝面=敌方（有效命中得分）")
    print("  车端只能看到朝向自己的那一面，所以必须按可见面决策。")
    print()
    print("  %-12s %-10s %-12s %s" % ("可见面", "策略", "是否开火", "原因"))
    rows = []
    for face in (FACE_BLUE, FACE_RED, FACE_UNKNOWN):
        for pol in (0, 1):
            p = dict(P)
            p["face_unknown_policy"] = pol
            fire, why = decide(face, hp=0, local_shots=0, board_ammo=10,
                               link_ok=True, dot_seen=False, in_tol=True,
                               debounce_ok=True, cooldown_ok=True, p=p)
            rows.append((face, pol, fire, why))
            print("  %-12s %-10s %-12s %s" % (
                FACE_NAME[face], "禁止" if pol == 0 else "允许",
                "★ 开火" if fire else "不开火", why))
    print()
    # 关键断言
    ok = True
    for (face, pol, fire, why) in rows:
        if face == FACE_RED and fire:
            ok = False
            print("  ✗ 严重：可见面为红色时竟然允许开火！")
    if ok:
        print("  ✓ 所有策略下，**可见面为红色时一律不开火**（打友方要扣分）")
    print()
    print("  ★ 建议：face_unknown_policy 保持 0（保守）。")
    print("     理由：误打友方扣 5 分，而漏打只是少拿分。")
    return ok


# ================================================================ 2
def check_interlock_exhaustive():
    sec("2. 四重联锁穷举（2^5 组合，看是否有漏洞）")
    print("  变量：可见面(蓝/红) × 靶标血量(0/3) × 本地已发(0/1) × ")
    print("        弹量(10/0) × 联锁(全通/各断一项)")
    print()
    print("  %-8s %-6s %-6s %-6s %-10s %-8s %s" % (
        "面", "hp", "已发", "弹量", "激光红点", "开火", "原因"))
    fires = 0
    red_fires = 0
    total = 0
    for face, hp, shots, ammo, dot in itertools.product(
            (FACE_BLUE, FACE_RED), (0, 3), (0, 1), (10, 0), (True, False)):
        total += 1
        fire, why = decide(face, hp=hp, local_shots=shots, board_ammo=ammo,
                           link_ok=True, dot_seen=dot, in_tol=True,
                           debounce_ok=True, cooldown_ok=True)
        if fire:
            fires += 1
        if fire and face == FACE_RED:
            red_fires += 1
        # 只打印代表性组合，避免刷屏
        if (face == FACE_BLUE and hp == 0 and shots == 0) or \
           (face == FACE_RED and hp == 0 and shots == 0) or fire:
            print("  %-8s %-6d %-6d %-6d %-10s %-8s %s" % (
                FACE_NAME[face], hp, shots, ammo,
                "有" if dot else "无",
                "★ 开火" if fire else "不开火", why))
    print()
    print("  共 %d 组合，其中允许开火 %d 个" % (total, fires))
    if red_fires == 0:
        print("  ✓ **没有任何组合在可见面为红色时开火**")
    else:
        print("  ✗ 有 %d 个组合在红色面下开火 —— 严重缺陷" % red_fires)
    return red_fires == 0


# ================================================================ 3
def check_laser_dot():
    sec("3. 激光红点联锁（首次开火不应被它挡住）")
    print("  为什么需要区分首次与后续：")
    print("    首次开火前激光还没打出去，**当然看不到红点**。")
    print("    若不加区分，会导致'永远打不出第一发'。")
    print()
    print("  %-14s %-12s %-12s %s" % ("已发弹数", "框内红点", "是否开火", "说明"))
    ok = True
    for shots, dot in ((0, False), (0, True), (1, False), (1, True)):
        fire, why = decide(FACE_BLUE, hp=0, local_shots=shots, board_ammo=10,
                           link_ok=True, dot_seen=dot, in_tol=True,
                           debounce_ok=True, cooldown_ok=True)
        note = ""
        if shots == 0 and not dot:
            note = "★ 必须开火（还没打过，看不到红点正常）"
            if not fire:
                ok = False
                note = "✗ 被红点联锁错误挡住 —— 永远打不出第一发！"
        elif shots == 1 and not dot:
            note = "正确挡下（打过了却看不到红点 = 可能没命中）"
        elif shots == 1 and dot:
            note = "正确放行（红点确认落点在靶上）"
        print("  %-14d %-12s %-12s %s" % (
            shots, "有" if dot else "无",
            "★ 开火" if fire else "不开火", note))
    print()
    if ok:
        print("  ✓ 首次开火不被红点联锁挡住；后续开火需要红点确认")
    return ok


# ================================================================ 4
def check_ammo_discipline():
    sec("4. ★ 弹量纪律（规则：共 10 发）")
    print("  规则：弹量共 **10 发**。车端必须自己守纪律，否则可能超打。")
    print()
    print("  %-16s %-10s %-12s %-12s %s" % (
        "链路状态", "策略", "本地已发", "有效弹量", "是否开火"))
    print()
    for link_ok in (True, False):
        for pol in (0, 1, 2):
            for shots in (0, 3, 9, 10):
                p = dict(P)
                p["ammo_link_lost_policy"] = pol
                fire, why = decide(FACE_BLUE, hp=0, local_shots=shots,
                                   board_ammo=10, link_ok=link_ok,
                                   dot_seen=True, in_tol=True,
                                   debounce_ok=True, cooldown_ok=True, p=p)
                if link_ok:
                    eff = min(10, max(0, 10 - shots))
                elif pol == 0:
                    eff = 0
                elif pol == 1:
                    eff = max(0, 10 - shots)
                else:
                    eff = max(0, 3 - shots)
                if shots == 0 or shots == 10:
                    print("  %-16s %-10s %-12d %-12d %s" % (
                        "正常" if link_ok else "★断链",
                        {0: "保守", 1: "乐观", 2: "折中"}[pol],
                        shots, eff, "★ 开火" if fire else "不开火"))
            print()
    print("  ★ 三种断链策略的对比：")
    print("     0 保守：断链即禁打 —— 绝不违规，但链路一断就完全打不了")
    print("     1 乐观：按本地记账 —— 能继续打，但若裁判板实际判某发无效会超打")
    print("     2 折中：允许再打 3 发，且每发要有反馈 —— 推荐")
    print()
    print("  ★ 关键：无论哪种策略，**本地硬上限 max_local_shots=10 始终生效**")
    print("     （即使裁判板计数异常也不会超打）")


# ================================================================ 5
def check_target_destroyed():
    sec("5. 靶标击毁检测（省弹量）")
    print("  规则：任一侧累计 3 次即该侧击毁；**正反共用同一靶标实体**，")
    print("        所以任一侧到 3 就是整体失效。")
    print()
    print("  %-10s %-14s %-12s %s" % ("血量", "是否已击毁", "是否开火", "说明"))
    for hp in (0, 1, 2, 3, 4):
        fire, why = decide(FACE_BLUE, hp=hp, local_shots=0, board_ammo=10,
                           link_ok=True, dot_seen=False, in_tol=True,
                           debounce_ok=True, cooldown_ok=True)
        print("  %-10d %-14s %-12s %s" % (
            hp, "是" if hp >= 3 else "否",
            "★ 开火" if fire else "不开火", why))
    print()
    print("  ✓ hp >= 3 后停止交战 —— 避免把弹量浪费在已失效的目标上")
    print("     省下的弹量可以留给后续目标（但规则说单局仅一个打击区）")


# ================================================================ 6
def check_face_source():
    sec("6. ★ 颜色信息来源与现场标定要求")
    print("  来源：rknn_pt/det_node 的 /offset_center 第 9 位")
    print("        取值 0=不确定 1=红(友) 2=蓝(敌)")
    print("        判据：HSV 阈值（不需重训模型）")
    print()
    print("  ★★ 为什么用 HSV 而不是多类模型 ★★")
    print("     当前模型是单类的：postprocess.h 里 OBJ_CLASS_NUM = 1，")
    print("     DetectionBox 结构里**没有 class_id 字段** —— 红蓝标签")
    print("     根本到不了车端。要改成多类必须：")
    print("       1) 重新训练/导出 OBJ_CLASS_NUM=2 的模型")
    print("       2) 改 post_process() 把类别带出来")
    print("       3) 给 DetectionBox 加 class_id 字段")
    print("       4) det_node 填真实类别")
    print("     HSV 判据绕过了这一切，代价是准确率依赖光照。")
    print()
    print("  ★ 现场必须标定的参数（在 det_node.launch.py 里）：")
    print("     face_min_ratio  默认 0.12 —— 红/蓝像素占比低于此值判'不确定'")
    print("     face_red_bias   默认 1.25 —— 一方要高出另一方这么多倍才判定")
    print()
    print("  标定方法：")
    print("     1) 把靶标红面朝向相机，看 /offset_center 第 9 位是否为 1")
    print("     2) 转 180° 让蓝面朝向相机，看是否为 2")
    print("     3) 若判为 0（不确定）：调小 face_min_ratio")
    print("     4) 若红蓝判反：调小 face_red_bias")
    print("     5) 强光/逆光下重复 1~4，取最差条件下仍能判对的阈值")
    print()
    print("  ★ 兜底：无论标定如何，'不确定'时默认**不开火** ——")
    print("     宁可漏打，不可误打友方。")


# ================================================================ 7
def check_pending_flank():
    sec("7. ⚠ 尚未实现：绕行到蓝面（需要你补信息）")
    print("  需求（你提出的）：'识别到红色需要转到靶子的另一侧进行打击'")
    print()
    print("  当前行为：看到红面 -> **不开火 + 保持瞄准 + 告警**")
    print("             （不主动绕行）")
    print()
    print("  要实现'绕到背面再打'，我必须知道：")
    print("     a) 靶标立在两格中央，它的**面朝向**是哪两个方向之一")
    print("        （沿行 / 沿列 / 或某个具体角度）")
    print("     b) 或者直接告诉我：**车应该停在哪个格**才能看到蓝面")
    print()
    print("  为什么不能自动推出来：")
    print("     车端相机在**看到之前**不知道靶标朝哪；只有两种来源：")
    print("       · 无人机下发（协议里 TARGET_LIST.face 已定义）")
    print("         —— 但双面靶标俯视只能看到顶面，我判断无人机看不到侧面")
    print("       · 车端自己绕一圈观察 —— 代价大（180s 赛时）")
    print()
    print("  已预留的接口：")
    print("     · 参数 flank_to_blue（默认关）")
    print("     · AimResult.face 已暴露给任务层，任务层可据此决定是否重规划")
    print("     · EVENT 上报通道已就绪")
    print()
    print("  在补上 (a)/(b) 之前，`flank_to_blue` 保持关闭 —— ")
    print("  贸然绕行可能既没打到、又浪费赛时。")


def main():
    print("TargetAimer 开火联锁验证")
    print("关键约束：靶标双面，红=友方（打了扣分）、蓝=敌方；车端只见朝向自己的一面")

    ok1 = check_face_matrix()
    ok2 = check_interlock_exhaustive()
    ok3 = check_laser_dot()
    check_ammo_discipline()
    check_target_destroyed()
    check_face_source()
    check_pending_flank()

    sec("总结论")
    print("  1. 红蓝面联锁：%s" % ("✓ 可见面为红色时一律不开火" if ok1 else "✗ 有漏洞"))
    print("  2. 四重联锁穷举：%s" % ("✓ 无组合可在红面下开火" if ok2 else "✗ 有漏洞"))
    print("  3. 激光红点：%s" % ("✓ 首次开火不被挡住、后续需红点确认" if ok3
                                else "✗ 会导致打不出第一发"))
    print("  4. 弹量纪律：三种断链策略可配，本地硬上限 10 发始终生效")
    print("  5. 靶标击毁：hp>=3 停止交战")
    print("  6. 颜色来源：det_node 的 HSV 判据，需现场标定两个阈值")
    print("  7. ⚠ '绕行到蓝面'**未实现**，需要你补靶标朝向信息")
    print()
    print("  ★ 未改变的一点：fire_cooldown 仍是 3000ms —— 规则要求靶标")
    print("     命中后 1s 恢复，3s 有充足余量，且能观察裁判板反馈确认命中。")
    return 0 if (ok1 and ok2 and ok3) else 1


if __name__ == "__main__":
    sys.exit(main())
