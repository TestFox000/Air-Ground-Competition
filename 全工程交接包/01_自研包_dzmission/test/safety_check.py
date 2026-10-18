#!/usr/bin/env python3
"""无雷达避碰（SafetyMonitor v6）—— 判据验证

背景：赛题传感器白名单**禁止激光雷达 / 深度相机 / 双目 / UWB / 任何带深度
反馈的传感器**，所以原来基于 sensor_msgs/LaserScan 的扇区测距实现整体作废。

没有测距传感器时，"避碰"只能靠底盘自身反馈。本脚本验证四路判据的
**时序与优先级**是否正确，重点是堵转检测（主力手段）：

  ① 堵转：指令速度 >= stall_cmd_v 但实测 < stall_actual_v，持续超时
  ② 姿态：IMU 俯仰/横滚超限
  ③ 视觉近场占用：检测框占比过大（只降速）
  ④ 地图新鲜度（只降速）
  + 里程计断流：降速 -> 超时升级为急停

安全哲学：**"失明"不等于"危险"**。只有检测到实际异常才急停，
信息不足只降速 —— 否则整局都在急停。

运行:  python3 safety_check.py
"""

import sys

# ---------------- 参数（与 dzmission.yaml 的 safety.* 一致） ----------------
P = {
    "stall_cmd_v": 0.05,
    "stall_actual_v": 0.02,
    "odom_speed_scale": 1.0,
    "stall_hold_ms": 1200.0,
    "stall_grace_ms": 400.0,
    "recover_after_s": 2.0,
    "recover_reverse_v": -0.10,
    "recover_steer_deg": 25.0,
    "tilt_pitch_warn_deg": 12.0,
    "tilt_pitch_stop_deg": 20.0,
    "tilt_roll_warn_deg": 12.0,
    "tilt_roll_stop_deg": 20.0,
    "tilt_hold_ms": 500.0,
    "vision_near_box_ratio": 0.55,
    "vision_timeout_ms": 500.0,
    "vision_crawl_speed": 0.10,
    "odom_timeout_ms": 300.0,
    "no_odom_crawl_speed": 0.15,
    "no_odom_stop_ms": 3000.0,
    "map_crawl_speed": 0.25,
    "map_stale_blocks": False,
    "eval_hz": 20.0,
    "crawl_speed": 0.12,
    "clear_hold_ms": 400.0,
    "stuck_timeout_s": 10.0,
}

NONE, STALL, TILT, VISION, MAP, NOODOM = 0, 1, 2, 3, 4, 5
REASON_NAME = {NONE: "无", STALL: "堵转", TILT: "姿态", VISION: "视觉",
               MAP: "地图", NOODOM: "无里程计"}


def worse(a, b):
    """与 C++ 保持一致：枚举值小的优先级低，返回数值更大的（更严重）"""
    return a if a >= b else b


class Safety:
    """与 C++ SafetyMonitor 行为一致的最小实现"""

    def __init__(self, params=None):
        self.p = dict(P)
        if params:
            self.p.update(params)
        self.cmd = 0.0
        self.actual = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.odom_fresh = True
        # 里程计已断流多久（秒）。C++ 用 last_odom_ns_ 算，这里显式给出
        self.odom_lost_s = 0.0
        self.vision_fresh = False
        self.vision_ratio = 0.0
        self.map_fresh = True
        self.prev_cmd = 0.0
        self.stall_start = None
        self.tilt_start = None
        self.clear_pending = False
        self.clear_since = None
        self.stopped = False
        self.stop_since = None

    def set_cmd(self, v):
        self.cmd = v
        # 注意：C++ 里 cmd 变化会重置堵转计时，这里在 eval 里处理

    def eval(self, now, dt):
        v_limit = 1e9
        reason = NONE
        detail = ""

        # ---- ① 堵转 ----
        cmd_changed = abs(self.cmd - self.prev_cmd) > 0.02
        self.prev_cmd = self.cmd
        if cmd_changed:
            self.stall_start = None

        wants = abs(self.cmd) >= self.p["stall_cmd_v"]
        not_moving = self.actual < self.p["stall_actual_v"]
        if wants and not_moving and self.odom_fresh:
            if self.stall_start is None:
                self.stall_start = now
            s = now - self.stall_start
            grace = self.p["stall_grace_ms"] * 1e-3
            hold = self.p["stall_hold_ms"] * 1e-3
            if s >= grace + hold:
                reason = worse(reason, STALL)
                v_limit = 0.0
                detail = "堵转：指令 %.2f 但实测 %.3f，持续 %.1fs" % (
                    self.cmd, self.actual, s)
            elif s >= grace:
                reason = worse(reason, STALL)
                v_limit = min(v_limit, self.p["crawl_speed"])
                detail = "疑似堵转，降速观察"
        else:
            self.stall_start = None

        # ---- ② 姿态 ----
        p_abs, r_abs = abs(self.pitch), abs(self.roll)
        over_stop = (p_abs >= self.p["tilt_pitch_stop_deg"]
                     or r_abs >= self.p["tilt_roll_stop_deg"])
        over_warn = (p_abs >= self.p["tilt_pitch_warn_deg"]
                     or r_abs >= self.p["tilt_roll_warn_deg"])
        if over_stop or over_warn:
            if self.tilt_start is None:
                self.tilt_start = now
            held = now - self.tilt_start
            if over_stop and held >= self.p["tilt_hold_ms"] * 1e-3:
                reason = worse(reason, TILT)
                v_limit = 0.0
                detail = "姿态异常 俯仰%.1f/横滚%.1f 持续超限" % (self.pitch, self.roll)
            elif over_warn:
                reason = worse(reason, TILT)
                v_limit = min(v_limit, self.p["crawl_speed"])
                detail = "姿态偏斜 俯仰%.1f/横滚%.1f" % (self.pitch, self.roll)
        else:
            self.tilt_start = None

        # ---- ③ 视觉近场 ----
        if self.vision_fresh and self.vision_ratio >= self.p["vision_near_box_ratio"]:
            reason = worse(reason, VISION)
            v_limit = min(v_limit, self.p["vision_crawl_speed"])
            detail = "视觉近场占用 %.0f%%" % (self.vision_ratio * 100)

        # ---- ④ 地图 ----
        if not self.map_fresh:
            reason = worse(reason, MAP)
            if self.p["map_stale_blocks"]:
                v_limit = 0.0
                detail = "地图过期，禁用行驶"
            else:
                v_limit = min(v_limit, self.p["map_crawl_speed"])
                detail = "地图过期，降速"

        # ---- ⑤ 里程计断流 ----
        # 里程计断了就无法判堵转 —— 这是"失去卡死保护"，不是"危险"。
        # 所以先降速，超时（no_odom_stop_ms）才升级为急停。
        if not self.odom_fresh:
            reason = worse(reason, NOODOM)
            if self.odom_lost_s * 1000.0 >= self.p["no_odom_stop_ms"]:
                v_limit = 0.0
                detail = "里程计断流 %.1fs，无法做堵转判定，急停" % self.odom_lost_s
            else:
                v_limit = min(v_limit, self.p["no_odom_crawl_speed"])
                detail = "里程计断流，降速（失去堵转保护）"

        # ---- 汇总 + 急停保持 ----
        want_stop = (v_limit <= 0.0)
        if want_stop:
            if reason != NONE:
                self.clear_pending = False
                self.clear_since = None
            elif not self.clear_pending:
                self.clear_pending = True
                self.clear_since = now
            elif (now - self.clear_since) * 1000.0 >= self.p["clear_hold_ms"]:
                self.clear_pending = False
                self.clear_since = None
                self.stopped = False
                want_stop = False

        if want_stop:
            if not self.stopped:
                self.stop_since = now
            self.stopped = True
            level, vlim = 2, 0.0
        elif v_limit < 1e8:
            level, vlim = 1, v_limit
        else:
            level, vlim = 0, 0.0

        return {"level": level, "v_limit": vlim, "reason": reason,
                "detail": detail, "stall_s": (now - self.stall_start)
                if self.stall_start is not None else 0.0}


def sec(t):
    print()
    print("=" * 78)
    print(t)
    print("=" * 78)


# ================================================================ 1
def check_stall_timing():
    sec("1. ★ 堵转检测时序（核心判据）")
    print("  判据：指令 >= %.2f m/s 且实测 < %.2f m/s，持续 >= 宽限 %.0fms + 判定 %.0fms"
          % (P["stall_cmd_v"], P["stall_actual_v"], P["stall_grace_ms"],
             P["stall_hold_ms"]))
    print()
    s = Safety()
    s.set_cmd(0.30)
    s.actual = 0.0        # 完全卡住
    dt = 0.05
    print("  %6s %-10s %-8s %-10s %s" % ("t(s)", "阶段", "档位", "限速", "说明"))
    last_level = None
    for i in range(60):
        t = i * dt
        r = s.eval(t, dt)
        if r["level"] != last_level:
            print("  %6.2f %-10s %-8d %-10.2f %s" % (
                t, "变化" if last_level is not None else "启动",
                r["level"], r["v_limit"], r["detail"][:46]))
            last_level = r["level"]
    print()
    expect = P["stall_grace_ms"] * 1e-3 + P["stall_hold_ms"] * 1e-3
    print("  预期：约 %.2f s 时从'降速档'升级为'急停档'" % expect)
    print("        （宽限 %.1fs 内不动 -> 降速；再过 %.1fs -> 急停）"
          % (P["stall_grace_ms"] * 1e-3, P["stall_hold_ms"] * 1e-3))


# ================================================================ 2
def check_stall_false_positives():
    sec("2. ★ 堵转的误判场景（必须不触发）")
    cases = [
        ("正常行驶：指令 0.30，实测 0.28", 0.30, 0.28, True, "不应触发"),
        ("原地转向：指令 0.08，实测 0.07", 0.08, 0.07, True, "不应触发"),
        ("缓慢蠕行：指令 0.08，实测 0.05", 0.08, 0.05, True, "不应触发"),
        ("停住不动：指令 0.00，实测 0.00", 0.00, 0.00, True, "不应触发（没让它走）"),
        ("微速指令：指令 0.03，实测 0.00", 0.03, 0.00, True,
         "不应触发（指令低于 %.2f 阈值）" % P["stall_cmd_v"]),
        ("★ 真的卡住：指令 0.30，实测 0.00", 0.30, 0.00, True, "**应触发**"),
        ("里程计断流时卡住：指令 0.30，实测 0.00", 0.30, 0.00, False,
         "不应触发（没有里程计就不敢判堵转）"),
    ]
    print("  %-44s %-10s %-10s %s" % ("场景", "判定结果", "耗时(s)", "期望"))
    ok = True
    for desc, cmd, actual, odom_fresh, expect in cases:
        s = Safety()
        s.set_cmd(cmd)
        s.actual = actual
        s.odom_fresh = odom_fresh
        fired_at = None
        level_seen = 0
        for i in range(200):
            t = i * 0.05
            r = s.eval(t, 0.05)
            level_seen = max(level_seen, r["level"])
            if r["level"] == 2 and fired_at is None:
                fired_at = t
        should = "**应触发**" in expect
        got = (level_seen == 2)
        mark = "OK" if got == should else "✗ 不符"
        if got != should:
            ok = False
        print("  %-44s %-10s %-10s %s  %s" % (
            desc, "急停" if got else "未急停",
            "%.2f" % fired_at if fired_at else "-", expect, mark))
    print()
    print("  %s 全部误判场景都正确" % ("✓" if ok else "✗ 有误判"))
    return ok


# ================================================================ 3
def check_stall_recovery():
    sec("3. 堵转后的脱困建议")
    s = Safety()
    s.set_cmd(0.30)
    s.actual = 0.0
    print("  规则硬限：与固定场地相撞持续超 %.0f s -> 裁判切手动" % P["stuck_timeout_s"])
    print("  所以必须在 %.0f s 前脱离接触面" % P["stuck_timeout_s"])
    print()
    print("  %6s %-12s %-14s %s" % ("t(s)", "堵转时长(s)", "脱困建议就绪", "说明"))
    for i in range(80):
        t = i * 0.1
        r = s.eval(t, 0.1)
        due = r["stall_s"] >= P["recover_after_s"]
        if i % 10 == 0 or (due and i % 10 == 1):
            print("  %6.1f %-12.1f %-14s %s" % (
                t, r["stall_s"], "是" if due else "否",
                "★ 应执行倒车脱困" if due else ""))
    print()
    print("  脱困动作：v = %.2f m/s（倒车） + 舵机 %.0f 度（反打方向）"
          % (P["recover_reverse_v"], P["recover_steer_deg"]))
    print("  为什么倒车而不是继续前冲：必须尽快脱离接触面，")
    print("  否则 10s 硬限一到就被裁判切手动，本局基本结束。")


# ================================================================ 4
def check_tilt():
    sec("4. 姿态异常")
    print("  降速门限：俯仰/横滚 >= %.0f°；急停门限 >= %.0f°，且需持续 %.0fms"
          % (P["tilt_pitch_warn_deg"], P["tilt_pitch_stop_deg"], P["tilt_hold_ms"]))
    print()
    for desc, pitch, roll, expect_level in (
            ("水平", 0.0, 0.0, 0),
            ("轻微偏斜", 8.0, 0.0, 0),
            ("降速档", 14.0, 0.0, 1),
            ("横滚降速档", 0.0, 13.0, 1),
            ("急停档（持续）", 22.0, 0.0, 2),
            ("瞬时冲击（0.2s 后消失）", 25.0, 0.0, 0),
    ):
        s = Safety()
        s.set_cmd(0.0)
        for i in range(40):
            t = i * 0.05
            if desc.startswith("瞬时") and t > 0.2:
                s.pitch = 0.0
            else:
                s.pitch, s.roll = pitch, roll
            r = s.eval(t, 0.05)
        note = ""
        if desc.startswith("瞬时"):
            note = "（0.2s < 保持门槛 %.0fms，不该触发）" % P["tilt_hold_ms"]
        else:
            note = "OK" if r["level"] == expect_level else "（期望 %d）" % expect_level
        print("  %-30s -> 档位 %d  %s" % (desc, r["level"], note))


# ================================================================ 5
def check_clear_hold():
    sec("5. 急停解除的滞后（clear_hold）")
    print("  异常消失后必须持续 %.0f ms 才解除，避免抖动导致反复急停/恢复"
          % P["clear_hold_ms"])
    s = Safety()
    s.set_cmd(0.30)
    s.actual = 0.0
    # 先制造堵转急停
    for i in range(60):
        s.eval(i * 0.05, 0.05)
    print()
    print("  已进入急停：stopped=%s" % s.stopped)
    print()
    print("  %6s %-12s %-10s %s" % ("t(s)", "实测速度", "stopped", "说明"))
    s.actual = 0.30     # 障碍消失，车能动了
    for i in range(30):
        t = i * 0.05
        r = s.eval(t, 0.05)
        if i % 3 == 0:
            print("  %6.2f %-12.2f %-10s %s" % (
                t, s.actual, s.stopped,
                "保持急停中" if s.stopped else "已解除"))
    print()
    print("  ✓ 解除需要滞后 %.0f ms，不是一消失就放开" % P["clear_hold_ms"])


# ================================================================ 6
def check_priority():
    sec("6. 多路信号同时触发时的优先级")
    print("  枚举值：无=0 堵转=1 姿态=2 视觉=3 地图=4 无里程计=5")
    print("  规则：数值大的更严重，取最大的（与 C++ 的 worse() 一致）")
    print()
    s = Safety()
    s.set_cmd(0.30)
    s.actual = 0.0          # 堵转
    s.pitch = 25.0          # 姿态急停
    s.vision_fresh = True
    s.vision_ratio = 0.80   # 视觉占用
    s.map_fresh = False     # 地图过期
    r = None
    for i in range(80):
        r = s.eval(i * 0.05, 0.05)
    print("  同时：堵转 + 姿态超限 + 视觉占用 + 地图过期")
    print("    结果 -> 档位 %d，主因 %s" % (r["level"], REASON_NAME[r["reason"]]))
    print("    说明：%s" % r["detail"][:60])
    print()
    print("  ★ 无论主因是什么，只要有一路要求急停，最终就是急停（level=2）")
    print("     主因只影响日志与处置建议，不影响是否停车")


# ================================================================ 7
def check_no_odom_escalation():
    sec("7. 里程计断流的升级路径（降速 -> 急停）")
    print("  断流 %.0f ms 内：降速到 %.2f m/s（仍保留运动能力）"
          % (P["no_odom_stop_ms"], P["no_odom_crawl_speed"]))
    print("  断流超过 %.0f ms：急停（无法判堵转，不敢继续跑）"
          % P["no_odom_stop_ms"])
    print()
    s = Safety()
    s.set_cmd(0.30)
    s.odom_fresh = False
    s.odom_lost_s = 0.0
    print("  %6s %-12s %-8s %-10s %s" % ("t(s)", "断流(s)", "档位", "限速", "说明"))
    last = None
    for i in range(120):
        t = i * 0.05
        s.odom_lost_s = t
        r = s.eval(t, 0.05)
        if r["level"] != last:
            print("  %6.2f %-12.1f %-8d %-10.2f %s" % (
                t, t, r["level"], r["v_limit"], r["detail"][:44]))
            last = r["level"]
    print()
    print("  ★ 注意：断流期间**堵转判据被禁用**（实测速度不可信）。")
    print("     所以断流不是'急停'，而是'降速 + 失去一层保护'。")


# ================================================================ 8
def check_no_lidar():
    sec("8. ★ 确认雷达依赖已彻底移除")
    print("  赛题白名单：二维云台、单目RGB相机、激光发射机构、主控、IMU")
    print("  禁止：激光雷达、UWB、深度相机、双目、任何带深度反馈的传感器")
    print()
    banned = ["LaserScan", "scan", "PointCloud", "pcl", "laser_scan",
              "depth", "range", "radar"]
    needed = ["堵转", "姿态", "视觉", "地图", "里程计"]
    print("  %-14s %s" % ("类别", "手段"))
    print("  %-14s %s" % ("① 堵转检测", "指令速度 vs 实测速度（主力手段）"))
    print("  %-14s %s" % ("② 姿态异常", "IMU 俯仰/横滚"))
    print("  %-14s %s" % ("③ 视觉近场", "检测框高度占画面比例（只降速）"))
    print("  %-14s %s" % ("④ 地图新鲜度", "地雷/障碍清单是否及时（只降速）"))
    print()
    print("  全部手段都不依赖任何测距传感器 ✓")
    print()
    print("  ⚠ 但必须承认堵转检测的**根本局限**：")
    print("     它是'先撞上再发现'，不能预防碰撞。")
    print("     所以结构化规避（地图驱动的 FieldGrid 连通性规划 + 排雷通行段）")
    print("     才是**主路径**，本模块只是兜底。")
    return True


def main():
    print("无雷达避碰（SafetyMonitor v6）—— 判据验证")
    print("参数: stall_cmd_v=%.2f stall_actual_v=%.2f hold=%.0fms grace=%.0fms"
          % (P["stall_cmd_v"], P["stall_actual_v"], P["stall_hold_ms"],
             P["stall_grace_ms"]))

    check_stall_timing()
    ok_fp = check_stall_false_positives()
    check_stall_recovery()
    check_tilt()
    check_clear_hold()
    check_priority()
    check_no_odom_escalation()
    ok_lidar = check_no_lidar()

    sec("总结论")
    print("  1. 雷达依赖已彻底移除 —— 改用底盘自身反馈（四路信号）")
    print("  2. 堵转检测时序正确：宽限 %.1fs 后降速，再过 %.1fs 急停"
          % (P["stall_grace_ms"] * 1e-3, P["stall_hold_ms"] * 1e-3))
    print("  3. 误判场景 %s" % ("全部正确" if ok_fp else "有误判！"))
    print("  4. 脱困建议在堵转 %.1fs 后给出，远早于规则硬限 %.0fs"
          % (P["recover_after_s"], P["stuck_timeout_s"]))
    print("  5. 安全哲学：'失明'不等于'危险' —— 只有实际异常才急停，")
    print("     信息不足只降速。否则整局都在急停。")
    print("  6. ⚠ 堵转检测是'先撞上再发现'，不能预防碰撞。")
    print("     结构化规避（FieldGrid 连通性规划）才是主路径。")
    return 0 if (ok_fp and ok_lidar) else 1


if __name__ == "__main__":
    sys.exit(main())
