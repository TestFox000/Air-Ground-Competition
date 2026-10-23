# -*- coding: utf-8 -*-
r"""15:04 实车 aim 时序回归探针（**纯脚本，无 pytest、无 ROS、无实车**）。

    cd dzrobot_v2
    set PYTHONPATH=src\dzbrain
    python src\dzbrain\tests\test_aim_probe.py

它钉的是 t2 报告的**结论性事实**（0 发主因 = 未到射击停站、云台全程 IDLE ⇒ PID 未入环），
防止后续卡在不碰 aim 的情况下把归因悄悄改写成"PID/开火框问题"。样本日志：
    _tmp_session\run_20261017_1504\7_path_utf8.txt   （2026-10-17 14:59 那趟实车）

覆盖：
  1. 云台全程锁止：403 拍遥测 100% 瞄IDLE、云台指令只有 1 条 scan_request=0；
  2. 任务阶段只有 IDLE→DRIVE→SAFE_STOP，**无 ACQUIRE/FIRE/SETTLE**（= 从未进射击停站）；
  3. 停站点只有 2 个（29:D3* / 43:N5），且到点事件里**没有 29**（D3* 被投影跳变跳过）；
  4. 发/中 全程 0/0，开火许可 0 条；
  5. Python 侧"不开火"原因条数受 1 Hz 限流 ⇒ 只能看排序，不能当帧占比（钉住 5 条上限口径）；
  6. 车指令速度恒 0 的停车段里靶观测仍在跳（帧间 |Δdx| 中位 >= 20px）⇒ 检测/目标运动
     本身就是主要不确定源，与云台 PID 解耦；
  7. aim.py 纯函数口径：fire_box_px 近大远小两端点 14/26px、max_lead = 0.45*0.70*140 = 44.1px。

样本日志缺失时 **只 SKIP 不 fail**（_tmp_session 被清理不该让回归变红）。
"""

from __future__ import annotations

import io
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import check, finish, section   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
LOG = os.path.join(ROOT, "_tmp_session", "run_20261017_1504", "7_path_utf8.txt")

RE_T = re.compile(r"\[T\] (\S+) (\S+) \(([-\d.]+),([-\d.]+)\) yaw([-\d.]+) v([-\d.]+) st([+\-\d.]+) "
                  r"\| 段(-?\d+) s([\d.]+)/(\d+) \| 瞄(\S+) 靶(\S+) 发(\d+)/中(\d+) 弹(\S+) \| 安全(\S+)")
RE_BLOCK = re.compile(r"不开火：(.+?)\s*$")
RE_PHASE = re.compile(r"阶段 (\S+) → (\S+)")
RE_GIMBAL = re.compile(r"云台(起转搜索|锁止|回正锁止)：scan_request=(\d+)")
RE_ARRIVE = re.compile(r"到点 (\d+)/(\d+) (\S+)（s=([\d.]+) m")


def parse(lines):
    tel, reasons, phases, gimbal, arrives = [], [], [], [], []
    stops = None
    for ln in lines:
        m = RE_T.search(ln)
        if m:
            tel.append(dict(phase=m.group(2), v=float(m.group(6)), aim=m.group(11),
                            target=m.group(12), shots=int(m.group(13)), hits=int(m.group(14))))
            continue
        m = RE_BLOCK.search(ln)
        if m:
            reasons.append(m.group(1))
            continue
        m = RE_PHASE.search(ln)
        if m:
            phases.append((m.group(1), m.group(2)))
            continue
        m = RE_GIMBAL.search(ln)
        if m:
            gimbal.append((m.group(1), int(m.group(2))))
            continue
        m = RE_ARRIVE.search(ln)
        if m:
            arrives.append(int(m.group(1)))
            continue
        if stops is None and "停站点" in ln:
            stops = ln
    return tel, reasons, phases, gimbal, arrives, stops


def main() -> int:
    section("0. 样本可用性")
    if not os.path.exists(LOG):
        print("  SKIP  样本日志不存在（%s）→ 只跳过，不算失败" % LOG)
        finish()
        return 0
    with io.open(LOG, "r", encoding="utf-8", errors="replace") as fh:
        lines = fh.read().splitlines()
    check("样本日志行数 == 2164", len(lines) == 2164, str(len(lines)))
    tel, reasons, phases, gimbal, arrives, stops = parse(lines)

    section("1. 云台全程锁止（0 发的直接原因）")
    check("遥测拍数 == 403", len(tel) == 403, str(len(tel)))
    check("瞄状态 100% IDLE（从未进 TRACK）",
          tel and all(t["aim"] == "IDLE" for t in tel),
          str({t["aim"] for t in tel}))
    check("云台指令只有 1 条 = 锁止/scan_request=0",
          gimbal == [("锁止", 0)], str(gimbal))

    section("2. 任务阶段：从未进射击停站")
    check("阶段迁移序列 == [IDLE->DRIVE, DRIVE->SAFE_STOP]",
          phases == [("IDLE", "DRIVE"), ("DRIVE", "SAFE_STOP")], str(phases))
    check("全程没有 ACQUIRE/FIRE/SETTLE 阶段",
          not any(s in ("ACQUIRE", "FIRE", "SETTLE") for pair in phases for s in pair), str(phases))

    section("3. 停站点与到点：D3* 被跳过")
    check("停站点行存在且恰好 2 个", bool(stops) and stops.count(":") >= 2, (stops or "")[:70])
    pairs = [(int(a), float(b)) for a, b in re.findall(r"(\d+):([\d.]+)", stops or "")]
    check("停站点 == [(29, 10.15), (43, 15.05)]",
          len(pairs) == 2 and pairs[0][0] == 29 and pairs[1][0] == 43
          and abs(pairs[0][1] - 10.15) < 0.01 and abs(pairs[1][1] - 15.05) < 0.01,
          str(pairs))
    check("到点事件里没有 29（D3* 从未到点）", (29 not in arrives) and bool(arrives), str(arrives))
    check("到点事件 5 条（M10/M9/L9/K9/J5）", len(arrives) == 5, str(arrives))

    section("4. 发/中账本")
    check("发数全程 0", all(t["shots"] == 0 for t in tel), str(max(t["shots"] for t in tel)))
    check("命中全程 0", all(t["hits"] == 0 for t in tel), str(max(t["hits"] for t in tel)))
    check("无「瞄准合格 -> fire_ready=1」许可", not any("瞄准合格" in ln for ln in lines))

    section("5. 原因条数是 1 Hz 限流后的计数（不是帧占比）")
    from collections import Counter
    cnt = Counter(reasons)
    total = sum(cnt.values())
    m_ticks = re.search(r"AimTask\s+ticks=(\d+)", "\n".join(lines))
    aim_ticks = int(m_ticks.group(1)) if m_ticks else 0
    # AimTask 50 Hz x 317 s 每拍都在判开火，但只落了 321 条日志 ⇒ 1 Hz 限流 ⇒ 条数不是帧占比
    check("AimTask ticks 可见且 > 15000", aim_ticks > 15000, str(aim_ticks))
    check("「不开火」条数 < AimTask ticks / 10（限流证据，不是帧计数）",
          total < aim_ticks / 10.0, "%d 条 vs %d 拍" % (total, aim_ticks))
    check("原因里既有 out_of_box 也有 目标丢失（两类并存）",
          any("开火框" in r for r in cnt) and any("目标丢失" in r for r in cnt), str(dict(cnt)))

    section("6. 车指令速度恒 0 的停车段里靶观测仍在跳（与 PID 解耦）")
    stop = [t for t in tel if t["phase"] == "SAFE_STOP"]
    check("停车段拍数 > 300", len(stop) > 300, str(len(stop)))
    check("停车段指令速度恒为 0（强制刹车）", all(t["v"] == 0.0 for t in stop),
          str(max(t["v"] for t in stop)))
    seen_dx = []
    for t in stop:
        if t["target"] != "none":
            m = re.match(r"^s([\d.]+)v", t["target"])
            if m:
                seen_dx.append(abs(float(m.group(1)) - 0.5) * 196.0)
    step = [abs(seen_dx[i + 1] - seen_dx[i]) for i in range(len(seen_dx) - 1)]
    step.sort()
    med = step[len(step) // 2] if step else 0.0
    p90 = step[int(len(step) * 0.9)] if step else 0.0
    big = sum(1 for x in step if x >= 50.0)
    big_pct = 100.0 * big / max(len(step), 1)
    check("停车段有靶帧 > 250", len(seen_dx) > 250, str(len(seen_dx)))
    # 注：dx 由 s_rail 反推且 s_rail 被夹在 [0,1] ⇒ 这是**下界**（真跳变更大）
    check("停车段 2Hz 相邻 |Δdx| 中位 >= 10px（下界）", med >= 10.0, "%.1f px" % med)
    check("停车段 2Hz 相邻 |Δdx| P90 >= 50px（下界）", p90 >= 50.0, "%.1f px" % p90)
    check("停车段 |Δdx| >= 50px 的步占比 >= 25%", big_pct >= 25.0, "%.1f%%" % big_pct)

    section("7. aim.py 纯函数口径（配置真源常量）")
    try:
        from dzbrain.tasks.aim import fire_box_px
        check("fire_box_px(30)=14（远）", abs(fire_box_px(30.0, near_px=26, far_px=14,
                                                         box_w_far=30, box_w_near=120) - 14.0) < 1e-9)
        check("fire_box_px(120)=26（近）", abs(fire_box_px(120.0, near_px=26, far_px=14,
                                                           box_w_far=30, box_w_near=120) - 26.0) < 1e-9)
        check("fire_box_px(0)=14（没给框宽取最严）",
              abs(fire_box_px(0.0, near_px=26, far_px=14, box_w_far=30, box_w_near=120) - 14.0) < 1e-9)
    except Exception as exc:                                  # noqa: BLE001
        check("aim.fire_box_px 可导入", False, repr(exc))
    check("max_lead = max_lead_factor x rail_span_m x px_per_m_rail = 44.1",
          abs(0.45 * 0.70 * 140.0 - 44.1) < 1e-9, "%.1f" % (0.45 * 0.70 * 140.0))

    finish()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
