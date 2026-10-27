# uavlink.report —— 由 tools/uav_link.py 拆出（飞行黑匣子取证与体验报告）。与原文件逐行一致（AST 校验过）。
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import socket
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path
from uavlink.blackbox import read_blackbox
from uavlink.rules import UavRuleEngine
from uavlink.telemetry import TelemetryState, _float_or_none, _percentile, _wrap_deg, classify_topic

# ---------------------------------------------------------------- report 离线复盘

SEVERITY_LABEL = {"critical": "严重", "warn": "注意", "info": "信息"}
_SEVERITY_RANK = {"critical": 0, "warn": 1, "info": 2}
def _row_t(row: dict) -> float | None:
    """黑匣子行的时间：采样器 t 优先，退化本机 recv。

    ⚠️ 必须显式判 None 不能用 or：t=0.0 是合法时刻，or 会把它当空值退化到 recv。
    """
    t = _float_or_none(row.get("t"))
    return t if t is not None else _float_or_none(row.get("recv"))
# ---- flight_bb 格式契约（tools/orin_flight_blackbox.py 只读参考，勿改； ----
# ---- 2026-10-16 已从其源码 _emit/_on_* 回调逐字核实） ----
# 行形：{"ts","mono","seq","node":"flight_bb","ev","lvl","session",<事件字段>}
FLIGHT_BB_NAV_NAMES = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "MISSION", 4: "LOITER",
    5: "RTL", 7: "TAKEOFF", 8: "LAND", 10: "PRECLAND", 11: "ORBIT",
    14: "OFFBOARD", 17: "LANDENGFAIL", 18: "RTGS",
}
FLIGHT_BB_VC_NAMES = {176: "DO_SET_MODE", 21: "NAV_LAND", 400: "ARM_DISARM"}
# KILL 语义（与 flight_console 同源）：cmd=400 param1=0 param2=21196 强制断解锁，
# 它自己的 fmu.cmd 回调把这种行标成 lvl=error
FLIGHT_BB_KILL = (400, 0.0, 21196.0)
FLIGHT_BB_SLAM_GAP_S = 5.0     # armed 期间超过此秒数无 slam.odom = SLAM 断流（EKF 输入中断）
FLIGHT_BB_PAIR_WINDOW_S = 1.0  # 姿态/位置与 fmu.target 配对窗口
def classify_blackbox_row(row: dict) -> str:
    """黑匣子行格式判别：有 ev 且 node=="flight_bb" → flight_bb；有 topic → 现行
    采样格式；其余 → other（走现行坏行容错路径：ctl/hello/alert/summary/垃圾行）。

    ⚠️ node 条件不可省：ag_core.blackbox 的行同样带 ev 键，没有 node 判别会把
    车端黑匣子误当 flight_bb 解析（2026-10-17 审查修正）。
    """
    if "ev" in row and row.get("node") == "flight_bb":
        return "flight_bb"
    if "topic" in row:
        return "sample"
    return "other"
def _pair_fb_att_target(rep: dict, fb: dict, t: float | None) -> None:
    """flight.att ↔ fmu.target 配对（窗口 1s）：yaw 偏差 >15° 时段。

    fmu.target 是轨迹目标（trajectory_setpoint），角度量只有 yaw——姿态-目标
    偏差在 flight_bb 格式里就是"航向跟踪偏差"，阈值与实时规则 ATT_DIVERGE 一致。
    """
    if t is None or fb["att_yaw"] is None or fb["tgt_yaw"] is None:
        return
    if abs(fb["att_t"] - fb["tgt_t"]) > FLIGHT_BB_PAIR_WINDOW_S:
        return
    dev = abs(_wrap_deg(fb["att_yaw"] - fb["tgt_yaw"]))
    seg = fb["div_seg"]
    if dev > UavRuleEngine.ATT_DIVERGE_DEG:
        if seg is None:
            fb["div_seg"] = [t, t, dev]
        else:
            seg[1], seg[2] = t, max(seg[2], dev)
    elif seg is not None:
        rep["att_divergence"].append(tuple(seg))
        fb["div_seg"] = None
def _pair_fb_pos_target(rep: dict, fb: dict, t: float | None) -> None:
    """flight.pos ↔ fmu.target 位置配对（窗口 1s）：三维位置误差入统计。"""
    if t is None or fb["pos"] is None or fb["tgt_pos"] is None:
        return
    if abs(fb["pos_t"] - fb["tgt_pos_t"]) > FLIGHT_BB_PAIR_WINDOW_S:
        return
    rep["pos_err"].append(math.dist(fb["pos"], fb["tgt_pos"]))
def _fb_armed_at(fb: dict, x: float) -> bool:
    """时刻 x 是否处于某段 armed 区间（含尚未闭合的当前区间）。"""
    for s, e in fb["armed_intervals"]:
        if s - 1e-9 <= x <= e + 1e-9:
            return True
    return fb["armed_since"] is not None and x >= fb["armed_since"] - 1e-9
def _fb_close_slam_gap(rep: dict, fb: dict, end_t: float | None) -> None:
    """闭合一段"无 slam.odom"区间：时长 >5s 且跨越 5s 阈值那一刻处于 armed → 记断流。

    闭合时机有三个：下一条 slam.odom 到达、armed→disarm 跳变（断流段以 disarm 为界）、
    文件收尾。判定看的是"阈值跨越点是否 armed"而不是"闭合点是否 armed"——
    否则"armed 中断了 6s 然后 disarm"会被漏报（2026-10-16 冒烟实测抓到）。
    """
    last = fb["last_slam_t"]
    if last is None or end_t is None:
        return
    dur = end_t - last
    if dur > FLIGHT_BB_SLAM_GAP_S \
            and _fb_armed_at(fb, last + FLIGHT_BB_SLAM_GAP_S):
        rep["slam_gaps"].append((last, end_t, dur))
def _feed_flight_bb_row(rep: dict, fb: dict, row: dict, t: float | None) -> None:
    """flight_bb 事件 → 复盘全部分析维度。

    lvl 尊重：error 级一律进 error_rows（[严重] 候选，KILL/低电另有专条）；
    warn 级里 flight.status/fmu.cmd/ctrl.cmd 是"翻转/命令留痕"语义（每次必
    warn），已由 armed 时间线与命令审计覆盖，不重复进 findings；goto.state
    翻转单列计数进 [注意]。
    """
    ev = str(row.get("ev") or "")
    lvl = str(row.get("lvl") or "info")
    label = ev   # error_rows 用的可读标签，fmu.cmd 分支会换成含参数的完整标签

    if ev == "flight.status":
        # on-change + 每秒保底心跳：时间线按变化段（与上一签名比对），心跳自然去重
        arming = row.get("arming")
        nav = row.get("nav")
        fsf = bool(row.get("failsafe"))
        armed = arming == 2                    # ARMING_NAMES：1=STANDBY 2=ARMED
        nav_disp = row.get("nav_name") or FLIGHT_BB_NAV_NAMES.get(nav, nav)
        cur = (armed, nav_disp, fsf)
        if t is not None:
            if fb["sig"] is None or cur[0] != fb["sig"][0]:
                rep["armed_timeline"].append((t, cur[0], cur[1]))
            if fb["sig"] is not None and cur[1] != fb["sig"][1]:
                rep["mode_switches"].append((t, cur[1]))
            if cur[2] and (fb["sig"] is None or not fb["sig"][2]):
                rep["failsafe_events"].append((t, cur[1]))
        if lvl == "warn":                      # 变化行=warn，保底心跳=info
            rep["status_flips"] += 1
        # armed 区间跟踪（SLAM 断流的"阈值跨越点是否 armed"判定用）
        if t is not None and armed != fb["armed"]:
            if armed:
                fb["armed_since"] = t
                # M5：rearm（False→True）时 SLAM 断流计时也归零——注释承诺
                # "rearm 后重新连续 5s"：上一段断流是上一个架次的事，rearm 后
                # 给 SLAM 重新连续 5s 的机会，不把旧静默期带进新架次
                fb["last_slam_t"] = t
            else:
                if fb["armed_since"] is not None:
                    fb["armed_intervals"].append((fb["armed_since"], t))
                    fb["armed_since"] = None
                # 断流段以 disarm 为界闭合；闭合后从 disarm 起算新段（disarmed
                # 期间不算，rearm 后需重新连续 5s）
                _fb_close_slam_gap(rep, fb, t)
                fb["last_slam_t"] = t
        fb["sig"] = cur
        fb["armed"] = armed
    elif ev == "flight.att":
        # roll/pitch/yaw 均为弧度（它的 quat_to_euler 输出）→ 入统计划成度
        roll = _float_or_none(row.get("roll"))
        pitch = _float_or_none(row.get("pitch"))
        yaw = _float_or_none(row.get("yaw"))
        if roll is not None:
            rep["att_abs_roll"].append(abs(math.degrees(roll)))
        if pitch is not None:
            rep["att_abs_pitch"].append(abs(math.degrees(pitch)))
        if yaw is not None:
            fb["att_yaw"] = math.degrees(yaw)
            fb["att_t"] = t
            _pair_fb_att_target(rep, fb, t)
    elif ev == "fmu.target":
        yaw = _float_or_none(row.get("yaw"))
        if yaw is not None:
            fb["tgt_yaw"] = math.degrees(yaw)
            fb["tgt_t"] = t
            _pair_fb_att_target(rep, fb, t)
        pos = [_float_or_none(row.get(k))
               for k in ("target_x", "target_y", "target_z")]
        if all(v is not None for v in pos):
            fb["tgt_pos"] = (pos[0], pos[1], pos[2])
            fb["tgt_pos_t"] = t
            _pair_fb_pos_target(rep, fb, t)
    elif ev == "flight.pos":
        vx, vy, vz = (_float_or_none(row.get(k)) for k in ("vx", "vy", "vz"))
        x, y, z = (_float_or_none(row.get(k)) for k in ("x", "y", "z"))
        if vx is not None and vy is not None and vz is not None:
            rep["speed"].append(math.sqrt(vx * vx + vy * vy + vz * vz))
        if z is not None:
            rep["alt"].append(z)
        if x is not None and y is not None and z is not None:
            fb["pos"] = (x, y, z)
            fb["pos_t"] = t
            _pair_fb_pos_target(rep, fb, t)
    elif ev == "flight.batt":
        rem = _float_or_none(row.get("remaining"))
        vol = _float_or_none(row.get("voltage"))
        b = rep["battery"]
        if rem is not None:
            b["min_remaining"] = rem if b["min_remaining"] is None \
                else min(b["min_remaining"], rem)
        if vol is not None:
            b["min_voltage"] = vol if b["min_voltage"] is None \
                else min(b["min_voltage"], vol)
        if lvl == "error":                     # 它自己判的 remaining<0.15
            rep["batt_error_rows"] += 1
    elif ev == "slam.odom":
        rep["slam_rows"] += 1
        if t is not None:
            # EKF 适配：armed 期间 >5s 无 slam.odom = SLAM 断流（闭合上一段）
            _fb_close_slam_gap(rep, fb, t)
            fb["last_slam_t"] = t
    elif ev == "fmu.cmd":
        command = row.get("command")
        name = row.get("name") or FLIGHT_BB_VC_NAMES.get(command, f"vc{command}")
        p1, p2, p7 = (_float_or_none(row.get(k)) for k in ("p1", "p2", "p7"))
        params = " ".join(f"{k}={v:g}" for k, v in
                          (("p1", p1), ("p2", p2), ("p7", p7)) if v is not None)
        label = f"fmu.cmd {name}" + (f"({params})" if params else "")
        if (command, p1, p2) == FLIGHT_BB_KILL:
            label = "fmu.cmd KILL(400 p1=0 p2=21196)"
            if t is not None:
                rep["kill_events"].append((t, label))
        if t is not None:
            rep["cmd_audit"].append((t, label))
    elif ev == "ctrl.cmd":
        name = row.get("name") or str(row.get("cmd"))
        d = [_float_or_none(row.get(k)) for k in ("dx", "dy", "dz")]
        dtxt = "" if all(v is None for v in d) else \
            "(" + ",".join("--" if v is None else f"{v:g}" for v in d) + ")"
        if t is not None:
            rep["cmd_audit"].append((t, f"ctrl.cmd {name}{dtxt}"))
    elif ev == "goto.state":
        rep["goto_flips"] += 1
        if t is not None:
            rep["cmd_audit"].append(
                (t, f"goto {'ON' if row.get('enabled') else 'OFF'}"))
    elif ev == "bb.hb":
        counts = row.get("counts")
        rep["bb_hb"] = (counts if isinstance(counts, dict) else {},
                        _float_or_none(row.get("up")))
    elif ev == "bb.exit":
        rep["clean_exit"] = True

    if lvl == "error" and t is not None:
        rep["error_rows"].append((t, ev, label))
def build_report(rows: list[dict]) -> dict:
    """黑匣子行 → 复盘数据（纯函数，可测）。

    输入为 read_blackbox 的原始行。**双格式兼容**：按行判别（classify_blackbox_row）——
    有 ev → flight_bb（orin_flight_blackbox 机载黑匣子），有 topic → 本工具采样格式，
    两格式可混合（时间线/统计共用同一套维度）。所有时间线/事件时间统一归零到首时刻。
    """
    rep: dict = {
        "n_rows": len(rows),
        "per_ch": Counter(),
        "formats": Counter(),         # "sample" / "flight_bb" 行数
        "span_s": 0.0,
        "topics_found": [],
        "armed_timeline": [],     # [(t, armed, nav_state)]（含首帧；flight_bb 侧 nav 为名字）
        "mode_switches": [],      # [(t, nav_state)]
        "failsafe_events": [],    # [(t, nav_state)]（False→True 跳变）
        "att_abs_roll": [], "att_abs_pitch": [], "rate_max": [],
        "att_divergence": [],     # [(t_start, t_end, max_deg)]，阈值同 ATT_DIVERGE
        "vibe_clip_total": 0,
        "has_vibe": False,        # 任一格式含振动/imu 通道（flight_bb 永远没有）
        "link_gaps": [],          # [(t_start, t_end, dur)]，>2s 无消息段（全行流）
        "battery": {"min_remaining": None, "min_voltage": None},
        "dzuav": {"rows": 0, "null_rows": 0},
        "ctl_receipts": [],       # [{"action","ok","detail"}]
        "alert_counts": Counter(),
        # ---- flight_bb 专属维度 ----
        "cmd_audit": [],          # [(t, label)] fmu.cmd/ctrl.cmd/goto.state 合并审计
        "kill_events": [],        # [(t, label)] KILL（400 p1=0 p2=21196）
        "goto_flips": 0,
        "status_flips": 0,        # flight.status 变化行数（lvl=warn；心跳不计）
        "error_rows": [],         # [(t, ev, label)] lvl=error 聚合
        "batt_error_rows": 0,     # flight.batt lvl=error 行数（它自判 remaining<15%）
        "slam_rows": 0,
        "slam_gaps": [],          # [(t_start, t_end, dur)] armed 期间 >5s 无 slam.odom
        "bb_hb": None,            # 最后一条 bb.hb（counts 字典, up 秒）
        "clean_exit": False,      # 是否有 bb.exit（无 = 黑匣子被断电/强杀）
        "pos_err": [],            # flight.pos ↔ fmu.target 三维位置误差（米）
        "speed": [],              # flight.pos 速度模长
        "alt": [],                # flight.pos z（NED）
    }
    t0 = t1 = None
    last_status: tuple | None = None        # (armed, nav_state, failsafe)
    last_att = last_tgt = None
    last_att_t = last_tgt_t = None          # 采样格式姿态/目标的各自时刻（新鲜度窗用）
    div_seg = None                          # [t_start, t_end, max_deg]（采样格式）
    last_clip = None
    prev_t = None
    fb = {"sig": None, "armed": False,      # flight_bb 处理状态
          "att_yaw": None, "att_t": None, "tgt_yaw": None, "tgt_t": None,
          "pos": None, "pos_t": None, "tgt_pos": None, "tgt_pos_t": None,
          "last_slam_t": None, "div_seg": None,
          "armed_since": None, "armed_intervals": []}
    for row in rows:
        # ---- 非遥测行（ctl/hello/alert/summary）走现行路径 ----
        if row.get("src") == "ctl" or row.get("action") and "topic" not in row:
            if row.get("src") == "ctl":
                rep["ctl_receipts"].append(
                    {"action": row.get("action"), "ok": bool(row.get("ok")),
                     "detail": str(row.get("detail", ""))[:200]})
            continue
        if "hello" in row:
            tf = row.get("topics_found")
            if isinstance(tf, list):
                rep["topics_found"] = [str(x) for x in tf]
            continue
        if row.get("src") == "alert" and row.get("rule"):
            rep["alert_counts"][str(row["rule"])] += 1
            continue
        if "summary" in row:
            continue
        kind = classify_blackbox_row(row)
        if kind == "sample":
            t = _row_t(row)
        elif kind == "flight_bb":
            t = _float_or_none(row.get("ts"))
        else:
            continue                      # 坏行容错：静默跳过
        rep["formats"][kind] += 1
        if t is not None:
            t0 = t if t0 is None else min(t0, t)
            t1 = t if t1 is None else max(t1, t)
            if prev_t is not None and t - prev_t > 2.0:
                rep["link_gaps"].append((prev_t, t, t - prev_t))
            prev_t = t

        if kind == "flight_bb":
            _feed_flight_bb_row(rep, fb, row, t)
            continue

        # ---- 采样格式行（本工具 run 落盘）----
        topic = str(row["topic"])
        ch = str(row["ch"]) if row.get("ch") else classify_topic(topic) or "?"
        data = row.get("data") if isinstance(row.get("data"), dict) else {}
        rep["per_ch"][ch] += 1

        if ch == "status":
            cur = (bool(data.get("armed")), data.get("nav_state"),
                   bool(data.get("failsafe")))
            if t is not None:
                if last_status is None or cur[0] != last_status[0]:
                    rep["armed_timeline"].append((t, cur[0], cur[1]))
                if last_status is not None and cur[1] != last_status[1]:
                    rep["mode_switches"].append((t, cur[1]))
                if cur[2] and (last_status is None or not last_status[2]):
                    rep["failsafe_events"].append((t, cur[1]))
            last_status = cur
        elif ch == "attitude":
            rpy = TelemetryState._num3(data.get("rpy_deg"))
            if rpy:
                rep["att_abs_roll"].append(abs(rpy[0]))
                rep["att_abs_pitch"].append(abs(rpy[1]))
                last_att, last_att_t = rpy, t
        elif ch == "attitude_target":
            tgt = TelemetryState._num3(data.get("rpy_deg"))
            if tgt:
                last_tgt, last_tgt_t = tgt, t
        elif ch == "rates":
            dps = TelemetryState._num3(data.get("dps"))
            if dps:
                rep["rate_max"].append(max(abs(v) for v in dps))
        elif ch == "imu":
            rep["has_vibe"] = True
            clip = data.get("clip")
            if isinstance(clip, (int, float)):
                if last_clip is not None and clip > last_clip:
                    rep["vibe_clip_total"] += int(clip - last_clip)
                last_clip = int(clip)
        elif ch == "battery":
            rem, vol = _float_or_none(data.get("remaining")), \
                _float_or_none(data.get("voltage"))
            b = rep["battery"]
            if rem is not None:
                b["min_remaining"] = rem if b["min_remaining"] is None \
                    else min(b["min_remaining"], rem)
            if vol is not None:
                b["min_voltage"] = vol if b["min_voltage"] is None \
                    else min(b["min_voltage"], vol)
        elif ch == "dzuav":
            rep["dzuav"]["rows"] += 1
            if not data:
                rep["dzuav"]["null_rows"] += 1

        # 姿态-目标偏差时段（>15°，阈值与实时规则一致）。
        # 1s 新鲜度窗（与 flight_bb 侧 FLIGHT_BB_PAIR_WINDOW_S 一致）：姿态/目标
        # 任一停滞超过 1s 就不许再配对——拿三分钟前的目标比当前姿态是假偏差。
        fresh = (last_att_t is not None and last_tgt_t is not None
                 and abs(t - last_att_t) <= FLIGHT_BB_PAIR_WINDOW_S
                 and abs(t - last_tgt_t) <= FLIGHT_BB_PAIR_WINDOW_S) \
            if t is not None else False
        if t is not None and last_att is not None and last_tgt is not None and fresh:
            dev = max(abs(_wrap_deg(a - b)) for a, b in zip(last_att, last_tgt))
            if dev > UavRuleEngine.ATT_DIVERGE_DEG:
                if div_seg is None:
                    div_seg = [t, t, dev]
                else:
                    div_seg[1], div_seg[2] = t, max(div_seg[2], dev)
            elif div_seg is not None:
                rep["att_divergence"].append(tuple(div_seg))
                div_seg = None
    # 收尾：未闭合的偏差段 / armed 区间 / SLAM 断流尾部（armed 到最后都没恢复）
    if div_seg is not None:
        rep["att_divergence"].append(tuple(div_seg))
    if fb["div_seg"] is not None:
        rep["att_divergence"].append(tuple(fb["div_seg"]))
    if t1 is not None and fb["armed_since"] is not None:
        fb["armed_intervals"].append((fb["armed_since"], t1))
        fb["armed_since"] = None
    if t1 is not None:
        _fb_close_slam_gap(rep, fb, t1)
    if t0 is not None and t1 is not None:
        rep["span_s"] = t1 - t0
    # 时间归零：所有时间线/事件统一减去首时刻，report 全按相对秒展示
    if t0:
        def _shift(v: float) -> float:
            return v - t0
        rep["armed_timeline"] = [(_shift(t), a, n)
                                 for t, a, n in rep["armed_timeline"]]
        rep["mode_switches"] = [(_shift(t), n) for t, n in rep["mode_switches"]]
        rep["failsafe_events"] = [(_shift(t), n)
                                  for t, n in rep["failsafe_events"]]
        rep["att_divergence"] = [(_shift(a), _shift(b), d)
                                 for a, b, d in rep["att_divergence"]]
        rep["link_gaps"] = [(_shift(a), _shift(b), d)
                            for a, b, d in rep["link_gaps"]]
        rep["cmd_audit"] = [(_shift(t), la) for t, la in rep["cmd_audit"]]
        rep["kill_events"] = [(_shift(t), la) for t, la in rep["kill_events"]]
        rep["slam_gaps"] = [(_shift(a), _shift(b), d)
                            for a, b, d in rep["slam_gaps"]]
        rep["error_rows"] = [(_shift(t), e, la) for t, e, la in rep["error_rows"]]
    return rep
def _stat_line(vals) -> str:
    """mean/p95/max 一行（vals 非空）。"""
    s = sorted(vals)
    return (f"mean={sum(s) / len(s):.2f} p95={_percentile(s, 95):.2f} "
            f"max={s[-1]:.2f}")
def build_findings(rep: dict) -> list[dict]:
    """复盘数据 → 分级结论（严重/注意/信息，照 ag_core log_diagnosis 分级）。"""
    fs: list[dict] = []

    def add(sev: str, title: str, detail: str = "") -> None:
        fs.append({"severity": sev, "title": title, "detail": detail})

    n_fs = len(rep["failsafe_events"])
    if n_fs:
        times = "、".join(f"t={t:.1f}s" for t, _ in rep["failsafe_events"][:5])
        add("critical", f"FAILSAFE 事件 {n_fs} 次",
            f"发生点：{times}——查触发前后姿态/电池/链路")
    b = rep["battery"]
    if rep["batt_error_rows"]:
        # flight_bb 自判的 remaining<15%（lvl=error）——与最低电量规则去重，取更严的
        add("critical", f"电池 error 级告警 {rep['batt_error_rows']} 次"
                        f"（remaining<15%）",
            "flight_bb 侧判定的危险电量——飞控不稳可能只是饿的，先换满电电池再排查")
    elif b["min_remaining"] is not None:
        if b["min_remaining"] < 0.10:
            add("critical", f"电池电量极低（最低 {b['min_remaining'] * 100:.0f}%）",
                "飞控不稳可能只是饿的——先换满电电池再排查")
        elif b["min_remaining"] < 0.20:
            add("warn", f"电池电量偏低（最低 {b['min_remaining'] * 100:.0f}%）")
    if rep["kill_events"]:
        times = "、".join(f"t={t:.1f}s" for t, _ in rep["kill_events"][:5])
        add("critical", f"KILL 命令记录 {len(rep['kill_events'])} 次",
            f"{times}——强制断解锁=电机立即停转（param2=21196），"
            f"必须查清是谁发的、为什么")
    # lvl=error 里除 KILL/电池外的其余事件（flight.status 的 error 等）也不许静默
    other_err = [(t, ev, la) for t, ev, la in rep["error_rows"]
                 if ev != "flight.batt" and "KILL" not in la]
    if other_err:
        agg = Counter(ev for _t, ev, _la in other_err)
        add("critical", f"error 级事件 {len(other_err)} 条",
            "、".join(f"{ev}x{n}" for ev, n in agg.items())
            + "——flight_bb 自判严重，逐条对照原始日志")
    if rep["link_gaps"]:
        longest = max(g[2] for g in rep["link_gaps"])
        add("warn", f"链路空洞 {len(rep['link_gaps'])} 段（>2s 无消息，最长 "
                    f"{longest:.1f}s）",
            "空洞内的飞控行为完全不可见——查 Wi-Fi 信号/MicroXRCEAgent 重启")
    if rep["vibe_clip_total"]:
        add("warn", f"加速度计削波累计 +{rep['vibe_clip_total']}",
            "振动已达削波——查桨叶损伤/机架松动/减震座；振动是姿态发散常见根因")
    if rep["att_divergence"]:
        worst = max(s[2] for s in rep["att_divergence"])
        add("warn", f"姿态-目标偏差 >15° 共 {len(rep['att_divergence'])} 段"
                    f"（最大 {worst:.1f}°）",
            "姿态跟踪回路异常——结合角速度峰值与削波看是振动还是调参")
    if rep["rate_max"] and max(rep["rate_max"]) > UavRuleEngine.RATE_SAT_DPS:
        add("warn", f"角速度峰值 {max(rep['rate_max']):.0f}°/s 超 "
                    f"{UavRuleEngine.RATE_SAT_DPS:.0f}°/s",
            "机体在甩——先查振动，再看速率环调参")
    dz = rep["dzuav"]
    if dz["rows"] and dz["null_rows"] == dz["rows"]:
        add("warn", "dzuav 健康通道全程无数据",
            "侦察进程可能没起——tmux 看 dzuav 窗口，或 orin_log_capture 是否在场")
    # ---- flight_bb 专属结论 ----
    if rep["slam_gaps"]:
        longest = max(g[2] for g in rep["slam_gaps"])
        add("warn", f"SLAM 断流 {len(rep['slam_gaps'])} 段（armed 期间 >"
                    f"{FLIGHT_BB_SLAM_GAP_S:.0f}s 无 slam.odom，最长 {longest:.1f}s）",
            "EKF 的 SLAM 输入中断——查 fastlio/livox 窗口；断流段定位只剩飞控内插")
    elif rep["formats"].get("flight_bb") and rep["slam_rows"] == 0:
        add("warn", "全程无 slam.odom",
            "FAST-LIO 没起或 /Odometry 话题名变了——查 fastlio 窗口")
    if rep["goto_flips"]:
        add("warn", f"GOTO 开关翻转 {rep['goto_flips']} 次",
            "人工/地面台介入留痕——对照 armed 时间线看介入时机是否异常")
    if rep["formats"].get("flight_bb") and not rep["clean_exit"]:
        add("warn", "黑匣子未正常收尾（无 bb.exit）",
            "orin_flight_blackbox 可能被断电/强杀——尾部遥测或丢，"
            "命令审计不受影响（关键事件 fsync）")
    if not rep["armed_timeline"]:
        add("info", "全程未见 vehicle_status 采样行",
            "话题名变了或 MicroXRCEAgent 没起——对照 topics_found 检查")
    if not rep["per_ch"] and not rep["formats"].get("flight_bb"):
        add("info", "黑匣子里没有采样行", "可能 run 刚启动就退出，或采样器在报错")
    return sorted(fs, key=lambda f: _SEVERITY_RANK[f["severity"]])
def render_report(rep: dict) -> list[str]:
    """复盘数据 → 打印行（严重/注意/信息 分级输出）。采样/flight_bb 双格式共用。"""
    lines = ["===== 黑匣子复盘 ====="]
    per = ", ".join(f"{k}x{v}" for k, v in sorted(rep["per_ch"].items())) or "无"
    lines.append(f"[信息] 时长 {rep['span_s']:.1f}s，采样行 "
                 f"{sum(rep['per_ch'].values())} / 总行 {rep['n_rows']}（{per}）")
    fmts = rep["formats"]
    if fmts.get("flight_bb"):
        desc = "、".join(f"{k}x{v}" for k, v in sorted(fmts.items()))
        lines.append(f"[信息] 识别格式: {desc}（flight_bb=orin_flight_blackbox）")
    if rep["topics_found"]:
        lines.append(f"[信息] 采样器发现 {len(rep['topics_found'])} 个话题: "
                     + ", ".join(rep["topics_found"][:8])
                     + (" …" if len(rep["topics_found"]) > 8 else ""))
    # armed/模式时间线（合并 fmu.cmd/ctrl.cmd/goto 命令审计，按时间交错展示）
    if rep["armed_timeline"] or rep["cmd_audit"]:
        tl_items = [(t, f"{'ARMED' if a else 'disarmed'}(nav={n})")
                    for t, a, n in rep["armed_timeline"]]
        tl_items += [(t, f"cmd:{label}") for t, label in rep["cmd_audit"]]
        tl_items.sort(key=lambda x: x[0])
        tl = "；".join(f"t={t:.1f}s {txt}" for t, txt in tl_items[:16])
        if len(tl_items) > 16:
            tl += f"；…（共 {len(tl_items)} 条）"
        lines.append(f"[信息] armed 时间线: {tl}")
    if rep["cmd_audit"]:
        kinds = Counter(label.split(" ", 1)[0] for _t, label in rep["cmd_audit"])
        ktxt = " ".join(f"{k}x{v}" for k, v in sorted(kinds.items()))
        lines.append(f"[信息] 命令审计 {len(rep['cmd_audit'])} 条（{ktxt}）")
    if rep["mode_switches"]:
        sw = "；".join(f"t={t:.1f}s nav→{n}" for t, n in rep["mode_switches"][:10])
        lines.append(f"[信息] 模式切换: {sw}")
    # 姿态/角速度统计
    if rep["att_abs_roll"]:
        lines.append(f"[信息] |roll|(°): {_stat_line(rep['att_abs_roll'])}")
        lines.append(f"[信息] |pitch|(°): {_stat_line(rep['att_abs_pitch'])}")
    if rep["rate_max"]:
        lines.append(f"[信息] 角速度峰值(°/s): {_stat_line(rep['rate_max'])}")
    # flight.pos：位置/速度统计
    if rep["speed"]:
        lines.append(f"[信息] 速度(m/s): {_stat_line(rep['speed'])}")
    if rep["alt"]:
        lines.append(f"[信息] 高度 z∈[{min(rep['alt']):.2f}, {max(rep['alt']):.2f}]m"
                     "（NED，负值为上空）")
    if rep["pos_err"]:
        lines.append(f"[信息] 位置-目标误差(m): {_stat_line(rep['pos_err'])}")
    # bb.hb 链路健康摘要（各话题实测 Hz = 累计计数 / up 秒）
    if rep["bb_hb"]:
        counts, up = rep["bb_hb"]
        if up:
            lines.append(
                f"[信息] 链路健康(bb.hb): pos={counts.get('flight.pos', 0) / up:.1f}Hz "
                f"att={counts.get('flight.att', 0) / up:.1f}Hz "
                f"slam={counts.get('slam.odom', 0) / up:.1f}Hz "
                f"fmu.cmd={counts.get('fmu.cmd', 0)} "
                f"ctrl.cmd={counts.get('ctrl.cmd', 0)} up={up:.0f}s")
    # flight_bb 无振动通道：明示而非静默（VIBE_CLIP 维度不适用）
    if fmts.get("flight_bb") and not rep["has_vibe"]:
        lines.append("[信息] 该黑匣子未含振动通道（flight_bb 无 sensor_combined "
                     "事件），VIBE_CLIP 维度不适用")
    if rep["ctl_receipts"]:
        rc = "；".join(f"{r['action']}({'成' if r['ok'] else '败'})"
                       for r in rep["ctl_receipts"][:10])
        lines.append(f"[信息] ctl 回执 {len(rep['ctl_receipts'])} 条: {rc}")
    if rep["alert_counts"]:
        ac = ", ".join(f"{k}x{v}" for k, v in rep["alert_counts"].items())
        lines.append(f"[信息] 实时告警命中: {ac}")
    # 分级结论（严重 → 注意 → 信息）
    findings = build_findings(rep)
    if findings:
        lines.append("---- 诊断结论 ----")
        for f in findings:
            line = f"[{SEVERITY_LABEL[f['severity']]}] {f['title']}"
            if f["detail"]:
                line += f"——{f['detail']}"
            lines.append(line)
    else:
        lines.append("[信息] 未发现异常模式（不代表飞控没问题，只是这份日志里没有）")
    return lines
def cmd_report(args) -> int:
    p = Path(args.blackbox)
    if not p.is_file():
        print(f"[失败] 黑匣子不存在: {p}", file=sys.stderr)
        return 2
    rows = read_blackbox(p)
    if not rows:
        print(f"[失败] 黑匣子为空或无有效行: {p}", file=sys.stderr)
        return 1
    print(f"[黑匣子] {p}")
    rep = build_report(rows)
    for ln in render_report(rep):
        print(ln)
    return 0
