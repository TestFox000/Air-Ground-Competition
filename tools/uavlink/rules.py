# uavlink.rules —— 由 tools/uav_link.py 拆出（无人机遥测告警规则状态机）。与原文件逐行一致（AST 校验过）。
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
from uavlink.telemetry import _percentile, _wrap_deg

class UavRuleEngine:
    """无人机遥测告警规则状态机（照 track_diag RuleEngine 范式：冷却 + 持续判定）。

    update(t, st)：st 为遥测快照 dict（键都可缺省，缺了对应规则自然不评）：
      failsafe: bool            vehicle_status.failsafe
      estimator_ok: bool        estimator 类 flag 全零=True
      att_deg: (r,p,y)          姿态角（度）
      att_target_deg: (r,p,y)   姿态目标（度，有才评 ATT_DIVERGE）
      rate_dps: (x,y,z)         角速度（度/秒）
      clip_count: int           sensor_combined 加速度计削波累计计数
      battery_remaining: float  0..1
      dzuav_t: float|None       最近一次 data 非空的 dzuav 健康行时刻（本机钟；
                                data=null 的通道行不刷新——否则 dzuav 真挂了
                                DZUAV_SILENT 永不触发，S1 审查修正）
      dzuav_null_since: float|None  dzuav 通道行 data 持续为 null 的起始时刻
    note_sample(t, topic, hz)：每条采样行到达时调用（LINK_LAG 用）。
    命中返回 [{"rule","t","msg"}]。
    """

    COOLDOWN_S = 5.0        # 每条规则命中后的冷却
    EKF_SUSTAIN_S = 2.0     # EKF_DEGRADED 需持续时长
    ATT_DIVERGE_DEG = 15.0  # ATT_DIVERGE 偏差阈值
    ATT_SUSTAIN_S = 1.0
    RATE_SAT_DPS = 300.0    # RATE_SAT 角速度限幅阈值（侦察机持续 300°/s 必异常）
    RATE_SUSTAIN_S = 1.0
    BAT_LOW_FRAC = 0.20     # BAT_LOW 剩余电量阈值
    DZUAV_SILENT_S = 5.0    # DZUAV_SILENT 静默阈值
    LINK_LAG_RATIO = 2.0    # LINK_LAG：采样间隔 p95 > 期望 × 此倍数
    LINK_WINDOW_S = 5.0     # LINK_LAG 观察窗
    LINK_MIN_SAMPLES = 10   # LINK_LAG 至少这么多间隔样本才评
    LINK_FAST_HZ = 5.0      # 只有声明 hz >= 此值的快通道才评 LINK_LAG

    MESSAGES = {
        "FAILSAFE": "飞控 FAILSAFE 触发——立即接管/降落，查触发原因",
        "EKF_DEGRADED": "EKF 估计器 flag 异常——定位/姿态估计退化，查估计器状态",
        "ATT_DIVERGE": "姿态与目标偏差过大——跟踪回路异常，查角速率/振动/调参",
        "RATE_SAT": "角速度接近限幅——可能在甩，查振动/调参/机械",
        "VIBE_CLIP": "加速度计削波计数上升——振动过大，查桨/机架松动/减震",
        "BAT_LOW": "电池剩余 <20%——尽快降落",
        "DZUAV_SILENT": "dzuav 健康 5s 无更新——侦察进程可能挂了",
        "DZUAV_NULL": "dzuav 通道有行但数据持续为空——dzuav 活着但健康不可见",
        "LINK_LAG": "遥测链路滞后——采样间隔 p95 超期望 2 倍，查 Wi-Fi/串口桥",
    }

    def __init__(self) -> None:
        self.expect_dzuav = True
        self.start_t: float | None = None
        self.cond_since: dict[str, float] = {}
        self.last_fired: dict[str, float] = {}
        self.counts: Counter = Counter()
        self._last_clip: int | None = None
        self._link: dict[str, deque] = {}   # topic -> deque[(t, interval, expected)]

    # ---- 内部工具（与 track_diag RuleEngine 同款）----
    def _cooled(self, rule: str, t: float) -> bool:
        return t - self.last_fired.get(rule, -1e18) >= self.COOLDOWN_S

    def _fire(self, rule: str, t: float, extra: str = "") -> dict:
        self.last_fired[rule] = t
        self.counts[rule] += 1
        return {"rule": rule, "t": t, "msg": self.MESSAGES[rule] + extra}

    def _sustained(self, rule: str, t: float, cond: bool, sustain: float) -> bool:
        """条件持续 sustain 秒且过冷却 → 应触发（触发后重新计时）。"""
        if not cond:
            self.cond_since.pop(rule, None)
            return False
        since = self.cond_since.setdefault(rule, t)
        if t - since >= sustain and self._cooled(rule, t):
            self.cond_since[rule] = t
            return True
        return False

    # ---- 链路采样间隔跟踪 ----
    def note_sample(self, t: float, topic: str, hz: float | None) -> None:
        """记录快通道采样间隔（LINK_LAG 原料）。hz 为采样器声明的限速档位。"""
        if not hz or hz < self.LINK_FAST_HZ:
            return
        dq = self._link.setdefault(topic, deque())
        if dq:
            interval = t - dq[-1][0]
            if interval > 0:
                dq.append((t, interval, 1.0 / hz))
        else:
            dq.append((t, 0.0, 1.0 / hz))   # 首样本只占位（interval=0 不参与统计）
        while dq and t - dq[0][0] > self.LINK_WINDOW_S:
            dq.popleft()

    def _link_lag_ratio(self, t: float) -> tuple[str, float] | None:
        """窗口内 p95 间隔 / 期望周期 的最大比值（>1 即滞后）；样本不足 None。"""
        worst = None
        for topic, dq in self._link.items():
            pts = [p for p in dq if p[1] > 0 and t - p[0] <= self.LINK_WINDOW_S + 1e-9]
            if len(pts) < self.LINK_MIN_SAMPLES:
                continue
            ints = sorted(p[1] for p in pts)
            p95 = _percentile(ints, 95)
            expected = pts[-1][2]
            if expected > 0:
                ratio = p95 / expected
                if worst is None or ratio > worst[1]:
                    worst = (topic, ratio)
        return worst

    # ---- 主入口 ----
    def update(self, t: float, st: dict) -> list[dict]:
        alerts: list[dict] = []
        if self.start_t is None:
            self.start_t = t

        # FAILSAFE：failsafe=true 立即报（冷却内不重复）
        if st.get("failsafe") and self._cooled("FAILSAFE", t):
            alerts.append(self._fire("FAILSAFE", t))

        # EKF_DEGRADED：estimator flag 异常持续 2s
        ok = st.get("estimator_ok")
        if self._sustained("EKF_DEGRADED", t, ok is False, self.EKF_SUSTAIN_S):
            alerts.append(self._fire("EKF_DEGRADED", t))

        # ATT_DIVERGE：姿态与目标偏差 >15° 持续 1s（无目标则不评）
        att = st.get("att_deg")
        tgt = st.get("att_target_deg")
        dev = None
        if att is not None and tgt is not None:
            dev = max(abs(_wrap_deg(a - b)) for a, b in zip(att, tgt))
        if self._sustained("ATT_DIVERGE", t,
                           dev is not None and dev > self.ATT_DIVERGE_DEG,
                           self.ATT_SUSTAIN_S):
            alerts.append(self._fire("ATT_DIVERGE", t, f"（偏差 {dev:.1f}°）"))

        # RATE_SAT：|角速度| 超限持续 1s
        rate = st.get("rate_dps")
        peak = max((abs(v) for v in rate), default=None) if rate else None
        if self._sustained("RATE_SAT", t,
                           peak is not None and peak > self.RATE_SAT_DPS,
                           self.RATE_SUSTAIN_S):
            alerts.append(self._fire("RATE_SAT", t, f"（峰值 {peak:.0f}°/s）"))

        # VIBE_CLIP：削波计数上升即报（计数器回落=飞控重启，静默重设基线）
        clip = st.get("clip_count")
        if clip is not None:
            if self._last_clip is not None and clip > self._last_clip \
                    and self._cooled("VIBE_CLIP", t):
                alerts.append(self._fire(
                    "VIBE_CLIP", t, f"（{self._last_clip}→{clip}）"))
            self._last_clip = clip

        # BAT_LOW：剩余 <20%（电平型，冷却内不重复）
        rem = st.get("battery_remaining")
        if rem is not None and rem < self.BAT_LOW_FRAC \
                and self._cooled("BAT_LOW", t):
            alerts.append(self._fire("BAT_LOW", t, f"（剩余 {rem * 100:.0f}%）"))

        # DZUAV_SILENT：健康 5s 无更新（从未见过也算，自 start_t 起计；
        # dzuav_t 只被 data 非空行刷新，data=null 心跳糊弄不了这条规则）
        if self.expect_dzuav:
            last = st.get("dzuav_t")
            last = last if last is not None else self.start_t
            if t - last > self.DZUAV_SILENT_S and self._cooled("DZUAV_SILENT", t):
                alerts.append(self._fire("DZUAV_SILENT", t))

        # DZUAV_NULL：dzuav 通道行在来、但 data 持续 null 超 5s——采样器活着，
        # 健康源（orin_log_capture jsonl / tmux capture-pane）不在场。
        # 取舍（S1 审查）：dzuav 进程本身可能活得好好的，只是健康采集缺席；
        # 宁可误报不可漏报——"健康通道全程空"与"侦察进程死了"在笔记本端
        # 无法区分，此时告警让人去机上看一眼，比静默可信。
        null_since = st.get("dzuav_null_since")
        if self.expect_dzuav and null_since is not None \
                and t - null_since > self.DZUAV_SILENT_S \
                and self._cooled("DZUAV_NULL", t):
            alerts.append(self._fire("DZUAV_NULL", t))

        # LINK_LAG：快通道采样间隔 p95 > 期望 2 倍
        lag = self._link_lag_ratio(t)
        if lag is not None and lag[1] > self.LINK_LAG_RATIO \
                and self._cooled("LINK_LAG", t):
            alerts.append(self._fire(
                "LINK_LAG", t, f"（{lag[0]} p95 间隔为期望 {lag[1]:.1f} 倍）"))
        return alerts
