#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""移动靶速度标定工具 · 稳定版（复杂条件可用）。

基准（2026-10-17 实测）：v≈0.138m/s 匀速，单程 3.40s，端部停顿 ~0.2s，
周期 7.2s；端点 ±5mm 滞留 ~0.4s = 最优开火窗口。

韧性模型（与车位无关，近大远小自适应）：
  v_px / w_px = v_world / W_plate = 常数
  任意车位：预期每秒像素位移 = RATIO × 当前靶面像素宽
  在线反算：v_world = v_px × W_plate / w_px

稳定性设计（每项对应一类现场失效，均有 --self-test 对抗场景自证）：
  [干扰物]  候选轨迹竞标：只有"沿直线持续移动"的色块成为目标
            （静止蓝物/红桶/海报自动出局，S2 场景）
  [光照]    HSV 色域自学习 + 每 5s 从内内容素刷新（曝光泵动/色温漂移，S4）
  [遮挡]    丢靶惯性保持 ≤1s → 沿轨道线受限重捕 → 全局重捕（S3）
  [野值]    alpha-beta 门控关联 + Theil-Sen 稳健速度（S6）
  [低帧率]  <15fps 自动加宽速度估计窗（S5, 10fps）
  [抖动]    帧间高频抖动被 Theil-Sen 中值斜率吸收（S2 内含相机抖动）
  [断流]    打开重试×3 + 2s 停流告警；Ctrl-C/异常仍输出汇总
  [双色面]  红/蓝均为环形色相表示（红色环绕 0° 天然支持）

用法：
  python target_speed_calib.py --source 0                       # 车端/本机摄像头
  python target_speed_calib.py --source http://ip:8080/video    # IP 流
  python target_speed_calib.py --source rec.mp4                 # 离线视频
  python target_speed_calib.py --source 0 --plate-width 0.20    # 已知靶宽→世界速度校验
  python target_speed_calib.py --self-test                      # 六类对抗回归（无需硬件）

  --color blue|red   靶面颜色（默认 blue=敌方面）
  --roi X Y W H      检测子区域    --duration 30   采样时长(s)
  --no-display       无屏模式      --out r.json    结果落盘
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

V_WORLD_REF = 0.138   # m/s 实测基准
PERIOD_REF = 7.2      # s

# 初始 HSV（环形色相）：h_center/h_half + S/V 下限，运行中自学习细化
INIT_RANGE = {
    "blue": (115.0, 22.0, 70, 110),
    "red":  (0.0,   26.0, 80, 110),
}
MAX_H_HALF = 42.0     # 色相半宽上限，防止学飞


# ---------------------------------------------------------------- 工具
def hue_dist(h: np.ndarray, c: float) -> np.ndarray:
    """环形色相距离（0-90）。"""
    d = np.abs(h.astype(np.float32) - c)
    return np.minimum(d, 180.0 - d)


def circular_mean(h: np.ndarray) -> float:
    """环形色相均值（OpenCV 色相 0~180 环绕）。

    倍角公式 atan2(⟨sin2h⟩,⟨cos2h⟩)/2：h 的周期是 180°，直接对 h 取
    sin/cos 在红色档必坏——{170°,10°}（真值 0°）会输出 90°（审查数值实证）。
    先倍角把环绕点拼合，平均后再折回半角。
    """
    a2 = np.deg2rad(h.astype(np.float64)) * 2.0
    return float(np.degrees(np.arctan2(np.sin(a2).mean(),
                                       np.cos(a2).mean())) / 2.0 % 180.0)


def theil_sen(xs: np.ndarray, ys: np.ndarray) -> tuple[float, float]:
    """稳健斜率 + p90 残差（供质量判断）。"""
    n = len(xs)
    if n < 3:
        return 0.0, 1e9
    slopes = []
    for i in range(n):
        for j in range(i + 1, n):
            dx = xs[j] - xs[i]
            if abs(dx) > 1e-4:
                slopes.append((ys[j] - ys[i]) / dx)
    if not slopes:
        return 0.0, 1e9
    k = float(np.median(slopes))
    resid = np.abs(ys - (k * (xs - xs[0]) + ys[0]))
    return k, float(np.percentile(resid, 90))


# ---------------------------------------------------------------- 跟踪器
@dataclass
class Meas:
    t: float
    x: float
    y: float
    extent: float   # 沿运动轴投影宽（近大远小尺度）


@dataclass
class Cand:
    """捕获期候选轨迹（竞标用）。"""
    pts: list = field(default_factory=list)   # (t,x,y)
    last_t: float = -1e9
    miss: int = 0


class Tracker:
    """状态机：ACQUIRING → TRACKING → (COASTING → REACQ) → LOST → ACQUIRING。"""

    def __init__(self, color: str, min_area: int, s_lo: int, v_lo: int, roi=None):
        self.color = color
        self.min_area = min_area
        self.roi = roi
        self.hsv_rng = list(INIT_RANGE[color])          # [h_c, h_half, s_lo, v_lo]
        self.hsv_rng[2] = s_lo
        self.hsv_rng[3] = v_lo
        self.mode = "ACQUIRING"
        self.cands: list[Cand] = []
        # 滤波状态
        self.pos = np.zeros(2)          # 平滑位置
        self.vel = np.zeros(2)          # px/s（仅用于门控预测）
        self.axis = np.array([1.0, 0.0])
        self.axis_locked = False
        self.meas: list[Meas] = []
        self.last_meas_t = None
        self.coast_since = None
        self.last_hsv_learn = -1e9
        self.endpoints: tuple[float, float] | None = None   # (lo, hi) 沿轴投影
        self.proj_hist: list = []                            # 沿轴投影历史（端点估计用）
        self._ref_lo: float | None = None                    # 端点级精化（停顿事件 EMA）
        self._ref_hi: float | None = None
        self._v_typ = 0.0                                    # 典型速度基准（EMA）
        self._bad = 0                                        # 停顿判定宽限计数
        self.dwell_events: list = []                         # (t_mid, level, dur)
        self._dwell: list | None = None                      # [t0, side, [proj...]]
        self.outliers = 0
        self.n_frames = 0
        self.n_hit = 0
        self.fps_est = 30.0
        self._t_prev = None
        self.reacq_pending = None

    # ---------- 检测 ----------
    def _blobs(self, frame_roi: np.ndarray) -> list[tuple[float, float, float, np.ndarray]]:
        """返回 [(cx, cy, extent_along_axis, contour)]。"""
        hsv = cv2.cvtColor(frame_roi, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        m = (hue_dist(h, self.hsv_rng[0]) <= self.hsv_rng[1]) & \
            (s >= self.hsv_rng[2]) & (v >= self.hsv_rng[3])
        mask = (m.astype(np.uint8)) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in cnts:
            if cv2.contourArea(c) < self.min_area:
                continue
            rect = cv2.minAreaRect(c)
            (cx, cy) = rect[0]
            pts = cv2.boxPoints(rect)
            proj = pts @ self.axis[:2]
            out.append((float(cx), float(cy), float(proj.max() - proj.min()), c))
        return out

    def _learn_hsv(self, hsv_roi: np.ndarray, contour: np.ndarray, t: float) -> None:
        """从匹配轮廓内内容素刷新 HSV 范围（EMA 防投毒）。"""
        mask = np.zeros(hsv_roi.shape[:2], np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        px = hsv_roi[mask > 0]
        if len(px) < 60:
            return
        h, s, v = px[:, 0].astype(np.float32), px[:, 1].astype(np.float32), px[:, 2].astype(np.float32)
        h_c_new = circular_mean(h)
        dh = hue_dist(h, self.hsv_rng[0])           # 相对旧中心的偏差（防跳到别色）
        if np.percentile(dh, 90) > 45:
            return
        half_new = float(np.percentile(hue_dist(h, h_c_new), 92)) + 8.0
        half_new = min(half_new, MAX_H_HALF)
        a = 0.4  # EMA
        # 环形均值混合（倍角公式，同 circular_mean：红色横跨 0° 环绕，
        # 直接混合 h 会把 {170°,10°} 拉向 90°）
        rad = math.radians(self.hsv_rng[0]) * 2, math.radians(h_c_new) * 2
        blended = math.degrees(math.atan2(
            (1 - a) * math.sin(rad[0]) + a * math.sin(rad[1]),
            (1 - a) * math.cos(rad[0]) + a * math.cos(rad[1]))) / 2 % 180.0
        self.hsv_rng[0] = blended
        self.hsv_rng[1] = min((1 - a) * self.hsv_rng[1] + a * half_new, MAX_H_HALF)
        self.hsv_rng[2] = int(max(30, (1 - a) * self.hsv_rng[2] + a * max(30, np.percentile(s, 8) - 15)))
        self.hsv_rng[3] = int(max(50, (1 - a) * self.hsv_rng[3] + a * max(50, np.percentile(v, 8) - 20)))
        self.last_hsv_learn = t

    # ---------- 主入口 ----------
    def process(self, frame: np.ndarray, t: float):
        """逐帧处理（frame=整帧，内部再裁 ROI）。返回 (mode, meas_or_None)。"""
        self.n_frames += 1
        if self._t_prev is not None and t > self._t_prev:
            dt = t - self._t_prev
            if 0.001 < dt < 1.0:
                self.fps_est = 0.9 * self.fps_est + 0.1 / dt
        self._t_prev = t

        rx, ry = 0, 0
        img = frame
        if self.roi:
            rx, ry, rw, rh = self.roi
            img = frame[ry:ry + rh, rx:rx + rw]
        hsv_roi = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        blobs = self._blobs(img)

        if self.mode == "ACQUIRING":
            self._acquire(blobs, t, rx, ry)
            return self.mode, None

        # ---------- 门控关联 ----------
        dt = (t - self.last_meas_t) if self.last_meas_t is not None else 0.033
        dt = max(min(dt, 0.5), 1e-3)
        pred = self.pos + self.vel * dt
        gate = max(70.0, 2.5 * np.linalg.norm(self.vel) * dt + 40.0)
        best, best_d = None, gate
        for cx, cy, ext, c in blobs:
            d = math.hypot(cx + rx - pred[0], cy + ry - pred[1])
            if d < best_d:
                best, best_d = (cx + rx, cy + ry, ext, c), d
        if best is not None:
            m = Meas(t, best[0], best[1], best[2])
            self._update(m, t, hsv_roi, best[3])
            return self.mode, m

        # 无匹配 → 惯性保持 / 重捕
        if self.coast_since is None:
            self.coast_since = t
            self.reacq_pending = None
        coasted = t - self.coast_since
        if coasted <= 1.0:
            self.pos = self.pos + self.vel * (1.0 / max(self.fps_est, 5.0))
            self.vel *= 0.98                      # 惯性保持期速度缓衰减
            self.mode = "COASTING"
            return self.mode, None
        # 受限重捕：沿轨道线找
        pick = self._reacquire(blobs, t, rx, ry)
        if pick is not None:
            cx, cy, ext, c = pick
            if self.reacq_pending is None:
                self.reacq_pending = (t, cx, cy, ext, c)
                self.mode = "REACQ"
                return self.mode, None
            t0, *prev = self.reacq_pending
            if math.hypot(cx - prev[0], cy - prev[1]) < 120:
                m = Meas(t, cx, cy, ext)
                self._update(m, t, hsv_roi, c, snap=True)
                self.reacq_pending = None
                return self.mode, m
            self.reacq_pending = (t, cx, cy, ext, c)
            self.mode = "REACQ"
            return self.mode, None
        if coasted > 3.0:
            self.mode = "ACQUIRING"
            self.cands = []
            self.coast_since = None
        self.mode = "COASTING" if coasted <= 3.0 else "LOST"
        return self.mode, None

    # ---------- 捕获期：候选竞标 ----------
    def _acquire(self, blobs, t, rx, ry):
        for cx, cy, ext, _ in blobs:
            x, y = cx + rx, cy + ry
            linked = False
            for cd in self.cands:
                if t - cd.last_t > 0.35:
                    continue
                px, py = cd.pts[-1][1], cd.pts[-1][2]
                if math.hypot(x - px, y - py) < 70:
                    cd.pts.append((t, x, y))
                    cd.last_t = t
                    cd.miss = 0
                    linked = True
                    break
            if not linked:
                self.cands.append(Cand(pts=[(t, x, y)], last_t=t))
        # 清老化候选
        self.cands = [c for c in self.cands if t - c.last_t < 2.0]
        if t < 2.0:
            return
        for cd in self.cands:
            if len(cd.pts) < 25:
                continue
            arr = np.array(cd.pts)
            span_t = arr[-1, 0] - arr[0, 0]
            d = arr[:, 1:3] - arr[:, 1:3].mean(0)
            if d.std() < 1e-6:
                continue
            _, _, vt = np.linalg.svd(d, full_matrices=False)
            ax = vt[0]
            proj = d @ ax
            span_px = proj.max() - proj.min()
            persist = len(cd.pts) / max(span_t * max(self.fps_est, 5.0), 1.0)
            if span_px > 110 and persist > 0.4 and span_t > 1.5:
                # 竞标成功（沿直线持续移动者胜）→ 升格
                self.axis = ax if ax[0] >= 0 else -ax
                self.pos = arr[-1, 1:3].astype(float)
                self.vel = np.zeros(2)
                last = arr[-1]
                self.meas = [Meas(float(last[0]), float(last[1]), float(last[2]), 0.0)]
                self.mode = "TRACKING"
                self.last_meas_t = t
                self.coast_since = None
                return

    # ---------- 受限重捕：投影在轨道带内且横向偏移小 ----------
    def _reacquire(self, blobs, t, rx, ry):
        if not self.meas:
            return None
        base = np.array([self.meas[-1].x, self.meas[-1].y])
        best, best_score = None, 1e18
        for cx, cy, ext, c in blobs:
            p = np.array([cx + rx, cy + ry])
            rel = p - base
            along = rel @ self.axis
            lateral = abs(rel[0] * -self.axis[1] + rel[1] * self.axis[0])
            if lateral > 90:
                continue
            if self.endpoints:
                lo, hi = self.endpoints
                cur = self.meas[-1].x * self.axis[0] + self.meas[-1].y * self.axis[1]
                if not (lo - 60 <= cur + along <= hi + 60):
                    continue
            score = lateral + 0.3 * abs(along)
            if score < best_score:
                best, best_score = (cx + rx, cy + ry, ext, c), score
        return best

    # ---------- 测量更新 ----------
    def _update(self, m: Meas, t: float, hsv_roi, contour, snap: bool = False):
        if snap:
            self.pos = np.array([m.x, m.y])
            self.vel = np.zeros(2)
        else:
            dt = max(t - (self.last_meas_t if self.last_meas_t is not None else t - 0.033), 1e-3)
            pred = self.pos + self.vel * dt
            r = np.array([m.x, m.y]) - pred
            if np.linalg.norm(r) > 260:      # 野值直接拒（理论上门控已挡）
                self.outliers += 1
                return
            h = 0.45
            self.pos = pred + h * r
            self.vel = self.vel + (h / dt) * 0.35 * r
        # 尺寸：中值 + 跳变保护（中值须有效，防止 0 值污染把真实尺寸压死）
        exts = [s.extent for s in self.meas[-5:]]
        med_e = float(np.median(exts)) if exts else 0.0
        if med_e > 1.0 and abs(m.extent - med_e) / med_e > 0.45:
            m.extent = med_e
        self.meas.append(m)
        self.meas = self.meas[-3000:]
        self.n_hit += 1
        self.last_meas_t = t
        self.coast_since = None
        self.mode = "TRACKING"
        # 轴向：未锁定时 PCA 持续估计，行程足够后锁定
        if not self.axis_locked:
            pts = np.array([[s.x, s.y] for s in self.meas[-120:]])
            if len(pts) >= 20:
                d = pts - pts.mean(0)
                if d.std() > 1e-6:
                    _, _, vt = np.linalg.svd(d, full_matrices=False)
                    ax = vt[0]
                    self.axis = ax if ax[0] >= 0 else -ax
                    proj = d @ self.axis
                    if proj.max() - proj.min() > 140 and len(pts) >= 60:
                        self.axis_locked = True
                        self.proj_hist = []   # 轴向锁定，投影历史重置
        # HSV 学习（锁定后每 5s 刷新）
        if self.axis_locked and t - self.last_hsv_learn > 5.0:
            self._learn_hsv(hsv_roi, contour, t)
        # 端点与停顿
        self._dwell_logic(m, t)

    def _dwell_logic(self, m: Meas, t: float):
        """停顿检测：速度门控 + 端区判定。

        不依赖端点精确位置（首程过冲/端点漂移会污染位置门限）：
        Theil-Sen 窗口斜率在端点换向处自然归零 → "慢"判据对抖动、
        过冲、端点级偏移均稳健；端区只用宽松外带，级值由事件 EMA 精化。
        """
        proj = m.x * self.axis[0] + m.y * self.axis[1]
        self.proj_hist.append(proj)
        if len(self.proj_hist) > 2600:
            del self.proj_hist[:len(self.proj_hist) - 2600]
        need = int(3.0 * max(self.fps_est, 5.0))
        if len(self.proj_hist) < need or t < 5.0:
            self._dwell = None
            return
        # 近端速度（0.8s Theil-Sen，中值斜率吸收高频抖动）
        seg = [s for s in self.meas if t - s.t <= 0.8]
        v_now = 0.0
        if len(seg) >= 6:
            xs = np.array([s.t for s in seg])
            ys = np.array([s.x * self.axis[0] + s.y * self.axis[1] for s in seg])
            v_now, _ = theil_sen(xs, ys)
        # 典型速度基准（只统计巡航段，防止端点零速拖低）
        if self._v_typ <= 0:
            if abs(v_now) > 40:
                self._v_typ = abs(v_now)
        elif abs(v_now) > 0.5 * self._v_typ:
            self._v_typ = 0.95 * self._v_typ + 0.05 * abs(v_now)
        if self._v_typ < 40:
            self._dwell = None
            return
        # 端区：原始极值定带宽（宽松），端点级由已检出事件精化
        arr = np.array(self.proj_hist)
        lo_obs, hi_obs = float(np.min(arr)), float(np.max(arr))
        lo_lvl = self._ref_lo if self._ref_lo is not None else lo_obs
        hi_lvl = self._ref_hi if self._ref_hi is not None else hi_obs
        span = max(hi_lvl - lo_lvl, hi_obs - lo_obs, 1.0)
        zone = max(25.0, 0.12 * span)
        self.endpoints = (lo_lvl, hi_lvl)
        in_lo = proj <= lo_lvl + zone
        in_hi = proj >= hi_lvl - zone
        side = "lo" if (in_lo and not in_hi) else ("hi" if in_hi else None)
        slow = abs(v_now) <= max(20.0, 0.22 * self._v_typ)
        ok = side is not None and slow
        if ok:
            if self._dwell is None:
                self._dwell = [t, side, [proj]]
            elif self._dwell[1] == side:
                self._dwell[2].append(proj)
            else:                       # 直接换侧（罕见）：先关旧事件
                self._close_dwell(t)
                self._dwell = [t, side, [proj]]
            self._bad = 0
        else:
            if self._dwell is not None:
                self._bad += 1
                if self._bad >= 3:      # 宽限 3 帧，抗单帧抖动
                    self._close_dwell(t)
            else:
                self._bad = 0

    def _close_dwell(self, t: float):
        if self._dwell is None:
            return
        t0, sd, projs_d = self._dwell
        dur = t - t0
        if 0.10 <= dur <= 1.5:
            lv = float(np.median(projs_d))
            self.dwell_events.append((0.5 * (t0 + t), lv, dur))
            if sd == "lo":
                self._ref_lo = lv if self._ref_lo is None else 0.5 * self._ref_lo + 0.5 * lv
            else:
                self._ref_hi = lv if self._ref_hi is None else 0.5 * self._ref_hi + 0.5 * lv
        self._dwell = None

    # ---------- 速度估计 ----------
    def speed(self, t: float) -> tuple[float, float]:
        """返回 (v_px 沿轴带符号, 置信 0-1)。"""
        if len(self.meas) < 8:
            return 0.0, 0.0
        win = 1.0 if self.fps_est >= 15 else 2.0
        seg = [s for s in self.meas if t - s.t <= win]
        if len(seg) < 6:
            return 0.0, 0.0
        xs = np.array([s.t for s in seg])
        ys = np.array([s.x * self.axis[0] + s.y * self.axis[1] for s in seg])
        k, resid = theil_sen(xs, ys)
        if resid > 12.0 and win == 1.0:      # 窗内含换向/污染 → 缩窗重试
            seg2 = [s for s in seg if t - s.t <= 0.5]
            if len(seg2) >= 6:
                xs = np.array([s.t for s in seg2])
                ys = np.array([s.x * self.axis[0] + s.y * self.axis[1] for s in seg2])
                k, resid = theil_sen(xs, ys)
        age = max(0.0, 1.0 - (t - (self.meas[-1].t)) / 1.5)
        inlier = max(0.0, 1.0 - resid / 15.0)
        conf = round(min(1.0, 0.4 * age + 0.4 * inlier + 0.2 * min(len(self.meas) / 150.0, 1.0)), 2)
        return k, conf


# ---------------------------------------------------------------- 流适配
def open_source(src: str):
    for attempt in range(3):
        if src.startswith(("http://", "https://", "rtsp://")):
            cap = cv2.VideoCapture(src)
        elif src.isdigit():
            cap = cv2.VideoCapture(int(src))
        else:
            cap = cv2.VideoCapture(src)
            if cap.isOpened():
                return cap, True
            print(f"[错误] 无法打开视频文件: {src}（重试 {attempt + 1}/3）")
            time.sleep(1.0)
            continue
        if cap.isOpened():
            return cap, False
        print(f"[错误] 无法打开流: {src}（重试 {attempt + 1}/3）")
        time.sleep(1.0)
    sys.exit(1)


def run_stream(frames_iter, tracker: Tracker, args, live: bool, label: str) -> dict:
    print(f"[标定] 数据源: {label}  靶色: {args.color}  "
          f"靶宽: {args.plate_width or '未提供'}m")
    last_report, t0 = -1.0, time.monotonic()
    stall_warned = False
    try:
        for t, frame in frames_iter:
            if live:
                t = time.monotonic() - t0
            mode, m = tracker.process(frame, t)
            v_px, conf = tracker.speed(t)
            if live and t - (tracker._t_prev if tracker._t_prev else 0) > 2.0 and not stall_warned:
                print("[警告] 视频流疑似停滞 >2s")
                stall_warned = True
            if not args.no_display:
                disp = frame.copy()
                if m is not None:
                    cv2.circle(disp, (int(m.x), int(m.y)), 10, (0, 255, 255), 2)
                    cv2.putText(disp, f"{m.extent:.0f}px {v_px:.0f}px/s", (int(m.x) + 12, int(m.y)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(disp, mode, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow("target_speed_calib (q=quit)", disp)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("[标定] 手动退出。")
                    break
            if t - last_report >= 0.5:
                last_report = t
                size = tracker.meas[-1].extent if tracker.meas else 0.0
                extra = ""
                if args.plate_width and size:
                    vw = abs(v_px) * args.plate_width / size
                    extra = f"  v_world={vw:.3f}m/s"
                print(f"  t={t:6.2f}s  [{mode:9s}] 靶宽={size:6.1f}px  "
                      f"v_px={v_px:7.1f}px/s  conf={conf:.2f}{extra}")
            if args.duration and t >= args.duration:
                break
    except KeyboardInterrupt:
        print("\n[标定] Ctrl-C：输出当前汇总。")
    finally:
        if not args.no_display:
            cv2.destroyAllWindows()
    return summarize(tracker, args)


# ---------------------------------------------------------------- 汇总
def summarize(tracker: Tracker, args) -> dict:
    print("\n" + "=" * 64)
    print("标定汇总（射击位像素速度 + 近大远小韧性模型）")
    print("=" * 64)
    res: dict = {"frames": tracker.n_frames, "hits": tracker.n_hit,
                 "outliers_rejected": tracker.outliers,
                 "hsv_learned": [round(x, 1) if isinstance(x, float) else x for x in tracker.hsv_rng]}
    if len(tracker.meas) < 40:
        print("[不足] 有效跟踪样本太少。排查：靶色档位/--min-sat/--min-val/--roi/光照。")
        return res
    # 稳定运动段（时间基滑窗 ≤1.5s，低帧率自适应）
    vs = []
    for i in range(10, len(tracker.meas), 5):
        t_end = tracker.meas[i].t
        seg = [s for s in tracker.meas[max(0, i - 150):i + 1] if t_end - s.t <= 1.5]
        if len(seg) >= 8:
            xs = np.array([s.t for s in seg])
            ys = np.array([s.x * tracker.axis[0] + s.y * tracker.axis[1] for s in seg])
            k, r = theil_sen(xs, ys)
            if r < 10:
                vs.append(abs(k))
    if len(vs) < 5:
        print("[不足] 未观测到足够长的匀速运动段。")
        return res
    v_med = float(np.median(vs))
    t_vs = tracker.meas[len(tracker.meas) // 3:]
    exts = [s.extent for s in t_vs if s.extent > 1.0]
    if not exts:
        exts = [s.extent for s in tracker.meas if s.extent > 1.0]
    w_med = float(np.median(exts)) if exts else 0.0
    res["v_px_median"] = round(v_med, 1)
    res["target_px_width"] = round(w_med, 1)
    print(f"\n[1] 当前车位实测：每秒像素位移 ≈ {v_med:.0f} px/s；靶面像素宽 ≈ {w_med:.0f}px")
    if w_med > 5:
        ratio = v_med / w_med
        res["ratio_pxps_per_px"] = round(ratio, 4)
        print(f"[2] 不变比值 RATIO ≈ {ratio:.3f} (px/s)/px（与车位/远近无关）")
        print(f"    任意车位预期：px/s = {ratio:.3f} × 当前靶面像素宽")
        for w in (100, 200, 300, 400):
            print(f"      靶面 {w}px → {ratio * w:6.0f} px/s")
    if args and args.plate_width and w_med > 5:
        vw = v_med * args.plate_width / w_med
        dev = (vw - V_WORLD_REF) / V_WORLD_REF * 100
        res["v_world"] = round(vw, 4)
        res["world_check"] = "PASS" if abs(dev) <= 12 else "FAIL"
        print(f"[3] 世界速度反算 {vw:.3f} m/s（基准 {V_WORLD_REF}，偏差 {dev:+.1f}%）"
              f"{'✓' if abs(dev) <= 12 else ' ✗ 检查靶宽/颜色档'}")
    if len(tracker.dwell_events) >= 2:
        dws = sorted(tracker.dwell_events)
        durs = [d[2] for d in dws]
        # 相邻停顿异侧，间隔即单程；漏检时中值吸收离群
        halves = [dws[i][0] - dws[i - 1][0] for i in range(1, len(dws))]
        if halves:
            half = float(np.median(halves))
            res["one_way_s"] = round(half, 2)
            res["period_s"] = round(2 * half, 2)
            res["dwell_s"] = round(float(np.median(durs)), 2)
            print(f"[4] 端部停顿≈{np.median(durs):.2f}s  半周期≈{half:.2f}s  周期≈{2*half:.2f}s"
                  f"（基准 {PERIOD_REF}s）")
    hit_rate = tracker.n_hit / max(tracker.n_frames, 1)
    res["track_hit_rate"] = round(hit_rate, 3)
    print(f"\n[稳定性] 帧命中 {hit_rate*100:.0f}%  拒野值 {tracker.outliers} 次  "
          f"HSV(学) H{tracker.hsv_rng[0]:.0f}±{tracker.hsv_rng[1]:.0f} "
          f"S≥{tracker.hsv_rng[2]} V≥{tracker.hsv_rng[3]}")
    if hit_rate < 0.6:
        print("         ⚠ 命中率偏低：结果仅供参考，优先排查遮挡/光照/干扰物")
    print("\n[5] 火控接入（激光瞬时命中）")
    print("    · 主策略（最稳）：云台预瞄轨道端点（世界坐标已知）→ 靶进端点区连续3帧 → 开火")
    print("      （不依赖速度估计，端点 ±5mm 滞留 ~0.4s/周期）")
    print("    · 备选中段拦截：瞄准 = 当前 + RATIO×w_px×t_loop；v_world 在线反算逐帧自适应")
    if args and getattr(args, "out", ""):
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(res, f, ensure_ascii=False, indent=2)
        print(f"\n[落盘] {args.out}")
    return res


# ---------------------------------------------------------------- 自测
def _triangular(t: float, v: float = 120.0, dwell: float = 0.2,
                x0: float = 140.0, x1: float = 500.0) -> float:
    """匀速三角波位置（含端部停顿）。"""
    travel, one_way = x1 - x0, (x1 - x0) / v + dwell
    period = 2 * one_way
    ph = t % period
    if ph < one_way:
        u = ph - dwell / 2
        prog = 0.0 if u < 0 else min(u * v, travel) if u * v < travel else travel
        if ph > one_way - dwell / 2:
            prog = travel
        return x0 + prog
    u = ph - one_way - dwell / 2
    prog = travel if u < 0 else max(travel - u * v, 0.0) if u * v < travel else 0.0
    if ph > period - dwell / 2:
        prog = 0.0
    return x0 + prog


def _synth_frames(scenario: str, fps: int = 30, dur: float = 24.0):
    """合成对抗场景帧序列 (t, frame)。"""
    rng = np.random.default_rng(7)
    drop_iv, flash_t = [], []
    if scenario == "S3":
        for _ in range(8):
            s = rng.uniform(1, dur - 1)
            drop_iv.append((s, s + rng.uniform(0.15, 0.5)))
    if scenario == "S6":
        flash_t = list(np.arange(2, dur, 2.0))
    shake = np.zeros(2)
    n = int(dur * fps)
    for i in range(n):
        t = i / fps
        W, H = 640, 480
        fr = np.full((H, W, 3), 235, np.uint8)
        if scenario == "S2":
            shake += rng.normal(0, 1.2, 2)
            shake = np.clip(shake, -6, 6)
            cv2.rectangle(fr, (int(50 + shake[0]), int(60 + shake[1])),
                          (int(140 + shake[0]), int(130 + shake[1])), (255, 120, 0), -1)  # 静止大蓝块
            wx, wy = 300 + 40 * math.sin(t * 1.7) + shake[0], 100 + shake[1]
            cv2.rectangle(fr, (int(wx - 15), int(wy - 15)), (int(wx + 15), int(wy + 15)),
                          (255, 160, 40), -1)   # 游走蓝块
            cv2.rectangle(fr, (540, 390), (590, 440), (60, 60, 200), -1)  # 红桶(红面)干扰
        hidden = any(a <= t <= b for a, b in drop_iv)
        if not hidden:
            x = _triangular(t)
            hue = 115 + (15 * t / dur if scenario == "S4" else 0)
            v_val = int(200 - (80 * t / dur if scenario == "S4" else 0))
            col = cv2.cvtColor(np.uint8([[[int(hue) % 180, 190, v_val]]]), cv2.COLOR_HSV2BGR)[0, 0]
            off = shake.astype(int) if scenario == "S2" else (0, 0)
            cv2.rectangle(fr, (int(x - 20 + off[0]), int(240 - 30 + off[1])),
                          (int(x + 20 + off[0]), int(240 + 30 + off[1])), tuple(int(c) for c in col), -1)
        if any(abs(t - ft) < 0.5 / fps for ft in flash_t):
            fx, fy = int(rng.integers(20, 580)), int(rng.integers(20, 440))
            cv2.rectangle(fr, (fx, fy), (fx + 50, fy + 40), (255, 130, 20), -1)  # 单帧野蓝
        if scenario == "S5" and i % 3 != 0:
            continue
        noise = rng.normal(0, 2.0 if scenario == "S5" else 0.5)
        fr = np.clip(fr.astype(np.int16) + int(noise), 0, 255).astype(np.uint8)
        yield i / fps, fr


def self_test() -> int:
    print("=" * 64)
    print("稳定性自测：六类对抗场景（合成，无需硬件）")
    print("=" * 64)
    cases = [
        ("S1 清洁基准",      "S1", 120, 8),
        ("S2 干扰物+相机抖动", "S2", 120, 10),
        ("S3 遮挡丢靶(15%)",  "S3", 120, 10),
        ("S4 光照漂移",       "S4", 120, 10),
        ("S5 低帧率10fps",    "S5", 120, 15),
        ("S6 单帧野值",       "S6", 120, 10),
    ]
    n_pass = 0
    for name, sc, v_true, tol in cases:
        tr = Tracker("blue", min_area=80, s_lo=70, v_lo=90, roi=None)
        class A:  # 汇总参数壳
            plate_width = 0.0
            out = ""
            no_display = True
            color = "blue"
            duration = 0
        frames = _synth_frames(sc)
        for t, fr in frames:
            tr.process(fr, t)
        v_est = 0.0
        vs = []
        # 时间基滑窗（≤1.5s），低帧率自适应
        for i in range(10, len(tr.meas), 5):
            t_end = tr.meas[i].t
            seg = [s for s in tr.meas[max(0, i - 150):i + 1] if t_end - s.t <= 1.5]
            if len(seg) >= 8:
                xs = np.array([s.t for s in seg])
                ys = np.array([s.x * tr.axis[0] + s.y * tr.axis[1] for s in seg])
                k, r = theil_sen(xs, ys)
                if r < 10:
                    vs.append(abs(k))
        ok_v = False
        if vs:
            v_est = float(np.median(vs))
            ok_v = abs(v_est - v_true) <= tol
        half = None
        if len(tr.dwell_events) >= 3:
            # 相邻停顿事件异侧（一上一下），间隔即单程时长；漏检一侧时间隔
            # 翻倍成离群值，中值吸收
            dws = sorted(tr.dwell_events)
            halves = [dws[i][0] - dws[i - 1][0] for i in range(1, len(dws))]
            if halves:
                half = float(np.median(halves))
        ok_p = half is not None and abs(half - 3.2) <= 0.5
        ok = ok_v and ok_p
        n_pass += ok
        v_s = f"{v_est:6.1f}" if vs else "  n/a "
        p_s = f"{half:5.2f}s" if half else "  n/a "
        print(f"  {name:14s}  v_est={v_s}px/s (真值{v_true}, ±{tol})  "
              f"半周期={p_s}(真值3.20s)  停顿事件{len(tr.dwell_events):2d}个  "
              f"{'✓ PASS' if ok else '✗ FAIL'}")
    print(f"\n自测结果: {n_pass}/{len(cases)} 通过" + ("  —— 稳定性验证 OK" if n_pass == len(cases) else "  —— 需排查"))
    return 0 if n_pass == len(cases) else 1


# ---------------------------------------------------------------- 主入口
def main():
    ap = argparse.ArgumentParser(description="移动靶速度标定（稳定版）")
    ap.add_argument("--source", default="", help="摄像头序号/流地址/视频文件")
    ap.add_argument("--self-test", action="store_true", help="六类对抗场景自测")
    ap.add_argument("--color", default="blue", choices=["blue", "red"])
    ap.add_argument("--plate-width", type=float, default=0.0)
    ap.add_argument("--roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"))
    ap.add_argument("--min-area", type=int, default=60)
    ap.add_argument("--min-sat", type=int, default=70)
    ap.add_argument("--min-val", type=int, default=110)
    ap.add_argument("--duration", type=float, default=0)
    ap.add_argument("--no-display", action="store_true")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    if args.self_test:
        sys.exit(self_test())
    if not args.source:
        ap.error("需要 --source 或 --self-test")

    cap, is_file = open_source(args.source)
    vfps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    tracker = Tracker(args.color, args.min_area, args.min_sat, args.min_val, args.roi)

    def gen():
        i = 0
        while True:
            ok, fr = cap.read()
            if not ok:
                return
            yield i / vfps, fr
            i += 1

    try:
        run_stream(gen(), tracker, args, live=not is_file, label=args.source)
    finally:
        cap.release()


if __name__ == "__main__":
    main()
