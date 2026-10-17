# -*- coding: utf-8 -*-
"""「信念 vs 真实」小车循迹对照诊断工具。

在笔记本上运行，同时拿两份状态做对照：
  ① 信念：SSH 连小车（香橙派，ROS2 Humble），车上传话采样器实时回报
     小车**自认为**的状态（/odom/localization 位姿、/nav/waypoint 最近路线、
     /pursuitAngle 与 /cmd_vel 最近控制命令）；
  ② 真实：俯拍相机（RealSense 或普通 UVC）用 ArUco 标记解算场上**真实**状态
     （车的真实格位与朝向）。
两边对照 + 规则引擎报警，定位"小车代码问题"（定位漂了 / 执行器没动 /
朝向断言错了——即历史 68.6° 画圆事故那类，见 docs/archive/画圆根因分析-20260930.md）。

用法:
  # 1) 先打印标记（id0 贴车顶、id1~4 贴场角、id5 中心校验可选）
  python tools/track_diag.py print-markers out_markers/

  # 2) 主模式：SSH + 相机全开（密码走环境变量，参照 tools/remote_run.py）
  set AG_CAR_PASS=******
  python tools/track_diag.py run --car-host 10.15.166.188 --show

  # 3) 只相机 / 只 SSH
  python tools/track_diag.py run --no-car --show
  python tools/track_diag.py run --car-host 10.15.166.188 --no-camera

依赖: opencv-python(含 aruco) numpy paramiko（pyrealsense2 可选，有 RealSense 时自动用）
"""
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import re
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path

import numpy as np

try:
    import cv2
    import cv2.aruco as aruco
except ImportError:  # pragma: no cover - 仅缺依赖时触发
    cv2 = None
    aruco = None


# ---------------------------------------------------------------- 场地契约（已核实，勿改）
# 14 列(A~N) x 11 行(1~11)，格边 0.35m。场地米制坐标系与官方 path_executor 一致：
# 原点在 N11 格右下角，x 沿行指向第 1 行、y 沿列指向 A 列；
# x∈[0,3.85]、y∈[0,4.90]；yaw 0=+x 方向、逆时针为正（0x4002/map 同架）。
FIELD_COLS = 14
FIELD_ROWS = 11
CELL_M = 0.35
FIELD_W_M = 3.85   # x 向（沿行，11 格 x 0.35）
FIELD_H_M = 4.90   # y 向（沿列，14 格 x 0.35）

CAR_MARKER_ID = 0                 # 车顶标记（顶边朝车头方向贴）
CORNER_IDS = (1, 2, 3, 4)         # 场角：1=左下 2=右下 3=右上 4=左上（相机视角）
CENTER_MARKER_ID = 5              # 中心校验（可选）

# 默认场角（id:场地米制 x,y），与 --corners 默认串一致
DEFAULT_CORNERS = {1: (0.0, 0.0), 2: (FIELD_W_M, 0.0),
                   3: (FIELD_W_M, FIELD_H_M), 4: (0.0, FIELD_H_M)}

# 首帧成功前在这些字典间自动探测，成功后锁定
ARUCO_CANDIDATES = ("DICT_4X4_50", "DICT_5X5_50", "DICT_5X5_100", "DICT_6X6_250")
PRINT_DICT = "DICT_4X4_50"

# 信念侧契约：/odom/localization 是 block 单位（×0.35 化米）+ 四元数 raw yaw。
# path_executor 内部 +π/2 再 snap 得到控制用 yaw；采样器拿不到 snap 结果，
# 故"裸信念 yaw" = 四元数 yaw + π/2（未 snap），对照以"按最近 reset 复算的信念"为主。
BELIEF_YAW_OFFSET = math.pi / 2
HEADING_TO_YAW = {"x": 0.0, "y": math.pi / 2}

RESET_RE = re.compile(r"^\s*reset:\s*([A-Na-n])(\d{1,2})\s*,\s*([xyXY])\s*$")


# ---------------------------------------------------------------- 纯函数：角度 / 格位

def wrap_angle(a: float) -> float:
    """角度归一化到 (-π, π]。"""
    return (a + math.pi) % (2 * math.pi) - math.pi


def cell_to_field(cell: str) -> tuple[float, float]:
    """格号（如 "N10"）→ 场地米制中心坐标。

    x = (10.5-(行号-1))*0.35，y = (13.5-列序)*0.35（列序 A=0 … N=13）。
    """
    m = re.match(r"^\s*([A-Na-n])(\d{1,2})\s*$", cell)
    if not m:
        raise ValueError(f"非法格号: {cell!r}（应为 A1~N11）")
    col = ord(m.group(1).upper()) - ord("A")
    row = int(m.group(2))
    if not (0 <= col < FIELD_COLS and 1 <= row <= FIELD_ROWS):
        raise ValueError(f"格号超出场地: {cell!r}")
    return (10.5 - (row - 1)) * CELL_M, (13.5 - col) * CELL_M


def field_to_cell(x: float, y: float) -> str:
    """场地米制 → 最近格号（越界截断到场地边缘，仅用于显示）。"""
    row = int(round(10.5 - x / CELL_M)) + 1
    col = int(round(13.5 - y / CELL_M))
    row = min(max(row, 1), FIELD_ROWS)
    col = min(max(col, 0), FIELD_COLS - 1)
    return f"{chr(ord('A') + col)}{row}"


# ---------------------------------------------------------------- 纯函数：角点配置 / 单应

def parse_corners_text(text: str) -> dict[int, tuple[float, float]]:
    """解析 --corners 串：'1:0,0 2:3.85,0 3:3.85,4.9 4:0,4.9'（id:场地米制 x,y）。"""
    out: dict[int, tuple[float, float]] = {}
    for tok in text.split():
        try:
            k, v = tok.split(":", 1)
            xs, ys = v.split(",", 1)
            out[int(k)] = (float(xs), float(ys))
        except ValueError:
            raise ValueError(f"--corners 片段无法解析: {tok!r}（应形如 1:0,0）") from None
    return out


def load_corner_config(path: str | Path) -> dict[int, tuple[float, float]]:
    """从 JSON 读角点：{\"corners\": {\"1\": [x, y], ...}} 或扁平 {\"1\": [x, y], ...}。"""
    obj = json.loads(Path(path).read_text(encoding="utf-8"))
    if isinstance(obj, dict) and "corners" in obj:
        obj = obj["corners"]
    if not isinstance(obj, dict):
        raise ValueError(f"角点配置应为字典: {path}")
    return {int(k): (float(v[0]), float(v[1])) for k, v in obj.items()}


def validate_corners(corners: dict[int, tuple[float, float]],
                     expect_w: float = FIELD_W_M, expect_h: float = FIELD_H_M,
                     tol: float = 0.15, strict: bool = False) -> list[str]:
    """校验四角应近似构成 expect_w x expect_h 的矩形（容差 tol）。

    返回警告列表（空 = 通过）；strict=True 时有警告直接 raise ValueError。
    缺角必报警。用户场地可能不是标准尺寸，调用方一般打印警告后继续。
    """
    warns: list[str] = []
    missing = [i for i in CORNER_IDS if i not in corners]
    if missing:
        warns.append(f"缺场角标记坐标: id {missing}（需要 1~4 齐全才能投影）")
        if strict:
            raise ValueError("；".join(warns))
        return warns
    p = {i: np.array(corners[i], dtype=np.float64) for i in CORNER_IDS}

    def dist(a: int, b: int) -> float:
        return float(np.linalg.norm(p[a] - p[b]))

    s12, s23, s34, s41 = dist(1, 2), dist(2, 3), dist(3, 4), dist(4, 1)
    d13, d24 = dist(1, 3), dist(2, 4)
    checks = [
        (abs(s12 - expect_w) > tol * expect_w, f"边 1-2 长 {s12:.3f}m，期望≈{expect_w:.2f}m"),
        (abs(s34 - expect_w) > tol * expect_w, f"边 3-4 长 {s34:.3f}m，期望≈{expect_w:.2f}m"),
        (abs(s23 - expect_h) > tol * expect_h, f"边 2-3 长 {s23:.3f}m，期望≈{expect_h:.2f}m"),
        (abs(s41 - expect_h) > tol * expect_h, f"边 4-1 长 {s41:.3f}m，期望≈{expect_h:.2f}m"),
        (abs(s12 - s34) > tol * expect_w, f"对边不等：1-2={s12:.3f}m vs 3-4={s34:.3f}m"),
        (abs(s23 - s41) > tol * expect_h, f"对边不等：2-3={s23:.3f}m vs 4-1={s41:.3f}m"),
        (abs(d13 - d24) > tol * max(d13, d24), f"对角线不等：{d13:.3f}m vs {d24:.3f}m（非矩形）"),
    ]
    warns.extend(msg for bad, msg in checks if bad)
    if strict and warns:
        raise ValueError("；".join(warns))
    return warns


def _need_cv2() -> None:
    if cv2 is None:  # pragma: no cover - 仅缺依赖时触发
        raise RuntimeError("缺少 opencv（含 aruco）：python -m pip install opencv-python")


def homography_fit(img_pts, field_pts) -> np.ndarray:
    """由 ≥4 对（图像像素, 场地米制）点拟合单应 H（图像 → 场地米）。"""
    _need_cv2()
    src = np.asarray(img_pts, dtype=np.float64).reshape(-1, 2)
    dst = np.asarray(field_pts, dtype=np.float64).reshape(-1, 2)
    if len(src) < 4:
        raise ValueError("单应拟合至少需要 4 对点")
    # ⚠️ 用 findHomography 而非 getPerspectiveTransform：后者只收 CV_32F，
    # float64 直接断言崩溃（OpenCV 5.0 实测）；findHomography 收 float64 且精度更高。
    H = cv2.findHomography(src, dst)[0]
    if H is None:
        raise ValueError("单应拟合失败（点可能共线）")
    return H


def homography_apply(H: np.ndarray, pt) -> tuple[float, float]:
    """把图像像素点经 H 投影到场地米制坐标。"""
    v = np.array([pt[0], pt[1], 1.0], dtype=np.float64)
    r = H @ v
    if abs(r[2]) < 1e-12:
        raise ValueError("单应投影退化（w≈0）")
    return float(r[0] / r[2]), float(r[1] / r[2])


def corners_homography(dets: dict[int, np.ndarray],
                       corners: dict[int, tuple[float, float]]) -> np.ndarray | None:
    """四角标记齐全时由检测结果拟合 H；不齐返回 None。"""
    if not all(i in dets for i in CORNER_IDS):
        return None
    img = [tuple(dets[i].mean(axis=0)) for i in CORNER_IDS]
    fld = [corners[i] for i in CORNER_IDS]
    return homography_fit(img, fld)


def marker_yaw_from_corners(H: np.ndarray, corners4: np.ndarray) -> float:
    """由标记四角（TL,TR,BR,BL 像素序）求"顶边指向"在场地中的 yaw。

    单应保直线，故把中心点与顶边中点分别投影到场地再取方向即可。
    """
    c4 = np.asarray(corners4, dtype=np.float64).reshape(4, 2)
    c_img = c4.mean(axis=0)
    top_img = (c4[0] + c4[1]) / 2
    cx, cy = homography_apply(H, c_img)
    tx, ty = homography_apply(H, top_img)
    return math.atan2(ty - cy, tx - cx)


def solve_car_pose(H: np.ndarray, car_corners4: np.ndarray,
                   yaw_offset_deg: float = 0.0) -> tuple[float, float, float]:
    """车顶标记 → 场地真值位姿 (x, y, yaw)。yaw_offset_deg 修正贴歪的标记。"""
    c4 = np.asarray(car_corners4, dtype=np.float64).reshape(4, 2)
    x, y = homography_apply(H, c4.mean(axis=0))
    yaw = wrap_angle(marker_yaw_from_corners(H, c4) + math.radians(yaw_offset_deg))
    return x, y, yaw


# ---------------------------------------------------------------- 纯函数：信念复算 / Δ / 采样器行

def parse_reset(route: str | None) -> tuple[str, str] | None:
    """从路线/命令串中解析 reset 命令：'reset:N10,y' → ('N10', 'y')；非 reset 返回 None。"""
    if not route:
        return None
    m = RESET_RE.match(route)
    if not m:
        return None
    return f"{m.group(1).upper()}{int(m.group(2))}", m.group(3).lower()


def heading_to_yaw(heading: str) -> float:
    """reset 断言朝向 → 场地 yaw（x=0，y=π/2）。"""
    return HEADING_TO_YAW[heading.lower()]


def compute_delta(belief: tuple[float, float, float],
                  truth: tuple[float, float, float]) -> dict:
    """Δ = 真值 - 信念。返回米/格/度三种口径。"""
    dx = truth[0] - belief[0]
    dy = truth[1] - belief[1]
    dpos_m = math.hypot(dx, dy)
    return {
        "dx_m": dx, "dy_m": dy,
        "dpos_m": dpos_m,
        "dpos_blocks": dpos_m / CELL_M,
        "dyaw_deg": math.degrees(wrap_angle(truth[2] - belief[2])),
    }


def _num3(v) -> tuple[float, float, float] | None:
    """宽松解析 [x, y, yaw] 三元素数值列表，不合法返回 None。"""
    if isinstance(v, (list, tuple)) and len(v) == 3:
        try:
            return float(v[0]), float(v[1]), float(v[2])
        except (TypeError, ValueError):
            return None
    return None


def _num2(v) -> tuple[float, float] | None:
    if isinstance(v, (list, tuple)) and len(v) == 2:
        try:
            return float(v[0]), float(v[1])
        except (TypeError, ValueError):
            return None
    return None


def parse_sampler_line(line: str) -> dict | None:
    """解析采样器一行 NDJSON；坏行/非 JSON 返回 None（容错跳过）。

    正常返回 {"t", "belief", "route", "pursuit", "cmdvel", "error": None}；
    采样器主动报错时 error 为字符串。
    """
    line = line.strip()
    if not line or not line.startswith("{"):
        return None
    try:
        obj = json.loads(line)
    except json.JSONDecodeError:
        return None
    if not isinstance(obj, dict):
        return None
    if obj.get("error"):
        return {"t": None, "belief": None, "route": None, "pursuit": None,
                "cmdvel": None, "error": str(obj["error"])}
    try:
        t = float(obj["t"]) if obj.get("t") is not None else None
    except (TypeError, ValueError):
        t = None
    route = obj.get("route")
    return {
        "t": t,
        "belief": _num3(obj.get("belief")),
        "route": str(route) if route is not None else None,
        "pursuit": _num2(obj.get("pursuit")),
        "cmdvel": _num2(obj.get("cmdvel")),
        "error": None,
    }


class BeliefTracker:
    """按"最近 reset 复算的信念"修正采样器 raw 信念（契约见文件头）。

    - 裸信念(raw)：block ×0.35 化米，yaw = 四元数 raw yaw + π/2（未 snap）；
    - 复算信念(est)：收到新 reset:<格>,<向> 后，以该格中心为锚，
      位置 = 锚格中心 + 之后 raw 位移增量（×0.35），yaw ≈ 断言向（x=0/y=π/2）。
      车上真正用于控制的是 path_executor 内部 snap 后的 yaw，采样器拿不到，
      故复算 yaw 直接取断言向；对照诊断以 est 为主、raw 备查。
    - est-snap 感知：path_executor 在射击/扫描停站与到点时会中途"锚定"信念位姿，
      裸信念在两次采样间出现瞬时跳变。路线活跃时检测到 >SNAP_JUMP_M 的瞬时跳变
      （两帧间隔 ≤SNAP_MAX_DT_S；间隔更长的位移无法与真实运动区分，按运动处理），
      把 est 锚点同步平移到跳变后的位置（保持 est 连续、防 POS_DRIFT 误报），
      事件记入 snaps 日志，不报警。
    """

    SNAP_JUMP_M = 0.3    # 裸信念两帧间瞬时跳变阈值（米），超过视为车上中途锚定
    SNAP_MAX_DT_S = 0.5  # "瞬时"的判定窗（秒）：10Hz 采样下车速 ≤0.35m/s 不可能
                         # 在此窗内位移 0.3m；间隔更长的大位移按真实运动处理

    def __init__(self) -> None:
        self.route: str | None = None
        self.reset: tuple[str, str] | None = None
        self.anchor_blk: tuple[float, float] | None = None
        self.pending_anchor = False       # reset 已到、raw 首帧未到的等待态
        self.raw_blk: tuple[float, float, float] | None = None
        self.last_t: float | None = None  # 本机收到最近信念帧的时刻
        self.snaps: list[dict] = []       # 中途锚定事件日志 {"t","dx_m","dy_m","dist_m"}

    def update(self, t: float, belief: tuple[float, float, float] | None,
               route: str | None) -> None:
        if route is not None:
            self.route = route
            r = parse_reset(route)
            if r is not None and r != self.reset:
                self.reset = r
                self.pending_anchor = True
                self.anchor_blk = None
        if belief is not None:
            new_raw = (float(belief[0]), float(belief[1]), float(belief[2]))
            # est-snap 检测：路线活跃且已有上一帧时，裸信念瞬时跳变 >0.3m 说明车上
            # path_executor 中途锚定（射击/扫描停站、到点）。reset 后的首帧大跳变
            # 是预期锚定（pending_anchor 等待态），不算 snap；两帧间隔超过
            # SNAP_MAX_DT_S 的大位移无法与真实运动区分，同样按运动处理。
            if self.raw_blk is not None and self.route is not None \
                    and not self.pending_anchor:
                dt = t - self.last_t
                dx_blk = new_raw[0] - self.raw_blk[0]
                dy_blk = new_raw[1] - self.raw_blk[1]
                dist_m = math.hypot(dx_blk * CELL_M, dy_blk * CELL_M)
                if dist_m > self.SNAP_JUMP_M and 0.0 <= dt <= self.SNAP_MAX_DT_S:
                    if self.anchor_blk is not None:
                        # 锚点跟随跳变 → est 复算保持连续，不发散出 POS_DRIFT 误报
                        self.anchor_blk = (self.anchor_blk[0] + dx_blk,
                                           self.anchor_blk[1] + dy_blk)
                    self.snaps.append({"t": t, "dx_m": dx_blk * CELL_M,
                                       "dy_m": dy_blk * CELL_M, "dist_m": dist_m})
            self.raw_blk = new_raw
            self.last_t = t
            if self.pending_anchor:
                self.anchor_blk = (self.raw_blk[0], self.raw_blk[1])
                self.pending_anchor = False

    def snapshot(self) -> dict:
        """当前信念：raw（裸）与 est（按 reset 复算）均为 (x_m, y_m, yaw_rad) 或 None。"""
        raw = est = None
        if self.raw_blk is not None:
            x, y, w = self.raw_blk
            raw = (x * CELL_M, y * CELL_M, wrap_angle(w + BELIEF_YAW_OFFSET))
        if self.reset is not None and self.anchor_blk is not None \
                and self.raw_blk is not None:
            cx, cy = cell_to_field(self.reset[0])
            dx = (self.raw_blk[0] - self.anchor_blk[0]) * CELL_M
            dy = (self.raw_blk[1] - self.anchor_blk[1]) * CELL_M
            est = (cx + dx, cy + dy, heading_to_yaw(self.reset[1]))
        return {"raw": raw, "est": est, "route": self.route, "reset": self.reset,
                "snaps": list(self.snaps)}


# ---------------------------------------------------------------- 规则引擎

class RuleEngine:
    """诊断规则状态机（纯逻辑、可测）。每条规则带冷却，命中返回告警 dict。

    update(t, belief, truth, delta)：belief/truth 为 (x_m, y_m, yaw) 或 None，
    delta 为 compute_delta 结果或 None；返回本次命中的告警列表。
    expect_truth / expect_belief 控制 NO_TRUTH / NO_BELIEF 是否启用
    （--no-camera / --no-car 时不该报"无真值/无信念"）。
    """

    COOLDOWN_S = 5.0       # 每条规则命中后的冷却
    SUSTAIN_S = 2.0        # HEADING_BIAS / POS_DRIFT 需持续时长
    HEADING_DEG = 10.0     # HEADING_BIAS 阈值
    POS_BLOCKS = 0.6       # POS_DRIFT 阈值（格）
    TRUTH_SPEED_MS = 0.05  # ODOM_STALL 判"真值在动"的速度阈值
    STILL_M = 0.01         # "2s 未动"位移阈值
    MOVE_M = 0.05          # ACTUATOR_DEAD 判"信念在变"位移阈值
    WINDOW_S = 2.0         # 停更/未动观察窗
    CIRCLE_DEG = 300.0     # CIRCLING 净转角阈值
    CIRCLE_DISP_M = CELL_M  # CIRCLING 位移上限（1 格）
    NO_SIGNAL_S = 3.0      # NO_TRUTH / NO_BELIEF 静默阈值

    MESSAGES = {
        "HEADING_BIAS": "信念朝向偏差——画圆前兆，查 reset 断言/align",
        "POS_DRIFT": "信念位置漂移——对照真实格位，查定位/里程计",
        "ODOM_STALL": "定位停更——查 serial_reader",
        "ACTUATOR_DEAD": "执行器未动车——查 dzactuator 模式/命令通道",
        "CIRCLING": "画圈！立即停车，查朝向链路（reset 断言/align/标记朝向）",
        "NO_TRUTH": "相机 3s 未见车标记——查标记遮挡/相机画面",
        "NO_BELIEF": "SSH 3s 无信念帧——查采样器/serial_reader 定位链",
    }

    def __init__(self) -> None:
        self.expect_truth = True
        self.expect_belief = True
        self.start_t: float | None = None
        self.last_truth_t: float | None = None
        self.last_belief_t: float | None = None
        self.cond_since: dict[str, float] = {}
        self.last_fired: dict[str, float] = {}
        self.counts: Counter = Counter()
        self.truth_hist: deque = deque()   # (t, x, y, yaw)
        self.belief_hist: deque = deque()  # (t, x, y)

    # ---- 内部工具 ----
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

    @staticmethod
    def _window_span_disp(hist: deque, t: float, window: float):
        """窗口 [t-window, t] 内 (时长, 首末位移)；样本不足返回 (None, None)。"""
        pts = [h for h in hist if h[0] >= t - window - 1e-9]
        if len(pts) < 2 or pts[-1][0] - pts[0][0] < window * 0.8:
            return None, None
        span = pts[-1][0] - pts[0][0]
        disp = math.hypot(pts[-1][1] - pts[0][1], pts[-1][2] - pts[0][2])
        return span, disp

    def _truth_speed(self, t: float):
        """最近 1s 真值平均速度（m/s），样本不足返回 None。"""
        span, disp = self._window_span_disp(self.truth_hist, t, 1.0)
        if span is None or span < 0.5:
            return None
        return disp / span

    # ---- 主入口 ----
    def update(self, t: float, belief, truth, delta) -> list[dict]:
        alerts: list[dict] = []
        if self.start_t is None:
            self.start_t = t
        if belief is not None:
            self.belief_hist.append((t, belief[0], belief[1]))
            self.last_belief_t = t
            while self.belief_hist and t - self.belief_hist[0][0] > 4.0:
                self.belief_hist.popleft()
        if truth is not None:
            self.truth_hist.append((t, truth[0], truth[1], truth[2]))
            self.last_truth_t = t
            while self.truth_hist and t - self.truth_hist[0][0] > 6.0:
                self.truth_hist.popleft()

        # HEADING_BIAS：|Δyaw|>10° 持续 2s
        cond = delta is not None and abs(delta["dyaw_deg"]) > self.HEADING_DEG
        if self._sustained("HEADING_BIAS", t, cond, self.SUSTAIN_S):
            alerts.append(self._fire(
                "HEADING_BIAS", t, f"（Δyaw={delta['dyaw_deg']:+.1f}°）"))

        # POS_DRIFT：|Δpos|>0.6 格 持续 2s
        cond = delta is not None and delta["dpos_blocks"] > self.POS_BLOCKS
        if self._sustained("POS_DRIFT", t, cond, self.SUSTAIN_S):
            alerts.append(self._fire(
                "POS_DRIFT", t, f"（Δpos={delta['dpos_blocks']:.2f}格）"))

        # ODOM_STALL：真值速度>0.05m/s 且信念位置 2s 未变
        _, b_disp = self._window_span_disp(self.belief_hist, t, self.WINDOW_S)
        v_true = self._truth_speed(t)
        cond = (v_true is not None and v_true > self.TRUTH_SPEED_MS
                and b_disp is not None and b_disp < self.STILL_M)
        if cond and self._cooled("ODOM_STALL", t):
            alerts.append(self._fire(
                "ODOM_STALL", t, f"（真值速度 {v_true:.2f}m/s）"))

        # ACTUATOR_DEAD：信念在变（2s 位移>5cm）且真值 2s 不动
        _, t_disp = self._window_span_disp(self.truth_hist, t, self.WINDOW_S)
        cond = (b_disp is not None and b_disp > self.MOVE_M
                and t_disp is not None and t_disp < self.STILL_M)
        if cond and self._cooled("ACTUATOR_DEAD", t):
            alerts.append(self._fire("ACTUATOR_DEAD", t))

        # CIRCLING：观察窗内净转角 |ΣΔθ_true|>300° 且位移<1 格
        if len(self.truth_hist) >= 3:
            pts = list(self.truth_hist)
            acc = sum(wrap_angle(pts[i + 1][3] - pts[i][3])
                      for i in range(len(pts) - 1))
            disp = math.hypot(pts[-1][1] - pts[0][1], pts[-1][2] - pts[0][2])
            if abs(math.degrees(acc)) > self.CIRCLE_DEG \
                    and disp < self.CIRCLE_DISP_M and self._cooled("CIRCLING", t):
                alerts.append(self._fire(
                    "CIRCLING", t, f"（净转角 {math.degrees(acc):+.0f}°）"))

        # NO_TRUTH / NO_BELIEF：静默 3s（从未见过也算，自 start_t 起计）
        if self.expect_truth:
            last = self.last_truth_t if self.last_truth_t is not None else self.start_t
            if t - last > self.NO_SIGNAL_S and self._cooled("NO_TRUTH", t):
                alerts.append(self._fire("NO_TRUTH", t))
        if self.expect_belief:
            last = self.last_belief_t if self.last_belief_t is not None else self.start_t
            if t - last > self.NO_SIGNAL_S and self._cooled("NO_BELIEF", t):
                alerts.append(self._fire("NO_BELIEF", t))
        return alerts


# ---------------------------------------------------------------- 车端采样器（内嵌源码，SFTP 上传）

SAMPLER_SRC = r'''#!/usr/bin/env python3
"""track_diag 车端采样器（由 tools/track_diag.py 自动上传到 /tmp，勿手改）。

订阅 /odom/localization（信念位姿，block 单位）、/nav/waypoint（最近路线/reset）、
/pursuitAngle 与 /cmd_vel（最近控制命令，BEST_EFFORT），
每 0.1s 向 stdout 打一行 NDJSON 供笔记本端对照。
"""
import json
import math
import sys
import time


def main():
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
    except Exception as e:  # noqa: BLE001 - 车上环境缺啥都原样报回笔记本
        print(json.dumps({"error": "rclpy 导入失败: %s: %s" % (type(e).__name__, e)}),
              flush=True)
        sys.exit(3)

    state = {"belief": None, "route": None, "pursuit": None, "cmdvel": None}

    def yaw_of(q):
        # 四元数 → 平面 raw yaw（未 +pi/2 未 snap，复算在笔记本端做）
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    class Sampler(Node):
        def __init__(self):
            super().__init__("track_diag_sampler")
            # pursuit/cmd_vel 是 BEST_EFFORT 发布，订阅端必须同档否则收不到
            # ⚠️ reliability 是 QoSProfile 构造参数不是方法：当方法调上车即崩
            be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(
                Odometry, "/odom/localization",
                lambda m: state.__setitem__("belief", [
                    m.pose.pose.position.x, m.pose.pose.position.y,
                    yaw_of(m.pose.pose.orientation)]), 10)
            self.create_subscription(
                String, "/nav/waypoint",
                lambda m: state.__setitem__("route", m.data), 10)
            self.create_subscription(
                Twist, "/pursuitAngle",
                lambda m: state.__setitem__("pursuit", [m.linear.x, m.angular.z]), be)
            self.create_subscription(
                Twist, "/cmd_vel",
                lambda m: state.__setitem__("cmdvel", [m.linear.x, m.angular.z]), be)
            self.create_timer(0.1, self.tick)

        def tick(self):
            print(json.dumps({
                "t": time.time(), "belief": state["belief"],
                "route": state["route"], "pursuit": state["pursuit"],
                "cmdvel": state["cmdvel"]}), flush=True)

    rclpy.init()
    node = Sampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
'''

SAMPLER_REMOTE = "/tmp/track_diag_sampler.py"


def sampler_launch_cmd() -> str:
    """车端采样器启动命令（bash -lc 非交互 shell，不读 ~/.bashrc）。

    先 source ~/dzrobot/dzros.env（车上 ROS_DOMAIN_ID=118 等域配置，与车上
    start_all.sh 同款顺序，文件不存在时静默跳过），再 source 两个 setup.bash；
    漏掉 dzros.env 会落在默认域 0，永远看不到车上 118 域的节点。
    """
    return ("bash -lc '[ -f ~/dzrobot/dzros.env ] && source ~/dzrobot/dzros.env || true; "
            "source /opt/ros/humble/setup.bash && "
            "source ~/dzrobot/install/setup.bash && "
            f"setsid python3 -u {SAMPLER_REMOTE}'")


def read_password(args) -> str:
    """密码优先级：--password > AG_CAR_PASS 环境变量 > 交互提问。

    ⚠️ 与 tools/remote_run.py L39-55 同一坑：没有交互终端时 getpass 会
    **静默死等**（看着像"卡死"），这里直接报错并说明该给什么。
    """
    pw = getattr(args, "password", None) or os.environ.get("AG_CAR_PASS")
    if pw:
        return pw
    import getpass
    if not sys.stdin.isatty():
        raise SystemExit(
            "缺少车端密码（当前没有交互终端，无法弹出密码提问）：\n"
            "  ① 设环境变量：set AG_CAR_PASS=<密码>\n"
            "  ② 或命令行传入：--password <密码>"
        )
    return getpass.getpass(f"{args.car_user}@{args.car_host} 的密码: ")


class CarLink(threading.Thread):
    """SSH 信念链路：上传采样器 → setsid 流式跑 → NDJSON 入队。

    远程用 setsid 自成进程组，stop() 时 pkill 整组回收（docs 里有孤儿进程教训）。
    启动命令见 sampler_launch_cmd()：非交互 shell 不 source ~/.bashrc，dzros.env
    （车上域配置）与两个 setup.bash 都必须显式 source，否则跨不到车上 ROS 域。
    """

    def __init__(self, host: str, user: str, password: str, port: int = 22) -> None:
        super().__init__(daemon=True, name="track-diag-carlink")
        self.host, self.user, self.password, self.port = host, user, password, port
        self.queue: queue.Queue = queue.Queue()
        self.stop_evt = threading.Event()
        self.error: str | None = None
        self._cli = None

    def run(self) -> None:  # pragma: no cover - 需要真实小车，本地不测
        try:
            import paramiko
        except ImportError:
            self.error = "缺少 paramiko，请先执行: python -m pip install paramiko"
            return
        try:
            cli = paramiko.SSHClient()
            cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            cli.connect(hostname=self.host, port=self.port, username=self.user,
                        password=self.password, timeout=30, banner_timeout=30,
                        auth_timeout=30)
            self._cli = cli
            sftp = cli.open_sftp()
            with sftp.open(SAMPLER_REMOTE, "w") as f:
                f.write(SAMPLER_SRC)
            sftp.close()
            cmd = sampler_launch_cmd()
            _, stdout, stderr = cli.exec_command(cmd, get_pty=False)
            for line in iter(stdout.readline, ""):
                if self.stop_evt.is_set():
                    break
                obj = parse_sampler_line(line)
                if obj is not None:
                    self.queue.put((time.time(), obj))
            if not self.stop_evt.is_set():
                code = stdout.channel.recv_exit_status()
                err = stderr.read().decode("utf-8", errors="replace")[:500]
                self.error = f"采样器退出（码 {code}）: {err.strip()}"
        except Exception as e:  # noqa: BLE001 - 网络层异常原样上报
            if not self.stop_evt.is_set():
                self.error = f"{type(e).__name__}: {e}"

    def stop(self) -> None:  # pragma: no cover - 需要真实小车，本地不测
        self.stop_evt.set()
        try:
            if self._cli is not None:
                # setsid 让采样器自成进程组，pkill 整组回收，不留孤儿
                self._cli.exec_command(f"pkill -f {SAMPLER_REMOTE}", timeout=5)
        except Exception:  # noqa: BLE001
            pass
        try:
            if self._cli is not None:
                self._cli.close()
        except Exception:  # noqa: BLE001
            pass


# ---------------------------------------------------------------- 相机

class UvcCam:
    """普通 UVC 相机 / 网络视频流源（cv2.VideoCapture，索引或 URL 均可）。"""

    def __init__(self, index: "int | str") -> None:
        _need_cv2()
        # HTTP/网络流必须显式指定 FFMPEG 后端，否则 OpenCV 自动选后端时
        # 可能选到 MSMF 等不支持 HTTP 的后端，表现为 isOpened() 真但 read() 卡死超时
        if isinstance(index, str) and index.startswith("http"):
            # DroidCam/IP Webcam 等一次只允许一个客户端，若上一个连接未正常释放
            # 会返回 busy 页面导致 isOpened() 真但 read() 拿不到帧。打开前先 /override
            # 抢占（失败也不阻断，多数情况直接连上即可）。
            try:
                from urllib.parse import urlparse
                parsed = urlparse(index)
                override_url = f"{parsed.scheme}://{parsed.netloc}/override"
                import urllib.request
                urllib.request.urlopen(override_url, timeout=3).read()
            except Exception:  # noqa: BLE001 - 无 /override 端点或已空闲均忽略
                pass
            self.cap = cv2.VideoCapture(index, cv2.CAP_FFMPEG)
        else:
            self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"打不开 UVC 相机 index={index}")

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def release(self) -> None:
        # 相机中途被拔掉时 release 也可能抛错，退出回收阶段一律降级为警告
        try:
            self.cap.release()
        except Exception as e:  # noqa: BLE001
            print(f"[相机] UVC 回收异常（已忽略）: {type(e).__name__}: {e}")


class RealSenseCam:
    """RealSense 景深相机，只取彩色流（平面单应足够，深度留扩展口）。"""

    def __init__(self) -> None:
        import pyrealsense2 as rs  # 守护式 import：没装/没接设备时抛给上层回退
        self._rs = rs
        self._started = False  # start 成功前 stop 会直接抛错，必须先落状态
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipe.start(cfg)
        self._started = True

    def read(self):
        frames = self.pipe.wait_for_frames(5000)
        color = frames.get_color_frame()
        if not color:
            return None
        return np.asanyarray(color.get_data())

    def release(self) -> None:
        # 只在成功 start 过才 stop（否则抛 "pipe.stop() cannot be called before
        # start()"）；相机已断开时 stop 同样会抛，回收阶段一律降级为警告
        if not getattr(self, "_started", False):
            return
        try:
            self.pipe.stop()
        except Exception as e:  # noqa: BLE001
            print(f"[相机] RealSense 回收异常（已忽略）: {type(e).__name__}: {e}")
        finally:
            self._started = False


def open_camera(args):
    """按参数开相机：--camera 指定 UVC；--camera-url 指定网络流（如手机 IP Webcam）；
    --realsense 强制 RealSense；默认自动（先 RealSense，失败回退 UVC 0）。
    打不开返回 None。"""
    if args.camera is not None:
        return UvcCam(args.camera)
    if args.camera_url:
        return UvcCam(args.camera_url)
    if args.realsense:
        return RealSenseCam()
    try:
        return RealSenseCam()
    except Exception as e:  # noqa: BLE001 - 任何失败都回退 UVC
        print(f"[相机] RealSense 不可用（{type(e).__name__}），回退 UVC 0")
        try:
            return UvcCam(0)
        except Exception as e2:  # noqa: BLE001
            print(f"[相机] UVC 0 也打不开（{e2}），进入无相机模式")
            return None


_DETECTORS: dict = {}


def detect_with_autodict(frame, locked: str | None = None):
    """检测 ArUco 标记；未锁定时在候选字典间自动探测，首个有命中的锁定。

    返回 ({id: 4x2 角点}, 锁定的字典名或 None)。
    """
    _need_cv2()
    names = (locked,) if locked else ARUCO_CANDIDATES
    for name in names:
        if name not in _DETECTORS:
            d = aruco.getPredefinedDictionary(getattr(aruco, name))
            _DETECTORS[name] = aruco.ArucoDetector(d, aruco.DetectorParameters())
        corners, ids, _ = _DETECTORS[name].detectMarkers(frame)
        if ids is not None and len(ids):
            dets = {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)}
            return dets, name
    return {}, locked


# ---------------------------------------------------------------- print-markers

MARKER_SPECS = [
    (CAR_MARKER_ID, "car_roof", "车顶标记：打印约 10cm，marker 顶边朝车头方向贴"),
    (1, "corner1_bottom_left", "场角 1：相机视角左下，标记中心对准场角"),
    (2, "corner2_bottom_right", "场角 2：相机视角右下，标记中心对准场角"),
    (3, "corner3_top_right", "场角 3：相机视角右上，标记中心对准场角"),
    (4, "corner4_top_left", "场角 4：相机视角左上，标记中心对准场角"),
    (CENTER_MARKER_ID, "center_check", "中心校验（可选）：贴场地正中心，用于投影误差自检"),
]

PLACEMENT_GUIDE = """\
摆放指南：
  · id0 贴车顶，**marker 顶边（打印品的上边）朝车头方向**；贴歪了用
    --car-marker-yaw-offset 修正（度，逆时针为正）。
  · id1~4 贴场地四角，按相机画面视角：1=左下、2=右下、3=右上、4=左上；
    标记中心对准场角，建议打印 14cm 以上。
  · 若场角不在标准位置（非 3.85x4.9 场地），用
    --corners "1:x,y 2:x,y 3:x,y 4:x,y"（场地米制）或 --corner-config 指定实际坐标。
  · id5 可选，贴场地正中心，出现时工具会打印投影误差（厘米级自检）。
  · 打印后核对：安静区（白边）完整、无反光覆膜、贴平不起翘。"""


def cmd_print_markers(args) -> int:
    _need_cv2()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    d = aruco.getPredefinedDictionary(getattr(aruco, PRINT_DICT))
    border = max(args.size // 4, 10)  # 安静区：检测硬性要求，至少 1 格宽
    for mid, name, desc in MARKER_SPECS:
        img = aruco.generateImageMarker(d, mid, args.size)
        img = cv2.copyMakeBorder(img, border, border, border, border,
                                 cv2.BORDER_CONSTANT, value=255)
        fn = out / f"marker_id{mid}_{name}.png"
        if not cv2.imwrite(str(fn), img):
            print(f"[失败] 写入 {fn}", file=sys.stderr)
            return 1
        print(f"[已生成] {fn}（{desc}）")
    print()
    print(PLACEMENT_GUIDE)
    return 0


# ---------------------------------------------------------------- run 主模式

def fmt_pose(p) -> str:
    if p is None:
        return "  --无--  "
    return f"({p[0]:.2f},{p[1]:.2f})m {field_to_cell(p[0], p[1]):>4s} yaw {math.degrees(p[2]):+7.1f}°"


def draw_overlay(frame, H, belief, truth, alerts) -> None:
    """--show 画面：场地网格（H 反投影）+ 信念箭头(蓝) vs 真值箭头(绿) + 告警。"""
    Hinv = np.linalg.inv(H)

    def to_img(x, y):
        v = Hinv @ np.array([x, y, 1.0])
        return int(v[0] / v[2]), int(v[1] / v[2])

    for i in range(FIELD_ROWS + 1):  # x 向网格线（沿行）
        x = i * CELL_M
        cv2.line(frame, to_img(x, 0), to_img(x, FIELD_H_M), (90, 90, 90), 1)
    for j in range(FIELD_COLS + 1):  # y 向网格线（沿列）
        y = j * CELL_M
        cv2.line(frame, to_img(0, y), to_img(FIELD_W_M, y), (90, 90, 90), 1)

    def arrow(pose, color):
        x, y, yaw = pose
        cv2.arrowedLine(frame, to_img(x, y),
                        to_img(x + 0.3 * math.cos(yaw), y + 0.3 * math.sin(yaw)),
                        color, 3, tipLength=0.3)

    if belief is not None:
        arrow(belief, (255, 80, 0))    # 信念：蓝
    if truth is not None:
        arrow(truth, (0, 200, 0))      # 真值：绿
    for i, a in enumerate(alerts[-4:]):
        cv2.putText(frame, f"[{a['rule']}] {a['msg']}", (10, 30 + 28 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)


def _summary_lines(dpos: list, dyaw: list, counts: Counter) -> list[str]:
    lines = ["==== 收尾统计 ===="]
    if dpos:
        a, b = np.asarray(dpos), np.asarray(dyaw)
        lines.append(f"配对样本 {len(a)} 条")
        lines.append(f"Δpos(m): mean={a.mean():.3f} p95={np.percentile(a, 95):.3f} max={a.max():.3f}")
        lines.append(f"Δyaw(°): mean={b.mean():.2f} p95={np.percentile(b, 95):.2f} max={b.max():.2f}")
    else:
        lines.append("无有效配对样本")
    lines.append("告警命中: " + (", ".join(f"{k}x{v}" for k, v in counts.items())
                                 if counts else "无"))
    return lines


def cmd_run(args) -> int:
    _need_cv2()
    # ---- 场角配置：--corners / --corner-config 都没有则"未标定"模式（只检测不投影）
    corners = None
    if args.corner_config:
        corners = load_corner_config(args.corner_config)
    elif args.corners:
        corners = parse_corners_text(args.corners)
    if corners is not None:
        for w in validate_corners(corners):
            print(f"[警告] 场角配置可疑：{w}（仍继续，确认场地实际尺寸）")
    else:
        print("[相机] 未给场角坐标，进入未标定模式：只检测标记，不投影场地坐标")

    cam = None if args.no_camera else open_camera(args)

    car = None
    if args.car_host and not args.no_car:
        car = CarLink(args.car_host, args.car_user, read_password(args), args.car_port)
        car.start()
        print(f"[车端] 已发起 SSH 采样 {args.car_user}@{args.car_host} …")
    elif args.no_car:
        print("[车端] --no-car：不连车，只跑相机侧")
    else:
        print("[车端] 未给 --car-host，只跑相机侧（--car-host 给了才连车）")

    out_path = Path(args.out) if args.out else \
        Path("logs") / "track_diag" / (time.strftime("%Y%m%d_%H%M%S") + ".jsonl")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    tracker = BeliefTracker()
    engine = RuleEngine()
    engine.expect_truth = cam is not None
    engine.expect_belief = car is not None

    locked_dict = None
    H = None
    alerts: list[dict] = []
    dpos_stat: list[float] = []
    dyaw_stat: list[float] = []
    last_print = 0.0
    sampler_err_shown = False
    snap_shown = 0                  # 已提示过的中途锚定事件数（tracker.snaps 前缀）
    center_hint_last = 0.0
    last_truth = None           # (t, x, y, yaw)
    last_pursuit = last_cmdvel = None

    print(f"[日志] {out_path}")
    if args.car_marker_id != CAR_MARKER_ID:
        print(f"[配置] 车顶标记使用 ID {args.car_marker_id}（默认 {CAR_MARKER_ID}），"
              f"中心校验已让位关闭")
    try:
        with open(out_path, "w", encoding="utf-8", newline="\n") as log:
            while True:
                t = time.time()
                truth = None
                truth_px = None

                # ---- 相机侧：检测 → H → 真值位姿
                if cam is not None:
                    frame = cam.read()
                    if frame is None:
                        time.sleep(0.02)
                        continue
                    dets, locked_dict = detect_with_autodict(frame, locked_dict)
                    if corners is not None:
                        Hnew = corners_homography(dets, corners)
                        if Hnew is not None:
                            H = Hnew
                    if args.car_marker_id in dets:
                        truth_px = tuple(dets[args.car_marker_id].mean(axis=0))
                        if H is not None:
                            x, y, yaw = solve_car_pose(
                                H, dets[args.car_marker_id], args.car_marker_yaw_offset)
                            truth = (x, y, yaw)
                            last_truth = (t, x, y, yaw)
                    # 车顶标记顶替 id5 时，id5 已不再是中心校验点，跳过避免误报
                    if CENTER_MARKER_ID != args.car_marker_id \
                            and CENTER_MARKER_ID in dets and H is not None \
                            and t - center_hint_last > 5.0:
                        cx, cy = homography_apply(
                            H, dets[CENTER_MARKER_ID].mean(axis=0))
                        err = math.hypot(cx - FIELD_W_M / 2, cy - FIELD_H_M / 2)
                        center_hint_last = t
                        if err > 0.02:
                            print(f"[相机] 中心标记投影误差 {err * 100:.1f}cm，"
                                  f"检查场角标记是否贴准")
                    if args.show:
                        if H is not None:
                            belief_show = tracker.snapshot()["est"] or \
                                tracker.snapshot()["raw"]
                            draw_overlay(frame, H, belief_show, truth, alerts)
                        cv2.imshow("track_diag", frame)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            break
                else:
                    time.sleep(0.05)

                # ---- 车侧：排空队列，最新一帧进 tracker
                if car is not None:
                    latest = None
                    while True:
                        try:
                            latest = car.queue.get_nowait()
                        except queue.Empty:
                            break
                    if latest is not None:
                        t_recv, obj = latest
                        if obj["error"]:
                            if not sampler_err_shown:
                                sampler_err_shown = True
                                print(f"[车端] 采样器报错：{obj['error']}")
                        else:
                            tracker.update(t_recv, obj["belief"], obj["route"])
                            last_pursuit = obj["pursuit"] or last_pursuit
                            last_cmdvel = obj["cmdvel"] or last_cmdvel
                    if car.error and not sampler_err_shown:
                        sampler_err_shown = True
                        print(f"[车端] 链路异常：{car.error}")

                snap = tracker.snapshot()
                belief = snap["est"] or snap["raw"]

                # path_executor 中途锚定（射击/扫描停站、到点）造成的信念快照跳变：
                # est 锚点已在 BeliefTracker 内跟随保持连续，这里只提示不报警
                for ev in tracker.snaps[snap_shown:]:
                    print(f"[车端] 信念快照跳变 {ev['dist_m']:.2f}m"
                          f"——path_executor 中途锚定，est 锚点已跟随（不报警）")
                snap_shown = len(tracker.snaps)

                # ---- 配对（1s 时间窗内最新信念/真值）→ Δ → 规则引擎
                truth_fresh = None
                if last_truth is not None and t - last_truth[0] <= 1.0:
                    truth_fresh = last_truth[1:]
                belief_fresh = None
                if belief is not None and tracker.last_t is not None \
                        and t - tracker.last_t <= 1.0:
                    belief_fresh = belief
                delta = None
                if belief_fresh is not None and truth_fresh is not None:
                    delta = compute_delta(belief_fresh, truth_fresh)
                    dpos_stat.append(delta["dpos_m"])
                    dyaw_stat.append(abs(delta["dyaw_deg"]))
                fired = engine.update(t, belief_fresh, truth_fresh, delta)
                for a in fired:
                    print(f"[诊断] {a['rule']}: {a['msg']}")
                alerts.extend(fired)

                # ---- 2Hz 状态表 + jsonl 记录
                if t - last_print >= 0.5:
                    last_print = t
                    ds = ("Δ %.2fm(%.1f格) %+.1f°" % (
                        delta["dpos_m"], delta["dpos_blocks"], delta["dyaw_deg"])) \
                        if delta else "Δ --"
                    if corners is None and truth_px is not None:
                        ts = f"像素({truth_px[0]:.0f},{truth_px[1]:.0f}) 未标定"
                    else:
                        ts = fmt_pose(truth_fresh)
                    active = ",".join(sorted({a["rule"] for a in alerts
                                              if t - a["t"] <= 5.0})) or "无"
                    line = {
                        "t": t,
                        "belief": {"est": snap["est"], "raw": snap["raw"],
                                   "route": snap["route"],
                                   "pursuit": last_pursuit, "cmdvel": last_cmdvel},
                        "truth": truth_fresh,
                        "delta": delta,
                        "alerts": [a["rule"] for a in fired],
                        "snaps": len(snap["snaps"]),
                    }
                    log.write(json.dumps(line, ensure_ascii=False) + "\n")
                    log.flush()
                    print(f"{time.strftime('%H:%M:%S')} 信念 {fmt_pose(belief_fresh)}"
                          f" | 真值 {ts} | {ds} | 告警: {active}")
    except KeyboardInterrupt:
        print("\n[退出] Ctrl-C，回收资源…")
    finally:
        if car is not None:
            car.stop()
            car.join(timeout=5)
        if cam is not None:
            cam.release()
        if args.show:
            cv2.destroyAllWindows()
        lines = _summary_lines(dpos_stat, dyaw_stat, engine.counts)
        for ln in lines:
            print(ln)
        with open(out_path, "a", encoding="utf-8", newline="\n") as log:
            log.write(json.dumps({"summary": lines}, ensure_ascii=False) + "\n")
        print(f"[日志已收尾] {out_path}")
    return 0


# ---------------------------------------------------------------- CLI

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="track_diag.py",
        description="「信念 vs 真实」小车循迹对照诊断工具（详见文件头说明）")
    sub = ap.add_subparsers(dest="cmd")

    pm = sub.add_parser("print-markers", help="生成可打印 ArUco 标记 PNG（id0~5）")
    pm.add_argument("out_dir", help="输出目录")
    pm.add_argument("--size", type=int, default=800, help="标记像素边长（不含安静区）")
    pm.set_defaults(func=cmd_print_markers)

    rn = sub.add_parser("run", help="对照诊断主模式（默认）")
    rn.add_argument("--car-host", default=None, help="小车 IP（给了才连车）")
    rn.add_argument("--car-user", default="duzhong", help="车端用户名（默认 duzhong）")
    rn.add_argument("--car-port", type=int, default=22)
    rn.add_argument("--password", default=None,
                    help="车端密码；更推荐 AG_CAR_PASS 环境变量")
    rn.add_argument("--camera", type=int,
                    default=(int(os.environ["TRACK_DIAG_CAMERA_INDEX"])
                             if os.environ.get("TRACK_DIAG_CAMERA_INDEX") else None),
                    help="UVC 相机索引（DroidCam/Iriun 等虚拟摄像头通常是 1，"
                         "笔记本自带相机占 0；也可设 TRACK_DIAG_CAMERA_INDEX）")
    rn.add_argument("--camera-url", default=os.environ.get("TRACK_DIAG_CAMERA_URL"),
                    help="网络流相机源 URL（安卓 IP Webcam 免费且 1080p："
                         "http://手机IP:8080/video；也可设 TRACK_DIAG_CAMERA_URL，"
                         "bat 菜单自动生效；--camera 优先于本参数）")
    rn.add_argument("--realsense", action="store_true", help="强制用 RealSense")
    rn.add_argument("--corners", default=None,
                    help='场角场地米制坐标，如 "1:0,0 2:3.85,0 3:3.85,4.9 4:0,4.9"')
    rn.add_argument("--corner-config", default=None, help="场角 JSON 配置文件")
    rn.add_argument("--car-marker-yaw-offset", type=float, default=0.0,
                    help="车顶标记贴歪修正（度，逆时针为正）")
    rn.add_argument("--car-marker-id", type=int,
                    default=int(os.environ.get("TRACK_DIAG_CAR_MARKER_ID", "0")),
                    help="车顶标记的 ArUco ID（默认 0。id0 打印件缺失时可把富余的 "
                         "id5（中心校验，可选件）贴车顶，并在 tools/track_diag.local.env "
                         "写 TRACK_DIAG_CAR_MARKER_ID=5——此时中心校验自动让位关闭）")
    rn.add_argument("--show", action="store_true", help="显示诊断画面（网格+双箭头）")
    rn.add_argument("--out", default=None, help="jsonl 日志路径（默认 logs/track_diag/<时间戳>.jsonl）")
    rn.add_argument("--no-car", action="store_true", help="只跑相机侧")
    rn.add_argument("--no-camera", action="store_true", help="只跑 SSH 侧")
    rn.set_defaults(func=cmd_run)
    return ap


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    # 默认子命令是 run：`track_diag.py --car-host ...` 等价 `track_diag.py run ...`
    subs = {"print-markers", "run"}
    if argv and argv[0] not in subs and argv[0] not in ("-h", "--help"):
        if argv[0].startswith("-"):
            argv.insert(0, "run")
        else:
            print(f"未知子命令: {argv[0]}（应为 print-markers / run）", file=sys.stderr)
            return 2
    elif not argv:
        argv = ["run"]
    ap = build_parser()
    args = ap.parse_args(argv)
    if not getattr(args, "func", None):
        ap.print_help()
        return 0
    # ⚠️ 同 remote_run.py：Windows 控制台默认 GBK，先开容错防止特殊符号崩整条命令
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(errors="replace")
        except Exception:  # noqa: BLE001
            pass
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
