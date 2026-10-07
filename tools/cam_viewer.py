#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
云台摄像头实时画面 · 浏览器弹窗
================================================================

把板子的 ROS 2 图像话题（默认 /image_raw）推成 MJPEG 流，并用一个网页
提供**可拖动、可缩放**的弹窗，包含：

    · 图像显示区域（原始画面 / 标注画面 可切换）
    · 刷新 / 重连 按钮
    · 状态提示（连接中 / 已连接 / 无信号 / 连接失败）
    · 目标识别与云台状态实时数值

在 Windows 上直接用浏览器打开即可，**完全不依赖 VNC / DISPLAY**，
也不占板子桌面资源。

用法
----------------------------------------------------------------
    python3 cam_viewer.py                          # 默认 :8080
    python3 cam_viewer.py --port 8081
    python3 cam_viewer.py --fps 25 --quality 85
    python3 cam_viewer.py --scale 0.5              # 缩一半，省 CPU
    python3 cam_viewer.py --topic /image_raw

HTTP 接口
----------------------------------------------------------------
    GET /                          主页面
    GET /popup                     纯弹窗页面（window.open 用）
    GET /stream?mode=raw|ann       MJPEG 视频流
    GET /api/status                状态 JSON（前端轮询）
    GET /api/snapshot?mode=ann     单帧 JPEG

依赖：rclpy / cv2 / numpy / flask（板子均已安装）
"""
from __future__ import annotations

import argparse
import os
import json
import re
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from flask import Flask, Response, jsonify, request
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Int32MultiArray, UInt8

# ============================================================
#  常量
# ============================================================

HERE = Path(__file__).resolve().parent
HTML_PATH = HERE / "cam_viewer.html"

# 调参面板的参数规范表（同目录 param_spec.py）
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import param_spec  # noqa: E402  （必须在 sys.path 处理之后）
import rc_control  # noqa: E402  浏览器遥控器（接管仲裁 + 失效保护）

# 目标类别：视觉端做了 +1 偏移 —— 1=蓝靶、2=红靶，0 表示无目标
CLASS_CN = {0: "无目标", 1: "蓝靶", 2: "红靶"}
CLASS_ASCII = {0: "NONE", 1: "BLUE", 2: "RED"}
# BGR 颜色
CLASS_COLOR = {1: (255, 190, 0), 2: (60, 60, 255)}

# 云台状态机
AIM_CN = {0: "IDLE 空闲", 1: "SCAN 扫描", 2: "TRACK 锁定"}
AIM_ASCII = {0: "IDLE", 1: "SCAN", 2: "TRACK"}

# 超过这个秒数没收到新帧就判定为"无信号"
STALE_SEC = 1.0


# ============================================================
#  图像消息 -> OpenCV BGR
# ============================================================

def _rows(msg: Image, channels: int) -> np.ndarray:
    """把 sensor_msgs/Image 的 data 还原成 (h, w, channels)，正确处理行填充。"""
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    h, w, step = msg.height, msg.width, msg.step
    need = w * channels

    if step == need:
        return arr[: h * step].reshape(h, w, channels)

    # 行间有填充，逐行取有效字节
    out = np.empty((h, w, channels), dtype=np.uint8)
    for i in range(h):
        out[i] = arr[i * step: i * step + need].reshape(w, channels)
    return out


def imgmsg_to_bgr(msg: Image) -> np.ndarray | None:
    """把 ROS 图像消息转成 BGR ndarray。不支持的类型返回 None。"""
    enc = (msg.encoding or "").lower()
    try:
        if enc in ("bgr8", "8uc3"):
            return _rows(msg, 3).copy()
        if enc == "rgb8":
            return _rows(msg, 3)[:, :, ::-1].copy()
        if enc in ("mono8", "8uc1"):
            return cv2.cvtColor(_rows(msg, 1), cv2.COLOR_GRAY2BGR)
        if enc == "bgra8":
            return cv2.cvtColor(_rows(msg, 4), cv2.COLOR_BGRA2BGR)
        if enc == "rgba8":
            return cv2.cvtColor(_rows(msg, 4), cv2.COLOR_RGBA2BGR)
        if enc in ("yuv422", "yuyv", "yuv422_yuy2", "yuy2"):
            # YUYV 打包格式，reshape 成 (h, w, 2) 后直接交给 OpenCV
            return cv2.cvtColor(_rows(msg, 2), cv2.COLOR_YUV2BGR_YUYV)
        if enc in ("mjpeg", "jpeg", "jpg", "compressed"):
            # MJPEG 压缩帧，直接解码成 BGR
            return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8),
                                cv2.IMREAD_COLOR)
    except Exception:  # noqa: BLE001 - 解码失败按无信号处理，不拖垮节点
        return None
    return None


# ============================================================
#  绘制
# ============================================================

def _text(img, s, org, color, scale=0.5, thick=1):
    """带黑色描边的文字，保证任何背景下都看得清。"""
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0),
                thick + 2, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color,
                thick, cv2.LINE_AA)


def annotate(img: np.ndarray, det: dict, aim: int, scale: float) -> np.ndarray:
    """在画面上叠加检测框 / 靶心关键点 / 偏移箭头 / 数值 HUD。

    注意：OpenCV 的 putText 不支持中文，所以图上只写英文，
    中文由网页的状态栏显示。
    """
    out = img.copy()
    h, w = out.shape[:2]
    cx, cy = w // 2, h // 2

    # 画面中心参考十字
    cv2.line(out, (cx - 18, cy), (cx + 18, cy), (105, 105, 105), 1)
    cv2.line(out, (cx, cy - 18), (cx, cy + 18), (105, 105, 105), 1)

    cls = int(det.get("cls", 0))
    bw = int(det.get("bw", 0))
    bh = int(det.get("bh", 0))

    if cls in (1, 2) and bw > 0 and bh > 0:
        bcx = cx + det["dx"] * scale
        bcy = cy + det["dy"] * scale
        bw_s = max(2.0, bw * scale)
        bh_s = max(2.0, bh * scale)
        x1 = int(bcx - bw_s / 2)
        y1 = int(bcy - bh_s / 2)
        x2 = int(bcx + bw_s / 2)
        y2 = int(bcy + bh_s / 2)

        col = CLASS_COLOR[cls]
        cv2.rectangle(out, (x1, y1), (x2, y2), col, 2)

        # 画面中心 -> 框中心 的偏移箭头
        cv2.arrowedLine(out, (cx, cy), (int(bcx), int(bcy)), col, 1,
                        tipLength=0.10)

        # 靶心关键点（精度高于框中心）
        kconf = int(det.get("kconf", 0))
        if kconf > 0:
            kx = int(cx + det["kx"] * scale)
            ky = int(cy + det["ky"] * scale)
            cv2.drawMarker(out, (kx, ky), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
            cv2.circle(out, (kx, ky), 11, (0, 255, 0), 1)

        _text(out, "%s %d%%" % (CLASS_ASCII.get(cls, "?"), kconf),
              (x1, max(14, y1 - 8)), col, 0.55)

    # 左上角数值 HUD
    hud = [
        "dx %+d   dy %+d" % (det.get("dx", 0), det.get("dy", 0)),
        "box %dx%d" % (bw, bh),
        "kp %+d %+d   conf %d%%" % (det.get("kx", 0), det.get("ky", 0),
                                    det.get("kconf", 0)),
        "cls %d %s" % (cls, CLASS_ASCII.get(cls, "?")),
        "aim %s" % AIM_ASCII.get(aim, "?"),
    ]
    pad, lh = 8, 18
    box_w = 210
    box_h = pad * 2 + lh * len(hud)
    roi = out[8:8 + box_h, 8:8 + box_w]
    if roi.size:
        cv2.addWeighted(roi, 0.35, np.zeros_like(roi), 0.65, 0, roi)
    for i, line in enumerate(hud):
        _text(out, line, (8 + pad, 8 + pad + lh * (i + 1) - 5),
              (200, 210, 220), 0.45)

    return out


def make_placeholder(hub, text: str) -> bytes:
    """没有图像时用的占位帧。"""
    w, h = 640, 360
    img = np.full((h, w, 3), 21, np.uint8)
    for x in range(0, w, 40):
        cv2.line(img, (x, 0), (x, h), (33, 33, 33), 1)
    for y in range(0, h, 40):
        cv2.line(img, (0, y), (w, y), (33, 33, 33), 1)
    _text(img, text, (w // 2 - 90, h // 2 - 6), (95, 95, 95), 0.9, 2)
    _text(img, "waiting on %s" % hub.topic,
          (w // 2 - 105, h // 2 + 26), (70, 70, 70), 0.5)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return buf.tobytes() if ok else b""


# ============================================================
#  共享状态
# ============================================================

def load_gun_limits(path: str) -> tuple:
    """从配置中心读弹药上限，供网页显示 "已打 N / 共 M 发"。

    读不到就用 (10, 0) 兜底 —— 网页面板显示不出来是小事，
    绝不能因为它读不到配置就影响视频流。
    """
    try:
        import yaml
        with open(path, encoding="utf-8") as fp:
            d = yaml.safe_load(fp) or {}
        p = (d.get("/dzrknn_dataprocess") or {}).get("ros__parameters", {}) or {}
        return int(p.get("max_shots", 10)), int(p.get("max_shots_per_target", 0))
    except Exception:
        return 10, 0


# 每次开火记录下来的一条事件（供网页"开火记录"面板展示）
# 关键价值：把"打了几发"和"当时锁的是什么、瞄得准不准"绑在一起。
# 单看弹药数只能知道打了多少；配上类别/偏移/框宽，才能判断
# "这一发是打蓝靶打的，还是瞄偏了打的"。
# =============================================================================
#  配置中心读写（调参面板的后端）
# -----------------------------------------------------------------------------
#  ★ 为什么"按行精确改写"而不是 yaml.safe_dump 整体回写：
#    config/robot.yaml 共 333 行，其中 137 行是注释（占 41%）——
#    它既是配置也是文档。safe_dump 会把注释全部抹掉，
#    那比改错一个参数还糟（后来的人再也看不懂为什么这么设）。
#    所以这里只在目标键所在行上做替换，其余字节原样保留。
#
#  ★ 为什么改完要"重启节点"而不是 ros2 param set 热改：
#    节点只在构造函数里 get_parameter 一次，之后不会重新读取
#    （见 memo 第 34 条：scan_pitch_offset_deg 声明了却漏了 get）。
#    所以"写 YAML + 重启该节点"是唯一可靠且行为可预期的路径。
# =============================================================================

CONFIG_PATH = os.environ.get(
    "ROBOT_CONFIG", os.path.expanduser("~/dzrobot/config/robot.yaml"))
RESTART_SH = str(HERE / "restart_node.sh")


def _fmt_val(item, v):
    """按参数类型把 Python 值格式化成 YAML 字面量。"""
    t = item.get("type", "num")
    if t == "bool":
        return "true" if bool(v) else "false"
    if t == "num":
        dec = int(item.get("dec", 2))
        if dec <= 0:
            return str(int(round(float(v))))
        return ("%." + str(dec) + "f") % float(v)
    txt = str(v).strip()
    if re.fullmatch(r"[A-Za-z0-9_.,:*/+-]+", txt):
        return txt
    return '"' + txt.replace('"', '\\"') + '"'


def _coerce(item, raw):
    """把前端传来的值转成配置里该有的类型；越界/非法抛 ValueError。"""
    t = item.get("type", "num")
    if t == "bool":
        if isinstance(raw, bool):
            return raw
        return str(raw).strip().lower() in ("1", "true", "yes", "on")
    if t == "num":
        try:
            f = float(raw)
        except (TypeError, ValueError):
            raise ValueError("不是数字: %r" % (raw,))
        lo, hi = item.get("min"), item.get("max")
        if lo is not None and f < lo:
            raise ValueError("小于安全下限 %s" % lo)
        if hi is not None and f > hi:
            raise ValueError("大于安全上限 %s" % hi)
        return int(round(f)) if int(item.get("dec", 2)) <= 0 else f
    if t == "list":
        if isinstance(raw, list):
            parts = [str(x) for x in raw]
        else:
            parts = re.split(r"[,\s]+", str(raw).strip())
        out = [p.strip().upper() for p in parts if p.strip()]
        if not out:
            raise ValueError("列表不能为空")
        return out
    txt = str(raw).strip()
    choices = item.get("choices")
    if choices and txt not in choices:
        raise ValueError("只能是 %s" % " / ".join(choices))
    return txt


def _section_range(lines, section):
    """返回 YAML 顶层段的行区间 [start, end)。"""
    pat = re.compile(r"^%s:\s*$" % re.escape(section))
    start = None
    for i, ln in enumerate(lines):
        if pat.match(ln):
            start = i
            break
    if start is None:
        return None, None
    for j in range(start + 1, len(lines)):
        ln = lines[j]
        if ln and not ln[0].isspace() and not ln.lstrip().startswith("#"):
            return start, j
    return start, len(lines)


def set_yaml_value(text, section, key, item, value):
    """在指定段内精确改写 key 的值，保留行尾注释。返回 (新文本, 错误)。"""
    lines = text.split("\n")
    s0, s1 = _section_range(lines, section)
    if s0 is None:
        return text, "配置里找不到段 %s" % section

    kpat = re.compile(r"^(\s+)%s:(\s*)(.*)$" % re.escape(key))
    hit = None
    for i in range(s0 + 1, s1):
        if lines[i].lstrip().startswith("#"):
            continue
        m = kpat.match(lines[i])
        if m:
            hit = (i, m)
            break
    if hit is None:
        return text, "段 %s 里找不到键 %s" % (section, key)

    i, m = hit
    indent = m.group(1)
    rest = m.group(3)

    if item.get("type") == "list":
        # 块状列表：把 key 行 + 其后的 "- x" 行整段换掉
        j = i + 1
        while j < s1 and re.match(r"^\s*-\s", lines[j]):
            j += 1
        body = ["%s  - %s" % (indent, str(x).strip()) for x in value]
        lines[i:j] = ["%s%s:" % (indent, key)] + body
        return "\n".join(lines), None

    # 保留行尾注释，并尽量维持它原来的列位 ——
    # 否则每改一次值都让注释左右跳一格，diff 会变得很难读。
    head = "%s%s: %s" % (indent, key, _fmt_val(item, value))
    cm = re.search(r"\s+#", rest)
    if cm:
        # rest 是从 "key:" 之后开始的，换算回整行列位：
        #   base = 缩进 + 键名 + 冒号 + 冒号后的空白
        #   hash_at = '#' 在 rest 里的下标（要算上它前面的空白）
        base = len(indent) + len(key) + 1 + len(m.group(2))
        hash_at = cm.start() + len(cm.group(0)) - 1
        pad = max(2, base + hash_at - len(head))
        lines[i] = head + " " * pad + rest[hash_at:].strip()
    else:
        lines[i] = head
    return "\n".join(lines), None


def config_snapshot():
    """读配置中心，按规范表组装成面板需要的数据结构。"""
    import yaml
    try:
        with open(CONFIG_PATH, encoding="utf-8") as fp:
            data = yaml.safe_load(fp) or {}
    except Exception as exc:
        return {"ok": False, "error": "配置读取失败: %s" % exc, "groups": []}

    groups = []
    for g in param_spec.PARAM_GROUPS:
        section = param_spec.NODES.get(g["node"])
        live = ((data.get(section) or {}).get("ros__parameters") or {})
        items = []
        for it in g["items"]:
            items.append({
                "key": it["key"],
                "label": it["label"],
                "type": it.get("type", "num"),
                "unit": it.get("unit", ""),
                "min": it.get("min"),
                "max": it.get("max"),
                "step": it.get("step"),
                "dec": it.get("dec", 2),
                "choices": it.get("choices"),
                "hint": it.get("hint", ""),
                "value": live.get(it["key"]),
                "missing": it["key"] not in live,
            })
        groups.append({
            "id": g["id"], "name": g["name"], "restart": g.get("restart"),
            "note": g.get("note", ""), "items": items,
            "section": section,          # 前端提交时用它拼 changes 的键
        })
    return {"ok": True, "groups": groups, "config": CONFIG_PATH}


def restart_nodes(targets):
    """调 restart_node.sh 重启指定节点，返回逐节点结果。"""
    results = []
    env = dict(os.environ)
    env.setdefault("ROBOT_CONFIG", CONFIG_PATH)
    for t in targets:
        try:
            r = subprocess.run([RESTART_SH, t], capture_output=True,
                               text=True, timeout=60, env=env)
            tail = (r.stdout or r.stderr or "").strip().splitlines()
            results.append({"node": t, "ok": r.returncode == 0,
                            "log": "\n".join(tail[-4:])})
        except subprocess.TimeoutExpired:
            results.append({"node": t, "ok": False, "log": "重启超时(60s)"})
        except Exception as exc:
            results.append({"node": t, "ok": False, "log": str(exc)})
    return results


class Hub:
    """线程安全的共享状态：最新帧 + 检测结果 + 编码缓存。"""

    def __init__(self, topic: str, stream_fps: float, quality: int, scale: float,
                 max_shots: int = 10, per_target: int = 0):
        self.topic = topic
        self.max_shots = int(max_shots)      # 全程弹药上限(来自配置中心)
        self.per_target = int(per_target)    # 单目标连发上限
        self.stream_fps = max(1.0, float(stream_fps))
        self.quality = int(quality)
        self.scale = float(scale) if scale > 0 else 1.0

        self._lock = threading.Lock()
        self._frame = None            # 最新 BGR 帧（已按 scale 缩放）
        self._frame_t = 0.0
        self._enc = ""
        self._wh = (0, 0)
        self._frames = 0
        self._times = deque(maxlen=60)

        # ---- ★地图轨迹 ----
        # /odom/localization 的 position 单位是【格】不是米：
        #     x = 11.5 - row      y = 13.5 - col
        # （踩过坑：把它当米读，会以为车越界了）
        self._track = deque(maxlen=4000)   # [(x_格, y_格, t), ...]
        self._trk_last = None              # 上一个采样点（做抽稀用）
        # ★运动状态 0=前进 1=倒车（来自 /nav/motion_state）
        #   轨迹点要带上这个标记，前端才能把倒车段画成另一种颜色 ——
        #   踩过的坑：不分色时倒车轨迹画成了前进，看起来像"车倒着长距离跑"。
        self._motion = 0
        self._map = self._load_map()       # 地图要素（来自 config/map.yaml）

        self._det = dict(cls=0, dx=0, dy=0, bw=0, bh=0, kx=0, ky=0, kconf=0)
        self._det_t = 0.0
        self._aim = 0

        # ---- 开火记录 ----
        self._shots = 0                 # 累计发射数(来自 /shoot_count)
        self._shot_log = deque(maxlen=60)   # 最近若干条开火事件
        self._shot_prev_t = 0.0

        self._jpeg = {"raw": None, "ann": None}
        self._jpeg_lock = threading.Lock()
        self._clients_lock = threading.Lock()
        self._clients = 0
        self._placeholder = None
        self._stop = threading.Event()

    # ---------- 订阅回调 ----------

    def on_image(self, msg: Image) -> None:
        frame = imgmsg_to_bgr(msg)
        if frame is None:
            return
        if self.scale != 1.0:
            frame = cv2.resize(frame, None, fx=self.scale, fy=self.scale,
                               interpolation=cv2.INTER_AREA)
        now = time.time()
        with self._lock:
            self._frame = frame
            self._frame_t = now
            self._enc = msg.encoding
            self._wh = (msg.width, msg.height)
            self._frames += 1
            self._times.append(now)

    def on_det(self, msg: Int32MultiArray) -> None:
        d = list(msg.data) + [0] * (8 - len(msg.data))
        with self._lock:
            self._det = dict(cls=int(d[2]), dx=int(d[0]), dy=int(d[1]),
                             bw=int(d[3]), bh=int(d[4]), kx=int(d[5]),
                             ky=int(d[6]), kconf=int(d[7]))
            self._det_t = time.time()

    def on_aim(self, msg: UInt8) -> None:
        with self._lock:
            self._aim = int(msg.data)

    @staticmethod
    def _load_map():
        """读 config/map.yaml（地图要素）。读不到就返回空，界面只画轨迹。"""
        try:
            import yaml
            p = os.path.expanduser("~/dzrobot/config/map.yaml")
            if not os.path.exists(p):
                return {}
            with open(p, encoding="utf-8") as fp:
                return (yaml.safe_load(fp) or {}).get("map") or {}
        except Exception:
            return {}

    def reload_map(self) -> None:
        """热重载地图（改完 map.yaml 不用重启 viewer）。"""
        self._map = self._load_map()
        self._track.clear()
        self._trk_last = None

    def on_odom(self, msg: Odometry) -> None:
        """记录轨迹点。位置单位是【格】，直接按格存，前端换算成像素。

        抽稀：与上一个点距离小于 0.02 格就不记 —— 50Hz 全记会把缓冲撑爆，
        而且画出来也看不出区别。
        """
        try:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            now = time.time()
            with self._lock:
                if self._trk_last is not None:
                    dx = px - self._trk_last[0]
                    dy = py - self._trk_last[1]
                    if dx * dx + dy * dy < 0.0004:      # < 0.02 格
                        return
                self._trk_last = (px, py)
                self._track.append((round(px, 3), round(py, 3), round(now, 2), self._motion))
        except Exception:
            pass

    def _shoot_points(self):
        """从 robot.yaml 的 mission_blocks 里挑出射击点（带 * 的项）。

        这是【循迹当前真正会去停的射击点】，比重新算一遍更可信 ——
        浏览器上显示的就是车上实际执行的，不会出现"界面显示A、车去了B"。
        顺便把带 R: 前缀的倒车点也标出来。
        """
        out = {"shoot": [], "reverse": []}
        try:
            import yaml
            p = os.path.expanduser("~/dzrobot/config/robot.yaml")
            if not os.path.exists(p):
                return out
            with open(p, encoding="utf-8") as fp:
                d = yaml.safe_load(fp) or {}
            mb = ((d.get("path_executor") or {}).get("ros__parameters") or {}
                  ).get("mission_blocks") or []
            for tok in mb:
                t = str(tok).strip()
                if not t:
                    continue
                rev = t.upper().startswith("R:")
                if rev:
                    t = t[2:].strip()
                is_shoot = t.endswith("*")
                b = (t[:-1] if is_shoot else t).upper()
                if ":" in b:
                    b = b.split(":", 1)[0]
                if "," in b:
                    b = b.split(",", 1)[0]
                b = b.strip()
                if len(b) < 2 or not b[1:].isdigit():
                    continue
                if is_shoot:
                    out["shoot"].append(b)
                if rev:
                    out["reverse"].append(b)
        except Exception:
            pass
        return out

    def on_motion(self, msg) -> None:
        """缓存运动状态（0=前进 1=倒车）。"""
        try:
            self._motion = int(msg.data)
        except Exception:
            pass

    def track_payload(self) -> dict:
        """给 /api/track 的完整数据：地图要素 + 轨迹 + 当前位置。"""
        with self._lock:
            trk = list(self._track)
            det = dict(self._det)
            aim = self._aim
        cur = trk[-1] if trk else None
        sp = self._shoot_points()
        return {
            "map": self._map,
            # 每点 [x格, y格, 运动状态]；状态 0=前进 1=倒车
            "track": [[p[0], p[1], p[3] if len(p) > 3 else 0] for p in trk],
            "current": [cur[0], cur[1]] if cur else None,
            "n": len(trk),
            "det": det,
            "aim": aim,
            "shoot_points": sp["shoot"],      # ★最佳射击点（车上实际执行的）
            "reverse_points": sp["reverse"],  # 倒车点
        }

    def on_shot(self, msg: Int32) -> None:
        """收到一发开火事件的瞬间，把"打的时候是什么情况"一起快照下来。

        注意：/shoot_count 发的是【累计值】，不是增量。云台重启会从 0 重新计数，
        所以这里用"值变大"判断新的一发，值变小则视为重置。
        """
        with self._lock:
            count = int(msg.data)
            if count <= self._shots and count != 0:
                return                      # 重复/乱序，忽略
            if count < self._shots:
                self._shot_log.clear()      # 云台重启，计数归零
            self._shots = count

            now = time.time()
            gap = (now - self._shot_prev_t) if self._shot_prev_t else None
            self._shot_prev_t = now
            self._shot_log.append({
                "n": count,
                "t": round(now, 3),
                "wall": time.strftime("%H:%M:%S", time.localtime(now)),
                "gap": round(gap, 2) if gap is not None else None,
                "cls": int(self._det.get("cls", 0)),
                "cls_name": CLASS_CN.get(self._det.get("cls", 0), "?"),
                "dx": int(self._det.get("dx", 0)),
                "dy": int(self._det.get("dy", 0)),
                "bw": int(self._det.get("bw", 0)),
                "conf": int(self._det.get("kconf", 0)),
                "aim": int(self._aim),
            })

    # ---------- 客户端计数 ----------

    @property
    def clients(self) -> int:
        with self._clients_lock:
            return self._clients

    def add_client(self) -> None:
        with self._clients_lock:
            self._clients += 1

    def drop_client(self) -> None:
        with self._clients_lock:
            self._clients = max(0, self._clients - 1)

    # ---------- 取帧 ----------

    def jpeg(self, mode: str) -> bytes | None:
        with self._jpeg_lock:
            return self._jpeg.get(mode)

    def placeholder(self) -> bytes:
        if self._placeholder is None:
            self._placeholder = make_placeholder(self, "NO SIGNAL")
        return self._placeholder

    def _fps_locked(self) -> float:
        if len(self._times) < 2:
            return 0.0
        # ★ 帧停了之后不能把最后一次算出的 fps 一直挂着，
        #   否则"无信号"状态下工具栏还显示 18.1 fps，会误导排查。
        if time.time() - self._times[-1] > 2.0:
            return 0.0
        span = self._times[-1] - self._times[0]
        return (len(self._times) - 1) / span if span > 1e-6 else 0.0

    # ---------- 编码线程 ----------

    def encode_loop(self) -> None:
        """只在有浏览器连着时才编码，避免白烧板子 CPU。"""
        params = [cv2.IMWRITE_JPEG_QUALITY, self.quality]
        while not self._stop.is_set():
            if self.clients == 0:
                time.sleep(0.08)
                continue

            t0 = time.time()
            with self._lock:
                frame = None if self._frame is None else self._frame
                det = dict(self._det)
                aim = self._aim
            if frame is not None:
                raw = frame
                ann = annotate(frame, det, aim, 1.0)  # 帧已缩放，偏移量同尺度
                for mode, img in (("raw", raw), ("ann", ann)):
                    ok, buf = cv2.imencode(".jpg", img, params)
                    if ok:
                        with self._jpeg_lock:
                            self._jpeg[mode] = buf.tobytes()

            dt = time.time() - t0
            time.sleep(max(0.0, 1.0 / self.stream_fps - dt))

    def stop(self) -> None:
        self._stop.set()

    # ---------- 状态 ----------

    def status(self) -> dict:
        with self._lock:
            now = time.time()
            age = (now - self._frame_t) if self._frame_t else 9999.0
            ok = age < STALE_SEC
            payload = {
                "image_ok": ok,
                "age": round(age, 3) if ok else None,
                "fps": round(self._fps_locked(), 1),
                "frames": self._frames,
                "topic": self.topic,
                "encoding": self._enc,
                "width": self._wh[0],
                "height": self._wh[1],
                "aim_status": self._aim,
                "aim_name": AIM_CN.get(self._aim, "?"),
                "det": dict(self._det, name=CLASS_CN.get(self._det["cls"], "?")),
                "det_age": round(now - self._det_t, 3) if self._det_t else None,
                "shots": self._shots,
                "shot_log": list(self._shot_log)[-12:],   # 只回传最近 12 条
            }
        payload["stream_clients"] = self.clients
        payload["stream_fps"] = self.stream_fps
        payload["max_shots"] = self.max_shots
        payload["per_target"] = self.per_target
        return payload


# ============================================================
#  ROS 节点
# ============================================================

class CamViewerNode(Node):
    def __init__(self, hub: Hub):
        super().__init__("cam_viewer")
        self.hub = hub

        # best_effort 订阅者可以同时兼容 reliable / best_effort 两种发布者
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Image, hub.topic, hub.on_image, qos)
        self.create_subscription(Int32MultiArray, "/offset_center", hub.on_det, qos)
        self.create_subscription(UInt8, "/aim/status", hub.on_aim, qos)
        self.create_subscription(Int32, "/shoot_count", hub.on_shot, qos)
        # ★轨迹订阅：/odom/localization（serial_reader 发布，单位=格）
        self.create_subscription(Odometry, "/odom/localization", hub.on_odom, qos)
        # ★运动状态（前进/倒车）-> 轨迹分色
        self.create_subscription(UInt8, "/nav/motion_state", hub.on_motion, qos)
        self.get_logger().info("cam_viewer 已启动，订阅 %s" % hub.topic)


# ============================================================
#  Web
# ============================================================

def mjpeg_gen(hub: Hub, mode: str):
    """MJPEG 分块推流。"""
    hub.add_client()
    hdr = b"--frame\r\nContent-Type: image/jpeg\r\n"
    interval = 1.0 / hub.stream_fps
    try:
        while True:
            buf = hub.jpeg(mode)
            if buf is None:
                buf = hub.placeholder()
                time.sleep(0.4)
            else:
                time.sleep(interval)
            yield (hdr + b"Content-Length: " + str(len(buf)).encode()
                   + b"\r\n\r\n" + buf + b"\r\n")
    finally:
        hub.drop_client()


def load_page(popup: bool) -> str:
    html = HTML_PATH.read_text(encoding="utf-8")
    return html.replace("__POPUP__", "true" if popup else "false")


def load_pitch_offset(path: str) -> float:
    """读配置中心的俯仰基准角 —— 遥控云台回正时要用同一个姿态。"""
    try:
        import yaml
        with open(path, encoding="utf-8") as fp:
            d = yaml.safe_load(fp) or {}
        p = (d.get("/dzrknn_dataprocess") or {}).get("ros__parameters", {}) or {}
        return float(p.get("scan_pitch_offset_deg", -26.0))
    except Exception:
        return -26.0


def create_app(hub: Hub, rc: "rc_control.RemoteControl" = None) -> Flask:
    app = Flask(__name__)
    app.config["JSON_AS_ASCII"] = False

    @app.route("/")
    def index():
        return Response(load_page(False), mimetype="text/html; charset=utf-8")

    @app.route("/popup")
    def popup():
        return Response(load_page(True), mimetype="text/html; charset=utf-8")

    @app.route("/api/status")
    def api_status():
        body = json.dumps(hub.status(), ensure_ascii=False)
        return Response(body, mimetype="application/json; charset=utf-8")

    @app.route("/api/track")
    def api_track():
        """地图轨迹数据（前端 4Hz 轮询）。"""
        return jsonify(hub.track_payload())

    @app.route("/api/reload_map", methods=["POST", "GET"])
    def api_reload_map():
        """热重载 config/map.yaml。"""
        hub.reload_map()
        m = hub.track_payload().get("map") or {}
        return jsonify({"ok": True, "name": m.get("name", ""),
                        "cells": sum(len(m.get(k) or [])
                                     for k in ("mines", "obstacles",
                                               "targets", "bridges"))})

    @app.route("/api/snapshot")
    def api_snapshot():
        mode = request.args.get("mode", "ann")
        if mode not in ("raw", "ann"):
            mode = "ann"
        return Response(hub.jpeg(mode) or hub.placeholder(),
                        mimetype="image/jpeg")

    @app.route("/stream")
    def stream():
        mode = request.args.get("mode", "ann")
        if mode not in ("raw", "ann"):
            mode = "ann"
        return Response(
            mjpeg_gen(hub, mode),
            mimetype="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
        )

    # ============================================================
    #  ★ 配置中心读写（调参面板）
    # ============================================================

    @app.route("/api/config", methods=["GET"])
    def api_config_get():
        """返回调参面板需要的全部信息：分组 + 每项当前值（直接读配置中心）。"""
        return jsonify(config_snapshot())

    @app.route("/api/config", methods=["POST"])
    def api_config_post():
        """写入配置中心，然后重启受影响的节点让它生效。

        请求体: {"changes": {"path_executor": {"linear_speed": 0.2}, ...}}
        返回:   {"ok":bool, "written":[...], "errors":[...], "restarts":[...]}

        设计要点：
          · 只允许改参数规范表（param_spec.py）里白名单内的键 —— 其余参数
            属于标定常量，在网页上误改的代价太高，留给配置文件直接编辑。
          · 有任何一项非法就【整体不落盘】，避免出现"改了一半"的中间态。
          · 落盘前备份 .bak，落盘后回读校验 YAML 合法，不合法自动回滚。
        """
        body = request.get_json(silent=True) or {}
        changes = body.get("changes") or {}
        if not isinstance(changes, dict) or not changes:
            return jsonify({"ok": False, "error": "没有要修改的参数"}), 400

        try:
            with open(CONFIG_PATH, encoding="utf-8") as fp:
                text = fp.read()
        except Exception as exc:
            return jsonify({"ok": False, "error": "配置读取失败: %s" % exc}), 500

        written, errors, targets = [], [], set()

        for section, kv in changes.items():
            if not isinstance(kv, dict):
                continue
            for key, raw in kv.items():
                g, item = param_spec.find_item(section, key)
                if item is None:
                    errors.append("%s.%s：不在可调白名单内，请直接编辑配置文件"
                                  % (section, key))
                    continue
                try:
                    val = _coerce(item, raw)
                except ValueError as exc:
                    errors.append("%s.%s：%s" % (section, key, exc))
                    continue
                text, err = set_yaml_value(text, section, key, item, val)
                if err:
                    errors.append("%s.%s：%s" % (section, key, err))
                    continue
                written.append("%s.%s = %s" % (section, key, _fmt_val(item, val)))
                if g and g.get("restart"):
                    targets.add(g["restart"])

        if errors:
            return jsonify({"ok": False, "errors": errors,
                            "hint": "未写入任何改动，配置文件保持原样"}), 400

        import shutil
        try:
            shutil.copyfile(CONFIG_PATH, CONFIG_PATH + ".bak")
            tmp = CONFIG_PATH + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fp:
                fp.write(text)
            os.replace(tmp, CONFIG_PATH)
        except Exception as exc:
            return jsonify({"ok": False, "error": "写入失败: %s" % exc}), 500

        # 回读校验：写坏了立刻回滚，绝不让车读到一份坏配置
        try:
            import yaml
            with open(CONFIG_PATH, encoding="utf-8") as fp:
                yaml.safe_load(fp)
        except Exception as exc:
            shutil.copyfile(CONFIG_PATH + ".bak", CONFIG_PATH)
            return jsonify({"ok": False,
                            "error": "写入后 YAML 无法解析，已自动回滚: %s" % exc}), 500

        order = [t for t in ("gimbal", "vision", "path") if t in targets]
        restarts = restart_nodes(order) if order else []
        return jsonify({"ok": True, "written": written,
                        "restarts": restarts, "snapshot": config_snapshot()})

    # ============================================================
    #  ★ 遥控器 API
    # ------------------------------------------------------------
    #  安全要点（前端与后端共同保证）：
    #    · 除 estop / state 外，所有遥控动作都要求【已接管】——
    #      没接管就发速度指令会被直接拒绝，避免与循迹抢控制权。
    #    · estop 任何时候都生效（保命按钮不该有前置条件）。
    #    · 服务端有接管看门狗，浏览器崩了/网断了会自动刹车并释放。
    # ============================================================

    def _need_rc():
        if rc is None:
            return jsonify({"ok": False, "error": "遥控模块未启用"}), 503
        return None

    @app.route("/api/rc", methods=["GET"])
    def api_rc_state():
        bad = _need_rc()
        if bad:
            return bad
        return jsonify(rc.state())

    @app.route("/api/rc/takeover", methods=["POST"])
    def api_rc_takeover():
        bad = _need_rc()
        if bad:
            return bad
        body = request.get_json(silent=True) or {}
        return jsonify(rc.takeover(bool(body.get("on"))))

    @app.route("/api/rc/drive", methods=["POST"])
    def api_rc_drive():
        bad = _need_rc()
        if bad:
            return bad
        body = request.get_json(silent=True) or {}
        try:
            v = float(body.get("v", 0.0))
            w = float(body.get("w", 0.0))
        except (TypeError, ValueError):
            return jsonify({"ok": False, "error": "v/w 必须是数字"}), 400
        res = rc.drive(v, w)
        return jsonify(res), (200 if res.get("ok") else 409)

    @app.route("/api/rc/gimbal", methods=["POST"])
    def api_rc_gimbal():
        bad = _need_rc()
        if bad:
            return bad
        body = request.get_json(silent=True) or {}
        if body.get("home"):
            res = rc.gimbal_home()
        elif body.get("dp") is not None or body.get("dy") is not None:
            # 增量步进：前端按住方向键时按固定节奏发 dp/dy
            try:
                dp = float(body.get("dp") or 0.0)
                dy = float(body.get("dy") or 0.0)
            except (TypeError, ValueError):
                return jsonify({"ok": False, "error": "dp/dy 必须是数字"}), 400
            res = rc.gimbal_step(dp, dy)
        else:
            try:
                p = None if body.get("pitch") is None else float(body["pitch"])
                y = None if body.get("yaw") is None else float(body["yaw"])
            except (TypeError, ValueError):
                return jsonify({"ok": False, "error": "pitch/yaw 必须是数字"}), 400
            res = rc.gimbal(p, y)
        return jsonify(res), (200 if res.get("ok") else 409)

    @app.route("/api/rc/shoot", methods=["POST"])
    def api_rc_shoot():
        bad = _need_rc()
        if bad:
            return bad
        res = rc.shoot()
        return jsonify(res), (200 if res.get("ok") else 409)

    @app.route("/api/rc/laser", methods=["POST"])
    def api_rc_laser():
        """激光常亮开关（调试用）。?on=1|0  &secs=3（>0 则 N 秒后自动熄灭）"""
        bad = _need_rc()
        if bad:
            return bad
        on = request.args.get("on", "1") not in ("0", "false", "False")
        try:
            secs = float(request.args.get("secs", "0") or 0)
        except ValueError:
            secs = 0.0
        return jsonify(rc.laser(on, secs))

    @app.route("/api/rc/reset_shots", methods=["POST"])
    def api_rc_reset_shots():
        """重置射击计数（软件侧），10 发弹药恢复满额。"""
        bad = _need_rc()
        if bad:
            return bad
        return jsonify(rc.reset_shots())

    @app.route("/api/referee", methods=["GET"])
    def api_referee():
        """裁判系统状态：弹药/累计射击/血量/被击打。stale=True 表示超过 3s 没更新。"""
        return jsonify(rc.referee_state())

    @app.route("/api/rc/referee", methods=["POST"])
    def api_rc_referee():
        """向裁判系统下发任意协议帧。body: {"hex":"B5 5B 03 21 01 25"}"""
        bad = _need_rc()
        if bad:
            return bad
        payload = request.get_json(silent=True) or {}
        hexstr = payload.get("hex") or request.args.get("hex", "")
        return jsonify(rc.referee_cmd(hexstr))

    @app.route("/api/rc/estop", methods=["POST"])
    def api_rc_estop():
        bad = _need_rc()
        if bad:
            return bad
        return jsonify(rc.estop())

    @app.after_request
    def no_cache(resp):
        resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
        return resp

    return app


# ============================================================
#  入口
# ============================================================

def lan_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:  # noqa: BLE001
        return "127.0.0.1"


def port_busy(host: str, port: int) -> bool:
    """先探一次端口。比等 Flask 抛 'Address already in use' 更早、更友好。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind((host, port))
        return False
    except OSError:
        return True
    finally:
        s.close()


def main() -> int:
    ap = argparse.ArgumentParser(description="云台摄像头浏览器弹窗")
    ap.add_argument("--topic", default="/image_raw", help="图像话题")
    ap.add_argument("--host", default="0.0.0.0", help="监听地址")
    ap.add_argument("--port", type=int, default=8080, help="监听端口")
    ap.add_argument("--fps", type=float, default=15.0, help="推流帧率上限")
    ap.add_argument("--quality", type=int, default=75, help="JPEG 质量 1~100")
    ap.add_argument("--scale", type=float, default=1.0, help="图像缩放系数")
    ap.add_argument("--config", default=os.path.expanduser("~/dzrobot/config/robot.yaml"),
                    help="配置中心路径（用于读取弹药上限，显示为 N/10）")
    args = ap.parse_args()

    if not HTML_PATH.exists():
        print("[错误] 找不到界面文件：%s" % HTML_PATH, file=sys.stderr)
        print("       请确保 cam_viewer.py 与 cam_viewer.html 放在同一目录。",
              file=sys.stderr)
        return 1

    if port_busy(args.host, args.port):
        print("[错误] 端口 %d 已被占用 —— 可能已经有一个 cam_viewer 在跑。"
              % args.port, file=sys.stderr)
        print("       查占用:  ss -lntp | grep %d" % args.port, file=sys.stderr)
        print("       停掉它:  pkill -f cam_viewer.py", file=sys.stderr)
        print("       换端口:  python3 %s --port %d"
              % (Path(__file__).name, args.port + 1), file=sys.stderr)
        return 1

    max_shots, per_target = load_gun_limits(args.config)
    print("  弹药上限: %d 发/全场, %s" % (
        max_shots, ("%d 发/目标" % per_target) if per_target > 0 else "单目标不限"))

    hub = Hub(args.topic, args.fps, args.quality, args.scale,
              max_shots=max_shots, per_target=per_target)

    rclpy.init()
    node = CamViewerNode(hub)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=hub.encode_loop, daemon=True).start()

    # ★遥控器：接管仲裁 + 四层失效保护（详见 rc_control.py 顶部说明）
    rc = rc_control.RemoteControl(node,
                                  pitch_offset_deg=load_pitch_offset(CONFIG_PATH))

    ip = lan_ip()
    print("=" * 62)
    print("  云台摄像头 · 浏览器弹窗")
    print("-" * 62)
    print("  本机:      http://%s:%d" % (ip, args.port))
    print("  本机回环:  http://127.0.0.1:%d" % args.port)
    print("  图像话题:  %s" % args.topic)
    print("  推流:      %.0f fps | JPEG 质量 %d | 缩放 %.2f"
          % (hub.stream_fps, hub.quality, hub.scale))
    print("  Ctrl+C 退出")
    print("=" * 62)

    print("  遥控器:    已启用（接管后可用，静默 %.0fs 自动释放）" % rc.WATCHDOG_S)
    print("=" * 62)

    app = create_app(hub, rc)
    try:
        app.run(host=args.host, port=args.port, threaded=True,
                debug=False, use_reloader=False)
    except KeyboardInterrupt:
        pass
    except SystemExit as exc:          # Flask 启动失败时会抛 SystemExit
        print("[错误] Web 服务启动失败：%s" % exc, file=sys.stderr)
        return 1
    finally:
        # ★ 这里刻意【不调用】 node.destroy_node() / rclpy.try_shutdown()：
        #   rclpy.spin 跑在独立线程里，主线程并发销毁节点会在 RMW 层触发
        #   abort —— 表现为 "terminate called without an active exception"
        #   加一个 core dump。进程马上退出，交给操作系统回收更安全。
        hub.stop()
        print("\n已退出。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
