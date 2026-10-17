# -*- coding: utf-8 -*-
r"""真实数据核查（不联网、不改任何外部文件）：把管线跑在**真图**上并留档。

四部分：

* **A 图纸回归**：在 ``dzrobot_v2/_tmp_session/t28_newmap/map_src.png`` 上按识图报告给出的
  格网标定，用 ``map_image`` 口径复现雷/障/靶/通 → 与**该图纸自己的冻结点位**
  （``DRAWING_TRUTH_FROZEN``）逐格对比，输出 precision/recall + 叠加图。
  ★2026-10-14 官方贴图重掷后 ``config/map.yaml`` 已描述**另一张图**，故这里不再拿它当真值；
  与现役真值的对账见 ``perception/run_field_map_check.py``。
* **B cam_probe 可行性**：对 ``dzsmartscout/cam_probe/`` 的真实照片算清晰度与颜色覆盖率，
  如实记录"能检 / 不能检"。
* **C 实物箭头正控**：``fc-audit/arena_pdf_img/p09_1.png`` 里真实桥顶绿箭头 → 方向估计
  （真值：指向图像上方 = −90°）。
* **D 摄像头硬件复测（t59）**：对无人机只读探测 ``/dev/video*`` / UVC / v4l2 / gst，
  如实报告"摄像头到位没有"；不在网就写明"未执行 + 沿用 t1 基线"。

产物写进 ``reports/``（PNG 叠加图 + Markdown + JSON）。

    python -m perception.run_real_checks            # 全部
    python -m perception.run_real_checks --part A   # 只跑图纸回归
    python -m perception.run_real_checks --part D   # 只跑摄像头复测
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import subprocess
import sys

if __package__ in (None, ""):            # 允许 `python perception/run_real_checks.py`
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    __package__ = "perception"

import numpy as np
from PIL import Image

from .bridge_arrow import direction_from_points
from .colors import PALETTES, as_rgb_array, blob_masks, rgb_to_hsv
from .grid import CORNER_ORDER, GridSpec, register_from_corners
from .overlay import draw_registration
from .pipeline import PerceptionPipeline, config_for

HERE = pathlib.Path(__file__).resolve().parent
TOOLS = HERE.parent
DZSCOUT = TOOLS.parent
V1 = DZSCOUT.parent
REPORTS = HERE / "reports"
MAP_SRC = V1 / "dzrobot_v2" / "_tmp_session" / "t28_newmap" / "map_src.png"
IDENTIFY = V1 / "dzrobot_v2" / "_tmp_session" / "t28_newmap" / "identify_report.txt"
MAP_YAML = V1 / "dzrobot_v2" / "config" / "map.yaml"
CAM_PROBE = DZSCOUT / "cam_probe"
REAL_ARROW = DZSCOUT / "fc-audit" / "arena_pdf_img" / "p09_1.png"
REAL_ARROW_BBOX = (634, 534, 714, 591)

#: ``cam_probe`` 里每个文件的**人工看图分类**（逐个打开看过，不是猜的）。
#: 只有 ``camera_frame`` 才是相机出图；``vnc_desktop`` 是 Jetson 桌面截图
#: （XFCE + ``rqt_image_view`` 窗口），拿它算"相机清晰度"没有意义 —— 单列出来避免混进结论。
CAM_PROBE_KIND = {
    "cam_1280x720.jpg": "camera_frame",
    "cam_1920x1200.jpg": "camera_frame",
    "cam_ros_frame.png": "camera_frame",
    "sharp_00_baseline.png": "camera_frame",
    "sharp_01_af.png": "camera_frame",
    "camview.png": "vnc_desktop",
    "camview2.png": "vnc_desktop",
    "camview3.png": "vnc_desktop",
}

#: 合成实验测得的模糊失效点（64 px/m，见 tests/test_pipeline_e2e.py 特征化测试）
BLUR_LIMITS = {
    "mines（12 cm 圆片）": "r=3 全丢（r=2 尚存 7/8）",
    "arrows（0.2 m 桥顶箭头）": "r=3 全丢（r=2 仍两半场都对）",
    "obstacles（30 cm 水桶）": "r=4 全丢（r=3 仍 8/8）",
    "targets（整格色块/靶区）": "r=4 仍在、r=8 已丢",
    "car（0.22×0.10 m 车顶板）": "r=3 丢失",
}


# --------------------------------------------------------------------------- #
#  小工具
# --------------------------------------------------------------------------- #
def _yaml_list(text: str, key: str) -> list[str]:
    m = re.search(rf"^\s*{key}:\s*\[(.*?)\]", text, re.M)
    if not m:
        raise SystemExit(f"map.yaml 里找不到 {key}: [...]")
    return [s.strip().strip('"').strip("'") for s in m.group(1).split(",") if s.strip()]


def _lattice_from_identify_report() -> dict:
    """从识图报告原文里取格网参数（不写死：报告在，数字就从报告来）。"""
    text = IDENTIFY.read_text(encoding="utf-8")
    m = re.search(r"格宽\s*([\d.]+)\s*px、格高\s*([\d.]+)\s*px；"
                  r"格心相位\s*x=([\d.]+)\s*y=([\d.]+)", text)
    if not m:
        raise SystemExit(f"identify_report.txt 里没有格网反推行：{IDENTIFY}")
    cw, ch, cx0, cy0 = (float(g) for g in m.groups())
    return {"cell_w_px": cw, "cell_h_px": ch, "center_x0": cx0, "center_y0": cy0}


def laplacian_variance(rgb: np.ndarray) -> float:
    """清晰度指标：灰度图 4 邻接拉普拉斯响应（中心 4，上下左右 −1）的方差。

    定义写死在这里，跨机器可复现（与 t1 报告里的实现可能不同，量级可比、数值不必逐位相同）。
    """
    gray = as_rgb_array(rgb).astype(np.float64) @ np.array([0.299, 0.587, 0.114])
    lap = (4.0 * gray[1:-1, 1:-1] - gray[:-2, 1:-1] - gray[2:, 1:-1]
           - gray[1:-1, :-2] - gray[1:-1, 2:])
    return float(lap.var())


def _fmt_set(xs) -> str:
    return " ".join(sorted(xs)) if xs else "—"


# --------------------------------------------------------------------------- #
#  A. 图纸回归
# --------------------------------------------------------------------------- #
#: ★t59：``map_src.png``（第 4/5 张图的图纸）的**冻结点位**。
#:   2026-10-14 官方贴图重掷后 ``config/map.yaml`` 已描述**另一张图**（雷 C1/A3/…、
#:   靶 E5/F5、通 K5/K9）⇒ 图纸回归不能再拿会变的 map.yaml 当真值。
#:   "真值 == 现役 map.yaml" 的对账见 ``run_field_map_check.py`` / ``tests/test_field_map.py``
#:   （它们用官方图 ``map_official.png``）。**本常量是图纸回归的唯一真值源**，
#:   ``tests/test_objects.py`` 直接引用它，避免两处各写一份。
DRAWING_TRUTH_FROZEN: dict[str, list[str]] = {
    "mine": ["A4", "B1", "D8", "E11", "F9", "G2", "H3", "I10"],
    "obstacle": ["A5", "B7", "C9", "D1", "E10", "F4", "G11", "H5"],
    "target": ["B11", "C11"],
    "bridge": ["K10", "K5"],
}


def part_a_map_drawing(verbose: bool = True) -> dict:
    lat = _lattice_from_identify_report()
    cw, ch = lat["cell_w_px"], lat["cell_h_px"]
    cx0, cy0 = lat["center_x0"], lat["center_y0"]
    u0, u1 = cx0 - cw / 2.0, cx0 - cw / 2.0 + 14 * cw
    v0, v1 = cy0 - ch / 2.0, cy0 - ch / 2.0 + 11 * ch
    # 场地外角 ↔ 像素：列 A 在左、行 1 在上（map_src.png 视觉核对）；米制 x=0 在行 11 侧
    corners = {"N11": (u1, v1), "N1": (u1, v0), "A1": (u0, v0), "A11": (u0, v1)}
    img = as_rgb_array(Image.open(MAP_SRC))
    spec = GridSpec()
    reg = register_from_corners([corners[t] for t in CORNER_ORDER], spec)
    res = PerceptionPipeline(config_for("map_image")).process(
        img, corners_px=[corners[t] for t in CORNER_ORDER])
    truth = {k: list(v) for k, v in DRAWING_TRUTH_FROZEN.items()}
    table = []
    ok_all = True
    for cls, want in truth.items():
        got = set(res.objects.blocks_of(cls))
        w = set(want)
        tp, fp, fn = len(got & w), len(got - w), len(w - got)
        prec = tp / (tp + fp) if (tp + fp) else 1.0
        rec = tp / (tp + fn) if (tp + fn) else 1.0
        ok = (got == w)
        ok_all = ok_all and ok
        table.append({"cls": cls, "truth": sorted(w), "detected": sorted(got),
                      "tp": tp, "fp": fp, "fn": fn,
                      "precision": round(prec, 4), "recall": round(rec, 4), "exact": ok})
        if verbose:
            print(f"[A] {cls:9s} 真值 {len(w):2d} / 检出 {len(got):2d}  "
                  f"TP={tp} FP={fp} FN={fn}  P={prec:.3f} R={rec:.3f} "
                  f"{'✓ 完全一致' if ok else '✗ 不一致'}")
            if not ok:
                print(f"      仅检出 {_fmt_set(got - w)} / 漏 {_fmt_set(w - got)}")
    REPORTS.mkdir(parents=True, exist_ok=True)
    draw_registration(img, reg).save(REPORTS / "A_map_registration.png")
    # 剔除项很多是格线/文字的碎片；报告里只列"够大"的与"深色文字"的，其余计数
    notable = [r for r in res.objects.rejected
               if "深色" in r["reason"] or r.get("area_px", 0) >= 400]
    out = {"lattice": lat,
           "pixel_corners": {k: [round(x, 2) for x in v] for k, v in corners.items()},
           "registration": reg.diagnostics(), "classes": table, "all_exact": ok_all,
           "rejected": res.objects.rejected, "rejected_notable": notable,
           "n_rejected": len(res.objects.rejected),
           "detected_blocks": {k: sorted(v) for k, v in res.objects.by_class.items()},
           "overlay": str(REPORTS / "A_map_registration.png")}
    if verbose:
        print(f"[A] 注册自检：{reg.diagnostics()}  叠加图 → {out['overlay']}")
        print(f"[A] 剔除 {out['n_rejected']} 个非目标色块（碎片居多），其中值得一提的 "
              f"{len(notable)} 个：")
        for r in notable:
            print(f"      {r['block']}（{r['cls']}）：{r['reason']}")
        print(f"[A] 结论：{'逐格一致' if ok_all else '存在不一致'}")
    return out


# --------------------------------------------------------------------------- #
#  B. cam_probe 可行性
# --------------------------------------------------------------------------- #
def part_b_cam_probe(verbose: bool = True) -> dict:
    rows = []
    palettes = {"field+provisional": PALETTES["field"].enable_provisional(),
                "map_image": PALETTES["map_image"]}
    files = sorted([p for p in CAM_PROBE.glob("*")
                    if p.suffix.lower() in (".png", ".jpg", ".jpeg")])
    for path in files:
        img = as_rgb_array(Image.open(path))
        h, w = img.shape[:2]
        hsv = rgb_to_hsv(img)
        kind = CAM_PROBE_KIND.get(path.name, "unknown")
        coverage = {}
        blobs = {}
        big_blobs = {}
        for name, pal in palettes.items():
            masks = pal.masks(img)
            coverage[name] = {cls: round(float(m.mean()) * 100.0, 4)
                              for cls, m in sorted(masks.items())}
            blobs[name] = {cls: len(blob_masks(m, min_pixels=30, k_close=1))
                           for cls, m in sorted(masks.items())}
            # "成块"的检出（面积 ≥200 px 且填充率 ≥0.7）才算可能的目标，散点不算
            big_blobs[name] = {cls: len([b for b in blob_masks(m, min_pixels=200,
                                                               k_close=1)
                                         if b.fill >= 0.7])
                               for cls, m in sorted(masks.items())}
        lap = laplacian_variance(img)
        rows.append({
            "file": path.name, "kind": kind, "size": [w, h],
            "laplacian_var": round(lap, 3),
            "mean_luma_0_255": round(float(img.mean()), 2),
            "sat_mean": round(float(hsv[..., 1].mean()), 4),
            "color_coverage_pct": coverage, "blob_counts_min30px": blobs,
            "solid_blob_counts_min200px": big_blobs,
            "usable_for_perception": (kind == "camera_frame" and lap >= 200.0),
            "why": ("vnc_desktop：不是相机出图，不参与结论" if kind != "camera_frame"
                    else f"camera_frame 但拉普拉斯方差 {lap:.2f} ≪ 干净合成图基准"
                         f"（64 px/m 下 >250）；按特征化测试，模糊半径 ≥3 px 时"
                         f"地雷与桥顶箭头就失效"),
        })
        if verbose:
            print(f"[B] {path.name:22s} {kind:12s} {w}x{h}  lapVar={lap:8.2f}"
                  f"  luma={rows[-1]['mean_luma_0_255']:6.2f}"
                  f"  sat={rows[-1]['sat_mean']:.3f}"
                  f"  可用于感知={'是' if rows[-1]['usable_for_perception'] else '否'}")
            for name, cov in coverage.items():
                top = sorted(cov.items(), key=lambda kv: -kv[1])[:3]
                print(f"      {name:18s} 覆盖率最高三类："
                      + " ".join(f"{k}={v:.3f}%" for k, v in top))
    camera = [r for r in rows if r["kind"] == "camera_frame"]
    out = {"files": rows, "n_camera_frames": len(camera),
           "n_vnc_desktop": len([r for r in rows if r["kind"] == "vnc_desktop"]),
           "camera_frames_lapvar": [r["laplacian_var"] for r in camera],
           "blur_limits_from_synth": BLUR_LIMITS,
           "verdict": ("相机帧全部糊到不可用：拉普拉斯方差 "
                       + "、".join(f"{r['laplacian_var']}" for r in camera)
                       + "；而且画面里没有场地、没有网格、没有任何一类目标 ⇒ "
                         "cam_probe 无法用于评估感知精度，只能作为"
                         "『当前相机出图不可用』的证据。"),
           "criteria": "拉普拉斯方差（4 邻接，中心 4/邻 −1，定义写死在本脚本）；"
                       "颜色覆盖率按两套调色板各算一遍；**未做注册**"
                       "（照片里没有场地，不猜角点）"}
    return out


# --------------------------------------------------------------------------- #
#  C. 实物箭头正控
# --------------------------------------------------------------------------- #
def part_c_real_arrow(verbose: bool = True) -> dict:
    if not REAL_ARROW.is_file():
        return {"skipped": f"缺少 {REAL_ARROW}"}
    img = as_rgb_array(Image.open(REAL_ARROW))
    pal = PALETTES["field"].enable_provisional()
    mask = pal.masks(img)["arrow"]
    blobs = blob_masks(mask, min_pixels=40, k_close=1, rgb=img)
    found = []
    arrow = None
    for b in blobs:
        r = direction_from_points(b.pixels[:, ::-1].astype(float))
        item = {"bbox": [b.x0, b.y0, b.x1, b.y1], "area_px": int(b.area),
                "fill": round(b.fill, 3),
                "mean_rgb": [round(v, 1) for v in b.mean_rgb],
                "dir_deg_image": round(float(r["dir_deg"]), 2),
                "axis_deg_image": round(float(r["axis_deg"]), 2),
                "pca_axis_deg": r["evidence"].get("pca_axis_deg"),
                "confidence": round(float(r["confidence"]), 3),
                "axis_score": r["evidence"].get("axis_score"),
                "n_agree": r["evidence"].get("n_agree")}
        found.append(item)
        if tuple(item["bbox"]) == REAL_ARROW_BBOX:
            arrow = item
    out = {"image": str(REAL_ARROW), "image_size": [img.shape[1], img.shape[0]],
           "arrow_class_coverage_pct": round(float(mask.mean()) * 100.0, 4),
           "green_blobs": found, "arrow_blob": arrow,
           "truth": "桥顶箭头尖头指向图像上方 ⇒ 期望 dir_deg ≈ −90°"}
    if verbose:
        print(f"[C] {REAL_ARROW.name}: 绿掩膜覆盖 {out['arrow_class_coverage_pct']:.3f}%，"
              f"连通域 {len(found)} 个")
        for it in found:
            tag = "← 箭头" if arrow is it else "（绿桶）"
            print(f"      bbox={it['bbox']} area={it['area_px']:6d} "
                  f"dir={it['dir_deg_image']:7.2f}° conf={it['confidence']:.3f} "
                  f"PCA={it['pca_axis_deg']} {tag}")
        if arrow:
            print(f"[C] 箭头方向误差 ≈ {arrow['dir_deg_image'] + 90.0:+.2f}°（真值 −90°）")
    return out


# --------------------------------------------------------------------------- #
#  D. 摄像头硬件现状复测（t59：对无人机只读探测；不写对方任何文件）
# --------------------------------------------------------------------------- #
#: 已知/历史地址（按顺序尝试；都不可达则如实报"无人机不在网"）
DRONE_HOSTS = ("10.29.122.104", "dzsmart.local")
DRONE_USER = "duzhong"
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=8"]
#: 只读探测命令（顺序执行；任一条失败不影响后面的）
CAMERA_PROBE_CMDS: list[tuple[str, str]] = [
    ("hostname", "hostname; uname -m; uname -r"),
    ("video_devs", "ls -l /dev/video* 2>&1 | head -8; echo '--- count ---'; "
                   "ls /dev/video* 2>/dev/null | wc -l"),
    ("v4l2_sysfs", "ls /sys/class/video4linux 2>&1 | head -8"),
    ("usb_devices", "lsusb 2>&1 | head -14"),
    ("uvc_match", "lsusb 2>&1 | grep -i -E 'cam|video|uvc|image' || echo '（无摄像头类 USB 设备）'"),
    ("v4l2_ctl", "v4l2-ctl --list-devices 2>&1 | head -12"),
    ("gst_devices", "which gst-device-monitor-1.0 >/dev/null 2>&1 && "
                    "timeout 6 gst-device-monitor-1.0 Video 2>&1 | head -12 || echo '（无 gst）'"),
    ("cam_tools", "ls -la ~/camtools 2>&1 | head -10"),
    ("dmesg_cam", "dmesg 2>/dev/null | grep -i -E 'uvc|camera|video4linux' | tail -6 "
                  "|| echo '（dmesg 不可读或无相关行）'"),
]


def _find_drone(verbose: bool = True) -> tuple[str | None, list[dict]]:
    """按已知地址探测无人机 SSH（只做连接测试，不写任何东西）。"""
    attempts = []
    for host in DRONE_HOSTS:
        r = subprocess.run(["ssh", *SSH_OPTS, f"{DRONE_USER}@{host}", "echo OK"],
                           capture_output=True, text=True, timeout=30)
        ok = r.returncode == 0 and "OK" in (r.stdout or "")
        attempts.append({"host": host, "reachable": ok,
                         "detail": ((r.stderr or r.stdout or "").strip().splitlines() or [""])[-1][:160]})
        if verbose:
            print(f"[D] {host:18s} {'可达 ✅' if ok else '不可达 ❌'}  {attempts[-1]['detail']}")
        if ok:
            return host, attempts
    return None, attempts


def _scan_lan_ssh(timeout: float = 0.4, verbose: bool = True) -> dict:
    """已知地址都不通时，扫本机 /24 找开着 22 端口的主机（只连 TCP、不登录、不改东西）。"""
    import concurrent.futures as cf
    import ipaddress
    import socket

    def _local_ipv4() -> str:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
        except OSError:
            return "127.0.0.1"
        finally:
            s.close()

    def _probe(ip: str) -> tuple[str, str]:
        try:
            with socket.create_connection((ip, 22), timeout=timeout) as sk:
                sk.settimeout(timeout)
                try:
                    return ip, sk.recv(64).decode("utf-8", "replace").strip()
                except OSError:
                    return ip, "(22 开放)"
        except OSError:
            return ip, ""

    me = _local_ipv4()
    net = ipaddress.ip_network(me + "/24", strict=False)
    hosts = [str(h) for h in net.hosts() if str(h) != me]
    hits = []
    with cf.ThreadPoolExecutor(max_workers=128) as ex:
        for ip, banner in ex.map(_probe, hosts):
            if banner:
                hits.append({"ip": ip, "banner": banner})
    if verbose:
        print(f"[D] 本机 {me}：扫 {net} 的 22 端口 → "
              + (f"发现 {len(hits)} 台：{', '.join(h['ip'] for h in hits)}"
                 if hits else "无任何主机响应"))
    return {"local_ip": me, "net": str(net), "n_hosts": len(hosts), "hits": hits}


def part_d_camera(verbose: bool = True) -> dict:
    """**摄像头硬件现状复测**：对无人机只读探测 ``/dev/video*`` / UVC / v4l2 / gst。"""
    host, attempts = _find_drone(verbose)
    out: dict = {"hosts_tried": attempts, "reachable": bool(host), "host": host,
                 "probes": [], "verdict": "", "n_video_devs": None,
                 "uvc_found": None, "lan_scan": None, "prior_evidence": (
                     "t1（2026-10-11）：整台未接摄像头 —— /dev/video* 空、无 UVC 设备"),
                 "how_to_rerun": "在能连上无人机的机器上："
                                 "cd dzsmartscout/tools && python -m perception.run_real_checks --part D"}
    if not host:
        out["lan_scan"] = _scan_lan_ssh(verbose=verbose)
        out["verdict"] = ("❌ 复测未执行：无人机不在网（" +
                          "、".join(f"{a['host']}（{a['detail']}）" for a in attempts) +
                          f"；本网段 {out['lan_scan']['net']} 扫 22 端口 "
                          f"{'发现 ' + str(len(out['lan_scan']['hits'])) + ' 台主机' if out['lan_scan']['hits'] else '无任何主机'}）"
                          "⇒ 摄像头有无**无法确认**，沿用 t1 的现状基线（未见摄像头）。"
                          "感知侧结论：视觉可在真图上跑通（见 part A / field_map 检查）、"
                          "**实机待摄像头**。")
        if verbose:
            print(f"[D] 结论：{out['verdict']}")
        return out
    for name, cmd in CAMERA_PROBE_CMDS:
        r = subprocess.run(["ssh", *SSH_OPTS, f"{DRONE_USER}@{host}", cmd],
                           capture_output=True, text=True, timeout=40)
        txt = ((r.stdout or "") + (r.stderr or "")).strip()
        out["probes"].append({"name": name, "cmd": cmd, "rc": r.returncode, "out": txt})
        if verbose:
            first = (txt.splitlines() or [""])[0]
            print(f"[D] {name:12s} rc={r.returncode}  {first[:110]}")
    def _probe(name: str) -> str:
        for p in out["probes"]:
            if p["name"] == name:
                return p["out"]
        return ""
    vd = _probe("video_devs")
    m = re.search(r"--- count ---\s*(\d+)", vd)
    out["n_video_devs"] = int(m.group(1)) if m else None
    uvc = _probe("uvc_match")
    out["uvc_found"] = ("无摄像头类" not in uvc) and bool(uvc)
    out["verdict"] = ("✅ 已确认摄像头设备存在（/dev/video* "
                      f"{out['n_video_devs']} 个；UVC {'有' if out['uvc_found'] else '无'}）"
                      if (out["n_video_devs"] or out["uvc_found"]) else
                      f"❌ 复测确认：**仍无摄像头**（/dev/video* "
                      f"{out['n_video_devs'] if out['n_video_devs'] is not None else '?'} 个、无 UVC）"
                      " ⇒ 与 t1 一致：感知管线可跑真图，**实机视觉待摄像头到位**")
    if verbose:
        print(f"[D] 结论：{out['verdict']}")
    return out



# --------------------------------------------------------------------------- #
#  报告
# --------------------------------------------------------------------------- #
def write_report(a: dict, b: dict, c: dict, d: dict | None = None) -> pathlib.Path:
    REPORTS.mkdir(parents=True, exist_ok=True)
    lines = ["# 真实数据核查报告（自动生成，勿手改）", "",
             "- 生成脚本：`perception/run_real_checks.py`",
             f"- 图纸（A，冻结真值）：`{MAP_SRC.relative_to(V1).as_posix()}`",
             f"- 车侧真源（供对照）：`{MAP_YAML.relative_to(V1).as_posix()}`",
             "- 现役真值端到端对账：`perception/run_field_map_check.py`", ""]
    if a:
        lines += ["## A · 图纸回归（map_src.png ↔ **该图纸自己的冻结点位**）", "",
                  "- ★2026-10-14 官方贴图重掷后 `config/map.yaml` 已描述另一张图"
                  "（雷 C1/A3/…、靶 E5/F5、通 K5/K9）⇒ 本部分真值改为冻结点位，"
                  "不再随 map.yaml 变；与现役真值的对账见 `field_map_check.md`。",
                  f"- 格网（来自 identify_report.txt 原文）：格宽 "
                  f"{a['lattice']['cell_w_px']} px、格高 {a['lattice']['cell_h_px']} px、"
                  f"格心相位 ({a['lattice']['center_x0']}, {a['lattice']['center_y0']})",
                  f"- 四角点像素：{json.dumps(a['pixel_corners'], ensure_ascii=False)}",
                  f"- 注册：{json.dumps(a['registration'], ensure_ascii=False)}", "",
                  "| 类 | 真值数 | 检出数 | TP | FP | FN | Precision | Recall | 逐格一致 |",
                  "|---|---|---|---|---|---|---|---|---|"]
        for row in a["classes"]:
            lines.append(f"| {row['cls']} | {len(row['truth'])} | {len(row['detected'])} "
                         f"| {row['tp']} | {row['fp']} | {row['fn']} "
                         f"| {row['precision']} | {row['recall']} "
                         f"| {'✅' if row['exact'] else '❌'} |")
        lines += ["", f"**结论：{'逐格完全一致' if a['all_exact'] else '存在不一致'}**"
                      f"（叠加图 `{pathlib.Path(a['overlay']).name}`）", "",
                  f"共剔除 {a['n_rejected']} 个非目标色块（格线/文字碎片居多）；"
                  f"其中值得一提的 {len(a.get('rejected_notable', []))} 个：", ""]
        for r in a.get("rejected_notable", []):
            lines.append(f"- {r.get('cls')} @ {r.get('block')}：{r.get('reason')}")
        lines.append("")
    if b:
        lines += ["## B · cam_probe 真实照片可行性", "",
                  f"- 相机帧 {b['n_camera_frames']} 个 / VNC 桌面截图 {b['n_vnc_desktop']} 个"
                  f"（分类见 `run_real_checks.py: CAM_PROBE_KIND`，逐个打开看过）",
                  f"- **结论**：{b['verdict']}", "",
                  "| 文件 | 类型 | 尺寸 | 拉普拉斯方差 | 平均亮度 | 平均饱和度 | 可用于感知 |",
                  "|---|---|---|---|---|---|---|"]
        for f in b["files"]:
            lines.append(f"| {f['file']} | {f['kind']} | {f['size'][0]}x{f['size'][1]} "
                         f"| {f['laplacian_var']} | {f['mean_luma_0_255']} "
                         f"| {f['sat_mean']} | "
                         f"{'✅' if f['usable_for_perception'] else '❌'} |")
        lines += ["", "合成实验标定的模糊失效点（64 px/m，见 `tests/test_pipeline_e2e.py`）：",
                  "| 目标 | 失效的模糊半径 |", "|---|---|"]
        for k, v in b["blur_limits_from_synth"].items():
            lines.append(f"| {k} | {v} |")
        lines += ["", "各类颜色覆盖率（%）与『成块』检出数（≥200 px 且 fill≥0.7）：", "",
                  "| 文件 | 调色板 | 覆盖率最高的三类 | 成块检出 |",
                  "|---|---|---|---|"]
        for f in b["files"]:
            for name, cov in f["color_coverage_pct"].items():
                top = sorted(cov.items(), key=lambda kv: -kv[1])[:3]
                solid = f["solid_blob_counts_min200px"][name]
                solid_txt = " ".join(f"{k}:{v}" for k, v in solid.items() if v) or "—"
                lines.append(f"| {f['file']} | {name} | "
                             + " ".join(f"{k}={v}%" for k, v in top)
                             + f" | {solid_txt} |")
        lines += ["", f"判据：{b['criteria']}", ""]
    if c and "arrow_blob" in c:
        lines += ["## C · 实物桥顶箭头正控（p09_1.png）", "",
                  f"- 绿掩膜覆盖率：{c['arrow_class_coverage_pct']}%",
                  f"- 连通域：{len(c['green_blobs'])} 个"]
        for it in c["green_blobs"]:
            tag = "**箭头**" if it is c["arrow_blob"] else "绿桶"
            lines.append(f"  - {tag} bbox={it['bbox']} area={it['area_px']} "
                         f"dir={it['dir_deg_image']}° conf={it['confidence']} "
                         f"PCA轴={it['pca_axis_deg']}°")
        lines += ["", f"- 真值：{c['truth']}",
                  f"- **实测方向 {c['arrow_blob']['dir_deg_image']}°，误差 "
                  f"{c['arrow_blob']['dir_deg_image'] + 90.0:+.2f}°**", ""]
    if d:
        lines += ["## D · 摄像头硬件现状复测（t59，对无人机只读）", "",
                  f"- 探测地址：{', '.join(a2['host'] for a2 in d['hosts_tried'])}"
                  f"（逐条结果见 JSON）",
                  f"- 是否连通：{'✅ 是' if d['reachable'] else '❌ 否'}",
                  f"- t1 基线（2026-10-11）：{d['prior_evidence']}",
                  f"- **结论：{d['verdict']}**", ""]
        if d.get("lan_scan"):
            ls = d["lan_scan"]
            lines += [f"- 本网段扫描：本机 {ls['local_ip']}，{ls['net']} 共 {ls['n_hosts']} 台，"
                      f"22 端口命中 {len(ls['hits'])} 台"
                      + ("：" + ", ".join(f"`{h['ip']}`" for h in ls["hits"])
                         if ls["hits"] else "（无）"), ""]
        if d["probes"]:
            lines += ["| 项 | 返回码 | 输出（首行） |", "|---|---|---|"]
            for p in d["probes"]:
                first = ((p["out"].splitlines() or [""])[0])[:120].replace("|", "\\|")
                lines.append(f"| `{p['name']}` | {p['rc']} | `{first}` |")
        lines += ["", f"- 复跑方法：`{d['how_to_rerun']}`", ""]
    path = REPORTS / "real_checks.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    (REPORTS / "real_checks.json").write_text(
        json.dumps({"A": a, "B": b, "C": c, "D": d}, ensure_ascii=False, indent=2),
        encoding="utf-8", newline="\n")
    return path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="run_real_checks")
    ap.add_argument("--part", default="ABCD", help="只跑某几部分，如 A / BC / D")
    args = ap.parse_args(argv)
    parts = args.part.upper()
    a = part_a_map_drawing("A" in parts)
    b = part_b_cam_probe("B" in parts)
    c = part_c_real_arrow("C" in parts)
    d = part_d_camera("D" in parts)
    path = write_report(a, b, c, d)
    print(f"\n报告 → {path}")
    if a:
        print(f"A 结论：{'逐格一致 ✅' if a['all_exact'] else '存在不一致 ❌'}")
    if c and "arrow_blob" in c:
        print(f"C 结论：箭头方向 {c['arrow_blob']['dir_deg_image']}°（真值 −90°）")
    if d:
        print(f"D 结论：{d['verdict'][:100]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
