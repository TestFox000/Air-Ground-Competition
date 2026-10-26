# -*- coding: utf-8 -*-
"""循迹调试工具链：规划线 / 真实轨迹 三线对齐（手机俯拍 = 真值）。

背景：循迹问题（画圆/对角漂移/切角/倒车/里程漂移）的定位方法只有一个——
把"规划器以为车在哪"与"车实际在哪"画在同一张场地图上。真值来源按优先级：

    1. 手机 + 三脚架俯拍 + 车顶 ArUco 码     ← 本工具 track（精度 ~1-2cm）
    2. 深度相机上车录 RGBD                    ← 记录滑移/倒车证据（复核用）
    3. 机上 0x4002（无人机看车的位姿）         ← 本工具 sniff（它是 belief 之一，
                                               全链测试时与真值对齐可验证 PnP 精度）

子命令：
    tag      生成车顶 ArUco 码（打印后贴车顶，正对上方）
    track    手机视频 -> 轨迹 CSV（4 点网格单应标定 + ArUco 自动跟踪）
    analyze  规划路线 (+轨迹 CSV) -> 叠加图 PNG + 逐航点接近/横向误差/
             倒车段/分段速度/雷格过心 报告
    sniff    笔记本侧 UDP:50000 记录器（0x4002 车位姿 / 0x4003 车命令）

坐标架：全部用 ag_integrate.field_frame 的本机世界坐标（米，x=列向 A→N，
y=行向 11→1，0=东、逆时针正），不引入第二套场地常量。

⚠️ sniff 只能在**笔记本**上跑：car/drone 上的决策进程刻意不设 SO_REUSEADDR，
同机第二份 bind 会 fail-fast（这是防线不是 bug）。

用法示例见 tools/README-循迹调试.md（E0~E6 实验阶梯）。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_SRC = _REPO / "src" / "ag_integrate"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from ag_integrate.field_frame import (  # noqa: E402
    CELL_SIZE, COLS, LETTERS, ROWS, X0, Y0, CellRef, cell_to_world,
)

# path_executor 到点半径（docs/执行层深优化-20261016 问题 3 同口径）
ARRIVAL_M = 0.157
# 排雷判分圆：任意轮中心进入雷标 12cm 圆
MINE_CIRCLE_M = 0.12
# 格号正则（两位行号分支在前，防 "N10" 被截成 "N1"）
_CELL_RE = re.compile(r"[A-N](?:1[01]|[1-9])")


# ============================================================ 纯几何核心（可单测）

def _cell_of(name: str) -> CellRef:
    m = _CELL_RE.fullmatch(name.strip().upper())
    if not m:
        raise ValueError(f"非法格号 {name!r}")
    return CellRef(LETTERS.index(name.strip().upper()[0]), int(name.strip().upper()[1:]) - 1)


def route_points(route_text: str) -> list[tuple[str, float, float, bool]]:
    """路线命令串 -> 逐格途经点 [(name, x, y, explicit)]。

    压缩路线只列拐点/必经点，相邻格必共线且为单位方向步整数倍（compress_route
    契约），gcd 展开补隐式途经格；'*' 尾缀与末尾朝向 token 由正则忽略。
    """
    pts: list[tuple[str, float, float, bool]] = []
    prev: tuple[int, int] | None = None
    for name in _CELL_RE.findall(route_text or ""):
        c = _cell_of(name)
        if prev is None:
            pts.append((c.name, *cell_to_world(c), True))
        else:
            dc, dr = c.col - prev[0], c.row - prev[1]
            g = math.gcd(abs(dc), abs(dr)) or 1
            for k in range(1, g + 1):
                cc = CellRef(prev[0] + dc * k // g, prev[1] + dr * k // g)
                pts.append((cc.name, *cell_to_world(cc), k == g))
        prev = (c.col, c.row)
    return pts


def _seg_dist(px: float, py: float, ax: float, ay: float,
              bx: float, by: float) -> tuple[float, float]:
    """点到线段距离与投影参数 t∈[0,1]。"""
    vx, vy = bx - ax, by - ay
    l2 = vx * vx + vy * vy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / l2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy)), t


def lateral_errors(traj: list[tuple]) -> list[tuple]:
    """每轨迹点到规划折线的垂距 + 所在段号。

    traj: [(t, x, y, yaw)] -> [(t, x, y, yaw, dist, seg_idx)]
    """
    out = []
    for t, x, y, yaw in traj:
        best = (float("inf"), -1)
        for si, (pa, pb) in enumerate(zip(_plan_pts, _plan_pts[1:])):
            d, _ = _seg_dist(x, y, pa[1], pa[2], pb[1], pb[2])
            if d < best[0]:
                best = (d, si)
        out.append((t, x, y, yaw, best[0], best[1]))
    return out


def waypoint_approach(traj: list[tuple], pts: list[tuple]) -> list[dict]:
    """显式航点的最近接近：dmin<=ARRIVAL_M 记"到达"（到点切换的物理判据）。"""
    res = []
    for idx, (name, x, y, explicit) in enumerate(pts):
        if not explicit:
            continue
        best = min(traj, key=lambda r: math.hypot(r[1] - x, r[2] - y))
        res.append({
            "idx": idx, "cell": name,
            "dmin": math.hypot(best[1] - x, best[2] - y),
            "t": best[0],
            "arrived": math.hypot(best[1] - x, best[2] - y) <= ARRIVAL_M,
        })
    return res


def reverse_episodes(traj: list[tuple], cos_th: float = 0.3) -> list[dict]:
    """倒车段：位移方向与车头夹角 >107°（cos<-0.3）的连续片段。

    E4 倒车验证的核心判据——真倒车时车头朝向不变、位置沿来路退；
    若实为画圈掉头，朝向会连续翻转，此函数给出的倒车距离会很小。
    """
    eps: list[dict] = []
    cur: dict | None = None
    for k in range(1, len(traj)):
        _, x0, y0, _ = traj[k - 1]
        t1, x1, y1, yaw1 = traj[k]
        step = math.hypot(x1 - x0, y1 - y0)
        if step < 1e-4:
            continue
        fwd = math.cos(math.atan2(y1 - y0, x1 - x0) - yaw1)
        if fwd < -cos_th:
            if cur is None:
                cur = {"t_start": t1, "t_end": t1, "dist": step}
            else:
                cur["t_end"] = t1
                cur["dist"] += step
        else:
            if cur is not None and cur["dist"] > 0.05:
                eps.append(cur)
            cur = None
    if cur is not None and cur["dist"] > 0.05:
        eps.append(cur)
    return eps


def segment_speeds(traj: list[tuple], pts: list[tuple],
                   approaches: list[dict]) -> list[dict]:
    """按"已到达航点"切段的实际速度（供 time_cruise_mps 等回填标定）。"""
    arrived = sorted(
        (a for a in approaches if a["arrived"]), key=lambda a: a["t"])
    out = []
    for a, b in zip(arrived, arrived[1:]):
        plan_len = sum(
            math.hypot(pts[i + 1][1] - pts[i][1], pts[i + 1][2] - pts[i][2])
            for i in range(a["idx"], b["idx"]))
        dt = b["t"] - a["t"]
        if dt <= 0:
            continue
        out.append({
            "from": a["cell"], "to": b["cell"],
            "plan_len_m": round(plan_len, 3), "dt_s": round(dt, 2),
            "speed_mps": round(plan_len / dt, 3),
            "diag": _is_diag(pts[a["idx"]], pts[b["idx"]]),
        })
    return out


def _is_diag(pa: tuple, pb: tuple) -> bool:
    """段内是否含对角步（起终点行列同时变化且非共线正交）。"""
    ca, cb = _cell_of(pa[0]), _cell_of(pb[0])
    return ca.col != cb.col and ca.row != cb.row


_plan_pts: list[tuple] = []   # lateral_errors 用（analyze 注入，避免全局态扩散）


# ============================================================ 叠加图

def draw_overlay(pts, traj, mines, shoot, out_png: Path,
                 reverse_mask=None) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(11, 9), dpi=150)
    # 场地网格（线 i=0..14 / j=0..11：格心 ± 半格）
    for i in range(COLS + 1):
        x = X0 + (i - 0.5) * CELL_SIZE
        ax.plot([x, x], [Y0 - 0.5 * CELL_SIZE, Y0 + (ROWS - 0.5) * CELL_SIZE],
                color="0.85", lw=0.6, zorder=0)
    for j in range(ROWS + 1):
        y = Y0 + (ROWS - 0.5 - j) * CELL_SIZE
        ax.plot([X0 - 0.5 * CELL_SIZE, X0 + (COLS - 0.5) * CELL_SIZE],
                [y, y], color="0.85", lw=0.6, zorder=0)
    for c in range(COLS):
        for r in range(ROWS):
            ax.text(X0 + c * CELL_SIZE, Y0 + (ROWS - 1 - r) * CELL_SIZE,
                    f"{LETTERS[c]}{r + 1}", ha="center", va="center",
                    fontsize=3.2, color="0.7", zorder=1)
    # 规划线
    ax.plot([p[1] for p in pts], [p[2] for p in pts], "-", color="#1f77b4",
            lw=1.8, label="plan", zorder=3)
    ax.plot([p[1] for p in pts if p[3]], [p[2] for p in pts if p[3]], "o",
            color="#1f77b4", ms=5, zorder=4)
    for name in shoot or []:
        c = _cell_of(name)
        w = cell_to_world(c)
        ax.plot(w[0], w[1], "*", color="orange", ms=13, zorder=5)
    for name in mines or []:
        c = _cell_of(name)
        w = cell_to_world(c)
        ax.add_patch(plt.Circle(w, MINE_CIRCLE_M, color="red", fill=False,
                                lw=1.2, zorder=5))
    # 真实轨迹
    if traj:
        ts = [r[0] for r in traj]
        sc = ax.scatter([r[1] for r in traj], [r[2] for r in traj], c=ts,
                        cmap="viridis", s=4, zorder=6, label="actual")
        fig.colorbar(sc, ax=ax, label="t [s]", shrink=0.7)
        if reverse_mask:
            ax.scatter([r[1] for r, m in zip(traj, reverse_mask) if m],
                       [r[2] for r, m in zip(traj, reverse_mask) if m],
                       marker="x", color="red", s=26, zorder=7, label="reverse")
    ax.set_aspect("equal")
    ax.set_xlabel("world x [m]  (A -> N)")
    ax.set_ylabel("world y [m]  (row 11 -> 1)")
    ax.set_title("plan vs actual")
    ax.legend(loc="upper right", fontsize=8)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, bbox_inches="tight")
    plt.close(fig)


# ============================================================ 子命令：analyze

def _load_traj(path: Path) -> list[tuple]:
    traj = []
    with path.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("x") in (None, ""):
                continue
            traj.append((float(row["t"]), float(row["x"]), float(row["y"]),
                         float(row.get("yaw", 0.0))))
    if len(traj) < 2:
        raise SystemExit(f"轨迹太短（{len(traj)} 点）: {path}")
    return traj


def cmd_analyze(args) -> int:
    mines: list[str] = []
    shoot: list[str] = []
    if args.route_plan:
        payload = json.loads(Path(args.route_plan).read_text(encoding="utf-8"))
        route_text = " ".join(payload.get("route", []))
        mines = [m for m in payload.get("snapshot", {}).get("mines", [])]
        shoot = list(payload.get("car_cmds", {}).get("shoot_cells", []))
    else:
        route_text = args.route or ""
        if args.mines:
            mines = [m.strip() for m in args.mines.split(",") if m.strip()]
    if not route_text:
        raise SystemExit("需要 --route 'N10 M9 ...' 或 --route-plan route_plan.json")

    global _plan_pts
    _plan_pts = pts = route_points(route_text)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    traj = _load_traj(Path(args.traj)) if args.traj else []
    rev_mask = None
    lines: list[str] = []

    if traj:
        lat = lateral_errors(traj)
        dists = [r[4] for r in lat]
        dists_sorted = sorted(dists)
        p95 = dists_sorted[max(0, int(len(dists) * 0.95) - 1)]
        approaches = waypoint_approach(traj, pts)
        missed = [a["cell"] for a in approaches if not a["arrived"]]
        revs = reverse_episodes(traj)
        rev_mask = [
            any(e["t_start"] <= r[0] <= e["t_end"] + 0.2 for e in revs)
            for r in traj]
        speeds = segment_speeds(traj, pts, approaches)
        ortho = [s["speed_mps"] for s in speeds if not s["diag"]]
        diag = [s["speed_mps"] for s in speeds if s["diag"]]

        lines.append(f"轨迹点 {len(traj)}，时长 {traj[-1][0] - traj[0][0]:.1f}s")
        lines.append(
            f"横向误差（对规划折线）: mean {sum(dists)/len(dists)*100:.1f}cm | "
            f"max {max(dists)*100:.1f}cm | p95 {p95*100:.1f}cm")
        lines.append(
            f"航点到达（阈值 {ARRIVAL_M*100:.0f}cm）: "
            f"{len(approaches)-len(missed)}/{len(approaches)}"
            + (f"，未到达 {missed}" if missed else ""))
        for a in approaches:
            lines.append(f"  {a['cell']:>4}  dmin {a['dmin']*100:5.1f}cm "
                         f"t {a['t']:6.2f}s  {'OK' if a['arrived'] else 'MISS'}")
        for name in mines:
            c = cell_to_world(_cell_of(name))
            dmin = min(math.hypot(r[1]-c[0], r[2]-c[1]) for r in traj)
            lines.append(f"雷格 {name}: 轨迹距格心最近 {dmin*100:.1f}cm "
                         f"({'过心OK' if dmin <= MINE_CIRCLE_M else '未进12cm判分圆!'})")
        if revs:
            total = sum(e["dist"] for e in revs)
            lines.append(f"倒车段 {len(revs)} 段共 {total*100:.0f}cm: "
                         + "; ".join(f"[{e['t_start']:.1f}~{e['t_end']:.1f}s "
                                     f"{e['dist']*100:.0f}cm]" for e in revs))
        else:
            lines.append("倒车段: 无（若路线含回头段而此处为无 → 实车在画圈掉头，"
                         "关 reverse_gear）")
        if speeds:
            med = sorted(s["speed_mps"] for s in speeds)[len(speeds)//2]
            lines.append(f"分段速度: 中位 {med:.2f} m/s（正交 "
                         f"{(sum(ortho)/len(ortho)) if ortho else 0:.2f} / 对角 "
                         f"{(sum(diag)/len(diag)) if diag else 0:.2f}）"
                         f"→ time_cruise_mps 回填参考 {med:.2f}")
            for s in speeds:
                lines.append(f"  {s['from']}→{s['to']}  {s['plan_len_m']:.2f}m / "
                             f"{s['dt_s']:.2f}s = {s['speed_mps']:.2f} m/s "
                             f"{'(对角)' if s['diag'] else ''}")
        if mines and args.traj:
            pass  # 雷格明细已在上面逐条输出
    else:
        lines.append(f"仅画规划线: {len(pts)} 途经格 "
                     f"({sum(1 for p in pts if p[3])} 显式航点)")

    draw_overlay(pts, traj, mines, shoot, out_dir / "overlay.png", rev_mask)
    report = "\n".join(lines)
    (out_dir / "report.txt").write_text(report + "\n", encoding="utf-8")
    print(report)
    print(f"\n产物: {out_dir/'overlay.png'} / {out_dir/'report.txt'}")
    return 0


# ============================================================ 子命令：track

def _pick_points(frame, count: int, title: str) -> list[tuple[int, int]]:
    import cv2
    pts: list[tuple[int, int]] = []
    state = {"done": False}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < count:
            pts.append((x, y))

    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(title, on_mouse)
    while len(pts) < count and not state["done"]:
        disp = frame.copy()
        for i, p in enumerate(pts):
            cv2.circle(disp, p, 8, (0, 0, 255), 2)
            cv2.putText(disp, str(i + 1), (p[0] + 10, p[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv2.putText(disp, f"click {count - len(pts)} more (grid corners TL,TR,BR,BL)",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow(title, disp)
        if cv2.waitKey(50) in (27, ord("q")):
            raise SystemExit("标定取消")
    cv2.destroyWindow(title)
    return pts


def _line_world(i: int, j: int) -> tuple[float, float]:
    """网格线交点 (i=0..14 竖线, j=0..11 横线, 0=左/上) 的世界坐标。"""
    return (X0 + (i - 0.5) * CELL_SIZE, Y0 + (ROWS - 0.5 - j) * CELL_SIZE)


def cmd_track(args) -> int:
    import cv2
    import numpy as np

    video = Path(args.video)
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise SystemExit(
            f"打不开视频 {video}（中文路径用英文重命名；或转 MP4 后重试）")
    fps = args.fps_override or cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    print(f"视频 {fps:.1f}fps, {total} 帧")
    if args.start_sec:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(args.start_sec * fps))

    # 标定帧：取分析区间中点附近一帧（起点太近可能没拍到网格全貌）
    ok, frame = cap.read()
    if not ok:
        raise SystemExit("读不到首帧")
    px = _pick_points(frame, 4, "calibration: click 4 grid intersections TL,TR,BR,BL")
    ij = input("输入该矩形四角的网格线编号 i0 j0 i1 j1（如 0 0 14 11）: ").split()
    if len(ij) != 4:
        raise SystemExit("需要 4 个整数")
    i0, j0, i1, j1 = map(int, ij)
    world_px = [_line_world(i0, j0), _line_world(i1, j0),
                _line_world(i1, j1), _line_world(i0, j1)]
    H, _ = cv2.findHomography(
        np.array(px, dtype=np.float32), np.array(world_px, dtype=np.float32))
    reproj = cv2.perspectiveTransform(
        np.array(px, dtype=np.float32).reshape(-1, 1, 2), H).reshape(-1, 2)
    err = max(abs(reproj[k][0] - world_px[k][0]) +
              abs(reproj[k][1] - world_px[k][1]) for k in range(4))
    print(f"单应重投影最大误差 {err*1000:.1f}mm（>5mm 说明点没点准，重跑）")

    dic = cv2.aruco.getPredefinedDictionary(getattr(
        cv2.aruco, f"DICT_{args.tag_dict.upper()}"))
    params = cv2.aruco.DetectorParameters()
    rows = []
    fidx = max(0, int(args.start_sec * fps)) - 1
    while True:
        for _ in range(max(1, args.step)):
            ok, frame = cap.read()
            fidx += 1
            if not ok:
                cap.release()
                _write_traj(rows, Path(args.out))
                return 0
            t = fidx / fps
            if args.end_sec and t > args.end_sec:
                cap.release()
                _write_traj(rows, Path(args.out))
                return 0
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), dic, parameters=params)
        if ids is None:
            continue
        for k in range(len(ids)):
            if args.tag_id is not None and int(ids[k][0]) != args.tag_id:
                continue
            quad = corners[k][0]           # TL,TR,BR,BL（图像坐标）
            wpts = cv2.perspectiveTransform(
                quad.reshape(-1, 1, 2).astype(np.float32), H).reshape(-1, 2)
            cx = float(wpts[:, 0].mean()); cy = float(wpts[:, 1].mean())
            yaw = math.atan2(float(wpts[1][1] - wpts[0][1]),
                             float(wpts[1][0] - wpts[0][0]))
            rows.append((fidx / fps, round(cx, 4), round(cy, 4),
                         round(yaw, 4), int(ids[k][0])))
    return 0


def _write_traj(rows, out: Path) -> None:
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "yaw", "tag_id"])
        w.writerows(rows)
    dt = rows[-1][0] - rows[0][0] if len(rows) > 1 else 0
    print(f"轨迹 {len(rows)} 点 / {dt:.1f}s -> {out}"
          + ("" if rows else "（0 点：检查 ArUco 是否打印清晰/尺寸够大）"))


# ============================================================ 子命令：tag / sniff

def cmd_tag(args) -> int:
    import cv2
    dic = cv2.aruco.getPredefinedDictionary(getattr(
        cv2.aruco, f"DICT_{args.tag_dict.upper()}"))
    gen = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    img = gen(dic, args.id, args.size_mm * 12)
    # 外扩白边（提高边缘检出率）；黑边线便于核对打印比例
    pad = args.size_mm * 3
    out = cv2.copyMakeBorder(img, pad, pad, pad, pad, cv2.BORDER_CONSTANT,
                             value=255)
    out_path = Path(args.out)
    ok, buf = cv2.imencode(".png", out)
    out_path.write_bytes(buf.tobytes())
    print(f"ArUco {args.tag_dict} id={args.id} -> {out_path} "
          f"(打印时 100% 缩放，实测码宽应为 {args.size_mm}mm)")
    return 0


def cmd_sniff(args) -> int:
    import socket
    from ag_integrate.intel_protocol import (
        MSG_ID_CAR_COMMAND, detect_msg_id, parse, parse_car_command, parse_car_pose,
    )
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", args.port))
    sock.settimeout(0.5)
    rows: list[list] = []
    counts = {"intel": 0, "pose": 0, "cmd": 0, "bad": 0}
    print(f"监听 UDP:{args.port}（Ctrl-C 结束并落盘）——只能在笔记本跑，"
          "机载/车端同端口 bind 会 fail-fast")
    import time as _time
    t0 = _time.monotonic()
    try:
        while True:
            if args.duration and _time.monotonic() - t0 > args.duration:
                break
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            recv_mono = _time.monotonic()
            mid = detect_msg_id(data)
            try:
                if mid == MSG_ID_CAR_COMMAND:
                    cmd = parse_car_command(data)
                    rows.append([recv_mono, "cmd", "", "", "",
                                 cmd.reset, cmd.route])
                    counts["cmd"] += 1
                elif mid == 0x4002:
                    p = parse_car_pose(data)
                    rows.append([recv_mono, "pose", p.x, p.y, p.yaw, "", ""])
                    counts["pose"] += 1
                elif mid is not None:
                    parse(data)
                    counts["intel"] += 1
                    if counts["intel"] % 50 == 0:
                        rows.append([recv_mono, "intel", "", "", "", "", ""])
                else:
                    counts["bad"] += 1
            except ValueError:
                counts["bad"] += 1
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_mono", "type", "x", "y", "yaw", "reset", "route"])
        w.writerows(rows)
    print(f"收帧: {counts} -> {args.out}")
    return 0


# ============================================================ main

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("tag", help="生成车顶 ArUco 码")
    p.add_argument("--id", type=int, default=0)
    p.add_argument("--size-mm", type=int, default=80)
    p.add_argument("--tag-dict", default="4X4_50")
    p.add_argument("--out", default="car_tag.png")
    p.set_defaults(fn=cmd_tag)

    p = sub.add_parser("track", help="手机视频 -> 轨迹 CSV")
    p.add_argument("video")
    p.add_argument("--out", default="traj.csv")
    p.add_argument("--tag-id", type=int, default=None)
    p.add_argument("--tag-dict", default="4X4_50")
    p.add_argument("--start-sec", type=float, default=0.0)
    p.add_argument("--end-sec", type=float, default=0.0)
    p.add_argument("--step", type=int, default=2, help="每 N 帧处理 1 帧")
    p.add_argument("--fps-override", type=float, default=0.0)
    p.set_defaults(fn=cmd_track)

    p = sub.add_parser("analyze", help="规划线(+轨迹) -> 叠加图与指标报告")
    p.add_argument("--route", default="", help="路线文本（车端方言，如 'N10 M9 N5,x'）")
    p.add_argument("--route-plan", default="", help="route_plan.json（route/mines/shoot 自动读）")
    p.add_argument("--traj", default="", help="track 产出的 traj.csv")
    p.add_argument("--mines", default="", help="雷格格号，逗号分隔（覆盖 route-plan）")
    p.add_argument("--out", default="track_out")
    p.set_defaults(fn=cmd_analyze)

    p = sub.add_parser("sniff", help="笔记本侧 UDP:50000 记录器")
    p.add_argument("--port", type=int, default=50000)
    p.add_argument("--out", default="sniff.csv")
    p.add_argument("--duration", type=float, default=0.0, help="秒数，0=Ctrl-C 结束")
    p.set_defaults(fn=cmd_sniff)

    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    raise SystemExit(main())
