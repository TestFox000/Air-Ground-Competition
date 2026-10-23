# -*- coding: utf-8 -*-
r"""桥顶通行箭头识别 → ``bridge_passable{half, block}``。

## 现场形态（证据）

* 说明书 p8/p10：「两处桥洞中由裁判随机指定可通行通道，**在桥梁顶部设置箭头标识**
  标明通行状态」；p8 还写明「地面机器人由于视角限制无法直接识别桥梁顶部设置对应标识，
  **必须由无人机完成高空侦察**」——这就是本模块存在的理由。
* 实物照片 ``fc-audit/arena_pdf_img/p09_1.png``：**黄色桥面板**上贴一张**白纸 +
  绿色箭头贴纸**；箭头贴纸在桥面**某一块板**上，尖头**指向穿洞方向**（沿列方向）。
* 赛道模型（``dzrobot_v2/config/map.yaml:33-35``）：「整个 K 列是一座桥，每格都是
  横向贯通的桥洞，格与格之间有侧壁」。所以**桥带长轴 = 行方向(x)**，
  **穿洞方向 = 列方向(y)**。

## 两个语义假设（都实现，默认取 H1）

* **H1 位置（默认）**：箭头贴在哪块板上，那块板对应的 K 列格就是该半场的通行格。
  依据：照片里箭头确实贴在**某一块**桥面板上，且尖头朝穿洞方向（不是沿桥带方向）。
* **H2 方向（可选，``infer_mode="direction"``）**：箭头是"沿桥带指向相邻通行格"的指示牌，
  则通行格 = 箭头所在格沿指向偏移 ``direction_steps`` 格。

无论哪种模式，``dir_kind`` 都会告诉你箭头是**沿桥带**还是**沿穿洞方向**——
H1 成立时应当看到 ``along_tunnel``（照片实测正是如此）。

## 方向估计怎么做（可复现）

在**场地米制系**里对箭头像素做 PCA 得到主轴；沿主轴切 ``n_bins`` 段，统计每段的
垂直宽度与点数，然后在两端各取 25% 的段做平均，得到**三条独立线索**：

1. ``votes_width``：尖端那一端更**窄**（尾杆宽度恒定，尖头收敛到 0）；
2. ``votes_count``：尖端那一端**点数更少**（既窄又短）；
3. ``votes_skew``：沿主轴的三阶矩，质量在杆侧 ⇒ 长尾朝尖。

三票多数决（宽度是主判据：1:2 且宽度指向 +u 时以宽度为准）；
``n_agree``=3/2 决定置信度折扣（1.0 / 0.8），再乘各向异性闸门。

**这个方法的上限在哪（实测）**：干净形状下方向误差 0°，噪声 σ ≤ 0.015 m（箭头长
0.2 m）时误差 ≤ 4.5°；σ ≥ 0.03 m 后三条线索一起失效，置信度塌到 0.14（会低于默认
闸门 0.30 从而不出结论）。但 σ=0.04 m 时曾出现"置信度 0.82 且方向翻转"的假自信
——所以**默认只用位置判据 H1 定通行格，方向只作为证据**：位置判据不受方向错误影响。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .colors import Palette, as_rgb_array, blob_masks
from .grid import ArenaRegistration, GridSpec

__all__ = [
    "ArrowSpec",
    "ArrowDetection",
    "BridgeCall",
    "direction_from_points",
    "detect_arrows",
    "infer_bridge_passable",
]


@dataclass(frozen=True)
class ArrowSpec:
    """箭头检测参数。"""

    cls: str = "arrow"
    bridge_col: str = "K"
    min_cell_frac: float = 0.015      # 面积 / 该格像素面积
    max_cell_frac: float = 1.20
    min_pixels: int = 10
    k_open: int = 0
    k_close: int = 1
    min_conf: float = 0.30            # 低于此值不产生 bridge_passable
    n_bins: int = 14                  # 主轴分段数（宽度剖面）
    infer_mode: str = "position"      # position(H1) | direction(H2)
    direction_steps: int = 1          # H2：沿指向偏移几格
    allow_off_column: bool = False    # 是否允许非 bridge_col 的箭头也参与


@dataclass
class ArrowDetection:
    block: str
    row_idx: int
    col_idx: int
    half: str
    x_m: float
    y_m: float
    area_px: int
    area_frac: float
    axis_deg_map: float               # 主轴角（度，场地米制系，∈[0,180)）
    dir_deg_map: float                # 尖头指向（度，场地米制系，(-180,180]）
    dir_confidence: float
    confidence: float
    dir_kind: str                     # along_strip(±x) / along_tunnel(±y)
    evidence: dict = field(default_factory=dict)

    def to_json(self) -> dict:
        return {"block": self.block, "half": self.half,
                "x_m": round(self.x_m, 3), "y_m": round(self.y_m, 3),
                "axis_deg_map": round(self.axis_deg_map, 1),
                "dir_deg_map": round(self.dir_deg_map, 1),
                "dir_kind": self.dir_kind,
                "confidence": round(self.confidence, 3),
                "dir_confidence": round(self.dir_confidence, 3),
                "area_px": self.area_px, "area_frac": round(self.area_frac, 3)}


@dataclass
class BridgeCall:
    """一个半场的通行格结论（对应车侧 ``bridge_passable``）。"""

    half: str
    block: str
    confidence: float
    dir_deg_map: float | None = None
    dir_kind: str = ""
    candidates: tuple[str, ...] = ()
    evidence: tuple[str, ...] = ()

    def to_json(self) -> dict:
        return {"half": self.half, "block": self.block,
                "confidence": round(self.confidence, 3),
                "dir_deg_map": (None if self.dir_deg_map is None
                                else round(self.dir_deg_map, 1)),
                "dir_kind": self.dir_kind, "candidates": list(self.candidates),
                "evidence": list(self.evidence)}


# --------------------------------------------------------------------------- #
#  方向估计（与"像素→米"解耦，方便直接喂像素做真实照片验证）
# --------------------------------------------------------------------------- #
def _profile_along(points_centered: np.ndarray, u: np.ndarray,
                   n_bins: int) -> tuple[np.ndarray, np.ndarray]:
    """沿 ``u`` 分段统计：返回 ``(稳健宽度, 点数)``，按投影 t 递增排列（只含非空段）。"""
    t = points_centered @ u
    s = points_centered @ np.array([-u[1], u[0]])
    lo, hi = float(t.min()), float(t.max())
    if hi - lo < 1e-12:
        return np.zeros(0), np.zeros(0)
    edges = np.linspace(lo, hi, int(n_bins) + 1)
    idx = np.clip(np.searchsorted(edges, t, side="right") - 1, 0, int(n_bins) - 1)
    w = np.zeros(int(n_bins), dtype=np.float64)
    c = np.zeros(int(n_bins), dtype=np.float64)
    for k in range(int(n_bins)):
        sel = s[idx == k]
        c[k] = sel.size
        if sel.size >= 4:
            # 稳健宽度 p95−p05：对少量离群像素不敏感，仍反映该段的横向跨度
            w[k] = float(np.percentile(sel, 95) - np.percentile(sel, 5))
        elif sel.size:
            w[k] = float(sel.max() - sel.min())
    occ = np.flatnonzero(c > 0)
    return w[occ], c[occ]


def _const_run(seq: np.ndarray, w_max: float, low_frac: float = 0.65,
               tol_frac: float = 0.15) -> int:
    """从该端起，最长**等宽且明显窄于最宽处**的连续段长度（= 箭杆的特征）。"""
    k = 1
    while k < seq.size and seq[k] <= low_frac * w_max:
        seg = seq[:k + 1]
        if float(seg.max() - seg.min()) <= tol_frac * max(float(seg.mean()), 1e-12):
            k += 1
        else:
            break
    return k


def _mono_run(seq: np.ndarray, w_max: float, tol_frac: float = 0.05,
              reach_frac: float = 0.85) -> int:
    """从该端起最长**非减**连续段；要求总涨幅显著**且真正张到接近最宽处**。

    "必须张到头部"这条不能省：否则一段纯噪声的缓升也会被当成"张开"。
    """
    if seq.size < 2:
        return 1
    tol = max(1e-12, float(seq.max()) * tol_frac)
    k = 1
    while k < seq.size and seq[k] >= seq[k - 1] - tol:
        k += 1
    seg_max = float(seq[:k].max())
    if (seg_max - float(seq[0])) >= tol and seg_max >= reach_frac * w_max:
        return k
    return 1


def _axis_evidence(p: np.ndarray, u: np.ndarray, n_bins: int) -> dict:
    """给定轴 ``u`` 的结构证据：**一端是等宽箭杆，另一端单调张开到头部**。

    这是把"箭头轴"和"横向轴"分开的关键（两者都有窄端，光看端部宽度分不开）：

    * 箭头轴：一端 ≈ 等宽低平台（箭杆），另一端从尖端单调张开到最宽（头部）；
    * 横向轴：两端镜像对称，端部是**渐变的翼尖**（既不等宽也不张开），或中部隆起。

    实测（实物照片 p09_1.png 的绿箭头）：

    | 轴 | 归一化宽度剖面 | 判据 |
    |---|---|---|
    | **90°（真值）** | 0.16 0.30 0.47 0.64 0.82 0.92 0.97 1.00 \\| 0.66 0.63 0.28 0.28 0.29 0.28 | 杆 4 段 + 张开 8 段 ✓ |
    | 0° | 0.27 0.29 0.29 0.28 0.31 0.94 … 0.24 0.24 0.25 0.24 | **两端都像杆** ⇒ 歧义，扣分 |
    | 60° | 0.73 0.84 0.92 1.00 0.55 … 0.84 0.15 | 端部渐变，无杆 ⇒ 0 分 |

    合成箭头的同一判据：真轴 (杆 7 段 + 张开 6 段) vs 横轴 (杆 1 段 ⇒ 0 分)。
    """
    w, c = _profile_along(p - p.mean(axis=0), u, n_bins)
    if w.size < 4:
        return {"score": 0.0, "ok": False, "n_occ": int(w.size)}
    w_max = float(w.max())
    if w_max <= 1e-12:
        return {"score": 0.0, "ok": False, "n_occ": int(w.size)}
    n = int(w.size)
    lo_const = _const_run(w, w_max)
    hi_const = _const_run(w[::-1], w_max)
    lo_mono = _mono_run(w, w_max)
    hi_mono = _mono_run(w[::-1], w_max)
    pairs = []                                   # (杆长, 张开长, 杆在哪端)
    if hi_const >= 2 and lo_mono >= 2:
        pairs.append((hi_const, lo_mono, "hi"))
    if lo_const >= 2 and hi_mono >= 2:
        pairs.append((lo_const, hi_mono, "lo"))
    both_shafts = (lo_const >= 2 and hi_const >= 2)
    w_min = float(w.min())
    contrast = (w_max / w_min) if w_min > 1e-9 else 99.0
    if pairs:
        shaft, mono, side = max(pairs, key=lambda t: t[0] + t[1])
        score = (shaft + mono) / (2.0 * n)
        if both_shafts:
            score *= 0.5                          # 两端都像杆 ⇒ 这不是箭头轴
        if contrast < 1.5:
            score *= 0.5
    else:
        shaft, mono, side = 0, 0, ""
        score = 0.0
    i_min_w = int(np.argmin(w))
    i_min_c = int(np.argmin(c))
    end_tol = 1
    end_pts = (1 if (i_min_w <= end_tol or i_min_w >= n - 1 - end_tol) else 0) \
        + (1 if (i_min_c <= end_tol or i_min_c >= n - 1 - end_tol) else 0)
    return {"score": float(score), "ok": True, "n_occ": n, "w": w, "c": c,
            "contrast": float(contrast), "shaft": int(shaft), "mono": int(mono),
            "shaft_side": side, "lo_const": int(lo_const), "hi_const": int(hi_const),
            "lo_mono": int(lo_mono), "hi_mono": int(hi_mono),
            "both_shafts": bool(both_shafts), "end_pts": int(end_pts),
            "i_min_w": i_min_w, "i_min_c": i_min_c}


def _rel(a: float, b: float) -> float:
    denom = max(abs(a), abs(b))
    return float(abs(a - b) / denom) if denom > 1e-12 else 0.0


def _tip_side(u: np.ndarray, ev: dict, points_centered: np.ndarray) -> dict:
    """尖端在哪一端：**结构判据（杆在另一端）为主**，三线索投票只作一致性校验。"""
    w, c = ev["w"], ev["c"]
    n = w.size
    n_end = max(1, int(round(0.15 * n)))          # ★ 端部窗口要小：箭头头部比杆宽得多，
    w_lo, w_hi = float(w[:n_end].mean()), float(w[-n_end:].mean())   # 取 25% 会把头部最宽处
    c_lo, c_hi = float(c[:n_end].mean()), float(c[-n_end:].mean())   # 算进尖端窗口 ⇒ 判反
    t = points_centered @ u
    std = float(t.std())
    skew = float(((t - t.mean()) ** 3).mean() / (std ** 3)) if std > 1e-12 else 0.0
    tip_plus_w = w_hi < w_lo
    tip_plus_c = c_hi < c_lo
    tip_plus_s = skew < 0.0
    # 结构判据：杆在 hi 端 ⇒ 尖端在 lo 端 ⇒ 指向 −u（tip_plus=False）
    if ev.get("shaft_side") == "hi":
        tip_at_plus = False
    elif ev.get("shaft_side") == "lo":
        tip_at_plus = True
    else:
        n_yes0 = int(tip_plus_w) + int(tip_plus_c) + int(tip_plus_s)
        tip_at_plus = (n_yes0 >= 2) or (n_yes0 == 1 and tip_plus_w)
    agree = sum(1 for v in (tip_plus_w, tip_plus_c, tip_plus_s)
                if v == tip_at_plus)
    return {"tip_plus": bool(tip_at_plus), "n_agree": int(max(agree, 3 - agree)),
            "w_lo": w_lo, "w_hi": w_hi, "c_lo": c_lo, "c_hi": c_hi, "skew": skew,
            "votes": (bool(tip_plus_w), bool(tip_plus_c), bool(tip_plus_s)),
            "agree": int(agree)}


def direction_from_points(points_xy, n_bins: int = 14, angle_step: float = 1.0,
                          refine_step: float = 0.25, refine_span: float = 1.0,
                          refine_min_gain: float = 0.01) -> dict:
    """从一团点估计"尖头"指向。``points_xy`` 可为像素或米制，返回同坐标系结果。

    **为什么不用 PCA 主轴**：实物箭头（``arena_pdf_img/p09_1.png``）头部宽 80 px、
    整个箭头才 58 px 高 —— 头部宽度大于箭杆长度，PCA 主轴因此是**横向**的
    （实测 PCA 给 −2.5°，真值 −90°，差 87°）。

    做法：扫描候选轴（默认 2.5° 步长，再在最优处 0.5° 精修），对每个轴算
    :func:`_axis_evidence` 的结构分 —— **一端等宽箭杆 + 另一端单调张开**才是箭头轴；
    尖端取"箭杆的另一端"。退化（点太少 / 各向同性 / 找不到杆）时 ``confidence=0``。
    """
    p = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    out = {"dir_xy": None, "dir_deg": 0.0, "axis_deg": 0.0,
           "confidence": 0.0, "evidence": {"n": int(p.shape[0])}}
    if p.shape[0] < 8:
        out["evidence"]["reason"] = "点太少"
        return out
    d = p - p.mean(axis=0)
    mxx = float((d[:, 0] ** 2).mean())
    myy = float((d[:, 1] ** 2).mean())
    mxy = float((d[:, 0] * d[:, 1]).mean())
    tr, det = mxx + myy, mxx * myy - mxy * mxy
    disc = max(0.0, tr * tr / 4.0 - det)
    lam_max, lam_min = tr / 2.0 + np.sqrt(disc), tr / 2.0 - np.sqrt(disc)
    aniso = 0.0 if lam_max <= 1e-15 else float(max(0.0, 1.0 - max(0.0, lam_min) / lam_max))
    theta = 0.5 * np.arctan2(2.0 * mxy, (mxx - myy))
    pca_deg = float(np.degrees(theta) % 180.0)
    out["evidence"].update({"anisotropy": round(aniso, 3), "pca_axis_deg": round(pca_deg, 2)})

    def make_u(deg: float) -> np.ndarray:
        a = np.radians(deg)
        return np.array([np.cos(a), np.sin(a)])

    cands = set(np.arange(0.0, 180.0, max(0.25, float(angle_step))).tolist())
    cands.add(pca_deg)

    def evaluate(deg: float) -> dict | None:
        """结构分 × 尖端线索一致性折扣 —— 轴必须被"尖端在哪端"的线索印证。"""
        ev = _axis_evidence(d, make_u(deg), n_bins)
        if not ev.get("ok") or ev["score"] <= 0.0:
            return None
        tip = _tip_side(make_u(deg), ev, d)
        ev["tip"] = tip
        ev["score_eff"] = ev["score"] * (1.0 if tip["agree"] >= 2 else 0.5)
        return ev

    best_deg, best_ev = None, None
    for deg in sorted(cands):
        ev = evaluate(deg)
        if ev is None:
            continue
        if best_ev is None or ev["score_eff"] > best_ev["score_eff"]:
            best_deg, best_ev = float(deg), ev
    if best_ev is None:
        out["evidence"].update({"reason": "找不到箭头结构（无箭杆+张开）"})
        return out

    # 局部精修：只接受足够大的增益，避免在分箱噪声上乱走
    step = max(0.1, float(refine_step))
    k = 1
    while k * step <= float(refine_span):
        improved = False
        for cand in (best_deg + k * step, best_deg - k * step):
            ev = evaluate(cand)
            if ev is not None and ev["score_eff"] > best_ev["score_eff"] + \
                    float(refine_min_gain):
                best_deg, best_ev, improved = float(cand) % 180.0, ev, True
        if not improved:
            break
        k += 1

    u = make_u(best_deg)
    tip = best_ev["tip"]
    dir_vec = u if tip["tip_plus"] else -u
    agree_factor = {3: 1.0, 2: 0.9, 1: 0.7, 0: 0.5}[int(tip["agree"])]
    # 各向同性闸门放宽到 0.20：箭头"头比身长"时 PCA 各向异性本来就不高（实物 ≈0.45），
    # 沿用 0.35 会把真箭头也压死。
    gate = float(min(1.0, aniso / 0.20))
    conf = min(1.0, best_ev["score_eff"] / 0.30) * agree_factor * gate
    ang = float(np.degrees(np.arctan2(dir_vec[1], dir_vec[0])))
    out.update({
        "dir_xy": dir_vec, "dir_deg": (ang + 180.0) % 360.0 - 180.0,
        "axis_deg": float(best_deg % 180.0),
        "confidence": float(max(0.0, min(1.0, conf))),
        "evidence": dict(out["evidence"], **{
            "axis_deg_scanned": round(float(best_deg), 2),
            "axis_score": round(float(best_ev["score"]), 3),
            "axis_score_eff": round(float(best_ev["score_eff"]), 3),
            "shaft_len": int(best_ev["shaft"]), "mono_len": int(best_ev["mono"]),
            "shaft_side": best_ev["shaft_side"],
            "lo_const": int(best_ev["lo_const"]), "hi_const": int(best_ev["hi_const"]),
            "lo_mono": int(best_ev["lo_mono"]), "hi_mono": int(best_ev["hi_mono"]),
            "both_shafts": best_ev["both_shafts"],
            "axis_contrast": round(float(best_ev["contrast"]), 3),
            "end_pts": int(best_ev["end_pts"]),
            "w_end_minus_u": round(tip["w_lo"], 4), "w_end_plus_u": round(tip["w_hi"], 4),
            "c_end_minus_u": round(tip["c_lo"], 2), "c_end_plus_u": round(tip["c_hi"], 2),
            "width_contrast": round(_rel(tip["w_hi"], tip["w_lo"]), 3),
            "count_contrast": round(_rel(tip["c_hi"], tip["c_lo"]), 3),
            "skew": round(tip["skew"], 3),
            "votes_width": tip["votes"][0], "votes_count": tip["votes"][1],
            "votes_skew": tip["votes"][2], "n_agree": int(tip["agree"]),
            "n_bins": int(n_bins),
        }),
    })
    return out

# --------------------------------------------------------------------------- #
#  检测
# --------------------------------------------------------------------------- #
def _dir_kind(deg: float) -> str:
    """±x（行方向，桥带长轴）还是 ±y（列方向，穿洞方向）。"""
    a = np.radians(deg)
    return "along_strip" if abs(np.cos(a)) >= abs(np.sin(a)) else "along_tunnel"


def detect_arrows(rgb, reg: ArenaRegistration, palette: Palette,
                  spec: ArrowSpec | None = None) -> list[ArrowDetection]:
    """返回按置信度降序的箭头候选（可能为空）。"""
    spec = spec or ArrowSpec()
    arr = as_rgb_array(rgb)
    masks = palette.masks(arr)
    if spec.cls not in masks:
        return []
    blobs = blob_masks(masks[spec.cls], min_pixels=spec.min_pixels,
                       k_open=spec.k_open, k_close=spec.k_close, rgb=arr)
    col = spec.bridge_col.strip().upper()
    out: list[ArrowDetection] = []
    for blob in blobs:
        if blob.pixels is None:
            continue
        uv = blob.pixels[:, ::-1].astype(np.float64)
        pts = reg.px_to_meters(uv)
        centroid = pts.mean(axis=0)
        hit = reg.meters_to_hit(float(centroid[0]), float(centroid[1]))
        cell_area = reg.cell_area_px(hit.block)
        area_frac = blob.area / cell_area if cell_area > 0 else 0.0
        if not (spec.min_cell_frac <= area_frac <= spec.max_cell_frac):
            continue
        res = direction_from_points(pts, n_bins=spec.n_bins)
        conf_area = 1.0 if 0.3 <= area_frac <= 1.0 else 0.6
        conf = float(res["confidence"]) * conf_area
        out.append(ArrowDetection(
            block=hit.block, row_idx=hit.row_idx, col_idx=hit.col_idx,
            half=hit.half, x_m=hit.x_m, y_m=hit.y_m, area_px=int(blob.area),
            area_frac=float(area_frac), axis_deg_map=float(res["axis_deg"]),
            dir_deg_map=float(res["dir_deg"]), dir_confidence=float(res["confidence"]),
            confidence=conf, dir_kind=_dir_kind(float(res["dir_deg"])),
            evidence=dict(res["evidence"], on_bridge_column=(hit.block[0] == col),
                          bbox=[blob.x0, blob.y0, blob.x1, blob.y1]),
        ))
    out.sort(key=lambda a: -a.confidence)
    return out


def infer_bridge_passable(arrows, spec: ArrowSpec | None = None,
                          grid: GridSpec | None = None) -> list[BridgeCall]:
    """把箭头候选折成"每个半场一个通行格"（车侧 ``bridge_passable`` 的载荷）。

    每个半场独立决策：取该半场置信度最高的箭头；无合格箭头则该半场**不输出**
    （宁可不报，也不要猜 —— 车侧对过期/缺失情报有 30 s 兜底，见 ``drone.py:18-21``）。
    """
    spec = spec or ArrowSpec()
    grid = grid or GridSpec()
    col = spec.bridge_col.strip().upper()
    calls: list[BridgeCall] = []
    for half in ("lower", "upper"):
        cands = [a for a in arrows
                 if a.half == half and a.confidence >= spec.min_conf
                 and (spec.allow_off_column or a.block[0] == col)]
        if not cands:
            continue
        best = cands[0]
        block = best.block
        ev = [f"箭头在 {best.block}（conf={best.confidence:.2f}），"
              f"指向 {best.dir_deg_map:.0f}°（{best.dir_kind}）",
              f"判据 H1 位置：桥面板在上，板下即通行格；"
              f"方向证据 一致线索数 {best.evidence.get('n_agree')}/3、"
              f"宽度对比 {best.evidence.get('width_contrast')}"]
        if spec.infer_mode == "direction":
            a = np.radians(best.dir_deg_map)
            r, c = grid.block_to_index(best.block)
            # 沿指向走 direction_steps 格：+x ⇒ 行号减小；+y ⇒ 列号减小
            dr = -int(round(np.cos(a))) * spec.direction_steps
            dc = -int(round(np.sin(a))) * spec.direction_steps
            block = grid.index_to_block(r + dr, c + dc)
            ev.append(f"判据 H2 方向：{best.block} + {spec.direction_steps} 格"
                      f"（Δrow={dr}, Δcol={dc}）→ {block}")
        calls.append(BridgeCall(
            half=half, block=block, confidence=best.confidence,
            dir_deg_map=best.dir_deg_map, dir_kind=best.dir_kind,
            candidates=tuple(a.block for a in cands), evidence=tuple(ev),
        ))
    return calls
