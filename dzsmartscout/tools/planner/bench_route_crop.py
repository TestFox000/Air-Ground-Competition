#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""bench_route_crop.py —— L0 裁剪的 H1 / H2 bench（离线，不需要车、不需要 ROS）

对应决策层方案 §7.1 的两条**待现场验证的假设**，本文件把它们变成**可离线复跑**的证据：

======  ============================================================================
H1      「首点 = 车当前格时，Tracker 不会倒车去首点」
H2      「已压过的雷被重新纳入整线不会要求回头压」
======  ============================================================================

## 测量方式（★ 近似模型，不是真车复现 —— 这条必须写在结论里）

H1 要问的是**控制器会不会要求倒车**，而 `path_executor` 本体不在 vendor 子集里（vendor 只有
`core/` + `route/`）。本 bench 因此在**参数层面**复刻标准纯追踪：

* 全部增益/限幅/前视/到位半径 **从 config 真值读**（`planner_engine.read_pursuit`，
  与车侧同一份 `Config`：`pursuit.*`）—— 不许自己编数；
* 转向律用 SCHEMA 注释里写明的教科书式 `δ = atan(2·L·sinα / ld)`（`config.py:72`
  "curvature_gain: 1.0 = 教科书式 δ=atan(2L·sinα/ld)"），`ld = clamp(k_v·v, lo, hi)`；
* 舵角限幅 `max_steer_deg`、舵角速率限幅 `max_steer_rate_deg_s` 照 config 施加；
* 车侧 `allow_reverse=false` ⇒ 模型**不会倒车**；因此凡是 `|α| > 90°` 的拍都记为
  **"需要倒车/掉头"**（`n_reversal_ticks`）—— 这就是 H1 的判据。

⇒ bench 结论只能背书"**在这个近似模型下**，裁剪后的交接不需要倒车"。
   真车那 20 拍的 `pursuitAngle` 仍要现场录一次（本卡不改车侧代码，也不假装跑过真车）。

## H2 为什么"裁剪"就能解决

整线是**从起点重算**的（`plan_route` 没有 `already_covered` 入参）⇒ 车已压过的那段雷天然
还在整线里。裁掉前缀后它们**不在后缀里**，所以不会要求回头压。
但若车的实际行进顺序与整线不同（中途换了桥洞），某格"已压"却出现在后缀里 —— L0 修不了，
只能**报冲突拒绝下发**（`repress`）。H2c 就是这条。

跑法：
    python dzsmartscout\tools\planner\bench_route_crop.py            # 表格 + 写 EVIDENCE
    python dzsmartscout\tools\planner\bench_route_crop.py --no-evidence
退出码：0 = 全部用例 PASS / 1 = 有用例 FAIL。
"""
from __future__ import annotations

import argparse
import math
import pathlib
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

HERE = pathlib.Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
VENDOR = HERE / "vendor" / "dzbrain_ws"
EVID = HERE / "EVIDENCE"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import route_crop as RC                                       # noqa: E402

#: 纯追踪预览的仿真拍频（车侧 Mission tick = 10 Hz，见决策层方案 §6.2 的 269 ms/拍 说明）
TICK_S = 0.1
TICKS = 20                                                    # §7.1 H1：「看 pursuitAngle 首 20 拍」


# --------------------------------------------------------------------------- #
#  车侧参数（真值，不编）
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class PursuitParams:
    wheelbase_m: float
    max_steer_deg: float
    max_steer_rate_deg_s: float
    curvature_gain: float
    lookahead_min_m: float
    lookahead_max_m: float
    lookahead_k_v: float
    cruise_speed_mps: float
    arrival_m: float
    source: str = ""

    def lookahead(self, speed: float) -> float:
        ld = self.lookahead_k_v * abs(speed)
        return min(max(ld, self.lookahead_min_m), self.lookahead_max_m)


def load_params() -> PursuitParams:
    """从 vendor 树（= 真源同步产物）读 `pursuit.*`；缺键直接报错，不填默认值。"""
    import planner_engine as PE
    raw = PE.read_pursuit(VENDOR)
    need = ("wheelbase_m", "max_steer_deg", "max_steer_rate_deg_s", "curvature_gain",
            "lookahead_min_m", "lookahead_max_m", "lookahead_k_v", "cruise_speed_mps",
            "arrival_m")
    miss = [k for k in need if raw.get(k) is None]
    if miss:
        raise SystemExit(f"config 里缺 pursuit.{miss}（vendor 树不完整？先跑 sync_planner_from_source.py）")
    return PursuitParams(**{k: float(raw[k]) for k in need}, source=str(raw["config_path"]))


def vendor_route() -> List[str]:
    """机侧整线 = vendor 树的 `mission_blocks`（板载回退路线）。

    为什么用 vendor 而不是现跑规划器：L0 是**在无人机上**跑的，车上/机上用的是
    `vendor/**` 这棵树；本 bench 要证明"这棵树产出的整线能被正确裁剪"。
    `planner/tests/test_planner_sync.py` 里那条「机算 == mission_blocks」断言**只在真源
    与 vendor 同步时**成立；一旦漂移它就 FAIL —— 那时本 bench 会在输出里打 ⚠️，
    并把 vendor 与真源两条整线**都跑一遍**（`main()` 里做的事）。
    """
    import planner_engine as PE
    mb = PE.read_mission_blocks(VENDOR / "config" / "robot.yaml")
    if len(mb) < 8:
        raise SystemExit("vendor robot.yaml 里没有可用的 mission_blocks")
    return mb


def source_route() -> Optional[List[str]]:
    """`dzrobot_v2` 真源里的 `mission_blocks`（只读；用于验证"裁剪与路线无关"）。

    真源与 vendor 可能不同步（并发卡改车侧代码/换路线）——本函数只读，不触发同步。
    """
    p = WORKSPACE / "dzrobot_v2" / "config" / "robot.yaml"
    if not p.is_file():
        return None
    import planner_engine as PE
    mb = PE.read_mission_blocks(p)
    return mb if len(mb) >= 8 else None


# --------------------------------------------------------------------------- #
#  H1 的测量：纯追踪近似模型
# --------------------------------------------------------------------------- #
def _polyline_dist(pt: Sequence[float], pts: Sequence[Sequence[float]]) -> float:
    """点到折线的最短距离（米）。"""
    x, y = float(pt[0]), float(pt[1])
    best = float("inf")
    for a, b in zip(pts[:-1], pts[1:]):
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        vx, vy = bx - ax, by - ay
        den = vx * vx + vy * vy
        t = 0.0 if den <= 1e-12 else max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / den))
        best = min(best, math.hypot(x - (ax + t * vx), y - (ay + t * vy)))
    return best


def preview(car_xy: Sequence[float], car_yaw_deg: float, blocks: Sequence[str],
            p: PursuitParams, *, ticks: int = TICKS, dt: float = TICK_S,
            next_wp: int = 1) -> Dict[str, Any]:
    """从 (car_xy, 车头) 出发，按纯追踪追 `blocks`，返回逐拍指标。

    `next_wp=1` 是车侧 `_load_tokens` 的真实起点（首点装 `P_ROUTE_INDEX=0`、从第 2 点开始追），
    **不是**我为了方便设的。
    """
    spec = RC.grid()
    # 收 token 原文（`F3*` / `N5,y`）也收纯格名：统一取格名再换算，语义尾巴不参与几何
    blocks = [RC.token_block(b) or str(b) for b in blocks]
    pts = [spec.block_to_meters(b) for b in blocks]
    if len(pts) < 2:
        raise ValueError("至少 2 个航点")
    nxt = max(1, min(int(next_wp), len(pts) - 1))
    x, y = float(car_xy[0]), float(car_xy[1])
    th = math.radians(float(car_yaw_deg))
    v, L = p.cruise_speed_mps, p.wheelbase_m
    delta_prev = 0.0
    alphas: List[float] = []
    dists: List[float] = []
    devs: List[float] = []
    travelled = 0.0
    d_first = None
    for _ in range(int(ticks)):
        ld = p.lookahead(v)
        # 前视点：从 nxt 起第一个距车 ≥ ld 的航点（都不够就用最后一个）
        carrot = pts[-1]
        for j in range(nxt, len(pts)):
            if math.hypot(pts[j][0] - x, pts[j][1] - y) >= ld:
                carrot = pts[j]
                break
        d = math.hypot(carrot[0] - x, carrot[1] - y)
        if d_first is None:
            d_first = d
        alpha = RC.wrap180(math.degrees(math.atan2(carrot[1] - y, carrot[0] - x))
                           - math.degrees(th))
        alphas.append(abs(alpha))
        dists.append(d)
        # 教科书式纯追踪：δ = atan(2·L·sinα / ld)
        delta = math.degrees(math.atan(2.0 * L * math.sin(math.radians(alpha)) / ld
                                       * p.curvature_gain))
        delta = max(-p.max_steer_deg, min(p.max_steer_deg, delta))
        rate = p.max_steer_rate_deg_s * dt
        delta = max(delta_prev - rate, min(delta_prev + rate, delta))
        delta_prev = delta
        th += v / L * math.tan(math.radians(delta)) * dt
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        travelled += v * dt
        devs.append(_polyline_dist((x, y), pts))
        if math.hypot(pts[nxt][0] - x, pts[nxt][1] - y) <= p.arrival_m:
            nxt = min(nxt + 1, len(pts) - 1)
    return {
        "d_to_first_target_m": round(float(d_first or 0.0), 3),
        "first_alpha_deg": round(alphas[0], 2),
        "max_alpha_deg": round(max(alphas), 2),
        "n_reversal_ticks": sum(1 for a in alphas if a > 90.0),
        "travelled_m": round(travelled, 3),
        "max_path_dev_m": round(max(devs), 3),
        "min_dist_to_carrot_m": round(min(dists), 3),
        "last_dist_to_carrot_m": round(dists[-1], 3),
        "alphas_deg": [round(a, 1) for a in alphas],
    }


# --------------------------------------------------------------------------- #
#  用例
# --------------------------------------------------------------------------- #
@dataclass
class CaseResult:
    name: str
    title: str
    passed: bool
    checks: List[Tuple[str, bool, str]] = field(default_factory=list)
    metrics: Dict[str, Any] = field(default_factory=dict)

    def add(self, what: str, ok: bool, detail: str = "") -> "CaseResult":
        self.checks.append((what, bool(ok), detail))
        if not ok:
            self.passed = False
        return self

    def to_lines(self) -> List[str]:
        out = [f"{'PASS' if self.passed else 'FAIL'}  {self.name}  {self.title}"]
        for what, ok, detail in self.checks:
            out.append(f"      {'✓' if ok else '✗'} {what}" + (f"  [{detail}]" if detail else ""))
        if self.metrics:
            out.append("      metrics: " + ", ".join(f"{k}={v}" for k, v in self.metrics.items()
                                                    if k != "alphas_deg"))
        return out


def _car(block: str, yaw: Optional[float] = None, phase: str = "DRIVE") -> RC.CarState:
    return RC.CarState(block=block, yaw_deg=yaw, phase=phase, source="bench")


#: 交接判据：车到「第一个前视目标」的距离上限（2 格 = 0.70 m）。
#: 为什么用距离而不只看航向：整线原样下发时第一个目标是 2.8 m 外的 M10，航向可能只是"略偏"，
#: 距离才是"车得往回开多远"的直接度量。
D_FIRST_MAX_M = 0.75


def _case_h1a(route: List[str], p: PursuitParams) -> CaseResult:
    """H1a：直线上交接（G10；该处路线沿 +y 直行）⇒ 首拍航向误差≈0、0 拍倒车。"""
    res = RC.crop_route(route, _car("G10", yaw=90.0))
    r = CaseResult("H1a", "裁剪@G10（直段交接）首拍不倒车", res.ok, metrics=dict(res.stats or {}))
    r.add("裁剪被接受", res.ok, "; ".join(res.reject))
    r.add("首点 = 车当前格", res.route[:1] == ("G10",), str(res.route[:2]))
    r.add("交接夹角 ≤ 10°", (res.stats.get("handover") or {}).get("turn_deg", 999) <= 10.0,
          str((res.stats.get("handover") or {}).get("turn_deg")))
    m = preview(RC.cell_xy("G10"), 90.0, list(res.route), p)
    r.metrics.update(m)
    r.add("首个前视目标 ≤ 2 格（0.75 m）", m["d_to_first_target_m"] <= D_FIRST_MAX_M,
          f"{m['d_to_first_target_m']} m")
    r.add("首拍航向误差 ≤ 10°", m["first_alpha_deg"] <= 10.0, f"{m['first_alpha_deg']}°")
    r.add("0 拍需要倒车", m["n_reversal_ticks"] == 0, f"{m['n_reversal_ticks']}/{TICKS} 拍")
    r.add("20 拍内偏离路线 ≤ 0.10 m", m["max_path_dev_m"] <= 0.10, f"{m['max_path_dev_m']} m")
    r.add("20 拍内实际前进 ≥ 0.60 m（未停滞/打转）", m["travelled_m"] >= 0.60,
          f"{m['travelled_m']} m")
    return r


def _case_h1b(route: List[str], p: PursuitParams) -> CaseResult:
    """H1b：拐点上交接（E10，路线在此 90° 转向）⇒ 前视会切角，首拍 ≤ 60° 且不被判回头。

    ★ 不断言"20 拍内 0 拍 |α|>90"：90° 拐点 + 0.36 m 前视 + 0.12 m 到位判定会让纯追踪
    切角后短暂"目标在身后"，**每个拐点都会这样**（路线固有，不是裁剪引入）。本 bench 的
    H1 结论只针对**交接瞬间**，后续拐角由车侧的吸附/转角减速层负责（本模型不复刻）。
    """
    res = RC.crop_route(route, _car("E10", yaw=90.0))
    r = CaseResult("H1b", "裁剪@E10（90° 拐点交接）交接瞬间不回头", res.ok,
                   metrics=dict(res.stats or {}))
    r.add("裁剪被接受", res.ok, "; ".join(res.reject))
    r.add("交接未判为回头（verdict != reversal）",
          (res.stats.get("handover") or {}).get("verdict") != "reversal",
          str((res.stats.get("handover") or {}).get("verdict")))
    m = preview(RC.cell_xy("E10"), 90.0, list(res.route), p)
    r.metrics.update(m)
    r.add("首个前视目标 ≤ 2 格（0.75 m）", m["d_to_first_target_m"] <= D_FIRST_MAX_M,
          f"{m['d_to_first_target_m']} m")
    r.add("首拍航向误差 ≤ 60°（切角把 90° 摊到多拍）", m["first_alpha_deg"] <= 60.0,
          f"{m['first_alpha_deg']}°")
    return r


def _case_h1c(route: List[str], p: PursuitParams) -> CaseResult:
    """H1c：车心偏 0.10 m（横向，belief 不在格心）⇒ 交接瞬间仍不回头。"""
    res = RC.crop_route(route, _car("G10", yaw=90.0))
    xy = RC.cell_xy("G10")
    off = (xy[0] + 0.10, xy[1])            # 车头朝 +y ⇒ 横向是 ±x
    m = preview(off, 90.0, list(res.route), p)
    r = CaseResult("H1c", "裁剪@G10 + 横向偏 0.10 m 仍不回头", res.ok, metrics=m)
    r.add("裁剪被接受", res.ok, "; ".join(res.reject))
    r.add("首个前视目标 ≤ 2 格", m["d_to_first_target_m"] <= D_FIRST_MAX_M,
          f"{m['d_to_first_target_m']} m")
    r.add("首拍航向误差 ≤ 30°", m["first_alpha_deg"] <= 30.0, f"{m['first_alpha_deg']}°")
    return r


def _case_h1d(route: List[str], p: PursuitParams) -> CaseResult:
    """H1d：车心沿车头后退 0.10 m（还没到格心）⇒ 交接瞬间仍不回头。"""
    res = RC.crop_route(route, _car("G10", yaw=90.0))
    xy = RC.cell_xy("G10")
    off = (xy[0], xy[1] - 0.10)            # 车头朝 +y ⇒ 后退 0.10 m 是 −y
    m = preview(off, 90.0, list(res.route), p)
    r = CaseResult("H1d", "裁剪@G10 + 纵向偏 0.10 m 仍不回头", res.ok, metrics=m)
    r.add("裁剪被接受", res.ok, "; ".join(res.reject))
    r.add("首个前视目标 ≤ 2 格", m["d_to_first_target_m"] <= D_FIRST_MAX_M,
          f"{m['d_to_first_target_m']} m")
    r.add("首拍航向误差 ≤ 30°", m["first_alpha_deg"] <= 30.0, f"{m['first_alpha_deg']}°")
    return r


def _case_h1e(route: List[str], p: PursuitParams) -> CaseResult:
    """H1e **对照组**：不裁剪（整线原样下发，首点仍是 N10）⇒ 车会被命令往回开 2.8 m。

    这一条是"必须要有裁剪"的反证：航向 180°、首个目标 2.8 m（8 格）开外。
    """
    r = CaseResult("H1e", "对照组：整线原样下发 ⇒ 车被命令往回开", True)
    m = preview(RC.cell_xy("E10"), 90.0, route, p, next_wp=1)      # belief 被当成 N10
    r.metrics.update(m)
    r.add("首个前视目标 > 2 m（车得往回开好几格）", m["d_to_first_target_m"] > 2.0,
          f"{m['d_to_first_target_m']} m")
    r.add("首拍航向误差 > 135°（等于命令车往回开）", m["first_alpha_deg"] > 135.0,
          f"{m['first_alpha_deg']}°")
    r.add("绝大多数拍都需要倒车/掉头", m["n_reversal_ticks"] >= 15,
          f"{m['n_reversal_ticks']}/{TICKS} 拍")
    return r


def _case_h2a(route: List[str]) -> CaseResult:
    """H2a：裁剪@A2，已压雷（D9/H10）在**被裁掉的前缀**里 ⇒ 0 回头、0 重压。"""
    res = RC.crop_route(route, _car("A2", yaw=0.0), pressed=["D9", "H10"])
    st = res.stats or {}
    r = CaseResult("H2a", "裁剪@A2：已压雷落在前缀 ⇒ 不回头压", res.ok, metrics=dict(st))
    r.add("裁剪被接受", res.ok, "; ".join(res.reject))
    r.add("首点 = A2", res.route[:1] == ("A2",), str(res.route[:2]))
    r.add("裁掉 23 个前缀 token", st.get("n_dropped") == 23, str(st.get("n_dropped")))
    r.add("已压格 D9/H10 不出现在后缀", not ({"D9", "H10"} & set(RC.blocks_of(list(res.route)))))
    r.add("冲突数 = 0（无 repress / 无 revisit）", not res.conflicts,
          str([c["kind"] for c in res.conflicts]))
    r.add("裁剪后的后缀里没有重走过任何前缀格",
          not (set(st.get("dropped_prefix") or []) & set(RC.blocks_of(list(res.route)[1:]))))
    return r


def _case_h2b(route: List[str]) -> CaseResult:
    """H2b：同一场景**不裁剪** ⇒ 23 个已过格会被重走（这就是必须裁的理由）。"""
    res = RC.crop_route(route, _car("N10", yaw=0.0))          # 锚点被迫回到起点
    st = res.stats or {}
    r = CaseResult("H2b", "对照组：锚点=起点 ⇒ 整段重走（crop_required=False 是危险态）", True,
                   metrics={"n_out": st.get("n_out"), "n_dropped": st.get("n_dropped")})
    r.add("锚点在起点 ⇒ 一个 token 都没裁（crop_required=False）",
          st.get("crop_required") is False, str(st.get("crop_required")))
    r.add("若此时车其实在 A2，后缀会包含 23 个已走过格（回头段）",
          st.get("n_out") == len(route), f"{st.get('n_out')} vs {len(route)}")
    return r


def _case_h2c(route: List[str]) -> CaseResult:
    """H2c：车已压过**后缀里**的某格（换洞/改道导致与整线顺序不一致）⇒ 必须报 repress 拒绝。

    ★ 冲突格不写死：取「锚点之后第 5 格」（任何整线都成立），避免 bench 绑死某条路线。
    """
    toks = route
    blocks = RC.blocks_of(toks)
    idx = blocks.index("A2")
    victim = blocks[idx + 5]
    res = RC.crop_route(toks, _car("A2", yaw=0.0), pressed=[victim])
    kinds = [c["kind"] for c in res.conflicts]
    hit = [c for c in res.conflicts if c["block"] == victim]
    r = CaseResult("H2c", f"已压格 {victim}（锚点后第 5 格）落在后缀 ⇒ 拒绝下发", not res.ok,
                   metrics={"n_conflicts": len(res.conflicts), "victim": victim})
    r.add("裁剪被拒绝", not res.ok, "; ".join(res.reject)[:120])
    r.add("冲突类型含 repress", "repress" in kinds, str(kinds))
    r.add(f"冲突格是 {victim}", bool(hit), str([c.get("block") for c in res.conflicts]))
    return r


def _case_h2d(route: List[str]) -> CaseResult:
    """H2d：打靶/收尾阶段不许改线（phase 闸门）。"""
    r = CaseResult("H2d", "phase 闸门：STOP/FINISH 阶段拒绝改线", True)
    for ph in ("STOP", "FINISH", "SAFE", "IDLE"):
        res = RC.crop_route(route, _car("A2", yaw=0.0, phase=ph))
        r.add(f"phase={ph} 被拒绝", (not res.ok) and any("phase=" in x for x in res.reject),
              "; ".join(res.reject)[:60])
    for ph in ("DRIVE", "APPROACH"):
        res = RC.crop_route(route, _car("A2", yaw=0.0, phase=ph))
        r.add(f"phase={ph} 放行", res.ok, "; ".join(res.reject)[:60])
    return r


def _field_verify(*, complete: bool, cells: Sequence[str], reported_mines: Sequence[str] = ()):
    """造一条 t10 口径的 `field_verify`（带 coverage/slice/scan_id）。"""
    if str(RC.TOOLS) not in sys.path:
        sys.path.insert(0, str(RC.TOOLS))
    from perception.protocol import build_field_verify, make_scan_id
    return build_field_verify(
        mines=list(reported_mines), obstacles=[],
        coverage={"cells": list(cells), "complete": bool(complete), "mode": "strip"},
        slice={"id": "S1", "index": 1, "of": 1, "route": "N8>M8>L8>K8"},
        scan_id=make_scan_id(1700000000.0, token="bench"),
    )


def _case_h2e(route: List[str]) -> CaseResult:
    """H2e：t10 的负证据语义被正确继承（complete=true 才产生 ABSENT）。"""
    res = RC.crop_route(route, _car("A2", yaw=0.0))
    suffix = list(RC.blocks_of(list(res.route))[1:])
    r = CaseResult("H2e", "coverage 负证据：只有 complete=true 才减少未知格", True)

    inc = _field_verify(complete=False, cells=suffix)
    cleared_i, note_i = RC.cleared_from_field_verify(inc)
    r_i = RC.crop_route(route, _car("A2", yaw=0.0), cleared=cleared_i)
    r.add("complete=false ⇒ cleared 为 None（不产生 ABSENT）", cleared_i is None, "; ".join(note_i))
    r.add("complete=false ⇒ 未知格数不变",
          r_i.stats.get("n_unknown_ahead") == res.stats.get("n_unknown_ahead"),
          f"{r_i.stats.get('n_unknown_ahead')} vs {res.stats.get('n_unknown_ahead')}")

    full = _field_verify(complete=True, cells=list(RC.grid().all_blocks()))
    cleared_t, note_t = RC.cleared_from_field_verify(full)
    r_t = RC.crop_route(route, _car("A2", yaw=0.0), cleared=cleared_t)
    r.add("complete=true ⇒ cleared 非空", bool(cleared_t), "; ".join(note_t))
    r.add("complete=true 覆盖全场 ⇒ 后缀未知格 = 0",
          r_t.stats.get("n_unknown_ahead") == 0, str(r_t.stats.get("n_unknown_ahead")))

    old = RC.cleared_from_field_verify({"type": "field_verify", "mines": ["A3"]})
    r.add("旧报文无 coverage ⇒ None + 说明", old[0] is None and bool(old[1]), "; ".join(old[1])[:80])
    return r


def _case_h2f() -> CaseResult:
    """H2f：同一格在整线里出现多次时的锚点消歧（无 yaw 必须拒绝）。"""
    dup = ["N10", "M10", "M9", "M10", "L10", "K5", "N5"]      # M10 出现两次
    r = CaseResult("H2f", "锚点消歧：重复格 + 无 yaw ⇒ 拒绝；有 yaw ⇒ 选对", True)
    bad = RC.crop_route(dup, _car("M10", yaw=None))
    r.add("无 yaw ⇒ 拒绝（不猜）", not bad.ok, "; ".join(bad.reject)[:100])
    # 第 2 次出现（下标 1）的入线方向 N10→M10 = +y(90°)；第 4 次（下标 3）的 M9→M10 = 180°
    f1 = RC.locate_anchor(dup, _car("M10", yaw=90.0))[0]
    f2 = RC.locate_anchor(dup, _car("M10", yaw=180.0))[0]
    r.add("yaw=90° ⇒ 选中下标 1（入线 N10→M10 = +y）", f1 is not None and f1.index == 1,
          None if f1 is None else f"{f1.index}/{f1.mode}")
    r.add("yaw=180° ⇒ 选中下标 3（入线 M9→M10 = −x）", f2 is not None and f2.index == 3,
          None if f2 is None else f"{f2.index}/{f2.mode}")
    return r


def _case_h2g(route: List[str]) -> CaseResult:
    """H2g：车格不在整线里（被外部改道）⇒ 默认拒绝；--allow-nearest 才接受。"""
    r = CaseResult("H2g", "车格不在整线里：默认拒绝，显式才就近", True)
    res = RC.crop_route(route, _car("N11", yaw=0.0))
    r.add("默认拒绝", not res.ok, "; ".join(res.reject)[:100])
    res2 = RC.crop_route(route, _car("N11", yaw=0.0), allow_nearest=True)
    r.add("--allow-nearest ⇒ 接受并标注 nearest",
          res2.ok and (res2.anchor is not None) and res2.anchor.mode == "nearest",
          None if res2.anchor is None else res2.anchor.mode)
    return r


def _case_h2h(route: List[str]) -> CaseResult:
    """H2h：车头与锚点矛盾（在 B9 却朝 −x，而路线要求往 +x）⇒ 判 handover_reversal 拒绝。

    这是"belief 报的格和车头对不上"的兜底：宁可拒发，也不让车按矛盾量掉头。
    """
    r = CaseResult("H2h", "车头与锚点矛盾 ⇒ 拒绝（不让车按矛盾量掉头）", True)
    res = RC.crop_route(route, _car("B9", yaw=180.0))
    kinds = [c["kind"] for c in res.conflicts]
    r.add("被拒绝", not res.ok, "; ".join(res.reject)[:100])
    r.add("冲突类型含 handover_reversal", "handover_reversal" in kinds, str(kinds))
    ok = RC.crop_route(route, _car("B9", yaw=0.0))
    r.add("同一格车头朝 +x 时放行", ok.ok, "; ".join(ok.reject)[:100])
    return r


def run_bench(route: Optional[Sequence[str]] = None) -> List[CaseResult]:
    p = load_params()
    toks = list(route) if route is not None else vendor_route()
    return [
        _case_h1a(toks, p),
        _case_h1b(toks, p),
        _case_h1c(toks, p),
        _case_h1d(toks, p),
        _case_h1e(toks, p),
        _case_h2a(toks),
        _case_h2b(toks),
        _case_h2c(toks),
        _case_h2d(toks),
        _case_h2e(toks),
        _case_h2f(),
        _case_h2g(toks),
        _case_h2h(toks),
    ]


def _run_one(label: str, toks: List[str]) -> Tuple[List[CaseResult], List[str]]:
    lines = [f"--- 整线来源：{label}（{len(toks)} token）---", " ".join(toks), ""]
    results = run_bench(toks)
    for r in results:
        lines += r.to_lines()
        lines.append("")
    n_pass = sum(1 for r in results if r.passed)
    lines.append(f"{label} 结论：{n_pass}/{len(results)} 用例 PASS"
                 + ("  ✅" if n_pass == len(results) else "  ❌ 有 FAIL"))
    lines.append("")
    return results, lines


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="L0 裁剪的 H1/H2 bench（离线）")
    ap.add_argument("--no-evidence", action="store_true", help="不写 EVIDENCE/route_crop_bench.txt")
    ap.add_argument("--route", default=None, help='显式指定整线（默认取 vendor mission_blocks）')
    args = ap.parse_args(list(argv) if argv is not None else sys.argv[1:])

    p = load_params()
    vroute = RC.split_route(args.route) if args.route else vendor_route()
    sroute = source_route()

    head = ["=== route_crop H1/H2 bench（离线近似模型，非真车）===",
            f"pursuit 参数真值（{p.source}）: wheelbase={p.wheelbase_m} max_steer={p.max_steer_deg}° "
            f"rate={p.max_steer_rate_deg_s}°/s ld=[{p.lookahead_min_m},{p.lookahead_max_m}] "
            f"k_v={p.lookahead_k_v} v={p.cruise_speed_mps} arrival={p.arrival_m}",
            f"预览：{TICKS} 拍 × {TICK_S}s @ {p.cruise_speed_mps} m/s", ""]
    if sroute is not None and sroute != vroute:
        head.append("⚠️ 真源 dzrobot_v2 的 mission_blocks 与 vendor 不同步（并发卡改了车侧）——"
                    "两条整线都跑一遍，证明裁剪逻辑与具体路线无关")
        head.append("")

    all_results: List[CaseResult] = []
    body: List[str] = []
    for label, toks in (("vendor（机侧实跑树）", vroute), ("source dzrobot_v2（只读对照）", sroute)):
        if toks is None:
            body += [f"--- 整线来源：{label} 不可用，跳过 ---", ""]
            continue
        res, lines = _run_one(label, toks)
        all_results += res
        body += lines

    n_pass = sum(1 for r in all_results if r.passed)
    tail = [f"总结论：{n_pass}/{len(all_results)} 用例 PASS"
            + ("  ✅ bench 全过" if n_pass == len(all_results) else "  ❌ 有 FAIL"),
            "",
            "口径三条（别把 bench 当现场背书）：",
            "  ① 近似模型：转向律按 config 注释的教科书式 δ=atan(2·L·sinα/ld)，"
            "path_executor 本体不在 vendor 子集内 ⇒ 不是逐行复现；",
            "  ② 拐点交接会出现 |α|>90 的拍（切角 + 0.12 m 到位判定），**每个拐点都这样**，"
            "属路线固有；H1 结论只针对交接瞬间；",
            "  ③ 真车那 20 拍 pursuitAngle 仍需现场录一次（本卡不改车侧代码）。"]
    text = "\n".join(head + body + tail)
    print(text)
    if not args.no_evidence:
        EVID.mkdir(parents=True, exist_ok=True)
        (EVID / "route_crop_bench.txt").write_text(text + "\n", encoding="utf-8")
        print(f"\n证据：{EVID / 'route_crop_bench.txt'}")
    return 0 if n_pass == len(all_results) else 1


if __name__ == "__main__":
    sys.exit(main())
