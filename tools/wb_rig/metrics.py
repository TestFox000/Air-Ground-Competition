# -*- coding: utf-8 -*-
"""指标分级、置灰与末行契约（卡 `WB-2-RIG-20261031`）。

## 三条硬规则（每条都对应一次真实误读）

1. **`transfer` ∈ {可迁移, 仅结构, 不可迁移}** —— `AGENTS.md` §硬约束 5 与 06 册 §2：
   "sim 绿不得表述为实车可行"。所以**每一条**指标都带一个分级，UI 据此决定可信度标注。
2. **`field_modeled=0` 时越界类指标输出 `null`** —— 06 册 §2 的原文是"显示 `0` 会被读成
   场地内零越界"，所以这里给 `None`（JSON 里是 `null`），让"没建模"与"零违规"在**数据层**
   就区分开，而不是靠 UI 记得置灰（UI 会忘，数据不会）。
3. **`dzrobot-sim-result/1` 五键只增不改** —— `finished/time_s/rms_m/out_of_bounds/length_ratio`
   是 UI 依赖面（06 册 §2 边界 2）。本台架用**新 schema 号** `dzrobot-trackbench-result/1`，
   五键形状**照抄**（同名同义），其余一律加在别处。

## 分级判据（为什么这一项是这个档）

| 档 | 判据 | 例子 |
|---|---|---|
| **可迁移** | 结论只依赖"生产线程 × 路线几何 × 控制器"，与**底盘机械**无关 | 到点台账、吸附次数、投影自愈次数、段号/弧长 |
| **仅结构** | 结论依赖执行器链的**形状**（有/无平滑、有/无量化），但**数值**取决于未标定参数 | 舵角速率峰值、死区拍数、看门狗拍数、"负速度被吃掉"的拍数 |
| **不可迁移** | 数值直接由**未标定的注入量**或**未建模的物理**决定 | 信念↔真值 RMS 的绝对值、横向误差绝对值、越界拍数（场地未建模时） |

⚠️ 特别地：`belief_error_rms_m` 在 `realistic` 档**不可迁移**（注入量本身待 wb-8 标定），
   但在 `ideal` 档它是**结构性判据**（必须 ≡0，那是"注入链没偷改物理"的证明）——
   同一个键在两档里分级**不同**，这是刻意的：分级说的是"这个数能不能拿去对实车"，
   不是"这个键名属于哪一类"。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

#: 本台架的契约 schema（**新号**，与 `dzrobot-sim-result/1` 并存不取代）
SCHEMA = "dzrobot-trackbench-result/1"

#: 现网五键（`dzrobot-sim-result/1`）—— 形状冻结，本台架**同名同义**照抄
FROZEN_FIVE = ("finished", "time_s", "rms_m", "out_of_bounds", "length_ratio")

#: transfer 三档
TRANSFER_MIGRATABLE = "可迁移"
TRANSFER_STRUCTURAL = "仅结构"
TRANSFER_NOT = "不可迁移"
TRANSFER_LEVELS = (TRANSFER_MIGRATABLE, TRANSFER_STRUCTURAL, TRANSFER_NOT)

#: ★ 越界类指标：**只有「地图边界可知」是前置条件**的那几个键。
#:   ⚠️ 2026-11-01 WB-11 修正：`field_stopped` / `field_stop_t` **不在**本元组里。
#:   06 册 §2 明写它们「由 `stop_on_field_violation` 决定，**两种模式都在**」
#:   （与 `out_of_bounds_steps` / `bounds_margin_m` 同一句话的同一类），
#:   tier-0 `sim.py:1594-1596` 也是 `if/else` **同级无条件写**（未触发 ⇒ `0.0` / `-1.0`，
#:   `-1` 是有效值不是 null）。塞进本元组会让它们继承「`field_modeled=0` ⇒ 压 null」
#:   的语义 ⇒ **两种模式下都消失**（建场地时没人写、不建场地时被 else 分支压 null）。
#:   ⇒ 存在性判据：**不随 `field_modeled` 改变**。
BOUNDS_KEYS: tuple[str, ...] = ("out_of_bounds", "out_of_bounds_steps",
                                "bounds_margin_m")

#: ★ 场地停车键（**两种模式都无条件写**，见上注）。放在这里只为了「一处声明、两处引用」，
#:   **绝不**参与 `field_modeled=0 ⇒ null` 的强制置灰。
FIELD_STOP_KEYS: tuple[str, ...] = ("field_stopped", "field_stop_t")

#: 指标分级表（键 → 档）。**新增指标必须在这里登记**，
#: 未登记的键在 `grade()` 里落到 `不可迁移`（宁可从严，不许乐观默认）。
GRADES: dict[str, str] = {
    # ---- 到点/推进台账（生产状态机自己记账，与底盘无关）----
    "steps": TRANSFER_MIGRATABLE,
    "time_s": TRANSFER_STRUCTURAL,          # 时长含执行器滞后 ⇒ 数值不能直接对实车
    "finished": TRANSFER_STRUCTURAL,
    "waypoints_reached": TRANSFER_MIGRATABLE,
    "waypoints_total": TRANSFER_MIGRATABLE,
    "stop_points_reached": TRANSFER_MIGRATABLE,
    "stop_points_total": TRANSFER_MIGRATABLE,
    "seg_final": TRANSFER_MIGRATABLE,
    "s_final_m": TRANSFER_MIGRATABLE,
    # ★★ WB-17：跳段可观测性（`finished` 可以被跳段骗过 ⇒ 必须有东西约束它）
    "seg_visited_count": TRANSFER_MIGRATABLE,
    "seg_expected_count": TRANSFER_MIGRATABLE,
    #: ★ 与 `finished` **并列**的第二道判据：" 走过多少段"，不受跳段骗
    "route_coverage": TRANSFER_MIGRATABLE,
    #: ★ 终点可达性单列（不靠分母；跳过终点时 coverage 可能仍高）
    "end_seg_reached": TRANSFER_MIGRATABLE,
    #: ★ I06 跳段观测量：投影跳段次数（相邻拍 seg 差>1）
    "seg_jump_count": TRANSFER_MIGRATABLE,
    "phase_final": TRANSFER_MIGRATABLE,
    "mission_completed": TRANSFER_STRUCTURAL,
    # ---- 路线几何（真值投影，与控制器无关）----
    "cross_track_rms_m": TRANSFER_STRUCTURAL,   # 数值受执行器链影响
    "cross_track_max_m": TRANSFER_STRUCTURAL,
    "final_error_m": TRANSFER_STRUCTURAL,
    "length_ratio": TRANSFER_STRUCTURAL,
    "rms_m": TRANSFER_STRUCTURAL,
    # ---- 信念误差：**两档分级不同**（见模块 docstring 末段）----
    "belief_error_rms_m": TRANSFER_NOT,      # ideal 档由 grade() 覆写成 可迁移
    "belief_error_max_m": TRANSFER_NOT,
    # ---- 执行器链的形状判据 ----
    "actuator_dead_zone_steps": TRANSFER_STRUCTURAL,
    "actuator_watchdog_steps": TRANSFER_STRUCTURAL,
    "actuator_smooth_lag_steps": TRANSFER_STRUCTURAL,
    "steer_exec_max_deg": TRANSFER_STRUCTURAL,
    "steer_cmd_max_deg": TRANSFER_STRUCTURAL,
    "speed_quant_count_max": TRANSFER_STRUCTURAL,
    "negative_speed_eaten_steps": TRANSFER_STRUCTURAL,
    # ---- 场地（未建模 ⇒ 不可迁移，且值必须是 None）----
    "field_modeled": TRANSFER_NOT,
    "out_of_bounds": TRANSFER_NOT,
    "out_of_bounds_steps": TRANSFER_NOT,
    "field_stopped": TRANSFER_NOT,
    "bounds_margin_m": TRANSFER_NOT,
    # ---- 注入量本身（它就是"不迁移"的定义：实车量级不等于实车）----
    "inject_speed_scale_err": TRANSFER_NOT,
    "inject_yaw_drift_dps": TRANSFER_NOT,
    "inject_pos_noise_m": TRANSFER_NOT,
    # ---- 生产计数器（`cnt_*` / `tel_*` 由 counters 统一给 可迁移）----
}


def grade(key: str, *, arm: str = "") -> str:
    """某键在某档下的 transfer 分级（`arm` 为空 = 与档无关）。"""
    if key.startswith("cnt_") or key.startswith("tel_"):
        return TRANSFER_MIGRATABLE
    if key == "belief_error_rms_m" and str(arm) == "ideal":
        # ideal 档下这条是**结构性判据**（必须 ≡0），可以拿去做"注入链没偷改物理"的验收
        return TRANSFER_STRUCTURAL
    return GRADES.get(key, TRANSFER_NOT)


@dataclass
class Metrics:
    """一轮跑动的指标容器：**值 + 分级**一起走，避免"值到处漂、分级在别处丢"。"""

    values: dict[str, object] = field(default_factory=dict)
    arm: str = ""
    field_modeled: int = 0
    notes: dict[str, str] = field(default_factory=dict)

    # ------------------------------ 写 ------------------------------ #
    def put(self, key: str, value: object, note: str = "") -> None:
        """写一个指标。越界类在 `field_modeled=0` 时**强制**落成 `None`。"""
        if key in BOUNDS_KEYS and self.field_modeled <= 0:
            self.values[key] = None
            self.notes[key] = note or "场地未建模（field_modeled=0）⇒ 置灰，不给 0"
            return
        self.values[key] = _finite_or_none(value)
        if note:
            self.notes[key] = note

    def put_ungraded(self, key: str, value: object, note: str = "") -> None:
        """写一个**旁路**数值（不参与分级表；契约行里落在 `extras`）。"""
        self.values[key] = _finite_or_none(value)
        if note:
            self.notes[key] = note

    # ------------------------------ 读 ------------------------------ #
    def graded(self) -> dict[str, str]:
        """键 → transfer 分级（`arm` 相关的覆写在这里生效）。"""
        return {k: grade(k, arm=self.arm) for k in self.values}

    def five(self) -> dict[str, object]:
        """现网五键（形状冻结）。**缺键补 `None`**，绝不用 0 冒充。"""
        out: dict[str, object] = {}
        for key in FROZEN_FIVE:
            out[key] = self.values.get(key)
        return out

    def by_transfer(self) -> dict[str, list[str]]:
        """按档分组（UI 直接按档渲染：可迁移一栏、仅结构一栏、不可迁移置灰一栏）。"""
        buckets: dict[str, list[str]] = {lvl: [] for lvl in TRANSFER_LEVELS}
        for key, lvl in self.graded().items():
            buckets.setdefault(lvl, []).append(key)
        return {k: sorted(v) for k, v in buckets.items()}

    # ---------------------------- 序列化 ----------------------------- #
    def payload(self, *, meta: dict | None = None, extras: dict | None = None) -> dict:
        """构造末行契约对象（`SCHEMA`）。**保证严格 JSON**（NaN/Inf → null）。"""
        return {
            "schema": SCHEMA,
            "ok": True,
            "arm": self.arm,
            "field_modeled": int(self.field_modeled),
            # 五键（形状冻结，UI 依赖面）
            "metrics": self.five(),
            # 全部指标（只增；分级随值一起走）
            "metricsFull": _sanitize(dict(self.values)),
            "transfer": self.graded(),
            "transfer_groups": self.by_transfer(),
            "notes": dict(self.notes),
            "meta": _sanitize(dict(meta or {})),
            "extras": _sanitize(dict(extras or {})),
        }

    def line(self, *, meta: dict | None = None, extras: dict | None = None,
             prefix: str = "TRACKBENCH_RESULT_JSON") -> str:
        """末行字符串（前缀区分于现网 `SIM_RESULT_JSON`，且**永远是 stdout 最后一行**）。"""
        body = json.dumps(self.payload(meta=meta, extras=extras),
                          ensure_ascii=False, sort_keys=True, allow_nan=False)
        return f"{prefix} {body}"


# --------------------------------------------------------------------------- #
#  小工具
# --------------------------------------------------------------------------- #
def _finite_or_none(value: object) -> object:
    """非有限浮点 → `None`（契约要求严格 JSON，`allow_nan=False` 会把 NaN 直接炸掉）。"""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    return value


def _sanitize(value: object) -> object:
    """递归清洗：NaN/±Inf → None；tuple → list；其余原样（与 `sim_cli._json_sanitize` 同口径）。"""
    if isinstance(value, dict):
        return {str(k): _sanitize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize(v) for v in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (bool, int, str)) or value is None:
        return value
    return str(value)


def format_table(metrics: Metrics, *, title: str = "台架指标（按 transfer 分级）") -> str:
    """人读表：**按档分栏**打印，不可迁移那栏一律打 `（置灰）`。"""
    lines = ["=" * 78, title]
    grp = metrics.by_transfer()
    for lvl in TRANSFER_LEVELS:
        keys = grp.get(lvl, [])
        if not keys:
            continue
        lines.append(f"── {lvl}（{len(keys)} 项）──")
        for key in keys:
            val = metrics.values.get(key)
            if lvl == TRANSFER_NOT:
                shown = "（置灰）" if val is None else f"{val!r}  ← 数值不可用于对实车"
            else:
                shown = "（无值）" if val is None else f"{val!r}"
            lines.append(f"   {key:32s} {shown}")
    lines.append("=" * 78)
    return "\n".join(lines)


def _same_strict(va: object, vb: object) -> bool:
    """严格相等（★ `None` 与 `0` **不算相同**，否则「没建模」与「零违规」会在 diff 里摸平）。"""
    if va is None or vb is None:
        return va is None and vb is None
    if isinstance(va, bool) or isinstance(vb, bool):
        return va is vb
    return va == vb


def diff_metrics(a: "Metrics", b: "Metrics", *,
                 keys: tuple[str, ...] | None = None) -> dict:
    """双臂 A/B 逐键 diff（WB-12 F3）。

    ★ 为什么要它：WB-2 acceptance #5 写了「输出两份指标与 diff」，而旧实现
      **只有两份指标、没有 diff** ⇒ 两臂对照靠人眼对两张表，而人眼
      最容易看漏的恰恰是**差异最小、最可疑的那几键**（0 vs 0、None vs 0）。

    返回（机器可读，直接落 JSON）::

        {"a": "臂A", "b": "臂B", "n_keys": N, "n_same": S, "n_diff": D,
         "rows": [{"key": k, "grade": "可迁移/仅结构/不可迁移",
                   "a": v_a, "b": v_b, "delta": v_b - v_a, "same": bool}, ...]}
    """
    ka = keys if keys is not None else tuple(sorted(set(a.values) | set(b.values)))
    ga, gb = a.graded(), b.graded()
    rows: list[dict] = []
    for k in ka:
        va, vb = a.values.get(k), b.values.get(k)
        both_num = (isinstance(va, (int, float)) and not isinstance(va, bool)
                    and isinstance(vb, (int, float)) and not isinstance(vb, bool))
        # 任一侧为 None ⇒ delta 为 None（**不当 0 算**）
        delta = (float(vb) - float(va)) if both_num else None
        rows.append({"key": k, "grade": ga.get(k) or gb.get(k) or TRANSFER_NOT,
                     "a": va, "b": vb, "delta": delta,
                     "same": _same_strict(va, vb)})
    return {"a": a.arm, "b": b.arm, "n_keys": len(ka),
            "n_same": sum(1 for r in rows if r["same"]),
            "n_diff": sum(1 for r in rows if not r["same"]),
            "rows": rows}


__all__ = [
    "SCHEMA", "FROZEN_FIVE", "BOUNDS_KEYS", "FIELD_STOP_KEYS",
    "GRADES", "TRANSFER_LEVELS",
    "TRANSFER_MIGRATABLE", "TRANSFER_STRUCTURAL", "TRANSFER_NOT",
    "Metrics", "grade", "format_table", "diff_metrics",
]
