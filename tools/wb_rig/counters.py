# -*- coding: utf-8 -*-
"""计数器收集器（卡 `WB-2-RIG-20261031`）。

★ 铁律：**只读生产任务自己暴露的公开计数/公开方法**，不解析日志文本。
  日志是给人看的，措辞随时会改；把它当指标来源 = 把回归网钉在措辞上（"假绿"事故的
  一类：措辞一改，正则悄悄不匹配，测试照样"通过"）。
  这一条与 `AGENTS.md` §11「验收必须机器可判」同一口径。

每张表都写明：取值走哪个属性/方法 + 它在生产代码里的语义 + 这一项能否迁移到实车。
"""

from __future__ import annotations

from dataclasses import dataclass, field


# --------------------------------------------------------------------------- #
#  取值口径（属性名 → 说明）。改生产代码的属性名 = 这里立刻 AttributeError（这是好事）
# --------------------------------------------------------------------------- #
#: Localization 的公开计数（`tasks/localization.py::__init__` 里逐个赋 0）
LOC_COUNTERS: tuple[tuple[str, str], ...] = (
    ("ticks", "本任务跑了多少拍（20 Hz）"),
    ("missing_pose_ticks", "拿不到位姿的拍数（桥没帧/age=inf）"),
    ("projected_ticks", "成功写出路线投影的拍数"),
    ("anchored", "**到位吸附**次数（格心锚定，`fix.anchor_at_waypoint`）"),
    ("drift_warnings", "漂移告警次数（航点距离 > drift_warn_m）"),
    ("stuck_scans", "判「投影卡死」并做全路线重投影的次数"),
    ("projection_heals", "其中真正改了上报值的次数（卡死自愈生效）"),
    ("stuck_heal_refused", "大跳跃且新投影离车太远而**拒绝**自愈的次数"),
    ("anchor_refused", "因「误差会变大/超上限」**拒绝**吸附的次数"),
    ("reanchored", "横向重锚成功次数（只在顶点窗口内）"),
    ("motion_limited_ticks", "处于限速/停机状态的拍数（漂移护栏）"),
    ("motion_limit_published", "实际下发 `P_MOTION_LIMIT` 的次数（变化才发）"),
    ("boundary_alerts", "进入边界护栏区的次数"),
    ("drift_run", "结束时连续「有漂移未纠回」拍数"),
    ("direction_rejects", "投影方向一致性闸拒绝次数（逆行进方向的共线支）"),
)

#: Tracker 的公开计数（部分在 `telemetry()` 里，部分在属性上）
TRK_ATTRS: tuple[tuple[str, str], ...] = (
    ("_rev_push_timeouts", "倒车推车超时次数（3 s 未成）"),
    ("_rev_push_giveups", "同段放弃闩锁次数"),
    # ★★ 2026-11-01 WB-15：PIVOT 计数器**生产早就有、台架一直没登记**。
    #   生产 `tasks/tracker.py:250-251` 定义、`:841`/`:864` 真实累加，
    #   但本表漏登记 ⇒ 运行期 `metricsFull` 里**一个 pivot 键都没有**
    #   ⇒ WB-3 的 I05/I06/I15 三条现场案例无法重放（它们全靠 PIVOT 观测量）。
    #   现场案例 #5「PIVOT 无限重入，240 s 只走 4.4/34 m」是**两档仿真都测不出来**的
    #   教科书例子：tier-0 `sim.py` 不建模 PIVOT（根本不跑），tier-1 跑了却不报数。
    #   ⚠️ 这里**只登记取数**，生产侧一字不动（`tracker.py` 是冻结区）。
    #   ⚠️ 取数走 `hasattr` 判断 ⇒ 生产若哪天删了这两个字段，它们会自动消失而不是填 0
    #      （见下面的 `_read_counters` 约定：取不到就跳过，不硬造值）。
    ("pivot_count", "PIVOT 完成次数（原地转向成功收口）"),
    ("pivot_abandon_count", "PIVOT 超时放弃次数（**无限重入案的直接观测量**）"),
)

#: Tracker 遥测字典里的键（`TrackerTask.telemetry()` → `_last_cmd`）
TRK_TEL_KEYS: tuple[tuple[str, str], ...] = (
    ("blocked", "本拍是否被拦停（阶段/安全/无路线）"),
    ("reason", "拦停原因（人读；不作为指标）"),
    ("speed", "本拍**下发**的速度命令（m/s）"),
    ("steer_deg", "本拍**下发**的舵角命令（度）"),
    ("align", "是否处于桥洞直行保护段"),
    ("reverse", "是否处于倒车段"),
    ("seg_index", "本拍认为在**第几段**"),
    ("s", "本拍认为的**沿程弧长**（m）"),
    ("dist_to_path", "本拍到折线的横向距离（m）"),
    ("gate", "限速主因（cruise/curve/stop_approach/corner/min_speed/brake）"),
    # ★★ 2026-11-01 WB-17（I15 缺件补齐）：PIVOT 逐拍相位。
    #   生产 `tracker.py:703` **早就上报** `"pivot_phase": str(self._pivot_phase)`
    #   （值 IDLE / SETTLE / TURN，见 `PivotPhase`），而本表一直没登记
    #   ⇒ 台架 `trk_tel` 里只有 `align`、**没有 PIVOT 相位**
    #   ⇒ I15「PIVOT/ALIGN 段冲出去（限速被绕过）」判不了：
    #     要判"这段是不是在 PIVOT 里"，必须先看得见"这段是不是 PIVOT"。
    #   ⚠️ 纯加法，生产侧一字不动（`tracker.py` 是冻结区）。
    ("pivot", "本拍是否处于 PIVOT 原地转向（bool）"),
    ("pivot_phase", "PIVOT 相位（IDLE/SETTLE/TURN）⇒ I15 的直接观测量"),
)


def _read_counters(obj: object, table: tuple[tuple[str, str], ...]) -> dict[str, float]:
    """按表取值。**取不到就抛**（不静默填 0 —— 静默填 0 正是"假绿"的物理来源）。"""
    out: dict[str, float] = {}
    for name, _desc in table:
        if not hasattr(obj, name):
            raise AttributeError(
                f"{type(obj).__name__} 缺少公开计数 {name!r}；"
                f"生产代码改了属性名 ⇒ 本台架的台账必须同步改（不许静默填 0）")
        value = getattr(obj, name)
        try:
            out[name] = float(value)
        except (TypeError, ValueError) as exc:
            raise AttributeError(f"{type(obj).__name__}.{name} 不是数值（{value!r}）") from exc
    return out


@dataclass
class CounterBook:
    """一轮跑动的全部台账（含"这一项能不能迁移实车"的分级说明）。"""

    loc: dict[str, float] = field(default_factory=dict)
    trk_attrs: dict[str, float] = field(default_factory=dict)
    trk_tel: dict[str, object] = field(default_factory=dict)
    safety: dict[str, float] = field(default_factory=dict)
    mission: dict[str, object] = field(default_factory=dict)
    task_health: dict[str, dict] = field(default_factory=dict)
    #: 采信口径：本字典里的每个键 → (中文说明, transfer 分级)
    notes: dict[str, tuple[str, str]] = field(default_factory=dict)

    def as_metrics(self) -> dict[str, object]:
        """摊平成"可写进契约行"的扁字典（键名 `cnt_<区>_<名>`，前缀消歧）。"""
        out: dict[str, object] = {}
        for area, table in (("loc", self.loc), ("trk", self.trk_attrs),
                            ("saf", self.safety)):
            for k, v in table.items():
                out[f"cnt_{area}_{k.lstrip('_')}"] = v
        return out


def collect(*, loc: object | None = None, trk: object | None = None,
            saf: object | None = None, mis: object | None = None,
            group: object | None = None) -> CounterBook:
    """从**生产对象**收集台账。任一对象为 `None` 时该区留空（不造假数据）。"""
    book = CounterBook()
    if loc is not None:
        book.loc = _read_counters(loc, LOC_COUNTERS)
        for name, desc in LOC_COUNTERS:
            book.notes[f"cnt_loc_{name}"] = (desc, "可迁移")
    if trk is not None:
        # ★ 倒车推车相关的计数是**私名**属性：生产上它们没有公开 getter，
        #   但它们是**状态机自己的记账**（不是日志），所以仍然采信；取不到就跳过而不是填 0。
        for name, desc in TRK_ATTRS:
            if hasattr(trk, name):
                book.trk_attrs[name] = float(getattr(trk, name))
                book.notes[f"cnt_trk_{name.lstrip('_')}"] = (desc, "可迁移")
        tel = trk.telemetry() if callable(getattr(trk, "telemetry", None)) else {}
        book.trk_tel = {k: tel.get(k) for k, _ in TRK_TEL_KEYS}
        for k, desc in TRK_TEL_KEYS:
            # `speed`/`steer_deg`/`s`/`seg_index` 是**命令量**：跨平台可迁移
            # （同一套控制器、同一条路线，实车下发的是同一串数）。
            book.notes[f"tel_{k}"] = (desc, "可迁移")
    if saf is not None:
        for name, desc in (("ticks", "安全任务拍数"), ("brakes", "成功发出零速指令次数"),
                           ("brake_failures", "零速指令发布失败次数"),
                           ("forced_stops", "强制停车次数"),
                           ("stop_events", "安全停车事件条数"),
                           ("clear_events", "安全清除事件条数")):
            if hasattr(saf, name):
                book.safety[name] = float(getattr(saf, name))
                book.notes[f"cnt_saf_{name}"] = (desc, "可迁移")
        book.safety["stopped"] = float(bool(getattr(saf, "stopped", False)))
        book.notes["cnt_saf_stopped"] = ("结束时是否锁存 SAFE_STOP", "可迁移")
    if mis is not None:
        acct = mis.accounting() if callable(getattr(mis, "accounting", None)) else {}
        book.mission = dict(acct or {})
        book.mission["phase"] = str(getattr(mis, "phase", ""))
        book.mission["route_source"] = str(getattr(mis, "_route_source", ""))
    if group is not None:
        book.task_health = {name: {"state": h.state, "ticks": h.ticks,
                                   "errors": h.errors, "overruns": h.overruns}
                            for name, h in group.health().items()}
    return book


def transfer_of(key: str) -> str:
    """任一计数器键的 transfer 分级（未登记 ⇒ `不可迁移`，宁可从严）。"""
    if key.startswith("cnt_loc_") or key.startswith("cnt_saf_") or key.startswith("cnt_trk_"):
        return "可迁移"
    if key.startswith("tel_"):
        return "可迁移"
    return "不可迁移"


__all__ = ["CounterBook", "LOC_COUNTERS", "TRK_ATTRS", "TRK_TEL_KEYS", "collect", "transfer_of"]
