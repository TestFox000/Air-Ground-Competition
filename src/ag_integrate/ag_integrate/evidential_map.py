"""证据格地图：正/负证据积累的场地语义地图（机载建图的核心数据结构）。

设计依据（2026-10-17 方案）：
    - 证据模型：log-odds（贝叶斯对数几率）。正证据 = 落格检测（带 D13 置信度），
      负证据 = 视野清晰覆盖该格且未检出（"空"的证据）——这是它区别于
      field_matrix 滑窗投票的本质：能积累、能清除、能量化置信。
    - 负证据纪律（禁火铁律的守门员）：
        * 画质不足（quality < min_quality）的覆盖不产生负证据，只计覆盖率；
        * 粘性类（红靶/红桶）一旦确认，负证据与时间衰减都永不击穿——
          对应 world_model 的红靶粘性否决，宁可信其有；
        * 雷语义类（PAPER）负证据减半：mine R≈98%，"没看到"≠"没有"。
    - 动态语义：靶类不落死格——set_track() 后靶证据写在轨道级（TrackSpot），
      瞬时落格位置不进静态地图（靶以 0.138m/s 在 D8-E8 轨道往返）。
    - 衰减：logit 指数半衰（向 0），读取时惰性计算，无后台定时器。

可靠性承诺（测试锁定）：
    - 两次高置信检测确认，单帧误检不进门（与 field_matrix 同门槛语义）；
    - 负证据能清除低置信 FP，但无法轻易清除已确认的雷（减半权重）；
    - 粘性类确认后任何负证据/衰减都不再失去确认（禁火）；
    - 快照只报 ≥drop_logit 的目标，带 confirmed 标志，无闪断。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .field_frame import COLS, ROWS, CellRef
from .intel_protocol import ObjectType


@dataclass(frozen=True)
class MapSpot:
    cell: CellRef
    kind: ObjectType
    conf: float          # 衰减后的信念 p ∈ (0,1)
    confirmed: bool


@dataclass(frozen=True)
class TrackSpot:
    """靶轨道级目标：轨道格集合 + 信念（不落死格）。"""

    kind: ObjectType
    cells: tuple[CellRef, ...]
    conf: float
    confirmed: bool


@dataclass(frozen=True)
class EvidenceParams:
    confirm_logit: float = 2.5        # 确认门限：单帧 0.9 不进门，两帧 0.9 确认
    drop_logit: float = -0.5          # 快照剔除门限
    neg_gain: float = 0.7             # 单次清晰负证据减量（基准）
    pos_conf_min: float = 0.05        # 正证据置信度夹挤（防 logit 爆炸）
    pos_conf_max: float = 0.95
    logit_floor: float = -5.0         # 负证据总下限（可恢复，不硬删）
    sticky_kinds: frozenset = frozenset({ObjectType.TARGET_RED,
                                         ObjectType.BUCKET_RED})
    # 禁火类独立确认语义：单次强观测（conf≥0.73）即粘性确认，此后
    # 负证据与时间衰减永不击穿（宁可信其有——误击红靶 −5/次）
    sticky_confirm_logit: float = 1.0
    min_quality: float = 0.5          # 负证据合法性的最低画质
    half_life_s: float = 600.0        # logit 半衰期
    track_kinds: frozenset = frozenset({ObjectType.TARGET_BLUE,
                                        ObjectType.TARGET_RED})
    # 负证据类别权重：雷语义减半（mine R≈98%，"没看到"≠"没有"）
    neg_weight: dict = field(default_factory=lambda: {ObjectType.PAPER: 0.5})


_DEFAULT_TRACKED = frozenset({ObjectType.PAPER, ObjectType.OBSTACLE,
                              ObjectType.TARGET_BLUE, ObjectType.TARGET_RED})


def _sigmoid(l: float) -> float:
    if l >= 0:
        return 1.0 / (1.0 + math.exp(-l))
    e = math.exp(l)
    return e / (1.0 + e)


class EvidentialMap:
    """11×14 场地格 × 类别 的证据地图。时间戳为单调钟秒。"""

    def __init__(self, params: EvidenceParams | None = None,
                 tracked_kinds: frozenset | None = None):
        self.p = params or EvidenceParams()
        self.tracked = tracked_kinds if tracked_kinds is not None else _DEFAULT_TRACKED
        self._logits: dict[tuple[int, int], dict[ObjectType, float]] = {}
        # t(单调秒) → 该时刻有正证据的格集合（负证据互斥用，v3）
        self._pos_seen: dict[float, set] = {}
        self._sticky_confirmed: dict[tuple[int, int], set] = {}
        self._views: dict[tuple[int, int], int] = {}
        self._clear_views: dict[tuple[int, int], int] = {}
        self._last_t: dict[tuple[int, int], float] = {}
        self._track_cells: tuple[CellRef, ...] | None = None
        self._track_logits: dict[ObjectType, float] = {}
        self._track_sticky: set = set()
        self._track_last_t: dict[ObjectType, float] = {}

    # ------------------------------------------------ 正证据
    def add_positive(self, cell: CellRef, kind: ObjectType, conf: float,
                     t: float) -> None:
        """一次落格检测（conf 为 D13 类别置信度）。

        ★ 低置信重映射（2026-10-24 v3 接线修复）：log-odds 对 conf<0.5 的检测
        给**负**增益——而 D13 的原始分数未校准（0.45 的检测常是真目标，
        10-20 已实测"降阈值多送候选"优于"高阈值单帧把关"）。
        把证据增益下限夹到 +0.15（=conf 0.537 的 logit）：低置信检测不再
        *反对*目标存在，只是积累得慢——五帧 0.45~0.57 从 logit 0.08 修到
        ≈2.9（可确认）。高置信语义不变（0.95 → +2.94）。
        """
        if kind not in self.tracked:
            raise ValueError(f"kind {kind} not tracked by this map")
        cell.validate()
        conf = min(max(conf, self.p.pos_conf_min), self.p.pos_conf_max)
        # 增益曲线（v3 定稿）：raw log-odds，下限夹到 0.15；对 0.4~0.7 的
        # "低置信真目标带"做**线性放大**（×3.0，上限 1.2）——五帧 0.45~0.57
        # 积累 2.9 可确认；恒 0.45 五帧 2.25≈门线（再一帧即确认）；
        # 0.3 幽灵仍只有 0.15/帧，三帧 0.45 远低于确认门。
        raw = math.log(conf / (1.0 - conf))
        if 0.4 <= conf < 0.7:
            gain = min(max(raw, 0.15) * 3.0, 1.2)
        else:
            gain = max(raw, 0.15)
        if kind in self.p.track_kinds and self._track_cells is not None:
            self._track_logits[kind] = self._track_logits.get(kind, 0.0) + gain
            self._track_last_t[kind] = t
            if (kind in self.p.sticky_kinds
                    and self._track_logits[kind] >= self.p.sticky_confirm_logit):
                self._track_sticky.add(kind)
            return
        key = (cell.col, cell.row)
        per = self._logits.setdefault(key, {})
        per[kind] = per.get(kind, 0.0) + gain
        self._last_t[key] = t
        # 同帧该格已有正证据 ⇒ 本帧负证据不成立（"检测到了"≠"看清没有"）。
        # v3（2026-10-24）：旧版对检测帧也扣负证据，低置信正增益被当场吃掉一半。
        self._pos_seen.setdefault(t, set()).add(key)
        if (kind in self.p.sticky_kinds
                and per[kind] >= self.p.sticky_confirm_logit):
            self._sticky_confirmed.setdefault(key, set()).add(kind)

    # ------------------------------------------------ 覆盖 / 负证据
    def observe_cell(self, cell: CellRef, t: float, quality: float) -> None:
        """一格被视野覆盖一次。

        quality ≥ min_quality 视为"看清了"：对格内已有证据施加负证据
        （粘性类除外）；画质不足只计覆盖率（负证据的合法性来源）。
        """
        cell.validate()
        key = (cell.col, cell.row)
        self._views[key] = self._views.get(key, 0) + 1
        self._last_t[key] = max(self._last_t.get(key, t), t)
        if quality < self.p.min_quality:
            return
        if key in self._pos_seen.get(t, set()):
            return          # 同帧正证据在先：本帧负证据不成立（v3 修复）
        self._clear_views[key] = self._clear_views.get(key, 0) + 1
        per = self._logits.get(key)
        if not per:
            return
        sticky = self._sticky_confirmed.get(key, set())
        for kind in list(per):
            if kind in self.p.sticky_kinds and kind in sticky:
                continue                       # 粘性否决：负证据无效
            l = per[kind]
            if l >= self.p.confirm_logit:
                # 已确认目标：负证据**比例衰减且有地板**（2026-10-24 修复，两版迭代）。
                # v1 线性 -0.35/帧：单帧 0.95 确认的箭头 3 帧磨损出局——箭头每场
                # 只露 1~2 帧，"看清没看到"不该杀掉一个已过确认门的目标。
                # v2 无地板比例衰减：跌破 confirm 门后落入线性路径被加速清出——
                # 一次穿越门线的抖动就会闪断，违背"已确认不闪断"承诺。
                # 终版：×0.955/帧 但地板 = confirm_logit ⇒ 负证据最多把已确认目标
                # 磨回"恰在门上"；真正淘汰已确认目标只有两条合法路径：
                # ① 半衰期（长时间无任何复现，_effective_logit 惰性计算）；
                # ② 同格更高票的其它类别胜出（一格一目标语义，规划层裁决）。
                # 幽灵清除能力不变：未确认目标仍走线性路径。
                per[kind] = max(l * 0.955, self.p.confirm_logit)
            else:
                # 未确认目标：负证据**比例衰减**（l - 0.175 → l×0.93 当 l≈2.5；
                # l→0 时近似不动）。v3 修复（2026-10-24）：旧版固定 -0.35/帧对
                # 低置信真雷是"积累<磨损"——五帧 0.45~0.57（logit 每帧 +0.2~+0.3）
                # 永远追不上 -0.35 的固定扣减，低置信真目标结构性无法确认
                # （这正是本次接线的动因场景）。比例衰减让小 logit 减得慢、
                # 大 logit 减得快：幽灵（单帧孤证）几帧内归零，缓升真目标
                # 的积累恒大于磨损（+logit/帧 > l×0.07 当 l<3）。
                w = self.p.neg_weight.get(kind, 1.0)
                per[kind] = max(l - abs(l) * 0.07 - 0.02 * w, self.p.logit_floor)

    def observe_cells(self, cells, t: float, quality: float) -> None:
        for c in cells:
            self.observe_cell(c, t, quality)

    # ------------------------------------------------ 查询
    def _effective_logit(self, key, kind, t: float) -> float:
        per = self._logits.get(key)
        if not per or kind not in per:
            return 0.0
        dt = max(t - self._last_t.get(key, t), 0.0)
        l = per[kind] * 0.5 ** (dt / self.p.half_life_s)
        if (kind in self._sticky_confirmed.get(key, set())
                and kind in self.p.sticky_kinds):
            l = max(l, self.p.sticky_confirm_logit)
        return l

    def belief(self, cell: CellRef, kind: ObjectType, t: float) -> float:
        return _sigmoid(self._effective_logit((cell.col, cell.row), kind, t))

    def cell_is_clear(self, cell: CellRef) -> bool:
        """该格是否被"看清过"至少一次（负证据已合法化的格子）。"""
        return self._clear_views.get((cell.col, cell.row), 0) > 0

    def coverage_frac(self) -> float:
        total = COLS * ROWS
        return len(self._clear_views) / total

    def set_track(self, cells) -> None:
        """设定移动靶轨道格集合（如 D8-E8 两格）；靶类证据改写轨道级。"""
        cells = tuple(cells)
        if not 1 <= len(cells) <= 4:
            raise ValueError("track must have 1..4 cells")
        for c in cells:
            c.validate()
        self._track_cells = cells

    def track_belief(self, kind: ObjectType, t: float) -> float:
        if kind not in self._track_logits:
            return 0.0
        dt = max(t - self._track_last_t.get(kind, t), 0.0)
        l = self._track_logits[kind] * 0.5 ** (dt / self.p.half_life_s)
        if kind in self._track_sticky:
            l = max(l, self.p.sticky_confirm_logit)
        return _sigmoid(l)

    def snapshot(self, t: float) -> list:
        """当前可信目标列表：MapSpot（≥drop_logit）+ TrackSpot（轨道级）。

        confirmed = 粘性已确认（禁火类，永不失）或 logit ≥ confirm_logit；
        快照带滞后（confirm/drop 双门限），单次负证据不会让已确认目标闪断。
        """
        spots: list = []
        for key, per in self._logits.items():
            for kind in per:
                l = self._effective_logit(key, kind, t)
                sticky_hit = (kind in self.p.sticky_kinds
                              and kind in self._sticky_confirmed.get(key, set()))
                if l >= self.p.drop_logit or sticky_hit:
                    spots.append(MapSpot(CellRef(key[0], key[1]), kind,
                                         _sigmoid(max(l, self.p.drop_logit)),
                                         sticky_hit or l >= self.p.confirm_logit))
        if self._track_cells is not None:
            for kind in list(self._track_logits):
                dt = max(t - self._track_last_t.get(kind, t), 0.0)
                l = self._track_logits[kind] * 0.5 ** (dt / self.p.half_life_s)
                sticky_hit = kind in self._track_sticky
                if sticky_hit:
                    l = max(l, self.p.sticky_confirm_logit)
                if l >= self.p.drop_logit:
                    spots.append(TrackSpot(kind, self._track_cells,
                                           _sigmoid(l),
                                           sticky_hit or l >= self.p.confirm_logit))
        return spots

    def confirmed_only(self, t: float) -> list:
        return [s for s in self.snapshot(t) if s.confirmed]
