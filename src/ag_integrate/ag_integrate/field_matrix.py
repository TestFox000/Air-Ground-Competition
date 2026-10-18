"""识别结果矩阵输出：把 IntelFrame 的各类目标落到 11×14 场地格矩阵。

教员建议的矩阵形式态势汇报：
    - 每个识别目标（爆炸物/障碍物/靶/桥洞/车）吸附到场地格矩阵，一目了然；
    - 同时保留精确世界坐标（米，机载 PnP 结果），供命中解算/事后复核。

可靠性设计：
    - MatrixFusion 多帧时序融合：滑窗投票确认 + 中值坐标 + 自动老化，
      抑制单帧漏检（矩阵闪断）、单帧误检（幽灵目标）与坐标抖动；
    - 融合参数 window=5 / confirm=2：窗口内出现 ≥2 帧才上矩阵，
      移动靶（B/R）换格后最多滞后 1 帧重新确认，属可接受延迟。

矩阵符号（场地规则：一格至多一个目标，同格冲突取票数最高者，
平票障碍优先——安全考虑，规划宁可绕开）：
    M=mine(爆炸物)  O=obstacle(障碍物)  B=蓝靶(敌方)  R=红靶(友方)
    A=arrow(桥洞)   C=car(车)           .=空
CELL_LABEL 是定位基础设施（每帧数百个），默认不进矩阵/明细，参数可开。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from .field_frame import COLS, LETTERS, ROWS, world_to_cell
from .intel_protocol import IntelFrame, ObjectType

SYMBOLS: dict[ObjectType, str] = {
    ObjectType.PAPER: "M",       # paper 落格后语义：普通格=雷(M)，K 列=箭头(A)——见 matrix()
    ObjectType.OBSTACLE: "O",
    ObjectType.TARGET_BLUE: "B",
    ObjectType.TARGET_RED: "R",
    ObjectType.CAR: "C",
}

KIND_NAMES: dict[ObjectType, str] = {
    ObjectType.PAPER: "paper",
    ObjectType.CELL_LABEL: "cell_label",
    ObjectType.CAR: "car",
    ObjectType.TARGET_RED: "target_red",
    ObjectType.TARGET_BLUE: "target_blue",
    ObjectType.OBSTACLE: "obstacle",
    ObjectType.BUCKET_RED: "bucket_red",
    ObjectType.BUCKET_GREEN: "bucket_green",
    ObjectType.BUCKET_CYAN: "bucket_cyan",
    ObjectType.CAR_START: "car_start",
    ObjectType.CAR_END: "car_end",
    ObjectType.UAV_START: "uav_start",
    ObjectType.UAV_END: "uav_end",
}

# 同格多目标时的显示优先级（越小越优先）
_PRIORITY: dict[ObjectType, int] = {
    ObjectType.PAPER: 0,
    ObjectType.OBSTACLE: 1,
    ObjectType.TARGET_BLUE: 2,
    ObjectType.TARGET_RED: 3,
    ObjectType.CAR: 5,
}

# 符号 -> 优先级的反查表（_priority_of 用，避免每次遍历两个字典）
_PRIORITY_OF_SYMBOL: dict[str, int] = {sym: _PRIORITY[k] for k, sym in SYMBOLS.items()}

# 会移动的类别：超过 window 帧未复现则从矩阵清除（防移动靶拖影）
_MOVING_KINDS = {ObjectType.TARGET_BLUE, ObjectType.TARGET_RED,
                 ObjectType.CAR}


@dataclass(frozen=True)
class ObjectSpot:
    """单个识别目标：类别 + 落格 + 精确世界坐标。"""
    kind: ObjectType
    x: float                 # 世界坐标 x（米，机载 PnP）
    y: float                 # 世界坐标 y（米）
    col: int | None          # 落格列（0基）；出界/残差过大时为 None
    row: int | None          # 落格行（0基）

    @property
    def cell(self) -> str | None:
        if self.col is None or self.row is None:
            return None
        return f"{LETTERS[self.col]}{self.row + 1}"

    @property
    def kind_name(self) -> str:
        return KIND_NAMES[self.kind]


def spots_from_frame(frame: IntelFrame,
                     include_labels: bool = False) -> list[ObjectSpot]:
    """把一帧识别结果全部转为 ObjectSpot（保留精确坐标）。

    include_labels=True 时 CELL_LABEL 也进列表（量大，慎用于打印）。
    """
    spots: list[ObjectSpot] = []
    for obj in frame.objects:
        if obj.type is ObjectType.CELL_LABEL and not include_labels:
            continue
        try:
            c = world_to_cell(obj.x, obj.y)
            col: int | None = c.col
            row: int | None = c.row
        except ValueError:
            col = row = None  # 出界/离格心过远：保留坐标，不落格
        spots.append(ObjectSpot(obj.type, obj.x, obj.y, col, row))
    return spots


class MatrixFusion:
    """多帧时序融合：滑窗投票 + 中值坐标 + 自动老化。

    - 单帧漏检：目标在窗口内出现 ≥confirm 帧才确认，偶发漏帧不再闪断；
    - 单帧误检：孤立出现（1/5 帧）进不了矩阵，幽灵目标被过滤；
    - 坐标抖动：窗口内多次观测取中值，输出更稳的精确坐标；
    - 自动老化：观测只保留最近 window 帧序号，僵死目标自然淘汰，
      移动靶换格后新格重新计数确认。
    """

    def __init__(self, window: int = 5, confirm: int = 2) -> None:
        if not (1 <= confirm <= window):
            raise ValueError(f"need 1<=confirm<=window, got {confirm}/{window}")
        self.window = window
        self.confirm = confirm
        self._seq = 0
        # key=(kind, col, row) → 最近各帧观测 [(seq, x, y), ...]
        self._obs: dict[tuple[ObjectType, int, int],
                        deque[tuple[int, float, float]]] = {}
        # 出界目标不参与融合（无落格 key），但保留最近一次坐标供明细展示
        self._outside: dict[ObjectType, tuple[float, float]] = {}

    def add_frame(self, frame: IntelFrame,
                  include_labels: bool = False) -> list[ObjectSpot]:
        """喂入一帧，返回融合后的确认目标列表（含中值坐标）。

        融合粒度：同格同类多目标合并为一格一目标（坐标中值）。
        确认延迟：目标首次出现后需第 confirm 帧才上矩阵。
        遗忘机制：静态类（雷/障碍/桥洞）确认后永久保留（漏检容忍）；
        移动类（蓝/红靶、车）超过 window 帧未复现即清除，防止移动靶
        把走过的格永久留在矩阵上。
        """
        self._seq += 1
        for obj in frame.objects:
            if obj.type is ObjectType.CELL_LABEL and not include_labels:
                continue
            try:
                c = world_to_cell(obj.x, obj.y)
            except ValueError:
                self._outside[obj.type] = (obj.x, obj.y)
                continue
            key = (obj.type, c.col, c.row)
            q = self._obs.setdefault(key, deque(maxlen=self.window))
            q.append((self._seq, obj.x, obj.y))
        # 移动类遗忘：最近一次出现距今超过 window 帧 → 整键删除
        stale = [k for k, q in self._obs.items()
                 if k[0] in _MOVING_KINDS
                 and q and self._seq - q[-1][0] >= self.window]
        for k in stale:
            del self._obs[k]
        return self.fused_spots()

    def fused_spots(self) -> list[ObjectSpot]:
        """当前窗口的确认目标：票数≥confirm，坐标取中值。

        场地规则：一格至多一个目标。同格多种类别均过确认线时，
        取票数最高者为该格唯一目标；平票时障碍优先（安全：
        规划宁可绕开，避免把桶当雷导致撞桶）。
        """
        best: dict[tuple[int, int], tuple[int, ObjectType, float, float]] = {}
        for key, q in self._obs.items():
            if len(q) < self.confirm:
                continue
            kind, col, row = key
            xs = sorted(o[1] for o in q)
            ys = sorted(o[2] for o in q)
            n = len(q)
            mx = xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2
            my = ys[n // 2] if n % 2 else (ys[n // 2 - 1] + ys[n // 2]) / 2
            cell = (col, row)
            votes = n
            cur = best.get(cell)
            if cur is None or votes > cur[0] or (
                    votes == cur[0] and self._tie_break(kind, cur[1])):
                best[cell] = (votes, kind, mx, my)
        return [ObjectSpot(k, mx, my, c, r)
                for (c, r), (_, k, mx, my) in best.items()]

    @staticmethod
    def _tie_break(a: ObjectType, b: ObjectType) -> bool:
        """平票裁决：a 是否胜过 b。障碍优先，其余按显示优先级。"""
        a_obs = a is ObjectType.OBSTACLE
        b_obs = b is ObjectType.OBSTACLE
        if a_obs != b_obs:
            return a_obs
        return _PRIORITY.get(a, 99) < _PRIORITY.get(b, 99)

    def outside_spots(self) -> list[ObjectSpot]:
        """最近观测到的出界目标（不落格，仅报坐标）。"""
        return [ObjectSpot(k, x, y, None, None)
                for k, (x, y) in self._outside.items()]


def build_grid(spots: list[ObjectSpot]) -> list[list[str]]:
    """11×14 符号矩阵（row0=第1行，col0=A）。"""
    grid = [["." for _ in range(COLS)] for _ in range(ROWS)]
    for s in spots:
        if s.col is None or s.row is None or s.kind not in SYMBOLS:
            continue
        cur = grid[s.row][s.col]
        if cur == "." or _PRIORITY[s.kind] < _priority_of(cur):
            grid[s.row][s.col] = _symbol_of(s)
    return grid


def _priority_of(symbol: str) -> int:
    return _PRIORITY_OF_SYMBOL.get(symbol, len(_PRIORITY))


def _symbol_of(s: "ObjectSpot") -> str:
    """落格符号。paper 语义分流：K 列桥洞区纸片=通行箭头(A)，其余格=雷(M)。"""
    sym = SYMBOLS[s.kind]
    if (s.kind is ObjectType.PAPER and s.col is not None
            and 0 <= s.col < len(LETTERS) and LETTERS[s.col] == "K"):
        sym = "A"
    return sym


def render_matrix(spots: list[ObjectSpot]) -> str:
    """带表头的矩阵文本。"""
    grid = build_grid(spots)
    lines = ["     " + "  ".join(LETTERS)]
    for r in range(ROWS):
        lines.append(f" {r + 1:>2}  " + "  ".join(grid[r]))
    lines.append("图例: M=爆炸物 O=障碍 B=蓝靶(敌) R=红靶(友) "
                 "A=桥洞 C=车 .=空")
    return "\n".join(lines)


def render_spots(spots: list[ObjectSpot]) -> str:
    """精确坐标明细表：序号 类别 格号 x(米) y(米)。"""
    if not spots:
        return "（无识别目标）"
    lines = [f"{'#':>3}  {'kind':<13}{'cell':<5}{'x(m)':>9}{'y(m)':>9}"]
    for i, s in enumerate(spots, 1):
        cell = s.cell if s.cell else "OUT"
        lines.append(f"{i:>3}  {s.kind_name:<13}{cell:<5}"
                     f"{s.x:>9.3f}{s.y:>9.3f}")
    return "\n".join(lines)


def render_report_from_spots(spots: list[ObjectSpot],
                             timestamp_ms: int,
                             fused: bool = False) -> str:
    """完整态势报告（从已转换的 spots 渲染）：矩阵 + 精确坐标明细。"""
    tag = "融合" if fused else "单帧"
    head = (f"=== 场地态势矩阵[{tag}] t={timestamp_ms}ms "
            f"目标={len(spots)} ===")
    return "\n".join([head, render_matrix(spots), "", render_spots(spots)])


def render_report(frame: IntelFrame,
                  include_labels: bool = False) -> str:
    """单帧渲染（兼容旧接口；可靠性场景建议用 MatrixFusion）。"""
    return render_report_from_spots(
        spots_from_frame(frame, include_labels), frame.timestamp_ms)
