"""识别结果矩阵输出：把 IntelFrame 的各类目标落到 11×14 场地格矩阵。

教员建议的矩阵形式态势汇报：
    - 每个识别目标（爆炸物/障碍物/靶/桥洞/车）吸附到场地格矩阵，一目了然；
    - 同时保留精确世界坐标（米，机载 PnP 结果），供命中解算/事后复核。

可靠性设计：
    - MatrixFusion 多帧时序融合：滑窗投票确认 + 中值坐标 + 自动老化，
      抑制单帧漏检（矩阵闪断）、单帧误检（幽灵目标）与坐标抖动；
    - 融合参数 window=5 / confirm=2：窗口内出现 ≥2 帧才上矩阵，
      移动靶（B/R）换格后最多滞后 1 帧重新确认，属可接受延迟。

矩阵符号（同格多目标时按 M > O > B > R > A > C 取优先级最高者）：
    M=mine(爆炸物)  O=obstacle(障碍物)  B=蓝靶(敌方)  R=红靶(友方)
    A=arrow(桥洞)   C=car(车)           .=空
CELL_LABEL 是定位基础设施（每帧数百个），默认不进矩阵/明细，参数可开。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from .field_frame import COLS, ROWS, world_to_cell
from .intel_protocol import IntelFrame, ObjectType

SYMBOLS: dict[ObjectType, str] = {
    ObjectType.MINE: "M",
    ObjectType.OBSTACLE: "O",
    ObjectType.TARGET_BLUE: "B",
    ObjectType.TARGET_RED: "R",
    ObjectType.ARROW: "A",
    ObjectType.CAR: "C",
}

KIND_NAMES: dict[ObjectType, str] = {
    ObjectType.MINE: "mine",
    ObjectType.CAR: "car",
    ObjectType.ARROW: "arrow",
    ObjectType.TARGET_RED: "target_red",
    ObjectType.CELL_LABEL: "cell_label",
    ObjectType.TARGET_BLUE: "target_blue",
    ObjectType.OBSTACLE: "obstacle",
}

# 同格多目标时的显示优先级（越小越优先）
_PRIORITY: dict[ObjectType, int] = {
    ObjectType.MINE: 0,
    ObjectType.OBSTACLE: 1,
    ObjectType.TARGET_BLUE: 2,
    ObjectType.TARGET_RED: 3,
    ObjectType.ARROW: 4,
    ObjectType.CAR: 5,
}


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
        return f"{'ABCDEFGHIJKLMN'[self.col]}{self.row + 1}"

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
        """
        self._seq += 1
        seen_keys: set[tuple[ObjectType, int, int]] = set()
        for obj in frame.objects:
            if obj.type is ObjectType.CELL_LABEL and not include_labels:
                continue
            try:
                c = world_to_cell(obj.x, obj.y)
            except ValueError:
                self._outside[obj.type] = (obj.x, obj.y)
                continue
            key = (obj.type, c.col, c.row)
            seen_keys.add(key)
            q = self._obs.setdefault(key, deque(maxlen=self.window))
            q.append((self._seq, obj.x, obj.y))
        return self.fused_spots()

    def fused_spots(self) -> list[ObjectSpot]:
        """当前窗口的确认目标：票数≥confirm，坐标取中值。"""
        out: list[ObjectSpot] = []
        for key, q in self._obs.items():
            if len(q) < self.confirm:
                continue
            kind, col, row = key
            xs = sorted(o[1] for o in q)
            ys = sorted(o[2] for o in q)
            n = len(q)
            mx = xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2
            my = ys[n // 2] if n % 2 else (ys[n // 2 - 1] + ys[n // 2]) / 2
            out.append(ObjectSpot(kind, mx, my, col, row))
        return out

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
            grid[s.row][s.col] = SYMBOLS[s.kind]
    return grid


def _priority_of(symbol: str) -> int:
    for k, sym in SYMBOLS.items():
        if sym == symbol:
            return _PRIORITY[k]
    return len(_PRIORITY)


def render_matrix(spots: list[ObjectSpot]) -> str:
    """带表头的矩阵文本。"""
    grid = build_grid(spots)
    lines = ["     " + "  ".join("ABCDEFGHIJKLMN")]
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
