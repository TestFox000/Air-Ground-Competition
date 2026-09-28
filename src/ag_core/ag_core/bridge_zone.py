"""桥洞通道的占格推理与校验（纯算法，无 ROS）。

规则原文（`docs/raw_txt/02_15空地协同侦排挑战赛.txt` 第 5 节）：
    * 场地设两处桥洞通道，35 cm 挡板架空，通道之间有隔板隔离；
    * 裁判在两处桥洞中**各随机指定 1 条可通行通道**，通行状态标识贴在**桥梁顶部**；
    * 地面车视角看不到桥顶标识 —— **必须由无人机侦察**；
    * 「下面的桥洞通道共占用 5 个格子，裁判系统会在这 5 个格子中选择 1 个可通行通道」
      （另一处 6 个格子）。

占格推断（两处 5 + 6 = 11，恰好等于场地行数 11）：
    * 场地是 14 列（A~N，沿 5 m 边）x 11 行（1~11，沿 4 m 边）；
    * 场地布置说明书 V2 的图 7/图 8 里，通行标识分别出现在 **K9** 与 **K4**；
    * K 列的第 1~5 行正好 5 格、第 6~11 行正好 6 格，且 K4 ∈ 前段、K9 ∈ 后段 —— 与"5 选 1 / 6 选 1"完全吻合。

因此默认假设：**两处桥洞都沿 K 列排布，按行切成 5 格（A 段）+ 6 格（B 段）**。
⚠️ 这只是**基于公开资料的推断**，必须现场核实：
   1. 现场量一下两处桥洞各自覆盖哪几格（是否真在 K 列）；
   2. 若不符，改 `params.yaml` 的 `bridge_zone_col` / `bridge_zone_split`，或在
      `BridgeZones` 里直接给两段的格号列表。
   3. 在核实之前，本模块的校验结果只用于**告警**，不参与丢弃情报
      （车端 `strict_bridge_zone: false`）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .grid_map import CellID

__all__ = ["BridgeZones", "BridgeCheck", "check_bridge_pair"]

#: 依据《场地布置说明书 V2》图 7/图 8 的标识位置（K4 / K9）推断出的默认布点
DEFAULT_COL = "K"
DEFAULT_SPLIT = 5          # 前段 5 格，余下为后段 6 格


@dataclass
class BridgeZones:
    """两处桥洞通道各自覆盖的格号集合。

    col     桥洞所在列（默认 "K"）
    split   前段的行数（默认 5）；后段为该列剩下的行
    rows    该列的行区间 (起, 止)，含端点；默认用 1..rows
    """

    col: str = DEFAULT_COL
    split: int = DEFAULT_SPLIT
    rows: int = 11
    row_start: int = 1

    zone_a: Tuple[str, ...] = field(init=False, default=())
    zone_b: Tuple[str, ...] = field(init=False, default=())

    def __post_init__(self) -> None:
        end = self.row_start + self.rows - 1
        a_end = min(end, self.row_start + self.split - 1)
        self.zone_a = tuple(f"{self.col}{r}" for r in range(self.row_start, a_end + 1))
        self.zone_b = tuple(f"{self.col}{r}" for r in range(a_end + 1, end + 1))

    # ---------------- 查询 ----------------
    def zone_of(self, cell: str) -> Optional[str]:
        """格号属于哪个桥洞区域：'a' / 'b' / None（不在任一桥洞内）。"""
        name = (cell or "").strip().upper()
        if name in self.zone_a:
            return "a"
        if name in self.zone_b:
            return "b"
        return None

    def cells(self, zone: str) -> Tuple[str, ...]:
        return self.zone_a if zone.lower() == "a" else self.zone_b

    def describe(self) -> str:
        return (f"A 段 {self.zone_a[0]}~{self.zone_a[-1]}（{len(self.zone_a)} 选 1）"
                f" / B 段 {self.zone_b[0]}~{self.zone_b[-1]}（{len(self.zone_b)} 选 1）")


@dataclass
class BridgeCheck:
    """桥洞情报校验结果。"""

    ok: bool
    problems: List[str] = field(default_factory=list)
    zone_a: Optional[str] = None
    zone_b: Optional[str] = None
    strict: bool = False

    @property
    def level(self) -> str:
        """给日志用的级别：ok / warn（不阻断）/ reject（严格模式下拒绝）。"""
        if self.ok:
            return "ok"
        return "reject" if self.strict else "warn"

    def message(self) -> str:
        if self.ok:
            return f"桥洞情报正常：A={self.zone_a} B={self.zone_b}"
        return "；".join(self.problems)


def check_bridge_pair(
    bridge_a: Optional[str],
    bridge_b: Optional[str],
    zones: Optional[BridgeZones] = None,
    grid=None,
    strict: bool = False,
) -> BridgeCheck:
    """校验两处桥洞的可通行格号是否自洽。

    检查项：
        1. 格号本身是否合法（越界/写错）
        2. 是否落在桥洞区域内（区域外说明无人机看错了或我们的布点假设不对）
        3. 两处是否落进了**同一个**区域（那就是同一处桥洞报了两次）
        4. 两处是否同一个格号（重复）
    """
    z = zones or BridgeZones()
    problems: List[str] = []

    a = (bridge_a or "").strip().upper() or None
    b = (bridge_b or "").strip().upper() or None

    for label, cell in (("A 段", a), ("B 段", b)):
        if cell is None:
            problems.append(f"{label} 缺失（无人机还没侦察到该处桥洞）")
            continue
        try:
            cid = CellID.parse(cell)
        except Exception:                                   # noqa: BLE001
            problems.append(f"{label} 格号无法解析: {cell!r}")
            continue
        if grid is not None and not grid.contains(cid):
            problems.append(f"{label} {cell} 超出场地范围")

    if a and b and a == b:
        problems.append(f"两处桥洞报成了同一格 {a}")

    za, zb = z.zone_of(a) if a else None, z.zone_of(b) if b else None
    if a and za is None:
        problems.append(f"A 段 {a} 不在推断的桥洞区域内（{z.describe()}）")
    if b and zb is None:
        problems.append(f"B 段 {b} 不在推断的桥洞区域内（{z.describe()}）")
    if za is not None and za == zb:
        problems.append(f"两处桥洞落在同一区域（都在 {za.upper()} 段），应各占一处")

    return BridgeCheck(
        ok=not problems, problems=problems, zone_a=za, zone_b=zb, strict=strict
    )


def zone_cells_map(zones: Optional[BridgeZones] = None) -> Dict[str, Tuple[str, ...]]:
    z = zones or BridgeZones()
    return {"a": z.zone_a, "b": z.zone_b}


def render_zones(zones: Optional[BridgeZones] = None, highlight: Iterable[str] = ()) -> str:
    """把桥洞布点画成一张小示意图（现场核对用）。"""
    z = zones or BridgeZones()
    marks = {c.strip().upper() for c in highlight}
    lines = [f"桥洞布点（默认推断）：{z.describe()}", ""]
    for r in range(z.row_start, z.row_start + z.rows):
        cell = f"{z.col}{r}"
        seg = "A" if cell in z.zone_a else "B"
        mark = "  <== 本次可通行" if cell in marks else ""
        lines.append(f"  {cell:>4} [{seg}]{mark}")
    return "\n".join(lines)


def pick_passable(zones: Optional[BridgeZones] = None, which: str = "a") -> Sequence[str]:
    """返回某个桥洞的所有候选格号（现场人工指定可通行格时可用）。"""
    return (zones or BridgeZones()).cells(which)
