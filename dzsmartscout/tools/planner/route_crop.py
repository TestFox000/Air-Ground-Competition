#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""route_crop.py —— L0 过渡线：按「车当前格」裁剪机侧整线，再经 A 通道下发

对应决策层方案 §7.1（L0）+ §12.2 卡 **t5e**，是 R1 裁决里"L0 只作 bench 对照"的那条线：

::

    机侧算整线（vendor planner, engine 从 config 读）
            │  token 串（44 token 现役）
            ▼
    car_state{block,yaw_deg,phase}   ← /dzlink/car_to_drone（车 1~2 Hz 回传）
            │
            ▼  ★ 本模块：定位车当前格 → 裁掉前缀 → 自检（不回头/不压已占格）
    cropped route = "E10 E9 D9 C9 … N5,y"      ← 首点 = 车当前格
            │
            ▼
    A 通道 /nav/waypoint（RELIABLE，复用车侧契约校验 + dz_send_waypoint 下发）

## 为什么是"裁剪"而不是"重规划"（L0 的能力边界，别当成终局）

L0 拿不到「已压雷 / 已过区域」的语义（`plan_route` 签名无 `already_covered` 入参），
所以它只能做**几何上的前缀裁剪**：把车已经走过的那段从整线里切掉，让车按剩余后缀继续走。

* 能做到：车不再被命令回到起点（H1）；已压过的雷不会被要求回头再压（H2，因为它在被裁掉的前缀里）。
* 做不到：如果车**实际走的路**与机侧重算的整线**顺序不同**（例如车中途换了桥洞），
  某格已在车侧压过、却在裁剪后的后缀里 —— L0 **无法自己修**，只能**报冲突拒绝下发**。
  这条就是 §7.1 表格 H2 的实测落点，见 `check_conflicts()` / 冲突类型 `repress`。

## 三条硬规则（本模块的全部"智能"就这三条）

1. **只裁前缀，绝不重排、绝不改 token 原文**：输出恒为 `tokens[i:]`，
   首点 = 车当前格（车侧 `_load_tokens` 把首点装成 `P_ROUTE_INDEX=0`、`_next_wp=1`
   ⇒ 车直接走第 2 点，理论上不掉头）。
2. **拒绝优先于猜测**：锚点定不下来（车格不在线里 / 同一格出现多次且航向也无法区分）、
   phase 不是 DRIVE/APPROACH、后缀里出现已占格 —— 一律 `ok=False` 并给出原因，
   绝不"猜一个差不多的"就往车上发（一条错线会让车"收到但行为不明"）。
3. **UNKNOWN ≠ ABSENT**：`coverage` 只在 `complete=true` 时才给出负证据（ABSENT 格），
   `complete=false` / 旧报文无 coverage ⇒ 一律当"没扫过"，不减少未知格计数。
   该语义**不在本模块重实现**，直接消费 `perception.protocol.field_verify_absent`
   （= t5a/t10 的单一真源）。

## 只读边界

* `dzrobot_v2/**` 只读（车侧契约、config、car_state 字段都从那里读，不写）。
* `vendor/**` 只读（真源同步产物，手工改会在 `--verify-only` 上爆）。
* 本模块**不 import rclpy**：跑规划/下发都走既有入口（`dz_send_waypoint.py route`、
  `dzlink_supervisor.py` 的 cmd-file JSONL），Windows 本机可全链路 dry-run。

## 用法

::

    # ① 本地干跑（不连 ROS）：拿 car_state 裁剪，只打印报文
    python dzsmartscout\tools\planner\route_crop.py crop \
        --from-plan --car-block E10 --car-yaw 90 --car-phase DRIVE

    # ② 用真实回传（supervisor 日志或 listen 输出）里的最后一条 car_state
    python dzsmartscout\tools\planner\route_crop.py crop --from-plan \
        --car-state ~/dzlink_logs/supervisor.jsonl --pressed D9 H10

    # ③ 真发（A 通道 /nav/waypoint，RELIABLE ×3）
    python dzsmartscout\tools\planner\route_crop.py crop --from-plan \
        --car-state ./car_state.jsonl --publish

    # ④ 只出「cmd-file 的一行」（常驻 supervisor 走这条，不新起 ROS 进程）
    python dzsmartscout\tools\planner\route_crop.py crop --from-plan \
        --car-block E10 --car-yaw 90 --emit cmd

退出码：0 = 裁出可下发路线 / 1 = 拒绝（冲突或锚点不确定，**不要发**）/ 2 = 用法或路线文本错。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import re
import subprocess
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

HERE = pathlib.Path(__file__).resolve().parent            # dzsmartscout/tools/planner
TOOLS = HERE.parent                                       # dzsmartscout/tools
VENDOR = HERE / "vendor" / "dzbrain_ws"

# --------------------------------------------------------------------------- #
#  路线语法（只解析，不重新定义语义 —— 形状判定以车侧 route/parser.py 为准）
# --------------------------------------------------------------------------- #
#: 格名：★ 交替顺序必须 `1[01]|[1-9]`（先长后短），否则 `K10` 的 `1` 会被先吃掉
BLOCK_RE = re.compile(r"^[A-N](?:1[01]|[1-9])$")
#: token 头：格名后必须紧跟结尾 / `*` / `,` / `:`（`G10*`、`N5,y`、`F3:SCAN_STOP`）
_TOKEN_BLOCK_RE = re.compile(r"^(?P<block>[A-N](?:1[01]|[1-9]))(?=$|[*,:])")
#: token 尾巴白名单（★ 顺序固定：`*` → `:SCAN_STOP`/`:SCAN`/`:PIVOT` → `,x|,y`）。
#: 有白名单才不会把 `N5,x,y`（两个朝向后缀）当成合法 token 放过去 —— 车侧会"收到但行为不明"。
#: 2026-10-24：补 `:PIVOT`/`:SCAN` —— 第六图 mission_blocks（车侧现役 68 token）已用
#: `:PIVOT` 声明折返点（`route/parser.py`/`route_check` 侧早已接受），本白名单没跟上，
#: L0 过渡线裁剪把车自己的现役路线判成非法 token（实测：I4:PIVOT/G4:PIVOT/A1:PIVOT 全拒）。
_TAIL_RE = re.compile(r"^(?:\*)?(?:[:](?:SCAN_STOP|SCAN|PIVOT))?(?:,[XY])?$")
#: 车侧 `route/parser.py` 认的命令前缀（`多点 A B` 之类）
PREFIX_WORDS = ("多点", "多航点", "路线", "route", "multi", "waypoints", "单点", "single", "goto")
#: ★ 路线文本的分隔符**不含逗号**：`,` 在路线里是语义（`N5,y` 的朝向后缀）
SEP_RE = re.compile(r"[\s;|]+")
#: 格名**参数**（`--pressed` / `--occupied` / 情报里的格名表）的分隔符：逗号、顿号都当分隔
BLOCK_SEP_RE = re.compile(r"[\s,;|、]+")

#: 允许被裁剪打断的 phase —— 打靶（STOP）/收尾（FINISH/SAFE）/待发（IDLE/HOLD）一律不动路线。
#: 依据：`core/blackboard.py:28` 的 phase 枚举；`_load_tokens` 会重编译路径并归零进度。
CROP_SAFE_PHASES: Tuple[str, ...] = ("DRIVE", "APPROACH")

#: 交接处航向判定阈值（度）
HANDOVER_WARN_DEG = 90.0      # > 90° ⇒ 需要先掉头（车侧 allow_reverse=false，掉头靠多点转弯）
HANDOVER_REVERSAL_DEG = 135.0  # > 135° ⇒ 基本是"往回走"，判为必须拒绝的回头段


class CropError(ValueError):
    """路线文本/参数层面就不可裁（用法错误）。**冲突类拒绝走 CropResult.ok=False**。"""


# --------------------------------------------------------------------------- #
#  token 工具
# --------------------------------------------------------------------------- #
def _split_head(tok: Any) -> Tuple[str, str]:
    """token → `(head 前缀 R:/SHOOT@, 去掉 head 后的原文)`。"""
    t = str(tok).strip()
    u = t.upper()
    n = 0
    if u.startswith("R:"):
        n = 2
    if u[n:].startswith("SHOOT@"):
        n += 6
    return t[:n], t[n:]


def token_block(tok: Any) -> Optional[str]:
    """token → 格名（`G10*` / `N5,y` / `R:F11` / `SHOOT@G11` → 格名）；不是**合法** token 返回 None。

    "合法" = 格名 + 白名单尾巴（`*` / `:SCAN_STOP` / `,x|,y`）。`N5,x,y`、`A12`、`Z1` 一律 None。
    """
    _head, body = _split_head(tok)
    u = body.upper()
    m = _TOKEN_BLOCK_RE.match(u)
    if m is None:
        return None
    if not _TAIL_RE.match(u[m.end():]):
        return None
    return m.group("block")


def token_tail(tok: Any) -> str:
    """token 去掉格名后的尾巴（**原样大小写**：`,y` / `*` / `:SCAN_STOP` / `R:`）。"""
    head, body = _split_head(tok)
    u = body.upper()
    m = _TOKEN_BLOCK_RE.match(u)
    if m is None:
        return ""
    return head + body[m.end():]


def split_route(text: Any) -> List[str]:
    """路线文本 → token 列表（去前缀词、去分隔符）；只支持裸航点/多点，不支持 reset:/load:。"""
    body = " ".join(str(text or "").replace(";", " ").replace("|", " ").split())
    if not body:
        raise CropError("路线为空")
    toks = body.split(" ")
    if toks[0].lower() in PREFIX_WORDS:
        toks = toks[1:]
    if not toks:
        raise CropError("只有前缀词（多点/route…），没有航点")
    head = toks[0].lower()
    if head.startswith("reset:") or head.startswith("load:"):
        raise CropError(f"{toks[0]!r} 不是航点序列：L0 只裁剪裸航点/多点路线"
                        "（reset:/load: 的语义是复位与读文件，不能按后缀裁）")
    bad = [t for t in toks if token_block(t) is None]
    if bad:
        raise CropError(f"有不是格名的 token：{bad}（合法：A1…N11 / G10* / N5,y / F3:SCAN_STOP）")
    return toks


def blocks_of(tokens: Sequence[str]) -> List[str]:
    """token 序列 → 格名序列（长度必然一致，非法 token 会抛错）。"""
    out: List[str] = []
    for t in tokens:
        b = token_block(t)
        if b is None:
            raise CropError(f"token {t!r} 取不出格名")
        out.append(b)
    return out


def norm_blocks(values: Optional[Iterable[Any]]) -> List[str]:
    """任意分隔形式的格名列表 → 去空/大写/去重/校验；`None` ⇒ 空表。"""
    out: List[str] = []
    for v in values or ():
        if v is None:
            continue
        for part in BLOCK_SEP_RE.split(str(v)):
            b = part.strip().upper()
            if not b:
                continue
            if not BLOCK_RE.match(b):
                raise CropError(f"格名非法：{part!r}（应为 A1…N11）")
            if b not in out:
                out.append(b)
    return out


# --------------------------------------------------------------------------- #
#  几何（复用 perception.grid 的格↔米 口径，不另立一套坐标系）
# --------------------------------------------------------------------------- #
_GRID_CACHE: Dict[str, Any] = {}


def grid():
    """`perception.grid.GridSpec`（14×11、0.35 m/格）。import 失败给可执行的提示。"""
    if "spec" not in _GRID_CACHE:
        if str(TOOLS) not in sys.path:
            sys.path.insert(0, str(TOOLS))
        try:
            from perception.grid import GridSpec             # noqa: PLC0415
        except Exception as exc:                             # noqa: BLE001
            raise CropError(
                "拿不到 perception.grid.GridSpec（格↔米换算的唯一真源）：%s\n"
                "  修：确认 dzsmartscout/tools/perception 完整，或 "
                "set PYTHONPATH=dzsmartscout\\tools" % exc) from exc
        _GRID_CACHE["spec"] = GridSpec()
    return _GRID_CACHE["spec"]


def wrap180(deg: float) -> float:
    return (float(deg) + 180.0) % 360.0 - 180.0


def bearing_deg(a: Sequence[float], b: Sequence[float]) -> float:
    """a→b 的方位角（度，atan2(dy,dx)，与车侧 `core/geometry.py heading_of` 同口径）。"""
    return math.degrees(math.atan2(float(b[1]) - float(a[1]), float(b[0]) - float(a[0])))


def cell_xy(block: str, spec: Any = None) -> Tuple[float, float]:
    return (spec or grid()).block_to_meters(str(block).strip().upper())


# --------------------------------------------------------------------------- #
#  车状态（car_state）—— 字段口径 = 车侧 drone.py 的 car_state 报文
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class CarState:
    """一条车回传的位姿信念。`yaw_deg` 语义见 perception/car_pose.py：

    ``0`` = 车头朝 +x（行号减小，上半场）；``+90`` = 朝 +y（列号减小）；``±180`` = 朝 −x。
    """

    block: str
    yaw_deg: Optional[float] = None
    phase: str = ""
    ts: Optional[float] = None
    source: str = ""

    def xy(self, spec: Any = None) -> Tuple[float, float]:
        return cell_xy(self.block, spec)

    def to_json(self) -> Dict[str, Any]:
        return {"block": self.block, "yaw_deg": self.yaw_deg, "phase": self.phase,
                "ts": self.ts, "source": self.source}


def car_state_from_payload(obj: Any, source: str = "") -> CarState:
    """任意 `car_state` 形态 → CarState。认三种来源：

    * 车侧原生报文 `{"type":"car_state","block":"E10","yaw_deg":90,"phase":"DRIVE"}`
    * supervisor 日志行（`{"kind":"car_state","block":"E10",…}`，见 `JsonlLogger`）
    * 宽容形态 `{"block":"E10"}`（只有格名也能裁，但会少掉航向消歧能力）

    **缺 block / 格名非法一律抛错**（不猜 —— 猜错就是"按错误的格裁前缀"，比不裁更危险）。
    """
    if not isinstance(obj, Mapping):
        raise CropError(f"car_state 不是 JSON 对象（{type(obj).__name__}）")
    kind = str(obj.get("type") or obj.get("kind") or "").strip().lower()
    if kind and kind not in ("car_state", "car-state", "carstate"):
        raise CropError(f"这条不是 car_state（type/kind={kind!r}）")
    block = token_block(obj.get("block"))
    if block is None:
        raise CropError(f"car_state 的 block 缺失或非法：{obj.get('block')!r}")
    yaw = obj.get("yaw_deg", obj.get("yaw"))
    try:
        yaw_f = None if yaw is None else float(yaw)
    except (TypeError, ValueError):
        raise CropError(f"car_state 的 yaw_deg 不是数字：{yaw!r}") from None
    ts = obj.get("ts")
    try:
        ts_f = None if ts is None else float(ts)
    except (TypeError, ValueError):
        ts_f = None
    return CarState(block=block, yaw_deg=yaw_f, phase=str(obj.get("phase") or "").strip().upper(),
                    ts=ts_f, source=source)


def car_state_from_lines(lines: Iterable[str], source: str = "") -> Optional[CarState]:
    """从若干行 JSON（JSONL / 日志 / listen 输出）里取**最后一条** car_state。"""
    last: Optional[CarState] = None
    for ln in lines:
        s = str(ln).strip().lstrip("\ufeff")
        if not s or s.startswith("#"):
            continue
        try:
            obj = json.loads(s)
        except Exception:                                   # noqa: BLE001
            continue
        if not isinstance(obj, Mapping):
            continue
        try:
            last = car_state_from_payload(obj, source=source or "lines")
        except CropError:
            continue                                        # 非 car_state 的行直接跳过
    return last


def car_state_from_file(path: Any) -> CarState:
    """读文件（JSONL / supervisor 日志 / 单行 JSON）→ 最后一条 car_state。"""
    p = pathlib.Path(str(path)).expanduser()
    if not p.is_file():
        raise CropError(f"car_state 文件不存在：{p}")
    st = car_state_from_lines(p.read_text(encoding="utf-8", errors="replace").splitlines(),
                              source=str(p))
    if st is None:
        raise CropError(f"{p} 里没有一条可解析的 car_state（要 "
                        + "{'block':'E10'} 这样的行）")
    return st


# --------------------------------------------------------------------------- #
#  锚点定位（车当前格 → 整线里的下标）
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class AnchorFix:
    """车当前格在整线里的位置。`mode` 说明"凭什么定的"，便于现场复盘。"""

    index: int
    block: str
    mode: str                       # exact | yaw_disambiguated | nearest
    candidates: Tuple[int, ...] = ()
    turn_in_deg: Optional[float] = None     # 入线航向 vs 车头（度，绝对值）
    notes: Tuple[str, ...] = ()

    def to_json(self) -> Dict[str, Any]:
        return {"index": self.index, "block": self.block, "mode": self.mode,
                "candidates": list(self.candidates),
                "turn_in_deg": None if self.turn_in_deg is None
                else round(self.turn_in_deg, 1),
                "notes": list(self.notes)}


def _course_through(blocks: Sequence[str], i: int, spec: Any = None) -> Optional[float]:
    """第 i 格的"行进方向"（度）：优先进入方向 (i-1→i)，起点格退化用离开方向 (i→i+1)。"""
    spec = spec or grid()
    pts = [spec.block_to_meters(b) for b in blocks]
    if i > 0:
        return bearing_deg(pts[i - 1], pts[i])
    if len(pts) > 1:
        return bearing_deg(pts[0], pts[1])
    return None


def locate_anchor(tokens: Sequence[str], car: CarState, *,
                  allow_nearest: bool = False,
                  spec: Any = None) -> Tuple[Optional[AnchorFix], List[str]]:
    """车当前格 → 整线下标。返回 `(fix, 拒绝原因)`；拒绝时 fix=None（不猜）。

    同一格在整线里出现多次时（换洞/折返路线会出现），用**车头航向**消歧：
    取「到达该格的行进方向」与车头夹角最小的那个下标；夹角差 < 5° 判为**不可消歧**。
    `allow_nearest=True` 时允许车格不在整线里（车被外部改道），落到最近格的锚点并标注。
    """
    toks = list(tokens)
    if len(toks) < 2:
        return None, [f"整线只有 {len(toks)} 个 token，没有可裁的段"]
    blocks = blocks_of(toks)
    hits = [i for i, b in enumerate(blocks) if b == car.block]

    if not hits:
        if not allow_nearest:
            return None, [f"车当前格 {car.block} 不在整线里（{len(blocks)} 个格）⇒ 无法定位锚点；"
                          "若车确实被外部改道，用 --allow-nearest 显式接受「就近格」近似"]
        spec = spec or grid()
        cx, cy = cell_xy(car.block, spec)
        best = min(range(len(blocks)),
                   key=lambda i: (lambda p: math.hypot(p[0] - cx, p[1] - cy))(
                       spec.block_to_meters(blocks[i])))
        dist = math.hypot(spec.block_to_meters(blocks[best])[0] - cx,
                          spec.block_to_meters(blocks[best])[1] - cy)
        return AnchorFix(index=best, block=blocks[best], mode="nearest",
                         candidates=(best,),
                         notes=(f"车格 {car.block} 不在整线；就近取 {blocks[best]}"
                                f"（相距 {dist:.2f} m，--allow-nearest 显式接受）",)), []

    if len(hits) == 1:
        idx = hits[0]
        course = _course_through(blocks, idx, spec)
        turn = None if (course is None or car.yaw_deg is None) else abs(
            wrap180(course - car.yaw_deg))
        return AnchorFix(index=idx, block=car.block, mode="exact", candidates=(idx,),
                         turn_in_deg=turn), []

    # ---- 同一格多次出现：用航向消歧 ----
    if car.yaw_deg is None:
        return None, [f"车格 {car.block} 在整线里出现 {len(hits)} 次且这条 car_state 没有 yaw_deg "
                      "⇒ 无法消歧（补 yaw_deg，或确认整线没有重复格）"]
    scored = []
    for i in hits:
        course = _course_through(blocks, i, spec)
        if course is None:
            continue
        scored.append((abs(wrap180(course - car.yaw_deg)), i))
    if not scored:
        return None, [f"车格 {car.block} 出现 {len(hits)} 次且都取不到行进方向 ⇒ 无法消歧"]
    scored.sort()
    best_turn, best_i = scored[0]
    if len(scored) > 1 and abs(scored[1][0] - best_turn) < 5.0:
        return None, [f"车格 {car.block} 出现 {len(hits)} 次，按航向也分不出来"
                      f"（前两名夹角差 < 5°：{[(round(t, 1), i) for t, i in scored]}）⇒ 拒绝"]
    return AnchorFix(index=best_i, block=car.block, mode="yaw_disambiguated",
                     candidates=tuple(hits),
                     turn_in_deg=best_turn,
                     notes=(f"同格 {len(hits)} 次出现，按车头 {car.yaw_deg:.1f}° 选下标 {best_i}"
                            f"（入线夹角 {best_turn:.1f}°）",)), []


# --------------------------------------------------------------------------- #
#  覆盖/负证据（t5a+t10）：complete=true 才有 ABSENT
# --------------------------------------------------------------------------- #
def cleared_from_field_verify(payload: Any, *, spec: Any = None
                              ) -> Tuple[Optional[Set[str]], List[str]]:
    """一条 `field_verify` 载荷 → `(已判空的格集合 | None, 说明)`。

    * `coverage.complete=true`  ⇒ 返回 `coverage.cells − 已报出的雷/障/靶`（= ABSENT，负面证据）；
    * `coverage.complete=false` ⇒ `None` + 说明（**片内未报仍是 UNKNOWN，不得当空的**）；
    * 载荷里没有 `coverage`（旧报文）⇒ `None` + 说明。

    语义**不在本模块重实现**：直接调 `perception.protocol.field_verify_absent`
    （t10 修的正是"空表 ≠ 没扫过"这条歧义）。
    """
    if payload is None:
        return None, ["没有给 field_verify ⇒ 无负证据，全部格按 UNKNOWN 计"]
    if str(TOOLS) not in sys.path:
        sys.path.insert(0, str(TOOLS))
    from perception.protocol import field_verify_absent        # noqa: PLC0415
    try:
        absent = field_verify_absent(payload, spec=spec)
    except ValueError as exc:
        return None, [f"无负证据：{exc}"]
    if absent is None:
        return None, ["coverage.complete=false ⇒ 片内未报格仍是 UNKNOWN"
                      "（t10 语义：不完整的片绝不产生 ABSENT）"]
    return set(absent), [f"coverage.complete=true ⇒ {len(absent)} 格被判空（ABSENT）"]


# --------------------------------------------------------------------------- #
#  裁剪主体
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class CropResult:
    """一次裁剪的结论。`ok=False` 时 `reject` 说明为什么不该把这条线发出去。"""

    ok: bool
    route: Tuple[str, ...] = ()            # 裁剪后 token（原文，未改写）
    anchor: Optional[AnchorFix] = None
    dropped: Tuple[str, ...] = ()          # 被裁掉的前缀 token（证据）
    reject: Tuple[str, ...] = ()           # 拒绝原因（ok=False）
    conflicts: Tuple[Dict[str, Any], ...] = ()
    warnings: Tuple[str, ...] = ()
    stats: Dict[str, Any] = field(default_factory=dict)

    def route_text(self) -> str:
        return " ".join(self.route)

    def cmd_json(self) -> str:
        """supervisor `--cmd-file` 的一行（`{"type":"route","data":…}`）。"""
        return json.dumps({"type": "route", "data": self.route_text()},
                          ensure_ascii=False, separators=(",", ":"))

    def to_json(self) -> Dict[str, Any]:
        """★ 被拒绝时 `route` 恒为空串：下游拿到的 JSON 里**永远不存在**一条"被拒绝却看着能用"
        的路线（候选线另放 `route_rejected`，只给人看）。"""
        return {"ok": self.ok, "route": self.route_text() if self.ok else "",
                "route_rejected": "" if self.ok else self.route_text(),
                "n_tokens": len(self.route),
                "anchor": None if self.anchor is None else self.anchor.to_json(),
                "dropped": list(self.dropped), "reject": list(self.reject),
                "conflicts": [dict(c) for c in self.conflicts],
                "warnings": list(self.warnings), "stats": dict(self.stats)}


def handover_check(car_xy: Sequence[float], car_yaw_deg: Optional[float],
                   blocks: Sequence[str], spec: Any = None) -> Dict[str, Any]:
    """交接几何：车头 vs「首点→下一格」。车侧 `_next_wp=1` ⇒ 纯追踪的目标就是第 2 个航点。"""
    spec = spec or grid()
    if len(blocks) < 2:
        return {"first_leg_deg": None, "turn_deg": None, "verdict": "no_leg"}
    p0 = spec.block_to_meters(blocks[0])
    p1 = spec.block_to_meters(blocks[1])
    leg = bearing_deg(p0, p1)
    out: Dict[str, Any] = {"first_leg_deg": round(leg, 1), "turn_deg": None,
                           "verdict": "unknown_yaw"}
    if car_yaw_deg is None:
        return out
    turn = abs(wrap180(leg - float(car_yaw_deg)))
    out["turn_deg"] = round(turn, 1)
    out["car_to_first_m"] = round(math.hypot(p0[0] - float(car_xy[0]),
                                             p0[1] - float(car_xy[1])), 3)
    if turn > HANDOVER_REVERSAL_DEG:
        out["verdict"] = "reversal"
    elif turn > HANDOVER_WARN_DEG:
        out["verdict"] = "warn"
    else:
        out["verdict"] = "ok"
    return out


def crop_route(tokens: Sequence[str], car: CarState, *,
               pressed: Optional[Iterable[Any]] = None,
               occupied: Optional[Iterable[Any]] = None,
               cleared: Optional[Set[str]] = None,
               allow_nearest: bool = False,
               allow_revisit: bool = False,
               spec: Any = None) -> CropResult:
    """按车当前格裁掉前缀，并对结果做「不回头 / 不压已占格」自检。

    * `pressed`  —— 车侧**已经压过**的格（雷/靶；一手事实，可来自台账或 `--pressed`）；
    * `occupied` —— 不可进入的格（障碍/场外；同样一手，通常来自地图或现场标定）；
    * `cleared`  —— 完整片 coverage 判空的格（负面证据）。**只用于解释，不用于放行**：
      车侧的"已压"是一手事实，机侧"没看到"是二手观测，冲突时以车侧为准（报 contradiction）。

    冲突（`ok=False`）：
      · `repress`  —— 裁剪后的后缀里有车已压过的格 ⇒ 会要求"回头再压"（H2 的真风险）；
      · `occupied` —— 后缀里有不可进入的格；
      · `revisit`  —— 后缀里重走了前缀的格（纯几何回头段，默认也算错误）；
      · `phase`    —— 当前阶段不该改路线（打靶/收尾/待发）；
      · `handover_reversal` —— 交接处航向与首段夹角 > 135°（等于命令车往回开）。
    """
    spec = spec or grid()
    toks = list(tokens)
    pressed_s = set(norm_blocks(pressed))
    occupied_s = set(norm_blocks(occupied))
    warnings: List[str] = []

    # ---- ① phase 闸门：打靶/收尾阶段不许改线 ----
    phase = (car.phase or "").strip().upper()
    if phase and phase not in CROP_SAFE_PHASES:
        return CropResult(ok=False, reject=(f"phase={phase} 不是 {'/'.join(CROP_SAFE_PHASES)}："
                                            "打靶/收尾/待发阶段改线会让车重启路径并归零进度",),
                          anchor=None, stats={"phase": phase})

    # ---- ② 锚点 ----
    fix, why = locate_anchor(toks, car, allow_nearest=allow_nearest, spec=spec)
    if fix is None:
        return CropResult(ok=False, reject=tuple(why), stats={"phase": phase})
    blocks = blocks_of(toks)
    route = tuple(toks[fix.index:])
    prefix_blocks = blocks[:fix.index]
    suffix_blocks = blocks[fix.index + 1:]
    dropped = tuple(toks[:fix.index])

    # ---- ③ 冲突检查 ----
    conflicts: List[Dict[str, Any]] = []
    for off, b in enumerate(suffix_blocks, start=1):
        if b in pressed_s:
            conflicts.append({"kind": "repress", "block": b, "index": fix.index + off,
                              "why": "车已压过该格（pressed）⇒ 裁剪后会要求回头再压",
                              "fix": "L0 无解：需重规划（L2）或本轮跳过该段"})
        elif b in occupied_s:
            conflicts.append({"kind": "occupied", "block": b, "index": fix.index + off,
                              "why": "该格不可进入（occupied）", "fix": "换路线（L2）"})
        if b in prefix_blocks and not allow_revisit:
            conflicts.append({"kind": "revisit", "block": b, "index": fix.index + off,
                              "why": "重走已走过的格（回头段）",
                              "fix": "确认整线是否有重复格；确属设计需 --allow-revisit"})
    # 后缀**内部**的重复格（含「回到当前格」）：车会在同一格走两遍 = 回头段
    seen: Set[str] = {blocks[fix.index]}
    for off, b in enumerate(suffix_blocks, start=1):
        if b in seen and not allow_revisit:
            conflicts.append({"kind": "revisit", "block": b, "index": fix.index + off,
                              "why": "裁剪后的路线在同一格出现两次（回头段/绕回当前格）",
                              "fix": "确认整线是否有重复格；确属设计需 --allow-revisit"})
        seen.add(b)
    # 交接航向
    handover = handover_check(car.xy(spec), car.yaw_deg, blocks[fix.index:], spec)
    if handover["verdict"] == "reversal":
        conflicts.append({"kind": "handover_reversal", "block": blocks[fix.index],
                          "index": fix.index,
                          "why": f"交接处夹角 {handover['turn_deg']}° > {HANDOVER_REVERSAL_DEG}°",
                          "fix": "锚点很可能取错了（车其实已过该格）"})
    elif handover["verdict"] == "warn":
        warnings.append(f"交接处夹角 {handover['turn_deg']}° > {HANDOVER_WARN_DEG}°："
                        "车需要先大幅转弯（不是倒车，但会慢）")
    if handover["verdict"] == "unknown_yaw":
        warnings.append("car_state 没有 yaw_deg ⇒ 交接航向未校验（H1 只能靠 bench 的近似模型背书）")

    # ---- ④ 覆盖率（负证据）----
    prefix_set = set(prefix_blocks)
    cleared_s = set(cleared or ())
    unknown = [b for b in suffix_blocks if b not in prefix_set and b not in cleared_s]
    contradictions = sorted(set(pressed_s | occupied_s) & cleared_s)
    if contradictions:
        warnings.append(f"情报冲突：{contradictions} 车侧记已占/已压，但完整片 coverage 判为空的"
                        " ⇒ 以车侧为准（不据此放行）")
    if not cleared_s:
        warnings.append(f"没有负证据（coverage）⇒ 后缀里 {len(unknown)} 格仍是黑图未知"
                        "（L0 的固有限制：拿不到「已扫过区域」语义）")

    # ---- ⑤ 语义提示（不拦，但必须让人看见）----
    anchor_tail = token_tail(toks[fix.index])
    if "*" in anchor_tail:
        warnings.append(f"锚点 token {toks[fix.index]!r} 带停站打靶标记：车侧 `_install_route` 一律"
                        "把 `_next_wp` 置 1（mission.py:2299）、`P_ROUTE_INDEX` 置 0（:2325）"
                        " ⇒ 首点的停站要么在 DRIVE 首拍就触发、要么被跳过（需车侧确认）")
    if ":SCAN_STOP" in anchor_tail.upper():
        warnings.append(f"锚点 token {toks[fix.index]!r} 带 :SCAN_STOP 语义，裁剪后含义待车侧确认")
    if len(route) < 2:
        warnings.append("裁剪后只剩 1 个 token（车就在最后一个航点）：下发等于原地待命")

    stats = {
        "phase": phase,
        "n_in": len(toks),
        "n_out": len(route),
        "n_dropped": len(dropped),
        "n_prefix_cells": len(prefix_blocks),
        "n_suffix_cells": len(suffix_blocks),
        "n_pressed": len(pressed_s),
        "n_occupied": len(occupied_s),
        "n_cleared": len(cleared_s),
        "n_unknown_ahead": len(unknown),
        "n_conflicts": len(conflicts),
        "crop_required": fix.index > 0,
        "anchor_mode": fix.mode,
        "handover": handover,
        "unknown_ahead": unknown,
        "dropped_prefix": list(prefix_blocks),
    }
    ok = not conflicts
    reject: Tuple[str, ...] = () if ok else tuple(
        f"{c['kind']}@{c['block']}: {c['why']}" for c in conflicts)
    return CropResult(ok=ok, route=route, anchor=fix, dropped=dropped, reject=reject,
                      conflicts=tuple(conflicts), warnings=tuple(warnings), stats=stats)


# --------------------------------------------------------------------------- #
#  车侧契约校验 + A 通道下发（复用车侧同一套形状判定与发送器，不新造）
# --------------------------------------------------------------------------- #
def link_dir() -> pathlib.Path:
    """`dz_send_waypoint.py` / `dzlink_supervisor.py` 所在目录（本机与无人机两种布局）。"""
    env = os.environ.get("DZ_LINK_DIR")
    cands: List[pathlib.Path] = []
    if env:
        cands.append(pathlib.Path(env).expanduser())
    cands += [TOOLS / "link", pathlib.Path("~/dzlink_tools").expanduser()]
    for c in cands:
        if (c / "dz_send_waypoint.py").is_file():
            return c
    raise CropError("找不到 dz_send_waypoint.py（在本机应位于 dzsmartscout/tools/link；"
                    "机上应位于 ~/dzlink_tools；也可 set DZ_LINK_DIR 显式指定）")


def validate_route_contract(text: str) -> Tuple[bool, List[str]]:
    """按车侧 `route/parser.py` 的形状校验一条路线（复用 supervisor 的实现，不抄正则）。"""
    d = link_dir()
    if str(d) not in sys.path:
        sys.path.insert(0, str(d))
    import dzlink_supervisor as sup                          # noqa: PLC0415
    return sup.validate_route(text)


def send_route(text: str, *, repeat: int = 3, wait_match_s: float = 3.0,
               dry_run: bool = False, timeout: float = 120.0) -> Tuple[int, str]:
    """经 A 通道下发（复用 `dz_send_waypoint.py route`，RELIABLE ×repeat）。

    返回 `(rc, 输出)`；rc 语义沿用发送器：0 正常 / 2 无 ROS 环境 / 3 没有车侧订阅者。
    """
    d = link_dir()
    cmd = [sys.executable, str(d / "dz_send_waypoint.py"), "route", text,
           "--repeat", str(int(repeat)), "--wait-match", str(float(wait_match_s))]
    if dry_run:
        cmd.append("--dry-run")
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1",
               PYTHONDONTWRITEBYTECODE="1")
    p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                       errors="replace", cwd=str(d), env=env, timeout=timeout)
    return p.returncode, (p.stdout or "") + (p.stderr or "")


def plan_tokens(*, map_path: Optional[str] = None, cross_lower: Optional[str] = None,
                cross_upper: Optional[str] = None) -> List[str]:
    """机侧算整线（复用 `dzlink_mission_ctl` 的 plan→门禁 管线：引擎从 config 读）。"""
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    import dzlink_mission_ctl as ctl                         # noqa: PLC0415
    engine, _ = ctl._read_engine()
    rc, payload, _dt = ctl._plan(engine, map_path=map_path, cross_lower=cross_lower,
                                 cross_upper=cross_upper)
    if rc != 0:
        raise CropError(f"机侧规划失败（rc={rc}）：{json.dumps(payload, ensure_ascii=False)[:600]}")
    toks = ctl._tokens_from_payload(payload)
    if not toks:
        raise CropError("规划成功但没拿到 token 列表")
    return toks


# --------------------------------------------------------------------------- #
#  人读报告
# --------------------------------------------------------------------------- #
def format_report(res: CropResult, *, route: Optional[str] = None) -> str:
    st = res.stats or {}
    lines = ["=== route_crop（L0：按车当前格裁前缀）==="]
    if res.anchor is not None:
        a = res.anchor
        lines.append(f"锚点：{a.block} → 下标 {a.index}（模式 {a.mode}）"
                     + (f"，候选 {list(a.candidates)}" if a.candidates else ""))
    if "n_in" in st:
        lines.append(f"整线 {st.get('n_in')} token → 裁剪后 {st.get('n_out')} token"
                     f"（裁掉 {st.get('n_dropped')}，前缀格 {st.get('n_prefix_cells')}）")
        hv = st.get("handover") or {}
        if hv:
            lines.append(f"交接几何：首段方位 {hv.get('first_leg_deg')}°，车头夹角 "
                         f"{hv.get('turn_deg')}° ⇒ {hv.get('verdict')}")
        lines.append(f"黑图未知格（后缀内、非已走过、非判空）：{st.get('n_unknown_ahead')}"
                     f"  （负证据格 {st.get('n_cleared')}）")
    lines.append(f"结论：{'✅ 可下发' if res.ok else '❌ 拒绝（不要发）'}")
    for r in res.reject:
        lines.append(f"  - 拒绝：{r}")
    for c in res.conflicts:
        lines.append(f"  - 冲突[{c['kind']}] {c['block']}（token#{c['index']}）：{c['why']}"
                     f" ⇒ {c['fix']}")
    for w in res.warnings:
        lines.append(f"  ⚠️  {w}")
    if res.ok:
        lines.append("裁剪后路线：" + res.route_text())
    if route:
        lines.append("原始整线  ：" + route)
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
#  CLI
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="L0：按车当前格裁剪机侧整线并（可选）经 /nav/waypoint 下发")
    sub = ap.add_subparsers(dest="cmd")

    p = sub.add_parser("crop", help="裁剪一条整线（默认只打印，不发）")
    src = p.add_argument_group("路线来源（三选一）")
    src.add_argument("--route", default=None, help='整线 token 串，如 "N10 M10 … N5,y"')
    src.add_argument("--route-file", default=None, help="整线 token 串所在文件（一行）")
    src.add_argument("--from-plan", action="store_true", help="调机侧规划器现算整线（引擎读 config）")
    src.add_argument("--map", default=None, help="配合 --from-plan：覆盖地图 yaml")
    src.add_argument("--cross-lower", default=None, help="配合 --from-plan：下半场穿洞格")
    src.add_argument("--cross-upper", default=None, help="配合 --from-plan：上半场穿洞格")

    veh = p.add_argument_group("车状态（二选一：显式 / 文件）")
    veh.add_argument("--car-block", default=None, help="车当前格（car_state.block），如 E10")
    veh.add_argument("--car-yaw", type=float, default=None, help="车头方位角（度，atan2(dy,dx)）")
    veh.add_argument("--car-phase", default="DRIVE", help="车 phase（默认 DRIVE；只允许 DRIVE/APPROACH）")
    veh.add_argument("--car-state", default=None,
                     help="car_state 文件（JSONL / supervisor 日志 / listen 输出），取最后一条")
    veh.add_argument("--allow-nearest", action="store_true",
                     help="车格不在整线里时，允许就近格近似（默认拒绝）")

    it = p.add_argument_group("已占格与负证据")
    it.add_argument("--pressed", nargs="*", default=None, help="车已压过的格（雷/靶）")
    it.add_argument("--occupied", nargs="*", default=None, help="不可进入的格（障碍/场外）")
    it.add_argument("--field-verify", default=None,
                    help="一条 field_verify JSON（t10 口径，含 coverage）文件，用来算负证据")
    it.add_argument("--allow-revisit", action="store_true", help="允许重走已走过的格（默认拒绝）")

    out = p.add_argument_group("输出与下发")
    out.add_argument("--emit", choices=("text", "json", "cmd", "argv"), default="text",
                     help="stdout 输出形态：text=路线串 / json=完整结论 / cmd=supervisor cmd-file 一行 / argv=shell 参数")
    out.add_argument("--report", action="store_true", help="把人类报告打到 stderr（默认在 emit!=json 时打）")
    out.add_argument("--publish", action="store_true", help="真的经 A 通道发出去（默认干跑）")
    out.add_argument("--repeat", type=int, default=3, help="下发重复遍数（默认 3，抵 VOLATILE 首帧丢）")
    out.add_argument("--wait-match", type=float, default=3.0, help="下发前等 DDS 匹配的秒数")
    return ap


def _route_from_args(args: argparse.Namespace) -> Tuple[List[str], str]:
    n_src = sum([bool(args.route), bool(args.route_file), bool(args.from_plan)])
    if n_src != 1:
        raise CropError("路线来源必须且只能给一个：--route / --route-file / --from-plan")
    if args.from_plan:
        toks = plan_tokens(map_path=args.map, cross_lower=args.cross_lower,
                           cross_upper=args.cross_upper)
        return toks, " ".join(toks)
    text = args.route
    if args.route_file:
        text = pathlib.Path(args.route_file).expanduser().read_text(
            encoding="utf-8", errors="replace").strip().splitlines()[0]
    toks = split_route(text)
    return toks, " ".join(toks)


def _car_from_args(args: argparse.Namespace) -> CarState:
    explicit = args.car_block is not None
    if explicit and args.car_state:
        raise CropError("--car-block 与 --car-state 只给一个（避免两个来源打架）")
    if args.car_state:
        return car_state_from_file(args.car_state)
    if not explicit:
        raise CropError("必须给车状态：--car-block（+ --car-yaw）或 --car-state <文件>")
    block = token_block(args.car_block)
    if block is None:
        raise CropError(f"--car-block 非法：{args.car_block!r}（应为 A1…N11）")
    return CarState(block=block, yaw_deg=args.car_yaw, phase=str(args.car_phase).upper(),
                    source="cli")


def _cleared_from_args(args: argparse.Namespace) -> Tuple[Optional[Set[str]], List[str]]:
    if not args.field_verify:
        return None, ["没有给 --field-verify ⇒ 无负证据，全部按 UNKNOWN 计"]
    p = pathlib.Path(args.field_verify).expanduser()
    if not p.is_file():
        raise CropError(f"--field-verify 文件不存在：{p}")
    txt = p.read_text(encoding="utf-8", errors="replace").strip()
    payload = None
    for ln in txt.splitlines():
        try:
            obj = json.loads(ln)
        except Exception:                                   # noqa: BLE001
            continue
        if isinstance(obj, Mapping) and str(obj.get("type", "")).lower() == "field_verify":
            payload = obj
    if payload is None:
        payload = json.loads(txt)
    return cleared_from_field_verify(payload)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = build_parser()
    args = ap.parse_args(list(argv) if argv is not None else sys.argv[1:])
    if args.cmd != "crop":
        ap.print_help()
        return 2
    try:
        toks, route_text = _route_from_args(args)
        car = _car_from_args(args)
        cleared, cov_notes = _cleared_from_args(args)
    except CropError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    res = crop_route(toks, car, pressed=args.pressed, occupied=args.occupied,
                     cleared=cleared, allow_nearest=args.allow_nearest,
                     allow_revisit=args.allow_revisit)
    res = CropResult(ok=res.ok, route=res.route, anchor=res.anchor, dropped=res.dropped,
                     reject=res.reject, conflicts=res.conflicts,
                     warnings=tuple(list(res.warnings) + cov_notes), stats=res.stats)

    show_report = args.report or args.emit != "json"
    if show_report:
        print(format_report(res, route=route_text), file=sys.stderr, flush=True)

    if args.emit == "json":
        print(json.dumps(res.to_json(), ensure_ascii=False, indent=1))
    elif args.emit == "cmd":
        print(res.cmd_json() if res.ok else "")
    elif args.emit == "argv":
        print(res.route_text() if res.ok else "")
    else:
        print(res.route_text() if res.ok else "")

    if not res.ok:
        return 1

    # 车侧契约校验（形状）：ours 是 suffix，理论上必然合法；校验是为了"不把自己的假设当事实"
    ok_shape, probs = validate_route_contract(res.route_text())
    if not ok_shape:
        print("ERROR: 裁剪结果没过车侧形状契约：" + "; ".join(probs), file=sys.stderr)
        return 1

    if args.publish:
        rc, out = send_route(res.route_text(), repeat=args.repeat,
                             wait_match_s=args.wait_match, dry_run=False)
        print(out.strip(), file=sys.stderr)
        if rc not in (0, 3):
            print(f"ERROR: 下发失败 rc={rc}", file=sys.stderr)
            return 1
        if rc == 3:
            print("⚠️ 发完了但没有车侧订阅者（车没起栈/域号/组播）", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
