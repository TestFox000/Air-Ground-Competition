# -*- coding: utf-8 -*-
"""航点 token 解析 —— 把旧 `follow_route_command`（path_executor.py:1098-1295，198 行）
从"5 个平行数组 + 手工同步"改写成**纯函数 → 不可变 Route**。

兼容性依据：`docs/survey_path_executor.md` §3「航点 token 语义」，行号级证据逐条对应。

语法总表（兼容旧协议，★ = 显式报错）：

| 写法 | 含义 | 旧协议行号 |
|---|---|---|
| `K9` | 普通航点，直线连到下一个点 | 1192, 1204 |
| `D3*` | 射击停站（到达后停车、找靶、稳定后放行开火） | 1182-1183, 1206 |
| `D3:SHOOT_STOP` / `D3:SHOOT` | 等价于 `D3*`（可读性写法，内部归一成 `*`） | 1174-1178 |
| `F2:SCAN_STOP` / `F2:SCAN` | 纯扫描停站（**不射击**） | 1179-1180 |
| `F8:PIVOT` | ★ 折返点**声明**（该顶点车头需转过 ≥135°，循迹层自动 PIVOT）—— 不是停车指令 | 新增 |
| `R:B5` | 倒车到位（该段线速度为负，车尾朝目标摆） | 1167-1169, 1208 |
| `R:B5*` | 倒车 + 射击（唯一合法顺序：`R:` 在前、`*` 在后） | 注释 1163-1166 |
| `SHOOT@G11` | **指令而非坐标**：调 shoot_planner 算候选链并整链插入，共用一个组号 | 1136-1161 |
| `N5,y` / `N5,x` | 终点朝向后缀，**只有最后一个 token 允许带** | 1108-1117；校验 375-380 |

边界写法的坑，本模块的对策：

- ⚠️ 非法 token **必须显式报错**：认不出的写法不能当普通格名往下走（越界格名要到几何
  计算时才炸，或干脆当成不存在的点）。本模块一律抛 `ValueError`，并由 `validate.py`
  的 R002 兜住"没经过 parser 的路线"。
- ⚠️ "去重"只在相邻同格时合并，且 shoot/scan **取并集**、`reverse` 不同则**绝不合并**
  （否则倒车段丢失）；射击标记不能搬来搬去。
- ⚠️ 插入 `SHOOT@` 候选链**不能在多个平行数组上各 insert 一次**（漏一个就静默错位）。
  本模块先算出整数索引区间，再对**已经构建好的** Waypoint 列表做一次切片拼接，天然不会错位。
"""

from __future__ import annotations

import os
import re
from typing import Any, Callable, Iterable

from ..core import geometry as geo
from .model import Route, RouteBuilder, Waypoint

__all__ = [
    "REVERSE_PREFIX",
    "SHOOT_SUFFIX",
    "SHOOT_AT_PREFIX",
    "SCAN_SUFFIXES",
    "PIVOT_SUFFIX",
    "SHOOT_SUFFIXES",
    "HEADING_SUFFIXES",
    "normalize_token",
    "parse_token",
    "parse_mission",
    "parse_reset",
    "format_route",
]

REVERSE_PREFIX = "R:"
SHOOT_SUFFIX = "*"
SHOOT_AT_PREFIX = "SHOOT@"

#: 归一化时一律折成 `*` 的射击后缀
SHOOT_SUFFIXES = frozenset({":SHOOT_STOP", ":SHOOT"})
#: 纯扫描停站后缀 —— 只剥后缀，**不射击**
SCAN_SUFFIXES = frozenset({":SCAN_STOP", ":SCAN"})
#: 扫描后缀的归一形态（`:SCAN_STOP` / `:SCAN` 都折成它，回写时短一档）
SCAN_CANON = ":SCAN"
#: ★ 折返点声明后缀（新增）：该顶点处**车头需要转过 ≥135°**，由循迹层自动 PIVOT 完成。
#: 与 `*` / `:SCAN` 一样只是**后缀声明**：不产生停站（`Waypoint.is_stop_point` 不含它）、
#: 不改变 `pursuit/**` 与 `tasks/tracker.py` 的任何行为（那些模块不读这个字段）。
#: 回写顺序：`R:` → 格名 → `*` → `:SCAN` → `:PIVOT` → `,x|,y`（朝向恒在最后）。
PIVOT_SUFFIX = ":PIVOT"
#: 终点朝向后缀
HEADING_SUFFIXES = frozenset({",X", ",Y"})
#: 兼容写法：`,y` 也可被写成 `:y`（旧文档同时出现过；两者语义一致）
_HEADING_ALT = (":Y", ":X")

_HEAD_BLOCK_RE = re.compile(r"^([A-N])([0-9]{1,3})")
_BARE_GAP_RE = re.compile(r"[,\s;]+")

#: ★ 后缀的**唯一合法顺序**匹配器（从左到右即回写顺序）：
#:   `*` → `:SCAN` → `:PIVOT` → `,x|,y`
#: 用一次正则匹配而不是"逐个 endswith 剥离"，为的是**只接受规范序**：
#: 乱序写法（如 `F2:SCAN*`）仍然报错，不会被"先剥谁"的顺序悄悄修正成合法写法。
_CANON_TAIL_RE = re.compile(
    r"^(?P<head>.*?)"
    r"(?P<shoot>\*|:SHOOT_STOP|:SHOOT)?"
    r"(?P<scan>:SCAN_STOP|:SCAN)?"
    r"(?P<pivot>:PIVOT)?"
    r"(?P<heading>,[XY]|:[XY])?$",
    re.IGNORECASE | re.DOTALL,
)


# --------------------------------------------------------------------------- #
#  1. 规范化
# --------------------------------------------------------------------------- #
def normalize_token(token: str) -> str:
    """大小写/空格规范化；保留 `R:` 前缀与后缀；`,x|,y` 后缀转小写。

    例：``'k9 '`` -> ``'K9'``；``'r:b5*'`` -> ``'R:B5*'``；``'N5,Y'`` -> ``'N5,y'``；
    ``'f8:pivot'`` -> ``'F8:PIVOT'``。

    后缀**唯一合法顺序**（``_CANON_TAIL_RE``）：`*`（或 `:SHOOT_STOP`/`:SHOOT` 归一成 `*`）
    → `:SCAN_STOP|:SCAN`（归一成 `:SCAN`）→ `:PIVOT` → `,x|,y`；乱序写法抛 `ValueError`。

    非法输入（非字符串、空串）抛 `ValueError`；**不在这里判越界**（那由 `parse_token` 负责）。
    """
    if not isinstance(token, str):
        raise ValueError(f"token 必须是字符串，收到 {type(token).__name__}: {token!r}")
    text = token.strip()
    if not text:
        raise ValueError("空 token")

    # ① `R:` 前缀（只认开头的，大小写不敏感；合法顺序：R: 在前、后缀在后）
    reverse = False
    if text[:2].upper() == REVERSE_PREFIX:
        reverse = True
        text = text[2:].strip()
        if text.upper().startswith(REVERSE_PREFIX):     # 'R:R:B5' —— 重复前缀按非法处理
            raise ValueError(f"重复的 {REVERSE_PREFIX} 前缀: {token!r}")
        if not text:
            raise ValueError(f"{REVERSE_PREFIX} 前缀后没有格名: {token!r}")

    # ② 一次匹配出全部后缀（严格规范序，见 `_CANON_TAIL_RE`）
    hit = _CANON_TAIL_RE.match(text)
    if hit is None:                                     # 正则允许 head 兜底，这里只可能不匹配空串
        raise ValueError(f"无法解析的 token: {token!r}")
    shoot = bool(hit.group("shoot"))
    scan = bool(hit.group("scan"))
    pivot = bool(hit.group("pivot"))
    heading_group = hit.group("heading")
    heading = heading_group[-1].lower() if heading_group else None
    text = hit.group("head")

    head = text.strip().replace(" ", "")
    if not head:
        raise ValueError(f"token 缺少格名: {token!r}")
    if not head[0].isalpha():
        # `*B5` / `,yN5` 这类"后缀跑到前面"的写法：若不拦会静默产出垃圾点，这里直接拦
        raise ValueError(f"格名必须以列字母开头: {token!r}")

    # ③ 组分隔符（`G:10`）折掉；剩下的非法残留（如 `K9:FOO`）必须报错，不许静默忽略
    if len(head) > 1 and head[1] == ":":
        head = head[0] + head[2:]
    if not head[0].isalpha() or not head[1:].isdigit():
        raise ValueError(f"无法解析的 token: {token!r}")

    out = f"{head.upper()}"
    if reverse:
        out = REVERSE_PREFIX + out
    if shoot:
        out += SHOOT_SUFFIX
    elif scan:
        out += SCAN_CANON            # `F2:SCAN_STOP` 归一成 `F2:SCAN`（语义不变，回写短一档）
    if pivot:
        out += PIVOT_SUFFIX
    if heading:
        out += "," + heading
    return out


def _normalize_shoot_directive(token: str) -> str:
    """`'shoot@g11 '` -> `'SHOOT@G11'`。"""
    text = token.strip()
    body = text[len(SHOOT_AT_PREFIX):].strip()
    if not body:
        raise ValueError(f"{SHOOT_AT_PREFIX} 后面缺少靶标格名: {token!r}")
    return SHOOT_AT_PREFIX + body.upper().replace(" ", "")


def _split_block_suffix(block: str, raw: str) -> tuple[str, str | None]:
    """'N5' + ',y' 拆成 ('N5', 'y')；无后缀返回 (block, None)。"""
    if len(block) >= 2 and block[-2] == ",":
        head = block[:-2]
        if not head:
            raise ValueError(f"朝向后缀前没有格名: {raw!r}")
        return head, block[-1].lower()
    return block, None


# --------------------------------------------------------------------------- #
#  2. 单 token
# --------------------------------------------------------------------------- #
def parse_token(token: str) -> dict:
    """单 token -> dict；**非法 token 抛 `ValueError`**（绝不静默返回空）。

    返回键（与 `docs/P0接口约定.md` §1 一致）::

        {'block', 'reverse', 'shoot', 'scan', 'heading', 'group', 'raw'}

    另有**附加键**（约定里的键一个不少，只是多给下游信息）：

    - ``'pivot'``     : bool，**折返点声明**（`:PIVOT` 后缀；该顶点处车头需转过 ≥135°）
    - ``'shoot_at'``  : bool，token 是 `SHOOT@<靶标>` 指令而非坐标
    - ``'target'``    : str | None，`SHOOT@` 指令的目标格名
    - ``'token'``     : str，规范化后的 token（`Waypoint.token` 用它，便于日志回溯）
    """
    if not isinstance(token, str):
        raise ValueError(f"token 必须是字符串，收到 {type(token).__name__}: {token!r}")
    raw = token
    stripped = token.strip()
    if not stripped:
        raise ValueError("空 token")

    if stripped.upper().startswith(SHOOT_AT_PREFIX):
        norm = _normalize_shoot_directive(stripped)
        target = norm[len(SHOOT_AT_PREFIX):]
        geo.block_to_index(target)                      # 靶标格名必须合法，否则抛 ValueError
        return {
            "block": target, "reverse": False, "shoot": True, "scan": False,
            "heading": None, "group": None, "raw": raw,
            "pivot": False, "shoot_at": True, "target": target, "token": norm,
        }

    norm = normalize_token(stripped)
    text = norm
    reverse = False
    if text.startswith(REVERSE_PREFIX):
        reverse = True
        text = text[len(REVERSE_PREFIX):]
    # 尾部后缀按**回写顺序的逆序**逐层剥（朝向 → :PIVOT → :SCAN → *）
    text, heading = _split_block_suffix(text, raw)
    pivot = text.upper().endswith(PIVOT_SUFFIX.upper())
    if pivot:
        text = text[: len(text) - len(PIVOT_SUFFIX)]
    scan = text.upper().endswith(SCAN_CANON.upper())    # `F2:SCAN` / `F2:SCAN_STOP` 归一后的形态
    if scan:
        text = text[: len(text) - len(SCAN_CANON)]
    shoot = text.endswith(SHOOT_SUFFIX)
    if shoot:
        text = text[:-1]
    block = text
    if not block:
        raise ValueError(f"缺少格名: {raw!r}")

    geo.block_to_index(block)                           # 越界/非法格名 -> ValueError
    return {
        "block": block, "reverse": reverse, "shoot": shoot, "scan": scan,
        "heading": heading, "group": None, "raw": raw,
        "pivot": pivot, "shoot_at": False, "target": None, "token": norm,
    }


# --------------------------------------------------------------------------- #
#  3. 路线
# --------------------------------------------------------------------------- #
def _looks_like_file(spec: str) -> bool:
    if "\n" in spec or "\r" in spec:
        return False
    if spec.lower().endswith((".txt", ".route", ".path")):
        return True
    if os.sep in spec or (os.altsep and os.altsep in spec):
        return True
    return os.path.isfile(spec)


def _read_token_file(path: str) -> list[str]:
    """读路线文件：**整行一个 token**；`#` 注释行与空行跳过（也支持行内 `#` 注释）。"""
    if not os.path.isfile(path):
        raise ValueError(f"路线文件不存在: {path}")
    out: list[str] = []
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            body = line.split("#", 1)[0].strip()
            if not body:
                continue
            for piece in _BARE_GAP_RE.split(body):
                if piece:
                    out.append(piece)
    if not out:
        raise ValueError(f"路线文件里没有任何 token: {path}")
    return out


def _as_token_list(tokens: Any) -> list[str]:
    if isinstance(tokens, str):
        spec = tokens.strip()
        if not spec:
            return []
        if _looks_like_file(spec):
            return _read_token_file(spec)
        return [p for p in _BARE_GAP_RE.split(spec) if p]
    if isinstance(tokens, (list, tuple)):
        return list(tokens)
    if isinstance(tokens, Iterable):
        return list(tokens)
    raise ValueError(f"token 集合类型不支持: {type(tokens).__name__}")


def _call_shoot_planner(planner: Callable[[str], Any], target: str) -> list[str]:
    """调 shoot_planner，返回值一律规整成 `list[str]`。"""
    chain = planner(target)
    if isinstance(chain, str):
        chain = _BARE_GAP_RE.split(chain)
    if chain is None:
        return []
    out: list[str] = []
    for item in chain:
        if item is None:
            continue
        text = str(item).strip()
        if text:
            out.append(text)
    return out


def parse_mission(tokens, *, shoot_planner: Callable[[str], Any] | None = None,
                  name: str = "mission") -> Route:
    """token 列表 -> `Route`。

    :param tokens: token 序列；也接受**路线文件路径**或空格/逗号分隔的字符串。
    :param shoot_planner: `SHOOT@<靶标>` 的候选点计算器，返回候选格名序列。
        缺省 = 靶标自身一格（shoot_planner 不可用时的降级行为）。
    :param name: 路线名（日志/回溯用）。

    非法 token、越界格名、**中间点带朝向后缀**一律抛 `ValueError`。
    """
    planner = shoot_planner
    if planner is None:
        planner = lambda target: [target]                  # noqa: E731  靶标自身一格
    raw_tokens = _as_token_list(tokens)

    builder = RouteBuilder(name)
    insert_at: int | None = None                           # 当前 SHOOT@ 链的起始下标
    group: str | None = None
    final_yaw: float | None = None
    have_final_heading = False

    for pos, raw in enumerate(raw_tokens):
        info = parse_token(raw)
        if info["shoot_at"]:
            # ---- 指令：靶标 -> 候选链，**整链插入**，共享同一 group ----
            target = info["target"]
            chain = _call_shoot_planner(planner, target)
            if not chain:
                raise ValueError(f"shoot_planner({target!r}) 没有返回任何候选点")
            group = target
            if insert_at is None:
                insert_at = len(builder)
            for cand in chain:
                cand_info = parse_token(cand)
                if cand_info["shoot_at"]:
                    raise ValueError(f"shoot_planner({target!r}) 的候选点里又出现 SHOOT@: {cand!r}")
                x, y = geo.block_to_meters(cand_info["block"])
                builder.add(Waypoint(
                    block=cand_info["block"], x=x, y=y, yaw=geo.HEADING_X,
                    reverse=cand_info["reverse"], shoot=True, scan=cand_info["scan"],
                    group=group, token=cand_info["token"], pivot=cand_info["pivot"],
                ))
            continue

        # 普通坐标点 —— 分组只覆盖**紧邻**的候选链，遇到任何普通点即封口
        group = None
        insert_at = None

        block = info["block"]
        heading = info["heading"]
        if heading is not None:
            if pos != len(raw_tokens) - 1:
                raise ValueError(
                    f"朝向后缀只允许出现在最后一个 token（第 {pos + 1}/{len(raw_tokens)} 个: {raw!r}）")
            final_yaw = geo.heading_token_to_yaw(heading)
            have_final_heading = True

        x, y = geo.block_to_meters(block)
        builder.add(Waypoint(
            block=block, x=x, y=y, yaw=geo.HEADING_X,
            reverse=info["reverse"], shoot=info["shoot"], scan=info["scan"],
            group=group, token=info["token"], pivot=info["pivot"],
        ))

    # ★ `drop_consecutive_duplicates=False`：并点必须走本模块的 `_dedup_waypoints`。
    #   `RouteBuilder.build` 的内置去重是"二选一"（保留信息更丰富的那个），
    #   会把 `B5*` + `B5:SCAN_STOP` 合并成只剩 shoot 或只剩 scan —— 违反"标记取并集"的约定。
    route = builder.build(drop_consecutive_duplicates=False,
                          fix_final_yaw=final_yaw if have_final_heading else None)
    route = _dedup_waypoints(route)
    crossing = detect_crossing(route)
    if crossing:
        route = Route(route.waypoints, name=route.name, crossing=crossing)
    return route


def _dedup_waypoints(route: Route) -> Route:
    """相邻同格 **且 reverse 相同** → 合并（shoot/scan/pivot 取并集）；reverse 不同绝不合并。

    以 `F11 / E11 / R:F11 / F10` 为例：`R:F11` 与它前面的 `E11` 不同格（不合并），
    与它后面的 `F10` 也不同格；而同格的 `R:F11` 出现在 `E11` 之后，段 `E11 -> F11` 才是倒车段。
    `B5 R:B5` 若合并，倒车段就丢了（所以绝不合并）。
    """
    merged: list[Waypoint] = []
    for wp in route.waypoints:
        if merged and merged[-1].block == wp.block and merged[-1].reverse == wp.reverse:
            prev = merged[-1]
            shoot = prev.shoot or wp.shoot
            scan = prev.scan or wp.scan
            token = (wp.token or wp.block) if (wp.shoot or wp.scan) else (prev.token or prev.block)
            merged[-1] = Waypoint(
                block=prev.block, x=prev.x, y=prev.y, yaw=prev.yaw,
                reverse=prev.reverse, shoot=shoot, scan=scan,
                speed_scale=prev.speed_scale, dwell_s=prev.dwell_s,
                group=prev.group or wp.group, branch=prev.branch or wp.branch,
                token=token, pivot=prev.pivot or wp.pivot,
            )
            continue
        merged.append(wp)
    return Route(tuple(merged), name=route.name, crossing=dict(route.crossing))


def detect_crossing(route: Route) -> dict:
    """从路线自身推断穿洞格：`{"lower": "K9", "upper": "K4"}`。

    判据：路线里落在 K 列（`geometry` 的列索引 10）的格，按行号分半场
    （下半场 row 7~11 / 上半场 row 1~6，见 `config/robot.yaml` 的 `bridge_order`）。
    这样 `validate.py` 的 R008/R009 不需要另外喂配置就能定位穿洞点。
    """
    k_col = geo.COL_LETTERS.index("K")
    lower: str | None = None
    upper: str | None = None
    for wp in route.waypoints:
        try:
            row_idx, col_idx = geo.block_to_index(wp.block)
        except ValueError:
            continue
        if col_idx != k_col:
            continue
        if row_idx >= 6:                                   # row 7~11
            lower = lower or wp.block
        else:                                              # row 1~6
            upper = upper or wp.block
    out: dict[str, str] = {}
    if lower:
        out["lower"] = lower
    if upper:
        out["upper"] = upper
    return out


# --------------------------------------------------------------------------- #
#  4. 起步复位
# --------------------------------------------------------------------------- #
def parse_reset(spec: str) -> tuple[str, float]:
    """``'N10,y'`` -> ``('N10', π/2)``；``'N10'`` / ``'N10,x'`` -> ``('N10', 0.0)``。

    与 `reset:BLOCK,HEAD` 的语义一致：belief = 目标格心 + 目标朝向。
    非法格名抛 `ValueError`；`R:` / `*` / 扫描后缀出现在复位串里同样视为非法。
    """
    if not isinstance(spec, str) or not spec.strip():
        raise ValueError(f"复位串非法: {spec!r}")
    info = parse_token(spec)
    if info["shoot_at"]:
        raise ValueError(f"复位串不能是 SHOOT@ 指令: {spec!r}")
    if info["reverse"] or info["shoot"] or info["scan"] or info["pivot"]:
        raise ValueError(f"复位串不允许带 R:/*/扫描/:PIVOT 后缀: {spec!r}")
    heading = info["heading"] or "x"                       # 无后缀 = 朝 X+
    return info["block"], geo.heading_token_to_yaw(heading)


# --------------------------------------------------------------------------- #
#  5. 回写
# --------------------------------------------------------------------------- #
def format_route(route: Route) -> list[str]:
    """`Route` -> token 列表（回写 robot.yaml 用）。

    **必须能往返**：`parse_mission(format_route(r))` 的块名序列与 `r` 一致（除朝向规范化）。
    后缀顺序恒定：`R:` 前缀 → 格名 → `*` → `:SCAN` → `:PIVOT` → `,x|,y`。
    """
    if route is None:
        raise ValueError("format_route: route 为 None")
    out: list[str] = []
    last = len(route.waypoints) - 1
    for idx, wp in enumerate(route.waypoints):
        block = wp.block
        if not block:
            raise ValueError(f"第 {idx} 个航点没有格名，无法回写")
        token = REVERSE_PREFIX + block if wp.reverse else block
        if wp.shoot:
            token += SHOOT_SUFFIX
        if wp.scan:
            token += SCAN_CANON
        if wp.pivot:
            token += PIVOT_SUFFIX
        if idx == last and abs(wp.yaw - geo.HEADING_Y) < 1e-6:
            token += ",y"
        out.append(token)
    return out
