# -*- coding: utf-8 -*-
"""路线规划器 —— **换一张 `map.yaml` 就能自动生成路线**，不再手写 token。

用户诉求原话："我只要把地图点位（map.yaml）替换掉就行了，小车会自己算路径然后跑起来。"
本模块就是那个"自己算"的部分；它只读 `map.yaml` 的六个字段
（`start/end/mines/obstacles/targets/bridges`）+ `Config` 里的几个约束值，
**不复制任何一份地图要素**（真源唯一性，见 `validate.py` 记录的 K13 教训）。

--------------------------------------------------------------------------------
赛道模型（`config/map.yaml` 顶部注释 + `robot.yaml` 的 `route` / `bridge_order`）
--------------------------------------------------------------------------------
整个 K 列是一座桥、每格都是一个桥洞；第 6/7 行之间有一条横跨 D~N 列的隔板，
只有 A/B/C 列能上下穿行。于是合法路线只能是一条 **U 形**：

    start（下半场，桥东侧）→ 穿下半场桥洞（**向西**）→ 压雷/避障/打靶 →
    绕隔板北上（A/B/C 列）→ 穿上半场桥洞（**向东**）→ end（上半场，桥东侧）

本模块把这个 U 形结构**显式建模**，不指望 A* 自己摸出来：

1. 按行号把必访点分到上/下半场。`map.bridges` 只是"本局识图结果"，
   真正的候选集合是 `Config` 的 `route.cross_lower/upper_candidates`
   （每局随机、由无人机情报选格），缺省才回落到 `map.bridges`，再缺省才用整列。
2. 顺序固定为 `start → 下半场节点 → K(lower) → 上半场节点 → K(upper) → end`，
   结构上保证 `validate.py` 的 R007（每半场**恰好**穿一次、方向正确）不可能被破坏
   —— 不靠"搜出来碰巧对"。
3. 半场内顺序用"最近邻 + 2-opt"，代价来自**带朝向的 A\\***；而且**朝向本身也参与优化**
   （见下节）—— 这条是必需的，否则会出现"到达桥洞时车头朝东、下一步却要往西"的
   180° 折返。

--------------------------------------------------------------------------------
为什么顺序优化里必须同时优化"朝向"
--------------------------------------------------------------------------------
阿克曼车不能横移，所以"从哪个方向进某个格"直接决定下一段能不能顺畅出发。
第一版只按"格到格的最短代价"排序，结果生成出 `I10 -> I9 -> K9 -> J9`：车从东边
开进 K9 后**原地 180° 折返**再往东走 —— 一条能过校验、但现场很蠢的路线。
现在把朝向作为状态的一部分：

* `cost[(a, b)]` = 从 a 的**四个朝向**出发、到达 b 的**四个朝向**的最小代价（顺序轮用）；
* `h_cost[(a, b)][ha][hb]` = 带朝向的精确代价（朝向轮用）。

两者交替优化（顺序 → 朝向 → 顺序 …，最多 3 轮），起点朝向 = **设计常量 `START_HEADING`**
（车头 +y，见常量区「起始航向」说明），与 `mission.reset` 无关。
2-opt 反转子段时，两端的朝向选择会自动重新评估（成本函数里每次都取当前朝向）。

--------------------------------------------------------------------------------
三个必须显式处理的坑（都有现场依据）
--------------------------------------------------------------------------------
① **K 列只在桥洞行可穿越**。`validate.py` 的 R007 要求"每半场**恰好**一次穿越 K 列，
   所以规划期就把"在非桥洞行进出 K 列"直接拉黑。否则 A* 会为了抄近路从
   `I10 -> K10 -> J10` 蹭一下 K 列 —— 那会多算一次穿越，行为上也是撞侧壁。
   同时拉黑"K 列内上下挪格"（R009：桥洞只有单格宽）与"K 邻列斜切"（R009 的 10° 上限）。

② **180° 折返必须改写成倒车段**（`R:` 前缀）。
   实车原地转向是"打死舵 + 慢慢前进"，走的是圆弧；180° 掉头会横向扫出约 0.54 m
   —— 比一格(0.35 m)还宽，在地图角落必然甩出边界。倒车零横向摆动，是阿克曼车处理
   死角的正确解法，也是现有手写路线里那 3 段 `R:` 的来历。
   编译阶段检测"前进→前进 且折返角 > 150°"并改写，随后 `assert_legal` **断言**
   结果里不再存在任何"同模式 180° 折返"；改写后车头朝向**重算**
   （倒车掉头与原地掉头的车头正好相反，弄错会让终点姿态与云台基准全错）。

③ **射击停站必须落在真能打到靶的位置**。
   距离判据（`0.20~0.45 m`）**复用现有函数** `validate._euc`
   （= `shoot_planner.dist_m` = `route_check` 那条规则的算法），不另写一套；
   候选点 = 各靶格的 4 个正交邻居（`targets` 是**相连两格** = 一处轨道靶区，
   所以"一个点同时服务两格"的位置天然在候选里 —— 手写路线的 `G10` 正是这种点）。

--------------------------------------------------------------------------------
与现有校验器的关系
--------------------------------------------------------------------------------
本模块**不复制、不放宽**任何规则：结构判据直接复用 `validate.py` 的
`_crossing_at_column` / `_barrier_crossing` / `_euc` / `_stop_ok`，
朝向与代价语义对齐 `tools/global_planner.py`。生成结果在
`tools/plan_route.py --check` 里再跑一遍 `tools/route_check.py`（0 error 才算通过）。
两套校验器口径若冲突，本模块**如实抛错并贴出规则原话**，绝不擅自放宽。
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

from ..core import geometry as geo
from ..core.config import Config
from . import physics as _phys
from . import validate as V
from .model import Route, RouteBuilder, Waypoint
from .parser import (PIVOT_SUFFIX as _PIVOT_SUFFIX, SCAN_CANON as _SCAN_CANON,
                     SHOOT_SUFFIX as _SHOOT_SUFFIX, parse_token)

__all__ = [
    "PlannerError",
    "PlanResult",
    "MapSpec",
    "Grid",
    "plan_route",
    "load_map",
    "map_section",
    "assert_legal",
    "assert_legal_no_reverse",
    "check_route_structure",
    "shoot_candidates",
    # 双射击停站（对侧互备）：候选判据 + 引擎无关的 token 落标（t7，自动规划收口）
    "second_shoot_stop",
    "ensure_second_shoot_stop",
    "shoot_stop_distance",
    "plate_side_of",
    "stop_distance_m",
    "turn_angle_deg",
    "route_tokens",
    "pivot_vertices",
    "mark_pivots",
    "DIRS",
    "DEFAULT_MAX_SECONDS",
    # 起始航向 = 设计常量（+y），两引擎共用；见常量区的说明
    "START_HEADING",
    "START_HEADING_YAW",
    "PIVOT_MARK_DEG",
    "PIVOT_PENALTY",
    "MINE_PRESS_TOL_M",
    "MINE_TURN_PENALTY",
    "BARRIER_TURN_PENALTY",
    "straighten_mines",
    # tracker-safe 引擎（自动规划追平现役图路线；见文件末尾那一节）
    "plan_route_tracker_safe",
    "TS_TURN_W",
    "TS_MINE_TURN_PENALTY_M",
    "TS_PHASE_CELL",
    "TS_TURNED_MINES",
    "TS_MAX_TURN_DEG",
    "TS_MINE_STRAIGHT_DEG",
    "TS_MAX_SECONDS",
    # 顺序 DP 的到达状态上限（Pareto 截断，现役图恒不触发）
    "TS_PARETO_KEEP",
    # 桥洞进洞侧冗余墙（规划器侧判据 + 兜底长度）
    "BRIDGE_ENTRY_REDUNDANCY_M",
    "BRIDGE_ENTRY_CLEAR_EPS",
    "BRIDGE_ENTRY_SAMPLES",
    # 进洞前对正惩罚（洞口前 1 格 90° 拐弯 ⇒ 罚，驱动拐弯顶点前移）
    "BRIDGE_APPROACH_ALIGN_PENALTY_M",
    # 停站点视线判据（隔板背后的格不算候选）
    "barrier_sight_blocked",
    "SHOOT_DISTANCE_M_DEFAULT",
    "SHOOT_DISTANCE_TOL_M_DEFAULT",
]

# --------------------------------------------------------------------------- #
#  常量
# --------------------------------------------------------------------------- #
#: 朝向索引：0=北(行号减小) 1=西(列号减小) 2=南(行号增大) 3=东(列号增大)。
#: 与 `tools/global_planner.py` 的 `DIRS` 逐字一致 —— 现场语义已经这么用；
#: 换算成 `geometry` 的地图 yaw：0/2 → `HEADING_X`、1/3 → `HEADING_Y`。
DIRS: tuple[tuple[int, int], ...] = ((-1, 0), (0, -1), (1, 0), (0, 1))
TURN_LEFT, TURN_RIGHT = -1, +1
HEAD_NAMES = ("北(行号减小)", "西(列号减小)", "南(行号增大)", "东(列号增大)")

#: **起始航向 = 设计常量**（用户定案；不再由 `mission.reset` 反推）。
#:
#: 现场约定：每次发车，车都摆在起点格（现场图 = `N10`）、**车头水平朝图左** ——
#: 即朝 K 列桥洞方向、列号减小方向，也就是地图 `+y`（`geometry.HEADING_Y`）
#: = `HEAD_NAMES[1]`「西」。**摆位由现场用户保证**，判据里不需要、也不允许有
#: "摆错就拒绝发车"这类校验（用户定案）。
#:
#: 规划器把这个常量当**设计输入**：**不为起点生成任何转向消解** ——
#: ① 不按 `mission.reset` 反推朝向（旧实现把 `reset_yaw` 当起点朝向真源）；
#: ② 不再在"起点朝西不可行"时把起点朝向改成朝 X+（那是一条静默的朝向改写）。
#: 现场图 `N10 → M10` 本来就是 +y ⇒ 输出与历史**逐字节一致**（零影响契约）。
START_HEADING: int = 1                     # DIRS 索引：1 = 西 = 列号减小 = +y
START_HEADING_YAW: float = geo.HEADING_Y   # 同一朝向的地图 yaw（+π/2）

#: 动作编号（`flood` 回传，用于判断某一段是否倒车）
ACT_START, ACT_FORWARD, ACT_BACKWARD, ACT_LEFT, ACT_RIGHT = 9, 0, 1, 2, 3

#: 阿克曼动作代价：倒车比前进贵（更慢、更难控制），转向有固定代价。
COST_FORWARD = 1.0
COST_BACKWARD = 2.2
COST_TURN = 1.4
#: ⚠️ 与 tools/global_planner.py:88-89 的 COST_TURN/COST_BACKWARD（1.6/1.9）存在数值分叉，
#: 属历史口径。n1 对照实验（_tmp_verify/n1_const/cost_compare.json，2026-10-17）：
#: 四组取值下可下发路线串完全相同，差异只体现在 A* 自报代价。统一前须重跑该实验并复核
#: 现役图（map.yaml/44 token）路线，不得单边改数值。

#: 障碍安全势场：切比雪夫距离 1 / 2 两档加罚，逼路线离水桶远一点。
#: 罚值刻意"软"而不是拉黑 —— 现场 A6/C4 这类雷点本身就贴着障碍，
#: 硬拉黑会直接无解；软罚只影响"绕不绕得开"的取舍。
OBSTACLE_DANGER_RADIUS = 2
OBSTACLE_DANGER_W1 = 0.9
OBSTACLE_DANGER_W2 = 0.4
#: 隔板相邻行的额外罚（缺口只有 A/B/C 三列，别贴着隔板擦）
BARRIER_NEAR_COST = 0.15

#: 编译阶段的倒车改写阈值（度）：>150° 才算"折返"，正常 90° 拐弯绝不动它。
UTURN_REVERSE_DEG = 150.0

#: ★ 折返点（`:PIVOT`）的判定阈值（度）。取值 = `pursuit.pivot_angle_deg`
#: （`core/config.py` 默认 135°，真车已实测完成），判据与 `pursuit.path.corner_turn_deg`
#: 一致（顶点处**车头**需转过的角度）：相邻两段有倒车时车头一度都不用转，不算折返。
#: 现场无倒车路线实测只有两处：`F8` / `D6`（都是"开进死格再原路退出"的压雷死角）。
PIVOT_MARK_DEG = 135.0
#: ★ TSP/顺序搜索里的**折返惩罚**（等效代价单位）：
#: 访问顺序里一处"进去再原路退出来"的顶点按 5.0 计（= 5 格前进代价）。
#: 为什么需要：无倒车模式下这种访问必然让车在顶点原地掉头（PIVOT 实测 ~10.8 s、
#: 横向扫出 ~0.54 m > 一格 0.35 m），比多绕两格贵得多 ——
#: 只按 A* 里程排序会把它当成"顺路"，于是折返点变成路线里最贵的动作却没被计价。
PIVOT_PENALTY = 5.0

#: ★★ 雷心必须被**车心**压过（用户 2026-10-11 口径）：参考折线到每个雷格中心的
#: 最近距离 ≤ 该容差（米）。实现见 `straighten_mines()` —— 把雷格做成"进—雷—出"
#: 三点直线的**中点**，纯追踪于是沿直线穿过雷心；不这么做的后果有实测依据：
#: ① 90° 拐角处的雷被切角"擦过"（取证报告 §3.5：C5 差 3.1 cm、I8 差 2.7 cm）；
#: ② 180° 折返顶点上的雷更糟：`pursuit.pivot_dist_m=0.25` 让车在距顶点 0.25 m 处
#:    就掉头，车心**根本到不了**雷心（A9 实测差 12.7 cm）。
MINE_PRESS_TOL_M = 0.03

#: ★ 顺序层"雷格不得作为拐点"的惩罚权重（等效代价单位）：90° 拐点记 1 份、180° 记 8 份。
#: 1.0 份 ≈ 1 格前进代价 —— 权衡"多绕一格"与"雷被切角擦过"，后者要严重得多
#: （真车漏压雷 = 直接丢分），所以取 2.0 让排序优先满足压雷。
MINE_TURN_PENALTY = 2.0

#: ★★ 隔板相邻行"死胡同式 90° 转向"的**推荐**惩罚权重（等效代价单位/处）。
#: 取值依据（`_tmp_session/t6_planner/`）：现场 54 点路线在这种顶点上有 2 处
#: （`N7` / `H7`），每处实测触线 175~178 拍；一处 90° 转向的绕路替代（多走 1~2 格）
#: 的里程代价 ≈ 1.0~2.0 等效单位，所以取 **8.0**（4~8 倍里程代价）足以让搜索把
#: 访问顺序与段路径一起改掉，而不是"便宜就认了"。
#: ★ 默认值（`plan_route` 的键缺失回落到 **0.0**）刻意是 **0.0**：关闭 = 与历史
#: `route.tokens` / 四处逐 token 恒等式逐字一致。启用由 `planner.barrier_turn_penalty`
#: 决定（本模块不替现场做启用决定）。
BARRIER_TURN_PENALTY = 8.0

#: **过门转弯点**离隔板东端的**几何**要求（几何推导，不是固定阈值）。
#: 用户 2026-10-11 口径：`所需余量 ≥ R·sinθ + w/2·cosθ`，其中
#:   · `R = wheelbase / tan(max_steer)` —— **同源取 `pursuit.wheelbase_m` / `pursuit.max_steer_deg`**，
#:     绝不在本模块写常量（现场 `0.157/tan30° = 0.27193 m`）；
#:   · `w/2` = 车半宽（`vehicle.width_m / 2`，现场 0.0675 m）；
#:   · `θ` = 过门那次转弯的角度（现场 90° ⇒ 所需 0.2719 m）。
#: ★ 与固定阈值方案的差别：固定 **0.5 m** ⇒ 只有 A(1.050)/B(0.700) 合规、C(0.350) 判红；
#:   几何化后 90° 只需 **0.2719 m** ⇒ **C 列其实合规**（0.350 > 0.2719）。
#:   用户要求"按几何推导取代固定阈值"正是为此。默认权重 0.0 ⇒ 零影响。
DOOR_TURN_PENALTY = 6.0

#: ★ 约束②：最外行/列的雷，**进入该行/列的那次转弯**必须离雷 ≥ 该格数。
#: 理由（§4）：转弯点贴着外行/列的雷时，纯追踪以 0.18~0.21 m 横向偏移骑到边界外侧
#: （hand_v1 出场地车心 3.2 cm 就是这么来的）；"先长直再横穿"的走法偏移只有 0.065 m。
EDGE_MINE_TURN_MIN_CELLS = 2
#: 约束② 的软代价权重（等效代价单位/处）。**0.0（默认）= 关闭**。
EDGE_MINE_TURN_PENALTY = 6.0

#: 单次规划的时间预算（秒）。超了抛 `PlannerError`，绝不静默给半成品路线。
#: ★★ 预算修正（门禁负载下曾出现 flake）：0.8 → **2.0 s**。
#:   实测（本机，现役图 config/map.yaml）`plan_route(allow_reverse=False, pivot_penalty=5.0)`
#:   单次耗时 **0.55 s**（8/8 次跑在 548~569 ms），离旧默认 0.8 s 只有 0.25 s 余量；
#:   在 4 忙核下 `test_planner.py:647` 那次调用稳定超时（单跑 5/5 绿 ⇒ 纯负载敏感 flake）。
#:   取 **2.0 s** 的理由：① 与现场真源 `config/robot.yaml` 的 `planner.max_seconds: 2.0` 一致
#:   （车上本来就是这个预算，常量只是"没有配置时的兜底"）；② 余量 3.6× 足够吸收负载抖动。
#:   ⚠️ 超时仍是**硬错误**（不做静默降级），只是不再在正常负载下误报。
DEFAULT_MAX_SECONDS = 2.0
#: "顺序 ↔ 朝向"交替优化的轮数上限
MAX_TOUR_ROUNDS = 3
#: 2-opt 的收敛上限（防病态输入吃光时间预算）
MAX_2OPT_PASSES = 6
#: 相邻航点最大间距（格）。超过就插中间点：`validate.py` 的 R011 在 2.5 格处告警，
#: 而且 1.4 m 的长直线段在纯追踪下会切角，插点同时解决这两件事。
MAX_HOP_BLOCKS = 2
#: ★ 出车前校验器 `tools/route_check.py` 的**实际接受窗口**（该文件第 579 / 615-618 行）：
#:     ``SHOOT_DIST_MIN - 0.03 <= d <= SHOOT_DIST_MAX + 0.06``
#: 即 0.17 ~ 0.51 m。一个停站格能同时服务两个靶格，正是靠这个容差
#: （例如到两靶格分别 0.35 m / 0.495 m，后者超过 0.45 m 的标称上限）。
#: 本规划器把它作为**唯一真相的接受判据**（显式写出，不是偷偷放宽）：规划器自己不发明
#: 阈值，接受窗口由"最终会被执行的那条校验规则"决定。
ROUTE_CHECK_TOL_LO = 0.03
ROUTE_CHECK_TOL_HI = 0.06

_CROSS_COL = geo.COL_LETTERS.index(V.CROSS_COLUMN)

#: ★ 目标函数口径（TRACK-PLAN-B1-20261021）。**默认 legacy = 历史行为逐字节不变**。
#:   · legacy：最短巡回 + 硬约束，结构不满足抛 PlannerError（旧口径）；
#:   · tasks ：任务优先 —— 先最大化成功任务数（8 雷 + 可达射击停站），再最小化时间；
#:     约束不满足不再判"无解"，而是显式 degraded + 可执行兜底路线。
OBJECTIVE_LEGACY = "legacy"
OBJECTIVE_TASKS = "tasks"
OBJECTIVES = (OBJECTIVE_LEGACY, OBJECTIVE_TASKS)


class PlannerError(RuntimeError):
    """规划失败（无解 / 与校验器冲突 / 超出时间预算）。**绝不静默降级。**"""


# --------------------------------------------------------------------------- #
#  地图
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class MapSpec:
    """`map.yaml` 的 `map:` 段（只取规划需要的字段，**不复制**要素真源）。"""

    name: str
    start: str
    end: str
    mines: tuple[str, ...]
    obstacles: tuple[str, ...]
    targets: tuple[str, ...]
    bridges: tuple[str, ...]
    barrier_col_from: str = geo.BARRIER_COL_FROM
    barrier_row_hi: int = geo.BARRIER_ROW_HI
    barrier_row_lo: int = geo.BARRIER_ROW_LO
    source: str = ""


def map_section(map_cfg: Any) -> dict:
    """取 `map:` 段 —— 直接复用 `validate._map_section`（口径必须与校验器一致）。"""
    return V._map_section(map_cfg)                     # noqa: SLF001  同包既有实现


def _as_cells(value: Any) -> tuple[str, ...]:
    """列表归一化（大写、去空、保序）—— 复用 `validate._cells` 的规则。"""
    return tuple(V._cells({"value": value}, "value"))  # noqa: SLF001


def _checked(cells: Iterable[str], group: str) -> tuple[str, ...]:
    for cell in cells:
        try:
            geo.block_to_index(cell)
        except ValueError as exc:
            raise PlannerError(f"地图 {group} 里的格名非法: {exc}") from exc
    return tuple(cells)


def load_map(map_cfg: Any, *, source: str = "") -> MapSpec:
    """`map.yaml` 原始 dict（或已展平 dict / `Config`）-> `MapSpec`。

    缺失/非法一律抛 `PlannerError`，**绝不猜一个默认起点** —— 起点错了整条路线都错位。
    """
    merged = map_section(map_cfg)
    if not merged:
        raise PlannerError("地图里没有 map: 段（或为空），无法规划")

    start = str(merged.get("start") or "").strip().upper()
    end = str(merged.get("end") or "").strip().upper()
    if not start or not end:
        raise PlannerError("地图缺少 map.start / map.end")
    _checked((start, end), "start/end")

    barrier_col = str(merged.get("barrier_col_from") or geo.BARRIER_COL_FROM).strip().upper()
    if barrier_col not in geo.COL_LETTERS:
        raise PlannerError(f"map.barrier_col_from 非法: {barrier_col!r}")

    def _int(key: str, fallback: int) -> int:
        try:
            return int(merged.get(key, fallback))
        except (TypeError, ValueError):
            return fallback

    return MapSpec(
        name=str(merged.get("name") or ""), start=start, end=end,
        mines=_checked(_as_cells(merged.get("mines")), "mines"),
        obstacles=_checked(_as_cells(merged.get("obstacles")), "obstacles"),
        targets=_checked(_as_cells(merged.get("targets")), "targets"),
        bridges=_checked(_as_cells(merged.get("bridges")), "bridges"),
        barrier_col_from=barrier_col,
        barrier_row_hi=_int("barrier_row_hi", geo.BARRIER_ROW_HI),
        barrier_row_lo=_int("barrier_row_lo", geo.BARRIER_ROW_LO),
        source=source,
    )


# --------------------------------------------------------------------------- #
#  栅格与代价
# --------------------------------------------------------------------------- #
class Grid:
    """通行栅格：障碍占格 + 隔板跨越约束 + K 列桥洞约束 + 障碍安全势场。

    坐标一律用 ``(row, col)``：``row`` 是 1-based 行号，``col`` 是 0-based 列索引
    （与 `geometry.block_to_index` 的返回值口径对齐）。
    """

    def __init__(self, spec: MapSpec, bridge_rows: Iterable[int] = (),
                 *, allow_reverse: bool = True,
                 barrier_turn_margin_m: float = 0.0) -> None:
        self.spec = spec
        #: 无倒车模式（真车底盘不支持倒车）：`_moves` 不再产出倒车动作，
        #: 180° 折返 = 正向开进死格、原地转向（2×COST_TURN）后正向开出 ——
        #: 顶点处由 TrackerTask 自动 PIVOT。默认 True = 旧行为（允许倒车）。
        self.allow_reverse = bool(allow_reverse)
        #: ★★ 隔板相邻行"行内 90° 转向"的**最小安全裕度**（米）：
        #: 行心到隔板只有半格（0.175 m），这个值就是"行内必须留出的摆动空间"。
        #: `0.0`（默认）= 判据按纯几何生效。
        #: 放在 `Grid` 上而不是多开一个函数参数：判据 `_barrier_turn_cell()` 与
        #: 代价函数 `_moves()` 都要用它，挂在地图对象上最不容易漏传。
        #: ★★ 负值不再"关闭该判据" —— 旧文档的"负值 = 只报不拦"
        #:   是一个放宽开关，与"硬约束不得放宽"冲突，已在构造口钳位为 0.0。
        #:   想关判据请改判据本身（并给出物理依据），不允许留暗门。
        self.barrier_turn_margin_m = max(0.0, float(barrier_turn_margin_m))
        self.blocked: set[tuple[int, int]] = set()
        for cell in spec.obstacles:
            row_idx, col_idx = geo.block_to_index(cell)
            self.blocked.add((row_idx + 1, col_idx))
        # ---------- ★★ 跨格靶板 = 占两格的**实体**，同样不可压过 ----------
        # 用户 2026-10-11 裁定：「C1+D1 是一块跨格靶板，**所有靶子都是一个占用两格的跨格靶板**」。
        # 为什么必须"拉黑"而不是"加代价"：靶板是**物理实体**，压过去就是撞板；加代价只能表达
        # "尽量别压"。现场实测（`_tmp_session/n8_newmap/报告_跨格靶板约束.md`）现路线在新图确实
        # 压过靶板（@0.15 C1 61 拍 / D1 93 拍，@0.30 各 45 拍）—— 这是**非法**，不是"次优"。
        #
        # 为什么"拉黑"与"射击停站"不冲突（两个不同机制，互不削弱）：
        #   · **通行**判据问"车能不能开进这一格" → 靶板是实体，答案永远是不能；
        #   · **停站**判据问"站在这儿能不能打到靶板" → 停站是靶格的**正交邻居**，
        #     本来就不是靶格自身（`shoot_candidates()` 取的正是 `DIRS` 邻居；
        #     现场图 `D3`/`G3` 分居 `E5`/`F5` 靶板两侧就是这种点）。
        #   ⇒ 拉黑靶格**自动**把靶格剔出射击候选（`shoot_candidates` 用 `grid.passable()` 筛邻居），
        #     而"邻近停站仍能打到靶板"这条完全不受影响（见 `shoot_candidates` 的注释）。
        #
        # 向后兼容：现图（`config/map.yaml`，靶 `E5`/`F5`）的 40 token 路线本来就不进靶格，
        # 实测"拉黑 vs 不拉黑"输出**逐 token 相同**（`_tmp_session/t11_target_block/
        # probe_would_change.txt` §①）⇒ 对现图零影响。
        for cell in spec.targets:
            row_idx, col_idx = geo.block_to_index(cell)
            self.blocked.add((row_idx + 1, col_idx))
        #: ★ 靶板格（不可通行，与 `obstacles` 同待遇）；单独留一份供日志/诊断区分
        #: "这格为什么不能走"，也让 `_build_danger` 能在不改变障碍语义的前提下单独处理它。
        self.target_cells: set[tuple[int, int]] = {
            (geo.block_to_index(c)[0] + 1, geo.block_to_index(c)[1]) for c in spec.targets
        }
        #: ★★ 势场源 = **只含障碍**，不含靶板（见 `_build_danger` 的说明：
        #: 靶板要贴着打，加"离远点"势场是反着用力；本轮实测它会把射击停站从 G10
        #: 挤到 F10 并让现图路线越界 196 拍）。`blocked` 仍含靶板（管通行）。
        self.hazard_cells: set[tuple[int, int]] = {
            (geo.block_to_index(c)[0] + 1, geo.block_to_index(c)[1]) for c in spec.obstacles
        }
        self.barrier_col = geo.COL_LETTERS.index(spec.barrier_col_from)
        self.barrier_hi = spec.barrier_row_hi
        self.barrier_lo = spec.barrier_row_lo
        self.bridge_rows = {int(r) for r in bridge_rows}
        self._danger = self._build_danger()

    # ---------------- 查询 ----------------
    def in_bounds(self, row: int, col: int) -> bool:
        return 1 <= row <= geo.MAP_ROWS and 0 <= col < geo.MAP_COLS

    def passable(self, row: int, col: int) -> bool:
        return self.in_bounds(row, col) and (row, col) not in self.blocked

    def barrier_cross_ok(self, row_from: int, row_to: int, col: int) -> bool:
        """跨越第 hi/lo 行之间的隔板：**只有列索引 < barrier_col_from 的列**能过。"""
        if {row_from, row_to} == {self.barrier_hi, self.barrier_lo}:
            return col < self.barrier_col
        return True

    def zone_of(self, row: int) -> int:
        """0 = 上半场（row <= barrier_row_hi），1 = 下半场。"""
        return 0 if row <= self.barrier_hi else 1

    def move_ok(self, row: int, col: int, nrow: int, ncol: int) -> bool:
        """一步是否合法。

        四条判据（每条都对应一条现场硬约束）：
         ① 目标格可通行（障碍不可通行）；
         ② 不非法跨越第 hi/lo 行隔板（只有 A/B/C 列能上下穿行）；
         ③ K 列内不上下挪格（R009：桥洞单格宽，必撞侧壁）；
         ④ **只在桥洞行进出 K 列**（R007：每半场恰好一次穿越；非桥洞行进出会多算穿越，
            行为上也是撞桥墩）。K 邻列的斜切同时被本条拦掉（R009 的 10° 上限）。

        ★★ 前置断言：**每一步必须是正交相邻的一格**，
        即 `abs(drow) + abs(dcol) == 1`。为什么加：现在"恒正交相邻"是靠调用链
        （`_moves()` 只按 `DIRS` 产出单位步）**隐式**保证的；一旦将来有人在这里塞
        "跳格移动"（例如为了走对角线而直接跳到隔一格），下面的 ③/④ 判据全部只在
        "目标格"上成立，会**静默**放过中间格上的桥墩/隔板 ⇒ 这里把不变量显式钉住，
        跳格会当场炸而不是悄悄产出非法路线。
        """
        drow, dcol = nrow - row, ncol - col
        if abs(drow) + abs(dcol) != 1:
            raise PlannerError(
                f"内部错误：move_ok 只接受正交相邻的一步，收到 "
                f"({row},{col}) -> ({nrow},{ncol})（位移 ({drow},{dcol})）"
                f"—— 跳格移动会让后面的桥洞/隔板判据失效")
        if not self.passable(nrow, ncol):
            return False
        if not self.barrier_cross_ok(row, nrow, ncol):
            return False
        drow, dcol = nrow - row, ncol - col
        if col == _CROSS_COL and ncol == _CROSS_COL and drow != 0:
            return False
        crosses_cross_col = (col - _CROSS_COL) * (ncol - _CROSS_COL) <= 0 and col != ncol
        if crosses_cross_col:
            row_at_cross = nrow if ncol == _CROSS_COL else row
            if row_at_cross not in self.bridge_rows:
                return False
        return True

    def danger(self, row: int, col: int) -> float:
        return self._danger[row][col]

    def _build_danger(self) -> list[list[float]]:
        """障碍安全势场。**只由 `hazard_cells`（= 障碍）生成，不由靶板生成。**

        ★★ 为什么必须分开：靶板与障碍虽然**都不可通行**，
        但它们的"势场语义"不同：
          · 障碍（水桶）：会挡视线、蹭上去风险高 → 附近加软代价让路线离远点；
          · 靶板：是**该去打的目标**，射击停站本来就必须紧贴它（现场图 `D3`/`G3`
            分居 `E5`/`F5` 靶板两侧）—— 给靶板加同样的"离远点"势场，等于**反着用力**。
        实测踩到的后果：把靶格并进 `blocked` 后，`F10` 的势场从 0.4 抬到 0.9
        （它同时被 `F11`+`G11` 两个"假障碍"覆盖），于是 `E9→F10`/`F10→E10` 的腿各贵
        0.5 —— 射击停站被挤离原位，且新路线仿真**越界**（`_tmp_session/t11_target_block/probe_delta.txt`）。
        ⇒ **可通行性与势场必须解耦**：`blocked` 管"能不能走"（含靶板），
          `hazard_cells` 管"想不想靠近"（只有障碍）。
        """
        grids = [[0.0] * geo.MAP_COLS for _ in range(geo.MAP_ROWS + 1)]
        for (orow, ocol) in self.hazard_cells:
            for drow in range(-OBSTACLE_DANGER_RADIUS, OBSTACLE_DANGER_RADIUS + 1):
                for dcol in range(-OBSTACLE_DANGER_RADIUS, OBSTACLE_DANGER_RADIUS + 1):
                    row, col = orow + drow, ocol + dcol
                    if not self.in_bounds(row, col):
                        continue
                    cheb = max(abs(drow), abs(dcol))
                    if cheb == 0:
                        continue
                    value = OBSTACLE_DANGER_W1 if cheb == 1 else OBSTACLE_DANGER_W2
                    if value > grids[row][col]:
                        grids[row][col] = value
        for row in (self.barrier_hi, self.barrier_lo):
            if 1 <= row <= geo.MAP_ROWS:
                for col in range(geo.MAP_COLS):
                    grids[row][col] += BARRIER_NEAR_COST
        return grids


# --------------------------------------------------------------------------- #
#  ★ 隔板相邻行的"死胡同式 90° 转向"
# --------------------------------------------------------------------------- #
#: ★★ 规划器的搜索状态：``(row, col, head)`` 在**开了隔板避让**时扩成
#: ``(row, col, head, prev_head, prev_head_valid)`` —— 多带"上一段的行进朝向"，
#: 用于识别"从隔板行南侧邻行北上进入隔板行、随后在隔板行内 90° 转向"这个三格形态。
#: 关掉避让（默认）时 `prev_head_valid=False`，状态空间等价于旧的 `(row, col, head)`。
_STATE_START = -1


def _barrier_turn_cell(grid: Grid, row: int, col: int, prev_head: int, head: int,
                       margin_m: float | None = None) -> bool:
    """★★ 当前格是不是"隔板相邻行的侧向 90° 转向"顶点。

    ---------- 现场机理（逐事件实测，见 `_tmp_session/t6_planner/probe_corner2.txt`） ----------
    隔板下方那一行（本图 = 第 7 行）只有 **0.175 m** 净宽（行心 `x=1.575` 到隔板 `x=1.75`），
    而车身在行内做侧向 90° 转向的横向摆动是"转弯半径 R 量级"
    （`R = 0.157/tan30° = 0.272 m`，见 `docs/路线加固盘点与候选方案.md` §3）
    —— 走廊半格**物理上装不下**，车会甩进隔板。

    `base_v015`（现场手工路线 @0.15）实测 353 拍触线 = **2 个事件，全部**落在同一形态上：
      · `N7`：车从 `N8` 沿行主轴（西）开到 `N7`，再转向南     —— 178 拍
      · `H7`：车从 `H8` 沿行主轴（西）开到 `H7`，再转向南     —— 175 拍
    `A11_v015` 219 拍 = 1 个事件，同样是这一形态（`H7`）—— 这就是该约束①
    "A11 单独上车反而更差"的几何原因：A11 只是把另一处挪到了行 8，没消掉这一处。
    同行的另两类拐角（`J7` 西→北、`E7` 西→北）实测**都不触线**：车从行 8 北上进入行 7、
    再折向西，摆动被行 8 吸收 —— 所以判据必须**区分"来向"**，不能只按"在第 7 行转向"一刀切。

    ---------- 判据（几何量全部来自地图/栅格本身，不新造角度阈值） ----------
    命中的充要形态：**车从"另一侧那一行"（本图 = 第 8 行）开进隔板紧邻行，
    在行内做一个侧向 90° 转向，再回/继续在行内或行外走**。逐条：
      ① 当前格不在 K 列（K 列桥洞另有 R007/R009 规则管）；
      ② 当前格就是隔板**紧邻**行（`row == grid.barrier_lo`，本图 = 第 7 行）；
      ③ 转向前的来向与转向后的去向**垂直**（90°，纯几何）；
      ④ 定向来向 = "从 `barrier_lo ± 1` 那一行进来"：由 `prev_head`（上一步的真实行进朝向）
         反推上一格；**上一格必须落在紧邻行**（本图 = 第 7 行）**或另一侧那一行**（第 8 行），
         即这一转向确实发生在"行内 / 行际"交界，而不是隔着半张地图的巧合；
      ⑤ 行内可用净宽（半格，`0.5 × geo.BLOCK_SIZE`）装不下这次转向所需的摆动：
         `margin_m` 是操作者显式给的**最小安全裕度**（`planner.barrier_turn_margin_m`）：
           * `≥ 0`（推荐 0.0）⇒ 半格 0.175 m 永远不够 ⇒ 一律拦下；
           * `< 0`（如 −1.0）⇒ **关闭本条**（退化成"只报不拦"的诊断模式）。

    ⚠️ **不变量**：`head`/`prev_head` 是 `_heading_of()` 的**行进方向**枚举
    （0=北 1=西 2=南 3=东），不是"车头朝向"。路线串里相邻点可能相隔多格
    （`N8→N7`、`N7→M7` 各跨 1 格），但每一格的行列**差值只有一个非零**，
    所以"来向"能由 `prev_head` 唯一还原（`prev_cell = (row,col) - DIRS[prev_head]`）。
    """
    if col == _CROSS_COL:
        return False
    # ★ 行号口径：规划器内部一律用 `_cell()` 的 **1-based 行号**（`block_to_index + 1`），
    #   与 `spec.barrier_row_lo/hi`（块行号）**同口径** —— 隔板在第 `barrier_row_hi` 与
    #   `barrier_row_lo` 行之间，所以"隔板紧邻行"就是 `barrier_row_lo`。
    bar_row = grid.barrier_lo                          # 隔板紧邻行（本图 = 第 7 行）
    if row != bar_row or grid.barrier_col <= 0:
        return False
    # 根因修正：隔板只存在于 barrier_col..N 列 —— A/B/C 列是**过板缺口**，那里
    #   没有隔板，行内转向物理上不可能蹭板。旧判据漏了列域，会把缺口处的合法转向
    #   误判成危险形态（tour 侧 penalty 默认 0 所以从未暴露；把判据接进
    #   tracker-safe 的 turn_gate 硬否决前必须修，否则 ABC 走廊的转向全被误杀）。
    if col < grid.barrier_col:
        return False
    if prev_head not in (0, 1, 2, 3):
        return False
    dr, dc = DIRS[int(prev_head)]
    prev_cell = (row - dr, col - dc)                   # 上一格（沿来向反推一格）
    dr2, dc2 = DIRS[int(head)]
    next_cell = (row + dr2, col + dc2)                 # 下一格（沿去向一格）
    if not (1 <= prev_cell[0] <= geo.MAP_ROWS and 1 <= next_cell[0] <= geo.MAP_ROWS):
        return False
    if prev_cell[0] not in (bar_row, bar_row + 1):
        return False
    d_out = _heading_of(next_cell[0] - row, next_cell[1] - col, -1)
    if d_out < 0:
        return False
    # 来向 = 上一步的真实行进朝向（`DIRS` 枚举本身就是单位方位，直接取奇偶性）；
    # 去向同理。两者不同 ⇒ 这是一次 90° 转向。
    if (int(prev_head) % 2) == (d_out % 2):
        return False
    margin = (float(margin_m) if margin_m is not None
              else float(getattr(grid, "barrier_turn_margin_m", 0.0)))
    if margin < 0.0:                                 # ★ 负值=关闭的旧暗门已封（钳到 0）
        margin = 0.0
    # ★ 判据方向（本轮在这里踩了两次）：`margin_m ≥ 0` ⇒ **认定行内装不下侧向 90° 转向**
    #   （行内可用净宽只有半格 0.175 m，而侧向转向要吃掉整行可用宽度）⇒ 命中、拦下。
    #   只在显式给出 `margin_m < 0` 时关闭本条。取 `≤` 而不是 `<`：margin=0 时必须命中。
    return 0.5 * geo.BLOCK_SIZE >= margin - 1e-9


def _door_turn_margin_m(grid: Grid, row: int, col: int) -> float:
    """★★ 约束①：该格作为**过门转弯点**时，离隔板东端的余量（米）。

    = `(index(barrier_col_from) − col) × 格宽`；现场 D 列 ⇒ A 1.050 / B 0.700 / C 0.350 /
    D 0.000 m。**只在隔板相邻行（`barrier_row_lo`，现场第 7 行）有意义**；其它行返回 +inf
    （不构成过门转弯，不参与约束）。判据取"行"而不是列位：过门转弯就是把车从隔板相邻行
    拐进 A/B/C 列缺口的那一次转向。

    ★ 为什么不放进 `_moves()`（搜索代价）而只放在顺序层 `penalty_of()`：
    过门转弯的**转角**由顺序决定（`a→b→c` 三点），不是单步动作的性质；放在顺序层
    才能和 `mine_turn_penalty` / `barrier_turn_penalty` 用同一把尺子，且**缺省 0.0 时
    对 v2.3 零影响**。
    """
    if row != grid.barrier_lo:
        return float("inf")
    return (grid.barrier_col - col) * geo.BLOCK_SIZE


def _phys_params(cfg: "Config | None") -> dict:
    """★ 从 config 读**物理几何**参数（真源在 `config/robot.yaml`，公式在 `physics.py`）。

    `None`（无 config，例如纯单元测试）⇒ 用 `physics.py` 的默认形参，行为等价于现场值。
    :return: ``{length_m, width_m, coverage_radius_m, obstacle_radius_m, mine_pad_m,
        barrier_width_m, barrier_cols, obstacle_margin_m, wheelbase_m, max_steer_deg,
        bridge_entry_redundancy_m}``
    """
    def g(key: str, fallback: float | str) -> Any:
        if cfg is None:
            return fallback
        return V._cfg_get(cfg, key, fallback)                             # noqa: SLF001
    return {
        "length_m": float(g("vehicle.length_m", 0.28)),
        "width_m": float(g("vehicle.width_m", 0.135)),
        "coverage_radius_m": float(g("vehicle.coverage_radius_m", 0.035)),
        "obstacle_radius_m": float(g("field.obstacle_radius_m", 0.1275)),
        "mine_pad_m": float(g("field.mine_pad_m", 0.12)),
        "barrier_width_m": float(g("field.barrier_width_m", 0.02)),
        "barrier_cols": str(g("field.barrier_cols", "D..N")),
        "obstacle_margin_m": float(g("planner.obstacle_margin_m", 0.02)),
        "wheelbase_m": float(g("pursuit.wheelbase_m", 0.157)),
        "max_steer_deg": float(g("pursuit.max_steer_deg", 30.0)),
        # 桥洞进洞侧冗余长度（= 1/4 格）；缺 key ⇒ 兜底常量（与 YAML 同值）
        "bridge_entry_redundancy_m": float(
            g("field.bridge_entry_redundancy_m", BRIDGE_ENTRY_REDUNDANCY_M)),
        # 进洞前 1 格拐弯的惩罚（米）；缺 key ⇒ 兜底常量（与 YAML 同值）
        "bridge_approach_align_penalty_m": float(
            g("planner.bridge_approach_align_penalty_m", BRIDGE_APPROACH_ALIGN_PENALTY_M)),
    }


def _door_turn_required_margin_m(turn_deg: float, phys: dict) -> float:
    """★ 过门转弯所需的**最小余量**（几何推导，不是固定 0.5 m）。

    `所需余量 ≥ R·sinθ + w/2·cosθ`，其中 `R = wheelbase/tan(max_steer)`（与 `pursuit.*` 同源）、
    `w/2` = 车半宽、`θ` = 转弯角。

    为什么是这个式子：车绕顶点转 θ 时，车体中心相对顶点的横向外扩 = `R·sinθ`（圆弧外扩）
    `+ w/2·cosθ`（车身横放时的半宽投影），两者之和就是必须让出的横向空间。

    现场 90°：`R = 0.27193`、`w/2 = 0.0675` ⇒ `0.27193 + 0` = **0.2719 m**。
    `θ = 0`（直行）⇒ `w/2 = 0.0675 m`。
    ⚠️ `θ → 180°`（掉头）时 `cosθ → −1`，裸式子会给出负数 —— 物理上掉头需要的横向空间
    远大于两个端点，故取 `max(w/2, …)` 兜底（不用 `|cosθ|`：掉头不是"变窄"）。
    """
    rad = math.radians(max(0.0, min(180.0, float(turn_deg))))
    _, r_min = _phys.sweep_radius(length_m=phys["length_m"], width_m=phys["width_m"],
                                  wheelbase_m=phys["wheelbase_m"],
                                  max_steer_deg=phys["max_steer_deg"])
    half_w = phys["width_m"] / 2.0
    return max(half_w, r_min * math.sin(rad) + half_w * math.cos(rad))


def _turn_deg_from_dirs(entry: int, leave: int) -> float:
    """两个行进方位（0北/1西/2南/3东）之间的**转角**（度，∈ [0,180]）。

    直行 = 0°；正交转向（90° 或 270°）= **90°**；反向 = 180°。
    """
    delta = abs((int(entry) - int(leave)) % 4)
    return {0: 0.0, 1: 90.0, 2: 180.0, 3: 90.0}[delta]


def _door_turn_margin_m(grid: Grid, row: int, col: int, *,
                        turn_deg: float = 90.0,
                        phys: "dict | None" = None) -> float:
    """★★ 该格作为**过门转弯点**时的**可用余量**（米）。

    = `(index(barrier_col_from) − col) × 格宽`；现场 D 列 ⇒ A 1.050 / B 0.700 / C 0.350 /
    D 0.000 m。**只在隔板相邻行（`barrier_row_lo`）有意义**；其它行返回 `inf`（不构成过门转弯）。

    ⚠️ 早期版本返回的就是这个**可用余量**，而判据侧的阈值是**固定 0.5 m**；
    现在阈值改成 `_door_turn_required_margin_m(turn_deg, phys)`（几何推导）——
    调用方要比较 `可用余量 >= 所需余量`，**不要再写死 0.5**。
    """
    if row != grid.barrier_lo:
        return float("inf")
    return (grid.barrier_col - col) * geo.BLOCK_SIZE


def _obstacle_turn_clearance_m(grid: Grid, cell: tuple[int, int],
                              obstacles: Iterable[tuple[int, int]], *,
                              at_corner: bool = True, phys: "dict | None" = None) -> float:
    """★ 该转弯点到最近**障碍柱心**的距离（米）—— 判据 `>= 所需中心距`。

    所需中心距 = `柱半径 + 车身半径 + 余量`（`physics.required_obstacle_center_distance_m`）：
      · **直段** `车身半径 = 车半宽 0.0675` ⇒ **0.215 m**；
      · **拐角** `车身半径 = 扫掠半径 0.15542` ⇒ **0.3029 m**（转弯时车身外包圆更大）。
    `inf`（无障碍或无障碍格）表示无约束。
    """
    if not obstacles:
        return float("inf")
    x, y = geo.block_to_meters(geo.index_to_block(cell[0] - 1, cell[1]))
    best = float("inf")
    for (orow, ocol) in obstacles:
        ox, oy = geo.block_to_meters(geo.index_to_block(orow - 1, ocol))
        d = _phys.obstacle_clearance_point_m(x, y, ox, oy)
        if d < best:
            best = d
    return best


def _required_obstacle_clearance_m(phys: dict, *, at_corner: bool) -> float:
    """★ 障碍净空门槛（米）——直段用车身半径、拐角用扫掠半径，**不写常量**。"""
    sweep, _r = _phys.sweep_radius(length_m=phys["length_m"], width_m=phys["width_m"],
                                   wheelbase_m=phys["wheelbase_m"],
                                   max_steer_deg=phys["max_steer_deg"])
    body_r = sweep if at_corner else phys["width_m"] / 2.0
    return _phys.required_obstacle_center_distance_m(
        obstacle_radius_m=phys["obstacle_radius_m"], body_radius_m=body_r,
        margin_m=phys["obstacle_margin_m"])


def _mine_pressed_at(cell: tuple[int, int], mine_cell: tuple[int, int],
                     phys: dict) -> bool:
    """★ 车心在 `cell`（1-based 行）时，`mine_pad` 是否被压到（`physics.mine_pressed`）。

    判据 = 车心 3.5 cm 圆与 **0.12×0.12 垫片**（中心在雷格格心）产生过重叠
    ⇔ 车心到垫片最近距离 ≤ `coverage_radius_m`。
    ⚠️ 这**不是**"车心到雷心 ≤ 0.035"：贴边压过时车心离雷心可到 `0.06+0.035 = 0.095 m`。
    """
    x, y = geo.block_to_meters(geo.index_to_block(cell[0] - 1, cell[1]))
    mx, my = geo.block_to_meters(geo.index_to_block(mine_cell[0] - 1, mine_cell[1]))
    return _phys.mine_pressed(x, y, mx, my,
                              coverage_radius_m=phys["coverage_radius_m"],
                              mine_pad_m=phys["mine_pad_m"])


def _edge_mine_turn_gap(seq_cell: tuple[int, int],
                        edge_mines: Iterable[tuple[int, int]]) -> int:
    """★★ 约束②：该转弯点到"最外行/列雷"的最小曼哈顿格距。

    返回格数（>=0）；`edge_mines` 为空时返回一个大数（= 无约束）。
    """
    row, col = seq_cell
    best = 10 ** 9
    for (mr, mc) in edge_mines:
        d = abs(row - mr) + abs(col - mc)
        if d < best:
            best = d
    return best


def _edge_mine_cells(grid: Grid, spec: MapSpec) -> set[tuple[int, int]]:
    """★ 约束② 作用对象：位于**最外行（1 / MAP_ROWS）或最外列（A / N）**的雷格（1-based 行）。"""
    out: set[tuple[int, int]] = set()
    for mine in spec.mines:
        r, c = geo.block_to_index(mine)
        row, col = r + 1, c
        if row in (1, geo.MAP_ROWS) or col in (0, geo.MAP_COLS - 1):
            out.add((row, col))
    return out


def _moves(grid: Grid, row: int, col: int, head: int,
           prev_head: int = _STATE_START, prev_valid: bool = False,
           barrier_turn_penalty: float = 0.0):
    """``((row, col, head, prev_head, prev_valid), cost, action)``：阿克曼可执行的动作集。

    `grid.allow_reverse=False`（无倒车模式）时**不产出倒车动作**：真车底盘不支持倒车
    （负速度方向位被 STM32 无视），所以路线里任何依赖倒车的路径都必须改用
    "正向开进 + 原地转向 + 正向开出"（顶点处由 TrackerTask PIVOT）。

    `barrier_turn_penalty > 0` 时给 `_barrier_turn_cell()` 命中的转向动作加软代价
    （见该函数文档：隔板相邻行的死胡同式 90° 转向 = 车会甩进隔板）。
    """
    drow, dcol = DIRS[head]
    nrow, ncol = row + drow, col + dcol
    if grid.move_ok(row, col, nrow, ncol):
        yield (nrow, ncol, head, head, True), COST_FORWARD + grid.danger(nrow, ncol), ACT_FORWARD
    if not grid.allow_reverse:
        left, right = (head + TURN_LEFT) % 4, (head + TURN_RIGHT) % 4
        extra = (barrier_turn_penalty
                 if (barrier_turn_penalty > 0.0 and prev_valid
                     and _barrier_turn_cell(grid, row, col, prev_head, left)) else 0.0)
        yield (row, col, left, prev_head, prev_valid), COST_TURN + extra, ACT_LEFT
        extra = (barrier_turn_penalty
                 if (barrier_turn_penalty > 0.0 and prev_valid
                     and _barrier_turn_cell(grid, row, col, prev_head, right)) else 0.0)
        yield (row, col, right, prev_head, prev_valid), COST_TURN + extra, ACT_RIGHT
        return
    brow, bcol = row - drow, col - dcol
    if grid.move_ok(row, col, brow, bcol):
        yield (brow, bcol, head, head, True), COST_BACKWARD + grid.danger(brow, bcol), ACT_BACKWARD
    yield (row, col, (head + TURN_LEFT) % 4, prev_head, prev_valid), COST_TURN, ACT_LEFT
    yield (row, col, (head + TURN_RIGHT) % 4, prev_head, prev_valid), COST_TURN, ACT_RIGHT


def flood(grid: Grid, source: tuple[int, int, int],
          barrier_turn_penalty: float = 0.0):
    """从 ``(row, col, heading)`` 出发的**一次** Dijkstra 洪泛。

    :return: ``(dist, came, actions, settled, rank)``；
        ``dist[(row, col, head, prev_head, prev_valid)]`` = 最短代价，``came`` 是前驱状态，
        ``actions[...]`` = 到达该状态用的动作，``settled`` = 实际弹出的状态数
        （搜索代价的读数，供日志/断言用），``rank`` = 各状态的**收紧次序**
        （同代价多状态时的确定性 tie-break，见 `_reconstruct`）。

    ★ 状态比"位置+朝向"多带一维 ``prev_head``（上一段的行进朝向）—— 隔板相邻行的
    "死胡同式 90° 转向"判据需要它（见 `_barrier_turn_cell`）。`barrier_turn_penalty=0.0`
    （默认）时 `prev_valid` 恒为 False、`prev_head` 恒为初始值，状态空间与旧版等价，
    输出的路径也逐字相同（`_reconstruct` 的路径元组仍是 `(row, col, head, action)`）。

    ★ 为什么要洪泛而不是逐对 A*：代价矩阵要"一个点到其它所有点 × 四个到达朝向"，
      逐对 A* 是 16 次搜索/对，19 个点就是 2700+ 次 —— 直接吃光时间预算。
      状态空间只有 11×14×4 = 616，一次洪泛就能拿到全部朝向的距离，
      而且**仍然是最优解**（Dijkstra，不做启发式剪枝）。
    """
    counter = 0
    settled = 0
    src = (source[0], source[1], source[2], _STATE_START, False)
    heap: list[tuple[float, int, tuple[int, int, int, int, bool]]] = [(0.0, counter, src)]
    dist: dict[tuple[int, int, int, int, bool], float] = {src: 0.0}
    came: dict[tuple[int, int, int, int, bool], tuple[int, int, int, int, bool] | None] = {src: None}
    actions: dict[tuple[int, int, int, int, bool], int] = {src: ACT_START}
    #: ★ 状态的**收紧次序**（= 它第一次拿到有限距离时的序号）。用途：`_reconstruct` /
    #: `_flood_value` 在"位置+朝向相同、代价也相同"的多个状态之间必须**确定性**地选一个。
    #: 为什么必要：状态多带 `prev_head` 后，同一格同一朝向会出现多个同代价状态，
    #: 而纯字典迭代序不是确定性的 —— 会选出"代价一样但几何不同"的腿（本轮实测踩过：
    #: 默认参数下路线突然在下半场穿 K 列两次）。用收紧次序做 tie-break，
    #: 等价于旧版"一个 (格, 朝向) 只有一个状态"的语义。
    rank: dict[tuple[int, int, int, int, bool], int] = {src: 0}

    while heap:
        value, _, cur = heapq.heappop(heap)
        if value > dist.get(cur, float("inf")) + 1e-9:
            continue
        settled += 1
        row, col, head, prev_head, prev_valid = cur
        for nxt, cost, action in _moves(grid, row, col, head, prev_head, prev_valid,
                                       barrier_turn_penalty):
            nv = value + cost
            if nv < dist.get(nxt, float("inf")) - 1e-9:
                dist[nxt] = nv
                came[nxt] = cur
                actions[nxt] = action
                counter += 1
                rank[nxt] = counter
                heapq.heappush(heap, (nv, counter, nxt))
    return dist, came, actions, settled, rank


def _reconstruct(dist: dict, came: dict, actions: dict, goal: tuple[int, int, int],
                 rank: dict | None = None) -> tuple[list[tuple[int, int, int, int]], float]:
    """从洪泛结果里取出一条最短路径（目标朝向固定）。

    ★ 洪泛的状态多带 `prev_head/prev_valid` 两维，但**回传的路径元组仍是
      `(row, col, head, action)`** —— 调用方（`_collapse` / `_Tour.path` /
      `plan_route` 的编译段）不需要知道新增维度。取法：在 `dist` 里找"位置+朝向
      匹配 goal"的最优状态，再沿 `came` 回溯。同代价多状态时按 `rank`（收紧次序）
      tie-break —— 保证与旧版"每 (格, 朝向) 只有一个状态"的语义等价且**确定性**。
    """
    best: tuple[float, int, tuple[int, int, int, int, bool]] | None = None
    for state, value in dist.items():
        if state[0] != goal[0] or state[1] != goal[1] or state[2] != goal[2]:
            continue
        # ★ 容差比较：多状态投影下同一 (格, 朝向) 会出现**相差 1e-15 的等价代价**
        #   （浮点求和次序不同），严格 `<` 会让"谁更小"变成噪声判决 —— 必须按
        #   (代价, 收紧次序) 全序比较，否则默认参数下路线会变（本轮实测踩过）。
        rk = (rank or {}).get(state, 1 << 30)
        if best is None or (value, rk) < (best[0], best[1]):
            best = (value, rk, state)
    if best is None:
        return [], float("inf")
    path: list[tuple[int, int, int, int]] = []
    node: tuple[int, int, int, int, bool] | None = best[2]
    while node is not None:
        path.append((node[0], node[1], node[2], actions.get(node, ACT_START)))
        node = came[node]
    path.reverse()
    return path, best[0]


def _flood_value(dist: dict, goal: tuple[int, int, int], rank: dict | None = None) -> float:
    """`dist` 里"位置+朝向"匹配 `goal` 的最优代价（状态多带 `prev_head` 维时的取法）。

    与 `_reconstruct` 用**同一条**取法（含同一条 tie-break）—— 否则
    `_Tour.cost()`（读 `dist`）与 `_Tour.path()`（回溯）会挑到不同的腿，
    "代价说 A、路线走 B"。
    """
    best: tuple[float, int] | None = None
    for state, value in dist.items():
        if state[0] != goal[0] or state[1] != goal[1] or state[2] != goal[2]:
            continue
        key = (value, (rank or {}).get(state, 1 << 30))
        if best is None or key < best:
            best = key
    return best[0] if best is not None else float("inf")


def _heading_of(drow: int, dcol: int, fallback: int) -> int:
    """位移 -> 朝向索引；零位移返回 fallback。"""
    if drow < 0:
        return 0
    if drow > 0:
        return 2
    if dcol < 0:
        return 1
    if dcol > 0:
        return 3
    return fallback


def _path_reverse(path: Sequence[tuple[int, int, int, int]]) -> list[bool]:
    """逐步路径 -> 每一步是否倒车（``back[i]`` 指到达点 i 的那一步）。"""
    return [action == ACT_BACKWARD for (_, _, _, action) in path]


# --------------------------------------------------------------------------- #
#  判据（复用 validate 的口径）
# --------------------------------------------------------------------------- #
def stop_distance_m(block_a: str, block_b: str) -> float:
    """两格中心的欧氏距离（米）。

    **复用** `validate._euc` —— 它与 `tools/shoot_planner.dist_m`、
    `tools/route_check.py` 的射击点规则是同一个算法（`BLOCK_SIZE × 格距`）。
    """
    return V._euc(Waypoint.at_block(block_a), block_b)      # noqa: SLF001


def turn_angle_deg(start: tuple[int, int], mid: tuple[int, int],
                   end: tuple[int, int]) -> float:
    """折返角（度）：进入段与离开段的夹角。0° = 直行，180° = 原地掉头。"""
    d1 = (mid[0] - start[0], mid[1] - start[1])
    d2 = (end[0] - mid[0], end[1] - mid[1])
    n1, n2 = math.hypot(*d1), math.hypot(*d2)
    if n1 < 1e-9 or n2 < 1e-9:
        return 0.0
    cosv = max(-1.0, min(1.0, (d1[0] * d2[0] + d1[1] * d2[1]) / (n1 * n2)))
    return math.degrees(math.acos(cosv))


def plate_center_m(spec: MapSpec) -> tuple[float, float]:
    """靶板中心 = **各靶格中心的算术中点**（米制，地图帧）。

    现场靶区是**一块跨格薄板**（第五张图 = `E5`+`F5` 两格 ⇒ 中心 = 两格心中点
    `(2.2750, 3.1500)`）。旧口径取"到**靶格**的距离"，而实际打的是**板心**：
    实测 `E4` 到 E5 格心 0.35 m（看着合规），到板心只有 **0.3913 m** ⇒ 旧口径
    选出的停站点其实比标称近得多。本函数给新口径提供唯一的板心真源。

    `targets` 为单格时退化为该格中心（与旧口径在该退化情形下一致）。
    """
    pts = [geo.block_to_meters(t) for t in spec.targets]
    if not pts:
        return (0.0, 0.0)
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


#: SHOOT 停站点到**靶板中心**的距离窗口（米）。现场真源 =
#: `config/robot.yaml` 的 `route.shoot_distance_m` / `route.shoot_distance_tol_m`
#: 现役值 **0.90 / 0.12 ⇒ 窗口 [0.78, 1.02]**（命中 `D3`，板心 0.8750 m）。
#: 缺键时用本常量兜底（不硬编码第二份真源 ⇒ 必须与 `robot.yaml` 同值，`test_planner` 会钉）。
SHOOT_DISTANCE_M_DEFAULT = 0.90
SHOOT_DISTANCE_TOL_M_DEFAULT = 0.12


def shoot_distance_window(accept: tuple[float, float] | None = None) -> tuple[float, float]:
    """停站点到板心的接受窗口 `(lo, hi)`（米，升序）。

    `accept` 可以是 `(目标距离, 半宽)`；缺省用 `SHOOT_DISTANCE_*` 常量。
    恒满足 `lo <= 目标 <= hi`（半宽为负时夹到 0）。
    """
    if accept is None:
        mid, tol = SHOOT_DISTANCE_M_DEFAULT, SHOOT_DISTANCE_TOL_M_DEFAULT
    else:
        mid, tol = float(accept[0]), abs(float(accept[1]))
    return mid - tol, mid + tol


def plate_bisector_axis_m(spec: MapSpec) -> tuple[float, float]:
    """轨道**垂直平分线**在网格坐标里的表达（排序依据）。

    靶板是**两格相连**的一处轨道靶区 ⇒ 两个格心连线就是轨道轴；其**垂直平分线**
    经过板心。网格上一般**不存在正好落在平分线上的格心**，所以口径是"**离平分线最近**"
    而不是"在平分线上"。返回 `(nx, ny)`：平分线的**法向**（= 轨道轴方向），
    `|(P − 板心)·n|` 就是点 P 到平分线的距离。

    单格靶区（无轴可言）返回 `(0, 0)` ⇒ 该项恒 0，不影响排序。
    """
    pts = [geo.block_to_meters(t) for t in spec.targets]
    if len(pts) < 2:
        return (0.0, 0.0)
    ax = (pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
    n = math.hypot(*ax)
    return (0.0, 0.0) if n < 1e-12 else (ax[0] / n, ax[1] / n)


def plate_face_normal_m(spec: MapSpec) -> tuple[float, float]:
    """靶板**板面法向**（= 垂直于两靶格连线的单位向量，米制地图帧）。单格靶区返回 (0,0)。

    ★ 为什么必须与 plate_bisector_axis_m() 分开（2026-10-18 用户指令「射击点要顾及
      靶子的两个面」）：两靶格连线是**轨道轴**（板沿它排开），而板面法向是它的**垂线** ——
      两者差 90°，拿错轴的后果是把「南北」当成「东西」：
      新图靶板 D7+E7（两格沿 y 排开、板心 (1.575,3.5)、面法向 = ±x）上，
      C9=(0.875,4.025) 与 F9=(0.875,2.975) 的 x 都小于板心 ⇒ **物理上同在南侧**；
      旧实现按轨道轴 (0,-1) 投影（比 y）却判出 C9=-1 / F9=+1 ⇒ 「双停站」是假的。
    """
    ax, ay = plate_bisector_axis_m(spec)
    if ax == 0.0 and ay == 0.0:
        return (0.0, 0.0)
    return (-ay, ax)                      # 逆时针 90°：法向


def barrier_sight_blocked(spec: MapSpec, p: tuple[float, float],
                          q: tuple[float, float]) -> bool:
    """线段 `p→q` 是否被**隔板**挡住。

    隔板 = 第 `barrier_row_hi` 与 `barrier_row_hi+1` 行之间、`barrier_col_from`~`N` 列
    的**一条连续直线**（现场 = 第 6/7 行之间 D~N，无缺口；见 `config/map.yaml`）。
    判据 = 线段在 x 方向**跨越**隔板所在 x（两行格心中点）且交点 y 落在隔板 y 区间内
    —— 与 `validate._barrier_crossing` 同一几何口径，只是这里判的是**视线**而不是行驶。

    ★ 为什么需要它（实测，不是预防性设计）：
      窗口放宽后，落进窗口的可通行格里会同时出现**隔板另一侧**的 `E8` / `F8`
      （到板心距离同样在窗口内）。规划器原来只判"距离 + `grid.passable()`"，
      会**优先选中它们**（在下半场走顺路、代价更低）；而靶板在上半场、中间是
      D~N 连续无缺口的隔板 ⇒ 视线被挡，物理上打不到。
      `tools/route_check.py` 与 `validate_route` 都**只查距离**，这种"看得见数字、
      看不见墙"的停站点照样全绿 ⇒ 判据必须由规划器自己兜住。
      现役窗口 [0.78,1.02]（停站 `D3`）下**同侧候选已在窗口内**、`E8`/`F8` 都在窗外
      ⇒ 本判据当前"不触发"；但它是**窗口的函数**：任何人把窗口推远都会立刻需要它
      ⇒ 保留，不因"当前不触发"删掉。回归里由 `test_planner` 的 4b 段守"不许静默选错点"。
    """
    try:
        col_from = geo.COL_LETTERS.index(str(spec.barrier_col_from).strip().upper())
    except ValueError:                                  # 隔板列名非法 ⇒ 不拦（交给校验器报错）
        return False
    row_hi = int(spec.barrier_row_hi)
    if not 1 <= row_hi < geo.MAP_ROWS:
        return False
    # 隔板 x = 第 row_hi 行与第 row_hi+1 行**格心中点**（行号增大 ⇒ x 减小）
    x_wall = 0.5 * (geo.block_to_meters(geo.index_to_block(row_hi - 1, 0))[0]
                    + geo.block_to_meters(geo.index_to_block(row_hi, 0))[0])
    y_from = geo.block_to_meters(geo.index_to_block(0, col_from))[1]
    y_to = geo.block_to_meters(geo.index_to_block(0, geo.MAP_COLS - 1))[1]
    y_lo, y_hi = min(y_from, y_to), max(y_from, y_to)
    d0, d1 = p[0] - x_wall, q[0] - x_wall
    if d0 * d1 >= 0.0:                                  # 同侧（或端点贴墙）⇒ 不可能被隔板挡
        return False
    t = (x_wall - p[0]) / (q[0] - p[0])
    y = p[1] + t * (q[1] - p[1])
    # ★靶窗例外（2026-10-18 用户指令：「射击点就是要顾及靶子的两个面」）：
    #   靶板本身嵌在两半场隔板上 ⇒ 从隔板另一侧看过去时，视线交点必然落在**靶板自己的
    #   y 跨度内**（穿过靶窗），这不是"被墙挡"，而是"正好从靶窗看靶"。
    #   窗口 = 靶格 y 跨度 ± 半格（靶板可能比格心连线略宽半格）。
    #   ⚠️ 只对"靶板确实嵌在这道隔板上"的图生效：靶格 x 与墙 x 相差 ≤ 半格；
    #   其余图/其余方向仍按整墙拦（不做"任何靶都能穿墙看"的通用放行）。
    half = 0.5 * geo.BLOCK_SIZE
    tgt_y = [geo.block_to_meters(tg)[1] for tg in spec.targets]
    tgt_x = [geo.block_to_meters(tg)[0] for tg in spec.targets]
    on_wall = any(abs(tx - x_wall) <= half + 1e-9 for tx in tgt_x)
    if on_wall and tgt_y and (min(tgt_y) - half - 1e-9) <= y <= (max(tgt_y) + half + 1e-9):
        return False
    return y_lo <= y <= y_hi


def shoot_candidates(spec: MapSpec, grid: Grid,
                     accept: tuple[float, float] | None = None,
                     *, pool: Sequence[str] | None = None) -> list[str]:
    """SHOOT 停站点的候选格（新口径）：到**靶板中心**距离落在窗口内的可通行格。

    ## 口径说明

    旧口径（早期）：候选 = 各靶格的**正交邻居**，且到**该靶格中心**的距离落在
    `accept_window(0.20~0.45)` 内 —— 现役图命中 `E4`（到 E5 0.35 m）。
    新口径：候选 = **任意**可通行格，且到**板心**距离落在
    `[目标 − 半宽, 目标 + 半宽]`（默认 **[0.70, 0.90]**）内；再排序取首选。

    ## 排序（定序三键，全序、确定性）

    1. **离轨道垂直平分线最近**（`|(P−板心)·n|`）—— 车停在平分线上时，靶沿轨道
       摆动带来的距离变化**最小**（扫掠区间最窄）；
    2. **距离误差最小**（`|到板心距离 − 目标距离|`）—— 落在标称距离附近；
    3. **格名序**（`sorted`）—— 兜底，保证可复现。

    ## 排除项

    `grid.passable()` 已含障碍/雷格/隔板/K 列桥洞约束（与规划器同源）；
    另显式剔除 `spec.obstacles` 与 `spec.mines`，避免"贴靶的雷格"被选中
    （实测：第五张图 `E3` 到板心 0.7215 m，与 `F3` 同距，但 `E3` 是**雷**必须排除）。
    ★ 再剔除**视线被隔板挡住**的格（`barrier_sight_blocked()`）—— 否则窗口放宽后
    隔板另一侧的 `E8`/`F8`（板心同样 1.0645 m）会被当成合法停站点，而那堵墙
    `route_check` / `validate_route` **都不判**。

    :param accept: `(目标距离, 半宽)`；缺省 = `SHOOT_DISTANCE_M_DEFAULT/TOL_M_DEFAULT`。
    :param pool: 限制候选集（如"现役路线上的格"）；缺省 = 整张图扫格。
    :returns: 按上述三键排好序的候选格名列表（**首选在前**）。
    """
    lo, hi = shoot_distance_window(accept)
    ctr_x, ctr_y = plate_center_m(spec)
    bis_x, bis_y = plate_bisector_axis_m(spec)
    blocked = set(spec.obstacles) | set(spec.mines)
    mid = 0.5 * (lo + hi)

    if pool is not None:
        cells: list[str] = []
        for name in pool:
            text = str(name).strip().upper().rstrip("*")
            if text and text not in cells:
                cells.append(text)
    else:
        cells = [geo.index_to_block(r - 1, c)
                 for r in range(1, geo.MAP_ROWS + 1) for c in range(geo.MAP_COLS)]

    ranked: list[tuple[float, float, str]] = []
    for block in cells:
        if block in blocked:
            continue
        try:
            row_idx, col_idx = geo.block_to_index(block)
        except ValueError:
            continue
        if not grid.passable(row_idx + 1, col_idx):
            continue
        x, y = geo.block_to_meters(block)
        dist = math.hypot(x - ctr_x, y - ctr_y)
        if not (lo - 1e-9 <= dist <= hi + 1e-9):
            continue
        if barrier_sight_blocked(spec, (x, y), (ctr_x, ctr_y)):
            continue
        off_bisector = abs((x - ctr_x) * bis_x + (y - ctr_y) * bis_y)
        ranked.append((round(off_bisector, 9), round(abs(dist - mid), 9), block))
    ranked.sort()
    return [block for _off, _err, block in ranked]


def accept_window(limits: tuple[float, float]) -> tuple[float, float]:
    """规划器采用的射击点接受窗口 = `tools/route_check.py` 的容差窗口。

    见 `ROUTE_CHECK_TOL_LO/HI` 的说明：规划器不自创阈值，接受窗口跟随"真正会执行的那条
    校验规则"。返回值仍是 ``(lo, hi)`` 升序。
    """
    lo, hi = (limits if limits[0] <= limits[1] else (limits[1], limits[0]))
    return lo - ROUTE_CHECK_TOL_LO, hi + ROUTE_CHECK_TOL_HI


def covering_stops(spec: MapSpec, pool: Sequence[str],
                   accept: tuple[float, float] | None = None) -> dict[str, tuple[str, ...]]:
    """每个候选射击点**能覆盖到哪些靶格**（`block -> (靶格, ...)`），按接受窗口判。

    ★ 口径（与 `shoot_candidates()` 同步）：判据从「到**靶格中心** ∈
    0.20~0.45（`accept_window(_limits(cfg))`）」改为「到**靶格中心** ∈ **板心窗口**
    `[目标 − 半宽, 目标 + 半宽]`」（默认 **[0.70, 0.90]**）。
    为什么必须同步：新口径把停站点推到离**板心** 0.70~0.90 m ⇒ 到"较远那个靶格"
    的距离必然 ≈ 0.70 m 以上（实测 `F3` 到 `E5` 0.7826 / 到 `F5` 0.7000），
    用旧窗口（≤0.51）判会得出「覆盖 0/2」而把**正确落点**判死。

    覆盖语义 = **逐个靶格**判"本停站点是否落在该靶的有效射击距离内"（与
    `tools/route_check.py` 的任务层口径一致：它也是"存在某个射击点到该靶 ∈ 区间"，
    不要求同一个点同时贴住两个格心）。默认取首选候选即可满足全覆盖。
    """
    lo, hi = shoot_distance_window(accept)
    return {block: tuple(t for t in spec.targets
                         if lo - 1e-9 <= stop_distance_m(block, t) <= hi + 1e-9)
            for block in pool}


def plate_side_of(spec: MapSpec, block: str) -> int:
    """格 `block` 在靶板**哪一面**：+1 / -1（0 = 正好在板心平面或点名非法）。

    判据 = 该格相对板心的位移在 **板面法向**（plate_face_normal_m()，= 两靶格连线的
    垂线）上的投影符号 ⇒ 同一侧看到同一个板面；两侧看到**相反**的板面。
    ★ 2026-10-18 修：旧实现错把「靶格连线方向」（轨道轴）当法向，两靶沿 y 排开时会在
      南北（±x）两侧都给出 ±1 —— 详见 plate_face_normal_m() 的注释。

    ★ 物理依据：靶板一面蓝一面红，**蓝面朝向每局随机但恒定**。只打蓝、
    打红扣分 ⇒ 必须两侧各留一个停站，保证至少一侧必见蓝面（见 `second_shoot_stop`）。
    """
    if not block:
        return 0
    ctr_x, ctr_y = plate_center_m(spec)
    nx, ny = plate_face_normal_m(spec)
    try:
        x, y = geo.block_to_meters(block)
    except ValueError:
        return 0
    proj = (x - ctr_x) * nx + (y - ctr_y) * ny
    if proj > 1e-9:
        return 1
    if proj < -1e-9:
        return -1
    return 0


def second_shoot_stop(spec: MapSpec, grid: Grid, primary: str,
                      accept: tuple[float, float] | None = None) -> str:
    """★ 与主射击停站**分居板面两侧**的第二个停站候选（无解返回 `""`）。

    筛选与排序完全复用 `shoot_candidates()`（窗口 / 可通行 / 视线不被隔板挡 / 三键定序），
    只多两条：① 必须在主停站的**对侧**（`plate_side_of` 符号相反）；② 必须与主停站一样
    **覆盖全部靶格**（否则"换个停站还得再打另一块靶"就白折腾）。

    为什么不做成"再搜一条经过它的路线"：现役 44 点路线的 `G3` 本就在路径上（`D3*` → …
    → `G3` → …），把 `*` 标上去即可，**零路径改动**、不引入新的穿洞/折返风险。
    调用方（tracker-safe 编译）只在"它确实是路线上的普通航点"时才会落标。
    """
    if not primary or not spec.targets:
        return ""
    side = plate_side_of(spec, primary)
    if side == 0:
        return ""
    lo, hi = shoot_distance_window(accept)
    cover = covering_stops(spec, shoot_candidates(spec, grid, accept), accept)
    ctr_x, ctr_y = plate_center_m(spec)
    for cand in shoot_candidates(spec, grid, accept):
        if cand == primary:
            continue
        if plate_side_of(spec, cand) != -side:
            continue
        if len(cover.get(cand, ())) != len(spec.targets):
            continue
        x, y = geo.block_to_meters(cand)
        dist = math.hypot(x - ctr_x, y - ctr_y)
        if not (lo - 1e-9 <= dist <= hi + 1e-9):
            continue
        return cand
    return ""


def shoot_stop_distance(block: str, map_cfg: Any, *, source: str = "") -> float:
    """停站格到**靶板中心**的距离（米）—— 与 `shoot_candidates()` 同一口径。

    板心 = 各靶格中心之算术中点（`plate_center_m`）。`PlanResult.shoot_distance_m`
    用的是同一个式子，所以"事后补标的第二停站"也能给出可对账的数字，而不是 0。
    """
    spec = map_cfg if isinstance(map_cfg, MapSpec) else load_map(map_cfg, source=source)
    ctr_x, ctr_y = plate_center_m(spec)
    x, y = geo.block_to_meters(block)
    return math.hypot(x - ctr_x, y - ctr_y)


def ensure_second_shoot_stop(tokens: Sequence[str], map_cfg: Any,
                             cfg: "Config | None" = None, *,
                             source: str = "") -> tuple[list[str], str, str]:
    """★ 自动规划产物的「双射击停站」落标 —— **零路径改动**，只补一个 `*`（t7）。

    ## 为什么需要这一层（现场事故的根因链）

    靶板一面蓝一面红、蓝面朝向**每局随机但恒定**，只打蓝、打红扣分 ⇒ 路线必须在靶板
    **两侧各留一个射击停站**（`second_shoot_stop()` 的判据）。但"两个停站"在引擎里的
    实现形态是**编译期顺手落标**（tracker-safe 的 `_compile`）——
      · `tour` 引擎（缺省引擎）**没有这段落标**：它的 `plan_route()` 只标主停站；
      · tracker-safe 的落标只在"候选格恰好是本路线上的普通航点"时生效，落不上只 warn。
    ⇒ 任何一处退化，自动规划就**静默**给出"单停站路线"，蓝面朝错方向那一局整场打不到靶。
    本函数是**引擎无关**的收口：拿一条已经规划好的 token 串，用与 `second_shoot_stop()`
    完全同一份判据算出对侧候选；候选确实在路线上时，把那个 token 标成 `BLOCK*`。

    ★ 为什么"零路径改动"是硬约束：候选格本来就是路线上的普通航点（现图 = `G3`，
    夹在 `F3` 与 `G4` 之间），标星只改"到站停不停"，**不动 token 个数/顺序/里程** ⇒
    不会引入新的穿洞/折返/贴墙风险（那三样的判据全在规划搜索里，这里一概不碰）。

    ★ 为什么可以直接 `Grid(spec)` 而不用引擎那套 `bridge_rows` 参数：
    `shoot_candidates()` 只用到 `Grid.passable()`，而它 = `in_bounds() and not blocked`
    （见 `Grid.passable`），`bridge_rows` 只在 `move_ok()` 的 K 列进出判据里使用 ⇒
    候选集与两个引擎各自构造的网格**逐格相同**。

    :param tokens: 已规划好的路线 token（`parse_mission` 能吃的那一套写法）
    :param map_cfg: 该次规划用的 `map.yaml` 原始 dict（也接受 `MapSpec`）
    :param cfg: `core.config.Config`（取板心窗口 `route.shoot_distance_m/tol_m`）
    :param source: 地图来源（只进错误/证据文案）
    :return: ``(tokens2, marked_block, note)`` —— `marked_block` 非空 = 本次真的补了星号；
        `note` 恒为一句可直接进日志的**原因/证据**（补标成功就写补了哪格、多远）
    """
    out = [str(t) for t in (tokens or ())]
    if not out:
        return out, "", "路线为空"
    parsed: list[dict] = []
    for token in out:
        try:
            parsed.append(parse_token(token))
        except ValueError as exc:           # 坏 token 不在这里判死（parse_mission 会给出正式报错）
            return out, "", f"token 无法解析（{token!r}）：{exc}"
    shoot_pos = [i for i, info in enumerate(parsed) if info["shoot"]]
    if not shoot_pos:
        return out, "", "路线没有任何射击停站（不补：补星号等于凭空加一个停车点）"
    if len(shoot_pos) >= 2:
        return out, "", ("已有 %d 个射击停站：%s"
                         % (len(shoot_pos), " ".join(parsed[i]["block"] for i in shoot_pos)))
    primary = parsed[shoot_pos[0]]["block"]
    spec = map_cfg if isinstance(map_cfg, MapSpec) else load_map(map_cfg, source=source)
    if not spec.targets:
        return out, "", "地图没有靶格（不需要射击停站）"
    second = second_shoot_stop(spec, Grid(spec), primary, _shoot_accept(cfg))
    if not second:
        return out, "", ("第二停站无解：板面另一侧没有能覆盖全部靶格 %s 的候选格"
                         % "/".join(spec.targets))
    idx = next((i for i, info in enumerate(parsed) if info["block"] == second), None)
    if idx is None:
        return out, "", f"对侧候选 {second} 不在本路线上（不擅自改路线 ⇒ 仍是单停站）"
    info = parsed[idx]
    token = ("R:" if info["reverse"] else "") + info["block"] + _SHOOT_SUFFIX
    if info["scan"]:
        token += _SCAN_CANON
    if info["pivot"]:
        token += _PIVOT_SUFFIX
    if info["heading"]:
        token += "," + str(info["heading"]).lower()
    out[idx] = token
    return out, second, ("第二射击停站 %s（板面另一侧，到板心 %.4f m）"
                         % (second, shoot_stop_distance(second, spec)))


# --------------------------------------------------------------------------- #
#  编译：A* 路径 -> Route
# --------------------------------------------------------------------------- #
@dataclass
class _Stop:
    """编译期内部结构：一个航点。

    ★ 不变量：``reverse[i] == True`` ⇔ **段 (i-1 → i) 是倒车段**。
      这与 `validate.py` / `route_check.py` 的读数口径完全一致
      （它们判的是"第 i 个航点是否由倒车到位"）。
    ``heading`` = **到达**该点时的车头；``depart`` = **从该点出发**时的车头
    （末点为 -1）。倒车段"车头不变"，所以 `reverse[i]` 时 `depart[i-1] == depart[i]`。
    """

    row: int
    col: int
    block: str
    heading: int = 0            # 到达时的车头朝向索引（0北/1西/2南/3东）
    depart: int = -1            # 出发时的车头朝向（末点 = -1）
    reverse: bool = False       # 段 (i-1 -> i) 倒车到位
    shoot: bool = False
    tag: str = ""               # start / end / mine / lower_bridge / upper_bridge / shoot
    #: `:PIVOT` 折返点声明（该顶点处车头需转过 ≥ `PIVOT_MARK_DEG`）。**纯声明。**
    pivot: bool = False


def _collapse(path: Sequence[tuple[int, int, int, int]],
              keep: set[tuple[int, int]]) -> list[tuple[int, int, int, bool]]:
    """逐步路径 -> ``[(row, col, face, back), ...]``。

    保留：起点、终点、`keep` 里的强制点（雷 / 桥洞 / 射击点）、**方向变化的拐点**、
    以及"步进模式变化"的拐点（前进↔倒车）。`face` 是车头**实际**指向
    （倒车时与行进方向相反），`back` 是"到达该点的那一步是否倒车"。

    ★ `keep` 必须包含全部雷点：`global_planner` 踩过的坑 —— 落在直线上的中间格会被
      压缩掉，规划器内部算的是全覆盖、输出的路线却漏一半，而且**不报任何错**。
    """
    if not path:
        return []
    moving = _path_reverse(path)

    steps: list[tuple[int, int, int, bool]] = []
    for i, (row, col, head, _) in enumerate(path):
        if i == 0:
            steps.append((row, col, head, False))
            continue
        prow, pcol = path[i - 1][0], path[i - 1][1]
        if (row, col) == (prow, pcol):
            if steps:
                steps[-1] = (steps[-1][0], steps[-1][1], head, steps[-1][3])
            continue
        direction = _heading_of(row - prow, col - pcol, head)
        face = (direction + 2) % 4 if moving[i] else direction
        steps.append((row, col, face, moving[i]))

    if len(steps) <= 2:
        return steps

    kept: list[tuple[int, int, int, bool]] = [steps[0]]
    for i in range(1, len(steps) - 1):
        prev, cur, nxt = steps[i - 1], steps[i], steps[i + 1]
        d1 = (cur[0] - prev[0], cur[1] - prev[1])
        d2 = (nxt[0] - cur[0], nxt[1] - cur[1])
        straight = (d1[0] * d2[1] - d1[1] * d2[0]) == 0 and (d1[0] * d2[0] + d1[1] * d2[1]) > 0
        same_mode = prev[3] == cur[3] == nxt[3]
        if (cur[0], cur[1]) in keep or not (straight and same_mode):
            kept.append(cur)
    kept.append(steps[-1])

    out: list[tuple[int, int, int, bool]] = []
    for i, (row, col, face, back) in enumerate(kept):
        if i + 1 < len(kept):
            nrow, ncol = kept[i + 1][0], kept[i + 1][1]
            direction = _heading_of(nrow - row, ncol - col, face)
            face = (direction + 2) % 4 if kept[i + 1][3] else direction
        out.append((row, col, face, back))
    return out


def _split_long_hops(stops: list[_Stop], grid: Grid,
                     max_blocks: int = MAX_HOP_BLOCKS) -> list[_Stop]:
    """在过长的相邻航点之间插入中间点（每格一个，只在直线段上）。

    ★ 为什么要插：`validate.py` 的 R011 在 "> 2.5 格" 处告警，而 1.4 m 以上的长直线
      在纯追踪下会切角（现场过桥洞尤其危险）。插点后段长恒 <= `max_blocks` 格，
      R011 一条都不出现，循迹也更贴格心。
    中间点**只从 A* 已经验证过可通行的格子里取**（沿直线插），所以不会踩障碍。
    """
    if len(stops) < 2:
        return stops
    out: list[_Stop] = [stops[0]]
    for cur in stops[1:]:
        prev = out[-1]
        drow, dcol = cur.row - prev.row, cur.col - prev.col
        steps = max(abs(drow), abs(dcol))
        straight = drow == 0 or dcol == 0
        if steps > max_blocks and straight:
            srow = 0 if drow == 0 else (1 if drow > 0 else -1)
            scol = 0 if dcol == 0 else (1 if dcol > 0 else -1)
            for k in range(1, steps):
                row, col = prev.row + srow * k, prev.col + scol * k
                block = geo.index_to_block(row - 1, col)
                if not grid.passable(row, col) or block in out[-1].block:
                    continue
                if out and (row, col) == (out[-1].row, out[-1].col):
                    continue
                out.append(_Stop(row, col, block, heading=cur.heading, reverse=cur.reverse))
            out[-1].reverse = cur.reverse
        out.append(cur)
    return out


def _fix_uturn_reverse(stops: list[_Stop]) -> int:
    """把"前进 → 前进"的 180° 折返改写成倒车段（与 `global_planner` 同一策略）。

    判定（避免误伤正常 90° 拐弯）：
      ① 折返角 > `UTURN_REVERSE_DEG`；② 两段都不是倒车段；③ 下一格可通行。
    改写 = 给**到达段**打 `reverse`（段 i → i+1 变成倒车到位）：车头保持不变、
    倒着退过去，零横向摆动。

    为什么必须改：原地 180° 掉头横向扫出约 0.54 m > 一格 0.35 m，在地图角落必出界；
    这也是现有手写路线里 `R:F11` / `R:B6` / `R:D6` 三段倒车的来历。
    """
    fixed = 0
    for i in range(1, len(stops) - 1):
        prev, cur, nxt = stops[i - 1], stops[i], stops[i + 1]
        if cur.reverse or nxt.reverse:
            continue            # 已经带倒车段（车头与行进方向不一致），不再改写
        if turn_angle_deg((prev.row, prev.col), (cur.row, cur.col),
                          (nxt.row, nxt.col)) <= UTURN_REVERSE_DEG:
            continue
        # ★ "车头朝西开进死胡同、再倒着退出来" = 进与出**两段都是倒车**：
        #   倒车段的车头保持不变，所以 `cur -> nxt` 的位移方向仍与车头一致时，
        #   必须把这**两段**都标成倒车 —— 只标一段就会让车头在中途凭空翻 180°。
        #   与现场手写路线的 `F10 F11 E11 R:F11 F10` 逐字同形。
        cur.reverse = True
        nxt.reverse = True
        fixed += 1
    return fixed


def _merge_duplicate_stops(stops: list[_Stop]) -> list[_Stop]:
    """合并相邻同格航点，**并保持 `reverse` 不变量**（两个同格点不可能同时倒车到位）。

    `_collapse` 偶尔会产出 `[F11(reverse=False), F11(reverse=True)]` 这种"同格两点"：
    前者是到达点、后者是出发段倒车的标记。保留两个会让 `RouteBuilder` 的去重把
    `reverse=True` 丢掉（它只保留前一个点），随后 `assert_legal` 就会报
    "E11 处 180° 折返" —— 这正是本轮踩过的坑。这里显式合并：同格时**保留反向标记为
    False 的那个到达语义**（`reverse[i]=True` 只表示"段 (i-1→i) 倒车"），
    射击/桥洞 tag 一并合并。
    """
    out: list[_Stop] = []
    for stop in stops:
        if out and (out[-1].row, out[-1].col) == (stop.row, stop.col):
            prev = out[-1]
            prev.reverse = prev.reverse and stop.reverse
            prev.shoot = prev.shoot or stop.shoot
            prev.tag = prev.tag or stop.tag
            continue
        out.append(stop)
    return out


def _barrier_turn_stops(grid: Grid, stops: Sequence["_Stop"]) -> list[str]:
    """★★ 在**编译后的逐点路线**里找出"隔板相邻行死胡同式 90° 转向"的顶点。

    与 `_Tour.penalty_of()` 的同类判据相比，这一版**不依赖 `head_cost` 里的标称腿**，
    而是直接看编译出来的航点序列：对每个内部点，取"上一点的来向"与"下一点的去向"，
    喂给同一个 `_barrier_turn_cell()`。两条路径共用同一条判据（一个函数），
    所以"搜索时避开了、编译后又冒出来"这种不一致不可能悄悄发生。

    :return: 命中的格名列表（顺序即路线顺序；空 = 干净）
    """
    out: list[str] = []
    cells = [(stop.row, stop.col) for stop in stops]
    for i in range(1, len(cells) - 1):
        d_in = _heading_of(cells[i][0] - cells[i - 1][0], cells[i][1] - cells[i - 1][1], -1)
        d_out = _heading_of(cells[i + 1][0] - cells[i][0], cells[i + 1][1] - cells[i][1], -1)
        if d_in < 0 or d_out < 0:
            continue
        if _barrier_turn_cell(grid, cells[i][0], cells[i][1], d_in, d_out):
            out.append(geo.index_to_block(cells[i][0] - 1, cells[i][1]))
    return out


def mark_pivots(stops: list[_Stop], *,
                min_deg: float = PIVOT_MARK_DEG) -> list[tuple[int, str, float]]:
    """给折返顶点打上 `:PIVOT` 声明，返回 ``[(索引, 格名, 角度°), ...]``。

    判据与 `pivot_vertices()` **逐字相同**（只是作用在编译期的 `_Stop` 上）：
    折返角 ≥ `min_deg` 且相邻两段都不是倒车段 ⇒ `stop.pivot = True`。

    ★ 调用时机：必须在 `_merge_duplicate_stops` **之后**（否则索引会变），
      且只在**无倒车模式**下调用 —— 允许倒车模式下折返由 `R:` 完成，
      不应出现 `:PIVOT` 声明（这也是 `--allow-reverse` 与历史基线逐字节一致的结构性保证）。
    ★ `:PIVOT` 只是声明：不打车、不停车、不改 `pursuit/**` 与 `tracker.py` 的行为。
    """
    marked: list[tuple[int, str, float]] = []
    for i in range(1, len(stops) - 1):
        prev, cur, nxt = stops[i - 1], stops[i], stops[i + 1]
        if cur.reverse or nxt.reverse:
            continue
        angle = turn_angle_deg((prev.row, prev.col), (cur.row, cur.col),
                               (nxt.row, nxt.col))
        if angle >= min_deg - 1e-9:
            cur.pivot = True
            marked.append((i, cur.block, angle))
    return marked


def _refresh_headings(stops: list[_Stop]) -> None:
    """重算每个航点记的**车头实际朝向**（= 到达该点时车头指向哪边）。

    唯一一条物理规则：**倒车段车头不变**（车头背着目标、倒着退过去）。于是

    * 前进到达 p → p 的车头 = 行进方向；
    * 倒车到达 p → p 的车头 = 行进方向的**反方向**（车头背着 p，倒着退进来了）。

    这条规则对"前进段起点"同时也是"下一段出发方向"（车确实朝那边开），
    对"倒车段起点"则给出背离目标的车头 —— 与现场手写路线的 `F10 F11 E11 R:F11 F10`
    逐字同形（`R:F11` 那段就是"车头朝西、倒着往东退"）。

    ★ 调用时机：必须在 `_fix_uturn_reverse` **之后**。折返改写会翻转某一段的 `reverse`
      （车头随之翻 180°），先算后改就会把终点姿态与云台基准朝向写错。

    注意：`RouteBuilder` 给中间点自动补的 yaw 是"离开段方向"，在倒车段上会与车头差
    180°（现场手写路线也一样）；`tracker` 判断倒车用的是
    `path.segment_is_reverse()` 而不是 yaw，所以 yaw 只对**终点姿态**有意义 ——
    这正是本函数要保证 `_final_heading` 正确的原因。
    """
    for i in range(1, len(stops)):
        prev, cur = stops[i - 1], stops[i]
        direction = _heading_of(cur.row - prev.row, cur.col - prev.col, cur.heading)
        cur.heading = (direction + 2) % 4 if cur.reverse else direction
    # 出发车头：前进 = 行进方向；倒车 = 行进方向的反方向（车头背着目标退过去）。
    #   —— 倒车段"车头不变"，所以 `reverse[i+1]` 时 `depart[i]` 必须等于 `heading[i]`。
    for i in range(len(stops) - 1):
        cur, nxt = stops[i], stops[i + 1]
        direction = _heading_of(nxt.row - cur.row, nxt.col - cur.col, cur.heading)
        cur.depart = (direction + 2) % 4 if nxt.reverse else direction
    stops[-1].depart = -1


def _final_heading(stops: Sequence[_Stop]) -> int | None:
    """末点航向（供 `RouteBuilder.fix_final_yaw`）= 到达末点时的车头（已含倒车翻转）。"""
    return stops[-1].heading if stops else None


def _cell(block: str) -> tuple[int, int]:
    row_idx, col_idx = geo.block_to_index(block)
    return row_idx + 1, col_idx


def _to_route(stops: Sequence[_Stop], *, name: str, crossing: dict,
              final_heading: int | None = None) -> Route:
    """`_Stop` 序列 -> `Route`（经 `RouteBuilder` 补朝向）。

    ★ `drop_consecutive_duplicates=False`：相邻同格点必须走 `_merge_duplicate_stops`
      （它知道 `reverse` 的语义），`RouteBuilder` 的内置去重会把 `reverse=True` 丢掉。
    """
    builder = RouteBuilder(name)
    for stop in stops:
        x, y = geo.block_to_meters(stop.block)
        yaw = geo.HEADING_Y if stop.heading in (1, 3) else geo.HEADING_X
        token = (("R:" if stop.reverse else "") + stop.block
                 + ("*" if stop.shoot else "")
                 + (_PIVOT_SUFFIX if stop.pivot else ""))
        builder.add(Waypoint(block=stop.block, x=x, y=y, yaw=yaw,
                             reverse=stop.reverse, shoot=stop.shoot,
                             pivot=stop.pivot, token=token))
    fix_final = None
    if final_heading is not None:
        fix_final = geo.HEADING_Y if final_heading in (1, 3) else geo.HEADING_X
    route = builder.build(drop_consecutive_duplicates=False, fix_final_yaw=fix_final)
    return Route(route.waypoints, name=name, crossing=dict(crossing))


def route_tokens(route: Route, *, final_heading: bool = True) -> list[str]:
    """`Route` -> token 列表（下发 `/nav/waypoint` / 回写 `robot.yaml`）。

    后缀顺序与 `parser.normalize_token` 的约定一致：
    `R:` 前缀 → 格名 → `*` → `:PIVOT` → `,x|,y`。
    """
    tokens: list[str] = []
    last = len(route.waypoints) - 1
    for idx, wp in enumerate(route.waypoints):
        token = ("R:" if wp.reverse else "") + wp.block + ("*" if wp.shoot else "")
        if wp.pivot:
            token += _PIVOT_SUFFIX
        if final_heading and idx == last:
            token += ",y" if abs(wp.yaw - geo.HEADING_Y) < 1e-6 else ",x"
        tokens.append(token)
    return tokens


# --------------------------------------------------------------------------- #
#  自检
# --------------------------------------------------------------------------- #
def check_route_structure(route: Route, spec: MapSpec, grid: Grid,
                          limits: tuple[float, float],
                          acc: tuple[float, float] | None = None, *,
                          expected_start: str | None = None,
                          already_covered: Sequence[str] = (),
                          crossing_halves: tuple[bool, bool] = (True, True),
                          ) -> list[str]:
    """规划器自检（返回问题列表，空 = 通过）。**只报不改。**

    这些是"规划器必须自己保证"的结构约束；完整校验仍以 `validate.validate_route`
    与 `tools/route_check.py` 为准（见 `tools/plan_route.py --check`）。

    `acc` = 射击停站的 `(目标板心距离, 半宽)`（缺省用模块常量）。

    ★ 增量续算（**全部关键字参数，缺省 = 与旧行为逐位一致**）：
        · `expected_start`  —— 本轮起点（缺省 = `spec.start`）。增量续算时是 `from_cell`，
          车并不在地图起点，故不能拿 `spec.start` 判"起点错位"。
        · `already_covered` —— 已压格（只对**雷**生效）：已在其中 ⇒ 不再计入"雷漏压"。
        · `crossing_halves` —— `(下半场, 上半场)` 是否**还要求**穿洞。`False` = 该半场穿越
          在本次续算之前已完成（允许 0 次；若路线里仍出现，次数/方向照样逐条查）。
    """
    problems: list[str] = []
    blocks = route.blocks()
    if not blocks:
        return ["路线为空"]

    start_name = (expected_start or spec.start).strip().upper()
    if blocks[0] != start_name:
        problems.append(f"起点不是 {start_name}（实际 {blocks[0]}）")
    if blocks[-1] != spec.end:
        problems.append(f"终点不是 map.end={spec.end}（实际 {blocks[-1]}）")

    covered = {str(c).strip().upper() for c in already_covered}
    mine_set, obs_set = set(spec.mines), set(spec.obstacles)
    miss = sorted(mine_set - set(blocks) - covered)
    if miss:
        problems.append(f"雷漏压 {' '.join(miss)}")
    on_obs = sorted(obs_set & set(blocks))
    if on_obs:
        problems.append(f"路线踩到障碍格 {' '.join(on_obs)}")
    u_cols = {geo.COL_LETTERS.index(c) for c in V.U_TURN_COLUMNS}
    if not any(geo.block_to_index(b)[1] in u_cols for b in blocks):
        problems.append(f"路线完全没有经过 {'/'.join(V.U_TURN_COLUMNS)} 列（没绕过隔板）")

    # ★ 一次过洞由**两段**组成：进洞那一半（`L→K`，起点不在 K 列）+ 出洞那一半（`K→J`，
    #   起点就在 K 列上）—— `validate._is_cross` 把**两半都算**成"朝 K 列推进/穿过它"。
    #   所以这里必须用 `validate.crossing_entries()`（**一次过洞 = 一条记录**，与 R007 的
    #   run 计数等价）来数次数；直接用 `_crossing_at_column` 会把现场路线
    #   `L7->K7` + `K7->J7` 数成 2 次 ⇒ 自检误红（2026-10-11 实测）。
    runs: list[tuple[str, str, tuple]] = list(V.crossing_entries(blocks))   # noqa: SLF001
    lower = [r for r in runs if V.LOWER_HALF_ROWS[0] <= round(r[2][0]) <= V.LOWER_HALF_ROWS[1]]
    upper = [r for r in runs if V.UPPER_HALF_ROWS[0] <= round(r[2][0]) <= V.UPPER_HALF_ROWS[1]]
    # `crossing_halves=False` 的半场 = 穿越已在本轮之前完成 ⇒ 允许 0 次；
    #   一旦路线里仍出现，下面的次数/方向判据照旧生效（不会因为开关而放过斜切/反向）。
    if crossing_halves[0]:
        if len(lower) != 1:
            problems.append(f"下半场应恰好穿洞 1 次，实际 {len(lower)} 次")
        elif sum(r[2][3] for r in lower) >= 0:
            problems.append("下半场穿洞方向不是**向西**（R007：L* -> K* -> J*）")
    elif len(lower) > 1:
        problems.append(f"下半场已过洞，本段却仍穿 {len(lower)} 次（R007 会判 2 次）")
    if crossing_halves[1]:
        if len(upper) != 1:
            problems.append(f"上半场应恰好穿洞 1 次，实际 {len(upper)} 次")
        elif sum(r[2][3] for r in upper) <= 0:
            problems.append("上半场穿洞方向不是**向东**（R007：J* -> K* -> L*）")
    elif len(upper) > 1:
        problems.append(f"上半场已过洞，本段却仍穿 {len(upper)} 次（R007 会判 2 次）")
    for a, b, hit in runs:
        if hit[1] > V.MAX_CROSS_ANGLE_DEG + 1e-9:
            problems.append(f"{a}->{b} 斜切入洞 {hit[1]:.1f}°"
                            f"（R009 上限 {V.MAX_CROSS_ANGLE_DEG:.0f}°）")

    for a, b in zip(blocks, blocks[1:]):
        hit = V._barrier_crossing(a, b)                 # noqa: SLF001
        if hit and hit[0]:
            name = geo.COL_LETTERS[min(geo.MAP_COLS - 1, max(0, int(round(hit[1]))))]
            problems.append(f"{a}->{b} 非法跨越第 {grid.barrier_hi}/{grid.barrier_lo} 行隔板"
                            f"（交点列 {name}，只有 {'/'.join(V.U_TURN_COLUMNS)} 列能过）")

    stops = route.shoot_points()
    if spec.targets:
        # 射击停站从「恰好 1 个」放宽到「1~2 个」，且 2 个时必须**分居板面两侧**。
        #   为什么放宽：靶板一面蓝一面红、蓝面朝向每局随机但恒定，只打蓝 ⇒ 单侧停站有
        #   50% 概率整场看不到合法面（现场实测就是这个坑）。两个停站 = 至少一侧必见蓝面。
        #   为什么不能更多：每多一个停站就多一次停站（30s 硬死线）+ 一段绕行，收益边际递减，
        #   且"两侧"的物理目的 2 个就够；再多说明路线上开始乱标 `*`。
        if not 1 <= len(stops) <= 2:
            problems.append(f"射击停站应为 1~2 个，实际 {len(stops)}")
        if len(stops) == 2:
            _sides = {plate_side_of(spec, wp.block) for _i, wp in stops}
            if 0 in _sides or len(_sides) != 2:
                problems.append(
                    "2 个射击停站必须分居靶板两侧（实际同一侧/含板心平面: "
                    + " ".join(wp.block for _i, wp in stops) + "）")
        for _, wp in stops:
            # 窗口改为**靶板中心窗口**（默认 [0.70, 0.90] m），与
            #   `shoot_candidates()`/`covering_stops()` 同源（真源 =
            #   `route.shoot_distance_m` ± `route.shoot_distance_tol_m`）。
            #   判据仍是"**每个**靶格都要被这个停站覆盖"——但覆盖区间换成板心窗口：
            #   实测 `F3` 到 `E5` 0.7826 / 到 `F5` 0.7000，两者都落在 [0.70, 0.90]
            #   （旧窗口 0.17~0.51 会把**正确落点**判死）。
            lo, hi = shoot_distance_window(acc)
            for tgt in spec.targets:
                if not V._stop_ok(wp, tgt, lo, hi):                 # noqa: SLF001
                    problems.append(
                        f"射击停站 {wp.block} 到靶 {tgt} 的距离 "
                        f"{V._euc(wp, tgt):.3f} m 不在窗口 "
                        f"{lo:.2f}~{hi:.2f} m（route_check 会判覆盖不足）")
    elif stops:
        problems.append("地图没有 targets，路线里却有射击停站")

    for a, b in zip(blocks, blocks[1:]):
        ax, ay = geo.block_to_meters(a)
        bx, by = geo.block_to_meters(b)
        length = math.hypot(bx - ax, by - ay)
        if length < V.MIN_SEGMENT_M:
            problems.append(f"段 {a}->{b} 长度 {length:.4f} m < {V.MIN_SEGMENT_M} m（重复点？）")
        elif length / geo.BLOCK_SIZE > V.MAX_SEGMENT_BLOCKS:
            problems.append(f"段 {a}->{b} 长 {length / geo.BLOCK_SIZE:.2f} 格"
                            f" > {V.MAX_SEGMENT_BLOCKS} 格（R011）")

    reverse_m = total_m = 0.0
    for i, (a, b) in enumerate(zip(blocks, blocks[1:])):
        ax, ay = geo.block_to_meters(a)
        bx, by = geo.block_to_meters(b)
        seg = math.hypot(bx - ax, by - ay)
        total_m += seg
        if route.waypoints[i + 1].reverse:
            reverse_m += seg
    if total_m > 1e-9 and reverse_m / total_m > V.MAX_REVERSE_RATIO:
        problems.append(f"倒车占比 {reverse_m / total_m * 100:.1f}%"
                        f" > R014 的 {V.MAX_REVERSE_RATIO * 100:.0f}%（现场硬约束）")
    return problems


def assert_legal(route: Route) -> None:
    """硬性合法性断言：token 可解析 + 无相邻重复点 + **同模式 180° 折返为 0**。

    最后一条是本规划器最关键的不变量：180° 折返必须以 `R:`（倒车）出现，
    否则实车只能靠原地转向掉头，横向扫出 0.54 m > 一格，直接出界。
    """
    for idx, wp in enumerate(route.waypoints):
        token = ("R:" if wp.reverse else "") + wp.block + ("*" if wp.shoot else "")
        info = parse_token(token)
        if info["block"] != wp.block or info["reverse"] != wp.reverse:
            raise PlannerError(f"第 {idx} 个航点 {token!r} 与模型不一致")
    blocks = route.blocks()
    for a, b in zip(blocks, blocks[1:]):
        if a == b:
            raise PlannerError(f"相邻重复航点 {a}")

    for i in range(1, len(route.waypoints) - 1):
        prev, cur, nxt = route.waypoints[i - 1], route.waypoints[i], route.waypoints[i + 1]
        # 只要相邻两段里有倒车段，就不算"原地掉头"（车头没掉过来）——与
        # route_check 的 _is_rev_vertex 同一判据。
        # ★ 判据必须落在**与本顶点相邻的两段**上：`reverse[i]` 表示"段 (i-1→i) 倒车"，
        #   所以顶点 i 的两条邻段是 cur.reverse（进）与 nxt.reverse（出）。
        #   旧写法 `prev.reverse or cur.reverse` 看的是顶点 i-1/i 的段：
        #   ① 漏判"正向进 + 倒车出"——现场 41 token 第 7 顶点 H10→H8→R:H9 被误报 180° 折返，
        #      导致规划器自己产出的合法倒车路线过不了 assert_legal；
        #   ② 错杀"上一段倒车"的正常顶点。与 pivot_vertices 的判据逐字对齐。
        if cur.reverse or nxt.reverse:
            continue
        angle = _angle_of(prev, cur, nxt)
        if angle > UTURN_REVERSE_DEG:
            raise PlannerError(f"第 {i} 个航点 {cur.block} 处存在同模式 {angle:.0f}° 折返"
                               f"（未改写为倒车段）")


def _angle_of(a: Waypoint, b: Waypoint, c: Waypoint) -> float:
    d1, d2 = (b.x - a.x, b.y - a.y), (c.x - b.x, c.y - b.y)
    n1, n2 = math.hypot(*d1), math.hypot(*d2)
    if n1 < 1e-9 or n2 < 1e-9:
        return 0.0
    cosv = max(-1.0, min(1.0, (d1[0] * d2[0] + d1[1] * d2[1]) / (n1 * n2)))
    return math.degrees(math.acos(cosv))


def pivot_vertices(route: Route, *, min_deg: float = PIVOT_MARK_DEG
                   ) -> list[tuple[int, str, float]]:
    """折返顶点清单 ``[(索引, 格名, 折返角°), ...]``（升序）。

    判据与循迹层**逐条对齐**（这不是另发明一套）：
      * 角度 = `pursuit.path.corner_turn_deg` = 顶点处**车头**要转过的角度；
        无倒车模式下与 `corner_angle_deg`（行进方向夹角）同值；
      * 阈值 = `pursuit.pivot_angle_deg`（`PIVOT_MARK_DEG` = 135°，只做声明）；
      * 相邻两段中**任一段是倒车段**就不算折返（`route_check._is_rev_vertex` 同一判据）：
        倒车段的车头一度都不用转，那里不会触发 PIVOT。
    """
    out: list[tuple[int, str, float]] = []
    wps = route.waypoints
    for i in range(1, len(wps) - 1):
        prev, cur, nxt = wps[i - 1], wps[i], wps[i + 1]
        if cur.reverse or nxt.reverse:              # 相邻段有倒车 → 车头不用掉头
            continue
        angle = _angle_of(prev, cur, nxt)
        if angle >= min_deg - 1e-9:
            out.append((i, cur.block, angle))
    return out


def assert_legal_no_reverse(route: Route) -> None:
    """无倒车模式的硬性合法性断言（`plan_route(allow_reverse=False)` 内必调）。

    四条不变量：
      ① 每个航点的 token 可解析，且 `block` / `reverse` / `pivot` 与模型一致；
      ② 相邻航点不重复；
      ③ **0 段 `R:`** —— 真车底盘不支持倒车（负速度方向位被 STM32 无视，车以 |v|
         朝车头方向前进），出现一段 `R:` 就整条路线报废（旧行为：不允许）；
      ④ **折返点声明完整、且不虚报**：每个折返角 ≥ `PIVOT_MARK_DEG`（= 循迹层
         `pursuit.pivot_angle_deg`，判据同 `pursuit.path.corner_turn_deg`）的顶点
         都必须带 `:PIVOT`；反之带 `:PIVOT` 的顶点必须真的是折返点。

    第 ④ 条就是"合法性自检"的负向控制入口：把某处 `:PIVOT` 去掉即抛 `PlannerError`。
    `:PIVOT` 本身**不改变**循迹层行为（`pursuit/**` / `tasks/tracker.py` 不读该字段），
    这里断言的是"声明与实际几何必须一致"，不是"照着声明去执行"。
    """
    for idx, wp in enumerate(route.waypoints):
        token = ("R:" if wp.reverse else "") + wp.block + ("*" if wp.shoot else "")
        if wp.pivot:
            token += _PIVOT_SUFFIX
        info = parse_token(token)
        if (info["block"] != wp.block or info["reverse"] != wp.reverse
                or info["pivot"] != wp.pivot):
            raise PlannerError(f"第 {idx} 个航点 {token!r} 与模型不一致")
        if wp.reverse:
            raise PlannerError(
                f"无倒车模式却出现倒车段：第 {idx} 个航点 {token!r}"
                f"（真车底盘不支持倒车，负速度方向位被 STM32 无视）")
    blocks = route.blocks()
    for a, b in zip(blocks, blocks[1:]):
        if a == b:
            raise PlannerError(f"相邻重复航点 {a}")

    # ---- ④ 折返点声明的一致性（完整性 + 不虚报）----
    need = {i: (block, deg) for i, block, deg in pivot_vertices(route)}
    declared = {i: wp.block for i, wp in enumerate(route.waypoints) if wp.pivot}
    missing = sorted(set(need) - set(declared))
    if missing:
        detail = "; ".join(f"#{i} {need[i][0]} {need[i][1]:.0f}°" for i in missing)
        raise PlannerError(
            f"折返点缺少 {_PIVOT_SUFFIX} 声明：{detail}"
            f"（折返角 ≥ {PIVOT_MARK_DEG:.0f}° 的顶点必须显式声明 —— "
            f"该处车头要转 {need[missing[0]][1]:.0f}°，无倒车模式下车只能原地掉头）")
    extra = sorted(set(declared) - set(need))
    if extra:
        detail = "; ".join(f"#{i} {declared[i]}" for i in extra)
        raise PlannerError(
            f"虚报 {_PIVOT_SUFFIX}：{detail}（这些顶点的折返角 "
            f"< {PIVOT_MARK_DEG:.0f}°，或相邻段是倒车段 —— 车头一度都不用转）")


# --------------------------------------------------------------------------- #
#  规划结果
# --------------------------------------------------------------------------- #
@dataclass
class PlanResult:
    """规划结果（含全部证据数值，便于直接贴日志 / 出 JSON）。"""

    route: Route
    tokens: list[str]
    stops: tuple["_Stop", ...] = ()
    crossing: dict = field(default_factory=dict)
    shoot_stop: str = ""
    shoot_distance_m: float = 0.0
    shoot_targets: tuple[str, ...] = ()
    #: 第二射击停站（与 `shoot_stop` **分居板面两侧**；无解 = `""`）。
    #: 两侧各停一次是"蓝面朝向每局随机"的物理对策：至少一侧必见蓝面、必能打。
    shoot_stop2: str = ""
    shoot_distance_m2: float = 0.0
    #: ★t7 本次**事后补标**的第二射击停站（非空 = 引擎本身没落标，由
    #:  `ensure_second_shoot_stop()` 在规划后补上的；空 = 引擎自己已落标或压根不需要）
    second_stop_added: str = ""
    cost: float = 0.0
    expanded: int = 0
    elapsed_ms: float = 0.0
    reverse_segments: int = 0
    uturn_fixes: int = 0
    warnings: tuple[str, ...] = ()
    map_name: str = ""
    map_source: str = ""
    start: str = ""
    end: str = ""
    polyline_m: float = 0.0
    start_heading: str = ""
    #: 本次规划是否**允许倒车**（`allow_reverse`；False = 车上默认的无倒车模式）
    allow_reverse: bool = True
    #: ★ 折返点清单 ``[(航点索引, 格名, 折返角°), ...]`` —— 与 `:PIVOT` 声明一一对应
    pivots: tuple[tuple[int, str, float], ...] = ()
    #: 折返惩罚（等效代价单位/处）；0.0 = 未启用
    pivot_penalty: float = 0.0
    #: "雷格不得作拐点"的惩罚权重（等效代价单位）；0.0 = 未启用
    mine_turn_penalty: float = 0.0
    #: ★★ 隔板相邻行"死胡同式 90° 转向"的惩罚权重；0.0 = 未启用
    barrier_turn_penalty: float = 0.0
    #: ★★ 编译后路线里**实际残留**的隔板死胡同转向格名（启用惩罚时应为空；
    #: 非空即抛 `PlannerError`，这里只是留证据字段）
    barrier_turns: tuple[str, ...] = ()
    #: ★ 逐雷穿越轴 ``[(雷格名, 轴编号0北1西2南3东, 是否找到可行轴), …]`` ——
    #: `ok=False` 表示该雷找不到直线穿越轴（会被切角擦过，需人工核对地图）
    mine_axes: tuple[tuple[str, int, bool], ...] = ()
    #: ★【自诊断】逐雷"进点前共线格数" ``[(雷格名, 格数), …]`` —— 见 `_mine_aligned_runs`。
    #: `<2` 的雷在纯追踪下会被切角（车心压不到雷心），plan_route 会另附一条 warning。
    mine_runs: tuple[tuple[str, int], ...] = ()
    #: 顺序代价（含折返惩罚）—— 供"加惩罚前 vs 后"对比用
    tour_cost: float = 0.0
    #: ★ 任务向量（TRACK-PLAN-B1 新口径）：本路线**实际覆盖**的雷格名（按路线序）。
    tasks_mines: tuple[str, ...] = ()
    #: 本路线**实际落标**的射击停站格名（= token 里的 `*`，按路线序）。
    tasks_stops: tuple[str, ...] = ()
    #: 地图雷总数（分母；与 `tasks_mines` 一起构成 `雷 n/N`）。
    tasks_mines_total: int = 0
    #: True = 本条路线是**降级解**（任务没全做完 / 结构约束不满足）。
    degraded: bool = False
    #: 降级原因（`缺雷 …` / `缺停站` / `超时` / `结构`；空 = 未降级）。
    degraded_reason: str = ""

    @property
    def pivot_count(self) -> int:
        return len(self.pivots)

    def shoot_blocks(self) -> list[str]:
        """全部**射击**停站格名（按路线序）—— 双停站时是 `[主停站, 对侧停站]`。

        口径唯一来自 `Route.shoot_points()`（= token 里的 `*`），不另记一份，
        避免"PlanResult 说有 2 个停站、真路线只有 1 个"这种两套账。
        """
        return [wp.block for _i, wp in self.route.shoot_points()]

    def as_dict(self) -> dict:
        return {
            "map": self.map_name, "map_file": self.map_source,
            "start": self.start, "end": self.end,
            "start_heading": self.start_heading,
            "tokens": list(self.tokens), "token_count": len(self.tokens),
            "crossing": dict(self.crossing),
            "shoot_stop": self.shoot_stop,
            "shoot_distance_m": round(self.shoot_distance_m, 4),
            # ★t7：双停站必须能一眼看见（同侧 + 对侧各一个；JSON 里不再只有主停站）
            "shoot_stops": self.shoot_blocks(),
            "shoot_stop2": self.shoot_stop2,
            "shoot_distance_m2": round(self.shoot_distance_m2, 4),
            "second_stop_added": self.second_stop_added,
            "shoot_targets": list(self.shoot_targets),
            "cost": round(self.cost, 4),
            "polylength_m": round(self.polyline_m, 4),
            "reverse_segments": self.reverse_segments,
            "uturn_fixes": self.uturn_fixes,
            "expanded": self.expanded,
            "elapsed_ms": round(self.elapsed_ms, 3),
            "warnings": list(self.warnings),
            "allow_reverse": self.allow_reverse,
            "pivot_penalty": round(self.pivot_penalty, 4),
            "mine_turn_penalty": round(self.mine_turn_penalty, 4),
            "barrier_turn_penalty": round(self.barrier_turn_penalty, 4),
            "barrier_turns": list(self.barrier_turns),
            "mine_axes": [{"block": b, "axis": a, "ok": o} for b, a, o in self.mine_axes],
            "mine_runs": [{"block": b, "run": r} for b, r in self.mine_runs],
            "tour_cost": round(self.tour_cost, 4),
            # ★ 任务优先口径的显式读数（acceptance A 逐图要求 planned/degraded/reason/tasks）
            "tasks_mines": list(self.tasks_mines),
            "tasks_stops": list(self.tasks_stops),
            "tasks": {
                "mines_covered": len(self.tasks_mines),
                "mines_total": int(self.tasks_mines_total or len(self.tasks_mines)),
                "stops": len(self.tasks_stops),
                "full": (not self.degraded),
            },
            "degraded": bool(self.degraded),
            "degraded_reason": self.degraded_reason,
            "pivot_count": self.pivot_count,
            "pivots": [{"index": i, "block": b, "deg": round(d, 1)}
                       for i, b, d in self.pivots],
        }

    def summary(self) -> str:
        shoot = (f"射击停站 {self.shoot_stop} 到靶 {self.shoot_distance_m:.3f} m"
                 f"（{'/'.join(self.shoot_targets)}）" if self.shoot_stop else "无射击停站")
        if self.shoot_stop2:
            shoot += f" + 对侧停站 {self.shoot_stop2}（板心 {self.shoot_distance_m2:.3f} m）"
        pivots = (" ".join(f"{b} {d:.0f}°" for _, b, d in self.pivots) or "无")
        lines = [
            f"路线[{self.route.name}] {len(self.tokens)} token / {self.polyline_m:.2f} m"
            f" | 穿洞 下={self.crossing.get('lower', '?')} 上={self.crossing.get('upper', '?')}"
            f" | {shoot}",
            f"  A* 代价 {self.cost:.2f} / 展开 {self.expanded} 次 / 耗时 {self.elapsed_ms:.1f} ms"
            f" | 倒车段 {self.reverse_segments}（折返改写 {self.uturn_fixes}）"
            f" | 无倒车模式 {'是' if not self.allow_reverse else '否'}",
            f"  折返点 {self.pivot_count} 处（{_PIVOT_SUFFIX} 声明）：{pivots}"
            f" | 顺序代价 {self.tour_cost:.2f}（折返惩罚 {self.pivot_penalty:.1f}/处）",
            "  " + " ".join(self.tokens),
        ]
        if self.degraded:
            lines.append(f"  🔶 降级：{self.degraded_reason or '未命名'} "
                         f"（任务 雷 {len(self.tasks_mines)}/{self.tasks_mines_total}、"
                         f"射击停站 {len(self.tasks_stops)}）")
        lines.extend(f"  ⚠️ {w}" for w in self.warnings)
        return "\n".join(lines)


# --------------------------------------------------------------------------- #
#  顺序 + 朝向联合优化
# --------------------------------------------------------------------------- #
class _Tour:
    """顺序与朝向的联合优化器 + 段路径缓存。

    数据：
      ``order_cost[(a, b)]``（顺序轮用；四个朝向对里的最小值，是真实代价的**下界**）
      ``head_cost[(a, b)][ha][hb]``（朝向轮用；带朝向的精确代价）

    ★ 代价全部来自"每个点一次 Dijkstra 洪泛"（见 `flood`），不是逐对 A* ——
      后者在 19 个点上要 2700+ 次搜索，会直接顶爆 `DEFAULT_MAX_SECONDS`。
    """

    def __init__(self, grid: Grid, deadline: float,
                 pivot_penalty: float = 0.0, record_dirs: bool = True,
                 mine_cells: Iterable[tuple[int, int]] = (),
                 mine_turn_penalty: float = 0.0,
                 barrier_turn_penalty: float = 0.0,
                 door_turn_penalty: float = 0.0,
                 edge_mine_turn_penalty: float = 0.0,
                 edge_mine_cells: Iterable[tuple[int, int]] = (),
                 obstacle_turn_penalty: float = 0.0,
                 obstacle_cells: Iterable[tuple[int, int]] = (),
                 phys: "dict | None" = None) -> None:
        self.grid = grid
        self.deadline = deadline
        #: ★ 折返惩罚（等效代价单位/处）。0.0 = 关闭（允许倒车模式的默认值，
        #: 也是"加惩罚前 vs 后"对比实验的对照组）。
        self.pivot_penalty = max(0.0, float(pivot_penalty))
        #: ★★ 隔板相邻行"死胡同式 90° 转向"的软代价。0.0（默认）
        #: = 关闭（= `route.tokens`/历史逐 token 一致）；> 0 = 规划器主动避开这种转向
        #: （见 `_barrier_turn_cell`）。它同时进**洪泛代价**与**顺序惩罚** ——
        #: 前者决定"段路径往哪绕"，后者决定"访问顺序怎么排"，
        #: 两个都得看得到它，否则会出现"顺序层以为占了便宜、段路径层再偷偷加价"。
        self.barrier_turn_penalty = max(0.0, float(barrier_turn_penalty))
        #: ★ 约束①：过门转弯点离隔板东端余量不足的软代价（0.0 = 关闭）。
        self.door_turn_penalty = max(0.0, float(door_turn_penalty))
        #: ★ 约束②：最外行/列雷附近转弯的软代价（0.0 = 关闭）。
        self.edge_mine_turn_penalty = max(0.0, float(edge_mine_turn_penalty))
        #: 约束② 的作用对象（最外行/列的雷格，1-based 行）
        self.edge_mine_cells: set[tuple[int, int]] = set(edge_mine_cells)
        #: 圆柱障碍在**转弯点**的软代价（0.0 = 关闭）与作用格集合
        self.obstacle_turn_penalty = max(0.0, float(obstacle_turn_penalty))
        self.obstacle_cells: set[tuple[int, int]] = set(obstacle_cells)
        #: 物理几何参数（`config` 的 vehicle/field/planner.* 键；见 `_phys_params`）
        self.phys: dict = dict(phys) if phys else _phys_params(None)
        #: 是否记录"每对访问点最短腿的起步/末步朝向"——折返惩罚与
        #: `straighten_mines()`（雷心直线穿越）都要用它，故默认开启。
        self.record_dirs = bool(record_dirs)
        #: ★ 雷格集合 + **雷格不得作为拐点**的惩罚权重：顺序里若某个雷格处有转角
        #: （90° 记 1 份、180° 记 8 份），按该权重计价。理由同 `MINE_PRESS_TOL_M`：
        #: 雷格一旦是拐点，纯追踪就会切角/掉头，车心压不到雷心 —— 这一项让
        #: "顺序 ↔ 朝向"优化主动把雷格排到**直线上**，而不是事后补救。
        self.mine_cells = set(mine_cells)
        self.mine_turn_penalty = max(0.0, float(mine_turn_penalty))
        self._floods: dict[tuple[int, int], tuple[dict, dict, dict, int]] = {}
        self.settled = 0
        self.head_cost: dict[tuple[tuple[int, int], tuple[int, int]],
                             list[list[float]]] = {}
        self.order_cost: dict[tuple[tuple[int, int], tuple[int, int]], float] = {}
        #: 每对访问点之间"最短腿"的**起步朝向**与**末步朝向**（用于判折返）
        self._exit_dir: dict[tuple[tuple[int, int], tuple[int, int]], int] = {}
        self._entry_dir: dict[tuple[tuple[int, int], tuple[int, int]], int] = {}

    # ---------------- 洪泛与查询 ----------------
    def _flood_of(self, cell: tuple[int, int]) -> tuple[dict, dict, dict, int, dict]:
        hit = self._floods.get(cell)
        if hit is None:
            if time.perf_counter() > self.deadline:
                raise PlannerError("规划超时（代价洪泛阶段）")
            hit = flood(self.grid, (cell[0], cell[1], 0),
                        barrier_turn_penalty=self.barrier_turn_penalty)
            self._floods[cell] = hit
            self.settled += hit[3]
        return hit

    def cost(self, a: tuple[int, int], b: tuple[int, int], ha: int, hb: int) -> float:
        """带朝向的精确代价 `(a, ha) -> (b, hb)`（洪泛里直接读，O(1)）。"""
        if a == b:
            return 0.0 if ha == hb else COST_TURN
        dist, _came, _actions, _settled, rank = self._flood_of(a)
        return _flood_value(dist, (b[0], b[1], hb), rank)

    def path(self, a: tuple[int, int], b: tuple[int, int], ha: int, hb: int
             ) -> tuple[list[tuple[int, int, int, int]], float]:
        """取一段最短路径（同一 (起,终,朝向对) 只算一次）。"""
        if a == b:
            return ([(a[0], a[1], ha, ACT_START)], 0.0) if ha == hb else ([], float("inf"))
        dist, came, actions, _settled, rank = self._flood_of(a)
        return _reconstruct(dist, came, actions, (b[0], b[1], hb), rank)

    # ---------------- 代价表 ----------------
    def build(self, cells: Sequence[tuple[int, int]]) -> None:
        """预建两套代价表（每个点一次洪泛 + O(点数²) 的查表）。"""
        ordered = sorted(set(cells))
        for cell in ordered:
            self._flood_of(cell)
        for i, a in enumerate(ordered):
            for b in ordered[i + 1:]:
                table = [[self.cost(a, b, ha, hb) for hb in range(4)]
                         for ha in range(4)]
                self.head_cost[(a, b)] = table
                self.head_cost[(b, a)] = [[table[hb][ha] for hb in range(4)]
                                          for ha in range(4)]
                value = min(min(row) for row in table)
                self.order_cost[(a, b)] = self.order_cost[(b, a)] = value
        # ★ 折返惩罚与"雷心直线穿越"都要用"每对点之间最短腿的起步/末步朝向"
        #   （见 `_record_leg_dirs`）
        if self.record_dirs or self.pivot_penalty > 0.0:
            for a in ordered:
                for b in ordered:
                    if a != b:
                        self._record_leg_dirs(a, b)

    def _record_leg_dirs(self, a: tuple[int, int], b: tuple[int, int]) -> None:
        """记下 `a -> b` **最短腿**的起步朝向与末步朝向（只在末步/起步方向判折返）。

        为什么这么判：无倒车模式里"车在顶点掉头"的物理表现就是**到达方向与离开方向
        正好相反**（原路退回）。这里取的是纯里程最短腿（朝向对里代价最小者，平局按
        (ha, hb) 升序 → 确定性），所以它是顺序层的**下界估计**，不会高估也不会漏判
        那些"必须以折返结束/开始"的访问。
        """
        table = self.head_cost.get((a, b))
        if table is None:
            return
        best: tuple[float, int, int] | None = None
        for ha in range(4):
            for hb in range(4):
                value = table[ha][hb]
                if not math.isfinite(value):
                    continue
                if best is None or value < best[0] - 1e-12:
                    best = (value, ha, hb)
        if best is None:
            return
        path, _ = self.path(a, b, best[1], best[2])
        dirs: list[int] = []
        for prev, cur in zip(path, path[1:]):
            if (cur[0], cur[1]) != (prev[0], prev[1]):
                dirs.append(_heading_of(cur[0] - prev[0], cur[1] - prev[1], -1))
        dirs = [d for d in dirs if d >= 0]
        if dirs:
            self._exit_dir[(a, b)] = dirs[0]
            self._entry_dir[(a, b)] = dirs[-1]

    def leg_cost(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        """`a -> b` 的**顺序层代价**（四个朝向对里的最小值，与 `order_cost` 同口径）。

        新插入的格子（`straighten_mines` 的进点/出点）不在 `order_cost` 里，
        这里按需洪泛并取最小，保证"选哪条穿越轴"的判决与顺序优化用同一把尺子。
        """
        if a == b:
            return 0.0
        hit = self.order_cost.get((a, b))
        if hit is not None:
            return hit
        best = float("inf")
        for ha in range(4):
            for hb in range(4):
                value = self.cost(a, b, ha, hb)
                if value < best:
                    best = value
        return best

    def reversals(self, seq: Sequence[tuple[int, int]]) -> int:
        """访问顺序里的**折返次数**：`a -> b -> c` 且"进 b 的末步朝向"与"出 b 的起步朝向"相反。"""
        count = 0
        for i in range(1, len(seq) - 1):
            entry = self._entry_dir.get((seq[i - 1], seq[i]))
            leave = self._exit_dir.get((seq[i], seq[i + 1]))
            if entry is None or leave is None:
                continue
            if (entry - leave) % 4 == 2:                 # 正相反 = 原路折返
                count += 1
        return count

    def penalty_of(self, seq: Sequence[tuple[int, int]]) -> float:
        """顺序里"不该有的动作"的总代价：折返 + 雷格作拐点 + 隔板死胡同转向
        + **过门转弯余量不足**（约束①） + **外行雷转弯贴太近**（约束②）。

        ★ 第三项：顺序层里 `a → b → c` 若在 `b` 处构成
        "横着切进隔板相邻行、再在行内 90° 转向"（`_barrier_turn_cell`），
        按 `barrier_turn_penalty` 计价。只用顺序相邻两点判据。
        ★ 第四/五项：见 `_door_turn_margin_m()` 与 `_edge_mine_turn_gap()`。
        两者同结构：都是"顺序里某个**转弯点**违反了一条几何约束"⇒ 按次数计价；
        权重缺省 0.0 ⇒ 本函数与 v2.3 逐字节等价（零影响契约）。
        """
        total = 0.0
        if self.pivot_penalty > 0.0:
            total += self.pivot_penalty * self.reversals(seq)
        if self.mine_turn_penalty > 0.0 and self.mine_cells:
            for i in range(1, len(seq) - 1):
                if seq[i] not in self.mine_cells:
                    continue
                entry = self._entry_dir.get((seq[i - 1], seq[i]))
                leave = self._exit_dir.get((seq[i], seq[i + 1]))
                if entry is None or leave is None:
                    continue
                delta = (entry - leave) % 4
                if delta == 0:
                    continue                    # 直线穿越 = 合格
                total += self.mine_turn_penalty * (8.0 if delta == 2 else 1.0)
        if self.barrier_turn_penalty > 0.0:
            for i in range(1, len(seq) - 1):
                row, col = seq[i]
                entry = self._entry_dir.get((seq[i - 1], seq[i]))
                leave = self._exit_dir.get((seq[i], seq[i + 1]))
                if entry is None or leave is None:
                    continue
                if _barrier_turn_cell(self.grid, row, col, entry, leave):
                    total += self.barrier_turn_penalty
        if self.door_turn_penalty > 0.0 or self.edge_mine_turn_penalty > 0.0:
            for i in range(1, len(seq) - 1):
                entry = self._entry_dir.get((seq[i - 1], seq[i]))
                leave = self._exit_dir.get((seq[i], seq[i + 1]))
                if entry is None or leave is None or entry == leave:
                    continue                    # 非转弯点不问
                if self.door_turn_penalty > 0.0:
                    row, col = seq[i]
                    # 阈值走**几何推导**（R·sinθ + w/2·cosθ），不再是固定 0.5 m
                    need = _door_turn_required_margin_m(
                        _turn_deg_from_dirs(entry, leave), self.phys)
                    if _door_turn_margin_m(self.grid, row, col) < need - 1e-12:
                        total += self.door_turn_penalty
                if self.edge_mine_turn_penalty > 0.0 and self.edge_mine_cells:
                    if _edge_mine_turn_gap(seq[i], self.edge_mine_cells) \
                            < EDGE_MINE_TURN_MIN_CELLS:
                        total += self.edge_mine_turn_penalty
        if self.obstacle_turn_penalty > 0.0 and self.obstacle_cells:
            # 圆柱障碍在**转弯点**的净空（拐角用扫掠半径 ⇒ 门槛 0.3029 m）
            threshold = _required_obstacle_clearance_m(self.phys, at_corner=True)
            for i in range(1, len(seq) - 1):
                entry = self._entry_dir.get((seq[i - 1], seq[i]))
                leave = self._exit_dir.get((seq[i], seq[i + 1]))
                if entry is None or leave is None or entry == leave:
                    continue
                if _obstacle_turn_clearance_m(self.grid, seq[i], self.obstacle_cells) \
                        < threshold - 1e-12:
                    total += self.obstacle_turn_penalty
        return total

    # ---------------- 顺序 ----------------
    def order_cost_total(self, seq: Sequence[tuple[int, int]]) -> float:
        """整条顺序的代价（含惩罚项）。**用 `leg_cost`**，所以对 `straighten_mines`
        插进来的新格子也成立 —— 否则 `seq_cost` 读不到预建表会返回 `inf`，
        报告里就会出现"顺序代价 inf"这种假读数（本轮踩过）。
        """
        total = self.penalty_of(seq)
        for a, b in zip(seq, seq[1:]):
            value = self.leg_cost(a, b)
            if not math.isfinite(value):
                return float("inf")
            total += value
        return total

    def seq_cost(self, seq: Sequence[tuple[int, int]]) -> float:
        total = self.penalty_of(seq)
        for a, b in zip(seq, seq[1:]):
            value = self.order_cost.get((a, b), float("inf"))
            if not math.isfinite(value):
                return float("inf")
            total += value
        return total

    def nearest(self, first: tuple[int, int],
                nodes: Sequence[tuple[int, int]]) -> list[tuple[int, int]]:
        """最近邻。**确定性**：距离相同时按格名序，绝不依赖 set/dict 迭代顺序。"""
        remaining = sorted(set(nodes))
        out: list[tuple[int, int]] = []
        cur = first
        while remaining:
            best = min(remaining,
                       key=lambda n: (self.order_cost.get((cur, n), float("inf")),
                                      geo.index_to_block(n[0] - 1, n[1])))
            remaining.remove(best)
            out.append(best)
            cur = best
        return out

    def two_opt(self, seq: list[tuple[int, int]],
                *, keep_ends: bool = True) -> list[tuple[int, int]]:
        """2-opt（子段反转）直到不再改进或达到轮数上限。

        ★ `keep_ends=True`（默认）时**只反转内部子段**：起点/终点是任务硬约束
          （起点 = `map.start` + 设计起始航向 `START_HEADING`，终点要停在 `map.end`），
          2-opt 不该动它们。
          旧实现允许反转含首/末的子段，一旦代价函数变化就会把起点挪走，
          只能在 `plan_route` 末尾靠"内部错误：优化后起点/终点被挪动"兜住 ——
          那是把约束写在断言里而不是算法里，本轮踩到后改为按约束搜索。
        """
        if len(seq) < 3:
            return seq
        i_lo = 1 if keep_ends else 0
        i_hi = len(seq) - 1 if keep_ends else len(seq) - 1
        j_hi = len(seq) - 1 if keep_ends else len(seq)
        improved, passes = True, 0
        while improved and passes < MAX_2OPT_PASSES:
            improved, passes = False, passes + 1
            best = self.seq_cost(seq)
            for i in range(i_lo, i_hi):
                for j in range(i + 2, j_hi):
                    candidate = seq[:i] + list(reversed(seq[i:j + 1])) + seq[j + 1:]
                    cost = self.seq_cost(candidate)
                    if cost < best - 1e-9:
                        seq, best, improved = candidate, cost, True
        return seq

    # ---------------- 朝向 ----------------
    def pair_cost(self, a: tuple[int, int], b: tuple[int, int], ha: int, hb: int) -> float:
        if a == b:
            return 0.0 if ha == hb else COST_TURN
        table = self.head_cost.get((a, b))
        if table is None:
            return self.cost(a, b, ha, hb)
        return table[ha][hb]

    def route_cost(self, order: Sequence[tuple[int, int]], heads: Sequence[int]) -> float:
        return (sum(self.pair_cost(order[i], order[i + 1], heads[i], heads[i + 1])
                    for i in range(len(order) - 1))
                + self.penalty_of(order))

    def best_headings(self, order: Sequence[tuple[int, int]],
                      seed: Sequence[int] | None = None,
                      fixed: dict[tuple[int, int], int] | None = None) -> list[int]:
        """给定顺序，最小化 `Σ pair_cost(i, i+1)` 的朝向序列（坐标下降）。

        起点朝向 = 设计常量 `START_HEADING`（放在 `fixed` 里）；终点朝向自由
        （可在终点原地转向，代价由 `pair_cost` 的 `COST_TURN` 体现）。
        """
        fixed = dict(fixed or {})
        heads: list[int] = []
        for idx, cell in enumerate(order):
            if cell in fixed:
                heads.append(fixed[cell])
            elif seed is not None and idx < len(seed):
                heads.append(seed[idx] % 4)
            else:
                heads.append(0)
        best = self.route_cost(order, heads)
        improved, passes = True, 0
        while improved and passes < MAX_2OPT_PASSES:
            improved, passes = False, passes + 1
            for idx, cell in enumerate(order):
                if cell in fixed:
                    continue
                keep = heads[idx]
                for head in range(4):
                    if head == keep:
                        continue
                    heads[idx] = head
                    value = self.route_cost(order, heads)
                    if value < best - 1e-9:
                        best, keep, improved = value, head, True
                heads[idx] = keep
        return heads

    def polish(self, order: list[tuple[int, int]], heads: Sequence[int],
               rounds: int = 2) -> tuple[list[tuple[int, int]], list[int], float]:
        """顺序 ↔ 朝向交替优化 —— 这正是"相邻两段的到达/出发朝向必须一致"的那一环。"""
        best_order, best_heads = list(order), list(heads)
        best = self.route_cost(best_order, best_heads)
        for _ in range(max(0, rounds)):
            order2 = self.two_opt(list(best_order))
            heads2 = self.best_headings(order2, seed=best_heads)
            value = self.route_cost(order2, heads2)
            if value < best - 1e-9:
                best_order, best_heads, best = order2, heads2, value
            else:
                break
        return best_order, best_heads, best


# --------------------------------------------------------------------------- #
#  主入口
# --------------------------------------------------------------------------- #
def _limits(cfg: Config | None) -> tuple[float, float]:
    """射击距离窗口。真源 = `Config`（`path_executor.shoot_distance_min/max`），
    缺省值直接用 `validate.DEFAULT_SHOOT_DISTANCE_*`（不硬编码第二份默认值）。"""
    lo = float(V._cfg_get(cfg, "path_executor.shoot_distance_min",     # noqa: SLF001
                          V.DEFAULT_SHOOT_DISTANCE_MIN_M))
    hi = float(V._cfg_get(cfg, "path_executor.shoot_distance_max",     # noqa: SLF001
                          V.DEFAULT_SHOOT_DISTANCE_MAX_M))
    return (lo, hi) if lo <= hi else (hi, lo)


def _shoot_accept(cfg: Config | None) -> tuple[float, float]:
    """★ SHOOT 停站点到**靶板中心**的 `(目标距离, 半宽)`。

    真源 = `Config` 的 `route.shoot_distance_m` / `route.shoot_distance_tol_m`
    （现场 `config/robot.yaml` 有就用现场的；缺键落到 `SHOOT_DISTANCE_*_DEFAULT`）。
    """
    mid = float(V._cfg_get(cfg, "route.shoot_distance_m", SHOOT_DISTANCE_M_DEFAULT))  # noqa: SLF001
    tol = float(V._cfg_get(cfg, "route.shoot_distance_tol_m",                          # noqa: SLF001
                           SHOOT_DISTANCE_TOL_M_DEFAULT))
    return mid, abs(tol)


def _cross_pools(cfg: Config | None, spec: MapSpec) -> tuple[list[str], list[str]]:
    """上下半场穿洞候选格。优先级（与 `validate.py` 的口径一致）：

    ① `Config` 的 `route.cross_lower/upper_candidates`（每局随机、无人机情报选格）
    ② `map.bridges`（本局现场识图结果）—— **非空时与①取交集**：换一张地图就必须用那张
       地图的桥洞，否则会拿上一张图的候选格去撞新图的桥墩（本轮实测踩过：换成
       `bridges: [K1,K7]` 的新图后，`robot.yaml` 的 `K9/K4` 仍然被当成候选，
       路线会横穿非桥洞的 K 列）。
    ③ 交集为空或都没给 → 退到"整列 K 的对应半场"（**不写死任何一格**）
    """
    def split(items: Iterable[str]) -> tuple[list[str], list[str]]:
        lower, upper = [], []
        for cell in items:
            try:
                row_idx, col_idx = geo.block_to_index(str(cell))
            except ValueError:
                continue
            if col_idx != _CROSS_COL:
                continue
            row = row_idx + 1
            if V.LOWER_HALF_ROWS[0] <= row <= V.LOWER_HALF_ROWS[1]:
                lower.append(str(cell))
            elif V.UPPER_HALF_ROWS[0] <= row <= V.UPPER_HALF_ROWS[1]:
                upper.append(str(cell))
        return lower, upper

    lower, upper = split(V._as_list(V._cfg_get(                  # noqa: SLF001
        cfg, "route.cross_lower_candidates", None)))
    upper = upper + split(V._as_list(V._cfg_get(                 # noqa: SLF001
        cfg, "route.cross_upper_candidates", None)))[1]
    b_lower, b_upper = split(spec.bridges)
    if b_lower and lower:
        lower = [c for c in lower if c in set(b_lower)] or list(b_lower)
    if b_upper and upper:
        upper = [c for c in upper if c in set(b_upper)] or list(b_upper)
    lower = lower or b_lower
    upper = upper or b_upper
    if not lower:
        lower = [f"{V.CROSS_COLUMN}{r}"
                 for r in range(V.LOWER_HALF_ROWS[0], V.LOWER_HALF_ROWS[1] + 1)]
    if not upper:
        upper = [f"{V.CROSS_COLUMN}{r}"
                 for r in range(V.UPPER_HALF_ROWS[0], V.UPPER_HALF_ROWS[1] + 1)]
    return sorted(set(lower)), sorted(set(upper))


def _default_cross(cfg: Config | None, pool: Sequence[str], half: str) -> str:
    preferred = str(V._cfg_get(cfg, f"route.cross_{half}_default", "") or "")   # noqa: SLF001
    preferred = preferred.strip().upper()
    if preferred and preferred in pool:
        return preferred
    return sorted(pool)[0]


def _u_shape_order(tour: "_Tour", start: tuple[int, int], end: tuple[int, int],
                   mines_lower: Sequence[tuple[int, int]], mines_upper: Sequence[tuple[int, int]],
                   cross_lower: tuple[int, int], cross_upper: tuple[int, int],
                   *, seed_from_cross: bool = False) -> list[tuple[int, int]]:
    """★★ U 形单回路顺序：把两个穿洞格**钉在各自半场的边界**上。

    为什么需要它：`_Tour.polish()`（= 顺序↔朝向交替 2-opt）会在**整条 order** 上反转
    任意子段，于是把 `cross_upper` 从"本半场末尾"挪到半场中间。实测（第三张图
    K1+K11 那张）：

        polish 前  N10 K11 I7 D8 E10 C4 G4 F3 H2 B2 K1 N5   ← 合规形状
        polish 后  N10 K11 E10 D8 I7 C4 B2 K1 H2 G4 F3 N5   ← `K1` 夹在中间
        编译结果   … H1 I1 J1 K1 L1 … 且 `J1→K1` 出现两次
                   ⇒ `上半场应恰好穿洞 1 次，实际 2 次`

    现役手工路线（`config/robot.yaml` 的 `route.tokens`，44 token、`route_check` 0 error）
    正是这个拓扑：**`cross_lower` 是本半场离开东区后的第一跳、`cross_upper` 是本半场
    回到东区前的最后一跳**。

    本函数把"每半场一次穿洞"做成**硬约束**（不是加罚）：
      · 下半场序 = `2-opt_from(start, 下半场雷)` + `[cross_lower]`；
      · 上半场序 = `2-opt_from(cross_lower, 上半场雷)` + `[cross_upper]`；
      · 全序 = `[start] + 下半场序 + 上半场序 + [end]`。
    `2-opt` 只用在本半场节点集合内 ⇒ 跨 K 的往返**在搜索空间里不存在**，不可能被破坏。

    :return: 候选顺序（长度 = 2 + 两个半场节点数 + 两个穿洞格）；**调用方仍需跑
        结构自检**——本函数只保证形状，不保证编译后一定合规（射击停站/雷心穿越是后处理）。
    """
    if mines_lower:
        # ★★ 关键：`cross_lower` 必须是**下半场第一跳**（实测枚举 24 种下半场顺序，
        #   只有"`K11` 在最前"这种拓扑能得到"下洞 1 次"；放在末尾必然多一次
        #   `L11→K11→J11 … J11→K11` 往返 —— 见 `probe_lower_orders.txt`）。
        #   所以这里**先**把 `cross_lower` 固定为起点，再在其后做本半场 2-opt；
        #   不再让 `two_opt` 有机会把它挪到别处。
        if seed_from_cross:
            tail = tour.two_opt(tour.nearest(cross_lower, list(mines_lower)))
            lower = [cross_lower] + [c for c in tail if c != cross_lower]
        else:
            tail = tour.two_opt(tour.nearest(start, list(mines_lower)))
            lower = [cross_lower] + [c for c in tail if c != cross_lower]
    else:
        lower = [cross_lower]
    upper_nodes = list(mines_upper) + [cross_upper]
    upper = tour.two_opt(tour.nearest(cross_lower, upper_nodes)) if upper_nodes else [cross_upper]
    if cross_upper not in upper:                         # 纯防御
        upper.append(cross_upper)
    else:
        upper = [c for c in upper if c != cross_upper] + [cross_upper]
    return [start] + lower + upper + [end]


# --------------------------------------------------------------------------- #
#  ★ 任务优先口径（TRACK-PLAN-B1-20261021）的辅助函数
# --------------------------------------------------------------------------- #
def objective_mode(value: object) -> str:
    """校验 objective 开关（缺省 legacy）。未知值 → PlannerError（不静默回退）。"""
    obj = str(value or OBJECTIVE_LEGACY).strip().lower()
    if obj not in OBJECTIVES:
        raise PlannerError(f"未知 objective={value!r}（只支持 {'/'.join(OBJECTIVES)}）")
    return obj


def _side_split(cells: Sequence[tuple[int, int]]) -> tuple[list[tuple[int, int]],
                                                          list[tuple[int, int]]]:
    """按 K 列把本半场必访点分成「东侧（穿洞前）」与「西侧（穿洞后）」。"""
    east = [c for c in cells if c[1] > _CROSS_COL]
    west = [c for c in cells if c[1] < _CROSS_COL]
    on = [c for c in cells if c[1] == _CROSS_COL]
    return east, west + on


def _side_shape_order(tour: "_Tour", start: tuple[int, int], end: tuple[int, int],
                      mines_lower: Sequence[tuple[int, int]],
                      mines_upper: Sequence[tuple[int, int]],
                      cross_lower: tuple[int, int],
                      cross_upper: tuple[int, int]) -> list[tuple[int, int]]:
    """U 形单回路 + 每半场按 K 列**东西分组**的顺序（任务优先口径的结构候选）。

    旧 U 形候选把「本半场雷」当一个整体排序，再硬钉穿洞格在边界；随机图里雷会
    同时落在 K 列两侧 ⇒ 2-opt 可能让一条腿**来回穿 K 两次**（实测 19 张图）。
    本候选把结构写进顺序本身：**先东侧、穿洞一次、再西侧**，每半场只穿一次洞。
    """
    def walk(anchor: tuple[int, int], nodes: Sequence[tuple[int, int]]
             ) -> tuple[list[tuple[int, int]], tuple[int, int]]:
        if not nodes:
            return [], anchor
        seq = tour.two_opt(tour.nearest(anchor, list(nodes)))
        return seq, seq[-1]

    lower_east, lower_west = _side_split(mines_lower)
    upper_east, upper_west = _side_split(mines_upper)
    seq: list[tuple[int, int]] = [start]
    prev = start
    part, prev = walk(prev, lower_east)
    seq += part
    seq.append(cross_lower)
    prev = cross_lower
    part, prev = walk(prev, lower_west)
    seq += part
    part, prev = walk(prev, upper_west)
    seq += part
    seq.append(cross_upper)
    prev = cross_upper
    part, prev = walk(prev, upper_east)
    seq += part
    seq.append(end)
    out: list[tuple[int, int]] = []
    for cell in seq:
        if not out or out[-1] != cell:
            out.append(cell)
    return out


def _shoot_positions_for(order: Sequence[tuple[int, int]], cell: tuple[int, int],
                         cross_lower: tuple[int, int] | None,
                         cross_upper: tuple[int, int] | None) -> list[int]:
    """停站的**合法插入位**：按「半场 × K 列东西」限制在穿洞格之间，避免多穿一次洞。"""
    n = len(order)
    if n < 2:
        return []
    if n == 2:
        return [1]
    li = order.index(cross_lower) if cross_lower in order else -1
    ui = order.index(cross_upper) if cross_upper in order else -1
    lower = cell[0] >= V.LOWER_HALF_ROWS[0]
    east = cell[1] > _CROSS_COL
    if li >= 0 and ui > li:
        if lower and east:
            lo, hi = 1, li + 1
        elif lower:
            lo, hi = li + 1, ui
        elif east:
            lo, hi = ui + 1, n - 1
        else:
            lo, hi = li + 1, ui
    else:
        lo, hi = 1, n - 1
    lo = max(1, lo)
    hi = min(n - 1, hi)
    if hi <= lo:
        return [lo] if 0 < lo < n else []
    return list(range(lo, hi))


def _insert_stop_ranged(tour: "_Tour", order: list[tuple[int, int]],
                        cell: tuple[int, int], positions: Sequence[int]
                        ) -> list[tuple[int, int]]:
    """在给定插入位里选「绕路增量最小」的一处插入停站（不新增阈值）。"""
    best: tuple[float, int] | None = None
    for i in positions:
        if i < 1 or i > len(order) - 1:
            continue
        prev_cell, next_cell = order[i - 1], order[i]
        value = (tour.leg_cost(prev_cell, cell) + tour.leg_cost(cell, next_cell)
                 - tour.leg_cost(prev_cell, next_cell))
        if not math.isfinite(value):
            continue
        key = (value, i)
        if best is None or key < best:
            best = key
    if best is None:
        return order
    out = list(order)
    out.insert(best[1], cell)
    return out


def _insert_shoot_sided(tour: "_Tour", order: list[tuple[int, int]],
                         pool: Sequence[str], cross_lower: tuple[int, int],
                         cross_upper: tuple[int, int]
                         ) -> tuple[list[tuple[int, int]], str]:
    """任务优先口径的**主**射击停站插入：只在不会破坏穿洞次数的区间里选点。"""
    best: tuple[float, int, str] | None = None
    for block in pool:
        cell = _cell(block)
        if cell in order:
            pos = order.index(cell)
            if pos in (0, len(order) - 1):      # 起点/终点不能当射击停站
                continue
            cand, detour = list(order), 0.0
        else:
            positions = _shoot_positions_for(order, cell, cross_lower, cross_upper)
            if not positions:
                continue
            cand = _insert_stop_ranged(tour, order, cell, positions)
            if cell not in cand:
                continue
            pos = cand.index(cell)
            prev_cell = order[pos - 1] if pos > 0 else None
            next_cell = order[pos] if pos < len(order) else None
            if prev_cell is None or next_cell is None:
                continue
            detour = (tour.leg_cost(prev_cell, cell) + tour.leg_cost(cell, next_cell)
                      - tour.leg_cost(prev_cell, next_cell))
            if not math.isfinite(detour):
                continue
        key = (detour, pos, block)
        if best is None or key < best[0]:
            best = (key, cand, block)
    if best is None:
        return order, ""
    return best[1], best[2]


def _insert_second_sided(tour: "_Tour", order: list[tuple[int, int]],
                         block: str, cross_lower: tuple[int, int],
                         cross_upper: tuple[int, int]) -> list[tuple[int, int]]:
    """对侧第二射击停站：作为**普通航点**插入（不收口时也保留路径形状）。"""
    cell = _cell(block)
    if cell in order:
        return list(order)
    positions = _shoot_positions_for(order, cell, cross_lower, cross_upper)
    return _insert_stop_ranged(tour, order, cell, positions)


def _reachable_any(dist: dict, rank: dict, cell: tuple[int, int]) -> bool:
    return any(math.isfinite(_flood_value(dist, (cell[0], cell[1], head), rank))
               for head in range(4))


def _pick_crossing_tasks(spec: MapSpec, cfg: "Config | None",
                         lower_pool: Sequence[str], upper_pool: Sequence[str],
                         *, allow_reverse: bool, barrier_turn_margin_m: float
                         ) -> tuple[str, str] | None:
    """任务优先口径：在候选穿洞格里选「终点可达 + 可达雷最多」的一对（≤8 组）。

    为什么需要：随机图的 bridges 可能两格都在同一半场，或默认格恰好被靶板/障碍
    堵死；旧实现只取 sorted(pool)[0]，一条路走到黑。这里用每次洪泛（ms 级）先
    选可行组合，再交给完整规划，避免在不可行组合上白烧整轮编译。
    """
    start_cell, end_cell = _cell(spec.start), _cell(spec.end)

    def preferred(pool: Sequence[str], half: str) -> str:
        want = str(V._cfg_get(cfg, f"route.cross_{half}_default", "") or "").strip().upper()
        return want if want in pool else sorted(pool)[0]

    combos: list[tuple[str, str, int, int]] = []
    for lower in lower_pool:
        for upper in upper_pool:
            try:
                lr = geo.block_to_index(lower)[0] + 1
                ur = geo.block_to_index(upper)[0] + 1
            except ValueError:
                continue
            combos.append((lower, upper, lr, ur))
    if not combos:
        return None
    pl, pu = preferred(lower_pool, "lower"), preferred(upper_pool, "upper")
    combos.sort(key=lambda c: (0 if (c[0] == pl and c[1] == pu) else 1, c[0], c[1]))
    best: tuple[tuple[int, int], str, str] | None = None
    for lower, upper, lr, ur in combos[:8]:
        try:
            grid = Grid(spec, bridge_rows=(lr, ur), allow_reverse=allow_reverse,
                        barrier_turn_margin_m=barrier_turn_margin_m)
        except Exception:                              # noqa: BLE001
            continue
        if not grid.passable(*start_cell) or not grid.passable(*end_cell):
            continue
        dist, _came, _actions, _settled, rank = flood(
            grid, (start_cell[0], start_cell[1], START_HEADING))
        end_ok = int(_reachable_any(dist, rank, end_cell))
        mines_ok = sum(1 for mine in spec.mines
                       if _reachable_any(dist, rank, _cell(mine)))
        score = (end_ok, mines_ok)
        if best is None or score > best[0]:
            best = (score, lower, upper)
    if best is None:
        return None
    return best[1], best[2]


def _best_effort_result(spec: MapSpec, cfg: "Config | None", grid: Grid, *,
                        name: str, source: str, t0: float, allow_reverse: bool,
                        reason: str, warnings: Sequence[str],
                        deadline: float | None = None) -> PlanResult:
    """降级兜底：终点不可达时，沿**真实可通行**路径走到离终点最近的格并落串。

    ★ 这不是「假装规划成功」：返回 degraded=True + degraded_reason 会进 CLI/JSON；
      token 全部是 A* 验证过的可通行相邻格，车能照着跑（只是到不了终点）。
      绝不发明穿障碍/穿桥墩的段。
    """
    start_cell = _cell(spec.start)
    dist, came, actions, _settled, rank = flood(
        grid, (start_cell[0], start_cell[1], START_HEADING))
    cells = {(k[0], k[1]) for k in dist}
    if not cells:
        raise PlannerError(f"无法生成降级路线：起点 {spec.start} 在栅格里不可达")
    ex, ey = geo.block_to_meters(spec.end.strip().upper())

    def end_dist(cell: tuple[int, int]) -> float:
        block = geo.index_to_block(cell[0] - 1, cell[1])
        x, y = geo.block_to_meters(block)
        return math.hypot(x - ex, y - ey)

    # ★ 任务优先兜底：先把**本连通块内可达的雷**都排进去，再去离终点最近的可行格；
    #   这样降级路线仍尽可能多压雷（而不是第一步就停在离终点最近的邻居）。
    reachable_mines = [_cell(m) for m in spec.mines
                       if _reachable_any(dist, rank, _cell(m))]
    goal = min(cells, key=lambda c: (end_dist(c), geo.index_to_block(c[0] - 1, c[1])))
    nodes = [start_cell] + reachable_mines + [goal]
    dl = deadline if deadline is not None else t0 + DEFAULT_MAX_SECONDS
    tour = _Tour(grid, dl)
    tour.build(nodes)
    middle = tour.nearest(start_cell, reachable_mines) if reachable_mines else []
    order = [start_cell] + middle
    if goal != order[-1]:
        order.append(goal)
    if len(order) > 2:
        order = tour.two_opt(order, keep_ends=True)
    heads = tour.best_headings(order, fixed={order[0]: START_HEADING})
    keep = set(order)
    stops: list[_Stop] = [_Stop(start_cell[0], start_cell[1], spec.start,
                                heading=START_HEADING, tag="start")]
    for i2 in range(len(order) - 1):
        path, _seg = tour.path(order[i2], order[i2 + 1], heads[i2], heads[i2 + 1])
        if not path:
            continue
        for (row, col, face, back) in _collapse(path, keep):
            if (row, col) == (stops[-1].row, stops[-1].col):
                stops[-1].reverse = back
                stops[-1].heading = face
                continue
            blk = geo.index_to_block(row - 1, col)
            stops.append(_Stop(row, col, blk, heading=face, reverse=back,
                               tag="mine" if blk in set(spec.mines) else "path"))
    stops = _split_long_hops(stops, grid)
    stops = _merge_duplicate_stops(stops)
    _refresh_headings(stops)
    if not allow_reverse:
        mark_pivots(stops)
    route = _to_route(stops, name=name, crossing={},
                      final_heading=_final_heading(stops))
    tokens = route_tokens(route)
    blocks = route.blocks()
    covered = tuple(m for m in spec.mines if m.strip().upper() in set(blocks))
    shoots = tuple(wp.block for _i, wp in route.shoot_points())
    polyline = sum(math.hypot(route.waypoints[i + 1].x - route.waypoints[i].x,
                              route.waypoints[i + 1].y - route.waypoints[i].y)
                   for i in range(len(route.waypoints) - 1))
    return PlanResult(
        route=route, tokens=tokens, crossing={},
        cost=float("inf"), expanded=0,
        elapsed_ms=(time.perf_counter() - t0) * 1000.0,
        reverse_segments=sum(1 for wp in route.waypoints if wp.reverse),
        warnings=tuple(warnings) + (
            "降级路线只走到离终点最近的可行格（不是完整任务路线）",),
        map_name=spec.name, map_source=source, start=spec.start, end=spec.end,
        polyline_m=polyline, start_heading=HEAD_NAMES[START_HEADING],
        allow_reverse=bool(allow_reverse),
        tasks_mines=covered, tasks_stops=shoots,
        tasks_mines_total=len(spec.mines),
        degraded=True, degraded_reason=reason,
    )


def plan_route(map_cfg: Any, cfg: Config | None = None, *,
               cross_lower: str | None = None, cross_upper: str | None = None,
               max_seconds: float = DEFAULT_MAX_SECONDS,
               allow_reverse: bool = True,
               objective: str = OBJECTIVE_LEGACY,
               pivot_penalty: float | None = None,
               mine_turn_penalty: float | None = None,
               barrier_turn_penalty: float | None = None,
               barrier_turn_margin_m: float | None = None,
               door_turn_penalty: float | None = None,
               edge_mine_turn_penalty: float | None = None,
               obstacle_turn_penalty: float | None = None,
               name: str = "planner", source: str = "") -> PlanResult:
    """**核心入口**：一张地图 -> 一条可直接下发的路线。

    :param map_cfg: `config/map.yaml` 的原始 dict（`{'map': {...}}`）；也接受 `MapSpec`
    :param cfg: `core.config.Config`；提供射击窗口、穿洞候选集合、代价项等
        （**不再**从它读起点朝向：起始航向是设计常量 `START_HEADING`）
    :param cross_lower/cross_upper: 指定穿洞格（通常由无人机情报给出）；
        缺省用 `route.cross_*_default`；硬门 = **K 列 + 本半场行段**（下 7~11 / 上 1~6），行段外/非 K 列/坏格名才抛 `PlannerError`（候选集合只用来挑缺省洞，无人机情报可纠正过期地图）
    :param max_seconds: 时间预算；超出抛 `PlannerError`
    :param allow_reverse: **False = 无倒车模式**（车上 dzbrain 的默认用法）。
        真车底盘不支持倒车（实测：负速度方向位被 STM32 无视），所以无倒车模式：
        ① 搜索动作集里**没有倒车**（`Grid.allow_reverse`）；② **跳过** `_fix_uturn_reverse`
        （不把 180° 折返改写成 `R:`）；③ 给每个折返顶点打 `:PIVOT` 声明（`mark_pivots`）；
        ④ 合法性断言换成 `assert_legal_no_reverse`（0 段 `R:` + 折返声明完整且不虚报）。
        True = 旧行为（`R:` 倒车段处理死角），**默认值不变**（签名向后兼容）。
    :param pivot_penalty: 顺序搜索的折返惩罚（等效代价单位/处）。`None`（缺省）=
        **跟随配置** `planner.pivot_penalty`（键缺失 → 0.0，行为与历史基线一致）。
    :param mine_turn_penalty: 顺序搜索里"**雷格不得作为拐点**"的惩罚权重。
        `None`（缺省）= 跟随配置 `planner.mine_turn_penalty`（键缺失 → `MINE_TURN_PENALTY`）。
        每压一个雷都要多 2 格路（进点/出点），这一项保证"宁可多绕一格，也不让雷被切角擦过"。
    :param barrier_turn_penalty: ★★ **隔板相邻行的行内 90° 转向** 的惩罚权重
        `None`（缺省）= 跟随配置 `planner.barrier_turn_penalty`
        （**键缺失 → 0.0，行为与历史逐 token 一致**）。> 0 时：洪泛代价与顺序代价
        都会避开"横着切进隔板相邻行、再在行内 90° 转向"这一形态；规划结束后
        再做一次**硬校验**，若结果里仍存在该形态则抛 `PlannerError`（绝不静默降级）。
        机理与实测归因见 `_barrier_turn_cell()`。
    :param barrier_turn_margin_m: 该形态判据的**最小安全裕度**（米）。
        `None`（缺省）= 跟随配置 `planner.barrier_turn_margin_m`（键缺失 → 0.0）。
        行心到隔板只有半格（`0.5 × 0.35 = 0.175 m`）：`0.0` = 按纯几何一律拦下；
        **负值 = 关闭该判据**（退化成诊断模式，路线与历史逐字相同）。
    :param objective: 目标函数口径（★TRACK-PLAN-B1-20261021）。**缺省 legacy = 历史
        行为逐字节不变**；`tasks` = 任务优先（先最大化成功任务数：8 雷 + 可达射击
        停站；再最小化时间）。`tasks` 下结构/任务约束不满足不再判"无解"：先选可达的
        穿洞组合、按 K 列东西分组压雷，仍不满足则返回 `degraded=True` 的**可执行兜底
        路线**（`degraded_reason` 进 CLI/JSON）。未知值抛 `PlannerError`（不静默回退）。
    :raises PlannerError: 无解 / 结构约束矛盾 / 超时 / 折返声明不一致 /
        隔板行内 90° 转向未避开（legacy 口径；tasks 口径见 `objective`）
    """
    t0 = time.perf_counter()
    spec = map_cfg if isinstance(map_cfg, MapSpec) else load_map(map_cfg, source=source)
    objective = objective_mode(objective)
    limits = _limits(cfg)
    warnings: list[str] = []
    timed_out = False
    missing_mines: list[str] = []
    shoot_missing = False
    deadline = t0 + max(0.05, float(max_seconds))
    if pivot_penalty is None:
        pivot_penalty = float(V._cfg_get(cfg, "planner.pivot_penalty", 0.0) or 0.0)   # noqa: SLF001
    pivot_penalty = max(0.0, float(pivot_penalty))
    if mine_turn_penalty is None:
        mine_turn_penalty = float(V._cfg_get(                                          # noqa: SLF001
            cfg, "planner.mine_turn_penalty", MINE_TURN_PENALTY) or 0.0)
    mine_turn_penalty = max(0.0, float(mine_turn_penalty))
    if barrier_turn_penalty is None:
        barrier_turn_penalty = float(V._cfg_get(                                       # noqa: SLF001
            cfg, "planner.barrier_turn_penalty", 0.0) or 0.0)
    barrier_turn_penalty = max(0.0, float(barrier_turn_penalty))
    if barrier_turn_margin_m is None:
        barrier_turn_margin_m = float(V._cfg_get(                                      # noqa: SLF001
            cfg, "planner.barrier_turn_margin_m", 0.0) or 0.0)
    barrier_turn_margin_m = float(barrier_turn_margin_m)
    # ★ 两条硬约束的权重（缺省 0.0 ⇒ 零影响，与 v2.3 逐字节一致）
    if door_turn_penalty is None:
        door_turn_penalty = float(V._cfg_get(                                          # noqa: SLF001
            cfg, "planner.door_turn_penalty", 0.0) or 0.0)
    door_turn_penalty = max(0.0, float(door_turn_penalty))
    if edge_mine_turn_penalty is None:
        edge_mine_turn_penalty = float(V._cfg_get(                                     # noqa: SLF001
            cfg, "planner.edge_mine_turn_penalty", 0.0) or 0.0)
    edge_mine_turn_penalty = max(0.0, float(edge_mine_turn_penalty))
    # 物理几何参数与"障碍转弯净空"权重（缺省 0.0 ⇒ 零影响）
    phys = _phys_params(cfg)
    if obstacle_turn_penalty is None:
        obstacle_turn_penalty = float(V._cfg_get(                                       # noqa: SLF001
            cfg, "planner.obstacle_turn_penalty", 0.0) or 0.0)
    obstacle_turn_penalty = max(0.0, float(obstacle_turn_penalty))

    start_cell, end_cell = _cell(spec.start), _cell(spec.end)

    # ---------------- 穿洞格（先定，再建带桥洞约束的栅格） ----------------
    # 权威门放宽 —— 判据从"必须在
    #   `route.cross_{half}_candidates ∩ map.bridges` 的池子里"放宽为
    #   **K 列 + 本半场行段**（下 K7~K11 / 上 K1~K6）：这样无人机的新鲜情报可以
    #   **纠正过期地图**（地图/候选还停在上一局、只开了别的洞时，行段内别的洞照样能走）。
    #   仍未放宽的：格名非法（`Z9`/`ZZ9`）、不在 K 列（`A1`）、跨半场行段 —— 一律 PlannerError。
    #   池子仍然参与**缺省值**选择（`_default_cross`），只是不再当硬门。
    lower_pool, upper_pool = _cross_pools(cfg, spec)
    lower_cross = (cross_lower or _default_cross(cfg, lower_pool, "lower")).strip().upper()
    upper_cross = (cross_upper or _default_cross(cfg, upper_pool, "upper")).strip().upper()
    if objective == OBJECTIVE_TASKS and not (cross_lower or cross_upper):
        _pick = _pick_crossing_tasks(spec, cfg, lower_pool, upper_pool,
                                     allow_reverse=allow_reverse,
                                     barrier_turn_margin_m=barrier_turn_margin_m)
        if _pick is not None:
            lower_cross, upper_cross = _pick
    for cell, half, band in ((lower_cross, "下半场", V.LOWER_HALF_ROWS),
                             (upper_cross, "上半场", V.UPPER_HALF_ROWS)):
        try:
            row_idx, col_idx = geo.block_to_index(cell)
        except ValueError as exc:
            raise PlannerError(f"{half}穿洞格 {cell} 格名非法（{exc}）") from None
        if col_idx != _CROSS_COL:
            raise PlannerError(f"穿洞格 {cell} 不在 {V.CROSS_COLUMN} 列上")
        row = row_idx + 1
        if not (band[0] <= row <= band[1]):
            raise PlannerError(
                f"{half}穿洞格 {cell} 不在 {V.CROSS_COLUMN}{band[0]}~{V.CROSS_COLUMN}{band[1]} "
                f"行段内（本半场只允许这一段 K 列格；跨半场/非 K 列的池外非法格一律拒绝）"
                f" —— 缺省候选来自 route.cross_lower/upper_candidates 与 map.bridges")
        # 池外但在本半场行段内 → 采纳（池子只用于缺省值；上面的行段判据才是硬门）
    cross_lower_cell, cross_upper_cell = _cell(lower_cross), _cell(upper_cross)
    if cross_lower_cell == cross_upper_cell:
        raise PlannerError(f"下半场与上半场选了同一个桥洞 {lower_cross}（必须各穿一次）")

    grid = Grid(spec, bridge_rows=(cross_lower_cell[0], cross_upper_cell[0]),
                allow_reverse=allow_reverse,
                barrier_turn_margin_m=barrier_turn_margin_m)
    if not grid.passable(*start_cell):
        raise PlannerError(f"起点 {spec.start} 是障碍格，无法出发")
    if not grid.passable(*end_cell):
        raise PlannerError(f"终点 {spec.end} 是障碍格，无法到达")
    for cell, name, half in ((cross_lower_cell, lower_cross, "下半场"),
                             (cross_upper_cell, upper_cross, "上半场")):
        if not grid.passable(*cell):
            raise PlannerError(f"{half}穿洞格 {name} 是障碍格，无法通行")
    if objective == OBJECTIVE_TASKS:
        _dist0, _came0, _act0, _set0, _rank0 = flood(
            grid, (start_cell[0], start_cell[1], START_HEADING))
        if not _reachable_any(_dist0, _rank0, end_cell):
            warnings.append(f"终点 {spec.end} 在该穿洞组合下不可达（起点连通块到不了"
                            f"上半场东侧）→ 输出降级兜底路线")
            return _best_effort_result(
                spec, cfg, grid, name=name, source=source, t0=t0,
                allow_reverse=allow_reverse, reason="结构（终点不可达）",
                warnings=warnings, deadline=deadline)
        for mine in spec.mines:
            cell = _cell(mine)
            if grid.passable(*cell) and not _reachable_any(_dist0, _rank0, cell):
                missing_mines.append(mine)
        if missing_mines:
            warnings.append(f"雷不可达、任务降级：{' '.join(missing_mines)}")

    # ---------------- 起点朝向 = 设计常量 ----------------
    #   ★ 真源不再是 `mission.reset`（旧实现按 `reset_yaw` 反推，还会在"朝西不可行"时
    #     把起点朝向改成朝 X+）—— 那正是要删掉的"为起点生成转向消解"：
    #     现场约定车每次都摆在起点、车头恒定朝 +y（图左 / K 列方向），摆位由用户保证。
    #   ⇒ 只读设计常量：不校验、不拒绝、不改写（见 `START_HEADING` 的说明）。
    start_heading = START_HEADING

    # ---------------- U 形结构可行性 ----------------
    # 下半场：起点在桥东 -> 穿 K(lower) 向西 -> 收站；上半场：收站 -> 穿 K(upper) 向东 -> 终点。
    # 于是两条必要条件是"起点/终点必须在各自桥洞的东侧"，否则必然要穿两次（违反 R007）。
    if start_cell[1] < _CROSS_COL:
        raise PlannerError(
            f"起点 {spec.start}（列索引 {start_cell[1]}）不在下半场桥洞 {lower_cross}"
            f"（列索引 {_CROSS_COL}）以东 —— 与「每半场只穿一次桥洞」（validate R007）"
            f"矛盾：开到西侧再回东侧起点就必须穿两次。请检查地图或穿洞候选")
    if end_cell[1] < _CROSS_COL:
        raise PlannerError(
            f"终点 {spec.end}（列索引 {end_cell[1]}）不在上半场桥洞 {upper_cross}"
            f"（列索引 {_CROSS_COL}）以东 —— 与「每半场只穿一次桥洞」（validate R007）"
            f"矛盾。请检查地图或穿洞候选")

    # ---------------- 必访点：雷按半场分组 ----------------
    mines_lower, mines_upper = [], []
    _missing_set = {m.strip().upper() for m in missing_mines}
    for mine in spec.mines:
        if str(mine).strip().upper() in _missing_set:
            continue                                    # 任务优先：不可达雷已记降级
        cell = _cell(mine)
        if not grid.passable(*cell):
            if objective == OBJECTIVE_TASKS:
                missing_mines.append(mine)
                warnings.append(f"雷 {mine} 落在障碍格上 → 任务降级（跳过）")
                continue
            raise PlannerError(f"雷 {mine} 落在障碍格上（地图自相矛盾），无法压雷")
        (mines_lower if grid.zone_of(cell[0]) == 1 else mines_upper).append(cell)

    shoot_pool: list[str] = []
    if spec.targets:
        acc = _shoot_accept(cfg)
        acc_lo, acc_hi = shoot_distance_window(acc)
        pool = shoot_candidates(spec, grid, acc)
        if not pool and objective == OBJECTIVE_TASKS:
            shoot_missing = True
            warnings.append(f"靶 {' '.join(spec.targets)} 周围没有窗口 "
                            f"{acc_lo:.2f}~{acc_hi:.2f} m 内的可通行射击点 → 缺停站")
        elif not pool:
            raise PlannerError(f"靶 {' '.join(spec.targets)} 周围没有落在窗口 "
                               f"{acc_lo:.2f}~{acc_hi:.2f} m 内的可通行射击点")
        cov = covering_stops(spec, pool, acc) if pool else {}
        shoot_pool = [b for b in pool if len(cov[b]) == len(spec.targets)]
        if not shoot_pool and objective == OBJECTIVE_TASKS:
            shoot_missing = True
            detail = "; ".join(f"{b}->{'/'.join(cov[b]) or '无'}" for b in pool)
            warnings.append(f"没有一个停站能同时覆盖全部 {len(spec.targets)} 格靶"
                            f"（候选 {detail}）→ 缺停站")
        elif not shoot_pool:
            detail = "; ".join(f"{b}->{'/'.join(cov[b]) or '无'}" for b in pool)
            raise PlannerError(
                f"靶区 {' '.join(spec.targets)} 找不到**一个**能同时覆盖全部 "
                f"{len(spec.targets)} 格靶的射击停站点（route_check 会判「靶覆盖 "
                f"{len(spec.targets) - 1}/{len(spec.targets)}」）。候选与覆盖：{detail}。"
                f"若这是现场实际布局，请确认 targets 是否写成了不相邻的两格，"
                f"或射击窗口 {limits[0]:.2f}~{limits[1]:.2f} m 是否需要现场标定")

    # ---------------- 顺序 + 朝向联合优化 ----------------
    #   ★ "雷格不得作拐点"的惩罚同样只在无倒车模式下生效（理由同上：允许倒车 = 旧行为）
    mine_cells = set() if allow_reverse else {_cell(m) for m in spec.mines}
    edge_mine_cells = _edge_mine_cells(grid, spec) if edge_mine_turn_penalty > 0.0 else set()
    # 障碍格 = blocked 去掉靶板（靶板另有格级判据，不参与"圆柱净空"）
    obstacle_cells = set(grid.blocked - grid.target_cells) if obstacle_turn_penalty > 0.0 else set()
    tour = _Tour(grid, deadline, pivot_penalty=pivot_penalty,
                 mine_cells=mine_cells, mine_turn_penalty=mine_turn_penalty,
                 barrier_turn_penalty=barrier_turn_penalty,
                 door_turn_penalty=door_turn_penalty,
                 edge_mine_turn_penalty=edge_mine_turn_penalty,
                 edge_mine_cells=edge_mine_cells,
                 obstacle_turn_penalty=obstacle_turn_penalty,
                 obstacle_cells=obstacle_cells, phys=phys)
    tour.build([start_cell] + mines_lower + mines_upper
               + [cross_lower_cell, cross_upper_cell, end_cell]
               + [_cell(b) for b in shoot_pool])

    lower_seq = tour.two_opt(tour.nearest(start_cell, mines_lower + [cross_lower_cell])) \
        if mines_lower else [cross_lower_cell]
    if cross_lower_cell not in lower_seq:              # 2-opt 不会丢点，纯防御
        lower_seq.append(cross_lower_cell)
    upper_nodes = mines_upper + [cross_upper_cell]
    upper_seq = tour.two_opt(tour.nearest(lower_seq[-1], upper_nodes)) if upper_nodes \
        else [cross_upper_cell]
    order = [start_cell] + lower_seq + upper_seq + [end_cell]

    heads = tour.best_headings(order, fixed={start_cell: start_heading})
    order, heads, _ = tour.polish(order, heads, rounds=MAX_TOUR_ROUNDS - 1)
    if order[0] != start_cell or order[-1] != end_cell:
        raise PlannerError("内部错误：优化后起点/终点被挪动")
    heads[0] = start_heading                          # 起点朝向 = 设计常量
    order_candidates: list[dict] = [{"order": list(order), "side": False}]
    # ★★ U 形单回路**硬约束**兜底候选 ——见 `_u_shape_order()` 的说明。
    #   只在候选 1（现有行为：polish 整条）**结构自检失败**时才会被用到 ⇒
    #   对已经合规的地图（n8 图等）零影响。
    #   ★ 两个变体都试：`nearest(start, …)` 与 `nearest(cross_lower, …)` 的下半场顺序
    #   在第三张图（K1+K11）上表现不同（前者编译后会多一次 `L11→K11→J11` 往返）。
    order_candidates.append({"order": _u_shape_order(tour, start_cell, end_cell,
                                          mines_lower, mines_upper,
                                          cross_lower_cell, cross_upper_cell),
                             "side": False})
    order_candidates.append({"order": _u_shape_order(tour, start_cell, end_cell,
                                          mines_lower, mines_upper,
                                          cross_lower_cell, cross_upper_cell,
                                          seed_from_cross=True),
                             "side": False})
    # ★ 任务优先口径的结构候选：每半场先东侧、穿洞一次、再西侧（治『随机图多穿一次』）。
    _side_cand = {"order": _side_shape_order(tour, start_cell, end_cell,
                                           mines_lower, mines_upper,
                                           cross_lower_cell, cross_upper_cell),
                  "side": True}
    if objective == OBJECTIVE_TASKS:
        order_candidates.insert(0, _side_cand)      # 新口径先试结构正确的候选（快）
    else:
        order_candidates.append(_side_cand)         # 旧口径：仅作末位兜底（零影响）

    # ---------------- 顺序候选（U 形单回路硬约束）----------------
    #   候选 1 = 现有行为（polish 整条）；候选 2 = U 形单回路（穿洞格钉在本半场边界）。
    #   依次编译并跑**结构自检**，取第一个通过的；两者都不通过 ⇒ 抛错（不静默降级）。
    #   ⇒ 对已经合规的地图（历史现图/n8 图）行为**逐字节不变**（候选 1 就通过，不会走到候选 2）。
    route: Route | None = None
    uturn_fixes = 0
    marked: list[tuple[int, str, float]] = []
    mine_runs: list[tuple[str, int]] = []
    mine_axes: list[tuple[str, int, bool]] = []
    problems: list[str] = []
    barrier_turns: list[str] = []
    for _ci, _cand in enumerate(order_candidates):
        _tour = _Tour(grid, deadline, pivot_penalty=pivot_penalty,
                      mine_cells=mine_cells, mine_turn_penalty=mine_turn_penalty,
                      barrier_turn_penalty=barrier_turn_penalty,
                      door_turn_penalty=door_turn_penalty,
                      edge_mine_turn_penalty=edge_mine_turn_penalty,
                      edge_mine_cells=edge_mine_cells,
                      obstacle_turn_penalty=obstacle_turn_penalty,
                      obstacle_cells=obstacle_cells, phys=phys)
        _tour.__dict__.update(tour.__dict__)            # 共享洪泛缓存（只读复用）
        _order = list(_cand["order"])
        _is_side = bool(_cand.get("side"))
        _heads = _tour.best_headings(_order, fixed={_order[0]: start_heading})

        # ---------------- 射击停站：主停站 + 对侧第二停站（★P0-2 双面覆盖） ----------------
        _shoot = ""
        _second = ""
        if shoot_pool:
            if _is_side:
                _order, _shoot = _insert_shoot_sided(
                    _tour, _order, shoot_pool, cross_lower_cell, cross_upper_cell)
            else:
                _order, _heads, _shoot = _insert_shoot(_tour, _order, _heads, shoot_pool)
            if _shoot:
                _second = second_shoot_stop(spec, grid, _shoot, _shoot_accept(cfg))
                if _second:
                    _order = _insert_second_sided(
                        _tour, _order, _second, cross_lower_cell, cross_upper_cell)
            _heads = _tour.best_headings(_order, fixed={_order[0]: start_heading})

        # ---------------- ★ 雷心直线穿越 ----------------
        _axes: list[tuple[str, int, bool]] = []
        if not allow_reverse:
            _order, _axes = straighten_mines(_order, spec, grid, _tour)
            _heads = _tour.best_headings(_order, fixed={_order[0]: start_heading})

        # ---------------- 编译成航点 ----------------
        _keep = {start_cell, end_cell, cross_lower_cell, cross_upper_cell}
        _keep |= set(mines_lower) | set(mines_upper)
        if _shoot:
            _keep.add(_cell(_shoot))
        _stops: list[_Stop] = [_Stop(start_cell[0], start_cell[1], spec.start,
                                     heading=start_heading, tag="start")]
        _cost = 0.0
        _bad_seg = ""
        for i2 in range(len(_order) - 1):
            a, b = _order[i2], _order[i2 + 1]
            path, seg_cost = _tour.path(a, b, _heads[i2], _heads[i2 + 1])
            if not path:
                _bad_seg = (f"{geo.index_to_block(a[0] - 1, a[1])} -> "
                            f"{geo.index_to_block(b[0] - 1, b[1])}")
                break
            _cost += seg_cost
            for (row, col, face, back) in _collapse(path, _keep):
                if (row, col) == (_stops[-1].row, _stops[-1].col):
                    _stops[-1].reverse = back
                    _stops[-1].heading = face
                    continue
                _blk = geo.index_to_block(row - 1, col)
                _stops.append(_Stop(row, col, _blk, heading=face, reverse=back,
                                    tag="mine" if _blk in set(spec.mines) else "path"))
            if time.perf_counter() > deadline:
                if objective == OBJECTIVE_TASKS:
                    timed_out = True
                    _bad_seg = "超时"
                    warnings.append(f"规划超时（>{max_seconds:.2f} s）→ 降级兜底")
                    break
                raise PlannerError(f"规划超时（>{max_seconds:.2f} s），未生成完整路线")
        if _bad_seg:
            if _ci >= len(order_candidates) - 1:
                if objective == OBJECTIVE_TASKS:
                    warnings.append(f"顺序候选均无可行段：{_bad_seg}")
                    continue
                raise PlannerError(f"无解：{_bad_seg}")
            continue
        if (_stops[0].row, _stops[0].col) != start_cell:
            raise PlannerError("内部错误：路线首点不是起点")
        if (_stops[-1].row, _stops[-1].col) != end_cell:
            raise PlannerError("内部错误：路线末点不是终点")

        # ---------------- 长段插点 + 标记 + 180° 折返改写 ----------------
        _stops = _split_long_hops(_stops, grid)
        for _st in _stops:
            if (_st.row, _st.col) == cross_lower_cell:
                _st.tag = "lower_bridge"
            elif (_st.row, _st.col) == cross_upper_cell:
                _st.tag = "upper_bridge"
        if _shoot:
            for _st in _stops:
                if _st.block == _shoot:
                    _st.shoot = True
                    _st.tag = "shoot"
                    break
            else:                                   # pragma: no cover - 防御性
                raise PlannerError(f"内部错误：射击点 {_shoot} 不在编译后的路线里")

        _fixes = _fix_uturn_reverse(_stops) if allow_reverse else 0
        _stops = _merge_duplicate_stops(_stops)
        _refresh_headings(_stops)
        _marked: list[tuple[int, str, float]] = []
        if not allow_reverse:
            _marked = mark_pivots(_stops)
        _route = _to_route(_stops, name=name,
                           crossing={"lower": lower_cross, "upper": upper_cross},
                           final_heading=_final_heading(_stops))
        problems = check_route_structure(_route, spec, grid, limits, _shoot_accept(cfg),
                                         already_covered=tuple(missing_mines))
        # ★★【硬校验】隔板相邻行的"侧向 90° 转向"—— 与结构自检同层：
        #   只在 `barrier_turn_penalty > 0`（显式启用）时生效；默认 0.0 ⇒ 恒空、零影响。
        _bturns: list[str] = []
        if barrier_turn_penalty > 0.0 and not problems:
            _bturns = _barrier_turn_stops(grid, _stops)
            if _bturns:
                problems = ["隔板相邻行仍有『侧向 90° 转向』：" + " ".join(_bturns)
                            + "（第 %d 行贴隔板那一侧的走廊净宽只有半格 %.3f m，"
                              "车身在行内做侧向 90° 转向的横向摆动是转弯半径量级 —— "
                              "实测会把车甩进隔板）"
                            % (spec.barrier_row_lo, 0.5 * geo.BLOCK_SIZE)]
        if problems and _ci < len(order_candidates) - 1:
            warnings.append("顺序候选 #%d 未过结构自检（%s）→ 换用其它顺序候选重试"
                            % (_ci + 1, "；".join(problems[:2])))
            continue
        if problems:
            if objective == OBJECTIVE_TASKS:
                warnings.append("所有顺序候选均未过结构自检（"
                                + "；".join(problems[:3]) + "）→ 降级兜底")
                break
            raise PlannerError("规划结果未通过结构自检：\n  - " + "\n  - ".join(problems))
        # ---- 该候选通过，固化为最终结果 ----
        tour, order, heads, stops, _ = _tour, _order, _heads, _stops, _shoot
        cost, uturn_fixes, marked, mine_axes = _cost, _fixes, _marked, _axes
        barrier_turns = _bturns
        route = _route
        break
    if route is None:
        if objective == OBJECTIVE_TASKS:
            reason = "超时" if timed_out else "结构（无可行顺序）"
            return _best_effort_result(
                spec, cfg, grid, name=name, source=source, t0=t0,
                allow_reverse=allow_reverse, reason=reason, warnings=warnings,
                deadline=deadline)
        raise PlannerError("内部错误：没有任何顺序候选被接受")   # pragma: no cover - 旧口径必抛

    # ★【自诊断】逐雷"雷前共线格数"：<2 的雷在纯追踪下会被切角，车心压不到雷心。
    #   只报一条汇总 warning（每个雷一行会把启动日志淹掉），数据同时进 `PlanResult.mine_runs`。
    if not allow_reverse:
        mine_runs = _mine_aligned_runs(stops, spec)
        short = [(b, r) for b, r in mine_runs if r < 2]
        if short:
            warnings.append(
                "雷心压过风险：%d/%d 个雷的进点前共线不足 2 格（%s）—— "
                "纯追踪要 ~2 格（0.7 m）才贴回参考线，这些雷会被切角，"
                "车心到雷心实测可差 3~8 cm。"
                "要真正消除需要改访问顺序（会额外增加 PIVOT/里程），"
                "当前按『不动路线』处理，仅提示。"
                % (len(short), len(mine_runs),
                   " ".join("%s=%d" % (b, r) for b, r in short)))
    if allow_reverse:
        assert_legal(route)
    else:
        assert_legal_no_reverse(route)

    crossing = {"lower": lower_cross, "upper": upper_cross}
    tokens = route_tokens(route)
    shoot_at = next((wp.block for _, wp in route.shoot_points()), "")
    shoot_dist = 0.0
    shoot_targets: tuple[str, ...] = ()
    if shoot_at:
        # 记录的是**到靶板中心**的距离（与 `shoot_candidates()` 同源口径），
        #   不再是"到最近靶格中心"。板心 = 各靶格中心之算术中点（`plate_center_m`）。
        acc_lo, acc_hi = shoot_distance_window(_shoot_accept(cfg))
        _cx, _cy = plate_center_m(spec)
        _sx, _sy = geo.block_to_meters(shoot_at)
        shoot_dist = math.hypot(_sx - _cx, _sy - _cy)
        shoot_targets = tuple(t for t in spec.targets
                              if acc_lo - 1e-9 <= stop_distance_m(shoot_at, t) <= acc_hi + 1e-9)
    polyline = sum(math.hypot(route.waypoints[i + 1].x - route.waypoints[i].x,
                              route.waypoints[i + 1].y - route.waypoints[i].y)
                   for i in range(len(route.waypoints) - 1))

    # ---------------- 任务向量 + 降级标记（★TRACK-PLAN-B1 新口径） ----------------
    _blocks = set(route.blocks())
    tasks_mines = tuple(str(m) for m in spec.mines
                        if str(m).strip().upper() in _blocks)
    tasks_stops = tuple(wp.block for _i, wp in route.shoot_points())
    _lost = [str(m) for m in spec.mines if str(m).strip().upper() not in _blocks]
    _reasons: list[str] = []
    if _lost:
        _reasons.append("缺雷 " + " ".join(_lost))
    if spec.targets and not tasks_stops:
        _reasons.append("缺停站")
    degraded = bool(_reasons)
    degraded_reason = "；".join(_reasons)

    return PlanResult(
        route=route, tokens=tokens, stops=tuple(stops), crossing=crossing,
        shoot_stop=shoot_at, shoot_distance_m=float(shoot_dist),
        shoot_targets=shoot_targets, cost=cost, expanded=tour.settled,
        elapsed_ms=(time.perf_counter() - t0) * 1000.0,
        reverse_segments=sum(1 for wp in route.waypoints if wp.reverse),
        uturn_fixes=uturn_fixes, warnings=tuple(warnings), map_name=spec.name,
        map_source=spec.source, start=spec.start, end=spec.end,
        polyline_m=polyline, start_heading=HEAD_NAMES[start_heading],
        allow_reverse=bool(allow_reverse), pivots=tuple(marked),
        pivot_penalty=float(pivot_penalty),
        mine_turn_penalty=float(mine_turn_penalty), mine_axes=tuple(mine_axes),
        mine_runs=tuple(mine_runs),
        barrier_turn_penalty=float(barrier_turn_penalty),
        barrier_turns=tuple(barrier_turns),
        tour_cost=tour.order_cost_total(order),
        tasks_mines=tasks_mines, tasks_stops=tasks_stops,
        tasks_mines_total=len(spec.mines),
        degraded=degraded, degraded_reason=degraded_reason,
    )


def _insert_shoot(tour: _Tour, order: list[tuple[int, int]], heads: Sequence[int],
                  pool: Sequence[str]) -> tuple[list[tuple[int, int]], list[int], str]:
    """把射击停站插到"绕路代价最小"的位置，返回 (新顺序, 新朝向, 选中的射击格)。

    绕路代价 = `m(prev,stop) + m(stop,next) - m(prev,next)`（不新增任何阈值），
    再加上**折返惩罚的增量**（插入点可能把一处顺路访问变成"进去再退出来"），
    然后用朝向轮重算一次朝向。平局用 (绕路代价, 插入位置, 格名) 全序决定 —— **确定性**。
    """
    positions = range(1, len(order))
    best: tuple[float, int, str] | None = None
    base_penalty = tour.penalty_of(order)
    for block in pool:
        cell = _cell(block)
        if cell in order:
            detour, pos = 0.0, order.index(cell)
        else:
            scored = []
            for i in positions:
                prev_cell, next_cell = order[i - 1], order[i]
                value = (tour.order_cost.get((prev_cell, cell), float("inf"))
                         + tour.order_cost.get((cell, next_cell), float("inf"))
                         - tour.order_cost.get((prev_cell, next_cell), 0.0))
                if math.isfinite(value) and tour.pivot_penalty > 0.0:
                    trial = list(order)
                    trial.insert(i, cell)
                    value += tour.penalty_of(trial) - base_penalty
                scored.append((value, i))
            detour, pos = min(scored, key=lambda item: (item[0], item[1]))
        if not math.isfinite(detour):
            continue
        key = (detour, pos, block)
        if best is None or key < best:
            best = key
    if best is None:
        raise PlannerError("没有可用的射击停站点（全部绕路代价无穷）")

    _, pos, block = best
    cell = _cell(block)
    new_order = list(order)
    if cell not in new_order:
        new_order.insert(pos, cell)
    new_heads = tour.best_headings(new_order, seed=None, fixed={new_order[0]: heads[0]})
    return new_order, new_heads, block


# --------------------------------------------------------------------------- #
#  ★ 雷心直线穿越（用户 2026-10-11 口径：车心必须压过每个雷格中心）
# --------------------------------------------------------------------------- #
def _axis_pass_ok(grid: Grid, cell: tuple[int, int], d: tuple[int, int]
                  ) -> tuple[bool, tuple[int, int], tuple[int, int]]:
    """沿方向 `d` 能否做"进点—雷—出点"直线穿越，返回 (可行, 进点, 出点)。

    三条硬约束（每条都对应一条既有规则，不新造判据）：
      ① 进/出点必须可通行（障碍不可通行）；
      ② 进/出点**不能在 K 列**：否则会平白多一次 K 列穿行（R007 要求每半场恰好一次）；
      ③ 进/出两步不得非法跨越第 6/7 行隔板（只有 A/B/C 列能过，R003）。

    ★ 曾经还要求"雷前第 2 格（起跑点）也通行"并把它插进航点序列，实测**更差**：
      A* 会先绕到起跑点再折回，PIVOT 从 1 个涨到 5 个、里程 19.95→23.45 m。
      车心到雷心的真值偏差改用"速度/前视"扫描来压（取证报告 §A3），不靠插起跑点。
    """
    m_in = (cell[0] - d[0], cell[1] - d[1])
    m_out = (cell[0] + d[0], cell[1] + d[1])
    for c in (m_in, m_out):
        if not grid.passable(*c):
            return False, m_in, m_out
        if c[1] == _CROSS_COL:
            return False, m_in, m_out
        if not grid.barrier_cross_ok(cell[0], c[0], c[1]):
            return False, m_in, m_out
    return True, m_in, m_out


def _dir_turn_cost(from_dir: int, to_dir: int) -> float:
    """朝向索引 `from_dir -> to_dir` 的转角代价：0° = 0，90° = 1，180° = 100（几乎禁止）。

    180° 之所以重罚：无倒车模式下它必然是一次原地 PIVOT（实测 ~10.8 s、横向扫出 ~0.54 m），
    而且 `pursuit.pivot_dist_m=0.25` 让车在距顶点 0.25 m 处就掉头 —— 只要 180° 落在
    雷格**旁边那一格**，同样会把雷心的压过精度毁掉。所以选轴时必须把"新插点处会不会
    出现折返"一并计进去。
    """
    delta = (int(from_dir) - int(to_dir)) % 4
    if delta == 0:
        return 0.0
    if delta == 2:
        return 100.0
    return 1.0


def _mine_aligned_runs(stops: Sequence["_Stop"], spec: MapSpec) -> list[tuple[str, int]]:
    """【自诊断】逐雷量"雷格前有几格是共线的"（编译后路线口径，单位 = 格）。

    口径：从雷格往前数，**方向不变**的连续格数（含紧邻的那一格）。
    `1` = 转向点就压在雷前那一格上 —— 纯追踪此时还在切角，车心压不到雷心；
    `>= 2` = 车进雷格之前已经有 ~0.7 m 直线，够贴回参考线。

    为什么放在规划器里：这条是**真车丢分的直接原因**（漏压雷 = 直接丢分），
    但以前只有跑完仿真才知道（§3.5 / §A3 都是事后测量）。现在规划器自己
    把风险报出来（`PlanResult.mine_runs` + 一条 warning），新地图不用先上车就能看出
    哪几个雷压不实。判据本身不发明阈值：`MINE_PRESS_TOL_M` 与实测口径一致。
    """
    cells = [(stop.row, stop.col) for stop in stops]
    out: list[tuple[str, int]] = []
    for name in spec.mines:
        cell = _cell(name)
        if cell not in cells:
            out.append((name, 0))
            continue
        idx = cells.index(cell)
        if idx == 0 or idx >= len(cells) - 1:
            out.append((name, 0))
            continue
        step_in = (cell[0] - cells[idx - 1][0], cell[1] - cells[idx - 1][1])
        step_out = (cells[idx + 1][0] - cell[0], cells[idx + 1][1] - cell[1])
        if step_in != step_out or step_in == (0, 0):
            out.append((name, 0))          # 雷格是拐点（不是直线穿越）→ 必然切角
            continue
        run = abs(step_in[0]) + abs(step_in[1])
        k = idx - 1
        while k > 0:
            prev = (cells[k][0] - cells[k - 1][0], cells[k][1] - cells[k - 1][1])
            if prev != step_in:
                break
            run += abs(prev[0]) + abs(prev[1])
            k -= 1
        out.append((name, run))
    return out


def straighten_mines(order: list[tuple[int, int]], spec: MapSpec, grid: Grid,
                     tour: "_Tour"
                     ) -> tuple[list[tuple[int, int]], list[tuple[str, int, bool]]]:
    """把每个雷格变成"进—雷—出"三点直线的**中点**，返回 (新顺序, 穿越轴清单)。

    为什么必须动顺序而不是只动编译：雷格上的转角是**拓扑**决定的 —— 只要雷格是
    访问顺序里的一个拐点，纯追踪就会在它上面切角（实测 2.7~3.1 cm）；若是 180°
    折返点，车更是在距顶点 0.25 m 处就掉头（`pursuit.pivot_dist_m`），车心压不到雷心
    （A9 实测差 12.7 cm）。这里在顺序里给雷格补上**进点**与**出点**（各离雷格一格、
    与雷格共线），于是编译出的折线在雷格处是**直线**：

        …, 进点, 雷格, 出点, …        （进点 → 雷格 → 出点 共线）

    轴的选择（确定性）：对每个可行轴算"进点处的转角 + 出点处的转角"（180° 记 100、
    90° 记 1、直行记 0，见 `_dir_turn_cost`），取代价最小者；平局取轴编号小者。
    三条硬约束见 `_axis_pass_ok()`。

    :return: ``(新顺序, [(雷格名, 轴编号(0北1西2南3东), 是否找到可行轴), …])``
        —— 找不到可行轴时**如实记 False**，由 `plan_route` 报 warning，绝不假装压到了。
    """
    mines = {_cell(m) for m in spec.mines}
    out: list[tuple[int, int]] = []
    axes: list[tuple[str, int, bool]] = []
    for idx, cell in enumerate(order):
        if cell not in mines:
            out.append(cell)
            continue
        block = geo.index_to_block(cell[0] - 1, cell[1])
        prev_cell = order[idx - 1] if idx > 0 else None
        next_cell = order[idx + 1] if idx + 1 < len(order) else None
        chosen: tuple[tuple[int, float, int], tuple[int, int], tuple[int, int]] | None = None
        for axis in range(4):
            ok, m_in, m_out = _axis_pass_ok(grid, cell, DIRS[axis])
            if not ok:
                continue
            # ① 这个轴会不会在进点/出点**制造 180° 折返**（新 PIVOT，必须优先避免）
            bad = 0
            if prev_cell is not None and prev_cell != m_in:
                tour._record_leg_dirs(prev_cell, m_in)                          # noqa: SLF001
                arrive = tour._entry_dir.get((prev_cell, m_in))                 # noqa: SLF001
                if arrive is not None and (int(arrive) - axis) % 4 == 2:
                    bad = 1
            if next_cell is not None and next_cell != m_out:
                tour._record_leg_dirs(m_out, next_cell)                         # noqa: SLF001
                depart = tour._exit_dir.get((m_out, next_cell))                 # noqa: SLF001
                if depart is not None and (axis - int(depart)) % 4 == 2:
                    bad = 1
            # ② 同分时比**实际路径代价**（进点→雷格→出点 各一格 = 2 格，可忽略不计），
            #    这把"绕远路去压雷"和"走近路但要多打一次舵"放在同一把尺子上量。
            cost = 0.0
            if prev_cell is not None and prev_cell != m_in:
                cost += tour.leg_cost(prev_cell, m_in)
            if next_cell is not None and next_cell != m_out:
                cost += tour.leg_cost(m_out, next_cell)
            # ③ 按 (是否制造 180°, 实际代价, 轴编号) 取最优 —— 全序、确定性
            key = (bad, round(cost, 6), axis)
            if chosen is None or key < chosen[0]:
                chosen = (key, m_in, m_out)
        if chosen is None:
            axes.append((block, -1, False))
            out.append(cell)
            continue
        (_bad, _cost, axis), m_in, m_out = chosen
        # 进点 → 雷格 → 出点（三点共线）
        if m_in != prev_cell and (not out or out[-1] != m_in):
            out.append(m_in)
        out.append(cell)
        if m_out != next_cell:
            out.append(m_out)
        axes.append((block, axis, True))
    # 去重相邻重复点（插点可能与原顺序里的邻居重合）
    dedup: list[tuple[int, int]] = []
    for c in out:
        if not dedup or dedup[-1] != c:
            dedup.append(c)
    return dedup, axes


# =========================================================================== #
#  **tracker-safe 引擎** —— "自动规划追平现役图路线（44 token）"
# --------------------------------------------------------------------------- #
#  为什么另起一个引擎（实测的差距根因，全部有数字）：
#   ① **原地转向（PIVOT）不可仿真**：旧引擎用 180° 折返 + `:PIVOT` 处理死角，
#      而 `tools/sim.py` 与真车循迹层都**不建模 PIVOT** ⇒ 真值轨迹只能画一个大弧兜回
#      去：实测峰值横向误差可达 **0.4292 m**（段 `D8→F8`，90° 折返）。
#   ② **紧凑 U 形折返会被整段切掉**：两条反向腿之间只隔 1 格时（如
#      `D10→E10→E11→D11→C11`），前视 0.36~0.40 m ≥ 单格 0.35 m ⇒ 车在 `E10` 就已经
#      瞄到 `D11` 之后；实测车心离 `D11` 顶点最近只有 **0.240 m**（峰值 0.2467 m）。
#   ③ **雷在顶点被拐过去 = 漏压**：纯追踪在顶点切内，实测漏压 `B2`（0.0364 m，
#      阈值 0.035）/ `E10`（0.0800 m）。
#  ⇒ 本引擎的硬规则（全部可枚举、可复现）：
#       · 只走**单格**（正交），复用内置 `Grid.move_ok` 的隔板/K 列判据；
#       · **禁 180°**（转角 ≤ 90°）⇒ 路线里**不含 `:PIVOT`**；
#       · **禁紧凑 U 形折返**（反向腿之间只隔 1 格）；
#       · **雷必须直线穿心**（转角 ≤ 10°），"口袋雷"（正交下无法穿心，默认 `D8`）
#         允许 90° 拐过并给重罚 —— 现场手工路线正是这么压 `D8` 的（实测 9.2 mm）；
#       · 净空用**物理口径**（障碍段 ≥ 0.215 / 拐角顶点 ≥ 0.3029、靶板 ≥ 0.0675 / 0.1554、
#         挡板细条 ≥ 0.0775）。
#  搜索：朝向感知 Dijkstra（状态 = 格 × 本腿方向 × 上一腿方向，后者用于判紧凑 U）
#        + 分阶段顺序 DP（下半天雷 → 隔板过渡格 → 上半天雷+射击点 → 上桥洞 → 终点），
#        代价 = **实走长度 + 转角权重 + 雷转角重罚**（不掺"绕远省钱"的软偏好）。
# =========================================================================== #

#: 转角权重（米/90°）：让搜索在同长度下偏好"少打舵"的走法
TS_TURN_W = 0.10
#: 雷格"被拐过去"的重罚（米/90°）：强烈偏好直线穿心，只在口袋雷上让位
TS_MINE_TURN_PENALTY_M = 1.5
#: 单格移动集（正交）—— 与现场手工路线同族（实测：加入对角/跃迁会让峰值误差涨到 0.24 m）
TS_ORTHO = ((1, 0), (-1, 0), (0, 1), (0, -1))
#: 顶点转角上限（> 该值即禁）：90° = 禁 180° 折返（真车靠 PIVOT，仿真不建模）
TS_MAX_TURN_DEG = 90.0
#: 雷"直线穿心"的转角容差（度）
TS_MINE_STRAIGHT_DEG = 10.0
#: 隔板过渡格（两半场之间的过渡航点）。第五张图：`C6` → **`A6`**。
#: 为什么换：`C6` 相位的解**在 C9 撞障碍**（FootprintObs 真值足迹净空 −0.00013 m；
#: `tools/route_check.py` 的折线口径在该处乐观、判"0.350 ≥ 0.215"）；
#: `A6`（与 `B6` 数值完全一致）的解双档全绿（0 碰撞 / 0 硬违规 / 越界 0 / K 穿越 2 次）。
#: 证据：历史报告 §3/§4（24 组 phase×turned 矩阵 + `diag_collision.py` 定位）。
TS_PHASE_CELL = "A6"
#: "口袋雷"：正交移动集下无法直线穿心，允许 90° 拐过。
#: 第五张图：加入 **`A4`** —— 新图上 `A4` 是口袋雷（西侧出界、南侧 `A5` 是障碍 ⇒ 正交移动集
#: 下不可能直线穿心）；不加它 `plan_route_tracker_safe()` 直接 `PlannerError: 无解`
#: （车上自规划失败 → 回退现役手工路线 → 新图上 `config_selfcheck` 12 error、不可发车）。
#: `D8` 仍在新图雷位里，保留。证据：历史报告 §5（24 组矩阵：不含 A4 的组合全无解）。
#: ★★ spec 增补 §H：加入 **`I6`** —— 把"桥洞入口邻格横向穿行"接成硬失败后，`I6`
#:   的唯一直线穿心走法（`H6→I6→J6` 东西向）在 `J6` 死端（`J6→J5` 横向进洞已禁、`K6` 竖向
#:   禁、`J7` 跨板禁），南北向又止于隔板 ⇒ `I6` 变成"准口袋雷"：只能带罚 90° 拐过
#:   （`H6→I6→I5→J5→K5`，其中 `I5→J5→K5` 恰是实测 **0 拍**的沿轴进洞形态）。
#:   不加它：硬约束下全图 `PlannerError: 无解`（二分定位见 `_tmp_session/t13_bisect.py`）。
#:   盖雷有效性由仿真验证（盖雷 8/8、桥墙 0 拍 —— 见台账）。
TS_TURNED_MINES = ("A4", "D8", "I6")
#: 顺序 DP 每个节点保留的**到达状态上限**（Pareto 截断；`None` = 不截断）。
#: 依据：`_ts_phase` 的到达前沿在现役图实测 **max=2**（16 次调用：前沿 1 出现 8 次、
#: 2 出现 8 次）⇒ 本图恒不触发、是等价变换；它只是给"到达状态更多"的图兜一个上限。
#: A/B：`keep=2` 与不截断在现役图/异地图上**逐候选产物逐 token 相同**（历史证据目录）。
TS_PARETO_KEEP = 2
#: tracker-safe 引擎的默认时间预算（秒）。这是**签名兜底值**，车上的真源是#: `config/robot.yaml: planner.tracker_max_seconds`（两者同值，且 `tools/plan_route.py`
#: 的 `--max-seconds` 缺省直接引用本常量 ⇒ 改一处三处同步）。
#: ★ **ARM 车端实测**（RK3588S，8 核，现场 `~/dzrobot` 同一份地图/配置）——
#:   提速前 6515/6693/6752 ms（settled 470118，`_ts_phase` 16 次）；
#:   提速后 **2788/2788/2786 ms**（settled 200811，`_ts_phase` 5 次）。
#:   取值规则 = max(实测中位 × 3, 实测中位 + 5 s) = max(8.36, 7.79) ⇒ **10.0 s**。
#:   （旧值 5.0 s 是 PC 实测 1.5 s 年代的；第七趟实车就是被这个 5 s 卡住而回退手工路线。）
TS_MAX_SECONDS = 10.0

#: 桥洞**进洞侧冗余墙**的兜底长度（米）= **1/4 格** = 0.35 × 0.25 = 0.0875。
#: 语义与几何见 `physics.bridge_entry_walls()`：把桥室南北壁沿 y 向进洞侧虚拟延长这段长度，
#: 逼车"进洞前 1/4 格内已经摆正"。**这段墙现实中不存在**（设计冗余）。
#: 真源是 `config/robot.yaml: field.bridge_entry_redundancy_m`（本常量只在缺 config 时兜底，
#: 与 `planner.obstacle_margin_m` / `barrier_width_m` 等键同一套"签名兜底值"惯例）。
BRIDGE_ENTRY_REDUNDANCY_M = 0.0875
#: ★ 判据容差（米）：间隙必须 **> 本值**才算通过。
#: ⚠️ 为什么必须严格大于 0 而不是 `>= 0`：本模块的距离口径是"**重叠 = 0**"
#:   （`physics._seg_seg_distance` / `polygon_distance_m` 都这么写）⇒ 写成 `>= 0` 会
#:   **恒真**、判据形同虚设（落地时实测踩过：`>= 0` 版本对旧路线逐 token 不变，
#:   即判据从未生效）。取 1e-9 与既有代码（`< req - 1e-12`）同量级的数值卫生。
BRIDGE_ENTRY_CLEAR_EPS = 1e-9
#: 段级/顶点级判据沿"进洞那一步"的**采样点数**（含两端点）。7 点 ⇒ 步长 = 0.35/6 ≈ 58 mm，
#: 远小于车宽 135 mm（见 `_TSGrid._entry_sweep_ok` 关于"离散化不是解析最小"的说明）。
BRIDGE_ENTRY_SAMPLES = 7

#: **桥洞进洞前对正**惩罚（米；与 `_ts_leg` 的代价同单位）。
#: 触发形态：转弯顶点 = **桥洞格的前一格**，且该转弯的 `d_out` **直接通向桥洞格**
#:   ⇒ 就是"洞口前 1 格 90° 拐弯"，按本值加罚，驱动 A* 把拐弯顶点前移 1 格。
#:
#: 为什么需要（判定性结论，`docs/仿真复验_桥洞冗余墙_t5.md` §4）：
#:   早期的 `entry_gate_*` 把车头**恒取段方向**（= 假设进洞时已摆正），所以对"进洞前还在
#:   转弯"的路线判绿；但真车 `R_min = wheelbase/tan(max_steer) = 0.157/tan30° = 0.2719 m`
#:   **大于**洞口可用带宽 0.175 m ⇒ 在洞口前 1 格起转的车，进洞瞬间车身必然还是斜的
#:   （实测摆正误差 38~52°），车体角点斜插冗余带，最大穿透 **56.7 mm@K9 / 44.4 mm@K5**。
#:   实测对照：现役形态（洞口前 1 格拐弯）**246 拍**；拐弯顶点前移 1 格
#:   （`M10→M9→L9→K9`，对正段 2 格 = 0.70 m ≥ R_min）降到 **6 拍**。
#:
#: 为什么用**惩罚**而不是硬禁：惩罚**不可能让图无解** —— 真没有替代走法时仍出路线
#:   （只带罚值），而硬禁会让"进洞前一格恰是唯一通路"的图直接 `PlannerError`。
#:   本图上替代走法与原走法**等长等转角**（`M10→L10→L9→K9` vs `M10→M9→L9→K9`，都是
#:   3 格 + 1 个 90°），⇒ 任何正罚值都足以翻转选择；取 2.0 m（≈5.7 格）留足余量，
#:   与 `TS_MINE_TURN_PENALTY_M`(1.5) / `PIVOT_PENALTY`(5.0) 同量级。
#: 真源 = `config/robot.yaml: planner.bridge_approach_align_penalty_m`（本常量是签名兜底值）。
BRIDGE_APPROACH_ALIGN_PENALTY_M = 2.0


def _ts_unit(v: tuple[int, int]) -> tuple[int, int]:
    """方向向量约到最简（本引擎只有单格移动，直接返回）。"""
    g = math.gcd(abs(v[0]), abs(v[1])) or 1
    return (v[0] // g, v[1] // g)


def _ts_map_yaw(d: tuple[int, int]) -> float:
    """栅格步方向 `(drow, dcol)` → **地图 yaw**（车头指向的弧度）。

    `x = (10.5−row)·0.35`、`y = (13.5−col)·0.35` ⇒ 一步 `(drow,dcol)` 的米制位移是
    `(−drow, −dcol)·0.35`，故 `yaw = atan2(−dcol, −drow)`。自检：
    `DIRS[1]=(0,-1)`（西/+y）→ `+π/2 = HEADING_Y`；`DIRS[0]=(-1,0)`（北/+x）→ `0 = HEADING_X`。
    """
    return math.atan2(-d[1], -d[0])


def _ts_turn_deg(a: tuple[int, int], b: tuple[int, int]) -> float:
    """两个行进方向之间的转角（度，∈ [0,180]）。"""
    return abs(math.degrees(geo.normalize_angle(math.atan2(b[0], b[1])
                                                - math.atan2(a[0], a[1]))))


@dataclass
class _TSGrid:
    """tracker-safe 引擎的栅格：复用内置 `Grid`（通行/隔板/K 列）+ 物理净空 + 转角规则。

    与内置 `Grid.move_ok` 的分工：**段级**合法性（能不能走这一步）继续由它负责
    （它已经把"隔板只有 A/B/C 能上下""K 列只能水平进出通格""靶格不可通行"写死了）；
    本类只补三样它没有的：**物理净空**（障碍/靶板/挡板细条）、**转角规则**
    （禁 180°、禁紧凑 U、雷穿心）、以及转角/雷的**代价**。
    """

    spec: MapSpec
    grid: Grid
    phys: dict
    turn_w: float = TS_TURN_W
    mine_turn_penalty_m: float = TS_MINE_TURN_PENALTY_M
    turned_mines: tuple[str, ...] = TS_TURNED_MINES
    moves: tuple[tuple[int, int], ...] = TS_ORTHO

    def __post_init__(self) -> None:
        self.mine_cells = {_cell(m) for m in self.spec.mines}
        self.straight_mines = {c for c in self.mine_cells
                               if geo.index_to_block(c[0] - 1, c[1]) not in set(self.turned_mines)}
        self.obs_xy = {o: geo.block_to_meters(o) for o in self.spec.obstacles}
        self.tgt_xy = {t: geo.block_to_meters(t) for t in self.spec.targets}
        self.obs_r = float(self.phys["obstacle_radius_m"])
        self.width_m = float(self.phys["width_m"])
        self.length_m = float(self.phys["length_m"])
        # ★ spec 增补 §H.2：净空余量只能**加**安全，不能**减**——负余量会把
        #   required_obstacle_center_distance_m 压到物理重叠边界之下（甚至负数 ⇒ 判据
        #   恒真），等于关掉障碍净空检查。消费口钳位 ≥ 0，配置/直传都封死这条放宽向量。
        self.mgn = max(0.0, float(self.phys["obstacle_margin_m"]))
        sweep, _r = _phys.sweep_radius(length_m=self.length_m, width_m=self.width_m,
                                       wheelbase_m=self.phys["wheelbase_m"],
                                       max_steer_deg=self.phys["max_steer_deg"])
        self.obs_seg_req = _phys.required_obstacle_center_distance_m(
            obstacle_radius_m=self.obs_r, body_radius_m=self.width_m / 2.0, margin_m=self.mgn)
        self.obs_corner_req = _phys.required_obstacle_center_distance_m(
            obstacle_radius_m=self.obs_r, body_radius_m=sweep, margin_m=self.mgn)
        self.board_seg_req = _phys.required_target_body_clearance_m(
            length_m=self.length_m, width_m=self.width_m, at_corner=False)
        self.board_corner_req = _phys.required_target_body_clearance_m(
            length_m=self.length_m, width_m=self.width_m, at_corner=True)
        self.barrier_req = _phys.required_barrier_distance_m(
            barrier_width_m=float(self.phys["barrier_width_m"]), car_width_m=self.width_m)
        c0, c1 = _phys.parse_col_span(str(self.phys["barrier_cols"]))
        self.bar_x, self.bar_y0, self.bar_y1 = _phys.barrier_centerline(
            col_from=c0, col_to=c1, row_hi=self.spec.barrier_row_hi,
            row_lo=self.spec.barrier_row_lo)
        # ---------- 桥洞进洞侧冗余墙 ----------
        #   只对**本图两个穿洞格**建墙（`cross_lower`/`cross_upper` 由 map/route 决定），
        #   不把 K 列每格都当洞 —— K 列其它格根本不许横向进出（`Grid` 的 R007 已拦）。
        self.entry_redundancy_m = float(
            self.phys.get("bridge_entry_redundancy_m", BRIDGE_ENTRY_REDUNDANCY_M))
        _bridges = set(self.grid.bridge_rows)
        self.entry_walls: dict[tuple[int, int], tuple] = {}
        #: **进洞入口邻格** → 该格的桥洞格。
        #:   冗余墙把桥室南北壁向进洞侧延长，落点正好压进这个邻格里（K9⇒`L9`、K5⇒`J5`）
        #:   ⇒ 车在这个格里**横向**穿行/拐弯时余量只剩 ~2 cm，必然蹭墙（见
        #:   `entry_cell_crossed_sideways()` 的实测证据）。沿桥洞轴（y）穿行则余量 0.1075 m。
        self.entry_adjacent: dict[tuple[int, int], tuple[int, int]] = {}
        if self.entry_redundancy_m > 0.0:
            for _r in _bridges:
                _blk = geo.index_to_block(int(_r) - 1, _CROSS_COL)
                self.entry_walls[(_r, _CROSS_COL)] = _phys.bridge_entry_walls(
                    _blk, self.entry_redundancy_m)
                # 进洞方向由 row 决定（与 `physics.bridge_entry_walls()` 同一约定）：
                #   下半场（row_idx >= 6）向西穿 ⇒ 从**东侧**踏进（col+1）；
                #   上半场（row_idx < 6）向东穿 ⇒ 从**西侧**踏进（col-1）。
                _dy = 1 if (int(_r) - 1) >= _phys.BRIDGE_ENTRY_LOWER_ROW_IDX else -1
                self.entry_adjacent[(int(_r), _CROSS_COL + _dy)] = (int(_r), _CROSS_COL)
        self._legal: dict[tuple, bool] = {}
        self._turn: dict[tuple, tuple[bool, float, float]] = {}
        self.settled = 0

    # ---------------- 桥洞进洞侧冗余墙判据 ----------------
    def _entry_walls_of(self, cell: tuple[int, int]):
        """该格若是**桥洞格**则返回其进洞侧冗余墙（两段竖线段），否则 `None`。"""
        return self.entry_walls.get(cell)

    def _entry_clear_ok(self, px: float, py: float, yaw: float, walls) -> bool:
        """车体（以 `(px,py,yaw)` 摆正）与冗余墙的间隙是否 **> 0**。

        ⚠️ 必须严格大于 0：`body_bridge_wall_clearance_m()` 的距离口径是"**重叠 = 0**"
        （与 `polygon_distance_m` / `_seg_seg_distance` 一致），返回值**恒 ≥ 0**
        ⇒ 写成 `>= 0` 会**恒真**、判据形同虚设（卡面写的就是 `>= 0`，落地时实测确认
        它是空判据，故按物理语义收严成 `> 0`；见 `BRIDGE_ENTRY_CLEAR_EPS` 的说明）。
        """
        if not walls:
            return True
        return _phys.body_bridge_wall_clearance_m(
            px, py, yaw, walls, length_m=self.length_m,
            width_m=self.width_m) > BRIDGE_ENTRY_CLEAR_EPS

    def _entry_sweep_ok(self, x0: float, y0: float, x1: float, y1: float,
                        yaw: float, walls) -> bool:
        """从 `(x0,y0)` 到 `(x1,y1)`（直线）**姿态恒定 `yaw`** 扫一遍，全程间隙 > 0？

        采样 = 沿线均匀 `BRIDGE_ENTRY_SAMPLES` 个位姿（含两端点）。
        ⚠️ 这是**离散化**而非解析最小：每个角点到固定线段的距离是 t 的**凸**函数，但
          "4 角点取 min" 之后不保证凸 ⇒ 极小值理论上可能落在两个采样点之间。取 7 个点
          （步长 = 0.35/6 ≈ 58 mm，远小于车宽 135 mm）足以覆盖本几何；且对本图的
          "沿 y 直进" 情形，极小值就在**端点**上（x 恒定 ⇒ 横向距离与 t 无关）。
        """
        if not walls:
            return True
        n = BRIDGE_ENTRY_SAMPLES
        for i in range(n):
            t = i / (n - 1)
            if not self._entry_clear_ok(x0 + t * (x1 - x0), y0 + t * (y1 - y0), yaw, walls):
                return False
        return True

    def entry_gate_segment(self, a: tuple[int, int], b: tuple[int, int]) -> bool:
        """段 `a→b` **水平直穿进入**某桥洞格时，沿途车身是否不碰该洞的进洞侧冗余墙。

        触发条件（两条同时成立才判，出洞侧一律不判 —— 用户只要进洞侧）：
          ① `b` 是桥洞格（`cross_lower`/`cross_upper` 之一，= `grid.bridge_rows` 的列 K 格）；
          ② `a` 与 `b` **同行**（`drow == 0`）⇒ 这就是"水平直穿进入"，
             与 `Grid.move_ok` 的"只在桥洞行进出 K 列"口径一致。

        ★ 为什么"出洞侧天然不判"：进洞是 `a` 在洞口外、`b` 是桥洞格（触发）；出洞是
          `a` 是桥洞格、`b` 在洞口外（`b` 不是桥洞格 ⇒ 不触发）。**不需要**额外判方向，
          且冗余墙本身只向进洞侧延伸（`physics.bridge_entry_walls()`），双重保证。

        车头一律按**段方向**摆正（`_ts_map_yaw`）—— 判据问的就是"沿段方向摆正的车体"。
        """
        walls = self._entry_walls_of(b)
        if not walls or a[0] != b[0]:
            return True
        yaw = _ts_map_yaw((b[0] - a[0], b[1] - a[1]))
        ax, ay = geo.block_to_meters(geo.index_to_block(a[0] - 1, a[1]))
        bx, by = geo.block_to_meters(geo.index_to_block(b[0] - 1, b[1]))
        return self._entry_sweep_ok(ax, ay, bx, by, yaw, walls)

    def entry_gate_turn(self, cell: tuple[int, int], d_out: tuple[int, int]) -> bool:
        """顶点 `cell` 处**下一步进入桥洞格**时，转弯扫掠车身是否不碰冗余墙。

        触发条件：`cell + d_out` 是桥洞格（即"这一转向把车带进洞"）。
        判据用**顶点处的实际位姿**：车头取 `d_out`（进洞方向）—— 顶点转向时车身绕
        顶点转，取"出向已摆正"是**保守**的一侧（车正对洞口时车身横向占用 = 车宽；
        取"入向"则横向占用 = 车长，反而更松）。沿途取点同 `entry_gate_segment`。
        """
        nxt = (cell[0] + d_out[0], cell[1] + d_out[1])
        walls = self._entry_walls_of(nxt)
        if not walls:
            return True
        yaw = _ts_map_yaw(d_out)
        cx, cy = geo.block_to_meters(geo.index_to_block(cell[0] - 1, cell[1]))
        nx, ny = geo.block_to_meters(geo.index_to_block(nxt[0] - 1, nxt[1]))
        return self._entry_sweep_ok(cx, cy, nx, ny, yaw, walls)

    # ---------------- 进洞前"对正段"约束 ----------------
    def bridge_approach_penalty(self, cell: tuple[int, int], d_out: tuple[int, int]) -> float:
        """顶点 `cell` 的转弯**把车直接带进桥洞格**时，返回进洞前对正惩罚（米）。

        触发条件（两条同时成立）：`cell + d_out` 是桥洞格（本转弯即进洞那一步）。
        ⇒ 顶点 `cell` 就是"桥洞格的前一格"，车在离洞口 **1 格（0.35 m）** 处做这次转弯。

        ★ 为什么这个形态必须罚（判定性结论）：
          真车最小转弯半径 `R_min = 0.157/tan(30°) = 0.2719 m`。在距洞口 0.35 m 处起转，
          车沿圆弧**只能横向走完 0.272 m 才转正**，而洞口可用带宽只有半格 0.175 m
          ⇒ 车体进入冗余带时必然还是斜的（实测摆正误差 38~52°，最大穿透 56.7 mm）。
          把转弯顶点前移 1 格后，进洞前有 **2 格 = 0.70 m > R_min** 的纯直线段
          ⇒ 进洞瞬间已经摆正（夹具 `M10→M9→L9→K9`：246 → **6** 拍）。

        ★ 为什么返回"惩罚"而不是"禁"：见 `BRIDGE_APPROACH_ALIGN_PENALTY_M` 的说明 ——
          惩罚**不可能造成无解**，这对"只换地图就自动生成路线"的目标是硬要求。

        :return: 惩罚值（米）；不触发时 `0.0`。
        """
        nxt = (cell[0] + d_out[0], cell[1] + d_out[1])
        if self._entry_walls_of(nxt) is None:
            return 0.0
        return float(self.phys.get("bridge_approach_align_penalty_m",
                                   BRIDGE_APPROACH_ALIGN_PENALTY_M))

    def entry_cell_crossed_sideways(self, cell: tuple[int, int], d_in: tuple[int, int],
                                    d_out: tuple[int, int]) -> bool:
        """**桥洞入口邻格**（K9 的 `L9` / K5 的 `J5`）是否被**横向**（沿 x / 换行）穿行。

        ★ 这是实测挖到的**第二条独立判据**，也是 K5/K9 违规的共同真因（§5 + 实测）：
          冗余墙把桥室南北壁向进洞侧延长，落点正好压进**入口邻格**里 ——
          K9 的墙 `y∈[0.9625,1.05]` 落在 `L9` 格内、K5 的墙 `y∈[1.4000,1.4875]` 落在 `J5` 格内。
          于是车只要**横向**穿行该格（或在该格拐弯），车身与墙的余量只剩
          `0.175 − 0.135/2 − 0.175/2 ≈ 0.02 m`；而纯追踪的横向跟踪误差 RMS 约 0.05 m
          ⇒ **必然**蹭墙。
          反之，沿桥洞轴（y）穿行时余量是 `0.175 − 0.0675 = 0.1075 m` ≫ 0.05 m ⇒ 安全。

          实测印证（全部来自夹具/实测扫描，见 artifacts）：
            · `L10→L9→K9`（L9 横向进）= **124 拍**；`M10→M9→L9→K9`（L9 纵向进）= **0 拍**；
            · `J6→J5→K5`（J5 横向进）  = **122 拍**；`I5→J5→K5`（J5 纵向进）  = **0 拍**；
            · `H6→H5→I5→J5→K5`（J5 纵向进）= **3 拍**；`J4→I4→H4→H5→I5→J5→K5` 里
              `J6→J5→J4` 段（J5 被横向穿过）仍报 **120 拍** —— 说明"横向穿行"本身即违规，
              与"是不是进洞那一步"无关。

        :return: `True` = 该格被横向穿行（应罚）；`False` = 不触发或沿轴穿行。
        """
        if cell not in self.entry_adjacent:
            return False
        # 沿 x = 行号变化（drow != 0）；沿 y = 列号变化。任一端的行进方向沿 x 即算横向。
        return d_in[0] != 0 or d_out[0] != 0

    # ---------------- 段级 ----------------
    def move_ok(self, a: tuple[int, int], b: tuple[int, int]) -> bool:
        key = (a, b)
        hit = self._legal.get(key)
        if hit is not None:
            return hit
        ok = self._move_ok_raw(a, b)
        self._legal[key] = ok
        return ok

    def _move_ok_raw(self, a: tuple[int, int], b: tuple[int, int]) -> bool:
        if (b[0] - a[0], b[1] - a[1]) not in self.moves:
            return False
        if not self.grid.move_ok(a[0], a[1], b[0], b[1]):
            return False
        ax, ay = geo.block_to_meters(geo.index_to_block(a[0] - 1, a[1]))
        bx, by = geo.block_to_meters(geo.index_to_block(b[0] - 1, b[1]))
        if _phys.barrier_distance_segment_m(ax, ay, bx, by, x=self.bar_x,
                                            y0=self.bar_y0, y1=self.bar_y1) \
                < self.barrier_req - 1e-12:
            return False
        for ox, oy in self.obs_xy.values():
            if _phys.obstacle_clearance_segment_m(ax, ay, bx, by, ox, oy) \
                    < self.obs_seg_req - 1e-12:
                return False
        half = _phys.board_half_m()
        for cx, cy in self.tgt_xy.values():
            if _phys.segment_rect_distance_m(ax, ay, bx, by, cx, cy, half) \
                    < self.board_seg_req - 1e-12:
                return False
        # 把"水平直穿进入桥洞"的那一步也按**进洞侧冗余墙**判一次
        #   （只在进洞侧判；出洞侧的 `a` 是桥洞格、`b` 不是 ⇒ 天然不触发）。
        if not self.entry_gate_segment(a, b):
            return False
        # ★★ 用户硬约束 · spec 增补 §H：桥洞**入口邻格**（K9 的 L9 / K5 的 J5）
        #   禁止横向（行号变化）穿行。决定性实测（见 entry_cell_crossed_sideways 注释）：
        #   横向穿行该格 = 车身与戳进格内的冗余墙余量仅 ~0.02 m < 纯追踪横向误差
        #   RMS ~0.05 m ⇒ 122/124 拍必然蹭墙；沿桥洞轴（列向）穿行余量 0.1075 m ⇒ 0 拍。
        #   判据由实测落地但此前未接线；本处接成**硬失败** —— 硬约束优先级最高，
        #   不得为通过性/效率/路径长度放宽（现役路线 I6→J6→J5→K5 的 J6→J5 即此形态，
        #   修后自动改走 I5→J5→K5 纵向进洞，实测 0 拍）。冗余 0.0（设计关闭）时
        #   entry_adjacent 为空 ⇒ 本条 inert，其余实体照常硬。
        _d = (b[0] - a[0], b[1] - a[1])
        if self.entry_cell_crossed_sideways(a, _d, _d) \
                or self.entry_cell_crossed_sideways(b, _d, _d):
            return False
        return True

    # ---------------- 顶点级 ----------------
    def turn_gate(self, cell: tuple[int, int], d_prev: tuple[int, int] | None,
                  d_in: tuple[int, int], d_out: tuple[int, int]
                  ) -> tuple[bool, float, float]:
        """``(是否允许, 转角°, 附加代价)``。

        ★ 三条硬规则（每条都有实测依据，见本节顶部注释）：
          ① 转角 > 90° ⇒ 禁（180° 折返只能靠 PIVOT，仿真/真车循迹都不建模）；
          ② 上一腿与本腿**反向**且中间只隔 1 格 ⇒ 禁（紧凑 U，会被纯追踪整段切掉）；
          ③ 雷格上转角 > 10° ⇒ 禁（会被切角漏压）；仅"口袋雷"例外（罚 `mine_turn_penalty_m`）。
        """
        key = (cell, d_prev, d_in, d_out)
        hit = self._turn.get(key)
        if hit is not None:
            return hit
        deg = _ts_turn_deg(d_in, d_out)
        ok, extra = True, 0.0
        if deg > TS_MAX_TURN_DEG + 1e-9:
            ok = False
        if ok and d_prev is not None and deg > 45.0:
            if _ts_turn_deg(d_prev, d_in) > 45.0 and _ts_unit((-d_prev[0], -d_prev[1])) == _ts_unit(d_out):
                ok = False
        if ok and cell in self.mine_cells:
            if cell in self.straight_mines and deg > TS_MINE_STRAIGHT_DEG + 1e-9:
                ok = False
            else:
                extra += self.mine_turn_penalty_m * (deg / 90.0)
        if ok and deg > 10.0:
            x, y = geo.block_to_meters(geo.index_to_block(cell[0] - 1, cell[1]))
            for ox, oy in self.obs_xy.values():
                if _phys.obstacle_clearance_point_m(x, y, ox, oy) \
                        < self.obs_corner_req - 1e-12:
                    ok = False
                    break
            if ok:
                half = _phys.board_half_m()
                for cx, cy in self.tgt_xy.values():
                    if _phys.point_rect_distance_m(x, y, cx, cy, half) \
                            < self.board_corner_req - 1e-12:
                        ok = False
                        break
        # 顶点处"下一步进洞"⇒ 按**进洞侧冗余墙**判转弯扫掠车身
        #   （不限于 deg>10°：即使直着进洞，`entry_gate_segment` 已经判过同一件事，
        #     但顶点路径（`turn_gate` 是唯一能拦"进洞前那一次转向"的地方）必须独立判 ——
        #     否则 A* 可以"在顶点先转向摆平、下一步直着进洞"绕过段级判据）。
        if ok and not self.entry_gate_turn(cell, d_out):
            ok = False
        # ★★ 用户硬约束 · spec 增补 §H：隔板相邻行的"死胡同式 90° 转向"
        #   （N7/H7 形态：实车 178/175 拍触线，`_barrier_turn_cell` 的实测归因）从
        #   tour 侧的"软惩罚（默认 penalty=0 ⇒ 实际不生效）+ 事后报告（penalty>0 才跑）"
        #   升格为 tracker-safe 的**硬失败**。判据复用 `_barrier_turn_cell`（经验判据，
        #   同一函数两条链路不分叉）；`_barrier_turn_cell` 已在补上隔板列域
        #   （只对 barrier_col..N 列生效，A/B/C 缺口处的行内转向不误杀）。
        if ok:
            _h_in = _heading_of(d_in[0], d_in[1], -1)
            _h_out = _heading_of(d_out[0], d_out[1], -1)
            if _h_in >= 0 and _h_out >= 0 \
                    and _barrier_turn_cell(self.grid, cell[0], cell[1], _h_in, _h_out):
                ok = False
        # **进洞前 1 格做 90° 拐弯** ⇒ 加进洞对正惩罚，驱动 A* 把拐弯顶点前移 1 格。
        #   为什么要 `deg > 0` 这一条：本顶点若"直着进洞"（`d_in == d_out`，例如
        #   `M9→L9→K9` 里的 `L9`）就是**我们要的形状**（进洞前已有 ≥1 格直线对正段），
        #   绝不能罚；只有"进洞这一步本身还在转"（`d_in != d_out`）才是判红的那种形态。
        if ok and deg > 1e-9:
            extra += self.bridge_approach_penalty(cell, d_out)
        # ★★ 判据升格：**桥洞入口邻格被横向穿行**（含在该格拐弯）⇒ 硬否决。
        #   这是实测挖到的第二条独立根因：冗余墙向进洞侧延长后压进入口邻格（K9⇒L9、K5⇒J5），
        #   横向穿行时余量只剩 ~2 cm < 纯追踪 RMS 0.05 m ⇒ 必然蹭墙；沿轴穿行余量 0.1075 m。
        #   实测：`H6→I6→J6→J5→K5`（J5 横向进）129 拍 vs `…→I5→J5→K5`（J5 沿轴进）0 拍。
        #   按 spec 增补 §H 把它从 2.0 m 软罚升格为硬失败（段级 `_move_ok_raw` 同步
        #   接了同一条判据 —— 两层同源，直调 turn_gate 的语义也与搜索一致）。
        if ok and self.entry_cell_crossed_sideways(cell, d_in, d_out):
            ok = False
        out = (ok, deg, extra)
        self._turn[key] = out
        return out


def _ts_tight_reversals(cells: Sequence[tuple[int, int]]) -> list[int]:
    """扫描折线，列出"紧凑 U 形折返"的顶点下标（两条反向腿之间只隔 1 格）。"""
    bad: list[int] = []
    for i in range(1, len(cells) - 2):
        d1 = (cells[i][0] - cells[i - 1][0], cells[i][1] - cells[i - 1][1])
        d2 = (cells[i + 1][0] - cells[i][0], cells[i + 1][1] - cells[i][1])
        d3 = (cells[i + 2][0] - cells[i + 1][0], cells[i + 2][1] - cells[i + 1][1])
        if (_ts_turn_deg(_ts_unit(d1), _ts_unit(d2)) > 45.0
                and _ts_turn_deg(_ts_unit(d2), _ts_unit(d3)) > 45.0
                and _ts_unit((-d1[0], -d1[1])) == _ts_unit(d3)):
            bad.append(i)
    return bad


def _ts_leg(ts: _TSGrid, a: tuple[int, int], b: tuple[int, int],
            h_in: tuple[int, int], cache: dict,
            first_dir: tuple[int, int] | None = None) -> dict:
    """从 ``a``（进入方向 `h_in`）到 ``b`` 的全部**到达朝向** Pareto 腿。

    :param first_dir: 只允许**第一步**朝这个方向走（`None` = 不限）。
        「起点朝向 = 设计常量」的硬含义：车头朝 `+y` 时**不许在起点转向** ——
        第一步必须沿 `+y` 直行，否则等代价的"先右转 90° 再走"会在 tie-break 下胜出，
        产出一条"起步就掉头"的路线，那正是"为起点生成转向消解"。
    :return: ``{(到达方向, 上一腿方向): (cost, [cells...])}``；`cells` 不含起点、含终点。
    """
    key = (a, b, h_in, first_dir)
    if key in cache:
        return cache[key]
    start = (a, _ts_unit(h_in), None)
    dist: dict = {start: 0.0}
    came: dict = {}
    pq: list[tuple[float, tuple]] = [(0.0, start)]
    goal: dict = {}
    while pq:
        d, st = heapq.heappop(pq)
        if d > dist.get(st, float("inf")) + 1e-15:
            continue
        ts.settled += 1
        cell, h, h_prev = st
        if cell == b and st != start:
            goal[(h, h_prev)] = d
            continue
        for v in ts.moves:
            if first_dir is not None and st == start and _ts_unit(v) != first_dir:
                continue
            nxt = (cell[0] + v[0], cell[1] + v[1])
            if not ts.move_ok(cell, nxt):
                continue
            ok, deg, extra = ts.turn_gate(cell, h_prev, h, _ts_unit(v))
            if not ok:
                continue
            nd = d + geo.BLOCK_SIZE + ts.turn_w * (deg / 90.0) + extra
            nst = (nxt, _ts_unit(v), h)
            if nd < dist.get(nst, float("inf")) - 1e-15:
                dist[nst] = nd
                came[nst] = st
                heapq.heappush(pq, (nd, nst))
    out: dict = {}
    for (h, h_prev), d in goal.items():
        st = (b, h, h_prev)
        cells: list[tuple[int, int]] = []
        while st != start:
            cells.append(st[0])
            st = came[st]
        cells.reverse()
        out[(h, h_prev)] = (d, cells)
    cache[key] = out
    return out


def _ts_cut(out: dict, keep: int | None) -> dict:
    """Pareto 截断 —— 只保留代价最小的前 `keep` 个到达状态（`None` = 不截断）。

    ⚠️ 这是**启发式**（不保证与不截断时同解），因此只在"实测前沿本来就 ≤ keep"的图上
    才是恒等变换。本图的实测前沿见 `TS_PARETO_KEEP` 的注释。
    """
    if keep is None or keep <= 0 or len(out) <= keep:
        return out
    return dict(sorted(out.items(), key=lambda kv: kv[1][0])[:keep])


def _ts_phase(ts: _TSGrid, cache: dict, order: Sequence[tuple[int, int]],
              cur: tuple[int, int], hpair: tuple, last: tuple[int, int],
              deadline: float, *, keep: int | None = TS_PARETO_KEEP) -> list[tuple[float, list, tuple]]:
    """分阶段顺序 DP：从 `cur`（进入朝向 `hpair`）访问 `order` 全部，最后到 `last`。

    ★ 两处**不改产物**的提速（各自都有 A/B 证据，见 历史证据目录）：

    ① **记忆键去掉 `h_prev`**（`key = (mask, node, hp[0])`）。
       为什么精确等价：本 DP 往下的每一次 `_ts_leg(ts, node, target, h, cache)` 只读
       `h = hp[0]`，`h_prev` 在 `_ts_leg` 里被置成 `None`（腿的第一步不判"紧凑 U"），
       所以 `(mask, node, (h, hp1))` 与 `(mask, node, (h, hp2))` 的子问题**逐位相同**。
       旧写法把它们当两个键各算一遍 ⇒ 同一棵子树最多重复 4 次（实测 1.09~1.25×）。
       ⚠️ 返回字典的**键**仍是 `(h, h_prev)`（来自 `_ts_leg` 的到达状态）—— 那是上层
       挑选用的粒度，未改动，故父层拿到的键集合、迭代顺序、代价全部不变。
    ② **每个子问题只留前 `keep` 个到达状态**（Pareto 截断，`keep=None` 关闭）。
       本图上到达前沿 max=2（16 次调用：前沿 1 出现 8 次、2 出现 8 次）⇒ **恒不触发**；
       它的作用是给"到达状态更多"的图兜一个上限（见 `TS_PARETO_KEEP`）。
    """
    full = (1 << len(order)) - 1
    memo: dict = {}

    def go(mask: int, node: tuple[int, int], hp: tuple) -> dict:
        key = (mask, node, hp[0])
        hit = memo.get(key)
        if hit is not None:
            return hit
        h = hp[0]
        if mask == full:
            out = {hp2: (c, cells)
                   for hp2, (c, cells) in _ts_leg(ts, node, last, h, cache).items()}
            memo[key] = _ts_cut(out, keep)
            return memo[key]
        out = {}
        for j, target in enumerate(order):
            if mask & (1 << j):
                continue
            if time.perf_counter() > deadline:
                raise PlannerError("tracker-safe 规划超时（顺序搜索阶段）")
            for hp2, (c, cells) in _ts_leg(ts, node, target, h, cache).items():
                for hp3, (c2, cells2) in go(mask | (1 << j), target, hp2).items():
                    tot = c + c2
                    if hp3 not in out or tot < out[hp3][0] - 1e-12:
                        out[hp3] = (tot, cells + cells2)
        memo[key] = _ts_cut(out, keep)
        return memo[key]

    res = go(0, cur, hpair)
    return sorted(((c, cells, hp) for hp, (c, cells) in res.items()), key=lambda t: t[0])


def _ts_search_multi(ts: _TSGrid, spec: MapSpec, *,
                     cross_lower_cell: tuple[int, int],
                     cross_upper_cell: tuple[int, int],
                     shoot_cells: Sequence[tuple[int, int]],
                     phase_cell: tuple[int, int], start_heading: int,
                     deadline: float, collect: int = 24,
                     keep: int | None = TS_PARETO_KEEP,
                     ) -> dict[tuple[int, int], list[tuple[float, list, int]]]:
    """**多候选共享**的两阶段顺序搜索 —— `{射击点格: [(cost, cells, 紧凑U个数), …]}`。

    与原版"**逐候选各跑一遍 `_ts_search`**"的差别只有一处：把**候选无关**的部分
    （`legs0`：起点→下半场穿洞格；下半场 `_ts_phase`）提到候选循环外**只算一次**，
    候选循环里只剩"上半场 DP（雷 + 该候选射击点）→ 上半场穿洞格 → 终点"。

    * 为什么这样改：射击停站只改变**上半场 DP 的最后一个必访点**，其它一切与候选无关；
      原实现把 4 个候选的完整搜索各跑一遍（settled 累加 470118），其中约 3/4 是纯重复。
      A/B：现役图 1412 ms → 581 ms（**2.43×**），**逐候选产物逐 token 相同**。
    * ★ 为什么不按"取第一个可行候选即停"来省：实测候选代价 = F4 **24.95** / D4 = D5 25.85 /
      F5 28.15，而 `full_cover` 顺序的第一个是 D4 ⇒ 提前停在第一个可行解会把产物换成
      D4 那条（+0.90 m、62→64 格），违反"产物逐 token 不变"的硬门禁。
      证据：历史证据目录的逐候选表脚本 的逐候选表。
    * 迭代顺序与原版**逐层相同**（legs0 顺序 → 下半场结果按代价序 → 上半场结果按代价序），
      因此 `out` 的插入顺序、`sort` 的稳定性、以及最终 `collect` 截断都一致。
    """
    start, end = _cell(spec.start), _cell(spec.end)
    lower = [_cell(m) for m in spec.mines if _cell(m)[0] >= spec.barrier_row_lo]
    upper = [_cell(m) for m in spec.mines if _cell(m)[0] <= spec.barrier_row_hi]
    cache: dict = {}
    # 起点朝向 = **设计常量**（车头 +y），不再是"从起点指向下半场穿洞格的方向"
    #   —— 旧写法 `_ts_unit((cross_lower - start))` 在斜向时给出的是 (1,-3) 这类**斜向量**，
    #   当朝向用等于给车一个 ~108° 的虚拟车头（实测它让"起步先向南 90° 掉头"比直行更便宜）。
    #   同时禁止起点转向：第一步必须沿设计朝向直行（见 `_ts_leg(first_dir=…)`）。
    h0 = DIRS[start_heading]
    legs0 = _ts_leg(ts, start, cross_lower_cell, h0, cache, first_dir=h0)
    if not legs0:
        return {sc: [] for sc in shoot_cells}
    # ---- 候选无关：下半场（穿洞格 → phase，必访下半场雷）只算一次 ----
    #   ★ 去重键用 `hp[0]`（**进入朝向**）：本 DP 只读 `hpair[0]`（`h_prev` 不参与，见
    #     `_ts_phase` 的说明）⇒ 起点那一段若有多个到达状态而进入朝向相同，DP 结果是同一份。
    #     实测现役图 `legs0` 的两个到达状态进入朝向都是 `(0,-1)`，旧实现把同一份 DP 算了两遍。
    lower_res: dict = {}
    for hp in legs0:
        if hp[0] not in lower_res:
            lower_res[hp[0]] = _ts_phase(ts, cache, lower, cross_lower_cell,
                                         (hp[0], None), phase_cell, deadline, keep=keep)
    results: dict[tuple[int, int], list[tuple[float, list, int]]] = {}
    for shoot_cell in shoot_cells:
        out: list[tuple[float, list, int]] = []
        order_upper = list(upper) + [shoot_cell]
        #   ★ 同一个下半场到达状态 `hpA` 可能从多个 `legs0` 桶里出现（现役图：同一 hpA
        #     出现 2 次）—— 旧实现逐次重算整棵上半场 DP。这里按 `hpA` 复用：`_ts_phase`
        #     是纯函数（同样的 order/cur/hpair/last + 同一份 `cache` ⇒ 逐位相同结果），
        #     复用不改变任何代价、路径与迭代顺序。
        upper_res: dict = {}
        for hp, (c0, cells0) in legs0.items():
            for costA, pathA, hpA in lower_res[hp[0]]:
                hit = upper_res.get(hpA)
                if hit is None:
                    hit = upper_res[hpA] = _ts_phase(ts, cache, order_upper, phase_cell,
                                                     hpA, cross_upper_cell, deadline,
                                                     keep=keep)
                for costB, pathB, hpB in hit:
                    for _hp2, (c1, cells2) in _ts_leg(ts, cross_upper_cell, end, hpB[0],
                                                      cache).items():
                        full = [start] + cells0 + pathA + pathB + cells2
                        bad = len(_ts_tight_reversals(full))
                        out.append((c0 + costA + costB + c1, full, bad))
        out.sort(key=lambda t: t[0])
        results[shoot_cell] = out[:collect]
    return results


def _ts_search(ts: _TSGrid, spec: MapSpec, *, cross_lower_cell: tuple[int, int],
               cross_upper_cell: tuple[int, int], shoot_cell: tuple[int, int],
               phase_cell: tuple[int, int], start_heading: int,
               deadline: float, collect: int = 24) -> list[tuple[float, list, int]]:
    """单候选包装（保留原调用形态；内部走多候选实现）。"""
    return _ts_search_multi(ts, spec, cross_lower_cell=cross_lower_cell,
                            cross_upper_cell=cross_upper_cell,
                            shoot_cells=[shoot_cell], phase_cell=phase_cell,
                            start_heading=start_heading, deadline=deadline,
                            collect=collect)[shoot_cell]


# =========================================================================== #
#  拐角**条件切角**（90° 直角 → 45° 斜段；纯格名 token，零语法改动）
# =========================================================================== #
#  为什么是"删顶点"而不是"插半格偏移点"（卡面写的是后者）：
#    ① 全库**没有**"格心+米制偏移"的 token 语法（`parser.normalize_token()` 要求格名后
#       全部是数字，`F3+0.175` 直接 `ValueError: 无法解析的 token`）；而路线是**以 token 串**
#       传给车侧（`tasks/mission.py` → `parse_mission`、`main.py` 回写 `route.tokens`）
#       ⇒ 偏移点要么解析失败、要么被静默丢回格心 ⇒ **到不了车上**；
#    ② 删掉 90° 顶点后，前后两点（对角相邻）之间**本身就是 45° 斜段**，
#       等价于"把 90° 转角分成两段 45°"，且 token 仍是纯格名 ⇒ 往返/门禁/车侧全兼容。
#  实测（现役 44 点，`_tmp_session/t53_corner/`）：切 6 个角 → 38 点，
#    顶点超调 0.04912→0.03887 m（−20.9%）、拐角 RMS 0.0532→0.0466、急转弯 18→11 处、
#    靶板净空 0.1441→0.2048 m（净空不足拍 49→0）、挡板净距 0.0783→0.1361 m，
#    雷覆盖 8/8、硬违规全 0、`route_check` exit 0、`config_selfcheck` 0 error/0 warn。
#  默认**关闭**（`planner.corner_cut`，缺省 False）⇒ 现役发车路线逐 token 不变；
#  开启 = 一行配置（`config/robot.yaml`，写域之外，待队长裁决）。
CORNER_CUT_MIN_TURN_DEG = 88.0     # 只切"直角"（90°±2°）：其它转角切了没有 45° 语义
CORNER_CUT_MAX_TURN_DEG = 92.0
CORNER_CUT_MINE_CLEAR_M = 0.15     # 卡面硬规则②：内切路径离最近雷心 < 0.15 m ⇒ 不切


def corner_cut_enabled(cfg: "Config | None") -> bool:
    """切角开关。缺省 **False**（= 现役发车路线逐 token 不变，零行为变化）。"""
    if cfg is None:
        return False
    try:
        return bool(cfg.get("planner.corner_cut", False))
    except Exception:                       # noqa: BLE001 —— 老调用方/坏配置一律当关闭
        return False


def _seg_point_dist(px: float, py: float, ax: float, ay: float,
                    bx: float, by: float) -> float:
    """点 (px,py) 到线段 AB 的距离。"""
    vx, vy = bx - ax, by - ay
    l2 = vx * vx + vy * vy
    if l2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / l2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def _min_clear(centers: Sequence[tuple[str, tuple[float, float]]],
               segs: Sequence[tuple[float, float, float, float]]) -> float:
    """一组圆心到一组线段的最近距离。"""
    best = float("inf")
    for _nm, (cx, cy) in centers:
        for (ax, ay, bx, by) in segs:
            best = min(best, _seg_point_dist(cx, cy, ax, ay, bx, by))
    return best


def _barrier_line_clear(segs: Sequence[tuple[float, float, float, float]],
                        row_hi: float, col_from: str) -> float:
    """到"挡板细条"（第 row_hi/row_hi+1 行之间、col_from..N 列）的最近距离。

    挡板是**线**不是点，所以按 0.02 m 步长采样连线（够密；判据本身就是毫米量级余量）。
    """
    y_mid = (row_hi + 0.5) * geo.BLOCK_SIZE
    cols = {chr(c) for c in range(ord(col_from), ord("N") + 1)}
    best = float("inf")
    for (ax, ay, bx, by) in segs:
        for i in range(101):
            t = i / 100.0
            x = ax + t * (bx - ax)
            y = ay + t * (by - ay)
            if geo.meters_to_block(x, y)[0] in cols:
                best = min(best, abs(y - y_mid))
    return best


def corner_cut_cells(cells: Sequence[tuple[int, int]], spec: MapSpec,
                     *, barrier_row_hi: float, barrier_col_from: str,
                     mine_clear_m: float = CORNER_CUT_MINE_CLEAR_M,
                     stop_cells: Sequence[tuple[int, int]] = (),
                     crossings: Sequence[tuple[int, int]] = ()) \
        -> tuple[list[tuple[int, int]], list[tuple[int, int]], list[str]]:
    """对格列表做**条件切角**：删除被"七条硬规则"放行的 90° 直角顶点。

    规则（① ② ③ 来自卡面；④–⑦ 是实测**必须补**的，缺一条就会打红门禁或丢雷）：
      ① 拐角格本身是雷格 → 不切；
      ② 切后线段（prev→next）离任一雷心 < `mine_clear_m`，或比切前的两条腿更近 → 不切；
      ③ 切后线段对 障心 / 挡板细条 / 靶格心 的最近距离比切前更近 → 不切（净空不放松）；
      ④ 拐角格是某雷的**进点/出点**（prev 或 next 是雷格）→ 不切（保护「进点—雷格—出点」三连）；
      ⑤ 拐角格紧邻桥洞三连（prev 或 next 在 K 列 / 是桥洞格）→ 不切（保护 R007/R009 水平直穿）；
      ⑥ 只切 90°±2° 的直角；相邻两个拐角（索引差 ≤ 1）不同时切；
      ⑦ 起点/终点/雷格/桥洞格/射击或扫描停站/PIVOT 顶点一律不切（token 后缀严格保留）。

    :return: `(新格列表, 被删的格列表, 逐角原因说明)` —— 纯函数，同入参必同出参。
    """
    cells = list(cells)
    mine_pts = [(m, geo.block_to_meters(m)) for m in spec.mines]
    obst_pts = [(o, geo.block_to_meters(o)) for o in spec.obstacles]
    tgt_pts = [(t, geo.block_to_meters(t)) for t in spec.targets]
    mine_set = {m for m, _ in mine_pts}
    bridge_set = set(spec.bridges)
    cross_set = set(crossings)
    stop_set = set(stop_cells)
    notes: list[str] = []
    drop: list[int] = []
    last_cut = -10

    for i in range(1, len(cells) - 1):
        if i - last_cut <= 1:
            continue
        a, v, c = cells[i - 1], cells[i], cells[i + 1]
        blk = geo.index_to_block(v[0] - 1, v[1])
        blk_a = geo.index_to_block(a[0] - 1, a[1])
        blk_c = geo.index_to_block(c[0] - 1, c[1])
        ax, ay = geo.block_to_meters(blk_a)
        vx, vy = geo.block_to_meters(blk)
        cx, cy = geo.block_to_meters(blk_c)
        h_in = math.atan2(vy - ay, vx - ax)
        h_out = math.atan2(cy - vy, cx - vx)
        turn = abs(math.degrees(geo.ang_diff(h_out, h_in)))
        if not (CORNER_CUT_MIN_TURN_DEG <= turn <= CORNER_CUT_MAX_TURN_DEG):
            continue
        if v in stop_set:
            notes.append(f"{blk}:⑦停站/指令格")
            continue
        if blk in mine_set:
            notes.append(f"{blk}:①拐角格是雷")
            continue
        if blk in bridge_set or v in cross_set:
            notes.append(f"{blk}:⑤桥洞格")
            continue
        if blk_a in mine_set or blk_c in mine_set:
            notes.append(f"{blk}:④雷三连（{blk_a}/{blk_c} 是雷）")
            continue
        if blk_a in bridge_set or blk_c in bridge_set or a in cross_set or c in cross_set:
            notes.append(f"{blk}:⑤桥洞三连（{blk_a}/{blk_c}）")
            continue
        seg_old = [(ax, ay, vx, vy), (vx, vy, cx, cy)]
        seg_new = [(ax, ay, cx, cy)]
        d_mine_new = _min_clear(mine_pts, seg_new)
        d_mine_old = _min_clear(mine_pts, seg_old)
        if d_mine_new < mine_clear_m - 1e-9:
            notes.append(f"{blk}:②切后离雷 {d_mine_new:.3f} < {mine_clear_m:.2f}")
            continue
        if d_mine_new < d_mine_old - 1e-9:
            notes.append(f"{blk}:②切后比切前更近雷（{d_mine_old:.3f}→{d_mine_new:.3f}）")
            continue
        d_bar_new = _barrier_line_clear(seg_new, barrier_row_hi, barrier_col_from)
        d_bar_old = _barrier_line_clear(seg_old, barrier_row_hi, barrier_col_from)
        if d_bar_new < d_bar_old - 1e-9:
            notes.append(f"{blk}:③挡板更近（{d_bar_old:.3f}→{d_bar_new:.3f}）")
            continue
        d_ob_new = _min_clear(obst_pts, seg_new)
        d_ob_old = _min_clear(obst_pts, seg_old)
        if d_ob_new < d_ob_old - 1e-9:
            notes.append(f"{blk}:③障碍更近（{d_ob_old:.3f}→{d_ob_new:.3f}）")
            continue
        d_tg_new = _min_clear(tgt_pts, seg_new)
        d_tg_old = _min_clear(tgt_pts, seg_old)
        if d_tg_new < d_tg_old - 1e-9:
            notes.append(f"{blk}:③靶板更近（{d_tg_old:.3f}→{d_tg_new:.3f}）")
            continue
        drop.append(i)
        last_cut = i
        notes.append(f"{blk}:✅切（转角 {turn:.0f}° / 离雷 {d_mine_new:.3f} / "
                     f"离障 {d_ob_new:.3f} / 离靶 {d_tg_new:.3f}）")

    kept = [cell for j, cell in enumerate(cells) if j not in set(drop)]
    return kept, [cells[j] for j in drop], notes


# =========================================================================== #
#  增量续算（G4）—— `from_cell` + `already_covered`
# =========================================================================== #
#  为什么必须**新增入参**，而不是"把 `MapSpec.start` 改成当前格"（§E4 的失败路径）：
#    · `MapSpec` 是**地图**（map.yaml 解析产物）。起点进地图 ⇒ 任何同时读 `spec.start`
#      的地方（结构自检首点、落盘 token 首点、现场回写、备份对账）都会跟着漂；
#    · 实测（docs §11 E4）：把 start 改到 E10 / F2 之后，规划器直接
#      `PlannerError: 起点 ... 不在下半场桥洞 ... 以东` —— 那条路**走不通**，
#      也**不该**走通：R007「每半场恰好一次穿洞」依赖"车在洞东 ⇒ 必须且只能往西穿一次"
#      这个几何事实；把起点挪到洞西再要求穿一次，等于要求穿两次。
#  **起点硬门降级规则**（= docs §6 第 6 条）：原硬门「起点在洞东、必须穿一次」
#  在 `from_cell` 位于**洞已穿过**的位置时**降级为"已满足"**：
#    · `from_cell` 在下半场、K 列以东  ⇒ 仍是原硬门（下半场要穿一次）；
#    · `from_cell` 在下半场、K 列以西  ⇒ 下半场穿洞**已发生**，本轮不再要求（允许 0 次）；
#    · `from_cell` 在上半场             ⇒ 下半场穿洞已发生；上半场若仍在 K 列以西则要穿一次。
#  ⚠️ 不做"隐式回填"：若剩余必压雷落在**已封闭的那一侧**（例如人在上半场、下半场还有雷），
#    续算必须再穿一次洞 ⇒ R007 必红 ⇒ 这里**如实** `PlannerError`，不偷偷放行两次穿越。
def _inc_start_cell(grid: "Grid", from_cell: str) -> tuple[int, int]:
    """`from_cell` 归一 + 合法性硬门（格名 / 场地内 / 可通行）。"""
    name = str(from_cell).strip().upper()
    try:
        row, col = _cell(name)
    except ValueError as exc:
        raise PlannerError(f"from_cell {name!r} 不是合法格名（{exc}）") from None
    if not grid.in_bounds(row, col):
        raise PlannerError(f"from_cell {name} 不在场地内（行 {row} / 列 {col}）")
    if not grid.passable(row, col):
        raise PlannerError(f"from_cell {name} 不可通行（障碍格 / 被封锁）")
    return row, col


def _inc_covered(already_covered: Sequence[str] | None) -> frozenset[str]:
    """归一 `already_covered`（只认格名；非雷格也接受，但只对**雷**生效）。"""
    out: set[str] = set()
    for cell in already_covered or ():
        name = str(cell).strip().upper()
        if not name:
            continue
        try:
            _cell(name)                   # 非法格名直接抛（不静默吞）
        except ValueError as exc:
            raise PlannerError(
                f"already_covered 里的 {name!r} 不是合法格名（{exc}）") from None
        out.add(name)
    return frozenset(out)


def _inc_shape(spec: MapSpec, start_cell: tuple[int, int],
               covered: frozenset[str]) -> dict:
    """增量续算的**段形状**：哪些雷还要压、哪些半场还要穿洞。"""
    row_lo = int(spec.barrier_row_lo)
    lower_mines = [c for c in map(_cell, spec.mines) if c[0] >= row_lo]
    upper_mines = [c for c in map(_cell, spec.mines) if c[0] < row_lo]

    def _name(c: tuple[int, int]) -> str:
        return geo.index_to_block(c[0] - 1, c[1])

    rem_lower = [c for c in lower_mines if _name(c) not in covered]
    rem_upper = [c for c in upper_mines if _name(c) not in covered]

    srow, scol = start_cell
    in_lower = srow >= row_lo
    need_lower = bool(in_lower and scol > _CROSS_COL)
    need_upper = bool(scol < _CROSS_COL) if not in_lower else True

    # ---- 可行性硬门：续算**不许**靠"再穿一次洞"兜底（R007 必红，且现场会撞桥墩）----
    if not in_lower and rem_lower:
        raise PlannerError(
            "增量续算不可行：起点 %s 已在上半场，但下半场仍有未压雷 %s"
            "（回到下半场必须再穿一次洞 ⇒ R007『每半场恰好一次』被破）。"
            "请先补齐下半场，或改用整线规划（不带 from_cell）。"
            % (_name(start_cell), " ".join(_name(c) for c in rem_lower)))
    if not need_upper and rem_upper:
        raise PlannerError(
            "增量续算不可行：起点 %s 已在上半场桥洞以东，但上半场仍有未压雷 %s"
            "（回去必再穿一次洞 ⇒ R007 被破）。请改用整线规划。"
            % (_name(start_cell), " ".join(_name(c) for c in rem_upper)))
    return {"in_lower": in_lower, "need_lower": need_lower, "need_upper": need_upper,
            "rem_lower": rem_lower, "rem_upper": rem_upper}


def _ts_search_incremental(ts: "_TSGrid", spec: MapSpec, *,
                           cross_lower_cell: tuple[int, int],
                           cross_upper_cell: tuple[int, int],
                           shoot_cells: Sequence[tuple[int, int]],
                           phase_cell: tuple[int, int], start_cell: tuple[int, int],
                           shape: dict, deadline: float, collect: int = 24,
                           keep: int | None = TS_PARETO_KEEP,
                           ) -> dict[tuple[int, int], list[tuple[float, list, int]]]:
    """增量续算的段装配（**复用** `_ts_leg` / `_ts_phase` 的同一套代价模型）。

    与 `_ts_search_multi` 的差别只有"从哪起、要穿几次洞"：
      ① `start_cell` 可以是**中途格**（不锁首步方向 —— 现场车头已由上一段决定）；
      ② 已穿过的半场**不再要求**穿洞（`shape["need_lower"]/["need_upper"]`）；
      ③ 已压雷从必访集里剔除（`shape["rem_lower"]/["rem_upper"]`）。
    产物形状与 `_ts_search_multi` 逐项同构 ⇒ 上层挑候选/编译/自检代码零改动。
    """
    end = _cell(spec.end)
    rem_lower, rem_upper = shape["rem_lower"], shape["rem_upper"]
    cache: dict = {}
    # 起点车头未知 ⇒ 只当作搜索状态的 `h_in` 种子（首步无转向代价），**不**锁首步方向。
    h0 = DIRS[START_HEADING]

    # ---------------- ① 到"下半场阶段入口" ----
    if shape["need_lower"]:
        legs = _ts_leg(ts, start_cell, cross_lower_cell, h0, cache)
        if not legs:
            return {sc: [] for sc in shoot_cells}
        lower_res: dict = {}
        for hp in legs:
            if hp[0] not in lower_res:
                lower_res[hp[0]] = _ts_phase(ts, cache, rem_lower, cross_lower_cell,
                                             (hp[0], None), phase_cell, deadline, keep=keep)
        seeds = [(c0, cells0, hp) for hp, (c0, cells0) in legs.items()]
        upper_from = phase_cell
    elif shape["in_lower"]:
        # 已过洞：直接从当前格进入下半场阶段（先把剩余下半场雷压完，再收在半场过渡点）
        hr = (h0, None)
        lower_res = {hr[0]: _ts_phase(ts, cache, rem_lower, start_cell, hr,
                                      phase_cell, deadline, keep=keep)}
        seeds = [(0.0, [], hr)]
        upper_from = phase_cell
    else:
        # 已在上半场：下半场阶段整体跳过
        hr = (h0, None)
        lower_res = {hr[0]: [(0.0, [], hr)]}
        seeds = [(0.0, [], hr)]
        upper_from = start_cell

    results: dict[tuple[int, int], list[tuple[float, list, int]]] = {}
    for shoot_cell in shoot_cells:
        out: list[tuple[float, list, int]] = []
        order_upper = list(rem_upper) + [shoot_cell]
        upper_res: dict = {}
        for c0, cells0, hp in seeds:
            for costA, pathA, hpA in lower_res[hp[0]]:
                hit = upper_res.get(hpA)
                if hit is None:
                    if shape["need_upper"]:
                        hit = upper_res[hpA] = _ts_phase(
                            ts, cache, order_upper, upper_from, hpA, cross_upper_cell,
                            deadline, keep=keep)
                    else:
                        # 上半场也已过洞：压完剩余上半场雷 + 射击点，直接回终点
                        hit = upper_res[hpA] = _ts_phase(
                            ts, cache, order_upper, upper_from, hpA, end,
                            deadline, keep=keep)
                for costB, pathB, hpB in hit:
                    if shape["need_upper"]:
                        tails = _ts_leg(ts, cross_upper_cell, end, hpB[0], cache)
                    else:
                        tails = {hpB: (0.0, [])}
                    for _hp2, (c1, cells2) in tails.items():
                        full = [start_cell] + cells0 + pathA + pathB + cells2
                        bad = len(_ts_tight_reversals(full))
                        out.append((c0 + costA + costB + c1, full, bad))
        out.sort(key=lambda t: t[0])
        results[shoot_cell] = out[:collect]
    return results


def plan_route_tracker_safe(map_cfg: Any, cfg: "Config | None" = None, *,
                            cross_lower: str | None = None,
                            cross_upper: str | None = None,
                            phase_cell: str = TS_PHASE_CELL,
                            turned_mines: Sequence[str] = TS_TURNED_MINES,
                            turn_w: float = TS_TURN_W,
                            max_seconds: float = TS_MAX_SECONDS,
                            name: str = "planner-tracker-safe",
                            source: str = "",
                            from_cell: str | None = None,
                            already_covered: Sequence[str] = ()) -> PlanResult:
    """**tracker-safe 引擎** —— 生成"真车能跟踪"的最短路线（自动追平现役图路线，44 token）。

    与 `plan_route()` 的差别只有一处**策略**：本引擎把"真车能不能跟住这条折线"当成
    硬约束（禁 180°/禁紧凑 U/雷穿心），代价只由"实走长度 + 转角重量"决定；
    旧引擎则用"原地转向 + 软惩罚"绕开死角，代价里掺了大量绕远惩罚 ⇒ 输出的路线
    更长、且带 `:PIVOT`（仿真不建模）。

    :param phase_cell: 两半场之间的过渡航点（默认 `C6`，与现场手工路线同点）。
    :param turned_mines: 允许 90° 拐过的"口袋雷"（正交移动集下无法直线穿心的那些）。
    :param turn_w: 转角权重（米/90°）。
    :param from_cell: **增量续算的当前格**（缺省 `None` = 从 `spec.start` 整线规划）。
        给出后：路线首点 = `from_cell`（不再要求车在地图起点），且按"洞已穿过则不再要求穿洞"
        的降级规则装配段（详见上方 `_inc_shape()` 的说明）。
        ⚠️ 增量**只能**走本入参：把 `MapSpec.start` 改成中途格是 §E4 的失败路径，
        那条路至今仍然失败（结构自检 / 起点可达性会拦），不要"修"它。
    :param already_covered: **已压格集合**（只对雷生效）——写进来的雷不再要求重压。
    :return: `PlanResult`（字段与 `plan_route` 同构；`allow_reverse=False`、
        `pivots` 恒为空 —— 本引擎不产生折返）
    :raises PlannerError: 无解 / 超时 / 增量续算位置不可行
    """
    t0 = time.perf_counter()
    spec = map_cfg if isinstance(map_cfg, MapSpec) else load_map(map_cfg, source=source)
    limits = _limits(cfg)
    deadline = t0 + max(0.05, float(max_seconds))
    phys = _phys_params(cfg)
    start_cell, end_cell = _cell(spec.start), _cell(spec.end)

    lower_pool, upper_pool = _cross_pools(cfg, spec)
    lower_cross = (cross_lower or _default_cross(cfg, lower_pool, "lower")).strip().upper()
    upper_cross = (cross_upper or _default_cross(cfg, upper_pool, "upper")).strip().upper()
    # 与 `plan_route` 同口径 —— 硬门 = K 列 + 本半场行段（候选池只用来挑缺省洞）
    for cell_name, half, band in ((lower_cross, "下半场", V.LOWER_HALF_ROWS),
                                  (upper_cross, "上半场", V.UPPER_HALF_ROWS)):
        try:
            row_idx, col_idx = geo.block_to_index(cell_name)
        except ValueError as exc:
            raise PlannerError(f"{half}穿洞格 {cell_name} 格名非法（{exc}）") from None
        if col_idx != _CROSS_COL:
            raise PlannerError(f"穿洞格 {cell_name} 不在 {V.CROSS_COLUMN} 列上")
        row = row_idx + 1
        if not (band[0] <= row <= band[1]):
            raise PlannerError(
                f"{half}穿洞格 {cell_name} 不在 {V.CROSS_COLUMN}{band[0]}~{V.CROSS_COLUMN}"
                f"{band[1]} 行段内（本半场只允许这一段 K 列格）")
    cross_lower_cell, cross_upper_cell = _cell(lower_cross), _cell(upper_cross)
    if cross_lower_cell == cross_upper_cell:
        raise PlannerError(f"下半场与上半场选了同一个桥洞 {lower_cross}（必须各穿一次）")
    phase = _cell(phase_cell) if isinstance(phase_cell, str) else tuple(phase_cell)

    grid = Grid(spec, bridge_rows=(cross_lower_cell[0], cross_upper_cell[0]),
                allow_reverse=False)
    ts = _TSGrid(spec=spec, grid=grid, phys=phys, turn_w=turn_w,
                 turned_mines=tuple(turned_mines))

    # ---------------- 起点朝向 = 设计常量 ----------------
    #   与 `plan_route` 同源：车摆在起点格、车头恒定 +y（图左 / K 列方向），
    #   摆位由现场用户保证 ⇒ 不读 `mission.reset`、不校验、不生成转向消解。
    start_heading = START_HEADING

    # ---------------- 射击停站候选（必须覆盖**全部**靶格） ----------------
    acc = _shoot_accept(cfg)
    pool = shoot_candidates(spec, grid, acc)
    cover = covering_stops(spec, pool, acc)
    full_cover = [b for b in pool if len(cover.get(b, ())) == len(spec.targets)]
    if spec.targets and not full_cover:
        raise PlannerError(f"没有任何射击点能覆盖全部靶格 {spec.targets}（候选 {pool}）")
    shots = full_cover or [""]

    # ---------------- 逐候选搜索（共享候选无关部分；语义仍是"取代价最小者"）----
    #   为什么不能"取第一个可行候选即停"：实测候选代价 F4=24.95 / D4=D5=25.85 / F5=28.15，
    #   而 full_cover 的第一个是 D4 ⇒ 提前停会把产物换成 D4（+0.90 m、62→64 格）。
    #   见 `_ts_search_multi()` 的说明与 历史证据目录的逐候选表脚本 的逐候选表。
    shoot_cells = [_cell(s) if s else cross_upper_cell for s in shots]
    # ---------------- 增量续算分支（缺省 = 走原路，逐位不变） ----------------
    incremental = bool(from_cell) or bool(already_covered)
    inc_start = start_cell
    covered: frozenset[str] = frozenset()
    shape: dict | None = None
    if incremental:
        if from_cell:
            inc_start = _inc_start_cell(grid, from_cell)
        covered = _inc_covered(already_covered)
        shape = _inc_shape(spec, inc_start, covered)
        searched = _ts_search_incremental(
            ts, spec, cross_lower_cell=cross_lower_cell,
            cross_upper_cell=cross_upper_cell, shoot_cells=shoot_cells,
            phase_cell=phase, start_cell=inc_start, shape=shape, deadline=deadline)
    else:
        searched = _ts_search_multi(ts, spec, cross_lower_cell=cross_lower_cell,
                                    cross_upper_cell=cross_upper_cell,
                                    shoot_cells=shoot_cells, phase_cell=phase,
                                    start_heading=start_heading, deadline=deadline)
    best: tuple[float, list, int, str] | None = None
    for shoot, shoot_cell in zip(shots, shoot_cells):
        cands = searched.get(shoot_cell, [])
        clean = [c for c in cands if c[2] == 0]
        pick = clean[0] if clean else (cands[0] if cands else None)
        if pick is None:
            continue
        cost, cells, _bad = pick
        if best is None or cost < best[0] - 1e-12:
            best = (cost, cells, _bad, shoot)
    if best is None:
        raise PlannerError("tracker-safe 引擎无解（可能被『禁 180°/禁紧凑 U/雷穿心』筛空）")
    cost, cells, n_bad_u, shoot = best
    warnings: list[str] = []

    # ---------------- 条件切角（默认关闭；切不过 ⇒ 安全回退）----------------
    cells_uncut = list(cells)
    cut_report: list[str] = []
    if corner_cut_enabled(cfg) and len(cells) > 2:
        kept, dropped, cut_report = corner_cut_cells(
            cells, spec,
            barrier_row_hi=float(spec.barrier_row_hi),
            barrier_col_from=str(spec.barrier_col_from),
            mine_clear_m=CORNER_CUT_MINE_CLEAR_M,
            stop_cells=[shoot_cell for _s, shoot_cell in zip(shots, shoot_cells)]
            + [phase, start_cell, inc_start, end_cell],
            crossings=[cross_lower_cell, cross_upper_cell])
        if dropped:
            cells = kept

    # ---------------- 编译成航点 / Route ----------------
    #   ★ t7：第二射击停站候选**只由地图/靶板/窗口决定**（与路线形状无关）⇒ 在这里算一次；
    #     编译闭包里只判"落不落得上"。落不上/无解都必须在 warnings 里留痕 ——
    #     旧版"候选算出来却没落到路线上"是**完全静默**的，等于把双停站悄悄退化成单停站。
    _second_stop = second_shoot_stop(spec, grid, shoot, acc) if shoot else ""

    def _compile(_cells: list) -> tuple[list, Any, list]:
        _stops: list[_Stop] = []
        for row, col in _cells:
            blk = geo.index_to_block(row - 1, col)
            tag = ("start" if (row, col) == inc_start else
                   "end" if (row, col) == end_cell else
                   "mine" if blk in set(spec.mines) else
                   "lower_bridge" if (row, col) == cross_lower_cell else
                   "upper_bridge" if (row, col) == cross_upper_cell else
                   "shoot" if blk == shoot else "path")
            _stops.append(_Stop(row, col, blk, shoot=(blk == shoot), tag=tag))
        # 只有"真的从地图起点出发"时才把首点朝向钉成设计常量；
        #   增量续算的首点朝向由第一段行进方向决定（车头已由上一段决定，不该被改写）。
        if inc_start == start_cell:
            _stops[0].heading = start_heading
        _refresh_headings(_stops)
        # ---- 第二射击停站（对侧）----
        #   只在"该格本来就是这条路线上的普通航点"时才落标（现图 = `G3`），
        #   否则**不改路线**（宁可少一个停站，也不擅自改穿洞/折返形态），
        #   但落不上/无解会在下面统一变成一条**显式 warning**（绝不静默）。
        if _second_stop:
            for _st in _stops:
                if _st.block == _second_stop and not _st.shoot and _st.tag == "path":
                    _st.shoot = True
                    _st.tag = "shoot"
                    break
        _marked = mark_pivots(_stops)
        if _marked:
            raise PlannerError("tracker-safe 引擎内部错误：路线里出现了折返点 "
                               + " ".join(b for _i, b, _d in _marked))
        _route = _to_route(_stops, name=name,
                           crossing={"lower": lower_cross, "upper": upper_cross},
                           final_heading=_final_heading(_stops))
        return _stops, _route, check_route_structure(
            _route, spec, grid, limits, acc,
            expected_start=geo.index_to_block(inc_start[0] - 1, inc_start[1]),
            already_covered=tuple(sorted(covered)),
            crossing_halves=((shape["need_lower"], shape["need_upper"])
                             if shape else (True, True)))

    stops, route, problems = _compile(cells)
    if problems and cells != cells_uncut:            # 切角不放过结构自检 ⇒ 回退（绝不影响可规划性）
        warnings.append("切角后结构自检未过 ⇒ 已回退未切角路线： "
                        + "；".join(problems[:2]))
        cells = cells_uncut
        cut_report = []
        stops, route, problems = _compile(cells)
    if cut_report:
        _cuts = [n for n in cut_report if "✅切" in n]
        if _cuts:
            warnings.append("拐角条件切角 %d 处（%s）" % (len(_cuts), "，".join(_cuts)))
    if problems:
        raise PlannerError("tracker-safe 结果未通过结构自检：\n  - " + "\n  - ".join(problems))
    assert_legal_no_reverse(route)

    mine_runs = _mine_aligned_runs(stops, spec)
    mine_axes = tuple((m, -1, run >= 2) for m, run in mine_runs)
    short = [(b, r) for b, r in mine_runs if r < 2]
    if short:
        warnings.append("雷心压过风险：%s（进点前共线不足 2 格）" %
                        " ".join("%s=%d" % (b, r) for b, r in short))
    if n_bad_u:
        warnings.append("仍有 %d 处紧凑 U 形折返（已被搜遍仍无法避开）" % n_bad_u)
    if shoot:
        warnings.append("射击停站由『覆盖全部靶格』精确筛出：%s" % shoot)
    # ★★ t7 双停站台账（**报警线**，不是装饰）：落不上 / 无解都必须在日志里看得见。
    #   为什么必须是 warning 而不是静默：靶板两面蓝/红随机，单停站意味着"蓝面朝错那局整场打不到"，
    #   而 15:04 现场那次丢停站在日志里**一个字都没有**，靠人肉比对 token 串才发现。
    _shoot_hit = [wp.block for _i, wp in route.shoot_points()]
    if shoot and _second_stop and _second_stop not in _shoot_hit:
        warnings.append("⚠️ 第二射击停站未落标：对侧候选 %s 不在本路线航点上 ⇒ 本路线只有 %d 个"
                        "射击停站（蓝面朝另一侧那局会打不到）" % (_second_stop, len(_shoot_hit)))
    elif shoot and not _second_stop:
        warnings.append("⚠️ 第二射击停站无解：板面另一侧没有能覆盖全部靶格的候选格 ⇒ "
                        "本路线只有 %d 个射击停站" % len(_shoot_hit))
    # 增量续算必须在产物里**显式留痕**（现场/日志一眼看出这不是整线路线）
    if incremental:
        _inc_name = geo.index_to_block(inc_start[0] - 1, inc_start[1])
        _skip = [h for h, need in (("下半场", shape["need_lower"]),
                                   ("上半场", shape["need_upper"])) if not need]
        warnings.append(
            "增量续算：起点 from_cell=%s（非地图起点 %s）；已压 %d 格（%s）；"
            "跳过穿洞=%s"
            % (_inc_name, spec.start, len(covered),
               " ".join(sorted(covered)) or "无",
               "／".join(_skip) or "无（两个半场都还要穿）"))

    tokens = route_tokens(route)
    shoot_at = next((wp.block for _i, wp in route.shoot_points()), "")
    shoot_dist = 0.0
    shoot_targets: tuple[str, ...] = ()
    # 第二个射击停站（对侧）——从**最终路线**里取，避免与编译闭包耦合；
    #   没有就是 `""`，PlanResult 的其余字段与单停站版本逐位一致。
    shoot_at2 = next((wp.block for _i, wp in route.shoot_points()
                      if wp.block != shoot_at), "")
    shoot_dist2 = 0.0
    if shoot_at:
        # 记录的是**到靶板中心**的距离（与 `shoot_candidates()` 同源口径），
        #   不再是"到最近靶格中心"。板心 = 各靶格中心之算术中点（`plate_center_m`）。
        acc_lo, acc_hi = shoot_distance_window(_shoot_accept(cfg))
        _cx, _cy = plate_center_m(spec)
        _sx, _sy = geo.block_to_meters(shoot_at)
        shoot_dist = math.hypot(_sx - _cx, _sy - _cy)
        shoot_targets = tuple(t for t in spec.targets
                              if acc_lo - 1e-9 <= stop_distance_m(shoot_at, t) <= acc_hi + 1e-9)
        if shoot_at2:
            _tx, _ty = geo.block_to_meters(shoot_at2)
            shoot_dist2 = math.hypot(_tx - _cx, _ty - _cy)
            warnings.append("第二射击停站 %s 到靶板中心 %.3f m（对侧，板面相反）"
                            % (shoot_at2, shoot_dist2))
    polyline = sum(math.hypot(route.waypoints[i + 1].x - route.waypoints[i].x,
                              route.waypoints[i + 1].y - route.waypoints[i].y)
                   for i in range(len(route.waypoints) - 1))
    return PlanResult(
        route=route, tokens=tokens, stops=tuple(stops),
        crossing={"lower": lower_cross, "upper": upper_cross},
        shoot_stop=shoot_at, shoot_distance_m=float(shoot_dist),
        shoot_stop2=shoot_at2, shoot_distance_m2=float(shoot_dist2),
        shoot_targets=shoot_targets, cost=float(cost), expanded=int(ts.settled),
        elapsed_ms=(time.perf_counter() - t0) * 1000.0, reverse_segments=0,
        warnings=tuple(warnings), map_name=spec.name, map_source=spec.source,
        start=(geo.index_to_block(inc_start[0] - 1, inc_start[1])
               if incremental else spec.start),
        end=spec.end, polyline_m=polyline,
        start_heading=HEAD_NAMES[start_heading], allow_reverse=False, pivots=(),
        pivot_penalty=0.0, mine_turn_penalty=float(ts.mine_turn_penalty_m),
        mine_axes=mine_axes, mine_runs=tuple(mine_runs), tour_cost=float(cost))
