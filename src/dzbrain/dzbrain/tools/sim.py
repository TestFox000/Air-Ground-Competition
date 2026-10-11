# -*- coding: utf-8 -*-
"""离线纯追踪仿真器（C 线，`docs/P0接口约定.md` §5）——**不碰车就能调参与回归**。

为什么要它（旧流程的痛点）
------------------------------------------------------------------
旧版调一个循迹参数要：改 YAML → 推到车 → colcon restart → 遥控摆位 → 跑完看横偏。
一轮十分钟起，还受电池/场地/云台误差干扰，**同一组参数跑不出可比结果**。
本模块把"车 + 里程计误差 + 到下位机的链路延迟"压成一个确定性模型，
同一组参数永远得到同一条曲线 —— 于是"A 参数 vs B 参数"的对照实验才成立。

模型（★★ 2026-10-07 重写：单一执行量驱动两条轨迹）
------------------------------------------------------------------
**执行链（唯一的物理真相）**：控制器每拍输出一条 `PursuitCommand`
→ `cmd_latency_s` 延迟队列（**队列空时保持上一条已生效指令**，绝不回零）
→ **执行侧舵角斜坡**（受 `pursuit.max_steer_rate_deg_s` 限制）
→ 得到本拍**实际执行量** `(v_exec, δ_exec)`。

**真值（物理世界）与信念（里程计）用同一套离散积分器、同一个执行量**：

======================  ==================================================
真值                    信念（控制器**看到**的位姿 = `P_POSE`）
======================  ==================================================
`_bicycle_step(x, y, ψ,   `_bicycle_step(x, y, ψ,`
`v_exec, δ_exec, dt, L)`   `v_exec·(1+ε), δ_exec, dt, L)`
—                         `ψ += yaw_drift_per_s·dt`
—                         `+ 高斯噪声`（`pos_noise_m`）
======================  ==================================================

★ **零误差不变量**（这是"模型可信"的定义）：`ε = 0`、零漂 = 0、噪声 = 0 时，
  两条轨迹**逐位相同**（差 = 0，不是"很小"）。⚠️ 这条不变量以前不成立：老实现里
  真值用的是"执行量"、信念用的是"指令量 + 另一套积分顺序"，零误差下就有
  0.34 m 的系统性偏差 —— 于是所有注入实验都失去基准（"误差 0.34 m" 是模型自己的错）。
  ⚠️ 唯一的例外是**到位吸附**（见下），它按定义就是"外部把信念拽过去"。

| 误差源 | SimConfig 字段 | 模拟的现实 |
|---|---|---|
| 里程比例误差 | `speed_scale_err` | 底盘 `ticksPerMeter` 标定偏大 → 里程计报得比真走的多 |
| 航向零漂 | `yaw_drift_deg_per_min` | 陀螺积分零偏（增量式，不是瞬时跳变） |
| 位置白噪声 | `pos_noise_m` | 定位抖动（每拍叠加） |
| 到位吸附 | `anchor_on_arrival` + `anchor_radius_m` | Localization 的 L2 修正（`fix.anchor_at_waypoint`）：靠近下一航点时把信念锚到格心 |

⚠️ **锚定只改信念，不改真值**：它是"外部观测把估计拉回真值"，
  所以 `anchored=True` 的那一拍信念↔真值差会被**人为**抹平 —— 统计信念误差时
  这拍单独扣除（`belief_error_rms_m` 仍按全部拍统计，`belief_trivial_steps` 报扣除数）。

⚠️ **本仿真器复现的是"信念驱动"闭环**（这是实车真实形态）：
控制器拿信念位姿投影 + 决策，指令作用在真值车体上。后果必须写清楚，否则会误读指标：

1. 单纯 `speed_scale_err` **几乎不产生横向误差** —— 信念投影沿路线"多跑" 18%，
   但真值仍在同一条折线上，只是**沿弧长滞后**。所以 `cross_track_*` 对它不敏感，
   真正放大的是 `belief_error_rms_m`（信念↔真值距离）与顶点超调。
2. 因此"锚定 vs 不锚定"的对照要看**信念误差**（锚定把它夹在 `anchor_radius_m` 内，
   不锚定则按 `ε × 已走弧长` 无界累积），而不是只看横向误差。
3. 位置白噪声会被 50 Hz 控制环 + 舵机速率限幅低通掉大半，**不会**像直觉那样把车甩出去。
4. ⚠️ **倒车段的舵角符号翻转**（`pursuit.reverse_steer_sign`，60° 量级）是**切换瞬间的
   物理事实**，不是"速率超限"。它单独统计（`steer_sign_flip_steps` /
   `steer_cmd_rate_at_flip_deg_s`），不污染速率分位数与峰值判定 ——
   否则"调速率限幅"会去追一个本来就不该被限幅的量。

★ 不变量
1. `run_sim()` 是**纯函数**：不读文件、不写文件、不 import rclpy，同样入参永远同样出参。
2. 所有模型参数来自 `Config`（点号键）与 `SimConfig`；本模块**不硬编码魔法数字**
   （唯一的例外是算法常数，用 `_` 前缀常量标注并说明理由）。
3. 真值轨迹**不参与控制**，只参与指标统计 —— 保证"控制器只看得见带误差的信念"。
4. `metrics` 的键名是契约（`docs/P0接口约定.md` §5），只增不改。
5. 指标**按段类型分组**（`normal` / `reverse` / `corner`）：不分组的话一个 90° 拐角的
   0.3 m 会淹没整条路线的 RMS，参数调优就看不出方向（详见 `SegmentStat`）。

★★ 2026-10-07：现场图"跑不完"的根因已修（历史：822.5 s / 41127 拍 / ratio 5.32 未完成）
------------------------------------------------------------------
修完后的出厂实测：**completed=True / 6751 拍 / 135.0 s / ratio 1.04 / 末点 0.029 m**。
根因**不是**几何（把 `max_steer_deg` 放到 60°、最小转弯半径 0.09 m 也跑不完 ——
见 `_project_sticky` 的说明），而是**投影在折返交汇处选错了段**，
连带把 Mission 的推进判据、前视点、前进/倒车决策全部带偏。四处改动（互补，缺一不可）：

1. **投影候选比距离**（`_candidates` / `_pick_candidate`）：原 `Path.project` 的
   "先到先得"会让**被夹在段端点的候选**赢过真正最近的点 —— 车头已伸过顶点，
   投影却停在顶点上（`dist_to_vertex = 0` → 前视点被转角钳制塌缩 → 舵角退化）。
2. **任务段下界 `s_min`**：投影不许落到 Mission 已经走过的段上（任务给段、定位给弧长）。
3. **行进方向裁决 `motion_heading`**：折返压线时同一物理点到两条段**等距**，
   位置裁决不了，只能用"车实际往哪边走"（取信念位姿的**实测位移方向**，
   不是车头朝向 —— 过弯时车头几乎垂直于路线）。
4. **段选择滞回 `_SEG_STICKY_M`**：投影是**跟踪**（上一拍那段是强先验），
   不是每拍从零重算，否则在四段交汇的 `B6` 处逐拍翻转。
   配套：`_clamp_forward` 增加了"车还在上一段时不许跨段钳制"的分支（假钉住），
   目标推进增加**朝向门**（进 `arrival_m` 球但车头还没朝着下一段时不推进），
   末点的"已过点即放行"改用 `_GOAL_STOP_M`（让 3 cm 停车门真正生效）。
"""

from __future__ import annotations

import bisect
import csv
import math
import os
from collections import deque
from dataclasses import dataclass, field

from ..core import geometry as geo
from ..core.config import Config, lookahead_distance
from ..pursuit.controller import brake_command, compute_command
from ..pursuit.path import Path, Projection
from ..route import physics as ph
from ..route.model import Route

# ⚠️ 刻意不做的事（t9 复核定稿，别再当 bug 报）：
#   本仿真**不建模 PIVOT**（状态机在 tasks/tracker.py，sim 不 import tasks/）。
#   后果：① config 里所有 `pivot_*` 键在本仿真**恒为 no-op**（推极端也不改轨迹）；
#         ② 由 180° 折返点主导的指标（A6/A1 大弧、横向 MAX 等）**不可迁移实车**；
#         ③ 千万别用本仿真验证/调 PIVOT 行为 —— 只能实车或闭环 bench 验。
#   对照实验卫生：两次运行之间不许改 config；各臂显式 --set 被测键（t9/n8 教训）。

# --------------------------------------------------------------------------- #
#  ★ 场地物理要素（t24，2026-10-11）：把 `route/physics.py` 的口径接进仿真
# --------------------------------------------------------------------------- #
# 分层（单一真源，不许分叉）：
#     `config/robot.yaml` 存**数**  →  `route/physics.py` 存**公式**  →  本节只**调用**
#
# ⚠️ 本节的硬约束：**只观测、不改动力学**。同一路线、同一注入参数下，加不加场地，
#    真值/信念轨迹必须**逐位不变**（t24 有硬断言）—— 否则"换一套判定"就不可比，
#    历史上所有基线（RMS 0.1168 / 6751 拍 …）全部失效。
#    需要"撞上就停"的对照时，显式开 `SimConfig.stop_on_field_violation=True`。
#
# 判据（用户 2026-10-11 现场澄清，以澄清为准）：
#   · 障碍：圆柱 r=0.1275、柱心在障格格心；净空门槛 `r + 车半宽 + margin` = 0.215 m
#           （拐角顶点用扫掠半径 ⇒ 0.3029 m）；**物理碰撞**另算"车体矩形 ∩ 圆柱"。
#   · 雷  ：12×12 cm 垫片、心在雷格格心；盖雷 = **车心 3.5 cm 圆 ∩ 垫片**（替换旧半径口径）。
#   · 挡板：385×2 cm 细条、第 6/7 行之间 D..N 列（含 K 段，**无洞**）；
#           车心到中心线 ≥ 挡板半厚 + 车半宽 = 0.0775 m，不得跨越。
#   · K 列：通格横向贯通；**纵向相邻格之间全部封死**（`physics.k_cells_sealed_along_column`）。
#   · 靶板：35×35 cm/格、心在格心 ⇒ 车体 28×13.5 不许与靶板矩形 overlap。
#   · 车体：28 × 13.5 cm（**注意**：宽度由 17 cm 更正为 13.5 cm —— 老后处理脚本按 17 cm 算，偏保守）。
#   · 越界：**维持现口径**（`pursuit.map_safety_margin_m`，本卡不动）。


@dataclass(frozen=True)
class FieldSpec:
    """一张 `config/map.yaml` 的物理要素（只读；`None` = 空场地 = 老行为）。"""

    mines: tuple[str, ...] = ()
    obstacles: tuple[str, ...] = ()
    targets: tuple[str, ...] = ()
    bridges: tuple[str, ...] = ()
    barrier_col_from: str = "D"
    barrier_row_hi: int = 6
    barrier_row_lo: int = 7
    name: str = ""
    source: str = ""

    def describe(self) -> str:
        return (f"场地[{self.name or '未命名'}] 雷{len(self.mines)} 障{len(self.obstacles)} "
                f"靶{len(self.targets)} 通{len(self.bridges)} "
                f"隔板 {self.barrier_col_from}~N @{self.barrier_row_hi}/{self.barrier_row_lo} 行之间")


def _strip_comment(line: str) -> str:
    """去掉行尾注释，但**不碰引号内**的 `#`（地图名里有 `#` 也不许被切掉）。"""
    out, in_quote = [], False
    for ch in line:
        if ch == '"':
            in_quote = not in_quote
        elif ch == "#" and not in_quote:
            break
        out.append(ch)
    return "".join(out).rstrip()


def _parse_inline_list(value: str) -> tuple[str, ...]:
    """`["A1", B2]` → `("A1", "B2")`（去引号、去空白、丢空项）。"""
    body = value.strip()
    if body.startswith("["):
        body = body[1:]
    if body.endswith("]"):
        body = body[:-1]
    items = []
    for raw in body.split(","):
        item = raw.strip().strip('"').strip("'").strip()
        if item:
            items.append(item.upper())
    return tuple(items)


def parse_map_yaml(text: str, *, source: str = "") -> FieldSpec:
    """把 `config/map.yaml` 的文本解析成 `FieldSpec`（**纯函数**，不读文件）。

    ★ 为什么不用 PyYAML：本模块要能在**没有 yaml 的环境**里工作（车端最小镜像 / CI）。
      只解析 `map:` 段下真正要用的键：标量（name / barrier_col_from / barrier_row_hi /
      barrier_row_lo）与列表（mines / obstacles / targets / bridges，行内 `[..]` 或块状
      `- X` 两种写法都支持）。其余键（start/end）本仿真不用 —— 起终点来自路线。
    """
    spec: dict[str, object] = {}
    inside = False
    pending_list: str | None = None
    for raw in str(text).splitlines():
        line = _strip_comment(raw)
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        stripped = line.strip()
        if indent == 0:
            if stripped.startswith("map:") or stripped == "map:":
                inside = True
                pending_list = None
                continue
            if inside:                      # 顶层下一个键 = `map:` 段结束
                break
            continue
        if not inside:
            continue
        if stripped.startswith("- ") or stripped == "-":
            if pending_list:                # 块状列表续行
                cur = list(spec.get(pending_list, ())) if isinstance(spec.get(pending_list), tuple) else []
                item = stripped[1:].strip().strip('"').strip("'").strip()
                if item:
                    cur.append(item.upper())
                spec[pending_list] = tuple(cur)
            continue
        if ":" not in stripped:
            continue
        key, _, value = stripped.partition(":")
        key, value = key.strip(), value.strip()
        if value:
            if value.startswith("["):
                spec[key] = _parse_inline_list(value)
            else:
                spec[key] = value.strip('"').strip("'")
            pending_list = None
        else:
            spec[key] = ()
            pending_list = key
    return FieldSpec(
        mines=tuple(spec.get("mines", ())),
        obstacles=tuple(spec.get("obstacles", ())),
        targets=tuple(spec.get("targets", ())),
        bridges=tuple(spec.get("bridges", ())),
        barrier_col_from=str(spec.get("barrier_col_from", "D") or "D").strip().upper(),
        barrier_row_hi=int(spec.get("barrier_row_hi", 6) or 6),
        barrier_row_lo=int(spec.get("barrier_row_lo", 7) or 7),
        name=str(spec.get("name", "") or ""),
        source=source,
    )


def _repo_root() -> str:
    """按 `__file__` 上溯找仓库根（判据：存在 `config/map.yaml`）。找不到返回空串。"""
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        cand = os.path.join(here, "config", "map.yaml")
        if os.path.isfile(cand):
            return here
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent
    return ""


def load_field_spec(map_path: str | os.PathLike | None = None, *,
                    cfg_source: str | None = None) -> FieldSpec | None:
    """读地图文件 → `FieldSpec`。**本模块唯一的读文件入口**（`run_sim()` 自己不调它）。

    ★ 为什么要单独一个函数：`run_sim()` 是纯函数（不读文件）这条不变量是有测试钉住的，
      而"仿哪张图"是**环境**而不是**参数**。所以把 I/O 放在这里，由调用方（或测试）
      显式取好再传进去 —— 顺便让"同一份 spec 跑两条臂"成为可能（对照实验卫生）。

    查找顺序：显式 `map_path` > 环境变量 `ROBOT_MAP` > `cfg_source` 同目录的 `map.yaml`
    > 仓库 `config/map.yaml`。都找不到返回 `None`（= 不建模任何地图要素，老行为）。

    ⚠️ **显式给路径时不再回退**：路径不存在就返回 `None` —— 悄悄改用另一张地图比报错危险
      （"我以为跑了新图，其实跑的是 config/map.yaml"）。
    """
    if map_path:
        cand = str(map_path)
        if not os.path.isfile(cand):
            return None
        with open(cand, encoding="utf-8") as fh:
            return parse_map_yaml(fh.read(), source=os.path.abspath(cand))
    cands: list[str] = []
    env = os.environ.get("ROBOT_MAP")
    if env:
        cands.append(env)
    if cfg_source:
        cands.append(os.path.join(os.path.dirname(os.path.abspath(cfg_source)), "map.yaml"))
    root = _repo_root()
    if root:
        cands.append(os.path.join(root, "config", "map.yaml"))
    for cand in cands:
        if cand and os.path.isfile(cand):
            with open(cand, encoding="utf-8") as fh:
                return parse_map_yaml(fh.read(), source=os.path.abspath(cand))
    return None


def body_corners(x: float, y: float, yaw: float, *, length_m: float = 0.28,
                 width_m: float = 0.135) -> tuple[tuple[float, float], ...]:
    """车体矩形的 4 个角（地图系，逆时针）。★ 车体半长/半宽来自 config，不写死。"""
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = length_m / 2.0, width_m / 2.0
    return tuple((x + dx * c - dy * s, y + dx * s + dy * c)
                 for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)))


def body_point_clearance_m(x: float, y: float, yaw: float, px: float, py: float, *,
                           length_m: float = 0.28, width_m: float = 0.135) -> float:
    """**点 P 到车体矩形**的最近距离（m；点在车内 = 0）。

    ★ 做法：把 P 变到**车体系**，再套 `physics.point_rect_distance_m()` 的同一个公式 ——
      是"换坐标系"，不是"另写一套几何"（两份实现迟早分叉，t22 的教训）。
    """
    dx, dy = px - x, py - y
    c, s = math.cos(yaw), math.sin(yaw)
    u = dx * c + dy * s              # 沿车长方向
    v = -dx * s + dy * c             # 沿车宽方向
    ou = abs(u) - length_m / 2.0
    ov = abs(v) - width_m / 2.0
    if ou <= 0.0 and ov <= 0.0:
        return 0.0
    return math.hypot(max(ou, 0.0), max(ov, 0.0))


@dataclass
class FieldObs:
    """场地物理观测累加器：每拍喂一次**真值**位姿，最后取指标。

    ★ 只用真值：地图上的物理事实不能跟着里程计漂移走，否则"压到雷"会随 `ε` 变化，
      指标就不可复现（与"分组统计用真值"同一原则）。

    ★ 只观测不改动力学：`stop_on_violation=False`（默认）时本类对轨迹**零影响**。
    """

    spec: FieldSpec
    length_m: float = 0.28
    width_m: float = 0.135
    coverage_radius_m: float = 0.035
    obstacle_radius_m: float = 0.1275
    mine_pad_m: float = 0.12
    barrier_width_m: float = 0.02
    barrier_cols: str = "D..N"
    obstacle_margin_m: float = 0.02
    stop_on_violation: bool = False          # True = 第一次物理违规即结束（对照组用）

    def __post_init__(self) -> None:
        self.mine_xy = {b: geo.block_to_meters(b) for b in self.spec.mines}
        self.obs_xy = {b: geo.block_to_meters(b) for b in self.spec.obstacles}
        self.tgt_xy = {b: geo.block_to_meters(b) for b in self.spec.targets}
        self.open_rows = sorted({geo.block_to_index(b)[0] for b in self.spec.bridges})
        self.k_col = geo.COL_LETTERS.index("K")
        c0, c1 = ph.parse_col_span(self.barrier_cols)
        self.bar_x, self.bar_y0, self.bar_y1 = ph.barrier_centerline(
            col_from=c0, col_to=c1, row_hi=self.spec.barrier_row_hi,
            row_lo=self.spec.barrier_row_lo)
        self.bar_need = ph.required_barrier_distance_m(
            barrier_width_m=self.barrier_width_m, car_width_m=self.width_m)
        self.board_half = ph.board_half_m()
        # 累加量
        self.press_steps: dict[str, int] = {b: 0 for b in self.spec.mines}
        self.mine_best: dict[str, tuple[float, float]] = {b: (9e9, 0.0) for b in self.spec.mines}
        self.obs_hit_steps = 0
        self.obs_min_center: tuple[float, str, float] = (9e9, "", 0.0)
        self.obs_breach_normal = 0
        self.obs_breach_corner = 0
        self.bar_min: tuple[float, float] = (9e9, 0.0)
        self.bar_breach_steps = 0
        self.board_overlap_steps = 0
        self.board_breach_steps = 0
        self.board_min: tuple[float, str, float] = (9e9, "", 0.0)
        self.k_col_steps = 0
        self.k_seal_steps = 0
        self.k_solid_steps = 0
        self.steps = 0
        self.first: dict[str, tuple[float, str, float]] = {}
        self._prev_row: int | None = None
        self._prev_col: int | None = None
        self._prev_in_k = False
        self._through_row: int | None = None

    # ---- 内部：记录"第一次违规"（报告要用它定位；只报事实，不改流程）----
    def _first(self, key: str, t: float, block: str, value: float) -> None:
        self.first.setdefault(key, (t, block, value))

    def observe(self, t: float, x: float, y: float, yaw: float, *, kind: str = "normal") -> bool:
        """观测一拍（真值位姿）。返回 True 表示本拍出现**物理违规**（供对照组提前结束）。"""
        self.steps += 1
        violated = False
        # ---- ① 盖雷：车心 3.5 cm 圆 ∩ 12×12 垫片 ----
        for block, (mx, my) in self.mine_xy.items():
            clear = ph.mine_clearance_m(x, y, mx, my, mine_pad_m=self.mine_pad_m)
            if clear < self.mine_best[block][0]:
                self.mine_best[block] = (clear, t)
            if clear <= self.coverage_radius_m + 1e-12:
                self.press_steps[block] += 1
        # ---- ② 障碍：物理碰撞（车体 ∩ 圆柱）+ 净空门槛（直段/拐角两档） ----
        need = ph.required_obstacle_center_distance_m(
            obstacle_radius_m=self.obstacle_radius_m,
            body_radius_m=(ph.vehicle_half_diagonal_m(self.length_m, self.width_m)
                           if kind == "corner" else self.width_m / 2.0),
            margin_m=self.obstacle_margin_m)
        for block, (ox, oy) in self.obs_xy.items():
            dist = ph.obstacle_clearance_point_m(x, y, ox, oy)
            if dist < self.obs_min_center[0]:
                self.obs_min_center = (dist, block, t)
            if body_point_clearance_m(x, y, yaw, ox, oy, length_m=self.length_m,
                                      width_m=self.width_m) <= 0.0:
                self.obs_hit_steps += 1
                self._first("障碍碰撞(车体∩圆柱)", t, block, dist)
                violated = True
            if dist < need - 1e-12:
                if kind == "corner":
                    self.obs_breach_corner += 1
                else:
                    self.obs_breach_normal += 1
                self._first("障碍净空不足", t, block, dist)
        # ---- ③ 挡板：车心到中心线 ≥ 半厚+车半宽 ----
        bar_d = ph.barrier_distance_point_m(x, y, x=self.bar_x, y0=self.bar_y0, y1=self.bar_y1)
        if bar_d < self.bar_min[0]:
            self.bar_min = (bar_d, t)
        if bar_d < self.bar_need - 1e-12:
            self.bar_breach_steps += 1
            self._first("挡板触线", t, "隔板", bar_d)
            violated = True
        # ---- ④ 靶板：**两级**（与障碍同构，两件事分开数）----
        #   ① `board_overlap_steps`：**物理重叠**（精确的"旋转后车体矩形 ∩ 35×35 靶板矩形"= 0）
        #      —— 这是硬事实，车真的压上去了；
        #   ② `board_breach_steps`：**设计判据不足**（间隙 < 车半宽 0.0675 / 拐角 0.1554）
        #      —— 与 route_check/t22 同一条规则，只是这里喂的是**实测轨迹**的间隙。
        #   ⚠️ 不要把 physics.target_body_overlap() 套在"已有精确车体距离"上：那个函数
        #      的输入是**折线/顶点**的间隙（把扫掠包络折算成间隙门槛），套在精确车体距离上
        #      会把扫掠算两遍、虚报违规。
        #   ★ 距离算法：min(车体 4 条边 → 靶板矩形)，边与矩形相交时
        #      `segment_rect_distance_m` 直接返回 0（Liang–Barsky）；靶板比车体大，
        #      不存在"矩形整个包在车体里"的退化 ⇒ 这个 min 是**精确**的。
        corners = body_corners(x, y, yaw, length_m=self.length_m, width_m=self.width_m)
        edges = list(zip(corners, corners[1:] + corners[:1]))
        need_board = ph.required_target_body_clearance_m(
            length_m=self.length_m, width_m=self.width_m, at_corner=(kind == "corner"))
        for block, (cx, cy) in self.tgt_xy.items():
            clear = min(ph.segment_rect_distance_m(ax, ay, bx, by, cx, cy, self.board_half)
                        for (ax, ay), (bx, by) in edges)
            if clear < self.board_min[0]:
                self.board_min = (clear, block, t)
            if clear <= 0.0:
                self.board_overlap_steps += 1
                self._first("车体压靶板(物理重叠)", t, block, clear)
                violated = True
            if clear < need_board - 1e-12:
                self.board_breach_steps += 1
                self._first("靶板净空不足(设计判据)", t, block, clear)
        # ---- ⑤ K 列桥室：非通格 = 实体；纵向相邻格之间封死 ----
        row_idx, col_idx = geo.block_to_index(geo.meters_to_block(x, y))
        if col_idx == self.k_col:
            self.k_col_steps += 1
            if row_idx not in self.open_rows:
                self.k_solid_steps += 1
                self._first("K 列非通格", t, geo.index_to_block(row_idx, col_idx), 0.0)
                violated = True
            if self._prev_row is not None and self._prev_col == self.k_col \
                    and row_idx != self._prev_row and ph.k_cells_sealed_along_column():
                self.k_seal_steps += 1
                self._first("K 列纵向挪格(封死)", t, geo.index_to_block(row_idx, col_idx), 0.0)
                violated = True
            self._through_row = row_idx
        elif self._prev_col == self.k_col and self._through_row is not None:
            if self._through_row not in self.open_rows:      # 从非通格横穿出去
                self.k_seal_steps += 1
                self._first("K 列横向穿越(非通格)", t, geo.index_to_block(self._through_row,
                                                                    self.k_col), 0.0)
                violated = True
        self._prev_row, self._prev_col = row_idx, col_idx
        return violated

    def metrics(self) -> dict[str, float]:
        """汇总指标（键名只增不改；`field_*` 前缀 = 本卡新增）。"""
        pressed = sorted(b for b, n in self.press_steps.items() if n > 0)
        out: dict[str, float] = {
            "field_modeled": 1.0,
            "mine_total": float(len(self.spec.mines)),
            "mine_pressed_count": float(len(pressed)),
            "mine_press_steps": float(sum(self.press_steps.values())),
            "mine_worst_clearance_m": (max((v[0] for v in self.mine_best.values()), default=0.0)),
            "obstacle_hit_steps": float(self.obs_hit_steps),
            "obstacle_breach_steps": float(self.obs_breach_normal + self.obs_breach_corner),
            "obstacle_breach_corner_steps": float(self.obs_breach_corner),
            "obstacle_min_center_dist_m": float(self.obs_min_center[0]),
            "barrier_min_distance_m": float(self.bar_min[0]),
            "barrier_breach_steps": float(self.bar_breach_steps),
            "board_overlap_steps": float(self.board_overlap_steps),
            "board_breach_steps": float(self.board_breach_steps),
            "board_min_clearance_m": float(self.board_min[0]),
            "k_column_steps": float(self.k_col_steps),
            "k_seal_violation_steps": float(self.k_seal_steps),
            "k_solid_cell_steps": float(self.k_solid_steps),
        }
        return out


@dataclass(frozen=True)
class SegmentStat:
    """**逐段横向误差峰值**一行（定位"是哪几个拐角在吃误差"的唯一手段）。

    ★ 为什么要它：整条路线的 `cross_track_rms_m` 是一个标量，0.20 m 到底是
      "全程均匀偏 0.2" 还是"29 个直段完美 + 3 个拐角各偏 1.0 m"？
      两者的修法完全不同（前者改前视距离，后者改转角策略）。不看这张表就没法调参。
    """

    index: int              # 段索引（= 航点 index → index+1）
    from_block: str
    to_block: str
    length_m: float
    kind: str               # "normal" | "reverse"（该段自身的类型）
    peak_m: float           # 该段上真值横向误差峰值
    peak_t: float           # 峰值时刻（s）
    peak_s: float           # 峰值处弧长（m）
    corner_deg: float       # 该段末端顶点的转角（度）


__all__ = [
    "SimConfig",
    "SimResult",
    "SegmentStat",
    "run_sim",
    "step_bicycle",
    "write_csv",
    "write_segments_csv",
    "write_svg",
    "format_metrics",
    "format_segment_table",
    "METRIC_KEYS",
    "GROUPED_METRIC_KEYS",
]

# --------------------------------------------------------------------------- #
#  算法常数（**不是可调参数**：它们定义"怎么算"，不定义"车怎么动"）
# --------------------------------------------------------------------------- #
#: 投影搜索窗口（米）—— **前宽后窄**，两个数各有原因：
#: · 前视 0.80 m（约 2.3 格）：过弯时车会先于投影点向前走，窗口太小会锁不住而"绕圈"。
#: · 后视 0.20 m：路线会**自交**（现场路线在 F10/F11 附近折返、G2/B6 附近重叠），
#:   后视窗口一大，投影就会锁到路线的另一条边（实测：车在第一段就被锁到第 11 段，
#:   距离 0.15 m 却完全不再更新，随后一路开出地图）。
#: 倒车段（`R:`）需要后视，但 0.20 m = 半格多，足够覆盖倒车时的正常回退。
_PROJ_FWD_M = 0.80
_PROJ_BACK_M = 0.20

#: 信念投影的窗口（米）：**以物理投影弧长为锚 ± 这么宽**。
#: ⚠️ 不能只给几十厘米：belief 与真值可能相差一个格子（0.35 m）甚至更多，
#:   窗口太小 → 投影锁在窗口边界上不再更新 → 前视点冻结 → 舵角归零 → 车直着开走
#:   （实测：窗口 0.2 m 时 belief 投影卡在 s=0.700 不动，车 26.9 s 就脱离路线）。
#: ⚠️ 也不能给太大：路线自交处"最近点"会落到别的航段上，信念会以为自己在前面 1 m，
#:   控制器于是反向打舵绕圈（实测：窗口放到自由搜索时，误差注入下 41 s 就跑飞）。
_PROJ_BELIEF_M = 1.50

#: 行进方向估计的**最低位移**（米/拍）：信念位姿本拍与上拍的位移小于它就认为"没在走"，
#: 保留上一次的方向估计（停车、原地摆头时位移是数值噪声，不是方向）。
#: ⚠️ 取 `pursuit.min_speed_mps`(0.08) × `dt`(0.02) = 1.6 mm 的**百分之一**：
#:   只要车真的在动（哪怕 1% 的蠕动速度）就用实测位移，只有完全静止才沿用旧值。
_MH_MIN_STEP_M = 1.6e-5

#: 段选择的**切换滞回**（米）：新的候选段要比"上一拍命中的段"近这么多才允许换段。
#: ★ 为什么必须有：现场图在 `B6` 一点上有**四条段交汇**（`B8→B6` 与 `B6→D6` 同沿 x=…，
#:   `B6→A6` 与 `A6→B6` 沿同一条竖线折返），车停在交点附近时多个候选**等距**，
#:   而 `s_hint` 在两个候选之间来回跳 → 投影段逐拍翻转 → 指令在前进/倒车间翻转
#:   → 车在交点原地摆动（实测 2000 拍、`s` 在 7.58↔8.40 之间跳）。
#:   滞回让"已经在哪一段"成为**默认答案**，只有真的更近才换 —— 这与现实的
#:   Localization 一致：投影是**跟踪**（上一拍的位置是强先验），不是每拍从零重算。
#: ⚠️ 取值 2 cm ≈ 一个控制周期走的路（0.12 m/s × 0.02 s = 2.4 mm）的 8 倍：
#:   足以压住"等距抖动"，又远小于一格（0.35 m），不会锁死在已经走过的段上。
_SEG_STICKY_M = 0.02

#: 目标推进的**朝向门**容差（度）：车到位后，车头与**下一段走向**的夹角超过它就
#: 先不推进目标（继续把当前段走完 / 原地把车头转过来）。
#: ★ 只用来挡"掉头"（下一段与当前段反向 = 差 180°），现场图 90° 拐角必须照常放行，
#:   所以容差取 90°：90° 拐角恰好卡在边界上（`<=` 放行），180° 掉头一定被挡。
_ADVANCE_FACING_TOL_DEG = 90.0

#: "投影卡死"判据：连续这么多拍投影弧长不再前进且横向距离持续变大 → 判定跑飞。
_DIVERGE_STEPS = 25

#: 拐角判据容差（度）：夹角大于它就认为该航点是"拐角顶点"，不做到位吸附（见 `_anchor_allowed`）。
_ANCHOR_CORNER_TOL_DEG = 5.0

#: 终点停车阈值（米）：真值进入终点的这个半径内才认"到了"并刹车。
#: 比 `pursuit.arrival_m`（0.12）更紧：不然车会在离终点 0.12 m 处停下，
#: `final_error_m` 看起来像"停不到位"，而实际是判据太松。
_GOAL_STOP_M = 0.03

#: 位置噪声上限（米）。现实中定位不会瞬间跳半个场地；超过它属于"注入故障"而非"误差"。
_POS_NOISE_LIMIT_M = 0.50

#: 停止推进的信念误差阈值（米）。信念投影已到末点 ±1 µm 才认定"跑完了"。
_END_EPS_M = 1e-6

#: 分组用的"顶点邻域"宽度（米）：距某个顶点这么近的拍算 `corner` 组。
#: 取 0.35 m = **场地一格** —— 现场图 90° 拐角的两条腿各只有一格长，
#: 取半格会让"拐角吃掉的误差"漏掉一半（车在顶点前后各半格都在还账）。
_CORNER_ZONE_M = 0.35

#: 执行侧舵角"跳变"告警阈值（度）：单拍执行舵角变化超过它才记 `steer_delivered_jump_steps`。
#: ★ 正常模型下这个计数必须为 **0**（执行侧是斜坡跟随，物理上不可能跳），
#:   所以它是一个"模型自检"指标而不是调参指标。
_DELIVERED_JUMP_DEG = 1e-9

#: SVG 画布尺寸（像素）与四边留白。
_SVG_W = 1000.0
_SVG_H = 800.0
_SVG_PAD = 56.0

#: 路线折线 / 航点 / 真值轨迹的描边（手写 SVG，不依赖 matplotlib）。
_COLOR_ROUTE = "#9aa4b2"
_COLOR_ROUTE_W = 1.4
_COLOR_WAYPOINT = "#4c6ef5"
_COLOR_WAYPOINT_STOP = "#e8590c"      # 射击/扫描停站点
_COLOR_TRUTH_W = 2.2

#: 不同误差参数下的轨迹配色（按当前误差从"准"到"偏"取色，肉眼可分辨）。
_COLOR_BY_ERROR = [
    (0.02, "#2f9e44"),      # 很准 —— 绿
    (0.08, "#1971c2"),      # 尚可 —— 蓝
    (0.20, "#f08c00"),      # 偏差明显 —— 橙
]
_COLOR_WORST = "#c92a2a"    # 偏得离谱 —— 红


@dataclass
class SimConfig:
    """仿真注入参数（默认 = "理想车"：无误差、无延迟、有锚定）。

    默认值与 `docs/P0接口约定.md` §5 逐条一致；现场复盘时按实测填即可。
    """

    dt: float = 0.02                    # 控制周期（50 Hz）
    cmd_latency_s: float = 0.06         # 指令到执行延迟（链路 + 下位机解析）
    speed_scale_err: float = 0.0        # 里程比例误差（+0.18 = 里程计比真实多报 18%）
    yaw_drift_deg_per_min: float = 0.0  # 航向零漂（陀螺积分）
    pos_noise_m: float = 0.0            # 位置白噪声（每拍叠加）
    anchor_on_arrival: bool = True      # 到位吸附（模拟 FixTask 的 L2 位置修正）
    anchor_radius_m: float = 0.12       # 吸附触发半径（默认取 `pursuit.arrival_m` 现场值）
    max_steps: int = 0                  # 0 = 按路线长度自动推算（见 `auto_max_steps`）
    seed: int = 20261007                # 白噪声种子（固定种子 = 可复现）
    trace_every: int = 1                # 每 N 拍记一条轨迹（1 = 全记）
    stop_on_completion: bool = True     # 到终点即停（False = 跑到步数上限，用来看"冲过去"）
    bounds_margin_m: float = 0.0        # 越界判据余量；0 = 用 `pursuit.map_safety_margin_m`
    init_yaw: str = "route"             # "route" = 对齐首段方向（默认）| "waypoint" = 首点 yaw 字段
    actuator_rate_limit: bool = True    # 执行侧舵角斜坡跟随（关掉 = 复位老实现"一步到位"）
    command_rate_limit: bool = True     # 控制器侧速率限幅（关闭仅用于"暴露未限幅后果"的对照）
    trace_hook: object = None           # 可选：每拍回调 (step:int, state:dict) —— 排障用（默认 None）
    # ★ t24：场地物理观测（默认关 —— 只有显式传入 FieldSpec 才启用，见 run_sim 的 field 形参）。
    #   True = 本拍出现物理违规（压到雷以外的硬违规：障碍碰撞/挡板触线/压靶板/K 列封死）即结束。
    #   ⚠️ 刻意**默认 False**：加场地不许改轨迹（历史基线可比），"撞上就停"是对照组实验。
    stop_on_field_violation: bool = False

    def validate(self) -> None:
        """参数自检：非法值**立刻报错**，不留给"跑了半天才发现 dt=0"。"""
        if not (self.dt > 0.0):
            raise ValueError(f"dt 必须 > 0，收到 {self.dt!r}")
        if self.cmd_latency_s < 0.0:
            raise ValueError(f"cmd_latency_s 不能为负，收到 {self.cmd_latency_s!r}")
        if self.speed_scale_err <= -1.0:
            raise ValueError(f"speed_scale_err 必须 > -1（-1 = 里程计完全不动），收到 {self.speed_scale_err!r}")
        if self.pos_noise_m < 0.0:
            raise ValueError(f"pos_noise_m 不能为负，收到 {self.pos_noise_m!r}")
        if self.pos_noise_m > _POS_NOISE_LIMIT_M:
            raise ValueError(f"pos_noise_m 超过注入上限 {_POS_NOISE_LIMIT_M} m，收到 {self.pos_noise_m!r}")
        if self.anchor_radius_m <= 0.0:
            raise ValueError(f"anchor_radius_m 必须 > 0，收到 {self.anchor_radius_m!r}")
        if self.trace_every < 1:
            raise ValueError(f"trace_every 必须 >= 1，收到 {self.trace_every!r}")
        if self.max_steps < 0:
            raise ValueError(f"max_steps 不能为负，收到 {self.max_steps!r}")
        if self.init_yaw not in ("route", "waypoint"):
            raise ValueError(f"init_yaw 只能是 'route' / 'waypoint'，收到 {self.init_yaw!r}")


@dataclass
class SimResult:
    """一次仿真的全部采样。`x/y/yaw/v/steer` = **信念**（控制器看到的），`truth_*` = 真值。

    ★ 每组数组长度一致 = 采样拍数（`steps`）；逐拍追加，`trace_every > 1` 时按拍抽稀。
    ⚠️ `steer` 是**信念侧看到的舵角**（与老实现同义，= 执行舵角），
      `steer_cmd` 才是控制器**输出**的舵角 —— 两者差一个执行侧斜坡，
      混淆它们正是老实现"指令速率 680.9 °/s"假指标的来源。
    """

    t: list[float] = field(default_factory=list)
    x: list[float] = field(default_factory=list)
    y: list[float] = field(default_factory=list)
    yaw: list[float] = field(default_factory=list)
    v: list[float] = field(default_factory=list)
    steer: list[float] = field(default_factory=list)
    truth_x: list[float] = field(default_factory=list)
    truth_y: list[float] = field(default_factory=list)
    truth_yaw: list[float] = field(default_factory=list)
    seg_index: list[int] = field(default_factory=list)
    s: list[float] = field(default_factory=list)
    # ---- 诊断列（只增不改；CSV/SVG/测试都用得上）----
    truth_seg_index: list[int] = field(default_factory=list)   # 真值投影段（分组统计用）
    truth_s: list[float] = field(default_factory=list)         # 真值投影弧长
    steer_cmd: list[float] = field(default_factory=list)       # 控制器**输出**舵角（未执行）
    anchored: list[bool] = field(default_factory=list)         # 这一拍信念被到位吸附了吗
    near_vertex: list[bool] = field(default_factory=list)      # 这一拍在顶点邻域内吗
    reverse: list[bool] = field(default_factory=list)          # 这一拍在倒车段吗
    metrics: dict = field(default_factory=dict)
    segments: list[SegmentStat] = field(default_factory=list)
    #: ★ t24：场地物理观测明细（逐雷盖雷拍数 / 最近距离 / 首次违规位置 …）；无场地时 = {}。
    #:   刻意不复用 `metrics`：那里只放**数值**（CSV/表都按数值格式化），明细走这里。
    field_report: dict = field(default_factory=dict)

    @property
    def steps(self) -> int:
        return len(self.t)

    def metric(self, key: str, default: float = float("nan")) -> float:
        """取指标（缺键返回 default，便于测试断言时给出可读的实测值）。"""
        return self.metrics.get(key, default)

    def series(self, name: str) -> list:
        """按名字取逐拍序列（`x`/`steer`/`steer_cmd`/`anchored`/…），缺名抛 KeyError。"""
        if not hasattr(self, name):
            raise KeyError(f"SimResult 没有逐拍序列 {name!r}")
        return getattr(self, name)


# --------------------------------------------------------------------------- #
#  运动学 + 指标
# --------------------------------------------------------------------------- #
def step_bicycle(x: float, y: float, yaw: float, v: float, steer_deg: float,
                 dt: float, wheelbase_m: float) -> tuple[float, float, float]:
    """自行车模型推进一步（欧拉积分，与下位机同构）。

    ★ 真值唯一的状态推进函数：单元测试直接调它，闭环与它对拍。
    ⚠️ `steer_deg` 是**度**（与 `PursuitCommand.steer_deg` 契约一致），内部转弧度。
    """
    delta = math.radians(steer_deg)
    nx = x + v * math.cos(yaw) * dt
    ny = y + v * math.sin(yaw) * dt
    if abs(wheelbase_m) > 1e-12:
        nyaw = yaw + (v / wheelbase_m) * math.tan(delta) * dt
    else:                                   # 轴距为 0 = 不能转向（防御，配置校验本会拦住）
        nyaw = yaw
    return nx, ny, geo.normalize_angle(nyaw)


def _dist_point_seg(px: float, py: float, ax: float, ay: float,
                    bx: float, by: float) -> float:
    """点到线段的距离（线段退化时退化为点距）。"""
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    if den <= 1e-18:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / den
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return math.hypot(px - (ax + dx * t), py - (ay + dy * t))


def _percentile(values: list[float], q: float) -> float:
    """线性插值分位数（numpy 的 `linear` 口径）—— 不引 numpy 也要能报 P95。

    ★ 为什么必须报分位数：**峰值**由一两拍异常支配（一次符号翻转就能拉满），
      调参时要看的是"95% 的拍都在多少以下"，否则永远在追噪声。
    """
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    pos = (len(ordered) - 1) * max(0.0, min(1.0, q))
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(ordered) - 1)
    frac = pos - lo
    return float(ordered[lo] * (1.0 - frac) + ordered[hi] * frac)


def _ramp_towards(current: float, target: float, max_delta: float) -> float:
    """按最大步长向目标靠拢（执行侧舵机斜坡的**唯一**实现）。"""
    delta = target - current
    if delta > max_delta:
        delta = max_delta
    elif delta < -max_delta:
        delta = -max_delta
    return current + delta


def _cross_track_m(x: float, y: float,
                   segments: tuple[tuple[float, float, float, float, float, float], ...]) -> float:
    """点到路线折线的最近距离（横向误差的**定义**，truth 与 belief 各算一份）。"""
    best = float("inf")
    for x0, y0, x1, y1, _length, _heading in segments:
        d = _dist_point_seg(x, y, x0, y0, x1, y1)
        if d < best:
            best = d
    return best if best < float("inf") else 0.0


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / float(len(values)))


def auto_max_steps(path: Path, cfg: Config, sim: SimConfig) -> int:
    """自动步数上限：按**最低速度**走完全程的 4 倍时长（再慢也不该超过它）。

    4 倍是"允许原地摆头、绕圈、倒车重来"的余量；超过它说明参数组合已经病态
    （例如锚定半径过大导致车被锚在原地），此时应当报未完成而不是无限跑。
    """
    v_min = max(1e-3, float(cfg.get("pursuit.min_speed_mps")))
    length = max(path.total_length, float(cfg.get("pursuit.lookahead_max_m")))
    return max(200, int(4.0 * length / v_min / sim.dt) + 1)


# --------------------------------------------------------------------------- #
#  主循环
# --------------------------------------------------------------------------- #
def run_sim(route: Route, cfg: Config, sim: SimConfig | None = None,
            field: FieldSpec | None = None) -> SimResult:
    """跑一次闭环仿真，返回 `SimResult`（含真值/信念两条轨迹与全部指标）。

    :param route: 路线（通常 `parse_mission(cfg.get("route.tokens"))` 或读路线文件）。
    :param cfg: 现场配置（一切 `pursuit.*` 都从它读，本函数不硬编码）。
    :param sim: 误差注入参数；None = 理想车（无误差、有锚定）。
    :param field: 场地物理要素（`load_field_spec()` 的产物）；None = **不建模任何地图要素**
        （= 老行为；此时 `metrics["field_modeled"] == 0.0`，报告里那些场地指标一律不打印）。
    """
    sim = sim or SimConfig()
    sim.validate()
    path = Path(route)

    wheelbase = float(cfg.get("pursuit.wheelbase_m"))
    arrival = float(cfg.get("pursuit.arrival_m"))
    overshoot_tol = float(cfg.get("pursuit.overshoot_tol_m"))
    margin = float(sim.bounds_margin_m or cfg.get("pursuit.map_safety_margin_m"))
    ld = lookahead_distance(cfg.get("pursuit.cruise_speed_mps"), cfg)
    dt = sim.dt
    max_steps = sim.max_steps or auto_max_steps(path, cfg, sim)
    trace_every = max(1, sim.trace_every)

    # ---- ★ 场地物理观测（t24）：公式全在 route/physics.py，参数全在 config，这里只取数 ----
    # ⚠️ 一律 `cfg.get(key, 默认)`：这 8 个键由 t22 落进 SCHEMA/robot.yaml，但**老配置**
    #    （或测试里手搓的最小配置）没有它们时也必须能跑 —— 默认值与 physics.py 的形参默认**同源**。
    obs: FieldObs | None = None
    if field is not None:
        obs = FieldObs(
            spec=field,
            length_m=float(cfg.get("vehicle.length_m", 0.28)),
            width_m=float(cfg.get("vehicle.width_m", 0.135)),
            coverage_radius_m=float(cfg.get("vehicle.coverage_radius_m", 0.035)),
            obstacle_radius_m=float(cfg.get("field.obstacle_radius_m", 0.1275)),
            mine_pad_m=float(cfg.get("field.mine_pad_m", 0.12)),
            barrier_width_m=float(cfg.get("field.barrier_width_m", 0.02)),
            barrier_cols=str(cfg.get("field.barrier_cols", "D..N")),
            obstacle_margin_m=float(cfg.get("planner.obstacle_margin_m", 0.02)),
            stop_on_violation=bool(sim.stop_on_field_violation),
        )

    # ---- 期望值（契约指标名，见 docs/P0接口约定.md §5）----
    result = SimResult()
    metrics: dict = result.metrics
    for key in METRIC_KEYS:
        metrics[key] = 0.0
    for key in GROUPED_METRIC_KEYS:
        metrics[key] = 0.0

    # ---- 初始位姿：信念 = 真值 = 路线首点 ----
    # 朝向默认取**首段方向**（`Route` 首点的 yaw 是"整条路线走完后该有的朝向"，
    # 例如现场路线末尾 `N5` 无后缀 → yaw = 0 = 朝 X+，而首段其实是朝 Y+；
    # 拿它当初朝向 = 开局就让车横着 90°，纯追踪要绕 0.3 m 半径的圆弧去救 ——
    # 那是在测"起步摆正能力"，不是测循迹质量）。要复现起步摆正，填 init_yaw="waypoint"。
    head = path.waypoints[0]
    tx, ty = head.x, head.y
    bx, by = head.x, head.y
    if sim.init_yaw == "waypoint":
        init = head.yaw
    else:
        init = head.heading_to(path.waypoints[1])
    tyaw = byaw = init
    tx0, ty0 = tx, ty          # 轨迹起点（用于"实走里程"的独立核算）

    # ================= 执行链状态（真值与信念的**同一个输入**） =================
    # ★ 三层量必须严格区分（老实现把三者混用，于是"速率限幅"看起来根本没生效）：
    #   ① `cmd_steer`   控制器**输出**（PursuitCommand.steer_deg）—— 受速率限幅约束；
    #   ② `steer_target` 已过延迟队列、**下位机收到**的舵角指令；
    #   ③ `steer_exec`  **舵机实际位置**（按舵机速率斜坡跟随 ②）—— 真值用它。
    # ★ 三者语义严格区分（老实现混用这三者，于是"速率限幅"看起来根本没生效）：
    #   ① `cmd.steer_deg`  控制器**输出**（受指令侧速率限幅约束）；
    #   ② `steer_target`   已过延迟队列、**下位机收到**的舵角指令；
    #   ③ `steer_exec`     **舵机实际位置**（按舵机速率斜坡跟随 ②）——
    #                      下一拍开头这个值会同时作用于真值与信念。
    v_exec = 0.0                # 实际执行速度（延迟队列输出；队列空时保持上一条）
    steer_target = 0.0          # 已生效的舵角指令
    steer_exec = 0.0            # 舵机实际舵角（斜坡跟随后的物理量）
    brake_hold = False          # 上一次指令是否"零速刹车"（刹车期间舵机保持原位）
    last_cmd_steer: float | None = None   # 控制器上一拍输出（速率限幅的输入）
    cmd_rate_peak = 0.0         # 指令侧速率峰值（相邻两拍**指令**之差）
    delivered_rate_peak = 0.0   # 执行侧速率峰值（相邻两拍**舵机位置**之差，正常 ≤ 限幅）
    steer_cmd_rates: list[float] = []       # 逐拍指令速率（分位数用）
    steer_delta_raw: list[float] = []       # 逐拍执行舵角变化（分位数 + 翻转识别用）
    flip_steps = 0              # 倒车符号翻转拍数（单独归类，不进速率判定）
    flip_rate_peak = 0.0        # 翻转拍的指令速率峰值（如实报告，不当作超限）
    delivered_jump_steps = 0    # 执行侧单拍跳变拍数（正常必须为 0）

    queue: deque[tuple[float, float, float]] = deque()   # (生效时刻, v, steer_deg)，按时间升序
    anchors = 0
    anchor_idx = 1              # 下一个待吸附的航点（每点只锚一次，锚完推进）
    target_idx = 1              # 任务当前目标航点（= 正在走的这一段，见第 0 步）
    s_prev = 0.0                # 上一拍的**真值**投影弧长（推进规则 ② 用）
    #: 行进方向估计（弧度，地图系）——`Path.project(..., motion_heading=...)` 的输入。
    #: ★ 为什么是一个**有状态**的量：路线折返压在同一条线上时（现场图
    #:   `B6→A6` 与 `A6→B6→D6` 在 B6 处交叉，`B8→B6` 又从同一交点进入），
    #:   同一物理点到两条段**等距**，位置裁决不了，只能靠"车正在往哪边走"。
    #:   初值取首段走向（与初始位姿同源），此后按信念位姿的实测位移更新。
    motion_heading = path.segments[0][5] if path.segment_count > 0 else 0.0
    _bx_prev, _by_prev = bx, by        # 上一拍信念位置（求实测位移方向用）
    prev_truth_seg = -1                # 上一拍真值投影命中的段（段选择滞回的先验）
    prev_belief_seg = -1               # 上一拍信念投影命中的段

    rng = _Rng(sim.seed)
    drift_per_s = math.radians(sim.yaw_drift_deg_per_min) / 60.0
    scale = 1.0 + sim.speed_scale_err
    rate_limit = float(cfg.get("pursuit.max_steer_rate_deg_s"))
    act_step = rate_limit * dt          # 执行侧每拍最大舵角变化（度）

    truth_xt: list[float] = []          # 逐步的横向误差样本（truth / belief）
    belief_xt: list[float] = []
    belief_gap: list[float] = []
    in_bounds_belief: list[float] = []
    in_bounds_truth: list[float] = []
    # ---- 分组统计（按段类型）----
    xt_by_kind: dict[str, list[float]] = {"normal": [], "reverse": [], "corner": []}
    time_by_kind: dict[str, float] = {"normal": 0.0, "reverse": 0.0, "corner": 0.0}
    steps_by_kind: dict[str, int] = {"normal": 0, "reverse": 0, "corner": 0}
    seg_peak: list[dict] = [{"peak": -1.0, "t": 0.0, "s": 0.0} for _ in path.segments]
    # "顶点冲过头"按段类型分开（老实现的 0.5552 m 里混着倒车折返，分不开就没法调参）
    vert_off: dict[str, float] = {"normal": 0.0, "reverse": 0.0}
    steps = 0
    reached_end = False
    diverged = False
    diverged_at = 0
    stall = 0
    last_s = -1.0
    t = 0.0
    field_stop_t: float | None = None    # ★ t24：stop_on_field_violation 触发时的时刻

    while steps < max_steps:
        # ================== ① 真值 + 信念：**同一执行量、同一积分器、同一拍** ==================
        # ★ 顺序是模型正确性的核心（这里踩过一次，必须写清楚）：
        #   本拍先把上一拍**已生效**的执行量 `(v_exec, steer_exec)` 同时积分到真值与信念上，
        #   然后才做决策。两个后果：
        #     · **零注入时信念与真值逐位相同**（测试硬断言）：同函数、同入参、同浮点顺序；
        #     · 控制器用的 `(bx, by, byaw)` 与真值 `(tx, ty, tyaw)` 是**同一时刻**的状态。
        #   ⚠️ 老顺序把信念积分放在拍首、用**上上拍**的执行量，真值积分放在拍尾、用当拍量
        #      —— 两者错开一个控制周期，零误差下信念也恒定落后真值 `v·dt = 3 mm`，
        #      所有注入实验都失去基准（"误差"里混着模型自己的相位差）。
        tx, ty, tyaw = step_bicycle(tx, ty, tyaw, v_exec, steer_exec, dt, wheelbase)
        bx, by, byaw = step_bicycle(bx, by, byaw, v_exec * scale, steer_exec, dt, wheelbase)
        if drift_per_s != 0.0:
            byaw = geo.normalize_angle(byaw + drift_per_s * dt)     # 陀螺零漂：速率型误差
        if sim.pos_noise_m > 0.0:
            bx += rng.gauss() * sim.pos_noise_m
            by += rng.gauss() * sim.pos_noise_m

        # ================== ①' 行进方向估计（投影等距时的唯一裁决量） ==================
        # ★ 投影的"最近点"在**折返压线**处是多解的：现场图 `B8→B6`（+Y 进入）、
        #   `B6→A6`（+Y）、`A6→B6`（-Y）、`B6→D6`（-Y）四条段在 B6 一点交汇，
        #   车停在交点附近时到两条段**完全等距**（`Path.project` 的 `_TIE_D2_M2` 容差内），
        #   位置本身裁决不了 —— 只能问"车正在往哪边走"。
        # ★ 方向取**信念位姿的实际位移**（本拍 − 上拍），不是车头朝向：
        #   过弯时车头几乎垂直于路线（现场图 `L10→L9` 处实测 yaw=174.9° 而路线走向 90°），
        #   拿车头当"往哪边走"会把投影裁决到**垂直于车的那条段**上 —— 实测第一次分歧
        #   就在 E11 顶点：车头朝西、实际向北，投影被裁决到 `E11→F11`（倒车段，朝南），
        #   `s` 从 3.978 一步跳到 4.422，从此整条路线跑偏（xt_rms 0.107 → 0.424）。
        #   位移方向是"车真的往哪边走"的**直接测量**，过弯时也正确（它就是速度方向）。
        # ★ 只在真的走了 `_MH_MIN_STEP_M` 以上时才更新：停车/原地摆头时位移是数值噪声，
        #   保留上一次的方向估计（哨兵 0 表示"还没有可用估计"）。
        _mvx = bx - _bx_prev
        _mvy = by - _by_prev
        if math.hypot(_mvx, _mvy) >= _MH_MIN_STEP_M:
            motion_heading = math.atan2(_mvy, _mvx)
        _bx_prev, _by_prev = bx, by

        # ================== ② 到位吸附（FixTask 的 L2 位置修正） ==================
        # ★ 语义（与 `docs/P1接口约定.md` §3.1 的 FixTask 一致）：**车确实到了某个航点**时，
        #   外部观测（已知格心）把里程计估计拉回真值。
        # ★ 四条约束缺一不可（每条都是踩出来的）：
        #   ① 触发用**真值**（车真的到了那个点才锚）：用信念判等于"车还在半路就把它拽到
        #      下一个格心"，误差单方向累积 —— 实测零误差下信念↔真值 RMS 从 0.003 涨到 0.44 m；
        #   ② 每个航点**只锚一次**、且锚完立即推进（否则"锚→弧长不动→再锚"自锁）；
        #   ③ 只锚"还没走过"的点（`_advance_anchor`：否则会把信念向**后**拽）；
        #   ④ **拐角顶点不锚**（`_anchor_allowed`）：锚到顶点会让信念落在
        #      `t=1, dist_to_vertex=0` 的退化位置，前视点塌缩成投影点本身 → 零舵角直行。
        # ⚠️ 不动 `byaw`：航向是另一路估计（陀螺积分），把航向一起掰到段方向等于凭空给车
        #   一个"瞬间转 90°"的角速度（实测跑 60 s 就跑飞）。
        anchored_now = False
        if sim.anchor_on_arrival:
            anchor_idx = _advance_anchor(path, anchor_idx, s_prev)
            if anchor_idx < len(path.waypoints) and _anchor_allowed(path, anchor_idx):
                target_wp = path.waypoints[anchor_idx]
                if math.hypot(target_wp.x - tx, target_wp.y - ty) <= sim.anchor_radius_m:
                    # ★ 修正量 = 钉到**真值此刻的位置**（`bx,by = tx,ty`）。三个候选都实测过：
                    #   · 钉**格心** → 把"过弯没压到格心"的 0.44 m 偏差灌进估计（信念误差反而变大）；
                    #   · 沿路线**法向**归零 → 抹不掉纵向里程误差（ε 的主要表现），对 ε 几乎无用；
                    #   · 钉**真值位置**（本实现）→ 零注入时修正量恒为 0（**不变量成立**），
                    #     注入时把累积误差按"已知格心观测"抹平 —— 这正是 L2 修正的价值。
                    # ⚠️ **只在修正幅度不超过 `anchor_radius_m` 时才真的修**：一次跳 1~2 m 会把
                    #   信念投影甩到别的航段上（路线在 F11/B6 附近自交），控制器随即按错误的前视点
                    #   打舵 —— 实测"照单全收"版本在 ε=+0.18 下信念误差反而更大（1.15 m vs 0.23 m）。
                    #   观测的可用范围应当由观测本身（格心命中半径）界定。
                    if math.hypot(tx - bx, ty - by) <= sim.anchor_radius_m:
                        bx, by = tx, ty
                        anchors += 1
                        anchored_now = True
                        anchor_idx += 1

        # ================== ③ 任务推进（MissionTask：到点即换下一个航点） ==================
        # ★ 这是仿真能否跑完的**关键**，也是现实里真实存在的机制：循迹永远只朝**当前航段**追。
        #   两条推进规则缺一不可：
        #   ① 到位：真值进入当前航点的 `pursuit.arrival_m` 球内 → 推进；
        #   ② **已过点即放行**：投影弧长已越过该顶点（`s >= s_at_waypoint(target)`）→ 推进。
        #   ⚠️ 只有 ① 会死锁两处（都是实测）：
        #     · `max_steer_deg=30°` + `wheelbase=0.157 m` → 最小转弯半径 0.27 m，
        #       而现场路线一个 90° 弯只给 0.35 m 段长，车过弯必然从顶点外侧掠过
        #       （实测最近 0.24 m），永远进不了 0.12 m 到位球 → `target_idx` 卡死、无限绕圈；
        #     · 倒车段（`R:B6` → `R:D6`）要求 0.35 m 的最小转弯半径，而车的物理下限是
        #       `L/tan(δmax) = 0.157/tan30° = 0.272 m` —— 这一段只有 0.70 m 空间可用。
        #   现实里执行器就是"过了这个点就认账、往下走"（旧 OVERSHOOT 补触发同语义）。
        #   ⚠️ **"过了"的判据必须包含"车已经越过该顶点的垂面"**，只用投影弧长会死锁：
        #      `_clamp_forward` 把投影钉在顶点弧长上（`s == s_at_waypoint(i)`），
        #      而判据是 `s >= s_at_waypoint(i)` —— 差一个浮点 epsilon 就永远不成立。
        #      实测（现场图，A6/B6 折返）：车在外圈绕到天荒地老，`target_idx` 卡在 19，
        #      投影弧长死在 8.050 m 不动，822 s 也没跑完。
        #      物理上"越过垂面"= 车到目标点的距离 < 车到**当前段起点**的距离。
        if target_idx < len(path.waypoints):
            wp = path.waypoints[target_idx]
            is_goal = (target_idx == len(path.waypoints) - 1)
            passed = s_prev >= path.s_at_waypoint(target_idx) - _END_EPS_M
            if not passed:
                anchor_wp = path.waypoints[target_idx - 1]
                d_target = math.hypot(tx - wp.x, ty - wp.y)
                d_prev = math.hypot(tx - anchor_wp.x, ty - anchor_wp.y)
                # ⚠️ 末点的"已过点即放行"必须用同一个更紧半径：用 `arrival_m`(0.12) 判
                #    "越过了终点垂面"等于在 0.12 m 外就认账，`_GOAL_STOP_M` 永远轮不到生效
                #    （这正是 `final_error_m` 长期钉在 0.118 的原因）。
                passed = d_target < d_prev and d_target <= (_GOAL_STOP_M if is_goal else arrival)
            # ⚠️ 末点用更紧的到位半径：一旦按 `arrival_m`(0.12) 推进到末点，
            #    `s_floor` 立刻抬到 `total_length`，投影被夹到末点 → 控制器认为"到终点了"，
            #    车会停在离终点 0.12 m 处（实测 `final_error_m = 0.117`）。
            #    末点必须真的走到 `_GOAL_STOP_M` 内才算到 —— 与现场"终点要停车"一致。
            radius = _GOAL_STOP_M if is_goal else arrival
            # ★★ **朝向门**：进到位球还不够，车头必须已经**朝着下一段的走向**。
            #   为什么（端到端实测出来的最后一处死锁）：现场图 `F11→E11`（朝北）之后紧接
            #   `R:F11`（朝南，倒车）—— 车在 E11 的 `arrival_m` 球内、**还朝北**时就推进目标，
            #   于是 `s_floor` 抬到 E11、投影被拉到 `E11→F11` 段上且**朝向翻 180°**：
            #     · `dist_to_vertex = 0` → `find_lookahead_point` 把前视点钳成顶点本身
            #       （前视点塌缩 = 方位角退化，见 `_anchor_allowed` 的注释）；
            #     · 控制器按"已到 E11、该往南走"算，可车还在 E11 北侧 0.17 m 处 ——
            #       指令与实际位置不自洽，车被推离路线且再也回不来
            #       （实测：`s` 在 4.200 上钉死 1100 拍、`xt_rms` 0.107 → 0.249）。
            #   加门之后，车会先按"还差最后一段"把最后 0.12 m 走完 / 原地把车头转到
            #   下一段方向，投影与指令重新自洽，折返才能完成。
            #   ⚠️ 容差 90°：现场图 90° 拐角（`L10` / `F10` / `A6` / `D6` …）必须照常放行，
            #      只挡"掉头"这一种（下一段与当前段反向 = 差 180°）。
            #   ⚠️ `passed`（已过垂面）不受门限制：那是"车已经走过去了"的既成事实，
            #      再等朝向只会让它原地打转。
            facing = True
            if target_idx < path.segment_count:
                out_heading = path.segments[target_idx][5]
                facing = abs(geo.normalize_angle(out_heading - tyaw)) <= math.radians(
                    _ADVANCE_FACING_TOL_DEG)
            if math.hypot(tx - wp.x, ty - wp.y) <= radius and facing:
                target_idx += 1
            elif passed:
                target_idx += 1

        # ================== ④ 物理投影（"Localization 报的 P_ROUTE_S"） ==================
        # ⚠️ 投影弧长**不允许退回上一段**：纯追踪一定会把车带过顶点一点，一旦越过顶点，
        #   `Path.project` 窗口内的"最近点"可能仍是那个顶点 → 弧长**再也不前进**
        #   → 前视点塌缩到顶点（车已在它上面）→ 方位角退化 → 舵角恒 0 → 车直着开走。
        s_floor = path.s_at_waypoint(target_idx - 1) if target_idx >= 1 else 0.0
        proj_t = _project_sticky(path, tx, ty, s_hint=s_prev, window_m=_PROJ_FWD_M,
                                 allow_backward=True, motion_heading=motion_heading,
                                 prev_seg=prev_truth_seg, s_min=s_floor)
        proj_t = _clamp_forward(path, proj_t, s_floor, tx, ty, arrival)
        s_prev = proj_t.s
        prev_truth_seg = proj_t.seg_index

        # 终点判据（**用真值，不用信念**）：吸附会把信念瞬移到格心，
        # 用信念判会提前 `arrival_m` 就刹车，`final_error_m` 平白多 0.12 m。
        goal = path.waypoints[-1]
        if math.hypot(tx - goal.x, ty - goal.y) <= _GOAL_STOP_M:
            reached_end = True
        if proj_t.s >= path.total_length - _END_EPS_M and not reached_end:
            reached_end = True      # 投影已到末点（车开过终点的情况）

        # ================== ⑤ 信念投影 → 纯追踪算一拍指令 ==================
        # 信念投影：**以物理弧长为锚 ± `_PROJ_BELIEF_M`**，再夹到当前航段之后。
        # ⚠️ 不能给它一个很大的自由窗口：车在拐角处时"最近点"可能落在**别的航段**上
        #   （现场路线在 B6/F11 附近自交），于是信念投影跳到前面 1.2 m 处，
        #   控制器拿着"我还在前面"的假信息反向打舵 —— 实测误差注入下就在这里开始绕圈。
        proj = _project_sticky(path, bx, by, s_hint=proj_t.s, window_m=_PROJ_BELIEF_M,
                               allow_backward=True, motion_heading=motion_heading,
                               prev_seg=prev_belief_seg, s_min=s_floor)
        proj = _clamp_forward(path, proj, s_floor, bx, by, arrival)
        prev_belief_seg = proj.seg_index

        if reached_end and sim.stop_on_completion:
            cmd = brake_command(cfg)
        else:
            cmd = compute_command(geo.Pose(bx, by, byaw), path, proj, cfg,
                                  last_steer_deg=(last_cmd_steer if sim.command_rate_limit
                                                  else None),
                                  dt=dt)
        # ---- 指令侧速率：相邻两拍**控制器输出**之差（这才是"速率限幅"的观测量）----
        # ⚠️ 老实现拿"当前指令 − 上一拍采样到的执行值"当指令速率，末端/对齐点上会算出
        #    680.9 °/s 的**假峰值**（真实指令从没超过 120 °/s，见测试 test_sim 第 8 组）。
        if dt > 0.0 and last_cmd_steer is not None:
            cmd_rate = abs(cmd.steer_deg - last_cmd_steer) / dt
            steer_cmd_rates.append(cmd_rate)
            if cmd_rate > cmd_rate_peak:
                cmd_rate_peak = cmd_rate
        last_cmd_steer = cmd.steer_deg

        # ================== ⑥ 指令延迟队列 + 舵机执行模型 ==================
        # ★ 队列空时**保持上一条已生效指令**（`_take` 的 hold 语义）。
        #   ⚠️ 老实现在队列空时返回 (0, 0) —— 而 dt < cmd_latency_s 时队列**每拍都是空的**，
        #      于是"舵角每拍在 ±30° 与 0 之间跳"，执行侧速率 1500 °/s，
        #      信念还在拿这个跳变量积分 ⇒ 两条轨迹都不是车的真实运动。
        _schedule(queue, t + sim.cmd_latency_s, cmd.linear_x, cmd.steer_deg)
        v_exec, steer_target = _take(queue, t, v_exec, steer_target)
        # ⚠️ `brake_now` 必须看**延迟后生效的指令**（`v_exec`/`steer_target`），
        #   不能看当拍刚算出的 `cmd` —— 刹车判定提前 3 拍会让舵机在车还在动时就冻住。
        brake_now = (abs(v_exec) < 1e-12 and abs(steer_target) < 1e-12)
        # 舵机斜坡：执行舵角是**舵机实际位置**，按 `max_steer_rate_deg_s` 跟随指令。
        # ⚠️ 刹车（v=0 且指令舵角 0）时舵机保持原位、不掉头 —— 现实中停车不会自动回正。
        # ★ 本拍算出的 `(v_exec, steer_exec)` 在**下一拍开头**同时作用于真值与信念。
        if dt > 0.0 and sim.actuator_rate_limit:
            steer_exec = (steer_exec if brake_hold
                          else _ramp_towards(steer_exec, steer_target, act_step))
        else:
            steer_exec = steer_target
        brake_hold = brake_now

        # ================== ⑦ 采样与分组统计 ==================
        # 分组（normal/reverse/corner）与逐段峰值都基于**真值**（物理位置），
        # 不基于信念 —— 否则误差注入下"车在哪一组"会跟着估计漂移。
        # "拐角组" = 真值投影落在某个顶点 ±`_CORNER_ZONE_M` 内（顶点在本段末端或前端都算）。
        # ★ 站点的横向误差几乎全部产生在顶点附近（进弯切内、出弯甩外），
        #   把它单独拎出来才能回答"这 0.2 m 是直段蹭的还是拐角吃的"。
        d_front = proj_t.s - path.s_at_waypoint(proj_t.seg_index)
        near_vertex = min(proj_t.dist_to_vertex, max(0.0, d_front)) <= _CORNER_ZONE_M
        is_rev = path.segment_is_reverse(proj_t.seg_index)
        kind = ("reverse" if is_rev
                else "corner" if near_vertex else "normal")
        xt_truth = _cross_track_m(tx, ty, path.segments)
        truth_xt.append(xt_truth)
        belief_xt.append(_cross_track_m(bx, by, path.segments))
        belief_gap.append(math.hypot(bx - tx, by - ty))
        in_bounds_belief.append(1.0 if geo.in_bounds(bx, by, margin) else 0.0)
        in_bounds_truth.append(1.0 if geo.in_bounds(tx, ty, margin) else 0.0)
        xt_by_kind[kind].append(xt_truth)
        steps_by_kind[kind] += 1
        time_by_kind[kind] += dt
        # 逐段峰值（用"点在线段上的投影距离"，与 `_cross_track_m` 同一距离定义）
        if 0 <= proj_t.seg_index < len(seg_peak):
            slot = seg_peak[proj_t.seg_index]
            if xt_truth > slot["peak"]:
                slot["peak"] = xt_truth
                slot["t"] = t
                slot["s"] = proj_t.s

        # ================== ⑦' ★ 场地物理观测（t24） ==================
        # ★ 用**真值**位姿（物理世界）：地图上的事实不能跟着里程计漂移走。
        # ⚠️ 本块**只读** `(tx, ty, tyaw)` 并累加计数 —— 对轨迹零影响（t24 有硬断言）。
        field_violation = False
        if obs is not None:
            field_violation = obs.observe(t, tx, ty, tyaw, kind=kind)

        # 执行侧速率：相邻两拍**舵机位置**之差；倒车符号翻转单独归类
        if dt > 0.0:
            d_steer = abs(steer_exec - (_prev_steer(result.steer)))
            steer_delta_raw.append(d_steer)
            if d_steer > act_step * (1.0 + 1e-9):
                delivered_jump_steps += 1
            sign_flip = (bool(result.reverse) and result.reverse[-1] != is_rev
                         and abs(steer_target - steer_exec) > act_step)
            if sign_flip:
                flip_steps += 1
                if cmd_rate_peak > flip_rate_peak:
                    flip_rate_peak = cmd_rate_peak
            if d_steer / dt > delivered_rate_peak:
                delivered_rate_peak = d_steer / dt

        if steps % trace_every == 0:
            result.t.append(t)
            result.x.append(bx)
            result.y.append(by)
            result.yaw.append(byaw)
            result.v.append(v_exec)
            result.steer.append(steer_exec)
            result.truth_x.append(tx)
            result.truth_y.append(ty)
            result.truth_yaw.append(tyaw)
            result.seg_index.append(proj.seg_index)
            result.s.append(proj.s)
            result.truth_seg_index.append(proj_t.seg_index)
            result.truth_s.append(proj_t.s)
            result.steer_cmd.append(cmd.steer_deg)
            result.anchored.append(anchored_now)
            result.near_vertex.append(near_vertex)
            result.reverse.append(is_rev)

        steps += 1
        t += dt

        if sim.trace_hook is not None:      # 排障钩子：只在显式传入时生效，正常跑无开销
            sim.trace_hook(steps, {     # type: ignore[operator]
                "t": t, "bx": bx, "by": by, "byaw": byaw, "tx": tx, "ty": ty,
                "tyaw": tyaw, "v": v_exec, "steer": steer_exec,
                "steer_cmd": cmd.steer_deg, "steer_target": steer_target,
                "s": proj.s, "seg": proj.seg_index, "dist": proj.dist,
                "s_truth": proj_t.s, "seg_truth": proj_t.seg_index, "kind": kind,
                "anchors": anchors, "anchored": anchored_now,
                "target_idx": target_idx, "s_floor": s_floor,
                "goal_d": math.hypot(tx - goal.x, ty - goal.y),
                "trk": math.hypot(tx - tx0, ty - ty0),
            })

        # ---------- 7. 完成 / 跑飞判据 ----------
        # ⚠️ `reached_end` 在真值推进**之后**才可能为真（终点判据看的是真值位置），
        #   所以这里先刹车（上一拍的 `reached_end` 已让控制器低速滑行）再退出，
        #   保证末拍的真值就停在终点上（`final_error_m` 才有意义）。
        if sim.stop_on_completion and reached_end:
            break
        # ★ t24 对照组：撞上场地要素（障碍/挡板/靶板/K 列封死）即结束。
        #   ⚠️ 默认关闭：本仿真测的是"循迹能力"，加了硬停就不再与历史基线可比。
        if sim.stop_on_field_violation and field_violation:
            field_stop_t = t
            break

        # ---- 跑飞看门狗：投影弧长连续 `_DIVERGE_STEPS` 拍不前进且已远离路线 → 收工 ----
        # 没有它，参数病态时仿真会白跑满步数（41000 拍里 76 秒全在绕圈），
        # 而测试只看到"未完成"，看不出是"稳不住"还是"走错路"。
        if abs(proj_t.s - last_s) < 1e-4:
            stall += 1
        else:
            stall = 0
            last_s = proj_t.s
        if stall >= _DIVERGE_STEPS and proj_t.dist > 0.90:
            diverged = True
            diverged_at = steps
            break

    # ---- 收尾采样：把最终状态记一条（轨迹终点可见，不需要脑补） ----
    _record_final(result, t, bx, by, byaw, v_exec, steer_exec, tx, ty, tyaw,
                  proj_t.s if truth_xt else 0.0)

    # ---- 指标 ----
    final = path.waypoints[-1]
    # "顶点冲过头"：真值轨迹在某个顶点处**离格心最近**时的超出量（超过到位半径的部分）。
    # 严格沿路线走的车该值是 0；信念领先（ε>0）或转弯半径不够时车会在拐角外侧掠过顶点 → 出现正值。
    # ★ 按段类型分开统计：老实现把 34 个点混在一起取 max，于是 0.5552 m 里
    #   "倒车折返 R:F11 在原地掉头"和"90° 弯切角"分不开 —— 前者不用修，后者要调参。
    vertex_off = {"normal": 0.0, "reverse": 0.0}
    overshoot = 0.0
    for idx, wp in enumerate(path.waypoints):
        # 该顶点"属于"哪一类：看它前一等段是否倒车（首点看后一段）
        seg_for_kind = idx - 1 if idx > 0 else 0
        kind = "reverse" if path.segment_is_reverse(seg_for_kind) else "normal"
        d_min = min(math.hypot(tx2 - wp.x, ty2 - wp.y)
                    for tx2, ty2 in zip(result.truth_x, result.truth_y))
        excess = d_min - arrival
        if excess > vertex_off[kind]:
            vertex_off[kind] = excess
        if excess > overshoot:
            overshoot = excess

    walked = 0.0
    for i in range(1, len(result.truth_x)):
        walked += math.hypot(result.truth_x[i] - result.truth_x[i - 1],
                             result.truth_y[i] - result.truth_y[i - 1])
    final_truth_err = math.hypot(result.truth_x[-1] - final.x, result.truth_y[-1] - final.y)
    final_belief_err = math.hypot(result.x[-1] - final.x, result.y[-1] - final.y)

    # ---- 逐段峰值表（定位"是哪几个拐角在吃误差"）----
    stats: list[SegmentStat] = []
    for i in range(path.segment_count):
        a = path.waypoints[i]
        b = path.waypoints[i + 1]
        slot = seg_peak[i]
        peak = slot["peak"] if slot["peak"] >= 0.0 else 0.0
        stats.append(SegmentStat(
            index=i, from_block=a.block, to_block=b.block,
            length_m=path.segments[i][4],
            kind="reverse" if path.segment_is_reverse(i) else "normal",
            peak_m=peak, peak_t=slot["t"], peak_s=slot["s"],
            corner_deg=path.corner_angle_deg(i),
        ))
    result.segments = stats

    # ---- 信念↔真值：扣除"锚定拍"后的残差（锚定按定义就是把信念拽到真值上）----
    # ★ 零注入时这个残差必须 ≈ 0 —— 它是"模型自洽"的可测定义（测试硬断言：
    #   零误差跑的每一拍信念与真值逐位相同，只有 anchored=True 的拍例外）。
    belief_gap_free = [math.hypot(result.x[i] - result.truth_x[i],
                                  result.y[i] - result.truth_y[i])
                       for i in range(len(result.t)) if not result.anchored[i]]

    metrics.update({
        # ---- 契约键（docs/P0接口约定.md §5，键名固定，勿改）----
        "completed": bool(reached_end),
        "time_s": float(result.t[-1]) if result.t else 0.0,
        "cross_track_rms_m": _rms(truth_xt),
        "cross_track_max_m": max(truth_xt) if truth_xt else 0.0,
        "final_error_m": final_truth_err,
        "overshoot_max_m": max(0.0, overshoot),
        "max_steer_deg": max((abs(v) for v in result.steer), default=0.0),
        "max_steer_rate_deg_s": cmd_rate_peak,
        "max_steer_rate_delivered_deg_s": delivered_rate_peak,
        "path_len_ratio": (walked / path.total_length) if path.total_length > 0 else 0.0,
        "anchors": float(anchors),
        "steps": float(steps),
        # ---- 附加键（本仿真器为"看得见误差"而加，只增不改）----
        "belief_cross_track_rms_m": _rms(belief_xt),
        "belief_cross_track_max_m": max(belief_xt) if belief_xt else 0.0,
        "belief_error_rms_m": _rms(belief_gap),
        "belief_error_max_m": max(belief_gap) if belief_gap else 0.0,
        "belief_error_max_free_m": max(belief_gap_free) if belief_gap_free else 0.0,
        "belief_final_error_m": final_belief_err,
        "out_of_bounds_steps": float(sum(1 for v in in_bounds_truth if v < 0.5)),
        "belief_out_of_bounds_steps": float(sum(1 for v in in_bounds_belief if v < 0.5)),
        "bounds_margin_m": margin,
        "path_length_m": path.total_length,
        "route_points": float(len(path.waypoints)),
        "lookahead_m": ld,
        "dt_s": dt,
        "cmd_latency_s": sim.cmd_latency_s,
        "speed_scale_err": sim.speed_scale_err,
        "yaw_drift_deg_per_min": sim.yaw_drift_deg_per_min,
        "pos_noise_m": sim.pos_noise_m,
        "anchor_on_arrival": bool(sim.anchor_on_arrival),
        "anchor_radius_m": sim.anchor_radius_m,
        "hit_step_limit": bool(steps >= max_steps and not reached_end),
        "max_steps": float(max_steps),
        "diverged": bool(diverged),
        "diverged_at_step": float(diverged_at),
        # ---- ★ 分组指标（调参的唯一依据；键名只增不改）----
        "cross_track_rms_normal_m": _rms(xt_by_kind["normal"]),
        "cross_track_rms_reverse_m": _rms(xt_by_kind["reverse"]),
        "cross_track_rms_corner_m": _rms(xt_by_kind["corner"]),
        "cross_track_max_normal_m": max(xt_by_kind["normal"]) if xt_by_kind["normal"] else 0.0,
        "cross_track_max_reverse_m": max(xt_by_kind["reverse"]) if xt_by_kind["reverse"] else 0.0,
        "cross_track_max_corner_m": max(xt_by_kind["corner"]) if xt_by_kind["corner"] else 0.0,
        "vertex_overshoot_normal_m": max(0.0, vertex_off["normal"]),
        "vertex_overshoot_reverse_m": max(0.0, vertex_off["reverse"]),
        "steer_rate_p95_deg_s": _percentile(steer_cmd_rates, 0.95),
        "steer_rate_max_deg_s": max(steer_cmd_rates) if steer_cmd_rates else 0.0,
        "steer_delivered_rate_p95_deg_s": _percentile(
            [d / dt for d in steer_delta_raw] if dt > 0 else [], 0.95),
        "steer_sign_flip_steps": float(flip_steps),
        "steer_rate_at_flip_deg_s": flip_rate_peak,
        "steer_delivered_jump_steps": float(delivered_jump_steps),
        "steer_rate_limit_deg_s": rate_limit,
        "time_s_normal": time_by_kind["normal"],
        "time_s_reverse": time_by_kind["reverse"],
        "time_s_corner": time_by_kind["corner"],
        "steps_normal": float(steps_by_kind["normal"]),
        "steps_reverse": float(steps_by_kind["reverse"]),
        "steps_corner": float(steps_by_kind["corner"]),
        "belief_trivial_steps": float(sum(1 for f in result.anchored if f)),
        "reverse_segment_count": float(sum(1 for i in range(path.segment_count)
                                          if path.segment_is_reverse(i))),
    })
    # ---- ★ 场地物理指标（t24，只增不改）----
    # ⚠️ 未建模场地时 `field_modeled = 0.0`，`format_metrics` 据此**整块不打印** ——
    #    这样"没接地图"与"接了地图但零违规"在控制台上不会长得一样（不说谎）。
    if obs is not None:
        metrics.update(obs.metrics())
        result.field_report = {
            "spec": obs.spec.describe(),
            "source": obs.spec.source,
            "cells": {"mines": list(obs.spec.mines), "obstacles": list(obs.spec.obstacles),
                      "targets": list(obs.spec.targets), "bridges": list(obs.spec.bridges)},
            "press_steps": dict(obs.press_steps),
            "mine_best": dict(obs.mine_best),
            "first": dict(obs.first),
            "board_min": obs.board_min,
            "obs_min_center": obs.obs_min_center,
            # ★ 画图参数（渲染按物理形状画，尺寸必须与判据同源；`write_svg` 从这里取）
            "shape": {
                "mine_pad_m": obs.mine_pad_m,
                "obstacle_radius_m": obs.obstacle_radius_m,
                "barrier_width_m": obs.barrier_width_m,
                "board_half_m": obs.board_half,
                "barrier": (obs.bar_x, obs.bar_y0, obs.bar_y1),
                "barrier_cols": obs.barrier_cols,
                "k_col": obs.k_col,
                "open_rows": tuple(obs.open_rows),
                "rows": geo.MAP_ROWS,
                "cols": geo.MAP_COLS,
                "block_m": geo.BLOCK_SIZE,
            },
        }
    else:
        metrics["field_modeled"] = 0.0
    metrics["field_stopped"] = 1.0 if field_stop_t is not None else 0.0
    metrics["field_stop_t"] = float(field_stop_t) if field_stop_t is not None else -1.0
    # ⚠️ `completed` / `anchor_on_arrival` 是 bool，其余指标是 float —— 混类型是刻意的：
    #    CSV/SVG/控制台都不依赖 metrics 的类型统一，而测试要读布尔真值。
    return result


def _prev_steer(history: list[float]) -> float:
    return history[-1] if history else 0.0


def _record_final(result: SimResult, t: float, bx: float, by: float, byaw: float,
                  v_eff: float, steer_eff: float, tx: float, ty: float, tyaw: float,
                  s: float) -> None:
    """把"最后一拍之后"的状态补记一条，让轨迹终点真的落在停车位置。

    ⚠️ 只补位置列（`t/x/y/…`），诊断列用**上一拍的值**兜底 —— 少一列会让
      CSV 与分组统计错位，是比"终点少一条采样"严重得多的 bug。
    """
    if result.t and result.t[-1] >= t:
        return
    result.t.append(t)
    result.x.append(bx)
    result.y.append(by)
    result.yaw.append(byaw)
    result.v.append(v_eff)
    result.steer.append(steer_eff)
    result.truth_x.append(tx)
    result.truth_y.append(ty)
    result.truth_yaw.append(tyaw)
    result.seg_index.append(result.seg_index[-1] if result.seg_index else 0)
    result.s.append(s)
    result.truth_seg_index.append(result.truth_seg_index[-1] if result.truth_seg_index else 0)
    result.truth_s.append(result.truth_s[-1] if result.truth_s else s)
    result.steer_cmd.append(result.steer_cmd[-1] if result.steer_cmd else steer_eff)
    result.anchored.append(False)
    result.near_vertex.append(result.near_vertex[-1] if result.near_vertex else False)
    result.reverse.append(result.reverse[-1] if result.reverse else False)


def _project_with_seg(path: Path, seg: int, x: float, y: float):
    """把点投影到**指定段**（返回 `Projection`；段退化时退化为端点距）。"""
    x0, y0, x1, y1, length, heading = path.segments[seg]
    if length <= 1e-9:
        return _projection_at(path, seg, path.s_at_waypoint(seg), x, y)
    ux = (x1 - x0) / length
    uy = (y1 - y0) / length
    t = ((x - x0) * ux + (y - y0) * uy) / length
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    s = path.s_at_waypoint(seg) + length * t
    return _projection_at(path, seg, s, x, y)


def _candidates(path: Path, x: float, y: float, i_lo: int, i_hi: int):
    """窗口 `[i_lo, i_hi]` 内每段的投影候选（按段序返回）。"""
    out = []
    for i in range(max(0, i_lo), min(i_hi, path.segment_count - 1) + 1):
        out.append(_project_with_seg(path, i, x, y))
    return out


def _pick_candidate(cands, *, x: float, y: float, hint: float | None,
                    motion_heading: float | None,
                    allow_beyond: bool):
    """从候选里挑一个：**先比距离**，等距时依次用 行进方向 → 连续性 → 段序 裁决。

    ★ 为什么不能直接相信搜索顺序（这是端到端复现出来的**真正死锁**）：
      `Path.project` 在窗口内按"先用 `<` 严格更近"挑点，遇到"投影被夹到段端点"
      的候选（车已经越过顶点）时，先被考虑到的**前一段**会赢 —— 于是车头已经伸过
      顶点，投影却仍然停在顶点上：`s` 不再前进、`dist_to_vertex = 0`、
      前视点被转角钳制塌缩到顶点本身，控制器算出 0 舵角直着开走；
      车在 `A6` 拐角外绕 0.4 m 的圈，`s` 死钉在 8.050 m、`target_idx` 死在 20，
      8000 拍跑不完（`xt_rms` 0.107 → 0.266）。
      正确的判据是"哪个候选离车**最近**"（被夹住的端点在车越过顶点后会**更远**），
      而不是"哪个候选先被看到"。
    ★ `allow_beyond=False` 时（任务仍在当前段内）不许选任务段之后的候选：
      那是"车还没走到就先看见下一段"，会让前视点跳到还没到的路上去。
    """
    best = None
    best_d = None
    best_h = None
    for c in cands:
        if not allow_beyond and best is not None and c.seg_index > best.seg_index:
            break
        if best is None:
            take = True
        elif c.dist < best_d - 1e-9:
            take = True
        elif c.dist > best_d + 1e-9:
            take = False
        else:
            take = False
            if motion_heading is not None:
                cos_new = math.cos(c.seg_heading - motion_heading)
                cos_best = math.cos(best_h - motion_heading)
                if cos_new > cos_best + 1e-12:
                    take = True
                elif abs(cos_new - cos_best) <= 1e-12 and hint is not None:
                    take = abs(c.s - hint) < abs(best.s - hint) - 1e-12
            elif hint is not None:
                take = abs(c.s - hint) < abs(best.s - hint) - 1e-12
            else:
                take = c.seg_index > best.seg_index
        if take:
            best, best_d, best_h = c, c.dist, c.seg_heading
    return best


def _project_sticky(path: Path, x: float, y: float, *, s_hint: float | None,
                    window_m: float, allow_backward: bool,
                    motion_heading: float | None, prev_seg: int, s_min: float = 0.0):
    """带**任务下界 + 段选择滞回**的投影（`Path.project` 的加强版）。

    ★ 为什么需要（这是本仿真器跑不完现场图的**核心**，两条独立线索都指向它）：
      · 现场图在 `B6` 一点上有**四条段交汇**（`B8→B6` 与 `B6→D6` 共线、
        `B6→A6` 与 `A6→B6` 折返），车停在交点附近时多个候选**等距**；
      · `Path.project` 用 `s_hint = 上一拍真值弧长` 做连续性裁决，而真值弧长
        本身在"抬到 s_floor"与"落回真实最近点"之间摆动 → 投影段逐拍翻转
        → 指令在前进/倒车间翻转 → 车在交点原地摆动（实测 2000 拍，
        `s` 在 7.58 ↔ 8.40 之间跳、`target_idx` 死在 20）。
    ★ 三层修法（互补，缺一条都跑不完）：
      ① **`s_min` = 当前任务段的起点弧长**：投影不能落到任务段**之前**的段上。
         现场实测：`target_idx` 已推进到第 20 段（`R:D6`）时车在 `B6` 通道里，
         而"上一拍那段"是第 16 段 —— 它离车更近，滞回于是把投影**钉在已经走过的
         段上**，车在 `B6` 附近摆 4000 拍也进不了 D6 通道。任务给段、定位在段内给
         弧长，这与现实的 Mission/Localization 分工一致。
      ② **窗口内比距离（而不是比搜索顺序）**：见 `_pick_candidate`。
      ③ **上一拍那段是强先验**：别的候选至少要近 `_SEG_STICKY_M` 才允许换段，
         换段后立即生效（滞回是**不对称**的，不会把车锁死在旧段上）。
    ★ `motion_heading` 仍然要传：它解决的是"同一拍内两个候选等距"时该选哪个
      （折返压线的判据只能是"车正在往哪边走"）。
    """
    if s_hint is not None and s_hint < s_min:
        s_hint = s_min
    floor_seg = _seg_index_of_s(path, s_min) if s_min > 0.0 else 0
    proj = path.project(x, y, s_hint=s_hint, window_m=window_m,
                        allow_backward=allow_backward, motion_heading=motion_heading)
    i_lo, i_hi = _window_segments(path, s_hint, window_m, allow_backward)
    i_lo = max(i_lo, floor_seg)
    if i_hi >= i_lo:
        cands = _candidates(path, x, y, i_lo, i_hi)
        # 任务还没走完当前段时不许"先看见"后面的段（车还没到那儿）
        allow_beyond = (prev_seg < 0) or (_seg_index_of_s(path, s_min) <= prev_seg
                                          if s_min > 0.0 else True)
        pick = _pick_candidate(cands, x=x, y=y, hint=s_hint,
                               motion_heading=motion_heading,
                               allow_beyond=allow_beyond)
        if pick is not None:
            proj = pick
    if s_min > 0.0 and proj.s < s_min and proj.seg_index < floor_seg:
        proj = _project_with_seg(path, floor_seg, x, y)
    if prev_seg < 0 or prev_seg >= path.segment_count or prev_seg == proj.seg_index:
        return proj
    if prev_seg < floor_seg:
        return proj                 # 先验本身落在任务段之前 → 不可用
    keep = _project_with_seg(path, prev_seg, x, y)
    if keep.dist <= proj.dist - _SEG_STICKY_M:
        return keep
    return proj


def _window_segments(path: Path, s_hint: float | None, window_m: float,
                     allow_backward: bool) -> tuple[int, int]:
    """与 `Path.project` 同口径的搜索窗口（段区间），供候选枚举使用。"""
    n = path.segment_count
    if s_hint is None:
        return 0, n - 1
    hint = s_hint if 0.0 <= s_hint <= path.total_length else (
        0.0 if s_hint < 0.0 else path.total_length)
    lo = hint - window_m if allow_backward else hint
    lo = 0.0 if lo < 0.0 else lo
    hi = hint + window_m
    hi = path.total_length if hi > path.total_length else hi
    i_first = _seg_index_of_s(path, lo)
    i_last = bisect.bisect_right(path._s_start, hi) - 1      # noqa: SLF001 - 只读查询
    if i_last < i_first:
        i_last = i_first
    if i_last > n - 1:
        i_last = n - 1
    return i_first, i_last


def _seg_index_of_s(path: Path, s: float) -> int:
    """弧长 `s` 落在哪一段（边界归**后**一段，与 `Path._seg_index_at` 同义）。"""
    i = bisect.bisect_right(path._s_start, s) - 1      # noqa: SLF001 - 同包内只读查询
    if i < 0:
        return 0
    if i > path.segment_count - 1:
        return path.segment_count - 1
    return i


def _projection_at(path: Path, seg: int, s: float, x: float, y: float):
    """构造"段 `seg` 内、弧长 `s` 处"的投影（`dist` 如实取到车的位置）。

    ★ 与 `Path.project` 返回的 `Projection` 同构：`t` 由弧长反解，`seg_heading` 取该段走向。
      只用于 `_clamp_forward` 的钳制分支 —— 那里需要"把投影放到某个确定的弧长上"，
      而 `Path.point_at()` 只给坐标、不给完整的投影记录。
    """
    x0, y0, x1, y1, length, heading = path.segments[seg]
    if length <= 1e-9:
        t = 0.0
        px, py = x0, y0
    else:
        t = (s - path.s_at_waypoint(seg)) / length
        t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
        px = x0 + (x1 - x0) * t
        py = y0 + (y1 - y0) * t
    return Projection(seg_index=seg, t=t, x=px, y=py, s=s,
                      dist=math.hypot(x - px, y - py), seg_length=length,
                      seg_heading=heading, passed_end=False)


def _clamp_forward(path: Path, proj, s_floor: float, x: float, y: float,
                   release_dist: float):
    """投影弧长单调不退回：把 `proj` 抬到 `s_floor`（= 当前航段起点的弧长）。

    ★ 为什么需要：纯追踪过弯时必然越过顶点一点，而 `Path.project` 在窗口内的"最近点"
      可能仍是那个顶点 —— 于是弧长卡住、前视点塌缩、舵角归零、车直走。
      现实里 Mission 已经推进到下一段了，所以这里按 Mission 的语义把投影钉在段内。

    ⚠️ **但"钉住"必须能解开**（老实现没有这个出口，实测两处死锁、无法完成路线）：
      当车已经很接近当前航段起点时，钉住是对的（它确实还在顶点附近）；
      当车**在段边界外绕圈/倒车**（到被钉住那点的距离 > `release_dist`）时，
      "车还在段起点"这个假设已经错了 —— 此时必须让投影落回**真实最近点**
      （哪怕它退到上一段），否则：
        · 倒车段（`R:D6`）的投影永远不动 → 控制器以为车没动 → 一直倒；
        · Mission 的"已过点即放行"判据永远不成立 → `target_idx` 卡死 → 822 s 跑不完。
    ★★ **真正的死锁机制（端到端复现出来的那一处，比"绕圈"更隐蔽）**：
      `s_floor` 来自 **Mission 的目标**，而 Mission 会在"车进入顶点 `arrival_m` 球内"
      时**提前**推进 —— 此时车**还差一点没到顶点**，`s_floor` 却已经跳到下一段的起点：
        · 车在 `F11→E11` 段上朝北走、离 E11 还有 0.15 m（横向 0.17 m，> `arrival_m`）
          → 目标推进到 `R:F11` → `s_floor = 4.200`（E11 = 4.200）
        · 原始投影 `s = 4.2000`（顶死在顶点）、`dist = 0.17 > arrival_m`
        · 两个"释放"判据都要求 `dist <= arrival_m` → **都不成立** → 投影被钉在 **E11 顶点
          朝南** → 控制器认为"我已到 F11、该倒车了"，发 `v<0` + 满舵
        · 车被推离路线（实测 0.38 m），`xt_rms` 从 0.107 涨到 0.424，路线再也跑不完
      ⇒ 只要"车其实还在**上一段**上"（`proj.seg_index < s_floor` 所在段），钉住就必然是假的：
        上一段**还没走完**，投影不该跨到下一段去。此时取"原始投影"与"本段起点弧长"的
        **较大者** —— 既不许它退回本段起点之前（保留单调不后退的原有好处），
        也不许它越到下一段去（那是 Mission 提前推进造成的幻觉）。
      ⚠️ 阈值仍用 `release_dist`（= `pursuit.arrival_m`）：它就是现场"算不算到位"的判据。
    ★ 两条候选取**较大者**：既保留"顶点附近不后退"的原有好处，
      又不阻止真实的后退（倒车时 `proj.s` 自然会小于 `s_floor`）。
    """
    if proj.s >= s_floor:
        return proj
    floor_seg = path._seg_index_at(s_floor)          # `s_floor` 落在哪一段（下一段）
    if proj.seg_index < floor_seg:
        # 车还在**上一段**（Mission 提前推进造成的假钉住）：只在上一段内钳制，不跨段
        s_min = path.s_at_waypoint(proj.seg_index)
        if proj.s >= s_min:
            return proj
        return _projection_at(path, proj.seg_index, s_min, x, y)
    # 原始投影已经离路线很远 → "车还在段起点"解释不了它，用真实最近点（可后退）
    if proj.dist > release_dist:
        return proj
    x2, y2, seg = path.point_at(s_floor + _END_EPS_M)
    if math.hypot(x - x2, y - y2) > release_dist:
        return proj                 # 车已离开段起点 → 用真实投影（可后退）
    seg_len = path.segments[seg][4]
    t = 0.0 if seg_len <= 1e-9 else min(1.0, (_END_EPS_M) / seg_len)
    dist = math.hypot(x - x2, y - y2)
    return Projection(seg_index=seg, t=t, x=x2, y=y2, s=s_floor, dist=dist,
                      seg_length=seg_len, seg_heading=path.segments[seg][5],
                      passed_end=proj.passed_end)


def _segment_hint(path: Path, target_idx: int) -> float:
    """当前航段中点（`Path.project` 的 `s_hint`）。保留给无状态调用方使用。"""
    n = len(path.waypoints)
    if target_idx <= 0:
        return 0.0
    if target_idx >= n:
        return path.total_length
    return 0.5 * (path.s_at_waypoint(target_idx - 1) + path.s_at_waypoint(target_idx))


def _anchor_allowed(path: Path, idx: int) -> bool:
    """该航点是否允许被吸附？**拐角顶点不允许**。

    ★ 原因（实测，不是猜测）：把 belief 锚到拐角顶点后，`Path.project(belief)` 会返回
      `t = 1.0, dist_to_vertex = 0.0`；而 `find_lookahead_point` 的转角钳制分支
      （"夹角 > 阈值 且 距顶点 < vertex_clamp_m"）于是把前视点钳成**投影点本身**，
      相对方位角退化成 0/180 的不确定值 —— 纯追踪算出 0 舵角，车直着开走，
      此后再也不会转回来（现场路线实测：第一个拐角 L10 之后直接脱离路线）。

    ⚠️ 这是**仿真器对现实的一处刻意收紧**：现实中 Localization 的到位吸附发生在
      "车确实停在航点上"的时刻（那时纯追踪已进入下一点），而本仿真器每 20 ms 都在跑，
      把 belief 拍在顶点上的瞬时状态放大成了持续状态。真正的根治在
      `pursuit/lookahead.py`（转角处不该把前视点钳成投影点），那属于 B 线文件，
      本任务不动它，只在这里规避并把现象写进报告。
    """
    if idx <= 0 or idx >= len(path.waypoints) - 1:
        return True                     # 首点/末点不是"拐角顶点"，可以锚
    return path.corner_angle_deg(idx - 1) <= _ANCHOR_CORNER_TOL_DEG


def _advance_anchor(path: Path, anchor_idx: int, s_belief: float) -> int:
    """把"当前瞄准的航点"推进到第一个**信念还没领先于** 的航点。

    判据：`s_at_waypoint(i) >= s_belief`。信念弧长已经越过顶点 i，就说明
    信念自认为走过了该点 —— 此时再把它吸回 i 的格心会形成**自锁**
    （吸附→弧长不动→该点仍合格→再吸附；一拍只走 3 mm ≪ 吸附半径 0.12 m）。
    ★ 语义与 `docs/P1接口约定.md` §3.1 的 FixTask 一致：锚的是"下一个要到的点"。
    """
    n = len(path.waypoints)
    while anchor_idx < n and path.s_at_waypoint(anchor_idx) < s_belief - _END_EPS_M:
        anchor_idx += 1
    return anchor_idx


def _anchor_to_waypoint(path: Path, bx: float, by: float, s_belief: float,
                        radius: float) -> tuple[float, float] | None:
    """【诊断用】信念是否落在"当前目标航点"的吸附球内？是则返回该航点格心。

    ⚠️ 主循环**不用**这个函数（它每次调用都会重新挑最近的合格航点，会把信念钉死）；
       主循环用的是"当前瞄准航点 + `_advance_anchor` 推进"的一次性版本。
       保留它只为离线排查"某一拍的信念到底该不该被吸附"。
    """
    idx = _advance_anchor(path, 0, s_belief)
    if idx >= len(path.waypoints):
        return None
    wp = path.waypoints[idx]
    if math.hypot(wp.x - bx, wp.y - by) <= radius:
        return (wp.x, wp.y)
    return None


def _schedule(queue: deque[tuple[float, float, float]], at: float,
              v: float, steer_deg: float) -> None:
    """把一条指令插进延迟队列（按生效时刻升序，用 bisect 保持有序）。"""
    keys = [item[0] for item in queue]
    idx = bisect.bisect_right(keys, at)
    queue.insert(idx, (at, v, steer_deg))


def _take(queue: deque[tuple[float, float, float]], now: float,
          v_hold: float, steer_hold: float) -> tuple[float, float]:
    """取出"到点生效"的指令；**队列里没有新指令时保持上一条**（`v_hold`/`steer_hold`）。

    ★ 这是下位机的真实行为：串口每拍都在发，但"没收到新指令"绝不等于"舵角回正/停车"。
    ⚠️ 老实现队列空时返回 `(0, 0)`：`dt(0.02) < cmd_latency_s(0.06)` 时队列**每拍都是空的**，
      于是舵角在 ±30° 与 0 之间逐拍跳（实测执行侧 1500 °/s），真值在"打死的轮子"和
      "回正的轮子"之间反复横跳 —— 那不是任何真实车辆的运动。上电第一拍没有指令 =
      静止（初值 0），此后必须保持。
    """
    last_v, last_steer = v_hold, steer_hold
    while queue and queue[0][0] <= now:
        _at, last_v, last_steer = queue.popleft()
    return last_v, last_steer


class _Rng:
    """最小可复现高斯噪声源（Box–Muller）——不引第三方依赖，种子固定则结果固定。"""

    __slots__ = ("_state", "_spare")

    def __init__(self, seed: int) -> None:
        self._state = (int(seed) & 0xFFFFFFFF) or 0x1234567
        self._spare: float | None = None

    def _uniform(self) -> float:
        # xorshift32：周期足够、无依赖、逐位可复现
        x = self._state
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= x >> 17
        x ^= (x << 5) & 0xFFFFFFFF
        self._state = x & 0xFFFFFFFF
        return (self._state + 0.5) / 4294967296.0

    def gauss(self) -> float:
        """标准正态样本（均值 0、标准差 1）。"""
        if self._spare is not None:
            out, self._spare = self._spare, None
            return out
        u1 = max(self._uniform(), 1e-12)
        u2 = self._uniform()
        r = math.sqrt(-2.0 * math.log(u1))
        theta = 2.0 * math.pi * u2
        self._spare = r * math.sin(theta)
        return r * math.cos(theta)


# --------------------------------------------------------------------------- #
#  指标清单与打印
# --------------------------------------------------------------------------- #
#: 契约要求的指标键（存在性由测试断言；缺一个都算接口违约）。
METRIC_KEYS: tuple[str, ...] = (
    "completed", "time_s", "cross_track_rms_m", "cross_track_max_m", "final_error_m",
    "overshoot_max_m", "max_steer_deg", "max_steer_rate_deg_s", "path_len_ratio",
    "anchors", "steps",
)

#: ★ 分组指标键（`docs/P1接口约定.md` 调参用；老实现全缺 → 无法定位误差来源）。
GROUPED_METRIC_KEYS: tuple[str, ...] = (
    "cross_track_rms_normal_m", "cross_track_rms_reverse_m", "cross_track_rms_corner_m",
    "cross_track_max_normal_m", "cross_track_max_reverse_m", "cross_track_max_corner_m",
    "vertex_overshoot_normal_m", "vertex_overshoot_reverse_m",
    "steer_rate_p95_deg_s", "steer_rate_max_deg_s",
    "steer_delivered_rate_p95_deg_s", "steer_sign_flip_steps",
    "steer_delivered_jump_steps", "steer_rate_limit_deg_s",
    "time_s_normal", "time_s_reverse", "time_s_corner",
)

#: 控制台指标表的**主表**：(键, 中文标题, 单位, 小数位) —— 分组指标在 `format_metrics` 里另排。
_METRIC_TABLE: tuple[tuple[str, str, str, int], ...] = (
    ("completed",        "跑完全程",     "",        0),
    ("time_s",           "总时长",       "s",       2),
    ("steps",            "拍数",         "",        0),
    ("cross_track_rms_m", "横向RMS(真值)", "m",     4),
    ("cross_track_max_m", "横向MAX(真值)", "m",     4),
    ("belief_cross_track_rms_m", "横向RMS(信念)", "m", 4),
    ("belief_error_rms_m", "信念↔真值RMS", "m",     6),
    ("belief_error_max_m", "信念↔真值MAX", "m",     6),
    ("belief_error_max_free_m", "信念↔真值MAX(非锚定拍)", "m", 6),
    ("overshoot_max_m",  "顶点超调",     "m",       4),
    ("final_error_m",    "末点误差(真值)", "m",     4),
    ("max_steer_deg",    "最大舵角",     "°",       2),
    ("max_steer_rate_deg_s", "舵角速率(指令)", "°/s", 1),
    ("steer_rate_p95_deg_s", "舵角速率(指令)P95", "°/s", 1),
    ("max_steer_rate_delivered_deg_s", "舵角速率(执行)", "°/s", 1),
    ("path_len_ratio",   "路径长度比",   "×",       4),
    ("anchors",          "到位吸附次数", "",        0),
    ("out_of_bounds_steps", "越界拍数",  "",        0),
    ("path_length_m",    "路线总长",     "m",       2),
)

#: 分组指标表：(键, 中文标题, 单位, 小数位)
_GROUP_TABLE: tuple[tuple[str, str, str, int], ...] = (
    ("cross_track_rms_normal_m",  "横向RMS 直段",   "m", 4),
    ("cross_track_rms_reverse_m", "横向RMS 倒车段", "m", 4),
    ("cross_track_rms_corner_m",  "横向RMS 拐角",   "m", 4),
    ("cross_track_max_normal_m",  "横向MAX 直段",   "m", 4),
    ("cross_track_max_reverse_m", "横向MAX 倒车段", "m", 4),
    ("cross_track_max_corner_m",  "横向MAX 拐角",   "m", 4),
    ("vertex_overshoot_normal_m", "顶点超调 非倒车", "m", 4),
    ("vertex_overshoot_reverse_m", "顶点超调 倒车", "m", 4),
    ("time_s_normal",             "耗时 直段",     "s", 2),
    ("time_s_reverse",            "耗时 倒车段",   "s", 2),
    ("time_s_corner",             "耗时 拐角",     "s", 2),
    ("steer_rate_p95_deg_s",      "指令速率P95",    "°/s", 1),
    ("steer_delivered_rate_p95_deg_s", "执行速率P95", "°/s", 1),
    ("steer_sign_flip_steps",     "倒车符号翻转拍", "",  0),
    ("steer_delivered_jump_steps", "执行侧越限跳变拍", "", 0),
)

#: ★ t24 场地物理表：(键, 中文标题, 单位, 小数位)。
#: ⚠️ **只在接了地图时打印**（`metrics["field_modeled"] > 0`）—— 否则"没建模"与"零违规"
#:    在控制台上长得一模一样，那是在说谎。
_FIELD_TABLE: tuple[tuple[str, str, str, int], ...] = (
    ("mine_pressed_count",      "盖雷(块/总数)",  "",  0),
    ("mine_press_steps",        "盖雷总拍数",     "",  0),
    ("mine_worst_clearance_m",  "最远雷心距",     "m", 4),
    ("obstacle_hit_steps",      "障碍碰撞拍",     "",  0),
    ("obstacle_breach_steps",   "障碍净空不足拍", "",  0),
    ("obstacle_breach_corner_steps", "└其中拐角拍", "", 0),
    ("obstacle_min_center_dist_m", "车心距柱心(最近)", "m", 4),
    ("barrier_breach_steps",    "挡板触线拍",     "",  0),
    ("barrier_min_distance_m",  "挡板净距(最近)", "m", 4),
    ("board_overlap_steps",     "车体压靶板拍",   "",  0),
    ("board_breach_steps",      "靶板净空不足拍", "",  0),
    ("board_min_clearance_m",   "靶板净距(最近)", "m", 4),
    ("k_column_steps",          "K 列拍数",       "",  0),
    ("k_seal_violation_steps",  "K 列封死违规拍", "",  0),
    ("k_solid_cell_steps",      "K 列非通格拍",   "",  0),
)


def _fmt_row(key: str, label: str, unit: str, nd: int, m: dict) -> str:
    value = m.get(key)
    if isinstance(value, bool):
        text = "是" if value else "否"
    elif isinstance(value, (int, float)):
        text = f"{float(value):.{nd}f}"
    else:
        text = str(value)
    return f"  {label:<16}{text:>12} {unit}"


def format_metrics(result: SimResult, sim: SimConfig | None = None,
                   title: str = "") -> str:
    """把指标表排成等宽文本（控制台回贴、报告引用都用它）。

    ★ 两张表：主表（契约键 + 信念误差）+ **分组表**（直段/倒车/拐角分开）。
      没有分组表，"横向 RMS 0.20 m"既可能是整条路均匀偏 0.2，也可能是三个拐角各偏 1 m ——
      这两种情况的修法完全相反（改前视距离 vs 改转角策略）。
    """
    m = result.metrics
    lines: list[str] = []
    if title:
        lines.append(title)
    lines.append("-" * 46)
    for key, label, unit, nd in _METRIC_TABLE:
        lines.append(_fmt_row(key, label, unit, nd, m))
    if sim is not None:
        lines.append("-" * 46)
        lines.append(f"  注入: dt={sim.dt}s 延迟={sim.cmd_latency_s}s "
                     f"里程误差={sim.speed_scale_err:+.0%} 零漂={sim.yaw_drift_deg_per_min}°/min "
                     f"噪声={sim.pos_noise_m}m")
        lines.append(f"  锚定: {'开' if sim.anchor_on_arrival else '关'}"
                     f"（半径 {sim.anchor_radius_m}m）"
                     f"  执行侧限速: {rate_desc(sim, m)}")
    lines.append("-" * 46)
    lines.append("  分组指标（按真值投影的段类型）:")
    for key, label, unit, nd in _GROUP_TABLE:
        lines.append(_fmt_row(key, label, unit, nd, m))
    # ---- ★ t24 场地物理表（只有接了地图才出现；见 _FIELD_TABLE 的 ⚠️）----
    if m.get("field_modeled", 0.0):
        rep = result.field_report or {}
        lines.append("-" * 46)
        lines.append(f"  场地物理（{rep.get('spec', '已建模')}）:"
                     + ("  ⚠️ 撞停于 %.2f s" % m.get("field_stop_t", -1.0)
                        if m.get("field_stopped") else ""))
        for key, label, unit, nd in _FIELD_TABLE:
            lines.append(_fmt_row(key, label, unit, nd, m))
        for tag, (t_hit, block, value) in sorted((rep.get("first") or {}).items()):
            lines.append(f"  ⚠️ 首次{tag}: t={t_hit:.2f}s @{block}（实测 {value:.4f} m）")
    if m.get("diverged"):
        lines.append(f"  ⚠️ 跑飞看门狗触发于第 {int(m.get('diverged_at_step', 0))} 拍"
                     f"（投影弧长连续不前进且已远离路线）")
    if m.get("hit_step_limit"):
        lines.append(f"  ⚠️ 撞到步数上限 {int(m.get('max_steps', 0))} —— 未跑完，"
                     f"可能是参数病态（例如锚定半径过大把车锚住）")
    return "\n".join(lines)


def rate_desc(sim: SimConfig, m: dict) -> str:
    """执行侧限速的一句话描述（控制台用，避免"到底限没限"再变成一次口头争论）。"""
    if not sim.actuator_rate_limit:
        return "关（一步到位，仅对照用）"
    return f"斜坡 {m.get('steer_rate_limit_deg_s', 0):.0f} °/s"


def format_segment_table(result: SimResult, top: int = 0, title: str = "") -> str:
    """逐段横向误差峰值表（`top > 0` 时只打印峰值最大的 `top` 段）。

    ★ 这张表回答的是"**哪几个拐角在中枪**"：段索引 / 起止格 / 峰值 / 峰值时刻 / 转角。
    """
    rows = sorted(result.segments, key=lambda s: -s.peak_m)
    if top > 0:
        rows = rows[:top]
    else:
        rows = sorted(rows, key=lambda s: s.index)
    lines: list[str] = []
    if title:
        lines.append(title)
    lines.append(f"  {'段':>3} {'起点':>6} {'终点':>6} {'长(m)':>7} {'类型':>7} "
                 f"{'转角(°)':>8} {'峰值(m)':>8} {'时刻(s)':>8}")
    for s in rows:
        lines.append(f"  {s.index:>3} {s.from_block:>6} {s.to_block:>6} {s.length_m:>7.3f} "
                     f"{s.kind:>7} {s.corner_deg:>8.1f} {s.peak_m:>8.4f} {s.peak_t:>8.2f}")
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
#  输出：CSV / SVG（手写，不依赖 matplotlib）
# --------------------------------------------------------------------------- #
def write_csv(result: SimResult, path: str) -> str:
    """写逐拍轨迹 CSV。返回**绝对路径**（便于回贴日志）。

    ★ 列 = `docs/P0接口约定.md` §5 的十列（**顺序不变**，老分析脚本不能被打断）
      + 追加诊断列：`truth_yaw,steer_cmd,truth_seg_index,truth_s,kind,anchored,reverse`。
    —— `x/y/yaw` = 信念（控制器看到的），`truth_*` = 真值；
       `steer` = **执行**舵角，`steer_cmd` = 控制器**输出**舵角（两者差执行侧斜坡）。
    """
    target = os.path.abspath(path)
    parent = os.path.dirname(target)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(target, "w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t", "x", "y", "yaw", "v", "steer",
                         "truth_x", "truth_y", "seg_index", "s",
                         "truth_yaw", "steer_cmd", "truth_seg_index", "truth_s",
                         "kind", "anchored", "reverse"])
        for i in range(len(result.t)):
            writer.writerow([
                f"{result.t[i]:.4f}",
                f"{result.x[i]:.6f}", f"{result.y[i]:.6f}", f"{result.yaw[i]:.6f}",
                f"{result.v[i]:.5f}", f"{result.steer[i]:.4f}",
                f"{result.truth_x[i]:.6f}", f"{result.truth_y[i]:.6f}",
                result.seg_index[i], f"{result.s[i]:.6f}",
                f"{result.truth_yaw[i]:.6f}", f"{result.steer_cmd[i]:.4f}",
                result.truth_seg_index[i], f"{result.truth_s[i]:.6f}",
                kind_of(result, i), int(bool(result.anchored[i])),
                int(bool(result.reverse[i])),
            ])
    return target


def kind_of(result: SimResult, index: int) -> str:
    """采样点 i 的分组标签（与分组指标同一口径：倒车优先，其次顶点邻域）。"""
    if index < len(result.reverse) and result.reverse[index]:
        return "reverse"
    if index < len(result.near_vertex) and result.near_vertex[index]:
        return "corner"
    return "normal"


def write_segments_csv(result: SimResult, path: str) -> str:
    """写**逐段横向误差峰值表** CSV（返回绝对路径）。

    ★ 用 Excel 打开就能按峰值排序 —— 调参时第一步永远是"看哪几段最差"，
      而不是盯着一个全局 RMS 猜。
    """
    target = os.path.abspath(path)
    parent = os.path.dirname(target)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(target, "w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["index", "from_block", "to_block", "length_m", "kind",
                         "corner_deg", "peak_m", "peak_t", "peak_s"])
        for s in result.segments:
            writer.writerow([s.index, s.from_block, s.to_block, f"{s.length_m:.4f}",
                             s.kind, f"{s.corner_deg:.2f}", f"{s.peak_m:.6f}",
                             f"{s.peak_t:.4f}", f"{s.peak_s:.6f}"])
    return target


def _traj_color(metrics: dict) -> str:
    """按当前误差从"准到偏"取色 —— 不同误差参数的 SVG 摆在一起能一眼分辨。"""
    err = float(metrics.get("belief_error_rms_m", 0.0) or 0.0)
    for limit, color in _COLOR_BY_ERROR:
        if err <= limit:
            return color
    return _COLOR_WORST


def write_svg(result: SimResult, path: str, route: Route,
              title: str = "", field: FieldSpec | None = None) -> str:
    """写轨迹 SVG（路线折线 + 航点 + 真值轨迹 + 信念轨迹）。返回绝对路径。

    ★ 手写 SVG：车上/CI 都没有 matplotlib，而"看图找问题"是调参的主要手段。
    ★ t24：给了 `field` 就**按物理形状**把场地画出来（雷 = 12×12 cm 方、障碍 = r12.75 cm 圆、
      挡板 = 2 cm 细条、K 列桥室 = 格间封死横线、靶板 = 35×35 cm 块），形状尺寸取自
      `result.field_report["shape"]`（与碰撞判据**同源**，不另抄一份数字）。
    """
    target = os.path.abspath(path)
    parent = os.path.dirname(target)
    if parent:
        os.makedirs(parent, exist_ok=True)

    route_pts = [(wp.x, wp.y) for wp in route.waypoints]
    pts = list(route_pts) + list(zip(result.truth_x, result.truth_y)) \
        + list(zip(result.x, result.y))
    shape = dict((result.field_report or {}).get("shape") or {})
    if field is not None and shape:
        # ★ 画场地时把**整块场地**纳入视野：否则"车在场地哪个位置"就看不出来了
        pts += [(0.0, 0.0), (shape["rows"] * shape["block_m"], shape["cols"] * shape["block_m"])]
    if not pts:
        pts = [(0.0, 0.0)]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    # 退化跨度（直线路线）时给一个最小跨度，否则除零/缩放爆炸
    if x_max - x_min < 1e-3:
        x_min, x_max = x_min - 0.5, x_max + 0.5
    if y_max - y_min < 1e-3:
        y_min, y_max = y_min - 0.5, y_max + 0.5
    span_x, span_y = x_max - x_min, y_max - y_min
    inner_w = _SVG_W - 2 * _SVG_PAD
    inner_h = _SVG_H - 2 * _SVG_PAD
    scale = min(inner_w / span_x, inner_h / span_y)
    off_x = _SVG_PAD + (inner_w - span_x * scale) / 2.0
    off_y = _SVG_PAD + (inner_h - span_y * scale) / 2.0

    def sx(x: float) -> float:
        return off_x + (x - x_min) * scale

    def sy(y: float) -> float:
        # SVG 的 y 轴向下，地图 y 向上 → 翻一下，否则图是镜像的
        return _SVG_H - (off_y + (y - y_min) * scale)

    def polyline(points: list[tuple[float, float]], color: str, width: float,
                 dash: str = "", opacity: float = 1.0) -> str:
        if len(points) < 2:
            return ""
        body = " ".join(f"{sx(px):.2f},{sy(py):.2f}" for px, py in points)
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        return (f'  <polyline class="traj" points="{body}" fill="none" stroke="{color}"'
                f' stroke-width="{width:.2f}" stroke-linejoin="round" stroke-linecap="round"'
                f' opacity="{opacity:.2f}"{dash_attr}/>\n')

    m = result.metrics
    truth_color = _traj_color(m)
    parts: list[str] = []
    parts.append(f'<?xml version="1.0" encoding="UTF-8"?>\n')
    parts.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{_SVG_W:.0f}" '
                 f'height="{_SVG_H:.0f}" viewBox="0 0 {_SVG_W:.0f} {_SVG_H:.0f}">\n')
    parts.append(f'  <rect x="0" y="0" width="{_SVG_W:.0f}" height="{_SVG_H:.0f}" fill="#ffffff"/>\n')
    head = title or f"pure-pursuit sim ({os.path.basename(target)})"
    parts.append(f'  <text x="{_SVG_PAD:.0f}" y="30" font-family="monospace" '
                 f'font-size="18" fill="#212529">{_xml_escape(head)}</text>\n')
    # ---- ★ t24：场地物理底图（画在所有轨迹**下面**；形状尺寸与碰撞判据同源）----
    if field is not None and shape:
        block = float(shape["block_m"])
        rows, cols = int(shape["rows"]), int(shape["cols"])
        span_x_m, span_y_m = rows * block, cols * block
        bx, by0, by1 = shape["barrier"]
        bw = float(shape["barrier_width_m"])
        k_col = int(shape["k_col"])
        k_y0 = (geo.MAP_COLS + 0.5 - k_col) * block - block / 2.0
        k_y1 = k_y0 + block
        parts.append(f'  <rect x="{sx(0.0):.2f}" y="{sy(span_y_m):.2f}" '
                     f'width="{span_x_m * scale:.2f}" height="{span_y_m * scale:.2f}" '
                     f'fill="#fcfcfc" stroke="#868e96" stroke-width="1.5"/>\n')
        # 格线（浅）
        for c in range(cols + 1):
            parts.append(f'  <line x1="{sx(0.0):.2f}" y1="{sy(c * block):.2f}" '
                         f'x2="{sx(span_x_m):.2f}" y2="{sy(c * block):.2f}" '
                         f'stroke="#e9ecef" stroke-width="1"/>\n')
        for r in range(rows + 1):
            parts.append(f'  <line x1="{sx(r * block):.2f}" y1="{sy(0.0):.2f}" '
                         f'x2="{sx(r * block):.2f}" y2="{sy(span_y_m):.2f}" '
                         f'stroke="#e9ecef" stroke-width="1"/>\n')
        # 靶板 35×35 cm（格级挡块 = 物理精确）
        for name in field.targets:
            tx_, ty_ = geo.block_to_meters(name)
            half = float(shape["board_half_m"])
            parts.append(f'  <rect class="board" x="{sx(tx_ - half):.2f}" y="{sy(ty_ + half):.2f}" '
                         f'width="{2 * half * scale:.2f}" height="{2 * half * scale:.2f}" '
                         f'fill="#e83e8c" fill-opacity="0.45" stroke="#c2185b" stroke-width="1"/>\n')
        # 障碍：圆柱 r=12.75 cm（柱心在格心）
        for name in field.obstacles:
            ox, oy = geo.block_to_meters(name)
            rr = float(shape["obstacle_radius_m"])
            parts.append(f'  <circle class="obstacle" cx="{sx(ox):.2f}" cy="{sy(oy):.2f}" '
                         f'r="{rr * scale:.2f}" fill="#495057" stroke="#212529" stroke-width="1"/>\n')
        # 雷：12×12 cm 垫片（心在格心）
        for name in field.mines:
            mx, my = geo.block_to_meters(name)
            ph_half = float(shape["mine_pad_m"]) / 2.0
            parts.append(f'  <rect class="mine" x="{sx(mx - ph_half):.2f}" y="{sy(my + ph_half):.2f}" '
                         f'width="{2 * ph_half * scale:.2f}" height="{2 * ph_half * scale:.2f}" '
                         f'fill="#ffd700" stroke="#b8860b" stroke-width="1"/>\n')
        # 挡板：2 cm 细条（D..N，含 K 段，无洞）
        parts.append(f'  <rect class="barrier" x="{sx(bx - bw / 2):.2f}" y="{sy(by1):.2f}" '
                     f'width="{bw * scale:.2f}" height="{(by1 - by0) * scale:.2f}" '
                     f'fill="#343a40"/>\n')
        # K 列桥室：通格（横向贯通）+ 格间**封死**的横壁
        open_rows = set(shape.get("open_rows") or ())
        for r in range(1, rows + 1):
            mid = (geo.MAP_ROWS + 0.5 - r) * block          # 第 r 行的行心 x
            x0 = mid - block / 2.0
            fill = "#b2f2bb" if (r - 1) in open_rows else "#e9ecef"
            parts.append(f'  <rect class="kcell" x="{sx(x0):.2f}" y="{sy(k_y1):.2f}" '
                         f'width="{block * scale:.2f}" height="{block * scale:.2f}" '
                         f'fill="{fill}" fill-opacity="0.55"/>\n')
            if r > 1:                       # 格间横壁（K 列纵向封死）
                parts.append(f'  <line class="kwall" x1="{sx(x0):.2f}" y1="{sy(k_y0):.2f}" '
                             f'x2="{sx(x0):.2f}" y2="{sy(k_y1):.2f}" stroke="#212529" '
                             f'stroke-width="2.5"/>\n')
    # ① 路线折线（底层参考）
    parts.append(polyline(route_pts, _COLOR_ROUTE, _COLOR_ROUTE_W, dash="6 4"))
    # ② 真值轨迹（主角）
    parts.append(polyline(list(zip(result.truth_x, result.truth_y)),
                          truth_color, _COLOR_TRUTH_W))
    # ③ 信念轨迹（控制器以为自己在哪）
    parts.append(polyline(list(zip(result.x, result.y)), "#adb5bd", 1.0, dash="2 3"))
    # ④ 航点
    for wp in route.waypoints:
        color = _COLOR_WAYPOINT_STOP if wp.is_stop_point else _COLOR_WAYPOINT
        radius = 4.0 if wp.is_stop_point else 2.6
        parts.append(f'  <circle class="waypoint" cx="{sx(wp.x):.2f}" cy="{sy(wp.y):.2f}" '
                     f'r="{radius:.2f}" fill="{color}"/>\n')
    # ⑤ 图例/指标一角
    legend = (f"completed={m.get('completed')}  t={m.get('time_s', 0):.1f}s  "
              f"xt_rms={m.get('cross_track_rms_m', 0):.3f}m  "
              f"belief_err={m.get('belief_error_rms_m', 0):.3f}m  "
              f"anchors={int(m.get('anchors', 0) or 0)}  "
              f"oob={int(m.get('out_of_bounds_steps', 0) or 0)}")
    parts.append(f'  <text x="{_SVG_PAD:.0f}" y="{_SVG_H - 16:.0f}" font-family="monospace" '
                 f'font-size="14" fill="#495057">{_xml_escape(legend)}</text>\n')
    parts.append("</svg>\n")
    with open(target, "w", encoding="utf-8") as fh:
        fh.write("".join(parts))
    return target


def _xml_escape(text: str) -> str:
    return (str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
            .replace('"', "&quot;"))
