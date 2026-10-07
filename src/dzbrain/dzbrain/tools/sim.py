# -*- coding: utf-8 -*-
"""离线纯追踪仿真器（C 线，`docs/P0接口约定.md` §5）——**不碰车就能调参与回归**。

为什么要它（旧流程的痛点）
------------------------------------------------------------------
旧版调一个循迹参数要：改 YAML → 推到车 → colcon restart → 遥控摆位 → 跑完看横偏。
一轮十分钟起，还受电池/场地/云台误差干扰，**同一组参数跑不出可比结果**。
本模块把"车 + 里程计误差 + 到下位机的链路延迟"压成一个确定性模型，
同一组参数永远得到同一条曲线 —— 于是"A 参数 vs B 参数"的对照实验才成立。

模型（与现场一致的部分 / 有意简化的部分）
------------------------------------------------------------------
真值（物理世界）::

    x   += v·cos(yaw)·dt
    y   += v·sin(yaw)·dt
    yaw += v/L·tan(δ)·dt          L = pursuit.wheelbase_m，δ = 舵角（弧度）

信念（控制器**看到**的位姿，= `P_POSE`）在真值上叠加四类误差：

| 误差源 | SimConfig 字段 | 模拟的现实 |
|---|---|---|
| 里程比例误差 | `speed_scale_err` | 底盘 `ticksPerMeter` 标定偏大 → 里程计报得比真走的多 |
| 航向零漂 | `yaw_drift_deg_per_min` | 陀螺积分零偏 |
| 位置白噪声 | `pos_noise_m` | 定位抖动（每拍叠加，可与前三项叠加） |
| 到位吸附 | `anchor_on_arrival` + `anchor_radius_m` | Localization 的 L2 修正（`fix.anchor_at_waypoint`）：靠近下一航点时把信念锚到格心 |

指令延迟 `cmd_latency_s`：每拍写出的 `PursuitCommand` 先入"延迟队列"，
到点才作用于真值（模拟 50 Hz 控制环 → 下位机 115200 串口这条链路）。

⚠️ **本仿真器复现的是"信念驱动"闭环**（这是实车真实形态）：
控制器拿信念位姿投影 + 决策，指令作用在真值车体上。
后果必须写清楚，否则会误读指标（实测结论见 `docs/sim_report` 段 / 任务报告）：

1. 单纯 `speed_scale_err` **几乎不产生横向误差** —— 信念投影沿路线"多跑" 18%，
   但真值仍在同一条折线上，只是**沿弧长滞后**。所以 `cross_track_*` 对它不敏感，
   真正放大的是 `belief_error_rms_m`（信念↔真值距离）与 `overshoot_max_m`（顶点冲过头）。
2. 因此"锚定 vs 不锚定"的对照要看**信念误差**（锚定把它夹在 `anchor_radius_m` 内，
   不锚定则按 `ε × 已走弧长` 无界累积），而不是只看横向误差。
3. 位置白噪声会被 50 Hz 控制环 + 舵机速率限幅低通掉大半，**不会**像直觉那样把车甩出去。

★ 不变量
1. `run_sim()` 是**纯函数**：不读文件、不写文件、不 import rclpy，同样入参永远同样出参。
2. 所有模型参数来自 `Config`（点号键）与 `SimConfig`；本模块**不硬编码魔法数字**
   （唯一的例外是算法常数，用 `_` 前缀常量标注并说明理由）。
3. 真值轨迹**不参与控制**，只参与指标统计 —— 保证"控制器只看得见带误差的信念"。
4. `metrics` 的键名是契约（`docs/P0接口约定.md` §5），只增不改。
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
from ..route.model import Route

__all__ = [
    "SimConfig",
    "SimResult",
    "run_sim",
    "step_bicycle",
    "write_csv",
    "write_svg",
    "format_metrics",
    "METRIC_KEYS",
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
_PROJ_BELIEF_M = 0.75

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
    trace_hook: object = None           # 可选：每拍回调 (step:int, state:dict) —— 排障用（默认 None）

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
    """一次仿真的全部采样。`x/y/yaw/v/steer` = **信念**（控制器看到的），`truth_*` = 真值。"""

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
    metrics: dict = field(default_factory=dict)

    @property
    def steps(self) -> int:
        return len(self.t)

    def metric(self, key: str, default: float = float("nan")) -> float:
        """取指标（缺键返回 default，便于测试断言时给出可读的实测值）。"""
        return self.metrics.get(key, default)


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
def run_sim(route: Route, cfg: Config, sim: SimConfig | None = None) -> SimResult:
    """跑一次闭环仿真，返回 `SimResult`（含真值/信念两条轨迹与全部指标）。

    :param route: 路线（通常 `parse_mission(cfg.get("route.tokens"))` 或读路线文件）。
    :param cfg: 现场配置（一切 `pursuit.*` 都从它读，本函数不硬编码）。
    :param sim: 误差注入参数；None = 理想车（无误差、有锚定）。
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

    # ---- 期望值（契约指标名，见 docs/P0接口约定.md §5）----
    result = SimResult()
    metrics: dict = result.metrics
    for key in METRIC_KEYS:
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

    # 误差注入的**接收端**（信念）与真值分别积分：
    # 只在"信念"这条线上乘 (1+ε)，等价于里程计每一拍都多报 ε 的比例。
    v_eff = 0.0                 # 真值实际速度（延迟队列的输出）
    steer_eff = 0.0             # 真值实际舵角（延迟队列的输出，度）
    last_steer: float | None = None      # 控制器上一拍**输出**的舵角（速率限幅用）
    cmd_rate_peak = 0.0         # 指令侧舵角速率峰值（= 控制器每拍真的下了多快）
    steer_rate_peak = 0.0       # 执行侧舵角速率峰值（含延迟造成的阶跃）

    queue: deque[tuple[float, float, float]] = deque()   # (生效时刻, v, steer_deg)，按时间升序
    s_belief = 0.0              # 路线弧长（投影结果，下一拍的搜索提示）
    anchors = 0
    anchor_idx = 0              # 当前瞄准的航点索引（吸附只对它生效，且一次一个）
    target_idx = 1              # 任务当前目标航点（= 正在走的这一段，见第 0 步）
    s_prev = 0.0                # 上一拍的投影弧长（推进规则 ② 用）

    rng = _Rng(sim.seed)
    drift_per_s = math.radians(sim.yaw_drift_deg_per_min) / 60.0
    scale = 1.0 + sim.speed_scale_err
    yaw_err = 0.0               # 陀螺零漂累积角（信念 yaw − 真值 yaw）

    truth_xt: list[float] = []          # 逐步的横向误差样本（truth / belief）
    belief_xt: list[float] = []
    belief_gap: list[float] = []
    in_bounds_belief: list[float] = []
    in_bounds_truth: list[float] = []
    steps = 0
    reached_end = False
    diverged = False
    diverged_at = 0
    stall = 0
    last_s = -1.0
    t = 0.0

    while steps < max_steps:
        # ================== ① 信念（里程计 + 陀螺）：误差注入的接收端 ==================
        # ★ 关键建模决定（踩过坑）：
        #   · **航向**来自陀螺 —— 它测的是车体**真实**的角速度，所以信念 yaw = 真值 yaw + 零漂角。
        #     若"用指令舵角去积分信念 yaw"，就等于假设陀螺不存在：舵角一旦被限幅，
        #     信念 yaw 立刻与真值差 0.48 rad（实测），而车还在按真值转弯 —— 仿真结果全是假的。
        #   · **位移**是真正的航位推算：用 (1+ε)×速度 积分，于是信念沿路线**超前** ε×里程，
        #     这正是 `ticksPerMeter` 偏大的物理后果（+0.18 → 报的比走的多 18%）。
        bx += v_eff * scale * math.cos(tyaw + yaw_err) * dt
        by += v_eff * scale * math.sin(tyaw + yaw_err) * dt
        yaw_err += drift_per_s * dt                    # 陀螺零漂：误差**速率**，不是瞬时跳变
        if sim.pos_noise_m > 0.0:
            bx += rng.gauss() * sim.pos_noise_m
            by += rng.gauss() * sim.pos_noise_m
        byaw = geo.normalize_angle(tyaw + yaw_err)

        # ================== ② 到位吸附（FixTask 的 L2 位置修正） ==================
        # ★ 四条约束缺一不可（每条都是踩出来的）：
        #   ① 资格用**物理弧长**（车真的到了那个点才锚），不能用信念弧长；
        #   ② 每个航点**只锚一次**、且锚完立即推进（否则"锚→弧长不动→再锚"自锁）；
        #   ③ 只锚"还没走过"的点（否则会把信念向**后**拽 0.15 m —— 比一拍 3 mm 大 50 倍）；
        #   ④ **拐角顶点不锚**（`_anchor_allowed`）：锚到顶点会让信念落在
        #      `t=1, dist_to_vertex=0` 的退化位置，前视点塌缩成投影点本身 → 零舵角直行。
        if sim.anchor_on_arrival:
            anchor_idx = _advance_anchor(path, anchor_idx, s_prev)
            if anchor_idx < len(path.waypoints) and _anchor_allowed(path, anchor_idx):
                target_wp = path.waypoints[anchor_idx]
                if math.hypot(target_wp.x - bx, target_wp.y - by) <= sim.anchor_radius_m:
                    bx, by = target_wp.x, target_wp.y
                    anchors += 1
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
        #     · 倒车段（`R:B6`）贴着目标点，车在原地正/倒车来回抖（±0.09 m/s、位置只动 1 cm）。
        #   现实里执行器就是"过了这个点就认账、往下走"（旧 OVERSHOOT 补触发同语义）。
        if steps > 0 and target_idx < len(path.waypoints):
            wp = path.waypoints[target_idx]
            passed = s_prev >= path.s_at_waypoint(target_idx) - _END_EPS_M
            # ⚠️ 末点用更紧的到位半径：一旦按 `arrival_m`(0.12) 推进到末点，
            #    `s_floor` 立刻抬到 `total_length`，投影被夹到末点 → 控制器认为"到终点了"，
            #    车会停在离终点 0.12 m 处（实测 `final_error_m = 0.117`）。
            #    末点必须真的走到 `_GOAL_STOP_M` 内才算到 —— 与现场"终点要停车"一致。
            radius = _GOAL_STOP_M if target_idx == len(path.waypoints) - 1 else arrival
            if math.hypot(tx - wp.x, ty - wp.y) <= radius or passed:
                target_idx += 1

        # ================== ④ 物理投影（"Localization 报的 P_ROUTE_S"） ==================
        # ⚠️ 投影弧长**不允许退回上一段**：纯追踪一定会把车带过顶点一点，一旦越过顶点，
        #   `Path.project` 窗口内的"最近点"可能仍是那个顶点 → 弧长**再也不前进**
        #   → 前视点塌缩到顶点（车已在它上面）→ 方位角退化 → 舵角恒 0 → 车直着开走。
        s_floor = path.s_at_waypoint(target_idx - 1) if target_idx >= 1 else 0.0
        proj_truth = path.project(tx, ty, s_hint=s_prev, window_m=_PROJ_FWD_M,
                                  allow_backward=True)
        proj_truth = _clamp_forward(path, proj_truth, s_floor, tx, ty)
        s_prev = proj_truth.s

        # 终点判据（**用真值，不用信念**）：吸附会把信念瞬移到格心，
        # 用信念判会提前 `arrival_m` 就刹车，`final_error_m` 平白多 0.12 m。
        goal = path.waypoints[-1]
        if math.hypot(tx - goal.x, ty - goal.y) <= _GOAL_STOP_M:
            reached_end = True
        if proj_truth.s >= path.total_length - _END_EPS_M and not reached_end:
            reached_end = True      # 投影已到末点（车开过终点的情况）

        # ================== ⑤ 信念投影 → 纯追踪算一拍指令 ==================
        # 信念投影：**以物理弧长为锚 ± `_PROJ_BELIEF_M`**，再夹到当前航段之后。
        # ⚠️ 不能给它一个很大的自由窗口：车在拐角处时"最近点"可能落在**别的航段**上
        #   （现场路线在 B6/F11 附近自交），于是信念投影跳到前面 1.2 m 处，
        #   控制器拿着"我还在前面"的假信息反向打舵 —— 实测误差注入下就在这里开始绕圈。
        proj = path.project(bx, by, s_hint=proj_truth.s, window_m=_PROJ_BELIEF_M,
                            allow_backward=True)
        proj = _clamp_forward(path, proj, s_floor, bx, by)

        if reached_end and sim.stop_on_completion:
            cmd = brake_command(cfg)
        else:
            cmd = compute_command(geo.Pose(bx, by, byaw), path, proj, cfg,
                                  last_steer_deg=last_steer, dt=dt)
        if dt > 0.0:
            # 指令侧速率：控制器真的下了多快（速率限幅的实测值，理论上 ≤ max_steer_rate_deg_s）
            delta_cmd = abs(cmd.steer_deg - (last_steer if last_steer is not None else 0.0))
            if delta_cmd / dt > cmd_rate_peak:
                cmd_rate_peak = delta_cmd / dt
        last_steer = cmd.steer_deg

        # ---------- 4. 指令延迟队列（每隔 dt 写入一条，延迟到点才作用于真值） ----------
        _schedule(queue, t + sim.cmd_latency_s, cmd.linear_x, cmd.steer_deg)
        v_eff, steer_eff = _take(queue, t)
        if dt > 0.0:
            # 执行侧速率：延迟把"连续小步"变成"到点阶跃"，所以这个值会明显更大
            delta = abs(steer_eff - _prev_steer(result.steer))
            if delta / dt > steer_rate_peak:
                steer_rate_peak = delta / dt

        # ---------- 5. 真值推进（物理世界） ----------
        tx, ty, tyaw = step_bicycle(tx, ty, tyaw, v_eff, steer_eff, dt, wheelbase)

        # ---------- 6. 采样 ----------
        if steps % trace_every == 0:
            result.t.append(t)
            result.x.append(bx)
            result.y.append(by)
            result.yaw.append(byaw)
            result.v.append(v_eff)
            result.steer.append(steer_eff)
            result.truth_x.append(tx)
            result.truth_y.append(ty)
            result.truth_yaw.append(tyaw)
            result.seg_index.append(proj.seg_index)
            result.s.append(proj.s)

        truth_xt.append(_cross_track_m(tx, ty, path.segments))
        belief_xt.append(_cross_track_m(bx, by, path.segments))
        belief_gap.append(math.hypot(bx - tx, by - ty))
        in_bounds_belief.append(1.0 if geo.in_bounds(bx, by, margin) else 0.0)
        in_bounds_truth.append(1.0 if geo.in_bounds(tx, ty, margin) else 0.0)

        steps += 1
        t += dt

        if sim.trace_hook is not None:      # 排障钩子：只在显式传入时生效，正常跑无开销
            sim.trace_hook(steps, {     # type: ignore[operator]
                "t": t, "bx": bx, "by": by, "byaw": byaw, "tx": tx, "ty": ty,
                "tyaw": tyaw, "v": v_eff, "steer": steer_eff,
                "s": proj.s, "seg": proj.seg_index, "dist": proj.dist,
                "anchors": anchors, "target_idx": target_idx, "s_floor": s_floor,
                "s_truth": proj_truth.s, "goal_d": math.hypot(tx - goal.x, ty - goal.y),
                "trk": math.hypot(tx - tx0, ty - ty0),
            })

        # ---------- 7. 完成 / 跑飞判据 ----------
        # ⚠️ `reached_end` 在真值推进**之后**才可能为真（终点判据看的是真值位置），
        #   所以这里先刹车（上一拍的 `reached_end` 已让控制器低速滑行）再退出，
        #   保证末拍的真值就停在终点上（`final_error_m` 才有意义）。
        if sim.stop_on_completion and reached_end:
            break

        # ---- 跑飞看门狗：投影弧长连续 `_DIVERGE_STEPS` 拍不前进且已远离路线 → 收工 ----
        # 没有它，参数病态时仿真会白跑满步数（41000 拍里 76 秒全在绕圈），
        # 而测试只看到"未完成"，看不出是"稳不住"还是"走错路"。
        if abs(proj_truth.s - last_s) < 1e-4:
            stall += 1
        else:
            stall = 0
            last_s = proj_truth.s
        if stall >= _DIVERGE_STEPS and proj_truth.dist > 0.90:
            diverged = True
            diverged_at = steps
            break

    # ---- 收尾采样：把最终状态记一条（轨迹终点可见，不需要脑补） ----
    _record_final(result, t, bx, by, byaw, v_eff, steer_eff, tx, ty, tyaw, s_belief, steps)

    # ---- 指标 ----
    n = float(len(truth_xt)) or 1.0
    final = path.waypoints[-1]
    # "顶点冲过头"：真值轨迹在某个顶点处**离格心最近**时的超出量（超过到位半径的部分）。
    # 严格沿路线走的车该值是 0；信念领先（ε>0）时车会在拐角外侧掠过顶点 → 出现正值。
    overshoot = 0.0
    for wp in path.waypoints:
        d_min = min(math.hypot(tx2 - wp.x, ty2 - wp.y)
                    for tx2, ty2 in zip(result.truth_x, result.truth_y))
        excess = d_min - arrival
        if excess > overshoot:
            overshoot = excess

    walked = 0.0
    for i in range(1, len(result.truth_x)):
        walked += math.hypot(result.truth_x[i] - result.truth_x[i - 1],
                             result.truth_y[i] - result.truth_y[i - 1])
    final_truth_err = math.hypot(result.truth_x[-1] - final.x, result.truth_y[-1] - final.y)
    final_belief_err = math.hypot(result.x[-1] - final.x, result.y[-1] - final.y)

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
        "max_steer_rate_delivered_deg_s": steer_rate_peak,
        "path_len_ratio": (walked / path.total_length) if path.total_length > 0 else 0.0,
        "anchors": float(anchors),
        "steps": float(steps),
        # ---- 附加键（本仿真器为"看得见误差"而加，只增不改）----
        "belief_cross_track_rms_m": _rms(belief_xt),
        "belief_cross_track_max_m": max(belief_xt) if belief_xt else 0.0,
        "belief_error_rms_m": _rms(belief_gap),
        "belief_error_max_m": max(belief_gap) if belief_gap else 0.0,
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
    })
    # ⚠️ `completed` / `anchor_on_arrival` 是 bool，其余指标是 float —— 混类型是刻意的：
    #    CSV/SVG/控制台都不依赖 metrics 的类型统一，而测试要读布尔真值。
    return result


def _prev_steer(history: list[float]) -> float:
    return history[-1] if history else 0.0


def _record_final(result: SimResult, t: float, bx: float, by: float, byaw: float,
                  v_eff: float, steer_eff: float, tx: float, ty: float, tyaw: float,
                  s: float, steps: int) -> None:
    """把"最后一拍之后"的状态补记一条，让轨迹终点真的落在停车位置。"""
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


def _clamp_forward(path: Path, proj, s_floor: float, x: float, y: float):
    """投影弧长单调不退回：把 `proj` 抬到 `s_floor`（= 当前航段起点的弧长）。

    ★ 为什么需要：纯追踪过弯时必然越过顶点一点，而 `Path.project` 在窗口内的"最近点"
      可能仍是那个顶点 —— 于是弧长卡住、前视点塌缩、舵角归零、车直走。
      现实里 Mission 已经推进到下一段了，所以这里按 Mission 的语义把投影钉在段内。
    ⚠️ 只抬 `s` 与 `t`，`seg_index` 保持投影结果（它是 `int`，重建需要 `point_at` 反查）。
    """
    if proj.s >= s_floor:
        return proj
    x2, y2, seg = path.point_at(s_floor + _END_EPS_M)
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


def _take(queue: deque[tuple[float, float, float]], now: float) -> tuple[float, float]:
    """取出"到点生效"的指令；队列空时返回 (0, 0)（= 上电瞬间没有指令 = 不动）。"""
    last: tuple[float, float] = (0.0, 0.0)
    while queue and queue[0][0] <= now:
        _at, v, steer = queue.popleft()
        last = (v, steer)
    return last


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

#: 控制台指标表：(键, 中文标题, 单位, 小数位)
_METRIC_TABLE: tuple[tuple[str, str, str, int], ...] = (
    ("completed",        "跑完全程",     "",        0),
    ("time_s",           "总时长",       "s",       2),
    ("steps",            "拍数",         "",        0),
    ("cross_track_rms_m", "横向RMS(真值)", "m",     4),
    ("cross_track_max_m", "横向MAX(真值)", "m",     4),
    ("belief_cross_track_rms_m", "横向RMS(信念)", "m", 4),
    ("belief_error_rms_m", "信念↔真值RMS", "m",     4),
    ("belief_error_max_m", "信念↔真值MAX", "m",     4),
    ("overshoot_max_m",  "顶点超调",     "m",       4),
    ("final_error_m",    "末点误差(真值)", "m",     4),
    ("max_steer_deg",    "最大舵角",     "°",       2),
    ("max_steer_rate_deg_s", "舵角速率(指令)", "°/s", 1),
    ("max_steer_rate_delivered_deg_s", "舵角速率(执行)", "°/s", 1),
    ("path_len_ratio",   "路径长度比",   "×",       4),
    ("anchors",          "到位吸附次数", "",        0),
    ("out_of_bounds_steps", "越界拍数",  "",        0),
    ("path_length_m",    "路线总长",     "m",       2),
)


def format_metrics(result: SimResult, sim: SimConfig | None = None,
                   title: str = "") -> str:
    """把指标表排成等宽文本（控制台回贴、报告引用都用它）。"""
    m = result.metrics
    lines: list[str] = []
    if title:
        lines.append(title)
    lines.append("-" * 46)
    for key, label, unit, nd in _METRIC_TABLE:
        value = m.get(key)
        if isinstance(value, bool):
            text = "是" if value else "否"
        elif isinstance(value, (int, float)):
            text = f"{float(value):.{nd}f}"
        else:
            text = str(value)
        lines.append(f"  {label:<14}{text:>12} {unit}")
    if sim is not None:
        lines.append("-" * 46)
        lines.append(f"  注入: dt={sim.dt}s 延迟={sim.cmd_latency_s}s "
                     f"里程误差={sim.speed_scale_err:+.0%} 零漂={sim.yaw_drift_deg_per_min}°/min "
                     f"噪声={sim.pos_noise_m}m")
        lines.append(f"  锚定: {'开' if sim.anchor_on_arrival else '关'}"
                     f"（半径 {sim.anchor_radius_m}m）")
    if m.get("hit_step_limit"):
        lines.append(f"  ⚠️ 撞到步数上限 {int(m.get('max_steps', 0))} —— 未跑完，"
                     f"可能是参数病态（例如锚定半径过大把车锚住）")
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
#  输出：CSV / SVG（手写，不依赖 matplotlib）
# --------------------------------------------------------------------------- #
def write_csv(result: SimResult, path: str) -> str:
    """写逐拍轨迹 CSV。返回**绝对路径**（便于回贴日志）。

    列：`t,x,y,yaw,v,steer,truth_x,truth_y,seg_index,s`
    —— `x/y/yaw` = 信念（控制器看到的），`truth_*` = 真值。
    """
    target = os.path.abspath(path)
    parent = os.path.dirname(target)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(target, "w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t", "x", "y", "yaw", "v", "steer",
                         "truth_x", "truth_y", "seg_index", "s"])
        for i in range(len(result.t)):
            writer.writerow([
                f"{result.t[i]:.4f}",
                f"{result.x[i]:.6f}", f"{result.y[i]:.6f}", f"{result.yaw[i]:.6f}",
                f"{result.v[i]:.5f}", f"{result.steer[i]:.4f}",
                f"{result.truth_x[i]:.6f}", f"{result.truth_y[i]:.6f}",
                result.seg_index[i], f"{result.s[i]:.6f}",
            ])
    return target


def _traj_color(metrics: dict) -> str:
    """按当前误差从"准到偏"取色 —— 不同误差参数的 SVG 摆在一起能一眼分辨。"""
    err = float(metrics.get("belief_error_rms_m", 0.0) or 0.0)
    for limit, color in _COLOR_BY_ERROR:
        if err <= limit:
            return color
    return _COLOR_WORST


def write_svg(result: SimResult, path: str, route: Route,
              title: str = "") -> str:
    """写轨迹 SVG（路线折线 + 航点 + 真值轨迹 + 信念轨迹）。返回绝对路径。

    ★ 手写 SVG：车上/CI 都没有 matplotlib，而"看图找问题"是调参的主要手段。
    """
    target = os.path.abspath(path)
    parent = os.path.dirname(target)
    if parent:
        os.makedirs(parent, exist_ok=True)

    route_pts = [(wp.x, wp.y) for wp in route.waypoints]
    pts = list(route_pts) + list(zip(result.truth_x, result.truth_y)) \
        + list(zip(result.x, result.y))
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
