# -*- coding: utf-8 -*-
"""情景（scenario）定义：场地真值 vs 车侧静态信念 vs 链路/无人机/任务/靶板参数。

★ 单一真源纪律：本模块**只读** ``dzrobot_v2/config/robot.yaml`` 与 ``config/map.yaml``；
情景变体（如"真值桥洞与地图不符"）只在内存里构造，**绝不落盘**到 dzrobot_v2/config。

## 本轮正式参数（队长 2026-10 最新口径，取代旧假设）

① 无人机：高度 1.5 m、感知半径 2.5 格、飞行速度 0.5 m/s；
② 总完赛预算 3 分钟（车全程 + 机全程 + 打靶≤30 s；机/车伴飞并发）；
③ 桥洞每场随机：入口 K7~K11、出口 K1~K6（默认格 K10/K5 只是范围合法兜底）；
④ 靶 = 可动双面板（蓝/红各一面）+ 水平轨道两格（0.7 m），朝向每场随机，0.1 m/s 来回，
   打红面 = 扣分；蓝面朝向需无人机判读（YOLO 口径）；停站点须在蓝面可见扇区（法线 ±45°，敏感度 ±30/±45/±60）；
⑤ 桥顶箭头 = 画了箭头的垫片（与雷同物理形态），YOLO 解决，不再当独立未知。
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field

from dzbrain.core.config import Config
from dzbrain.tools.sim import FieldSpec, parse_map_yaml

from .geo import block_to_index, index_to_block

_REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "..", "..", "..", "dzrobot_v2"))


def _repo_cfg() -> Config:
    return Config.load(os.path.join(_REPO, "config", "robot.yaml"))


def _repo_map_text() -> str:
    with open(os.path.join(_REPO, "config", "map.yaml"), encoding="utf-8") as fh:
        return fh.read()


@dataclass
class DroneParams:
    """无人机飞行/感知参数（本包的建模旋钮，非车侧配置）。"""

    height_m: float = 1.5                  # ① 飞行高度（2D 仿真无几何效应，仅记录口径）
    speed_mps: float = 0.5                 # ① 飞行速度（正式仿真参数）
    takeoff_delay_s: float = 0.0           # 起飞延迟（受限臂的"晚到"旋钮）
    reveal_radius_cells: float = 2.5       # ① 单拍揭示半径（格）；2.5 格 = 0.875 m
    heartbeat_hz: float = 1.0              # 心跳节奏（车侧 3 s 超时）
    reemit_s: float = 5.0                  # 全量情报周期性重发（续 30 s 新鲜度窗口）
    hover_after_land: bool = True          # 到达 M2 后悬停续发（持续覆盖姿态）


@dataclass
class LinkParams:
    """dzlink（DDS BEST_EFFORT）通信代价注入。"""

    latency_s: float = 0.06                # 量级延迟（队长补充 C）
    jitter_s: float = 0.02                 # 均匀抖动 [0, jitter]（FIFO 投递）
    drop_p: float = 0.0                    # 每报文丢包率（独立同分布）
    seed: int = 20261007                   # 可复现


@dataclass
class MissionParams:
    """车侧任务/采纳门参数（真值来自 robot.yaml，这里只是显式快照）。"""

    drone_enabled: bool = True
    heartbeat_timeout_s: float = 3.0       # drone.heartbeat_timeout_s
    info_stale_s: float = 30.0             # drone.info_stale_s（t6 BridgeIntel 过期阈值）
    bridge_info_wait_s: float = 2.0        # mission.bridge_info_wait_s（发车等情报窗口）
    cruise_speed_mps: float = 0.15         # pursuit.cruise_speed_mps（侦察截止时间用）
    auto_plan_enabled: bool = True         # planner.auto_plan_enabled
    planner_engine: str = "tracker-safe"
    budget_s: float = 180.0                # ② 总完赛预算 3 分钟
    dwell_worst_s: float = 30.0            # ② 打靶驻留最坏（0 命中 = 30 s 硬死线，t26）
    dwell_hit_s: float = 7.4               # ② 命中情景驻留（settle 0.4 + acquire 2.0 + 10×0.5 s）
    wait_cap_s: float = 12.0               # 车到桥前等待新鲜情报的上限（受预算约束，见 budget 分析）
    startup_wait_s: float = 2.0            # 发车窗口（mission.bridge_info_wait_s 同值）


@dataclass
class TargetParams:
    """④ 可动双面板靶模型（本包建模，车侧代码现无此模型）。"""

    track_cells: tuple[str, ...] = ("B11", "C11")   # 水平轨道两格（0.7 m）
    panel_len_m: float = 0.14                        # ② 面板很薄、约 0.4 格宽（0.14 m）
    speed_mps: float = 0.1                           # ② 来回移动速度
    orientation_deg: float = 0.0                     # 蓝面法线（每场随机；0=朝北/朝场内）
    judge_error_deg: float = 0.0                     # 无人机判读误差（受限臂：+180°=蓝红翻转）
    sector_deg: float = 45.0                         # ② 命中时车→靶连线须在蓝面法线扇区（敏感度 ±30/±45/±60）
    lead_time_s: float = 0.35                        # 移动靶提前量（aim.lead_time_s 同源）
    shot_cooldown_s: float = 0.5
    shots: int = 10
    #: 停站点候选（都在现役/重规划路线上，行 10 邻靶格；A11/D11 不在路线上，作为局限记录）
    stop_candidates: tuple[str, ...] = ("A10", "B10", "C10", "D10")


@dataclass
class Scenario:
    """一次对照实验的全部输入（不落盘，纯内存）。"""

    name: str
    truth: FieldSpec                       # 场地**真值**（无人机权威；物理在环判据用它）
    belief_bridges: tuple[str, ...]        # 车侧静态地图（map.yaml）的桥洞
    candidates: dict[str, tuple[str, ...]] # 车侧 route.cross_{half}_candidates
    defaults: dict[str, str]               # 车侧 route.cross_{half}_default（回落目标）
    arrows: dict[str, str]                 # 桥顶箭头垫片真值：桥洞格 → 穿洞方向（E/W）
    target: TargetParams = field(default_factory=TargetParams)
    drone: DroneParams = field(default_factory=DroneParams)
    link: LinkParams = field(default_factory=LinkParams)
    mission: MissionParams = field(default_factory=MissionParams)
    #: 采纳门/规划器**改进变体**开关（默认 False = 现车代码语义）：
    #:  authority_gate = 采纳门只做 列/行段 校验（去掉"候选∩地图桥洞"锁死），规划器池 ∪ {情报格}；
    #:  wait_at_bridge  = K 列三连入口格（L10）前等待新鲜情报（现车只在发车窗口等 2 s）。
    authority_gate: bool = False
    wait_at_bridge: bool = False
    notes: str = ""

    def pool(self, half: str) -> tuple[str, ...]:
        """车侧 ``mission._bridge_pool`` 同口径：候选 ∩ 地图桥洞，非空取交集。"""
        cands = self.candidates.get(half, ())
        bridges = self.belief_bridges
        if cands and bridges:
            return tuple(sorted(set(cands) & set(bridges)))
        return tuple(sorted(set(cands or bridges)))


def _map_spec_from_text(text: str) -> FieldSpec:
    return parse_map_yaml(text, source="config/map.yaml（只读）")


def _base_mission(cfg: Config, **kw) -> MissionParams:
    base = dict(
        drone_enabled=bool(cfg.get("drone.enabled")),
        heartbeat_timeout_s=float(cfg.get("drone.heartbeat_timeout_s") or 3.0),
        info_stale_s=float(cfg.get("drone.info_stale_s") or 30.0),
        bridge_info_wait_s=max(0.0, float(cfg.get("mission.bridge_info_wait_s") or 0.0)),
        cruise_speed_mps=float(cfg.get("pursuit.cruise_speed_mps") or 0.15),
        auto_plan_enabled=bool(cfg.get("planner.auto_plan_enabled")),
        planner_engine=str(cfg.get("planner.engine") or "tracker-safe"),
    )
    base.update(kw)
    return MissionParams(**base)


def _base_candidates(cfg: Config) -> dict[str, tuple[str, ...]]:
    return {"lower": tuple(cfg.get("route.cross_lower_candidates") or ()),
            "upper": tuple(cfg.get("route.cross_upper_candidates") or ())}


def _base_defaults(cfg: Config) -> dict[str, str]:
    return {"lower": str(cfg.get("route.cross_lower_default")),
            "upper": str(cfg.get("route.cross_upper_default"))}


def _spec_with_bridges(spec: FieldSpec, bridges: tuple[str, ...]) -> FieldSpec:
    return FieldSpec(
        mines=spec.mines, obstacles=spec.obstacles, targets=spec.targets,
        bridges=bridges,
        barrier_col_from=spec.barrier_col_from, barrier_row_hi=spec.barrier_row_hi,
        barrier_row_lo=spec.barrier_row_lo,
        name=f"真值（桥洞 {'/'.join(bridges)}）",
        source="内存构造（不落盘）",
    )


def _spec_with_targets(spec: FieldSpec, targets: tuple[str, ...]) -> FieldSpec:
    """场地真值里的靶格替换（t28 ③：只改**内存** FieldSpec，绝不落 config/map.yaml）。"""
    return FieldSpec(
        mines=spec.mines, obstacles=spec.obstacles, targets=targets,
        bridges=spec.bridges,
        barrier_col_from=spec.barrier_col_from, barrier_row_hi=spec.barrier_row_hi,
        barrier_row_lo=spec.barrier_row_lo,
        name=f"真值（靶格 {'/'.join(targets)}）",
        source="内存构造（不落盘）",
    )


def parse_target_cells(text: str) -> tuple[str, ...]:
    """'F11 G11' / 'F11,G11' → ('F11','G11')（格名一律大写，非法抛 ValueError）。"""
    parts = [p for p in str(text).replace(",", " ").split() if p]
    if len(parts) != 2:
        raise ValueError(f"靶格必须恰好两格（同列相邻或同行相邻），收到 {parts!r}")
    cells = tuple(p.strip().upper() for p in parts)
    _validate_target_pair(cells)
    return cells


def _validate_target_pair(cells: tuple[str, ...]) -> None:
    if len(cells) != 2:
        raise ValueError(f"靶格必须恰好两格，收到 {cells!r}")
    try:
        (r1, c1), (r2, c2) = (block_to_index(c) for c in cells)
    except ValueError as exc:
        raise ValueError(f"靶格名非法（{exc}）")
    if not ((r1 == r2 and abs(c1 - c2) == 1) or (c1 == c2 and abs(r1 - r2) == 1)):
        raise ValueError(f"靶格必须同列相邻或同行相邻，收到 {'/'.join(cells)}")


def derive_stop_candidates(cells: tuple[str, ...]) -> tuple[str, ...]:
    """由靶格推停站点候选（朝场地中心一侧的邻排，±1 格展开，与 t11 的
    A10/B10/C10/D10 口径同构——B11/C11 → E10~H10？不：±1 展开 → A10..D10 同源）。

    ★ 横向对（同行相邻列）：邻排 = 朝场地中心的行（row≥7 → row-1，否则 row+1）；
    ★ 纵向对（同列相邻行）：邻排 = 朝场地中心的列（col≥7 → col-1，否则 col+1）。
    """
    (r1, c1), (r2, c2) = (block_to_index(c) for c in cells)
    out: list[str] = []
    if r1 == r2:
        nr = r1 - 1 if r1 >= 6 else r1 + 1          # 0-based；row 11(idx10)→9(row10)
        for c in range(max(0, min(c1, c2) - 1), min(13, max(c1, c2) + 1) + 1):
            out.append(index_to_block(nr, c))
    else:
        nc = c1 - 1 if c1 >= 7 else c1 + 1          # col 13(N) 邻 → 12(M)；col 0(A) 邻 → 1(B)
        for r in range(max(0, min(r1, r2) - 1), min(10, max(r1, r2) + 1) + 1):
            out.append(index_to_block(r, nc))
    return tuple(out)


def apply_target_override(scenario: "Scenario", cells: tuple[str, ...] | list[str],
                          orient_deg: float | None = None) -> "Scenario":
    """t28 ③ 靶坐标更改与方向设定：返回**内存克隆**（原情景不动、不落盘）。

    - ``truth.targets`` 与 ``target.track_cells`` 同步替换（物理在环压靶判据 + 射击模型
      都吃新靶格）；
    - 朝向（蓝面法线 φ，度）可选覆盖；
    - 停站点候选按新靶格重推（``derive_stop_candidates``）；
    - 校验：新靶格不得与雷/障/桥洞重叠（冲突抛 ValueError）。
    """
    import copy
    sc = copy.deepcopy(scenario)
    cells = tuple(str(c).strip().upper() for c in cells)
    _validate_target_pair(cells)
    clash = set(cells) & (set(sc.truth.mines) | set(sc.truth.obstacles)
                          | set(sc.truth.bridges))
    if clash:
        raise ValueError(f"靶格与场地要素冲突：{'/'.join(sorted(clash))}"
                         f"（雷/障/桥洞格不能放靶）")
    sc.truth = _spec_with_targets(sc.truth, cells)
    sc.target.track_cells = cells
    if orient_deg is not None:
        sc.target.orientation_deg = float(orient_deg) % 360.0
    sc.target.stop_candidates = derive_stop_candidates(cells)
    return sc


def build_world_a() -> Scenario:
    """世界 A：桥洞 = 默认值 K10/K5（30 组合里的 1 种，现基线世界）；靶蓝面朝北（可打）。

    用途：无无人机基线（等价现基线）/ 理想揭示 / 链路丢包 三组对照。
    """
    cfg = _repo_cfg()
    spec = _map_spec_from_text(_repo_map_text())
    return Scenario(
        name="A-桥洞K10/K5(默认),靶蓝面北",
        truth=spec,
        belief_bridges=spec.bridges,
        candidates=_base_candidates(cfg),
        defaults=_base_defaults(cfg),
        arrows={"K10": "W", "K5": "E"},
        target=TargetParams(orientation_deg=0.0, judge_error_deg=0.0),
        mission=_base_mission(cfg),
        notes="通口=默认值：无人机情报对轨迹零影响（预期），价值只在信息契约与 B/C 世界。",
    )


def build_world_b(entrance: str = "K9", exit_cell: str = "K4") -> Scenario:
    """世界 B：随机桥洞（入口≠K10、出口≠K5），靶蓝面朝东（φ=-90°，需改选停站点 D10）。

    用途：无情报直冲失败 / 现车代码情报被拒 / 桥前等待策略 / 朝向误判红面扣分。
    """
    cfg = _repo_cfg()
    spec = _map_spec_from_text(_repo_map_text())
    truth = _spec_with_bridges(spec, (entrance, exit_cell))
    return Scenario(
        name=f"B-桥洞{entrance}/{exit_cell}(随机),靶蓝面东",
        truth=truth,
        belief_bridges=spec.bridges,
        candidates=_base_candidates(cfg),
        defaults=_base_defaults(cfg),
        arrows={entrance: "W", exit_cell: "E"},
        target=TargetParams(orientation_deg=-90.0, judge_error_deg=0.0),
        mission=_base_mission(cfg),
        notes="入口/出口都偏离默认格：默认格直冲必撞封死桥室（失败概率分析见报告）。",
    )


def build_world_c() -> Scenario:
    """世界 C：最坏组合（入口 K11 揭示最晚、出口 K1 揭示最晚），靶蓝面朝南 = 贴墙不可打。

    用途：3 分钟预算下"车到桥前等待"的最长可等窗口与裕量（输出 b/c）。
    """
    cfg = _repo_cfg()
    spec = _map_spec_from_text(_repo_map_text())
    truth = _spec_with_bridges(spec, ("K11", "K1"))
    return Scenario(
        name="C-桥洞K11/K1(最坏揭示),靶蓝面南(贴墙)",
        truth=truth,
        belief_bridges=spec.bridges,
        candidates=_base_candidates(cfg),
        defaults=_base_defaults(cfg),
        arrows={"K11": "W", "K1": "E"},
        target=TargetParams(orientation_deg=180.0, judge_error_deg=0.0),
        mission=_base_mission(cfg),
        notes="最坏揭示组合 + 贴墙不可打：预算压力上限（0 命中 → 驻留 30 s）。",
    )


ALL_BRIDGE_COMBOS: tuple[tuple[str, str], ...] = tuple(
    (f"K{r_in}", f"K{r_out}")
    for r_in in range(7, 12) for r_out in range(1, 7))
