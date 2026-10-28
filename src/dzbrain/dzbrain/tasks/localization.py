# -*- coding: utf-8 -*-
"""LocalizationTask —— 定位与锚定（20 Hz，架构 §3.1 的 T1）。

职责（docs/P1接口约定.md §3.1，逐条落地）：
1. 每拍读 `bridge.pose()`（io 层已把**格单位 + 原始车体 yaw** 换算成米制地图 Pose），
   `None` 或 `bridge.pose_age()` 为 inf → **不写 `P_POSE`**（宁愿让下游看到"没有位姿"，
   也不要用旧值冒充新值）、写 `P_ODOM_AGE = inf`。
2. 应用 `P_FIX`（`core.state.FixState`，P1 阶段全 0）：`FixState.corrected()` 之后再写 `P_POSE`；
   未修正的原值写 `P_POSE_RAW`（遥测/复盘要能看出"修正了多少"）。
3. 有 `P_ROUTE` 时投影：`Path(route).project(x, y, s_hint=上一拍的 s)` → 写 `P_ROUTE_INDEX`/`P_ROUTE_S`。
   ★ 必须带 `s_hint`：没有它每拍都是 O(整条路线) 全扫（旧 `find_nearest_segment` 的热点），
   而且路线自交时会误锁到另一条边。
   ★★ **卡死自愈**（`_heal_stuck`，2026-10-07 补）：带 `s_hint` 的投影有一个**吸收态** ——
   `Path.project(allow_backward=False)` 把窗口下界夹在 `s_hint` 上（`t_floor`），
   车的垂足一旦落到窗口**之前**，"窗口内最近点"就永远是下界点本身 ⇒ 返回的 `s` 恰好
   等于上一拍 hint ⇒ 下一拍 hint 不变 = 不动点，车继续走而上报弧长**永久停住**。
   实测（bench 日志 + 离线逐值复算）冻结值 `(12,4.41) (15,6.78) (19/20,9.10) (14,4.9)
   (10,3.73) (17/18,8.05)`，上报投影点离线 0.5~3.5 m 占 20~37% 拍；`tasks/fix.py`
   的段比值因此被污染成物理不可能的 0.12~0.23。对策：三条判据同时成立才做**一次**
   全路线重投影（不是每拍全扫）。
4. **到位吸附**（`fix.anchor_at_waypoint`）：车距"当前段末端航点 / 最近的未完成航点"
   小于 `pursuit.arrival_m` 时，把 `P_POSE` 的**位置**锚到该航点格心（yaw 不动），
   发 `EV_POSE_ANCHORED`，并把本次锚定前的偏差记进 `P_FIX` 的
   `anchor_count / last_anchor / drift_m / max_drift_m`。
   为什么要有这一步：纯轮式死推里程计会累积漂移，锚定把"每过一个已知航点"变成一次
   误差归零的机会；偏差不丢弃而是记进 `FixState`，P3 的 FixTask 才有估计素材。
5. 漂移告警：单段漂移 > `pursuit.drift_warn_m` → `EV_DRIFT_WARNING`（`core.logging.Throttle` 限流）。
6. ★★ **航向完整性守卫**（`_update_yaw_guard`，2026-10-19 task-44）：每拍用
   **相邻两帧位姿**估 yaw 率，与偏航率**经验包络** `max(spin, gain·v)` 比；超限/单帧跳变/IMU 矛盾
   持续 `localization.yaw_bad_ticks` 个新帧 ⇒ 判「航向失去可信」，经 `P_MOTION_LIMIT` +
   `EV_TASK_FAULT` 请求停车（Safety 仍锁存）；连续 `yaw_clear_ticks` 个可信新帧后守卫自动复位。

铁律（core/task.py 文档 + P1 §4）：
- `tick()` 非阻塞、无长 I/O；
- **不 import rclpy**：只依赖注入的 `bridge` 鸭子类型（车端 `io.ros_io.RosBridge` / 测试 `fakes.FakeBridge`）；
- **不调 `time.time()/time.monotonic()`**：一律用注入的 `clock()`，否则测试无法快进时间。

本文件不改任何 core/route/pursuit/io 代码，只消费它们的公开接口。
"""

from __future__ import annotations

import math
from typing import Callable, Protocol

from ..core import geometry as geo
from ..core.blackboard import (P_FIX, P_ODOM_AGE, P_POSE, P_POSE_RAW, P_ROUTE,  # noqa: F401
                               P_ROUTE_INDEX, P_ROUTE_S, Blackboard)
from ..core.config import Config
from ..core.events import EV_DRIFT_WARNING, EV_POSE_ANCHORED, EV_TASK_FAULT, EventBus
from ..core.logging import Throttle
from ..core.state import FixState
from ..core.task import TaskThread
from ..pursuit.path import Path
from ..route.model import Route

__all__ = ["LocalizationTask", "PoseSource", "P_MOTION_LIMIT"]

_INF = float("inf")
#: 自愈判定的"有没有变"容差（米）：全扫与窗口投影的差异小于它就算一致（浮点噪声/顶点归属）。
_EPS_S = 1e-9

#: ★ 自愈跳跃的"大小"分界（米）：跳跃不超过它就直接采纳（正常追回冻结的几厘米）。
_STUCK_MAX_JUMP_M = 0.50
#: ★ 大跳跃时的**可信离线距离**（米）：全路线重投影给出的新位置若离车超过这么远，
#: 说明车已经不在任何一条路线上，"最近的那条腿"只是几何上稍近一点（很可能是路线的
#: 另一支/终点段），采纳它等于把上报弧长瞬移到 10 m 之外。取 0.45 m（一个整格 0.35 m
#: 的 1.3 倍，与 pursuit.corner_slow_dist_m 同量级）：实测合法自愈（车已贴在新走廊上）
#: 离线 0.33 m，而 2026-10-17 实车那次误锁离线 0.68 m —— 取两者之间的最保守分界。
#:
#: ★ 2026-10-17 14:59 趟的实证（_tmp_session/run_20261017_1504/7_path_utf8.txt）：
#:   行 207 TrackerTask 先按"更近 + 不倒退"接受了 s 1.288 → 14.112 m（+12.824 m，
#:   新段离线 0.683 m）；行 210 本任务的 _heal_stuck 同拍也接受 1.290 → 14.110
#:   （+12.820 m）。两处都只判"新投影更近"，**没有任何跳跃上限** —— 于是车在 K9 桥洞
#:   蹭墙偏航后，投影把"稍近一点"的终点段当成真值，前视点瞬间指到 12.7 m 之外，
#:   转角减速层同时失效（0.12 → 0.35 m/s），车直线冲出场地边界。
_STUCK_TRUST_DIST_M = 0.45

#: 末点（路线最后一个航点）的**吸附半径**（米）：对齐 `tools/sim.py` 的 `_GOAL_STOP_M`。
#: 末点吸附若沿用 `arrival_m`(0.12)，信念会在离终点 0.118 m 处被提前拽到格心（车端
#: 7_path.log 实测"到位吸附 A10 偏差 0.118 m → 锚回格心"）→ Mission 见信念已到 →
#: FINISH 提前刹车 → 物理车停在格心前 ~11 cm。收紧到 0.03 m：偏差落在停车容差内，
#: FINISH 只可能因"真到了"触发。非末点航点仍按 `arrival_m` 吸附，行为完全不变。
_GOAL_STOP_M = 0.03

#: ★漂移**升级**（限速/停机）的附加余量（米）：_report_drift 告警仍按 drift_warn_m(0.30)，
#: 但"判不健康 → drift_run → 降速/停车"必须超过 drift_warn_m + 它。
#: 为什么需要（2026-10-18 倒车窗口实测）：仿真真值在 R:H9 折返入口的**正常切角**横向离线
#: 0.304~0.308 m（只是略大于 0.30 的门限），20 Hz 下 0.4 s 就攒够 8 拍 ⇒ 假 drift_stop；
#: 而 15:04 类真漂移全路线离线 0.62~0.78 m，仍远大于本门限（0.35）。取 0.05：既盖住
#: 0.308 的正常切角，又不放过 0.40 m 的真横向漂移（test_localization §⑮ 双带牙）。
_DRIFT_ESCALATE_MARGIN_M = 0.05

# --------------------------------------------------------------------------- #
#  ★★ 航向完整性守卫的默认阈值（现场可经 SCHEMA `localization.yaw_*` 覆盖）
# --------------------------------------------------------------------------- #
#: 偏航率**经验包络**（2026-10-19 task-44「T-HEAD 航向闩锁修复」）：
#:   背景（2026-10-17 14:59 K9 事故）：IMU 地图 yaw 1.8 s 内 +90°→−78°（峰值 120.3°/s
#:   @ 0.112 m/s）；旧版把 yaw 当"直接可用观测" ⇒ 下游一路用坏航向直到出界。
#:   ★ 2026-10-27 现场更正（旧注释写"本车是差速底盘，可原地转向"，是错的）：
#:     本车是**阿克曼**底盘 —— `pursuitAngle.angular.z` 是**舵角(度)**（底盘侧
#:     `servo=角+60` 夹到 ±35°），`pursuit/controller.py` 自己写的就是阿克曼自行车模型，
#:     没有任何左右轮独立速度通道 ⇒ **不存在原地旋转**。名义最小转弯半径
#:     `L/tan(δmax)=0.157/tan30°≈0.272 m`，掉头横向扫幅 ≈0.544 m（> 一格 0.35 m）。
#:   · 但**本段的数值增益不因此改动**：`yaw_diff_gain_dps_per_mps=525` 是 A2 实测
#:     79.8°/s @ v=0.19（≈420°/s per m/s）再加 25% 余量标定的**经验包络**，而 420 已高于
#:     名义阿克曼极限 210.7°/s per m/s —— 说明实车 yaw 率不服从名义模型（含滑移/有效轴距
#:     差异）。按名义值收紧会把实测到的真实运动判成"物理不可能"⇒ 假停车。
#:     **结论：改前提（不能再假设原地转向），不改这个经验阈值；要改必须先用实车数据重标。**
#:   · 低速/原地：仅作包络下限 `yaw_diff_spin_dps`（60°/s），**不代表车真能原地转**；
#:   · 随速度：yaw 率与 v 成正比（当前取 525°/s per m/s，见上）；
#:   · 包络 = `max(yaw_diff_spin_dps, yaw_diff_gain_dps_per_mps × v)`（v≥min_speed）；
#:     v<min_speed 仍走静止判据 `yaw_static_dps`（停车后被转/被搬 = 不可信）；
#:   · 单帧 yaw 跳变 > `yaw_jump_deg`（30°）⇒ 立即判故障（重定位/坐标跳变）；
#:   · 连续 `yaw_clear_ticks`（20 个新帧 ≈ 5 Hz 下 4 s）可信 ⇒ 自动复位守卫（Safety 另清）。
_DEFAULT_YAW_DIFF_GAIN_DPS_PER_MPS = 525.0
_DEFAULT_YAW_DIFF_SPIN_DPS = 60.0
_DEFAULT_YAW_JUMP_DEG = 30.0
_DEFAULT_YAW_CLEAR_TICKS = 20
_DEFAULT_YAW_IMU_CHECK_DPS = 30.0
_DEFAULT_YAW_STATIC_DPS = 45.0
_DEFAULT_YAW_MIN_SPEED_MPS = 0.03
_DEFAULT_YAW_BAD_TICKS = 3
_DEFAULT_YAW_PLAUSIBLE_SPEED_MPS = 0.70
_DEFAULT_YAW_MAX_DT_S = 0.60
_DEFAULT_YAW_IMU_MISMATCH_RATIO = 0.35


def _cfg_float(cfg: Config, key: str, fallback: float) -> float:
    """取数值配置：**只有键缺失（None）才用兜底值** —— 0 是合法值，不许用 `or` 吞掉。"""
    value = cfg.get(key)
    return float(fallback if value is None else value)


# --------------------------------------------------------------------------- #
#  ★★ 新黑板键：限速/停机请求（对外接口）
# --------------------------------------------------------------------------- #
#: `P_MOTION_LIMIT`：Localization **写**、执行侧（Tracker / pursuit 的速度钳制）**读**。
#:
#: 取值语义（读侧必须严格按此实现，否则会破坏"无漂移时逐位一致"的承诺）：
#:   · `None`（或键不存在）→ **完全 no-op**：执行侧一行代码都不该因此改变 ——
#:     这是为了 "零注入大扫描"能与现版本**逐位一致**；
#:   · `{"speed_mps": float, "reason": str, "drift_run": int, "anchor_refused": int,
#:       "distance_to_boundary_m": float|None, "since": float}` → 把线速度**钳到
#:     `speed_mps` 以内**（只降不升；负速度/倒车段按绝对值钳），`reason` 仅供日志。
#:   本键**不**取代 Safety：`P_SAFETY` 仍是唯一的强制停车者，这里只是"提前一脚刹车"。
#:
#: ⚠️ 键名字符串按写域要求定义在本文件（`core/blackboard.py` 不在本次改动范围）；
#: 建议队长收口时把它登记进 `core/blackboard.py` 并加进"共享键表"文档。
P_MOTION_LIMIT = "motion_limit"


class PoseSource(Protocol):
    """`bridge` 只被用到的两个方法（鸭子类型下限，写在这里当契约）。"""

    def pose(self):                      # -> geo.Pose | None
        ...

    def pose_age(self) -> float:
        ...


class LocalizationTask(TaskThread):
    """把原始里程计变成"信念位姿 + 路线进度"，并在航点处吸附纠偏。"""

    def __init__(self, bridge, bb: Blackboard, bus: EventBus, cfg: Config, *,
                 log: Callable[[str], None] = print, group=None,
                 clock: Callable[[], float] = None) -> None:
        from time import monotonic as _monotonic
        super().__init__("Localization", 20.0, bus=bus, blackboard=bb, logger=log)
        self.bridge = bridge
        self.cfg = cfg
        self.group = group
        self.clock = _monotonic if clock is None else clock

        # 参数一次读齐（tick 里不再碰 cfg，避免每拍 20 次键查找 + 校验开销）
        self.pose_timeout_s = float(cfg.get("safety.pose_timeout_s") or 1.0)
        self.arrival_m = float(cfg.get("pursuit.arrival_m") or 0.12)
        self.drift_warn_m = float(cfg.get("pursuit.drift_warn_m") or 0.30)
        self.anchor_enabled = bool(cfg.get("fix.anchor_at_waypoint"))
        #: 漂移判定的"越段闸门"：车沿路线走到离顶点还剩 ≤ 这个距离就进入可判定窗口。
        #: 取 0.18 m ≈ arrival_m(0.12) 的一半再放宽 → 正常行驶（0.15 m/s @20 Hz = 7.5 mm/拍）
        #: 必然经过该窗口，不会"跳过去"；而 1.5 倍 arrival_m 又保证不会提前太久报警。
        #: ★ 该判据纯弧长、与横向偏差无关 —— 否则"漂得越狠越报不出来"。
        self.drift_seg_gate_m = float(cfg.get("pursuit.drift_seg_gate_m") or 0.18)
        #: ★ 卡死自愈的三条判据（`localization.stuck_*`，理由见 `_heal_stuck`）：
        #: · `stuck_ticks`：连续这么多拍 s 前进不足 `stuck_eps_m`；
        #: · `stuck_dist_m`：上报的投影点离车这么远（说明它已经不是车的位置）；
        #: · 位姿在这几拍里移动 > `arrival_m`（"贴住不动"必须配上"车确实在动"）。
        self.stuck_ticks = max(2, int(cfg.get("localization.stuck_ticks") or 5))
        self.stuck_eps_m = float(cfg.get("localization.stuck_eps_m") or 0.01)
        self.stuck_dist_m = float(cfg.get("localization.stuck_dist_m") or 0.20)
        #: ★ 大跳跃可信闸的离线距离阈值（米，2026-10-17 实车 12.8 m 误锁的修复）：
        #: 大跳跃且新投影离车 > 它 ⇒ 拒绝采纳（理由与取值见模块常量 _STUCK_TRUST_DIST_M）。
        #: 现场若要调，改这一行即可（未登记 SCHEMA 是刻意的：本轮写域不含 core/config.py）。
        self.stuck_trust_dist_m = float(_STUCK_TRUST_DIST_M)

        # ---- ★★ 航向完整性守卫（偏航率经验包络；2026-10-19 task-44）----
        #: 总开关。关掉 ⇒ 行为与加本守卫前**逐位一致**（回归对照/带牙实验用）。
        self.yaw_guard_enabled = bool(cfg.get("localization.yaw_guard"))
        #: 经验包络：低速下限 + 随速度的增益（v≥min_speed 时取 max）。
        #: **本车是阿克曼，不能原地旋转**；spin 只是数值下限，不是「原地转能力」。
        self.yaw_diff_gain_dps_per_mps = _cfg_float(
            cfg, "localization.yaw_diff_gain_dps_per_mps",
            _DEFAULT_YAW_DIFF_GAIN_DPS_PER_MPS)
        self.yaw_diff_spin_dps = _cfg_float(
            cfg, "localization.yaw_diff_spin_dps", _DEFAULT_YAW_DIFF_SPIN_DPS)
        self.yaw_jump_deg = _cfg_float(
            cfg, "localization.yaw_jump_deg", _DEFAULT_YAW_JUMP_DEG)
        self.yaw_clear_ticks = max(1, int(_cfg_float(
            cfg, "localization.yaw_clear_ticks", _DEFAULT_YAW_CLEAR_TICKS)))
        self.yaw_imu_check_dps = _cfg_float(
            cfg, "localization.yaw_imu_check_dps", _DEFAULT_YAW_IMU_CHECK_DPS)
        #: 静止期（v < yaw_min_speed_mps）允许的 yaw 率：车没动却被"转"了 —— 只可能是被搬/
        #: 被转或 yaw 链坏了，两种都不许拿来导航（事故静止段实测 169.2°/s）。
        self.yaw_static_dps = _cfg_float(cfg, "localization.yaw_static_dps",
                                         _DEFAULT_YAW_STATIC_DPS)
        self.yaw_min_speed_mps = _cfg_float(cfg, "localization.yaw_min_speed_mps",
                                            _DEFAULT_YAW_MIN_SPEED_MPS)
        self.yaw_bad_ticks = max(1, int(_cfg_float(cfg, "localization.yaw_bad_ticks",
                                                    _DEFAULT_YAW_BAD_TICKS)))
        #: ★位姿连续性闸：帧间位移/Δt 超过它 ⇒ 这不是"车开过去的"（重定位/测试传送），跳过。
        self.yaw_plausible_speed_mps = _cfg_float(
            cfg, "localization.yaw_plausible_speed_mps", _DEFAULT_YAW_PLAUSIBLE_SPEED_MPS)
        #: ★帧间隔上限：超过它角速度就不可信（丢帧/卡顿/pose 陈旧）。
        self.yaw_max_dt_s = _cfg_float(cfg, "localization.yaw_max_dt_s",
                                       _DEFAULT_YAW_MAX_DT_S)
        self.yaw_imu_mismatch_ratio = _cfg_float(cfg, "localization.yaw_imu_mismatch_ratio",
                                                 _DEFAULT_YAW_IMU_MISMATCH_RATIO)
        #: 桥的可选能力（鸭子类型：FakeBridge 没有 imu，判据自动降级为纯位姿判据）。
        self._imu_fn = getattr(bridge, "imu", None)
        self._imu_age_fn = getattr(bridge, "imu_age", None)
        self._manual_fn = getattr(bridge, "manual_override", None)

        self._yaw_prev: tuple | None = None    # (帧时刻, x, y, yaw)
        self._yaw_bad_run = 0                  # 连续超限/跳变/矛盾的**新帧**数
        self._yaw_good_run = 0                 # 闩锁后连续可信的新帧数（自动复位用）
        self._yaw_reason = ""
        self._yaw_last_rate_dps = 0.0
        self._yaw_last_limit_dps = 0.0
        self._yaw_last_dyaw_deg = 0.0
        self.yaw_max_rate_dps = 0.0     # 本轮见过的最大 yaw 率（误报率/峰值评估用）
        self._yaw_fault_published = False
        # 计数（测试/遥测直接断言，不靠解析日志）
        self.yaw_untrusted = False    # 闩锁；条件消失后可自动复位或 reset_yaw_guard()
        self.yaw_guard_trips = 0
        self.yaw_guard_recoveries = 0  # 自动复位次数
        self.yaw_manual_resets = 0     # 人工复位次数
        self.yaw_bad_samples = 0     # 超限的新帧样本数（含尚未达到 N 的）
        self.yaw_guard_samples = 0   # 参与判定的新帧样本数（误报率的分母）
        self.yaw_skipped_frames = 0  # 因不连续/丢帧跳过的帧
        self.yaw_manual_ticks = 0    # 判为"手动旋转/遥控接管"的样本数（只记账，不判故障）
        self.yaw_imu_mismatch = 0    # pose 角速度与 IMU gyro 矛盾的样本数

        # ---- ★实车漂移护栏（2026-10-11 首跑 SAFE_STOP@B1 的证据：13 m 里漂 0.65 m）----
        # 全部走"代码默认值"（config/*.yaml 统一维护；清单已回报队长，待并入）。
        #: 单次吸附允许把信念挪动的**上限**（米）。吸附只在 `arrival_m` 半径内触发，
        #: 所以这个上限是"双保险"：超过它说明不是"到位纠偏"而是"被外力拽走"。
        self.anchor_max_pull_m = float(cfg.get("localization.anchor_max_pull_m")
                                       or self.arrival_m)
        #: "误差不变大"判据的容差（米）：吸附后离线距离允许与之持平/略差这么多，
        #: 超过就拒绝 —— ★ 早期发现#1「吸附在制造误差」的正面堵法。
        self.anchor_reject_eps_m = float(cfg.get("localization.anchor_reject_eps_m") or 0.005)
        #: 连续多少拍"有漂移但没纠回来"→ 请求降速（`P_MOTION_LIMIT`）。
        self.drift_slow_after = max(1, int(cfg.get("localization.drift_slow_after") or 3))
        self.drift_slow_speed_mps = float(cfg.get("localization.drift_slow_speed_mps") or 0.08)
        #: 连续多少拍还没纠回来 → 请求停车（速度 0，仍需 Safety/执行侧裁决）。
        self.drift_stop_after = max(2, int(cfg.get("localization.drift_stop_after") or 8))
        #: 连续多少拍"干净"才撤销限速（避免抖动式反复开关）。
        self.drift_clear_ticks = max(1, int(cfg.get("localization.drift_clear_ticks") or 5))
        #: 信念接近**安全边界**（`pursuit.map_safety_margin_m` 内再多这么多）就开始限速。
        self.boundary_guard_m = float(cfg.get("localization.boundary_guard_m") or 0.10)
        self.boundary_guard_speed_mps = float(cfg.get("localization.boundary_guard_speed_mps")
                                              or 0.08)
        #: 离边界这么近就请求停车（越界后由 Safety 硬停；这里是**提前**拦一道）。
        self.boundary_stop_m = float(cfg.get("localization.boundary_stop_m") or 0.03)
        #: ★横向重锚的单次上限（米）：在**顶点窗口内**发现信念横向残差 > `drift_warn_m` 时，
        #: 把它沿**垂直方向**拉回折线（这一步按构造只会让"离线距离"变小）；
        #: 残差超过这个上限 ⇒ 不硬拉（那已经不是"横向噪声"而是"整段跑偏"），拒绝并请求降速。
        self.reanchor_pull_max_m = float(cfg.get("localization.reanchor_pull_max_m") or 0.35)
        #: ★★横向重锚的**单拍步长上限**（米，2026-10-25 F5）：
        #: 0.35 上限允许一次性把信念拽回 0.3 m 量级 —— 实车 run4 里 0.303 m 的"一步到位"
        #: 制造了 0.313 m 的位姿突跳（> 0.20 m 突跳门限）→ FixTask 清窗 + 漂移护栏停机。
        #: 改为**分拍收敛**：每拍最多拉回这么多，几拍内收敛到折线，单拍位移 < 突跳门限。
        #: 0.12 = 与"到位吸附"同款上限（沿用实车标定口径），可按配置覆盖。
        self.reanchor_step_max_m = float(cfg.get("localization.reanchor_step_max_m") or 0.12)
        #: 收敛完成阈值（米）：剩余横向残差 ≤ 它即视为已拉回折线（默认 0.02 = 2 cm）。
        self.reanchor_done_m = float(cfg.get("localization.reanchor_done_m") or 0.02)
        #: ★★「吸附掩盖尺度误差」闸（2026-10-11 实车标定教训）：
        #: 实车 17.9% 里程虚增时，**吸附每段归零**会让漂移告警一条都不出（实测 13/13 吸附、
        #: 0 告警），而真值早在终点前 69 cm —— "仅靠吸附不产生告警 ≠ 里程健康"。
        #: 本闸把每次吸附的**沿程修正量**除以"距上次吸附走了多远"，得到**隐含尺度误差**；
        #: 超过 `anchor_scale_max`（默认 8%）就**拒绝这次吸附**（不再替里程计掩盖），
        #: 记进台账并参与限速升级 ⇒ 尺度误差从"被掩盖"变成"看得见 + 会降速"。
        #: ⚠️ **不**硬编码任何 ticks/scale 标定值（标定归队长），只做健康度判别。
        self.anchor_scale_max = float(cfg.get("localization.anchor_scale_max") or 0.08)
        #: 隐含尺度误差的滑动窗口长度（取中位数上报，抗单次噪声）
        self.implied_scale_window = max(1, int(cfg.get("localization.implied_scale_window") or 4))

        self._warn = Throttle(float(cfg.get("logging.rate_limit_s") or 1.0))

        # 路线缓存：Route 不可变，用 `is` 判同一性即可
        self._route: Route | None = None
        self._path: Path | None = None
        self._s_hint: float | None = None
        self._last_seg: int = -1
        self._anchor_idx: int = -1          # 上一个"已吸附过"的航点索引（防同一航点反复锚）
        self._anchor_key: tuple | None = None   # 上次发过漂移告警的锚点（按航点，不按时间）
        self._reanchor_idx: int = -1        # 上一处已横向重锚过的顶点索引（防每拍重做）
        self._reanchor_active: bool = False     # ★F5 分拍收敛进行中（残差 > 触发门限后继续拉）
        self._reanchor_active_idx: int = -1     # 正在收敛的顶点索引（换顶点即作废）
        #: ★★ 横向漂移补偿（2026-10-11 实车标定第 3/4 趟：直线 4.55 m 右偏 10/11 cm ≈ 1.4°）
        #: 陀螺只测角速度，"恒定横漂"（车头与真实前进方向固定夹角）它测不到 ⇒
        #: 信念 yaw 永远等于陀螺 yaw，控制器自认为笔直、物理车却在平移侧滑。
        #: 本参数把信念 yaw 额外旋转 `yaw_trim_deg` 度。操作口径:车右漂→负值、左漂→正值、
        #: 0=关闭。符号反了只会把漂移放大,翻过来即可。控制器见信念偏右 → 左打舵 → 横漂被抵消。
        #: 标定法：实测横向偏移(cm) ÷ 行程(m) → 角度(°)，同向取正值。
        self.yaw_trim_deg = float(cfg.get("localization.yaw_trim_deg") or 0.0)
        #: ★尺度误差健康度的滑动窗口与判据（见 `anchor_scale_max` / `_implied_scale`）
        self._implied_scales: list[float] = []
        self._anchor_ds: list[float] = []   # 最近几次吸附的**沿程修正量**（闸③的判别窗口）
        self._last_anchor_s: float | None = None
        self.scale_suspect = False          # 是否出现过"疑似尺度误差"（> anchor_scale_max）
        self.scale_suspect_max = 0.0        # 见过的最大疑似尺度误差（绝对值）
        self._last_odom_ts: float | None = None

        # ---- ★★行进方向裁决（2026-10-18 10:14 烟测：折返压线把 s 锁死在 0.70）----
        #: 位移小于它（米）就认为"车没在动"，方向锁存上次可信值（**不能**退回命令符号：
        #: 命令符号→选段→segment_is_reverse→命令符号 是自指环，见 tracker.py:351 的长注释）。
        self._motion_eps_m = float(cfg.get("pursuit.motion_heading_eps_m") or 0.001)
        #: 锁存的"车正在往哪边走"（地图系弧度）。倒车时它是**车头反向**（由位移天然得到）。
        self._travel_heading: float | None = None
        self._last_motion_xy: tuple[float, float] | None = None
        #: 带牙实验开关：关掉 = 回退到"不传 motion_heading"的旧行为（测试用）。
        self.motion_heading_enabled = True
        self.motion_heading_ticks = 0     # 用位移方向更新的拍数（可观测）
        #: 上一次**被接受**的投影（方向一致性闸拒绝时原样沿用，见 _opposes_travel）。
        self._last_proj = None
        self.direction_rejects = 0        # 因"逆着行进方向"被拒绝的投影次数（可观测）
        #: 路线内容签名（token/长度变化 ⇒ 换载）；identity 之外的第二道判据。
        self._route_sig: tuple | None = None

        # 卡死判定的滑动窗口（起点：s 与位姿。s 一有前进就重新起算）
        self._stuck_s: float | None = None
        self._stuck_n: int = 0
        self._stuck_x: float = 0.0
        self._stuck_y: float = 0.0

        # 遥测/自检计数（测试直接断言这些，不靠解析日志）
        self.ticks = 0
        self.missing_pose_ticks = 0
        self.projected_ticks = 0
        self.anchored = 0
        self.drift_warnings = 0
        self.stuck_scans = 0        # 判定卡死 → 全路线重投影的次数（含"全局在身后"的）
        self.projection_heals = 0   # 其中真正改了上报值（全局投影更靠前）的次数
        self.stuck_heal_refused = 0  # ★ 因"大跳跃且新投影离车太远"被**拒绝**的自愈次数
        # ---- ★漂移护栏的计数与状态----
        self.anchor_refused = 0          # 因"误差会变大/超上限"**拒绝**吸附的次数（显式记账）
        self.drift_run = 0               # 连续"有漂移、未纠回"的拍数
        self.healthy_run = 0             # 连续干净拍数（用于撤销限速）
        self.boundary_alerts = 0         # 进入边界护栏区（提前限速）的次数
        self.reanchored = 0              # ★横向重锚成功次数（"误差变小才施加"的那条路径）
        self.motion_limited_ticks = 0    # 处于限速/停机状态的拍数
        self.motion_limit_published = 0  # 实际下发 `P_MOTION_LIMIT` 的次数（变化才发）
        self._limit_state: tuple | None = None      # 上次下发的限速状态（去重用）
        self._boundary_warned = False               # 边界告警只打一次（退出护栏区复位）

    # ------------------------------------------------------------------ #
    #  生命周期
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        self._log(f"定位：锚定={'ON' if self.anchor_enabled else 'OFF'}"
                  f" 到位半径={self.arrival_m:.3f}m 漂移告警={self.drift_warn_m:.3f}m"
                  f" 前向投影窗口={self.arrival_m:.3f}m 定位超时={self.pose_timeout_s:.2f}s"
                  f" 卡死自愈={self.stuck_ticks}拍/{self.stuck_eps_m:.3f}m/{self.stuck_dist_m:.2f}m")

    def tick(self) -> None:
        self.ticks += 1
        now = self.clock()

        # ---------------- ① 取位姿：拿不到就明确"没有"，绝不沿用旧值 ---------------- #
        pose = self.bridge.pose()
        age = _INF if pose is None else self._float_or_inf(self.bridge.pose_age())
        if pose is None or age == _INF:
            self.missing_pose_ticks += 1
            # ★ P_ODOM_AGE 每拍都要写（Safety 的第①道检查直接读它）
            self.bb.set(P_ODOM_AGE, _INF, source=self.name, ts=now)
            if self.missing_pose_ticks == 1 or self._warn.allow("pose_missing", now):
                self._log(f"定位缺失：pose={'None' if pose is None else '有'} age={age}"
                          f"（已 {self.missing_pose_ticks} 拍不写 P_POSE）")
            return
        self._last_odom_ts = now
        self.bb.set(P_ODOM_AGE, age, source=self.name, ts=now)

        # ---------------- ①b ★航向完整性守卫（先于任何"拿 yaw 去算"的事） ---------------- #
        # 放在 ② 之前：P_FIX / yaw_trim 可能带来离散跳变，守卫必须看**原始帧**，
        # 否则一次修正会被当成"车转了"。（吸附/重锚只改位置、不改 yaw，故无此问题。）
        self._update_yaw_guard(pose, age, now)

        # ---------------- ①c ★行进方向（投影裁决的输入；折返压线时唯一能分段的判据） ---------------- #
        self._update_travel_heading(pose)

        # ---------------- ② 应用 P_FIX 修正 ---------------- #
        fix = self.bb.get(P_FIX)
        if not isinstance(fix, FixState):
            fix = FixState()
        cx, cy, cyaw = fix.corrected(pose.x, pose.y, pose.yaw)
        if self.yaw_trim_deg:
            cyaw = geo.normalize_angle(cyaw + math.radians(self.yaw_trim_deg))
        self.bb.set(P_POSE_RAW, pose, source=self.name, ts=now)

        # ---------------- ③ 路线投影（带 s_hint） ---------------- #
        proj = self._project(cx, cy, cyaw)
        if proj is not None:
            self.projected_ticks += 1
            self.bb.set(P_ROUTE_INDEX, proj.seg_index, source=self.name, ts=now)
            self.bb.set(P_ROUTE_S, proj.s, source=self.name, ts=now)

        # ---------------- ④ 到位吸附 + 漂移判定 ---------------- #
        anchored_pose = self._anchor(cx, cy, cyaw, proj, now)
        unhealthy = False
        if anchored_pose is None and proj is not None and self.anchor_enabled:
            gate = self._warning_gate(proj)
            # ---------------- ④b ★横向重锚：只在"已知顶点窗口内"动手 ----------------
            anchored_pose = self._reanchor(proj, cx, cy, cyaw, gate, now)
            if anchored_pose is None and gate is not None:
                gidx, gwp = gate
                self._report_drift(gwp.block, gidx, gwp.distance_to(cx, cy), now,
                                   anchored=False)
                # ★"有漂移、没纠回来"= 不健康的一拍（驱动限速/停机）
                # ★★「到下一航点远」**不等于**漂移：倒车/共线折返入口处投影会落后 0.6~0.9 m
                #   （P_ROUTE_S 追不上真值弧长，实测见 _tmp_session/t14_proj/P0_reverse_lag.md），
                #   此时连**窗口投影点到车的距离**（proj.dist）也是 0.6~0.9 —— 它衡量的是
                #   "离窗口里那个候选点多远"，不是"离折线多远"，**不能**拿它判别。
                #   唯一可靠的判据是「车是否真的离开了折线」：用**全路线**投影量横向离线
                #   （hint=None + 全窗口；只在顶点窗口且航点距离超限时才扫，不是每拍热点）。
                #   15:04 类真漂移：全路线最近点仍离线 0.62~0.78 m ⇒ 照旧停（§⑮ 带牙）。
                if gwp.distance_to(cx, cy) > self.drift_warn_m:
                    off_path = self._path.project(cx, cy, s_hint=None,
                                                  window_m=self._path.total_length,
                                                  allow_backward=True)
                    # ★升级门限 = drift_warn + 余量：正常切角 0.304~0.308 不升级；
                    #   真漂移 0.40/0.62~0.78 照旧升级（数值依据见 _DRIFT_ESCALATE_MARGIN_M）
                    unhealthy = (float(off_path.dist)
                                 > self.drift_warn_m + _DRIFT_ESCALATE_MARGIN_M)
                else:
                    unhealthy = False

        # ---------------- ⑤ ★信念漂移护栏 ---------------- #
        # 先算信念离安全边界还有多远（`geo.in_bounds/map_bounds` 与 Safety 同源，不重复实现），
        # 再据此 + 吸附健康度下发 `P_MOTION_LIMIT`。★ 读侧在无信号时必须完全 no-op。
        belief = anchored_pose if anchored_pose is not None else geo.Pose(cx, cy, cyaw)
        self._update_drift_guard(belief, unhealthy=unhealthy, now=now)

        # 写 P_POSE 的必须是"最终信念"，且 P_ROUTE_S 要与之一致 →
        # 吸附把点挪了，就重新投影一次（单段内，代价可忽略）
        if anchored_pose is not None:
            self.bb.set(P_POSE, anchored_pose, source=self.name, ts=now)
            proj2 = self._project(anchored_pose.x, anchored_pose.y, anchored_pose.yaw)
            if proj2 is not None:
                self.bb.set(P_ROUTE_INDEX, proj2.seg_index, source=self.name, ts=now)
                self.bb.set(P_ROUTE_S, proj2.s, source=self.name, ts=now)
        else:
            self.bb.set(P_POSE, geo.Pose(cx, cy, cyaw), source=self.name, ts=now)

    def teardown(self) -> None:
        self._log(f"定位退出：{self.ticks} 拍 / 缺位姿 {self.missing_pose_ticks} 拍 / "
                  f"吸附 {self.anchored} 次 / 漂移告警 {self.drift_warnings} 次 / "
                  f"投影卡死自愈 {self.projection_heals} 次（判定 {self.stuck_scans} 次 / "
                  f"拒绝 {self.stuck_heal_refused} 次）")
        # 漂移护栏台账（"吸附失败要显式记账"的收尾口径）
        self._log(f"漂移护栏：吸附被拒 {self.anchor_refused} 次 / 限速或停机 "
                  f"{self.motion_limited_ticks} 拍（下发 {self.motion_limit_published} 次）/ "
                  f"边界护栏 {self.boundary_alerts} 拍 / 最终连续不健康 {self.drift_run} 拍")
        self._log(f"航向守卫：{'❌已判失可信' if self.yaw_untrusted else '✅未触发'}"
                  f"（{self._yaw_reason or '-'}）/{self.yaw_guard_samples} 个新帧样本中 "
                  f"{self.yaw_bad_samples} 个超限 / 跳过 {self.yaw_skipped_frames} 帧 / "
                  f"手动旋转 {self.yaw_manual_ticks} 拍 / IMU 矛盾 {self.yaw_imu_mismatch} 次")

    # ------------------------------------------------------------------ #
    #  内部
    # ------------------------------------------------------------------ #
    @staticmethod
    def _float_or_inf(value) -> float:
        try:
            out = float(value)
        except (TypeError, ValueError):
            return _INF
        return _INF if math.isinf(out) or math.isnan(out) else out

    def _project(self, x: float, y: float, yaw: float | None = None):
        """用当前 s_hint 投影（无路线/路线太短 → None）。

        ★ 调用形状是**试出来的**，不是随手填的：`Path.project` 在 `allow_backward=True` 时把
          窗口下界 `s_hint - window_m` 折算成 `t_floor` 并夹住本段的 t。于是当 `window_m`
          大于"本拍到 hint 的距离"时（例如 window_m=0.45 m、车每拍走 0.05 m），
          **投影点每拍都被夹回 s_hint → s 永远停住**（实测：整条 0.35 m 路线走完，s 恒为 0）。

          正确形状 = `allow_backward=False` + `window_m = arrival_m`：
          · 前向窗口 [s_hint, s_hint + 0.12] —— 足以覆盖 20 Hz × 0.6 m/s 的正常位移
            （0.03 m/拍），并在噪声下仍不后退；
          · `allow_backward=False` 保证 `s` 单调（P1 §3.1 验收："P_ROUTE_S 单调"）。
          代价：倒车段（`R:`）投影只能停在最小 s 上 —— 倒车段要回退时改传 True，
          但那属于 Tracker/Mission 的用法，本任务只报"信念沿程位置"。
          ★★ 这套形状本身有个**吸收态**：车的垂足落到窗口之前时，返回的 s 恰好等于
          hint ⇒ 不动点。由 `_heal_stuck()` 兜底（判据与实证见那里），
          所以本函数的调用方不需要额外的看门狗。
        """
        route = self.bb.get(P_ROUTE)
        if not isinstance(route, Route) or len(route.waypoints) < 2:
            return None
        # ★换载重置：identity 之外再比「内容签名」（航点数 + 首末格）。Route 是 frozen 的，
        #   identity 相同通常内容也相同；这道签名是给「换了新对象但内容/长度变了」的防御，
        #   以及给中途外部命令换线一个明确复位点。**绝不能沿用上一趟的 s_hint**
        #   （2026-10-18 烟测：上一趟终点 s=0.70 被带进新趟 ⇒ 共线折返上锁死）。
        sig = (len(route.waypoints), route.waypoints[0].block, route.waypoints[-1].block)
        if self._route is not route or sig != self._route_sig:
            self._route = route
            self._route_sig = sig
            self._path = Path(route)
            self._s_hint = None
            self._last_seg = -1
            self._anchor_idx = -1
            self._reanchor_idx = -1
            self._anchor_key = None
            self._clear_stuck()
            # 换载后方向必须重新建立（上一趟的方向/位移不能带到新路线）
            self._travel_heading = None
            self._last_motion_xy = None
            self._log(f"路线已装载：{len(route.waypoints)} 点 / {self._path.total_length:.2f} m")
        assert self._path is not None
        # ★★必须带 motion_heading：路线折返压在同一条线上时（N10→M10→R:N10 就是），
        #   同一物理点到两条段完全等距，位置裁决不了；s_hint 一旦坏掉就再也回不来。
        #   方向取「实际位移」（倒车时天然是车头反向），见 _update_travel_heading。
        # 方向：优先实测位移；换载后首拍还没有位移 ⇒ 用调用方的信念 yaw（车头方向）。
        heading = self._travel_heading if self.motion_heading_enabled else None
        if self.motion_heading_enabled and heading is None and yaw is not None:
            heading = float(yaw)
        proj = self._path.project(x, y, s_hint=self._s_hint,
                                  window_m=self.arrival_m, allow_backward=False,
                                  motion_heading=heading)
        proj = self._heal_stuck(proj, x, y)
        # ---- ★★方向一致性闸：投影不许落在「逆着行进方向的共线支」上 ----
        # 现场会怎么错：车在 N10→M10 去程上离顶点 6.4 cm 时触发「到位吸附 M10」，信念被锚到
        # 顶点 ⇒ s_hint 被推到 0.35；下一拍原始位姿还在顶点**之前**，而前向窗口 [0.35, 0.47]
        # 里只有回程段（共线、方向相反），投影返回回程段 0.414（离车 0）—— 位置更近、方向反了。
        # 之后 _warning_gate 的下一个航点变成 N10，离车 0.31 m ⇒ 假漂移 → drift_stop。
        # 判据：投影段几何方向与实际行进方向夹角 > 150°（折返压线就是 180°；90°/135° 的
        # 正常直角/钝角弯不受影响）⇒ 拒绝本次推进，沿用上一拍被接受的投影（s 不往前跳）。
        if self._opposes_travel(proj) and self._last_proj is not None:
            self.direction_rejects += 1
            if self._warn.allow("proj_dir_reject", self.clock()):
                self._log(f"⛔ 投影方向不一致：段 {proj.seg_index} s={proj.s:.3f} 与行进方向"
                          f" {geo.deg(self._travel_heading):.0f}° 相反（共线折返的另一支）"
                          f" → 保留上一拍 s={self._last_proj.s:.3f}"
                          f"（累计 {self.direction_rejects} 次）")
            return self._last_proj
        self._s_hint = proj.s
        self._last_seg = proj.seg_index
        self._last_proj = proj
        return proj

    # ------------------------------------------------------------------ #
    #  ★ 投影卡死自愈（本文件 2026-10-07 新增的核心修复）
    # ------------------------------------------------------------------ #
    def _clear_stuck(self) -> None:
        self._stuck_s = None
        self._stuck_n = 0
        self._stuck_x = 0.0
        self._stuck_y = 0.0

    def _heal_stuck(self, proj, x: float, y: float):
        """投影"贴住不动"但车确实在动 → 做**一次**全路线重投影，把 hint 拉回真值。

        ## 为什么必须修（这是带 `s_hint` 的投影链的结构性吸收态，不是调参问题）

        `Path.project(allow_backward=False)` 把窗口下界夹在 `s_hint` 上：
        `lo = hint`、`t_floor = (lo - s_start[i_first]) / seg_len`，于是**首段**的 t 被夹到
        `t_floor`（= 弧长 `hint` 处那个点）。当车的垂足落在窗口**之前**时，
        窗口内最近点就永远是那个下界点 ⇒ `proj.s == hint`（逐位相等）⇒ 下一拍 hint 不变
        ⇒ **不动点，永不自行恢复** —— 因为本任务每拍都拿自己的上一拍当 hint。

        bench 实证（`_tmp_session/bench_*.txt` 的 `[T]` 行 + `Path` 全局投影对拍）：
        上报投影点离线 >0.5 m 占 20~37%、最大 3.53~3.67 m；冻结值集中在顶点吸引子
        `(12,4.41) (15,6.78) (19/20,9.10) (14,4.9) (10,3.73) (17/18,8.05)`；
        用本任务的调用形状（`window_m=0.12, allow_backward=False`）离线复算逐值命中。

        ## 判据（三条同时成立；每条都必要，见各自的理由）

        1. `proj.s` 相对窗口起点前进 < `stuck_eps_m` 且已持续 ≥ `stuck_ticks` 拍
           —— "贴住不动"。取 0.01 m：最低速 0.08 m/s @20 Hz 五拍也前进 20 mm，
           所以正常行驶（哪怕蠕动）**不可能**被误判；
        2. 位姿在同样这几拍里移动 > `arrival_m` —— "车确实在动"。没有它，停车等靶时
           的横向偏差会被当成卡死（那种情形 s 本来就该不动）；
        3. `proj.dist > stuck_dist_m` —— 上报的投影点已经**不是车的位置**了。取 0.20 m
           （与仿真实测的跟踪 RMS 同量级）：便宜地挡掉"投影贴住但确实就在车脚下"的
           情形（例如刚越过末点时 dist 会涨，那种 s 本来就该停住）。

        ## 方向：不做"只向前"限制（及为什么仍不破坏单调语义）

        全路线重投影给出的 `s` 可能**在冻结值身后**（车真的倒回去了/被推回了）。
        ⚠️ 实测结论（2026-10-07，合成路线 + bench 位姿）：**不能**限制方向 —— 车"切角
        之后往回退一点"时真值本来就在身后（本例 0.550 → 0.402），限制方向等于把错误值
        留在原地。所以改为靠判据 3 兜底：`proj.dist > stuck_dist_m` 保证"车已经不在
        路线上"，此时"贴着路线的单调"没有意义，报真值更重要；而**沿路线正常倒车**
        （dist ≈ 0 < 0.20）根本不会进自愈分支，`allow_backward=False` 的单调语义
        在正常情形下一字未改。

        ## 代价

        只在判定卡死时做一次全扫（33 段 ≈104 µs，正常每拍仍是窗口投影 ≈2.5 µs）；
        "判定卡死但全扫结果与窗口一致"时也重新起算窗口，所以最多每 `stuck_ticks` 拍
        一次全扫，不会退化成"每拍全路线扫描"。
        """
        if self._path is None or not self._stuck_check(proj, x, y):
            return proj
        self.stuck_scans += 1
        # ★★全路线搜索的裁决顺序（2026-10-18 烟测修）：
        #   ① 有**行进方向**时，先按方向裁决、**不传 s_hint** —— 因为「卡死」的定义就是
        #      s_hint 已经错了，此时「离 hint 最近」恰好会选回错误的共线支（实测：
        #      s_hint=0.70 时全扫按 hint 选到回程段 0.483，而真值是去程段 0.217）。
        #      方向能唯一区分前一趟/回程趟（两者等距），这是位置+连续性都做不到的。
        #   ② 没有方向（车没动过/方向未知）时才退回旧的「保留 hint 连续性」，
        #      即「仍歧义时按与上一拍 s 连续裁决」——不得跳到另一条共线段。
        #   ③ 两条分支都保留既有的大跳跃可信闸（下面），不会因方向误判就把 s 瞬移。
        heading = self._travel_heading if self.motion_heading_enabled else None
        if heading is not None:
            glob = self._path.project(x, y, s_hint=None,
                                      window_m=self._path.total_length,
                                      allow_backward=True, motion_heading=heading)
        else:
            glob = self._path.project(x, y, s_hint=self._s_hint,
                                      window_m=self._path.total_length, allow_backward=True)
        jump = float(glob.s) - float(proj.s)
        # ---- ★大跳跃可信闸（2026-10-17 实车 +12.82 m 误锁的修复）----
        # 只判"新投影更近"是不够的：车跑到两条路线腿**之间**时，两条腿都离车 0.6~0.8 m，
        # "稍近一点"的那条完全可能是 10 m 之外的终点段（实车实证：离线 0.68 m 的段 40
        # 比离线 0.78 m 的当前段"近 0.10 m"⇒ 上报 s 从 1.29 跳到 14.11）。跳跃小（≤1.4 格）
        # 视为正常追回；跳跃大就必须"新投影确实贴在车上"才可信，否则保留窗口投影。
        if abs(jump) > _EPS_S:
            if abs(jump) > _STUCK_MAX_JUMP_M and float(glob.dist) > self.stuck_trust_dist_m:
                self.stuck_heal_refused += 1
                if self._warn.allow("proj_heal_reject", self.clock()):
                    self._log(f"⛔ 投影自愈被拒：全路线重投影会跳 {jump:+.3f} m"
                              f"（第 {glob.seg_index} 段，离线 {glob.dist:.2f} m > 可信 "
                              f"{self.stuck_trust_dist_m:.2f} m）→ 车已不在任何一条路线腿上，"
                              f"保留窗口投影 {proj.s:.3f}（累计拒绝 {self.stuck_heal_refused} 次）")
                self._stuck_s, self._stuck_x, self._stuck_y, self._stuck_n = proj.s, x, y, 0
                return proj
            self.projection_heals += 1
            self._stuck_s, self._stuck_x, self._stuck_y, self._stuck_n = glob.s, x, y, 0
            if self._warn.allow("proj_heal", self.clock()):
                self._log(f"⚠️ 投影卡死自愈：s 冻在 {proj.s:.3f}（第 {proj.seg_index} 段，"
                          f"离线 {proj.dist:.2f} m），车已在动 → 全路线重投影得 "
                          f"{glob.s:.3f}（第 {glob.seg_index} 段，离线 {glob.dist:.2f} m，"
                          f"{glob.s - proj.s:+.3f} m，累计自愈 {self.projection_heals} 次）")
            return glob
        # 全扫结果与窗口一致 ⇒ 冻住的值就是真最近点（车在等/在原地转）：只重新起算窗口
        self._stuck_s, self._stuck_x, self._stuck_y, self._stuck_n = proj.s, x, y, 0
        return proj

    def _stuck_check(self, proj, x: float, y: float) -> bool:
        """维护"贴住不动"的滑动窗口；返回 True 表示三条判据都成立。"""
        if proj.passed_end:                 # 已跑过末点：s 本来就该停在总长上
            self._stuck_s, self._stuck_x, self._stuck_y, self._stuck_n = proj.s, x, y, 0
            return False
        if self._stuck_s is None:           # 窗口起点（换路线/首拍/刚自愈过）
            self._stuck_s, self._stuck_x, self._stuck_y, self._stuck_n = proj.s, x, y, 0
            return False
        if proj.s - self._stuck_s >= self.stuck_eps_m:   # 有前进 → 重新起算
            self._stuck_s, self._stuck_x, self._stuck_y, self._stuck_n = proj.s, x, y, 0
            return False
        self._stuck_n += 1
        if self._stuck_n < self.stuck_ticks:
            return False
        moved = math.hypot(x - self._stuck_x, y - self._stuck_y)
        return moved > self.arrival_m and proj.dist > self.stuck_dist_m

    # ------------------------------------------------------------------ #
    #  ★★ 航向完整性守卫（偏航率经验包络；2026-10-17 K9 事故，2026-10-19 task-44）
    # ------------------------------------------------------------------ #
    def _update_yaw_guard(self, pose, age: float, now: float) -> None:
        """用**相邻两帧位姿**估 yaw 率，按偏航率经验包络判航向是否可信。

        ## 判据（三条闸 + 可恢复闩锁）

        1. **帧间隔可信**：`dt = 帧时刻差`，必须 `1e-6 < dt ≤ yaw_max_dt_s`。
           同一帧被两拍重复读（20 Hz 任务 vs 50 Hz 里程计）时 dt=0 ⇒ 不是新样本，跳过
           （**不清**连续计数）；丢帧/卡顿（dt > 0.6 s）角速度算不准 ⇒ 跳过并清零。
        2. **位姿连续**：帧间位移/Δt ≤ `yaw_plausible_speed_mps`。超过它说明这一帧不是
           "车开过去的"（重定位、测试传送、坐标系切换），角速度没有物理意义 ⇒ 跳过。
        3. **偏航率经验包络**（不用 Ackermann 名义值 `v·tan(δmax)/L`）：
           · 运动（v ≥ yaw_min_speed_mps）：
             `ω_max = max(yaw_diff_spin_dps, yaw_diff_gain_dps_per_mps × v)`
             —— 60°/s 只是**数值下限**（低速噪声地板），不是「车能原地转」；
             随速度线性放大（A2 实测 79.8°/s @ 0.19 m/s ⇒ ≈420°/s per m/s，取 525 含 25% 余量）。
           · ⚠️ 为何不用阿克曼名义值：名义 210.7°/s per m/s 低于实测 ≈420，说明实车 yaw 率
             含滑移/有效轴距差异；按名义收紧会把真实运动判成「物理不可能」⇒ 假停车。
             本车**底盘**是阿克曼（不能原地转向），但**判据阈值**是实测标定的经验值，两者不矛盾。
           · 静止（v < yaw_min_speed_mps）：`ω_max = yaw_static_dps`（45°/s）。
             车没动却被"转"了 —— 只可能是被搬/被转或 yaw 链坏了。
        4. **单帧跳变**：`|Δyaw| > yaw_jump_deg`（30°）⇒ 立即判故障（重定位/坐标跳变）。
        5. **IMU 交叉核对**：位姿 yaw 率 > `yaw_imu_check_dps` 且 IMU |gz| < ratio×位姿率
           ⇒ 判"链内部矛盾"（与包络值无关，IMU 说没转就是不可信）。
        6. **持续 N 帧**：超限/矛盾连续 `yaw_bad_ticks` 个新帧才判故障；单帧毛刺不刹。

        ## 闩锁与恢复（2026-10-19 改）

        判故障后 `yaw_untrusted=True`，经 `P_MOTION_LIMIT{speed_mps:0}` + `EV_TASK_FAULT`
        请求停车（Safety 仍锁存 SAFE_STOP，需显式 `clear_safety()`）。
        **不再永久锁死**：连续 `yaw_clear_ticks`（默认 20 个新帧）全部可信 ⇒ 自动复位
        `yaw_untrusted=False` 并把 P_MOTION_LIMIT 撤为 None；也可调用
        `reset_yaw_guard()` 人工复位。复位只清"航向守卫"，Safety 的清场仍按原规程人工执行。

        ## 为什么用帧间隔而不是拍间隔

        任务 20 Hz、里程计 50 Hz、serial_reader 打印 5 Hz（事故日志实测 dt 中位 0.2 s）。
        用拍间隔（0.05 s）算 0.2 s 里 24° 的跳变会得到 **480°/s 的 4 倍假读数**；
        用帧时刻差才是物理角速度。
        """
        if not self.yaw_guard_enabled:
            return
        frame_t = now - age
        prev = self._yaw_prev
        self._yaw_prev = (frame_t, float(pose.x), float(pose.y), float(pose.yaw))
        if prev is None:
            return                          # 首帧：没有参照，不算角速度
        dt = frame_t - prev[0]
        if dt <= 1e-6:
            return                          # 同一帧重复读：不是新样本，也不清连续计数
        if dt > self.yaw_max_dt_s:
            self._yaw_bad_run = 0
            self.yaw_skipped_frames += 1
            return
        v = math.hypot(pose.x - prev[1], pose.y - prev[2]) / dt
        if v > self.yaw_plausible_speed_mps:
            self._yaw_bad_run = 0
            self.yaw_skipped_frames += 1
            return
        dyaw = geo.ang_diff(pose.yaw, prev[3])
        rate = abs(geo.deg(dyaw)) / dt
        limit = self._yaw_limit_dps(v)
        self._yaw_last_rate_dps = rate
        self._yaw_last_limit_dps = limit
        self._yaw_last_dyaw_deg = geo.deg(dyaw)
        if rate > self.yaw_max_rate_dps:
            self.yaw_max_rate_dps = rate
        self.yaw_guard_samples += 1
        mismatch = self._yaw_imu_mismatch(rate)
        jump = abs(geo.deg(dyaw)) > self.yaw_jump_deg
        # ---- 手动旋转 / 遥控接管：人比机器优先（但 IMU 矛盾不豁免）----
        if self._manual_override() and not mismatch:
            self.yaw_manual_ticks += 1
            self._yaw_bad_run = 0
            self._yaw_good_run = 0
            if self._warn.allow("yaw_manual", now):
                self._log(f"航向守卫：遥控接管中 yaw {rate:.1f}°/s / 单帧 {geo.deg(dyaw):+.1f}°"
                          f"（上限 {limit:.1f}°/s，{'静止' if v < self.yaw_min_speed_mps else '运动'}，"
                          f"v={v:.2f} m/s）→ 按人工操作处理，不判故障"
                          f"（累计 {self.yaw_manual_ticks} 拍）")
            return
        bad = jump or rate > limit or mismatch
        if not bad:
            self._yaw_bad_run = 0
            if self.yaw_untrusted:
                self._yaw_good_run += 1
                if self._yaw_good_run >= self.yaw_clear_ticks:
                    self._clear_yaw_untrusted(now)
            return
        self.yaw_bad_samples += 1
        self._yaw_good_run = 0
        if jump:
            # 单帧跳变：不等 N 帧，立即判故障（重定位/坐标跳变不会连续 3 帧才出现）
            self._yaw_bad_run = self.yaw_bad_ticks
            self._trip_yaw_untrusted(v, rate, limit, now,
                                     jump_deg=abs(geo.deg(dyaw)), mismatch=mismatch)
            return
        self._yaw_bad_run += 1
        if self._yaw_bad_run < self.yaw_bad_ticks:
            return
        self._trip_yaw_untrusted(v, rate, limit, now, mismatch=mismatch)

    def _yaw_limit_dps(self, v: float) -> float:
        """当前速度下允许的 yaw 率上限（°/s）；经验包络，理由见 `_update_yaw_guard`。"""
        if v < self.yaw_min_speed_mps:
            return self.yaw_static_dps
        return max(self.yaw_diff_spin_dps,
                   self.yaw_diff_gain_dps_per_mps * v)

    def _yaw_imu_mismatch(self, rate: float) -> bool:
        """位姿 yaw 率与 IMU gyro 是否矛盾；无 IMU / 低于核对阈值 → False（自动降级）。"""
        if self._imu_fn is None or rate < self.yaw_imu_check_dps:
            return False
        imu_rate = self._imu_yaw_rate_dps()
        if imu_rate is None:
            return False
        return imu_rate < rate * self.yaw_imu_mismatch_ratio

    def _trip_yaw_untrusted(self, v: float, rate: float, limit: float, now: float,
                            *, jump_deg: float | None = None,
                            mismatch: bool = False) -> None:
        """判「航向失去可信」：走既有 `P_MOTION_LIMIT` + `EV_TASK_FAULT` 两条通路。"""
        if self.yaw_untrusted:
            return                          # 已闩锁：不重复计数/不重复发事件
        self.yaw_untrusted = True
        self.yaw_guard_trips += 1
        moving = v >= self.yaw_min_speed_mps
        imu_rate = self._imu_yaw_rate_dps()
        if mismatch:
            self.yaw_imu_mismatch += 1
        if jump_deg is not None:
            reason = "yaw_jump"
            text = (f"单帧 yaw 跳变 {jump_deg:.1f}° > 阈值 {self.yaw_jump_deg:.1f}°"
                    f"（帧间隔角速度 {rate:.1f}°/s，v={v:.2f} m/s）"
                    f" = 重定位/坐标跳变，航向不可信")
        elif not moving:
            reason = "yaw_static_rotation"
            text = (f"静止期 yaw {rate:.1f}°/s > 上限 {limit:.1f}°/s（v={v:.3f} m/s）："
                    f"车没在动、yaw 却被转了 = 被搬/被转或 yaw 链坏了")
        elif mismatch and imu_rate is not None:
            reason = "yaw_pose_imu_mismatch"
            text = (f"yaw 率与 IMU 矛盾：位姿 {rate:.1f}°/s > 包络 {limit:.1f}°/s，"
                    f"但 IMU |gz|={imu_rate:.1f}°/s（v={v:.2f} m/s）⇒ 位姿/航向链内部不一致")
        else:
            reason = "yaw_rate_over_diff"
            text = (f"yaw {rate:.1f}°/s > 包络上限 {limit:.1f}°/s（v={v:.2f} m/s，"
                    f"增益 {self.yaw_diff_gain_dps_per_mps:.0f}°/s per m/s，"
                    f"低速允许 {self.yaw_diff_spin_dps:.0f}°/s）")
        self._yaw_reason = reason
        if self.yaw_guard_trips == 1:
            self._log(f"🧭❌ 航向失去可信（{reason}）：{text} → 请求停车 + 发 EV_TASK_FAULT"
                      f"（连续 {self._yaw_bad_run} 个新帧；连续 {self.yaw_clear_ticks} 帧可信后自动复位）")
        if not self._yaw_fault_published:
            self._yaw_fault_published = True
            # ★ 复用既有 fault 事件通路：SafetyTask 检查④ 据此锁存 SAFE_STOP 并逐拍刹车。
            self.bus.publish(EV_TASK_FAULT, source=self.name, task=self.name,
                             where="heading_untrusted", error=text)

    def _clear_yaw_untrusted(self, now: float) -> None:
        """条件消失后自动复位守卫（Safety 的 SAFE_STOP 仍需显式 clear_safety()）。"""
        if not self.yaw_untrusted:
            return
        self.yaw_untrusted = False
        self.yaw_guard_recoveries += 1
        self._yaw_bad_run = 0
        self._yaw_good_run = 0
        self._yaw_fault_published = False
        self._log(f"✅ 航向守卫自动恢复：连续 {self.yaw_clear_ticks} 个新帧可信"
                  f"（累计恢复 {self.yaw_guard_recoveries} 次）→ 撤销限速请求；"
                  f"Safety 若已锁存，需显式 clear_safety()")

    def reset_yaw_guard(self, now: float | None = None) -> None:
        """人工复位航向守卫（条件消失但不想等自动复位时用；不替代 Safety 清场）。"""
        ts = self.clock() if now is None else now
        was = self.yaw_untrusted
        self.yaw_untrusted = False
        self._yaw_bad_run = 0
        self._yaw_good_run = 0
        self._yaw_fault_published = False
        self._yaw_reason = ""
        self.yaw_manual_resets += 1
        if was:
            self.bb.set(P_MOTION_LIMIT, None, source=self.name, ts=ts)
        self._log(f"航向守卫：人工复位（第 {self.yaw_manual_resets} 次；原状态"
                  f"{'闩锁' if was else '未闩锁'}）→ Safety 仍需显式 clear_safety()")

    def _imu_yaw_rate_dps(self) -> float | None:
        """最近一帧 IMU 的 |gyro z|（°/s）；无 IMU/过期/形状不对 → None（判据自动降级）。"""
        if self._imu_fn is None:
            return None
        try:
            imu = self._imu_fn()
        except Exception:                    # noqa: BLE001（桥接口异常不能拖垮定位线程）
            return None
        if not imu:
            return None
        if self._imu_age_fn is not None:
            try:
                if float(self._imu_age_fn()) > 0.25:
                    return None
            except Exception:                # noqa: BLE001
                return None
        try:
            return abs(geo.deg(float(imu[2])))
        except (TypeError, ValueError, IndexError):
            return None

    def _manual_override(self) -> bool:
        """遥控接管？（鸭子类型：桥没有该方法 ⇒ 视为未接管）"""
        if self._manual_fn is None:
            return False
        try:
            return bool(self._manual_fn())
        except Exception:                    # noqa: BLE001
            return False

    # ------------------------------------------------------------------ #
    #  ★★ 行进方向裁决（共线折返时投影唯一能用的分段判据）
    # ------------------------------------------------------------------ #
    def _update_travel_heading(self, pose) -> None:
        """用**实际位移**估计「车正在往哪边走」（地图系弧度），位移过小时锁存上次值。

        ## 为什么必须做（2026-10-18 10:14 实车烟测，路线 N10 M10 R:N10）

        折返段压在同一条直线上：同一个物理点到「去程段」和「回程段」**完全等距**，
        位置信息裁决不了它属于哪一段。Localization 启动时 s_hint=None，第一次全路线
        投影在 N10（= 终点格，s=0.70）上按「无提示取更靠后的段」选中了**回程段末点**
        ⇒ P_ROUTE_S=0.700 并当场触发「到位吸附 N10（航点 #2）」；之后每拍用
        allow_backward=False 的窗口 [0.70, 0.82] 投影，车怎么走都只能返回 0.700，
        dist 从 0.11 涨到 0.32 ⇒ 假漂移 0.301 → drift 限速 → drift_stop 急停。

        唯一能区分两条共线段的判据是「车正在往哪边走」：往前一趟 ⇒ 命中去程段，
        倒回来 ⇒ 命中回程段。方向取**实测位移**：
        · 倒车时位移方向天然是车头反向，不需要知道 segment_is_reverse，
          也就不会形成「命令符号 → 选段 → is_rev → 命令符号」的自指环
          （tasks/tracker.py 的同款长注释有完整血证）；
        · 位移小于 pursuit.motion_heading_eps_m（1 mm）时方向是噪声 ⇒ 锁存上次可信值；
        · 从未动过（首拍/换载后首拍）⇒ 退化为车头 yaw（自动任务总是从路线起点对准
          首段出发；一旦车动 ≥1 mm，位移方向立刻纠正它）。

        ⚠️ 已知边界：yaw 兜底假设「首段是正向」。若路线首段本身是倒车段（R: 开头）
        且车头已对准倒车方向，首拍方向会反 —— 本卡的烟测路线首段是正向，未覆盖；
        车一动（≥1 mm）即自我纠正。
        """
        dist = 0.0
        dx = dy = 0.0
        if self._last_motion_xy is not None:
            dx = float(pose.x) - self._last_motion_xy[0]
            dy = float(pose.y) - self._last_motion_xy[1]
            dist = math.hypot(dx, dy)
        self._last_motion_xy = (float(pose.x), float(pose.y))
        # 位姿不连续（重定位/测试传送/换载）：方向不可信，不更新。
        # 门限 0.10 m/拍：本车最快 0.35 m/s，20 Hz = 1.8 cm、5 Hz 回放 = 7 cm，都在门限内；
        # 超过 0.10 m 的"单拍位移"不可能是开出来的（测试里常见的 set_pose 传送就会命中）。
        if dist >= 0.10:
            return
        if dist >= self._motion_eps_m:
            self._travel_heading = math.atan2(dy, dx)   # 倒车时天然是车头反向
            self.motion_heading_ticks += 1
        elif self._travel_heading is None:
            self._travel_heading = float(pose.yaw)

    def _opposes_travel(self, proj) -> bool:
        """投影点所在段的几何方向是否与「实际行进方向」相反（折返压线误锁的判据）。

        为什么需要（2026-10-18 烟测）：折返段与去程段完全共线时，顶点附近的同一个物理点
        同时属于两条段；窗口一旦把搜索范围推到顶点之后（例如「到位吸附」把信念锚到顶点、
        下一拍原始位姿还在顶点之前），最近点会落在**回程段**上 —— 距离比去程段的顶点候选
        更近（0 vs 6.4 cm），纯位置/连续性都裁决不了，只有方向能。
        ⚠️ 阈值取 150°（不是 90°）：90° 直角弯、135° PIVOT 换向都属正常，不能误伤；
        共线折返是 180°，留 30° 给噪声与「先原地转再走」的过渡拍。
        """
        if not self.motion_heading_enabled or self._travel_heading is None:
            return False
        if self._path is None:
            return False
        i = int(proj.seg_index)
        segs = self._path.segments
        if i < 0 or i >= len(segs):
            return False
        return abs(geo.ang_diff(segs[i][5], self._travel_heading)) > math.radians(150.0)

    def _anchor(self, x: float, y: float, yaw: float, proj, now: float):
        """到位吸附。返回锚定后的 Pose；未触发返回 None。

        两件事分开判，不要混：
        · **吸附**只有一种触发条件 —— 车离目标航点 < `arrival_m`（"到位"就是到位）；
          ★ 唯一例外是**末点**：吸附半径收紧到 `_GOAL_STOP_M`(0.03) —— 末点按 0.12 m
           吸附会把信念在 0.12 m 外拽到格心，Mission 据此提前 FINISH、物理车停在格心前
           ~11 cm（2026 直线标定 N10→A10 实测欠程，见模块常量 `_GOAL_STOP_M` 注释）。
        · **漂移判定**在 `_warning_gate()` 处独立进行 —— 否则就有个结构性死角：
          `arrival_m`(0.12) 远小于 `drift_warn_m`(0.30)，横向漂了 0.5 m 的车
          永远进不了吸附半径 ⇒ 既不纠偏、也永远报不出漂移（漂移这件事"报不出来"）。

        ★★ 实测（2026-10-11 实车 SAFE_STOP@B1，13 m 漂 0.65 m）新增**施加前的两道闸**，
        针对早期发现#1「吸附在制造误差（只有让误差变小时才该施加）」：
          ① 上限闸：`drift > anchor_max_pull_m` ⇒ 拒绝（这不是"到位纠偏"，是被拽走）；
          ② 误差闸：把信念挪到格心后，**离折线的距离不能变大**（`_error_shrinks`）⇒
             否则拒绝。判据用"离线距离"而不是"离格心距离"：格心只是格的几何中心，
             真正代表"车应在哪"的是折线本身；吸附若把信念推离折线，就是在制造误差。
        两条闸都**显式记账**（`anchor_refused` + 事件 + 日志），绝不静默。
        """
        if proj is None or self._path is None or not self.anchor_enabled:
            return None
        candidates = self._anchor_candidates(proj, x, y)
        target_idx, target = -1, None
        last_idx = len(self._path.waypoints) - 1
        for idx, wp in candidates:
            # ★ 末点吸附半径收紧到 `_GOAL_STOP_M`(0.03)：见模块常量注释 —— 非末点照旧。
            radius = _GOAL_STOP_M if idx == last_idx else self.arrival_m
            if wp.distance_to(x, y) < radius:
                target_idx, target = idx, wp
                break
        if target is None:
            self._reset_anchor_latch(x, y, proj, now)
            return None
        if target_idx == self._anchor_idx:  # 同一航点只锚一次，不反复重锚（防"越锚越偏"）
            return geo.Pose(target.x, target.y, yaw)

        drift = math.hypot(x - target.x, y - target.y)
        # ---- ★闸①：位移上限 ----
        if drift > self.anchor_max_pull_m + 1e-9:
            self._refuse_anchor(target.block, target_idx, drift, now,
                                f"位移 {drift:.3f} m > 上限 {self.anchor_max_pull_m:.3f} m")
            self._anchor_idx = target_idx        # 本航点不再反复尝试（与施加后的闩锁一致）
            return None
        # ---- ★闸②：吸附后离线距离不得变大 ----
        if not self._error_shrinks(target, proj):
            self._refuse_anchor(target.block, target_idx, drift, now,
                                "吸附后离折线的距离会变大（制造误差）")
            self._anchor_idx = target_idx
            return None
        # ---- ★闸③：不得替里程计掩盖**尺度误差**（实车 17.9% 虚增的教训）----
        susp = self._scale_suspected(target, proj)
        if susp is not None:
            implied, why = susp
            self.scale_suspect = True
            self.scale_suspect_max = max(self.scale_suspect_max, abs(implied))
            self._refuse_anchor(target.block, target_idx, drift, now,
                                f"疑似尺度误差（{why}）—— 不再用吸附把它掩盖掉")
            self._anchor_idx = target_idx
            return None

        self._anchor_idx = target_idx
        self.anchored += 1
        self.drift_run = 0                       # ★ 纠回来了：连续不健康拍数归零
        implied = self._implied_scale(target, proj)
        if implied is not None:
            self._implied_scales.append(implied)
            if len(self._implied_scales) > self.implied_scale_window:
                self._implied_scales.pop(0)
        ds = self._along_ds(target, proj)
        if ds is not None:
            self._anchor_ds.append(ds)
            if len(self._anchor_ds) > self.implied_scale_window:
                self._anchor_ds.pop(0)
        try:                                             # 记下本次吸附所在的弧长
            self._last_anchor_s = float(self._path.project(target.x, target.y,
                                                           s_hint=proj.s).s)
        except Exception:                                # noqa: BLE001
            self._last_anchor_s = float(proj.s)
        self._store_drift(target, drift, now)
        self.bus.publish(EV_POSE_ANCHORED, source=self.name, block=target.block,
                         index=target_idx, drift=round(drift, 4),
                         x=round(target.x, 4), y=round(target.y, 4))
        self._log(f"到位吸附：{target.block}（航点 #{target_idx}）偏差 {drift:.3f} m"
                  f" → 锚回格心 ({target.x:.3f}, {target.y:.3f})，yaw 不变 {yaw:+.3f} rad")
        self._report_drift(target.block, target_idx, drift, now, anchored=True)
        return geo.Pose(target.x, target.y, yaw)

    def _error_shrinks(self, target, proj) -> bool:
        """★"误差不变大"判据（闸②）：把信念挪到 `target` 之后，离折线不许更远。

        · 用**离线距离**（`Path.project().dist`）而不是"离格心距离"：格心只是格的几何中心，
          车真正该在的位置是折线；吸附把点推离折线 = 制造误差。
        · 容差 `anchor_reject_eps_m`(0.005 m)：允许"持平/噪声级变差"，挡住明显变差。
        · 这一判据**只读**（用 `self._path.project` 直接投影，不碰 `self._s_hint` 状态）。
        """
        assert self._path is not None
        try:
            after = self._path.project(target.x, target.y, s_hint=proj.s,
                                       window_m=self.arrival_m, allow_backward=False)
        except Exception:                                    # noqa: BLE001 投影失败不阻断定位
            return True                                      # 判不了就别拦（保守放行）
        return float(after.dist) <= float(proj.dist) + self.anchor_reject_eps_m

    def _along_ds(self, target, proj) -> float | None:
        """本次吸附的**沿程修正量**（米，带符号）：>0 = 把信念往前拉，<0 = 往后拉。"""
        assert self._path is not None
        try:
            after = self._path.project(target.x, target.y, s_hint=proj.s,
                                       window_m=self.arrival_m, allow_backward=False)
        except Exception:                                    # noqa: BLE001
            return None
        return float(after.s) - float(proj.s)

    def _scale_suspected(self, target, proj) -> tuple[float, str] | None:
        """★★"吸附掩盖尺度误差"判别（闸③）：**同号 + 幅度到位半径级 + 隐含尺度越上限**才疑似。

        为什么不用"修正量 ÷ 段长"：段长只有 0.35 m，而单次吸附的沿程修正量天然可以到
        `arrival_m`(0.12) —— 噪声级的一次修正除以短段长就是 30%+，会**误杀正常吸附**
        （实测把 test_localization ⑦c 的第二次吸附也拦掉了）。
        真正区分"里程尺度误差"与"正常到位噪声"的是**模式**：
        ・连续 ≥3 次吸附的沿程修正量**同号**（总是往同一个方向拽）；
        ・幅度中位数 ≥ `0.5 × arrival_m`（不是噪声级，而是稳定地差一大截）。
        ・★（2026-10-17 实车追加，N5 多跑 10cm 的根因）隐含尺度 |est| ≥ `anchor_scale_max`：
          实车那趟 34 连拒的"连续同向 + 0.104m"其实是**到位边界偏移**（吸附总在
          arrival_m 边缘触发，修正量恒 ~0.1m 且不随距离增长），它的隐含尺度随分母
          增长一路衰减（44.7%→0.2%）——真实尺度误差的比值不会衰减。所以必须再用
          `anchor_scale_max`(8%) 这道门把"固定偏移"放行，只拦真正成比例的尺度误差。
          这正是实车那趟（4.55 m 里每段都往前拽、累计 69 cm）的指纹；
          而健康车的修正量是零均值的小噪声，凑不出"连续同号 + 大偏差"。
        返回 `(隐含尺度误差估计, 理由文本)`；不疑似返回 None。
        """
        ds = self._along_ds(target, proj)
        if ds is None:
            return None
        window = list(self._anchor_ds) + [ds]
        if len(window) < 3:
            return None
        same_sign = all(v > 0 for v in window) or all(v < 0 for v in window)
        size = sorted(abs(v) for v in window)[len(window) // 2]
        if not same_sign or size < 0.5 * self.arrival_m:
            return None
        ratio = self._implied_scale(target, proj)
        if ratio is None:
            # 无法判定（第一次吸附/段长太短/投影失败）→ 不拦（`_implied_scale` 同口径）
            return None
        est = ratio
        if abs(est) < self.anchor_scale_max:
            # 固定偏移（到位边界效应）会被长分母稀释到 8% 以下，放行；
            # 真实尺度误差的比值不会随距离衰减，会稳稳越过 8%。
            return None
        return est, (f"连续 {len(window)} 次同向、幅度中位 {size:.3f} m ≥ "
                     f"{0.5 * self.arrival_m:.3f} m；隐含尺度≈{est:+.1%} ≥ "
                     f"上限 {self.anchor_scale_max:.0%}")

    def _implied_scale(self, target, proj) -> float | None:
        """★隐含尺度误差 = 本次吸附的**沿程修正量** ÷ 距上次吸附走过的弧长。

        为什么这能看穿"吸附掩盖尺度误差"：里程虚增 ε 时，信念在**车还没到**顶点时就先到了，
        吸附于是把信念往前拽 ε×段长 —— 这个修正量与段长之比就是 ε 本身。
        只看"吸附次数/漂移告警"永远是健康的（实车 13/13 吸附、0 告警），
        但这条比值会稳稳地指着 18% ⇒ 尺度误差**看得见**了。

        返回 None 表示本拍无法判定（第一次吸附、段长太短、投影失败）——此时**不拦**。
        """
        assert self._path is not None
        try:
            after = self._path.project(target.x, target.y, s_hint=proj.s,
                                       window_m=self.arrival_m, allow_backward=False)
        except Exception:                                    # noqa: BLE001
            return None
        ds = float(after.s) - float(proj.s)                  # 沿程修正量（带符号）
        base = float(proj.s) - self._last_anchor_s if self._last_anchor_s is not None else None
        if base is None or base < 0.10:                      # 第一次 / 段太短：判不了
            return None
        return ds / base

    def _refuse_anchor(self, block: str, index: int, drift: float, now: float,
                       reason: str) -> None:
        """★吸附/重锚被拒 = **显式记账**：计数 + 日志 + 遥测。

        ⚠️ 刻意**不**复用 `EV_DRIFT_WARNING`：那是"漂移告警"的语义，且带"每航点至多一条"
        的去重约定（test_localization ⑦ 直接断言它）。拒绝是**另一件事**（本可纠偏但没纠），
        若混用会把那条去重契约搅乱 —— 所以拒绝走"计数 + 日志 + `P_MOTION_LIMIT` 载荷"，
        同样可被逐条审计（`anchor_refused` 计数、`⛔ 吸附被拒` 日志行）。
        """
        self.anchor_refused += 1
        self.drift_run += 1
        if self._warn.allow(f"anchor_refuse:{block}#{index}", now):
            self._log(f"⛔ 吸附被拒（{reason}）：{block}（航点 #{index}）偏差 {drift:.3f} m"
                      f" → 保留当前信念（累计拒绝 {self.anchor_refused} 次）")

    def guard_state(self) -> dict:
        """漂移护栏的**只读**状态快照（遥测/测试/收尾台账共用）。"""
        scales = sorted(self._implied_scales)
        median = None
        if scales:
            mid = len(scales) // 2
            median = round(scales[mid] if len(scales) % 2 else
                           (scales[mid - 1] + scales[mid]) / 2.0, 4)
        return {
            "anchor_refused": self.anchor_refused,
            "reanchored": self.reanchored,
            "drift_run": self.drift_run,
            "healthy_run": self.healthy_run,
            "motion_limited_ticks": self.motion_limited_ticks,
            "motion_limit_published": self.motion_limit_published,
            "boundary_alerts": self.boundary_alerts,
            "limit": (None if self._limit_state is None
                      else {"speed_mps": self._limit_state[1], "reason": self._limit_state[2]}),
            # ★尺度误差健康度（"吸附掩盖尺度误差"故障模式的可见化）：
            #   `implied_scale_median` = 隐含尺度误差中位数；`scale_suspect` = 是否越过上限。
            #   ⚠️ 本字段只做**判别与告警**，不改任何标定值（标定归队长）。
            "implied_scale_median": median,
            "implied_scale_n": len(self._implied_scales),
            "scale_suspect": self.scale_suspect,
            "scale_suspect_max": round(self.scale_suspect_max, 4),
            "anchor_scale_max": self.anchor_scale_max,
            # ★航向完整性守卫（2026-10-17 K9 事故）：判定结果 + 分母（误报率评估用）
            # ★行进方向裁决（折返压线）的可见化
            "motion_heading_ticks": self.motion_heading_ticks,
            "direction_rejects": self.direction_rejects,
            "travel_heading_deg": (None if self._travel_heading is None
                                   else round(geo.deg(self._travel_heading), 1)),
            "yaw_untrusted": self.yaw_untrusted,
            "yaw_guard_trips": self.yaw_guard_trips,
            "yaw_guard_recoveries": self.yaw_guard_recoveries,
            "yaw_manual_resets": self.yaw_manual_resets,
            "yaw_clear_ticks": self.yaw_clear_ticks,
            "yaw_jump_deg": self.yaw_jump_deg,
            "yaw_diff_gain_dps_per_mps": self.yaw_diff_gain_dps_per_mps,
            "yaw_diff_spin_dps": self.yaw_diff_spin_dps,
            "yaw_guard_samples": self.yaw_guard_samples,
            "yaw_bad_samples": self.yaw_bad_samples,
            "yaw_skipped_frames": self.yaw_skipped_frames,
            "yaw_manual_ticks": self.yaw_manual_ticks,
            "yaw_imu_mismatch": self.yaw_imu_mismatch,
            "yaw_reason": self._yaw_reason,
            "yaw_last_rate_dps": round(self._yaw_last_rate_dps, 1),
            "yaw_max_rate_dps": round(self.yaw_max_rate_dps, 1),
            "yaw_last_limit_dps": round(self._yaw_last_limit_dps, 1),
            "yaw_last_dyaw_deg": round(self._yaw_last_dyaw_deg, 1),
            "warnings": self.drift_warnings,
            "note": "吸附会掩盖尺度误差：漂移告警为 0 **不等于**里程健康",
        }

    def _reanchor(self, proj, cx: float, cy: float, yaw: float, gate, now: float):
        """★横向重锚（"重锚/自愈"路径）：在**顶点窗口内**把信念的横向分量拉回折线。

        为什么只在顶点窗口内动手：**"误差变小才施加"**这条规则要落地，必须先知道
        "车此刻应该在哪"。唯一可靠的这类时刻就是"正在过一个已知顶点"（`_warning_gate`
        给出的弧长窗口）—— 此时车按路线就在折线上，把信念的横向残差清零是**有依据的**
        纠偏；而在自由行驶段拉信念，就真的只是在"制造误差"（早期发现#1 的反面）。
        三道闸：
          ① 残差 `proj.dist ≤ drift_warn_m` → 误差本来就不大，**不动**（no-op）；
          ② 残差 `> reanchor_pull_max_m` → 不硬拉（那是整段跑偏，不是横向噪声）⇒
             `_refuse_anchor()` 显式记账 + 交给护栏降速/停机；
          ③ 通过①② ⇒ 把信念移到**垂直足点**（只消横向分量、不改沿程位置），
             按构造"离线距离 0 ≤ 原来的 dist"，即"误差确实变小了"。
        """
        if proj is None or self._path is None or gate is None:
            self._reanchor_active = False
            return None
        lat = float(proj.dist)
        gidx, gwp = gate
        # ★ 顶点换了 ⇒ 收敛状态作废（避免跨顶点沿用"正在拉回"标记）
        if self._reanchor_active_idx != gidx:
            self._reanchor_active = False
            self._reanchor_active_idx = gidx
        if lat <= self.drift_warn_m and not self._reanchor_active:
            return None                                  # ① 不制造误差：没必要动
        if gidx == self._reanchor_idx:
            # ★ 同一顶点只重锚一次：它是**幂等的**（把信念投影到垂直足点），每拍重做不会更准，
            #   只会把计数和日志刷爆；换顶点后重新允许（与 `_anchor_idx` 同款闩锁）。
            return None
        if lat > self.reanchor_pull_max_m:               # ② 超上限 → 拒绝 + 记账
            self._reanchor_idx = gidx
            self._refuse_anchor(gwp.block, gidx, lat, now,
                                f"横向残差 {lat:.3f} m > 重锚上限 {self.reanchor_pull_max_m:.3f} m")
            return None
        # ★★ F5（2026-10-25 实车 run4）：**分拍收敛**，而不是一步拽到足点。
        #   run4 证据：残差 0.303 m 一次拉满 → 位姿突跳 0.313 m（> 0.20 门限）→
        #   FixTask 判"清窗" + 漂移护栏 drift_run 累到 8 → drift_stop 停死。
        #   现在每拍最多拉 reanchor_step_max_m（默认 0.12 m），几拍内收敛到
        #   reanchor_done_m（默认 0.02 m），单拍位移 < 突跳门限；收敛完成才闩锁该顶点。
        step = min(lat, self.reanchor_step_max_m)
        frac = step / lat
        tx = cx + (float(proj.x) - cx) * frac
        ty = cy + (float(proj.y) - cy) * frac
        remain = lat - step
        done = remain <= self.reanchor_done_m
        if done:
            self._reanchor_idx = gidx
            self._reanchor_active = False
            self.reanchored += 1
        else:
            self._reanchor_active = True
        self.drift_run = 0
        new_pose = geo.Pose(tx, ty, yaw)
        self._store_drift(new_pose, remain, now)
        self.bus.publish(EV_POSE_ANCHORED, source=self.name, block=gwp.block, index=gidx,
                         drift=round(lat, 4), step=round(step, 4),
                         x=round(tx, 4), y=round(ty, 4))
        if done or self._warn.allow("reanchor", now):
            self._log(f"↩️ 横向重锚：{gwp.block}（航点 #{gidx}）横向残差 {lat:.3f} m"
                      f" → 本拍拉回 {step:.3f} m（上限 {self.reanchor_step_max_m:.2f}）"
                      f"，剩 {max(0.0, remain):.3f} m{'' if done else '，下拍继续'}")
        return new_pose

    # ------------------------------------------------------------------ #
    #  ★信念漂移护栏：P_MOTION_LIMIT 的**写**侧
    # ------------------------------------------------------------------ #
    def _distance_to_boundary(self, pose) -> float | None:
        """信念离**安全边界**还有多远（米）；边界本身用与 Safety 同源的 `geo.map_bounds`。

        安全边界 = 地图外框内缩 `pursuit.map_safety_margin_m`（Safety 的越界判据同源），
        所以这里的"还有多远"= 到那个内缩矩形的距离；在矩形内为正、越界为负。
        """
        margin = float(self.cfg.get("pursuit.map_safety_margin_m") or 0.0)
        x_min, x_max, y_min, y_max = geo.map_bounds(margin)
        dx = min(pose.x - x_min, x_max - pose.x)
        dy = min(pose.y - y_min, y_max - pose.y)
        return min(dx, dy)

    def _update_drift_guard(self, pose, *, unhealthy: bool, now: float) -> None:
        """把"漂移健康度 + 边界接近度"折算成一条**限速/停机请求**，写 `P_MOTION_LIMIT`。

        ★ 语义约定（读侧必须遵守，见模块 `P_MOTION_LIMIT` 注释）：
          · 键**存在且非空** → 把线速度钳到 `speed_mps` 以内（只降不升）；
          · 键**不存在 None** → 完全 no-op，执行侧行为与没有本功能时**逐位一致**。
        这样做是为了让零注入大扫描在"无漂移"时拿到与现版本完全相同的结果。
        """
        self.drift_run = self.drift_run + 1 if unhealthy else 0
        self.healthy_run = 0 if unhealthy else self.healthy_run + 1

        dist_b = self._distance_to_boundary(pose)
        # ---- 边界护栏：提前降速 + 告警（而不是只有越界后的硬 SAFE_STOP）----
        boundary_level = ""
        if dist_b is not None:
            if dist_b <= self.boundary_stop_m:
                boundary_level = "stop"
            elif dist_b <= self.boundary_guard_m:
                boundary_level = "guard"
            if boundary_level:
                self.boundary_alerts += 1
                if not self._boundary_warned:
                    self._boundary_warned = True
                    self._log(f"⚠️ 信念已接近安全边界：离边界 {dist_b:.3f} m "
                              f"（护栏 {self.boundary_guard_m:.2f} m / 停车 {self.boundary_stop_m:.2f} m）"
                              f" → 请求{'停车' if boundary_level == 'stop' else '降速'}，"
                              f"避免带着漂移撞上硬 SAFE_STOP")
                self._log_throttled(f"boundary:{boundary_level}", now,
                                    f"边界护栏（{boundary_level}）：离边界 {dist_b:.3f} m，"
                                    f"限速 {self.boundary_guard_speed_mps:.2f} m/s")
                self.bus.publish(EV_DRIFT_WARNING, source=self.name, block="",
                                 index=-1, drift=round(-dist_b, 4),
                                 threshold=self.boundary_guard_m, anchored=False,
                                 refused=False, reason=f"boundary_{boundary_level}")
            elif self._boundary_warned:
                self._boundary_warned = False        # 离开护栏区：告警可再次触发

        # ---- 漂移护栏：连续不健康 → 降速 → 停车 ----
        level, speed, reason = "none", None, ""
        if self.drift_run >= self.drift_stop_after:
            level, speed, reason = "stop", 0.0, "drift_stop"
        elif self.drift_run >= self.drift_slow_after:
            level, speed, reason = "slow", self.drift_slow_speed_mps, "drift"
        if boundary_level == "stop":
            level, speed, reason = "stop", 0.0, "boundary_stop"
        elif boundary_level == "guard" and level != "stop":
            level, speed, reason = "slow", min(self.boundary_guard_speed_mps,
                                               speed if speed is not None else 1e9), "boundary"
        # ---- ★★航向失可信 = 最高优先级硬停（判据见 _update_yaw_guard）----
        # 放在最后：它压过漂移/边界的一切"降速"，且闩锁后每拍都是 0 速请求。
        if self.yaw_untrusted:
            level, speed, reason = "stop", 0.0, (self._yaw_reason or "yaw_untrusted")

        if level != "none":
            self.motion_limited_ticks += 1
        elif self.drift_run == 0 and self.healthy_run >= self.drift_clear_ticks:
            level, speed, reason = "clear", None, ""

        state = (level, speed, reason) if level in ("slow", "stop") else None
        if state != self._limit_state:
            self._limit_state = state
            if state is None:
                # 撤销：写 None（读侧必须把"无信号"当 no-op）
                self.bb.set(P_MOTION_LIMIT, None, source=self.name, ts=now)
                if level == "clear" and self.motion_limited_ticks:
                    self._log("✅ 漂移/边界护栏解除：信念已连续 "
                              f"{self.healthy_run} 拍干净，撤销限速请求")
            else:
                payload = {"speed_mps": float(speed or 0.0), "reason": reason,
                           "drift_run": self.drift_run,
                           "anchor_refused": self.anchor_refused,
                           "distance_to_boundary_m": (None if dist_b is None
                                                      else round(dist_b, 4)),
                           # ★尺度误差健康度随限速一起上报（"吸附掩盖尺度误差"的可见化）
                           "implied_scale_median": self.guard_state()["implied_scale_median"],
                           "scale_suspect": self.scale_suspect,
                           # ★航向守卫的可见化（读侧只该用 speed_mps/reason，其余仅供遥测）
                           "yaw_untrusted": self.yaw_untrusted,
                            "yaw_guard_recoveries": self.yaw_guard_recoveries,
                           "yaw_rate_dps": round(self._yaw_last_rate_dps, 1),
                           "yaw_limit_dps": round(self._yaw_last_limit_dps, 1),
                           "since": now}
                self.bb.set(P_MOTION_LIMIT, payload, source=self.name, ts=now)
                self.motion_limit_published += 1
                self._log(f"🛑 下发限速请求 P_MOTION_LIMIT={payload['speed_mps']:.2f} m/s"
                          f"（原因 {reason}；连续不健康 {self.drift_run} 拍；"
                          f"离边界 {dist_b if dist_b is None else round(dist_b, 3)} m）")

    def _log_throttled(self, key: str, now: float, text: str) -> None:
        if self._warn.allow(key, now):
            self._log(text)

    def _anchor_candidates(self, proj, x: float, y: float) -> tuple[tuple[int, object], ...]:
        """吸附候选，按优先级：① 当前段末端航点 ② 其后最近的未完成航点（兜底）。

        ★ 判据用**段索引**而不是"离最近格心 < 阈值"：后者会让车在 K 列附近任何位置都被吸附
          （旧代码桥洞保护踩过的"僵直"坑，survey §6.1 防误伤设计）。

        兜底候选（②）的准入 = 「正在趋近该顶点」：顶点尚未被甩到身后
        （`dist_to_vertex + drift_warn_m >= 0`，允许顶点略在身后），且横向偏差不超过
        `max(arrival_m, drift_warn_m)` —— 吸附半径是 `arrival_m`（0.12 m），
        这个准入只是"值得算一下距离"的必要条件，不构成吸附判据本身。
        ★ 末点例外：吸附半径在 `_anchor` 里收紧到 `_GOAL_STOP_M`(0.03)，准入条件不变。
        """
        assert self._path is not None
        pts = self._path.waypoints
        n = len(pts)
        nxt = proj.seg_index + 1
        if proj.passed_end:
            return ((n - 1, pts[n - 1]),)
        if nxt >= n:
            return ()
        if proj.dist_to_vertex + self.drift_warn_m < 0.0:
            return ()
        if proj.dist > max(self.arrival_m, self.drift_warn_m):
            return ()
        out: list[tuple[int, object]] = [(nxt, pts[nxt])]
        if proj.dist <= self.drift_warn_m:      # 贴着折线才有资格补锚更远的点
            out.extend((idx, pts[idx]) for idx in range(nxt + 1, n))
        return tuple(out)

    # ------------------------------------------------------------------ #
    #  漂移判定（独立于吸附）
    # ------------------------------------------------------------------ #
    def _warning_gate(self, proj) -> tuple[int, object] | None:
        """当前是否处于"可判定漂移"的窗口：车正在接近某个顶点。

        ★ 主判据是**弧长**（沿路走到离顶点还剩 ≤ `pursuit.drift_seg_gate_m`），
          不是"离折线的横向距离"。这是刻意的：横向距离大的车恰恰是漂移最大的车，
          用它当门槛 = "漂得越狠越报不出来"。弧长判据与横向无关，任何方向漂移都进窗。
          辅以顶点半径判据，覆盖"刚越过顶点一点、横向还歪着"的情形。
        """
        assert self._path is not None
        pts = self._path.waypoints
        n = len(pts)
        if proj.passed_end:
            return (n - 1, pts[n - 1])
        idx = proj.seg_index + 1
        if idx >= n:
            return None
        wp = pts[idx]
        d_back = proj.dist_to_vertex
        d_vertex = wp.distance_to(proj.x, proj.y)
        in_seg_gate = (d_back >= -1e-9) and (d_back <= self.drift_seg_gate_m)
        if in_seg_gate or d_vertex <= self.drift_warn_m:
            return (idx, wp)
        return None

    def _reset_anchor_latch(self, x: float, y: float, proj, now: float) -> None:
        """本次没吸附成：判断是"还没到"还是"已经走过且干净"。

        ★ 只有"投影已经离开被闩住的航点"才解开闩锁。
          两个反面情形都要挡住：
          · 车还在半路（离下一航点 0.3 m > arrival 0.12 m）：此时若解锁，
            等真正到位时会**再锚一次** —— 同一航点反复吸附，越锚越偏；
          · 车刚走过航点但投影还停在它附近（例如 M10 → M9 的下一段）：
            此时若随便解锁，会把"刚刚锚过的点"再锚一遍。
          判据 = 投影的段索引已越过被闩住的航点，**或**该航点已不在可判定窗口里。
        """
        if self._anchor_idx < 0:
            return
        passed = (proj.seg_index + 1 > self._anchor_idx) or proj.passed_end
        if passed and self._warning_gate(proj) is None:
            if self._warn.allow("anchor_depart", now):
                self._log(f"已走完航点 #{self._anchor_idx}（投影已在其之后、且已离开判定窗口）"
                          f" → 下一个航点可再次吸附")
            self._anchor_idx = -1

    def _report_drift(self, block: str, index: int, drift: float, now: float,
                      *, anchored: bool) -> None:
        """漂移告警。★ 每个航点最多一条：按 `(block, index)` 去重，Throttle 只作兜底。

        时间限流下同一航点会按 20 Hz 刷屏；按航点去重则是"每航点至多一条" ——
        既保证异常逃不掉，又不会把日志冲掉。
        """
        if drift <= self.drift_warn_m:
            return
        key = (block, index)
        if self._anchor_key == key:
            return
        self._anchor_key = key
        self.drift_warnings += 1
        self.bus.publish(EV_DRIFT_WARNING, source=self.name, block=block, index=index,
                         drift=round(drift, 4), threshold=self.drift_warn_m,
                         anchored=anchored)
        self._log(f"⚠️ 漂移告警：{block} 偏差 {drift:.3f} m > {self.drift_warn_m:.3f} m"
                  f"（{'已锚回格心' if anchored else '未吸附（未进入到位半径）'}，"
                  f"累计 {self.drift_warnings} 次）")

    def _store_drift(self, target, drift: float, now: float) -> None:
        """把本次锚定偏差写进 P_FIX（P3 的 FixTask 用它做估计，不能丢）。"""
        fix = self.bb.get(P_FIX)
        if not isinstance(fix, FixState):
            fix = FixState()
        new_fix = FixState(
            dx=fix.dx, dy=fix.dy, dyaw=fix.dyaw, scale=fix.scale,
            anchor_count=fix.anchor_count + 1,
            last_anchor=target.block,
            drift_m=drift,
            max_drift_m=max(fix.max_drift_m, drift),
        )
        self.bb.set(P_FIX, new_fix, source=self.name, ts=now)

