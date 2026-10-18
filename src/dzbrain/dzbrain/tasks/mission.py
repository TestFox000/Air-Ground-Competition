# -*- coding: utf-8 -*-
"""MissionTask（10 Hz）—— 任务调度：出发 → 航点推进 → 停站三段 → 终点横幅。

对应 `docs/P1接口约定.md` §3.3；停站三段式与超时兜底复刻旧实现
`path_executor` 的打靶四阶段状态机（survey §4.3，旧 2034–2193），改动两处：

1. **三段而不是四段**：旧版 `SETTLE → ACQUIRE → STABLE → FIRE` 里的 `STABLE`
   （等瞄稳 40 px 连续 5 帧）在新版归 `AimTask`（云台自己判定"锁定"），
   Mission 只在 `ACQUIRE` 里等 `aim_status == TRACK` 且视觉有目标 → 直接进 `FIRE`。
2. **每段独立超时**：绝不允许任何一段把主循环卡住（旧版 `SHOOT_WAIT_TIMEOUT` 是死变量）。

铁律：**宁可漏打，不可卡死**。所以每一段都有"超时即放行"分支，且放行前一定
`publish_fire_gate(False)` 收尾（旧版 2181–2183 的同一条纪律）。

其他约定与出处：

- 出发：`mission.auto_start` + `mission.auto_start_delay_s` → `parse_reset('N10,y')` 复位 belief
  → `parse_mission(route.tokens)` → 写 `P_ROUTE` → `EV_ROUTE_LOADED` → 阶段 `DRIVE`（§3.3）。
- 开机跨帧一致性：`mission.scan_mode == 'continuous'` 时开局发 `publish_scan_request(2)`
  （**持续**扫描）。readme【十】第 31 条：绝不能在到点时发 `1` 把它降级成单趟。
- ★★ **云台扫描单写者**（与 AimTask 的锁止配套）：`aim.gimbal_lock_enabled=true`
  （默认）时，云台扫描开关**由 AimTask 独家掌管**（巡迹段 0 锁止 / 最佳射击点停站 2 起转 /
  离开停站与 FINISH 回 0），**Mission 一个值都不发** —— 本文件的 `_request_scan()` 直接返回，
  `mission.scan_mode` 在该模式下 **inert**（只影响横幅文案）。理由见 `_request_scan` docstring
  （run11 实证：Mission 的 2 在巡迹全程压住了 Aim 的 0，而 C++ 侧是电平保持语义）。
  仅 `aim.gimbal_lock_enabled=false`（旧行为/对照档）时，Mission 才按 `scan_mode` 照旧下发。
- 终点：最后一个航点到达 → `FINISH`（**锁存**）、`publish_brake()`、（旧行为档才发 `scan_request(0)`）、
  `EV_MISSION_FINISHED`、结束横幅（用时/发数/漂移）。
- ★ PIVOT 原地掉头顶点：Tracker 完成/放弃 PIVOT 时经 `PIVOT_DONE_KEY` 通知本任务，
  该顶点直接判到达（`pivot_dist_m` > `arrival_m`，正常判据恒假，bench 实测 E11/A6/B6
  被拐角绕圈兜底强制跳过）。停站点恰是掉头顶点时照常进入三段式停站。
- 外部命令（`/nav/waypoint`）：`reset:BLOCK,HEAD` / `load:文件` / `单点 BLOCK,HEAD` / `多点 ...`
  全部支持；**未知命令只告警不崩**（旧版 `waypoint_callback` 的教训）。

★ 时间来源一律走构造注入的 `clock`（默认真实 `time.monotonic`），三段超时全部基于它，
  测试用 `FakeClock` 快进即可 —— 任务内**不得**直接调 `time.time()`。

★ 自动规划（2026-10-08）：`planner.auto_plan_enabled: true` 时**启动即自己算路** ——
  读 `planner.map_path` 的地图 → `route.planner.plan_route(..., allow_reverse=...)` →
  成功就用 `PlanResult.tokens` 当执行路线（日志打**完整 token 列表 + 规划耗时(ms)**，
  横幅标「路线来源：自动规划」）。失败语义按 `planner.fallback_to_tokens`：
  `true` → warn（含异常类型+消息）后回退 `route.tokens`，**仍进入 DRIVE**；
  `false` → error + **停在 IDLE 不出发**（绝不 publish `EV_MISSION_STARTED`）。
  规划入口 `plan_route_from_config()` 与 `python -m dzbrain --plan-only` **共用同一段代码**
  —— 否则"静态规划通过"不等于"车上规划通过"，那种绿灯毫无意义。
  ⚠️ `planner.*` 目前**未登记进 `core/config.py` 的 SCHEMA**（不在改动范围内）：
  `Config.validate()` 会给这 5 个键各记一条 "未知配置段" warn（不是 error，门禁仍过），
  且键名拼错不会被 SCHEMA 兜住 —— 建议收口时补进 SCHEMA 并把 `planner.` 加进
  `known_prefixes`。

**无人机桥洞情报接线**（`EV_BRIDGE_INFO` / `P_DRONE` → 穿洞格选择）
  缺口：`tasks/drone.py` 收到 `bridge_passable{half,block}` 时写 `P_DRONE`（`DroneState`，
  `bridge_of(half, now, stale_s)` 已含"半场匹配 + 情报过期 `>drone.info_stale_s`"判据），
  但**接线前 Mission 全文不读它** ⇒ 情报到车即弃，穿洞格永远走 `route.cross_*_default`。
  ── （按早期实现的覆盖式赋值缺陷风险修正）：情报的**唯一真源 = `EV_BRIDGE_INFO` 事件流** ──
  ⚠️ D3（实测）：`DroneState` 只有**一组** `bridge_half/bridge_block/bridge_ts` 槽，
    `DroneTask._on_bridge_passable` 每收一条 `bridge_passable` 就**覆盖式赋值**（drone.py:283）。
    无人机每轮按 **lower → upper** 顺序发，于是稳态下槽里恒是 `upper`：只看 `P_DRONE` 快照，
    `bridge_of("lower")` **恒为 None** —— 第一版"读槽 + 按半场累积"虽然凑出了两半场，
    但它把"能不能同时拿到两半场"绑在了**轮询时机**上（槽被覆盖的那一拍正好没 tick 就丢）。
  修法（采选项①）：**订阅 `EV_BRIDGE_INFO`**，每条事件自带 `half/block/ts`，
    按半场各存一份（`_intel_seen`），**后到的 upper 事件永远顶不掉先到的 lower 记录**。
    事件载荷里的 `ts` 就是 `DroneTask` 写 `bridge_ts` 用的**同一个数**（drone.py:283-287），
    所以过期判据 `now - ts > drone.info_stale_s` 与 `DroneState.bridge_of()` **逐字同源**。
    `P_DRONE` 仍要读，但只用来判**链路状态**（是否存在 / 是否在线）—— 那是黑盒事实，不是情报值；
    `DroneTask` 的"写槽"与"发事件"在同一段代码里（drone.py:283-287），
    故 `事件覆盖 ⊇ 槽覆盖`：生产上不存在"槽里有、事件没有"的情报 —— 槽位不再是情报真源。
  采纳顺序（**两个半场各自独立判定**，任一半场失败不影响另一半场；已通过的 5 道门 + 过期判据
  **逐点未改**，只换了"原始格与 ts 从哪儿来"）：
    ① `drone.enabled=false` → 默认格；② 无 `P_DRONE`（DroneTask 未运行/被禁用）→ 默认格；
    ③ 无人机离线（`DroneState.online=False`，心跳超 `drone.heartbeat_timeout_s`）→ 默认格
       （**即使 `bridge_ts` 还在过期窗口内**：离线即情报不可信，保守口径）；
    ④ 情报过期（`now - 事件 ts > drone.info_stale_s`）/ 该半场从未收到事件 → 默认格；
    ⑤ 格名非法（`Z9`/`ZZ9`）、不在 K 列（`A1`）、或不在本半场行段
       （下 K7~K11 / 上 K1~K6）→ 默认格
       —— **★ 权威门放宽**：判据从"候选集合 ∩ `map.bridges`"（`planner._cross_pools`
       同口径）改为**K 列 + 本半场行段**，于是无人机的新鲜情报可以**纠正过期地图**
       （地图/候选还停在上一局、只开了别的洞时，行段内别的 K 列格照样采纳）；
       池外但在行段内 → 采纳，并把口径写进 `reason`/日志（"池外…纠正过期地图"）。
       **绝不把一条坏情报变成规划器的 `PlannerError`**（那会让 `fallback_to_tokens=false`
       的配置直接停在 IDLE）：坏情报只降级成默认格，不炸自动规划、不卡发车；
    ⑥ 其余 → 采用无人机格。
  两个出口（各自的"默认值路径"与接线前**逐字节一致**）：
    · `planner.auto_plan_enabled=true` → 作为 `cross_lower/cross_upper` 关键字传给
      `plan_route_from_config()`（缺省 `None` = 完全走规划器自己的 `_default_cross`）；
    · 手动路线（`route.tokens` / `mission_blocks`）→ 把"进洞口-K 列格-出洞口"**同行三连**
      整体挪到新行（`rewrite_crossing_tokens`：只认规划器生成的那种形状），且结果**必须**过
      `validate_route`(0 error) 才允许执行；形状不符/校验不过 → 原样不动（走默认洞）。
  默认穿洞格的**信号源**＝配置 `route.cross_{half}_default`：
    ★ 现场真值（`config/robot.yaml:520-521`，实测）＝ **lower `K10` / upper `K5`**，
      与候选集合（robot.yaml:513-514 ＝ `["K10"]` / `["K5"]`）、现役 `route.tokens` 的穿洞三连
      （`L10→K10→J10` / `J5→K5→L5`）三者一致。
    ✅ `core/config.py` 的 SCHEMA 缺省**已同步为 `K10`/`K5`**（收口；取证报告里"SCHEMA 仍是
      K9/K4"那句已失效）。本文件不做"临时兜底改默认值"：兜底一律以配置为唯一真源，
      否则现场翻不出到底走的哪个洞。
  等待窗口（按 F1 裁决**收回判据**）：`mission.bridge_info_wait_s`（现场 2s）内，
  只有"**等得到东西**"才推迟发车 —— 需要同时满足：
    · 车侧有无人机链路（`P_DRONE` 存在）**且 `online=True`**（离线即情报不可信）；
    · 并且**手上已经有一半场的有效新鲜情报**（>=1 个半场 `source=drone`，即 age ≤ `info_stale_s`
      且通过了 5 道采纳门）；
  两半场都齐 → 不等；离线/无链路/情报过期或从未到达 → **不等、立即按默认格 K10/K5 出发**。
  硬上限仍是 `auto_start_delay_s + bridge_info_wait_s`（兜底：到点必发车，绝不卡死），
  窗口耗尽时用"已得情报 + 默认格"。
  ⚠️ 为什么要收回：`DroneTask` **每拍都写 `P_DRONE`**（无论在线与否），所以"有 link"这条判据
  恒真 —— 只看 link 会让无人机离线时也白等到硬上限（审出的 F1）。
  窗口耗尽照常出发（用已得情报 + 默认格）。**无 `P_DRONE` 时不等、不换，行为与接线前逐字一致。**
  边界（刻意不改，见 `_handle_command`）：外部命令（`/nav/waypoint` 的 `多点`/`单点`/`load:`）
  按操作者原意执行，不做换洞；发车后晚到的情报只进遥测（`state()["bridge"]`，其中 `used`
  是**冻结**的实际用格），路线已冻结不再换洞。

**L10 桥前等待 + 迟到情报改道一次**（机制设计段先行，按设计建议②⑤ / 复核）

  ① 保持/让位仲裁（**不新增第二个驱动指令写者**）：新增 `MissionPhase.HOLD`（`io/contract.py`）。
     `tracker.ACTIVE_PHASES` 是**白名单**（只有 DRIVE/APPROACH），且 `_blocked_reason_now()` 里
     "阶段不在白名单 → `_hold()` → `publish_brake()`" 已经存在 ⇒ Mission 一切到 HOLD，
     Tracker 每拍自己刹车；HOLD 同时登记进 `tracker.BLOCKED_PHASES`（文档口径一致）。
     Mission 侧在**进 HOLD 那一拍**也发一次 `publish_brake()`（双保险，语义与停站/终点一致）。
  ② 决策点 = **洞口（K 列格）航点的前一个航点**：默认路线是 `L10`（下半场）/`J5`（上半场）。
     `_decision_point()` 返回"第一个**洞口尚未过**的穿洞三连"，取 `(决策点下标, 洞口下标, 半场)`。
     ⇒ 不写死 L10：换一张图/换一个洞，决策点自动跟着走。
  ③ 等待条件（缺一不可，全部满足才进 HOLD；**任一条不满足 = 不等**）：
     · 车侧**有**链路（`P_DRONE` 存在）且 `online=True`（离线/无链路 = 情报不可信，**逐拍与当前版本一致**）；
     · 两个半场**没有**都拿到新鲜情报（都齐 ⇒ 直接决策，不停车）；
     · 上限 `cap > 0`（见 ④）。
  ④ 上限口径：`cap = min(mission.bridge_wait_max_s(默认 12s), 预算余量)`，
     预算余量 = `mission.race_budget_s(默认 180s，3 分钟口径) − (now − 本局 DRIVE 起点)`，
     取 `max(0, …)`。**到点必发车**（语义不变）：`waited ≥ cap` 那一拍立刻放行，绝不再等；
     `cap ≤ 0`（预算已耗尽 / 等待上限为 0）⇒ **不进 HOLD**，直接带已得情报走。
  ⑤ 迟到情报改道（**全程 ≤ `mission.bridge_reroute_max`(默认 1) 次**）：
     · **决策点前**（洞口未过）到达的新鲜情报 → 用 `_apply_intel_to_tokens()`（形状 + `validate_route`
       0 error 才通过）把穿洞三连换到新行 → `parse_mission` → **`_install_route(..., keep_progress=True)`**：
       保留 `_passed/_arrived/_next_wp/_min_dist/_arrived_by/_skipped_at/_route_s/_last_s/_handled_stops`
       与黑盒 `P_ROUTE_INDEX/P_ROUTE_S`（修掉"重装路线 = 进度归零 = 车回 N10"这条阻断）；
       换洞只动穿洞三连的 3 个 token ⇒ 决策点及其之前的**下标一一对应**，所以保留索引是自洽的。
     · **决策点后**（洞口已过 / 改道次数用尽 / 情报不新鲜）→ **维持原 token**（只进遥测）。
  ⑥ 起点窗口（`bridge_info_wait_s`）**语义保留但默认关闭**：`mission.bridge_wait_at_start`
     （默认 false）—— 等情报这件事从起点**移到** L10 决策点（设计建议②）；要复现起点等待的
     对照档，把它显式置 true（`tests/test_mission.py` 的 14.30/14.31/14.31b 就是这么钉的）。
  ⑦ 遥测：`bridge.note.hold = {active, cap_s, waited_s, half, reroutes}`（现场看"为什么停着"）。

⚠️ SCHEMA 缺键（只剩一个仍缺）：
  仍缺：`mission.shot_timeout_s`（本文件用 `cfg.get(key, 30.0)` 兜底）。缺它 ⇒ `Config.validate()`
  看不到这个键，键名拼错不会被"未知配置段"告警兜住。
  已登记（无需再改）：`mission.shots_per_stop` / `settle_time_s` / `settle_eps_m` /
  `settle_speed_mps` / `lock_timeout_s` / `scan_stop_timeout_s` / `bridge_info_wait_s`
  / `drone.*` / `route.cross_*`。

★ 规划引擎开关：`planner.engine`（`tour` 缺省 / `tracker-safe`）。
  `tracker-safe`（"真车能跟住"）的产物**必须**过双门禁
  （`tools/route_check.py` 0 error + `route.validate_route` 0 error/0 warn），
  否则按 `fallback_to_tokens` 回退现役手工路线并打印原因。缺省 `tour` = 历史行为逐字节不变。
  门禁实现见 `check_planned_route()` / `GateResult`（同一段代码也被 `--plan-only` 复用）。
  ⚠️ 与上面 `planner.*` 一样，`planner.engine` / `planner.tracker_max_seconds`
  **未登记进 SCHEMA** ⇒ `Config.validate()` 会各记一条"未知配置段"warn（不是 error）。

★ 步进循环（T27，docs/step_loop.md）：本任务新增**段执行**接口（`request_segment()` /
  `stop_segment()` / `segment_status()` / `reset_pose()`）。main 的 StepLoop 在**另一个
  线程**投递段请求，本任务在 tick 开头（`_process_segment_requests`）才真正载入/发车
  —— 与外部 /nav/waypoint 命令同一模式，保证 route/阶段只被本线程改写。
  段结束时 `_check_finish()` 把完成单（segment_id / target_cell / block / pose_block /
  known_count / elapsed_s）记进 `segment_status()["done"]`；**FINISH 锁存跨段必须解闩**
  （`request_segment()` 接受请求时就清掉 done/error，任务线程起段时清 `_finish_latched`）。
  段规划器见 `plan_segment_route()`（点对点；未知格由 main 以 `blocked_extra` 传入）。
"""

from __future__ import annotations

import heapq
import math
import pathlib
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable

from ..core import geometry as geo
from ..core.blackboard import (P_AIM_RED, P_DRONE, P_MISSION_PHASE, P_POSE, P_ROUTE,
                               P_ROUTE_INDEX, P_ROUTE_S, P_SAFETY, P_STOP_POINT,
                               P_TELEMETRY, Blackboard)
from ..core.config import Config, pivot_dist_eff
from ..core.events import (EV_BRIDGE_INFO, EV_MISSION_FINISHED, EV_MISSION_STARTED,
                           EV_ARRIVED_STOP_POINT, EV_ROUTE_LOADED, EV_STOP_POINT_DONE,
                           EV_WAYPOINT_REACHED, EventBus, Subscription)
from ..core.logging import FaultGuard, Throttle, banner
from ..core.state import DroneState, SafetyState, StopKind, StopPhase, StopPointState
from ..core.task import TaskThread
from ..io.contract import AimStatus, MissionPhase
from ..pursuit.path import Path
from ..route.model import Route, Waypoint
from ..route.parser import detect_crossing, parse_mission, parse_reset, parse_token
from ..route.validate import (CROSS_COLUMN, CROSS_LOWER_ROWS, CROSS_UPPER_ROWS,
                              validate_route)
from .tracker import PIVOT_DONE_KEY, TRACKER_CMD_KEY, PivotPhase

__all__ = ["MissionTask", "MISSION_STATE_KEY", "plan_route_from_config", "resolve_map_path",
           "PlanOutcome", "AutoPlanBlocked", "ROUTE_SOURCE_AUTO", "ROUTE_SOURCE_MANUAL",
           # 引擎开关 + 规划产物双门禁
           "ENGINE_TOUR", "ENGINE_TRACKER_SAFE", "DEFAULT_PLAN_ENGINE", "PLAN_ENGINES",
           "resolve_plan_engine", "plan_budget_seconds", "GateResult", "check_planned_route",
           "route_check_script", "load_route_check_module",
           # 参考点偏移感知足迹（挡板/障碍/靶板顾问级诊断）
           "FootprintReport", "footprint_diagnostic", "vehicle_geometry",
           "FOOTPRINT_GATE_MODES", "DEFAULT_FOOTPRINT_GATE",
           # 无人机桥洞情报（EV_BRIDGE_INFO → 穿洞格）
           "BRIDGE_HALVES", "BridgeIntel", "rewrite_crossing_tokens", "retag_block_token",
           # ★T27 步进循环的段规划（点对点；未知格由调用方作为 blocked_extra 传入）
           "SegmentPlan", "plan_segment_route", "heading_dir_from_yaw"]

#: 遥测键：Mission 的当前上下文（停站点/航点/用时）
MISSION_STATE_KEY = "mission_state"

#: 终点停车判据（米）：**末点（最后一个航点）**的"到点/终点"判定半径。
#: 对齐 `tools/sim.py` 的 `_GOAL_STOP_M`（0.03）：比 `pursuit.arrival_m`(0.12) 更紧 ——
#: 末点若沿用 0.12 m 的到位余量，会在离终点 0.12 m 处就把它判成"已到"（配合
#: Localization 同半径的末点吸附，信念被提前拽到格心）→ FINISH 提前刹车 →
#: 物理车停在格心前 ~11 cm。实车 2026 直线标定 N10→A10 实测欠程 11 cm，而
#: 车端 scale=1.00003 说明不是里程计误差。非末点航点仍用 `arrival_m`，行为不变。
_GOAL_STOP_M = 0.03

#: ★★静止闸（2026-10-18 H8 卡死；见 _detect_stall_and_skip）：车**实际静止**时禁止
#: "绕圈兜底跳点"。没有这道闸，s 一冻结就每 12 s 跳一个航点（实车 E9/D9/D10/C10…），
#: 一路跳完 = 车一步没动却报 FINISH（假完赛）。判据 = 连续 N 拍**位姿位移 < ε 且
#: 瞬时速度 < v_eps**：N=10（10 Hz ⇒ 1 s）、ε=0.01 m/拍、v_eps=0.03 m/s
#: （低于最低蠕行命令 0.08 m/s，正常行驶/切角/绕圈都不可能命中）。
_STILL_TICKS = 10
_STILL_DISP_M = 0.01
_STILL_SPEED_MPS = 0.03

# --------------------------------------------------------------------------- #
#  ★ 自动规划：路线来源标注 + 规划入口
# --------------------------------------------------------------------------- #
#: 横幅/日志里的"路线来源"字样（验收项要求原文出现这两个词）
ROUTE_SOURCE_AUTO = "自动规划"
ROUTE_SOURCE_MANUAL = "手动配置"
#: 外部命令（/nav/waypoint）载入的路线
ROUTE_SOURCE_COMMAND = "外部命令"

#: ★t7 停站台账的**类型**口径：`SHOOT` / `SCAN` 来自 `core.state.StopKind`，
#: 末点单独标 `END` —— 终点也要停车（减速/判定），但它**绝不是一个打靶点**。
#: 旧版横幅只印 `停站点 N 个：索引:格名`，15:04 现场就把"终点 N5"误读成"第 2 个射击停站"，
#: 于是"两个打靶停站"的验收被一条错账糊过去了（实测真正生效的只有 D3*）。
STOP_KIND_END = "END"

#: 默认地图路径（相对配置文件目录 / 相对工作区，二者按序解析）
DEFAULT_MAP_PATH = "config/map.yaml"
_DEFAULT_PLAN_SECONDS = 2.0

# --------------------------------------------------------------------------- #
#  规划引擎开关（`planner.engine`）+ 规划产物"双门禁"
# --------------------------------------------------------------------------- #
#  背景：`route.planner.plan_route_tracker_safe()`（"真车能跟住"的新引擎，现役图
#  44 token / 15.05 m，追平现役手工路线）原来只有 `tools/plan_route.py --engine
#  tracker-safe` 手动跑；现在接进车上入口，并且**默认零影响**：`planner.engine`
#  缺省 `tour` ⇒ 走的还是老引擎、同一条路线。
#
#  为什么 tracker-safe 要额外过"双门禁"才允许装车执行：
#    实测教训："门禁绿 ≠ 真车过得去"（纯追踪在顶点切角，折线口径看不见），
#    但反过来"没门禁就上车"更危险（自动规划路线曾静态全绿、物理链却漏压 3 个雷、
#    还撞 D7）。所以新引擎的产物必须同时过 `tools/route_check.py`(0 error) 与
#    `route.validate_route`(0 error/0 warn)，否则按 `fallback_to_tokens` 回退现役
#    手工路线（回退 = 出车，不是停摆）。
#  为什么 tour **不加**这道门禁：它的历史行为必须逐字节不变 —— 例：`test_autoplan`
#    的 1.12「缺 mines 的图其实能解」就是被既有语义钉住的用例，给它加门禁会改变
#    "什么算规划成功"的定义（不在本开关的职责内）。
# --------------------------------------------------------------------------- #
#: 历史引擎（`planner.plan_route`）：保持既有行为逐字节一致
ENGINE_TOUR = "tour"
#: tracker-safe 新引擎（`planner.plan_route_tracker_safe`）：禁 180°/禁紧凑 U/雷直线穿心
ENGINE_TRACKER_SAFE = "tracker-safe"
DEFAULT_PLAN_ENGINE = ENGINE_TOUR
PLAN_ENGINES = (ENGINE_TOUR, ENGINE_TRACKER_SAFE)
#: 引擎名容错映射（现场手写配置的常见写法）；**不在这里的一律回退默认并告警**
_ENGINE_ALIASES = {
    "": ENGINE_TOUR, "tour": ENGINE_TOUR, "legacy": ENGINE_TOUR, "dijkstra": ENGINE_TOUR,
    "tracker-safe": ENGINE_TRACKER_SAFE, "tracker_safe": ENGINE_TRACKER_SAFE,
    "trackersafe": ENGINE_TRACKER_SAFE, "t7": ENGINE_TRACKER_SAFE,
}
#: 新引擎预算兜底（秒）：`planner.tracker_max_seconds` 缺省时用它；
#: 运行时优先读 `planner.TS_MAX_SECONDS`（引擎自己才是预算的真源）。
_FALLBACK_TRACKER_SECONDS = 5.0
#: 用户侧权威门禁脚本的位置（相对配置文件目录的父目录 / 相对工作区）
ROUTE_CHECK_REL = ("tools", "route_check.py")
#: 足迹诊断的严重级：`warn`（默认，只报不拦）| `error`（硬拦：有侵入即不放行）
FOOTPRINT_GATE_MODES = ("warn", "error")
DEFAULT_FOOTPRINT_GATE = "warn"
#: 足迹诊断的采样步长（米）：折线上按这个步长取点，yaw 取所在航段方向（保守折线口径）
_FOOTPRINT_SAMPLE_M = 0.05

# --------------------------------------------------------------------------- #
#  无人机桥洞情报（EV_BRIDGE_INFO 事件流 → 穿洞格；P_DRONE 只判链路）
# --------------------------------------------------------------------------- #
#: 两个半场（与 `tasks/drone.py` 的 `_BRIDGE_HALVES` 同名同义：lower=下半场 / upper=上半场）
BRIDGE_HALVES: tuple[str, ...] = ("lower", "upper")
#: `EV_BRIDGE_INFO` 订阅队列长度：情报是**低频**事件（每轮两半场各一条），
#: 64 足够；真被灌满时 `Subscription.dropped` 会计数并进遥测 `bridge.events_dropped`，
#: 不静默丢（"最后一条"语义下最坏结局也只是回落默认格，绝不卡死）。
_BRIDGE_SUB_MAXSIZE = 64


@dataclass(frozen=True)
class BridgeIntel:
    """一个半场的穿洞格"结论"（值 + 来源 + 证据），遥测与日志共用同一份口径。"""

    half: str = "lower"
    #: ★ 本次**采用**的穿洞格（采纳无人机情报或回落默认值，恒非空——除非配置里连默认都没有）
    cell: str = ""
    #: `route.cross_{half}_default`（回落目标；日志里用来对照"本来会走哪个洞"）
    default: str = ""
    #: 来源：`drone`（采纳情报）/ `default` / `stale` / `missing` / `offline` / `disabled`
    #: / `no_link` / `rejected`（情报非法：坏格名 / 非 K 列 / 不在本半场行段；不再要求"在候选集合"）
    source: str = "no_link"
    #: 无人机原始报格（未采纳时留证据；采纳时与 `cell` 相同）
    raw: str = ""
    #: 人类可读的原因（日志/遥测/测试断言共用）
    reason: str = ""
    #: 情报年龄（秒，`now - bridge_ts`）；拿不到 = None
    age_s: float | None = None

    @property
    def from_drone(self) -> bool:
        """是否真的采纳了无人机情报（= 与"用默认格"不同的那条路径）。"""
        return self.source == "drone"

    def describe(self) -> str:
        if self.from_drone:
            age = "（ts 差未知）" if self.age_s is None else f"（新鲜度 {self.age_s:.2f}s）"
            # reason 里可能带"池外但在行段内 ⇒ 采纳、纠正过期地图"的口径，日志要带上
            #   （现场排"这个洞怎么跟我记的候选不一样"时要能直接看出来）。
            extra = f"；{self.reason}" if self.reason else ""
            return f"{self.half} → {self.cell} ★无人机情报{age}{extra}；默认 {self.default}"
        return f"{self.half} → {self.cell}（{self.source}：{self.reason}）"

    def as_dict(self) -> dict[str, Any]:
        return {"half": self.half, "cell": self.cell, "default": self.default,
                "source": self.source, "raw": self.raw, "reason": self.reason,
                "age_s": (None if self.age_s is None else round(self.age_s, 3)),
                "from_drone": self.from_drone}


def retag_block_token(token: str, old_block: str, new_block: str) -> str:
    """把一个 token 里的格名换成新格名，**保留前后缀**（`K10*` → `K9*`、`R:K10` → `R:K9`）。

    ★ 用词边界正则而不是 `str.replace`：`K1` 会命中 `K10` 的前缀 —— 那会把洞名改错位。
    """
    old = str(old_block).strip().upper()
    new = str(new_block).strip().upper()
    return re.sub(rf"(?<![A-Za-z0-9]){re.escape(old)}(?![0-9])", new, str(token))


def _token_block(token: Any) -> str | None:
    """token → 格名（解析失败返回 None，绝不抛）。"""
    try:
        block = str(parse_token(str(token))["block"]).strip().upper()
    except Exception:                                    # noqa: BLE001  坏 token 由调用方拒
        return None
    return block or None


def rewrite_crossing_tokens(tokens: Any, half: str, old_cell: str,
                            new_cell: str) -> list[str] | None:
    """把某个半场的穿洞**三连**（`L{r} K{r} J{r}` 形状）整体挪到新行。形状不符 → `None`。

    为什么是"三连"而不是只换 K 列那一格：换洞 = 换**穿洞行**。只把 `K10` 改成 `K9` 会让
    `L10 -> K9 -> J10` 变成两段 45° 斜切，`validate_route` 的 R009（斜切入洞上限 10°）
    立刻报 error —— 那种"换了洞但过不去"的路线比不换更糟。所以进洞口/洞/出洞口必须同行。

    只认**规划器生成的形状**（`_ts`/tour 引擎的产物都是这个形状）：
      · 三连必须**同一行**，中间那格在 K 列，两侧是 K 列的**相邻列**（|Δ列| = 1）；
      · 洞格在整条 token 列表里必须**只出现一次**（重复 = 路线自折叠，语义不明 → 拒）。
    其余一律返回 `None`（手工怪形状交给调用方"原样不动"处理，宁可走默认洞）。
    """
    try:
        items = [str(t) for t in tokens]
    except TypeError:
        return None
    half = str(half).strip().lower()
    if half not in BRIDGE_HALVES:
        return None
    old = str(old_cell).strip().upper()
    new = str(new_cell).strip().upper()
    if not old or not new or old == new:
        return None
    hits = [i for i, t in enumerate(items) if _token_block(t) == old]
    if len(hits) != 1:
        return None
    i = hits[0]
    if i == 0 or i + 1 >= len(items):
        return None
    try:
        old_row, old_col = geo.block_to_index(old)
        new_row = geo.block_to_index(new)[0]
        k_col = geo.COL_LETTERS.index(CROSS_COLUMN)
    except ValueError:
        return None
    if old_col != k_col:
        return None
    band = CROSS_LOWER_ROWS if half == "lower" else CROSS_UPPER_ROWS
    if not (band[0] <= new_row + 1 <= band[1]):          # 新洞必须落在同一半场的行段里
        return None
    out = list(items)
    for j in (i - 1, i, i + 1):
        block = _token_block(out[j])
        if block is None:
            return None
        try:
            row, col = geo.block_to_index(block)
        except ValueError:
            return None
        if row != old_row:                               # 三连必须同行（换洞 = 换行）
            return None
        if j != i and (col == k_col or abs(col - k_col) != 1):
            return None                                  # 两侧必须是 K 列的邻格
        out[j] = retag_block_token(out[j], block, geo.index_to_block(new_row, col))
    return out


def resolve_plan_engine(cfg: Any = None, log: Callable[[str], None] | None = None) -> str:
    """`planner.engine` → 引擎名（`tour` / `tracker-safe`）。**永不抛异常**。

    未知值（含打错键名/打错拼写）→ 回退 `DEFAULT_PLAN_ENGINE` 并**告警**：
    宁可装上老引擎出车，也不能因为一个配置字符串写错就停在 IDLE。
    """
    if cfg is None:
        return DEFAULT_PLAN_ENGINE
    raw = cfg.get("planner.engine", DEFAULT_PLAN_ENGINE)
    engine = _ENGINE_ALIASES.get(str(raw if raw is not None else "").strip().lower())
    if engine is None:
        if log is not None:
            log(f"⚠️ planner.engine={raw!r} 不是已知引擎（{'/'.join(PLAN_ENGINES)}）"
                f"→ 按默认 {DEFAULT_PLAN_ENGINE} 规划（键名拼错不该让车停摆）")
        return DEFAULT_PLAN_ENGINE
    return engine


def plan_budget_seconds(cfg: Any, engine: str = DEFAULT_PLAN_ENGINE,
                        log: Callable[[str], None] | None = None) -> float:
    """按引擎解析规划时间预算（秒）。

    · `tour` → `planner.max_seconds`（缺省 `_DEFAULT_PLAN_SECONDS`）—— **与历史同一行代码**；
    · `tracker-safe` → `planner.tracker_max_seconds`（缺省 `planner.TS_MAX_SECONDS`）——
      新引擎本机实测 1.59~1.67 s，共用 2.0 s 会踩那类 flake（见 robot.yaml 注释）。
    """
    try:
        if engine == ENGINE_TRACKER_SAFE:
            default = _FALLBACK_TRACKER_SECONDS
            try:
                from ..route import planner as PL
                default = float(getattr(PL, "TS_MAX_SECONDS", default) or default)
            except Exception:                        # noqa: BLE001  拿不到常量就用兜底
                pass
            return float(cfg.get("planner.tracker_max_seconds", default) or default)
        return float(cfg.get("planner.max_seconds", _DEFAULT_PLAN_SECONDS)
                     or _DEFAULT_PLAN_SECONDS)
    except (TypeError, ValueError):
        if log is not None:
            log(f"⚠️ planner 时间预算不是数字 → 用默认（引擎 {engine}）")
        return _FALLBACK_TRACKER_SECONDS if engine == ENGINE_TRACKER_SAFE else _DEFAULT_PLAN_SECONDS


@dataclass
class GateResult:
    """规划产物的**双门禁**结果（`check_planned_route()` 的返回；本类只装证据、不抛异常）。"""

    ok: bool = True
    #: `tools/route_check.py` 是否真的跑到了（车端通常没部署它 → False，不算失败）
    route_check_available: bool = False
    #: route_check 退出码：0 = 无 error（有 warn 时也是 0，口径与它自己的 CLI 完全一致）
    route_check_rc: int | None = None
    route_check_errors: list[str] = field(default_factory=list)
    route_check_warns: list[str] = field(default_factory=list)
    #: 双门禁结果
    validate_errors: list[str] = field(default_factory=list)
    validate_warns: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    #: 偏移感知足迹诊断（折线口径，顾问级）
    footprint: "FootprintReport | None" = None

    @property
    def checks(self) -> str:
        rc = f"rc={self.route_check_rc}" if self.route_check_available else "不可用"
        return (f"route_check {rc} / validate {len(self.validate_errors)} error, "
                f"{len(self.validate_warns)} warn")

    @property
    def summary(self) -> str:
        """一行结论（日志、遥测、测试断言共用同一份口径）。"""
        if self.ok:
            return f"门禁通过（{self.checks}）"
        villains = ([f"validate: {e}" for e in self.validate_errors]
                    + [f"validate(warn): {w}" for w in self.validate_warns]
                    + [f"route_check: {e}" for e in self.route_check_errors])
        if self.route_check_rc not in (None, 0):
            villains.append(f"route_check 退出码 {self.route_check_rc}")
        villains += list(self.notes)
        return f"门禁未过（{self.checks}）：" + "；".join(villains[:6])


def route_check_script(cfg: Any = None) -> pathlib.Path | None:
    """定位 `tools/route_check.py`（配置文件目录的同级 → 工作区根）。找不到返回 `None`。"""
    cands: list[pathlib.Path] = []
    cdir = config_dir(cfg) if cfg is not None else None
    if cdir is not None:
        cands.append(cdir.parent.joinpath(*ROUTE_CHECK_REL))
    cands.append(workspace_root().joinpath(*ROUTE_CHECK_REL))
    for cand in cands:
        if cand.is_file():
            return cand
    return None


def load_route_check_module(cfg: Any = None) -> Any:
    """把 `tools/route_check.py` 当模块加载（它刻意**不 import dzbrain**，是独立脚本）。

    返回模块对象；找不到/加载失败 → `None`（调用方按"该门禁不可用"处理，绝不因此拒车）。
    """
    path = route_check_script(cfg)
    if path is None:
        return None
    import importlib.util                                # 惰性导入：不用门禁时不付这个成本

    try:
        spec = importlib.util.spec_from_file_location("dzbrain_tools_route_check", str(path))
        if spec is None or spec.loader is None:
            return None
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    except Exception:                                    # noqa: BLE001  车端未必有它的依赖
        return None


def _run_route_check(mod: Any, tokens: Any, map_cfg: Any,
                     cfg: Any) -> tuple[int, list[str], list[str]]:
    """在**本进程**里跑一次 route_check（等价 CLI：`--route "…" --config <robot.yaml>`）。

    ★ 三个坑（都实测过）：
      ① `route_check.FAIL/WARN/OK/INFO` 是**模块级、跨调用累积**的列表 —— 不复位会把上一次
         的结论算进这一次；
      ② 它自己按 `config/map.yaml` / `ROBOT_MAP` 找地图 —— 换了 `planner.map_path` 就会
         校验**另一张图**。所以这里用本次规划真正用的那张图的要素**覆盖**它的几何输入；
      ③ 它 `print()` 一大片人读报文 —— 输出重定向到内存，只把结论留给日志。
    """
    import contextlib
    import io

    for name in ("FAIL", "WARN", "OK", "INFO"):
        lst = getattr(mod, name, None)
        if isinstance(lst, list):
            lst.clear()
    src = str(getattr(cfg, "source", "") or "")
    cfg_path = src if src and src not in ("<memory>", "<dict>") else ""
    rcfg = (mod.load_cfg(cfg_path) if cfg_path
            else {"obstacles": [], "mines": [], "targets": [], "bridges": []})
    merged = map_cfg.get("map") if isinstance(map_cfg, dict) else None
    geo_map = merged if isinstance(merged, dict) else (map_cfg if isinstance(map_cfg, dict) else {})
    for key in ("mines", "obstacles", "targets", "bridges"):
        if geo_map.get(key):
            rcfg[key] = list(geo_map[key])
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = int(mod.run_check(" ".join(str(t) for t in tokens), rcfg))
    return rc, [str(x) for x in (getattr(mod, "FAIL", None) or [])], \
        [str(x) for x in (getattr(mod, "WARN", None) or [])]


def check_planned_route(tokens: Any, map_cfg: Any, cfg: Any,
                        log: Callable[[str], None] | None = None,
                        route_check_module: Any = None) -> GateResult:
    """规划产物**双门禁**：`validate_route`(0 error/0 warn) + `tools/route_check.py`(0 error)。

    :param route_check_module: `None` = 自动查找并加载；`False` = 显式跳过（车端未部署/测试）；
        也可以直接传一个已加载的模块（测试注入失败分支用）。
    :returns: `GateResult`。**本函数永不抛异常** —— 任何内部错误都记进 `notes`，
        并按"该门禁不可用"处理（绝不允许门禁自己把 Mission 线程炸掉）。

    语义（第 4 条）：
      · 任一 error（或 `validate_route` 的 warn）→ `ok=False` ⇒ 调用方**不得**执行这条路线；
      · `tools/route_check.py` 找不到/加载失败 → **不算失败**（车端 `tools/` 不随包部署，
        这是已知事实，`docs/部署清单` 里只有 3 个 tools 脚本），但 `notes` 里留痕；
        此时门禁退化为"内包 `validate_route`"。决策依据：车端宁可少一道**用户侧**检查，
        也不能因为缺一个开发脚本就拒绝自动规划（回退到手工路线仍是安全态）。
    """
    res = GateResult()
    # ---- ① 内包 validate_route（纯函数、无 ROS、dzbrain 包内一定有） ----
    try:
        from ..route.parser import parse_mission
        from ..route.validate import validate_route
        route = parse_mission([str(t) for t in tokens], name="planner-gate")
        for issue in validate_route(route, map_cfg=map_cfg, cfg=cfg):
            bucket = (res.validate_errors if getattr(issue, "level", "") == "error"
                      else res.validate_warns)
            bucket.append(str(issue))
    except Exception as exc:                             # noqa: BLE001
        res.validate_errors.append(f"{type(exc).__name__}: {exc}")

    # ---- ② tools/route_check.py（用户侧权威门禁；规则不复制第二份） ----
    if route_check_module is False:
        mod = None
        skip_note = ("route_check 显式跳过（调用方传 route_check_module=False；"
                     "车端不部署 tools/ 时等价）→ 本次只用内置 validate_route 门禁")
    else:
        mod = route_check_module if route_check_module is not None else load_route_check_module(cfg)
        skip_note = ("route_check 不可用（未找到/加载失败 tools/route_check.py；"
                     "车端不部署它）→ 本次只用内置 validate_route 门禁")
    if mod is None:
        res.notes.append(skip_note)
        if log is not None:
            log("⚠️ " + skip_note)
    else:
        try:
            rc, errs, warns = _run_route_check(mod, tokens, map_cfg, cfg)
            res.route_check_available = True
            res.route_check_rc = rc
            res.route_check_errors = errs
            res.route_check_warns = warns
        except Exception as exc:                         # noqa: BLE001
            res.notes.append(f"route_check 执行异常（{type(exc).__name__}: {exc}）→ 该门禁忽略")

    # ---- ③ 参考点偏移感知的足迹诊断（顾问级；`planner.footprint_gate: error` 才硬拦） ----
    report = footprint_diagnostic(tokens, map_cfg, cfg)
    res.footprint = report
    if report.risks:
        res.notes.append("足迹诊断：" + "；".join(report.risks[:4]))
        if log is not None:
            log("⚠️ " + report.summary)

    res.ok = ((not res.validate_errors) and (not res.validate_warns)
              and res.route_check_rc in (None, 0))
    # ★ 足迹诊断的严重级由 `planner.footprint_gate` 决定（默认 warn = 只报不拦"现役风格"）。
    mode = str(cfg.get("planner.footprint_gate", DEFAULT_FOOTPRINT_GATE)
               or DEFAULT_FOOTPRINT_GATE).strip().lower()
    if mode not in FOOTPRINT_GATE_MODES:
        mode = DEFAULT_FOOTPRINT_GATE
    if report.risks and mode == "error":
        res.ok = False
    return res


@dataclass
class FootprintReport:
    """**参考点偏移感知**的车体足迹诊断（折线口径；顾问级：默认只报不拦）。

    ★ 为什么是"顾问级"而不是硬门禁：折线口径看不到纯追踪的**拐角切内**，而它按
      "航段方向"给 yaw —— 在 `J8→J7→I7` 这种"垂直进、水平出"的拐角上，它会把
      "车头正对挡板"那一瞬间算进去（保守 ~5 cm），而真值轨迹里车到那一点时 yaw 已经
      转到 ~45°（实测侵入只有 5 mm 级）。所以本报告用来**暴露风险**，授权判据仍是
      `route_check` + `validate_route`（真值轨迹级结论见历史证据目录）。
    """

    offset_m: float = 0.0
    samples: int = 0
    min_barrier_m: float = float("inf")
    min_barrier_where: str = ""
    min_obstacle_m: float = float("inf")
    min_obstacle_where: str = ""
    min_target_m: float = float("inf")
    min_target_where: str = ""
    risks: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.risks

    @property
    def summary(self) -> str:
        if not self.samples:
            return "足迹诊断：未采样（路线为空）"
        text = (f"足迹诊断（offset {self.offset_m:+.3f} m / {self.samples} 采样 / "
                f"**折线口径=保守上界**）：挡板 {self.min_barrier_m:+.4f}")
        if math.isfinite(self.min_obstacle_m):
            text += f" / 障碍 {self.min_obstacle_m:+.4f}（{self.min_obstacle_where}）"
        if math.isfinite(self.min_target_m):
            text += f" / 靶板 {self.min_target_m:+.4f}（{self.min_target_where}）"
        text += " m（真值轨迹级复核见 sim/离线足迹链；折线口径会高估侵入）"
        return text + ("；⚠️ " + "；".join(self.risks) if self.risks else "；✅ 无侵入")


def vehicle_geometry(cfg: Any) -> dict[str, float]:
    """`vehicle.*` 三个尺寸 + **参考点偏移**（读不到就退 SCHEMA 默认，绝不抛异常）。

    ★ 键路径口径（实测的坑，必须写明）：尺寸/偏移的真源写在
      `config/robot.yaml` 的 **`/dzactuator: ros__parameters: vehicle:`** 段里
      （与 `length_m`/`width_m` 同一个段），`Config._flatten()` 之后是
      **`dzactuator.vehicle.*`**；而 `SCHEMA` 里的规范名是 `vehicle.*`。
      所以这里**先读带段前缀的真源键，再退规范名**：
      `cfg.get("vehicle.length_m", 0.28)` 这类写法（带非 None 默认值）在键不在 `_flat` 里时
      会**直接返回默认值**、看不到 YAML 的改动 —— 现场量了新尺寸却读不到，是最坏的一类 bug。
    """

    def _f(keys: tuple[str, ...], fallback: float) -> float:
        for key in keys:
            try:
                val = cfg.get(key)
            except Exception:                            # noqa: BLE001  配置缺失/关键键报错
                continue
            if val is None:
                continue
            try:
                return float(val)
            except (TypeError, ValueError):
                continue
        return fallback

    return {
        "length_m": _f(("dzactuator.vehicle.length_m", "vehicle.length_m"), 0.28),
        "width_m": _f(("dzactuator.vehicle.width_m", "vehicle.width_m"), 0.135),
        "offset_m": _f(("dzactuator.vehicle.ref_offset_m", "vehicle.ref_offset_m"), 0.0),
    }


def _map_geometry(map_cfg: Any) -> dict:
    """兼容 `{'map': {...}}` 与扁平两种地图写法，返回要素字典。"""
    merged = map_cfg.get("map") if isinstance(map_cfg, dict) else None
    return merged if isinstance(merged, dict) else (map_cfg if isinstance(map_cfg, dict) else {})


def _merged_target_rect(geo_map: dict) -> tuple[float, float, float, float] | None:
    """跨格靶板的**合并矩形**（心 + 两半边长）—— 现场口径：一块 35×70 板，不是两格。"""
    cells = [str(c) for c in (geo_map.get("targets") or [])]
    if not cells:
        return None
    try:
        pts = [geo.block_to_meters(c) for c in cells]
    except ValueError:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    hx = (max(xs) - min(xs)) / 2.0 + geo.BLOCK_SIZE / 2.0
    hy = (max(ys) - min(ys)) / 2.0 + geo.BLOCK_SIZE / 2.0
    return ((max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0, hx, hy)


def footprint_diagnostic(tokens: Any, map_cfg: Any, cfg: Any) -> FootprintReport:
    """折线口径的**偏移感知**足迹诊断（挡板 / 障碍 / 靶板三类实体）。**永不抛异常**。

    采样：沿相邻航点连线按 `_FOOTPRINT_SAMPLE_M` 步长取点，`yaw` = 该航段方向；
    再对每个航点用"进/出航段方向"各采一次（等价覆盖折线拐角的两端状态）。
    """
    from ..route import physics as ph                       # 惰性导入：不跑诊断就不付成本

    rep = FootprintReport()
    body = vehicle_geometry(cfg)
    rep.offset_m = body["offset_m"]
    geo_map = _map_geometry(map_cfg)
    try:
        route = parse_mission([str(t) for t in tokens], name="footprint")
    except Exception as exc:                                # noqa: BLE001
        rep.risks.append(f"足迹诊断无法解析路线（{type(exc).__name__}: {exc}）")
        return rep
    wps = list(route.waypoints)
    if len(wps) < 2:
        return rep

    # ---- 场地实体的物理形状（与 sim/route_check 同源公式） ----
    try:
        bar_x, bar_y0, bar_y1 = ph.barrier_centerline(
            col_from=str(geo_map.get("barrier_col_from") or "D"),
            col_to=str(geo_map.get("barrier_col_to") or "N"),
            row_hi=int(geo_map.get("barrier_row_hi") or 6),
            row_lo=int(geo_map.get("barrier_row_lo") or 7))
        barrier_w = float(cfg.get("field.barrier_width_m", 0.02) or 0.02)
    except (TypeError, ValueError):
        bar_x, bar_y0, bar_y1, barrier_w = 1.75, 0.0, 3.85, 0.02
    barrier_poly = ph.corner_poly(bar_x, (bar_y0 + bar_y1) / 2.0,
                                  barrier_w / 2.0, (bar_y1 - bar_y0) / 2.0)
    obs = []
    for cell in (geo_map.get("obstacles") or []):
        try:
            ox, oy = geo.block_to_meters(str(cell))
        except ValueError:
            continue
        obs.append((str(cell), ox, oy))
    obs_r = float(cfg.get("field.obstacle_radius_m", 0.1275) or 0.1275)
    target_rect = _merged_target_rect(geo_map)

    def _state(px: float, py: float, yaw: float, where: str) -> None:
        rep.samples += 1
        poly = ph.body_corners(px, py, yaw, length_m=body["length_m"],
                               width_m=body["width_m"], offset_m=body["offset_m"])
        depth = ph.polygon_overlap_depth(poly, barrier_poly)
        gap = 0.0 if depth > 0 else ph.polygon_distance_m(poly, barrier_poly)
        if gap - 1e-12 < rep.min_barrier_m:
            rep.min_barrier_m = gap
            rep.min_barrier_where = where
        if depth > 0.0:
            rep.risks.append(f"车体侵入挡板 {depth * 1000:.1f} mm @ {where}"
                             f"（yaw {math.degrees(yaw):.0f}°）")
        for name, ox, oy in obs:
            gap_i = ph.body_obstacle_clearance_m(
                px, py, yaw, ox, oy, obstacle_radius_m=obs_r,
                length_m=body["length_m"], width_m=body["width_m"],
                offset_m=body["offset_m"])
            if gap_i < rep.min_obstacle_m:
                rep.min_obstacle_m = gap_i
                rep.min_obstacle_where = name
            if gap_i <= 0.0:
                rep.risks.append(f"车体侵入障碍 {name}（{-gap_i * 1000:.1f} mm）@ {where}")
        if target_rect is not None:
            tc, ty, hx, hy = target_rect
            tpoly = ph.corner_poly(tc, ty, hx, hy)
            td = ph.polygon_overlap_depth(poly, tpoly)
            tg = 0.0 if td > 0 else ph.polygon_distance_m(poly, tpoly)
            if tg - 1e-12 < rep.min_target_m:
                rep.min_target_m = tg
                rep.min_target_where = where
            if td > 0.0:
                rep.risks.append(f"车体侵入靶板 {td * 1000:.1f} mm @ {where}")

    for i in range(len(wps) - 1):
        a, b = wps[i], wps[i + 1]
        dx, dy = b.x - a.x, b.y - a.y
        dist = math.hypot(dx, dy)
        if dist <= 1e-9:
            continue
        yaw = math.atan2(dy, dx)
        n = max(1, int(dist / _FOOTPRINT_SAMPLE_M))
        for k in range(n + 1):
            u = k / float(n)
            _state(a.x + u * dx, a.y + u * dy, yaw,
                   f"{a.block}->{b.block} t={u:.2f}")
    # 拐角两端：进/出航段方向各算一次（折线口径的保守补扫）
    for i in range(1, len(wps) - 1):
        p, q, r = wps[i - 1], wps[i], wps[i + 1]
        for (src, tag) in ((p, "进"), (r, "出")):
            dx, dy = src.x - q.x, src.y - q.y
            if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                _state(q.x, q.y, math.atan2(-dy, -dx), f"{q.block}拐角({tag})")
    return rep


class AutoPlanBlocked(RuntimeError):
    """`planner.fallback_to_tokens=false` 且规划失败 —— 停在 IDLE，**绝不**带病出车。"""


@dataclass
class PlanOutcome:
    """一次自动规划的结果（**不抛异常**，失败信息全在 `error` 里，便于日志与测试断言）。"""

    ok: bool = False
    tokens: list[str] = field(default_factory=list)
    elapsed_ms: float = 0.0
    error: str = ""                      # 失败时 "<异常类型>: <消息>"
    map_path: str = ""                   # 实际用的地图文件（已解析成绝对路径）
    allow_reverse: bool = False
    route_len_m: float = 0.0
    reverse_segments: int = 0
    cost: float = 0.0
    #: 这次用的引擎（`tour` / `tracker-safe`）
    engine: str = DEFAULT_PLAN_ENGINE
    #: 双门禁结论（只有 `tracker-safe` 会跑门禁；tour 恒为"未跑 = True"）
    gate_ok: bool = True
    gate_detail: str = ""
    #: 参考点偏移感知的足迹诊断（一行结论；折线口径，顾问级）
    footprint: str = ""
    #: 足迹诊断是否有侵入（决定 `planner.footprint_gate: error` 时是否硬拦）
    footprint_ok: bool = True
    #: 本次**实际采用**的穿洞格（来自 `PlanResult.crossing`；失败时为空）
    cross_lower: str = ""
    cross_upper: str = ""
    #: ★t7 本路线**射击**停站格名（按路线序；双停站 = `[主停站, 对侧停站]`）。
    #: 唯一口径 = token 里的 `*`（`ensure_second_shoot_stop()` 收口后），不另记一份账。
    shoot_stops: tuple[str, ...] = ()
    #: ★t7 事后补标的对侧停站（非空 = 引擎自己没落标，由收口函数补上的；见其 docstring）
    second_stop_added: str = ""

    @property
    def source(self) -> str:
        return ROUTE_SOURCE_AUTO if self.ok else ""

    @property
    def mode(self) -> str:
        return "允许倒车" if self.allow_reverse else "无倒车"

    @property
    def engine_line(self) -> str:
        """日志/横幅用的引擎行（含门禁结论，缺省引擎不额外占字）。"""
        text = f"引擎 {self.engine}"
        if self.engine != DEFAULT_PLAN_ENGINE:
            text += f"（门禁{'✅通过' if self.gate_ok else '❌未过'}：{self.gate_detail}）"
        return text


def workspace_root() -> pathlib.Path:
    """工作区根目录（`<ws>/src/dzbrain/dzbrain/tasks/mission.py` → `parents[4]`）。

    ⚠️ 这里是 **parents[4]** 不是 [3]：`tasks/` 比 `main.py` 深一层
    （`main.py` 在 `<ws>/src/dzbrain/dzbrain/`，其 `parents[3]` 才是 `<ws>`）。
    取错一位会让"相对工作区"的地图候选变成 `<ws>/src/config/map.yaml` —— 实测
    `python -m dzbrain --plan-only` 报 `FileNotFoundError: 地图文件不存在:
    D:\\...\\dzrobot_v2\\src\\config\\map.yaml`，正好把这个 off-by-one 暴露出来。
    """
    return pathlib.Path(__file__).resolve().parents[4]


def config_dir(cfg: Any) -> pathlib.Path | None:
    """配置文件所在目录（`Config.from_dict` 造的配置没有真实文件 → None）。"""
    src = str(getattr(cfg, "source", "") or "")
    if not src or src in ("<memory>", "<dict>"):
        return None
    return pathlib.Path(src).parent


def resolve_map_path(cfg: Any = None, spec: str | None = None) -> pathlib.Path:
    """地图路径解析顺序：**绝对路径 → 相对配置文件目录 → 相对工作区**。

    三个候选按序 `is_file()`；都不存在时返回**工作区候选**（调用方据此报"地图文件不存在"，
    错误信息里能看到实际尝试的路径）。本函数不抛异常、也不读文件内容。
    """
    text = spec if spec is not None else (cfg.get("planner.map_path") if cfg is not None else None)
    text = str(text or DEFAULT_MAP_PATH)
    raw = pathlib.Path(text).expanduser()
    if raw.is_absolute():
        return raw
    cands: list[pathlib.Path] = []
    cdir = config_dir(cfg) if cfg is not None else None
    if cdir is not None:
        cands.append(cdir / raw)
    cands.append(workspace_root() / raw)
    for cand in cands:
        if cand.is_file():
            return cand
    return cands[-1]


def _read_map_yaml(path: pathlib.Path) -> dict:
    """读地图 YAML（失败即抛，由 `plan_route_from_config` 统一转成 `PlanOutcome.error`）。"""
    import yaml                                   # 车上/本地都有 PyYAML（core/config.py 同款）

    if not path.is_file():
        raise FileNotFoundError(f"地图文件不存在: {path}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"地图文件不是 YAML 映射（顶层应为 dict）: {path}")
    return raw


def plan_route_from_config(cfg: Any, *, map_path: str | None = None,
                           log: Callable[[str], None] | None = None,
                           cross_lower: str | None = None,
                           cross_upper: str | None = None) -> PlanOutcome:
    """按 `planner.*` 配置规划一条路线 —— **永不抛异常**（失败 → `ok=False` + `error`）。

    调用契约（与规划器 API 契约一致，本文件**只消费不修改** `route/`）：
        `plan_route(map_cfg, cfg, allow_reverse=bool, max_seconds=float, source=str) -> PlanResult`
        `plan_route_tracker_safe(map_cfg, cfg, *, max_seconds=float, source=str) -> PlanResult`
        `PlanResult.tokens: list[str]` / `.elapsed_ms: float` —— 失败抛 `PlannerError`。

    :param cross_lower: 下半场穿洞格（**新鲜无人机情报**），`None` = 用
        `route.cross_lower_default`（与接线前逐字节一致）。后规划器的硬门 = **K 列 +
        本半场行段**（候选集合只用来挑缺省洞），行段外/非 K 列/坏格名才抛 `PlannerError`；
        调用方（MissionTask）仍先做一次同口径校验，绝不把坏格喂进来。
    :param cross_upper: 同上，上半场。

    引擎开关（`planner.engine`，默认 `tour`）：
      · `tour`（缺省）→ `plan_route(...)` + `planner.max_seconds`：**与历史逐字节一致**；
      · `tracker-safe` → `plan_route_tracker_safe(...)` + `planner.tracker_max_seconds`，
        且产物**必须过双门禁**（`tools/route_check.py` 0 error + `validate_route` 0-0），
        否则本次返回 `ok=False`（调用方按 `fallback_to_tokens` 回退现役路线）。

    ★ 宽捕获 `Exception`：任何规划器内部错误都必须变成"可回退的失败"，
      绝不允许把 MissionTask 一起炸掉（宁可漏规划，不可卡死）。
    """
    engine = resolve_plan_engine(cfg, log)
    want_reverse = bool(cfg.get("planner.allow_reverse", False))
    # 只在"真的采纳了情报"时把格透传下去；`None` 走的还是规划器自己的默认路径。
    want_lower = str(cross_lower or "").strip().upper() or None
    want_upper = str(cross_upper or "").strip().upper() or None
    try:
        budget = float(cfg.get("planner.max_seconds", _DEFAULT_PLAN_SECONDS)
                       or _DEFAULT_PLAN_SECONDS)
    except (TypeError, ValueError):
        budget = _DEFAULT_PLAN_SECONDS
    if engine != DEFAULT_PLAN_ENGINE:
        budget = plan_budget_seconds(cfg, engine, log)     # 新引擎预算独立（见 robot.yaml）
    try:
        pivot_penalty = float(cfg.get("planner.pivot_penalty", 0.0) or 0.0)
    except (TypeError, ValueError):
        pivot_penalty = 0.0
    path = resolve_map_path(cfg, map_path if map_path is not None
                            else cfg.get("planner.map_path"))
    map_text = str(path)
    gate = GateResult()
    gate_ran = False                     # 只有 tracker-safe 会跑门禁（tour 保持历史行为）
    try:
        raw = _read_map_yaml(path)
        from ..route import planner as PL          # ★ 惰性导入：无规划器时也能 import 本模块
        if engine == ENGINE_TRACKER_SAFE:
            res = PL.plan_route_tracker_safe(raw, cfg, cross_lower=want_lower,
                                             cross_upper=want_upper,
                                             max_seconds=budget, source=map_text)
        else:
            res = PL.plan_route(raw, cfg, cross_lower=want_lower, cross_upper=want_upper,
                                allow_reverse=want_reverse,
                                pivot_penalty=pivot_penalty,
                                max_seconds=budget, source=map_text)
        tokens = [str(t) for t in (res.tokens or [])]
        if not tokens:
            raise ValueError("规划器返回了空 token 列表")
        # ★★ t7（用户口径"自动规划必须保留所有 `*` 停站"）：靶板一面蓝一面红、蓝面朝向
        #   每局随机且恒定，只打蓝、打红扣分 ⇒ 路线必须在**靶板两侧各留一个射击停站**。
        #   但"落第二颗星"在两个引擎里是**非对称**的：
        #     · tracker-safe：编译期顺手落标（`_compile`），落不上只 warn；
        #     · tour（缺省引擎）：`plan_route()` **压根没有这段**，只标主停站。
        #   所以在这里做**引擎无关**的收口：用规划器同一份判据（`second_shoot_stop`）算出
        #   对侧候选，候选确实在路线上就把那颗星补上 —— **零路径改动**（token 数/顺序/里程
        #   一个都不变，只多一个"到站停车"）。补不上就**报警**，绝不静默单停站。
        tokens, _second_added, _second_note = PL.ensure_second_shoot_stop(
            tokens, raw, cfg, source=map_text)
        _shoot_stops = tuple(parse_token(t)["block"] for t in tokens if parse_token(t)["shoot"])
        if log is not None:
            if _second_added:
                log(f"★ 双射击停站已补齐：{_second_note}"
                    f"（对侧互备；零路径改动 ⇒ token 数/里程不变）")
            elif len(_shoot_stops) < 2:
                log(f"⚠️ 自动规划路线只有 {len(_shoot_stops)} 个射击停站（{_second_note}）"
                    f"：靶板蓝面朝另一侧那局会打不到")
            else:
                log(f"双射击停站：{' + '.join(_shoot_stops)}（引擎自带，未改动）")
        crossing = dict(getattr(res, "crossing", None) or {})
        if want_lower or want_upper:
            if log is not None:
                log(f"穿洞格（无人机情报）：下半场 {want_lower or '（默认）'} / "
                    f"上半场 {want_upper or '（默认）'} → 规划产物 crossing={crossing}")
        gate_detail = f"缺省引擎 {DEFAULT_PLAN_ENGINE} 不跑门禁（历史行为逐字节不变）"
        footprint_text = ""
        footprint_ok = True
        if engine == ENGINE_TRACKER_SAFE:
            gate = check_planned_route(tokens, raw, cfg, log=log)
            gate_ran = True
            gate_detail = gate.summary
            if log is not None:
                log(f"规划门禁[{engine}]：{gate_detail}")
            if not gate.ok:
                raise ValueError(f"新引擎 {engine} 产出未过门禁：{gate_detail}")
        # ★ 足迹诊断对**两种引擎**都算（tour 也报，但只作顾问；`--plan-only` 同源可见）：
        #   现场卷尺实测的 +7.5 cm 参考点偏移一旦计入，挡板余量就是报告的核心读数。
        fp = (gate.footprint if (engine == ENGINE_TRACKER_SAFE and gate.footprint is not None)
              else footprint_diagnostic(tokens, raw, cfg))
        if fp is not None:
            footprint_text = fp.summary
            footprint_ok = fp.ok
            if log is not None:
                log(("⚠️ " if not fp.ok else "") + footprint_text)
        return PlanOutcome(
            ok=True, tokens=tokens, elapsed_ms=float(getattr(res, "elapsed_ms", 0.0) or 0.0),
            error="", map_path=map_text, allow_reverse=want_reverse,
            route_len_m=float(getattr(res, "polyline_m", 0.0) or 0.0),
            reverse_segments=int(getattr(res, "reverse_segments", 0) or 0),
            cost=float(getattr(res, "cost", 0.0) or 0.0),
            engine=engine, gate_ok=gate.ok, gate_detail=gate_detail,
            footprint=footprint_text, footprint_ok=footprint_ok,
            cross_lower=str(crossing.get("lower") or ""),
            cross_upper=str(crossing.get("upper") or ""),
            shoot_stops=_shoot_stops,
            second_stop_added=_second_added,
        )
    except Exception as exc:                       # noqa: BLE001  ★ 规划失败一律回退，不崩
        msg = f"{type(exc).__name__}: {exc}"
        if log is not None:
            log(f"规划失败（将按 fallback_to_tokens 处理）：map={map_text} 引擎 {engine} "
                f"预算 {budget:g}s 穿洞(下/上)={want_lower or '默认'}/{want_upper or '默认'} {msg}")
        # ★ 门禁真的跑过且没过时，把证据留在遥测里（`gate_ok=False` + 报文）；
        #   其它失败（坏图/超时）门禁根本没跑到 ⇒ `gate_ok` 保持 True + detail 为空。
        return PlanOutcome(ok=False, tokens=[], elapsed_ms=0.0, error=msg,
                           map_path=map_text, allow_reverse=want_reverse, engine=engine,
                           gate_ok=(gate.ok if gate_ran else True),
                           gate_detail=(gate.summary if gate_ran else ""))


# --------------------------------------------------------------------------- #
#  ★T27 段规划：步进循环的"点对点"路线（未知格=不可通行，但**不是**物理障碍）
# --------------------------------------------------------------------------- #
#  背景（docs/step_loop.md §3）：无人机逐帧下发布局 v2，车端按 target_cell 把整趟
#  拆成若干**段**；每段只需要"从当前格走到目标格"，不要求覆盖雷、不要求射击停站。
#
#  ★ 与整线 plan_route_tracker_safe() 的**刻意差异**（本卡的关键裁决，见报告）：
#    整线引擎把"雷必须直线穿心 / 隔板邻行不许 90° 转弯 / 桥洞入口邻格不许横穿"都
#    升成了**硬失败**。这些约束是"覆盖型整线"的策略，不是"点对点移动"的物理极限：
#    本图实测（第五张图）从 N10 出发，整线 tracker-safe 只能到东侧 12 个格
#    （M7→L7→K7→J7 的转弯被"隔板邻行转弯"永久否决）⇒ 若段规划也照搬硬失败，
#    无人机认为"可达"的前沿点，车端会永远 WAIT_INFO。故段规划只保留三条**硬**约束：
#      ① 物理净空（_TSGrid.move_ok：障碍/靶板/挡板/桥洞入口按车身几何算）；
#      ② 禁 >90° 转角（180° 折返只能靠 PIVOT，纯追踪跟不住）；
#      ③ 禁"紧凑 U 形折返"（两条反向腿之间只隔 1 格，会被纯追踪整段切掉）。
#    其余策略性否决（雷格转弯/隔板邻行转弯/入口邻格/进洞扫掠）**降级为代价**
#    （+_SEG_SOFT_BAN_COST_M 米）：既不会无解，又会优先选安全几何；命中处写进
#    SegmentPlan.warnings，现场能在日志里看见"这段路有一处整线引擎会否决的转弯"。
_SEG_MAX_TURN_DEG = 90.0
#: 段规划转角权重（米/90°），与 tracker-safe 的 TS_TURN_W 同量级
_SEG_TURN_W_M = 0.10
#: 整线引擎的策略性硬否决在段规划里的降级代价（米）：足够大 ⇒ 有替代路就绕开
_SEG_SOFT_BAN_COST_M = 2.0


@dataclass
class SegmentPlan:
    """一次段规划的结果（**不抛异常**；失败原因全在 error 里）。

    与 PlanOutcome 分开的理由：段规划**不是**整线任务规划 —— 没有射击停站/双门禁/
    穿洞三连那些字段，硬塞进 PlanOutcome 会让"失败回退 route.tokens"的语义混进来。
    """

    ok: bool = False
    tokens: list[str] = field(default_factory=list)
    cells: list[str] = field(default_factory=list)
    from_cell: str = ""
    target_cell: str = ""
    engine: str = DEFAULT_PLAN_ENGINE
    route_len_m: float = 0.0
    cost: float = 0.0
    elapsed_ms: float = 0.0
    error: str = ""
    warnings: tuple[str, ...] = ()

    @property
    def single_cell(self) -> bool:
        """目标格 == 当前格（无需发段；步进器据此直接判段完成）。"""
        return bool(self.ok) and len(self.cells) <= 1

    @property
    def engine_line(self) -> str:
        if self.engine == DEFAULT_PLAN_ENGINE:
            return f"引擎 {self.engine}"
        return f"引擎 {self.engine}（段规划：物理净空硬约束，策略性转弯降级为代价）"


def heading_dir_from_yaw(yaw: float) -> tuple[int, int]:
    """地图 yaw → 栅格行进方向（4 邻域单位步）。与 planner._ts_map_yaw 互逆。"""
    dx = math.cos(float(yaw))
    dy = math.sin(float(yaw))
    if abs(dx) >= abs(dy):
        return (-1, 0) if dx >= 0.0 else (1, 0)
    return (0, -1) if dy >= 0.0 else (0, 1)


def _seg_turn_deg(a: tuple[int, int], b: tuple[int, int]) -> float:
    """两个行进方向之间的转角（度，∈ [0,180]）；口径与 planner._ts_turn_deg 相同。"""
    return abs(math.degrees(geo.normalize_angle(math.atan2(b[0], b[1])
                                                - math.atan2(a[0], a[1]))))


def _seg_unit(v: tuple[int, int]) -> tuple[int, int]:
    g = math.gcd(abs(int(v[0])), abs(int(v[1]))) or 1
    return (int(v[0]) // g, int(v[1]) // g)


def plan_segment_route(map_cfg: Any, cfg: Any = None, *,
                       from_cell: str, target_cell: str,
                       heading_dir: tuple[int, int] | None = None,
                       engine: str | None = None,
                       blocked_extra: Iterable[str] = (),
                       source: str = "") -> SegmentPlan:
    """段规划：from_cell → target_cell 的最短可执行栅格路线（**永不抛异常**）。

    :param map_cfg: {"map": {...}}（与整线规划器同形状）；未知格的"不可通行"由
        blocked_extra 传，**不要**塞进 obstacles —— 未知格不是实体水桶，若按障碍
        加物理净空势场，一条两侧都是未知格、只有一格宽的 known 走廊会**合法但规划不出**。
    :param heading_dir: 车当前车头朝向（栅格单位步）；None = 未知，首步任意（测试/复位后）。
    :param blocked_extra: 额外禁止进入的格（步进循环传"不在 known 里的格"）。
    :param engine: 缺省按 planner.engine；tracker-safe 用物理净空扫掠，tour 用基础
        网格判据（无物理扫掠）—— 两者都禁 180° 与紧凑 U。
    """
    t0 = time.perf_counter()
    eng = str(engine or resolve_plan_engine(cfg)).strip().lower()
    if eng not in PLAN_ENGINES:
        eng = DEFAULT_PLAN_ENGINE
    out = SegmentPlan(engine=eng)
    try:
        from ..route import planner as PL

        try:
            spec = PL.load_map(map_cfg, source=source)
        except Exception as exc:                       # noqa: BLE001 坏图=可等待的失败
            out.error = f"地图非法：{type(exc).__name__}: {exc}"
            return out
        try:
            srow, scol = geo.block_to_index(str(from_cell).strip().upper())
            grow, gcol = geo.block_to_index(str(target_cell).strip().upper())
        except ValueError as exc:
            out.error = f"起点/目标格名非法：{exc}"
            return out
        start = (srow + 1, scol)
        goal = (grow + 1, gcol)
        out.from_cell = geo.index_to_block(srow, scol)
        out.target_cell = geo.index_to_block(grow, gcol)
        if start == goal:                              # 已站在目标格：空段（无需执行）
            out.ok = True
            out.cells = [out.from_cell]
            out.elapsed_ms = (time.perf_counter() - t0) * 1000.0
            return out

        # ---- 建栅格：桥洞行只认 K 列格（layout.bridges 已是可通行桥洞）----
        bridge_rows: list[int] = []
        for cell in spec.bridges:
            try:
                br, bc = geo.block_to_index(str(cell))
            except ValueError:
                continue
            if bc == geo.COL_LETTERS.index("K"):
                bridge_rows.append(br + 1)
        grid = PL.Grid(spec, bridge_rows=bridge_rows, allow_reverse=False)
        # ★ 未知格只进 blocked（通行性），不进 spec.obstacles（物理净空/势场）
        blocked: set[tuple[int, int]] = set()
        for cell in blocked_extra or ():
            try:
                r, c = geo.block_to_index(str(cell).strip().upper())
            except ValueError as exc:
                out.error = f"blocked_extra 里的格名非法：{cell!r}（{exc}）"
                return out
            blocked.add((r + 1, c))
        grid.blocked |= blocked
        if start in blocked:
            out.error = f"当前格 {out.from_cell} 在禁止集合里（未知格不能作为起点）"
            return out
        if not grid.passable(*start):
            out.error = f"当前格 {out.from_cell} 不可通行（障碍/靶板）"
            return out
        if goal in blocked:
            out.error = f"目标格 {out.target_cell} 不在 known 里（车绝不进未知格）"
            return out
        if not grid.passable(*goal):
            out.error = f"目标格 {out.target_cell} 不可通行（障碍/靶板）"
            return out
        ts = PL._TSGrid(spec=spec, grid=grid, phys=PL._phys_params(cfg))  # noqa: SLF001

        def _move_ok(a: tuple[int, int], b: tuple[int, int]) -> bool:
            if eng == ENGINE_TRACKER_SAFE:
                return ts.move_ok(a, b)
            return grid.move_ok(a[0], a[1], b[0], b[1])

        # ---- 朝向感知 Dijkstra（状态 = (格, 到达朝向, 上一腿朝向)）----
        dirs = PL.TS_ORTHO
        first = None
        if heading_dir is not None:
            cand = (int(heading_dir[0]), int(heading_dir[1]))
            if cand in dirs:
                first = cand
        def _search(dirs0: tuple) -> tuple[tuple | None, dict, dict]:
            dist0: dict[tuple, float] = {}
            came0: dict[tuple, tuple | None] = {}
            pq0: list[tuple[float, int, tuple]] = []
            cnt = 0
            for st in [(start, d, None) for d in dirs0]:
                dist0[st] = 0.0
                came0[st] = None
                heapq.heappush(pq0, (0.0, cnt, st))
                cnt += 1
            while pq0:
                value, _order, st = heapq.heappop(pq0)
                if value > dist0.get(st, float("inf")) + 1e-11:
                    continue
                cell, h, h_prev = st
                if cell == goal:
                    return st, dist0, came0
                for v in dirs:
                    nxt = (cell[0] + v[0], cell[1] + v[1])
                    if not _move_ok(cell, nxt):
                        continue
                    deg = 0.0
                    extra = 0.0
                    if h is not None:
                        deg = _seg_turn_deg(h, v)
                        if deg > _SEG_MAX_TURN_DEG + 1e-9:      # 180°：硬禁
                            continue
                        if (h_prev is not None and deg > 45.0
                                and _seg_turn_deg(h_prev, h) > 45.0
                                and _seg_unit((-h_prev[0], -h_prev[1])) == v):
                            continue                            # 紧凑 U：硬禁
                        ok_turn, gate_deg, gate_extra = ts.turn_gate(cell, h_prev, h, v)
                        if ok_turn:
                            extra += float(gate_extra)
                        else:                                   # 策略性否决 → 降级为代价
                            extra += _SEG_SOFT_BAN_COST_M
                        deg = float(gate_deg)
                    nd = value + geo.BLOCK_SIZE + _SEG_TURN_W_M * (deg / 90.0) + extra
                    ns = (nxt, v, h)
                    if nd < dist0.get(ns, float("inf")) - 1e-11:
                        dist0[ns] = nd
                        came0[ns] = st
                        cnt += 1
                        heapq.heappush(pq0, (nd, cnt, ns))
            return None, dist0, came0

        goal_state, dist, came = _search(dirs if first is None else (first,))
        turned_around = False
        if goal_state is None and first is not None:
            # ★ 起点车头朝向与唯一可行首步相反（180°）时，若直接判"无解"会永久 WAIT_INFO
            #   （段边界上很常见：上一段从西往东开，下一段要往西回）。纯追踪无法从静止
            #   直接横移/掉头，但车可以原地转/小圆弧掉头 —— 故允许"首步任意方向"再搜一次，
            #   并如实留 warning（现场需确认该格前方有 ~0.3 m 掉头空间）。
            goal_state, dist, came = _search(dirs)
            turned_around = True
        if goal_state is None:
            out.error = (f"段规划无解：{out.from_cell} → {out.target_cell}"
                         "（只走 known 格；物理净空/禁 180°/禁紧凑 U）")
            return out
        cells = [goal_state[0]]
        st = goal_state
        while came.get(st) is not None:
            st = came[st]
            cells.append(st[0])
        cells.reverse()
        out.cells = [geo.index_to_block(r - 1, c) for r, c in cells]
        out.tokens = list(out.cells)
        out.route_len_m = max(0.0, (len(cells) - 1) * geo.BLOCK_SIZE)
        out.cost = float(dist[goal_state])
        # ---- 自检（绝不把坏路线交给任务线程）----
        problems: list[str] = []
        for i in range(1, len(cells)):
            a, b = cells[i - 1], cells[i]
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
                problems.append(f"{out.cells[i - 1]}→{out.cells[i]} 非正交相邻")
            if b in blocked:
                problems.append(f"进入未知格 {out.cells[i]}")
            if not _move_ok(a, b):
                problems.append(f"{out.cells[i - 1]}→{out.cells[i]} 复核 move_ok 失败")
        if problems:
            out.error = "段路线自检未过：" + "；".join(problems[:3])
            return out
        # ---- 策略性转弯留痕（整线引擎会否决、段规划降级为代价的那些顶点）----
        soft: list[str] = []
        for i in range(1, len(cells) - 1):
            d_in = (cells[i][0] - cells[i - 1][0], cells[i][1] - cells[i - 1][1])
            d_out = (cells[i + 1][0] - cells[i][0], cells[i + 1][1] - cells[i][1])
            d_prev = ((cells[i - 1][0] - cells[i - 2][0],
                       cells[i - 1][1] - cells[i - 2][1]) if i >= 2 else None)
            ok_turn, _deg, _ex = ts.turn_gate(cells[i], d_prev, d_in, d_out)
            if not ok_turn:
                soft.append(out.cells[i])
        warns: list[str] = []
        if turned_around:
            warns.append("起点车头朝向与首段相反（180°）⇒ 已按「允许原地转向」重规划："
                         "车会先转头/小圆弧掉头，请确认该格前方约有 0.3 m 空间")
        if soft:
            warns.append("段路线含 %d 处整线 tracker-safe 会否决的策略性转弯"
                         "（已按代价处理）：%s" % (len(soft), " ".join(soft)))
        if eng != ENGINE_TRACKER_SAFE:
            warns.append(f"引擎 {eng}：段规划用基础网格判据（无物理净空扫掠）")
        out.warnings = tuple(warns)
        out.ok = True
        out.elapsed_ms = (time.perf_counter() - t0) * 1000.0
        return out
    except Exception as exc:                           # noqa: BLE001  ★ 段规划失败=可等待
        out.error = f"{type(exc).__name__}: {exc}"
        out.elapsed_ms = (time.perf_counter() - t0) * 1000.0
        return out


#: 只有这些阶段允许"发车/推进"（★ 白名单：未知阶段一律不动）
_DRIVING_PHASES: tuple[str, ...] = (MissionPhase.DRIVE, MissionPhase.APPROACH)

#: 终局阶段：不再自动推进，也不再被外部命令复位
_TERMINAL_PHASES: tuple[str, ...] = (MissionPhase.FINISH, MissionPhase.SAFE_STOP)

_EPS = 1e-9


@dataclass
class _StopContext:
    """一个停站点的**可变**上下文（只在 Mission 线程内使用，不跨线程共享）。"""

    index: int = -1
    block: str = ""
    kind: str = StopKind.SHOOT
    phase: str = StopPhase.SETTLE
    started_at: float = 0.0
    phase_started_at: float = 0.0
    settle_armed_at: float | None = None
    last_x: float | None = None
    last_y: float | None = None
    shots_at_arrival: int = 0
    shots_needed: int = 3
    detail: str = ""

    @property
    def active(self) -> bool:
        return self.index >= 0 and self.phase != StopPhase.DONE


class MissionTask(TaskThread):
    """任务调度（10 Hz）。"""

    def __init__(self, bridge, bb: Blackboard, bus: EventBus, cfg: Config, *,
                 log: Callable[[str], None] = print, group=None,
                 clock: Callable[[], float] | None = None) -> None:
        super().__init__("MissionTask", 10.0, bus=bus, blackboard=bb, logger=log)
        self.bridge = bridge
        self.cfg = cfg
        self.group = group
        if clock is None:
            import time as _time
            clock = _time.monotonic
        self._clock = clock

        # ---- 配置（一次性读出；缺键全部走显式默认值，见模块 docstring 的 ⚠️） ----
        self._auto_start = bool(cfg.get("mission.auto_start"))
        self._auto_start_delay_s = float(cfg.get("mission.auto_start_delay_s") or 0.0)
        self._reset_spec = str(cfg.get("mission.reset") or "")
        self._scan_mode = str(cfg.get("mission.scan_mode") or "").strip().lower()
        # ---- 云台扫描**单写者**铁律（与 AimTask 的锁止配套）----
        #  为什么 Mission 必须放权（run11 日志实证的顺序）：
        #      云台锁止 scan_request=0（阶段=None）
        #   →  mission「扫描模式 continuous（continuous → scan_request=2）」
        #   →  云台起转搜索（最佳射击点 F4）
        #  即：Mission 在**巡迹全程**用 2 压住了 Aim 的 0；而 C++ 侧 `scan_request` 是
        #  **电平保持**语义，Aim 又有 `_gimbal_cmd` 去重（值没变就不再重发 0）
        #  ⇒ 云台实际一直在转 —— 正是用户要杜绝的形态（死灰复燃的洞）。
        #  因此：`aim.gimbal_lock_enabled=true`（默认）时，**Mission 一个值都不发**
        #  （路线就绪的 2、FINISH 的 0 全部交给 Aim 按 P_STOP_POINT/阶段决定）；
        #  只有显式 `aim.gimbal_lock_enabled=false`（旧行为/对照档）时，Mission 才照旧发。
        self._gimbal_lock_enabled = bool(cfg.get("aim.gimbal_lock_enabled", True))
        self._shots_per_stop = int(cfg.get("mission.shots_per_stop", 3) or 3)
        # ---- 双射击停站 + 红面早退 ----
        #   ① `_red_early_leave_s` 与 AimTask 用**同一个键**（`aim.red_early_leave_s`），
        #      保证"标志成立"和"Mission 允许早退"不会因两份默认值而打架；
        #      Mission 这边再核一次"已停留 ≥ 阈值"，是防止陈旧标志（对上一停站点成立的）
        #      把新停站点一进去就踢出来 —— 双保险，缺一不可。
        #   ② `_hit_this_stop` 由 `_phase_fire` 在"发数增量达标"那一路置位；`_complete_stop`
        #      据此跳过后续射击停站（第二个停站只为"看另一面"而设，已经打出去了就不必绕）。
        self._red_early_leave_s = float(cfg.get("aim.red_early_leave_s", 3.0) or 3.0)
        self._skip_second_after_hit = bool(
            cfg.get("mission.skip_second_stop_after_hit", True))
        self._hit_this_stop = False
        self._settle_time_s = float(cfg.get("mission.settle_time_s", 0.8))
        self._settle_eps_m = float(cfg.get("mission.settle_eps_m", 0.02))
        self._settle_speed_mps = float(cfg.get("mission.settle_speed_mps", 0.02))
        self._lock_timeout_s = float(cfg.get("mission.lock_timeout_s", 18.0))
        self._shot_timeout_s = float(cfg.get("mission.shot_timeout_s", 30.0))
        self._scan_stop_timeout_s = float(cfg.get("mission.scan_stop_timeout_s", 8.0))
        # ★★ **到站级硬死线**（秒）—— 自"到站那一刻"起累计超过它
        #   就强制离站（0 发也一样），口径 = "最佳射击点停留最长 30s"。
        #   为什么必须单独有一条：三段式各自超时是**按段**计时的（SETTLE 0.4×30=12s +
        #   ACQUIRE 6s + FIRE 30s = 最长 48s），与用户口径"到站起 30s"不符；"0 发"时旧实现
        #   要等完三段才放行。真源沿用 `mission.shoot_timeout_s`（现场 30.0，不动它），
        #   另给一个可选覆盖键 `mission.stop_deadline_s`（未写入 robot.yaml，避免两个同义键）。
        try:
            self._stop_deadline_s = float(cfg.get("mission.stop_deadline_s",
                                                  self._shot_timeout_s)
                                          or self._shot_timeout_s)
        except (TypeError, ValueError):
            self._stop_deadline_s = self._shot_timeout_s
        self._settle_grace_s = max(0.4 * self._shot_timeout_s, 1.0)     # ★ 旧版 0.4×SHOOT_TIMEOUT
        self._arrival_m = float(cfg.get("pursuit.arrival_m"))
        self._stop_radius_m = float(cfg.get("pursuit.stop_radius_m"))
        self._approach_m = float(cfg.get("pursuit.approach_m"))
        self._rate_limit_s = float(cfg.get("logging.rate_limit_s") or 1.0)
        # ---- 拐角绕圈兜底（仿真实测噪声 0.01m/拍会让车原地转圈、末点误差 2.96m）----
        self._stall_timeout_s = float(cfg.get("mission.stall_timeout_s", 12.0))
        self._stall_yaw_deg = float(cfg.get("mission.stall_yaw_deg", 360.0))
        # ---- ★自动规划：全部走 `planner.*`，缺键用显式默认 ----
        self._auto_plan_enabled = bool(cfg.get("planner.auto_plan_enabled", False))
        self._planner_map_path = str(cfg.get("planner.map_path") or DEFAULT_MAP_PATH)
        self._planner_fallback = bool(cfg.get("planner.fallback_to_tokens", True))
        self._planner_allow_reverse = bool(cfg.get("planner.allow_reverse", False))
        try:
            self._planner_max_seconds = float(cfg.get("planner.max_seconds",
                                                      _DEFAULT_PLAN_SECONDS)
                                              or _DEFAULT_PLAN_SECONDS)
        except (TypeError, ValueError):
            self._planner_max_seconds = _DEFAULT_PLAN_SECONDS
        # 引擎开关（缺省 `tour` ⇒ 与历史逐字节一致）。这里只**读**一次；
        #   真正的分派与门禁在 `plan_route_from_config()`（与 `--plan-only` 共用同一段代码）。
        self._planner_engine = resolve_plan_engine(cfg, self._log)
        #: 本次实际用的时间预算（tour=`planner.max_seconds`；tracker-safe=`planner.tracker_max_seconds`）
        self._planner_budget_s = plan_budget_seconds(cfg, self._planner_engine, self._log)

        # ---- 无人机桥洞情报（EV_BRIDGE_INFO / P_DRONE → 穿洞格）----
        #   配置（`drone.*` / `mission.bridge_info_wait_s` / `route.cross_*` 都已在 SCHEMA 里）
        self._drone_link_enabled = bool(cfg.get("drone.enabled", True))
        try:
            self._drone_info_stale_s = float(cfg.get("drone.info_stale_s") or 30.0)
        except (TypeError, ValueError):
            self._drone_info_stale_s = 30.0
        try:
            self._bridge_wait_s = max(0.0, float(cfg.get("mission.bridge_info_wait_s") or 0.0))
        except (TypeError, ValueError):
            self._bridge_wait_s = 0.0
        # ---- 桥前等待（L10 决策点）+ 赛事预算 + 迟到改道上限 ----
        #: 起点是否仍为情报推迟发车（默认 False = 等移到 L10 决策点，设计建议②）
        self._bridge_wait_at_start = bool(cfg.get("mission.bridge_wait_at_start", False))
        #: L10 桥前等待上限（与预算余量取 min）
        try:
            self._bridge_wait_max_s = max(0.0, float(cfg.get("mission.bridge_wait_max_s") or 0.0))
        except (TypeError, ValueError):
            self._bridge_wait_max_s = 0.0
        #: 本局总预算（3 分钟口径）—— 桥前等待"预算余量"的数据源
        try:
            self._race_budget_s = max(0.0, float(cfg.get("mission.race_budget_s") or 0.0))
        except (TypeError, ValueError):
            self._race_budget_s = 0.0
        #: 决策点前允许的换洞改道次数（全程上限，默认 1）
        try:
            self._bridge_reroute_max = max(0, int(cfg.get("mission.bridge_reroute_max") or 0))
        except (TypeError, ValueError):
            self._bridge_reroute_max = 0
        #: 已改道次数 / 本次保持状态 / 已决策过的洞（`(half, 洞口下标)`）
        self._reroutes = 0
        self._hold_started_at: float | None = None
        self._hold_cap_s = 0.0
        self._hold_half = ""
        self._hold_decided_for: tuple[str, int] | None = None
        #: 两个半场的**兜底**穿洞格（`route.cross_{half}_default`；缺键取 SCHEMA 默认）
        self._cross_default: dict[str, str] = {}
        for _half in BRIDGE_HALVES:
            self._cross_default[_half] = str(cfg.get(f"route.cross_{_half}_default")
                                             or "").strip().upper()
        #: 最近一次（每拍刷新）的情报结论：half → `BridgeIntel`
        self._intel: dict[str, BridgeIntel] = {}
        #: 按半场存**原始情报**（half → (格, 该条情报的 ts)），真源 = `EV_BRIDGE_INFO`。
        #:   两半场各占一个键 ⇒ 后到的 `upper` 事件**顶不掉**先到的 `lower` 记录
        #:   （第一版读 `P_DRONE` 单槽时，覆盖式赋值会让另一半天永久丢失 —— 早期实现的覆盖式赋值缺陷）
        self._intel_seen: dict[str, tuple[str, float]] = {}
        #: `EV_BRIDGE_INFO` 订阅句柄（惰性创建；`setup()` 里先建好，遥测也能看到队列状态）
        self._bridge_sub: Subscription | None = None
        #: 已消费的情报事件条数（遥测 `bridge.events`；现场排"情报到底到没到"看它）
        self._intel_events = 0
        #: 本局**实际装进执行路线**的穿洞格（路线载入即冻结；晚到的情报不再改它）
        self._intel_used: dict[str, str] = {}
        #: 等待窗口/换洞决策的证据（遥测 `bridge.note`）
        self._intel_note: dict[str, Any] = {}
        self._intel_wait_started: float | None = None
        self._intel_waited_s: float | None = None
        self._intel_guard = FaultGuard()             # 一次性日志（进/出等待窗口）
        self._intel_warn = Throttle(self._rate_limit_s)
        #: 候选集合 / 本地地图 只算一次的缓存（每拍刷新情报要用，不能每拍读文件）
        self._pool_cache: dict[str, tuple[str, ...]] = {}
        self._map_cfg_cache: dict | None = None
        self._map_cfg_tried = False

        # ---- 任务态 ----
        self._created_at = self._clock()
        self._started_at: float | None = None
        self._phase = MissionPhase.IDLE
        self._route: Route | None = None
        self._path: Path | None = None
        self._route_name = ""
        #: ★路线来源（`ROUTE_SOURCE_AUTO` / `ROUTE_SOURCE_MANUAL` / `ROUTE_SOURCE_COMMAND`），
        #: 横幅与遥测的"路线来源"行用它；`planner.auto_plan_enabled=false` 时开局即手动配置。
        self._route_source = ROUTE_SOURCE_MANUAL if not self._auto_plan_enabled else ""
        #: 最近一次自动规划的证据（map/耗时/错误），写进 `MISSION_STATE_KEY` 便于遥测与测试断言
        self._plan_info: dict[str, Any] = {}
        #: ★硬失败定格：`fallback_to_tokens=false` 且规划失败 → 停在 IDLE 且**不再每拍重试**
        self._autoplan_blocked = False
        self._waypoint_count = 0
        self._passed: list[bool] = []
        self._arrived = 0
        self._next_wp = 1
        self._handled_stops: set[int] = set()
        # ⚠️ 名字必须避开 `self._stop`：`threading.Thread` 内部有同名方法，
        #    被实例属性遮蔽后 `stop()/join()` 会在 `self._stop()` 处炸
        #    （`TypeError: 'NoneType' object is not callable`）—— 已实测踩过。
        self._stopctx: _StopContext | None = None
        self._last_stop_state = StopPointState()
        self._fire_gate_state: bool = False
        self._route_s: float | None = None
        self._dist_to_next_stop: float | None = None
        self._finish_latched = False
        self._finish_done = False
        self._finish_info: dict[str, Any] = {}
        # ★ 需求2：本进程内是否已到过终点。True 后 auto_start 计时器不再自动发车
        #   （到终点复位回 IDLE 后，若不拦，auto_start=true 会在下一拍又自动出发 → 无限循环）。
        self._auto_start_consumed = False
        # ★ 需求2：FINISH 后延迟一拍复位（让 FINISH 阶段/结算横幅至少可见一次，再回 IDLE）。
        self._finish_pending_reset = False
        self._scans: list[int] = []
        self._cmd_stats: dict[str, int] = {"total": 0, "rejected": 0}
        self._log_throttle = Throttle(self._rate_limit_s)

        # ---- ★T27 步进循环：段执行上下文（main 的 StepLoop 投递，本线程 tick 处理）----
        #   为什么走"请求队列 + tick 处理"而不是让 main 直接调 _load_tokens：
        #   main 循环在**另一个线程**，直接改 _route/_passed 会与 10 Hz 的 tick 竞态
        #   （改到一半被投影/到点判定读到）。这里只用一个锁保护"请求槽 + 完成回报"，
        #   route/阶段等执行态**只在本线程**改 —— 与外部 /nav/waypoint 命令同一模式。
        self._segment_lock = threading.Lock()
        self._segment_req: dict[str, Any] | None = None
        self._segment_stop_req: str | None = None
        self._segment_active = False
        self._segment_id = 0
        self._segment_target = ""
        self._segment_known = 0
        self._segment_done: dict[str, Any] | None = None
        self._segment_error = ""

        # 停站点（含末点）的弧长前缀与目标索引，路线载入时算一次
        self._stop_s: dict[int, float] = {}
        self._stop_blocks: list[str] = []
        #: ★t7 停站点**类型**台账（索引 → `SHOOT` / `SCAN` / `END`）：横幅与遥测共用。
        #:  为什么必须单开一份：`_stop_s` 里"末点"与"射击停站"混在一起（末点只是减速/判定锚点），
        #:  旧横幅直接印个数 ⇒ 现场把终点当成"第 2 个射击停站"（15:04 那趟的错账）。
        self._stop_kind: dict[int, str] = {}
        self._last_s = 0.0
        # ---- 拐角绕圈兜底的状态 ----
        self._stall_since: float | None = None       # 弧长开始停滞的时刻（None = 没在停滞）
        self._stall_last_s: float = 0.0              # 停滞开始时的弧长
        self._stall_last_yaw: float | None = None    # 停滞期间累计航向（绕圈判定）
        #: ★到点判定的"是否贴在路径上"：`_current_s` 现算投影时记下的横向偏差（米）
        self._last_lat_m: float | None = None
        self._stall_yaw_sum: float = 0.0
        #: ★静止闸窗口（见 _update_still_window）：上一拍位姿/时刻 + 连续静止拍数
        self._still_prev: tuple[float, float, float] | None = None
        self._still_last_t: float | None = None
        self._still_n: int = 0
        self._stall_block_logged: bool = False        # 本轮停滞只打一条"禁止跳点"告警
        self.stall_skip_blocked: int = 0              # ★因车静止被拦下的跳点次数（可观测）
        self._forced_skips: int = 0                  # 强制跳过航点的次数（结束横幅要汇总）
        # ---- ★PIVOT 完成通知（Tracker 经 `PIVOT_DONE_KEY` 写，见 tracker.py 注释）----
        #: 经 PIVOT 通知判到达的航点集合：停站点若由此判到达，仍要进三段式停站
        #: （`_next_stop_candidate` 据此放行），不能像普通"已过"那样直接跳过。
        self._passed_by_pivot: set[int] = set()
        #: 通知判到达的"车仍在顶点附近"窗口：真完成时车距顶点 ≈ 触发距离（掉头蠕转
        #: 还会带出少量位移）；跨轮次/竞态残留的通知里车在起点，二者差好几段，被窗口挡住。
        #: ★G1 速度自适应：窗口随有效触发距离缩放（0.30 巡航时 3×0.30，与 tracker 同源口径）。
        self._pivot_accept_m = max(
            pivot_dist_eff(cfg, float(cfg.get("pursuit.cruise_speed_mps"))) * 3.0,
            self._arrival_m * 6.0)

        # ---- ★逐点到达台账（硬约束：绝不静默丢点）----
        #: 每个待推进航点的**历史最近距离**（米）。几何复核用：s 一步跨过航点时，
        #: 只看"当拍"的距离会误判 —— 车常在几拍前就从格心旁穿过（此时 s 还没到），
        #: 等到 s 跳过去那一拍，车已经走远了。历史最近距离能如实回答"它到底来没来过"。
        self._min_dist: list[float] = []
        #: 索引 → 判达依据（"s+近路径" / "距格心" / "PIVOT 通知" / "最近接近 X m"）
        self._arrived_by: dict[int, str] = {}
        #: 索引 → 显式跳过依据（停滞兜底 / 收尾清账）。**凡未到达必在此或 `_arrived_by` 里**
        self._skipped_at: dict[int, str] = {}
        #: 索引 → "越窗待定"依据（finding D1）。**只是候选，不置位**：投影跳变的同一拍车
        #: 可能马上又靠近这个点 —— 立即判死会把本来能登记到达的航点提前毙掉
        #: （bench 实测：雷 I8 在基线里是「到点 8/45」，立即跳过版把它变成跳点 ⇒ 雷 7/8）。
        self._pending_skip: dict[int, str] = {}
        #: s 已越过、但车从未接近 ⇒ 显式跳过（与停滞兜底分开计数，便于分别审计）
        self._cross_skips: int = 0
        #: "最近接近"复核半径 = 半格（0.175 m）。**刻意不取更宽**：格间距就是 0.35 m，
        #: 半径超过半格会把"从邻居格心穿过"也算成到过。正常切角（离线心 ~0.29 m）由
        #: `near`（横向偏差判据）负责登记，这里只认"真从格心附近穿过"。
        self._close_m = max(self._arrival_m, 0.5 * geo.BLOCK_SIZE)
        #: 判"越窗跳过"的**超窗余量**：s 越过航点不足这个距离时先挂起（可能正在靠近），
        #: 越过这么多还没接近过 ⇒ 显式记账。取值 = 4×arrival_m 与 1 格取大（≈0.48 m）：
        #: 既不会被投影跳变的瞬间误判成"跳点"，也保证一个航点不会无限期悬着。
        self._skip_over_m = max(4.0 * self._arrival_m, geo.BLOCK_SIZE)

    # ------------------------------------------------------------------ #
    #  只读接口（测试/遥测）
    # ------------------------------------------------------------------ #
    @property
    def phase(self) -> str:
        return self._phase

    @property
    def route(self) -> Route | None:
        return self._route

    @property
    def stop_context(self) -> _StopContext | None:
        return self._stopctx

    @property
    def handled_stops(self) -> set[int]:
        return set(self._handled_stops)

    def state(self) -> dict[str, Any]:
        """最近一拍的任务上下文（与 `P_TELEMETRY[MISSION_STATE_KEY]` 一致）。"""
        return {
            "phase": self._phase, "route_len": self._waypoint_count,
            "next_wp": self._next_wp, "arrived": self._arrived,
            "handled_stops": sorted(self._handled_stops),
            # ★t7 停站台账（机器可读）：count/shoot/scan/end + 逐点 kind
            "stops": self.stop_ledger(),
            "stop_index": self._stopctx.index if self._stopctx else -1,
            "stop_block": self._stopctx.block if self._stopctx else "",
            "stop_phase": self._stopctx.phase if self._stopctx else StopPhase.DONE,
            "stop_detail": self._stopctx.detail if self._stopctx else "",
            "dist_to_next_stop": self._dist_to_next_stop,
            "finish": dict(self._finish_info),
            "commands": dict(self._cmd_stats),
            # ★路线来源与自动规划证据（横幅同源；遥测/测试据此断言「自动规划」/「手动配置」）
            "route_source": self._route_source,
            "plan": dict(self._plan_info),
            # ★逐点到达台账（硬约束）：`unexplained` 恒为 0 才叫"无静默丢点"
            "accounting": self.accounting(),
            # 无人机桥洞情报（实时结论 + 本局**冻结**的实际用格）
            "bridge": self.bridge_state(),
            # ★T27 段执行上下文（步进循环遥测；与 StepLoop 的 P_CAR_STATE 同源）
            "segment": self.segment_status(),
        }

    # ------------------------------------------------------------------ #
    #  ★T27 步进循环接口：主线程投递段任务，任务线程在 tick 里执行
    # ------------------------------------------------------------------ #
    def request_segment(self, tokens, *, segment_id: int = 0, target_cell: str = "",
                        known_count: int = 0, source: str = "step") -> dict[str, Any]:
        """请求执行一段路线（**线程安全**；tick 开头才真正载入/发车）。

        拒绝条件（返回 accepted=False + reason，调用方据此 WAIT_INFO 重试）：
          · 本段仍在执行 ⇒ **换段只允许在段边界**（T27 验收项）；
          · 上一段请求还没被任务线程取走（10 Hz，最多 100 ms）。
        """
        try:
            toks = [str(t) for t in (tokens or ())]
        except TypeError:
            return {"accepted": False, "reason": "tokens 不是可迭代对象"}
        if not toks:
            return {"accepted": False, "reason": "段路线为空"}
        req = {"tokens": toks, "segment_id": int(segment_id),
               "target_cell": str(target_cell).strip().upper(),
               "known_count": int(known_count), "source": str(source)}
        with self._segment_lock:
            if self._segment_req is not None:
                return {"accepted": False, "reason": "上一段请求尚未被任务线程取走"}
            if self._segment_active:
                return {"accepted": False, "reason": "本段仍在执行（换段只允许在段边界）"}
            self._segment_req = req
            # ★ 一接受就清掉上一段的完成回报：否则调用方可能把旧 done 当成本段结果
            self._segment_done = None
            self._segment_error = ""
            return {"accepted": True, "reason": ""}

    def stop_segment(self, reason: str = "") -> dict[str, Any]:
        """请求终止当前段并回到 IDLE（安全停机用；tick 开头执行）。"""
        with self._segment_lock:
            self._segment_stop_req = str(reason or "外部请求停止段")
            self._segment_req = None                   # 未处理的段请求一并作废
            return {"requested": True, "reason": self._segment_stop_req}

    def segment_status(self) -> dict[str, Any]:
        """段执行状态（**只读**；遥测/测试/StepLoop 共用同一份口径）。"""
        with self._segment_lock:
            done = dict(self._segment_done) if isinstance(self._segment_done, dict) else None
            return {"active": bool(self._segment_active),
                    "segment_id": int(self._segment_id),
                    "target_cell": str(self._segment_target),
                    "known_count": int(self._segment_known),
                    "done": done, "error": str(self._segment_error),
                    "phase": self._phase}

    def reset_pose(self, spec: str) -> None:
        """把 belief 放到 spec（BLOCK,HEAD）格心（步进器启动时复位一次；不改阶段/路线）。"""
        self._reset_belief(spec)

    def _process_segment_requests(self) -> None:
        """tick 开头处理段请求（★ 只有本线程改 route/阶段，见 __init__ 注释）。"""
        with self._segment_lock:
            stop = self._segment_stop_req
            self._segment_stop_req = None
            req = self._segment_req
            self._segment_req = None
        if stop is not None:
            if self._segment_active:
                self._segment_active = False
                self._segment_error = f"段被终止：{stop}"
            self._clear_route(f"段停止（{stop}）")
            self._write_phase(MissionPhase.IDLE)
            try:
                self.bridge.publish_brake()
            except Exception:                          # noqa: BLE001 收尾刹车失败不崩
                pass
            self._log(f"■ 段停止：{stop}（已清空路线、回 IDLE）")
            return
        if req is None:
            return
        self._segment_id = int(req["segment_id"])
        self._segment_target = str(req["target_cell"])
        self._segment_known = int(req["known_count"])
        self._segment_error = ""
        try:
            # ★ 段边界 = 新的轮次：清掉上一段的终点锁存，否则本段永远判不到 FINISH
            #   （FINISH 是锁存的；跨段重启必须显式解闩）。
            self._finish_latched = False
            self._finish_done = False
            self._finish_pending_reset = False
            self._segment_active = True
            self._load_tokens(req["tokens"],
                              source=f"段{self._segment_id}({req['source']})")
            self._start_mission()
            self._log(f"▶ 段 {self._segment_id} 开始：{len(req['tokens'])} 点 / "
                      f"目标 {self._segment_target} / known {self._segment_known} 格 / "
                      f"来源 {req['source']}")
        except Exception as exc:                       # noqa: BLE001  ★ 段坏路线绝不崩
            self._segment_active = False
            self._segment_error = f"{type(exc).__name__}: {exc}"
            self._log(f"❌ 段 {self._segment_id} 载入失败（保持 IDLE）：{self._segment_error}")
            self._clear_route("段载入失败")
            self._write_phase(MissionPhase.IDLE)

    def accounting(self) -> dict[str, Any]:
        """逐点到达台账（遥测/测试/结束横幅共用同一份口径）。

        口径（验收口径，**不放宽**）：
          · 分母 = `len(route) - 1`（起点不计）；
          · `arrived` = 登记到达数（含"最近接近"几何复核、含 PIVOT 通知判达）；
          · `skipped` = 显式跳过数（停滞兜底 + s 越窗），每一条都有事件与日志；
          · `unexplained` = 既没登记到达、也没显式跳过的航点 —— **必须为空**。
        """
        n = self._waypoint_count
        unexplained = [i for i in range(1, n)
                       if not self._passed[i] and i not in self._skipped_at
                       and i not in self._pending_skip] if self._passed else []
        by: dict[str, int] = {}
        for reason in self._arrived_by.values():
            key = reason.split(" ")[0]
            by[key] = by.get(key, 0) + 1
        return {
            "denominator": max(0, n - 1),
            "arrived": self._arrived,
            "arrived_by": by,
            "stall_skips": self._forced_skips,
            "cross_skips": self._cross_skips,
            "skipped_total": len(self._skipped_at),
            "skipped": {str(k): v for k, v in sorted(self._skipped_at.items())},
            # ★"越窗待定"（已记账、未定案）：收尾时由 `_close_ledger()` 转成正式跳过，
            #   期间若车又靠近，上面的判据会把它登记为**到达**（雷 I8 就是这种）
            "pending_skips": {str(k): v for k, v in sorted(self._pending_skip.items())},
            "unexplained": unexplained,
            "close_m": round(self._close_m, 4),
            "skip_over_m": round(self._skip_over_m, 4),
        }

    def closest_approaches(self) -> dict[int, float]:
        """每个航点的**历史最近距离**（米，-1 表示还没测到）。

        只读接口，供"车体到底有没有盖住格心"的复核（雷 A9 / 雷覆盖对账）——
        本方法**不改变**任何到达/跳过判定，只是把 Mission 已经算出来的距离摊开给人看。
        """
        return {i: (round(v, 4) if v != math.inf else -1.0)
                for i, v in enumerate(self._min_dist)}

    # ------------------------------------------------------------------ #
    #  无人机桥洞情报（EV_BRIDGE_INFO 事件流 → 穿洞格；P_DRONE 只判链路）
    # ------------------------------------------------------------------ #
    def bridge_state(self) -> dict[str, Any]:
        """桥洞情报的遥测快照（`state()["bridge"]`；现场不翻日志也能看到结论与来源）。"""
        return {
            "enabled": self._drone_link_enabled,
            "stale_s": self._drone_info_stale_s,
            "wait_s": self._bridge_wait_s,
            "waited_s": (None if self._intel_waited_s is None
                         else round(self._intel_waited_s, 3)),
            "link": self._drone_state() is not None,
            # 情报通道的证据（订阅在听 + 已消费条数 + 队列溢出计数）
            "subscribed": self._bridge_sub is not None,
            "events": self._intel_events,
            "events_dropped": (self._bridge_sub.dropped if self._bridge_sub is not None else 0),
            # 每拍刷新的**实时**结论（发车后晚到的情报在这里可见）
            "lower": self._intel.get("lower", BridgeIntel("lower")).as_dict(),
            "upper": self._intel.get("upper", BridgeIntel("upper")).as_dict(),
            # 本局实际装进执行路线的格（路线载入即冻结）
            "used": dict(self._intel_used),
            # ★ 按半场存的原始情报（格 + 该条情报自己的 ts）：真源是 `EV_BRIDGE_INFO` 事件流
            #   （每条自带 half，**不受 `P_DRONE` 单槽覆盖式赋值影响**）—— 现场排"情报到没到"看它
            "seen": {half: {"cell": cell, "ts": round(ts, 3)}
                     for half, (cell, ts) in sorted(self._intel_seen.items())},
            "note": dict(self._intel_note),
        }

    def _drone_state(self) -> Any:
        """读 `P_DRONE`（`DroneState`）；缺失/异常 → `None`（= 车侧没有无人机链路）。"""
        try:
            state = self.bb.get(P_DRONE)
        except Exception:                                # noqa: BLE001  黑盒读失败不该杀任务
            return None
        if state is None:
            return None
        # 只要"看起来像 DroneState"就接受（测试替身/gzip 后的旧版都兼容）
        if isinstance(state, DroneState):
            return state
        if all(hasattr(state, attr) for attr in ("bridge_block", "bridge_half", "bridge_ts")):
            return state
        return None

    def _intel_map_cfg(self) -> dict | None:
        """本地地图（候选集合的真源之一）；**只读一次**并缓存，读不到 → `None`。

        为什么缓存：情报每拍刷新，而这里会走文件 I/O —— 10 Hz 每拍读 YAML 是不可接受的。
        地图内容在任务生命周期内不变（进程内只读），缓存是安全的。
        """
        if not self._map_cfg_tried:
            self._map_cfg_tried = True
            try:
                path = resolve_map_path(self.cfg, self._planner_map_path)
                self._map_cfg_cache = _read_map_yaml(path)
            except Exception as exc:                     # noqa: BLE001  地图不可用不该杀任务
                self._map_cfg_cache = None
                if self._intel_warn.allow("map_missing", self._clock()):
                    self._log(f"⚠️ 桥洞情报：本地地图不可用（{type(exc).__name__}: {exc}）"
                              f"→ 候选校验只按 route.cross_*_candidates，手动换洞一律不做")
        return self._map_cfg_cache

    def _map_bridges(self) -> tuple[str, ...]:
        """`map.bridges` 的 K 列格（读不到地图 = 空）。"""
        geo_map = _map_geometry(self._intel_map_cfg())
        out: list[str] = []
        for cell in (geo_map.get("bridges") or []):
            text = str(cell).strip().upper()
            try:
                _, col_idx = geo.block_to_index(text)
            except ValueError:
                continue
            if col_idx == geo.COL_LETTERS.index(CROSS_COLUMN):
                out.append(text)
        return tuple(sorted(set(out)))

    def _half_candidates(self, half: str) -> tuple[str, ...]:
        """`route.cross_{half}_candidates` 收窄到"K 列 + 本半场行段"。"""
        band = CROSS_LOWER_ROWS if half == "lower" else CROSS_UPPER_ROWS
        raw = self.cfg.get(f"route.cross_{half}_candidates")
        out: list[str] = []
        for cell in (raw if isinstance(raw, (list, tuple)) else []):
            text = str(cell).strip().upper()
            try:
                row_idx, col_idx = geo.block_to_index(text)
            except ValueError:
                continue
            if col_idx == geo.COL_LETTERS.index(CROSS_COLUMN) and band[0] <= row_idx + 1 <= band[1]:
                out.append(text)
        return tuple(sorted(set(out)))

    def _bridge_pool(self, half: str) -> tuple[str, ...]:
        """某半场的合法穿洞格集合 —— 与 `planner._cross_pools` **同口径**（结果缓存）。

        口径：`route.cross_{half}_candidates` 与 `map.bridges` **都非空时取交集**，
        否则取非空的那个；两边都空 → 退成"整段 K 列"（规划器也是这么兜的）。
        为什么必须一模一样：这里放行的格会被喂给规划器，口径更松就会撞上规划器的
        `PlannerError`（`fallback_to_tokens=false` 时直接把车停在 IDLE）。
        """
        if half in self._pool_cache:
            return self._pool_cache[half]
        cands = self._half_candidates(half)
        band = CROSS_LOWER_ROWS if half == "lower" else CROSS_UPPER_ROWS
        bridges = tuple(c for c in self._map_bridges()
                        if band[0] <= geo.block_to_index(c)[0] + 1 <= band[1])
        pool = tuple(sorted(set(cands) & set(bridges))) if (cands and bridges) else (cands or bridges)
        if not pool:
            pool = tuple(f"{CROSS_COLUMN}{r}" for r in range(band[0], band[1] + 1))
        self._pool_cache[half] = pool
        return pool

    def _intel_cell_ok(self, half: str, cell: str) -> tuple[bool, str]:
        """无人机报的格是否可用（★ 权威门放宽：**列 → 本半场行段** 两道即可）。

        （按设计建议①、复核）：判据从"候选集合 ∩ `map.bridges`"放宽为
        **K 列 + 本半场行段**（下 K7~K11 / 上 K1~K6）—— 新鲜情报即可**纠正过期地图**：
        地图/候选停在上一局（例如只开 K10）时，无人机报的同半场行段内别的洞仍被采纳。

        仍然必须拒绝的（语义未放宽）：格名非法（`Z9`/`ZZ9`）、不在 K 列（`A1`）、
        不在本半场行段（跨半场借用）—— 采纳它们会让规划器/校验器直接报错
        （`fallback_to_tokens=false` 时把车停在 IDLE）。

        :returns: `(可采纳, 备注)`；备注在"池外但在行段内"时写明口径（遥测/日志对账用）。
        """
        text = str(cell or "").strip().upper()
        if not text:
            return False, "格名为空"
        try:
            row_idx, col_idx = geo.block_to_index(text)
        except ValueError as exc:
            return False, f"格名非法（{exc}）"
        if col_idx != geo.COL_LETTERS.index(CROSS_COLUMN):
            return False, f"不在 {CROSS_COLUMN} 列（桥洞列）上"
        band = CROSS_LOWER_ROWS if half == "lower" else CROSS_UPPER_ROWS
        row = row_idx + 1
        if not (band[0] <= row <= band[1]):
            return False, (f"第 {row} 行不属于"
                           f"{'下' if half == 'lower' else '上'}半场（{band[0]}~{band[1]} 行）")
        pool = self._bridge_pool(half)
        if text not in pool:
            # 池外**但**在本半场行段内 ⇒ 采纳（这就是"纠正过期地图"那条路）
            return True, (f"池外但在 K 列第 {row} 行（{band[0]}~{band[1]} 行）内 ⇒ 采纳，"
                          f"纠正过期地图（本半场候选/桥洞 {' '.join(pool)}）")
        return True, ""

    def _bridge_subscription(self) -> Subscription:
        """`EV_BRIDGE_INFO` 订阅句柄（**惰性**创建：调用方没走 `setup()` 也照样消费情报）。

        ★ 为什么惰性：`TaskGroup` 正常会先 `setup()`，但测试/bench/局部装配可能只调 `tick()`
        —— 那种情况下若订阅缺失，情报会被**静默忽略**（最坏是走默认洞，现场很难查）。
        惰性创建让"第一个 tick 起就在听"，代价只是"订阅之前发布的事件收不到"（与 `setup()`
        订阅同一语义：事件总线**不重放**历史）。
        """
        sub = self._bridge_sub
        if sub is None:
            sub = self.bus.subscribe(f"{self.name}:bridge", [EV_BRIDGE_INFO],
                                    maxsize=_BRIDGE_SUB_MAXSIZE)
            self._bridge_sub = sub
        return sub

    @staticmethod
    def _event_ts(data: dict, now: float) -> float:
        """情报事件里的 `ts`（`DroneTask` 写的是**它收到该条消息的时刻** ＝ 它写 `bridge_ts` 的值）。

        ★ 为什么用载荷里的 `ts` 而不是 `Event.ts`：`Event.ts` 默认是 `time.time()`（**墙钟**），
        而全部任务线程统一用注入时钟（车上是 `time.monotonic`）—— 两者不同源，混用会让
        "新鲜度"永远算错。载荷 `ts` 与 `P_DRONE.bridge_ts` 是**同一个数**（drone.py:283-287），
        所以这里的过期判据与 `DroneState.bridge_of()` **逐字同源**（已核验的 14.10/14.17）。
        载荷没带 / 非法（外部脚本手发的事件）→ 用本拍时钟兜底（等价于"刚收到"）。
        """
        try:
            ts = float(data.get("ts"))
        except (TypeError, ValueError):
            return now
        return ts if math.isfinite(ts) and ts > 0.0 else now

    def _record_intel(self, half: str, block: str, ts: float) -> bool:
        """按半场记一条情报（`half` 必须合法，调用方已过滤）。返回是否真的写进去了。

        ★ 两条语义（缺一不可）：
          ① **按半场独立**：`lower`/`upper` 各占一个键，互不覆盖 —— 这就是 D3 的修法。
          ② **同半场不倒退**：`ts` 早于现有记录的事件**不覆盖**（乱序/重放/外部脚本重放旧情报
             时，不能让一条旧情报把新鲜记录顶掉）。真车上 `DroneTask` 单线程按到达顺序发布，
             `ts` 天然单调，所以这条守卫在正常情况下永不触发。
        """
        prev = self._intel_seen.get(half)
        if prev is not None and ts < prev[1]:
            return False
        self._intel_seen[half] = (str(block).strip().upper(), ts)
        self._intel_events += 1
        return True

    def _drain_bridge_events(self, now: float, max_items: int = _BRIDGE_SUB_MAXSIZE) -> int:
        """消费本拍到达的 `EV_BRIDGE_INFO`（**桥洞情报的唯一真源**）。

        每条事件自带 `half/block/ts`（`drone.py:286-287`），所以**不受 `P_DRONE` 单槽的
        覆盖式赋值影响**（D3：先 lower 后 upper 时槽里只剩 upper）。
        **非阻塞**（`Subscription.poll` 是 `get_nowait` 循环）、**不上抛**（坏事件只丢弃 + 限流日志）、
        无文件 I/O —— 满足"宁可漏打，不可卡死"。
        事件队列溢出时 `Subscription.dropped` 只会被记一次日志并进遥测（不是每拍刷屏）。
        """
        sub = self._bridge_subscription()
        got = 0
        for ev in sub.poll(max_items):
            got += 1
            data = getattr(ev, "data", None)
            if not isinstance(data, dict):
                continue
            half = str(data.get("half") or "").strip().lower()
            block = str(data.get("block") or "").strip().upper()
            if half not in BRIDGE_HALVES or not block:
                # ★ 缺字段/半场非法：丢这一条（不猜、不影响另一半场、不炸任务）
                if self._intel_warn.allow("bad_bridge_event", now):
                    self._log(f"⚠️ 桥洞情报事件字段不全：half={half!r} block={block!r}"
                              f"（按『无这条情报』处理，另一半场不受影响）")
                continue
            self._record_intel(half, block, self._event_ts(data, now))
        if sub.dropped and self._intel_guard.first("bridge_drops"):
            self._log(f"⚠️ 桥洞情报事件队列溢出：累计丢 {sub.dropped} 条"
                      f"（队列 {_BRIDGE_SUB_MAXSIZE}）→ 仍按『最后一条』语义工作，"
                      f"被丢的那个半场会回落默认格（不卡死）")
        return got

    def _intel_of(self, half: str) -> BridgeIntel:
        """一个半场的**当前**情报结论（采纳顺序见模块 docstring；**永不上抛**）。"""
        half = str(half).strip().lower()
        default = self._cross_default.get(half, "")
        if not self._drone_link_enabled:
            return BridgeIntel(half, default, default, "disabled", "", "drone.enabled=false → 不读情报")
        state = self._drone_state()
        if state is None:
            return BridgeIntel(half, default, default, "no_link", "",
                               "未收到 P_DRONE（DroneTask 未运行/未启用）→ 用默认格")
        now = self._clock()
        rec = self._intel_seen.get(half)
        if not bool(getattr(state, "online", False)):
            # ★ 离线口径：即使情报还在过期窗口内也**回落默认**（离线 = 情报不可信）。
            #   现场含义：无人机断链后车不会再拿到新情报，宁可走配置里的兜底洞。
            raw = rec[0] if rec else ""
            age = (now - rec[1]) if rec else None
            return BridgeIntel(half, default, default, "offline", raw,
                               f"无人机离线（online=False，最近情报 {self._fmt_age(age)}）", age)
        if rec is None:
            return BridgeIntel(half, default, default, "missing", "",
                               "该半场从未收到 bridge_passable（EV_BRIDGE_INFO 事件）→ 用默认格")
        cell, ts = rec
        age = (now - ts) if ts else None
        if age is not None and age > self._drone_info_stale_s:
            return BridgeIntel(half, default, default, "stale", cell,
                               f"情报已过期 {age:.2f}s > {self._drone_info_stale_s:g}s"
                               f" → 用默认格", age)
        ok, why = self._intel_cell_ok(half, cell)
        if not ok:
            return BridgeIntel(half, default, default, "rejected", cell,
                               f"情报 {cell} 不可用（{why}）→ 用默认格", age)
        # `why` 非空 = 池外但在本半场行段内被采纳（纠正过期地图），把口径带进遥测/日志
        note = (f"新鲜情报（{self._fmt_age(age)} < 过期阈值 "
                f"{self._drone_info_stale_s:g}s）")
        if why:
            note = f"{note}；{why}"
        return BridgeIntel(half, cell, default, "drone", cell, note, age)

    def _refresh_intel(self, *, log: bool = True) -> dict[str, BridgeIntel]:
        """刷新两个半场的情报结论；`log=True` 时**只在结论变化时**记一条（限流兜底）。

        ★ 每拍先消费 `EV_BRIDGE_INFO`（情报真源），再算两半场的结论 —— 顺序不能反。
        """
        now = self._clock()
        self._drain_bridge_events(now)
        snap: dict[str, BridgeIntel] = {}
        for half in BRIDGE_HALVES:
            try:
                snap[half] = self._intel_of(half)
            except Exception as exc:                     # noqa: BLE001  ★ 情报读取绝不许杀任务
                default = self._cross_default.get(half, "")
                snap[half] = BridgeIntel(half, default, default, "no_link", "",
                                         f"情报读取异常（{type(exc).__name__}: {exc}）→ 用默认格")
        if log:
            for half, it in snap.items():
                prev = self._intel.get(half)
                changed = (prev is None or prev.source != it.source or prev.cell != it.cell
                           or prev.raw != it.raw)
                if changed and self._intel_warn.allow(f"intel:{half}:{it.source}:{it.cell}",
                                                      now):
                    self._log(f"桥洞情报[{half}]：{it.describe()}")
        self._intel = snap
        return snap

    def _intel_complete(self, snap: dict[str, BridgeIntel] | None = None) -> bool:
        """两半场是否都拿到了**新鲜情报**（等待窗口的退出条件）。"""
        snap = self._intel if snap is None else snap
        return all(bool(snap.get(half) and snap[half].from_drone) for half in BRIDGE_HALVES)

    def _has_fresh_half(self, snap: dict[str, BridgeIntel] | None = None) -> bool:
        """是否**已经有一半场拿到了可用的新鲜情报**（`source=drone` = 在线 + 新鲜 + 过 5 道门）。

        （F1）：这就是"等得到东西"的判据 —— 等待窗口只为"再等另一半"而开。
        """
        snap = self._intel if snap is None else snap
        return any(bool(snap.get(half) and snap[half].from_drone) for half in BRIDGE_HALVES)

    def _should_wait_for_intel(self, elapsed: float) -> bool:
        """是否该为情报推迟发车（`mission.bridge_info_wait_s` 窗口内，见模块 docstring）。

        （按 F1 裁决**收回判据**）：只有"**等得到东西**"才推迟发车，六条缺一不可：
          ① 窗口 ≤ 0；
          ② 硬上限未到（`elapsed < auto_start_delay_s + bridge_info_wait_s`，到点必发车，不卡死）；
          ③ 配置没关掉无人机链路（`drone.enabled=false` → 不等）；
          ④ 车侧**有**无人机链路（`P_DRONE` 存在）—— "全程无无人机"时不进窗口，
             行为与接线前**逐字一致**（不等、不换）；
          ⑤ 无人机**在线**（`DroneState.online=True`）—— 离线时情报不可信，等也白等；
          ⑥ 手上**已经有一半新鲜情报**（>=1 个半场 `source=drone`）—— 情报过期/从未到达时不等，
             立即按默认格出发，别让一个哑掉的链路压着发车时间；两半场都齐则无需等待。
        ⚠️ 为什么要收回（F1 原文）：`DroneTask` **每拍都写 `P_DRONE`**（无论在线与否），
        所以旧判据里的"有 link"**恒真** ⇒ 无人机离线也会白等到硬上限才发车。
        """
        if self._bridge_wait_s <= 0.0:
            return False
        if elapsed >= self._auto_start_delay_s + self._bridge_wait_s:
            return False
        if not self._drone_link_enabled:
            return False
        state = self._drone_state()
        if state is None:                                # ④ 无链路：不等、不换（接线前行为）
            return False
        if not bool(getattr(state, "online", False)):     # ⑤ 离线：不等
            return False
        if self._intel_complete():                       # ⑥ 两半场都齐：不必等
            return False
        return self._has_fresh_half()                    # ⑥ 至少要有一半可用的新鲜情报

    # ------------------------------------------------------------------ #
    #  L10 桥前等待 + 迟到情报改道一次（机制见模块 docstring 的桥前等待段）
    # ------------------------------------------------------------------ #
    def _bridge_wait_cap(self) -> float:
        """桥前等待上限 = `min(bridge_wait_max_s, 预算余量)`（★ ④，恒 ≥ 0）。"""
        if self._bridge_wait_max_s <= 0.0:
            return 0.0
        base = self._started_at if self._started_at is not None else self._created_at
        used = max(0.0, self._clock() - base)
        remain = max(0.0, self._race_budget_s - used)
        return max(0.0, min(self._bridge_wait_max_s, remain))

    def _decision_point(self) -> tuple[int, int, str] | None:
        """第一个"洞口尚未过"的桥洞决策点 → `(决策点下标, 洞口下标, 半场)`；没有则 `None`。

        ★ 决策点 = 洞口（K 列格）航点的**前一个**航点：默认路线是 `L10`（下）/`J5`（上）。
          不写死格名：换图/换洞后自动跟着走。
        """
        route = self._route
        if route is None or not self._passed:
            return None
        candidates: list[tuple[int, int, str]] = []
        for half, cell in (route.crossing or {}).items():
            want = str(cell).strip().upper()
            if not want:
                continue
            for i, wp in enumerate(route.waypoints):
                if i > 0 and str(wp.block).strip().upper() == want:
                    candidates.append((i - 1, i, str(half)))
                    break
        for dp in sorted(candidates, key=lambda item: item[1]):
            cross_idx = dp[1]
            if cross_idx < len(self._passed) and not self._passed[cross_idx]:
                return dp
        return None

    def _link_online(self) -> bool:
        """车侧有链路且无人机在线（**无链路/离线 = 不等待、不改道**，逐拍与当前版本一致）。"""
        if not self._drone_link_enabled:
            return False
        state = self._drone_state()
        return state is not None and bool(getattr(state, "online", False))

    def _maybe_enter_bridge_hold(self) -> None:
        """到达决策点这一拍做决策（★ ③④）：该等就进 HOLD，否则直接带已得情报走。"""
        if self._phase != MissionPhase.DRIVE:
            return
        dp = self._decision_point()
        if dp is None:
            return
        decision_idx, cross_idx, half = dp
        if not (decision_idx < len(self._passed) and self._passed[decision_idx]):
            return                                   # 还没到决策点
        if self._hold_decided_for == (half, cross_idx):
            return                                   # 这个洞已经决策过（含等待结束）
        self._hold_decided_for = (half, cross_idx)
        cap = self._bridge_wait_cap()
        if self._intel_complete() or (not self._link_online()) or cap <= 0.0:
            # 不等：把"此刻已到手的新鲜情报"用上（决策点前到 ⇒ 允许改道 ≤ 上限）
            reason = ("两半场情报已齐" if self._intel_complete()
                      else ("无链路/离线" if not self._link_online() else "上限为 0（预算耗尽）"))
            self._reroute_at_decision(half, why=f"不等（{reason}）")
            return
        self._hold_started_at = self._clock()
        self._hold_cap_s = cap
        self._hold_half = half
        self._write_phase(MissionPhase.HOLD)
        self.bridge.publish_brake()
        self._intel_note["hold"] = {"active": True, "half": half, "cap_s": round(cap, 3),
                                    "waited_s": 0.0, "reroutes": self._reroutes}
        self._publish_intel_note(waiting=True)
        self._log(f"桥前等待[{half}]：在决策点第 {decision_idx} 点（洞口 {cross_idx}）停车等情报，"
                  f"上限 min(bridge_wait_max_s={self._bridge_wait_max_s:g}s, 预算余量)={cap:.2f}s｜"
                  f"lower={self._intel.get('lower', BridgeIntel('lower')).describe()}；"
                  f"upper={self._intel.get('upper', BridgeIntel('upper')).describe()}")

    def _run_bridge_hold(self) -> None:
        """HOLD 每拍：两半场齐或到点 → 放行（**到点必发车**，语义不变）。"""
        if self._hold_started_at is None:            # 异常兜底：没有起点信息就立刻恢复 DRIVE
            self._write_phase(MissionPhase.DRIVE)
            return
        waited = max(0.0, self._clock() - self._hold_started_at)
        note = self._intel_note.get("hold")
        if isinstance(note, dict):
            note["waited_s"] = round(waited, 3)
        if self._intel_complete():
            self._release_bridge_hold(f"两半场情报已齐（等了 {waited:.2f}s）")
            return
        if waited >= self._hold_cap_s - 1e-9:
            self._release_bridge_hold(f"窗口到点（{self._hold_cap_s:.2f}s）→ 必须发车")
            return
        self.bridge.publish_brake()                  # Tracker 也刹（阶段不在白名单），这里双保险

    def _release_bridge_hold(self, reason: str) -> None:
        half = self._hold_half
        waited = 0.0 if self._hold_started_at is None else self._clock() - self._hold_started_at
        self._hold_started_at = None
        self._hold_half = ""
        if half:
            self._reroute_at_decision(half, why=reason)   # 决策点前迟到情报 → 换洞一次
        note = self._intel_note.get("hold")
        if isinstance(note, dict):
            note.update({"active": False, "waited_s": round(waited, 3),
                         "reroutes": self._reroutes, "reason": reason})
        self._write_phase(MissionPhase.DRIVE)
        self._publish_intel_note(waiting=False)
        self._log(f"桥前等待结束：{reason}｜lower={self._intel.get('lower', BridgeIntel('lower')).describe()}；"
                  f"upper={self._intel.get('upper', BridgeIntel('upper')).describe()}")

    def _reroute_at_decision(self, half: str, *, why: str = "") -> bool:
        """决策点前换洞一次（★ ⑤）：**保留当前段索引**，绝不把车送回起点。

        :returns: 是否真的换了洞。任何一步不满足（次数用尽/情报不新鲜/形状不符/校验不过）
            都原样保留旧路线 —— 与"改道失败就按原 token 走"的纪律一致。
        """
        if self._route is None or self._reroutes >= self._bridge_reroute_max:
            return False
        snap = self._intel or self._refresh_intel(log=True)
        it = snap.get(half)
        if it is None or not it.from_drone:
            return False
        current = str((self._route.crossing or {}).get(half) or "").strip().upper()
        if current == it.cell:
            return False                              # 与当前路线一致 → 不用改
        try:
            tokens = [str(t) for t in self._route.tokens()]
        except Exception as exc:                       # noqa: BLE001
            self._log(f"⚠️ 决策点换洞跳过（拿不到 token：{type(exc).__name__}）")
            return False
        new_tokens, accepted = self._apply_intel_to_tokens(tokens, {half: it})
        if not accepted or list(new_tokens) == tokens:
            self._intel_note["reroute_rejected"] = dict(self._intel_note.get("manual_substituted") or {})
            self._log(f"⚠️ 决策点换洞未生效[{half}]（形状/校验不过）→ 维持原 token（{why}）")
            return False
        try:
            route = parse_mission(new_tokens, shoot_planner=self._shoot_planner,
                                  name=self._route_name)
        except Exception as exc:                       # noqa: BLE001
            self._log(f"⚠️ 决策点换洞解析失败（{type(exc).__name__}: {exc}）→ 维持原 token")
            return False
        index_before = self.bb.get(P_ROUTE_INDEX)
        self._install_route(route, source=f"{self._route_name}+决策点改道", keep_progress=True)
        self._reroutes += 1
        self._intel_note["reroute"] = {"half": half, "old": current, "new": it.cell,
                                       "count": self._reroutes, "index_kept": index_before,
                                       "why": why}
        note = self._intel_note.get("hold")
        if isinstance(note, dict):
            note["reroutes"] = self._reroutes
        self._log(f"★ 决策点改道[{half}]（第 {self._reroutes}/{self._bridge_reroute_max} 次，{why}）："
                  f"{current or '默认'} → {it.cell}；保留段索引 {index_before}（不回车起点）")

    def _apply_intel_to_tokens(self, tokens: Any,
                               snap: dict[str, BridgeIntel] | None = None
                               ) -> tuple[Any, dict[str, tuple[str, str]]]:
        """把新鲜情报换进**手动** token 列表（`route.tokens` / `mission_blocks`）。

        :returns: `(tokens, accepted)`；`accepted = {half: (旧格, 新格)}`。
            **一个半场都没换成功时原样返回传入对象**（"行为与接线前逐字一致"的硬保证）。
        每个半场独立判定：形状不符 / 校验不过 → 该半场不换（另一半场照常）。
        """
        snap = self._intel if snap is None else snap
        want = {half: snap[half] for half in BRIDGE_HALVES
                if snap.get(half) and snap[half].from_drone}
        if not want:
            return tokens, {}
        try:
            work = [str(t) for t in tokens]
        except TypeError:
            return tokens, {}
        if not work:
            return tokens, {}
        accepted: dict[str, tuple[str, str]] = {}
        for half in BRIDGE_HALVES:
            it = want.get(half)
            if it is None:
                continue
            try:
                old = detect_crossing(parse_mission(list(work))).get(half)
            except Exception as exc:                     # noqa: BLE001  手工路线解析失败→不换
                self._intel_note[f"reject_{half}"] = f"路线解析失败（{type(exc).__name__}）"
                continue
            if not old:
                self._intel_note[f"reject_{half}"] = "路线里找不到本半场的穿洞格 → 不换洞"
                continue
            if old == it.cell:
                continue                                 # 情报与现役格相同：无需改写
            cand = rewrite_crossing_tokens(work, half, old, it.cell)
            if cand is None:
                reason = (f"路线形状不是『进洞口-{CROSS_COLUMN}列格-出洞口』同行三连"
                          f"（{old} 附近）→ 不换洞，走默认 {old}")
                self._intel_note[f"reject_{half}"] = reason
                if self._intel_warn.allow(f"shape:{half}", self._clock()):
                    self._log(f"⚠️ 桥洞情报[{half}]：{reason}")
                continue
            ok, why = self._swapped_tokens_ok(cand, half, old, it.cell)
            if not ok:
                self._intel_note[f"reject_{half}"] = why
                if self._intel_warn.allow(f"gate:{half}", self._clock()):
                    self._log(f"⚠️ 桥洞情报[{half}]：{old} → {it.cell} 未采纳：{why}"
                              f"（原样走 {old}）")
                continue
            work = cand
            accepted[half] = (old, it.cell)
            self._log(f"★ 桥洞情报[{half}]换洞生效：{old} → {it.cell}"
                      f"（三连同改 + validate_route 0 error）")
        if not accepted:
            return tokens, {}
        return work, accepted

    def _swapped_tokens_ok(self, tokens: list[str], half: str, old: str,
                           new: str) -> tuple[bool, str]:
        """换洞后的候选 token 列表能否执行：`validate_route` **0 error** 才放行。

        ★ 为什么必须真的跑校验：换洞会改动三格的位置，可能把路线挪到障碍/雷上，
        或让雷覆盖从 8/8 掉到 6/8（R005）。这些错**不能**靠"看上去对"来判。
        ★ 为什么地图不可用就拒：`validate_route` 的 R004/R005 依赖 `map_cfg`；没有地图时
        校验收窄成"只查几何"，那时放行等于**无证据地改路线** —— 宁可走配置里的默认洞。
        """
        map_cfg = self._intel_map_cfg()
        if map_cfg is None:
            return False, "本地地图不可用 → 不做换洞（保守：读得到地图才允许改路线）"
        try:
            route = parse_mission([str(t) for t in tokens])
        except Exception as exc:                         # noqa: BLE001
            return False, f"换洞后路线解析失败（{type(exc).__name__}: {exc}）"
        blocks = [w.block for w in route.waypoints]
        if new not in blocks or old in blocks:
            return False, f"换洞后 token 列表不符（{old} 仍在 / {new} 缺失）"
        try:
            issues = validate_route(route, map_cfg=map_cfg, cfg=self.cfg)
        except Exception as exc:                         # noqa: BLE001
            return False, f"换洞后校验异常（{type(exc).__name__}: {exc}）"
        errors = [str(i) for i in issues if getattr(i, "level", "") == "error"]
        if errors:
            return False, "换洞后校验未过（R004/R005/R008/R009 等）：" + "；".join(errors[:3])
        return True, ""

    @staticmethod
    def _fmt_age(age: float | None) -> str:
        return "未知" if age is None else f"{age:.2f}s"

    # ------------------------------------------------------------------ #
    #  生命周期
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        self._write_phase(MissionPhase.IDLE)
        # 桥洞情报的订阅在这里建好（**必须在任何 tick 之前**：事件总线不重放历史）。
        #   惰性兜底见 `_bridge_subscription()`。
        self._bridge_subscription()
        self._log(f"就绪：auto_start={self._auto_start} 延时={self._auto_start_delay_s:g}s "
                  f"复位={self._reset_spec!r} 扫描模式={self._scan_mode or '未设置'}")
        # 把桥洞情报的配置口径打全（现场一眼看出"情报从哪来、会不会等、默认走哪个洞"）
        self._log(f"桥洞情报：来源=EV_BRIDGE_INFO 事件流（半场各自独立） "
                  f"drone.enabled={self._drone_link_enabled} "
                  f"过期阈值={self._drone_info_stale_s:g}s 等待窗口={self._bridge_wait_s:g}s "
                  f"默认格 lower={self._cross_default.get('lower') or '未配置'}"
                  f"/upper={self._cross_default.get('upper') or '未配置'} "
                  f"（新鲜情报优先，缺失/过期/离线一律回落默认）")
        if self._bridge_wait_s > 0.0:
            self._log(f"桥洞情报：等待窗口只在『车侧有无人机链路（P_DRONE 存在）且两半场情报未齐』"
                      f"时生效；无 P_DRONE 时不等、不换，行为与接线前一致")

    def teardown(self) -> None:
        try:
            if self._bridge_sub is not None:            # 退订（别在总线上留悬挂队列）
                self._bridge_sub.unsubscribe()
                self._bridge_sub = None
        except Exception:                            # noqa: BLE001
            pass
        try:
            self.bridge.publish_fire_gate(False)     # ★ 收尾关闸：绝不留一个开着的放行门
        except Exception:                            # noqa: BLE001
            pass
        try:
            self.bridge.publish_brake()
        except Exception:                            # noqa: BLE001
            pass

    # ------------------------------------------------------------------ #
    #  单拍
    # ------------------------------------------------------------------ #
    def tick(self) -> None:
        now = self._clock()
        self._drain_commands()
        # ★T27 步进循环：段请求/停止请求（主线程投递）在本线程执行 —— 与外部命令同一
        #   模式，保证 route/阶段只被本线程改写（见 __init__ 里 _segment_lock 的说明）。
        self._process_segment_requests()
        # 每拍消费 `EV_BRIDGE_INFO` 事件流并刷新两半场结论（10 Hz；非阻塞 `poll`
        #   + 缓存的候选集合，**无文件 I/O**）。结论变化时才记一条日志（`_refresh_intel` 内部
        #   限流），所以不会告警风暴。
        self._refresh_intel()

        safety: SafetyState | None = self.bb.get(P_SAFETY)
        if safety is not None and not safety.ok:
            self._enter_safe_stop()
            self._publish_state(now)
            return
        if self._phase == MissionPhase.SAFE_STOP:
            self._recover_from_safe_stop()

        self._maybe_auto_start()
        if self._phase == MissionPhase.FINISH:
            # ★★ 需求2（用户 2026-10-16）：到终点后延迟一拍复位回 IDLE，进入"准备发车"状态。
            #   `_check_finish` 设置 FINISH 的**那一拍**已经完成结算（横幅+事件+刹车），
            #   这里下一拍才复位 —— 让 FINISH 阶段/结算横幅至少可见一次，遥测也能读到成绩。
            if self._finish_pending_reset:
                self._reset_for_next_run()
            else:
                self._finish_pending_reset = True
        elif self._phase in _DRIVING_PHASES:
            self._drive()
            self._maybe_enter_bridge_hold()         # 到 L10 决策点 → 可能进 HOLD
        elif self._phase == MissionPhase.HOLD:
            self._run_bridge_hold()                 # 保持中（Tracker 每拍自刹，见段①）
        elif self._phase == MissionPhase.STOP:
            self._run_stop_point()
        self._check_finish()
        self._publish_state(now)

    # ------------------------------------------------------------------ #
    #  出发
    # ------------------------------------------------------------------ #
    def _maybe_auto_start(self) -> None:
        if not self._auto_start or self._phase != MissionPhase.IDLE:
            return
        if self._auto_start_consumed:               # ★ 需求2：到过终点后不再自动发车
            return
        elapsed = self._clock() - self._created_at
        if elapsed < self._auto_start_delay_s:
            return
        if self._route is not None:                 # 外部命令已经载入过路线
            return
        if self._autoplan_blocked:                  # ★硬失败已定格（error 日志出过一次）→ 不再重试
            return
        # 情报等待窗口（`mission.bridge_info_wait_s`）。
        #   只在"车侧确实有无人机链路 + 两半场情报未齐"时推迟发车；窗口耗尽照常出发。
        #   默认**不在起点等**（`mission.bridge_wait_at_start=false`，等移到 L10 决策点）；
        #   置 true 即回到起点窗口语义（对照档/回归用）。
        if self._bridge_wait_at_start and self._should_wait_for_intel(elapsed):
            if self._intel_wait_started is None:
                self._intel_wait_started = self._clock()
            if self._intel_guard.first("wait"):
                self._log(f"等待无人机桥洞情报：最多 {self._bridge_wait_s:g}s"
                          f"（到 {self._auto_start_delay_s + self._bridge_wait_s:g}s 为止）｜"
                          f"当前 lower={self._intel.get('lower', BridgeIntel('lower')).describe()}"
                          f"；upper={self._intel.get('upper', BridgeIntel('upper')).describe()}")
            self._publish_intel_note(waiting=True)
            return
        if self._intel_wait_started is not None and self._intel_guard.first("wait_end"):
            self._intel_waited_s = self._clock() - self._intel_wait_started
            verdict = "两半场情报已齐" if self._intel_complete() else "窗口耗尽/无条件，用已得情报"
            self._log(f"桥洞情报等待结束：等了 {self._intel_waited_s:.2f}s（{verdict}）｜"
                      f"lower={self._intel.get('lower', BridgeIntel('lower')).describe()}；"
                      f"upper={self._intel.get('upper', BridgeIntel('upper')).describe()}")
            self._publish_intel_note(waiting=False)     # 遥测别停在"还在等"的假状态上
        try:
            self._reset_belief(self._reset_spec)
            self._load_startup_route()              # ★ 自动规划 / 读 route.tokens（见函数 docstring）
        except AutoPlanBlocked:
            self._autoplan_blocked = True           # ★ 停在 IDLE：不 publish EV_MISSION_STARTED
            return
        except Exception as exc:                    # noqa: BLE001  配置错就停着，绝不带病发车
            if self._log_throttle.allow("auto_start_failed", self._clock()):
                self._log(f"自动出发失败（保持 IDLE）：{type(exc).__name__}: {exc}")
            return
        self._log(f"自动出发：延时 {self._auto_start_delay_s:g}s 到，复位 {self._reset_spec!r}")
        self._start_mission()

    def _publish_intel_note(self, *, waiting: bool) -> None:
        """把等待窗口的状态写进遥测（`bridge.note`），便于现场判断"为什么还没出发"。"""
        self._intel_note["waiting"] = bool(waiting)
        self._intel_note["wait_s"] = self._bridge_wait_s
        if self._intel_wait_started is not None:
            self._intel_note["waited_s"] = round(self._clock() - self._intel_wait_started, 3)

    def _load_startup_route(self) -> None:
        """启动路线来源：`planner.auto_plan_enabled` 决定"自动规划"还是"读 route.tokens"。

        ★ 失败语义（两条分支都有独立测试）：
          - `planner.fallback_to_tokens: true` → warn（含异常类型+消息）后回退 `route.tokens`，
            **仍进入 DRIVE**（宁可路线不是最优，也要出车）；
          - `false` → error 日志 + 抛 `AutoPlanBlocked` → `_maybe_auto_start` 捕获后**停在 IDLE**，
            绝不 publish `EV_MISSION_STARTED`。

        `planner.engine: tracker-safe` 时"门禁未过"也走**同一条**失败路径
          （`plan_route_from_config` 返回 `ok=False` + error 里带门禁报文）——
          回退到现役手工路线并打印原因，绝不把没过门禁的路线装上执行。

        穿洞格优先用**新鲜无人机情报**（`self._intel`，本拍已刷新）：
          · 自动规划 → 作为 `cross_lower/cross_upper` 透传（未采纳的半场传 `None`）；
          · 手动路线 → 换洞三连改写 + `validate_route` 0 error 才执行；
          两半场各自独立。所有分支都在这里收口，`out.tokens` 之外没有第二个真源。
        """
        snap = self._intel or self._refresh_intel(log=True)
        cross = {half: (snap[half].cell if (snap.get(half) and snap[half].from_drone) else None)
                 for half in BRIDGE_HALVES}
        if not self._auto_plan_enabled:
            self._route_source = ROUTE_SOURCE_MANUAL
            tokens = self.cfg.get("route.tokens") or self.cfg.get("mission_blocks")
            tokens, accepted = self._apply_intel_to_tokens(tokens, snap)
            self._intel_note["manual_substituted"] = {
                half: f"{old}→{new}" for half, (old, new) in accepted.items()}
            src = "route.tokens(或 mission_blocks)"
            if accepted:
                src += "（无人机情报换洞）"
            self._load_tokens(tokens, source=src)
            return

        out = plan_route_from_config(self.cfg, map_path=self._planner_map_path, log=self._log,
                                     cross_lower=cross["lower"], cross_upper=cross["upper"])
        self._plan_info = {
            "ok": out.ok, "map_path": out.map_path, "elapsed_ms": out.elapsed_ms,
            "error": out.error, "allow_reverse": out.allow_reverse,
            "max_seconds": self._planner_budget_s, "token_count": len(out.tokens),
            "fallback_to_tokens": self._planner_fallback,
            # 引擎与门禁证据（遥测/测试据此断言"走的是哪个引擎、门禁过没过"）
            "engine": out.engine, "gate_ok": out.gate_ok, "gate_detail": out.gate_detail,
            # 足迹诊断（现场卷尺实测偏移下的挡板/障碍/靶板余量）
            "footprint": out.footprint, "footprint_ok": out.footprint_ok,
            # 穿洞格（请求的 + 规划产物实际采用的），现场对账用
            "cross_requested": {"lower": cross["lower"] or "", "upper": cross["upper"] or ""},
            "cross_lower": out.cross_lower, "cross_upper": out.cross_upper,
            # ★t7 双射击停站证据（遥测/现场一眼看出"这趟有几个打靶停站、第二个是不是补的"）
            "shoot_stops": list(out.shoot_stops),
            "second_stop_added": out.second_stop_added,
        }
        if out.ok:
            self._route_source = ROUTE_SOURCE_AUTO
            # ★ 验收项：日志必须能看到【完整 token 列表】与【规划耗时(ms)】
            self._log(f"自动规划成功：map={out.map_path} {out.mode} {out.engine_line} "
                      f"耗时 {out.elapsed_ms:.1f} ms / {len(out.tokens)} token / "
                      f"{out.route_len_m:.2f} m / 倒车段 {out.reverse_segments} / "
                      f"A* 代价 {out.cost:.2f}"
                      f" / 穿洞 下={out.cross_lower or '?'} 上={out.cross_upper or '?'}")
            self._log(f"自动规划 token 列表（{len(out.tokens)}）：" + " ".join(out.tokens))
            self._load_tokens(out.tokens, source=f"{ROUTE_SOURCE_AUTO}({out.map_path})")
            return

        detail = (f"自动规划失败：map={out.map_path} {out.mode} 引擎 {out.engine or self._planner_engine} "
                  f"预算 {self._planner_budget_s:g}s → {out.error}")
        if self._planner_fallback:
            self._log(f"⚠️ {detail}；fallback_to_tokens=true → 回退 route.tokens/mission_blocks 并继续出发")
            self._route_source = ROUTE_SOURCE_MANUAL
            tokens = self.cfg.get("route.tokens") or self.cfg.get("mission_blocks")
            # 回退路线**也是要执行**的路线，所以同样吃情报换洞（否则"规划失败"
            #   会把穿洞格悄悄退回默认，而现场恰恰最需要这条兜底路线是安全的）。
            tokens, accepted = self._apply_intel_to_tokens(tokens, snap)
            self._intel_note["manual_substituted"] = {
                half: f"{old}→{new}" for half, (old, new) in accepted.items()}
            src = "route.tokens/mission_blocks(自动规划失败回退)"
            if accepted:
                src += "（无人机情报换洞）"
            self._load_tokens(tokens, source=src)
            return
        self._log(f"❌ {detail}；fallback_to_tokens=false → 停在 IDLE，不出发")
        raise AutoPlanBlocked(out.error)

    def _start_mission(self) -> None:
        """路线就绪 → 阶段 DRIVE + 事件 + 扫描下发（见模块 docstring）。

        lock 模式下这一句 `_request_scan` 会被单写者守卫直接吞掉（只留一条日得），
        值由 AimTask 独家下发。
        """
        self.bus.publish(EV_MISSION_STARTED, source=self.name,
                         route=len(self._route) if self._route else 0)
        self._write_phase(MissionPhase.DRIVE)
        if self._scan_mode == "continuous":
            self._request_scan(2)                   # ★ 2 = 持续，绝不用 1（会被降级成单趟）
        else:
            self._request_scan(1 if self._scan_mode == "waypoint" else 0)
        self._banner_start()

    def _engine_banner_text(self) -> str:
        """横幅里的引擎行：缺省引擎（tour）只印引擎名，新引擎额外印门禁结论。"""
        engine = str(self._plan_info.get("engine") or self._planner_engine)
        if engine == DEFAULT_PLAN_ENGINE:
            return f"引擎 {engine}"
        gate = "✅通过" if self._plan_info.get("gate_ok") else "❌未过"
        return f"引擎 {engine}（门禁{gate}：{self._plan_info.get('gate_detail') or '未记录'}）"

    def _cross_source_line(self) -> str:
        """本局穿洞格的**来源**（无人机情报 / 默认格 / 拒收）。**两个半场独立列出**。"""
        names = {"drone": "无人机情报", "default": "默认", "missing": "未收到情报",
                 "stale": "情报过期", "offline": "无人机离线", "no_link": "无链路",
                 "disabled": "链路已关", "rejected": "情报被拒"}
        parts: list[str] = []
        for half in BRIDGE_HALVES:
            used = self._intel_used.get(half, "") or "未配置"
            it = self._intel.get(half)
            if it is not None and it.from_drone and used == it.cell:
                parts.append(f"{half}=无人机情报({used})")
            elif it is not None and it.from_drone:
                parts.append(f"{half}=默认({used}，情报 {it.cell} 未生效)")
            elif it is not None and it.source == "rejected":
                sub = self._intel_note.get(f"reject_{half}") or it.reason
                parts.append(f"{half}=默认({used}，情报 {it.raw} 被拒：{sub})")
            else:
                parts.append(f"{half}={names.get(it.source if it else '', '未刷新')}({used})")
        return " ".join(parts)

    def _intel_summary_line(self) -> str:
        """一行情报摘要（新鲜度 + 缺情报的原因），现场不用翻日志。"""
        items = []
        for half in BRIDGE_HALVES:
            it = self._intel.get(half)
            items.append(f"{half}=" + (it.describe() if it is not None else "未刷新"))
        return "｜".join(items)

    def _banner_start(self) -> None:
        route = self._route
        if route is None:
            return
        # ★ 路线来源行：验收项要求横幅里能读到「路线来源：自动规划 / 手动配置」，
        #   自动规划时顺带把地图与规划耗时一起打出来（与上面的"自动规划成功"日志同源）。
        src_line = f"路线来源：{self._route_source or ROUTE_SOURCE_MANUAL}"
        if self._plan_info.get("ok"):
            src_line += (f"（map={self._plan_info.get('map_path')} "
                         f"耗时 {float(self._plan_info.get('elapsed_ms') or 0.0):.1f} ms "
                         # 横幅里看得见引擎与门禁结论（缺省 tour 不额外占字）
                         f"{self._engine_banner_text()} "
                         f"{'允许倒车' if self._plan_info.get('allow_reverse') else '无倒车'}）")
        banner([
            f"【任务开始】路线 {len(route)} 点 / {route.path_length():.2f} m / 穿洞 {route.crossing}",
            # 穿洞格的**来源**（无人机情报 / 默认 / 拒收原因）——现场第一个要看的行
            f"穿洞格来源：{self._cross_source_line()}｜情报 {self._intel_summary_line()}",
            # ★t7：停站台账按类型分列（射击/扫描/终点），不再把终点混进"第二个停站"
            self._stop_ledger_line(),
            (f"扫描模式 {self._scan_mode or '未设置'}（lock 模式**inert**："
             f"单写者=AimTask）" if self._gimbal_lock_enabled
             else f"扫描模式 {self._scan_mode or '未设置'}（continuous → scan_request=2）"),
            src_line,
        ], log=self._log)

    def _reset_belief(self, spec: str) -> None:
        """`reset:BLOCK,HEAD` / `mission.reset` → 把 belief 设为该格心 + 朝向。"""
        if not spec or not str(spec).strip():
            raise ValueError("复位串为空")
        block, yaw = parse_reset(spec)
        x, y = geo.block_to_meters(block)
        pose = geo.Pose(x, y, yaw)
        self.bb.update(self.name, **{P_POSE: pose})
        self.bb.update(self.name, **{P_ROUTE_INDEX: -1, P_ROUTE_S: 0.0})
        self._last_s = 0.0
        self._route_s = 0.0
        self._log(f"复位 belief：{block} ({x:.3f}, {y:.3f}) yaw={geo.deg(yaw):.1f}°")

    # ------------------------------------------------------------------ #
    #  路线载入
    # ------------------------------------------------------------------ #
    def _load_tokens(self, tokens, *, source: str = "route.tokens") -> int:
        if not tokens:
            raise ValueError(f"{source} 为空，无法载入路线")
        route = parse_mission(tokens, shoot_planner=self._shoot_planner, name=source)
        n = len(route)
        if n < 2:
            raise ValueError(f"路线只有 {n} 个航点（Path 至少需要 2 点）")
        self._install_route(route, source=source)
        self._started_at = self._clock()            # 计时起点 = 路线就绪（终点横幅用它算用时）
        self.bus.publish(EV_ROUTE_LOADED, source=self.name, route=n, blocks=list(route.blocks()))
        self._log(f"路线载入[{source}]：{route.describe()}")
        return n

    def _load_single_point(self, spec: str) -> int:
        """单点命令 `BLOCK,HEAD`（可带 `*` / `:SCAN` / `R:`）。

        ★ 单点**不经过 `parse_mission`**：只有 1 个航点，`Path` 会拒绝编译，
          而且 `parse_mission` 的"朝向后缀只允许出现在最后一个 token"对单点是恒真的，
          直接在末尾航点上落 `yaw` 更简单、也更贴合旧版 `send_waypoint.py` 的语义。
        """
        text = spec.strip()
        if not text:
            raise ValueError("单点命令缺少格名")
        heading = None
        if len(text) >= 2 and text[-2] == ",":
            heading = text[-1].lower()
            if heading not in ("x", "y"):
                raise ValueError(f"朝向后缀只能是 ,x 或 ,y：{spec!r}")
            text = text[:-2].strip()
        info = parse_token(text)
        block = info["block"]
        x, y = geo.block_to_meters(block)
        yaw = geo.heading_token_to_yaw(heading) if heading else geo.HEADING_X
        wp = Waypoint(block=block, x=x, y=y, yaw=yaw, reverse=info["reverse"],
                      shoot=info["shoot"], scan=info["scan"], token=info["token"])
        route = Route((wp,), name="单点命令")
        self._install_route(route, source="单点命令")
        self._started_at = self._clock()
        self.bus.publish(EV_ROUTE_LOADED, source=self.name, route=1, blocks=[block])
        self._log(f"单点命令：{info['token']}（单点不编译 Path；Tracker 侧投影不可用，"
                  f"只用于「停在该点」类调试）")
        return 1

    def _install_route(self, route: Route, *, source: str = "",
                       keep_progress: bool = False) -> None:
        """装上执行路线（**唯一的路线真源**）。

        `keep_progress=True` = **保留当前段进度**的中途换洞（决策点前改道）——
        旧行为把 `P_ROUTE_INDEX/P_ROUTE_S` 与全部进度台账复位成 0，等于"把车送回起点"。
        这里在复位前快照、复位后回填；`_route/_path/_stop_s` 仍按新路线重算（换洞只动
        穿洞三连 ⇒ 决策点及其之前的航点下标一一对应，保留进度自洽）。
        """
        snapshot: dict[str, Any] = {}
        if keep_progress:
            for name in ("_passed", "_arrived", "_next_wp", "_min_dist", "_arrived_by",
                         "_skipped_at", "_pending_skip", "_cross_skips", "_route_s",
                         "_last_s", "_handled_stops"):
                value = getattr(self, name, None)
                snapshot[name] = list(value) if isinstance(value, list) else value
            snapshot[P_ROUTE_INDEX] = self.bb.get(P_ROUTE_INDEX)
            snapshot[P_ROUTE_S] = self.bb.get(P_ROUTE_S)
        self._route = route
        self._route_name = source or route.name
        # ★ 单点路线（`单点 BLOCK,HEAD`）只有 1 个航点，而 `Path` 要求 ≥2 点，
        #   所以这里 `_path = None`；推进/终点/停站判定全部对 `_path is None` 做防御。
        self._path = Path(route) if len(route) >= 2 else None
        self._waypoint_count = len(route)
        self._passed = [False] * len(route)
        self._arrived = 0
        self._next_wp = 1
        self._handled_stops = set()
        self._passed_by_pivot = set()                # ★ 新轮次：旧通知的"已到达"作废
        # ★★ 逐点到达台账换轮次：新路线必须从零开始，否则上一轮的"最近接近"会把
        #    新路线里**同名不同索引**的航点误判成到过（路线自折叠时格名会重复）。
        self._min_dist = [math.inf] * len(route)
        self._arrived_by = {}
        self._skipped_at = {}
        self._pending_skip = {}
        self._cross_skips = 0
        self._stopctx = None
        self._last_stop_state = StopPointState()
        self._route_s = 0.0
        self._last_s = 0.0
        self._dist_to_next_stop = None
        self._finish_info = {}
        self._stop_s = {}
        if self._path is not None:
            self._stop_s = {i: self._path.s_at_waypoint(i) for i, _ in route.stop_points()}
            if (len(route) - 1) not in self._stop_s:     # 末点一定是一个"停站点"（终点）
                self._stop_s[len(route) - 1] = self._path.total_length
        self._stop_kind = self._classify_stops(route)
        self._stop_blocks = [route[i].block for i in sorted(self._stop_s)]
        # 本局**实际装进执行路线**的穿洞格（`Route.crossing` 由 parse/planner 填）。
        #   一旦路线载入就**冻结**：发车后晚到的情报只进遥测，不再改路线。
        self._intel_used = {str(k): str(v) for k, v in (route.crossing or {}).items()}
        self.bb.update(self.name, **{P_ROUTE: route})
        self.bb.update(self.name, **{P_ROUTE_INDEX: 0, P_ROUTE_S: 0.0})
        # 新一轮/新图都要把"已决策过的洞"清掉（下标会变）
        self._hold_decided_for = None
        if keep_progress:
            for name, value in snapshot.items():
                if name in (P_ROUTE_INDEX, P_ROUTE_S):
                    continue
                setattr(self, name, list(value) if isinstance(value, list) else value)
            if snapshot.get(P_ROUTE_INDEX) is not None:
                self.bb.update(self.name, **{P_ROUTE_INDEX: snapshot[P_ROUTE_INDEX],
                                             P_ROUTE_S: snapshot.get(P_ROUTE_S) or 0.0})
        # ★ 清掉上一轮残留的 PIVOT 通知：旧通知对新路线毫无意义，
        #   留着会（在最坏的同几何重载场景）把新轮次航点开局误判成"已到达"。
        self.bb.update(self.name, **{PIVOT_DONE_KEY: None})

    def _classify_stops(self, route: Route) -> dict[int, str]:
        """★t7 把 `_stop_s` 里的每个索引归到 `SHOOT` / `SCAN` / `END`。

        `END` = **末点**：它也在 `_stop_s` 里（终点减速/到点判定的锚），但
        `_next_stop_candidate()` 明确把它排除在三段式停站之外（由 FINISH 处理）——
        所以台账必须把它与"打靶停站"分开，否则现场会把终点误读成第二个射击停站。
        """
        last = len(route) - 1
        kinds: dict[int, str] = {}
        for idx in sorted(self._stop_s):
            wp = route[idx] if 0 <= idx < len(route) else None
            if idx == last:
                kinds[idx] = STOP_KIND_END
            elif wp is not None and wp.shoot:
                kinds[idx] = StopKind.SHOOT
            elif wp is not None and wp.scan:
                kinds[idx] = StopKind.SCAN
            else:                                   # 理论不可达：stop_points() 只产出 shoot/scan
                kinds[idx] = StopKind.SHOOT
        return kinds

    def stop_kinds(self) -> dict[int, str]:
        """停站点类型台账（索引 → `SHOOT`/`SCAN`/`END`）；测试与遥测共用一份口径。"""
        return dict(self._stop_kind)

    def stop_ledger(self) -> dict[str, Any]:
        """★t7 停站台账（机器可读）：`{索引: (格名, 类型)}` + 分类计数。

        `shoot` 才是"这趟有几个打靶停站"的答案；`end` 永远单独计数。
        """
        route = self._route
        items = []
        for idx in sorted(self._stop_s):
            block = route[idx].block if (route is not None and idx < len(route)) else ""
            items.append({"index": idx, "block": block,
                          "kind": self._stop_kind.get(idx, StopKind.SHOOT)})
        counts = {"shoot": 0, "scan": 0, "end": 0}
        for it in items:
            if it["kind"] == StopKind.SHOOT:
                counts["shoot"] += 1
            elif it["kind"] == StopKind.SCAN:
                counts["scan"] += 1
            elif it["kind"] == STOP_KIND_END:
                counts["end"] += 1
        return {"count": len(items), "counts": counts, "points": items}

    def _stop_ledger_line(self) -> str:
        """横幅用的停站台账行：**数量按类型分列 + 逐点标类型**。

        例：`停站点 3 个（射击 2 / 终点 1）：29:D3(SHOOT) 32:G3(SHOOT) 43:N5(终点)`
        —— 现场一眼就能判断"两个打靶停站到底生不生效"，不必再回去数 token 里的星号。
        """
        ledger = self.stop_ledger()
        labels = {StopKind.SHOOT: "射击", StopKind.SCAN: "扫描", STOP_KIND_END: "终点"}
        counts = ledger["counts"]
        parts = []
        for key, kind in (("shoot", StopKind.SHOOT), ("scan", StopKind.SCAN),
                          ("end", STOP_KIND_END)):
            if counts.get(key):
                parts.append(f"{labels[kind]} {counts[key]}")
        body = " ".join("%d:%s(%s)" % (it["index"], it["block"],
                                       labels.get(it["kind"], it["kind"]))
                        for it in ledger["points"])
        return f"停站点 {ledger['count']} 个（{' / '.join(parts) or '无'}）：{body}"

    # ------------------------------------------------------------------ #
    #  `SHOOT@<靶标>` 的候选链计算
    # ------------------------------------------------------------------ #
    def _shoot_planner(self, target: str) -> list[str]:
        """靶格 → 候选射击停站点（4 邻格中落在射击窗口内的）。

        语法：`SHOOT@G11` 是**指令不是坐标**（`route/parser.py`），由本回调展开成候选链。
        取窗口与候选保留数沿用现场键 `path_executor.shoot_distance_target` /
        `shoot_distance_min` / `shoot_distance_max`（缺省 0.35 / 0.20 / 0.45，与 map.yaml 一致）。
        """
        tx, ty = geo.block_to_meters(target)
        win_lo = float(self.cfg.get("path_executor.shoot_distance_min", 0.20))
        win_hi = float(self.cfg.get("path_executor.shoot_distance_max", 0.45))
        self.cfg.get("path_executor.shoot_distance_target", 0.35)
        keep = int(self.cfg.get("path_executor.shoot_candidates_keep", 3) or 3)

        row_idx, col_idx = geo.block_to_index(target)
        cands: list[tuple[float, str]] = []
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            r, c = row_idx + dr, col_idx + dc
            if not (0 <= r < geo.MAP_ROWS and 0 <= c < geo.MAP_COLS):
                continue
            block = geo.index_to_block(r, c)
            bx, by = geo.block_to_meters(block)
            dist = math.hypot(bx - tx, by - ty)
            if win_lo - _EPS <= dist <= win_hi + _EPS:
                cands.append((dist, block))
        cands.sort(key=lambda item: (item[0], item[1]))
        out = [block for _, block in cands[:keep]]
        return out or [target]                       # 兜底：靶标自身一格（旧实现的降级行为）

    # ------------------------------------------------------------------ #
    #  航点推进
    # ------------------------------------------------------------------ #
    def _current_s(self) -> float:
        """本拍沿程弧长。

        ★ 读 `P_ROUTE_S` 之前**必须验一致性**：该键可能是上一次任务残留的旧值
          （例如刚 `reset`、或本轮 Localization/Tracker 还没写过），
          拿它当"车已经走到哪"会让"到点/到站/终点"判定全部失真。
          验法是把它与"用当前位姿现算的弧长"比对：差得离谱就丢掉、用现算值，
          并把现算值写回（与 Localization 写的是**同一投影语义**，不会互相打架）。
        """
        path = self._path
        if path is None:                            # 单点路线：没有弧长概念
            return self._last_s
        pose = self.bb.get(P_POSE)
        cached = self.bb.get(P_ROUTE_S)
        s: float | None = None
        if pose is not None:
            proj = path.project(pose.x, pose.y, s_hint=self._last_s)
            self._last_lat_m = float(proj.dist)      # ★ 横向偏差（"是否贴在路径上"的判据）
            if isinstance(cached, (int, float)) and abs(float(cached) - proj.s) <= self._arrival_m:
                s = float(cached)                   # 与位姿一致 → 沿用（Localization 的权威值）
            else:
                s = float(proj.s)
                self.bb.update(self.name, **{P_ROUTE_INDEX: int(proj.seg_index), P_ROUTE_S: s})
        elif isinstance(cached, (int, float)):
            s = float(cached)
        if s is None:
            return self._last_s
        self._last_s = s
        self._route_s = s
        return s

    def _advance_waypoints(self) -> None:
        route, path = self._route, self._path
        if route is None or path is None:
            return
        pose = self.bb.get(P_POSE)
        s = self._current_s()
        notice = self._pivot_notice()                # 本拍读一次，循环里复用
        for i, wp in enumerate(route.waypoints):
            if self._passed[i]:
                continue
            if i == 0:
                self._passed[0] = True           # 起点不报"到达"
                continue
            # ★ 弧长判据必须与"车确实在附近"**合取**，不能单用：
            #   投影链会跳变/冻结（诊断见 docs/诊断_投影与顶点.md），实测 s 从 3.50 冲到 4.20 时，
            #   #11 F11 / #12 E11 / #13 F11 三个**从未走到**的点被默默判成"已到达" ——
            #   这既不诚实（没有 forced_skip 记录），又会让后续航点计数整体错位。
            #
            # ★★ 但"在附近"的门槛不能取 2×arrival_m(0.24 m)：纯追踪**正常切角**时车离格心
            #   会超过它 —— bench 实测 L10→L9→K9 的直角，车路过 L9 时离线心 **0.288 m**
            #   ⇒ `near` 恒假 ⇒ L9 **永不置位** ⇒ 停滞兜底 12 s 后把它当"不可达"补跳。
            #   一次跑下来从 #3 一路补跳到 #27（12 个），车光在等 12 s×12 就把时间耗光了。
            #
            # ★★★ 真正的护栏是**"刚越过"窗口**，不是距离：s 跳变会一次性跨过好几个航点，
            #   那些点的 `over` 远大于 arrival_m，窗口直接把它们挡住；而正常逐拍推进时
            #   每个点都在窗口内被逮住（10 Hz × 0.15 m/s = 15 mm/拍 ≪ 120 mm）。
            #   距离门槛据此放宽到 4×arrival_m(≈0.48 m)：放行正常切角，仍挡住"车根本不在
            #   这条路附近，是投影跑到别处去了"的情形。
            s_wp = path.s_at_waypoint(i)
            dist = None if pose is None else wp.distance_to(pose.x, pose.y)
            if dist is not None and dist < self._min_dist[i]:
                # ★ 历史最近距离（几何复核用）：车常在"s 还没到"的几拍前从格心旁穿过，
                #   等 s 跳过去那一拍它已经走远 —— 只看当拍距离就会把这种**真的到过**
                #   判成"没到"。本行是 finding D1 里"静默丢点"的第一道解药。
                self._min_dist[i] = float(dist)
            # ★ 末点停车判定收紧到 `_GOAL_STOP_M`(0.03，对齐 tools/sim.py)：末点若沿用
            #   `arrival_m`(0.12) 的到位余量，"s+近路径 / 距格心"会在离终点 0.12 m 处就
            #   把末点判成"已到" → FINISH 提前刹车 → 物理车停在格心前 ~11 cm
            #   （2026 直线标定 N10→A10 实测欠程；车端 scale=1.00003 证明不是里程计误差）。
            #   非末点航点的判据（余量、近路径、几何复核、越窗待定）完全不变。
            arrival = _GOAL_STOP_M if i == len(route) - 1 else self._arrival_m
            over = s + arrival - s_wp                  # >0 = 已越过该航点
            s_reached = over >= 0.0
            # ★★ "车在附近"的判据改成**贴近路径**（横向偏差 `_last_lat_m`），不是贴近格心：
            #   纯追踪的**正常切角**会离线心很远却仍在路径上 —— bench 实测 L10→L9→K9 的
            #   直角，车路过 L9 时离线心 **0.288 m**、但横向偏差只有 ~0.09 m。旧判据
            #   `dist ≤ 2×arrival_m(0.24 m)` 在这种正常切角下恒假 ⇒ L9 **永不置位** ⇒
            #   停滞兜底 12 s 后把它当"不可达"补跳，一次跑从 #3 补跳到 #27（12 个），
            #   240 s 全耗在等待上（到点 30 → 16、绕圈 3 → 12、跑不完全程）。
            #   而"投影跑到别处造成的假到点"里车离**路径**也远（§11 用例：横偏 0.35 m），
            #   两条判据由此区分开，互不误伤。
            lat = self._last_lat_m
            # ★ 末点"附近窗口"同步收紧：2×arrival_m(0.24 m) 对末点改为 2×_GOAL_STOP_M(0.06 m) ——
            #   与 sim.py 末点"已过垂面必须 ≤ _GOAL_STOP_M"语义对齐：信念刚越过终点垂面、
            #   离格心还有 0.06~0.24 m 时，不再被 `s_reached and near` 提前判成到达。
            #   非末点航点的 2×arrival_m 切角放行窗口一字不变。
            near = ((lat is not None and lat <= 0.30)
                    or dist is None or dist <= 2.0 * arrival)
            why = ""
            if (s_reached and near) or (dist is not None and dist <= arrival):
                why = "s+近路径" if s_reached else "距格心"
            elif pose is not None and self._pivot_notice_accepts(notice, i, s_wp, s, dist):
                # ★ PIVOT 完成通知判到达：原地掉头后车原路返回，上面的两条判据恒假
                #   （pivot_dist_m > arrival_m），见 `PIVOT_DONE_KEY` 注释与
                #   `_pivot_notice_accepts` 的护栏说明。bench 实测 E11/A6/B6 三处
                #   此前就是在这里判不到、12 s 后被拐角绕圈兜底强制跳过。
                why = "PIVOT 通知"
                self._passed_by_pivot.add(i)
            elif s_reached and self._min_dist[i] <= self._close_m:
                # ★★ 几何复核（finding D1 的第二道解药）：s 跳变把"刚越过窗口"跳过去了，
                #    但**历史最近距离**证明车确实从格心附近穿过 ⇒ 如实登记为到达，
                #    而不是既不记到达、也不记跳过的静默丢弃。
                why = f"最近接近 {self._min_dist[i]:.3f}m"
            if why:
                revoked = self._pending_skip.pop(i, None)   # ★ 待定被"后来居上"的到达撤销
                self._passed[i] = True
                self._arrived += 1
                self._arrived_by[i] = why
                self._next_wp = i + 1
                self.bus.publish(EV_WAYPOINT_REACHED, source=self.name, index=i,
                                 block=wp.block, s=round(path.s_at_waypoint(i), 3),
                                 reason=why)
                self._log(f"到点 {i}/{len(route) - 1} {wp.block}"
                          f"（s={path.s_at_waypoint(i):.2f} m，依据 {why}）"
                          + ("（撤销先前的越窗待定）" if revoked else ""))
                continue
            if s_reached and over >= self._skip_over_m:
                # ★★ 显式记账（finding D1 的第三道解药，**绝不静默丢点**）：
                #    s 已越过这么多，而车**从头到尾**都没进过 `_close_m`(半格) ⇒ 它确实
                #    还没走到。这里只把它登记成"越窗**待定**"，**不置位 `_passed`** ——
                #    投影跳变的同一拍车可能马上又靠回来，那时上面的判据会正常登记到达
                #    （bench 实测雷 I8：基线里是「到点 8/45」，一旦立即判死就变成跳点、
                #    雷覆盖从 8/8 掉到 7/8）。真正定案放在 `_close_ledger()`（任务收尾），
                #    所以"无静默丢点"在收尾时仍然成立，而正常到达的机会一点没少。
                detail = (f"s 已越过 {over:.2f} m（> {self._skip_over_m:.2f} m），"
                          f"但最近只到 {self._min_dist[i]:.3f} m"
                          f"（> 复核半径 {self._close_m:.3f} m，横向偏差 {lat}）")
                if i not in self._pending_skip:
                    self._pending_skip[i] = detail
                    self._log(f"⚠️ 越窗待定（显式记账，不静默丢点）：航点 {wp.block}（#{i}）{detail}")

    def _detect_stall_and_skip(self) -> None:
        """拐角绕圈兜底：弧长停滞 / 原地绕圈 → 强制跳过不可达航点。

        ★ 为什么需要（仿真实测，见 docs/C线_仿真器报告.md §4）：
          当定位噪声或里程比例误差让"车的投影弧长 s"无法越过下一个航点时，
          `_advance_waypoints` 的 `s + arrival_m >= s_at_waypoint(i)` 永远不成立，
          车就会在拐角原地转圈（实测噪声 0.01m/拍 → 46.8s 卡死、末点误差 2.96m）。
          按 readme【八】27 "宁可漏打，不可卡死"，这里主动把卡住的点判为不可达并跳过，
          让任务继续，绝不原地空转。

        判据（任一成立即触发）：
          ① s 在 `mission.stall_timeout_s` 内前进不足 `arrival_m`；
          ② 停滞期间车累计转满 `mission.stall_yaw_deg`（原地绕圈特征）。
        ★ 例外：Tracker 正在执行 PIVOT 有界机动时**整体不计停滞**（见 `_pivot_in_progress`）——
          180° 折返的 PIVOT 实测要 ~10.8s、累计转 ~180°，与上面两条判据**在数值上正面撞车**：
          PIVOT 被修成"真的会转"之后，看门狗把每一次正常折返都当成绕圈（实闭环 bench
          实测 F11/A6/D6 三处误跳点）。PIVOT 自带 14s 硬超时且必然自终止，无卡死风险。
        触发后：把"下一个未到达航点"强制标记为已过（跳过），发事件并告警；
                若该点是停站点（打靶），跳过意味着放弃这一击 —— 宁可漏打，不可卡死。
        """
        route, path = self._route, self._path
        if route is None or path is None:
            self._reset_stall()
            return

        # ---- ★ 有界机动（PIVOT）期间不计停滞：时间窗整体重置（不是忽略本次判定）----
        if self._pivot_in_progress():
            self._reset_stall()
            return

        s = self._last_s
        pose = self.bb.get(P_POSE)
        yaw = pose.yaw if pose is not None else None
        # ★维护「车实际静止」窗口（跳点安全闸的输入，见 _update_still_window）
        self._update_still_window(pose)

        # ---- 维护停滞窗口 ----
        progressed = self._stall_last_s is None or (s - self._stall_last_s) >= self._arrival_m
        if progressed:
            self._reset_stall()
        if self._stall_since is None:
            self._stall_since = self._clock()
            self._stall_last_s = s
            self._stall_last_yaw = yaw
            self._stall_yaw_sum = 0.0
        elif yaw is not None and self._stall_last_yaw is not None:
            self._stall_yaw_sum += abs(geo.ang_diff(yaw, self._stall_last_yaw))
            self._stall_last_yaw = yaw

        # ---- 判定 ----
        stalled_time = self._clock() - self._stall_since
        yaw_circled = self._stall_yaw_sum >= math.radians(self._stall_yaw_deg)
        if stalled_time < self._stall_timeout_s and not yaw_circled:
            return

        # ---- 找下一个未到达航点（跳过它）----
        next_idx = next((i for i in range(1, len(route)) if not self._passed[i]), None)
        if next_idx is None:
            self._reset_stall()
            return
        wp = route[next_idx]
        # ---- ★★静止闸：车实际没动 ⇒ 禁止跳点（H8 卡死的「假完赛」安全洞）----
        #  为什么必须拦：s 冻结时每 12 s 跳一个点，车一步没动也能把航点逐个跳完 ⇒
        #  _passed 全 True ⇒ 任务报 FINISH（假完赛）。此时正确的行为是「停站 + 如实报警」，
        #  路线进度一动不动，等人来处理（车一动/s 一前进，窗口自然复位，跳点能力恢复）。
        #  ⚠️ 只拦「车静止」；车在动（切角抖动/原地绕圈）时照旧跳点（现有行为不变）。
        if self._still_n >= _STILL_TICKS:
            self.stall_skip_blocked += 1
            if not self._stall_block_logged:
                self._stall_block_logged = True
                self._log(f"⛔ 车静止（连续 {self._still_n} 拍位姿几乎不动）→ 禁止跳点："
                          f"航点 {wp.block}（#{next_idx}）停滞 {stalled_time:.1f}s，"
                          f"如实上报 stall（不推进路线 / 不报 FINISH，等人处理）")
            return
        self._passed[next_idx] = True
        self._forced_skips += 1
        self._next_wp = next_idx + 1
        # ★ 进台账（finding D1）：兜底跳过与"越窗跳过"共用一本账，收尾时逐点对账。
        self._skipped_at[next_idx] = (f"停滞兜底 {stalled_time:.1f}s / 累计转 "
                                      f"{math.degrees(self._stall_yaw_sum):.0f}°")
        self._log(f"⚠️ 拐角绕圈兜底：航点 {wp.block}（#{next_idx}）停滞 "
                  f"{stalled_time:.1f}s / 累计转 {math.degrees(self._stall_yaw_sum):.0f}° 仍不可达，"
                  f"强制跳过（宁可漏点，不可卡死）")
        self.bus.publish(EV_WAYPOINT_REACHED, source=self.name, index=next_idx,
                         block=wp.block, forced_skip=True, reason="stall")
        self._reset_stall()

    def _pivot_in_progress(self) -> bool:
        """Tracker 是否正在执行 PIVOT 机动（读它的遥测快照，不直接依赖其内部状态）。

        ★ 为什么停滞判定必须让位：180° 折返的 PIVOT 实测要 **~10.8s**、车累计转 **~180°**，
          而绕圈判据是"`stall_timeout_s`(=12s) 内 s 不前进" + "累计转满 `stall_yaw_deg`"
          —— **正常折返与"原地绕圈"在数值上完全撞车**。PIVOT 被修成真的会转之后，
          看门狗就会把每一次正常折返都判成绕圈跳点（实闭环 bench 实测 F11/A6/D6 三处，
          跳点=漏压雷/漏打靶）。PIVOT 自带 14s 硬超时并且必然自终止，没有卡死风险，
          所以有界机动期间整个停滞窗口重置（而不是只跳过这一次判定）。
        """
        snap = self.bb.get(P_TELEMETRY)
        if not isinstance(snap, dict):
            return False
        cmd = snap.get(TRACKER_CMD_KEY)
        if not isinstance(cmd, dict):
            return False
        return str(cmd.get("pivot_phase", PivotPhase.IDLE)) != PivotPhase.IDLE

    # ------------------------------------------------------------------ #
    #  ★PIVOT 完成通知（Tracker 写、本任务读）
    # ------------------------------------------------------------------ #
    def _pivot_notice(self) -> tuple | None:
        """读 PIVOT 完成/放弃通知：`(顶点航点索引, 顶点弧长, 结局)`；无/非法 → None。"""
        value = self.bb.get(PIVOT_DONE_KEY)
        if isinstance(value, tuple) and len(value) == 3:
            return value
        return None

    def _pivot_notice_accepts(self, notice, idx: int, s_wp: float, s: float,
                              dist: float | None) -> bool:
        """PIVOT 通知是否可把航点 `idx` 判为到达。

        护栏（防跨轮次残留/竞态误判，bench 实测缺一不可）：
          ① 航点索引与顶点弧长**逐位**对上（每条路线各顶点弧长唯一）；
          ② 车当前仍在顶点附近：`s_wp - s` 与几何距离都不得超过
             `_pivot_accept_m`（≈3×pivot_dist_m）——真完成时二者 ≈ pivot_dist_m，
             而残留通知里车在起点，差好几段长。
        """
        if notice is None or notice[0] != idx:
            return False
        if abs(float(notice[1]) - s_wp) > 1e-6:
            return False
        win = self._pivot_accept_m
        if (s_wp - s) > win:
            return False
        if dist is not None and dist > win:
            return False
        return True

    def _update_still_window(self, pose) -> None:
        """维护「车实际静止」窗口：连续 N 拍位移 < ε 且瞬时速度 < v_eps。

        数据只用 P_POSE（不依赖 Tracker 遥测，任务之间保持黑盒契约）：
        · 位移 < _STILL_DISP_M（0.01 m/拍）且 速度 < _STILL_SPEED_MPS（0.03 m/s）⇒ 计数 +1；
        · 任何一拍照样移动 ⇒ 清零（重新起算）；
        · 无位姿/时钟异常 ⇒ 清零（**不拦**跳点：拿不到位姿时保持旧行为，避免误伤）。
        """
        if pose is None:
            self._still_prev = None
            self._still_n = 0
            self._still_last_t = None
            return
        now = self._clock()
        cur = (float(pose.x), float(pose.y), now)
        if self._still_prev is None or self._still_last_t is None:
            self._still_prev = cur
            self._still_last_t = now
            self._still_n = 0
            return
        disp = math.hypot(cur[0] - self._still_prev[0], cur[1] - self._still_prev[1])
        dt = now - self._still_last_t
        speed = disp / dt if dt > 1e-6 else float("inf")
        if disp <= _STILL_DISP_M and speed <= _STILL_SPEED_MPS:
            self._still_n += 1
        else:
            self._still_n = 0
        self._still_prev = cur
        self._still_last_t = now

    def _reset_stall(self) -> None:
        self._stall_since = None
        self._stall_last_s = 0.0
        self._stall_last_yaw = None
        self._stall_yaw_sum = 0.0
        self._stall_block_logged = False

    def _next_stop_candidate(self) -> int | None:
        """下一个需要停车的停站点索引；**末点不算停站点**。

        ★ 末点虽然也在 `_stop_s` 里（它是"终点减速/终点判定"的锚），但它由 `FINISH`
          处理，绝不能进三段式停站流程 —— 否则终点会被当成一个打靶点停在那里。
        """
        last = self._waypoint_count - 1
        for idx in sorted(self._stop_s):
            if idx == last:
                continue
            if idx in self._handled_stops:
                continue
            # ★ 经 PIVOT 通知判到达的停站点**不**跳过：它还没走三段式停站
            #   （`_should_trigger_stop` 里同样的通知会让它立刻触发进站）。
            if idx < len(self._passed) and self._passed[idx] and idx not in self._passed_by_pivot:
                continue
            return idx
        return None

    def _dist_to_stop(self, idx: int, pose) -> float | None:
        if pose is None or self._route is None or idx >= len(self._route):
            return None
        wp = self._route[idx]
        return math.hypot(wp.x - pose.x, wp.y - pose.y)

    # ------------------------------------------------------------------ #
    #  驱动
    # ------------------------------------------------------------------ #
    def _drive(self) -> None:
        self._advance_waypoints()
        self._detect_stall_and_skip()     # ★ 拐角绕圈兜底：宁可漏点，不可卡死
        idx = self._next_stop_candidate()
        if idx is None:
            self._dist_to_next_stop = None
            self._write_phase(MissionPhase.DRIVE)
            return
        pose = self.bb.get(P_POSE)
        dist = self._dist_to_stop(idx, pose)
        self._dist_to_next_stop = dist
        s = self._current_s()
        if self._should_trigger_stop(idx, s, dist):
            self._enter_stop_point(idx, dist)
            return
        approach_m = self._approach_m
        self._write_phase(MissionPhase.APPROACH if (dist is not None and dist <= approach_m)
                          else MissionPhase.DRIVE)

    def _should_trigger_stop(self, idx: int, s: float, dist: float | None) -> bool:
        """到点判据：沿程已过停站点弧长，或几何距离进 `stop_radius_m`（旧版 0.17 m）。

        ★ 弧长与半径**取或**：弧长判据在"路线自交/投影跳段"时可能提前触发，
          半径判据在"车贴着点但投影没跟上"时兜底，两者互补（旧版只有半径 + 开过头补触发）。
        ★ PIVOT 通知也算：停站点恰是掉头顶点时，正常两条判据同样够不着
          （pivot_dist_m > arrival_m），但车已在顶点跟前完成掉头 → 照常进三段式。
        """
        path = self._path
        if path is not None and s + self._stop_radius_m >= self._stop_s.get(idx, math.inf):
            return True
        if dist is not None and dist <= max(self._stop_radius_m, self._arrival_m):
            return True
        if self._pivot_notice_accepts(self._pivot_notice(), idx,
                                      self._stop_s.get(idx, math.inf), s, dist):
            return True
        return False

    # ------------------------------------------------------------------ #
    #  停站点三段式（每段独立超时，绝不阻塞）
    # ------------------------------------------------------------------ #
    def _enter_stop_point(self, idx: int, dist: float | None) -> None:
        route = self._route
        if route is None:
            return
        wp = route[idx]
        now = self._clock()
        kind = StopKind.SCAN if (wp.scan and not wp.shoot) else StopKind.SHOOT
        shots = int(self.bridge.shoot_count())
        self._hit_this_stop = False          # 每个停站点独立计"是否已打出去"
        self._stopctx = _StopContext(
            index=idx, block=wp.block, kind=kind, phase=StopPhase.SETTLE,
            started_at=now, phase_started_at=now,
            shots_at_arrival=shots,
            shots_needed=(self._shots_per_stop if kind == StopKind.SHOOT else 0),
            detail=f"到站（距离 {dist:.3f} m，累计发数 {shots}）" if dist is not None
                   else f"到站（累计发数 {shots}）",
        )
        self._write_phase(MissionPhase.STOP)
        self._publish_stop_state(now)
        self.bus.publish(EV_ARRIVED_STOP_POINT, source=self.name, index=idx, block=wp.block,
                         kind=kind, shots=shots)
        self._log(f"★ 停站点 {idx} {wp.block}（{kind}）{self._stopctx.detail}"
                  f"；三段超时 SETTLE>{self._settle_grace_s:.1f}s "
                  f"ACQUIRE>{self._lock_timeout_s:.1f}s FIRE>{self._shot_timeout_s:.1f}s"
                  f"；★到站硬死线 {self._stop_deadline_s:.1f}s（发数增量 {self._stopctx.shots_needed} "
                  f"或到点，先到即离）")

    def _run_stop_point(self) -> None:
        stop = self._stopctx
        if stop is None:                            # 阶段与上下文不一致 → 安全回落
            self._write_phase(MissionPhase.DRIVE)
            return
        now = self._clock()
        # ★★ 用户指令 4（2026-10-11）：**到站级 30s 硬死线**，先于三段判定执行。
        #   口径："发数增量 ≥ shots_needed（=10，打完全部弹药）或 到站起 30s，两者先到即离站；
        #   即使 0 发，30s 到点也必须离站。"
        #   放行出口与三段超时**同一条**：`_abandon_stop` → `_close_stop_point`
        #   → phase=DONE → 放行下一个候选（结构不改，只是多一个更上位的死线）。
        if (now - stop.started_at) >= self._stop_deadline_s:
            shots_now = int(self.bridge.shoot_count())
            self._abandon_stop(
                stop, f"到站硬死线 {(now - stop.started_at):.2f}s"
                      f" ≥ {self._stop_deadline_s:.2f}s"
                      f"（发数增量 {shots_now - stop.shots_at_arrival}/{stop.shots_needed}）"
                      f" → 强制离站（0 发也不例外）")
            self._publish_stop_state(now)
            return
        # ---- 红面早退（先于三段判定；到站硬死线仍在最上面）----
        #   语义：靶板一面蓝一面红、蓝面朝向每局随机但恒定 ⇒ 本侧只见红面时，等到 30s 硬死线
        #   也只会白等。锁定红累计 ≥ 阈值的**结论**由 AimTask 写进 `P_AIM_RED`
        #   （它知道类别抖动/确认帧数），Mission 只做两件事：读标志 + 自己再核"已停留够久"。
        #   ⚠️ 只在 ACQUIRE/FIRE 段生效：SETTLE 段还没开始看靶，早退会把"还没瞄"当成"红面"。
        if (stop.kind == StopKind.SHOOT
                and stop.phase in (StopPhase.ACQUIRE, StopPhase.FIRE)):
            red = self.bb.get(P_AIM_RED)
            if isinstance(red, dict) and red.get("confirmed"):
                dwell = now - stop.started_at
                if dwell >= self._red_early_leave_s - 1e-9:
                    self._abandon_stop(
                        stop, f"红面早退：锁定红已持续 {float(red.get('run_s', 0.0)):.2f}s"
                              f" ≥ {self._red_early_leave_s:.2f}s，"
                              f"本停站点 {stop.block} 已停留 {dwell:.2f}s"
                              f" → 提前离站去下一个射击停站")
                    self._publish_stop_state(now)
                    return
        if stop.kind == StopKind.SCAN:
            self._run_scan_stop(stop, now)
            return
        if stop.phase == StopPhase.SETTLE:
            self._phase_settle(stop, now)
        elif stop.phase == StopPhase.ACQUIRE:
            self._phase_acquire(stop, now)
        elif stop.phase == StopPhase.FIRE:
            self._phase_fire(stop, now)
        else:
            self._abandon_stop(stop, f"未知阶段 {stop.phase}")
        self._publish_stop_state(now)

    # ---- 段 1：SETTLE ----
    def _phase_settle(self, stop: _StopContext, now: float) -> None:
        pose = self.bb.get(P_POSE)
        moved = None
        if pose is not None and stop.last_x is not None and stop.last_y is not None:
            moved = math.hypot(pose.x - stop.last_x, pose.y - stop.last_y)
        if pose is not None:
            stop.last_x, stop.last_y = pose.x, pose.y

        speed = self._last_speed()
        # 停稳 = 位置连续不变 **或** 速度近零（两者任一成立就武装，避免过站慢速时白等）
        if (moved is not None and moved <= self._settle_eps_m) or \
                (speed is not None and abs(speed) <= self._settle_speed_mps):
            if stop.settle_armed_at is None:
                stop.settle_armed_at = now
                stop.detail = f"停稳计时开始（Δ={moved if moved is not None else float('nan'):.4f} m，"
                stop.detail += f"v={speed if speed is not None else float('nan'):.3f} m/s）"

        if stop.settle_armed_at is not None:
            if (now - stop.settle_armed_at) >= self._settle_time_s:
                self._goto_phase(stop, StopPhase.ACQUIRE, now,
                                 f"停稳 {(now - stop.settle_armed_at):.2f}s"
                                 f" ≥ {self._settle_time_s:.2f}s")
                return
        if (now - stop.phase_started_at) >= self._settle_grace_s:
            self._goto_phase(stop, StopPhase.ACQUIRE, now,
                             f"停稳超时 {(now - stop.phase_started_at):.2f}s"
                             f" ≥ {self._settle_grace_s:.2f}s → 降级放行")

    # ---- 段 2：ACQUIRE ----
    def _phase_acquire(self, stop: _StopContext, now: float) -> None:
        if stop.kind == StopKind.SCAN:
            self._goto_phase(stop, StopPhase.FIRE, now, "扫描停站：跳过锁定")
            return
        status = int(self.bridge.aim_status())
        obs = self.bridge.vision(0.5)
        has_target = bool(obs is not None and getattr(obs, "has_target", False))
        if status == AimStatus.TRACK and has_target:
            self._goto_phase(stop, StopPhase.FIRE, now, "云台 TRACK 且有目标")
            return
        if (now - stop.phase_started_at) >= self._lock_timeout_s:
            self._abandon_stop(stop, f"锁定超时 {(now - stop.phase_started_at):.2f}s"
                                     f" ≥ {self._lock_timeout_s:.2f}s"
                                     f"（aim={AimStatus.name(status)} 目标={has_target}）")

    # ---- 段 3：FIRE ----
    def _phase_fire(self, stop: _StopContext, now: float) -> None:
        if stop.kind == StopKind.SCAN:
            if (now - stop.phase_started_at) >= self._scan_stop_timeout_s:
                self._complete_stop(stop, f"扫描停站完成（{self._scan_stop_timeout_s:.1f}s）")
            return
        self._set_fire_gate(True)                   # ★ 放行门：进本段即打开（幂等）
        delta = int(self.bridge.shoot_count()) - stop.shots_at_arrival
        if delta >= stop.shots_needed:
            # 这一路是"真的打出去了"的唯一证据（发数增量达标），
            #   第二个射击停站据此跳过（见 `_complete_stop`）。
            self._hit_this_stop = True
            self._complete_stop(stop, f"发数增量 {delta} ≥ {stop.shots_needed} → 命中窗口结束")
            return
        if (now - stop.phase_started_at) >= self._shot_timeout_s:
            self._set_fire_gate(False)
            self._abandon_stop(stop, f"开火超时 {(now - stop.phase_started_at):.2f}s"
                                     f" ≥ {self._shot_timeout_s:.2f}s（增量 {delta}"
                                     f"/{stop.shots_needed}）")

    # ---- 扫描停站（`F2:SCAN_STOP`）：不停顿打靶，只保证到位停稳 ----
    def _run_scan_stop(self, stop: _StopContext, now: float) -> None:
        if stop.phase == StopPhase.SETTLE:
            self._phase_settle(stop, now)
            return
        if stop.phase in (StopPhase.ACQUIRE, StopPhase.FIRE):
            if stop.phase == StopPhase.ACQUIRE:
                stop.phase = StopPhase.FIRE
                stop.phase_started_at = now
            if (now - stop.phase_started_at) >= self._scan_stop_timeout_s:
                self._complete_stop(stop, f"扫描停站完成（{self._scan_stop_timeout_s:.1f}s）")
            return
        self._abandon_stop(stop, f"未知阶段 {stop.phase}")

    # ---- 三段收尾 ----
    def _goto_phase(self, stop: _StopContext, phase: str, now: float, detail: str) -> None:
        self._set_fire_gate(False)                  # ★ 每段结束都关闸（旧版 2181–2183 的纪律）
        stop.phase = phase
        stop.phase_started_at = now
        stop.settle_armed_at = None
        stop.detail = detail
        self._log(f"停站点 {stop.index} {stop.block}：{detail} → {phase}")

    def _complete_stop(self, stop: _StopContext, detail: str) -> None:
        self._set_fire_gate(False)
        stop.detail = detail
        self._log(f"停站点 {stop.index} {stop.block} 完成：{detail}")
        skip_next = bool(self._hit_this_stop and self._skip_second_after_hit
                         and stop.kind == StopKind.SHOOT)
        self._hit_this_stop = False
        self._close_stop_point(stop)
        if skip_next:
            self._skip_following_shoot_stop(stop)

    def _skip_following_shoot_stop(self, done: _StopContext) -> None:
        """本停站点已打出去 ⇒ 跳过**下一个射击停站**（第二个站只为看另一面）。

        只跳"射击停站"，扫描停站（`F2:SCAN_STOP` 之类）照常执行 —— 它的作用是定向扫描，
        与"已经打过靶"无关。找不到下一个射击停站就什么都不做（末点/单停站路线=零副作用）。
        """
        route = self._route
        if route is None:
            return
        for idx in sorted(self._stop_s):
            if idx <= done.index or idx in self._handled_stops:
                continue
            wp = route[idx]
            if not wp.shoot:
                continue
            self._handled_stops.add(idx)
            self._log(f"跳过射击停站 {idx} {wp.block}："
                      f"停站点 {done.index} {done.block} 已打出去（发数增量达标）"
                      f" ⇒ 不再为看另一面绕行")
            return

    def _abandon_stop(self, stop: _StopContext, detail: str) -> None:
        self._set_fire_gate(False)
        stop.detail = detail
        self._log(f"停站点 {stop.index} {stop.block} 放弃：{detail}（放行下一个候选）")
        self._close_stop_point(stop)

    def _close_stop_point(self, stop: _StopContext) -> None:
        stop.phase = StopPhase.DONE
        self._handled_stops.add(stop.index)
        self._last_stop_state = self._build_stop_state(stop)
        self.bus.publish(EV_STOP_POINT_DONE, source=self.name, index=stop.index,
                         block=stop.block, kind=stop.kind, detail=stop.detail)
        self._stopctx = None
        self._write_phase(MissionPhase.DRIVE)

    def _publish_stop_state(self, now: float) -> None:
        stop = self._stopctx
        state = self._build_stop_state(stop) if stop is not None else self._last_stop_state
        self.bb.set(P_STOP_POINT, state, source=self.name)

    def _build_stop_state(self, stop: _StopContext) -> StopPointState:
        return StopPointState(
            index=stop.index, block=stop.block, kind=stop.kind, phase=stop.phase,
            started_at=stop.started_at, phase_started_at=stop.phase_started_at,
            shots_at_arrival=stop.shots_at_arrival, shots_needed=stop.shots_needed,
            detail=stop.detail,
        )

    def _last_speed(self) -> float | None:
        """读 Tracker 最近一拍的速度（只读遥测键，不跨任务调用）。"""
        snap = self.bb.get(P_TELEMETRY)
        if not isinstance(snap, dict):
            return None
        cmd = snap.get("tracker_cmd")
        if not isinstance(cmd, dict):
            return None
        value = cmd.get("speed")
        return float(value) if isinstance(value, (int, float)) else None

    # ------------------------------------------------------------------ #
    #  放行门 / 扫描 / 阶段 / 遥测
    # ------------------------------------------------------------------ #
    def _set_fire_gate(self, allow: bool) -> None:
        """放行门是**锁存式**的：只在变化时发（避免 10 Hz 刷屏，同时保证状态可追）。"""
        allow = bool(allow)
        if allow == self._fire_gate_state:
            return
        self.bridge.publish_fire_gate(allow)
        self._fire_gate_state = allow

    def _request_scan(self, mode: int) -> None:
        """下发 `scan_request`；**lock 模式下 Mission 不是写者**（单写者铁律）。

        返回值/记录语义未变（`_scans` 仍只记"本任务真的发出去的"），
        所以"Mission 在 lock 模式下从不写 scan_request"可以**用 `_scans` 空/非空直接断言**。
        """
        mode = int(mode)
        if self._gimbal_lock_enabled:
            # 由 AimTask（`_sync_gimbal_scan`）独占：巡迹段 0、SHOOT 停站 2、离开/FINISH 回 0
            if self._log_throttle.allow("scan_single_writer", self._clock()):
                self._log(f"跳过 scan_request={mode}：云台扫描单写者 = AimTask"
                          f"（aim.gimbal_lock_enabled=true；mission.scan_mode="
                          f"{self._scan_mode or '未设置'} 在 lock 模式下 inert）")
            return
        if mode == 1 and 2 in self._scans:
            # ★ readme【十】31：持续扫描期间绝不能再发 1（会被降级成单趟）
            if self._log_throttle.allow("scan_downgrade", self._clock()):
                self._log("拒绝 scan_request=1：当前是持续扫描（2），单趟会把它降级")
            return
        self.bridge.publish_scan_request(mode)
        self._scans.append(mode)

    def _write_phase(self, phase: str) -> None:
        if phase == self._phase:
            return
        old, self._phase = self._phase, phase
        self.bb.set(P_MISSION_PHASE, phase, source=self.name)
        if self._log_throttle.allow(f"phase:{old}->{phase}", self._clock()):
            self._log(f"阶段 {old} → {phase}")

    def _enter_safe_stop(self) -> None:
        reasons = self.bb.get(P_SAFETY)
        if self._phase != MissionPhase.SAFE_STOP:
            self._set_fire_gate(False)
            self._log(f"安全接管：{getattr(reasons, 'reasons', ())} → SAFE_STOP")
        self._write_phase(MissionPhase.SAFE_STOP)

    def _recover_from_safe_stop(self) -> None:
        """安全恢复后回到 DRIVE（停站点上下文若还在就继续那一段，不重来）。"""
        if self._stopctx is not None and self._stopctx.active:
            self._write_phase(MissionPhase.STOP)
        else:
            self._write_phase(MissionPhase.DRIVE)
        self._log("安全恢复 → 继续任务")

    def _publish_state(self, now: float) -> None:
        snap = self.bb.get(P_TELEMETRY)
        data = dict(snap) if isinstance(snap, dict) else {}
        data[MISSION_STATE_KEY] = self.state()
        self.bb.set(P_TELEMETRY, data, source=self.name)
        self._publish_stop_state(now)

    # ------------------------------------------------------------------ #
    #  终点（锁存）
    # ------------------------------------------------------------------ #
    def _check_finish(self) -> None:
        route, path = self._route, self._path
        if route is None or path is None or self._finish_latched:
            return
        if self._phase in (MissionPhase.IDLE, MissionPhase.SAFE_STOP):
            return
        last = len(route) - 1
        self._advance_waypoints()                   # 先补最后一次推进（末点也要报到达）
        s = self._current_s()
        # ★ 末点终点判据收紧到 `_GOAL_STOP_M`(0.03，对齐 tools/sim.py)：`arrival_m`(0.12)
        #   会让车在离终点 0.12 m 处触发 FINISH 提前刹车（2026 直线标定欠程 11 cm 的根因）。
        if not (self._passed[last] or s + _GOAL_STOP_M >= path.s_at_waypoint(last)):
            return

        self._finish_latched = True
        self._auto_start_consumed = True            # ★ 到终点后不再自动发车（回 IDLE 待命）
        self._close_ledger()                        # ★ 收尾：把"还悬着"的航点显式记账
        elapsed = (self._clock() - self._started_at) if self._started_at is not None else 0.0
        drift = 0.0
        fix = self.bb.get("fix")
        if fix is not None:
            drift = float(getattr(fix, "max_drift_m", 0.0) or 0.0)
        shots = int(self.bridge.shoot_count())
        hits = int(self.bridge.take_hits())
        self._finish_info = {"elapsed_s": round(elapsed, 2), "shots": shots, "hits": hits,
                             "drift_m": round(drift, 3), "waypoints": len(route)}
        # ★T27 段回报：Mission 的 FINISH = 本段最后一个航点到达/弧长达标 ⇒ 段完成。
        #   完成单与 segment_id 绑定；跨段重启由 request_segment() 清空（不会串段）。
        if self._segment_active:
            pose = self.bb.get(P_POSE)
            pose_block = geo.meters_to_block(pose.x, pose.y) if pose is not None else ""
            done = {
                "segment_id": int(self._segment_id),
                "target_cell": str(self._segment_target),
                "block": route.waypoints[last].block,
                "pose_block": pose_block,
                "known_count": int(self._segment_known),
                "elapsed_s": round(elapsed, 2),
                "waypoints": len(route),
                "finished_at": round(float(self._clock()), 3),
                "finish_info": dict(self._finish_info),
            }
            with self._segment_lock:
                self._segment_active = False
                self._segment_done = done
            self._log(f"■ 段 {done['segment_id']} 完成：停在 {done['block']}"
                      f"（belief {pose_block or '?'} / 用时 {done['elapsed_s']:.2f}s / "
                      f"{done['waypoints']} 点 / known {done['known_count']} 格）")
        self._write_phase(MissionPhase.FINISH)
        self.bridge.publish_brake()
        # ★ 单写者：lock 模式下 FINISH 的"停扫 0"也归 AimTask（它看到阶段=FINISH 就回锁止）。
        #   这里若照旧直发 0，就又是"Mission 覆写 Aim"的第二处洞（`_scans` 也不再记账）。
        if self._gimbal_lock_enabled:
            pass
        else:
            self.bridge.publish_scan_request(0)     # 0 = 停扫（旧行为/对照档）
            self._scans.append(0)
        if not self._finish_done:
            self._finish_done = True
            self.bus.publish(EV_MISSION_FINISHED, source=self.name, **self._finish_info)
            acc = self.accounting()
            denom = acc["denominator"]
            rate = (acc["arrived"] / denom * 100.0) if denom else 0.0
            banner([
                "【任务结束】" + (f"路线 {len(route)} 点 / {route.path_length():.2f} m"),
                f"用时 {elapsed:.1f} s（{self._fmt_duration(elapsed)}）",
                f"发数 {shots} / 命中 {hits} / 漂移 {drift:.3f} m",
                f"停站点 {len(self._stop_s)} 个（完成 {len(self._handled_stops)}）",
                # ★★ 逐点到达台账（finding D1 的验收口径）：登记到达 / 显式跳过 / 无主航点
                f"逐点到点率 {acc['arrived']}/{denom} = {rate:.2f}%"
                f"（依据 {acc['arrived_by']}）",
                f"显式跳过 {acc['skipped_total']} 个"
                f"（停滞兜底 {acc['stall_skips']} + s 越窗 {acc['cross_skips']}）"
                + (f"：{list(acc['skipped'])}" if acc["skipped"] else ""),
            ], log=self._log)
            unexplained = acc["unexplained"]
            if unexplained:
                # ★ 不变量被破坏才会走到这里：说明还有航点既没登记到达、也没显式记账。
                #   修复前（finding D1）这里会列出 7 个"静默丢点"；修复后应恒为空。
                banner([f"❌ 存在 {len(unexplained)} 个**无主航点**（既非到达、也非显式跳过）："
                        f"{' '.join(f'{route.waypoints[i].block}#{i}' for i in unexplained)}"],
                       char="!", log=self._log)
            else:
                self._log("✅ 逐点台账闭合：全部航点要么登记到达、要么显式跳过（无静默丢点）")
            # ★ 车体覆盖复核（只报不改）：实测提出"雷 A9 心在 PIVOT 顶点差 12.7 cm、
            #   车体没盖住"。Mission 只知道"车离航点最近到过多少"，把这份数据摊开，
            #   供用 0.127 m 之类的阈值逐雷对账 —— 本行**不影响**任何判定。
            uncovered = [(i, v) for i, v in self.closest_approaches().items()
                         if i >= 1 and v >= 0.0 and v > self._arrival_m]
            if uncovered:
                self._log(f"航点最近接近复核（min_dist > 到达半径 {self._arrival_m:.3f} m "
                          f"⇒ 车体未必盖住格心）："
                          + " ".join(f"#{i}{route.waypoints[i].block}={v:.3f}m"
                                     for i, v in uncovered))

    def _reset_for_next_run(self) -> None:
        """★ 需求2（用户 2026-10-16）：终点结算后回 IDLE，进入"准备发车"状态。

        **只回阶段 + 重开终点锁存，不清空路线数据** —— 这样：
          · 阶段 FINISH → IDLE（车停住、不再推进），满足"到达 N5 后进入准备发车状态"；
          · `_route`/`_passed`/`_finish_info` 全保留，终点台账与成绩遥测复位后仍可查，
            也**不破坏**依赖"FINISH 后 `_passed` 仍存在"的回归哨兵（test_goal_stop §3）；
          · `_auto_start_consumed=True`（`_check_finish` 里已置）保证 auto_start=true 不会
            在复位后立刻又自动发车（`_route is not None` 也拦一次，双保险）。
        下一发车靠外部命令（`reset:`/`load:`/`单点`/`多点`）覆盖旧路线，或重启栈。
        """
        self._write_phase(MissionPhase.IDLE)
        self._finish_latched = False
        self._finish_done = False
        self._finish_pending_reset = False
        self._log("已复位：回到 IDLE 准备发车状态（路线/成绩保留；下一发车靠外部命令或重启栈）")

    def _close_ledger(self) -> None:
        """收尾清账：任务结束时若还有航点"悬着"（s 越过但没到判跳过的余量），显式记账。

        finding D1 的"绝不静默丢点"在**任何退出路径**上都要成立 —— 不能只在
        "s 越窗"那一条分支上成立。收尾时把这些点一次性记为显式跳过并告警，
        使 `accounting()['unexplained']` 恒为空。
        """
        route = self._route
        if route is None or self._path is None:
            return
        last = len(route) - 1
        s = self._last_s
        # ★ 含末点：`_check_finish` 允许"弧长达标"就判完成（`s + _GOAL_STOP_M >=
        #   s_at_waypoint(last)`），
        #   那种情况下 `_passed[last]` 仍是 False —— 若把他排除在外，台账就永远闭不上
        #   （`unexplained` 里会永远吊着末点）。这里把他一并清账，并如实写明"车没接近"。
        for i in range(1, len(route)):
            if self._passed[i] or i in self._skipped_at:
                continue
            wp = route[i]
            over = s + self._arrival_m - self._path.s_at_waypoint(i)
            self._passed[i] = True
            self._cross_skips += 1
            tail = "（末点：由「弧长达标」判完成，车未接近）" if i == last else ""
            # ① 之前记过"越窗待定"的，用它自己的依据定案（含它当时看到的 min_dist/横向偏差）；
            # ② 其余（从没触发过越窗、也没到达的）用收尾清账的依据。
            detail = self._pending_skip.get(i) or (
                f"任务收尾清账（s 越过 {over:+.2f} m，最近只到 "
                f"{self._min_dist[i]:.3f} m > {self._close_m:.3f} m）")
            self._skipped_at[i] = detail
            self._pending_skip.pop(i, None)
            reason = "s_over_step" if "s 已越过" in detail else "close_ledger"
            self._log(f"⚠️ 逐点定案（显式记账）：航点 {wp.block}（#{i}）{detail}{tail}")
            self.bus.publish(EV_WAYPOINT_REACHED, source=self.name, index=i,
                             block=wp.block, forced_skip=True, reason=reason)

    @staticmethod
    def _fmt_duration(seconds: float) -> str:
        seconds = max(0.0, float(seconds))
        return f"{int(seconds // 60)}分{seconds % 60:.1f}秒"

    # ------------------------------------------------------------------ #
    #  外部命令（/nav/waypoint）：未知命令只告警不崩
    # ------------------------------------------------------------------ #
    def _drain_commands(self) -> None:
        try:
            commands = self.bridge.take_waypoint_commands(8)
        except Exception as exc:                    # noqa: BLE001  读命令失败不该杀任务
            if self._log_throttle.allow("cmd_read_failed", self._clock()):
                self._log(f"读取外部命令失败（忽略本拍）：{type(exc).__name__}: {exc}")
            return
        for raw in commands or []:
            self._cmd_stats["total"] += 1
            try:
                self._handle_command(raw)
            except Exception as exc:                # noqa: BLE001  ★ 一条坏命令不许崩
                self._cmd_stats["rejected"] += 1
                self._log(f"⚠️ 外部命令无效（已忽略）{raw!r}：{type(exc).__name__}: {exc}")

    def _handle_command(self, raw: Any) -> None:
        text = str(raw or "").strip()
        if not text:
            raise ValueError("空命令")
        low = text.lower()
        # ★ 外部命令载入的路线一律标"外部命令"来源（横幅会覆盖"自动规划/手动配置"），
        #   否则横幅会把 /nav/waypoint 下发的路线说成"启动时手动配置的 route.tokens"。
        self._route_source = ROUTE_SOURCE_COMMAND
        self._plan_info = {}

        if low.startswith("reset:"):
            self._reset_belief(text[len("reset:"):].strip())
            self._clear_route("reset 命令")
            self._write_phase(MissionPhase.IDLE)    # 复位后回到 IDLE，等下一单点/多点/自动出发
            return

        if low.startswith("load:"):
            spec = text[len("load:"):].strip()
            if not spec:
                raise ValueError("load: 后面缺少文件名")
            self._load_tokens(spec, source=f"load:{spec}")
            self._start_mission()
            return

        if low.startswith("shoot@"):
            raise ValueError("SHOOT@ 只能作为路线 token 出现，不能单独下发给 /nav/waypoint")

        # `单点 BLOCK,HEAD` → 单点路线（显式前缀，便于日志区分）
        for prefix in ("单点", "single", "goto"):
            if low == prefix or low.startswith(prefix + " ") or low.startswith(prefix + ":"):
                rest = text[len(prefix):].lstrip(" :").strip()
                self._load_single_point(rest)
                self._start_mission()
                return

        # `多点 ...` → 多航点路线（前缀只为可读性，内容与裸 token 列表一致）
        for prefix in ("多点", "multi", "route"):
            if low == prefix or low.startswith(prefix + " ") or low.startswith(prefix + ":"):
                rest = text[len(prefix):].lstrip(" :").strip()
                if not rest:
                    raise ValueError(f"{prefix} 后面缺少航点")
                self._load_tokens(self._normalize_command_tokens(rest), source="多点命令")
                self._start_mission()
                return

        tokens = self._normalize_command_tokens(text)
        if len(tokens) == 1:
            self._load_single_point(tokens[0])      # 裸 `BLOCK,HEAD` 也当单点
            self._start_mission()
            return
        self._load_tokens(tokens, source="waypoint命令")   # 裸多航点
        self._start_mission()

    @staticmethod
    def _normalize_command_tokens(text: str) -> list[str]:
        """`BLOCK,HEAD` / `A B C,y`（空格/分号/竖线分隔）→ token 列表。

        旧版 `waypoint_callback` 兼容 `BLOCK,HEAD`（逗号分隔）与空格分隔的串。
        朝向后缀 `,x|,y` 只允许挂在**最后一个** token 上（`parse_mission` 的硬约束），
        所以这里把误写在中间的裸 `x`/`y` 挪到最后，而不是直接报错。
        """
        body = text.replace(";", " ").replace("|", " ").strip()
        pieces = [p.strip() for p in body.replace(",", " ").split() if p.strip()]
        if not pieces:
            raise ValueError("命令里没有可解析的 token")
        tokens = pieces
        heads = {"x", "y"}
        moved: list[str] = []
        for tok in tokens:
            low = tok.lower()
            if low in heads:
                moved.append(low)
        if moved:
            tokens = [t for t in tokens if t.lower() not in heads]
            if not tokens:
                raise ValueError("只有朝向 token，缺少格名")
            tokens = tokens[:-1] + [tokens[-1] + "," + moved[-1]]
        return tokens

    def _clear_route(self, why: str) -> None:
        self._route = None
        self._path = None
        self._waypoint_count = 0
        self._passed = []
        self._passed_by_pivot = set()
        self._handled_stops = set()
        self._stopctx = None
        self._last_stop_state = StopPointState()
        self._stop_s = {}
        self._stop_blocks = []
        self._dist_to_next_stop = None
        self._finish_latched = False
        self._finish_done = False
        self._finish_info = {}
        self._intel_used = {}                       # 路线清空 → 用格证据也清空
        self._last_s = 0.0
        self._route_s = 0.0
        self._set_fire_gate(False)
        self.bb.set(P_ROUTE, None, source=self.name)
        self.bb.update(self.name, **{P_ROUTE_INDEX: -1, P_ROUTE_S: 0.0,
                                     PIVOT_DONE_KEY: None})
        # ★T27：路线被外部命令/停止请求清空 ⇒ 本段不再"执行中"（StepLoop 据此转 WAIT_INFO）
        with self._segment_lock:
            if self._segment_active:
                self._segment_active = False
                self._segment_error = self._segment_error or f"路线被清空（{why}）"
        self._log(f"清空路线（{why}）")
