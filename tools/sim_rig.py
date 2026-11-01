# -*- coding: utf-8 -*-
"""WB-2 循迹闭环台架入口（卡 `WB-2-RIG-20261031` · tier-1）。

## 一句话

**生产任务线程在环 × 实车量级噪声 × 真实执行器链 × 双臂对照 × 逐拍回放。**

一条命令跑出一个**可信的数**，且这个数**带 transfer 分级**：

```
PYTHONPATH=src/dzbrain python tools/sim_rig.py --scenario field7 --arm realistic --json
```

## 与两台"半个"的分工（这是本卡存在的全部理由）

| 台 | 有 | 缺 | 本台架怎么办 |
|---|---|---|---|
| `tools/sim_cli.py`（tier-0） | 注入 + 执行器链 | **任务状态机**（不 import `tasks/`）；自带平行投影 | import 生产 `tasks/` 本尊；投影 = `Path.project` |
| `main.py --bench --drive` | 真线程 + 状态机 | **无注入**（喂理想位姿）、无执行器链、不可复现（墙钟节拍） | 里程计生成 + 执行器链；虚拟时钟定序 |
| **本台架（tier-1）** | 两者都要 | — | tier-0/tier-1 并存；`sim.py`/`sim_cli.py` **一个字不改** |

## 四类硬判据落在哪

| 判据 | 落点 |
|---|---|
| ①台架内不得有第二份投影/推进 | `test_wb2_no_second_projection.py`（源码级 + 运行期同一性断言） |
| ②realistic 档 `belief_error_rms_m>0` / ideal ≡0 | `test_wb2_rig_invariants.py` §3 + 本文件 `--arm` 两档实跑 |
| ③指标带 transfer 分级；`field_modeled=0` 时越界类 `null` | `wb_rig/metrics.py`（数据层强制，不靠 UI 记得置灰） |
| ④末行契约 `dzrobot-trackbench-result/1`，五键只增不改 | `wb_rig/metrics.py::SCHEMA` / `Metrics.five()` |

## 退出码（与 `sim_cli.py` 同约定）

| 码 | 含义 |
|---|---|
| 0 | 跑完（**不代表"算法好"**，只代表"跑完了且契约行已产出"） |
| 2 | 参数/输入问题（未知场景、坏臂名、坏 `--variant`、夹具缺失） |
| 3 | 跑完了但**结构性判据失败**（ideal 档 `belief_error_rms_m != 0`；或 realistic 档 == 0） |
| 4 | 任务线程 FAULT（生产任务抛异常被抓；台架自己也失真，不许当结果用） |

## 刻意不做

- 不 import `rclpy`（必须在无 ROS 的笔记本上跑）。
- 不写 `config/**` 与 `src/dzbrain/dzbrain/**`（冻结区；本卡零写入）。
- 不 commit、不 push、不碰车/无人机。
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import math
import os
import pathlib
import sys
import traceback

# --------------------------------------------------------------------------- #
#  sys.path：让 `wb_rig`（本目录下的子包）与 `dzbrain`（src/dzbrain 下）都能 import
# --------------------------------------------------------------------------- #
_THIS = pathlib.Path(__file__).resolve()
_TOOLS = _THIS.parent                     # dzrobot_v2/tools
_REPO = _TOOLS.parent                     # dzrobot_v2
_SRC = _REPO / "src" / "dzbrain"          # dzrobot_v2/src/dzbrain
_TESTS = _SRC / "tests"
# ★ 路径顺序是**判据级**的，不是"能 import 就行"：
#   `dzrobot_v2/tools/dzbrain.py` 是个**同名的脚本入口**（`python tools/dzbrain.py --bench`），
#   它会把真正的包 `src/dzbrain/dzbrain/` 整个遮住。表现为
#   `ModuleNotFoundError: No module named 'dzbrain.main'; 'dzbrain' is not a package`
#   —— 一个和本卡毫无关系的报错，会把人带到错方向（我第一次就踩了）。
#   ⇒ 规则：`src/dzbrain` 与 `tests` 必须在**前**，`tools` 必须在**后**。
sys.path[:] = [p for p in sys.path
               if os.path.abspath(p or os.curdir) != str(_TOOLS)]
sys.path.insert(0, str(_TESTS))
sys.path.insert(0, str(_SRC))
sys.path.append(str(_TOOLS))

from dzbrain.core import geometry as geo                                  # noqa: E402
from dzbrain.core.blackboard import (P_MISSION_PHASE, P_POSE,            # noqa: E402
                                     P_ROUTE, P_ROUTE_S, Blackboard)
from dzbrain.core.config import Config                                    # noqa: E402
from dzbrain.core.events import EventBus                                  # noqa: E402
from dzbrain.core.task import TaskGroup                                    # noqa: E402
from dzbrain.route.parser import parse_mission                            # noqa: E402
from dzbrain.tools import sim as SIM                                      # noqa: E402

import wb_rig.arms as ARMS                                                # noqa: E402
import wb_rig.actuator as ACT                                             # noqa: E402
import wb_rig.counters as CNT                                             # noqa: E402
import wb_rig.metrics as MET                                              # noqa: E402
import wb_rig.scenarios as SCN                                            # noqa: E402
import wb_rig.sched as SCH                                                # noqa: E402
from sim_world import Injection, World                                    # noqa: E402

#: 本台架的契约前缀（**区分于**现网 `SIM_RESULT_JSON`：两个 schema 并存，互不覆盖）
RESULT_PREFIX = "TRACKBENCH_RESULT_JSON"

#: 默认任务登记顺序 = 生产 `main.build_tasks()` 的顺序（换了顺序指标会变，见 `wb_rig/sched.py`）
TASK_ORDER = ("localization", "safety", "tracker", "mission")

#: 场景无路线时的退出码（不猜一条路线出来，而是明确"这个场景不能跑闭环"）
RC_NO_ROUTE = 2
#: 结构性判据失败（ideal 档信念误差 != 0，或 realistic 档 == 0）
RC_STRUCTURAL = 3
#: 生产任务 FAULT
RC_FAULT = 4


# --------------------------------------------------------------------------- #
#  配置
# --------------------------------------------------------------------------- #
def load_config(*, scenario: SCN.Scenario, variant: str | None = None,
                auto_start_delay_s: float = 0.2,
                arm: ARMS.ArmProfile | None = None) -> Config:
    """读**现场真源** `config/robot.yaml`（只读，零写入），并压到"台架可跑"的运行时值。

    ★ 只压四类键，每类都写明理由；其余键全部保持现场值（否则夹具会掩盖现场口径）：
      1. `mission.auto_start` / `auto_start_delay_s`：现场等 10~45 s 指令，台架等不起；
      2. `planner.auto_plan_enabled=False` + `route.tokens=<夹具>`：本卡跑的是
         **夹具路线**。布局模式（`map_path=""`）需要无人机下发布局，那是 wb-3/wb-4 的事；
         台架若不压这两键，Mission 会因"无地图"抛 `AutoPlanBlocked` 停在 IDLE；
      3. `route.cross_*_candidates` / `*_default` 与场景地图对齐：无无人机链路时
         Mission 走 `no_link` 分支用默认格，默认格**必须**在候选集合里且**必须**
         是本场景地图的通口，否则 `validate` 判红/换洞失败；
      4. **臂档**的 `fix.anchor_at_waypoint` / `localization.yaw_trim_deg`：见
         `wb_rig/arms.py::ArmProfile` docstring —— 它们和注入量是同一类东西
         （"让信念偏离真值"），所以属于臂档而不属于"现场配置"。
    """
    cfg = Config.load(str(_REPO / "config" / "robot.yaml"))
    tokens = _read_tokens(scenario)
    bridges = _map_bridges(scenario.map_path)
    lower = bridges[0] if bridges else ""
    upper = bridges[1] if len(bridges) > 1 else lower
    cfg.override("mission.auto_start", True)
    cfg.override("mission.auto_start_delay_s", float(auto_start_delay_s))
    cfg.override("mission.scan_mode", "none")        # 云台单写者 = AimTask；台架不测扫描
    cfg.override("planner.auto_plan_enabled", False)
    cfg.override("planner.fallback_to_tokens", True)
    cfg.override("route.tokens", list(tokens))
    if lower:
        cfg.override("route.cross_lower_default", lower)
        cfg.override("route.cross_lower_candidates", [lower])
    if upper:
        cfg.override("route.cross_upper_default", upper)
        cfg.override("route.cross_upper_candidates", [upper])
    cfg.override("logging.rate_limit_s", 0.0)        # 台架要完整日志（不走 1 s 限流）
    if arm is not None:
        cfg.override("fix.anchor_at_waypoint", bool(arm.anchor_on_arrival))
        if arm.yaw_trim_deg is not None:
            cfg.override("localization.yaw_trim_deg", float(arm.yaw_trim_deg))
    if variant:
        _install_variant(cfg, variant)
    return cfg


def _read_tokens(scenario: SCN.Scenario) -> list[str]:
    """读夹具路线（文件模式，走 WB-1 修好的 `_read_token_file`）。"""
    if not scenario.route_path:
        raise SCN.ScenarioError(
            f"场景 {scenario.name!r} 没有路线快照（route=null）⇒ 闭环台架跑不了。"
            f"要么换一个带路线的场景，要么由布局模式（wb-3/wb-4）规划出路线。")
    from dzbrain.route.parser import _read_token_file
    return list(_read_token_file(scenario.route_path))


def _map_bridges(map_path: str) -> list[str]:
    """从场景地图取**通口格**（`map.bridges`）。读的是 `sim.parse_map_yaml` 的同一份解析器。"""
    spec = SIM.load_field_spec(map_path)
    if spec is None:
        return []
    return sorted(spec.bridges)


def _install_variant(cfg: Config, dotted: str) -> None:
    """`--variant <import 路径>`：**不改代码**换控制器实现。

    ★ 为什么用"运行时替换 tracker 模块里的 `compute_command`"而不是改文件：
      卡面要求"不改代码换控制器实现"，而 `TrackerTask` 内部是
      `from ..pursuit.controller import compute_command`（模块级绑定），
      所以把 `dzbrain.tasks.tracker.compute_command` 换掉即可生效；
      文件一个字节不动 ⇒ 换臂对照没有代码差异带来的噪声。
    ★ 变体对象的契约 = 与 `pursuit.controller.compute_command` **同签名同返回**
      （`(pose, path, proj, cfg, **kwargs) -> PursuitCommand`）。签名不符立刻抛，不静默降级。
    """
    try:
        mod_name, _, attr = dotted.rpartition(".")
        if not mod_name:
            raise ValueError(f"--variant 需要 `<模块>.<属性>` 形式，收到 {dotted!r}")
        module = importlib.import_module(mod_name)
        fn = getattr(module, attr)
    except Exception as exc:                            # noqa: BLE001
        raise SCN.ScenarioError(f"--variant 装载失败 {dotted!r}：{type(exc).__name__}: {exc}")
    # ★★ WB-12：必须是**可调用对象**。
    #   卡面示例 `dzbrain.pursuit.controller` 会解析成**模块**（模块不可调用），
    #   旧实现会把模块直接装进 `_tracker.compute_command`，运行到调用处才炸
    #   （或更糟：静默走原来的路）。宁可在装载期就报错。
    if not callable(fn):
        raise SCN.ScenarioError(
            f"--variant 需要**可调用对象**，{attr!r} 是 {type(fn).__name__}"
            f"（不可调用）。如果你想的是模块，请写 `<模块>.<函数>`，"
            f"如 dzbrain.pursuit.controller.compute_command")
    import dzbrain.tasks.tracker as _tracker
    _tracker.compute_command = fn
    cfg.override("_variant.dotted", dotted)             # 只做记录（不进 SCHEMA 校验路径）


# --------------------------------------------------------------------------- #
#  跑一轮
# --------------------------------------------------------------------------- #
def run(*, scenario: SCN.Scenario, arm: ARMS.ArmProfile, seconds: float,
        trace_every: int = 1, hash_only: bool = False, seed: int = 20261031,
        actuator: object | None = None, field_modeled: bool = True,
        variant: str | None = None,
        out_dir: str | None = None, log=None) -> dict:
    """跑一轮闭环。返回结果字典（含指标、逐拍行、契约行、健康台账）。

    ★ **纯函数性**：同一组入参（同一场景/臂/种子/秒数）永远得到**逐位相同**的
      `steps.csv` 与 `metrics.json`。来源只有三处：虚拟时钟、自备 LCG 噪声、固定 dt。
      `trace_every` 只影响**落盘抽稀**，不影响指标（否则 `--trace-every 5` 会改答案）。
    """
    emit = log or (lambda msg: print(msg, flush=True))
    # ---- ① 场景与配置 ----
    tokens = _read_tokens(scenario)
    route = parse_mission(tokens)
    cfg = load_config(scenario=scenario, arm=arm, variant=variant)
    field_spec = SIM.load_field_spec(scenario.map_path) if field_modeled else None

    # ---- ② 世界模型 + 执行器链 ----
    act = actuator
    if act is None:
        act = ACT.Actuator(latency_s=float(cfg.get("sim_rig.latency_s")
                                          or ACT.DEFAULT_LATENCY_S))
    injection = Injection.of(arm, seed=seed)
    start_x, start_y = 0.0, 0.0
    start_yaw = float(route.waypoints[0].yaw)
    # 起拍位姿 = **现场摆位约定**（设计起点格 + 车头方向），与 sim.py 的 `init_yaw="start"`
    # 同源：`mission.reset` 给的格 + 朝向。这里直接复用生产解析器，不自己推。
    reset_x, reset_y, reset_yaw = _resolve_start(cfg, route)
    start_x, start_y, start_yaw = reset_x, reset_y, reset_yaw
    world = World(route=route, injection=injection, actuator=act,
                  start_pose=(start_x, start_y, start_yaw),
                  wheelbase_m=float(cfg.get("pursuit.wheelbase_m") or 0.157))

    # ---- ③ 生产任务（**本尊**，只注入 FakeBridge/黑盒/总线/虚拟时钟）----
    import fakes
    clock = SCH.VirtualClock()
    bridge = fakes.FakeBridge(clock)
    bridge.set_pose(world.truth.x, world.truth.y, world.truth.yaw)
    bb, bus = Blackboard(), EventBus()
    group = TaskGroup(bus, bb, logger=lambda m: None)
    # ★ 时钟在这里传给 `_build_tasks`（**构造期**绑死），不是构造完再改 —— 原因见该函数 docstring
    tasks = _build_tasks(bridge=bridge, bb=bb, bus=bus, cfg=cfg, group=group, clock=clock)
    sched = SCH.Scheduler(clock)
    for name, task in tasks:
        sched.add(task)
    sched.setup_all()
    # ★ 起拍先喂一帧里程计：否则 Localization 首拍 `pose() is None` → 判"定位缺失"，
    #   而 Safety 的启动宽限只护 8 s，随后会误判 pose_lost 并锁存 SAFE_STOP。
    _feed_bridge(bridge, world, clock.now(), refresh=True)

    loc = dict(tasks)["Localization"]
    trk = dict(tasks)["TrackerTask"]
    saf = dict(tasks)["Safety"]
    mis = dict(tasks)["MissionTask"]
    field_obs = SIM.FieldObs(
        spec=field_spec,
        length_m=float(cfg.get("vehicle.length_m") or 0.28),
        width_m=float(cfg.get("vehicle.width_m") or 0.17),
        ref_offset_m=float(cfg.get("vehicle.ref_offset_m") or 0.0),
    ) if field_spec is not None else None

    # ---- ④ 逐拍主循环 ----
    rows: list[dict] = []
    belief_gap: list[float] = []
    xt_truth: list[float] = []
    xt_belief: list[float] = []
    final_truth_proj = None
    last_proj_s = 0.0
    steps = int(round(seconds / SCH.WORLD_DT_S))
    finished_at: float | None = None
    phase = ""
    odom_refresh_every = int(round(SCH.SAMPLE_HZ["localization"] / _ODOM_HZ))
    # ---- 信念↔真值的**同刻配对**：见主循环内注释（这一段只负责缓存）----
    odom_at: tuple[float, float] | None = None
    truth_at_odom: tuple[float, float] | None = None
    # ★★ WB-12（F4）：本次跑动中**是否观察到过 FINISH 相位**。
    #   为什么不能拿循环退出时的 `phase` 算 `mission_completed`：
    #   FINISH 实测只持续 **100 拍**（Mission 写完结算后自行回落 IDLE = 它的正常终态），
    #   而 `run()` 要在 FINISH 首次出现后再等 **0.5 s（250 拍）**才 `break`
    #   ⇒ 退出时 `phase` 已变回 IDLE ⇒ 旧写法恒为 0.0，且与 `finished=1.0` 撕裂。
    #   新写法与 `finished_at` **同源**（同一个 if 分支置位），不另立判据。
    saw_finish: bool = False
    for k in range(steps):
        now = clock.now()
        # ---- 里程计 100 Hz ----
        # ★★ 顺序是判据级的：**先喂帧、再跑任务**。
        #   倒过来（先 `run_step` 后喂帧）会让 Localization 读到**上一次迭代**写进桥的帧
        #   —— 于是 `P_POSE` 与"本拍真值"差一个 2 ms 步长，ideal 档的
        #   `belief_error_rms_m` 就停不到 0（实测 3.83e-4 = 0.284 m/s × 2 ms 的量级）。
        #   这就是两把尺子采样时刻差一拍，不是建模误差。
        if k % max(1, odom_refresh_every) == 0:
            _feed_bridge(bridge, world, now, refresh=True)
            odom_at = (world.last_odom().x, world.last_odom().y) if world.last_odom() else None
            truth_at_odom = (world.truth.x, world.truth.y)
        ran = sched.run_step()
        # ---- 控制器上一条已下发指令 → 执行器链 ----
        cmd = _last_cmd(bridge)
        act.command(now - SCH.WORLD_DT_S, cmd[0], cmd[1])
        # ★ 记下**本步推进之前**的真值：`P_POSE` 是 Localization 在同一拍内读里程计的那一刻
        #   写下的，里程计也在那一刻采样。本步末尾的 `world.step` 是下一个 2 ms ——
        #   拿推进后的真值比会稳定多出"一个 dt 的位移"（0.35 m/s × 2 ms ≈ 0.7 mm，
        #   实测 ideal 档因此停在 0.000766）。那不是建模误差，是**两把尺子差一拍**。
        truth_before = (world.truth.x, world.truth.y)
        world.step(SCH.WORLD_DT_S, cmd[0], cmd[1], now)
        # ---- 观测（真值投影 + 信念误差）----
        # ★★ 信念↔真值必须是**同拍配对**：`P_POSE` 由 Localization 在它自己那一拍写，
        #    而本循环每 2 ms 推一次真值。若拿"20 Hz 写下的信念"去比"2 ms 后的真值"，
        #    0.35 m/s × 0.05 s = 1.75 cm 的**纯时间错位**就会进 `belief_error_rms_m` ——
        #    那不是模型误差，是把两把不同步的尺子对着量（实测：ideal 档因此从 0 变成 0.006）。
        #    所以只在 Localization **刚跑过**的那一步配对比较（`sched.last_ran` 是事实，
        #    不是"按 k 取模"的猜测 —— 调度器落后重置时基时取模会取错）。
        loc_ran = "Localization" in sched.last_ran
        proj_t = world.project_truth()
        last_proj_s = float(proj_t.s)
        xt_truth.append(float(proj_t.dist))
        belief = bb.get(P_POSE)
        if isinstance(belief, geo.Pose) and loc_ran and truth_at_odom is not None:
            # ★★ 信念↔真值必须**同刻配对**：`P_POSE` = Localization 读里程计那一刻的观测
            #    （ideal 档下 = 那一刻的真值），所以基准必须是**采样帧被写进桥时的真值**
            #    `truth_at_odom`。若改用本步末尾推进后的真值，会稳定多出"一个 dt 的位移"
            #    （实测 0.284 m/s × 2 ms ≈ 0.57 mm）—— 那不是建模误差，是两把尺子的
            #    采样时刻差了一拍，而 `belief_error_rms_m` 会被它污染成非零。
            belief_gap.append(math.hypot(belief.x - truth_at_odom[0],
                                         belief.y - truth_at_odom[1]))
            xt_belief.append(float(world.project_belief(belief, s_hint=last_proj_s).dist))
        if field_obs is not None:
            field_obs.observe(now, world.truth.x, world.truth.y, world.truth.yaw,
                              kind=("corner" if proj_t.dist > 0.05 else "normal"))
        phase = str(bb.get(P_MISSION_PHASE) or "")
        # ★★ WB-17（I15）：**逐拍**取 tracker 的 PIVOT 相位。
        #   `trk_tel` 只是**末拍**快照；末拍时 PIVOT 早已结束（回到 IDLE）
        #   ⇒ 只靠它判不了"I15 那段是不是在 PIVOT 里"。
        #   生产 `tracker.py:703` 每拍都写 `"pivot_phase"`（IDLE/SETTLE/TURN），这里只读不改。
        try:
            _tel = trk.telemetry() if callable(getattr(trk, "telemetry", None)) else {}
            pivot_phase = str(_tel.get("pivot_phase") or "")
        except Exception:                             # noqa: BLE001
            pivot_phase = ""
        if k % max(1, trace_every) == 0:
            rows.append(_step_row(k, now, world, bridge, bb, proj_t, belief, phase, act,
                                  truth_at_odom=truth_at_odom, loc_ran=loc_ran,
                                  pivot_phase=pivot_phase))
        if phase == "FINISH" and finished_at is None:
            finished_at = now
            saw_finish = True
            # 到终点后给 0.5 s 收尾（让 Mission 写完结算、Tracker 刹完车），再停
            if now - clock.now() > -1.0:
                pass
        if finished_at is not None and (now - finished_at) >= 0.5:
            break

    teardown_notes = sched.teardown_all()
    faulty = group.faulty()

    # ---- ⑤ 指标 ----
    book = CNT.collect(loc=loc, trk=trk, saf=saf, mis=mis, group=group)
    m = _build_metrics(arm=arm, world=world, rows=rows, cfg=cfg,
                       belief_gap=belief_gap, xt_truth=xt_truth, xt_belief=xt_belief,
                       field_obs=field_obs, field_modeled=(1 if field_obs is not None else 0),
                       route=route, phase=phase, finished=finished_at is not None,
                       book=book, act=act, clock=clock, saw_finish=saw_finish)
    payload_meta = {
        "scenario": scenario.name,
        "map": os.path.basename(scenario.map_path),
        "route": os.path.basename(scenario.route_path or ""),
        "arm": arm.name,
        "arm_note": arm.note,
        "provisional": list(ARMS.PROVISIONAL),
        "seed": int(seed),
        "world_dt_s": SCH.WORLD_DT_S,
        "sample_hz": dict(SCH.SAMPLE_HZ),
        "rates_hz": {s.name: s.rate_hz for s in sched.slots},
        "ticks": {s.name: s.ticks for s in sched.slots},
        "task_health": book.task_health,
        "teardown": teardown_notes,
        "faulty": faulty,
        "actuator": ACT.describe(),
        "scenario_origin": scenario.origin,
        "route_total_m": float(world.path.total_length),
    }
    result = {
        "metrics": m,
        "rows": rows,
        "book": book,
        "meta": payload_meta,
        "faulty": faulty,
        "phase": phase,
        "shell_path": os.path.basename(__file__),
    }
    if not hash_only:
        result["line"] = m.line(meta=payload_meta)
    return result


def _resolve_start(cfg: Config, route) -> tuple[float, float, float]:
    """起拍位姿：**复用生产的 `mission.reset` 解析器**（不在台架里另写一套）。"""
    spec = str(cfg.get("mission.reset") or "")
    try:
        from dzbrain.route.parser import parse_reset
        block, yaw = parse_reset(spec)
        x, y = geo.block_to_meters(block)
        return float(x), float(y), float(yaw)
    except Exception:                                   # noqa: BLE001  坏 reset 就退回路线首点
        wp = route.waypoints[0]
        return float(wp.x), float(wp.y), float(wp.yaw)


_ODOM_HZ = SCH.SAMPLE_HZ["odom"]


def _build_tasks(*, bridge, bb, bus, cfg: Config, group: TaskGroup,
                 clock) -> list[tuple[str, object]]:
    """按**生产顺序**构造四个任务（import 生产类，不用 `main.build_tasks` 的 importlib 路径）。

    ★ 为什么不用 `main.build_tasks`：它会连 Aim/Fix/Drone/Telemetry 一起起（4 个与本卡
      判据无关的线程），其中 DroneTask 会在无链路时反复打日志、TelemetryTask 会写文件。
      本卡只要"循迹闭环"这四环，多起的线程只会让逐位复现更难。
      ⚠️ 这是**范围收窄**，不是第二份装配逻辑：四类的构造签名与 `build_tasks` 完全一致，
         `SafetyTask` 同样需要 `group=`（它靠 group 看别的心跳）。

    ★★ 时钟必须在**构造时**就注入（不是构造完再改绑）。为什么（我实测踩过）：
       `SafetyTask.__init__` 里有 `self._started_at = self.clock()`，而启动宽限
       `startup_grace_s=8.0` 按 `now - _started_at` 算。占位时钟返回 0.0 ⇒ `_started_at=0`，
       而虚拟时钟从 1000 s 起 ⇒ 宽限**一上来就被算成已耗尽** ⇒ 第一拍就判
       `task_stalled:TrackerTask 心跳 infs` → 锁存 SAFE_STOP → **车一动不动、指标全 0**
       （最像"通过"的那种失败）。所以 `clock` 必须由调用方传进来并在构造期绑死。
    """
    from dzbrain.tasks.localization import LocalizationTask
    from dzbrain.tasks.mission import MissionTask
    from dzbrain.tasks.safety import SafetyTask
    from dzbrain.tasks.tracker import TrackerTask

    common = {"log": lambda m: None, "clock": clock.now}
    loc = LocalizationTask(bridge, bb, bus, cfg, **common)
    saf = SafetyTask(bridge, bb, bus, cfg, group=group, **common)
    trk = TrackerTask(bridge, bb, bus, cfg, **common)
    mis = MissionTask(bridge, bb, bus, cfg, **common)
    out: list[tuple[str, object]] = []
    for t in (loc, saf, trk, mis):
        _bind_clock(t, clock.now)
        group.add(t)
        out.append((t.name, t))
    return out


def _bind_clock(task: object, fn) -> None:
    """把任务的内部时钟换成虚拟时钟（生产类**本来就**支持 `clock=` 注入）。

    `LocalizationTask` / `SafetyTask` 用属性名 `clock`，`TrackerTask` / `MissionTask`
    用 `_clock` —— 两种都写（生产代码里这确实是两套），所以这里探测而不是写死一个。
    """
    hit = False
    for attr in ("clock", "_clock"):
        if hasattr(task, attr):
            setattr(task, attr, fn)
            hit = True
    if not hit:
        raise AttributeError(
            f"{type(task).__name__} 不认注入时钟（既没有 `clock` 也没有 `_clock`）"
            f"⇒ 台架会跑在墙钟上、失去逐位复现；必须回来看生产类改了什么")


def _feed_bridge(bridge, world: World, now: float, *, refresh: bool) -> None:
    """把一帧里程计写进桥（真机 = DDS 回调；这里 = `FakeBridge.set_pose`）。"""
    sample = world.odom_frame(now, rng_refresh=refresh)
    bridge.set_pose(sample.x, sample.y, sample.yaw)


def _last_cmd(bridge) -> tuple[float, float]:
    """桥上下发的**最后一条**指令（= Tracker 已 publish 的那条）。"""
    cmds = getattr(bridge, "commands", None) or []
    return (float(cmds[-1][0]), float(cmds[-1][1])) if cmds else (0.0, 0.0)


def _step_row(k: int, now: float, world: World, bridge, bb, proj_t, belief, phase: str,
              act, truth_at_odom: tuple[float, float] | None = None,
              loc_ran: bool = False, pivot_phase: str = "") -> dict:
    """逐拍一行的原始样本（CSV 列在 `_write_steps_csv` 里定型）。

    卡面要求的列：t / odom_raw / belief_pose / cmd_v / cmd_steer / s / seg / 计数器 / 事件。
    """
    odom = world.last_odom()
    st = act.state
    return {
        "k": int(k),
        "t": float(now),
        "truth_x": float(world.truth.x), "truth_y": float(world.truth.y),
        "truth_yaw": float(world.truth.yaw),
        # ★★ WB-12（F5）：契约行 `belief_error_rms_m` 的**真实基准**。
        #   为什么要单独出两列：上面的 `truth_x/truth_y` 是**本步推进后**的真值，
        #   而 `belief_error_rms_m` 的基准是 `truth_at_odom`（里程计帧写进桥那一刻的真值）。
        #   两个基准差**一拍**（实测 0.284 m/s × 2 ms ≈ 0.57 mm/拍，累积成 6 mm 量级）。
        #   旧 CSV 只有推进后的真值 ⇒ 拿它复算契约行**必得 6.12 mm**（而不是 0.0），
        #   看上去像"契约行算错了"。加这两列后 CSV 自己就能证明两把尺子的差异。
        #   ⚠️ 契约行**不动**：ideal 档"belief_error_rms_m 必须 ≡0"是结构性判据，
        #   把它改成 6 mm 会毁掉整条注入链的可信度。
        "truth_at_odom_x": (float(truth_at_odom[0]) if truth_at_odom is not None
                            else float("nan")),
        "truth_at_odom_y": (float(truth_at_odom[1]) if truth_at_odom is not None
                            else float("nan")),
        # ★★ WB-12（F5）：本拍 Localization 是否刚跑过。
        #   契约行 `belief_error_rms_m` **只在定位拍**计入（`belief_gap` 仅 `loc_ran` 时 append），
        #   所以拿 CSV 复算时必须先筛出定位拍。旧 CSV 只能靠
        #   "看 belief_x 变没变"猜（启发式）；这一列用**事实**取代猜测。
        #   实测验证：只取 loc_ran=1 的行 + truth_at_odom 复算 ⇒ RMS 精确 0.0（与契约行完全一致）。
        "loc_ran": 1 if loc_ran else 0,
        # ★ WB-17（I15）：tracker 的 PIVOT 相位（IDLE/SETTLE/TURN）。
        #   有了它，"I15 这段是否在 PIVOT 里被限速绕过"才能机器判。
        "pivot_phase": str(pivot_phase),
        "odom_x": float(odom.x) if odom else float("nan"),
        "odom_y": float(odom.y) if odom else float("nan"),
        "odom_yaw": float(odom.yaw) if odom else float("nan"),
        "belief_x": float(belief.x) if isinstance(belief, geo.Pose) else float("nan"),
        "belief_y": float(belief.y) if isinstance(belief, geo.Pose) else float("nan"),
        "belief_yaw": float(belief.yaw) if isinstance(belief, geo.Pose) else float("nan"),
        "cmd_v": float(st.cmd_v_in), "cmd_steer": float(st.cmd_steer_in),
        "exec_v": float(world.truth.v_exec), "exec_steer": float(world.truth.steer_exec),
        "speed_counts": int(st.speed_counts),
        "s": float(proj_t.s), "seg": int(proj_t.seg_index), "xt": float(proj_t.dist),
        "phase": str(phase),
        "watchdog": int(st.watchdog_tripped), "dead_zone": int(st.dead_zone),
    }


def _build_metrics(*, arm, world, rows, cfg, belief_gap, xt_truth, xt_belief,
                   field_obs, field_modeled, route, phase, finished, book, act,
                   clock, saw_finish: bool = False) -> MET.Metrics:
    """把采样变成指标表（**每条指标在这里一次算完并带 note**）。"""
    m = MET.Metrics(arm=arm.name, field_modeled=int(field_modeled))
    n = len(rows)
    m.put("steps", float(world.steps))
    m.put("time_s", float(clock.elapsed))
    m.put("finished", float(1.0 if finished else 0.0))
    # ★★ WB-12（F4）：`mission_completed` 与 `phase_final` **两个不同问题**，不再理解成
    #   "退出时刻的 phase 是不是 FINISH"（那使它恒为 0，见 run() 里 saw_finish 的注）。
    #   · `phase_final` = 退出时刻的**真实终态**（保留原语义，它确实是 IDLE）；
    #   · `mission_completed` = 本次跑动**是否到过 FINISH**（与 `finished` 同源）。
    #   两者吔不撕裂：到过 FINISH ⇒ 两个都为 1.0；没到 ⇒ 两个都为 0.0。
    m.put("phase_final", phase,
          note="退出时刻的读数（真实终态；到过 FINISH 后 Mission 会自行回落 IDLE）")
    m.put("mission_completed", float(1.0 if saw_finish else 0.0),
          note="本次跑动是否到过 FINISH 相位（与 finished 同源；与 phase_final 的终态读数不同问题）")
    m.put("waypoints_total", float(len(route)))
    m.put("seg_final", float(rows[-1]["seg"] if rows else 0))
    m.put("s_final_m", float(rows[-1]["s"] if rows else 0.0))
    # ---- ★★ WB-17：`route_coverage` 与 `finished` **并列**（不替换）----
    #   为什么必需：实测 `--arm ideal` 跑 field7 得 `finished=1.0`，但 CSV 里
    #   **21/67 段一拍都没走过** ⇒ `finished` 可以被跳段骗过。
    #   分母口径（卡面钉死）= `waypoints_total - 1`（路点数 − 1 = 段数，段号 0..N-2）。
    #   ⚠️ 不能用 `waypoints_total` 做分母（那是路点数不是段数），否则
    #     "跳过终点"时会算出假的 100%——那正是本卡要抓的形态。
    _segs_seen = sorted({int(r["seg"]) for r in rows}) if rows else []
    _n_seg_total = max(1, len(route) - 1)          # 路点数 − 1 = 段数
    m.put("seg_visited_count", float(len(_segs_seen))
          , note="实际走过的段数（从 steps.csv 的 seg 列取并集）")
    m.put("seg_expected_count", float(_n_seg_total)
          , note="总段数 = waypoints_total - 1（段号 0..N-2）")
    m.put("route_coverage", float(len(_segs_seen)) / float(_n_seg_total)
          , note="走过的段 / 总段（与 finished 并列；finished=1 而本值<1 ⇒ 跳段式假成功）")
    # ★ 终点可达性必须**单列**：只看 coverage 会把"跳过终点但走了其他全部"
    #   也算成 99%，看不出来。末段号 = N-2。
    _end_seg = _n_seg_total - 1
    m.put("end_seg_reached", 1.0 if _end_seg in _segs_seen else 0.0
          , note="末段（seg=%d）是否真到过（单列判据，不靠分母）" % _end_seg)
    # ★ I06 跳段观测量：同一趟里 `s` 跳变幅度 >1 段的次数（"投影重定位"直接观测量）。
    #   场景 #5 与 I06 靠它判"车是不是被坐标系跳变带走的"。
    _seg_jumps = 0
    for _i in range(1, len(rows)):
        if abs(int(rows[_i]["seg"]) - int(rows[_i - 1]["seg"])) > 1:
            _seg_jumps += 1
    m.put("seg_jump_count", float(_seg_jumps)
          , note="投影跳段次数（相邻两拍 seg 差>1）⇒ I06 跳段備得观测量的直接抓手")
    # ---- 横向误差（真值 vs 信念，**两个键不许混**：06 册 §2 F-5 事故）----
    m.put("cross_track_rms_m", _rms(xt_truth),
          note="真值横向 RMS（车离折线多远）")
    m.put("cross_track_max_m", (max(xt_truth) if xt_truth else 0.0),
          note="真值横向 MAX")
    m.put("belief_error_rms_m", _rms(belief_gap),
          note="信念↔真值 RMS（**不是**横向误差；两档分级不同，见 metrics.grade）")
    m.put("belief_error_max_m", (max(belief_gap) if belief_gap else 0.0),
          note="信念↔真值 MAX")
    m.put("final_error_m", float(rows[-1]["xt"] if rows else 0.0),
          note="末拍真值横向误差")
    # ★ 现网五键里的 `rms_m` 是**横向 RMS 的别名**（`dzrobot-sim-result/1` 的
    #   `metrics["rms_m"]` 就落在 `cross_track_rms_m` 上，见 sim.py 的 METRIC_KEYS）。
    #   五键形状冻结 ⇒ 本台架必须**同名同义**给出，否则 UI 拿到 `null`。
    m.put("rms_m", m.values.get("cross_track_rms_m"),
          note="= cross_track_rms_m（现网五键别名，形状冻结）")
    length_ratio = (float(rows[-1]["s"]) / float(world.path.total_length)
                    if rows and world.path.total_length > 0 else 0.0)
    m.put("length_ratio", length_ratio, note="已走弧长 / 路线总长")
    # ---- 执行器链的形状判据 ----
    m.put("actuator_dead_zone_steps", float(act.dead_zone_steps),
          note="|cmd|>0 但量化成 0 计数 ⇒ 发过指令车不动（H8 类）")
    m.put("actuator_watchdog_steps", float(act.watchdog_steps),
          note="无指令 ≥1.0 s ⇒ 速度被底盘强制归零")
    m.put("steer_exec_max_deg", max((abs(r["exec_steer"]) for r in rows), default=0.0),
          note="实际执行舵角峰值（过 servo 夹取后）")
    m.put("steer_cmd_max_deg", max((abs(r["cmd_steer"]) for r in rows), default=0.0),
          note="控制器下发舵角峰值（未执行）")
    m.put("speed_quant_count_max", max((r["speed_counts"] for r in rows), default=0),
          note="速度量化计数峰值（1 计数 ≈ 0.014348 m/s）")
    m.put("negative_speed_eaten_steps",
          float(sum(1 for r in rows if r["cmd_v"] < -1e-9 and r["exec_v"] >= -1e-9)),
          note="下发了负速度但实际执行不为负的拍数（'负速度被吃掉'的直接观测量）")
    # ---- 场地 ----
    if field_obs is not None:
        fm = field_obs.metrics()
        for k, v in fm.items():
            m.put(k, v, note="场地物理观测（真值口径）")
    # ⚠️ 2026-11-01 WB-11：这里**不再**拿 `BOUNDS_KEYS` 去压 null。
    #   理由有两条，都是实测结论：
    #   ①「地图边界是否可知」与「场地要素是否建模」是**两件独立的事**。
    #     边界来自 `core/geometry.py` 的模块常量
    #     `MAP_X_MIN/MAX`、`MAP_Y_MIN/MAX`（:32-35），不依赖任何场地要素文件；
    #     实测 `geo.in_bounds(0,0,0.6)=True` 在未加载地图时依然成立 ⇒ **边界永远可知**。
    #     所以 `out_of_bounds` / `out_of_bounds_steps` / `bounds_margin_m` 应**两种模式都保真值**。
    #   ② `field_stopped` / `field_stop_t` 本来就不属越界类（已移出 `BOUNDS_KEYS`），
    #     在下面按 tier-0 口径无条件写。
    #   原写法是“先压 null 再被后面的 `m.values[...] = oob` 覆盖”：
    #   它的终值恰好也对，但一旦将来有人删掉那段覆盖，越界键就静默变 null
    #   而没人知道 ⇒ 把意图与终值一致化（不压）。
    pass
    # ---- 场地建模标志（**两种模式都无条件写**，对齐 tier-0 `sim.py:1593-1594`）----
    #   ⚠️ 同类第三例（与 `field_stopped` 同一根因）：原写法只有
    #   `field_obs.metrics()` 会带出 `field_modeled=1.0`，不建场地时没人写
    #   ⇒ `metricsFull` 里该键**消失**（而 `payload()` 顶层又有一份）。
    #   tier-0 的 `metrics` 字典里 `field_modeled` 两种模式都在（0.0 / 1.0）。
    #   写与不写影响很实际：本卡 verify 命令用 `m.get('field_modeled')` 读它，
    #   缺失和 `0` 在那里长得一样（都是 None）⇒ 存在性必须确定。
    m.put("field_modeled", float(int(field_modeled)),
          note="场地要素是否建模（0/1；两种模式都写，口径同 tier-0）")
    # ---- 场地停车（**两种模式都无条件写**，对齐 tier-0 `sim.py:1595-1596`）----
    #   口径逐字对齐（**不另立一套**）：未触发 ⇒ `0.0` / `-1.0`，
    #   `-1` 是**有效值不是 null**（《06 册》§2 明写）。
    #   `field_modeled=0` 时也照写 0.0：它和 `out_of_bounds_steps` 是同一类
    #   （“两种模式都在”），不随场地要素建模变化。
    #   本台架未实现 `stop_on_field_violation`（打开即会改真值轨迹，需另立卡），
    #   故 `field_stop_t` 恒为 -1.0；写死在这里而不是“不写”，正是为了让
    #   “能力未实现”与“能力开着但未触发”在数据上分得开。
    field_stop_t = None          # 本台架尚未产生该事件
    m.put("field_stopped", 1.0 if field_stop_t is not None else 0.0,
          note="场地停车是否触发（口径同 tier-0 sim.py；两种模式都写）")
    m.put("field_stop_t", float(field_stop_t) if field_stop_t is not None else -1.0,
          note="触发时刻（-1 = 未触发，**有效值不是 null**）")
    # ---- 越界（五键 `out_of_bounds`）----
    # ★★ 与"场地建模"是**两件事**（这里必须说清楚，否则两者会被混成一个键）：
    #   · `out_of_bounds` = 车跑出**地图边界**没有（`geo.in_bounds` + `map_safety_margin_m`，
    #     与 Safety 第②道检查同源同参）。它**不需要**场地要素（雷/障/靶）建模；
    #   · `field_modeled` = 雷/障/靶/桥洞这些**要素**建模了没有（决定压雷/撞柱类指标）。
    #   所以 `out_of_bounds` 不该被 `--no-field` 抹成 null —— 它有独立的真值来源
    #   （真值轨迹 + 地图边界常数）。被抹掉会让五键里最常用的那一键永远是 null。
    margin = float(cfg.get("pursuit.map_safety_margin_m") or 0.0)
    oob = 0
    for r in rows:
        if not geo.in_bounds(r["truth_x"], r["truth_y"], margin):
            oob += 1
    m.values["out_of_bounds"] = int(oob)
    m.notes["out_of_bounds"] = (f"真值轨迹越地图边界拍数（余量 ±{margin:.2f} m，与 Safety 同源）")
    m.values["out_of_bounds_steps"] = int(oob)
    m.notes["out_of_bounds_steps"] = "= out_of_bounds（同一口径，键名兼容 sim.py）"
    m.values["bounds_margin_m"] = margin
    # ---- 注入量（"不可迁移"的定义本身）----
    m.put("inject_speed_scale_err", float(world.injection.speed_scale_err))
    m.put("inject_yaw_drift_dps", float(world.injection.yaw_drift_dps))
    m.put("inject_pos_noise_m", float(world.injection.pos_noise_m))
    # ---- 生产计数器（可迁移）----
    for k, v in book.as_metrics().items():
        m.put_ungraded(k, v, note=book.notes.get(k, ("", ""))[0])
    return m


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


# --------------------------------------------------------------------------- #
#  落盘（steps.csv / metrics.json）
# --------------------------------------------------------------------------- #
STEP_COLUMNS = ("k", "t", "truth_x", "truth_y", "truth_yaw",
                "truth_at_odom_x", "truth_at_odom_y",   # ★ WB-12 F5：契约行的真实基准
                "loc_ran",                              # ★ WB-12 F5：本拍是否定位拍（复算时必须筛）
                "pivot_phase",                         # ★ WB-17 I15：PIVOT 相位（IDLE/SETTLE/TURN）
                "odom_x", "odom_y", "odom_yaw",
                "belief_x", "belief_y", "belief_yaw", "cmd_v", "cmd_steer", "exec_v",
                "exec_steer", "speed_counts", "s", "seg", "xt", "phase", "watchdog",
                "dead_zone")


def write_steps_csv(rows: list[dict], path: str) -> str:
    """逐拍 CSV。**固定列序 + 固定格式**（`%.10g`）—— 逐位复现靠这两条。"""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    lines = [",".join(STEP_COLUMNS)]
    for r in rows:
        cells = []
        for col in STEP_COLUMNS:
            val = r.get(col)
            if isinstance(val, float):
                cells.append("%.10g" % val)
            elif isinstance(val, int):
                cells.append(str(val))
            else:
                cells.append(str(val))
        lines.append(",".join(cells))
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(lines) + "\n")
    return path


def write_metrics_json(metrics: MET.Metrics, meta: dict, path: str) -> str:
    """指标 JSON（`sort_keys=True` + `indent=2` + LF；同输入 == 同字节）。"""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    blob = json.dumps(metrics.payload(meta=meta), ensure_ascii=False,
                      sort_keys=True, indent=2, allow_nan=False)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(blob + "\n")
    return path


def sha256_of(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


# --------------------------------------------------------------------------- #
#  CLI
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="sim_rig",
        description="WB-2 循迹闭环台架：生产任务线程在环 × 实车量级噪声 × 真实执行器链",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--scenario", default="field7",
                    help=f"具名场景（读 fixtures/route_sources.json）：{SCN.available()}")
    ap.add_argument("--arm", default="realistic", choices=sorted(ARMS.ARMS),
                    help="注入档：ideal（零注入，信念≡真值）/ realistic（实车量级）")
    ap.add_argument("--seconds", type=float, default=120.0, help="最长仿真时长（s，虚拟）")
    ap.add_argument("--seed", type=int, default=20261031, help="噪声种子（同种子逐位复现）")
    ap.add_argument("--variant", default=None,
                    help="换控制器实现（`<模块>.<属性>`，如 dzbrain.pursuit.controller")
    ap.add_argument("--repeat", type=int, default=1,
                    help="重复跑 N 次并比对 sha256（>1 时自动开 --hash-only）")
    ap.add_argument("--hash-only", action="store_true",
                    help="只打两行摘要 + sha256（不打整张指标表）")
    ap.add_argument("--out-dir", default=None,
                    help="落盘目录（默认 artifacts/sim_rig/<scenario>_<arm>）")
    ap.add_argument("--trace-every", type=int, default=1,
                    help="steps.csv 每 N 拍记一行（1 = 全记；**不影响指标**）")
    ap.add_argument("--no-field", action="store_true",
                    help="不建模场地（此时越界类指标一律 null，不许 0）")
    ap.add_argument("--json", dest="json_line", action="store_true",
                    help=f"stdout 末行追加 `{RESULT_PREFIX} {{...}}`（机器可读契约行）")
    ap.add_argument("--quiet", action="store_true", help="不打横幅与指标表")
    ap.add_argument("--compare-arms", action="store_true",
                    help="双臂 A/B：同参数同场地同种子跑 ideal+realistic，"
                         "输出两份指标 + 一份逐键 diff（WB-12 F3）")
    ap.add_argument("--list", action="store_true", help="列出已登记场景后退出")
    return ap

def _run_compare_arms(args, emit) -> int:
    """双臂 A/B：同参数/同场地/同种子跑两臂，出两份指标 + 一份逐键 diff。

    ★ 为什么必须"同种子"：两臂的**唯一差别必须是注入档**（`wb_rig/arms.py` 的契约）。
      种子一变，噪声序列就变，diff 里就分不清哪些差异是"注入的代价"、
      哪些是"噪声偶然不同"。
    """
    try:
        scenario = SCN.resolve(args.scenario)
    except (SCN.ScenarioError, KeyError) as exc:
        print(f"❌ 输入失败：{exc}", file=sys.stderr)
        return RC_NO_ROUTE
    names = ("ideal", "realistic")
    emit(f"== WB-12 双臂 A/B · {scenario.name} · 同种子 {args.seed} · "
         f"{args.seconds:g} s ==")
    out_dir = args.out_dir or str(_REPO / "artifacts" / "sim_rig"
                                  / f"{scenario.name}_AB")
    results = {}
    for nm in names:
        try:
            arm = ARMS.get(nm)
            results[nm] = run(scenario=scenario, arm=arm, seconds=args.seconds,
                              trace_every=max(1, int(args.trace_every)),
                              hash_only=True, seed=args.seed,
                              field_modeled=not args.no_field,
                              variant=args.variant, log=emit)
        except (SCN.ScenarioError, KeyError) as exc:
            print(f"❌ {nm} 臂输入失败：{exc}", file=sys.stderr)
            return RC_NO_ROUTE
        except Exception as exc:                        # noqa: BLE001
            print(f"❌ {nm} 臂台架异常：{type(exc).__name__}: {exc}", file=sys.stderr)
            traceback.print_exc(limit=3)
            return RC_FAULT
    a_met = results["ideal"]["metrics"]
    b_met = results["realistic"]["metrics"]
    diff = MET.diff_metrics(a_met, b_met)
    os.makedirs(os.path.abspath(out_dir), exist_ok=True)
    for nm in names:
        write_metrics_json(results[nm]["metrics"], results[nm]["meta"],
                           os.path.join(out_dir, f"run_{nm}_metrics.json"))
    diff_path = os.path.join(out_dir, "arm_diff.json")
    with open(diff_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(diff, ensure_ascii=False, sort_keys=True,
                            indent=2, allow_nan=False) + "\n")
    if not args.quiet:
        print("")
        print(f"双臂 逐键 diff（{diff['n_keys']} 键，相同 {diff['n_same']} / "
              f"不同 {diff['n_diff']}）：")
        print(f"  {'key':28s} {'ideal':>14s} {'realistic':>14s} {'delta':>12s}  grade")
        for r in diff["rows"]:
            if r["same"]:
                continue
            delta_txt = "" if r["delta"] is None else "%.6g" % r["delta"]
            print(f"  {r['key']:28s} {str(r['a']):>14s} {str(r['b']):>14s} "
                  f"{delta_txt:>12s}  {r['grade']}")
        print("")
        print(f"  指标：{out_dir}/run_ideal_metrics.json · "
              f"{out_dir}/run_realistic_metrics.json")
        print(f"  diff：{diff_path}")
    if args.json_line:
        body = {"schema": MET.SCHEMA, "ok": True, "mode": "compare-arms",
                "scenario": scenario.name, "diff": diff}
        print(RESULT_PREFIX + " " + json.dumps(body, ensure_ascii=False,
                                               sort_keys=True, allow_nan=False),
              flush=True)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.list:
        for name in SCN.available():
            sc = SCN.resolve(name)
            print(f"  {name:10s} {sc.describe()}")
        return 0
    emit = (lambda msg: None) if args.quiet else (lambda msg: print(msg, flush=True))
    # ---- ★ WB-12 F3：双臂 A/B（开关开了就只跑这一条路，不与单臂流程混走）----
    if args.compare_arms:
        return _run_compare_arms(args, emit)
    # ---- 输入校验（失败也有机器可读结果；退出码 2）----
    try:
        scenario = SCN.resolve(args.scenario)
        arm = ARMS.get(args.arm)
        if not args.json_line and args.repeat > 1:
            args.json_line = True
    except (SCN.ScenarioError, KeyError) as exc:
        print(f"❌ 输入失败：{exc}", file=sys.stderr)
        return RC_NO_ROUTE

    out_dir = args.out_dir or str(_REPO / "artifacts" / "sim_rig"
                                  / f"{scenario.name}_{arm.name}")
    emit(f"== WB-2 循迹台架 · {scenario.name} / {arm.name} ==")
    emit(scenario.describe())
    emit(f"注入：{arm.note}")
    emit(f"采样档：{SCH.SAMPLE_HZ}  · 世界 dt {SCH.WORLD_DT_S * 1000:.1f} ms")
    emit(ACT.describe())
    emit(f"输出：{out_dir}")

    runs = max(1, int(args.repeat))
    hashes: list[dict] = []
    result = None
    try:
        for i in range(runs):
            result = run(scenario=scenario, arm=arm, seconds=args.seconds,
                         trace_every=max(1, int(args.trace_every)),
                         hash_only=args.hash_only or runs > 1, seed=args.seed,
                         field_modeled=not args.no_field,
                         variant=args.variant, log=emit)
            if runs > 1:
                emit(f"  第 {i + 1}/{runs} 次跑完：{len(result['rows'])} 行 / "
                     f"phase={result['phase']}")
    except (SCN.ScenarioError, KeyError) as exc:
        print(f"❌ 输入失败：{exc}", file=sys.stderr)
        return RC_NO_ROUTE
    except Exception as exc:                            # noqa: BLE001  台架自身失真
        print(f"❌ 台架异常（不许当结果用）：{type(exc).__name__}: {exc}", file=sys.stderr)
        traceback.print_exc(limit=3)
        return RC_FAULT

    if result is None:
        print("❌ 没跑出结果", file=sys.stderr)
        return RC_FAULT

    m: MET.Metrics = result["metrics"]
    rows = result["rows"]
    meta = result["meta"]
    # ---- 落盘（每次都写；`--hash-only` 只少打表，不少落盘）----
    base = f"run_{result['phase'] or 'unknown'}"
    steps_path = write_steps_csv(rows, os.path.join(out_dir, f"{base}_steps.csv"))
    metrics_path = write_metrics_json(m, meta, os.path.join(out_dir, f"{base}_metrics.json"))
    sha_steps = sha256_of(steps_path)
    sha_metrics = sha256_of(metrics_path)
    hashes.append({"steps": sha_steps, "metrics": sha_metrics, "run": base})

    if not args.quiet and not args.hash_only and runs == 1:
        print(MET.format_table(m, title=f"台架指标 · {scenario.name} / {arm.name} "
                                        f"（按 transfer 分级）"))
        print(f"逐拍：{steps_path}（{len(rows)} 行）")
        print(f"指标：{metrics_path}")
        print(f"计数器：{len(result['book'].as_metrics())} 项（来源 = 生产任务公开计数）")

    # ---- 结构性判据（退出码 3 = 跑完了但判据不过）----
    rc = 0
    rms = m.values.get("belief_error_rms_m")
    if arm.name == "ideal":
        if not (isinstance(rms, float) and rms == 0.0):
            print(f"❌ 结构性判据失败：ideal 档 belief_error_rms_m 必须 ≡0，实得 {rms!r}",
                  file=sys.stderr)
            rc = RC_STRUCTURAL
    else:
        if not (isinstance(rms, float) and rms > 0.0):
            print(f"❌ 结构性判据失败：realistic 档 belief_error_rms_m 必须 >0，实得 {rms!r}"
                  "（注入未生效）", file=sys.stderr)
            rc = RC_STRUCTURAL
    if result["faulty"]:
        print(f"❌ 生产任务 FAULT：{result['faulty']}（台架自身失真，不许当结果用）",
              file=sys.stderr)
        rc = RC_FAULT

    # ---- 重复跑：逐位比对 ----
    if runs > 1:
        uniq = {(h["steps"], h["metrics"]) for h in hashes}
        same = len(uniq) == 1
        print(f"repeat={runs} steps.csv sha256={sha_steps[:16]} "
              f"metrics.json sha256={sha_metrics[:16]} "
              f"逐位相同={'✅' if same else '❌'}")
        if not same:
            for h in hashes:
                print(f"   {h['run']}: steps {h['steps'][:16]} metrics {h['metrics'][:16]}")
            rc = rc or RC_STRUCTURAL

    if args.json_line and not args.hash_only:
        print(m.line(meta=meta, prefix=RESULT_PREFIX))
    elif args.hash_only:
        # `--hash-only` 也要有机器可读摘要（否则 `--repeat 2 --hash-only` 无法被脚本消费）
        summary = {
            "schema": MET.SCHEMA, "ok": rc == 0, "arm": arm.name,
            "scenario": scenario.name,
            "repeat": {"runs": runs, "identical": len({(h["steps"], h["metrics"])
                                                       for h in hashes}) == 1,
                       "steps_sha256": sha_steps, "metrics_sha256": sha_metrics},
        }
        print(f"{RESULT_PREFIX}_HASH " + json.dumps(summary, ensure_ascii=False,
                                                   sort_keys=True))
    return rc


if __name__ == "__main__":      # pragma: no cover
    sys.exit(main())
