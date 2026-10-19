# -*- coding: utf-8 -*-
r"""dzbrain 主进程 —— 单进程多线程任务架构的入口。

设计要点（对应架构设计 §3）：
- **一个进程、一个 ROS 节点、一个 MultiThreadedExecutor**：ROS 回调只在 executor 线程池里跑，
  回调只做"解码 + 缓冲"；所有决策都在任务线程里，互不阻塞。
- **每个任务点一个线程**（`core.task.TaskThread`）：固定频率 + 独立状态机 + 独立失效兜底。
- **启动门禁**：配置校验 + 路线校验不过就**拒绝启动**（旧代码的静默回退是最大的坑来源）。
- **退出必刹车**：无论正常退出、异常、Ctrl-C，都先发零 Twist 再收摊。

用法：
    python -m dzbrain.main --dry-run                 # 只做启动门禁（配置+路线校验），不起线程
    python -m dzbrain.main                           # 正常上车运行（需要 ROS 环境）
    python -m dzbrain.main --bench --seconds 5       # 无车/无 ROS 的联调：FakeBridge 上跑真实任务线程

## 车端不留地图文件（2026-10-18 用户定版）

「车机内不留地图文件，跑图的规划依据由无人机下发信息」：
车端**不再部署 `config/map.yaml`**（`tools/deploy_to_car.py` 已把它从推送清单移除）。
无人机用 `field_layout` 报文下发本局场地布局 → `DroneTask` 校验后写黑盒
`P_FIELD_LAYOUT` → 本模块 `plan_route_from_field_layout()` 用**内存 dict** 规划
（`planner.load_map(dict)`，不读任何文件）→ 产物写回 `route.tokens` 交给 MissionTask 执行。
**没有布局就不规划、不出发**，绝不回退本地地图文件或旧 `route.tokens`。

* `--map <图>`：**仅开发/仿真入口**（显式传参才用文件；该文件不部署到车）。
* 车端（无可用的地图文件、且 `planner.auto_plan_enabled=true`）→ 启动时先起无人机线程等布局：
  超时（`drone.field_layout_wait_s`，缺省 30 s）/ 布局非法 ⇒ 退出码 3（不规划、不出发）；
  布局规划失败 ⇒ 退出码 4（同样不出发）。

## ★T27 步进循环（docs/step_loop.md，2026-10-18 定稿）

布局 v2（带 `known`）到达后，车端不再"规划一次就交给 MissionTask"，而是进入
**运行期步进循环**（`StepLoop`）：

1. **运行期布局**：`P_FIELD_LAYOUT` 每来一版（version 大者胜）就记下；DRIVING 期间
   只记不换段（**换段只在段边界**）。
2. **段规划**：`MapSpec{未知格=不可通行（blocked_extra，不是物理障碍）}`，
   `from_cell=当前格`、`target=layout.target_cell`；引擎按 `planner.engine`。
   规划失败/无 target/无位姿 ⇒ **WAIT_INFO，不退出**。
3. **段执行**：`MissionTask.request_segment()`（线程安全，10 Hz tick 里载入并发车），
   段末停 target_cell；`P_CAR_STATE` 回报 `phase/segment_id/known_count/target_cell/block`。
4. **WAIT_INFO**：停车保持位姿；**无超时**（`step.wait_timeout_s: 0 = 无限`），
   每 5 s 日志；收到新布局立即回 PLANNING。等多久都不会被判"完赛/卡死"。
5. **到 end 格** ⇒ FINISH（`P_CAR_STATE.phase=FINISH`，主循环退出码 0）。
6. **安全**：实际 belief 连续 3 拍落入未知格 ⇒ SAFE_STOP（停段 + 刹车，不再自动恢复）。

★ v1 布局（没有 `known` 键，旧机端）行为**逐字节不变**：仍走"规划一次"的旧路径
（`test_drone` §⑯ 的 exit 3/4/成功路径就是这条兼容证据）。

★ 退出码：0=正常；3=无布局/装配失败；4=布局规划失败；**5=ROS 环境/位姿管线未就绪**
（域号冲突 DZ_ROS_DOMAIN≠ROS_DOMAIN_ID、重复 dzbrain 实例、/odom/localization 无首帧、
运行期断流）；**6=启动契约不通过**（startup_contract() 表里必需 sub/pub 端点缺失或未匹配，
与 5 分开定位）。5 的现场判据与三条照抄命令见 _ROS_DIAG_COMMANDS。
"""

from __future__ import annotations

import argparse
import importlib
import math
import os
import pathlib
import signal
import sys
import threading
import time
from typing import Any, Callable, Iterable

from .core import geometry as geo
from .core.blackboard import P_POSE, Blackboard
from .core.config import Config
from .core.events import EventBus
from .core.logging import banner, kv_lines
from .core.task import TaskGroup

LOG = print

#: 显式接管的退出信号。**SIGTERM 必须自己接**（取证报告 §2）：
#: 车端 rclpy 默认 `SignalHandlerOptions.ALL` 会在 C++ 层接走 SIGTERM，
#: 它只做 `rcl_shutdown`（上下文失效）—— **不抛异常、不退进程** ⇒ 上下文死、任务线程活、
#: RCLError 刷屏（2026-10-11 16:18 收栈就是这么变成"假崩溃"的）。
_STOP_SIGNALS = ("SIGINT", "SIGTERM")


def install_stop_signals(stop_flag: dict, log: Callable[[str], None] = LOG) -> list[str]:
    """显式接管 SIGINT/SIGTERM：**只置停机标志 + 留痕**，不做任何 rclpy 调用。

    为什么只置位：信号处理器在解释器主线程里"异步"跑，任何非异步信号安全的动作
    （发 ROS 指令、拿锁、print 到半截）都可能死锁或重入。真正的收尾由主循环退出后
    的 `finally` 做（`group.stop_all` → `bridge.shutdown` → `finish_report`）。

    返回实际装上的信号名（测试断言用）。
    """
    installed: list[str] = []
    for name in _STOP_SIGNALS:
        signum = getattr(signal, name, None)
        if signum is None:                       # 平台没有这个信号（Windows 上很少见）
            continue

        def _handler(_signum, _frame, _name=name):
            # ★ 处理器内**只写字典**：不 print、不拿锁、不碰 ROS —— 这是信号上下文，
            #   print 的缓冲锁可能正被主线程持有（收栈时尤其容易），会死锁。
            #   留痕由主循环醒来后打印（见 `_live_loop` / `run_bench`）。
            stop_flag["stop"] = True
            stop_flag["signal"] = _name
        try:
            signal.signal(signum, _handler)
        except (ValueError, OSError):            # 非主线程 / 平台不支持
            continue
        installed.append(name)
    return installed


def ros_context_alive() -> bool:
    """ROS 上下文是否还活着（`rclpy.ok()`）；无 ROS 环境（bench/单测）恒判 True。

    ★ 主循环每拍调它：上下文被**任何来源**销毁（SIGTERM 的 rcl_shutdown、别的
      executor 退出、守护进程收栈）都必须 ≤1 拍退出，而不是让线程继续打 RCLError。
    """
    try:
        import rclpy
    except Exception:
        return True                              # 没有 rclpy = 没有上下文可失（bench）
    try:
        return bool(rclpy.ok())
    except Exception:
        return True                              # 判活本身出错时不误杀正常流程


def default_config_path() -> str:
    """默认配置：车上用 ~/dzrobot/config/robot.yaml；开发机上退回工作区的 config/robot.yaml。"""
    car = pathlib.Path(os.path.expanduser("~/dzrobot/config/robot.yaml"))
    if car.is_file():
        return str(car)
    ws = pathlib.Path(__file__).resolve().parents[3]
    return str(ws / "config" / "robot.yaml")


# --------------------------------------------------------------------------- #
#  启动门禁
# --------------------------------------------------------------------------- #
def load_and_check(config_path: str | None, map_path: str | None,
                   *, force: bool = False, announce: bool = True,
                   allow_missing_map: bool = False) -> tuple[Config, dict, list[str]]:
    """加载配置 + 路线校验。返回 (cfg, map_cfg, errors)。

    ★ `announce=False`：调用方自己会把错误**带来源标注**打印一次。
      否则 `--plan-only` 下门禁报文会打印两遍（`load_and_check` 一遍 + 调用方一遍），
      而且读者分不清那些 error 是针对手写 `route.tokens` 还是针对规划结果。
    ★ `allow_missing_map=True`（车端"布局模式"）：**没有地图文件不算启动错误** ——
      车端本来就不留 `config/map.yaml`，规划依据由无人机 `field_layout` 下发；
      本轮门禁的 `route.tokens` 也只是占位（真正执行的是布局规划产物）。
      只对 `planner.auto_plan_enabled=true` 生效；关自动规划时仍按"地图缺失=拒绝启动"。
    """
    cfg = Config.load(config_path or default_config_path())
    issues = cfg.validate(strict=False)
    errors = [str(i) for i in issues if i.level == "error"]

    ws = pathlib.Path(__file__).resolve().parents[3]
    env_map = os.environ.get("ROBOT_MAP")
    if map_path:
        map_file = pathlib.Path(map_path)
    elif env_map:
        map_file = pathlib.Path(env_map)
    else:
        # 与配置同目录的 map.yaml（车上 ~/dzrobot/config、开发机 <ws>/config）
        map_file = pathlib.Path(cfg.source).parent / "map.yaml"
        if not map_file.is_file():
            map_file = ws / "config" / "map.yaml"
    map_cfg: dict = {}
    if map_file.is_file():
        try:
            import yaml
            map_cfg = yaml.safe_load(map_file.read_text(encoding="utf-8")) or {}
        except Exception as exc:               # 地图坏了也要报出来，不能当没事
            errors.append(f"地图文件读取失败: {map_file}: {exc}")
    elif allow_missing_map and bool(cfg.get("planner.auto_plan_enabled", False)):
        # ★ 车端不留地图文件：布局由无人机下发，启动门禁不再因"没有 map.yaml"拒绝启动。
        #   注意这里**不猜任何默认地图**（map_cfg 保持空）—— 后续规划只认 P_FIELD_LAYOUT。
        if announce:
            LOG("")
            LOG(f"地图：车端无地图文件（{map_file} 不存在）→ 规划依据由无人机 "
                f"field_layout 下发（启动后等待布局，未收到则不起任务、不出发）")
    else:
        errors.append(f"地图文件不存在: {map_file}")

    route_errors: list[str] = []
    if map_cfg and not errors:
        try:
            from .route.parser import parse_mission
            from .route.validate import validate_route
            route = parse_mission(cfg.get("route.tokens") or [])
            for iss in validate_route(route, map_cfg=map_cfg, cfg=cfg):
                if iss.level == "error":
                    route_errors.append(str(iss))
        except Exception as exc:
            route_errors.append(f"路线校验异常: {type(exc).__name__}: {exc}")
    errors.extend(route_errors)

    if errors and not force and announce:
        LOG("")
        banner(["启动门禁未通过（拒绝启动）"] + errors, char="!", log=LOG)
        LOG("提示：确认无误可用 --force 跳过（现场应急），但不建议。")
    return cfg, map_cfg, errors


# --------------------------------------------------------------------------- #
#  车端不留地图文件：无人机 field_layout → 内存 MapSpec → 自规划
# --------------------------------------------------------------------------- #
#: 与 `tasks.drone.P_FIELD_LAYOUT` 同值（这里不 import 任务模块，避免装配前加载无人机链）。
P_FIELD_LAYOUT = "field_layout"


def load_field_layout(bb: Blackboard | None) -> dict[str, Any] | None:
    """读黑盒 `P_FIELD_LAYOUT` 并**再校验一次**；没有 / 非法 → `None`（调用方据此"不规划"）。

    为什么要二次校验：黑盒对写入方没有类型约束（测试/别处也可能写），而这份 dict
    直接喂给规划器 —— 一次防御性归一化比"相信上游"便宜得多。
    """
    if bb is None:
        return None
    from .tasks.drone import normalize_field_layout

    raw = bb.get(P_FIELD_LAYOUT)
    if not isinstance(raw, dict):
        return None
    layout, _reason = normalize_field_layout(raw)
    return layout


def plan_route_from_field_layout(cfg: Config, layout: dict[str, Any], *,
                                 log: Callable[[str], None] | None = None,
                                 cross_lower: str | None = None,
                                 cross_upper: str | None = None) -> Any:
    """用无人机下发的**内存布局**规划路线（**永不抛异常**；失败 → `PlanOutcome.ok=False`）。

    与 `tasks.mission.plan_route_from_config()` 的**唯一区别**是地图来源：
    这里直接 `{"map": layout}` → `planner.load_map(dict)`，**不读任何文件**；
    引擎选择 / 预算 / 双射击停站 / 双门禁 / 足迹诊断复用同一组 helper，
    保证"同一张图、同一份配置"下与文件入口 **逐 token 相同**（`test_drone` §⑮ 钉住）。

    ★ 失败时**绝不**回退地图文件或旧 `route.tokens`：调用方据此拒绝发车。
    """
    from .route import planner as PL
    from .route.parser import parse_token
    from .tasks.mission import (DEFAULT_PLAN_ENGINE, ENGINE_TRACKER_SAFE, GateResult,
                                PlanOutcome, check_planned_route, footprint_diagnostic,
                                plan_budget_seconds, resolve_plan_engine)

    # 形状与 map.yaml 的 `map:` 段一致（展平 dict 会丢 barrier_* 键，见 validate._map_section）
    map_cfg = {"map": dict(layout)}
    map_text = "drone:field_layout"
    engine = resolve_plan_engine(cfg, log)
    want_reverse = bool(cfg.get("planner.allow_reverse", False))
    want_lower = str(cross_lower or "").strip().upper() or None
    want_upper = str(cross_upper or "").strip().upper() or None
    try:
        budget = plan_budget_seconds(cfg, engine, log)
    except (TypeError, ValueError):
        budget = 2.0
    try:
        pivot_penalty = float(cfg.get("planner.pivot_penalty", 0.0) or 0.0)
    except (TypeError, ValueError):
        pivot_penalty = 0.0
    gate = GateResult()
    gate_ran = False
    try:
        PL.load_map(map_cfg)                       # 提前校验：坏布局不进规划器
        if engine == ENGINE_TRACKER_SAFE:
            res = PL.plan_route_tracker_safe(map_cfg, cfg, cross_lower=want_lower,
                                             cross_upper=want_upper,
                                             max_seconds=budget, source=map_text)
        else:
            res = PL.plan_route(map_cfg, cfg, cross_lower=want_lower,
                                cross_upper=want_upper, allow_reverse=want_reverse,
                                pivot_penalty=pivot_penalty,
                                max_seconds=budget, source=map_text)
        tokens = [str(t) for t in (res.tokens or [])]
        if not tokens:
            raise ValueError("规划器返回了空 token 列表")
        tokens, second_added, _second_note = PL.ensure_second_shoot_stop(
            tokens, map_cfg, cfg, source=map_text)
        gate_detail = f"缺省引擎 {DEFAULT_PLAN_ENGINE} 不跑门禁（历史行为逐字节不变）"
        if engine == ENGINE_TRACKER_SAFE:
            gate = check_planned_route(tokens, map_cfg, cfg, log=log)
            gate_ran = True
            gate_detail = gate.summary
            if not gate.ok:
                raise ValueError(f"新引擎 {engine} 产出未过门禁：{gate_detail}")
        fp = (gate.footprint if (engine == ENGINE_TRACKER_SAFE and gate.footprint is not None)
              else footprint_diagnostic(tokens, map_cfg, cfg))
        crossing = dict(getattr(res, "crossing", None) or {})
        shoot_stops = tuple(parse_token(t)["block"] for t in tokens if parse_token(t)["shoot"])
        return PlanOutcome(
            ok=True, tokens=tokens,
            elapsed_ms=float(getattr(res, "elapsed_ms", 0.0) or 0.0),
            error="", map_path=map_text, allow_reverse=want_reverse,
            route_len_m=float(getattr(res, "polyline_m", 0.0) or 0.0),
            reverse_segments=int(getattr(res, "reverse_segments", 0) or 0),
            cost=float(getattr(res, "cost", 0.0) or 0.0),
            engine=engine, gate_ok=gate.ok, gate_detail=gate_detail,
            footprint=(fp.summary if fp is not None else ""),
            footprint_ok=(fp.ok if fp is not None else True),
            cross_lower=str(crossing.get("lower") or ""),
            cross_upper=str(crossing.get("upper") or ""),
            shoot_stops=shoot_stops, second_stop_added=second_added,
        )
    except Exception as exc:                       # noqa: BLE001  ★ 规划失败一律不许崩
        msg = f"{type(exc).__name__}: {exc}"
        if log is not None:
            log(f"布局规划失败：{msg}（不会回退地图文件/旧路线）")
        return PlanOutcome(ok=False, tokens=[], elapsed_ms=0.0, error=msg,
                           map_path=map_text, allow_reverse=want_reverse, engine=engine,
                           gate_ok=(gate.ok if gate_ran else True),
                           gate_detail=(gate.summary if gate_ran else ""))


def plan_startup_route(cfg: Config, bb: Blackboard | None, *,
                       log: Callable[[str], None] | None = None) -> Any:
    """启动自规划的唯一入口：**只认内存布局**。

    没有布局（或布局非法）→ 返回 `None`：不调规划器、不用 `route.tokens`、不读地图文件
    ——"不规划"是上车前的硬语义，不是"失败后回退"。
    """
    layout = load_field_layout(bb)
    if layout is None:
        if log is not None:
            log("未收到无人机 field_layout → 不规划（车端不留地图文件，绝不回退 route.tokens）")
        return None
    return plan_route_from_field_layout(cfg, layout, log=log)


# --------------------------------------------------------------------------- #
#  ★T27 步进循环（docs/step_loop.md）：运行期布局 → 段规划 → 段执行 → 段回报
# --------------------------------------------------------------------------- #
#: 车→机回报键：车端步进状态（dict）。字段见 StepLoop._publish_car_state()。
#: ★ 键名与 T28（机端 car_state 发送）对齐：phase / segment_id / known_count / target_cell
#:   / block / **stop_kind**（T32：'shoot'|'mine'|'path'|'end'，机端据此知道"这段为什么停"）。
#:   当前常量暂放这里（与 P_FIELD_LAYOUT 同样的历史做法：不 import 任务模块）；
#:   T26/T28 收口时可整体迁到 core/blackboard.py，字符串值保持不变即可。
P_CAR_STATE = "car_state"

#: 步进阶段（写进 P_CAR_STATE["phase"]，也用于日志/测试断言）
STEP_PHASE_WAIT_LAYOUT = "WAIT_LAYOUT"
STEP_PHASE_PLANNING = "PLANNING"
STEP_PHASE_DRIVING = "DRIVING"
STEP_PHASE_WAIT_INFO = "WAIT_INFO"
STEP_PHASE_SEGMENT_DONE = "SEGMENT_DONE"
STEP_PHASE_FINISH = "FINISH"
STEP_PHASE_SAFE_STOP = "SAFE_STOP"
STEP_PHASE_WAIT_TIMEOUT = "WAIT_INFO_TIMEOUT"
#: ★2026-10-18 现场事故：DRIVING 中 /odom/localization 断流 ⇒ 任务层拿着旧信念空跑。
#: 运行期位姿看门狗判死后的阶段（main 退出码 5；不自动恢复，必须人查图/域号后重起）。
STEP_PHASE_ROS_LOST = "ROS_LOST"

#: belief 连续落在未知格的确认拍数（10 Hz ⇒ 0.3 s）。单拍投影抖动不判死；
#: 连续 3 拍仍不在 known 里 ⇒ SAFE_STOP（docs/step_loop.md §3.5 的"实际投影落入未知格"）。
#: 刻意不新增配置键（"所有新增配置键必须进 SCHEMA"，本卡不碰 robot.yaml/SCHEMA）。
_STEP_UNKNOWN_STOP_TICKS = 3


def _step_cells(value: Any, key: str, *, allow_empty: bool = True
                ) -> tuple[list[str] | None, str]:
    """布局 v2 的格名列表校验 → (大写去重列表, 错误原因)；非法返回 (None, 原因)。"""
    if not isinstance(value, (list, tuple)):
        return None, f"{key} 必须是格名列表（收到 {type(value).__name__}）"
    out: list[str] = []
    seen: set[str] = set()
    for item in value:
        if not isinstance(item, str):
            return None, f"{key} 含非字符串条目 {item!r}"
        text = item.strip().upper()
        if not text:
            return None, f"{key} 含空格名"
        try:
            geo.block_to_index(text)
        except ValueError as exc:
            return None, f"{key} 含非法格名 {text!r}（{exc}）"
        if text not in seen:
            seen.add(text)
            out.append(text)
    if not out and not allow_empty:
        return None, f"{key} 不能为空"
    return out, ""


def normalize_step_layout(payload: Any) -> tuple[dict[str, Any] | None, str]:
    """布局 → 步进循环的归一化 dict（**兼容 v1**；非法整份拒收）。

    在 tasks.drone.normalize_field_layout()（v1 字段：start/end/mines/obstacles/
    targets/bridges/barrier_* + version/ts）之上，补 v2 三字段：
      · known        —— 已观测格（**缺这个键 = v1 布局**，历史语义"整图已知"）；
      · target_cell  —— 本段目标格（空 ⇒ 车只等不做）；
      · segment_id   —— 段序号（回车端回报原样带回）。
    校验规则（docs/step_loop.md §2/§3）：known 里只许合法格名；mines/obstacles/targets
    **必须 ⊆ known**（半张图不能拿来规划）；target_cell 非法/segment_id 非整数 ⇒ 整份拒收。

    ★ 为什么 v1 布局不报错而返回"无 known"的原 dict：T27 必须能在**旧机端**（只发 v1）
      下逐字节保持 T22 行为（test_drone §⑯ 钉住 exit 3/4/成功路径）；run_live 用
      _step_mode_enabled()（= 有没有 known 键）决定走步进循环还是旧"规划一次"。
    """
    from .tasks.drone import normalize_field_layout

    base, reason = normalize_field_layout(payload)
    if base is None:
        return None, reason
    if not isinstance(payload, dict):
        return None, f"报文不是 JSON 对象（{type(payload).__name__}）"
    if "known" not in payload:
        # 半个 v2（有 target_cell/segment_id 却没 known）不能当 v1 放行：那会让
        # "目标格"被悄悄忽略，车永远等不到段。
        if (payload.get("target_cell") not in (None, "")
                or payload.get("segment_id") not in (None, 0, "0")):
            return None, "布局带 target_cell/segment_id 但缺 known（v2 必须给 known）"
        return base, ""

    known, reason = _step_cells(payload.get("known"), "known")
    if known is None:
        return None, reason
    feature = set(base["mines"]) | set(base["obstacles"]) | set(base["targets"])
    outside = sorted(feature - set(known))
    if outside:
        return None, ("特征格不在 known 里（特征必须 ⊆ known）："
                      + " ".join(outside[:8]) + (" …" if len(outside) > 8 else ""))

    target = ""
    raw_target = payload.get("target_cell")
    if raw_target not in (None, ""):
        if not isinstance(raw_target, str):
            return None, f"target_cell 不是字符串：{raw_target!r}"
        target = raw_target.strip().upper()
        try:
            geo.block_to_index(target)
        except ValueError as exc:
            return None, f"target_cell 非法：{target!r}（{exc}）"

    raw_seg = payload.get("segment_id", 0)
    try:
        segment_id = int(raw_seg if raw_seg is not None else 0)
    except (TypeError, ValueError):
        return None, f"segment_id 不是整数：{raw_seg!r}"

    out = dict(base)
    out["known"] = known
    out["target_cell"] = target
    out["segment_id"] = segment_id
    return out, ""


def load_step_layout(bb: Blackboard | None) -> dict[str, Any] | None:
    """读黑盒 P_FIELD_LAYOUT → 步进布局（含 known/target_cell/segment_id）；非法 → None。"""
    if bb is None:
        return None
    raw = bb.get(P_FIELD_LAYOUT)
    if not isinstance(raw, dict):
        return None
    layout, _reason = normalize_step_layout(raw)
    return layout


def _step_mode_enabled(layout: Any) -> bool:
    """这份布局是否走"运行期步进循环"（v2 = 带 known 键）；v1 保持旧"规划一次"行为。"""
    return isinstance(layout, dict) and "known" in layout


def all_map_cells() -> list[str]:
    """整张图的格名（行优先）：unknown = 全图 - known 的唯一口径。"""
    return [geo.index_to_block(r, c)
            for r in range(geo.MAP_ROWS) for c in range(geo.MAP_COLS)]


def _segment_map_cfg(layout: dict[str, Any], from_cell: str,
                     target_cell: str) -> dict[str, Any]:
    """步进布局 → 段规划用的 `{"map": {...}}`（**未知格不进这里**，见下一函数）。

    单独抽出来：`plan_segment_from_layout()` 与 `classify_segment_target()`（空段分支
    的停站分类）必须看到**同一份**地图要素，否则"规划时判成射击格、回报时说 path"。
    """
    return {"map": {
        "name": "drone:step", "start": str(from_cell).strip().upper(),
        "end": str(target_cell).strip().upper(),
        "mines": list(layout.get("mines") or []),
        "obstacles": list(layout.get("obstacles") or []),
        "targets": list(layout.get("targets") or []),
        "bridges": list(layout.get("bridges") or []),
        "barrier_col_from": layout.get("barrier_col_from", geo.BARRIER_COL_FROM),
        "barrier_row_hi": int(layout.get("barrier_row_hi", geo.BARRIER_ROW_HI)),
        "barrier_row_lo": int(layout.get("barrier_row_lo", geo.BARRIER_ROW_LO)),
    }}


def classify_segment_target(cfg: Config, layout: dict[str, Any], target_cell: str) -> str:
    """段目标格的停站类型（`end`/`shoot`/`mine`/`path`；与 `SegmentPlan.stop_kind` 同源）。

    为什么单独成函数：`_plan()` 的"已在目标格"空段分支**不经过规划器**（直接判段完成），
    也要如实回报 `stop_kind` —— 否则空段永远报 `path`，机端会把"射击停站空段"当普通到达。
    ★ T21 判据只在 `tasks.mission.stop_kind_for_cell()` 一处实现，这里只负责把 map_cfg 拼好。
    """
    from .tasks.mission import SEG_STOP_PATH, stop_kind_for_cell

    target = str(target_cell or "").strip().upper()
    if not target:
        return SEG_STOP_PATH
    try:
        src = str(layout.get("start") or "").strip().upper() or target
        kind, _dist = stop_kind_for_cell(
            _segment_map_cfg(layout, src, target), target, cfg,
            end_cell=str(layout.get("end") or "").strip().upper())
        return kind
    except Exception:                                  # noqa: BLE001  分类失败绝不挡段
        return SEG_STOP_PATH


def plan_segment_from_layout(cfg: Config, layout: dict[str, Any], *,
                             from_cell: str, target_cell: str,
                             heading_dir: tuple[int, int] | None = None,
                             log: Callable[[str], None] | None = None) -> Any:
    """用**步进布局**规划一段：未知格（不在 known）= 不可通行，但**不是**物理障碍。

    返回 tasks.mission.SegmentPlan（永不抛异常；失败原因在 .error）。
    map_cfg 只把**真障碍/靶格/雷/桥洞/隔板**给规划器；未知格通过 blocked_extra
    传（只影响通行性），否则一格宽的 known 走廊会因为"把未知格当水桶算净空"而无解。

    ★T32 段末停站语义：目标格命中**射击停站**（T21 判据，见
    `tasks.mission.stop_kind_for_cell()`）时，`plan.tokens` 末项已由
    `plan_segment_route()` 标成 `C9*` ⇒ MissionTask 到站后走三段式（打靶）再报段完成。
    目标格 = 布局终点时本函数把 `stop_kind` 覆盖成 `end`（终点绝不停站打靶）。
    """
    from .tasks.mission import (SEG_STOP_END, SEG_STOP_MINE, SEG_STOP_SHOOT,
                                SegmentPlan, plan_segment_route)

    known = [str(c) for c in (layout.get("known") or [])]
    known_set = set(known)
    target = str(target_cell).strip().upper()
    src = str(from_cell).strip().upper()
    if not known:
        return SegmentPlan(ok=False, from_cell=src, target_cell=target,
                           error="布局里 known 为空（无人机还没看到任何格）")
    if target not in known_set:
        return SegmentPlan(ok=False, from_cell=src, target_cell=target,
                           error=f"目标格 {target} 不在 known 里（车绝不进未知格）")
    # ⚠️ 当前格若还没被观测到，**只对起点破例**：车物理上就在那里，把它判成未知格
    #    会让每一步都规划失败（真·死锁）。除起点外一律不许进未知格。
    unknown = [c for c in all_map_cells() if c not in known_set and c != src]
    map_cfg = _segment_map_cfg(layout, src, target)
    plan = plan_segment_route(map_cfg, cfg, from_cell=src, target_cell=target,
                              heading_dir=heading_dir, blocked_extra=unknown,
                              source="drone:step_layout")
    if plan.ok:
        bad = [c for c in plan.cells if c in set(unknown)]
        if bad:                                        # 双保险：规划器保证 ∩ 这里再查一次
            return SegmentPlan(ok=False, from_cell=src, target_cell=target,
                               error=f"段路线进入未知格（自检拒收）：{' '.join(bad[:5])}")
        # ★T32：终点优先 —— 到 end 就 FINISH，绝不在终点停站打靶（task-33 要求 2）
        end_cell = str(layout.get("end") or "").strip().upper()
        if end_cell and plan.target_cell == end_cell:
            plan.stop_kind = SEG_STOP_END
            plan.stop_distance_m = 0.0
        if log is not None:
            log(f"段规划：{src} → {target}｜{len(plan.cells)} 格 / {plan.route_len_m:.2f} m / "
                f"{plan.elapsed_ms:.1f} ms｜{plan.engine_line}"
                f"｜停站 {plan.stop_kind}"
                + (f"（到靶板中心 {plan.stop_distance_m:.3f} m）"
                   if plan.stop_kind == SEG_STOP_SHOOT else "")
                + (("｜⚠️ " + "；".join(plan.warnings)) if plan.warnings else ""))
            if plan.stop_kind == SEG_STOP_SHOOT:
                log(f"★ 段末射击停站：{target} ⇒ 末 token {plan.tokens[-1] if plan.tokens else target}"
                    f"（到站后 SETTLE→ACQUIRE→FIRE，打完再报段完成）")
            elif plan.stop_kind == SEG_STOP_MINE:
                log(f"段末压雷停站：{target}（目标是雷格：车压过即可，不停站、不标 *）")
    return plan


def _find_task(group: Any, name: str) -> Any:
    """在任务组里按线程名找任务（StepLoop 需要 MissionTask 的段执行接口）。"""
    for task in getattr(group, "_tasks", ()) or ():
        if getattr(task, "name", "") == name:
            return task
    return None


class StepLoop:
    """车端步进循环（T27）：运行期监听布局 → 段规划 → MissionTask 执行 → 段报告/等待。

    状态机（phase 即 P_CAR_STATE["phase"]）：
        WAIT_LAYOUT → PLANNING → DRIVING → SEGMENT_DONE →（新布局）PLANNING
                                                ↘（无新布局）WAIT_INFO →（新布局）PLANNING
        SEGMENT_DONE（目标 = end）→ FINISH；规划失败/无目标/无位姿 → WAIT_INFO（不退出）
    ★ 三条硬语义：
      ① WAIT_INFO **无超时**（step.wait_timeout_s 缺省 0 = 无限；>0 才超时退出）；
      ② **换段只发生在段边界**：DRIVING 期间到达的新布局只记 version，绝不重规划；
      ③ **布局新鲜度**（缺口 E）：规划前 now-收到时刻 > step.layout_ttl_s（缺省 10s，0=关）
         ⇒ WAIT_INFO(reason=layout_stale)，不拿过期布局规划；version 递增的新布局到达即清零。
         sender ts 只作诊断（机/车时钟不同步，不参与门控）。
    """

    def __init__(self, cfg: Config, bb: Blackboard, bridge: Any, mission: Any, *,
                 bus: Any = None, log: Callable[[str], None] = LOG,
                 clock: Callable[[], float] | None = None,
                 layout_getter: Callable[[], dict | None] | None = None,
                 planner: Callable[..., Any] | None = None) -> None:
        self.cfg = cfg
        self.bb = bb
        self.bridge = bridge
        self.mission = mission
        self.bus = bus
        self._log = log
        if clock is None:
            import time as _time
            clock = _time.monotonic
        self._clock = clock
        self._layout_getter = layout_getter or (lambda: load_step_layout(self.bb))
        self._planner = planner or self._default_planner
        try:
            self._wait_timeout_s = max(0.0, float(cfg.get("step.wait_timeout_s", 0) or 0))
        except (TypeError, ValueError):
            self._wait_timeout_s = 0.0
        # ★2026-10-18 现场防线③：运行期位姿看门狗（DRIVING 中连续这么多秒没新帧 ⇒ ROS_LOST）
        try:
            self._odom_watch_s = max(0.0, float(cfg.get("step.odom_watch_s", 6.0) or 0.0))
        except (TypeError, ValueError):
            self._odom_watch_s = 6.0
        try:
            self._odom_fresh_s = max(1.0, float(cfg.get("safety.pose_timeout_s", 1.5) or 1.5))
        except (TypeError, ValueError):
            self._odom_fresh_s = 1.5
        self._odom_bad_since: float | None = None
        # ★缺口 E（2026-10-19）：布局新鲜度。规划前若 now - 收到时刻 > ttl ⇒ WAIT_INFO(layout_stale)，
        #   不拿过期布局规划；新（version 递增的）布局到达即清零。sender ts 只作诊断，
        #   **不参与门控**（机/车时钟不同步，拿它判过期必然误杀）。
        try:
            self._layout_ttl_s = max(0.0, float(cfg.get("step.layout_ttl_s", 10.0) or 0.0))
        except (TypeError, ValueError):
            self._layout_ttl_s = 10.0
        self._layout_rx_at = 0.0          # monotonic：收到"当前版本"布局的时刻（0=还没有）
        self._layout_sender_ts = 0.0      # 机侧 ts，仅诊断

        self._phase = STEP_PHASE_WAIT_LAYOUT
        self._layout: dict[str, Any] | None = None
        self._layout_version = 0
        self._seen_version = 0
        self._planned_version = -1
        self._target_cell = ""
        self._segment_id = 0
        #: ★T32 本段停站类型（'shoot'|'mine'|'path'|'end'）：随 P_CAR_STATE 上报，
        #: 机端据此知道"停在目标格是打靶/压雷/路过/终点"，不再用 block 猜。
        self._stop_kind = ""
        self._known_count = 0
        self._reason = ""
        self._wait_started = 0.0
        self._last_wait_log = 0.0
        self._last_old_log = 0.0
        self._segments_started = 0
        self._segments_done = 0
        self._replans = 0
        self._last_done_block = ""
        self._last_target = ""
        self._segment_started_at = 0.0
        self._unknown_hits = 0
        self._pose_seeded = False
        self._last_car_state: dict[str, Any] | None = None

    # ------------------------------------------------------------------ #
    #  只读状态（遥测/测试）
    # ------------------------------------------------------------------ #
    @property
    def phase(self) -> str:
        return self._phase

    def status(self) -> dict[str, Any]:
        return {
            "phase": self._phase,
            "layout_version": int(self._layout_version),
            "seen_version": int(self._seen_version),
            "planned_version": int(self._planned_version),
            "target_cell": self._target_cell,
            "segment_id": int(self._segment_id),
            "stop_kind": self._stop_kind,
            "known_count": int(self._known_count),
            "reason": self._reason,
            "segments_started": int(self._segments_started),
            "segments_done": int(self._segments_done),
            "replans": int(self._replans),
            "wait_s": (round(self._clock() - self._wait_started, 2)
                       if self._phase == STEP_PHASE_WAIT_INFO else 0.0),
            # ★缺口 E：布局新鲜度（测试/遥测共用一份口径）
            "layout_ttl_s": self._layout_ttl_s,
            "layout_age_s": (round(self._clock() - self._layout_rx_at, 2)
                             if self._layout_rx_at else 0.0),
            "layout_stale": self._layout_is_stale(self._clock()),
        }

    # ------------------------------------------------------------------ #
    #  起循环 / 单拍
    # ------------------------------------------------------------------ #
    def start(self, layout: dict[str, Any] | None = None) -> None:
        """吃进启动门拿到的首份布局；复位一次 belief（与 T22 auto_start 的复位同源）。"""
        if isinstance(layout, dict):
            self._layout = dict(layout)
            try:
                self._layout_version = int(layout.get("version") or 0)
            except (TypeError, ValueError):
                self._layout_version = 0
            self._seen_version = self._layout_version
            self._planned_version = -1
            self._layout_rx_at = self._clock()
            self._layout_sender_ts = self._layout_ts_of(layout)
            self._phase = STEP_PHASE_PLANNING
            self._reason = "首份布局"
            self._seed_pose(self._layout)
            self._log(f"步进循环启动：首份布局 v{self._layout_version}"
                      f"（known {len(self._layout.get('known') or [])} 格 / "
                      f"target {self._layout.get('target_cell') or '空'} / "
                      f"start {self._layout.get('start')} → end {self._layout.get('end')}）")
        else:
            self._phase = STEP_PHASE_WAIT_LAYOUT
            self._reason = "还没有布局"
        self._publish_car_state(self._clock())

    def tick(self, now: float | None = None) -> str:
        """一拍（10 Hz）：推进状态机 + 写 P_CAR_STATE；返回当前阶段。"""
        now = self._clock() if now is None else float(now)
        self._poll_layout(now)
        if self._phase == STEP_PHASE_PLANNING:
            self._plan(now)
        elif self._phase == STEP_PHASE_DRIVING:
            self._drive_tick(now)
        elif self._phase == STEP_PHASE_SEGMENT_DONE:
            self._after_segment_done(now)
        elif self._phase == STEP_PHASE_WAIT_INFO:
            self._wait_tick(now)
        self._publish_car_state(now)
        return self._phase

    def run(self, *, stop_flag: dict, alive: Callable[[], bool] = ros_context_alive,
            sleep: Callable[[float], None] = time.sleep,
            poll_s: float = 0.1, finish_wait_s: float = 2.0,
            log: Callable[[str], None] = LOG) -> str:
        """阻塞跑步进循环：返回 finish / stop / context_dead / timeout / safe_stop。"""
        while not stop_flag.get("stop"):
            try:
                ok = bool(alive())
            except Exception:                      # noqa: BLE001  判活本身出错不误杀
                ok = True
            if not ok:
                log("‼️ 步进循环：ROS 上下文已失效 → 立即停止（不再空转刷 RCLError）")
                return "context_dead"
            phase = self.tick()
            if phase == STEP_PHASE_FINISH:
                if finish_wait_s > 0:
                    sleep(finish_wait_s)
                return "finish"
            if phase == STEP_PHASE_WAIT_TIMEOUT:
                return "timeout"
            if phase == STEP_PHASE_SAFE_STOP:
                return "safe_stop"
            if phase == STEP_PHASE_ROS_LOST:
                return "ros_lost"
            sleep(poll_s)
        sig = stop_flag.get("signal")
        log(f"收到信号 {sig}：步进循环停止（刹车由收尾统一发）" if sig
            else "收到停机请求：步进循环停止")
        return "stop"

    # ------------------------------------------------------------------ #
    #  状态机各步
    # ------------------------------------------------------------------ #
    @staticmethod
    def _layout_ts_of(layout: dict[str, Any]) -> float:
        """布局报文里的机侧 ts（**只作诊断**：机/车时钟不同步，绝不拿它判过期）。"""
        try:
            return float(layout.get("ts") or 0.0)
        except (TypeError, ValueError):
            return 0.0

    def _layout_is_stale(self, now: float) -> bool:
        """当前手上这份布局是否已过期（step.layout_ttl_s；0=关闭；还没收到过=False）。"""
        return bool(self._layout_ttl_s > 0.0 and self._layout_rx_at > 0.0
                    and (now - self._layout_rx_at) > self._layout_ttl_s)

    def _layout_stale_reason(self, now: float) -> str:
        """layout_stale 的 WAIT_INFO 原因（reason 以 layout_stale 开头，测试/机端可判）。"""
        age = max(0.0, now - self._layout_rx_at)
        note = (f"，sender_ts={self._layout_sender_ts:g}"
                if self._layout_sender_ts else "")
        return (f"layout_stale：布局已 {age:.1f}s 未更新"
                f"（step.layout_ttl_s={self._layout_ttl_s:g}s{note}；"
                f"等无人机下一份布局，不拿旧布局规划）")


    def _poll_layout(self, now: float) -> None:
        layout = self._layout_getter()
        if not isinstance(layout, dict):
            return
        try:
            version = int(layout.get("version") or 0)
        except (TypeError, ValueError):
            version = 0
        if version <= self._seen_version:
            if version < self._seen_version and now - self._last_old_log >= 5.0:
                self._last_old_log = now
                self._log(f"忽略旧版布局 v{version}（当前 v{self._seen_version}；"
                          "车端只接受 >= 当前版本）")
            return
        self._seen_version = version
        self._layout_version = version
        self._layout = dict(layout)
        # ★缺口 E：version 递增 = 新布局到达 ⇒ 新鲜度计时清零（同版心跳不刷新，见 _plan 注释）
        self._layout_rx_at = now
        self._layout_sender_ts = self._layout_ts_of(layout)
        if self._phase == STEP_PHASE_WAIT_LAYOUT:   # 启动门没给首份布局时也能自愈
            self._phase = STEP_PHASE_PLANNING
        target = str(layout.get("target_cell") or "") or "（空，只等不做）"
        suffix = "（本段执行中 → 段边界再重规划）" if self._phase == STEP_PHASE_DRIVING else ""
        self._log(f"布局运行期到达：v{version}｜known {len(layout.get('known') or [])} 格"
                  f"｜target={target}｜segment={layout.get('segment_id', 0)}{suffix}")

    def _plan(self, now: float) -> None:
        layout = self._layout
        if layout is None:
            self._to_wait_info("没有可用的布局", now)
            return
        try:
            self._planned_version = int(layout.get("version") or 0)
            self._segment_id = int(layout.get("segment_id") or 0)
        except (TypeError, ValueError):
            self._planned_version = self._seen_version
            self._segment_id = 0
        self._target_cell = str(layout.get("target_cell") or "").strip().upper()
        self._known_count = len(layout.get("known") or [])
        # ★T32 先清空：规划失败/无目标时 P_CAR_STATE 不该带着上一段的停站类型
        self._stop_kind = ""
        if not self._target_cell:
            self._to_wait_info("布局未给 target_cell（车只等不做）", now)
            return
        from_cell = self._current_block(prefer_prev=True)
        if not from_cell:
            self._to_wait_info("没有位姿（P_POSE 缺失）→ 无法确定当前格", now)
            return
        if from_cell == self._target_cell:
            # 空段不经过规划器：停站类型单独分类（否则空段永远报 path）
            self._stop_kind = classify_segment_target(self.cfg, layout, self._target_cell)
            self._mark_segment_done(now, block=from_cell, elapsed=0.0, skipped=True)
            return
        # ★缺口 E：规划前先验布局新鲜度（过期 ⇒ WAIT_INFO(layout_stale)，绝不拿旧布局规划）。
        #   注意 _planned_version 已在上面写成当前版本 ⇒ _wait_tick 不会每拍重复规划；
        #   只有 version 递增的新布局才会把本段重新拉回 PLANNING。
        if self._layout_is_stale(now):
            self._to_wait_info(self._layout_stale_reason(now), now)
            return
        self._replans += 1
        try:
            plan = self._planner(layout, from_cell, self._target_cell, self._heading_dir())
        except Exception as exc:                   # noqa: BLE001  ★ 规划异常绝不炸主循环
            self._to_wait_info(f"段规划异常：{type(exc).__name__}: {exc}", now)
            return
        if not plan.ok:
            self._to_wait_info(f"段规划失败：{plan.error}", now)
            return
        # ★T32 段末停站类型（规划器已按 T21 判据算好；终点由 plan_segment_from_layout 覆盖）
        self._stop_kind = str(getattr(plan, "stop_kind", "") or "")
        if plan.single_cell:
            self._mark_segment_done(now, block=self._target_cell, elapsed=0.0, skipped=True)
            return
        res = self.mission.request_segment(
            plan.tokens, segment_id=self._segment_id, target_cell=self._target_cell,
            known_count=self._known_count, source=f"step:v{self._planned_version}",
            stop_kind=self._stop_kind)
        if not res.get("accepted"):
            # 不是失败：任务线程可能还在上一段收尾（≤1 拍）。planned_version=-1
            # 让下一拍 _wait_tick 立刻转回 PLANNING 重试。
            self._planned_version = -1
            self._to_wait_info(f"任务线程未接收本段：{res.get('reason')}", now, retry=True)
            return
        self._segments_started += 1
        self._segment_started_at = now
        self._phase = STEP_PHASE_DRIVING
        self._reason = ""
        self._last_target = self._target_cell
        self._unknown_hits = 0
        self._log(f"▶ 段 {self._segment_id} 下发：{from_cell} → {self._target_cell}"
                  f"｜{len(plan.cells)} 格 / {plan.route_len_m:.2f} m / {plan.elapsed_ms:.1f} ms"
                  f"｜{plan.engine_line}｜停站 {self._stop_kind or '未标注'}"
                  f"｜known {self._known_count} 格"
                  + (("｜⚠️ " + "；".join(plan.warnings)) if plan.warnings else ""))

    def _drive_tick(self, now: float) -> None:
        # ★ 防线③：运行期位姿看门狗（先于任何"用旧位姿判完成"的判定）
        if self._odom_watch(now):
            return
        st = self.mission.segment_status() if hasattr(self.mission, "segment_status") else {}
        err = str(st.get("error") or "")
        if err:
            self._to_wait_info(f"段执行失败：{err}", now)
            return
        done = st.get("done") if isinstance(st.get("done"), dict) else None
        if done and int(done.get("segment_id", -1)) == self._segment_id:
            self._mark_segment_done(now, block=str(done.get("block") or self._target_cell),
                                    elapsed=float(done.get("elapsed_s") or 0.0),
                                    stop_kind=str(done.get("stop_kind") or ""))
            return
        if self._unknown_violation():
            self._to_safe_stop("belief 连续落在未知格（实际投影进入未知区域）", now)
            return
        if str(st.get("phase") or "") == "IDLE" and not done:
            # 任务线程既没在跑段、也没给完成单 ⇒ 路线被外部命令清空/复位了。
            # 给 1 s 宽限（段请求刚投递、任务线程还没取走时也是 IDLE）。
            if now - self._segment_started_at > 1.0:
                self._to_wait_info("任务线程回到 IDLE（路线被外部命令清空？）", now)

    def _after_segment_done(self, now: float) -> None:
        end = str((self._layout or {}).get("end") or "").strip().upper()
        if end and self._last_done_block == end:
            self._to_finish(now)
            return
        if self._seen_version > self._planned_version:
            self._phase = STEP_PHASE_PLANNING
            self._reason = "段边界：新布局已到 → 立即重规划"
            self._log(f"段边界：布局 v{self._seen_version}（已处理 v{self._planned_version}）"
                      "→ 转 PLANNING")
            return
        self._to_wait_info(f"停在 {self._last_done_block or '?'}，等无人机下一份布局", now)

    def _wait_tick(self, now: float) -> None:
        if self._seen_version > self._planned_version:
            self._phase = STEP_PHASE_PLANNING
            self._log(f"WAIT_INFO 收到新布局 v{self._seen_version} → 立即转 PLANNING")
            return
        waited = max(0.0, now - self._wait_started)
        if self._wait_timeout_s > 0.0 and waited >= self._wait_timeout_s:
            self._phase = STEP_PHASE_WAIT_TIMEOUT
            self._reason = (f"WAIT_INFO 超时 {waited:.1f}s"
                            f"（step.wait_timeout_s={self._wait_timeout_s:g}）")
            self._log(f"⛔ {self._reason}")
            return
        if now - self._last_wait_log >= 5.0:
            self._last_wait_log = now
            self._log(f"… WAIT_INFO 已 {waited:.0f}s：{self._reason}"
                      f"｜known {self._known_count} 格"
                      f"｜{'无超时（等下一份布局）' if self._wait_timeout_s <= 0 else f'超时 {self._wait_timeout_s:g}s'}"
                      "｜等无人机下一份布局")

    # ------------------------------------------------------------------ #
    #  状态迁移 + 回报
    # ------------------------------------------------------------------ #
    def _mark_segment_done(self, now: float, *, block: str, elapsed: float,
                           skipped: bool = False, stop_kind: str = "") -> None:
        self._segments_done += 1
        self._last_done_block = str(block or self._target_cell)
        self._last_target = self._last_done_block
        if stop_kind:                          # ★T32：任务线程的完成单是权威（空段用分类结果）
            self._stop_kind = stop_kind
        self._phase = STEP_PHASE_SEGMENT_DONE
        self._reason = "已在目标格（空段）" if skipped else "段完成"
        self._publish_car_state(now)
        self._log(f"■ 段 {self._segment_id} 到达 {self._last_done_block}"
                  f"（停站 {self._stop_kind or '未标注'}"
                  f" / {'空段，未发车' if skipped else f'用时 {elapsed:.2f}s'}）"
                  f"→ P_CAR_STATE phase={STEP_PHASE_SEGMENT_DONE}")

    def _to_wait_info(self, reason: str, now: float, *, retry: bool = False) -> None:
        self._phase = STEP_PHASE_WAIT_INFO
        self._reason = str(reason)
        self._wait_started = now
        self._last_wait_log = now
        try:
            self.bridge.publish_brake()            # 停车保持位姿（Tracker 在 IDLE 也每拍刹）
        except Exception:                          # noqa: BLE001 刹车发不出去不崩循环
            pass
        if not retry:
            self._log(f"⏸ WAIT_INFO：{reason}｜known {self._known_count} 格｜"
                      f"{'无超时（等下一份布局）' if self._wait_timeout_s <= 0 else f'超时 {self._wait_timeout_s:g}s'}")

    def _to_finish(self, now: float) -> None:
        self._phase = STEP_PHASE_FINISH
        self._reason = f"到达终点 {self._last_done_block}"
        try:
            self.bridge.publish_brake()
        except Exception:                          # noqa: BLE001
            pass
        self._publish_car_state(now)
        self._log(f"🏁 步进循环终点：{self._last_done_block} == end → FINISH")

    # ------------------------------------------------------------------ #
    #  运行期位姿看门狗（2026-10-18 现场事故防线③）
    # ------------------------------------------------------------------ #
    def _odom_watch(self, now: float) -> bool:
        """DRIVING 中连续 step.odom_watch_s 秒没有新 /odom/localization ⇒ 判死。

        为什么不在 Safety 里做：Safety 的 pose_stale 只负责"刹住"，它不知道步进循环会
        继续用旧信念规划/发段；现场就是这样 78 秒空跑（指令在发、车不动）。
        阈值语义：起步计时累计，任何一帧新 odom 都把计时清零（间歇断流不误杀）。
        """
        if self._odom_watch_s <= 0.0:
            return False
        try:
            age = float(self.bridge.pose_age())
        except Exception:                                # noqa: BLE001
            age = float("inf")
        if age <= self._odom_fresh_s:
            self._odom_bad_since = None
            return False
        if self._odom_bad_since is None:
            self._odom_bad_since = now
            return False
        if now - self._odom_bad_since < self._odom_watch_s:
            return False
        self._to_ros_lost(now, age)
        return True

    def _to_ros_lost(self, now: float, age: float) -> None:
        """位姿断流 → 停段 + 刹车 + 诊断 + 阶段 ROS_LOST（main 退出码 5，不自动恢复）。"""
        try:
            self.mission.stop_segment("运行期位姿断流（ROS_LOST）")
        except Exception:                                # noqa: BLE001
            pass
        try:
            self.bridge.publish_brake()
        except Exception:                                # noqa: BLE001
            pass
        self._phase = STEP_PHASE_ROS_LOST
        self._reason = (f"/odom/localization 断流 {age:.2f}s"
                        f"（> step.odom_watch_s={self._odom_watch_s:g}s）")
        self._publish_car_state(now)
        self._log(f"⛔ 运行期位姿看门狗：{self._reason} ⇒ 已停段+刹车；"
                  "退出码 5（不再用旧信念半死跑）")
        _ros_diagnosis(self.bridge, log=self._log)

    def _to_safe_stop(self, reason: str, now: float) -> None:
        try:
            self.mission.stop_segment(reason)
        except Exception:                          # noqa: BLE001
            pass
        try:
            self.bridge.publish_brake()
        except Exception:                          # noqa: BLE001
            pass
        self._phase = STEP_PHASE_SAFE_STOP
        self._reason = str(reason)
        self._publish_car_state(now)
        self._log(f"⛔ SAFE_STOP：{reason}（已请求停止本段并刹车；不再自动恢复）")

    def _publish_car_state(self, now: float) -> None:
        data = {
            "phase": self._phase,
            "segment_id": int(self._segment_id),
            "known_count": int(self._known_count),
            "target_cell": self._target_cell,
            # ★T32 段末停站类型：机端据此知道"这段为什么停"（shoot/mine/path/end）
            "stop_kind": self._stop_kind,
            "block": self._current_block(),
            "version": int(self._layout_version if self._layout_version else self._seen_version),
            "reason": self._reason,
            "ts": round(float(now), 3),
            "segments_started": int(self._segments_started),
            "segments_done": int(self._segments_done),
            "replans": int(self._replans),
            "wait_s": (round(float(now) - self._wait_started, 2)
                       if self._phase == STEP_PHASE_WAIT_INFO else 0.0),
        }
        if data == self._last_car_state:
            return
        self.bb.set(P_CAR_STATE, data, source="StepLoop")
        self._last_car_state = data

    # ------------------------------------------------------------------ #
    #  小工具
    # ------------------------------------------------------------------ #
    def _current_block(self, *, prefer_prev: bool = False) -> str:
        pose = self.bb.get(P_POSE)
        if pose is None:
            return ""
        if prefer_prev and self._last_target:
            tx, ty = geo.block_to_meters(self._last_target)
            if math.hypot(pose.x - tx, pose.y - ty) <= 0.5 * geo.BLOCK_SIZE:
                return self._last_target
        return geo.meters_to_block(pose.x, pose.y)

    def _heading_dir(self) -> tuple[int, int] | None:
        pose = self.bb.get(P_POSE)
        if pose is None:
            return None
        from .tasks.mission import heading_dir_from_yaw
        return heading_dir_from_yaw(pose.yaw)

    def _unknown_violation(self) -> bool:
        known = set((self._layout or {}).get("known") or [])
        if not known:
            return False
        block = self._current_block()
        if not block:
            return False
        if block in known:
            self._unknown_hits = 0
            return False
        self._unknown_hits += 1
        return self._unknown_hits >= _STEP_UNKNOWN_STOP_TICKS

    def _seed_pose(self, layout: dict[str, Any]) -> None:
        spec = str(self.cfg.get("mission.reset") or "").strip()
        if not spec:
            spec = f"{layout.get('start') or 'N10'},x"
        try:
            self.mission.reset_pose(spec)
            self._pose_seeded = True
            self._log(f"步进循环：复位 belief = {spec!r}（与 T22 auto_start 的复位同源，只做一次）")
        except Exception as exc:                   # noqa: BLE001 复位失败不阻塞循环
            self._log(f"⚠️ 步进循环复位失败（继续用现有 belief）：{type(exc).__name__}: {exc}")

    def _default_planner(self, layout: dict[str, Any], from_cell: str,
                         target_cell: str, heading_dir: tuple[int, int] | None) -> Any:
        return plan_segment_from_layout(self.cfg, layout, from_cell=from_cell,
                                        target_cell=target_cell, heading_dir=heading_dir,
                                        log=self._log)


def _run_step_loop(loop: StepLoop, *, stop_flag: dict,
                   alive: Callable[[], bool] = ros_context_alive,
                   log: Callable[[str], None] = LOG) -> str:
    """run_live 的步进循环入口（单独成函数便于测试替身/日志留痕）。"""
    why = loop.run(stop_flag=stop_flag, alive=alive, log=log)
    st = loop.status()
    log(f"步进循环退出：{why}｜{st['phase']}｜布局 v{st['seen_version']}"
        f"｜段 下发{st['segments_started']}/完成{st['segments_done']}"
        f"｜重规划 {st['replans']} 次｜known {st['known_count']} 格")
    return why


def _layout_mode_enabled(cfg: Config, map_path: str | None) -> bool:
    """车端"布局模式"：自动规划开启、且**没有可用的地图文件**。

    · `--map <存在的文件>` ⇒ 文件模式（开发/仿真唯一入口；该文件不部署到车）；
    · `--map <不存在的文件>`（start_all.sh 的历史参数在车端就是这种形态）⇒ 布局模式；
    · 不传 `--map` ⇒ 布局模式。
    `planner.auto_plan_enabled=false` 时仍走旧手写路线分支（不需要地图文件）。
    """
    if map_path and pathlib.Path(map_path).is_file():
        return False
    return bool(cfg.get("planner.auto_plan_enabled", False))


def _start_layout_listener(bridge: Any, bb: Blackboard, bus: EventBus,
                           cfg: Config, *, log: Callable[[str], None] = LOG) -> Any:
    """先起一个独立 DroneTask 专等 `field_layout`（规划要在 MissionTask 构造前完成）。"""
    from .tasks.drone import DroneTask

    task = DroneTask(bridge, bb, bus, cfg, log=log)
    task.start()
    return task


# --------------------------------------------------------------------------- #
#  启动自检 + 转场门（2026-10-18 现场 pose_stale 事故的防线）
# --------------------------------------------------------------------------- #
#  现场形态（7_path_step.log 可复核）：/dzlink 全程通，/odom/localization 只在开头到、
#  之后断 13s→22s→271s；Tracker/Aim 的 tick 被阻塞 14.8s/22.1s（Mission 仅 3.9ms）；
#  dzactuator 报 3.7s 未收到速度指令。⇒ 车端 ROS 图（同机 DDS 发现/传输）失配，
#  而决策层**没有任何自检**：它照常规划、照常发 v=0.120，车一步不动，直到 Safety
#  用 1.5s pose_stale 锁 SAFE_STOP —— 排障要两分钟且看不出根因。
#  下面三道门把这类"半死图"变成"当场大声失败 + 给出排查命令"：
#    ① 启动自检（env 域号/RMW + 订阅清单 + 关键话题 pub/sub 计数 + 重复实例）；
#    ② 位姿就绪门（限时等首帧 /odom/localization，超时 exit 5 不出发）；
#    ③ 运行期位姿看门狗（DRIVING 中连续 step.odom_watch_s 秒无新帧 ⇒ exit 5）。

#: 现场排障三条命令（位姿门/看门狗失败时原样打印，现场可直接照抄）
_ROS_DIAG_COMMANDS: tuple[str, ...] = (
    "ros2 topic info /odom/localization -v | head -30   # 订阅者应含 /dzbrain",
    "ros2 node list | grep -E 'dzbrain|serial_reader'   # 两个节点都该在",
    "tr '\\0' '\\n' < /proc/self/environ | grep -E 'ROS_DOMAIN_ID|DZ_ROS_DOMAIN|ROS_LOCALHOST_ONLY|RMW'",
)


def _ros_diagnosis(bridge: Any, *, log: Callable[[str], None] = LOG) -> None:
    """打印可落盘的 ROS 现场诊断（env + 计数 + 最近一帧年龄 + 三条命令）。"""
    from .io.ros_io import format_env_line

    log(format_env_line())
    try:
        counts = bridge.topic_counts()
    except Exception as exc:                             # noqa: BLE001
        counts = {}
        log(f"（话题计数读取失败：{type(exc).__name__}: {exc}）")
    for key, item in (counts or {}).items():
        log(f"  话题计数 {key}: publishers={item.get('publishers')} "
            f"subscribers={item.get('subscribers')}")
    try:
        age = float(bridge.pose_age())
        log(f"  最近一帧 /odom/localization 年龄：{age:.2f}s")
    except Exception:                                    # noqa: BLE001
        pass
    for line in _ROS_DIAG_COMMANDS:
        log("   " + line)


# --------------------------------------------------------------------------- #
#  启动契约（缺口 A）：声明必需 sub/pub，用图上的端点计数对账
# --------------------------------------------------------------------------- #
#  为什么需要：2026-10-18 实车"收到布局、段规划 9.3ms 成功、但 /odom/localization 没有
#  订阅（图里 0）→ pose_stale → 车一步不动"。旧防线只查 env（_startup_self_check）和
#  "第一帧 odom 数据"（_wait_for_pose_ready）；前者查不出端点缺失，后者只能事后等超时。
#  本函数把"谁必须在对的图上"写成可读表格，把三种失败层次分开：
#    ① 本节点**没创建订阅**      -> row.note=本节点未创建订阅        （装配顺序/代码问题）
#    ② 订阅在、对端发布方 0      -> pubs=0                      （DDS 域/RMW/对端未起）
#    ③ 双方都在但数据不来        -> 契约通过，位姿就绪门 exit 5    （QoS/传输/发布异常）
#  分级（减少赛时门禁）：required=赛时就卡；race=只在步进赛时卡；advisory=赛前 prepare 才卡。
#  赛时只验 token 的口径不变：契约只在启动自检跑一次，不进入每段规划。

#: (逻辑键, 方向, 级别, 要求发布方>=1, 要求订阅方>=1, 要求本节点已创建该订阅)
_CONTRACT_ROWS: tuple[tuple[str, str, str, bool, bool, bool], ...] = (
    ("odom", "sub", "required", True, True, True),
    ("pursuit_angle", "pub", "required", True, True, False),
    ("drone_in", "sub", "race", True, True, True),
    ("imu", "sub", "advisory", True, False, True),
    ("offset_center", "sub", "advisory", True, False, True),
    ("aim_status", "sub", "advisory", True, False, True),
    ("shoot_count", "sub", "advisory", True, False, True),
    ("manual_override", "sub", "advisory", True, False, True),
    ("manual_gimbal", "sub", "advisory", True, False, True),
    ("waypoint_cmd", "sub", "advisory", True, False, True),
    # referee_hp 不在 ros_io.REQUIRED_SUBSCRIPTIONS（那份清单是"决策层必建"的会话口），
    # 但仍必须收到对端（命中/弹药唯一真源）⇒ 只查图计数，不查"本节点是否建了订阅"。
    ("referee_hp", "sub", "advisory", True, False, False),
    ("drone_out", "pub", "advisory", True, False, False),
    ("aim_fire_gate", "pub", "advisory", True, False, False),
    ("aim_fire_ready", "pub", "advisory", True, False, False),
    ("scan_request", "pub", "advisory", True, False, False),
    ("motion_state", "pub", "advisory", True, False, False),
    ("voice_switch", "pub", "advisory", True, False, False),
    ("aim_lead", "pub", "advisory", True, False, False),
)

#: 契约模式 -> 哪些级别在本模式里"阻断"（其余只打印/告警）
_CONTRACT_BLOCKING = {
    "live": ("required",),
    "race-step": ("required", "race"),
    "prepare": ("required", "race", "advisory"),
    "bench": (),
}

#: 启动契约总开关：DZ_SKIP_CONTRACT=1 时整体跳过（bench/离线联调用）
CONTRACT_SKIP_ENV = "DZ_SKIP_CONTRACT"


def _contract_skip_requested() -> bool:
    return str(os.environ.get(CONTRACT_SKIP_ENV, "") or "").strip().lower() in (
        "1", "true", "yes", "on")


def _endpoint_counts(bridge: Any, key: str, topic: str) -> tuple[int, int]:
    """一个话题的 (publishers, subscribers)；读不到返回 (-1, -1)（未知≠缺失）。

    优先级：真 rclpy 节点计数（bridge.node.count_publishers/subscribers）→ 替身的
    endpoint_counts(topic) → 替身的 topic_counts()[逻辑键]。未知值不阻断契约，
    只有"明确读到 0"或"本节点确实没建这条订阅"才判红。
    """
    node = getattr(bridge, "node", None)
    if node is not None:
        try:
            return (int(node.count_publishers(topic)),
                    int(node.count_subscribers(topic)))
        except Exception:                                # noqa: BLE001
            pass
    fn = getattr(bridge, "endpoint_counts", None)
    if callable(fn):
        try:
            pair = fn(topic)
            if pair is not None:
                return int(pair[0]), int(pair[1])
        except Exception:                                # noqa: BLE001
            pass
    try:
        counts = bridge.topic_counts() or {}
    except Exception:                                    # noqa: BLE001
        counts = {}
    item = counts.get(key) if isinstance(counts, dict) else None
    if isinstance(item, dict):
        return int(item.get("publishers", -1)), int(item.get("subscribers", -1))
    return -1, -1


def _subscription_topics_of(bridge: Any) -> list[str] | None:
    """本节点已创建的订阅话题名；替身不支持时返回 None（未知，不阻断）。"""
    fn = getattr(bridge, "subscription_topics", None)
    if not callable(fn):
        return None
    try:
        out = fn()
    except Exception:                                    # noqa: BLE001
        return None
    if out is None:
        return None
    try:
        return [str(x) for x in out]
    except TypeError:
        return None


def startup_contract(bridge: Any, mode: str = "live", *,
                     log: Callable[[str], None] | None = None) -> tuple[bool, list[dict]]:
    """启动契约（缺口 A）：逐条对账必需 sub/pub 端点；返回 (ok, rows)。

    :param mode: live（赛时基础，只卡 required）/ race-step（步进赛时，追加 race）
                 / prepare（赛前全量，连 advisory 也卡）/ bench（跳过）
    :param log: 给了就打印整张表（含 ✅/❌/⚠️ 与缺失原因）；None 时不打印。
    rows 字段：dir/key/topic/level/pubs/subs/ok/blocking/note。
    ★ DZ_SKIP_CONTRACT=1 ⇒ 直接 ok=True（一行 note 表），供 bench/离线联调跳过。
    """
    mode = str(mode or "live").strip().lower()
    if mode not in _CONTRACT_BLOCKING:
        mode = "live"
    if _contract_skip_requested() or mode == "bench":
        rows = [{"dir": "-", "key": "-", "topic": "-", "level": mode,
                 "pubs": -1, "subs": -1, "ok": True, "blocking": False,
                 "note": f"已跳过（{CONTRACT_SKIP_ENV}=1 或 mode=bench）"}]
        if log is not None:
            log(f"启动契约：跳过（{CONTRACT_SKIP_ENV}=1 或 mode=bench）")
        return True, rows

    from .io import contract as ct

    blocking_levels = _CONTRACT_BLOCKING[mode]
    created = _subscription_topics_of(bridge)
    rows: list[dict] = []
    for key, direction, level, need_pubs, need_subs, need_own in _CONTRACT_ROWS:
        try:
            topic = ct.topic(key).name
        except Exception:                                # noqa: BLE001 契约里没这条键
            topic = f"<未知键 {key}>"
        pubs, subs = _endpoint_counts(bridge, key, topic)
        problems: list[str] = []
        if need_pubs and pubs == 0:
            problems.append("发布方 0（对端未起/域不同）")
        if need_subs and subs == 0:
            problems.append("订阅方 0（无人接收：QoS 不匹配或对端没订阅）")
        if need_own and created is not None and topic not in set(created):
            problems.append("本节点未创建订阅（装配/代码问题）")
        blocking = level in blocking_levels
        rows.append({"dir": direction, "key": key, "topic": topic, "level": level,
                     "pubs": pubs, "subs": subs,
                     "ok": not problems, "blocking": blocking,
                     "note": "；".join(problems)})
    ok = all(row["ok"] for row in rows if row["blocking"])
    if log is not None:
        log(_format_contract_table(rows, mode=mode, ok=ok))
    return ok, rows


def _format_contract_table(rows: list[dict], *, mode: str = "live",
                           ok: bool = True) -> str:
    """契约表（纯文本，多行）：现场可直接复制进报告/群。"""
    fails = [r for r in rows if not r["ok"]]
    blocks = [r for r in rows if not r["ok"] and r["blocking"]]
    head = (f"启动契约（mode={mode}）：{len(rows)} 项 / 失败 {len(fails)} / "
            f"阻断 {len(blocks)} / {'通过' if ok else '不通过'}")
    lines = [head]
    for row in rows:
        mark = "✅" if row["ok"] else ("❌" if row["blocking"] else "⚠️")
        level = row["level"]
        lines.append(f"  {mark} {row['dir']:>3} {row['key']:<15} {row['topic']:<22}"
                     f" pubs={row['pubs']} subs={row['subs']} [{level}]"
                     + (f"  ← {row['note']}" if row["note"] else ""))
    return "\n".join(lines)


def _log_subscriptions(bridge: Any, *, log: Callable[[str], None] = LOG) -> None:
    """打印决策层真实创建了的订阅清单（与 required_subscription_topics 对账）。"""
    from .io.ros_io import required_subscription_topics

    try:
        actual = list(bridge.subscription_topics())
    except Exception:                                    # noqa: BLE001
        actual = []
    required = required_subscription_topics()
    missing = [key for key, topic in required.items() if topic not in set(actual)]
    log("订阅清单：" + (", ".join(actual) if actual else "<空：ROS 图里不会有本节点订阅>"))
    if missing:
        log(f"⚠️ 订阅清单缺 {missing}（与 required_subscription_topics 对账失败）")


def _startup_self_check(bridge: Any, *, log: Callable[[str], None] = LOG) -> bool:
    """启动自检（env + 订阅清单 + 关键话题计数）。返回 False ⇒ 不出发（exit 5）。"""
    from .io.ros_io import env_problems, format_env_line

    log(format_env_line())
    errors, warnings = env_problems()
    for item in warnings:
        log("⚠️ ROS 环境：" + item)
    for item in errors:
        log("⛔ ROS 环境自检失败：" + item)
    _log_subscriptions(bridge, log=log)
    try:
        counts = bridge.topic_counts()
    except Exception as exc:                             # noqa: BLE001
        counts = {}
        log(f"⚠️ 话题计数读取失败：{type(exc).__name__}: {exc}")
    for key, item in (counts or {}).items():
        log(f"话题计数 {key}: publishers={item.get('publishers')} "
            f"subscribers={item.get('subscribers')}")
    if errors:
        log("⛔ 不规划、不出发（退出码 5）：域号/RMW 不一致时 DDS 图会静默分裂 ——"
            "典型表现是收得到跨机 /dzlink、收不到同机 /odom/localization，车一步不动")
        return False
    return True


def _duplicate_dzbrain_instances() -> list[str]:
    """同机**重复 dzbrain 实例**（同一 ROS 图里同名节点会互抢话题/QoS 匹配）。

    只在有 pgrep 的机器上检查（Windows/测试机返回空表）；查询失败一律返回空表 ——
    这是"安全网"不是"硬依赖"，绝不因为外部命令不可用而拒绝启动。
    """
    import shutil
    import subprocess

    pgrep = shutil.which("pgrep")
    if not pgrep:
        return []
    try:
        proc = subprocess.run([pgrep, "-af", "dzbrain"], capture_output=True,
                              text=True, timeout=2.0)
        out = proc.stdout or ""
    except Exception:                                    # noqa: BLE001
        return []
    me = os.getpid()
    others: list[str] = []
    for line in out.splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) < 2 or not parts[0].isdigit():
            continue
        pid = int(parts[0])
        cmd = parts[1]
        if pid == me:
            continue
        if "dzbrain.main" in cmd or cmd.rstrip().endswith("dzbrain"):
            others.append(line.strip())
    return others


def _wait_for_pose_ready(bridge: Any, cfg: Config, *, stop_flag: dict,
                         alive: Callable[[], bool] = ros_context_alive,
                         log: Callable[[str], None] = LOG,
                         sleep: Callable[[float], None] = time.sleep,
                         now: Callable[[], float] = time.monotonic) -> bool:
    """位姿就绪门：限时等**第一帧** /odom/localization 进决策层；超时 ⇒ False（不出发）。

    窗口 = step.pose_ready_wait_s（缺省 8s；<=0 关闭 —— 测试/bench 用）。
    判据 = bridge.pose() 非空 且 pose_age <= max(1.0, safety.pose_timeout_s)。
    现场教训见文件头注释：没有这道门时，缺 odom 的车会带着**种子位姿**照常下发指令。
    """
    try:
        wait_s = max(0.0, float(cfg.get("step.pose_ready_wait_s", 8.0) or 0.0))
    except (TypeError, ValueError):
        wait_s = 8.0
    try:
        fresh_s = max(1.0, float(cfg.get("safety.pose_timeout_s", 1.5) or 1.5))
    except (TypeError, ValueError):
        fresh_s = 1.5
    if wait_s <= 0.0:
        log("位姿就绪门：已关闭（step.pose_ready_wait_s<=0）")
        return True
    t0 = now()
    deadline = t0 + wait_s
    last_note = t0
    while True:
        pose = None
        age = float("inf")
        try:
            pose = bridge.pose()
            age = float(bridge.pose_age())
        except Exception:                                # noqa: BLE001
            pose, age = None, float("inf")
        if pose is not None and age <= fresh_s:
            log(f"✅ 位姿就绪门通过：/odom/localization 已进决策层（age={age:.2f}s）")
            return True
        if stop_flag.get("stop"):
            return False
        try:
            alive_now = bool(alive())
        except Exception:                                # noqa: BLE001
            alive_now = True
        if not alive_now:
            return False
        if now() - last_note >= 2.0:
            last_note = now()
            log(f"… 位姿就绪门：/odom/localization 还没进来"
                f"（已等 {now() - t0:.1f}/{wait_s:.0f}s，"
                f"pose={'有' if pose is not None else '无'}，age={age:.2f}s）")
        if now() >= deadline:
            break
        sleep(0.1)
    log("⛔ 位姿就绪门超时：窗口内 /odom/localization 没有进入决策层 ⇒ 不规划、不出发（退出码 5）")
    _ros_diagnosis(bridge, log=log)
    return False


def _wait_for_field_layout(bb: Blackboard, cfg: Config, *, stop_flag: dict,
                           alive: Callable[[], bool] = ros_context_alive,
                           log: Callable[[str], None] = LOG,
                           sleep: Callable[[float], None] = time.sleep,
                           now: Callable[[], float] = time.monotonic) -> dict | None:
    """阻塞等待无人机下发布局（缺省 30 s），超时/停机/上下文失效 → `None`（不规划）。

    ★T27：这里返回的是**步进归一化布局**（v2 会带 known/target_cell/segment_id；v1 与
      `load_field_layout()` 逐键相同）—— 启动门据此决定走"步进循环"还是旧"规划一次"。
    """
    wait_s = float(cfg.get("drone.field_layout_wait_s", 30.0) or 30.0)
    wait_s = max(0.0, wait_s)
    t0 = now()
    deadline = t0 + wait_s
    last_note = t0
    while True:
        layout = load_step_layout(bb)
        if layout is not None:
            extra = ("｜known %d 格 target=%s seg=%s → 步进循环"
                     % (len(layout.get("known") or []),
                        layout.get("target_cell") or "空",
                        layout.get("segment_id", 0))) if "known" in layout else \
                    "→ 开始内存地图规划（v1 布局/旧行为）"
            log(f"收到无人机场地布局（起 {layout['start']} 终 {layout['end']} "
                f"雷 {len(layout['mines'])} 障 {len(layout['obstacles'])} "
                f"靶 {len(layout['targets'])} 桥洞 {layout['bridges']}）{extra}")
            return layout
        if stop_flag.get("stop"):
            log("等待场地布局期间收到停机信号 → 不规划、不出发")
            return None
        try:
            alive_now = bool(alive())
        except Exception:                          # noqa: BLE001  判活本身出错不误杀
            alive_now = True
        if not alive_now:
            log("等待场地布局期间 ROS 上下文失效 → 不规划、不出发")
            return None
        t = now()
        if t >= deadline:
            log(f"⚠️ 等待无人机 field_layout 超时（{wait_s:g}s）→ 不规划、不出发")
            return None
        if t - last_note >= 5.0:
            log(f"等待无人机下发场地布局…（已 {t - t0:.0f}/{wait_s:.0f}s）")
            last_note = t
        sleep(0.1)


# --------------------------------------------------------------------------- #
#  任务装配（可选任务缺失只告警，不阻断）
# --------------------------------------------------------------------------- #
def build_tasks(bridge: Any, bb: Blackboard, bus: EventBus, cfg: Config,
                *, log: Callable[[str], None] = LOG,
                skip: Iterable[str] | None = None) -> tuple[TaskGroup, list[str]]:
    group = TaskGroup(bus, bb, logger=log)
    skipped: list[str] = []
    skip_set = {str(x) for x in (skip or ())}

    def add(module: str, cls_name: str, label: str) -> None:
        if module in skip_set:                   # 布局模式：DroneTask 已在装配前单独启动
            return
        try:
            # ★ 不能用 __import__（函数内没有包上下文，会报 "'__name__' not in globals"）
            mod = importlib.import_module(f".tasks.{module}", package=__package__)
            cls = getattr(mod, cls_name)
        except Exception as exc:
            skipped.append(f"{label}: {type(exc).__name__}: {exc}")
            return
        # Safety 需要 group 才能看别的心跳；其余任务不需要
        kwargs = {"log": log}
        if cls_name == "SafetyTask":
            kwargs["group"] = group
        group.add(cls(bridge, bb, bus, cfg, **kwargs))

    # 核心四线程（P1 已实现）
    add("localization", "LocalizationTask", "定位")
    add("safety", "SafetyTask", "安全")
    add("tracker", "TrackerTask", "循迹")
    add("mission", "MissionTask", "任务调度")
    # P2/P3 线程（实现即自动接入）
    add("aim", "AimTask", "云台打击")
    add("fix", "FixTask", "误差修正")
    add("drone", "DroneTask", "无人机")
    add("telemetry", "TelemetryTask", "遥测")
    return group, skipped


def start_banner(cfg: Config, group: TaskGroup, *, plan: Any = None,
                 layout_mode: bool = False, step_mode: bool = False) -> None:
    tokens = cfg.get("route.tokens") or []
    plan_on = bool(cfg.get("planner.auto_plan_enabled", False))
    if step_mode:
        route_source = "步进循环（无人机布局 → 段规划）"
        route_line = ("运行期等布局（version 大者胜）→ 每段从当前格规划到 target_cell → "
                      "MissionTask 执行 → 段末回报；规划失败/无目标 ⇒ WAIT_INFO"
                      f"（超时 {cfg.get('step.wait_timeout_s', 0)}s，0=无限）")
    elif plan is not None:
        route_source = "无人机布局自规划"
        route_line = (f"{len(plan.tokens)} token（引擎 {plan.engine}，{plan.mode}，"
                      f"耗时 {plan.elapsed_ms:.1f} ms，{plan.route_len_m:.2f} m；"
                      f"已写入 route.tokens 交任务执行）")
    elif plan_on:
        route_source = "自动规划"
        route_line = (f"自动规划（map_path={cfg.get('planner.map_path')}，"
                      f"{'允许倒车' if cfg.get('planner.allow_reverse') else '无倒车'}，"
                      f"预算 {cfg.get('planner.max_seconds')}s；实际 token 见任务开始横幅）")
    else:
        route_source = "手动配置"
        route_line = (f"{len(tokens)} 点（穿洞 {cfg.get('route.cross_lower_default')} / "
                      f"{cfg.get('route.cross_upper_default')}）")
    map_line = ("无人机 field_layout（车端不留地图文件）" if layout_mode
                else str(cfg.get("planner.map_path")))
    task_line = " ".join(f"{t.name}@{t.rate_hz:g}Hz" for t in group._tasks)  # noqa: SLF001
    if layout_mode:
        task_line += " + Drone(布局监听，先于任务组启动)"
    lines = ["dzbrain 启动 · 多线程任务架构"] + kv_lines({
        "配置文件": cfg.source,
        "地图": map_line,
        "路线来源": route_source,
        "路线": route_line,
        "巡航速度": f"{cfg.get('pursuit.cruise_speed_mps')} m/s",
        "自动出发": f"{cfg.get('mission.auto_start')}（延时 {cfg.get('mission.auto_start_delay_s')}s）",
        "越界余量": f"{cfg.get('pursuit.map_safety_margin_m')} m",
        "开火策略": f"{cfg.get('aim.fire_policy')}（提前量 {cfg.get('aim.lead_time_s')}s）",
        "任务线程": task_line,
    }, indent=2)
    banner(lines, log=LOG)


def _tele_field(tele: Any, name: str, default: Any = "—") -> Any:
    """遥测字段容错读取：Telemetry 对象或 dict 都能取（未实现 TelemetryTask 时可能是 dict）。"""
    if tele is None:
        return default
    if isinstance(tele, dict):
        return tele.get(name, default)
    return getattr(tele, name, default)


def finish_report(bb: Blackboard, group: TaskGroup, bus: EventBus,
                  *, elapsed: float) -> None:
    tele: Any = bb.get("telemetry")
    lines = ["dzbrain 退出报告"] + kv_lines({
        "运行时长": f"{elapsed:.1f} s",
        "阶段/位置": f"{_tele_field(tele, 'phase')} @ {_tele_field(tele, 'block')}",
        "发数/命中": f"{_tele_field(tele, 'shots')}/{_tele_field(tele, 'hits')}",
        "安全": f"ok={_tele_field(tele, 'safety_ok')}",
        "事件总数": bus.published,
    }, indent=2)
    banner(lines, log=LOG)
    for row in group.describe():
        LOG(f"  [{row['state']:8}] {row['name']:14} ticks={row['ticks']:<7} "
            f"errors={row['errors']} overruns={row['overruns']} "
            f"max_tick={row['max_tick_ms']}ms")


# --------------------------------------------------------------------------- #
#  bench：无 ROS 联调（真实任务线程 + FakeBridge）
# --------------------------------------------------------------------------- #
def run_bench(cfg: Config, *, seconds: float, drive: bool = False,
              speed_factor: float = 1.0, log: Callable[[str], None] = LOG) -> int:
    """在 FakeBridge 上跑真实任务线程 —— 不碰车、不需要 ROS，用来验证线程协作。

    `--drive` 再挂一辆**虚拟小车**：把循迹线程发出的 (速度, 舵角) 用自行车模型积分，
    每 20 ms 回写 FakeBridge 的位姿 ⇒ 形成完整闭环（任务层 + 运动学）。

    ⚠️ `speed_factor` 只能用于**冒烟**：控制器是按真实时序整定的（50 Hz、20 ms 控制周期、
    0.15 m/s 巡航），把虚拟车速放大 N 倍等于让车用 N 倍速度跑同一套整定 ⇒ 必然超调跑飞
    （实测 3× 时横向偏差 3 m）。**要看真实行为就用默认 1.0**；
    要快的、可信的全程仿真请用 `tools/sim_pursuit.py`（它有完整的三层执行链建模）。
    """
    tests_dir = pathlib.Path(__file__).resolve().parents[3] / "src" / "dzbrain" / "tests"
    sys.path.insert(0, str(tests_dir))
    try:
        import fakes  # type: ignore
    except Exception as exc:
        log(f"bench 需要 tests/fakes.py：{exc}")
        return 2

    from .route.parser import parse_mission
    from .pursuit.path import Path

    if drive:
        # bench 专用覆盖：立即出发（否则要等 auto_start_delay）
        cfg.override("mission.auto_start", True).override("mission.auto_start_delay_s", 0.3)

    # ★ 自动规划：bench 的虚拟小车按 `route.tokens` 建 Path / 统计"走到第 N 点"，
    #   而 MissionTask 在 `planner.auto_plan_enabled=true` 时执行的是**规划结果** ——
    #   两者必须一致，否则统计口径与实际走的路线整段错位（"走到第 20/44 点"是假数字）。
    #   这里先把规划结果回写到 cfg.route.tokens；MissionTask 自己还会再规划一次（同一份配置
    #   ⇒ 同一条路线），所以这不是"绕过规划"，只是让 bench 的观测口径对齐。
    if bool(cfg.get("planner.auto_plan_enabled", False)):
        try:
            from .tasks.mission import plan_route_from_config
            out = plan_route_from_config(cfg, log=log)
            if out.ok:
                cfg.override("route.tokens", list(out.tokens))
                log(f"bench：自动规划路线 {len(out.tokens)} token / {out.elapsed_ms:.1f} ms "
                    f"引擎 {out.engine}"
                    f"（已同步给虚拟小车，保证统计口径一致）")
            else:
                log(f"bench：自动规划失败（{out.error}）→ 按 route.tokens（{len(cfg.get('route.tokens') or [])} 点）")
        except Exception as exc:                    # noqa: BLE001
            log(f"bench：自动规划异常（{type(exc).__name__}: {exc}）→ 用 route.tokens")

    bridge = fakes.FakeBridge()
    route = parse_mission(cfg.get("route.tokens") or [])
    reset_block, reset_yaw = "N10", geo.HEADING_X
    try:
        from .route.parser import parse_reset
        reset_block, reset_yaw = parse_reset(cfg.get("mission.reset") or "N10,y")
    except Exception:
        pass
    bridge.set_pose_at_block(reset_block, yaw=reset_yaw)

    bb, bus = Blackboard(), EventBus()
    group, skipped = build_tasks(bridge, bb, bus, cfg, log=log)
    for s in skipped:
        log(f"  (跳过未实现的任务) {s}")

    stop_flag: dict = {"stop": False, "signal": None}
    # ★ 与上车运行用同一套信号接管：bench 只接了 SIGINT，且处理器里直接 print
    #   —— 漏了 SIGTERM，也让"留痕"发生在信号上下文里（见 install_stop_signals 注释）。
    install_stop_signals(stop_flag, log)

    start_banner(cfg, group)
    group.start_all()
    t0 = time.time()

    # ---------------- 虚拟小车线程（仅 --drive） ----------------
    car = {"x": 0.0, "y": 0.0, "yaw": 0.0, "v": 0.0, "steer": 0.0, "steps": 0,
           "last_cmd_index": 0, "reached": 0, "max_xt": 0.0, "stats_relocks": 0}
    pose0 = bridge.pose()
    if pose0:
        car.update(x=pose0.x, y=pose0.y, yaw=pose0.yaw)
    path = Path(route)
    s_hint = {"s": 0.0}

    def _car_loop() -> None:
        dt_real = 0.02
        wheelbase = float(cfg.get("pursuit.wheelbase_m") or 0.157)
        release_m = float(cfg.get("pursuit.projection_release_m") or 0.30)
        while not stop_flag["stop"]:
            time.sleep(dt_real)
            # 取最新指令（FakeBridge 记录了全部指令；取最后一条非刹车也照用）
            if len(bridge.commands) > car["last_cmd_index"]:
                v, steer = bridge.commands[-1]
                car["last_cmd_index"] = len(bridge.commands)
            else:
                v, steer = 0.0, 0.0
            car["v"], car["steer"] = v, steer
            dt = dt_real * speed_factor
            car["yaw"] = geo.normalize_angle(
                car["yaw"] + (v / wheelbase) * math.tan(math.radians(steer)) * dt)
            car["x"] += v * math.cos(car["yaw"]) * dt
            car["y"] += v * math.sin(car["yaw"]) * dt
            car["steps"] += 1
            bridge.set_pose(car["x"], car["y"], car["yaw"])
            # 统计：离路线多远 / 到了几个点
            #  ★ 统计这条投影链**必须和 Tracker 一样能"跟丢自愈"**：`s_hint` 每拍跟随上一拍的
            #    s，窗口（±0.6 m）会把错误的 s 变成自洽不动点 —— 一旦锁死，`proj.dist` 会随
            #    车开远而无限增大、`reached` 永远停在锁死处。实测（bench_relock 之前）：
            #    同一条跑动里 Tracker 报"到点 31"，而这里报"走到第 18/44 点、最大横向偏差
            #    4.098 m" —— 车其实一直在路线上（到点判据要求"车贴近路径"），
            #    4.098 m 是**统计口径**的假值，不是车跑飞了。
            try:
                proj = path.project(car["x"], car["y"], s_hint=s_hint["s"] or None,
                                    allow_backward=True)
                if proj.dist > release_m:                     # 跟丢 → 丢 hint 全路线重投影
                    proj_relock = path.project(car["x"], car["y"])
                    if proj_relock.dist < proj.dist - 1e-6:
                        proj = proj_relock
                        car["stats_relocks"] = car.get("stats_relocks", 0) + 1
                s_hint["s"] = proj.s
                car["max_xt"] = max(car["max_xt"], proj.dist)
                idx = sum(1 for i in range(len(route))
                          if path.s_at_waypoint(i) <= proj.s + 1e-9)
                car["reached"] = max(car["reached"], idx)
            except Exception:
                pass

    car_thread = None
    if drive:
        car_thread = threading.Thread(target=_car_loop, name="virtual-car", daemon=True)
        car_thread.start()
        log(f"虚拟小车已启动（时间快进 {speed_factor:g}×，路线 {len(route)} 点）…")

    try:
        deadline = time.time() + seconds
        while time.time() < deadline and not stop_flag["stop"]:
            if drive and bb.get("mission_phase") == "FINISH":
                log("虚拟小车已到终点。")
                break
            time.sleep(0.1)
    finally:
        stop_flag["stop"] = True
        if stop_flag.get("signal"):
            log(f"收到信号 {stop_flag['signal']}：bench 收尾（信号处理器只置位，留痕在主循环外）")
        if car_thread:
            car_thread.join(timeout=1.0)
        stuck = group.stop_all(timeout=2.0)
        if stuck:
            log(f"⚠️ 未按时退出的任务: {stuck}")
        if drive:
            log(f"虚拟小车：走到第 {car['reached']}/{len(route)} 点，"
                f"位置 ({car['x']:.2f}, {car['y']:.2f})，最大横向偏差 {car['max_xt']:.3f} m，"
                f"积分 {car['steps']} 拍，统计跟丢自愈 {car['stats_relocks']} 次")
        finish_report(bb, group, bus, elapsed=time.time() - t0)
        faulty = group.faulty()
        if faulty:
            log(f"❌ 任务故障: {faulty}")
            return 1
    return 0


# --------------------------------------------------------------------------- #
#  静态规划入口（--plan-only）
# --------------------------------------------------------------------------- #
def run_plan_only(cfg: Config, *, map_path: str | None = None,
                  log: Callable[[str], None] = LOG) -> int:
    """`--plan-only`：只规划、打印 token 列表 + 耗时。

    ★ 与车上自动规划**共用同一段代码**（`tasks.mission.plan_route_from_config`）：
      不起线程、不建 Bridge、不发任何指令、不 import ROS —— 所以它可以在没有 ROS
      的机器上跑，也可以当"这条路线能不能算出来"的离线体检。
      规划失败返回 1（退出码非 0 才有人会去看日志）。
    ★ `map_path`：`--map <图>` 必须真的决定**规划用的地图**。
      只作用于门禁（`load_and_check`）而规划仍走 `planner.map_path` ⇒ 同一条命令里
      两套地图口径并存，`--plan-only --map 变体图` 会打印配置里的图。
    """
    from .tasks.mission import plan_route_from_config
    out = plan_route_from_config(cfg, map_path=map_path, log=log)
    if not out.ok:
        banner([f"静态规划失败（--plan-only）：{out.error}",
                f"地图: {out.map_path}",
                f"引擎: {out.engine}（planner.engine）",
                f"模式: {out.mode}（planner.allow_reverse={out.allow_reverse}）"], char="!", log=log)
        return 1
    banner([
        "静态规划结果（--plan-only：未起线程、未发指令、未连 ROS）",
        f"地图: {out.map_path}",
        f"引擎: {out.engine}（planner.engine）",
        f"门禁: {'✅通过' if out.gate_ok else '❌未过'} —— {out.gate_detail}",
        f"模式: {out.mode}（planner.allow_reverse={out.allow_reverse}）",
        f"token 数: {len(out.tokens)} / 折线 {out.route_len_m:.2f} m / 倒车段 {out.reverse_segments}",
        f"A* 代价: {out.cost:.2f}",
        # 参考点偏移感知的足迹诊断（现场卷尺实测 offset 下的挡板/障碍/靶板余量）
        (f"足迹: {'✅' if out.footprint_ok else '⚠️'} {out.footprint}"
         if out.footprint else "足迹: 未诊断"),
    ], log=log)
    log(f"token 列表（{len(out.tokens)}）：" + " ".join(out.tokens))
    log(f"规划耗时：{out.elapsed_ms:.1f} ms")
    return 0


# --------------------------------------------------------------------------- #
#  正常上车运行
# --------------------------------------------------------------------------- #
def _live_loop(stop_flag: dict, *, bb: Blackboard, alive: Callable[[], bool] = ros_context_alive,
               log: Callable[[str], None] = LOG, poll_s: float = 0.2,
               finish_wait_s: float = 2.0) -> str:
    """上车运行的主循环：返回退出原因 `stop` / `context_dead` / `finish`。

    ★ 三条退出路径缺一不可：
      1. `stop_flag` —— 显式信号（SIGINT/SIGTERM）置位；
      2. **上下文失效** —— `alive()` 为假时 ≤1 拍退出（poll_s=0.2 s 一拍）。
         这是本次 P0：旧循环只看 stop_flag/FINISH，SIGTERM 把 rclpy 上下文
         在 C++ 层关掉后它**永远不退出**，8 个线程继续按 50/20/10 Hz 打 RCLError
         （实测约 100 行/s、持续到被 SIGKILL）。判活必须每拍做，因为上下文
         可能被**任何来源**销毁（不只是我们的 shutdown 路径）。
      3. FINISH —— 正常跑完。
    """
    while not stop_flag["stop"]:
        if not alive():
            log("‼️ ROS 上下文已失效（rclpy.ok()=False）→ 立即停止主循环、准备收尾"
                "（不再空转刷 RCLError）")
            return "context_dead"
        time.sleep(poll_s)
        if bb.get("mission_phase") == "FINISH":
            log(f"任务已到终点，等待 {finish_wait_s:g}s 后退出…")
            time.sleep(finish_wait_s)
            return "finish"
    sig = stop_flag.get("signal")
    log(f"收到信号 {sig}：开始安全收尾（刹车 → 停线程 → 退出报告）" if sig
        else "收到停机请求：开始安全收尾（刹车 → 停线程 → 退出报告）")
    return "stop"


def run_live(cfg: Config, *, log: Callable[[str], None] = LOG,
             alive: Callable[[], bool] = ros_context_alive,
             map_path: str | None = None) -> int:
    from .io.ros_io import RosBridge          # 只在真跑时导入（无 ROS 的机器 import 会失败但惰性）
    from .io import contract as ct

    bb, bus = Blackboard(), EventBus()
    bridge = RosBridge("dzbrain", log=log)

    stop_flag: dict = {"stop": False, "signal": None}
    installed = install_stop_signals(stop_flag, log)
    log(f"信号接管：{'/'.join(installed) or '无'} → 只置停机标志；"
        f"ROS 上下文判活 = rclpy.ok()（默认信号处理已在 RosBridge 里关闭）")

    # ★ 启动自检（2026-10-18 pose_stale 事故防线①）：
    #   重复实例 / ROS 域号 / 订阅清单 / 关键话题 pub-sub 计数，全部进启动横幅。
    dups = _duplicate_dzbrain_instances()
    if dups:
        log("⛔ 检测到重复的 dzbrain 实例（同一 ROS 图里同名节点会互抢话题/QoS 匹配，"
            "表现为『收不到里程计/指令不到底盘』）：")
        for line in dups:
            log("    " + line)
        log("⛔ 拒绝启动（退出码 5）：先 pkill -f dzbrain.main（或 start_all.sh stop），"
            "确认只剩一个实例再起")
        bridge.shutdown(brake=True)
        return 5
    if not _startup_self_check(bridge, log=log):
        bridge.shutdown(brake=True)
        return 5

    layout_mode = _layout_mode_enabled(cfg, map_path)

    # ★ 启动契约（缺口 A）：把"谁必须在对的图上"写成表格并用端点计数对账。
    #   失败判红 = 明确读到 0 或本节点没建那条订阅；**读不到（未知）不阻断**。
    #   在等布局/建任务组之前 ⇒ 端点缺失时不会先耗掉 30s 布局等待。
    contract_ok, _contract_rows = startup_contract(bridge, "live", log=log)
    if not contract_ok:
        log("⛔ 启动契约未通过（退出码 6）：上表标 ❌ 的必需端点缺失/未匹配 ⇒ 先查图，"
            "不规划、不出发（环境自检 exit 5；本项 exit 6，两者分开定位）")
        bridge.shutdown(brake=True)
        return 6

    # ---------------- 车端不留地图文件：先等布局，再构造任务组 ----------------
    # ★ 顺序是**功能前提**：MissionTask 的路线在构造/启动时就要定下来，所以布局规划
    #   必须发生在 build_tasks 之前；等布局期间只起一个独立 DroneTask（不进任务组，
    #   否则 start_all 会对已启动的线程二次 start）。
    layout_task = None
    layout_plan = None
    step_mode = False
    first_layout: dict | None = None
    if layout_mode:
        log("车端无可用的地图文件（未传 --map 或 --map 指向的文件不存在）→ 布局模式："
            "先起无人机线程等 field_layout；v1 布局=旧行为（规划一次），"
            "v2 布局（带 known）=T27 步进循环（运行期逐段规划/执行/回报）")
        if not bool(cfg.get("drone.enabled", True)):
            log("⛔ 布局模式需要无人机链路，但 drone.enabled=False → 永远收不到布局，"
                "不规划、不出发")
            bridge.shutdown(brake=True)
            return 3
        try:
            layout_task = _start_layout_listener(bridge, bb, bus, cfg, log=log)
        except Exception as exc:                     # noqa: BLE001  起不来就别硬等
            log(f"⛔ 无人机布局监听启动失败（{type(exc).__name__}: {exc}）→ 不规划、不出发")
            bridge.shutdown(brake=True)
            return 3
        bridge.spin_async(threads=4)
        first_layout = _wait_for_field_layout(bb, cfg, stop_flag=stop_flag,
                                              alive=alive, log=log)
        if first_layout is None:
            layout_task.stop(timeout=2.0)
            bridge.shutdown(brake=True)
            log("⛔ 未收到无人机场地布局 → 不规划、不出发"
                "（绝不用地图文件/旧 route.tokens 兜底）")
            return 3
        step_mode = _step_mode_enabled(first_layout)
        if step_mode:
            # ★T27：不再"规划一次再交给 MissionTask"，而是让任务组立即起、由
            #   StepLoop 在运行期逐段规划/执行/回报。MissionTask 的 auto_start 必须关掉
            #   —— 段由步进器显式 request_segment() 启动（否则会自己载入 route.tokens）。
            cfg.override("mission.auto_start", False)
            log(f"✅ 收到布局 v2（known {len(first_layout.get('known') or [])} 格）→ "
                f"进入运行期步进循环：段由无人机 target_cell 驱动；"
                f"MissionTask 只执行步进器下发的段")
            # ★步进赛时契约（race 级）：drone_in（布局来源）必须有对端发布方 + 本节点订阅
            race_ok, race_rows = startup_contract(bridge, "race-step")
            if race_ok:
                log("步进赛时契约：通过（drone_in 本节点订阅 + 对端发布方都在）")
            else:
                log(_format_contract_table(race_rows, mode="race-step", ok=False))
                log("⛔ 步进赛时契约未通过（退出码 6）：缺 drone_in 等步进必需端点 ⇒ "
                    "不规划、不出发")
                layout_task.stop(timeout=2.0)
                bridge.shutdown(brake=True)
                return 6
        else:
            layout_plan = plan_route_from_field_layout(cfg, first_layout, log=log)
            if not layout_plan.ok:
                layout_task.stop(timeout=2.0)
                bridge.shutdown(brake=True)
                log(f"⛔ 布局规划失败（{layout_plan.error}）→ 不出发"
                    f"（绝不回退地图文件/旧 route.tokens）")
                return 4
            # ★ 把规划产物写回 route.tokens，并关掉任务侧的"再规划"：MissionTask 走
            #   既有手动路线分支执行这条**由布局算出**的路线（不会去碰 planner.map_path）。
            cfg.override("route.tokens", list(layout_plan.tokens))
            cfg.override("planner.auto_plan_enabled", False)
            log(f"✅ 布局自规划成功：{len(layout_plan.tokens)} token / "
                f"{layout_plan.route_len_m:.2f} m / 引擎 {layout_plan.engine} / "
                f"{layout_plan.elapsed_ms:.1f} ms → 已写入 route.tokens 交任务执行")

    group, skipped = build_tasks(bridge, bb, bus, cfg, log=log,
                                 skip={"drone"} if layout_task is not None else None)
    for s in skipped:
        log(f"  (跳过未实现的任务) {s}")

    step_loop: StepLoop | None = None
    if step_mode:
        mission_task = _find_task(group, "MissionTask")
        if mission_task is None:
            if layout_task is not None:
                layout_task.stop(timeout=2.0)
            bridge.shutdown(brake=True)
            log("⛔ 步进循环需要 MissionTask，但任务组里没有（模块未实现/导入失败）→ 不出发")
            return 3
        step_loop = StepLoop(cfg, bb, bridge, mission_task, bus=bus, log=log)

    start_banner(cfg, group, plan=layout_plan, layout_mode=layout_mode,
                 step_mode=step_mode)
    log("ROS 契约：" + " | ".join(f"{k}={ct.topic(k).name}" for k in
                                 ("pursuit_angle", "scan_request", "aim_fire_gate",
                                  "aim_fire_ready", "drone_in")))
    if layout_task is None:
        bridge.spin_async(threads=4)
    # ★ 防线② 位姿就绪门（两种模式都过）：宁可当场退出 5，也不带着种子位姿空跑。
    if not _wait_for_pose_ready(bridge, cfg, stop_flag=stop_flag, alive=alive, log=log):
        if layout_task is not None:
            layout_task.stop(timeout=2.0)
        bridge.shutdown(brake=True)
        return 5
    group.start_all()
    t0 = time.time()
    rc = 0

    # 安全兜底：任务组出现 FAULT 时，Safety 会刹；这里只做退出条件判断
    try:
        if step_loop is not None:
            step_loop.start(first_layout)
            why = _run_step_loop(step_loop, stop_flag=stop_flag, alive=alive, log=log)
            log(f"退出原因 = {why}（步进循环；运行 {time.time() - t0:.1f}s；"
                f"信号={stop_flag.get('signal')}；rclpy.ok()={alive()}）")
            rc = (1 if why in ("context_dead", "safe_stop")
                  else 5 if why == "ros_lost"
                  else 4 if why == "timeout" else 0)
        else:
            why = _live_loop(stop_flag, bb=bb, alive=alive, log=log)
            # ★ teardown 留痕（实测指出收栈时**一条都没有**，事后无法判断是正常收尾还是被打死）
            log(f"退出原因 = {why}（运行 {time.time() - t0:.1f}s；"
                f"信号={stop_flag.get('signal')}；rclpy.ok()={alive()}）")
            if why == "context_dead":
                rc = 1
    finally:
        stuck = group.stop_all(timeout=2.0)
        if stuck:
            log(f"⚠️ 未按时退出的任务: {stuck}")
        if layout_task is not None and not layout_task.stop(timeout=2.0):
            log("⚠️ 无人机布局监听任务未按时退出")
        shutdown_rc = bridge.shutdown(brake=True)      # ★ 退出必刹车（上下文已死则跳过）
        log(f"收尾：ros 上下文{'已失效' if shutdown_rc else '正常'}，"
            f"刹车已尝试={'否' if shutdown_rc else '是'}，任务线程已停")
        finish_report(bb, group, bus, elapsed=time.time() - t0)
        if step_loop is not None:
            st = step_loop.status()
            log(f"  [步进循环] 阶段={st['phase']} 布局 v{st['seen_version']} "
                f"段下发={st['segments_started']} 段完成={st['segments_done']} "
                f"重规划={st['replans']} known={st['known_count']} 格")
        if layout_task is not None:
            h = layout_task.health()
            log(f"  [无人机/布局监听] ticks={h.ticks} errors={h.errors} "
                f"布局份数={getattr(layout_task, 'layout_updates', '?')} "
                f"拒收={getattr(layout_task, 'layout_rejects', '?')}")
    return rc


# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="dzbrain", description="dzrobot 新决策层（多线程任务架构）")
    ap.add_argument("--config", default=None, help="robot.yaml 路径（默认 ~/dzrobot/config/robot.yaml）")
    ap.add_argument("--map", dest="map_path", default=None, help="map.yaml 路径")
    ap.add_argument("--dry-run", action="store_true", help="只做启动门禁，不起线程")
    ap.add_argument("--plan-only", dest="plan_only", action="store_true",
                    help="只按 planner.* 静态规划一次，打印 token + 耗时（不起线程/不发指令/无 ROS）")
    ap.add_argument("--bench", action="store_true", help="FakeBridge 联调（无 ROS/无车）")
    ap.add_argument("--drive", action="store_true",
                    help="bench 时再挂虚拟小车（自行车模型闭环，可跑完整条路线）")
    ap.add_argument("--speed-factor", type=float, default=1.0,
                    help="虚拟小车时间快进倍数（默认 1.0=忠实；>1 仅作冒烟，会超调）")
    ap.add_argument("--seconds", type=float, default=5.0, help="bench 运行时长")
    ap.add_argument("--force", action="store_true", help="门禁不通过也启动（现场应急）")
    args = ap.parse_args(argv)

    cfg, _map_cfg, errors = load_and_check(
        args.config, args.map_path, force=args.force, announce=not args.plan_only,
        # ★ 车端不留地图文件：地图缺失时由无人机 field_layout 顶替（只在
        #   planner.auto_plan_enabled=true 时生效；见 load_and_check 的 docstring）。
        #   注意 start_all.sh 仍按历史习惯传 `--map $WS/config/map.yaml` —— 车端没有
        #   该文件时就属于这个分支：**允许启动，等布局**（run_live 会判布局模式）。
        allow_missing_map=True)

    # ★ --map 透传（2026-10-11 修复）：planner.map_path 的相对解析以 workspace_root() 为基准；
    #   install 拷贝树里 workspace_root() = install/dzbrain/lib → config/map.yaml 永远找不到
    #   → 自动规划静默失败、回退手写路线（车端 verify/bench 走 src 导入，看不出此问题，
    #   真起栈才暴露）。start_all.sh 起栈时传 --map，这里用
    #   绝对路径钉死 planner.map_path，与 resolve_map_path 的「绝对路径直接采用」分支闭环。
    #   --plan-only --map <图> 必须用同一张图，否则会静默用 config 里的图。
    if args.map_path:
        cfg.override("planner.map_path", str(pathlib.Path(args.map_path).resolve()))

    # ★ 静态规划：必须在门禁 `return 2`、以及 bench/live 之前分流 —— 这是**功能前提**，不是顺序偏好。
    #   `planner.auto_plan_enabled=true` 时车上执行的是**规划结果**；而门禁校验的恰恰是
    #   `route.tokens`（`load_and_check` → `validate_route(parse_mission(route.tokens))`）。
    #   旧手写路线一旦过时，门禁必然报 error，于是 `--plan-only` 直接 `return 2` **什么都算不出来**
    #   —— 而它正是"手写路线不对了、让机器自己算一条"唯一的离线入口，这一条比门禁更该先跑。
    #   故此处显式豁免：仍**打印**门禁错误（不隐瞒），但继续规划。
    #   ⚠️ 早期版本把这个分支放在 `if errors: return 2` **之后**，那句 `if errors:` 成了永不执行的
    #   死代码（`--plan-only` 在坏路线下恒 exit 2，实测 route.tokens 改成 2 个点时如此）——
    #   已调整顺序，`test_autoplan.py` 7.13/7.14 用"坏 route.tokens + --plan-only"钉住它。
    #   `--dry-run` / `--bench` / live 的门禁语义**未变**（仍是 errors → return 2）。
    #
    #   ★ 这段报文**只能打印一遍**（`load_and_check` + 这里会打印两遍），且必须写清
    #     error 是**针对手写 `route.tokens`** 的 —— 否则读者很容易以为"规划结果也有
    #     这些错"。⇒ `load_and_check(announce=False)` 不打印，这里只打印**一次**，
    #     并显式标注来源与"与规划结果无关"。
    if args.plan_only:
        if errors:
            banner([
                "启动门禁未通过（--plan-only 仍继续规划：执行路线由规划器产出）",
                f"以下 {len(errors)} 条错误**只针对 route.tokens（手写回退路线）**，"
                f"与下面的规划结果无关；",
                f"规划结果按 planner.map_path={cfg.get('planner.map_path')} 独立产出。",
            ] + errors, char="!", log=LOG)
        return run_plan_only(cfg, map_path=args.map_path)

    if errors and not args.force:
        return 2

    if args.dry_run:
        LOG("")
        LOG(f"启动门禁通过：配置 {cfg.source}")
        if _layout_mode_enabled(cfg, args.map_path):
            LOG("提示：车端布局模式（无可用的地图文件：未传 --map 或 --map 指向的文件不存在）"
                "→ 实际运行会先等无人机 field_layout；未收到布局就不起任务组、不发车"
                "（--dry-run 不等布局）")
        LOG(f"  {cfg.summary()}")
        return 0

    if args.bench:
        return run_bench(cfg, seconds=args.seconds, drive=args.drive, speed_factor=args.speed_factor)
    return run_live(cfg, map_path=args.map_path)


if __name__ == "__main__":
    raise SystemExit(main())
