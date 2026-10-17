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
from typing import Any, Callable

from .core import geometry as geo
from .core.blackboard import Blackboard
from .core.config import Config
from .core.events import EventBus
from .core.logging import banner, kv_lines
from .core.task import TaskGroup

LOG = print

#: 显式接管的退出信号。**SIGTERM 必须自己接**（t8 / t6 取证报告 §2）：
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
                   *, force: bool = False, announce: bool = True) -> tuple[Config, dict, list[str]]:
    """加载配置 + 路线校验。返回 (cfg, map_cfg, errors)。

    ★ `announce=False`（卡 D finding D3）：调用方自己会把错误**带来源标注**打印一次。
      否则 `--plan-only` 下门禁报文会打印两遍（`load_and_check` 一遍 + 调用方一遍），
      而且读者分不清那些 error 是针对手写 `route.tokens` 还是针对规划结果。
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
#  任务装配（可选任务缺失只告警，不阻断）
# --------------------------------------------------------------------------- #
def build_tasks(bridge: Any, bb: Blackboard, bus: EventBus, cfg: Config,
                *, log: Callable[[str], None] = LOG) -> tuple[TaskGroup, list[str]]:
    group = TaskGroup(bus, bb, logger=log)
    skipped: list[str] = []

    def add(module: str, cls_name: str, label: str) -> None:
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


def start_banner(cfg: Config, group: TaskGroup) -> None:
    tokens = cfg.get("route.tokens") or []
    plan_on = bool(cfg.get("planner.auto_plan_enabled", False))
    if plan_on:
        route_line = (f"自动规划（map_path={cfg.get('planner.map_path')}，"
                      f"{'允许倒车' if cfg.get('planner.allow_reverse') else '无倒车'}，"
                      f"预算 {cfg.get('planner.max_seconds')}s；实际 token 见任务开始横幅）")
    else:
        route_line = (f"{len(tokens)} 点（穿洞 {cfg.get('route.cross_lower_default')} / "
                      f"{cfg.get('route.cross_upper_default')}）")
    lines = ["dzbrain 启动 · 多线程任务架构"] + kv_lines({
        "配置文件": cfg.source,
        "路线来源": "自动规划" if plan_on else "手动配置",
        "路线": route_line,
        "巡航速度": f"{cfg.get('pursuit.cruise_speed_mps')} m/s",
        "自动出发": f"{cfg.get('mission.auto_start')}（延时 {cfg.get('mission.auto_start_delay_s')}s）",
        "越界余量": f"{cfg.get('pursuit.map_safety_margin_m')} m",
        "开火策略": f"{cfg.get('aim.fire_policy')}（提前量 {cfg.get('aim.lead_time_s')}s）",
        "任务线程": " ".join(f"{t.name}@{t.rate_hz:g}Hz" for t in group._tasks),  # noqa: SLF001
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

    # ★ 自动规划（任务卡 B）：bench 的虚拟小车按 `route.tokens` 建 Path / 统计"走到第 N 点"，
    #   而 MissionTask 在 `planner.auto_plan_enabled=true` 时执行的是**规划结果** ——
    #   两者必须一致，否则统计口径与实际走的路线整段错位（"走到第 20/34 点"是假数字）。
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
    # ★ 与上车运行用同一套信号接管（t8）：bench 只接了 SIGINT，且处理器里直接 print
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
            #    同一条跑动里 Tracker 报"到点 31"，而这里报"走到第 18/34 点、最大横向偏差
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
    ★ `map_path`（卡 D finding D2）：`--map <图>` 必须真的决定**规划用的地图**。
      此前只作用于门禁（`load_and_check`），规划仍走 `planner.map_path` ⇒ 同一条命令里
      两套地图口径并存，`--plan-only --map 变体图` 打印的还是配置里的图。
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
        # ★ 卡 t11：参考点偏移感知的足迹诊断（现场卷尺实测 offset 下的挡板/障碍/靶板余量）
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

    ★ 三条退出路径缺一不可（t8）：
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
             alive: Callable[[], bool] = ros_context_alive) -> int:
    from .io.ros_io import RosBridge          # 只在真跑时导入（无 ROS 的机器 import 会失败但惰性）
    from .io import contract as ct

    bb, bus = Blackboard(), EventBus()
    bridge = RosBridge("dzbrain", log=log)
    group, skipped = build_tasks(bridge, bb, bus, cfg, log=log)
    for s in skipped:
        log(f"  (跳过未实现的任务) {s}")

    stop_flag: dict = {"stop": False, "signal": None}
    installed = install_stop_signals(stop_flag, log)
    log(f"信号接管：{'/'.join(installed) or '无'} → 只置停机标志；"
        f"ROS 上下文判活 = rclpy.ok()（默认信号处理已在 RosBridge 里关闭）")

    start_banner(cfg, group)
    log("ROS 契约：" + " | ".join(f"{k}={ct.topic(k).name}" for k in
                                 ("pursuit_angle", "scan_request", "aim_fire_gate",
                                  "aim_fire_ready", "drone_in")))
    bridge.spin_async(threads=4)
    group.start_all()
    t0 = time.time()
    rc = 0

    # 安全兜底：任务组出现 FAULT 时，Safety 会刹；这里只做退出条件判断
    try:
        why = _live_loop(stop_flag, bb=bb, alive=alive, log=log)
        # ★ teardown 留痕（t6 指出收栈时**一条都没有**，事后无法判断是正常收尾还是被打死）
        log(f"退出原因 = {why}（运行 {time.time() - t0:.1f}s；"
            f"信号={stop_flag.get('signal')}；rclpy.ok()={alive()}）")
        if why == "context_dead":
            rc = 1
    finally:
        stuck = group.stop_all(timeout=2.0)
        if stuck:
            log(f"⚠️ 未按时退出的任务: {stuck}")
        shutdown_rc = bridge.shutdown(brake=True)      # ★ 退出必刹车（上下文已死则跳过）
        log(f"收尾：ros 上下文{'已失效' if shutdown_rc else '正常'}，"
            f"刹车已尝试={'否' if shutdown_rc else '是'}，任务线程已停")
        finish_report(bb, group, bus, elapsed=time.time() - t0)
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

    cfg, _map_cfg, errors = load_and_check(args.config, args.map_path, force=args.force,
                                           announce=not args.plan_only)

    # ★ --map 透传（2026-10-11 修复）：planner.map_path 的相对解析以 workspace_root() 为基准；
    #   install 拷贝树里 workspace_root() = install/dzbrain/lib → config/map.yaml 永远找不到
    #   → 自动规划静默失败、回退手写路线（车端 verify/bench 走 src 导入，看不出此问题，
    #   真起栈才暴露 —— 卡D验收第2步前查实）。start_all.sh 起栈时传 --map，这里用
    #   绝对路径钉死 planner.map_path，与 resolve_map_path 的「绝对路径直接采用」分支闭环。
    #   同时修复卡D finding D2：--plan-only --map <图> 此前静默用 config 里的图。
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
    #   ★ 卡 D finding D3：这段报文此前会**打印两遍**（`load_and_check` 一遍 + 这里一遍），
    #     而且没写清那些 error 是**针对手写 `route.tokens`** 的 —— 读者很容易以为
    #     "规划结果也有这些错"。现在改为：`load_and_check(announce=False)` 不打印，
    #     这里只打印**一次**，并显式标注来源与"与规划结果无关"。
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
        LOG(f"  {cfg.summary()}")
        return 0

    if args.bench:
        return run_bench(cfg, seconds=args.seconds, drive=args.drive, speed_factor=args.speed_factor)
    return run_live(cfg)


if __name__ == "__main__":
    raise SystemExit(main())
