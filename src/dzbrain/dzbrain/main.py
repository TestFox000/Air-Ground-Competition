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
from .core.config import Config, ConfigError
from .core.events import (EV_MISSION_FINISHED, EV_SAFETY_STOP, EV_TASK_FAULT, EventBus)
from .core.logging import banner, kv_lines
from .core.state import Telemetry
from .core.task import TaskGroup, TaskState, TaskThread

LOG = print


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
                   *, force: bool = False) -> tuple[Config, dict, list[str]]:
    """加载配置 + 路线校验。返回 (cfg, map_cfg, errors)。"""
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

    if errors and not force:
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
    lines = ["dzbrain 启动 · 多线程任务架构"] + kv_lines({
        "配置文件": cfg.source,
        "路线": f"{len(tokens)} 点（穿洞 {cfg.get('route.cross_lower_default')} / "
                f"{cfg.get('route.cross_upper_default')}）",
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

    stop_flag = {"stop": False}

    def _sigint(_signum, _frame):
        stop_flag["stop"] = True
    signal.signal(signal.SIGINT, _sigint)

    start_banner(cfg, group)
    group.start_all()
    t0 = time.time()

    # ---------------- 虚拟小车线程（仅 --drive） ----------------
    car = {"x": 0.0, "y": 0.0, "yaw": 0.0, "v": 0.0, "steer": 0.0, "steps": 0,
           "last_cmd_index": 0, "reached": 0, "max_xt": 0.0}
    pose0 = bridge.pose()
    if pose0:
        car.update(x=pose0.x, y=pose0.y, yaw=pose0.yaw)
    path = Path(route)
    s_hint = {"s": 0.0}

    def _car_loop() -> None:
        dt_real = 0.02
        wheelbase = float(cfg.get("pursuit.wheelbase_m") or 0.157)
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
            try:
                proj = path.project(car["x"], car["y"], s_hint=s_hint["s"] or None,
                                    allow_backward=True)
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
        if car_thread:
            car_thread.join(timeout=1.0)
        stuck = group.stop_all(timeout=2.0)
        if stuck:
            log(f"⚠️ 未按时退出的任务: {stuck}")
        if drive:
            log(f"虚拟小车：走到第 {car['reached']}/{len(route)} 点，"
                f"位置 ({car['x']:.2f}, {car['y']:.2f})，最大横向偏差 {car['max_xt']:.3f} m，"
                f"积分 {car['steps']} 拍")
        finish_report(bb, group, bus, elapsed=time.time() - t0)
        faulty = group.faulty()
        if faulty:
            log(f"❌ 任务故障: {faulty}")
            return 1
    return 0


# --------------------------------------------------------------------------- #
#  正常上车运行
# --------------------------------------------------------------------------- #
def run_live(cfg: Config, *, log: Callable[[str], None] = LOG) -> int:
    from .io.ros_io import RosBridge          # 只在真跑时导入（无 ROS 的机器 import 会失败但惰性）
    from .io import contract as ct

    bb, bus = Blackboard(), EventBus()
    bridge = RosBridge("dzbrain", log=log)
    group, skipped = build_tasks(bridge, bb, bus, cfg, log=log)
    for s in skipped:
        log(f"  (跳过未实现的任务) {s}")

    stop_flag = {"stop": False}

    def _sigint(_signum, _frame):
        stop_flag["stop"] = True
        log("收到 Ctrl-C，准备安全退出…")
    signal.signal(signal.SIGINT, _sigint)

    start_banner(cfg, group)
    log("ROS 契约：" + " | ".join(f"{k}={ct.topic(k).name}" for k in
                                 ("pursuit_angle", "scan_request", "aim_fire_gate",
                                  "aim_fire_ready", "drone_in")))
    bridge.spin_async(threads=4)
    group.start_all()
    t0 = time.time()

    # 安全兜底：任务组出现 FAULT 时，Safety 会刹；这里只做退出条件判断
    try:
        while not stop_flag["stop"]:
            time.sleep(0.2)
            if bb.get("mission_phase") == "FINISH":
                log("任务已到终点，等待 2s 后退出…")
                time.sleep(2.0)
                break
    finally:
        stuck = group.stop_all(timeout=2.0)
        if stuck:
            log(f"⚠️ 未按时退出的任务: {stuck}")
        bridge.shutdown(brake=True)           # ★ 退出必刹车
        finish_report(bb, group, bus, elapsed=time.time() - t0)
    return 0


# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="dzbrain", description="dzrobot 新决策层（多线程任务架构）")
    ap.add_argument("--config", default=None, help="robot.yaml 路径（默认 ~/dzrobot/config/robot.yaml）")
    ap.add_argument("--map", dest="map_path", default=None, help="map.yaml 路径")
    ap.add_argument("--dry-run", action="store_true", help="只做启动门禁，不起线程")
    ap.add_argument("--bench", action="store_true", help="FakeBridge 联调（无 ROS/无车）")
    ap.add_argument("--drive", action="store_true",
                    help="bench 时再挂虚拟小车（自行车模型闭环，可跑完整条路线）")
    ap.add_argument("--speed-factor", type=float, default=1.0,
                    help="虚拟小车时间快进倍数（默认 1.0=忠实；>1 仅作冒烟，会超调）")
    ap.add_argument("--seconds", type=float, default=5.0, help="bench 运行时长")
    ap.add_argument("--force", action="store_true", help="门禁不通过也启动（现场应急）")
    args = ap.parse_args(argv)

    cfg, _map_cfg, errors = load_and_check(args.config, args.map_path, force=args.force)
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
