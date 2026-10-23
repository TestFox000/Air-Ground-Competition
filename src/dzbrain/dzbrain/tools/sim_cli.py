# -*- coding: utf-8 -*-
"""仿真器命令行入口（`docs/P0接口约定.md` §5）。

用法（在工作区根）::

    python tools\\sim_pursuit.py --config config\\robot.yaml ^
        --route src\\dzbrain\\tests\\fixtures\\route_field.txt ^
        --out-dir artifacts\\sim\\baseline

    python tools\\sim_pursuit.py --config config\\robot.yaml ^
        --route src\\dzbrain\\tests\\fixtures\\route_field.txt ^
        --speed-scale-err 0.18 --yaw-drift 0.5 --no-anchor ^
        --out-dir artifacts\\sim\\drift

输出：控制台指标表 + `<out-dir>/traj.csv` + `<out-dir>/traj.svg` + `<out-dir>/by_segment.csv`。
**全程无异常，退出码 0**；参数/文件问题 → 退出码 2（与 `route_check.py` 同约定）。
`--json` 时 stdout 末尾追加一行 `SIM_RESULT_JSON {...}`（机器可读完整指标，契约版本见
模块内 `JSON_SCHEMA`；metrics 键名本体契约仍是 sim.py 头注 §5「只增不改」）。该一行永远
是 stdout 最后一行，文字表排版保持不变。

★ 刻意不做的事
- 不 import rclpy（本工具必须在没有 ROS 的笔记本上跑）。
- 不改任何配置/路线文件：读 `robot.yaml` 只取 `pursuit.*`，路线只读文件。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

from ..core.config import Config, ConfigError
from ..route.model import Route
from ..route.parser import parse_mission
from .sim import (SimConfig, format_metrics, format_segment_table, run_sim,
                  write_csv, write_segments_csv, write_svg)

__all__ = ["build_parser", "main"]

#: 工作区根（本文件在 <ws>/src/dzbrain/dzbrain/tools/ 下 → 上溯 4 层）
_WS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))

#: `SIM_RESULT_JSON` 行的契约版本（细则见移交包 sim_studio/CONTRACT.md；
#:   metrics 键名本体契约仍是 sim.py 头注 §5「只增不改」）。
JSON_SCHEMA = "dzrobot-sim-result/1"


def build_parser() -> argparse.ArgumentParser:
    """构造参数解析器（`main(argv)` 可注入 argv，便于测试）。"""
    ap = argparse.ArgumentParser(
        prog="sim_pursuit",
        description="dzrobot 离线纯追踪仿真器：不碰车就能调参与回归",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--config", default=os.path.join(_WS_ROOT, "config", "robot.yaml"),
                    help="robot.yaml 路径（只读 pursuit.* 与 route.tokens）")
    ap.add_argument("--route", default=None,
                    help="路线文件路径（整行一个 token）；缺省用 --config 里的 route.tokens")
    ap.add_argument("--out-dir", default=os.path.join(_WS_ROOT, "artifacts", "sim"),
                    help="输出目录（写 traj.csv / traj.svg，自动创建）")

    ap.add_argument("--dt", type=float, default=SimConfig.dt, help="控制周期 s")
    ap.add_argument("--latency", type=float, default=SimConfig.cmd_latency_s,
                    help="指令→执行延迟 s（0.06 = 现场链路估计）")
    ap.add_argument("--speed-scale-err", "--speed_scale_err", dest="speed_scale_err",
                    type=float, default=SimConfig.speed_scale_err,
                    help="里程比例误差（+0.18 = 里程计比真实多报 18%%）")
    ap.add_argument("--yaw-drift", "--yaw_drift", dest="yaw_drift_deg_per_min",
                    type=float, default=SimConfig.yaw_drift_deg_per_min,
                    help="航向零漂 °/min")
    ap.add_argument("--pos-noise", "--pos_noise", dest="pos_noise_m",
                    type=float, default=SimConfig.pos_noise_m, help="位置白噪声 m/拍")
    anchor = ap.add_mutually_exclusive_group()
    anchor.add_argument("--anchor", dest="anchor", action="store_true",
                        default=SimConfig.anchor_on_arrival,
                        help="开启到位吸附（模拟 FixTask L2 修正）")
    anchor.add_argument("--no-anchor", dest="anchor", action="store_false",
                        help="关闭到位吸附（对照组：看误差如何累积）")
    ap.add_argument("--anchor-radius", "--anchor_radius", dest="anchor_radius_m",
                    type=float, default=SimConfig.anchor_radius_m, help="吸附触发半径 m")
    ap.add_argument("--max-steps", type=int, default=0,
                    help="步数上限（0 = 按路线长度自动推算）")
    ap.add_argument("--set", action="append", metavar="KEY=VALUE", default=[],
                    help="覆盖任意点号配置键（可重复），如 --set pursuit.arrival_m=0.20；"
                         "只影响本次运行，不写磁盘")
    ap.add_argument("--seed", type=int, default=SimConfig.seed, help="噪声种子（可复现）")
    ap.add_argument("--no-stop", dest="stop_on_completion", action="store_false",
                    default=SimConfig.stop_on_completion,
                    help="到终点不停，一直跑到步数上限（看冲过头用）")
    ap.add_argument("--pretty", action="store_true",
                    help="打印完整配置摘要与逐步统计（默认只打指标表）")
    ap.add_argument("--segments-top", type=int, default=10,
                    help="打印『横向误差峰值最大的 N 段』（0 = 全部按段序打印，-1 = 不打印）")
    ap.add_argument("--quiet", action="store_true", help="只打印指标表，不打标题/文件路径")
    ap.add_argument("--json", dest="json_line", action="store_true",
                    help="在 stdout 末尾追加一行 'SIM_RESULT_JSON {...}'（机器可读完整指标，"
                         "供仿真台移交包解析；不影响上面的文字表，该行永远是最后一行）")
    return ap


def _resolve_route(args: argparse.Namespace, cfg: Config) -> tuple[Route, str]:
    """解析路线：优先 `--route` 文件，否则用配置里的 `route.tokens`。"""
    if args.route:
        path = os.path.abspath(args.route)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"路线文件不存在: {path}")
        return parse_mission(path, name=os.path.basename(path)), path
    tokens = cfg.get("route.tokens")
    if not tokens:
        raise ValueError("既没有 --route，配置里 route.tokens 也是空的")
    return parse_mission(list(tokens), name="cfg:route.tokens"), "<robot.yaml:route.tokens>"


def _parse_set_overrides(items: list[str]) -> dict[str, object]:
    """把 `--set pursuit.arrival_m=0.20` 解析成点号键覆盖表。

    ★ 为什么需要它：仿真器的用途就是"**不碰车**做参数对照实验"。若只能读
      `config/robot.yaml`，那么"现场图要哪组参数才跑得完"这种结论就必须去改仓库里的
      YAML 才能复现 —— 那正是本工具要消灭的动作。有了 `--set`，
      一次调参就是一条自包含、可回贴、可进测试的命令。
    值按 YAML 解析（`0.20`→float、`true`→bool、`[a, b]`→list），失败则当字符串。
    """
    out: dict[str, object] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"--set 需要 key=value 形式，收到 {item!r}")
        key, _, raw = item.partition("=")
        key = key.strip()
        if not key:
            raise ValueError(f"--set 的键为空: {item!r}")
        text = raw.strip()
        try:
            import yaml as _yaml
            value = _yaml.safe_load(text)
        except Exception:                       # pragma: no cover - 没装 PyYAML / 解析失败
            value = text
        out[key] = text if value is None else value
    return out


def _apply_overrides(cfg: Config, overrides: dict[str, object]) -> Config:
    """覆盖配置键（不改磁盘文件），并重新跑一次旧键迁移。"""
    if not overrides:
        return cfg
    flat = dict(cfg._flat)
    flat.update(overrides)
    new = Config(flat, source=f"{cfg.source} + --set×{len(overrides)}")
    new._migrate_legacy()
    return new


def _json_sanitize(value: object) -> object:
    """把非有限浮点（NaN/±Inf）换成 None，保证输出是严格 JSON；其余类型原样。"""
    if isinstance(value, dict):
        return {k: _json_sanitize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_sanitize(v) for v in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (bool, int, str)) or value is None:
        return value
    return str(value)


def _emit_json_error(stage: str, exc: Exception) -> None:
    """--json 模式下，参数/输入类失败也产出一行机器可读结果（ok=false，退出码不变）。"""
    print("SIM_RESULT_JSON " + json.dumps(
        {"schema": JSON_SCHEMA, "ok": False, "stage": stage, "error": str(exc)},
        ensure_ascii=False))


def main(argv: list[str] | None = None) -> int:
    """跑一次仿真并落盘。返回进程退出码（0 正常 / 2 参数或输入有问题）。"""
    args = build_parser().parse_args(argv)

    # ---- 读配置（失败要给人话，而不是堆栈）----
    try:
        cfg = Config.load(args.config)
    except ConfigError as exc:
        print(f"❌ 配置读取失败: {exc}", file=sys.stderr)
        if args.json_line:
            _emit_json_error("config", exc)
        return 2

    # ---- 应用 --set 覆盖（不改磁盘上的 YAML）----
    try:
        overrides = _parse_set_overrides(args.set or [])
        cfg = _apply_overrides(cfg, overrides)
    except ValueError as exc:
        print(f"❌ --set 解析失败: {exc}", file=sys.stderr)
        if args.json_line:
            _emit_json_error("set", exc)
        return 2

    # ---- 读路线 ----
    try:
        route, route_src = _resolve_route(args, cfg)
    except (ValueError, FileNotFoundError) as exc:
        print(f"❌ 路线读取失败: {exc}", file=sys.stderr)
        if args.json_line:
            _emit_json_error("route", exc)
        return 2

    # ---- 组装注入参数 ----
    sim = SimConfig(
        dt=args.dt,
        cmd_latency_s=args.latency,
        speed_scale_err=args.speed_scale_err,
        yaw_drift_deg_per_min=args.yaw_drift_deg_per_min,
        pos_noise_m=args.pos_noise_m,
        anchor_on_arrival=bool(args.anchor),
        anchor_radius_m=args.anchor_radius_m,
        max_steps=args.max_steps,
        seed=args.seed,
        stop_on_completion=args.stop_on_completion,
    )
    try:
        result = run_sim(route, cfg, sim)
    except (ValueError, ConfigError) as exc:
        print(f"❌ 仿真参数非法: {exc}", file=sys.stderr)
        if args.json_line:
            _emit_json_error("run", exc)
        return 2

    # ---- 输出 ----
    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)
    csv_path = write_csv(result, os.path.join(out_dir, "traj.csv"))
    svg_path = write_svg(result, os.path.join(out_dir, "traj.svg"), route,
                         title=f"dzrobot sim: {os.path.basename(route_src)} "
                               f"ε={sim.speed_scale_err:+.0%} anchor={sim.anchor_on_arrival}")
    seg_path = write_segments_csv(result, os.path.join(out_dir, "by_segment.csv"))

    if not args.quiet:
        print("=" * 60)
        print(f"配置   : {os.path.abspath(args.config)}")
        print(f"路线   : {route_src}")
        print(route.describe())
        print("=" * 60)
    print(format_metrics(result, sim, title="指标（truth = 真值轨迹，belief = 控制器看到的位姿）"))
    # ★ 逐段峰值表：调参第一步永远是"看哪几段最差"，而不是盯着一个全局 RMS 猜
    if args.segments_top >= 0:
        print("-" * 46)
        print(format_segment_table(
            result, top=args.segments_top,
            title=f"逐段横向误差峰值（{'全部' if args.segments_top == 0 else f'TOP{args.segments_top}'}）"))
    if not args.quiet:
        print("-" * 46)
        print(f"CSV    : {csv_path}")
        print(f"SVG    : {svg_path}")
        print(f"分段CSV: {seg_path}")
    if args.pretty:
        print("-" * 46)
        print(cfg.summary())
        issues = cfg.validate(strict=False)
        print(f"配置校验: {len(issues)} 条 note（不阻断离线仿真）")
        for issue in issues[:10]:
            print(f"  {issue}")
    # ---- 机器可读契约行：永远是 stdout 最后一行；上面的文字表排版一个字不变 ----
    if args.json_line:
        payload = {
            "schema": JSON_SCHEMA,
            "ok": True,
            "metrics": _json_sanitize(result.metrics),
            "sim": {
                "dt": sim.dt,
                "cmd_latency_s": sim.cmd_latency_s,
                "speed_scale_err": sim.speed_scale_err,
                "yaw_drift_deg_per_min": sim.yaw_drift_deg_per_min,
                "pos_noise_m": sim.pos_noise_m,
                "anchor_on_arrival": sim.anchor_on_arrival,
                "anchor_radius_m": sim.anchor_radius_m,
                "seed": sim.seed,
                "stop_on_completion": sim.stop_on_completion,
            },
            "route": {"source": route_src},
            "config": os.path.abspath(args.config),
            "out_dir": out_dir,
            "artifacts": {"traj_csv": csv_path, "traj_svg": svg_path,
                          "segments_csv": seg_path},
        }
        print("SIM_RESULT_JSON " + json.dumps(payload, ensure_ascii=False))
    return 0


if __name__ == "__main__":      # pragma: no cover
    sys.exit(main())
