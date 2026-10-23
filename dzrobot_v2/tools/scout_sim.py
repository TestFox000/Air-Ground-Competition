# -*- coding: utf-8 -*-
r"""机端侦察仿真入口（步进循环协议 v2 / T28）。

生成"无人机侦察 -> 布局 v2 -> 目标格"的完整时间线，并给出与用户手绘路线的
覆盖率/帧数/时长对照。**纯离线**：不连 ROS、不发 DDS、不写 config。

用法：
    python tools\scout_sim.py                     # 人类可读报告（exit 0 = 指标达标）
    python tools\scout_sim.py --start-t -30       # T29 场景：t=-30s 起飞（默认就是 -30）
    python tools\scout_sim.py --json              # 机器可读（含布局序列）
    python tools\scout_sim.py --json --frames     # 额外附带逐帧 pose/视场
    python tools\scout_sim.py --car-block K7      # 车当前位置 -> target_cell

判据（与协议 §4/§5 一致的机端部分）：
  1. 覆盖率 = 154/154；
  2. 相邻帧物理足迹重叠 >= 30%；
  3. known 单调增长、布局 version 单调递增；
  4. 每份布局都能被车端 \`normalize_field_layout\` 接受（v2 校验闭环）。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve()
_WS = _HERE.parents[1]
sys.path.insert(0, str(_WS / "src" / "dzbrain"))

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

from dzbrain.scout import simulate                                   # noqa: E402
from dzbrain.scout.frames import DEFAULT_FRAME_HZ                    # noqa: E402
from dzbrain.scout.heartbeat import DEFAULT_HEARTBEAT_S              # noqa: E402


def _fmt_pct(value: float) -> str:
    return f"{value * 100:.1f}%"


def _route_row(label: str, stats: dict) -> str:
    missing = ",".join(stats.get("missing") or []) or "-"
    return (f"  {label:<14} {stats['frames']:>3} 帧 / {stats['duration_s']:>6.1f} s / "
            f"{stats['coverage_cells']}/{stats['cells_total']} "
            f"({_fmt_pct(stats['coverage_ratio'])})  缺: {missing}")


def report(res, *, check: bool = True) -> int:
    stats = res.stats
    cmp = res.comparison
    print("=" * 78)
    print("机端侦察仿真 · 协议 v2（T28）")
    print("=" * 78)
    print(f"真值图      : {res.truth.name or '(未命名)'}  "
          f"start={res.truth.start} end={res.truth.end}  "
          f"雷 {len(res.truth.mines)} / 障 {len(res.truth.obstacles)} / "
          f"靶 {len(res.truth.targets)} / 桥洞 {sorted(res.truth.bridges)}")
    print(f"视场模型    : {stats['fov_length_cells']} 格(沿航向) x "
          f"{stats['fov_width_cells']} 格(横向) = 1.40x0.70 m @ 1.5 m"
          f"（格点姿态，0°/90° 恰好 8 格）")
    print(f"帧位        : {stats['frames']} 帧 / {stats['duration_s']:.1f} s "
          f"(t={stats['start_t']:.1f}..{stats['end_t']:.1f} s, 2 Hz)")
    print(f"覆盖        : {stats['coverage_cells']}/{stats['cells_total']} "
          f"({_fmt_pct(stats['coverage_ratio'])})  缺: "
          f"{','.join(stats['missing']) or '-'}")
    print(f"相邻帧重叠  : 物理面积 min {_fmt_pct(stats['overlap_area_min'])} / "
          f"mean {_fmt_pct(stats['overlap_area_mean'])}；"
          f"格集 Jaccard min {_fmt_pct(stats['overlap_cell_min'])}")
    print(f"每帧格数    : {stats['cells_per_frame_min']}~{stats['cells_per_frame_max']}")
    print(f"布局        : {len(res.layouts)} 份 / known {res.known_count} 格 / "
          f"target={res.layouts[-1]['target_cell'] if res.layouts else '-'} / "
          f"末版 version={res.layouts[-1]['version'] if res.layouts else '-'}")
    print(f"心跳        : {len(res.heartbeat)} 条（变化 {len(res.layouts)} + "
          f"周期 {len(res.heartbeat) - len(res.layouts)}）")
    print("-" * 78)
    print("对照（同一视场模型）：")
    print(_route_row("推荐路线", cmp["generated"]))
    print(_route_row("用户手绘", cmp["user_route"]))
    print(_route_row("用户手绘0.7m", cmp["user_route_same_step"]))
    d = cmp["delta"]
    print(f"  Δ(推荐-用户手绘): 覆盖 {d['coverage_cells']:+d} 格 / "
          f"帧数 {d['frames']:+d} / 时长 {d['duration_s']:+.1f} s")
    print(f"  来源: {cmp['source']['image']}（{cmp['source']['method']}；"
          f"起点 {cmp['source']['start']}，终点 {cmp['source']['end']}）")
    print("=" * 78)

    ok = True
    if check:
        if stats["coverage_cells"] != stats["cells_total"]:
            print(f"❌ 覆盖率不足: {stats['coverage_cells']}/{stats['cells_total']}")
            ok = False
        if stats["overlap_area_min"] < 0.3 - 1e-9:
            print(f"❌ 相邻帧面积重叠 < 30%: {stats['overlap_area_min']:.3f}")
            ok = False
        knowns = [len(x["known"]) for x in res.layouts]
        if knowns != sorted(knowns):
            print("❌ known 不单调")
            ok = False
        versions = [int(x["version"]) for x in res.layouts]
        if versions != sorted(versions) or len(set(versions)) != len(versions):
            print("❌ version 非严格递增")
            ok = False
        try:
            from dzbrain.tasks.drone import FIELD_LAYOUT_VERSION, normalize_field_layout
            bad = 0
            for layout in res.layouts:
                norm, why = normalize_field_layout(layout)
                if norm is None:
                    bad += 1
                    print(f"❌ 车端拒收 layout v{layout.get('version')}: {why}")
            if bad:
                ok = False
            elif FIELD_LAYOUT_VERSION < 2:
                print(f"❌ 车端 FIELD_LAYOUT_VERSION={FIELD_LAYOUT_VERSION} < 2")
                ok = False
        except Exception as exc:                     # noqa: BLE001  门禁不可用不该静默
            print(f"❌ 车端校验闭环不可用: {type(exc).__name__}: {exc}")
            ok = False
        print("✅ 机端指标全部达标" if ok else "❌ 机端指标未达标")
    return 0 if ok else 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="机端侦察仿真（协议 v2）")
    ap.add_argument("--map", dest="map_path", default=None,
                    help="真值图（默认 <repo>/config/map.yaml；只读）")
    ap.add_argument("--start-t", type=float, default=-30.0,
                    help="帧位起始时刻（T29 场景 -30；默认 -30）")
    ap.add_argument("--frame-hz", type=float, default=DEFAULT_FRAME_HZ,
                    help=f"帧频（默认 {DEFAULT_FRAME_HZ:g} Hz）")
    ap.add_argument("--car-block", default=None,
                    help="车当前格（决定 target_cell；默认图上的 start）")
    ap.add_argument("--heartbeat-s", type=float, default=DEFAULT_HEARTBEAT_S,
                    help=f"心跳周期（默认 {DEFAULT_HEARTBEAT_S:g} s）")
    ap.add_argument("--json", action="store_true", help="输出机器可读 JSON")
    ap.add_argument("--frames", action="store_true",
                    help="JSON 中附带逐帧 pose/视场（较大）")
    ap.add_argument("--no-check", action="store_true", help="跳过达标判据（只打印）")
    ap.add_argument("--out", default=None, help="把 JSON 写到该路径（仍打印报告）")
    args = ap.parse_args(argv)

    res = simulate(map_path=args.map_path, start_t=args.start_t,
                   frame_hz=args.frame_hz, car_block=args.car_block,
                   heartbeat_interval_s=args.heartbeat_s)
    if args.json:
        print(json.dumps(res.as_dict(include_frames=args.frames),
                         ensure_ascii=False, indent=2))
        rc = 0
        if not args.no_check:
            st = res.stats
            rc = 0 if (st["coverage_cells"] == st["cells_total"]
                       and st["overlap_area_min"] >= 0.3 - 1e-9) else 1
    else:
        rc = report(res, check=not args.no_check)
    if args.out:
        pathlib.Path(args.out).write_text(
            json.dumps(res.as_dict(include_frames=args.frames),
                       ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"JSON 已写入 {args.out}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
