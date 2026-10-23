# -*- coding: utf-8 -*-
"""空地联合仿真主入口（t11 + t28 四项改进）。

用法（工作区根 ``D:\\User\\Desktop\\比赛存档\\v1``）::

    $env:PYTHONPATH='D:\\User\\Desktop\\比赛存档\\v1\\dzrobot_v2\\src\\dzbrain'
    $env:PYTHONIOENCODING='utf-8'; $env:PYTHONUTF8='1'
    python dzsmartscout\\tools\\sim_airground\\run_airground.py --help
    python dzsmartscout\\tools\\sim_airground\\run_airground.py --selfcheck      # 确定性自检（不跑全臂）
    python dzsmartscout\\tools\\sim_airground\\run_airground.py                  # 全臂矩阵
    python dzsmartscout\\tools\\sim_airground\\run_airground.py --arms a1,a2     # 指定臂
    python dzsmartscout\\tools\\sim_airground\\run_airground.py --quick          # 四臂快检
    # t28 ③ 靶坐标更改与方向设定（只机侧参数化，不落 config/map.yaml）:
    python ... run_airground.py --arms b3 --targets "F11 G11" --target-orient 0
    # t28 ④ 一键随机地图批量调试（先 10 张试点，再 50 张全量）:
    python ... run_airground.py --random-maps 10 --seed-base 20261012
    python ... run_airground.py --random-maps 50 --seed-base 20261012

输出：每臂 ``_tmp_session/sim_ag/<runid>/<arm>/`` 下 ledger.json + traj.csv + **恰好 1 张**
route_map.png（同一张图：车路线·蓝/车真值轨迹·红/60 个真值位姿·橙 + 无人机航线·青/侦察腿·紫，
图例区分）；随机批量在 ``<out-dir>/random_maps_n<N>/`` 下逐图 route_map.png +
random_maps_summary.json（明细行 + 分量级统计，失败/超时单列）。
**全程无异常 exit 0**；参数问题 exit 2（与 route_check 同约定）。

★ 只读边界：dzrobot_v2（config 只读、src 只 import）、dzsmartscout 既有 tools 只读、
perception 协议只 import；本脚本只写 ``_tmp_session/sim_ag/``。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_TOOLS = os.path.abspath(os.path.join(_HERE, ".."))          # dzsmartscout/tools
_WS = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))  # 工作区根
_DZB = os.path.join(_WS, "dzrobot_v2", "src", "dzbrain")
for _p in (_TOOLS, _DZB):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from dzbrain.core.config import Config                      # noqa: E402

from sim_airground.carsim import run_car                    # noqa: E402
from sim_airground.runner import (ArmResult, blind_failure_analysis,  # noqa: E402
                                  radius_sweep, run_arm, sector_sweep, speed_sweep)
from sim_airground.scenarios import (build_world_a, build_world_b, build_world_c)  # noqa: E402

_RUN_ROOT = os.path.join(_WS, "_tmp_session", "sim_ag")


def _clone(sc):
    import copy
    return copy.deepcopy(sc)


def build_arm_matrix(cfg: Config, baseline: list[str], out_root: str) -> list[tuple[str, dict, object, str]]:
    """返回 [(arm_id, kw, scenario, description)]。每个情景在内存克隆，不落盘。"""
    arms: list[tuple[str, dict, object, str]] = []
    wa = build_world_a()

    arms.append(("A1", dict(drone_on=False, stop_on_violation=False), wa,
                 "无无人机基线（默认格直冲，等价现基线；真值=默认 → 完赛）"))
    arms.append(("A2", dict(drone_on=True, stop_on_violation=False), _clone(wa),
                 "理想揭示（0.5 m/s / 2.5 格 / 0 延迟损耗）：情报对轨迹零影响（真值=默认），"
                 "价值在信息领先契约"))
    wa3 = _clone(wa)
    wa3.link.drop_p = 0.5
    arms.append(("A3", dict(drone_on=True, stop_on_violation=False), wa3,
                 "链路受限：丢包 0.5（0.06 s 延迟 + 抖动）→ 采纳时点抖动（真值=默认 → 轨迹不受影响）"))

    wb = build_world_b("K9", "K4")
    arms.append(("B1", dict(drone_on=False, stop_on_violation=True), _clone(wb),
                 "随机桥洞（K9/K4）+ 无无人机：默认 K10 直冲 → 撞封死桥室中断（③a 直冲失败样例）"))
    arms.append(("B2", dict(drone_on=True, stop_on_violation=True), _clone(wb),
                 "随机桥洞 + 理想无人机 + **现车代码**：情报 K9 被采纳门拒（候选∩地图锁死 K10）→ 仍中断"))
    wb3 = _clone(wb)
    wb3.authority_gate = True
    wb3.wait_at_bridge = True
    arms.append(("B3", dict(drone_on=True, stop_on_violation=True), wb3,
                 "改进变体（权威门 + L10 桥前等待 + 重规划）：采纳 K9/K4 → 完赛；改道 1 次；D10 停站蓝面命中"))
    wb4 = _clone(wb3)
    wb4.drone.takeoff_delay_s = 8.0
    wb4.mission.wait_cap_s = 4.0
    arms.append(("B4", dict(drone_on=True, stop_on_violation=True), wb4,
                 "受限：起飞晚 8 s + 等待上限 4 s → 入口情报 8.57 s 采到、出口情报 15.4 s 未采到"
                 " → 上半场撞封死桥室中断（等待上限敏感性）"))
    wb5 = _clone(wb3)
    wb5.target.judge_error_deg = 180.0        # 蓝红翻转误判
    arms.append(("B5", dict(drone_on=True, stop_on_violation=True), wb5,
                 "受限：靶朝向误判（蓝红翻转 180°）→ 选 A10 站打红面 ×10 = 扣分账（车程不受影响）"))
    wb6 = _clone(wb5)
    wb6.drone.takeoff_delay_s = 8.0
    arms.append(("B6", dict(drone_on=True, stop_on_violation=True), wb6,
                 "受限旗舰：随机桥洞 + 情报迟到 8 s + 朝向误判 → 等待 8.56 s + 红面扣分（三受限要素合并）"))
    wb7 = _clone(wb)
    wb7.drone.takeoff_delay_s = 8.0
    arms.append(("B7", dict(drone_on=True, stop_on_violation=True), wb7,
                 "受限对照：情报迟到 8 s + 现车代码（无桥前等待）→ 车已入 K10 才收到情报 → 中断"
                 "（改道采纳点≠揭示时刻的证据）"))

    wc = build_world_c()
    wc.authority_gate = True
    wc.wait_at_bridge = True
    arms.append(("C1", dict(drone_on=True, stop_on_violation=True), wc,
                 "最坏组合（入口 K11/出口 K1 揭示最晚 + 靶蓝面贴墙不可打）：3 分钟预算上限压力测试"))

    # 蒙特卡洛：无情报直冲失败概率的抽样验证（③a）
    for i, (ent, ext) in enumerate((("K9", "K4"), ("K10", "K4"), ("K11", "K5"), ("K10", "K5"))):
        w = build_world_b(ent, ext)
        arms.append((f"MC{i+1}", dict(drone_on=False, stop_on_violation=True), w,
                     f"蒙特卡洛抽样 {i+1}/4：真值桥洞 {ent}/{ext}、无情报直冲 → "
                     f"{'完赛' if (ent, ext) == ('K10', 'K5') else '中断'}（解析概率 29/30）"))

    # drive_est 标定验证：3 场全仿真（理想策略），对照 6.883 s/m 估计
    for i, (ent, ext) in enumerate((("K7", "K1"), ("K11", "K6"), ("K8", "K3"))):
        w = build_world_b(ent, ext)
        w.authority_gate = True
        w.wait_at_bridge = True
        arms.append((f"MV{i+1}", dict(drone_on=True, stop_on_violation=True), w,
                     f"drive_est 标定验证 {i+1}/3：真值 {ent}/{ext} 理想策略全仿真"
                     f"（对照 6.883 s/m 估计，验证多场次统计的行驶时长标定）"))
    return arms


def _fmt_arm(a: ArmResult) -> str:
    c = a.car
    g = a.gap
    b = a.budget
    s = a.shoot
    done = "是" if c["completed"] else f"否(t={c['field_stop_t']:.1f}s)"
    return (
        f"{a.arm_id:<4} 跑完={done:<12} 时长={c['time_s']:>7.2f}s  RMS={c['cross_track_rms_m']:.4f}  "
        f"MAX={c['cross_track_max_m']:.4f}  超调={c['overshoot_max_m']:.4f}  越界拍={c['out_of_bounds_steps']:>3.0f}  "
        f"改道={a.reroute_count}  关键格缺口拍={g.get('gap_key_ticks', g['gap_ticks']):>5}  "
        f"关键格缺口格={g.get('gap_key_cells', g['gap_cells']):>2}  "
        f"等待={a.wait_lower_s:.2f}s  预算={b['total_s']:>7.2f}s/{b['budget_s']:g}s({'+' if b['within'] else '超'} "
        f"{b['margin_s']:+.2f}s)  蓝中={s['blue_hits']}  红中(扣分)={s['red_hits']}  "
        f"盖雷={a.field_metrics.get('mine_pressed_count', '-')}/{a.field_metrics.get('mine_total', '-')}  "
        f"障碰={a.field_metrics.get('obstacle_hit_steps', '-')}  桥室={a.field_metrics.get('k_solid_cell_steps', '-')}"
    )


def run_all(out_root: str, arms: list[str] | None = None,
            targets: tuple[str, ...] | None = None,
            orient: float | None = None, no_stats: bool = False) -> int:
    cfg = Config.load(os.path.join(_WS, "dzrobot_v2", "config", "robot.yaml"))
    baseline = [str(t) for t in (cfg.get("route.tokens") or [])]
    matrix = build_arm_matrix(cfg, baseline, out_root)
    # t28 ③ 靶覆盖：只改内存情景（truth.targets / track_cells / 朝向 / 停站候选），不落盘
    if targets is not None or orient is not None:
        from sim_airground.scenarios import apply_target_override
        try:
            matrix = [(aid, kw, apply_target_override(sc, targets, orient) if targets
                       else apply_target_override(sc, sc.target.track_cells, orient), desc)
                      for aid, kw, sc, desc in matrix]
        except ValueError as exc:
            print(f"❌ 靶覆盖参数非法: {exc}", file=sys.stderr)
            return 2
    if arms:
        want = {a.strip() for a in arms}
        matrix = [row for row in matrix if row[0] in want]
        if not matrix:
            print(f"❌ 未知臂名：{arms}（可用：{sorted(r[0] for r in build_arm_matrix(cfg, baseline, out_root))}）",
                  file=sys.stderr)
            return 2
    results: list[ArmResult] = []
    t0 = time.perf_counter()
    for arm_id, kw, sc, desc in matrix:
        out_dir = os.path.join(out_root, arm_id.lower())
        res = run_arm(arm_id, sc, cfg, baseline, out_dir=out_dir, description=desc, **kw)
        results.append(res)
        print(_fmt_arm(res))
    print("=" * 130)

    # ---- ① 多场次随机抽取统计（桥洞 30 组合 × 靶面朝向 4 种 × 3 策略） ----
    # t28：--no-stats 跳过本段（360 次规划 ≈ 6 分钟），只跑臂 + 台账（靶对照等快速场景用）
    fa, rs, ss, ssw, ms = {}, [], [], [], {}
    if not no_stats:
        from dzbrain.tools.sim import parse_map_yaml
        from sim_airground.match_stats import run_match_stats
        with open(os.path.join(_WS, "dzrobot_v2", "config", "map.yaml"), encoding="utf-8") as fh:
            map_field = parse_map_yaml(fh.read(), source="config/map.yaml")
        ref_dir = os.path.join(out_root, "_ref")
        ref = run_car("_ref", baseline, cfg, map_field, out_dir=ref_dir)
        drive_s_per_m = ref.metrics["time_s"] / max(ref.metrics["path_length_m"], 1e-9)
        ms = run_match_stats(cfg, baseline, ref.arrival_sim, drive_s_per_m,
                             commit_t=2.0, arrival_l10_sim=ref.arrival_sim.get("L10", 5.9))
        print(f"[④] 无人机侦察耗时 = {ms['drone_recon_s']} s（航速 0.5 m/s；不含起飞延迟）｜"
              f"总预算 = {ms['budget_s']} s｜drive_est 标定 = {drive_s_per_m:.3f} s/m"
              f"（基线实测 {ref.metrics['time_s']:.2f} s / {ref.metrics['path_length_m']:.2f} m）")
        print("[①] 30 组合 × 4 朝向 多场次统计：")
        for pname in ("no_drone", "ideal_wait", "constrained"):
            st = ms[pname]
            print(f"  {pname:<12} 完赛率={st.completion_rate:>6.1%}  预算通过率={st.budget_pass_rate:>6.1%}  "
                  f"蓝面命中率={st.blue_rate:>6.1%}  红面扣分率={st.red_rate:>6.1%}  "
                  f"均分={st.mean_score:>5}  均裕量={st.mean_margin_s:>6.1f}s  "
                  f"最差裕量={st.worst_margin_s:>6.1f}s（{st.worst_case}）  均等待={st.mean_wait_s:.2f}s")

        # ---- 分析输出（③a/b/c/d/e + ④扇区表） ----
        fa = blind_failure_analysis({"lower": "K10", "upper": "K5"})
        print(f"[③a] 无情报直冲失败概率 = {fa['fail_probability']}（{fa['combos_pass']}/{fa['combos_total']} 组合可过）"
              f"；{fa['note']}")
        rs = radius_sweep(build_world_b("K9", "K4"))
        print("[③D] 感知半径敏感度（关键格覆盖）：")
        for row in rs:
            print(f"  半径 {row['radius_cells']:g} 格({row['radius_m']:g}m)：覆盖 {row['covered']}/{row['key_cells']}"
                  f"{'；未覆盖: ' + ','.join(row['uncovered']) if row['uncovered'] else ''}"
                  f"；最晚揭示 {row['max_reveal_t']}s")
        ss = speed_sweep(build_world_b("K9", "K4"))
        print("[③E] 无人机速度档位（揭示时刻，s）：")
        for row in ss:
            print(f"  v={row['speed_mps']:g} m/s：桥洞 {row['bridge_reveals']}；全要素揭示 ≤ {row['all_key_revealed_by']}s"
                  f"；航线 {row['flight_end_t']}s")
        ssw = sector_sweep(build_world_b("K9", "K4"))
        print("[④] 蓝面可见扇区敏感度（朝向 φ × 扇区 ±30/±45/±60）：")
        for row in ssw:
            print(f"  φ={row['phi_deg']:>4.0f}° 扇区±{row['sector_deg']:>2.0f}° → 停站={str(row['chosen_stop']):<5} "
                  f"{row['side']}（驻留 {row['dwell_s']}s）")

    def _stats_dict(st) -> dict:
        row_keys = ("entrance", "exit_cell", "phi", "adopted_lower", "adopted_upper",
                    "wait_s", "complete", "reason", "drive_est_s", "dwell_s", "budget_s",
                    "within", "margin_s", "blue", "red", "score", "stop")
        if not hasattr(st, "policy"):
            return st
        return {"policy": st.policy, "n": st.n, "completion_rate": st.completion_rate,
                "budget_pass_rate": st.budget_pass_rate, "blue_rate": st.blue_rate,
                "red_rate": st.red_rate, "mean_score": st.mean_score,
                "mean_margin_s": st.mean_margin_s, "worst_margin_s": st.worst_margin_s,
                "worst_case": st.worst_case, "max_wait_s": st.max_wait_s,
                "mean_wait_s": st.mean_wait_s,
                "rows": [{k: getattr(r, k) for k in row_keys} for r in st.rows]}

    with open(os.path.join(out_root, "ledger.json"), "w", encoding="utf-8") as fh:
        json.dump({"arms": [{k: v for k, v in r.__dict__.items()} for r in results],
                   "blind_failure": fa, "radius_sweep": rs, "speed_sweep": ss,
                   "sector_sweep": ssw,
                   "match_stats": {k: _stats_dict(st) for k, st in ms.items()},
                   "drone_recon_s": ms.get("drone_recon_s"),
                   "budget_s": ms.get("budget_s"),
                   "no_stats": no_stats},
                  fh, ensure_ascii=False, indent=2, default=str)
    print(f"\n台账: {os.path.join(out_root, 'ledger.json')}（{len(results)} 臂，"
          f"总耗时 {time.perf_counter() - t0:.1f}s，全确定性可复跑）")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="run_airground",
        description="空地联合仿真（t11+t28）：无人机天眼模型 + 情报注入小车（dzbrain run_sim 只读）")
    ap.add_argument("--out-dir", default=os.path.join(_RUN_ROOT, time.strftime("%Y%m%d_%H%M%S")),
                    help="输出根目录（每臂一个子目录）")
    ap.add_argument("--arms", default=None, help="逗号分隔的臂名（默认全部），如 a1,b3")
    ap.add_argument("--quick", action="store_true", help="四臂快检：a1 a2 b1 b3")
    ap.add_argument("--selfcheck", action="store_true", help="只跑确定性自检（tests/）")
    ap.add_argument("--no-stats", action="store_true",
                    help="跳过 360 场解析统计段（约省 6 分钟；臂与台账照常）")
    # ---- t28 ③ 靶坐标更改与方向设定（只机侧参数化，绝不落 config/map.yaml） ----
    ap.add_argument("--targets", default=None,
                    help='覆盖靶格位：同行/同列相邻两格，如 "F11 G11"（默认 B11 C11）')
    ap.add_argument("--target-orient", type=float, default=None,
                    help="覆盖靶蓝面朝向（度，0=朝北/场内）")
    ap.add_argument("--targets-file", default=None,
                    help='靶参数 JSON 文件：{"cells": ["F11","G11"], "orientation_deg": 90}'
                         "（CLI --targets/--target-orient 优先于文件）")
    # ---- t28 ④ 地图调试：一键多种子随机地图批量 ----
    ap.add_argument("--random-maps", type=int, default=0,
                    help="随机地图批量调试：>0 = 生成 N 张（桥洞/靶朝向按 t11 规则随机）")
    ap.add_argument("--seed-base", type=int, default=20261012,
                    help="随机地图种子基（第 i 张 seed = seed_base + i，可复现）")
    args = ap.parse_args(argv)

    if args.selfcheck:
        from sim_airground.tests import test_airground
        return 0 if test_airground.main() else 1

    # ---- 靶参数解析（CLI > 文件 > 默认） ----
    targets_cells: tuple[str, ...] | None = None
    orient_deg: float | None = None
    file_targets: tuple[str, ...] | None = None
    file_orient: float | None = None
    if args.targets_file:
        try:
            with open(args.targets_file, encoding="utf-8") as fh:
                spec = json.load(fh)
            file_targets = tuple(str(c).upper() for c in spec.get("cells") or ()) or None
            if "orientation_deg" in spec:
                file_orient = float(spec["orientation_deg"])
        except (OSError, ValueError, TypeError) as exc:
            print(f"❌ --targets-file 读取失败: {exc}", file=sys.stderr)
            return 2
    if args.targets:
        try:
            from sim_airground.scenarios import parse_target_cells
            targets_cells = parse_target_cells(args.targets)
        except ValueError as exc:
            print(f"❌ --targets 非法: {exc}", file=sys.stderr)
            return 2
    targets_cells = targets_cells or file_targets
    orient_deg = args.target_orient if args.target_orient is not None else file_orient

    os.makedirs(args.out_dir, exist_ok=True)

    # ---- t28 ④ 随机地图批量（独立模式；忽略 --arms） ----
    if args.random_maps > 0:
        from sim_airground.random_maps import run_random_maps
        cfg = Config.load(os.path.join(_WS, "dzrobot_v2", "config", "robot.yaml"))
        baseline = [str(t) for t in (cfg.get("route.tokens") or [])]
        root = os.path.join(args.out_dir, f"random_maps_n{args.random_maps}")
        t0 = time.perf_counter()
        stats = run_random_maps(cfg, baseline, root, n=args.random_maps,
                                seed_base=args.seed_base,
                                targets=targets_cells, orient=orient_deg)
        wall = time.perf_counter() - t0
        print(f"t28④ 随机地图批量：N={stats['n_total']} seed_base={args.seed_base} "
              f"wall={wall:.1f}s（单图 {wall / max(1, stats['n_total']):.2f}s）")
        print(f"  完赛 {stats['n_completed']}/{stats['n_total']}  "
              f"超预算 {stats['n_over_budget']}  "
              f"平均计算时间(规划,全样本) = {stats['avg_plan_s']:.3f}s  "
              f"平均完赛时间(完赛样本) = {stats['avg_finish_s']}s  "
              f"完赛率 = {stats['completion_rate']:.1%}  预算通过率 = {stats['budget_pass_rate']:.1%}")
        for r in stats["rows"]:
            print("   " + _fmt_random_row(r))
        if stats["abort_rows"]:
            print(f"  ⚠️ 失败样本单列（{stats['n_aborted']} 张，不进完赛均值）：")
            for r in stats["abort_rows"]:
                print("     " + _fmt_random_row(r))
        if stats["over_budget_rows"]:
            print(f"  ⚠️ 超预算样本单列（{stats['n_over_budget']} 张，不进完赛均值）：")
            for r in stats["over_budget_rows"]:
                print("     " + _fmt_random_row(r))
        print(f"  台账: {os.path.join(root, 'random_maps_summary.json')}")
        return 0

    arms = ["A1", "A2", "B1", "B3"] if args.quick else \
        ([a.strip().upper() for a in args.arms.split(",")] if args.arms else None)
    return run_all(args.out_dir, arms, targets=targets_cells, orient=orient_deg,
                   no_stats=args.no_stats)


def _fmt_random_row(r: dict) -> str:
    status = "完赛" if r["completed"] else f"中断@t={r['abort_t_s']}s"
    budget = ("预算内" if r["within_budget"] is True else
              "超预算" if r["within_budget"] is False else "—")
    return (f"{r['map_id']:<5} seed={r['seed']:<10} 桥={r['entrance']}/{r['exit_cell']:<4} "
            f"φ={r['phi']:>4.0f}°  规划={r['plan_s']:>5.2f}s tokens={r['plan_tokens_n']:>2} "
            f"len={r['polyline_m']:>6.2f}m  {status:<12} drive={r['drive_s']:>7.2f}s "
            f"wait={r['wait_s']:>4.2f}s dwell={r['dwell_s']:>5.2f}s 总={r['total_s']:>7.2f}s "
            f"{budget}  盖雷={r['mine_pressed']:<4} 蓝={r['blue']} 红={r['red']}  "
            f"traj={str(r['traj_sha256'])[:8]}")


if __name__ == "__main__":          # pragma: no cover
    sys.exit(main())
