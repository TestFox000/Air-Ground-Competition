# -*- coding: utf-8 -*-
"""sim_airground 确定性自检（`python dzsmartscout\\tools\\sim_airground\\run_airground.py --selfcheck`）。

覆盖：几何往返 / 无人机揭示时刻 / 链路确定性 / 采纳门镜像 / 三连换洞 / 双面靶命中侧 /
半径最坏可接受值 / 臂级复跑 sha256 逐字节相同。
"""

from __future__ import annotations

import os
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_TOOLS = os.path.abspath(os.path.join(_HERE, "..", ".."))
_WS = os.path.abspath(os.path.join(_HERE, "..", "..", "..", ".."))
for _p in (_TOOLS, os.path.join(_WS, "dzrobot_v2", "src", "dzbrain")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from dzbrain.core.config import Config                       # noqa: E402
from dzbrain.tools.sim import parse_map_yaml                 # noqa: E402

from sim_airground import geo as G                          # noqa: E402
from sim_airground.drone_model import build_timeline        # noqa: E402
from sim_airground.link import Link                         # noqa: E402
from sim_airground.car_gate import (CarIntelGate, MissionFlow, _rewrite_triple)  # noqa: E402
from sim_airground.carsim import run_car                    # noqa: E402
from sim_airground.plot import LEGEND_ROW_ROUTE, draw_map   # noqa: E402
from sim_airground.random_maps import run_random_maps       # noqa: E402
from sim_airground.runner import radius_sweep, run_arm      # noqa: E402
from sim_airground.scenarios import (LinkParams, apply_target_override,  # noqa: E402
                                     build_world_a, build_world_b, build_world_c)
from sim_airground.shoot import evaluate_shooting, panel_center  # noqa: E402

_CHECKS: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    _CHECKS.append((name, bool(ok), detail))


def main() -> bool:
    # ---- 1. 几何往返（车侧公式口径） ----
    n_ok = 0
    for r in range(11):
        for c in range(14):
            b = G.index_to_block(r, c)
            x, y = G.block_to_meters(b)
            if abs(x - (10.5 - r) * 0.35) < 1e-9 and abs(y - (13.5 - c) * 0.35) < 1e-9 \
                    and G.meters_to_block(x, y) == b:
                n_ok += 1
    check("geo 格名↔米制 154 格往返", n_ok == 154, f"{n_ok}/154")

    # ---- 2. 无人机揭示时刻（0.5 m/s，南向先行侦察腿） ----
    wb = build_world_b("K9", "K4")
    tl = build_timeline(wb)
    key = set(wb.truth.mines) | set(wb.truth.obstacles) | set(wb.truth.targets) \
        | set(wb.truth.bridges)
    check("K9 揭示 ≤2.2 s（K8 半径内）", tl.reveal_time.get("K9", 9e9) <= 2.2,
          f"实际 {tl.reveal_time.get('K9')}")
    check("K4 揭示 ≤8.5 s（北向全列腿 K5 处）", tl.reveal_time.get("K4", 9e9) <= 8.5,
          f"实际 {tl.reveal_time.get('K4')}")
    wc2 = build_world_c()
    tlc = build_timeline(wc2)
    check("K11 揭示 ≤3.6 s（南向腿先行）", tlc.reveal_time.get("K11", 9e9) <= 3.6,
          f"实际 {tlc.reveal_time.get('K11')}")
    check("K1 揭示 ≤11.3 s（北向全列腿末端）", tlc.reveal_time.get("K1", 9e9) <= 11.3,
          f"实际 {tlc.reveal_time.get('K1')}")
    check("2.5 格半径全关键格被揭示", key <= set(tl.reveal_time),
          f"漏 {sorted(key - set(tl.reveal_time))}")

    # ---- 3. 链路确定性 ----
    msgs = [(0.0, {"type": "heartbeat"}), (1.0, {"type": "heartbeat"})]
    l1, s1 = Link(LinkParams(seed=7, drop_p=0.3)).apply(msgs, 0.0)
    l2, s2 = Link(LinkParams(seed=7, drop_p=0.3)).apply(msgs, 0.0)
    check("链路同种子逐位相同", [(m.recv_t, m.payload) for m in l1] == [(m.recv_t, m.payload) for m in l2],
          f"{len(l1)}/{len(l2)} 条")
    l3, s3 = Link(LinkParams(seed=7, drop_p=1.0)).apply(msgs, 0.0)
    check("drop=1 全丢", len(l3) == 0 and s3.dropped == 2, f"dropped={s3.dropped}")

    # ---- 4. 采纳门镜像（t6 口径） ----
    wb2 = build_world_b("K9", "K4")
    gate = CarIntelGate(wb2)
    snap = gate.intel_snapshot(0.0)
    check("无报文 → no_link/回落默认",
          all(v.source == "no_link" and v.cell == wb2.defaults[h] for h, v in snap.items()),
          str({h: v.source for h, v in snap.items()}))
    gate.feed(0.1, {"type": "heartbeat"})
    gate.feed(1.0, {"type": "bridge_passable", "half": "lower", "block": "K9", "confidence": 1.0})
    snap = gate.intel_snapshot(1.0)
    check("现车候选锁死 → K9 被拒", snap["lower"].source == "rejected"
          and snap["lower"].cell == "K10", snap["lower"].describe())
    wb2.authority_gate = True
    snap = gate.intel_snapshot(1.0)
    check("权威门 → K9 采纳", snap["lower"].source == "drone" and snap["lower"].cell == "K9",
          snap["lower"].describe())
    snap31 = gate.intel_snapshot(31.0)
    check("心跳断 3 s → offline 回落（先于过期判定，镜像 mission.py:1502）",
          snap31["lower"].source == "offline" and snap31["lower"].cell == "K10",
          snap31["lower"].describe())
    gate.feed(30.5, {"type": "heartbeat"})
    snap31 = gate.intel_snapshot(31.05)
    check("心跳续上后按 age 判过期 → stale",
          snap31["lower"].source == "stale", snap31["lower"].describe())
    gate.feed(35.0, {"type": "heartbeat"})
    gate.feed(36.0, {"type": "bridge_passable", "half": "upper", "block": "K4", "confidence": 1.0})
    snap36 = gate.intel_snapshot(36.0)
    check("上半场情报独立采纳", snap36["upper"].source == "drone" and snap36["upper"].cell == "K4",
          snap36["upper"].describe())

    # ---- 5. 三连换洞（镜像 rewrite_crossing_tokens 形状） ----
    cfg = Config.load(os.path.join(_WS, "dzrobot_v2", "config", "robot.yaml"))
    baseline = [str(t) for t in (cfg.get("route.tokens") or [])]
    new = _rewrite_triple(baseline, "K10", "K9")
    check("三连换洞 L10K10J10→L9K9J9", new is not None and new[2] == "L9" and new[3] == "K9"
          and new[4] == "J9", " ".join((new or [])[:6]))

    # ---- 6. 双面移动靶命中侧（② 命中时扇区口径，逐发按面板位置） ----
    wb6 = build_world_b("K9", "K4")
    on_route = {"A10", "B10", "C10", "D10"}
    out = evaluate_shooting(wb6.target, "C10", on_route)          # φ=-90，判读无误
    check("φ=-90 → 选 D10 且蓝面命中为主", out.stop == "D10" and out.blue_hits >= 8
          and out.red_hits == 0, f"{out.stop} 蓝{out.blue_hits} 红{out.red_hits}")
    wb6.target.judge_error_deg = 180.0                            # 蓝红翻转
    out2 = evaluate_shooting(wb6.target, "C10", on_route)
    check("误判翻转 → A10 打红面扣分（蓝 0）", out2.stop == "A10" and out2.red_hits >= 2
          and out2.blue_hits == 0, f"{out2.stop} 蓝{out2.blue_hits} 红{out2.red_hits}")
    wc = build_world_c()                                          # φ=180 贴墙
    out3 = evaluate_shooting(wc.target, "C10", on_route)
    check("φ=180 贴墙 → 无解（0 发驻留 30 s）", out3.sector_fail and out3.dwell_s == 30.0
          and out3.blue_hits == 0 and out3.red_hits == 0, f"dwell={out3.dwell_s}")
    ys = [panel_center(t / 10.0, wb6.target)[1] for t in range(0, 113)]
    check("面板半行程 ±0.28 m（0.4 格宽板）", abs(max(ys) - min(ys) - 0.56) < 1e-6,
          f"行程 {max(ys) - min(ys):.3f}")

    # ---- 7. 重规划路线前缀一致性（改道 splice 论证的前提） ----
    flow = MissionFlow(wb6, cfg, baseline)
    snap0 = flow.gate.intel_snapshot(0.0)
    v1 = flow.load_route(snap0)
    check("基线路线前缀 N10 M10 L10", v1.tokens[:3] == ["N10", "M10", "L10"],
          " ".join(v1.tokens[:3]))

    # ---- 8. 半径最坏可接受值（2.0 全盖 / 1.75 漏 2 行偏移格） ----
    rs = radius_sweep(wb6, radii=(1.75, 2.0))
    r20 = next(r for r in rs if r["radius_cells"] == 2.0)
    r175 = next(r for r in rs if r["radius_cells"] == 1.75)
    check("半径 2.0 格全盖关键格", r20["covered"] == r20["key_cells"], str(r20["uncovered"]))
    r175_un = set(r175["uncovered"])
    check("半径 1.75 格漏行 1/行 11 关键格（2 行偏移）",
          r175_un >= {"B1", "D1", "E11", "G11", "B11", "C11"}, str(sorted(r175_un)))
    check("半径 1.75 格 B7/C9/E10 仍覆盖（A 列腿/行 9 腿直过）",
          r175_un.isdisjoint({"B7", "C9", "E10"}), str(sorted(r175_un)))

    # ---- 9. 臂级复跑逐字节相同（traj.csv sha256）+ 基线等价 ----
    wa = build_world_a()
    with open(os.path.join(_WS, "dzrobot_v2", "config", "map.yaml"), encoding="utf-8") as fh:
        field = parse_map_yaml(fh.read(), source="config/map.yaml")
    d1 = tempfile.mkdtemp(prefix="ag_test_")
    d2 = tempfile.mkdtemp(prefix="ag_test_")
    r1 = run_car("t", baseline, cfg, field, out_dir=d1)
    r2 = run_car("t", baseline, cfg, field, out_dir=d2)
    check("run_sim 复跑 traj.csv sha256 逐字节相同", r1.sha256 == r2.sha256, r1.sha256[:16])
    check("基线完成指标（104.54 s / RMS 0.0489 级，@0.35 权威档）", r1.metrics["completed"]
          and abs(r1.metrics["time_s"] - 104.54) < 1.0
          and abs(r1.metrics["cross_track_rms_m"] - 0.0489) < 0.002,
          f"{r1.metrics['time_s']:.2f}s RMS {r1.metrics['cross_track_rms_m']:.4f}")

    # ---- 10. t28① 一张图双路线（同坐标系同 PNG + 图例区分，按精确色值数像素） ----
    wb_plot = build_world_b("K9", "K4")
    wb_plot.authority_gate = True
    tl_plot = build_timeline(wb_plot)
    tokens_plot = _rewrite_triple(_rewrite_triple(list(baseline), "K10", "K9"), "K5", "K4")
    check("t28① 预备：三连换洞得 K9/K4 路线", tokens_plot is not None,
          " ".join((tokens_plot or [])[:6]))
    d_plot = tempfile.mkdtemp(prefix="ag_plot_")
    car_plot = run_car("plot", tokens_plot or list(baseline), cfg, wb_plot.truth,
                       out_dir=d_plot)
    png_path = draw_map(os.path.join(d_plot, "route_map.png"),
                        title="t28①自检", truth=wb_plot.truth, car_tokens=tokens_plot,
                        car_traj=[(p[0], p[1]) for p in car_plot.truth_pose],
                        car_poses=car_plot.truth_pose,
                        pose_every=max(1, (len(car_plot.truth_pose) + 59) // 60),
                        drone=tl_plot)
    from PIL import Image as _Image
    img_plot = _Image.open(png_path).convert("RGB")
    px = img_plot.load()
    W, H = img_plot.size

    def _count(color, y_max=None) -> int:
        y_hi = H if y_max is None else min(H, y_max)
        n = 0
        for yy in range(0, y_hi, 3):                 # 隔行抽样数像素（速度）
            for xx in range(0, W, 3):
                if px[xx, yy] == color:
                    n += 1
        return n

    # ★ 主画布区（图例带以下）：图例自带颜色色块，数像素必须排除图例区（H-80 以下两行）
    n_route, n_drone, n_traj, n_pose = (_count((30, 80, 220), H - 80),
                                        _count((60, 160, 200), H - 80),
                                        _count((200, 40, 40), H - 80),
                                        _count((240, 120, 30), H - 80))
    check("t28① 一张图四要素齐：车路线(蓝)/无人机航线(青)/车真值轨迹(红)/车位姿(橙) 像素全>0",
          n_route > 0 and n_drone > 0 and n_traj > 0 and n_pose > 0,
          f"蓝{n_route} 青{n_drone} 红{n_traj} 橙{n_pose}")
    check("t28① 图例含双路线与位姿口径条目",
          any("车路线" in t for t, _ in LEGEND_ROW_ROUTE)
          and any("无人机航线" in t for t, _ in LEGEND_ROW_ROUTE)
          and any("车位姿(真值)" in t for t, _ in LEGEND_ROW_ROUTE),
          "；".join(t for t, _ in LEGEND_ROW_ROUTE))
    png_nopose = draw_map(os.path.join(d_plot, "no_pose.png"), title="对照", truth=wb_plot.truth,
                          car_tokens=tokens_plot, drone=tl_plot)
    img_no = _Image.open(png_nopose).convert("RGB")
    px_no = img_no.load()
    n_pose_no = sum(1 for yy in range(0, img_no.size[1] - 80, 3)
                    for xx in range(0, img_no.size[0], 3)
                    if px_no[xx, yy] == (240, 120, 30))
    check("t28① 有牙：不给位姿/轨迹 → 主画布橙像素为 0（图例色块除外）", n_pose_no == 0,
          f"橙 {n_pose_no}")
    _stride = max(1, (len(car_plot.truth_pose) + 59) // 60)
    _drawn = -(-len(car_plot.truth_pose) // _stride)          # ceil(steps/stride)
    check("t28② 位姿点数定死 = min(60, steps)（真值口径）",
          _drawn == min(60, len(car_plot.truth_pose)) and _drawn == 60,
          f"steps={len(car_plot.truth_pose)} stride={_stride} drawn={_drawn}")

    # ---- 11. t28③ 靶坐标更改与方向设定（只内存，不落盘；≥2 组对照在交付说明） ----
    wb_tgt = build_world_b("K9", "K4")
    wb_t2 = apply_target_override(wb_tgt, ("H11", "I11"), 0.0)
    check("t28③ 覆盖后 truth.targets/track_cells = H11/I11",
          wb_t2.truth.targets == ("H11", "I11")
          and wb_t2.target.track_cells == ("H11", "I11"), str(wb_t2.truth.targets))
    check("t28③ 原情景不动（深拷贝隔离）", wb_tgt.truth.targets == ("B11", "C11")
          and wb_tgt.target.orientation_deg == -90.0, str(wb_tgt.truth.targets))
    check("t28③ 朝向覆盖生效（φ=0）", wb_t2.target.orientation_deg == 0.0,
          str(wb_t2.target.orientation_deg))
    check("t28③ 停站候选按新靶格重推（行 10，G~J 列）",
          set(wb_t2.target.stop_candidates) == {"G10", "H10", "I10", "J10"},
          " ".join(wb_t2.target.stop_candidates))
    out_t2 = evaluate_shooting(wb_t2.target, "H9", set(wb_t2.target.stop_candidates))
    check("t28③ H11/I11 φ=0 → 自动选行 10 站、蓝面命中",
          out_t2.stop in ("H10", "I10") and out_t2.blue_hits > 0 and out_t2.red_hits == 0,
          f"{out_t2.stop} 蓝{out_t2.blue_hits} 红{out_t2.red_hits}")
    raised = False
    try:
        apply_target_override(wb_tgt, ("B11", "B12"), None)
    except ValueError:
        raised = True
    check("t28③ 非法靶格（B12 越界）抛 ValueError", raised, "ValueError")
    raised2 = False
    try:
        apply_target_override(wb_tgt, ("B11", "D11"), None)
    except ValueError:
        raised2 = True
    check("t28③ 不相邻两格抛 ValueError", raised2, "ValueError")
    raised3 = False
    try:
        apply_target_override(wb_tgt, ("F11", "G11"), None)
    except ValueError:
        raised3 = True
    check("t28③ 靶格与障碍冲突（G11=障）抛 ValueError", raised3, "ValueError")
    # 纵向对（同列相邻行）停站候选推导 + 面板振荡轴切换（t28 shoot.py 泛化）
    wb_v = apply_target_override(wb_tgt, ("F10", "F11"), 0.0)
    check("t28③ 纵向对 F10/F11 候选落在朝中心的邻列 G（行 9~11）",
          set(wb_v.target.stop_candidates) == {"G9", "G10", "G11"},
          " ".join(wb_v.target.stop_candidates))
    ys_v = [panel_center(t / 10.0, wb_v.target) for t in range(0, 113)]
    check("t28③ 纵向轨道面板沿 x 振荡（y 恒；行程 = 0.7−0.14 = 0.56 m）",
          abs(max(p[0] for p in ys_v) - min(p[0] for p in ys_v) - 0.56) < 1e-6
          and max(abs(p[1] - ys_v[0][1]) for p in ys_v) < 1e-9,
          f"x 行程 {max(p[0] for p in ys_v) - min(p[0] for p in ys_v):.3f}")

    # ---- 12. t28④ 随机地图批量：结构 + 确定性（复跑逐位一致） ----
    d_b1 = tempfile.mkdtemp(prefix="ag_rm_")
    d_b2 = tempfile.mkdtemp(prefix="ag_rm_")
    st1 = run_random_maps(cfg, baseline, d_b1, n=5, seed_base=700)
    st2 = run_random_maps(cfg, baseline, d_b2, n=5, seed_base=700)
    check("t28④ N=5 复跑统计逐位一致（除墙钟）",
          st1["n_total"] == st2["n_total"] == 5
          and all({k: r1[k] for k in ("seed", "entrance", "exit_cell", "phi", "completed",
                                      "drive_s", "total_s", "traj_sha256")}
                  == {k: r2[k] for k in ("seed", "entrance", "exit_cell", "phi", "completed",
                                         "drive_s", "total_s", "traj_sha256")}
                  for r1, r2 in zip(st1["rows"], st2["rows"])),
          f"avg_plan {st1['avg_plan_s']:.3f}s")
    check("t28④ 有牙：随机 φ 覆盖 ≥2 种朝向（seed_base=700 实测 4 种）",
          len({r["phi"] for r in st1["rows"]}) >= 2,
          str(sorted({r["phi"] for r in st1["rows"]})))
    check("t28④ 统计字段齐（分量级 + 失败/超时单列键存在）",
          all(k in st1 for k in ("avg_plan_s", "avg_finish_s", "avg_drive_s", "avg_wait_s",
                                 "avg_dwell_s", "abort_rows", "over_budget_rows", "rows")),
          str(sorted(st1)))
    pngs_b = sorted(os.path.join(d_b1, d, f) for d in os.listdir(d_b1)
                    if os.path.isdir(os.path.join(d_b1, d))
                    for f in os.listdir(os.path.join(d_b1, d)) if f.endswith(".png"))
    check("t28④ 每张图恰好 1 张 route_map.png（N=5 → 5 张）",
          len(pngs_b) == 5 and all(f.endswith("route_map.png") for f in pngs_b),
          str(len(pngs_b)))
    ok_blue = [r for r in st1["rows"] if r["blue"] > 0]
    check("t28④ 抽样场存在蓝面命中（随机 φ 含 0° 且权威门+等待全完赛）",
          len(ok_blue) >= 1, f"{len(ok_blue)}/5")

    # ---- 13. t28 回归：臂级 A2 基线不变（绘图/位姿改动不得改轨迹语义） ----
    d_a2 = tempfile.mkdtemp(prefix="ag_a2_")
    arm_a2 = run_arm("A2", wa, cfg, baseline, drone_on=True, stop_on_violation=False,
                     out_dir=d_a2)
    check("t28 回归 A2：完赛 104.54±1 s / 盖雷 7/8 / 障碰 0 / 越界 0 / 图恰好 1 张 PNG",
          arm_a2.car["completed"] and abs(arm_a2.car["time_s"] - 104.54) < 1.0
          and arm_a2.field_metrics.get("mine_pressed_count") == 7
          and arm_a2.field_metrics.get("mine_total") == 8
          and arm_a2.field_metrics.get("obstacle_hit_steps") == 0
          and arm_a2.car["out_of_bounds_steps"] == 0
          and len([f for f in os.listdir(d_a2) if f.endswith(".png")]) == 1
          and os.path.exists(os.path.join(d_a2, "route_map.png")),
          f"{arm_a2.car['time_s']:.2f}s 盖雷 {arm_a2.field_metrics.get('mine_pressed_count')}/"
          f"{arm_a2.field_metrics.get('mine_total')}")

    # ---- 汇总 ----
    fails = [c for c in _CHECKS if not c[1]]
    for name, ok, detail in _CHECKS:
        print(f"  [{'OK' if ok else 'FAIL'}] {name}" + (f"  —— {detail}" if detail else ""))
    print(f"自检：{len(_CHECKS) - len(fails)}/{len(_CHECKS)} 通过")
    return not fails


if __name__ == "__main__":      # pragma: no cover
    sys.exit(0 if main() else 1)
