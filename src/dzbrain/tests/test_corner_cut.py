# -*- coding: utf-8 -*-
r"""既有拐角测试树：七条保护规则、现役 tour 与连续曲率轨迹。

D-023 已裁决旧 tracker-safe / 第五图 44→39 点快照过期。本文件保留
七条逐项硬规则，改用仓内第七图回归夹具和现役 tour；连续轨迹检查也
落在本文件，不能用新文件的绿色替代旧物理失败。

完整运行（含 8/8 雷覆盖、车体/挡板/边界等闭环硬验收）：
    python src\dzbrain\tests\test_corner_cut.py
仅几何开发检查（不是完整验收）：
    python src\dzbrain\tests\test_corner_cut.py --geometry-only
"""

from __future__ import annotations

import pathlib
import math
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.stdout.reconfigure(errors="replace")

import yaml                                                       # noqa: E402

from dzbrain.core import geometry as geo                          # noqa: E402
from dzbrain.core.config import Config                            # noqa: E402
from dzbrain.route import parser as P                             # noqa: E402
from dzbrain.route import planner as PL                           # noqa: E402
from dzbrain.route import validate as V                           # noqa: E402
from dzbrain.tools import sim as S                                # noqa: E402
from harness import check, section, finish, run_case, check_raises                        # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]
CFG_PATH = WS / "config" / "robot.yaml"
MAP_PATH = pathlib.Path(__file__).resolve().parent / "fixtures" / "map_carloop_20261024.yaml"

MAP = yaml.safe_load(MAP_PATH.read_text(encoding="utf-8"))
SPEC = PL.load_map(MAP, source=str(MAP_PATH))
LIVE = [str(t) for t in (Config.load(str(CFG_PATH)).get("route.tokens") or [])]
BAR = {"barrier_row_hi": 6.0, "barrier_col_from": "D"}


def _cfg(**over):
    """主配置的**独立副本**（`Config.override` 是就地改，必须每次重新 load）。"""
    cfg = Config.load(str(CFG_PATH))
    for k, v in over.items():
        cfg.override(k, v)
    return cfg


def _cells(blocks):
    return [(geo.block_to_index(b)[0] + 1, geo.block_to_index(b)[1]) for b in blocks]


def _map(mines=(), obstacles=(), targets=(), bridges=("K5", "K9"), start="N10", end="N5"):
    return {"map": {"name": "cut-test", "start": start, "end": end,
                    "mines": list(mines), "obstacles": list(obstacles),
                    "targets": list(targets), "bridges": list(bridges),
                    "barrier_col_from": "D", "barrier_row_hi": 6, "barrier_row_lo": 7}}


def _cut(blocks, *, mines=(), obstacles=(), targets=(), bridges=("K5", "K9"),
         stop=(), mine_clear=0.15):
    spec = PL.load_map(_map(mines, obstacles, targets, bridges), source="<cut-test>")
    kept, dropped, notes = PL.corner_cut_cells(
        _cells(blocks), spec, mine_clear_m=mine_clear,
        stop_cells=[_cells([b])[0] for b in stop], crossings=(), **BAR)
    return ([geo.index_to_block(r - 1, c) for r, c in kept],
            [geo.index_to_block(r - 1, c) for r, c in dropped], notes)


# --------------------------------------------------------------------------- #
#  1. 七条硬规则
# --------------------------------------------------------------------------- #
def test_rules() -> None:
    section("1. 七条硬规则逐条（合成直角 A4→A3→B3，拐角格 = A3）")
    base = ["A4", "A3", "B3"]

    kept, dropped, notes = _cut(base)
    check("1.0 基准：空旷场上直角顶点被切（A3 删除 → 2 点）",
          dropped == ["A3"] and kept == ["A4", "B3"], f"{kept} / {dropped} / {notes}")

    kept, dropped, notes = _cut(base, mines=("A3",))
    check("1.1 ①拐角格是雷 ⇒ 不切",
          dropped == [] and any("①" in n for n in notes), f"{dropped} / {notes}")

    kept, dropped, notes = _cut(base, mines=("A4",))
    check("1.2 ④雷三连（prev 是雷）⇒ 不切",
          dropped == [] and any("④" in n for n in notes), f"{dropped} / {notes}")

    kept, dropped, notes = _cut(base, mines=("B3",))
    check("1.3 ④雷三连（next 是雷）⇒ 不切",
          dropped == [] and any("④" in n for n in notes), f"{dropped} / {notes}")

    # ② 用真实几何 J9→J10→I10（弦到雷 H10 实测 0.350 m）：阈值抬到 0.40 必触发，
    #    生产阈值 0.15 不触发 —— 如实记录"②在当前格心几何下是非绑定护栏"。
    kept, dropped, notes = _cut(["J9", "J10", "I10"], mines=("H10",), mine_clear=0.40)
    check("1.4a ②弦离雷 < 阈值 ⇒ 不切（阈值 0.40 触发；实测弦-雷 = 0.350）",
          dropped == [] and any("②" in n for n in notes), f"{dropped} / {notes}")
    kept, dropped, notes = _cut(["J9", "J10", "I10"], mines=("H10",))
    check("1.4b 生产阈值 0.15 下同一条不触发（②目前非绑定，护栏仍在）",
          dropped == ["J10"], f"{dropped} / {notes}")

    kept, dropped, notes = _cut(["A2", "B2", "B1"], obstacles=("A1",))
    check("1.5 ③切后离障碍更近 ⇒ 不切（障 A1：切前 0.350 → 切后 0.247）",
          dropped == [] and any("③障碍更近" in n for n in notes), f"{dropped} / {notes}")

    kept, dropped, notes = _cut(["J9", "K9", "K10"], bridges=("K9",))
    check("1.6a ⑤拐角格本身就是桥洞格 ⇒ 不切",
          dropped == [] and any("⑤桥洞格" in n for n in notes), f"{dropped} / {notes}")

    kept, dropped, notes = _cut(["L10", "L9", "K9"], bridges=("K9",))
    check("1.6b ⑤拐角格紧邻桥洞三连（next 是 K9）⇒ 不切",
          dropped == [] and any("⑤" in n for n in notes), f"{dropped} / {notes}")

    kept, dropped, notes = _cut(base, stop=("A3",))
    check("1.7 ⑦停站/指令格 ⇒ 不切",
          dropped == [] and any("⑦" in n for n in notes), f"{dropped} / {notes}")

    kept, dropped, notes = _cut(["A5", "A4", "B3"])
    check("1.8 ⑥非直角（45° 拐角）⇒ 不参与（连原因都不记）",
          dropped == [] and notes == [], f"{dropped} / {notes}")

    kept, dropped, notes = _cut(["A5", "A4", "A3", "B3", "B2"])
    check("1.9 相邻两个直角不同时切（索引差 ≤1 只切第一个）",
          dropped == ["A3"], f"{dropped} / {notes}")


# --------------------------------------------------------------------------- #
#  2. 开关默认关闭 + 冻结路线不变
# --------------------------------------------------------------------------- #
def _plan(cfg):
    # D-023: tour is the authoritative engine. The old tracker-safe + 44-token
    # fifth-map snapshot was already a known crash, not a valid acceptance input.
    return PL.plan_route(MAP, cfg, cross_lower="K10", cross_upper="K4",
                         allow_reverse=False, max_seconds=10.0)


def test_default_off() -> None:
    section("2. 切角默认关闭；现役 tour 的配置输入保持不变")
    check("2.1 cfg=None 时关闭", PL.corner_cut_enabled(None) is False)
    cfg = _cfg()
    check("2.2 robot 未开启角点删减", PL.corner_cut_enabled(cfg) is False)
    off = _plan(cfg)
    explicit = _plan(_cfg(**{"planner.corner_cut": False}))
    check("2.3 缺省与显式关闭逐 token 一致", off.tokens == explicit.tokens)
    check("2.4 未执行角点删减", not any("拐角条件切角" in w for w in off.warnings))
    check("2.5 真实双停站保留", off.shoot_blocks() == ["F9", "C5"])


def test_cut_on() -> None:
    section("3. 角点删减函数的真实产物（不把 tour 中未接线的开关当成生效）")
    cfg = _cfg(**{"planner.corner_cut": True})
    check("3.1 显式开启被读到", PL.corner_cut_enabled(cfg) is True)
    kept, dropped, notes = _cut(["A4", "A3", "B3"])
    check("3.2 无保护的90度点被删，端点保序", kept == ["A4", "B3"] and dropped == ["A3"])
    kept, dropped, notes = _cut(["A4", "A3", "B3"], mines=("A3",), stop=("A3",))
    check("3.3 雷/停站必经点不可删", kept == ["A4", "A3", "B3"] and dropped == [])
    route = _plan(_cfg())
    issues = V.validate_route(P.parse_mission(route.tokens), map_cfg=MAP, cfg=_cfg())
    check("3.4 现役 tour 原路线 validate 0 error", not [i for i in issues if i.severity == "error"],
          V.summary(issues)[:150])


def test_determinism() -> None:
    section("4. 现役 tour 确定性")
    a, b = _plan(_cfg()), _plan(_cfg())
    check("4.1 相同地图相同 token", a.tokens == b.tokens)
    check("4.2 相同地图相同 warning", a.warnings == b.warnings)


def test_continuous_geometry() -> None:
    from dzbrain.pursuit.trajectory import Trajectory, TrajectoryInfeasible
    from dzbrain.route.model import Route, Waypoint
    section("6. 连续几何、接缝、cusp、R恢复与真实拒绝")
    cfg = _cfg()
    def make(coords, flags=None, yaw=0.0):
        flags = flags or [False] * len(coords)
        return Route(tuple(Waypoint("", x, y, yaw, reverse=rev)
                           for (x, y), rev in zip(coords, flags)))
    cases = [
        ("straight", make([(0.,0.),(2.,0.)])),
        ("wide90", make([(0.,0.),(1.,0.),(1.,1.)],yaw=math.pi/2)),
        ("short90", make([(0.,0.),(.35,0.),(.35,.35)],yaw=math.pi/2)),
        ("implicit cusp", make([(0.,0.),(.35,0.),(0.,0.)])),
        ("explicit R then forward", make([(0.,0.),(-.35,0.),(0.,0.)],[False,True,False])),
        ("continuous R", make([(0.,0.),(-.35,0.),(-.70,0.)],[False,True,True])),
        ("auto cusp R forward",make([(0.,0.),(.35,0.),(0.,0.),(-.35,0.),(0.,0.)],
                                    [False,False,False,True,False])),
    ]
    built = {}
    for name, route in cases:
        t = Trajectory(route,cfg)
        built[name]=t
        samples=t.samples()
        check(f"6 {name}: 解析κ有界",max(abs(q.curvature) for q in samples)<=t.max_curvature+1e-9)
        check(f"6 {name}: mandatory位置精确",max(math.hypot(t.sample(t.s_at_waypoint(i)).x-w.x,
              t.sample(t.s_at_waypoint(i)).y-w.y) for i,w in enumerate(route.waypoints))<1e-8)
        check(f"6 {name}: 终点车身yaw",abs(geo.normalize_angle(t.sample(t.total_length).body_yaw-route.end.yaw))<1e-8)
        seam_error=0.;heading_error=0.;steer_error=0.;bad_cusp=False
        for left,right in zip(t._pieces,t._pieces[1:]):
            x0,y0,h0,k0=left.at(left.length);x1,y1,h1,k1=right.at(0.)
            seam_error=max(seam_error,math.hypot(x1-x0,y1-y0))
            bh0=h0+(math.pi if left.reverse else 0.);bh1=h1+(math.pi if right.reverse else 0.)
            heading_error=max(heading_error,abs(geo.normalize_angle(bh1-bh0)))
            steer_error=max(steer_error,abs((-k0 if left.reverse else k0)-(-k1 if right.reverse else k1)))
            if left.reverse!=right.reverse:bad_cusp|=abs(k0)>1e-8 or abs(k1)>1e-8
        check(f"6 {name}: 每接缝位置/车头/转向κ连续",seam_error<1e-8 and heading_error<1e-8 and steer_error<1e-8,
              f"position={seam_error:.3g} yaw={heading_error:.3g} k={steer_error:.3g}")
        check(f"6 {name}: cusp两侧κ=0且都入stop_s",not bad_cusp and set(t.cusp_s)<=set(t.stop_s))
        projected=t.project(samples[len(samples)//2].x,samples[len(samples)//2].y,
                            s_hint=samples[len(samples)//2].s,allow_backward=False)
        check(f"6 {name}: 投影弧长地板",projected.s>=samples[len(samples)//2].s-1e-10)
    t=built["explicit R then forward"]
    check("6 R后非R恢复前进",t.sample(.10).reverse and not t.sample(t.total_length-.10).reverse)
    check("6 连续R不擅自恢复前进",all(q.reverse for q in built["continuous R"].samples()))
    t=built["auto cusp R forward"]
    check("6 推断cusp遇显式R后非R仍恢复前进",not t.sample(t.total_length-.10).reverse)
    check("6 原路折返1个cusp且长度0.7m",len(built["implicit cusp"].cusp_s)==1
          and abs(built["implicit cusp"].total_length-.7)<1e-9)
    overlap=built["implicit cusp"]
    bounded=overlap.project(.2,0.,s_hint=.3,motion_heading=math.pi,s_max=.35)
    check("6 重叠cusp投影只搜索停车前分支（不是事后夹s）",
          abs(bounded.s-.2)<1e-8 and abs(bounded.x-.2)<1e-8 and bounded.dist<1e-8)
    partial=overlap.project(.3,0.,s_max=.203)
    check("6 投影上界在采样边内部时真实截断",abs(partial.s-.203)<1e-9
          and abs(partial.x-.203)<1e-9 and not partial.passed_end)
    mine_route=Route((Waypoint("",0.,0.,0.),Waypoint("mine",1.,0.,0.),Waypoint("",1.,1.,math.pi/2)))
    mine_curve=Trajectory(mine_route,cfg,{"mines":["mine"]})
    mine_pose=mine_curve.sample(mine_curve.s_at_waypoint(1));offset=cfg.get("vehicle.ref_offset_m")
    check("6 雷航点约束真实车心而非后轴",math.hypot(
        mine_pose.x+offset*math.cos(mine_pose.body_yaw)-1.,
        mine_pose.y+offset*math.sin(mine_pose.body_yaw))<1e-8)
    check("6 雷位姿平移后下一连接仍位置连续",all(
        math.hypot(a.at(a.length)[0]-b.at(0.)[0],a.at(a.length)[1]-b.at(0.)[1])<1e-9
        for a,b in zip(mine_curve._pieces,mine_curve._pieces[1:])))
    initial=Trajectory(cases[0][1],cfg,start_yaw=math.pi/2)
    check("6 显式真实初始车头不被首段方向覆盖",abs(geo.normalize_angle(
        initial.sample(0.).body_yaw-math.pi/2))<1e-9)
    check_raises("6 不偷偷平移mandatory首点",TrajectoryInfeasible,Trajectory,cases[0][1],cfg,
                 start_pose=geo.Pose(.1,0.,0.))
    check_raises("6 无可行区域明确拒绝（不得回退折线）",TrajectoryInfeasible,Trajectory,
                 cases[0][1],cfg,pose_valid=lambda x,y,yaw:False,max_nodes=1)
    limited_cfg=_cfg(**{"pursuit.max_steer_rate_deg_s":10.0})
    limited=Trajectory(cases[1][1],limited_cfg)
    rate=math.radians(10.0);wheelbase=limited_cfg.get("pursuit.wheelbase_m")
    values=limited.samples()
    check("6 转向速率进入速度包络",all(
        abs(wheelbase*q.curvature_rate/(1+(wheelbase*q.curvature)**2))*q.v_max<=rate+1e-10
        for q in values) and min(q.v_max for q in values)<limited_cfg.get("pursuit.cruise_speed_mps"))
    derivative_errors=[]
    for start,piece in zip(limited._piece_start,limited._pieces):
        for ratio in (.2,.5,.8):
            s=start+piece.length*ratio;eps=1e-5
            numeric=(limited.sample(s+eps).curvature-limited.sample(s-eps).curvature)/(2*eps)
            derivative_errors.append(abs(numeric-limited.sample(s).curvature_rate))
    check("6 κ_s解析值与独立差分一致",max(derivative_errors)<1e-4,
          f"max error={max(derivative_errors):.3g}")


def test_bridge_axis() -> None:
    from dzbrain.pursuit.trajectory import Trajectory
    section("7. 桥洞前中后同一条直线且车头不摆")
    raw=_map(mines=("L9",),bridges=("K9",))
    route=P.parse_mission(["M9","L9","K9","J9","I9"])
    route=route.__class__(route.waypoints[:-1]+(route.waypoints[-1].with_yaw(math.pi/2),),route.name,route.crossing)
    t=Trajectory(route,_cfg(),raw)
    lo=t.s_at_waypoint(1);hi=t.s_at_waypoint(3)
    values=[t.sample(lo+(hi-lo)*j/100) for j in range(100)]
    check("7 进洞到出洞κ恒0",max(abs(q.curvature) for q in values)<1e-9)
    check("7 车头始终沿洞轴",all(abs(geo.normalize_angle(q.body_yaw-math.pi/2))<1e-9 for q in values))
    check("7 三格中心线不偏",max(abs(q.x-route.waypoints[2].x) for q in values)<1e-9)
    q=t.sample(t.s_at_waypoint(1));offset=_cfg().get("vehicle.ref_offset_m")
    check("7 洞前邻格雷事件按车心映射且保护直线保留",math.hypot(
        q.x+offset*math.cos(q.body_yaw)-route.waypoints[1].x,
        q.y+offset*math.sin(q.body_yaw)-route.waypoints[1].y)<1e-9)


# --------------------------------------------------------------------------- #
#  5. 同输入闭环：原折线纯追踪 vs 连续轨迹
# --------------------------------------------------------------------------- #
def test_sim_compare() -> None:
    from dzbrain.pursuit.trajectory import Trajectory
    sys.path.insert(0,str(WS / "tools"))
    from route_check import TrajectoryFieldGate, compare_simulation
    section("5. 真实闭环：车心雷8/8 / 全区域无违规 / 完成后RMS优于原折线")
    field = S.load_field_spec(str(MAP_PATH))
    cfg = _cfg()
    route = P.parse_mission(_plan(cfg).tokens)
    gate = TrajectoryFieldGate(field,cfg)
    initial_yaw=(S.DESIGN_START_YAW if route.start.block.strip().upper() == S.DESIGN_START_BLOCK
                 else route.start.heading_to(route.waypoints[1]))
    trajectory = Trajectory(route,cfg,field,pose_valid=gate,start_yaw=initial_yaw)
    comparison = compare_simulation(route,cfg,field,gate,trajectory,max_steps=20000)
    old,new = (comparison["arms"][key] for key in ("polyline","trajectory"))
    mo,mn = (arm["native_simulator_metrics"] for arm in (old,new))
    observed = new["observed_field"]
    violations = observed["sampled_region_violations"]
    accepted = comparison["candidate_acceptance"]["checks"]
    check("5.1 两臂均完成，RMS才可比较",comparison["rms_comparison_eligible"],
          f"legacy={old['completed']} candidate={new['completed']}")
    check("5.2 候选相对新轨迹的横向RMS优于旧折线",
          accepted["rms_improved"],
          f"{old['cross_track_rms_m']:.6f} -> {new['cross_track_rms_m']:.6f} m")
    check("5.3 真实车心严格3.5cm压雷8/8",accepted["body_center_mines"] and observed["mine_total"] == 8,
          str(observed["body_center_mine_sample_distances_m"]))
    check("5.4 真车体不压靶板0拍",violations["target_surface_m"] == 0,
          str(observed["sampled_clearances_m"]["target_surface_m"]))
    check("5.5 障碍表面净空达标/挡板0/越界0/K封死0/洞横偏达标",
          accepted["sampled_physical_regions"],str(violations))
    check("5.6 全程完成且最终输出转向角/速率受限",accepted["completed"]
          and accepted["final_command_steer"] and accepted["final_command_steer_rate"],
          f"completed={new['completed']} max_rate={mn['max_steer_rate_deg_s']:.6f}")
    check("5.7 审计期间实现文件未变化",comparison["source_files_unchanged_during_run"])
    check("5.8 纵加减速/jerk受限且零约束越界",all(accepted[key] for key in
          ("accel_up","accel_down","jerk","no_constraint_override")),str(accepted))
    check("5.9 含射击停站总预算180s内",accepted["mission_time_within_180s"],
          str(new["mission_time_budget"]))
    # The old vertex distance is a diagnostic, not a safety threshold for a new
    # curve allowed to leave ordinary grid centres. Field geometry uses one gate,
    # including rear-reference -> actual-body-centre conversion, in both arms.
    print(f"     ordinary-vertex diff {mo['vertex_overshoot_normal_m']:.6f} -> "
          f"{mn['vertex_overshoot_normal_m']:.6f} m; travel time {old['time_s']:.2f} -> "
          f"{new['time_s']:.2f} s; mission budget = {new['mission_time_budget']}")


def main() -> int:
    print("=" * 78)
    print("TRAJ-GEOM · 拐角条件切角 + 连续轨迹（原有物理硬判据仍执行）")
    print("=" * 78)
    test_rules()
    run_case("continuous geometry",test_continuous_geometry)
    run_case("bridge axis",test_bridge_axis)
    if "--geometry-only" not in sys.argv:
        run_case("default off",test_default_off)
        run_case("cut function",test_cut_on)
        run_case("tour determinism",test_determinism)
        run_case("physical closed loop",test_sim_compare)
    finish()
    return 0


if __name__ == "__main__":
    sys.exit(main())
