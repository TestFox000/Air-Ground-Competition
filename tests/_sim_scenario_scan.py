"""场景筛选工具：批量跑完整 SIL 仿真，找能稳定到达终点的雷位组合。

用法：python tests/_sim_scenario_scan.py
（调试/选型用，不是 pytest 测试；文件名不以 test_ 开头不会被收集）
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "src" / "ag_integrate"))

import ros_stub  # noqa: E402
from sim_car import TrackingWorld  # noqa: E402
from ag_integrate.integrate_node import IntegrateNode  # noqa: E402
from ag_integrate.intel_protocol import IntelObject, ObjectType  # noqa: E402

LETTERS = "ABCDEFGHIJKLMN"


def obj(cell, typ):
    col = LETTERS.index(cell[0])
    row = int(cell[1:]) - 1
    return IntelObject(typ, 0.065 + col * 0.35, 0.295 + (10 - row) * 0.35, 0.3, 0.3)


def run_scenario(mines, seed=42, budget=90.0):
    objs = tuple(obj(c, ObjectType.PAPER) for c in mines) + (
        obj("D8", ObjectType.TARGET_BLUE),)
    w = TrackingWorld(start_cell="N10", true_yaw_deg=158.6,
                      theta_boot_deg=90.0, seed=seed)
    ros_stub.set_overrides(
        udp_port=0, dry_run=False, publish_reset=True,
        start_cell="N10", goal_cell="N11", start_heading="y", final_heading="x",
        heading_bias_source="align", target_track=["D8", "E8"],
        target_blue_face="S", matrix_log=False)
    ig = IntegrateNode()
    w.attach_integrate(ig, objs)
    try:
        if not w.run_until(lambda: ig._align_state == "aligned", 15.0):
            return ("ALIGN_FAIL", w)
        w.start_car_stack()
        ok = w.run_until(lambda: w.pe.arrived, budget)
        return ("ARRIVED" if ok else "STUCK", w)
    finally:
        route = [m.data for m in ig.topic("/nav/waypoint").published]
        w.route = route
        w.close()
        ros_stub.clear_overrides()


if __name__ == "__main__":
    candidates = [
        ("J7", "G6"), ("J9", "F10"), ("K9", "G9"), ("J8", "G10"),
        ("K10", "H10"), ("J10", "F9"), ("K8", "H9"), ("J9", "G8"),
        ("K9", "H8"), ("J11", "G10"), ("K10", "G9"), ("J10", "H9"),
        ("K9", "G10"), ("J8", "H10"),
    ]
    for mines in candidates:
        status, w = run_scenario(mines)
        pe = w.pe
        extra = ""
        if status == "STUCK" and pe is not None:
            extra = (f" 卡在信念({pe.current_x:.2f},{pe.current_y:.2f}) "
                     f"seg={pe.current_target_idx}/{len(pe.path_points)}")
        print(f"{status:10s} {mines} align_t={w.align_done_t} "
              f"arr_t={w.arrived_t} 距终点={w.dist_to_goal():.2f}m"
              f" 净转角={math.degrees(w.chassis.yaw_traveled):.0f}°{extra}")
        if status == "ARRIVED":
            print("    route:", w.route[1] if len(w.route) > 1 else None)
