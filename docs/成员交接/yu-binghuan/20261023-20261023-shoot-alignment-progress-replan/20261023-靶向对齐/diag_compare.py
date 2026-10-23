# -*- coding: utf-8 -*-
"""对照诊断：同一场景分别在 baseline / work 两套代码上跑，定位差异来源。"""
import sys
import warnings

ROOT = r"c:\Users\29212\Documents\Trae-CN\default\20261023-靶向对齐"
sys.path.insert(0, ROOT)          # 让 "baseline"/"work" 可作为包父目录

def run(tag):
    # 强制重新加载指定副本
    for m in [k for k in sys.modules if k.startswith("ag_integrate")]:
        del sys.modules[m]
    if tag == "baseline":
        sys.path.insert(0, ROOT + r"\baseline")
    else:
        sys.path.insert(0, ROOT + r"\work")
    try:
        from ag_integrate.field_frame import CellRef
        from ag_integrate.world_model import TargetRef, WorldSnapshot
        from ag_integrate.mission_router import plan_route

        LETTERS = "ABCDEFGHIJKLMN"
        def cell(n):
            return CellRef(LETTERS.index(n[0]), int(n[1:]) - 1)
        def snap(mines=(), passable=("K4", "K9"), obstacles=(), targets=()):
            return WorldSnapshot(0, tuple(cell(c) for c in mines),
                                 tuple(cell(c) for c in passable),
                                 tuple(cell(c) for c in obstacles),
                                 tuple(targets))
        track = (cell("D7"), cell("E7"))
        t = TargetRef(cell("D7"), True, track=track, blue_face="S")

        cases = {
            "moving@D7 无障碍 N10->N5": snap(mines=("E3", "I2"), targets=(t,)),
            "moving@D7 堵D9E9 N10->N5": snap(mines=("E3", "I2"), obstacles=("D9", "E9"), targets=(t,)),
            "无靶 N10->N5": snap(mines=("E3", "I2")),
            "无靶 N11->N6": snap(mines=("E3", "I2")),
        }
        print(f"===== {tag} =====")
        for name, s in cases.items():
            start, goal = ("N10", "N5") if "N10" in name else ("N11", "N6")
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                try:
                    r = plan_route(s, cell(start), cell(goal),
                                   start_heading="y", final_heading="y")
                    wtxt = [str(w.message)[:40] for w in caught
                            if "正对朝向" in str(w.message)]
                    print(f"[OK ] {name}: {len(r)} 格 | 降级告警={wtxt}")
                except ValueError as e:
                    print(f"[ERR] {name}: {str(e)[:90]}")
    finally:
        sys.path.remove(ROOT + ("\\" + tag))

run("baseline")
run("work")
