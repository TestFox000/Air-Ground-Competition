# -*- coding: utf-8 -*-
"""G1-α A/B 路线对比（生产口径）：旧几何（拐点起步弧 35°）vs 新几何（切角弧 30°）。

口径对齐 video_field_recon.py 生产管线（2026-10-18 修正后真源）：
  - 官方基线：OFFICIAL_MINES/OBSTACLES/PASSABLE/TRACK_DFT、START=N10/GOAL=N5
  - 靶先验裁决：视频静靶落在移动靶轨道 ±1 格邻域 → 误检剔除，回退两段式（is_blue=None）
  - 靶落障碍格 → 剔除；障碍 = 官方 ∪ 视频保守并集；passable = 官方 ∪ 视频箭头
  - 可达性三级阶梯：L1 并集避障+转弯约束 → L2 官方避障+转弯约束 → L3 转弯放宽

对比判据：
  a) 同阶梯内路线长度/转弯数（新几何放行被误禁转向 → 更短或更优路线）
  b) 所需阶梯深度（新几何在更浅阶梯成功 = 强解结证据）
  c) 新几何 turn_violations 交叉复核旧版路线（执行层真值第二意见）
"""
from __future__ import annotations

import importlib.util
import json
import sys
from collections import Counter
from pathlib import Path

from importlib.machinery import SourceFileLoader

SRC = Path(r"c:\Users\29212\Documents\Trae-CN\default"
           r"\路径规划交接_20261017\src\ag_integrate")
sys.path.insert(0, str(SRC))

from ag_integrate import mission_router as mr_new          # noqa: E402
from ag_integrate.field_frame import CellRef, LETTERS       # noqa: E402
from ag_integrate.world_model import WorldSnapshot, TargetRef  # noqa: E402

BAK = SRC / "ag_integrate" / "mission_router.py.bak_20261018_turngeo"
_loader = SourceFileLoader("ag_integrate.mission_router_old", str(BAK))
spec = importlib.util.spec_from_loader("ag_integrate.mission_router_old",
                                       _loader)
mr_old = importlib.util.module_from_spec(spec)
sys.modules["ag_integrate.mission_router_old"] = mr_old
_loader.exec_module(mr_old)

# ---- 官方基线（video_field_recon.py 生产真源，2026-10-18 修正）----------
OFFICIAL_MINES = ["B1", "A2", "I2", "C3", "E3", "F5", "H8", "G9"]
OFFICIAL_OBSTACLES = ["D5", "I6", "B7", "F8", "C9", "E10", "G10", "A11"]
OFFICIAL_PASSABLE = ["K4", "K10"]
TRACK_DFT = ("D7", "E7")
START, GOAL = "N10", "N5"


def cell(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


def build_snapshot(fs: dict) -> dict:
    """复刻生产口径：返回各实体列表（规划输入）。"""
    tgt_blue = list(fs.get("targets_video", {}).get("blue", []))
    tgt_red = list(fs.get("targets_video", {}).get("red", []))
    # 靶先验裁决：移动靶轨道 ±1 邻域的静靶 = 误检
    tc_ = [cell(c) for c in TRACK_DFT]

    def _near(c):
        p = cell(c)
        return any(abs(p.col - t.col) <= 1 and abs(p.row - t.row) <= 1
                   for t in tc_)

    tgt_blue = [c for c in tgt_blue if not _near(c)]
    tgt_red = [c for c in tgt_red if not _near(c)]
    v_obst = list(fs.get("obstacles_video", []))
    v_pass = list(fs.get("passable_video", []))
    use_obstacles = sorted(set(OFFICIAL_OBSTACLES) | set(v_obst))
    use_passable = sorted(set(OFFICIAL_PASSABLE) | set(v_pass))
    tgt_dropped = [c for c in (tgt_blue + tgt_red) if c in use_obstacles]
    tgt_blue = [c for c in tgt_blue if c not in use_obstacles]
    tgt_red = [c for c in tgt_red if c not in use_obstacles]
    ts = ([TargetRef(cell(c), True) for c in tgt_blue]
          + [TargetRef(cell(c), False) for c in tgt_red])
    if not ts:
        ts = [TargetRef(cell(TRACK_DFT[0]), None,
                        track=(cell(TRACK_DFT[0]), cell(TRACK_DFT[1])),
                        blue_face=None)]
    return {
        "mines": list(OFFICIAL_MINES),
        "passable": use_passable,
        "obstacles": use_obstacles,
        "targets": ts,
        "dropped": tgt_dropped,
    }


def route_stats(route, mr):
    cells = [cell(n) for n in route]
    length = 0.0
    for a, b in zip(cells, cells[1:]):
        length += ((b.col - a.col) ** 2 + (b.row - a.row) ** 2) ** 0.5
    turns = []
    for i in range(1, len(cells) - 1):
        u1 = (cells[i].col - cells[i - 1].col,
              cells[i].row - cells[i - 1].row)
        u2 = (cells[i + 1].col - cells[i].col,
              cells[i + 1].row - cells[i].row)
        if u1 == u2:
            continue
        h1 = mr._DIRS.index(u1) if u1 in mr._DIRS else None
        h2 = mr._DIRS.index(u2) if u2 in mr._DIRS else None
        if h1 is None or h2 is None:
            turns.append(-1)
            continue
        turns.append(45 * min(abs(h1 - h2), 8 - abs(h1 - h2)))
    rset = set(route)
    return {
        "cells": len(route),
        "length": round(length, 2),
        "turn_deg": dict(sorted(Counter(turns).items())),
        "dup": len(route) - len(rset),
        "mines_covered": sum(1 for c in OFFICIAL_MINES if c in rset),
    }


def run_ladders(mr, m_list, p_l, o_l, tgts):
    """生产三级阶梯：返回 (阶梯名, snapshot, route) 或 (None, snap, err)。"""
    snap = WorldSnapshot(0, tuple(cell(c) for c in m_list),
                         tuple(cell(c) for c in p_l),
                         tuple(cell(c) for c in o_l), tuple(tgts))
    ladders = [
        ("L1 并集避障+转弯约束", p_l, o_l, True),
        ("L2 官方避障+转弯约束", OFFICIAL_PASSABLE, OFFICIAL_OBSTACLES, True),
        ("L3 官方避障+转弯放宽", OFFICIAL_PASSABLE, OFFICIAL_OBSTACLES, False),
    ]
    last_err = ""
    for name, pl, ol, use_tc in ladders:
        s = WorldSnapshot(0, tuple(cell(c) for c in m_list),
                          tuple(cell(c) for c in pl),
                          tuple(cell(c) for c in ol), tuple(tgts))
        tc = mr._turn_model_of(s) if use_tc else None
        try:
            route = mr.plan_route(s, cell(START), cell(GOAL),
                                  start_heading="y", final_heading="y",
                                  turn_clearance=tc)
            return name, s, list(route)
        except ValueError as e:
            last_err = str(e)[:110]
    return None, snap, last_err


def main():
    base = Path(r"c:\Users\29212\Documents\Trae-CN\default\检测召回AB\balanced")
    sessions = ["cam_20261017_205245", "cam_20261017_205411",
                "cam_20261017_205906", "cam_20261018_091746"]
    out = {}
    for s in sessions:
        f = base / s / "field_state.json"
        fs = json.loads(f.read_text(encoding="utf-8"))
        d = build_snapshot(fs)
        print(f"\n===== {s} =====")
        if d["dropped"]:
            print(f"  靶落障碍格剔除: {d['dropped']}")
        res = {}
        for name, mr in (("旧几何", mr_old), ("新几何", mr_new)):
            ladder, snap, route_or_err = run_ladders(
                mr, d["mines"], d["passable"], d["obstacles"], d["targets"])
            if ladder is None:
                print(f"  [{name}] 三级阶梯全失败: {route_or_err}")
                res[name] = {"ladder": None, "err": route_or_err}
                continue
            st = route_stats(route_or_err, mr)
            st["ladder"] = ladder
            st["_route"] = list(route_or_err)
            st["_snap"] = None
            # 新几何交叉复核（执行层真值第二意见）
            st["_cross"] = mr_new.turn_violations(tuple(route_or_err), snap)
            res[name] = st
            print(f"  [{name}] {ladder} | {st['cells']}格 直线{st['length']}格 "
                  f"转弯{st['turn_deg']} 重复{st['dup']} "
                  f"雷覆盖{st['mines_covered']}/8"
                  + (f" ⚠交叉复核撞: {st['_cross']}" if st["_cross"] else " ✓复核通过"))
        if res.get("旧几何", {}).get("ladder") and res.get("新几何", {}).get("ladder"):
            same = res["旧几何"]["_route"] == res["新几何"]["_route"]
            dl = round(res["旧几何"]["length"] - res["新几何"]["length"], 2)
            print(f"  ==> 路线{'相同' if same else '不同'}，"
                  f"长度 {res['旧几何']['length']} → {res['新几何']['length']}"
                  f"（新 {'短' if dl > 0 else '长'} {abs(dl)} 格）"
                  f"; 阶梯 {res['旧几何']['ladder'][:2]} → {res['新几何']['ladder'][:2]}")
        for k in res:
            res[k].pop("_route", None)
            res[k].pop("_cross", None)
        out[s] = res
    dst = Path(__file__).with_name("G1_AB_路线对比.json")
    dst.write_text(json.dumps(out, ensure_ascii=False, indent=1,
                              default=str), encoding="utf-8")
    print(f"\n已落盘 {dst.name}")


if __name__ == "__main__":
    main()
