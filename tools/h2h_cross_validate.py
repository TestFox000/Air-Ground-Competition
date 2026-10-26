# -*- coding: utf-8 -*-
"""头对头成果全量交叉验证（2026-10-26，[H2H-XVAL-20261026]）。

四个方向，全部对着 `air_ground_car`（车端官方代码）与 `_h2h_20261026`：

① 正向全量：我方每条解出路线（严格 34 + 宽松 48，重复图各验各的）→
   `route_command` 完整编码（`*` 射击点 + `:PIVOT` 折返声明）→
   车端官方 `route_check` 六层校验；逐层统计 ❌/⚠️，任何 ❌ 都点名。
② 反向模型等价：对方 tour 解出路线里的 shoot_stops 是否全部落在我方
   0.78~1.02 m 环带定义内（对某轨端偏移 ∈ {√5, √8}）——两边停站
   语义是否同构。
③ 停站视线（LOS）审计：我方停靠格 → 被服务轨端的连线走廊里，若有
   障碍格心距线段 < 0.20 m（桶 0.35 m 方，光路近距擦边即视为可疑挡光），
   点名——车端校验器不查这条，但激光是光路，挡住就是白停一趟。
④ 结果落盘 `_h2h_20261026/cross_validate.json`，供回归比对。

用法：python tools/h2h_cross_validate.py
"""
from __future__ import annotations

import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
OURS_ROOT = os.path.dirname(HERE)
THEIR_WT = os.path.abspath(os.path.join(OURS_ROOT, "..", "air_ground_car"))
OUT_DIR = os.path.join(OURS_ROOT, "_h2h_20261026")
LETTERS = "ABCDEFGHIJKLMN"
CELL = 0.35


def _pos(name: str) -> tuple[float, float]:
    return (LETTERS.index(name[0]), int(name[1:]) - 1)


def _serves(stop: str, rail: str) -> bool:
    dc = LETTERS.index(stop[0]) - LETTERS.index(rail[0])
    dr = int(stop[1:]) - int(rail[1:])
    return dc * dc + dr * dr in (5, 8)


def _seg_point_dist(p, a, b) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    n2 = dx * dx + dy * dy
    if not n2:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / n2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def run_checker(route_cmd: str, map_path: str, cfg_dir: str) -> dict:
    shutil.copyfile(map_path, os.path.join(cfg_dir, "map.yaml"))
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1")
    p = subprocess.run(
        [sys.executable, "tools/route_check.py",
         "--config", os.path.join(cfg_dir, "robot.yaml"), "--route", route_cmd],
        cwd=THEIR_WT, capture_output=True, text=True,
        encoding="utf-8", timeout=90, env=env)
    out = p.stdout
    errs = [ln.strip() for ln in out.splitlines() if "❌" in ln]
    warns = [ln.strip() for ln in out.splitlines() if "⚠" in ln]
    bullets = []
    grab = False
    for ln in out.splitlines():
        if "项失败" in ln and "——" in ln:
            grab = True
            continue
        if grab:
            s = ln.strip()
            if s.startswith("·"):
                bullets.append(s)
            elif bullets:
                break
    return dict(ok=not errs, n_err=len(errs), n_warn=len(warns),
                errs=errs[:4], warns_summary=[w[:60] for w in warns[:3]],
                bullets=bullets, rc=p.returncode)


def main() -> None:
    sys.path.insert(0, os.path.join(OURS_ROOT, "src", "ag_integrate"))
    from ag_integrate.car_commands import RoutePlan, route_command

    data = json.load(open(os.path.join(OUT_DIR, "results.json"),
                          encoding="utf-8"))
    cfg_dir = os.path.join(THEIR_WT, "_h2h_xval_config")
    os.makedirs(cfg_dir, exist_ok=True)
    shutil.copyfile(os.path.join(THEIR_WT, "config", "robot.yaml"),
                    os.path.join(cfg_dir, "robot.yaml"))

    forward, reverse_bad, los_flags = [], [], []
    n_runs = 0
    for row in data["rows"]:
        idx = row["idx"]
        map_path = os.path.join(OUT_DIR, "maps", f"map_{idx:02d}.yaml")
        for side in ("ours", "ours_p"):
            res = row.get(side) or {}
            if not res.get("ok"):
                continue
            route = res["route"]
            stops = sorted({x for x in route
                            if any(_serves(x, t) for t in row["targets"])})
            cmd = route_command(RoutePlan(cells=tuple(route),
                                          shoot_at=frozenset(stops)))
            chk = run_checker(cmd, map_path, cfg_dir)
            n_runs += 1
            forward.append(dict(idx=idx, side=side, stops=stops,
                                tokens=len(route), **chk))
            flag = "ok" if chk["ok"] else "❌" + str(chk["n_err"])
            print(f"[{idx:02d}]{side:6s} {flag} warn={chk['n_warn']}",
                  flush=True)
            # ③ LOS：停靠格 → 每个被服务轨端
            import yaml
            ym = yaml.safe_load(open(map_path, encoding="utf-8"))["map"]
            for s in stops:
                sp = _pos(s)
                for rail in row["targets"]:
                    if not _serves(s, rail):
                        continue
                    rp = _pos(rail)
                    for ob in ym["obstacles"]:
                        d = _seg_point_dist(_pos(ob), sp, rp) * CELL
                        if d < 0.20:
                            los_flags.append(dict(
                                idx=idx, stop=s, rail=rail, obstacle=ob,
                                dist_m=round(d, 3)))
        # ② 反向：对方停站是否都在我方环带定义内
        for st in (row["tour"].get("shoot_stops") or []):
            if not any(_serves(st, t) for t in row["targets"]):
                reverse_bad.append(dict(idx=idx, stop=st,
                                        targets=row["targets"]))

    n_ok = sum(1 for f in forward if f["ok"])
    summary = {
        "forward_runs": n_runs,
        "forward_pass": n_ok,
        "forward_fail": n_runs - n_ok,
        "fail_detail": [dict(idx=f["idx"], side=f["side"],
                             bullets=f["bullets"], errs=f["errs"])
                        for f in forward if not f["ok"]],
        "warn_only": sum(1 for f in forward if f["ok"] and f["n_warn"]),
        "reverse_ring_mismatch": reverse_bad,
        "los_suspect": los_flags,
    }
    out = os.path.join(OUT_DIR, "cross_validate.json")
    json.dump(dict(summary=summary, forward=forward),
              open(out, "w", encoding="utf-8"), ensure_ascii=False,
              indent=1, default=str)
    shutil.rmtree(cfg_dir, ignore_errors=True)
    print("\n==== CROSS-VALIDATE ====")
    print(json.dumps(summary, ensure_ascii=False, indent=1)[:3000])


if __name__ == "__main__":
    main()
