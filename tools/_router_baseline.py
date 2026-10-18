"""规划器行为基线采集/比对：重构 _dijkstra 等内部实现前后各跑一次，输出必须逐字节一致。

用法：
    python tools/_router_baseline.py write   # 采集/更新基线（runs/router_baseline.json）
    python tools/_router_baseline.py check   # 与基线逐字节比对，不一致退出码 1
"""
import json
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src" / "ag_integrate"))

from ag_integrate.field_frame import CellRef, COLS, ROWS, LETTERS
from ag_integrate.world_model import WorldSnapshot, TargetRef
from ag_integrate.mission_router import plan_route


def cell(name):
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


def name(c):
    return f"{LETTERS[c.col]}{c.row + 1}"


def scenarios():
    # 1) 代表性赛局（桥洞 K5 + 移动靶 D8-E8 蓝南）
    # 注：D8-E8 为历史场景轨道——2026-10-17 官方地图已改 D7-E7，此处不改行为
    yield "demo", dict(
        mines=["I1", "A2", "G3", "B4", "H5", "E6", "C7", "G11"],
        obstacles=["C2", "F4", "D7", "H8", "A9", "G9", "B10", "J11"],
        passable=["K5"], track=("D8", "E8"), face="S", blue=True,
        start="N11", goal="N6", sh="y", fh="y",
    )
    # 2) 种子随机局：8 雷 + 8 障碍 + 随机桥洞箭头 + 随机靶
    rng = random.Random(20261014)
    for i in range(40):
        cells = [CellRef(c, r) for c in range(COLS) for r in range(ROWS)]
        rng.shuffle(cells)
        mines = cells[:8]
        obstacles = cells[8:16]
        arrow_col = 10                      # K 列
        passable = [CellRef(arrow_col, rng.choice([4, 5]))]
        # 随机移动靶：水平或垂直相邻两格，随机蓝面朝向
        a = cells[20]
        horiz = rng.random() < 0.5
        if horiz and a.col + 1 < COLS:
            b = CellRef(a.col + 1, a.row)
            face = rng.choice("SN")
        elif not horiz and a.row + 1 < ROWS:
            b = CellRef(a.col, a.row + 1)
            face = rng.choice("EW")
        else:
            b = CellRef(a.col, a.row)
            face = "S"
        yield f"rand{i}", dict(
            mines=[name(c) for c in mines],
            obstacles=[name(c) for c in obstacles],
            passable=[name(c) for c in passable],
            track=(name(a), name(b)), face=face, blue=rng.random() < 0.5,
            start="N11", goal="N6", sh="y", fh="y",
        )


def run(s):
    track = (cell(s["track"][0]), cell(s["track"][1]))
    snap = WorldSnapshot(
        0,
        tuple(cell(n) for n in s["mines"]),
        tuple(cell(n) for n in s["passable"]),
        tuple(cell(n) for n in s["obstacles"]),
        (TargetRef(track[0], s["blue"], track=track, blue_face=s["face"]),),
    )
    t0 = time.perf_counter()
    try:
        route = plan_route(snap, cell(s["start"]), cell(s["goal"]),
                           start_heading=s["sh"], final_heading=s["fh"])
        out = list(route)
    except ValueError as exc:
        out = ["ERROR", str(exc)]
    dt = time.perf_counter() - t0
    return out, dt


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "write"
    result = {}
    total_t = 0.0
    for tag, s in scenarios():
        out, dt = run(s)
        result[tag] = out
        total_t += dt
    print(f"total planning time: {total_t:.3f}s over {len(result)} scenarios")
    f = Path(__file__).parent / "runs" / "router_baseline.json"
    f.parent.mkdir(exist_ok=True)
    if mode == "write":
        f.write_text(json.dumps(result, ensure_ascii=False, indent=1))
        print(f"baseline written: {f}")
    else:
        base = json.loads(f.read_text())
        bad = [k for k in base if base[k] != result[k]]
        if bad:
            print(f"MISMATCH in {len(bad)} scenarios: {bad[:5]}")
            for k in bad[:2]:
                print(" base:", base[k])
                print("  new:", result[k])
            sys.exit(1)
        print("MATCH: all scenarios identical")


if __name__ == "__main__":
    main()
