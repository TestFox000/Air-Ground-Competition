
import sys, yaml, time
SRC = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\src\dzbrain"
CFG = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\config\robot.yaml"
MAP = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t40_map_parse\map_new.yaml"
if SRC not in sys.path: sys.path.insert(0, SRC)
from dzbrain.core.config import Config
from dzbrain.route import planner as PL
from dzbrain.core import geometry as geo

cfg = Config.load(CFG)
map_cfg = yaml.safe_load(open(MAP, encoding="utf-8"))
spec = PL.load_map(map_cfg)
phys = PL._phys_params(cfg)
start = PL._cell(spec.start); end = PL._cell(spec.end)
h0 = PL.DIRS[PL.START_HEADING]
print("START", start, "END", end, "h0", h0, "START_HEADING", PL.START_HEADING)
print("DIRS", PL.DIRS)

for lower_row in (7,8,9,10,11):
    for upper_row in (5,):
        cl = (lower_row, 11); cu = (upper_row, 11)   # col K index=10
        grid = PL.Grid(spec, bridge_rows=(cl[0], cu[0]), allow_reverse=False)
        ts = PL._TSGrid(spec=spec, grid=grid, phys=phys, turn_w=PL.TS_TURN_W, turned_mines=tuple(spec.mines))
        cache = {}
        t0=time.time()
        legs0 = PL._ts_leg(ts, start, cl, h0, cache, first_dir=h0)
        dt=time.time()-t0
        print(f"lower K{lower_row} -> legs0={len(legs0)} ({dt:.2f}s) keys={list(legs0)[:3]}")
        if legs0:
            # full search
            try:
                res = PL._ts_search_multi(ts, spec, cross_lower_cell=cl, cross_upper_cell=cu,
                                          shoot_cells=[PL._cell("B4")], phase_cell=PL._cell("A6"),
                                          start_heading=PL.START_HEADING, deadline=time.time()+20)
                n = len(res.get(PL._cell("B4"), []))
                print("    search_multi results:", n)
            except Exception as e:
                print("    search_multi ERR", type(e).__name__, str(e)[:120])
