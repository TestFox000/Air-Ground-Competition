
import sys, yaml, time
SRC = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\src\dzbrain"
CFG = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\config\robot.yaml"
MAP = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t40_map_parse\map_new.yaml"
if SRC not in sys.path: sys.path.insert(0, SRC)
from dzbrain.core.config import Config
from dzbrain.route import planner as PL

cfg = Config.load(CFG)
map_cfg = yaml.safe_load(open(MAP, encoding="utf-8"))
spec = PL.load_map(map_cfg)
grid = PL.Grid(spec, bridge_rows=(7,5), allow_reverse=False)
acc = PL._shoot_accept(cfg)
print("shoot window:", acc)
try:
    pool = PL.shoot_candidates(spec, grid, acc)
    print("shoot candidates:", pool)
    cover = PL.covering_stops(spec, pool, acc)
    print("coverage:", {k: list(v) for k,v in cover.items()})
except Exception as e:
    print("shoot calc ERR", type(e).__name__, e)

mines = ["A2","F4","H4","G5","E8","B10","D11","I11"]
for phase in ("A6","B6","C6","A5","C5","B5"):
    t0=time.time()
    try:
        r = PL.plan_route_tracker_safe(map_cfg, cfg, cross_lower="K7", cross_upper="K5",
                                       phase_cell=phase, turned_mines=tuple(mines), max_seconds=30)
        print(f"OK phase={phase} {time.time()-t0:5.2f}s n={len(r.tokens)} len={r.route_len_m:.2f} cross={r.crossing}")
        print("   " + " ".join(r.tokens))
    except Exception as e:
        print(f"FAIL phase={phase} {time.time()-t0:5.2f}s {type(e).__name__}: {str(e)[:220]}")
