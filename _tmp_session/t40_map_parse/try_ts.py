
import sys, yaml, itertools, time
SRC = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\src\dzbrain"
CFG = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\config\robot.yaml"
MAP = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t40_map_parse\map_new.yaml"
if SRC not in sys.path: sys.path.insert(0, SRC)
from dzbrain.core.config import Config
from dzbrain.route import planner as PL

cfg = Config.load(CFG)
map_cfg = yaml.safe_load(open(MAP, encoding="utf-8"))
mines = ["A2","F4","H4","G5","E8","B10","D11","I11"]
combos = [
    ("all8", tuple(mines)),
    ("none", ()),
    ("oldconst", ("A4","D8","I6")),
    ("F4G5", ("F4","G5")),
    ("H4G5", ("H4","G5")),
    ("F4H4G5", ("F4","H4","G5")),
    ("A2F4H4G5", ("A2","F4","H4","G5")),
]
for name, tm in combos:
    t0=time.time()
    try:
        r = PL.plan_route_tracker_safe(map_cfg, cfg, cross_lower="K7", cross_upper="K5",
                                       phase_cell="A6", turned_mines=tm, max_seconds=30)
        print(f"OK   {name:12s} {time.time()-t0:5.2f}s n={len(r.tokens)} len={r.route_len_m:.2f}m cross={r.crossing}")
        print("     " + " ".join(r.tokens))
    except Exception as e:
        print(f"FAIL {name:12s} {time.time()-t0:5.2f}s {type(e).__name__}: {str(e)[:200]}")
