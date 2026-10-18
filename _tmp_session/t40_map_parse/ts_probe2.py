
import sys, yaml, time
SRC = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\src\dzbrain"
CFG = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\config\robot.yaml"
MAP = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t40_map_parse\map_new.yaml"
if SRC not in sys.path: sys.path.insert(0, SRC)
from dzbrain.core.config import Config
from dzbrain.route import planner as PL

cfg = Config.load(CFG)
map_cfg = yaml.safe_load(open(MAP, encoding="utf-8"))
spec = PL.load_map(map_cfg); phys = PL._phys_params(cfg)
start = PL._cell(spec.start); h0 = PL.DIRS[PL.START_HEADING]
grid = PL.Grid(spec, bridge_rows=(7,5), allow_reverse=False)
ts = PL._TSGrid(spec=spec, grid=grid, phys=phys, turned_mines=())
cache = {}
def cell(n): return PL._cell(n)
targets = ["M10","M9","M8","N7","M7","L7","K7","J7","L8","L9","M7"]
for t in targets:
    legs = PL._ts_leg(ts, start, cell(t), h0, cache, first_dir=h0)
    print(f"start->{t}: {len(legs)} keys={list(legs)[:2]}")
# direct legs
for a,b in [("M7","L7"),("L7","K7"),("K7","J7"),("L7","L6"),("L8","L7"),("M7","M6")]:
    legs = PL._ts_leg(ts, cell(a), cell(b), None, cache)
    print(f"{a}->{b}: {len(legs)} keys={list(legs)[:2]}")
# move_ok checks
for a,b in [("L7","K7"),("M7","L7"),("N7","M7"),("K7","J7"),("M10","M9"),("N10","M10")]:
    print("move_ok", a, b, ts.move_ok(cell(a), cell(b)), "grid", grid.move_ok(*cell(a), *cell(b)))
