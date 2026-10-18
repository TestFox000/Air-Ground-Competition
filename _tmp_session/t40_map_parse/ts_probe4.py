
import sys, yaml
SRC = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\src\dzbrain"
CFG = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\config\robot.yaml"
MAP = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t40_map_parse\map_new.yaml"
if SRC not in sys.path: sys.path.insert(0, SRC)
from dzbrain.core.config import Config
from dzbrain.route import planner as PL
cfg = Config.load(CFG); map_cfg = yaml.safe_load(open(MAP, encoding="utf-8"))
spec = PL.load_map(map_cfg); phys = PL._phys_params(cfg)
grid = PL.Grid(spec, bridge_rows=(7,5), allow_reverse=False)
ts = PL._TSGrid(spec=spec, grid=grid, phys=phys, turned_mines=())
def c(n): return PL._cell(n)
cache={}
for a,b,hin in [("M7","L7",(-1,0)),("M7","L7",(0,-1)),("M8","L7",(-1,0)),("L7","K7",(0,-1)),("M7","K7",(-1,0))]:
    legs = PL._ts_leg(ts, c(a), c(b), hin, cache)
    print(a,"->",b,"h_in",hin,"=>",len(legs), list(legs)[:3])
print("start->L7 first_dir west:", len(PL._ts_leg(ts, c("N10"), c("L7"), PL.DIRS[1], {}, first_dir=PL.DIRS[1])))
print("start->M7 first_dir west:", len(PL._ts_leg(ts, c("N10"), c("M7"), PL.DIRS[1], {}, first_dir=PL.DIRS[1])))
