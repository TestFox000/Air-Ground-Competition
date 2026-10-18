
import sys, yaml
SRC = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\src\dzbrain"
CFG = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\config\robot.yaml"
MAP = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t40_map_parse\map_new.yaml"
if SRC not in sys.path: sys.path.insert(0, SRC)
from dzbrain.core.config import Config
from dzbrain.route import planner as PL
from dzbrain.core import geometry as geo

cfg = Config.load(CFG)
map_cfg = yaml.safe_load(open(MAP, encoding="utf-8"))
spec = PL.load_map(map_cfg); phys = PL._phys_params(cfg)
grid = PL.Grid(spec, bridge_rows=(7,5), allow_reverse=False)
ts = PL._TSGrid(spec=spec, grid=grid, phys=phys, turned_mines=())
def cell(n): return PL._cell(n)

for a,b in [("M7","L7"),("L7","K7"),("N7","M7"),("L8","L7"),("L6","L7")]:
    A,B = cell(a),cell(b)
    ax,ay = geo.block_to_meters(geo.index_to_block(A[0]-1,A[1]))
    bx,by = geo.block_to_meters(geo.index_to_block(B[0]-1,B[1]))
    print(f"--- {a}->{b} move_ok={ts.move_ok(A,B)} grid={ts.grid.move_ok(*A,*B)}")
    print("    barrier_d", round(PL._phys.barrier_distance_segment_m(ax,ay,bx,by,x=ts.bar_x,y0=ts.bar_y0,y1=ts.bar_y1),4), "req", round(ts.barrier_req,4))
    print("    obs_seg_min", round(min(PL._phys.obstacle_clearance_segment_m(ax,ay,bx,by,ox,oy) for ox,oy in ts.obs_xy.values()),4), "req", round(ts.obs_seg_req,4))
    print("    tgt_min", round(min(PL._phys.segment_rect_distance_m(ax,ay,bx,by,cx,cy,PL._phys.board_half_m()) for cx,cy in ts.tgt_xy.values()),4), "req", round(ts.board_seg_req,4))
    print("    entry_gate_seg", ts.entry_gate_segment(A,B))
    d=(B[0]-A[0],B[1]-A[1])
    print("    sideways_a", ts.entry_cell_crossed_sideways(A,d,d), "sideways_b", ts.entry_cell_crossed_sideways(B,d,d))
    print("    entry_walls_of B", ts.entry_walls.get(B))
