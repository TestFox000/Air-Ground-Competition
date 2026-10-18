
import sys, yaml
SRC = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\src\dzbrain"
CFG = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\config\robot.yaml"
MAP = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t40_map_parse\map_new.yaml"
if SRC not in sys.path: sys.path.insert(0, SRC)
from dzbrain.core.config import Config
from dzbrain.route.parser import parse_mission
from dzbrain.route.validate import validate_route
cfg = Config.load(CFG)
map_cfg = yaml.safe_load(open(MAP, encoding="utf-8"))
tokens = "N10 N9 N8 N7 M7 L7 K7 J7 J8 J9 J10 J11 I11 H11 G11 F11 E11 D11 C11 B11 B10 B9 B8 D8 E8 F8 F7 E7 D7 C7 C6 C5 C4 E4 F4 G4 H4 I4:PIVOT H4 G4:PIVOT H4 H5 G5 F5 F4 E4 D4 C4 B4* B3 A3 A2 A1:PIVOT A2 A4 B4 C4 D4 E4 E5 G5 H5 I5 J5 K5 L5 M5 N5,y".split()
route = parse_mission(tokens)
iss = validate_route(route, map_cfg=map_cfg, cfg=cfg)
errs = [str(i) for i in iss if i.level=="error"]
warns = [str(i) for i in iss if i.level=="warn"]
print("waypoints", len(route))
print("ERRORS", len(errs))
for e in errs[:20]: print("  E", e)
print("WARNS", len(warns))
for w in warns[:25]: print("  W", w)
