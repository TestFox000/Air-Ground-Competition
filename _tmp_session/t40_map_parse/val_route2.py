
import sys, yaml
SRC = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\src\dzbrain"
CFG = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\config\robot.yaml"
MAP = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\config\map.yaml"
if SRC not in sys.path: sys.path.insert(0, SRC)
from dzbrain.core.config import Config
from dzbrain.route.parser import parse_mission
from dzbrain.route.validate import validate_route
cfg = Config.load(CFG); map_cfg = yaml.safe_load(open(MAP, encoding="utf-8"))
tokens = cfg.get("route.tokens")
print("tokens", len(tokens), tokens[:7])
route = parse_mission(tokens)
iss = validate_route(route, map_cfg=map_cfg, cfg=cfg)
errs=[str(i) for i in iss if i.level=="error"]; warns=[str(i) for i in iss if i.level=="warn"]
print("ERRORS", len(errs), "WARNS", len(warns))
for e in errs[:10]: print(" E", e)
for w in warns[:10]: print(" W", w)
