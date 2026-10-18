import sys

sys.path.insert(0, r"src\dzbrain")
sys.path.insert(0, r"src\dzbrain\tests")

from test_aim import Rig, cfg_of  # noqa: E402

cfg = cfg_of(aim={"fire_only_blue": True, "red_early_leave_s": 3.0})
print("cfg.get fob      =", repr(cfg.get("aim.fire_only_blue")))
print("cfg.get fob dflt =", repr(cfg.get("aim.fire_only_blue", False)))
print("cfg.get red      =", repr(cfg.get("aim.red_early_leave_s")))
print("flat keys        =", [k for k in getattr(cfg, "_flat", {}) if "only_blue" in k])
rig = Rig(cfg=cfg)
print("task cfg is same =", rig.task.cfg is cfg)
print("task fob         =", rig.task._fire_only_blue)
print("task cfg.get fob =", repr(rig.task.cfg.get("aim.fire_only_blue")))
