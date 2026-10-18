import sys

sys.path.insert(0, r"src\dzbrain")
sys.path.insert(0, r"src\dzbrain\tests")

from test_aim import Rig, cfg_of  # noqa: E402
from dzbrain.core.blackboard import P_AIM_RED, P_STOP_POINT  # noqa: E402
from dzbrain.core.state import StopKind, StopPhase, StopPointState  # noqa: E402
from dzbrain.io.contract import ShotClass  # noqa: E402

cfg = cfg_of(aim={"fire_only_blue": True, "red_early_leave_s": 3.0})
rig = Rig(cfg=cfg)
rig.bb.set(P_STOP_POINT,
           StopPointState(index=29, block="D3", kind=StopKind.SHOOT, phase=StopPhase.FIRE),
           "test")
for i in range(6):
    rig.frame(dx=30, cls=ShotClass.RED, dt=0.1)
    st = rig.bb.get(P_STOP_POINT)
    print(i, "locked=", rig.task._locked_cls, "fire_only_blue=", rig.task._fire_only_blue,
          "run_s=", round(rig.task._red_run_s, 3),
          "thr=", rig.task._red_early_leave_s,
          "| bb stop=", getattr(st, "kind", None), getattr(st, "phase", None),
          "| flag=", rig.bb.get(P_AIM_RED))
