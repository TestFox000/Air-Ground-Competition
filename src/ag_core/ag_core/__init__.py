"""ag_core：空地协同侦排挑战赛 —— 与 ROS 无关的纯算法核心。

设计原则：
    1. 本包内 **不 import 任何 ROS / rknn / onnxruntime**，只依赖 numpy；
    2. 所有算法可在 Windows 上用 pytest 直接验证，无需板卡与真机；
    3. ROS 节点（ag_perception / ag_mission / ag_referee / ag_bridge）只做
       "消息搬运 + 调用本包"，从而把可测逻辑与不可测的硬件环境隔离。
"""

__version__ = "0.1.0"

from .postprocess import Detection, decode_anchor_free, decode_anchored, letterbox_params, scale_coords
from .grid_map import FieldGrid, CellID
from .geometry import pixel_to_gimbal_delta, clamp_gimbal
from .target_policy import TargetPolicy, TargetDecision
from .mission_planner import MissionPlanner, TaskType
from .referee_protocol import RefereeState, RefereeCodec
from .link_protocol import FieldIntel, LinkCodec
from .backend import BaseBackend, DummyBackend, load_backend

__all__ = [
    "Detection",
    "decode_anchor_free",
    "decode_anchored",
    "letterbox_params",
    "scale_coords",
    "FieldGrid",
    "CellID",
    "pixel_to_gimbal_delta",
    "clamp_gimbal",
    "TargetPolicy",
    "TargetDecision",
    "MissionPlanner",
    "TaskType",
    "RefereeState",
    "RefereeCodec",
    "FieldIntel",
    "LinkCodec",
    "BaseBackend",
    "DummyBackend",
    "load_backend",
]
