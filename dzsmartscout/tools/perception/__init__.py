# -*- coding: utf-8 -*-
r"""无人机俯瞰场地感知管线（纯 Python：numpy + Pillow，cv2 仅可选）。

给"无人机摄像头读场地"这条链路提供**可离线验证**的一层：

| 模块 | 作用 | 主要产出 |
|---|---|---|
| :mod:`colors` | 颜色分类（HSV/RGB 规则 + 调色板）、形态学、连通域 | ``cls → 掩膜``、blob 统计 |
| :mod:`grid` | 四角点 → 单应 → 任意像素 → 格名 A1..N11 | ``ArenaRegistration`` |
| :mod:`objects` | 雷 / 障 / 靶区（+图纸"通"格）→ 格名 | 格名列表 + 置信度 + 拒收理由 |
| :mod:`car_pose` | 车顶标记 → ``block`` + ``yaw``（透视校正） | ``CarPose`` |
| :mod:`bridge_arrow` | 桥顶箭头 → 方向 → 半场通行格 | ``BridgeCall`` |
| :mod:`protocol` | 车侧契约 JSON（``drone.py`` 口径） | 4 种报文 + 校验 + 车侧解析镜像 |
| :mod:`field_map` | **出图**：感知输出 → 与 ``config/map.yaml`` 同 schema 的场地地图 | ``FieldMap``（JSON/YAML + 对账 + 置信度） |
| :mod:`pipeline` | 总装（一帧图 → 一组报文） | ``PerceptionResult`` |

入口见 ``README.md``；能力盘点见 ``CAPABILITY_L1.md``；离线自检 ``python -m perception.cli selftest``。
"""

from __future__ import annotations

__version__ = "1.1.0"

from .bridge_arrow import (ArrowDetection, ArrowSpec, BridgeCall,
                           detect_arrows, direction_from_points,
                           infer_bridge_passable)
from .car_pose import CarPose, CarSpec, best_car, detect_car
from .colors import (FIELD_DEFAULT, MAP_IMAGE, PALETTES, Blob, ColorRule, Palette,
                     as_rgb_array, blob_masks, classify, close_, connected_components,
                     dilate, erode, hsv_to_rgb, open_, rgb_to_hsv, sample_patch_stats)
from .field_map import (MAP_SCHEMA_KEYS, BlueFaceEstimate, FieldMap, Reconciliation,
                        build_field_map, build_field_map_from_result,
                        confidence_thresholds, estimate_blue_face, motion_budget,
                        parse_map_yaml, reconcile_with_map_yaml, write_field_map)
from .grid import (COL_LETTERS, CORNER_ORDER, ArenaRegistration, CellHit, GridSpec,
                   Homography, RegistrationError, arena_corner_meters,
                   register_from_arena_frame, register_from_corners,
                   register_from_named_corners)
from .objects import (FIELD_SPECS, MAP_IMAGE_SPECS, FieldObjects, ObjectClassSpec,
                      ObjectDetection, detect_field_objects, group_blocks)
from .pipeline import PerceptionConfig, PerceptionPipeline, PerceptionResult, config_for
from .protocol import (BRIDGE_HALVES, CAR_TO_DRONE_TOPIC, DRONE_TO_CAR_TOPIC,
                       PAYLOAD_TYPES, QOS_DRONE_LINK, build_bridge_passable,
                       build_field_verify, build_heartbeat, build_target_zone, encode,
                       encode_all, mirror_car_side_parse, recommended_cadence,
                       validate_payload)

__all__ = [
    "__version__",
    # colors
    "Palette", "ColorRule", "Blob", "MAP_IMAGE", "FIELD_DEFAULT", "PALETTES",
    "as_rgb_array", "rgb_to_hsv", "hsv_to_rgb", "classify", "blob_masks",
    "connected_components", "dilate", "erode", "open_", "close_", "sample_patch_stats",
    # grid
    "GridSpec", "ArenaRegistration", "CellHit", "Homography", "RegistrationError",
    "COL_LETTERS", "CORNER_ORDER", "arena_corner_meters", "register_from_corners",
    "register_from_named_corners", "register_from_arena_frame",
    # objects
    "ObjectClassSpec", "ObjectDetection", "FieldObjects", "MAP_IMAGE_SPECS",
    "FIELD_SPECS", "detect_field_objects", "group_blocks",
    # car
    "CarSpec", "CarPose", "detect_car", "best_car",
    # bridge
    "ArrowSpec", "ArrowDetection", "BridgeCall", "detect_arrows",
    "infer_bridge_passable", "direction_from_points",
    # protocol
    "DRONE_TO_CAR_TOPIC", "CAR_TO_DRONE_TOPIC", "QOS_DRONE_LINK", "PAYLOAD_TYPES",
    "BRIDGE_HALVES", "build_heartbeat", "build_bridge_passable", "build_target_zone",
    "build_field_verify", "encode", "encode_all", "validate_payload",
    "mirror_car_side_parse", "recommended_cadence",
    # pipeline
    "PerceptionConfig", "PerceptionPipeline", "PerceptionResult", "config_for",
    # field_map（t59 出图）
    "MAP_SCHEMA_KEYS", "FieldMap", "BlueFaceEstimate", "Reconciliation",
    "build_field_map", "build_field_map_from_result", "write_field_map",
    "estimate_blue_face", "motion_budget", "confidence_thresholds",
    "parse_map_yaml", "reconcile_with_map_yaml",
]
