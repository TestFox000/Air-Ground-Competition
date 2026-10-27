"""任务路由器：世界态势 -> 绕障的有序格序列。

输入 world_model.WorldSnapshot，输出有序格号（tuple[str]），交给 car_adapter。

运动模型（对齐官方 path_executor 路线语法"直线连接、可对角"）：
    - 8 邻域移动：正交 1.0 格、对角 √2 格；对角移动要求两侧正交格均非障碍（防切角）；
    - 转向代价：阿克曼底盘（轴距 150mm、最小转弯半径≈0.34m），每 45° 转向
      ≈ 0.27m 弧长 ≈ 0.76 格，默认 turn_45=0.75（设 0 则退化为纯最短路）；
    - 朝向：start_heading / final_heading 支持 'x'（东）/ 'y'（北），
      对应车端 reset 断言朝向与路线末尾 ",x/,y"；中途朝向自动链式传递。

做法（近似全局最优）：
    - 障碍格为 blocked；必经点 = 雷 + 可通行桥洞格 + 蓝色敌靶格（开过去才能打击）；
    - 各必经点间用朝向感知 Dijkstra（状态 = 格 + 航向）求最省代价路径作边权，
      Held-Karp 位压 DP 求访问顺序（边权对"进入朝向"取乐观界；n 超过
      _TSP_EXACT_MAX_NODES 时退化为多起点最近邻，避免 2^n 卡死）；
    - 枚举所有达到乐观最优的顺序，逐段按真实朝向链拼接、顺路避让已踩格，
      取（真实代价, 重复踩格）最小者；
    - 不可达必经点剔除（跳过）；终点不可达报错并附连通性诊断
      （封锁点分类 + 开放桥门，2026-10-17）。

中场挡板：默认不建模；现场证实"只能从可通行格穿越半场"后传入 cross_gates
（允许穿越的格）。桥洞一格宽，穿越中线的移动限定正交且贴着 gate 格。
"""
# 2026-10-27 模块化拆分：实现移入同目录扁平模块 mr_params/mr_board/mr_nav/mr_shooting/mr_route。
# 本文件是对外稳定门面：from .mission_router import X / from ag_integrate.mission_router import X 语义不变（含 _ 私有名）。

from __future__ import annotations

import heapq
import warnings
from itertools import count, product
from math import gcd

from .field_frame import CellRef, ROWS, COLS, LETTERS
from .fire_guard import blue_targets_hit, decide_fire
from .world_model import WorldSnapshot
from .mr_params import _ALL_HEADINGS, _BIG_INT, _DIRS, _EPS, _GEAR_SWITCH, _HEADING_IDX, _HK_HEADING_MAX, _INF, _NEED_MM, _REV_MULT, _SPACE_W, _SQRT2, _TSP_EXACT_MAX_NODES, _TURN_GRADE, _TURN_GUARD  # noqa: F401
from .mr_board import _BARRIER_COLS, _BOARD_INTRUDE_MM, _CELLS, _EDGES, _IDIAG, _INEDGES, _INNER, _MID_BOUNDARY, _N_CELLS, _SIDES, _TURNS, _build_edges, _build_inedges, _build_solid_barriers, _build_turn_side_tables, _crosses_mid, _turn_units, _zone_of  # noqa: F401
from .mr_nav import _DIR_INDEX, _ORTHO_STEPS, _arrival_state, _assemble, _cached_dijkstra, _classify_blocked, _dijkstra, _enumerate_optimal_orders, _flatten, _greedy_orders, _held_karp_orders, _held_karp_orders_heading, _norm_dir, _plan_core, _pos, _reconstruct_order, _route_leg, _route_shape, _trace_states, _unreachable_detail, ensure_mine_center_pass  # noqa: F401
from .mr_shooting import _FACE_OPP, _SHOOT_COMBO_CAP, _STANDOFF_OFFSETS, _face_heading, _los_clear, _shoot_candidates, _shoot_candidates_moving, _standoff_serves  # noqa: F401
from .mr_route import _CORRIDOR_CROSS, _ZONE_NAMES, _derive_cross_gates, _dist, _pick_transit_dijkstra, _transit_blocked, _verify_zone_covered, compress_route, mission_keep, plan_route, plan_route_zoned  # noqa: F401


# ---- 旧式模块级调参兼容（保持拆分前用法） ----
# 拆分前 mission_router 是单模块，`mission_router._TURN_GUARD = True` 这类运行时打补丁
# 直接改写规划代码读到的全局量；拆分后各实现模块持有各自的值拷贝，这里把对门面的
# 赋值广播到所有已加载的实现模块，保证旧用法（含测试 test_turn90_guard_runtime_toggle）继续生效。
import sys as _sys
import types as _types

_SUBMODULES = ("mr_params", "mr_board", "mr_nav", "mr_shooting", "mr_route")

class _TunableForwardModule(_types.ModuleType):
    def __setattr__(self, name, value):
        _pkg = __package__
        for _m in _SUBMODULES:
            _mod = _sys.modules.get(_pkg + "." + _m)
            if _mod is not None and hasattr(_mod, name) and name not in ("__name__", "__file__", "__package__", "__doc__", "__spec__", "__loader__"):
                object.__setattr__(_mod, name, value)
        super().__setattr__(name, value)

_sys.modules[__name__].__class__ = _TunableForwardModule
