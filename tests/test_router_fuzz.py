"""mission_router 随机地图 fuzz：按裁判规则随机布场，验证规划不变量。

场地模型（官方规则 02 + 03）：
  - 14 列(A-N) × 11 行；K 列(col 10)为桥洞墙，北段 K1-K6、南段 K7-K11
    各随机 1 个 ARROW 可通行格；
  - 白色长挡板在第 6/7 行之界（mid_boundary=6），覆盖 D..N 列，A/B/C 列自由穿越；
  - 8 雷 + 8 障碍随机落格；蓝靶随机。
不变量：路线连续、不踩障碍/靶/桥洞墙、只经 gate 穿越中场、
可达雷全覆盖、规划结果确定可复现。
"""

import os
import random
import time

from ag_integrate.field_frame import COLS, ROWS, CellRef, cell_to_world
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType
from ag_integrate.mission_router import _BARRIER_COLS, _MID_BOUNDARY, plan_route
from ag_integrate.world_model import WorldModel

_START = CellRef(13, 9)    # N10
_GOAL = CellRef(13, 10)    # N11
_K_COL = 10


def _obj(cell, t):
    x, y = cell_to_world(cell)
    return IntelObject(t, x, y, 0.35, 0.35)


def _cells_of(objects, otype):
    """从 IntelObject 序列还原指定类型的格。"""
    out = []
    for o in objects:
        if o.type is otype:
            col = round((o.x - 0.065) / 0.35)
            row = 10 - round((o.y - 0.295) / 0.35)
            out.append(CellRef(col, row))
    return out


def _random_scene(rng):
    """生成一局随机场景，返回 (objects, mines, arrows, blue_target)。

    物理约束：K 列被桥洞结构占据，裁判不会在 K 列放雷/障碍/靶
    （矩阵交接：K 列 mine 检测一律过滤），故投放池排除整列 K。
    """
    reserved = {_START.name, _GOAL.name}
    arrows = [CellRef(_K_COL, rng.randint(0, 4)),
              CellRef(_K_COL, rng.randint(5, 10))]
    pool = [CellRef(c, r) for c in range(COLS) for r in range(ROWS)
            if c != _K_COL and CellRef(c, r).name not in reserved]
    rng.shuffle(pool)
    mines = pool[:8]
    obstacles = pool[8:16]
    blue = pool[16] if rng.random() < 0.5 else None

    objects = [_obj(m, ObjectType.MINE) for m in mines]
    objects += [_obj(o, ObjectType.OBSTACLE) for o in obstacles]
    objects += [_obj(a, ObjectType.ARROW) for a in arrows]
    if blue is not None:
        objects.append(_obj(blue, ObjectType.TARGET_BLUE))
    return objects, mines, arrows, blue


def _bridge_open(arrows):
    """桥洞列可通行格：ARROW 格 + 朝向中场一侧的 1 格引道（与 mission_router 同语义）。"""
    open_cells = set()
    for a in arrows:
        open_cells.add(a.name)
        dr = -1 if a.row >= _MID_BOUNDARY else 1
        r = a.row + dr
        if 0 <= r < ROWS:
            open_cells.add(CellRef(a.col, r).name)
    return open_cells


def _gates(arrows):
    """自动 cross_gates：A/B/C 列边界两格 + 桥洞列中紧贴边界的可通行格。"""
    gates = set()
    for col in range(COLS):
        if col not in _BARRIER_COLS:
            gates.add(CellRef(col, _MID_BOUNDARY - 1).name)
            gates.add(CellRef(col, _MID_BOUNDARY).name)
    for name in _bridge_open(arrows):
        c = CellRef(ord(name[0]) - ord("A"), int(name[1:]) - 1)
        if c.row in (_MID_BOUNDARY - 1, _MID_BOUNDARY):
            gates.add(name)
    return gates


def test_random_maps_invariants():
    rng = random.Random(20261014)
    t0 = time.monotonic()
    # 默认 120 局（全密度场景每局含一次完整全局规划，x86 约 0.5s/局）；
    # 赛前深度验证可用 AIC_FUZZ_CASES=1000 python3 -m pytest tests/test_router_fuzz.py 跑彻夜档。
    n_cases = int(os.environ.get("AIC_FUZZ_CASES", "120"))
    for trial in range(n_cases):
        objects, mines, arrows, blue = _random_scene(rng)
        snap = WorldModel().update(IntelFrame(trial, tuple(objects)))
        route = plan_route(snap, _START, _GOAL)
        names = list(route)
        obstacles = _cells_of(objects, ObjectType.OBSTACLE)

        # 首尾正确
        assert names[0] == _START.name and names[-1] == _GOAL.name

        open_k = _bridge_open(arrows)
        gates = _gates(arrows)
        blocked_names = {c.name for c in obstacles}
        if blue is not None:
            blocked_names.add(blue.name)

        for a, b in zip(names, names[1:]):
            ca, ra = ord(a[0]) - ord("A"), int(a[1:]) - 1
            cb, rb = ord(b[0]) - ord("A"), int(b[1:]) - 1
            # 相邻连续（8 邻域一步）
            assert abs(ca - cb) <= 1 and abs(ra - rb) <= 1 and (ca, ra) != (cb, rb)
            # 穿越中场（行 5↔6，0 基）只能发生在 gate 格
            if ca == cb and {ra, rb} == {_MID_BOUNDARY - 1, _MID_BOUNDARY}:
                assert a in gates and b in gates, (
                    f"trial {trial}: 非 gate 穿越 {a}->{b}")

        # 不踩障碍 / 靶 / 桥洞墙
        for n in names:
            assert n not in blocked_names, f"trial {trial}: 踩禁格 {n}"
            if n[0] == "K":
                assert n in open_k, f"trial {trial}: 闯入桥洞墙 {n}"

        # 可达雷全覆盖：被剔除的雷在"仅它一个雷"的同障碍场景下也必须不可达
        in_route = set(names)
        for m in mines:
            if m.name in in_route:
                continue
            solo_objects = [_obj(m, ObjectType.MINE)]
            solo_objects += [_obj(o, ObjectType.OBSTACLE) for o in obstacles]
            solo_objects += [_obj(a, ObjectType.ARROW) for a in arrows]
            if blue is not None:
                # 靶格在全局场景同样是 blocked，单独验证时不可漏掉，
                # 否则会出现"单独可达（走了穿靶的路）却误判全局误剔"的假阳性
                solo_objects.append(_obj(blue, ObjectType.OBSTACLE))
            solo_snap = WorldModel().update(IntelFrame(trial, tuple(solo_objects)))
            solo_route = plan_route(solo_snap, _START, _GOAL)
            assert m.name not in solo_route, (
                f"trial {trial}: 雷 {m.name} 单独可达却被全局路线剔除")

        # 确定性：同输入同输出
        if trial % 50 == 0:
            assert plan_route(snap, _START, _GOAL) == route

    elapsed = time.monotonic() - t0
    # 性能回归守卫按局数缩放（2026-10-14 P0 修复前最坏单局 33.7s——等最优顺序
    # 枚举×逐段两次 Dijkstra；修复后最坏 ~1.2s）。
    # ⚠️ 墙钟断言是机器速度相关的：慢 CI/老机器上 1s/局 会假阳性，放宽到 3 倍
    # 余量——本断言只防"灾难性回退"（数量级退化），不做精细性能回归。
    assert elapsed < 3.0 * n_cases, (
        f"{n_cases} 局 fuzz 耗时 {elapsed:.1f}s（均值 {elapsed / n_cases:.2f}s/局），规划性能退化")
