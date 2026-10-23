"""路线执行时间预算器：把格序列折算成秒，对齐 180s 硬顶（同分比用时）。

官方规则（docs/raw_txt/02）：比赛 3 分钟硬顶；双机均满足终点判定计时才停；
超时按 180s 计；总分相同比用时。因此"这条路跑完大约几秒"是规划侧
必须自查的量：路线太长 = 被计时器腰斩，排名分直接被时间差吞掉。

用法：
    from ag_integrate.route_time import RouteTimeParams, estimate_seconds
    budget = estimate_seconds(route, shoot_cells=cmds.shoot_cells,
                              mine_cells={c.name for c in snapshot.mines})
    if budget.over_soft_cap: ...   # 日志/黑匣子报警

⚠️ 所有时间参数都必须来自实车标定（直一格秒数、过雷减速、射击停靠）。
默认值偏保守（宁高估不低估）；实车测一次后回填 match_params / 调用处。
"""

from __future__ import annotations

from dataclasses import dataclass

from .field_frame import CELL_SIZE, LETTERS

# 官方计时规则常量
MATCH_LIMIT_S = 180.0   # 比赛硬顶
SOFT_CAP_S = 165.0      # 软预算：留 15s 给起降抖动/裁判计时误差/意外自救


@dataclass(frozen=True)
class RouteTimeParams:
    """实车标定参数（占位默认偏保守）。

    cruise_mps        巡航速度 m/s。官方车纯追踪直线实测回填（预估 0.2~0.4）。
    accel_overhead_s  每条**直线段**的加减速附加（转向/折返才算新的一段，
                      逐格与压缩两种表达必须算出同一个数，见 _count_straight_runs）。
    mine_slow_s       过雷格的减速附加（轮心进 12cm 圆即得分，无需停稳）。
    shoot_stop_s      每个射击位：停靠 + 瞄准 + 开火 + 计数器确认。
    """

    cruise_mps: float = 0.25
    accel_overhead_s: float = 0.4
    mine_slow_s: float = 0.6
    shoot_stop_s: float = 4.0


@dataclass(frozen=True)
class RouteTimeBudget:
    seconds: float          # 总预算秒数
    distance_m: float       # 行驶里程（格心连线）
    n_segments: int         # 直线段数
    n_mines: int            # 途经雷格数
    n_shoots: int           # 射击停靠数
    over_soft_cap: bool     # 超 SOFT_CAP_S：应压缩任务或提速
    over_match_limit: bool  # 超 MATCH_LIMIT_S：必然被腰斩，不可接受


def _pos(name: str) -> tuple[int, int]:
    s = name.strip().upper().rstrip("*").rstrip(",xy")
    return LETTERS.index(s[0]), int(s[1:]) - 1


def _count_straight_runs(names: list[str]) -> int:
    """路线折成几条直线段（= 加减速发生次数）。

    与路线用"逐格"还是"压缩"表达**无关**：方向不变就仍属同一条直线段。
    折返（同线反向）单独算一段——那需要停稳再起步。
    """
    if len(names) < 2:
        return 0
    runs = 1
    prev: tuple[int, int] | None = None
    for a, b in zip(names, names[1:]):
        (c1, r1), (c2, r2) = _pos(a), _pos(b)
        cur = (c2 - c1, r2 - r1)
        if prev is not None:
            cross = cur[0] * prev[1] - cur[1] * prev[0]
            dot = cur[0] * prev[0] + cur[1] * prev[1]
            if cross != 0 or dot <= 0:
                runs += 1
        prev = cur
    return runs


def estimate_seconds(
    route: tuple[str, ...] | list[str],
    *,
    shoot_cells=frozenset(),
    mine_cells=frozenset(),
    params: RouteTimeParams = RouteTimeParams(),
) -> RouteTimeBudget:
    """把有序格号序列折算成执行秒数。

    route 接受压缩后路线（相邻格号=直线段，可对角）也接受逐格路线，且**两者
    必须给出同一预算**（段数按直线段数算，不按格数）；shoot_cells/mine_cells
    传格名集合（如 "B3"）。
    """
    if len(route) < 2:
        return RouteTimeBudget(0.0, 0.0, 0, 0, 0, False, False)

    names = [n.strip().upper().rstrip("*").split(",")[0] for n in route]
    dist_cells = 0.0
    for a, b in zip(names, names[1:]):
        (c1, r1), (c2, r2) = _pos(a), _pos(b)
        dist_cells += ((c2 - c1) ** 2 + (r2 - r1) ** 2) ** 0.5

    distance_m = dist_cells * CELL_SIZE
    # 段数按**直线段**算，不是按格数。accel_overhead_s 的定义就是"每条直线段的
    # 加减速附加"，而 plan_route 返回的是逐格序列：旧写法 n_segments=len-1 把它
    # 变成"每格 0.4s"，于是同一条物理路线按逐格喂和按 compress_route 压缩后喂
    # 会算出不同预算（实测 59 格全场路线 120.2s vs 106.6s，差 13.6s/13%，
    # 里程完全相同）。pipeline 走的是逐格、tests 走的是压缩，等于生产门限比
    # 测试门限严 13s —— 幻影开销足以把一条真实 158s 的路线判成超 165s 软顶，
    # 触发不必要的弃靶/弃雷（每弃一项 −5 分）。
    n_segments = _count_straight_runs(names)
    # 按**出现次数**计，不按集合：同一格被路线经过两次，车就要减速/停靠两次。
    mine_names = {m.strip().upper() for m in mine_cells}
    shoot_names = {s.strip().upper() for s in shoot_cells}
    n_mines = sum(1 for n in names if n in mine_names)
    n_shoots = sum(1 for n in names if n in shoot_names)

    seconds = (
        distance_m / params.cruise_mps
        + n_segments * params.accel_overhead_s
        + n_mines * params.mine_slow_s
        + n_shoots * params.shoot_stop_s
    )
    return RouteTimeBudget(
        seconds=seconds,
        distance_m=distance_m,
        n_segments=n_segments,
        n_mines=n_mines,
        n_shoots=n_shoots,
        over_soft_cap=seconds > SOFT_CAP_S,
        over_match_limit=seconds > MATCH_LIMIT_S,
    )
