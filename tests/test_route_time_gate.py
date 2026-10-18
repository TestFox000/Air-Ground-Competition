"""build_mission 时间预算门控测试（route_time × 规划器，180s 硬顶纪律）。

门控语义：
- soft_cap_s 不传/<=0 → 完全不启用（旧行为逐字节不变）；
- 启用后预估秒数写入 commands.est_seconds；
- 超软预算且有射击位 → 砍射击停靠重规划（shed_shooting=True），
  雷一颗不能少；纯排雷链超预算不自动砍雷（人的决策）。
"""

from ag_integrate.car_adapter import adapt
from ag_integrate.field_frame import CellRef, LETTERS
from ag_integrate.pipeline import build_mission
from ag_integrate.world_model import TargetRef, WorldSnapshot


def _c(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


def _snap(with_blue=True):
    mines = [_c(n) for n in ("I1", "A2", "G3", "B4", "H5", "E6", "C7", "G11")]
    obstacles = [_c(n) for n in ("C2", "F4", "D7", "H8", "A9", "G9", "B10", "J11")]
    targets = ()
    if with_blue:
        track = (_c("D8"), _c("E8"))
        targets = (TargetRef(track[0], True, track=track, blue_face="S"),)
    return WorldSnapshot(0, tuple(mines), (), tuple(obstacles), targets)


def test_gate_disabled_by_default():
    cmds = build_mission(_snap(), "N11", "N6", "y", "y")
    assert cmds.est_seconds == -1.0
    assert cmds.shed_shooting is False
    assert cmds.shoot_cells   # 有蓝靶就有射击位


def test_fits_budget_no_shed():
    cmds = build_mission(_snap(), "N11", "N6", "y", "y", soft_cap_s=10000.0)
    assert cmds.est_seconds > 0
    assert cmds.shed_shooting is False
    assert cmds.shoot_cells
    assert "*" in cmds.route


def test_over_budget_sheds_shooting_but_keeps_mines():
    cmds = build_mission(_snap(), "N11", "N6", "y", "y", soft_cap_s=1.0)
    assert cmds.shed_shooting is True
    assert cmds.shoot_cells == ()
    assert "*" not in cmds.route
    for m in ("I1", "A2", "G3", "B4", "H5", "E6", "C7", "G11"):
        assert m in cmds.route.split(" "), f"砍射击位后漏雷 {m}"


def test_shed_route_is_faster():
    full = build_mission(_snap(), "N11", "N6", "y", "y", soft_cap_s=10000.0)
    shed = build_mission(_snap(), "N11", "N6", "y", "y", soft_cap_s=1.0)
    assert shed.est_seconds < full.est_seconds


def test_over_budget_without_shooting_keeps_route():
    # 无蓝靶 = 无射击位可砍：超预算也保留路线（砍雷是人的决策），只记账
    cmds = build_mission(_snap(with_blue=False), "N11", "N6", "y", "y",
                         soft_cap_s=1.0)
    assert cmds.shed_shooting is False
    assert cmds.est_seconds > 1.0
    assert "E6" in cmds.route.split(" ")


def test_gate_estimates_match_route_time_module():
    # 门控估算值与 route_time 直接计算一致（同一口径，漂移即 bug）
    from ag_integrate.route_time import RouteTimeParams, estimate_seconds
    snap = _snap()
    cmds = build_mission(snap, "N11", "N6", "y", "y", soft_cap_s=10000.0)
    ordered = tuple(t.split(",")[0].rstrip("*") for t in cmds.route.split(" "))
    est = estimate_seconds(ordered, shoot_cells=cmds.shoot_cells,
                           mine_cells={c.name for c in snap.mines},
                           params=RouteTimeParams())
    assert abs(cmds.est_seconds - est.seconds) < 1e-6
