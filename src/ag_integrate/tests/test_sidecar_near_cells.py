# -*- coding: utf-8 -*-
"""机载 sidecar 重规划守卫的路线邻近格插值（step() 守卫的配套测试）。

远场障碍不换线的守卫依赖 _route_near_cells 把压缩路线展开成途经格：
对角段隐式格必须被插值出来，否则航段中段（M2 盲区同款）的障碍
变化会被误判为"远场"而漏放行。
"""
import importlib.util
import pathlib
import sys

_SIDE = pathlib.Path(__file__).resolve().parents[3] / "tools" / "orin_plan_sidecar.py"
_spec = importlib.util.spec_from_file_location("orin_plan_sidecar", _SIDE)
mod = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = mod   # dataclass 解析字符串注解需要模块已注册
_spec.loader.exec_module(mod)


def test_near_cells_interpolates_and_dilates(tmp_path):
    core = mod.PlannerCore(mod.PlanConfig(), tmp_path, plan_mode="on-change")
    core.last_route = "A1 C3* N5,y"
    near = core._route_near_cells()
    assert "A1" in near and "C3" in near and "N5" in near   # 首尾 + '*' 尾缀不影响
    assert "B2" in near    # 对角段 A1→C3 的隐式途经格被插值展开
    assert "D3" in near    # C3 的 8 邻域膨胀
    assert "E5" not in near  # 与任何途经格/其邻域都不相干的格不进集合


def test_near_cells_empty_route(tmp_path):
    core = mod.PlannerCore(mod.PlanConfig(), tmp_path, plan_mode="on-change")
    assert core._route_near_cells() == frozenset()
