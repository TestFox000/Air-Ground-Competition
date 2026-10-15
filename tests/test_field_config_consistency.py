"""配置一致性守护：match_params.yaml / integrate_node 默认值 / mission_router
三处的场地常量必须一致，防止"改了一处忘了另一处"（本仓库 2026-10 真实事故：
_MID_BOUNDARY 修为 6 后，yaml/节点/管线仍停留在 5，两个场地模型分裂）。

不引入 PyYAML：复用 tests/yaml_lite。
"""

import re
from pathlib import Path

import yaml_lite

from ag_integrate.mission_router import _MID_BOUNDARY

REPO = Path(__file__).resolve().parent.parent
YAML_PATH = REPO / "src" / "ag_integrate" / "config" / "match_params.yaml"
NODE_PATH = REPO / "src" / "ag_integrate" / "ag_integrate" / "integrate_node.py"


def _yaml_params():
    return yaml_lite.load_params(YAML_PATH)["integrate_node"]


def test_mid_boundary_consistent_everywhere():
    """mid_boundary：yaml == 节点默认值 == mission_router._MID_BOUNDARY。"""
    yaml_val = int(_yaml_params()["mid_boundary"])
    src = NODE_PATH.read_text(encoding="utf-8")
    m = re.search(r'declare_parameter\("mid_boundary",\s*(\d+)\)', src)
    assert m, "integrate_node.py 找不到 mid_boundary 默认值声明"
    node_default = int(m.group(1))
    assert yaml_val == _MID_BOUNDARY, (
        f"yaml mid_boundary={yaml_val} 与 mission_router._MID_BOUNDARY="
        f"{_MID_BOUNDARY} 不一致——路线将按错误的中场挡板位置规划")
    assert node_default == _MID_BOUNDARY, (
        f"节点默认 mid_boundary={node_default} 与 mission_router._MID_BOUNDARY="
        f"{_MID_BOUNDARY} 不一致")


def test_start_goal_cells_well_formed():
    """起终点格号合法、朝向断言合法（每局必改项的防呆）。"""
    params = _yaml_params()
    for key in ("start_cell", "goal_cell"):
        name = str(params[key]).strip().upper()
        assert re.fullmatch(r"[A-N](?:[1-9]|1[01])", name), (
            f"{key}={name!r} 不是合法格号（A1..N11）")
    for key in ("start_heading", "final_heading"):
        v = str(params[key]).strip().lower()
        assert v in ("x", "y"), f"{key}={v!r} 只支持 x/y"


def test_target_track_prior_well_formed():
    """移动靶先验：要么留空（全靠无人机识别），要么轨道两格 + 蓝面朝向齐全。"""
    params = _yaml_params()
    track = [s for s in params.get("target_track", []) if str(s).strip()]
    face = str(params.get("target_blue_face", "")).strip().upper()
    if not track:
        return
    assert len(track) == 2, f"target_track 需恰好两格，收到 {track}"
    for name in track:
        assert re.fullmatch(r"[A-N](?:[1-9]|1[01])", name.strip().upper())
    assert face in ("S", "N", "E", "W"), f"target_blue_face={face!r} 非法"
