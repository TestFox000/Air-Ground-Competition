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
RECON_PATH = REPO / "src" / "ag_integrate" / "ag_integrate" / "video_recon.py"


def _yaml_params():
    return yaml_lite.load_params(YAML_PATH)["integrate_node"]


def test_mid_boundary_consistent_everywhere():
    """mid_boundary：yaml == 节点默认值 == mission_router._MID_BOUNDARY。"""
    yaml_val = int(_yaml_params()["mid_boundary"])
    src = NODE_PATH.read_text(encoding="utf-8")
    m = re.search(r'declare_parameter\("mid_boundary",\s*(\d+|_MID_BOUNDARY)\)', src)
    assert m, "integrate_node.py 找不到 mid_boundary 默认值声明"
    # 节点用常量引用声明时一致性由构造保证；字面量则按值校验
    node_default = _MID_BOUNDARY if m.group(1) == "_MID_BOUNDARY" else int(m.group(1))
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


def test_turn_balance_params_consistent():
    """步数-姿态平衡参数：yaml == 节点默认值 == 路由层默认值。

    turn_45/allow_diagonal 是 match_params.yaml -> integrate_node ->
    build_mission -> plan_route_zoned 整链透传的新配置；三处默认值
    必须一致（否则"没写 yaml 就改行为"的静默分裂会复发——mid_boundary 事故同款）。
    """
    params = _yaml_params()
    src = NODE_PATH.read_text(encoding="utf-8")
    m_t = re.search(r'declare_parameter\("turn_45",\s*([\d.]+)\)', src)
    assert m_t, "integrate_node.py 找不到 turn_45 默认值声明"
    assert float(params["turn_45"]) == float(m_t.group(1)), (
        f"yaml turn_45={params['turn_45']} 与节点默认 {m_t.group(1)} 不一致")
    # 节点读到的是布尔；yaml 用 true/false
    assert bool(params["allow_diagonal"]) is True, "yaml allow_diagonal 应为 true"
    m_d = re.search(r'declare_parameter\("allow_diagonal",\s*(True|False)\)', src)
    assert m_d and m_d.group(1) == "True", "integrate_node.py allow_diagonal 默认应为 True"


def _norm(name: str) -> str:
    return str(name).strip().upper()


def test_start_goal_and_track_consistent_with_yaml():
    """起终点与移动靶轨道：yaml == integrate_node 默认值 == video_recon 回退值。

    2026-10-17 按官方裁判地图更新时只改了 yaml，漏掉另两处，实测残留：
      integrate_node.py  declare_parameter("goal_cell", "N11")   yaml 是 N5
      video_recon.py     START_DFT, GOAL_DFT = "N11", "N6"       yaml 是 N10/N5
      video_recon.py     TRACK_DFT = ("D8", "E8")                yaml 是 D7/E7
    节点默认值在 launch 没加载到 match_params.yaml 时直接生效（ros2 run 手起、
    参数文件路径错、share 未重装都会走到），video_recon 回退值在视频没拍到
    车/靶时生效——两者都会按过期格号规划整条路线。mid_boundary 事故同款。
    """
    params = _yaml_params()
    y_start, y_goal = _norm(params["start_cell"]), _norm(params["goal_cell"])
    y_track = [_norm(c) for c in params.get("target_track", []) if str(c).strip()]

    node_src = NODE_PATH.read_text(encoding="utf-8")
    for key, expected in (("start_cell", y_start), ("goal_cell", y_goal)):
        m = re.search(rf'declare_parameter\("{key}",\s*"([A-N]\d{{1,2}})"\)', node_src)
        assert m, f"integrate_node.py 找不到 {key} 默认值声明"
        assert _norm(m.group(1)) == expected, (
            f"integrate_node.py {key} 默认 {m.group(1)!r} 与 yaml {expected!r} 不一致"
            f"——launch 未加载 match_params.yaml 时会按旧格号规划")

    recon_src = RECON_PATH.read_text(encoding="utf-8")
    m = re.search(r'START_DFT,\s*GOAL_DFT\s*=\s*"([A-N]\d{1,2})",\s*"([A-N]\d{1,2})"',
                  recon_src)
    assert m, "video_recon.py 找不到 START_DFT/GOAL_DFT 定义"
    assert (_norm(m.group(1)), _norm(m.group(2))) == (y_start, y_goal), (
        f"video_recon.py START/GOAL_DFT={m.group(1)}/{m.group(2)} 与 yaml "
        f"{y_start}/{y_goal} 不一致——视频没拍到车时彩排路线会用过期起终点")
    if y_track:
        m_t = re.search(r'TRACK_DFT\s*=\s*\(\s*"([A-N]\d{1,2})",\s*"([A-N]\d{1,2})"\s*\)',
                        recon_src)
        assert m_t, "video_recon.py 找不到 TRACK_DFT 定义"
        got = [_norm(m_t.group(1)), _norm(m_t.group(2))]
        assert got == y_track, (
            f"video_recon.py TRACK_DFT={got} 与 yaml target_track={y_track} 不一致"
            f"——视频没拍到靶时移动靶先验会落在过期轨道上")
