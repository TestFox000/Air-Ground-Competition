"""参数一致性测试：防止 params.yaml 与节点声明各说各话。

两类典型事故：
    1. yaml 里把参数名拼错 —— 节点用默认值静默运行，现场才发现不对
    2. 节点加了新参数却忘了写进 yaml —— 现场调参时找不到入口

这里用 ros_stub 实例化真实节点，拿到它们声明的参数名做比对。
"""

from pathlib import Path

import pytest

import ros_stub  # noqa: F401
import yaml_lite  # noqa: E402

PARAMS = Path(__file__).resolve().parents[1] / "src" / "ag_bringup" / "config" / "params.yaml"

NODES = {
    "vision_node": ("ag_perception.vision_node", "VisionNode"),
    "intel_fusion_node": ("ag_perception.intel_fusion_node", "IntelFusionNode"),
    "car_node": ("ag_car.car_node", "CarNode"),
    "aim_guard_node": ("ag_car.aim_guard_node", "AimGuardNode"),
    "mission_node": ("ag_mission.mission_node", "MissionNode"),
    "target_node": ("ag_mission.target_node", "TargetNode"),
    "referee_node": ("ag_referee.referee_node", "RefereeNode"),
    "chassis_node": ("ag_chassis.chassis_node", "ChassisNode"),
    "air_link_node": ("ag_bridge.air_link_node", "AirLinkNode"),
    "ground_link_node": ("ag_bridge.ground_link_node", "GroundLinkNode"),
    "fake_intel_node": ("ag_bridge.fake_intel_node", "FakeIntelNode"),
}


def _declared(section: str) -> set:
    mod_name, cls_name = NODES[section]
    mod = __import__(mod_name, fromlist=[cls_name])
    node = getattr(mod, cls_name)()
    return set(node._params.keys())


def test_params_file_exists():
    assert PARAMS.is_file(), f"找不到 {PARAMS}"


@pytest.mark.parametrize("section", sorted(NODES))
def test_yaml_keys_are_declared(section):
    cfg = yaml_lite.load_params(PARAMS)
    if section not in cfg:
        pytest.skip(f"{section} 未出现在 params.yaml（可用默认值运行）")
    declared = _declared(section)
    unknown = sorted(set(cfg[section]) - declared)
    assert not unknown, f"{section}: params.yaml 里有节点未声明的参数 {unknown}"


@pytest.mark.parametrize("section", sorted(NODES))
def test_config_covers_declared_params(section):
    """配置里至少要覆盖节点声明的一部分参数，避免整段配错对象。"""
    cfg = yaml_lite.load_params(PARAMS)
    if section not in cfg:
        pytest.skip(f"{section} 未出现在 params.yaml")
    declared = _declared(section)
    covered = set(cfg[section]) & declared
    assert covered, f"{section}: params.yaml 中的键与节点声明完全对不上"


def test_every_yaml_section_maps_to_a_node():
    cfg = yaml_lite.load_params(PARAMS)
    stray = sorted(set(cfg) - set(NODES))
    assert not stray, f"params.yaml 里有未知节点段: {stray}"


def test_known_sections_present():
    cfg = yaml_lite.load_params(PARAMS)
    for section in ("vision_node", "mission_node", "target_node", "referee_node",
                    "air_link_node", "ground_link_node"):
        assert section in cfg, f"params.yaml 缺少 {section} 段"


def test_homography_has_nine_numbers():
    cfg = yaml_lite.load_params(PARAMS)
    h = cfg["intel_fusion_node"]["homography"]
    assert isinstance(h, list) and len(h) == 9
    assert all(isinstance(v, (int, float)) for v in h)


#: 必须「yaml 与节点默认值逐字相同」的参数。
#: 其余参数允许两边不同（日志/调试类），但这几个一旦不一致就是静默事故：
#: 单看 yaml 以为在 14x11 场地上跑，实际节点用 11x14 的默认值 —— 地雷会被全部丢掉。
MUST_MATCH = {
    "mission_node": ("cols", "rows", "cell_size", "start_cell", "goal_cell"),
    "car_node": ("cols", "rows", "cell_size", "start_cell", "goal_cell"),
    "intel_fusion_node": ("cols", "rows", "cell_size"),
    "vision_node": ("img_size",),
    "target_node": ("img_w", "img_h"),
    "chassis_node": ("cell_size",),
}


@pytest.mark.parametrize("section", sorted(MUST_MATCH))
def test_yaml_value_matches_node_default(section):
    """yaml 里显式写了的参数，值必须与节点 declare 的默认值一致。

    为什么要有这条：`colcon build` 之前节点只会用源码里的默认值，
    params.yaml 是后加载的；两边不一致时，先跑起来的那一份说了算，
    于是"看起来改了参数"其实没生效。
    """
    cfg = yaml_lite.load_params(PARAMS)
    if section not in cfg:
        pytest.skip(f"{section} 未出现在 params.yaml")
    mod_name, cls_name = NODES[section]
    node = getattr(__import__(mod_name, fromlist=[cls_name]), cls_name)()
    for key in MUST_MATCH[section]:
        if key not in cfg[section]:
            continue
        assert cfg[section][key] == node._params[key], (
            f"{section}.{key}: params.yaml={cfg[section][key]!r} "
            f"但节点默认值={node._params[key]!r}"
        )


def test_field_grid_matches_car_constants():
    """场地尺寸必须与车端 path_executor 的常量一致（14 列 A~N x 11 行 1~11）。"""
    cfg = yaml_lite.load_params(PARAMS)
    for section in ("mission_node", "intel_fusion_node"):
        assert cfg[section]["cols"] == 14, section
        assert cfg[section]["rows"] == 11, section
    # 车端起点/终点
    assert cfg["mission_node"]["start_cell"] == "N10"
