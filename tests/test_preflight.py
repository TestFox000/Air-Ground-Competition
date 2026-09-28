"""上场前体检 `tools/preflight_check.py` 的测试。

这些项之所以要自动查，是因为它们全在"上场前 2 分钟没人会一条条看"的清单里：
单应没标定、dry_run 还是 true、广播地址还是默认值 —— 任何一条漏了都是白跑一局。
"""

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _load():
    spec = importlib.util.spec_from_file_location(
        "preflight_check", ROOT / "tools" / "preflight_check.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)  # type: ignore[union-attr]
    return mod


def _cfg(**over):
    base = {
        "ground_link_node": {"peer_ip": "192.168.1.255"},
        "intel_fusion_node": {"homography": [0.0] * 9},
        "car_node": {"dry_run": True, "start_cell": "N10", "goal_cell": "N11",
                     "bridge_zone_col": "K"},
        "vision_node": {"model_path": ""},
    }
    for section, values in over.items():
        base.setdefault(section, {}).update(values)
    return base


def test_uncalibrated_homography_is_fail():
    pf = _load()
    rows = pf.check_params(_cfg())
    h = [r for r in rows if r.item == "单应矩阵"]
    assert h and h[0].level == "FAIL"
    assert "calib_homography" in h[0].fix


def test_calibrated_homography_is_ok():
    pf = _load()
    rows = pf.check_params(_cfg(intel_fusion_node={"homography": [1.0] + [0.0] * 7 + [1.0]}))
    assert [r for r in rows if r.item == "单应矩阵"][0].level == "OK"


def test_dry_run_still_true_is_fail():
    pf = _load()
    rows = pf.check_params(_cfg())
    assert [r for r in rows if r.item == "dry_run"][0].level == "FAIL"
    rows = pf.check_params(_cfg(car_node={"dry_run": False}))
    assert [r for r in rows if r.item == "dry_run"][0].level == "OK"


def test_default_broadcast_address_is_fail():
    """255.255.255.255 在多网卡/跨网段时常常发不出去，现场必须改。"""
    pf = _load()
    rows = pf.check_params(_cfg(ground_link_node={"peer_ip": "255.255.255.255"}))
    assert [r for r in rows if r.item == "链路广播地址"][0].level == "FAIL"


def test_missing_model_path_is_warn():
    pf = _load()
    assert [r for r in pf.check_params(_cfg()) if r.item == "模型路径"][0].level == "WARN"


def test_check_env_never_crashes():
    pf = _load()
    rows = pf.check_env()
    assert rows and all(r.level in ("OK", "WARN", "INFO", "FAIL") for r in rows)


def test_main_returns_one_when_there_are_fails(tmp_path, capfd):
    """当前仓库状态本来就有 FAIL（单应未标定 / dry_run=true），所以必须是 1。"""
    pf = _load()
    rc = pf.main(["--params", str(ROOT / "src" / "ag_bringup" / "config" / "params.yaml")])
    out = capfd.readouterr().out
    assert rc == 1
    assert "FAIL" in out and "单应矩阵" in out


def test_pad_is_display_width_aware():
    """中文是双宽字符，用 str.ljust 会对不齐（实测列会错位）。"""
    pf = _load()
    # 4 个中文占 8 列，补 4 个空格 = 显示宽度 12
    assert pf._pad("单应矩阵", 12) == "单应矩阵" + " " * 4
    # 7 个 ASCII 占 7 列，补 5 个空格
    assert pf._pad("dry_run", 12) == "dry_run" + " " * 5
