"""打包结构测试 —— 保证「本地能跑」等于「真机能 colcon build 且 ros2 run 能找到入口」。
"""

import ast
import configparser
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src"

PY_PKGS = sorted(p.name for p in SRC.iterdir()
                 if p.is_dir() and (p / "setup.py").is_file())
ALL_PKGS = sorted(p.name for p in SRC.iterdir()
                  if p.is_dir() and (p / "package.xml").is_file())


def _declared_packages(pkg: str) -> list:
    """读 setup.py 的 packages=[...]（ag_bringup 是只有 launch/config 的空包，合法）。"""
    tree = ast.parse((SRC / pkg / "setup.py").read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "setup":
            for kw in node.keywords:
                if kw.arg == "packages" and isinstance(kw.value, ast.List):
                    return [e.value for e in kw.value.elts if isinstance(e, ast.Constant)]
    return []


def _entry_points(pkg: str) -> dict:
    """读 setup.py 里的 console_scripts（用 AST，不执行文件）。"""
    tree = ast.parse((SRC / pkg / "setup.py").read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "setup":
            for kw in node.keywords:
                if kw.arg == "entry_points" and isinstance(kw.value, ast.Dict):
                    out = {}
                    for k, v in zip(kw.value.keys, kw.value.values):
                        if getattr(k, "value", "") == "console_scripts":
                            for elt in v.elts:
                                name, target = elt.value.split("=", 1)
                                out[name.strip()] = target.strip()
                    return out
    return {}


def test_found_expected_packages():
    assert "ag_core" in PY_PKGS and "ag_car" in PY_PKGS
    assert len(ALL_PKGS) >= 9


@pytest.mark.parametrize("pkg", ALL_PKGS)
def test_package_xml_exists(pkg):
    assert (SRC / pkg / "package.xml").is_file()


@pytest.mark.parametrize("pkg", PY_PKGS)
def test_inner_package_dir_exists(pkg):
    """setup.py 里声明的 packages 必须真的存在。

    真实事故：ag_core 早期是「src/ag_core/*.py」的扁平布局，
    本地测试靠 sys.path 能过，真机 `colcon build` 却报
    `error: package directory 'ag_core' does not exist`。
    """
    for name in _declared_packages(pkg):
        assert (SRC / pkg / name / "__init__.py").is_file(), \
            f"{pkg}: setup.py 声明了 packages=['{name}']，但 {pkg}/{name}/__init__.py 不存在"


@pytest.mark.parametrize("pkg", PY_PKGS)
def test_setup_cfg_redirects_scripts(pkg):
    """必须带 setup.cfg，把入口脚本装到 lib/<pkg>/。

    真实事故：缺 setup.cfg 时脚本落到 install/<pkg>/bin/，
    `ros2 pkg executables` 查不到 → `ros2 run` 与 launch 启动瞬间失败。
    """
    cfg_path = SRC / pkg / "setup.cfg"
    assert cfg_path.is_file(), f"{pkg} 缺 setup.cfg（ros2 run 会找不到可执行文件）"
    cfg = configparser.ConfigParser()
    cfg.read(cfg_path, encoding="utf-8")
    want = f"$base/lib/{pkg}"
    for section in ("develop", "install"):
        assert cfg.has_section(section), f"{pkg}/setup.cfg 缺 [{section}]"
        key = "script_dir" if section == "develop" else "install_scripts"
        assert cfg[section].get(key) == want, f"{pkg}/setup.cfg [{section}] {key} 应为 {want}"


@pytest.mark.parametrize("pkg", PY_PKGS + ["ag_interfaces"])
def test_build_type_declared(pkg):
    xml = (SRC / pkg / "package.xml").read_text(encoding="utf-8")
    if pkg == "ag_interfaces":
        assert "ament_cmake" in xml
    else:
        assert "ament_python" in xml


@pytest.mark.parametrize("pkg", PY_PKGS)
def test_entry_point_targets_exist(pkg):
    """`e = pkg.mod:main` 里的文件和函数必须真的存在（拼错=启动即崩）。"""
    for name, target in _entry_points(pkg).items():
        mod_path, _, func = target.partition(":")
        rel = Path(*mod_path.split(".")[1:])       # 去掉顶层包名
        f = SRC / pkg / pkg / (str(rel) + ".py")
        assert f.is_file(), f"{pkg}: {name} 指向的 {mod_path} 不存在 ({f})"
        src = f.read_text(encoding="utf-8")
        assert f"def {func}(" in src, f"{pkg}: {mod_path} 里没有 {func}()"


@pytest.mark.parametrize("pkg", PY_PKGS)
def test_no_stray_cmake_for_python_pkg(pkg):
    assert not (SRC / pkg / "CMakeLists.txt").is_file(), f"{pkg} 是 ament_python，不应有 CMakeLists.txt"


def test_interfaces_package_has_msgs():
    msgs = sorted(p.name for p in (SRC / "ag_interfaces" / "msg").glob("*.msg"))
    assert msgs, "ag_interfaces/msg 下没有消息定义"
    cmake = (SRC / "ag_interfaces" / "CMakeLists.txt").read_text(encoding="utf-8")
    for m in msgs:
        assert f"msg/{m}" in cmake, f"CMakeLists.txt 里没有注册 {m}"
