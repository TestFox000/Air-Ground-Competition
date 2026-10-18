"""tools/ 冒烟测试：每个工具都必须"能被导入"，且不许指向不存在的脚本。

两条真实事故换来的，都不是假设：

  * ``tools/monitor_link.py`` 在模块顶层引用了 13 类协议里根本不存在的
    ``ObjectType.MINE`` / ``ObjectType.ARROW`` -> import 即 AttributeError。
    操作员的现场态势大屏连启动都启动不了，而它在 ``tools/`` 下、不在
    ``src/`` 下，没有任何测试会扫到它。
  * ``tools/preflight.py`` 顶层 ``from verify_contract import verify``，而
    ``verify_contract`` 在整个仓库里根本不存在 -> 赛前自检工具自己就是坏的。
    正应了它 docstring 里那句"比赛当天最怕的不是坏，是不知道哪坏了"。

两者共同的性质：**只会在比赛当天、在最需要它们的那一刻暴露**。工具脚本
不参与业务逻辑，因此天然逃过单元测试——这个文件就是补上那道网。
"""
from __future__ import annotations

import importlib.util
import json
import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
TOOLS = REPO / "tools"

# 在子进程里逐个导入：preflight 会 reconfigure stdout、monitor_link 会改
# sys.path，隔离掉这些副作用，也更接近操作员真实的 `python tools/x.py`。
_PROBE = r"""
import importlib.util, json, sys, pathlib
root = pathlib.Path(sys.argv[1]).resolve()
sys.path.insert(0, str(root))
import conftest  # noqa: F401  装好 src/* 路径与 rclpy 桩，与 pytest 同环境


def load(path):
    spec = importlib.util.spec_from_file_location("_tool_" + path.stem, path)
    mod = importlib.util.module_from_spec(spec)
    # 必须先登记再 exec：模块里有 `from __future__ import annotations` + @dataclass 时，
    # dataclasses._is_type 会去 sys.modules.get(cls.__module__) 取 __dict__，
    # 没登记就拿到 None -> AttributeError，看起来像工具坏了其实是加载姿势错了。
    sys.modules[spec.name] = mod
    try:
        spec.loader.exec_module(mod)
    finally:
        sys.modules.pop(spec.name, None)
    return mod


res = {}
for path in sorted((root / "tools").glob("*.py")):
    try:
        load(path)
        res[path.stem] = None
    except BaseException as exc:            # SystemExit 也要接住
        res[path.stem] = f"{type(exc).__name__}: {exc}"

# preflight.fix_cmd 的自证能力：脚本不在包里时必须当场标出来，
# 否则比赛当天会把人指挥去跑一个不存在的脚本。
try:
    pf = load(root / "tools" / "preflight.py")
    res["__fix_cmd__"] = {
        "real": pf.fix_cmd("tools/preflight.py", "--match"),
        "phantom": pf.fix_cmd("tools/dznav_calib.py"),
        "sh": pf.fix_cmd("tools/orin_bringup.sh", "up"),
    }
except BaseException as exc:
    res["__fix_cmd__"] = f"{type(exc).__name__}: {exc}"

print(json.dumps(res))
"""


@pytest.fixture(scope="module")
def probe() -> dict:
    """跑一次探测子进程，返回 {工具名: 错误串或 None}。"""
    proc = subprocess.run([sys.executable, "-c", _PROBE, str(REPO)],
                          capture_output=True, text=True, encoding="utf-8",
                          errors="replace", timeout=180, cwd=str(REPO))
    assert proc.returncode == 0, (
        f"探测子进程本身崩了（rc={proc.returncode}）\n"
        f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}")
    # 子进程可能顺带打印告警，JSON 只在最后一行
    return json.loads(proc.stdout.strip().splitlines()[-1])


def test_every_tool_module_imports(probe):
    broken = {k: v for k, v in probe.items()
              if not k.startswith("__") and v is not None}
    # 重依赖豁免：torch 属训练/评估工作站依赖（repo 线 ML 链经
    # reference/onboard/dzuav/src/cell_classifier 硬依赖它），requirements 与
    # 机载均不装。缺依赖时跳过 import 冒烟；装了 torch 的机器照常全扫。
    # 注意 probe 的键是 stem（无 .py 后缀）。
    _OPTIONAL_DEP_TOOLS = ("sim_match", "render_field_matrix")
    import importlib.util
    if importlib.util.find_spec("torch") is None:
        for name in _OPTIONAL_DEP_TOOLS:
            broken.pop(name, None)
    assert not broken, (
        "以下工具一 import 就炸，等于比赛当天不存在：\n"
        + "\n".join(f"  tools/{k}.py -> {v}" for k, v in sorted(broken.items())))


def test_probe_actually_covered_every_tool(probe):
    """防止探测静默漏扫：tools/*.py 必须一个不少地被跑到。"""
    on_disk = {p.stem for p in TOOLS.glob("*.py")}
    assert on_disk, "tools/ 下一个 .py 都没有，测试失去意义"
    assert on_disk <= set(probe), f"漏扫: {sorted(on_disk - set(probe))}"


def test_preflight_fix_cmd_flags_phantom_scripts(probe):
    fx = probe["__fix_cmd__"]
    assert isinstance(fx, dict), f"fix_cmd 探测失败: {fx}"
    assert fx["real"] == "python tools/preflight.py --match"
    assert fx["sh"] == "bash tools/orin_bringup.sh up"
    assert "本包未附此脚本" in fx["phantom"], (
        "fix_cmd 没有标出不存在的脚本——自检的'修法'会把人指挥去跑空气")


# ---------------------------------------------------------------- 显示层漂移


def _load_monitor_link():
    spec = importlib.util.spec_from_file_location(
        "_monitor_link", TOOLS / "monitor_link.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod          # 同 _PROBE.load：dataclass 需要它
    try:
        spec.loader.exec_module(mod)
    finally:
        sys.modules.pop(spec.name, None)
    return mod


def test_monitor_link_covers_every_object_type():
    """看板必须认识协议里的每一类目标。

    monitor_link 当初就是死在这：SYMBOLS 按旧的 7 类写死，协议升到 13 类后
    多出来的成员让模块顶层 AttributeError。这里从枚举反查，新增类别而忘了
    配符号 -> 当场红，而不是等到大屏黑屏。
    """
    from ag_integrate.intel_protocol import ObjectType

    ml = _load_monitor_link()
    members = {t.name for t in ObjectType}
    mapped = set(ml._KIND_BY_TYPE) | {ml.ObjectType.PAPER}   # PAPER 走 _kind 分派
    unknown = members - {t.name for t in mapped}
    assert not unknown, (
        f"ObjectType 有成员没有看板符号: {sorted(unknown)}"
        "（在 monitor_link._KIND_BY_TYPE 里补上）")


def test_monitor_link_symbols_and_colors_are_complete():
    ml = _load_monitor_link()
    kinds = {k for k in ml._KIND_BY_TYPE.values()} | {"mine", "arrow"}
    assert kinds <= set(ml.SYMBOLS), f"缺符号: {sorted(kinds - set(ml.SYMBOLS))}"
    assert kinds <= set(ml.COLORS), f"缺颜色: {sorted(kinds - set(ml.COLORS))}"
    # 'unknown' 是兜底，必须存在，否则未知类别会在渲染时 KeyError
    assert "unknown" in ml.SYMBOLS and "unknown" in ml.COLORS


def test_monitor_link_kind_falls_back_for_unknown_type():
    """未知类别要落到 'unknown' 而不是 KeyError——大屏宁可画错也不许崩。"""
    ml = _load_monitor_link()

    class _Fake:
        def __init__(self, name):
            self.name = name

    assert ml._kind(_Fake("NOT_A_REAL_CLASS"), "A1") == "unknown"
    # PAPER 按落格分派：K 列是桥洞箭头，其余是雷（与 world_model 同一规则）
    assert ml._kind(ml.ObjectType.PAPER, "K5") == "arrow"
    assert ml._kind(ml.ObjectType.PAPER, "E3") == "mine"


# ------------------------------------------------------------ 引用完整性


# 全仓引用了、但从未被写出来的脚本。留在这里当"待办清单"：新增的幽灵引用
# 会立刻让 test_tool_references_resolve 变红，而这一份是已知的历史欠账。
#
# 2026-10-18 合并分支重编：repo 线带来 calibrate_camera/release_models/
# sim_match/fake_udp_sender/car_match_bringup 五个实体脚本（原登记为"从未
# 落地"仅对部署线成立），已从清单移除；同时 repo 线的 docs/archive 历史文档
# 扫出一批早期开发脚本的幽灵引用，逐条登记。
_DOCS_OWE = "早期开发脚本，仅 docs/archive 历史文档引用，未随仓"
_CORE_OWE = "ag_core 诊断/黑匣子提示语引用的早期脚本，未随仓"
_HJY_OWE = ("胡锦宇交接材料自引的车端线工具（材料在 reference/members/hjy 与 "
            "docs/成员交接/hu-jinyu），本体副本在 reference/members/hjy/tools/，"
            "正源 preserve/members/hjy-handover-20261018，不落本仓 tools/")
KNOWN_MISSING_TOOLS = {
    # --- 从未落地（代码里的活跃提示语仍会指到它们）---
    "tools/dznav_calib.py": "dznav 转向线性表现场标定器从未落地（servo.py 提示语引用）",
    "tools/probe_car.sh": "remote_run.py 用法示例里的只读探测脚本从未落地",
    # --- ag_core 提示语欠账 ---
    "tools/car_camera_check.sh": _CORE_OWE,
    "tools/car_ros_logs.sh": _CORE_OWE,
    "tools/verify_decode.py": _CORE_OWE,
    "tools/traj_report.py": "早期轨迹报告器，ag_core 诊断提示与 docs/probe 引用，未随仓",
    "tools/car_probe_cmdvel.sh": "automations 记忆引用的早期探测脚本，未随仓",
    # --- docs/archive 历史欠账 ---
    "tools/air_smoke_check.sh": _DOCS_OWE,
    "tools/calib_homography.py": _DOCS_OWE,
    "tools/car_align_official.sh": _DOCS_OWE,
    "tools/car_build.sh": _DOCS_OWE,
    "tools/car_cleanup.sh": _DOCS_OWE,
    "tools/car_dryrun_test.sh": _DOCS_OWE,
    "tools/car_grab_frame.sh": _DOCS_OWE,
    "tools/car_guard_selftest.sh": _DOCS_OWE,
    "tools/car_inventory.sh": _DOCS_OWE,
    "tools/car_log_rehearsal.sh": _DOCS_OWE,
    "tools/car_look_down.sh": _DOCS_OWE,
    "tools/car_patch_executor.sh": _DOCS_OWE,
    "tools/car_preflight.sh": _DOCS_OWE,
    "tools/car_probe_circle.sh": _DOCS_OWE,
    "tools/car_probe_turn.sh": _DOCS_OWE,
    "tools/car_probe_velstate.sh": _DOCS_OWE,
    "tools/car_restore_official.sh": _DOCS_OWE,
    "tools/car_stage1_vision.sh": _DOCS_OWE,
    "tools/car_stage2a_referee.sh": _DOCS_OWE,
    "tools/car_stage2b_guard_proof.sh": _DOCS_OWE,
    "tools/car_verify.sh": _DOCS_OWE,
    "tools/car_xcheck_constants.sh": _DOCS_OWE,
    "tools/probe_car2.sh": _DOCS_OWE,
    "tools/setup_sync.py": _DOCS_OWE,
    "tools/sim_selfcheck.py": _DOCS_OWE,
    "tools/traj_geometry.py": _DOCS_OWE,
    # --- attic 归档自引 ---
    "tools/autolabel_yolo.py": "202b346 归档至 tools/attic/，attic 文件内历史路径自引",
    "tools/b9_test.py": "202b346 归档至 tools/attic/，attic 文件内历史路径自引",
    # --- 胡锦宇交接材料自引（2026-10-18 收编，agents/trae-glm/absorb-members-20261018）---
    "tools/plan_route.py": _HJY_OWE,
    "tools/gimbal_filter_sim.py": _HJY_OWE,
    "tools/drone_sim.py": _HJY_OWE,
    "tools/gen_map.py": _HJY_OWE,
    "tools/sim_selftest.py": _HJY_OWE,
}

# 前置否定：`~/drone_tools/localization_watchdog.py` 是**机载远端路径**，不是本仓
# tools/ 的引用；同理别的交付包里的 `<包名>/tools/<脚本>` 也不该算进来。
# 不加这层就会把它们误判成幽灵引用。
_TOOL_REF = re.compile(r"(?<![A-Za-z0-9_./])tools/[A-Za-z0-9_]+\.(?:py|sh)")
_SCAN_SUFFIX = {".py", ".sh", ".md", ".yaml", ".yml", ".txt"}
# 本文件自己不扫：它的欠账清单与注释里必然出现 tools/ 字面量，扫了就是自指。
_SKIP_FILES = {Path(__file__).name}


def _referenced_tool_paths() -> dict[str, set[str]]:
    hits: dict[str, set[str]] = {}
    for p in REPO.rglob("*"):
        if ("__pycache__" in p.parts or p.suffix not in _SCAN_SUFFIX
                or p.name in _SKIP_FILES):
            continue
        try:
            text = p.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        for m in _TOOL_REF.finditer(text):
            hits.setdefault(m.group(0), set()).add(p.relative_to(REPO).as_posix())
    return hits


def test_tool_references_resolve():
    """文档/报错里让人去跑的脚本必须真的存在。

    最阴的一种坏：orin_deploy.py 在部署失败时 raise SystemExit 提示
    "先跑 tools/release_models.py" ——偏偏那个脚本不存在，于是在你已经
    出事的时刻，把你指挥去追一个幽灵。
    """
    refs = _referenced_tool_paths()
    assert refs, "一个 tools/ 引用都没扫到，正则或扫描范围坏了"
    phantoms = {k: v for k, v in refs.items()
                if not (REPO / k).exists() and k not in KNOWN_MISSING_TOOLS}
    assert not phantoms, (
        "以下 tools/ 路径被引用但不存在（要么补上脚本，要么改掉引用；"
        "确认是历史欠账就登记进 KNOWN_MISSING_TOOLS 并写清原因）：\n"
        + "\n".join(f"  {k}\n      <- {', '.join(sorted(v))}"
                    for k, v in sorted(phantoms.items())))


def test_known_missing_list_has_no_stale_entries():
    """欠账清单不许自己发霉：哪天脚本补上了，就该从清单里删掉。"""
    now_present = sorted(k for k in KNOWN_MISSING_TOOLS if (REPO / k).exists())
    assert not now_present, (
        f"这些脚本已经存在了，请从 KNOWN_MISSING_TOOLS 移除: {now_present}")
