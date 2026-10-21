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

import ast
import importlib.util
import json
import re
import shutil
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
    res["__engine_defaults__"] = pf.run_dzuav_defaults(
        "ENGINES", "HBB_ENGINE", "CELL_ENGINE")
except BaseException as exc:
    res["__fix_cmd__"] = f"{type(exc).__name__}: {exc}"
    res["__engine_defaults__"] = res["__fix_cmd__"]

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
# 只登记**活引用**：本仓代码或现役文档真的会把操作员指过去的脚本。
# 仅被归档材料（docs/成员交接、reference/、docs/archive、tools/attic）引用的
# 不登记——那些目录整个不在扫描范围内，见 _ARCHIVE_PREFIXES。
# 清单由 test_known_missing_list_has_no_stale_entries 双向守：脚本补上了要删，
# 引用方消失了也要删。
_CORE_OWE = "ag_core 诊断/黑匣子提示语引用的早期脚本，未随仓"
_DOCS_OWE = "早期开发脚本，仅 docs/probe 现场排查记录引用，未随仓"
KNOWN_MISSING_TOOLS = {
    # --- 从未落地（代码里的活跃提示语仍会指到它们）---
    "tools/dznav_calib.py": "dznav 转向线性表现场标定器从未落地（servo.py 提示语引用）",
    "tools/probe_car.sh": "remote_run.py 用法示例里的只读探测脚本从未落地",
    # --- ag_core 提示语欠账（活的：log_diagnosis.py / blackbox.py 仍会打印）---
    "tools/car_camera_check.sh": _CORE_OWE,
    "tools/car_ros_logs.sh": _CORE_OWE,
    "tools/verify_decode.py": _CORE_OWE,
    "tools/traj_report.py": "早期轨迹报告器，ag_core 诊断提示与 docs/probe 引用，未随仓",
    "tools/car_probe_cmdvel.sh": "automations 记忆引用的早期探测脚本，未随仓",
    # --- docs/probe 现场排查记录里的历史欠账 ---
    "tools/car_cleanup.sh": _DOCS_OWE,
    "tools/car_inventory.sh": _DOCS_OWE,
    "tools/car_preflight.sh": _DOCS_OWE,
    "tools/car_probe_circle.sh": _DOCS_OWE,
    "tools/car_restore_official.sh": _DOCS_OWE,
    "tools/car_stage2b_guard_proof.sh": _DOCS_OWE,
}

# 前置否定：`~/drone_tools/localization_watchdog.py` 是**机载远端路径**，不是本仓
# tools/ 的引用；同理别的交付包里的 `<包名>/tools/<脚本>` 也不该算进来。
# 不加这层就会把它们误判成幽灵引用。
_TOOL_REF = re.compile(r"(?<![A-Za-z0-9_./])tools/[A-Za-z0-9_]+\.(?:py|sh)")
_SCAN_SUFFIX = {".py", ".sh", ".md", ".yaml", ".yml", ".txt"}
# 本文件自己不扫：它的欠账清单与注释里必然出现 tools/ 字面量，扫了就是自指。
_SKIP_FILES = {Path(__file__).name}

# 归档/第三方材料整个目录不扫。理由不是"它们不重要"，而是**它们内部的 tools/
# 引用指的根本不是本仓**：
#   docs/成员交接/、reference/members/  队员交接材料，逐字保存的别线工程
#                                       （官方 dzrobot 车端、胡锦宇 dzmission 等），
#                                       里面的 tools/sim.py、tools/config_loader.py
#                                       是**他们那棵树**的脚本
#   reference/                          官方 dzpx4/dzuav 归档版与车端官方源码，
#                                       部署文档§六明令"不部署、不作为协议真源"
#   docs/archive/、tools/attic/         本仓历史归档，只增不改
# 扫它们会造成一台**永久跑步机**：每收编一份交接材料就冒出一批新幽灵、CI 就红一次，
# 只能靠往 KNOWN_MISSING_TOOLS 逐条登记来消音——胡锦宇（2026-10-18）与余秉桓
# （2026-10-19）两次交接各触发一轮，第二轮直接把 PR 门禁（AGENTS.md：CI 双轮绿）
# 挡死。而本测试真正要守的是"**我们的**文档/报错别把操作员指挥去追幽灵"，
# 归档材料不会指挥我们的操作员去任何地方。
# 实测口径（2026-10-19，枢纽 9bb9140）：全量扫 89 个引用/46 个缺失；
# 排除归档后 56 个引用/13 个缺失，且 13 个**全部已登记**——排除归档不放过任何活引用。
_ARCHIVE_PREFIXES = (
    "docs/成员交接/",
    "docs/archive/",
    "reference/",
    "tools/attic/",
)


def _referenced_tool_paths() -> dict[str, set[str]]:
    hits: dict[str, set[str]] = {}
    for p in REPO.rglob("*"):
        if ("__pycache__" in p.parts or p.suffix not in _SCAN_SUFFIX
                or p.name in _SKIP_FILES):
            continue
        rel = p.relative_to(REPO).as_posix()
        if rel.startswith(_ARCHIVE_PREFIXES):
            continue
        try:
            text = p.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        for m in _TOOL_REF.finditer(text):
            hits.setdefault(m.group(0), set()).add(rel)
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
    """欠账清单不许自己发霉：哪天脚本补上了，就该从清单里删掉。

    第二个发霉方向同样要拦：**引用方消失了，登记项却还留着**。清单一旦只增不减，
    就会退化成一份没人敢动的坟场，后来者读到会以为那些还是活欠账。
    归档目录不再纳入扫描后（见 _ARCHIVE_PREFIXES），只被归档材料引用的登记项
    必须一并清掉。
    """
    now_present = sorted(k for k in KNOWN_MISSING_TOOLS if (REPO / k).exists())
    assert not now_present, (
        f"这些脚本已经存在了，请从 KNOWN_MISSING_TOOLS 移除: {now_present}")

    still_referenced = set(_referenced_tool_paths())
    orphaned = sorted(k for k in KNOWN_MISSING_TOOLS if k not in still_referenced)
    assert not orphaned, (
        "这些登记项在扫描范围内已经没有任何引用方（脚本仍不存在，但也没人再指向它），"
        "请从 KNOWN_MISSING_TOOLS 移除，别让欠账清单发霉:\n  "
        + "\n  ".join(orphaned))


# 同一条失败类的另一半：tools/ 的幽灵引用由上面守着，**docs/*.md 的悬空引用**
# 此前没人管。2026-10-21 现场踩到：给 docs/无人机工作流程.md §7.1 补分级说明时
# 顺手写了一句「完整分级见 docs/上机脚本群.md」，而那个文件当时还不存在——
# 操作手在现场按图索骥，翻不到文件只会怀疑自己拿到的部署包不完整。
#
# 扫描范围**故意只取操作手会被指挥到的那一层**：docs/ 顶层 .md + tools/ 脚本。
# 不收 docs/probe/（现场排查流水记录，逐字留档、里面的路径常是当时那台机器的）、
# docs/成员交接/、docs/archive/、reference/（同 _ARCHIVE_PREFIXES 的理由：
# 那些材料里的路径指的根本不是本仓），也不收 AGENTS.md —— 它第 31 行的
# `-Files "docs/成果.md,tools/脚本.py"` 是**演示参数语法的占位符**，不是断言
# 该文件存在；收进来就是一条永远红、只能靠登记消音的门（见 _ARCHIVE_PREFIXES
# 注释里那台"永久跑步机"）。
_DOC_REF = re.compile(r"docs/[^\s`'\"()\[\]<>，。、；：*]+\.md")
_DOC_REF_SOURCES = ("docs/*.md", "tools/*.py", "tools/*.sh")


def _referenced_doc_paths(root: Path = REPO,
                          patterns=_DOC_REF_SOURCES) -> dict[str, set[str]]:
    hits: dict[str, set[str]] = {}
    for pat in patterns:
        for p in sorted(root.glob(pat)):
            if not p.is_file():
                continue
            rel = p.relative_to(root).as_posix()
            text = p.read_text(encoding="utf-8", errors="replace")
            for m in _DOC_REF.finditer(text):
                target = m.group(0)
                # 模板名（docs/进度报告-YYYYMMDD.md）与尖括号占位不是断言，
                # 它们指的是"你将来会建的那个文件"。通配符 * 已在字符类里排除。
                if "YYYY" in target or "<" in target:
                    continue
                hits.setdefault(target, set()).add(rel)
    return hits


def test_doc_references_resolve():
    """操作手册里让人去读的文档必须真的存在。

    与 test_tool_references_resolve 是同一类坏：在你需要答案的时刻，
    把你指挥去一个不存在的地方。区别只是幽灵从脚本换成了文档。
    """
    refs = _referenced_doc_paths()
    assert refs, "一个 docs/*.md 引用都没扫到，正则或扫描范围坏了"
    dangling = {k: v for k, v in refs.items() if not (REPO / k).exists()}
    assert not dangling, (
        "以下 docs/*.md 被操作手会读到的文档/脚本引用，但文件不存在"
        "（要么把文档补上，要么改掉引用）：\n"
        + "\n".join(f"  {k}\n      <- {', '.join(sorted(v))}"
                    for k, v in sorted(dangling.items())))


def test_doc_reference_scan_is_not_vacuous():
    """上面那条测试必须真的在看东西，而不是靠一个空集合永远绿。

    两道确认：① 分级说明那条**已知活引用**确实在扫描结果里、且目标文件存在；
    ② 在临时树里造一条悬空引用，扫描器抓得到。少了 ②，正则里错一个字符类
    就会静默退化成"零引用零悬空"的永绿门——而 ① 只能证明它扫到了东西，
    证明不了它对**缺失**敏感。
    """
    refs = _referenced_doc_paths()
    assert "docs/上机脚本群.md" in refs, (
        "docs/无人机工作流程.md §7.1 指向 docs/上机脚本群.md 的那条引用没被扫到"
        "——扫描范围或正则坏了")
    assert (REPO / "docs/上机脚本群.md").is_file(), (
        "docs/上机脚本群.md 不存在：分级说明（L0~L4「什么输出证明什么」）无处可查")


def test_doc_reference_scanner_flags_a_forged_dangling_ref(tmp_path):
    """反向确认：扫描器对"引用了但文件不在"必须敏感。

    在临时树里跑，不碰真仓。同时钉住三条排除规则**没有**过宽：
    模板名（YYYY）与尖括号占位要放过，通配符与真悬空要抓住。
    """
    (tmp_path / "docs").mkdir()
    (tmp_path / "docs" / "手册.md").write_text(
        "见 docs/存在的.md 与 docs/不存在的.md。\n"
        "模板 docs/进度报告-YYYYMMDD.md 不算断言。\n"
        "占位 docs/<你的方案>.md 也不算。\n"
        "通配 docs/*.md 更不算。\n",
        encoding="utf-8")
    (tmp_path / "docs" / "存在的.md").write_text("x\n", encoding="utf-8")

    refs = _referenced_doc_paths(root=tmp_path)
    assert set(refs) == {"docs/存在的.md", "docs/不存在的.md"}, (
        f"排除规则过宽或过窄，实扫到: {sorted(refs)}")
    assert not [k for k in refs if (tmp_path / k).exists() and k != "docs/存在的.md"]
    assert [k for k in refs if not (tmp_path / k).exists()] == ["docs/不存在的.md"], (
        "扫描器没能把悬空引用挑出来——上面的 test_doc_references_resolve 是永绿门")


# ---------------------------------------------------- 部署链的跨脚本耦合


def _sh_default(script: str, var: str) -> str:
    """取 shell 脚本里 `VAR="${VAR:-默认}"` 的默认值（写法变了就当场报错）。"""
    text = (TOOLS / script).read_text(encoding="utf-8", errors="replace")
    m = re.search(rf'^{var}="\$\{{{var}:-(.+?)\}}"', text, re.M)
    assert m, (
        f"{script} 里 {var} 不再是 {var}=\"${{{var}:-...}}\" 写法，"
        f"本文件与 tools/preflight.py 的解析器都会静默取不到值——请一并改")
    return m.group(1)


def _sh_plain(script: str, var: str) -> str:
    """取 shell 脚本里 `VAR=字面值` 形式的赋值（无 ${VAR:-} 包装）。"""
    text = (TOOLS / script).read_text(encoding="utf-8", errors="replace")
    m = re.search(rf'^{var}=([A-Za-z0-9_.+-]+)\s*$', text, re.M)
    assert m, f"{script} 里找不到 {var}=<字面值> 形式的赋值"
    return m.group(1)


def test_convert_and_run_agree_on_engine_names():
    """重转出来的引擎名 = 启动脚本要找的名字 = L3 验证用的名字。

    三个脚本各自写死一份引擎名，中间没有任何东西把它们连起来。改一边忘改
    另一边的后果是：换机/重刷后跑 orin_convert_engines.sh 一切正常，接着
    orin_run_dzuav.sh 报"找不到引擎文件"——而比赛当天没有第二次重转的机会。
    第三个（orin_build_verify.sh）漂移更阴：L3 会拿另一个引擎验出"通过"，
    让人以为机器能上场。
    """
    def loaded(script, var):
        return _sh_default(script, var).rsplit("/", 1)[-1]

    produced = {
        "hbb": _sh_default("orin_convert_engines.sh", "HBB_ENGINE_NAME"),
        "cell": _sh_default("orin_convert_engines.sh", "CELL_ENGINE_NAME"),
    }
    run_sh = {
        "hbb": loaded("orin_run_dzuav.sh", "HBB_ENGINE"),
        "cell": loaded("orin_run_dzuav.sh", "CELL_ENGINE"),
    }
    verify = {
        "hbb": _sh_plain("orin_build_verify.sh", "CANON_HBB_ENGINE"),
        "cell": _sh_plain("orin_build_verify.sh", "CANON_CELL_ENGINE"),
    }
    assert produced == run_sh == verify, (
        "引擎名三处不一致，重转的产物启动脚本找不到、L3 也可能验错引擎：\n"
        f"  orin_convert_engines.sh 产出: {produced}\n"
        f"  orin_run_dzuav.sh      加载: {run_sh}\n"
        f"  orin_build_verify.sh   L3 用: {verify}")


def test_preflight_engine_parser_matches_run_script(probe):
    """preflight 的引擎名派生器必须与脚本同步——它派生失败时会拒绝判定。

    引擎检查从"数个数"改成"按名字查"之后，preflight 自己也要读同一份真源。
    解析器悄悄取空会让 check_orin 打出"无法解析…本项拒绝判定"的红灯，
    比原来的假绿灯安全，但一样是坏的自检。
    """
    dflt = probe["__engine_defaults__"]
    assert isinstance(dflt, dict), f"preflight.run_dzuav_defaults 调用失败: {dflt}"
    assert dflt.get("ENGINES"), "preflight 取不到引擎目录，check_orin 会误判红"
    assert {k: v.rsplit("/", 1)[-1] for k, v in dflt.items() if k != "ENGINES"} == {
        "HBB_ENGINE": _sh_default("orin_run_dzuav.sh", "HBB_ENGINE").rsplit("/", 1)[-1],
        "CELL_ENGINE": _sh_default("orin_run_dzuav.sh", "CELL_ENGINE").rsplit("/", 1)[-1],
    }


# ------------------------------------------------ 相机标定基准 vs 采集分辨率


def _load_calibrate_camera():
    spec = importlib.util.spec_from_file_location(
        "_calibrate_camera", TOOLS / "calibrate_camera.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    try:
        spec.loader.exec_module(mod)
    finally:
        sys.modules.pop(spec.name, None)
    return mod


def _write_calib(dirpath: Path, w: int, h: int) -> None:
    import yaml

    dirpath.mkdir(parents=True, exist_ok=True)
    (dirpath / "calibration.yaml").write_text(yaml.safe_dump({
        "fx": 1234.5, "fy": 1236.25, "cx": 950.0, "cy": 530.5,
        "dist": [-0.04, -0.003, -0.009, 0.0007, 0.0],
        "width": w, "height": h, "rms_px": 0.31, "n_views": 22,
        "pattern": "9x6", "square_mm": 25.0, "date": "2026-10-20 00:00:00",
    }), encoding="utf-8")


class _Args:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def test_capture_size_default_is_derived_from_deploy_spec():
    """capture --size 的默认值必须是机上真正会采集的分辨率。

    曾经默认 1920x1200 而部署线是 1920x1080：照默认标定再 inject，
    MapLocalizer 的基准分母被改成 1200，机上仍按 1080 采集，
    scaledIntrinsics 就把 fy/cy 乘 0.9 —— 定位系统性偏 11%，而 L1/L2/L3
    全绿（三级验证都不碰内参）。默认值改成从脚本派生，杜绝第四份硬编码。
    """
    cc = _load_calibrate_camera()
    sizes = cc.deploy_frame_sizes()
    assert sizes, "从 orin_bringup.sh / orin_run_dzuav.sh 一个采集分辨率都没解析出来"
    assert len(set(sizes.values())) == 1, (
        f"两个启动脚本给的采集分辨率不一致，标定基准无从对齐: {sizes}")
    w, h = next(iter(sizes.values()))
    assert cc._default_capture_size() == f"{w}x{h}"

    # argparse 上挂的默认值也要是它（防止只在函数里改了、参数上还是写死的）
    out = subprocess.run(
        [sys.executable, str(TOOLS / "calibrate_camera.py"), "capture", "--help"],
        capture_output=True, text=True, encoding="utf-8", errors="replace",
        timeout=120, cwd=str(REPO))
    assert out.returncode == 0, out.stderr
    assert f"{w}x{h}" in out.stdout, (
        f"capture --help 里看不到派生出的默认分辨率 {w}x{h}:\n{out.stdout}")


_SPEC_RE = re.compile(r"^(\d+)x(\d+)(?:@(\d+))?$")


def _cpp_parse_spec_accepts(spec: str) -> bool:
    """main.cpp parseSpec 的等价判据：语法匹配 + 回写比对一致。

    C++ 侧先 sscanf("%dx%d[@%d]")，再把解析出的 w/h/f 用 snprintf 回写成规范串
    与入参逐字比对，不等就拒绝——尾部残余字符与前导零都过不了这一关。
    """
    m = _SPEC_RE.match(spec)
    if not m:
        return False
    w, h, fps = int(m.group(1)), int(m.group(2)), m.group(3)
    if w <= 0 or h <= 0:
        return False
    canon = f"{w}x{h}" if fps is None else f"{w}x{h}@{int(fps)}"
    return canon == spec


def test_deploy_camera_specs_are_accepted_by_parse_spec():
    """启动脚本默认传给 dzuav 的相机 spec 必须能被 main.cpp 的 parseSpec 接受。

    parseSpec 改为严格解析：解析不了就 return 2 退出，不再静默沿用默认值
    （原写法要求 sscanf 恰好匹配 3 项，于是漏写 @fps 的 "1920x1080" 会让宽高
    一起被丢弃）。变严之后，脚本里任何一个不合语法的 spec 都会让 dzuav 直接退出、
    整局零情报，所以这里把工具链上真正会传下去的默认字面量全查一遍。
    """
    found = {}
    for script, var, pat in (
        ("orin_run_dzuav.sh", "size_fps", r'^\s*size_fps="\$\{3:-([^}"]+)\}"'),
        ("orin_bringup.sh", "DZUAV_SPEC", r'^DZUAV_SPEC="\$\{DZUAV_SPEC:-([^}"]+)\}"'),
    ):
        m = re.search(pat, (TOOLS / script).read_text(encoding="utf-8", errors="replace"), re.M)
        assert m, f"{script} 里解析不到 {var} 的默认值（正则或脚本改了？）"
        found[f"{script}:{var}"] = m.group(1)

    bad = {k: v for k, v in found.items() if not _cpp_parse_spec_accepts(v)}
    assert not bad, (
        f"这些默认 spec 会被 parseSpec 拒绝，dzuav 直接 return 2: {bad}；"
        f"合法形式为 WxH 或 WxH@fps")


def test_parse_spec_rejects_malformed_and_silent_default_forms():
    """钉住 parseSpec 的严格性本身：这些形式必须被拒，不能悄悄退回默认值。"""
    for bad in ("", "garbage", "1920x1080@30xyz", "1920x1080@", "0x1080@30",
                "1920x1080@30 ", "01920x1080@30", "1920X1080@30", "1920x1080@-1"):
        assert not _cpp_parse_spec_accepts(bad), f"{bad!r} 不该被接受"
    for good in ("1920x1080@30", "1920x1080", "1280x720@60", "640x480"):
        assert _cpp_parse_spec_accepts(good), f"{good!r} 该被接受"


def _main_cpp_text() -> str:
    return (REPO / "onboard" / "dzuav" / "src" / "main.cpp").read_text(
        encoding="utf-8", errors="replace")


def test_main_cpp_parse_spec_is_strict():
    """源码级确认 parseSpec 真的返回 bool、调用方检查了它、且回写比对在拒绝分支里。

    只验 Python 镜像不够——镜像与 C++ 一起漂移时镜像测试照样全绿
    （见 tests/test_hbb_postprocess.py 的同类教训）。
    """
    text = _main_cpp_text()
    assert re.search(r"^bool parseSpec\(", text, re.M), "parseSpec 不再是 bool 返回"
    assert re.search(r"if \(!parseSpec\(spec, width, height, fps\)\)", text), (
        "runCamera 没有检查 parseSpec 的返回值——退回静默默认值就等于没修")

    # 回写比对必须**真的在拒绝条件里**。突变测试实测：只断言 "snprintf" in text
    # 时，把 `|| spec != canon` 整段删掉照样全绿——因为 snprintf 调用还在，
    # 但它的返回值再没跟入参比过，"1920x1080@30xyz" 会被静默接受成 1920x1080@30。
    guard = re.search(r"if \(len[^;]*?\)\s*\n\s*return false;", text, re.S)
    assert guard, "找不到 parseSpec 里以 len 开头的拒绝分支（回写比对被删了？）"
    assert "spec != canon" in guard.group(0), (
        "parseSpec 的回写结果没有与入参比对——尾部残余字符/前导零会被静默接受")

    # 回写用的格式串必须与解析用的逐字一致，否则比的是两套语法，比对本身失效。
    assert re.search(r'sscanf\(spec\.c_str\(\), "%dx%d@%d", &w, &h, &f\)', text), (
        "带 fps 的解析格式串变了，回写比对随之失配")
    assert re.search(r'snprintf\(canon, sizeof canon, "%dx%d@%d", w, h, f\)', text), (
        "带 fps 的回写格式串与 sscanf 不一致")
    assert re.search(r'snprintf\(canon, sizeof canon, "%dx%d", w, h\)', text), (
        "省略 fps 时的回写格式串与 sscanf 不一致")


def test_offline_video_output_dir_is_optional_without_cwd_flood():
    """省略 output_dir 时不得把标注图写进当前目录。

    processFrame 用 `annotate = !output.empty()` 决定要不要落盘，而
    `path("") / "x_0.jpg"` == "x_0.jpg" 是非空相对路径——原写法会在
    "不带 output_dir 跑离线视频"（正是 usage 里的可选参数用法）时每帧写一张
    JPEG 到 CWD，3 分钟素材 ≈ 5400 张，Orin 上足以把盘写满。
    """
    text = _main_cpp_text()
    assert re.search(r"output_dir\.empty\(\)\s*\?\s*std::filesystem::path\{\}", text), (
        "outputFor 没有对空 output_dir 返回空路径；省略该参数会往 CWD 刷标注图")


def test_camera_reopen_is_bounded_probed_and_backed_off():
    """断流重开五条不变量：先 release、退避、试读一帧才算成功、有界、不用 break。

    这一段原来一次重开失败就 `return 4`（整局再无情报），且只看 isOpened()——
    与本文件初次打开时的判据自相矛盾（那里明写"not-negotiated 时 isOpened
    也可能是 true"）。假成功的重开会让 empty_frames 再攒满 300 帧（≈10s）
    才进下一次重试，三次共 ≈30s，占 180s 硬时限的六分之一。
    """
    text = _main_cpp_text()
    start = text.index("if (empty_frames >= 300)")
    block = text[start:text.index("continue;", start)]   # 重开分支整体，到外层 continue 为止
    # 结构断言一律在**去掉行注释**之后做：这段的注释本身就在讲"此处不能 break"，
    # 直接 `"break" not in block` 会被自己的说明文字判红。
    # 简单剥 `//…$` 够用——本块字符串字面量里没有 `//`。
    code = re.sub(r"//[^\n]*", "", block)

    assert "break" not in code, (
        "重开分支里出现 break：外层就是 while(g_running)，break 会直接结束整局侦察")
    assert code.index("capture.release()") < code.index("capture.open("), (
        "没有先 release 就 open 新管道：可能打开失败，也会泄漏旧的 GStreamer 管道")
    assert re.search(r"usleep\(200000 \* reopen_attempts\)", code), (
        "退避不随重试次数增长——连打三次会挤在同一瞬间，USB 还没枚举完就再试")
    assert re.search(
        r"capture\.isOpened\(\) && capture\.read\(probe\) && !probe\.empty\(\)", code), (
        "重开成功判据退回只看 isOpened()：not-negotiated 时会假成功，"
        "与初次打开的判据不一致")
    # empty_frames 只能在**真出了帧**之后清零；清零若发生在重开之前，
    # 每次失败都要再攒 300 空帧才重试。
    assert code.count("empty_frames = 0;") == 1, (
        "重开分支里 empty_frames 清零的次数变了（应恰好一次，在成功分支内）")
    assert code.index("empty_frames = 0;") > code.index("capture.open("), (
        "empty_frames 在重开尝试之前就被清零——失败一次要多烧 ≈10s")
    assert "return 4;" in code, "重试耗尽后没有非零退出，会变成无声空转"
    # 收到好帧后必须把两个计数一起归还。锚点从 `if (empty_frames >= 300)` 之后
    # 起算并先剥注释：整文件里 `unsigned reopen_attempts = 0;` 的声明行同样带
    # 尾注释，直接全文搜 `reopen_attempts = 0;\s*//` 会命中声明，把"这行被注释
    # 掉了"判成通过（突变测试实测存活）。
    after = re.sub(r"//[^\n]*", "", text[text.index("if (empty_frames >= 300)"):])
    assert re.search(r"^\s*empty_frames = 0;\s*\n\s*reopen_attempts = 0;", after, re.M), (
        "收到好帧后没有归还重试额度：一次恢复过的掉线会永久吃掉整局的重试次数")


def test_camera_checks_negotiated_size_against_requested():
    """请求到的 caps 不等于协商到的 caps，必须对账并喊出来。

    候选 pipeline 那一圈只保证"能出帧"，不保证出的是请求的尺寸；GStreamer
    可以悄悄降档。内参按帧分辨率缩放（MapLocalizer::scaledIntrinsics），
    尺寸一变定位就系统性偏移，而 L1/L2/L3 三级自检都不碰内参 —— 全绿。
    这一类偏差在 2026-10-20 已经真发生过一次（标定按 1200 出、机上按 1080 采）。
    """
    text = _main_cpp_text()
    # 结束锚点必须从 start 之后找：std::array<float, kObjectClassCount> 在
    # processFrame 里也出现过，全文 index() 会取到那一处，切出空串。
    start = text.index("cv::VideoCapture capture;")
    region = text[start:text.index("std::array<float, kObjectClassCount>", start)]

    # 尺寸必须取自**试读到的那一帧**，不是 CAP_PROP_*：后者报的可能仍是请求的
    # caps，恰恰是要对账的那一侧，拿它自己跟自己比等于没比。
    assert re.search(r"got = probe\.size\(\);", region), (
        "没有记录实际出帧尺寸；无法发现 pipeline 悄悄降档")
    assert "CAP_PROP_FRAME_WIDTH" not in region, (
        "用 CAP_PROP_FRAME_WIDTH 对账等于拿请求值跟自己比，查不出降档")
    assert region.index("got = probe.size();") < region.index("negotiated frame size:"), (
        "先打印再取尺寸：打出来的是默认构造的 0x0")
    assert re.search(r"if \(got\.width != width \|\| got\.height != height\)\s*\n"
                     r"\s*std::cerr << \"WARNING: negotiated ", region), (
        "实际尺寸与请求不一致时没有告警——降档会静默改变内参缩放系数")
    # 打印必须无条件执行：一致时也要留下证据，否则事后无法判断当时到底协商成了什么。
    assert re.search(r"std::cout << \"negotiated frame size: \"", region), (
        "没有把协商到的尺寸打进日志")


def test_offline_video_loop_is_interruptible_and_bounded():
    """离线视频循环三条不变量：可 Ctrl-C、单帧异常不毁整段、连续异常有界退出。

    离线复盘是验证检测/定位的主要手段：一帧异常就 return 会让整段素材被单个坏帧
    终结；完全不设上限则会在持久故障下无限空转。计数上限与相机分支共用
    kMaxFrameErrors——两处各写一个字面量 30 迟早分叉。
    """
    text = _main_cpp_text()
    # 离线分支在 main() 末尾，outputFor 之后再没有相机分支那套同名结构，
    # 所以从这里切片就能把断言限定在离线循环上（相机分支另有自己的 try/catch）。
    region = text[text.index("const auto outputFor"):]

    assert re.search(
        r"for \(size_t index = 0; g_running\.load\(\) && capture\.read\(frame\)", region), (
        "离线视频循环不检查 g_running；SIGINT 只置标志不终止进程，于是 Ctrl-C 停不下来")
    assert len(re.findall(r"\btry\b", region)) == 1, (
        "离线循环的 try 保护数量变了——多帧异常处理必须只有一处，否则口径不一致")
    assert re.search(
        r"try\s*\{\s*processFrame\(frame, hbb, cell, map, udp, outputFor\(index\)\);"
        r"\s*frame_errors = 0;", region), (
        "processFrame 不在 try 内，或成功后没有清零 frame_errors："
        "前者让单帧异常终结整段素材，后者让偶发异常一路累积到上限后误退出")
    assert re.search(r"catch \(const std::exception& e\)\s*\{\s*\+\+frame_errors;", region), (
        "catch 里没有累加连续异常计数——异常会被无声吞掉，故障永远不上报")
    assert re.search(r"if \(frame_errors >= kMaxFrameErrors\)\s*\n\s*return 5;", region), (
        "离线循环没有非零退出上限；持久故障会变成无限空转而不是失败退出")
    assert re.search(r"^constexpr unsigned kMaxFrameErrors", text, re.M), (
        "kMaxFrameErrors 常量不见了（相机分支与离线分支必须共用同一上限）")
    assert len(re.findall(r"frame_errors >= kMaxFrameErrors", text)) == 2, (
        "相机分支与离线分支应各自用 kMaxFrameErrors 收口，实际命中数变了："
        "少一处=有一条路径无界空转，多一处=上限口径又分叉了")


def test_inject_refuses_calibration_basis_mismatch(tmp_path):
    """基准与采集分辨率不一致时 inject 必须拒绝，而不是静默改坏内参。"""
    cc = _load_calibrate_camera()
    w, h = next(iter(cc.deploy_frame_sizes().values()))
    bad_h = h + 120                      # 1080 -> 1200，正是历史事故那一档
    _write_calib(tmp_path, w, bad_h)
    cc.CALIB_DIR = tmp_path
    cc.CPP = tmp_path / "MapLocalizer_copy.cpp"
    cc.CPP.write_text((REPO / "onboard" / "dzuav" / "src" / "MapLocalizer.cpp")
                      .read_text(encoding="utf-8"), encoding="utf-8")
    original = cc.CPP.read_text(encoding="utf-8")

    with pytest.raises(SystemExit) as exc:
        cc.inject(_Args(allow_basis_mismatch=False))
    msg = str(exc.value)
    assert f"{w}x{bad_h}" in msg and f"{w}x{h}" in msg, msg
    assert f"{h}/{bad_h}" in msg, f"没把 fy 的缩放比讲清楚: {msg}"
    assert cc.CPP.read_text(encoding="utf-8") == original, "拒绝注入却已经改了文件"

    # 显式覆盖必须放行（现场可能要故意用别的基准）
    assert cc.inject(_Args(allow_basis_mismatch=True)) == 0


def test_inject_updates_every_patch_point(tmp_path):
    """inject 要连缩放分母和那句"基准 WxH"注释一起改。

    只改 K_ 不改分母 = 内参与基准错配；改了分母不改注释 = 下一个人读到的是
    假信息。少一处都算没注入成功。
    """
    cc = _load_calibrate_camera()
    w, h = next(iter(cc.deploy_frame_sizes().values()))
    _write_calib(tmp_path, w, h)
    cc.CALIB_DIR = tmp_path
    cc.CPP = tmp_path / "MapLocalizer_copy.cpp"
    cc.CPP.write_text((REPO / "onboard" / "dzuav" / "src" / "MapLocalizer.cpp")
                      .read_text(encoding="utf-8"), encoding="utf-8")

    assert cc.inject(_Args(allow_basis_mismatch=False)) == 0
    text = cc.CPP.read_text(encoding="utf-8")
    assert "1234.5, 0, 950.0" in text, "fx/cx 没注入"
    assert "0, 1236.25, 530.5" in text, "fy/cy 没注入"
    assert f"sz.width) / {w}" in text, "宽分母没跟着改"
    assert f"sz.height) / {h}" in text, "高分母没跟着改"
    assert f"（基准 {w}x{h}）" in text, "注释里的基准分辨率没跟着改"


# =====================================================================
# L4 相机验收脚本（tools/orin_camera_smoke.sh）与部署闸门接线
#
# 这两块的存在理由：orin_build_verify.sh 的 L3 走的是 `dzuav <图片> <hbb>
# <cell> <port>` 离线分支，**从不进 runCamera()**。相机路径上的修改（spec
# 严格解析、有界重开、协商尺寸对账）因此永远停在"已写待验"，没有任何一级
# 自检能把它推到"已修"。L4 就是补这一级；而它自己也得被测——一个没被验过的
# 验收脚本，比没有验收脚本更危险，因为它会发结论。
# =====================================================================
SMOKE_SH = "orin_camera_smoke.sh"


def _smoke_text() -> str:
    return (TOOLS / SMOKE_SH).read_text(encoding="utf-8", errors="replace")


def _bash_array(text: str, name: str) -> list[str]:
    """抠出 shell 里 `NAME=( ... )` 的字符串项（每项一行的写法）。

    行尾注释、以及跨行注释的续行都不是以引号开头，正则自然跳过。
    """
    m = re.search(rf"^{name}=\(\n(.*?)^\)\n", text, re.S | re.M)
    assert m, f"{SMOKE_SH} 里找不到数组 {name}（写法变了？本解析器要同步）"
    out = [q.group(1) for q in
           (re.match(r'^\s*"((?:[^"\\]|\\.)*)"', ln) for ln in m.group(1).splitlines())
           if q]
    assert out, f"{name} 解析出来是空的——正则或数组写法变了"
    return out


def _expected_spec_echo(spec: str, caller_default_fps: int = 30) -> str:
    """main.cpp 打印的 `camera spec: WxH@FPS` 应该长什么样。

    省略 @fps 时 parseSpec **不写** fps，沿用 runCamera 的初值 30
    （main.cpp:344），所以回显与输入不同；fps=0 是合法的一档，原样回显
    （两个 pipeline builder 都是 `if (fps > 0)` 才往 caps 里塞 framerate=）。
    """
    m = _SPEC_RE.match(spec)
    assert m, f"{spec!r} 不是合法 spec，不该拿它算期望回显"
    w, h, fps = int(m.group(1)), int(m.group(2)), m.group(3)
    return f"{w}x{h}@{caller_default_fps if fps is None else int(fps)}"


def test_camera_smoke_bad_specs_are_rejected_by_cpp_mirror():
    """验收脚本的负例表必须真的会被 parseSpec 拒。

    两张表各写各的、中间没有联系时，最坏的情况是负例里混进一个 C++ 其实会
    放行的写法：机上那条断言当场红，而操作手会以为是相机坏了，去查线查驱动。
    """
    bad = _bash_array(_smoke_text(), "BAD_SPECS")
    assert len(bad) >= 8, f"负例表被削到 {len(bad)} 条，覆盖变薄了"
    accepted = [s for s in bad if _cpp_parse_spec_accepts(s)]
    assert not accepted, f"这些『坏 spec』其实会被 parseSpec 放行: {accepted}"


def test_camera_smoke_good_specs_and_echoes_match_cpp_mirror():
    """正例表的每一条：spec 要能过 parseSpec，且期望回显逐字对得上。"""
    good = _bash_array(_smoke_text(), "GOOD_SPECS")
    assert len(good) >= 6, f"正例表被削到 {len(good)} 条"
    for pair in good:
        assert "|" in pair, f"正例项 {pair!r} 不是『输入|期望回显』两段"
        spec, want = pair.split("|", 1)
        assert _cpp_parse_spec_accepts(spec), f"合法 spec {spec!r} 会被 parseSpec 拒"
        assert want == _expected_spec_echo(spec), (
            f"{spec!r} 的期望回显写成 {want!r}，按 main.cpp 应是 "
            f"{_expected_spec_echo(spec)!r}")


def test_camera_smoke_proves_default_fps_writeback_not_initial_value():
    """省略 @fps 的正例里，至少要有一条宽高**不等于** runCamera 的初值。

    runCamera 的初值恰好是 1920x1080@30。只用 1920x1080 当正例的话，
    "宽高真被 parseSpec 写进去了"与"没写、碰巧等于初值"输出一模一样，
    断言绿着也证明不了任何事——正是 2026-10-20 那个 bug 的观测盲区。
    """
    no_fps = [p.split("|", 1) for p in _bash_array(_smoke_text(), "GOOD_SPECS")
              if "@" not in p.split("|", 1)[0]]
    assert no_fps, "正例表里没有省略 @fps 的项了，默认帧率行为无人验"
    distinct = [s for s, _ in no_fps if not s.startswith("1920x1080")]
    assert distinct, (
        f"省略 @fps 的正例全是 1920x1080（{[s for s, _ in no_fps]}），"
        f"与 runCamera 初值同宽高，验不出宽高回写")
    for spec, want in no_fps:
        assert want.endswith("@30"), (
            f"省略 @fps 时应回显调用方默认 30，{spec!r} 写的是 {want!r}")


def test_camera_smoke_covers_the_documented_exit_codes():
    """main.cpp 的退出码语义表要在验收脚本里都有**判据**，不能只在文案里提一嘴。

    2 用法错误 / 3 打不开相机或离线输入 / 4 重开次数耗尽 / 5 连续帧异常超限。

    断言全部落在真实的比较语句与 grep 模式表上。只查"文中出现 rc=4 字样"是
    不够的：脚本头部注释里就写着 rc=4，把 E3 的比较整段删掉那条断言照样绿
    ——变异测试实测过这个假绿。
    """
    text = _smoke_text()

    # rc=2 / rc=3：必须真的有 run_expect 拿它当期望值
    wants = set(re.findall(r'run_expect\s+"[^"]*"\s+"([^"]*)"', text))
    assert "2" in wants, f"没有任何 run_expect 断言 rc=2（用法错误）: {sorted(wants)}"
    assert "3" in wants, f"没有任何 run_expect 断言 rc=3（打不开）: {sorted(wants)}"

    # rc=4：E3 必须把实测退出码与 4 比较
    assert re.search(r'\[\[\s*"\$ERC"\s*==\s*"4"\s*\]\]', text), (
        "E3 没有把实测退出码与 4 比较——重开耗尽的兜底退出无人验")

    # 重开上限必须与 3 比较（对齐 main.cpp 的 reopen_attempts < 3）
    assert re.search(r'-gt 3\b', text), "重开次数没有与上限 3 比较"

    # rc=5 的日志判据必须在 D6 的"不该出现"模式表里
    d6 = re.search(r"D6_BAD=0\n(.*?)\n\s*\[\[ \$D6_BAD == 0 \]\]", text, re.S)
    assert d6, "找不到 D6 的故障模式扫描块"
    for pat in ("camera reopen failed", "camera stream did not recover",
                "too many consecutive frame errors", "frame processing error"):
        assert pat in d6.group(1), f"D6 的故障模式表里少了「{pat}」"


def test_camera_smoke_scopes_every_assertion_to_its_own_run():
    """run_expect 必须**先打点再执行**，否则 grep_log 读到上一轮的残留。

    本脚本首版就是这么假绿的：tail_log 从头读整个日志，于是"无参数那次打过的
    usage:"会被后面每一次 grep_log "usage:" 命中——断言看着在验，其实验的是
    上一轮的输出。这类假绿不会自己暴露，只有靠读代码或变异测试才能发现。
    """
    text = _smoke_text()
    body = re.search(r"^run_expect\(\) \{\n(.*?)^\}\n", text, re.S | re.M)
    assert body, "run_expect 的定义形状变了，本断言失效"
    lines = [ln.strip() for ln in body.group(1).splitlines()
             if ln.strip() and not ln.strip().startswith("#")]
    mark = next((i for i, ln in enumerate(lines) if ln.startswith("mark_log")), None)
    call = next((i for i, ln in enumerate(lines) if ln.startswith('"$@"')), None)
    assert mark is not None, "run_expect 里没有 mark_log 打点，日志窗口不隔离"
    assert call is not None, "run_expect 里找不到执行命令的那一行"
    assert mark < call, "打点排在执行命令之后 = 本次输出没被隔离，grep_log 会读到残留"


def test_camera_smoke_log_helpers_slice_from_the_mark():
    text = _smoke_text()
    for fn in ("mark_log", "tail_log", "grep_log", "count_log"):
        assert re.search(rf"^{fn}\(\)", text, re.M), f"缺少日志窗口函数 {fn}"
    assert re.search(r'tail_log\(\) \{ tail -c \+"\$\(\(MARK_BYTES \+ 1\)\)"', text), (
        "tail_log 不再按 MARK_BYTES 截取，窗口隔离失效（见上一条的假绿事故）")


def test_camera_smoke_does_not_hardcode_engine_names():
    """引擎名只能来自 orin_run_dzuav.sh（唯一真源），不许在验收脚本里抄第四份。

    与 test_convert_and_run_agree_on_engine_names 同一条命门：抄一份就会分叉，
    分叉的后果是 L4 拿一个引擎验出"通过"，而 bringup 加载的是另一个。
    """
    text = _smoke_text()
    assert 'eval "$(grep' in text and "orin_run_dzuav.sh" in text, (
        "引擎名不再从 orin_run_dzuav.sh 解析（改成写死了？）")
    leaked = re.findall(r"\b(?:hbb_detector|cell_classifier)[A-Za-z0-9_.-]*\.engine", text)
    assert not leaked, f"验收脚本里写死了引擎名: {sorted(set(leaked))}"


def test_camera_smoke_is_shell_safe():
    """LF 行尾 + bash 语法通过。

    CRLF 的 shebang 是 `#!/bin/bash\\r`，机上执行直接 bad interpreter；
    而这类脚本平时在 Windows 上编辑，最容易悄悄带上 CRLF。
    """
    raw = (TOOLS / SMOKE_SH).read_bytes()
    assert b"\r\n" not in raw, f"{SMOKE_SH} 是 CRLF，上机即 bad interpreter"
    assert raw.startswith(b"#!/usr/bin/env bash"), "shebang 变了"
    bash = shutil.which("bash")
    if not bash:
        pytest.skip("本机没有 bash，跳过语法检查")
    proc = subprocess.run([bash, "-n", str(TOOLS / SMOKE_SH)],
                          capture_output=True, text=True,
                          encoding="utf-8", errors="replace")
    assert proc.returncode == 0, f"bash -n 报语法错误:\n{proc.stderr}"


# ------------------------------------------------------------- 部署闸门接线
def _deploy_ast():
    return ast.parse((TOOLS / "orin_deploy.py").read_text(encoding="utf-8"))


def _deployed_orin_py() -> set[str]:
    """从 orin_deploy.py 的 AST 里取 ORIN_PY_TOOLS 的字面量（不 import，
    免得把 paramiko 变成测试依赖）。"""
    for n in ast.walk(_deploy_ast()):
        if isinstance(n, ast.Assign) and any(
                getattr(t, "id", None) == "ORIN_PY_TOOLS" for t in n.targets):
            return {e.value for e in n.value.elts}
    raise AssertionError("orin_deploy.py 里找不到 ORIN_PY_TOOLS")


# tools/ 下存在、但**不该**随部署上机的 orin_*.py，及其理由。
# 这份表是"故意不推"与"忘了登记"之间唯一的分界线：没有它，一个文件不上机
# 到底是决定还是事故，就只能靠翻聊天记录。
ORIN_PY_NOT_DEPLOYED = {
    "orin_deploy.py": "PC 侧部署器本身，不是机载工具（机上也没有 paramiko）",
    # 下面三个是 repo 线的产物。合并线的 orin_bringup.sh 采用部署现实（fix 线）
    # 的 9+1 窗口版，没有 flight_wd / wdog / 采集窗口，故不推。
    # 出处见 orin_deploy.py 里 ORIN_PY_TOOLS 上方那段 2026-10-18 合并线说明。
    "orin_watchdog.py": "合并线 bringup 无 wdog 窗口",
    "orin_agent_watchdog.py": "合并线 bringup 无 flight_wd 窗口",
    "orin_capture_dataset.py": "合并线 bringup 无采集窗口",
}


def test_orin_py_tools_registers_every_orin_python_tool():
    """tools/ 下每个 orin_*.py 都必须有明确归属：要么登记上机，要么写明不推。

    部署脚本按 `orin_*.sh` 通配推 shell，却按**显式名单**推 python：漏登记的
    后果是该工具静默不上机，而部署照样报"完成"。这个洞本文件的第一版就真踩了
    ——orin_deploy_gate.py 的通过提示让人在机上再跑一次做对账，而它自己不在
    名单里，那条提示是假的。

    不允许第三种状态（既不在名单也不在排除表）：那样"忘了登记"会永远静默。
    """
    listed = _deployed_orin_py()
    present = {p.name for p in TOOLS.glob("orin_*.py")}
    assert present, "tools/ 下没有 orin_*.py 了？"

    unaccounted = present - listed - set(ORIN_PY_NOT_DEPLOYED)
    assert not unaccounted, (
        f"这些机载 Python 工具既没登记进 ORIN_PY_TOOLS、也没写进排除表，"
        f"部署会静默跳过它们: {sorted(unaccounted)}")
    overlap = listed & set(ORIN_PY_NOT_DEPLOYED)
    assert not overlap, f"同时出现在两份名单里，归属矛盾: {sorted(overlap)}"
    phantom = listed - present
    assert not phantom, f"ORIN_PY_TOOLS 里登记了不存在的文件: {sorted(phantom)}"


def test_orin_py_not_deployed_list_has_no_stale_entries():
    """排除表里的文件必须真的还在，且排除理由必须还成立。

    文件删了还留在表里 = 表在腐烂；下一次真漏登记时，没人会认真读一份
    已知过期的名单。
    """
    bringup = (TOOLS / "orin_bringup.sh").read_text(encoding="utf-8", errors="replace")
    for name, reason in ORIN_PY_NOT_DEPLOYED.items():
        assert (TOOLS / name).is_file(), (
            f"{name} 已不存在，请从 ORIN_PY_NOT_DEPLOYED 删掉")
        # 理由是"bringup 没有对应窗口"的，就去 bringup 里查一眼。
        if "bringup" in reason:
            assert name not in bringup, (
                f"{name} 的排除理由是「{reason}」，但 orin_bringup.sh 里出现了它"
                f"——窗口会在机上因为文件没推上去而起不来，请登记进 ORIN_PY_TOOLS")


def test_bringup_only_starts_deployed_python_tools():
    """bringup 起的每个 orin_*.py 都必须在部署名单里。

    上一条的正向版，也是后果最直接的一条：窗口命令引用的文件没上机，tmux
    窗口秒退，而 verify_up 只会在沉降 12 秒后报一句 rc，现场很难第一时间
    想到是"部署名单漏了一个"。
    """
    listed = _deployed_orin_py()
    bringup = (TOOLS / "orin_bringup.sh").read_text(encoding="utf-8", errors="replace")
    referenced = set(re.findall(r"tools/(orin_[A-Za-z0-9_]+\.py)", bringup))
    assert referenced, "orin_bringup.sh 不再引用任何 orin_*.py，本断言失效"
    missing = referenced - listed
    assert not missing, (
        f"bringup 会起这些机载 Python 工具，但它们不在 ORIN_PY_TOOLS 里、"
        f"部署不会推上机，窗口必然起不来: {sorted(missing)}")


def test_deploy_gate_is_registered_for_on_machine_recheck():
    """闸门的通过提示让人在机上再跑一次做对账，那它就必须真的被推上去。

    单独钉住是因为这条最容易被"顺手清理名单"改坏：orin_deploy_gate.py 看起来
    像纯 PC 侧工具（部署前才跑），但机上那份对账同样靠它。
    """
    assert "orin_deploy_gate.py" in _deployed_orin_py(), (
        "orin_deploy_gate.py 不在部署名单里，而它自己的通过提示印着"
        "『部署后在 Orin 上跑 python3 tools/orin_deploy_gate.py』——那条提示是假的")


def test_deploy_runs_source_gate_before_touching_credentials():
    """闸门必须排在 read_password / 任何 sftp 之前。

    排在后面的话：闸门红了也已经输过密码、开过 SSH 连接，甚至可能已经推了
    半个树——"部署前拦截"就名不副实了。用 AST 比行号，不靠字符串顺序猜。
    """
    tree = _deploy_ast()
    main_fn = next(n for n in tree.body
                   if isinstance(n, ast.FunctionDef) and n.name == "main")
    # 取每个名字的**最早**出现行：ast.walk 会走进 main 里嵌套的 def put()，
    # 取最晚一次的话，"闸门排在第一次上传之后"这种真问题会被后一次调用盖掉。
    first: dict[str, int] = {}
    for n in ast.walk(main_fn):
        if not isinstance(n, ast.Call):
            continue
        name = n.func.id if isinstance(n.func, ast.Name) else getattr(n.func, "attr", "")
        if name and name not in first:
            first[name] = n.lineno

    assert "run_source_gate" in first, "orin_deploy.main 没有调用源码闸门"
    gate_line = first["run_source_gate"]
    for later in ("read_password", "sftp_put", "run"):
        if later in first:
            assert gate_line < first[later], (
                f"源码闸门（第 {gate_line} 行）排在 {later}"
                f"（第 {first[later]} 行）之后——拦不住已经发生的动作")


def test_deploy_gate_bypass_is_env_only_and_loud():
    """放行旧树只能走 ALLOW_STALE_SOURCE=1，且必须留下警告。

    与 orin_bringup.sh 的 ALLOW_STRAY、orin_build_verify.sh 的
    ALLOW_ENGINE_FALLBACK 同一套口径：不给"--skip-gate"这种一按就忘的开关，
    放行必须显式、必须在输出里留下痕迹。

    断言落在**读环境变量并比较**那条语句上，不是"文中出现过这个变量名"：
    变异测试实测过，把判断替换成恒假、只留警告文案里的名字，后者照样绿。
    """
    text = (TOOLS / "orin_deploy.py").read_text(encoding="utf-8", errors="replace")
    assert re.search(r'os\.environ\.get\("ALLOW_STALE_SOURCE"\)\s*!=\s*"1"', text), (
        "闸门放行不再由 ALLOW_STALE_SOURCE=1 决定（判断被改掉、或只剩文案提一嘴）")
    assert "--skip-gate" not in text and "--no-gate" not in text, (
        "出现了命令行跳过开关——太容易顺手加上，逃生口只留环境变量")
    assert re.search(r"\[警告\].*ALLOW_STALE_SOURCE", text, re.S), (
        "放行旧树时没有打印警告，事后无从得知这次部署推的是残缺树")

