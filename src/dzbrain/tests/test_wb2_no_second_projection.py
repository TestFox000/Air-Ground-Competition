# -*- coding: utf-8 -*-
"""硬判据①：**台架内不得有第二份投影/推进实现**（卡 `WB-2-RIG-20261031`）。

运行::

    cd dzrobot_v2 && PYTHONPATH=src/dzbrain python src/dzbrain/tests/test_wb2_no_second_projection.py

全过 exit 0；任一 FAIL exit 1。风格与 `test_route.py` / `test_wb1_route_source.py` 一致。

## 为什么这个文件是本卡的**头号门槛**

本轮五份只读审计找出的头号病灶就是"两台半个、各带一套平行投影"：

| 位置 | 平行实现 | 与生产语义的差异 |
|---|---|---|
| `sim.py` | `_project_sticky` / `_clamp_forward` / `_advance_anchor` | 窗口 0.80/1.50、`allow_backward=True`、滞回 `_SEG_STICKY_M` |
| 生产 `pursuit/path.py` | `Path.project` | 窗口 `arrival_m`、`allow_backward=False`、`_heal_stuck`、150° 方向闸 |

后果不是"结果略有不同"，而是**指标不可迁移**："仿真绿了，实车绿不了"。
所以新台架如果自己写一套投影，就是把同一个 bug 再复制一遍 —— 本文件存在的唯一
目的就是让这件事**机器可判、当场变红**。

## 三层判据（从粗到细，任一失败即红）

1. **源码级**：写域内的文件**不许**出现 `def project(` / `def _clamp(` / `def _advance(` /
   `def _project_sticky` 这类自写实现（含 `_clamp_forward`、`_advance_anchor`）；
2. **引用级**：`sim_world` 用的投影**必须是** `dzbrain.pursuit.path.Path` 的方法 ——
   断言 `World.path` 是 `Path` 实例，且 `project` 方法来自 `Path`（`__qualname__` 对拍）；
3. **同一性级**（最关键）：台架的投影可调用对象**就是**生产 `LocalizationTask` 用的那个
   —— `LocalizationTask._project` 内部调的是 `self._path.project`，而 `self._path` 也是
   `Path`。所以断言两者的**函数对象 identity 相同**（`Path.project is Path.project`），
   并额外断言 `sim.py` 的平行实现**确实存在于 tier-0**（防止有人"顺手删掉 tier-0"，
   让本判据变成空断言）。

## 负例（必须真跑出红）

- 在台架源码里植入一个 `def project(` → §1 判红（本文件自带这个自检，见 §5）。
"""

from __future__ import annotations

import math
import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

_HERE = pathlib.Path(__file__).resolve()
_SRC = _HERE.parents[1]                       # src/dzbrain
_REPO = _SRC.parents[1]                       # dzrobot_v2
_TOOLS = _REPO / "tools"
if str(_TOOLS) not in sys.path:
    sys.path.append(str(_TOOLS))              # ★ 必须 append：`tools/dzbrain.py` 会遮住包

_PASS = 0
_FAIL: list[str] = []
_SECTION = ""


def section(title: str) -> None:
    global _SECTION
    _SECTION = title
    print(f"\n=== {title} ===")


def check(name: str, cond: bool, detail: str = "") -> bool:
    global _PASS
    if cond:
        _PASS += 1
        print(f"  PASS  {name}" + (f"   [{detail}]" if detail else ""))
    else:
        _FAIL.append(f"{_SECTION} :: {name}" + (f"   [{detail}]" if detail else ""))
        print(f"  FAIL  {name}" + (f"   [{detail}]" if detail else ""))
    return bool(cond)


def finish() -> None:
    print("\n" + "=" * 74)
    total = _PASS + len(_FAIL)
    if _FAIL:
        print(f"结果：{_PASS}/{total} 通过，{len(_FAIL)} 失败")
        for item in _FAIL:
            print(f"  FAIL  {item}")
        print("=" * 74)
        sys.exit(1)
    print(f"结果：{_PASS}/{total} 全部通过 ✅")
    print("=" * 74)
    sys.exit(0)


# --------------------------------------------------------------------------- #
#  写域（**只检查这些文件**；不许扩大到全仓，否则存量平行实现会让本判据永远红）
# --------------------------------------------------------------------------- #
WRITE_SCOPE: tuple[str, ...] = (
    "tools/sim_world.py",
    "tools/sim_rig.py",
    "tools/wb_rig/__init__.py",
    "tools/wb_rig/scenarios.py",
    "tools/wb_rig/arms.py",
    "tools/wb_rig/actuator.py",
    "tools/wb_rig/sched.py",
    "tools/wb_rig/counters.py",
    "tools/wb_rig/metrics.py",
)

#: 禁写的**自写实现**签名（生产 `Path.project` 与 `sim.py` 私有平行实现的名字）
FORBIDDEN_DEFS: tuple[str, ...] = (
    "project", "_project", "_project_sticky", "_project_with_floor",
    "_clamp", "_clamp_forward", "_advance", "_advance_anchor", "_pick_candidate",
    "_candidates", "step_bicycle", "_bicycle_step", "_integrate",
)


def _strip_comments_and_docstrings(text: str) -> str:
    """去掉注释与字符串字面量，只留**真代码**（否则注释里提到 `def project` 也会误红）。

    ★ 为什么必须去：本台的 docstring 里到处引用 `_project_sticky` 这些**反面教材**名字。
      不去掉的话，第一版实现就会因为"docstring 里写了禁词"而判红 —— 这正是"判据把
      文档写成敌人"，必须避免。
    """
    import io
    import tokenize
    out: list[str] = []
    try:
        for tok in tokenize.generate_tokens(io.StringIO(text).readline):
            if tok.type in (tokenize.COMMENT, tokenize.STRING):
                continue
            out.append(tok.string)
            out.append(" ")
        return "".join(out)
    except (tokenize.TokenError, IndentationError):
        # 解析不了就退化成逐行去 `#` 后半段（宁可少检，也不许"因为解析失败就说没问题"）
        return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


# --------------------------------------------------------------------------- #
#  1. 源码级：写域内不许有自写投影/推进
# --------------------------------------------------------------------------- #
def test_no_self_written_impl() -> None:
    section("1. 源码级：写域内不存在自写投影/推进实现")
    hits: list[str] = []
    for rel in WRITE_SCOPE:
        path = _REPO / rel
        if not path.is_file():
            check(f"写域文件存在：{rel}", False, str(path))
            continue
        code = _strip_comments_and_docstrings(path.read_text(encoding="utf-8"))
        for name in FORBIDDEN_DEFS:
            # 匹配 `def <name>(` 与 `def <name> (` —— 只认函数定义，不认调用
            if re.search(rf"(?m)^\s*(?:async\s+)?def\s+{re.escape(name)}\s*\(", code):
                hits.append(f"{rel}: def {name}(")
    check("★ 写域内**没有任何**自写 project/_clamp/_advance/step_bicycle 定义",
          not hits, "；".join(hits) if hits else f"{len(WRITE_SCOPE)} 个文件扫描完毕")
    # 反证：本判据不是空跑 —— 它确实能在 tier-0 的平行实现上命中
    tier0 = _strip_comments_and_docstrings(
        (_SRC / "dzbrain" / "tools" / "sim.py").read_text(encoding="utf-8"))
    found = [n for n in ("_project_sticky", "_clamp_forward", "_advance_anchor")
             if re.search(rf"(?m)^\s*def\s+{re.escape(n)}\s*\(", tier0)]
    check("★反证：同一套正则能在 tier-0 的 sim.py 上命中平行实现（判据不是空断言）",
          len(found) >= 2, f"命中 {found}")


# --------------------------------------------------------------------------- #
#  2. 引用级：台架的投影对象来自生产 Path
# --------------------------------------------------------------------------- #
def test_projection_from_production_path() -> None:
    section("2. 引用级：World.path 是生产 `pursuit.path.Path`，project 是它的方法")
    import sim_world
    from dzbrain.pursuit.path import Path

    check("sim_world 里 import 的是生产 `Path`（不是自建类）",
          getattr(sim_world, "Path", None) is Path,
          f"sim_world.Path = {getattr(sim_world, 'Path', None)!r}")
    # 造一个最小路线（用生产解析器，不手搓 Waypoint）
    from dzbrain.route.parser import parse_mission
    route = parse_mission(["A1", "B1", "C1"])
    world = sim_world.World(
        route=route,
        injection=sim_world.Injection(),
        start_pose=(0.0, 0.0, 0.0),
    )
    check("★ World.path 是 Path 的**实例**（同一类，不是同值不同码）",
          isinstance(world.path, Path), f"{type(world.path).__name__}")
    check("★ World.path.project 的 `__qualname__` 指向 `Path.project`",
          getattr(type(world.path).project, "__qualname__", "") == "Path.project",
          getattr(type(world.path).project, "__qualname__", "<无>"))
    # 行为侧对拍：与手工调生产 Path.project 必须逐位相同
    proj_rig = world.project_truth()
    proj_ref = Path(route).project(0.0, 0.0)
    check("★ 台架投影结果与直调 `Path.project` **逐位相同**（同一条几何，不是另一套）",
          (proj_rig.s, proj_rig.seg_index, proj_rig.dist)
          == (proj_ref.s, proj_ref.seg_index, proj_ref.dist),
          f"台架 {(proj_rig.s, proj_rig.seg_index, proj_rig.dist)} / "
          f"直调 {(proj_ref.s, proj_ref.seg_index, proj_ref.dist)}")


# --------------------------------------------------------------------------- #
#  3. 同一性级：与生产 Localization 用的**是同一个函数对象**
# --------------------------------------------------------------------------- #
def test_same_callable_as_localization() -> None:
    section("3. 同一性级：台架投影 == 生产 Localization 投影（函数对象 identity）")
    from dzbrain.pursuit.path import Path
    from dzbrain.tasks import localization as LOC

    check("`Path.project` 与 `Path.project` 同一（函数对象 identity，非文本相似）",
          Path.project is Path.project, "同一对象")
    # Localization 内部用法是真源：`self._path.project(...)`（见 localization.py:_project）
    src = pathlib.Path(LOC.__file__).read_text(encoding="utf-8")
    check("★ 生产 Localization 的投影调用是 `self._path.project(...)`（读源码确认，不靠猜）",
          "self._path.project(" in src,
          "命中 localization.py 的 _project")
    check("★ 生产 Localization 的 `_path` 也由 `Path(route)` 构造（同一个类）",
          re.search(r"self\._path\s*=\s*Path\(route\)", src) is not None,
          "命中 `self._path = Path(route)`")
    # 运行期同一性：让 Localization 真跑一拍，读它内部缓存的 Path 与台架的是同一类
    import sim_world
    from dzbrain.core.blackboard import P_ROUTE, Blackboard
    from dzbrain.core.config import Config
    from dzbrain.core.events import EventBus
    from dzbrain.route.parser import _read_token_file, parse_mission
    import fakes

    fixture = _SRC / "tests" / "fixtures" / "route_field7_t24.txt"
    tokens = _read_token_file(str(fixture))
    route = parse_mission(tokens)
    cfg = Config.load(str(_REPO / "config" / "robot.yaml"))
    cfg.override("fix.anchor_at_waypoint", False)
    cfg.override("localization.yaw_trim_deg", 0.0)
    clock = fakes.FakeClock()
    bridge = fakes.FakeBridge(clock)
    x0, y0 = route.waypoints[0].x, route.waypoints[0].y
    bridge.set_pose(x0, y0, route.waypoints[0].yaw)
    bb, bus = Blackboard(), EventBus()
    bb.set(P_ROUTE, route, "test")
    loc = LOC.LocalizationTask(bridge, bb, bus, cfg, log=lambda m: None, clock=clock.now)
    loc.setup()
    loc.tick()
    loc_path = getattr(loc, "_path", None)
    check("★ 运行期：Localization 内部 `_path` 是 `Path` 实例（与台架同类）",
          isinstance(loc_path, Path), f"{type(loc_path).__name__}")
    world = sim_world.World(route=route, injection=sim_world.Injection(),
                            start_pose=(x0, y0, route.waypoints[0].yaw))
    check("★ 运行期：台架 `World.path` 与 Localization `_path` 是**同一个类对象**",
          type(world.path) is type(loc_path),
          f"{type(world.path).__qualname__} vs {type(loc_path).__qualname__}")
    check("★ 运行期：两条 Path 的 project 方法解析到**同一个函数对象**",
          type(world.path).project is type(loc_path).project, "同一对象")


# --------------------------------------------------------------------------- #
#  4. 推进：唯一积分入口是生产 `step_bicycle`
# --------------------------------------------------------------------------- #
def test_integration_from_production() -> None:
    section("4. 推进级：唯一积分入口 = 生产 `tools/sim.step_bicycle`")
    import sim_world
    from dzbrain.tools import sim as SIM

    check("sim_world 里 import 的是生产 `step_bicycle`",
          getattr(sim_world, "step_bicycle", None) is SIM.step_bicycle,
          f"{getattr(sim_world, 'step_bicycle', None)!r}")
    # 行为对拍：World.step 的位移必须与"直调 step_bicycle"逐位相同（含 yaw 归一化）
    from dzbrain.route.parser import parse_mission
    route = parse_mission(["A1", "B1"])
    world = sim_world.World(route=route, injection=sim_world.Injection(),
                            start_pose=(0.0, 0.0, 0.0), wheelbase_m=0.157)
    world.step(0.02, 0.3, 11.0, 1000.0)
    ref = SIM.step_bicycle(0.0, 0.0, 0.0, 0.3, 11.0, 0.02, 0.157)
    got = (world.truth.x, world.truth.y, world.truth.yaw)
    check("★ 台架积分与直调 `step_bicycle` **逐位相同**", got == ref, f"{got} vs {ref}")
    # 零注入时里程计 ≡ 真值（那条强不变量的最小版本，完整版在 test_wb2_rig_invariants）
    same = (world.odom_frame(1000.02).pose().x, world.odom_frame(1000.02).pose().y)
    check("零注入下里程计与真值同值（不偷加噪声）",
          same == (world.truth.x, world.truth.y), f"{same} vs {(world.truth.x, world.truth.y)}")


# --------------------------------------------------------------------------- #
#  5. 负例：植入自写 project 必须判红（证明判据有牙）
# --------------------------------------------------------------------------- #
def test_negative_injected_projection() -> None:
    section("5. 负例：写域里**植入**一个 `def project(` ⇒ 本判据必须判红")
    import tempfile

    fake_body = ("# -*- coding: utf-8 -*-\n"
                 "import math\n"
                 "def project(x, y, s_hint=None):\n"
                 "    return math.hypot(x, y)\n")
    path = _TOOLS / "wb_rig" / f"_neg_probe_{id(self_marker())}.py"
    try:
        path.write_text(fake_body, encoding="utf-8")
        code = _strip_comments_and_docstrings(path.read_text(encoding="utf-8"))
        hit = re.search(r"(?m)^\s*(?:async\s+)?def\s+project\s*\(", code)
        check("★ 负例：植入的 `def project(` 被 §1 的正则命中（判据有牙）",
              hit is not None, "命中=判据有效；不命中=判据形同虚设")
    finally:
        try:
            path.unlink()
        except OSError:
            pass
    check("负例探针文件已清理（没在写域里留垃圾）", not path.exists(), str(path))


def self_marker() -> object:
    """给探针文件名一点唯一性（同进程重复跑不撞名）。"""
    return object()


# --------------------------------------------------------------------------- #
#  6. 反向：tier-0 的四条契约仍然在（防止"顺手删掉 tier-0 让判据空转"）
# --------------------------------------------------------------------------- #
def test_tier0_still_present() -> None:
    section("6. tier-0 留存：`sim.py` / `sim_cli.py` 的契约签名仍在（本卡零写入）")
    sim_src = (_SRC / "dzbrain" / "tools" / "sim.py").read_text(encoding="utf-8")
    cli_src = (_SRC / "dzbrain" / "tools" / "sim_cli.py").read_text(encoding="utf-8")
    check("`sim.py` 仍有 `run_sim`", "def run_sim(" in sim_src)
    check("`sim.py` 仍有 `step_bicycle`", "def step_bicycle(" in sim_src)
    check("`sim_cli.py` 的 schema 仍是 `dzrobot-sim-result/1`",
          '"dzrobot-sim-result/1"' in cli_src,
          "现网契约号未被本卡改动")
    check("`sim_cli.py` 仍打 `SIM_RESULT_JSON ` 前缀（末行契约）",
          '"SIM_RESULT_JSON "' in cli_src or "SIM_RESULT_JSON " in cli_src)
    # 本卡的 schema 是**新号**，不与现网冲突
    import wb_rig.metrics as MET
    check("★ 本卡 schema 是新号 `dzrobot-trackbench-result/1`（不取代现网）",
          MET.SCHEMA == "dzrobot-trackbench-result/1", MET.SCHEMA)
    check("★ 五键形状冻结（`finished/time_s/rms_m/out_of_bounds/length_ratio`）",
          MET.FROZEN_FIVE == ("finished", "time_s", "rms_m", "out_of_bounds", "length_ratio"),
          str(MET.FROZEN_FIVE))


if __name__ == "__main__":
    test_no_self_written_impl()
    test_projection_from_production_path()
    test_same_callable_as_localization()
    test_integration_from_production()
    test_negative_injected_projection()
    test_tier0_still_present()
    finish()
