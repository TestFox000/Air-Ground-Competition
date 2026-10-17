# -*- coding: utf-8 -*-
r"""标定参数回填工具链（任务卡 t2 / n2）—— 实测值 → `config/robot.yaml` 键 → 测试断言重钉清单。

## 这个工具解决什么问题

实车标定（R3 直线 / R4 原地转圈）拿到卷尺实测数之后，需要同时改动**三个地方**：

  1. `config/robot.yaml` 的标定标量（`dzactuator.ticksPerMeter` / `dzactuator.ticksPer2PI` /
     `localization.yaw_trim_deg` / `pursuit.wheelbase_m`）；
  2. **钉住这些值的测试断言**（改配置不改断言 = 全量回归立刻变红，或者更糟：断言还钉着旧值却"绿"）；
  3. 说明书/报告里的历史数字（只提示，不自动改）。

三者不同步是这类项目最常见的翻车方式（"配置改了、断言没改、回归红一片"），
所以本工具把 ①② 绑成**同一次操作**：`--apply-config` 改配置，`--apply-tests` 重钉断言，
两边都用**行锚点 + 原文子串**双重校验，行号漂了就拒绝写入（`--selfcheck` 能提前发现）。

## 模式（默认全部是**干跑**，不动任何文件）

| 命令 | 作用 | 退出码 |
|---|---|---|
| `python tools\calib_backfill.py --selfcheck` | 自检：键可读 / 行锚点唯一 / 每条钉的 `find` 子串仍在 / 覆盖缺口 / 交叉一致性 | 0 干净；3 有 error |
| `python tools\calib_backfill.py --list` | 打印"参数 → yaml 键 → 钉位"映射总表 | 0 |
| `python tools\calib_backfill.py --template` | 打印待队长填写的实测值模板（= `calib_backfill_template.yaml` 同源） | 0 |
| `python tools\calib_backfill.py --values <文件>` | **干跑**：算判定档 / 给出配置改动预览 + 重钉清单 | 0 可回填或 PENDING；2 有 REFUSE |
| `python tools\calib_backfill.py --values <文件> --apply-config` | 真写 `config/robot.yaml`（先备份 `*.bak_<ts>`） | 同上 |
| `python tools\calib_backfill.py --values <文件> --apply-tests` | 真写测试断言（**必须先看清重钉清单**） | 同上 |
| `--json` | 机器可读输出（供验证卡 n3 复核） | — |
| `--emit-report <路径>` | 把干跑结论落成 Markdown 报告 | — |
| （任何模式） | `--values` 指向的文件不存在 | 3 |

## 铁律

* **默认不写盘**：不加 `--apply-*` 就只打印。
* `--apply-*` 只改**本表登记的标量行/断言行**，且改前校验、改后复校（自动再跑一次 selfcheck）。
* 本工具**不**碰 `route/`、不 commit、不部署、不连车。
* 判定不足（PENDING）或触发护栏（REFUSE）时**只给一个动作**，与 `_tmp_session/car_calib/R4_spin_calib_plan.md`
  的"每档只做一件事"口径一致。

跑法（PowerShell）：
    $env:PYTHONPATH='D:\User\Desktop\dzrobot_v2\src\dzbrain'; $env:PYTHONIOENCODING='utf-8'
    python tools\calib_backfill.py --selfcheck
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import pathlib
import re
import shutil
import statistics
import sys

_HERE = pathlib.Path(__file__).resolve()
_WS = _HERE.parents[1]
sys.path.insert(0, str(_WS / "src" / "dzbrain"))

try:  # 车端/本地控制台不一定是 UTF-8，替换而不是崩
    sys.stdout.reconfigure(errors="replace")  # type: ignore[union-attr]
except Exception:  # pragma: no cover
    pass

import yaml  # noqa: E402

from dzbrain.core.config import Config  # noqa: E402

# --------------------------------------------------------------------------- #
#  常量与来源锚点
# --------------------------------------------------------------------------- #
CONFIG_DEFAULT = _WS / "config" / "robot.yaml"
TEMPLATE_DEFAULT = _HERE.parent / "calib_backfill_template.yaml"
CARW_H = _WS / "src" / "dzactuator" / "include" / "dzactuator.h"

#: 轮距来源：`#define CARW 0.157 // 轮距`（车端 C++ 头文件；2026-10-11 用户卷尺实测 0.157，头文件已同步）
_CARW_RE = re.compile(r"#define\s+CARW\s+([0-9.]+)")

#: 默认容差（与 R3/R4 报告的判据一致）
TOL_LONG_CM = 4.0        # 纵向残差容差（robot.yaml:563「停位距 A10 中心 ≤4 cm」）
TOL_YAW_TRIM_DEG = 0.3   # yaw_trim 变动小于此值不折腾
TOL_WHEELBASE_M = 0.002  # 轴距小于 2 mm 不折腾


def _pin(file: str, line: int, find: str, replace: str, note: str = "") -> dict:
    """一条断言钉：`file:line` 行内必须含 `find`，回填时替换成 `replace`（`{v}` = 新值）。
    `line` 只作粗提示：实际定位走内容式（见 _resolve_pin_line），容忍目标文件中部插行。"""
    return {"file": file, "line": line, "find": find, "replace": replace, "note": note}


def _resolve_pin_line(lines: list[str], p: dict, window: int = 40) -> tuple[int | None, str]:
    """内容式定位一条钉：全文找「行内恰好出现一次 find」的候选行。
    唯一命中 ⇒ 直接采用（中部插行导致行号漂移时自动跟随）；
    多命中 ⇒ 取登记行号 ±window 内最近的；
    全不中 ⇒ (None, 原因)。"""
    find = p["find"]
    hits = [i for i, s in enumerate(lines, 1) if s.count(find) == 1]
    if not hits:
        return None, "全文已不含约定原文（行号漂了）"
    if len(hits) == 1:
        return hits[0], ""
    anchor = int(p.get("line", 1))
    near = [i for i in hits if abs(i - anchor) <= window]
    if not near:
        return None, f"原文命中 {len(hits)} 处且都远离登记行号 {anchor}，无法唯一定位"
    return min(near, key=lambda i: abs(i - anchor)), ""


# --------------------------------------------------------------------------- #
#  ★ 映射总表（本工具的唯一真源）
#     yaml_key  : Config 点号键（读）
#     yaml_leaf : YAML 里的叶子键名（写：用行锚点唯一命中）
#     pins      : 钉住该值的测试断言（写：手写清单，行锚点）
#     manual    : True = 只登记钉位、本卡不回填（需另开卡）
# --------------------------------------------------------------------------- #
CHANNELS: tuple[dict, ...] = (
    {
        "id": "ticksPerMeter",
        "title": "直线里程刻度",
        "card": "R3",
        "yaml_key": "dzactuator.ticksPerMeter",
        "yaml_leaf": "ticksPerMeter",
        "unit": "tick/m",
        "decimals": 1,
        "lo": 1000.0, "hi": 20000.0,
        "guard_pct": 25.0,
        "inputs": ("r3.belief_travel_m", "r3.long_residual_cm", "r3.residual_measured_by"),
        "note": ("公式（robot.yaml:557-563 的实测口径）：new = old × 信念行程 / 实走行程，"
                 "实走行程 = 信念行程 − 欠程。容差 |欠程| ≤ 4 cm ⇒ 冻结不改。"
                 "⚠车端 C++ 另有内置默认值 dzactuator.cpp:1127 `declare_parameter(\"ticksPerMeter\", 1000.0)` —— "
                 "只在参数没传进去时才会用到（本卡不碰 C++）。"),
        "comment_note": "robot.yaml:553-563 的标定说明段落需同步改写（本工具只改标量行）",
        "pins": (
            _pin("src/dzbrain/tests/test_core.py", 164, "→ 3280.0）", "→ {v}）",
                 "断言失败信息里的历史数字"),
            _pin("src/dzbrain/tests/test_core.py", 165, "== 3280.0", "== {v}",
                 "★ 断言本体：dzactuator.ticksPerMeter"),
            _pin("src/dzbrain/tests/test_motion_limit.py", 220, "ticksPerMeter 3280）", "ticksPerMeter {v_short}）",
                 "断言失败信息里的历史数字"),
            _pin("src/dzbrain/tests/test_motion_limit.py", 224, "- 3280.0", "- {v}",
                 "★ 断言本体：abs(...ticksPerMeter) - <值>"),
        ),
    },
    {
        "id": "ticksPer2PI",
        "title": "原地一圈计数（航向）",
        "card": "R4",
        "yaml_key": "dzactuator.ticksPer2PI",
        "yaml_leaf": "ticksPer2PI",
        "unit": "tick/360°",
        "decimals": 1,
        "lo": 500.0, "hi": 12000.0,
        "guard_pct": 60.0,
        "inputs": ("r4.runs[].delta_counts", "r4.runs[].yaw_gyro_deg", "r4.turns",
                   "geometry.wheel_track_m"),
        "note": ("r_A = (ΔN/turns)/现值；判档见 R4 方案 §2（≤2% 收敛 / 2~5% 重测 / >5% 机械核查）。"
                 "几何下界：ticksPer2PI_geo = 2π · 轮距 · ticksPerMeter。"
                 "⚠车端 C++ 另有内置默认值 dzactuator.cpp:1128 `declare_parameter(\"ticksPer2PI\", 3200.0)` —— "
                 "只在参数没传进去时才会用到（本卡不碰 C++）。"),
        "comment_note": "robot.yaml:565-572 的 3236/2309.4 说明段需同步改写",
        "pins": (),   # ★ dzbrain 测试里 0 处钉它（只有 route/validate.py:10 注释）—— 见 --selfcheck 的"覆盖缺口"
    },
    {
        "id": "yaw_trim_deg",
        "title": "横向漂移补偿角",
        "card": "R3",
        "yaml_key": "localization.yaw_trim_deg",
        "yaml_leaf": "yaw_trim_deg",
        "unit": "deg",
        "decimals": 1,
        "lo": -10.0, "hi": 10.0,
        "guard_pct": None,          # 用绝对护栏：|Δ| > 3.0° 拒绝
        "guard_abs": 3.0,
        "inputs": ("r3.lat_drift_cm", "r3.belief_travel_m", "r3.lat_drift_confirmed"),
        "note": ("符号约定（tasks/localization.py:172-178）：实测横向偏移 ÷ 行程 → 角度；"
                 "**车偏右 ⇒ 取负值**。suggested = −degrees(atan(漂移m / 行程m))。"),
        "comment_note": ("robot.yaml:515-517 的「直线4.55m 右偏 10/11 cm ≈ 1.4°」与 "
                         "tasks/localization.py:172-177 的同段注释需按实测改写"),
        "pins": (),   # ★ 无任何断言钉它（且不在 SCHEMA 里 ⇒ 无范围校验）—— 见 --selfcheck
    },
    {
        "id": "wheelbase_m",
        "title": "轴距（自行车模型）",
        "card": "现场卷尺",
        "yaml_key": "pursuit.wheelbase_m",
        "yaml_leaf": "wheelbase_m",
        "unit": "m",
        "decimals": 3,
        "lo": 0.05, "hi": 0.60,
        "guard_pct": 10.0,
        "inputs": ("geometry.wheelbase_m",),
        "note": "注意：这是**轴距**（前后轮），不是轮距（左右轮）。轮距只进几何校核，不回填。"
                "⚠ 代码默认值也在钉（core/config.py:65 的 `Spec(0.157, …)`）——改轴距要连它一起改，"
                "否则任何「没传参数」的构造会回到旧轴距。",
        "comment_note": "robot.yaml:422 的注释（旧代码硬编码 path_executor.py:629）+ core/config.py:66 的说明",
        "pins": (
            _pin("src/dzbrain/tests/test_core.py", 168, "== 0.157", "== {v}",
                 "★ 断言本体：pursuit.wheelbase_m（SCHEMA 默认读数）"),
            _pin("src/dzbrain/dzbrain/core/config.py", 65, "Spec(0.157,", "Spec({v},",
                 "★ 代码默认值本体（Spec 默认值）"),
        ),
        "formula_fixtures": (
            _pin("src/dzbrain/tests/test_pursuit.py", 395, "wheelbase_m=0.157", "wheelbase_m={v}"),
            _pin("src/dzbrain/tests/test_pursuit.py", 404, "2.0 * 0.157", "2.0 * {v}"),
            _pin("src/dzbrain/tests/test_pursuit.py", 412, "wheelbase_m=0.157", "wheelbase_m={v}"),
            _pin("src/dzbrain/tests/test_pursuit.py", 415, "0.157 * math.sin", "{v} * math.sin"),
            _pin("src/dzbrain/tests/test_pursuit.py", 455, "wheelbase_m=0.157", "wheelbase_m={v}"),
            _pin("src/dzbrain/tests/test_drift.py", 82, '"wheelbase_m": 0.157', '"wheelbase_m": {v}'),
            _pin("src/dzbrain/tests/test_drift.py", 359, "2.0 * 0.157", "2.0 * {v}"),
            _pin("src/dzbrain/tests/test_drift.py", 370, "dt, 0.157)", "dt, {v})"),
            _pin("src/dzbrain/tests/test_drift.py", 372, "dt, 0.157)", "dt, {v})"),
            _pin("src/dzbrain/tests/test_sim.py", 81, "0.157 / math.tan", "{v} / math.tan"),
            _pin("src/dzbrain/tests/test_sim.py", 187, "0.02, 0.157)", "0.02, {v})"),
            _pin("src/dzbrain/tests/test_sim.py", 190, "0.02, 0.157)", "0.02, {v})"),
            _pin("src/dzbrain/tests/test_sim.py", 193, "0.02, 0.157)", "0.02, {v})"),
            _pin("src/dzbrain/tests/test_sim.py", 195, "0.15 / 0.157", "0.15 / {v}"),
            _pin("src/dzbrain/tests/test_sim.py", 196, "0.02, 0.157)", "0.02, {v})"),
            _pin("src/dzbrain/tests/test_sim.py", 197, "0.02, 0.157)", "0.02, {v})"),
        ),
    },
    # ---- 几何输入（不是 yaml 键；只进判据，指向车端 C++，本卡不动）----
    {
        "id": "wheel_track_m",
        "title": "轮距（左右轮中心距，几何校核用）",
        "card": "R4",
        "yaml_key": None,
        "yaml_leaf": None,
        "unit": "m",
        "decimals": 3,
        "lo": 0.05, "hi": 0.60,
        "guard_pct": None,
        "inputs": ("geometry.wheel_track_m",),
        "note": ("只用于 ticksPer2PI 的几何下界校核，**没有 yaml 键可回填**；"
                 "若实测 ≠ dzactuator.h 的 CARW，需要**另开卡改车端 C++**（本卡不碰）。"),
        "pins": (),
    },
    # ---- 本卡不回填、只登记钉位（另开卡）----
    {
        "id": "cruise_speed_mps",
        "title": "巡航速度（G1 裁决项）",
        "card": "G1（另开卡）",
        "manual": True,
        "yaml_key": "pursuit.cruise_speed_mps",
        "yaml_leaf": "cruise_speed_mps",
        "unit": "m/s",
        "decimals": 2,
        "lo": 0.02, "hi": 0.60,
        "guard_pct": None,
        "inputs": (),
        "note": ("必须与旧键 `path_executor.linear_speed`（robot.yaml:251）**同步**改，"
                 "否则车里出现两个速度（robot.yaml:430 的注释就是这么约束的）。"),
        "pins": (
            _pin("src/dzbrain/tests/test_core.py", 154, "（0.30，与 cruise 同步）", "（{v}，与 cruise 同步）"),
            _pin("src/dzbrain/tests/test_core.py", 155, "== 0.30", "== {v}"),
            _pin("src/dzbrain/tests/test_core.py", 156, "由旧键迁移（0.30）", "由旧键迁移（{v}）"),
            _pin("src/dzbrain/tests/test_core.py", 157, "== 0.30", "== {v}"),
            _pin("src/dzbrain/tests/test_motion_limit.py", 221, "- 0.30", "- {v}"),
            _pin("src/dzbrain/tests/test_route.py", 568, "linear_speed（0.30）", "linear_speed（{v}）"),
            _pin("src/dzbrain/tests/test_route.py", 569, "- 0.30", "- {v}"),
        ),
    },
    {
        "id": "px_per_m_rail",
        "title": "轨道 px/m（打靶标定）",
        "card": "打靶卡（另开卡）",
        "manual": True,
        "yaml_key": "aim.px_per_m_rail",
        "yaml_leaf": "px_per_m_rail",
        "unit": "px/m",
        "decimals": 1,
        "lo": 20.0, "hi": 600.0,
        "guard_pct": None,
        "inputs": (),
        "note": ("标定法见 robot.yaml:478-483（靶放轨道两端量 |Δdx|）。"
                 "test_aim.py:264 只断言「键存在」，**不钉值** ⇒ 改 140 不会红，属覆盖缺口。"),
        "pins": (),   # 无值钉（test_aim.py:264 只查键存在）
    },
)

CHANNEL_BY_ID = {c["id"]: c for c in CHANNELS}

#: 判断"合法 bool"用
_TRUE = ("true", "yes", "1", "y", "是", "确认")

#: 扫描"未被登记的历史引用"时跳过的目录名（任一路径段命中即跳过）
_SCAN_SKIP = {".git", "build", "install", "log", "backup", "artifacts", "__pycache__",
              "_tmp_session", "_scratch_aim", "_scratch_auto_plan", "_tmp_install_repro", ".agent-teams"}


def _skip_dir(name: str) -> bool:
    """该目录名是否属于"草稿/临时"（不参与旧值引用扫描）。

    ★ 为什么必须按**路径上任意一段**判、而不是只判文件名前缀（t16 根因）：
      `_SCAN_SKIP` 列了 `_tmp_session`，但**后缀不同的兄弟目录**（如 `_tmp_verify/`）会漏掉；
      而"只判文件名 `startswith('_tmp_')`"更漏 —— 变异副本里**被扫的是 `tests/test_core.py`**，
      文件名根本不带 `_tmp_`。于是 `_tmp_verify/t9/mut/*/tests/test_core.py`（t9 变异测试的
      临时副本，与 src 下的真 test_core 同名）被当成"已登记钉的重复引用"⇒ test 10.2 假红。
      ⇒ 统一口径：路径上任意一段以 `_tmp_` 开头，或命中 `_SCAN_SKIP`，即跳过。
    """
    return name.startswith("_tmp_") or name in _SCAN_SKIP


#: ★ 历史部署快照/草稿的**注册表**（卡 t16）——扫描时按"整条相对路径"豁免。
#  为什么用独立注册表而不是塞进 `CHANNELS[*].pins`：pins 的语义是"**会被回填的钉位**"
#  （`manual=False` 的通道会真的 `plan_repins` 改文件；test 3.1/3.3 还要求每条钉都能
#  内容式定位、且重钉后文本必变）。把"历史快照"塞进 pins 会让它们出现在"待回填清单"里，
#  既语义错（快照不该被改）又破坏回填自检。这里是**只登记不参与回填**的豁免表。
_HISTORICAL_SNAPSHOTS: tuple[dict, ...] = (
    {"file": "_backups/deploy_map3_20261011_142413/robot.yaml",
     "why": "历史部署快照（2026-10-11 14:24 换图前备份）"},
    {"file": "_backups/g1_030_Azu_20261011_135423/robot.yaml",
     "why": "历史部署快照（2026-10-11 13:54 g1_030 备份）"},
    {"file": "_deploy_cfg.yaml",
     "why": "历史部署草稿；活跃真值是 config/robot.yaml"},
)


def _scan_snapshot_paths(root: pathlib.Path) -> set[pathlib.Path]:
    """注册表 → 绝对路径集合（扫描时整条豁免）。"""
    return {(root / s["file"]).resolve() for s in _HISTORICAL_SNAPSHOTS}


_SCAN_EXT = {".py", ".yaml", ".yml", ".md", ".sh", ".json", ".txt"}


# --------------------------------------------------------------------------- #
#  小工具
# --------------------------------------------------------------------------- #
def _fmt(value: float, decimals: int) -> str:
    s = f"{float(value):.{decimals}f}"
    return s


def _short(value: float, decimals: int) -> str:
    """失败信息里的"短写法"：整数值去掉小数（3280.0 → 3280）。"""
    v = float(value)
    if abs(v - round(v)) < 1e-9:
        return str(int(round(v)))
    return _fmt(v, decimals)


def _as_bool(v) -> bool:
    if isinstance(v, bool):
        return v
    if v is None:
        return False
    return str(v).strip().lower() in _TRUE


def get_ws() -> pathlib.Path:
    return _WS


def carw_m() -> float | None:
    """读车端头文件里的轮距（只读，取现场真源）。"""
    try:
        text = CARW_H.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    m = _CARW_RE.search(text)
    return float(m.group(1)) if m else None


def locate_yaml_line(text: str, leaf: str) -> tuple[int, str, str, str]:
    """按叶子键名在 YAML 文本里唯一命中一行。

    返回 (1-based 行号, 缩进, 旧标量, 行尾注释+换行)。命中 0 或 >1 行 → ValueError。
    """
    pat = re.compile(r"^([ \t]*)" + re.escape(leaf) + r":([ \t]*)(\S+)(.*)$")
    hits = []
    for i, line in enumerate(text.splitlines(), start=1):
        m = pat.match(line)
        if m:
            hits.append((i, m))
    if not hits:
        raise ValueError(f"YAML 里找不到叶子键 {leaf!r}")
    if len(hits) > 1:
        raise ValueError(f"叶子键 {leaf!r} 命中 {len(hits)} 行（{ [h[0] for h in hits] }），拒绝写入")
    i, m = hits[0]
    return i, m.group(1), m.group(3), m.group(4)


def repin_line(src: str, find: str, replace: str) -> str:
    """行内子串替换（要求恰好命中一次）。"""
    if src.count(find) != 1:
        raise ValueError(f"子串 {find!r} 在行内命中 {src.count(find)} 次（要求 1 次）")
    return src.replace(find, replace)


def _read_text(path: pathlib.Path) -> str:
    """**保留原样换行**地读文本（`read_text` 会把 CRLF 归一成 LF，写回就会整篇换行符抖动）。

    本仓 `config/robot.yaml` 在工作区是 CRLF（`.gitattributes` 里 `*.yaml eol=lf` 只在提交时归一），
    所以回填必须逐行保留原行尾，否则一次回填会改满 577 行。
    """
    with open(path, "r", encoding="utf-8", newline="") as fh:
        return fh.read()


def _write_text(path: pathlib.Path, text: str) -> None:
    with open(path, "w", encoding="utf-8", newline="") as fh:
        fh.write(text)


def _write_text_lf(path: pathlib.Path, text: str) -> None:
    """新文件统一写 LF（本仓 `.gitattributes` 里 *.py/*.yaml/*.md 都是 eol=lf）。"""
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)


def _split_lines(text: str) -> list[str]:
    """不带行尾切行（CRLF / LF 都吃掉，供正则与比较用）。"""
    return text.splitlines()


def _split_lines_keepends(text: str) -> list[str]:
    """保留行尾切行（供写回用）。"""
    return text.splitlines(keepends=True)


def _parse_values_text(text: str) -> dict:
    data = yaml.safe_load(text) or {}
    if not isinstance(data, dict):
        raise ValueError("实测值文件顶层必须是映射（dict）")
    return data


def load_values(path: pathlib.Path) -> dict:
    return _parse_values_text(path.read_text(encoding="utf-8"))


def _dig(values: dict, dotted: str, default=None):
    cur = values
    for part in dotted.split("."):
        if isinstance(cur, dict) and part in cur:
            cur = cur[part]
        else:
            return default
    return cur


# --------------------------------------------------------------------------- #
#  通道判定（纯函数，便于测试）
# --------------------------------------------------------------------------- #
def calc_ticks_per_meter(values: dict, current: float, cfg: dict) -> dict:
    """R3 直线标定：纵向残差 → ticksPerMeter_new。"""
    belief = _dig(values, "r3.belief_travel_m")
    long_cm = _dig(values, "r3.long_residual_cm")
    by = (_dig(values, "r3.residual_measured_by") or "").strip().lower()
    explicit = _dig(values, "ticksPerMeter")

    if explicit is not None:
        new = float(explicit)
        if abs(new - current) / current > 0.001:
            return dict(verdict="SET", new=new,
                        action=f"直接用给定值 {_fmt(new, 1)}（Δ {(new / current - 1) * 100:+.2f}%）",
                        detail=["显式给定 ticksPerMeter，跳过公式推导"])
        return dict(verdict="KEEP", new=current, action="显式给定值 == 现值 ⇒ 冻结", detail=[])

    if belief is None or long_cm is None:
        return dict(verdict="PENDING", new=None,
                    action="补 r3.belief_travel_m（日志信念行程）与 r3.long_residual_cm（卷尺纵向残差）",
                    detail=["缺实测数：纵向通道未闭环"])
    belief = float(belief)
    long_cm = float(long_cm)
    detail = [f"信念行程 {belief:.3f} m，卷尺纵向残差 {long_cm:+.1f} cm（+ = 欠程/停不到位）"]
    if by and by != "tape":
        return dict(verdict="REFUSE", new=None,
                    action="用**卷尺**量纵向残差后重填 r3.residual_measured_by=tape",
                    detail=detail + [f"residual_measured_by={by!r}：日志信念侧取数会被「到位吸附」抹平"
                                     "（R3_verdict §③），不能作为冻结/改值依据"])
    if abs(long_cm) <= TOL_LONG_CM:
        return dict(verdict="KEEP", new=current,
                    action=f"|欠程| {abs(long_cm):.1f} cm ≤ 4 cm ⇒ 冻结 {_fmt(current, 1)}，不改配置、不动断言",
                    detail=detail)
    actual = belief - long_cm / 100.0
    if actual <= 0.5 * belief:
        return dict(verdict="REFUSE", new=None,
                    action="复核卷尺读数（实走行程不足信念一半，疑似量错或车没走完）",
                    detail=detail + [f"实走行程 = {actual:.3f} m 不合理"])
    new = current * belief / actual
    delta_pct = (new / current - 1.0) * 100.0
    detail.append(f"实走行程 {actual:.3f} m ⇒ 尺度比 {belief / actual:.4f} ⇒ "
                  f"new = {current:.1f} × {belief:.2f}/{actual:.2f} = {new:.1f}")
    if abs(delta_pct) > cfg["guard_pct"]:
        return dict(verdict="REFUSE", new=new,
                    action=f"先做机械核查（改动 {delta_pct:+.1f}% 超护栏 ±{cfg['guard_pct']:.0f}%）",
                    detail=detail)
    return dict(verdict="SET", new=new,
                action=f"回填 {_fmt(current, cfg['decimals'])} → {_fmt(new, cfg['decimals'])}（{delta_pct:+.2f}%）",
                detail=detail)


def calc_ticks_per_2pi(values: dict, current: float, cfg: dict,
                       ticks_per_meter: float | None = None,
                       track_m: float | None = None,
                       allow_mechanical_fix: bool = False) -> dict:
    """R4 原地转圈：ΔN/陀螺/几何 → ticksPer2PI_new（判档见 R4 方案 §2、§3.2）。"""
    turns = float(_dig(values, "r4.turns", 2) or 2)
    full_deg = 360.0 * turns
    runs = _dig(values, "r4.runs") or []
    detail: list[str] = []
    have = []
    for r in runs:
        if not isinstance(r, dict):
            continue
        dn = r.get("delta_counts")
        if dn in (None, "", 0):
            continue
        dn = abs(float(dn))
        meas = dn / turns                      # 每 360° 计数（方案 §1.4：ΔN/2）
        ra = meas / current
        yaw = r.get("yaw_gyro_deg")
        rb = (abs(float(yaw)) / full_deg) if yaw not in (None, "", 0) else None
        have.append({"dir": str(r.get("dir", "?")).upper(), "dn": dn, "meas": meas,
                     "rA": ra, "rB": rb,
                     "yaw": (abs(float(yaw)) if yaw not in (None, "", 0) else None)})
        detail.append(f"{have[-1]['dir']:<3} ΔN={dn:8.0f} ticks/{full_deg:.0f}° ⇒ "
                      f"meas={meas:8.1f} r_A={ra:.4f}" +
                      (f"  yaw_gyro={have[-1]['yaw']:.1f}° r_B={rb:.4f}" if rb else "  （无陀螺数）"))
    explicit = _dig(values, "ticksPer2PI")
    if explicit is not None:
        new = float(explicit)
        kind = "SET" if abs(new - current) / current > 0.002 else "KEEP"
        return dict(verdict=kind, new=(new if kind == "SET" else current),
                    action=f"显式给定 ticksPer2PI={new:.1f}（Δ {(new / current - 1) * 100:+.2f}%）",
                    detail=detail or ["显式给定值"])
    if not have:
        return dict(verdict="PENDING", new=None,
                    action="补 r4.runs[].delta_counts（每趟原始计数增量，CW/CCW 各 2 趟）",
                    detail=["缺实测数：航向通道未闭环（s 通道与航向通道不可互代）"])
    ra_all = [h["rA"] for h in have]
    med = statistics.median(ra_all)
    spread = (max(ra_all) - min(ra_all)) / statistics.mean(ra_all)
    cw = [h["rA"] for h in have if h["dir"].startswith("CW")]
    ccw = [h["rA"] for h in have if h["dir"].startswith("CC")]
    dir_gap = None
    if cw and ccw:
        mc, mw = statistics.median(cw), statistics.median(ccw)
        dir_gap = abs(mc - mw) / ((mc + mw) / 2.0)
        detail.append(f"CW 中位 r_A={mc:.4f}（n={len(cw)}） / CCW 中位 r_A={mw:.4f}（n={len(ccw)}）"
                      f" ⇒ 方向差 {dir_gap * 100:.1f}%")
    detail.append(f"全体中位 r_A={med:.4f}，四趟离散度 {spread * 100:.1f}%")

    rb_all = [h["rB"] for h in have if h["rB"] is not None]
    rb_med = statistics.median(rb_all) if rb_all else None
    new = current * med

    # ---- 几何下界（R4 方案 §0.1）----
    geo = None
    if ticks_per_meter and track_m:
        geo = 2.0 * math.pi * track_m * ticks_per_meter
        detail.append(f"几何预期 2π·{track_m:.3f}·{ticks_per_meter:.1f} = {geo:.1f}"
                      f"（实测 {med * current:.1f} ⇒ 比 ×{med * current / geo:.3f}）")
        if abs(new - geo) / geo > 0.10:
            detail.append(f"⚠ 实测与几何差 {abs(new - geo) / geo * 100:.1f}% > 10%："
                          "原地转有横向刮擦、滑移会计入，需在报告中解释（方案 §0.1 提醒）")

    # ---- 陀螺交叉判据（方案 §2 交叉表）----
    if rb_med is not None:
        if abs(rb_med - 1.0) <= 0.02 and abs(med - 1.0) <= 0.02:
            detail.append("交叉：r_A≈1 且 r_B≈1 ⇒ 编码器与陀螺互相印证且都对地面")
        elif abs(rb_med - 1.0) > 0.02 and abs(med - 1.0) <= 0.02:
            detail.append("交叉：r_A≈1 但 r_B 偏离 ⇒ 陀螺/航向通道有问题（或零偏）："
                          "以源 A vs 源 C 为准，陀螺另立标定项")
        elif abs(rb_med - 1.0) <= 0.02 and abs(med - 1.0) > 0.02:
            detail.append("交叉：★ 「编码器配置错」指纹（r_A 偏、r_B≈1）")
        else:
            detail.append("交叉：r_A、r_B 都偏离 ⇒ 两者至少一个的基准有问题，先复核地面 720° 基准")
    else:
        detail.append("无陀螺数 ⇒ 交叉判据不可用（只有源 A vs 源 C）")

    # ---- 判档（方案 §2 / §3.1：每档只做一件事）----
    if dir_gap is not None and dir_gap > 0.05:
        return dict(verdict="REFUSE", new=new,
                    action="先解决机械问题（CW/CCW 方向差 >5% ⇒ 单侧滑移/阻力），再谈标定值",
                    detail=detail, geo=geo, rA=med, rB=rb_med)
    if spread > 0.05:
        return dict(verdict="REFUSE", new=new,
                    action="重测（四趟离散度 >5% ⇒ 手控转速不稳/采集不同步，改用栈内命令转）",
                    detail=detail, geo=geo, rA=med, rB=rb_med)
    dev = abs(med - 1.0)
    if dev <= 0.02:
        return dict(verdict="KEEP", new=current,
                    action=f"|r−1|={dev * 100:.2f}% ≤ 2% ⇒ 收敛，冻结 {_fmt(current, 1)}，不改配置",
                    detail=detail, geo=geo, rA=med, rB=rb_med)
    if dev <= 0.05:
        return dict(verdict="REFUSE", new=new,
                    action=f"|r−1|={dev * 100:.2f}% 落在 2~5%：重测该方向 1 趟取中位（此档不许改配置）",
                    detail=detail, geo=geo, rA=med, rB=rb_med)
    if not allow_mechanical_fix:
        return dict(verdict="REFUSE", new=new,
                    action=f"|r−1|={dev * 100:.1f}% > 5%：先做一次机械/轮距核查（方案 §3.2），"
                           "核查结论先于任何配置改动；若确认稳定偏离再加 --allow-mechanical-fix",
                    detail=detail, geo=geo, rA=med, rB=rb_med)
    delta_pct = (new / current - 1.0) * 100.0
    if abs(delta_pct) > cfg["guard_pct"]:
        return dict(verdict="REFUSE", new=new,
                    action=f"改动 {delta_pct:+.1f}% 超护栏 ±{cfg['guard_pct']:.0f}% ⇒ 需 --force 才写",
                    detail=detail, geo=geo, rA=med, rB=rb_med)
    return dict(verdict="SET", new=new,
                action=f"回填 {_fmt(current, 1)} → {_fmt(new, 1)}（{delta_pct:+.2f}%，"
                       "已按 §3.2 认定「配置值错」）",
                detail=detail, geo=geo, rA=med, rB=rb_med)


def calc_yaw_trim(values: dict, current: float, cfg: dict) -> dict:
    """R3 横向：漂移 cm + 行程 m → yaw_trim_deg（符号：右漂 ⇒ 负值）。"""
    lat_cm = _dig(values, "r3.lat_drift_cm")
    travel = _dig(values, "r3.belief_travel_m")
    explicit = _dig(values, "yaw_trim_deg")
    if explicit is not None:
        new = float(explicit)
        kind = "SET" if abs(new - current) > TOL_YAW_TRIM_DEG else "KEEP"
        return dict(verdict=kind, new=(new if kind == "SET" else current),
                    action=f"显式给定 yaw_trim_deg={new:+.1f}（现 {current:+.1f}）", detail=[])
    if lat_cm is None or travel in (None, 0):
        return dict(verdict="PENDING", new=None,
                    action="补 r3.lat_drift_cm（卷尺横向漂移 cm）与 r3.belief_travel_m（行程 m）",
                    detail=["缺实测数：横向通道未闭环"])
    if not _as_bool(_dig(values, "r3.lat_drift_confirmed")):
        return dict(verdict="REFUSE", new=None,
                    action="现场确认符号约定后把 r3.lat_drift_confirmed 置 true（+ = 车偏右 ⇒ 取负）",
                    detail=[f"横向漂移 {float(lat_cm):+.1f} cm @ {float(travel):.2f} m；"
                            "符号搞反会把漂移放大（tasks/localization.py:176）"])
    lat_cm = float(lat_cm)
    travel = float(travel)
    suggested = -math.degrees(math.atan2(lat_cm / 100.0, travel))
    detail = [f"漂移 {lat_cm:+.1f} cm @ {travel:.3f} m ⇒ {math.degrees(math.atan2(abs(lat_cm) / 100.0, travel)):.2f}°",
              f"符号约定（+ = 偏右 ⇒ 负值）⇒ suggested = {suggested:+.2f}°，四舍五入 {suggested:+.1f}°",
              f"容差参考：4 cm @ 4.55 m ≈ 0.50°"]
    new = round(suggested, 1)
    if abs(new - current) <= TOL_YAW_TRIM_DEG:
        return dict(verdict="KEEP", new=current,
                    action=f"|Δ| {abs(new - current):.2f}° ≤ {TOL_YAW_TRIM_DEG}° ⇒ 保留 {current:+.1f}",
                    detail=detail)
    if abs(new - current) > cfg["guard_abs"]:
        return dict(verdict="REFUSE", new=new,
                    action=f"改动 {abs(new - current):.2f}° 超护栏 {cfg['guard_abs']:.1f}°："
                           "复核符号约定与行程（疑似反号或量错），确认后加 --force",
                    detail=detail)
    return dict(verdict="SET", new=new,
                action=f"回填 {current:+.1f} → {new:+.1f}（Δ {new - current:+.2f}°）", detail=detail)


def calc_wheelbase(values: dict, current: float, cfg: dict) -> dict:
    meas = _dig(values, "geometry.wheelbase_m")
    if meas is None:
        return dict(verdict="PENDING", new=None,
                    action="卷尺量轴距（前后轮中心距）后填 geometry.wheelbase_m；不量就保持 0.157",
                    detail=["轴距不是轮距；轮距只进几何校核"])
    meas = float(meas)
    detail = [f"卷尺轴距 {meas:.3f} m vs 现值 {current:.3f} m（Δ {meas - current:+.3f} m）"]
    if abs(meas - current) <= TOL_WHEELBASE_M:
        return dict(verdict="KEEP", new=current,
                    action=f"|Δ| ≤ {TOL_WHEELBASE_M * 1000:.0f} mm ⇒ 不改", detail=detail)
    if abs(meas - current) / current * 100 > cfg["guard_pct"]:
        return dict(verdict="REFUSE", new=meas,
                    action=f"改动 {abs(meas - current) / current * 100:.1f}% 超护栏 ⇒ 复核卷尺（别把轮距当轴距）",
                    detail=detail)
    return dict(verdict="SET", new=meas,
                action=f"回填 {current:.3f} → {meas:.3f}（Δ {meas - current:+.3f} m）", detail=detail)


def calc_wheel_track(values: dict, current_cfg: dict) -> dict:
    """轮距：只有几何校核意义，没有 yaml 键。"""
    meas = _dig(values, "geometry.wheel_track_m")
    carw = carw_m()
    src = f"dzactuator.h CARW={carw}" if carw is not None else "dzactuator.h 读不到"
    if meas is None:
        return dict(verdict="PENDING", new=None,
                    action=f"卷尺量左右轮中心距后填 geometry.wheel_track_m（当前几何源：{src}）",
                    detail=["轮距不进 yaml；用于验算 ticksPer2PI 的几何下界"])
    meas = float(meas)
    detail = [f"实测轮距 {meas:.3f} m；几何源 {src}"]
    if carw is None:
        return dict(verdict="REFUSE", new=None, action="先确认 dzactuator.h 的 CARW 值", detail=detail)
    if abs(meas - carw) <= 0.005:
        return dict(verdict="KEEP", new=carw,
                    action=f"实测与 CARW={carw} 一致（Δtolerance 5 mm）⇒ 几何判据可用", detail=detail)
    return dict(verdict="REFUSE", new=None,
                action=f"实测 {meas:.3f} ≠ CARW={carw} ⇒ 复核卷尺；若实测无误则**另开卡**改车端 C++（本卡不碰 dzactuator）",
                detail=detail + [f"2π·{meas:.3f}·ticksPerMeter 与 2π·{carw}·ticksPerMeter 差 "
                                 f"{(meas - carw) / carw * 100:+.1f}%"])


# --------------------------------------------------------------------------- #
#  干跑 / 回填
# --------------------------------------------------------------------------- #
def evaluate(values: dict, config_path: pathlib.Path, root: pathlib.Path,
             allow_mechanical_fix: bool = False, force: bool = False) -> dict:
    """对每个通道出判定（纯计算 + 读盘，不写盘）。"""
    cfg_obj = Config.load(str(config_path))
    current = {c["id"]: (cfg_obj.get(c["yaml_key"]) if c["yaml_key"] else None) for c in CHANNELS}
    tpm = current.get("ticksPerMeter")
    track = _dig(values, "geometry.wheel_track_m")
    track = float(track) if track not in (None, "") else carw_m()

    result = {"config": str(config_path), "channels": []}
    for spec in CHANNELS:
        cid = spec["id"]
        entry = {"id": cid, "title": spec["title"], "card": spec["card"],
                 "yaml_key": spec["yaml_key"], "unit": spec["unit"],
                 "current": current[cid], "manual": bool(spec.get("manual")),
                 "inputs": list(spec["inputs"]), "note": spec.get("note", ""),
                 "comment_note": spec.get("comment_note", ""),
                 "pins": [dict(p) for p in spec.get("pins", ())],
                 "formula_fixtures": [dict(p) for p in spec.get("formula_fixtures", ())]}
        if spec.get("manual"):
            entry.update(verdict="MANUAL", new=None, action="本卡不回填（另开卡）；下面只登记需要同步的钉位",
                         detail=[f"现值 = {current[cid]!r}"])
            result["channels"].append(entry)
            continue
        try:
            if cid == "ticksPerMeter":
                r = calc_ticks_per_meter(values, float(current[cid]), spec)
            elif cid == "ticksPer2PI":
                r = calc_ticks_per_2pi(values, float(current[cid]), spec, ticks_per_meter=tpm,
                                       track_m=track, allow_mechanical_fix=allow_mechanical_fix)
            elif cid == "yaw_trim_deg":
                r = calc_yaw_trim(values, float(current[cid]), spec)
            elif cid == "wheelbase_m":
                r = calc_wheelbase(values, float(current[cid]), spec)
            elif cid == "wheel_track_m":
                r = calc_wheel_track(values, spec)
            else:  # pragma: no cover
                r = dict(verdict="PENDING", new=None, action="未实现", detail=[])
        except (TypeError, ValueError) as exc:
            r = dict(verdict="REFUSE", new=None, action=f"输入非法：{exc}", detail=[])
        if r["verdict"] == "SET" and not force:
            new, cur = r.get("new"), current[cid]
            if new is not None and cur:
                pct = abs(new / cur - 1.0) * 100 if cur else 0.0
                guard = spec.get("guard_pct")
                if guard is not None and pct > guard:
                    r = dict(r, verdict="REFUSE", action=f"改动 {pct:.1f}% 超护栏 ±{guard:.0f}% ⇒ 需 --force",
                             detail=list(r.get("detail", [])))
                ga = spec.get("guard_abs")
                if ga is not None and abs(new - cur) > ga:
                    r = dict(r, verdict="REFUSE", action=f"改动 {abs(new - cur):.2f} 超护栏 {ga} ⇒ 需 --force",
                             detail=list(r.get("detail", [])))
        entry.update(r)
        entry["diff"] = plan_yaml_edit(spec, current[cid], r, config_path) if r["verdict"] == "SET" else None
        entry["repins"] = plan_repins(spec, r, root) if r["verdict"] == "SET" else []
        entry["proposed_pin"] = proposed_pin_snippet(spec, current[cid], r)
        result["channels"].append(entry)

    counts: dict[str, int] = {}
    for e in result["channels"]:
        counts[e["verdict"]] = counts.get(e["verdict"], 0) + 1
    result["summary"] = counts
    return result


def plan_yaml_edit(spec: dict, current, r: dict, config_path: pathlib.Path) -> dict:
    """给出"要改哪一行、改成什么"（只读盘）。"""
    lines = _split_lines(_read_text(config_path))
    leaf = spec["yaml_leaf"]
    lineno, indent, old, tail = locate_yaml_line("\n".join(lines), leaf)
    new_scalar = _fmt(r["new"], spec["decimals"])
    new_line = f"{indent}{leaf}: {new_scalar}{tail}"
    return {"file": str(config_path), "line": lineno, "leaf": leaf,
            "old_scalar": old, "new_scalar": new_scalar,
            "old_line": lines[lineno - 1], "new_line": new_line}


def plan_repins(spec: dict, r: dict, root: pathlib.Path) -> list[dict]:
    """给出断言重钉清单（只读盘，校验 find 子串仍在）。"""
    out = []
    if r["new"] is None:
        return out
    for p in spec.get("pins", ()):
        path = root / p["file"]
        item = dict(p)
        item["abs"] = str(path)
        item["replace_to"] = p["replace"].format(v=_fmt(r["new"], spec["decimals"]),
                                                 v_short=_short(r["new"], spec["decimals"]))
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
            resolved, why = _resolve_pin_line(lines, p)
            if resolved is None:
                item.update(status="STALE", why=why)
            else:
                item.update(status="OK", line=resolved, drift=resolved - p["line"],
                            old_line=lines[resolved - 1],
                            line_after=repin_line(lines[resolved - 1], p["find"], item["replace_to"]))
        except OSError as exc:
            item.update(status="ERROR", why=str(exc))
        out.append(item)
    return out


def proposed_pin_snippet(spec: dict, current, r: dict) -> str | None:
    """无钉（或需新增钉）的键：给出可直接粘进测试的补钉片段。"""
    if not spec.get("yaml_key") or spec.get("manual"):
        return None          # 另开卡的键由那张卡自己补钉（本卡只登记钉位）
    if spec.get("pins"):
        return None
    value = r["new"] if r.get("verdict") == "SET" else current
    key = spec["yaml_key"]
    return (f'check("★ 标定回填：{key}（{spec["card"]} 实测/裁决）",\n'
            f'      cfg.get("{key}") == {_fmt(value, spec["decimals"])})')


def apply_config(edits: list[dict], config_path: pathlib.Path) -> list[str]:
    """真写 robot.yaml（逐行替换 + 备份 + 保留原行尾）。

    安全：每条改动都先比对"该行是否仍等于预览时的原文"，不等就**整批拒绝**（不写半个文件）。
    """
    log: list[str] = []
    if not edits:
        return ["无可回填项（没有 SET 通道）"]
    lines = _split_lines_keepends(_read_text(config_path))
    for e in edits:                                     # 先全量校验，再写
        idx = e["line"] - 1
        if not (0 <= idx < len(lines)):
            raise RuntimeError(f"{config_path}:{e['line']} 行号越界，拒绝写入")
        if lines[idx].rstrip("\r\n") != e["old_line"]:
            raise RuntimeError(f"{config_path}:{e['line']} 改前校验失败（文件已变），拒绝写入")
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    bak = config_path.with_name(config_path.name + f".bak_{stamp}")
    shutil.copy2(config_path, bak)
    log.append(f"备份 → {bak}")
    for e in edits:
        idx = e["line"] - 1
        cur = lines[idx]
        ending = "\r\n" if cur.endswith("\r\n") else ("\n" if cur.endswith("\n") else "")
        lines[idx] = e["new_line"] + ending
        log.append(f"{config_path}:{e['line']}  {e['old_line'].strip()}  →  {e['new_line'].strip()}")
    _write_text(config_path, "".join(lines))
    log.append(f"已写入 {config_path}（行尾保持原样："
               f"{'CRLF' if _read_text(config_path).count(chr(13) + chr(10)) else 'LF'}）")
    return log


def apply_repins(plan: list[dict]) -> list[str]:
    """真写断言（按 file 分组，逐行替换；STALE 项直接拒绝并抛错）。"""
    log: list[str] = []
    stale = [p for p in plan if p.get("status") != "OK"]
    if stale:
        raise RuntimeError("有 " + str(len(stale)) + " 条钉不是 OK 状态（行号漂移），拒绝写入："
                           + "; ".join(f"{p['file']}:{p['line']}({p.get('status')})" for p in stale))
    by_file: dict[str, list[dict]] = {}
    for p in plan:
        by_file.setdefault(p["file"], []).append(p)
    for rel, items in by_file.items():
        path = pathlib.Path(items[0]["abs"])
        lines = _split_lines_keepends(_read_text(path))
        for p in items:                                 # 先全量校验，再写
            idx = p["line"] - 1
            if not (0 <= idx < len(lines)):
                raise RuntimeError(f"{rel}:{p['line']} 行号越界，拒绝写入")
            if lines[idx].rstrip("\r\n") != p["old_line"]:
                raise RuntimeError(f"{rel}:{p['line']} 改前校验失败，拒绝写入")
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        bak = path.with_name(path.name + f".bak_{stamp}")
        shutil.copy2(path, bak)
        log.append(f"备份 → {bak.name}")
        for p in sorted(items, key=lambda x: x["line"]):
            idx = p["line"] - 1
            cur = lines[idx]
            ending = "\r\n" if cur.endswith("\r\n") else ("\n" if cur.endswith("\n") else "")
            lines[idx] = p["line_after"] + ending
            log.append(f"{rel}:{p['line']}  {p['find']} → {p['replace_to']}")
        _write_text(path, "".join(lines))
        log.append(f"已写入 {rel}")
    return log


# --------------------------------------------------------------------------- #
#  自检（selfcheck）
# --------------------------------------------------------------------------- #
def selfcheck(config_path: pathlib.Path, root: pathlib.Path) -> dict:
    """对账：键可读 / 行锚点唯一 / 钉未漂 / 覆盖缺口 / 交叉一致性。"""
    errors: list[str] = []
    warns: list[str] = []
    info: list[str] = []
    cfg = Config.load(str(config_path))
    text = config_path.read_text(encoding="utf-8")
    for spec in CHANNELS:
        cid = spec["id"]
        if spec.get("yaml_key"):
            val = cfg.get(spec["yaml_key"])
            if val is None:
                errors.append(f"[{cid}] {spec['yaml_key']} 在 {config_path.name} 里读不到")
            else:
                info.append(f"[{cid}] {spec['yaml_key']} = {val!r}  ← {', '.join(spec['inputs']) or '（无输入）'}")
            if spec.get("yaml_leaf"):
                try:
                    lineno, _ind, old, _tail = locate_yaml_line(text, spec["yaml_leaf"])
                    info.append(f"[{cid}] yaml 行锚点 = {config_path.name}:{lineno}（{spec['yaml_leaf']}: {old}）")
                except ValueError as exc:
                    errors.append(f"[{cid}] {exc}")
        else:
            info.append(f"[{cid}] 无 yaml 键（几何输入）：{spec['title']}")
        # 钉的新鲜度
        for p in spec.get("pins", ()):
            path = root / p["file"]
            if not path.is_file():
                errors.append(f"[{cid}] 钉指向不存在的文件 {p['file']}")
                continue
            lines = path.read_text(encoding="utf-8").splitlines()
            resolved, why = _resolve_pin_line(lines, p)
            if resolved is None:
                errors.append(f"[{cid}] {p['file']}:{p['line']} {why}")
            elif resolved != p["line"]:
                info.append(f"[{cid}] {p['file']} 钉由行 {p['line']} 内容式定位到行 {resolved}（目标文件中部插行，自动跟随）")
        # 覆盖缺口
        if spec.get("yaml_key") and not spec.get("pins"):
            warns.append(f"[{cid}] {spec['yaml_key']} 无任何测试断言钉住 ⇒ 回填后测试不会红（{spec['title']}）")
        for p in spec.get("formula_fixtures", ()):
            path = root / p["file"]
            if not path.is_file():
                warns.append(f"[{cid}] 公式夹具文件不存在：{p['file']}")
                continue
            lines = path.read_text(encoding="utf-8").splitlines()
            resolved, why = _resolve_pin_line(lines, p)
            if resolved is None:
                warns.append(f"[{cid}] 公式夹具钉失效：{p['file']}:{p['line']} {why}")
    # 交叉一致性
    tpm = cfg.get("dzactuator.ticksPerMeter")
    t2p = cfg.get("dzactuator.ticksPer2PI")
    carw = carw_m()
    if tpm and t2p and carw:
        geo = 2 * math.pi * carw * float(tpm)
        info.append(f"[交叉] 几何预期 ticksPer2PI = 2π·{carw}·{tpm:.1f} = {geo:.1f}；"
                    f"现值 {t2p} ⇒ ×{float(t2p) / geo:.3f}")
    cruise = cfg.get("pursuit.cruise_speed_mps")
    legacy = cfg.get("path_executor.linear_speed")
    if cruise is not None and legacy is not None:
        if abs(float(cruise) - float(legacy)) > 1e-9:
            errors.append(f"[交叉] pursuit.cruise_speed_mps={cruise} ≠ path_executor.linear_speed={legacy}")
        else:
            info.append(f"[交叉] 巡航双键同步：{cruise} == {legacy} ✓")
    else:
        warns.append("[交叉] 巡航双键之一读不到，无法核对同步性")
    info.append(f"[交叉] 轮距来源 dzactuator.h CARW = {carw}")
    # yaw_trim 无 SCHEMA
    if "localization.yaw_trim_deg" not in _schema_keys():
        warns.append("[yaw_trim_deg] 不在 core/config.py 的 SCHEMA 里 ⇒ 无范围校验（写错不报错）；"
                     "工具内的 ±10° 护栏是本卡新加的")
    return {"errors": errors, "warns": warns, "info": info}


def _schema_keys() -> set[str]:
    try:
        from dzbrain.core.config import SCHEMA
    except Exception:  # pragma: no cover
        return set()
    return set(SCHEMA.keys())


def scan_untracked_refs(old_literal: str, root: pathlib.Path,
                        pins: tuple | list, extra_skip: set[pathlib.Path] | None = None) -> list[str]:
    """扫仓库里出现旧值的引用，报告**未登记进本表**的那些（提示不能自动替换）。

    跳过（口径 = **按路径判**，不靠"临时目录里恰好没有副本"）：
      · 路径上任意一段命中 `_SCAN_SKIP` 或以 `_tmp_` 开头（含 `_tmp_verify/` 等兄弟目录，
        以及其中形如 `tests/test_core.py` 的变异副本 —— 文件名本身不带 `_tmp_`，见 t16）；
      · 已登记的钉文件、`_HISTORICAL_SNAPSHOTS` 注册的历史快照/草稿、本工具自身与模板。
    """
    files = {(root / p["file"]).resolve() for p in pins}
    files |= {p.resolve() for p in (extra_skip or set())}
    files |= _scan_snapshot_paths(root)
    files |= {_HERE.resolve(), (_HERE.parent / "calib_backfill_template.yaml").resolve()}
    hits: list[str] = []
    for path in root.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in _SCAN_EXT:
            continue
        if any(_skip_dir(part) for part in path.parts):
            continue
        if path.resolve() in files:
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        if old_literal in text:
            rel = path.relative_to(root).as_posix()
            cnt = text.count(old_literal)
            hits.append(f"{rel}（{cnt} 处）")
    return sorted(hits)


# --------------------------------------------------------------------------- #
#  模板
# --------------------------------------------------------------------------- #
TEMPLATE_TEXT = """\
# =============================================================================
#  标定实测值模板（t2 / n2）—— 由队长填写；null = 还没测
#  用法：python tools\\calib_backfill.py --values tools\\calib_backfill_template.yaml
#        真写：加 --apply-config（改 config/robot.yaml）与/或 --apply-tests（重钉断言）
#  ⚠ 只有"卷尺/对线"实测数才算数；日志信念侧取数（residual_measured_by: log）会被
#    「到位吸附」抹平，工具会直接 REFUSE（见 _tmp_session/car_calib/R3_verdict.md §③）。
# =============================================================================
meta:
  date: null            # YYYY-MM-DD
  operator: null        # 谁量的
  note: null

# ---- R3 直线标定（ticksPerMeter / yaw_trim_deg）------------------------------
r3:
  belief_travel_m: null         # 日志本趟信念行程（m），例：4.55
  long_residual_cm: null        # 卷尺：停车点距目标格心的**纵向**残差（cm，+ = 欠程/停不到位）
  lat_drift_cm: null            # 卷尺：轨迹相对名义直线的**横向**漂移（cm，+ = 车偏右）
  lat_drift_confirmed: false    # 符号约定已现场确认？（+ = 偏右 ⇒ yaw_trim 取负）
  yaw_deg: null                 # 终点航向与名义航向之差（°，仅记录，不参与回填）
  residual_measured_by: null    # tape（卷尺，可用）/ log（日志，工具会拒绝）

# ---- R4 原地转圈（ticksPer2PI）---------------------------------------------
r4:
  turns: 2                      # 每趟转的整圈数（方案 = 2 圈 = 720°）
  runs:                         # CW/CCW 各 2 趟；delta_counts 取 |左|、|右| 的**平均**增量
    - {dir: CW,  delta_counts: null, yaw_gyro_deg: null}
    - {dir: CW,  delta_counts: null, yaw_gyro_deg: null}
    - {dir: CCW, delta_counts: null, yaw_gyro_deg: null}
    - {dir: CCW, delta_counts: null, yaw_gyro_deg: null}

# ---- 几何（卷尺两把，只进判据/轴距键）--------------------------------------
geometry:
  wheel_track_m: null           # 左右轮触地点中心距（≠ 轴距！）。null → 取 dzactuator.h 的 CARW
  wheelbase_m: null             # 轴距（前后轮中心距）→ pursuit.wheelbase_m；不量就保持 0.157
"""


def build_template() -> str:
    return TEMPLATE_TEXT


# --------------------------------------------------------------------------- #
#  打印
# --------------------------------------------------------------------------- #
def print_list() -> None:
    print("=" * 96)
    print("标定回填映射总表（本工具的唯一真源）")
    print("=" * 96)
    for spec in CHANNELS:
        kind = "MANUAL（另开卡）" if spec.get("manual") else ("几何输入" if not spec.get("yaml_key") else "可回填")
        print(f"[{spec['id']}] {spec['title']}  卡={spec['card']}  类型={kind}")
        print(f"    yaml 键   : {spec['yaml_key'] or '（无）'}")
        print(f"    输入      : {', '.join(spec['inputs']) or '（无）'}")
        pins = spec.get("pins", ())
        if pins:
            for p in pins:
                print(f"    钉        : {p['file']}:{p['line']}  含 {p['find']!r}")
        else:
            print("    钉        : ★ 无（覆盖缺口）")
        if spec.get("formula_fixtures"):
            print(f"    公式夹具  : {len(spec['formula_fixtures'])} 处（口径同步用，非硬钉）")
        if spec.get("comment_note"):
            print(f"    需同步文档: {spec['comment_note']}")
    print("-" * 96)
    print(f"轮距几何源：dzactuator.h CARW={carw_m()}")


def print_selfcheck(rep: dict) -> None:
    print("=" * 96)
    print("自检：keys / 行锚点 / 钉新鲜度 / 覆盖缺口 / 交叉一致性")
    print("=" * 96)
    for line in rep["info"]:
        print("  · " + line)
    print("-" * 96)
    for line in rep["warns"]:
        print("  ⚠ WARN  " + line)
    for line in rep["errors"]:
        print("  ✗ ERROR " + line)
    print("-" * 96)
    print(f"结论：error {len(rep['errors'])} / warn {len(rep['warns'])}"
          + ("   ✅ 干净" if not rep["errors"] else "   ❌ 需先修"))
    if rep["errors"]:
        print("（无需回填时也可以先跑这一步：它只读盘，确认标定断言的锚点还在原处）")


def print_result(res: dict, old_literals: dict | None = None) -> None:
    print("=" * 96)
    print(f"标定回填 · 干跑   config={res['config']}")
    print("=" * 96)
    for e in res["channels"]:
        mark = {"KEEP": "✅ 冻结", "SET": "→ 回填", "PENDING": "⏳ 待实测",
                "REFUSE": "⛔ 拒绝", "MANUAL": "▷ 另开卡"}[e["verdict"]]
        cur = "—" if e["current"] is None else repr(e["current"])
        print(f"\n[{e['id']}] {e['title']}   现值={cur} {e['unit']}")
        print(f"  {mark}  {e['action']}")
        for d in e.get("detail", []):
            print(f"         {d}")
        if e.get("diff"):
            d = e["diff"]
            print(f"  配置改动 {d['file']}:{d['line']}")
            print(f"      - {d['old_line'].strip()}")
            print(f"      + {d['new_line'].strip()}")
        if e.get("repins"):
            print(f"  断言重钉 {len(e['repins'])} 处：")
            for p in e["repins"]:
                tag = "OK" if p.get("status") == "OK" else f"✗{p.get('status')}"
                print(f"      [{tag}] {p['file']}:{p['line']}  {p['find']} → {p['replace_to']}")
                if p.get("status") != "OK":
                    print(f"             {p.get('why')}")
        if e.get("proposed_pin"):
            print("  ★ 建议补钉（当前无断言钉住该键）：")
            for ln in e["proposed_pin"].splitlines():
                print("      " + ln)
        if e.get("comment_note"):
            print(f"  需同步文档/注释：{e['comment_note']}")
        if e.get("manual") and e.get("pins"):
            print("  （另开卡时要同步的钉位）")
            for p in e["pins"]:
                print(f"      {p['file']}:{p['line']}  含 {p['find']!r}")
    if old_literals:
        print("\n" + "-" * 96)
        print("未登记的旧值引用（不会自动改；回填后需人工核对是否过期）：")
        for cid, hits in old_literals.items():
            if hits:
                print(f"  [{cid}] 旧值 {hits['literal']} 还出现在：")
                for h in hits["files"]:
                    print(f"        {h}")
    s = res["summary"]
    print("\n" + "=" * 96)
    print("汇总：" + " / ".join(f"{k} {v}" for k, v in sorted(s.items())))
    if s.get("REFUSE"):
        print("⚠ 有 REFUSE 通道：按上面的「唯一下一步」办完再回填（不要绕过去）。")
    if s.get("PENDING"):
        print("⏳ 有 PENDING 通道：等队长实测数（本工具此刻不改任何文件）。")
    print("=" * 96)


def to_markdown(res: dict, scan: dict | None = None) -> str:
    out = [f"# 标定回填干跑报告（{_dt.datetime.now().strftime('%Y-%m-%d %H:%M')}）", "",
           f"配置文件：`{res['config']}`；模式：**干跑**（未写任何文件）。", ""]
    out += ["| 通道 | 卡 | 现值 | 判定 | 动作 |", "|---|---|---|---|---|"]
    for e in res["channels"]:
        out.append(f"| `{e['id']}` | {e['card']} | {e['current']!r} | **{e['verdict']}** | {e['action']} |")
    out += ["", f"汇总：{res['summary']}", ""]
    for e in res["channels"]:
        out += [f"## {e['id']} — {e['title']}", "", f"- 判定：**{e['verdict']}**", f"- 动作：{e['action']}"]
        for d in e.get("detail", []):
            out.append(f"- 依据：{d}")
        if e.get("diff"):
            d = e["diff"]
            out += ["", f"配置改动 `{d['file']}:{d['line']}`：", "```diff",
                    f"- {d['old_line'].strip()}", f"+ {d['new_line'].strip()}", "```"]
        if e.get("repins"):
            out += ["", "| 钉位 | 行 | 原文 | 改为 | 状态 |", "|---|---|---|---|---|"]
            for p in e["repins"]:
                out.append(f"| `{p['file']}` | {p['line']} | `{p['find']}` | `{p['replace_to']}` | {p.get('status')} |")
        if e.get("proposed_pin"):
            out += ["", "建议补钉：", "```python", e["proposed_pin"], "```"]
        if e.get("comment_note"):
            out.append(f"- 需同步注释/文档：{e['comment_note']}")
        out.append("")
    if scan:
        out += ["## 未登记的旧值引用（人工核对）", ""]
        for cid, hits in scan.items():
            if hits:
                out.append(f"- `{cid}` 旧值 `{hits['literal']}`：{', '.join(hits['files'])}")
        out.append("")
    return "\n".join(out) + "\n"


# --------------------------------------------------------------------------- #
#  CLI
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="calib_backfill.py",
        description="标定参数回填工具链（实测值 → robot.yaml 键 → 测试断言重钉清单）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=str(CONFIG_DEFAULT), help="目标 robot.yaml（默认工作区那份）")
    p.add_argument("--root", default=str(_WS), help="仓库根（钉位相对路径解析用）")
    p.add_argument("--values", default=None,
                   help="实测值 YAML（默认 tools/calib_backfill_template.yaml → 全部 PENDING）")
    p.add_argument("--selfcheck", action="store_true", help="只读自检：键/行锚点/钉新鲜度/覆盖缺口")
    p.add_argument("--list", action="store_true", dest="list_map", help="打印映射总表")
    p.add_argument("--template", action="store_true", help="打印待填写的实测值模板")
    p.add_argument("--emit-template", default=None, metavar="PATH", help="把模板写到指定路径")
    p.add_argument("--apply-config", action="store_true", help="真写 robot.yaml（默认干跑）")
    p.add_argument("--apply-tests", action="store_true", help="真写测试断言（默认干跑）")
    p.add_argument("--allow-mechanical-fix", action="store_true",
                   help="R4 判 >5%% 时允许按 §3.2 结论回填（默认拒绝，先做机械核查）")
    p.add_argument("--force", action="store_true", help="越过护栏（需在报告里写明理由）")
    p.add_argument("--json", action="store_true", help="机器可读输出")
    p.add_argument("--emit-report", default=None, metavar="PATH", help="把干跑结论写成 Markdown")
    p.add_argument("--scan-old", action="store_true", help="额外扫描未登记的旧值引用（慢一点）")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    root = pathlib.Path(args.root).resolve()
    config_path = pathlib.Path(args.config).resolve()

    if args.emit_template:
        out = pathlib.Path(args.emit_template)
        out.write_text(build_template(), encoding="utf-8", newline="\n")
        print(f"模板已写入 {out}")
    if args.template:
        print(build_template())
        return 0
    if args.list_map:
        print_list()
        return 0
    if args.selfcheck and not args.values and not args.apply_config:
        rep = selfcheck(config_path, root)
        if args.json:
            print(json.dumps(rep, ensure_ascii=False, indent=2))
        else:
            print_selfcheck(rep)
        return 3 if rep["errors"] else 0

    values_path = pathlib.Path(args.values) if args.values else TEMPLATE_DEFAULT
    if not values_path.is_file():
        print(f"✗ 找不到实测值文件：{values_path}", file=sys.stderr)
        return 3
    values = load_values(values_path)

    res = evaluate(values, config_path, root,
                   allow_mechanical_fix=args.allow_mechanical_fix, force=args.force)
    res["values"] = str(values_path)
    scan = None
    if args.scan_old:
        scan = {}
        for e in res["channels"]:
            if e.get("manual") or not e.get("yaml_key") or e.get("current") is None:
                continue
            lit = _short(e["current"], CHANNEL_BY_ID[e["id"]]["decimals"])
            if len(lit) < 3:
                continue
            files = scan_untracked_refs(lit, root, e.get("pins", ()), extra_skip={config_path,
                                       root / "src/dzbrain/tests/test_calib_backfill.py"})
            if files:
                scan[e["id"]] = {"literal": lit, "files": files}

    applied: list[str] = []
    refused = [e for e in res["channels"] if e["verdict"] == "REFUSE"]
    to_set = [e for e in res["channels"] if e["verdict"] == "SET"]

    if args.apply_config:
        edits = [e["diff"] for e in to_set if e.get("diff")]
        print("=" * 96)
        print(f"★ --apply-config：将改写 {config_path}（{len(edits)} 处）")
        print("=" * 96)
        applied += apply_config(edits, config_path)
        for line in applied:
            print("  " + line)
    if args.apply_tests:
        plan = [p for e in to_set for p in e.get("repins", [])]
        print("=" * 96)
        print(f"★ --apply-tests：将重钉 {len(plan)} 处断言")
        print("=" * 96)
        applied += apply_repins(plan)
        for line in applied:
            print("  " + line)
    if args.apply_config or args.apply_tests:
        rep = selfcheck(config_path, root)
        print("-" * 96)
        print(f"改后 selfcheck：error {len(rep['errors'])} / warn {len(rep['warns'])}")
        for line in rep["errors"]:
            print("  ✗ " + line)
        for line in rep["warns"]:
            print("  ⚠ " + line)

    if args.emit_report:
        _write_text_lf(pathlib.Path(args.emit_report), to_markdown(res, scan))
        print(f"\n报告已写入 {args.emit_report}")
    if args.json:
        print(json.dumps(res, ensure_ascii=False, indent=2, default=str))
    else:
        print_result(res, scan)
    if applied:
        print("已写盘动作：")
        for line in applied:
            print("  " + line)
    return 2 if refused else 0


if __name__ == "__main__":
    raise SystemExit(main())
