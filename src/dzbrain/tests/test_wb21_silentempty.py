# -*- coding: utf-8 -*-
"""WB-21：「静默得到一个空对象」比报错更难查 —— bench 入口的空路线显式拦截。

## 为什么需要这张卡（本卡的"四条路径 × 四种行为"）
同一种「`route.tokens` 为空」，仓里有四条路径、四种处理规格（**同一输入、四种结局**）：

| 路径 | 行为 | 可读性 | 本卡 |
|---|---|---|---|
| `tasks/mission.py:2949` `_load_tokens` | `raise ValueError("route.tokens 为空，无法载入路线")` | ✅ 可读 | **不改**（只读对照） |
| `main.py:222`（**有地图**） | `validate_route` → **R001 error** → 启动门禁拦 | ✅ 可读 | **不改**（R001 已拦，改了是画蛇添足） |
| `main.py:222`（**布局模式 / 无地图**） | 整块 `if map_cfg and not errors` 被跳过，报的是"未配置地图" | ⚠️ **归因错位** | **只记录不改**（动它会改地图门禁语义，属另一张卡） |
| `main.py:2823`（`--bench`） | **裸 `ValueError` Traceback** 打在 `Path(route)` | ❌ 不可读 | **本卡改这里** |

⇒ 不是"某处静默"，而是 **同一件事在不同入口有不同的处理规格**（L1 实例）。
⇒ 还有一个更早的观察被本卡实测推翻：**`validate_route(空 Route)` 不是 0 error，是 1 条 R001 error**
（我原先推断"0 error ⇒ 静默放行"，实测错；这条写进卡面防后人再踩）。

## 覆盖
- §1 最小复现：空 ⇒ 空 Route（**纯函数视角**），并证明下游 `Path(route)` 才炸
- §2 四条路径实测对照（每行给可复跑命令）
- §3 **端到端**（L1 推论）：真跑 `--bench`，断言可读报错 + rc=2 + **无 Traceback**
- §4 三态区分：`missing`（键不存在）/ `empty`（显式 `[]`）/ `short`（不足 2 点）
  —— 用 `cfg._flat` 判，因为 `cfg.get()` **分不出来**（SCHEMA 兜底使两者都返回 `[]`）
- §5 **不误伤**：正常路径（自动规划成功回写 68 token）必须**不被拦**、rc=0
  —— 这是本卡最重要的负例：改了闸门却没误伤 --bench 的常规用法
- §6 只读性 / 未改 `mission.py` / 未改 `:222`

跑法（工作区根 `dzrobot_v2/`）：
    PYTHONPATH=src/dzbrain python src/dzbrain/tests/test_wb21_silentempty.py
全过 exit 0。
"""

from __future__ import annotations

import copy
import hashlib
import pathlib
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
try:
    sys.stdout.reconfigure(errors="replace")
    sys.stderr.reconfigure(errors="replace")
except Exception:
    pass

import yaml                                                        # noqa: E402

from harness import check, finish, section                         # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]
CFG_MAIN = WS / "config" / "robot.yaml"
FIXTURE_MAP = WS / "src" / "dzbrain" / "tests" / "fixtures" / "map_field7_t24.yaml"
MAIN_PY = WS / "src" / "dzbrain" / "dzbrain" / "main.py"
MISSION_PY = WS / "src" / "dzbrain" / "dzbrain" / "tasks" / "mission.py"

_TMP = pathlib.Path(tempfile.mkdtemp(prefix="wb21_"))
_CFG_SHA_BEFORE = hashlib.sha256(CFG_MAIN.read_bytes()).hexdigest()
_RAW = yaml.safe_load(CFG_MAIN.read_text(encoding="utf-8"))

ENV = {"PYTHONPATH": str(WS / "src" / "dzbrain"), "PYTHONIOENCODING": "utf-8",
       "PYTHONUTF8": "1", "PATH": __import__("os").environ.get("PATH", "")}


def make_cfg(*, tokens=_RAW["/dzbrain"]["ros__parameters"].get("route", {}).get("tokens", []),
             omit_tokens=False, auto_plan=None, tag="x") -> pathlib.Path:
    """派生一份临时 robot.yaml。`omit_tokens=True` ⇒ **删掉** route.tokens 键（测 missing）。"""
    raw = copy.deepcopy(_RAW)
    planner = raw["/dzbrain"]["ros__parameters"].setdefault("route", {})
    if omit_tokens:
        planner.pop("tokens", None)
    else:
        planner["tokens"] = tokens
    if auto_plan is not None:
        raw["/dzbrain"]["ros__parameters"]["planner"]["auto_plan_enabled"] = auto_plan
    path = _TMP / f"robot_{tag}.yaml"
    path.write_text(yaml.safe_dump(raw, allow_unicode=True), encoding="utf-8")
    return path


def run_bench(cfg_path: pathlib.Path, *extra: str, seconds: str = "1") -> tuple[int, str, str]:
    """真跑 `--bench`（**端到端**，不是调函数）。返回 (rc, stdout, stderr)。"""
    proc = subprocess.run(
        [sys.executable, "-m", "dzbrain.main", "--bench", "--seconds", seconds,
         "--config", str(cfg_path), *extra],
        capture_output=True, cwd=str(WS), env=ENV)
    return (proc.returncode,
            proc.stdout.decode("utf-8", errors="replace"),
            proc.stderr.decode("utf-8", errors="replace"))


# --------------------------------------------------------------------------- #
section("§1 最小复现：空 ⇒ 空 Route（纯函数视角）")
# --------------------------------------------------------------------------- #
from dzbrain.route.parser import parse_mission                       # noqa: E402
from dzbrain.route.validate import validate_route                    # noqa: E402
from dzbrain.pursuit.path import Path as PursuitPath                 # noqa: E402
from dzbrain.core.config import Config                               # noqa: E402

_empty_route = parse_mission([])
check("1.1 `parse_mission([])` **不报错**，返回一个合法空 Route（0 航点）",
      len(_empty_route) == 0 and type(_empty_route).__name__ == "Route",
      f"{_empty_route!r}")
try:
    _empty_route[0]
    _idx_err = None
except Exception as exc:                                            # noqa: BLE001
    _idx_err = type(exc).__name__
check("1.2 取 `route[0]` 才 IndexError（『空对象静默往下』的具体形态）",
      _idx_err == "IndexError", f"{_idx_err}")
try:
    PursuitPath(_empty_route)
    _path_err = None
except Exception as exc:                                            # noqa: BLE001
    _path_err = (type(exc).__name__, str(exc))
check("1.3 `Path(空 Route)` 抛 ValueError（bench 原本就死在这里，且无人 catch）",
      _path_err is not None and _path_err[0] == "ValueError", f"{_path_err}")

# --------------------------------------------------------------------------- #
section("§2 四条路径 × 四种行为（实测对照；每行给可复跑命令）")
# --------------------------------------------------------------------------- #
_map_cfg = yaml.safe_load(FIXTURE_MAP.read_text(encoding="utf-8"))
_live_cfg = Config.load(str(CFG_MAIN))

# 路径②：mission._load_tokens 的空输入行为（只读，不改 mission.py）
import inspect                                                     # noqa: E402
from dzbrain.tasks import mission as _mission                      # noqa: E402

_ls_src = inspect.getsource(_mission.MissionTask._load_tokens)
check("2.1 路径② `_load_tokens` 空输入 ⇒ raise ValueError（可读）",
      "if not tokens:" in _ls_src and "raise ValueError" in _ls_src)

# 路径③：main.py:222 有地图 ⇒ R001 拦（**推翻"静默放行"的原推断**）
_issues = validate_route(_empty_route, map_cfg=_map_cfg, cfg=_live_cfg)
_errs = [i for i in _issues if i.level == "error"]
check("2.2 路径③ `:222` 有地图 ⇒ `validate_route(空Route)` 返回 **R001 error**（不是 0 error！）",
      len(_errs) == 1 and _errs[0].code == "R001",
      f"issues={[i.code + '/' + i.level for i in _issues]}")
check("2.3 该 error 的原文点名『路线为空』（人一看就知道该配什么）",
      "路线为空" in str(_errs[0]) if _errs else False,
      str(_errs[0])[:60] if _errs else "(无)")

# 路径④：bench —— 本卡要改的那条。改前是裸 Traceback，见 §3 端到端。
check("2.4 路径④ `--bench` 原先**没有**前置校验：`parse_mission` 之后直接 `Path(route)`",
      True, "见 §3.2 修后行为 + 卡面执行报告里的改前实测")

# --------------------------------------------------------------------------- #
section("§3 端到端（L1 推论）：真跑 `--bench`，空路线必须**可读报错**")
# --------------------------------------------------------------------------- #
# 为什么要 --force：`main()` 里 `load_and_check`（:3329）先于 `run_bench`（:3385），
#   而它在"有地图"时会用 **R001** 把空路线拦掉，根本进不到 bench 分支。
#   ⇒ 要走到 bench 自己的闸门，必须跳过启动门禁（--force），
#     这也是现场真能到达这条路径的方式（自动规划失败 + 强行联调）。
_cfg_empty = make_cfg(tokens=[], auto_plan=False, tag="empty")
_rc1, _out1, _err1 = run_bench(_cfg_empty, "--force")
check("3.1 空 tokens + --force 进到 bench ⇒ 被本闸拦下，rc=2",
      _rc1 == 2, f"rc={_rc1}")
check("3.2 报错**点名 route.tokens**（不许只给一个退出码）",
      "bench 拒绝启动" in _out1 and "route.tokens" in _out1,
      (([ln for ln in _out1.splitlines() if "拒绝启动" in ln] or ["(无)"])[0])[:70])
check("3.3 报错给出**可行动作**（① 传地图自动规划 / ② 显式填 route.tokens）",
      "① 传一张地图" in _out1 and "② 显式给路线" in _out1)
check("3.4 ★ **没有裸 Traceback**（这正是本卡要消灭的形态）",
      "Traceback" not in _err1 and "Traceback" not in _out1,
      f"stderr 首行={(  _err1.strip().splitlines() or ['(空)'])[0][:60]}")
check("3.5 报错**声明与另外两条路径同口径**（不是新造一套规格）",
      "同一个口径" in _out1)

# --------------------------------------------------------------------------- #
section("§4 三态区分：missing / empty / short（`cfg.get()` 分不出，必须用 `_flat`）")
# --------------------------------------------------------------------------- #
# ★ 本卡坐实的实测事实：SCHEMA 兜底（`route.tokens` default=`[]`）使
#   **"键不存在"与"显式空列表"在 `cfg.get()` 层返回同一个 `[]`** ⇒ 分不出来。
#   唯一判据是 `"route.tokens" in cfg._flat`。不做区分的后果 = 归因错位（L14 同族）：
#   用户明明没配这一项，却被告知"你配了个空列表"。
_cfg_missing = make_cfg(omit_tokens=True, auto_plan=False, tag="missing")
_cfg_empty2 = make_cfg(tokens=[], auto_plan=False, tag="empty2")
_cfg_short = make_cfg(tokens=["A1"], auto_plan=False, tag="short")

_missing_cfg = Config.load(str(_cfg_missing))
_empty_cfg = Config.load(str(_cfg_empty2))
check("4.1 `cfg.get()` 对 missing / empty **返回同一个 `[]`**（所以不能用它判）",
      _missing_cfg.get("route.tokens") == [] and _empty_cfg.get("route.tokens") == [],
      f"missing={_missing_cfg.get('route.tokens')!r} empty={_empty_cfg.get('route.tokens')!r}")
check("4.2 `_flat` 能区分：missing=False / empty=True（**唯一可用判据**）",
      ("route.tokens" in _missing_cfg._flat) is False
      and ("route.tokens" in _empty_cfg._flat) is True)

_rc_m, _out_m, _err_m = run_bench(_cfg_missing, "--force")
_rc_e, _out_e, _err_e = run_bench(_cfg_empty2, "--force")
_rc_s, _out_s, _err_s = run_bench(_cfg_short, "--force", seconds="1")
check("4.3 missing ⇒ 报『**没有** route.tokens 这一项』（说清是没配，不是配错）",
      _rc_m == 2 and "没有" in _out_m and "route.tokens 这一项" in _out_m,
      ([ln for ln in _out_m.splitlines() if "拒绝启动" in ln] or ["(无)"])[0][:70])
check("4.4 empty ⇒ 报『**显式的空列表**』（与 4.3 措辞不同 ⇒ 三态可区分）",
      _rc_e == 2 and "显式的空列表" in _out_e,
      ([ln for ln in _out_e.splitlines() if "拒绝启动" in ln] or ["(无)"])[0][:70])
check("4.5 short（1 点）⇒ 报『只有 1 点（不足 2 点）』（第三种措辞）",
      _rc_s == 2 and "只有 1 点" in _out_s,
      ([ln for ln in _out_s.splitlines() if "拒绝启动" in ln] or ["(无)"])[0][:70])
# 三态措辞必须**互不相同** —— 直接把三条"拒绝启动"行拿出来做集合比较，
# 而不是自己拼一个恒真的表达式（本卡第一版就写成了恒真，这里改掉）。
_reason = lambda out: next((ln.strip() for ln in out.splitlines() if "拒绝启动" in ln), "")
_r_m, _r_e, _r_s = _reason(_out_m), _reason(_out_e), _reason(_out_s)
check("4.6 三态报错**互不相同**（不是同一句话套三种情况——那就等于没区分）",
      len({_r_m, _r_e, _r_s}) == 3,
      f"missing/empty/short 三行互异={len({_r_m, _r_e, _r_s}) == 3}")

# --------------------------------------------------------------------------- #
section("§5 ★ 不误伤：正常路径（自动规划成功）必须**不被拦**（最重要的负例）")
# --------------------------------------------------------------------------- #
# 改了闸门却没把常规用法打断 —— 这才是"闸门装对了"的证据。
# 正常 `--bench --map <图>` 的时序：`load_and_check` 先跑（空 tokens ⇒ R001 拦）⇒
#   **必然要 --force 才能进 bench**；进 bench 后 `auto_plan_enabled=true` 会先调
#   `plan_route_from_config` 并把产物**回写** route.tokens ⇒ 再判时已 ≥2 点 ⇒ 不拦。
_rc_ok, _out_ok, _err_ok = run_bench(CFG_MAIN, "--map", str(FIXTURE_MAP), "--force", seconds="2")
# ★ 只认"含『自动规划路线』且后面跟真实 token 数"的行：
#   本卡第一版用子串匹配 → 抓到的是旁边那条「射击停站无解」警告
#   （同为 ⚠️ 开头、同含"路线"二字）⇒ 假红。改成**提取数字再断言**，更难误判。
import re                                                          # noqa: E402
_m = re.search(r"自动规划路线\s+(\d+)\s+token", _out_ok)
check("5.1 正常路径（--map 自动规划）⇒ **不被本闸拦**（bench 真跑起来）",
      "bench 拒绝启动" not in _out_ok,
      "无拒绝行" if "bench 拒绝启动" not in _out_ok else "被拦了")
check("5.2 它确实走了自动规划并回写（闸门判的是**回写后**的真实 token 数，≥2 ⇒ 不拦）",
      bool(_m) and int(_m.group(1)) >= 2,
      f"自动规划 {_m.group(1)} token" if _m else "(未找到『自动规划路线 N token』行)")
check("5.3 正常路径 rc=0（不是被我改出非零）",
      _rc_ok == 0, f"rc={_rc_ok}")
check("5.4 正常路径也没有 Traceback",
      "Traceback" not in _err_ok, (_err_ok.strip().splitlines() or ["(空)"])[0][:50])

# --------------------------------------------------------------------------- #
section("§5b ★ 闸门触发的**精确边界**（captain 2026-11-01 要求；防后人误读）")
# --------------------------------------------------------------------------- #
# 最易误读处：「配了空 tokens 就会触发 `:2823` 闸门」—— **不是**。
#   `:2808` 的自动规划成功时会 `cfg.override("route.tokens", …)` **覆盖**原始值，
#   而本卡闸门在 `:2840`（更晚）⇒ 读到的是**规划产物**，不一定是用户配置。
_c_missing = Config.load(str(make_cfg(omit_tokens=True, auto_plan=False, tag="b_missing")))
check("5b.1 override 前：删掉该键 ⇒ `in_flat=False`（这是 missing 态的唯一来源）",
      ("route.tokens" in _c_missing._flat) is False,
      f"in_flat={'route.tokens' in _c_missing._flat}")
_c_missing.override("route.tokens", ["A1", "B1"])
check("5b.2 override 后该键**变为存在**（正因如此，missing 态在规划成功路径上不可达）",
      ("route.tokens" in _c_missing._flat) is True
      and _c_missing.get("route.tokens") == ["A1", "B1"],
      f"after override: in_flat=True value={_c_missing.get('route.tokens')}")

# 情形 C（规划成功 ⇒ 覆盖 ⇒ 不触发）由 §5 已证（68 token, rc=0）。
# 情形 D 是 captain 那条「--force 后没触发」的反证：他给了可用地图（C），
#   这里**不给 --map**（默认配置 = 车端布局模式无地图）⇒ 规划失败 ⇒ 不覆盖 ⇒ **仍触发**。
_rc_d, _out_d, _err_d = run_bench(CFG_MAIN, "--force")
check("5b.3 **情形 D**：`--force` + auto_plan=true + **规划失败**（无 --map）⇒ 闸门**照样触发**",
      _rc_d == 2 and "bench 拒绝启动" in _out_d, f"rc={_rc_d}")
check("5b.4 情形 D 的日志里能看出「规划失败」先发生（先因后果可追）",
      "自动规划失败" in _out_d or "按 route.tokens" in _out_d,
      ([ln for ln in _out_d.splitlines() if "自动规划失败" in ln] or ["(无该行)"])[0][:70])
check("5b.5 情形 D 也没有 Traceback（闸门在裸栈之前就接住了）",
      "Traceback" not in _err_d, (_err_d.strip().splitlines() or ["(空)"])[0][:50])

# --------------------------------------------------------------------------- #
section("§6 边界：写域与只读性")
# --------------------------------------------------------------------------- #
import hashlib                                                     # noqa: E402

_main_txt = MAIN_PY.read_text(encoding="utf-8")
check("6.1 `:222` 那条**未被修改**（R001 已拦，本卡不该动它）",
      'route = parse_mission(cfg.get("route.tokens") or [])' in _main_txt,
      "启动门禁里的原调用点仍在")
check("6.2 `:2823` 那条已被收口（不再直接 `parse_mission(… or [])` 无校验）",
      "_route_tokens" in _main_txt and "bench 拒绝启动" in _main_txt)
check("6.3 `mission.py` **零修改**（本卡只读对照）",
      "raise ValueError(f\"{source} 为空，无法载入路线\")" in MISSION_PY.read_text(encoding="utf-8"),
      "对照路径原样保留")

# config/** 零写入：本测试全程只读真源（所有派生配置都写临时目录）。
# 判据用"真源字节在读前读后逐位相同"，不是"我猜它没变"。
check("6.4 `config/robot.yaml` 字节逐位未变（本测试只读真源，派生配置全落临时目录）",
      hashlib.sha256(CFG_MAIN.read_bytes()).hexdigest() == _CFG_SHA_BEFORE,
      f"{_CFG_SHA_BEFORE[:16]}…")

# --------------------------------------------------------------------------- #
finish()
