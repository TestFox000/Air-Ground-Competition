# -*- coding: utf-8 -*-
"""WB-4 多图批量扫描（卡 `WB-4-SWEEP-20261031` · tier-1）。

## 一句话

**一条命令跑完 N 张图**，输出的是**分布**而不是"逐个套件的通过/失败"：

```
PYTHONPATH=src/dzbrain python tools/track_sweep.py --maps ../artifacts/synth_maps/ \
        --arms ideal,realistic --report ../artifacts/synth_maps/sweep.csv
```

## 输出的四件东西

| 产物 | 内容 |
|---|---|
| `--report` CSV | 每张图 × 每档一条行：有解/无解/降级原因/路径冗余比/违规类型 |
| stdout 分布表 | **可行率** + **违规类型分布** + **冗余比分布**（卡面要的就是这三张） |
| `SWEEP_DISTRIBUTION_JSON` | 末行机器可读汇总（UI / 下游卡直接吃） |
| 退出码 | **恒 0**（"扫完了"就是成功；有图无解**不是**本工具的失败 —— 见下） |

## ★ 为什么"有图无解"也必须 rc=0

卡面写得很直白：「多图扫描可能证明大比例地图无解 —— 那是需要的事实，不是坏消息」。
本工具的**判据是"扫完了没"**，不是"规划器赢了没"。把"无解"算成退出码非零，
就等于逼下一个写卡的人去调合成器复杂度、把分布修好看 —— 正是卡面 §残酷结果预案
明令禁止的那件事。所以：`rc=0` = 扫描完成；**分布本身就带结论**（可行率 < 50% 时
本工具会把 `[项] wb-4.1 需要裁决` 打进结论行，而不是自己改判据）。

## 每张图测什么（三层，从便宜到贵）

1. **有解/无解**：`planner.plan_route()` / `plan_route_tracker_safe()` 抛不抛 `PlannerError`；
2. **安全违规类型**：有解时把产物喂 `route.validate.validate_route()`（**生产校验器**），
   按 `R0xx` 编号归类计数 —— 这就是"违规类型分布"；
3. **路径冗余比**：`规划路径长度 / 参考解长度`（参考解 = `map_synth` 造的 `.ref.txt`）。
   `> 1` = 规划器比人工解更长；`< 1` = 规划器比人工解更短（**合法**，人工解不求最优）。

**没有参考解的图**（`--maps` 指向现场既有夹具时）不出冗余比，且**不**把它的无解
计入"规划器无解" —— 卡面负例 3：「生成图缺参考解却被计入规划器无解」判 FAIL。
它的归类是 `NO_REFERENCE`（= 数据不全，不是结论）。

## 两个已知的口径事实（本工具如实分列，不合并）

- **`tour` 与 `tracker-safe` 是两个引擎**：卡面 `--arms` 指的是**噪声档**（ideal/realistic，
  WB-2 的臂），而"哪个规划引擎"是另一个维度。缺省对**两个引擎都扫**，CSV 里带 `engine` 列；
  `--engines tour` 可以只扫一个（tour 是 `config/robot.yaml` 的现役值）。
- **`route_check` 比 `validate_route` 严**：前者多查「航点压过靶板」「航段横穿靶格」两条
  （`validate_route` **没有**这两条）。所以本工具对每个有解产物**同时**跑两者，CSV 里
  分 `validate_errors` 与 `routecheck_rc` 两列 —— 只报一个会把"其实会压靶板"藏起来。

## 边界

- 不碰 `config/**` 与 `src/dzbrain/dzbrain/**`（冻结区，零写入）；本工具**只读调用**。
- 不 commit / 不 push / 不部署、不碰车。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import sys
from collections import Counter, OrderedDict
from dataclasses import dataclass

_THIS = pathlib.Path(__file__).resolve()
_TOOLS = _THIS.parent
_REPO = _TOOLS.parent
_SRC = _REPO / "src" / "dzbrain"

def _norm(entry: str) -> str:
    """`sys.path` 项归一成绝对路径（有的项是调用方刚 `insert` 进去的**相对串** 'tools'）。"""
    import os
    try:
        return os.path.abspath(entry or os.curdir)
    except Exception:                            # noqa: BLE001
        return entry


for _entry in list(sys.path):
    if _norm(_entry) in (str(_TOOLS), str(_REPO)):
        sys.path.remove(_entry)
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
sys.path.append(str(_TOOLS))
# ★ 若 `dzbrain` 已被同名脚本 `tools/dzbrain.py` 遮住并进了 `sys.modules`，
#   光改 sys.path 不够 —— 必须把假包从缓存里摘掉（判据：它的 `__file__` 落在 tools/ 下）。
_pkg = sys.modules.get("dzbrain")
if _pkg is not None and str(_TOOLS) in str(getattr(_pkg, "__file__", "") or ""):
    for _name in [n for n in sys.modules if n == "dzbrain" or n.startswith("dzbrain.")]:
        del sys.modules[_name]

import yaml                                                    # noqa: E402

from dzbrain.core import geometry as geo                       # noqa: E402
from dzbrain.core.config import Config                          # noqa: E402
from dzbrain.route import planner as PL                         # noqa: E402
from dzbrain.route import physics as PH                         # noqa: E402
from dzbrain.route import validate as V                         # noqa: E402
from dzbrain.route.model import Route, Waypoint, make_route     # noqa: E402

RC_OK, RC_USAGE = 0, 2

#: 一条扫描结果的全部列（CSV 头顺序**冻结**，加列只许往后追加）
CSV_COLUMNS = (
    "map", "arm", "engine", "status", "plan_ok", "prod_timeout", "prod_reason", "reason",
    "ref_tokens", "plan_tokens", "redundancy_ratio",
    "validate_errors", "validate_warns", "violation_types",
    "routecheck_rc", "routecheck_errors", "routecheck_types",
    "degraded", "degraded_reason", "plan_m",
)


# --------------------------------------------------------------------------- #
#  输入：地图目录 → [(名字, 地图路径, 参考解路径|None)]
# --------------------------------------------------------------------------- #
def collect_maps(maps_dir: pathlib.Path) -> list[tuple[str, pathlib.Path, pathlib.Path | None]]:
    """`--maps` 目录里的每一张图（`*.yaml`），配对同名 `.ref.txt`（可缺）。

    参考解命名约定 = `map_synth` 的产物：`<name>.yaml` + `<name>.ref.txt`。
    缺参考解的图**不剔除**（现场既有夹具就该能扫），但会被标成 `NO_REFERENCE`。
    """
    out: list[tuple[str, pathlib.Path, pathlib.Path | None]] = []
    for map_path in sorted(maps_dir.glob("*.yaml")):
        ref = map_path.with_suffix(".ref.txt")
        out.append((map_path.stem, map_path, ref if ref.is_file() else None))
    return out


def load_map_cfg(map_path: pathlib.Path) -> dict:
    return yaml.safe_load(map_path.read_text(encoding="utf-8")) or {}


def tokens_of(ref_path: pathlib.Path | None) -> list[str] | None:
    if ref_path is None:
        return None
    text = ref_path.read_text(encoding="utf-8").split()
    return text or None


def route_of(tokens: list[str]) -> Route:
    wps = [Waypoint.at_block(t.rstrip("*").split(",")[0],
                             shoot=t.endswith("*"), token=t) for t in tokens]
    return make_route(wps, name="sweep")


# --------------------------------------------------------------------------- #
#  单个 (图 × 引擎) 的扫描
# --------------------------------------------------------------------------- #
#: 超时后重试用的**宽松预算**（秒）。目的不是"多给点时间让它过"，而是
#: **把"没算完"与"真无解"分开** —— 见 `plan_one()` 的说明。
RETRY_BUDGET_S = 20.0


def plan_one(map_cfg: dict, *, engine: str, cfg: Config | None,
             map_path: pathlib.Path, retry_budget_s: float = RETRY_BUDGET_S
             ) -> tuple[PL.PlanResult | None, str, bool, str]:
    """跑一个规划引擎。返回 `(PlanResult, 失败理由, 是否生产预算内超时)`。

    ## ★★ 为什么超时要**重试**（这是本工具最重要的一处设计，实测逼出来的）

    `planner` 的预算 `planner.max_seconds` 是**墙钟**（`planner.py:3065`
    `deadline = t0 + max(0.05, float(max_seconds))`），不是指令数/展开数。
    ⇒ 同一张图、同一份代码，在**负载**下会时而算完、时而超时。

    实测铁证（2026-11-01，30 张合成图）：

    | 观测 | 结果 |
    |---|---|
    | 生产预算 2.0 s，连跑 4 轮 | 超时数在 1/2/3/4 张之间**逐轮变化**，`solved` 恒 20 |
    | 同一张 `n001`，同一预算跑 6 次 | 判定在 `STRUCT` 与 `TIMEOUT` 之间**翻转** |
    | 预算放宽到 20 s，连跑 3 轮 | `solved=20 structural=10 timeout=0`，**三轮逐位相同** |

    ⇒ "生产预算内超时"**不是这张图的属性，是这台机器当时的属性**。
      把它并进「无解」，可行率就会随机器负载在 56.7%~66.7% 之间飘 ——
      那不是分布，那是把测量噪声当结论（正是卡面负例 3 那类假归因）。

    所以：**主判据 = 可解性**（宽预算下确定性），**另记**生产预算内是否超时。
    ## ★★ 两种失败理由**必须是不同的文本**（captain 2026-11-01 追问，已核实）

    本工具靠**文本前缀**分辨"超时"与"结构未通过"，所以这两条必须能对得上。
    实测原文（2026-11-01，`planner` 生产代码）：

    | 分支 | reason 原文 | 出处 |
    |---|---|---|
    | 超时 | `规划超时（代价洪泛阶段）` / `规划超时（>2.00 s），未生成完整路线` | `planner.py:2259` / `:3383` |
    | 结构 | `规划结果未通过结构自检：
  - 下半场应恰好穿洞 1 次，实际 3 次…` | `planner.py` 结构自检 |

    两者**互不包含**（"超时"二字只出现在超时分支）⇒ 判定可靠。
    ⚠️ 但第一版有个副作用：**重试成功后，原始超时 reason 被丢掉了**，
    于是旧 CSV 里 `prod_timeout=1` 的图看着像"结构失败"，无法从产物验证归因。
    ⇒ 现返回第 4 项 `prod_reason`（**生产预算内的原始理由原文**），CSV 单列保留。

    `prod_timeout=True` 是一个**独立的事实**（"车上 2 s 内规划不出来"），
    它照样要报，但不许污染"这张图有没有解"。
    """
    kwargs = dict(allow_reverse=False, source=str(map_path))

    def _run(budget: float | None) -> tuple[object | None, str]:
        kw = dict(kwargs)
        if budget is not None:
            kw["max_seconds"] = budget
        try:
            if engine == "tour":
                return PL.plan_route(map_cfg, cfg, **kw), ""
            if engine == "tracker-safe":
                tk = {k: v for k, v in kw.items() if k != "allow_reverse"}
                return PL.plan_route_tracker_safe(map_cfg, cfg, **tk), ""
            raise ValueError(f"未知引擎 {engine!r}")
        except PL.PlannerError as exc:
            return None, f"{type(exc).__name__}: {exc}"
        except Exception as exc:                    # noqa: BLE001  引擎内部崩了也要归类
            return None, f"ENGINE_CRASH {type(exc).__name__}: {exc}"

    result, reason = _run(None)                     # ① 生产预算（现场真源）
    if result is not None:
        return result, "", False, ""
    if "超时" not in reason:
        return None, reason, False, reason          # 确定性失败（结构/无解），不重试
    # ② 超时 ⇒ 宽预算重试，只为**分清**"没算完"与"真无解"
    prod_reason = reason                           # ★ 原始超时原文**必须留住**
    retry, why2 = _run(max(float(retry_budget_s), 1.0))
    if retry is not None:
        return retry, "", True, prod_reason         # 有解，但生产预算内没算完
    if "超时" in why2:
        return (None, f"TIMEOUT_BOTH: 生产预算超时且 {retry_budget_s:.0f}s 放宽后仍超时",
                True, prod_reason)
    return None, why2, True, prod_reason             # 放宽后判出确定性失败


def validate_plan(result: PL.PlanResult, map_cfg: dict,
                  cfg: Config | None) -> tuple[int, int, str]:
    """生产校验器 `validate_route()`：`(error 数, warn 数, 违规类型串)`。"""
    issues = V.validate_route(result.route, map_cfg=map_cfg, cfg=cfg)
    errors = [i for i in issues if i.level == "error"]
    warns = [i for i in issues if i.level == "warn"]
    types = "|".join(sorted({i.code for i in errors}))
    return len(errors), len(warns), types


def routecheck_plan(map_path: pathlib.Path, tokens: list[str],
                    cfg_path: pathlib.Path) -> tuple[int, int, str]:
    """`tools/route_check.py` 子进程（**原文件调用，不复制它的规则**）。

    返回 `(rc, ❌行数, 违规类型串)`。类型串从 ❌ 行里抽（route_check 的报文不带 R0xx 编号，
    所以按**检查项名**归类，与 `validate_route` 的 R0xx 并列而不是替代）。
    """
    import os
    import subprocess
    env = dict(os.environ)
    env["ROBOT_MAP"] = str(map_path)
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"
    cmd = [sys.executable, str(_TOOLS / "route_check.py"),
           "--route", " ".join(tokens), "--config", str(cfg_path)]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300,
                              cwd=str(_REPO), env=env, encoding="utf-8", errors="replace")
    except subprocess.TimeoutExpired:
        return 1, 0, "TIMEOUT"
    errs = []
    for line in (proc.stdout or "").splitlines():
        if "❌" in line:
            body = line.split("❌", 1)[1].strip()
            # 去掉 ANSI 与列对齐空白，只留检查项名（"航点压过靶板 1 处" → "航点压过靶板"）
            body = "".join(ch for ch in body if ch.isprintable())
            name = body.split("  ")[0].strip()
            name = name.split(" ")[0] if name else ""
            errs.append(name or "?")
    return proc.returncode, len(errs), "|".join(sorted(set(errs)))


def sweep_one(name: str, map_path: pathlib.Path, ref_path: pathlib.Path | None,
              *, engines: list[str], cfg: Config | None,
              cfg_path: pathlib.Path, do_routecheck: bool) -> list[dict]:
    """一张图 × 每个引擎一条行（`arm` 列由调用方回填 —— 规划与噪声档无关）。"""
    map_cfg = load_map_cfg(map_path)
    ref_tokens = tokens_of(ref_path)
    ref_route = route_of(ref_tokens) if ref_tokens else None
    ref_m = ref_route.path_length() if ref_route else None
    rows: list[dict] = []
    for engine in engines:
        result, reason, prod_timeout, prod_reason = plan_one(
            map_cfg, engine=engine, cfg=cfg, map_path=map_path)
        row = {k: "" for k in CSV_COLUMNS}
        row.update(map=name, engine=engine, plan_ok=0, prod_timeout=int(bool(prod_timeout)),
                   prod_reason=prod_reason,
                   ref_tokens=(len(ref_tokens) if ref_tokens else ""))
        if result is None:
            # ★★ 三类**必须分开**，合并任何两个都是假归因：
            #   · `NO_REFERENCE` = 这张图**没有参考解** ⇒ 根本判不了（卡面负例 3）；
            #   · `TIMEOUT`     = 规划器**没算完**（撞 `planner.max_seconds` 墙钟预算）
            #                     ⇒ **不是无解**！证据：n001 同一张图同一预算跑 6 次，
            #                     结果在 STRUCT / TIMEOUT 之间**翻转**（见 §八 更正记录）；
            #                     而把预算放到 20 s 后它稳定判 STRUCT。墙钟预算对负载敏感，
            #                     这是 `planner.py:250-258` 自己就记着的已知 flake
            #                     （"4 忙核下稳定超时、单跑 5/5 绿"）。
            #   · `NO_SOLUTION` = 规划器**真的判了无解**（结构自检等确定性失败）。
            #   本工具第一版把 TIMEOUT 并进 NO_SOLUTION，于是同一份代码跑两遍
            #   可行率会在 56.7%~66.7% 之间飘 —— 那不是"分布"，那是测出来的噪声。
            if ref_tokens is None:
                row["status"] = "NO_REFERENCE"
            else:
                row["status"] = "NO_SOLUTION"
            row["reason"] = reason
            rows.append(row)
            continue
        n_err, n_warn, vtypes = validate_plan(result, map_cfg, cfg)
        # ★★ 没有参考解 ⇒ **不知道这张图本来可不可行** ⇒ 规划器"有解"也**不能**
        #   计入可行率分母（那会把"数据不全"当成"规划器赢了"，与负例 3 同一类假归因）。
        #   实测来源：本文件的 §3 自己先红了 —— 第一次写的时候没参考解的图仍然
        #   报 SOLVED、可行率算出 100%，而那个 100% 的分子里有一张**根本没法判**的图。
        #   ⇒ 无参考解时状态一律 `NO_REFERENCE`；`plan_ok` 另存一列如实记录规划结果。
        row.update(status=("SOLVED" if ref_tokens is not None else "NO_REFERENCE"),
                   plan_ok=1, reason="",
                   plan_tokens=len(result.tokens),
                   plan_m=round(result.polyline_m, 4),
                   validate_errors=n_err, validate_warns=n_warn,
                   violation_types=vtypes,
                   degraded=int(bool(result.degraded)),
                   degraded_reason=result.degraded_reason if result.degraded else "")
        if ref_m and ref_m > 0:
            row["redundancy_ratio"] = round(result.polyline_m / ref_m, 4)
        if do_routecheck:
            rc, rc_errs, rc_types = routecheck_plan(map_path, result.tokens, cfg_path)
            row.update(routecheck_rc=rc, routecheck_errors=rc_errs,
                       routecheck_types=rc_types)
        rows.append(row)
    return rows


# --------------------------------------------------------------------------- #
#  分布统计（卡面要的三张表）
# --------------------------------------------------------------------------- #
def _bar(value: float, width: int = 24) -> str:
    filled = int(round(value * width))
    return "█" * filled + "·" * (width - filled)


def distribution(rows: list[dict]) -> dict:
    """按 (engine) 汇总：可行率 / 违规类型 / 冗余比。**不用逐套件通过率**。"""
    by_engine: dict[str, dict] = {}
    for row in rows:
        eng = row["engine"]
        slot = by_engine.setdefault(eng, {
            "total": 0, "solved": 0, "no_solution": 0, "no_reference": 0,
            "timeout": 0,
            "prod_timeout": 0,
            "had_prod_timeout": False,
            "degraded": 0, "validate_errors": 0, "routecheck_red": 0,
            "violation_types": Counter(), "routecheck_types": Counter(),
            "redundancy": [], "no_solution_reasons": Counter(),
        })
        slot["total"] += 1
        status = row["status"]
        if row["prod_timeout"] in (1, "1", True):
            slot["prod_timeout"] += 1
            slot["had_prod_timeout"] = True
        if status == "SOLVED":
            slot["solved"] += 1
            if row["validate_errors"]:
                slot["validate_errors"] += 1
            for t in str(row["violation_types"]).split("|"):
                if t:
                    slot["violation_types"][t] += 1
            if row["routecheck_rc"] not in ("", None, 0):
                slot["routecheck_red"] += 1
            for t in str(row["routecheck_types"]).split("|"):
                if t:
                    slot["routecheck_types"][t] += 1
            if row["redundancy_ratio"] not in ("", None):
                slot["redundancy"].append(float(row["redundancy_ratio"]))
            if row["degraded"] in (1, "1", True):
                slot["degraded"] += 1
        elif status == "NO_SOLUTION":
            slot["no_solution"] += 1
            head = str(row["reason"]).split(":", 1)[0]
            slot["no_solution_reasons"][head] += 1
        elif status == "NO_REFERENCE":
            slot["no_reference"] += 1

    out = {}
    for eng, slot in by_engine.items():
        # 分母 = 有解 + 真·无解。**不含** NO_REFERENCE（判不了）与 TIMEOUT（没算完）。
        judged = slot["solved"] + slot["no_solution"]
        red = slot["redundancy"]
        out[eng] = {
            "total": slot["total"],
            "solved": slot["solved"],
            "no_solution": slot["no_solution"],
            "no_reference": slot["no_reference"],
            "timeout": slot["prod_timeout"],
            "prod_timeout": slot["prod_timeout"],
            "had_prod_timeout": bool(slot["had_prod_timeout"]),
            "feasible_rate": (slot["solved"] / judged) if judged else None,
            "degraded": slot["degraded"],
            "validate_error_maps": slot["validate_errors"],
            "routecheck_red_maps": slot["routecheck_red"],
            "violation_types": dict(sorted(slot["violation_types"].items())),
            "routecheck_types": dict(sorted(slot["routecheck_types"].items())),
            "no_solution_reasons": dict(sorted(slot["no_solution_reasons"].items())),
            "redundancy": {
                "n": len(red),
                "min": round(min(red), 4) if red else None,
                "median": round(sorted(red)[len(red) // 2], 4) if red else None,
                "max": round(max(red), 4) if red else None,
                "gt1": sum(1 for x in red if x > 1.0),
                "le1": sum(1 for x in red if x <= 1.0),
            },
        }
    return out


def render_distribution(dist: dict) -> str:
    lines: list[str] = []
    lines.append("=" * 78)
    lines.append("  多图扫描分布（判据 = 分布，不是逐套件通过率）")
    lines.append("=" * 78)
    for eng, d in dist.items():
        rate = d["feasible_rate"]
        rate_txt = f"{rate * 100:.1f}%" if rate is not None else "n/a（无可判图）"
        lines.append(f"\n[{eng}] 图 {d['total']} 张 · 有解 {d['solved']} · "
                     f"无解 {d['no_solution']} · 超时 {d['timeout']} · "
                     f"缺参考解 {d['no_reference']}")
        lines.append(f"  可行率 {rate_txt}  {_bar(rate or 0.0)}")
        if d["prod_timeout"]:
            lines.append(
                f"  ⚠️ 生产预算内超时 {d['prod_timeout']} 张 —— **这不是可解性判据，是负载度量**。"
                f"同代码同图在负载下该值在 1~4 张之间浮动（宽预算重试后仍超时的 = 0 张，"
                f"即这 {d['prod_timeout']} 张全部有解）。归因原文见 CSV `prod_reason` 列，"
                f"**不计入无解**、**不进可行率分母**。")
        elif d.get("had_prod_timeout"):
            lines.append("  （本轮 0 张生产预算内超时；该量是**负载函数**，不要拿它判可解性）")
        lines.append(f"  降级解 {d['degraded']} 张 · 有 validate error {d['validate_error_maps']} 张"
                     f" · 有 route_check ❌ {d['routecheck_red_maps']} 张")
        vt = d["violation_types"]
        lines.append("  违规类型分布（validate R0xx）: "
                     + ("; ".join(f"{k}×{v}" for k, v in vt.items()) if vt else "无"))
        rt = d["routecheck_types"]
        lines.append("  违规类型分布（route_check ❌）: "
                     + ("; ".join(f"{k}×{v}" for k, v in rt.items()) if rt else "无"))
        rs = d["redundancy"]
        if rs["n"]:
            lines.append(f"  冗余比（规划/参考解）n={rs['n']} "
                         f"min={rs['min']} median={rs['median']} max={rs['max']} "
                         f">1 的 {rs['gt1']} 张 / ≤1 的 {rs['le1']} 张")
        else:
            lines.append("  冗余比：n/a（没有参考解）")
        ns = d["no_solution_reasons"]
        if ns:
            lines.append("  无解归因: " + "; ".join(f"{k}×{v}" for k, v in ns.items()))
    return "\n".join(lines)


def verdict(dist: dict, min_feasible: float) -> str:
    """结论行。**只陈述事实 + 指向 wb-4.1**，不改判据、不调合成器。"""
    parts: list[str] = []
    for eng, d in dist.items():
        rate = d["feasible_rate"]
        if rate is None:
            parts.append(f"{eng}: 无可判图")
            continue
        head = f"{eng} 可行率 {rate * 100:.1f}%"
        # ★ 三个区间都要有结论。卡面原预案只覆盖 <50% 与 >90%，**中间那段是漏洞**；
        #   captain 2026-11-01 裁决：「56.7% 更要立刻记账」⇒ 50%~90% 也必须指向 wb-4.1。
        if rate < min_feasible:
            head += f" < {min_feasible * 100:.0f}% ⇒ **wb-4.1 必须开**"
        elif rate < 0.90:
            head += f" 落在 {min_feasible * 100:.0f}%~90% ⇒ **wb-4.1 必须开**（近半合法图给不出解）"
        elif d["violation_types"]:
            head += " ≥90% 但违规类型集中 ⇒ 按类型立卡"
        else:
            head += " ≥90% 且违规不集中 ⇒ 在阈值内"
        if d["timeout"]:
            head += (f"；另 {d['timeout']} 张**仅在生产预算内超时**"
                     f"（= 负载度量，非可解性判据；宽预算下全部有解，不计入分母）")
        parts.append(head)
    return "结论: " + " ｜ ".join(parts)




# =========================================================================== #
#  扫掠判据层（S001/S002/S003）—— 「点判据 → 扫掠外包判据」一等公民
# =========================================================================== #
# ★ 这一层**原先**是独立文件 `tools/wb_sweep.py`。它被并进本文件，理由有两条：
#   ① 卡面写域只给了 `tools/map_synth.py` 与 `tools/track_sweep.py` 两个新工具，
#      "另开第三个 .py" 就是写域越界（`tools/wf/check_reuse.py` 当场把
#      `wb_sweep.py` 判成"新增未登记" RED —— 门禁自己抓出来的，不是靠人眼看）；
#   ② 扫掠判据**只服务于扫描**：`map_synth` 不需要它（合成器只产图与参考解），
#      测试从本模块 import 即可。放一个地方 = 不会有第二份。

#: 判据编号（稳定，测试与报告引用；**不许改号**）
SWEEP_BOUNDS = "S001"   # 越界（车体矩形外包出界）
SWEEP_PIVOT = "S002"    # 掉头点（转向顶点的车身外摆）
SWEEP_STOP = "S003"     # 停站格（停站时整车压实体）

#: 三档判据的默认开关（卡面：**可选一等公民** ⇒ 能单独开关、能组合）
SWEEP_KINDS = (SWEEP_BOUNDS, SWEEP_PIVOT, SWEEP_STOP)


@dataclass(frozen=True)
class SweepConfig:
    """扫掠判据的物理量（**全部**来自 config / physics，不新造）。"""

    length_m: float = 0.28
    width_m: float = 0.17
    offset_m: float = 0.0
    barrier_width_m: float = 0.02
    obstacle_radius_m: float = 0.1275

    @classmethod
    def of(cls, cfg) -> "SweepConfig":
        def g(key: str, fallback: float) -> float:
            try:
                v = cfg.get(key) if cfg is not None else None
            except Exception:                        # noqa: BLE001  配置缺失不该让判据崩
                return fallback
            return fallback if v is None else float(v)

        return cls(
            length_m=g("vehicle.length_m", 0.28),
            width_m=g("vehicle.width_m", 0.17),
            offset_m=g("vehicle.ref_offset_m", 0.0),
            barrier_width_m=g("field.barrier_width_m", 0.02),
            obstacle_radius_m=g("field.obstacle_radius_m", 0.1275),
        )

    @property
    def half_diag(self) -> float:
        """车体半对角线 = 转向时的**扫掠半径**（`physics.vehicle_half_diagonal_m`）。"""
        return PH.vehicle_half_diagonal_m(self.length_m, self.width_m)

    def body(self, x: float, y: float, yaw: float) -> tuple:
        """车体矩形 4 角（**真源** `physics.body_corners`，含参考点偏移）。"""
        return PH.body_corners(x, y, yaw, length_m=self.length_m,
                               width_m=self.width_m, offset_m=self.offset_m)


@dataclass(frozen=True)
class SweepHit:
    """一次扫掠违规（`kind` = S001/S002/S003）。"""

    kind: str
    index: int          # 航点索引（发生违规的那一段/那个点）
    block: str
    detail: str

    def __str__(self) -> str:
        return f"[{self.kind}] 航点#{self.index} {self.block}: {self.detail}"


# --------------------------------------------------------------------------- #
#  S001 · 越界（车体矩形外包 vs 场界）
# --------------------------------------------------------------------------- #
def sweep_bounds(route, *, sc: SweepConfig, margin_m: float = None) -> list[SweepHit]:
    """整条路线的车体矩形**外包**与场界比。

    ★ 点判据（`route_check` 的 `oob`）问的是"**航点坐标**在不在界内"；
      本判据问的是"**车身**在不在界内"。两者差一个 `车体半长/半宽`，
      而现役 `MAP_SAFETY_MARGIN = 0.60 m` 把这点差全吃掉了 —— 所以在现役图上
      本判据**不触发**（下面 `--selfcheck` 的 `S001 不触发` 就是这条）。
      它会在"贴近边界的停站/转向"上触发：车心离界 0.10 m 时，车头方向一转就出去。

    ★★ 边界口径（实测踩过，必须写清楚）：`geo.map_bounds()` 返回的是**最外格的中心线**
      （`MAP_X_MIN=0.175` 就是第 11 行的行心），**不是**场地物理边缘。场地是
      `11 × 14` 个 0.35 m 的格 ⇒ 物理边缘 = 中心线 **再外扩半格**（0.175）。
      第一版直接拿 `map_bounds(0)` 比车体角点，在场地图上当场报 16 处"越界"
      —— 全是 N 列/第 11 行这些**最外圈**的合法航点（车身本来就该盖到格边）。
      那是判据的边界定义错了，不是路线错了。
      ⇒ 缺省 `margin_m = BLOCK_SIZE/2`（= 物理边缘）；要判"车心离边界留多少"
        请显式传 `margin_m`（负值 = 比物理边缘更严，用于留安全余量）。

    :param margin_m: 场界外扩（缺省 `BLOCK_SIZE/2` = 物理边缘）。
    """
    if margin_m is None:
        margin_m = geo.BLOCK_SIZE / 2.0
    xmin, xmax, ymin, ymax = geo.map_bounds(margin_m)
    hits: list[SweepHit] = []
    for i, wp in enumerate(route.waypoints):
        x0, y0 = float(wp.x), float(wp.y)
        # 到下一个航点的行进朝向（末点用自身 yaw），与 Waypoint.yaw 的定义一致
        yaw = float(route.waypoints[i + 1].heading_to(wp) if i + 1 < len(route.waypoints)
                    else wp.yaw)
        # heading_to 的方向是"下一点→本点"？统一用离开段方向（与 build 时一致）
        yaw = float(wp.yaw)
        corners = sc.body(x0, y0, yaw)
        out = [(px, py) for px, py in corners
               if not (xmin <= px <= xmax and ymin <= py <= ymax)]
        if out:
            worst = max(max(xmin - px, px - xmax, ymin - py, py - ymax)
                        for px, py in out)
            hits.append(SweepHit(SWEEP_BOUNDS, i, wp.block,
                                 f"车身角点出界 {len(out)}/4，最远越界 {worst * 1000:.1f} mm"
                                 f"（边界 x[{xmin:.3f},{xmax:.3f}] y[{ymin:.3f},{ymax:.3f}]）"))
    return hits


# --------------------------------------------------------------------------- #
#  S002 · 掉头点（转向顶点的车身外摆）
# --------------------------------------------------------------------------- #
def sweep_pivot(route, *, sc: SweepConfig, map_cfg: dict | None = None) -> list[SweepHit]:
    """每个**转向顶点**的车身外摆（半径 = 半对角线）与"实体"比。

    实体 = 障碍柱面 + 靶板矩形 + 挡板细条 + 场界 —— 与 `route_check.TrajectoryFieldGate.inspect`
    **同一组对象**，只是把"该点的车体矩形"换成"该顶点车身扫过的外包圆"。

    ★ 与点判据的差别：`route_check` 的"拐角净空"用的是**该顶点处的车体矩形**；
      本判据用**外摆圆**（半对角线）—— 二者在 90° 转向处差 0.0788 m
      （`physics.required_target_body_clearance_m` 的 docstring 里就有这个数）。
      **95° 以上的掉头**（`PIVOT`）差得更多：车头从指向 A 转到指向 B，
      车身扫过的是一个近乎半圆的扇面，比"两个端点矩形"大得多。
    """
    spec = _spec_of(map_cfg) if map_cfg else None
    hits: list[SweepHit] = []
    wps = list(route.waypoints)
    if len(wps) < 3:
        return hits
    for i in range(1, len(wps) - 1):
        w0, w1, w2 = wps[i - 1], wps[i], wps[i + 1]
        deg = _turn_deg(w0, w1, w2)
        if deg <= 10.0:                       # 基本直线 ⇒ 不会外摆（与 route_check 同门限）
            continue
        x, y = float(w1.x), float(w1.y)
        radius = sc.half_diag
        # ---- 障碍（柱面）：外摆圆到柱心 ----
        if spec:
            for ob in spec.obstacles:
                ox, oy = geo.block_to_meters(ob)
                need = radius + sc.obstacle_radius_m
                d = math.hypot(x - ox, y - oy)
                if d < need - 1e-9:
                    hits.append(SweepHit(
                        SWEEP_PIVOT, i, w1.block,
                        f"{deg:.0f}° 掉头外摆半径 {radius:.4f} m 吃进障碍 {ob}"
                        f"（车心距柱心 {d:.4f} < 需 {need:.4f} m，差 {(need - d) * 1000:.1f} mm）"))
            # ---- 靶板（跨格矩形）：外摆圆到矩形 ----
            for tgt in spec.targets:
                tx, ty = geo.block_to_meters(tgt)
                d = PH.point_rect_distance_m(x, y, tx, ty, geo.BLOCK_SIZE / 2.0)
                if d < radius - 1e-9:
                    hits.append(SweepHit(
                        SWEEP_PIVOT, i, w1.block,
                        f"{deg:.0f}° 掉头外摆半径 {radius:.4f} m 吃进靶板 {tgt}"
                        f"（车心到板 {d:.4f} < {radius:.4f} m，差 {(radius - d) * 1000:.1f} mm）"))
        # ---- 挡板（细条）：外摆圆到中心线 ----
        if spec:
            bx, by0, by1 = PH.barrier_centerline(
                col_from=str(spec.barrier_col_from), col_to="N",
                row_hi=int(spec.barrier_row_hi), row_lo=int(spec.barrier_row_lo))
            d = PH.barrier_distance_point_m(x, y, x=bx, y0=by0, y1=by1)
            need = radius + sc.barrier_width_m / 2.0
            if d < need - 1e-9 and not _is_bridge_row(w1, spec):
                hits.append(SweepHit(
                    SWEEP_PIVOT, i, w1.block,
                    f"{deg:.0f}° 掉头外摆半径 {radius:.4f} m 吃进挡板"
                    f"（车心距中心线 {d:.4f} < {need:.4f} m，差 {(need - d) * 1000:.1f} mm）"))
    return hits


def _is_bridge_row(wp, spec) -> bool:
    """该航点是不是桥洞格（桥洞处挡板**有洞**，按 `physics` 的豁免口径不判）。"""
    return str(wp.block).strip().upper() in {str(b).strip().upper() for b in spec.bridges}


def _turn_deg(w0, w1, w2) -> float:
    """三点折返角（度）；与 `planner.turn_angle_deg` 同式（按格坐标，不按米制）。"""
    d1 = (float(w1.x) - float(w0.x), float(w1.y) - float(w0.y))
    d2 = (float(w2.x) - float(w1.x), float(w2.y) - float(w1.y))
    n1, n2 = math.hypot(*d1), math.hypot(*d2)
    if n1 < 1e-9 or n2 < 1e-9:
        return 0.0
    cosv = max(-1.0, min(1.0, (d1[0] * d2[0] + d1[1] * d2[1]) / (n1 * n2)))
    return math.degrees(math.acos(cosv))


# --------------------------------------------------------------------------- #
#  S003 · 停站格（停站时整车压实体）—— 复刻 A1 现场事故
# --------------------------------------------------------------------------- #
def sweep_stop(route, *, sc: SweepConfig, map_cfg: dict | None = None,
               min_clearance_m: float = 0.0) -> list[SweepHit]:
    """每个**停站格**（`shoot` / `scan`）上，整车矩形与靶板/障碍/挡板的最近距离。

    ★ A1 现场事故的形态（卡面点名要复刻）：停站格到**板心** 0.90±0.12 m 窗口内
      ⇒ 点判据「射击点距离」全绿；但 0.28×0.17 的车身矩形与 0.35×0.35 的靶板
      矩形最近距离 = **0.0 m**（贴着）；再差 0.7 mm 就是"看着有余量、实际贴上"。

    :param min_clearance_m: 要求的最小间隙（缺省 0.0 = "不许贴上"；
        墙上要留余量时传正数）。
    """
    spec = _spec_of(map_cfg) if map_cfg else None
    hits: list[SweepHit] = []
    if spec is None:
        return hits
    for i, wp in enumerate(route.waypoints):
        if not getattr(wp, "is_stop_point", False):
            continue
        x, y, yaw = float(wp.x), float(wp.y), float(wp.yaw)
        # 靶板
        for tgt in spec.targets:
            tx, ty = geo.block_to_meters(tgt)
            d = PH.body_rect_clearance_m(x, y, yaw, rect_cx=tx, rect_cy=ty,
                                         rect_half_x=geo.BLOCK_SIZE / 2.0,
                                         rect_half_y=geo.BLOCK_SIZE / 2.0,
                                         length_m=sc.length_m, width_m=sc.width_m,
                                         offset_m=sc.offset_m)
            if d <= min_clearance_m + 1e-9:
                hits.append(SweepHit(
                    SWEEP_STOP, i, wp.block,
                    f"停站整车压靶板 {tgt}：车体矩形到板 {d * 1000:.2f} mm"
                    f"（{'贴上' if d <= 1e-9 else '余量不足'}）"))
        # 障碍
        for ob in spec.obstacles:
            ox, oy = geo.block_to_meters(ob)
            d = PH.body_obstacle_clearance_m(x, y, yaw, ox, oy,
                                             obstacle_radius_m=sc.obstacle_radius_m,
                                             length_m=sc.length_m, width_m=sc.width_m,
                                             offset_m=sc.offset_m)
            if d <= min_clearance_m + 1e-9:
                hits.append(SweepHit(
                    SWEEP_STOP, i, wp.block,
                    f"停站整车贴障碍 {ob}：车体到柱面 {d * 1000:.2f} mm"))
        # 挡板
        bx, by0, by1 = PH.barrier_centerline(
            col_from=str(spec.barrier_col_from), col_to="N",
            row_hi=int(spec.barrier_row_hi), row_lo=int(spec.barrier_row_lo))
        d = PH.body_barrier_clearance_m(x, y, yaw, barrier_width_m=sc.barrier_width_m,
                                        barrier_x=bx, barrier_y0=by0, barrier_y1=by1,
                                        length_m=sc.length_m, width_m=sc.width_m,
                                        offset_m=sc.offset_m)
        if d <= min_clearance_m + 1e-9 and not _is_bridge_row(wp, spec):
            hits.append(SweepHit(
                SWEEP_STOP, i, wp.block, f"停站整车贴挡板：车体到板面 {d * 1000:.2f} mm"))
    return hits


def _spec_of(map_cfg):
    """`map_cfg`（`{'map': {...}}`）-> `planner.MapSpec`（**复用**生产解析器）。"""
    from dzbrain.route import planner as PL
    try:
        return PL.load_map(map_cfg)
    except Exception:                            # noqa: BLE001  地图坏了由上游报，这里不吞
        return None


# --------------------------------------------------------------------------- #
#  统一入口（三判据可选组合 —— 卡面「可选一等公民」）
# --------------------------------------------------------------------------- #
def sweep_route(route, *, kinds=SWEEP_KINDS, sc: SweepConfig | None = None,
                map_cfg: dict | None = None, margin_m: float | None = None,
                stop_min_clearance_m: float = 0.0) -> list[SweepHit]:
    """按 `kinds` 组合跑扫掠判据。返回**全部**命中（`[]` = 扫掠层通过）。"""
    sc = sc or SweepConfig()
    hits: list[SweepHit] = []
    if SWEEP_BOUNDS in kinds:
        hits += sweep_bounds(route, sc=sc, margin_m=margin_m)
    if SWEEP_PIVOT in kinds:
        hits += sweep_pivot(route, sc=sc, map_cfg=map_cfg)
    if SWEEP_STOP in kinds:
        hits += sweep_stop(route, sc=sc, map_cfg=map_cfg,
                           min_clearance_m=stop_min_clearance_m)
    return hits


def summarize(hits) -> str:
    """人类可读一行（供报告/CSV）。"""
    if not hits:
        return "扫掠层通过（S001/S002/S003 无命中）"
    by_kind: dict[str, int] = {}
    for h in hits:
        by_kind[h.kind] = by_kind.get(h.kind, 0) + 1
    return "扫掠层命中 " + "; ".join(f"{k}×{v}" for k, v in sorted(by_kind.items()))


# --------------------------------------------------------------------------- #
#  主流程
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="WB-4 多图批量扫描（N 图 × 引擎 → CSV + 分布表）")
    ap.add_argument("--maps", required=True, help="地图目录（含 *.yaml，可选同名 *.ref.txt）")
    ap.add_argument("--arms", default="ideal,realistic",
                    help="噪声档（WB-2 的臂名，仅用于 CSV 标注与分组；规划本身与噪声无关）")
    ap.add_argument("--engines", default="tour,tracker-safe",
                    help="规划引擎（tour / tracker-safe，逗号分隔）")
    ap.add_argument("--report", default=None, help="CSV 落盘路径（缺省不落盘）")
    ap.add_argument("--config", default=None, help="配置中心（缺省 config/robot.yaml）")
    ap.add_argument("--min-feasible", type=float, default=0.50,
                    help="可行率裁决线（卡面：<50% 直接开 [项] 卡，默认 0.50）")
    ap.add_argument("--no-routecheck", action="store_true",
                    help="跳过 tools/route_check.py 子进程（只跑 validate_route，快很多）")
    ap.add_argument("--limit", type=int, default=0, help="只扫前 N 张（0 = 全部，调试用）")
    ap.add_argument("--json", action="store_true", help="末行输出 SWEEP_DISTRIBUTION_JSON")
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    maps_dir = pathlib.Path(args.maps)
    if not maps_dir.is_dir():
        print(f"❌ --maps 不是目录: {maps_dir}", file=sys.stderr)
        return RC_USAGE
    cfg_path = pathlib.Path(args.config) if args.config else (_REPO / "config" / "robot.yaml")
    cfg = Config.load(str(cfg_path)) if cfg_path.is_file() else None
    engines = [e.strip() for e in str(args.engines).split(",") if e.strip()]
    arms = [a.strip() for a in str(args.arms).split(",") if a.strip()] or ["(none)"]

    pairs = collect_maps(maps_dir)
    if args.limit:
        pairs = pairs[: args.limit]
    if not pairs:
        print(f"❌ {maps_dir} 里没有 *.yaml", file=sys.stderr)
        return RC_USAGE
    print(f"扫描 {len(pairs)} 张图 × 引擎 {engines} × 档 {arms}"
          f"（route_check {'OFF' if args.no_routecheck else 'ON'}）")

    base_rows: list[dict] = []
    for name, map_path, ref_path in pairs:
        rows = sweep_one(name, map_path, ref_path, engines=engines, cfg=cfg,
                         cfg_path=cfg_path, do_routecheck=not args.no_routecheck)
        base_rows += rows
        brief = " · ".join(f"{r['engine']}={r['status']}" for r in rows)
        print(f"  {name}: {brief}")

    # `--arms` 维度：规划与噪声档**无关**（这是事实，不是简化 —— 噪声只在闭环仿真里起作用）。
    # 所以 CSV 里每张图 × 引擎 × 档各一行，档只影响 `arm` 列；闭环侧的横向误差分布由
    # `--arm` 喂给 sim_rig 的那条线负责（本工具不重复跑 110 s 闭环，那会让扫描从 30 s 变 2 天）。
    rows: list[dict] = []
    for row in base_rows:
        for arm in arms:
            copy = dict(row)
            copy["arm"] = arm
            rows.append(copy)

    dist = distribution(base_rows)        # 分布按**去档**统计（否则每个数会翻倍且无意义）
    print()
    print(render_distribution(dist))
    print()
    line = verdict(dist, args.min_feasible)
    print(line)

    if args.report:
        report = pathlib.Path(args.report)
        report.parent.mkdir(parents=True, exist_ok=True)
        with report.open("w", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(CSV_COLUMNS))
            writer.writeheader()
            for row in rows:
                writer.writerow({k: row.get(k, "") for k in CSV_COLUMNS})
        print(f"\nCSV → {report}（{len(rows)} 行）")

    if args.json:
        payload = OrderedDict(schema="dzrobot-sweep-distribution/1",
                              maps=len(pairs), engines=engines, arms=arms,
                              distribution=dist, verdict=line)
        print("SWEEP_DISTRIBUTION_JSON "
              + json.dumps(payload, ensure_ascii=False, sort_keys=True))
    # ★ 恒 0：扫完了就是成功（"有图无解"是事实，不是本工具的失败）
    return RC_OK


if __name__ == "__main__":
    sys.exit(main())
