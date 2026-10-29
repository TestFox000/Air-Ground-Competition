# -*- coding: utf-8 -*-
r"""t2：标定参数回填工具链（`tools/calib_backfill.py`）的自检与判定对拍。

覆盖（对应任务卡 t2 的验收）：
  1. 映射表自洽：参数 → `config/robot.yaml` 键（Config 可读）+ YAML 行锚点**唯一**；
  2. `--selfcheck` 干净（error 0）且每一条断言钉的「文件:行 + 原文子串」仍然命中（行没漂）；
  3. 覆盖缺口**显式登记**：哪个键没有断言钉住（回填后测试不会红），必须报出来；
  4. 判定逻辑对拍（R3 纵向 / yaw_trim、R4 转圈四档、轴距、轮距几何）；
  5. **写盘安全**：改配置/重钉都先校验后写、先备份，干跑绝不碰真文件；STALE 直接拒绝；
  6. `--template` 与空模板干跑：全 PENDING/MANUAL、exit 0（等实测数，不写任何文件）。

跑法：`$env:PYTHONPATH='…\\src\\dzbrain'; python src\\dzbrain\\tests\\test_calib_backfill.py`
全过 exit 0；任一失败 exit 1。**本文件只读仓库、只写临时目录**。
"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import os
import pathlib
import shutil
import sys
import tempfile

WS = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS / "src" / "dzbrain"))

try:
    sys.stdout.reconfigure(errors="replace")  # type: ignore[union-attr]
except Exception:                              # pragma: no cover
    pass

import yaml  # noqa: E402

from dzbrain.core.config import Config  # noqa: E402

TOOL_PATH = pathlib.Path(
    os.environ.get("CALIB_TOOL_PATH") or (WS / "tools" / "calib_backfill.py"))
#: 被测工具的**非源码**配套（模板）始终取仓库真件：即使为了负向对照换了工具副本，
#: "模板与 build_template() 同源"这条判据仍然对拍真仓库（换掉的是被检查的实现，不是真源）。
TEMPLATE_PATH = WS / "tools" / "calib_backfill_template.yaml"
REAL_CFG = WS / "config" / "robot.yaml"


def _load_tool():
    spec = importlib.util.spec_from_file_location("calib_backfill_under_test", TOOL_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)          # type: ignore[union-attr]
    return mod


CB = _load_tool()

_PASS = 0
_FAIL = 0
_FAILED: list[str] = []


def ok(name: str, cond: bool, detail: str = "") -> None:
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print(f"PASS  {name}" + (f"   [{detail}]" if detail else ""))
    else:
        _FAIL += 1
        _FAILED.append(name)
        print(f"FAIL  {name}   [{detail}]")


def sec(title: str) -> None:
    print()
    print(f"--- {title} ---")


def close(name: str, got: float, want: float, tol: float = 1e-9) -> None:
    ok(name, abs(float(got) - float(want)) <= tol, f"got={got!r} want={want!r} tol={tol}")


def _raised(fn, *a, **kw) -> tuple[bool, str]:
    try:
        fn(*a, **kw)
    except (Exception, SystemExit) as exc:  # SystemExit 不是 Exception 的子类（argparse --help 走它）
        return True, f"{type(exc).__name__}: {exc}"
    return False, ""


# =========================================================================== #
#  0. 前置事实
# =========================================================================== #
print("=" * 78)
print("t2 标定参数回填工具链 —— 自检与判定对拍")
print("=" * 78)
print(f"工具      : {TOOL_PATH}  ({TOOL_PATH.stat().st_size} bytes)")
print(f"模板      : {TEMPLATE_PATH}  ({TEMPLATE_PATH.stat().st_size} bytes)")
print(f"真配置    : {REAL_CFG}  ({REAL_CFG.stat().st_size} bytes)")
_REAL_CFG_BEFORE = REAL_CFG.read_bytes()
_TMP = pathlib.Path(tempfile.mkdtemp(prefix="t2_calib_backfill_"))
print(f"临时目录  : {_TMP}")

sec("1. 映射表自洽：键可读 + YAML 行锚点唯一")
cfg = Config.load(str(REAL_CFG))
yaml_text = REAL_CFG.read_text(encoding="utf-8")
ids = [c["id"] for c in CB.CHANNELS]
ok("1.1 通道 id 唯一", len(ids) == len(set(ids)), f"{ids}")

for spec in CB.CHANNELS:
    cid = spec["id"]
    if spec.get("yaml_key"):
        val = cfg.get(spec["yaml_key"])
        ok(f"1.2 [{cid}] {spec['yaml_key']} 可读且为数值",
           isinstance(val, (int, float)) and not isinstance(val, bool), f"{val!r}")
        hit, msg = _raised(CB.locate_yaml_line, yaml_text, spec["yaml_leaf"])
        ok(f"1.3 [{cid}] YAML 叶子键 {spec['yaml_leaf']} 唯一命中", not hit, msg)
        if not hit:
            lineno, _i, old, _t = CB.locate_yaml_line(yaml_text, spec["yaml_leaf"])
            ok(f"1.4 [{cid}] 行锚点与 Config 读数一致",
               abs(float(old) - float(val)) < 1e-9, f"yaml:{lineno} {old!r} vs config {val!r}")
    else:
        ok(f"1.2 [{cid}] 无 yaml 键（几何输入，只进判据）", spec.get("yaml_key") is None)
    ok(f"1.5 [{cid}] 输入声明非空" if spec["inputs"] or spec.get("manual") else f"1.5 [{cid}] 非 manual 必须声明输入",
       bool(spec["inputs"]) or bool(spec.get("manual")), f"inputs={spec['inputs']}")

# =========================================================================== #
sec("2. selfcheck：钉新鲜度 + 覆盖缺口")
rep = CB.selfcheck(REAL_CFG, WS)
ok("2.1 selfcheck error 0", rep["errors"] == [], "; ".join(rep["errors"]))
ok("2.2 CLI --selfcheck exit 0", CB.main(["--selfcheck"]) == 0)
_trim = CB.CHANNEL_BY_ID["yaw_trim_deg"]
ok("2.3 yaw_trim_deg 的 ±10° 护栏由本卡提供（SCHEMA 里没有它）",
   "localization.yaw_trim_deg" not in CB._schema_keys())
# 覆盖缺口必须"要么有钉、要么报警"（自洽不变式，不写死名单）
_pinned = {c["id"] for c in CB.CHANNELS if c.get("pins")}
_gap_warned = {c["id"] for c in CB.CHANNELS
               if c.get("yaml_key") and any(f"[{c['id']}] " in w for w in rep["warns"])}
_need = {c["id"] for c in CB.CHANNELS if c.get("yaml_key")}
ok("2.4 每个有 yaml 键的通道：有钉 或 有覆盖缺口告警（自洽）",
   _need <= (_pinned | _gap_warned), f"有钉={sorted(_pinned)} 报警={sorted(_gap_warned)}")
ok("2.5 ticksPer2PI 的覆盖缺口被如实报出（dzbrain 测试里 0 处钉它）",
   "ticksPer2PI" in _gap_warned)
ok("2.6 无钉的键给出可粘贴的补钉片段", all(
    (CB.proposed_pin_snippet(c, cfg.get(c["yaml_key"]), {"verdict": "PENDING"}) is not None)
    == (not c.get("pins") and not c.get("manual"))
    for c in CB.CHANNELS if c.get("yaml_key")))

sec("3. 断言重钉清单：所有钉（含公式夹具）当前都命中")
_all_pins_ok = True
_bad: list[str] = []
for spec in CB.CHANNELS:
    for p in list(spec.get("pins", ())) + list(spec.get("formula_fixtures", ())):
        path = WS / p["file"]
        lines = path.read_text(encoding="utf-8").splitlines() if path.is_file() else []
        # ★ 2026-10-11 重钉（t20）：定位口径与**工具本身**统一（内容式，见 `CB._resolve_pin_line`）。
        #   工具早已按"行内唯一命中即跟随、多命中取登记行号 ±40 最近"定位；本断言却仍要求
        #   "登记行号那一行原文未变" ⇒ 任何在中部合法插行/删行的改动（t20 换图重钉就是）
        #   都会把这条判成红，而 `plan_repins` 明明能正确定位。改为同一条口径：
        #   ① **必须能内容式定位**；② 定位结果与登记行号的漂移 ≤ 工具窗口 ±40（不许漂远）。
        #   仍然可红：原文消失或漂出窗口（例如钉位被删/被搬走）立刻红。
        _idx, _why = CB._resolve_pin_line(lines, p)
        _drift = abs(_idx - int(p["line"])) if _idx is not None else None
        if _idx is None or _drift is None or _drift > 40:
            _all_pins_ok = False
            _bad.append(f"{p['file']}:{p['line']}"
                        f"({'未命中' if _idx is None else f'漂移 {_drift}'}：{_why or '超过窗口 ±40'})")
ok("3.1 映射表里每条钉都能内容式定位（漂移 ≤ 工具窗口 ±40）", _all_pins_ok, "; ".join(_bad))
_n_pins = sum(len(c.get("pins", ())) for c in CB.CHANNELS)
_n_fix = sum(len(c.get("formula_fixtures", ())) for c in CB.CHANNELS)
ok("3.2 钉位规模", _n_pins >= 10 and _n_fix >= 15, f"硬钉 {_n_pins} 处 / 公式夹具 {_n_fix} 处")

for spec in CB.CHANNELS:
    if not spec.get("pins"):
        continue
    # 取一个"在声明精度下必然改变文本"的假值（否则重钉会退化成空操作）
    fake_new = float(cfg.get(spec["yaml_key"])) + 2.0 * 10 ** (-int(spec["decimals"]))
    plan = CB.plan_repins(spec, {"new": fake_new}, WS)
    ok(f"3.3 [{spec['id']}] 重钉清单 {len(plan)} 条全部 OK（可安全回填）",
       bool(plan) and all(p.get("status") == "OK" for p in plan),
       "; ".join(f"{p['file']}:{p['line']}={p.get('status')}" for p in plan))
    ok(f"3.4 [{spec['id']}] 每条重钉都真的改变了该行（不是空操作）",
       all(p["line_after"] != p["old_line"] for p in plan),
       f"new={fake_new} " + "; ".join(p["line_after"] for p in plan))

# =========================================================================== #
sec("4. 判定：R3 纵向 → ticksPerMeter")
_tpm = float(cfg.get("dzactuator.ticksPerMeter"))
r = CB.calc_ticks_per_meter({"r3": {"belief_travel_m": 4.55, "long_residual_cm": 3.0,
                                    "residual_measured_by": "tape"}}, _tpm, CB.CHANNEL_BY_ID["ticksPerMeter"])
ok("4.1 |欠程| ≤4 cm ⇒ KEEP（冻结，不改配置）", r["verdict"] == "KEEP", r["action"])
r = CB.calc_ticks_per_meter({"r3": {"belief_travel_m": 4.55, "long_residual_cm": 10.0,
                                    "residual_measured_by": "tape"}}, _tpm, CB.CHANNEL_BY_ID["ticksPerMeter"])
ok("4.2 欠程 10 cm ⇒ SET", r["verdict"] == "SET", r["action"])
close("4.3 公式 new = old × 信念/实走（4.55→4.45）", r["new"], _tpm * 4.55 / 4.45, 0.05)
r_log = CB.calc_ticks_per_meter({"r3": {"belief_travel_m": 4.55, "long_residual_cm": 10.0,
                                        "residual_measured_by": "log"}}, _tpm, CB.CHANNEL_BY_ID["ticksPerMeter"])
ok("4.4 信念侧取数（log）⇒ REFUSE（吸附会抹平，R3_verdict §③）", r_log["verdict"] == "REFUSE", r_log["action"])
r_big = CB.calc_ticks_per_meter({"r3": {"belief_travel_m": 4.55, "long_residual_cm": 150.0,
                                        "residual_measured_by": "tape"}}, _tpm, CB.CHANNEL_BY_ID["ticksPerMeter"])
ok("4.5 改动超护栏 ±25% ⇒ REFUSE（先机械核查）", r_big["verdict"] == "REFUSE", r_big["action"])
r_none = CB.calc_ticks_per_meter({"r3": {}}, _tpm, CB.CHANNEL_BY_ID["ticksPerMeter"])
ok("4.6 缺实测数 ⇒ PENDING（只给一个动作）", r_none["verdict"] == "PENDING" and "补 r3." in r_none["action"])

sec("5. 判定：R4 → ticksPer2PI（四档 + 方向差 + 几何）")
_t2p = float(cfg.get("dzactuator.ticksPer2PI"))
_spec2 = CB.CHANNEL_BY_ID["ticksPer2PI"]


def _runs(ra: float, rb: float = 1.0, n: int = 2):
    base = _t2p * 2.0 * ra
    return [{"dir": d, "delta_counts": base, "yaw_gyro_deg": 720.0 * rb}
            for d in (["CW"] * n + ["CCW"] * n)]


r = CB.calc_ticks_per_2pi({"r4": {"turns": 2, "runs": _runs(1.000)}}, _t2p, _spec2,
                          ticks_per_meter=_tpm, track_m=0.157)
ok("5.1 r_A=1.000 ⇒ KEEP（收敛，冻结）", r["verdict"] == "KEEP", r["action"])
r = CB.calc_ticks_per_2pi({"r4": {"turns": 2, "runs": _runs(1.040)}}, _t2p, _spec2)
ok("5.2 r_A=1.04（2~5%）⇒ REFUSE 重测该方向 1 趟", r["verdict"] == "REFUSE" and "重测" in r["action"], r["action"])
r = CB.calc_ticks_per_2pi({"r4": {"turns": 2, "runs": _runs(1.400)}}, _t2p, _spec2)
ok("5.3 r_A=1.40（>5%）默认 ⇒ REFUSE 机械核查", r["verdict"] == "REFUSE" and "机械" in r["action"], r["action"])
r = CB.calc_ticks_per_2pi({"r4": {"turns": 2, "runs": _runs(1.400)}}, _t2p, _spec2,
                          ticks_per_meter=_tpm, track_m=0.157, allow_mechanical_fix=True)
ok("5.4 加 --allow-mechanical-fix ⇒ SET 到 ≈1.4×现值 量级", r["verdict"] == "SET", r["action"])
close("5.5 new = 现值 × r_A 中位", r["new"], _t2p * 1.4, 0.2)
ok("5.6 几何差 >10% 被显式记入依据（刮擦/滑移需解释）",
   any("10%" in d for d in r["detail"]), "; ".join(r["detail"][-2:]))
close("5.7 几何预期 = 2π·0.157·3280", r["geo"], 2 * 3.141592653589793 * 0.157 * _tpm, 0.5)
r = CB.calc_ticks_per_2pi({"r4": {"turns": 2, "runs": [
    {"dir": "CW", "delta_counts": _t2p * 2 * 1.00}, {"dir": "CW", "delta_counts": _t2p * 2 * 1.00},
    {"dir": "CCW", "delta_counts": _t2p * 2 * 1.10}, {"dir": "CCW", "delta_counts": _t2p * 2 * 1.10}]}},
    _t2p, _spec2, allow_mechanical_fix=True)
ok("5.8 CW/CCW 方向差 >5% ⇒ REFUSE（单侧滑移，先修机械）",
   r["verdict"] == "REFUSE" and "机械" in r["action"], r["action"])
r = CB.calc_ticks_per_2pi({"r4": {"turns": 2, "runs": []}}, _t2p, _spec2)
ok("5.9 无计数 ⇒ PENDING", r["verdict"] == "PENDING", r["action"])
r = CB.calc_ticks_per_2pi({"r4": {"turns": 2, "runs": _runs(1.000, rb=0.95)}}, _t2p, _spec2)
ok("5.10 r_A≈1 但 r_B 偏 ⇒ 陀螺另立标定项（不当基准）",
   any("陀螺" in d for d in r["detail"]), "; ".join(r["detail"]))

sec("6. 判定：yaw_trim / 轴距 / 轮距")
# 固定基准 −1.4，不取真配置现值：真配置一旦落盘工具建议值（如 −0.6），
# 6.5 的 SET/KEEP 判定会自反（Δ ≤ TOL ⇒ KEEP ≠ SET）。见 t3 复核报告。
_yt = -1.4
r = CB.calc_yaw_trim({"r3": {"lat_drift_cm": 20.0, "belief_travel_m": 4.55, "lat_drift_confirmed": False}},
                     _yt, CB.CHANNEL_BY_ID["yaw_trim_deg"])
ok("6.1 符号未确认 ⇒ REFUSE（符号反了会放大漂移）", r["verdict"] == "REFUSE", r["action"])
r = CB.calc_yaw_trim({"r3": {"lat_drift_cm": 20.0, "belief_travel_m": 4.55, "lat_drift_confirmed": True}},
                     _yt, CB.CHANNEL_BY_ID["yaw_trim_deg"])
ok("6.2 已确认 ⇒ SET（右漂为正输入、取负角）", r["verdict"] == "SET", r["action"])
close("6.3 suggested = −atan(0.20/4.55)", r["new"], -2.5, 0.06)
r = CB.calc_yaw_trim({"r3": {"lat_drift_cm": 50.0, "belief_travel_m": 4.55, "lat_drift_confirmed": True}},
                     _yt, CB.CHANNEL_BY_ID["yaw_trim_deg"])
ok("6.4 改动超 ±3.0° ⇒ REFUSE（复核符号/行程）", r["verdict"] == "REFUSE", r["action"])
r = CB.calc_yaw_trim({"r3": {"lat_drift_cm": 5.0, "belief_travel_m": 4.55, "lat_drift_confirmed": True}},
                     _yt, CB.CHANNEL_BY_ID["yaw_trim_deg"])
ok("6.5 漂移变小（5 cm）⇒ 补偿角相应减小到 −0.6 量级",
   r["verdict"] == "SET" and r["new"] > -1.0, f"{r['verdict']} {r.get('new')}")

_wb = float(cfg.get("pursuit.wheelbase_m"))
r = CB.calc_wheelbase({"geometry": {"wheelbase_m": 0.156}}, _wb, CB.CHANNEL_BY_ID["wheelbase_m"])
ok("6.6 轴距差 ≤2 mm ⇒ KEEP", r["verdict"] == "KEEP", r["action"])
r = CB.calc_wheelbase({"geometry": {"wheelbase_m": 0.150}}, _wb, CB.CHANNEL_BY_ID["wheelbase_m"])
ok("6.7 轴距 0.150 ⇒ SET", r["verdict"] == "SET" and abs(r["new"] - 0.150) < 1e-9, r["action"])
r = CB.calc_wheelbase({"geometry": {"wheelbase_m": 0.100}}, _wb, CB.CHANNEL_BY_ID["wheelbase_m"])
ok("6.8 轴距差 >10% ⇒ REFUSE（别把轮距当轴距）", r["verdict"] == "REFUSE", r["action"])
r = CB.calc_wheel_track({"geometry": {"wheel_track_m": 0.111}}, {})
ok("6.9 轮距 0.111 ≠ CARW(0.157) ⇒ REFUSE（复核卷尺/C++）",
   r["verdict"] == "REFUSE" and "C++" in r["action"], r["action"])
r = CB.calc_wheel_track({"geometry": {"wheel_track_m": 0.157}}, {})
ok("6.10 轮距实测=0.157=CARW ⇒ KEEP 几何判据可用（用户卷尺实测闭环）",
   r["verdict"] == "KEEP", r["action"])

# =========================================================================== #
sec("7. 干跑不改任何文件；--apply-* 先校验后写、先备份")
res = CB.evaluate({"r3": {"belief_travel_m": 4.55, "long_residual_cm": 10.0,
                          "residual_measured_by": "tape"}}, REAL_CFG, WS)
_e = [e for e in res["channels"] if e["id"] == "ticksPerMeter"][0]
ok("7.1 真配置：干跑给出改动预览", _e["verdict"] == "SET" and _e["diff"] is not None, str(_e.get("diff")))
ok("7.2 干跑后真配置字节未变", REAL_CFG.read_bytes() == _REAL_CFG_BEFORE)
ok("7.3 预览行号落在 ticksPerMeter 行", _e["diff"]["line"] == CB.locate_yaml_line(
    REAL_CFG.read_text(encoding="utf-8"), "ticksPerMeter")[0])
ok("7.4 重钉清单全部 OK（真仓库）", all(p.get("status") == "OK" for p in _e["repins"]))

cfg_copy = _TMP / "robot.yaml"
shutil.copy2(REAL_CFG, cfg_copy)
res2 = CB.evaluate({"r3": {"belief_travel_m": 4.55, "long_residual_cm": 10.0,
                           "residual_measured_by": "tape"}}, cfg_copy, _TMP)
edits = [e["diff"] for e in res2["channels"] if e.get("diff")]
log = CB.apply_config(edits, cfg_copy)
_a = _REAL_CFG_BEFORE.decode("utf-8").splitlines()
_b = cfg_copy.read_text(encoding="utf-8").splitlines()
_diff_idx = [i for i, (x, y) in enumerate(zip(_a, _b)) if x != y]
ok("7.5 回填只改 1 行且就是 ticksPerMeter 行",
   len(_diff_idx) == 1 and "ticksPerMeter" in _a[_diff_idx[0]], f"{_diff_idx}")
_new_val = Config.load(str(cfg_copy)).get("dzactuator.ticksPerMeter")
ok("7.6 回填后 Config 读到新值且等于公式值",
   abs(float(_new_val) - _tpm * 4.55 / 4.45) < 0.05, f"got {_new_val!r}")
ok("7.7 有备份文件", any(p.name.startswith("robot.yaml.bak_") for p in _TMP.iterdir()),
   str([p.name for p in _TMP.iterdir()]))
ok("7.8 真仓库配置依旧未变", REAL_CFG.read_bytes() == _REAL_CFG_BEFORE)
# ★ 行尾：工作区 robot.yaml 是 CRLF（`.gitattributes` 只在提交时归一），回填后必须仍是 CRLF，
#   否则一次回填会把整篇 577 行的换行符都换掉（= 一个巨大的伪 diff）。
_crlf_before = _REAL_CFG_BEFORE.count(b"\r\n")
_crlf_after = cfg_copy.read_bytes().count(b"\r\n")
ok("7.9 回填保留原行尾（CRLF 数不变）", _crlf_before == _crlf_after,
   f"before={_crlf_before} after={_crlf_after}")
ok("7.10 回填只改了标量本身（字节增量 = 新旧标量长度差）",
   len(cfg_copy.read_bytes()) - len(_REAL_CFG_BEFORE) ==
   len(_e["diff"]["new_scalar"]) - len(_e["diff"]["old_scalar"]),
   f"Δbytes={len(cfg_copy.read_bytes()) - len(_REAL_CFG_BEFORE)} "
   f"Δscalar={len(_e['diff']['new_scalar']) - len(_e['diff']['old_scalar'])}")

cfg_copy2 = _TMP / "robot2.yaml"
shutil.copy2(REAL_CFG, cfg_copy2)
res3 = CB.evaluate({"r3": {"belief_travel_m": 4.55, "long_residual_cm": 10.0,
                           "residual_measured_by": "tape"}}, cfg_copy2, _TMP)
edits3 = [e["diff"] for e in res3["channels"] if e.get("diff")]
cfg_copy2.write_text(cfg_copy2.read_text(encoding="utf-8").replace("ticksPerMeter: 3280.0",
                                                                   "ticksPerMeter: 9999.0"), encoding="utf-8")
_hit, _msg = _raised(CB.apply_config, edits3, cfg_copy2)
ok("7.11 改前校验失败（文件已变）⇒ 拒绝写入", _hit, _msg)

sec("8. 重钉：先校验后写；STALE 直接拒绝")
fake = _TMP / "test_fake.py"
fake.write_text('check("a", cfg.get("dzactuator.ticksPerMeter") == 3280.0)\n', encoding="utf-8")
spec_pin = {"file": "test_fake.py", "line": 1, "find": "== 3280.0", "replace": "== {v}"}
plan = CB.plan_repins({"id": "x", "yaml_key": "dzactuator.ticksPerMeter", "decimals": 1,
                       "pins": (spec_pin,)}, {"new": 3353.7}, _TMP)
ok("8.1 plan 命中且状态 OK", len(plan) == 1 and plan[0]["status"] == "OK", str(plan[0].get("status")))
ok("8.2 空操作不会发生（行确实变化）", plan[0]["line_after"] != plan[0]["old_line"], plan[0]["line_after"])
CB.apply_repins(plan)
ok("8.3 重钉真的写进去了", "== 3353.7" in fake.read_text(encoding="utf-8"))
ok("8.4 重钉有备份", any(p.name.startswith("test_fake.py.bak_") for p in _TMP.iterdir()))
# ★ 内容式定位加固（t7 踩坑：中部插行曾导致误报 STALE + KeyError）
fake2 = _TMP / "test_fake2.py"
fake2.write_text('# 行1\ncheck("a", cfg.get("dzactuator.ticksPerMeter") == 3280.0)\n# 行3\n', encoding="utf-8")
spec3 = {"file": "test_fake2.py", "line": 2, "find": "== 3280.0", "replace": "== {v}"}
fake2.write_text('# 新插行\n# 行1\ncheck("a", cfg.get("dzactuator.ticksPerMeter") == 3280.0)\n# 行3\n', encoding="utf-8")
plan3 = CB.plan_repins({"id": "x", "yaml_key": "dzactuator.ticksPerMeter", "decimals": 1,
                        "pins": (spec3,)}, {"new": 3353.7}, _TMP)
ok("8.4a 中部插行后钉自动跟随（内容式定位，不再误报 STALE）",
   plan3[0]["status"] == "OK" and plan3[0]["line"] == 3,
   f"status={plan3[0].get('status')} line={plan3[0].get('line')} drift={plan3[0].get('drift')}")
CB.apply_repins(plan3)
_txt3 = fake2.read_text(encoding="utf-8")
ok("8.4b 跟随定位后重钉写对行", "== 3353.7" in _txt3 and "# 新插行" in _txt3 and "== 3280.0" not in _txt3,
   _txt3.replace("\n", " / "))
spec2 = {"file": "test_fake.py", "line": 5, "find": "== 3280.0", "replace": "== {v}"}
plan2 = CB.plan_repins({"id": "x", "yaml_key": "dzactuator.ticksPerMeter", "decimals": 1,
                        "pins": (spec2,)}, {"new": 3353.7}, _TMP)
ok("8.5 行号漂了 ⇒ 标 STALE", plan2[0]["status"] == "STALE", plan2[0].get("why"))
_hit, _msg = _raised(CB.apply_repins, plan2)
ok("8.6 STALE 清单 ⇒ 拒绝写入", _hit, _msg)
ok("8.7 拒绝后文件未被破坏", "== 3353.7" in fake.read_text(encoding="utf-8"))

sec("9. 模板：可解析、空模板全 PENDING/MANUAL、exit 0")
tpl_text = CB.build_template()
tpl = yaml.safe_load(tpl_text)
ok("9.1 模板可被 YAML 解析且含 r3/r4/geometry", isinstance(tpl, dict)
   and {"r3", "r4", "geometry"} <= set(tpl), str(sorted(tpl)) if isinstance(tpl, dict) else "")
ok("9.2 模板文件与 build_template() 同源",
   TEMPLATE_PATH.read_text(encoding="utf-8") == tpl_text)
res0 = CB.evaluate(tpl, REAL_CFG, WS)
ok("9.3 空模板 ⇒ 只有 PENDING/MANUAL（绝不 SET）",
   all(e["verdict"] in ("PENDING", "MANUAL") for e in res0["channels"]),
   str(res0["summary"]))
ok("9.4 空模板干跑 exit 0（等实测数，不是错误）",
   CB.main(["--values", str(TEMPLATE_PATH)]) == 0)
ok("9.5 无钉的键在空模板下也给出补钉片段（队长可直接粘）",
   all(CB.proposed_pin_snippet(c, cfg.get(c["yaml_key"]), {"verdict": "PENDING"}) is not None
       for c in CB.CHANNELS if c.get("yaml_key") and not c.get("pins") and not c.get("manual")))

sec("10. 旧值引用扫描（提示，不自动改）")
_hits = CB.scan_untracked_refs("3280", WS, [p for c in CB.CHANNELS for p in c.get("pins", ())])
def _hit_is_clean(hit: str) -> bool:
    """发现口径判据：该命中是否"干净"（不含临时副本、不含历史快照）。

    ★ 判据本体放在函数里，便于**合成串自检**（见 10.1 的第 ④ 项）：若把它写成
      "只看文件名是否以 `_tmp_` 开头"（t16 之前的退化口径），下面那两条合成串都会返回
      True ⇒ 自检立刻红 —— 这样这条回归钉永远不会变成空转。
    """
    path = hit.split("（")[0]
    name = path.split("/")[-1]
    return (not any(seg.startswith("_tmp_") for seg in path.split("/"))
            and not (name == "test_core.py" and "_tmp_" in path)
            and not path.startswith("_backups/")
            and path != "_deploy_cfg.yaml")


# ★ 10.1（卡 t16 升级）：本条同时是**发现口径回归钉**（原判据保留，断言总数不变）。
#   为什么必须"自己数一遍旧口径"再对拍：现行工具扫描本来就不含 `_tmp_*` 里的副本，
#   所以只断言"命中里没有 test_core.py"**可能恒真**（空转）。这里每次现场造一个同名
#   干扰副本，并用同一遍历的 `_raw`（不排除 `_tmp_`）与工具扫描 `_hits2` 对拍。
_guard = _TMP / "tmp_verify_style_copy"
_guard.mkdir(parents=True, exist_ok=True)
(_guard / "test_core.py").write_text(
    "# 模拟 t9 变异测试的临时副本（与 src 下真 test_core.py 同名）\n"
    "TICKS_PER_METER = 3280  # 旧标定字面量\n"
    "assert 3280 == 3280\n", encoding="utf-8")
# ★ 2026-10-29（夜间裁剪余波登记）：_raw_guard 原依赖工作区里恰好存在的 t9 变异
#   自然副本（_tmp_session 下含 3280 的 _tmp_* 文件）。10-29 凌晨 _tmp_session 裁剪后
#   自然副本消失 ⇒ 本条必红（与 calib_backfill 改动无关，stash 对拍证实）。改成测试
#   自带 WS 内守卫样本：gitignored 的 _tmp_verify/（工具应排除、raw 应包含），
#   每次现造、finally 现清——判据强度不变，但不再靠工作区残骸吃饭。
_wsguard = WS / "_tmp_verify" / "t2_calib_guard_copy"
_wsguard.mkdir(parents=True, exist_ok=True)
(_wsguard / "test_core.py").write_text(
    "TICKS_PER_METER = 3280  # 守卫样本（本测试自带，非自然副本\n", encoding="utf-8")


def _raw_scan_3280() -> list[str]:
    """等价"旧口径"：同一遍历，但**不排除** `_tmp_*`（只跳过 .git/__pycache__）。"""
    out = []
    for p in WS.rglob("*"):
        if not p.is_file() or p.suffix.lower() not in CB._SCAN_EXT:
            continue
        if {".git", "__pycache__"} & set(p.parts):
            continue
        try:
            t = p.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        if "3280" in t:
            out.append(p.relative_to(WS).as_posix())
    return sorted(out)


try:
    _hits2 = CB.scan_untracked_refs("3280", WS,
                                    [p for c in CB.CHANNELS for p in c.get("pins", ())])
    _raw = _raw_scan_3280()
    _raw_guard = [h for h in _raw if h.startswith("_tmp_")]
    _synth = ["_tmp_verify/t9/mut/base/tests/test_core.py（2 处）",   # 必须判"不干净"
              "_backups/deploy_map3_x/robot.yaml（4 处）",           # 必须判"不干净"
              "src/dzbrain/tests/test_core.py（2 处）"]              # 已登记钉，判"干净"
    _synth_ok = (not _hit_is_clean(_synth[0]) and not _hit_is_clean(_synth[1])
                 and _hit_is_clean(_synth[2]))
    _clean_all = (all(_hit_is_clean(h) for h in _hits2)
                  and not any(h.startswith("_tmp_") for h in _hits2))
    ok("10.1 能列出未登记的旧值引用 + 干扰副本/历史快照不进命中（t16 口径对拍）",
       len(_hits) >= 1 and len(_hits2) >= 1 and bool(_raw_guard) and _clean_all
       and _synth_ok and len(_hits2) < len(_raw),
       f"工具扫描 {len(_hits2)} 个 / 旧口径 {len(_raw)} 个（其中 _tmp_* {len(_raw_guard)} 个）；"
       f"判据合成串自检={_synth_ok}")
finally:
    shutil.rmtree(_guard, ignore_errors=True)
    shutil.rmtree(_wsguard, ignore_errors=True)
ok("10.2 已登记的钉不再重复列出", not any("test_core.py" == h.split("（")[0].split("/")[-1] for h in _hits),
   "; ".join(_hits[:4]))
ok("10.3 工具自身/模板/目标配置不再自报（去噪）",
   not any(h.startswith("tools/calib_backfill") for h in _hits), "; ".join(_hits))

sec("11. CLI 冒烟：--help / --list / --template 都能渲染（help 里的 % 必须写成 %%）")


def _cli(*argv) -> tuple[int, str]:
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        code = CB.main(list(argv))
    return code, buf.getvalue()


ok("11.1 --help 不炸（argparse 会把 help 里的 % 当格式符）",
   "标定参数回填工具链" in CB.build_parser().format_help())
_help_buf = io.StringIO()
with contextlib.redirect_stdout(_help_buf):
    _h_hit, _h_msg = _raised(CB.main, ["--help"])
ok("11.2 --help 走 argparse 正常退出（SystemExit 0）并打印用法",
   _h_hit and _h_msg == "SystemExit: 0" and "usage: calib_backfill.py" in _help_buf.getvalue(),
   f"{_h_msg} | {_help_buf.getvalue().splitlines()[:1]}")
_t_code, _t_out = _cli("--template")
ok("11.3 --template 渲染模板且 exit 0", _t_code == 0 and "r3:" in _t_out and "r4:" in _t_out)
_l_code, _l_out = _cli("--list")
ok("11.4 --list 渲染映射总表且含全部通道",
   _l_code == 0 and all(c["id"] in _l_out for c in CB.CHANNELS))
ok("11.5 找不到实测值文件 ⇒ exit 3（不是 crash）", _cli("--values", str(_TMP / "nope.yaml"))[0] == 3)

# =========================================================================== #
shutil.rmtree(_TMP, ignore_errors=True)
ok("12.1 收尾：真仓库配置自始至终未被本测试改动", REAL_CFG.read_bytes() == _REAL_CFG_BEFORE)

print()
print("=" * 78)
print(f"结果：{_PASS}/{_PASS + _FAIL} 通过" + ("" if not _FAIL else f"，{_FAIL} 失败"))
if _FAIL:
    print("失败项：" + "; ".join(_FAILED))
    raise SystemExit(1)
print("全部通过 ✅")
raise SystemExit(0)
