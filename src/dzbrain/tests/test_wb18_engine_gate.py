# -*- coding: utf-8 -*-
"""WB-18：三键危险组合的**启动期硬闸**（engine=tracker-safe + auto_plan=true + fallback=false）。

为什么需要这道闸（WB-14 实测，不是推测）：
  `tracker-safe` 在四张现场真实图上 **4/4 无解**；逐组放宽证明三种成因；放宽后产出仍被
  `assert_legal_no_reverse` 拒绝 180° 掉头 ⇒ **没有"调参修好"路径**。
  与另两键组合起来 = "自动规划开着 + 失败不回退手写线" ⇒ 每次规划失败即停车报错，
  **配置能瞬间把车变成不能动**。
  闸门的价值：把这件事从「调试时才发现」变成「启动时就报错」并给出三种可行动作。

覆盖（对应任务卡 WB-18 验收 ①~⑤ + 本线判据 L1/L6/L7）：
  §1 纯判定：危险组合命中；**三条负例**各一条（换 tour / 开 fallback / 关 auto_plan）必须放行。
  §2 缺键语义（**本卡最容易假红的地方**）：`fallback_to_tokens` 的 SCHEMA 兜底是 **True**
     ⇒ 缺该键时组合安全，**不许**判危险。另验 engine/auto_plan 缺键时的兜底语义。
  §3 报错正文可读：含三种可行动作、含三键实际值、含退出码与 gate key；且**不含**任何
     "已为你自动修改"之类越权措辞（本闸只读配置）。
  §4 端到端拒绝：`_run_live_body` 在**建 ROS bridge 之前**返回 exit 7（用哨兵证明 bridge
     根本没被构造 ⇒ 危险配置不会留下半启动节点，也不会先耗掉「等布局」的时间）。
  §5 端到端放行（三条负例）：越过闸门（哨兵被调用）—— 证明不是"一刀切"。
  §6 只读性：判定前后 `cfg._flat` 三键取值不变（本闸不改配置、不写 config/**）。
  §7 **不误伤合法配置**（卡面特别要求）：现役 config/robot.yaml 必须判安全；
     且枚举 2×2×2 的 8 种组合，**恰好只有 1 种**判危险 —— 闸门的判别面必须是"这一个组合"，
     不是"凡是 tracker-safe 就拦"（否则下一个人把 fallback 打开就被误伤）。

跑法（工作区根 `dzrobot_v2/`）：
    PYTHONPATH=src/dzbrain python src/dzbrain/tests/test_wb18_engine_gate.py
全过 exit 0；任一失败 exit 1。
"""

from __future__ import annotations

import copy
import pathlib
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
try:
    sys.stdout.reconfigure(errors="replace")
    sys.stderr.reconfigure(errors="replace")
except Exception:
    pass

import yaml                                                        # noqa: E402

import dzbrain.main as M                                           # noqa: E402
from dzbrain.core.config import Config                             # noqa: E402
from harness import check, finish, section                         # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]
CFG_PATH = WS / "config" / "robot.yaml"
PLANNER_KEYS = ("engine", "auto_plan_enabled", "fallback_to_tokens")
#: 危险组合的三个键值（**判据真源**：改这里 = 改判据）。
DANGER = {"engine": M.GATE_DANGER_ENGINE, "auto_plan_enabled": True, "fallback_to_tokens": False}

_RAW = yaml.safe_load(CFG_PATH.read_text(encoding="utf-8"))
_TMP = pathlib.Path(tempfile.mkdtemp(prefix="wb18_"))


def build_cfg(**over) -> Config:
    """按现役 robot.yaml 派生一份临时配置；`over` 里值为 None ⇒ **删掉该键**（测缺键语义）。"""
    raw = copy.deepcopy(_RAW)
    planner = raw["/dzbrain"]["ros__parameters"]["planner"]
    for key, value in over.items():
        if value is None:
            planner.pop(key, None)
        else:
            planner[key] = value
    path = _TMP / f"robot_{len(list(_TMP.glob('*.yaml')))}.yaml"
    path.write_text(yaml.safe_dump(raw, allow_unicode=True), encoding="utf-8")
    return Config.load(str(path))


def is_danger(cfg: Config) -> bool:
    return M._planner_engine_combo_risk(cfg) is not None


# --------------------------------------------------------------------------- #
section("§1 纯判定：危险组合命中，三条负例各放行")
# --------------------------------------------------------------------------- #
check("1.0 危险组合（ts + auto_plan=true + fallback=false）⇒ 命中",
      is_danger(build_cfg(**DANGER))
      and M._planner_engine_combo_risk(build_cfg(**DANGER))["engine"] == M.GATE_DANGER_ENGINE)
check("1.1 负例① engine=tour ⇒ 放行（闸门只拦 tracker-safe）",
      not is_danger(build_cfg(engine="tour")))
check("1.2 负例② fallback_to_tokens=true ⇒ 放行",
      not is_danger(build_cfg(engine="tracker-safe", fallback_to_tokens=True)))
check("1.3 负例③ auto_plan_enabled=false ⇒ 放行",
      not is_danger(build_cfg(engine="tracker-safe", auto_plan_enabled=False)))
# captain 点名补的第 4 条：**合法但不常见**的组合（手写模式 + 历史引擎）。
# 为什么单列：它同时关掉了 auto_plan 换走了引擎，是最容易被"一刀切闸门"误伤的一种。
check("1.4 负例④ engine=tour + auto_plan_enabled=false（合法但不常见的手写模式）⇒ 放行",
      not is_danger(build_cfg(engine="tour", auto_plan_enabled=False))
      and not is_danger(build_cfg(engine="tour", auto_plan_enabled=False,
                                  fallback_to_tokens=False)))
# L6 反向半边（wb-core 要求）：不是"逐条负例放行"就算完，还要证明
# 「tracker-safe 的**所有**危险朝向都被拦」—— 与负例方向相反的一条。
check("1.5 反向：engine=tracker-safe 且 auto_plan=true 时，fallback 取 false ⇒ **必拦**，"
      "取 true ⇒ 放行（闸门对同一引擎能区分两态）",
      is_danger(build_cfg(engine="tracker-safe", auto_plan_enabled=True, fallback_to_tokens=False))
      and not is_danger(build_cfg(engine="tracker-safe", auto_plan_enabled=True,
                                  fallback_to_tokens=True)))

# --------------------------------------------------------------------------- #
section("§2 缺键语义（本卡最容易假红的地方）")
# --------------------------------------------------------------------------- #
# SCHEMA 兜底：engine=tour / auto_plan_enabled=False / fallback_to_tokens=**True**
# ⇒ 缺 fallback 时生效值是 True ⇒ 安全。若按"值等于 False"判而不看兜底，就会误伤。
check("2.1 缺 fallback_to_tokens（兜底 True）⇒ **安全**（不许判危险）",
      not is_danger(build_cfg(engine="tracker-safe", fallback_to_tokens=None))
      and bool(build_cfg(engine="tracker-safe", fallback_to_tokens=None)
               .get("planner.fallback_to_tokens")) is True)
check("2.2 缺 auto_plan_enabled（兜底 False）⇒ 安全",
      not is_danger(build_cfg(engine="tracker-safe", auto_plan_enabled=None))
      and bool(build_cfg(engine="tracker-safe", auto_plan_enabled=None)
               .get("planner.auto_plan_enabled")) is False)
check("2.3 缺 engine（兜底 tour）⇒ 安全，且生效值是 tour",
      not is_danger(build_cfg(engine=None))
      and str(build_cfg(engine=None).get("planner.engine")) == "tour")
check("2.4 三键全缺 ⇒ 安全（组合退化为 tour + 不自动规划）",
      not is_danger(build_cfg(engine=None, auto_plan_enabled=None, fallback_to_tokens=None))
      and build_cfg(engine=None, auto_plan_enabled=None,
                    fallback_to_tokens=None).get("planner.engine") == "tour")
# 反向：把 fallback 显式写成 false 与"缺键"的**判定必须不同**（L6 单侧断言的反向半边）
check("2.5 反向：显式 fallback=false 判危险，而缺该键判安全 ⇒ 判据能区分两态",
      is_danger(build_cfg(engine="tracker-safe", fallback_to_tokens=False))
      and not is_danger(build_cfg(engine="tracker-safe", fallback_to_tokens=None)))

# --------------------------------------------------------------------------- #
section("§3 报错正文：可读 + 三种动作 + 不越权")
# --------------------------------------------------------------------------- #
_lines = M._engine_combo_error_lines(M._planner_engine_combo_risk(_danger_cfg := build_cfg(**DANGER)))
_text = "\n".join(_lines)
check("3.1 首行含退出码与 gate key（机端可 grep）",
      _lines[0].startswith("❌") and f"exit {M.EXIT_ENGINE_COMBO}" in _lines[0]
      and f"gate={M.GATE_ENGINE_COMBO}" in _lines[0])
check("3.2 三键实际值都打印出来（排障一眼看到是哪个键的问题）",
      all(f"{k} = " in _text for k in (
          "planner.engine", "planner.auto_plan_enabled", "planner.fallback_to_tokens")))
check("3.3 三条出路都在正文里（一条可用 + 两条看似可行实则无效）",
      "planner.engine 改回 tour" in _text
      and "打开 fallback_to_tokens" in _text
      and "关掉 auto_plan_enabled" in _text)
# ★ captain 2026-11-01 更正：原卡面写"三种组合都必须正常启动"是**错的** ——
#   `mission.py` 的 B 分支（fallback=true, :2842）与 C 分支（auto_plan=false, :2797）
#   读的是**同一个键** `route.tokens`，而该键已随 NOLEGACY 删除 ⇒ 两条路通向同一堵墙，
#   运行期以 `ValueError("route.tokens 为空，无法载入路线")` 失败（:2949）。
#   ⇒ 文案必须**只把 ① 列为可用**，②③ 归入"为什么不通"的说明区。
#   这条断言把"不许把 ② 列进可用"钉死（否则又是空安全网 —— L1 声明面=可用面）。
check("3.3a 文案把 ②③ 明确标注为**当前无效**（不许再出现『三选一/任一条即可解除』）",
      "另有两条看似可行、但在本仓当前配置下无效" in _text
      and "三选一" not in _text and "任一条即可解除" not in _text)
check("3.3b ②③ 的无效原因都指向同一个键 route.tokens（说清是同一堵墙，不是两个问题）",
      _text.count("route.tokens") >= 2)
check("3.3c 唯一可用出路被单列，且给出要真用 ②③ 的前置条件（可行动的下一步）",
      "可用出路只有一条" in _text and "必须先把手工路线写回 route.tokens" in _text)
check("3.4 不越权：正文不承诺自动改配置（本闸只读）",
      "已自动" not in _text and "为你修改" not in _text and "已修改" not in _text)
# ★ captain 第 ④ 条：拒绝**不许杀进程**（否则车可能失去遥控通道，
#   把"配置错"升级成"现场救不回来"）。正文要明确说明栈仍在、可被接管。
check("3.4a 正文声明不杀进程、栈仍在可被接管（遥控通道不因配置错丢失）",
      "不杀进程" in _text and "可被接管" in _text)
check("3.5 稀出配置来源（知道该改哪个文件）",
      str(_danger_cfg.source) in _text)

# --------------------------------------------------------------------------- #
section("§4 端到端拒绝：在建 ROS bridge **之前** 返回 exit 7")
# --------------------------------------------------------------------------- #
# 用哨兵替换 `ros_io.RosBridge`：`_run_live_body` 里是 `from .io.ros_io import RosBridge`
# （**函数内局部 import**），所以替换必须打在 `ros_io` 模块属性上，不是 `main` 上
# （实测 `main` 根本没有 `RosBridge` 属性 ⇒ 打错位置会 AttributeError，而不是"测不到"）。
# 为什么用哨兵而不是看 rc：无 ROS 的机器上，闸门放行后必然在 `RosBridge()` 处
# 抛 `ModuleNotFoundError`。若只断言 rc，就分不清"被闸门拦(7)"与"没 ROS 环境崩"。
# 哨兵把"有没有越过闸门"变成**与 ROS 无关的可判事实**（本线 L7：负例必须能区分）。
class _Reached(Exception):
    """哨兵：`_run_live_body` 越过闸门并试图构造 RosBridge。"""


import dzbrain.io.ros_io as _ros_io                              # noqa: E402

_reached = {"n": 0}


def _bridge_sentinel(*args, **kwargs):
    _reached["n"] += 1
    raise _Reached("越过闸门（闸门已放行）")


class _sentinel_installed:
    """with 语句：装上哨兵、跑、还原。"""

    def __enter__(self):
        _reached["n"] = 0
        self._saved = _ros_io.RosBridge
        _ros_io.RosBridge = _bridge_sentinel
        return self

    def __exit__(self, *exc):
        _ros_io.RosBridge = self._saved
        return False


_logs: list[str] = []
_rc = None
_gate_passed = False
with _sentinel_installed() as _s:
    try:
        _rc = M._run_live_body(_danger_cfg, log=_logs.append, alive=lambda: False)
    except _Reached:
        # ★ 走到这里 = **闸门没拦住**（危险组合被放行，bridge 被构造）。
        #   必须在这里捕获并判红，不许让异常逃出去把整个脚本崩掉 ——
        #   崩掉时只会打 traceback、**一行 FAIL 都没有**，看起来像"测试坏了"而不是
        #   "闸门坏了"。实测：拆闸后正是这种形态（2026-11-01 本卡自证时抓到）。
        _gate_passed = True
check("4.1 危险组合 ⇒ 返回 EXIT_ENGINE_COMBO（7），不是 0/5/6（也没被放行）",
      _rc == M.EXIT_ENGINE_COMBO and not _gate_passed,
      f"rc={_rc} 被放行={_gate_passed}")
check("4.2 **bridge 根本没被构造**（闸门在最前面 ⇒ 不留半启动节点）",
      _reached["n"] == 0, f"bridge 调用次数={_reached['n']}")
check("4.3 拒绝时把完整报错打进日志（不是只给退出码）",
      any(f"gate={M.GATE_ENGINE_COMBO}" in ln for ln in _logs)
      and any("可用出路只有一条" in ln for ln in _logs))
check("4.4 拒绝早于「等布局」：日志里没有布局/无人机的等待痕迹",
      not any("等 field_layout" in ln or "布局模式" in ln for ln in _logs))

# --------------------------------------------------------------------------- #
section("§5 端到端放行：四条负例都越过闸门（机制判据，不判运行期结局）")
# --------------------------------------------------------------------------- #
# ★★ captain 2026-11-01 更正（原卡面判据写错了，这里按新口径）：
#   原卡面要求「三种组合都必须**正常启动**」——**那是错的**。
#   `mission.py:2797`（auto_plan=false）与 `:2842`（fallback=true）都读 `route.tokens`，
#   而该键已随 NOLEGACY 删除 ⇒ 这两条路径在运行期必然 `ValueError("route.tokens 为空…")`。
#   若沿用"必须正常启动"，为了让断言过就只能去改 mission.py（本卡禁止）或放宽判据（禁止）。
#
#   ⇒ 判据拆成三件（captain 的 A/B/C）：
#     A. **机制判据**：启动日志里**没有闸门拒绝行**（= 闸门没拦它）  ← 本节断言这个
#     B. **运行期结局单独记录**：该组合若失败，原因是 route.tokens 缺失，
#        并以 `ValueError` 呈现，**不是闸门** ⇒ 不许把它写成"闸门假红"
#     C. 只有 (tracker-safe, true, false) 这一个组合，闸门必须拒绝  ← §1/§4/§7 断言
#
#   本节只做 A + 把 B 的结局**如实打印**（不作通过/失败判据）；C 由别节钉。
for _name, _over in (("① engine=tour", dict(engine="tour")),
                     ("② fallback=true", dict(engine="tracker-safe", fallback_to_tokens=True)),
                     ("③ auto_plan=false", dict(engine="tracker-safe", auto_plan_enabled=False)),
                     ("④ tour+手写模式", dict(engine="tour", auto_plan_enabled=False))):
    _n_before = _reached.copy()

    def _pass_through(over=_over):
        """返回 (哨兵调用次数, 日志, 运行期异常名或None)。"""
        logs: list[str] = []
        exc_name = None
        with _sentinel_installed():
            try:
                M._run_live_body(build_cfg(**over), log=logs.append, alive=lambda: False)
            except _Reached:
                pass
            except Exception as exc:                        # noqa: BLE001
                exc_name = type(exc).__name__
            return _reached["n"], logs, exc_name

    _n, _lg, _exc = _pass_through()
    check(f"5.{_name[0]} 负例{_name} ⇒ 越过闸门（哨兵被调用，且没打本闸报错）",
          _n == 1 and not any(M.GATE_ENGINE_COMBO in ln for ln in _lg),
          f"bridge 调用={_n}")
    # B：如实记录运行期结局。**这一条不判通过/失败**（今天 ②③ 必然 ValueError，
    #    那是 NOLEGACY 遗留，不是本卡的问题；把它判红就是"过严也是失真"）。
    print(f"       └ 运行期结局（B，仅记录不判）：{_exc or '未走到失败点'}")

# --------------------------------------------------------------------------- #
section("§5b 判据 B：②③ 的运行期结局**不是闸门**，是 route.tokens 缺失（如实分档）")
# --------------------------------------------------------------------------- #
# 为什么要单独证明这条（captain 第 ②③ 条 / L6 能区分两种失败）：
#   §5 证明"闸门没拦它们"（机制层）。但**不许把它们的运行期失败写成闸门假红**，
#   也不许为了让断言过而放宽判据。所以这里把"它们的失败原因"**独立**钉死：
#     ① 配置里 `route.tokens` 这个键**不存在**（NOLEGACY 删掉了 path_executor 整段）
#     ② `MissionTask._load_tokens(None)` **必然** `raise ValueError`
#   ⇒ 两条分支（:2797 / :2842）读的都是这个键 ⇒ 结局是 ValueError，与闸门无关。
import inspect                                                 # noqa: E402

from dzbrain.tasks import mission as _mission                  # noqa: E402

_live_rp = yaml.safe_load(CFG_PATH.read_text(encoding="utf-8"))["/dzbrain"]["ros__parameters"]
check("5b.1 `route.tokens` 在**现役配置里没有真值**（②③ 回退的目标本身是空的）",
      "tokens" not in (_live_rp.get("route") or {}),
      f"route 段键={list((_live_rp.get('route') or {}).keys())}")
# ★ 精确口径（别写成"键不存在"就完事）：键在 **SCHEMA 里仍存在**（config.py:319，兜底 `[]`），
#   只是**现役 YAML 没配它** ⇒ `cfg.get()` 返回 `[]`（空列表，不是 None）。
#   但 `if not tokens:` 对 `[]` 与 `None` 同样成立 ⇒ 结局一致（都会 ValueError）。
#   这条差别要说清，否则下一个人会以为"键没了、代码读会 KeyError"，做出错误修法。
check("5b.1a `cfg.get('route.tokens')` 返回**空列表 `[]`**（SCHEMA 兜底），不是 None",
      Config.load(str(CFG_PATH)).get("route.tokens") == []
      and "route.tokens" not in Config.load(str(CFG_PATH))._flat)
check("5b.1b 空列表与 None 在 `if not tokens:` 下**等价**（所以结局相同，不是两种毛病）",
      bool(not []) and bool(not None))
check("5b.2 `path_executor` 段也**不存在**（手工路线是真被整段删了，不是改名）",
      "path_executor" not in _live_rp)
# 两条分支读的是不是同一个键 —— 直接读源码，不靠转述。
_src = inspect.getsource(_mission.MissionTask._load_startup_route)
check("5b.3 两条分支读**同一个键** route.tokens（说清是同一堵墙，不是两个问题）",
      _src.count('self.cfg.get("route.tokens")') == 2,
      f"出现次数={_src.count('self.cfg.get(\"route.tokens\")')}")
_sig = inspect.getsource(_mission.MissionTask._load_tokens)
check("5b.4 `_load_tokens` 对空输入**必然 ValueError**（②③ 走到那里就是硬失败）",
      "if not tokens:" in _sig and "raise ValueError" in _sig)
check("5b.5 因此 ②③ 的结局归因为 `route.tokens 缺失`，**不是**本闸（两件事分开记账）",
      True, "见 5b.1/5b.3/5b.4 三条事实链")


# --------------------------------------------------------------------------- #
_c = build_cfg(**DANGER)
_before = {k: _c.get(f"planner.{k}") for k in PLANNER_KEYS}
_ = M._planner_engine_combo_risk(_c)
_after = {k: _c.get(f"planner.{k}") for k in PLANNER_KEYS}
check("6.1 判定前后三键取值逐位不变（本闸只读，不改默认值）", _before == _after,
      f"{_before}")
check("6.2 `_flat` 三键仍在（没有被 pop / 覆盖）",
      all(f"planner.{k}" in _c._flat for k in PLANNER_KEYS))

# --------------------------------------------------------------------------- #
section("§7 不误伤合法配置（卡面特别要求）")
# --------------------------------------------------------------------------- #
check("7.1 **现役** config/robot.yaml 判安全（当前 tour + auto_plan=true + fallback=false）",
      not is_danger(Config.load(str(CFG_PATH)))
      and str(Config.load(str(CFG_PATH)).get("planner.engine")) == "tour")

# 枚举三键的全部 8 种组合：**恰好只有 1 种**判危险。
# 这条防的是"凡是 tracker-safe 就拦"这类过度防御 —— 打开 fallback 或关掉 auto_plan
# 都是**合法**配置（现场可选），闸门不许把它们一起拦掉（判据过严同样是失真）。
_grid = []
for eng in ("tour", "tracker-safe"):
    for ap in (True, False):
        for fb in (True, False):
            cfg_g = build_cfg(engine=eng, auto_plan_enabled=ap, fallback_to_tokens=fb)
            _grid.append(((eng, ap, fb), is_danger(cfg_g)))
_dangerous = [combo for combo, hit in _grid if hit]
check("7.2 8 种组合里**恰好 1 种**判危险（不是『凡 tracker-safe 就拦』）",
      len(_grid) == 8 and len(_dangerous) == 1, f"危险组合={_dangerous}")
check("7.3 那个唯一的危险组合正是 (tracker-safe, True, False)",
      _dangerous == [("tracker-safe", True, False)], f"{_dangerous}")
check("7.4 合法配置不被误伤：tracker-safe+fallback=true 的 2 种组合全部放行",
      all(not hit for combo, hit in _grid
          if combo[0] == "tracker-safe" and combo[2] is True))

# --------------------------------------------------------------------------- #
finish()
