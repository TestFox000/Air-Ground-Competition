# -*- coding: utf-8 -*-
r"""调参实验室 · 跑分器（A/B 双臂实跑 + 机器判据 + 人话结论）。

职责边界（刻意窄）
------------------------------------------------------------------------
本模块**只做三件事**：调 `sim_cli` 子进程、把它的 `SIM_RESULT_JSON` 行解析成指标、
把 A/B 两臂比成"一张表 + 一句结论 + 一条机器判据"。它**不重写仿真、不 import dzbrain 内部**：

* `sim.py` 头注 §5 明文规定 metrics 键名是契约（"只增不改"），走 CLI 就等于走契约；
* 每臂都需要 **退出码** 与 **产物落盘**（`traj.csv` / `traj.svg` / `by_segment.csv`）——
  这是卡上的硬判据，进程内 import 拿不到这两样；
* 子进程 = 真用户路径：工具打印的"复跑命令"学生能在自己终端里一字不改跑通；
* 顺带能测出只在这一层暴露的问题（`--set` 的 YAML 解析、`--anchor/--no-anchor` 互斥组）。

判据与退出码（`check_expectation`）
------------------------------------------------------------------------
    0   每臂退出码 0、产物 3/3 非空、expect 判据成立
    3   expect 判据不成立 —— **最有价值的一种失败**：
        `expect=same` 却出现差异（模型里有随机源，或别的键被带动了）
        `expect=change` 却逐位相同（键名拼错 / 被钳位链吃掉 / 实验设计坏了）
        `expect=obs_only` 却连轨迹都变了（"只观测"的边界被打破）
    4   某一臂退出码非 0，或产物缺失 / 空文件（跑都没跑完，谈任何结论都是假的）

⚠️ 为什么不用"人眼扫表"代替判据：现场最常见的假结论是「我看表觉得没变」。
   0.1187→0.1187 与 0.1187→0.1108 在终端里长得很像，但一个是"参数被钳住"、
   一个是"前视真的变了"。判据必须机器判，人只负责解释。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import unicodedata
from dataclasses import dataclass, field

#: 仓库根（本文件在 <repo>/tools/sim_lab/ → 上溯两层）
REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

#: 默认产物根（dzrobot_v2/artifacts/ 已在 .gitignore 里，不入库、不打扰 git status）
DEFAULT_OUT = os.path.join(REPO, "artifacts", "sim_lab")

#: 被测包的 import 根（子进程用 PYTHONPATH 注入，不污染当前进程）
PY_SRC = os.path.join(REPO, "src", "dzbrain")

#: sim_cli 的机器可读契约行前缀（见 tools/sim_cli.py:262-284）
JSON_PREFIX = "SIM_RESULT_JSON "

#: 每臂必须落地的三件产物（硬判据：缺一即判失败）
ARTIFACT_NAMES = ("traj.csv", "traj.svg", "by_segment.csv")

#: 指标字典：key → (人话名, 单位, 小数位)。顺序 = 报告里指标表的顺序。
#: 键名本体是 sim.py 头注 §5 的契约，本表只负责"怎么给人看"。
METRICS: dict[str, tuple[str, str, int]] = {
    "completed": ("跑完全程", "", -1),
    "diverged": ("发散", "", -1),
    "time_s": ("总时长", "s", 2),
    "steps": ("控制拍数", "拍", 0),
    "cross_track_rms_m": ("横向RMS(真值)", "m", 4),
    "cross_track_max_m": ("横向MAX(真值)", "m", 4),
    "cross_track_rms_corner_m": ("横向RMS(拐角)", "m", 4),
    "final_error_m": ("末点误差(真值)", "m", 4),
    "overshoot_max_m": ("顶点超调", "m", 4),
    "path_len_ratio": ("路径长度比", "×", 4),
    "out_of_bounds_steps": ("越界拍数", "拍", 0),
    "anchors": ("到位吸附次数", "次", 0),
    "steer_rate_p95_deg_s": ("舵角速率P95", "°/s", 1),
    "max_steer_rate_delivered_deg_s": ("执行侧峰值速率", "°/s", 1),
    "belief_error_rms_m": ("信念↔真值RMS", "m", 4),
    "belief_error_max_m": ("信念↔真值MAX", "m", 4),
    "steps_corner": ("拐角拍数", "拍", 0),
}

#: "车到底怎么动了"的指标集：`expect=obs_only` 要求这一组**逐位不变**。
MOTION_KEYS = (
    "time_s", "steps", "cross_track_rms_m", "cross_track_max_m", "cross_track_rms_corner_m",
    "final_error_m", "overshoot_max_m", "path_len_ratio", "anchors",
    "steer_rate_p95_deg_s", "max_steer_rate_delivered_deg_s",
)

#: 判等容差（两臂是同一份确定性代码，正常应逐位相同；留一点浮点余量）
_REL_TOL = 1e-9
_ABS_TOL = 1e-12


# --------------------------------------------------------------------------- #
#  终端排版：中文按 2 列算宽度，否则表永远是歪的
# --------------------------------------------------------------------------- #
def disp_width(text: str) -> int:
    """字符串显示宽度（东亚宽字符 2 列，其余 1 列）。"""
    width = 0
    for ch in text:
        width += 2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1
    return width


def pad(text: str, width: int, align: str = "l") -> str:
    """按显示宽度补空格；align=l 左对齐，r 右对齐。"""
    gap = max(0, width - disp_width(text))
    return (text + " " * gap) if align == "l" else (" " * gap + text)


def rule(width: int = 96, ch: str = "-") -> str:
    """分隔线。"""
    return ch * width


# --------------------------------------------------------------------------- #
#  取值与格式化
# --------------------------------------------------------------------------- #
def fmt_value(key: str, value) -> str:
    """按指标字典把值格式化成给人看的样子。"""
    label, unit, digits = METRICS.get(key, (key, "", 4))
    if value is None:
        return "—"
    if digits < 0:                      # 布尔类
        return "是" if value else "否"
    if digits == 0:
        return "%d" % int(round(float(value)))
    return ("%." + str(digits) + "f") % float(value)


def fmt_pct(before: float, after: float) -> str:
    """相对变化（分母为 0 时不给百分比，避免 inf 骗人）。"""
    if before is None or after is None or abs(float(before)) < 1e-12:
        return ""
    pct = (float(after) - float(before)) / abs(float(before)) * 100.0
    return "%+.1f%%" % pct


def same_value(a, b) -> bool:
    """两臂某指标是否"逐位相同"（带浮点余量）。"""
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    if a is None or b is None:
        return a is b
    try:
        fa, fb = float(a), float(b)
    except (TypeError, ValueError):
        return a == b
    return abs(fa - fb) <= max(_ABS_TOL, _REL_TOL * max(abs(fa), abs(fb)))


# --------------------------------------------------------------------------- #
#  跑一臂
# --------------------------------------------------------------------------- #
@dataclass
class ArmRun:
    """一臂的实跑结果（命令、退出码、指标、产物清单）。"""

    exp_id: str
    arm_id: str
    label: str
    argv: list[str] = field(default_factory=list)
    rc: int = -1
    metrics: dict = field(default_factory=dict)
    artifacts: list[tuple[str, int]] = field(default_factory=list)
    stderr_tail: str = ""
    seconds: float = 0.0
    error: str = ""

    @property
    def ok(self) -> bool:
        """退出码 0 且三件产物都非空。"""
        return self.rc == 0 and len(self.artifacts) == len(ARTIFACT_NAMES) and all(
            size > 0 for _, size in self.artifacts)

    def cmdline(self) -> str:
        """可复跑命令（相对仓库根，POSIX 分隔符；Windows/Linux 都能照抄）。"""
        return " ".join(_quote(a) for a in self.argv)


def _quote(arg: str) -> str:
    """命令回显用：带空格/中文的片段加引号。"""
    if any(ch in arg for ch in ' "\'') or not arg:
        return '"' + arg.replace('"', '\\"') + '"'
    return arg


def run_arm(exp: dict, arm: dict, dest: str, python_exe: str | None = None,
            timeout_s: float = 600.0) -> ArmRun:
    """跑一臂：`sim_cli` 子进程 + 解析 `SIM_RESULT_JSON` + 核对三件产物。

    :param exp: catalog 里的实验字典
    :param arm: 该实验的一臂（id/label/set/flags）
    :param dest: 本臂产物目录（不存在会自动创建）
    """
    python_exe = python_exe or sys.executable
    argv = [python_exe, "-m", "dzbrain.tools.sim_cli", "--json", "--quiet",
            "--segments-top", "0", "--out-dir", dest]
    for key_value in arm.get("set", []):
        argv += ["--set", key_value]
    argv += list(arm.get("flags", []))

    env = dict(os.environ)
    env["PYTHONPATH"] = PY_SRC + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    env["PYTHONIOENCODING"] = "utf-8"      # Windows 默认 GBK 会把 ★/↔ 打成 UnicodeEncodeError
    os.makedirs(dest, exist_ok=True)

    started = time.time()
    proc = subprocess.run(argv, cwd=REPO, env=env, capture_output=True, text=True,
                          encoding="utf-8", errors="replace", timeout=timeout_s)
    elapsed = time.time() - started

    run = ArmRun(exp_id=exp["id"], arm_id=arm["id"], label=arm["label"],
                 argv=argv, rc=proc.returncode, seconds=elapsed,
                 stderr_tail=(proc.stderr or "").strip()[-600:])

    payload = None
    for line in (proc.stdout or "").splitlines():
        if line.startswith(JSON_PREFIX):
            payload = line[len(JSON_PREFIX):]
    if payload is None:
        run.error = "stdout 里没有 %s 行（sim_cli 没跑到输出阶段）" % JSON_PREFIX.strip()
        return run
    try:
        data = json.loads(payload)
    except ValueError as exc:
        run.error = "SIM_RESULT_JSON 解析失败: %s" % exc
        return run
    if not data.get("ok"):
        run.error = "sim_cli 报错(stage=%s): %s" % (data.get("stage"), data.get("error"))
        return run
    run.metrics = data.get("metrics") or {}

    for name in ARTIFACT_NAMES:
        path = os.path.join(dest, name)
        size = os.path.getsize(path) if os.path.isfile(path) else -1
        run.artifacts.append((name, size))
    if run.rc != 0:
        run.error = "退出码 %d" % run.rc
    elif any(size <= 0 for _, size in run.artifacts):
        run.error = "产物缺失或空文件: %s" % ", ".join(
            "%s=%s" % (n, s) for n, s in run.artifacts)
    return run


# --------------------------------------------------------------------------- #
#  比较与结论
# --------------------------------------------------------------------------- #
def diff_rows(ra: ArmRun, rb: ArmRun) -> list[dict]:
    """两臂指标差异（只保留 METRICS 里登记过的键，按登记顺序）。"""
    rows = []
    for key, (label, unit, digits) in METRICS.items():
        if key not in ra.metrics and key not in rb.metrics:
            continue
        a, b = ra.metrics.get(key), rb.metrics.get(key)
        changed = not same_value(a, b)
        delta = ""
        if changed and digits >= 0 and a is not None and b is not None:
            delta = "%+.*f" % (digits, float(b) - float(a))
        rows.append({
            "key": key, "label": label, "unit": unit,
            "a": fmt_value(key, a), "b": fmt_value(key, b),
            "delta": delta, "pct": fmt_pct(a, b) if changed else "", "changed": changed,
        })
    return rows


def comparability_note(ra: ArmRun, rb: ArmRun) -> str:
    """两臂"能不能横比"的守卫，返回空串 = 可比。

    ★ 为什么必须有这条：未跑完全程（`diverged`）的臂**拍数更少**，于是横向 RMS / 总时长
      这些"越跑越差"的累计指标反而更漂亮。2026-10-29 实测 `latency` 实验：
      延迟 0.06→0.26 s 后 RMS 从 0.1187 掉到 0.0695（-41.5%），
      但真相是 B 臂 4476 拍就发散了 —— 若没有人拦，这张表会被读成"延迟有益"。
      所以：只要两臂的 completed / diverged 不一致，就必须在结论前打旗，禁止横比。
    """
    a_done, b_done = bool(ra.metrics.get("completed")), bool(rb.metrics.get("completed"))
    a_div, b_div = bool(ra.metrics.get("diverged")), bool(rb.metrics.get("diverged"))
    if a_done == b_done and a_div == b_div:
        return ""
    bad = []
    for run, done, div in ((ra, a_done, a_div), (rb, b_done, b_div)):
        if not done or div:
            bad.append("%s 臂只跑了 %s 拍%s" % (
                run.arm_id, fmt_value("steps", run.metrics.get("steps")),
                "（判定发散）" if div else "（未跑完全程）"))
    return ("⚠ 两臂不可横比：" + "；".join(bad) +
            "。早停会让横向 RMS / 总时长这类累计指标变好看，"
            "别把『跑得少』读成『跑得好』——结论只能是：这组参数/这个注入下跑不完。")


def conclusion_line(exp: dict, ra: ArmRun, rb: ArmRun) -> str:
    """一句话结论：被测键 旧→新 ⇒ 观测指标 旧→新（变化%）。

    只用**实测数**；某个观测指标没变化就写"无变化"，绝不把它写成"变了"。
    """
    flag = "[⚠ 未跑完·不可横比] " if comparability_note(ra, rb) else ""
    head = "%s%s %s→%s ⇒ " % (flag, exp["title"], exp["from_to"][0], exp["from_to"][1])
    parts = []
    for key in exp["watch"]:
        label, unit, _ = METRICS.get(key, (key, "", 4))
        a, b = ra.metrics.get(key), rb.metrics.get(key)
        if same_value(a, b):
            parts.append("%s 无变化（%s%s）" % (label, fmt_value(key, a), unit))
            continue
        text = "%s %s→%s%s" % (label, fmt_value(key, a), fmt_value(key, b), unit)
        pct = fmt_pct(a, b)
        if pct:
            text += "（%s）" % pct
        parts.append(text)
    if all(same_value(ra.metrics.get(k), rb.metrics.get(k)) for k in exp["watch"]):
        return head + "每个观测指标都**逐位相同**（这不是失败，是判据要求的结果，见下方机理）"
    return head + "｜".join(parts)


def check_expectation(exp: dict, ra: ArmRun, rb: ArmRun) -> tuple[bool, str]:
    """机器判据：本实验的 expect 是否成立。返回 (是否通过, 人话结论)。"""
    mode = exp.get("expect", "change")
    watched = [(k, same_value(ra.metrics.get(k), rb.metrics.get(k))) for k in exp["watch"]]
    moved = [k for k, same in watched if not same]
    still = [k for k, same in watched if same]

    if mode == "change":
        if moved:
            return True, "expect=change 成立：%d/%d 个观测指标有变化（%s）" % (
                len(moved), len(watched), "、".join(METRICS.get(k, (k,))[0] for k in moved))
        return False, ("expect=change **不成立**：%d 个观测指标全部逐位相同。"
                       "先查三件事：①键名拼错了吗（点号键必须与 core/config.py 的 SCHEMA 一致）"
                       "②被别的键钳住了吗（clamp/互斥/迁移）③两臂的命令真的不同吗"
                       % len(watched))

    if mode == "same":
        if not moved:
            return True, "expect=same 成立：%d 个观测指标逐位相同（改动被钳位链吃掉，符合机理）" % len(watched)
        return False, ("expect=same **不成立**：本该不变却变了 %s。"
                       "说明模型里有随机源，或这次改动顺带带动了别的键 —— 结论不可用"
                       % "、".join(METRICS.get(k, (k,))[0] for k in moved))

    if mode == "obs_only":
        motion_moved = [k for k in MOTION_KEYS if not same_value(ra.metrics.get(k), rb.metrics.get(k))]
        if motion_moved:
            return False, ("expect=obs_only **不成立**：动了只该观测的键，"
                           "但车的行为也变了（%s）——『只观测』的边界被打破"
                           % "、".join(METRICS.get(k, (k,))[0] for k in motion_moved))
        if moved:
            return True, "expect=obs_only 成立：轨迹/时长/误差逐位不变，只有 %s 变（观测指标）" % \
                "、".join(METRICS.get(k, (k,))[0] for k in moved)
        return False, "expect=obs_only **不成立**：连观测指标都没变，这个实验什么也没证明"

    return False, "未知 expect=%r（catalog 写错了）" % mode


# --------------------------------------------------------------------------- #
#  落盘
# --------------------------------------------------------------------------- #
def write_report(exp: dict, ra: ArmRun, rb: ArmRun, rows: list[dict],
                 verdict_ok: bool, verdict_text: str, out_dir: str) -> str:
    """写 report.txt（每实验一份）：命令 + 指标表 + 一句话结论 + 判据。"""
    lines = []
    lines.append("=" * 96)
    lines.append(" 实验 %s · %s" % (exp["id"], exp["title"]))
    lines.append("=" * 96)
    lines.append("问题    : %s" % exp["question"])
    lines.append("被测键  : %s" % exp["key"])
    lines.append("现值    : %s" % exp["now"])
    lines.append("合法档  : %s" % exp["schema"])
    lines.append("消费者  : %s" % exp["consumed"])
    lines.append("看什么  : %s" % "、".join(
        "%s(%s)" % (METRICS.get(k, (k,))[0], k) for k in exp["watch"]))
    lines.append("")
    for run in (ra, rb):
        lines.append("臂 %s %-22s 退出码 %-3s 产物 %d/%d %s  (%.1f s)" % (
            run.arm_id, run.label, run.rc,
            sum(1 for _, s in run.artifacts if s > 0), len(ARTIFACT_NAMES),
            "OK" if run.ok else "FAIL", run.seconds))
        lines.append("    复跑: %s" % run.cmdline())
        if run.error:
            lines.append("    ⚠ %s" % run.error)
        if run.stderr_tail:
            lines.append("    stderr尾: %s" % run.stderr_tail.replace("\n", " | ")[-300:])
    lines.append("")

    width = max(28, max(disp_width(r["label"]) for r in rows) + 2)
    header = "%s %12s %12s %12s %10s" % (
        pad("指标", width), pad("A " + ra.label, 12, "r")[:24], pad("B " + rb.label, 12, "r")[:24],
        pad("Δ", 12, "r"), pad("Δ%", 10, "r"))
    lines.append(header)
    lines.append(rule(96))
    watched = set(exp["watch"])
    for row in rows:
        mark = "★" if row["key"] in watched else " "
        lines.append("%s %s %12s %12s %12s %10s" % (
            mark, pad(row["label"], width - 2), pad(row["a"], 12, "r"),
            pad(row["b"], 12, "r"), pad(row["delta"] or "—", 12, "r"),
            pad(row["pct"] or "—", 10, "r")))
    lines.append(rule(96))
    note = comparability_note(ra, rb)
    lines.append("★ 一句话结论：" + conclusion_line(exp, ra, rb))
    if note:
        lines.append("可比性   : " + note)
    lines.append("")
    lines.append("判据     : %s" % verdict_text)
    lines.append("机理     : %s" % exp.get("expect_note", ""))
    lines.append("产物     : 每臂一套 %s（在本目录的 %s/ 与 %s/ 下）" % (
        " / ".join(ARTIFACT_NAMES), ra.arm_id, rb.arm_id))
    lines.append("")

    path = os.path.join(out_dir, "report.txt")
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(lines))
    return path


def write_lab_index(summary: list[dict], out_dir: str) -> str:
    """写 index.txt：全实验室目录 + 每实验的一句话结论 + 判据状态。"""
    lines = []
    lines.append("=" * 96)
    lines.append(" 调参实验室 · 总索引（由 run_experiments.py 实跑生成，勿手改）")
    lines.append("=" * 96)
    lines.append("怎么跑：python tools/sim_lab/run_experiments.py --all")
    lines.append("看什么：每行后面的『一句话结论』都是从实测指标算出来的；report.txt 里有全表")
    lines.append("")
    lines.append("%s %s %s %s" % (pad("实验", 20), pad("分组", 10), pad("判据", 8), "一句话结论"))
    lines.append(rule(96, "="))
    for item in summary:
        lines.append("%s %s %s %s" % (
            pad(item["id"], 20), pad(item["group"], 10),
            pad("通过" if item["ok"] else "失败", 8), item["conclusion"]))
    lines.append(rule(96, "="))
    ok = sum(1 for i in summary if i["ok"])
    lines.append("合计 %d 个实验 / 通过 %d / 失败 %d" % (len(summary), ok, len(summary) - ok))
    path = os.path.join(out_dir, "index.txt")
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(lines) + "\n")
    return path


# --------------------------------------------------------------------------- #
#  判据自检（--selftest）：证明"判据不是空转"
# --------------------------------------------------------------------------- #
def _fake(arm_id: str, **metrics) -> ArmRun:
    """造一臂假数据（只填 metrics，不跑仿真）——专给 selftest 用。"""
    base = {"completed": True, "diverged": False, "steps": 100.0, "time_s": 10.0,
            "cross_track_rms_m": 0.1, "cross_track_max_m": 0.2, "final_error_m": 0.03,
            "overshoot_max_m": 0.1, "path_len_ratio": 0.9, "anchors": 4.0,
            "cross_track_rms_corner_m": 0.1, "steer_rate_p95_deg_s": 20.0,
            "max_steer_rate_delivered_deg_s": 40.0, "out_of_bounds_steps": 7.0,
            "belief_error_rms_m": 0.0, "belief_error_max_m": 0.0}
    base.update(metrics)
    return ArmRun(exp_id="selftest", arm_id=arm_id, label=arm_id, rc=0, metrics=base,
                  artifacts=[(n, 1) for n in ARTIFACT_NAMES])


def selftest() -> tuple[bool, list[str]]:
    """判据自检：用**合成数据**验证 change/same/obs_only 三种判据都真的会判错。

    ★ 为什么必须有它：判据最容易写成"永远返回 True"（例如比较写错变量、容差写成无穷大），
      于是实验室永远绿、永远没人发现问题。本函数用 4 个**必失败**用例 + 3 个**必通过**用例
      把判据钉住 —— 不需要跑仿真，毫秒级。
    """
    ok = True
    lines = []

    def check(name: str, got: bool, want: bool) -> None:
        nonlocal ok
        good = got == want
        ok = ok and good
        lines.append("  [%s] %s（判据返回 %s，期望 %s）" % ("OK " if good else "RED", name, got, want))

    exp_change = {"id": "t", "title": "t", "from_to": ("1", "2"), "watch": ["time_s"],
                  "expect": "change"}
    exp_same = dict(exp_change, expect="same")
    exp_obs = {"id": "t", "title": "t", "from_to": ("1", "2"), "watch": ["out_of_bounds_steps"],
               "expect": "obs_only"}

    a = _fake("A")
    b_same = _fake("B")
    b_diff = _fake("B", time_s=12.0)
    b_obs = _fake("B", out_of_bounds_steps=99.0)
    b_motion = _fake("B", out_of_bounds_steps=99.0, time_s=99.0)

    check("change + 两臂相同 → 必须判失败", check_expectation(exp_change, a, b_same)[0], False)
    check("change + 两臂不同 → 必须判通过", check_expectation(exp_change, a, b_diff)[0], True)
    check("same + 两臂相同 → 必须判通过", check_expectation(exp_same, a, b_same)[0], True)
    check("same + 两臂不同 → 必须判失败", check_expectation(exp_same, a, b_diff)[0], False)
    check("obs_only + 只动观测指标 → 必须判通过", check_expectation(exp_obs, a, b_obs)[0], True)
    check("obs_only + 轨迹也动了 → 必须判失败", check_expectation(exp_obs, a, b_motion)[0], False)

    both_done = comparability_note(a, b_diff) == ""
    check("两臂都跑完 → 不打「不可横比」旗", both_done, True)
    early = comparability_note(a, _fake("B", completed=False, diverged=True, steps=50.0)) != ""
    check("B 臂发散 → 必须打「不可横比」旗", early, True)

    concl = conclusion_line(exp_change, a, b_diff)
    check("结论里带实测数（10.00→12.00）", ("10.00" in concl and "12.00" in concl), True)
    return ok, lines
