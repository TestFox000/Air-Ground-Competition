# -*- coding: utf-8 -*-
r"""收栈（SIGTERM）僵尸态测试（纯脚本，无 pytest）。

运行：python src\dzbrain\tests\test_shutdown.py

背景（2026-10-11 16:18 实车"假崩溃"，取证报告 `_tmp_verify/t6/报告.md`）：
`start_all.sh stop` 按 pid 1→7 逐个 `kill -TERM -- -PGID` 收栈时，dzbrain 最后一个被 TERM ——
但它**没有接管 SIGTERM**，信号被 `rclpy.init()` 默认装的 C++ 处理器接走，那个处理器只做
`rcl_shutdown`（上下文失效）、**不抛 Python 异常、不退进程**。于是：

    while not stop_flag["stop"]:      # 只看停机标志/FINISH
        time.sleep(0.2)               # ← 上下文已死，这一圈永远退不出去

8 个任务线程继续按 50/20/10 Hz 打 `RCLError: publisher's context is invalid`（约 100 行/s），
直到 3 s 后被 `kill_group` 的 SIGKILL 兜掉。日志里**没有**任何 teardown 留痕。

本测试钉死修复后的三条：
  1. `SIGINT`/`SIGTERM` **都由程序显式接管**：处理器只置停机标志 + 记信号名（信号上下文里
     不 print、不碰 ROS），留痕由主循环打印；
  2. 主循环**每拍**检查 `rclpy.ok()`：上下文失效后 ≤1 拍退出（本测试用注入的 alive 替身，
     不依赖真 ROS），并给出足够信息量够的 teardown 留痕；
  3. 信号置位也能正常退出（`stop` 路径），且 EXIT 原因可区分。

⚠️ 本文件**不 import rclpy**、不连任何话题：`_live_loop` 的 alive 判据是注入的，
   `RosBridge` 的 `rclpy.init(signal_handler_options=...)` 由 test_referee 的替身覆盖。
"""

from __future__ import annotations

import ast
import json
import os
import pathlib
import signal
import subprocess
import sys
import types

#: 被测包根覆盖（**变异测试专用**）：设了它就先把它插到 `sys.path[0]`。
#  ★ 为什么必须有这一条：本文件下面那句 `sys.path.insert(0, ../..)` 会把**真源**
#    （`src/dzbrain`）钉在最前，于是"只用 PYTHONPATH 指到变异体复制树"是**无效**的
#    —— 真源仍然赢（实测：变异的 main.py 在 PYTHONPATH 里，但导入到的是真源）。
#    变异复现必须显式覆盖，否则会得到"变异体竟然全绿"的假结论。
_PKG_ROOT_OVERRIDE = os.environ.get("DZ_PKG_ROOT")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
if _PKG_ROOT_OVERRIDE:
    # ★ 顺序要紧：先插 tests（保持原行为），**再**插覆盖根 ⇒ 覆盖根落在 sys.path[0]。
    #   反过来（先插覆盖根）会被后插的 tests/../.. 压到第 2 位，真源照旧赢。
    sys.path.insert(0, _PKG_ROOT_OVERRIDE)

try:                                    # 车端/CI 控制台默认不是 UTF-8，中文会乱码
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

import dzbrain.main as M                                                    # noqa: E402
from dzbrain.core.blackboard import Blackboard                              # noqa: E402
from harness import check, finish, section                                  # noqa: E402

#: 本测试自己的目录（放子进程哨兵 `_run_loop_case.py`）
TESTS_DIR = pathlib.Path(__file__).resolve().parent

# ★ 自证"变异体真的被导入了"：设了 `DZ_PKG_ROOT` 却仍然导入到真源，会让**变异测试得到
#   "变异体全绿"的假结论**（本卡实测踩过：只给 PYTHONPATH 时，本文件自己的
#   `sys.path.insert(0, ../..)` 让真源赢）。有覆盖时必须报红。
if _PKG_ROOT_OVERRIDE:
    check("⓪ 变异复现：DZ_PKG_ROOT 覆盖生效（导入到的是被测副本，不是真源）",
          pathlib.Path(M.__file__).resolve().is_relative_to(
              pathlib.Path(_PKG_ROOT_OVERRIDE).resolve()),
          f"M.__file__={M.__file__} root={_PKG_ROOT_OVERRIDE}")


def rec(logs: list) -> object:
    """收集日志的 logger（主循环的"留痕"就是被它接住的）。"""
    return logs.append


def run_loop_case(case: str, *, wait_s: float = 4.0, root: str | None = None,
                  hard_s: float = 30.0) -> dict:
    """在**子进程**里跑一次 `_live_loop`，带**硬超时**（O7，t9 复核 §4）。

    为什么不能在本进程直接调：M4 变异（把 `if not alive():` 改成永不成立 = 去掉判活退出）
    会让 `_live_loop` 永久空转（正是现场僵尸态），直接调用的话测试**挂死而不是红**
    ——t9 实测 5 分钟不结束，CI 会被卡住。子进程 + `subprocess.run(timeout)` ⇒
    任何僵尸态变异都变成一条可读的 **FAIL（timeout）**，而不是把门禁拖死。

    `root`：含 `dzbrain/` 的包根目录（变异体复现用）；None = 被测真源。
    返回：`{timeout, exit_code, result(dict|None), stderr}`。
    """
    cmd = [sys.executable, str(TESTS_DIR / "_run_loop_case.py"),
           root or "none", case, f"{wait_s:g}"]
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    # ★ 变异复现的关键：把"被测包根"同时交给**子进程**。
    #   root=None 时**继承**父进程的 `DZ_PKG_ROOT` —— 否则变异复现会变成
    #   "父进程跑变异体、子进程跑真源"的假象（实测踩过：M4 全绿 40/40）。
    eff_root = root or os.environ.get("DZ_PKG_ROOT") or None
    if eff_root:
        env["DZ_PKG_ROOT"] = eff_root
        env["PYTHONPATH"] = os.pathsep.join(
            [eff_root, str(TESTS_DIR.parent), env.get("PYTHONPATH", "")])
    else:
        env.pop("DZ_PKG_ROOT", None)
        env["PYTHONPATH"] = os.pathsep.join(
            [str(TESTS_DIR.parent), env.get("PYTHONPATH", "")])
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                              errors="replace", timeout=hard_s, env=env, cwd=str(TESTS_DIR))
    except subprocess.TimeoutExpired:
        return {"timeout": True, "exit_code": 124, "result": None,
                "stderr": f"子进程 {hard_s:g}s 未退出（僵尸态：连内层线程等待都没能收口）"}
    parsed = None
    for line in (proc.stdout or "").splitlines():
        if line.startswith("RESULT_JSON="):
            try:
                parsed = json.loads(line[len("RESULT_JSON="):])
            except Exception:                            # noqa: BLE001
                parsed = None
    return {"timeout": bool(parsed.get("timed_out")) if parsed else False,
            "exit_code": proc.returncode, "result": parsed,
            "stderr": (proc.stderr or "")[-400:]}


# --------------------------------------------------------------------------- #
section("① 信号接管：SIGINT/SIGTERM 都显式接管，处理器只置位 + 记信号名")
# 先保存现场，测完恢复（本文件是脚本，但别把宿主进程的信号处理改坏）
saved = {name: signal.getsignal(getattr(signal, name)) for name in ("SIGINT", "SIGTERM")
         if hasattr(signal, name)}
logs_1: list = []
stop_flag: dict = {"stop": False, "signal": None}
installed = M.install_stop_signals(stop_flag, rec(logs_1))
check("① 装上了 SIGINT 与 SIGTERM 两个信号", set(installed) == {"SIGINT", "SIGTERM"},
      f"got {installed}")
for name, sig in (("SIGINT", signal.SIGINT), ("SIGTERM", signal.SIGTERM)):
    _h = signal.getsignal(sig)
    # ★ O5（t9 复核 §4）：先判"拿到的是不是**我们装的处理器**"再调用。
    #   M3 变异（`_STOP_SIGNALS = ("SIGINT",)` 去掉 SIGTERM）下，`getsignal(SIGTERM)` 会返回
    #   默认处理器对象，直接 `(...)(sig, None)` 会**抛异常把整个测试进程带走** —— 能拦住门禁
    #   （exit 1）但只输出半截、丢失"后续段落是否也会红"的信息。现在改成可控 FAIL。
    check(f"① {name} 处理器已换成可调用的自定义处理器（M3：去掉接管即在此红）",
          callable(_h), f"got {_h!r}")
    if callable(_h):
        stop_flag.update(stop=False, signal=None)
        _h(sig, None)                                    # 直接调处理器（不真发信号）
        check(f"① {name} 处理器置位停机标志", stop_flag["stop"] is True)
        check(f"① {name} 处理器记下信号名（供主循环留痕）", stop_flag["signal"] == name,
              f"got {stop_flag['signal']}")
check("① 处理器内**不 print**（信号上下文里不碰 stdout 锁）", logs_1 == [], f"got {logs_1}")

# ★ 关键防回归：SIGTERM 若漏接，就会被 rclpy 的 C++ 处理器接走（只 rcl_shutdown、不退进程）
check("★ SIGTERM 在显式接管清单里（漏接 = 回到僵尸态）", "SIGTERM" in installed,
      f"got {installed}")
for name, sig in saved.items():
    signal.signal(getattr(signal, name), sig)


# --------------------------------------------------------------------------- #
section("② 主循环：上下文失效 → 立刻退出（不再空转刷 RCLError）")
# ★ O7：本段一律**子进程 + 硬超时**执行（见 run_loop_case 文档）。
#   M4 变异（去掉判活退出）下 `_live_loop` 会永久空转，子进程超时被杀 ⇒ 本段给"红"，
#   而不是把整个 CI 挂死。子进程同时回报 why/alive_calls/logs，所以信息量不减。
_r2 = run_loop_case("context_dead", wait_s=4.0)
res2 = _r2["result"] or {}
check("② 退出原因 = context_dead", res2.get("why") == "context_dead",
      f"got {res2.get('why')!r}（timeout={_r2['timeout']} exit={_r2['exit_code']}"
      f" stderr={_r2['stderr']!r}）")
check("★ ≤1 拍即退出（只问了 1 次判活）", res2.get("alive_calls") == 1,
      f"got {res2.get('alive_calls')}")
check("② 打了一条明确的上下文失效留痕",
      any("ROS 上下文已失效" in m for m in res2.get("logs", [])), f"{res2.get('logs')}")
check("② 留痕里点名了 rclpy.ok()（排障时能一眼认出判据）",
      any("rclpy.ok()=False" in m for m in res2.get("logs", [])), f"{res2.get('logs')}")
check("② 不把 FINISH 当退出原因（没到终点就是异常退出）", res2.get("why") != "finish")
check("② 单例耗时远小于超时（真退出，不是压着超时线跑）",
      isinstance(res2.get("dt"), (int, float)) and res2["dt"] < 1.0,
      f"dt={res2.get('dt')}")


# --------------------------------------------------------------------------- #
section("③ 主循环：判活正常时**不**误退；信号置位走 stop 路径并留痕")
# 同样放进子进程（同一层硬超时保护）；子进程内部计数 alive 调用次数并回报日志。
_r3 = run_loop_case("alive_normal", wait_s=4.0)
res3 = _r3["result"] or {}
check("③ 退出原因 = stop（不是 context_dead；子进程正常收口、不挂死）",
      res3.get("why") == "stop" and _r3["timeout"] is False,
      f"got {res3.get('why')!r} timeout={_r3['timeout']} err={res3.get('error')!r}")
check("③ 判活被每拍调用（说明判活确实在循环里）", (res3.get("alive_calls") or 0) >= 4,
      f"got {res3.get('alive_calls')}")
check("③ 信号留痕写明信号名 + 走了安全收尾（收栈后能追是谁发的）",
      any("SIGTERM" in m for m in res3.get("logs", []))
      and any("安全收尾" in m for m in res3.get("logs", [])), f"{res3.get('logs')}")

# 无信号的外部停机请求（例如上层直接置位）也应留痕，但不得假称收到信号
_r4 = run_loop_case("stop_flag", wait_s=4.0)
res4 = _r4["result"] or {}
check("③ 已置位时直接退出（不空转）",
      res4.get("why") == "stop" and res4.get("logs") != [],
      f"why={res4.get('why')} logs={res4.get('logs')}")
check("③ 没有信号时**不**假称信号（文案区分）",
      all("收到信号" not in m for m in res4.get("logs", [])), f"{res4.get('logs')}")


# --------------------------------------------------------------------------- #
section("④ 判活默认实现：无 ROS 环境不误杀（bench/开发机）")
check("④ ros_context_alive() 在无 rclpy 时返回 True（没有上下文可失）",
      M.ros_context_alive() is True, f"got {M.ros_context_alive()!r}")


# --------------------------------------------------------------------------- #
section("⑤ 源码级钉死：RosBridge 显式传 signal_handler_options（不用默认信号处理）")
src_io = (pathlib.Path(M.__file__).resolve().parent / "io" / "ros_io.py").read_text(
    encoding="utf-8")
check("⑤ RosBridge 调 rclpy.init 时显式传 signal_handler_options",
      "rclpy.init(signal_handler_options=" in src_io)
# ★ 用 AST 拿掉 docstring、再逐行剔掉注释，只扫**真实可执行代码**：
#   · 必须出现"取值 NO"的声明；
#   · 真实代码里**不许**出现 SignalHandlerOptions.ALL —— 出现就是把信号交回 rclpy 的
#     C++ 处理器（`install_signal_handlers`），僵尸态原样复发。
#   （文件/函数 docstring 里写 "…SignalHandlerOptions.ALL…" 是根因说明，AST 之后不再命中。）
_tree = ast.parse(src_io)
for _node in ast.walk(_tree):
    if isinstance(_node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
        _d = ast.get_docstring(_node, clean=False)
        if _d:
            src_io = src_io.replace(_d, "", 1)
_body = "\n".join(l for l in src_io.splitlines() if not l.lstrip().startswith("#"))
check("⑤ 真实代码里 SignalHandlerOptions.ALL 一个都不许出现",
      "SignalHandlerOptions.ALL" not in _body,
      f"got {[l.strip() for l in _body.splitlines() if 'SignalHandlerOptions.ALL' in l]}")
check("⑤ 真实代码里出现取值 NO（SignalHandlerOptions.NO 或 getattr(opts, 'NO')）",
      ("SignalHandlerOptions.NO" in _body) or ('"NO"' in _body), "no NO token")
src_main = pathlib.Path(M.__file__).read_text(encoding="utf-8")
_s = src_main.split("_live_loop")
check("⑤ 源码里**调用**过 _live_loop（不是只有定义；t14 新增）",
      len(_s) >= 3 and "(" in _s[1], f"occurrences={len(_s) - 1}")

# --------------------------------------------------------------------------- #
section("⑥ 真发一次 SIGTERM：显式接管必须真的接住（不是只在测试里调处理器）")
if sys.platform == "win32":
    # ★ Windows 的 `os.kill(pid, SIGTERM)` 走的是 TerminateProcess（**不经过** Python 的
    #   信号处理器，进程被直接杀、退出码 15）—— 这是平台事实，不是实现缺陷，故跳过真发信号，
    #   只断言"处理器已装上"（POSIX/车端会真发，见 else 分支）。
    check("⑥ Windows 平台跳过真发 SIGTERM（os.kill 走 TerminateProcess，不经处理器）",
          True, f"platform={sys.platform}")
    check("⑥ 但仍确认 SIGTERM 已显式接管（车端 POSIX 上真发即走这条）",
          "SIGTERM" in installed, f"got {installed}")
else:
    _stop_real: dict = {"stop": False, "signal": None}
    M.install_stop_signals(_stop_real, logs_1.append)
    os.kill(os.getpid(), signal.SIGTERM)      # ★ 真的给本进程发 SIGTERM
    check("★ 真 SIGTERM 被我们的处理器接住（进程还活着且标志已置位）",
          _stop_real["stop"] is True, f"got {_stop_real}")
    check("★ 记下的信号名正是 SIGTERM", _stop_real["signal"] == "SIGTERM",
          f"got {_stop_real['signal']}")
for name, sig in saved.items():
    signal.signal(getattr(signal, name), sig)


# --------------------------------------------------------------------------- #
section("⑦ RosBridge：init 显式传 NO + 上下文失效时的刹车/收尾行为（替身 rclpy）")
# ★ 用**替身 rclpy** 实例化真 `RosBridge`（这是"只看逻辑、不连 ROS"的唯一干净做法）：
#   替身暴露 init/ok/shutdown/executor/node + 假消息类型，其余逻辑全是真代码。
_had_rclpy = "rclpy" in sys.modules
_saved_mods = {m: sys.modules.get(m) for m in
               ("rclpy", "rclpy.node", "rclpy.qos", "rclpy.executors", "geometry_msgs",
                "geometry_msgs.msg", "std_msgs", "std_msgs.msg", "sensor_msgs",
                "sensor_msgs.msg", "nav_msgs", "nav_msgs.msg")}

class _Msg:
    """最小消息替身：任意属性名都能赋值/读取（Twist 的 linear.x 等）。"""

    def __init__(self) -> None:
        self._o = {}

    def __getattr__(self, n):
        if n.startswith("_"):
            raise AttributeError(n)
        return self._o.setdefault(n, _Msg())

    def __setattr__(self, n, v):
        if n == "_o":
            super().__setattr__(n, v)
        else:
            self._o[n] = v


class _Node:
    def __init__(self, name: str) -> None:
        self.name = name
        self.pubs: dict = {}

    def create_subscription(self, *_a, **_k):
        return object()

    def create_publisher(self, msg_type, topic, qos):
        self.pubs[topic] = (msg_type, _Pub())
        return self.pubs[topic][1]

    def create_timer(self, *_a, **_k):
        return object()

    def get_clock(self):
        return object()

    def destroy_node(self):
        self.destroyed = True


class _Pub:
    def __init__(self) -> None:
        self.n = 0

    def publish(self, _msg) -> None:
        self.n += 1


class _Ctx:
    """rclpy 替身：记录 init 的关键字、可切 ok()、可记 shutdown。"""

    def __init__(self) -> None:
        self.init_kwargs: list = []
        self.ok_flag = True
        self.shutdowns = 0
        self.signals = types.SimpleNamespace(SignalHandlerOptions=types.SimpleNamespace(
            NO="NO", ALL="ALL", SIGINT_ONLY="SIGINT_ONLY"))

    def init(self, *a, **kw):
        self.init_kwargs.append(kw)

    def ok(self) -> bool:
        return self.ok_flag

    def shutdown(self, *a, **kw):
        self.shutdowns += 1


_FEM = {
    "Twist": type("Twist", (_Msg,), {}), "Int32": type("Int32", (_Msg,), {}),
    "Int32MultiArray": type("Int32MultiArray", (_Msg,), {}),
    "String": type("String", (_Msg,), {}), "UInt8": type("UInt8", (_Msg,), {}),
    "UInt8MultiArray": type("UInt8MultiArray", (_Msg,), {}),
    "Float32": type("Float32", (_Msg,), {}),
    "Odometry": type("Odometry", (_Msg,), {}), "Imu": type("Imu", (_Msg,), {}),
}


def _install_rclpy_stub(ctx: _Ctx, *, with_signals: bool = True) -> None:
    """把关掉的 rclpy/消息系统塞进 sys.modules（`RosBridge` 只会 import 名字，不校验）。"""
    g = types.ModuleType("geometry_msgs.msg")
    for _n, _cls in _FEM.items():
        setattr(g, _n, _cls)
    s = types.ModuleType("std_msgs.msg")
    for _n in ("Int32", "Int32MultiArray", "String", "UInt8", "UInt8MultiArray", "Float32"):
        setattr(s, _n, _FEM[_n])
    sg = types.ModuleType("sensor_msgs.msg")
    sg.Imu = _FEM["Imu"]
    nv = types.ModuleType("nav_msgs.msg")
    nv.Odometry = _FEM["Odometry"]
    sys.modules.update({
        "geometry_msgs": types.ModuleType("geometry_msgs"), "geometry_msgs.msg": g,
        "std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": s,
        "sensor_msgs": types.ModuleType("sensor_msgs"), "sensor_msgs.msg": sg,
        "nav_msgs": types.ModuleType("nav_msgs"), "nav_msgs.msg": nv,
    })
    rm = types.ModuleType("rclpy")
    rm.init, rm.ok, rm.shutdown = ctx.init, ctx.ok, ctx.shutdown
    if with_signals:
        rm.signals = ctx.signals          # 没有这个属性 = 老/假 rclpy（走"不传参数"分支）
    rm.Node = _Node
    rn = types.ModuleType("rclpy.node")
    rn.Node = _Node
    rq = types.ModuleType("rclpy.qos")
    rq.QoSProfile = type("QoSProfile", (), {"__init__": lambda self, **kw: None})
    rq.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rq.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2)
    rq.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1, TRANSIENT_LOCAL=2)
    rex = types.ModuleType("rclpy.executors")
    rex.MultiThreadedExecutor = type("MultiThreadedExecutor", (), {
        "__init__": lambda self, **kw: None,
        "add_node": lambda self, n: None, "spin": lambda self: None,
        "shutdown": lambda self, **kw: None})
    sys.modules.update({"rclpy": rm, "rclpy.node": rn, "rclpy.qos": rq,
                        "rclpy.executors": rex})


try:
    from dzbrain.io.ros_io import RosBridge                                 # noqa: E402
    ctx = _Ctx()
    ctx.ok_flag = False                      # ← 让 RosBridge 走 rclpy.init 分支
    _install_rclpy_stub(ctx)
    logs_7: list = []
    br = RosBridge("dzbrain-test", log=logs_7.append)
    check("⑦ rclpy.init 被调用且显式传了 signal_handler_options",
          bool(ctx.init_kwargs) and "signal_handler_options" in ctx.init_kwargs[0],
          f"got {ctx.init_kwargs}")
    check("★ 传的值是 SignalHandlerOptions.NO（不是 ALL/None）",
          bool(ctx.init_kwargs) and ctx.init_kwargs[0].get("signal_handler_options") == "NO",
          f"got {ctx.init_kwargs[0] if ctx.init_kwargs else None}")
    check("⑦ 顺带确认没有用默认信号处理（ALL 会重新交给 C++ rcl_shutdown）",
          all(kw.get("signal_handler_options") != "ALL" for kw in ctx.init_kwargs),
          f"got {ctx.init_kwargs}")

    # —— 上下文活着：刹车真的发出去 ——
    ctx.ok_flag = True
    br.publish_brake()
    check("⑦ 上下文正常时刹车发出去了（zero_commands=1）",
          br.stats.zero_commands == 1, f"got {br.stats.zero_commands}")

    # —— 上下文死了：publish_brake 必须抛（Safety 靠这个识别"没发出去"）——
    ctx.ok_flag = False
    raised = ""
    try:
        br.publish_brake()
        raised = "（没有抛！）"
    except Exception as exc:                    # noqa: BLE001
        raised = type(exc).__name__
    check("★ 上下文失效时 publish_brake 抛异常（旧实现会静默成功）",
          raised != "（没有抛！）" and raised != "", f"got {raised}")
    check("⑦ 失效时不再累加 zero_commands（没发出去就不算发）",
          br.stats.zero_commands == 1, f"got {br.stats.zero_commands}")

    # —— 收尾：上下文已死 → 跳过刹车、留痕、返回 rc=1 ——
    _logs_before = len(logs_7)
    rc = br.shutdown(brake=True)
    check("★ 上下文已死时 shutdown 返回 rc=1（调用方据此写 teardown 留痕）", rc == 1,
          f"got {rc}")
    check("⑦ 跳过刹车时有明确留痕（说明不再刷 RCLError）",
          any("跳过刹车指令" in m for m in logs_7[_logs_before:]), f"{logs_7[_logs_before:]}")

    # —— 上下文正常时收尾：照旧先刹车、rc=0 ——
    ctx2 = _Ctx()
    ctx2.ok_flag = False
    _install_rclpy_stub(ctx2)
    br2 = RosBridge("dzbrain-test2", log=lambda m: None)
    ctx2.ok_flag = True                      # 初始化完成 ⇒ 上下文活着
    rc2 = br2.shutdown(brake=True)
    check("⑦ 上下文正常时 shutdown 先刹车（zero_commands=1）且 rc=0",
          br2.stats.zero_commands == 1 and rc2 == 0,
          f"zero={br2.stats.zero_commands} rc={rc2}")
    check("⑦ 正常收尾确实调了 rclpy.shutdown（不留上下文）", ctx2.shutdowns == 1,
          f"got {ctx2.shutdowns}")

    # —— 老/假 rclpy 上没有 signals：不得因一个可选开关把启动搞挂 ——
    #   （注意 `rclpy.ok()` 现在是 True ⇒ 再建桥不会调 init；先把它置假来走初始化分支）
    ctx3 = _Ctx()
    ctx3.ok_flag = False
    _install_rclpy_stub(ctx3, with_signals=False)
    br3 = RosBridge("dzbrain-test3", log=lambda m: None)
    check("⑦ 取不到 SignalHandlerOptions 时桥照样建起来（不抛）", br3.node is not None)
    check("★ 此路径**不传** signal_handler_options"
          "（传 None 等于又把默认处理器装回来，与修复目的相反）",
          ctx3.init_kwargs == [{}], f"got {ctx3.init_kwargs}")
finally:
    for _m, _old in _saved_mods.items():
        if _old is None:
            sys.modules.pop(_m, None)
        else:
            sys.modules[_m] = _old
    # 真 rclpy 从未 import 过（本机无 ROS）→ 顺手别把替身留在 sys.modules 里
    if not _had_rclpy:
        sys.modules.pop("rclpy", None)


finish()
