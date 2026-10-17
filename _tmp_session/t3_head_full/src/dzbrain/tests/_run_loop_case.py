# -*- coding: utf-8 -*-
r"""`_live_loop` 单例**可超时**运行器（test_shutdown.py 的子进程哨兵）。

为什么必须单独一个进程（O7，t9 复核 §4）：
    M4 变异（把 `if not alive():` 改成永不成立 = 去掉判活退出）会让 `_live_loop`
    **永久空转** —— 那正是 2026-10-11 现场被 SIGTERM 后的僵尸态。若在测试进程里直接调用，
    测试不是"红"而是"僵"（t9 实测 5 分钟不结束，只能靠外部看门狗拿结论）⇒ CI 会被卡死。
    本运行器把该调用放进**子进程**：父测试用 `subprocess.run(timeout=...)` 兜底，
    超时即杀、判为"红（timeout）"。内部再套一层线程 `join(timeout)`，让**未超时**的调用
    能回报详细日志（子进程一旦被杀，其 stdout 就没人收了）。

用法（参数一律 ASCII，便于跨机器录证据）：
    python _run_loop_case.py <root> <case> [timeout_s]
      <root>      含 `dzbrain/` 的包根目录；`none` = 用本测试进程的 sys.path
                  （即"被测真源"：src/dzbrain 运行树）
      <case>      context_dead | alive_normal | stop_flag
      [timeout_s] 子进程内部线程等待上限，默认 5.0（小于父侧 subprocess 超时）

输出：**最后一行**是一条 JSON（`RESULT_JSON=<json>`），键见 `payload`：
    exit / why / alive_calls / logs / dt（秒）/ timed_out / error
退出码：0 = 正常产出 payload；9 = 运行器自身异常（payload.error 有原因）
"""

from __future__ import annotations

import json
import pathlib
import sys
import threading
import time
import traceback

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

DEFAULT_WAIT_S = 5.0


def load_main(root: pathlib.Path | None):
    """加载 main 模块。

    ★ 先由调用方 import **同包的** `dzbrain.core.blackboard`：`dzbrain/main.py` 用的是
      **相对 import**（`from .core.blackboard import ...`），只有包已就绪时才成立；
      而含 `$root/../tests` 的 PYTHONPATH 足以让父测试进程跑起来、却不足以让"包内相对 import"
      在**新进程**里解析。
    """
    if root is not None and str(root) not in sys.path:
        sys.path.insert(0, str(root))
    import importlib
    return importlib.import_module("dzbrain.main")


def payload(**kw) -> dict:
    base = {"exit": None, "why": None, "alive_calls": None, "logs": [],
            "dt": None, "timed_out": False, "error": ""}
    base.update(kw)
    return base


def main(argv: list[str]) -> int:
    root_arg = argv[1] if len(argv) > 1 else "none"
    case = argv[2] if len(argv) > 2 else "context_dead"
    wait_s = float(argv[3]) if len(argv) > 3 else DEFAULT_WAIT_S
    root = None if root_arg.lower() in ("none", "-", "") else pathlib.Path(root_arg).resolve()

    out = payload()
    try:
        from dzbrain.core.blackboard import Blackboard
        if root is not None:
            M = load_main(root)
        else:
            import dzbrain.main as M
    except Exception:
        out["error"] = traceback.format_exc(limit=3).strip().splitlines()[-1]
        print("RESULT_JSON=" + json.dumps(out, ensure_ascii=False))
        return 0

    logs: list = []
    calls = {"n": 0}

    def _dead() -> bool:
        calls["n"] += 1
        return False

    def _stop_flag() -> dict:
        return {"stop": True, "signal": None}

    def _alive_then_break() -> bool:
        """前 3 拍判活为真；第 4 拍起置停机标志（模拟信号处理器在下一拍介入）。"""
        calls["n"] += 1
        if calls["n"] >= 4:
            flag["stop"] = True
            flag["signal"] = "SIGTERM"
        return True

    if case == "context_dead":
        flag = {"stop": False, "signal": None}
        alive = _dead
    elif case == "stop_flag":
        flag = _stop_flag()
        alive = lambda: True                                    # noqa: E731
    else:                                                       # alive_normal
        case = "alive_normal"
        flag = {"stop": False, "signal": None}
        alive = _alive_then_break

    holder = {"why": None, "t0": time.perf_counter()}

    def _run() -> None:
        holder["why"] = M._live_loop(flag, bb=Blackboard(), alive=alive,
                                     log=logs.append, poll_s=0.0, finish_wait_s=0.0)

    th = threading.Thread(target=_run, name="live-loop", daemon=True)
    th.start()
    th.join(wait_s)
    out["timed_out"] = th.is_alive()
    if out["timed_out"]:
        out["why"] = None
        out["error"] = f"_live_loop 在 {wait_s:g}s 内未返回（僵尸态：判活退出被判据删除）"
    else:
        out["why"] = holder["why"]
    out["dt"] = round(time.perf_counter() - holder["t0"], 4)
    out["alive_calls"] = calls["n"]
    out["logs"] = list(logs)
    out["exit"] = "context_dead" if case == "context_dead" else (
        "stop" if case == "stop_flag" else "alive_normal")
    print("RESULT_JSON=" + json.dumps(out, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
