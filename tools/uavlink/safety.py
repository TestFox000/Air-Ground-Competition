# uavlink.safety —— 由 tools/uav_link.py 拆出（危险动作安全闸门（确认词/倒计时/高度夹限））。与原文件逐行一致（AST 校验过）。
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import socket
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path
from uavlink.config import COMMAND_MAP, COUNTDOWN_ACTIONS, COUNTDOWN_S, WORD_CONFIRM, YES_REQUIRED

# ---------------------------------------------------------------- 纯函数：安全闸门 / 倒计时

def check_gate(action: str, yes: bool) -> str | None:
    """安全闸门：返回 None=放行；否则返回拒绝原因（调用方打印后退出）。

    arm/takeoff/stack-restart/kill 属高风险，必须显式 --yes——默认什么都不做。
    kill 在 --yes 之外另有全词确认（confirm_word，WORD_CONFIRM）。
    """
    if action in YES_REQUIRED and not yes:
        extra = "，且 kill 还要再输入全词 'kill' 确认" if action == "kill" else ""
        return (f"动作 {action} 属高风险，必须显式加 --yes 才会执行{extra}"
                f"（默认什么都不做）。确认无人机周围安全后重试。")
    return None
def confirm_word(action: str, ask=input, out=print, is_tty=None) -> bool:
    """全词确认闸门（kill 专用）：--yes 之外还必须逐字输入全词才放行。

    对齐 flight_console.py:619 ask_confirm("kill", ...) 的强度：倒计时挡不住
    shell 历史肌肉记忆（`ctl land --yes` 上翻一格就是 `ctl kill --yes`），
    只有"手动敲出全词"能证明操作员此刻想的就是这个动作。
    无交互终端时拒绝执行（没法确认 = 默认什么都不做）。
    """
    word = WORD_CONFIRM.get(action)
    if word is None:
        return True
    if is_tty is None:
        is_tty = sys.stdin.isatty()
    if not is_tty:
        out(f"[拒绝] {action} 需要交互终端逐字输入 '{word}' 确认"
            f"（当前没有交互终端，默认什么都不做）")
        return False
    out("!" * 60)
    out(f"!! {action} = {COMMAND_MAP[action]['desc']}")
    out(f"!! 这是最后手段：逐字输入 {word} 回车才会执行，其他任何输入都取消")
    out("!" * 60)
    try:
        typed = ask(f"输入 '{word}' 确认: ")
    except (EOFError, KeyboardInterrupt):
        out("\n[中止] 确认被中断，未执行")
        return False
    if typed.strip() != word:
        out(f"[中止] 输入 {typed.strip()!r} ≠ {word!r}，未执行")
        return False
    return True
def clamp_alt(alt: float) -> float:
    """takeoff --alt 夹到 0.5~10m：过低撞地效区/过高超侦察剖面，现场手滑常见。"""
    return min(10.0, max(0.5, float(alt)))
def countdown(action: str, seconds: int = COUNTDOWN_S, yes: bool = False,
              out=print, sleep=time.sleep) -> None:
    """次高风险动作执行前的醒目警告 + 逐秒倒计时；--yes 跳过（out/sleep 可注入便于测试）。"""
    if yes or action not in COUNTDOWN_ACTIONS:
        return
    out("!" * 60)
    out(f"!! 即将执行 {action}（{COMMAND_MAP[action]['desc']}）")
    out(f"!! {seconds} 秒内 Ctrl-C 可中止")
    out("!" * 60)
    for i in range(seconds, 0, -1):
        out(f"   {i} …")
        sleep(1.0)
