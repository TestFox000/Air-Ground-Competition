# uavlink.blackbox —— 由 tools/uav_link.py 拆出（黑匣子 NDJSON 写入/读取）。与原文件逐行一致（AST 校验过）。
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

class BlackboxWriter:
    """黑匣子写入器：每行 NDJSON，盖本机接收时刻 recv，逐行 flush（断电也尽量少丢）。

    ⚠️ 必须 newline="\\n"：Windows 默认会把 \\n 翻译成 \\r\\n（remote_run.py 踩过的坑）。
    ⚠️ 写盘失败（盘满/权限）响亮警告一次后停用黑匣子、遥测照常——
       对齐 orin_flight_blackbox 的"落盘失败绝不拖死节点"语义（M2）：
       黑匣子是证据通道，绝不允许它杀死遥测主链路。
    """

    def __init__(self, path, append: bool = False):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, "a" if append else "w",
                        encoding="utf-8", newline="\n")
        self._disabled = False     # 写盘失败后置 True，后续 write 变空操作

    def write(self, row: dict, recv: float | None = None) -> None:
        if self._disabled:
            return
        out = {"recv": time.time() if recv is None else float(recv)}
        out.update(row)
        try:
            self._fh.write(json.dumps(out, ensure_ascii=False) + "\n")
            self._fh.flush()
        except OSError as e:
            # 盘满/权限问题：响亮警告一次，停用黑匣子继续遥测（不再刷屏）
            print(f"[警告] 黑匣子写盘失败（{type(e).__name__}: {e}）——"
                  f"黑匣子已停用，遥测继续；尽快检查磁盘空间/权限: {self.path}",
                  file=sys.stderr)
            self._disabled = True
            try:
                self._fh.close()
            except Exception:  # noqa: BLE001 - 收尾异常不掩盖主流程
                pass

    def close(self) -> None:
        if self._disabled:
            return
        try:
            self._fh.close()
        except Exception:  # noqa: BLE001 - 收尾阶段异常不掩盖主流程
            pass
def read_blackbox(path) -> list[dict]:
    """读黑匣子 jsonl → 原始行列表；坏行容错跳过（文件可能被 Ctrl-C 截尾）。"""
    rows: list[dict] = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or not line.startswith("{"):
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(obj, dict):
                rows.append(obj)
    return rows
