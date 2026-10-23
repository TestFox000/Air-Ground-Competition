#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/ 共享场地常量 —— 单一真源 = `src/dzbrain/dzbrain/core/geometry.py`。

为什么需要这个文件
--------------------------------------------------------------------------------
`global_planner.py` / `route_check.py` / `shoot_planner.py` 都是**独立脚本**：
车上不 `pip install dzbrain` 也要能 `python tools/xxx.py` 直接跑。若每个脚本自己抄一份
`COL_LETTERS / MAP_COLS / MAP_ROWS / BLOCK_SIZE / BARRIER_* / MAP_X*_MIN..MAX`（共 4 份），
副本一旦漂移，就会出现"规划器按 14 列、校验器按 13 列"
这种只能在车上现场才发现的鬼故事（`docs/survey_path_executor.md` 的 K13 教训）。

本模块把「脚本自举」收敛到**一处**：把 `src/dzbrain` 插到 `sys.path` 首位后，
从 `core.geometry` **取值再导出**（不复制任何数字）。脚本侧用法：

    from _field_const import (COL_LETTERS, MAP_COLS, MAP_ROWS, BLOCK_SIZE,
                              BARRIER_ROW_HI, BARRIER_ROW_LO, BARRIER_COL_FROM)

运行口径（硬约束）
--------------------------------------------------------------------------------
* **不需要 PYTHONPATH**：本文件自己按 `__file__` 推导 `src/dzbrain` 并插入 `sys.path`。
* **任意 cwd 可跑**：所有路径从 `__file__` 解析，与当前工作目录无关；`tools/` 也
  不必在 `sys.path` 里（脚本直跑时 Python 自动加，被别处 import 时本文件会补）。
* **不新增包依赖**：只 import `dzbrain.core.geometry`（纯 math/dataclasses，无 ROS、
  无 yaml、无第三方库）。

★ import 顺序陷阱（`tools/plan_route.py:66` 踩过）：`python tools/xxx.py` 会把 `tools/`
塞进 `sys.path[0]`，而那里躺着一个**同名模块** `tools/dzbrain.py`（ROS 包装脚本）——
于是 `import dzbrain` 会解析成那个脚本而不是真包。所以本模块在 import dzbrain 之前
先把 `src/dzbrain` 插到首位，并把 `tools/` / 工作区根从搜索路径里摘掉。
"""

from __future__ import annotations

import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve().parent       # .../tools
_WS = _HERE.parent                                    # 仓库根（dzrobot_v2/）
_SRC = _WS / "src" / "dzbrain"                        # dzbrain 源码树

for _entry in (str(_HERE), str(_WS)):
    while _entry in sys.path:
        sys.path.remove(_entry)
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from dzbrain.core.geometry import (                   # noqa: E402
    BARRIER_COL_FROM,
    BARRIER_ROW_HI,
    BARRIER_ROW_LO,
    BLOCK_SIZE,
    COL_LETTERS,
    MAP_COLS,
    MAP_ROWS,
    MAP_X_MAX,
    MAP_X_MIN,
    MAP_Y_MAX,
    MAP_Y_MIN,
)

__all__ = [
    "COL_LETTERS", "MAP_COLS", "MAP_ROWS", "BLOCK_SIZE",
    "BARRIER_ROW_HI", "BARRIER_ROW_LO", "BARRIER_COL_FROM",
    "MAP_X_MIN", "MAP_X_MAX", "MAP_Y_MIN", "MAP_Y_MAX",
]
