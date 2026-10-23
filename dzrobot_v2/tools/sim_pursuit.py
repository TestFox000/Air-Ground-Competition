#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""工作区薄包装：`python tools\\sim_pursuit.py ...` → `dzbrain.tools.sim_cli:main()`。

存在的理由：车上/车下都不装 `dzbrain` 包（只 colcon build 进 install 树），
而离线调参要在**工作区根**直接敲命令。这里只做一件事：把 `src/dzbrain` 挂进
`sys.path`，然后把参数原样转交给 `dzbrain.tools.sim_cli.main()`。

⚠️ 本文件**没有**任何仿真逻辑 —— 逻辑全在 `src/dzbrain/dzbrain/tools/sim.py`，
   免得"包装脚本里偷偷改了一个参数"这种最难查的分叉。
"""

from __future__ import annotations

import os
import pathlib
import sys

# 纯脚本直跑：把 src/dzbrain 挂进 sys.path，再 import dzbrain.*
_HERE = pathlib.Path(__file__).resolve()
_SRC = _HERE.parents[1] / "src" / "dzbrain"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

# ⚠️ Windows 控制台默认 GBK，指标表里的 °/×/⚠️ 编不出来会打断输出：
#    降级成替换字符，保证"仿真结果"永远不被编码问题掩盖。
try:
    sys.stdout.reconfigure(errors="replace")   # type: ignore[union-attr]
except Exception:                              # pragma: no cover
    pass

from dzbrain.tools.sim_cli import main         # noqa: E402

if __name__ == "__main__":
    sys.exit(main(os.sys.argv[1:]))
