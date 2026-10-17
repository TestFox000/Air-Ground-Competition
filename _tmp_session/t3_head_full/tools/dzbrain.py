# -*- coding: utf-8 -*-
r"""工作区薄包装：本地直接跑新决策层（不用装 colcon 包）。

用法（在 <工作区> 目录下）：
    python tools\dzbrain.py --dry-run                # 启动门禁：配置 + 路线校验
    python tools\dzbrain.py --bench --seconds 5      # 无 ROS/无车联调（真实任务线程 + FakeBridge）
    python tools\dzbrain.py                          # 上车运行（需要 source ROS + install）

车上正式运行走 colcon 安装后的入口：`ros2 run dzbrain dzbrain_node` 或直接
`python -m dzbrain.main`（install 树已在 PYTHONPATH 里）。
"""

from __future__ import annotations

import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve()
_SRC = _HERE.parents[1] / "src" / "dzbrain"
sys.path.insert(0, str(_SRC))

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass

from dzbrain.main import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
