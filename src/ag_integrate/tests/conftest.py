# -*- coding: utf-8 -*-
"""tests 公共引导：包路径注入 + Windows readline 桩。

1) sys.path 注入 src/ag_integrate（ROS 包布局：包实体在 src/<pkg>/<pkg>/），
   测试可直接 `from ag_integrate.xxx import ...`，各测试文件不再各写一份。
2) Windows + conda 的 pyreadline3 在无控制台 shell（CI/后台任务/管道）下
   `import readline` 会无限阻塞（2026-10-26 本机实测：pytest 零输出 rc=124）。
   预置空模块挡住该 import——pytest/测试链路在 Windows 上无真实 readline
   消费者，Linux（机上）不受影响。
"""
import sys
import types
from pathlib import Path

_PKG_DIR = Path(__file__).resolve().parents[1]
if str(_PKG_DIR) not in sys.path:
    sys.path.insert(0, str(_PKG_DIR))

if sys.platform == "win32" and "readline" not in sys.modules:
    sys.modules["readline"] = types.ModuleType("readline")
