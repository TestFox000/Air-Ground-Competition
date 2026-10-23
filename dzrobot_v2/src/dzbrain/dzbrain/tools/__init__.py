# -*- coding: utf-8 -*-
"""tools：离线工具（**不 import rclpy**）。

- `sim`     —— 纯追踪离线仿真器（自行车模型 + 闭环 + 误差注入 + CSV/SVG）
- `sim_cli` —— 仿真器命令行入口（工作区 `tools/sim_pursuit.py` 只是它的薄包装）

设计意图：把"改参数 → 上车 → 看跑偏"的循环压到本地几秒内，
让 `pursuit.*` 的每个键都能在没有车的情况下做**回归对照实验**
（横向误差 / 超调 / 舵角速率 / 锚定次数，见 `docs/P0接口约定.md` §5）。
"""

from __future__ import annotations

__all__: list[str] = []
