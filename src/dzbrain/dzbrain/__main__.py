# -*- coding: utf-8 -*-
r"""`python -m dzbrain` 的包入口 —— 只做转发，逻辑一律留在 `main.py`。

★ 为什么必须有这个文件：
    文档、`tools/` 与验收命令里用的都是 `python -m dzbrain ...`，而**缺 `__main__.py`
    时该命令直接失败**（`No module named dzbrain.__main__; 'dzbrain' is a package and
    cannot be directly executed`）—— 也就是说 `--plan-only` / `--dry-run` 这些入口
    在"命令行怎么打"这一层根本不可达。

用法（与 `python -m dzbrain.main` 完全等价）：
    python -m dzbrain --plan-only        # 只规划，打印 token + 耗时，不起线程/不发指令
    python -m dzbrain --dry-run          # 只做启动门禁（配置 + 路线校验）
    python -m dzbrain --bench --seconds 5
"""

from __future__ import annotations

from .main import main

if __name__ == "__main__":
    raise SystemExit(main())
