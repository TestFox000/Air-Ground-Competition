"""测试夹具：把 src/ag_integrate 与 src/ag_core 加入 sys.path，使两包可导入。"""

import sys
from pathlib import Path

for _pkg in ("ag_integrate", "ag_core"):
    _SRC = Path(__file__).resolve().parent.parent / "src" / _pkg
    if str(_SRC) not in sys.path:
        sys.path.insert(0, str(_SRC))
