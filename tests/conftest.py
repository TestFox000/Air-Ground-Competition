"""测试夹具：把 src/ag_integrate 加入 sys.path，使 ag_integrate 可导入。"""

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent.parent / "src" / "ag_integrate"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
