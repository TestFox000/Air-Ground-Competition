"""测试夹具：把 src/ag_integrate 与 src/ag_core 加入 sys.path，使两包可导入。"""

import os
import sys
import tempfile
from pathlib import Path

# 黑匣子日志：测试里**强制**重定向到临时目录。
# 必须用赋值而不是 setdefault：现场会把 AG_LOG_DIR 设成真实日志目录（推荐做法），
# 那样 setdefault 就失效了，一跑测试就把上百个 node.ready / session.begin
# 塞进要复盘的那批日志里，报告直接被污染（实测踩过）。
os.environ["AG_LOG_DIR"] = str(Path(tempfile.gettempdir()) / "ag_logs_test")
os.environ["AG_LOG_SESSION"] = "pytest"

for _pkg in ("ag_integrate", "ag_core"):
    _SRC = Path(__file__).resolve().parent.parent / "src" / _pkg
    if str(_SRC) not in sys.path:
        sys.path.insert(0, str(_SRC))
