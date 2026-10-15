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

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"

# ---------------------------------------------------------------------------
# 循迹链路 SIL 仿真（tests/sim_car.py + tests/test_sim_tracking.py）保真边界：
# 验证的是"契约 + 控制逻辑"（命令语义、dzactuator 模式仲裁、对齐状态机、
# reset 断言偏置的画圆机理），不是动力学参数辨识 —— 理想定位、无滑移、
# 速度瞬时到位、串口打包与速度平滑不建模。详见 sim_car.py 文件头。
# ---------------------------------------------------------------------------

# ROS2 的 Python 包采用 src/<pkg>/<pkg>/ 结构，因此每个包目录都要单独入 path，
# 才能在 Windows 上直接 import ag_core / ag_mission / ag_referee / ag_bridge。
paths = [SRC]
if SRC.is_dir():
    paths += sorted(p for p in SRC.iterdir() if p.is_dir())

for p in paths:
    s = str(p)
    if s not in sys.path:
        sys.path.insert(0, s)
