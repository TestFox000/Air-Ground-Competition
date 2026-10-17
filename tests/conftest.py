"""测试夹具：把 src/ag_integrate 与 src/ag_core 加入 sys.path，使两包可导入。"""

import os
import sys
import tempfile
from pathlib import Path

import pytest

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


@pytest.fixture(autouse=True)
def _ros_stub_globals_cleanup():
    """兜底收尾：每个测试结束后无条件复位 ros_stub 的三个全局单例。

    为什么存在（2026-10-17 横切扫描）：ros_stub 的 SIM（仿真时钟）/ BUS（topic
    总线）/ _OVERRIDES（参数覆盖表）是进程级单例，而仿真链路并非都经 fixture
    清理——tests/sim_car.py 的 TrackingWorld.__init__ 直接 BUS.reset() +
    SIM.enable(0.0)，不经 fixture 就会把"时钟已启用/总线已打开"泄漏给后续
    用例（pytest-randomly 随机序下这类污染表现为偶发假阳性，极难定位）。
    这里统一兜底：三个清理调用全是幂等操作（disable/clear 重复执行无副作用），
    与 test_sim_tracking.py 的 world_factory 自带收尾并存不冲突——autouse
    夹具后装先拆的反向次序保证它是**最后**一个收尾，状态必然归零。
    """
    yield
    import ros_stub  # 延迟导入：conftest 自身导入零副作用，跑首个测试时才装 rclpy 桩
    ros_stub.BUS.disable()
    ros_stub.SIM.disable()
    ros_stub.clear_overrides()
