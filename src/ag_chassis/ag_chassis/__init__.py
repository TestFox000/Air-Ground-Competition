"""ag_chassis：底盘执行层。

对外只认两个契约：
    订阅 /mission/waypoints（JSON）-> 走
    发布 /mission/task_done（格号）-> 回报

底盘驱动协议对接放在 chassis_link，仿真模式下按 eta 计时推进，
实机拿到底盘协议后只改这一处。
"""

__version__ = "0.1.0"
