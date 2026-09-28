"""ag_mission：车端任务规划 / 敌我识别决策。

与 ag_core 的分工：
    本包只做「ROS 消息 <-> 算法核心」的搬运与状态管理，
    所有可测逻辑放在 ag_core 或本包的 field_fusion（无 ROS 依赖）。
"""

__version__ = "0.1.0"
