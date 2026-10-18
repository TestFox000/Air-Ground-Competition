# -*- coding: utf-8 -*-
"""dznav：项目自有的循迹执行栈（发学员/官方 executor 的进化后继）。

模块：
- contract  场地/底盘常量、坐标变换、话题与命令语法
- servo     舵机-曲率模型（理想/实测不对称/标定表三档案）
- refpath   参考路径生成（曲率受限圆角 + ARC 机动兜底）
- pursuit   统一跟踪控制器（前馈+反馈+限速+恢复）
- fusion    定位融合（距离 DR + 航向锚 + 偏差自学习）
- executor_node  ROS 节点（契约兼容，可替换 legacy 栈）
"""

__version__ = "1.0.0"
