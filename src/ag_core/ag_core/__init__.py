"""ag_core：与 ROS 无关的复盘工具（黑匣子记录 + 日志诊断）。

2026-10-01 按官方技术总框架瘦身：侦察/规划/开车均由官方栈与 ag_integrate 承担，
本包只保留实车复盘用的 blackbox 与 log_diagnosis。
"""

__version__ = "0.1.0"

from .blackbox import BlackBox

__all__ = ["BlackBox"]
