"""train_release_pipeline 纯逻辑测试：训练完成判定状态机 + 文件文本等待。

不碰真实训练/模型/子进程链路（那些在训练机冒烟）。
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools" / "ml"))

from train_release_pipeline import (  # noqa: E402
    IDLE_S,
    decide_wait_state,
    last_epoch,
    wait_for_file_text,
)


def _write_results(path: Path, epochs: list[int]) -> None:
    path.write_text("epoch,loss\n" + "".join(f"{e},0.1\n" for e in epochs),
                    encoding="utf-8")


def test_last_epoch_parsing(tmp_path):
    assert last_epoch(tmp_path / "nope.csv") is None
    p = tmp_path / "results.csv"
    p.write_text("epoch,loss\n", encoding="utf-8")   # 只有表头
    assert last_epoch(p) is None
    _write_results(p, [0, 1, 2, 39])
    assert last_epoch(p) == 39
    p.write_text("epoch,loss\n1,0.2\n坏行\n", encoding="utf-8")
    assert last_epoch(p) is None   # 坏行不炸，返回 None


def test_wait_state_machine():
    # 到达目标 epoch
    assert decide_wait_state(40, 40, 0, True, 0, 3600) == "done_epoch"
    # 早停/中断：csv 超 15 分钟未更新且无训练进程
    assert decide_wait_state(31, 40, IDLE_S + 1, False, 0, 3600) == "done_idle"
    # 早停判定要求至少有 epoch 记录（连 csv 都没有说明没开始）
    assert decide_wait_state(None, 40, IDLE_S + 1, False, 0, 3600) == "wait"
    # 还在正常训练：继续等
    assert decide_wait_state(10, 40, 5, True, 0, 3600) == "wait"
    # 卡死但进程还在：不判 idle（进程可能在写盘前缓冲）
    assert decide_wait_state(10, 40, IDLE_S + 1, True, 0, 3600) == "wait"
    # 超时
    assert decide_wait_state(10, 40, 5, True, 3700, 3600) == "timeout"


def test_wait_for_file_text(tmp_path):
    p = tmp_path / "watch.log"
    assert wait_for_file_text(p, "评估退出码", 0.3, poll_s=0.05) is False
    p.write_text("推理完成\n评估退出码: 0\n", encoding="utf-8")
    assert wait_for_file_text(p, "评估退出码", 1.0, poll_s=0.05) is True
    # 文本出现得晚也能等到
    assert wait_for_file_text(p, "评估退出码: 0", 1.0, poll_s=0.05) is True
    assert wait_for_file_text(p, "不存在的内容", 0.3, poll_s=0.05) is False
