# -*- coding: utf-8 -*-
"""7cls 蒸馏训练守护进程：训练挂了自动从 last.pt 恢复。

用法（后台）:
    C:\\Users\\a\\venvs\\air_ground\\Scripts\\python.exe tools/ml/_guard_7cls.py

与 watch_eval 配合：本脚本负责训练进程存活/续训，watch_eval 负责完成后跑评估。
"""
import os
import subprocess
import sys
import time
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parents[2]
PYTHON = sys.executable
LAST_PT = WORK_DIR / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls" / "weights" / "last.pt"
TRAIN_SCRIPT = WORK_DIR / "tools" / "ml" / "_train_7cls.py"
LOG = WORK_DIR / "logs" / "7cls_guard.log"


def log(msg: str) -> None:
    line = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}"
    print(line, flush=True)
    LOG.parent.mkdir(parents=True, exist_ok=True)
    with LOG.open("a", encoding="utf-8") as f:
        f.write(line + "\n")


def main() -> None:
    log(f"guard started (PID={os.getpid()}, python={PYTHON})")
    while True:
        if LAST_PT.exists() and LAST_PT.stat().st_size > 0:
            log(f"resume from {LAST_PT}")
            # ultralytics resume：从 last.pt 读取 args，继续训练
            cmd = [
                PYTHON, "-c",
                "from ultralytics import YOLO; "
                f"YOLO(r'{LAST_PT}').train(resume=True)",
            ]
        else:
            log("no last.pt, starting fresh")
            cmd = [PYTHON, str(TRAIN_SCRIPT)]
        try:
            proc = subprocess.run(cmd, cwd=str(WORK_DIR))
            log(f"train exited code={proc.returncode}, restart in 10s")
        except Exception as e:
            log(f"train crashed: {e}, restart in 10s")
        time.sleep(10)


if __name__ == "__main__":
    main()
