"""训练守护进程：训练挂了自动从 last.pt 恢复。

用法：
    C:\\Users\\29212\\Desktop\\ultralytics-main\\.venv\\Scripts\\python.exe tools/ml/guard_train.py

开机自启：把命令做成快捷方式放进 shell:startup。
"""
import subprocess
import sys
import time
from pathlib import Path

PYTHON = r"C:\Users\29212\Desktop\ultralytics-main\.venv\Scripts\python.exe"
WORK_DIR = Path(r"C:\Users\29212\CodeBuddy\20260927120106\air_ground")
# 搜索最新的 last.pt
LAST_PT = WORK_DIR / "runs" / "detect" / "runs" / "detect" / "aerial_field" / "weights" / "last.pt"


def log(msg: str) -> None:
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line, flush=True)
    with open(WORK_DIR / "train_guard.log", "a", encoding="utf-8") as f:
        f.write(line + "\n")


def main() -> None:
    log(f"guard started (PID={__import__('os').getpid()})")
    while True:
        if LAST_PT.exists():
            cmd = [PYTHON, "tools/ml/train_detect.py", "--resume"]
            log(f"resume from {LAST_PT}")
        else:
            cmd = [PYTHON, "tools/ml/train_detect.py",
                   "--data", "capture/datasets/aerial_field/data.yaml",
                   "--batch", "8"]
            log("no last.pt, starting fresh")

        try:
            proc = subprocess.run(cmd, cwd=str(WORK_DIR))
            log(f"train exited code={proc.returncode}, restart in 10s")
        except Exception as e:
            log(f"train crashed: {e}, restart in 10s")
        time.sleep(10)


if __name__ == "__main__":
    main()
