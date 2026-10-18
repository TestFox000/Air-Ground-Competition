"""训练守护进程：训练挂了自动从 last.pt 恢复。

用法：
    <venv-python> tools/ml/guard_train.py

开机自启：把命令做成快捷方式放进 shell:startup。
路径均按本文件位置自动推导，换机器/换目录无需修改。
"""
import subprocess
import sys
import time
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parents[2]


def find_last_pt() -> Path | None:
    """在 runs/ 下找最新的 last.pt（兼容历史嵌套目录）。"""
    runs = WORK_DIR / "runs"
    candidates = list(runs.rglob("last.pt")) if runs.exists() else []
    candidates = [p for p in candidates if p.stat().st_size > 0]
    return max(candidates, key=lambda p: p.stat().st_mtime, default=None)


def log(msg: str) -> None:
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line, flush=True)
    with open(WORK_DIR / "train_guard.log", "a", encoding="utf-8") as f:
        f.write(line + "\n")


def main() -> None:
    log(f"guard started (PID={__import__('os').getpid()}, python={sys.executable})")
    while True:
        last_pt = find_last_pt()
        if last_pt is not None:
            cmd = [sys.executable, "tools/ml/train_detect.py", "--resume"]
            log(f"resume from {last_pt}")
        else:
            cmd = [sys.executable, "tools/ml/train_detect.py",
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
