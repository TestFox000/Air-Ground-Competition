# -*- coding: utf-8 -*-
"""7cls 训练完成监控：检测到 results.csv 到达 epoch 75（或进程退出）后自动跑评估。

用法（后台）:
    python tools/ml/_7cls_watch_eval.py
"""
import subprocess
import sys
import time
from pathlib import Path

try:
    import psutil
    _HAS_PSUTIL = True
except ImportError:
    _HAS_PSUTIL = False

REPO = Path(__file__).resolve().parents[2]
PYTHON = r"C:\Users\a\venvs\air_ground\Scripts\python.exe"
RESULTS = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls" / "results.csv"
EVAL = REPO / "runs" / "distill_7cls" / "run_eval_7cls.py"
LOG = REPO / "logs" / "7cls_watch_eval.log"
TARGET_EPOCH = 75
POLL = 120  # 2 min


def log(msg: str) -> None:
    line = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}"
    print(line, flush=True)
    LOG.parent.mkdir(parents=True, exist_ok=True)
    with LOG.open("a", encoding="utf-8") as f:
        f.write(line + "\n")


def last_epoch() -> int | None:
    if not RESULTS.exists():
        return None
    try:
        lines = RESULTS.read_text(encoding="utf-8").strip().splitlines()
        return int(float(lines[-1].split(",")[0])) if len(lines) >= 2 else None
    except Exception:
        return None


def is_training_running() -> bool:
    """检测是否有 7cls 训练进程在运行（避免续训首epoch未写盘时误判结束）。"""
    try:
        if _HAS_PSUTIL:
            for p in psutil.process_iter(["name", "cmdline"]):
                if p.info["name"] and "python" in p.info["name"].lower():
                    cmd = " ".join(p.info.get("cmdline") or [])
                    if "aerial_field_7cls" in cmd and "train" in cmd:
                        return True
            return False
        # fallback: wmic
        r = subprocess.run(
            ["wmic", "process", "where", "name='python.exe'", "get", "commandline"],
            capture_output=True, text=True, timeout=10,
        )
        return "aerial_field_7cls" in r.stdout and "train" in r.stdout
    except Exception:
        return False


def main() -> None:
    log(f"watch_eval 启动，目标 epoch={TARGET_EPOCH}")
    last_logged = -1
    idle_count = 0
    while True:
        ep = last_epoch()
        if ep is not None and ep != last_logged:
            log(f"  训练进度: epoch={ep}/{TARGET_EPOCH}")
            last_logged = ep
        if ep is not None and ep >= TARGET_EPOCH:
            log(f"训练完成 epoch={ep}，启动评估...")
            break
        # 空闲检测：results.csv 超过 15 分钟未更新且训练进程不在了
        age = time.time() - RESULTS.stat().st_mtime if RESULTS.exists() else 0
        if age > 900 and not is_training_running():
            idle_count += 1
            if idle_count >= 2:
                log(f"results.csv 已 {age:.0f}s 未更新且无训练进程，判定训练结束，启动评估...")
                break
        else:
            idle_count = 0
        time.sleep(POLL)

    # 运行评估
    log(f"执行: {PYTHON} {EVAL}")
    result = subprocess.run([PYTHON, str(EVAL)], cwd=str(REPO),
                            capture_output=True, text=True, timeout=1800)
    log("=== 评估输出 ===")
    log(result.stdout)
    if result.stderr:
        log("=== stderr ===")
        log(result.stderr)
    log(f"评估退出码: {result.returncode}")
    log("watch_eval 完成。")


if __name__ == "__main__":
    main()
