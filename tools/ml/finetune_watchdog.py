#!/usr/bin/env python3
"""训练完成后自动微调的 watchdog。

功能：
  1. 轮询监控原 YOLO 训练（results.csv 最后修改时间 + 最后 epoch）
  2. 训练正常完成（epoch>=195 或日志含完成标志）后，自动用混合数据集微调
  3. 微调完成后写汇总日志
  4. 全程记录到 logs/finetune_watchdog.log

用法（后台运行）：
  start /B python tools/ml/finetune_watchdog.py > logs/watchdog_console.log 2>&1
"""

from __future__ import annotations

import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PYTHON = r"C:\Users\a\venvs\air_ground\Scripts\python.exe"

# 原训练产物
RESULTS_CSV = ROOT / "runs" / "detect" / "runs" / "detect" / "aerial_field" / "results.csv"
BEST_PT = ROOT / "runs" / "detect" / "runs" / "detect" / "aerial_field" / "weights" / "best.pt"
TRAIN_LOG = Path(r"C:\Users\a\AppData\Local\Temp\trae-agent-toolhost\jobs\job-99ab665ac43140ebaa8a39015e17b04d\output.log")

# 微调配置
MIXED_DATA = ROOT / "capture" / "datasets" / "aerial_field_mixed" / "data.yaml"
FT_PROJECT = "runs/detect"
FT_NAME = "aerial_field_finetuned"
FT_EPOCHS = 80
FT_LR0 = 0.001  # 微调学习率，比原训练 0.01 低一个量级
FT_BATCH = 8
FT_IMGSZ = 1280

# 监控参数
POLL_INTERVAL = 120  # 每 2 分钟检查一次
IDLE_THRESHOLD = 600  # results.csv 10 分钟未更新视为训练结束
LOG_FILE = ROOT / "logs" / "finetune_watchdog.log"


def log(msg: str) -> None:
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line, flush=True)
    LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    with LOG_FILE.open("a", encoding="utf-8") as f:
        f.write(line + "\n")


def get_last_epoch() -> int | None:
    if not RESULTS_CSV.exists():
        return None
    try:
        lines = RESULTS_CSV.read_text(encoding="utf-8").strip().splitlines()
        if len(lines) < 2:
            return None
        last = lines[-1].split(",")
        return int(float(last[0]))
    except Exception:
        return None


def results_mtime_age() -> float | None:
    """results.csv 距上次修改的秒数，文件不存在返回 None。"""
    if not RESULTS_CSV.exists():
        return None
    return time.time() - RESULTS_CSV.stat().st_mtime


def train_finished_normally() -> tuple[bool, str]:
    """判断原训练是否正常完成。

    正常完成条件（满足其一）：
      1. results.csv 最后 epoch >= 195
      2. 训练日志末尾含 Ultralytics 完成标志（"Results saved" / "Speed:" 汇总行）
    """
    epoch = get_last_epoch()
    if epoch is not None and epoch >= 195:
        return True, f"epoch={epoch} >= 195"

    if TRAIN_LOG.exists():
        try:
            tail = TRAIN_LOG.read_text(encoding="utf-8", errors="ignore")[-2000:]
            if "Results saved to" in tail and "Speed:" in tail:
                return True, "log contains completion markers"
        except Exception:
            pass
    return False, f"epoch={epoch}, no completion markers"


def wait_for_train_finish() -> bool:
    """等待原训练结束。返回 True=正常完成，False=超时/异常。"""
    log("watchdog 启动，开始监控原训练...")
    start_wait = time.time()
    last_logged_epoch = -1

    while True:
        epoch = get_last_epoch()
        age = results_mtime_age()

        if epoch is not None and epoch != last_logged_epoch:
            log(f"  训练进度: epoch={epoch}/200, results.csv 年龄={age:.0f}s")
            last_logged_epoch = epoch

        # 训练结束判定：results.csv 长时间未更新
        if age is not None and age > IDLE_THRESHOLD:
            ok, reason = train_finished_normally()
            if ok:
                log(f"原训练正常完成（{reason}）")
                return True
            else:
                log(f"WARNING: 训练已停止但未正常完成（{reason}）。不启动微调，等待人工处理。")
                log(f"  （若训练是被你手动中断的，请检查后手动启动微调）")
                return False

        # 超时保护：等 24 小时还没完就退出
        if time.time() - start_wait > 24 * 3600:
            log("ERROR: 等待超过 24 小时，watchdog 退出。")
            return False

        time.sleep(POLL_INTERVAL)


def run_finetune() -> int:
    """启动微调训练，返回进程退出码。"""
    if not BEST_PT.exists():
        log(f"ERROR: best.pt 不存在: {BEST_PT}")
        return -1

    cmd = [
        PYTHON, "-m", "ultralytics",
        "detect", "train",
        f"model={BEST_PT}",
        f"data={MIXED_DATA}",
        f"epochs={FT_EPOCHS}",
        f"lr0={FT_LR0}",
        f"batch={FT_BATCH}",
        f"imgsz={FT_IMGSZ}",
        "workers=0",
        "amp=False",
        f"project={FT_PROJECT}",
        f"name={FT_NAME}",
        "exist_ok=True",
        "patience=30",
        "cos_lr=True",
        "verbose=True",
    ]
    log("启动微调训练:")
    log("  " + " ".join(cmd))

    ft_log = ROOT / "logs" / "finetune_train.log"
    with ft_log.open("w", encoding="utf-8") as f:
        proc = subprocess.run(cmd, cwd=str(ROOT), stdout=f, stderr=subprocess.STDOUT)
    log(f"微调进程退出码: {proc.returncode}")
    return proc.returncode


def summarize() -> None:
    """微调完成后写汇总。"""
    ft_results = ROOT / FT_PROJECT / FT_NAME / "results.csv"
    ft_best = ROOT / FT_PROJECT / FT_NAME / "weights" / "best.pt"

    log("=" * 60)
    log("微调汇总:")
    if ft_results.exists():
        lines = ft_results.read_text(encoding="utf-8").strip().splitlines()
        if len(lines) >= 2:
            last = lines[-1].split(",")
            log(f"  最终 epoch: {last[0]}")
            log(f"  mAP50-95: {last[9] if len(last) > 9 else 'N/A'}")
            log(f"  mAP50:    {last[8] if len(last) > 8 else 'N/A'}")
    if ft_best.exists():
        log(f"  best.pt: {ft_best} ({ft_best.stat().st_size / 1e6:.1f} MB)")
    else:
        log("  WARNING: best.pt 未生成，微调可能失败")
    log("=" * 60)


def main() -> None:
    log("finetune_watchdog 启动")
    log(f"  原训练 results.csv: {RESULTS_CSV}")
    log(f"  微调数据集: {MIXED_DATA}")

    # 1. 等原训练完成
    if not wait_for_train_finish():
        log("watchdog 退出：原训练未正常完成，未启动微调。")
        sys.exit(1)

    # 2. 启动微调
    rc = run_finetune()
    if rc != 0:
        log(f"微调失败（退出码 {rc}），请检查 logs/finetune_train.log")
        sys.exit(1)

    # 3. 汇总
    summarize()
    log("watchdog 全部完成。")


if __name__ == "__main__":
    main()
