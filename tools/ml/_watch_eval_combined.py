# -*- coding: utf-8 -*-
"""7cls_combined 训练完成监控 + 省赛综合精度自动评估。

检测到 results.csv 到达 epoch 60（或训练进程退出）后：
1. 用 best.pt 在原始 7cls val 帧（34 帧，与 91.07% 基线同口径）推理生成 YOLO 标签
2. 运行 eval_comprehensive.py 输出省赛综合识别精度

用法（后台）:
    python tools/ml/_watch_eval_combined.py
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

RESULTS = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls_combined" / "results.csv"
BEST_PT = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls_combined" / "weights" / "best.pt"

# 与 91.07% 综合精度基线相同的 val 集（原始 7cls 34 帧，含人工 GT）
VAL_IMG = Path(r"E:/ml_datasets/aerial_field_7cls/images/val")
VAL_GT = Path(r"E:/ml_datasets/aerial_field_7cls/labels/val")
NAMES = "mine,car,arrow,target_red,cell_label,target_blue,obstacle"
EXCLUDE = "car"

PRED_DIR = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls_combined" / "pred_val"
EVAL_SCRIPT = REPO / "tools" / "ml" / "eval_comprehensive.py"
LOG = REPO / "logs" / "7cls_combined_watch_eval.log"

TARGET_EPOCH = 60
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
    """检测是否有 combined 训练进程在运行。"""
    try:
        if _HAS_PSUTIL:
            for p in psutil.process_iter(["name", "cmdline"]):
                if p.info["name"] and "python" in p.info["name"].lower():
                    cmd = " ".join(p.info.get("cmdline") or [])
                    if "aerial_field_7cls_combined" in cmd and "train" in cmd:
                        return True
            return False
        r = subprocess.run(
            ["wmic", "process", "where", "name='python.exe'", "get", "commandline"],
            capture_output=True, text=True, timeout=10,
        )
        return "aerial_field_7cls_combined" in r.stdout and "train" in r.stdout
    except Exception:
        return False


def run_eval() -> None:
    """推理 + eval_comprehensive。"""
    if not BEST_PT.exists():
        log(f"错误: best.pt 不存在 ({BEST_PT})")
        return

    # 1. 推理生成 YOLO 预测标签
    log(f"推理: {BEST_PT} -> {PRED_DIR}")
    PRED_DIR.mkdir(parents=True, exist_ok=True)
    # 清理旧预测
    for f in PRED_DIR.glob("*.txt"):
        f.unlink()

    infer_cmd = [
        PYTHON, "-c",
        (
            "from ultralytics import YOLO; "
            f"m=YOLO(r'{BEST_PT}'); "
            f"m.predict(source=r'{VAL_IMG}', imgsz=1280, conf=0.25, iou=0.45, "
            f"save_txt=True, save_conf=False, project=r'{PRED_DIR.parent}', "
            f"name='{PRED_DIR.name}', exist_ok=True, verbose=False, device=0)"
        ),
    ]
    r = subprocess.run(infer_cmd, cwd=str(REPO), capture_output=True, text=True, timeout=1800)
    if r.returncode != 0:
        log(f"推理失败 (exit={r.returncode}): {r.stderr[-2000:]}")
        return
    log("推理完成")

    # 2. 综合精度评估（ultralytics save_txt 标签在 {save_dir}/labels/）
    pred_labels = PRED_DIR / "labels"
    log(f"运行 eval_comprehensive.py (pred={pred_labels})")
    eval_cmd = [
        PYTHON, str(EVAL_SCRIPT),
        "--pred-dir", str(pred_labels),
        "--gt-dir", str(VAL_GT),
        "--img-dir", str(VAL_IMG),
        "--names", NAMES,
        "--exclude", EXCLUDE,
        "--iou", "0.5",
    ]
    r = subprocess.run(eval_cmd, cwd=str(REPO), capture_output=True, text=True, timeout=600)
    log("=== eval_comprehensive 输出 ===")
    log(r.stdout)
    if r.stderr:
        log("=== stderr ===")
        log(r.stderr[-2000:])
    log(f"评估退出码: {r.returncode}")


def main() -> None:
    log(f"watch_eval_combined 启动，目标 epoch={TARGET_EPOCH}")
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
                log(f"results.csv 已 {age:.0f}s 未更新且无训练进程，判定训练结束(ep={ep})，启动评估...")
                break
        else:
            idle_count = 0
        time.sleep(POLL)

    run_eval()
    log("watch_eval_combined 完成。")


if __name__ == "__main__":
    main()
