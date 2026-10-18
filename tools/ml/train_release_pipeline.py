# -*- coding: utf-8 -*-
"""训完即评 → 过泛化门禁 → 发布：一键串联（训练机无人值守用）。

链路：
  1. 盯训练：results.csv 到目标 epoch，或（15 分钟未更新 且 无训练进程）判结束；
  2. 等 val 综合评估（可选）：watch_eval 守护写完日志即认（--watch-log）；
     综合精度是它的事，本脚本不重复评；
  3. 泛化门禁：tools/ml/eval_generalization.py（官方明牌考点，拒绝即中止，
     不发布——人工复核后 --update-baseline 重钉基线再手动发）；
  4. 过门禁即发布：tools/release_models.py --hbb best.pt --tag <tag>
     --generalization-images <留出图集>（release 内部会再跑一次门禁留痕）。

用法（训练机，虚拟环境 C:\\Users\\a\\venvs\\air_ground；赛前挂后台即可）：
  python tools/ml/train_release_pipeline.py \
      --run-dir "runs/detect/runs/detect/runs/detect/aerial_field_7cls_combined_minecpv2" \
      --tag 7cls_combined_minecpv2 --target-epoch 40 \
      --holdout-images "E:\\...\\1-爆炸物\\B-爆炸物"

  --no-release  只盯训练+跑门禁，不发布（演练用）
  --max-wait-hours  默认 12h 超时退出码 2
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8", errors="replace")  # Windows GBK 控制台防炸（§5.6）

REPO = Path(__file__).resolve().parents[2]
LOG = REPO / "logs" / "train_release_pipeline.log"

DEFAULT_RUN_DIR = (REPO / "runs" / "detect" / "runs" / "detect" / "runs"
                   / "detect" / "aerial_field_7cls_combined_minecpv2")
DEFAULT_WATCH_LOG = REPO / "logs" / "7cls_combined_minecpv2_watch_eval.log"

POLL_S = 120          # 2 分钟轮询
IDLE_S = 900          # results.csv 15 分钟未更新
TIMEOUT_RC = 2
GATE_REJECT_RC = 1


def log(msg: str) -> None:
    line = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}"
    print(line, flush=True)
    LOG.parent.mkdir(parents=True, exist_ok=True)
    with LOG.open("a", encoding="utf-8") as f:
        f.write(line + "\n")


# ---------------- 纯逻辑（pytest 直测） ----------------

def last_epoch(results_csv: Path) -> int | None:
    """results.csv 最后一行的 epoch 号；文件不存在/为空/坏行返回 None。"""
    if not results_csv.exists():
        return None
    try:
        lines = results_csv.read_text(encoding="utf-8").strip().splitlines()
        if len(lines) < 2:
            return None
        return int(float(lines[-1].split(",")[0]))
    except Exception:
        return None


def decide_wait_state(epoch: int | None, target_epoch: int,
                      csv_age_s: float, training_running: bool,
                      waited_s: float, max_wait_s: float) -> str:
    """等待状态机：done_epoch / done_idle / timeout / wait。"""
    if epoch is not None and epoch >= target_epoch:
        return "done_epoch"
    if csv_age_s > IDLE_S and not training_running and epoch is not None:
        return "done_idle"   # 早停/中断：有什么评什么
    if waited_s > max_wait_s:
        return "timeout"
    return "wait"


def training_process_running(run_key: str) -> bool:
    """是否有命令行里带 run_key 的 python 训练进程（psutil 优先，wmic 兜底）。"""
    try:
        import psutil
        for p in psutil.process_iter(["name", "cmdline"]):
            if p.info["name"] and "python" in p.info["name"].lower():
                cmd = " ".join(p.info.get("cmdline") or [])
                if run_key in cmd and "train" in cmd:
                    return True
        return False
    except ImportError:
        try:
            r = subprocess.run(
                ["wmic", "process", "where", "name='python.exe'", "get", "commandline"],
                capture_output=True, text=True, timeout=10)
            return run_key in r.stdout and "train" in r.stdout
        except Exception:
            return False
    except Exception:
        return False


def wait_for_file_text(path: Path, needle: str, timeout_s: float,
                       poll_s: float = POLL_S) -> bool:
    """等 watch_eval 日志里出现指定文本（如"评估退出码: 0"）；超时返回 False。"""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            if path.exists() and needle in path.read_text(encoding="utf-8",
                                                          errors="replace"):
                return True
        except Exception:
            pass
        time.sleep(poll_s)
    return False


# ---------------- 主流程 ----------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-dir", type=Path, default=DEFAULT_RUN_DIR,
                    help=f"训练 run 目录（默认本轮 combined_minecpv2: {DEFAULT_RUN_DIR}）")
    ap.add_argument("--tag", default=None,
                    help="发布版本标签（默认用 run 目录名）")
    ap.add_argument("--target-epoch", type=int, default=40)
    ap.add_argument("--holdout-images", type=Path, default=None,
                    help="泛化门禁留出图集（默认读环境变量 AIC_MINE_HOLDOUT）")
    ap.add_argument("--watch-log", type=Path, default=DEFAULT_WATCH_LOG,
                    help="watch_eval 守护日志（等它写完综合精度再走门禁；"
                         "不需要等就传空串 ''）")
    ap.add_argument("--no-release", action="store_true",
                    help="只盯训练+跑门禁，不发布（演练用）")
    ap.add_argument("--max-wait-hours", type=float, default=12.0)
    args = ap.parse_args()

    results_csv = args.run_dir / "results.csv"
    best_pt = args.run_dir / "weights" / "best.pt"
    tag = args.tag or args.run_dir.name
    run_key = args.run_dir.name
    t0 = time.time()

    log(f"pipeline 启动: run={args.run_dir.name} tag={tag} "
        f"target_epoch={args.target_epoch}")

    # 1. 盯训练完成
    while True:
        ep = last_epoch(results_csv)
        age = time.time() - results_csv.stat().st_mtime if results_csv.exists() else 0.0
        running = training_process_running(run_key)
        state = decide_wait_state(ep, args.target_epoch, age, running,
                                  time.time() - t0, args.max_wait_hours * 3600)
        if state == "timeout":
            log(f"超时（{args.max_wait_hours}h），退出；训练未完成(ep={ep})")
            return TIMEOUT_RC
        if state.startswith("done"):
            log(f"训练结束（{state}，ep={ep}）")
            break
        time.sleep(POLL_S)

    if not best_pt.exists():
        log(f"错误: best.pt 不存在（{best_pt}），中止")
        return 1

    # 2. 等 val 综合评估（watch_eval 守护的产出；空串跳过）
    if str(args.watch_log):
        log("等 watch_eval 综合精度（最多再等 30 分钟）...")
        if wait_for_file_text(args.watch_log, "评估退出码", 1800):
            log("watch_eval 已产出综合精度（见其日志）")
        else:
            log("警告: 未等到 watch_eval 产出，继续走门禁（综合精度稍后自查）")

    # 3. 泛化门禁
    gate_cmd = [sys.executable, str(REPO / "tools" / "ml" / "eval_generalization.py"),
                "--weights", str(best_pt)]
    if args.holdout_images:   # 缺省由门禁脚本读 AIC_MINE_HOLDOUT/报缺失
        gate_cmd += ["--images", str(args.holdout_images)]
    log(f"泛化门禁: {' '.join(gate_cmd)}")
    rc = subprocess.call(gate_cmd, cwd=str(REPO))
    if rc != 0:
        log("泛化门禁拒绝，发布中止。人工复核后用 --update-baseline 重钉基线，"
            "再手动 release_models.py 发布。")
        return GATE_REJECT_RC

    if args.no_release:
        log("--no-release：门禁已过，演练到此为止（未发布）")
        return 0

    # 4. 发布（release_models 内部会带 --record 再跑一次门禁留痕）
    rel_cmd = [sys.executable, str(REPO / "tools" / "release_models.py"),
               "--hbb", str(best_pt), "--tag", tag]
    if args.holdout_images:
        rel_cmd += ["--generalization-images", str(args.holdout_images)]
    log(f"发布: {' '.join(rel_cmd)}")
    rc = subprocess.call(rel_cmd, cwd=str(REPO))
    if rc != 0:
        log(f"发布失败（exit={rc}），查 release_models 输出")
        return rc
    log(f"完成：已发布 tag={tag}（models/release/ 登记表+current 指针已更新）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
