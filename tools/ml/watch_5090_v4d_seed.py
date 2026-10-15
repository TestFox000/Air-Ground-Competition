# -*- coding: utf-8 -*-
"""5090 训练完成自动收尾：监视 v4d + seed157 → SFTP 下载产物 → 本地 v4d 帧评估。

与 deploy_5090_v4d_seed.py 配套：开机 → 部署 → 并发训练 → 本脚本监视到训练完成
→ 自动下载 best.pt/metrics/results → 本地重跑 eval_cell_frames（val 帧）。
全程无人值守，本脚本退出即代表对比报告所需数据全部就绪。

用法（后台挂机）: python watch_5090_v4d_seed.py
"""
from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

import paramiko

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# 密码不再硬编码，从环境变量 AG_5090_PASS 读取（同 tools/train_on_5090.py 范式）
HOST, PORT, USER = "connect.bjb2.seetacloud.com", 37113, "root"
PWD = os.environ.get("AG_5090_PASS", "")
REMOTE = "/root/autodl-tmp"
DL = Path(r"E:\ml_datasets\downloaded")
RENDER = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\cell_recog_render")
PY = r"C:\Users\a\venvs\air_ground\Scripts\python.exe"
EVAL = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\tools\ml\eval_cell_frames.py")

BEST = {
    "v4d": f"{REMOTE}/cellv4d/runs/cell_v4d/best.pt",
    "seed": f"{REMOTE}/runs/seed/seed157_v1/weights/best.pt",
    "target": f"{REMOTE}/runs/target/target_merged_v1/weights/best.pt",
}
LOGS = {
    "v4d": f"{REMOTE}/cellv4d/pipeline.log",
    "seed": f"{REMOTE}/seed_train.log",
    "target": f"{REMOTE}/target_train.log",
}
PROC = {"v4d": "cell_classifier", "seed": "yolo", "target": "yolo"}
ARTIFACTS = {  # tag -> [(remote, local)] 仅在对应训练完成时下载
    "v4d": [
        (f"{REMOTE}/cellv4d/runs/cell_v4d/best.pt", DL / "cell_classifier_v4d_best.pt"),
        (f"{REMOTE}/cellv4d/runs/cell_v4d/metrics.csv", DL / "cell_v4d_metrics.csv"),
        (f"{REMOTE}/cellv4d/runs/cell_v4d/config.json", DL / "cell_v4d_config.json"),
    ],
    "seed": [
        (f"{REMOTE}/runs/seed/seed157_v1/weights/best.pt", DL / "seed157_v1_best.pt"),
        (f"{REMOTE}/runs/seed/seed157_v1/results.csv", DL / "seed157_v1_results.csv"),
    ],
    "target": [
        (f"{REMOTE}/runs/target/target_merged_v1/weights/best.pt", DL / "target_merged_v1_best.pt"),
        (f"{REMOTE}/runs/target/target_merged_v1/results.csv", DL / "target_merged_v1_results.csv"),
    ],
}


def connect() -> paramiko.SSHClient:
    if not PWD:
        raise SystemExit("错误: 未设置环境变量 AG_5090_PASS（5090 实例 root 密码），请先设置后再运行")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, port=PORT, username=USER, password=PWD, timeout=15)
    return c


def run(c, cmd: str, timeout: int = 60) -> str:
    _, out, err = c.exec_command(cmd, timeout=timeout)
    return out.read().decode(errors="replace") + err.read().decode(errors="replace")


def wait_ssh() -> paramiko.SSHClient:
    while True:
        try:
            c = connect()
            print(f"[{time.strftime('%m-%d %H:%M')}] 已连接 5090", flush=True)
            return c
        except Exception as e:
            print(f"[{time.strftime('%m-%d %H:%M')}] 未就绪: {type(e).__name__}", flush=True)
            time.sleep(60)


def probe(c, tag: str):
    """返回 (best.pt存在, 训练进程存活, 日志尾部500字节)。"""
    best = run(c, f"test -f {BEST[tag]} && echo Y || echo N").strip() == "Y"
    alive = PROC[tag] in run(c, f"ps aux | grep -F '{PROC[tag]}' | grep -v grep")
    tail = run(c, f"tail -c 500 {LOGS[tag]} 2>/dev/null || echo NOLOG")
    return best, alive, tail


def download(cli, status) -> None:
    ftp = cli.open_sftp()
    for tag, arts in ARTIFACTS.items():
        if status[tag] != "done":
            continue
        for r, l in arts:
            if l.is_file() and l.stat().st_size > 0:
                continue
            try:
                l.parent.mkdir(parents=True, exist_ok=True)
                ftp.get(r, str(l))
                print(f"下载 {tag}: {l.name} ({l.stat().st_size / 1e6:.1f} MB)", flush=True)
            except Exception as e:
                print(f"[WARN] 下载失败 {r}: {e}", flush=True)
    ftp.close()


def main() -> None:
    status = {"v4d": "wait", "seed": "wait", "target": "wait"}
    bad = {"v4d": 0, "seed": 0, "target": 0}  # 连续异常观测次数(防瞬时误判)
    c = wait_ssh()

    # 阶段1: 监视训练 (best.pt出现且进程退出=完成; 无best+无进程+Traceback=失败)
    while any(s == "wait" for s in status.values()):
        try:
            for tag in ("v4d", "seed", "target"):
                if status[tag] != "wait":
                    continue
                best, alive, tail = probe(c, tag)
                if best and not alive:
                    status[tag] = "done"
                    print(f"[{time.strftime('%H:%M')}] {tag} 训练完成", flush=True)
                elif not alive and "Traceback" in tail:
                    bad[tag] += 1
                    if bad[tag] >= 2:
                        status[tag] = "fail"
                        print(f"[{time.strftime('%H:%M')}] {tag} 异常退出:\n{tail}", flush=True)
                else:
                    bad[tag] = 0
            if any(s == "wait" for s in status.values()):
                time.sleep(120)
        except Exception as e:
            print(f"[{time.strftime('%H:%M')}] 连接断开({type(e).__name__})，重连...", flush=True)
            try:
                c.close()
            except Exception:
                pass
            c = wait_ssh()

    # 阶段2: 下载产物 (失败重连重试一次)
    try:
        download(c, status)
    except Exception as e:
        print(f"[WARN] SFTP 异常: {e}", flush=True)
    if any(status[t] == "done" and not ARTIFACTS[t][0][1].is_file() for t in status):
        print("主产物缺失，重连重试下载 ...", flush=True)
        try:
            c.close()
        except Exception:
            pass
        try:
            c2 = wait_ssh()
            download(c2, status)
            c2.close()
        except Exception as e:
            print(f"[WARN] 重试异常: {e}", flush=True)

    # 阶段3: 本地 v4d 帧评估 (v4c_v6 基线已先行存为 eval_frames_v4c_v6.json)
    if status["v4d"] == "done" and (DL / "cell_classifier_v4d_best.pt").is_file():
        evj = RENDER / "eval_frames.json"
        if evj.exists():
            evj.replace(RENDER / "eval_frames_prev.json")
        try:
            r = subprocess.run(
                [PY, str(EVAL), "--weights", str(DL / "cell_classifier_v4d_best.pt"),
                 "--frames", "val"],
                capture_output=True, text=True, timeout=1800)
            (RENDER / "eval_stdout_v4d.txt").write_text(r.stdout + r.stderr, encoding="utf-8")
            if evj.exists():
                evj.replace(RENDER / "eval_frames_v4d.json")
            print("\n".join(r.stdout.strip().splitlines()[-8:]), flush=True)
        except Exception as e:
            print(f"[WARN] 本地评估失败: {e}", flush=True)

    print("\n=== 收尾完成: " + " | ".join(f"{k}={v}" for k, v in status.items()) + " ===",
          flush=True)


if __name__ == "__main__":
    main()
