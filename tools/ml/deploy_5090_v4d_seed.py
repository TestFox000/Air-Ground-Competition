# -*- coding: utf-8 -*-
"""5090实例一键部署：cell v4d（几何自动标注扩量）+ seed_157（标靶种子）并发训练。

本地打包 -> 上传 -> 解压 -> 修路径 -> 并发拉起两训练（拉满GPU/CPU）。
用法:
    python deploy_5090_v4d_seed.py            # 立即尝试, 失败进入等待循环
    python deploy_5090_v4d_seed.py --once     # 只试一次(默认退出)
    python deploy_5090_v4d_seed.py --skip-seed / --skip-v4d
开机自动部署: 后台挂 --wait, 每60s探测, 连通即部署全部。
"""
from __future__ import annotations

import argparse
import io
import os
import sys
import time
import zipfile
from pathlib import Path

import paramiko

# 密码不再硬编码，从环境变量 AG_5090_PASS 读取（同 tools/train_on_5090.py 范式）
HOST, PORT, USER = "connect.bjb2.seetacloud.com", 37113, "root"
PWD = os.environ.get("AG_5090_PASS", "")
V4D_ZIP = Path(r"E:\ml_datasets\cellv4d_upload_v2.zip")  # v2: 正确 cell_labels/ 结构 + merged_v4d.csv
SEED_DIR = Path(r"E:\ml_datasets\seed_157")
SEED_ZIP = Path(r"E:\ml_datasets\seed_157_upload.zip")
REMOTE = "/root/autodl-tmp"


def pack_seed() -> Path:
    if SEED_ZIP.is_file() and SEED_ZIP.stat().st_mtime > max(
            f.stat().st_mtime for f in SEED_DIR.rglob("*") if f.is_file()):
        print(f"seed zip 已是最新: {SEED_ZIP.stat().st_size/1e6:.1f} MB")
        return SEED_ZIP
    print("打包 seed_157 ...")
    with zipfile.ZipFile(SEED_ZIP, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for f in sorted(SEED_DIR.rglob("*")):
            if f.is_file() and "__pycache__" not in str(f):
                z.write(f, f.relative_to(SEED_DIR).as_posix())
    print(f"seed zip: {SEED_ZIP.stat().st_size/1e6:.1f} MB")
    return SEED_ZIP


def upload(ftp, local: Path, remote: str):
    total = local.stat().st_size
    last = [0.0]

    def cb(done, tot):
        if time.time() - last[0] > 8:
            print(f"  {remote}: {done/1e6:.0f}/{tot/1e6:.0f} MB", flush=True)
            last[0] = time.time()

    ftp.put(str(local), remote, callback=cb)
    print(f"  {remote}: 上传完成 {total/1e6:.1f} MB", flush=True)


def run(c, cmd: str, timeout: int = 120) -> str:
    _, out, err = c.exec_command(cmd, timeout=timeout)
    o, e = out.read().decode(), err.read().decode()
    rc = out.channel.recv_exit_status()
    if rc != 0:
        print(f"[WARN] rc={rc}: {cmd}\n{e[:500]}")
    return o + e


def deploy(skip_seed: bool, skip_v4d: bool) -> bool:
    if not PWD:
        raise SystemExit("错误: 未设置环境变量 AG_5090_PASS（5090 实例 root 密码），请先设置后再运行")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, port=PORT, username=USER, password=PWD, timeout=15)
    print("已连接 5090 实例")
    ftp = c.open_sftp()

    gpu = run(c, "nvidia-smi --query-gpu=name,memory.total --format=csv,noheader")
    py = run(c, "/root/miniconda3/bin/python -c 'import torch;print(torch.__version__, torch.cuda.is_available())'")
    print(f"GPU: {gpu.strip()} | torch: {py.strip()}")

    if not skip_v4d:
        run(c, f"mkdir -p {REMOTE}/cellv4d")
        upload(ftp, V4D_ZIP, f"{REMOTE}/cellv4d/upload.zip")
        print("解压 v4d ...")
        run(c, f"cd {REMOTE}/cellv4d && unzip -oq upload.zip && rm upload.zip", timeout=600)
        v4d_pipe = (
            "cd /root/autodl-tmp/cellv4d && "
            "/root/miniconda3/bin/python build_cell_v4d.py --root . --aug-per 20 --aug-workers 12 && "
            "PYTHONPATH=. /root/miniconda3/bin/python -m cell_classifier.train "
            "--csv cell_labels/merged_v4d.csv --output runs/cell_v4d "
            "--epochs 80 --batch-size 256 --workers 12"
        )
        run(c, f'nohup bash -c {v4d_pipe!r} > {REMOTE}/cellv4d/pipeline.log 2>&1 & echo v4d_pid=$!')
        print("v4d 管线已拉起 (增广+训练)")

    if not skip_seed:
        z = pack_seed()
        run(c, f"mkdir -p {REMOTE}/seed_157")
        upload(ftp, z, f"{REMOTE}/seed_upload.zip")
        run(c, f"cd {REMOTE}/seed_157 && unzip -oq ../seed_upload.zip && rm ../seed_upload.zip",
            timeout=600)
        run(c, "sed -i 's|^path:.*|path: /root/autodl-tmp/seed_157|' "
               f"{REMOTE}/seed_157/data.yaml")
        seed_cmd = (
            "cd /root/autodl-tmp && /root/miniconda3/bin/yolo detect train model=yolo11s.pt "
            "data=seed_157/data.yaml imgsz=960 epochs=120 batch=24 workers=8 "
            "amp=True project=runs/seed name=seed157_v1"
        )
        run(c, f'nohup bash -c {seed_cmd!r} > {REMOTE}/seed_train.log 2>&1 & echo seed_pid=$!')
        print("seed157 标靶训练已拉起")

    print(run(c, "sleep 3; ps aux | grep -E 'cell_classifier|yolo' | grep -v grep | awk '{print $2, $11, $12, $13}'"))
    print("\n监控: ssh -p 37113 root@connect.bjb2.seetacloud.com")
    print(f"  tail -f {REMOTE}/cellv4d/pipeline.log   # v4d")
    print(f"  tail -f {REMOTE}/seed_train.log         # seed")
    c.close()
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--once", action="store_true", help="只尝试一次")
    ap.add_argument("--wait", action="store_true", help="等待循环模式(60s一轮)")
    ap.add_argument("--skip-seed", action="store_true")
    ap.add_argument("--skip-v4d", action="store_true")
    args = ap.parse_args()
    if not (args.once or args.wait):
        args.once = True  # 默认单次
    while True:
        try:
            deploy(args.skip_seed, args.skip_v4d)
            return
        except Exception as e:
            print(f"[{time.strftime('%H:%M:%S')}] 未就绪: {type(e).__name__}")
            if args.once:
                print("实例未开机。用 --wait 挂后台等开机自动部署。")
                return
            time.sleep(60)


if __name__ == "__main__":
    main()
