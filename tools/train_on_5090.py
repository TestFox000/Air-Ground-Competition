# -*- coding: utf-8 -*-
"""上传数据集到 AutoDL 5090 实例并启动 YOLO 训练，完成后下载结果。

用法:
  # 上传 + 训练 + 下载（一条龙）
  python tools/train_on_5090.py --host xxx.xxx.xxx.xxx --port 22 --user root --password xxx \\
      --dataset capture/datasets/aerial_field_mixed --weights runs/detect/.../best.pt \\
      --epochs 75 --batch 8 --imgsz 1280

  # 只上传数据集
  python tools/train_on_5090.py ... --upload-only

  # 只训练（数据已上传）
  python tools/train_on_5090.py ... --train-only

  # 只下载结果
  python tools/train_on_5090.py ... --download-only --run-name exp3

依赖: pip install paramiko scp
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

try:
    import paramiko
    from scp import SCPClient
except ImportError:
    print("缺少依赖: pip install paramiko scp", file=sys.stderr)
    raise SystemExit(2)


def connect(host, port, user, password):
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(hostname=host, port=port, username=user, password=password, timeout=30)
    return cli


def upload_dataset(cli, local_path: Path, remote_dir: str):
    """上传数据集目录到远程。"""
    print(f"上传数据集 {local_path} -> {remote_dir}")
    with SCPClient(cli.get_transport(), progress=lambda t, s: None) as scp:
        scp.put(str(local_path), recursive=True, remote_path=remote_dir)
    print("上传完成")


def run_train(cli, remote_dataset: str, weights: str, epochs: int, batch: int,
              imgsz: int, name: str, remote_workdir: str):
    """在远程运行 YOLO 训练。"""
    cmd = (
        f"cd {remote_workdir} && "
        f"source /etc/profile 2>/dev/null; "
        f"python -m ultralytics train "
        f"data={remote_dataset}/data.yaml "
        f"model={weights} "
        f"epochs={epochs} batch={batch} imgsz={imgsz} "
        f"workers=4 amp=False device=0 "
        f"project=runs/detect name={name} exist_ok=True "
        f"patience=30 seed=42"
    )
    print(f"远程训练命令:\n{cmd}\n")
    print("=== 训练开始（实时输出）===")
    stdin, stdout, stderr = cli.exec_command(cmd, get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")
    print("=== 训练结束 ===")
    return stdout.channel.recv_exit_status()


def download_results(cli, remote_workdir: str, name: str, local_out: Path):
    """下载训练结果。"""
    remote_path = f"{remote_workdir}/runs/detect/{name}"
    print(f"下载训练结果 {remote_path} -> {local_out}")
    local_out.mkdir(parents=True, exist_ok=True)
    with SCPClient(cli.get_transport()) as scp:
        scp.get(remote_path, local_path=str(local_out), recursive=True)
    print(f"下载完成: {local_out / name}")


def main() -> int:
    ap = argparse.ArgumentParser(description="AutoDL 5090 训练：上传 + 训练 + 下载")
    ap.add_argument("--host", required=True, help="AutoDL SSH 地址")
    ap.add_argument("--port", type=int, default=22, help="SSH 端口")
    ap.add_argument("--user", default="root", help="SSH 用户名")
    ap.add_argument("--password", default="", help="SSH 密码（也可用环境变量 AG_5090_PASS）")
    ap.add_argument("--dataset", required=True, help="本地数据集目录路径")
    ap.add_argument("--weights", required=True, help="预训练权重路径")
    ap.add_argument("--epochs", type=int, default=75)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--name", default="aerial_field", help="训练 run 名称")
    ap.add_argument("--remote-workdir", default="/root/air_ground", help="远程工作目录")
    ap.add_argument("--out", default="runs/detect/5090", help="结果下载到本地的目录")
    ap.add_argument("--upload-only", action="store_true")
    ap.add_argument("--train-only", action="store_true")
    ap.add_argument("--download-only", action="store_true")
    args = ap.parse_args()

    password = args.password or os.environ.get("AG_5090_PASS", "")
    if not password:
        print("请提供密码: --password 或环境变量 AG_5090_PASS", file=sys.stderr)
        return 1

    dataset_path = Path(args.dataset)
    weights_path = Path(args.weights)

    print(f"连接 {args.user}@{args.host}:{args.port} ...")
    cli = connect(args.host, args.port, args.user, password)
    print("已连接")

    remote_dataset = f"{args.remote_workdir}/{dataset_path.name}"

    try:
        if not args.download_only and not args.train_only:
            cli.exec_command(f"mkdir -p {args.remote_workdir}")
            upload_dataset(cli, dataset_path, args.remote_workdir)
            upload_dataset(cli, weights_path, args.remote_workdir)
            if args.upload_only:
                return 0

        if not args.download_only and not args.upload_only:
            remote_weights = f"{args.remote_workdir}/{weights_path.name}"
            code = run_train(cli, remote_dataset, remote_weights,
                             args.epochs, args.batch, args.imgsz, args.name, args.remote_workdir)
            print(f"训练退出码: {code}")
            if code != 0:
                print("训练可能失败，请检查远程日志", file=sys.stderr)
                return code

        if not args.upload_only and not args.train_only:
            download_results(cli, args.remote_workdir, args.name, Path(args.out))
    finally:
        cli.close()

    print("\n全部完成")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
