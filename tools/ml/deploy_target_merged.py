# -*- coding: utf-8 -*-
"""部署合并后的标靶数据集(target_merged, 652帧)到5090并启动训练。"""
import os
import time, zipfile
from pathlib import Path
import paramiko

# 密码不再硬编码，从环境变量 AG_5090_PASS 读取（同 tools/train_on_5090.py 范式）
HOST, PORT, USER = "connect.bjb2.seetacloud.com", 37113, "root"
PWD = os.environ.get("AG_5090_PASS", "")
DATA_DIR = Path(r"E:\ml_datasets\target_merged")
ZIP = Path(r"E:\ml_datasets\target_merged_upload.zip")
REMOTE = "/root/autodl-tmp"

def pack():
    if ZIP.is_file() and ZIP.stat().st_mtime > max(f.stat().st_mtime for f in DATA_DIR.rglob("*") if f.is_file()):
        print(f"zip已是最新: {ZIP.stat().st_size/1e6:.1f} MB")
        return ZIP
    print("打包 target_merged ...")
    with zipfile.ZipFile(ZIP, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for f in sorted(DATA_DIR.rglob("*")):
            if f.is_file() and "__pycache__" not in str(f):
                z.write(f, f.relative_to(DATA_DIR).as_posix())
    print(f"zip: {ZIP.stat().st_size/1e6:.1f} MB")
    return ZIP

def run(c, cmd, timeout=120):
    _, o, e = c.exec_command(cmd, timeout=timeout)
    out = o.read().decode(errors="replace")
    err = e.read().decode(errors="replace")
    return out, err

def run_bg(c, cmd):
    """后台命令: 重定向所有fd, 不等退出"""
    full = f"nohup bash -c {cmd!r} </dev/null >/dev/null 2>&1 &"
    c.exec_command(full)

def main():
    if not PWD:
        raise SystemExit("错误: 未设置环境变量 AG_5090_PASS（5090 实例 root 密码），请先设置后再运行")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, port=PORT, username=USER, password=PWD, timeout=15)
    print("已连接5090")

    # 1. 杀旧yolo
    out, _ = run(c, "pkill -f 'yolo detect' 2>/dev/null; sleep 1; echo done")
    print("杀旧yolo:", out.strip())

    # 2. 检查yolo11s.pt
    out, _ = run(c, f"stat -c%s {REMOTE}/yolo11s.pt 2>/dev/null || echo 0")
    sz = int(out.strip() or 0)
    print(f"yolo11s.pt 当前: {sz/1e6:.1f} MB")
    if sz < 18000000:
        run_bg(c, f"cd {REMOTE} && wget -c -q https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11s.pt -O yolo11s.pt")
        print("已启动yolo11s.pt后台续传")

    # 3. 上传数据集
    z = pack()
    ftp = c.open_sftp()
    print(f"上传 {ZIP.stat().st_size/1e6:.1f} MB ...")
    ftp.put(str(ZIP), f"{REMOTE}/target_merged.zip")
    ftp.close()
    print("解压...")
    run(c, f"cd {REMOTE} && rm -rf target_merged && unzip -oq target_merged.zip -d target_merged && rm target_merged.zip", timeout=300)
    run(c, f"sed -i 's|^path:.*|path: {REMOTE}/target_merged|' {REMOTE}/target_merged/data.yaml")
    out, _ = run(c, f"find {REMOTE}/target_merged/images -name '*.jpg' | wc -l")
    print(f"图片数: {out.strip()}")

    # 4. 等yolo11s.pt下载完成
    print("等待yolo11s.pt下载 ...")
    for i in range(80):  # max 20 min
        out, _ = run(c, f"stat -c%s {REMOTE}/yolo11s.pt 2>/dev/null || echo 0")
        sz = int(out.strip() or 0)
        if sz > 18000000:
            print(f"下载完成: {sz/1e6:.1f} MB")
            break
        time.sleep(15)
    else:
        print("超时, 用现有文件尝试")

    # 5. 开训
    train_cmd = (
        f"cd {REMOTE} && /root/miniconda3/bin/yolo detect train "
        f"model={REMOTE}/yolo11s.pt data={REMOTE}/target_merged/data.yaml "
        f"imgsz=960 epochs=150 batch=32 workers=8 amp=True "
        f"project=runs/target name=target_merged_v1"
    )
    run_bg(c, train_cmd + f" > {REMOTE}/target_train.log 2>&1")
    print("target_merged训练已启动!")
    print(f"日志: tail -f {REMOTE}/target_train.log")
    c.close()

if __name__ == "__main__":
    main()
