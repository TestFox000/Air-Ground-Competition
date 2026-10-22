# -*- coding: utf-8 -*-
"""把规划层接入 13 类主链：一键部署 / 编译验证 / 冒烟 / 回滚。

做什么
------
  1. 备份机上 13 类主链的 4 个原文件（.bak_plan13_<时间戳>），保留原样可回滚；
  2. 推送算法级适配后的源码（main / MapLocalizer / CellGrid / RoutePlanner / CMake）；
  3. 删产物强制重编（编译失败即停，绝不运行）；
  4. 冒烟：喂一段视频，检查是否产出 route_plan.json 并打印路线与耗时；
  5. --rollback 还原全部原文件并删除新增文件，重新编译回原状。

为什么是算法级移植而不是重写
----------------------------
  * 13 类主链已有：D13 检测（TensorRT）+ CellClassifier 格号 + MapLocalizer(PnP) + UDP；
  * 规划层（CellGrid 证据累积 + RoutePlanner Held-Karp/Dijkstra）已在机上 7 类血缘中存在；
  * 本次只把规划层接进 13 类，并按官方场地语义做落格映射：
        paper 在 K 列 = 桥洞可通行；其余 paper = 雷
        car_start / car_end = 通格（起点/终点）
        target_red / target_blue = 靶（占相邻两格，规划禁入）
        obstacle / bucket_* = 障碍
        不引入绿色像素投票、不预设桥体行（与 Python 版 ag_integrate 口径一致）。

用法
----
  set AG_ORIN_PASS=******            # 或 --password
  python deploy_plan13.py --host 192.168.0.197
  python deploy_plan13.py --host ... --rollback
  python deploy_plan13.py --host ... --smoke-only
"""
from __future__ import annotations

import argparse
import hashlib
import os
import pathlib
import re
import sys
import time

import paramiko

BASE = pathlib.Path(__file__).resolve().parent
SRC = BASE.parent / "src"
REMOTE = "/home/duzhong/dzuav_deploy/air_ground/onboard/dzuav_mjpg"
TS = time.strftime("%Y%m%d_%H%M%S")
# 稳定备份名 + cp -n：重复部署不会用"已改过的版本"覆盖最初的原文件。
BAK = ".bak_plan13_orig"

# 本地 -> 远端 相对路径
UPLOADS = {
    "main13_plan.cpp": "src/main.cpp",
    "MapLocalizer.hpp": "include/dzuav/MapLocalizer.hpp",
    "MapLocalizer.cpp": "src/MapLocalizer.cpp",
    "CellGrid.hpp": "include/dzuav/CellGrid.hpp",
    "CellGrid.cpp": "src/CellGrid.cpp",
    "RoutePlanner.hpp": "include/dzuav/RoutePlanner.hpp",
    "RoutePlanner.cpp": "src/RoutePlanner.cpp",
    "CMakeLists.txt": "CMakeLists.txt",
}
# 新增文件（回滚时删除）
NEW_FILES = ["include/dzuav/CellGrid.hpp", "src/CellGrid.cpp",
             "include/dzuav/RoutePlanner.hpp", "src/RoutePlanner.cpp"]

HBB = "~/dzuav/engines/hbb_detector_d13_fp16.engine"
CELL = "~/dzuav/engines/cell_classifier_v4d_fp16.engine"
VIDEO = "~/Videos/recordings/cam_20261017_205245.mp4"


def md5(path: pathlib.Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


class R:
    def __init__(self, host, user, pw):
        self.c = paramiko.SSHClient()
        self.c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.c.connect(host, username=user, password=pw, timeout=15)

    def run(self, cmd, timeout=900):
        _, o, e = self.c.exec_command(cmd, timeout=timeout)
        return (o.read().decode("utf-8", "replace")
                + e.read().decode("utf-8", "replace")).strip()

    def put(self, local: pathlib.Path, remote: str):
        sftp = self.c.open_sftp()
        try:
            sftp.put(str(local), remote)
        finally:
            sftp.close()

    def close(self):
        self.c.close()


def build(r: R) -> str:
    return r.run(f"bash -lc 'cd {REMOTE} && rm -f build/dzuav && "
                 f"cmake -S . -B build -DCMAKE_BUILD_TYPE=Release >/dev/null && "
                 f"cmake --build build --parallel $(nproc) 2>&1 | tail -8; "
                 f"echo BUILD_RC=${{PIPESTATUS[0]}}'", timeout=1500)


def deploy(r: R):
    print("=== 1/4 备份机上原文件 ===")
    print(r.run(" ; ".join(
        f"cp -n {REMOTE}/{rel} {REMOTE}/{rel}{BAK}" for rel in UPLOADS.values()
        if rel not in NEW_FILES) + f" ; find {REMOTE} -maxdepth 3 -name '*{BAK}' | wc -l"))

    print("=== 2/4 推送源码（md5 校验） ===")
    for local_name, rel in UPLOADS.items():
        lp = SRC / local_name
        if not lp.is_file():
            raise SystemExit(f"[err] 本地文件缺失：{lp}")
        r.run(f"mkdir -p {REMOTE}/{os.path.dirname(rel)}")
        r.put(lp, f"{REMOTE}/{rel}")
        got = r.run(f"md5sum {REMOTE}/{rel}").split()[0]
        ok = got == md5(lp)
        print(f"  {rel:34s} {'md5 一致' if ok else 'md5 不一致!'}")
        if not ok:
            raise SystemExit("[err] 源码推送校验失败")

    print("=== 3/4 编译（删产物强制重编） ===")
    out = build(r)
    print(out)
    if "BUILD_RC=0" not in out:
        raise SystemExit("[err] 编译失败 —— 已停止，不运行任何新二进制。回滚：--rollback")
    print("编译通过：" + r.run(f"ls -l {REMOTE}/build/dzuav").strip())


def smoke(r: R, stride: int = 5):
    print(f"=== 4/4 冒烟：视频 → 落格 → 规划（DZUAV_STRIDE={stride}） ===")
    work = f"/tmp/ag_plan13_smoke"
    r.run(f"rm -rf {work}; mkdir -p {work}")
    cmd = (f"cd {work} && DZUAV_STRIDE={stride} timeout 900 {REMOTE}/build/dzuav "
           f"{VIDEO} {HBB} {CELL} 50000 \"\" {work}/cell_matrix.json "
           f"{work}/route_plan.json {work}/planned_path.txt > {work}/run.log 2>&1; echo RC=$?")
    print(r.run(cmd, timeout=1000))
    print("--- 统计 ---")
    print(r.run(f"grep -o 'localized=[01]' {work}/run.log | sort | uniq -c; "
                f"grep -c '=== 路线规划 ===' {work}/run.log; "
                f"ls -l {work}/route_plan.json {work}/planned_path.txt 2>&1; "
                f"tail -6 {work}/route_plan.json 2>/dev/null", timeout=120))


def rollback(r: R):
    print("=== 回滚：还原备份 + 删除新增文件 ===")
    baks = r.run(f"find {REMOTE} -maxdepth 3 -name '*.bak_plan13_*' 2>/dev/null")
    if not baks:
        raise SystemExit("[err] 未找到备份文件（备份名含 .bak_plan13_ 时间戳）")
    for line in sorted(baks.splitlines()):   # 升序：较新的备份最后覆盖
        src = line.strip()
        dst = re.sub(r"\.bak_plan13_(orig|\d+)$", "", src)
        print(f"  {src} -> {dst}")
        r.run(f"mv -f {src} {dst}")
    for rel in NEW_FILES:
        r.run(f"rm -f {REMOTE}/{rel}")
    print(build(r))
    print("[done] 已回滚并重新编译")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="deploy_plan13.py",
                                 description="13 类主链接入规划层：部署/编译验证/冒烟/回滚")
    ap.add_argument("--host", default="192.168.0.197")
    ap.add_argument("--user", default="duzhong")
    ap.add_argument("--password", default=None)
    ap.add_argument("--stride", type=int, default=5, help="冒烟抽帧间隔（默认 5）")
    ap.add_argument("--smoke-only", action="store_true")
    ap.add_argument("--rollback", action="store_true")
    args = ap.parse_args(argv)
    pw = args.password or os.environ.get("AG_ORIN_PASS") or "duzhong"

    r = R(args.host, args.user, pw)
    try:
        if args.rollback:
            rollback(r)
        elif args.smoke_only:
            smoke(r, args.stride)
        else:
            deploy(r)
            smoke(r, args.stride)
        return 0
    finally:
        r.close()


if __name__ == "__main__":
    raise SystemExit(main())
