# -*- coding: utf-8 -*-
"""把机上 ~/dzuav_deploy/air_ground 收进版本控制（一键，含 Gitee 快照存证）。

背景：机上仓库长期 NO_GIT，多副本漂移是历史事故根因（项目铁律）。本脚本：
  1. git init（分支 main，仓库级身份配置，不动全局）；
  2. 写 .gitignore（大二进制/构建产物/备份锚不入库）+ AGENTS.md（团队规约）
     + BINARY_MANIFEST.md5（被忽略二进制的 md5 锁，漂移可 md5sum -c 检出）；
  3. 造两段真实历史：
       commit1 = 收编时的原始状态（用 .bak_align_orig 还原靶向对齐之前；
                 含 10-22 C++ 规划接入与 --sahi 演进——都是收编现状的一部分）
       commit2 = 2026-10-23 靶向对齐 + 最新进度重规划（从本地 work/ 推回）
  4. git bundle 回传本机 → 推 Gitee 快照分支 snapshot/orin-20261023
     （令牌只走环境变量，绝不落盘/落机上）。

安全边界：
  * 不触碰正在运行的侦察进程的文件（video_recon.py 不在换装清单）；
  * 全程 md5 校验，任何一步不符即停；
  * 机上不设 git remote（凭据不上机）。

用法：
  set GITEE_TOKEN=...   # 可选：设置后额外推 Gitee 快照分支
  python git_init_orin.py --host 192.168.0.197
  python git_init_orin.py --host ... --skip-gitee     # 只做机上收编
"""
from __future__ import annotations

import argparse
import hashlib
import os
import pathlib
import subprocess
import sys
import tempfile

import paramiko

BASE = pathlib.Path(__file__).resolve().parent
WORK = BASE / "work"
REMOTE = "/home/duzhong/dzuav_deploy/air_ground"
BUNDLE_LOCAL = BASE / "ag_orin_20261023.bundle"
SNAPSHOT_BRANCH = "snapshot/orin-20261023"

GITIGNORE = """\
# 大二进制不入库（血缘由 BINARY_MANIFEST.md5 锁定，md5sum -c 可检漂移）
engines/
models/
*.engine
*.onnx
*.pt
# 构建产物
build/
*.o
# Python 缓存
__pycache__/
*.pyc
# 测试素材（留在盘上，属输入不属于源）
*.jpg
*.mp4
*.avi
*.log
# 部署工具的回滚锚与目录快照（盘上保留；回滚自此改走 git）
*.bak_*
"""

# 换装清单：commit1 用 .bak_align_orig 还原；commit2 从本地 work/ 推回
RESTORE = [  # (备份, 现役)
    (f"{REMOTE}/src/ag_integrate/ag_integrate/mission_router.py.bak_align_orig",
     f"{REMOTE}/src/ag_integrate/ag_integrate/mission_router.py"),
    (f"{REMOTE}/src/ag_integrate/ag_integrate/integrate_node.py.bak_align_orig",
     f"{REMOTE}/src/ag_integrate/ag_integrate/integrate_node.py"),
    (f"{REMOTE}/tools/orin_plan_sidecar.py.bak_align_orig",
     f"{REMOTE}/tools/orin_plan_sidecar.py"),
]
NEW_FILE = f"{REMOTE}/src/ag_integrate/ag_integrate/route_progress.py"  # commit1 时移除
PUSH_BACK = [  # (本地 work 文件, 机上路径)
    (WORK / "ag_integrate" / "mission_router.py",
     f"{REMOTE}/src/ag_integrate/ag_integrate/mission_router.py"),
    (WORK / "ag_integrate" / "integrate_node.py",
     f"{REMOTE}/src/ag_integrate/ag_integrate/integrate_node.py"),
    (WORK / "ag_integrate" / "route_progress.py", NEW_FILE),
    (WORK / "tools" / "orin_plan_sidecar.py",
     f"{REMOTE}/tools/orin_plan_sidecar.py"),
]

COMMIT1 = (
    "收编: 机上13类部署线原始状态 (NO_GIT漂移期结束)\n"
    "\n"
    "含 10-22 C++规划接入 dzuav_mjpg、video_recon --sahi 演进、\n"
    "靶向对齐部署前状态。大二进制不入库, 由 BINARY_MANIFEST.md5 锁定。\n"
    "改前基线同步存证于 Gitee 个人档 20261023-shoot-alignment 交付。"
)
COMMIT2 = (
    "靶向对齐 + 重规划基于最新进度\n"
    "\n"
    "- mission_router: 停靠段终点态约束对齐到达(一次到位), 物理不可行\n"
    "  才降级停靠转身并显式告警\n"
    "- route_progress(新增): 0x4002车位姿x路线折线 -> 当前格/已扫集\n"
    "- integrate_node/sidecar: 中途重规划换到车当前格+剔已扫雷\n"
    "详见 Gitee: 20261023-shoot-alignment-progress-replan"
)


def md5_local(p: pathlib.Path) -> str:
    return hashlib.md5(p.read_bytes()).hexdigest()


class R:
    def __init__(self, host, user, pw):
        self.c = paramiko.SSHClient()
        self.c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.c.connect(host, username=user, password=pw, timeout=15)

    def run(self, cmd, timeout=300):
        _, o, e = self.c.exec_command(cmd, timeout=timeout)
        out = (o.read().decode("utf-8", "replace")
               + e.read().decode("utf-8", "replace")).strip()
        rc = o.channel.recv_exit_status()
        return rc, out

    def put(self, local, remote):
        sftp = self.c.open_sftp()
        try:
            sftp.put(str(local), remote)
        finally:
            sftp.close()

    def get(self, remote, local):
        sftp = self.c.open_sftp()
        try:
            sftp.get(remote, str(local))
        finally:
            sftp.close()

    def close(self):
        self.c.close()


def commit(r: R, msg: str) -> None:
    rc, out = r.run(
        f'cd {REMOTE} && git add -A && git commit -m "$(cat <<\'EOF\'\n{msg}\nEOF\n)"')
    if rc != 0:
        raise SystemExit(f"[err] commit 失败:\n{out}")
    print(out.splitlines()[0])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.0.197")
    ap.add_argument("--user", default="duzhong")
    ap.add_argument("--password", default="duzhong")
    ap.add_argument("--skip-gitee", action="store_true")
    args = ap.parse_args(argv)

    r = R(args.host, args.user, args.password)
    try:
        rc, _ = r.run(f"test -d {REMOTE}/.git && echo Y || echo N")
        if rc == 0 and _.strip() == "Y":
            raise SystemExit("[err] 机上已有 .git，拒绝重复初始化（人工检查）")

        print("=== 1/6 init + .gitignore + AGENTS.md + 身份（仓库级） ===")
        r.run(f'cd {REMOTE} && git init -b main')
        r.run(f'cd {REMOTE} && git config user.name "yu-binghuan" '
              f'&& git config user.email "yu-binghuan@users.noreply.gitee.com"')
        agents_path = pathlib.Path(os.environ["TEMP"]) / "AGENTS.md"
        # 去 BOM（PowerShell Out-File 会带 UTF-8 BOM，git 里会显示成 \ufeff）
        agents_bytes = agents_path.read_bytes()
        if agents_bytes.startswith(b"\xef\xbb\xbf"):
            agents_bytes = agents_bytes[3:]
        sftp = r.c.open_sftp()
        with sftp.file(f"{REMOTE}/.gitignore", "w") as f:
            f.write(GITIGNORE.encode("utf-8"))
        with sftp.file(f"{REMOTE}/AGENTS.md", "w") as f:
            f.write(agents_bytes)
        sftp.close()

        print("=== 2/6 大二进制清单（md5 锁） ===")
        rc, out = r.run(
            f'cd {REMOTE} && {{ echo "# 被忽略二进制的 md5 锁（引擎/模型不入库，'
            f'漂移用 md5sum -c BINARY_MANIFEST.md5 检出）"; '
            f'find engines models -type f 2>/dev/null | sort | xargs md5sum; }} '
            f'> BINARY_MANIFEST.md5 && wc -l BINARY_MANIFEST.md5')
        print(out)

        print("=== 3/6 还原收编时点状态（commit1：原始状态） ===")
        for bak, cur in RESTORE:
            rc, out = r.run(f"cp {bak} {cur} && md5sum {bak} {cur}")
            h1, h2 = out.splitlines()
            if h1.split()[0] != h2.split()[0]:
                raise SystemExit(f"[err] 还原校验失败: {cur}")
        r.run(f"rm -f {NEW_FILE}")
        commit(r, COMMIT1)

        print("=== 4/6 推回今日改动（commit2：靶向对齐） ===")
        for local, remote_path in PUSH_BACK:
            r.put(local, remote_path)
            rc, out = r.run(f"md5sum {remote_path}")
            if out.split()[0] != md5_local(local):
                raise SystemExit(f"[err] 推回校验失败: {remote_path}")
        commit(r, COMMIT2)

        print("=== 5/6 验证 ===")
        rc, out = r.run(f"cd {REMOTE} && git log --oneline && "
                        f"echo '--- status ---' && git status --short | head -5 && "
                        f"echo '--- 入库文件数/体积 ---' && "
                        f"git ls-files | wc -l && "
                        f"git ls-files -z | xargs -0 du -ch 2>/dev/null | tail -1")
        print(out)

        print("=== 6/6 bundle 回传 ===")
        r.run(f"cd {REMOTE} && git bundle create /tmp/ag_orin.bundle --all")
        r.get("/tmp/ag_orin.bundle", str(BUNDLE_LOCAL))
        r.run("rm -f /tmp/ag_orin.bundle")
        print(f"bundle -> {BUNDLE_LOCAL} "
              f"({BUNDLE_LOCAL.stat().st_size // 1024} KB)")
    finally:
        r.close()

    if args.skip_gitee:
        print("[done] 机上收编完成（--skip-gitee：未推 Gitee）")
        return 0

    token = os.environ.get("GITEE_TOKEN") or ""
    if not token:
        print("[warn] 未设 GITEE_TOKEN：bundle 已存本地，跳过 Gitee 快照")
        return 0

    print("=== Gitee 快照分支 ===")
    with tempfile.TemporaryDirectory() as td:
        repo = pathlib.Path(td) / "ag"
        subprocess.run(["git", "clone", "-q", str(BUNDLE_LOCAL), str(repo)],
                       check=True)
        url = f"https://oauth2:{token}@gitee.com/circle-of-joy/air_ground.git"
        # 令牌经 argv 瞬态传递，不落盘；进程结束即失效
        p = subprocess.run(
            ["git", "push", url, f"main:refs/heads/{SNAPSHOT_BRANCH}"],
            cwd=str(repo), capture_output=True, text=True, timeout=300)
        if p.returncode == 0:
            print(f"[OK] 已推 {SNAPSHOT_BRANCH}")
        else:
            print(f"[warn] 快照分支推送失败（可能无分支权限）:\n{p.stderr[-400:]}")
            print("      bundle 已在本地存证，可稍后人工推送")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
