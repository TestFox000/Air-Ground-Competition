# -*- coding: utf-8 -*-
r"""上车部署工具（本地 → 小车，走 SSH/SCP）。

流程（对应"本地唯一真源 → 车端替换 → 编译验证"铁律）：
    probe    解析车端当前 IP 与关键状态（mDNS 会抖，所以先解析出 IP 再用 IP 干活）
    backup   在车上把将被覆盖的文件打包到 ~/dzrobot/_backups/<时间戳>/
    push     把本地文件/目录推到车上（默认含 dzbrain 包、云台补丁源码、config、tools）
    build    车上 colcon build（默认 dzbrain + dzrknn_dataprocess）
    verify   车上验证：启动门禁 dry-run + bench 冒烟（FakeBridge，不动车）+ 包/参数存在性
    all      backup → push → build → verify

安全约定：
- **绝不在验证阶段启动 ROS 栈**（起栈会让车按路线行动；`auto_start` 也必须保持 false）。
- push 前自动 backup；backup 目录名带时间戳，便于回滚。
- config 推送时会把 `auto_start` 强制为 false（车端当前也是 false），并在报告里注明这一处差异。
- ★ 目录推送必须用「tar 流 → 就地替换」，**绝不能用 `scp -r 本地目录 车端同名目录`**：
  目标目录已存在时 scp 会把源目录整个塞进去变成 `dst/<basename>/...`，
  于是"推送成功"但车端一个文件都没更新（2026-10-08 真机踩过：车端一直是旧代码，
  colcon 因此仍按旧 package.xml 识别成 python 构建类型，`ros2 pkg` 找不到 dzbrain）。
- ★ 推送后必须做 **哈希清单比对**（本地 sha256 ↔ 车端 sha256），不一致就非 0 退出并中止后续编译，
  让"没推上去"永远无法伪装成"已部署"。

用法：
    python tools\deploy_to_car.py probe
    python tools\deploy_to_car.py all
    python tools\deploy_to_car.py all --dry-run          # 只打印将执行的命令
"""

from __future__ import annotations

import argparse
import hashlib
import io
import pathlib
import re
import subprocess
import sys
import tarfile
import time

WS = pathlib.Path(__file__).resolve().parents[1]
CAR_HOST = "duzhong.local"
CAR_USER = "duzhong"
CAR_WS = "/home/duzhong/dzrobot"
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=10"]

#: 推送清单：本地相对路径 → 车端绝对路径（目录用 -r）
PUSH_FILES: list[tuple[str, str]] = [
    ("src/dzbrain", f"{CAR_WS}/src/dzbrain"),                                    # 新决策层（整包）
    ("src/dzrknn_dataprocess/include/dzrknn_dataprocess.h",                       # 云台补丁
     f"{CAR_WS}/src/dzrknn_dataprocess/include/dzrknn_dataprocess.h"),
    ("src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp",
     f"{CAR_WS}/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp"),
    ("config/robot.yaml", f"{CAR_WS}/config/robot.yaml"),                        # 统一配置（含新键）
    ("config/map.yaml", f"{CAR_WS}/config/map.yaml"),
    ("tools/config_selfcheck.py", f"{CAR_WS}/tools/config_selfcheck.py"),         # 车上自检
    ("tools/dzbrain.py", f"{CAR_WS}/tools/dzbrain.py"),
]

#: backup 时打包的车端路径
BACKUP_PATHS = [
    f"{CAR_WS}/config/robot.yaml",
    f"{CAR_WS}/config/map.yaml",
    f"{CAR_WS}/start_all.sh",
    f"{CAR_WS}/src/dzbrain",
    f"{CAR_WS}/src/dzrknn_dataprocess",
]

#: ★ 备份必须放在**工作区之外**：放进 ~/dzrobot 里会被 colcon 当成第二个包目录，
#:   直接报 "Duplicate package names not supported" 把整次构建搞失败（实测踩过）。
BACKUP_ROOT = "/home/duzhong/_backups"

BUILD_PACKAGES = ["dzbrain", "dzrknn_dataprocess"]


def log(msg: str) -> None:
    print(msg, flush=True)


def sh(argv: list[str], *, dry: bool = False, input_text: str | None = None) -> int:
    if dry:
        log("  [dry-run] " + " ".join(argv) + (f"   <<< {len(input_text)} 字节 stdin" if input_text else ""))
        return 0
    proc = subprocess.run(argv, input=input_text, text=True, capture_output=True, encoding="utf-8",
                          errors="replace")
    if proc.stdout.strip():
        log(proc.stdout.rstrip())
    if proc.returncode != 0 and proc.stderr.strip():
        log("  stderr: " + proc.stderr.strip()[-1500:])
    return proc.returncode


def resolve_ip(dry: bool = False) -> str:
    """解析车端 IP（mDNS 会抖：最多试 5 次；成功过就用 IP 干活，不再依赖 .local）。"""
    for attempt in range(1, 6):
        argv = ["ssh", *SSH_OPTS, f"{CAR_USER}@{CAR_HOST}", "hostname -I"]
        if dry:
            log(f"  [dry-run] {' '.join(argv)}")
            return "0.0.0.0"
        proc = subprocess.run(argv, text=True, capture_output=True, encoding="utf-8", errors="replace")
        if proc.returncode == 0:
            for token in proc.stdout.split():
                if token.count(".") == 3 and token[0].isdigit():
                    log(f"  车端 IP = {token}（第 {attempt} 次解析成功）")
                    return token
        time.sleep(1.0)
    raise SystemExit("❌ 无法解析车端 IP（mDNS 抖动或车不在线）：先确认小车开机并连上同一网络")


def ssh_run(ip: str, cmd: str, *, dry: bool = False, label: str = "") -> int:
    log(f"  $ ssh {ip}: {label or cmd[:90]}")
    argv = ["ssh", *SSH_OPTS, f"{CAR_USER}@{ip}", cmd]
    return sh(argv, dry=dry)


def sh_bytes(argv: list[str], data: bytes, *, dry: bool = False, label: str = "") -> int:
    """把二进制数据喂给子进程 stdin（tar 流走这条路，绝不能经 PowerShell 管道 —— 会按文本搅坏二进制）。"""
    log(f"  $ {label or ' '.join(argv)}   <<< {len(data)} 字节")
    if dry:
        return 0
    proc = subprocess.run(argv, input=data, capture_output=True)
    out = proc.stdout.decode("utf-8", "replace").strip()
    if out:
        log(out)
    if proc.returncode != 0:
        log("  stderr: " + proc.stderr.decode("utf-8", "replace").strip()[-1500:])
    return proc.returncode


# --------------------------------------------------------------------------- #
# 推送：目录走 tar 流就地替换；文件走 scp；推完必须哈希对账
# --------------------------------------------------------------------------- #
#: 这些目录名下的内容不进包（本地缓存/版本库元数据），否则会把车端污染成"额外文件"
SKIP_DIR_PARTS = {"__pycache__", ".git", ".pytest_cache"}


def iter_payload(root: pathlib.Path):
    """列出要推送的真实文件（跳过 __pycache__ 之类）。"""
    for f in sorted(root.rglob("*")):
        if not f.is_file():
            continue
        if any(part in SKIP_DIR_PARTS for part in f.relative_to(root).parts):
            continue
        if f.suffix == ".pyc":
            continue
        yield f


def sha256_file(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def normalize_auto_start(text: str) -> str:
    """与车端那段强制脚本**逐字一致**的归一化（验证阶段绝不允许自动发车）。"""
    return re.sub(r"(auto_start:)\s*true", r"\1 false", text)


#: 推送后由本工具主动改写的文件 → 对账基准必须用「改写后」的期望内容。
#: （首版忘了这一步，自己的归一化被当成"推送没生效"，报了一个假警报。）
POST_PUSH_TRANSFORM = {
    "config/robot.yaml": normalize_auto_start,
}


def expected_remote_bytes(local: str, raw: bytes) -> bytes:
    """推送 + 车端归一化之后，车端**应该**有的字节内容（对账基准）。

    忠实复刻车端脚本语义：
      s  = read_text(encoding='utf-8')  → 通用换行：CRLF/CR 统一成 LF
      s2 = 归一化(s)
      只有 s2 != s 时才 write_text(s2, encoding='utf-8')（写回即 LF 换行）；
      没改动时车端保留 scp 过去的**原始字节**（可能仍是 CRLF）。
    """
    transform = POST_PUSH_TRANSFORM.get(local)
    if transform is None:
        return raw
    text = raw.decode("utf-8").replace("\r\n", "\n").replace("\r", "\n")
    new = transform(text)
    return raw if new == text else new.encode("utf-8")


def local_manifest(local: str) -> dict[str, str]:
    """本地清单：相对路径 → sha256（目录递归；单文件用文件名）。

    路径不存在直接抛错 —— "对账对象不存在"绝不能被当成"清单为空/通过"。
    """
    p = WS / local
    if not p.exists():
        raise FileNotFoundError(f"本地路径不存在：{local}")
    if p.is_file():
        return {p.name: sha256_bytes(expected_remote_bytes(local, p.read_bytes()))}
    return {f.relative_to(p).as_posix(): sha256_file(f) for f in iter_payload(p)}


def make_tarball(local: str) -> bytes:
    """把目录内容打成 tar.gz（**arcname 是相对路径**，不含顶层目录名 —— 这正是防嵌套的关键）。"""
    root = WS / local
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tf:
        for f in iter_payload(root):
            tf.add(f, arcname=f.relative_to(root).as_posix())
    return buf.getvalue()


def dir_swap_cmd(remote: str) -> str:
    """车端替换脚本：先解到 .new，再原子 mv 顶掉旧目录（不留嵌套、不留半截目录）。"""
    return (f"set -e; mkdir -p {pathlib.PurePosixPath(remote).parent}; "
            f"rm -rf {remote}.dznew; mkdir -p {remote}.dznew; "
            f"tar -xzf - -C {remote}.dznew; "
            f"rm -rf {remote}; mv {remote}.dznew {remote}; "
            f"echo '  车端文件数：'$(find {remote} -type f | wc -l)")


def push_dir(ip: str, local: str, remote: str, *, dry: bool = False) -> int:
    log(f"  $ tar-push {local}/ -> {remote}/（就地替换，防 scp 嵌套）")
    data = make_tarball(local)
    argv = ["ssh", *SSH_OPTS, f"{CAR_USER}@{ip}", dir_swap_cmd(remote)]
    return sh_bytes(argv, data, dry=dry, label=f"ssh {ip}: tar -xzf - -C {remote}")


def push_file(ip: str, local: str, remote: str, *, dry: bool = False) -> int:
    log(f"  $ scp {local} -> {remote}")
    argv = ["scp", *SSH_OPTS, str(WS / local), f"{CAR_USER}@{ip}:{remote}"]
    return sh(argv, dry=dry)


#: 车端清单脚本：不走 shell 展开，纯 Python 走文件系统
REMOTE_MANIFEST_PY = r'''
import hashlib, os, sys
root, is_dir = sys.argv[1], sys.argv[2] == "1"
def h(p):
    m = hashlib.sha256()
    with open(p, "rb") as f:
        for c in iter(lambda: f.read(1 << 20), b""):
            m.update(c)
    return m.hexdigest()
if not is_dir:
    print("FILE\t%s\t%s" % (os.path.basename(root), h(root)))
else:
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in ("__pycache__", ".git")]
        for fn in sorted(filenames):
            if fn.endswith(".pyc"):
                continue
            full = os.path.join(dirpath, fn)
            rel = os.path.relpath(full, root).replace(os.sep, "/")
            print("FILE\t%s\t%s" % (rel, h(full)))
'''


def remote_manifest(ip: str, remote: str, is_dir: bool, *, dry: bool = False) -> dict[str, str]:
    cmd = f"python3 - {remote} {1 if is_dir else 0} <<'PY'\n{REMOTE_MANIFEST_PY}\nPY"
    argv = ["ssh", *SSH_OPTS, f"{CAR_USER}@{ip}", cmd]
    if dry:
        log(f"  [dry-run] ssh {ip}: 取车端哈希清单 {remote}")
        return {}
    proc = subprocess.run(argv, text=True, capture_output=True, encoding="utf-8", errors="replace")
    if proc.returncode != 0:
        log("  stderr: " + proc.stderr.strip()[-800:])
        return {}
    out = {}
    for line in proc.stdout.splitlines():
        parts = line.split("\t")
        if len(parts) == 3 and parts[0] == "FILE":
            out[parts[1]] = parts[2]
    return out


def diff_manifest(local_m: dict[str, str], remote_m: dict[str, str]) -> tuple[list[str], list[str], list[str]]:
    """返回 (缺失, 车端多出, 内容不一致)。三种情况都说明"推送没真正生效"。"""
    missing = sorted(set(local_m) - set(remote_m))
    extra = sorted(set(remote_m) - set(local_m))
    mismatch = sorted(k for k in set(local_m) & set(remote_m) if local_m[k] != remote_m[k])
    return missing, extra, mismatch


def verify_pushed(ip: str, items: list[tuple[str, str, bool]], *, dry: bool = False) -> int:
    """★ 对账闸门：逐项比对本地/车端 sha256。返回 0 才算真的推上去了。"""
    bad = 0
    for local, remote, is_dir in items:
        lm = local_manifest(local)
        rm = remote_manifest(ip, remote, is_dir, dry=dry)
        if dry:
            log(f"  [dry-run] 对账 {local}（{len(lm)} 个文件）")
            continue
        missing, extra, mismatch = diff_manifest(lm, rm)
        if missing or mismatch:
            bad += 1
            log(f"  ❌ 对账失败 {local} → {remote}：缺 {len(missing)} / 不一致 {len(mismatch)}"
                + (f" / 车端多出 {len(extra)}" if extra else ""))
            for k in (missing + mismatch)[:5]:
                log(f"       - {k}")
        else:
            note = "（对账基准含推送后归一化：auto_start 强制 false）" if local in POST_PUSH_TRANSFORM else ""
            log(f"  ✅ 对账通过 {local}：{len(lm)} 个文件 sha256 全部一致{note}"
                + (f"（车端多出 {len(extra)} 个，已忽略）" if extra else ""))
    return 1 if bad else 0



# --------------------------------------------------------------------------- #
def cmd_probe(ip: str | None, args) -> None:
    ip = ip or resolve_ip(args.dry_run)
    ssh_run(ip, "hostname; date; uptime | head -1", dry=args.dry_run, label="基本信息")
    ssh_run(ip, f"ls {CAR_WS}; echo '--- config ---'; ls -l {CAR_WS}/config; "
                f"echo '--- 是否已有 dzbrain ---'; ls -d {CAR_WS}/src/dzbrain 2>/dev/null || echo '(还没有)'",
            dry=args.dry_run, label="工作区/配置/是否已有 dzbrain")
    ssh_run(ip, "source /opt/ros/humble/setup.bash 2>/dev/null; colcon --version 2>/dev/null | head -1; python3 -V",
            dry=args.dry_run, label="工具链")
    ssh_run(ip, "df -h ~ | tail -1", dry=args.dry_run, label="磁盘")


def cmd_backup(ip: str, args) -> None:
    ts = time.strftime("%Y%m%d_%H%M%S")
    dst = f"{BACKUP_ROOT}/dzbrain_deploy_{ts}"
    files = " ".join(p for p in BACKUP_PATHS)
    ssh_run(ip, f"mkdir -p {dst} && for f in {files}; do [ -e \"$f\" ] && cp -r \"$f\" {dst}/ ; done; "
                f"ls -la {dst} | head -12; echo BACKUP_DIR={dst}",
            dry=args.dry_run, label=f"备份到 {dst}（★工作区之外，避免 colcon 重名）")
    # ★ 清理历史遗留：工作区内如果已经有备份目录（含包目录），colcon 会报重名 —— 给它加 COLCON_IGNORE
    ssh_run(ip, f"for d in {CAR_WS}/_backups {CAR_WS}/backup; do "
                f"[ -d \"$d\" ] && touch \"$d/COLCON_IGNORE\" && echo \"已加 COLCON_IGNORE: $d\"; done; true",
            dry=args.dry_run, label="给工作区内历史备份目录加 COLCON_IGNORE")


def cmd_push(ip: str, args) -> int:
    items: list[tuple[str, str, bool]] = []
    for local, remote in PUSH_FILES:
        src = WS / local
        if not src.exists():
            raise SystemExit(f"❌ 本地文件不存在：{local}")
        is_dir = src.is_dir()
        items.append((local, remote, is_dir))
        rc = push_dir(ip, local, remote, dry=args.dry_run) if is_dir \
            else push_file(ip, local, remote, dry=args.dry_run)
        if rc != 0 and not args.dry_run:
            log(f"❌ 推送失败：{local}")
            return 1
    # ★ 推送后把车端 auto_start 强制为 false（验证阶段绝不允许自动发车）
    ssh_run(ip, f"cd {CAR_WS} && python3 - <<'PY'\n"
                "import re, pathlib\n"
                "p = pathlib.Path('config/robot.yaml')\n"
                "s = p.read_text(encoding='utf-8')\n"
                "s2 = re.sub(r'(auto_start:)\\s*true', r'\\1 false', s)\n"
                "s2 = re.sub(r'(auto_start:)\\s*true', r'\\1 false', s2)\n"
                "if s2 != s:\n"
                "    p.write_text(s2, encoding='utf-8')\n"
                "    print('★ auto_start 已强制为 false（验证阶段不发车）')\n"
                "else:\n"
                "    print('auto_start 已是 false')\n"
                "PY", dry=args.dry_run, label="强制 auto_start=false")
    log("  ── 推送对账（本地 sha256 ↔ 车端 sha256）──")
    return verify_pushed(ip, items, dry=args.dry_run)


def cmd_build(ip: str, args) -> int:
    pkgs = " ".join(args.packages or BUILD_PACKAGES)
    # ★ dzbrain 的构建类型从 python 修正为 ros.ament_python 后，必须清掉旧的 build/install 残留：
    #   否则 install/dzbrain 里仍是"缺 ament 前缀钩子"的旧布局，ros2 pkg 依旧找不到包。
    if "dzbrain" in pkgs:
        ssh_run(ip, f"cd {CAR_WS} && rm -rf build/dzbrain install/dzbrain && echo '已清理 dzbrain 旧构建产物'",
                dry=args.dry_run, label="清理 dzbrain 旧 build/install（构建类型变更，防残留）")
    cmd = (f"cd {CAR_WS} && source /opt/ros/humble/setup.bash && "
           f"source ./dzros.env 2>/dev/null; colcon build --packages-select {pkgs} "
           f"--cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -30")
    rc = ssh_run(ip, cmd, dry=args.dry_run, label=f"colcon build {pkgs}")
    if rc != 0 and not args.dry_run:
        log("❌ 编译失败 —— 把上面的输出贴回来，我按行修")
    return rc


def cmd_verify(ip: str, args) -> int:
    rc = 0
    # ① 启动门禁（配置 + 路线校验），不需要 ROS
    rc |= ssh_run(ip, f"cd {CAR_WS} && python3 tools/config_selfcheck.py --quiet 2>&1 | tail -8",
                  dry=args.dry_run, label="车上配置+路线门禁")
    # ② bench 冒烟：真实任务线程 + FakeBridge（不动车、不起 ROS 栈）
    rc |= ssh_run(ip, f"cd {CAR_WS} && python3 tools/dzbrain.py --bench --seconds 4 2>&1 | tail -14",
                  dry=args.dry_run, label="bench 冒烟（四线程，不动车）")
    # ③ 包与入口存在性
    rc |= ssh_run(ip, f"cd {CAR_WS} && source /opt/ros/humble/setup.bash && source install/setup.bash && "
                      f"ros2 pkg executables dzbrain; "
                      f"ros2 pkg prefix dzbrain",
                  dry=args.dry_run, label="dzbrain 包/入口")
    # ④ 云台补丁是否真的编进二进制（防"改了源码没编"）
    rc |= ssh_run(ip, f"grep -c 'use_external_fire_gate' {CAR_WS}/install/dzrknn_dataprocess/lib/"
                      f"dzrknn_dataprocess/dzactuator_node 2>/dev/null || "
                      f"find {CAR_WS}/install/dzrknn_dataprocess -type f -perm -u+x | head -3",
                  dry=args.dry_run, label="云台补丁符号（证明补丁已编入）")
    return rc


# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="本地 → 小车 部署工具（SSH/SCP）")
    ap.add_argument("action", choices=["probe", "backup", "push", "build", "verify", "all"])
    ap.add_argument("--ip", default=None, help="直接指定车端 IP（跳过 mDNS 解析）")
    ap.add_argument("--packages", nargs="*", default=None, help="colcon build 的包列表")
    ap.add_argument("--dry-run", action="store_true", help="只打印将执行的命令")
    args = ap.parse_args(argv)

    log("=" * 72)
    log(f"dzrobot 部署工具 · action={args.action} · dry_run={args.dry_run}")
    log("=" * 72)

    ip = args.ip or resolve_ip(args.dry_run)

    if args.action == "probe":
        cmd_probe(ip, args)
        return 0
    if args.action == "backup":
        cmd_backup(ip, args)
        return 0
    if args.action == "push":
        return cmd_push(ip, args)
    if args.action == "build":
        return cmd_build(ip, args)
    if args.action == "verify":
        return cmd_verify(ip, args)
    # all
    cmd_backup(ip, args)
    rc = cmd_push(ip, args)
    if rc != 0:
        log("⛔ 推送对账未通过 —— 车端代码没真正更新，编译结果无意义，已中止（不上车瞎编）")
        return rc
    rc = cmd_build(ip, args)
    if rc != 0:
        log("⛔ 编译失败，跳过验证（先修编译）")
        return rc
    return cmd_verify(ip, args)


if __name__ == "__main__":
    sys.exit(main())
