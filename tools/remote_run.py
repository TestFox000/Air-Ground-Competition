"""远程在车端/机载端执行命令（SSH，免装客户端工具）。

用途：把 `tools/probe_car.sh` 这类只读探测脚本送过去跑一遍，把输出抓回来，
省得在车的小屏终端上敲命令、再人肉誊写。

用法：
    # 1) 密码走环境变量（推荐，不会被写进历史）
    set AG_CAR_PASS=******
    python tools/remote_run.py --host 10.15.166.188 --user duzhong --script tools/probe_car.sh

    # 2) 直接跑一条命令
    python tools/remote_run.py --host 10.15.166.188 --user duzhong --cmd "ros2 pkg list | head"

    # 3) 输出同时存盘
    python tools/remote_run.py ... --out probe_report.txt

依赖：pip install paramiko（仅本机需要，车端不需要装任何东西）

安全提示：本工具只做「上传脚本 → 执行 → 回传输出」，不会改车端配置。
密码建议走 AG_CAR_PASS 环境变量；不要写进任何文件。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

try:
    import paramiko
except ImportError:  # pragma: no cover - 仅在缺依赖时触发
    print("缺少 paramiko，请先执行: python -m pip install paramiko", file=sys.stderr)
    raise SystemExit(2)


def read_password(args) -> str:
    pw = args.password or os.environ.get("AG_CAR_PASS")
    if pw:
        return pw
    import getpass

    return getpass.getpass(f"{args.user}@{args.host} 的密码: ")


def run(host: str, user: str, password: str, cmd: str, timeout: float,
        port: int = 22) -> tuple[int, str, str]:
    """执行一条远程命令，返回 (退出码, stdout, stderr)。"""
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(hostname=host, port=port, username=user, password=password,
                timeout=timeout, banner_timeout=timeout, auth_timeout=timeout)
    try:
        stdin, stdout, stderr = cli.exec_command(cmd, timeout=timeout, get_pty=False)
        stdin.close()
        out = stdout.read().decode("utf-8", errors="replace")
        err = stderr.read().decode("utf-8", errors="replace")
        code = stdout.channel.recv_exit_status()
        return code, out, err
    finally:
        cli.close()


def sftp_get(host: str, user: str, password: str, remote: str, local: str,
             port: int = 22, timeout: float = 60.0) -> int:
    """把远程文件/目录（递归）取回本地，返回文件数。"""
    import stat as stat_mod

    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(hostname=host, port=port, username=user, password=password,
                timeout=timeout, banner_timeout=timeout, auth_timeout=timeout)
    n = 0
    try:
        sftp = cli.open_sftp()
        src = Path(remote)
        dst = Path(local)

        def walk(r: str, l: Path) -> None:
            nonlocal n
            l.mkdir(parents=True, exist_ok=True)
            for entry in sftp.listdir_attr(r):
                rp = f"{r}/{entry.filename}"
                lp = l / entry.filename
                if stat_mod.S_ISDIR(entry.st_mode):
                    walk(rp, lp)
                else:
                    lp.parent.mkdir(parents=True, exist_ok=True)
                    sftp.get(rp, str(lp))
                    n += 1

        try:
            st = sftp.stat(remote)
            if stat_mod.S_ISDIR(st.st_mode):
                walk(remote, dst / src.name)
            else:
                dst.parent.mkdir(parents=True, exist_ok=True)
                sftp.get(remote, str(dst))
                n = 1
        finally:
            sftp.close()
    finally:
        cli.close()
    return n


def sftp_put(host: str, user: str, password: str, local: str, remote: str,
             port: int = 22, timeout: float = 60.0,
             skip: tuple = ("__pycache__", ".pytest_cache", ".git")) -> int:
    """把本地文件/目录（递归）上传到远程，返回文件数。"""
    import stat as stat_mod

    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(hostname=host, port=port, username=user, password=password,
                timeout=timeout, banner_timeout=timeout, auth_timeout=timeout)
    n = 0
    try:
        sftp = cli.open_sftp()

        def mkdirs(path: str) -> None:
            parts = path.strip("/").split("/")
            cur = ""
            for p in parts:
                cur += "/" + p
                try:
                    sftp.stat(cur)
                except IOError:
                    sftp.mkdir(cur)

        src = Path(local)
        if src.is_dir():
            base = remote.rstrip("/")
            mkdirs(base)

            def walk(d: Path, r: str) -> None:
                nonlocal n
                for child in sorted(d.iterdir()):
                    if child.name in skip:
                        continue
                    rp = f"{r}/{child.name}"
                    if child.is_dir():
                        mkdirs(rp)
                        walk(child, rp)
                    else:
                        sftp.put(str(child), rp)
                        n += 1

            walk(src, base)
        else:
            # ⚠️ 远程路径必须按 POSIX 处理：用 pathlib 会得到 Windows 的 '\'，
            # 传到 Linux 上就变成「文件名里带反斜杠」→ PermissionError。
            parent = remote.rsplit("/", 1)[0] if "/" in remote else "."
            if parent not in ("", "."):
                mkdirs(parent)
            sftp.put(str(src), remote)
            n = 1
        sftp.close()
    finally:
        cli.close()
    return n


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="SSH 远程执行（车端探测用）")
    ap.add_argument("--host", required=True, help="车端/机载端 IP，如 10.15.166.188")
    ap.add_argument("--user", default="duzhong", help="登录用户名（渡众车默认 duzhong）")
    ap.add_argument("--password", default=None, help="密码；更推荐用 AG_CAR_PASS 环境变量")
    ap.add_argument("--port", type=int, default=22)
    ap.add_argument("--script", default=None, help="上传并执行的本地脚本（sh）")
    ap.add_argument("--cmd", default=None, help="直接执行的远程命令")
    ap.add_argument("--get", default=None, help="取回远程文件/目录（递归），配合 --to 指定本地位置")
    ap.add_argument("--to", default=None, help="--get 的本地目标路径")
    ap.add_argument("--put", default=None, help="上传本地文件/目录（递归）到远程，配合 --dest 指定远程路径")
    ap.add_argument("--dest", default=None, help="--put 的远程目标路径")
    ap.add_argument("--out", default=None, help="把标准输出保存到本地文件")
    ap.add_argument("--env", action="append", default=None, metavar="K=V",
                    help="给远程脚本传环境变量，可重复（如 --env AG_LOG_DIR=/tmp/ag_logs）")
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args(argv)

    if args.get:
        if not args.to:
            ap.error("--get 需要同时给 --to")
        try:
            cnt = sftp_get(args.host, args.user, read_password(args), args.get, args.to,
                           args.port, args.timeout)
        except paramiko.AuthenticationException:
            print("认证失败：用户名或密码不对", file=sys.stderr)
            return 3
        except Exception as e:  # noqa: BLE001
            print(f"拉取失败: {type(e).__name__}: {e}", file=sys.stderr)
            return 4
        print(f"[已取回] {cnt} 个文件 -> {args.to}")
        return 0

    if args.put:
        if not args.dest:
            ap.error("--put 需要同时给 --dest")
        try:
            cnt = sftp_put(args.host, args.user, read_password(args), args.put, args.dest,
                           args.port, args.timeout)
        except paramiko.AuthenticationException:
            print("认证失败：用户名或密码不对", file=sys.stderr)
            return 3
        except Exception as e:  # noqa: BLE001
            print(f"上传失败: {type(e).__name__}: {e}", file=sys.stderr)
            return 4
        print(f"[已上传] {cnt} 个文件 -> {args.dest}")
        return 0

    if not args.script and not args.cmd:
        ap.error("至少给一个 --script / --cmd / --get / --put")

    if args.script:
        text = Path(args.script).read_text(encoding="utf-8")
        remote = f"/tmp/{Path(args.script).name}.sh"
        # 用 base64 传输，避免换行/编码在 exec_command 里被吃掉
        import base64

        payload = base64.b64encode(text.encode("utf-8")).decode("ascii")
        cmd = (f"echo {payload} | base64 -d > {remote} && chmod +x {remote} && bash {remote}")
    else:
        cmd = args.cmd

    if args.env:
        # 用 `export` 而不是 `K=V cmd`：远程命令常常是 `a && b && c` 的复合串，
        # 前缀赋值只会作用到第一个命令上，export 才是对整个 shell 生效。
        import shlex

        parts = []
        for item in args.env:
            if "=" not in item:
                ap.error(f"--env 需要 KEY=VALUE 形式，收到: {item}")
            k, v = item.split("=", 1)
            parts.append(f"export {k}={shlex.quote(v)}")
        cmd = "; ".join(parts) + "; " + cmd

    try:
        code, out, err = run(args.host, args.user, read_password(args), cmd, args.timeout, args.port)
    except paramiko.AuthenticationException:
        print("认证失败：用户名或密码不对（车端默认用户名 duzhong，密码见 05_VNC远程桌面使用方法）",
              file=sys.stderr)
        return 3
    except Exception as e:  # noqa: BLE001 - 网络/超时等一律原样报出，便于现场判断
        print(f"连接失败: {type(e).__name__}: {e}", file=sys.stderr)
        return 4

    if args.out:
        Path(args.out).write_text(out, encoding="utf-8")
        print(f"[已保存] {args.out}（{len(out)} 字节）")
    sys.stdout.write(out)
    if err.strip():
        sys.stderr.write("\n--- stderr ---\n" + err)
    print(f"\n[远程退出码] {code}")
    return 0 if code == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
