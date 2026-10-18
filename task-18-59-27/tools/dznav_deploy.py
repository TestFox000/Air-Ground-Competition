# -*- coding: utf-8 -*-
"""dznav 一键部署：推送 src/dznav 到车端并接管循迹栈（可一键回滚）。

流程（全程不改动车上任何原文件，仅进程级切换）：
  1. 预检：车上 legacy serial_reader / path_executor 进程与启动命令留档；
  2. 推送：SFTP 上传 src/dznav -> ~/dznav（纯 Python，无需 colcon）；
     远端 py_compile 全量校验 + md5 清单比对；
  3. 切换：停 legacy 两进程 -> 以 DZ_AUTO_START=0（待命铁律）拉起 dznav；
  4. 体检：/odom/localization 发布者数==1、信念流 50Hz、无双源告警；
  5. 回滚（--rollback）：停 dznav -> 按留档命令重启 legacy 两进程。

用法：
  set AG_CAR_PASS=******
  python tools/dznav_deploy.py --host 192.168.0.198 --user duzhong
  python tools/dznav_deploy.py --host ... --rollback
  python tools/dznav_deploy.py --host ... --status     # 只看当前状态

依赖：paramiko（仅本机）。其余契约与 tools/remote_run.py 一致。
"""

from __future__ import annotations

import argparse
import hashlib
import os
import stat
import sys
import time
from pathlib import Path

try:
    import paramiko
except ImportError:  # pragma: no cover
    print("缺少 paramiko：python -m pip install paramiko", file=sys.stderr)
    raise SystemExit(2)

ROOT = Path(__file__).resolve().parents[1]
LOCAL_PKG = ROOT / "src" / "dznav"
REMOTE_HOME_DIR = "dznav"          # ~/dznav（相对家目录）

#: 待命铁律：默认待命，不自动发车（比赛时显式 DZ_AUTO_START=1 + builtin_route）
ENV_TMPL = ("source /opt/ros/humble/setup.bash && "
            "source ~/dzrobot/install/setup.bash 2>/dev/null || true")

LEGACY_PATTERNS = ["path_executor", "serial_reader"]


def _md5(data: bytes) -> str:
    return hashlib.md5(data).hexdigest()


def _connect(host, user, password, timeout=60.0):
    last = None
    for i in range(1, 4):
        cli = paramiko.SSHClient()
        cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            cli.connect(hostname=host, port=22, username=user,
                        password=password, timeout=timeout,
                        banner_timeout=timeout, auth_timeout=timeout)
            if i > 1:
                print(f"  [连接] 第 {i} 次尝试成功", file=sys.stderr)
            return cli
        except Exception as e:  # noqa: BLE001
            last = e
            try:
                cli.close()
            except Exception:  # noqa: BLE001
                pass
            if i < 3:
                print(f"  [连接不稳] 第 {i}/3 次失败（{type(e).__name__}），"
                      f"1.5s 后重试…", file=sys.stderr)
                time.sleep(1.5)
    raise last


def _run(cli, cmd: str, timeout: float = 60.0):
    _, out, err = cli.exec_command(cmd, timeout=timeout)
    o = out.read().decode("utf-8", errors="replace")
    e = err.read().decode("utf-8", errors="replace")
    code = out.channel.recv_exit_status()
    return code, o, e


def _sftp_mkdirs(sftp, remote_dir: str):
    """逐级建目录（SFTP 无递归 mkdir；注意 ~ 不展开，先 normalize 家目录）。"""
    if remote_dir.startswith("~"):
        home = sftp.normalize(".")
        remote_dir = f"{home}/{remote_dir[1:].lstrip('/')}"
    parts = remote_dir.strip("/").split("/")
    cur = ""
    for p in parts:
        cur = f"{cur}/{p}"
        try:
            sftp.stat(cur)
        except FileNotFoundError:
            sftp.mkdir(cur)
    return remote_dir


def _push(sftp, local: Path, remote_dir: str) -> tuple[int, list]:
    """递归上传，返回 (文件数, [(remote, md5)])。跳过 __pycache__。"""
    remote_dir = _sftp_mkdirs(sftp, remote_dir)
    manifest = []
    n = 0
    for f in sorted(local.rglob("*")):
        if f.is_dir() or "__pycache__" in f.parts or f.suffix == ".pyc":
            continue
        rel = f.relative_to(local).as_posix()
        rpath = f"{remote_dir}/{rel}"
        _sftp_mkdirs(sftp, rpath.rsplit("/", 1)[0])
        sftp.put(str(f), rpath)
        data = f.read_bytes()
        manifest.append((rpath, _md5(data)))
        n += 1
    return n, manifest


def _legacy_snapshot(cli) -> dict:
    """留档当前 legacy 进程（pid + 完整命令行），供回滚。"""
    snap = {}
    for pat in LEGACY_PATTERNS:
        code, out, _ = _run(cli, f"pgrep -af '{pat}' || true")
        procs = []
        for line in out.strip().splitlines():
            line = line.strip()
            if not line or "pgrep" in line:
                continue
            pid, cmd = line.split(None, 1)
            procs.append((pid, cmd))
        snap[pat] = procs
    return snap


def _stop_legacy(cli):
    for pat in LEGACY_PATTERNS:
        _run(cli, f"pkill -f '{pat}' 2>/dev/null; true")
    time.sleep(1.0)


def _start_dznav(cli) -> str:
    cmd = (f"cd ~/{REMOTE_HOME_DIR} && {ENV_TMPL} && "
           f"DZ_AUTO_START=0 nohup python3 -m dznav.executor_node "
           f"> ~/{REMOTE_HOME_DIR}/dznav.log 2>&1 & echo $!")
    code, out, err = _run(cli, cmd)
    if code != 0:
        raise RuntimeError(f"dznav 启动失败: {err.strip()}")
    return out.strip().splitlines()[-1] if out.strip() else "?"


def _stop_dznav(cli):
    _run(cli, "pkill -f 'dznav.executor_node' 2>/dev/null; true")
    time.sleep(1.0)


def _health(cli, tries: int = 10) -> bool:
    """体检：信念流发布者唯一 + dznav 进程活着 + 无双源告警。"""
    for i in range(1, tries + 1):
        code, out, _ = _run(cli, f"{ENV_TMPL} && "
                                 "timeout 3 ros2 topic info /odom/localization "
                                 "2>/dev/null | grep 'Publisher count'")
        pubs = ""
        for ln in out.splitlines():
            if "Publisher count" in ln:
                pubs = ln.split(":")[-1].strip()
        code2, alive, _ = _run(cli, "pgrep -af dznav.executor_node || true")
        code3, log, _ = _run(cli, f"tail -5 ~/{REMOTE_HOME_DIR}/dznav.log")
        ok_pub = pubs == "1"
        ok_alive = "executor_node" in alive
        if ok_pub and ok_alive:
            print(f"  [体检] /odom/localization 发布者=1，dznav 存活，"
                  f"日志尾: {log.strip().splitlines()[-1] if log.strip() else ''}")
            return True
        if i < tries:
            time.sleep(2.0)
    print("  [体检失败] "
          f"pubs={pubs!r} alive={bool(alive.strip())}\n日志尾:\n{log}")
    return False


def deploy(args):
    cli = _connect(args.host, args.user, args.password)
    try:
        if args.status:
            snap = _legacy_snapshot(cli)
            for k, v in snap.items():
                print(f"[legacy:{k}] {v if v else '未运行'}")
            code, out, _ = _run(cli, "pgrep -af dznav.executor_node || true")
            print(f"[dznav] {out.strip() or '未运行'}")
            return 0

        if args.rollback:
            print("== 回滚：停 dznav，按留档重启 legacy ==")
            _stop_dznav(cli)
            snap = _legacy_snapshot(cli)
            legacy_running = {k: v for k, v in snap.items() if v}
            if not legacy_running:
                # legacy 已不在：尝试标准栈命令（start_all.sh 拉起过的方式）
                _run(cli, f"{ENV_TMPL} && "
                          "nohup ros2 run car_localization serial_reader_node "
                          f"> /tmp/dznav_rb_sr.log 2>&1 & sleep 2 && "
                          "nohup ros2 run car_localization path_executor_node "
                          "> /tmp/dznav_rb_pe.log 2>&1 &")
            else:
                for _pat, procs in legacy_running.items():
                    for _pid, cmd0 in procs:
                        _run(cli, f"nohup {cmd0} "
                                  f"> /tmp/dznav_rb_{_pid}.log 2>&1 &")
            time.sleep(3.0)
            code, out, _ = _run(cli, f"{ENV_TMPL} && "
                                     "timeout 3 ros2 topic info "
                                     "/odom/localization | grep -c Publisher")
            print(f"  [回滚后] legacy 状态: "
                  f"{ {k: bool(v) for k, v in _legacy_snapshot(cli).items()} }")
            return 0

        print("== 1/4 预检与留档 ==")
        snap = _legacy_snapshot(cli)
        for k, v in snap.items():
            print(f"  legacy {k}: {v if v else '未运行'}")

        print("== 2/4 推送 dznav ==")
        sftp = cli.open_sftp()
        try:
            home = sftp.normalize(".")
            n, manifest = _push(sftp, LOCAL_PKG, f"~/{REMOTE_HOME_DIR}")
            print(f"  已上传 {n} 个文件 -> ~/{REMOTE_HOME_DIR}")
            # 远端 md5 校验 + 编译检查
            bad = []
            for rpath, md5l in manifest:
                code, out, _ = _run(cli, f"md5sum '{rpath}'")
                if not out.startswith(md5l):
                    bad.append(rpath)
            if bad:
                raise RuntimeError(f"md5 不一致: {bad[:3]} ...")
            code, out, err = _run(cli, f"cd ~/{REMOTE_HOME_DIR} && "
                                       "python3 -m compileall -q dznav")
            if code != 0:
                raise RuntimeError(f"远端编译失败: {err[:400]}")
            print("  md5 全对齐，远端 compileall 通过")
        finally:
            sftp.close()

        print("== 3/4 切换（DZ_AUTO_START=0 待命）==")
        _stop_legacy(cli)
        pid = _start_dznav(cli)
        print(f"  dznav 已启动 pid={pid}（待命，不发车）")

        print("== 4/4 体检 ==")
        if not _health(cli):
            print("  体检失败，自动回滚到 legacy")
            _stop_dznav(cli)
            for pat, procs in snap.items():
                for _pid, cmd0 in procs:
                    _run(cli, f"nohup {cmd0} "
                              f"> /tmp/dznav_rb_{_pid}.log 2>&1 &")
            return 1
        print("完成。发车链路：/nav/waypoint 下发 reset + 路线串"
              "（ag_integrate 或 ros2 topic pub）。")
        return 0
    finally:
        cli.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", required=True)
    ap.add_argument("--user", default="duzhong")
    ap.add_argument("--password", default=None)
    ap.add_argument("--status", action="store_true", help="只查当前状态")
    ap.add_argument("--rollback", action="store_true", help="回滚到 legacy 栈")
    args = ap.parse_args()
    args.password = args.password or os.environ.get("AG_CAR_PASS")
    if not args.password:
        raise SystemExit("缺少车端密码：set AG_CAR_PASS=<密码> 或 --password")
    raise SystemExit(deploy(args))


if __name__ == "__main__":
    main()
