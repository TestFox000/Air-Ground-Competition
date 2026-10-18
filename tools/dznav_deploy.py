# -*- coding: utf-8 -*-
"""dznav 一键部署：推送 src/dznav 到车端并接管循迹栈（可一键回滚）。

流程（全程不改动车上任何原文件，仅进程级切换）：
  1. 预检：车上 legacy serial_reader / path_executor 进程与启动命令留档，
     并把留档落到车端 ~/dznav/legacy_snapshot.json（--rollback 在另一个
     进程里跑，内存留档到那时已经不存在了）；
  2. 推送：SFTP 上传 src/dznav -> ~/dznav（纯 Python，无需 colcon）；
     远端 py_compile 全量校验 + md5 清单比对；
  3. 切换：停 legacy 两进程 -> 以 DZ_AUTO_START=0（待命铁律）拉起 dznav，
     并导出 DZ_NAV_CONFIG 指向随包推送的 config/dznav_default.yaml；
     启动抛异常时立刻按留档恢复 legacy（切换窗口内不许留瘫车）；
  4. 体检：/odom/localization 发布者数==1、信念流 50Hz、无双源告警；
     失败同样自动回滚；
  5. 回滚（--rollback）：停 dznav -> 按**落档**命令重启 legacy 两进程。

用法：
  set AG_CAR_PASS=******
  python tools/dznav_deploy.py --host 192.168.0.198 --user duzhong
  python tools/dznav_deploy.py --host ... --rollback
  python tools/dznav_deploy.py --host ... --status     # 只看当前状态
  python tools/dznav_deploy.py --host ... --config '~/dznav/config/race.yaml'

依赖：paramiko（仅本机）。其余契约与 tools/remote_run.py 一致。
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import re
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

#: 车端配置：随包一起推上去（src/dznav/config -> ~/dznav/config）。
#: 原先 dznav_deploy.py 从不导出 DZ_NAV_CONFIG，这份 yaml 上传了却永远
#: 不被加载 —— 车上跑的其实是 executor_node.DEFAULTS 里的硬编码值，
#: 改 yaml 毫无效果且无任何提示。
REMOTE_CONFIG = f"~/{REMOTE_HOME_DIR}/config/dznav_default.yaml"


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
            parts = line.split(None, 1)
            if len(parts) < 2 or not parts[1].strip():
                continue          # 只有 pid 没有命令行：回滚无从下手，跳过
            procs.append((parts[0], parts[1]))
        snap[pat] = procs
    return snap


#: legacy 留档落地路径。--rollback 通常在一个**新的** deploy 进程里跑，
#: 那时 legacy 早已被停掉、内存里的留档也不存在了 —— 原先的"按留档命令
#: 重启"必然退化成硬编码兜底命令，start_all.sh 传过的任何参数都丢失。
SNAPSHOT_PATH = f"~/{REMOTE_HOME_DIR}/legacy_snapshot.json"


def _save_snapshot(cli, snap: dict) -> None:
    payload = json.dumps({k: [list(p) for p in v] for k, v in snap.items()},
                         ensure_ascii=False)
    # 经 base64 落地，避免命令行里的引号/空格在两层 shell 转义中被吃掉。
    b64 = base64.b64encode(payload.encode("utf-8")).decode("ascii")
    _run(cli, f"mkdir -p ~/{REMOTE_HOME_DIR} && echo {b64} | "
              f"base64 -d > {SNAPSHOT_PATH}")


def _load_snapshot(cli) -> dict:
    code, out, _ = _run(cli, f"cat {SNAPSHOT_PATH} 2>/dev/null || true")
    try:
        raw = json.loads(out)
    except (ValueError, TypeError):
        return {}
    return {k: [(str(p[0]), str(p[1])) for p in v]
            for k, v in raw.items() if isinstance(v, list)}


def _stop_legacy(cli):
    for pat in LEGACY_PATTERNS:
        _run(cli, f"pkill -f '{pat}' 2>/dev/null; true")
    time.sleep(1.0)


def _start_dznav(cli, config: str = REMOTE_CONFIG) -> str:
    """拉起 dznav（待命）。显式导出 DZ_NAV_CONFIG，并在文件缺失时喊出来。

    缺配置文件不是致命错（节点会退回 DEFAULTS 并打 error 日志），但必须
    让操作员在部署输出里看见——否则"我改了 yaml 怎么没生效"会一路带到赛场。
    """
    code, out, _ = _run(cli, f"test -f {config} && echo yes || echo no")
    if "yes" not in out:
        print(f"  [警告] 车端配置不存在: {config} —— dznav 将用内置默认值")
        env_cfg = ""
    else:
        env_cfg = f"DZ_NAV_CONFIG={config} "
    cmd = (f"cd ~/{REMOTE_HOME_DIR} && {ENV_TMPL} && "
           f"{env_cfg}DZ_AUTO_START=0 nohup python3 -m dznav.executor_node "
           f"> ~/{REMOTE_HOME_DIR}/dznav.log 2>&1 & echo $!")
    code, out, err = _run(cli, cmd)
    if code != 0:
        raise RuntimeError(f"dznav 启动失败: {err.strip()}")
    return out.strip().splitlines()[-1] if out.strip() else "?"


def _stop_dznav(cli):
    _run(cli, "pkill -f 'dznav.executor_node' 2>/dev/null; true")
    time.sleep(1.0)


def _restart_legacy(cli, snap: dict) -> None:
    """按留档命令行重启 legacy 两进程；留档为空时用标准栈命令兜底。

    单独抽出来是因为"切换窗口内 dznav 起不来"必须走同一条恢复路径——
    原先只有体检失败会回滚，_start_dznav 抛异常时直接冒出 traceback，
    把一台没有任何控制器/定位源的车留在场上。
    """
    running = {k: v for k, v in (snap or {}).items() if v}
    if not running:
        _run(cli, f"{ENV_TMPL} && "
                  "nohup ros2 run car_localization serial_reader_node "
                  "> /tmp/dznav_rb_sr.log 2>&1 & sleep 2 && "
                  "nohup ros2 run car_localization path_executor_node "
                  "> /tmp/dznav_rb_pe.log 2>&1 &")
        time.sleep(3.0)
        return
    for _pat, procs in running.items():
        for _pid, cmd0 in procs:
            _run(cli, f"nohup {cmd0} > /tmp/dznav_rb_{_pid}.log 2>&1 &")
    time.sleep(3.0)


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
            # 回滚时 legacy 必然已停，现场快照一定是空的；
            # 必须读部署时落地的留档，否则只能退回硬编码兜底命令。
            saved = _load_snapshot(cli)
            if not any(saved.values()):
                print(f"  [提示] 无留档（{SNAPSHOT_PATH}），用标准栈命令兜底")
            _restart_legacy(cli, saved)
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
        _save_snapshot(cli, snap)

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
        try:
            pid = _start_dznav(cli, config=args.config)
        except Exception as e:  # noqa: BLE001 - 切换窗口内必须恢复
            # 此刻 legacy 已停、dznav 没起来：车上没有任何控制器，
            # 也没有 /odom/localization 发布者。先把 legacy 拉回来。
            print(f"  dznav 启动失败（{type(e).__name__}: {e}），回滚到 legacy")
            _restart_legacy(cli, snap)
            return 1
        print(f"  dznav 已启动 pid={pid}（待命，不发车）")

        print("== 4/4 体检 ==")
        if not _health(cli):
            print("  体检失败，自动回滚到 legacy")
            _stop_dznav(cli)
            _restart_legacy(cli, snap)
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
    ap.add_argument("--config", default=REMOTE_CONFIG,
                    help="车端 DZ_NAV_CONFIG 路径（默认随包推送的 "
                         "config/dznav_default.yaml）")
    ap.add_argument("--status", action="store_true", help="只查当前状态")
    ap.add_argument("--rollback", action="store_true", help="回滚到 legacy 栈")
    args = ap.parse_args()
    args.password = remote_run.read_password(args, env_name="AG_CAR_PASS")
    # --config 会被拼进远端 shell 命令，白名单挡掉引号/分号/反引号。
    if not re.fullmatch(r"[~A-Za-z0-9_./-]+", args.config):
        raise SystemExit(f"--config 含非法字符: {args.config!r}")
    raise SystemExit(deploy(args))


if __name__ == "__main__":
    main()
