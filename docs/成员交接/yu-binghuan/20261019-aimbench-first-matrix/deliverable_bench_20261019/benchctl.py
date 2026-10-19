#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BenchCTL — 在 Windows 上一条命令完成「上车跑实验 → 拉回数据 → 自动出报告」。

它只做四件事：
  1. SSH 到车（paramiko，密码走环境变量 AG_CAR_PASS，不落盘）；
  2. 把 bench/aim_bench.py 传到车端 /tmp/aim_bench（可 --remote-root 改）；
  3. 用 source 过 ROS 环境的脚本跑 capture / hil 实验；
  4. 把 run 目录拉回本地 bench/runs/，自动调用 bench_analyze.py 出 metrics/report/plots。

用法（PowerShell）：
  $env:AG_CAR_PASS='******'
  python _cloud\\bench\\benchctl.py check
  python _cloud\\bench\\benchctl.py capture --seconds 60 --tag real_baseline
  python _cloud\\bench\\benchctl.py hil --scenario ramp --seconds 25 --latency-ms 267 --tag kp10_kd2
  python _cloud\\bench\\benchctl.py analyze runs\\..._hil_ramp
  python _cloud\\bench\\benchctl.py hil ... --compare runs\\..._hil_ramp

注意：
  * capture 只读，不发控制指令；hil 会真的转云台、会停/恢复 4_det（和必要时 6_judgment）。
  * 默认 host 192.168.0.197 / user duzhong，可用环境变量 AG_CAR_HOST / AG_CAR_USER 覆盖。
  * 车端必须已部署 ~/dzrobot/install（ROS2 Humble）+ 现役 dzrknn_dataprocess。
"""
from __future__ import annotations

import argparse
import getpass
import io
import os
import posixpath
import shlex
import shutil
import stat
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH_PY = os.path.join(HERE, "aim_bench.py")
ANALYZE_PY = os.path.join(HERE, "bench_analyze.py")


# ----------------------------------------------------------------------
# SSH / SFTP
# ----------------------------------------------------------------------
def _password(args):
    pw = args.password or os.environ.get("AG_CAR_PASS")
    if pw:
        return pw
    if sys.stdin.isatty():
        return getpass.getpass("AG_CAR_PASS（车端 %s 密码）：" % args.user)
    print("缺少密码：PowerShell 用 $env:AG_CAR_PASS='<密码>'（cmd 用 set AG_CAR_PASS=...）；"
          "或 --password（不推荐）", file=sys.stderr)
    raise SystemExit(2)


def connect(args):
    try:
        import paramiko
    except Exception as exc:  # noqa: BLE001
        print("需要 paramiko：pip install paramiko（%s）" % exc, file=sys.stderr)
        raise SystemExit(2)
    pw = _password(args)
    last = None
    for _ in range(4):
        cli = paramiko.SSHClient()
        cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            cli.connect(args.host, args.port, args.user, pw, timeout=10,
                        banner_timeout=15, auth_timeout=15,
                        look_for_keys=False, allow_agent=False)
            try:
                cli.get_transport().set_keepalive(30)
            except Exception:
                pass
            return cli
        except paramiko.AuthenticationException as exc:  # 不重试密码，防锁号
            last = exc
            try:
                cli.close()
            except Exception:
                pass
            break
        except Exception as exc:  # noqa: BLE001
            last = exc
            try:
                cli.close()
            except Exception:
                pass
            time.sleep(2)
    print("SSH 连接失败（%s@%s）：%s" % (args.user, args.host, last), file=sys.stderr)
    raise SystemExit(3)


def sftp_mkdirs(sftp, remote_dir):
    parts = remote_dir.strip("/").split("/")
    cur = ""
    for p in parts:
        cur += "/" + p
        try:
            sftp.stat(cur)
        except IOError:
            sftp.mkdir(cur, mode=0o700)


def upload_bytes(sftp, data: bytes, remote_path):
    sftp_mkdirs(sftp, posixpath.dirname(remote_path) or "/")
    sftp.putfo(io.BytesIO(data), remote_path)


def upload_file(sftp, local, remote):
    sftp_mkdirs(sftp, posixpath.dirname(remote) or "/")
    sftp.put(local, remote)


def fetch_tree(sftp, remote_dir, local_dir):
    os.makedirs(local_dir, exist_ok=True)
    for attr in sftp.listdir_attr(remote_dir):
        safe_name = attr.filename.replace("\\", "_").replace("/", "_")
        rp = posixpath.join(remote_dir, attr.filename)
        lp = os.path.join(local_dir, safe_name)
        try:
            is_dir = stat.S_ISDIR(attr.st_mode)
            is_link = stat.S_ISLNK(attr.st_mode)
        except Exception:
            is_dir = str(attr.longname or "").startswith("d")
            is_link = str(attr.longname or "").startswith("l")
        if is_link:
            try:
                is_dir = stat.S_ISDIR(sftp.stat(rp).st_mode)
            except Exception:
                is_dir = False
        if is_dir:
            fetch_tree(sftp, rp, lp)
        else:
            sftp.get(rp, lp)


def expand_remote_root(sftp, root):
    """SFTP 不展开 ~；用远端 home 替换。"""
    if root.startswith("~"):
        home = sftp.normalize(".")
        return home.rstrip("/") + root[1:]
    return root


# ----------------------------------------------------------------------
# 远端脚本
# ----------------------------------------------------------------------
def ros_script(args, py_args, tag):
    """生成车端 bash 脚本：source ROS 环境后跑 aim_bench.py。"""
    remote = posixpath.join(args.remote_root, tag)
    py = [args.remote_root.rstrip("/") + "/aim_bench.py"]
    py += py_args + ["--out", remote]
    domain = int(args.ros_domain_id)  # parser 已是 int；double check 防注入
    lines = [
        "#!/bin/bash",
        "set +u",
        "source /opt/ros/humble/setup.bash",
        "source ~/dzrobot/install/setup.bash",
        "export ROS_DOMAIN_ID=%d" % domain,
        "export ROS_LOCALHOST_ONLY=0",
        "export PYTHONUNBUFFERED=1",
        "export PYTHONIOENCODING=utf-8",
        "cd %s" % shlex.quote(args.remote_root),
        'echo "[benchctl] host=$(hostname) time=$(date +%F_%T)"',
        "python3 " + " ".join(shlex.quote(x) for x in py),
    ]
    return "\n".join(lines) + "\n", remote


def check_script(args):
    domain = int(args.ros_domain_id)
    return """#!/bin/bash
set +u
if [ ! -f /opt/ros/humble/setup.bash ]; then echo "ERROR: /opt/ros/humble/setup.bash missing"; exit 2; fi
source /opt/ros/humble/setup.bash || { echo "ERROR: source ROS failed"; exit 2; }
if [ ! -f "$HOME/dzrobot/install/setup.bash" ]; then echo "ERROR: ~/dzrobot/install/setup.bash missing"; exit 3; fi
source "$HOME/dzrobot/install/setup.bash" || { echo "ERROR: source dzrobot install failed"; exit 3; }
command -v ros2 >/dev/null 2>&1 || { echo "ERROR: ros2 not found"; exit 4; }
export ROS_DOMAIN_ID=%d
echo "===== 主机 ====="
hostname; date; uname -m; uptime | head -1
echo "===== 节点 ====="
timeout 8 ros2 node list 2>/dev/null
echo "===== 关键话题 ====="
for t in /offset_center /camemo_feedback /CameMo_Conl /aim/status /LaserShot_Command /aim/lead_x /aim/fire_gate /aim/fire_ready; do
  echo "--- $t"; timeout 4 ros2 topic info "$t" 2>/dev/null | head -4
done
echo "===== 帧率 ====="
timeout 6 ros2 topic hz /offset_center 2>/dev/null | tail -2
echo "===== 云台参数 ====="
for p in track_mode pid_enable pid_kp pid_ki pid_kd lead_apply_gain gyro_ff_gain gyro_ff_sign smooth_track use_external_fire_gate fire_dz_far_px fire_dz_near_px max_shots; do
  printf "%%-24s = " "$p"; timeout 3 ros2 param get /dzrknn_dataprocess "$p" 2>/dev/null | tail -1
done
echo "===== 激光订阅者（>0 表示 dzjudgment 在跑，hil 会先停它）====="
timeout 4 ros2 topic info /LaserShot_Command 2>/dev/null | grep -iE "subscription|subscriber" || true
""" % domain


def exec_remote(cli, command, timeout):
    # 合并 stderr，避免 stdout/stderr 顺序读造成死锁；远端 timeout 保证不会永久挂住
    secs = int(max(5, timeout))
    qcmd = shlex.quote(command)
    inner = "timeout -k 5s %ds bash -lc %s 2>&1" % (secs, qcmd)
    fallback = "bash -lc %s 2>&1" % qcmd
    wrapped = ("if command -v timeout >/dev/null 2>&1; then %s; else %s; fi"
               % (inner, fallback))
    stdin, stdout, stderr = cli.exec_command(wrapped, timeout=secs + 10, get_pty=False)
    out = stdout.read().decode("utf-8", "replace")
    err = stderr.read().decode("utf-8", "replace")
    rc = stdout.channel.recv_exit_status()
    return rc, out, err


def run_remote_experiment(args, py_args, tag):
    cli = connect(args)
    try:
        sftp = cli.open_sftp()
        args.remote_root = expand_remote_root(sftp, args.remote_root)
        sftp_mkdirs(sftp, args.remote_root)
        upload_file(sftp, BENCH_PY, posixpath.join(args.remote_root, "aim_bench.py"))
        script, remote_run = ros_script(args, py_args, tag)
        remote_script = posixpath.join(args.remote_root, "run_%s.sh" % tag)
        upload_bytes(sftp, script.encode("utf-8"), remote_script)
        sftp.close()
        print("[benchctl] 远端运行：%s" % remote_script)
        timeout = max(60.0, float(args.seconds) + 120.0)
        try:
            rc, out, err = exec_remote(cli, "bash %s" % shlex.quote(remote_script), timeout)
        except Exception as exc:  # noqa: BLE001  (WiFi 掉线/超时)
            print("[benchctl] 远端执行失败/超时：%s" % exc, file=sys.stderr)
            return 4, None
        print(out)
        if err.strip():
            print(err, file=sys.stderr)
        # 拉回 run 目录
        local_run = os.path.join(HERE, "runs", tag)
        if os.path.exists(local_run):
            shutil.rmtree(local_run)
        sftp = cli.open_sftp()
        try:
            fetch_tree(sftp, remote_run, local_run)
            print("[benchctl] 已拉回：%s" % local_run)
        except IOError as exc:
            print("[benchctl] 拉回失败（远端 run 不存在？）：%s" % exc, file=sys.stderr)
            local_run = None
        finally:
            sftp.close()
        return (rc if rc != 0 else (0 if local_run else 5)), local_run
    finally:
        cli.close()


# ----------------------------------------------------------------------
# 本地分析
# ----------------------------------------------------------------------
def analyze_local(run_dir, compare=None, plot=True, strict=False, ledger=True):
    run_dir = os.path.abspath(run_dir)
    if compare:
        compare = os.path.abspath(compare)
    cmd = [sys.executable, ANALYZE_PY, run_dir]
    if compare:
        cmd += ["--compare", compare]
    if not plot:
        cmd += ["--no-plot"]
    if strict:
        cmd += ["--strict"]
    if not ledger:
        cmd += ["--no-ledger"]
    print("\n[benchctl] 本地分析：%s" % " ".join(cmd))
    return subprocess.call(cmd, cwd=HERE)


# ----------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------
def _env_int(name, default):
    try:
        return int(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return int(default)


def add_ssh(p):
    p.add_argument("--host", default=os.environ.get("AG_CAR_HOST", "192.168.0.197"))
    p.add_argument("--port", type=int, default=22)
    p.add_argument("--user", default=os.environ.get("AG_CAR_USER", "duzhong"))
    p.add_argument("--password", default=None, help="不推荐；优先 AG_CAR_PASS")
    p.add_argument("--remote-root", default="/tmp/aim_bench",
                   help="远端目录；绝对路径或 ~/...（会展开）")
    p.add_argument("--ros-domain-id", type=int, default=_env_int("ROS_DOMAIN_ID", 118))


def main(argv=None):
    ap = argparse.ArgumentParser(description="AimBench 电脑侧调度器（SSH + 自动分析）")
    sub = ap.add_subparsers(dest="cmd", required=True)

    ck = sub.add_parser("check", help="只读体检：节点/话题/帧率/关键参数/激光订阅者")
    add_ssh(ck)

    for name, help_text in (("capture", "只读录制真实链路"),
                            ("hil", "无靶闭环台架（会转云台、默认停/恢复 det）")):
        p = sub.add_parser(name, help=help_text)
        add_ssh(p)
        p.add_argument("--seconds", type=float, default=30.0)
        p.add_argument("--tag", default=None)
        p.add_argument("--no-analyze", action="store_true")
        p.add_argument("--no-plot", action="store_true")
        p.add_argument("--strict", action="store_true", help="分析判据失败时 benchctl 退出码非 0")
        p.add_argument("--compare", default=None, help="baseline run 目录")
        if name == "capture":
            p.add_argument("--scan", type=int, default=None, choices=(0, 1, 2))
        else:
            p.add_argument("--scenario", default="ramp")
            p.add_argument("--latency-ms", type=float, default=0.0)
            p.add_argument("--noise-px", type=float, default=2.0)
            p.add_argument("--px-per-deg", type=float, default=None)
            p.add_argument("--px-per-tick", type=float, default=None)
            p.add_argument("--yaw-center-tick", type=float, default=None)
            p.add_argument("--yaw-sign", type=int, default=None, choices=(1, -1))
            p.add_argument("--yaw-ticks-per-deg", type=float, default=None)
            p.add_argument("--imu-topic", default=None)
            p.add_argument("--det-display", default=None)
            p.add_argument("--rate-hz", type=float, default=30.0)
            p.add_argument("--lead-time-s", type=float, default=0.0)
            p.add_argument("--gates", action="store_true")
            p.add_argument("--fake-imu", action="store_true")
            p.add_argument("--allow-laser", action="store_true")
            p.add_argument("--allow-mission", action="store_true")
            p.add_argument("--ignore-param-guard", action="store_true")
            p.add_argument("--no-stop-det", action="store_true")
            p.add_argument("--no-restart-det", action="store_true")
            p.add_argument("--no-stop-judgment", action="store_true")
            p.add_argument("--no-restart-judgment", action="store_true")

    p = sub.add_parser("analyze", help="只分析本地 run 目录/CSV")
    p.add_argument("path")
    p.add_argument("--compare", default=None)
    p.add_argument("--no-plot", action="store_true")
    p.add_argument("--no-ledger", action="store_true")
    p.add_argument("--strict", action="store_true")

    args = ap.parse_args(argv)

    if args.cmd == "analyze":
        return analyze_local(args.path, args.compare, plot=not args.no_plot,
                             strict=args.strict, ledger=not args.no_ledger)
    if getattr(args, "seconds", 1.0) <= 0:
        ap.error("--seconds 必须 > 0")
    if args.cmd == "hil" and getattr(args, "rate_hz", 30.0) <= 0:
        ap.error("--rate-hz 必须 > 0")
    if getattr(args, "compare", None) and not os.path.exists(args.compare):
        ap.error("--compare 路径不存在：%s" % args.compare)

    if args.cmd == "check":
        cli = connect(args)
        try:
            stdin, stdout, stderr = cli.exec_command("bash -s", timeout=120)
            stdin.write(check_script(args))
            stdin.channel.shutdown_write()
            out = stdout.read().decode("utf-8", "replace")
            err = stderr.read().decode("utf-8", "replace")
            rc = stdout.channel.recv_exit_status()
            print(out)
            if err.strip():
                print(err, file=sys.stderr)
            return rc
        finally:
            cli.close()

    # capture / hil
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = "".join(c if (c.isalnum() or c in "-_") else "_"
                   for c in (args.tag or args.cmd))[:40].strip("-_") or args.cmd
    # 始终带时间戳（含毫秒），避免远端/本地同名旧目录被复用
    ms = int((time.time() % 1.0) * 1000)
    tag = "%s_%s%03d" % (base, stamp[-6:], ms)
    py_args = [args.cmd, "--seconds", str(args.seconds), "--tag", tag]
    if args.cmd == "capture":
        if args.scan is not None:
            py_args += ["--scan", str(args.scan)]
    else:
        py_args += ["--scenario", args.scenario,
                    "--latency-ms", str(args.latency_ms),
                    "--noise-px", str(args.noise_px),
                    "--rate-hz", str(args.rate_hz),
                    "--lead-time-s", str(args.lead_time_s)]
        if args.px_per_deg is not None:
            py_args += ["--px-per-deg", str(args.px_per_deg)]
        if args.px_per_tick is not None:
            py_args += ["--px-per-tick", str(args.px_per_tick)]
        if args.yaw_center_tick is not None:
            py_args += ["--yaw-center-tick", str(args.yaw_center_tick)]
        if args.yaw_sign is not None:
            py_args += ["--yaw-sign", str(args.yaw_sign)]
        if args.yaw_ticks_per_deg is not None:
            py_args += ["--yaw-ticks-per-deg", str(args.yaw_ticks_per_deg)]
        if args.imu_topic:
            py_args += ["--imu-topic", args.imu_topic]
        if args.det_display is not None:
            py_args += ["--det-display", args.det_display]
        if args.gates:
            py_args.append("--gates")
        if args.fake_imu:
            py_args.append("--fake-imu")
        if args.allow_laser:
            py_args.append("--allow-laser")
        if args.allow_mission:
            py_args.append("--allow-mission")
        if args.ignore_param_guard:
            py_args.append("--ignore-param-guard")
        if args.no_stop_det:
            py_args.append("--no-stop-det")
        if args.no_restart_det:
            py_args.append("--no-restart-det")
        if args.no_stop_judgment:
            py_args.append("--no-stop-judgment")
        if args.no_restart_judgment:
            py_args.append("--no-restart-judgment")
    rc, local_run = run_remote_experiment(args, py_args, tag)
    if rc != 0:
        print("[benchctl] 远端退出码=%s（仍尝试分析已拉回的数据）" % rc, file=sys.stderr)
    arc = 0
    if local_run and not args.no_analyze:
        arc = analyze_local(local_run, args.compare, plot=not args.no_plot,
                            strict=args.strict)
    return rc if rc != 0 else arc


if __name__ == "__main__":
    raise SystemExit(main())
