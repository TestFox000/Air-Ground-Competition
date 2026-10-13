"""赛前开机自检：本地 / 车端 / 机载 三端一键体检。

比赛当天最怕的不是坏，是不知道哪坏了。本脚本把"上场前必须活着"的环节
逐项点亮，绿灯放行、红灯给修法：

    python tools/preflight.py          # 本地：模型契约/md5/配置/标定
    python tools/preflight.py --match  # 本地·比赛模式：dry_run/标定/指针 判为致命
    python tools/preflight.py --car   # 车端(SSH)：包/UDP/黑匣子/检测模型
    python tools/preflight.py --orin  # 机载(SSH)：TRT/引擎/相机/二进制
    python tools/preflight.py --all   # 三端全查

车端: --host 192.168.1.188 --user duzhong --password 或环境变量 AG_CAR_PASS
机载: --orin-host 192.168.55.1 --user duzhong --password 或环境变量 AG_ORIN_PASS
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "tools" / "ml"))
from verify_contract import verify  # noqa: E402

PASSED, WARN, FAILED = 0, 1, 2
ICON, COLOR = {PASSED: "[通过]", WARN: "[警告]", FAILED: "[失败]"}, {PASSED: 32, WARN: 33, FAILED: 31}
bad_total = 0


def report(status: int, msg: str, fix: str = "") -> None:
    global bad_total
    bad_total += status == FAILED
    print(f"\033[1;{COLOR[status]}m{ICON[status]}\033[0m {msg}"
          + (f"\n      修法: {fix}" if fix and status != PASSED else ""))


def check_local(match: bool = False) -> None:
    print("\n===== 本地（发布物 / 配置 / 标定）"
          + ("  [比赛模式：关键项红灯即禁赛]" if match else "") + " =====")
    rel = REPO / "models" / "release"

    # 1. ONNX 契约
    onnx_files = sorted(rel.glob("*.onnx"))
    if not onnx_files:
        report(FAILED, "models/release 下没有 .onnx", "跑 tools/release_models.py 发布")
    for f in onnx_files:
        ok, msg = verify(f)
        report(PASSED if ok else FAILED, msg,
               "重新导出：python tools/release_models.py --cell/--hbb ...")

    # 2. current 指针指向的文件必须存在
    for pointer in ("current_hbb_detector", "current_cell_classifier"):
        p = rel / pointer
        if not p.is_file():
            report(FAILED if match else WARN,
                   f"无 {pointer} 指针（orin_convert_engines.sh 会回退旧名）",
                   "确认 models/release/ 下 current_* 指针文件存在且指向本次要上场的模型")
            continue
        target = rel / p.read_text(encoding="utf-8").strip()
        report(PASSED if target.is_file() else FAILED,
               f"指针 {pointer} -> {target.name}"
               + ("" if target.is_file() else "（目标不存在！）"))

    # 3. 登记表 md5 复核（防"文件换了登记没更新"）
    text = (rel / "README.md").read_text(encoding="utf-8")
    for name, md5 in re.findall(r"\| `([^`]+)` \| `([0-9A-F]{32})`", text):
        f = rel / name
        if not f.is_file():
            continue  # 历史登记（已删文件）不追究
        import hashlib
        h = hashlib.md5(f.read_bytes()).hexdigest().upper()
        report(PASSED if h == md5 else FAILED,
               f"md5 {name}" + ("" if h == md5 else f"（文件 {h} != 登记 {md5}）"),
               "重跑 tools/release_models.py 或改回登记表")

    # 4. 每局配置
    import yaml
    cfg = REPO / "src/ag_integrate/config/match_params.yaml"
    if cfg.is_file():
        data = yaml.safe_load(cfg.read_text(encoding="utf-8"))["integrate_node"]["ros__parameters"]
        dry = "true（只算不发！）" if data.get("dry_run") else "false（实发）"
        report(PASSED, f"match_params.yaml: {data.get('start_cell')}->{data.get('goal_cell')} "
                       f"head={data.get('start_heading')} dry_run={dry}")
        if data.get("dry_run"):
            report(FAILED if match else WARN,
                   "dry_run=true"
                   + ("（比赛模式致命：车不动=完赛失败，必须改 false）" if match
                      else "，正式比赛记得改 false"),
                   "把 src/ag_integrate/config/match_params.yaml 的 dry_run 改 false")
    else:
        report(FAILED, "缺 match_params.yaml")

    # 5. 相机标定与 C++ 内参一致性
    cal = REPO / "runs/camera_calib/calibration.yaml"
    cpp = REPO / "onboard/dzuav/src/MapLocalizer.cpp"
    if cal.is_file():
        d = yaml.safe_load(cal.read_text(encoding="utf-8"))
        m = re.search(r"K_ = \(cv::Mat_<double>\(3, 3\) << ([\d.]+), 0, ([\d.]+)",
                      cpp.read_text(encoding="utf-8"))
        same = m and abs(float(m.group(1)) - d["fx"]) < 1e-9 and abs(float(m.group(2)) - d["cx"]) < 1e-9
        report(PASSED if same else WARN,
               f"标定 rms={d['rms_px']:.3f}px @{d['width']}x{d['height']}（{d['date']}）"
               + ("" if same else "，MapLocalizer.cpp 内参与标定不一致！"),
               "python tools/calibrate_camera.py inject")
    else:
        report(FAILED if match else WARN,
               "尚未做相机标定（PnP 精度根基）"
               + ("（比赛模式致命：无内参则 PnP 定位不可用）" if match else ""),
               "python tools/calibrate_camera.py board 起（board→capture→calibrate→inject）")


def ssh_run(host, user, password, cmd, port=22, timeout=30):
    import paramiko
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(host, port=port, username=user, password=password, timeout=timeout,
                banner_timeout=timeout, look_for_keys=True)
    try:
        _, out, err = cli.exec_command(cmd, timeout=timeout)
        rc = out.channel.recv_exit_status()
        return rc, (out.read() + err.read()).decode("utf-8", "replace")
    finally:
        cli.close()


def get_password(env_name: str, args, flag: str) -> str:
    import os
    pw = getattr(args, flag) or os.environ.get(env_name)
    if pw:
        return pw
    if not sys.stdin.isatty():
        raise SystemExit(f"无交互终端，设环境变量 {env_name}=<密码> 或用 {flag} 传入")
    import getpass
    return getpass.getpass(f"{args.user}@{args.host} 密码: ")


def check_car(args) -> None:
    print(f"\n===== 车端（{args.user}@{args.host}）=====")
    pw = get_password("AG_CAR_PASS", args, "password")
    ws = args.ws
    pre = f"source /opt/ros/$(ls /opt/ros | head -1)/setup.bash; source {ws}/install/setup.bash 2>/dev/null"

    def run(cmd, timeout=60):
        return ssh_run(args.host, args.user, pw, f"{pre}; {cmd}", timeout=timeout)

    rc, out = run("ros2 pkg list 2>/dev/null | grep -E '^(ag_integrate|ag_core)$'")
    pkgs = {ln.strip() for ln in out.splitlines()}
    report(PASSED if "ag_integrate" in pkgs else FAILED,
           f"ag_integrate{'已安装' if 'ag_integrate' in pkgs else '未安装！'}",
           "python tools/car_deploy.py --host " + args.host)
    report(PASSED if "ag_core" in pkgs else WARN,
           "ag_core（黑匣子）已安装" if "ag_core" in pkgs else "ag_core 未安装（黑匣子关闭，可容忍）",
           "python tools/car_deploy.py --host " + args.host)

    rc, out = run("ros2 pkg list 2>/dev/null | grep -iE 'path|executor' | head -5")
    report(PASSED if "path_executor" in out else WARN,
           f"官方 path_executor: {out.strip() or '未见（确认官方框架包名）'}")

    rc, out = run("ss -ulnp 2>/dev/null | grep :50000")
    report(PASSED if out else WARN,
           "UDP:50000 监听中（integrate_node 在跑）" if out
           else "UDP:50000 无监听（节点未起）", "ros2 launch ag_integrate integrate.launch.py")

    rc, out = run(f"find ~/dzrobot -name 'light_det*.rknn' -exec md5sum {{}} + 2>/dev/null | head -5")
    report(PASSED if "49f3700590a907994e7ee20e4d30cc13" in out.lower() else WARN,
           "light_det.rknn 契约版在位" if "49f370" in out.lower() else f"车端 rknn: {out.strip() or '未找到'}")

    rc, out = run("ls -lt /tmp/ag_logs/*/*.jsonl 2>/dev/null | head -2")
    report(PASSED if out else WARN,
           f"黑匣子最近记录: {out.splitlines()[0].split()[-1] if out else '无（先跑一局 dry_run）'}")


def check_orin(args) -> None:
    print(f"\n===== 机载（{args.user}@{args.orin_host}）=====")
    pw = get_password("AG_ORIN_PASS", args, "orin_password")

    def run(cmd, timeout=60):
        return ssh_run(args.orin_host, args.user, pw, cmd, timeout=timeout)

    rc, out = run("ls /usr/src/tensorrt/bin/trtexec /usr/local/bin/trtexec 2>/dev/null | head -1")
    report(PASSED if out else FAILED, f"trtexec: {out.strip() or '未找到'}",
           "确认 JetPack 已刷 TensorRT 组件")

    rc, out = run("ls -lh ~/dzuav/engines/*.engine 2>/dev/null")
    n = len(out.splitlines())
    report(PASSED if n >= 2 else FAILED,
           f"TRT 引擎 {n}/2 个" + (f": {'; '.join(ln.split()[-1] for ln in out.splitlines())}" if n else ""),
           "tools/orin_convert_engines.sh 重转")

    rc, out = run("ls /dev/video* 2>/dev/null")
    report(PASSED if "/dev/video" in out else FAILED,
           f"相机设备: {' '.join(out.split()) or '无（检查 USB/CSI 连接）'}")

    rc, out = run("ls ~/ag_test/dzuav/build/dzuav 2>/dev/null")
    report(PASSED if "dzuav" in out else FAILED, "dzuav 二进制在位" if out else "dzuav 未编译",
           "tools/orin_build_verify.sh")

    rc, out = run("tmux has-session -t agup 2>/dev/null && echo alive")
    report(PASSED if "alive" in out else WARN,
           "全栈会话 agup 运行中" if "alive" in out else "全栈未启动",
           "tools/orin_bringup.sh up")

    rc, out = run("ls /opt/ros 2>/dev/null | head -1")
    report(PASSED if out else FAILED, f"ROS 发行版: {out.strip() or '无'}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--match", action="store_true",
                    help="比赛模式：dry_run=true / 无标定 / 无 current 指针 判为致命")
    ap.add_argument("--all", action="store_true", help="三端全查")
    ap.add_argument("--car", action="store_true")
    ap.add_argument("--orin", action="store_true")
    ap.add_argument("--host", default="192.168.1.188", help="车端 IP")
    ap.add_argument("--orin-host", default="192.168.55.1", help="机载 IP（NCM Type-C）")
    ap.add_argument("--user", default="duzhong")
    ap.add_argument("--password", default=None, help="车端密码（或 AG_CAR_PASS）")
    ap.add_argument("--orin-password", default=None, help="机载密码（或 AG_ORIN_PASS）")
    ap.add_argument("--ws", default="~/ag_ws", help="车端工作空间")
    args = ap.parse_args()

    check_local(args.match)
    if args.car or args.all:
        try:
            check_car(args)
        except Exception as exc:
            report(FAILED, f"车端 SSH 连不上: {exc}", "检查供电/网线/IP")
    if args.orin or args.all:
        args.host = args.orin_host  # get_password 提示用
        try:
            check_orin(args)
        except Exception as exc:
            report(FAILED, f"机载 SSH 连不上: {exc}", "Type-C NCM 连接，确认 192.168.55.1 可 ping")

    if bad_total:
        tail = "有致命项，必须处理！" + ("（比赛模式：红灯未清不得上场）" if args.match else "")
    else:
        tail = "全部关键项通过，可以上场。" + ("（含比赛模式防呆三项）" if args.match else "")
    print(f"\n{'=' * 40}\n{tail}")
    return 1 if bad_total else 0


if __name__ == "__main__":
    sys.exit(main())
