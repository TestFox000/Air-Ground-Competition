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
import hashlib
import re
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "tools" / "ml"))

# Windows 上 stdout 默认走 GBK（本机实测 sys.stdout.encoding == 'gbk'）。
# 本项目注释里 ⚠ 满天飞，谁把它顺手抄进 report() 文案，print 就会
# UnicodeEncodeError —— 自检崩在最有用的那一项之前，比不检查还糟。
# 统一 UTF-8 + errors=replace：难看的输出永远好过没有输出。
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

PASSED, WARN, FAILED = 0, 1, 2
ICON, COLOR = {PASSED: "[通过]", WARN: "[警告]", FAILED: "[失败]"}, {PASSED: 32, WARN: 33, FAILED: 31}
# 重定向进日志文件时别掺 ANSI 转义（实测一次本地检查会写入 16 个 ESC 字节）。
_COLOR = sys.stdout.isatty()
bad_total = 0

# 车端灯检模型的契约 md5（小写，与 md5sum 输出同格式）
_LIGHT_DET_MD5 = "49f3700590a907994e7ee20e4d30cc13"

# 机载引擎检查的真源是 orin_run_dzuav.sh 的默认值，不是这里另抄一份常量：
# 抄一份就必然漂移，而漂移的后果是自检绿灯放行一台"启动脚本根本找不到引擎"
# 的机器。tests/test_tools_smoke.py 同时锁住本解析与 convert 脚本的产物名。
_RUN_DZUAV = REPO / "tools" / "orin_run_dzuav.sh"


def run_dzuav_defaults(*vars_: str) -> dict[str, str]:
    """解析 orin_run_dzuav.sh 里 `VAR="${VAR:-默认}"` 形式的默认值。

    只认这一种写法。改成别的形式（或直接写死）时本函数返回不全，调用方
    必须判红而不是静默降级——宁可自检报错，不可自检说谎。
    脚本读不到时返回空 dict 而不是抛异常：本文件的既有原则是"自检崩在最有用的
    那一项之前，比不检查还糟"，缺文件同样走调用方的判红分支。
    """
    try:
        text = _RUN_DZUAV.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return {}
    out: dict[str, str] = {}
    for var in vars_:
        m = re.search(rf'^{var}="\$\{{{var}:-(.+?)\}}"', text, re.M)
        if m:
            out[var] = m.group(1)
    return out


def report(status: int, msg: str, fix: str = "") -> None:
    global bad_total
    bad_total += status == FAILED
    tag = f"\033[1;{COLOR[status]}m{ICON[status]}\033[0m" if _COLOR else ICON[status]
    print(f"{tag} {msg}"
          + (f"\n      修法: {fix}" if fix and status != PASSED else ""))


def fix_cmd(rel_path: str, *args: str) -> str:
    """拼"修法"里的命令串；脚本本身不在包里就当场标出来。

    自检工具最坏的情况不是报错，是把人指挥去跑一个不存在的脚本——比赛
    当天按提示敲下去只会浪费时间。这里让提示语自己校验自己。
    """
    cmd = ("python " if rel_path.endswith(".py") else "bash ") + rel_path
    if args:
        cmd += " " + " ".join(args)
    return cmd if (REPO / rel_path).is_file() else f"{cmd}   ← 本包未附此脚本！"


def load_verifier():
    """惰性导入 tools/ml/verify_contract.verify；缺失返回 None（不炸整个自检）。

    原来是模块顶层 `from verify_contract import verify`：该模块不随包发布，
    import 直接 ModuleNotFoundError，导致本脚本连 --help 都跑不起来，
    三端十几项检查全部陪葬。契约检查只是其中一项，缺了就降级成警告。
    """
    try:
        from verify_contract import verify
        return verify
    except ImportError as exc:
        report(WARN, f"无法导入 tools/ml/verify_contract（{exc}），ONNX 契约检查跳过",
               "把 tools/ml/verify_contract.py 一起打包")
        return None


def check_local(match: bool = False) -> None:
    print("\n===== 本地（发布物 / 配置 / 标定）"
          + ("  [比赛模式：关键项红灯即禁赛]" if match else "") + " =====")
    rel = REPO / "models" / "release"

    # 1. ONNX 契约
    onnx_files = sorted(rel.glob("*.onnx"))
    if not onnx_files:
        report(FAILED, "models/release 下没有 .onnx"
                       + ("" if rel.is_dir() else "（连目录都不存在：模型未随本包发布）"),
               fix_cmd("tools/release_models.py"))
    verify = load_verifier()
    for f in onnx_files:
        if verify is None:
            report(WARN, f"{f.name} 在位，但契约无法校验（verify_contract 缺失）",
                   "把 tools/ml/verify_contract.py 一起打包；或手工核对输入名/尺寸/类别数")
            continue
        ok, msg = verify(f)
        report(PASSED if ok else FAILED, msg,
               fix_cmd("tools/release_models.py", "--cell/--hbb", "..."))

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
    reg = rel / "README.md"
    if not reg.is_file():
        report(WARN, "models/release/README.md 登记表缺失，md5 复核跳过",
               "发布模型时同步生成登记表（文件名 + md5 大写 32 位）")
    else:
        text = reg.read_text(encoding="utf-8")
        for name, md5 in re.findall(r"\| `([^`]+)` \| `([0-9A-F]{32})`", text):
            f = rel / name
            if not f.is_file():
                continue  # 历史登记（已删文件）不追究
            h = hashlib.md5(f.read_bytes()).hexdigest().upper()
            report(PASSED if h == md5 else FAILED,
                   f"md5 {name}" + ("" if h == md5 else f"（文件 {h} != 登记 {md5}）"),
                   fix_cmd("tools/release_models.py") + " 或改回登记表")

    # 4. 每局配置
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
        m = (re.search(r"K_ = \(cv::Mat_<double>\(3, 3\) << ([\d.]+), 0, ([\d.]+)",
                       cpp.read_text(encoding="utf-8")) if cpp.is_file() else None)
        same = m and abs(float(m.group(1)) - d["fx"]) < 1e-9 and abs(float(m.group(2)) - d["cx"]) < 1e-9
        report(PASSED if same else WARN,
               f"标定 rms={d['rms_px']:.3f}px @{d['width']}x{d['height']}（{d['date']}）"
               + ("" if same else "，MapLocalizer.cpp 内参与标定不一致！"),
               fix_cmd("tools/calibrate_camera.py", "inject"))
    else:
        report(FAILED if match else WARN,
               "尚未做相机标定（PnP 精度根基）"
               + ("（比赛模式致命：无内参则 PnP 定位不可用）" if match else ""),
               fix_cmd("tools/calibrate_camera.py", "board")
               + " 起（board→capture→calibrate→inject）")


def ssh_run(host, user, password, cmd, port=22, timeout=30):
    import paramiko
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    # 注意：look_for_keys=True 是全仓库孤例（remote_run.connect 等其它 SSH
    # 入口都不传，即默认 False 纯密码认证）。2026-10-17 密码守卫收编时仅标注
    # 不改行为：本机恰好配了 SSH 密钥时它会兜底尝试密钥，暂无对齐计划。
    cli.connect(host, port=port, username=user, password=password, timeout=timeout,
                banner_timeout=timeout, look_for_keys=True)
    try:
        _, out, err = cli.exec_command(cmd, timeout=timeout)
        rc = out.channel.recv_exit_status()
        return rc, (out.read() + err.read()).decode("utf-8", "replace")
    finally:
        cli.close()


def get_password(env_name: str, args, flag: str) -> str:
    """密码守卫已收编到 remote_run.read_password（2026-10-17 横切扫描）；
    本壳保留原参数形态（env_name/flag 直传，env 变量名语义不变）。

    延迟导入 remote_run：它顶层即 import paramiko，而本地检查段（CI 也只跑
    本地段）不需要 paramiko——只有走到车端/机载 SSH 段才会调本函数。"""
    _TOOLS = str(REPO / "tools")
    if _TOOLS not in sys.path:
        sys.path.insert(0, _TOOLS)
    import remote_run
    return remote_run.read_password(args, env_name=env_name, flag=flag)


def check_car(args) -> None:
    print(f"\n===== 车端（{args.user}@{args.host}）=====")
    pw = get_password("AG_CAR_PASS", args, "password")
    ws = args.ws
    # dzros.env = 车上 ROS 域真源（默认导 ROS_DOMAIN_ID=118）：远程命令经 SSH
    # 非交互执行不读 .bashrc，不显式 source 会落域 0 看不到车上节点——必需而非
    # 保险；文件不存在静默跳过，保持可移植
    pre = (f"source /opt/ros/$(ls /opt/ros | head -1)/setup.bash; "
           f"source {ws}/install/setup.bash 2>/dev/null; "
           f"[ -f ~/dzrobot/dzros.env ] && source ~/dzrobot/dzros.env || true")

    def run(cmd, timeout=60):
        return ssh_run(args.host, args.user, pw, f"{pre}; {cmd}", timeout=timeout)

    rc, out = run("ros2 pkg list 2>/dev/null | grep -E '^(ag_integrate|ag_core)$'")
    pkgs = {ln.strip() for ln in out.splitlines()}
    report(PASSED if "ag_integrate" in pkgs else FAILED,
           f"ag_integrate{'已安装' if 'ag_integrate' in pkgs else '未安装！'}",
           fix_cmd("tools/car_deploy.py", "--host", args.host))
    report(PASSED if "ag_core" in pkgs else WARN,
           "ag_core（黑匣子）已安装" if "ag_core" in pkgs else "ag_core 未安装（黑匣子关闭，可容忍）",
           fix_cmd("tools/car_deploy.py", "--host", args.host))

    rc, out = run("ros2 pkg list 2>/dev/null | grep -iE 'path|executor' | head -5")
    report(PASSED if "path_executor" in out else WARN,
           f"官方 path_executor: {out.strip() or '未见（确认官方框架包名）'}")

    rc, out = run("ss -ulnp 2>/dev/null | grep :50000")
    report(PASSED if out else WARN,
           "UDP:50000 监听中（integrate_node 在跑）" if out
           else "UDP:50000 无监听（节点未起）", "ros2 launch ag_integrate integrate.launch.py")

    rc, out = run(f"find ~/dzrobot -name 'light_det*.rknn' -exec md5sum {{}} + 2>/dev/null | head -5")
    # 判定与文案必须用同一个（完整）md5：原先状态看全串、文案看 6 位前缀，
    # 两者不一致时会打出"[警告] light_det.rknn 契约版在位"这种自相矛盾的提示。
    rknn_ok = _LIGHT_DET_MD5 in out.lower()
    report(PASSED if rknn_ok else WARN,
           "light_det.rknn 契约版在位" if rknn_ok
           else f"车端 rknn 非契约版: {out.strip() or '未找到'}")

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

    # 引擎必须按「启动脚本真正会加载的名字」逐个查，不许数个数。
    # 历史事故（2026-10-20）：目录里躺着 9 月的 7 类 hbb_detector_fp16.engine +
    # cell_classifier_fp16.engine，原先的 `ls *.engine | n>=2` 判绿放行，而
    # orin_run_dzuav.sh 要找的 d13/v4d 一个都不在 —— 自检给一台必然启动失败
    # 的机器发了通行证。名字对了才有意义，多出来的旧引擎只是陷阱不是资产。
    dflt = run_dzuav_defaults("ENGINES", "HBB_ENGINE", "CELL_ENGINE")
    engines_dir = dflt.get("ENGINES")
    want = {lbl: dflt.get(var, "").rsplit("/", 1)[-1]
            for var, lbl in (("HBB_ENGINE", "检测"), ("CELL_ENGINE", "格号"))}
    if not engines_dir or not all(want.values()):
        report(FAILED,
               f"无法从 {_RUN_DZUAV.name} 解析引擎名默认值（得到 {dflt or '脚本读不到'}），本项拒绝判定",
               f"确认 tools/{_RUN_DZUAV.name} 随包发布，且 ENGINES/HBB_ENGINE/CELL_ENGINE "
               f"仍是 VAR=\"${{VAR:-...}}\" 写法")
    else:
        rc, out = run(f"ls -1 {engines_dir}/*.engine 2>/dev/null")
        present = {ln.strip().rsplit("/", 1)[-1] for ln in out.splitlines() if ln.strip()}
        missing = [n for n in want.values() if n not in present]
        report(PASSED if not missing else FAILED,
               f"TRT 引擎 {'、'.join(want.values())} 均在位" if not missing
               else f"缺 {len(missing)}/{len(want)} 个启动脚本要加载的引擎: {'、'.join(missing)}"
                    f"（目录现有 {len(present)} 个 .engine，个数不作数）",
               fix_cmd("tools/orin_convert_engines.sh") + " 重转")
        strays = sorted(present - set(want.values()))
        if strays:
            report(WARN,
                   f"{engines_dir} 另有 {len(strays)} 个不会被加载的引擎: {'、'.join(strays)}",
                   "启动脚本只认上面两个名字；确认这些旧引擎不是你要上场的那一版")

    rc, out = run("ls /dev/video* 2>/dev/null")
    report(PASSED if "/dev/video" in out else FAILED,
           f"相机设备: {' '.join(out.split()) or '无（检查 USB/CSI 连接）'}")

    # 部署口径已统一为 orin_deploy.py 的仓库形状布局 ~/dzuav_deploy/air_ground/
    # （旧路径 ~/ag_test/dzuav/build/dzuav 随人肉 scp 时代作废，不再查）
    rc, out = run("ls ~/dzuav_deploy/air_ground/onboard/dzuav/build/dzuav 2>/dev/null")
    report(PASSED if "dzuav" in out else FAILED, "dzuav 二进制在位" if out else "dzuav 未编译",
           fix_cmd("tools/orin_build_verify.sh"))

    rc, out = run("tmux has-session -t agup 2>/dev/null && echo alive")
    report(PASSED if "alive" in out else WARN,
           "全栈会话 agup 运行中" if "alive" in out else "全栈未启动",
           fix_cmd("tools/orin_bringup.sh", "up"))

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
