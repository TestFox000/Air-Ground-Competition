#!/usr/bin/env python3
"""无人机操作向导：把《无人机操作手册》SOP 变成步骤引导面板。

把手册的 S0~S10（网络→关键件→防呆→拉栈→窗口→飞控→侦察→定位质量→UDP→
起飞准备→收工）做成"每步：作用/命令/通过标准/排查提示 + 一键执行验证"。

安全边界（与方案一致）：
  * 面板内只执行"安全动作"：检查、orin_bringup up/down（确认词）、状态查询；
  * ARM/降落/KILL **绝不在这里实现**——S9 只显示该按什么键，交给 teleop
    键盘与 flight_console 的实战闸门（不重复造飞行安全件）；
  * 机上两个自治 watchdog 的介入逻辑不复制，只展示其健康行。

用法（笔记本上，密码走 AG_ORIN_PASS）:
    python tools/uav_wizard.py --host <Orin IP> --user <用户>   # 全屏面板
    python tools/uav_wizard.py --host ... --check               # 非交互全量自检（退出码可进冒烟/CI）
    python tools/uav_wizard.py --rehearse                       # 无硬件彩排（Fake 传输 + 假 UDP 流）
    双击 无人机向导.bat 有菜单入口

面板键位（j/k/数字选步，不用方向键——Windows conhost 方向键前缀虽已解码，
但 j/k 在所有终端都稳）:
    j/k=选步  Enter=执行/验证本步  r=重验  s=跳过  a=全量自检
    d=停全栈(需确认)  q=退出

依赖: paramiko 仅真机模式需要（惰性导入，彩排/单测零第三方依赖）；
UDP 校验复用 src/ag_integrate 的 intel_protocol（与机载 C++ 字节级互逆）。
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import queue
import re
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO / "src" / "ag_integrate"))

from ag_integrate.intel_protocol import (  # noqa: E402
    MSG_ID,
    MSG_ID_CAR_POSE,
    detect_msg_id,
    parse,
    parse_car_pose,
)

DEPLOY = "~/dzuav_deploy/air_ground"          # 2026-10-16 起统一布局
REMOTE_ENV = ("source /opt/ros/humble/setup.bash 2>/dev/null; "
              "source ~/dzpx4/install/setup.bash 2>/dev/null; ")
TMUX_SESSION = "agup"

# ---------------------------------------------------------------- 状态与配色
S_PENDING, S_RUNNING, S_PASS, S_WARN, S_FAIL, S_SKIP = (
    "pending", "running", "pass", "warn", "fail", "skip")
STATUS_ICON = {S_PENDING: "○", S_RUNNING: "▶", S_PASS: "✓", S_WARN: "!",
               S_FAIL: "✗", S_SKIP: "–"}
STATUS_CN = {S_PENDING: "待执行", S_RUNNING: "运行中", S_PASS: "通过",
             S_WARN: "警告", S_FAIL: "失败", S_SKIP: "跳过"}

ANSI_CLEAR = "\x1b[2J\x1b[H"
ANSI_RED = "\x1b[1;31m"
ANSI_YELLOW = "\x1b[1;33m"
ANSI_GREEN = "\x1b[1;32m"
ANSI_CYAN = "\x1b[1;36m"
ANSI_DIM = "\x1b[2m"
ANSI_RESET = "\x1b[0m"
STATUS_COLOR = {S_PENDING: ANSI_DIM, S_RUNNING: ANSI_CYAN, S_PASS: ANSI_GREEN,
                S_WARN: ANSI_YELLOW, S_FAIL: ANSI_RED, S_SKIP: ANSI_DIM}


# ---------------------------------------------------------------- 纯解析器
def parse_keyfile(text: str) -> list[tuple[str, bool]]:
    """S1 输出 'WIZARD_OK <path>' / 'WIZARD_MISS <path>' 行 -> [(路径, 存在)]。"""
    out = []
    for line in text.splitlines():
        if line.startswith("WIZARD_OK "):
            out.append((line[10:].strip(), True))
        elif line.startswith("WIZARD_MISS "):
            out.append((line[12:].strip(), False))
    return out


def parse_bringup_status(text: str) -> tuple[bool, dict[str, bool]]:
    """orin_bringup.sh status 输出 -> (会话存在, {窗口名: 活})。

    原始行样式（orin_bringup.sh:93-101）：'  [活] dzuav' / '  [死] odom'；
    会话不存在时输出 '会话 agup 不存在'。
    """
    session = "不存在" not in text
    windows: dict[str, bool] = {}
    for m in re.finditer(r"\[(活|死)\]\s+(\S+)", text):
        windows[m.group(2)] = m.group(1) == "活"
    return session, windows


def parse_localized(text: str) -> tuple[int, int] | None:
    """dzuav stdout -> 最后一次 (localized, landmarks)；无输出返回 None。"""
    hits = re.findall(r"localized=(\d+)\s+landmarks=(\d+)", text)
    if not hits:
        return None
    loc, marks = hits[-1]
    return int(loc), int(marks)


def parse_topic_hz(text: str) -> float | None:
    """ros2 topic hz 输出 -> 最后一个 average rate；没有样本返回 None。"""
    rates = re.findall(r"average rate:\s*([0-9]+(?:\.[0-9]+)?)", text)
    return float(rates[-1]) if rates else None


_watchdog_cache: dict = {}


def watchdog_mod():
    """惰性加载 localization_watchdog 的纯解析函数（std 库，无 ROS 依赖）。"""
    if "mod" not in _watchdog_cache:
        path = REPO / "onboard" / "dzpx4_patch" / "localization_watchdog.py"
        spec = importlib.util.spec_from_file_location("wiz_localization_watchdog", path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules["wiz_localization_watchdog"] = mod
        spec.loader.exec_module(mod)
        _watchdog_cache["mod"] = mod
    return _watchdog_cache["mod"]


def last_health_line(text: str) -> dict | None:
    """多行日志文本 -> 最后一条 [px4_odom_convert_health] 的 kv dict。"""
    parse_health_line = watchdog_mod().parse_health_line
    last = None
    for line in text.splitlines():
        h = parse_health_line(line)
        if h is not None:
            last = h
    return last


def judge_health(h: dict | None, now: float, max_age: float = 15.0):
    """健康行 -> (status, notes)。年龄容忍 15s（3s 周期 + NTP 摆动余量）。"""
    if not h or "t" not in h:
        return S_FAIL, ["无健康行：px4_odom_convert 未跑或日志不在 ~/drone_logs"]
    notes = [f"guard={h.get('guard')} slam_hz={h.get('slam_hz')} "
             f"l1_lock={h.get('l1_lock')} z_src={h.get('z_src')} nan_drop={h.get('nan_drop')}"]
    try:
        age = now - float(h["t"])
    except (TypeError, ValueError):
        return S_FAIL, notes + ["健康行 t= 非数值，无法判龄"]
    notes.append(f"距最后健康行 {age:.0f}s（周期 3s）")
    guard = str(h.get("guard", ""))
    if age > max_age:
        return S_FAIL, notes + ["健康行停更：节点死 / SLAM 断流（本地化看门狗同款判据）"]
    if "STORM" in guard or h.get("l1_lock") == 1:
        return S_FAIL, notes + ["L1 熔断/EKF reset 风暴中：悬停勿动，看 blackbox 4 行归因；"
                                "双实例 ERROR 则 down 后重新 up（补丁会拒绝第二实例）"]
    slam_hz = float(h.get("slam_hz") or 0)
    if guard == "HOLD" or slam_hz < 5 or h.get("nan_drop", 0) or h.get("z_src") == 0:
        return S_WARN, notes + ["降级态（HOLD/SLAM 低频/NaN 丢帧/z 未换源）：可继续观察，起飞前再看一眼"]
    return S_PASS, notes


# ---------------------------------------------------------------- 传输层
class FakeTransport:
    """脚本化回放：script=[(正则, code, out, err)]，out 可为 callable(cmd)->str。

    供 --rehearse 彩排与单测：零网络、零 paramiko，命令全部留痕 self.calls。
    """

    def __init__(self, script=None):
        self.script = list(script or [])
        self.calls: list[str] = []

    def run(self, cmd: str, timeout: float = 60.0) -> tuple[int, str, str]:
        self.calls.append(cmd)
        for pat, code, out, err in self.script:
            if re.search(pat, cmd):
                if callable(out):
                    return code, out(cmd), (err(cmd) if callable(err) else err)
                return code, out, err
        return 0, "", "no scripted response for: " + cmd[:80]

    def close(self):
        pass


class SshTransport:
    """持久 SSH 连接（flight_console _Shell 同款：惰性连接 + 断线重置）。

    remote_run/paramiko 惰性导入——彩排与单测不需要装它们。
    所有命令统一 LC_ALL=C.UTF-8 前缀，防 [活]/[死] 中文输出被错误解码。
    """

    def __init__(self, host: str, user: str, password: str, port: int = 22):
        self.host, self.user, self.password, self.port = host, user, password, port
        self.cli = None
        self.lock = threading.Lock()

    def _ssh(self):
        with self.lock:
            if self.cli is None:
                sys.path.insert(0, str(TOOLS))
                try:
                    import remote_run  # noqa: PLC0415  顶层 import 会把 paramiko 变成硬依赖
                except SystemExit as e:   # remote_run 缺 paramiko 时 import 即 SystemExit(2)，
                    # SystemExit 不是 Exception 子类，穿透 worker 的 except Exception
                    # 会让线程无声死掉、步骤永远卡"运行中"——转成普通异常走 FAIL 路径
                    raise RuntimeError(
                        f"真机模式缺 paramiko（{e}）：python -m pip install paramiko，"
                        "或用 --rehearse 彩排（零依赖）") from e
                self.cli = remote_run.connect(self.host, self.user, self.password,
                                              port=self.port, timeout=15.0)
            return self.cli

    def _full(self, cmd: str) -> str:
        """远端命令统一加 LC_ALL（防 [活]/[死] 中文被错误 locale 解码）。

        必须用 `export ...; cmd` 而不是 `LC_ALL=... cmd` 前缀：S1 的探测命令是
        `for f in ...; do ...; done` 复合命令，sh 的环境变量前缀不能修饰保留字
        （`LC_ALL=x for ...` 是语法错误，真机上 S1 必挂；2026-10-22 复审实锤，
        彩排 FakeTransport 不经 shell 所以此前没暴露）。
        """
        return f"export LC_ALL=C.UTF-8; {cmd}"

    def run(self, cmd: str, timeout: float = 60.0) -> tuple[int, str, str]:
        full = self._full(cmd)
        for attempt in (1, 2):   # 连接级重试一次（remote_run 已带 3 次连接内重试）
            cli = self._ssh()
            try:
                stdin, stdout, stderr = cli.exec_command(full, timeout=timeout)
                stdin.close()
                out = stdout.read().decode("utf-8", errors="replace")
                err = stderr.read().decode("utf-8", errors="replace")
                code = stdout.channel.recv_exit_status()
                return code, out, err
            except Exception:  # noqa: BLE001 - 断线则重连一次再试
                with self.lock:
                    self.cli = None
                if attempt == 2:
                    raise
        raise RuntimeError("unreachable")

    def close(self):
        with self.lock:
            if self.cli is not None:
                try:
                    self.cli.close()
                except Exception:  # noqa: BLE001
                    pass
                self.cli = None


# ---------------------------------------------------------------- UDP 探针
def udp_probe(port: int, seconds: float, stop: threading.Event | None = None) -> dict:
    """本地听 :port 若干秒，用 intel_protocol 校验 dzuav 广播（S8）。

    返回 {good, bad, car_frames, fps, duration}。帧的好坏判定与车端
    integrate_node 同款（magic/version/msg_id/CRC 任一不符即 ValueError）。
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", port))
    s.settimeout(0.3)
    t0 = time.time()
    good = bad = car = 0
    try:
        while time.time() - t0 < seconds and not (stop and stop.is_set()):
            try:
                data, _addr = s.recvfrom(4096)
            except socket.timeout:
                continue
            try:
                msg_id = detect_msg_id(data)
                if msg_id == MSG_ID_CAR_POSE:
                    parse_car_pose(data)
                    car += 1
                elif msg_id == MSG_ID:
                    parse(data)
                else:
                    raise ValueError(f"unknown msg_id {msg_id}")
                good += 1
            except (ValueError, IndexError, KeyError):
                bad += 1
    finally:
        s.close()
    dur = max(time.time() - t0, 1e-6)
    return {"good": good, "bad": bad, "car_frames": car,
            "fps": good / dur, "duration": dur}


# ---------------------------------------------------------------- 步骤模型
@dataclass
class CheckResult:
    status: str
    detail: list[str] = field(default_factory=list)


@dataclass
class WizardContext:
    transport: object
    host: str = ""
    rehearse: bool = False
    udp_port: int = 50000
    udp_seconds: float = 5.0
    deploy: str = DEPLOY
    now: object = time.time
    preflight_bin: str = str(TOOLS / "preflight.py")


@dataclass
class Step:
    sid: str
    title: str
    kind: str                 # check（只读验证）| action（改远端状态，需确认）| display（纯指引）
    purpose: str
    command: str              # 展示用命令文本
    pass_criteria: str
    fail_hints: list[str] = field(default_factory=list)
    confirm_word: str = ""    # 非空 = 执行前需逐字输入
    run: object = None        # (ctx) -> CheckResult


def _res(status, *detail):
    return CheckResult(status, list(detail))


def build_steps() -> list[Step]:
    """S0~S10，与 docs/无人机操作手册.md 的每日标准流程一一对应。"""
    steps: list[Step] = []

    # S0 网络连通
    def s0(ctx: WizardContext):
        code, out, err = ctx.transport.run("echo WIZARD_PING_OK", timeout=10)
        if code == 0 and "WIZARD_PING_OK" in out:
            return _res(S_PASS, "SSH 往返正常", "下一步核机上关键件")
        return _res(S_FAIL, f"SSH 不通 rc={code}: {(err or out).strip()[:120]}")
    steps.append(Step(
        "S0", "网络连通", "check",
        "确认笔记本→Orin 的 SSH 链路通（后续所有步骤都走它）",
        "echo WIZARD_PING_OK",
        "SSH 往返拿到回显",
        ["ping <IP> 丢包>5% 或不通：查同一台路由器/同网段（UDP 广播也不出路由器）",
         "SSH 被拒：AG_ORIN_PASS 密码/用户名"],
        run=s0))

    # S1 机上关键件
    keyfiles = [
        ("~/dzuav_deploy/air_ground/onboard/dzuav/build/dzuav", "未编译：机上跑 tools/orin_bringup.sh 前先 orin_build_verify.sh"),
        ("~/dzuav/engines/hbb_detector_fp16.engine", "引擎缺失：机上跑 tools/orin_convert_engines.sh -o ~/dzuav/engines"),
        ("~/dzuav/engines/cell_classifier_fp16.engine", "引擎缺失：机上跑 tools/orin_convert_engines.sh -o ~/dzuav/engines"),
        ("/dev/video0", "相机设备节点不存在：查 USB 连接 / DZUAV_CAM 换设备号"),
    ]
    keyfile_cmd = ("for f in " + " ".join(p for p, _ in keyfiles) +
                   "; do [ -e \"$f\" ] && echo \"WIZARD_OK $f\" || echo \"WIZARD_MISS $f\"; done")

    def s1(ctx: WizardContext):
        code, out, _ = ctx.transport.run(keyfile_cmd, timeout=15)
        found = parse_keyfile(out)
        if not found:
            return _res(S_FAIL, f"关键件探测无输出 rc={code}: {out.strip()[:120]}")
        # 远端 sh 会把 for 列表里的 ~ 展开成 /home/<user>/...，回显是展开后
        # 路径；按尾部匹配（保留 ~/ 前缀剥掉后比对），两种回显都兼容。
        def present(path: str) -> bool:
            tail = path[2:] if path.startswith("~/") else path
            return any(ok and (r == path or r.endswith(tail)) for r, ok in found)
        missing = [(p, h) for p, h in keyfiles if not present(p)]
        if not missing:
            return _res(S_PASS, *(f"✓ {p}" for p, _ in keyfiles))
        return _res(S_FAIL, *(f"✗ {p} —— {h}" for p, h in missing))
    steps.append(Step(
        "S1", "机上关键件", "check",
        "dzuav 二进制 / 两个 TRT 引擎 / 相机设备节点，四件全在才能起栈",
        keyfile_cmd,
        "四个路径全部存在",
        run=s1))

    # S2 本地防呆
    def s2(ctx: WizardContext):
        if ctx.rehearse:
            return _res(S_SKIP, "彩排模式：无本机真源，preflight --match 跳过")
        try:
            proc = subprocess.run([sys.executable, ctx.preflight_bin, "--match"],
                                  capture_output=True, text=True, timeout=120,
                                  cwd=str(REPO))
        except (OSError, subprocess.TimeoutExpired) as e:
            return _res(S_FAIL, f"preflight 本地段没跑起来: {e}")
        tail = (proc.stdout or "").strip().splitlines()[-6:] or [proc.stderr.strip()[:160]]
        if proc.returncode == 0:
            return _res(S_PASS, "preflight --match 退出码 0", *tail)
        return _res(S_FAIL, "preflight --match 非零退出（红灯=不得上场）", *tail)
    steps.append(Step(
        "S2", "本地防呆", "check",
        "笔记本侧 preflight 本地段（模型契约/current 指针/dry_run），比赛模式红灯即停",
        "python tools/preflight.py --match",
        "退出码 0",
        ["红灯项按 preflight 输出的修法提示逐条处理，处理完重验"],
        run=s2))

    # S3 拉起全栈（action，需确认）
    up_cmd = f"bash {DEPLOY}/tools/orin_bringup.sh up"

    def s3(ctx: WizardContext):
        code, out, err = ctx.transport.run(
            f"cd {ctx.deploy} && bash tools/orin_bringup.sh up", timeout=180)
        text = out + err
        tail = [ln for ln in text.strip().splitlines() if ln.strip()][-8:]
        if code == 0 and "失败" not in text:
            warns = [ln for ln in tail if "警告" in ln]
            status = S_WARN if warns else S_PASS
            return _res(status, "bringup up 已执行（窗口就绪有间隔，S4 再验存活）", *tail)
        return _res(S_FAIL, f"bringup up rc={code}", *tail)
    steps.append(Step(
        "S3", "拉起全栈", "action",
        "tmux 会话 agup 按依赖顺序拉起窗口（dzuav 最先，车端只依赖它的 UDP；"
        "2026-10-22 起含 plan 机载规划 sidecar）。"
        "注意：会 kill 重建 tmux，黑匣子窗口被清掉——本步之后再去开 flight_console",
        up_cmd, "退出码 0 且无 [失败]",
        ["dzuav 未编译警告：先 orin_build_verify.sh",
         "相机不存在警告：DZUAV_CAM=/dev/video1 指定后再 up"],
        confirm_word="y", run=s3))

    # S4 窗口存活
    status_cmd = f"bash {DEPLOY}/tools/orin_bringup.sh status"
    win_hints = {
        "dzuav": "dzuav 死：相机断流 3 次重开失败（换 USB/设备号），或 DZUAV_DIR 旧目录坑",
        "plan": "plan 死：机载规划 sidecar（orin_plan_sidecar.py）——查 orin_recon_bg 窗口日志与 0x4003 链路",
        "px4": "px4 死：MicroXRCEAgent 起不来——查 /dev/ttyACM0 串口与供电",
        "fastlio": "fastlio 死：查 livox 窗口与点云地图加载",
        "odom": "odom 死：补丁版会拒绝第二实例，先 down 全栈再 up",
        "control": "control 死：查 ROS 环境与 px4 链路",
        "path": "path 死：查 params.yaml（record_mode/follow_mode）与重编译",
    }

    def s4(ctx: WizardContext):
        code, out, err = ctx.transport.run(status_cmd, timeout=20)
        session, windows = parse_bringup_status(out)
        if not session:
            return _res(S_FAIL, "tmux 会话 agup 不存在：全栈未启动",
                        "回到 S3 按 Enter（输 y 确认）执行 bringup up")
        dead = [w for w, alive in windows.items() if not alive]
        lines = [f"{'✓' if alive else '✗'} {name}" for name, alive in windows.items()]
        if not windows:
            return _res(S_FAIL, "status 无窗口行", out.strip()[:160] or err.strip()[:160])
        if dead:
            return _res(S_FAIL, f"死亡窗口: {', '.join(dead)}", *lines,
                        *(win_hints.get(w, f"{w} 窗口死：tmux attach -t agup 看窗口内报错") for w in dead))
        return _res(S_PASS, f"{len(windows)} 个窗口全部存活", *lines)
    steps.append(Step(
        "S4", "窗口存活", "check",
        "tmux 窗口逐一验活（dzuav/plan/livox/fastlio/px4/odom/imu/control/path[/teleop]）",
        status_cmd, "全部 [活]",
        run=s4))

    # S5 飞控链路
    hz_cmd = (REMOTE_ENV + "timeout 8 ros2 topic hz /fmu/out/vehicle_local_position 2>&1 | tail -n 6")

    def s5(ctx: WizardContext):
        code, out, err = ctx.transport.run(hz_cmd, timeout=20)
        rate = parse_topic_hz(out + err)
        if rate and rate >= 1.0:
            return _res(S_PASS, f"vehicle_local_position 平均 {rate:.1f} Hz", "飞控串口桥与 EKF 输出正常")
        if rate:
            return _res(S_WARN, f"平均仅 {rate:.1f} Hz（偏低）", "可能 uXRCE 链路不稳，观察 orin_agent_watchdog 健康行")
        return _res(S_FAIL, "8s 内没等到任何频率样本",
                    "链路死：px4 窗口 MicroXRCEAgent / /dev/ttyACM0；",
                    "自检可跑 uav_link.py ctl stack-status --host <IP>")
    steps.append(Step(
        "S5", "飞控链路", "check",
        "ros2 topic hz 采样飞控 EKF 输出（MicroXRCEAgent 串口桥活性的直接证据）",
        "ros2 topic hz /fmu/out/vehicle_local_position",
        "average rate ≥ 1 Hz",
        run=s5))

    # S6 dzuav 侦察
    pane_cmd = f"tmux capture-pane -t {TMUX_SESSION}:dzuav -p -S -40 2>/dev/null | tail -n 12"

    def s6(ctx: WizardContext):
        code, out, _ = ctx.transport.run(pane_cmd, timeout=15)
        hit = parse_localized(out)
        if hit is None:
            return _res(S_FAIL, "dzuav 窗口无输出",
                        "窗口可能刚死（S4 验活）或还在起（等 3s 看下一帧打印）",
                        "手动看: tmux attach -t agup 后切 dzuav 窗")
        loc, marks = hit
        if loc == 1:
            return _res(S_PASS, f"localized=1 landmarks={marks}", "PnP 自定位有效，目标正在投影广播")
        return _res(S_WARN, f"localized=0 landmarks={marks}",
                    "PnP 未成功：需 ≥4 个分散格号且凸包面积 ≥2% 画面",
                    "升高/移动到格号分散视野；确认地面格号未被遮挡")
    steps.append(Step(
        "S6", "dzuav 侦察", "check",
        "看 dzuav 窗口 stdout：localized=1 才有情报（PnP 需 ≥4 格号）",
        pane_cmd, "最新一帧 localized=1",
        run=s6))

    # S7 定位质量
    tail_cmd = ("tail -n 80 ~/drone_logs/node_odom_*.log 2>/dev/null | "
                "grep px4_odom_convert_health | tail -n 3")

    def s7(ctx: WizardContext):
        code, out, _ = ctx.transport.run(tail_cmd, timeout=15)
        status, notes = judge_health(last_health_line(out), ctx.now())
        return _res(status, *notes)
    steps.append(Step(
        "S7", "定位质量", "check",
        "px4_odom_convert 健康行（3s 周期）：新鲜度/L1 熔断/EKF reset 风暴/SLAM 频率",
        tail_cmd, "行新鲜(≤15s)、guard=OK、无 STORM、slam_hz≥5",
        run=s7))

    # S8 UDP 出网
    def s8(ctx: WizardContext):
        r = udp_probe(ctx.udp_port, ctx.udp_seconds)
        if r["good"] > 0 and r["fps"] >= 2:
            extra = [f"0x4002 车位姿帧 x{r['car_frames']}"] if r["car_frames"] else \
                    ["暂无 0x4002（检测到车+定位有效才发，正常）"]
            return _res(S_PASS, f"{r['good']} 好帧 / {r['fps']:.1f} fps", *extra)
        if r["good"] > 0:
            return _res(S_WARN, f"仅 {r['fps']:.1f} fps（<2）", "dzuav 处理慢或丢帧，机上 dzuav 窗口看单帧耗时")
        return _res(S_FAIL, f"{ctx.udp_seconds:.0f}s 内 0 好帧（坏帧 {r['bad']}）",
                    "本机 50000 端口被占会导致探针收不到：先关 monitor_link/别的监听再验",
                    "机上 dzuav localized=1 但这里收不到 → 查跨网段/路由器隔离（广播不出路由器）",
                    "车端等价看板：python tools/monitor_link.py")
    steps.append(Step(
        "S8", "UDP 出网", "check",
        f"本地听 :{50000} 若干秒，按车端同款解析验帧（magic/CRC）",
        "本地 udp_probe(:50000)", "好帧>0 且 fps≥2",
        run=s8))

    # S9 起飞准备（display——飞行操作不进向导，只给指引）
    steps.append(Step(
        "S9", "起飞准备", "display",
        "人工步骤。向导不做飞行控制——按键在 teleop/flight_console，闸门已实战检验",
        "（无命令）",
        "操作员逐条确认",
        ["teleop（tmux attach -t agup 切 teleop 窗）: 0=Offboard → 1=解锁起飞 → 4=放行航线",
         "flight_console 应急: L=降落(y) K=停GOTO悬停 X=DISARM !=KILL(最后手段)",
         "人工确认: 人员清场 / 电池≥80% / 遥控器在手"],
        run=lambda ctx: _res(S_SKIP,
                             "按上面三条走；起飞后可回向导按 a 复核 S4~S8")))

    # S10 收工（display + 面板 d 键提供 bringup down）
    steps.append(Step(
        "S10", "收工", "display",
        "人工步骤。降落后停栈与取黑匣子",
        "bash ~/dzuav_deploy/air_ground/tools/orin_bringup.sh down",
        "飞机已落地锁桨",
        ["teleop: 3=降落，STANDBY 后 2=锁桨",
         "停全栈: 面板 d 键（输 y 确认）= bringup down",
         "黑匣子: python tools/flight_console.py --host <IP> --fetch ./flight_logs"],
        run=lambda ctx: _res(S_SKIP, "落地锁桨后按 d 停全栈，黑匣子按上行命令取回")))

    return steps


# ---------------------------------------------------------------- 状态机
class WizardState:
    def __init__(self, steps: list[Step]):
        self.steps = steps
        self.status = {s.sid: S_PENDING for s in steps}
        self.detail: dict[str, list[str]] = {}
        self.idx = 0

    def step_by_sid(self, sid: str) -> Step | None:
        return next((s for s in self.steps if s.sid == sid), None)

    def apply(self, sid: str, result: CheckResult):
        self.status[sid] = result.status
        self.detail[sid] = result.detail

    def counts(self) -> dict[str, int]:
        c = {k: 0 for k in STATUS_ICON}
        for v in self.status.values():
            c[v] += 1
        return c

    def all_green(self) -> bool:
        return all(v in (S_PASS, S_WARN, S_SKIP) for v in self.status.values())


def run_step(step: Step, ctx: WizardContext) -> CheckResult:
    """执行一步并收敛异常 -> CheckResult（面板/自检/测试共用入口）。"""
    if step.run is None:
        return _res(S_SKIP, "纯指引步骤")
    try:
        return step.run(ctx)
    except Exception as e:  # noqa: BLE001 - 单步失败不拖垮整个向导
        return _res(S_FAIL, f"步骤异常 {type(e).__name__}: {e}",
                    *step.fail_hints)


# ---------------------------------------------------------------- TUI
class KeyReader(threading.Thread):
    """跨平台单键读取（flight_console 同款 + 可停止；Windows 方向键前缀解码为 UP/DOWN）。

    stop() 必须真正终止读取循环：菜单模式下同一进程会反复进出面板，旧线程
    不停就会和新面板/菜单 input() 抢键盘，表现为"偶发吃键"（2026-10-21 审计）。
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.q: "queue.Queue[str]" = queue.Queue()
        self._old_termios = None
        self._stopped = threading.Event()
        self._start_platform()

    def _start_platform(self):
        if os.name == "nt":
            import msvcrt
            self._msvcrt = msvcrt
            return
        import termios
        import tty
        try:
            self._old_termios = termios.tcgetattr(0)
            tty.setcbreak(0)
        except termios.error:
            pass

    def run(self):
        while not self._stopped.is_set():
            try:
                ch = self._read_one()
                if ch:
                    self.q.put(ch)
            except Exception:  # noqa: BLE001
                time.sleep(0.1)

    def _read_one(self):
        if os.name == "nt":
            m = self._msvcrt
            if not m.kbhit():
                time.sleep(0.02)
                return None
            k = m.getch()
            if k in (b"\x00", b"\xe0"):   # 方向键前缀：第二字节映射为 UP/DOWN
                seq = m.getch()
                return {b"H": "UP", b"P": "DOWN", b"K": "LEFT", b"M": "RIGHT"}.get(seq)
            return k.decode("utf-8", "replace")
        import select
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        if not r:
            return None
        ch = sys.stdin.read(1)
        if ch == "\x1b":                    # POSIX 方向键转义序列
            r2, _, _ = select.select([sys.stdin], [], [], 0.02)
            if r2 and sys.stdin.read(1) == "[":
                r3, _, _ = select.select([sys.stdin], [], [], 0.02)
                if r3:
                    return {"A": "UP", "B": "DOWN"}.get(sys.stdin.read(1))
            return "\x1b"
        return ch or None

    def stop(self):
        self._stopped.set()
        if os.name != "nt" and self._old_termios:
            import termios
            try:
                termios.tcsetattr(0, termios.TCSADRAIN, self._old_termios)
            except termios.error:
                pass


class Panel:
    """步骤引导面板：状态机 + 后台执行线程 + 确认词闸门。"""

    def __init__(self, state: WizardState, ctx: WizardContext):
        self.state = state
        self.ctx = ctx
        self.keys: KeyReader | None = None
        self.log: deque = deque(maxlen=8)
        self.lock = threading.Lock()
        self.worker: threading.Thread | None = None
        self.stop_flag = threading.Event()
        self._last_lines = 0
        # 确认闸门（flight_console confirm_key 教训：确认中的任何键绝不退出面板）
        self.confirm_need = ""
        self.confirm_buf = ""
        self.confirm_desc = ""
        self.pending_action = None

    def note(self, text: str, lvl: str = "info"):
        with self.lock:
            self.log.appendleft((time.strftime("%H:%M:%S"), text, lvl))

    # ---- 执行（后台线程，避免 SSH 往返冻结面板）
    def busy(self) -> bool:
        return self.worker is not None and self.worker.is_alive()

    def _launch(self, fn, desc: str):
        if self.busy():
            self.note("上一项还在跑，稍等", "warn")
            return

        def runner():
            try:
                fn()
            except Exception as e:  # noqa: BLE001 - 后台线程异常必须可见，不能静默蒸发
                self.note(f"后台执行异常: {e}", "error")
        self.worker = threading.Thread(target=runner, daemon=True)
        self.worker.start()
        self.note(desc)

    def run_one(self, step: Step):
        with self.lock:
            self.state.status[step.sid] = S_RUNNING
        self._launch(lambda: self.state.apply(step.sid, run_step(step, self.ctx)),
                     f"{step.sid} {step.title} 执行中")

    def run_all_checks(self):
        def runner():
            for step in self.state.steps:
                if self.stop_flag.is_set():
                    return
                if step.kind != "check":
                    continue
                with self.lock:
                    self.state.status[step.sid] = S_RUNNING
                self.state.apply(step.sid, run_step(step, self.ctx))
                self.note(f"{step.sid} {step.title} -> {STATUS_CN[self.state.status[step.sid]]}")
        if self.busy():
            self.note("上一项还在跑，稍等", "warn")
            return
        self.worker = threading.Thread(target=runner, daemon=True)
        self.worker.start()
        self.note("全量自检 S0~S8 运行中")

    def bringup_down(self):
        def runner():
            try:
                code, out, err = self.ctx.transport.run(
                    f"cd {self.ctx.deploy} && bash tools/orin_bringup.sh down", timeout=30)
                self.note(f"bringup down rc={code}", "info" if code == 0 else "error")
            except Exception as e:  # noqa: BLE001
                self.note(f"bringup down 失败: {e}", "error")
        self._launch(runner, "停止全栈")

    # ---- 确认闸门
    def ask_confirm(self, need: str, desc: str, action):
        self.confirm_need = need
        self.confirm_buf = ""
        self.confirm_desc = desc
        self.pending_action = action

    def confirm_key(self, ch: str) -> bool:
        """确认模式按键。恒 False（任何确认中的键都不得退出面板——
        flight_console 旧实现恒 True 的教训）。"""
        if ch in ("\x1b", "q"):
            self.pending_action = None
            self.note("已取消")
        elif ch in ("\r", "\n"):
            act, self.pending_action = self.pending_action, None
            if act is not None and self.confirm_buf.strip().lower() == self.confirm_need:
                act()
            else:
                self.note("确认词不符，已取消", "warn")
        elif ch in ("\x7f", "\x08"):
            self.confirm_buf = self.confirm_buf[:-1]
        elif ch.isprintable():
            self.confirm_buf += ch
        return False

    # ---- 按键分发（抽出为纯方法，单测不碰终端）
    def handle_key(self, ch: str) -> bool:
        """返回 True = 退出面板。确认模式优先（见 confirm_key 注释）。"""
        if self.pending_action is not None:
            return self.confirm_key(ch)
        n = len(self.state.steps)
        if ch in ("j", "DOWN"):
            self.state.idx = (self.state.idx + 1) % n
        elif ch in ("k", "UP"):
            self.state.idx = (self.state.idx - 1) % n
        elif ch.isdigit() and int(ch) < n:
            self.state.idx = int(ch)
        elif ch in ("\r", "\n"):
            step = self.state.steps[self.state.idx]
            if step.confirm_word:
                self.ask_confirm(step.confirm_word, f"执行 {step.sid} {step.title}",
                                 lambda: self.run_one(step))
            else:
                self.run_one(step)
        elif ch == "r":
            self.handle_key("\r")
        elif ch == "s":
            step = self.state.steps[self.state.idx]
            with self.lock:
                self.state.status[step.sid] = S_SKIP
                self.state.detail[step.sid] = ["手动跳过"]
        elif ch == "a":
            self.run_all_checks()
        elif ch == "d":
            self.ask_confirm("y", "停止全栈 orin_bringup.sh down", self.bringup_down)
        elif ch == "q":
            return True
        elif ch.isprintable():
            # 面板是热键驱动没有输入行，敲错键必须给可见反馈，
            # 否则用户以为"输入没反应"（2026-10-21 现场反馈）
            self.note(f"未知键 {ch!r} —— 单键操作见底部热键行", "warn")
        return False

    # ---- 绘制
    def draw(self) -> str:
        st = self.state
        ctx = self.ctx
        head = (f"无人机操作向导 · {('彩排' if ctx.rehearse else '真机')}"
                f" · {ctx.host or 'fake'}:{ctx.udp_port}")
        L: list[str] = [ANSI_CYAN + head + ANSI_RESET, "─" * 64]
        for i, step in enumerate(st.steps):
            mark = "▶" if i == st.idx else " "
            color = STATUS_COLOR[st.status[step.sid]]
            L.append(f"{mark} {color}{STATUS_ICON[st.status[step.sid]]} {step.sid:<4}"
                     f"{step.title}{ANSI_RESET}"
                     f"  {ANSI_DIM}{STATUS_CN[st.status[step.sid]]}{ANSI_RESET}")
        L.append("─" * 64)
        cur = st.steps[st.idx]
        detail = st.detail.get(cur.sid) or []
        L.append(f"{cur.sid} {cur.title} · {STATUS_CN[st.status[cur.sid]]}")
        for ln in (f"作用: {cur.purpose}", f"命令: {cur.command}",
                   f"通过标准: {cur.pass_criteria}"):
            L.extend(self._wrap(ln))
        if cur.fail_hints:
            L.append("指引:")
            for h in cur.fail_hints:
                L.extend(self._wrap(" · " + h, indent=5))
        if detail:
            L.append("明细:")
            for ln in detail:
                L.extend(self._wrap(" " + ln, indent=3))
        if self.pending_action is not None:
            L.append(ANSI_YELLOW +
                     f"确认 [{self.confirm_desc}] 输 {self.confirm_need!r} 回车执行"
                     f"（buf: {self.confirm_buf}，q 取消）" + ANSI_RESET)
        L.append("─" * 64)
        for ts, text, lvl in list(self.log)[:3]:
            c = {"error": ANSI_RED, "warn": ANSI_YELLOW}.get(lvl, ANSI_DIM)
            L.append(c + f"{ts} {text}" + ANSI_RESET)
        L.append(ANSI_DIM +
                 "j/k 选步 Enter=执行/验证 r=重验 s=跳过 a=全量自检 d=停全栈 q=退出"
                 + ANSI_RESET)
        # 面板高度预算：cmd 默认窗口 25 行，总高必须 ≤24，否则每帧滚动=满屏刷
        # （2026-10-21 用户实测实锤）。超预算时截明细/指引中段，保头尾。
        return "\n".join(self._clamp(L))

    @staticmethod
    def _clamp(lines: list[str], budget: int = 24) -> list[str]:
        if len(lines) <= budget:
            return lines
        keep_head = 15   # 标题+分隔+11步+分隔(=14行)+当前步标题行（索引14）
        tail_want = budget - keep_head - 1   # -1 给折叠提示行
        return lines[:keep_head] + ["  …（明细已折叠，j/k 切步或加高窗口看全量）"] \
            + lines[len(lines) - tail_want:]

    @staticmethod
    def _wrap(text: str, width: int = 62, indent: int = 0) -> list[str]:
        import unicodedata

        def wlen(s: str) -> int:
            return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)

        out, line, used = [], "", indent
        pad = " " * indent
        for word in text.split(" "):
            piece = word if not line else " " + word
            if wlen(pad + line + piece) > width and line:
                out.append(pad + line)
                line, used = word, 0
            else:
                line += piece
        if line:
            out.append(pad + line)
        return out or [pad]

    def render_frame(self) -> str:
        """无闪烁整帧：光标归位 + 每行行尾擦除；比上一帧短时补空行擦残留。

        不用 \x1b[2J 整屏清屏——那个每次重绘都闪，且与"内容高于窗口=每帧滚动"
        叠加就是满屏刷（2026-10-21 用户实测）。末行不带换行：正好贴住窗口底部
        时多打一个 \n 会顶掉首行。纯函数可单测。
        """
        lines = self.draw().split("\n")
        out = "\x1b[H" + "\n".join(ln + "\x1b[K" for ln in lines)
        pad = self._last_lines - len(lines)
        if pad > 0:
            out += "\n\x1b[K" * pad
        self._last_lines = len(lines)
        return out

    def exit_summary(self) -> str:
        c = self.state.counts()
        return (f"已退出面板 —— 通过 {c[S_PASS]} / 警告 {c[S_WARN]} / 失败 {c[S_FAIL]} / "
                f"跳过 {c[S_SKIP]} / 待执行 {c[S_PENDING]}（重进面板：菜单选 1 或 3）")

    def run(self):
        os.system("")   # Windows conhost 激活 VT 转义
        for stream in (sys.stdout, sys.stderr):
            try:
                stream.reconfigure(encoding="utf-8")
            except Exception:  # noqa: BLE001
                pass
        self._last_lines = 0
        self.keys = KeyReader()
        self.keys.start()   # 漏了 start() 面板收不到任何键（2026-10-21 审计实锤）
        print("\x1b[?25l", end="", flush=True)   # 隐藏光标，防每帧闪烁
        try:
            while True:
                print(self.render_frame(), flush=True)
                deadline = time.time() + 0.5
                while time.time() < deadline:
                    try:
                        ch = self.keys.q.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    if self.handle_key(ch):
                        return
        finally:
            print("\x1b[?25h", end="", flush=True)   # 恢复光标
            self.keys.stop()
            self.stop_flag.set()
            # 退出清屏：不清的话最后一帧永远挂在屏上、菜单内容接在它下面
            # （用户看到"旧窗口保留"）；末帧末行无换行还会让下一个提示粘在
            # 热键行尾巴上。清掉并留一行状态摘要补偿信息损失（2026-10-21）。
            print("\x1b[2J\x1b[H" + self.exit_summary(), flush=True)


# ---------------------------------------------------------------- 彩排脚本
def rehearse_script() -> list[tuple[str, int, object, str]]:
    """FakeTransport 脚本：健康行的 t= 用当前时间，保证 S7 判龄永远新鲜。"""
    def health_out(_cmd):
        t = time.time()
        return (f"[px4_odom_convert_health] t={t:.3f} guard=OK armed=0 aligned=1 "
                f"ekf_valid=1 slam_frames=129 slam_hz=43.0 hold_ratio=0.0 "
                f"jump_max_m=0.01 jump_mean_m=0.0 cov_max=0.0 l1_lock=0 "
                f"l1_age_s=-1.0 l1_total=0 nan_drop=0 z_src=1 bump_age_s=-1.0\n")

    def status_out(_cmd):
        wins = ["dzuav", "plan", "livox", "fastlio", "px4", "odom",
                "imu", "control", "path", "teleop"]
        return "\n".join(f"  [活] {w}" for w in wins) + "\n"

    def keyfile_out(_cmd):
        return "\n".join([
            "WIZARD_OK /home/duzhong/dzuav_deploy/air_ground/onboard/dzuav/build/dzuav",
            "WIZARD_OK /home/duzhong/dzuav/engines/hbb_detector_fp16.engine",
            "WIZARD_OK /home/duzhong/dzuav/engines/cell_classifier_fp16.engine",
            "WIZARD_OK /dev/video0", ""])

    def dzuav_out(_cmd):
        return ("opening camera: v4l2src device=/dev/video0 ...\n"
                " localized=1 landmarks=6 [{\"class\":\"mine\",\"center\":[0.9,1.8]}]\n"
                " localized=1 landmarks=7 []\n")

    return [
        (r"WIZARD_PING_OK", 0, "WIZARD_PING_OK\n", ""),
        (r"WIZARD_OK|WIZARD_MISS|for f in", 0, keyfile_out, ""),
        (r"orin_bringup\.sh up", 0, "==>[通过] dzuav（UDP:50000 广播）\n==== 全栈已启动 ====\n", ""),
        (r"orin_bringup\.sh status", 0, status_out, ""),
        (r"orin_bringup\.sh down", 0, "已停止 agup\n", ""),
        (r"ros2 topic hz", 0, "average rate: 43.157\n   min: 0.020s max: 0.031s\n", ""),
        (r"capture-pane.*dzuav", 0, dzuav_out, ""),
        (r"node_odom_.*log|px4_odom_convert_health", 0, health_out, ""),
    ]


def start_fake_udp(port: int) -> subprocess.Popen | None:
    """彩排：拉起 fake_udp_sender 子进程模拟 dzuav 广播（S8 有帧可收）。

    走回环单播而非广播：实测部分 Windows 上 255.255.255.255 不回环给本机
    监听者（防火墙/接口策略），单播 127.0.0.1 在所有机器都可达。
    """
    try:
        return subprocess.Popen(
            [sys.executable, str(TOOLS / "fake_udp_sender.py"),
             "--port", str(port), "--rate", "10", "--to", "127.0.0.1"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError as e:  # pragma: no cover - 启不动只影响 S8
        print(f"[彩排] fake_udp_sender 启动失败（S8 会 0 帧）: {e}")
        return None


# ---------------------------------------------------------------- 非交互自检
def run_check(ctx: WizardContext) -> int:
    """--check：顺序跑全部 check 步（action/display 只标注），退出码可进 CI。"""
    tty = sys.stdout.isatty()
    state = WizardState(build_steps())
    fails = warns = 0
    for step in state.steps:
        if step.kind != "check":
            state.apply(step.sid, _res(S_SKIP, "非交互自检不执行（面板内确认后执行）"))
        else:
            print(f"[..] {step.sid} {step.title} ...", flush=True)
            state.apply(step.sid, run_step(step, ctx))
        st = state.status[step.sid]
        if st == S_FAIL:
            fails += 1
        elif st == S_WARN:
            warns += 1
        icon = {"pass": "✓", "warn": "!", "fail": "✗", "skip": "–"}.get(st, "·")
        label = {"pass": "通过", "warn": "警告", "fail": "失败", "skip": "跳过"}.get(st, st)
        color = {"pass": ANSI_GREEN, "warn": ANSI_YELLOW, "fail": ANSI_RED,
                 "skip": ANSI_DIM}.get(st, "")
        end = ANSI_RESET if tty else ""
        head = f"[{icon}{label}] {step.sid} {step.title}" if tty else f"[{label}] {step.sid} {step.title}"
        print((color if tty else "") + head + end)
        for ln in state.detail[step.sid]:
            print(f"    {ln}")
    verdict = "PASS" if fails == 0 else "FAIL"
    print(f"[体检结论] {verdict} —— {fails} 失败 / {warns} 警告"
          f"（warning 不拦截，失败必须处理）")
    return 0 if fails == 0 else 1


# ---------------------------------------------------------------- 入口
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="无人机操作向导：SOP 步骤引导面板（详见 docs/无人机操作手册.md）")
    ap.add_argument("--host", default="", help="Orin IP（--rehearse 可省）")
    ap.add_argument("--user", default="duzhong")
    ap.add_argument("--port", type=int, default=22)
    ap.add_argument("--password", default=None, help="SSH 密码（或 AG_ORIN_PASS）")
    ap.add_argument("--deploy", default=DEPLOY, help=f"机上部署目录（默认 {DEPLOY}）")
    ap.add_argument("--check", action="store_true", help="非交互全量自检（退出码可进冒烟/CI）")
    ap.add_argument("--rehearse", action="store_true",
                    help="无硬件彩排：Fake 传输 + 本机假 UDP 流（训练/演示）")
    ap.add_argument("--udp-port", type=int, default=50000,
                    help="S8 探针端口（彩排默认自动换 50123 防与真链路冲突）")
    ap.add_argument("--udp-seconds", type=float, default=5.0, help="S8 探针时长（秒）")
    return ap


def resolve_password(args) -> str:
    pw = args.password or os.environ.get("AG_ORIN_PASS")
    if pw:
        return pw
    if not sys.stdin.isatty():
        raise SystemExit("缺少密码：set AG_ORIN_PASS=******（或 --password）")
    import getpass
    return getpass.getpass(f"{args.user}@{args.host} 的密码: ")


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    udp_port = args.udp_port
    if args.rehearse:
        transport = FakeTransport(rehearse_script())
        if udp_port == 50000:
            udp_port = 50123   # 防与真实链路/别的监听冲突
        ctx = WizardContext(transport=transport, host="rehearse", rehearse=True,
                            udp_port=udp_port, udp_seconds=args.udp_seconds,
                            deploy=args.deploy)
        fake = start_fake_udp(udp_port)
        try:
            if args.check:
                return run_check(ctx)
            Panel(WizardState(build_steps()), ctx).run()
            return 0
        finally:
            if fake is not None:
                fake.terminate()
    else:
        if not args.host:
            build_parser().error("--host 必填（或用 --rehearse 彩排）")
        ctx = WizardContext(transport=SshTransport(args.host, args.user,
                                                   resolve_password(args), args.port),
                            host=args.host, udp_port=udp_port,
                            udp_seconds=args.udp_seconds, deploy=args.deploy)
        if args.check:
            return run_check(ctx)
        Panel(WizardState(build_steps()), ctx).run()
        return 0


if __name__ == "__main__":
    sys.exit(main())
