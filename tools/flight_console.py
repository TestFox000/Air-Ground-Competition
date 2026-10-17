#!/usr/bin/env python3
"""无人机地面台：实时看到无人机在干什么 + 应急指挥 + 黑匣子管理。

配套 orin_flight_blackbox.py（机上录制器）组成"飞行黑匣子三件套"：
  ① 【看】收黑匣子的 UDP 实时镜像，刷新遥测面板（模式/解锁/位置/姿态/电池/告警）；
  ② 【支配】键盘应急命令——优先走 px4_control 的安全通道（空中拒 DISARM、
     地面才 ARM），最后手段 KILL 直接发飞控（电机停转，空中=坠机，双确认）；
  ③ 【黑匣子】一键在机上 tmux 拉起录制器（黑匣子持续留痕），坠机后 --fetch 取回。

用法（笔记本上，密码走环境变量）:
    set AG_ORIN_PASS=******
    python tools/flight_console.py --host <Orin IP> --user <用户>
    python tools/flight_console.py --host ... --stop          # 只停机上黑匣子
    python tools/flight_console.py --host ... --fetch ./logs  # 取回机上黑匣子日志
    python tools/flight_console.py --host ... --skip-launch   # 黑匣子已在录，只开面板

按键:
    L=降落(确认)   K=停GOTO悬停   O=切Offboard   M=移动模式(WASDQE+l/r, 同 teleop)
    X=DISARM(确认, 空中被 px4_control 拒绝)   !=KILL 电机停转(输入全词, 最后手段)
    R=刷新   Q=退出面板(黑匣子继续在机上录)

命令走 /px4_control/command 安全通道的原理：px4_control_node 对每条命令有
状态防护（ARM 需地面+Offboard、DISARM 空中拒绝、LAND 按状态机），我们在
地面台只做"决策"，安全执行权留给机载状态机。所有命令（含 teleop 的人工
按键）都会被黑匣子 ctrl.cmd / fmu.cmd 事件审计留痕。

依赖: pip install paramiko（仅笔记本；复用 tools/remote_run.py 的连接原语）
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import socket
import sys
import threading
import time
from collections import deque
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import remote_run  # noqa: E402  连接重试 / sftp_put / connect 原语（踩过的坑都修过了）

try:
    import paramiko
except ImportError:  # pragma: no cover
    raise SystemExit("缺少 paramiko：python -m pip install paramiko")

# 与 reference/onboard/dzpx4/px4_control/msg/ControlCommand.msg 对齐
CMD_OFFBOARD, CMD_ARM, CMD_DISARM, CMD_LAND = 1, 2, 3, 4
CMD_MOVE, CMD_YAW, CMD_GOTO_ENABLE, CMD_GOTO_DISABLE = 5, 6, 7, 8
CTRL_NAMES = {1: "OFFBOARD", 2: "ARM", 3: "DISARM", 4: "LAND",
              5: "MOVE", 6: "YAW", 7: "GOTO_ON", 8: "GOTO_OFF"}

# 与 params.yaml 一致的步进量（teleop 同款）
STEP_XY, STEP_Z, STEP_YAW = 0.5, 0.1, 0.15

BB_REMOTE_TOOLS = "~/dzuav_deploy/air_ground/tools"
BB_REMOTE_LOGROOT = "~/ag_flight_logs"
TMUX_SESSION, TMUX_WIN = "agup", "bb"

ANSI_CLEAR = "\x1b[2J\x1b[H"
ANSI_RED = "\x1b[1;31m"
ANSI_YELLOW = "\x1b[1;33m"
ANSI_GREEN = "\x1b[1;32m"
ANSI_CYAN = "\x1b[1;36m"
ANSI_DIM = "\x1b[2m"
ANSI_RESET = "\x1b[0m"


def local_source_ip(host: str) -> str:
    """笔记本到 Orin 的出接口 IP（纯路由查询，不发包）——黑匣子镜像回这个地址。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((host, 9))
        return s.getsockname()[0]
    finally:
        s.close()


class KeyReader(threading.Thread):
    """跨平台单键读取线程：Windows msvcrt / POSIX termios raw。"""

    def __init__(self):
        super().__init__(daemon=True)
        self.q: "queue.Queue[str]" = queue.Queue()
        self._old_termios = None
        self._start_platform()

    def _start_platform(self):
        if os.name == "nt":
            import msvcrt
            self._msvcrt = msvcrt
            return
        import termios
        import tty
        self._tty = tty
        try:
            self._old_termios = termios.tcgetattr(0)
            tty.setcbreak(0)
        except termios.error:
            pass

    def run(self):
        while True:
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
            if k in (b"\x00", b"\xe0"):  # 方向键前缀：吃掉第二字节，忽略
                m.getch()
                return None
            return k.decode("utf-8", "replace")
        import select
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        if not r:
            return None
        ch = sys.stdin.read(1)
        return ch or None

    def stop(self):
        if os.name != "nt" and self._old_termios:
            import termios
            try:
                termios.tcsetattr(0, termios.TCSADRAIN, self._old_termios)
            except termios.error:
                pass


class MirrorListener(threading.Thread):
    """收黑匣子 UDP 镜像：维护 latest 状态 + 事件率 + 告警/指令流水。"""

    def __init__(self, port: int):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.latest: dict[str, dict] = {}
        self.stamps: dict[str, deque] = {}
        self.flow: deque = deque(maxlen=10)          # (时刻, 等级, 文本)
        self.last_mirror_ts = 0.0
        self._stop = threading.Event()
        self._last_alert: dict[str, float] = {}

    def run(self):
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            now = time.monotonic()
            self.last_mirror_ts = now
            for line in data.decode("utf-8", "replace").splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue
                ev = rec.get("ev", "")
                self.latest[ev] = rec
                dq = self.stamps.setdefault(ev, deque(maxlen=64))
                dq.append(now)
                self._check_alerts(ev, rec)

    def _check_alerts(self, ev: str, rec: dict):
        """镜像事件流上的告警检测（翻转/电池/定位失效），限每类 10s 一次。"""
        now = time.time()
        tstr = time.strftime("%H:%M:%S")

        def once(key: str, lvl: str, text: str):
            if now - self._last_alert.get(key, 0.0) > 10.0:
                self._last_alert[key] = now
                self.flow.appendleft((tstr, lvl, text))

        if ev == "flight.status" and rec.get("prev") is not None:
            prev, cur = rec["prev"], rec
            if prev.get("nav") != cur.get("nav"):
                once(f"nav", "warn" if not cur.get("failsafe") else "error",
                     f"模式翻转 {prev.get('nav')} -> {cur.get('nav_name')}")
            if prev.get("arming") != cur.get("arming"):
                once("arming", "info",
                     f"解锁翻转 {prev.get('arming')} -> {cur.get('arming_name')}")
            if cur.get("failsafe"):
                once("failsafe", "error", "PX4 FAILSAFE 触发！")
        elif ev == "flight.batt":
            rem = rec.get("remaining")
            if rem is not None and rem < 0.2:
                once("batt", "error", f"电池低 {round(rem * 100)}%")
        elif ev == "flight.pos":
            if rec.get("xy_valid") == 0:
                once("pos_invalid", "error", "飞控本地位置失效 xy_valid=0")
        elif ev == "fmu.cmd":
            name = rec.get("name", rec.get("command"))
            lvl = "error" if rec.get("lvl") == "error" else "warn"
            self.flow.appendleft((tstr, lvl, f"[飞控] {name} p1={rec.get('p1')} p2={rec.get('p2')}"))
        elif ev == "ctrl.cmd":
            name = rec.get("name", rec.get("cmd"))
            self.flow.appendleft((tstr, "info", f"[指令] {name} dx={rec.get('dx')} dy={rec.get('dy')} dz={rec.get('dz')}"))
        elif ev == "goto.state":
            self.flow.appendleft((tstr, "warn", f"GOTO {'开启' if rec.get('enabled') else '关闭并悬停'}"))

    def hz(self, ev: str) -> float:
        dq = self.stamps.get(ev)
        if not dq or len(dq) < 2:
            return 0.0
        dt = dq[-1] - dq[0]
        return round((len(dq) - 1) / dt, 1) if dt > 0 else 0.0

    def stop(self):
        self._stop.set()


class FlightConsole:
    def __init__(self, args):
        self.args = args
        self.host = args.host
        self.cli = None                      # 持久 SSH 连接（发命令延迟低）
        self.cmd_log: deque = deque(maxlen=10)   # (时刻, 文本, 等级)
        self.mode = "view"                   # view / move / confirm
        self.pending = None                  # 待确认动作描述
        self.pending_action = None
        self.confirm_need = ""
        self.confirm_buf = ""
        self.status_line = "就绪"
        self.keys = KeyReader()
        self.mirror = MirrorListener(args.mirror_port)
        self.lock = threading.Lock()

    # ------------------------------------------------------------ SSH 基础
    def ssh(self):
        """惰性连接 + 断线自动重连（复用 remote_run.connect 的 3 次重试）。"""
        if self.cli is None:
            self.cli = remote_run.connect(self.host, self.args.user, self.args.password,
                                          port=self.args.port, timeout=15.0)
        return self.cli

    def exec(self, cmd: str, timeout: float = 20.0) -> tuple[int, str, str]:
        cli = self.ssh()
        stdin, stdout, stderr = cli.exec_command(cmd, timeout=timeout)
        stdin.close()
        out = stdout.read().decode("utf-8", errors="replace")
        err = stderr.read().decode("utf-8", errors="replace")
        code = stdout.channel.recv_exit_status()
        return code, out, err

    def note(self, text: str, lvl: str = "info"):
        with self.lock:
            self.cmd_log.appendleft((time.strftime("%H:%M:%S"), text, lvl))

    # ------------------------------------------------------------ 黑匣子管理
    def ensure_blackbox(self):
        """上传录制器 + 生成启动脚本 + tmux 拉起（幂等：窗口存在则先杀）。"""
        mirror_addr = f"{local_source_ip(self.host)}:{self.args.mirror_port}"
        self.note(f"上传黑匣子 -> {BB_REMOTE_TOOLS}")
        remote_run.sftp_put(self.host, self.args.user, self.args.password,
                            str(TOOLS / "orin_flight_blackbox.py"),
                            f"{BB_REMOTE_TOOLS}/orin_flight_blackbox.py",
                            port=self.args.port)
        # 启动脚本动态生成（镜像地址注入），避免 tmux 多层引号地狱
        start_sh = (
            "#!/usr/bin/env bash\n"
            '# 由 flight_console.py 生成\n'
            'source "$(ls -d /opt/ros/*/setup.bash 2>/dev/null | head -1)" 2>/dev/null\n'
            'source "$HOME/dzpx4/install/setup.bash" 2>/dev/null\n'
            'exec python3 "$HOME/dzuav_deploy/air_ground/tools/orin_flight_blackbox.py" '
            f'--log-dir "{BB_REMOTE_LOGROOT}" --udp-mirror {mirror_addr}\n'
        )
        import base64
        payload = base64.b64encode(start_sh.encode("utf-8")).decode("ascii")
        code, out, err = self.exec(
            f"mkdir -p {BB_REMOTE_LOGROOT} && echo {payload} | base64 -d > {BB_REMOTE_LOGROOT}/bb_start.sh "
            f"&& chmod +x {BB_REMOTE_LOGROOT}/bb_start.sh "
            f"&& (tmux has-session -t {TMUX_SESSION} 2>/dev/null || tmux new-session -d -s {TMUX_SESSION} -n hold 'read') "
            f"&& tmux kill-window -t {TMUX_SESSION}:{TMUX_WIN} 2>/dev/null; "
            f"tmux new-window -d -t {TMUX_SESSION} -n {TMUX_WIN} 'bash {BB_REMOTE_LOGROOT}/bb_start.sh' "
            f"&& echo BB_STARTED")
        if "BB_STARTED" not in out:
            self.note(f"黑匣子启动异常 rc={code}: {(err or out).strip()[:120]}", "error")
            return False
        self.note(f"黑匣子已启动（tmux {TMUX_SESSION}:{TMUX_WIN}），镜像 -> {mirror_addr}")
        return True

    def stop_blackbox(self):
        code, _, err = self.exec(f"tmux kill-window -t {TMUX_SESSION}:{TMUX_WIN} 2>/dev/null; echo RC=$?")
        self.note("已停止机上黑匣子窗口" + (f"（{err.strip()[:80]}）" if err.strip() else ""))

    def fetch_logs(self, dest: str):
        n = remote_run.sftp_get(self.host, self.args.user, self.args.password,
                                BB_REMOTE_LOGROOT, dest, port=self.args.port)
        print(f"[已取回] {n} 个文件 -> {dest}（日志分析: python tools/log_report.py ...）")

    # ------------------------------------------------------------ 指挥通道
    def _ros_env_prefix(self) -> str:
        return ('source "$(ls -d /opt/ros/*/setup.bash 2>/dev/null | head -1)" 2>/dev/null; '
                'source "$HOME/dzpx4/install/setup.bash" 2>/dev/null; ')

    def send_ctrl_async(self, cmd: int, name: str, dx: float = 0.0, dy: float = 0.0,
                        dz: float = 0.0):
        """经 px4_control 安全通道发指令（异步，避免 SSH 往返卡住面板）。"""
        yaml = f"{{cmd: {cmd}, dx: {dx}, dy: {dy}, dz: {dz}}}"
        # ros2 topic pub --once 有 DDS 发现竞态（对端未匹配时首帧可能丢），
        # 连发 3 次兜底；px4_control 侧命令幂等（状态机守门），重复无害
        remote_cmd = (f'bash -lc \'{self._ros_env_prefix()}'
                      f'for i in 1 2 3; do ros2 topic pub --once /px4_control/command '
                      f'px4_control/msg/ControlCommand "{yaml}"; sleep 0.3; done\'')
        self.note(f"[地面台] {name} -> px4_control", "warn")

        def worker():
            try:
                code, out, err = self.exec(remote_cmd, timeout=25.0)
                if code == 0 and "publishing" in out:
                    self.note(f"[回执] {name} 已发布到 /px4_control/command", "info")
                else:
                    self.note(f"[回执] {name} 失败 rc={code}: {(err or out).strip()[-100:]}",
                             "error")
            except Exception as e:  # noqa: BLE001
                self.cli = None  # 下次重连
                self.note(f"[回执] {name} SSH 异常: {type(e).__name__}（已重置连接）", "error")

        threading.Thread(target=worker, daemon=True).start()

    def send_kill_async(self):
        """最后手段：直接向飞控发 COMMAND_COMPONENT_ARM_DISARM(400, p2=21196)。
        语义 = 电机立即停转，空中使用等同于主动坠机，仅在即将撞人/撞物时止损。"""
        yaml = ("{command: 400, param1: 0.0, param2: 21196.0, param3: 0.0, param4: 0.0, "
                "param5: 0.0, param6: 0.0, param7: 0.0, target_system: 1, target_component: 1, "
                "source_system: 1, source_component: 1, from_external: true}")
        # 同 send_ctrl_async：--once 有 DDS 发现竞态，连发 3 次兜底（KILL 幂等，
        # 第一帧生效后电机已停，后续帧不改变结果）
        remote_cmd = (f'bash -lc \'{self._ros_env_prefix()}'
                      f'for i in 1 2 3; do ros2 topic pub --once /fmu/in/vehicle_command '
                      f'px4_msgs/msg/VehicleCommand "{yaml}"; sleep 0.3; done\'')
        self.note("[!!] KILL 已发出：电机停转指令 -> /fmu/in/vehicle_command", "error")

        def worker():
            try:
                code, out, err = self.exec(remote_cmd, timeout=25.0)
                if code == 0 and "publishing" in out:
                    self.note("[回执] KILL 已发布（黑匣子已留痕）", "error")
                else:
                    self.note(f"[回执] KILL 失败 rc={code}: {(err or out).strip()[-80:]}",
                              "error")
            except Exception as e:  # noqa: BLE001
                self.cli = None
                self.note(f"[回执] KILL SSH 异常: {type(e).__name__}", "error")

        threading.Thread(target=worker, daemon=True).start()

    # ------------------------------------------------------------ 确认交互
    def ask_confirm(self, need: str, action, desc: str):
        self.mode = "confirm"
        self.confirm_need = need
        self.confirm_buf = ""
        self.pending_action = action
        self.pending = desc

    def confirm_key(self, ch: str) -> bool:
        """确认模式按键处理。恒返回 True（确认模式下 q 是取消而非退出，防误操作）。"""
        if ch in ("\x1b", "q"):
            self.mode, self.pending = "view", None
            self.note("已取消")
            return True
        if ch == "\r" or ch == "\n":
            if self.confirm_buf.strip().lower() == self.confirm_need:
                self.mode, self.pending = "view", None
                self.pending_action()
            else:
                self.mode, self.pending = "view", None
                self.note("确认词不符，已取消", "warn")
            return True
        if ch == "\x7f":  # backspace
            self.confirm_buf = self.confirm_buf[:-1]
            return True
        if ch.isprintable():
            self.confirm_buf += ch
        return True

    # ------------------------------------------------------------ 面板
    def draw(self) -> str:
        m = self.mirror
        L: list[str] = []
        st = m.latest.get("flight.status", {})
        pos = m.latest.get("flight.pos", {})
        att = m.latest.get("flight.att", {})
        slam = m.latest.get("slam.odom", {})
        batt = m.latest.get("flight.batt", {})
        hb = m.latest.get("bb.hb", {})
        up = m.latest.get("goto.state", {})

        armed = st.get("arming_name", "?")
        nav = st.get("nav_name", "?")
        fs = st.get("failsafe")
        bb_alive = (time.monotonic() - m.last_mirror_ts) < 4.0 if m.last_mirror_ts else False

        # 状态行配色：armed+failsafe 红，armed 黄，其他 绿
        head_color = ANSI_RED if fs else (ANSI_YELLOW if armed == "ARMED" else ANSI_GREEN)
        L.append(f"{ANSI_CYAN}════════ 无人机地面台 · 飞行黑匣子 ════════{ANSI_RESET}")
        L.append(f" 链路 {self.host}  黑匣子: "
                 f"{ANSI_GREEN if bb_alive else ANSI_RED}{'● 录制中' if bb_alive else '○ 无镜像(检查 tmux bb 窗口)'}"
                 f"{ANSI_RESET}  事件率 {m.hz('flight.pos') + m.hz('slam.odom'):.1f}/s"
                 f"  机上运行 {hb.get('up', '?')}s"
                 f"  {ANSI_DIM}镜像端口 {self.args.mirror_port}{ANSI_RESET}")
        L.append("")
        L.append(f" 状态 : {head_color}● {armed}  {nav}{'   FAILSAFE!' if fs else ''}{ANSI_RESET}"
                 f"     GOTO: {'开' if up.get('enabled') else '关' if up else '—'}")
        z = pos.get("z")
        alt = f"{-z:.2f}" if isinstance(z, (int, float)) else "?"
        yaw = pos.get("yaw")

        def fmt(v):
            return f"{v:+.2f}" if isinstance(v, (int, float)) else " ?  "

        if pos:
            yaw_txt = f"{yaw * 57.3:+.0f}°" if isinstance(yaw, (int, float)) else "?"
            L.append(f" NED  : x {fmt(pos.get('x'))}  y {fmt(pos.get('y'))}"
                     f"  z {fmt(pos.get('z'))}   (高度 {alt} m)   yaw {yaw_txt}")
        else:
            L.append(" NED  : 位置数据未到")
        vx, vy = pos.get("vx"), pos.get("vy")
        speed = ((vx * vx + vy * vy) ** 0.5
                 if isinstance(vx, (int, float)) and isinstance(vy, (int, float)) else None)
        sp_txt = f"{speed:.2f} m/s" if speed is not None else "?"
        r_, p_, y_ = att.get("roll"), att.get("pitch"), att.get("yaw")
        att_txt = (f"roll {r_ * 57.3:+.0f}°  pitch {p_ * 57.3:+.0f}°  yaw {y_ * 57.3:+.0f}°"
                   if all(isinstance(v, (int, float)) for v in (r_, p_, y_)) else "姿态数据未到")
        L.append(f" 速度 : {sp_txt}    姿态: {att_txt}")
        L.append(f" SLAM : x {fmt(slam.get('x'))}  y {fmt(slam.get('y'))}  z {fmt(slam.get('z'))}"
                 f"   ({m.hz('slam.odom')}Hz)  {ANSI_DIM}FAST-LIO{ANSI_RESET}")
        rem = batt.get("remaining")
        rem_txt = f"{round(rem * 100)}%" if isinstance(rem, (int, float)) else "?"
        volt = batt.get("voltage")
        volt_txt = f"{volt:.2f}V" if isinstance(volt, (int, float)) else "?"
        L.append(f" 电池 : {rem_txt}  {volt_txt}    定位: "
                 f"{'valid' if pos.get('xy_valid') == 1 else 'INVALID' if pos else '—'}"
                 f"  pos话题 {m.hz('flight.pos')}Hz  status {m.hz('flight.status')}Hz")
        L.append("")

        # 断流检测（armed 时更醒目）
        stale = []
        for ev, name in (("flight.pos", "飞控位置"), ("slam.odom", "SLAM"),
                         ("flight.status", "飞控状态")):
            dq = m.stamps.get(ev)
            if armed == "ARMED" and (not dq or time.monotonic() - dq[-1] > 3.0):
                stale.append(name)
        if stale:
            L.append(f"{ANSI_RED} ⚠ 关键话题断流: {', '.join(stale)}（armed 状态下！）{ANSI_RESET}")
        L.append(f"{ANSI_DIM}──────────────── 告警 / 指令流水（新→旧）────────────────{ANSI_RESET}")
        if not m.flow:
            L.append(f"{ANSI_DIM} （暂无事件，等待黑匣子镜像…若 10s 仍无：tmux 里查 bb 窗口）{ANSI_RESET}")
        for tstr, lvl, text in list(m.flow)[:6]:
            color = {"error": ANSI_RED, "warn": ANSI_YELLOW, "info": ""}.get(lvl, "")
            L.append(f" {ANSI_DIM}{tstr}{ANSI_RESET} {color}{text[:80]}{ANSI_RESET}")
        L.append("")
        for tstr, text, lvl in list(self.cmd_log)[:4]:
            color = {"error": ANSI_RED, "warn": ANSI_YELLOW}.get(lvl, "")
            L.append(f" {ANSI_DIM}{tstr}{ANSI_RESET} {color}{text[:80]}{ANSI_RESET}")

        L.append("")
        if self.mode == "confirm":
            L.append(f"{ANSI_RED} ▶ 确认 [{self.pending}]：输入 {self.confirm_need!r} 回车执行，Esc 取消"
                     f"{ANSI_RESET}")
            L.append(f"   已输入: {'*' * len(self.confirm_buf)}")
        elif self.mode == "move":
            L.append(f"{ANSI_YELLOW} ▶ 移动模式（步进 {STEP_XY}m/{STEP_Z}m/{STEP_YAW}rad，"
                     f"同 teleop）: W/S前后 A/D左右 Q/E升降 L/R旋转  Esc退出{ANSI_RESET}")
        else:
            L.append(" [L]降落 [K]停GOTO悬停 [O]切Offboard [M]移动模式 [1]ARM [X]DISARM"
                 f" {ANSI_RED}[!]KILL(最后手段){ANSI_RESET} [Q]退出")
        L.append(f"{ANSI_DIM} 黑匣子在机: {BB_REMOTE_LOGROOT}/<会话>/  取回: --fetch <目录>  "
                 f"状态: {self.status_line}{ANSI_RESET}")
        return "\n".join(L)

    # ------------------------------------------------------------ 主循环
    def run_panel(self):
        if os.name == "nt":
            os.system("")  # 激活 conhost 的 VT 转义支持
        for s in (sys.stdout, sys.stderr):
            try:
                s.reconfigure(encoding="utf-8", errors="replace")
            except Exception:  # noqa: BLE001
                pass
        self.keys.start()
        self.mirror.start()
        try:
            while True:
                print(ANSI_CLEAR + self.draw(), flush=True)
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    try:
                        ch = self.keys.q.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    if self.handle_key(ch):
                        return
        finally:
            self.keys.stop()
            self.mirror.stop()

    def handle_key(self, ch: str) -> bool:
        """返回 True = 退出面板。"""
        k = ch.lower()
        if self.mode == "confirm":
            return self.confirm_key(ch)
        if self.mode == "move":
            if ch in ("\x1b", "m"):
                self.mode = "view"
                self.note("退出移动模式")
                return False
            steps = {"w": (CMD_MOVE, STEP_XY, 0, 0), "s": (CMD_MOVE, -STEP_XY, 0, 0),
                     "a": (CMD_MOVE, 0, -STEP_XY, 0), "d": (CMD_MOVE, 0, STEP_XY, 0),
                     "q": (CMD_MOVE, 0, 0, -STEP_Z), "e": (CMD_MOVE, 0, 0, STEP_Z),
                     "l": (CMD_YAW, STEP_YAW, 0, 0), "r": (CMD_YAW, -STEP_YAW, 0, 0)}
            if k in steps:
                cmd, dx, dy, dz = steps[k]
                self.send_ctrl_async(cmd, f"move({k})", dx, dy, dz)
            return False
        # ---- view 模式 ----
        if k == "q":
            return True
        if k == "l":
            self.ask_confirm("y", lambda: self.send_ctrl_async(CMD_LAND, "LAND"), "LAND 降落")
        elif k == "k":
            self.send_ctrl_async(CMD_GOTO_DISABLE, "GOTO_OFF(悬停)")
        elif k == "o":
            self.send_ctrl_async(CMD_OFFBOARD, "切Offboard")
        elif k == "1":
            self.ask_confirm("y", lambda: self.send_ctrl_async(CMD_ARM, "ARM+起飞"), "ARM 起飞")
        elif k == "m":
            self.mode = "move"
        elif k == "x":
            self.ask_confirm("disarm", lambda: self.send_ctrl_async(CMD_DISARM, "DISARM"),
                             "DISARM（空中会被 px4_control 拒绝）")
        elif k == "!":
            self.ask_confirm("kill", self.send_kill_async,
                             "KILL 电机停转（空中=坠机，仅撞人前止损）")
        return False


def read_password(args) -> str:
    pw = args.password or os.environ.get("AG_ORIN_PASS") or os.environ.get("AG_CAR_PASS")
    if pw:
        return pw
    import getpass
    if not sys.stdin.isatty():
        raise SystemExit("缺少 Orin 密码：set AG_ORIN_PASS=******（或 --password）")
    return getpass.getpass(f"{args.user}@{args.host} 的密码: ")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="无人机地面台：实时遥测 + 应急指挥 + 飞行黑匣子管理")
    ap.add_argument("--host", required=True, help="Orin IP")
    ap.add_argument("--user", default=os.environ.get("AG_ORIN_USER"),
                    help="Orin 登录用户（或 AG_ORIN_USER）")
    ap.add_argument("--password", default=None, help="密码；推荐 AG_ORIN_PASS 环境变量")
    ap.add_argument("--port", type=int, default=22)
    ap.add_argument("--mirror-port", type=int, default=7447,
                    help="接收黑匣子镜像的本地 UDP 端口（默认 7447）")
    ap.add_argument("--skip-launch", action="store_true",
                    help="不上传/不启动黑匣子，只开面板（黑匣子已在录时用）")
    ap.add_argument("--stop", action="store_true", help="停止机上黑匣子窗口后退出")
    ap.add_argument("--fetch", metavar="DIR", default=None,
                    help="取回机上 ~/ag_flight_logs 全部日志到本地 DIR 后退出（坠机复盘用）")
    args = ap.parse_args()
    if not args.user:
        ap.error("需要 --user（或设 AG_ORIN_USER）")
    args.password = read_password(args)

    con = FlightConsole(args)
    try:
        if args.stop:
            con.stop_blackbox()
            return 0
        if args.fetch:
            con.fetch_logs(args.fetch)
            return 0
        if not args.skip_launch:
            con.ensure_blackbox()
        con.run_panel()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if con.cli:
                con.cli.close()
        except Exception:  # noqa: BLE001
            pass
    print(f"\n黑匣子仍在机上录制（tmux {TMUX_SESSION}:{TMUX_WIN} 窗口）。"
          f"\n停止: python tools/flight_console.py --host {args.host} --user {args.user} --stop"
          f"\n取回: python tools/flight_console.py --host {args.host} --user {args.user} --fetch ./flight_logs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
