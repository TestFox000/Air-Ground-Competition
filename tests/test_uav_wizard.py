"""uav_wizard 单测：零硬件、零网络、零 paramiko（SshTransport 冒烟除外）。

覆盖：
  * 纯解析器：tmux [活]/[死]、localized= 行、健康行判龄/风暴、ros2 topic hz、关键件清单；
  * 步骤状态机：跳过/通过/失败流转；
  * FakeTransport 端到端：S0~S10 全跑（=--rehearse --check 的等价逻辑路径），
    其中 S2 彩排跳过、S3/S9/S10 非交互不执行；
  * UDP 探针：本机起 fake_udp_sender 子进程（回环，非外发断言网），
    用 intel_protocol 与机载 C++ 互逆的同一编码器喂帧；
  * 确认闸门：确认词不符/取消都不执行动作（flight_console 旧教训回归）。
"""

from __future__ import annotations

import importlib.util
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]


def load_module(name: str, file: Path):
    spec = importlib.util.spec_from_file_location(name, file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


wiz = load_module("uav_wizard", REPO / "tools" / "uav_wizard.py")


# ---------------------------------------------------------------- 解析器
class TestParsers:
    def test_bringup_status_alive_dead(self):
        text = ("dzuav: 9窗 \n  [活] dzuav\n  [活] livox\n  [死] px4\n  [活] odom\n")
        session, wins = wiz.parse_bringup_status(text)
        assert session
        assert wins == {"dzuav": True, "livox": True, "px4": False, "odom": True}

    def test_bringup_status_no_session(self):
        session, wins = wiz.parse_bringup_status("会话 agup 不存在\n")
        assert not session
        assert wins == {}

    def test_localized_last_wins(self):
        text = ("a.jpg localized=0 landmarks=2 []\n"
                " localized=1 landmarks=6 [{...}]\n")
        assert wiz.parse_localized(text) == (1, 6)
        assert wiz.parse_localized("no output") is None

    def test_topic_hz(self):
        text = "average rate: 43.157\n   min: 0.020s max: 0.031s std dev: 0.00\n"
        assert wiz.parse_topic_hz(text) == pytest.approx(43.157)
        assert wiz.parse_topic_hz("no samples") is None

    def test_keyfile_listing(self):
        text = ("WIZARD_OK /a/b\nWIZARD_MISS /c/d\nnoise line\n")
        assert wiz.parse_keyfile(text) == [("/a/b", True), ("/c/d", False)]

    def test_health_fresh_pass(self):
        line = (f"[px4_odom_convert_health] t={time.time():.3f} guard=OK slam_hz=43.0 "
                "l1_lock=0 z_src=1 nan_drop=0")
        status, notes = wiz.judge_health(wiz.last_health_line(line), time.time())
        assert status == wiz.S_PASS

    def test_health_stale_fail(self):
        line = (f"[px4_odom_convert_health] t={time.time() - 120:.3f} guard=OK slam_hz=43.0")
        status, _ = wiz.judge_health(wiz.last_health_line(line), time.time())
        assert status == wiz.S_FAIL

    def test_health_storm_and_l1(self):
        base = "slam_hz=43.0 l1_lock=0 z_src=1 nan_drop=0"
        storm = f"[px4_odom_convert_health] t={time.time():.3f} guard=OK+STORM {base}"
        l1 = (f"[px4_odom_convert_health] t={time.time():.3f} guard=L1_LOCK "
              "slam_hz=0 l1_lock=1 z_src=1 nan_drop=0")
        assert wiz.judge_health(wiz.last_health_line(storm), time.time())[0] == wiz.S_FAIL
        assert wiz.judge_health(wiz.last_health_line(l1), time.time())[0] == wiz.S_FAIL

    def test_health_degraded_warn(self):
        line = (f"[px4_odom_convert_health] t={time.time():.3f} guard=HOLD "
                "slam_hz=30 l1_lock=0 z_src=0 nan_drop=0")
        assert wiz.judge_health(wiz.last_health_line(line), time.time())[0] == wiz.S_WARN

    def test_health_missing(self):
        assert wiz.judge_health(None, time.time())[0] == wiz.S_FAIL


# ---------------------------------------------------------------- 状态机与传输
class TestStateAndTransport:
    def test_state_flow(self):
        st = wiz.WizardState(wiz.build_steps())
        assert st.counts()[wiz.S_PENDING] == len(st.steps)
        st.apply("S0", wiz.CheckResult(wiz.S_PASS, ["ok"]))
        assert st.status["S0"] == wiz.S_PASS
        assert not st.all_green()
        for s in st.steps:
            if s.sid != "S0":
                st.apply(s.sid, wiz.CheckResult(wiz.S_WARN, []))
        assert st.all_green()

    def test_fake_transport_script_and_calls(self):
        ft = wiz.FakeTransport([(r"echo X", 0, "X\n", "")])
        code, out, _ = ft.run("echo X")
        assert (code, out) == (0, "X\n")
        assert ft.run("unmatched")[2].startswith("no scripted response")
        assert ft.calls == ["echo X", "unmatched"]

    def test_fake_transport_callable_out(self):
        ft = wiz.FakeTransport([(r"health", 0, lambda cmd: f"t={time.time()}", "")])
        assert "t=" in ft.run("tail health")[1]


# ---------------------------------------------------------------- 端到端（=rehearse 等价）
class TestEndToEnd:
    def test_all_steps_with_fake_transport(self):
        ctx = wiz.WizardContext(transport=wiz.FakeTransport(wiz.rehearse_script()),
                                host="rehearse", rehearse=True, udp_seconds=0.8)
        st = wiz.WizardState(wiz.build_steps())
        for step in st.steps:
            res = wiz.run_step(step, ctx)
            st.apply(step.sid, res)
            # 彩排脚本全绿；S2 彩排跳过；S3/S9/S10 不在 check 集内单独验证
            if step.sid in ("S2", "S9", "S10"):
                continue
            if step.sid == "S8":
                # 无假流时 S8 允许失败（本用例不起子进程），单独在探针用例覆盖
                continue
            assert res.status in (wiz.S_PASS, wiz.S_WARN), \
                f"{step.sid} 意外状态 {res.status}: {res.detail}"

    def test_s3_action_and_confirm_word_defined(self):
        s3 = next(s for s in wiz.build_steps() if s.sid == "S3")
        assert s3.kind == "action" and s3.confirm_word == "y"
        ctx = wiz.WizardContext(transport=wiz.FakeTransport(wiz.rehearse_script()),
                                rehearse=True)
        assert wiz.run_step(s3, ctx).status == wiz.S_PASS

    def test_s1_reports_missing(self):
        def miss_out(_cmd):
            return ("WIZARD_OK /home/x/dzuav\n"
                    "WIZARD_MISS /home/x/hbb_detector_fp16.engine\n"
                    "WIZARD_MISS /dev/video0\n")
        ft = wiz.FakeTransport([(r"for f in", 0, miss_out, "")])
        res = wiz.run_step(next(s for s in wiz.build_steps() if s.sid == "S1"),
                           wiz.WizardContext(transport=ft))
        assert res.status == wiz.S_FAIL
        assert any("engine" in d for d in res.detail)

    def test_s4_session_missing_hint(self):
        ft = wiz.FakeTransport([(r"orin_bringup\.sh status", 0, "会话 agup 不存在\n", "")])
        res = wiz.run_step(next(s for s in wiz.build_steps() if s.sid == "S4"),
                           wiz.WizardContext(transport=ft))
        assert res.status == wiz.S_FAIL
        assert any("S3" in d for d in res.detail)


# ---------------------------------------------------------------- UDP 探针（回环）
class TestUdpProbe:
    def test_probe_receives_fake_sender(self):
        # 临时端口起 fake_udp_sender（与机载 C++ 字节级一致的车端编码器）
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("", 0))
            port = probe.getsockname()[1]
        proc = subprocess.Popen(
            [sys.executable, str(REPO / "tools" / "fake_udp_sender.py"),
             "--port", str(port), "--rate", "20", "--to", "127.0.0.1"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            time.sleep(0.4)   # 等首发
            r = wiz.udp_probe(port, 1.5)
            assert r["good"] >= 5, r
            assert r["bad"] == 0
            assert r["fps"] > 3
        finally:
            proc.terminate()

    def test_probe_timeout_zero_frames(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(("", 0))
            port = s.getsockname()[1]
        r = wiz.udp_probe(port, 0.5)
        assert r["good"] == 0 and r["fps"] == 0


# ---------------------------------------------------------------- 面板确认闸门
class TestConfirmGate:
    def make_panel(self):
        st = wiz.WizardState(wiz.build_steps())
        ctx = wiz.WizardContext(transport=wiz.FakeTransport([]), rehearse=True)
        return wiz.Panel(st, ctx), st

    def test_keyreader_stop_terminates_thread(self):
        """回归×2：① Panel.run 曾创建 KeyReader 不 start()——面板收不到任何键；
        ② stop() 曾只恢复 termios 不停循环——菜单模式反复进出面板时旧线程
        抢键盘（偶发吃键）。"""
        kr = wiz.KeyReader()
        kr.start()
        time.sleep(0.2)
        assert kr.is_alive()
        kr.stop()
        kr.join(timeout=2.0)
        assert not kr.is_alive(), "KeyReader.stop() 后线程必须退出"

    def test_confirm_wrong_word_no_execute(self):
        panel, _ = self.make_panel()
        fired = []
        panel.ask_confirm("y", "测试", lambda: fired.append(1))
        for ch in "n\r":                 # 输错词 + 回车
            assert panel.handle_key(ch) is False   # 确认中的键绝不退出面板
        assert fired == []

    def test_confirm_right_word_executes(self):
        panel, _ = self.make_panel()
        fired = []
        panel.ask_confirm("y", "测试", lambda: fired.append(1))
        for ch in "y\r":
            assert panel.handle_key(ch) is False
        assert fired == [1]

    def test_confirm_cancel_then_q_quits(self):
        panel, _ = self.make_panel()
        panel.ask_confirm("y", "测试", lambda: None)
        assert panel.handle_key("q") is False      # 确认中 q=取消
        assert panel.handle_key("q") is True       # 取消后 q 才退出

    def test_enter_on_s3_pushes_confirm_not_execute(self):
        panel, st = self.make_panel()
        st.idx = next(i for i, s in enumerate(st.steps) if s.sid == "S3")
        panel.handle_key("\r")
        assert panel.pending_action is not None    # 进确认而非直接执行
        assert st.status["S3"] == wiz.S_PENDING    # 未执行

    def test_j_k_navigation(self):
        panel, st = self.make_panel()
        panel.handle_key("j")
        assert st.idx == 1
        panel.handle_key("k")
        assert st.idx == 0
        panel.handle_key("DOWN")
        assert st.idx == 1

    def test_unknown_printable_key_gets_visible_feedback(self):
        """回归（2026-10-21 现场反馈）：面板无输入行，敲错键若无声无息，
        用户会误以为"输入没反应"。未知可打印键必须在日志区提示。"""
        panel, st = self.make_panel()
        panel.handle_key("z")
        assert any("未知键" in text for _, text, _ in panel.log)
        assert st.idx == 0            # 状态不受扰动
        panel.handle_key("\x1b")      # 控制字符不提示（避免 ESC 刷屏）
        assert not any("\x1b" in text for _, text, _ in panel.log)

    def test_panel_exit_clears_screen_and_prints_summary(self, monkeypatch, capsys):
        """回归（2026-10-21 现场反馈"按 q 退出旧窗口保留"）：退出时必须清屏
        并打印状态摘要——不清的话最后一帧永久留在屏上，菜单内容接在旧帧
        下面，末帧末行无换行还会让下一个提示粘在热键行尾巴上。
        运行期重绘严禁整屏清屏（闪烁），只有退出这一次。"""
        import queue as _queue

        class FakeKR:
            def __init__(self):
                self.q = _queue.Queue()

            def start(self):
                self.q.put("q")     # 启动即投递退出键：跑一帧就退出

            def stop(self):
                pass

        monkeypatch.setattr(wiz, "KeyReader", FakeKR)
        st = wiz.WizardState(wiz.build_steps())
        st.apply("S0", wiz.CheckResult(wiz.S_PASS, ["ok"]))
        st.apply("S4", wiz.CheckResult(wiz.S_FAIL, ["dead"]))
        panel = wiz.Panel(st, wiz.WizardContext(
            transport=wiz.FakeTransport([]), rehearse=True))
        panel.run()
        out = capsys.readouterr().out
        assert out.count("\x1b[2J\x1b[H") == 1, "整屏清屏应只在退出时出现一次"
        assert "\x1b[?25h" in out                    # 光标已恢复
        assert "已退出面板" in out and "通过 1" in out and "失败 1" in out

    def test_panel_draw_renders_all_sections(self):
        """回归：fail_hints 曾以嵌套列表混入行数组使 draw 抛 TypeError。
        选矮步骤 S8（20 行，不触发高度折叠）验全区块；折叠行为另有专测。"""
        panel, st = self.make_panel()
        st.idx = 8
        st.apply("S8", wiz.CheckResult(wiz.S_PASS, ["25 好帧 / 9.2 fps"]))
        out = panel.draw()
        assert isinstance(out, str) and out.count("\n") > 10
        assert "S8 UDP 出网" in out
        for section in ("作用:", "命令:", "通过标准:", "明细:", "j/k"):
            assert section in out, f"缺区块 {section}"

    def test_panel_clamps_tall_content_with_marker(self):
        """高帧折叠：保留头尾，中段换成折叠提示，总高不超预算。"""
        panel, st = self.make_panel()
        st.idx = 0
        st.detail["S0"] = [f"行{j}" for j in range(20)]
        out = panel.draw()
        lines = out.split("\n")
        assert len(lines) <= 24
        assert any("折叠" in ln for ln in lines)

    def test_clamp_keeps_current_step_title(self):
        """回归（2026-10-22 复审）：keep_head=14 差一行，折叠帧把当前步标题
        （索引 14）切掉——折叠后看不到自己选的是哪步。"""
        panel, st = self.make_panel()
        st.idx = 0
        st.detail["S0"] = [f"行{j}" for j in range(20)]
        out = panel.draw()
        assert "S0 网络连通 ·" in out, "折叠帧必须保留当前步标题行"

    def test_panel_height_fits_default_cmd_window(self):
        """回归（2026-10-21 用户实测"一直刷新"）：面板最高 31 行超过 cmd 默认
        25 行窗口，每帧滚动=满屏刷。任何步骤选中+富明细下总高必须 ≤24。"""
        panel, st = self.make_panel()
        for i, step in enumerate(st.steps):
            st.idx = i
            rich = [f"明细行{j}" for j in range(8)]
            st.detail[step.sid] = rich
            height = panel.draw().count("\n") + 1
            assert height <= 24, f"{step.sid} 面板 {height} 行，超出 24 行预算"

    def test_render_frame_flicker_free_and_pads(self):
        """回归：整屏清屏 \x1b[2J 每帧闪；改归位+行尾擦除，缩短帧要补空行擦残留。"""
        panel, st = self.make_panel()
        st.idx = 9   # S9 指引长 → 高帧
        tall = panel.render_frame()
        assert tall.startswith("\x1b[H")
        assert "\x1b[2J" not in tall
        assert "\x1b[K" in tall
        st.idx = 8   # S8 矮帧
        short = panel.render_frame()
        pad = short.count("\n\x1b[K")
        assert pad >= 1, "变矮的帧必须补空行擦掉上一帧残留"
        assert not short.endswith("\n"), "末行不能带换行（贴底会顶掉首行）"


# ---------------------------------------------------------------- SshTransport 仅构造冒烟
def test_ssh_transport_lazy_no_paramiko_needed_at_import():
    # 模块顶层已成功 import（本文件 load_module 处隐式验证）；
    t = wiz.SshTransport("127.0.0.1", "u", "p", port=22)
    assert t.cli is None and t.port == 22


def test_ssh_transport_lc_all_prefix_uses_export_form():
    """回归（2026-10-22 复审，真机致命）：LC_ALL 前缀形式 `LC_ALL=... for ...`
    修饰保留字是 sh 语法错误——S1 的 for 循环命令在真机必挂；必须 export 形式。"""
    t = wiz.SshTransport("127.0.0.1", "u", "p")
    cmd = "for f in ~/a ~/b; do [ -e $f ] && echo ok; done"
    full = t._full(cmd)
    assert full == f"export LC_ALL=C.UTF-8; {cmd}"
    assert not full.startswith("LC_ALL=")


def test_ssh_transport_paramiko_missing_becomes_fail_not_hang(monkeypatch):
    """缺 paramiko 时 remote_run 导入即 SystemExit(2)（BaseException），会穿透
    worker 的 except Exception 让线程无声死掉、S0 永卡"运行中"——必须转成
    普通 RuntimeError 走 S0 FAIL 路径（2026-10-21 闪退排查时实锤加固）。

    用 meta_path finder 在导入点抛 SystemExit：比 sys.path 桩可靠——_ssh 自身
    会 insert(0, TOOLS) 抢在最前，路径桩排不到它前面。
    """
    class _RaisingFinder:
        def find_spec(self, fullname, path=None, target=None):
            if fullname == "remote_run":
                raise SystemExit(2)
            return None

    finder = _RaisingFinder()
    sys.meta_path.insert(0, finder)
    # 顺序隔离：同进程先跑过 test_remote_run 会把真 remote_run 缓存进
    # sys.modules，import 不再触发 finder（CI 随机序下必踩）。先摘掉，
    # 结束后原样放回。
    saved_rr = sys.modules.pop("remote_run", None)
    try:
        t = wiz.SshTransport("127.0.0.1", "u", "p")
        with pytest.raises(RuntimeError, match="paramiko"):
            t._ssh()
        # 整步收敛：S0 应 FAIL 且带指引，而不是卡死
        s0 = next(s for s in wiz.build_steps() if s.sid == "S0")
        res = wiz.run_step(s0, wiz.WizardContext(transport=t, host="127.0.0.1"))
        assert res.status == wiz.S_FAIL
        assert any("paramiko" in d or "rehearse" in d for d in res.detail)
    finally:
        sys.meta_path.remove(finder)
        if saved_rr is not None:
            sys.modules["remote_run"] = saved_rr
        else:
            sys.modules.pop("remote_run", None)


def test_check_mode_exit_code_with_fakes(capsys):
    """--check 语义：check 步执行、action/display 跳过、退出码随失败。

    S8 要有真帧才绿：临时端口起 fake_udp_sender（与机载 C++ 字节级一致的
    车端编码器），探针采样同一个端口。
    """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("", 0))
        port = probe.getsockname()[1]
    proc = subprocess.Popen(
        [sys.executable, str(REPO / "tools" / "fake_udp_sender.py"),
         "--port", str(port), "--rate", "20", "--to", "127.0.0.1"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(0.4)
        ctx = wiz.WizardContext(
            transport=wiz.FakeTransport(wiz.rehearse_script()),
            host="rehearse", rehearse=True, udp_port=port, udp_seconds=1.2)
        assert wiz.run_check(ctx) == 0
        out = capsys.readouterr().out
        assert "S0" in out and "体检结论" in out
        assert "[失败]" not in out
    finally:
        proc.terminate()
