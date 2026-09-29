"""黑匣子（结构化日志）与复盘诊断的测试。

这里测的不是"日志能不能写"，而是**出了事能不能查出来**：
    1. 日志自己坏掉（磁盘满/目录是文件/进程被杀）时，主流程必须照常
    2. 字段不能互相覆盖 —— 业务字段把 `ev` 吃掉，整条记录就废了
    3. 复盘规则给出的结论要对得上现场真实卡点（没情报 / 没发车 / 白等打击）
"""

import importlib.util
import json
import time
from enum import IntEnum
from pathlib import Path

import pytest

from ag_core.blackbox import (
    AIM_BLOCK_WHY,
    EV,
    REASON,
    Event,
    BlackBox,
    default_session,
    read_events,
    session_start_index,
)
from ag_core.log_diagnosis import diagnose, summarize

ROOT = Path(__file__).resolve().parents[1]


def _load_tool(name: str):
    """按路径加载 tools/*.py（tools 不在包的 import 路径里）。"""
    spec = importlib.util.spec_from_file_location(f"_tool_{name}", ROOT / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)  # type: ignore[union-attr]
    return mod


def _lines(tmp_path: Path, name: str = "car_node") -> list:
    """读回该节点写下的所有 JSON 行（按文件顺序）。"""
    out = []
    for f in sorted(tmp_path.rglob("*.jsonl")):
        for line in f.read_text(encoding="utf-8").splitlines():
            if line.strip():
                out.append(json.loads(line))
    return out


# ------------------------------------------------------------------ 写入端
def test_write_basic_fields(tmp_path):
    bb = BlackBox("car_node", log_dir=str(tmp_path), session="s1")
    bb.log(EV.ROUTE_PUBLISH, kind="route", cmd="G8 N11,y")
    bb.close()

    recs = _lines(tmp_path)
    assert len(recs) == 1
    r = recs[0]
    # 框架字段齐全是复盘的前提：缺 ts 就没法排时间轴，缺 node 就分不清谁在说话
    for key in ("ts", "mono", "seq", "node", "ev", "lvl", "session"):
        assert key in r, key
    assert r["ev"] == EV.ROUTE_PUBLISH
    assert r["node"] == "car_node"
    assert r["session"] == "s1"
    assert r["kind"] == "route"
    assert r["cmd"] == "G8 N11,y"


def test_payload_cannot_shadow_framework_keys(tmp_path):
    """业务字段撞上框架字段时必须让路，否则 `node` 被覆盖，整条记录就废了。

    （`ev` 是 `log()` 的第一个形参，Python 自己就会拦，所以这里测能传进来的那几个。）
    """
    bb = BlackBox("n", log_dir=str(tmp_path), session="s")
    bb.log(EV.INTEL_RECV, node="假的", seq=999, ts=0.0, mines=["C3"])
    bb.close()

    r = _lines(tmp_path)[0]
    assert r["ev"] == EV.INTEL_RECV
    assert r["node"] == "n"
    assert r["seq"] == 1                     # 框架自己的递增序号
    assert r["data"] == {"node": "假的", "seq": 999, "ts": 0.0}
    assert r["mines"] == ["C3"]


def test_enum_and_unknown_objects_are_serialized(tmp_path):
    """现场会把枚举/自定义对象直接丢进来，不能因此写失败。"""
    class TaskType(IntEnum):
        MINE = 3

    class Weird:
        def __str__(self):
            return "weird!"

    bb = BlackBox("n", log_dir=str(tmp_path), session="s")
    bb.log(EV.PLAN_BUILT, tasks=[TaskType.MINE], obj=Weird())
    bb.close()

    r = _lines(tmp_path)[0]
    assert r["tasks"] == [3]
    assert r["obj"] == "weird!"
    assert bb.errors == 0


def test_min_level_filters_debug(tmp_path):
    bb = BlackBox("n", log_dir=str(tmp_path), session="s", min_level="info")
    bb.log(EV.ROS_LOG, level="debug", msg="看不到")
    bb.log(EV.ROS_LOG, level="info", msg="看得到")
    bb.close()
    assert [r["msg"] for r in _lines(tmp_path)] == ["看得到"]


def test_throttle_and_sampling():
    bb = BlackBox("n", log_dir=None, session="s", enabled=False)  # 不落盘，只测闸门
    assert bb.throttled("k", 60.0) is True      # 第一次必须放行
    assert bb.throttled("k", 60.0) is False
    assert bb.throttled("other", 60.0) is True  # 不同 key 互不影响

    hits = [bb.every("e", 3) for _ in range(6)]
    assert hits == [False, False, True, False, False, True]


def test_write_failure_never_raises(tmp_path):
    """日志目录不可用（磁盘满/路径是文件）时**不能在回调里抛异常**。"""
    not_a_dir = tmp_path / "file"
    not_a_dir.write_text("x", encoding="utf-8")

    bb = BlackBox("n", log_dir=str(not_a_dir), session="s")
    assert bb.enabled is False
    assert bb.errors >= 1
    bb.log(EV.NODE_ERROR, where="x")   # 不应抛出
    bb.close()


def test_rotation_keeps_bounded_files(tmp_path):
    bb = BlackBox("n", log_dir=str(tmp_path), session="s",
                  max_bytes=300, keep=2, flush_every=1)
    for i in range(40):
        bb.log(EV.INTEL_RECV, seq=i, mines=["C3", "G8", "D4", "A1", "B2"])
    bb.close()

    files = sorted(p.name for p in tmp_path.rglob("*.jsonl"))
    # 当前文件 + 最多 keep 份滚动文件（否则 /tmp 会被写满）
    assert len(files) <= 3, files
    assert any(".1." in f for f in files), files
    # 但也不能只留最后一条 —— 复盘要的正是"出事之前发生了什么"
    assert len(_lines(tmp_path)) >= 4


def test_default_session_from_env(monkeypatch):
    monkeypatch.setenv("AG_LOG_SESSION", "final run 3")
    assert default_session() == "final_run_3"      # 空格变下划线（文件名安全）
    monkeypatch.delenv("AG_LOG_SESSION")
    assert len(default_session()) == 8             # YYYYMMDD


def test_required_key_is_dropped_instead_of_written(tmp_path):
    """每次 log 都会补 `_log`，所以这里直接依赖节点侧不传该名字。"""
    bb = BlackBox("n", log_dir=str(tmp_path), session="s")
    bb.log(EV.AIM_PASS, cls=1)
    bb.close()
    assert _lines(tmp_path)[0]["ev"] == EV.AIM_PASS


# ------------------------------------------------------------------ 读取端
def test_read_events_sorts_and_counts_bad_lines(tmp_path):
    bb = BlackBox("a", log_dir=str(tmp_path), session="s")
    bb.log(EV.INTEL_RECV, seq=2)
    bb.log(EV.INTEL_RECV, seq=1)
    bb.close()
    f = next(tmp_path.rglob("*.jsonl"))
    with open(f, "a", encoding="utf-8") as fh:
        fh.write("{不是 json}\n\n")

    events, bad = read_events(tmp_path)
    assert bad == 1                       # 坏行要计数（日志本身损坏也是线索）
    assert [e.get("seq") for e in events] == [2, 1]   # 按写入时间排序，不是按内容
    assert events[0].src.endswith(".jsonl")


def test_events_from_multiple_nodes_merge_on_one_timeline(tmp_path):
    a = BlackBox("air_link_node", log_dir=str(tmp_path), session="s")
    b = BlackBox("car_node", log_dir=str(tmp_path), session="s")
    a.log(EV.LINK_TX, seq=1)
    b.log(EV.INTEL_RECV, seq=1)
    a.close()
    b.close()

    events, bad = read_events(tmp_path)
    assert bad == 0
    assert {e.node for e in events} == {"air_link_node", "car_node"}
    assert [e.ev for e in events] == [EV.LINK_TX, EV.INTEL_RECV]


def test_session_start_index_picks_last_run(tmp_path):
    bb = BlackBox("car_node", log_dir=str(tmp_path), session="s")
    bb.log(EV.SESSION_BEGIN, start_cell="N10")
    bb.log(EV.INTEL_RECV, seq=1)
    bb.log(EV.SESSION_BEGIN, start_cell="N10")   # 第二局
    bb.log(EV.ROUTE_PUBLISH, kind="route", cmd="x")
    bb.close()

    events, _ = read_events(tmp_path)
    idx = session_start_index(events)
    assert idx == 2
    assert [e.ev for e in events[idx:]] == [EV.SESSION_BEGIN, EV.ROUTE_PUBLISH]


def test_logger_mirror_writes_text_and_dedups(tmp_path):
    from ag_core.blackbox import _LoggerMirror

    class FakeLogger:
        def __init__(self):
            self.records = []

        def info(self, msg):
            self.records.append(msg)

        def warning(self, msg):
            self.records.append("W:" + msg)

    bb = BlackBox("n", log_dir=str(tmp_path), session="s")
    orig = FakeLogger()
    mirror = _LoggerMirror(orig, bb, dedup_s=60.0)

    mirror.info("只确认了 2 颗雷，暂不发车")
    mirror.info("只确认了 2 颗雷，暂不发车")   # 上游每帧都喊，不该进两条
    mirror.warning("桥洞情报可疑")
    bb.close()

    msgs = [r["msg"] for r in _lines(tmp_path)]
    assert msgs == ["只确认了 2 颗雷，暂不发车", "桥洞情报可疑"]
    # 去重只影响落盘，**真实 ROS 日志一条都不能少**
    assert orig.records == ["只确认了 2 颗雷，暂不发车", "只确认了 2 颗雷，暂不发车",
                            "W:桥洞情报可疑"]


# ------------------------------------------------------------------ 诊断规则
def _events(tmp_path: Path, records, session=None) -> list:
    """把 (事件名, 字段dict) 列表写成日志再读回来（顺带验证读回路径）。"""
    bb = BlackBox("car_node", log_dir=str(tmp_path), session="s")
    bb.log(EV.SESSION_BEGIN, start_cell="N10", goal_cell="N11", **(session or {}))
    for ev, fields in records:
        bb.log(ev, **fields)
    bb.close()
    events, bad = read_events(tmp_path)
    assert bad == 0
    return events


def _codes(findings) -> set:
    return {f.code for f in findings}


def test_diagnose_empty_log_is_critical():
    findings = diagnose([])
    assert _codes(findings) == {"no-events"}
    assert findings[0].severity == "critical"


def test_diagnose_missing_intel(tmp_path):
    events = _events(tmp_path, [(EV.ROUTE_BLOCKED, {})])
    m = summarize(events)
    assert m["intel_recv"] == 0
    assert "no-intel" in _codes(diagnose(events))


def test_diagnose_stable_but_no_route_lists_reason(tmp_path):
    events = _events(tmp_path, [
        (EV.INTEL_STABLE, {"mines": ["C3"]}),
        (EV.ROUTE_BLOCKED, {"reason": REASON.FACE_UNKNOWN, "detail": "face=None"}),
    ])
    findings = diagnose(events)
    codes = _codes(findings)
    assert "no-route" in codes
    f = next(f for f in findings if f.code == "no-route")
    assert "face_unknown" in f.detail          # 原因必须出现在结论里
    assert f.hint                              # 且必须给出下一步动作


def test_diagnose_repeated_route_and_missing_reset(tmp_path):
    events = _events(tmp_path, [
        (EV.INTEL_STABLE, {"mines": ["C3"]}),
        (EV.ROUTE_PUBLISH, {"kind": "route", "cmd": "A", "shoot_stops": ["D2"]}),
        (EV.ROUTE_PUBLISH, {"kind": "route", "cmd": "B", "shoot_stops": ["D2"]}),
    ])
    codes = _codes(diagnose(events))
    assert "route-repeated" in codes      # 车端会从头重跑
    assert "no-reset" in codes            # 里程计没复位


def test_diagnose_aim_guard_blocking_enemy_is_critical(tmp_path):
    events = _events(tmp_path, [
        (EV.AIM_BLOCK, {"cls": 1, "why": AIM_BLOCK_WHY.NOT_ALLOWED}),
    ])
    findings = diagnose(events)
    f = next(f for f in findings if f.code == "aim-blocks-enemy")
    assert f.severity == "critical"


def test_diagnose_aim_guard_blocking_friend_is_ok(tmp_path):
    events = _events(tmp_path, [
        (EV.AIM_BLOCK, {"cls": 2, "why": AIM_BLOCK_WHY.NOT_ALLOWED}),
        (EV.AIM_PASS, {"cls": 1}),
    ])
    f = next(f for f in diagnose(events) if f.code == "aim-blocks-friend")
    assert f.severity == "ok"


def test_diagnose_reports_stall(tmp_path):
    bb = BlackBox("car_node", log_dir=str(tmp_path), session="s")
    bb.log(EV.SESSION_BEGIN, start_cell="N10")
    bb.log(EV.ROUTE_PUBLISH, kind="route", cmd="x")
    bb.close()
    # 手工把两条记录的时间拉开，模拟"发完车就没动静了"
    f = next(tmp_path.rglob("*.jsonl"))
    recs = [json.loads(x) for x in f.read_text(encoding="utf-8").splitlines()]
    recs[1]["ts"] = recs[0]["ts"] + 90.0
    f.write_text("\n".join(json.dumps(r, ensure_ascii=False) for r in recs) + "\n",
                 encoding="utf-8")

    events, _ = read_events(tmp_path)
    findings = diagnose(events, max_gap_s=30.0)
    assert "stall" in _codes(findings)
    assert summarize(events)["max_gap_s"] == pytest.approx(90.0, abs=0.01)


def test_diagnose_dry_run_does_not_report_missing_route(tmp_path):
    """dry_run 下"没发路线"是**预期行为**，报成严重就是误报。

    这条是真机踩出来的：车端干跑一圈，报告第一行就是"情报已收敛，但没有发出任何路线"，
    而实际一切正常 —— 误报会让人不再信任报告。
    """
    events = _events(tmp_path, [(EV.INTEL_STABLE, {"mines": ["C3"]})],
                     session={"dry_run": True})
    codes = _codes(diagnose(events))
    assert "no-route" not in codes
    assert "dry-run" in codes


def test_diagnose_no_referee_separates_absent_from_silent(tmp_path):
    """要分清"裁判节点没起"（联调常态）与"起了但读不到数据"（真问题）。"""
    absent = _events(tmp_path / "a", [(EV.INTEL_RECV, {"seq": 1})],
                     session={"dry_run": True})
    f = next(x for x in diagnose(absent) if x.code == "no-referee")
    assert f.severity == "info"

    # 同一目录里出现 referee_node 的痕迹 = 它跑了，但一条状态都没产出
    bb = BlackBox("referee_node", log_dir=str(tmp_path / "a"), session="s")
    bb.log(EV.NODE_READY, pid=1)
    bb.close()
    silent, _ = read_events(tmp_path / "a")
    f2 = next(x for x in diagnose(silent) if x.code == "no-referee")
    assert f2.severity == "warn"


def test_summarize_counts_route_and_shots(tmp_path):
    events = _events(tmp_path, [
        (EV.INTEL_STABLE, {"mines": ["C3"]}),
        (EV.ROUTE_PUBLISH, {"kind": "reset", "cmd": "reset:N10,y"}),
        (EV.ROUTE_PUBLISH, {"kind": "route", "cmd": "A D2* N11,y", "shoot_stops": ["D2"]}),
        (EV.SHOOT_COUNT, {"value": 7}),
    ])
    m = summarize(events)
    assert m["reset_sent"] == 1 and m["route_sent"] == 1
    assert m["shoot_stops"] == ["D2"]
    assert m["shots"] == 7


# ------------------------------------------------------------------ 报告工具
def test_report_cli_prints_and_flags_critical(tmp_path, capsys):
    tool = _load_tool("log_report")
    events = [(EV.INTEL_STABLE, {"mines": ["C3"]})]   # 收敛了却没发车 -> critical
    _events(tmp_path, events)

    assert tool.main(["--dir", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "情报已收敛，但没有发出任何路线" in out

    assert tool.main(["--dir", str(tmp_path), "--strict"]) == 1
    capsys.readouterr()


def test_report_cli_json_is_machine_readable(tmp_path, capsys):
    tool = _load_tool("log_report")
    _events(tmp_path, [(EV.INTEL_RECV, {"seq": 1, "ok": True, "mines": ["C3"]})])

    assert tool.main(["--dir", str(tmp_path), "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["summary"]["intel_recv"] == 1
    assert isinstance(payload["findings"], list)
    assert payload["paths"]


def test_report_cli_missing_dir_is_actionable(tmp_path, capsys):
    tool = _load_tool("log_report")
    rc = tool.main(["--dir", str(tmp_path / "nope")])
    assert rc == 2
    assert "先跑一局" in capsys.readouterr().err


def test_report_markdown_contains_timeline_and_metrics(tmp_path):
    tool = _load_tool("log_report")
    events = _events(tmp_path, [
        (EV.INTEL_RECV, {"seq": 1, "ok": True, "mines": ["C3"]}),
        (EV.INTEL_STABLE, {"mines": ["C3"], "target_face": "blue"}),
        (EV.ROUTE_PUBLISH, {"kind": "route", "cmd": "C3 D2* N11,y", "shoot_stops": ["D2"]}),
    ])
    md = tool.render_markdown(sorted(tmp_path.rglob("*.jsonl")), events, 0, 30.0, 300)
    assert "## 一、结论" in md
    assert "## 三、时间轴" in md
    assert "D2* N11,y" in md            # 路线原文要能直接看到
    assert "intel.stable" in md


# ------------------------------------------------------------------ 节点接线
def _car_node(tmp_path: Path, monkeypatch, **kw):
    """在隔离的日志目录里用 ros_stub 实例化真实的 car_node。"""
    import ros_stub

    from ag_car.car_node import CarNode
    from ag_interfaces.msg import FieldIntelMsg

    monkeypatch.setenv("AG_LOG_DIR", str(tmp_path))
    monkeypatch.setenv("AG_LOG_SESSION", "t")
    ros_stub.set_overrides(**kw)
    try:
        node = CarNode()
    finally:
        ros_stub.clear_overrides()

    def feed(seq, mines=("C3", "G8"), face="blue"):
        node.deliver("/link/field_intel", FieldIntelMsg(
            mines=list(mines), obstacles=["E5"], bridge_a="K4", bridge_b="K9",
            target_cells=["D2", "E2"], target_face=face, seq=seq))

    return node, feed


def test_car_node_writes_session_and_intel_flow(tmp_path, monkeypatch):
    node, feed = _car_node(tmp_path, monkeypatch, min_votes=2)
    for i in (1, 2, 3, 4):
        feed(i)
    node.bb.close()

    events, bad = read_events(tmp_path)
    assert bad == 0
    kinds = [e.ev for e in events]

    assert kinds.count(EV.SESSION_BEGIN) == 1     # 一局的开头必须有据可查
    assert kinds.count(EV.INTEL_RECV) == 4        # 每一帧都要留证据（投票要靠它复盘）
    assert kinds.count(EV.INTEL_STABLE) == 1      # 内容没变就不重复刷
    assert kinds.count(EV.PLAN_BUILT) == 1        # 路线没变就不重复算
    assert EV.ROUTE_PUBLISH not in kinds          # dry_run 下一条命令都不许发

    session = next(e for e in events if e.ev == EV.SESSION_BEGIN)
    assert session.get("dry_run") is True
    assert session.get("start_cell") == "N10"
    assert session.get("min_mines") == 1


def test_car_node_runs_offline_even_if_log_dir_is_broken(tmp_path, monkeypatch):
    """日志写不进去时，节点必须照常算路线（日志绝不能拖垮主流程）。"""
    broken = tmp_path / "a_file"
    broken.write_text("x", encoding="utf-8")
    node, feed = _car_node(tmp_path, monkeypatch, min_votes=2)
    node.bb.set_target(log_dir=str(broken), session="t")

    feed(1)
    feed(2)                                        # 触发 replan

    assert node.bb.enabled is False
    assert node.bb.errors >= 1
    assert node.bb.path is None


# ------------------------------------------------------------------ 相机断流
def _vision(tmp_path: Path, monkeypatch, **kw):
    """在隔离的日志目录里用 ros_stub 实例化真实的 vision_node。"""
    import ros_stub

    from ag_perception.vision_node import VisionNode
    monkeypatch.setenv("AG_LOG_DIR", str(tmp_path))
    monkeypatch.setenv("AG_LOG_SESSION", "t")
    ros_stub.set_overrides(**kw)
    try:
        return VisionNode()
    finally:
        ros_stub.clear_overrides()


def test_vision_watchdog_reports_when_camera_is_silent(tmp_path, monkeypatch):
    """官方 usb_cam 崩了以后链路是静默的 —— 必须自己把"没图像"报出来。

    真实场景：usb_cam 以 SIGABRT 崩掉，det_node 只会无限刷 waiting for /image_raw，
    我们这边也安静得不发一帧检测。现场表现就是"什么都检测不到"，
    而真正的原因在链路上游三跳之外。
    """
    import time

    node = _vision(tmp_path, monkeypatch, max_image_gap_s=2.0, watchdog_period_s=0.5)
    node._t0 = time.monotonic() - 10.0      # 假装已经等了 10 秒还没图
    node.tick()                             # 触发看门狗定时器
    node.bb.close()

    events, _ = read_events(tmp_path)
    stale = [e for e in events if e.ev == EV.VISION_STALE]
    assert stale, "相机断流必须有事件，否则现场只能看到'什么都检测不到'"
    assert stale[-1].get("never") is True
    assert stale[-1].get("gap_s") >= 10.0


def test_vision_watchdog_recovers_when_image_returns(tmp_path, monkeypatch):
    import time

    node = _vision(tmp_path, monkeypatch, max_image_gap_s=2.0, watchdog_period_s=0.5)
    node._t0 = time.monotonic() - 10.0
    node.tick()                             # 报断流
    node._last_img_t = time.monotonic()     # 图像回来了（只测看门狗逻辑，不走解码）
    node.tick()                             # 应报恢复
    node.bb.close()

    events, _ = read_events(tmp_path)
    stale = [e for e in events if e.ev == EV.VISION_STALE]
    assert len(stale) == 2
    assert stale[-1].get("note") == "图像恢复"
    assert stale[-1].get("gap_s", 99) < 1.0


def test_diagnose_no_image_is_critical(tmp_path):
    events = _events(tmp_path, [
        (EV.VISION_STALE, {"never": True, "gap_s": 12.0, "images": 0, "note": "从来没收到过图像"}),
    ])
    f = next(x for x in diagnose(events) if x.code == "no-image")
    assert f.severity == "critical"
    assert "usb_cam" in f.hint


def test_diagnose_vision_stale_is_warn(tmp_path):
    events = _events(tmp_path, [
        (EV.VISION_STALE, {"never": False, "gap_s": 9.0, "images": 300, "note": "相机断流"}),
    ])
    f = next(x for x in diagnose(events) if x.code == "vision-stale")
    assert f.severity == "warn"


# ------------------------------------------------------------------ 单应标定
def test_intel_fusion_reports_uncalibrated_homography(tmp_path, monkeypatch):
    """单应没标定必须显式报出来。

    不报的后果：`_cell_of` 对每个检测都返回空串 -> 机载端永远发不出情报，
    而复盘报告会把"车端没情报"归因为链路/UDP 断了 —— 方向全错（真机上够查半天）。
    """
    import ros_stub

    from ag_perception.intel_fusion_node import IntelFusionNode

    monkeypatch.setenv("AG_LOG_DIR", str(tmp_path))
    monkeypatch.setenv("AG_LOG_SESSION", "t")
    node = IntelFusionNode()          # 默认 homography = 全零（未标定）
    node.bb.close()

    events, _ = read_events(tmp_path)
    h = [e for e in events if e.ev == EV.HOMOGRAPHY]
    assert h, "单应未标定必须有事件"
    assert h[0].get("ok") is False
    assert h[0].get("stage") == "startup"


def test_intel_fusion_ok_when_homography_is_set(tmp_path, monkeypatch):
    import ros_stub

    from ag_perception.intel_fusion_node import IntelFusionNode

    monkeypatch.setenv("AG_LOG_DIR", str(tmp_path))
    monkeypatch.setenv("AG_LOG_SESSION", "t")
    ros_stub.set_overrides(homography=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    try:
        node = IntelFusionNode()
    finally:
        ros_stub.clear_overrides()
    node.bb.close()

    events, _ = read_events(tmp_path)
    h = [e for e in events if e.ev == EV.HOMOGRAPHY]
    assert h and h[0].get("ok") is True


def test_diagnose_no_homography_points_at_calibration_not_link(tmp_path):
    events = _events(tmp_path, [(EV.HOMOGRAPHY, {"ok": False, "stage": "startup"})])
    findings = diagnose(events)
    f = next(x for x in findings if x.code == "no-homography")
    assert f.severity == "critical"
    assert "calib_homography" in f.hint
    # 根因已知时，不该再把人引去查链路（no-intel 会被抑制）
    assert not [x for x in findings if x.code == "no-intel"]


def test_slicing_must_not_hide_nodes_started_earlier(tmp_path):
    """★ 切局不能把"比 car_node 先启动的节点"切掉。

    真车实测（2026-09-29）：`referee_node` 在 phase 1 就起来了，`car_node` 在 phase 4
    才起、由它写 `session.begin`。于是 `referee_node` 的 `node.ready` 落在"最后一局"
    窗口之外 —— 报告一口咬定"referee_node 没在跑"，而真相是
    **"在跑，但整局没收到裁判状态"**（后者才指得出下一步该查什么）。
    """
    bb = BlackBox("referee_node", log_dir=str(tmp_path), session="s")
    bb.log(EV.NODE_READY, pid=1)                 # 先启动
    bb.close()
    time.sleep(0.02)      # 拉开时间戳，否则两个文件的事件会交错、切局结果不确定
    bb2 = BlackBox("car_node", log_dir=str(tmp_path), session="s")
    bb2.log(EV.SESSION_BEGIN, start_cell="N10", goal_cell="N11")
    bb2.log(EV.INTEL_RECV, seq=1)
    bb2.close()

    events, bad = read_events(tmp_path)
    assert bad == 0
    sliced = events[session_start_index(events):]
    assert "referee_node" not in {e.node for e in sliced}      # 确实被切掉了

    # 只按窗口算 → 误判成"没在跑"（信息级）
    wrong = next(f for f in diagnose(sliced) if f.code == "no-referee")
    assert wrong.severity == "info"

    # 传全量节点集合 → 正确的"在跑但没数据"（注意级，带可行动提示）
    right = next(f for f in diagnose(sliced, nodes_all={"referee_node", "car_node"})
                 if f.code == "no-referee")
    assert right.severity == "warn"
    assert "/HpAndHitmsg" in right.hint


def test_describe_covers_every_event_without_crash():
    """时间轴对每个事件都要能写出一行话（新增事件忘了加分支也不该崩）。"""
    tool = _load_tool("log_report")
    for ev in [v for k, v in vars(EV).items() if not k.startswith("_")]:
        e = Event(ts=1.0, ev=ev, node="n", data={"msg": "m", "cmd": "c"})
        assert isinstance(tool.describe(e), str)
