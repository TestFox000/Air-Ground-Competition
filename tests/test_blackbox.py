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


def test_open_failure_leaves_path_none(tmp_path):
    """打开失败时 path 必须是 None，不能指向一个从未成功打开的文件。"""
    not_a_dir = tmp_path / "file"
    not_a_dir.write_text("x", encoding="utf-8")
    bb = BlackBox("n", log_dir=str(not_a_dir), session="s")
    assert bb.enabled is False
    assert bb.path is None


def test_flush_every_controls_when_records_hit_disk(tmp_path):
    """`flush_every=N` 累计 N 条才落盘 —— 行缓冲已去掉，这个参数不再被架空。

    默认 `flush_every=1` 仍是逐条 flush，"进程被杀不丢日志"的约束不变。
    """
    bb = BlackBox("n", log_dir=str(tmp_path), session="s", flush_every=3)
    bb.log(EV.INTEL_RECV, seq=1)
    bb.log(EV.INTEL_RECV, seq=2)
    f = next(tmp_path.rglob("*.jsonl"))
    assert f.read_text(encoding="utf-8") == ""       # 不足 3 条，还在用户态缓冲
    bb.log(EV.INTEL_RECV, seq=3)
    assert f.read_text(encoding="utf-8").count("\n") == 3
    bb.close()


def test_attach_twice_does_not_double_wrap(tmp_path):
    """重复 attach：logger 只套一层 _LoggerMirror（套两层会各记一遍、去重状态分裂）。"""
    from ag_core.blackbox import _LoggerMirror, attach

    class _Node:
        def __init__(self):
            self._logger = type("L", (), {"info": lambda self, m: None})()

        def get_logger(self):
            return self._logger

    node = _Node()
    attach(node, BlackBox("n", log_dir=str(tmp_path), session="s"))
    first = node.get_logger()
    assert isinstance(first, _LoggerMirror)
    attach(node, BlackBox("n", log_dir=str(tmp_path), session="s"))
    assert node.get_logger() is first                # 没有叠第二层


def test_set_target_resets_warned(tmp_path, capsys):
    """换目标后再次失败必须重新告警 —— _warned 跟着旧目标走会吞掉唯一一次提示。"""
    bad1 = tmp_path / "f1"
    bad1.write_text("x", encoding="utf-8")
    bb = BlackBox("n", log_dir=str(bad1), session="s")
    assert bb.errors >= 1
    assert "[blackbox]" in capsys.readouterr().err   # 第一次告警

    bb.enabled = True                                # 测试场景显式复活
    bb.set_target(log_dir=str(tmp_path / "good"), session="s")
    assert bb.path is not None
    capsys.readouterr()

    bad2 = tmp_path / "f2"
    bad2.write_text("x", encoding="utf-8")
    bb.set_target(log_dir=str(bad2), session="s")    # 再坏：要重新告警
    assert bb.enabled is False
    assert "[blackbox]" in capsys.readouterr().err


def test_find_log_dirs_only_lists_dirs_with_files(tmp_path):
    """同一批文件不再让 root 与各级子目录重复上榜：只有直接含 .jsonl 的目录出现。"""
    from ag_core.blackbox import find_log_dirs
    BlackBox("a", log_dir=str(tmp_path), session="s1").close()
    dirs = find_log_dirs(tmp_path)
    assert dirs == [tmp_path / "s1"]                 # root 没有直属文件，不上榜


def test_read_events_skip_bad_false_raises(tmp_path):
    """skip_bad=False 是校验语义：坏行直接抛 ValueError，而不是计数后当没发生。"""
    bb = BlackBox("n", log_dir=str(tmp_path), session="s")
    bb.log(EV.INTEL_RECV, seq=1)
    bb.close()
    f = next(tmp_path.rglob("*.jsonl"))
    with open(f, "a", encoding="utf-8") as fh:
        fh.write("{不是 json}\n")

    events, bad = read_events(tmp_path)              # 默认 skip_bad=True：计数不抛
    assert bad == 1 and len(events) == 1
    with pytest.raises(ValueError):
        read_events(tmp_path, skip_bad=False)


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
    """`ev` 是 log() 的位置形参，payload 根本传不进来 —— 记录里永远是框架写的值。

    顺带锁定另一条边界：业务payload若撞上框架保留键（如 lvl），
    会被挪进 data 子对象，事件等级不被劫持（否则按等级过滤/统计全会错）。
    """
    bb = BlackBox("n", log_dir=str(tmp_path), session="s")
    bb.log(EV.AIM_PASS, level="warn", cls=1, lvl="业务字段")
    bb.close()

    r = _lines(tmp_path)[0]
    assert r["ev"] == EV.AIM_PASS       # 框架字段由位置参数写入，不受 payload 影响
    assert r["lvl"] == "warn"           # 事件等级由 level= 参数写
    assert r["data"] == {"lvl": "业务字段"}   # 撞名的业务值被挪进 data，不覆盖框架 lvl
    assert r["cls"] == 1


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
    """把 (事件名, 字段dict) 列表写成日志再读回来（顺带验证读回路径）。

    节点名与 session.begin 字段对齐现役 `integrate_node` 的 `_blog` 调用点
    （start/goal/dry_run/publish_reset）——用旧事件流喂规则，
    规则改错了也抓不到（本文件曾经就这样盲了 147 个测试）。
    """
    bb = BlackBox("integrate_node", log_dir=str(tmp_path), session="s")
    bb.log(EV.SESSION_BEGIN, start="N10", goal="N11", **(session or {}))
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
    events = _events(tmp_path, [(EV.ROUTE_BLOCKED, {"reason": REASON.MIN_MINES})])
    m = summarize(events)
    assert m["intel_recv"] == 0
    assert "no-intel" in _codes(diagnose(events))


def test_diagnose_intel_but_blocked_lists_reason(tmp_path):
    """情报在收、门控评估过，但路线一直没发：报告要点名被拦原因。"""
    events = _events(tmp_path, [
        (EV.INTEL_RECV, {"n": 1, "mines": 1}),
        (EV.ROUTE_BLOCKED, {"reason": REASON.FACE_UNKNOWN}),
    ])
    findings = diagnose(events)
    codes = _codes(findings)
    assert "no-route" in codes
    f = next(f for f in findings if f.code == "no-route")
    assert "face_unknown" in f.detail          # 原因必须出现在结论里
    assert f.hint                              # 且必须给出下一步动作


def test_diagnose_intel_no_decision_is_critical(tmp_path):
    """情报在收，但 blocked/plan/publish 一个都没有 —— 收包到决策之间断了。

    （旧规则盯 intel.stable：其生产者 IntelVoter 已随旧栈删除，条件恒真、
    每局必误报；现役语义看决策链有没有任何输出。）
    """
    events = _events(tmp_path, [(EV.INTEL_RECV, {"n": 2, "mines": 0})])
    findings = diagnose(events)
    f = next(f for f in findings if f.code == "intel-no-decision")
    assert f.severity == "critical"


def test_diagnose_healthy_session_has_no_critical(tmp_path):
    """端到端：健康局事件流（现役词汇）跑 diagnose，零 critical 误报。

    这是本文件的定盘星：规则收敛到现役语义后，一局正常跑车
    （收情报 -> 门控放行 -> plan.built -> 发 reset+route）不该报任何严重。
    """
    events = _events(tmp_path, [
        (EV.NODE_ARGS, {"dry_run": False, "start": "N10", "goal": "N11"}),
        (EV.INTEL_RECV, {"n": 3, "mines": 1}),
        (EV.PLAN_BUILT, {"reset": "reset:N10,y", "route": "D2 E5* N11,x",
                         "shoot": "E5", "heading": "y"}),
        (EV.ROUTE_PUBLISH, {"kind": "reset", "cmd": "reset:N10,y"}),
        (EV.ROUTE_PUBLISH, {"kind": "route", "cmd": "D2 E5* N11,x"}),
    ])
    m = summarize(events)
    assert m["reset_sent"] == 1 and m["route_sent"] == 1
    assert m["shoot_stops"] == ["E5"]          # plan.built 的逗号串 shoot= 也要认
    findings = diagnose(events)
    criticals = [f for f in findings if f.severity == "critical"]
    assert criticals == [], [f"{f.code}: {f.title}" for f in criticals]


def test_diagnose_repeated_route_and_missing_reset(tmp_path):
    events = _events(tmp_path, [
        (EV.INTEL_RECV, {"n": 1, "mines": 1}),
        (EV.PLAN_BUILT, {"route": "A"}),
        (EV.ROUTE_PUBLISH, {"kind": "route", "cmd": "A", "shoot_stops": ["D2"]}),
        (EV.ROUTE_PUBLISH, {"kind": "route", "cmd": "B", "shoot_stops": ["D2"]}),
    ])
    codes = _codes(diagnose(events))
    assert "route-repeated" in codes      # 车端会从头重跑
    assert "no-reset" in codes            # 里程计没复位


def test_diagnose_no_reset_respects_publish_reset_flag(tmp_path):
    """publish_reset=false 是"由人手动 reset"的合法配置：只能报 info，不能报 critical。

    标志从 session.begin 读（现役 integrate_node 开局就写）。
    """
    events = _events(tmp_path, [
        (EV.INTEL_RECV, {"n": 1, "mines": 1}),
        (EV.PLAN_BUILT, {"route": "A"}),
        (EV.ROUTE_PUBLISH, {"kind": "route", "cmd": "A"}),
    ], session={"publish_reset": False})
    f = next(f for f in diagnose(events) if f.code == "no-reset")
    assert f.severity == "info"


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
    bb = BlackBox("integrate_node", log_dir=str(tmp_path), session="s")
    bb.log(EV.SESSION_BEGIN, start="N10", goal="N11")
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
    events = _events(tmp_path, [(EV.INTEL_RECV, {"n": 1, "mines": 1}),
                                (EV.ROUTE_BLOCKED, {"reason": REASON.MIN_MINES})],
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
        (EV.INTEL_RECV, {"n": 1, "mines": 1}),
        (EV.PLAN_BUILT, {"route": "A D2* N11,y", "shoot": "D2"}),
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
    # 情报在收、门控拦下、路线没发 -> no-route critical
    events = [(EV.INTEL_RECV, {"n": 1, "mines": 1}),
              (EV.ROUTE_BLOCKED, {"reason": REASON.MIN_MINES})]
    _events(tmp_path, events)

    assert tool.main(["--dir", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "情报到了、门控也评估过，但没有发出任何路线" in out

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
        (EV.INTEL_RECV, {"n": 1, "ts": 1, "mines": 1}),
        (EV.PLAN_BUILT, {"reset": "reset:N10,y", "route": "C3 D2* N11,y",
                         "shoot": "D2", "heading": "y"}),
        (EV.ROUTE_PUBLISH, {"kind": "reset", "cmd": "reset:N10,y"}),
        (EV.ROUTE_PUBLISH, {"kind": "route", "cmd": "C3 D2* N11,y",
                            "shoot_stops": ["D2"]}),
    ])
    md = tool.render_markdown(sorted(tmp_path.rglob("*.jsonl")), events, 0, 30.0, 300)
    assert "## 一、结论" in md
    assert "## 三、时间轴" in md
    assert "D2* N11,y" in md            # 路线原文要能直接看到
    assert "plan.built" in md
