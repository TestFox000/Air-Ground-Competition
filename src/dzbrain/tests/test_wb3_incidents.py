# -*- coding: utf-8 -*-
r"""WB-3 案例场景库验收（卡 `WB-3-INCIDENTS-20261031`）。

    PYTHONPATH=src/dzbrain python src/dzbrain/tests/test_wb3_incidents.py

## 这张卡要证的四件事（每件都有机器判据）

| # | 要证 | 本文件小节 |
|---|---|---|
| ① | 夹具**真的在测注入**，不是在测别的 | §2 正例判据全绿 + §3 负例判据全红（**逐条**，不是"整体"） |
| ② | 事件注入 API 四类可用 | §4 yaw-jump / drop-s / pose-step / scale-step / latency-step |
| ③ | 台架里**没有第二份投影/推进** | §1 源码级守卫（本卡写域也扫） |
| ④ | 无人机步进 stub 能驱动真 StepLoop 闭环 | §5 stub ↔ `DroneTask` ↔ `StepLoop` |

## 为什么负例要**逐条**判，而不是只判"整体没复现"

一条夹具通常有 3~4 条判据。如果负例只判「整体 symptom_seen=False」，那么只要
**其中一条**在负例里碰巧也不成立，整条夹具就算"负例通过" —— 另外两条从来没被
反证过，它们在测什么就没人知道。本文件逐条比对：**正例里每一条都绿，负例里每一条都红**。

★ 只有一类判据例外，**`negative.unchanged` 里显式列出的**：那些是**不变式**而不是症状
（例如 I11 的 `finished`：注入一帧不该把一次能跑完的行程变成跑不完），负例里必须**仍然绿**。
把它们标出来而不是假装它们也是症状，是这张卡最容易放水的地方。
"""

from __future__ import annotations

import json
import os
import pathlib
import re
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve()
#: `tests/` → `src/dzbrain` → `src` → `dzrobot_v2`（三级，别写成 parents[2]，
#: 那一个是 `src`；写错了 `tools/` 就进不了 sys.path，报「No module named wb_events」
REPO = HERE.parents[3]                          # dzrobot_v2
TOOLS = REPO / "tools"                          # dzrobot_v2/tools
SRC = REPO / "src" / "dzbrain"                  # dzrobot_v2/src/dzbrain
sys.path.insert(0, str(SRC))
sys.path.insert(0, str(HERE.parent))
sys.path.append(str(TOOLS))

from harness import check, finish, section       # noqa: E402

import wb_events as W                            # noqa: E402
import wb_drone_stub as DS                       # noqa: E402

#: 本卡写域（源码级守卫只扫这些文件；不许扩大到全仓 —— 存量平行实现会让判据永远红）
WRITE_SCOPE: tuple[str, ...] = (
    "tools/wb_events.py",
    "tools/wb_drone_stub.py",
    "src/dzbrain/tests/test_wb3_incidents.py",
)

#: 禁写的**自写实现**签名（与 wb-2 同一张清单，本卡也照此自查）
FORBIDDEN_DEFS: tuple[str, ...] = (
    "def project", "def _project", "def _project_sticky", "def _project_with_floor",
    "def _clamp", "def _clamp_forward", "def _advance", "def _advance_anchor",
    "def step_bicycle", "def _bicycle_step", "def _integrate",
    "def normalize_field_layout", "def plan_route",
)


def _strip_comments_and_docstrings(text: str) -> str:
    """去掉注释与**字符串字面量**，只留真代码。

    ★ 为什么字符串也要去：本文件**自己**就在 `FORBIDDEN_DEFS` 这个元组里把禁写签名
      当数据写着（`"def project"`, `"def _clamp"`, …）。只去注释的话，扫描本文件就会
      命中自己那张名单 —— 判据把「判据的定义」当成了违规，这正是"判据把文档写成敌人"
      的同一类错误（wb-2 那条判据也栽过，它的注释里写明必须去字面量）。
    """
    text = re.sub(r'"""[\s\S]*?"""', "", text)
    text = re.sub(r"'''[\s\S]*?'''", "", text)
    text = re.sub(r'r?"([^"\\\n]|\\.)*"', '""', text)          # 单行双引号
    text = re.sub(r"r?'([^'\\\n]|\\.)*'", "''", text)          # 单行单引号
    out: list[str] = []
    for line in text.splitlines():
        if line.lstrip().startswith("#"):
            continue
        out.append(line.split("  #", 1)[0] if "  #" in line else line)
    return "\n".join(out)


def _verify_scope_files_exist() -> None:
    for rel in WRITE_SCOPE:
        path = REPO / rel
        check(f"写域文件存在 {rel}", path.is_file(), str(path))


# --------------------------------------------------------------------------- #
#  §1 判据① 台架内不得有第二份投影/推进（本卡写域）
# --------------------------------------------------------------------------- #
def sec1_no_second_projection() -> None:
    section("1. 判据①：本卡写域内不得出现第二份投影/推进实现")
    _verify_scope_files_exist()
    for rel in WRITE_SCOPE:
        text = _strip_comments_and_docstrings((REPO / rel).read_text(encoding="utf-8"))
        hits = [sig for sig in FORBIDDEN_DEFS if sig in text]
        check(f"{rel} 无自写实现 {hits or '（无命中）'}", not hits, str(hits))
    # ---- 引用级：事件 API 必须**调用**生产件，而不是自己造 ----
    src = (TOOLS / "wb_events.py").read_text(encoding="utf-8")
    check("wb_events 调的是 sim_rig.run（台架唯一编排入口）", "sim_rig.run(" in src)
    check("wb_events 继承生产 World（不是新建一个积分器）",
          "from sim_world import World" in src and "class _EventWorld(World)" in src)
    check("wb_events 继承生产 Actuator（不是新建执行链）",
          "class _EventActuator(Actuator)" in src)
    check("wb_events 的投影走项目生产 Path（不写第二份）",
          "path.project" not in src and "_project" not in _strip_comments_and_docstrings(src))
    # ---- 反证：同一套正则能在 tier-0 sim.py 上命中（证明它不是永不命中的空判据）----
    sim_text = _strip_comments_and_docstrings(
        (SRC / "dzbrain" / "tools" / "sim.py").read_text(encoding="utf-8"))
    tier0_hits = [sig for sig in FORBIDDEN_DEFS if sig in sim_text]
    check("反证：同一套签名在 tier-0 sim.py 上确实命中（判据不是空转）",
          len(tier0_hits) >= 3, f"tier-0 命中 {tier0_hits}")
    # ---- 负例：往写域里塞一行 `def project(` ⇒ 上面那几条必须红 ----
    probe = "def project(x, y):\n    return x\n"
    hits = [sig for sig in FORBIDDEN_DEFS if sig in _strip_comments_and_docstrings(probe)]
    check("负例：植入 `def project(` 立刻被判据抓到", hits == ["def project"], str(hits))


# --------------------------------------------------------------------------- #
#  §2/§3 夹具：正例全绿 + 负例逐条全红
# --------------------------------------------------------------------------- #
def _run(inc, *, on: bool, patch=None, scen=None, arm=None) -> dict:
    """跑一次夹具。`arm` 缺省用夹具自己写的档（负例可以换档，见各夹具 `negative.arm`）。"""
    return W.run_incident(inc, arm_name=arm or inc.arm, events_on=on, quiet=True,
                          patch=patch, scenario_name=scen, seed=20261031)


def _neg_args(inc) -> tuple:
    neg = inc.negative or {}
    return (dict(neg.get("arm_patch") or {}) or None, neg.get("scenario"),
            neg.get("arm") or inc.arm)


def sec2_fixtures() -> int:
    section("2. 案例夹具：正例判据逐条全绿（symptom_seen=True）")
    slugs = W.registered_slugs()
    print(f"已登记 {len(slugs)} 条：{slugs}")
    check("夹具条数 ≥ 8（卡面硬判据）", len(slugs) >= 8, f"实得 {len(slugs)}")
    check("夹具 id 唯一", len({W.load(s).id for s in slugs}) == len(slugs))
    n_pos = 0
    for slug in slugs:
        inc = W.load(slug)
        section(f"2.{inc.id} {inc.title}")
        print(f"   症状：{inc.symptom}")
        print(f"   根因：{inc.root_cause}")
        print(f"   出处：{'；'.join(inc.sources)}")
        # ---- 出处必须给（卡面：无出处不许入库） ----
        check(f"{inc.id} 有出处（≥1 条 `文件:行` 或文档章节）", len(inc.sources) >= 1,
              str(inc.sources))
        check(f"{inc.id} 有机器判据（≥1 条）", len(inc.criteria) >= 1)
        pos = _run(inc, on=True)
        check(f"{inc.id} 正例无任务 FAULT", not pos["faulty"], str(pos["faulty"]))
        for c in pos["criteria"]:
            check(f"{inc.id} 正例判据绿「{c['name']}」"
                  f"（{c['metric']} {c['op']} {c['value']} → 实得 {c['actual']}）",
                  c["pass"], c.get("why", ""))
        check(f"{inc.id} 正例 symptom_seen=True", pos["reproduced"])
        if pos["reproduced"]:
            n_pos += 1
        # ★ 事件类夹具：事件必须**真的按点生效**，否则"症状出现"可能是别的原因
        if inc.events:
            check(f"{inc.id} 注入的事件全部按点生效（{len(inc.events)} 条）",
                  len(pos["fired"]) == len(inc.events),
                  f"fired={pos['fired']}")
    check(f"★ 可复现的夹具条数 ≥ 8（硬判据），实得 {n_pos}", n_pos >= 8, f"{n_pos}/{len(slugs)}")
    return n_pos


def sec3_negatives() -> None:
    section("3. 负例：把注入/开关/场景改掉后，**每一条症状判据**都必须消失")
    n_neg = 0
    for slug in W.registered_slugs():
        inc = W.load(slug)
        neg = inc.negative or {}
        if not neg:
            check(f"{inc.id} 声明了负例", False, "夹具没写 negative 段 ⇒ 无法证明它在测注入")
            continue
        patch, scen, narm = _neg_args(inc)
        section(f"3.{inc.id} {neg.get('desc', '')}")
        ng = _run(inc, on=bool(neg.get("events_on", False)), patch=patch, scen=scen,
                  arm=narm)
        check(f"{inc.id} 负例无任务 FAULT", not ng["faulty"], str(ng["faulty"]))
        check(f"{inc.id} 负例实际换了档/开关（不是原样重跑）",
              (narm != inc.arm) or bool(patch) or bool(scen)
              or not bool(neg.get("events_on", False)),
              f"arm {inc.arm}→{narm} patch={patch} scenario={scen} "
              f"events_on={neg.get('events_on')}")
        unchanged = tuple(str(x) for x in (neg.get("unchanged") or ()))
        pos = _run(inc, on=True)
        pos_by_name = {c["name"]: c for c in pos["criteria"]}
        for c in ng["criteria"]:
            name = c["name"]
            if name in unchanged:
                check(f"{inc.id} 负例不变式仍绿「{name}」（不是症状，必须两边都绿）",
                      bool(c["pass"]), f"实得 pass={c['pass']} actual={c['actual']}")
            else:
                check(f"{inc.id} 负例判据**红**「{name}」"
                      f"（正例 {pos_by_name.get(name, {}).get('actual')} → "
                      f"负例 {c['actual']}）", not c["pass"], f"实得 pass={c['pass']}")
        flagged = [n for n in unchanged if n not in {c["name"] for c in ng["criteria"]}]
        check(f"{inc.id} negative.unchanged 里的名字都真实存在 {flagged or '（无）'}",
              not flagged, str(flagged))
        check(f"{inc.id} 负例整体 symptom_seen=False", not ng["reproduced"],
              f"实得 {ng['reproduced']}")
        if not ng["reproduced"]:
            n_neg += 1
    check(f"★ 负例逐条全红的夹具条数 = 全部，实得 {n_neg}", n_neg == len(W.registered_slugs()),
          f"{n_neg}/{len(W.registered_slugs())}")


def sec3b_sub_example_i11() -> None:
    section("3b. I11 的不变式口径（唯一一条「负例里必须仍绿」的判据）")
    inc = W.load("I11")
    neg = inc.negative or {}
    check("I11 显式声明了 unchanged 判据", bool(neg.get("unchanged")), str(neg))
    patch, scen, narm = _neg_args(inc)
    pos = _run(inc, on=True)
    ng = _run(inc, on=False, patch=patch, scen=scen, arm=narm)
    unch = tuple(str(x) for x in (neg.get("unchanged") or ()))
    for c in pos["criteria"]:
        if c["name"] in unch:
            check(f"I11 不变式「{c['name']}」正例绿", c["pass"], f"actual={c['actual']}")
    for c in ng["criteria"]:
        if c["name"] in unch:
            check(f"I11 不变式「{c['name']}」负例**仍**绿", c["pass"], f"actual={c['actual']}")


# --------------------------------------------------------------------------- #
#  §4 事件注入 API 四类
# --------------------------------------------------------------------------- #
def sec4_event_api() -> None:
    section("4. 事件注入 API：卡面四个入口 + 时间基 + 语法负例")

    # ---- 4.1 纯解析层（不跑仿真，快） ----
    shorthand = [("yaw-jump=35", "yaw-jump", 35.0), ("drop-s=1.5", "drop", 1.5),
                 ("pose-step=0.30", "pose-step", 0.30)]
    for text, kind, value in shorthand:
        ev = W.Event.parse(text)
        check(f"卡面简写 `{text}` → kind={kind} value={value} @{W.DEFAULT_AT_T:g}s",
              (ev.kind, ev.value, ev.at_t) == (kind, value, W.DEFAULT_AT_T))
        check(f"`{text}` 记为「未显式给 @t」", ev.had_at is False)
    for text, kind, value, at in [("yaw-jump=35@8.0", "yaw-jump", 35.0, 8.0),
                                  ("scale-step=0.45@5.0", "scale-step", 0.45, 5.0),
                                  ("latency-step=1.20@6.0", "latency-step", 1.20, 6.0),
                                  ("drop=3.0@60.0", "drop", 3.0, 60.0)]:
        ev = W.Event.parse(text)
        check(f"完整写法 `{text}`", (ev.kind, ev.value, ev.at_t) == (kind, value, at))
    # 负例：语法/类型错必须抛
    for bad in ("", "yaw-jump", "bogus=1@2", "yaw-jump=abc@2", "yaw-jump=35@-1",
                "drop=-1@2", "latency-step=-1@2"):
        try:
            W.Event.parse(bad)
            check(f"坏事件 {bad!r} 必须抛", False, "没抛 ⇒ 静默忽略 = 夹具可能什么都没测")
        except W.EventError:
            check(f"坏事件 {bad!r} 抛 EventError", True)
    # 负例：未知派生量 kind / 未知判据 op 必须红
    try:
        W._derive([], {"x": {"kind": "no_such_kind"}})
        check("未知派生量 kind 必须抛", False)
    except W.FixtureError:
        check("未知派生量 kind 抛 FixtureError", True)
    j = W._judge({"metric": "steps", "op": "no_such_op", "value": 1}, {"steps": 5.0})
    check("未知比较符判红（不是跳过）", not j["pass"] and "未登记" in j["why"], j["why"])
    j = W._judge({"metric": "no_such_metric", "op": "gt", "value": 1}, {"steps": 5.0})
    check("缺指标判红（不是跳过）", not j["pass"] and "缺失" in j["why"], j["why"])
    j = W._judge({"metric": "phase_final", "op": "gt", "value": "DRIVE"},
                 {"phase_final": "SAFE_STOP"})
    check("字符串指标用 gt 必须判红（字典序比较会给出无意义的通过）",
          not j["pass"] and "只允许 eq" in j["why"], j["why"])

    # ---- 4.2 时间基（这是第一版栽过的坑，必须有守卫）----
    section("4.2 时间基：虚拟时钟从 1000 s 起，事件按**相对秒**生效")
    drv = W.EventDriver(events=W.parse_events(["yaw-jump=35@8.0"]))
    at_0 = drv._take(1000.0, ("yaw-jump",))
    check("t=1000.0（相对 0 s）时事件**不**生效", at_0 == [], f"实得 {at_0}")
    at_8 = drv._take(1008.0, ("yaw-jump",))
    check("t=1008.0（相对 8 s）时事件生效", len(at_8) == 1, f"实得 {at_8}")
    check("t0 被钉死为 1000.0", drv.t0 == 1000.0, str(drv.t0))

    # ---- 4.3 三个卡面入口**真跑**（短时）----
    section("4.3 卡面三个入口真跑（--inject 简写，30 s）")
    for text, expect_phase_hint in (("yaw-jump=35", "守卫锁存"), ("drop-s=1.5", "定位丢样"),
                                    ("pose-step=0.30", "单帧突跳")):
        out = subprocess.run(
            [sys.executable, str(TOOLS / "wb_events.py"), "--inject", text,
             "--seconds", "30", "--json"],
            cwd=str(REPO), capture_output=True, text=True, encoding="utf-8",
            env={**os.environ, "PYTHONPATH": str(SRC), "PYTHONIOENCODING": "utf-8"})
        last = [ln for ln in out.stdout.splitlines() if ln.startswith(W.INCIDENT_PREFIX)]
        check(f"`--inject {text}` 退出码 0（{expect_phase_hint}）", out.returncode == 0,
              f"rc={out.returncode} stderr={out.stderr[-300:]}")
        check(f"`--inject {text}` 末行是 {W.INCIDENT_PREFIX} 契约行", bool(last),
              f"末行={out.stdout.splitlines()[-1][:120] if out.stdout else '（空）'}")
        if last:
            body = json.loads(last[-1][len(W.INCIDENT_PREFIX):].strip())
            check(f"`--inject {text}` 契约行 schema 正确", body["schema"] == "dzrobot-wb3-incident/1")
            check(f"`--inject {text}` 事件按点生效（events_fired 非空）",
                  bool(body["events_fired"]), str(body["events_fired"]))
            check(f"`--inject {text}` 症状判据绿（events_fired_n == 1）",
                  body["symptom_seen"], str(body["criteria"]))

    # ---- 4.4 负例：--no-events 关掉注入 ⇒ API 判据必须红 ----
    section("4.4 负例：--no-events 关掉注入 ⇒ 注入 API 判据必须红")
    out = subprocess.run(
        [sys.executable, str(TOOLS / "wb_events.py"), "--inject", "yaw-jump=35",
         "--seconds", "30", "--no-events", "--json"],
        cwd=str(REPO), capture_output=True, text=True, encoding="utf-8",
        env={**os.environ, "PYTHONPATH": str(SRC), "PYTHONIOENCODING": "utf-8"})
    last = [ln for ln in out.stdout.splitlines() if ln.startswith(W.INCIDENT_PREFIX)]
    check("`--no-events` 退出码 != 0（判据红）", out.returncode != 0, f"rc={out.returncode}")
    if last:
        body = json.loads(last[-1][len(W.INCIDENT_PREFIX):].strip())
        check("`--no-events` 契约行 symptom_seen=False", not body["symptom_seen"])
        check("`--no-events` events_fired 为空", not body["events_fired"],
              str(body["events_fired"]))
        check("`--no-events` events_fired_n 判据是红的",
              not any(c["pass"] for c in body["criteria"]), str(body["criteria"]))


# --------------------------------------------------------------------------- #
#  §5 无人机步进 stub → 真 DroneTask → 真 StepLoop 闭环
# --------------------------------------------------------------------------- #
SEGS = [["N10", "M10", "M9", "M8", "K7"], ["M7", "L7", "J7", "J8", "K5"]]


def _drive(pacing: str, *, cutoff: int = 1, t_end: float = 40.0, period: float = 8.0):
    """stub 喂真 `DroneTask`，返回 (drone, bb, stub, 已见过的 segment_id 集合)。"""
    import fakes
    import wb_rig.sched as SCH
    from dzbrain.core.blackboard import P_FIELD_LAYOUT, Blackboard
    from dzbrain.core.config import Config
    from dzbrain.core.events import EventBus
    from dzbrain.tasks.drone import DroneTask

    clock = SCH.VirtualClock()
    bridge = fakes.FakeBridge(clock)
    bb, bus = Blackboard(), EventBus()
    cfg = Config.from_dict({"drone": {"enabled": True, "heartbeat_timeout_s": 3.0,
                                      "coverage_enabled": False,
                                      "field_verify_policy": "record"},
                            "logging": {"rate_limit_s": 0.0}})
    drone = DroneTask(bridge, bb, bus, cfg, log=lambda m: None, clock=clock.now)
    drone.setup()
    stub = DS.DroneStub(segments=SEGS, mines=["A2"], pacing=pacing,
                        segment_period_s=period, cutoff_after_segments=cutoff)
    last_t = None
    seen: set[int] = set()
    for _ in range(int(t_end / 0.05)):
        now = clock.now()
        DS.pump(stub, bridge, now - clock.t0, last_t=last_t)
        last_t = now - clock.t0
        clock.advance(0.05)
        drone.tick()
        lay = bb.get(P_FIELD_LAYOUT)
        if isinstance(lay, dict):
            seen.add(int(lay.get("segment_id") or 0))
    drone.teardown()
    return drone, bb, stub, seen


def sec5_drone_stub() -> None:
    section("5. 无人机步进 stub：真 DroneTask 收报 → 校验 → 落黑盒（闭环所需的最小件）")
    from dzbrain.core.blackboard import P_FIELD_LAYOUT
    from dzbrain.tasks.drone import normalize_field_layout

    # ---- 5.1 三种节奏都收进去，且**零拒收** ----
    for pacing in DS.PACINGS:
        drone, bb, stub, seen = _drive(pacing, cutoff=1)
        check(f"{pacing}：DroneTask 收到并被接受 {drone.layout_updates} 份",
              drone.layout_updates >= 1, str(drone.layout_updates))
        check(f"{pacing}：**零拒收**（拒收 = 我的发报机不合协议）", drone.layout_rejects == 0,
              f"rejects={drone.layout_rejects} stale={drone.layout_stale}")
        check(f"{pacing}：报文落进 P_FIELD_LAYOUT",
              isinstance(bb.get(P_FIELD_LAYOUT), dict))

    # ---- 5.2 逐段给 vs 一次给全：known 只增、终点相同 ----
    d_all, bb_all, s_all, _ = _drive("all-at-once", t_end=20.0)
    d_seg, bb_seg, s_seg, seen_seg = _drive("per-segment", t_end=40.0, period=8.0)
    kn_all = tuple(bb_all.get(P_FIELD_LAYOUT).get("known") or ())
    kn_seg = tuple(bb_seg.get(P_FIELD_LAYOUT).get("known") or ())
    check("一次给全与逐段给，最终 known 相同（节奏只影响时序不影响内容）",
          kn_all == kn_seg, f"all={len(kn_all)} seg={len(kn_seg)}")
    check("逐段给确实经过多个 segment_id（不是一步到底）", len(seen_seg) >= 2,
          str(sorted(seen_seg)))

    # ---- 5.3 断供：车端**不该无限等**（B 区裁决），stub 之后不再发 ----
    d_cut, bb_cut, s_cut, seen_cut = _drive("cutoff", cutoff=0, t_end=40.0)
    check("cutoff：只给到第 1 段就停（known 不再增长）",
          len(bb_cut.get(P_FIELD_LAYOUT).get("known") or ()) <= len(SEGS[0]),
          str(len(bb_cut.get(P_FIELD_LAYOUT).get("known") or ())))
    check("cutoff：stub 自己标记 stalled=True", s_cut.stalled)
    after = s_cut.messages_at(80.0, last_t=79.0)
    check("cutoff：断供之后 stub 不再发任何报文（不会自己恢复）", after == [], str(after))

    # ---- 5.4 协议硬约束：特征 ⊆ known（v2 的整份拒收条件）----
    section("5.4 协议硬约束：v2 要求 特征 ⊆ known（不满足则整份拒收）")
    bad = DS.DroneStub(segments=[["N10", "M10"]], mines=["A2"], pacing="per-segment")
    lay0 = bad.layout_at(0.0)
    lay0["mines"] = ["A2"]                        # 故意报一个没观测到的雷格
    ok, reason = normalize_field_layout(lay0)
    check("反证：把未观测的雷塞进报文 ⇒ 车端整份拒收", ok is None, str(reason))
    good = DS.DroneStub(segments=[["N10", "M10", "A2"]], mines=["A2"], pacing="per-segment")
    ok2, reason2 = normalize_field_layout(good.layout_at(0.0))
    check("正例：把那个雷格放进第一段 ⇒ 车端接受", ok2 is not None, str(reason2))
    check("stub 默认只报 known 里的特征（自动过滤）",
          DS.DroneStub(segments=[["N10"]], mines=["A2"],
                       pacing="per-segment").layout_at(0.0)["mines"] == [])
    # 负例：bridges 不许空
    try:
        DS.DroneStub(bridges=[])
        check("bridges=[] 必须抛（车端 allow_empty=False）", False)
    except ValueError:
        check("bridges=[] 抛 ValueError", True)

    # ---- 5.5 确定性：同输入两条报文逐字节相同 ----
    a = DS.DroneStub(segments=SEGS, mines=["A2"], pacing="per-segment").messages_at(
        8.0, last_t=0.0)
    b = DS.DroneStub(segments=SEGS, mines=["A2"], pacing="per-segment").messages_at(
        8.0, last_t=0.0)
    check("同输入两次 messages_at 逐字相同（可重放）", a == b, f"{a} vs {b}")
    check("版本单调递增（车端会拒收更小版本）",
          all(json.loads(a[i])["version"] < json.loads(a[i + 1])["version"]
              for i in range(len(a) - 1)))


# --------------------------------------------------------------------------- #
#  §6 夹具/API 的工程判据
# --------------------------------------------------------------------------- #
def sec6_engineering() -> None:
    section("6. 工程判据：夹具不可绕过、确定性、契约行")
    # ---- 6.1 未知夹具必须显式报错，不回落默认图 ----
    try:
        W.load("I99")
        check("未知夹具必须抛", False)
    except W.FixtureError:
        check("未知夹具抛 FixtureError（不回落 config/map.yaml）", True)
    # ---- 6.2 夹具 schema 校验 ----
    base = pathlib.Path(W.INCIDENTS_DIR)
    check("夹具目录存在", base.is_dir(), str(base))
    for fn in sorted(p.name for p in base.glob("*.json")) if base.is_dir() else []:
        doc = json.loads((base / fn).read_text(encoding="utf-8"))
        check(f"{fn} schema = dzrobot-incident/1", doc.get("schema") == W.FIXTURE_SCHEMA,
              str(doc.get("schema")))
        check(f"{fn} 有 negative 段（否则证不了它在测注入）", bool(doc.get("negative")),
              str(doc.get("negative")))
        check(f"{fn} 有 sources（出处）", bool(doc.get("sources")))
    # ---- 6.3 确定性：同夹具两次跑，指标逐位相同 ----
    inc = W.load("I16")
    r1 = _run(inc, on=True)
    r2 = _run(inc, on=True)
    keys = ("phase_final", "length_ratio", "out_of_bounds", "belief_error_rms_m")
    check("同夹具两次跑指标逐位相同（确定性）",
          all(r1["values"].get(k) == r2["values"].get(k) for k in keys), str(keys))
    # ---- 6.4 契约行 ----
    line = W.contract_line(r1["payload_payload"])
    check(f"契约行前缀 = {W.INCIDENT_PREFIX}", line.startswith(W.INCIDENT_PREFIX + " "))
    body = json.loads(line[len(W.INCIDENT_PREFIX):].strip())
    for key in ("schema", "incident", "slug", "arm", "events", "events_fired",
                "symptom_seen", "criteria", "values", "negative"):
        check(f"契约行含键 {key}", key in body, str(sorted(body)))


def main() -> int:
    print("=" * 78)
    print("WB-3 案例场景库验收 · 16 条现场案例 → 可重放夹具 + 事件注入 API + 无人机 stub")
    print("=" * 78)
    sec1_no_second_projection()
    sec2_fixtures()
    sec3_negatives()
    sec3b_sub_example_i11()
    sec4_event_api()
    sec5_drone_stub()
    sec6_engineering()
    finish()
    return 0


if __name__ == "__main__":
    sys.exit(main())
