"""日志复盘：把黑匣子事件流变成「概览指标 + 异常清单」。

`blackbox.py` 负责"记下来"，这里负责"看懂"。设计目标是**不需要人盯着日志猜**：

    findings = diagnose(events)
    for f in findings:
        print(f.severity, f.title, "->", f.hint)

一条规则一个 `_rule_*` 函数，输入都是 `(ctx)`，输出 `List[Finding]`。
新增规则只要写函数 + 加进 `_RULES`，不用改别处。

## 规则怎么定的

不是"看到某个词就报警"，而是按**跑车这件事的真实卡点**倒推：
一局要跑通，必须依次发生
   情报收到 → 投票收敛 → 路线发出 → 车端复位 → 到停靠点打击 → 回终点
中间任何一环断了，报告要直接指出"断在哪一环、为什么、去看哪个参数"。
"""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Sequence

from ag_core.blackbox import EV, Event

__all__ = ["Finding", "diagnose", "summarize", "SEVERITY_ORDER"]

#: 严重度排序（报告按它排）
SEVERITY_ORDER = {"critical": 0, "warn": 1, "info": 2, "ok": 3}

#: `route.blocked` 的 reason -> 现场排查提示。
#: 节点写入的 reason 必须来自 `blackbox.REASON`，否则这里查不到提示。
BLOCK_HINT = {
    "min_mines": "确认的雷数不够 —— 看 intel.recv 里的 mines 是「没拍到」还是「投票被过滤」",
    "face_unknown": "敌我未判决 —— 检查视觉能否看到敌我识别区（/offset_center 的 cls 是否恒为 0）",
    "no_cells": "任务序列为空 —— 情报里既没有雷也没有终点，看 IntelVoter 的 stable 内容",
    "min_shoot_stops": "打击停靠点少于 min_shoot_stops，被主动拦下",
    "route_error": "路线串拼装失败 —— 大概率是格号越界（14 列 A~N x 11 行 1~11）",
    "bridge_reject": "桥洞布点与推断区域不符且开了 strict_bridge_zone",
    "bridge_zone": "桥洞自检告警（非阻断）",
}


@dataclass
class Finding:
    """一条诊断结论。`hint` 是给现场的人看的下一步动作，别写成废话。"""

    code: str
    severity: str
    title: str
    detail: str = ""
    hint: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _pct(part: int, whole: int) -> str:
    return f"{(100.0 * part / whole):.0f}%" if whole else "-"


def summarize(events: Sequence[Event]) -> Dict[str, Any]:
    """把事件流压成一份指标字典（报告与诊断共用同一份口径）。"""
    s: Dict[str, Any] = {
        "events": len(events),
        "nodes": [],
        "kinds": {},
        "start_ts": 0.0,
        "end_ts": 0.0,
        "span_s": 0.0,
        "sessions": [],
        "intel_recv": 0,
        "intel_recv_bad": 0,
        "intel_stable": 0,
        "intel_first_stable_s": None,
        "link_tx": 0,
        "link_rx": 0,
        "link_rx_bad": 0,
        "link_drop": 0,
        "link_drop_reasons": {},
        "vision_detect": 0,
        "plan_built": 0,
        "route_publish": 0,
        "route_sent": 0,          # kind == "route"
        "reset_sent": 0,          # kind == "reset"
        "route_blocked": 0,
        "block_reasons": {},
        "route_reject": 0,
        "reject_reasons": {},
        "route_cmds": [],
        "shoot_stops": [],
        "dry_run": None,
        "publish_reset": None,
        "homography_bad": 0,           # 单应未标定/无效（情报链在这里断）
        "vision_stale": 0,             # 相机断流/从没收到图像
        "vision_never": False,         # 一帧图像都没收到过（相机没起 / 话题不对）
        "car_node_present": False,     # 实车路径（发 /nav/waypoint）
        "shots": None,
        "aim_pass": 0,
        "aim_block": 0,
        "aim_block_cls": {},
        "aim_block_reasons": {},
        "fire_request": 0,
        "fire_denied": 0,
        "fire_denied_reasons": {},
        "fire_sent": 0,
        "referee_updates": 0,
        "referee_first": {},
        "referee_last": {},
        "ros_errors": [],
        "ros_warnings": 0,
        "node_errors": 0,
        "max_gap_s": 0.0,
        "max_gap_at": None,
        "max_gap_prev": None,
        "last_event": None,
        "blackbox_errors": 0,
    }

    if not events:
        return s

    kinds = Counter(e.ev for e in events)
    s["kinds"] = dict(kinds)
    s["nodes"] = sorted({e.node for e in events if e.node})
    s["sessions"] = sorted({e.session for e in events if e.session})
    s["start_ts"] = events[0].ts
    s["end_ts"] = events[-1].ts
    s["span_s"] = round(events[-1].ts - events[0].ts, 3)
    t0 = events[0].ts

    block_reasons: Counter = Counter()
    reject_reasons: Counter = Counter()
    drop_reasons: Counter = Counter()
    aim_cls: Counter = Counter()
    aim_reasons: Counter = Counter()
    denied: Counter = Counter()

    prev: Optional[Event] = None
    for e in events:
        ev = e.ev
        if prev is not None:
            gap = e.ts - prev.ts
            if gap > s["max_gap_s"]:
                s["max_gap_s"] = round(gap, 3)
                s["max_gap_at"] = f"{e.node}/{e.ev}"
                s["max_gap_prev"] = f"{prev.node}/{prev.ev}"
        prev = e

        if ev == EV.SESSION_BEGIN:
            # 开局那一行里带着"这局是按什么配置跑的"：很多结论要看它才判得准
            if e.get("dry_run") is not None:
                s["dry_run"] = bool(e.get("dry_run"))
            if e.get("publish_reset") is not None:
                s["publish_reset"] = bool(e.get("publish_reset"))
        elif ev == EV.INTEL_RECV:
            s["intel_recv"] += 1
            if e.get("ok", True) is False:
                s["intel_recv_bad"] += 1
        elif ev == EV.INTEL_STABLE:
            s["intel_stable"] += 1
            if s["intel_first_stable_s"] is None:
                s["intel_first_stable_s"] = round(e.ts - t0, 2)
        elif ev == EV.LINK_TX:
            s["link_tx"] += 1
        elif ev == EV.LINK_RX:
            s["link_rx"] += 1
            if e.get("ok", True) is False:
                s["link_rx_bad"] += 1
        elif ev == EV.LINK_DROP:
            s["link_drop"] += 1
            drop_reasons[e.text("why", "?")] += 1
        elif ev == EV.VISION_DETECT:
            s["vision_detect"] += 1
        elif ev == EV.HOMOGRAPHY:
            if e.get("ok") is False:
                s["homography_bad"] += 1
        elif ev == EV.VISION_STALE:
            if e.get("level") != "info" and e.text("note", "") != "图像恢复":
                s["vision_stale"] += 1
                if e.get("never"):
                    s["vision_never"] = True
        elif ev == EV.PLAN_BUILT:
            s["plan_built"] += 1
            if e.get("shoot_stops") is not None:
                s["shoot_stops"] = list(e.get("shoot_stops") or [])
        elif ev == EV.ROUTE_PUBLISH:
            s["route_publish"] += 1
            kind = e.text("kind", "route")
            if kind == "reset":
                s["reset_sent"] += 1
            else:
                s["route_sent"] += 1
                s["route_cmds"].append(e.text("cmd", ""))
            if e.get("dry_run") is not None:
                s["dry_run"] = bool(e.get("dry_run"))
            if e.get("shoot_stops") is not None:
                s["shoot_stops"] = list(e.get("shoot_stops") or [])
        elif ev == EV.ROUTE_BLOCKED:
            s["route_blocked"] += 1
            block_reasons[e.text("reason", "?")] += 1
        elif ev == EV.ROUTE_REJECT:
            s["route_reject"] += 1
            reject_reasons[e.text("reason", "?")] += 1
        elif ev == EV.SHOOT_COUNT:
            s["shots"] = int(e.num("value", 0))
        elif ev == EV.AIM_PASS:
            s["aim_pass"] += 1
        elif ev == EV.AIM_BLOCK:
            s["aim_block"] += 1
            aim_cls[str(int(e.num("cls", -1)))] += 1
            aim_reasons[e.text("why", "?")] += 1
        elif ev == EV.FIRE_REQUEST:
            s["fire_request"] += 1
        elif ev == EV.FIRE_DENIED:
            s["fire_denied"] += 1
            denied[e.text("reason", "?")] += 1
        elif ev == EV.FIRE_SENT:
            s["fire_sent"] += 1
        elif ev == EV.REFEREE_STATE:
            s["referee_updates"] += 1
            snap = {k: e.data.get(k) for k in ("hp", "ammo", "shots", "hit",
                                               "illuminated", "target_hp")}
            if not s["referee_first"]:
                s["referee_first"] = snap
            s["referee_last"] = snap
        elif ev == EV.NODE_ERROR:
            s["node_errors"] += 1
        elif ev == EV.ROS_LOG:
            lvl = e.text("lvl", "info")
            if lvl == "error":
                if len(s["ros_errors"]) < 5:
                    s["ros_errors"].append(e.text("msg", ""))
            elif lvl == "warn":
                s["ros_warnings"] += 1

    # 走的哪条路径，决定哪些规则适用（实车 car_node 发 /nav/waypoint；
    # 自研/仿真走 mission_node + chassis_node，不发路线）—— 不做这个区分就会误报。
    s["car_node_present"] = _has_node(events, "car_node")
    s["mission_node_present"] = _has_node(events, "mission_node")
    s["block_reasons"] = dict(block_reasons)
    s["reject_reasons"] = dict(reject_reasons)
    s["link_drop_reasons"] = dict(drop_reasons)
    s["aim_block_cls"] = dict(aim_cls)
    s["aim_block_reasons"] = dict(aim_reasons)
    s["fire_denied_reasons"] = dict(denied)
    le = events[-1]
    s["last_event"] = f"{le.node}/{le.ev} (T+{le.ts - t0:.1f}s)"
    return s


# ---------------------------------------------------------------- 规则
def _has_node(events: Sequence[Event], name: str) -> bool:
    return any(e.node == name for e in events)


def _has_ev(events: Sequence[Event], ev: str) -> bool:
    return any(e.ev == ev for e in events)


def _rule_sanity(ev: Sequence[Event], m: Dict[str, Any]) -> List[Finding]:
    out: List[Finding] = []
    if not _has_ev(ev, EV.SESSION_BEGIN):   # 车端 car_node / 自研路径的 mission_node 都会写
        out.append(Finding(
            "no-session-begin", "info", "没有 session.begin（可能只跑了机载端）",
            "报告按全部事件统计，可能混入别的时段。",
            "车端 car_node 启动会写这条；只调试机载端时可以忽略。",
        ))
    if m["blackbox_errors"]:
        out.append(Finding(
            "log-io", "warn", f"日志自身写入失败 {m['blackbox_errors']} 次",
            "磁盘满 / 权限不足 / 路径不可写。**报告可能不完整**。",
            "检查 /tmp 可用空间与权限。",
        ))
    return out


def _rule_intel(ev: Sequence[Event], m: Dict[str, Any]) -> List[Finding]:
    out: List[Finding] = []
    car_present = _has_node(ev, "car_node") or _has_ev(ev, EV.ROUTE_PUBLISH) \
        or _has_ev(ev, EV.INTEL_RECV) or _has_ev(ev, EV.ROUTE_BLOCKED)
    air_present = any(_has_node(ev, n) for n in
                      ("air_link_node", "intel_fusion_node", "vision_node", "fake_intel_node"))

    if m["homography_bad"]:
        # 单应没标定 = 像素落不到格号 = 机载端**根本产不出情报**。
        # 这时"车端没收到情报"只是表象，报 no-intel 会把人引去查网卡和端口，方向全错。
        out.append(Finding(
            "no-homography", "critical", "单应矩阵没标定 / 无效",
            "像素坐标换算不出格号，机载端即使看到目标也发不出情报 —— "
            "情报链在这里就断了，跟 UDP 链路无关。",
            "用 `tools/calib_homography.py` 标定（残差 < 5 cm），把 9 个数填进 "
            "params.yaml 的 homography；**换场地必须重标**。",
        ))
    elif car_present and m["intel_recv"] == 0:
        out.append(Finding(
            "no-intel", "critical", "车端一帧情报都没收到",
            "链路层断了：车端 car_node 起了，但 /link/field_intel 上没有任何一帧。",
            "查三样：① 机载端 air_link_node 是否在跑；② 两端是否同网段"
            "（车是 DHCP，IP 会变，见 docs/实机接口对照.md §9）；"
            "③ UDP 端口是否被防火墙挡。",
        ))
    if m["intel_recv"] > 0 and m["intel_stable"] == 0:
        out.append(Finding(
            "intel-not-stable", "critical",
            f"收到 {m['intel_recv']} 帧情报，但投票始终没收敛",
            "车端 IntelVoter 要求连续多帧一致才认；抖动说明要么视觉误检，要么链路误码。",
            "把 intel.recv 里各帧的 mines 摆一起看是否在抖；"
            "必要时降 min_votes、看 link.drop 是否有坏帧。",
        ))
    if m["intel_recv_bad"]:
        out.append(Finding(
            "intel-bad-frames", "warn", f"情报里有 {m['intel_recv_bad']} 帧被判为无效",
            "乱序 / 重复 / 过期帧。少量正常，占比高就要查链路。",
            "看 intel.recv 的 why 字段与 link.drop 的原因分布。",
        ))
    if air_present and m["link_tx"] > 0 and m["link_rx"] == 0 and car_present:
        out.append(Finding(
            "link-one-way", "critical",
            f"机载端发了 {m['link_tx']} 帧，车端一帧没收到",
            "单向不通 —— 九成是网段/端口/防火墙，而不是代码。",
            "两端互相 ping 一下；确认广播地址与端口一致。",
        ))
    return out


def _rule_route(ev: Sequence[Event], m: Dict[str, Any]) -> List[Finding]:
    out: List[Finding] = []
    # 这条规则只对**实车路径**（car_node → /nav/waypoint）成立：
    #   * dry_run 下本来就一条都不会发；
    #   * 自研/仿真路径（mission_node + chassis_node）根本不发路线，只发 /mission/waypoints，
    #     在这儿报"没发路线"就是纯误报。
    if (m["car_node_present"] and m["intel_stable"] > 0
            and m["route_publish"] == 0 and m["dry_run"] is not True):
        reasons = m["block_reasons"] or m["reject_reasons"] or {}
        top = ", ".join(f"{k}x{v}" for k, v in
                        sorted(reasons.items(), key=lambda kv: -kv[1])) or "（没有记录到原因）"
        hint = "；".join(BLOCK_HINT.get(k, "") for k in reasons if BLOCK_HINT.get(k))
        out.append(Finding(
            "no-route", "critical", "情报已收敛，但没有发出任何路线",
            f"被拦下的原因分布：{top}",
            hint or "看 ROUTE_BLOCKED 事件上游的节点日志，确认是哪一步 return 了。",
        ))
    if m["route_sent"] > 1:
        out.append(Finding(
            "route-repeated", "warn", f"路线下发了 {m['route_sent']} 次",
            "车端 path_executor 收到新路线会**从头重跑**，重复下发等于把车拽回起点。",
            "确认是不是情报在抖导致反复重规划；报告里的 route_cmds 可直接对比差异。",
        ))
    if m["route_sent"] > 0 and m["reset_sent"] == 0:
        # 注意：`publish_reset=false` 是"由人手动 reset"的合法配置，
        # 这时日志里本来就不该有 reset —— 报 critical 就是误报，会教人不信报告。
        if m["publish_reset"] is False:
            out.append(Finding(
                "no-reset", "info", "没发 reset（publish_reset=false）",
                "配置声明由人手动 reset，所以日志里不会有这条。",
                "确认你真的手动发过 reset —— 漏了的话车会按旧位姿跑飞。",
            ))
        else:
            out.append(Finding(
                "no-reset", "critical", "发了路线但没发 reset",
                "车是被人搬到起点的，里程计不知道自己在哪 —— 不发 reset 车会按旧位姿跑飞。",
                "确认 publish_reset 参数；或用 "
                "`ros2 topic pub --once /nav/waypoint std_msgs/msg/String "
                "\"{data: 'reset:N10,y'}\"` 手工补一次。",
            ))
    if m["route_reject"]:
        out.append(Finding(
            "route-reject", "critical", f"路线拼装失败 {m['route_reject']} 次",
            f"原因：{', '.join(f'{k}x{v}' for k, v in m['reject_reasons'].items())}",
            "基本都是格号越界（场地是 14 列 A~N x 11 行 1~11，没有 K14 这格）。",
        ))
    if m["dry_run"] is True:
        out.append(Finding(
            "dry-run", "info", "本次是 dry_run（只计算不发命令）",
            "日志里没有真正发车的记录，属于预期行为。",
            "确认路线无误后把 dry_run 改成 false 再跑。",
        ))
    return out


def _rule_shoot(ev: Sequence[Event], m: Dict[str, Any]) -> List[Finding]:
    out: List[Finding] = []
    if m["route_sent"] > 0 and not m["shoot_stops"]:
        out.append(Finding(
            "no-shoot-stop", "warn", "这局路线里没有打击停靠点",
            "等于主动放弃 15 分打击分（路线不含 '*'）。",
            "常见原因：敌我判决不是 blue（红面=友方，正确行为）、或时间预算不够"
            "（_fits_shoot_wait 判否）。看 route.blocked / ros.log 里的原话。",
        ))
    if m["shoot_stops"]:
        out.append(Finding(
            "shoot-stop", "ok", f"打击停靠点：{', '.join(m['shoot_stops'])}",
            "车端会在该格停车，等 /shoot_count 打满（上限 150s）再继续。",
            "确认这个格确实在敌方靶前、且是可达的。",
        ))
    if m["shots"] is not None:
        out.append(Finding(
            "shots", "info", f"车端上报累计发射 {m['shots']} 发",
            "整车弹药 10 发，打完就没法再打。",
            "",
        ))
    return out


def _rule_aim(ev: Sequence[Event], m: Dict[str, Any]) -> List[Finding]:
    out: List[Finding] = []
    blue_blocked = m["aim_block_cls"].get("1", 0)
    red_blocked = m["aim_block_cls"].get("2", 0)
    if blue_blocked:
        out.append(Finding(
            "aim-blocks-enemy", "critical", f"瞄准闸门拦掉了 {blue_blocked} 次**敌方(蓝)**目标",
            "白名单配反了 —— 该拦的是红面（友方）。",
            "检查 aim_guard_node 的 allow_classes / class_blue 参数，"
            "并核对 det_node 的类别号（1=蓝 2=红）。",
        ))
    if red_blocked:
        out.append(Finding(
            "aim-blocks-friend", "ok", f"瞄准闸门正确拦下 {red_blocked} 次红面（友方）",
            "官方 dzrknn_dataprocess 本身不分敌我，这道闸门就是防误击 −15 的。",
            "",
        ))
    if m["aim_block"] and not m["aim_pass"] and not red_blocked and not blue_blocked:
        out.append(Finding(
            "aim-gate", "warn", f"闸门拦截 {m['aim_block']} 次且从未放行",
            f"原因分布：{m['aim_block_reasons']}",
            "若是 enable_fire=false，属于人为关闸；否则看 why 字段。",
        ))
    if m["fire_denied"]:
        out.append(Finding(
            "fire-denied", "info", f"拒绝开火 {m['fire_denied']} 次",
            f"原因：{m['fire_denied_reasons']}",
            "弹药耗尽 / 冷却中 / 非敌方面，属于正常闸门行为。",
        ))
    return out


def _rule_referee(ev: Sequence[Event], m: Dict[str, Any]) -> List[Finding]:
    out: List[Finding] = []
    if m["referee_updates"] == 0:
        # 分清"节点跑了但读不到数据"（真问题）与"这次压根没起裁判节点"（联调常态），
        # 否则每次干跑都会挂一条注意，很快就没人看了。
        if "referee_node" in m["nodes"]:
            out.append(Finding(
                "no-referee", "warn", "referee_node 在跑，但整局没收到任何裁判状态",
                "dzjudgment 没起 / 串口没接 / 话题名不对，都会这样。"
                "**拿不到血量与弹药就无法判断打中没打中。**",
                "确认 dzjudgment 在跑，且 /HpAndHitmsg 有数据；"
                "referee_node 的 source 应为 dz。",
            ))
        else:
            out.append(Finding(
                "no-referee", "info", "本次没有任何裁判数据（referee_node 没在跑）",
                "干跑/联调时正常；但实车一局若也这样，等于整局拿不到血量与弹药。",
                "实车请把 referee_node 起来（source: dz），它订阅官方 /HpAndHitmsg。",
            ))
        return out
    first, last = m["referee_first"], m["referee_last"]
    try:
        hp0, hp1 = float(first.get("hp") or 0), float(last.get("hp") or 0)
    except (TypeError, ValueError):
        hp0 = hp1 = 0.0
    if hp0 and hp1 < hp0:
        sev = "critical" if hp1 <= 0 else "warn"
        out.append(Finding(
            "hp-drop", sev, f"己方血量 {hp0:.0f} -> {hp1:.0f}",
            "被对方打中。血量归零后整车基本失去得分能力。",
            "看 referee.state 序列里的 hit 方向字段，判断从哪边被打。",
        ))
    try:
        ammo = float(last.get("ammo") or 0)
    except (TypeError, ValueError):
        ammo = 0.0
    if ammo <= 0:
        out.append(Finding(
            "out-of-ammo", "info", "弹药已耗尽",
            "剩余时间再多也拿不到打击分。",
            "排雷/桥洞任务优先，别把时间耗在等开火上。",
        ))
    return out


def _rule_link(ev: Sequence[Event], m: Dict[str, Any]) -> List[Finding]:
    out: List[Finding] = []
    if m["link_drop"]:
        total = m["link_rx"] + m["link_drop"]
        out.append(Finding(
            "link-drop", "warn",
            f"链路丢弃 {m['link_drop']} 帧（占 {_pct(m['link_drop'], total)}）",
            f"原因：{m['link_drop_reasons']}",
            "坏帧多说明无线环境差；少量乱序被去重是正常设计。",
        ))
    if m["ros_errors"] or m["node_errors"]:
        detail = " | ".join(m["ros_errors"][:3])
        out.append(Finding(
            "node-error", "warn",
            f"节点报错 {len(m['ros_errors'])} 条（另有 NODE_ERROR {m['node_errors']} 条）",
            detail,
            "先修第一条错误 —— 后面的多半是它的连锁反应。",
        ))
    return out


def _rule_stall(ev: Sequence[Event], m: Dict[str, Any], max_gap_s: float) -> List[Finding]:
    out: List[Finding] = []
    if m["max_gap_s"] > max_gap_s:
        out.append(Finding(
            "stall", "warn",
            f"最长静默 {m['max_gap_s']:.1f}s（阈值 {max_gap_s:.0f}s）",
            f"从 {m['max_gap_prev']} 之后一直没动静，直到 {m['max_gap_at']}。",
            "这是「卡住」的位置：看这段时间前后哪条链路在等 —— "
            "等车到点、等开火打满、还是等情报。",
        ))
    out.append(Finding(
        "last-event", "info", f"最后一条事件：{m['last_event']}",
        "如果最后停在 route.publish，说明命令发出去了、后面全靠车端自己跑；"
        "停在 intel 相关，说明链路断了。",
        "",
    ))
    return out


def _rule_vision(ev: Sequence[Event], m: Dict[str, Any]) -> List[Finding]:
    """相机断流。

    为什么要单独一条规则：`usb_cam` 是**官方节点**，它崩了我们这边不会有任何异常，
    `det_node` 也只会无限刷 "waiting for /image_raw"。
    现场表现永远只是"什么都检测不到"，而真正的原因在链路上游三跳之外。
    """
    out: List[Finding] = []
    if m["vision_never"]:
        out.append(Finding(
            "no-image", "critical", "一帧图像都没收到过",
            "usb_cam 没起 / 崩了，或 input_topic 与实际发布的话题不一致。",
            "先 `ros2 node list` 看 usb_cam 在不在、`ros2 topic hz /image_raw` 有没有帧；"
            "再核 vision_node 的 input_topic。官方 usb_cam 实测会以 SIGABRT 崩掉。",
        ))
    elif m["vision_stale"]:
        out.append(Finding(
            "vision-stale", "warn", f"相机断流 {m['vision_stale']} 次",
            "中途有过一段时间没有图像：可能是 usb_cam 崩了、USB 带宽不足或被别的进程占了设备。",
            "跑 `tools/car_camera_check.sh` 单独验相机；上场前把 /image_raw 的帧率盯一会儿。",
        ))
    if m["vision_detect"] == 0 and _has_node(ev, "vision_node") and not m["vision_stale"]:
        out.append(Finding(
            "no-detection", "warn", "视觉节点在跑，但一局下来零检测",
            "模型没加载 / 解码参数不对（cls_activation、kpt_unit、input_layout）/ 画面里确实没目标。",
            "用 `tools/verify_decode.py` 在真机校准解码参数；看 vision.model 事件里的参数快照。",
        ))
    return out


_RULES = (_rule_sanity, _rule_intel, _rule_route, _rule_shoot,
          _rule_aim, _rule_referee, _rule_link, _rule_vision)


def diagnose(events: Sequence[Event], max_gap_s: float = 30.0) -> List[Finding]:
    """跑全部规则，返回按严重度排序的结论列表。"""
    if not events:
        # 空日志提前返回：否则"没有裁判状态""最后一条事件…"这些会一起冒出来，
        # 掩盖真正的那一条结论（日志压根没生成）。
        return [Finding(
            "no-events", "critical", "日志里没有任何事件",
            "要么节点没起来，要么日志没写到你以为的地方。",
            "确认 AG_LOG_DIR / log_dir 参数，并检查节点是否有写权限；"
            "RViz 里能看到节点在跑但这里为空，就是路径问题。",
        )]
    m = summarize(events)
    out: List[Finding] = []
    for rule in _RULES:
        try:
            out.extend(rule(events, m))
        except Exception as e:  # noqa: BLE001
            # 一条规则写挂了不能毁掉整份报告
            out.append(Finding("rule-error", "warn", f"规则 {rule.__name__} 执行失败",
                               f"{type(e).__name__}: {e}", "修 log_diagnosis.py 里这条规则。"))
    out.extend(_rule_stall(events, m, max_gap_s))
    out.sort(key=lambda f: SEVERITY_ORDER.get(f.severity, 9))
    return out
