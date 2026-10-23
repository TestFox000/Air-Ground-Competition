#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""dzlink_supervisor —— 无人机侧「空地链路」常驻指挥节点。

职责（一架无人机一直挂着这一个进程，就完成感知→下发→闭环）：

  ① 1 Hz 心跳         → /dzlink/drone_to_car      {"type":"heartbeat","seq":N}
  ② 感知结果转发      → /dzlink/drone_to_car      bridge_passable / target_zone / field_verify
                        输入三选一（可叠加）：文件尾随 JSONL / ROS 话题 String / UDP JSON
  ③ 车回传闭环        → 订阅 /dzlink/car_to_drone，解析 car_state / ack，写日志 + ack 台账
  ④ 路线 / 复位下发   → /nav/waypoint             RELIABLE depth10（语法见 `contract` 子命令）

设计要点
--------
· **纯逻辑层与 rclpy 层分离**：`SupervisorCore` 只吃「时间戳 + 输入样本」，吐「待发事件」，
  完全不 import rclpy。所以 Windows 本机可以 `--dry-run` / 跑单测，不需要 ROS。
· **QoS 逐条对着车侧订阅端设死**（读车侧源码核实，不是猜的）：
    /nav/waypoint        std_msgs/String  RELIABLE    / VOLATILE / depth10   ← 车 path_executor
    /dzlink/drone_to_car std_msgs/String  BEST_EFFORT / VOLATILE / depth10   ← 车 DroneTask
    /dzlink/car_to_drone std_msgs/String  BEST_EFFORT                      ← 车往这里发
  QoS 不匹配的表现是「完全收不到 **且不报错**」，所以这块不允许自由发挥。
· **域号唯一真相源**：import 阶段读 `~/dzlink_tools/dzlink.env`（与车侧 `~/dzrobot/dzros.env`
  逐字节对应的 118）。必须在 `rclpy.init()` **之前**生效，所以放在模块导入期做。
· **决策规则纯函数**：`decide(sample)` 把一份感知样本映射成「发什么 type」，可单独测（见 tests/）。

用法（无人机上，先 source ROS 环境）
------------------------------------
  # 0) 先看契约与自检（不发任何东西）
  python3 dzlink_supervisor.py contract
  python3 dzlink_supervisor.py dry-run --bridge lower=K9 --target G10,H10 --mines A3 --obstacles K9

  # 1) 常驻（最常用）：1Hz 心跳 + 尾随感知输出 + 监听车回传 + 吃命令文件
  python3 dzlink_supervisor.py run \
      --perception-glob '~/dzlink_tools/perception/out/*.jsonl' \
      --cmd-file ~/dzlink_tools/cmd.jsonl \
      --log-file ~/dzlink_logs/supervisor.jsonl

  # 2) 只跑一次：把当前情报发一遍（3 遍，防 VOLATILE 首帧丢）
  python3 dzlink_supervisor.py once --bridge lower=K9 --bridge upper=K4

  # 3) 下发路线 / 复位
  python3 dzlink_supervisor.py route "N10 M10 L10 L9 K9 J9 J10 I10 H10 G10* F10 F11 E11 N5"
  python3 dzlink_supervisor.py reset N10,y

  # 4) 纯监听车回传（诊断用，不发指令）
  python3 dzlink_supervisor.py listen --duration 30

退出码：0 正常 / 2 没有 ROS 环境（rclpy 不可用）/ 3 发出去了但没有任何车侧订阅者 / 1 参数错

命令文件（`--cmd-file`）格式：一行一个 JSON 对象，增量读取，可随时追加：
  {"type":"route","data":"N10 M10 L10 L9 K9"}
  {"type":"route","file":"/home/duzhong/dzlink_tools/routes/r1.txt"}
  {"type":"reset","block":"N10","yaw":"y"}
  {"type":"heartbeat"}            ← 只触发一次立即心跳
  {"type":"bridge","half":"lower","block":"K9","confidence":0.9}
  {"type":"stop"}                 ← 让常驻进程干净退出（等价 Ctrl+C）

感知输入（文件尾随 / ROS 话题 / UDP 三选一，接受的 JSON 形态一致，字段尽量宽容）：
  {"type":"bridge_passable","half":"lower","block":"K9","confidence":0.9,"ts":...}
  {"type":"bridge","direction":"left","hole":"K9","confidence":0.9}       ← 方向词自动映射
  {"type":"target_zone","blocks":["G10","H10"]}
  {"type":"field_verify","mines":["A3"],"obstacles":["K9"]}
  {"type":"mines","blocks":["A3"]} / {"type":"obstacles","blocks":["K9"]} ← 拆开办
  {"type":"heartbeat"}                                                     ← 感知侧也可以打
  也接受外层包裹：{"payload":{...}} / {"data":{...}} / {"intel":{...}}

如实交代（未做的事，不冒充）
--------------------------
· `run` 模式**没有在无人机真机上跑过**（本轮无真机窗口）；Windows 本机验证的是纯逻辑层
  单测 + dry-run 报文，无人机真机验证由 `smoke_ros.sh`（只读、发到本机回环订阅者）完成。
· direction→half 的默认映射（left→lower, right→upper）**是按场地约定做的假设**，不是从感知
  代码里读出来的事实。写错的表现是「车穿错洞」而不是报错，所以：默认打印一次映射表，
  可用 `--bridge-left-halves` 反转，也可在感知输出里直接写 canonical 的 `half` 绕开假设。
· **不解析 dzuav 的私有 UDP 二进制帧**（端口 50000，`ef ef ef ef 10` + u64 ts + 25B/目标 +
  CRC16 + `fe fe fe fe`）：帧格式属于感知层，本节点只吃 JSON。要接的话用
  `--perception-udp <port>` 让感知侧直接发 JSON，或加一个薄适配器转 JSON。
· 桥面是 z=0.35 的**第二个平面**（见 docs/感知可行性_摄像头评估.md），地面格号单应对桥顶不适用；
  但**那是感知侧要算的**，本节点只负责把 `{half,block}` 原样下发，不做像素→格换算。
"""
from __future__ import annotations

import argparse
import glob as globmod
import json
import os
import platform
import re
import signal
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence, Set, Tuple

__version__ = "1.0.0"

# --------------------------------------------------------------------------- #
#  话题与 QoS（★ 必须与车侧订阅端一致）
# --------------------------------------------------------------------------- #
TOPIC_ROUTE = "/nav/waypoint"             # 车：路线命令入口   RELIABLE
TOPIC_TO_CAR = "/dzlink/drone_to_car"     # 车：情报入口(JSON) BEST_EFFORT
TOPIC_FROM_CAR = "/dzlink/car_to_drone"   # 车：回传 car_state / ack
TOPIC_PERCEPTION = "/dzlink/perception"   # 可选：感知管线发 JSON String 的入口

QOS_ROUTE = "reliable"
QOS_LINK = "best_effort"

#: 车侧契约（`drone.py` / `contract.py` / `config/robot.yaml` 读源码核实）—— 写进日志，避免"我以为"
#: ★ 2026-10-12（t16）按车侧**真值**对齐：真值取自车上 `~/dzrobot/config/robot.yaml`
#:   · `mission.bridge_info_wait_s`  = 2.0   （robot.yaml:546）
#:   · `mission.auto_start_delay_s`  = 45.0  （robot.yaml:525）
#:   · `route.cross_lower_default`   = "K10" （robot.yaml:520）
#:   · `route.cross_upper_default`   = "K5"  （robot.yaml:521）
#:   历史口径 20.0 s / 10.0 s / K9,K4 已过期 —— 发车窗口从 20 s 缩到 2 s，
#:   机侧必须在**车起栈前**把情报备好（`--cmd-file` 预置或一次性 `once`）。
CAR_CONTRACT = {
    "info_stale_s": 30.0,        # drone.info_stale_s：情报超过这么久车侧当 None
    "mission_wait_s": 2.0,       # mission.bridge_info_wait_s：等不到就用默认穿洞格
    "auto_start_delay_s": 45.0,  # mission.auto_start_delay_s：起栈后多久自动出发
    "car_state_period_s": 0.7,   # 车侧 car_state 上报周期
    "heartbeat_offline_s": 3.0,  # 3s 没心跳车侧判无人机离线
    "bridge_default_lower": "K10",  # route.cross_lower_default（车侧兜底下洞格）
    "bridge_default_upper": "K5",   # route.cross_upper_default（车侧兜底上洞格）
}

DEFAULT_DOMAIN_ID = 118
DEFAULT_ENV_FILE = "~/dzlink_tools/dzlink.env"

#: 情报下发的**固定顺序**（不靠字典序，免得每次跑出来的顺序不一样）：
#: 桥洞最有时效性（车侧 `mission.bridge_info_wait_s` = 2.0 s 的等情报窗口）
#: → 先发；靶区/雷障是核对性质 → 后发。
EMIT_ORDER: Tuple[str, ...] = ("bridge_passable", "target_zone", "field_verify")


# --------------------------------------------------------------------------- #
#  域号：唯一真相源（导入期执行 —— 必须早于 rclpy.init）
# --------------------------------------------------------------------------- #
def _parse_env_file(path: str) -> Dict[str, str]:
    out: Dict[str, str] = {}
    try:
        with open(os.path.expanduser(path), "r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                if line.startswith("export "):
                    line = line[len("export "):].strip()
                k, v = line.split("=", 1)
                out[k.strip()] = v.strip().strip('"').strip("'")
    except OSError:
        return {}
    return out


def bootstrap_domain(env_file: str = DEFAULT_ENV_FILE, argv: Optional[Sequence[str]] = None,
                     quiet: bool = False) -> Dict[str, Any]:
    """把 ROS_DOMAIN_ID 收敛到唯一真相源，返回诊断 dict。

    顺序（与 dz_send_waypoint.py 一致）：
      1. 命令行 `--domain N` 最优先（显式就是显式；写成 `--domain N` 或 `--domain=N` 都认）
      2. dzlink.env / dzros.env 里的值
      3. 内置兜底 118
    如果调用方在外部**显式**设了别的域号，不覆盖，但必须**报警**——静默走错域是历史事故。
    """
    info: Dict[str, Any] = {"source": "builtin", "domain": DEFAULT_DOMAIN_ID, "external": None,
                            "env_file": env_file, "warnings": []}
    argv = list(argv or sys.argv[1:])
    cli_domain: Optional[str] = None
    for i, tok in enumerate(argv):
        if tok == "--domain" and i + 1 < len(argv):
            cli_domain = argv[i + 1]
        elif tok.startswith("--domain="):
            cli_domain = tok.split("=", 1)[1]
        elif tok == "--env-file" and i + 1 < len(argv):
            env_file = argv[i + 1]
        elif tok.startswith("--env-file="):
            env_file = tok.split("=", 1)[1]
    info["env_file"] = env_file

    external = os.environ.get("ROS_DOMAIN_ID")
    info["external"] = external

    if cli_domain is not None:
        domain, source = cli_domain, "cli"
    else:
        file_vals = _parse_env_file(env_file)
        if "ROS_DOMAIN_ID" in file_vals:
            domain, source = file_vals["ROS_DOMAIN_ID"], "envfile"
        else:
            domain, source = str(DEFAULT_DOMAIN_ID), "builtin"

    info["source"] = source
    info["domain"] = domain

    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")

    if external is not None and str(external) != str(domain):
        tag = "cli" if source == "cli" else "envfile"
        info["warnings"].append(
            f"外部已设 ROS_DOMAIN_ID={external}，被 {tag} 的 {domain} 覆盖"
            + ("（命令行显式指定，符合预期）" if source == "cli"
               else "（若你是有意 A/B 域号，请用 --domain 明确指定）"))
    if source == "builtin":
        info["warnings"].append(
            f"{env_file} 读不到 ROS_DOMAIN_ID，回退内置 {DEFAULT_DOMAIN_ID}")

    if not quiet:
        diag(f"[domain] ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']} (来源: {source})")
        for w in info["warnings"]:
            diag(f"[domain] ⚠️  {w}")
    return info


def default_log_path() -> str:
    if platform.system() == "Windows":
        return os.path.join(os.getcwd(), "dzlink_supervisor.jsonl")
    return "~/dzlink_logs/supervisor.jsonl"


def expand(path: str) -> str:
    return os.path.expanduser(os.path.expandvars(path))


def _force_utf8(stream) -> Any:
    """把 stdout/stderr 切到 UTF-8 + errors=replace。

    为什么必须做：Windows 控制台默认 GBK，一遇到 ⚠️/→ 这类字符就抛
    UnicodeEncodeError 把整个进程打断（本文件第一版就是这么崩的）；无人机上是
    POSIX/C locale 时同理。宁可让个别字符变成 '?'，也不能让节点因为一句日志死掉。
    """
    try:
        if (getattr(stream, "encoding", None) or "").lower().replace("-", "") != "utf8":
            stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[attr-defined]
    except (AttributeError, ValueError, OSError):
        pass
    return stream


_force_utf8(sys.stdout)
_force_utf8(sys.stderr)


def diag(msg: str) -> None:
    """诊断信息走 stderr —— 便于 `... | grep` 时不把噪音混进数据流。"""
    print(msg, file=sys.stderr, flush=True)


# --------------------------------------------------------------------------- #
#  数据模型（纯数据，无 ROS 依赖）
# --------------------------------------------------------------------------- #
@dataclass
class IntelRecord:
    """一份情报记录的当前状态。"""

    payload: Dict[str, Any]           # canonical 报文（已带 type/ts/seq）
    updated_at: float                 # 主机墙钟（= time.time()）
    source: str = ""                  # 来源说明（文件名/话题）
    key: str = ""                     # 去重键（canonical 内容的指纹）
    etype: str = ""                   # 该槽位承载的情报类型（bridge_passable 是分半场的槽）
    slot: str = ""                    # 槽位名（= records 的键）

    def age(self, now: float) -> float:
        return max(0.0, now - self.updated_at)


@dataclass
class OutEvent:
    topic: str
    qos: str
    text: str
    label: str
    kind: str = "intel"               # heartbeat | intel | route
    repeat: int = 1
    seq: Optional[int] = None
    created_at: float = 0.0
    force: bool = True                # 是否等待订阅者匹配（route 用 True）
    note: str = ""


@dataclass
class CarMsg:
    kind: str                         # ack | car_state | heartbeat | ? | non-json
    raw: str
    ts: float
    fields: Dict[str, Any] = field(default_factory=dict)


@dataclass
class CoreStats:
    started_at: float = 0.0
    samples_seen: int = 0
    samples_rejected: int = 0
    intel_emitted: int = 0
    heartbeat_emitted: int = 0
    route_emitted: int = 0
    car_msgs: int = 0
    acks: int = 0
    car_states: int = 0
    last_car_at: float = 0.0
    last_ack_seq: Optional[int] = None
    last_car_block: str = ""
    last_car_phase: str = ""

    def snapshot(self, now: float) -> Dict[str, Any]:
        idle = (now - self.last_car_at) if self.last_car_at else None
        return {
            "uptime_s": round(now - self.started_at, 1) if self.started_at else 0.0,
            "samples_seen": self.samples_seen,
            "samples_rejected": self.samples_rejected,
            "intel_emitted": self.intel_emitted,
            "heartbeat_emitted": self.heartbeat_emitted,
            "route_emitted": self.route_emitted,
            "car_msgs": self.car_msgs,
            "acks": self.acks,
            "car_states": self.car_states,
            "car_last_ago_s": None if idle is None else round(idle, 1),
            "last_ack_seq": self.last_ack_seq,
            "last_car_block": self.last_car_block,
            "last_car_phase": self.last_car_phase,
        }


# --------------------------------------------------------------------------- #
#  决策规则（★ 纯函数，单独可测）
# --------------------------------------------------------------------------- #
def decide(sample: Dict[str, Any], left_halves: str = "lower"
           ) -> List[Tuple[str, Dict[str, Any]]]:
    """一份感知样本 → [(event_type, canonical_payload), ...]。

    只做「翻译」，不做「该不该发」——后者是 tick 的事。
    认不出来的返回空列表（绝不猜）。
    `left_halves` 决定方向词 left 落到哪个半场（默认 lower，可用 CLI 反转）。
    """
    if not isinstance(sample, dict):
        return []

    # 外层包裹：感知侧可能整包塞在 payload / data / intel / result 里
    for wrapper in ("payload", "data", "intel", "result", "msg"):
        inner = sample.get(wrapper)
        if isinstance(inner, dict) and ("type" in inner or "kind" in inner):
            sample = inner
            break

    t = str(sample.get("type") or sample.get("kind") or sample.get("msg_type") or "").strip().lower()
    t = t.replace("-", "_").replace(" ", "_")

    if not t:
        # 没有 type：看字段猜（宽容但克制）
        if "half" in sample or ("direction" in sample and ("block" in sample or "hole" in sample)):
            t = "bridge_passable"
        elif "mines" in sample or "obstacles" in sample:
            t = "field_verify"
        elif "blocks" in sample:
            t = "target_zone"
        else:
            return []

    if t in ("heartbeat", "ping", "hb", "alive"):
        return [("heartbeat", {})]

    if t in ("bridge_passable", "bridge", "bridge_info", "hole", "tunnel", "cross"):
        p = _bridge_payload(sample, left_halves)
        return [] if p is None else [("bridge_passable", p)]

    if t in ("target_zone", "target", "targets", "blocks", "shoot_targets", "靶区"):
        blocks = _block_list(sample, ("blocks", "targets", "cells", "target", "zone"))
        if blocks is None:
            return []
        if not blocks:
            return []
        return [("target_zone", {"blocks": blocks})]

    if t in ("field_verify", "field", "mines", "obstacles", "mine", "obstacle", "雷障"):
        mines = _block_list(sample, ("mines", "mine", "雷", "mine_blocks"))
        obstacles = _block_list(sample, ("obstacles", "obstacle", "障", "obstacle_blocks"))
        if t == "mines" and mines is None:
            mines = _block_list(sample, ("blocks", "cells"))
        if t == "obstacles" and obstacles is None:
            obstacles = _block_list(sample, ("blocks", "cells"))
        if mines is None and obstacles is None:
            return []
        out: Dict[str, Any] = {}
        if mines is not None:
            out["mines"] = mines
        if obstacles is not None:
            out["obstacles"] = obstacles
        if "mines" not in out and "obstacles" not in out:
            return []
        return [("field_verify", out)]

    return []


def _norm_block(value: Any) -> Optional[str]:
    """'k9' / ' K9 ' / 'K9,y' → 'K9'；非法返回 None。"""
    if value is None:
        return None
    s = str(value).strip().upper()
    if not s:
        return None
    s = s.split(",")[0].strip()
    if re.fullmatch(r"SHOOT@[A-N](?:1[01]|[1-9])", s):
        return s[6:]
    if re.fullmatch(r"[A-N](?:1[01]|[1-9])", s):
        return s
    return None


def _block_list(sample: Dict[str, Any], keys: Sequence[str]) -> Optional[List[str]]:
    """从若干候选键里取格名列表；一个都没命中返回 None（区分「没给」和「给了空」）。"""
    for k in keys:
        if k not in sample:
            continue
        v = sample[k]
        if v is None:
            continue
        if isinstance(v, str):
            raw = [x for x in re.split(r"[\s,;|]+", v) if x]
        elif isinstance(v, (list, tuple, set)):
            raw = list(v)
        else:
            raw = [v]
        out: List[str] = []
        bad: List[str] = []
        for item in raw:
            b = _norm_block(item)
            if b is None:
                bad.append(str(item))
            elif b not in out:
                out.append(b)
        if bad:
            diag(f"[decide] ⚠️ {k} 里有无法识别的格名，已跳过: {bad}")
        return out
    return None


def _bridge_payload(sample: Dict[str, Any], left_halves: str = "lower") -> Optional[Dict[str, Any]]:
    """桥洞情报 → {"half","block","confidence"}；解析不出返回 None。"""
    half = str(sample.get("half", "") or "").strip().lower()
    block = _norm_block(sample.get("block") or sample.get("hole") or sample.get("cell")
                        or sample.get("bridge_block") or sample.get("cross"))
    if block is None:
        return None

    if half in ("lower", "upper"):
        pass
    elif half in ("low", "bottom", "bottom_half", "下", "下半场", "下半"):
        half = "lower"
    elif half in ("up", "top", "top_half", "上", "上半场", "上半"):
        half = "upper"
    else:
        # 用方向词映射：这是**场地约定假设**，不是读出来的事实 → 默认打印映射表
        direction = str(sample.get("direction") or sample.get("dir")
                        or sample.get("arrow") or sample.get("side") or "").strip().lower()
        mapping = _direction_map(left_halves)
        if direction in mapping:
            half = mapping[direction]
            diag(f"[decide] 方向词 {direction!r} → half={half!r}（约定映射，非源码事实）")
        else:
            diag(f"[decide] ⚠️ 桥洞情报缺 half 且方向词 {direction!r} 认不出，已丢弃: {sample}")
            return None

    conf = sample.get("confidence", sample.get("conf", 0.9))
    try:
        conf = float(conf)
    except (TypeError, ValueError):
        conf = 0.9
    conf = max(0.0, min(1.0, conf))
    return {"half": half, "block": block, "confidence": conf}


def _direction_map(left_halves: str) -> Dict[str, str]:
    left, right = ("lower", "upper") if left_halves == "lower" else ("upper", "lower")
    return {"left": left, "l": left, "west": left, "左": left, "向左": left,
            "right": right, "r": right, "east": right, "右": right, "向右": right}


def canonical_text(event_type: str, payload: Dict[str, Any], seq: Optional[int],
                   ts: Optional[float] = None) -> str:
    """把 (type, payload) 组装成发给车的 JSON 文本。

    字段与车侧契约严格一致（`drone.py:204-327`）：
      heartbeat        {"type":"heartbeat","ts":...[,"seq":N]}
      bridge_passable  {"type":"bridge_passable","half":...,"block":...,"confidence":...,"ts":...}
      target_zone      {"type":"target_zone","blocks":[...],"ts":...}
      field_verify     {"type":"field_verify","mines":[...],"obstacles":[...],"ts":...}
    带 seq → 车回 {"type":"ack","of":N} 到 /dzlink/car_to_drone（这是"车确实收到了"的硬证据）。
    """
    body: Dict[str, Any] = {"type": event_type}
    if event_type == "bridge_passable":
        body["half"] = str(payload["half"])
        body["block"] = str(payload["block"])
        body["confidence"] = float(payload.get("confidence", 0.9))
    elif event_type == "target_zone":
        body["blocks"] = [str(b) for b in payload.get("blocks", [])]
    elif event_type == "field_verify":
        if "mines" in payload:
            body["mines"] = [str(b) for b in payload["mines"]]
        if "obstacles" in payload:
            body["obstacles"] = [str(b) for b in payload["obstacles"]]
    elif event_type == "heartbeat":
        pass
    else:
        raise ValueError(f"未知情报类型: {event_type!r}")
    body["ts"] = round(float(ts if ts is not None else time.time()), 3)
    if seq is not None:
        body["seq"] = int(seq)
    return json.dumps(body, ensure_ascii=False, separators=(",", ":"))


def intel_key(event_type: str, payload: Dict[str, Any]) -> str:
    """内容指纹（用于"变了才重发"）。"""
    if event_type == "bridge_passable":
        return f"bridge|{payload.get('half')}|{payload.get('block')}|{payload.get('confidence')}"
    if event_type == "target_zone":
        return "target|" + ",".join(sorted(str(b) for b in payload.get("blocks", [])))
    if event_type == "field_verify":
        return ("field|"
                + ",".join(sorted(str(b) for b in payload.get("mines", [])))
                + "|" + ",".join(sorted(str(b) for b in payload.get("obstacles", []))))
    return event_type


def slot_sort_key(slot: str) -> Tuple[int, str]:
    """槽位排序：按 EMIT_ORDER 定序，同类按名字（lower 在 upper 前）。"""
    etype = slot.split(":", 1)[0]
    try:
        rank = EMIT_ORDER.index(etype)
    except ValueError:
        rank = len(EMIT_ORDER)
    return (rank, slot)


def slot_key(event_type: str, payload: Dict[str, Any]) -> str:
    """情报**槽位**：同一槽位的新内容替换旧内容，不同槽位互不干扰。

    ★ 为什么必须分槽：车侧 `P_DRONE.bridge_of("lower")` / `bridge_of("upper")` 是
    **两个独立半场**（`drone.py`），所以 lower 与 upper 的桥洞情报必须同时存在。
    第一版把两者存在同一个 `bridge_passable` 记录里 → 后到的 upper **直接覆盖** lower，
    表现是"我只看到一个半场的情报"，而且不报错。这是本文件修掉的第二个真 bug。
    """
    if event_type == "bridge_passable":
        return f"bridge_passable:{payload.get('half', '?')}"
    return event_type


# --------------------------------------------------------------------------- #
#  路线命令（★ 契约校验，纯函数）
# --------------------------------------------------------------------------- #
#: ★ 交替顺序必须 `1[01]|[1-9]`（先长后短）。写成 `[1-9]|1[01]` 时正则引擎会先吃掉
#: `K10` 的 `1`，把 `0` 当成多余字符 → 所有 10/11 列全部判非法（第一版就踩了这个坑）。
#: ★ 开头的 `(?!,)` 也必需：否则引擎会把格名组整个回溯成"不存在"，再让 `,y` 去配
#: 那个 `^...$` 里的空匹配 —— `N5,y` 会莫名变成"形状不认识"。
#: ★ 朝向后缀写成 `[xyXY]`（不用 re.IGNORECASE）：Python 3.12 里 `re.IGNORECASE is
#: re.UNICODE == 32`，把它当 match-flag 传给已编译 pattern 反而会**清掉** UNICODE，
#: 让 `N10` 这种最普通的输入直接匹配失败。这类"显式 flag 打翻默认 flag"的坑不值得踩。
RE_BLOCK = re.compile(r"^[A-N](?:1[01]|[1-9])$")
RE_TAIL = re.compile(r"^(?!,)([A-N](?:1[01]|[1-9]))?(\*)?(:SCAN_STOP)?(?:,([xyXY]))?$")
PREFIX_WORDS = ("多点", "多航点", "路线", "route", "multi", "waypoints", "单点", "single", "goto")


def validate_route(text: str) -> Tuple[bool, List[str]]:
    """按车侧 `route/parser.py` 的合法形状校验一条路线命令。

    返回 (ok, 问题列表)。**只校验形状，不校验地图可达性**（那是车侧的事）。
    空/非法输入必须被挡住 —— 一条畸形路线会让车"收到但行为不明"。
    """
    problems: List[str] = []
    body = " ".join(str(text).replace(";", " ").replace("|", " ").split())
    if not body:
        return False, ["路线为空"]

    tokens = body.split(" ")
    if tokens and tokens[0].lower() in PREFIX_WORDS and len(tokens) == 1:
        return False, [f"只有前缀 {tokens[0]!r}，没有航点"]

    start = 0
    if tokens and tokens[0].lower() in PREFIX_WORDS:
        start = 1

    is_reset = tokens[0].lower().startswith("reset:")
    is_load = tokens[0].lower().startswith("load:")

    if is_reset or is_load:
        head = tokens[0]
        if len(tokens) > 1:
            problems.append(f"{head.split(':')[0]}: 命令后面不该再有 token（收到 {tokens[1:]!r}）")
        arg = head.split(":", 1)[1].strip()
        if not arg:
            problems.append(f"{head.split(':')[0]}: 缺少参数")
        elif is_reset:
            m = re.fullmatch(r"([A-N](?:1[01]|[1-9]))(?:,([xyXY]))?", arg.upper())
            if not m:
                problems.append(f"reset 的参数必须是 格名[,x|,y]，收到 {arg!r}（例：reset:N10,y）")
        return (not problems), problems

    for idx, tok in enumerate(tokens[start:], start=start):
        problems.extend(_route_token_problems(tok, idx, last=(idx == len(tokens) - 1)))
    return (not problems), problems


def _route_token_problems(tok: str, idx: int, last: bool) -> List[str]:
    out: List[str] = []
    t = tok.strip()
    if not t:
        return out

    if re.fullmatch(r"SHOOT@[A-N](?:1[01]|[1-9])", t.upper()):
        return out

    body = t.upper()
    if body.startswith("R:"):
        body = body[2:]
    tail_match = RE_TAIL.fullmatch(body)
    if tail_match is None:
        out.append(f"token#{idx} {tok!r} 形状不认识"
                   + "（合法：格名 / G10* / R:F11 / R:B5* / N5,y / :SCAN_STOP / SHOOT@G11）")
        return out
    block, star, scan, facing = (tail_match.group(1), tail_match.group(2),
                                 tail_match.group(3), tail_match.group(4))
    if block is None and scan is None:
        out.append(f"token#{idx} {tok!r} 只有朝向后缀/星号，没有格名")
    if facing is not None and not last:
        out.append(f"token#{idx} {tok!r} 的朝向后缀 ,{facing.lower()} 只允许挂在最后一个 token"
                   "（车侧会自动挪到末尾，但写明更安全）")
    return out


def route_blocks(text: str) -> List[str]:
    """从路线里抽出格名序列（日志/比对用）。"""
    body = " ".join(str(text).replace(";", " ").replace("|", " ").split()).upper()
    out: List[str] = []
    for tok in body.split(" "):
        t = tok[2:] if tok.startswith("R:") else tok
        if t.startswith("SHOOT@"):
            t = t[6:]
        m = re.match(r"^([A-N](?:1[01]|[1-9]))", t)
        if m:
            out.append(m.group(1))
    return out


def reset_command(block: str, yaw: str = "y") -> str:
    b = _norm_block(block)
    if b is None:
        raise ValueError(f"reset 的格名非法: {block!r}（例：N10）")
    y = str(yaw).strip().lower()
    if y not in ("x", "y", ""):
        raise ValueError(f"reset 的朝向只能是 x/y，收到 {yaw!r}")
    return f"reset:{b},{y}" if y else f"reset:{b}"


def route_payload(text: str) -> str:
    """路线命令就是裸文本（不加 JSON 包装，车侧直接解析）。"""
    ok, problems = validate_route(text)
    if not ok:
        raise ValueError("路线不合法：\n  - " + "\n  - ".join(problems))
    return " ".join(str(text).replace(";", " ").replace("|", " ").split())


# --------------------------------------------------------------------------- #
#  P2 · 文件尾随（跨平台，含截断/轮转处理）
# --------------------------------------------------------------------------- #
class FileTailer:
    """按行增量读取文本文件；文件被截断或轮转时自动从头再读。

    为什么不用 `tail -f`：本机（Windows）与无人机都要能跑，且要能对"读了一半的行"
    做缓冲（写方若是分两次 write，直接 split 会产生半行 JSON）。
    """

    def __init__(self, path: str, from_end: bool = True):
        self.path = expand(path)
        self._offset = 0
        self._partial = ""
        self._lines = 0
        if from_end:
            try:
                self._offset = os.path.getsize(self.path)
            except OSError:
                self._offset = 0

    def poll(self, max_bytes: int = 1 << 20) -> List[str]:
        out: List[str] = []
        try:
            size = os.path.getsize(self.path)
            if size < self._offset:
                # 被截断或换了新文件
                self._offset = 0
                self._partial = ""
            if size == self._offset:
                return out
            with open(self.path, "r", encoding="utf-8", errors="replace") as fh:
                fh.seek(self._offset)
                chunk = fh.read(max_bytes)
                self._offset = fh.tell()
        except OSError:
            return out

        self._partial += chunk
        if chunk and not chunk.endswith("\n"):
            # 保留最后一段不完整行，下次再接
            idx = self._partial.rfind("\n")
            if idx < 0:
                return out
            lines = self._partial[:idx].split("\n")
            self._partial = self._partial[idx + 1:]
        else:
            lines = self._partial.split("\n")
            self._partial = ""

        for raw in lines:
            line = raw.strip().lstrip("\ufeff")
            if not line or line.startswith("#"):
                continue
            out.append(line)
            self._lines += 1
        return out

    @property
    def lines_read(self) -> int:
        return self._lines


class GlobTailer:
    """尾随一个 glob 匹配到的所有文件（每个文件各自记 offset）。"""

    def __init__(self, pattern: str, from_end: bool = True):
        self.pattern = pattern
        self._from_end = from_end
        self._tails: Dict[str, FileTailer] = {}
    def match(self) -> List[str]:
        return sorted(globmod.glob(expand(self.pattern)))

    def poll(self) -> List[Tuple[str, str]]:
        out: List[Tuple[str, str]] = []
        for path in self.match():
            if path not in self._tails:
                self._tails[path] = FileTailer(path, from_end=self._from_end)
                diag(f"[tail] 开始尾随 {path}")
            for line in self._tails[path].poll():
                out.append((path, line))
        return out


def sample_from_file_line(line: str) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
    """JSONL 一行 → 样本 dict + 错误说明。"""
    try:
        obj = json.loads(line)
    except Exception as exc:  # noqa: BLE001 - 诊断要给出原始行
        return None, f"不是合法 JSON（{exc}）"
    if not isinstance(obj, dict):
        return None, f"JSON 顶层不是对象（{type(obj).__name__}）"
    return obj, None


class UdpJsonSource:
    """收 UDP 报文里的 **JSON**（一行/一包一个样本）。

    ★ 这里刻意**不**解析 dzuav 的私有二进制帧（`ef ef ef ef 10 + u64 ts + 25B/目标
    + CRC16 + fe fe fe fe`，端口 50000）：那是感知层的产物，帧格式/CRC 一旦变动，
    两侧一起坏。本节点只吃**稳定的 JSON 契约**，二进制帧 → JSON 由感知侧(或其薄适配器)
    负责。所以：感知侧要接本节点时，直接往本端口发 JSON 即可。
    """

    def __init__(self, host: str, port: int):
        import socket
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(self.addr)
        self.sock.setblocking(False)
        self.packets = 0
        self.rejected = 0

    def poll(self, max_packets: int = 200) -> List[Tuple[str, Dict[str, Any]]]:
        out: List[Tuple[str, Dict[str, Any]]] = []
        for _ in range(max_packets):
            try:
                data, peer = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            self.packets += 1
            text = data.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            obj, err = sample_from_file_line(text)
            if obj is None:
                self.rejected += 1
                diag(f"[udp] 跳过一包（{err}）来自 {peer}: {text[:120]}")
                continue
            out.append((f"udp:{peer[0]}:{peer[1]}", obj))
        return out

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


def parse_car_message(raw: str, ts: float) -> CarMsg:
    """车回传的一条字符串 → CarMsg。畸形输入不抛异常，只降级标注。"""
    try:
        obj = json.loads(raw)
    except Exception:  # noqa: BLE001
        return CarMsg(kind="non-json", raw=raw, ts=ts, fields={})
    if not isinstance(obj, dict):
        return CarMsg(kind="non-json", raw=raw, ts=ts, fields={})
    kind = str(obj.get("type") or obj.get("kind") or "?").strip().lower()
    return CarMsg(kind=kind, raw=raw, ts=ts, fields=obj)


# --------------------------------------------------------------------------- #
#  P3 · 闭环台账（ack 记账 / 车状态记录）
# --------------------------------------------------------------------------- #
class AckLedger:
    """记录"我发出去的 seq"与"车回没回 ack"，并给出行级摘要。

    为什么要有它：`/dzlink/drone_to_car` 是 BEST_EFFORT + VOLATILE，发出去不等于收到。
    只有 ack 才是"车确实收到了"的硬证据（车侧 `drone.py:259-269`）。没有回执时必须
    重发，而不是假定成功。

    两类流量分开记，免得心跳把情报的回执率淹掉：
      · `sent`/`acks` —— **情报**（低频、有语义、每条都值得对账）
      · `hb_*`        —— **心跳**（1Hz 高频；只做「最近一段时间的回执率」统计）
    """

    def __init__(self, cap: int = 500, hb_cap: int = 300):
        self._cap = cap
        self._seqs: "Deque[int]" = deque(maxlen=cap)
        self._acked: Set[int] = set()
        self.sent: Dict[int, Dict[str, Any]] = {}
        self.acks: List[Tuple[int, float]] = []
        self._hb_sent: "Deque[int]" = deque(maxlen=hb_cap)
        self._hb_acked: Set[int] = set()
        self.hb_sent_total = 0
        self.hb_ack_total = 0

    def note_sent(self, seq: int, info: Dict[str, Any], now: float) -> None:
        if info.get("type") == "heartbeat":
            self._hb_sent.append(seq)
            self.hb_sent_total += 1
            return
        if seq not in self.sent:
            self._seqs.append(seq)
        self.sent[seq] = dict(info, sent_at=now)

    def note_ack(self, of: Optional[int], now: float) -> bool:
        if of is None:
            return False
        of = int(of)
        self.acks.append((of, now))
        if of in self._hb_sent:
            self._hb_acked.add(of)
            self.hb_ack_total += 1
            return True
        if of in self.sent or of in set(self._seqs):
            self._acked.add(of)
            return True
        return False

    @property
    def pending(self) -> List[int]:
        return [s for s in self._seqs if s not in self._acked]

    def summary(self) -> Dict[str, Any]:
        total = len(self._seqs)
        acked = len([s for s in self._seqs if s in self._acked])
        hb_window = len(self._hb_sent)
        hb_acked = len([s for s in self._hb_sent if s in self._hb_acked])
        return {"intel_sent": total, "intel_acked": acked, "pending_seq": total - acked,
                "hb_sent": self.hb_sent_total, "hb_acked": self.hb_ack_total,
                "hb_recent_sent": hb_window, "hb_recent_acked": hb_acked,
                #: 只要任一路收到过车回执，就说明**回传方向是通的**。
                #: 现场判据别写成 `intel_acked > 0` —— 旧核不吃 JSON 情报时那永远为 0。
                "any_acked": bool(acked or hb_acked),
                #: 有 ack 但没一条对得上我们发过的 seq → 几乎肯定是**域号不一致**
                #: （能听见别人域里的车，或车听见了另一个域的我们）。
                "suspicious_mismatch": bool(self.acks) and not (acked or hb_acked),
                "recent_acks": self.acks[-5:]}


# --------------------------------------------------------------------------- #
#  日志（JSONL，落盘；纯 stdlib）
# --------------------------------------------------------------------------- #
class JsonlLogger:
    """极简 JSONL 日志器。每次 write 都 flush —— 断电/被 kill 也不丢上下文。"""

    def __init__(self, path: Optional[str], echo: bool = False, cap_bytes: int = 64 << 20):
        self.path = expand(path) if path else None
        self.echo = echo
        self.cap_bytes = cap_bytes
        self._fh = None
        self.written = 0
        if self.path:
            d = os.path.dirname(self.path)
            if d:
                os.makedirs(d, exist_ok=True)
            self._fh = open(self.path, "a", encoding="utf-8")

    def log(self, kind: str, **fields: Any) -> Dict[str, Any]:
        rec = {"ts": round(time.time(), 3), "iso": time.strftime("%Y-%m-%dT%H:%M:%S"),
               "kind": kind}
        rec.update(fields)
        line = json.dumps(rec, ensure_ascii=False, separators=(",", ":"))
        if self._fh:
            # 简单的体积保护：超过上限就轮转一次，不无限长
            try:
                if self._fh.tell() > self.cap_bytes:
                    self._rotate()
            except OSError:
                pass
            self._fh.write(line + "\n")
            self._fh.flush()
            self.written += 1
        if self.echo:
            print(f"[log] {line}")
        return rec

    def _rotate(self) -> None:
        if not self.path or not self._fh:
            return
        self._fh.close()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        try:
            os.replace(self.path, f"{self.path}.{stamp}.bak")
        except OSError:
            pass
        self._fh = open(self.path, "a", encoding="utf-8")

    def close(self) -> None:
        if self._fh:
            try:
                self._fh.close()
            finally:
                self._fh = None


# --------------------------------------------------------------------------- #
#  ★ 纯逻辑核心：吃「时间戳 + 样本」，吐「待发事件」
# --------------------------------------------------------------------------- #
@dataclass
class CoreConfig:
    domain: str = str(DEFAULT_DOMAIN_ID)
    heartbeat_period_s: float = 1.0
    bridge_period_s: float = 2.0
    target_period_s: float = 0.0          # 0 = 变了才发
    field_period_s: float = 0.0
    info_ttl_s: float = CAR_CONTRACT["info_stale_s"]
    intel_repeat: int = 3
    route_repeat: int = 3
    ema: bool = True                      # 是否汇总"当前信念"（便于 --status / 日志）
    left_halves: str = "lower"

    def normalise(self) -> "CoreConfig":
        self.left_halves = "lower" if str(self.left_halves).lower().startswith("low") else "upper"
        self.heartbeat_period_s = max(0.05, float(self.heartbeat_period_s))
        self.bridge_period_s = max(0.0, float(self.bridge_period_s))
        self.target_period_s = max(0.0, float(self.target_period_s))
        self.field_period_s = max(0.0, float(self.field_period_s))
        self.info_ttl_s = max(1.0, float(self.info_ttl_s))
        self.intel_repeat = max(1, int(self.intel_repeat))
        self.route_repeat = max(1, int(self.route_repeat))
        return self

    def period_for(self, etype: str) -> float:
        """某类情报的重发周期（秒）。0 = 只在内容变化时发。"""
        return {"bridge_passable": self.bridge_period_s,
                "target_zone": self.target_period_s,
                "field_verify": self.field_period_s}.get(etype, 0.0)


class SupervisorCore:
    """★ 无 rclpy 依赖的决策核心。ROS 层只负责把它的输出搬到 DDS 上。"""

    def __init__(self, cfg: Optional[CoreConfig] = None, logger: Optional[JsonlLogger] = None):
        self.cfg = (cfg or CoreConfig()).normalise()
        self.log = logger or JsonlLogger(None)
        self.stats = CoreStats(started_at=time.time())
        self.records: Dict[str, IntelRecord] = {}
        self.ledger = AckLedger()
        self.car_recent: "Deque[CarMsg]" = deque(maxlen=200)
        self._seq = 0
        self._next_hb = 0.0
        self._hb_count = 0
        self._next_intel: Dict[str, float] = {}
        self._routes: "Deque[OutEvent]" = deque()
        self._route_log: List[str] = []
        self._stopping = False
        self.notes: List[str] = []

    # ---------------- 序号与任务 ---------------- #
    def next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def submit_route(self, text: str, repeat: Optional[int] = None, now: Optional[float] = None,
                     source: str = "cli") -> None:
        """路线命令入队（会做契约校验，非法直接抛 ValueError）。"""
        payload = route_payload(text)
        n = self.cfg.route_repeat if repeat is None else max(1, int(repeat))
        ev = OutEvent(topic=TOPIC_ROUTE, qos=QOS_ROUTE, text=payload, label="route",
                      kind="route", repeat=n, created_at=now if now is not None else time.time(),
                      force=True, note=source)
        self._routes.append(ev)
        blocks = route_blocks(payload)
        self._route_log.append(payload)
        self.log.log("route_queued", text=payload, repeat=n, blocks=blocks,
                     n_blocks=len(blocks), source=source)
        print(f"[route] 入队（发 {n} 遍）: {payload[:120]}"
              + (f"  … 共 {len(blocks)} 个格点" if blocks else ""))

    def submit_reset(self, block: str, yaw: str = "y", repeat: Optional[int] = None,
                     now: Optional[float] = None) -> None:
        cmd = reset_command(block, yaw)
        n = self.cfg.route_repeat if repeat is None else max(1, int(repeat))
        self._routes.append(OutEvent(topic=TOPIC_ROUTE, qos=QOS_ROUTE, text=cmd, label="reset",
                                     kind="route", repeat=n,
                                     created_at=now if now is not None else time.time(),
                                     force=True, note="reset"))
        self.log.log("reset_queued", text=cmd, repeat=n)
        print(f"[reset] 入队（发 {n} 遍）: {cmd}")

    def submit_heartbeat(self) -> None:
        """安排下一次 tick 立刻发一条心跳（命令文件 / stdin 的 {"type":"heartbeat"} 用）。"""
        self._next_hb = 0.0
        diag("[cmd] 已安排一次立即心跳")

    def request_stop(self) -> None:
        self._stopping = True

    @property
    def stopping(self) -> bool:
        return self._stopping

    # ---------------- 输入 ---------------- #
    def ingest_sample(self, sample: Dict[str, Any], source: str = "",
                      now: Optional[float] = None) -> List[str]:
        """吃一份感知样本，落到 intel 台账。

        返回本次「新/更新」的**槽位**名（bridge 是 `bridge_passable:lower` 这种分半场槽）。
        """
        now = time.time() if now is None else now
        self.stats.samples_seen += 1
        pairs = decide(sample, self.cfg.left_halves)
        if not pairs:
            self.stats.samples_rejected += 1
            self.log.log("sample_rejected", source=source, sample=_short(sample))
            diag(f"[intel] 认不出的样本，已忽略（{source}）: {_short(sample)}")
            return []

        updated: List[str] = []
        for etype, payload in pairs:
            slot = slot_key(etype, payload)
            key = intel_key(etype, payload)
            rec = self.records.get(slot)
            if rec is not None and rec.key == key and (now - rec.updated_at) < self.cfg.info_ttl_s:
                # 内容没变：只刷新时效，不刷屏；同时把"下一次重发"也推后，
                # 免得周期到点后把一份**没变过的**情报又发一遍（现场日志会被刷满）。
                rec.updated_at = now
                rec.source = source or rec.source
                period = self.cfg.period_for(etype)
                if period > 0:
                    self._next_intel[slot] = now + period
                continue
            self.records[slot] = IntelRecord(payload=payload, updated_at=now,
                                             source=source, key=key, etype=etype, slot=slot)
            updated.append(slot)
            self.log.log("intel_in", type=etype, slot=slot, payload=payload, source=source,
                         key=key)
            diag(f"[intel] ← {etype}[{slot}]: {json.dumps(payload, ensure_ascii=False)}"
                 f"  ({source})")
        if updated:
            # 内容变了 → 立即重发（不等周期），抢占发车窗口
            for slot in updated:
                self._next_intel[slot] = 0.0
        return updated

    def ingest_car_raw(self, raw: str, now: Optional[float] = None) -> CarMsg:
        """吃一条车回传字符串，记账 + 落日志。"""
        now = time.time() if now is None else now
        msg = parse_car_message(raw, now)
        self.car_recent.append(msg)
        self.stats.car_msgs += 1
        self.stats.last_car_at = now
        if msg.kind == "ack":
            of = msg.fields.get("of")
            matched = self.ledger.note_ack(of if isinstance(of, int) else None, now)
            self.stats.acks += 1
            self.stats.last_ack_seq = of if isinstance(of, int) else None
            self.log.log("car_ack", of=of, matched=matched, raw=msg.raw[:400])
            diag(f"[car] ← ack of={of}{'' if matched else '（未匹配到已发 seq）'}")
        elif msg.kind == "car_state":
            self.stats.car_states += 1
            self.stats.last_car_block = str(msg.fields.get("block", "") or "")
            self.stats.last_car_phase = str(msg.fields.get("phase", "") or "")
            self.log.log("car_state", block=self.stats.last_car_block,
                         yaw_deg=msg.fields.get("yaw_deg"), phase=self.stats.last_car_phase,
                         seq=msg.fields.get("seq"))
            diag(f"[car] ← car_state block={self.stats.last_car_block or '?'} "
                 f"phase={self.stats.last_car_phase or '?'} "
                 f"yaw={msg.fields.get('yaw_deg')}")
        else:
            self.log.log("car_other", type=msg.kind, raw=msg.raw[:400])
            diag(f"[car] ← {msg.kind}: {msg.raw[:120]}")
        return msg

    # ---------------- 决策：本次 tick 该发什么 ---------------- #
    def _stale_slots(self, now: float) -> List[str]:
        return [s for s, r in self.records.items()
                if r.age(now) > self.cfg.info_ttl_s]

    def _make_intel_event(self, slot: str, now: float, note: str = "") -> Optional[OutEvent]:
        rec = self.records.get(slot)
        if rec is None:
            return None
        age = rec.age(now)
        if age > self.cfg.info_ttl_s:
            return None
        seq = self.next_seq()
        text = canonical_text(rec.etype, rec.payload, seq)
        ev = OutEvent(topic=TOPIC_TO_CAR, qos=QOS_LINK, text=text, label=rec.etype,
                      kind="intel", repeat=self.cfg.intel_repeat, seq=seq, created_at=now,
                      force=False, note=note or f"age={age:.1f}s")
        self.ledger.note_sent(seq, {"type": rec.etype, "slot": slot, "payload": rec.payload}, now)
        return ev

    def tick(self, now: Optional[float] = None) -> List[OutEvent]:
        """返回本次要发的事件列表（可能为空）。纯函数式：无副作用（除计数器/日志）。"""
        now = time.time() if now is None else now
        out: List[OutEvent] = []

        # ⑤ 先看有没有过期情报要撤回（打了日志才不会被"我明明发过"误导）
        for slot in self._stale_slots(now):
            rec = self.records[slot]
            self.log.log("intel_stale", type=rec.etype, slot=slot, age_s=round(rec.age(now), 1),
                         ttl_s=self.cfg.info_ttl_s, payload=rec.payload)
            diag(f"[intel] ⚠️ {slot} 已过期（{rec.age(now):.1f}s > {self.cfg.info_ttl_s:g}s），"
                 f"停止下发，车侧将回落默认值")

        # ④ 路线优先：路线一变，后面的情报就没意义了
        if self._routes:
            ev = self._routes.popleft()
            self.stats.route_emitted += 1
            self.log.log("route_emit", text=ev.text, repeat=ev.repeat, source=ev.note)
            out.append(ev)

        # ① 心跳
        if self._next_hb == 0.0 or now >= self._next_hb:
            seq = self.next_seq()
            text = canonical_text("heartbeat", {}, seq, ts=now)
            self._next_hb = now + self.cfg.heartbeat_period_s
            self._hb_count += 1
            self.stats.heartbeat_emitted += 1
            # 心跳也入台账：车侧只对带 seq 的消息回 ack（`drone.py:259-269`），
            # 所以心跳的 ack 率就是"链路还活着"的最灵敏指标（1Hz，几秒就能看出来）。
            self.ledger.note_sent(seq, {"type": "heartbeat"}, now)
            out.append(OutEvent(topic=TOPIC_TO_CAR, qos=QOS_LINK, text=text, label="heartbeat",
                                kind="heartbeat", repeat=1, seq=seq, created_at=now,
                                force=False, note=f"#{self._hb_count}"))

        # ② 情报（★ 按槽位遍历：lower/upper 是两个独立槽，互不覆盖；顺序固定）
        #
        # ★ t16：桥洞两半场必须**同轮成对发出**。
        #   车侧 `mission.bridge_info_wait_s`（现 2.0 s）只覆盖「两半场情报**相隔 ≤2 s** 到达」：
        #   先到的那半场入库，超过窗口才到的半场只进遥测、路线已冻结（车侧不换洞）。
        #   若两半场分别到点（各自的重发周期有相位差），车侧就可能看到 lower/upper
        #   相隔 ≈ bridge_period_s，恰好卡在窗口边缘。所以：**任一新鲜桥洞半场到点时，
        #   把所有新鲜桥洞半场一起发**（代价：每个周期多发几条 ≤160 B 的报文）。
        fresh_bridge = [s for s, r in self.records.items()
                        if r.etype == "bridge_passable"
                        and r.age(now) <= self.cfg.info_ttl_s]
        bridge_round_due = any(now >= self._next_intel.get(s, 0.0) for s in fresh_bridge)
        for slot in sorted(self.records, key=slot_sort_key):
            rec = self.records[slot]
            if rec.age(now) > self.cfg.info_ttl_s:
                continue
            due = self._next_intel.get(slot, 0.0)
            if now < due and not (bridge_round_due and rec.etype == "bridge_passable"):
                continue
            ev = self._make_intel_event(slot, now)
            if ev is None:
                continue
            period = self.cfg.period_for(rec.etype)
            # 周期为 0 → 只在内容变化时发（"刷新时效"不算变化）
            self._next_intel[slot] = (now + period) if period > 0 else float("inf")
            self.stats.intel_emitted += 1
            self.log.log("intel_emit", type=rec.etype, slot=slot, text=ev.text, seq=ev.seq,
                         age_s=round(rec.age(now), 1), repeat=ev.repeat)
            out.append(ev)
        return out

    # ---------------- 状态与日志 ---------------- #
    def status(self, now: Optional[float] = None) -> Dict[str, Any]:
        now = time.time() if now is None else now
        intel = {}
        for slot, r in self.records.items():
            intel[slot] = {"type": r.etype, "payload": r.payload,
                           "age_s": round(r.age(now), 1),
                           "fresh": r.age(now) <= self.cfg.info_ttl_s, "source": r.source}
        return {
            "version": __version__,
            "domain": os.environ.get("ROS_DOMAIN_ID"),
            "config": {
                "heartbeat_period_s": self.cfg.heartbeat_period_s,
                "bridge_period_s": self.cfg.bridge_period_s,
                "target_period_s": self.cfg.target_period_s,
                "field_period_s": self.cfg.field_period_s,
                "info_ttl_s": self.cfg.info_ttl_s,
                "intel_repeat": self.cfg.intel_repeat,
            },
            "intel": intel,
            "stats": self.stats.snapshot(now),
            "ack_ledger": self.ledger.summary(),
            "pending_routes": len(self._routes),
            "car_contract": CAR_CONTRACT,
        }

    def log_status(self, tag: str = "status") -> None:
        self.log.log(tag, **self.status())


def _short(obj: Any, limit: int = 200) -> str:
    try:
        s = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
    except (TypeError, ValueError):
        s = repr(obj)
    return s[:limit]


# --------------------------------------------------------------------------- #
#  ROS 层（★ 唯一 import rclpy 的地方）
# --------------------------------------------------------------------------- #
def make_qos(reliability: str):
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
    rel = (ReliabilityPolicy.RELIABLE if reliability == "reliable"
           else ReliabilityPolicy.BEST_EFFORT)
    return QoSProfile(depth=10, reliability=rel, durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST)


PUB_SETTERS: Dict[str, Callable[["SupervisorNode", int], None]] = {}


class SupervisorNode:
    """把 SupervisorCore 接到 DDS 上。存在的意义只有三件：收发、重试、日志。"""

    def __init__(self, core: SupervisorCore, perception_topic: Optional[str] = None,
                 status_period_s: float = 30.0, echo_car: bool = True):
        import rclpy
        from std_msgs.msg import String

        self.core = core
        self.String = String
        self.echo_car = echo_car
        self.status_period_s = max(0.0, float(status_period_s))
        self._next_status = time.time() + self.status_period_s if self.status_period_s else 0.0
        self._spin_routes: Dict[int, Any] = {}
        self._subs_count: Dict[str, int] = {}

        self.node = rclpy.create_node("dzlink_supervisor")
        self.pub_route = self.node.create_publisher(String, TOPIC_ROUTE, make_qos(QOS_ROUTE))
        self.pub_link = self.node.create_publisher(String, TOPIC_TO_CAR, make_qos(QOS_LINK))
        self.sub_car = self.node.create_subscription(
            String, TOPIC_FROM_CAR, self._on_car, make_qos(QOS_LINK))
        self.sub_perception = None
        if perception_topic:
            self.sub_perception = self.node.create_subscription(
                String, perception_topic, self._on_perception, make_qos(QOS_LINK))
            diag(f"[ros] 订阅感知话题 {perception_topic}（String/JSON，BEST_EFFORT）")

    # ---------- 回调 ---------- #
    def _on_car(self, msg) -> None:
        if self.echo_car:
            self.core.ingest_car_raw(msg.data)

    def _on_perception(self, msg) -> None:
        obj, err = sample_from_file_line(msg.data)
        if obj is None:
            diag(f"[ros] 感知话题收到非 JSON 内容（{err}）: {msg.data[:120]}")
            self.core.stats.samples_rejected += 1
            return
            self.core.ingest_sample(obj, source=f"topic:{TOPIC_PERCEPTION}")
    # ---------- 发送 ---------- #
    def _settle(self, timeout_s: float) -> None:
        import rclpy
        end = time.time() + max(0.0, timeout_s)
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _subscription_count(self, pub) -> int:
        try:
            return int(pub.get_subscription_count())
        except Exception:  # noqa: BLE001
            return 0

    def dispatch(self, events: List[OutEvent]) -> int:
        """发送一批事件；返回「订阅者匹配数 == 0」的事件数（0 = 全部有订阅者）。"""
        import rclpy
        unmatched = 0
        for ev in events:
            pub = self.pub_route if ev.topic == TOPIC_ROUTE else self.pub_link
            matched = self._subscription_count(pub)
            if ev.kind == "route":
                # 路线是 RELIABLE，且车收到即出发 —— 值得等一下发现
                end = time.time() + 3.0
                while time.time() < end and self._subscription_count(pub) == 0:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                matched = self._subscription_count(pub)
                self._settle(0.5)   # 发现异步收敛
                matched = self._subscription_count(pub)

            self._subscriptions_note(ev.topic, matched)
            if matched == 0:
                unmatched += 1
                diag(f"[send] ⚠️ {ev.topic} 订阅者匹配数=0（{ev.kind}）—— "
                     f"车侧栈没跑 / 域号不一致（当前域 {os.environ.get('ROS_DOMAIN_ID')}）"
                     f" / 组播被拦")
            for i in range(max(1, ev.repeat)):
                m = self.String()
                m.data = ev.text
                pub.publish(m)
                rclpy.spin_once(self.node, timeout_sec=0.02)
                diag(f"[send] → {ev.label} [{i + 1}/{max(1, ev.repeat)}] "
                     f"subs={matched} {ev.text[:150]}")
                if i + 1 < max(1, ev.repeat):
                    self._settle(0.2)
        return unmatched

    def _subscriptions_note(self, topic: str, matched: int) -> None:
        prev = self._subs_count.get(topic)
        if prev != matched:
            self._subs_count[topic] = matched
            self.core.log.log("subs", topic=topic, count=matched)

    def spin_step(self, timeout_s: float = 0.1) -> None:
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=timeout_s)

    def maybe_status(self, now: Optional[float] = None) -> None:
        if not self.status_period_s:
            return
        now = time.time() if now is None else now
        if now >= self._next_status:
            st = self.core.status(now)
            self.core.log.log("status", **{k: v for k, v in st.items() if k != "car_contract"})
            print(f"[status] {json.dumps(st['stats'], ensure_ascii=False)} "
                  f"intel={list(st['intel'])} ack={st['ack_ledger']}", file=sys.stderr,
                  flush=True)
            self._next_status = now + self.status_period_s

    def destroy(self) -> None:
        try:
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
#  命令文件（JSONL，增量）
# --------------------------------------------------------------------------- #
class CommandFile:
    """读 `--cmd-file`（JSONL）并执行；追加即生效。格式见模块 docstring。"""

    def __init__(self, path: str, from_end: bool = True):
        self.tailer = FileTailer(path, from_end=from_end)
        self.path = expand(path)

    def poll(self, core: SupervisorCore) -> int:
        """返回**被识别并执行**的命令条数（坏行/未知类型不算）。"""
        n = 0
        for line in self.tailer.poll():
            try:
                cmd = json.loads(line)
            except Exception as exc:  # noqa: BLE001
                diag(f"[cmd] 跳过非法 JSON 行（{exc}）: {line[:120]}")
                core.log.log("cmd_bad", line=line[:400], error=str(exc))
                continue
            if not isinstance(cmd, dict):
                diag(f"[cmd] 跳过非对象行: {line[:120]}")
                continue
            if self.execute(cmd, core):
                n += 1
        return n

    @staticmethod
    def execute(cmd: Dict[str, Any], core: SupervisorCore) -> bool:
        """执行一条命令。返回 True=认了并做了，False=不认识/参数坏（已忽略）。

        坏命令只跳过、绝不抛出去：命令文件的作者可能是感知脚本，一条写错的类型或格名
        不应该把常驻节点带走（无人机上没人守着终端）。
        """
        t = str(cmd.get("type") or cmd.get("cmd") or "").strip().lower()
        core.log.log("cmd_in", cmd=cmd)
        try:
            if t in ("route", "waypoint", "nav"):
                data = cmd.get("data") or cmd.get("text") or cmd.get("route")
                if not data and cmd.get("file"):
                    with open(expand(str(cmd["file"])), "r", encoding="utf-8") as fh:
                        data = fh.read().strip()
                if not data:
                    diag("[cmd] route 命令缺 data/text/file，已跳过")
                    return False
                core.submit_route(str(data), repeat=cmd.get("repeat"), source="cmd-file")
                return True
            if t == "reset":
                core.submit_reset(str(cmd.get("block") or ""), str(cmd.get("yaw") or "y"),
                                  repeat=cmd.get("repeat"))
                return True
            if t == "heartbeat":
                core.submit_heartbeat()
                return True
            if t in ("bridge", "bridge_passable"):
                core.ingest_sample(dict(cmd, type="bridge_passable"), source="cmd-file")
                return True
            if t in ("target", "target_zone"):
                core.ingest_sample(dict(cmd, type="target_zone"), source="cmd-file")
                return True
            if t in ("field", "field_verify"):
                core.ingest_sample(dict(cmd, type="field_verify"), source="cmd-file")
                return True
            if t in ("stop", "quit", "exit"):
                diag("[cmd] 收到 stop，准备退出")
                core.request_stop()
                return True
        except (ValueError, OSError) as exc:
            diag(f"[cmd] 命令被拒（{exc}），已跳过: {_short(cmd)}")
            core.log.log("cmd_rejected", cmd=cmd, error=str(exc))
            return False
        diag(f"[cmd] 不认识的命令类型 {t!r}，已忽略")
        return False


# --------------------------------------------------------------------------- #
#  CLI 子命令
# --------------------------------------------------------------------------- #
def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="dzlink_supervisor",
        description="无人机侧空地链路常驻节点（心跳 / 情报 / 回传闭环 / 路线下发）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--domain", default=None, help=f"显式指定 ROS 域号（默认读 dzlink.env={DEFAULT_DOMAIN_ID}）")
    ap.add_argument("--env-file", default=DEFAULT_ENV_FILE, help=f"域号文件（默认 {DEFAULT_ENV_FILE}）")
    ap.add_argument("--version", action="version", version=f"dzlink_supervisor {__version__}")

    sub = ap.add_subparsers(dest="cmd")

    def common(p):
        # 域号/环境文件也放到子命令上：`run --domain 7` 与 `--domain 7 run` 都要能用
        p.add_argument("--domain", default=None, help="显式指定 ROS 域号（最高优先级）")
        p.add_argument("--env-file", default=DEFAULT_ENV_FILE,
                       help=f"域号文件（默认 {DEFAULT_ENV_FILE}）")
        p.add_argument("--log-file", default=None, help=f"JSONL 日志（默认 {default_log_path()}）")
        p.add_argument("--no-log", action="store_true", help="不落日志（只打印）")
        p.add_argument("--heartbeat-hz", type=float, default=1.0, help="心跳频率（默认 1Hz）")
        p.add_argument("--bridge-period", type=float, default=2.0,
                       help="桥洞情报重发周期秒（默认 2.0；0=只在变化时发）")
        p.add_argument("--target-period", type=float, default=0.0,
                       help="靶区重发周期秒（默认 0=只在变化时发）")
        p.add_argument("--field-period", type=float, default=0.0,
                       help="雷障重发周期秒（默认 0=只在变化时发）")
        p.add_argument("--info-ttl", type=float, default=CAR_CONTRACT["info_stale_s"],
                       help=f"情报时效秒（默认 {CAR_CONTRACT['info_stale_s']:g}，与车侧一致）")
        p.add_argument("--repeat", type=int, default=3, help="每条情报重复发送遍数（默认 3）")
        p.add_argument("--route-repeat", type=int, default=3, help="路线重复发送遍数（默认 3）")
        p.add_argument("--bridge-left-halves", choices=["lower", "upper"], default="lower",
                       help="方向词 left 映射到哪个半场（默认 lower；★ 这是约定假设，见 --help 顶部说明）")
        p.add_argument("--echo-log", action="store_true", help="同时把日志打到 stdout")

    # ---- run：常驻 ----
    p = sub.add_parser("run", help="常驻运行（心跳 + 感知转发 + 车回传 + 命令文件）")
    common(p)
    p.add_argument("--perception-glob", default=None,
                   help="感知输出的 glob（如 '~/dzlink_tools/perception/out/*.jsonl'）")
    p.add_argument("--perception-file", action="append", default=None,
                   help="单个感知文件（可重复给）")
    p.add_argument("--perception-from-start", action="store_true",
                   help="感知文件从**头**读（默认从尾读，即只处理启动之后新写的行）")
    p.add_argument("--perception-topic", default=None,
                   help=f"额外订阅感知话题（默认不订阅；填 {TOPIC_PERCEPTION} 即用约定话题）")
    p.add_argument("--perception-udp", type=int, default=None, metavar="PORT",
                   help="额外监听 UDP 端口收 JSON 样本（★ 只吃 JSON，不解析 dzuav 二进制帧）")
    p.add_argument("--perception-udp-host", default="0.0.0.0",
                   help="UDP 绑定地址（默认 0.0.0.0）")
    p.add_argument("--cmd-file", default=None, help="命令文件（JSONL，追加即生效）")
    p.add_argument("--cmd-from-start", action="store_true",
                   help="命令文件从**头**读（默认从尾读，即只执行启动之后追加的命令）")
    p.add_argument("--route", default=None, help="启动时先下发这条路线")
    p.add_argument("--reset", default=None, help="启动时先下发复位（如 N10,y）")
    p.add_argument("--stdin", action="store_true", help="从 stdin 读命令（JSONL，一行一个）")
    p.add_argument("--duration", type=float, default=0.0, help="跑多少秒后退出（0=一直跑）")
    p.add_argument("--status-period", type=float, default=30.0, help="状态日志周期秒（0=关）")
    p.add_argument("--bridge", action="append", default=None, metavar="HALF=BLOCK",
                   help="启动时先注入桥洞情报（可重复），如 --bridge lower=K9")
    p.add_argument("--target", default=None, help="启动时先注入靶区，逗号分隔")
    p.add_argument("--mines", default=None, help="启动时先注入雷格名，逗号分隔")
    p.add_argument("--obstacles", default=None, help="启动时先注入障碍格名，逗号分隔")
    p.add_argument("--dry-run", action="store_true", help="不连 ROS：只把决策结果打印出来")
    p.add_argument("--loop-hz", type=float, default=20.0, help="决策循环频率（默认 20Hz）")

    # ---- once：发一次性情报 ----
    p = sub.add_parser("once", help="把命令行给的情报发一遍（1 次调用，测试用）")
    common(p)
    p.add_argument("--bridge", action="append", default=None, metavar="HALF=BLOCK",
                   help="桥洞情报（可重复），如 --bridge lower=K9")
    p.add_argument("--bridge-left-halves-raw", default=None, help=argparse.SUPPRESS)
    p.add_argument("--target", default=None, help="靶区格名，逗号分隔，如 G10,H10")
    p.add_argument("--mines", default=None, help="雷格名，逗号分隔")
    p.add_argument("--obstacles", default=None, help="障碍格名，逗号分隔")
    p.add_argument("--no-heartbeat", action="store_true", help="不要先发一条心跳")
    p.add_argument("--dry-run", action="store_true", help="只打印报文，不连 ROS")

    # ---- route / reset：一次性下发 ----
    p = sub.add_parser("route", help="下发一条路线（/nav/waypoint，RELIABLE）")
    p.add_argument("route", help='如 "N10 M10 L10 L9 K9" / "reset:N10,y" / "load:r1.txt"')
    common(p)
    p.add_argument("--dry-run", action="store_true")

    p = sub.add_parser("reset", help="下发复位（reset:格名[,x|,y]）")
    p.add_argument("reset", help="如 N10,y 或 N10")
    common(p)
    p.add_argument("--dry-run", action="store_true")

    # ---- listen：只监听 ----
    p = sub.add_parser("listen", help="只监听 /dzlink/car_to_drone（诊断）")
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--log-file", default=None)
    p.add_argument("--no-log", action="store_true")

    # ---- dry-run：本地看报文 ----
    p = sub.add_parser("dry-run", help="（= once --dry-run）本地组装报文并打印")
    common(p)
    p.add_argument("--bridge", action="append", default=None, metavar="HALF=BLOCK")
    p.add_argument("--target", default=None)
    p.add_argument("--mines", default=None)
    p.add_argument("--obstacles", default=None)
    p.add_argument("--route", default=None)
    p.add_argument("--reset", default=None)

    # ---- contract / self-test ----
    sub.add_parser("contract", help="打印与车侧的接口契约摘要")
    sub.add_parser("selftest", help="跑纯逻辑自检（不需要 ROS）")
    return ap


def _core_from_args(args) -> SupervisorCore:
    cfg = CoreConfig(
        domain=str(os.environ.get("ROS_DOMAIN_ID", DEFAULT_DOMAIN_ID)),
        heartbeat_period_s=1.0 / max(0.05, float(getattr(args, "heartbeat_hz", 1.0))),
        bridge_period_s=getattr(args, "bridge_period", 2.0),
        target_period_s=getattr(args, "target_period", 0.0),
        field_period_s=getattr(args, "field_period", 0.0),
        info_ttl_s=getattr(args, "info_ttl", CAR_CONTRACT["info_stale_s"]),
        intel_repeat=getattr(args, "repeat", 3),
        route_repeat=getattr(args, "route_repeat", 3),
        left_halves=getattr(args, "bridge_left_halves", "lower"),
    )
    log_path = None if getattr(args, "no_log", False) else (getattr(args, "log_file", None)
                                                            or default_log_path())
    logger = JsonlLogger(log_path, echo=bool(getattr(args, "echo_log", False)))
    core = SupervisorCore(cfg, logger)
    return core


def _add_samples_from_args(core: SupervisorCore, args) -> None:
    """把 --bridge/--target/--mines/--obstacles 变成一次 ingest。"""
    for kv in (getattr(args, "bridge", None) or []):
        if "=" not in kv:
            raise SystemExit(f"--bridge 要写成 half=block，收到 {kv!r}（例：lower=K9）")
        half, block = kv.split("=", 1)
        core.ingest_sample({"type": "bridge_passable", "half": half.strip(),
                            "block": block.strip(), "confidence": 0.9}, source="cli")
    if getattr(args, "target", None):
        core.ingest_sample({"type": "target_zone",
                            "blocks": [b for b in re.split(r"[\s,;|]+", args.target) if b]},
                           source="cli")
    if getattr(args, "mines", None) or getattr(args, "obstacles", None):
        sample: Dict[str, Any] = {"type": "field_verify"}
        if getattr(args, "mines", None):
            sample["mines"] = [b for b in re.split(r"[\s,;|]+", args.mines) if b]
        if getattr(args, "obstacles", None):
            sample["obstacles"] = [b for b in re.split(r"[\s,;|]+", args.obstacles) if b]
        core.ingest_sample(sample, source="cli")


def cmd_contract() -> int:
    print(f"=== dzlink_supervisor {__version__} —— 空地链路契约 ===\n")
    print("话题与 QoS（★ 对着车侧订阅端设死，改错＝静默收不到）:")
    print(f"  {TOPIC_ROUTE:24} std_msgs/String  RELIABLE    /VOLATILE/depth10  ← 车 path_executor")
    print(f"  {TOPIC_TO_CAR:24} std_msgs/String  BEST_EFFORT /VOLATILE/depth10  ← 车 DroneTask")
    print(f"  {TOPIC_FROM_CAR:24} std_msgs/String  BEST_EFFORT                  ← 车往这里发")
    print(f"  {TOPIC_PERCEPTION:24} std_msgs/String  BEST_EFFORT（可选）        ← 感知管线可发这里\n")
    print("情报 JSON（→ /dzlink/drone_to_car）:")
    print('  {"type":"heartbeat","ts":...,"seq":N}')
    print('  {"type":"bridge_passable","half":"lower|upper","block":"K9","confidence":0.9,"ts":...,"seq":N}')
    print('  {"type":"target_zone","blocks":["G10","H11"],"ts":...,"seq":N}')
    print('  {"type":"field_verify","mines":["A3"],"obstacles":["K9"],"ts":...,"seq":N}')
    print("  带 seq → 车回 {\"type\":\"ack\",\"of\":N} 到 /dzlink/car_to_drone（收到的硬证据）\n")
    print("路线命令（→ /nav/waypoint，裸文本）:")
    print("  裸 token 列表  N10 M10 L10 L9 K9 …   ← 最常用")
    print("  G10*           该点停车打靶")
    print("  R:F11          R: 前缀＝倒车（顺序：R: 在前、* 在后）")
    print("  N5,y / N5,x    朝向后缀，只允许挂最后一个 token")
    print("  SHOOT@G11      指令非坐标：车侧算候选链整链插入")
    print("  reset:N10,y    复位信念位姿 + 清路线 → IDLE")
    print("  load:文件名    读路径文件并出发")
    print("  分隔符：空格 / 分号 / 竖线\n")
    print("时序约束（车侧契约，读源码核实；真值取自车上 config/robot.yaml）:")
    print(f"  情报过期           {CAR_CONTRACT['info_stale_s']:g}s  → 车侧当无情报")
    print(f"  Mission 等情报     {CAR_CONTRACT['mission_wait_s']:g}s  → 等不到用默认 "
          f"{CAR_CONTRACT['bridge_default_lower']}(下)/{CAR_CONTRACT['bridge_default_upper']}(上)")
    print(f"  自动出发延时       {CAR_CONTRACT['auto_start_delay_s']:g}s（auto_start=true 时）")
    print(f"  心跳判离线         {CAR_CONTRACT['heartbeat_offline_s']:g}s 无心跳")
    print(f"  车状态上报         {CAR_CONTRACT['car_state_period_s']:g}s/次 car_state")
    print(f"\n  ⇒ 等情报窗口只有 {CAR_CONTRACT['mission_wait_s']:g}s，情报**必须提前备好**"
          f"（`--cmd-file` 预置 / 一次性 `once`），不能指望发车后再抢窗口；")
    print(f"    故本节点 bridge 默认每 {2.0:g}s 重发一次（且内容一变立即重发）。\n")
    print("域号：唯一真相源 = 无人机 ~/dzlink_tools/dzlink.env ↔ 车 ~/dzrobot/dzros.env")
    print(f"  本机当前 ROS_DOMAIN_ID = {os.environ.get('ROS_DOMAIN_ID', '(未设)')}")
    return 0


def cmd_selftest() -> int:
    """不依赖 pytest 的自检（无人机上也能跑：python3 dzlink_supervisor.py selftest）。"""
    failures: List[str] = []

    def check(name: str, cond: bool, detail: str = "") -> None:
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
        if not cond:
            failures.append(name)

    print(f"=== dzlink_supervisor {__version__} selftest（纯逻辑，无需 ROS）===")

    # 域号
    info = bootstrap_domain("/nonexistent/dzlink.env", argv=[], quiet=True)
    check("域号兜底=118", os.environ.get("ROS_DOMAIN_ID") == "118", f"source={info['source']}")

    core = SupervisorCore(CoreConfig(heartbeat_period_s=1.0, bridge_period_s=2.0), JsonlLogger(None))
    evs = core.tick(now=1000.0)
    check("首次 tick 发心跳", any(e.kind == "heartbeat" for e in evs), f"{len(evs)} 条")
    evs2 = core.tick(now=1000.1)
    check("0.1s 后不重复发心跳", len(evs2) == 0)
    evs3 = core.tick(now=1001.0)
    check("1.0s 后重发心跳", any(e.kind == "heartbeat" for e in evs3))

    up = core.ingest_sample({"type": "bridge", "direction": "left", "hole": "K9"}, now=2000.0)
    check("方向词+字段名宽容解析", up == ["bridge_passable:lower"], str(up))
    evs4 = core.tick(now=2000.0)
    br = [e for e in evs4 if e.label == "bridge_passable"]
    check("瞬发桥洞情报且带 seq", bool(br) and '"half":"lower"' in br[0].text
          and '"block":"K9"' in br[0].text, br[0].text if br else "")

    evs5 = core.tick(now=2000.2)
    check("桥洞周期内不重发", not [e for e in evs5 if e.label == "bridge_passable"])

    before = len(core.tick(now=2030.0))
    core.ingest_sample({"type": "bridge_passable", "half": "upper", "block": "K4"}, now=2031.0)
    evs6 = core.tick(now=2031.0)
    check("内容变化立即重发", any(e.label == "bridge_passable" and "upper" in e.text for e in evs6),
          f"前一次 tick {before} 条")
    # ★ 回归：lower/upper 必须是两条独立槽，后到的 upper 不得覆盖 lower
    core.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=2041.0)
    core.ingest_sample({"type": "bridge_passable", "half": "upper", "block": "K5"}, now=2041.5)
    both = [e for e in core.tick(now=2041.6) if e.label == "bridge_passable"]
    halves = sorted(json.loads(e.text)["half"] for e in both)
    check("lower/upper 两半场互不覆盖", halves == ["lower", "upper"], str(halves))

    core.ingest_sample({"type": "target_zone", "blocks": ["g10", "H11", "g10"]}, now=2100.0)
    evs7 = core.tick(now=2100.0)
    tg = [e for e in evs7 if e.label == "target_zone"]
    check("靶区去重+大写化", bool(tg) and '"blocks":["G10","H11"]' in tg[0].text,
          tg[0].text if tg else "")
    evs8 = core.tick(now=2100.5)
    check("靶区只在变化时发", not [e for e in evs8 if e.label == "target_zone"])

    core.ingest_sample({"type": "field_verify", "mines": "A3,B4", "obstacles": ["k9"]}, now=2200.0)
    evs9 = core.tick(now=2200.0)
    fv = [e for e in evs9 if e.label == "field_verify"]
    check("雷障解析", bool(fv) and '"mines":["A3","B4"]' in fv[0].text and '"obstacles":["K9"]' in fv[0].text,
          fv[0].text if fv else "")

    check("畸形样本被拒", core.ingest_sample({"foo": 1}, now=2300.0) == []
          and core.stats.samples_rejected >= 1)

    ok, probs = validate_route("N10 M10 L10 L9 K9 J9")
    check("合法路线通过", ok, str(probs))
    ok2, probs2 = validate_route("G10* F11 N5,y")
    check("带 * 与朝向后缀的路线通过", ok2, str(probs2))
    ok3, probs3 = validate_route("N5,y M10")
    check("朝向后缀不在末尾被拒", not ok3, str(probs3))
    ok4, probs4 = validate_route("")
    check("空路线被拒", not ok4)
    ok5, probs5 = validate_route("reset:N10,y")
    check("reset 通过", ok5, str(probs5))
    ok6, _ = validate_route("reset:ZZ9")
    check("非法 reset 被拒", not ok6)
    check("格名抽取", route_blocks("R:F11 G10* N5,y") == ["F11", "G10", "N5"])

    m = core.ingest_car_raw('{"type":"ack","of":3,"ts":1.0}', now=3000.0)
    check("ack 解析", m.kind == "ack" and m.fields.get("of") == 3)
    m2 = core.ingest_car_raw('{"type":"car_state","block":"K9","yaw_deg":180.0,"phase":"DRIVE"}',
                             now=3000.1)
    check("car_state 解析", m2.kind == "car_state" and m2.fields.get("block") == "K9")
    m3 = core.ingest_car_raw("not json", now=3000.2)
    check("非 JSON 降级不崩", m3.kind == "non-json")

    st = core.status(now=3000.3)
    check("status 有 intel/stats", "intel" in st and st["stats"]["car_msgs"] == 3)

    # 时效
    core2 = SupervisorCore(CoreConfig(info_ttl_s=5.0), JsonlLogger(None))
    core2.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=10.0)
    core2.tick(now=10.0)
    evs_old = core2.tick(now=20.0)
    check("过期情报不再下发", not [e for e in evs_old if e.kind == "intel"],
          f"{[e.label for e in evs_old]}")

    print(f"\n结果：{'全部通过' if not failures else 'FAILED: ' + ', '.join(failures)}")
    return 0 if not failures else 1


def cmd_dry_run(args) -> int:
    core = _core_from_args(args)
    print(f"[dry-run] 域={os.environ.get('ROS_DOMAIN_ID')} 心跳={1.0 / max(0.05, args.heartbeat_hz):.2f}s "
          f"bridge周期={args.bridge_period}s ttl={args.info_ttl}s repeat={args.repeat}")
    _add_samples_from_args(core, args)
    if getattr(args, "route", None):
        core.submit_route(args.route, repeat=getattr(args, "route_repeat", 3))
    if getattr(args, "reset", None):
        block, _, yaw = args.reset.partition(",")
        core.submit_reset(block.strip(), yaw.strip() or "y")
    evs = core.tick(now=time.time())
    if not evs:
        print("[dry-run] 没有任何要发的东西（没给 --bridge/--target/--mines/--obstacles/--route）")
        return 0
    for ev in evs:
        print(f"  → {ev.topic:24} qos={ev.qos:12} x{ev.repeat}  {ev.text}")
    print(f"[dry-run] 共 {len(evs)} 类事件（不连 ROS，不发任何东西）")
    return 0


def _run_ros(args, core: SupervisorCore, mode: str) -> int:
    import rclpy
    from std_msgs.msg import String  # noqa: F401 - 提前失败，给出清晰报错

    rclpy.init()
    node = SupervisorNode(core, perception_topic=getattr(args, "perception_topic", None),
                          status_period_s=getattr(args, "status_period", 30.0))
    rc = 0
    # ★ 必须在 try 之前初始化：`listen` 分支在 try 内 return，`finally` 却要引用它。
    #   （t4 真机缺陷 D1：listen 跑完正常、退出时却 UnboundLocalError: udp_src、退出码非 0）
    #   任何"在 try 之后才赋值、finally 里使用"的局部变量都有同一个坑，新增时请注意。
    udp_src = None
    try:
        if mode == "listen":
            end = time.time() + args.duration
            while time.time() < end:
                rclpy.spin_once(node.node, timeout_sec=0.2)
            st = core.status()
            print(f"[listen] 收到 {st['stats']['car_msgs']} 条（ack {st['stats']['acks']} / "
                  f"car_state {st['stats']['car_states']}）")
            rc = 0 if st["stats"]["car_msgs"] else 3
            return rc

        # 启动时的动作
        if getattr(args, "reset", None):
            text = args.reset
            block, _, yaw = text.partition(",")
            core.submit_reset(block.strip(), yaw.strip() or "y")
        if getattr(args, "route", None):
            core.submit_route(args.route)

        cmd_file = (CommandFile(args.cmd_file,
                               from_end=not bool(getattr(args, "cmd_from_start", False)))
                    if getattr(args, "cmd_file", None) else None)
        from_end = not bool(getattr(args, "perception_from_start", False))
        tailers: List[GlobTailer] = []
        if getattr(args, "perception_glob", None):
            tailers.append(GlobTailer(args.perception_glob, from_end=from_end))
        for f in (getattr(args, "perception_file", None) or []):
            tailers.append(GlobTailer(expand(f), from_end=from_end))
        if getattr(args, "perception_udp", None):
            udp_src = UdpJsonSource(getattr(args, "perception_udp_host", "0.0.0.0"),
                                    int(args.perception_udp))
            diag(f"[udp] 监听 JSON 样本 {udp_src.addr[0]}:{udp_src.addr[1]}（UDP）")

        stdin_lines: Deque[str] = deque()
        if getattr(args, "stdin", False):
            import threading

            def _reader():
                for line in sys.stdin:
                    if line.strip():
                        stdin_lines.append(line.strip())
            threading.Thread(target=_reader, daemon=True).start()
            diag("[stdin] 已开启命令输入（JSONL，一行一个；空行忽略）")

        print(f"[run] 常驻启动：心跳 {1.0 / max(0.05, args.heartbeat_hz):.2f}Hz / "
              f"bridge {args.bridge_period:g}s / ttl {args.info_ttl:g}s / "
              f"域 {os.environ.get('ROS_DOMAIN_ID')}")
        print(f"[run] 日志: {core.log.path or '(未落盘)'}")

        loop_period = 1.0 / max(1.0, float(getattr(args, "loop_hz", 20.0)))
        t_end = time.time() + args.duration if getattr(args, "duration", 0) else None
        last_stale_note = 0.0
        while not core.stopping:
            now = time.time()
            if t_end is not None and now >= t_end:
                print("[run] 到达 --duration，退出")
                break

            # 输入 1：文件尾随
            for tl in tailers:
                for src, line in tl.poll():
                    obj, err = sample_from_file_line(line)
                    if obj is None:
                        core.stats.samples_rejected += 1
                        core.log.log("sample_bad_line", source=src, error=err, line=line[:400])
                        diag(f"[tail] 跳过（{err}）: {line[:120]}")
                        continue
                    core.ingest_sample(obj, source=os.path.basename(src))

            # 输入 3：UDP JSON
            if udp_src is not None:
                for src, obj in udp_src.poll():
                    core.ingest_sample(obj, source=src)

            # 输入 2：命令文件 / stdin
            if cmd_file is not None:
                cmd_file.poll(core)
            while stdin_lines:
                line = stdin_lines.popleft()
                try:
                    cmd = json.loads(line)
                    if isinstance(cmd, dict):
                        CommandFile.execute(cmd, core)
                except Exception as exc:  # noqa: BLE001
                    diag(f"[stdin] 跳过非法行（{exc}）: {line[:120]}")

            # 决策 + 发送
            events = core.tick(now)
            if events:
                unmatched = node.dispatch(events)
                if unmatched:
                    rc = 3

            node.maybe_status(now)

            # 每 10s 汇总一次「多久没收到车消息」，方便现场判断车是否在跑
            if now - last_stale_note > 10.0:
                last_stale_note = now
                snap = core.stats.snapshot(now)
                ago = snap["car_last_ago_s"]
                if core.stats.car_msgs == 0 and snap["uptime_s"] > 10:
                    diag(f"[health] 至今 0 条车回传 —— 车侧栈可能没跑（域 "
                         f"{os.environ.get('ROS_DOMAIN_ID')}）")
                elif ago is not None and ago > 5.0:
                    diag(f"[health] ⚠️ 已 {ago:.0f}s 没收到车回传（车可能停了/链路掉了）")

            node.spin_step(loop_period)

        core.log_status("final_status")
        st = core.status()
        print(f"[run] 结束。统计: {json.dumps(st['stats'], ensure_ascii=False)}")
        print(f"[run] ack 台账: {st['ack_ledger']}")
        return rc
    finally:
        if udp_src is not None:
            udp_src.close()
        node.destroy()
        core.log.close()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


def main(argv: Optional[Sequence[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    ap = build_argparser()
    args = ap.parse_args(argv)

    # 域号必须在 rclpy.init 之前落定
    bootstrap_domain(args.env_file, argv=argv,
                     quiet=(args.cmd == "selftest"))

    if not args.cmd or args.cmd == "contract":
        return cmd_contract()
    if args.cmd == "selftest":
        return cmd_selftest()
    if args.cmd == "dry-run":
        return cmd_dry_run(args)

    if args.cmd == "listen":
        core = _core_from_args(args)
        return _run_ros(args, core, "listen")

    if args.cmd == "once":
        core = _core_from_args(args)
        _add_samples_from_args(core, args)
        if not getattr(args, "no_heartbeat", False):
            core.submit_heartbeat()
        events = core.tick(now=time.time())
        if args.dry_run:
            for ev in events:
                print(f"  → {ev.topic:24} qos={ev.qos:12} x{ev.repeat}  {ev.text}")
            return 0
        return _run_ros_once(args, core, events)

    if args.cmd in ("route", "reset"):
        core = _core_from_args(args)
        try:
            if args.cmd == "route":
                core.submit_route(args.route)
            else:
                block, _, yaw = args.reset.partition(",")
                core.submit_reset(block.strip(), yaw.strip() or "y")
        except ValueError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 1
        events = core.tick(now=time.time())
        if args.dry_run:
            for ev in events:
                if ev.kind == "route":
                    print(f"  → {ev.topic} qos={ev.qos} x{ev.repeat}  {ev.text}")
            return 0
        return _run_ros_once(args, core, events, only_route=True)

    if args.cmd == "run":
        core = _core_from_args(args)
        if args.dry_run:
            return cmd_dry_run(args)
        # ★ t4 真机缺陷 D2：run 子命令声明了 --bridge/--target/--mines/--obstacles
        #   （见 build_argparser 的 run 分支），但这里原先没有调用注入函数 →
        #   argparse 照收、真跑 intel_emitted=0、零提示（"以为发了其实没发"）。
        #   注意必须放在 dry_run 之后：cmd_dry_run 内部自己会注入一次，不能重复。
        _add_samples_from_args(core, args)
        return _run_ros(args, core, "run")

    ap.print_help()
    return 0


def _run_ros_once(args, core: SupervisorCore, events: List[OutEvent],
                  only_route: bool = False) -> int:
    """一次性发送（不常驻）：发完就退。"""
    import rclpy
    from std_msgs.msg import String  # noqa: F401

    rclpy.init()
    node = SupervisorNode(core)
    try:
        evs = [e for e in events if e.kind == "route"] if only_route else events
        if not evs:
            print("[once] 没有要发的事件（检查参数）")
            return 1
        unmatched = node.dispatch(evs)
        st = core.status()
        print(f"[once] 发完。ack 台账: {st['ack_ledger']}")
        if unmatched:
            print(f"[once] ⚠️ 有 {unmatched} 类事件没有任何订阅者 → 退出码 3")
            return 3
        # 给车一点时间回 ack（BEST_EFFORT + 4 次重复发送）
        print("[once] 等 2s 看车有没有回 ack…")
        end = time.time() + 2.0
        while time.time() < end:
            rclpy.spin_once(node.node, timeout_sec=0.1)
        st2 = core.status()
        al = st2["ack_ledger"]
        if al["intel_acked"]:
            print(f"[once] ✅ 车回了情报 ack：{al['recent_acks']}")
        elif al["hb_acked"]:
            print(f"[once] ✅ 车回了心跳 ack（共 {al['hb_acked']} 条证明回传方向通），"
                  f"但情报本身没有 ack —— 车侧旧核不吃 JSON 情报时就长这样")
        elif al["suspicious_mismatch"]:
            print(f"[once] ⚠️ 收到了 ack 但**一条都对不上我们发过的 seq** → 十有八九是"
                  f"域号不一致（当前域 {os.environ.get('ROS_DOMAIN_ID')}）")
        else:
            print("[once] 没收到任何车回执（车没起栈 / 车侧旧核不吃 JSON 情报 / 回传被拦）")
        return 0
    finally:
        node.destroy()
        core.log.close()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


def _install_signal_handlers() -> None:
    def _handler(signum, frame):  # noqa: ARG001
        diag(f"\n[signal] 收到 {signum}，正在干净退出（刷日志、关节点）…")
        raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _handler)
        except (ValueError, OSError):
            pass


if __name__ == "__main__":
    _install_signal_handlers()
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("[main] 中断退出")
        raise SystemExit(130)
    except (ModuleNotFoundError, ImportError) as exc:
        print(f"ERROR: 没有可用的 ROS 2 Python 环境（{exc}）", file=sys.stderr)
        print("       先执行: source /opt/ros/humble/setup.bash", file=sys.stderr)
        raise SystemExit(2)
