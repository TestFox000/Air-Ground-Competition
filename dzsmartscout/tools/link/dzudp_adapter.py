# -*- coding: utf-8 -*-
r"""DL1 适配器（生产件 · AGC-17 合入）：机端 UDP:50000 二进制 → 既有 JSON 情报链（/dzlink/drone_to_car）。

## 分层（每层都复用现成件，不复制协议）

| 层 | 做什么 | 复用什么（不复制） |
|---|---|---|
| ① 线上字节 → 结构化帧 | 魔数/版本/长度/CRC16/后缀 全校验 | 教员 udp_bridge.py 的 crc16 / parse_intel / parse_pose / parse_route（**import，不复制**） |
| ② 结构化帧 → 格名 | 机端 MapLocalizer 帧 → 列/行 + 残差 | 常数 0.065/0.295/0.35 与 field_frame.world_to_cell 同式（机端源：ag_integrate/field_frame.py:40-70） |
| ③ 格名 → JSON 载荷 | 既有五种报文的构造与校验 | 仓内 dzsmartscout/tools/perception/protocol.py 的 build_* / encode（车侧契约镜像同源） |
| ④ 没有 JSON 落点的字段 | 只记录 | 本模块 records（不进话题） |

## 三条硬边界（本模块的"不许"）

1. **不发布路线**：0x4003 只落 records["route"]。本文件源码里**不出现**车侧路线话题的
   字符串常量——连名字都没有，就不可能有发布路径；单测用"源码不含该串 + 路线帧产出零报文
   + 把解析核的 bridge() 换成炸弹仍能跑"三条一起钉死。
2. **缺失保持缺失**：二进制没有 coverage / slice / scan_id / 完整布局 / 逐目标置信度，
   ⇒ 报文里**不补** coverage、**不伪造** field_layout。空类别默认**整键不发**
   （见 include_empty_classes 的取舍说明），因为"这一帧没看到"≠"场上没有"。
3. **不合成心跳**：链路上行活由谁声明是 DL2 的决策，本层默认不发 heartbeat。

## 与机端现行 JSON 生产者的关系（单写者）

机端 dzbrain_bridge.py 已经会发同样的四种报文（_publish_intel，行 275-315）。
本适配器是**同一份语义的第二实现**，因此两者**不得同时常驻**（两个生产者写同一话题，
seq 互相插队、DroneTask 的状态被来回覆盖）。默认值与它逐项对齐（见 AdapterConfig），
差异只在「空类别是否整键不发」这一处，且已做成显式开关。

## 运行时约束：单写者（2026-10-24 Lead 裁决③）

本适配器是 `/dzlink/drone_to_car` 的**唯一生产者**：它**替代**机端
`dzbrain_bridge.py` 的情报出口，不与之并存。两者同时在线 = **双发布**
（同一话题两个 writer，seq 互相插队、车侧 DroneTask 状态被来回覆盖，且事后看不出是谁写的）。

部署顺序（缺一步不许起）：

    python dzudp_adapter.py --selfcheck     # ① 只**诊断**不占位：红了（退出码 2）先停掉另一个生产者
    python dzudp_adapter.py --run           # ② 占位：acquire claim + 心跳刷新 ts + finally release
    # ③ 再起监听进程（DL2：UDP:50000 → 本层 → /dzlink/drone_to_car）

自检三条通道，任一判红即 red：

| 通道 | 判什么 | 不可用时 |
|---|---|---|
| second_instance | 本机 claim 文件被另一个**存活** pid 占用（**存活即占用，与时间无关**） | —— |
| ros_graph | 该话题上出现了**不属于本进程**的 publisher（一个也算） | 记 unknown，**不判红** |
| foreign_process | 同机进程表里还跑着 `dzbrain_bridge` | 记 unknown，**不判红** |

unknown 不算红：离线 Windows 没有 rclpy 也没有 `/proc`，那是正常态，不能因此挡启动。

### 判据为什么是这三条（AGC-20 评审 F1–F3 的修正，别改回去）

* **F1 占位必须真的写 claim**：`--selfcheck` 只读不写 ⇒ 两个实例的 claim 通道都是"空闲"，
  ROS 图又只看到"已经在发的那个" ⇒ 两次 selfcheck 都 ok。所以新增 `--run`：acquire + 心跳 + release。
* **F2 存活就是占用**：旧判据 `alive and age<=600` 会把"跑满 10 分钟"的实例判成可接管
  ⇒ 合法抢走活人的 claim。`age` 只用来描述"死进程残留多久了"，不参与判红。
* **F3 一个外来 publisher 也算红**：`len(uniq)>1` 只在"两人同时在发"时成立，而 preflight 发生在
  自己还没建 publisher 之前 ⇒ 外来者独自在发时必须判红。判据改为"图里有名字不属于本进程"。

## 用法

    a = BinaryIntelAdapter()
    res = a.adapt(datagram)          # 单帧；坏帧不抛异常，kind="bad"
    res.messages                     # 待发布 JSON 载荷（只可能有 /dzlink/drone_to_car）
    res.encoded                      # 对应 std_msgs/String.data 文本
    res.records                      # 只记录的事实（含 0x4003 路线、0x4002 位姿）
"""

from __future__ import annotations

import importlib.util
import json
import math
import os
import re
import socket
import struct
import sys
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Sequence

__all__ = [
    "X0", "Y0", "CELL_M", "COLS", "ROWS", "LETTERS", "MAX_CELL_RESIDUAL",
    "MID_BOUNDARY", "CLASS_TABLE", "ClassRule", "JSON_ROLES", "DRONE_TO_CAR_TOPIC",
    "CellOutOfField", "CellResidualTooLarge",
    "Cell", "cell_to_world", "world_to_cell",
    "AdapterConfig", "FrameResult", "BinaryIntelAdapter", "frame_kind",
    "load_parse_core", "load_protocol", "adapter_diagnostics",
    "DoublePublishError", "SingleWriterGuard", "KNOWN_FOREIGN_PROCESS_NAMES",
    "RUNTIME_CONSTRAINT", "SELF_NODE_NAME",
    "main",
]

# --------------------------------------------------------------------------- #
#  机端帧常数（唯一真源：机端 MapLocalizer / ag_integrate/field_frame.py）
# --------------------------------------------------------------------------- #
#: MapLocalizer.world() 的原点（米）：world_x = X0 + col*0.35
X0: float = 0.065
#: world_y = Y0 + (ROWS-1-row)*0.35
Y0: float = 0.295
#: 格边（米）
CELL_M: float = 0.35
COLS: int = 14          # A..N
ROWS: int = 11          # 1..11
LETTERS: str = "ABCDEFGHIJKLMN"
#: 离格心残差上限（以格为单位）——与 field_frame.MAX_CELL_RESIDUAL 同值。
#: 0.5 会让这条检查永远不触发（round 天然保证 ≤0.5），0.35≈12cm 才真的会拒收。
MAX_CELL_RESIDUAL: float = 0.35
#: 半场分界的 0 基行号：row >= 6（第 7 行起）为 lower，与机端 mission_router._MID_BOUNDARY 同值。
MID_BOUNDARY: int = 6

#: 本层唯一的输出话题（车侧契约 dzrobot_v2/src/dzbrain/dzbrain/io/contract.py:94-95：
#: `std_msgs/String`、depth 10、**BEST_EFFORT**）。QoS 配错会让车侧静默收不到，
#: 所以这里连 QoS 一起抄下来，不自造。
DRONE_TO_CAR_TOPIC: str = "/dzlink/drone_to_car"
DRONE_TO_CAR_QOS: dict = {"depth": 10, "reliability": "best_effort"}

#: 本层在 ROS 图里的节点名（自检探针也用它）。run 模式可用 own_node_names 覆盖成真实节点名。
SELF_NODE_NAME: str = "_dzudp_selfcheck"

#: 运行时约束（机器可读，卡面与 --diagnostics 都引这一条）：本层是唯一生产者。
RUNTIME_CONSTRAINT: str = (
    "运行时约束：禁止同 topic 双写 —— 本适配器是 /dzlink/drone_to_car 的唯一生产者，"
    "机端 dzbrain_bridge.py 必须停掉（替代，不并存）；启动前先跑 --selfcheck")

_LOWER, _UPPER = "lower", "upper"
#: 半场发送顺序，与机端 dzbrain_bridge._BRIDGE_HALVES 一致
_BRIDGE_HALVES: tuple[str, str] = (_LOWER, _UPPER)


# --------------------------------------------------------------------------- #
#  13 类映射表（机端现行，2026-10-16 起 D 方案）
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class ClassRule:
    """一类机端目标 → JSON 情报落点。

    role 取值见 JSON_ROLES：
      * "paper"       —— 落格后分流（普通格→雷；K 列→桥洞通行箭头）
      * "obstacle"    —— 进 field_verify.obstacles
      * "target"      —— 进 target_zone.blocks（红/蓝都报；协议里没有颜色字段）
      * "record_only" —— 二进制里有、JSON 没有对应字段 ⇒ **只记录**
    """

    name: str
    role: str
    note: str = ""


JSON_ROLES: tuple[str, ...] = ("paper", "obstacle", "target", "record_only")

#: 13 类的**唯一映射表**。键必须正好是 0..12（单测断言完备性）。
#: 顺序与机端 intel_protocol.ObjectType / 教员 TYPE_NAMES 逐项相同。
CLASS_TABLE: dict[int, ClassRule] = {
    0: ClassRule("PAPER", "paper",
                 "场地纸片：普通格=雷，K 列桥洞区=通行箭头（机端 world_model.py:120-127 同判据）"),
    1: ClassRule("CELL_LABEL", "record_only",
                 "地面格号=定位基础设施（每帧数百个），不进态势、无 JSON 落点"),
    2: ClassRule("CAR", "record_only",
                 "车位姿链；机→车**没有** car_pose 报文（车侧只有 car_state 反向），"
                 "要用得新增 type 或只做交叉校验"),
    3: ClassRule("TARGET_RED", "target",
                 "友方红靶（禁火）。target_zone 只有 blocks，没有红/蓝语义 ⇒ 颜色信息只留在 records"),
    4: ClassRule("TARGET_BLUE", "target",
                 "敌方蓝靶。同上，颜色信息只留在 records"),
    5: ClassRule("OBSTACLE", "obstacle", "障碍物 → field_verify.obstacles"),
    6: ClassRule("BUCKET_RED", "record_only",
                 "红桶（禁火语义）；JSON 无桶字段 ⇒ 只记录（若要当障碍用需新字段，别偷偷并进 obstacles）"),
    7: ClassRule("BUCKET_GREEN", "record_only", "绿桶；同上"),
    8: ClassRule("BUCKET_CYAN", "record_only", "青桶；同上"),
    9: ClassRule("CAR_START", "record_only",
                 "智能车起点。**不是** field_layout.start：布局还缺 known/bridges/version，禁反推"),
    10: ClassRule("CAR_END", "record_only", "智能车终点；同上"),
    11: ClassRule("UAV_START", "record_only", "无人机起点；同上"),
    12: ClassRule("UAV_END", "record_only", "无人机终点；同上"),
}


# --------------------------------------------------------------------------- #
#  落格：机端帧 → (列, 行) + 残差
# --------------------------------------------------------------------------- #
class CellOutOfField(ValueError):
    """点落在 14×11 场地之外（机端 field_frame.CellRef.validate 同语义）。"""


class CellResidualTooLarge(ValueError):
    """点在场内但离最近格心太远（> max_residual 格），落哪一格已接近抛硬币。"""


@dataclass(frozen=True)
class Cell:
    """0 基列/行 + 残差（单位：格）。name 是 K9 这种格名。"""

    col: int
    row: int
    residual_col: float = 0.0
    residual_row: float = 0.0

    @property
    def name(self) -> str:
        return f"{LETTERS[self.col]}{self.row + 1}"

    @property
    def half(self) -> str:
        """row >= MID_BOUNDARY → lower（第 7 行起），否则 upper。"""
        return _LOWER if self.row >= MID_BOUNDARY else _UPPER

    @property
    def max_residual(self) -> float:
        return max(self.residual_col, self.residual_row)

    def to_dict(self) -> dict:
        return {"cell": self.name, "col": self.col, "row": self.row, "half": self.half,
                "residual_cell": round(self.max_residual, 4)}


def cell_to_world(col: int, row: int) -> tuple[float, float]:
    """(列, 行) 0 基 → 机端世界坐标（米）。与 field_frame.cell_to_world 逐字同式。"""
    if not (0 <= col < COLS and 0 <= row < ROWS):
        raise CellOutOfField(f"cell out of field: col={col} row={row}")
    return X0 + col * CELL_M, Y0 + (ROWS - 1 - row) * CELL_M


def world_to_cell(x: float, y: float, *, max_residual: float = MAX_CELL_RESIDUAL) -> Cell:
    """机端世界坐标（米）→ 最近格；出界或残差过大抛 ValueError。

    ★ **不要**用车端 core/geometry.py 的公式反解：轴向与原点都不同
    （车端 x=(10.5-row)*0.35, y=(13.5-col)*0.35），用错的表现是整场错位且不报错。
    """
    xf, yf = float(x), float(y)
    if not (math.isfinite(xf) and math.isfinite(yf)):
        raise CellOutOfField(f"non-finite point: x={x!r} y={y!r}")
    col_f = (xf - X0) / CELL_M
    row_f = (ROWS - 1) - (yf - Y0) / CELL_M
    col, row = round(col_f), round(row_f)
    if not (0 <= col < COLS and 0 <= row < ROWS):
        raise CellOutOfField(
            f"point out of field: x={xf:.3f} y={yf:.3f} -> col={col} row={row}"
            f" (field is col 0..{COLS - 1}, row 0..{ROWS - 1})")
    d_col, d_row = abs(col_f - col), abs(row_f - row)
    if d_col > max_residual or d_row > max_residual:
        raise CellResidualTooLarge(
            f"point too far from a cell center: x={xf:.3f} y={yf:.3f}"
            f" -> col={col} row={row} residual=({d_col:.3f},{d_row:.3f})cell"
            f" ({max(d_col, d_row) * CELL_M * 100:.1f}cm) > {max_residual:.2f}cell")
    return Cell(int(col), int(row), d_col, d_row)


# --------------------------------------------------------------------------- #
#  复用件定位：解析核（教员 udp_bridge.py）与 JSON 构造器（仓内 protocol.py）
# --------------------------------------------------------------------------- #
_CORE_ATTRS = ("crc16", "parse_intel", "parse_pose", "parse_route",
               "MSG_INTEL", "MSG_POSE", "MSG_ROUTE", "MAGIC", "VERSION")


def _repo_root(start: Path | None = None) -> Path:
    """向上找含 dzsmartscout/tools/perception/protocol.py 的目录（最多 9 层）。"""
    here = (start or Path(__file__)).resolve()
    cands = [here.parent] + list(here.parents)
    for cand in cands[:9]:
        if (cand / "dzsmartscout" / "tools" / "perception" / "protocol.py").is_file():
            return cand
    env = os.environ.get("AGC_REPO_ROOT")
    if env and (Path(env) / "dzsmartscout" / "tools" / "perception" / "protocol.py").is_file():
        return Path(env)
    raise RuntimeError(
        "找不到仓根（含 dzsmartscout/tools/perception/protocol.py）；可设 AGC_REPO_ROOT")


def _core_candidates(root: Path) -> list[Path]:
    """解析核候选路径（按优先级，**仓内钉死快照第一位**）。

    1. `AGC_UDP_BRIDGE` 环境变量 —— 只作测试/迁移逃生口；
    2. `dzsmartscout/tools/link/dzudp_core.py` —— **生产路径**：教员原件的逐字节
       快照 + 文件头来源说明，sha256 由 tests/test_dzudp_core_provenance.py 钉死；
    3. `_tmp_session/sqc-20261024/tools/udp_bridge.py` —— 取证镜像（同 sha256）；
    4. 教员交接目录 —— 会搬走，只作兜底。
    """
    out: list[Path] = []
    env = os.environ.get("AGC_UDP_BRIDGE")
    if env:
        out.append(Path(env))
    out.append(root / "dzsmartscout" / "tools" / "link" / "dzudp_core.py")
    out.append(root / "_tmp_session" / "sqc-20261024" / "tools" / "udp_bridge.py")
    out.append(Path("D:/User/Desktop/教员交接/udp_bridge.py"))
    return out


def load_parse_core(path: Path | str | None = None) -> Any:
    """import 解析核（**不复制**其源码，也不调用它的 sniff/bridge 循环）。

    教员原件的 --publish 会写车侧路线话题，因此本函数只取 crc16/parse_* 与消息常量，
    **并断言** PUBLISH_ENABLED 仍是 False（诊断输出里也记这一项）。
    """
    root = _repo_root()
    tried: list[str] = []
    cands = [Path(path)] if path is not None else _core_candidates(root)
    for cand in cands:
        tried.append(str(cand))
        if not cand.is_file():
            continue
        spec = importlib.util.spec_from_file_location("agc_udp_parse_core", cand)
        if spec is None or spec.loader is None:      # pragma: no cover - 环境异常
            continue
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        missing = [a for a in _CORE_ATTRS if not hasattr(mod, a)]
        if missing:
            raise RuntimeError(f"解析核缺属性 {missing}：{cand}")
        if getattr(mod, "PUBLISH_ENABLED", False) is not False:
            raise RuntimeError(
                f"解析核 {cand} 的 PUBLISH_ENABLED 不是 False：本层只借解析，绝不借发布开关")
        return mod
    raise RuntimeError("找不到解析核 udp_bridge.py，试过：" + " | ".join(tried))


def load_protocol(root: Path | None = None) -> Any:
    """import 仓内既有 JSON 构造器 dzsmartscout.tools.perception.protocol。"""
    root = root or _repo_root()
    sp = str(root)
    if sp not in sys.path:
        sys.path.insert(0, sp)
    from dzsmartscout.tools.perception import protocol as _protocol   # noqa: PLC0415
    return _protocol


# --------------------------------------------------------------------------- #
#  运行期自检：单写者（同一话题只许一个生产者 · 2026-10-24 Lead 裁决③）
# --------------------------------------------------------------------------- #
class DoublePublishError(RuntimeError):
    """同一话题上出现第二个生产者（自检判红时抛出）。"""


#: 机上同语义的第二实现（它也在发同样四种报文）：与它同时在线就是双写。
KNOWN_FOREIGN_PROCESS_NAMES: tuple[str, ...] = ("dzbrain_bridge.py", "dzbrain_bridge")


def _pid_alive(pid: int) -> bool:
    """pid 是否还活着（跨平台、不依赖 psutil）。"""
    if pid <= 0:
        return False
    if pid == os.getpid():
        return True
    if os.name == "nt":                      # pragma: no cover - Windows 分支
        import ctypes
        from ctypes import wintypes

        k32 = ctypes.windll.kernel32
        handle = k32.OpenProcess(0x1000, False, int(pid))   # QUERY_LIMITED_INFORMATION
        if not handle:
            return False
        try:
            code = wintypes.DWORD()
            if not k32.GetExitCodeProcess(handle, ctypes.byref(code)):
                return False
            return int(code.value) == 259                   # STILL_ACTIVE
        finally:
            k32.CloseHandle(handle)
    try:
        os.kill(int(pid), 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _default_claim_path(topic: str) -> Path:
    """claim 文件默认落系统临时目录（按话题分文件，多个话题互不干扰）。"""
    slug = re.sub(r"[^A-Za-z0-9]+", "_", str(topic)).strip("_") or "dzlink"
    return Path(tempfile.gettempdir()) / f"dzudp_adapter.{slug}.owner"


def _ros_publisher_probe(topic: str) -> list[str]:
    """用 rclpy 读该话题的 publisher 身份（只读，不发布任何报文）。不可用则抛异常。"""
    import rclpy                              # noqa: PLC0415
    from rclpy.node import Node               # noqa: PLC0415

    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = Node("_dzudp_selfcheck", context=ctx)
    try:
        infos = node.get_publishers_info_by_topic(topic)
        return [f"{str(i.node_namespace).rstrip('/')}/{i.node_name}" for i in infos]
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)


def _proc_table() -> list[tuple[int, str]]:
    """Linux /proc → [(pid, cmdline)]；非 Linux 返回空表。"""
    root = Path("/proc")
    out: list[tuple[int, str]] = []
    if not root.is_dir():
        return out
    for ent in root.iterdir():
        if not ent.name.isdigit():
            continue
        try:
            raw = ent.joinpath("cmdline").read_bytes()
        except OSError:
            continue
        text = raw.replace(b"\x00", b" ").decode("utf-8", "replace").strip()
        if text:
            out.append((int(ent.name), text))
    return out


class SingleWriterGuard:
    """运行期自检：确认本层是 topic 的唯一生产者（替代，不并存）。

    三条通道各自给 ok / red / unknown，preflight() 汇总：
    任一 red ⇒ status="red"（assert_ok() 抛 DoublePublishError）；无 red 但有 unknown ⇒ "unknown"。
    **unknown 不判红**：离线 Windows 没有 rclpy 也没有 /proc，那是正常态，不能因此挡启动。
    """

    def __init__(self, topic: str = DRONE_TO_CAR_TOPIC, *,
                 claim_path: Path | str | None = None,
                 publisher_probe: Callable[[str], list[str]] | None = None,
                 proc_reader: Callable[[], list[tuple[int, str]]] | None = None,
                 own_node_names: Sequence[str] | None = None,
                 own_pid: int | None = None) -> None:
        self.topic = str(topic)
        self.claim_path = (Path(claim_path) if claim_path is not None
                           else _default_claim_path(self.topic))
        self.publisher_probe = publisher_probe
        self.proc_reader = proc_reader
        #: ROS 图里算"自己"的节点名集合（F3：不在这个集合里的 publisher = 外来者 = 红）
        self.own_node_names = tuple(own_node_names or (SELF_NODE_NAME,))
        self.own_pid = int(own_pid if own_pid is not None else os.getpid())

    # ---------------- 三条通道 ----------------
    def _channel_claim(self) -> dict:
        rec = self.read_claim()
        if rec is None:
            return {"name": "second_instance", "status": "ok",
                    "detail": "claim 空闲（%s）" % self.claim_path}
        pid = int(rec.get("pid") or 0)
        if pid == self.own_pid:
            return {"name": "second_instance", "status": "ok",
                    "detail": "claim 由本进程持有（pid=%d）" % pid}
        age = time.time() - float(rec.get("ts") or 0.0)
        alive = _pid_alive(pid)
        if alive:
            # ★ F2：存活就是占用，**与时间无关**。旧判据（alive and age<=600）会把
            #   跑满 10 分钟的实例判成"可接管"⇒ acquire 抢走活人的 claim ⇒ 双写。
            return {"name": "second_instance", "status": "red",
                    "detail": "claim %s 被存活进程 pid=%d 占用（topic=%s，心跳 %.0fs 前）"
                              % (self.claim_path, pid, rec.get("topic"), age)}
        return {"name": "second_instance", "status": "ok",
                "detail": "死进程 pid=%d 的残留 claim（%.0fs 前写入）可清理" % (pid, age)}

    def _channel_ros(self) -> dict:
        probe = self.publisher_probe or _ros_publisher_probe
        try:
            names = list(probe(self.topic))
        except Exception as exc:                       # noqa: BLE001
            return {"name": "ros_graph", "status": "unknown",
                    "detail": "ROS 探针不可用（%s: %s）" % (type(exc).__name__, exc)}
        uniq = sorted(set(names))
        own = set(self.own_node_names)
        foreign = [n for n in uniq if n not in own]
        if foreign:
            # ★ F3：旧判据 len(uniq)>1 只在"两个都在发"时成立；preflight 发生在自己
            #   还没建 publisher 之前 ⇒ 外来者独自在发（uniq==1）会被漏判。改成看"名字是否属于本进程"。
            return {"name": "ros_graph", "status": "red",
                    "detail": "%s 上有非本进程的 publisher：%s（全量 %s）"
                              % (self.topic, foreign, uniq)}
        return {"name": "ros_graph", "status": "ok",
                "detail": "publishers=%s（仅本进程或无人）" % (uniq,)}

    def _channel_proc(self) -> dict:
        if self.proc_reader is not None:
            try:
                procs = list(self.proc_reader())
            except Exception as exc:                   # noqa: BLE001
                return {"name": "foreign_process", "status": "unknown",
                        "detail": "进程表不可用（%s: %s）" % (type(exc).__name__, exc)}
        elif Path("/proc").is_dir():
            procs = _proc_table()
        else:
            return {"name": "foreign_process", "status": "unknown",
                    "detail": "无 /proc（非 Linux）：无法扫描同机第二实现"}
        hits = [(pid, cmd[:120]) for pid, cmd in procs
                if pid != self.own_pid
                and any(name in cmd for name in KNOWN_FOREIGN_PROCESS_NAMES)]
        if hits:
            return {"name": "foreign_process", "status": "red",
                    "detail": "同机还跑着同语义生产者：%s" % (hits[:3],)}
        return {"name": "foreign_process", "status": "ok",
                "detail": "未见 dzbrain_bridge"}

    # ---------------- 对外 ----------------
    def preflight(self) -> dict:
        """只读自检（不建 claim、不连网、不发布）。"""
        channels = [self._channel_claim(), self._channel_ros(), self._channel_proc()]
        red = [c["detail"] for c in channels if c["status"] == "red"]
        unknown = [c["detail"] for c in channels if c["status"] == "unknown"]
        status = "red" if red else ("unknown" if unknown else "ok")
        return {"status": status, "ok": status != "red", "topic": self.topic,
                "claim_path": str(self.claim_path), "channels": channels,
                "mode": "diagnose-only（不占位；占位请用 --run）",
                "red": red, "unknown": unknown,
                "rule": "同一 topic 只许一个生产者：red ⇒ 不许启动，先停掉另一个再起"}

    def assert_ok(self) -> dict:
        """preflight + 判红即抛 DoublePublishError（入口脚本起监听前调用）。"""
        rep = self.preflight()
        if rep["status"] == "red":
            raise DoublePublishError(
                "%s 双发布自检判红：%s" % (self.topic, " | ".join(rep["red"])))
        return rep

    # ---------------- claim 互斥（跨进程；可选） ----------------
    def read_claim(self) -> dict | None:
        try:
            return json.loads(self.claim_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None

    def acquire(self) -> dict:
        """抢占 claim（O_EXCL 原子创建）。已被存活进程占用 ⇒ DoublePublishError。"""
        rec = self.read_claim()
        if rec is not None and int(rec.get("pid") or 0) == self.own_pid:
            return {"claimed": True, "already": True, "path": str(self.claim_path)}
        payload = json.dumps({"pid": self.own_pid, "ts": round(time.time(), 3),
                              "topic": self.topic, "host": socket.gethostname()},
                             ensure_ascii=False, sort_keys=True) + "\n"
        self.claim_path.parent.mkdir(parents=True, exist_ok=True)
        for _ in range(2):
            try:
                fd = os.open(self.claim_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
            except FileExistsError:
                ch = self._channel_claim()
                if ch["status"] == "red":
                    raise DoublePublishError("claim 已被占用：%s" % ch["detail"]) from None
                try:
                    self.claim_path.unlink()
                except OSError:
                    pass
                continue
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                fh.write(payload)
            return {"claimed": True, "already": False, "path": str(self.claim_path)}
        raise DoublePublishError("claim 抢占失败：%s" % self.claim_path)

    def refresh(self) -> dict | None:
        """心跳：只刷新**自己** claim 的 ts（别人的不碰）。返回写入后的记录或 None。

        为什么需要（F2）：claim 里的 ts 若从不刷新，"存活 + 很久以前写的"就会被别人当成
        废弃 claim；刷新后 pid 存活本身已是判红依据，ts 只用于描述心跳新鲜度。
        """
        rec = self.read_claim()
        if rec is None or int(rec.get("pid") or 0) != self.own_pid:
            return None
        rec["ts"] = round(time.time(), 3)
        body = json.dumps(rec, ensure_ascii=False, sort_keys=True) + "\n"
        tmp = self.claim_path.with_name(self.claim_path.name + ".tmp")
        tmp.write_text(body, encoding="utf-8")
        os.replace(tmp, self.claim_path)          # 原子替换，读方不会看到半个文件
        return rec

    def release(self) -> None:
        """释放自己持有的 claim（别人的不动）。"""
        rec = self.read_claim()
        if rec is not None and int(rec.get("pid") or 0) == self.own_pid:
            try:
                self.claim_path.unlink()
            except OSError:
                pass


def single_writer_preflight(topic: str = DRONE_TO_CAR_TOPIC, **kwargs: Any) -> dict:
    """一行式自检（不抛异常；调用方看 status）。"""
    return SingleWriterGuard(topic, **kwargs).preflight()


# --------------------------------------------------------------------------- #
#  适配结果
# --------------------------------------------------------------------------- #
@dataclass
class FrameResult:
    """一帧的适配结果。

    * kind：intel / pose / route / bad
    * messages：**只**能发布到 /dzlink/drone_to_car 的 JSON 载荷（可能为空）
    * records：二进制里有、但话题上没有落点的事实（只记录）
    * rejected：本帧被拒收的单个目标（出界 / 残差过大），不拖累同帧其它目标
    * errors：整帧级错误（坏 CRC / 截断 / 未知类 / 未知 msg_id）
    """

    kind: str
    raw_len: int
    frame_ts_ms: int | None = None
    messages: list[dict] = field(default_factory=list)
    records: dict = field(default_factory=dict)
    rejected: list[dict] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.kind != "bad" and not self.errors

    @property
    def encoded(self) -> list[str]:
        """对应 std_msgs/String.data 的文本（与既有链路同一 encode 口径）。"""
        return [json.dumps(m, ensure_ascii=False, separators=(",", ":")) for m in self.messages]

    @property
    def topic(self) -> str | None:
        """本条结果该发到哪个话题；None = **一个字节都不许发**（route/pose/bad）。

        0x4003 路线帧与 0x4002 位姿帧都只有 records ⇒ 返回 None，调用方即使
        dump 也拿不到"往哪发"的答案 —— 这是"只记录不发布"的结构性保证。
        """
        return DRONE_TO_CAR_TOPIC if self.messages else None

    @property
    def message_types(self) -> list[str]:
        return [str(m.get("type")) for m in self.messages]

    def to_dict(self) -> dict:
        return {"kind": self.kind, "ok": self.ok, "raw_len": self.raw_len,
                "frame_ts_ms": self.frame_ts_ms,
                "message_types": self.message_types,
                "messages": self.messages, "encoded": self.encoded,
                "records": self.records, "rejected": self.rejected,
                "warnings": self.warnings, "errors": self.errors}


# --------------------------------------------------------------------------- #
#  配置
# --------------------------------------------------------------------------- #
@dataclass
class AdapterConfig:
    """适配策略。默认值逐项对齐机端现行生产者 dzbrain_bridge._publish_intel。"""

    #: 落格残差上限（格）
    max_cell_residual: float = MAX_CELL_RESIDUAL
    #: bridge_passable.confidence。二进制**没有**置信度 ⇒ 这是合成常数。
    #: 默认 1.0 = 机端现行值（dzbrain_bridge.py:298）；车侧只记录不门控
    #: （drone.py:817-822 只写 bridge_confidence + EV_BRIDGE_INFO），所以今天不改行为。
    bridge_confidence: float = 1.0
    #: 空类别是否也发空表。False（默认）= **整键不发**："这一帧没看到"不等于"场上没有"，
    #: 二进制没有 coverage 语义，发空表会被车侧 _diff 读成负证据。
    #: True = 复刻机端 dzbrain_bridge 的"两键恒带"行为（它假定快照是全场的）。
    include_empty_classes: bool = False
    #: 是否顺带发 heartbeat。默认 False：链路上行活由 DL2 决定，本层不替它声明。
    emit_heartbeat: bool = False

    def to_dict(self) -> dict:
        return {"max_cell_residual": self.max_cell_residual,
                "bridge_confidence": self.bridge_confidence,
                "include_empty_classes": self.include_empty_classes,
                "emit_heartbeat": self.emit_heartbeat}


# --------------------------------------------------------------------------- #
#  适配器
# --------------------------------------------------------------------------- #
def frame_kind(data: bytes) -> int | None:
    """只按魔数/版本读 msg_id（不做完整校验）；格式不对返回 None。"""
    if not isinstance(data, (bytes, bytearray, memoryview)):
        return None
    raw = bytes(data)
    if len(raw) < 15 or raw[0:4] != b"\xef\xef\xef\xef" or raw[4] != 0x10:
        return None
    return struct.unpack_from("<H", raw, 13)[0]


class BinaryIntelAdapter:
    """机端二进制 → 既有 JSON 情报链的薄适配层（纯离线、无 ROS 依赖）。"""

    def __init__(self, config: AdapterConfig | None = None, *,
                 parse_core_path: Path | str | None = None,
                 repo_root: Path | str | None = None) -> None:
        self.root = Path(repo_root) if repo_root is not None else _repo_root()
        self.config = config or AdapterConfig()
        self.core = load_parse_core(parse_core_path)
        self.protocol = load_protocol(self.root)
        self.stats: dict[str, int] = {
            "frames": 0, "intel": 0, "pose": 0, "route": 0, "bad": 0,
            "objects": 0, "rejected_objects": 0, "messages": 0,
        }

    # ---------------- 对外入口 ----------------
    def adapt(self, data: bytes, *, now: float | None = None,
              next_seq: Callable[[], int] | None = None) -> FrameResult:
        """适配一帧。坏帧**不抛异常**：返回 kind="bad" + errors。"""
        if isinstance(data, (bytes, bytearray, memoryview)):
            raw = bytes(data)
        else:
            self.stats["frames"] += 1
            return self._bad(0, "非字节/空包")
        self.stats["frames"] += 1
        if not raw:
            return self._bad(0, "非字节/空包")
        msg_id = frame_kind(raw)
        if msg_id is None:
            return self._bad(len(raw), "非机端帧（魔数/版本不符）")
        if msg_id == self.core.MSG_INTEL:
            return self._adapt_intel(raw, now, next_seq)
        if msg_id == self.core.MSG_POSE:
            return self._adapt_pose(raw)
        if msg_id == self.core.MSG_ROUTE:
            return self._adapt_route(raw)
        return self._bad(len(raw), f"未知 msg_id 0x{msg_id:04x}")

    # ---------------- 内部 ----------------
    def _bad(self, n: int, msg: str) -> FrameResult:
        self.stats["bad"] += 1
        return FrameResult(kind="bad", raw_len=n, errors=[msg])

    def _ts(self, now: float | None) -> float:
        return round(float(time.time() if now is None else now), 3)

    def _add_seq(self, msg: dict, next_seq: Callable[[], int] | None) -> dict:
        if next_seq is not None:
            msg["seq"] = int(next_seq())
        return msg

    def _adapt_intel(self, raw: bytes, now: float | None,
                     next_seq: Callable[[], int] | None) -> FrameResult:
        try:
            ts_ms, objs = self.core.parse_intel(raw)
        except ValueError as exc:
            return self._bad(len(raw), f"intel 解析失败：{exc}")
        self.stats["intel"] += 1
        self.stats["objects"] += len(objs)
        res = FrameResult(kind="intel", raw_len=len(raw), frame_ts_ms=int(ts_ms))
        ts = self._ts(now)

        mines: list[str] = []
        obstacles: list[str] = []
        targets: list[str] = []
        passable: dict[str, list[str]] = {h: [] for h in _BRIDGE_HALVES}
        detail: list[dict] = []

        for idx, (type_index, x, y, length, width) in enumerate(objs):
            rule = CLASS_TABLE.get(int(type_index))
            if rule is None:                      # 解析核已挡 0..12，这里只兜底
                res.rejected.append({"index": idx, "type": int(type_index),
                                     "reason": "unknown class"})
                self.stats["rejected_objects"] += 1
                continue
            item = {"index": idx, "type": int(type_index), "class": rule.name,
                    "x": round(float(x), 4), "y": round(float(y), 4),
                    "length": round(float(length), 4), "width": round(float(width), 4),
                    "role": rule.role}
            try:
                cell = world_to_cell(x, y, max_residual=self.config.max_cell_residual)
            except (CellOutOfField, CellResidualTooLarge) as exc:
                item["reject_reason"] = str(exc)
                res.rejected.append(item)
                self.stats["rejected_objects"] += 1
                continue
            item.update(cell.to_dict())
            detail.append(item)
            if rule.role == "paper":
                # K 列桥洞区纸片=通行箭头，其余格=雷（机端 world_model.py:120-127 同判据）
                if LETTERS[cell.col] == "K":
                    passable[cell.half].append(cell.name)
                else:
                    _append_unique(mines, cell.name)
            elif rule.role == "obstacle":
                _append_unique(obstacles, cell.name)
            elif rule.role == "target":
                _append_unique(targets, cell.name)
            # record_only：只留在 detail 里，不进任何 JSON 键

        # ---- 报文构造（顺序与机端 dzbrain_bridge._publish_intel 一致）----
        synthetic: list[str] = []
        for half in _BRIDGE_HALVES:
            blocks = sorted(set(passable[half]))
            if not blocks:
                continue
            if len(blocks) > 1:
                res.warnings.append(
                    f"{half} 半场有多个桥洞格 {blocks}：车侧每半场只存一格，只报 {blocks[0]}")
            msg = self.protocol.build_bridge_passable(
                half, blocks[0], confidence=self.config.bridge_confidence, ts=ts)
            if "bridge_passable.confidence" not in synthetic:
                synthetic.append("bridge_passable.confidence")
            res.messages.append(self._add_seq(msg, next_seq))

        targets = sorted(set(targets))
        if targets:
            res.messages.append(self._add_seq(
                self.protocol.build_target_zone(targets, ts=ts), next_seq))

        mines_s, objs_s = sorted(set(mines)), sorted(set(obstacles))
        if mines_s or objs_s or self.config.include_empty_classes:
            if self.config.include_empty_classes:
                fv = self.protocol.build_field_verify(mines=mines_s, obstacles=objs_s, ts=ts)
            else:
                # 空类别**整键不发**：build_field_verify 收到 [] 会写空表，而空表会被
                # 车侧 _diff 读成"本地有、无人机没有"的负证据。二进制没有 coverage，
                # "这一帧没看到" ≠ "场上没有" ⇒ 传 None 让键缺席。
                fv = self.protocol.build_field_verify(
                    mines=mines_s or None, obstacles=objs_s or None, ts=ts)
            res.messages.append(self._add_seq(fv, next_seq))

        if self.config.emit_heartbeat:
            res.messages.append(self._add_seq(
                self.protocol.build_heartbeat(ts=ts), next_seq))
            synthetic.append("heartbeat(合成)")

        res.records = {
            "kind": "intel",
            "frame_ts_ms": int(ts_ms),
            "n_objects": len(objs),
            "objects": detail,
            "classes_seen": sorted({d["class"] for d in detail}),
            "mines": mines_s, "obstacles": objs_s, "targets": targets,
            "bridge_passable": {h: sorted(set(passable[h])) for h in _BRIDGE_HALVES},
            "synthetic_fields": synthetic,
            "absent_in_binary": ["coverage", "slice", "scan_id", "field_layout",
                                 "per_object_confidence"],
        }
        self.stats["messages"] += len(res.messages)
        return res

    def _adapt_pose(self, raw: bytes) -> FrameResult:
        try:
            ts_ms, x, y, yaw = self.core.parse_pose(raw)
        except ValueError as exc:
            return self._bad(len(raw), f"pose 解析失败：{exc}")
        self.stats["pose"] += 1
        rec: dict = {"kind": "pose", "frame_ts_ms": int(ts_ms),
                     "x": round(float(x), 4), "y": round(float(y), 4),
                     "yaw_rad": round(float(yaw), 6),
                     "yaw_deg": round(math.degrees(float(yaw)), 2),
                     "json_landing": None,
                     "note": "机→车没有 car_pose 报文（缺类型）；只记录，供与 car_state 交叉校验"}
        try:
            rec["cell"] = world_to_cell(x, y).to_dict()
        except (CellOutOfField, CellResidualTooLarge) as exc:
            rec["cell"] = None
            rec["cell_reject_reason"] = str(exc)
        return FrameResult(kind="pose", raw_len=len(raw), frame_ts_ms=int(ts_ms),
                           records=rec)

    def _adapt_route(self, raw: bytes) -> FrameResult:
        try:
            ts_ms, reset_cmd, route = self.core.parse_route(raw)
        except ValueError as exc:
            return self._bad(len(raw), f"route 解析失败：{exc}")
        self.stats["route"] += 1
        tokens = [t for t in str(route).split() if t]
        rec = {"kind": "route", "frame_ts_ms": int(ts_ms),
               "reset": reset_cmd, "route": route, "tokens": tokens,
               "n_tokens": len(tokens),
               "json_landing": None,
               "publish": "none（只记录：本层没有、也不许有路线发布路径）"}
        res = FrameResult(kind="route", raw_len=len(raw), frame_ts_ms=int(ts_ms),
                          records=rec)
        if not tokens:
            res.warnings.append("route 帧载荷为空 token 串")
        return res

    # ---------------- 诊断 ----------------
    def diagnostics(self) -> dict:
        return {
            "repo_root": str(self.root),
            "parse_core": {
                "path": str(getattr(self.core, "__file__", "?")),
                "publish_enabled": bool(getattr(self.core, "PUBLISH_ENABLED", False)),
                "used_for": ["crc16", "parse_intel", "parse_pose", "parse_route"],
            },
            "protocol": {
                "path": str(getattr(self.protocol, "__file__", "?")),
                "builders": ["build_bridge_passable", "build_target_zone",
                             "build_field_verify", "build_heartbeat", "encode"],
            },
            "runtime_constraint": RUNTIME_CONSTRAINT,
            "publish": {"topic": DRONE_TO_CAR_TOPIC, "qos": dict(DRONE_TO_CAR_QOS),
                        "topics_total": 1,
                        "max_publishers": 1,
                        "note": "机端 dzbrain_bridge 与本层是同语义第二实现："
                                "**不得同时常驻**（两个生产者写同一话题，seq 互相插队）",
                        "route_frames": "只记录，无发布会话（0x4003 → records['route']）",
                        "selfcheck": "python dzudp_adapter.py --selfcheck"
                                     "（red ⇒ 退出码 2，先停掉另一个生产者再起）"},
            "config": self.config.to_dict(),
            "frame_constants": {"x0": X0, "y0": Y0, "cell_m": CELL_M,
                                "cols": COLS, "rows": ROWS,
                                "max_cell_residual": MAX_CELL_RESIDUAL,
                                "mid_boundary": MID_BOUNDARY},
            "class_table": {i: {"name": r.name, "role": r.role} for i, r in
                            sorted(CLASS_TABLE.items())},
            "stats": dict(self.stats),
        }


def adapter_diagnostics(**kwargs: Any) -> dict:
    """一次构造 + 诊断（供 CLI/测试打印）。"""
    return BinaryIntelAdapter(**kwargs).diagnostics()


def _append_unique(seq: list[str], item: str) -> None:
    if item not in seq:
        seq.append(item)


def _hold_claim(guard: SingleWriterGuard, *, heartbeat_s: float = 5.0,
                seconds: float = 0.0) -> dict:
    """F1 的 run 模式：preflight → acquire → 定时刷新 ts（心跳）→ finally release。

    本层**不含** UDP 监听循环（那是地面侧 DL2 的进程），这里只负责"占位 + 心跳"：
    让第二个实例的 preflight 一定能看到**存活**的 claim（而不是"空闲"）。
    seconds=0 表示一直占位到 Ctrl+C；测试用 --seconds 给它上界。
    """
    rep = guard.preflight()
    if rep["status"] == "red":
        guard.assert_ok()                     # 抛 DoublePublishError（带判红原因）
    claimed = guard.acquire()
    started = time.time()
    beats = 0
    hb = max(0.2, float(heartbeat_s))
    try:
        while True:
            time.sleep(min(hb, 1.0))
            guard.refresh()
            beats += 1
            if seconds and (time.time() - started) >= float(seconds):
                break
    except KeyboardInterrupt:
        pass
    finally:
        guard.release()
    return {"event": "run-finished", "topic": guard.topic,
            "claim": str(guard.claim_path), "pid": guard.own_pid,
            "claimed": claimed, "beats": beats,
            "elapsed_s": round(time.time() - started, 2)}


def main(argv: list[str] | None = None) -> int:
    """命令行入口：只碰 claim / 进程表 / ROS 图，**不发布任何报文**。

    子命令（互斥，按下面的顺序判定）：
      --run         占位：acquire claim + 心跳刷新 ts + finally release（部署入口挂这里）
      --selfcheck   只**诊断**不占位（读 claim / ROS 图 / 进程表），红 ⇒ 退出码 2
      --diagnostics 打印适配器诊断面（话题/QoS/映射表/帧常数）

    退出码：0 = ok / unknown（unknown 不算红，离线 Windows 的正常态）；2 = 双发布判红。
    """
    import argparse

    ap = argparse.ArgumentParser(
        prog="dzudp_adapter.py",
        description="DL1 适配器：单写者自检 / 占位 / 诊断（不发布、不连网、不起 ROS 发布者）")
    ap.add_argument("--run", action="store_true",
                    help="占位模式：acquire claim + 心跳 + finally release（不含 UDP 监听）")
    ap.add_argument("--selfcheck", action="store_true", help="只诊断不占位（红 ⇒ 退出码 2）")
    ap.add_argument("--diagnostics", action="store_true", help="打印适配器诊断后退出")
    ap.add_argument("--topic", default=DRONE_TO_CAR_TOPIC)
    ap.add_argument("--claim", default=None, help="claim 文件路径（默认系统临时目录）")
    ap.add_argument("--node-name", default=None,
                    help="ROS 图里算「自己」的节点名，逗号分隔（默认 %s）" % SELF_NODE_NAME)
    ap.add_argument("--seconds", type=float, default=0.0, help="--run 最长秒数（0=直到 Ctrl+C）")
    ap.add_argument("--heartbeat", type=float, default=5.0, help="--run 的 claim 心跳间隔秒")
    ap.add_argument("--no-ros", action="store_true", help="跳过 ROS 图探测（记 unknown）")
    ap.add_argument("--no-proc", action="store_true", help="跳过进程扫描（记 unknown）")
    args = ap.parse_args(argv)

    own_names = tuple(n.strip() for n in (args.node_name or SELF_NODE_NAME).split(",")
                      if n.strip())

    probe: Callable[[str], list[str]] | None = None
    if args.no_ros:
        def probe(_topic: str) -> list[str]:
            raise RuntimeError("--no-ros 已禁用 ROS 图探测")

    reader: Callable[[], list[tuple[int, str]]] | None = None
    if args.no_proc:
        def reader() -> list[tuple[int, str]]:
            raise RuntimeError("--no-proc 已禁用进程扫描")

    guard = SingleWriterGuard(args.topic, claim_path=args.claim,
                              publisher_probe=probe, proc_reader=reader,
                              own_node_names=own_names)

    if args.run:
        try:
            info = _hold_claim(guard, heartbeat_s=args.heartbeat, seconds=args.seconds)
        except DoublePublishError as exc:
            print(json.dumps({"event": "refused", "reason": str(exc)},
                             ensure_ascii=False, sort_keys=True))
            return 2
        print(json.dumps(info, ensure_ascii=False, sort_keys=True))
        return 0
    if args.selfcheck:
        rep = guard.preflight()
        print(json.dumps(rep, ensure_ascii=False, indent=2, sort_keys=True))
        return 2 if rep["status"] == "red" else 0
    if args.diagnostics:
        print(json.dumps(adapter_diagnostics(), ensure_ascii=False,
                         indent=2, sort_keys=True))
        return 0
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
