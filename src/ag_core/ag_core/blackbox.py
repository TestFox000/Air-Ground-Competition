"""黑匣子：把一次实跑过程落成**结构化事件流**，跑完能复盘。

## 为什么不能再靠 ROS 原生日志

车端 `~/.ros/log/*.log` 是非结构化文本。一局跑完想知道"情报到没到、为什么没发路线、
为什么没开火、车卡在哪"，只能人工 grep 一堆互相穿插的行；而且经 SSH 跑节点时
stdout 是管道 → C 层全缓冲 → 进程被 kill 时缓冲整体丢失（这个坑见
`tools/car_ros_logs.sh` 的注释），现场经常"日志像没打一样"。

本模块把每个**关键决策点**写成一行 JSON：

    {"ts":1758960000.12,"mono":123.4,"seq":3,"node":"car_node",
     "ev":"route.blocked","lvl":"warn","session":"20260927",
     "reason":"face_unknown","msg":"敌我尚未判决，暂不发车"}

再由 `tools/log_report.py` 读回，产出「概览 + 异常清单 + 时间轴」。

## 三条硬要求（决定了这里的实现方式）

1. **绝不能拖垮主流程** —— 写盘失败只累计 `errors` 并关掉自己，不向外抛异常。
   日志把节点搞崩，比没有日志更糟。
2. **进程被 kill 也不能丢** —— 默认每条 `flush`（`flush_every=1`），
   对齐上面"spdlog 缓冲丢失"那个坑。
3. **默认不写家目录** —— 落到 `AG_LOG_DIR` 或系统临时目录下的 `ag_logs/`，
   与项目"只用 /tmp、不动车端家目录"的约定一致。

## 用法（节点里三行）

    from ag_core.blackbox import BlackBox, attach

    self.declare_parameter("log_enabled", True)
    self.declare_parameter("log_dir", "")
    self.bb = BlackBox.for_node("car_node", log_dir=str(p("log_dir")) or None,
                                enabled=bool(p("log_enabled")))
    attach(self, self.bb)        # 顺便把 get_logger() 的文本也镜像进同一时间轴

之后：`self.bb.log(EV.ROUTE_PUBLISH, cmd=route, kind="route")`。
高频事件用 `self.bb.throttled("det", 1.0)` 限流，别让 30fps 把盘写满。
"""

from __future__ import annotations

import json
import os
import socket
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Optional, Tuple

__all__ = [
    "AIM_BLOCK_WHY",
    "BlackBox",
    "EV",
    "Event",
    "REASON",
    "attach",
    "collect_jsonl",
    "default_log_dir",
    "default_session",
    "find_log_dirs",
    "read_events",
    "session_start_index",
]


# ---------------------------------------------------------------- 事件名常量
class EV:
    """事件名常量表。

    复盘规则（`ag_core/log_diagnosis.py`）按这些名字取数，所以**不要在节点里
    手写字符串** —— 拼错一个字母报告里就静悄悄少一块，而且不会有任何报错。
    """

    # ---- 生命周期 ----
    SESSION_BEGIN = "session.begin"   # 一局开始（car_node 启动时写，复盘据此切局）
    NODE_READY = "node.ready"         # 任一节点启动（pid / host / 日志路径）
    NODE_ARGS = "node.args"           # 关键参数快照（复现问题要用）
    NODE_ERROR = "node.error"         # 节点内部异常
    ROS_LOG = "ros.log"               # get_logger() 文本日志的镜像

    # ---- 情报：空中 -> 车 ----
    LINK_TX = "link.tx"               # 机载端发出
    LINK_RX = "link.rx"               # 车端收到（含校验结果）
    LINK_DROP = "link.drop"           # 丢弃（坏帧 / 乱序 / 过期）
    INTEL_RECV = "intel.recv"         # car_node 收到一帧情报
    INTEL_STABLE = "intel.stable"     # 多帧投票收敛（车端可据此决策）
    INTEL_ACCUM = "intel.accum"       # 机载端多帧累积更新

    # ---- 感知 ----
    VISION_DETECT = "vision.detect"   # 检测结果（限流记录）
    VISION_MODEL = "vision.model"     # 模型/解码参数
    VISION_STALE = "vision.stale"     # 相机断流 / 从没收到图像（看门狗）
    HOMOGRAPHY = "homography"         # 单应矩阵：未标定 -> 像素落不到格号 -> 情报出不来

    # ---- 规划与路线 ----
    PLAN_BUILT = "plan.built"         # 任务序列算出来了
    ROUTE_PUBLISH = "route.publish"   # 向 /nav/waypoint 真的发出去了
    ROUTE_BLOCKED = "route.blocked"   # 有情报但没发车（带 reason）
    ROUTE_REJECT = "route.reject"     # 路线串拼装/校验失败（带 reason）
    SHOOT_COUNT = "shoot.count"       # 车端上报的累计发射数

    # ---- 瞄准 / 开火 ----
    AIM_PASS = "aim.pass"
    AIM_BLOCK = "aim.block"           # 闸门拦截（带 why）
    FIRE_REQUEST = "fire.request"
    FIRE_DENIED = "fire.denied"       # 拒绝开火（带 reason）
    FIRE_SENT = "fire.sent"

    # ---- 裁判 ----
    REFEREE_STATE = "referee.state"

    # ---- 自研底盘（非渡众车才用）----
    MISSION_TASK = "mission.task"
    MISSION_DONE = "mission.done"


class REASON:
    """`route.blocked` / `route.reject` 的 reason 取值。

    节点写、复盘读，两边必须用同一套字面量；否则报告里只剩一个没人认识的字符串，
    提示（`log_diagnosis.BLOCK_HINT`）也挂不上。
    """

    MIN_MINES = "min_mines"              # 确认的雷数不够
    FACE_UNKNOWN = "face_unknown"        # 敌我还没判决
    BRIDGE_REJECT = "bridge_reject"      # 桥洞布点不符（严格模式）
    NO_CELLS = "no_cells"                # 任务序列为空
    MIN_SHOOT_STOPS = "min_shoot_stops"  # 打击点少于要求
    ROUTE_ERROR = "route_error"          # 路线串非法


class AIM_BLOCK_WHY:
    """`aim.block` 的 why 取值。"""

    GATE_OFF = "gate_off"                # enable_fire=false（人为关闸）
    NOT_ALLOWED = "not_in_allowlist"     # 类别不在白名单
    SHORT_FRAME = "short_frame"          # 帧长不对，当无目标处理


#: 记录里已被 BlackBox 占用的键；payload 若用同名键，会被挪进 `data` 子对象，
#: 避免把框架字段覆盖掉（否则复盘时 `ev` 被业务字段吃掉，整条记录就废了）。
RESERVED = frozenset({"ts", "mono", "seq", "node", "ev", "lvl", "session", "src", "data"})

#: 等级 -> 数值，用于 min_level 过滤
LEVELS = {"debug": 10, "info": 20, "warn": 30, "warning": 30, "error": 40}


def _safe(part: Any) -> str:
    """把任意字符串变成安全文件名片段（节点名可能带 '/' 或空格）。"""
    out = []
    for ch in str(part):
        out.append(ch if (ch.isalnum() or ch in "-_.") else "_")
    return "".join(out) or "unknown"


def _json_default(obj: Any) -> Any:
    """不可直接序列化的对象：Enum 取 value，其余退化到 str。

    注意顺序 —— 先看 `value` 是因为 `IntEnum`/`TaskType` 之类既像数字又有名字，
    记 `"MINE"` 比记 `<TaskType.MINE: 3>` 有用得多。
    """
    val = getattr(obj, "value", None)
    if isinstance(val, (str, int, float, bool)):
        return val
    return str(obj)


def default_log_dir() -> Path:
    """日志根目录：`AG_LOG_DIR` > 系统临时目录/ag_logs（**不写家目录**）。"""
    env = os.environ.get("AG_LOG_DIR", "").strip()
    if env:
        return Path(env)
    return Path(tempfile.gettempdir()) / "ag_logs"


def default_session() -> str:
    """会话名：`AG_LOG_SESSION` > 当天日期。

    一局一个目录时用 `AG_LOG_SESSION=final_run_3` 统一指定；
    不指定就按天归档，复盘工具会自动切到"最后一个 session.begin"之后。
    """
    env = os.environ.get("AG_LOG_SESSION", "").strip()
    if env:
        return _safe(env)
    return time.strftime("%Y%m%d")


def _host() -> str:
    try:
        return socket.gethostname()
    except Exception:  # noqa: BLE001
        return "?"


# ---------------------------------------------------------------- 写入端
class BlackBox:
    """一个节点一个实例，写 `<log_dir>/<session>/<node>-<pid>.jsonl`。"""

    def __init__(
        self,
        node: str,
        log_dir: Optional[str] = None,
        session: Optional[str] = None,
        enabled: bool = True,
        min_level: str = "info",
        max_bytes: int = 8 * 1024 * 1024,
        keep: int = 3,
        flush_every: int = 1,
    ) -> None:
        self.node = _safe(node)
        self.session = _safe(session or default_session())
        self.enabled = bool(enabled)
        self.min_level = str(min_level).lower()
        #: 打不开文件、写失败等次数。**这个数 > 0 本身就是一条要复盘的异常。**
        self.errors = 0
        self.max_bytes = int(max_bytes)
        self.keep = max(1, int(keep))
        self.flush_every = max(1, int(flush_every))

        self._dir = Path(log_dir) if log_dir else default_log_dir()
        self._base = self._dir / self.session
        self._file: Optional[Path] = None
        self._fh = None
        self._seq = 0
        self._since_flush = 0
        self._lock = threading.Lock()
        self._warned = False
        self._last: Dict[str, float] = {}
        self._count: Dict[str, int] = {}

        if self.enabled:
            self._open()

    # ---------------- 文件 ----------------
    @property
    def path(self) -> Optional[Path]:
        """当前日志文件路径（未启用 / 打开失败时为 None）。"""
        return self._file

    def _open(self) -> None:
        try:
            self._base.mkdir(parents=True, exist_ok=True)
            self._file = self._base / f"{self.node}-{os.getpid()}.jsonl"
            self._fh = open(self._file, "a", encoding="utf-8", buffering=1)
        except Exception as e:  # noqa: BLE001
            self._fail(f"打开日志文件失败: {type(e).__name__}: {e}")

    def _fail(self, msg: str) -> None:
        """任何日志侧错误都到这里：计数 + 只提示一次 + 关掉自己，绝不外抛。"""
        self.errors += 1
        self.enabled = False
        self._fh = None
        if not self._warned:
            self._warned = True
            print(f"[blackbox] {self.node}: {msg}（日志已停用，不影响主流程）",
                  file=sys.stderr)

    def _rotate_if_needed(self) -> None:
        """超过 max_bytes 就滚一份（.1 最新），只保留 keep 份。

        现场一局可能跑十几分钟、含高频事件，不轮转会把 /tmp 写满。
        """
        if self._fh is None or self._file is None or self.max_bytes <= 0:
            return
        try:
            if self._fh.tell() < self.max_bytes:
                return
            self._fh.close()
            # 已有的 .1 -> .2 -> ... 依次后退，超出 keep 的直接删
            for i in range(self.keep, 0, -1):
                src = self._file.with_suffix(f".{i}.jsonl")
                dst = self._file.with_suffix(f".{i + 1}.jsonl")
                if src.exists():
                    if i + 1 > self.keep:
                        src.unlink()
                    else:
                        os.replace(src, dst)
            os.replace(self._file, self._file.with_suffix(".1.jsonl"))
            self._fh = open(self._file, "a", encoding="utf-8", buffering=1)
        except Exception as e:  # noqa: BLE001
            self._fail(f"日志轮转失败: {type(e).__name__}: {e}")

    # ---------------- 接口 ----------------
    def set_target(self, log_dir: Optional[str] = None,
                   session: Optional[str] = None) -> None:
        """换目录/换会话（测试与"同一进程跑多局"用）。"""
        with self._lock:
            if self._fh is not None:
                try:
                    self._fh.close()
                except Exception:  # noqa: BLE001
                    pass
            self._dir = Path(log_dir) if log_dir else default_log_dir()
            self.session = _safe(session or default_session())
            self._base = self._dir / self.session
            self._file = None
            self._fh = None
            self._since_flush = 0
            if self.enabled:
                self._open()

    def log(self, ev: str, level: str = "info", **fields: Any) -> None:
        """写一条事件。`fields` 会成为 JSON 顶层字段（冲突的塞进 `data`）。"""
        if not self.enabled:
            return
        if LEVELS.get(str(level).lower(), 20) < LEVELS.get(self.min_level, 20):
            return

        with self._lock:
            if self._fh is None:
                return
            try:
                self._seq += 1
                rec: Dict[str, Any] = {
                    "ts": round(time.time(), 3),
                    "mono": round(time.monotonic(), 3),
                    "seq": self._seq,
                    "node": self.node,
                    "ev": str(ev),
                    "lvl": str(level).lower(),
                    "session": self.session,
                }
                extra = {k: v for k, v in fields.items() if k in RESERVED}
                for k, v in fields.items():
                    if k not in RESERVED:
                        rec[k] = v
                if extra:
                    rec["data"] = extra

                self._rotate_if_needed()
                if self._fh is None:
                    return
                self._fh.write(json.dumps(rec, ensure_ascii=False,
                                          default=_json_default) + "\n")
                self._since_flush += 1
                if self._since_flush >= self.flush_every:
                    self._since_flush = 0
                    self._fh.flush()
            except Exception as e:  # noqa: BLE001
                self._fail(f"写日志失败: {type(e).__name__}: {e}")

    write = log  # 别名，读起来更顺：bb.write(EV.SESSION_BEGIN, ...)

    def throttled(self, key: str, period_s: float) -> bool:
        """限流闸：同一 key 每 `period_s` 秒只放行一次。

        初值取 0.0 是故意的 —— **第一次必定放行**，现场才能确认这条链路活着。
        """
        now = time.monotonic()
        if now - self._last.get(key, 0.0) >= period_s:
            self._last[key] = now
            return True
        return False

    def every(self, key: str, n: int) -> bool:
        """抽样闸：同一 key 每 n 次放行一次（`n<=1` 时全放行）。"""
        n = max(1, int(n))
        self._count[key] = self._count.get(key, 0) + 1
        return self._count[key] % n == 0

    def close(self) -> None:
        with self._lock:
            if self._fh is not None:
                try:
                    self._fh.flush()
                    self._fh.close()
                except Exception:  # noqa: BLE001
                    pass
                self._fh = None

    def __enter__(self) -> "BlackBox":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    # ---------------- 便捷构造 ----------------
    @classmethod
    def for_node(
        cls,
        node: str,
        log_dir: Optional[str] = None,
        session: Optional[str] = None,
        enabled: bool = True,
        **kw: Any,
    ) -> "BlackBox":
        """给节点用：顺手写一条 `node.ready`（含 pid/host/文件路径）。

        复盘时"这条日志属于哪个进程、哪台机器"是最基本的问题，
        不能指望从别处推断。
        """
        bb = cls(node, log_dir=log_dir, session=session, enabled=enabled, **kw)
        bb.log(EV.NODE_READY, pid=os.getpid(), host=_host(),
               cwd=os.getcwd(), log_file=str(bb.path or ""))
        return bb

    @classmethod
    def from_params(cls, node: Any, name: Optional[str] = None, **kw: Any) -> "BlackBox":
        """节点里用这个：读 `log_dir` / `log_enabled` / `log_level` 三个参数并挂载。

        节点侧只要两步：先 `declare_parameter` 这三个名字，再 `self.bb = ...from_params(self)`。
        参数走 ROS 参数系统而不是硬编码，现场才能用
        `ros2 param set /car_node log_enabled false` 临时静音。
        """
        def _p(key: str, default: Any) -> Any:
            try:
                return node.get_parameter(key).value
            except Exception:  # noqa: BLE001
                return default

        try:
            node_name = name or node.get_name()
        except Exception:  # noqa: BLE001
            node_name = name or getattr(node, "node_name", "node")

        bb = cls.for_node(
            node_name,
            log_dir=(str(_p("log_dir", "") or "").strip() or None),
            enabled=bool(_p("log_enabled", True)),
            min_level=str(_p("log_level", "info") or "info"),
            **kw,
        )
        attach(node, bb)
        return bb


class _LoggerMirror:
    """把 ROS logger 的文本日志镜像进黑匣子，保留原 logger 的全部行为。

    为什么值得做：现场最有用的一句话往往藏在 warning 里
    （"只确认了 2 颗雷（要求 ≥3），暂不发车"）。让文本和结构化事件落在**同一条时间轴**上，
    复盘时才能看到"前一条事件之后，节点抱怨了什么"。

    为什么还要去重：`car_node.replan` 在情报收敛后**每来一帧就调用一次**，
    条件不满足时每次都 warning。原样镜像会把日志刷满同一个原因，
    反而看不出"第一次是什么时候发生的"。所以同一条消息在 `dedup_s` 内只落一条
    （**只影响落盘，不影响真实 ROS 日志**）。
    """

    def __init__(self, logger: Any, bb: BlackBox, dedup_s: float = 2.0) -> None:
        self._logger = logger
        self._bb = bb
        self._dedup_s = float(dedup_s)
        self._last: Dict[str, float] = {}

    def _log(self, level: str, msg: Any) -> None:
        text = str(msg)
        if self._dedup_s > 0:
            key = f"{level}:{text}"
            now = time.monotonic()
            if now - self._last.get(key, -1e9) < self._dedup_s:
                return
            self._last[key] = now
        self._bb.log(EV.ROS_LOG, level=level, msg=text)

    def info(self, msg: Any) -> None:
        self._log("info", msg)
        self._orig("info", msg)

    def warning(self, msg: Any) -> None:
        self._log("warn", msg)
        self._orig("warning", msg)

    warn = warning

    def error(self, msg: Any) -> None:
        self._log("error", msg)
        self._orig("error", msg)

    def debug(self, msg: Any) -> None:
        self._log("debug", msg)
        self._orig("debug", msg)

    def _orig(self, name: str, msg: Any) -> None:
        fn = getattr(self._logger, name, None)
        if callable(fn):
            try:
                fn(msg)
            except Exception:  # noqa: BLE001
                # 镜像不能把原来的日志打挂
                pass

    def __getattr__(self, name: str) -> Any:
        return getattr(self._logger, name)


def attach(node: Any, bb: BlackBox, mirror: bool = True) -> BlackBox:
    """把黑匣子挂到 ROS 节点上：`node.bb`，并把文本日志一起镜像。

    镜像靠替换 `get_logger()` 的返回对象实现（rclpy 里就是返回 `self._logger`）。
    万一某个版本不是这个实现，这里静默跳过 —— **只丢文本镜像，结构化事件照写**。
    """
    try:
        node.bb = bb
    except Exception:  # noqa: BLE001
        pass
    if mirror and bb.enabled:
        try:
            node._logger = _LoggerMirror(node.get_logger(), bb)
        except Exception:  # noqa: BLE001
            pass
    return bb


# ---------------------------------------------------------------- 读取端
@dataclass
class Event:
    """一条事件（读取端视角）。"""

    ts: float = 0.0
    mono: float = 0.0
    seq: int = 0
    node: str = ""
    ev: str = ""
    lvl: str = "info"
    session: str = ""
    src: str = ""
    data: Dict[str, Any] = field(default_factory=dict)

    def get(self, key: str, default: Any = None) -> Any:
        return self.data.get(key, default)

    def num(self, key: str, default: float = 0.0) -> float:
        v = self.data.get(key, default)
        try:
            return float(v)
        except (TypeError, ValueError):
            return default

    def text(self, key: str, default: str = "") -> str:
        v = self.data.get(key, default)
        return default if v is None else str(v)

    @classmethod
    def from_record(cls, rec: Dict[str, Any], src: str = "") -> "Event":
        data = {k: v for k, v in rec.items() if k not in RESERVED}
        extra = rec.get("data")
        if isinstance(extra, dict):
            data.update(extra)
        try:
            ts = float(rec.get("ts", 0.0))
        except (TypeError, ValueError):
            ts = 0.0
        try:
            mono = float(rec.get("mono", 0.0))
        except (TypeError, ValueError):
            mono = 0.0
        try:
            seq = int(rec.get("seq", 0))
        except (TypeError, ValueError):
            seq = 0
        return cls(
            ts=ts,
            mono=mono,
            seq=seq,
            node=str(rec.get("node", "")),
            ev=str(rec.get("ev", "")),
            lvl=str(rec.get("lvl", "info")),
            session=str(rec.get("session", "")),
            src=src,
            data=data,
        )


def collect_jsonl(path: Any) -> List[Path]:
    """把一个文件或目录展开成 .jsonl 文件列表（目录递归，排序稳定）。"""
    p = Path(path)
    if p.is_file():
        return [p]
    if not p.is_dir():
        return []
    return sorted(x for x in p.rglob("*.jsonl") if x.is_file())


def find_log_dirs(root: Any = None) -> List[Path]:
    """列出含日志的目录（自身或子目录），按**最后写入时间**倒序。

    现场最常见的困惑是"日志到底写哪去了"，所以报告工具会把它打出来。
    """
    root = Path(root) if root else default_log_dir()
    scored: List[Tuple[float, Path]] = []
    if not root.exists():
        return []
    for d in [root] + [x for x in root.rglob("*") if x.is_dir()]:
        files = collect_jsonl(d)
        if not files:
            continue
        try:
            newest = max(f.stat().st_mtime for f in files)
        except OSError:
            continue
        scored.append((newest, d))
    scored.sort(key=lambda t: t[0], reverse=True)
    return [d for _, d in scored]


def read_events(paths: Any, skip_bad: bool = True) -> Tuple[List[Event], int]:
    """读若干文件/目录 -> (按时间排序的事件, 坏行数)。

    返回坏行数是刻意的：**日志本身损坏**也是一条要复盘的线索
    （写盘被打断、磁盘满、手工编辑过），不能悄悄吞掉。
    """
    if isinstance(paths, (str, Path)):
        paths = [paths]
    files: List[Path] = []
    for p in paths:
        files.extend(collect_jsonl(p))

    events: List[Event] = []
    bad = 0
    for f in files:
        try:
            with open(f, "r", encoding="utf-8", errors="replace") as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        rec = json.loads(line)
                    except Exception:  # noqa: BLE001
                        bad += 1
                        continue
                    if not isinstance(rec, dict):
                        bad += 1
                        continue
                    events.append(Event.from_record(rec, src=f.name))
        except Exception:  # noqa: BLE001
            bad += 1
            if not skip_bad:
                continue
    events.sort(key=lambda e: (e.ts, e.seq))
    return events, bad


def session_start_index(events: List[Event]) -> int:
    """最后一条 `session.begin` 的下标（没有则 0）。

    一局一跑的场景下，这比"按时间猜"可靠得多：日志目录可能留着上午的调试记录，
    直接全读会把几局混成一份报告。
    """
    idx = 0
    for i, e in enumerate(events):
        if e.ev == EV.SESSION_BEGIN:
            idx = i
    return idx
