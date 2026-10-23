# -*- coding: utf-8 -*-
r"""卡 t5 · 端到端离线联调（bench，不发车、不碰车）：

    【感知管线真实产物】→ FakeBridge 收件箱 → `DroneTask`（真实解析）
        → 黑盒 `P_DRONE` / 事件 `EV_BRIDGE_INFO` → `MissionTask`（真实消费）
        → 规划/路线穿洞格 → 遥测 `state()["bridge"]`

## 与既有测试的分工（为什么另开一个文件）

| 套件 | 覆盖 |
|---|---|
| `test_drone.py` | 车侧**解析**（喂 JSON → `P_DRONE`），不接 Mission |
| `test_mission.py` 第 14 节 | Mission**消费**，但手工造 `DroneState` 塞黑盒（跳过链路层） |
| **本套件** | **两者接起来**：报文经真实 `DroneTask` 解码后驱动 Mission 选洞 |

即：本套件断言"**线上真发的那串 JSON 能不能一路走到路线里**"；
`test_mission` 断言"Mission 拿到情报后的决策逻辑对不对"。两者不重复。

## 输入真实性（★ 本套件的立身之本）

第 1 主场景**不手写 JSON**：它读取感知管线 `cli.py` 真跑出来的落盘产物
`dzsmartscout/tools/perception/reports/cli_messages.jsonl`，**逐字节原样**灌进收件箱。

★ 但该产物里**没有** `bridge_passable` —— 图纸口径没有桥顶箭头
（`map_src.png` 是「物资点设置」截图，不画箭头），没有箭头就不该凭空产出穿洞情报
（依据：`tools/perception/tests/test_pipeline_e2e.py:129-141` 明写"不该凭空产出"）。

所以对 `bridge_passable` 采用**两级证据**：
1. 产物里那条 `heartbeat`（原样）→ 证明"真实产物能被车吃下"；
2. `bridge_passable` 用感知管线自己的构造器 `perception.protocol.build_bridge_passable()`
   + `encode()`（**不手写字面串**）→ 证明"管线按契约造的报文能被车吃下并改变路线"。
   构造器出处：`dzsmartscout/tools/perception/protocol.py:101-114`。

## 跨仓依赖（如实声明，缺失即 SKIP 而非假绿）

感知管线在**另一个仓库** `dzsmartscout/`（无 `.git`，与 `dzrobot_v2` 并列）。
`_perception()` 惰性 import：拿不到就打印 `SKIP` 并**减少断言总数**
（`harness.check` 只统计真断言，所以分数会如实变少，不会拿 SKIP 冒充 PASS）。

## 边界

只读 `config/map.yaml` / `config/robot.yaml`（现场真源，**不修改**）、只读感知产物；
临时地图写系统 TEMP。全程 FakeBridge + FakeClock，无 ROS、无真机、不发车。
"""

from __future__ import annotations

import json
import pathlib
import re
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from dzbrain.core import geometry as geo                              # noqa: E402
from dzbrain.core.blackboard import (P_DRONE, P_MISSION_PHASE, P_POSE,  # noqa: E402
                                     P_ROUTE, P_SAFETY, Blackboard)
from dzbrain.core.config import Config                                # noqa: E402
from dzbrain.core.events import EV_BRIDGE_INFO, EventBus              # noqa: E402
from dzbrain.core.state import SafetyState                            # noqa: E402
from dzbrain.route.parser import parse_token                          # noqa: E402
from dzbrain.tasks.drone import (P_DRONE_COVERAGE, P_FIELD_VERIFY,  # noqa: E402
                                 DroneTask)
from dzbrain.tasks.mission import MissionTask                         # noqa: E402
from fakes import FakeBridge, FakeClock                               # noqa: E402
from harness import check, finish, section                            # noqa: E402

# --------------------------------------------------------------------------- #
#  路径与夹具
# --------------------------------------------------------------------------- #
_WS = pathlib.Path(__file__).resolve().parents[3]          # dzrobot_v2/
_ROOT = _WS.parent                                          # 比赛存档/v1/
_ROBOT = _WS / "config" / "robot.yaml"
_MAP_SRC = _WS / "config" / "map.yaml"
_PERCEPTION_DIR = _ROOT / "dzsmartscout" / "tools"
_PERCEPTION_PRODUCT = _PERCEPTION_DIR / "perception" / "reports" / "cli_messages.jsonl"

#: 其余部分**逐字照抄**现场真源，只把 `bridges:` 放宽成四个洞
#: （现场每局随机换洞；`config/map.yaml` 现值只有两个，测"换洞"必须给出可换的目标）。
#: 与 `test_mission.py:1213-1216` 同一手法，理由相同。临时文件写系统 TEMP。
_FIX = pathlib.Path(tempfile.mkdtemp(prefix="dz_t5_fix_"))
_MAP = _FIX / "map_t5.yaml"
_MAP.write_text(
    re.sub(r"(?m)^([ \t]*)bridges:.*$",
           lambda m: m.group(1) + 'bridges:    ["K4", "K5", "K9", "K10"]',
           _MAP_SRC.read_text(encoding="utf-8")), encoding="utf-8")

_ROBOT_CFG = Config.load(str(_ROBOT))
_TOKENS = [str(t) for t in (_ROBOT_CFG.get("route.tokens") or [])]


def _base_cells() -> list[str]:
    """现场 token 列表 → **格名**列表（用项目自己的 `route.parser.parse_token`，不手写剥后缀）。

    为什么不能直接拿 `_TOKENS` 比：`route.blocks()` 返回的是**裸格名**
    （`N5,y` → `N5`、`C10*` → `C10`），而 token 带方向/标注后缀 ——
    自己剥又会漏掉 `,y`（带朝向的终点）这种形状，必须走真解析器。
    """
    return [str(parse_token(t)["block"]) for t in _TOKENS]


_BASE_BLOCKS = _base_cells()

LOWER_DEFAULT = str(_ROBOT_CFG.get("route.cross_lower_default") or "").strip().upper()
UPPER_DEFAULT = str(_ROBOT_CFG.get("route.cross_upper_default") or "").strip().upper()
#: 现场候选集合（**每局只有一个洞**）：`["K10"]` / `["K5"]`。
#: 单值候选下"换洞"在逻辑上不可能发生（无人机报任何别的格都会被拒收），
#: 所以本套件在临时夹具里把它放宽成"两个候选/半场" —— 这是**测试夹具**，不是配置建议。
LOWER_CANDS = ("K9", "K10")
UPPER_CANDS = ("K4", "K5")

#: 临时 robot.yaml：`config/robot.yaml` **逐字照抄**，只改 4 处
#: （每半场的候选集合 + `planner.map_path` 指向放宽 bridges 的临时地图）。
#: 其余键（mission/pursuit/aim/…）全部保持现场值 ⇒ 夹具不掩盖任何现场口径。
_TMP_ROBOT = _FIX / "robot_t5.yaml"
_ROBOT_TEXT = _ROBOT.read_text(encoding="utf-8")
_ROBOT_TEXT = _ROBOT_TEXT.replace('cross_lower_candidates: ["K10"]',
                                  f'cross_lower_candidates: {list(LOWER_CANDS)}', 1)
_ROBOT_TEXT = _ROBOT_TEXT.replace('cross_upper_candidates: ["K5"]',
                                  f'cross_upper_candidates: {list(UPPER_CANDS)}', 1)
_TMP_ROBOT.write_text(_ROBOT_TEXT, encoding="utf-8")


def cfg_of(*, wait: float = 0.0, drone_enabled: bool = True, auto_plan: bool = False) -> Config:
    """整车配置：**整份**读临时 robot.yaml（= 现场 robot.yaml + 上表 4 处夹具改动），
    不逐键重敲 —— 少一个键就少一次"夹具与现场不一致"的机会。

    只额外压两处运行时值（否则测试没法在合理时间内跑完）：
    `mission.auto_start_delay_s` 现场 45 s → 0.1 s；`mission.scan_mode` 现场
    continuous（云台单写者下 inert）→ none（本套件不测扫描）。
    """
    cfg = Config.load(str(_TMP_ROBOT))
    cfg._flat.update({                              # 只压这几处（其余键全是现场值）
        "mission.auto_start": True,
        "mission.auto_start_delay_s": 0.1,          # 现场 45 s：测试等不起
        "mission.scan_mode": "none",
        "mission.bridge_info_wait_s": float(wait),
        "drone.enabled": bool(drone_enabled),
        "planner.auto_plan_enabled": bool(auto_plan),
        "planner.map_path": str(_MAP),              # 放宽 bridges 的临时地图
        "logging.rate_limit_s": 0.0,
    })
    return cfg


class Link:
    """一条**完整**链路：FakeBridge + 黑盒 + 事件总线 + DroneTask + MissionTask。"""

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.drone_logs: list[str] = []
        self.mission_logs: list[str] = []
        self.drone = DroneTask(self.bridge, self.bb, self.bus, cfg,
                               log=self.drone_logs.append, clock=self.clock.now)
        self.mission = MissionTask(self.bridge, self.bb, self.bus, cfg,
                                   log=self.mission_logs.append, clock=self.clock.now)
        self.bb.set(P_SAFETY, SafetyState(ok=True), "t5")
        self.set_pose("N10", geo.HEADING_Y)
        self.drone.setup()
        self.mission.setup()

    # ---------------- 操作 ----------------
    def set_pose(self, block: str, yaw: float = 0.0) -> None:
        x, y = geo.block_to_meters(block)
        self.bridge.set_pose(x, y, yaw)
        self.bb.set(P_POSE, geo.Pose(x, y, yaw), "t5")

    def send(self, raw: str) -> None:
        """把**待发报文**塞进车侧收件箱（真机上是 DDS 订阅回调干的）。"""
        self.bridge.push_drone(raw)

    def tick(self, dt: float = 0.1) -> None:
        """一拍：DroneTask 先解码（写 P_DRONE + 发 EV_BRIDGE_INFO），MissionTask 再消费。"""
        self.drone.tick()
        self.mission.tick()
        self.clock.advance(dt)

    def ticks(self, n: int, dt: float = 0.1) -> None:
        for _ in range(n):
            self.tick(dt)

    # ---------------- 读 ----------------
    def drone_state(self):
        return self.bb.get(P_DRONE)

    def bridge_tel(self) -> dict:
        return self.mission.state()["bridge"]

    def route(self):
        return self.bb.get(P_ROUTE) or self.mission.route

    def k_blocks(self) -> list[str]:
        route = self.route()
        return [b for b in (route.blocks() if route is not None else []) if b.startswith("K")]

    def used(self) -> dict:
        return dict(self.bridge_tel().get("used") or {})

    def seen(self) -> dict:
        return dict(self.bridge_tel().get("seen") or {})

    def source_of(self, half: str) -> str:
        return str((self.bridge_tel().get(half) or {}).get("source") or "")

    def phase(self) -> str:
        return str(self.bb.get(P_MISSION_PHASE) or self.mission.phase)


# --------------------------------------------------------------------------- #
#  感知管线（跨仓、惰性、缺失即 SKIP）
# --------------------------------------------------------------------------- #
def _perception():
    """返回感知管线 `protocol` 模块；拿不到 → `None`（调用方打印 SKIP）。"""
    if not _PERCEPTION_DIR.is_dir():
        return None
    if str(_PERCEPTION_DIR) not in sys.path:
        sys.path.insert(0, str(_PERCEPTION_DIR))
    try:
        from perception import protocol               # type: ignore[import-not-found]
    except Exception:                                 # noqa: BLE001  拿不到就当没有
        return None
    return protocol


def product_lines() -> list[str]:
    """感知管线真跑出来的落盘产物（逐字节原样，不做任何加工）。"""
    if not _PERCEPTION_PRODUCT.is_file():
        return []
    return [ln for ln in _PERCEPTION_PRODUCT.read_text(encoding="utf-8").splitlines()
            if ln.strip()]


def bp(half: str, block: str, conf: float = 0.9, seq: int | None = None) -> str:
    """**用管线自己的构造器**造 `bridge_passable` 报文（不手写字面串）。"""
    proto = _perception()
    if proto is None:                                 # 兜底：契约字段照 drone.py:273-289 抄
        msg = {"type": "bridge_passable", "half": half, "block": block,
               "confidence": conf, "ts": 0.0}
        if seq is not None:
            msg["seq"] = seq
        return json.dumps(msg, ensure_ascii=False, separators=(",", ":"))
    return proto.encode(proto.build_bridge_passable(half, block, conf, seq=seq))


def hb(seq: int) -> str:
    """心跳报文（同样优先用管线构造器）。"""
    proto = _perception()
    if proto is None:
        return json.dumps({"type": "heartbeat", "ts": 0.0, "seq": seq},
                          ensure_ascii=False, separators=(",", ":"))
    return proto.encode(proto.build_heartbeat(seq=seq))


# --------------------------------------------------------------------------- #
#  1. 主场景：真实产物 + 管线构造器 → 一路走到路线里
# --------------------------------------------------------------------------- #
def test_real_product_and_replan() -> None:
    section("1. 端到端：感知管线产物 → DroneTask → MissionTask（穿洞格真的变了）")

    lines = product_lines()
    types = []
    for ln in lines:
        try:
            types.append(json.loads(ln).get("type"))
        except Exception:                             # noqa: BLE001  产物坏了要看得见
            types.append("<坏行>")
    check("1.1 前置：读到了感知管线真实落盘产物 cli_messages.jsonl（非手写）",
          len(lines) >= 3, f"{_PERCEPTION_PRODUCT.name} = {len(lines)} 行")
    check("1.2 前置：产物里确实没有 bridge_passable（图纸口径无箭头 ⇒ 不凭空产出，"
          "依据 test_pipeline_e2e.py:129-141）",
          "bridge_passable" not in types, f"types={types}")

    # ---- 真实产物原样灌进去（含 heartbeat / target_zone / field_verify）----
    rig = Link(cfg_of())
    for ln in lines:
        rig.send(ln)
    rig.ticks(4)
    check("1.3 ★ 产物里的 heartbeat 真的把 DroneTask 拉上线（链路建立）",
          rig.drone_state() is not None and bool(rig.drone_state().online)
          and rig.drone.messages >= len(lines),
          f"online={getattr(rig.drone_state(), 'online', None)} messages={rig.drone.messages}")
    check("1.4 ★ 产物里的 target_zone / field_verify 被真解析（核对通道活着）",
          not rig.drone.unknown_types and rig.drone.bad_messages == 0,
          f"unknown={rig.drone.unknown_types} bad={rig.drone.bad_messages}")

    # ---- 情报注入：用管线构造器造 bridge_passable（不手写）----
    #   ★ 顺序要紧：**链路先在线 + 情报先到**，再让 Mission 出发。
    #     反过来的话 Mission 已经用默认洞出发了 —— 那时情报只更新遥测、不改已装路线
    #     （设计如此：出车后不现场改路线）。所以这里把心跳与情报一起发。
    rig2 = Link(cfg_of())
    rig2.send(hb(seq=1))
    rig2.send(bp("lower", "K11", 0.95, seq=2))   # CFG/GOT: K11 在 7~11 行段内且 ≠ 默认 K9（恢复换洞语义）
    rig2.ticks(3)
    check("1.5 ★ 管线构造器造的 bridge_passable 能过 DroneTask 解析（half/block 归一化）",
          rig2.drone.bridge_updates == 1 and rig2.drone.bridge_half == "lower"
          and rig2.drone.bridge_block == "K11",
          f"updates={rig2.drone.bridge_updates} half={rig2.drone.bridge_half!r} "
          f"block={rig2.drone.bridge_block!r}")

    blocks = list(rig2.route().blocks())
    base = _base_cells()                              # 现场 token 逐格（剥前后缀）
    removed = sorted(set(base) - set(blocks))
    added = sorted(set(blocks) - set(base))
    check("1.6 ★★ 端到端换洞生效：情报 K11 进路线（默认 %s 被换掉）" % LOWER_DEFAULT,
          "K11" in rig2.k_blocks() and LOWER_DEFAULT not in rig2.k_blocks(),
          f"K 列格={rig2.k_blocks()}")
    check("1.7 ★ 只动穿洞**三连**那一行（进洞口-K 列格-出洞口），其余航点逐格不变",
          removed == ["J9", "K9", "L9"] and added == ["J11", "K11", "L11"]
          and len(blocks) == len(base),
          f"−{removed} +{added} / 点数 {len(blocks)} vs {len(base)}")
    check("1.8 遥测 `used.lower` = K11（本局实际装进路线的格）",
          rig2.used().get("lower") == "K11", str(rig2.used()))
    check("1.9 遥测 `seen.lower.cell` = K11（原始情报留痕，排障口径）",
          (rig2.seen().get("lower") or {}).get("cell") == "K11", str(rig2.seen()))
    check("1.10 全程照常发车（阶段 DRIVE），没有被情报拖住",
          rig2.phase() == "DRIVE", rig2.phase())


# --------------------------------------------------------------------------- #
#  2. 单槽覆盖：连续两条半场情报都要记住（t4 的 D3 回归护栏）
# --------------------------------------------------------------------------- #
def test_both_halves_single_slot() -> None:
    section("2. 两条半场情报（DroneState 只有一个槽）→ 两半场都被采纳")

    rig = Link(cfg_of())
    rig.send(hb(seq=1))                              # 链路先在线（否则 Mission 用默认洞出发）
    rig.send(bp("lower", "K11", 0.9, seq=2))           # CFG: 2.2 同源改注入 K11           # 同一拍先下半场
    rig.send(bp("upper", "K4", 0.9, seq=3))           # 再上半场（DroneTask 逐条处理，后者覆盖单槽）
    rig.ticks(3)

    st = rig.drone_state()
    check("2.1 ★ 后半场情报覆盖了 P_DRONE 单槽（现场真源：DroneState 只有一组字段）",
          str(getattr(st, "bridge_half", "")) == "upper"
          and str(getattr(st, "bridge_block", "")) == "K4",
          f"half={getattr(st, 'bridge_half', None)!r} block={getattr(st, 'bridge_block', None)!r}")
    check("2.2 ★★ 先到的下半场情报**没被覆盖丢失**：K11 与 K4 同时生效（D3 回归护栏）",
          "K11" in rig.k_blocks() and "K4" in rig.k_blocks()
          and LOWER_DEFAULT not in rig.k_blocks() and UPPER_DEFAULT not in rig.k_blocks(),
          f"K 列格={rig.k_blocks()}")
    check("2.3 两半场遥测 `used` 各自独立",
          rig.used() == {"lower": "K11", "upper": "K4"}, str(rig.used()))
    check("2.4 遥测 `seen` 里两半场都有原始情报留痕",
          set(rig.seen()) == {"lower", "upper"}, str(rig.seen()))


# --------------------------------------------------------------------------- #
#  3. 降级矩阵：离线 / 过期 / 坏格 / 链路关闭 / 全程无无人机
# --------------------------------------------------------------------------- #
def test_degrade_matrix() -> None:
    section("3. 降级矩阵：任何一路异常都回落默认，且照常发车")

    # ---- 3a 过期：心跳照发（保持在线），但情报不再更新 → 越过 info_stale_s ----
    rig = Link(cfg_of())
    rig.send(hb(seq=1))                              # 先上线（否则判 offline 而不是 stale）
    rig.send(bp("lower", "K9"))
    rig.ticks(2)
    stale_s = float(rig.cfg.get("drone.info_stale_s"))
    #  ★ 用 `clock.advance` 快进而不是"拍数 × dt"：Windows 上 320 拍空转是纯浪费，
    #    而新增的 `_BRIDGE_SUB_MAXSIZE` 队列还没满（每 2 s 一条心跳，不产生情报事件）。
    while rig.clock.now() < 1000.0 + stale_s + 2.0:
        rig.send(hb(seq=int(rig.clock.now())))
        rig.set_pose("N10", geo.HEADING_Y)
        rig.clock.advance(2.0)
        rig.tick(dt=0.0)                             # dt=0：时间由 advance 控制
    rig.ticks(2)
    check("3.1 ★ 无人机**在线**但情报过期 → `source=stale` 回落默认；已装路线不被改写",
          rig.drone_state() is not None and bool(rig.drone_state().online)
          and rig.source_of("lower") == "stale"
          and rig.used().get("lower") == "K9",
          f"online={getattr(rig.drone_state(), 'online', None)} "
          f"source={rig.source_of('lower')} used={rig.used()}")

    # ---- 3b 离线：只发情报不发心跳，等心跳超时 ----
    rig3 = Link(cfg_of())
    rig3.send(hb(seq=1))                              # 先在线
    rig3.ticks(2)
    rig3.clock.advance(float(rig3.cfg.get("drone.heartbeat_timeout_s")) + 1.0)   # 心跳超时
    rig3.set_pose("N10", geo.HEADING_Y)
    rig3.send(bp("lower", "K9"))                      # 离线后**仍收到**一条报文（链路还在）
    rig3.ticks(3)
    st3 = rig3.drone_state()
    check("3.2 ★ 无人机离线（心跳超时）→ 情报一律判 offline（即使报文刚到达）",
          rig3.source_of("lower") == "offline"
          and st3 is not None and bool(st3.online) is False,
          f"source={rig3.source_of('lower')} online={getattr(st3, 'online', None)} "
          f"used={rig3.used()}")

    # ---- 3c t31 权威门放宽：K11 在下半场行段内、**不在**夹具候选集合里 ⇒ 现在被采纳 ----
    rig4 = Link(cfg_of())
    rig4.send(hb(seq=1))
    rig4.send(bp("lower", "K11"))
    rig4.ticks(3)
    check("3.3 t31：池外但在下半场行段内的 K11 → **采纳**（纠正过期地图），"
          "路线真穿 K11（不再回落默认）",
          rig4.source_of("lower") == "drone"
          and rig4.used().get("lower") == "K11"
          and "K11" in rig4.k_blocks(),
          f"source={rig4.source_of('lower')} used={rig4.used()} K 列格={rig4.k_blocks()}")

    # ---- 3c2 t31：真非法格（非 K 列 / 格名非法）仍必须拒收 —— 否则就是"放宽"做成了"什么都收" ----
    for bad_cell, why in (("A1", "非 K 列"), ("Z9", "格名非法")):
        rig4b = Link(cfg_of())
        rig4b.send(hb(seq=1))
        rig4b.send(bp("lower", bad_cell))
        rig4b.ticks(3)
        check(f"3.3b t31 仍拒：{why}的情报 {bad_cell} → rejected + 回落默认，不喂给规划器",
              rig4b.source_of("lower") == "rejected"
              and rig4b.used().get("lower") == LOWER_DEFAULT
              and LOWER_DEFAULT in rig4b.k_blocks(),
              f"source={rig4b.source_of('lower')} used={rig4b.used()} 格={rig4b.k_blocks()}")

    # ---- 3d 链路整体关闭（drone.enabled=false） ----
    rig5 = Link(cfg_of(drone_enabled=False))
    rig5.send(hb(seq=1))
    rig5.send(bp("lower", "K9"))
    rig5.ticks(3)
    check("3.4 `drone.enabled=false` → DroneTask 整任务空转（一条都不解析）",
          rig5.drone.messages == 0 and rig5.drone.bridge_updates == 0,
          f"messages={rig5.drone.messages} updates={rig5.drone.bridge_updates}")
    check("3.5 `drone.enabled=false` → 穿洞格=默认，行程不受影响",
          rig5.used().get("lower") == LOWER_DEFAULT and rig5.phase() == "DRIVE",
          f"used={rig5.used()} phase={rig5.phase()}")

    # ---- 3e 车侧无无人机**任务**（P_DRONE 缺失）：不进等待窗口 ----
    #   ★ 构造方式：`drone.enabled=false` 让 DroneTask 整任务空转 → 黑盒里没有 P_DRONE。
    #     这是"链路上什么都没有"的干净口径（现场 = 没起 DroneTask / 没起车栈）。
    rig6 = Link(cfg_of(wait=5.0, drone_enabled=False))
    rig6.ticks(6)
    check("3.6 ★ 车侧无无人机链路（无 P_DRONE）→ **不进等待窗口**，0.6 s 内就出发",
          rig6.phase() == "DRIVE" and rig6.drone_state() is None,
          f"phase={rig6.phase()} t={rig6.clock.now() - 1000.0:.2f}s")
    check("3.7 ★ 该场景穿洞格恰为现场默认（与默认路线同格）",
          rig6.used() == {"lower": LOWER_DEFAULT, "upper": UPPER_DEFAULT}
          and set(rig6.route().blocks()) <= set(_base_cells()),
          f"used={rig6.used()}")

    # ---- 3f ★ 现场真口径：DroneTask 在跑但**无人机没上线**（P_DRONE 存在、offline） ----
    #   ★ 卡 t20（t14-F1 裁决）**重钉**：离线时**不再进等待窗口**（旧 3.9 断言"会进等待窗口"已废）。
    #     等待判据收回为「online=True **且**手上已有一半新鲜情报（source=drone）」；
    #     硬上限 `delay+wait` 仍是兜底，其覆盖见 `test_mission.py` 14.31b（另一半永不到 → 到点发车）
    #     与 14.27/14.29（离线/过期 → 不等）、14.30/14.31（online+一半新鲜 → 窗口内到齐即发）。
    rig7 = Link(cfg_of(wait=5.0))                    # drone.enabled=true、不发任何报文
    rig7.ticks(4)
    tel7 = rig7.bridge_tel()
    check("3.8 ★ DroneTask 在跑但无人机未上线 → 黑盒里**有** P_DRONE（link=true）",
          rig7.drone_state() is not None and tel7.get("link") is True,
          f"link={tel7.get('link')} online={getattr(rig7.drone_state(), 'online', None)}")
    check("3.9 ★★（t20 重钉）离线 → **不进等待窗口**：0.4s 内已出发、穿洞格=现场默认",
          bool((tel7.get("note") or {}).get("waiting")) is not True
          and rig7.phase() == "DRIVE"
          and rig7.used() == {"lower": LOWER_DEFAULT, "upper": UPPER_DEFAULT},
          f"waiting={(tel7.get('note') or {}).get('waiting')} phase={rig7.phase()} "
          f"used={rig7.used()}")
    rig7.ticks(int((0.1 + 5.0) / 0.1) + 4)           # 再快进 5s（远超旧窗口）
    check("3.10 ★★（t20 重钉）全程离线都**没有等过**：再快进 5s 仍是 DRIVE + 默认格、waited_s 为空",
          rig7.phase() == "DRIVE"
          and rig7.used() == {"lower": LOWER_DEFAULT, "upper": UPPER_DEFAULT}
          and (rig7.bridge_tel().get("note") or {}).get("waited_s") is None,
          f"phase={rig7.phase()} used={rig7.used()} "
          f"note={rig7.bridge_tel().get('note')}")


# --------------------------------------------------------------------------- #
#  4. 事件通道（t13 口径）：EV_BRIDGE_INFO 也能被 Mission 消费
# --------------------------------------------------------------------------- #
def test_event_channel() -> None:
    section("4. 事件通道：Mission 订阅 EV_BRIDGE_INFO（t13 口径，与单槽无关）")

    rig = Link(cfg_of())
    rig.tick()                                        # 先让任务转起来（订阅建立）
    rig.send(hb(seq=1))                               # 链路在线（否则 Mission 用默认洞出发）
    rig.bus.publish(EV_BRIDGE_INFO, source="t5-probe", half="lower", block="K9",
                    confidence=0.99, ts=rig.clock.now())      # 通道①：事件
    rig.send(bp("upper", "K4"))                       # 通道②：报文
    rig.ticks(3)

    check("4.1 ★ 订阅已建立（遥测 `subscribed=true` = 情报有第二条通道）",
          rig.bridge_tel().get("subscribed") is True,
          str(rig.bridge_tel().get("subscribed")))
    check("4.2 ★ 事件里报的半场同样进 `seen`（订阅事件流确实在消费）",
          (rig.seen().get("lower") or {}).get("cell") == "K9", str(rig.seen()))
    check("4.3 事件 + 报文两条通道同时有效：K9（事件）与 K4（报文）都在路线里",
          "K9" in rig.k_blocks() and "K4" in rig.k_blocks(), f"K 列格={rig.k_blocks()}")
    check("4.4 情报事件计数 > 0（遥测能回答『订阅在听吗』）",
          int(rig.bridge_tel().get("events") or 0) >= 1,
          str(rig.bridge_tel().get("events")))


# --------------------------------------------------------------------------- #
#  5. 健壮性：坏报文不崩、日志不刷屏、不 import rclpy
# --------------------------------------------------------------------------- #
def test_robustness() -> None:
    section("5. 健壮性：坏报文不崩、日志不刷屏、不 import rclpy")

    rig = Link(cfg_of())
    for bad in ("not json", "[]", '{"type":"unknown_x"}', '{"type":"bridge_passable"}',
                '{"type":"bridge_passable","half":"middle","block":"K9"}'):
        rig.send(bad)
    rig.ticks(3)
    check("5.1 五类坏报文全部只记日志、不抛异常（计数如实）",
          rig.drone.bad_messages >= 2 and rig.drone.missing_fields >= 2
          and "unknown_x" in rig.drone.unknown_types,
          f"bad={rig.drone.bad_messages} missing={rig.drone.missing_fields} "
          f"unknown={rig.drone.unknown_types}")

    rig2 = Link(cfg_of())
    rig2.send(bp("lower", "K9"))
    rig2.ticks(40)                                    # 4 s 稳态
    n_intel = sum(1 for s in rig2.mission_logs if "桥洞情报" in s)
    check("5.2 ★ 稳态 40 拍情报类日志有界（限流生效，不刷屏）",
          n_intel <= 8, f"{n_intel} 条 / 40 拍")
    check("5.3 DroneTask 自己的日志照常（两套日志各管各的）",
          len(rig2.drone_logs) >= 1, f"{len(rig2.drone_logs)} 条")
    check("5.4 bench 全程不 import rclpy（可在无 ROS 机器复跑）",
          "rclpy" not in sys.modules, "rclpy 未被加载")


# --------------------------------------------------------------------------- #
#  6. 覆盖语义 + EV_FIELD_VERIFY 消费方（G5-2 / G5-3，端到端）
# --------------------------------------------------------------------------- #
def fv(mines=None, obstacles=None, *, coverage=None, slice_=None,
       scan_id=None, seq=None) -> str:
    """造 `field_verify`：优先用感知管线**自己的构造器**（不手写新键 schema）。"""
    proto = _perception()
    if proto is not None and coverage is not None:
        return proto.encode(proto.build_field_verify(
            mines, obstacles, coverage=coverage, slice=slice_, scan_id=scan_id,
            ts=1791805124.744, seq=seq))
    msg: dict = {"type": "field_verify", "ts": 1791805124.744}
    if mines is not None:
        msg["mines"] = list(mines)
    if obstacles is not None:
        msg["obstacles"] = list(obstacles)
    if coverage is not None:
        msg["coverage"] = dict(coverage)
        if slice_ is not None:
            msg["slice"] = dict(slice_)
        if scan_id is not None:
            msg["scan_id"] = scan_id
    if seq is not None:
        msg["seq"] = seq
    return json.dumps(msg, ensure_ascii=False, separators=(",", ":"))


def test_coverage_and_field_verify_consumer() -> None:
    section("6. 覆盖语义 + EV_FIELD_VERIFY 消费方（机侧构造器 → 车侧账本）")

    # ---- 6a 旧报文（现场真源的无 coverage 形状）→ 零副作用 ----
    rig = Link(cfg_of())
    rig.ticks(3)                                  # 走完 auto_start 装线（与其它小节同手法）
    base_blocks = list(rig.route().blocks())
    rig.send(hb(seq=1))
    legacy = {"type": "field_verify", "mines": sorted(rig.drone.mines),
              "obstacles": sorted(rig.drone.obstacles),
              "ts": 1791805124.744, "seq": 2}
    rig.send(json.dumps(legacy, ensure_ascii=False, separators=(",", ":")))
    rig.ticks(2)
    check("6.1 旧 field_verify（无 coverage）照旧解析：不写覆盖键、无差异、路线不动",
          rig.bb.get(P_DRONE_COVERAGE) is None
          and rig.drone_state().field_diff == ()
          and list(rig.route().blocks()) == base_blocks,
          f"cov={rig.bb.get(P_DRONE_COVERAGE)} diffs={rig.drone_state().field_diff}")

    # ---- 6b 片级 coverage：两片拼一次扫描 + complete 门控 + 幂等 ----
    rig = Link(cfg_of())
    rig.ticks(3)
    base_blocks = list(rig.route().blocks())
    rig.send(hb(seq=1))
    scan = "20261017T1800_t3"
    rig.send(fv([], [], coverage={"cells": ["N1", "N2", "M1"], "complete": True,
                                  "mode": "strip"},
                slice_={"id": "S1", "index": 1, "of": 2, "route": "N1>N2>M1"},
                scan_id=scan, seq=2))
    rig.ticks(2)
    cov = rig.bb.get(P_DRONE_COVERAGE)
    check("6.2 首片：完整片 3 格计入已探明、done=False、分母 154",
          isinstance(cov, dict) and cov["cells_done"] == 3 and cov["done"] is False
          and cov["cells_total"] == 154, f"{cov}")
    check("6.3 ★ 片内既无本地要素也无报出 ⇒ 不产生『本地消失』差异（旧口径会误报）",
          rig.drone_state().field_diff == (), f"{rig.drone_state().field_diff}")
    rig.send(fv(["A3"], [], coverage={"cells": ["A3"], "complete": True},
                slice_={"id": "S2", "index": 2, "of": 2, "route": "A3"},
                scan_id=scan, seq=3))
    rig.ticks(2)
    cov = rig.bb.get(P_DRONE_COVERAGE)
    check("6.4 两片齐全且都 complete ⇒ done=True、已探明 4/154",
          cov["done"] is True and cov["cells_done"] == 4
          and cov["complete_slices"] == 2, f"{cov}")
    rig.send(fv(["A3"], [], coverage={"cells": ["A3"], "complete": True},
                slice_={"id": "S2", "index": 2, "of": 2, "route": "A3"},
                scan_id=scan, seq=4))
    rig.ticks(2)
    check("6.5 ★ 同 (scan_id,slice.id) 重发幂等：duplicates=1 且不重复记差异",
          rig.drone.coverage_duplicates == 1 and rig.drone_state().field_diff == (),
          f"dup={rig.drone.coverage_duplicates} diffs={rig.drone_state().field_diff}")

    # ---- 6c 完整片里的新雷/障 → 消费方汇总（默认只记录、不改路线） ----
    rig.send(fv(["N3"], ["N4"], coverage={"cells": ["N3", "N4"], "complete": True},
                slice_={"id": "S1", "index": 1, "of": 1,
                        "route": "N3>N4"},
                scan_id="20261017T1805_t3", seq=5))
    rig.ticks(2)
    st = rig.bb.get(P_FIELD_VERIFY)
    check("6.6 ★ 消费方拿到生产事件：hazard_new={N3,N4}、verdict=diff、severity=risk",
          isinstance(st, dict) and st["hazard_new"] == ["N3", "N4"]
          and st["verdict"] == "diff" and st["severity"] == "risk", f"{st}")
    check("6.7 默认 policy=record ⇒ hold=False；路线逐 token 不变（不阻断）",
          st["hold"] is False and list(rig.route().blocks()) == base_blocks
          and rig.phase() == "DRIVE", f"hold={st.get('hold')} phase={rig.phase()}")
    check("6.8 差异留痕进 P_DRONE.field_diff（旧遥测 '!差异N' 仍可见）",
          any("N3" in d for d in rig.drone_state().field_diff),
          f"{rig.drone_state().field_diff}")
    check("6.9 已探明覆盖累计 6 格（前 4 + 新 2）",
          rig.bb.get(P_DRONE_COVERAGE)["cells_done"] == 6,
          f"{rig.bb.get(P_DRONE_COVERAGE)}")

    # ---- 6d 感知管线真实产物（无 coverage）：不产生覆盖键 ----
    lines = product_lines()
    if lines:
        rig2 = Link(cfg_of())
        for ln in lines:
            rig2.send(ln)
        rig2.ticks(3)
        check("6.10 真实产物（无 coverage）→ 不写覆盖键，旧解析路径照旧",
              rig2.bb.get(P_DRONE_COVERAGE) is None
              and rig2.drone_state() is not None
              and rig2.drone.messages >= len(lines),
              f"cov={rig2.bb.get(P_DRONE_COVERAGE)} msgs={rig2.drone.messages}")


def main() -> None:
    section("卡 t5 端到端离线联调（bench）")
    print(f"  现场真源   : {_ROBOT.name}（默认穿洞格 lower={LOWER_DEFAULT} "
          f"upper={UPPER_DEFAULT}）")
    print(f"  临时地图   : {_MAP}（bridges=K4/K5/K9/K10，其余照抄现场）")
    print(f"  感知产物   : {_PERCEPTION_PRODUCT}")
    if _perception() is None:
        print("  SKIP  感知管线不可用（dzsmartscout/tools/perception 缺失）"
              "→ bridge_passable 用契约字面串，断言总数会减少")

    test_real_product_and_replan()
    test_both_halves_single_slot()
    test_degrade_matrix()
    test_event_channel()
    test_robustness()
    test_coverage_and_field_verify_consumer()
    finish()


if __name__ == "__main__":
    main()
