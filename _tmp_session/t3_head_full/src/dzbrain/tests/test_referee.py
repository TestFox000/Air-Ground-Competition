# -*- coding: utf-8 -*-
r"""裁判回传 → 命中闭环 测试（修的是"声明了但没人生产"的静默失败）。

运行：python src\dzbrain\tests\test_referee.py

本套件钉死三件事：
  1. **帧格式取证**：解析顺序逐条对上车端真源 `judgment.cpp` / `judgment.h`（源码级断言，
     C++ 侧一改这里就红，不会等到实车打飞才发现）；
  2. **计数语义**：命中 = `targethp` 下降；1 Hz 心跳（值与实帧相同）**不得重复计数**；
     新一局 HP 复位不得误记命中；
  3. **接线**：用假 `rclpy` 注入实例化**真实的 `RosBridge`**，直接调回调，
     验证 `/HpAndHitmsg` 真的被订阅、命中真的进了 `take_hits()`、
     `referee_hp` 有效后旧的 `/referee/hit` 兜底话题被**关掉**（防双计）。
"""

from __future__ import annotations

import pathlib
import re
import sys
import types

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from dzbrain.io import contract as ct                          # noqa: E402
from dzbrain.io import referee as rf                            # noqa: E402
from harness import check, finish, section                      # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]
CPP = WS / "src" / "dzjudgment" / "src" / "judgment.cpp"
HDR = WS / "src" / "dzjudgment" / "src" / "judgment.h"

# =========================================================================== #
#  1. 帧格式取证：解析顺序必须与车端 C++ 完全一致
# =========================================================================== #
section("1. 帧格式取证（对车端真源做源码级钉死）")

cpp_src = CPP.read_text(encoding="utf-8", errors="replace")
hdr_src = HDR.read_text(encoding="utf-8", errors="replace")

m = re.search(r"void dzjudgment::publish_referee_state\(\).*?\n\}", cpp_src, re.S)
check("judgment.cpp 里能找到 publish_referee_state()", m is not None)
body = m.group(0) if m else ""
fields = re.findall(r"msg\.data\.push_back\((\w+)\);", body)
check("★ 发布字段顺序 = [对方HP, 己方HP, 被击打方位, 累计发数, 剩余弹药]",
      fields == ["targethp", "self_hp", "Bing_hit", "Shooting_Count", "ammo"], str(fields))
check("★ 帧长 5 与 FRAME_LEN 一致", len(fields) == rf.FRAME_LEN, f"{len(fields)} vs {rf.FRAME_LEN}")

m07 = re.search(r"case 0x07:(.*?)default:", cpp_src, re.S)
check("judgment.cpp 里能找到 case 0x07", m07 is not None)
c07 = m07.group(1) if m07 else ""
check("0x07 帧体 [4] → targethp", "targethp = charArray[4];" in c07)
check("0x07 帧体 [5] → self_hp", "self_hp = charArray[5];" in c07)
check("0x07 帧体 [6] → Bing_hit", "Bing_hit = charArray[6];" in c07)
check("0x07 帧体 [8] → ammo", "ammo = charArray[8];" in c07)
check("★ 车端有长度守卫（dataLength < 6 丢弃）", "dataLength < 6" in c07)
check("★ 我们也有长度守卫（< 5 丢弃、不猜字段）", rf.FRAME_LEN == 5)

check("judgment.h: targethp 初值 0（裁判未接时不产生假命中）",
      "uint8_t targethp = 0;" in hdr_src)
check("judgment.h: Bing_hit 初值 0x10 = 未被击打",
      "Bing_hit = 0x10" in hdr_src)
check("judgment.h: ammo 初值 10", "uint8_t ammo = 10;" in hdr_src)
check("★ Bing_hit 语义是【我方被击打方位】而非命中确认（纠正旧文档）",
      "是否被敌方击打" in hdr_src)
check("话题名与车端发布者一致（HpAndHitmsg）",
      'create_publisher<std_msgs::msg::UInt8MultiArray>("HpAndHitmsg"' in cpp_src)
check("契约登记的名字/类型与车端一致",
      ct.topic("referee_hp").name == "/HpAndHitmsg"
      and ct.topic("referee_hp").msg_type == "std_msgs/UInt8MultiArray"
      and ct.topic("referee_hp").direction == "sub",
      f"{ct.topic('referee_hp').name} / {ct.topic('referee_hp').msg_type}")

# =========================================================================== #
#  2. RefereeTracker 纯逻辑
# =========================================================================== #
section("2. RefereeTracker 纯逻辑（命中 / 挨打 / 弹药 / 边界）")


def frame(target=5, self_hp=5, bing=rf.BING_NOT_HIT, shots=0, ammo=10):
    return [target, self_hp, bing, shots, ammo]


t = rf.RefereeTracker()
check("未收到帧时 ammo = None（未知，不假设）", t.ammo is None)
check("未收到帧时 valid_seen=False", t.valid_seen is False)
check("age(now) 未收到时为 inf", t.age(1.0) == float("inf"))

d0 = t.feed(frame(), 10.0)
check("首帧只建基线、不计命中", d0.hits == 0 and t.hits_total == 0, f"hits={d0.hits}")
check("首帧后 valid_seen=True", t.valid_seen is True)
check("首帧 frame 带回状态", d0.frame is not None and d0.frame.targethp == 5)
check("首帧后 ammo 跟随", t.ammo == 10)
check("age 正确", t.age(10.5) == 0.5)

d1 = t.feed(frame(target=4, shots=1, ammo=9), 11.0)
check("★ targethp 5→4 = 命中 1 次", d1.hits == 1 and t.hits_total == 1)
check("硬件发数镜像", d1.frame.shots_hw == 1)
check("弹药镜像 10→9", t.ammo == 9)

d2 = t.feed(frame(target=2, shots=3, ammo=7), 12.0)
check("★ targethp 4→2 = 命中 2 次（按下降量记账）", d2.hits == 2 and t.hits_total == 3)

# 心跳：值与实帧完全相同 → 不得重复计数（这是本模块存在的第二理由）
before = t.hits_total
for k in range(50):
    t.feed(frame(target=2, shots=3, ammo=7), 12.1 + k * 0.1)
check("★★ 50 次同值心跳不产生任何命中（不按'收到帧'计数）",
      t.hits_total == before, f"{before} → {t.hits_total}")
check("心跳仍计帧数（排障用）", t.frames == 3 + 50)

d3 = t.feed(frame(target=5, shots=3, ammo=7), 20.0)
check("★ targethp 上升 = 新一局/复位：不计命中", d3.hits == 0 and d3.reset is True)
check("复位计数 resets=1", t.resets == 1)
check("复位后再次下降照常记命中",
      t.feed(frame(target=4, shots=4, ammo=6), 21.0).hits == 1 and t.hits_total == 4)

# 挨打：self_hp 才是计数依据；Bing_hit 只记方位（避免双计）
d4 = t.feed(frame(target=4, self_hp=4, bing=0x02, shots=4, ammo=6), 22.0)
check("★ self_hp 下降 = 我方挨打 1 次", d4.self_hits == 1 and t.self_hits_total == 1)
check("挨打不产生命中", d4.hits == 0)
check("方位可读（0x02 → 前）", d4.frame.bing_name == "前" and d4.frame.being_hit is True)

d5 = t.feed(frame(target=4, self_hp=4, bing=0x03, shots=4, ammo=6), 23.0)
check("★ Bing_hit 变化但 HP 未变 → 不重复计挨打（防双计）",
      d5.self_hits == 0 and t.self_hits_total == 1)

# 非法帧：绝不污染账本
n_hits, n_mal = t.hits_total, t.malformed
n_frames, n_seen = t.frames, t.last_seen_at
for bad in ([], [1], [1, 2, 3, 4], None):
    d = t.feed(bad, 24.0)
    check(f"坏帧 {bad!r} → frame=None 且无命中",
          d.frame is None and d.hits == 0 and d.self_hits == 0)
check("★ 坏帧计入 malformed（可观测，不静默）", t.malformed == n_mal + 4)
check("坏帧不改变命中总数", t.hits_total == n_hits)
check("坏帧不计入 frames", t.frames == n_frames, f"{n_frames} → {t.frames}")
check("★ 坏帧不刷新 last_seen（否则心跳超时永远判不出来）",
      t.last_seen_at == n_seen, f"{n_seen} → {t.last_seen_at}")
check("坏帧后 age 照常走（说明用的还是上一合法帧的时间戳）",
      t.age(24.0) > 0.0 and t.age(24.0) != float("inf"))

t_big = rf.RefereeTracker()
t_big.feed([300, 0, 0, 0, 0], 1.0)          # >255：按字节语义掩码
t_big.feed([255, 0, 0, 0, 0], 2.0)
check("取值按字节掩码（300 & 0xFF = 44，44→255 是上升=复位，不记命中）",
      t_big.hits_total == 0 and t_big.resets == 1)

check("describe() 无帧时提示裁判未开启", "裁判系统未开启" in rf.RefereeTracker().describe())
check("describe() 有帧时含关键字段",
      all(k in t.describe() for k in ("对方HP", "弹药", "命中")))

# =========================================================================== #
#  3. 长回放：10 发 5 中 + 心跳穿插 + 新一局
# =========================================================================== #
section("3. 长回放（模拟一局：10 发 5 中、心跳穿插、然后新一局）")

r = rf.RefereeTracker()
t_now = 100.0
hits_script = {2: 4, 4: 3, 6: 2, 8: 1, 10: 0}   # 第 N 发命中：5→4→3→2→1→0 = 5 次
hp = 5
r.feed(frame(target=hp, shots=0, ammo=10), t_now)
for shot in range(1, 11):
    if shot in hits_script:
        hp = hits_script[shot]
    ammo = max(0, 10 - shot)
    r.feed(frame(target=hp, shots=shot, ammo=ammo), t_now + shot * 0.5)
    # 每发之间穿插 1 Hz 心跳（值与上一帧完全相同）
    for _ in range(2):
        t_now += 0.5
        r.feed(frame(target=hp, shots=shot, ammo=ammo), t_now)
    t_now += 0.5

check("★ 10 发 5 中 → 命中恰好 5 次（心跳未污染）", r.hits_total == 5, f"{r.hits_total}")
check("弹药跟到 0", r.ammo == 0)
check("硬件发数跟到 10", r.state.shots_hw == 10)
check("无坏帧", r.malformed == 0)
check("未发生复位", r.resets == 0)
check("挨打 0 次（self_hp 未变）", r.self_hits_total == 0)

r.feed(frame(target=5, shots=0, ammo=10), t_now + 5.0)      # 新一局
check("新一局：targethp 回满 → resets=1 且命中数不变",
      r.resets == 1 and r.hits_total == 5)
r.feed(frame(target=3, shots=1, ammo=9), t_now + 5.5)
check("新一局内命中照常记账（3 血差 = 2 次）", r.hits_total == 7)

# =========================================================================== #
#  4. RosBridge 接线（假 rclpy 注入 → 跑真实回调）
# =========================================================================== #
section("4. RosBridge 接线（假 rclpy 注入，调真实回调）")


class _Msg:
    def __init__(self, data=None, **kw):
        if data is not None:
            self.data = data
        self.__dict__.update(kw)


def _install_fake_ros() -> None:
    """把 rclpy / 消息包塞进 sys.modules —— 只为验证**我们的接线**，不模拟 ROS 运行时。"""
    def _mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    class _Enum:
        def __init__(self, name):
            self.name = name

        def __repr__(self):
            return self.name

    class QoSProfile:
        def __init__(self, **kw):
            self.__dict__.update(kw)

    class HistoryPolicy:
        KEEP_LAST, KEEP_ALL = _Enum("KEEP_LAST"), _Enum("KEEP_ALL")

    class ReliabilityPolicy:
        RELIABLE, BEST_EFFORT = _Enum("RELIABLE"), _Enum("BEST_EFFORT")

    class DurabilityPolicy:
        VOLATILE, TRANSIENT_LOCAL = _Enum("VOLATILE"), _Enum("TRANSIENT_LOCAL")

    class _Pub:
        def __init__(self):
            self.sent: list = []

        def publish(self, msg):
            self.sent.append(msg)

    class Node:
        def __init__(self, name):
            self.name = name
            self.subs: dict = {}
            self.pubs: dict = {}
            self._handles: list = []

        def create_subscription(self, msg_type, topic, cb, qos):
            self.subs[topic] = (msg_type, cb)
            h = object()
            self._handles.append(h)
            return h

        def create_publisher(self, msg_type, topic, qos):
            pub = _Pub()
            self.pubs[topic] = (msg_type, pub)
            return pub

        def destroy_node(self):
            pass

    rclpy = _mod("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda *a, **k: None
    _mod("rclpy.node", Node=Node)
    _mod("rclpy.qos", QoSProfile=QoSProfile, HistoryPolicy=HistoryPolicy,
         ReliabilityPolicy=ReliabilityPolicy, DurabilityPolicy=DurabilityPolicy)

    class _Vec:
        def __init__(self):
            self.x = self.y = self.z = 0.0

    class Twist:
        def __init__(self):
            self.linear, self.angular = _Vec(), _Vec()

    def _mk(name):
        return type(name, (_Msg,), {})

    std = types.ModuleType("std_msgs.msg")
    for n in ("Int32", "Int32MultiArray", "String", "UInt8", "UInt8MultiArray", "Float32"):
        setattr(std, n, _mk(n))
    sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    sys.modules["std_msgs.msg"] = std
    sys.modules["std_msgs"].msg = std

    _mod("geometry_msgs")
    _mod("geometry_msgs.msg", Twist=Twist)
    sys.modules["geometry_msgs"].msg = sys.modules["geometry_msgs.msg"]
    _mod("nav_msgs")
    _mod("nav_msgs.msg", Odometry=_mk("Odometry"))
    sys.modules["nav_msgs"].msg = sys.modules["nav_msgs.msg"]
    _mod("sensor_msgs")
    _mod("sensor_msgs.msg", Imu=_mk("Imu"))
    sys.modules["sensor_msgs"].msg = sys.modules["sensor_msgs.msg"]


try:
    import rclpy  # noqa: F401
    check("本机有真 rclpy（本组仍用假体，保证跨平台结论一致）", True)
except Exception:
    check("本机无 rclpy → 注入假体（不影响结论）", True)
_install_fake_ros()

from dzbrain.io.ros_io import RosBridge                       # noqa: E402

bridge = RosBridge("dzbrain_test", log=lambda *_: None)
subs = set(bridge.node.subs)
check("★ 真的订阅了 /HpAndHitmsg", "/HpAndHitmsg" in subs, f"共 {len(subs)} 个订阅")
check("订阅类型是 UInt8MultiArray",
      bridge.node.subs["/HpAndHitmsg"][0].__name__ == "UInt8MultiArray")
check("旧的兜底话题也仍在订阅（未来桥接可用）",
      "/referee/hit" in subs and "/referee/ammo" in subs)

hp_cb = bridge.node.subs["/HpAndHitmsg"][1]
hit_cb = bridge.node.subs["/referee/hit"][1]
ammo_cb = bridge.node.subs["/referee/ammo"][1]
HP = bridge.node.subs["/HpAndHitmsg"][0]
I32 = bridge.node.subs["/referee/hit"][0]

check("初始 ammo = None（未知）", bridge.ammo() is None)
hp_cb(HP(data=[5, 5, 0x10, 0, 10]))
check("首帧后 ammo=10（真源生效）", bridge.ammo() == 10)
check("首帧不算命中", bridge.take_hits() == 0)

hp_cb(HP(data=[4, 5, 0x10, 1, 9]))
check("★ 命中经回调进账 → take_hits()==1", bridge.take_hits() == 1)
check("★ take_hits() 取走即清零", bridge.take_hits() == 0)
check("弹药跟随真源 9", bridge.ammo() == 9)
check("硬件发数可取", bridge.referee_shots_hw() == 1)

for _ in range(20):
    hp_cb(HP(data=[4, 5, 0x10, 1, 9]))          # 心跳
check("★ 20 次心跳后仍无命中", bridge.take_hits() == 0)

hp_cb(HP(data=[1, 2, 3]))                        # 坏帧
check("★ 坏帧被丢弃且计数（referee_bad）", bridge.stats.referee_bad == 1)
check("坏帧不产生命中", bridge.take_hits() == 0)

hit_cb(I32(data=1))
check("★★ 已有 referee_hp 真源时，/referee/hit 兜底被关掉（防双计）",
      bridge.take_hits() == 0)
hp_cb(HP(data=[3, 5, 0x10, 2, 8]))                   # targethp 4→3 = 1 次命中
check("★ 真源照常记账（4→3 = 1 次命中，心跳未污染账本）", bridge.take_hits() == 1)

hp_cb(HP(data=[3, 4, 0x04, 2, 8]))                   # self_hp 5→4 = 1 次挨打，targethp 不变
check("挨打经回调计数进 stats.self_hits", bridge.stats.self_hits == 1)
check("挨打不进命中账本", bridge.take_hits() == 0)

check("referee() 暴露解析器（排障）", bridge.referee().hits_total == 2)
check("referee_age() 是有限值（刚收到帧）", bridge.referee_age() < 5.0)

# 未收到任何真源帧时，兜底话题必须可用（否则桥接节点形同虚设）
bridge2 = RosBridge("dzbrain_test2", log=lambda *_: None)
h2 = bridge2.node.subs["/referee/hit"][1]
a2 = bridge2.node.subs["/referee/ammo"][1]
I32b = bridge2.node.subs["/referee/hit"][0]
h2(I32b(data=1))
check("★ 无真源时 /referee/hit 兜底生效", bridge2.take_hits() == 1)
a2(I32b(data=3))
check("★ 无真源时 /referee/ammo 兜底生效", bridge2.ammo() == 3)

summ = bridge.summary()
check("summary() 暴露裁判字段（否则线上无从排障）",
      all(k in summ for k in ("referee_msgs", "referee_bad", "self_hits", "referee_shots_hw")),
      str({k: summ[k] for k in ("referee_msgs", "referee_bad", "self_hits")}))
check("summary 里 ammo 与真源一致", summ["ammo"] == 8)

finish()
