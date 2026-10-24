# -*- coding: utf-8 -*-
"""evidential 线可靠性测试（2026-10-24 接线时建立，锁定语义防回归）。

覆盖四层：
    1) 协议 v2（0x11）conf 往返 / v1 兼容 / CRC / 边界
    2) EvidentialMap 语义（修复后的负证据地板/幽灵线性清除/粘性禁火/半衰）
    3) EvidentialFusion 端到端（雷/箭头/幽灵/红靶粘性 → WorldSnapshot）
    4) 低置信度场景专项（本次接线的动因）：
       0.4~0.6 真雷多帧积累确认 vs 0.3 幽灵被清 —— 二元票时代两者同权。

跑法：python src/ag_integrate/tests/test_evidential.py（与仓库其余测试同风格：
"=== N/N ===" 汇总行 + 非零退出码表示失败）。
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[1]      # <repo>/src/ag_integrate（包根父目录）
sys.path.insert(0, str(_PKG_ROOT))

from ag_integrate.evidential_fusion import EvidentialFusion          # noqa: E402
from ag_integrate.evidential_map import (EvidentialMap, EvidenceParams,  # noqa: E402
                                         MapSpot)
from ag_integrate.field_frame import CellRef, LETTERS, cell_to_world  # noqa: E402
from ag_integrate.intel_protocol import (IntelFrame, IntelObject,     # noqa: E402
                                         ObjectType, VERSION, VERSION_V2,
                                         build, detect_msg_id, parse)

PASS = FAIL = 0


def check(name: str, ok: bool, evidence: str = "") -> None:
    global PASS, FAIL
    if ok:
        PASS += 1
        print(f"  PASS  {name}" + (f"   [{evidence}]" if evidence else ""))
    else:
        FAIL += 1
        print(f"  FAIL  {name}" + (f"   [{evidence}]" if evidence else ""))


def cellref(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


def obj(name: str, typ: ObjectType, conf: float = 1.0) -> IntelObject:
    x, y = cell_to_world(cellref(name))
    return IntelObject(typ, x, y, 0.12, 0.12, conf)


def section(title: str) -> None:
    print(f"\n---- {title} " + "-" * (66 - len(title)))


def main() -> int:
    # ================= 1. 协议 v2 =================
    section("1. 协议 0x4001 v2（conf 字节）")
    f = IntelFrame(1000, (obj("B3", ObjectType.PAPER, 0.87),
                          obj("K9", ObjectType.PAPER, 0.5)))
    b2, b1 = build(f), build(f, version=VERSION)
    f2, f1 = parse(b2), parse(b1)
    check("v2 往返 conf 保真（±1/255）",
          abs(f2.objects[0].conf - 0.87) < 1 / 255 and abs(f2.objects[1].conf - 0.5) < 1 / 255,
          f"{[round(o.conf,3) for o in f2.objects]}")
    check("v1 往返 conf=1.0（老发射端零破坏）",
          all(o.conf == 1.0 for o in f1.objects) and len(b1) == len(b2) - 2,
          f"{len(b1)}B vs {len(b2)}B（2 目标 × 1B）")
    check("detect_msg_id 双版本识别",
          detect_msg_id(b1) == 0x4001 and detect_msg_id(b2) == 0x4001)
    check("版本字节区分", b1[4] == VERSION and b2[4] == VERSION_V2,
          f"0x{b1[4]:02x}/0x{b2[4]:02x}")
    bad = bytearray(b2); bad[12] ^= 0xFF
    try:
        parse(bytes(bad)); ok = False
    except ValueError:
        ok = True
    check("CRC 损坏拦截（v2 帧同样受保护）", ok)
    f3 = parse(build(IntelFrame(1, (obj("C7", ObjectType.OBSTACLE, 0.0),
                                     obj("C7", ObjectType.OBSTACLE, 1.0)))))
    check("conf 边界 0/1 不越界", f3.objects[0].conf == 0.0 and f3.objects[1].conf == 1.0)

    # ================= 2. EvidentialMap 语义 =================
    section("2. EvidentialMap（负证据地板修复后）")
    m = EvidentialMap()
    m.add_positive(cellref("K9"), ObjectType.PAPER, 0.95, t=100.0)
    check("单帧 0.95 过确认门（logit 2.94 > 2.5）",
          len(m.confirmed_only(100.0)) == 1)
    for t in range(101, 121):
        m.observe_cell(cellref("K9"), t=float(t), quality=0.5)
    l = m._logits[(10, 8)][ObjectType.PAPER]
    check("确认目标 20 帧负证据不磨损出局（地板=confirm）",
          abs(l - 2.5) < 0.01 and len(m.confirmed_only(120.0)) == 1,
          f"logit={l:.2f}")
    m2 = EvidentialMap()
    m2.add_positive(cellref("D9"), ObjectType.PAPER, 0.30, t=100.0)
    before2 = m2._logits[(3, 8)][ObjectType.PAPER]
    for t in (101.0, 102.0, 103.0):
        m2.observe_cell(cellref("D9"), t=t, quality=0.5)
    l2 = m2._logits[(3, 8)][ObjectType.PAPER]
    check("未确认幽灵被负证据持续衰减（v3 比例路径）",
          l2 < before2 * 0.75 and l2 >= EvidenceParams().logit_floor,
          f"{before2:.2f} → {l2:.2f}")
    m3 = EvidentialMap()
    m3.add_positive(cellref("C7"), ObjectType.TARGET_RED, 0.80, t=100.0)
    check("红靶单帧 0.8（≥0.73）粘性确认", len(m3.confirmed_only(100.0)) == 1)
    for t in range(101, 130):
        m3.observe_cell(cellref("C7"), t=float(t), quality=0.5)
    check("红靶粘性：负证据永不击穿", len(m3.confirmed_only(130.0)) == 1,
          "禁火铁律")
    m4 = EvidentialMap()
    m4.add_positive(cellref("F2"), ObjectType.OBSTACLE, 0.9, t=100.0)
    m4.add_positive(cellref("F2"), ObjectType.OBSTACLE, 0.9, t=101.0)
    check("两帧 0.9 确认障碍", len(m4.confirmed_only(101.0)) == 1)
    m5 = EvidentialMap()
    m5.add_positive(cellref("G5"), ObjectType.PAPER, 0.4, t=100.0)
    check("单帧 0.4 不进门（logit -0.41）", len(m5.confirmed_only(100.0)) == 0)

    # ================= 3. EvidentialFusion 端到端 =================
    section("3. EvidentialFusion → WorldSnapshot")
    fus = EvidentialFusion()
    fus.add_frame(IntelFrame(1000, (obj("K9", ObjectType.PAPER, 0.95),
                                    obj("B3", ObjectType.PAPER, 0.9),
                                    obj("D9", ObjectType.PAPER, 0.3))), recv_mono=100.0)
    snap = fus.snapshot
    check("帧1 快照形状（WorldSnapshot 同构）",
          snap is not None and [p.name for p in snap.passable] == ["K9"],
          f"箭头={[p.name for p in snap.passable]}")
    for i in range(6):
        fus.add_frame(IntelFrame(1100 + i, (obj("B3", ObjectType.PAPER, 0.9),)),
                      recv_mono=101.0 + i)
    snap = fus.snapshot
    names = [mm.name for mm in snap.mines]
    check("箭头 K9 持续在态势（负证据地板）", [p.name for p in snap.passable] == ["K9"])
    check("雷 B3 两帧确认", "B3" in names)
    check("幽灵 D9 被清（0.3 单帧）", "D9" not in names, f"雷={names}")
    check("PAPER 落 K 列=箭头、其余=雷（world_model 同语义）",
          all(p.name[:1] == "K" for p in snap.passable)
          and all(mm.name[:1] != "K" for mm in snap.mines))
    fus2 = EvidentialFusion()
    fus2.add_frame(IntelFrame(1, (obj("C7", ObjectType.TARGET_RED, 0.8),)),
                   recv_mono=1.0)
    red_then = [t.cell.name for t in fus2.snapshot.targets if not t.is_blue]
    for i in range(6):
        fus2.add_frame(IntelFrame(2 + i, ()), recv_mono=2.0 + i)
    red_now = [t.cell.name for t in fus2.snapshot.targets if not t.is_blue]
    check("红靶粘性跨帧（否决不闪断）", "C7" in red_then and "C7" in red_now,
          f"{red_then}->{red_now}")
    # 出界目标
    fus3 = EvidentialFusion()
    fus3.add_frame(IntelFrame(1, (IntelObject(ObjectType.PAPER, -9.9, -9.9, 0.1, 0.1, 0.9),)),
                   recv_mono=1.0)
    check("出界目标不进态势", fus3.snapshot is not None and not fus3.snapshot.mines)

    # ================= 4. 低置信度专项（本次接线动因） =================
    section("4. 低置信度场景（0.4~0.6 真雷 vs 0.3 幽灵）")
    fus4 = EvidentialFusion()
    frames = []
    for i in range(5):
        frames.append(IntelFrame(1000 + i, (
            obj("H4", ObjectType.PAPER, 0.45 + 0.03 * i),   # 真雷 0.45~0.57 爬升
            obj("E8", ObjectType.PAPER, 0.28),              # 幽灵恒低
        )))
    for i, fr in enumerate(frames):
        fus4.add_frame(fr, recv_mono=100.0 + i)
    names = [mm.name for mm in fus4.snapshot.mines]
    check("低置信真雷多帧积累确认（H4 @0.45~0.57）", "H4" in names, f"雷={names}")
    check("恒低幽灵不确认（E8 @0.28）", "E8" not in names)
    # coverage：每帧全场 observe（检测格因同帧互斥少计 1 次 clear_view，
    # 语义正确——"检测到"本就不该算"看清没有"）；>0.95 即视为全场覆盖。
    check("decision 可观测（含 coverage）", fus4.decision.coverage_frac > 0.95
          and fus4.decision.mines_confirmed >= 1, str(fus4.decision))
    # 对照：整帧替换语义下的闪断（旧 world_model 单帧路径）——证明升级价值。
    # 语义：丢帧的那一拍，雷从态势里消失（下一帧恢复）——低置信目标间歇
    # 漏检时该闪断每轮都发生，门控（min_mines）跟着抖动。
    from ag_integrate.world_model import WorldModel
    wm = WorldModel()
    wm.update(frames[0], recv_mono=100.0)                 # 有雷帧
    before = [mm.name for mm in wm.snapshot.mines]
    wm.update(IntelFrame(9999, ()), recv_mono=100.5)      # 漏检帧
    during = [mm.name for mm in wm.snapshot.mines] if wm.snapshot else []
    check("对照：整帧替换语义丢一帧即闪断（fusion 无此问题）",
          "H4" in before and "H4" not in during,
          f"旧链路 {before} →漏检帧→ {during}")

    print("\n" + "=" * 72)
    print(f"=== {PASS + FAIL} 项：{PASS} 通过 / {FAIL} 失败 ===")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
