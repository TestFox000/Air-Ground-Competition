# -*- coding: utf-8 -*-
"""F5（2026-10-25 实车 run4 回归）：横向重锚改成**分拍收敛**。

背景（run4 证据）：
  车到 E8->F4 段，定位报 `横向重锚 0.303 m`，**一步**把信念拽到足点 →
  下一拍 `位姿突跳 0.313 m > 0.20 m` → FixTask 判"清窗" + 漂移护栏累到 8 拍
  → `drift_stop` 把车停死。修法：单拍最多拉 `reanchor_step_max_m`（默认 0.12 m），
  残差 > 触发门限后继续分拍拉，直到剩余 ≤ `reanchor_done_m`（默认 0.02 m）才闩锁顶点。

本测试**不启动线程**：用 `object.__new__` 造一个只带 `_reanchor` 所需字段的实例，
用手算的"路径沿 y 轴、横向残差 d"几何逐拍喂进去。

跑法：`python src/dzbrain/tests/test_reanchor_step.py`，全过 exit 0。
"""

from __future__ import annotations

import math
import pathlib
import sys
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:
    sys.stdout.reconfigure(errors="replace")  # type: ignore[union-attr]
except Exception:                              # pragma: no cover
    pass

from dzbrain.tasks.localization import LocalizationTask                      # noqa: E402

_PASS = 0
_FAIL = 0
_FAILED: list[str] = []


def ok(name: str, cond: bool, detail: str = "") -> None:
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print(f"  PASS  {name}")
    else:
        _FAIL += 1
        _FAILED.append(name)
        print(f"  FAIL  {name}  {detail}")


class _Bus:
    def __init__(self) -> None:
        self.events: list[tuple[str, dict]] = []

    def publish(self, topic, **kw) -> None:
        self.events.append((topic, kw))


class _Warn:
    def allow(self, key: str, now: float) -> bool:
        return True


class _Fake:
    """只借 `_reanchor` 一个方法（TaskThread 的 Thread 基类不能裸 new）。"""

    _reanchor = LocalizationTask._reanchor


def _task():
    t = _Fake()
    t.name = "Localization"
    t._path = object()                      # 非 None 即可（_reanchor 不再用 path）
    t.drift_warn_m = 0.30
    t.reanchor_pull_max_m = 0.35
    t.reanchor_step_max_m = 0.12
    t.reanchor_done_m = 0.02
    t._reanchor_idx = -1
    t._reanchor_active = False
    t._reanchor_active_idx = -1
    t.reanchored = 0
    t.drift_run = 0
    t.bus = _Bus()
    t._warn = _Warn()
    t.logs: list[str] = []
    t._log = t.logs.append
    t.stored: list[tuple] = []
    t._store_drift = lambda pose, dist, now: t.stored.append((pose.x, pose.y, dist))
    t.refused: list[tuple] = []
    t._refuse_anchor = lambda *a, **k: t.refused.append((a, k))
    return t


def _proj(x: float, y: float, foot_x: float, foot_y: float):
    return SimpleNamespace(x=foot_x, y=foot_y, dist=math.hypot(x - foot_x, y - foot_y))


def main() -> int:
    print("=" * 72)
    print("F5 横向重锚分拍收敛")
    print("=" * 72)

    # 几何：路线沿 y 轴 (x=0)，车在 x=0.303 ⇒ 横向残差 0.303（= run4 实测值）
    gate = (7, SimpleNamespace(block="K7"))
    t = _task()
    x, y = 0.303, 1.0
    steps: list[float] = []
    poses = []
    for _ in range(5):
        proj = _proj(x, y, 0.0, y)
        pose = t._reanchor(proj, x, y, 0.0, gate, 1.0)
        if pose is None:
            break
        dx = math.hypot(pose.x - x, pose.y - y)
        steps.append(dx)
        x, y = pose.x, pose.y
        poses.append((x, y))

    ok("1.1 首拍只拉 step_max（0.12），不是一步 0.303",
       abs(steps[0] - 0.12) < 1e-9, f"steps={steps}")
    ok("1.2 每拍位移都 ≤ 0.12（< 0.20 突跳门限）",
       all(s <= 0.12 + 1e-9 for s in steps), f"steps={steps}")
    ok("1.3 3 拍内收敛到折线（x≈0，剩余 ≤ done）",
       abs(x) < 1e-9 and len(steps) == 3, f"x={x:.4f} steps={steps}")
    ok("1.4 收敛计数只加 1，且顶点已闩锁",
       t.reanchored == 1 and t._reanchor_idx == 7,
       f"reanchored={t.reanchored} latch={t._reanchor_idx}")
    ok("1.5 收敛完成后不再反复拉（第 4 拍 no-op）",
       t._reanchor(_proj(0.0, y, 0.0, y), 0.0, y, 0.0, gate, 2.0) is None)

    # 场景 2：没被触发过（残差 0.20 < warn 0.30）⇒ **不动**（不制造误差）
    t2 = _task()
    out2 = t2._reanchor(_proj(0.20, 2.0, 0.0, 2.0), 0.20, 2.0, 0.0, gate, 1.0)
    ok("2.1 未触发的 0.20 残差 ⇒ no-op（不凭空纠偏）",
       out2 is None and t2.reanchored == 0 and t2._reanchor_active is False)
    # 2.2：一旦触发过，后续 sub-warn 残差（0.183 / 0.063）必须继续拉（见 1.2/1.3 的 steps）
    ok("2.2 触发后的 sub-warn 残差继续收敛（steps 中 0.183→0.063 两拍）",
       len(steps) == 3 and abs(steps[1] - 0.12) < 1e-9 and abs(steps[2] - 0.063) < 1e-9,
       f"steps={steps}")

    # 场景 3：换顶点 ⇒ 收敛状态作废（不能拿旧顶点的 active 继续拉）
    t3 = _task()
    t3._reanchor_active = True
    t3._reanchor_active_idx = 7
    gate2 = (9, SimpleNamespace(block="K9"))
    out = t3._reanchor(_proj(0.10, 3.0, 0.0, 3.0), 0.10, 3.0, 0.0, gate2, 1.0)
    ok("3.1 换顶点后残差 0.10 ≤ warn 且非本顶点收敛 ⇒ no-op",
       out is None and t3._reanchor_active is False)

    # 场景 4：残差 > pull_max ⇒ 拒绝 + 记账（旧行为不变）
    t4 = _task()
    out4 = t4._reanchor(_proj(0.50, 4.0, 0.0, 4.0), 0.50, 4.0, 0.0, gate, 1.0)
    ok("4.1 0.50 > pull_max 0.35 ⇒ 拒绝并记账",
       out4 is None and len(t4.refused) == 1 and t4._reanchor_idx == 7)

    print()
    print("=" * 72)
    print(f"结果：{_PASS}/{_PASS + _FAIL} 通过" + ("" if not _FAIL else f"，{_FAIL} 失败"))
    return 0 if not _FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
