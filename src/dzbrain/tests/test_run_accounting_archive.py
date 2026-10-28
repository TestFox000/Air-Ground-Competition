# -*- coding: utf-8 -*-
r"""逐点台账 → 运行归档：取证链验收（2026-10-29 转角实验复盘产物）。

纯脚本、无 pytest（本项目惯例）：

    python src\dzbrain\tests\test_run_accounting_archive.py

## 为什么需要这个文件（真实事故驱动）
2026-10-29 实车转角实验（N10 → A10 → A1）。第二趟出现 **12 cm 横向偏差**，被当成
"转角收不回来"查了很久；实际是起步时车被摆错，A10 被 `mission.stall_timeout_s`
（12.0 s）的**停滞兜底强制跳过**，路线弧长在跳点处接错，后续几何全部失真。
`accounting()` 其实**早就有**这份台账（含每条跳点的原因），但它只出现在
`finish_report` 打到 stdout 的横幅里 —— **跑完就散**，事后无法机器判读。

本文件钉住两件事：
  1. `mission_accounting()` 取台账**永不许抛**（取证失败不能拖垮收尾）；
  2. `data_trust()` 的判据是"跳没跳点"而不是"横向数值好不好看"，且必须写进
     `summary.json` —— 让"这一场数据能不能用"成为磁盘上的事实。

全过 exit 0；任一失败 exit 1。
"""

from __future__ import annotations

import json
import pathlib
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve()
sys.path.insert(0, str(HERE.parent))                 # tests/（harness / fakes）
sys.path.insert(0, str(HERE.parents[2]))             # src/dzbrain

from harness import check, section, finish            # noqa: E402
from dzbrain.core.blackboard import Blackboard         # noqa: E402
from dzbrain.core.run_archive import (                 # noqa: E402
    RUN_FILES, SUMMARY_KEYS, RunArchive)
import dzbrain.main as M                               # noqa: E402

MAIN_SRC = pathlib.Path(M.__file__).read_text(encoding="utf-8")


# --------------------------------------------------------------------------- #
#  夹具
# --------------------------------------------------------------------------- #
def clean_accounting() -> dict:
    """干净场台账：零跳点、无未解释航点。"""
    return {
        "denominator": 5, "arrived": 5, "arrived_by": {"到达": 5},
        "stall_skips": 0, "cross_skips": 0, "skipped_total": 0, "skipped": {},
        "pending_skips": {}, "unexplained": [], "close_m": 0.12, "skip_over_m": 0.2,
    }


def dirty_accounting() -> dict:
    """脏场台账：A10（#1）被停滞兜底跳掉 —— 正是 2026-10-29 第二趟的形态。"""
    return {
        "denominator": 5, "arrived": 4, "arrived_by": {"到达": 3, "PIVOT": 1},
        "stall_skips": 1, "cross_skips": 0, "skipped_total": 1,
        "skipped": {"1": "停滞兜底 12.1s / 累计转 402°"},
        "pending_skips": {}, "unexplained": [], "close_m": 0.12, "skip_over_m": 0.2,
    }


def tele_bb(accounting, *, key: str = "mission_state") -> Blackboard:
    bb = Blackboard()
    bb.set("telemetry", {key: {"accounting": accounting}})
    return bb


class BoomBB:
    """位姿/遥测读取必炸的替身：取证代码必须扛住。"""

    def get(self, key, default=None):
        raise RuntimeError("黑盒炸了")


# --------------------------------------------------------------------------- #
section("A. mission_accounting 容错（负例：任何一种畸形输入都不许抛）")
# --------------------------------------------------------------------------- #
check("A1 无遥测 → None", M.mission_accounting(Blackboard()) is None)
check("A2 accounting 是 None → None", M.mission_accounting(tele_bb(None)) is None)

_bb_odd = Blackboard()
_bb_odd.set("telemetry", ["不是 dict"])
check("A3 telemetry 是 list → None", M.mission_accounting(_bb_odd) is None)

_bb_odd2 = Blackboard()
_bb_odd2.set("telemetry", {"mission_state": "字符串不是 dict"})
check("A4 mission_state 不是 dict → None", M.mission_accounting(_bb_odd2) is None)

check("A5 accounting 不是 dict → None",
      M.mission_accounting(tele_bb(["不是", "dict"])) is None)

try:
    _boom = M.mission_accounting(BoomBB())
    check("A6 黑盒 get 抛异常 → 返回 None（绝不向上抛）", _boom is None)
except Exception as exc:                             # noqa: BLE001
    check("A6 黑盒 get 抛异常 → 返回 None（绝不向上抛）", False,
          f"竟然抛出去了：{type(exc).__name__}: {exc}")

_acc = clean_accounting()
_got = M.mission_accounting(tele_bb(_acc))
check("A7 正常取到台账", isinstance(_got, dict) and _got.get("arrived") == 5)
check("A8 返回的是拷贝（不是黑盒里那份对象）",
      isinstance(_got, dict) and (_got is not _acc))
if isinstance(_got, dict):
    _got["arrived"] = 999
    _again = M.mission_accounting(tele_bb(_acc))
    check("A9 二次读取未被上一步污染",
          isinstance(_again, dict) and _again.get("arrived") == 5)


# --------------------------------------------------------------------------- #
section("B. data_trust 判据（跳点 ⇒ 不可评精度）")
# --------------------------------------------------------------------------- #
_t = M.data_trust(clean_accounting())
check("B1 零跳点 → trustworthy=True", _t.get("trustworthy") is True)
check("B2 零跳点的 why 里有到达点数", "到达 5" in str(_t.get("why")), str(_t.get("why")))

_t = M.data_trust(dirty_accounting())
check("B3 有停滞兜底 → trustworthy=False", _t.get("trustworthy") is False)
check("B4 脏场 why 明说不可评精度", "不可" in str(_t.get("why")), str(_t.get("why")))
check("B5 脏场把跳点清单带出来",
      str((_t.get("skipped") or {}).get("1") or "").startswith("停滞兜底"),
      json.dumps(_t.get("skipped"), ensure_ascii=False))

_only_cross = clean_accounting()
_only_cross["cross_skips"] = 2
_only_cross["skipped_total"] = 2
check("B6 只有越窗跳过也判 False", M.data_trust(_only_cross).get("trustworthy") is False)

_only_unexpl = clean_accounting()
_only_unexpl["unexplained"] = [3]
check("B7 未解释航点非空也判 False",
      M.data_trust(_only_unexpl).get("trustworthy") is False)

check("B8 台账缺失 → trustworthy=None（不是 False，也不是 True）",
      M.data_trust(None).get("trustworthy") is None)
check("B9 空 dict → None（“假 0”不许冒充干净场）",
      M.data_trust({}).get("trustworthy") is None)


# --------------------------------------------------------------------------- #
section("C. 归档接线：extra 进 summary.json，且六键齐不许被挤掉")
# --------------------------------------------------------------------------- #
with tempfile.TemporaryDirectory() as td:
    arch = RunArchive(td, clock=lambda: 1_700_000_000.0)
    summary = arch.finalize(
        reason="finish", duration_s=1.5, out_of_bounds=0, unknown_cells=0,
        extra={"accounting": dirty_accounting(),
               "data_trust": M.data_trust(dirty_accounting()),
               # 负例：extra 里塞必备键（必须被 setdefault 挡住，不许覆盖）
               "run_id": "HIJACK"})

    on_disk = json.loads((pathlib.Path(td) / arch.run_id / "summary.json")
                         .read_text(encoding="utf-8"))
    check("C1 summary.json 里有 accounting", "accounting" in on_disk)
    check("C2 summary.json 里有 data_trust", "data_trust" in on_disk)
    check("C3 data_trust 判为不可用（机器可判）",
          (on_disk.get("data_trust") or {}).get("trustworthy") is False)
    check("C4 六个必备键一个不少",
          all(k in on_disk for k in SUMMARY_KEYS),
          str([k for k in SUMMARY_KEYS if k not in on_disk]))
    check("C5 负例：extra 不许覆盖 run_id（setdefault 生效）",
          on_disk.get("run_id") != "HIJACK", str(on_disk.get("run_id")))
    check("C6 返回值与磁盘一致",
          str(summary.get("run_id")) == str(on_disk.get("run_id")))
    check("C7 summary.json 真的落地了",
          (pathlib.Path(td) / arch.run_id / "summary.json").is_file())

with tempfile.TemporaryDirectory() as td2:
    arch2 = RunArchive(td2, clock=lambda: 1_700_000_001.0)
    arch2.finalize(reason="exit_rc_0", duration_s=0.2)
    plain = json.loads((pathlib.Path(td2) / arch2.run_id / "summary.json")
                       .read_text(encoding="utf-8"))
    check("C8 负例：没传 extra 时不许凭空长出 accounting（防假字段骗验收）",
          "accounting" not in plain and "data_trust" not in plain, str(sorted(plain)))
    check("C9 没传 extra 时六键仍齐", all(k in plain for k in SUMMARY_KEYS))


# --------------------------------------------------------------------------- #
section("D. 真接线：run_live 里确实把台账写进了 summary")
# --------------------------------------------------------------------------- #
check("D1 main.py 调用了 mission_accounting(bb)",
      "mission_accounting(bb)" in MAIN_SRC)
check("D2 main.py 的 finalize 传了 extra=", "extra=_extra" in MAIN_SRC)
check("D3 report 里会带 accounting / data_trust 两个键",
      'report["accounting"]' in MAIN_SRC and 'report["data_trust"]' in MAIN_SRC)
check("D4 可信度为 False 时会把告警打到日志",
      "不可用于评控制精度" in MAIN_SRC)

# 行为级：黑盒里放一份脏台账 ⇒ 取出来的判据必须是"不可用"
bb_live = Blackboard()
bb_live.set("telemetry", {"mission_state": {"accounting": dirty_accounting()}})
_acc_live = M.mission_accounting(bb_live)
_trust_live = M.data_trust(_acc_live)
check("D5 端到端：脏台账 → 不可用（不是靠字符串匹配得出的）",
      _trust_live.get("trustworthy") is False,
      json.dumps(_trust_live.get("skipped"), ensure_ascii=False))


finish()
