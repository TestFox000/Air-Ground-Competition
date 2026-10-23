#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机上重跑规划层：用已落盘的 field_state.json 生成裁决后的 route_plan + 渲染图。

识别的耗时部分（逐帧推理）不复算，只重跑"态势 -> 靶裁决 -> 规划 -> 渲染"，
用于靶先验裁决改动后的快速复验（整段视频侦察要 19 分钟，这一步只要几秒）。

用法（机上仓库根目录）：
    python3 tools/orin_replan_from_state.py [field_state.json] [输出目录]
    加 --allow-static-targets 关闭"静靶→桶"裁决

⚠️ 必须带 main() 守卫：本仓 tests/test_tools_smoke.py 会 import 每个 tools/*.py，
模块顶层读 sys.argv 会在导入期读到 pytest 的参数（实测 PermissionError）。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO / "src" / "ag_integrate") not in sys.path:
    sys.path.insert(0, str(REPO / "src" / "ag_integrate"))


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    allow_static = "--allow-static-targets" in args
    pos = [a for a in args if not a.startswith("--")]
    state_path = Path(pos[0]) if pos else Path("/tmp/ag_recon_bg/field_state.json")
    out_dir = Path(pos[1]) if len(pos) > 1 else state_path.parent
    if not state_path.is_file():
        print(f"[失败] field_state.json 不存在: {state_path}\n"
              f"  先跑一次完整侦察（video_recon.py）或指定正确路径", file=sys.stderr)
        return 2

    from ag_integrate import video_recon as vr

    state = json.loads(state_path.read_text(encoding="utf-8"))
    plan = vr.plan_from_state(state, out_dir,
                              static_target_is_bucket=not allow_static)
    print("[replan] snapshot:", json.dumps(plan["snapshot"], ensure_ascii=False))
    print("[replan] 裁决留痕: 桶=", plan["source"]["targets_reclassified_as_bucket"],
          " 轨道观测=", plan["source"]["targets_track_observed"])
    print("[replan] shoot:", plan["car_cmds"]["shoot_cells"],
          "| 含*:", "*" in plan["car_cmds"]["route"], "| check:", plan["check"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
