#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视频侦察结果摘要（被人读，不参与决策）。

从 `orin_video_recon.sh` 末尾的嵌入式 heredoc 抽出来单独成文件：
shell 里混 Python 一旦引号/转义错一个字符，整段会以「未预期的记号 "("」
崩掉（2026-10-22 实测）。单独成文件后 shell 只做 `python3 本文件 <目录>`。

用法：
    python3 tools/orin_print_summary.py /tmp/ag_recon/<缓存目录>

退出码：0 正常；2 产物缺失或损坏（会打印"怎么修"）。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

MAX_MINES = 8          # 与 field_rules.MAX_MINES 一致的官方雷基线


def _load(path: Path):
    if not path.is_file():
        raise FileNotFoundError(
            f"缺少 {path.name}（目录：{path.parent}）\n"
            f"  该目录不是完整的识别产物，或识别还没跑到落盘那一步。\n"
            f"  处理：重跑识别（tools/orin_video_recon.sh <视频>），"
            f"或换一个缓存目录")
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"{path} 不是合法 JSON：{exc}\n"
            f"  多为落盘时被打断（磁盘满/进程被杀）。\n"
            f"  处理：加 --force 重跑识别，别用这份产物") from exc


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if not args:
        print("用法: orin_print_summary.py <产物目录>", file=sys.stderr)
        return 2
    out = Path(args[0])
    try:
        st = _load(out / "field_state.json")
        pl = _load(out / "route_plan.json")
    except (FileNotFoundError, ValueError) as exc:
        print(f"[失败] {exc}", file=sys.stderr)
        return 2

    mines = st.get("mines_video", [])
    n_mines = len(mines)
    kv = (st.get("frames_grid_ok"), st.get("frames_sampled"))
    print(f"  网格拟合帧 {kv[0]}/{kv[1]}   雷 {mines}"
          f"（{n_mines}/{MAX_MINES} 颗，官方基线 {MAX_MINES}）"
          f"{'' if n_mines >= MAX_MINES else '  ⚠ 可能漏检'}")
    print(f"  K列箭头 {st.get('passable_video')}   障碍 {st.get('obstacles_video')}"
          f"   靶 {st.get('targets_video')}")
    if st.get("targets_reclassified_as_bucket"):
        print(f"  靶裁决: 静靶→桶 {st['targets_reclassified_as_bucket']}"
              f"（场地真值口径，要保留静靶用 --allow-static-targets）")
    if st.get("phantoms_eliminated"):
        print(f"  幻影剔除: {st['phantoms_eliminated']}")

    cmds = pl.get("car_cmds") or {}
    print(f"  路线 {pl.get('route_len_cells', '?')} 格 / "
          f"压缩 {len(pl.get('compact') or [])} 点 / 转向 {pl.get('turns', '?')} 次")
    route = str(cmds.get("route", ""))
    print(f"  车命令: {route[:110]}{'…' if len(route) > 110 else ''}")
    print(f"  射击停靠: {cmds.get('shoot_cells')}   {pl.get('check')}")
    if "友方面禁火" in (pl.get("check") or {}):
        print(f"  友方面禁火: {pl['check']['友方面禁火']}")
    tracked = (pl.get("source") or {}).get("targets_track_observed") or {}
    if tracked:
        print(f"  轨道靶观测: {tracked}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
