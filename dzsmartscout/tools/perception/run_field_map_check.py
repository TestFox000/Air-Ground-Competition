# -*- coding: utf-8 -*-
r"""t59 · L1 出图真图核查：**官方贴图 → 感知 → FieldMap → 与 config/map.yaml 逐条对账**。

做什么（全部只读外部文件，只写本包 ``reports/``）：

1. 用 ``grid.register_from_arena_frame`` **自动标定**官方图纸（外框 + 灰格线，无需手点角点）；
2. 跑 t2 管线 → :class:`perception.field_map.FieldMap`（``map:`` 块与 map.yaml 同 schema）；
3. 与 ``dzrobot_v2/config/map.yaml`` 的现役真值**逐类逐格对账**（雷/障/靶/桥 + 起终点/隔板）；
4. 出图 ``reports/field_map_<name>.{json,yaml}`` + 报告 ``reports/field_map_check.md``；
5. 顺带把**每格置信度**、**车位姿**、**蓝面朝向（能判才判）**写进证据块。

    cd dzsmartscout/tools
    python -m perception.run_field_map_check              # 官方图（现役真值）
    python -m perception.run_field_map_check --all        # 官方图 + 旧图纸（冻结真值）
退出码：0 = 官方图逐格一致；1 = 有差异（差异逐条打印）。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import sys

if __package__ in (None, ""):            # 允许 `python perception/run_field_map_check.py`
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    __package__ = "perception"

import numpy as np
from PIL import Image

from . import field_map as FM
from .colors import MAP_IMAGE, as_rgb_array
from .grid import GridSpec, register_from_arena_frame, register_from_corners, CORNER_ORDER
from .objects import MAP_IMAGE_SPECS, detect_field_objects
from .car_pose import CarSpec, best_car
from .overlay import draw_registration

HERE = pathlib.Path(__file__).resolve().parent
V1 = HERE.parents[2]
REPORTS = HERE / "reports"
MAP_YAML = V1 / "dzrobot_v2" / "config" / "map.yaml"
OFFICIAL = V1 / "_tmp_session" / "t39_map_official" / "map_official.png"
DRAWING = V1 / "dzrobot_v2" / "_tmp_session" / "t28_newmap" / "map_src.png"
IDENTIFY = V1 / "dzrobot_v2" / "_tmp_session" / "t28_newmap" / "identify_report.txt"


def _truth_from_yaml() -> dict:
    """读现役真值（不依赖 PyYAML；只认 ``map:`` 块的键）。"""
    return FM.parse_map_yaml(MAP_YAML.read_text(encoding="utf-8"))


def _check_one(name: str, img, reg, *, truth: dict, image_name: str,
               verbose: bool = True) -> tuple[FM.FieldMap, FM.Reconciliation]:
    objs = detect_field_objects(img, reg, MAP_IMAGE, MAP_IMAGE_SPECS)
    car = best_car(img, reg, MAP_IMAGE, CarSpec(cls="car", nose_cls="car_nose"))
    # 蓝面朝向：官方图上靶面不可见 ⇒ 应如实 unknown（有像素证据）
    bf = FM.estimate_blue_face(img, reg, truth.get("targets") or [], )
    fm = FM.build_field_map(
        objs, reg, name=f"{name}", start=truth.get("start"), end=truth.get("end"),
        barrier_col_from=truth.get("barrier_col_from"),
        barrier_row_hi=int(truth.get("barrier_row_hi") or 6),
        barrier_row_lo=int(truth.get("barrier_row_lo") or 7),
        car=car, blue_face=bf, image=image_name)
    rec = fm.reconcile(truth)
    if verbose:
        print(f"\n[{name}] 注册：{reg.notes[-1] if reg.notes else '手点'}")
        print(f"[{name}] 残差 {reg.residual_px:.3f} px；格距 "
              f"{reg.cell_size_px('G6')[0]:.2f}×{reg.cell_size_px('G6')[1]:.2f} px "
              f"（1 px ≈ {0.35 / reg.cell_size_px('G6')[0] * 1000:.1f} mm）")
        print(rec.report())
        if fm.car:
            print(f"[{name}] 车位姿：{json.dumps(fm.car, ensure_ascii=False)}")
        print(f"[{name}] 蓝面朝向：{bf.facing}（蓝 {bf.blue_frac:.4f} / 红 {bf.red_frac:.4f}）"
              f" —— {bf.evidence.get('reason', bf.evidence.get('note', ''))}")
        for cls, per in sorted(fm.confidence.items()):
            c = ", ".join(f"{b}:{v:.2f}" for b, v in sorted(per.items()))
            print(f"[{name}] 置信度 {cls:9s} {c}")
    return fm, rec


def _drawing_truth() -> dict:
    """旧图纸的冻结点位（与 ``run_real_checks.DRAWING_TRUTH_FROZEN`` 同源）。"""
    from .run_real_checks import DRAWING_TRUTH_FROZEN as T
    return {"mines": T["mine"], "obstacles": T["obstacle"], "targets": T["target"],
            "bridges": T["bridge"], "start": "N10", "end": "N5",
            "barrier_col_from": "D", "barrier_row_hi": 6, "barrier_row_lo": 7}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="run_field_map_check")
    ap.add_argument("--all", action="store_true", help="连旧图纸（冻结真值）一起跑")
    args = ap.parse_args(argv)
    REPORTS.mkdir(parents=True, exist_ok=True)
    truth = _truth_from_yaml()
    print(f"现役真值（{MAP_YAML.relative_to(V1).as_posix()}）："
          f"{json.dumps(truth, ensure_ascii=False)}")

    results: list[dict] = []
    ok_all = True

    # ---- 官方贴图：真值 = 现役 map.yaml（本卡的核心验收）----
    if OFFICIAL.is_file():
        img = as_rgb_array(Image.open(OFFICIAL))
        reg = register_from_arena_frame(img)
        fm, rec = _check_one("official", img, reg, truth=truth, image_name=OFFICIAL.name)
        draw_registration(img, reg).save(REPORTS / "field_map_official_registration.png")
        jp, yp = FM.write_field_map(fm, REPORTS, stem="field_map_official")
        results.append({"name": "official", "image": str(OFFICIAL), "reconciliation": rec.to_json(),
                        "field_map": fm.to_json_dict(), "json": str(jp), "yaml": str(yp)})
        ok_all = ok_all and rec.ok
    else:
        print(f"⚠️ 找不到官方图 {OFFICIAL}（跳过）")
        ok_all = False

    # ---- 旧图纸：真值 = 冻结点位（证明管线在另一张真图上同样逐格复现）----
    if args.all and DRAWING.is_file() and IDENTIFY.is_file():
        text = IDENTIFY.read_text(encoding="utf-8")
        m = re.search(r"格宽\s*([\d.]+)\s*px、格高\s*([\d.]+)\s*px；"
                      r"格心相位\s*x=([\d.]+)\s*y=([\d.]+)", text)
        if m:
            cw, ch, cx0, cy0 = (float(g) for g in m.groups())
            u0, u1 = cx0 - cw / 2.0, cx0 - cw / 2.0 + 14 * cw
            v0, v1 = cy0 - ch / 2.0, cy0 - ch / 2.0 + 11 * ch
            img2 = as_rgb_array(Image.open(DRAWING))
            reg2 = register_from_corners([(u1, v1), (u1, v0), (u0, v0), (u0, v1)],
                                         GridSpec(), order=CORNER_ORDER)
            fm2, rec2 = _check_one("drawing(frozen truth)", img2, reg2,
                                   truth=_drawing_truth(), image_name=DRAWING.name)
            jp2, yp2 = FM.write_field_map(fm2, REPORTS, stem="field_map_drawing")
            results.append({"name": "drawing", "image": str(DRAWING),
                            "reconciliation": rec2.to_json(), "field_map": fm2.to_json_dict(),
                            "json": str(jp2), "yaml": str(yp2)})
            ok_all = ok_all and rec2.ok

    # ---- 报告 ----
    lines = ["# L1 感知出图核查（自动生成，勿手改）", "",
             "- 脚本：`perception/run_field_map_check.py`",
             f"- 真源：`{MAP_YAML.relative_to(V1).as_posix()}`（现役）",
             f"- 结论：**{'官方图逐格一致 ✅' if ok_all else '存在差异 ❌'}**", ""]
    for r in results:
        per = r["reconciliation"]["per_class"]
        lines += [f"## {r['name']}（{pathlib.Path(r['image']).name}）", "",
                  f"- 出图：`{pathlib.Path(r['json']).name}` / `{pathlib.Path(r['yaml']).name}`",
                  "", "| 类 | 感知 | 真值 | 交集 | 仅感知 | 仅真值 | 一致 |",
                  "|---|---|---|---|---|---|---|"]
        for cls, v in sorted(per.items()):
            lines.append(f"| {cls} | {len(v['perceived'])} | {len(v['truth'])} "
                         f"| {len(v['both'])} | {v['extra']} | {v['missing']} "
                         f"| {'✅' if v['equal'] else '❌'} |")
        pm = r["field_map"]["perception"]
        lines += ["", f"- 每格置信度：`{json.dumps(pm['cell_confidence'], ensure_ascii=False)}`",
                  f"- 车位姿：`{json.dumps(pm['car'], ensure_ascii=False)}`",
                  f"- 蓝面朝向：`{json.dumps(pm['blue_face'], ensure_ascii=False)}`",
                  f"- 标定：{pm['provenance']['registration']}（残差 "
                  f"{pm['provenance']['residual_px']} px）", ""]
    (REPORTS / "field_map_check.md").write_text("\n".join(lines) + "\n",
                                               encoding="utf-8", newline="\n")
    (REPORTS / "field_map_check.json").write_text(
        json.dumps({"ok": ok_all, "results": results}, ensure_ascii=False, indent=2),
        encoding="utf-8", newline="\n")
    print(f"\n报告 → {REPORTS / 'field_map_check.md'}")
    print(f"结论：{'官方图逐格一致 ✅' if ok_all else '存在差异 ❌'}")
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
