# -*- coding: utf-8 -*-
r"""命令行入口。

```bash
# 0) 环境自检（不读任何外部文件）
python -m perception.cli selftest

# 1) 一帧图 → 结论 + 报文
python -m perception.cli detect \
    --image 图.png --corners "905.7,718.4;905.7,14.4;8.5,14.4;8.5,718.4" \
    --order N11,N1,A1,A11 --preset map_image --overlay out.png --messages out.jsonl

# 2) 现场标定某个颜色（打印中位 HSV，可直接抄成 ColorRule）
python -m perception.cli calibrate --image shot.png --at 640,360 --radius 4

# 3) 生成合成样例图（人工看管线该长什么样）
python -m perception.cli synth --out-dir reports/synth

# 4) 真发 DDS（只在无人机上）
python -m perception.cli detect ... --publish
```

``--corners`` 支持两种写法：``"u,v;u,v;u,v;u,v"``（配 ``--order``）或
``"N11=u,v;N1=u,v;A1=u,v;A11=u,v"``（标签化，免记顺序）。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

if __package__ in (None, ""):            # 允许 `python perception/cli.py`
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = "perception"

import numpy as np
from PIL import Image

from .bridge_arrow import ArrowSpec
from .car_pose import CarSpec
from .colors import PALETTES, ColorRule, Palette, as_rgb_array, sample_patch_stats
from .grid import CORNER_ORDER, GridSpec
from .objects import FIELD_SPECS, MAP_IMAGE_SPECS
from .overlay import draw_result
from .pipeline import PerceptionConfig, PerceptionPipeline, config_for
from .protocol import recommended_cadence

DEFAULT_ORDER = ",".join(CORNER_ORDER)


# --------------------------------------------------------------------------- #
#  参数解析小工具
# --------------------------------------------------------------------------- #
def parse_corners(text: str, order: str = DEFAULT_ORDER) -> dict[str, tuple[float, float]]:
    """``"u,v;u,v;u,v;u,v"`` 或 ``"N11=u,v;N1=u,v;..."`` → 标签化字典。"""
    parts = [p.strip() for p in text.replace("|", ";").split(";") if p.strip()]
    out: dict[str, tuple[float, float]] = {}
    if all("=" in p for p in parts):
        for p in parts:
            tag, val = p.split("=", 1)
            u, v = [float(x) for x in val.replace(" ", "").split(",")]
            out[tag.strip().upper()] = (u, v)
        return out
    tags = [t.strip().upper() for t in order.split(",") if t.strip()]
    if len(tags) != len(parts):
        raise SystemExit(f"--corners 给了 {len(parts)} 个点，但 --order 是 {len(tags)} 个")
    for tag, p in zip(tags, parts):
        u, v = [float(x) for x in p.replace(" ", "").split(",")]
        out[tag] = (u, v)
    return out


def parse_hsv_rule(cls: str, text: str) -> ColorRule:
    """``"200,0.55,0.35;250,1,1"``（HSV 两段）或 ``"r,g,b;r,g,b"``（RGB 两段）。"""
    lo, _, hi = text.partition(";")
    a = [float(x) for x in lo.replace(" ", "").split(",")]
    b = [float(x) for x in hi.replace(" ", "").split(",")]
    if len(a) != 3 or len(b) != 3:
        raise SystemExit("颜色范围要写三段：'h,s,v;h,s,v' 或 'r,g,b;r,g,b'")
    if min(a + b) < 0.0 or max(a + b) > 360.0:
        raise SystemExit("数值越界：HSV 的 H 在 [0,360]、s/v 在 [0,1]；RGB 在 [0,255]")
    if a[0] > 1.0 or b[0] > 1.0:          # H 只可能 >1 ⇒ 判为 HSV
        if max(a[1:] + b[1:]) > 1.0:
            raise SystemExit("HSV 的 s/v 必须在 [0,1]")
        return ColorRule(cls, hsv_lo=(a[0], a[1], a[2]), hsv_hi=(b[0], b[1], b[2]))
    return ColorRule(cls, rgb_lo=(int(a[0]), int(a[1]), int(a[2])),
                     rgb_hi=(int(b[0]), int(b[1]), int(b[2])))


def _load_image(path: str) -> np.ndarray:
    p = Path(path)
    if not p.is_file():
        raise SystemExit(f"图像不存在：{p}")
    return as_rgb_array(Image.open(p))


def _build_config(args) -> PerceptionConfig:
    if args.palette_json:
        palette = Palette.load(args.palette_json)
    else:
        preset = "field" if args.preset in ("field", "实物") else "map_image"
        palette = PALETTES[preset]
    if args.provisional:
        palette = palette.enable_provisional()
    for spec_text in args.color or []:
        cls, _, rng = spec_text.partition(":")
        palette = palette.with_rule(parse_hsv_rule(cls.strip(), rng))
    objects = FIELD_SPECS if args.preset in ("field", "实物") else MAP_IMAGE_SPECS
    return PerceptionConfig(
        spec=GridSpec(), palette=palette, object_specs=objects,
        car_spec=CarSpec(cls=args.car_cls, nose_cls=args.nose_cls or None,
                         expect_yaw_deg=args.expect_yaw),
        arrow_spec=ArrowSpec(cls=args.arrow_cls, bridge_col=args.bridge_col,
                             min_conf=args.arrow_min_conf),
        min_object_confidence=args.min_object_conf,
        min_car_confidence=args.min_car_conf,
    )


# --------------------------------------------------------------------------- #
#  子命令
# --------------------------------------------------------------------------- #
def cmd_selftest(args) -> int:
    from . import run_tests
    modules = [m for m in run_tests.MODULES if "e2e" not in m or args.with_e2e]
    passed, failed, failures = run_tests.run(modules, verbose=args.verbose)
    print(f"\n自检：通过 {passed} / 失败 {failed}")
    for f in failures:
        print(f)
    return 1 if failed else 0


def cmd_detect(args) -> int:
    img = _load_image(args.image)
    cfg = _build_config(args)
    pipe = PerceptionPipeline(cfg)
    corners = parse_corners(args.corners, args.order)
    res = pipe.process(img, corners_px=corners)
    out = res.to_json()
    if args.json:
        Path(args.json).write_text(json.dumps(out, ensure_ascii=False, indent=2),
                                   encoding="utf-8", newline="\n")
    if args.messages:
        Path(args.messages).write_text("\n".join(res.json_lines()) + "\n",
                                       encoding="utf-8", newline="\n")
    if args.overlay:
        draw_result(img, res).save(args.overlay)
    if not args.quiet:
        print(json.dumps(out, ensure_ascii=False, indent=2))
        print("\n建议发布节奏：", json.dumps(recommended_cadence(), ensure_ascii=False))
    if args.publish:
        from .ros_publish import listen_acks, publish_messages
        info = publish_messages(res.messages, topic=args.topic)
        print(f"\n已发布 {info['sent']} 条 → {info['topic']}"
              f"（订阅者匹配数 {info['subscribers']}；0 = 没人订阅，先查 QoS/域号）")
        if info["subscribers"] == 0:
            return 3
        acks = listen_acks(args.ack_wait)
        print(f"{args.ack_wait:.0f}s 内收到车侧回传 {len(acks)} 条：")
        for a in acks:
            print("  ", a)
    return 0


def cmd_calibrate(args) -> int:
    img = _load_image(args.image)
    u, v = [int(float(x)) for x in args.at.split(",")]
    st = sample_patch_stats(img, u, v, radius=args.radius)
    print(json.dumps(st, ensure_ascii=False, indent=2))
    lo = st["p10_hsv"]
    hi = st["p90_hsv"]
    print(f"\n照抄成规则：--color \"{args.cls}:{lo[0]:.0f},{lo[1]:.2f},{lo[2]:.2f};"
          f"{hi[0]:.0f},{hi[1]:.2f},{hi[2]:.2f}\"")
    if args.out:
        palette = Palette(name=f"calibrated_{args.cls}",
                          provenance=f"{Path(args.image).name}@({u},{v}) r={args.radius}",
                          rules=(ColorRule(args.cls,
                                           hsv_lo=(max(0.0, lo[0] - 15), max(0.0, lo[1] - 0.15),
                                                   max(0.0, lo[2] - 0.15)),
                                           hsv_hi=(min(360.0, hi[0] + 15),
                                                   min(1.0, hi[1] + 0.15),
                                                   min(1.0, hi[2] + 0.15)),
                                           note="现场标定（±15° / ±0.15 余量）"),))
        palette.save(args.out)
        print(f"已写调色板：{args.out}")
    return 0


def cmd_synth(args) -> int:
    from .tests import synth
    outdir = Path(args.out_dir)
    outdir.mkdir(parents=True, exist_ok=True)
    spec = GridSpec()
    cases = {
        "field_full.png": dict(mines=["B1", "G2", "H3"], obstacles=["D1", "F4"],
                               targets=["B11", "C11"], deck_col="K",
                               arrows=[("K5", 90.0), ("K10", -90.0)],
                               car=("N10", 90.0), car_nose_offset=0.16),
        "drawing_map.png": dict(style="drawing", mines=["B1", "G2"], obstacles=["D1"],
                                targets=["B11", "C11"], bridges=["K5", "K10"]),
        "rot25_tilt.png": dict(rot_deg=25.0, tilt=0.25, mines=["H3"], obstacles=["A5"],
                               deck_col="K", arrows=[("K5", 90.0)],
                               car=("K10", 200.0), car_nose_offset=0.16),
    }
    for name, kw in cases.items():
        arena = synth.render_arena(spec, **kw)
        arena.save(outdir / name)
        corners = {k: tuple(round(x, 1) for x in v)
                   for k, v in arena.corners_px.items()}
        print(f"写出 {outdir / name}  {arena.image.shape[1]}x{arena.image.shape[0]}"
              f"  corners={corners}")
    return 0


def cmd_arrow(args) -> int:
    """对**一张没有标定的图**直接跑箭头方向估计（真实照片正控用）。"""
    img = _load_image(args.image)
    palette = (Palette.load(args.palette_json) if args.palette_json
               else PALETTES["field"].enable_provisional())
    masks = palette.masks(img)
    if args.cls not in masks:
        print(f"调色板里没有类 {args.cls}（可用：{sorted(masks)}）")
        return 2
    from .colors import blob_masks
    blobs = blob_masks(masks[args.cls], min_pixels=args.min_pixels, k_close=1, rgb=img)
    print(f"{args.cls} 掩膜覆盖率 {masks[args.cls].mean() * 100:.4f}%，"
          f"连通域 {len(blobs)} 个（min_pixels={args.min_pixels}）")
    from .bridge_arrow import direction_from_points
    for b in blobs:
        res = direction_from_points(b.pixels[:, ::-1].astype(float))
        print(f"  area={b.area:6d} bbox=({b.x0},{b.y0})-({b.x1},{b.y1}) "
              f"fill={b.fill:.2f} | 图像系方向 {res['dir_deg']:7.1f}° "
              f"conf={res['confidence']:.3f} n_agree={res['evidence'].get('n_agree')}")
    return 0


def cmd_topics(args) -> int:
    print(json.dumps(recommended_cadence(), ensure_ascii=False, indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="perception", description="无人机俯瞰场地感知管线")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("selftest", help="跑离线单测")
    s.add_argument("--with-e2e", action="store_true", help="连端到端测试一起跑（慢）")
    s.add_argument("--verbose", action="store_true")
    s.set_defaults(func=cmd_selftest)

    d = sub.add_parser("detect", help="一帧图 → 结论 + 报文")
    d.add_argument("--image", required=True)
    d.add_argument("--corners", required=True,
                   help="'u,v;u,v;u,v;u,v' 或 'N11=u,v;N1=u,v;A1=u,v;A11=u,v'")
    d.add_argument("--order", default=DEFAULT_ORDER, help=f"默认 {DEFAULT_ORDER}")
    d.add_argument("--preset", default="field", choices=["field", "map_image"])
    d.add_argument("--palette-json", default="")
    d.add_argument("--provisional", action="store_true",
                   help="打开没有实测证据的占位颜色规则（现场标定前不要用）")
    d.add_argument("--color", action="append",
                   help="覆盖某类颜色，如 --color 'car:18,0.65,0.55;38,1,1'")
    d.add_argument("--car-cls", default="car")
    d.add_argument("--nose-cls", default="")
    d.add_argument("--expect-yaw", type=float, default=None)
    d.add_argument("--arrow-cls", default="arrow")
    d.add_argument("--bridge-col", default="K")
    d.add_argument("--arrow-min-conf", type=float, default=0.30)
    d.add_argument("--min-object-conf", type=float, default=0.30)
    d.add_argument("--min-car-conf", type=float, default=0.35)
    d.add_argument("--json", default="")
    d.add_argument("--messages", default="", help="写出 JSONL（每条一行）")
    d.add_argument("--overlay", default="", help="写出叠加图 PNG")
    d.add_argument("--publish", action="store_true", help="真发 DDS（需 rclpy）")
    d.add_argument("--topic", default="/dzlink/drone_to_car")
    d.add_argument("--ack-wait", type=float, default=3.0)
    d.add_argument("--quiet", action="store_true")
    d.set_defaults(func=cmd_detect)

    c = sub.add_parser("calibrate", help="现场标定某类颜色")
    c.add_argument("--image", required=True)
    c.add_argument("--at", required=True, help="采样点 'u,v'")
    c.add_argument("--radius", type=int, default=4)
    c.add_argument("--cls", default="mine")
    c.add_argument("--out", default="", help="写出调色板 JSON")
    c.set_defaults(func=cmd_calibrate)

    y = sub.add_parser("synth", help="生成合成样例图")
    y.add_argument("--out-dir", default="reports/synth")
    y.set_defaults(func=cmd_synth)

    a = sub.add_parser("arrow", help="对未标定图直接跑箭头方向估计")
    a.add_argument("--image", required=True)
    a.add_argument("--cls", default="arrow")
    a.add_argument("--min-pixels", type=int, default=30)
    a.add_argument("--palette-json", default="")
    a.set_defaults(func=cmd_arrow)

    t = sub.add_parser("topics", help="打印话题/QoS/建议节奏")
    t.set_defaults(func=cmd_topics)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    t0 = time.perf_counter()
    rc = args.func(args)
    if args.cmd != "selftest":
        print(f"\n(用时 {time.perf_counter() - t0:.2f}s)")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
