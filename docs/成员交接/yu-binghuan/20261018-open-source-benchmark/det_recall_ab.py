# -*- coding: utf-8 -*-
"""检测召回 A/B 工具。

吸收目标：SpireCV 的「召回优先」检测策略（conf≈0.001~0.05 + NMS=0.60）——对上我们
已知的「视频识别漏检」痛点（见 field_state 里 8 雷只检出 3 颗）。

做法：导入 video_field_recon 模块，替换其模块级全局 CONF / NMS_IOU / VOTE_MIN 后调用
main()，在**同一个真机视频**上跑不同参数预设，量化对比：
    - 雷召回率（对照官方固定 8 雷基线 OFFICIAL_MINES）
    - 障碍/靶检出数（作为误检放大的代理指标）

用法：
    python det_recall_ab.py                          # 全部 4 段视频 × 3 预设
    python det_recall_ab.py --videos cam_20261018_091746 --max-frames 40
    python det_recall_ab.py --presets baseline recall
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))

import video_field_recon as vfr  # noqa: E402

VIDEO_DIR = Path(r"C:\Users\29212\Desktop\无人机视频")
OFFICIAL_MINES = ["B1", "A2", "I2", "C3", "E3", "F5", "H8", "G9"]

BASE_CONF = {
    "paper": 0.22, "cell_label": 0.35, "car": 0.25,
    "target_red": 0.25, "target_blue": 0.25, "obstacle": 0.40,
    "bucket_red": 0.40, "bucket_green": 0.40, "bucket_cyan": 0.40,
    "car_start": 0.25, "car_end": 0.25, "uav_start": 0.25, "uav_end": 0.25,
}


def _scale(conf: dict, k: float, floor: float = 0.02) -> dict:
    return {n: max(floor, round(v * k, 3)) for n, v in conf.items()}


# 预设：
#   baseline 现状
#   gridfit  放宽 v4d 格号读取下限(0.65→0.30) → 更多锚点 → 提高网格拟合成功率
#   gates    放宽投票门控（吸附残差 0.35→0.50，最少票数 2→1，v4d 0.65→0.40）
#   full     全放宽（检测阈值×0.25 + NMS0.60 + 网格 + 门控）
PRESETS = {
    "baseline": {"conf": dict(BASE_CONF), "nms": 0.45, "vote_min": 2, "v4d": 0.65, "resid": 0.35},
    "gridfit":  {"conf": dict(BASE_CONF), "nms": 0.45, "vote_min": 2, "v4d": 0.30, "resid": 0.35},
    "gates":    {"conf": dict(BASE_CONF), "nms": 0.45, "vote_min": 1, "v4d": 0.40, "resid": 0.50},
    "full":     {"conf": _scale(BASE_CONF, 0.25), "nms": 0.60, "vote_min": 1, "v4d": 0.30, "resid": 0.55},
}


def run_one(video: Path, preset: str, out_dir: Path, max_frames: int) -> dict:
    p = PRESETS[preset]
    vfr.CONF = dict(p["conf"])
    vfr.NMS_IOU = p["nms"]
    vfr.VOTE_MIN = p["vote_min"]
    vfr.V4D_MIN_CONF = p["v4d"]
    vfr.MAX_VOTE_RESIDUAL = p["resid"]
    sys.argv = ["video_field_recon.py", str(video), "--stride", "15", "--out", str(out_dir)]
    if max_frames:
        sys.argv += ["--max-frames", str(max_frames)]
    try:
        vfr.main()
    except Exception as e:  # noqa: BLE001
        # main() 在写完 field_state.json 之后（渲染/规划阶段）可能因缺 PIL 等报错；
        # A/B 只关心检测结果，容错继续。
        if not (out_dir / "field_state.json").exists():
            raise
        print(f"[warn] {video.stem}/{preset} 后续阶段报错(不影响检测): {type(e).__name__}: {e}")
    return json.loads((out_dir / "field_state.json").read_text(encoding="utf-8"))


def mine_recall(fs: dict) -> tuple[int, list[str]]:
    got = [m for m in fs.get("mines_video", []) if m in OFFICIAL_MINES]
    return len(got), got


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--videos", nargs="*", default=None,
                    help="视频文件名(不含 .mp4)或全路径；缺省=目录下全部")
    ap.add_argument("--presets", nargs="*", default=list(PRESETS))
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--out-root", default=str(BASE / "检测召回AB"))
    args = ap.parse_args()

    if args.videos:
        videos = [Path(v) if v.lower().endswith(".mp4") else (VIDEO_DIR / f"{v}.mp4")
                  for v in args.videos]
    else:
        videos = sorted(VIDEO_DIR.glob("*.mp4"))

    out_root = Path(args.out_root)
    table = []
    for vid in videos:
        if not vid.exists():
            print(f"[skip] 视频不存在: {vid}")
            continue
        for preset in args.presets:
            out_dir = out_root / preset / vid.stem
            out_dir.mkdir(parents=True, exist_ok=True)
            t0 = time.time()
            try:
                fs = run_one(vid, preset, out_dir, args.max_frames)
            except Exception as e:  # noqa: BLE001
                print(f"[ERR] {vid.stem} / {preset}: {type(e).__name__}: {e}")
                continue
            n_rec, got = mine_recall(fs)
            n_ok = fs.get("frames_grid_ok", 0)
            n_sam = fs.get("frames_sampled", 0) or 1
            row = {
                "video": vid.stem, "preset": preset,
                "frames_sampled": n_sam, "frames_grid_ok": n_ok,
                "grid_ok_rate": round(n_ok / n_sam, 3),
                "mine_found": n_rec, "mine_recall": round(n_rec / len(OFFICIAL_MINES), 3),
                "mines": fs.get("mines_video"),
                "obstacles_n": len(fs.get("obstacles_video", [])),
                "targets": fs.get("targets_video"),
                "sec": round(time.time() - t0, 1),
            }
            table.append(row)
            print(f"[OK] {vid.stem:24s} {preset:9s} grid_ok {n_ok}/{n_sam} "
                  f"({row['grid_ok_rate']:.0%})  雷 {n_rec}/8 障碍{row['obstacles_n']} "
                  f"靶{row['targets']}  {row['sec']}s")

    (out_root / "ab_summary.json").write_text(
        json.dumps(table, ensure_ascii=False, indent=2), encoding="utf-8")

    # 汇总：每个预设的平均雷召回
    print("\n===== 汇总（网格拟合率 / 雷召回 / 障碍数）=====")
    for preset in args.presets:
        rows = [r for r in table if r["preset"] == preset]
        if not rows:
            continue
        avg_g = sum(r["grid_ok_rate"] for r in rows) / len(rows)
        avg_rec = sum(r["mine_recall"] for r in rows) / len(rows)
        avg_obs = sum(r["obstacles_n"] for r in rows) / len(rows)
        print(f"  {preset:9s}  网格拟合率 {avg_g:.0%}   雷召回均值 {avg_rec:.0%}   障碍均值 {avg_obs:.1f}")
    print(f"\n明细: {out_root / 'ab_summary.json'}")


if __name__ == "__main__":
    main()
