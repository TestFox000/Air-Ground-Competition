"""泛化发布门禁（tools/ml/eval_generalization.py）纯逻辑测试。

不碰 ultralytics/图片：只测判定逻辑、基线读写、README 记录追加。
门禁语义：绝对底线 + 相对现役基线的回归容忍，双闸皆过才放行。
"""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools" / "ml"))

from eval_generalization import (  # noqa: E402
    HoldoutMetrics,
    _RECORD_HEADER,
    append_record,
    decide,
    load_baseline,
    save_baseline,
)


def _m(recall: float, weights: str = "best.pt", n: int = 100) -> HoldoutMetrics:
    det = round(recall * n)
    return HoldoutMetrics(
        weights=weights, images_dir="holdout", n_images=n,
        detected_images=det, recall=recall, n_boxes=det,
        conf_mean=0.6, conf_min=0.3, missed=("x.jpg",) * (n - det),
        conf=0.25, imgsz=1280, date="2026-10-14 20:00",
    )


def test_decide_pass_without_baseline():
    ok, reason = decide(_m(0.8), None, min_recall=0.5, max_regression_pp=2.0)
    assert ok and "首次评估" in reason


def test_decide_absolute_floor_rejects():
    ok, reason = decide(_m(0.4), None, min_recall=0.5, max_regression_pp=2.0)
    assert not ok and "绝对底线" in reason


def test_decide_regression_rejects():
    base = {"weights": "old.pt", "recall": 0.80}
    ok, reason = decide(_m(0.75), base, min_recall=0.5, max_regression_pp=2.0)
    assert not ok and "退化" in reason
    # 退化 1pp < 容忍 2pp：放行且理由含"容忍"
    ok, reason = decide(_m(0.79), base, min_recall=0.5, max_regression_pp=2.0)
    assert ok and "容忍" in reason


def test_decide_improvement_passes():
    base = {"weights": "old.pt", "recall": 0.80}
    ok, _ = decide(_m(0.85), base, min_recall=0.5, max_regression_pp=2.0)
    assert ok


def test_baseline_roundtrip(tmp_path):
    p = tmp_path / "baseline.json"
    assert load_baseline(p) is None
    m = _m(0.77, weights="candidate.pt")
    save_baseline(m, p)
    loaded = load_baseline(p)
    assert loaded["weights"] == "candidate.pt"
    assert loaded["recall"] == pytest.approx(0.77)


def test_append_record_creates_section_and_appends(tmp_path):
    readme = tmp_path / "README.md"
    readme.write_text("# models/release\n\n## 登记表\n\n| a |\n", encoding="utf-8")
    append_record(_m(0.9), "放行", readme)
    append_record(_m(0.6), "拒绝", readme)
    text = readme.read_text(encoding="utf-8")
    assert _RECORD_HEADER.strip() in text
    assert text.count("| 2026-10-14 20:00 |") == 2
    assert "| 90.0% |" in text and "| 60.0% |" in text
    # 既有内容未丢
    assert "## 登记表" in text


def test_append_record_into_existing_section(tmp_path):
    readme = tmp_path / "README.md"
    readme.write_text(
        "# t\n\n" + _RECORD_HEADER.strip() + "\n\n| 日期 | 模型 |\n|---|---|\n"
        "| 2026-10-13 | old.pt |\n", encoding="utf-8")
    append_record(_m(0.88), "放行", readme)
    text = readme.read_text(encoding="utf-8")
    # 不重复建 section
    assert text.count(_RECORD_HEADER.strip()) == 1
    assert "old.pt" in text and "best.pt" in text
