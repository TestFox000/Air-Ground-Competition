"""数据集体检工具测试（合成数据集，不需要真图）。"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "ml"))

from dataset_check import (  # noqa: E402
    DEFAULT_KPT_SHAPE,
    DEFAULT_NAMES,
    check_dataset,
    discover,
    label_path_for,
    parse_label,
)

VALID_LINE = "0 0.5 0.5 0.2 0.2 0.5 0.5 2"
VALID_RED = "1 0.3 0.4 0.1 0.1 0.3 0.4 2"


def _make(root: Path, split_lines: dict) -> Path:
    """split_lines: {split: {image_stem: label_text}}"""
    for split, items in split_lines.items():
        (root / "images" / split).mkdir(parents=True, exist_ok=True)
        (root / "labels" / split).mkdir(parents=True, exist_ok=True)
        for stem, text in items.items():
            (root / "images" / split / f"{stem}.jpg").write_bytes(b"")
            (root / "labels" / split / f"{stem}.txt").write_text(text, encoding="utf-8")
    return root


# ---------------------------------------------------------------- 解析
def test_parse_valid_line():
    boxes, errs = parse_label(VALID_LINE, 8, 2)
    assert not errs and len(boxes) == 1
    assert boxes[0][0] == 0 and boxes[0][7] == 2


def test_parse_field_count_must_match_kpt_shape():
    _, errs = parse_label("0 0.5 0.5 0.2 0.2 0.5 0.5", 8, 2)   # 少一个 kv
    assert errs and "字段数" in errs[0]


@pytest.mark.parametrize("line,keyword", [
    ("2 0.5 0.5 0.2 0.2 0.5 0.5 2", "类别"),          # 类别越界
    ("0 1.5 0.5 0.2 0.2 0.5 0.5 2", "归一化"),        # 框没归一化
    ("0 0.5 0.5 0.0 0.2 0.5 0.5 2", "宽高"),          # 宽=0
    ("0 0.5 0.5 0.2 0.2 1.4 0.5 2", "关键点"),        # 关键点没归一化
    ("0 0.5 0.5 0.2 0.2 0.5 0.5 5", "可见性"),        # kv 非法
    ("0 0.5 abc 0.2 0.2 0.5 0.5 2", "非数字"),
])
def test_parse_rejects_bad_lines(line, keyword):
    _, errs = parse_label(line, 8, 2)
    assert errs and any(keyword in e for e in errs), errs


# ---------------------------------------------------------------- 数据集
def test_valid_dataset_passes(tmp_path):
    _make(tmp_path, {
        "train": {"a": VALID_LINE, "b": VALID_RED + "\n" + VALID_LINE},
        "val": {"c": VALID_LINE},
    })
    r = check_dataset(tmp_path)
    assert r.ok, r.summary()
    assert r.images == 3 and r.labels == 3 and r.boxes == 4
    assert r.per_class == {0: 3, 1: 1}
    assert r.per_split == {"train": 2, "val": 1}


def test_contract_defaults_match_car_header():
    """与车端 postprocess.h 一致：2 类、kpt_shape [1,3]。"""
    assert DEFAULT_NAMES == ("blue_target", "red_target")
    assert DEFAULT_KPT_SHAPE == (1, 3)


def test_missing_label_is_error(tmp_path):
    _make(tmp_path, {"train": {"a": VALID_LINE}})
    (tmp_path / "images" / "train" / "b.jpg").write_bytes(b"")
    r = check_dataset(tmp_path)
    assert not r.ok
    assert any("没有对应标签" in e for e in r.errors)
    assert r.missing_labels == ["labels/train/b.txt"]


def test_bad_label_reports_file_and_line(tmp_path):
    _make(tmp_path, {"train": {"a": VALID_LINE + "\n0 9.9 0.5 0.2 0.2 0.5 0.5 2"}})
    r = check_dataset(tmp_path)
    assert not r.ok
    assert any("labels/train/a.txt" in e and "第 2 行" in e for e in r.errors)


def test_missing_class_warns(tmp_path):
    _make(tmp_path, {"train": {"a": VALID_LINE}, "val": {"b": VALID_LINE}})
    r = check_dataset(tmp_path)
    assert r.ok                      # 只是告警
    assert any("类别" in w and "一个样本都没有" in w for w in r.warnings)


def test_class_imbalance_warns(tmp_path):
    lines = "\n".join([VALID_LINE] * 20 + [VALID_RED])
    _make(tmp_path, {"train": {"a": lines}, "val": {"b": VALID_RED}})
    r = check_dataset(tmp_path)
    assert any("不均衡" in w for w in r.warnings)


def test_all_invisible_kpt_warns(tmp_path):
    line = "0 0.5 0.5 0.2 0.2 0.5 0.5 0"
    _make(tmp_path, {"train": {"a": line}, "val": {"b": line}})
    r = check_dataset(tmp_path)
    assert any("不可见" in w for w in r.warnings)


def test_empty_label_warns(tmp_path):
    _make(tmp_path, {"train": {"a": ""}, "val": {"b": VALID_LINE}})
    r = check_dataset(tmp_path)
    assert r.empty_labels == ["labels/train/a.txt"]


def test_missing_root_is_error(tmp_path):
    r = check_dataset(tmp_path / "nope")
    assert not r.ok and "不存在" in r.errors[0]


def test_empty_dataset_is_error(tmp_path):
    (tmp_path / "images" / "train").mkdir(parents=True)
    r = check_dataset(tmp_path)
    assert not r.ok and "没找到任何图像" in r.errors[0]


def test_split_layout_variant(tmp_path):
    """兼容 root/train/images 这种布局。"""
    d = tmp_path / "train" / "images"
    d.mkdir(parents=True)
    (d / "a.jpg").write_bytes(b"")
    ld = tmp_path / "train" / "labels"
    ld.mkdir(parents=True)
    (ld / "a.txt").write_text(VALID_LINE, encoding="utf-8")
    assert discover(tmp_path, ("train",))["train"]
    r = check_dataset(tmp_path, splits=("train",))
    assert r.ok and r.images == 1


def test_label_path_for_both_layouts(tmp_path):
    p1 = tmp_path / "images" / "train" / "a.jpg"
    assert label_path_for(p1, tmp_path) == tmp_path / "labels" / "train" / "a.txt"
    p2 = tmp_path / "train" / "images" / "a.jpg"
    assert label_path_for(p2, tmp_path) == tmp_path / "train" / "labels" / "a.txt"


def test_report_summary_is_readable(tmp_path):
    _make(tmp_path, {"train": {"a": VALID_LINE}, "val": {"b": VALID_RED}})
    s = check_dataset(tmp_path).summary()
    assert "图像 2 张" in s and "目标框 2 个" in s
