# -*- coding: utf-8 -*-
"""生成模型训练成果渲染图。"""
import csv
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

RESULTS = Path(r"E:\ml_datasets\results_5090")
OUT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\docs\figures")
OUT.mkdir(parents=True, exist_ok=True)

# 颜色方案
COLORS = ["#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2", "#ca8a04", "#be123c"]


def read_csv(path):
    """读取results.csv，返回dict of lists。"""
    if not path.exists():
        return None
    with open(path, encoding="utf-8") as f:
        reader = csv.DictReader(f)
        data = {k: [] for k in reader.fieldnames}
        for row in reader:
            for k, v in row.items():
                try:
                    data[k].append(float(v))
                except (ValueError, TypeError):
                    data[k].append(np.nan)
    return data


def plot_7cls_map_curves():
    """7cls各版本mAP50训练曲线对比。"""
    models = {
        "baseline(v3)": "aerial_field_7cls_v3_results.csv",
        "hsv(色彩增强)": "aerial_field_7cls_hsv_results.csv",
        "cp(copy_paste)": "aerial_field_7cls_cp_results.csv",
        "b32(大batch)": "aerial_field_7cls_b32_results.csv",
        "tune(几何增强)": "aerial_field_7cls_tune_results.csv",
        "yolo11m": "aerial_field_7cls_yolo11m_results.csv",
        "yolo11m_ft": "aerial_field_7cls_yolo11m_ft_results.csv",
    }
    fig, ax = plt.subplots(figsize=(12, 6))
    for i, (name, fname) in enumerate(models.items()):
        data = read_csv(RESULTS / fname)
        if data is None:
            continue
        epochs = data.get("epoch", [])
        map50 = data.get("metrics/mAP50(B)", [])
        if not map50:
            continue
        ax.plot(epochs, map50, label=name, color=COLORS[i % len(COLORS)], linewidth=1.5)
    ax.set_xlabel("Epoch", fontsize=12)
    ax.set_ylabel("mAP50", fontsize=12)
    ax.set_title("7cls检测模型各版本 mAP50 训练曲线对比", fontsize=14, fontweight="bold")
    ax.legend(loc="lower right", fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0.70, 0.82)
    plt.tight_layout()
    plt.savefig(OUT / "fig1_7cls_map_curves.png", dpi=150)
    plt.close()
    print("✅ fig1_7cls_map_curves.png")


def plot_7cls_best_bar():
    """各7cls模型best mAP50柱状图。"""
    models = {
        "baseline": 0.7857,
        "hsv": 0.7863,
        "cp": 0.7846,
        "b32": 0.7860,
        "tune": 0.7794,
        "yolo11m": 0.7848,
        "yolo11m_ft": 0.7852,
    }
    fig, ax = plt.subplots(figsize=(10, 5))
    names = list(models.keys())
    vals = list(models.values())
    bars = ax.bar(names, vals, color=COLORS[:len(names)], width=0.6)
    ax.set_ylabel("Best mAP50", fontsize=12)
    ax.set_title("7cls检测模型各版本 Best mAP50 对比", fontsize=14, fontweight="bold")
    ax.set_ylim(0.77, 0.80)
    for bar, v in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width() / 2, v + 0.0002, f"{v:.4f}",
                ha="center", va="bottom", fontsize=10, fontweight="bold")
    ax.grid(True, axis="y", alpha=0.3)
    plt.xticks(rotation=15)
    plt.tight_layout()
    plt.savefig(OUT / "fig2_7cls_best_bar.png", dpi=150)
    plt.close()
    print("✅ fig2_7cls_best_bar.png")


def plot_v4c_curves():
    """v4c格号识别器joint_accuracy曲线。"""
    versions = {
        "v4c(原始)": "cell_classifier_v4c_metrics.csv",
        "v4c_aug(模糊+JPEG+噪点)": "cell_classifier_v4c_aug_metrics.csv",
        "v4c_ft(小batch微调)": "cell_classifier_v4c_ft_metrics.csv",
    }
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # 左：joint_accuracy曲线
    ax = axes[0]
    for i, (name, fname) in enumerate(versions.items()):
        data = read_csv(RESULTS / fname)
        if data is None:
            continue
        # metrics.csv列名可能不同
        acc_key = None
        for k in data:
            if "joint" in k.lower():
                acc_key = k
                break
        if acc_key is None:
            continue
        epochs = list(range(1, len(data[acc_key]) + 1))
        ax.plot(epochs, data[acc_key], label=name, color=COLORS[i], linewidth=1.5)
    ax.set_xlabel("Epoch", fontsize=11)
    ax.set_ylabel("Joint Accuracy", fontsize=11)
    ax.set_title("格号识别器 v4c 各版本 Joint Accuracy 曲线", fontsize=13, fontweight="bold")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # 右：best值柱状图
    ax = axes[1]
    best_vals = {
        "v4c\n(原始)": 0.8249,
        "v4c_aug\n(域随机化)": 0.8418,
        "v4c_ft\n(小batch微调)": 0.8588,
    }
    bars = ax.bar(best_vals.keys(), best_vals.values(), color=COLORS[:3], width=0.5)
    ax.set_ylabel("Best Joint Accuracy", fontsize=11)
    ax.set_title("格号识别器 Best Joint Accuracy 对比", fontsize=13, fontweight="bold")
    ax.set_ylim(0.80, 0.88)
    for bar, v in zip(bars, best_vals.values()):
        ax.text(bar.get_x() + bar.get_width() / 2, v + 0.001, f"{v:.4f}",
                ha="center", va="bottom", fontsize=11, fontweight="bold")
    ax.grid(True, axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT / "fig3_v4c_comparison.png", dpi=150)
    plt.close()
    print("✅ fig3_v4c_comparison.png")


def plot_two_stage_mine():
    """两阶段雷检测效果对比。"""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # 左：单阶段vs两阶段 mAP50
    ax = axes[0]
    methods = ["单阶段YOLO\n(conf=0.15)", "两阶段\nYOLO+分类器\n(conf=0.05, thresh=0.3)"]
    map50 = [0.0, 0.6272]
    colors = ["#dc2626", "#16a34a"]
    bars = ax.bar(methods, map50, color=colors, width=0.5)
    ax.set_ylabel("mAP50", fontsize=12)
    ax.set_title("爆炸物检测：单阶段 vs 两阶段", fontsize=13, fontweight="bold")
    ax.set_ylim(0, 0.75)
    for bar, v in zip(bars, map50):
        ax.text(bar.get_x() + bar.get_width() / 2, v + 0.01, f"{v:.4f}",
                ha="center", va="bottom", fontsize=12, fontweight="bold")
    ax.grid(True, axis="y", alpha=0.3)

    # 右：阈值扫描曲线
    ax = axes[1]
    threshs = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    p_vals = [0.6378, 0.6387, 0.6417, 0.6417, 0.6417, 0.6417, 0.6417]  # 近似
    r_vals = [0.6868, 0.6703, 0.6593, 0.6593, 0.6593, 0.6593, 0.6593]
    ap_vals = [0.6272, 0.6142, 0.6073, 0.6073, 0.6073, 0.6073, 0.6073]
    ax.plot(threshs, p_vals, "o-", label="Precision", color="#2563eb", linewidth=2)
    ax.plot(threshs, r_vals, "s-", label="Recall", color="#16a34a", linewidth=2)
    ax.plot(threshs, ap_vals, "^-", label="mAP50", color="#dc2626", linewidth=2)
    ax.axvline(x=0.3, color="gray", linestyle="--", alpha=0.5, label="最优阈值=0.3")
    ax.set_xlabel("分类器阈值 (cls_thresh)", fontsize=11)
    ax.set_ylabel("指标", fontsize=11)
    ax.set_title("两阶段检测：分类器阈值扫描 (YOLO conf=0.05)", fontsize=13, fontweight="bold")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0.5, 0.75)
    plt.tight_layout()
    plt.savefig(OUT / "fig4_two_stage_mine.png", dpi=150)
    plt.close()
    print("✅ fig4_two_stage_mine.png")


def plot_summary_dashboard():
    """综合成果仪表盘。"""
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle("空地协同侦排挑战赛 — 模型训练成果总览 (2026-10-10)", fontsize=16, fontweight="bold", y=0.98)

    # 1. 7cls best mAP50
    ax1 = fig.add_subplot(2, 2, 1)
    models = ["baseline", "hsv", "cp", "b32", "tune", "yolo11m", "yolo11m_ft"]
    vals = [0.7857, 0.7863, 0.7846, 0.7860, 0.7794, 0.7848, 0.7852]
    bars = ax1.bar(models, vals, color=COLORS[:len(models)], width=0.6)
    ax1.set_ylabel("Best mAP50")
    ax1.set_title("7cls检测模型 Best mAP50 (34张val)")
    ax1.set_ylim(0.77, 0.80)
    for bar, v in zip(bars, vals):
        ax1.text(bar.get_x() + bar.get_width() / 2, v + 0.0002, f"{v:.4f}",
                 ha="center", va="bottom", fontsize=8, fontweight="bold")
    ax1.tick_params(axis="x", rotation=20)
    ax1.grid(True, axis="y", alpha=0.3)

    # 2. v4c格号识别
    ax2 = fig.add_subplot(2, 2, 2)
    v4c_models = ["v4c", "v4c_aug", "v4c_ft"]
    v4c_vals = [0.8249, 0.8418, 0.8588]
    bars = ax2.bar(v4c_models, v4c_vals, color=["#2563eb", "#16a34a", "#dc2626"], width=0.5)
    ax2.set_ylabel("Best Joint Accuracy")
    ax2.set_title("格号识别器 Best Joint Accuracy")
    ax2.set_ylim(0.80, 0.88)
    for bar, v in zip(bars, v4c_vals):
        ax2.text(bar.get_x() + bar.get_width() / 2, v + 0.001, f"{v:.4f}",
                 ha="center", va="bottom", fontsize=10, fontweight="bold")
    ax2.grid(True, axis="y", alpha=0.3)

    # 3. 两阶段雷检测
    ax3 = fig.add_subplot(2, 2, 3)
    methods = ["单阶段YOLO", "两阶段\nYOLO+分类器"]
    mine_map = [0.0, 0.6272]
    bars = ax3.bar(methods, mine_map, color=["#dc2626", "#16a34a"], width=0.4)
    ax3.set_ylabel("mAP50 (官方74张)")
    ax3.set_title("爆炸物检测 mAP50 (官方74张精确标注)")
    ax3.set_ylim(0, 0.75)
    for bar, v in zip(bars, mine_map):
        ax3.text(bar.get_x() + bar.get_width() / 2, v + 0.01, f"{v:.4f}",
                 ha="center", va="bottom", fontsize=11, fontweight="bold")
    ax3.grid(True, axis="y", alpha=0.3)

    # 4. 关键指标文字
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.axis("off")
    text = """
关键指标总结

【7cls检测器】
  Best mAP50: 0.7863 (hsv版)
  最优模型: aerial_field_7cls_hsv

【格号识别器 v4c】
  Best Joint Acc: 0.8588 (v4c_ft版)
  较初始提升: +3.39%

【爆炸物两阶段检测】
  单阶段 mAP50: 0.0000 (完全漏检)
  两阶段 mAP50: 0.6272 (召回68.68%)
  最优配置: YOLO conf=0.05 + 分类器thresh=0.3

【数据增强】
  copy-paste: 500张官方雷回贴航拍背景
  域随机化: 模糊+JPEG+噪点 (v4c提升2%)
"""
    ax4.text(0.05, 0.5, text, fontsize=11, va="center",
             bbox=dict(boxstyle="round,pad=1", facecolor="#f0f9ff", edgecolor="#0891b2"))

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(OUT / "fig0_summary_dashboard.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("✅ fig0_summary_dashboard.png")


if __name__ == "__main__":
    print("生成模型成果渲染图...\n")
    plot_summary_dashboard()
    plot_7cls_map_curves()
    plot_7cls_best_bar()
    plot_v4c_curves()
    plot_two_stage_mine()
    print(f"\n全部完成 → {OUT}")
