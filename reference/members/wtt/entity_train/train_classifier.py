# -*- coding: utf-8 -*-
"""路线A: 代码"事物"分类器 —— 驯化(微调)代码模型识别 水桶/爆炸物/敌打击区 等事物.

原理: 冻结的代码编码器(CodeBERT / Qwen-Coder) 把代码片段编码成向量,
再在向量上训练一个轻量分类头(逻辑回归或小 MLP), 学习"这段代码属于哪个事物".
编码阶段走 GPU, 分类头在 CPU 上秒级训练; 8GB 显存(4060)绰绰有余.

依赖安装(一次性, 国内网络推荐先用 ModelScope):
    pip install torch --index-url https://download.pytorch.org/whl/cu121
    pip install transformers modelscope scikit-learn

用法:
    python train_classifier.py                          # 默认 dataset.jsonl
    python train_classifier.py --model AI-ModelScope/codebert-base
    python train_classifier.py --head mlp --seed 7
模型下载优先级: ModelScope 仓库名 -> HuggingFace(自动走 hf-mirror.com 镜像)。
"""
import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

if sys.stdout and hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

import numpy as np

ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL = "AI-ModelScope/codebert-base"   # 国内可用; 也支持任意 HF 模型名
DEFAULT_DATA = ROOT / "dataset.jsonl"
OUTDIR = ROOT / "output"


# ---------------------------------------------------------------- 模型加载
def load_encoder(name):
    """ModelScope 优先, 失败自动切 HuggingFace(镜像)."""
    tokenizer = model = None
    try:
        from modelscope import AutoModel, AutoTokenizer
        tokenizer = AutoTokenizer.from_pretrained(name, trust_remote_code=True)
        model = AutoModel.from_pretrained(name, trust_remote_code=True)
        print(f"[encoder] ModelScope 加载成功: {name}")
    except Exception as e:
        print(f"[encoder] ModelScope 不可用({type(e).__name__}: {e}), 尝试 HuggingFace 镜像...")
        os.environ.setdefault("HF_ENDPOINT", "https://hf-mirror.com")
        from transformers import AutoModel, AutoTokenizer
        tokenizer = AutoTokenizer.from_pretrained(name, trust_remote_code=True)
        model = AutoModel.from_pretrained(name, trust_remote_code=True)
        print(f"[encoder] HuggingFace 加载成功: {name}")
    return tokenizer, model


def encode_all(texts, tokenizer, model, batch, max_len, device, cache_path):
    """批量编码 -> 平均池化向量(注意力掩码加权), 结果缓存到 .npz."""
    if cache_path.exists():
        arr = np.load(cache_path)["emb"]
        if arr.shape[0] == len(texts):
            print(f"[encode] 命中缓存 {cache_path} ({arr.shape})")
            return arr
    model.eval()
    model.to(device)
    embs = []
    n = len(texts)
    for i in range(0, n, batch):
        chunk = texts[i:i + batch]
        tok = tokenizer(chunk, padding=True, truncation=True,
                        max_length=max_len, return_tensors="pt")
        tok = {k: v.to(device) for k, v in tok.items()}
        with torch.no_grad():
            out = model(**tok)
            hid = out.last_hidden_state                      # [B, L, D]
        mask = tok["attention_mask"].unsqueeze(-1).float()   # [B, L, 1]
        pooled = (hid * mask).sum(1) / mask.sum(1).clamp(min=1e-9)
        embs.append(pooled.cpu().numpy())
        print(f"[encode] {min(i + batch, n)}/{n}", end="\r")
    arr = np.vstack(embs).astype(np.float32)
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(cache_path, emb=arr)
    print(f"\n[encode] 完成, 向量 {arr.shape}, 已缓存 -> {cache_path}")
    return arr


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(description="代码事物分类器训练(路线A)")
    ap.add_argument("--data", default=str(DEFAULT_DATA))
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--head", choices=["logreg", "mlp"], default="logreg")
    ap.add_argument("--max-len", type=int, default=512)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--test-size", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--cpu", action="store_true", help="强制 CPU(慢但无 GPU 也能跑)")
    ap.add_argument("--outdir", default=str(OUTDIR))
    args = ap.parse_args()

    # ---- 数据 ----
    samples = [json.loads(l) for l in open(args.data, encoding="utf-8")]
    texts = [s["text"] for s in samples]
    labels = [s["label"] for s in samples]
    label2id = {lab: i for i, lab in enumerate(sorted(set(labels)))}
    print(f"\n[data] {len(samples)} 条, 标签 {label2id}")

    # ---- 编码器 ----
    tokenizer, model = load_encoder(args.model)
    device = "cpu" if args.cpu or not torch.cuda.is_available() else "cuda"
    if device == "cuda":
        print(f"[device] {torch.cuda.get_device_name(0)}")
    tag = hashlib.md5(f"{args.model}|{args.max_len}".encode()).hexdigest()[:10]
    cache_path = Path(args.outdir) / f"emb_{tag}.npz"
    X = encode_all(texts, tokenizer, model, args.batch, args.max_len, device, cache_path)

    # ---- 分类头 ----
    from sklearn.linear_model import LogisticRegression
    from sklearn.neural_network import MLPClassifier
    from sklearn.model_selection import train_test_split
    from sklearn.metrics import (accuracy_score, classification_report,
                                 confusion_matrix)
    y = np.array([label2id[l] for l in labels])
    counts = np.bincount(y, minlength=len(label2id))
    stratify = y if counts.min() >= 2 else None
    X_tr, X_te, y_tr, y_te = train_test_split(
        X, y, test_size=args.test_size, random_state=args.seed, stratify=stratify)
    if stratify is None:
        print("[warn] 存在只有1条的类别, 改为普通随机划分")

    if args.head == "mlp":
        clf = MLPClassifier(hidden_layer_sizes=(256,), max_iter=300,
                            random_state=args.seed)
    else:
        clf = LogisticRegression(C=1.0, class_weight="balanced",
                                 max_iter=2000, random_state=args.seed)
    clf.fit(X_tr, y_tr)
    pred = clf.predict(X_te)
    names = [k for k, v in sorted(label2id.items(), key=lambda kv: kv[1])]

    acc = accuracy_score(y_te, pred)
    print(f"\n[test] 准确率 {acc:.4f}  ({len(y_te)} 条留出测试集)")
    print(classification_report(y_te, pred, target_names=names, zero_division=0))
    print("混淆矩阵(行=真值, 列=预测):")
    print("         " + " ".join(f"{n[:8]:>8}" for n in names))
    for r, row in enumerate(confusion_matrix(y_te, pred)):
        print(f"  {names[r]:8s}" + " ".join(f"{v:8d}" for v in row))

    # ---- 保存 ----
    import joblib
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    joblib.dump(clf, outdir / "head.joblib")
    json.dump({"model": args.model, "max_len": args.max_len,
               "label2id": label2id, "head": args.head},
              open(outdir / "config.json", "w", encoding="utf-8"),
              ensure_ascii=False, indent=2)
    with open(outdir / "results.txt", "w", encoding="utf-8") as f:
        f.write(f"accuracy={acc:.4f}\n")
        f.write(classification_report(y_te, pred, target_names=names,
                                      zero_division=0))
    print(f"\n[save] 分类头/配置/报告 -> {outdir}")
    print(f"[next] 推理: python predict.py --model-dir {outdir} --text \"<代码片段>\"")


if __name__ == "__main__":
    import torch  # 延迟导入, 让 --help 无依赖也可用
    main()
