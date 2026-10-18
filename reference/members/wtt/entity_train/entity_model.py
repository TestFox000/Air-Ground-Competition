# -*- coding: utf-8 -*-
"""驯化模型的导入接口 —— 在你的项目里一行 import 即可使用.

快速开始:
    import sys; sys.path.insert(0, "E:/1/无人机/entity_train")   # 换成你的实际路径
    from entity_model import EntityClassifier

    cls = EntityClassifier("output")               # 训练产物目录(config.json+head.joblib)
    for lab, p in cls.classify("BUCKETS_N = 6"):   # -> [('bucket', 0.98), ...]
        print(lab, p)

    print(cls.classify_batch([code1, code2]))      # 批量判断

命令行试用:
    python entity_model.py --model-dir output --text "EXPLOSIVES_N = 8"

说明: 训练只保存了分类头(head.joblib)+配置(config.json); 编码器本体导入时
按 config.json 里的模型名从 ModelScope 加载(失败自动切 HuggingFace 国内镜像)。
所有第三方库惰性导入: 没装 torch 也能 import 本模块, 实例化时才报错。
"""
import argparse
import json
import os
import sys
from pathlib import Path

if sys.stdout and hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def _load_encoder(name):
    try:
        from modelscope import AutoModel, AutoTokenizer
        tok = AutoTokenizer.from_pretrained(name, trust_remote_code=True)
        mdl = AutoModel.from_pretrained(name, trust_remote_code=True)
    except Exception:
        os.environ.setdefault("HF_ENDPOINT", "https://hf-mirror.com")
        from transformers import AutoModel, AutoTokenizer
        tok = AutoTokenizer.from_pretrained(name, trust_remote_code=True)
        mdl = AutoModel.from_pretrained(name, trust_remote_code=True)
    return tok, mdl


class EntityClassifier:
    """驯化好的代码事物分类器(路线A产物).

    - model_dir: 训练产物目录, 内含 config.json + head.joblib
    - device:    None=自动(cuda 优先); 可强制 "cuda" / "cpu"
    """

    def __init__(self, model_dir, device=None):
        import torch                                     # 惰性导入
        self.model_dir = Path(model_dir)
        self.cfg = json.load(open(self.model_dir / "config.json", encoding="utf-8"))
        self.label2id = {k: int(v) for k, v in self.cfg["label2id"].items()}
        self.id2label = {v: k for k, v in self.label2id.items()}
        self.max_len = int(self.cfg.get("max_len", 512))
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")

        import joblib
        self._tok, self._enc = _load_encoder(self.cfg["model"])
        self._enc.eval().to(self.device)
        self._head = joblib.load(self.model_dir / "head.joblib")
        self._id_list = [self.label2id[l] for l in self._head.classes_]
        print(f"[EntityClassifier] 已导入 {self.model_dir} | "
              f"encoder={self.cfg['model']} | device={self.device} | "
              f"{len(self.label2id)} 类")

    def _embed(self, texts):
        import torch
        tok = self._tok(texts, padding=True, truncation=True,
                        max_length=self.max_len, return_tensors="pt")
        tok = {k: v.to(self.device) for k, v in tok.items()}
        with torch.no_grad():
            hid = self._enc(**tok).last_hidden_state
        mask = tok["attention_mask"].unsqueeze(-1).float()
        return ((hid * mask).sum(1) / mask.sum(1).clamp(min=1e-9)).cpu().numpy()

    def classify(self, text, top=3):
        """单段代码 -> [(标签, 概率), ...], 按概率降序."""
        return self.classify_batch([text], top=top)[0]

    def classify_batch(self, texts, top=3):
        """多段代码 -> 每段 [(标签, 概率), ...]."""
        probs = self._head.predict_proba(self._embed(list(texts)))
        out = []
        for row in probs:
            order = row.argsort()[::-1][:top]
            out.append([(self.id2label[self._id_list[i]], float(row[i]))
                        for i in order])
        return out

    def tag(self, text):
        """只取最优标签(集成到业务逻辑最方便)."""
        return self.classify(text, top=1)[0][0]

    @property
    def labels(self):
        return sorted(self.label2id, key=self.label2id.get)


def main():
    ap = argparse.ArgumentParser(description="导入并试用驯化好的模型")
    ap.add_argument("--model-dir", default="output",
                    help="训练产物目录(默认 entity_train/output)")
    ap.add_argument("--text", required=True, help="要判断的代码片段")
    ap.add_argument("--top", type=int, default=3)
    args = ap.parse_args()

    cls = EntityClassifier(args.model_dir)
    print(f"标签集: {cls.labels}")
    for lab, p in cls.classify(args.text, top=args.top):
        print(f"  {lab:14s} {p:.4f}")


if __name__ == "__main__":
    main()
