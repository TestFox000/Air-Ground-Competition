# 编号双头分类器

输入 YOLO 裁剪出的正方形编号图片，一次前向同时输出：

- 字母头：A–N，共 14 类；
- 数字头：1–11，共 11 类。

骨干为 ImageNet 预训练的 MobileNetV3-Small，输入尺寸默认 96×96。训练增强包含任意角度旋转、轻微透视、亮度/对比度变化和模糊，但不使用水平或垂直镜像。

## 训练

在项目根目录运行：

```bash
python -m src.cell_classifier.train \
  --csv datasets/编号/annotations.csv \
  --output runs/cell_classifier_v1 \
  --device cuda:0 \
  --batch-size 256 \
  --epochs 80
```

输出目录包含：

- `best.pt`：验证集完整编号准确率最高的权重；
- `last.pt`：最后一轮权重，可用于断点续训；
- `metrics.csv`：每轮 loss、accuracy、macro precision/recall/F1；
- `config.json`：训练参数和运行环境。

## 完整评估

```bash
python -m src.cell_classifier.evaluate \
  --weights runs/cell_classifier_v1/best.pt \
  --csv datasets/编号/annotations.csv \
  --split val \
  --device cuda:0 \
  --output runs/cell_classifier_v1/evaluation
```

评估目录包含总体指标 JSON、逐图预测 CSV，以及字母头和数字头的混淆矩阵。

## 推理

```bash
python -m src.cell_classifier.infer \
  --weights runs/cell_classifier_v1/best.pt \
  --source datasets/编号/images/val \
  --device cuda:0 \
  --output runs/cell_classifier_v1/predictions.jsonl
```

`--rotation-tta` 会对 0/90/180/270 度预测取平均，可提高极端旋转的稳定性，但推理量约为四倍。常规实时管线默认关闭。

代码集成时直接使用：

```python
from src.cell_classifier import CellIdRecognizer

recognizer = CellIdRecognizer("best.pt", device="cuda:0", half=True)
results = recognizer.predict_paths(["crop.png"])
print(results[0].cell_id, results[0].joint_confidence)
```

## ONNX / TensorRT

```bash
python -m src.cell_classifier.export_onnx \
  --weights runs/cell_classifier_v1/best.pt \
  --output runs/cell_classifier_v1/cell_classifier.onnx
```

ONNX 有一个 `images` 输入和 `letter_logits`、`row_logits` 两个输出，可继续在 Jetson Orin Nano 上构建 FP16/INT8 TensorRT engine。
