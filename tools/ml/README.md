# 模型训练脚手架

目标：训出**能直接喂给车端 `det_node`** 的 YOLOv8-pose 模型，且类别顺序/关键点与车端契约完全一致。

## 契约（改动前先读这一段）

车端 `~/dzrobot/src/rknn_pt/include/det/postprocess.h` 写死了：

| 项 | 值 | 含义 |
|---|---|---|
| `OBJ_CLASS_NUM` | **2** | **0 = blue_target（蓝面 = 敌方 = 可打击）**、1 = red_target（红面 = 友方 = 禁止打击） |
| `KPT_NUM` / `KPT_DIM` | 1 / 3 | `kpt_shape: [1, 3]`，关键点 = 靶标发光中心 |
| 输出 | `[1, 9, 8400]` | `4(框) + 2(类) + 3(关键点)`，**DFL 已在模型内解码** |
| 单位 | 输入尺度像素 | 框与关键点都是 640 输入下的像素值，类别分数已激活 |

**类别顺序反了 = 把友方当敌方打**（每次 −5，最多 −15）。所以训练脚本会强制检查。

## 四步走

```bash
pip install -r requirements-train.txt

# ① 数据集体检（不需要 GPU，先跑这个）
python tools/ml/dataset_check.py --root /data/air_ground --json /data/report.json

# ② 复制模板改路径后训练
cp tools/ml/data.template.yaml /data/data.yaml
python tools/ml/train_pose.py --data /data/data.yaml --epochs 120 --batch 16

# ③ 导出 onnx（本机可做）
python tools/ml/export_rknn.py --weights runs/pose/air_ground_pose/weights/best.pt --onnx-only

# ④ 转 rknn（Linux x86 上做；量化要 30~100 张真图清单）
python tools/ml/export_rknn.py --weights best.pt --quant-images /data/imgs.txt
```

导出流程每一步都会跑 `tools/ml/verify_contract.py` 检查形状契约，
不符合就中止（**宁可不导出，也不要导出个上不了车的**）。

## 数据

- 标注格式：YOLO-pose，每行 `cls cx cy w h kx ky kv`（8 个字段）
- 建议直接用**车端 640×480 原图**（与 `det_node` 输入一致），letterbox 由训练/推理端统一做
- 车端 `~/dzrobot/camera_dataset/` 有 1244 张现场图（**无标注**），可作为补充真实域数据
- 赛事资料里的地雷图片可用于扩充地雷类别（当前模型只有蓝/红靶两类）

## 上车前的最后一道关

形状对了不代表数值语义对（`cls` 是否需要 sigmoid、关键点是像素还是归一化）。
必须**在板端用真图**跑一遍：

```bash
python tools/verify_decode.py --model best_new.rknn --image frame.jpg
```

结论写进 `params.yaml` 的 `cls_activation` / `kpt_unit`（这两个参数错了，检测会"看起来正常但一个目标都打不中"）。
