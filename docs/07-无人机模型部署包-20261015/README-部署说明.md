# 无人机模型部署包 2026-10-15

## 一、模型清单与精度

| 文件 | 用途 | 精度（val-B 综合口径，排除 car） |
|---|---|---|
| hbb_detector_7cls_minecpv2_20261015.onnx | **现役主检测器**（含 mine_cp_v2 双域增强 + 桶对照） | **91.60%**（按类别最优阈值） |
| hbb_detector_7cls_combined_20261015.onnx | 备选 A（无 minecpv2 微调） | 91.60% |
| hbb_detector_7cls_direct_20261015.onnx | 备选 B（无桶对照对照组） | 91.29% |
| cell_classifier_v4d_20261015.onnx | **现役格号分类器** | joint_acc **0.8701**（v4c 为 0.8588） |

参考基线：ep75 起点 91.28%；本地 combined 90.53%；无人机旧引擎（7 月 28 日）≈ distill 90.30% 口径。

## 二、无人机上的路径（Orin Nano，192.168.55.1，用户 duzhong）

```
~/dzuav/models/                                    # ONNX（4 个新模型已上传）
~/dzuav/engines/
├── hbb_detector_fp16.engine        # 现役 HBB 引擎 ← minecpv2（构建中，10-15 分钟）
├── cell_classifier_fp16.engine     # 现役格号引擎 ← v4d（构建中）
├── hbb_detector_combined_fp16.engine   # 备选 A 引擎
└── hbb_detector_direct_fp16.engine     # 备选 B 引擎
~/dzuav/backup_20261015/
├── models_orig/                    # 原 models 完整备份
└── engines_orig/                   # 原 engines 完整备份（7 月 28 日旧版）
```

## 三、minecpv2 部署参数（按类别最优阈值，val-B 91.60% 对应值）

```python
conf_per_class = [0.5, 0.25, 0.05, 0.25, 0.5, 0.3, 0.5]
# 类别顺序: mine, car, arrow, target_red, cell_label, target_blue, obstacle
edge_mine_thr  = 0.005   # mine 边缘过滤（中心落在画面边缘 0.5% 内剔除）
```

注意：arrow 与 cell 的最优 conf 为 0.05（低阈值保召回），若沿用全局 0.25 精度降至 89.69%。

## 四、引擎构建/回滚命令（无人机上）

```bash
# 重建现役引擎（已在后台自动执行，若需手动）
cd ~/dzuav
trtexec --onnx=models/hbb_detector_7cls_minecpv2_20261015.onnx --fp16 \
    --saveEngine=engines/hbb_detector_fp16.engine
trtexec --onnx=models/cell_classifier_v4d_20261015.onnx --fp16 \
    --minShapes=images:1x3x96x96 --optShapes=images:64x3x96x96 \
    --maxShapes=images:256x3x96x96 \
    --saveEngine=engines/cell_classifier_fp16.engine

# 回滚旧模型（完全恢复原状）
cp -a ~/dzuav/backup_20261015/models_orig/. ~/dzuav/models/
cp -a ~/dzuav/backup_20261015/engines_orig/. ~/dzuav/engines/
```

## 五、target_merged 标靶训练（补充）

云端 4090 正在训练（yolo11s/960/150ep，GPU 99%），完成后 ONNX 将补充到本目录。
数据：652 张（blue bucket / car / green bucket / plane）。

## 六、来源与复现

- 训练实例：AutoDL 4090（connect.bjb1.seetacloud.com:27379）
- 训练产物：/root/autodl-tmp/ag_cloud/runs/{combined_cloud, direct_minecpv2, minecpv2_cloud}/weights/best.pt
- v4d 产物：/root/autodl-tmp/cellv4d/runs/cell_v4d/best.pt
- 评估报告：云端 /root/autodl-tmp/ag_cloud/outputs/REPORT.md
- 桶对照数据 A/B：minecpv2（有）91.60% vs direct（无）91.29%，obstacle 误报 4→1
