---
来源: _tmp_session/vtest-1025/CONTRACT.md；docs/00_项目总览/交接_20261024_空地协同接手.md；docs/20_资料索引库/空地链路/05_步进协同与空地通信_实测复盘_20261024.md
取证: 实/档（侦察于 2026-10-25 14:1x，逐条带路径）
更新: 2026-10-25
status: in_progress
ns: vtest-1025
agent: video-bench
树节点: VT-3
写域: _tmp_session/vtest-1025/detbench/** 本卡
摘要: 离线检测基准：在 Windows 上按 dzuav 逐字复刻前端（1280 letterbox/pad114/RGB/÷255），跑 ONNX 模型 + 格号分类 + PnP 位姿，输出 CONTRACT §1 的 det.jsonl，并给出每类 P/R 与格命中率。
---

# VT-3-DETBENCH-20261025

## 1 五必填

- **读过的台账**：D-015（段边界等 layout ≤10 s）· D-022（布局过期只拦新规划）· D-029（不改 ticksPerMeter，先量）· D-031（靶速封死 1.0 m/s）· D-038/D-039（`field_layout` 唯一构造器 = `build_layout`，发射端必须 import）。
- **复用检索**：见本卡 §2；新增文件一律落 `_tmp_session/vtest-1025/`，**不注册进 复用清单**（过程件），进 `tools/` 才登记。
- **新增轮子及理由**：见 §2 末行"为什么现有不够"。
- **验收命令**：见本卡 §4。
- **回滚点**：本卡只写 `_tmp_session/vtest-1025/`；删目录即回滚，不动仓库。

## 2 事实基线（复刻口径，逐条可对源码）

- 输入：`letterbox(s=min(1280/w,1280/h)) → copyMakeBorder 到 1280×1280，pad=114 → blobFromImage(1/255, 1280, mean={}, swapRB=true)`（`HbbDetector.cpp:39-48`）。
- 输出：`[1,11,33600]` = 4 bbox + **7 类**，类序 `{mine,car,arrow,target_red,cell_label,target_blue,obstacle}`；解码 `cls=argmax(score[4:11])`、`conf<0.25` 丢、per-class NMS(0.45)、框坐标 `/s` 还原（`:51-91`）。
- 格号分类：96×96 双头 MobileNetV3，`letter_logits[14]` + `row_logits[11]`（`CellClassifier.cpp`）。
- 位姿：`MapLocalizer` 用格号地标的 **PnP** 解 `rvec/tvec`，内参 `K=(995.153295,0,1010.494099; 0,997.758431,481.641588; 0,0,1)`、畸变 `(-0.041172,-0.003300,-0.008916,0.000707,0)`，**按 1920×1080 标定**；检测框中心再 `project()` 到世界系（`MapLocalizer.cpp:17-25,46-153`）。
- 模型：`git show HEAD:air_ground_models/hbb_detector_d13_20261016.onnx`（指针文件 `current_hbb_detector` 指向它）与 `cell_classifier_v4d_20261015.onnx` **在工作树被删、但在 HEAD 里**，用 `git show > 临时目录` 取回，**不要 checkout 覆盖工作树**。
- ⚠ 已知差异：机上现役引擎是 **7 类**；d13 是非现役新模型。**必须先用 onnxruntime 打印两模型 I/O 形状**（`InferenceSession.get_inputs/get_outputs`），按实际通道数解码，并在报告里写清"用的是哪个模型、与机上差在哪"。别把 d13 的成绩冒充机上 7 类。

**为什么现有不够**：`dzsmartscout/tools/perception` 是**单帧整场俯视**的规则管线（颜色+几何），不是 dzuav 的 HBB 检测器；两者不可互替（一个要四角标定，一个要 1280 letterbox）。本评测器只做离线打分，**不进生产链路**。

## 3 交付物

1. `detbench/run_onnx.py`：`--video --model --cell-model --fps 60 --out det.jsonl [--limit N] [--truth map6]`，按 CONTRACT §1 输出；每帧记 `decode_ms/infer_ms/dropped`；`--fps` 为 0 时不限速（吞吐测试）。
2. `detbench/pnp.py`：格号→PnP→pose→`fov_cells`（复用 `scout/fov.py` 的 2×4 足迹口径，**import 不重写**：`PYTHONPATH=dzrobot_v2/src/dzbrain`）。
3. `detbench/score.py`：真值（第六图：雷 8/障 8/靶 2/桥 K5,K7）→ 每类 P/R、格命中率（±1 格另计）、虚警 Top10、漏检 Top10。
4. `detbench/REPORT.md`：模型 I/O 形状、预处理一致性核对、吞吐（CPU 实测 fps）、精度表、能否进阶段 B 的结论。

## 4 验收

`python run_onnx.py --video drone_recordings/cam_1080p_5s.mp4 --limit 20 --out /tmp/t.jsonl` exit 0，20 行合法 JSON；`score.py` 在自造合成图上 P=R=1.0（用作回归）。用户视频到达后 5 分钟内出第一张精度表。

## 5 边界

只写 `detbench/`；不碰 `dzsmartscout/tools/perception` 生产件；不声称与机上 TensorRT 逐位一致（写"同构评测"）。

## 5 禁止项

冻结区、`git add .`、设备写操作、非 ASCII 的机端 .sh 一律禁止；设备当前不可达（车 192.168.0.4 / 机 192.168.0.5 均 100% 丢包，2026-10-25 14:1x 实测）。

## 6 文档变更

无（本卡 + 树 + CONTRACT 均为过程件；结论成熟后再由 Lead 提炼进 `docs/20_资料索引库/空地链路/`）。
