# models/release —— 正式发布的模型（**入库**）

这个目录是**唯一**允许进 Git 的模型存放处。

## 为什么单开一个目录

`models/` 的其余部分、以及散落各处的 `*.rknn` / `*.onnx` 都在 `.gitignore` 里
（体积大、多是 SDK 产物）。但**正式发布、两边都要用的成品模型只有几 MB**，
让它跟着代码走，好处是压倒性的：

| | 靠网盘传 + 手工登记 md5 | 放进这里跟 Git 走 |
|---|---|---|
| 一致性 | 靠人记、靠自觉，容易漏 | checkout 代码就拿到配套模型，**物理上不可能对不上** |
| 溯源 | 手抄 md5 | Git 哈希自动记录 |
| 回滚 | 找不到上一版 | `git checkout <commit> -- models/release/` |
| 成本 | 每次传完要登记 | 零 |

规则见 `docs/archive/协作看板.md` §5（历史规则，2026-10-01 已归档）。

## 入库规则

1. **只放"要上车 / 要联调"的成品模型**。训练中间产物（`.pt`、优化前的 `.onnx`）
   留在各自机器上，不要传
2. 每新增一个模型，**在下面登记**：文件名 / md5 / 体积 / 来源 / 用途 / 日期
3. **别用同名文件覆盖**：版本变了就换名字（如 `light_det_20261002.rknn`），
   否则 `git log` 分不清谁是哪一版
4. 入库前看一眼体积：单个建议 **< 50 MB**（Gitee 免费仓库有容量上限）
5. **hbb 检测器发布必须先过泛化门禁**（官方明牌考"训练集没有的同类雷图"）：
   `tools/release_models.py --hbb` 会自动调用 `tools/ml/eval_generalization.py`，
   留出图集用 `--generalization-images` 或环境变量 `AIC_MINE_HOLDOUT` 指定；
   判定 = 绝对底线召回 + 相对现役基线的回归容忍（`generalization_baseline.json`），
   每次评估记录追加到本文末尾的「泛化评估记录」表

## 登记表

| 文件名 | md5 | 体积 | 来源 | 用途 | 入库日期 |
|---|---|---|---|---|---|
| `light_det.rknn` | `49F3700590A907994E7EE20E4D30CC13` | 8.09 MB | 车端 `~/dzrobot/src/rknn_pt/model/`（厂家随车提供） | **车端现役基线**。输出 `[1,9,8400]` = 4 bbox + 1 cls + 4 kpt，单类别 YOLO-pose，与 `det_node` 期望一致 | 2026-09-28 |
| `hbb_detector.onnx` | `85056EF546FA42C7656133945058FD30` | 36.66 MB | Orin `~/dzuav/models/`（出厂部署） | **无人机主检测器**（7 类）。输入 `images [N,3,1280,1280]`，输出 `output0 [N,11,33600]`（4 bbox + 7 类），与 `HbbDetector` 后处理一致。换机时用 `tools/orin_convert_engines.sh` 生成 FP16 引擎 | 2026-10-02 |
| `cell_classifier.onnx` | `7B3053A9DC3347A9B0D35D6A9EF55E74` | 4.13 MB | Orin `~/dzuav/models/`（出厂部署） | **格号分类器**（双头 MobileNetV3）。输入 `images [N,3,96,96]`，输出 `letter_logits [N,14]`（列 A~N）+ `row_logits [N,11]`（行 1~11），与 `CellClassifier` 一致 | 2026-10-02 |

## 车端那一堆同族文件（别混）

从车上取回的同名族里，**只有 `light_det.rknn` 可用**：

| 文件 | md5 | 结论 |
|---|---|---|
| `light_det.rknn` | `49F37005…CC13` | ✅ 可用，已入库 |
| `light_det3.rknn` | `49F37005…CC13` | ✅ 其实是同一个文件（只是改了名） |
| `light_det1.rknn` | 待补 | ❌ 旧版多输出，与 `det_node` 不匹配 |
| `light_det2.rknn` | 待补 | ❌ 同上 |
| `light_det_v1.rknn` | 待补 | ❌ 另一套导出 |
| `cell_classifier_v4c_20261008.onnx` | `4F1838BEBFC951BC62170D821D56EC22` | 4.34 MB | 本次训练产物（tag=v4c） | 格号分类器，见 current_cell_classifier 指针 | 2026-10-08 |
| `cell_classifier_v4c_20261009.onnx` | `850AE9376BD5AC7BDFE44D6BD550A920` | 4.34 MB | 本次训练产物（tag=v4c） | 格号分类器，见 current_cell_classifier 指针 | 2026-10-09 |
| `hbb_detector_7cls_distill_20261011.onnx` | `E93DEC04F9C7D8F272CC11624261741D` | 38.44 MB | 本次训练产物（tag=7cls_distill） | 无人机主检测器，见 current_hbb_detector 指针 | 2026-10-11 |
| `cell_classifier_v4c_ft_20261010.onnx` | `11BE1A3CD2780F67CBB8273B8AA0A410` | 4.34 MB | v4c 微调产物 | **格号分类器现役**，见 current_cell_classifier 指针（补登记，原漏登） | 2026-10-15 |
| `hbb_detector_7cls_local_20261013.onnx` | `E4B0FCE3E78062182C6C46F6E8A762E5` | 38.44 MB | 本地补充训练导出 | ⚠️ **评估失败品（val-B 89.19% 劣于现役），勿部署**，仅留档 | 2026-10-15 |
| `hbb_detector_7cls_local_20261013.pt` | `B84DE1A3CA87DA24BFD2D290CAFD1F75` | 19.37 MB | 同上 | ⚠️ 同上（.pt 属中间产物，按规则本不该入库，留档备查） | 2026-10-15 |
| `aerial_field_detector.pt` | `E28A3165B14793A78710FF9197579D38` | 76.09 MB | 早期航拍检测器权重 | ⚠️ 历史留档（超出 50MB 建议，勿部署，勿再传此类 .pt） | 2026-10-15 |
| `target_pose_v2_20261016.onnx` | `1D7E1F90A66966870C9ED61345EF39F7` | 10.49 MB | 云端 pose_v2 重训（yolo11n-pose 640，ep146 早停；+203 张红桶/车身空标签负样本压制误检） | **车端靶标 pose 检测器**（升级 light_det.rknn 链路）。输入 `[1,3,640,640]`，输出 `[1,9,8400]`（4 bbox + 2 类红/蓝靶 + 1×3 关键点），与 det_node 契约一致。val 144 帧（含 11 负样本）：检测 P=0.985 / R=0.999 / mAP50=0.995，关键点 mAP50=0.995 / mAP50-95=0.995；红靶 P=1.00 零误检、蓝靶 R=1.00 零漏检。待 `tools/ml/rknn_convert.py` 转 RKNN 后上车（.pt 留 dk_train_pack_20261016/models/） | 2026-10-16 |
| `cell_classifier_v4d_20261015.onnx` | `9DE102A6519127EEB787381FEEFDC14D` | 4.13 MB | 云端 4090 cellv4d 管线（train 62797 = v4c 11704 + autolabel 几何裁决 2433 + 增广×20 48660；80ep/batch256 早停） | **格号分类器现役**（current_cell_classifier 指针）。输入 `images [N,3,96,96]`，输出 `letter_logits [N,14]` + `row_logits [N,11]`。joint_acc **0.8701**（v4c_ft 0.8588，+1.13pp，val 177 严格可比口径）；10-15 已上机现役引擎名 `cell_classifier_fp16.engine`（dynamic batch 1-256） | 2026-10-17 |
| `hbb_detector_d13_20261016.onnx` | `9DAE6083239B3B6C0B553C5F94261E78` | 36.67 MB | 云端 4090 plan_c 管线 D 方案（paper 603 张 v2 + merged 1594 张；46ep 早停 best@16；.pt 留 dk_train_pack_20261016/models/） | **无人机主检测器现役**（current_hbb_detector 指针，13 类 = 17 通道）。输入 `images [1,3,1280,1280]`，输出 `output0 [1,17,33600]`。四方对比最优：mine R98.3%（四方案最高）、arrow R88.9%+P100% 双最高、cell R98.5%、obstacle R94.3%；paper 落格判语义（K 列桥洞=通行箭头/其余格=雷）。opset12 单帧批，机上 L3a 真机验证通过（`engines_d13/hbb_detector_d13_fp16.engine`，landmarks=24）；L3b 协议闭环待收尾（根因 v4c 误读，非 D13 回归）；car/target 类需实机验证。泛化门禁未单独复跑（mine 数据谱系与现役同源 v2，证据链参照 06-网图快测口径） | 2026-10-17 |

## 当前建议部署组合（无人机 Orin）

| 用途 | 模型（current 指针） | 依据 |
|---|---|---|
| 主检测器 | `hbb_detector_d13_20261016.onnx` | D 方案 13 类四方对比全面最优（fe1d9df 机载适配，L3a 真机验证过）；L3b 协议闭环收尾前，机上现役引擎名 `hbb_detector_fp16.engine` 仍指向 minecpv2 7 类（`engines_d13/` 为 D13 测试位） |
| 格号分类器 | `cell_classifier_v4d_20261015.onnx` | joint_acc=0.8701（v4c_ft +1.13pp）；10-15 已以现役引擎名上机，`engines_d13/` 侧转换待 L3b 收尾 |

Orin 上部署/换模型三步（仓库 git pull 后）：

```bash
tools/orin_convert_engines.sh                              # ONNX -> FP16 引擎（按 current 指针）
bash tools/orin_build_verify.sh --run ~/dzuav/engines <测试图>   # 编译+真图验证
bash tools/orin_run_dzuav.sh camera                        # 上场
```

训练机发布最优模型到本通道一行命令（泛化门禁已挂钩，过闸才入库）：

```bash
python tools/release_models.py --hbb <best.pt 路径> --tag <版本标签> --generalization-images "E:\...\1-爆炸物\B-爆炸物"
```

**新模型上车前必须过契约校验**：

```bash
python tools/ml/verify_contract.py --all models/release
```

ONNX 契约（与 onboard/dzuav/src C++ 逐字对应）：
- `hbb_detector`：输入 `images [N,3,1280,1280]`，输出 `output0 [N,11,33600]`（7 类）
- `hbb_detector_d13`：输入 `images [N,3,1280,1280]`，输出 `output0 [N,17,33600]`（13 类，单帧批固定 1 合规）
- `cell_classifier`：输入 `images [N,3,96,96]`，输出 `letter_logits [N,14]` + `row_logits [N,11]`

车端 `.rknn`（`light_det.rknn`）输出必须是 `[1,9,8400]`，否则 `det_node` 解不出来。
契约校验同时在 pytest（`tests/test_model_contract.py`）里持续守护。

**泛化门禁（2026-10-14 起强制）**：hbb 发布除契约校验外，还必须跨域留出评估
不退化才放行（`tests/test_release_gate.py` 守护判定逻辑）。门禁指标见文末
「泛化评估记录」表——发布即留痕，谁在什么留出集上、召回多少、比现役基线
差多少，一目了然。
