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
| `hbb_detector.onnx` | `85056EF546FA42C7656133945058FD30` | 36.66 MB | Orin `~/dzuav/models/`（出厂部署） | ⚠️ **出厂 7 类检测器（11 通道），已作废留档，勿部署**——与现役 `HbbDetector`（17 通道硬校验）不符，上机即 throw | 2026-10-02 |
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
| `hbb_detector_7cls_distill_20261011.onnx` | `E93DEC04F9C7D8F272CC11624261741D` | 38.44 MB | 本次训练产物（tag=7cls_distill） | ⚠️ **7 类旧契约（11 通道），已作废留档，勿部署**——机上按 17 通道硬校验；曾是 current_hbb_detector 指针目标 | 2026-10-11 |
| `cell_classifier_v4c_ft_20261010.onnx` | `11BE1A3CD2780F67CBB8273B8AA0A410` | 4.34 MB | v4c 微调产物 | ⚠️ **已被 v4d 取代，留档勿部署**（形状契约与 v4d 相同，校验区分不了，只认指针）；曾是 current_cell_classifier 指针目标 | 2026-10-15 |
| `hbb_detector_7cls_local_20261013.onnx` | `E4B0FCE3E78062182C6C46F6E8A762E5` | 38.44 MB | 本地补充训练导出 | ⚠️ **评估失败品（val-B 89.19% 劣于现役），勿部署**，仅留档 | 2026-10-15 |
| `hbb_detector_7cls_local_20261013.pt` | `B84DE1A3CA87DA24BFD2D290CAFD1F75` | 19.37 MB | 同上 | ⚠️ 同上（.pt 属中间产物，按规则本不该入库，留档备查） | 2026-10-15 |
| `aerial_field_detector.pt` | `E28A3165B14793A78710FF9197579D38` | 76.09 MB | 早期航拍检测器权重 | ⚠️ 历史留档（超出 50MB 建议，勿部署，勿再传此类 .pt） | 2026-10-15 |
| `hbb_detector_d13_20261016.onnx` | `9DAE6083239B3B6C0B553C5F94261E78` | 38.45 MB | D 方案 13 类统一模型（源 `dk_train_pack_20261016/models/hbb_d13.onnx`，权重 `hbb_d13_best.pt`） | **无人机主检测器现役**（13 类），见 current_hbb_detector 指针。契约 `images[1,3,1280,1280] → output0[1,17,33600]`，与 `HbbDetector.cpp` 硬校验一致。同族 `hbb_d13_640.onnx` / `hbb_d13_ft640.onnx` 是 640 分辨率（8400 锚点），**不符合本契约，未入库、勿部署** | 2026-10-20 |
| `cell_classifier_v4d_20261015.onnx` | `9DE102A6519127EEB787381FEEFDC14D` | 4.34 MB | v4d 云端训练产物（源 `cloud_train_20261015/deploy_pkg/`） | **格号分类器现役**（v4d），见 current_cell_classifier 指针。106/106 全真值裁剪验证；`cell_classifier_v4d_perm5_20261017.onnx` 移位方案**已作废**（会把目前读对的字符也搞错），未入库 | 2026-10-20 |

## 当前建议部署组合（无人机 Orin）

| 用途 | 模型（current 指针） | 机上引擎名（`orin_run_dzuav.sh` 默认） | 依据 |
|---|---|---|---|
| 主检测器 | `hbb_detector_d13_20261016.onnx` | `hbb_detector_d13_fp16.engine` | 13 类 D 方案，`types.hpp` 自 2026-10-16 起按 17 通道硬校验；`video_recon.py` 的 D13Detector 用的就是这份 |
| 格号分类器 | `cell_classifier_v4d_20261015.onnx` | `cell_classifier_v4d_fp16.engine` | v4d，2026-10-17 视频侦察全天验证 106/106 |

⚠️ **2026-10-20 之前的旧组合（`hbb_detector_7cls_distill_20261011` + `cell_classifier_v4c_ft_20261010`）已作废**：
7 类检测器输出 11 通道，机上 `HbbDetector` 构造期即 throw；v4c 与 v4d 形状契约相同，
`verify_contract.py` **区分不了两者**，只能靠文件名/指针把关。留档行仅备查。

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
- `hbb_detector`：输入 `images [N,3,1280,1280]`，输出 `output0 [N,17,33600]`
  （17 = 4 bbox + 13 类；类目数由 `onboard/dzuav/include/dzuav/types.hpp` 的
  `kObjectClassCount` 单点派生，`verify_contract.py` 现场解析，勿在本文件写死）
- `cell_classifier`：输入 `images [N,3,96,96]`，输出 `letter_logits [N,14]` + `row_logits [N,11]`

⚠️ **11 通道（7 类）是 2026-10-16 之前的旧契约**，与机上 `HbbDetector.cpp` 的
硬校验 `[1,17,33600]` 不符：7 类引擎一上机就在构造期 throw。下表里 7cls 各行
仅作历史留档，**不可用于当前 13 类部署线**。

车端 `.rknn`（`light_det.rknn`）输出必须是 `[1,9,8400]`，否则 `det_node` 解不出来。
契约校验同时在 pytest（`tests/test_model_contract.py`）里持续守护。

**泛化门禁（2026-10-14 起强制）**：hbb 发布除契约校验外，还必须跨域留出评估
不退化才放行（`tests/test_release_gate.py` 守护判定逻辑）。门禁指标见文末
「泛化评估记录」表——发布即留痕，谁在什么留出集上、召回多少、比现役基线
差多少，一目了然。
