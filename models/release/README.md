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

**新模型上车前必须过契约校验**：

```bash
python tools/ml/verify_contract.py --all models/release
```

ONNX 契约（与 onboard/dzuav/src C++ 逐字对应）：
- `hbb_detector`：输入 `images [N,3,1280,1280]`，输出 `output0 [N,11,33600]`
- `cell_classifier`：输入 `images [N,3,96,96]`，输出 `letter_logits [N,14]` + `row_logits [N,11]`

车端 `.rknn`（`light_det.rknn`）输出必须是 `[1,9,8400]`，否则 `det_node` 解不出来。
契约校验同时在 pytest（`tests/test_model_contract.py`）里持续守护。
