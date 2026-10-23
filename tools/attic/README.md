# tools/attic —— 死代码归档（2026-10-17 横切扫描）

这里收**无引用的一次性/已被取代的脚本**：归档时（2026-10-17）已对全仓库
（tests/、CI、bat/ps1/sh、docs）grep 确认零引用。文件原样保留仅作考古参考，
不再维护、不进现役链路；要复活请先确认无人已重写同款工具。

| 文件 | 原位置 | 归档原因 |
|---|---|---|
| `b9_test.py` | `tools/` | 68.6° 摆位偏差一次性实验脚本（reset/goto/watch）；结论已固化进 `docs/车朝向根除方案.md` 与 `tests/test_sim_tracking.py` 回归 |
| `autolabel_yolo.py` | `tools/` | 已被 `tools/ml/autolabel_v4d.py` 取代的旧版自动预标 |
| `debug_cell_data.py` | 根目录 | 格号数据集一次性 debug 脚本 |
| `augment_cell_labels.py` | 根目录 | 格号标签一次性增广脚本 |
| `crop_grid_from_referee.py` | 根目录 | 从裁判系统截图裁格子的一次性工具 |
| `make_contact_sheet.py` | 根目录 | 拼图（contact sheet）一次性工具 |
| `prelabel_official_mines.py` | 根目录 | 官方雷图一次性预标脚本 |
| `split_cell_labels.py` | 根目录 | 格号标签一次性切分脚本 |
| `test_crops_onnx.py` | 根目录 | 裁片 ONNX 一次性验证脚本（非 pytest 套件成员） |
| `convert_yolo_to_xanylabeling_json.py` | 根目录 | YOLO→X-AnyLabeling 一次性转换；现役改编版是 `tools/convert_capture_to_xanylabeling.py` |
