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
| `dzuav_mjpg/` | `onboard/dzuav_mjpg` | 机上13类部署线收编存档:板载 CellGrid/RoutePlanner 规划已被 sidecar 取代、参数序落后于 8df968f8、不在 orin_deploy 编排线(2026-10-25 未接线审计) |
| `orin_run_dzuav_d13.sh` | `tools/` | D13 侦察栈启动脚本,依赖 dzuav_mjpg 残留二进制;现役引擎线由 `tools/orin_run_dzuav.sh` 承担(默认即 D13+v4d) |
| `ground_projection.py`、`anchored_registration.py` | `src/ag_integrate/ag_integrate` | 2026-10-17 像素→场地系反投影方案;世界坐标已由 dzuav C++ PnP(types.hpp WorldObject)承担,Python 复线未采用、零引用(2026-10-25 未接线审计) |
