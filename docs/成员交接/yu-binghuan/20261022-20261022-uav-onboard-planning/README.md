# 机载自主规划：13 类主链接入规划层与视频跑道实测

- 归档日期：2026-10-22
- 归档目录：`docs/成员交接/yu-binghuan/20261022-20261022-uav-onboard-planning/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

依据官方规则「无人机定位/建图/识别/规划/控制须机载自主完成」。本次修复 13 类栈 MapLocalizer 旧门槛（localized 恒 0），并把 CellGrid+RoutePlanner 算法级接入 13 类主链；只喂视频即可产出 route_plan.json。实测：同段视频抽帧5耗时 67s、逐帧 277s；跨视频退出码 0、无崩溃、跨轮路线一致、首份规划约 0.6s。机上原件已备份 .bak_plan13_orig，可一键回滚。

## 文件清单

| 文件 | 大小 |
|---|---|
| `20261019-机载自主规划/bench_out/metrics.json` | 16,675 B |
| `20261019-机载自主规划/bench_out/mjpg13/cam_20261017_205906/cell_matrix.json` | 17,081 B |
| `20261019-机载自主规划/bench_out/mjpg13/cam_20261017_205906/planned_path.txt` | 2,213 B |
| `20261019-机载自主规划/bench_out/mjpg13/cam_20261017_205906/route_plan.json` | 4,882 B |
| `20261019-机载自主规划/bench_out/mjpg13/cam_20261018_131226/cell_matrix.json` | 17,360 B |
| `20261019-机载自主规划/bench_out/mjpg13/cam_20261018_131226/planned_path.txt` | 2,369 B |
| `20261019-机载自主规划/bench_out/mjpg13/cam_20261018_131226/route_plan.json` | 5,693 B |
| `20261019-机载自主规划/bench_out/report.md` | 1,584 B |
| `20261019-机载自主规划/README.md` | 4,420 B |
| `20261019-机载自主规划/src/CellGrid.cpp` | 6,427 B |
| `20261019-机载自主规划/src/CellGrid.hpp` | 2,351 B |
| `20261019-机载自主规划/src/CMakeLists.txt` | 1,473 B |
| `20261019-机载自主规划/src/main13_plan.cpp` | 25,517 B |
| `20261019-机载自主规划/src/MapLocalizer.cpp` | 8,419 B |
| `20261019-机载自主规划/src/MapLocalizer.hpp` | 2,092 B |
| `20261019-机载自主规划/src/PassageDetector.cpp` | 1,103 B |
| `20261019-机载自主规划/src/PassageDetector.hpp` | 427 B |
| `20261019-机载自主规划/src/ref_cmakelists13.txt` | 1,622 B |
| `20261019-机载自主规划/src/ref_main13.cpp` | 16,740 B |
| `20261019-机载自主规划/src/ref_main7.cpp` | 19,948 B |
| `20261019-机载自主规划/src/ref_maplocalizer13.cpp` | 7,454 B |
| `20261019-机载自主规划/src/ref_maplocalizer13.hpp` | 1,271 B |
| `20261019-机载自主规划/src/ref_maplocalizer7.cpp` | 3,751 B |
| `20261019-机载自主规划/src/ref_maplocalizer7.hpp` | 2,011 B |
| `20261019-机载自主规划/src/ref_types13.hpp` | 1,845 B |
| `20261019-机载自主规划/src/ref_types7.hpp` | 1,258 B |
| `20261019-机载自主规划/src/RoutePlanner.cpp` | 17,773 B |
| `20261019-机载自主规划/src/RoutePlanner.hpp` | 2,475 B |
| `20261019-机载自主规划/tools/air_plan_bench.py` | 26,709 B |
| `20261019-机载自主规划/tools/deploy_plan13.py` | 7,408 B |
| `20261019-机载自主规划/代码审查-20261020.md` | 23,197 B |
| `20261019-机载自主规划/明日部署与协同说明.md` | 14,245 B |
| `20261019-机载自主规划/机载规划差距与移植清单.md` | 38,851 B |

## 复现方式

```bash
# 视频侦察管线（Windows 可直接跑，无需 ROS）
python video_field_recon.py <video.mp4> --stride 15 --out <输出目录>
python render_waypoint_map.py <输出目录>
```

> 本 README 由 push_deliverable.py 自动生成。