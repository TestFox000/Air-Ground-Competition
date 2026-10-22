# 机载自主规划：13 类主链接入规划层与视频跑道实测

- 归档日期：2026-10-22
- 归档目录：`docs/成员交接/yu-binghuan/20261022-20261022-uav-onboard-planning/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

依据官方规则「无人机定位/建图/识别/规划/控制须机载自主完成」。本次修复 13 类栈 MapLocalizer 旧门槛（localized 恒 0），并把 CellGrid+RoutePlanner 算法级接入 13 类主链；只喂视频即可产出 route_plan.json。实测：同段视频抽帧5耗时 67s、逐帧 277s；跨视频退出码 0、无崩溃、跨轮路线一致、首份规划约 0.6s。机上原件已备份 .bak_plan13_orig，可一键回滚。

### 最新进展（2026-10-22 晚，覆盖上文中的旧数字）

- **识别口径**：视频侦察统一到 `tools/orin_video_recon.sh`（默认 `stride 5` = 实测最优点）。
  同一段视频实测：`stride 15` → 3 雷（漏检）、`stride 5` → 8 雷（=官方基线）、
  `stride 1` 逐帧 → 14 雷（7 个误检，且幻影消除误杀真雷 `E2`）——**两端都会掉精度**。
- **缺雷根因与修复**：`G9` 有 3 帧独立票但均值置信度 0.336 < 0.35，被均值门卡掉；
  放宽单框阈值(0.22→0.15)与收紧均值门互相打架。现改为 **≥3 票免疫均值门**，
  均值门只拦 2 票弱格；`check["排雷覆盖"]` 对官方基线 8 双向负责
  （不足报漏检、超过报误检），不再只打「覆盖 ✓」。
- **缓存语义**：命中缓存不再沿用旧 `field_state`，而是用 `frames_meta.json`
  按**当前门控**重算（`--reaggregate`，秒级）；门控修复不会被缓存吞掉。
- **路线结果**：41 格覆盖全部 8 雷，`射击停靠 D8`（移动靶两段式确认位）、避障通过。
  物证：`验收_stride5_field_state.json` / `验收_stride5_route_plan.json` /
  `验收_stride5_route_render.png`。
- **口径提醒**：`--stride` 只有"采样漏检"一种含义，没有"快一点的同结果"；
  正式结论一律逐帧。`video_field_recon.py`、`orin_demo_recon.sh`、
  `orin_verify_recon.sh` 已退役。

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
| `20261019-机载自主规划/机载规划差距与移植清单.md` | 43,556 B |
| `20261019-机载自主规划/验收_stride5_field_state.json` | 3,579 B |
| `20261019-机载自主规划/验收_stride5_route_plan.json` | 2,139 B |
| `20261019-机载自主规划/验收_stride5_route_render.png` | 36,590 B |
| `20261019-机载自主规划/验收_stride1_field_state.json` | 5,295 B |
| `20261019-机载自主规划/验收_stride1_route_plan.json` | 3,293 B |
| `20261019-机载自主规划/验收_stride1_route_render.png` | 41,067 B |

## 复现方式

```bash
# 视频侦察 → 路线规划（机上通用入口，任意视频；默认 stride 5 = 实测最优 8/8）
bash tools/orin_video_recon.sh <video.mp4>
bash tools/orin_video_recon.sh --replay-last   # 秒级：复用识别结果，按当前门控重算
```

> `video_field_recon.py` 与 `--stride 15` 是 2026-10-18 的旧归档口径，已退役：
> stride 15 会漏掉只短暂可见的雷（实测同段视频 8 颗只剩 2~3 颗，见 §6.10.2）。
> 本 README 由 push_deliverable.py 自动生成。