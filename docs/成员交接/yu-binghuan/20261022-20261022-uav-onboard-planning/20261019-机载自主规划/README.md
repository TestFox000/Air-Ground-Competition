# 机载自主规划 · 13 类主链接入

> 依据官方规则（`docs/raw_txt/02_15空地协同侦排挑战赛.txt` 第 2 页）：
> 「无人机的**定位、建图、识别、规划与控制**等实时任务须由**机载平台自主完成**」。
> 无人机不能起飞时，以「只喂视频、其余机载自主完成」作为等价验收。

## 两条链路别混（2026-10-22 晚补）

| 链路 | 入口 | 用途 |
|---|---|---|
| **离线视频侦察（现行）** | 机上 `tools/orin_video_recon.sh` → `src/ag_integrate/ag_integrate/video_recon.py` | 任意视频 → 13 类识别 → 落格 → 路线 → 渲染图；默认 `stride 5`（实测最优点 8/8）；缓存命中按**当前门控**秒级重算（`--reaggregate`） |
| 机载实时链 | `onboard/dzuav`（C++）+ `main13_plan` + `tools/orin_plan_sidecar.py` | 相机实时 → UDP:50000 → 0x4003 给车；本文下面的 `deploy_plan13.py` 属这条 |

识别口径的演进（雷 8/8、`stride` 语义、靶裁决、`--replay-safe`）都在**离线链路**
的 `video_recon.py` 里；C++ 实时链的地面参数与门槛另见
`机载规划差距与移植清单.md`。

## 结论（2026-10-22 实测）

**已跑通**：13 类主链（D13 检测 + v4d 格号 + PnP 定位）已接入规划层，
只喂一段视频即可在 Orin 上自主产出 `route_plan.json`（给车的路线）。

同一段视频（`cam_20261017_205245`）：

| 指标 | 抽帧 5（`DZUAV_STRIDE=5`） | 逐帧（stride=1） |
|---|---|---|
| 端到端耗时 | **67.7 s / 67.3 s**（两轮） | **276.9 s** |
| 处理帧数 | 453 / 2264 | 2264 / 2264 |
| 定位成功帧 | 29（6.4%） | 131（5.8%） |
| 规划触发次数 | 5 | 14 |
| 输出 | `route_plan.json` `ok=true`，44 格 | `ok=true`，58 格，5 雷 |
| 跨轮一致性 | 两轮路线 md5 完全一致 | — |

跨视频（逐帧，`air_plan_bench.py` 产物见 `bench_out/report.md`）：

| 视频 | 耗时 | 帧数 | 定位率 | 规划次数 | 首份规划 |
|---|---|---|---|---|---|
| `cam_20261018_131226` | 217.1 s | 1789 | 11.0% | 23 | 0.63 s |
| `cam_20261017_205906` | 42.6 s | 288 | 13.2% | 9 | 0.61 s |

两段视频均退出码 0、无崩溃、跨轮路线一致；首份 `route_plan.json` 约 **0.6 s** 产出。

## 本次做了什么

1. **定位门槛修复**（阻断项）
   13 类栈的 `MapLocalizer` 仍是旧门槛：凸包面积 `0.02`、PnP 内点绝对下限 `12`、
   单应内点绝对下限 `8`。近距视野只有 4~6 个地标时数学上无法满足 → `localized` 恒 0。
   改为已验证的比例制：`0.005` / `max(4,30%)` / `max(5,30%)`。
   实测同一视频 `localized=1` 从 **0 → 131 帧**。

2. **规划层算法级移植**（不引入第二套血缘）
   把机上已有的 `CellGrid`（证据累积）+ `RoutePlanner`（Held-Karp + Floyd-Warshall）
   接入 13 类主链，按官方场地语义落格：
   - `paper` 在 **K 列** = 桥洞可通行；其余 `paper` = 雷
   - `car_start` / `car_end` = 通格（起点/终点）
   - `target_red` / `target_blue` = 靶（占相邻两格，规划禁入，并自动绕行其侧面）
   - `obstacle` / `bucket_*` = 障碍
   - 不引入绿色像素投票、不预设桥体行（与 Python 版 `ag_integrate` 口径一致）

3. **固定起终点参数**（按用户指定）
   官方摆位常量，避免视频没拍到车起终点时无解，也避免桥洞通格被误当起点：
   - 默认 `N10 → N5`
   - 可覆盖：`DZUAV_START_CELL=N10 DZUAV_GOAL_CELL=N5`

## 用法

```powershell
# 部署（自动备份 → 推送 → 编译验证 → 冒烟）
python tools\deploy_plan13.py --host 192.168.0.197

# 只跑冒烟
python tools\deploy_plan13.py --host 192.168.0.197 --smoke-only --stride 5

# 一键回滚（还原 4 个原文件、删除新增文件并重编）
python tools\deploy_plan13.py --host 192.168.0.197 --rollback
```

机上直接跑：

```bash
DZUAV_STRIDE=5 ~/dzuav_deploy/air_ground/onboard/dzuav_mjpg/build/dzuav \
  ~/Videos/recordings/cam_20261017_205245.mp4 \
  ~/dzuav/engines/hbb_detector_d13_fp16.engine \
  ~/dzuav/engines/cell_classifier_v4d_fp16.engine 50000 "" \
  /tmp/cell_matrix.json /tmp/route_plan.json /tmp/planned_path.txt
```

## 文件

| 文件 | 作用 |
|---|---|
| `src/main13_plan.cpp` | 13 类主链 + 规划层（部署到机上 `src/main.cpp`） |
| `src/MapLocalizer.{hpp,cpp}` | 定位门槛修复 + `cellAt/projectAtHeight` |
| `src/CellGrid.{hpp,cpp}` | 格子证据累积（算法层，原样复用） |
| `src/RoutePlanner.{hpp,cpp}` | 路线规划 + 固定起终点参数 |
| `src/CMakeLists.txt` | 13 类构建（新增 CellGrid/RoutePlanner） |
| `tools/deploy_plan13.py` | 部署 / 编译验证 / 冒烟 / 回滚 |
| `tools/air_plan_bench.py` | 多视频 × 多轮评测（时间/稳定性） |

## 已知边界

- 单段巡拍视频定位覆盖率约 6%（地标少、视角低），矩阵覆盖率 < 50%，规划器会告警。
- 路线中 `mines_visited > mines_total` 是"绕行经过靶侧面"造成的重复经过，不是漏雷。
- 官方 **8 雷基线只作校验**：离线链路会报"仅检出 N/8（漏检风险）"或"N>8（必有误检）"，
  但**不会凭空补位**——雷位每局随机，视频没拍到的位置无从推断。实测同一段视频
  `stride 15` 只剩 3 颗、`stride 5` 出 8 颗（=基线）、`stride 1` 逐帧反而 14 颗
  （弱误检累积票 + 幻影消除误杀真雷）——采样两端都会掉精度。
- 机上原件已备份为 `*.bak_plan13_orig`，`--rollback` 一键还原。
