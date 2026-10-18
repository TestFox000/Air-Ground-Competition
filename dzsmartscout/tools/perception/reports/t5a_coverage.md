# t5a 证据：机侧覆盖 / 负证据报文（`coverage` / `slice` / `scan_id`）

- 卡片：`t5a`（air-ground-v3）· 依据 `dzrobot_v2/docs/黑图探索与动态规划_决策层方案.md` §4.2 / §5.1 / §12.2
- 写域：`dzsmartscout/tools/perception/**`（含 tests）· **未碰 `dzrobot_v2/**`**、未 commit
- 复跑（本机 Windows / Python 3.12）：

```bash
cd dzsmartscout/tools
python -m perception.run_tests            # 99 项（旧 85 + 新 14）
python -m perception.run_tests coverage   # 只跑 t5a 的 14 项
python -m perception.cli selftest         # 93 项（跳过 e2e/真图）
```

## 1. 改了什么（逐文件）

| 文件 | 变更 |
|---|---|
| `coverage.py`（新） | 片定义 `Slice`/`SliceCoverage`/`SlicePlan`、几何 `cells_of_route`、7 片/3 片/整场预设、`parse_slice_spec` |
| `protocol.py` | `build_field_verify` 增 `coverage/slice/scan_id/confidence`（additive，旧调用产物逐字节不变）；`validate_payload` 同套严格校验；`mirror_car_side_parse` 补旧车侧语义；新增 `make_scan_id` / `coverage_absent_cells` / `field_verify_absent` / `COVERAGE_MODES` |
| `pipeline.py` | `process(..., slices=..., scan_id=..., omit_empty_classes=...)` 逐片出 `field_verify`；`PerceptionResult.slices/scan_id`；`diagnostics.coverage`；缺 `mine/obstacle/target` 判据时**拒发覆盖** |
| `cli.py` | `detect` 增 `--slices/--coverage-mode/--corridor-m/--scan-id/--slice-incomplete/--omit-empty-classes/--coverage-report`；新子命令 `coverage`（纯几何预演，不读图） |
| `run_tests.py` | 注册 `perception.tests.test_coverage` |
| `tests/test_coverage.py`（新） | 14 项：几何、协议、负证据语义、e2e 7 片、不完整片、缺类拒发、CLI 全链路、确定性 |
| `README.md` / `CAPABILITY_L1.md` | 新增 ⑤b 节（语义/三条硬约束/CLI 例）；测试数 85→99；顺带修掉 README 里过期的 `K10(下)`（现役 `map.yaml` 是 `K9`） |

## 2. 7 片（= t4 §6.3 的 S3'，40 航点）实测

```
$ python -m perception.cli coverage --slices s3
片清单：mode=strip  corridor=0.525 m  7 片  并集 154/154 格
   S1 1/7   15 格  complete=True   N8>M8>L8>K8
   S2 2/7   24 格  complete=True   K7>K6>K5>K4…
   S3 3/7   12 格  complete=True   L2>M2
   S4 4/7   42 格  complete=True   B2>D2>F2>H2…
   S5 5/7   42 格  complete=True   N5>L5>J5>H5…
   S6 6/7   42 格  complete=True   B8>D8>F8>H8…
   S7 7/7   28 格  complete=True   N11>L11>J11>H11…
```

* 并集 **154/154**（合成场景验收口径）；`t1` 预设（现役 U 形 3 片）并集 **39/154**，
  与 t4 §8.3 的 `union cells 39 of 154` 独立复算一致。
* **coverage 只由航点几何决定**：`cells_of_route` = 格心到航段折线距离 ≤ `corridor_m`，
  与检测结果完全无关（否则"没检到"会被自动解释成"扫过且空"）。
* ⚠️ `corridor_m=0.525`（= 1.5 格）是**照 4 条横带的行距反推**出来的，**不是相机实测视场**；
  现场必须按实测视场收紧后再发车（已知限制，见 §5）。

## 3. 合成场景 e2e（8 雷 / 8 障 / 2 靶，跨 7 片）

`tests/test_coverage.py::test_pipeline_seven_slices_full_element_report`（真跑管线）：

- 报文类型：`heartbeat` + `field_verify`×7 + `target_zone`×2（靶区跨 3 片，逐片只报本片内的）
- 7 条 `field_verify` 的 `slice.id` = S1..S7、`of=7`、`coverage.complete` 全 `true`、
  `scan_id` 全相同、`coverage.mode="strip"`
- 跨片并集：雷 **8/8**、障 **8/8**、靶 **2/2**；格并集 **154**
- 每条报文 `validate_payload(m, GridSpec()) == []`；车侧镜像不含 `missing_fields`/未知类型
- S1 只报本片内的 `L8`/`M8`（别片的雷不会被算进来）；
  `field_verify_absent(S1)` = 15 格减 2 = 13 个 ABSENT

## 4. 负证据语义（本卡核心）

| 性质 | 证据 |
|---|---|
| `complete:true` ⇒ 片内未报 = ABSENT | `coverage_absent_cells({"cells":["A1","A2","A3"],"complete":True}, ["A1"]) == ["A2","A3"]` |
| `complete:false` ⇒ UNKNOWN，**不产生 ABSENT** | 同函数返回 `None`；`field_verify_absent` 返回 `None`；`diagnostics.coverage.absent_per_slice[S4] is None` |
| 缺 `complete` 键 ⇒ 不许猜 | 抛 `ValueError`（分不清"空的"与"没扫"） |
| 旧报文（无 coverage）没有负证据语义 | `field_verify_absent(...)` 抛 `ValueError` |
| 空片可表达 | `build_field_verify(mines=[], obstacles=[], coverage=..., slice=..., scan_id=...)` 合法；旧的 `build_field_verify()` 无参仍抛错 |

CLI 实测（`--slice-incomplete S4`）：7 条 `field_verify` 里 S4 的
`coverage.complete=false`，其余 true；S4 依旧报它**看得见**的雷/障（不因为不完整就不报）。

## 5. 已知限制（必须与队长/车侧同步）

1. **视场未实测**：`corridor_m` 默认 1.5 格是几何反推；真实相机视场要用实拍帧标定后收紧
   （否则会把"格边缘没看全"算成已判读）。单帧全场模式（`--slices full`）可绕开这个问题。
2. **旧车侧会把空表读成"全部消失"**：`drone.py:307-326` 只认 `mines/obstacles`，
   空表 ⇒ 一条 `仅本地` 差异（`EV_FIELD_VERIFY` 目前**零订阅**，只进遥测）。
   过渡期用 `--omit-empty-classes`（代价：旧车侧记 `missing_fields`）。
   **新语义必须由 t5b 台账按 `complete` 消费**，不能继续用 `_diff` 当判据。
3. **`targets` 仍走独立 `target_zone`**：决策层方案 §4.2 说"可合并进 `field_verify`"，
   但标注为待队长裁 R3 ⇒ 本卡未合并（不越权改契约）。
4. **D1（现场调色板标定）不在本卡**：`FIELD_DEFAULT` 缺 `mine/obstacle/target` 判据时
   `process(slices=...)` **直接抛错**（防"检不了"被当"空的"）；`--provisional` 可跑，
   但 `diagnostics.coverage.warnings` 会显式警告 ABSENT 只是"按占位色没看到"。
5. **未验真机**：本卡只做报文结构 + 合成场景；DDS 实发（`--publish`）、ROS 域号、ack、
   车侧台账端到端均**未执行**（无车/无无人机，按纪律不伪造）。

## 6. 只读纪律与仓库状态

- 本次运行 `git status --porcelain` 里属于本卡的只有
  `dzsmartscout/tools/perception/**`（`coverage.py`、`tests/test_coverage.py` 新增；
  `protocol.py`/`pipeline.py`/`cli.py`/`run_tests.py`/`__init__.py`/README/CAPABILITY_L1 修改）。
- `dzrobot_v2/**` **零写入**（只读过 `tasks/drone.py` 与 `docs/黑图探索与动态规划_决策层方案.md`）。
- 未 commit / 未 push / 未部署；测试产物只落在 `$TEMP`，不在仓库。
