# car_snapshot — 车端检测栈字节快照（AGC-21 / t10）

**目的**：把「车上到底跑的是哪几个字节」变成可复现证据，关闭 E1 的 U1（车端在用模型不在仓库 = 来源不明）。

- **采集时间**：2026-10-24 17:51 +08（scp 完成时刻），车 `duzhong@192.168.0.4` 在线
- **采集方式**：**只读**——车端仅执行 `sha256sum` / `stat` / `ls`，本地 `scp` 拉取；**未写车、未重启、未部署、未发车**
- **校验**：每个文件本地 SHA256 与车端 `sha256sum` 逐一相等（`MANIFEST.json: car_side_hash_verified=true`）
- **总体积**：29.78 MB（6 个文件）

## 清单

| 文件 | 字节 | mtime | SHA256 | 车上路径 |
|---|---|---|---|---|
| `dzrobot__det_node` | 776,112 | 10-24 09:06 | `1d0ae071732db7fa…` | `/home/duzhong/dzrobot/install/det_node/lib/det_node/det_node` |
| `dzrobot2__det_node` | 3,855,872 | 10-20 21:51 | `9b449ba21b6c629e…` | `/home/duzhong/dzrobot2/install/det_node/lib/det_node/det_node` |
| `dzrobot__light_det.rknn` | 8,125,813 | 10-23 19:12 | `8b17ec17bfed399d…` | `/home/duzhong/dzrobot/install/det_node/share/det_node/model/light_det.rknn` |
| `dzrobot__light_det.rknn.bak_20261023_1915` | 8,487,541 | 10-16 20:55 | `12a3b5487e0099f9…` | 同目录 `…light_det.rknn.bak_20261023_1915` |
| `dzrobot2__light_det.rknn` | 9,899,901 | 10-20 20:15 | `59aaad606a641d14…` | `/home/duzhong/dzrobot2/install/det_node/share/det_node/model/light_det.rknn` |
| `dzrobot__config_robot.yaml` | 76,923 | 10-23 21:29 | `49128a01ab95d9a1…` | `/home/duzhong/dzrobot/config/robot.yaml` |
| `dzrobot__config_map.yaml` | 356 | 10-18 10:25 | `b5629d0522b8b508…` | `/home/duzhong/dzrobot/config/map.yaml` |

完整 64 位哈希见 `MANIFEST.sha256`（`sha256sum -c` 兼容）与 `MANIFEST.json`。

### 地图对照：车端 ≠ 仓内（U2 实证，2026-10-24 补）

| 位置 | SHA256 | 大小 | 图 | 关键要素 |
|---|---|---|---|---|
| **车端** `dzrobot/config/map.yaml` | `b5629d05…` | 356 B · 10-18 10:25 | **第七张图** | mines H8 G9 A2 B1 C3 E3 **F5** I2 · obstacles D5 I6 B7 F8 C9 E10 G10 A11 · targets **D7 E7** · bridges **K10 K4** |
| 仓内 `dzrobot_v2/config/map.yaml` | `385e0718…` | 2,370 B · 10-17 22:04 | 第六张图 | mines A2 F4 H4 G5 E8 B10 D11 I11 · obstacles B1 D1 F3 I6 H7 A8 C9 E9 · targets **D5 D6** · bridges **K5 K7** |

⇒ **仓内地图不能用来判车端路线**（靶不同、桥不同、雷不同）。M1 路线校验必须以车端这份 356 B 的地图为准；这也是 U2「现场配置未入库」的字节级证据。

## 三套栈 ↔ 三个模型（别搞混）

| 栈 | 二进制 | 模型 | 状态 |
|---|---|---|---|
| **实跑（3 字段）** | `dzrobot2__det_node` = 9b449ba2 | `dzrobot2__light_det.rknn` = **59aaad60** | 2026-10-20 起实际运行（prep-processes.log pid 602768） |
| **8 字段（已构建未启用）** | `dzrobot__det_node` = 1d0ae071 | `dzrobot__light_det.rknn` = **8b17ec17** | 10-24 09:06 构建；10-24 09:06–10:39 真跑过 1895/1897 帧 |
| 回滚目标 | — | `.bak_20261023_1915` = 12a3b548 | 10-16 版本，10-23 19:15 换装时留的备份 |

### 同尺寸兄弟文件（未归档，仅登记哈希）

`/home/duzhong/dzrobot2/install/det_node/share/det_node/model/light_det2.rknn` = **5698ce0e**（9,899,901 B，与 `light_det.rknn` **同尺寸但不同字节**）。
按尺寸排查时极易把两者当成同一份。实跑加载的是 `light_det.rknn`（59aaad60），**不是** `light_det2.rknn`。

## 模型 8b17ec17 的来源链（U1 关闭依据）

1. **2026-10-23 19:12:35** 产出 `car_light_det_20261023_1912.rknn`（8,125,813 B，sha 8b17ec17）→ `_tmp_session/VISION-MODEL-20261023/models/`
2. **2026-10-23 19:15** 换装到车 `dzrobot/install/det_node/share/det_node/model/light_det.rknn`，旧模型留为 `.bak_20261023_1915`
3. 仓库内**两份同哈希副本**（生成脚本实时校验，均 EQUAL）：
   - `_tmp_session/VISION-MODEL-20261023/models/car_light_det_20261023_1912.rknn`
   - `_tmp_session/LEAD-VISION-20261023/target_pose_v2_20261016.rknn`（同哈希不同名）
4. `.bak_20261023_1915` = 12a3b548 = 仓库 **tracked** 文件 `dzrobot_v2/models/light_det.v2.3.2.rknn`（8,487,541 B）

⇒ 该模型**字节已在 artifacts 长期留存**，不再依赖 `_tmp_session` 存活；「不在仓库」只剩「未纳入 version control」一层含义，不是未知。

## 复跑（纯只读，可审计）

```bash
cd artifacts/agc-1024/car_snapshot && sha256sum -c MANIFEST.sha256
# 重新采集 + 与车端/仓库三方对哈希（需车在线）：
python -B -X utf8 _tmp_session/agc-1024/e1/gen_snapshot_manifest.py
```

## 给 Lead 的两条处置建议（本卡不执行）

1. **体积**：29.78 MB。`artifacts/` **未**被 `.gitignore` 忽略（`git ls-files artifacts` 现有 5 个文件），若入库会显著增大仓库。三种选择：① 只提交 `MANIFEST.*` + 本 README，模型字节留本地 artifacts（现状）；② 把 `*.rknn` 加 `.gitignore` 但提交清单；③ 用 LFS。**需 Lead 裁决**。
2. **登记现役模型**：8b17ec17 是 8 字段栈的现役检测模型，建议 M2 前登记进 `dzrobot_v2/models/`（tracked）。这是一次仓库变更，**本卡未做**，等 Lead 排期。
