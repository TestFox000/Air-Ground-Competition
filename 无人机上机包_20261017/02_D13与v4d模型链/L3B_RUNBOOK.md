# L3b 协议闭环 Runbook（v4d 格号引擎）· 2026-10-17

## 0. 现状速览（本次盘点结论）

| 项 | 状态 | 位置 |
|---|---|---|
| v4d 训练 | **已完成**（云端 27379 实例，80ep，`runs/cell_v4d/best.pt`） | 实例已释放 |
| v4d ONNX | **已导出、已在本地**（4.3MB，2026-10-15 20:29） | `C:\Users\29212\Documents\Trae-CN\default\cloud_train_20261015\deploy_pkg\cell_classifier_v4d_20261015.onnx` |
| v4d 上板 | deploy_now.py 曾把 4 个 ONNX 推到板端 `~/dzuav/models/` 并把现役 cell 引擎换成 v4d（带动态形状）| 上传是否成功**未确认**，需 P0 检查 |
| v4d 独立引擎 `engines_d13/cell_classifier_v4d_fp16.engine` | **未构建**（正是 l3b_with_v4d.py 步骤 1 要做的） | 板端 `~/dzuav/engines_d13/` |
| D13 hbb 引擎 + L3a | 已通过（17 通道契约真机验证 landmarks=24） | 板端 `~/dzuav/engines/hbb_detector_d13_fp16.engine` |
| 云端 4090D(16376)/旧 27379/台式机 169.254.9.146/板端 192.168.55.1 | **本次 TCP 探测全部不可达**（预期状态：实例关机、板端未开机/不在网） | — |

> 结论：走 **情形 A（主路径）** 的概率最大——v4d 已训好，只差板端转换 + L3b。

## 1. L3b 闭环判据（orin_build_verify.sh）

- **L3a**：`dzuav` 二进制用 [hbb 引擎 + cell 引擎] 跑一张测试图，stdout 出现 `localized=1`（PnP 定位要求 **≥4 个清晰格号**被 cell 分类器正确识别）。
- **L3b**：后台监听 UDP:50000，用**车端同一解析器** `ag_integrate.intel_protocol.parse` 做 magic/version/CRC16/后缀全量校验，收到合法帧打印 `UDP_OK`。
- **通过标志**：脚本输出 `L3 全链路通过：图像 -> TRT 推理 -> PnP 定位 -> UDP 广播 -> 车端 parse`。
- **PARTIAL**：仅 `L3a 通过`（localized=0，未发 UDP 属预期）——说明格号识别不足 4 个，**这正是 v4d 要解决的瓶颈**（v4c 格号值识别率仅 89%）。

## 2. l3b_with_v4d.py 剖析（四步）

| 步 | 动作 | 依赖/前置 |
|---|---|---|
| 1 kick | 板端无 `~/dzuav/engines_d13/cell_classifier_v4d_fp16.engine` 则 nohup `trtexec --onnx=$HOME/dzuav/models/cell_classifier_v4d_20261015.onnx --saveEngine=$HOME/dzuav/engines_d13/cell_classifier_v4d_fp16.engine --fp16`（幂等，已有则 ALREADY） | **ONNX 必须已在板端** `~/dzuav/models/` |
| 2 poll | 轮询 `~/v4d_engine.log` 的 BUILD_DONE/BUILD_FAIL（15s×40 ≈ 最长 10 分钟） | trtexec 在 `/usr/src/tensorrt/bin/trtexec` |
| 3 swap | `rm ~/dzuav/engines_d13/cell_classifier_fp16.engine`（旧名引擎）| 关键：否则 find `*cell*` 按字典序先命中旧名（f<v），v4d 不生效 |
| 4 L3 | nohup `orin_build_verify.sh --no-mirror --run engines_d13 ~/dzuav_deploy/air_ground/l3_field_88cells.jpg`，轮询 `~/l3_run.log` 尾部 `EXIT=` | 测试图必须在板端；`engines_d13` 需同时有 d13 hbb 引擎 |

公共依赖：板端 SSH `192.168.55.1` 用户/密码 `duzhong`（脚本内置，无需环境变量）；本机 `tools/remote_run.py`（已确认在位）+ paramiko（已确认 5.0.0）。每阶段自带 phase 重试（SSH 断连容忍）。

## 3. P0 前置检查（板端开机联网后，逐条照抄）

```powershell
# 0) 板端可达性（TCP 探活，不登录）
$c=New-Object Net.Sockets.TcpClient; $r=$c.BeginConnect('192.168.55.1',22,$null,$null); $r.AsyncWaitHandle.WaitOne(5000) -and $c.Connected
```
预期 `True`。`False` → 检查无人机供电/热点（SSID 见 05 手册）、笔记本是否连上无人机热点。

```powershell
# 1) 板端文件在位检查（一次看全：v4d ONNX / engines_d13 / 测试图 / trtexec）
$env:AG_CAR_PASS='duzhong'
python C:\Users\29212\Documents\kimi\tasks\2026-10-14\18-59-27-5bee8702\air_ground\tools\remote_run.py --host 192.168.55.1 --user duzhong --timeout 60 --cmd "ls -la ~/dzuav/models/cell_classifier_v4d_20261015.onnx 2>/dev/null || echo V4D_ONNX_MISSING; ls -la ~/dzuav/engines_d13/ 2>/dev/null || echo EDIR_MISSING; ls -la ~/dzuav_deploy/air_ground/l3_field_88cells.jpg 2>/dev/null || echo IMG_MISSING; ls /usr/src/tensorrt/bin/trtexec 2>/dev/null || echo TRTEXEC_MISSING"
```
预期：`cell_classifier_v4d_20261015.onnx`（约 4.3MB）、`engines_d13/` 内含 `hbb_detector_d13_fp16.engine`、`l3_field_88cells.jpg`（约 1.2MB）、trtexec 存在。

缺啥补啥（均为幂等，只补不删）：
```powershell
# 1a) 缺 v4d ONNX → 从本地 deploy_pkg 补推
python C:\Users\29212\Documents\kimi\tasks\2026-10-14\18-59-27-5bee8702\air_ground\tools\remote_run.py --host 192.168.55.1 --user duzhong --timeout 300 --put "C:\Users\29212\Documents\Trae-CN\default\cloud_train_20261015\deploy_pkg\cell_classifier_v4d_20261015.onnx" --dest "~/dzuav/models/cell_classifier_v4d_20261015.onnx"
# 1b) 缺测试图 → 跑幂等推图脚本（本地 frame_00124.jpg 已确认在位）
python C:\Users\29212\Documents\Trae-CN\default\dk_train_pack_20261016\l3b_field_frame.py
# 1c) 缺 d13 hbb 引擎 → 重跑 L3a 组装+验证（verify_d13_l3.py 会重建 engines_d13 硬链接）
python C:\Users\29212\Documents\Trae-CN\default\dk_train_pack_20261016\verify_d13_l3.py
```

## 4. 三种情形

### 情形 A：v4d 已训好待转换（主路径，本地 ONNX 在位即可）

前置：P0 全绿（板端可达 + ONNX 在板端 `~/dzuav/models/`）。

```powershell
cd C:\Users\29212\Documents\Trae-CN\default\dk_train_pack_20261016
python l3b_with_v4d.py
```
预期时序：`[1] KICKED`（或 ALREADY）→ `[2] 转换完成: -rwx… cell_classifier_v4d_fp16.engine`（约 2~10 分钟）→ `[3] engines_d13 切到 v4d`（ls 输出含 v4d 引擎、无旧 cell_classifier_fp16.engine）→ `[4] L3 运行中…` 数轮后打印 l3_run.log 尾 25 行 → **`[done] L3 全链通过（v4d 格号引擎生效）`**。

### 情形 B：v4d 未训练需先训（仅当本地/板端 ONNX 均丢失或指标不合格时）

前置：云端 AutoDL 实例开机（最新 4090D：`connect.bjb1.seetacloud.com:16376`，root，密钥 `BXLvIZ4hZ9Gm`，数据盘 `/root/autodl-tmp/`）。本次探测 16376 不可达——先在 AutoDL 控制台开机，若实例被释放则新开实例（地址/密钥会变，同步改脚本头）。

训练链路（照 `cloud_train_20261015\deploy_v4d.py` 的云端管线，命令线索）：
1. 上传：`air_ground/reference/onboard/dzuav/src/cell_classifier/*.py` + `cloud_train_20261015\v4d\autolabel_v4d_cloud.py` + `air_ground/tools/ml/build_cell_v4d.py` + `air_ground/tools/ml/gen_cell_synth.py` → `/root/autodl-tmp/cellv4d/`
2. 数据：`ln -sfn /root/autodl-tmp/ml_datasets/cell_labels cell_labels`（数据集需在新实例数据盘，缺则从本地重新上传）
3. 管线：`autolabel_v4d_cloud.py && build_cell_v4d.py --root . --aug-per 20 --aug-workers 24 && PYTHONPATH=. python -m cell_classifier.train --csv cell_labels/merged_v4d.csv --output runs/cell_v4d --epochs 80 --batch-size 256 --workers 12`
4. 导出+下载（照 `v4d_finish.py`）：`PYTHONPATH=. python -m cell_classifier.export_onnx --weights runs/cell_v4d/best.pt --output cell_classifier_v4d_20261015.onnx` → sftp 拉回 `deploy_pkg\`
5. 回到 P0 → 情形 A。

### 情形 C：v4d 就绪只需跑 l3b（板端引擎已建：`ls ~/dzuav/engines_d13/` 已见 v4d 引擎）

```powershell
cd C:\Users\29212\Documents\Trae-CN\default\dk_train_pack_20261016
python l3b_with_v4d.py
```
脚本幂等：kick=ALREADY 直接跳到轮询/swap/L3。若步骤 2 卡「构建中」直至超时（引擎在但 `~/v4d_engine.log` 无 BUILD_DONE）：
```powershell
python C:\Users\29212\Documents\kimi\tasks\2026-10-14\18-59-27-5bee8702\air_ground\tools\remote_run.py --host 192.168.55.1 --user duzhong --cmd "echo BUILD_DONE >> ~/v4d_engine.log"
```
再重跑脚本即可。

## 5. 失败分支处理

| 症状 | 原因 | 处理 |
|---|---|---|
| kick/poll 阶段反复 `SSHException` 后退出 2 | 板端 Wi-Fi 断连/热点掉线 | 脚本已重试 6~8 次；仍失败则检查热点后重跑（全流程幂等） |
| `[FAIL] trtexec 失败`（BUILD_FAIL） | kick 命令**未带动态形状**（v4d ONNX 输入为动态 batch） | 手动重建：`trtexec --onnx=$HOME/dzuav/models/cell_classifier_v4d_20261015.onnx --fp16 --minShapes=images:1x3x96x96 --optShapes=images:64x3x96x96 --maxShapes=images:256x3x96x96 --saveEngine=$HOME/dzuav/engines_d13/cell_classifier_v4d_fp16.engine`（先 `rm` 旧引擎与 `~/v4d_engine.log`），再回情形 C |
| L3a 推理阶段 shape/维度报错 | 同上（引擎 profile 不含运行期 batch） | 同上重建 |
| `[FAIL] 轮询超时`（步骤 2，引擎在、log 无 BUILD_DONE） | 引擎非本脚本所建 | 见情形 C 末尾的 `echo BUILD_DONE` 补丁 |
| `[FAIL] L3 轮询超时` | L3 后台仍在跑（L1 编译+引擎反序列化可超 4 分钟窗口），**≠失败** | 勿重跑脚本（会并发第二个 L3 实例、UDP 端口打架）；手动查：`--cmd "tail -25 ~/l3_run.log"`，看到 `EXIT=0` + `L3 全链路通过` 即成功 |
| `[PARTIAL] 仍 localized=0` | 格号识别 <4 个：图不够清晰 或 v4d 未生效 | ① 确认步骤 3 已执行（engines_d13 无旧 cell 引擎）；② 换更清晰实拍 88 格图重推（改 l3b_field_frame.py 的 IMG 路径）；③ 若仍 0，回情形 B 重训 |
| L3 输出「未找到 hbb/cell 引擎」 | engines_d13 缺 d13 hbb 引擎 | 跑 P0-1c（verify_d13_l3.py 重建硬链接）后重跑 |
| 云端 16376 不通 | 实例关机/释放 | AutoDL 控制台开机；地址密钥变了就改脚本头（本次盘点时全部不可达，属预期） |

## 6. 回滚

- 步骤 3 只删 `engines_d13/` 内的旧名 cell 硬链接，**不动现役** `~/dzuav/engines/`；
- 现役完整备份在板端 `~/dzuav/backup_20261015/`（models_orig + engines_orig），异常时一条 `cp -a` 回滚；
- A 方案（minecpv2）保持可回滚状态，赛前终审前勿销毁。
