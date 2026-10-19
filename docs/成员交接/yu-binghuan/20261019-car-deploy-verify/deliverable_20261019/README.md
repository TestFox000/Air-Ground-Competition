# 车端连接 · 补丁部署核验 · 昨日日志复盘（2026-10-19）

执行人：yu-binghuan ｜ 车端：duzhong@192.168.0.197（RK3588, Ubuntu 22.04, ROS2 Humble）｜ 核验窗口：2026-10-19 18:17~18:48

---

## 0. 结论速览

1. **车已连上（自动发现，未人工提供 IP）**：车 WiFi MAC `28:2d:06:23:d0:6e`（主机名 duzhong）经 MiFi LTE 路由的设备管理 API 自动定位到 `192.168.0.197`。WiFi 链路弱且抖（RTT 49~410ms，18:41~18:46 存在 5 分钟全断盲窗），已用重试+单连接批量化对抗。
2. **P0-1「提前量进控制环」补丁已在车端编译并运行**（另一位操作者/会话于 18:23~18:32 部署，本核验确认四位一体一致：源码=补丁、二进制含新参数、进程在编译后启动、配置带护栏）。源码与本机 `_cloud/patch` 逐行零差异（fc 验证；sha 差异仅为换行符）。
3. **P0-2 参数对齐已落地**：`max_shots 10`、两键开、`lead_apply_gain 1.0`、`fire_dz 18/26`、`pid 10/0.5/2.0`、`deadzone 3`、`predict_gain 0.2`、`cooldown 800`、`confirm 4`；det_node 的 imshow 已关（`draw:false`，P1-3 第一项）。
4. **昨日日志复盘核心结论**：
   - t42 实战轮（10-18 12:42）F9 停站：目标反复可见但 25 次「类别未确认/类别不允许」诊断——旧码类别锁清零 bug 叠加红靶+fire_only_blue 门控，是 F9 零发的直接原因（与方案 §3.3 一致）。
   - 两轮 10 发全打出（1138 与 kb11）均为**静止靶、云台停稳（v≈0, realV=0）**后出膛，**0 命中**。
   - ⇒ 锁定与判据链路修好后，**静止靶 10 发 0 中指向系统性指向偏差（激光-相机不同轴，laser_offset=0/0 假设不成立）**——P1-1 laser_offset 标定是当前提升命中率的第一优先级。
5. **缺口**：当前运行进程已打满 10 发（进程级弹药累计），下轮测试前必须重启 5_gimbal；gyro_ff 符号未标（增益保持 0 正确）；gitee main/preserve 仍是带 bug 快照（P0-3 未收口）。

---

## 1. 连接通道（可复用）

| 项 | 方法 | 结果 |
|---|---|---|
| 车的 IP 发现 | MiFi 管理 API `http://192.168.0.1/xml_action.cgi?method=get&module=duster&file=json_device_management_info<ts>`（无需登录） | 车实时在线状态与 IP（`28:2d:06:23:d0:6e` → 192.168.0.197） |
| SSH 通道 | `car.cjs`（Node ssh2 密码登录，非交互） | 单命令 / `run <脚本>` / `get`/`put` 传输 / `batch` 四模式，3 次重试 + 40s 握手超时 + keepalive |
| 弱链路对策 | 全部操作合并为每阶段 1 次连接；传输用 tar.gz 单文件 | 210KB~68KB 传输在重试下稳定成功 |
| 免密建议 | 本机已有 `id_ed25519`，建议下次接触车时 `ssh-copy-id` 装公钥 | 待办（可选） |

`car.cjs` 用法（工作目录 `赛事临时/v1/.ssh2-tmp/`）：
```
node car.cjs "命令"                      # 远程执行
node car.cjs run <本地脚本>               # 上传脚本到 /tmp 执行（推荐，无引号问题）
node car.cjs get <远程> <本地>             # 下载
node car.cjs put <本地> <远程>             # 上传
# 环境变量：CAR_HOST/CAR_USER/CAR_PW/CAR_RETRIES/CAR_TIMEOUT_MS
```

---

## 2. 部署核验证据链（2026-10-19 车端实测）

时间线（全部为车端时钟）：

```
17:46        车上电（uptime 起点）
18:23:41     dzrknn_dataprocess.h 定稿   sha256 ebccfc51…  == _cloud/patch/.h
18:25:56     config/robot.yaml 写入 P0-2 + 护栏参数   sha256 057f2db9…
18:31:55     dzrknn_dataprocess.cpp 定稿 sha256 9ea10779…（fc 与 _cloud/patch 逐行零差异，仅换行符）
18:32:42     colcon 编译产出 install 二进制（含 lead_timeout_ms / max_lead_apply_px 字符串 ⇒ 补丁版）
18:32:54     dzactuator_node 进程重启（pid 13492，exe 无 deleted 标记 ⇒ 运行新二进制）
18:36~18:39  打靶测试：pub_LaserShot [1..10/10] dz=20 v=0/0 realV=0/0（静止靶停稳出膛，10 发）
```

运行横幅（18:32 版进程）逐项核验：

```
射击: 动态开火框 18~26px | 保持 5 帧 | 预判 0.20 | 全程弹药 10 发/冷却 800ms | 扫描确认 4 帧
跟踪控制: 增量式PID kp=10.00 ki=0.500 kd=2.000 | 速率上限 2000 tick/s
开火权限(★补丁): 两键开火 已启用 | 提前量施加增益 1.00 (输入 /aim/lead_x)
凝视稳定: 车体转动前馈 已关闭 (增益 0.00)   ← 待 P1-2 标符号后开 0.8
```

配置护栏（`config/robot.yaml` 223-231 行）：

```
lead_apply_gain: 1.0    lead_slew_px_per_frame: 4    max_lead_apply_px: 60.0    lead_timeout_ms: 300
```

副本血统哈希对照（防「从旧快照恢复」事故）：

| 哈希(前16) | 版本 | 位置 |
|---|---|---|
| `4bf068150d66c08c` | 旧版（类别锁会清零；11:05 车端快照） | gitee preserve/car、本机 赛事临时/v1/dzrobot_v2 |
| `b37c4af99057f7e6` | 中间版（类别锁修复，无 P0-1） | gitee agents/dzhong |
| `1e0afbbac2346f75` | P0-1 补丁版 | `_cloud/patch/dzrknn_dataprocess.cpp` |
| `9ea1077974a343bb` | P0-1 补丁版（换行符差异，内容同上） | **车端现役源码** |
| `cf051d453372a4ef` | 车端 install 二进制（18:32:42 编译） | 车端 install/…/dzactuator_node |

⚠️ **P0-3 未收口**：gitee `main` / `preserve/car` 仍为 `4bf06815` 旧版。任何人从这两处恢复车端都会重现「永不开火」。本档案已保存部署真源三件套（cpp/.h/robot.yaml）；建议 `agents/dzhong` 合回 `main` 并在 `preserve/car` README 标注已知 bug（结构决策需队长裁决，未单方面执行）。

---

## 3. 昨日日志复盘（数据全部来自车端原始日志）

### 3.1 各轮日志的真相

| 日志 | 时间 | 内容 | 结论 |
|---|---|---|---|
| bringup_all_113833 | 10-18 11:38 | kp12/1.1/1.8、两键关、lead 0；10 发全打出，v≈0/realV=0 | 静止靶、停稳出膛；0 中 |
| bringup_t42_124230 | 10-18 12:42 | 实战路线 …F9*…；25 次「类别未确认/不允许」诊断 | F9 主因=类别锁清零 bug + 红靶 fire_only_blue 门控 |
| kb11_5_gimbal | 10-18 19:27 | 类别锁修复版；`类别已锁定 locked_cls=1` 后 10 发全打出 | 锁修复有效；仍 0 中 |
| 5_gimbal.log | 10-18 20:23 | kp2.5/0.35/0、两键关、lead 0.5、**车体前馈 0.8 开着** | 看到红靶被门控，5.9s 丢失，0 发；该轮 7_path 仅 10.5s 烟测 |
| lock_gate.log | 10-18 19:03~22:14 | 浸泡 65833 acc / 984 rej（1.5%） | 锁门浸泡稳定性可接受 |
| gimtest_* | 10-19 18:29/18:36 | 新栈烟测 + 打靶 10 发 | 补丁栈上线正常 |

### 3.2 日志字段解码（防误读）

`pub_LaserShot_Command [n/10] dz=X v=a/b realV=c/d` 中：
- **dz = 当时动态开火框回显**（`static_cast<int>(fire_dz)`），不是实际误差；
- v = 目标像速估计（px/s 级），realV = 云台电机实测转速。
- 历史轮 dz=14/19/25 的差异只是各轮开火框配置不同，**不代表误差分布**。

### 3.3 为什么「打得出、打不中」

静止靶在 `v≈0, realV=0`（判据全部通过）下出膛仍 0 中，且两轮 10 发均如此 ⇒ 误差是**系统性偏置**而非随机散布：最可能为激光与光轴不同轴（`laser_offset_x/y=0/0` 假设不成立）。按方案 §P1-1：标定后单发命中率 0.3→0.9，10 发预算下这是唯一决定性因素。

**注意耦合**：P0-1 上线后控制环收敛点为 `dx + lead = laser_offset`；一旦 offset 标成非零，**必须**同步在 dzbrain 的 `aim.py` pred 中减去 offset（约 5 行，方案推荐做法），否则两把钥匙口径打架、又回到不开火。

### 3.4 dx 分布数据缺口（如实说明）

云台节点按事件打日志，不落每帧 dx；昨实战轮的 dzbrain「框外/闭锁」统计所在的 7_path.log 已被 20:23 烟测覆盖。**下一轮实战建议**：在车端后台跑 `log_aim.py`（30Hz 订阅 /offset_center 落 CSV，已有现成探针脚本），即可拿到完整 dx 序列用于定 fire_dz 与滞后分析。

---

## 4. 剩余优化清单（按优先级）

| # | 事项 | 状态/动作 | 需要谁 |
|---|---|---|---|
| 1 | 下轮测试前重启 5_gimbal | 当前进程已打满 10 发（进程级累计，重启即复位） | 任何操作者 |
| 2 | P1-1 laser_offset 标定 | `laser_offset_calib.py`（成员分支已带）；标非零后**同步 dbrain pred 减 offset** | 现场（激光常亮+截图点光点） |
| 3 | P1-2 gyro_ff 符号 | `gimbal_ff_calib.py` 车体左右各转 30°+，标完开 0.8 | 现场 |
| 4 | fire_dz_far 18→14 | P0-1 生效后稳态 aim→0，可收回 14 提精度 | 配置秒改 |
| 5 | P1-3 延迟测量 | det_node 输出带图像时间戳后再动线程数；imshow 已关 | 代码 |
| 6 | P2 可观测性 | 横幅仍打不生效的 Kp20/Kd15；开火日志加 dx/lead/gate/ready；reset_ammo 服务；healthcheck 误报修复 | 代码 |
| 7 | P0-3 仓库收口 | agents/dzhong→main 合并 + preserve/car README 标注 | 队长决策 |

## 5. 本档案文件清单

- `README.md`：本文
- `dzrknn_dataprocess.cpp` / `dzrknn_dataprocess.h` / `robot.yaml`：**车端部署真源三件套**（18:31/18:23/18:25 定稿版，哈希见 §2）
- `gimtest_5_gimbal.log`：补丁栈首轮回调日志（含 10 发测试）
- `kb11_5_gimbal.log`：类别锁修复验证轮
- `5_gimbal.log` / `7_path.log`：10-18 晚烟测与 20:23 短跑（10.5s，标注为烟测）
- `car.cjs`：车端 SSH 自动化工具（用法见 §1）
