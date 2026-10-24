---
来源: 交接_20261024 §6；教员三件套实测；用户 2026-10-24 接手指令
取证: 档（机端同编码器样本）+ 只读源码 + 离线单测
更新: 2026-10-24
status: in_progress
ns: agc-1024
agent: dl-drone
树节点: AGC-3
写域: 本卡；`dzsmartscout/tools/link/dzudp_adapter.py`；`dzsmartscout/tools/link/dzudp_core.py`；`dzsmartscout/tools/link/tests/test_dzudp_*.py`（AGC-17 合入窗口，2026-10-24 Lead 开；禁改 `dzlink_supervisor.py`／车端任何文件／`dzbrain_bridge`）
摘要: 复用教员 udp_bridge.py 解析核，做二进制 50000 到既有 JSON 情报入口的薄适配，13 类映射与坏帧拒收可单测，0x4003 只记录不写 waypoint。
结论(2026-10-24 17:5x · 合入后)：生产件已入库（适配器／解析核钉死快照／provenance／单写者三件），link 套件 105→**195** 全绿、dl1 51 全绿、verify 0、supervisor selftest 通过；候选↔生产 10 组构造帧＋3 类坏帧＋两份真帧**逐项一致（PARITY_FAILURES=0）**。13 类/坏帧/字段保真/路线只记录结论不变；真实感知与实飞仍卡在 DL4 数据面（雷达 carrier=0、真实相机样本缺失、FAST-LIO/PX4 未验）。工作区**未提交**，等 Lead 放行 add+双推。
---

# AGC-DL1-ADAPTER-20261024 · 二进制:50000→既有 JSON 情报链薄适配层

owner: dl-drone；reviewer: e-integ（只读评审）。

## 1 依据与复用（五必填）

- 读过的台账：D-039（情报契约三态 source）；现行推进计划 §5 DL1/DL2。
- 复用检索：教员交接 `udp_bridge.py` 的 crc16/parse_* 解析核、现有 supervisor/契约、`dzlink_supervisor.py selftest`、`dzsmartscout/tools/link/tests`；**不复制协议、不启 udp_bridge 的路线发布开关**。
- 新增轮子：薄适配层（唯一理由：二进制:50000 与既有 JSON 入口之间缺接缝，仓内无第二实现）。
- 验收命令：`python -B -X utf8 -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py"` + 新增适配器单测。
- 回滚点：候选只在过程目录，未改生产即回滚点。

## 2 当前问题

二进制:50000 到既有 JSON 情报链缺薄适配层（已证）。机端 13 类 ≠ 仓内旧 7 类；坐标反解用机端帧常数 0.065/0.295/0.35；二进制不含 coverage/scan_id/完整布局。352B/13对象与 57B/4航点是**同编码器工具样本，不得冒充视觉实测**。

## 3 唯一假设与验收

- 假设：用现有解析核 + 一次字段映射即可接入既有入口，无需改协议或车端契约。
- 必测：13 类映射完备；坐标往返（机端帧常数）；未知类拒收；坏 CRC/截断拒绝；缺失字段保持缺失（**不补空 coverage、不伪造 field_layout**）；0x4003 只记录、不向 `/nav/waypoint` 发布。
- 边界：只做地面离线单测；不接真实相机、不改车端、不启飞行。

## 4 交付

1. 适配器候选 + 单测结果（命令+退出码）。
2. 字段保真表（二进制有/无 → JSON 有/缺）。
3. 合入建议（若需改 `dzsmartscout/tools/link/` 生产文件，先报 Lead 加锁）。
4. DL4 数据面阻塞清单（雷达 carrier=0、真实相机样本缺失等），只列不修。

## 5 执行记录（2026-10-24 17:2x · dl-drone）

候选件**全部只在过程目录**，未动生产文件：
`_tmp_session/agc-1024/dl1/agc_binary_adapter.py`（适配器）、`tests/test_agc_binary_adapter.py`（51 条单测）、
`verify_dl1.py`（独立复算，不依赖被测断言）、`tests_out.txt` / `verify_out.txt`（本次输出留证）。

### 5-1 复跑命令与退出码

| # | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 1 | `python -B -X utf8 -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py"` | **0** | Ran **105** tests · **OK**（生产链路基线未被改动） |
| 2 | `python -B -X utf8 -m unittest discover -s _tmp_session/agc-1024/dl1/tests -p "test_*.py"` | **0** | Ran **51** tests · **OK**（0 skip / 0 fail） |
| 3 | `python -B -X utf8 _tmp_session/agc-1024/dl1/verify_dl1.py` | **0** | 独立复算 7 组，逐项与适配器一致（下文数字取自它） |

### 5-2 复用是否真复用（第 3 条命令 §1）

`sha256[:16]=9ff259a36ce40f70`：适配器加载的解析核与 `D:/User/Desktop/教员交接/udp_bridge.py`
**逐字节相同**（用 `importlib` 从仓内快照 `_tmp_session/sqc-20261024/tools/udp_bridge.py` 载入，
不复制函数体）。加载时断言 `PUBLISH_ENABLED is False`，否则直接抛错；`sniff`/`sniff_loop`/`bridge`
一个都不调用（单测把这三个符号替换成炸弹，路线帧仍正常返回）。

### 5-3 13 类映射完备（三条独立证据）

1. 适配器 13 项名字 == 教员 `TYPE_NAMES` **逐项相同**（`names identical: True`）；
2. 适配器 13 项名字 == 机端部署 `intel_protocol.ObjectType` 逐项相同（单测实测，非 skip）；
3. 机端真帧 352B/13 对象解出的对象名与线上字节一致（下 §5-4）。

角色分配（唯一映射表 `CLASS_TABLE`）：0 PAPER→`paper`（**K 列=桥洞通行、其余格=雷**，与机端
`world_model.py:120-127` 同判据）；3/4 TARGET_RED/BLUE→`target`；5 OBSTACLE→`obstacle`；
1/2/6/7/8/9/10/11/12（格号/车/三色桶/四起终点）→`record_only`（JSON 无落点，只进 records）。
**与仓内旧 7 类的差别是致命的**：`dzsmartscout/.../include/dzuav/types.hpp:11-20` 里 4=CellLabel、5=TargetBlue，
按旧表解 13 类帧会把**蓝靶当格号（不开火）**、**障碍当蓝靶（对着桶开枪）**，7~12 直接抛错整帧丢。

### 5-4 真帧逐对象独立复算（352B 机端同编码器样本）

13 个对象全部落在残差 0.000 的格心上，独立算术与适配器**逐个一致**：

```
#00 PAPER A1 · #01 CELL_LABEL B2 · #02 CAR C3 · #03 TARGET_RED D4 · #04 TARGET_BLUE E5
#05 OBSTACLE F6 · #06~08 BUCKET_RED/GREEN/CYAN G7/H8/I9
#09 CAR_START J10 · #10 CAR_END K11 · #11 UAV_START L1 · #12 UAV_END M2
→ target_zone{"blocks":["D4","E5"]} + field_verify{"mines":["A1"],"obstacles":["F6"]}，rejected=[]
```

★ 这台样本里 `CAR_END` 落在 **K11**，但它是`CAR_END`不是`PAPER`，所以**不许**冒充桥洞
（`records.bridge_passable={"lower":[],"upper":[]}`）——K 列分流的判据是"类型=纸片"，不是"列=K"。

坐标：154 格往返失败 **0**；机端 A1 用车端 `geometry` 公式反解得到 **D11**（整场错位且不报错），
固有单测把这条钉成"两套帧不可互换"的门。

### 5-5 坏帧拒收（就地构造，全部 kind=bad、messages=0）

| 坏帧 | 解析核报错 |
|---|---|
| CRC 翻一个 body 字节 | `crc mismatch` |
| 截断 −1B | `size 351 != expected for count=13` |
| declen 谎报（偏移 15 写成 27） | `len 27 != 2+25*13` |
| 未知类 99（也测 13/14/200） | `unknown type 99` |
| 后缀非 `fefefefe` | `bad suffix` |
| 未知 msg_id 0x4004 / 非协议包 / 非 bytes / 空包 | 逐条 kind=bad |

坏帧**不污染计数**（`bad+1`，`intel` 不变），也**不上抛异常**（坏消息绝不杀回调）。

### 5-6 0x4003 只记录（三条一起钉）

① 适配器源码里**不出现** `/nav/waypoint` 这个串（连名字都没有 ⇒ 结构上无发布路径）；
② 真帧 57B/4 航点解出 `reset='reset:N10,x'`、`tokens=['N10','K10','M10','L10,x']`，`messages=[]`、
`encoded=[]`、`topic=None`；③ 解析核的 `bridge()/sniff()/sniff_loop()` 换成炸弹仍跑通。
唯一输出话题写死为 `/dzlink/drone_to_car`（**depth10 / BEST_EFFORT**，与
`dzrobot_v2/.../io/contract.py:41,94` 逐字对齐，单测直接读车端契约源码核对）。

### 5-7 生产合入（2026-10-24 17:5x · AGC-17，Lead 裁决①~④）

四条裁决全部执行、未改判；生产只动三处写域，`dzlink_supervisor.py`／车端任何文件／
`dzbrain_bridge` **零改动**：

| 文件 | 来源 | 这次改了什么 |
|---|---|---|
| `dzsmartscout/tools/link/dzudp_core.py` | 教员 `udp_bridge.py` 逐字节快照 | 新增：文件头 docstring（来源路径／sha256 全值／抓取日期／只同步不改／改后重跑两根命令）＋分界行；分界行之后与原件逐字节相同 |
| `dzsmartscout/tools/link/dzudp_adapter.py` | 候选 `agc_binary_adapter.py` | 解析核候选顺序（仓内快照升为第一）＋单写者自检 `SingleWriterGuard` 与 `--selfcheck` CLI ＋ `RUNTIME_CONSTRAINT` 常量；解析/映射逻辑一字未动 |
| `.../tests/test_dzudp_adapter.py` | 候选单测 51 条 | 平移＋新增 3 条（运行时约束可读／诊断指向自检／守卫可导入） |
| `.../tests/test_dzudp_core_provenance.py` | 新增 | 12 条：快照段 sha256==PIN、PUBLISH_ENABLED=False、候选顺序、与原件/镜像逐字节相等 |
| `.../tests/test_dzudp_single_writer.py` | 新增 | 24 条：三条通道＋acquire/release＋CLI 退出码 0/2 |

**验收命令全绿（退出码 0）**：

| # | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 1 | `python -B -X utf8 -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py"` | **0** | Ran **195** · OK（合入前基线 105） |
| 2 | `python -B -X utf8 -m unittest discover -s _tmp_session/agc-1024/dl1/tests -p "test_*.py"` | **0** | Ran **51** · OK（候选件未动） |
| 3 | `python -B -X utf8 _tmp_session/agc-1024/dl1/verify_dl1.py` | **0** | 7 组独立复算一致 |
| 4 | `python -B -X utf8 dzsmartscout/tools/link/dzlink_supervisor.py selftest` | **0** | 结果：全部通过 |

**合入等价性（不是"看着像"）**：候选件 vs 生产件，10 组构造帧（含单桥洞/多桥洞/红蓝靶/出界/NaN/去重）＋
3 类坏帧（CRC 翻字节/截断/坏后缀）＋真帧 352B＋真帧 57B，比对 `messages`／`records`／`kind`／
`rejected` 全部一致 —— **PARITY_FAILURES = 0**。唯一差异是解析核落点（生产件落仓内快照），
而两者快照段 sha256 相同。

**provenance 与裁决字面的一处出入（必须写明）**：裁决要"文件头 docstring"＋"本地 sha256==PIN"，
但加文件头必然改变整文件哈希，两条按字面不可能同时成立。落地取**可核查的等价实现**：
`sha256(去掉文件头 = 分界行之后的全部字节) == PIN`（9ff259a3…64e8，11821B），
且该段与教员原件、取证镜像**逐字节相等**；PIN 写死在 provenance 单测里作独立常量，不读被测模块自证。

### 5-8 提交与双推（2026-10-24 18:0x · Lead 放行后执行）

- commit `a5a662df`：`link: 二进制:50000→JSON 薄适配层入仓（教员解析核钉死快照+PIN+单写者自检）[AGC-17]`
- 提交面复核：`git diff --cached --name-only` **正好 5 个**（两个生产 py + 三个 test_dzudp_*），无第 6 个；未带任何 docs/ 改动
- 双推：`git push github main` → `458af0eb..a5a662df`；`git push gitee main` → `458af0eb..a5a662df`
- 核对：`git ls-remote` 两端 `refs/heads/main` 均 = `a5a662dfdb8c863fcac7d3a04a29dea3d8b1b790` = 本地 HEAD

**遗留（不在本卡写域，交 Lead 定）**：仓 `.gitattributes` 是 `* text=auto` 且本机 `core.autocrlf=true`。
本次入库的 blob 是 **LF**（`git ls-files --eol` = i/lf，`git show :path` 与工作区 sha256 同为 `cd1167f1…`），
所以**仓内内容**与 PIN 一致；但 Windows 重新检出/`reset --hard` 会把工作区该文件变成 CRLF ⇒ provenance 单测会红
（机端 Linux 无此问题）。一行修法：`.gitattributes` 加 `dzsmartscout/tools/link/dzudp_core.py -text`。

## 6 字段保真表（二进制有/无 → JSON 有/缺）

| 二进制字段 | 帧里有吗 | JSON 落点 | 结果 | 处理原则 |
|---|---|---|---|---|
| `type`(0..12) | 有 | — | 13 类全映射 | 未知类**整帧拒收**（宁可少一帧，不许错标） |
| `x,y`(f64，机端 MapLocalizer 帧) | 有 | 格名 | 有（换算） | 0.065/0.295/0.35；残差 >0.35 格**逐对象**拒收，不拖累同帧其它目标 |
| `length,width`(f32) | 有 | — | **无** | 只进 `records`（JSON 无尺寸字段） |
| `ts`(u64 ms，机端时钟) | 有 | `ts` | 有，但用**车收时刻** | 与 `drone.py` 的"从收到算过期"同口径；机端 ts 只进 `records.frame_ts_ms` |
| `count` | 有 | — | 无 | 只用于长度/越界校验 |
| 每个目标的置信度 | **无** | `bridge_passable.confidence` | **合成常数 1.0** | 机端现行值；'bridge_passable.confidence' 写入 `records.synthetic_fields` 明示"这是合成" |
| `coverage{cells,complete,mode}` | **无** | `coverage` | **不发** | 不补空表：空表会被车侧 `_diff` 读成负证据 |
| `slice` / `scan_id` | **无** | 同名键 | **不发** | 不伪造幂等键（`(scan_id,slice.id)` 去重靠真值） |
| `field_layout`(version/start/end/known/bridges/…) | **无** | `field_layout` | **不发** | 4 个起终点类只进 records；布局必须由机端**规划侧**发 JSON，禁从 0x4001 反推 |
| mines/obstacles 的**空类别** | — | 空表 | 默认**整键不发** | 二进制无 coverage ⇒"这帧没看到"≠"场上没有"；`include_empty_classes=True` 可复刻机端"两键恒带" |
| targets 的红/蓝**颜色** | 有(3/4) | `target_zone.blocks` | 格名有、**颜色无** | 协议没有颜色字段；红/蓝只留 records（开火判据不能从情报链拿） |
| 三色桶 / 四起终点 / 格号 / 车 | 有 | — | **无** | `record_only`；桶的禁火语义要上链必须**新增字段**，不许偷偷并进 obstacles |
| 车位姿 0x4002(x,y,yaw) | 有 | — | **无** | 机→车没有 `car_pose` 报文；只记录 + 可落格做交叉校验 |
| 路线 0x4003(reset,route) | 有 | — | **无** | 只记录，`topic=None`，不写 `/nav/waypoint` |
| `heartbeat` | **无** | `heartbeat` | 默认**不发** | 链路"活着"由谁声明是 DL2 的决策；`emit_heartbeat=True` 才发 |
| `seq`/`ack` | **无** | `seq` | 可选注入 | 不塞假 seq；给了 `next_seq` 才写 |

### 6-1 消费端两条硬规则（Lead 裁决④ · 车侧口径，改代码前先看这里）

1. **缺失 ≠ 空列表**：本层对空类别**整键不发**。车侧 `_diff`／unknown 语义把"键缺失"读成
   **unknown（不知道）**，把"键在但 `[]`"读成**负证据（扫过、确实没有）**。二进制帧没有
   coverage 语义，所以只配给 unknown —— 要负证据，必须由机端规划侧带 `coverage/slice/scan_id`
   的 JSON 发，不能从 0x4001 反推。同理不伪造 `field_layout`。
2. **`bridge_passable.confidence = 1.0` 是合成常数**（协议无逐目标置信度），只为兼容既有
   消费者；**禁止**拿它参与开火／通行阈值判定（常数比阈值＝永远成立）。它同时写进
   `records.synthetic_fields`，事后可追。

### 6-2 运行时约束：禁止同 topic 双写（Lead 裁决③）

本适配器是 `/dzlink/drone_to_car` 的**唯一生产者**：**替代**机端 `dzbrain_bridge.py` 的情报出口，
**不并存**。两者同在线 = 双发布：同话题两个 writer，seq 互相插队、车侧 DroneTask 状态被来回覆盖，
且事后看不出是谁写的。机器可读形式：`dzudp_adapter.RUNTIME_CONSTRAINT`（同时进 `--diagnostics`）。

运行期自检 `python dzsmartscout/tools/link/dzudp_adapter.py --selfcheck`，三条通道任一判红即红：

| 通道 | 判什么 | 通道不可用 |
|---|---|---|
| second_instance | claim 文件被另一个**存活** pid 占用（`acquire()` 走 O_EXCL 原子创建，陈旧 claim 可接管） | —— |
| ros_graph | 该话题 publisher 数 > 1（`get_publishers_info_by_topic`，只读不发布） | unknown，不判红 |
| foreign_process | 同机进程表里还跑着 `dzbrain_bridge` | unknown，不判红 |

退出码：**0** = ok／unknown（unknown 不判红：离线 Windows 没 rclpy、没 `/proc` 是正常态）；
**2** = 判红 ⇒ 先停掉另一个生产者再起。

## 7 合入结论（已落盘未提交 · 等 Lead 放行）

**已执行（2026-10-24 17:5x）**：裁决①定的是 `(a) 入仓一份`，`(b) 永久 importlib 外部快照` 被否决 ——
理由是教员交接目录会搬走、机上没有这个文件，外部路径等于**静默漂移**（今天能跑，下周换个目录就
报错或悄悄换了实现）。两步合入按顺序落地，候选→生产只改三件事：解析核候选顺序、自检/CLI 新增、
`REPO` 反推自动适配（现在从 `dzsmartscout/tools/link/` 向上第三层就是仓根，无需硬编码）。

`_core_candidates()` 的最终顺序：`AGC_UDP_BRIDGE` 环境变量（测试/迁移逃生口）→
**仓内 `dzudp_core.py`** → 取证镜像 `_tmp_session/sqc-20261024/tools/udp_bridge.py` →
教员交接目录（兜底）。provenance 单测把"第 0 项必须是 `dzudp_core.py`"钉成断言，
防止哪天被悄悄改回外部路径。

**提交状态**：工作区**未提交**（`git status --short -- dzsmartscout/` 只列本次五个文件），
按 Lead 要求**停在这里等放行**；放行后才 add 这三个路径（`dzudp_adapter.py`／`dzudp_core.py`／
`tests/test_dzudp_*.py`）＋提交＋双推（v1 AGENTS §11）。

## 8 DL4 数据面阻塞清单（只列不修，2026-10-24）

| # | 阻塞 | 已知证据 | 影响本卡什么 | 不修的边界 |
|---|---|---|---|---|
| D1 | **雷达链路 carrier=0 → Init fail** | DRONE-AUTOSTART 归档卡 §L-12.3：雷达网线未插，`eno1 carrier=0`、`ping 192.168.3.105` 全丢，`dz-livox` 报 Init fail 但进程常驻 | 点云/定位链不可用 ⇒ 真值轨迹与飞行前提缺失 | 需现场插线+重启，非软件 |
| D2 | **真实相机样本缺失** | 三件套实测：机上感知未启动（`pgrep -af dzuav` 只有 watchdog）；352B/13 对象与 57B/4 航点是**机端同编码器工具帧**，不是相机检测结果 | 本卡只证"字节→JSON"正确，**不证明情报是真的**；真实误检率/多检率/丢帧率零样本 | 要启感知+实拍，属独立授权 |
| D3 | **FAST-LIO / PX4 未验** | 飞控板未插（无 `/dev/ttyACM*`、`lsusb` 无 FMU）；`/localization` 无发布者（`transform_fusion.py` 因 `tf2_ros.transformations` ImportError 死亡） | 飞行侧位姿不可用 ⇒ M5"经独立验收的飞行接入地面闭环"无输入 | 需现场插板+飞控域配置 |
| D4 | 车端 `UDP:50000` **无监听**（`ss -ulnp` 空） | 三件套实测 §1 | 广播现状**无人消费**：适配器要落地必须在**地面侧新起一个 50000 监听进程**（不是改现有进程），且与机端 `dzbrain_bridge` 单写者互斥 | 部署动作，等 Lead 排 |
| D5 | 样本覆盖窄 | 离线构造：0x4002 位姿真值、同半场多桥洞格、真实链路坏帧率均未实测 | 对接真实链路后可能出现"一帧里同半场两条桥洞"等组合，当前取字典序首格+告警 | 采数后才能定 |

**TOP3（按对整赛闭环的阻塞强度）**：D1 雷达 → D3 FAST-LIO/PX4 → D2 真实相机样本。
D1/D3 是"能不能飞"，D2 是"飞了看到的东西可不可信"；三者都不在本卡写域内，本卡只做记录。

## 9 结论与未证

**成立（实测）**：13 类映射完备且三处对账一致；坐标往返 154/154；未知类/坏 CRC/截断/长度谎报/NaN/坏后缀/未知 msg_id 全部拒收；
缺失字段保持缺失（无 coverage/slice/scan_id/field_layout，空类别整键不发）；0x4003 只记录零发布；
编出的报文能被既有 `protocol.mirror_car_side_parse` 与 `validate_payload` 吃下，无 `missing_fields`。

**未证（明确不做）**：真实相机→检测→广播的在线帧；0x4002 真值；与机端 `dzbrain_bridge` 同时在线的行为；
真实链路坏帧率。**本卡不连无人机、不飞行、不改车端**（任务书硬边界）。

**评审**：reviewer = e-integ（只读评审）。本卡 status 保持 in_progress，等 e-integ 出结论后由 Lead 归档。

## 10 文档变更

新增本卡；修改本 ns 任务树节点状态。归档：无。
2026-10-24 17:5x（AGC-17 合入）**修改本卡**：front matter 写域/结论、§5-7 合入记录与四次复跑、
§6-1 消费端两条、§6-2 运行时约束与自检、§7 合入结论、§10 本行。
**库内/库外零新增 .md**（R1 不触发）：所有新内容只写进这张卡；§8 阻塞清单若 Lead 要入
<<BT>>20_资料索引库/空地链路/<<BT>> 再由 Lead 提炼登记。

