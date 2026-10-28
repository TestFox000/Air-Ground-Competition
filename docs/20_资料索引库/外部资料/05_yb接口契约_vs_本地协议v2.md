---
来源: ext_yb_tree_20261022.tar.gz :: yb_tree/docs/成员交接/hu-jinyu/dzmission-doc/interface_contract.md:1-256〔未入库〕；:: AIR_GROUND_PROTOCOL.md:44-136〔未入库〕；:: AIR_GROUND_INTERFACE.md:187-265〔未入库〕
取证: 档（外部成员接口契约）+ 实（本地协议真源 空地链路/01）
更新: 2026-10-24
status: 现行
取代: 无
---

# 外部资料 · 05 yb接口契约 vs 本地协议v2

## TL;DR（给 agent 看的）

- **这是什么**：外部成员（hjy）交付的**两套**空地接口契约：一套是车机接口契约（UDP 50000、情报帧 0x4001 + 车位姿帧 0x4002、魔数 0xefefefef、CRC-16/Modbus），另一套是「统一帧」协议（A5 C3、CRC-16/CCITT-FALSE、14 类 TYP、UDP 9300/9301）。
- **一句话结论**：hjy 的二进制契约与本地部署线**不兼容**；本地真源是 dzuav UDP 50000 → /dzlink JSON（13 类识别协议），已由吸收决策 V1 裁决**保留部署线、二进制实现不集成**。
- **关键数字**：hjy 统一帧 14 类 TYP（0x01/0x10~0x17/0x20/0x21/0x30/0x31/0x7F）、帧长 12+LEN 最大 1036 B、CRC-16/CCITT-FALSE 向量 0x29B1；情报帧对象 25 B/个；车位姿帧固定 20 B。
- **最大坑**：hjy 三份文档的端口/字段口径**自相矛盾**——interface_contract 说 UDP 50000 + 0xefefefef，AIR_GROUND_PROTOCOL/INTERFACE 说 A5 C3 + UDP 9300/9301/9100；**引用前必须认版本**。
- **与本项目关系**：本地协议见 [空地链路/01](../空地链路/01_协议v2与契约.md)，本篇是它的**外部对照**，不是本地契约。

## 1. 这一篇是什么

- 来源是 tar ext_yb_tree_20261022.tar.gz 内 yb_tree/docs/成员交接/hu-jinyu/dzmission-doc/ 的三份契约文档（共 449 行）。
- 结构：interface_contract.md 是**部署口径**（情报 0x4001 / 车位姿 0x4002）；AIR_GROUND_PROTOCOL.md 与 AIR_GROUND_INTERFACE.md 是**统一帧规范**（同一帧格式，后者更新 v2.0、新增拓扑与 EVENT）。
- 取证分级：**档**（外部文档）+ **实**（本地协议真源为 空地链路/01）。

## 2. hjy 两套契约

### 2.1 车机接口契约（interface_contract.md，部署口径）

| 项 | 值 |
|---|---|
| 链路 | dzuav ──UDP:50000──→ ag_integrate（无人车香橙派） |
| 协议 | 二进制 UDP，单向广播，约 10 Hz，小端 |
| 情报帧 | msg_id **0x4001**，magic 0xefefefef，version 0x10，len = 2 + 25×count |
| 车位姿帧 | msg_id **0x4002**，固定 len=20（x float64 / y float64 / yaw float32） |
| CRC | CRC-16/Modbus，覆盖 version 到最后一个目标 |
| 目标对象 | 25 B：type(u8) + x(f64) + y(f64) + length(f32) + width(f32) |
| 目标类别 | 0 MINE / 1 CAR / 2 ARROW / 3 TARGET_RED / 4 CELL_LABEL / 5 TARGET_BLUE / 6 OBSTACLE |
| 坐标系 | 原点 N11=(0.5,0.5) 格单位；UDP 用米、车端内部用格（1 格=0.35 m） |

（interface_contract.md:9-86〔未入库〕）

### 2.2 统一帧规范（AIR_GROUND_PROTOCOL.md / AIR_GROUND_INTERFACE.md）

| 项 | 值 |
|---|---|
| 帧格式 | 0xA5 0xC3 | VER | TYP | SEQ(u32) | LEN(u16) | PAYLOAD | CRC_lo | CRC_hi |
| 版本 | VER=2（车端拒绝不兼容版本并上报） |
| CRC | CRC-16/CCITT-FALSE（poly 0x1021，init 0xFFFF，不反射），向量 0x29B1 |
| 帧长 | 12 + LEN，最大 **1036 B**（INTERFACE 版记 272 B，MAP_FULL 330 B，口径不一致） |
| 端口 | PROTOCOL 记 UDP 9300/9301；INTERFACE 记车端 bind 0.0.0.0:9100 |
| TYP | 0x01 HEARTBEAT(5Hz) / 0x10 TEAM_INFO / 0x11 FIELD_FIXED / 0x12 MAP_FULL(1Hz) / 0x13 MAP_DELTA / 0x14 MINE_LIST / 0x15 BRIDGE_GATE / 0x16 TARGET_LIST / 0x17 OBSTACLE_LIST / 0x20 TASK_CMD / 0x21 ROUTE / 0x30 VEH_STATUS(5Hz) / 0x31 EVENT / 0x7F ACK |
| 格坐标 | 单格 2 B（col 0..13, row 0..10）；列表 1 + 2×N，强制排序 |
| MAP_FULL | 10 + 154×2 = 318 B 载荷 |

（AIR_GROUND_PROTOCOL.md:44-135,379-398〔未入库〕；AIR_GROUND_INTERFACE.md:187-265〔未入库〕）

## 3. 与本地协议 v2 的对照

| 维度 | hjy 二进制契约 | 本地部署线（/dzlink JSON，13 类） | 出处 |
|---|---|---|---|
| 载体 | 二进制 UDP（0xefefefef 或 A5 C3） | JSON 文本，std_msgs/String，话题 /dzlink/drone_to_car | 空地链路/01 §1-§2 |
| 识别协议 | 7 类（interface_contract）/ 14 类 TYP（统一帧） | **13 类识别协议**（fe1d9df 为真源） | 吸收决策 §1.2 |
| 端口 | 50000 / 9300 / 9301 / 9100（文档互相矛盾） | ROS 话题（DDS），域号 118 | 空地链路/01 §7 |
| 帧/报文 | 情报帧 0x4001 + 车位姿 0x4002 | heartbeat / bridge_passable / target_zone / field_verify；field_layout v2 + coverage | 空地链路/01 §2 §4 |
| 车位姿 | 0x4002 固定 20 B（x, y, yaw） | 本地无独立车位姿帧，align 走 0x4002（hjy 口径） | 03 篇 + 吸收决策 |
| 数据语义 | 米制坐标 + 目标数组 | 格名集合 + coverage/complete（负证据地基） | 空地链路/01 §3 |

> **关键裁决**（吸收决策 V1）：hjy 线的 14 类 TYP 二进制帧（UDP 9300/9301）与本地 13 类部署线**不兼容**；**保留 13 类部署线**（fe1d9df + 15:04 事故裁决血缘），其协议文档仅作参考入档，二进制实现**不集成**。（吸收决策与待验证清单-20261018.md:33〔未入库〕）

## 4. 三条最容易踩的坑

1. **端口口径三处不同**：50000 / 9300 / 9301 / 9100 都出现在 hjy 文档里，引用前必须认是「部署口径」还是「统一帧规范」。
2. **CRC 两套混用**：空地链路用 CCITT-FALSE（0x29B1），裁判板串口无 CRC、末字节 BCC，/mission_cmd 调试链路用 CRC-16/MODBUS（0x4B37）——**不要混用**。
3. **0x4001/0x4002 与本地字段不可直接对接**：本地车端消费的是 JSON 字段语义，二进制帧的 25 B 目标结构不能原样映射到本地 13 类协议。

## 5. 本篇指向的对照入口

- 本地协议真源：[../空地链路/01_协议v2与契约.md](../空地链路/01_协议v2与契约.md)
- 本地链路缺口：[../空地链路/02_链路联调与已知缺口.md](../空地链路/02_链路联调与已知缺口.md)
- 车侧话题对账：[../空地链路/03_车侧话题对账_20261018.md](../空地链路/03_车侧话题对账_20261018.md)
- 0x4002 精度方案：[03_yb_精度对照_0x4002.md](03_yb_精度对照_0x4002.md)
- 吸收裁决：[../赛事与治理/04_决策台账摘编.md](../赛事与治理/04_决策台账摘编.md)

## 相关文件

- 本册：[00_索引.md](00_索引.md)
- 原文：ext_yb_tree_20261022.tar.gz 内 yb_tree/docs/成员交接/hu-jinyu/dzmission-doc/
