---
来源: 2026-10-25 实车打靶会话实测（两块板串口归属、两键注入、裁判板上行帧）、dzrobot_v2/src/dzjudgment/**、dzrobot_v2/src/dzrknn_dataprocess/**、dzrobot_v2/config/robot.yaml、docs/20_资料索引库/运维与排障/03_弹仓与射击复位.md、docs/20_资料索引库/视觉与云台/01_云台瞄准与开火链路.md
取证: 实+档
更新: 2026-10-25
取代: 无（03 讲弹药账本、05 讲日常动作流、视觉与云台/01 讲链路真源；本篇只补「从零打出一发」的操作序列与两块板的串口归属）
status: 现行
---

# 实车打靶 runbook（从零打出一发）

> 本篇是**操作序列**，不是原理：账本/复位 → [03](03_弹仓与射击复位.md)；起栈发车 → [02](02_起栈与发车_runbook.md)；改参/重启/部署 → [05](05_日常动作速查.md)；链路真源 → [视觉与云台/01](../视觉与云台/01_云台瞄准与开火链路.md)。
> **本篇会让激光真的出光**：每一发前先过 §0 与 §8 检查表。与代码/现场冲突时以代码和现场为准，并回来改本篇。

## TL;DR（给 agent 看的）

- **先核串口归属，再谈开火**：`/dev/ttyACM0` = `/dev/dzactuator`（1a86:55d4 CH340，云台/底盘板）、`/dev/ttyACM1` = `/dev/dzjudgment`（303a:1001 Espressif ESP32-S3，裁判/激光板）。`fuser -v` 里同一个口上出现两个进程 ⇒ **一定打不出光**（10-25 事故根因，见 §1）。
- **`dzjudgment` 必须走 launch**：`ros2 launch dzjudgment judgment.launch.py`。要看到 `judgment_serialport: /dev/dzjudgment` 与 `Serial initialize successfully!`，且**不得**出现 `Serial communication error`。
- **开火 = 两把钥匙同租期**：`/aim/fire_gate`（任务级）+ `/aim/fire_ready`（瞄准级），各自 500 ms 租期，必须同时为 1。**单给一把是空操作，是设计不是故障**（10-25 实测：只注 `fire_gate=1`，射击计数不变）。
- **打一发三步**：`/nav/scan_request=2` 扫掠 → `/aim/status == 2`（TRACK，**是 2 不是 1**，见 §5 校正）→ 25 Hz 双键连发 1.0 s（§5 脚本）。
- **真打出去 = 三条判据全中**：dzjudgment 日志 `★ 开火指令已下发 (B5 5B 03 21 01 25)` + 视觉节点 `pub_LaserShot_Command` + `/shoot_count == 1`。
- **打完先清软件账本**：`/shoot_reset`；裁判板 `ammo` 是硬件计数，**软件复位不了**（未解，见 §6）。

## 0. 前置与安全

| 前置 | 怎么确认 |
|---|---|
| 云台/视觉节点在跑 | `ros2 node list` 里能查到 `dzrknn_dataprocess` 与 `det_node` |
| 裁判驱动在跑 | `ros2 node list` 里能查到 `dzjudgment`（三者起栈顺序见 [02](02_起栈与发车_runbook.md)） |
| 串口没被抢 | §1 的 `fuser -v`，同口只允许一个进程 |
| 靶在场且蓝面朝车 | 人确认；`fire_only_blue: true`（`robot.yaml:203`）下红面**整站正确不开火**（[03](03_弹仓与射击复位.md) §3） |
| 云台朝向安全 | **人确认**。激光会真的出光，打出去不可撤回 |

> 安全边界：§5 的双键注入**绕过 AimTask 的几何判据**，落点可能不在开火框内。只在测试场地、靶已就位、旁人离开光路时用；比赛流程里两把钥匙由 MissionTask/AimTask 给，不手动注入。

## 1. 两块板子的串口归属（本篇核心新知识）

| 设备 | udev 软链 | VID:PID | 是谁 |
|---|---|---|---|
| `/dev/ttyACM0` | `/dev/dzactuator` | 1a86:55d4（CH340） | 云台/底盘板，`dzactuator_node` 独占 |
| `/dev/ttyACM1` | `/dev/dzjudgment` | 303a:1001（Espressif ESP32-S3） | 裁判/激光板，`dzjudgment` 独占 |

> **不要按 ACM 序号记板子**：序号会随插拔/上电顺序互换，判断只能靠 udev 软链或 VID:PID。

```bash
# ① 软链与真实设备（车端执行；每条新 SSH 先 source ~/dzrobot/dzros.env）
ls -l /dev/ttyACM* /dev/dzactuator /dev/dzjudgment

# ② 用 VID:PID 核对是不是那块板
udevadm info -q property -n /dev/ttyACM1 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL'

# ③ 谁在占口：同一个口上不允许出现两个进程
fuser -v /dev/ttyACM0 /dev/ttyACM1
```

| 结果 | 判读 |
|---|---|
| ACM0 → `dzactuator_node`，ACM1 → `dzjudgment` | ✅ 正常，可继续 §2 |
| 同一个口上出现两个进程（典型：两个都开 ACM0） | ❌ 停手。抢口时"开火指令已下发"仍会打印，但帧写进了隔壁板 |

> **原事故（10-25 实车）**：`judgment.cpp:6` 把默认串口写死 `/dev/ttyACM0`，正好是云台板，与 `dzactuator_node` 抢同一个口。
> 后果四条：① 开火帧 `B5 5B 03 21 01 25` 写进云台账；② 日志照样打 `★ 开火指令已下发`（`write()` 成功 ≠ 写对了口）；③ 激光不亮、裁判不计数；④ 串口每秒刷 `device reports readiness to read but returned no data`。
> 已修：`judgment.cpp:6-17` 默认值改为软链 `/dev/dzjudgment` 并写注释；`robot.yaml:289-291` 的 `/dzjudgment: mcuserialport` 同样指向软链。**判「写没写对」只能看上面的 ③，不能看日志。**

## 2. 起裁判节点（必须走 launch）

```bash
source ~/dzrobot/dzros.env
pkill -x dzjudgment          # 只杀同名进程；不要用 pkill -f（见 §7 坑1）
sleep 1
pgrep -c dzjudgment          # 起之前期望 0
ros2 launch dzjudgment judgment.launch.py
```

| 项 | 通过标准 |
|---|---|
| 横幅 | `judgment_serialport: /dev/dzjudgment`（`judgment.cpp:28`）+ `Serial initialize successfully!` |
| 反面 | **不得**出现 `Serial communication error`（出现 = 抢口/掉线/板子没上电） |
| 节点名 | `ros2 node list` 里有 `/dzjudgment`（launch 里 `name='dzjudgment'`，`dzrobot_v2/src/dzjudgment/launch/judgment.launch.py:50-57`） |
| 日志 | 起栈槽位 `~/robot_logs/6_judgment.log`（`tools/restart_node.sh:43`） |

- **为什么必须走 launch**：`robot.yaml` 的参数段是 `/dzjudgment:`，而 `judgment.cpp:3` 的节点名是 `dzjudgment_node` —— 只有 launch 的 `name='dzjudgment'` 能让两者对上。手工起会整段跳过参数（§7 坑2）。
- 单点重启的等价命令：`~/dzrobot/tools/restart_node.sh judgment`（槽位与命令见 [05](05_日常动作速查.md) §4）。

## 3. 话题契约与核对

| 话题 | 类型 | 方向（相对 dzjudgment） | 作用 |
|---|---|---|---|
| `/LaserShot_Command` | `UInt8` | 订阅 | `data==1` → 写 6 字节固定帧 `B5 5B 03 21 01 25`（`judgment.cpp:88-94`、`:369-413`） |
| `/referee_cmd` | `UInt8MultiArray` | 订阅 | 字节**原样**写裁判串口，用于代码里没有的帧（`judgment.cpp:96-128`） |
| `/HpAndHitmsg` | `UInt8MultiArray` | 发布 | 1 Hz 心跳 + 收帧即发；`payload=[targethp, self_hp, Bing_hit, Shooting_Count, ammo]`（`judgment.cpp:204-214`、`:340-362`） |
| `/shoot_count` | `Int32` | 云台侧发布 | 事件型：只在开火/重置时发，值=累计发数（`dzrknn_dataprocess.cpp:1276`） |

```bash
ros2 topic info /LaserShot_Command -v     # 期望 Publisher count: 1 / Subscription count: 1
ros2 topic hz /HpAndHitmsg                # 本次实测 10.9 Hz（1 Hz 心跳 + 每收一帧再发）
ros2 topic echo /HpAndHitmsg --once       # 看 payload 五元组
```

- Pub/Sub 任一为 0 = 链路断，先别打（"打了没记账"的先手判据，见 [03](03_弹仓与射击复位.md) §5）。
- 10.9 Hz 是本次实车值；换板/换固件后是否仍成立**未验证**。

## 4. 两把钥匙（为什么单给一把打不响）

| 钥匙 | 正常发布者 | 含义 |
|---|---|---|
| `/aim/fire_gate` | MissionTask | 任务级：已到站、停稳、FIRE 阶段 |
| `/aim/fire_ready` | AimTask | 瞄准级：目标确认 + 预测落点在开火框内 + 冷却/预算允许 |

- 放行实现：`dzrknn_dataprocess.cpp:1146-1153` —— 两键都为 true，**且**各自最后接收距现在 < 500 ms（`FIRE_PERMISSION_TIMEOUT_MS`，`dzrknn_dataprocess.h:110`）。
- `use_external_fire_gate: true`（`robot.yaml:228`）⇒ 云台只认这两把钥匙；`dbrain`/AimTask 没起时两键都没有发布者，**云台永远不会自己开火**。
- 10-25 实测：只注入 `/aim/fire_gate=1` 一次 → C++ 打印 `★ 收到【开火放行】…`，但 `pub_LaserShot_Command` 计数不增、`/shoot_count` 不变；当时 `/aim/fire_ready` 的 Publisher count = **0**。⇒ **单给一把是空操作，是设计不是故障。**
- 查现状：`ros2 topic info /aim/fire_gate`、`ros2 topic info /aim/fire_ready`。

### 开火判据里的当前阈值（仓库版快照）

| 参数 | 值 | 位置 |
|---|---|---|
| `fire_dz_far_px` / `fire_dz_near_px` | 25 / 26 px（按框宽 30→120 px 线性映射） | `robot.yaml:177-186` |
| `fire_hold_frames` | 1（10-24 由 5 改 1） | `robot.yaml:187` |
| `target_confirm_frames` | 1 | `robot.yaml:193` |
| `shot_cooldown_ms` | 500 | `robot.yaml:198` |
| `max_shots` / `max_shots_per_target` | 10 / 10 | `robot.yaml:204`、`:208` |

> 这是**仓库版**，车端可能已分叉（[05](05_日常动作速查.md) §2/§9）：上车先 `grep -nE 'fire_hold_frames|target_confirm_frames|fire_dz_' ~/dzrobot/config/robot.yaml` 读现值。
> [视觉与云台/01](../视觉与云台/01_云台瞄准与开火链路.md) §4 表里的 `fire_hold_frames=5`/`target_confirm_frames=2` 是 10-18 口径，10-24 用户指令改为 1/1；以车端 yaml 为准（该表待其 owner 更新）。

## 5. 从零打出一发（本次实测通过的五步）

**Step 0 — 先清软件账本**（`max_shots` 是进程生命周期累计，试射遗留会让"打了没反应"，见 [03](03_弹仓与射击复位.md) §1）

```bash
ros2 topic pub --once /shoot_reset std_msgs/msg/UInt8 "{data: 1}"
# 期望日志：==> ★ 射击计数已重置：shots_fired_=0，可用弹药恢复为 N 发
ros2 topic echo /shoot_count --once      # 期望 data: 0
```

**Step 1 — 让云台扫掠锁靶**

```bash
ros2 topic pub --once /nav/scan_request std_msgs/msg/UInt8 "{data: 2}"
```

`data>=2` = 持续扫描（扫完自动重来），`1` = 单趟，`0` = 停止扫描并归位（`dzrknn_dataprocess.cpp:944-975`）。已在 TRACK 时非零请求**不会**打断跟随。

**Step 2 — 等 TRACK（有一处口径要校正）**

```bash
ros2 topic echo /aim/status --once       # 期望 data: 2 = TRACK
ros2 topic echo /offset_center --once    # 期望 cls=1（蓝面）
```

> ⚠️ **口径校正**：本次会话的事实记录曾写「等 `/aim/status` 变 1（TRACK）」，与源码不符 —— `dzrknn_dataprocess.h:140` 的枚举是 `AIM_IDLE=0, AIM_SCAN=1, AIM_TRACK=2`。**按 2 判**；停在 1 说明还在扫，停在 0 说明已回 IDLE（可先发 `scan_request=0` 再重发 2）。

**Step 3 — 双键注入（25 Hz × 1.0 s）**

在车上另开一个 shell（先 `source ~/dzrobot/dzros.env`），存成 fire_once.py（文件名自定）再 `python3 fire_once.py`：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试场地手动补齐两把开火钥匙：25 Hz 连发 fire_gate + fire_ready 共 1.0 s，结束补发 0。

前置：靶在场、云台已 /aim/status==2(TRACK)、云台朝向安全。
风险：绕过 AimTask 的几何判据 —— 落点可能不在开火框内。
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

HZ = 25
DURATION_S = 1.0


def main():
    rclpy.init()
    node = Node("manual_fire_keys")
    gate = node.create_publisher(UInt8, "/aim/fire_gate", 10)
    ready = node.create_publisher(UInt8, "/aim/fire_ready", 10)

    time.sleep(0.5)  # 等 DDS 发现订阅端（云台订阅是 best_effort、depth 1）

    on = UInt8(data=1)
    off = UInt8(data=0)
    period = 1.0 / HZ           # 25 Hz ≈ 0.04 s
    sent = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < DURATION_S:
        gate.publish(on)
        ready.publish(on)
        sent += 1
        time.sleep(period)

    gate.publish(off)           # 结束必须补 0，别把两键留在 1
    ready.publish(off)
    node.get_logger().info("双键已发 %d 轮（%.2f s），已补发 0" % (sent, DURATION_S))
    time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- **为什么必须连发**：两键各有 500 ms 租期，只发一次 1 会过期；C++ 每收到一帧就续租（`dzrknn_dataprocess.cpp:1155-1184`）。
- 本次实测：脚本起跑后 **t≈0.14 s** 出光。3 s 内没出光就 Ctrl-C 停手排查，别连发烧弹药。
- 分两条 `ros2 topic pub --once` 依次发两键能否落在同一 500 ms 租期 → **未验证**；本次只用上面的脚本。

**Step 4 — 三条判据（全中才算真打出去）**

| # | 看什么 | 命令 | 通过标准 |
|---|---|---|---|
| a | 裁判驱动收到并写出 | `grep '开火指令已下发' ~/robot_logs/6_judgment.log` | `[dzjudgment-->] ★ 开火指令已下发 (B5 5B 03 21 01 25)` |
| b | 云台真的发了指令 | `grep pub_LaserShot_Command ~/robot_logs/5_gimbal.log` | `pub_LaserShot_Command  [1/10]  dz=…  v=…  realV=…`（`dzrknn_dataprocess.cpp:381-385`） |
| c | 计数变了 | `ros2 topic echo /shoot_count --once` | `data: 1`（第 N 发 = N） |

- 只中 b 不中 a：`dzjudgment` 没起或在抢口 → 回 §1/§2。
- `/shoot_count` 是事件型：`--once` 一直等也不返回，说明**根本没开火**，Ctrl-C 后按 §4 查两键与开火框。

**本次实测那一发**

| 项 | 值 |
|---|---|
| 出光时刻 | 双键开始后 t≈0.14 s |
| `/offset_center`（开火帧） | dx=-13 dy=33 cls=1 box=46x45 |
| 现场 | 用户确认听到/看到出光 |

> dx/dy 是开火时刻的**原始读数**；判据用的是滤波值再减激光零点（`dzrknn_dataprocess.cpp:299-302`），两者不能直接对比。本例 dy=33 仍能开火，说明零点/滤波把有效误差压进了框内；具体换算**未验证**，别拿原始 dx/dy 反推开火框。
> 一次成功 ≠ 批量稳定：本次只打 1 发，多发改的连续性与散布**未验证**。

## 6. 打完复位

| 动作 | 命令 | 判据 |
|---|---|---|
| 清软件账本（每轮/每次试射前同样先发） | `ros2 topic pub --once /shoot_reset std_msgs/msg/UInt8 "{data: 1}"` | 日志 `==> ★ 射击计数已重置`；`/shoot_count` 重发 0 |
| 裁判板弹药 | **软件复位不了** | `ammo` 是硬件计数（[03](03_弹仓与射击复位.md) §2） |
| 裁判系统本身的复位/开启 | 通道有：`/referee_cmd` 原样下发任意帧（`judgment.cpp:96-128`） | **帧字节未确定 → 未解**（下条） |

> **未解**：裁判系统复位/开启帧的字节本次没拿到。三条线索：
> ① `dzrobot_v2/src/history/附加资源/智能车程序说明/案例9：云台控制与射击.pdf`；
> ② `dzrobot_v2/src/history/附加资源/车载裁判系统/SmartMaster裁判系统操作指南-V1.0.pdf`（11 页、几乎全图，需人看）；
> ③ `dzrobot_v2/src/history/附加资源/车载裁判系统/二进制文件.zip`（内含 `esp32s3web.ino` 的 elf/map；ESP32-S3 带 web 服务，可挖字符串）。
> **帧语义没确定前不要往 `/referee_cmd` 发字节**：它会原样写进裁判串口，写错帧可能被板子当命令执行。本次未验证任何自定义帧。

## 7. 本次实车新增的四个坑

| # | 现象 | 机制 | 处置 |
|---|---|---|---|
| 1 | `pkill -f 'install/dzjudgment/lib'` 之后"没输出、进程也没死"，重复几次反而堆出多个 `dzjudgment` 抢串口 | `pkill -f` 匹配整条命令行，而执行它的 shell 自己的 cmdline 里就含这个串，**先把自己杀了** | 只用 `pkill -x dzjudgment`；再 `pgrep -c dzjudgment` 必须为 1 |
| 2 | 手工起节点后串口参数不生效 | `judgment.cpp:3` 的节点名是 `dzjudgment_node`，与 `robot.yaml` 的 `/dzjudgment:` 段对不上 ⇒ 参数段被跳过，回落源码默认值 | 一律走 launch；默认值现已兜底为 `/dev/dzjudgment`（`judgment.cpp:6-17`） |
| 3 | 以为改 `dzjudgment` 已被 deploy 带上车 | `tools/deploy_to_car.py:55-74` 的 `PUSH_FILES` 不含 `src/dzjudgment`；`BUILD_PACKAGES` 只有 `dzbrain`/`dzrknn_dataprocess`（`:89`）。`--packages` 只影响 build、不影响 push | 改它要单独核对车上那份代码；别假设 deploy 已包含 |
| 4 | 裁判板上行原始帧看不懂 | 实测 `0a 0a 10 01 09 4c b5 5b 07 07` → `payload [10, 10, 16, 1, 9]`；`[2]=16=0x10` 与源码常量「未被击打」吻合 | 按 `publish_referee_state()` 的 push 顺序（`judgment.cpp:204-214`）= `[targethp, self_hp, Bing_hit, Shooting_Count, ammo]`，**末位才是弹药**。实测值 ⇒ 对方血 10 / 己方血 10 / 未被击打 0x10 / 已记 1 次射击 / **剩余弹药 9**。会话曾传「第一个 10=弹药」，**已证伪**，别再用首字段做弹药判据 |

## 8. 一页检查表（照着勾）

- [ ] `fuser -v /dev/ttyACM0 /dev/ttyACM1`：ACM0 只有 `dzactuator_node`，ACM1 只有 `dzjudgment`
- [ ] `ros2 launch dzjudgment judgment.launch.py`：横幅 `serialport: /dev/dzjudgment` + `initialize successfully`，无 `Serial communication error`
- [ ] `ros2 topic info /LaserShot_Command -v` → Publisher 1 / Subscription 1
- [ ] `/aim/fire_gate`、`/aim/fire_ready` 两个订阅端都在（发布端由 §5 脚本临时补）
- [ ] `/shoot_reset` 已发，`/shoot_count == 0`
- [ ] `/nav/scan_request=2` 已发，`/aim/status == 2`
- [ ] 靶在场、蓝面朝车、云台朝安全方向（**人确认**）
- [ ] fire_once.py 已跑（25 Hz × 1.0 s）
- [ ] 三条判据 a/b/c 全中
- [ ] 打完发 `/shoot_reset`，并记下 `pub_LaserShot_Command` 的计数

## 9. 未验证 / 未解（别当结论用）

| # | 条目 | 状态 |
|---|---|---|
| 1 | 裁判系统复位/开启帧的字节 | **未解**（三条线索见 §6） |
| 2 | `/HpAndHitmsg` payload 里哪个字段是剩余弹药 | **已解**：末位 = `ammo`（`publish_referee_state()`，`judgment.cpp:204-214`）。实测 `[10,10,16,1,9]` ⇒ `ammo=9`、`Shooting_Count=1`，即该帧时刻**裁判板已认账 1 发** |
| 3 | `fuser -v` 的实际输出格式 | 未验证（本次只按进程名核对，未留原始输出） |
| 4 | 车端 `/etc/udev/rules.d/dz{judgment,actuator}.rules` 的内容 | 未验证（规则不在本仓库；源码注释只说按 idVendor/idProduct 固定） |
| 5 | 两条独立 `ros2 topic pub --once` 依次发两键是否落在 500 ms 租期内 | 未验证 |
| 6 | 多发改的连续性与散布 | 未验证（本次 1 发） |
| 7 | `/HpAndHitmsg` 10.9 Hz 的可复现性（换板/换固件） | 未验证 |

## 变更记录

| 日期 | 变更 | 来源 |
|---|---|---|
| 2026-10-25 | 建篇：串口归属 + 起裁判节点 + 两键注入脚本 + 三条判据 + 复位 + 四个坑 + 检查表 | 10-25 实车实测 + `dzjudgment`/`dzrknn_dataprocess` 源码核对 |
| 2026-10-25 | 校正两处传入口径：`/aim/status` TRACK=**2**（非 1）；`/HpAndHitmsg` 的 ammo 在末位（非首位），并以「未定论」挂账 | `dzrknn_dataprocess.h:140`、`judgment.cpp:208-212` |
| 2026-10-25 | 结清 §9-2：弹药字段确认为末位（`[targethp, self_hp, Bing_hit, Shooting_Count, ammo]`），实测 `ammo=9`/`Shooting_Count=1`；「首字段=弹药」的传入口径已证伪 | `judgment.cpp:204-214` + 实车 `/HpAndHitmsg` 实测帧 |

## 相关文件

- [03_弹仓与射击复位.md](03_弹仓与射击复位.md)（弹药三口径、7 条 AND、红面拦截、裁判节点掉线）
- [02_起栈与发车_runbook.md](02_起栈与发车_runbook.md)、[05_日常动作速查.md](05_日常动作速查.md)、[00_索引.md](00_索引.md)
- [视觉与云台/01_云台瞄准与开火链路.md](../视觉与云台/01_云台瞄准与开火链路.md)（链路真源：拓扑、8 项下标、两键与 P0）
- `dzrobot_v2/src/dzjudgment/src/judgment.cpp`（`:6-17` 默认口、`:96-128` 自定义帧、`:204-214` 回传、`:340-362` 0x07 解析、`:369-413` 开火帧）
- `dzrobot_v2/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp`（`:299-302` 开火框、`:944-975` 扫描请求、`:1146-1153` 两键租期、`:1276` 计数发布）
- `dzrobot_v2/config/robot.yaml`（`:203` 只打蓝、`:228` 外部门、`:289-291` 串口参数段）
- `dzrobot_v2/tools/deploy_to_car.py`（`:55-74` 推送清单、`:89` 构建包）、`dzrobot_v2/tools/restart_node.sh`（`:43` judgment 槽位）
