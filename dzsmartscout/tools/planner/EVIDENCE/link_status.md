# DDS 链路现状复验（t60 验收④后半）

复验时间：2026-10-15 20:42:36（本机时钟）

> 只做非侵入探测（ICMP / TCP:22 / 本机网段 / 只读配置）。**未登车、未起 ROS、未发 DDS 包**。

## 1. 两端可达性

| 目标 | ICMP | TCP:22 | 说明 |
|---|---|---|---|
| 无人机 `10.29.122.104` | ❌ 不通(100% loss) | ❌ 关 | 无 RTT |
| 小车 `192.168.137.241` | ✅ 通 | ✅ 开 | Minimum = 8ms, Maximum = 9ms, Average = 8ms |

## 2. 本机网络（判断是否同网段）

```json
[
  {
    "IPAddress": "172.20.224.1",
    "PrefixLength": 20,
    "InterfaceAlias": "vEthernet (WSLCore)"
  },
  {
    "IPAddress": "169.254.34.108",
    "PrefixLength": 16,
    "InterfaceAlias": "本地连接* 2"
  },
  {
    "IPAddress": "169.254.59.251",
    "PrefixLength": 16,
    "InterfaceAlias": "蓝牙网络连接"
  },
  {
    "IPAddress": "169.254.251.204",
    "PrefixLength": 16,
    "InterfaceAlias": "本地连接* 1"
  },
  {
    "IPAddress": "192.168.137.51",
    "PrefixLength": 24,
    "InterfaceAlias": "WLAN"
  }
]
```

⇒ 本机与小车**同网段**：是 ✅（同 `192.168.137.0/24` 才为真）

## 3. 域号真值（只读取证）

| 来源 | 内容 |
|---|---|
| 机侧 `dzsmartscout/tools/link/dzlink.env` | `export ROS_DOMAIN_ID=118` |
| 车侧 `dzrobot_v2/dzros.env` | `DZ_ROS_DOMAIN="${DZ_ROS_DOMAIN:-118}"` |
| 车侧权威手册 `dzrobot_v2/docs/空地指挥链路_协议与运行手册.md` | 「域号 两端都 118」(:18/:259) |

⚠️ **与任务卡口径冲突（需队长裁断）**：t60 卡写「DDS 域两端 = 0」，但**车侧真源是 `118`**
（`dzros.env` 的 `DZ_ROS_DOMAIN=118`、`start_all.sh` 里 `ROS_DOMAIN_ID=118`、手册与 t4 实测一致）。
「0」只是**谁都没 source env 时**的默认值 —— 那正是项目里著名的静默故障（ping/SSH 全通、ROS 互不可见、零报错）。
⇒ 下发/探针**必须显式内联或 source**：机侧 `source ~/dzlink_tools/dzlink.env`（=118），
车侧由 `start_all.sh` source `~/dzrobot/dzros.env`（=118）。

## 4. 结论：本次**端到端 DDS 可见性未能验证**（网络前提未满足）

1. 无人机 `10.29.122.104` **不在线且不在本网段** ⇒ 无法验证「两端互见」。
2. 小车在本网段且 22 开，但按 t60 边界「车端栈保持冷态、不登车」⇒ 本次未做车侧 ROS 探测。
3. **要验证的下一步（无人机上电后）**：
   · 无人机接入**同一网段**（当前 192.168.137.0/24；接入后 `ip -4 addr` 应出现该网段地址）
   · 机侧：`source /opt/ros/humble/setup.bash && source ~/dzlink_tools/dzlink.env`（→118）
   · 机侧起常驻：`python3 ~/dzlink_tools/dzlink_supervisor.py run --domain 118 …`，看 `[domain] ROS_DOMAIN_ID=118 (来源: envfile)`
   · 车侧（**由队长决定窗口**，且车栈状态已知）再放只读探针 `/tmp/t4_car_consumer.py` 看 `[rx OK]`
   · 判据：机侧「订阅者匹配数 ≥1」+ 车侧 `[rx OK]` 同时成立；换 AP 后先跑一次组播测（`ros2 multicast send/receive`）
4. 在前提满足前，**任何「链路已通」的结论都是没有依据的** —— 本报告不假装通。

