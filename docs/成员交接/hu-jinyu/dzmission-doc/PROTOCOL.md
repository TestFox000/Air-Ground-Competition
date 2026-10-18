# dzmission 任务指令协议 v1

地面车接收无人机（或上位机）下发的任务指令，执行后回传状态。
物理层可为串口数传或 UDP，应用层帧格式一致。

---

## 1. 帧格式

小端（little-endian）。

```
+--------+--------+-----+-----+-----------------+--------+--------+
| 0xAA   | 0x55   | LEN | CMD | PAYLOAD[LEN-1]  | CRC_lo | CRC_hi |
+--------+--------+-----+-----+-----------------+--------+--------+
   1B       1B      1B    1B        LEN-1 B          1B       1B
```

| 字段 | 说明 |
|---|---|
| `0xAA 0x55` | 固定帧头 |
| `LEN` | `1 + PAYLOAD 长度`，取值 `1..64` |
| `CMD` | 指令码 / 上报码，见下表 |
| `PAYLOAD` | 指令参数 |
| `CRC16` | CRC-16/MODBUS（poly 0xA001 反射，init 0xFFFF），**计算范围：从 `LEN` 到 `PAYLOAD` 末字节** |

整帧长度 = `3 + LEN + 2`，最大 `69` 字节。

### CRC-16/MODBUS 参考实现

```c
uint16_t crc16_modbus(const uint8_t *d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; ++i) {
    crc ^= d[i];
    for (int b = 0; b < 8; ++b)
      crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
  }
  return crc;
}
```

验证向量：

| 输入 | CRC 计算范围 | 结果 |
|---|---|---|
| `"123456789"`（标准校验串） | 全部 9 字节 | `0x4B37` |
| CMD=0x00（NOP，PAYLOAD 空） | `{0x01, 0x00}` | `0x2000` |

NOP 完整帧：`AA 55 01 00 00 20`（CRC 低字节 `0x00` 在前，高字节 `0x20` 在后）。

`dzmission/test/protocol_check.py` 是同一算法的 Python 参考实现，
可离线跑 `python3 protocol_check.py` 自检，也可用 `--nav-to` / `--engage`
直接生成可注入的十六进制帧。

---

## 2. 上行指令（无人机 → 地面车）

| CMD | 名称 | PAYLOAD | 说明 |
|---|---|---|---|
| `0x00` | NOP | 空 | 心跳，仅刷新链路计时 |
| `0x01` | NAV_TO | 12B | 前往地图坐标点 |
| `0x02` | STOP | 2B | 立即停车 |
| `0x10` | ENGAGE | 7B | 进入交战，打击指定目标 |
| `0x11` | CEASE_FIRE | 1B | 停火，保留交战姿态 |
| `0x20` | RETURN_BASE | 1B | 返航到 home |
| `0x21` | HOLD | 1B | 原地待命 |
| `0x30` | TARGET_REPORT | 6B | 视觉目标上报（不改变任务状态） |
| `0x40` | SET_HOME | 9B | 设置 home 点 |
| `0x7F` | HEARTBEAT | 空 | 同 NOP |

### 2.1 NAV_TO (0x01)

| 偏移 | 类型 | 单位 | 字段 |
|---|---|---|---|
| 0 | int32 | cm | `x`（map 系） |
| 4 | int32 | cm | `y`（map 系） |
| 8 | int16 | 0.1° | `yaw`（-1800..1800） |
| 10 | uint8 | — | flags：bit0=1 表示使用 yaw |
| 11 | uint8 | — | 指令序号 seq |

地面车行为：把 `(x, y)` 转成 `NavigateToPose` 目标下发 nav2；
到达判定 `dist < mission.arrive_dist`（默认 0.25 m）。

### 2.2 STOP (0x02)

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | uint8 | `stop`：1=停车并清空所有任务意图；0=解除停车，回到 IDLE |
| 1 | uint8 | seq |

### 2.3 ENGAGE (0x10)

| 偏移 | 类型 | 单位 | 字段 |
|---|---|---|---|
| 0 | uint8 | — | `target_id`（1=敌方靶标，2=物资点） |
| 1 | uint8 | % | `confidence` 视觉置信度 0..100 |
| 2 | uint8 | — | flags，bit0=1 表示按方位距离解算接近点 |
| 3 | int16 | 0.1° | `bearing_deg`：目标相对**本车车头**的方位，正=右，负=左 |
| 5 | uint16 | cm | `range_m`：目标到本车的水平距离 |
| 6 | uint8 | — | seq |

地面车行为：
1. 解算目标点 `P = 车体位置 ⊕ rotate(bearing) * range`，并朝目标点收缩到
   `mission.engage_approach_dist`（默认 1.2 m）作为打击位置；
2. 下发导航目标前往；
3. 云台进入交战状态（扫描找靶 → 锁定 → 视觉伺服 → 开火）。

### 2.4 CEASE_FIRE (0x11) / RETURN_BASE (0x20) / HOLD (0x21)

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | uint8 | seq |

### 2.5 TARGET_REPORT (0x30)

| 偏移 | 类型 | 单位 | 字段 |
|---|---|---|---|
| 0 | uint8 | — | `target_id` |
| 1 | uint8 | % | `confidence` |
| 2 | int16 | 0.1° | `bearing_deg` |
| 4 | uint16 | cm | `range_m` |
| 5 | uint8 | — | seq |

仅更新目标缓存，不改变任务状态。用于无人机持续引导。

### 2.6 SET_HOME (0x40)

| 偏移 | 类型 | 单位 | 字段 |
|---|---|---|---|
| 0 | int32 | cm | `x` |
| 4 | int32 | cm | `y` |
| 8 | uint8 | — | seq |

home 航向取设置时刻本车航向。

---

## 3. 下行上报（地面车 → 无人机）

| CMD | 名称 | PAYLOAD |
|---|---|---|
| `0x81` | STATUS | 24B |
| `0x82` | TARGET_ACK | 14B |
| `0x83` | FAULT | 2B |

### 3.1 STATUS (0x81)

| 偏移 | 类型 | 单位 | 字段 |
|---|---|---|---|
| 0 | uint8 | — | seq（地面车自增） |
| 1 | uint8 | — | mission_state，见 3.4 |
| 2 | uint8 | — | fault，见 3.5 |
| 3 | uint8 | — | self_hp 己方血量 |
| 4 | uint8 | — | ammo 剩余弹量 |
| 5 | uint8 | — | hit_dir：0x01左 0x02前 0x03右 0x04后 0x10未中弹 |
| 6 | uint8 | — | shoot_count 累计射击次数 |
| 7 | uint8 | % | battery_pct |
| 8 | int32 | cm | x（map 系，定位补偿后） |
| 12 | int32 | cm | y |
| 16 | int16 | 0.1° | yaw |
| 18 | int16 | cm/s | v 线速度 |
| 20 | int16 | 0.01 rad/s | w 角速度 |
| 22 | uint8 | — | seq（重复，兼容） |
| 23 | uint8 | — | 保留 |

上报周期：`200 ms`（`MissionManager::tick` 中 `last_status_time_` 控制）。

### 3.2 TARGET_ACK (0x82)

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | uint8 | seq（回显 ENGAGE 的 seq） |
| 1 | uint8 | target_id |
| 2 | uint8 | result：0未知 1命中 2未命中 3中止 |
| 3 | uint8 | ammo_left |
| 4 | int32 | aim_x（cm，打击时刻本车位置） |
| 8 | int32 | aim_y（cm） |
| 12 | uint8 | 保留 |
| 13 | uint8 | 保留 |

命中判定：交战期间裁判板上报的 `target_hp` 相比交战开始时下降 → `result=1`。
开火后 2 s 内目标丢失且对方血量未下降 → `result=2`。
交战被联锁拒绝（无弹 / 低电 / 定位失效）→ `result=3`。

### 3.3 FAULT (0x83)

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | uint8 | fault code |
| 1 | uint8 | detail |

### 3.4 mission_state

| 值 | 状态 | 含义 |
|---|---|---|
| 0 | DISABLED | 未收到任何指令 / autonomy 关闭 |
| 1 | IDLE | 待命 |
| 2 | NAVIGATING | 前往目标点 |
| 3 | ENGAGING | 交战中 |
| 4 | RETURNING | 返航中 |
| 5 | HOLDING | 原地待命 |
| 6 | ESTOP | 安全急停（雷达近距 / 定位失效） |
| 7 | ARRIVED | 已到达目标点 |

### 3.5 fault code

| 值 | 含义 |
|---|---|
| 0 | 无故障 |
| 1 | 无弹（NO_AMMO） |
| 2 | 电量低（LOW_BATTERY） |
| 3 | 定位不可用（NO_LOCALIZATION） |
| 4 | 前方障碍堵塞（OBSTACLE_BLOCKED） |
| 5 | 裁判板链路丢失（REFEREE_LINK_LOST） |
| 6 | 指令链路丢失（CMD_LINK_LOST） |
| 7 | 视觉目标丢失（VISION_LOST） |

同一故障码最多每 2 s 重发一次，避免刷链路。

---

## 4. 安全联锁汇总

地面车在以下情况会**拒绝开火**（并回 `TARGET_ACK result=3` 或 `FAULT`）：

| 条件 | 参数 | 结果 |
|---|---|---|
| 弹量 < `aim.min_ammo_to_fire` | 默认 1 | 不开火 + `FAULT_NO_AMMO` |
| 电量 < `mission.low_battery_pct` | 默认 20% | 停火 + 返航 + `FAULT_LOW_BATTERY` |
| 定位不健康且 `engage_requires_localization` | 默认 true | 拒绝交战 + `FAULT_NO_LOCALIZATION` |
| 正在被击打（`being_hit`） | `referee.hit_hold_ms` 内 | 暂时不开火 |
| 仅当 `require_stationary=true` 时才要求车体静止 | 默认 false | — |

以下情况会**强制停车**（优先级最高）：

| 条件 | 参数 |
|---|---|
| 雷达前向最小间距 < `safety.stop_dist` | 默认 0.32 m |
| 侧向间距 < `safety.side_warn_dist * 0.6` | 默认 0.21 m |
| 定位数据 500 ms 内未更新 | 硬编码 0.5 s |
| `STOP` 指令 | — |

障碍消失并持续 `safety.clear_hold_ms`（默认 400 ms）后自动恢复原任务状态。

---

## 5. 轻量调试协议（ROS 话题）

若不方便直接对接二进制帧，可向 `/mission_cmd` 发布
`std_msgs/msg/UInt8MultiArray`，字段与二进制 PAYLOAD 一一对应：

```
data[0]  = CMD
data[1]  = seq
data[2]  = target_id
data[3]  = confidence
data[4]  = stop
data[5]  = has_yaw
data[6..7]  = bearing_deg * 10   (int16 小端)
data[8..9]  = range_m * 100      (uint16 小端)
data[10..15]= 保留（NAV_TO 时 x/y 请直接走二进制链路）
```

示例（进入交战，目标在右前方 30°，距离 3.5 m）：

```bash
ros2 topic pub --once /mission_cmd std_msgs/msg/UInt8MultiArray \
  "{data: [16, 1, 1, 90, 0, 0, 44, 1, 94, 1, 0, 0, 0, 0, 0, 0]}"
# data[6..7]  = 300 (0x012C) -> 低字节 0x2C=44, 高字节 0x01=1
# data[8..9]  = 350 (0x015E) -> 低字节 0x5E=94, 高字节 0x01=1
```

完整二进制帧版本（`/mission_cmd_raw` 仅用于观察，输入请用串口/UDP）：

可以通过 `cmd.transport=loopback` 让节点自身注入 NAV_TO，验证全链路。
