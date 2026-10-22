# 空地协同 · 统一数据模型与面板设计（t4）

> 撰写：coordination-architect　｜　日期：2026-10-17　｜　性质：**纯设计，未改一行代码**
> 上游：`docs/merge/空地协同插件整合方案.md`（讨论稿 v1，以下简称「讨论稿」）。
> 本文**不是**另起一套方案，而是讨论稿 §3.3 / §4 / §5 的**字段级细化 + 现状更正**。
> 凡与讨论稿不一致处，本文以「★更正」标注并给出代码行号证据。
> 凡我没有实测到的，明确写「未验证」。

---

## 0. 一句话结论

**合并后的面板不该是「两个节点各一张卡」，而该是「三条链路各自独立的一条状态轴」。**

因为车与机之间真正会坏的东西有三层，且**互不影响**：SSH 通不通、ROS 栈起没起、`/dzlink` 心跳有没有。
现有两个面板都把这三层压成一个布尔值（`probe.state`），于是「车能 SSH 但 ROS 没起」只能显示成一句
`底盘未发布电量（ROS 栈未运行）`（`dzrobot…/client.js:157`），而「机 SSH 通、飞控板插着、DDS 静默」被显示成 `无飞控`——**后者是现存 bug，不是设计口味问题**（证据见 §1.3 漏洞 B）。

---

## 1. 统一节点模型

### 1.1 现状：两个插件各有一份「节点」，形状还不对称

| 项 | 小车 | 无人机 | 出处 |
|---|---|---|---|
| 模块加载 id | `dsh-dzrobot-ssh` | `dsh-dzdrone-ssh` | `dzrobot…/client.js:34` vs `dzdrone…/client.js:33` |
| API 前缀 | `/api/dzrobot-ssh` | `/api/dzdrone-ssh` | `client.js:41` / `client.js:40` |
| locale NS | `dzrobotSsh` | `dzdroneSsh` | `client.js:42` / `client.js:41`（`dzdrone…` 的 `NS` 在 `:41`） |
| PANEL_ID | `dzrobot-car` | `dzdrone-drone` | `client.js:44` / `client.js:43` |
| inject 列表 | 5 项，两侧**逐字相同** | 同左 | `client.js:46` / `client.js:45` |
| `/status` 里的节点键名 | **`car`** | **`drone`** | `dzrobot…/index.js:161`、`dzdrone…/index.js:150` |
| 数据目录 | `dzrobot-ssh` | `dzdrone-ssh` | `src/store.js:21` 两侧 |
| 默认主机 | `duzhong.local` | `dzsmart.local` | `src/store.js:30` 两侧 |
| 默认 VNC 密码 | `''`（空 → noVNC 问） | `duzhong` | `src/store.js:52` 两侧 |

**只有键名 `car`/`drone` 与那几个字符串不同，其余结构同构**。这就是为什么统一节点模型不是「抽象一个新概念」，而是「把一个已经复制两份的东西收成一份」。

### 1.2 设计：`NodeSnapshot` —— 一条数据面同时表示两个节点

```jsonc
// GET /api/dzag/status  （★一个请求，两个节点）
{
  "schema": "dzag.status/1",
  "at": 1792234800000,                 // 宿主时钟，本响应的组装时刻（不是任何读数的时刻）
  "nodes": {
    "car": {
      "id": "car",
      "role": "car",                   // car | drone —— 决定 /:role/* 路由与能力集
      "kind": "ground",                // ground | air —— 协同层用它决定谁指挥谁
      "label": "小车",
      "target": "duzhong@duzhong.local",   // 留给 tooltip，与 ssh.js:211 同源（别名优先）
      "probe": {                            // ① SSH/登录层
        "state": "ready",                   // ready|unreachable|needs-password|host-key|error
        "detail": "免密登录可用",
        "latencyMs": 1774,
        "at": 1792234798226,                // ★这个探针本身是什么时候做的
        "source": "live"                    // live | cached —— 见 §1.3
      },
      "desc": { "user": "duzhong", "host": "rk3588", "kernel": "…", "up": "2 hours" },
      "caps": {                             // ★能力声明：UI 据此决定画什么，不写死
        "gauges": ["batteryPercent", "batteryVoltage", "cpuPercent", "uptimeSec"],
        "hasSim": true,
        "hasTerminal": true, "hasFiles": true, "hasVnc": true
      }
    },
    "drone": {
      "id": "drone", "role": "drone", "kind": "air", "label": "无人机",
      "target": "duzhong@dzsmart.local",
      "probe": { "state": "unreachable", "detail": "网络不可达（无人机还没连上 WiFi？）",
                 "latencyMs": 3070, "at": 1792234797796, "source": "live" },
      "desc": null,
      "caps": {
        "gauges": ["temp", "cpuPercent", "uptimeSec", "batteryPercent", "batteryVoltage"],
        "hasSim": false,
        "hasTerminal": true, "hasFiles": true, "hasVnc": true
      }
    }
  }
}
```

**关键取舍 1：`probe` 只描述 SSH 层，不许它兼任「在线」。**

理由：`probe()` 只执行一条 `printf <LOGIN_MARKER>`（`dzrobot…/src/ssh.js:193-218`），它**根本不知道 ROS 栈和 dzlink 的死活**。而现在两侧 UI 都拿它当在线判据（`dzrobot…/client.js:2175`、`:2181` 用 `probe.state === 'ready'` 决定绿点与文案）。合并后必须把「在线」拆成三条轴，否则同一个绿点会同时声称三件它没验证过的事。

### 1.3 掉线怎么如实表达（★这是本文最重要的一条，且现状已有两个漏洞）

**三条独立链路轴**，各自有独立的时钟与判据：

```jsonc
"links": {
  "ssh":     { "state":"ready",  "at":1792234798226, "staleMs":15000,
               "detail":"免密登录可用" },
  "metrics": { "state":"live",   "at":1792234797000, "staleMs":15000,
               "detail":"CPU 31.5% / 电量 78.0%" },
  "dzlink":  { "state":"online", "at":1792234799000, "staleMs":3000,
               "detail":"lower=K9（情报 4.2s 前，TTL 30s）" }
}
```

- `ssh.staleMs` = `UPTIME_STALE_MS` 的现值 `15_000`（`dzrobot…/client.js:1640`，注释明写「三个 5 秒轮询周期 + 余量」），两侧同值（`dzdrone…/client.js:1610`）。
- `dzlink.staleMs` = **3.0 s**，真源 `config/robot.yaml:840`（`drone.heartbeat_timeout_s: 3.0`）。
  **★ 手册的行号已漂移**：`docs/空地指挥链路_协议与运行手册.md:19` 写的是 `robot.yaml:671`，实测现文件只有 918 行，
  该键在 **`:840-841`**（`drone:` 段，`:838-841`）。这正是手册自己在 §3 事实更正里写下的教训——
  「引用配置一律"键名 + 行号 + md5"三件套，行号漂移时以键名为准」（`:246-247`）。
  ⇒ 本设计**只按键名取值**，不按行号硬编码；下文凡引 `robot.yaml` 均给键名 + 现测行号。
- 情报过期 = **30.0 s**（`robot.yaml:841` `drone.info_stale_s`），即 `dzlink.intel.staleMs`，
  与心跳的 3 s **不是同一个数**——手册 §0 把它们分成两行，合并后也必须分成两个字段。

**★ 漏洞 A（现存，会显示假的绿）：`/status` 返回的是缓存探针，UI 不看它的年龄。**

`GET /status` 的 `probe` 取自持久化快照，**不触发任何探测**：

```
dzrobot…/index.js:156-160   const snapshot = status.get()
                            return send(200, { …, probe: snapshot.probe, … })
```

而 client 挂载时直接把它当现状画：

```
dzrobot…/client.js:2101-2114   store.set({ probe: payload.probe, … })
dzrobot…/client.js:2175        <dot data-state={state.probe?.state ?? 'idle'}>
```

唯一的**活探针**是 120 s 一次的 `POST /probe`（`client.js:2822-2830`）。所以**面板打开后的头两分钟内，点的颜色可能来自上一次进程运行时写入磁盘的探针**。这个字段确实被持久化：本轮实测 `C:\Users\TestFox\.dsh\dzrobot-ssh\status.json` 存着 `{"probe":{"state":"unreachable",…},"at":1792234719285}`，`C:\Users\TestFox\.dsh\dzdrone-ssh\status.json` 存着 `{"probe":{"state":"unreachable","detail":"网络不可达（车还没连上 WiFi？）"},"at":1792234717832}`——两条都是**上一次运行留下的**。

> 讽刺的是，两侧代码的注释早就把这条纪律写出来了：`dzrobot…/client.js:2818-2819`、
> `dzdrone…/client.js:2782-2783`「a stale green dot is the one thing worse than none」。
> 但注释约束的是 120 s 定时器，没约束挂载时读缓存那一次。**纪律已经写对，只是没有覆盖到第二个读点。**

**设计裁决：点的状态从「5 个具名 + 1 个隐式」扩到 7 个，新增的是 `stale`。**

现状清点（准确版）：`STATE_KEY` 只有 **5 个具名态**（`client.js:378-384`：`ready`/`unreachable`/`needs-password`/`host-key`/`error`），
外加 **1 个隐式灰态**——`probe === undefined` 时文案走 `neverProbed`（`client.js:99`），而点的 `data-state` 落成 `'idle'`（`client.js:2175`），
CSS 里 `'idle'` 没有任何规则，于是取 `.dzr_dot` 的默认底色 `--dsw-alias-label-tertiary`（`client.js:389`，灰）。

```
ready         绿    probe.state==='ready' 且 source==='live' 且 age<=staleMs   ← 现有
stale         灰    ★新增：probe.state==='ready' 但（source==='cached' 或 age>staleMs）
                      → 保留「上次是好的」这个事实，但绝不染绿
unreachable   红    现有
needs-password 黄   现有
host-key      黄    现有
error         红    现有
idle/never    灰    现有（隐性，需补一条 CSS 规则让它显式）
```

**为什么 `stale` 必须自成一态、不能并进 `unreachable`**：并进去就是撒谎（车可能只是这台 PC 的探针没跑，网络是好的）；并进 `ready` 就是本文要杀的那个假绿。它必须是一个**同时携带两种事实**的状态：*上次是通的* + *现在不知道*。

并且在 `source==='cached'` 时，节点卡上**必须**打一行小字：`上次探测 12 分钟前`。**不接受「点了绿就先绿着」**。

**★ 漏洞 B（现存，且直接违反机侧自己的设计声明）：飞控板插着但 DDS 静默，会显示「无飞控」。**

机侧 Host 端**已经**把三种「没有数」分开了：

```
dzdrone…/src/metrics.js:244-248
  ...(Number.isFinite(batteryPercent) && Number.isFinite(batteryVoltage)
      ? { battery: { percent: …, voltage: … } }
      : batteryFc === 'present'
        ? { batteryNote: batteryNote ?? 'dds_silent' }
        : {})
```

注释 `:33-34` 与 `:242-243` 明写纪律：**「never 无飞控 about a plugged-in board」**。
但 client 的判决函数**根本没读 `batteryNote`**：

```
dzdrone…/client.js:1663-1667
  const battery = reading.battery
  if (battery != null && …) return { state:'live', … }
  return { state: 'nofc' }          // ← batteryFc=present + dds_silent 也掉进这里
```

于是 header 打出 `无飞控`：

```
dzdrone…/client.js:1991   `${t('floatBattery')}：${batt.state === 'live' ? … : batt.state === 'nofc' ? t('floatNoFc') : '—'}`
dzdrone…/client.js:190    floatNoFc: '无飞控'
```

一个插着的飞控板被标成「无飞控」——这正是 `metrics.js` 两条注释点名禁止的事。**合并时必须修，且修法就是 §2 的结构化缺数**：`battery` 这个 gauge 的 `quality` 与 `reason` 要能同时表达 `present + dds_silent`。

### 1.4 统一节点模型的落点

| 落点 | 复用 | 新增 |
|---|---|---|
| `src/nodes.js` | 讨论稿 §4.1 的 NODES 形状 | `kind`、`caps.gauges`、`links.*.staleMs` 三个字段 |
| `GET /:role/status` | 现 `/status` 原样（`index.js:156-177` 的车版 / `:146-166` 的机版），只把 `car`/`drone` 换成 `nodes[role]` | `probe.at` + `probe.source` 两个字段（漏洞 A 的最小修复面） |
| `GET /status` | — | 新增，`Promise.all` 并发两个节点，**只回缓存 + 各自 `at`**（不退化成同步探测，讨论稿 §8 风险表已列） |

---

## 2. 统一 metrics 契约

### 2.1 现状：两侧同构 90%，分叉全在「怎么表示没有数」

| 维度 | 小车 `src/metrics.js` | 无人机 `src/metrics.js` |
|---|---|---|
| CPU | `/proc/stat` 双采样（`:85-87`） | 同左（`:63-65`），`cpuUsage()` 逐字相同（`:266-275` / `:259-268`） |
| uptime / btime | `/proc/uptime` + `/proc/stat btime`（`:88-91`） | 同左（`:70-73`） |
| 负载 | `/proc/loadavg`（`:92`） | 同左（`:74`） |
| 主指标 | 电量：ROS `/Battery_Percentage`+`/PowerVoltage`（`:67-73`） | 温度：`/sys/class/thermal/thermal_zone*`（`:69`） |
| 电池 | 底盘节点发布 | PX4 `/fmu/out/battery_status`（`:86`） |
| 没有数的词汇 | **`batteryNote=no_stack`**（`:105`）+ **`derived`**（`:225`） | **`batteryFc=present/absent`**（`:209-211`）+ **`dds_silent` / `no_ros`**（`:213-216`） |
| 缺数的**数据结构** | 键**永远存在**，值为 `null`：`battery: null`（`:237`） | 键**整个不存在**：`battery` 缺席（`:244-248`） |
| 超时 | `timeoutMs = 12000`（`:121`，注释解释 ROS CLI 启动 ~1.5 s） | `9000`（`:100`） |
| 阈值常量 | `batteryLevelOf`：≤15 hot / ≤30 warm（`client.js:1616`） | `levelOf(temp, 65, 80)`（`client.js:2008`） |

**「缺数」的两种表示法（键为 null vs 键缺席）是本次合并必须消灭的第一个不一致**：正是这个不一致让机侧 `batteryView` 无法区分「没飞控」与「飞控静默」，也让车侧 `battTip()`（`client.js:1960-1967`）把任何 `battery===null` 都说成「ROS 栈未运行」。

### 2.2 设计：`dzag.metrics/1` —— 每格自带质量与缺席理由

```jsonc
{
  "schema": "dzag.metrics/1",
  "at": 1792234800000,                        // 宿主组装时刻
  "nodes": {
    "car": {
      "id": "car",
      "reading": {                            // 一次 SSH 往返的元信息
        "ok": true,
        "at": 1792234797000,                  // ★读数自身时刻（沿用现 metrics 的 `at`）
        "latencyMs": 1774,
        "target": "duzhong@duzhong.local",
        "timeoutMs": 12000
      },
      "gauges": {
        "batteryPercent": {
          "v": 78.0, "unit": "%",
          "quality": "measured",              // measured | derived | absent
          "reason": null,                     // quality==='absent' 时必填，见 §2.4
          "detail": null,
          "source": "ros:/Battery_Percentage"
        },
        "batteryVoltage": {
          "v": 11.90, "unit": "V", "quality": "measured",
          "reason": null, "detail": null, "source": "ros:/PowerVoltage"
        },
        "cpuPercent":  { "v": 31.5, "unit": "%",  "quality": "measured", "source": "/proc/stat" },
        "uptimeSec":   { "v": 9067, "unit": "s",  "quality": "measured", "source": "/proc/uptime" },
        "bootEpoch":   { "v": 1792225530, "unit": "s", "quality": "measured", "source": "/proc/stat btime" },
        "load":        { "v": [0.42,0.38,0.35], "unit": "1m/5m/15m", "quality": "measured" },
        "temp":        { "v": null, "unit": "°C", "quality": "absent",
                         "reason": "not_supported", "detail": "小车无 thermal 契约（caps.gauges 未声明）" }
      }
    },
    "drone": {
      "id": "drone",
      "reading": { "ok": true, "at": 1792234796000, "latencyMs": 2103,
                   "target": "duzhong@dzsmart.local", "timeoutMs": 9000 },
      "gauges": {
        "temp":        { "v": 61.2, "unit": "°C", "quality": "measured",
                         "source": "thermal_zone:…", "extra": { "at_id": "cpu-thermal", "zones": 9 } },
        "cpuPercent":  { "v": 44.0, "unit": "%", "quality": "measured", "source": "/proc/stat" },
        "uptimeSec":   { "v": 666, "unit": "s", "quality": "measured", "source": "/proc/uptime" },
        "batteryPercent": {
          "v": null, "unit": "%", "quality": "absent",
          "reason": "dds_silent",             // ★飞控在，话题静默
          "detail": "/dev/ttyACM* 存在；ros2 topic echo /fmu/out/battery_status --once 3s 无输出",
          "source": "px4:/fmu/out/battery_status"
        },
        "batteryVoltage": { "v": null, "unit": "V", "quality": "absent",
                            "reason": "dds_silent", "source": "px4:/fmu/out/battery_status" }
      }
    }
  }
}
```

**关键取舍 2：每个 gauge 都是一个对象，不接受裸数字。**

理由不是洁癖，是**现存的实伤**：车侧用「键在、值 null」，机侧用「键不在」，于是 UI 只能二选一地去猜。机侧 `batteryView` 猜错的那一次（§1.3 漏洞 B）就是这么来的——它拿到的信息**不足以做出判决**，任何实现都只能猜。把 `reason` 放进数据里，是让 UI 无猜可猜。

**关键取舍 3：`quality: 'derived'` 必须是独立状态，不许并入 `measured`。**

车侧有一条真实的推算路径：电压话题答了、电量话题没答，就用底盘自己的标定（12 V 满 / 10 V 空）折算，并打 `batteryNote='derived'`：

```
dzrobot…/src/metrics.js:223-226
  if (battery === null && batteryVolt !== null) {
    battery = Math.max(0, Math.min(100, Math.round(((batteryVolt - 10) / 2) * 1000) / 10))
    batteryNote = 'derived'
  }
```

UI 也已经如实区分（`client.js:156` `floatBattDerived: '{v} V · 按 12V满/10V空 折算'`）。
合并后**机侧没有对应概念**（飞控电量是实测 `remaining`），所以 `derived` 是**车侧专属的质量等级**；
契约里必须留着，否则合并时按「取并集」会把这条信息压平成一个假装实测的数字。

### 2.3 车侧 `derived` 之外的第二个陷阱：`normalizeBattery` 的静默拒绝

```
dzrobot…/src/metrics.js:251-257   percent = value <= 1.0001 ? value*100 : value
                                  if (percent > 100.0001) return null
```

`return null` 之后，`battery` 保持 null 而 `batteryNote` **仍是 null**（`:173-175` 初值），
于是 UI 的 `battTip()` 会落进「底盘未发布电量（ROS 栈未运行）」这一句（`client.js:157`、`:1961`）——
**一个数值越界被说成「ROS 栈没起」**。这是同类错误的第三个实例。

⇒ 契约裁决：`reason` 必须覆盖 **`bad_value`**（数值离谱被拒）这个独立原因，与 `no_stack` 分开。
现在两侧都没有这个词，是本次新增；代价为零（parser 里 `return null` 的两个点各加一行）。

### 2.4 统一缺席词汇表（把两套词合成一套）

| `reason` | 含义 | 层级 | 来自 | 现场该做什么 |
|---|---|---|---|---|
| `unreachable` | SSH 连不上（节点整机不在） | 传输 | 两侧 `refusalFor`（`dzrobot…/src/ssh.js:162-182`） | 查 WiFi / 上电 |
| `needs_password` | 能连但登录被拒 | 传输 | 同上 `:174-176` | 修复密钥 |
| `host_key` | 主机密钥变了 | 传输 | 同上 `:177-179` | 确认后清 known_hosts |
| `no_stack` | 机器在，ROS 栈没起 / 话题没发布 | 环境 | 原车侧 `metrics.js:105` | `start_all.sh` |
| `no_ros` | 机器在，但没有 `/opt/ros/humble` | 环境 | 原机侧 `metrics.js:215` | 装/换路径（真源 `:86` 的 `setup.bash` 判定） |
| `no_fc` | 没有飞控板（`/dev/ttyACM*` `ttyUSB*` 全无） | 设备 | 原机侧 `batteryFc=absent`（`metrics.js:86`、`:209-211`） | 插板/查线 |
| `dds_silent` | 板在，但 `/fmu/out/battery_status` 没数据 | 设备 | 原机侧 `metrics.js:215` | 查 uXRCE-DDS agent |
| `bad_value` | 读到了但数值不可信（越界/非数） | 解析 | **新增**，见 §2.3 | 看是不是话题语义变了 |
| `not_supported` | 该节点根本没有这项能力 | 能力 | **新增**，来自 `caps.gauges` | 忽略即可 |
| `no_sample` | 该节点这次 probe 没跑（`ok:false` 且非上述） | 传输 | **新增**（兜底） | 看 `reading.detail` |

**为什么是这十项而不是更少**：因为**每一项对应一种不同的现场动作**。这正是 `ssh.js:155-157` 已经写下的设计理由——
「the same underlying failure … lead the user to four different fixes, so the table lives in one exported place」。
合并只是把这张表从「SSH 层」推广到「所有指标层」，并让两侧共用一张，而不是各写一份。

**为什么不是更多**：不引入 `timeout` / `unknown` 这类无动作的词。`unreachable` 已含超时（`ssh.js:165` 把 `timedOut` 直接映射成 `unreachable`）。

### 2.5 兼容形状（保留旧字段一年内可读）

合并后 `/:role/metrics` 仍返回旧形状的**投影**，供 `float.html`（两侧各自写死 `const API = '/api/dzrobot-ssh/metrics'`，见 `dzrobot…/float.html:157`、`dzdrone…/float.html:157`）平滑过渡：

```jsonc
// 附加在响应尾部，不进 v1 契约
"legacy": {
  "car":   { "ok":true, "cpuPercent":31.5, "uptimeSec":9067, "bootEpoch":1792225530,
             "load":[0.42,0.38,0.35], "battery":78.0, "batteryVolt":11.9, "batteryNote":null },
  "drone": { "ok":true, "cpuPercent":44.0, "uptimeSec":666, "temps":[…],
             "temp":61.2, "tempAt":"cpu-thermal", "zones":9 }   // battery 缺席 = 原语义
}
```

这样 §6「不做清单」第 2 条说「浮窗第一版可以不改 float.html」就不是拍脑袋——它有字段可读。
**注意**：`legacy` 是**只读投影**，任何新 UI 不得依赖它；它只为了让两个 `float.html` 在过渡期不改。

---

## 3. 面板信息架构（3 层，侧栏 1 个图标）

### 3.1 三层结构

```
侧栏：1 个图标「空地协同」  id = dzag-ops          ← 对应讨论稿 §3.1 的 PANEL_ID
└── 面板（main seat，一处注册）
    │
    ├─ ① 总览  Overview（默认落地）
    │   ├ 节点条 ×2：车 / 机
    │   │   点(7 态) · 节点名 · 目标 · 「上次探测 12 分钟前」
    │   │   四格 gauge（按 caps.gauges 决定画几格、画什么）
    │   │   缺数格显示「为什么缺」原文，不显示 0 / 不显示 '—'
    │   ├ 链路轴 ×3（§1.3）：ssh / metrics / dzlink
    │   │   dzlink 行显示 `online:lower=K9` 这类真值（真源 `telemetry.py:184-200`）
    │   └ 协同条：[预检] [跑空地协同仿真] [打开仿真台] [下发路线]
    │
    ├─ ② 单节点操作  Node（点总览的节点卡进入；或 URL `?role=car`）
    │   ├ 终端      ← 复用 stock `kind:'terminal'`（机侧 `TOOLS` 的 terminal 行，`dzdrone…/client.js:2047`）
    │   ├ 文件      ← 复用 `makeFilesTabShell`（`dzrobot…/client.js:1524-1546`）
    │   ├ VNC       ← 复用 `makeVncTabShell` → `makeVncConnectorPanel`（`client.js:1494-1522`、`:2324`）
    │   ├ 浮窗       ← 复用 store 的 `floatOpen` 开关（`client.js:2132-2140`；翻转在 `:2135-2137`，关闭在 `:1856`）
    │   └ 仿真台     ← **只有 car 有**（`mode:'external'`，`client.js:2141-2156`）
    │
    └─ ③ 协同  Coordination
        ├ 仿真台（/sim.html：64 图压测 + 空地路线图 + droneRunAudit 校验）
        ├ 实时链路（三条轴的时序：心跳 / 情报 / ack）
        └ 双证人对比（§5 功能 1）
```

### 3.2 为什么是「总览 → 单节点 → 协同」，而不是三个平级 tab

取舍理由（只有一条）：**协同层的信息必须建立在「两个节点各自的真相」之上，所以它天然是第三层，不是第一层。** 若把「协同」放在第一屏，用户看到的是 `lower=K9` 这种情报，却不知道车这一侧到底有没有收到——而这恰恰是现场最容易误判的地方（手册 §2.2 `:99`：「`ack` 是"车真的收到了"的唯一硬证据」，BEST_EFFORT 链路上"发出去了"≠"收到了"）。

### 3.3 `TOOLS` 表怎么合并：加一列，不加一张表

两侧的 `TOOLS` 只差一行：

```
dzrobot…/client.js:2047-2056      5 行：terminal / files / vnc / float / **sim**
dzdrone…/client.js:2046-2051      4 行：terminal / files / vnc / float
```

⇒ **合并后保留一张表，每行加 `roles: ['car','drone']`，`sim` 行只写 `['car']`。**

```js
const TOOLS = [
  { id:'terminal', kind:'terminal', mode:'terminal', roles:['car','drone'], … },
  { id:'files',    kind:'dzr-files', mode:'tab',    roles:['car','drone'], … },
  { id:'vnc',      kind:'dzr-vnc',   mode:'tab',    roles:['car','drone'], … },
  { id:'float',    kind:null,        mode:'float',  roles:['car','drone'], … },
  { id:'sim',      kind:null,        mode:'external', roles:['car'],       … },
]
```

渲染时 `TOOLS.filter(t => t.roles.includes(role))`。**不新增「节点 → 工具表」的第二层映射**：第二层映射是讨论稿 §3.6 想省的那 2,400 行里的主要来源，而这里一张表 + 一列就够。

### 3.4 复用清单（明确到函数）

| 复用物 | 现位置（车侧行号；机侧同构偏移 ≤ 40 行） | 合并动作 |
|---|---|---|
| `createStore()` / `useStore()` | `client.js:808-831` | 一份，`value` 里多一个 `role` 与 `nodes` |
| `api(path)` | `client.js:552-562` | 一份，`API` 常量换成新前缀 |
| `connect()` / `openToolTab()` | `client.js:728`、`:704`（`ensureConversation` `:641`） | 一份，`kind` 由 TOOLS 行给 |
| `makeSettingsPage` | `client.js:849`（`makeVncView` `:1090`、`makeFilesView` `:1323`） | 一份，表单分两段（car / drone），`sanitize` 加白名单 |
| `makeVncConnectorPanel` | `client.js:2324` | 一份。**★机侧没有改进版，取车侧**（车侧带 `VNC_RESOLUTIONS`/`CONNECTOR_ID`/`bridgeConnectorConfig`，`client.js:2235-2293`；机侧 `CONNECTOR_ID` 在 `dzdrone…/client.js:2202`，等价但少了 `detectRuntime` 之外的车侧细节） |
| `makeFloatWindow` | `client.js:1772-2034`（`FloatWindow` 收尾在 `:2033-2034`） | 一份，改造成 §4 的双节点卡 |
| `CarGlyph` / `DroneGlyph` | `client.js:836`、机侧同位置 | 都留，按 `role` 选 |
| `ToolGlyph` | `client.js:2059-2084` | 一份，`sim` 字形保留 |
| 6 个 `test-sim-*.mjs` 的 `dzr_*` 断言 | `scripts/` | **CSS 前缀沿用 `dzr_`**，测试不用改（讨论稿 §6.5 已指出这个实惠） |

### 3.5 `sim.html` 的迁移是硬依赖，不能拖到 S3

`sim.html` 里 API 前缀是**写死**的：

```
dzrobot…/sim.html:420-421
  /* 页面与 API 同源（本文件就是插件 http 源上的 /api/dzrobot-ssh/sim.html）⇒ 相对路径即可。 */
  var API = '/api/dzrobot-ssh';
```

同理客户端打开仿真台时拼的路径也写死：

```
dzrobot…/client.js:2146   const path = '/api/dzrobot-ssh/sim.html'
```

⇒ 合并后这两处**必须同批改**，且 `src/simbench.js` 的根解析依赖 `sim-root.json`（讨论稿 §8 风险表已列）。
**裁决：把 `API` 从写死改成从 `document.currentScript.src` 推导，消灭这个耦合**，否则下一次改名还要再踩一遍。

---

## 4. 浮窗合并

### 4.1 现状：两块卡，且**同一件事放在两处**

| | 小车浮窗 | 无人机浮窗 |
|---|---|---|
| 标题 | `小车状态`（`client.js:149`、`:1988`） | `无人机状态`（`dzdrone…/client.js:185`、`:1978`） |
| header 第二段 | `开机时间：m:ss`（`:1994`，块 `:1987-1995`） | 同左（`:1984`，块 `:1977-1992`）**＋电量百分比**（`:1985-1991`） |
| body 两格 | `电量` + `CPU`（`:2005-2015`） | `温度` + `CPU`（`:2002-2014`） |
| 图表 y 轴 | 恒 ±10 %，clamp 0..100（`:1700-1701`） | 温度 ±5 °C、CPU ±10 %（`:1700-1702`） |
| 刻度 | 5 %（`:1702`） | 2.5 °C / 5 %（`:1702`） |
| 位置持久化 | `localStorage['dsh-dzrobot-ssh.float.pos']`（`:1561`） | `localStorage['dsh-dzdrone-ssh.float.pos']`（`dzdrone…:1539`） |
| 开关持久化 | `…float.open`（`:1576`） | 同构（`:1554`） |
| 注册 | `shell.overlay` id `dzrobot-float` order 40（`:2785-2792`） | id `dzdrone-float` order 40（`dzdrone…:2749-2756`） |

**注意「order 40」两侧相同**——现在两块卡若同时打开，在同一个 overlay 层里抢同一个层级序。这是「两块卡会互相遮挡」的结构性原因，不是运气。

### 4.2 裁决：**一张卡、N 格、按节点分组的行内网格**（不是页签）

```
┌─ 空地状态 ────────────────────────────── 小车 2:31:07 · 无人机 11:06 ─┐
│  ● 小车  电量 78.0% ▁▂▃▅   CPU 31.5% ▁▁▂▂     ← 两格并行（沿用现车卡布局）│
│  ● 无人机 温度 61.2°C ▂▃▅▆  CPU 44.0% ▁▂▃▄    ← 两格并行（沿用现机卡布局）│
│  ○ 无人机 飞控在 · DDS 静默（不是「无飞控」）                          │
├──────────────────────────────────────────────────────────────────────┤
│ 同步 18:20:03 ·  [透明度 ────●]  [↻]                                    │
└──────────────────────────────────────────────────────────────────────┘
```

**取舍理由（三条，都是硬理由）**：

1. **不能共用一张图**。车的 y 轴是 `%` 且必须 clamp 0..100（`dzrobot…:1700-1701`，注释：「a car sitting at 95 % has no +110 tick to draw」），机的温度轴是无界 `°C` 且步长 2.5（`dzdrone…:1700-1702`，注释：「温度 current ±5 °C」）。把两条曲线塞进一个坐标系必须选一个轴语义，选谁都是对另一个撒谎。
   ⇒ **保留每格独立的 `chartModel(kind, …)`，只合并容器。** `chartModel` 现签名已支持 `kind`（`dzrobot…:1698`），改造量为 0。
2. **不能用页签**。页签把另一个节点藏起来，而浮窗存在的全部意义就是「在忙别的事的时候还能看见」（`dzrobot…/client.js:1753-1757` 注释原话：*the whole point is to keep watching the car's battery while working somewhere else*）。页签等于把这条理由删掉。
3. **header 只放身份，不放指标**。现机侧把电量塞进 header（`:1991`）、车侧把电量放 body（`:2006`）。合并后统一：**header = 我是谁 + 我活了多久 + 我这条链路什么状态；body = 指标格。** 理由是 header 是一行、可拖动（`startDrag` 绑在 header，`:1974-1982`），塞进会变的数字会让人误点到数字上。

**掉线时怎么画（与 §1.3 同一口径）**：该节点的格子**变灰但保留最后一次数值 + 打上「12 分钟前」**，绝不归零、绝不隐藏。
理由：`chartModel` 在 `current` 非有限时返回 `null` 并画虚线空框（`dzrobot…:1698-1699`、`:1907-1911`）——这是「没有数」的正确画法，保留。

### 4.3 位置与开关的迁移

三个 localStorage 键必须**迁移或重新声明**，否则用户拖过的位置会跳回默认（现 `readFloatPos` 的 fallback 在 `dzrobot…:1555-1559`）：

```
旧: dsh-dzrobot-ssh.float.pos  ─┐
旧: dsh-dzdrone-ssh.float.pos  ─┼→ 新: dsh-dzag.float.pos     （取"曾打开过"的那一侧）
旧: dsh-dzrobot-ssh.float.open ─┐
旧: dsh-dzdrone-ssh.float.open ─┴→ 新: dsh-dzag.float.open    （或：任一为 '1' 则开）
```

只读一次、写新键、**不删旧键**（回滚要能回去）。

---

## 5. 「合并后才可能做」的真协同功能

判定标准（我自己用的、写在这里防止自我膨胀）：**删掉任一节点的数据，这个功能就产生不了该结论。** 只把两块面板并排不算。

### 功能 1 · 双证人链路对账（★最值得先做）

| 项 | 内容 |
|---|---|
| 做什么 | 把「无人机说自己发了」与「车说自己收了」并排对账，直接给出 `已闭环 / 单向 / 全断` |
| 节点 A 数据 | 机侧 `~/dzlink_logs/supervisor.jsonl` 的 `intel_emit` 行（`dzlink_supervisor.py:1191`：`log.log("intel_emit", type, slot, text, seq, …)`）与 `car_ack` 行（`:1094`：`log.log("car_ack", of=of, matched=matched, …)`） |
| 节点 B 数据 | 车侧 `~/robot_logs/7_path.log` 的遥测行（`start_all.sh:18,55,219-220` 决定落盘名）里的 `无人机…` 段，真源 `telemetry.py:184-200` 的 `_drone_text()`：`offline` / `online` / `online:lower=K9` / `!过期` / `!差异N` |
| 为什么只有合并能做 | 这两个文件**在两台不同的机器上**，且必须由同一个进程同时持有两条 SSH 通道才能对齐时间轴。现在要人肉开两个终端、手工比对 seq |
| 判定表 | 机 `intel_emit` 有 + 机 `car_ack.matched=true` + 车 `online:lower=K9` ⇒ **闭环**；机发了、机侧 ack 未匹配 ⇒ **车侧没收到**（BEST_EFFORT 丢了，手册 §2.2 `:99` 的原话："`ack` 是'车真的收到了'的唯一硬证据"）；车侧 `offline` ⇒ **心跳断了**（与情报无关，手册 §6 `:314` 的排障表就是这么分的） |
| 最小实现代价 | 一条 `tail` 端点（复用 `src/files.js` 的 SSH 读取范式：该模块的 `remote()` + `base64 -w0` 在 `:241-249`；**注意** `readTextFile` 有 2 MiB 上限 `files.js:27`、超限直接拒绝 `:245`，而 2 Hz 遥测行（`telemetry.py:222`）会迅速超限 ⇒ **必须用 `tail -c N` 而不是整读**）+ 两个 JSON/正则解析器 + 一张三态表 ≈ **200 行**，零后端协议改动 |
| ⚠ 诚实限制 | `intel_emit` 是**决策时刻的簿记，不是发布成功的回执**。证据：`dzlink_supervisor.py:1191-1192` 写日志，`:1193` 才 `out.append(ev)`；真正的 ROS 发布在 `tick()` 返回之后由调用方执行。所以「机侧发了」这一半的证据强度是**日志级、不是链路级**；判「车侧没收到」时必须以 `car_ack.matched=false`（`:1094`）为主证据，`intel_emit` 只作时间锚点。**这条限制必须写进 UI 的 tooltip，否则功能 1 自己会变成一个新的假绿** |

### 功能 2 · 桥洞情报时间线（把「等待」画出来）

| 项 | 内容 |
|---|---|
| 做什么 | 一条时间轴同时标出：无人机何时发 `bridge_passable`、车何时收到、车在桥前 HOLD 等了多久、换洞发生在第几秒 |
| 数据来源 | 机：`intel_emit`（同上）。车：`mission.py` 的 `bridge.note.hold = {active, cap_s, waited_s, half, reroutes}`（手册 `:244` 明写这是遥测可见字段），以及 `drone.py:619-620` 的「桥洞情报：lower 半场 → K9（置信度 0.90，第 N 次）」日志行 |
| 为什么只有合并能做 | 「等了 2 秒没等到配对半场」这个事实，**只有当你能同时看到两半场的下发时刻**才有意义。单看车侧只知道「没等到」，不知道「对面到底发了没有、第几秒发的」 |
| 最小实现代价 | 在功能 1 的解析器上加一个时间轴组件 ≈ **80 行**（复用 `sim.html` 已有的 `renderDroneTimeline` 视觉，`sim.html:2049-2082`，落点 `#droneTimeline` 在 `:365`） |

### 功能 3 · 仿真预测 vs 实跑复核（闭环「平行度」）

| 项 | 内容 |
|---|---|
| 做什么 | 用 `/sim/drone/run` 的台账（`dronesim.js:359-367`：`prior` / `truth` / `reroutes` / `confirmed` / `parallelS`）当**预测值**，用实跑的两侧日志当**实测值**，自动给出「预测 2 次全量重规划 / 实际 1 次」这种差异 |
| 数据来源 | 仿真侧：`src/dronesim.js:359-381` 的 `meta` 与 `scan`/`intel`（`index.js:372-383` 原样透传）。实测侧：功能 1 的两条日志 |
| 为什么只有合并能做 | 仿真台账在**车侧插件**里（`/sim/drone/run`），实跑日志在**机侧**。现在这两个世界之间没有通道——人只能记住「仿真说会重规划 2 次」，然后去猜实跑重规划了几次 |
| 最小实现代价 | 一个 diff 视图，两个 JSON 对字段 ≈ **100 行**。注意 `parallelS` 用 `Math.min(finishT, landT)` 估算（`dronesim.js:366`），实跑侧要取同类口径，否则这条对比会系统性偏差 |

### 功能 4 · 起降格避让的「实车侧闭合」（把仿真里的审计搬到实跑）

| 项 | 内容 |
|---|---|
| 做什么 | 仿真里已经有一条三态审计 `droneRunAudit()`（`sim.html:1848-1879`：ok / bad / blind），它比对的是**真源起降格**（`FIELD.drone`）与**台账 `record.plan.drone`**（`{enabled, hardAvoid, corridorRow}`，`simbench.js:875-884`）。把它推广到实跑：读实跑 `record.plan.drone`，同样给出三态 |
| 数据来源 | 车侧仿真台账 `record.plan.drone`（`simbench.js:875-884`）；真源起降格 `fieldSpec().drone`（`simbench.js:250-254`，`DRONE_FALLBACK={takeoff:'N8', land:'M2'}` 在 `:99`） |
| 为什么只有合并能做 | 因为**机侧的起降格信息在这份审计里是「真源」那一边**。现在审计只能证明「仿真规划避开了」，无法证明「实车那一局也避开了」 |
| 最小实现代价 | 复用 `droneRunAudit` 的判据，把输入从内存 `FIELD` 换成读到的台账 ≈ **60 行**（这个函数是纯函数，`sim.html:1848` 起，零依赖） |
| ⚠ 诚实的限制 | `corridorRow` 这条软代价加权**仍推迟在车侧**（`simbench.js:321`、`:346` 注释明写「规划代价加权推迟」），所以审计里 `corridorRow` 只能核对**字段齐不齐**，不能核对**实际生效**。这一点必须在 UI 上说出来，否则又是一个假绿 |

### 功能 5 · 单点故障自动归因（现场最省时间的那条）

| 项 | 内容 |
|---|---|
| 做什么 | 组合三轴（§1.3）直接给结论，而不是让人看三行红字自己推 |
| 规则示例 | 车 `ssh=ready` + 车 `metrics=absent/no_stack` ⇒ **车在，ROS 栈没起**（该跑 `start_all.sh`，不是查 WiFi）；机 `ssh=ready` + 机 `battery=absent/dds_silent` ⇒ **板在、DDS agent 没起**；两节点 `ssh=ready` + 车 `dzlink=offline` > 3 s ⇒ **B 通道断，按手册 §1 `:61` 的降级设计仍能完赛**（不是停车故障） |
| 数据来源 | 全部来自 §1.3 的三轴 + §2.4 的 `reason`，**没有一项是新采集** |
| 为什么只有合并能做 | 归因的本质是**跨节点对偶**：只有在「机侧一切正常」时，才能断定车速慢的原因在车侧。单面板只能看到自己那一半证据 |
| 最小实现代价 | 一张 if-else 表 ≈ **50 行**，纯前端。**这是投入产出比最高的一条** |

> **推荐先做 1 与 5**：功能 1 提供唯一真实证据链（ack），功能 5 是纯前端规则表、能立刻把「三段红字」变成一句人话。两者都不需要动协议、不需要新采集。

---

## 6. 明确不做（防范围膨胀）

| # | 不做 | 为什么 | 若将来要做，前提是什么 |
|---|---|---|---|
| 1 | ❌ 不改 `/dzlink/*` 与 `/nav/waypoint` 的**任何字段** | 协议真源 `docs/空地指挥链路_协议与运行手册.md` §2（`:66-114`）；且手册 `:70` 明写「新增字段安全、**新增类型不安全**」 | 先改协议手册 + 两端同步发布 |
| 2 | ❌ 不做 WebSocket / SSE 实时推送 | 现役 120 s 轮询（`client.js:2822-2830`）够用；浮窗已经是 5 s 轮询（`client.js:1811`） | 功能 1/2 的时间轴若卡到不可用，再改成 2 s 轮询，不上 WS |
| 3 | ❌ 不合并两个节点的**交互式 SSH 会话** | DSH 原生终端承载（`kind:'terminal'`，`client.js:2048`），一条会话绑一个 target | 不打算做 |
| 4 | ❌ 不做 >2 节点 | 讨论稿 §9 已定；`caps`/`role` 留了口子但不做多机 UI | 真出现第三台车 |
| 5 | ❌ 不重写/不升级 noVNC | 两侧 58 文件 570.1 KB **逐字节相同**（讨论稿 §1.2，`core/rfb.js` 哈希已验） | 需要新版协议时单独评估 |
| 6 | ❌ 不碰 `mission.py` / `drone.py` / `dzlink_supervisor.py` | 车端机端算法与协议栈，超出「插件合并」 | 赛题需求变更 |
| 7 | ❌ 不做「健康度」合成指标（把电量温度 CPU 折成一个分） | 讨论稿 §6.1 选项 C；会丢掉现场全部可动作信息 | 只在赛后复盘视图里做，不进主面板 |
| 8 | ❌ 不给车补温度 / 不给机补 ROS 电量以外的传感器 | `not_supported` 是诚实答案，不是待填的坑 | 硬件真的加了传感器 |
| 9 | ❌ 不做旧 API 前缀的 301/转发兼容层 | 唯一消费者是 `sim.html`（`sim.html:421` 写死），我们自己改 | 出现第三方脚本调旧前缀 |
| 10 | ❌ 不在本轮实现 `corridorRow` 的软代价加权 | 车侧明确推迟（`simbench.js:321`、`:346`）；审计只核字段存在性并如实标注 | 车侧 planner 落地后 |
| 11 | ❌ 不做「一键下发路线」的自动化（本卡只画按钮） | 讨论稿 §5 功能 3 属实现范畴；且它需要 `dz_send_waypoint.py` 的 `--wait-match` 语义配合（手册 `:112-114`） | S3 之后单独开卡 |

---

## 7. 与讨论稿的分歧与更正

| # | 讨论稿 | 本文 | 依据 |
|---|---|---|---|
| ★1 | §4.1「禁止用上次的值假装在线——UI 必须比对 `at` 时间戳」 | 同意，但必须**同时**引入第 7 态 `stale` 与 `probe.source: cached\|live`，否则「比对 `at`」没有落点：现 UI 只有 5 个具名 `STATE_KEY`（`client.js:378-384`）+ 1 个隐式 `idle`，没有「绿但旧」这一格 | 漏洞 A（§1.3） |
| ★2 | §4.2 的缺席词汇表有 6 项（`unreachable`/`no_stack`/`no_ros`/`dds_silent`/`no_fc`/`not_supported`） | 扩到 **10 项**，补 `needs_password`、`host_key`、`bad_value`、`no_sample` | `bad_value` 有实证（§2.3）；前两项已在 `ssh.js:174-179` 存在，漏掉会让「登录被拒」和「整机不在」又说成一句 |
| ★3 | §4.2 的 drone 示例用 `absent: { all: {...} }`（整节点缺席） | 改为**逐 gauge 缺席**（§2.2） | 漏洞 B：整节点级缺席无法表达「温度有、电量无」这种最常见状态 |
| ★4 | §4.2 未提 `derived` | 必须作为独立 `quality` 保留 | `metrics.js:223-226` + `client.js:156/1964-1966` 都在用 |
| ★5 | §3.4 浮窗「一张卡、两列」且示例 header 只有车/机两列 | 同结构，但 **N 格 + header 只放身份** | §4.2 三条理由（y 轴语义不同 / 页签违背浮窗目的 / header 可拖动） |
| ★6 | §5 功能 2「情报直接落图」 | **降级为可选**，本文的主力是**功能 1 双证人 + 功能 5 归因** | 「落图」只是把仿真图搬到网页，缺 `ack` 闭环证据（手册 `:99`）；闭环才是「合并后才可能」 |
| ★7 | §6.5 建议 CSS 前缀 `dza_` | **取 `dzr_`**（沿用车的） | 6 个 `test-sim-*.mjs` + `test-client.mjs` 里全是 `dzr_*` 断言，沿用即零改测试。中性命名不值这些改动量 |
| ★8 | §6.3 把浮窗列成「三选一待拍板」 | **不必再问**：y 轴语义不同已经排除了「共用一张图」（§4.2 理由 1），浮窗的目的已经排除了页签（理由 2）⇒ 只剩「一张卡 N 格」一个可行解 | 本节 ★5 + §4.2 |
| ★9 | （手册）§0 把心跳/情报引到 `robot.yaml:671-672` | 现测为 `:840-841` | 本卡实测 `grep -n` 结果；手册自己在 §3 `:246-247` 已写明「行号漂移以键名为准」。**建议 lead 顺手订正手册**，否则每次引用都在传抄一个错行号 |

---

## 8. 未验证 / 需要前置条件

| 项 | 状态 | 缺什么证据 |
|---|---|---|
| 合并后包名 `dsh-dzairground-ssh` | 讨论稿建议，用户未拍板 | 用户回答（讨论稿 §10） |
| 两侧实时 `/status` 当下都是 200 | **未验证**（本卡只读了落盘的 `status.json`，两条都是 `unreachable`） | 一次实跑（讨论稿 §1.8 记的是另一时刻的 `ready`） |
| `~/robot_logs/7_path.log` 的实际增长速度与大小 | **未验证** | 车在网时 `stat` 一次；这决定功能 1 的 `tail -c` 取多少字节 |
| `~/dzlink_logs/supervisor.jsonl` 现存量 | **未验证**（`default_log_path()` 在 `dzlink_supervisor.py:215-218`，非 Windows 返回 `~/dzlink_logs/supervisor.jsonl`） | 机在网时 `stat` 一次 |
| `dzlink` 三轴中的 `dzlink` 段能否**从插件侧**读到 | **未验证**。插件里**零** `dzlink` 字样（两侧 `grep -r dzlink` 均 0 命中），车辆状态只能靠读日志文件 | 需要确认车侧遥测行确实落盘且可 tail（`telemetry.py:263` `self._log(line)` → stdout → `start_all.sh:55`） |
| 机侧 `batteryFc` 实测值 | **未验证**（现有 `tmp-fc-*.sh` 三个脚本在 `dzsmartscout/dsh-plugin-dzdrone-ssh/`，是当时的探针副本） | 板插上后跑一次 `/metrics` |

---

## 9. 附录 · 引用索引

| 结论 | 证据位置 |
|---|---|
| 两插件同构、仅字符串不同 | `dzrobot…/client.js:33-46` vs `dzdrone…/client.js:36-45` |
| `/status` 返回**缓存**探针 | `dzrobot…/index.js:156-160`（`status.get()`）、`dzdrone…/index.js:146-151` |
| 挂载时直接画缓存探针 | `dzrobot…/client.js:2101-2114`、`:2175`、`:2181` |
| 120 s 才刷新一次点 | `dzrobot…/client.js:2822-2830`、`dzdrone…/client.js:2786-2794` |
| `status.json` 确实持久化探针 | 本轮实测 `C:\Users\TestFox\.dsh\dzrobot-ssh\status.json`、`…\dzdrone-ssh\status.json` |
| 5 具名态 + 1 隐式态的定义 | `dzrobot…/client.js:378-384`（`STATE_KEY`）、文案 `:93-97`、隐式 `idle` 的落点 `:2175`、无 `idle` 规则的 CSS `:389` |
| SSH 拒绝的 5 种状态 | `dzrobot…/src/ssh.js:162-182`；`probe()` `:193-218` |
| 车 metrics 的 `no_stack` | `dzrobot…/src/metrics.js:105`、`:237-239` |
| 车 metrics 的 `derived` | `dzrobot…/src/metrics.js:223-226`；UI `client.js:156`、`:1964-1966` |
| `normalizeBattery` 静默拒绝 | `dzrobot…/src/metrics.js:251-257`（`:255` `return null`） |
| 机 metrics 的 `batteryFc`/`batteryNote` | `dzdrone…/src/metrics.js:209-217`、`:244-248`；shell 判定 `:86` |
| **机侧「无飞控」误报** | `dzdrone…/client.js:1663-1667`（不读 `batteryNote`）+ `:1991` + `:190`；与 `metrics.js:33-34`、`:242-243` 的设计声明冲突 |
| 浮窗两格与 y 轴语义 | `dzrobot…/client.js:1698-1702`、`:2005-2016`；`dzdrone…/client.js:1697-1702`、`:2002-2015` |
| 浮窗 header 只有身份 + uptime | `dzrobot…/client.js:1987-1995`；机侧多塞了电量 `:1985-1991` |
| 浮窗持久化键 | `dzrobot…/client.js:1554-1581`；`dzdrone…/client.js:1532-1559` |
| 两块 overlay 同 order 40 | `dzrobot…/client.js:2785-2792`；`dzdrone…/client.js:2749-2756` |
| `TOOLS` 只差 sim 一行 | `dzrobot…/client.js:2047-2056`；`dzdrone…/client.js:2046-2051` |
| `mode:'external'` 拼 sim.html URL | `dzrobot…/client.js:2141-2156`（`:2146` 写死路径） |
| `sim.html` 写死 API 前缀 | `dzrobot…/sim.html:420-421` |
| `droneRunAudit` 三态 | `dzrobot…/sim.html:1848-1879` |
| 起降格真源 + 台账 | `src/simbench.js:250-254`、`:99`、`:875-884`；`corridorRow` 推迟 `:321`、`:346` |
| 空地协同仿真台账字段 | `src/dronesim.js:359-367`、`:369-381` |
| 心跳 3.0 s / 情报 30.0 s | **现测** `config/robot.yaml:840-841`（`drone.heartbeat_timeout_s` / `drone.info_stale_s`，`:838` 的 `drone:` 段）。★手册 `:19-20` 引的 `:671-672` **已漂移**（文件现 918 行） |
| `ack` 是唯一硬证据 | 手册 §2.2 `:99-101` |
| B 通道不是完赛必要条件 | 手册 §1 `:61`（设计铁律） |
| 车侧遥测行含无人机状态 | `src/dzbrain/dzbrain/tasks/telemetry.py:184-200`（`_drone_text`）、`:73-93`（`render_line`，`:91` 输出 `无人机{tel.drone}`） |
| 车侧遥测落盘路径 | `start_all.sh:18`（`LOGDIR=$HOME/robot_logs`）、`:55`（`setsid … > $LOGDIR/${name}.log`）、`:219-220`（`spawn 7_path … python3 -u -m dzbrain.main`） |
| 2 Hz 遥测 | `telemetry.py:222`（`period_s` 默认 0.5） |
| 机侧 JSONL 日志与字段 | `dzlink_supervisor.py:889-921`（`JsonlLogger`）、`:1191`（`intel_emit`）、`:1094`（`car_ack`）、`:1197-1220`（`status()`）、`:215-218`（默认路径） |
| 桥洞情报接收侧日志 | `src/dzbrain/dzbrain/tasks/drone.py:604-620`（`:619-620` 打印「桥洞情报：lower 半场 → K9」） |
| `bridge_of()` 的 30 s 过期判据 | `core/state.py:115-121` |
| 文件层 2 MiB 上限 | `src/files.js:27`（`MAX_READ_BYTES`）、`:245`（超限拒绝） |
| 文件层 SSH 读取范式 | `src/files.js:241-249`（`remote(settings, 'base64 -w0 -- …')`） |
