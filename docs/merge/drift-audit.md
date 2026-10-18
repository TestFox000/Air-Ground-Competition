# 机侧插件漂移审计：谁领先谁

任务：t2 · task-2　｜　审计人：drift-auditor　｜　审计时间：2026-10-17（本地时区 Asia/Shanghai）

**输入（全程只读，未修改任何插件文件、未安装、未部署、未跑 pnpm）**

- 车侧：`D:\User\Desktop\比赛存档\v1\dzrobot_v2\dsh-plugin-dzrobot-ssh\`（git HEAD `0264159`，工作区对该目录**干净**，见 §5.1）
- 机侧：`D:\User\Desktop\比赛存档\v1\dzsmartscout\dsh-plugin-dzdrone-ssh\`（**无版本控制**，`Test-Path .git` = False）
- 已装副本：`C:\Users\TestFox\.dsh\profiles\desktop\node_modules\dsh-dzrobot-ssh` / `dsh-dzdrone-ssh`
- 改名脚本：`D:\User\Desktop\比赛存档\v1\dzsmartscout\adapt_drone_plugin.mjs`（20 条规则，:12-41）

**口径声明（重要，读表前必看）**

| 名词 | 定义 | 复核命令 |
|---|---|---|
| **原样 diff** | 不改任何字节，直接 `git diff --no-index` 车侧 vs 机侧 | §5.2 |
| **改名归一 diff** | 把 `adapt_drone_plugin.mjs` 的 20 条规则**按原顺序**重放到车侧文件，再与机侧比 | §5.2 |
| **代码-only diff** | 在改名归一基础上**再剥掉注释与空行**，只剩可执行差异 | §5.3 |
| 「机侧改进」 | 机侧有、车侧没有，且是**真能力**（不是文案） | 表中「性质」列 |

---

## 0 结论（8 行）

1. **合并会丢功能，但丢的是车侧的，不是机侧的。** 机侧相对车侧**净减**一个完整能力域（离线仿真台 + 50 图压测 + 空地协同仿真，共 **12 条 `/sim*` 路由 + 5 个 Host 模块文件**），机侧**没有**任何一项车侧不存在且车侧也会想要的核心能力。
2. **机侧的真实增量只有 4 项**：① `metrics.js` 换成温度 + PX4 飞控电池；② `float.html` 从 3 格变 4 格（温度是新增，电量位置重排）；③ `batteryView()` 三态判据（`live`/`nofc`/`down`）；④ `bridgeConnectorConfig()` 的 loopback→`localhost` 改写。此外还有 4 项小改进（§2.3 Q1）。**53 个 hunk 里 8 个是纯文案/注释**，剩下的大半是改名残留与车侧功能被删的副产品。
3. **`metrics.js` 不是「严格超集」，是「换了一套」。** 机侧确实**同时**保留了 PX4 电池（`battery.percent/voltage`），但它**删掉了**车侧的 `batteryNote='no_stack'` 语义、`batteryVolt` 电平键、`normalizeBattery()` 导出与 12 V/10 V 折算。两侧**同名键 `battery` 类型不同**（车 `number|null`，机 `{percent,voltage}|missing`）——这是本次合并**唯一必须人工设计契约**的地方。
4. **机侧有一个必须修的真 bug（本轮新发现，非本任务原始 5 问之一）**：`src/metrics.js` 精心设计了 `batteryFc=present` + `batteryNote=dds_silent|no_ros` 三态，**但 UI 层一处都不读**。`client.js` 与 `float.html` 对 `batteryNote`/`dds_silent`/`no_ros`/`飞控无数据` 的引用次数**均为 0**（§5.6 证据），于是「飞控插着但 DDS 静默」会被渲染成 `无飞控`——正是该文件注释 :34 明令禁止的谎报。
5. **机侧测试也净减**：`test-host.mjs` 丢了车侧的 `normalizeBattery` 8 项参数化用例与 `cpuUsage` 2 项用例（机侧 `test-host.mjs:144` 只 import 了 `parseMetrics`），却新增 12 项 PX4 解析用例。合并后必须**两边用例都留**。
6. **稳定组件身份（stable-identity）不是车侧后加的功能**——它在 2026-10-07 的 fork 基线 `667a8be` 里就有（§5.5）。机侧是**后来主动删掉**了这层保护，属于**回退**，不是「机侧没赶上」。
7. **`vnc-app.html` / `vnc-connect.html` 两侧行数相同、哈希不同，差异是「中文文案」**：`adapt_drone_plugin.mjs` 的 `小车→无人机` 规则**命不中「这台车」**（缺「小」字），所以重放规则后车侧 `vnc-app.html:47,164` 仍是「这台车的 VNC 需要密码」；**机侧已手工改成「这台无人机」**（`:47,164`），属于**超出规则的机侧修补**。另有 2 处 JS 注释「the car's desktop」机侧已改成「the drone's」（`:139,223`）。**不是能力差异，且方向是机侧更干净。**
8. **`float.html` 机侧多 10 行,但那是「加两块、砍两块」的净和,不是纯增量。** 逐 hunk 对账（15 个 hunk,§2.3 Q5 与 §5.10 可复算）：机侧独有 50 行 = 温度格(HTML+渲染+i18n+赋值) + 电池三态渲染 +17；车侧独有 40 行 = **电量进度条 `<span class="bar"><i id="batteryBar">`（车 `:134`）**、**电量反向阈值 `battLevel()`（车 `:188`）**、**折算 tooltip 三条 i18n（车 `:167-169`）**。机侧电量格只剩 `value`+`sub`（机 `:143-147`）。`cpuBar` 两侧都还在（机 `:135` / 车 `:140`）——**是电量进度条单点丢失,不是整套进度条被删**。

---

## 1 关键裁决：合并会不会丢功能

**会。而且是单向的——只有车侧会丢。**

| 能力 | 车侧 | 机侧 | 若以机侧为底 |
|---|---|---|---|
| 离线仿真台 `/sim.html` + `/sim/*` 11 条 | ✅ | ❌ | **全丢** |
| 50 图压测 `/sim/stress*` 3 条 | ✅ | ❌ | **全丢** |
| 空地协同仿真 `/sim/drone/run`、`/sim/drone/history` | ✅ | ❌ | **全丢** |
| 温度监控（thermal_zone，多测点 + 峰值归因） | ❌ | ✅ | 车侧补：**可用** |
| PX4 飞控电池（`/fmu/out/battery_status`） | ❌ | ✅ | 车侧补：**可用** |
| 电量进度条（`#batteryBar`） | ✅ | ❌ | **丢** |
| `normalizeBattery()` 两种电量标度容错 + 8 项测试 | ✅ | ❌ | **丢** |
| `cpuUsage()` 2 项单测 | ✅ | ❌ | **丢** |
| 稳定组件身份（防 VNC 反复 remount） | ✅ `:1500,1526,2741` | ❌ | **丢（回退）** |
| VNC 断线诊断（`closeReason`/`closeCode`/`href`/`ws`） | ✅ `:1150-1180` | 部分 | **弱化** |
| `floatOpacity` 保存后即时下发 store | ✅ 无（车也没） | ✅ `:872-877` | 机侧**领先** |

**唯一正确的合并底座是「车侧为底」**，理由与 t1 结论一致（车侧 `index.js` 是超集），本审计从 client.js 侧独立复核到同一结论（见下表 #19/#20/#22 与 §2.2）。

---

## 2 差异总表

### 2.1 命名与量化

| 文件 | 原样 diff | 改名归一 diff | 代码-only diff | 结论 |
|---|---|---|---|---|
| `client.js` | — | **53 hunk / 266 车-only / 230 机-only / 496 行** | 62 hunk / 183 / 159 | 主要分叉面 |
| `index.js` | 车 581 行 / 机 510 行 | **3 hunk / 74 车-only / 3 机-only** | 3 / 57 / 0 | **车侧是超集**（差 71 行 = 12 条 `/sim*` 路由 + 3 个 import + 1 个文件常量） |
| `float.html` | 车 309 / 机 319 行 | **8 hunk / 40 车-only / 50 机-only** | 12 / 28 / 39 | 机侧 4 格 ⊃ 车侧 3 格，但丢进度条 |
| `vnc-app.html` | 233 / 233 | 2 hunk / 2 / 2 | 2 / 2 / 2 | **纯文案**（机侧手工修了规则没覆盖的「这台车」） |
| `vnc-connect.html` | 398 / 398 | 1 hunk / 2 / 2 | **0（完全相同）** | **纯注释折行**，能力 0 差异 |
| `src/store.js` | 184 / 184 | 2 hunk / 4 / 4 | 2 / 2 / 2 | 只有 `host` 与 `vncPassword` 两个默认值 |
| `src/ssh.js` | 254 / 252 | 3 hunk / 36 / 34 | 2 / 21 / 21 | **纯函数搬家**（见 §5.7） |
| `src/files.js` | 366 / 367 | 1 hunk / 1 / 2 | 1 / 1 / 2 | 只有 `homeShortcuts()` 数据项 |
| `src/vnc.js` | 403 / 403 | 5 hunk / 35 / 35 | 2 / 2 / 2 | 只有 1 处错误文案 + 1 处缩进 |
| `src/ws.js` | 295 / 295 | **改名归一后完全相同** | 0 | **零差异** |
| `src/trust.js` | 53 / 53 | **完全相同** | 0 | **零差异** |
| `src/reload.js` | 262 / 262 | **完全相同** | 0 | **零差异** |
| `cordis.patch.yml` | 296 B / 296 B | **完全相同** | 0 | **零差异** |
| `package.json` | — | 2 hunk / 4 / 2 | 3 / 4 / 2 | version + description + files 白名单 |
| `README.md` | — | 9 hunk / 202 / 62 | — | 文档，车侧更全 |

**lead 给的「394 行差异、22 个 hunk」口径澄清**：我用 git 默认 `-U3` 得到 **53 hunk / 492 行**。其中**变更行数 > 6 的 hunk 恰好 22 个，合计 393 行**（§5.4）——即 lead 的数字是**过滤掉小 hunk 后**的口径，两者不矛盾，本报告按 53 hunk 全覆盖分析。

### 2.2 22 个「大 hunk」逐个定性（+ 其余 31 个小 hunk 合并归类）

> 行号：车侧指 `dzrobot_v2/dsh-plugin-dzrobot-ssh/client.js`，机侧指 `dzsmartscout/dsh-plugin-dzdrone-ssh/client.js`。

| # | 车侧行 | 机侧行 | 内容 | 性质 | 合并必须保留谁 |
|---|---|---|---|---|---|
| 1 | 11-14 | 11-13 | 头注释：float 从「电量+CPU 两条曲线」改成「温度+CPU 动态轴」 | **纯文案** | 合并后重力写成「双节点卡」，两侧描述都作废 |
| 2 | 117-119 | 114-155 | 机侧把 `vncConnector*`/`conn*` **40 条 i18n 键整块搬到前面** | **纯搬迁**（内容仅 `connCarBridge`→`connDroneBridge` 等改名） | **取车侧位置，键名统一** |
| 3 | 144-148 | 180-185 | `floatBattery`→`floatTemp`，新增 `floatNoFc`/`floatBatteryV`，删 `floatBattVolts`/`floatBattDerived`/`floatBattNone` | **机侧改进**（温度+三态电量） | **取机侧这 3 个键**；但 `floatBattDerived`（12V/10V 折算）是车侧能力，**别丢** |
| 4 | 169-208 | 206 | 车侧的 `vncConnector*` 块（40 行）被搬到 #2 位置 | **纯搬迁** | 同上 |
| 5 | 284-284 | 279-318 | 英文 i18n 同上（40 行） | **纯搬迁** | 同上 |
| 6 | 307-313 | 341-348 | 英文 float 键：`Temperature`/`No FC`/`{v} V`，删 3 个 batt 键 | **机侧改进** | 同 #3 |
| 7 | 332-371 | 367 | 英文 `vncConnector*` 块搬迁 | **纯搬迁** | 同上 |
| 8 | 803-806 | 797-799 | 注释：把 `floatOpen` 说明上移合并 | **纯文案** | 任取 |
| 9 | 834-845 | 833-844 | `CarGlyph`→`DroneGlyph` **换了整套 SVG**：车侧是「小车+轮子+地面线」（5 元素，`viewBox 16`），机侧是「俯视飞机：机身+后掠翼+平尾+座舱点」（5 元素，`viewBox 24`） | **机侧改进（真实视觉资产）** | **取机侧**（车侧图标对无人机无意义；合并后车图标要另外找回） |
| 10 | 1148-1158 | 1150 | 断线消息：车侧 11 行分级（`closeReason` → `closeCode` → `reason`），机侧压成 1 行 `data.clean === false ? (data.reason \|\| vncDropped) : ''` | ⚠️ **机侧回退**（丢 `closeCode` 兜底） | **取车侧** |
| 11 | 1172-1179 | 1164 | `diag()` 载荷：车侧记 `closeCode`/`closeReason`/`href`/`ws` 6 字段，机侧只记 2 字段 | ⚠️ **机侧回退**（诊断信息减少） | **取车侧** |
| 12 | 1604-1611 | 1582 | `batteryLevelOf()`（电量阈值反向：≤15 hot / ≤30 warm）被**整段删除**，注释改成「Jetson 80 °C 节流」 | **机侧改进的一半 + 车侧能力的删除** | **两侧都要**：`levelOf` 正向用温度，`batteryLevelOf` 反向用电量 |
| 13 | 1668 | 1638（块 `:1640-1667`） | **新增 `batteryView(reading, error, now)` 28 行**（含 17 行三层注释）：`live`/`nofc`/`down` 三态，带 `UPTIME_STALE_MS` 陈旧界 | ✅ **机侧最大真改进** | **取机侧**（但见 §0 结论 4：`nofc` 粒度不够） |
| 14 | 1697-1702 | 1696（`isTemp` 在 `:1699-1702`，另 `:1717`、`:1741`） | `chartModel` 加 `isTemp`：温度 ±5 °C / 刻度 2.5，CPU 仍 ±10 / 5，**且温度不 clamp 到 0..100** | ✅ **机侧改进（正确，温度无 0..100 语义）** | **取机侧** |
| 15 | 1875-1878 | 1875-1882 | 引入 `const batt = batteryView(...)`，`battery`→`temp` 变量重命名 | **机侧改进** | 取机侧 |
| 16 | 1953-1964 | 1956 | **删除 `battTip()`** 12 行（车侧按 `batteryNote==='derived'` 显示「按 12V满/10V空 折算」） | ⚠️ **机侧回退 + 车侧能力丢失** | **取车侧**（机侧应改成按 `batteryNote` 显示「飞控无数据」） |
| 17 | 1992 | 1982-1988 | **新增 header 电量竖线** 7 行：`<span class="dzd_fbatt">电量：65% / 无飞控 / —` | ✅ **机侧改进** | 取机侧 |
| 18 | 2003-2006 | 2000-2005 | 左侧 cell 从 `battery` 换 `temp`：`°C`、`levelOf(temp, 65, 80)`、tooltip 改成 `{n} 个测点 · 最高 {id}` | ✅ **机侧改进** | 取机侧 |
| 19 | 2067-2076 | 2062（图标块 `:2062-2067`） | `float` 行图标：车侧是**电池形状**（矩形+端子+填充），机侧是**仪表盘指针**（弧+针+轴心） | **机侧改进（配套温度）** | 取机侧（合并后需再给电量找图标） |
| 20 | 2138-2153 | 2121 | **删除 `mode: 'external'` 分支** 16 行（弹窗打开 `/api/xxx-ssh/sim.html`，失败回填手动 URL） | ⚠️ **车侧功能，机侧没有** | **取车侧**（前提是合并底座含 sim 台） |
| 21 | 2284-2288 | 2251-2262 | `bridgeConnectorConfig` 增加 **loopback→`localhost` 改写**（`::1` 或 `127.x.x.x` → `localhost`），并新增 6 行说明 | ✅ **机侧改进** | **取机侧**（车侧 `test-client.mjs:540-542` 断言 `host === '127.0.0.1'`，合并后必须同步改成 `localhost`） |
| 22 | 2726-2733 | 2700 | 删除「connector 不再单独注册面板」的 8 行注释块 | **纯文案** | 任取 |
| — | 其余 31 个小 hunk | | 全部为：改名漏网（`carPassword`→`dronePassword` 等 6 处）、纯注释折行、i18n 单键改名、`toolClose` 从「Close」→「Hide」 | **纯文案/改名** | 见 §3 |

### 2.3 五个原始问题逐条作答

**Q1 「22 个 hunk 里哪些是机侧特有改进？」**

除已知的 4 项（float 温度、`batteryView`、`nofc`、图表 ±5/±2.5）之外，本轮**新查出 4 项真改进**：

1. **`bridgeConnectorConfig` 的 loopback 改写**（机 `client.js:2265-2272` / 车 `:2288-2299`）——车侧原样把 `127.0.0.1` 抛进「VNC 主机」字段（车 `:2294` `host: url.hostname`），机侧改写成 `localhost`（机 `:2268` 判 `::1`/`127.x`，`:2272` 才赋值），避免把用户从没输入过的环回地址泄漏到 UI。**有测试守着**（机 `test-client.mjs:761` 断言 `host === 'localhost'`）。
2. **`store.set({ floatOpacity })` 在 `/config` 保存后即时下发**（机 `client.js:872-877`）——车侧保存后**不下发**（车 `:869-881` 整段 save 里只有 `setSettings`/`setCommand`/`setNotice`），浮窗透明度要等下一次 `/status` 轮询才生效。
3. **`DroneGlyph` 整套 SVG 重画**（机 `client.js:833-844`）——`viewBox` 从车侧 `0 0 16 16` 改成 `0 0 24 24`，5 个元素全换（车 `:834-845` 是车身/车顶/两轮/地线，机是机身/两翼/平尾/座舱），并新增 `strokeWidth 1.7` + `strokeLinejoin round`。
4. **header 里的常驻电量竖线**（`client.js:1982-1988` 机）——车侧电量只在左侧大 cell，机侧把它同时挂到 header，切走视角仍可见。

**Q2 「车侧后来加的功能机侧完全没有？」**

**核实结论：`VncConnectorPanel` / `VNC_RESOLUTIONS` / `CONNECTOR_ID` / `bridgeConnectorConfig` 在机侧全部存在，不是车侧独有。**

| 符号 | 车侧 | 机侧 | 判定 |
|---|---|---|---|
| `makeVncConnectorPanel` | `:2324` | `:2298` | **两侧都有** |
| `VncConnectorPanel`（内部组件名） | `:2325` | `:2299` | **两侧都有** |
| `VNC_RESOLUTIONS` | `:2237` | `:2204` | **两侧都有** |
| `CONNECTOR_ID` | `:2235` = `'dzrobot-vnc-connector'` | `:2202` = `'dzdrone-vnc-connector'` | **两侧都有**，仅值不同 |
| `bridgeConnectorConfig` | `:2293` | `:2266` | **两侧都有**（机侧多 loopback 改写） |
| `bridgeHttp` | **仅 `test-client.mjs:540`** | **仅 `test-client.mjs:760`** | **两侧 client.js 都没有**——它是**测试局部变量**，不是产品符号 |
| `mode: 'external'` | `:2055` | ❌ **无** | ⚠️ **车侧独有** |
| `sim.html` 引用 | `:2054,2146` + `index.js` | ❌ **无** | ⚠️ **车侧独有** |
| `batteryLevelOf` | `:1616,2009` | ❌ 无 | ⚠️ **车侧独有** |
| `battTip()` | `:1960` | ❌ 无 | ⚠️ **车侧独有** |
| `origin` 存进 store 供 external 打开 | `:2113`（`store.set({origin})`） | ❌ 无 | ⚠️ **车侧独有**（机侧 `:2090` 的 store.set 无 `origin` 键） |

**所以车侧独有的是「仿真台入口链路」（`mode:'external'` + `sim.html` + `store.origin`），不是通用 VNC connector。** 通用 connector 是 2026-10-07 fork 时就有的共同祖先（§5.5 已在基线 `667a8be` 中验证）。

**Q3 「机侧 metrics.js 是不是车侧的严格超集？」**

**不是。是「换了一套」，有增有减。**

| 能力 | 车侧 `metrics.js` | 机侧 `metrics.js` | 增减 |
|---|---|---|---|
| CPU 双采样 + delta | ✅ `:82-107` | ✅ 同 | 相同 |
| uptime + btime + loadavg | ✅ `:88-92` | ✅ 同 | 相同 |
| 电量来源 | ROS `/Battery_Percentage` + `/PowerVoltage` | PX4 `/fmu/out/battery_status` + `batteryFc` 门控 | **换源** |
| 电量返回形状 | `battery: number\|null` + `batteryVolt` + `batteryNote:'derived'\|'no_stack'` | `battery: {percent, voltage}` **或** `batteryNote:'dds_silent'\|'no_ros'` **或** 两者皆无 | ⚠️ **同名不同型** |
| 12 V/10 V 折算 | ✅ `:223-226` | ❌ 删除 | **−1** |
| `normalizeBattery()` 双标度容错 | ✅ `:251-257`（导出） | ❌ 删除 | **−1** |
| 温度（多测点 + 峰值归因） | ❌ | ✅ `:69,162-171,236-238`（`temp`/`tempAt`/`zones`/`temps`） | **+1** |
| `timeoutMs` | 12000（ROS CLI 慢） | 9000 | 机侧更省 |
| 导出符号 | `METRICS_COMMAND, collectMetrics, parseMetrics, normalizeBattery, cpuUsage`（5） | `METRICS_COMMAND, collectMetrics, parseMetrics, cpuUsage`（4） | **−1** |

**`battery` 键类型冲突**已由 t1 独立发现（`capability-matrix.md:18`），本审计用代码独立复核：车 `metrics.js:235` 返回 `battery`（number|null），机 `metrics.js:245` 返回 `battery: {percent, voltage}`。**浅合并必炸**。

**Q4 「vnc-app.html / vnc-connect.html 同长度不同哈希，差异是什么？」**

| 文件 | 行数 | 差异位置 | 内容 | 判定 |
|---|---|---|---|---|
| `vnc-app.html` | 233 / 233 | 车侧 `:47`、`:164` | 重放规则后车侧仍是 `这台车的 VNC 需要密码` → **机侧是「这台无人机的」** | ✅ **机侧更干净**（`小车→无人机` 规则命不中「这台车」，机侧手工补了） |
| | | 车侧 `:139`、`:223` | 注释 `the car's desktop` → 机侧 `the drone's desktop` | 纯注释 |
| `vnc-connect.html` | 398 / 398 | 车侧 `:209-210` | 注释**折行位置**不同（词一样，断行差 1 个词） | **纯排版，能力 0 差异** |

**结论：`vnc-connect.html` 的能力差异为 0**（代码-only diff 为空）。`vnc-app.html` 的差异是**机侧遗留 2 处「小车」字样**——**合并不丢功能，但合并时应顺手修掉**。

**Q5 「float.html 机侧多 10 行的能力是什么？」**

行数：车侧 309 / 机侧 319（`split('\n')` 口径；`Get-Content .Count` 得 308/318，**两种口径差值恒为 10**，差 1 是行尾）。LCS 逐行比对：**车侧独有 40 行 / 机侧独有 50 行 / 净 +10**。

这 10 行**不是单一能力**，是 15 个 hunk 的净和（§5.10 可复现）：

| hunk | 车侧行 | 机侧行 | 内容 | 净行 |
|---|---|---|---|---|
| 1 | 10 | 10 | 头注释：删掉「电量来自底盘 ROS」整段说明 | −4 |
| 2 | 131 | 127 | 第一格 `batteryCell`（含 `<span class="bar">`）→ `tempCell`（**无 bar**） | −1 |
| 3 | 148 | 143 | `<main>` 末尾**新增** `batteryCell`（只有 value+sub） | **+5** |
| 4 | 162 | 162 | zh 字典：`battery`→`temp`，加 `noFc` | +1 |
| 5 | 166 | 167 | zh：删 `battVolts`/`battDerived`/`battNone`，加 `zones` | **−3** |
| 6 | 173 | 171 | en 字典同上 | +1 |
| 7 | 177 | 176 | en：删 3 个 batt 键，加 `zones` | **−3** |
| 8 | 192 | 188 | `batteryLabel`→`tempLabel` 赋值 | 0 |
| 9 | 195 | 191 | **新增** `batteryLabel` 赋值 | +1 |
| 10 | 201 | 198 | `STALE_MS` 注释补一句现象 | 0 |
| 11 | 215 | 212 | `level()` 注释改成 Jetson 80 °C | 0 |
| 12 | 217 | 214 | **删除** `battLevel()`（电量反向阈值） | **−2** |
| 13 | 232 | 227 | 电量渲染 10 行 → 温度渲染 8 行 | −2 |
| 14 | 264 | 257 | **新增电池三态渲染** 17 行（`live`/无值/断线 dash） | **+17** |
| 15 | 265 | 275 | `document.title` 从电量改成温度 | 0 |

**净 +10。** 两侧精确对账（可复算）：

- **机侧独有 50 行** = h1(5) + h2(4) + h3(5) + h4(2) + h5(1) + h6(2) + h7(1) + h8(1) + h9(1) + h10(1) + h11(1) + h13(8) + h14(17) + h15(1) = **50** ✓
- **车侧独有 40 行** = h1(9) + h2(5) + h4(1) + h5(4) + h6(1) + h7(4) + h8(1) + h10(1) + h11(1) + h12(2) + h13(10) + h15(1) = **40** ✓

**能力定性（不强行算术拆分——温度与电量在 h2/h5/h7/h13 里是同一批行的替换，拆不开）**：

| 变化 | 所在 hunk | 定性 |
|---|---|---|
| **温度格新增**：`tempCell` HTML、`tempValue` 渲染、`tempLabel` 赋值、`temp`/`zones` 两条 i18n | h2 +4、h8 +1、h13 +8、h4 +1、h5 +1、h6 +1、h7 +1 | ✅ **真新增能力** |
| **电池三态渲染新增**：`live` 显百分比+电压 / 无值显 `无飞控` / 断线显 `—` | h14 +17、h9 +1 | ✅ **真新增能力**（但 `nofc` 粒度不足，§0 结论 4） |
| **电池格重排**：从第一格移到末格，**且删掉 `<span class="bar"><i id="batteryBar">`** | h2 −5→+4（bar 行没了）、h3 +5 | ⚠️ **位置对调 + 进度条丢失** |
| **电量反向阈值 `battLevel()` 整段删除**（≤15 hot / ≤30 warm） | h12 −2 | ⚠️ **能力丢失** |
| **电量来源 tooltip 删除**：`battVolts`/`battDerived`/`battNone` 三条 i18n + 8 行渲染分支 | h5 −3、h7 −3、h13 中 10 行 | ⚠️ **能力丢失** |
| 头注释改写、`STALE_MS`/`level()` 注释补现象、title 改温度 | h1 −4、h10 0、h11 0、h15 0 | 纯文案 |

---

## 3 合并操作清单（按文件给：以谁为底、回贴什么）

> **总原则：除 `float.html` 与 `metrics.js` 外，一律以车侧为底、回贴机侧增量。** 理由：车侧 9 个 Host 模块中 4 个与机侧**逐字节相同**（`ws.js`/`trust.js`/`reload.js`/`cordis.patch.yml`），另 4 个仅有 1-4 行真差异；`index.js` 车侧是严格超集。

| 文件 | 底座 | 必须回贴的机侧增量 | 必须从车侧保留的 | 冲突点 |
|---|---|---|---|---|
| **`src/index.js`** | **车侧**（581 行） | ① `/status` 的 `drone` 键改名（车 `:231` `withCar` → 机 `:221` `withDrone`，**统一为 `node`**）；② 无需其它 | 全部 12 条 `/sim*` 路由（车 `:327-390`） | 无 |
| **`src/metrics.js`** | **必须重写，两侧都不是底座** | 机侧温度段（`:69,162-171`）、PX4 电池段（`:86,190-217,241-248`） | 车侧 `batteryNote:'no_stack'` 语义（`:105`）、`normalizeBattery()`（`:251-257`）、12V/10V 折算（`:223-226`） | ⚠️ **`battery` 键类型冲突，必须改名**：车→`carBattery`(number)，机→`droneBattery`({percent,voltage})。**这是唯一必须人工设计的接口** |
| **`client.js`** | **车侧**（2858 行） | ① `batteryView()` 块 `:1638-1667`（28 行，含三层注释）；② `chartModel` 的 `isTemp` 分支 `:1699-1702,1717,1741`；③ `DroneGlyph` SVG `:833-844`；④ header 电量竖线 `:1982-1988`；⑤ `bridgeConnectorConfig` loopback 改写 `:2265-2272`；⑥ `/config` 保存后 `store.set({floatOpacity})` `:872-877`；⑦ 温度 i18n 键 `floatTemp`/`floatNoFc`/`floatZones`（`:190,194,2003`）；⑧ `levelOf(temp, 65, 80)` `:2008`；⑨ float 仪表盘图标 `:2062-2067` | ① `batteryLevelOf()` `:1616`；② `battTip()` `:1960-1969`；③ 断线分级 `:1150-1160`；④ `diag` 6 字段 `:1177-1181`；⑤ `mode:'external'` + `sim.html` `:2055,2146`；⑥ `store.set({origin})` `:2113` | `batteryView` 与 `battTip` 必须共存：`batteryView` 判三态，`battTip` 渲染原因。**机侧 `nofc` 要拆成 `nofc`（真无飞控）/`noData`（飞控在但 DDS 静默）** |
| **`float.html`** | **机侧**（319 行，已含温度+电池） | — | ① 电量进度条 `<span class="bar"><i id="batteryBar">` 车 `:134`；② `battLevel()` 反向阈值 车 `:188`；③ `battVolts`/`battDerived`/`battNone` 三条 i18n 车 `:167-169`；④ 进度条驱动 JS 车 `:235-236` | 机侧 4 格布局优于车侧 3 格，**底座取机侧**；补回进度条后电量格与车侧等价且多温度 |
| **`src/store.js`** | **车侧**（184 行） | `vncPassword:'duzhong'` 的**注释说明**（机 `:51`） | `vncPassword:''`（**不能把机侧密码写进共用默认值**） | 合并后 `host` 必须**每节点独立配置**，`dzsmart.local` 与 `duzhong.local` 二选一都是错的 |
| **`src/ssh.js`** | **车侧**（254 行） | 无（机侧只是把 `refusalFor()` 从 `:162` 搬到 `:196`，**函数体逐字节相同**，见 §5.7） | 全部 | ⚠️ **机侧 `:204,206` 仍写「车已开…」「车还没连上 WiFi」**（车侧同位置 `:170,172` 本就写「车」，属正确；机侧改名规则命不中这两个中文串，故**原样继承**）。合并后必须**参数化**，否则无人机掉线会说「车还没连上 WiFi」 |
| **`src/files.js`** | **车侧** | `homeShortcuts()` 增加 `dzpx4` + `dzuav` 两项（机 `:362-363`） | `dzrobot_ws` 项（车 `:362`） | 捷径表应按节点分组，不是覆盖 |
| **`src/vnc.js`** | **车侧** | 无（仅 `:310` 错误文案「这台车/这台无人机」+ `:368` 的 `onData()` 缩进差 2 空格，**其余逐字节相同**） | 全部 | **任取其一，代码等价** |
| **`src/ws.js` / `src/trust.js` / `src/reload.js` / `cordis.patch.yml`** | **任取** | **零差异** | — | 无 |
| **`package.json`** | **车侧**（version `0.5.0`，含 `sim.html`+`sim-root.json` 白名单） | 机侧的 description 里「Jetson Orin Nano」「通用 VNC 连接器」措辞 | `files` 白名单必须含 sim 资产 | version 不能回退到 `0.3.0` |
| **`vnc-app.html`** | **机侧**（文案已干净） | — | — | 合并后**别把车侧重放版本盖回来**（否则「这台车的 VNC 需要密码」会重新出现） |
| **`vnc-connect.html`** | **任取** | **代码 0 差异** | — | 无 |
| **`DroneGlyph` SVG** | **机侧**（`:833-844`） | — | 需**另行为车侧找回**小车图标（车侧 `:834-845` 的 5 个元素在机侧已不存在） | 合并后两节点需要两套图标 |

### 3.1 合并后「三个必须保留项」（按丢失代价排序）

1. **`src/simbench.js` + `stressbench.js` + `dronesim.js` + `sim.html` + `sim-root.json` 与 12 条 `/sim*` 路由**（车侧 `index.js:38-40,327-390`）——这是**整块能力域**，机侧一行都没有，丢了等于回到「只能连机不能仿真」。
2. **`batteryNote` 三态语义 + `normalizeBattery()` 双标度容错**（车 `metrics.js:105,251-257`）——机侧 `metrics.js` 已经用 `batteryFc`/`batteryNote` 重造了前者的骨架，但**UI 层没接**（§0 结论 4），且后者（`normalizeBattery`）被彻底删除，其 8 项参数化测试也一并丢失。
3. **稳定组件身份 + VNC 断线诊断**（车 `client.js:1500,1526,2741` 与 `:1150-1180`）——前者的丢失会让 VNC tab 每次 `ctx.effect` 重跑都 remount，**直接表现为「连不上/画面反复重置」**（车侧注释 `:1494-1498` 记录了原症状：`diag logs hundreds of vnc-connector.ready an hour`）；后者的丢失把断线原因从「WebSocket code 1006 + 桥的拒绝理由」压缩成一句泛化文案。

---

## 4 未验证项（缺什么证据）

| 项 | 为什么未验证 | 需要什么才能定论 |
|---|---|---|
| 机侧 `batteryView` 的 `nofc` 在**真机上**的真实表现 | 本轮**未连机**（遵守只读约束，不 ssh/不跑 ROS）。代码层面已确认 `client.js` 不读 `batteryNote`，但**未在真机复现**「飞控插着但 DDS 静默时浮窗显示什么」 | 上机一次：拔/插飞控 USB，看浮窗文案；或 `node scripts/check-widget-target.mjs` 打印 `batteryNote` |
| 机侧是否曾有过 `batteryLevelOf` 又被删 | 机侧**无 git**（`Test-Path .git` = False），无法回溯。「删除」只能从两侧 diff 推断，不能证明时序 | 机侧的 `.git`、备份或 `dzsmartscout - 副本` 目录的哈希比对（本轮未展开） |
| `dzsmartscout` 存在**两份副本**（`Desktop\dzsmartscout` 与 `Desktop\比赛存档\v1\dzsmartscout`） | 已核实 `client.js`/`index.js`/`src/metrics.js`/`float.html` **四文件哈希完全相同**，但**未逐文件比对全部 98/99 项** | 全量哈希（本轮只抽了 4 个关键文件） |
| 「已装副本 vs 磁盘源码」的 3 个车侧文件差异 | 已确认 `sim.html`/`dronesim.js`/`simbench.js` **剥注释后逐字节相同**（§5.8），差异纯属 `★ tXX：` 注释标记被清理。但**为何已装副本是「带标记」的旧注释版**未查（无 git 记录的安装时点） | 车侧 `deploy.mjs` 的部署日志，或 `.dsh` 安装时间戳 |
| `README.md` 车侧多 202 行的具体能力 | 已确认车侧文档更全，但**未逐节比对**（本任务聚焦代码能力） | 若合并文档需要，另开卡 |

---

## 5 复核命令与证据（可原地重跑）

所有临时脚本写在 `%TEMP%`，**未写入仓库**。

### 5.1 两侧版本状态

```powershell
cd "D:\User\Desktop\比赛存档\v1\dzrobot_v2"
git rev-parse --short HEAD                    # 0264159
git diff --stat HEAD -- dsh-plugin-dzrobot-ssh   # 空 → 该目录无未提交变更
Test-Path "D:\User\Desktop\比赛存档\v1\dzsmartscout\.git"   # False
```

### 5.2 改名归一 diff（核心方法）

```powershell
# %TEMP%\audit_run.mjs —— 逐字照抄 adapt_drone_plugin.mjs:12-41 的 20 条规则
node "$env:TEMP\audit_run.mjs"
# 输出：client.js hunks=53 carOnly=266 droneOnly=230
#       index.js  hunks=3  carOnly=74  droneOnly=3
#       src/ws.js src/trust.js src/reload.js cordis.patch.yml → IDENTICAL-AFTER-ADAPT
```

> ⚠️ **踩坑记录**：用 `node -e "…"` 内联（PowerShell 单引号包裹）会把 `\bCar\b` 的 `\b` 吃掉，导致词边界规则失效、hunk 数虚高到 63。**必须写成 `.mjs` 文件再跑。** 本报告全部数据来自文件版脚本。

### 5.3 代码-only diff（剥注释）

```powershell
node "$env:TEMP\codeonly.mjs"
# 输出：client.js hunks=62 carOnly=183 droneOnly=159
#       vnc-connect.html → IDENTICAL（能力 0 差异）
#       src/ws.js → IDENTICAL
```

### 5.4 复现 lead 的「22 hunk / 394 行」

```powershell
node "$env:TEMP\hunks.mjs"
# 输出：total hunks=53  changed lines=492  copy-only hunks=8
#       hunks >6 changed lines: 22  (their lines: 393)
```

### 5.5 稳定组件身份在 fork 基线就存在

```powershell
cd "D:\User\Desktop\比赛存档\v1\dzrobot_v2"
git show 667a8be:dsh-plugin-dzrobot-ssh/client.js |
  Select-String 'const ConnectorPanel|const VncTabShell|const FilesTabShell'
# 三条全中 → 2026-10-07 fork 时车侧已有保护；机侧现在缺失 = 回退
```

### 5.6 机侧 `batteryNote` 无 UI 消费者（§0 结论 4 的证据）

```powershell
$drn = "D:\User\Desktop\比赛存档\v1\dzsmartscout\dsh-plugin-dzdrone-ssh"
foreach ($f in 'client.js','float.html') {
  Select-String -Path "$drn\$f" -Pattern 'batteryNote|dds_silent|no_ros|飞控无数据'
}
# 两个文件均无输出
Select-String -Path "$drn\src\metrics.js" -Pattern "batteryNote \?\? 'dds_silent'"   # :244 有
```

### 5.7 `src/ssh.js` 的 `refusalFor` 是纯搬家

```powershell
node "$env:TEMP\codeonly.mjs"    # 看 CODEONLY_src_ssh_js.diff
# 21 行 del + 21 行 add，内容逐字符相同，只是从 :90 移到 :111（归一化坐标）
# 原始坐标：车侧 export function refusalFor → :162；机侧 → :196
```

### 5.8 已装副本漂移是纯注释

```powershell
node "$env:TEMP\comment_only.mjs"
# sim.html / src/dronesim.js / src/simbench.js → comment-stripped identical = true
```

### 5.9 关键符号存在性核对（Q2）

```powershell
$car = "D:\User\Desktop\比赛存档\v1\dzrobot_v2\dsh-plugin-dzrobot-ssh\client.js"
$drn = "D:\User\Desktop\比赛存档\v1\dzsmartscout\dsh-plugin-dzdrone-ssh\client.js"
foreach ($s in 'makeVncConnectorPanel','VNC_RESOLUTIONS','CONNECTOR_ID','bridgeConnectorConfig','batteryLevelOf','batteryView') {
  "{0,-24} car={1,3} drone={2,3}" -f $s,
    (Select-String -Path $car -Pattern $s | Measure-Object).Count,
    (Select-String -Path $drn -Pattern $s | Measure-Object).Count
}
# makeVncConnectorPanel car=4 drone=4 / VNC_RESOLUTIONS 4/4 / CONNECTOR_ID 2/2
# bridgeConnectorConfig 4/4 / batteryLevelOf 3/0 / batteryView 0/3
```

### 5.10 float.html 15 个 hunk 逐条（Q5 证据）

```powershell
node "$env:TEMP\float_hunks.mjs"
# 15 hunk，净 +10 行；明细见 §2.3 Q5 表
```

---

## 6 一句话交给 Lead

**合并必须以车侧为底**：机侧净减一整个仿真能力域（12 条路由 + 5 个模块）与 3 处保护（稳定组件身份、断线诊断、电量折算），真增量只有温度监控与 PX4 电池两块，且 PX4 电池的 `batteryNote` 三态**在 UI 层一处没接**（会谎报「无飞控」）——`metrics.js` 的 `battery` 键两侧类型不同，是本次唯一必须人工设计契约的接口。
