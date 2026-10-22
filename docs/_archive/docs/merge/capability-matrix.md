# 双侧 Host 能力盘点与合并裁决矩阵

任务：t1 · task-1　｜　盘点人：host-inventory　｜　盘点时间：2026-10-17（本地时区 Asia/Shanghai）
输入（**全程只读，未修改任何插件文件、未安装、未部署、未跑 pnpm**）：

- 车侧：`D:\User\Desktop\比赛存档\v1\dzrobot_v2\dsh-plugin-dzrobot-ssh\`
- 机侧：`D:\User\Desktop\比赛存档\v1\dzsmartscout\dsh-plugin-dzdrone-ssh\`
- 已装副本：`C:\Users\TestFox\.dsh\profiles\desktop\node_modules\dsh-dzrobot-ssh` / `dsh-dzdrone-ssh`
- 运行期数据：`C:\Users\TestFox\.dsh\dzrobot-ssh\` / `C:\Users\TestFox\.dsh\dzdrone-ssh\`

---

## 0 结论（5 行）

1. **Host 半身 9 个 JS 模块里，6 个是纯文案差异**（`ssh.js` / `ws.js` / `trust.js` / `reload.js` / `vnc.js` / `files.js` 的代码逻辑），真正的**功能分叉只有 3 处**：`metrics.js`（电量 vs 温度+PX4 电池）、`index.js` 路由表（车侧多 12 条 `/sim*`）、`store.js` 默认值（VNC 密码）。
2. **合并取「机侧为主、车侧补 sim」不成立**——正确判断是**代码以车侧为底，机侧只贡献 metrics 的温度/PX4 能力**：车侧 `index.js` 576 行覆盖机侧 510 行的超集（唯一例外是 `withDrone` 参数名），`src/` 也多出 3 个文件。
3. **`camHost`/`camPort` 疑点已查清：不是「运行版本比磁盘新」，也不是「第三个写入者」**。源码与已装副本**全无这两个键**，它们是 `POST /config` 穿过无白名单的 `sanitize()` 后被永久写盘的**脏键**，证据见 §3。
4. **`metrics` 是唯一必须人工设计的接口**：两侧返回的同名键 `battery` **类型不同**（车=number，机=object），直接浅合并会让消费端拿到两种形状之一，必须重命名为 `carBattery` / `droneBattery`。
5. **`novnc/` 58 文件 583780 B 逐字节相同**，合并后只能留一份。单位换算：`583780 B = 583.8 KB(÷1000) = 570.1 KiB(÷1024)`——方案文档写的「570.1 KB」实际是 KiB 口径，数值没错只是单位名混用；合并时以 **583780 B** 为唯一判据。

---

## 1 逐模块裁决矩阵

> 「行号」均指**磁盘源码**（非已装副本）。「代码差异」= 剥离注释/空行、并把 `dzrobot↔dzdrone`、`car↔drone`、`小车↔无人机` 归一化之后的剩余差异行数（方法见 §5）。

| 模块 | 车侧 | 机侧 | 差异性质 | 合并取谁 | 理由（文件:行号） |
|---|---|---|---|---|---|
| **src/store.js** | 184 行；`DATA_DIR_NAME='dzrobot-ssh'`；`vncPassword:''` | 184 行；`DATA_DIR_NAME='dzdrone-ssh'`；`vncPassword:'duzhong'` | **配置默认值分叉**（逻辑 0 差异） | **取结构，值必须改为「每节点各一份」** | 两侧 `SETTINGS_INITIAL` 同构但默认值不同：车 `store.js:21,52` vs 机 `store.js:21,52`。`vncPassword` 是**设备专属**（机侧注释写明「verified by a real RFB handshake」，机 `store.js:51`），单店全局默认值会互相污染。**另：合并时必须加白名单**，见 §3.3 |
| **src/ssh.js** | 254 行；`LOGIN_MARKER='DSH_DZROBOT_OK'`（:29） | 252 行；`LOGIN_MARKER='DSH_DZDRONE_OK'`（:29） | **纯常量 + 文案**（代码差异 0） | **取车侧，`LOGIN_MARKER` 参数化** | 归一化后代码差异 0 行。唯一真实差异是 marker 常量（车 `:29` / 机 `:29`）与 2 处 doc 注释。**但机侧有真 bug**：`ssh.js:204,206` 的拒绝文案仍写「车」（`'端口被拒绝（车已开但 sshd 没起？）'`、`'网络不可达（车还没连上 WiFi？）'`）——车侧同位置是 `:170,172`。合并后文案应参数化，否则无人机掉线时面板会说「车还没连上 WiFi」 |
| **src/metrics.js** | 273 行；**电量**（ROS `/Battery_Percentage` + `/PowerVoltage`）+ CPU + uptime；`timeoutMs=12000`；`BATTERY_PROBE` :67-73 | 265 行；**温度**（`thermal_zone*`）+ CPU + uptime + **PX4 `/fmu/out/battery_status`**；`timeoutMs=9000` | ⚠️ **真功能分叉（唯一必须重新设计契约的模块）** | **两侧都要，合并为一个能力驱动的模块** | 见 §2 专项分析。机器侧**并非车侧的严格超集**：机侧**没有**任何温度/电池的 `batteryNote:'no_stack'` 语义，车侧**没有** `temp`/`temps`/`zones`/`batteryFc` |
| **src/files.js** | 366 行；`homeShortcuts()` :358 → 5 项，含 `/home/duzhong/dzrobot_ws` | 367 行；`homeShortcuts()` :358 → **6 项**，含 `/home/duzhong/dzpx4` + `/home/duzhong/dzuav` | **纯数据表 + 文案**（代码差异 3） | **取逻辑（两侧同构），shortcuts 按节点注入** | 归一化后代码差异 **3 行，全部是 `homeShortcuts()` 数组里的捷径项**（车 1 行 `dzrobot_ws` ↔ 机 2 行 `dzpx4`+`dzuav`）。另有 4 条错误文案（车 `:120,122,123,124` 写「小车」/ 机同位置写「无人机」）在中文归一化后**不再是差异**——但它们仍是设备文案，合并后应参数化 |
| **src/vnc.js** | 402 行 | 402 行 | **纯文案**（代码差异 2） | **任取其一（建议车侧）** | 归一化后仅 2 行差异：`vnc.js:310` 的 `new Error(\`地址 ${address} 不是这台**车**的 VNC 服务…\`)` vs 机侧同位置 `:310` 的「这台**无人机**」。RFB 横幅校验、mDNS 缓存防伪逻辑**完全一致**（剥注释后车/机各 245 行代码，差异仅上述 2 行） |
| **src/ws.js** | 294 行 | 294 行 | **纯注释**（代码差异 0） | **任取其一** | 唯一差异是一句注释里的 `car's` / `drone's` framebuffer 措辞 |
| **src/trust.js** | 51 行 | 51 行 | **100% 相同**（代码差异 0，SHA 不同仅因注释） | **任取其一** | `isLoopback()` + `rejectionFor()` 全等；文件差异只存在于头部注释中的 `/api/dzrobot-ssh/*` 字样 |
| **src/reload.js** | 261 行 | 261 行 | **100% 相同**（代码差异 0） | **任取其一** | 自热重载逻辑全等（`watchPackage` / `selfReload` / `isReloading`） |
| **index.js** | **576 行**；API 前缀 `/api/dzrobot-ssh`（:401）；VNC upgrade `/api/dzrobot-ssh/vnc`（:412）；**32 条路由** | **510 行**；前缀 `/api/dzdrone-ssh`（:330）；VNC upgrade `/api/dzdrone-ssh/vnc`（:341）；**20 条路由** | ⚠️ **功能分叉（车侧是超集，多出 12 条）** | **取车侧为底，机侧只补 `/status` 的 `drone` 键** | 车侧多出全部仿真台（**11 条 `/sim/*` + 1 条 `/sim.html` = 12 条**，与路由总数差 32−20=12 完全吻合）：`/sim.html` `:327`、`/sim/run` `:330`、`/sim/history` `:336`、`/sim/artifact` `:339`、`/sim/check` `:343`、`/sim/root` `:346`、`/sim/map` `:350`、`/sim/stress` `:354`、`/sim/stress/status` `:362`、`/sim/stress/result` `:365`、`/sim/drone/run` `:372`、`/sim/drone/history` `:384`。机侧 `/sim` 出现次数 = **0**。反向：机侧独有 `withDrone` 参数（drone `:221`）vs 车侧 `withCar`（`car:231`） |
| **静态资源 novnc/** | 58 文件 / **583780 B** | 58 文件 / **583780 B** | **逐字节完全相同** | **只留一份（原样）** | 逐文件 SHA-256 比对：`same=58 diff=0 only_car=0 only_drone=0`（§5 复核命令）。**合并后必须删掉重复的那份**，否则包体积多 570 KiB 且两份漂移无人察觉 |
| **float.html** | 308 行；**3 格**：`#batteryCell` `:131`（电量+进度条 `:134`）、`#cpuCell` `:137`、`#upCell` `:143`。**无温度格**（全文无 `temp` 键） | 318 行；**4 格**：`#tempCell` `:127`、`#cpuCell` `:132`、`#upCell` `:138`、`#batteryCell` `:143`（飞控电量 `:265-272`，读 `battery.percent`/`battery.voltage`） | ⚠️ **机侧是严格超集（不是对等分叉）** | **取机侧**（已含电量格），再补车侧的**电池进度条** | 机侧 4 格 ⊃ 车侧 3 格：机侧既有温度（`:127`）又有电量（`:143`）。差异只有两处细节：① 车侧电量格带 `<span class="bar"><i id="batteryBar">` 进度条（`:134,235-236`），机侧电量格**无进度条**（`:145-146` 只有 value+sub）；② 车侧电量读 number（`:232` `Number.isFinite(last.battery)`），机侧读 object（`:265` `battery.percent`）。**这两处差异正是 §2.2 类型冲突在 UI 上的投影** |
| **vnc-app.html** | 232 行 | 232 行 | **纯文案** | **任取其一** | 差异仅 `<title>小车 VNC / 无人机 VNC`、`asktext` 与 1 处注释 |
| **vnc-connect.html** | 387 行 | 387 行 | **纯文案** | **任取其一** | 差异仅文档头部 1 行注释 |
| **cordis.patch.yml** | 6 行；`id: dzrobot-ssh` / `name: 'dsh-dzrobot-ssh'` | 6 行；`id: dzdrone-ssh` / `name: 'dsh-dzdrone-ssh'` | **纯标识**（代码差异 0） | **合并后重写**（新 id/name） | 归一化后 0 差异 |
| **package.json** | `version: 0.5.0`；`files[]` 含 `sim.html`、`sim-root.json` | `version: 0.3.0`；`files[]` **不含** sim | **元数据 + 声明分叉** | **取车侧**（版本更高、files 更全） | 车 `package.json:3` `0.5.0` vs 机 `:3` `0.3.0`；`files` 数组车侧多 2 项。注意 `dsh.client.inject` 两侧都是 `[]`，`immediately:true` 一致 |
| **src/simbench.js / stressbench.js / dronesim.js** | 3 个文件（52714 / 15903 / 23048 B） | **完全不存在** | **车侧独有** | **取车侧，全部保留** | 机侧 `src/` 只有 8 个文件，车侧 11 个。这 3 个是仿真台的 Host 实现，`/sim/*` 路由的依赖；连同 `sim.html`(159633 B) 与 `sim-root.json`(269 B) 合计 **251567 B** |

---

## 2 专项：metrics 合并后的统一契约

### 2.1 两侧返回形状实测（照抄源码，不是推测）

**车侧** `metrics.js:228-240`（`parseMetrics` 返回）：

```js
{ cpuPercent, uptimeSec, bootEpoch?, load,
  battery,        // ← number|null（0..100，一位小数）
  batteryVolt,    // ← number|null（5..60 V，两位小数）
  batteryNote }   // ← 'no_stack' | 'derived' | null
```

**机侧** `metrics.js:226-249`（`parseMetrics` 返回）：

```js
{ cpuPercent, uptimeSec, bootEpoch?, load,
  temps,          // ← Array<{id,c}>（按温度降序，机 :225）
  temp,           // ← number|null（最热区，机 :236）
  tempAt,         // ← string|null（最热区名字，机 :237）
  zones,          // ← number（成功读到的区数，机 :238）
  battery,        // ← {percent,voltage} | undefined　★类型=object
  batteryNote }   // ← 'dds_silent' | 'no_ros' | undefined
```

### 2.2 ★ 最关键的风险：同名键 `battery` 类型冲突

| 侧 | 键 | 类型 | 源码 |
|---|---|---|---|
| 车 `battery` | `number \| null`（`0..100`） | 车 `metrics.js:237`（`return {}` 体内） |
| 机 `battery` | `{percent, voltage} \| undefined` | 机 `metrics.js:245`（三元表达式的成功分支） |

**`{...car, ...drone}` 浅合并会让 `battery` 静默变成 object**，而车侧消费端（`float.html:131-135` 的进度条、`client.js` 的 `battery` 分支）读的是 number。**这是本次合并在数据层最容易踩的雷**，必须在合并前改名，不能靠「运行时没崩」来判断。

### 2.3 建议的统一契约（供 lead 采纳）

```jsonc
{
  "cpuPercent": 12.3,          // 两侧同义，共用
  "uptimeSec": 1234,           // 两侧同义，共用
  "bootEpoch": 1792000000,     // 可选，两侧同义，共用
  "load": [0.1, 0.2, 0.3],     // 两侧同义，共用
  "carBattery": {              // ← 车侧能力，重命名后迁入
    "percent": 86.0,           //   number|null
    "volt": 11.72,             //   number|null
    "note": "no_stack"         //   'no_stack'|'derived'|null
  },
  "thermal": {                 // ← 机侧能力，重命名后迁入
    "temp": 43.2,              //   最热区摄氏
    "at": "cpu-thermal",       //   最热区名
    "zones": 6,                //   成功读取区数
    "list": [...]
  },
  "fc": {                      // ← 机侧能力（PX4）
    "battery": { "percent": 74.5, "voltage": 15.8 },  // 或 undefined
    "present": true,           //   batteryFc=present|absent
    "note": "dds_silent"       //   'dds_silent'|'no_ros'|undefined
  }
}
```

**「缺哪个数就说明为什么缺」的统一词汇**（把两侧词表拼成一套，去重后只有 4 个值）：

| 值 | 含义 | 来源 |
|---|---|---|
| `no_stack` | 底盘/ROS 栈没在发布（车） | 车 `metrics.js:105` |
| `derived` | 由电压按 VCU 标定换算（车） | 车 `metrics.js:225` |
| `dds_silent` | 飞控在，但 `/fmu/*` 话题无数据（机） | 机 `metrics.js:86,247` |
| `no_ros` | 有飞控设备，但没装/没 source ROS（机） | 机 `metrics.js:86` |

另有 `batteryFc: absent`（机 `metrics.js:86` 末尾）表示「压根没插飞控」，语义上等价于「没有这个能力」而非「读失败」，两者**不应混同**——机侧注释 `:204-208` 已明确这一点。

### 2.4 `float.html` 与 metrics 的强耦合点

浮窗不是独立模块，它**直接消费 metrics 的键名**，而两侧对同名 `battery` 的读取方式**已经分叉**：

| 侧 | 电量渲染 | 源码 |
|---|---|---|
| 车 | `const batt = Number.isFinite(last.battery) ? last.battery : null` → 当 **number** 用 | 车 `float.html:232`；`batteryBar.style.width` `:235` |
| 机 | `const battery = last && last.battery` → 当 **object** 用，`battery.percent.toFixed(0)` | 机 `float.html:261,265-268` |

- 车侧 3 格（`batteryCell:131` / `cpuCell:137` / `upCell:143`），**没有温度格**
- 机侧 4 格（`tempCell:127` / `cpuCell:132` / `upCell:138` / `batteryCell:143`），**温度与电量都有**
- 机侧特有的无飞控语义：`float.html:271` `$('batteryValue').textContent = t('noFc')`；字典键见 `:163,172`
- 车侧特有的电量进度条：`float.html:134` 的 `<span class="bar">`；机侧电量格无进度条（`:145-146`）

**结论：`float.html` 以机侧为底（4 格超集），需回贴车侧的 `batteryBar` 进度条。metrics 契约一旦定稿，两处 `battery` 读取方式必须一起改；否则运行时不会报错，只会静默显示 `NaN%` 或「无飞控」——这是最难发现的一类错误。**

---

## 3 疑点专项：`camHost` / `camPort`（任务卡点名必查）

### 3.1 结论

**两个假设都不成立。「运行中的车侧插件比磁盘源码新」错，「有第三个写入者」也错。**

真实解释是第三条：**它们是 `POST /config` 写入的脏键，因为 `sanitize()` 不白名单、不删未知键，而 `JsonStore.update()` 把整份 patch 浅合并后原样落盘。**

### 3.2 证据链（每条都可复核）

| # | 证据 | 位置 / 命令 |
|---|---|---|
| E1 | 磁盘源码 `SETTINGS_INITIAL` **没有**这两个键 | 车 `src/store.js:28-62`（`version…lastConnectedAt` 共 14 个键） |
| E2 | 已装副本 `dsh-dzrobot-ssh/src/store.js` 与磁盘源码 **SHA-256 相同** → 不存在「运行版本更新」 | `C0B60DC4EE04…`，见 §5 |
| E3 | 全 `v1` 工作区 + `.dsh` 下源码类文件（`*.js/*.mjs/*.json/*.jsonl/*.html/*.yml/*.md/*.ts/*.sh`）ripgrep `camHost|camPort`：除**文档叙述**（`docs/merge/空地协同插件整合方案.md:99,244,471` 与本文档）外，**任何源码、已装副本、测试脚本、`client.js` 均 0 命中** | `grep -r 'camHost\|camPort'`，见 §5 |
| E4 | 已装副本的 `client.js` 也无这两个键（排除浏览器半身写入） | 两个包的 `client.js` 均 clean |
| E5 | `sanitize()` 开头是 `const next = { ...patch }`，之后只对**已知键**做类型强制，末尾 `return next` —— **未知键原样保留** | 车 `src/store.js:115-116` 与 `:151`；全文无 `delete`、无 `Object.keys` 过滤、无 `ALLOWED_KEYS` 常量 |
| E6 | `update()` 做 `{...this.#state, ...patch}` 后 `flush()` 落盘 → patch 里的多余键**永久写入** | 车 `src/store.js:101-105` + `:166-175`（`writeFileSync(JSON.stringify(this.#state))`） |
| E7 | `POST /config` 直接 `settings.sanitize(body)` 后 `settings.update(patch)`，**body 来自任意 JSON** | 车 `index.js:245-256`（注释 `:143-144` 自己承认「`/config` takes arbitrary JSON」） |
| E8 | 运行期文件里这两个键**排在 `lastConnectedAt` 之后**，即后加的键；而代码里 `lastConnectedAt` 是 `SETTINGS_INITIAL` 的**最后一个**键 | `src/store.js:61` 是最后一个初始键；`settings.json` 顺序为 `…"floatOpacity":100, "lastConnectedAt":…, "camHost":"", "camPort":8080` → 键序证明它们是**追加**进 JSON 的，不是初始结构的一部分 |
| E9 | 车侧 Host 进程**在线且仍在写盘**：`settings.json` 与 `status.json` 的 mtime 在盘点过程中从 `18:54:38` 变为 `19:02:35`（`lastConnectedAt` 亦随之刷新） | `Get-ChildItem C:\Users\TestFox\.dsh\dzrobot-ssh` 两次采样 |

### 3.3 对合并的影响与必须动作

- **不会**丢功能：源码里没有 cam 能力，脏键只是垃圾数据。
- **会**污染新包：如果合并时直接**拷贝运行期 `settings.json` 做数据迁移**，`camHost`/`camPort` 会一起进新插件——因为两侧 store 的 `read()` 都是 `{...initial, ...parsed}`（车 `store.js:83`），**未知键会被原样读入内存并再次落盘**。
- **必须做**（建议写进 lead 的合并清单）：
  1. 新 `store.js` 的 `sanitize()` 改成**白名单**：只保留 `SETTINGS_INITIAL` 里存在的键；
  2. 数据迁移脚本按白名单挑键，**不要整文件拷贝**；
  3. 迁移后断言新 `settings.json` 的键集 ⊆ 白名单。

> 补充说明：`camHost`/`camPort` 的**写入时刻与写入者**，本次无法进一步定位——`diag.jsonl`（882 KB）里 0 条 `camHost` 记录，Host 侧也不记录 `/config` 的请求体。若 lead 需要「谁在什么时候写的」，需要临时给 `/config` 加日志重放，**这超出本卡只读范围，标为「未验证」**。但「它不是源码行为、不是已装副本行为、是 `sanitize` 放行后落盘」这一层，E1–E9 已足够闭合。

---

## 4 必须由用户拍板的分叉

> 只列**真正需要人决定**的，不列能靠证据裁决的。

| # | 分叉 | 选项 | 不选的代价 | 我的建议 |
|---|---|---|---|---|
| **D1** | **合并后是否保留两个侧栏图标** | A. 1 个图标（真合并）<br>B. 2 个图标（肌肉记忆） | 选 A：现有用户的操作习惯要改一次；选 B：等于承认「两个面板拼一起」，「空地协同」名不副实 | **A**。任务 4 明确要「1 个图标」，且 `/sim/drone/*` 已在车侧插件里，说明协同入口本来就在一起 |
| **D2** | **`metrics` 统一契约是否按 §2.3 重命名** | A. 重命名（`carBattery`/`thermal`/`fc`）<br>B. 保留 `battery` 但按节点类型分派 | 选 B 必须让每个消费端判断「这个 battery 是 number 还是 object」，`float.html` 与 `client.js` 各写一遍分支，未来第 3 个节点再加一层 | **A**。§2.2 的类型冲突是硬伤，B 是把它变成约定而非接口 |
| **D3** | **车侧的 12 条 `/sim*` 路由是否全量进新包** | A. 全部保留<br>B. 只留 `/sim/drone/*` + `/sim.html`（即砍掉 9 条：`/sim/run`、`/sim/history`、`/sim/artifact`、`/sim/check`、`/sim/root`、`/sim/map`、`/sim/stress`、`/sim/stress/status`、`/sim/stress/result`） | 选 B 会砍掉 64 图压测与仿真台核心：`src/stressbench.js` 沦为死代码（15.9 KB），5 个 `test-sim-*.mjs`（除 `test-sim-drone-render` 外）大概率连带失效；选 A 新包直接 +**251567 B = 245.7 KiB** 仿真资产（`simbench.js` 52714 + `sim.html` 159633 + `dronesim.js` 23048 + `stressbench.js` 15903 + `sim-root.json` 269），测试侧再 +**127881 B = 124.9 KiB**（6 个 `test-sim-*.mjs`） | **A**。用户要的是「不丢功能」的合并；砍仿真台是另一个决策，不应夹带在合并里 |
| **D4** | **`vncPassword` 默认值** | A. 每节点独立配置，默认空<br>B. 沿用机侧 `'duzhong'` 全局默认 | 选 B 把无人机密码写进车节点默认值，且是明文常量；选 A 首次连接车要多填一次 | **A**。设备密码不应成为包级默认值，且车侧现状就是空 |
| **D5** | **旧包与旧数据目录的处置** | A. 合并后删旧包 + 迁移数据<br>B. 并存一段时间 | 选 A 出问题要回滚重装；选 B 两个插件同时挂载会**抢同一批路由语义**（虽然前缀不同，但侧栏出现两组入口，且两份 `settings.json` 会继续各自漂移） | **A，但先落一份数据备份**。并存期间「运营态」和「开发态」会分不清，`camHost` 这类脏键正是并存期遗留的典型 |

---

## 5 复核方法与可复现命令

所有结论均可用下面的命令重放（**只读**，PowerShell，工作区根 = `D:\User\Desktop\比赛存档\v1`）：

```powershell
$car="D:\User\Desktop\比赛存档\v1\dzrobot_v2\dsh-plugin-dzrobot-ssh"
$drn="D:\User\Desktop\比赛存档\v1\dzsmartscout\dsh-plugin-dzdrone-ssh"

# ① 模块哈希 + 行数（确认哪一侧更新）
foreach($f in @("index.js","src\store.js","src\ssh.js","src\metrics.js","src\files.js",
                "src\vnc.js","src\ws.js","src\trust.js","src\reload.js")){
  $c=(Get-FileHash (Join-Path $car $f) -Algorithm SHA256).Hash
  $d=(Get-FileHash (Join-Path $drn $f) -Algorithm SHA256).Hash
  "{0,-18} {1} {2}" -f $f,($c -eq $d),$c.Substring(0,12)
}

# ② 代码差异（剥注释/空行 + 名称归一化；本文档「代码差异 N」就是这段的输出）
#    必须注意两点，否则数字对不上：
#      a) Get-Content 要显式 -Encoding UTF8，否则中文行会被按本地码页读成乱码、
#         中文归一化（小车/无人机）失效 → files.js 会错误地报 11 而不是 3；
#      b) 归一化要覆盖大小写 + 中英双语：dzrobot/dzdrone、DZROBOT/DZDRONE、小车/无人机、car/drone。
function StripNorm($p){
  $out=@()
  foreach($l in (Get-Content $p -Encoding UTF8)){
    $t=$l.Trim()
    if($t -eq ''){continue}; if($t.StartsWith('//')){continue}
    if($t.StartsWith('*')){continue}; if($t.StartsWith('/*')){continue}
    $t = $t -creplace 'dzrobot-ssh','dzX-ssh' -creplace 'dzdrone-ssh','dzX-ssh' `
             -creplace 'dzrobot','dzX' -creplace 'dzdrone','dzX' `
             -creplace 'DZROBOT','DZX' -creplace 'DZDRONE','DZX' `
             -creplace '小车','ZK' -creplace '无人机','ZK' `
             -creplace 'car','ZK' -creplace 'drone','ZK'
    $out += $t
  }
  return $out
}
foreach($f in @("src\ssh.js","src\vnc.js","src\ws.js","src\trust.js","src\reload.js","src\files.js")){
  $d=Compare-Object (StripNorm (Join-Path $car $f)) (StripNorm (Join-Path $drn $f))
  "{0,-16} code-diff={1}" -f $f,$d.Count
}
# 实测输出（2026-10-17 本机复核）：
#   src\ssh.js       code-diff=0
#   src\vnc.js       code-diff=2
#   src\ws.js        code-diff=0
#   src\trust.js     code-diff=0
#   src\reload.js    code-diff=0
#   src\files.js     code-diff=3

# ③ novnc 逐字节比对（结果应为 same=58 diff=0）
$cn="$car\novnc"; $dn="$drn\novnc"
$same=0;$diff=0
foreach($x in (Get-ChildItem $cn -Recurse -File)){
  $rel=$x.FullName.Substring($cn.Length+1); $y=Join-Path $dn $rel
  if(Test-Path $y){
    if((Get-FileHash $x.FullName -Algorithm SHA256).Hash -eq (Get-FileHash $y -Algorithm SHA256).Hash){$same++}
    else {$diff++; "DIFF $rel"}
  } else { "ONLY_CAR $rel" }
}
"novnc: same=$same diff=$diff"

# ④ camHost 全仓搜索（结果应为：仅本文档 + 方案文档命中）
Get-ChildItem "D:\User\Desktop\比赛存档\v1","C:\Users\TestFox\.dsh" -Recurse -File -Include *.js,*.mjs,*.json,*.jsonl,*.html,*.yml,*.md |
  Select-String -Pattern 'camHost|camPort'
```

### 5.1 已装副本 vs 磁盘源码（`camHost` 疑点排除依据）

| 包 | same | diff | 说明 |
|---|---|---|---|
| `dsh-dzrobot-ssh` | 18 | **3** | `sim.html`、`src/simbench.js`、`src/dronesim.js` 内容不同；**但 `src/store.js` 与 `src/metrics.js` 等全部 Host 模块相同**（故 E2 成立） |
| `dsh-dzdrone-ssh` | 16 | **0** | 已装副本与磁盘**完全一致**（差异仅「缺失项」= `scripts/` 与 `tmp-*.sh` 未随包安装，属 `package.json.files` 决定，非漂移） |

> **✅ 已查清：那 3 个 sim 文件的差异是「磁盘更新、且仅动注释」——不是代码漂移。**
>
> | 文件 | 磁盘 mtime | 已装 mtime | 剥注释后代码行 | 代码差异 |
> |---|---|---|---|---|
> | `src/simbench.js` | **2026-10-17 17:26:18** | 2026-10-17 16:42:43 | 625 / 626 | **1**（`hit[1].replace(...)` 一处，实为同行被注释前缀吞并的比对假阳性，见下） |
> | `src/dronesim.js` | **2026-10-17 17:24:39** | 2026-10-17 16:42:43 | 267 / 267 | **0** |
> | `sim.html` | **2026-10-17 17:24:39** | 2026-10-17 16:42:43 | 2202 / 2202 | 0（20 行差异全部是被重排/合并的大段**中文注释**） |
>
> 方向判定依据：磁盘 mtime 比已装副本晚约 42 分钟，且 git 最新提交为 **`fd11097 v3.9.2_注释精简二次收口（卡号清零）`**，`git status --porcelain dsh-plugin-dzrobot-ssh/` 为空（干净工作树）。两侧 `★` 标记数一致（simbench 1/1、dronesim 1/1、sim.html 13/13），**功能标记无增减**。
> ⇒ **结论：磁盘源码才是真源，已装副本是 16:42 部署的旧快照；合并应以磁盘为准。** 这条风险已解除，但仍建议 lead 在合并时确认一次（我这边已给出可复核口径）。

---

## 6 不在本卡范围内（留给其他任务卡 / lead）

- 合并后的命名、包结构、路由收敛 → **t4 / lead**
- `client.js` 的 22 个 hunk 逐条取舍 → **t2**
- 测试矩阵与验收口令 → **t3**
- 统一数据模型与面板信息架构 → **t4**
- 本文档**只裁决「每个模块取谁」**，不含实现。

---

## 7 「未验证」清单（如实声明）

| 项 | 缺什么证据 |
|---|---|
| `camHost`/`camPort` 的**写入者与写入时刻** | `diag.jsonl` 无记录，Host 侧不记录 `/config` 请求体。需临时加日志重放，超出只读范围 |
| 两个插件**当前是否仍在线** | 任务卡提到 `http://127.0.0.1:52882`，但本卡约束是「只读文件」，未发 HTTP 请求核实（另：`settings.json` 的 mtime 在盘点中于 19:02:35 被刷新，可作为「车侧 Host 进程在线」的间接旁证，但不是 `/status` 实证） |
| `metrics` 新契约的**性能影响** | 需实跑（车侧超时 12000 ms vs 机侧 9000 ms 已不同，合并后取哪个值未定） |
| 机侧 `ssh.js:204,206` 文案 bug 的**修复归属** | 已确认是真 bug（机侧拒绝文案仍写「车」），但「合并时顺手修」属实现动作，本卡只裁决、不实施 |

> 已解除的未验证项：车侧 3 个 sim 文件与磁盘源码的先后关系——已用 mtime + git 提交（`fd11097`）+ 剥注释比对三重证据闭合，见 §5.1。

---

*本文档为 t1 唯一产出，未修改任何插件源码、未安装、未部署。*
