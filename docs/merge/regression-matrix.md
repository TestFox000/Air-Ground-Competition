# t3 · 回归测试矩阵（合并前盘点：哪些测试会碎、碎在哪一行、怎么最小修）

> 交付人：regression-keeper ｜ 任务卡：task-3 ｜ 日期：2026-10-17
> 约束遵守：**只读**，本轮未修改任何测试/插件文件、未安装、未部署、未跑 pnpm、未实跑测试。
> 所有结论均指到 `文件:行号`；无法从静态证据判定的，明确标「待实跑验证」。

---

## 0. 结论先行（合并后最小可接受测试集）

1. **只有 2 条测试会因为改名而"结构性地"碎**：`test-host.mjs`（两侧各一份，硬编码 `/api/dzXX-ssh`）与 `test-client.mjs`（两侧各一份，硬编码插件名/CSS 前缀/tab id/bridge path）。其余测试**要么完全不碰这些标识符，要么只碰字符串常量而非行为**。
2. **6 个 `test-sim-*.mjs` 一个都不会因 `client.js` 改名而碎** —— 它们**全部读 `sim.html` / `index.js` / `src/*.js`，没有任何一个 import 或读取 `client.js`**（证据见 §3）。它们真正会被打破的是 **`sim.html` 里的 `var API = '/api/dzrobot-ssh'`（`sim.html:421`）**，属于产物改名，不是测试改名。
3. **发现一个既存缺陷（与合并无关但必须知道）**：`test-host.mjs` 的「目录穿越被拒绝」用例**在两个插件的当前代码下都是空转通过** —— `%2e%2e/%2e%2e` 在 `new URL()` 阶段就被规范化成 `/api/package.json`，根本没进 `serveNovnc` 的守卫（实测证据见 §4）。合并**不会让它变坏，但也别把它当成"穿越防护已被测试覆盖"**。
4. **合并后最小可接受测试集 = 6 条**（都在车侧，机侧能力由车侧同名测试覆盖）：

   | # | 测试 | 为什么是它 |
   |---|---|---|
   | 1 | `test-host.mjs` | 唯一覆盖 Host 全路由（status/config/metrics/files/novnc/vnc tunnel）的集成测试 |
   | 2 | `test-client.mjs` | 唯一覆盖客户端注册/渲染/tab/浮窗/connector/bridge 的集成测试 |
   | 3 | `test-vnc-wrapper.mjs` | 唯一守 noVNC `WebSocket` 子类判据（1006 的根因） |
   | 4 | `test-vnc-guard.mjs` | 唯一守缓存地址 + RFB banner 校验 |
   | 5 | `test-vnc-connect.mjs` | 唯一守 `vnc-connect.html` 的 postMessage 协议 |
   | 6 | `test-package-files.mjs` | 唯一守 `package.json#files` 与安装副本一致性（"打包后浏览器 404"那类事故） |

   **再加 2 条条件项**：`test-sim-map-render.mjs` + `test-sim-drone-render.mjs`（若合并保留仿真台；两者合计 92 断言，覆盖 `sim.html` 地图/无人机渲染，是纯前端唯一可离线验证的手段）。

5. **最容易漏的坑**：`test-package-files.mjs` 的 `RUNTIME_ASSETS`（车 `:46`）/`EXACT_FILES`（机 `:48`）是**手写白名单**。合并会把 `sim.html`/`sim-root.json`/`src/simbench.js` 这些**车侧独有文件**带进新包，白名单**不会自动跟着长** —— 新包一旦漏声明，文件不打包、测试**照样全绿**（因为它只查声明，不查"应该有哪些"）。

---

## 1. 断言计数方法（可复核）

不同测试用 4 种断言惯用法，**必须分开数**，否则数字没意义：

| 惯用法 | 特征 | 计数口径 |
|---|---|---|
| **A. `check(label, ok, detail)`** | 有布尔参数，直接计数 | 数 `check(` **调用点**（排除 `function check(`/`const check =` 定义），**再按外层 `for` 的迭代次数展开** |
| **B. `check(label, run)` + `throw`** | 有 try/catch，断言写在闭包里的 `throw new Error` | 数 `check(` 调用点；闭包内 `throw` 是**失败消息**不是独立断言 |
| **C. `if (...) renderFailures.push(...)`** | 车/机 `test-client.mjs` 用这套 | 数 `(renderFailures\|failures).push(` **调用点**，按外层 `for` 展开 |
| **D. `assert.*`** | `node:assert/strict` | 数 `assert.` 调用点 |

**复核命令**（PowerShell，只读，**已在 PowerShell 5.1 实测过、数字对得上**）：

```powershell
$f = 'dzrobot_v2\dsh-plugin-dzrobot-ssh\scripts\test-host.mjs'

# A/B 类：check( 调用点（排除定义行）
#  ★ 注意必须用 -cmatch（case-sensitive）：文件末尾 486 行有
#    "HOST CHECK(S) FAILED" 里的大写 "CHECK("，用 -match 会多算 1 个
(Get-Content $f | Where-Object {
  $_ -cmatch '(?<![\w.])check\s*\(' -and $_ -notmatch 'function\s+check\s*\(|const\s+check\s*='
}).Count
# → 50

# C 类：push( 调用点（对 test-client.mjs）
$c = 'dzrobot_v2\dsh-plugin-dzrobot-ssh\scripts\test-client.mjs'
(Select-String -Path $c -Pattern '(renderFailures|failures)\.push\(' -AllMatches).Matches.Count
# → 106

# D 类：assert.
(Select-String -Path $f -Pattern '\bassert\.' -AllMatches).Matches.Count
# → 0（car test-host 不用 assert）
```

> **⚠️ 我已踩过的坑（记下来免得复核时重复）**：`Select-String` 的 `-Pattern` **默认不区分大小写**，
> 而 `Where-Object { $_ -match ... }` 的正则**也不区分大小写**。数 `check(` 时这会让
> `test-host.mjs:486` 的 `HOST CHECK(S) FAILED` 被算进去（50 → 51）。
> **计数一律用 `-cmatch`**；`Select-String` 若需区分大小写用 `-CaseSensitive`。

> **计数口径说明**：下表的「断言数」= **静态调用点**（`站点`）与**循环展开后的断言数**（`展开`）两个数。展开只在循环体包含断言、且迭代次数**能从字面量数组直接求出**时计算；迭代次数依赖运行期数据的（如 `for (const cell of cells)` 依赖渲染结果、`Object.entries(registered)` 依赖注册槽位数）只记 1，因此**client 类的「展开」是下界**。

**展开计数用的是这个脚本**（Node，只读；它把 `for (... of [字面量数组])` 里的数组真的 `eval` 出来取 `.length`，不是猜）：

```js
// node count-expanded.cjs <file>
const fs = require('node:fs')
const src = fs.readFileSync(process.argv[2], 'utf8')
const loops = []; const re = /for\s*\(\s*(?:const\s+)?(\[[^\]]*\]|\w+)\s+of\s+/g; let m
while ((m = re.exec(src)) !== null) {
  const lineNo = src.slice(0, m.index).split(/\r?\n/).length
  const at = m.index + m[0].length
  let iters = null
  if (src[at] === '[') {                                  // 字面量数组 → 求值取长度
    let d = 0, i = at
    for (; i < src.length; i++) { if (src[i]==='[') d++; else if (src[i]===']') { d--; if(!d){i++;break} } }
    try { const arr = eval(src.slice(at, i).replace(/client\.__test\.DICT\.(zh|en)/g,'({})'))
          if (Array.isArray(arr)) iters = arr.length } catch {}
  }
  const bo = src.indexOf('{', m.index); let d2 = 0, be = bo     // 循环体范围（花括号配平）
  for (let i = bo; i < src.length; i++) { if (src[i]==='{') d2++; else if (src[i]==='}') { d2--; if(!d2){be=i;break} } }
  loops.push({ lineNo, iters, bodyStart: bo, bodyEnd: be })
}
let sites = 0, expanded = 0
const pre = /(?:renderFailures|failures)\.push\(/g
while ((m = pre.exec(src)) !== null) {
  sites++
  const encl = loops.filter(l => m.index > l.bodyStart && m.index < l.bodyEnd)
  let prod = 1, rt = false
  for (const l of encl) { if (l.iters === null) rt = true; else prod *= l.iters }
  expanded += rt ? 1 : prod
}
console.log({ sites, expandedLowerBound: expanded })
```

> **⚠️ 我踩过的三个坑（复核时别再踩）**
> 1. `Where-Object { $_ -match }` 与 `Select-String -Pattern` **都不区分大小写** → `test-host.mjs:486` 的 `HOST CHECK(S) FAILED` 会被算成第 51 个 `check(`。**必须用 `-cmatch`**。
> 2. **外层 `for` 的迭代次数会重复乘内层**：`:592`（2 语言）套 `:593`（26 key）时，手工算容易漏乘或重复乘。用上面的脚本让 `encl` 链自动相乘。
> 3. **多行数组**（`:240`、`:267`、`:680`、`:569`、`:799`）不能只看首行 —— 首行只写 `for (const [label, component] of [`，`]` 在 8 行之后。脚本用括号配平取了整段。

---

## 2. 总表（逐测试）

### 2.1 车侧 A：`dzrobot_v2\dsh-plugin-dzrobot-ssh\scripts\`

| 测试文件 | 行数 | 跑法 / 前置 | 断言（站点→展开） | 合并后必然要改的硬编码 | 合并后归谁 | 必须改什么 |
|---|---|---|---|---|---|---|
| **test-host.mjs** | 486 | `node scripts/test-host.mjs`；`DSH_HOME`=自建 temp（`:19`）；**起本机 loopback 端口，不需真车**；可选 `DZR_TEST_CAR_VNC=1` 才真连车（`:455`） | **A 类 50 站点 → 62 展开**（`:205` 8 项 ×1 + `:217` 6 项 ×1：`50 - 2 + 8 + 6 = 62`） | `/api/dzrobot-ssh`（`:82` base）<br>`/api/dzrobot-ssh/novnc/%2e%2e/...`（`:314`）<br>`ws://.../api/dzrobot-ssh/vnc`（`:424`、`:459`）<br>`source:'dzrobot-vnc-connect'`（`:336`）<br>`'duzhong'`/`'duzhong.local'`（`:110`,`:347`,`:382`,`:476-477`） | **留在空地协同**（改名不改逻辑） | 4 处前缀替换为 `/api/dzairground-ssh`（`:82`,`:314`,`:424`,`:459`）+ `:336` 协议名改为 `dzairground-vnc-connect`。**注意 `:336` 是测 `vnc-connect.html` 的产物内容，改测试必须同步改产物**（见 §5） |
| **test-client.mjs** | 749 | `node scripts/test-client.mjs`；`DSH_HOME`=temp（`:21`）；起真 Host loopback（`:94`）**并跑一次真 SSH probe** → **需要车可达，否则 `:419` 失败**（这行是真实设备依赖，不是改名问题） | **C 类 106 站点 → ≥199 断言（下界）**：80 站点在循环外各算 1；11 站点在**字面量**循环内按 `L240×6, L425×8, L465×2, L570×5, L571×2, L584×6, L588×6, L592×2, L593×26, L680×3, L729×8` 展开；15 站点在**运行期**循环内（`registered` 槽位数 / `cells` 渲染数）只算 1 → **下界** | `client.name !== 'dzrobot-ssh'`（`:77` **throw，最硬的一条**）<br>`activePanel='dzrobot-car'`（`:117`、`:443`，**作为 PANEL_ID 值，不是断言**）<br>`dzr_*` CSS 断言 **26 处**（`:310`,`:325`,`:329`,`:337`,`:342`,`:345`,`:353`,`:356`,`:374`,`:388`,`:729`,`:735`,`:738`…）<br>`'dsh-dzrobot-ssh/vnc'`/`'/files'`（`:455`,`:460`,`:465`）<br>`'dzrobot-float'`（`:481`）<br>`'api/dzrobot-ssh/vnc'` + `'carvnc'`（`:540-542`,`:545`,`:549`）<br>`'dzr-files'`/`'dzr-vnc'` kind（`:445`,`:448-449`,`:453`）<br>`--dzr-op`（`:388`,`:729`） | **留在空地协同**（**最大的一块改动**） | ① `:77` 插件名<br>② 26 处 `dzr_` → 新前缀（**如果 CSS 前缀选 `dza_`**）<br>③ `:455/:460/:465` tab id<br>④ `:481` overlay id<br>⑤ `:542` bridge path + `:540/545/549` 密码常量<br>⑥ `:445/448/449/453` 工具 kind<br>⑦ `:729/735/738` CSS 选择器<br>⑧ `:117/:443` stub 里的 PANEL_ID 值（只影响 stub 保真度，断言本身在 `:431/454` 查 `null`） |
| **test-package-files.mjs** | 141 | `node scripts/test-package-files.mjs`；读 `package.json#files`；**装副本存在时才查副本**（`:79-81` 读 `DSH_HOME`+`DZR_PROFILE`） | **A 7 站点 → 展开 19**：`:55`,`:95`,`:108`,`:124` 各 1 = 4；`:62`,`:71`,`:87` 在 `RUNTIME_ASSETS`（**5 项**）循环内 = 3 × 5 = 15；→ **4 + 15 = 19**（另有 10 处 `throw` 位于循环内，是**失败消息**不独立计数） | `:46 RUNTIME_ASSETS = ['vnc-app.html','novnc','float.html','src/metrics.js','src/store.js']`（**手写白名单**）<br>`:125` 逐字节比对列表 `['index.js','client.js','src/vnc.js','src/ws.js','src/metrics.js','src/store.js','float.html']`<br>`:81` `node_modules/${pkg.name}`（跟随包名，**自动）** | **留在空地协同** | ① `:46` 补合并新增的 `sim.html`/`vnc-connect.html`/`sim-root.json`/`src/simbench.js`/`src/dronesim.js`<br>② `:125` 补新文件名<br>③ **注意**：它只验证"声明了的确实在"，**不验证"该有的都声明了"** → 白名单漏项会假绿（见结论 5） |
| test-vnc-connect.mjs | 264 | `node scripts/test-vnc-connect.mjs`；读 `../vnc-connect.html` 源码 + DOM/RFB 桩，**不起服务** | **A 16 + D 14 assert + 13 throw = 43** | `:115 href='.../api/dzrobot-ssh/vnc-connect.html?embedded=1'`（**只影响 `location.search` 解析，路径前缀无用**）<br>`:132 source !== 'dzrobot-vnc-connect'`（**throw，硬**）<br>`:143`,`:146`,`:172`,`:214`,`:217` `source:'dzrobot-panel'`（5 处） | **留在空地协同** | `:132` + 5 处 `dzrobot-panel` + `:115` 前缀。**必须与产物 `vnc-connect.html` 同步改**（见 §5） |
| test-vnc-guard.mjs | 176 | `node scripts/test-vnc-guard.mjs`；`import '../src/vnc.js'`，本机起 TCP 桩 + `*.invalid` 域名 | **A 6 站点 = 6**（`:83`,`:110`,`:113`,`:130`,`:149`,`:164`） | 只有 **测试自用的假域名**：`'dzrobot-guard-ok.invalid'`（`:77`）等 4 个 | **留在空地协同**（原样可用） | **零改动**（假域名与产品标识符无关；改名只为整洁，非必须） |
| test-vnc-wrapper.mjs | 164 | `node scripts/test-vnc-wrapper.mjs`；`import '../novnc/core/websock.js'`；读 `vnc-app.html`+`vnc-connect.html` 的 `ws-log` 块 | **A 6 站点 → 展开 10**（`:71`,`:79` 各 1 = 2；`:118`,`:128`,`:134`,`:144` 在 `DOCUMENTS` 2 元素循环内 = 4×2 = 8） | `:35 DOCUMENTS=['vnc-app.html','vnc-connect.html']`（**文件名，不是标识符**）<br>`:93-94` `/* ws-log:start */` 标记 | **留在空地协同** | **零改动**（只要产物保留两个 html 与 ws-log 标记）—— **这是最容易被误判"要改"的一条，实际不用改** |
| **test-live-ws.mjs** | 44 | `node scripts/test-live-ws.mjs [url]`；**连接真实运行中的 DSH 端口**，无断言、只打印 | **0**（纯诊断脚本，`:36` 打印 RESULT） | `:8 'ws://127.0.0.1:19387/api/dzrobot-ssh/vnc'`（**默认参数**） | **保留为诊断工具**（不是回归测试） | `:8` 默认 URL。**不计入最小集**（无断言、无退出码语义） |
| check-files.mjs | 101 | `node scripts/check-files.mjs [host] [user]`；**需要真车 SSH**（`:21` 打 `ssh`、`:36`/`:74` `execFileSync`） | **A 13 站点 = 13** | `:9-10 'duzhong.local'`/`'duzhong'`；`:21`/`:87` `/home/duzhong` | **保留为现场诊断**（非 CI） | 无标识符需改（**设备地址与插件改名无关**） |
| check-ssh.mjs | 28 | `node scripts/check-ssh.mjs [host] [user] [port]`；**真连车** | **0**（打印 `splitOptions`/`interactiveArgv`/`probe` 结果） | `:12-14 'duzhong.local'`/`'duzhong'` | **保留为现场诊断** | 无 |
| check-divergence.mjs | 100 | `node scripts/check-divergence.mjs`；读 profile `package.json` 的 `file:` 依赖 | **A 2 站点 + 1 计数 = 3**（`:39` check 函数内 `failures+=1`；`:43` profile 声明；后续多树比对） | `:32 pkg.name`（**自动**）<br>比对"其他像本插件的树"时按目录名/内容判定 | **必须留**（合并期尤其重要） | 逻辑跟随 `pkg.name` 自动适配；**但合并后要更新它"合法副本"的判定目录**（新增 `dzairground` 树） |
| deploy.mjs | 53 | `node scripts/deploy.mjs`；**会写安装副本**（本卡未跑） | **0**（不是测试） | `:18 'dsh-dzrobot-ssh'`（**硬编码安装目录名**） | **改为新包名** | `:18` → `dsh-dzairground-ssh`；`DZR_PROFILE` 环境变量保留 |

### 2.2 车侧 B：6 个仿真台测试（`test-sim-*.mjs`）

> **核心问题回答见 §3。这里只列断言与受影响面。**

| 测试文件 | 行数 | 数据来源（读完即知） | 断言 | 会碎吗 | 归谁 | 必须改什么 |
|---|---|---|---|---|---|---|
| test-sim-drone-render.mjs | 205 | `sim.html`（`:23`）、`index.js`（`:48`）、`src/dronesim.js`（`:51`）、`src/simbench.js`、`src/dronesim.js`（import） | **A 31 站点 = 31** | **否** | 留（若保留仿真台） | **零改动**（不碰前缀/插件名） |
| test-sim-map-render.mjs | 632 | `sim.html`（`:45` `:70` `:97`）、`src/simbench.js`（`:42`） | **A 61 站点 = 61** | **否** | 留（若保留仿真台） | **零改动** |
| test-sim-overlay-fix.mjs | 239 | `sim.html`（`:22`）、`src/simbench.js`（`:23`,`:177`）；找 `dzrobot_v2` 工作区根（`:34-41`，判据 `src/dzbrain`+`config/robot.yaml`）；可选 `python`（`:202`） | **A 31 站点 = 31** | **否** | 留 | **零改动**（`findWorkspaceRoot` 从插件目录**上溯**找工作区，合并后插件若仍在 `dzrobot_v2/` 下则自动成立） |
| test-sim-overlay-ab.mjs | 379 | `sim.html`（`:38`）；`git show HEAD:...sim.html`（`:219`，**需在 git 仓库内**）；本机 DOM 桩 | **A 24 站点 = 24** | **否** | 留 | **零改动**（`:219` 的 `rel` 由 `path.relative(repoRoot, PLUGIN)` 算，目录名变了**自动跟随**） |
| test-sim-incremental.mjs | 281 | `src/simbench.js`（`:33`）、`src/dronesim.js`（`:53`）、`config/robot.yaml`+`map.yaml`+`tools/plan_route.py`（`:55-59`，经 `SIM_ROOT`）；**需 `python`**（`:176`,`:213`） | **A 24 站点 = 24** | **否** | 留 | **零改动**；但**依赖 `SIM_ROOT` 解析**（见 §3.2 风险） |
| test-sim-parity.mjs | 256 | `src/simbench.js`（`:33`）、`plan_route.py`+`config/*.yaml`（`:57-59`）、`map.yaml`（`:191`）；**需 `python`**（`:65-70` 无则 `exit 2`） | **A 27 站点 = 27**（`:105` 的 `for` 只 `push` 到 `created[]`，**不含断言**；断言在 `:107-110` 各一次） | **否** | 留 | **零改动**；同样依赖 `SIM_ROOT` |

### 2.3 机侧：`dzsmartscout\dsh-plugin-dzdrone-ssh\scripts\`

| 测试文件 | 行数 | 跑法 / 前置 | 断言（站点→展开） | 合并后必然要改的硬编码 | 合并后归谁 | 必须改什么 |
|---|---|---|---|---|---|---|
| **test-host.mjs** | 539 | `node scripts/test-host.mjs`；`DSH_HOME`=temp（`:19`）；loopback，**不需真机**；`DZD_TEST_DRONE_WRITE=1` 才写文件（`:335`）、`DZD_TEST_DRONE_VNC=1` 才真连（`:437`） | **A 54 站点 = 54**（机侧无循环内含断言的站点，唯一的 `for` 在 `:401` 是造测试数据） | `/api/dzdrone-ssh`（`:82`）<br>`/api/dzdrone-ssh/novnc/%2e%2e/...`（`:278`）<br>`ws://.../api/dzdrone-ssh/vnc`（`:406`、`:441`）<br>`'dzsmart'`（`:360` 断言 `/etc/hostname` 内容） | **合并为一份**（推荐：以车侧为基础，吸收机侧特有断言） | 3 处前缀（`:82`,`:278`,`:406`,`:441`）+ `:360` 的 `'dzsmart'` 需按节点参数化（机侧特有资产：`isTemp`/`noFc`/`dds_silent`/`no_ros` 断言在 `:161-233`，**必须保留**） |
| **test-client.mjs** | 813 | 同车侧；起真 Host + **真 SSH probe** → `:633` 需机可达 | **C 类 119 站点 → ≥203 断言（下界）**：91 站点在循环外各算 1；13 站点在字面量循环内按 `L240×6, L435×6, L439×2, L440×18, L443×2, L452×6, L569×3, L620×8, L639×8, L679×2, L799×6, L800×2` 展开；15 站点在运行期循环内只算 1 → **下界** | `client.name !== 'dzdrone-ssh'`（`:77`）<br>`dzd_*` **28 处**（`:311`,`:326`,`:332`,`:336`,`:344`,`:349`,`:352`,`:360`,`:363`,`:381`,`:400`,`:620`,`:626`,`:629`…）<br>`'dsh-dzdrone-ssh/vnc'`/`'/files'`（`:669`,`:674`,`:679`）<br>`'dzdrone-float'`（`:695`）<br>`'api/dzdrone-ssh/vnc'`+`'dronevnc'`（`:760-762`,`:771`,`:775`）<br>`'api/dzdrone-ssh/vnc-connect.html'`（`:794`）<br>`'dzd-files'`/`'dzd-vnc'` kind（`:659`,`:662-663`,`:667`）<br>`--dzd-op`（`:400`,`:620`） | **合并为一份** | 与车侧同构的 8 类改动。**机侧独有断言必须保留**：`:332` `dzd_fbatt`（电量）、`:339` `temp,cpu` cells、`:443` `floatPopup`/`floatPopupHint`、`:799` `connDroneBridge*`+`vncConnectorTitle` |
| **test-package-files.mjs** | 121 | `node scripts/test-package-files.mjs`；装副本不存在则 `SKIP`（`:90`） | **A 5 站点 → 14**：`:61`,`:104` 各 1 = 2；`:68`,`:77`,`:93` 在 `RUNTIME_ASSETS`（**4 项**）循环内 = 3 × 4 = 12；→ **2 + 12 = 14**（另 7 处 `throw` 位于循环内，是失败消息不独立计数） | `:45 RUNTIME_ASSETS = ['vnc-app.html','vnc-connect.html','float.html','novnc']`<br>`:48 EXACT_FILES`（9 项，含 `src/files.js`）<br>`:87 pkg.name`（自动） | **合并为一份** | ① `:45` 与 `:48` 取两侧并集（车侧有 `src/metrics.js`/`src/store.js`，机侧有 `src/files.js`/`vnc-connect.html`）<br>② **两侧白名单不一致本身就是要合并的证据**（车 5 项 vs 机 4 项） |
| test-vnc-connect.mjs | 264 | 同车侧（两侧**行数相同但内容改名**） | **A 16 + D 14 + 13 throw = 43** | `:115` 前缀、`:132 'dzdrone-vnc-connect'`、`:143`,`:146`,`:172`,`:214`,`:217 'dzdrone-panel'` | **合并为一份**（车侧那份即可，改名后覆盖机侧） | 同车侧 6 处 |
| test-vnc-guard.mjs | 177 | `import '../src/vnc.js'`；本机 TCP 桩 | **A 6 = 6** | `'dzdrone-guard-*.invalid'` 4 个假域名（`:78`,`:97`,`:125`,`:143`） | **合并为一份** | **零改动**（假域名无意义差异） |
| test-vnc-wrapper.mjs | 164 | 同车侧 | **A 6 → 展开 10**（同车侧） | 仅 `DOCUMENTS` 文件名 | **合并为一份** | **零改动** |
| test-live-ws.mjs | 44 | 同车侧（纯诊断） | **0** | `:8 '/api/dzdrone-ssh/vnc'` | 留作诊断 | `:8` |
| check-files.mjs | 101 | **真连机**（`dzsmart.local`，`:9-10`） | **A 13 = 13** | `:9-10`、`:21`、`:87` `/home/duzhong` | 留作诊断 | 无 |
| check-ssh.mjs | 28 | **真连机**（`:12-14 'dzsmart.local'`） | **0** | 设备地址 | 留作诊断 | 无 |
| **check-widget-target.mjs** | 28 | `node scripts/check-widget-target.mjs`；读 `~/.dsh/dzdrone-ssh/settings.json`（`:12`） | **0**（`metrics.ok !== true` 则 `exit 1`，`:21-24`） | `:12 join(homedir(),'.dsh','dzdrone-ssh','settings.json')`（**硬编码数据目录**） | **改为新路径** | `:12` → `.dsh/dzairground-ssh/`（且 settings 结构变 `{car,drone}`，**要加节点选择**）。**机侧独有，车侧无对应物** |

---

## 3. 关键核实①：6 个 `test-sim-*.mjs` 测的到底是什么？

### 3.1 结论：**全部读 `sim.html` / `index.js` / `src/*.js`，没有一个读 `client.js`**

**证据（全仓 grep，逐文件）**：

```powershell
# 6 个 sim 测试里，谁提到 client.js？
Get-ChildItem dzrobot_v2\dsh-plugin-dzrobot-ssh\scripts\test-sim-*.mjs |
  ForEach-Object { "$($_.Name): " + (Select-String -Path $_.FullName -Pattern 'client\.js' -AllMatches).Matches.Count }

# 谁提到 sim.html？
Get-ChildItem dzrobot_v2\dsh-plugin-dzrobot-ssh\scripts\test-sim-*.mjs |
  ForEach-Object { "$($_.Name): " + (Select-String -Path $_.FullName -Pattern 'sim\.html' -AllMatches).Matches.Count }
```

**实测结果**：

| 测试 | `client.js` 引用 | `sim.html` 引用 | 实际读取的文件（行号） |
|---|---|---|---|
| test-sim-drone-render.mjs | **0** | 3 | `sim.html:23`、`index.js:48`、`src/dronesim.js:51`、`src/simbench.js` / `src/dronesim.js`(import) |
| test-sim-map-render.mjs | **0** | 9 | `sim.html:45,70,97`、`src/simbench.js:42` |
| test-sim-overlay-fix.mjs | **0** | 2 | `sim.html:22`、`src/simbench.js:23,177` |
| test-sim-overlay-ab.mjs | **0** | 9 | `sim.html:38`、`git show HEAD:.../sim.html:219` |
| test-sim-incremental.mjs | **0** | 1（仅注释） | `src/simbench.js:33`、`src/dronesim.js:53`、`config/*.yaml`、`tools/plan_route.py` |
| test-sim-parity.mjs | **0** | **0** | `src/simbench.js:33`、`config/*.yaml`、`tools/plan_route.py`、`map.yaml:191` |

> 对照：**只有** `test-client.mjs`（车 `:516`、`:523` 等 3 处；机 4 处）与 `test-package-files.mjs`（2 处）读 `client.js`。这两个测试**读的是源码做字符串断言**（如 `:516` 切片 `makeVncTabShell`，`:723` 提 `const CSS = ...`），所以它们会随 `client.js` 内容变化而**对内容敏感**，但**文件名固定为 `client.js`**，改名不影响（除非连文件名都改）。

### 3.2 那么合并会打破 sim 测试吗？会，但**不是测试改名，是产物改名**

| 断链点 | 位置 | 后果 |
|---|---|---|
| `sim.html` 硬编码 API 前缀 | `sim.html:421 → var API = '/api/dzrobot-ssh'` | 页面所有 `api()` 调用打到旧前缀 → **404**。sim 测试**大多不测这一行**（它们是 DOM 桩，`fetchStub` 直接 `Promise.resolve({})`，如 `test-sim-map-render.mjs:73`），**所以测试仍然全绿而产品已坏** —— 这是**假绿风险最高的一处** |
| `index.js` 前缀剥离正则 | 车 `index.js:145 → replace(/^\/api\/dzrobot-ssh/,'')` | 不改则新前缀路由全 404 |
| `SIM_ROOT` 解析（**高危**） | `src/simbench.js:59-67`：①`DZROBOT_SIM_ROOT` ②`PLUGIN_DIR/..` ③`sim-root.json` 指针；判据 `looksLikeWorkspace() = exists(dir/src/dzbrain)` | 合并后插件目录**若不再位于 `dzrobot_v2/` 下**，② → 解析到错的树，③ → 靠 `sim-root.json`（车侧已有该文件，**内容当前指 `D:\...\v1\dzrobot_v2`**）。**必须确保新包 `files` 含 `sim-root.json` 且内容指向 `dzrobot_v2`** —— 否则 `test-sim-incremental`/`parity`/`overlay-fix` **会 `exit 2`**（`test-sim-incremental.mjs:58-59`、`test-sim-parity.mjs:62-63`） |

> **可复核的 `sim-root.json` 现状**：`dzrobot_v2\dsh-plugin-dzrobot-ssh\sim-root.json` 存在，内容 `{"root":"D:\\User\\Desktop\\比赛存档\\v1\\dzrobot_v2", ...}`。机侧目录**没有** `src/dzbrain`/`config/*.yaml`/`tools/plan_route.py`（实测均 `False`），所以**仿真台能力只在车侧**。

---

## 4. 关键核实②：noVNC 目录穿越用例合并后还测到同一逻辑吗？

### 4.1 结论：**该用例在当前两侧代码下都是"空转通过"，合并不会让它变坏，但它并没有测到它声称的东西**

**用例本体**（车 `test-host.mjs:312-319`；机 `:276-283`，逻辑完全相同）：

```js
http.request({ ..., path: '/api/dzrobot-ssh/novnc/%2e%2e/%2e%2e/package.json' })
check('encoded path traversal is refused', traversal === 403 || traversal === 404, ...)
```

**实测（用 Node 复现处理链，未跑测试、未改文件）**：

```
RAW      /api/dzrobot-ssh/novnc/%2e%2e/%2e%2e/package.json
pathname /api/package.json                 ← new URL() 已把 %2e%2e 规范化掉
route    /api/package.json | enters serveNovnc: false
```

`index.js:145` 的正则剥离后 `route = '/api/package.json'`，既不等于任何已知端点，也**不满足** `:387` 的 `route.startsWith('/novnc/')` → 直接落到 `:390 return send(404, {error:'no route ...'})`。断言 `403 || 404` **因为 404 而通过**，**`serveNovnc` 的守卫（`:479-506`）一次都没执行**。

**能真正打到守卫的形状**（实测）：

```
RAW      /api/dzrobot-ssh/novnc/%2e%2e%2f%2e%2e%2fpackage.json     ← 斜杠也编码
pathname /api/dzrobot-ssh/novnc/%2e%2e%2f%2e%2e%2fpackage.json
route    /novnc/%2e%2e%2f%2e%2e%2fpackage.json  | enters: true
guard    decoded="../../package.json" → target="D:\package.json" → inside=false → 403 forbidden ✓

RAW      /api/dzrobot-ssh/novnc/..%2f..%2fpackage.json
route    /novnc/..%2f..%2fpackage.json | enters: true
guard    decoded="../../package.json" → 403 forbidden ✓
```

**合并后怎么办**：

- 前缀改成 `/api/dzairground-ssh` 后，`%2e%2e/%2e%2e` 形状**仍然**在 `new URL()` 被规范化 → 仍然 404 通过。**"合并是否还测到同一逻辑"的答案是：它本来就没测到。**
- **最小修复（建议一并做，属于"测试变强"而非"改名"）**：把 `:314`（车，机 `:278`）的 path 改成 **`%2e%2e%2f%2e%2e%2f`（斜杠也编码）** 或 `..%2f..%2f`，断言收紧为 **`=== 403`**。这样才真正执行到 `serveNovnc` 的 `inside` 判据。
- 若合并后**路由表变成 `/:role/*`**（方案 A §3.2），则路径变为 `/api/dzairground-ssh/car/novnc/...` — 此时 `:387` 的 `startsWith('/novnc/')` 判据**也要跟着改**，否则 novnc 静态资源整体 404（**比穿越漏洞更严重**）。

---

## 5. 改名影响面清单（按产物 vs 测试，防止"只改测试不改产物"）

> **这是合并最容易出事的地方**：测试与产物里存在**同一批字符串的镜像**。只改一侧 = 测试假绿或产品真坏。

| 字符串 | 产物位置 | 测试位置（必须同步） |
|---|---|---|
| `/api/dzrobot-ssh` | `index.js:145,401,412,421`、`client.js`（10 处）、`sim.html:421`、`float.html`、`vnc-app.html`、`trust.js:4` | 车 `test-host.mjs:82,314,424,459`、`test-live-ws.mjs:8`、`test-vnc-connect.mjs:115` |
| `/api/dzdrone-ssh` | 机 `index.js:135,330,341`、`client.js`、`float.html`、`vnc-app.html`、`trust.js` | 机 `test-host.mjs:82,278,406,441`、`test-live-ws.mjs:8`、`test-vnc-connect.mjs:115`、`test-client.mjs:794` |
| `dzrobot-vnc-connect`（postMessage source） | 车 `vnc-connect.html`（2 处） | 车 `test-host.mjs:336`、`test-vnc-connect.mjs:132` |
| `dzdrone-vnc-connect` | 机 `vnc-connect.html`（2 处） | 机 `test-host.mjs`（锚点）、`test-vnc-connect.mjs:132` |
| `dzrobot-panel` / `dzdrone-panel` | `vnc-connect.html`、`vnc-app.html` | 车 `test-vnc-connect.mjs:143,146,172`；机同结构 |
| `dsh-dzrobot-ssh/vnc|files`（tab id） | 车 `client.js:2745`、`:11` 注释 | 车 `test-client.mjs:455,460,465` |
| `dsh-dzdrone-ssh/vnc|files` | 机 `client.js:2710,2722` | 机 `test-client.mjs:669,674,679` |
| `dzrobot-ssh`（插件 name） | 车 `index.js:42` | 车 `test-client.mjs:77` |
| `dzdrone-ssh` | 机 `index.js:39` | 机 `test-client.mjs:77` |
| `dzrobot-car`（PANEL_ID） | 车 `client.js:44` | 车 `test-client.mjs:117,443`（stub 值） |
| `dzdrone-drone`（PANEL_ID） | 机 `client.js:43` | 机（同结构） |
| `dzrobot-float` / `dzdrone-float`（overlay id） | 两侧 `client.js` overlay 注册 | 车 `:481`；机 `:695` |
| `dzr_*` / `dzd_*`（CSS 前缀） | 车 `client.js` 337 处；机 339 处 | 车 `test-client.mjs` 26 处；机 28 处 |
| `dzrobot-ssh`（数据目录） | `src/store.js:21 DATA_DIR_NAME` | 机 `check-widget-target.mjs:12` |
| `dzrobot-ssh`（安装目录名） | `scripts/deploy.mjs:18` | — |
| `dzr-files`/`dzr-vnc`/`dzd-*`（工具 kind） | `client.js` TOOLS | 车 `:445,448,449,453`；机 `:659,662,663,667` |
| `carvnc` / `dronevnc`（VNC 默认密码） | `client.js`（bridge 默认） | 车 `:540,545,549`；机 `:760,771,775` |
| `sim-root.json` 文件本身 | 车插件根（`files` 已声明） | `test-sim-*` 经 `simbench.js:46` 间接依赖 |

---

## 6. 合并后验收口令（PowerShell，可直接复制）

> **前置**：合并完成、新包已 `pnpm install` 到 profile、DSH 已重启。以下命令**全部只读**（`test-package-files` 会读安装副本；`deploy.mjs` 会写，**故不在验收序列里**）。
> **变量**：把 `$AG` 改成新包实际路径。所有测试都在**插件根目录**下跑（测试用 `path.resolve(HERE,'..')` 定位产物）。

### 6.1 一次性冒烟（确认没装错树）

```powershell
$ErrorActionPreference = 'Stop'
$AG  = "D:\User\Desktop\比赛存档\v1\dzairground\dsh-plugin-dzairground-ssh"   # ← 改成实际路径
$PROFILE = 'desktop'

Write-Host "=== 0. 包身份 ===" -ForegroundColor Cyan
$pkg = Get-Content "$AG\package.json" -Raw | ConvertFrom-Json
"package name : $($pkg.name)"
"files 声明数 : $($pkg.files.Count)"
if ($pkg.name -ne 'dsh-dzairground-ssh') { throw "包名不符：$($pkg.name)" }

Write-Host "=== 1. 安装副本存在且是最新 ===" -ForegroundColor Cyan
$installed = Join-Path $env:USERPROFILE ".dsh\profiles\$PROFILE\node_modules\$($pkg.name)"
if (-not (Test-Path $installed)) { throw "未安装：$installed" }
"installed : $installed"
foreach ($f in @('index.js','client.js','package.json')) {
  if (-not (Test-Path (Join-Path $installed $f))) { throw "安装副本缺 $f" }
  $a = (Get-FileHash "$AG\$f" -Algorithm SHA256).Hash
  $b = (Get-FileHash (Join-Path $installed $f) -Algorithm SHA256).Hash
  if ($a -ne $b) { Write-Warning "★ 源树与安装不等：$f（需重新安装，否则测的是旧代码）" }
}
```

### 6.2 最小可接受测试集（6 条，必须全绿）

```powershell
Write-Host "=== 2. 最小可接受测试集 ===" -ForegroundColor Cyan
$minimal = @(
  'test-host.mjs',            # Host 全路由
  'test-client.mjs',          # 客户端注册/渲染/tab/浮窗
  'test-vnc-wrapper.mjs',     # noVNC WebSocket 子类判据
  'test-vnc-guard.mjs',       # 缓存地址 + RFB banner
  'test-vnc-connect.mjs',     # connector postMessage 协议
  'test-package-files.mjs'    # files 声明 ↔ 安装副本
)
$fail = @()
Push-Location $AG
try {
  foreach ($t in $minimal) {
    $p = Join-Path $AG "scripts\$t"
    if (-not (Test-Path $p)) { $fail += "$t (缺失)"; Write-Host "MISS $t" -ForegroundColor Yellow; continue }
    Write-Host "`n----- $t" -ForegroundColor DarkGray
    $sw = [Diagnostics.Stopwatch]::StartNew()
    node $p
    $code = $LASTEXITCODE
    $sw.Stop()
    if ($code -ne 0) { $fail += "$t (exit $code)"; Write-Host "FAIL $t exit=$code  $($sw.Elapsed.TotalSeconds.ToString('0.0'))s" -ForegroundColor Red }
    else { Write-Host "PASS $t  $($sw.Elapsed.TotalSeconds.ToString('0.0'))s" -ForegroundColor Green }
  }
} finally { Pop-Location }

if ($fail.Count -gt 0) { Write-Host "`n✗ 最小集未通过：" -ForegroundColor Red; $fail | ForEach-Object { "   $_" }; exit 1 }
Write-Host "`n✓ 最小可接受测试集 6/6 全绿" -ForegroundColor Green
```

### 6.3 仿真台（若合并保留 `/sim/*`；**需要 `python` + `dzrobot_v2` 工作区**）

```powershell
Write-Host "=== 3. 仿真台（条件项） ===" -ForegroundColor Cyan
# 前置：SIM_ROOT 必须解析到 dzrobot_v2，否则会 exit 2
node -e "import('./src/simbench.js').then(m=>console.log('SIM_ROOT =',m.SIM_ROOT,'\nsource   =',m.SIM_ROOT_SOURCE))"

foreach ($t in @('test-sim-map-render.mjs','test-sim-drone-render.mjs')) {
  Write-Host "`n----- $t" -ForegroundColor DarkGray
  node (Join-Path $AG "scripts\$t"); if ($LASTEXITCODE -ne 0) { Write-Warning "$t exit $LASTEXITCODE" }
}
# 需 python 的两条（无 python 会 exit 2 = 环境前置不满足，不是回归）
$py = (Get-Command python -ErrorAction SilentlyContinue)
if ($null -eq $py) { Write-Warning "无 python：test-sim-incremental / test-sim-parity 无法验（会 exit 2，非回归失败）" }
else {
  foreach ($t in @('test-sim-incremental.mjs','test-sim-parity.mjs','test-sim-overlay-ab.mjs','test-sim-overlay-fix.mjs')) {
    Write-Host "`n----- $t" -ForegroundColor DarkGray
    node (Join-Path $AG "scripts\$t"); $c=$LASTEXITCODE
    switch ($c) { 0 { Write-Host "PASS $t" -ForegroundColor Green } 2 { Write-Warning "$t 环境前置不满足 (exit 2)" } default { Write-Warning "$t exit $c" } }
  }
}
```

### 6.4 合并后必须**新增**的守卫（防止假绿，建议在 S3/S4 阶段补）

```powershell
Write-Host "=== 4. 假绿体检（新增守卫） ===" -ForegroundColor Cyan

# 4.1 产物里不得残留旧前缀（唯一消费者 sim.html 我们说过自己改）
Write-Host "`n[4.1] 旧前缀残留检查"
$stale = Select-String -Path "$AG\*.html","$AG\client.js","$AG\index.js","$AG\src\*.js" `
  -Pattern 'dzrobot-ssh|dzdrone-ssh' -AllMatches -ErrorAction SilentlyContinue
if ($stale) { Write-Warning "发现旧标识符 $($stale.Count) 处："; $stale | Select-Object -First 12 | ForEach-Object { "  $($_.Filename):$($_.LineNumber)  $($_.Line.Trim())" } }
else { Write-Host "OK 无残留" -ForegroundColor Green }

# 4.2 sim.html 的 API 常量必须与新前缀一致
Write-Host "`n[4.2] sim.html 的 API 常量"
$simApi = Select-String -Path "$AG\sim.html" -Pattern "var API = '([^']+)'"
if ($simApi) { "  $($simApi.Line.Trim())" } else { Write-Warning "  sim.html 里找不到 var API =" }

# 4.3 package.json#files 必须覆盖所有运行时资产（白名单漏项是最大假绿源）
Write-Host "`n[4.3] files 覆盖体检（对照实际存在的运行时资产）"
$declared = (Get-Content "$AG\package.json" -Raw | ConvertFrom-Json).files
$mustExist = @('index.js','client.js','vnc-app.html','vnc-connect.html','float.html','novnc','src\store.js','src\ssh.js')
$mustExist += @('sim.html','sim-root.json','src\simbench.js','src\dronesim.js')   # ← 若保留仿真台
foreach ($m in $mustExist) {
  if (-not (Test-Path (Join-Path $AG $m))) { Write-Warning "  产物缺 $m（files 声明了也无意义）"; continue }
  $rel = $m -replace '\\','/'
  $hit = $declared | Where-Object { $rel -eq $_ -or $rel.StartsWith("$_/") }
  if (-not $hit) { Write-Warning "  ★ $m 存在但未被 files 声明 → 打包后会丢" } else { Write-Host "  OK $m" -ForegroundColor Green }
}

# 4.4 noVNC 穿越守卫真的执行了吗（见 §4：现用例是空转）
#     注意：这里用单引号 here-string @'...'@ —— 脚本里含 $ 与反引号，双引号 here-string 会吃掉它们
Write-Host "`n[4.4] 穿越守卫有效形状自检（不需要起服务）"
$js = Join-Path $env:TEMP 'ag-traversal-shape.js'
@'
const path = require('node:path')
const NOVNC = 'D:\\fake\\novnc'
function verdict (raw) {
  const u = new URL(raw, 'http://x')
  const r = u.pathname.replace(/^\/api\/dzairground-ssh/, '').replace(/\/+$/, '') || '/'
  if (!r.startsWith('/novnc/')) return 'NOT-REACHED  route=' + r
  let d
  try { d = decodeURIComponent(r.slice('/novnc/'.length)) } catch { d = '' }
  const t = path.resolve(NOVNC, d)
  return (t === NOVNC || t.startsWith(NOVNC)) ? 'INSIDE(served)' : '403 forbidden'
}
for (const p of [
  '/api/dzairground-ssh/novnc/%2e%2e/%2e%2e/package.json',
  '/api/dzairground-ssh/novnc/%2e%2e%2f%2e%2e%2fpackage.json',
]) {
  console.log(verdict(p).padEnd(22), p)
}
'@ | Set-Content -Path $js -Encoding UTF8
node $js
Remove-Item $js -Force
Write-Host "  期望：第 1 个 NOT-REACHED（空转，当前用例就是这个形状），第 2 个 403（真正执行守卫）" -ForegroundColor Yellow
```

### 6.5 现场端到端（需要 DSH 运行中 + 设备可达）

```powershell
Write-Host "=== 5. 端到端（DSH 必须在跑） ===" -ForegroundColor Cyan
$base = 'http://127.0.0.1:52882'      # ← 与 GUI URL 同端口
foreach ($path in @('/api/dzairground-ssh/status','/api/dzairground-ssh/metrics')) {
  try {
    $r = Invoke-WebRequest -Uri "$base$path" -TimeoutSec 10 -UseBasicParsing
    "$path -> $($r.StatusCode)  $($r.Content.Length)B"
  } catch { Write-Warning "$path -> $($_.Exception.Message)" }
}
# 关键：双节点必须各自如实（一个掉线不许显示假绿）
try {
  $s = Invoke-RestMethod -Uri "$base/api/dzairground-ssh/status" -TimeoutSec 10
  "car   : $($s.nodes.car.state)   drone : $($s.nodes.drone.state)"
} catch { Write-Warning "status 读不到" }
```

### 6.6 诊断工具（不是回归测试，故障时用）

```powershell
# 实时 VNC 隧道（连运行中的 DSH；只是打印，无断言）
node "$AG\scripts\test-live-ws.mjs" "ws://127.0.0.1:52882/api/dzairground-ssh/vnc"

# 现场设备层核对（需车/机可达）
node "$AG\scripts\check-ssh.mjs"   duzhong.local duzhong 22
node "$AG\scripts\check-files.mjs" duzhong.local duzhong
```

---

## 7. 未决 / 待实跑验证

| 项 | 为什么没定 | 需要什么才能定 |
|---|---|---|
| `test-client.mjs` 的两侧 `connect()` 需要**真设备**（车 `:419`、机 `:633`） | 静态可确认依赖，但**无法确认现场是否可达**。若不可达，这两条在 CI 里**必然红**，属环境而非合并问题 | 实跑一次，看 `connect did not succeed` 的 `reason`（`terminal-services-missing` / `probe` 的 state） |
| 循环展开后的**精确**断言总数（车 client ≥199、机 ≥203） | 这些是**下界**：15 个站点位于运行期循环内（`Object.entries(registered)` 的槽位数、`for (const cell of cells)` 的渲染 cell 数），静态不可知。字面量循环已按真实数组求值（脚本见 §1 备注） | 实跑时统计 `FAILURES:` 行数（仅失败时可见），或临时在 `check`/`push` 两侧加计数器（**属于改动测试，本卡不做**） |
| `test-sim-overlay-ab.mjs` 的历史臂 | `git show HEAD:...sim.html`（`:219`）在**插件目录不是 git 仓库或路径变更**时会走"Skipped"分支（`:234`）。合并后若新包不在 `dzrobot_v2` 仓库内，**历史臂静默跳过**（`info`，不算失败） | 实跑看 `0. 实验素材` 那行打印的 `历史臂：` 内容 |
| CSS 前缀最终选 `dza_` 还是沿用 `dzr_` | 属 lead/用户拍板（方案文档 §6.5 给了三选一）。**选 `dzr_` 可让车侧 26 处 + 机侧 28 处 CSS 断言零改动**；选 `dza_` 则两侧都要改 54 处 | 用户决定 |
| `test-package-files.mjs` 能否发现"该有的没声明" | **不能**（只验证"声明了的在"）。这是**既存设计缺陷**，不是合并引入 | 若要有牙：新增"运行时资产清单必须与 files 双向匹配"断言（见 §6.4 的 4.3） |

---

## 8. 给 lead 的三条行动建议

1. **改名成本排序**：`test-host.mjs`（4 处前缀）→ `test-vnc-connect.mjs`（6 处）→ `test-package-files.mjs`（白名单）→ `test-client.mjs`（**26~28 处 CSS + 8 类标识符，占全部工作量的 70%**）。**如果接受"CSS 前缀沿用 `dzr_`"，`test-client.mjs` 的改动量直接砍掉一大半**（这也是方案文档 §6.5 提到的实惠）。
2. **两侧 `test-client.mjs` 必须人工合并，不能机械二选一**：机侧独有 `dzd_fbatt`（电量，`:332`）、`temp,cpu` cells（`:339`）、`floatPopup*` DICT（`:443`）、`connDroneBridge*`+`vncConnectorTitle`（`:799`）四组断言，车侧没有；车侧独有 `makeCarPanel`/sim 相关。**删掉任何一侧都会丢覆盖**。
3. **`test-host.mjs` 的穿越用例建议顺手修强**（§4），否则合并后这份"安全测试"会继续给人虚假的安全感；同时若路由改 `/:role/*`，`:387` 的 `/novnc/` 判据**必须改**，那是功能性 404 而非安全问题。
