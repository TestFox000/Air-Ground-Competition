# 图传页 UI 改造验收报告（t4）

- **任务**：t4 图传页 UI 改造 —— 删轨迹地图 / 删常用参数 / 删速度控制 / 布局美化
- **执行成员**：ui-polish，attempt 1（`e5e70ac3-9eca-43ce-8ba6-2c98b1574d43`）
- **日期**：2026-10-11
- **产物**：`tools/cam_viewer.html`（64,075 B，sha256 `33545DC20130AFC0D1BB438615912AC344382559BE8A1EC70D8958267823D785`）+ 本报告
- **纪律**：未 scp 推送车端、未运行 `start_all.sh`、未改 `_scratch_displayfix/`、`_tmp_session/` 基线原件、未 git commit

---

## 0. 基线来源（必须先读）

按任务卡优先级逐个检查：

| 候选基线 | 大小 | 判定 |
|---|---|---|
| `_scratch_displayfix/cam_viewer_car.html` | 82,626 B，mtime 2026-10-11 16:30，sha256 `929AF969A31F25393B37F7005FECEC9D52DEDFCD6502833A54E13B2A24693698` | ✅ **采用**（队友已抓取的车端当前版） |
| `_tmp_session/cam_viewer_e906.html` | **0 B（空文件）** | ❌ 不可用 |
| `_tmp_session/cam_viewer_baseline_1858.html` | 155,622 B | 未使用（优先级更低） |

**风险标注**：本次基线是队友 16:30 抓取的车端副本，非本次会话直接从车端抓取（车端在 VNC/内网侧，本会话无 SSH 通道；推送与车端复验由 t3 统一执行）。若车端在 16:30 之后又被改过，`tools/cam_viewer.html` 与车端最新版可能存在差异 —— t3 推送前建议先 diff 车端当前 `/opt/.../cam_viewer.html`（或服务实际路径）与本产物，若不一致以车端为基线重放本报告第 1~4 节的改动点清单。

---

## 1. 改动点清单

### ① 删除轨迹地图视图

| 类别 | 删除内容 |
|---|---|
| HTML | `#cellMap` 整块（`<canvas id="mapcv">`、`.maplegend` 图例、`btnReloadMap` 重载地图按钮、`mapName/mapInfo/mapCur` 信息位） |
| HTML | 画面切换条 `#viewSwitch`（`data-view="video"` / **`data-view="map"`** 按钮 + "V 键切换 · 地图每 250ms 刷新" 提示） |
| JS | `const TRACK_URL='/api/track'`、`pollMap()`（250ms 轮询）、`drawMap()`（14×11 网格/雷障靶通/射击点★/倒车/轨迹折线绘制，约 145 行）、`btnReloadMap` 的 `/api/reload_map` 调用 |
| JS | `switchView()` + `curView` + `#viewSwitch` 点击绑定；启动段 `switchView(hash==='map'…)`、`pollMap()`、`setInterval(pollMap,250)` |
| JS | 快捷键 **`V`** 分支（`if (k === 'v') switchView(...)`） |
| CSS | `#mapcv`、`.maplegend`、`.viewswitch`、`.vsbtn`、`.cell` 的"叠放切换"语义（`.cell` 保留为视频单元格样式） |

### ② 删除「常用参数」面板 + 搜索过滤入口

| 类别 | 删除内容 |
|---|---|
| HTML | 主区底部 `.quickset` 整块（标题"常用参数"、`#qsGrid`、`#qsApply` 应用改动、`#qsReset` 放弃改动、`#qsMsg`） |
| HTML | 侧边栏 `.sidefilter` + `<input id="cfgSearch" placeholder="搜索参数名 / 分组…">` |
| JS | `QUICK_KEYS`（9 项快捷参数表）、`QUICK`、`quickFind()`、`renderQuick()`、`markQuick()`、`qsApply`/`qsReset` 两个 async 提交器（约 145 行）；`loadConfig()` 与 `applyGroup()`/`qsReset` 里对 `renderQuick()` 的 3 处调用 |
| JS | `#cfgSearch` 的 input 过滤处理器（同时过滤 8 组参数卡与常用参数行） |
| CSS | `.quickset/.qshead/.qsgrid/.qsit/.qsfoot`、`.sidefilter` |

> 侧边栏「参数调校」页（8 组参数纵览 + 每组"应用"写回 robot.yaml）**保留**，任务只要求删主区的常用参数条与搜索框。

### ③ 接管面板删除速度控制（保留接管/释放、方向、开火、急停）

| 删除 | 保留 |
|---|---|
| `#rcSpeed` 滑条（`<input type="range" min=0.05 max=0.30>`）、`#rcSpeedVal` 数值、"速度"标签行、`.rcrow input[type=range]` CSS、`rcSpeed` 的 input 事件处理器 | `#rcToggle` 接管/释放、`#rcEstop` 紧急停止、小车 D-pad（前进/后退/左/右/刹车）、云台 D-pad（含回正）、`#rcFire` 开火、裁判系统面板、键盘 WASD/IJKL/空格 |

`rcTick()` 里的速度来源改为常量 `const RC_SPEED = 0.15;  // 遥控行进速度(m/s)：固定值，页面不再提供速度调节`（原来是 `parseFloat($('rcSpeed').value)`，若不改会抛 null 引用打死 10Hz 指令循环）。10Hz 喂狗/刹车逻辑、失焦即停、接管看门狗语义均未变。

### ④ 残余清理 + 布局美化

**残余/断键清理**
- 删除底部悬浮快捷键条 `.hint`（`position:fixed;bottom:3px`，在矮窗口下与侧边栏底栏争位）及其 `body.popup .hint` 规则与 `body.classList.add('popup')`；快捷键信息改为主区「快捷键」卡片，`<kbd>` 统一样式。
- 删除死样式 `.console{...}`（该容器 HTML 早已不存在）。
- JS 引用的 `id` 与 HTML 中的 `id` 做了交叉审计：**JS 引用但 HTML 不存在的 id = 0 个**；HTML 定义但 JS 不引用的只剩 `cellVideo`/`stage`（纯结构性锚点，无害）。

**t2 display-fix 规格集成（队长指令）**
- **根因**：车端页面原 JS `document.querySelector('.console').classList.toggle('estop', rcEstop)` 引用了已删除的 `.console` 容器 → 启动同步抛 TypeError → 其后整段尾部脚本（含 `reconnect()`）被杀 → 浏览器永远不发起 `GET /stream` → 页面卡在「连接中…」。
- **修复 1（JS）**：改挂实际存在的接管面板 `#paneRc`，并做空值兜底：
  `const rcPane = $('paneRc'); if (rcPane) rcPane.classList.toggle('estop', rcEstop);`
- **修复 2（CSS）**：`.console.estop .rcbar` → `#paneRc.estop .rcbar`（含 `::after "⚠ 急停生效中"`），急停红条恢复生效。
- **修复 3（加固）**：`reconnect()` 从脚本最末尾**提前**到视频区监听注册完之后立即调用；启动收尾段 `applySide()` / `loadConfig()` 各包 try/catch —— 下次再有 UI 代码引用不存在的容器，也不会连坐图传。

**布局美化（深色控制台风格）**
- 主区从「视频 ⇄ 地图切换 + 常用参数条」简化为 **「视频大窗 + 底部三卡栅格」**：`.main{display:flex;flex-direction:column;gap:10px}`，`.viewwrap`（`flex:1 1 auto`，视频等比 contain，`min-height:160px` 兜底）+ `.cardgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:10px;max-height:38vh;overflow-y:auto}`。
- 三张卡片：**识别 / 云台**（目标 tag、偏移/框/置信、云台状态）、**视频流**（话题、帧率、分辨率）、**快捷键**（kbd 分行）。卡片 = `.card`（panel 底 + line 描边 + 10px 圆角 + 渐变卡头），与侧边栏 `.grp` 参数卡同族样式。
- 视频操作（刷新/重连、标注/原始 seg）收进 `.cellhead`，与状态 pill 同一行；`.cellbody` 纯黑底居中，无叠层冲突。
- 无重叠/溢出保障：所有滚动区（`.sidescroll/.congroups/.cardgrid/.shots-body`）统一 `scrollbar-width:thin` + 显式 webkit 滚动条；`body{overflow:hidden}` + `.app` 100vh grid；卡片行在矮窗口内自身滚动而不是把视频挤没；旧的 fixed 底部 `.hint` 条（会压住侧边栏底栏）已删除。

**行数**：基线 2040 行 → 产物 1569 行（仅基线有 435 行 / 仅产物有 105 行）。

---

## 2. 保留完好（视频流与遥控接口）

| 接口 | 调用点 |
|---|---|
| `const STREAM_URL = '/stream'` | `video.src = STREAM_URL + '?mode=' + mode + '&t=' + Date.now()`（reconnect） |
| `const RC_URL = '/api/rc'` | `rcApi()` → `/drive`、`/gimbal`、`/takeover`、`/estop`、`/shoot`、`/referee`、`/reset_shots`、`GET ''`（状态回读） |
| `/api/status` | `poll()` 800ms（fps/分辨率/识别框/云台状态/开火记录） |
| `/api/config` | `loadConfig()` / `applyGroup()`（参数调校页读写 robot.yaml） |
| `/api/referee` | `refPoll()` 1s（弹药/HP/被击） |

标注/原始切换（`#segMode` → `?mode=ann|raw`）、重连状态机（connecting/live/nosignal/failed）、开火记录折叠条、失焦 panic 停车、10Hz 喂狗循环全部未改语义。

---

## 3. 静态校验证据（三条契约 verify 命令，全部 exit 0）

```
> node -e "...new Function(编译全部 <script> 块)..."
JS syntax OK, blocks: 1

> node -e "...div 开闭计数..."
div open 67 close 67

> node -e "...must=['/stream','api/rc','接管','急停']; gone=['常用参数','data-view=\"map\"']..."
content assertions OK
```

**补充断言（自建，非契约要求）**：

| 断言 | 基线命中 | 产物命中 |
|---|---|---|
| `常用参数` | 11 | **0** |
| `data-view="map"` | 1 | **0** |
| `rcSpeed` / `rcSpeedVal` | 6 / 2 | **0 / 0** |
| `mapcv` / `pollMap` / `drawMap` / `cellMap` / `viewSwitch` / `switchView` | 5/5/2/2/3/4 | **0/0/0/0/0/0** |
| `TRACK_URL` / `/api/track` / `reload_map` | 2 / 1 / 1 | **0 / 0 / 0** |
| `qsGrid` / `qsApply` / `qsReset` / `renderQuick` / `quickFind` / `markQuick` / `cfgSearch` / `sidefilter` / `quickset` / `vsbtn` / `canvas` | 4/4/2/4/2/5/2/4/2/6/2 | **全 0** |
| `.console` | 4（JS 选择器 1 + CSS 2 + 注释 1） | **3（仅 CSS/JS 注释里的修复说明，零代码引用）** |
| `#paneRc.estop` | 0 | 2（CSS 规则 + JS `$('paneRc')`） |
| JS `$('id')` 引用了不存在的 id | — | **0** |

---

## 4. 交给 t3 的注意事项

1. 推送前先与车端当前文件 diff，确认基线风险（见第 0 节）；推送后在 VNC 浏览器里应看到：连接状态由「连接中…」进入「已连接」（t2 根因已修），`GET /stream` 有请求。
2. 页面仍由服务端替换 `__POPUP__` 占位符，推送的是**未替换**的源文件，与原流程一致。
3. 速度现在是固定 0.15 m/s（`RC_SPEED` 常量，JS 顶部 RC 参数区可改）；如需现场调速，走侧边栏「参数调校」页的 `linear_speed` 等参数（写回 robot.yaml 并重启节点），而非遥控面板。
4. 未在真浏览器渲染截图（本会话无浏览器）；建议 t3 推送后在 VNC 内截一张全页图归档。

## 5. 只读纪律自查

- `git status --porcelain`：本任务新增/修改仅 `tools/cam_viewer.html`（untracked 新文件）与本报告；`src/`、`config/`、`tools/dzcam_server.py`、`tools/start_all.sh`、`_scratch_displayfix/`、`_tmp_session/` **零写入**（`_tmp_session` 内仅短暂存在两份 git 快照临时文件，已删除）。
  说明：同一工作区里 `src/dzbrain/**`、`tools/calib_backfill.py`、`tools/plan_route.py`、`docs/上下文快照.md` 等条目在本任务窗口内也有变动，那是其他成员/历史会话的既有未提交工作，本卡一次也没有写过这些路径。
- 未执行 scp / ssh / `start_all.sh`；未 git commit / push。
