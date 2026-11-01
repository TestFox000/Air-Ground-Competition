---
卡号: WB-6b-PANELUI-20261101
ns: wb-1031
agent: wb-studio
树节点: wb-6b
kind: implementation
status: in_progress
创建: 2026-11-01 11:5x
owner: wb-studio
reviewer: wb-reviewer
objective: 补齐 WB-6 的真实 UI 缺口（卡面 #4 / #6 两条 partial）：把**逐拍回放时间轴**（可拖滑条 + 地图车/信念车/命令条同步）与**双臂并排 A/B 面板**做进插件单页面板。数据层已完备（`parseStepsCsv` / `buildTimeline` / `armDiffRows` 都有断言），本卡**只做 UI 接线**，不重写任何分档/解析逻辑。
acceptance:
  - 插件根下那个单页面板出现可拖时间轴控件（`type="range"`），拖动时**地图车 / 信念车 / 命令条三者同步**该拍的位姿与指令
  - 时间轴读取 `/sim/rig/panel` 的 `view.timeline.frames`，**页面零解析**（不得自带 CSV 解析、不得自算 `loc_ran` 过滤、不得自算 RMS）
  - 双臂并排：插件根下那个单页面板出现并排两臂面板，消费 `armDiffRows` 的结果，展示 key/grade/a/b/delta，**`null` 与 `0` 视觉可区分**
  - `trajectory_state` 三档徽标保持现状（绿 PASS / 黄 UNPROVEN 带"未证明"字样 / 红 FAIL 带"确定性失败"），`null` 显示"状态未知"绝不默认 PASS
  - **页面零本地计算这条机器判据必须仍然全绿**（WB-6 已建的 5 条正则 + 反证 ≥4/5 命中）
  - 两条缺口必须有**可判据的 UI 断言**（DOM 桩）：拖动改变状态 / 并排面板存在且 `null≠0`
  - **补一条 WB-R4 发现的覆盖缺口（F1）**：`buildTimeline({locRanOnly:true})` 的 `dropped` 差异断言
    ——WB-R4 实测 MUT1（把该分支的 `!==1` 改成 `>1`）**未变红**，因测试里 `locRanOnly` 只作用于 `beliefErrorOf`。
    ⇒ **是覆盖缺口不是假绿**（`beliefErrorOf` 那条已被 MUT2 证明真会红），但必须补上
  - 断言只增不减；WB-6 的九张台测必须全部仍 rc=0
verify:
  - cd "D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh" && node scripts/test-wb6-panel.mjs; echo rc=$?
  - cd "D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh" && node scripts/test-wb6-sim-html.mjs; echo rc=$?
  - cd "D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh" && grep -c 'type="range"' "$PANEL_HTML"   # $PANEL_HTML=插件根下那个单页面板
  - cd "D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh" && node scripts/test-sim-map-render.mjs; node scripts/test-sim-parity.mjs; node scripts/test-sim-incremental.mjs; node scripts/test-sim-drone-render.mjs; node scripts/test-sim-overlay-fix.mjs; node scripts/test-intel-route.mjs; node scripts/test-wb6-rig-routes.mjs
inScope: `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/` 下那个单页面板；必要时同目录 `src/wb6_panel.js`（仅加纯函数）；新测试 插件 `scripts/` 下的新 UI 台测（本卡新建）；`_tmp_session/wb-1031/wb6b/`
outOfScope: `dzrobot_v2/**`（零写入）；插件 `src/` 下的 simbench 与 dronesim 两个模块（不动）；`config/**`
---

# WB-6b · 补齐面板 UI：逐拍回放时间轴 + 双臂并排

## 读过的台账条目

- WB-6 卡面 `:227/:229`（#4、#6 两条 `partial`，含实测 `grep = 0` 证据）。
- WB-6 事故卡 A1–A5；`§12.10` **L14**（缺失≠替代值）/**L15**（降级跳过掩盖下游）/**L19**（卡面与字段必须同源）。

## 为什么这张卡必须开

WB-6 的卡名是「**rig 的编排与逐拍回放**」，而 #4（逐拍回放）与 #6（双臂并排）正是卡名里写着的两条，实测：
```
grep -c 'type="range"' "$PANEL_HTML"   # $PANEL_HTML=插件根下那个单页面板              → 0
面板内 `armDiff|并排|side-by-side` 命中            → 0
```
⇒ **数据层齐备、页面看不见 = 对人没用。** 一个只能在 JSON 里看回放的仿真台，不如没有。

## 硬边界（不许越）

1. **不许重写分档/解析逻辑**。`parseStepsCsv` / `buildTimeline` / `armDiffRows` 已在 `wb6_panel.js` 有断言，页面只能**消费**它们的结果。
2. **页面零本地计算**那条机器判据必须仍然全绿（WB-6 建了 5 条正则 + 反证）。若本卡让页面开始自己算东西，该判据会立刻红 —— **那是设计要拦的，不是要绕过的**。
3. **`null` 与 `0` 视觉必须可区分**（WB-6 已把这条写进数据层，UI 不能退化成都显示 0）。

## 验收（独立复跑）

见 front matter `verify` 四条 + DOM 桩断言。改动者自报不算过。

## 回滚点

`_tmp_session/wb-1031/wb6b/preimage/`（插件单页面板与 `wb6_panel` 纯函数层两份文件改前逐字节 + sha256）；禁止 `git checkout`。

## 执行报告

### 0 · 开工三步（已执行）

| 步 | 动作 | 实测 |
|---|---|---|
| ① 记基线 | 十张台测改前 rc | map-render 61/0 · parity 21+8SKIP/0 · incremental 15+4SKIP/0 · drone-render 31/0 · overlay-fix 50/0 · intel-route 24/0 · wb6-panel 125/0 · wb6-rig-routes 27/0 · wb6-sim-html 42/0（改前 38）· **wb6b 尚不存在** |
| ② `git status` | 认领写域 | HEAD 插件已入 git；工作树在途项均为前轮产物，本卡只碰「单页面板 + `wb6_panel.js`（仅加纯函数）+ 新建 UI 台测」 |
| ③ 行尾 | 按字节判读 | 插件根下那个单页面板 **LF（CRLF=0）** · `src/wb6_panel.js` **LF** · 新建台测 **LF**；插件 `src/` 下的 simbench 仍是 **CRLF=1554/1554**（本卡未动） |

#### 0.1 先复现 WB-R4 的 F1 覆盖缺口（不采信转述）
```
把 buildTimeline 的 `!== 1` 改成 `> 1` → node scripts/test-wb6-panel.mjs → rc=0（125 passed）
```
⇒ **确认 F1 成立**：`locRanOnly` 只被 `beliefErrorOf` 覆盖，`buildTimeline` 那条分支此前**从未被断言**。

### 1 · 交付物（2 改 + 1 新）

| 文件 | 性质 | 内容 |
|---|---|---|
| `src/wb6_panel.js`（插件 src 下） | 改（**仅加纯函数**） | `timelineFrameAt`（拖条 index→三车视图，含夹紧标记）· `displayValue`（`null`/`0` 三态）· `armSideBySide`（双臂两侧各自 displayValue + `deltaKind` + `nAbsent`）；`frameOf` 增 `exec_v`/`exec_steer`/`dead_zone`/`pivot_phase` 透传；`panelViewOf` 增 `armsSideBySide` 与 `timelineStart` |
| 插件根下那个单页面板 | 改 | 可拖时间轴 `type="range"`（id `wb6TlRange`）+ 三车同步帧框 + 命令条（指令 vs 实际执行 + `pivot_phase`）+ 双臂并排面板（`wb6Arms`） |
| 插件 `scripts/` 下新建 UI 台测 | **新增** | `test-wb6b-panelui.mjs` **39 断言**（A 拖动三车同步 14 条 / B 双臂 null≠0 12 条 / C 补 F1 覆盖缺口 7 条含 MUT1 自证 / D 面板区零本地计算 6 条） |

### 2 · 十张台测（每步复跑，最终态）

```
test-sim-map-render   rc=0  61/61
test-sim-parity       rc=0  21/21 + 8 SKIP
test-sim-incremental  rc=0  15/15 + 4 SKIP
test-sim-drone-render rc=0  31/31
test-sim-overlay-fix  rc=0  50/50
test-intel-route      rc=0  24 passed / 0 failed / 1 skipped
test-wb6-panel        rc=0  125 passed / 0 failed
test-wb6-rig-routes   rc=0  27 passed / 0 failed
test-wb6-sim-html     rc=0  42 passed / 0 failed
test-wb6b-panelui     rc=0  39 passed / 0 failed   ← 本卡新增
```

### 3 · 验收逐条（卡面 acceptance 7 条 · **L19：逐行比对全表**）

| # | 条目 | 状态 | 证据 |
|---|---|---|---|
| 1 | 可拖时间轴控件 + 三车同步 | **PASS** | `type="range"` 实测命中 ≥1（改前 0）；`wb6TlRange` id；DOM 桩 A3–A11 **真触发 input 事件**：拖到第 2 拍后帧框从 `0.500/0.200` 变 `0.700/0.300`，信念车变 `0.7004/0.3003`，命令条变 `cmd_v=0.32`、`pivot_phase=TURN`；A8 反向断言第 0 拍**不含**第 2 拍坐标（证明真的变了而非堆叠） |
| 2 | 时间轴读 `view.timeline.frames`，页面零解析 | **PASS** | DOM 桩 D1/D3/D4：**台架面板区**内无 STEP_COLUMNS/无 TRAJECTORY_STATES/无 `Math.sqrt`/无 CSV split，且不调用宿主 `timelineFrameAt`；D2 反证同组正则在宿主命中 4/4 |
| 3 | 双臂并排 + `null` 与 `0` 视觉可区分 | **PASS** | `wb6Arms` 面板；B2 同一行 `a.kind=zero` / `b.kind=absent`；B3 两侧**文本不同**（`0` vs `—（无记录）`）；B8 absent 带灰斜体；B11 反向断言该行最多 1 个裸 `0`；B4 `nAbsent=1` 单独计数 |
| 4 | `trajectory_state` 三档徽标保持现状 | **PASS（保持）** | 本卡未改三档逻辑；`test-wb6-sim-html` C6–C9 仍全过（`UNPROVEN` 带"未证明"、`FAIL` 带"确定性失败"、缺字段 ⇒ 灰 UNKNOWN） |
| 5 | 页面零本地计算判据仍全绿 | **PASS** | `test-wb6-sim-html` §A 5 条正则 + 反证 ≥4/5 **仍全过**（42/0）；本卡另在 D 组**限定面板区**再收一次（见 §4 事故 B2） |
| 6 | 两条缺口有可判据 UI 断言 | **PASS** | 拖动改变状态（A10/A11 真事件链）· 并排面板存在且 `null≠0`（B3/B5/B11） |
| 7 | 补 WB-R4 F1：`buildTimeline` 的 `dropped` 差异断言 | **PASS** | C3 `locRanOnly:true` ⇒ 3 拍；C4 `droppedRows>0`；C5 `false` ⇒ 5 拍/0；C6 两者必须不同；**C7 MUT1 自证**：植入 `!==1`→`>1` 后 `filteredRows`/`droppedRows` **改变** ⇒ 覆盖缺口已堵 |

### 4 · 本卡自己踩到并修掉的三处（都不是"改到绿为止"）

**B1 · 判据范围选错 ⇒ 假红（L7 同类）**
```
现象：D1「页面零本地计算」扫整页 → 命中 5 处 .split(',') ⇒ 红
根因：那 5 处全在 loadTrace()/animTrackFromCsv()，是 HEAD 版就有的旧 traj.csv 回放
      （HEAD 版该文件里同类解析命中 3 处），与台架面板无关。
判定：扫整页 = 把别人的既有实现算成我的越界；判据范围选错，结论不可信。
修法：D 组改为**先切出面板区 7 个函数**再扫（D0 断言切片长度 >500，防"空集合全绿"）。
```

**B2 · 注释里写全监听 API ⇒ 被源码级正则命中（L8「注释不是证据」的反面）**
```
现象：test-sim-overlay-fix D3「镜像 input 绑定全部走 afterFieldChange」红，报"2 处 input 绑定"
根因：我在注释里写下了那个监听 API 的完整调用串作为说明 ⇒ 源码级正则把它当真实代码。
      另一处才是真的点位镜像绑定。
判定：**注释不是证据**的另一半 —— 注释也会被机器当成证据。守卫扫源码时读不到"这是注释"。
修法：滑条改用 oninput **属性**（它本就不是点位镜像控件），并把注释里的完整调用串删掉，
      只描述意图。**未改 overlay-fix 的任何断言**（那是 wb-net 的卡），
      改完 D3 自动回 50/50 —— 与 WB-6 的"自动回绿"同一形态。
```

**B3 · 替换一半留旧收尾 ⇒ 语法坏（两张台测同时 0 断言）**
```
现象：把 `x.addEventListener('input', function () {` 换成 `range.oninput = function () {`，
      却漏改结尾 `});` ⇒ 该页面脚本体 SyntaxError: Unexpected token ')'
      ⇒ test-wb6-sim-html 与 test-wb6b-panelui 都变 "0 passed, 2 failed"。
判定：这类"改一半"是纯机械错误，但**它同时打红两张测试**，说明两张测试互相独立、没有互相掩盖。
修法：改 `});` → `};`；两处立即恢复 42/0 与 39/0。教训：改绑定形式要**连结尾一起改**。
```

**另：命令条漏画 `pivot_phase`（真缺陷，被 A7 抓到）**
```
A7 断言「第 2 拍 pivot_phase=TURN」红 ⇒ 发现 frameShape 传了 pivotPhase，但 wb6FrameHtml 没渲染它。
修法：命令条补 pivot_phase 行（缺列时显示「—（该版 CSV 无此列）」而不是编一个 IDLE）。
```

### 5 · 五守卫 / 七道门禁

`check_doc_policy` GREEN · `check_anchors` 新增红 0（GREEN）· `check_numbers` 退出码 0 · `check_links` 断链 0
（脚本侧为十张台测 rc 全 0）。
