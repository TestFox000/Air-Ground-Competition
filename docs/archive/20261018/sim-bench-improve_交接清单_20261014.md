# 交接清单 — sim-bench-improve 队（2026-10-15 修订版）

> 本文是 `sim-bench-improve` 团队的状态快照，供接手者直接续跑。
> **本版取代 2026-10-14 版**：旧版 §0/§2/§7 关于「SyntaxError 导致测试跑不起来」的记载**已被实测证伪**，
> 证伪过程与教训见 §2。原则不变：只记实测过的事实，疑点单独列节，不冒充结论。

---

## 0. 一句话现状（2026-10-15 17:00 实测）

三项改进（路径折线 / 位姿标注 / 靶朝向）**代码已落地**；
**官方测试 36/36 真绿**（`NODE_EXIT=0`，PowerShell `$LASTEXITCODE` 直取）；
**只剩 F3 一条**（`select#targetYaw` 的 change 监听 + reset 分流）未做。
阻塞原因是**模型配额限流**（`buddy/step-5-preview` 全账号受限，10-15 19:11 解禁），不是技术问题。

---

## 1. 版本与改动基线（实测）

| 项 | 值 |
|---|---|
| HEAD | `af0fb4e`（10-14 21:24）「收尾快照_官方图基线重钉ABC组与云台开火键名落地(**无版本tag**)」 |
| 本队 diff | `sim.html` **净 −4 行**（删靶字循环）+ `scripts/test-sim-map-render.mjs` **+159 行**（F4 断言改写 + F2 三组新断言 + 变异段记档） |
| `simbench.js` | **零改动**（targetYaw 透传链在 HEAD 里就有，本队只核验未改） |
| 并发写入者 | `src/dzbrain/tests/test_planner.py`（1 个 M，**本队不碰**） |

★ 注意 `af0fb4e` **没有 tag**。本项目的真源是 `D:\User\Desktop\比赛存档\v1\dzrobot_v2`；
`D:\User\Desktop\dzrobot_v2` 是**陈旧副本**（止于 10-11 的 `d2e4da1`），别在上面干活。

---

## 2. ★★ 旧版记载的证伪：SyntaxError 是「我读早了」+「cmd 重定向骗了我」两件事叠加

这是本清单最重要的一节，两个独立错误各自都足以误导接手者。

### 错误 A：旧版 §2 说「测试有 SyntaxError、跑不出分数」——**当时真存在，但已被修**

旧版是我在成员改动**落地之前**读文件写的，之后 sim-bench 把 `:145-156` 三处 `check()`
的括号补齐了。verifier 复核时报「三处 paren 配平、SyntaxError 不存在」——**它是对的，我那份记载过期了**。

教训：**写快照前必须重读当前文件**。成员在并发干活，我引用的是几分钟前的观察。

### 错误 B：我用 `cmd /c "... > file & echo EXIT=%errorlevel%"` 拿到 EXIT=0，而实际是 SyntaxError

同一条命令，`%errorlevel%` 取到的是**重定向链的码，不是 node 的码**。那次 SyntaxError 真实存在，
我的"实测"却报 0 —— **我写的坑（§6-3）我自己当场违反了**。

**正确姿势（本项目现在统一用这个）**：
```powershell
Set-Location '<repo>\dsh-plugin-dzrobot-ssh'
$o = node scripts\test-sim-map-render.mjs 2>&1
"NODE_EXIT=$LASTEXITCODE"      # ← PowerShell 原生变量，取的是 node 的真实退出码
$o | ForEach-Object { "$_" }   # ← 同时读输出内容，不能只看码
```

---

## 3. 四条 findings 的最终状态

| ID | 级别 | 内容 | 状态 |
|---|---|---|---|
| **F1** | blocker | `sim.html` per-cell「靶」字循环与 t28 并板块叠加 | ✅ **已修**（净 −4 行，整段删除） |
| **F4** | blocker | F1 必须与测试断言同步改（`>靶<` 期望 1→0） | ✅ **已修**（`:89-91`，断言名点明「t28 起不再每格出靶字」） |
| **F2** | medium | 断言未净增、A/B/C 三组不存在 | ✅ **已修**：21 → **36 条**（净增 15）。A 折线 4 / B 位姿 4 / C 靶朝向 7 |
| **F3** | low | `select#targetYaw` 无 change 监听 + reset 不参与 | ❌ **未做**（唯一残项，见下） |

### F1+F4 为什么必须两文件同步（机理，留证）

`sim.html` 的 `text()` 构造器产出 `'...>' + esc(body) + '</text>'`：
- `'靶'` → `>靶</text>` → **匹配** `/>靶</`
- `'靶板'` → `>靶板</text>` → **不匹配**（「靶」后紧跟「板」，不是「<」）

所以只删循环会让 `>靶<` 从 2 变 0，而旧断言期望 1 → 仍 FAIL。
verifier 用「内存替换 body 跑真 renderMap()」实测两种状态都 FAIL，是这条的铁证
（脚本 `dzrobot_v2/_tmp_verify_t3_fixsim.mjs`，未跟踪，可复用）。

### F3 的准确现状（我实测，与旧版描述不同）

```js
// sim.html:1145-1147  ← targetYaw 不在这个数组里
['mines', 'obstacles', 'bridges'].forEach(function (key) {
  $(key).addEventListener('input', function () { refreshFieldHints(); renderMap(); });
});
// sim.html:1148-1157  ← 只遍历 button[data-reset]
```

**准确表述**：`#targetYaw` 既无 change 监听（改完地图不即时重绘），
且 reset 按钮**根本不参与**（不是旧版说的"写空串"——`select` 没有对应 `data-reset` 按钮，
`$(key).value = (FIELD[key] || []).join(' ')` 这条路径压根不会为它执行）。

**修法**（两步，都在 sim.html）：
1. `$('targetYaw').addEventListener('change', function () { renderMap(); })`
   —— **select 用 `change` 不用 `input`**，不要塞进上面那个 input 数组。
2. 若将来给靶朝向加 reset：按 key 分流，select 恢复默认选中项（`''` = 跟随地图），
   textarea 才走 `join(' ')`。

★ **不要动** `targetBoardRect` 的尺寸逻辑：h/v 只改标注不改尺寸是 t28 的正确语义，
verifier 与 sim-ui 已各自独立复算确认（`E5+F5` auto=68×34、`E5+E6` auto=34×68）。

---

## 4. ★ 变异测试（破坏实验）：为什么测试文件里**没有**，以及该由谁做

sim-bench 三次尝试在测试文件内内置变异段，**全部失败**，已在 `:226-242` 用 14 行记档。
三种死法都有价值，别再写第四遍：

| 尝试 | 写法 | 为什么假绿 |
|---|---|---|
| ① | `mustFail` 判据 `threw !== null \|\| fn.length === 0` | 箭头函数无参时 `length` 恒 0 ⇒ 没抛错也判 OK ⇒ 六条全假绿 |
| ② | 变异输入与断言输入不一致（断言测 A1→D1，变异喂 A1→B1） | 相邻格正确输出本就是 2 点，`if (length===4) throw` 永不成立 |
| ③ | `expectFail` 把谓词写成"正确行为应满足的条件" | 实现正确时谓词恒 false ⇒ 永远"未捕获" ⇒ 九条全 WARN |

**根因（我复核后认同）**：变异测试要证的是"断言能抓住退化"，但**断言本身不在这个闭包里**，
无法自证。有效的退化验证必须由**能替换被测实现的一方**做 —— 即 verifier 的
「只读读 sim.html → 内存替换 body → 跑真 renderMap()」手法。

⇒ **t5/t6 的合同我已按此修订**（`agent_teams_amend_task`，各 1 条 revisions 记录）：
不再要求测试内置变异，改要求 verifier/reviewer 自己注入真实退化验证断言能否变红，
**三组全部有牙才判 pass**。防假绿强度不降反升。

---

## 5. 回归套件真实结果（读输出内容，不看退出码）

| 套件 | 结果 | 证据 |
|---|---|---|
| `test-host.mjs` | ✅ 真绿 | 输出 `HOST TEST PASSED` |
| `test-package-files.mjs` | ✅ 真绿 | 输出 `PACKAGE FILES TEST PASSED` |
| `test-client.mjs` | ⚠️ **1 条 FAIL，但已证为 HEAD 既存** | `tool list should have 4 rows, has 5` |

### ★ 既存性已闭环（旧版列为疑点 1，现已证明）

我用 `git archive HEAD` 出**零工作区改动**的干净副本，跑同一脚本，**仍 FAIL 同一条**：
```powershell
$dst = Join-Path $env:TEMP 'clean_head_sim2'
git archive HEAD -o "$dst\head.tar"     # ★ 必须落盘：`git archive | tar -x` 在 PowerShell 里
tar -xf "$dst\head.tar" -C $dst          #   会把二进制流打坏（"Damaged tar archive"）
Set-Location "$dst\dsh-plugin-dzrobot-ssh"; node scripts\test-client.mjs
# → CLEAN_TEST_CLIENT_EXIT=1，FAILURES: tool list should have 4 rows, has 5
```
⇒ 与本队改动**无关**，t5 **不得**据此判 FAIL。
（★ 绝不用 `git stash` —— 会扰动并发写入者的工作区。）

---

## 6. 本项目踩过的坑（下次必读）

1. **★ 退出码会单独撒谎**（两种独立形态，都被我踩过）：
   - `test-client.mjs` 有 FAILURES 却可能 exit 0（本次实测是 exit 1，但**不可依赖**）；
   - `cmd /c "... > file & echo EXIT=%errorlevel%"` 取到的是重定向链的码，不是目标程序的码。
   ⇒ **一律用 PowerShell `$LASTEXITCODE`，且必须同时读输出内容。**
2. **`node --check xxx.mjs` 会给假 EXIT=0**（ESM 顶层检查 ≠ 运行时模块加载）。验语法必须真跑。
3. **PowerShell 里 `git archive | tar -x` 会损坏二进制流**，报大量 `Damaged tar archive`。
   必须先 `-o file.tar` 落盘再 `-xf`。
4. **`head` 在 cmd 里不存在**，`git show ... | head -20` 会 exit 1，易误判成 git 失败。
5. **成员报的行号/状态可能过期**：并发写入者一直在动 `config/`、`src/`。
   任何"现状"结论下笔前重读一次文件。

---

## 7. 下次续跑的执行顺序

```
①  确认配额已恢复（10-15 19:11 之后），成员路由可用
②  sim-ui 做 F3（唯一残项）：targetYaw 绑 change 监听 + reset 分流；只写 sim.html
      ↓ 自验：node scripts/test-sim-map-render.mjs 必须仍 36/36（F3 不影响现有断言）
③  t5 verifier 复验（合同已修订）：
      - F1/F4：>靶<=0、>靶板<=1、stroke 在位
      - F2：36/36 且净增 15 条
      - ★ 自己注入退化验 A/B/C 三组断言能否变红（外部手法，内存替换被测实现）
      - F3：change 监听 + reset 分流到位
      - 回归：读输出内容；test-client 那条既存 FAIL 不判红
④  t6 reviewer 复轮审查（合同已修订）：
      - 删得干不干净（净 −4 行、IIFE 逐字未变、雷障洞起终未误删）
      - ★ 独立判断「变异无法自证」记档理由是否成立（不许无条件接受）
      - api return 扩展未破坏原 4 个函数
⑤  收口：决定 _tmp_* 去留、是否提交
      （★ 提交时注意：并发写入者还在动 config/ 与 src/，只 add 本队两个文件）
```

**当前 DAG**：`t1 failed(transport)` / `t2 failed(transport)` / `t3 failed(needs_revision)` /
`t4 failed(reject)` / `t5 pending(←t1,t2)` / `t6 pending(←t1,t2)`。
t1/t2 的**工作是完成的**（F1/F4/F2 已落盘），failed 只是成员掉线的系统标记；
续跑时按 `reassign_task` 让 sim-ui 补 F3 后交卡，再解锁 t5/t6。

---

## 8. 疑点清单（未验证，不冒充结论）

| # | 疑点 | 状态 |
|---|---|---|
| 1 | ~~test-client 那条 FAIL 是否既存~~ | ✅ **已证**：HEAD 既存（§5，git archive 干净副本） |
| 2 | `targetYaw` 落账是否真进 `run_meta.jsonl` | ❌ **未实测**。只读到代码（`simbench.js:338/351/470`，verifier 静态核验 8/8）。续跑需真跑一次 sim 看落账文件 |
| 3 | 删靶字循环后 legend「靶（跨格板）」与实际渲染的一致性 | ❌ **无断言保护**（reviewer 留的覆盖缺口） |
| 4 | `sim.html:689` 靶粉底填充被删能否被抓到 | ❌ **不能**，无断言覆盖（reviewer 实证） |
| 5 | F5：对角分支补的是直角折点，注释却写「切角过渡点」 | ⏸ **未决**。真实路线 54 token 全逐格正交，该分支 0 次触发 ⇒ 属文档/实现不一致，不影响当前行为 |
| 6 | F7：`:72-73` 用 `font-size="8.5"` 计数代理（CELLS+1）属间接耦合 | ⏸ **未处理**。将来再加 8.5 号字（如「起点位姿」正是 8.5）会误红 |
| 7 | 现有 36 条断言是否真能抓住退化 | ⏳ **待 t5 用外部注入法验证**（合同已修订，这是 t5 的核心判据） |

---

## 9. 工作区未跟踪文件（实测，全部保留）

| 文件 | 属主 | 说明 |
|---|---|---|
| `_tmp_verify_t3_fixsim.mjs` | verifier | ★ **F1+F4 铁证来源**，内存替换 body 跑真 renderMap()。t5 直接复用 |
| `_tmp_verify_t3_geom.mjs` / `_passthrough` / `_targetbug` / `_events` | verifier | 几何复算 / 透传链 / 靶字残留 / 事件绑定 取证 |
| `_tmp_verify_t5_baseline.mjs` | verifier | ★ 已用真 sim.html 独立复算全部基准值，与队长数字逐项一致，可直接当 t5 期望 |
| `_tmp_verify_t5_mutants.mjs` / `_m5diag.mjs` | verifier | 变异尝试与诊断（对应 §4 的三种死法） |
| `_tmp_t1_selfcheck.mjs` | sim-ui | 自验脚本；末尾汇总行 `countChecks()` 恒返 0（只影响文案，不影响逐条 PASS/FAIL） |
| `_t3b.txt` | verifier | `git status` 快照（PowerShell 重定向产物，非仓库误改） |
| `_t3c/_t3d/_t3e/_t3f_score.txt`、`_t1_score_now.txt`、`_reg_*.txt` | captain/成员 | 各次取证输出；`_t3f_score.txt` 是 UTF-16（read 工具会报 binary，用 `Get-Content -Encoding Unicode`） |

**处置建议**：全部保留（t5/t6 要复用）；收口时统一清理，**不要**在复验前删。

---

## 10. 团队与任务状态（2026-10-15 17:15）

### 花名册已换人（原成员限流掉线，旧名字不可复用）

| 新成员 | 接替 | 路由 | 负责卡 |
|---|---|---|---|
| **sim-ui2** | sim-ui | `deepseek-official/deepseek-flash` · high | t1（只剩 F3） |
| **sim-bench2** | sim-bench | 同上 | t2（核验 + 正式交卡） |
| **verifier2** | verifier | 同上 | t3（当前状态门禁）→ t5（复验） |
| **reviewer2** | reviewer | 同上 | t4（返工后审查）→ t6（复轮） |

★ **AgentTeams 机制限制（实测）**：
1. `removed` 的成员名**永久占用、不可复用**（报 `has already been used`）。
2. 运行中的团队**不能原地改成员模型**：`edit_plan` 对 running 团队只允许改
   未开工的 pending 任务，roster 改动要求 staged 计划。
   ⇒ 换路由必须 `remove_member → add_member`，成员会丢自己的会话上下文。
   ⇒ 所以关键事实必须写进 `executionPrompt` + 任务合同 + 本清单，不能指望成员记得。
3. ⚠ **副作用**：`remove_member` 会把该成员持有的卡重开成 `pending` 且**无依赖**，
   导致早已结束的 t3/t4 回到共享池、被新人抢走。
   已给两人下范围指令：**t3/t4 按当前代码判、F3 归 t6、退化验证归 t5**，
   避免重复劳动与再次触发 repair 循环（本队 loop 已 `escalated`，
   再来一轮 needs_revision 会直接卡死交付）。

### 六张卡

| 任务 | 状态 | 说明 |
|---|---|---|
| t1 sim-ui2 | claimed（attempt 6） | F1 已落盘；**只剩 F3**（targetYaw 绑 change 监听） |
| t2 sim-bench2 | in_progress（attempt 5） | F4 + F2 已落盘（+159 行、36/36 真绿），核验后正式交卡 |
| t3 verifier2 | in_progress（attempt 2） | 判据已按当前状态更新；**F3 不计入此卡** |
| t4 reviewer2 | claimed（attempt 2） | 审返工后代码；须**独立判断**「变异无法自证」记档是否成立 |
| t5 verifier2 | pending（←t1,t2） | **合同已修订**：外部注入退化，三组全有牙才 pass |
| t6 reviewer2 | pending（←t1,t2） | **合同已修订**：F3 归此卡 + 独立判断变异记档 |

### 路由为什么换 & Qwen 的核实结果

`buddy/step-5-preview`（CodeBuddy/腾讯）全账号限流至 **10-15 19:11**，
四人先后被打下来两次，且这是**单一故障点**——继续等就是两小时空转，
t5/t6 接力时还会撞同一堵墙。用户要求换 Qwen3.8-flash，实测**不可用**：

```
our-free-model/qwen3.8-flash  → not allowed for this Session
codearts/qwen3.8-flash        → not allowed for this Session
codearts/deepseek-flash       → not allowed for this Session
codearts/deepseek-v4-flash    → not allowed for this Session
```

本 session 只注册 4 个子智能体路由（`deepseek-official` / `codearts` /
`modlens-codearts` / `our-free-model`），`codearts` 不 advertise 模型列表、
不能作为子智能体路由。实际可选只有：
`deepseek-official/deepseek-flash`（V41-Flash，off/low/high/max）·
`our-free-model/mimo-v2.6-flash-free` · `our-free-model/space-bunny-free`。
⇒ 已按用户「华为/DeepSeek flash 档」意图取 **deepseek-official/deepseek-flash**。
