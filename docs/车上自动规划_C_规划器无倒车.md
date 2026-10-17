# 任务卡 C · 规划器无倒车路线 + 折返点 `:PIVOT` 标记 + 合法性自检

> ⚠️ **历史报告（t7 复审标注 2026-10-17）**：本文是 2026-10-11 的验收快照（43 token / 17.15 m / 上洞 K4），
> 正文数字**不代表现役**。现役 = **44 token / 15.05 m / 双停站 `D3*`+`G3*`**（2026-10-17 t13，上洞 K5），真源见 `docs/发车门禁口径.md` §6。

工作区：`D:\User\Desktop\dzrobot_v2`（Windows / PowerShell）
执行者：`planner-pivot-v5`（attempt 15）　日期证据见各命令原始输出
**纪律**：只改工作区文件；**未** `git commit` / `git push` / 部署到车；`config/` 只读。

---

## 0. 一页结论

| 交付 | 结果 | 一句话证据 |
|---|---|---|
| 默认产出 0 段 `R:` | ✅ | `--emit tokens --quiet` → 43 token，`R:` 段 **0**，含 2 处 `:PIVOT`（§3.1） |
| `:PIVOT` 进 token 语法 | ✅ | `parse_token` 新增布尔键 `pivot`；`Waypoint.pivot`；回写序 `R:`→格名→`*`→`:SCAN`→`:PIVOT`→`,x|,y`（§2） |
| 既有形态逐字节不变 | ✅ | 旧/新 parser **3432 个 token 全组合对拍**：既有 18 个形态 **0 差异**（§2.2） |
| 折返点全部带 `:PIVOT` | ✅ | 现场图 2 处（`F8`/`D6`，均 180°），用 `pursuit.path.corner_turn_deg` 复算点集完全一致（§3.2） |
| 合法性自检进 `plan_route` | ✅ | spy 实测被调用；负向控制 3 条全部抛 `PlannerError`（§5） |
| 折返惩罚 5.0/处 | ✅ | 现场图 折返点 **2 → 0**、里程 17.15 → 18.55 m；`_Tour.seq_cost` 差 = 5.0（§4） |
| `--no-reverse`（默认）/ `--allow-reverse` | ✅ | `--allow-reverse` stdout **逐字节 == 卡 A 基线**（sha256 对比，§7） |
| `--json` 折返点数量与清单 | ✅ | `pivot_count: 2` + `pivots[{index,block,deg}]`（§3.1） |
| 3 张随机扰动图 | ✅ | 3/3 规划成功、0 段 `R:`、折返点全带 `:PIVOT`、route_check exit 0（§6） |
| 全套测试 | ✅ | **21/21 套件 exit 0**（含卡 B 新增的 `test_autoplan.py`）；`test_route` 154/154、`test_planner` 83/83（§8） |
| 备份 | ✅ | `backup/cardC-before/`（9 份改前副本 + `SHA256.txt`），与卡 A 备份交叉一致（§1.2） |
| outOfScope 零改动 | ⚠️ 部分可达 | `pursuit/` **零改动**；`config/`、`tasks/mission.py`、`main.py`、`__main__.py` 的改动**全部来自并行卡 B**（内容即卡 B 的 `planner.*` 自动规划接线，§9） |

---

## 1. 改动摘要与备份

### 1.1 逐文件 diff 摘要（`backup/cardC-before/<file>` → 当前文件，含行号）

`git diff --no-index --numstat` + `-U0` 的 hunk 头（**行号为改前文件行号**）：

| 文件 | 行数（现） | +/- | hunks | 关键位置（现行号） |
|---|---|---|---|---|
| [model.py](src/dzbrain/dzbrain/route/model.py) | 205 | +18/-2 | 5 | `pivot` 字段 :36-40；`Route.pivot_points()/pivot_blocks()` :111-117；`RouteBuilder.build` 去重时 pivot 并集 :177-183 |
| [parser.py](src/dzbrain/dzbrain/route/parser.py) | 485 | +61/-42 | 22 | 语法表 :15；`PIVOT_SUFFIX` :70；`_CANON_TAIL_RE` :83-90；`normalize_token` 后缀段重写 :99-156；`parse_token` 剥 `:PIVOT` :220-224 + 键 :192/:211/:239；`Waypoint(pivot=)` :347/:368；去重并集 :384-402；`parse_reset` 拒收 :452-453；`format_route` 回写 :465/:480-481 |
| [planner.py](src/dzbrain/dzbrain/route/planner.py) | 1681 | +228/-23 | 34 | 导入 `_PIVOT_SUFFIX` :88；`__all__` :109-110；`PIVOT_MARK_DEG=135.0` :147、`PIVOT_PENALTY=5.0` :153；`_Stop.pivot` :541；`mark_pivots()` :682-712；`_to_route`/`route_tokens` 回写 :763/:785；`pivot_vertices()` :929-950；`assert_legal_no_reverse()` 第 ④ 条 :952-1000；`PlanResult` 新字段 :1033-1036；`as_dict` 新键 :1050-1065；`summary` :1078-1079；`_Tour(pivot_penalty=)` :1101-1106；`_record_leg_dirs/reversals/penalty_of/seq_cost` :1158-1210；`plan_route(pivot_penalty=)` :1389-1417；`_Tour(...)` :1509；只在无倒车模式打标记 :1595-1597；结果填充 :1634-1635；`_insert_shoot` 惩罚增量 :1661 |
| [tools/plan_route.py](tools/plan_route.py) | 346 | +23/-1 | 5 | 用法/参数说明 :20-42；`plan_once` 传参 :162；`--no-reverse/--allow-reverse` :198-205；`--pivot-penalty` :206-208 |
| [tools/route_check.py](tools/route_check.py) | 954 | +90/-16 | 25 | 六层文档 :32-39；`PIVOT_DECL_DEG=135.0`（并修正 `SHARP_CORNER_DEG` 的错注释）:89-95；`fallback_validate` 接受 `:PIVOT` :180-226；`parse_route` 第 5 个返回 `pivot_f` :229-275；层号 `/6` :449/469/480/540/657/709；解析层读 pivot :470-477；急转弯提示带"已声明 N 处" :529-533；`_pivot_layer()` :715-757；`load_cfg` 可选读 `pursuit.pivot_angle_deg` :806-834 |
| [tests/test_planner.py](src/dzbrain/tests/test_planner.py) | 675 | +154/-0 | 1 | 新增 `section("9 卡 C…")`（35 条断言） |
| [tests/test_route.py](src/dzbrain/tests/test_route.py) | 692 | +86/-0 | 2 | 新增 `test_pivot_suffix()`（29 条断言）+ 注册进 `cases` |

**未改**（在 inScope 里但本轮不需要动）：`route/validate.py`、`tests/test_pivot.py`（sha256 与改前一致，见 §1.2）。

### 1.2 开工第一步的备份（改前冻结）

```
$ Get-ChildItem -Recurse -File backup\cardC-before
backup\cardC-before\SHA256.txt
backup\cardC-before\src\dzbrain\dzbrain\route\{model,parser,planner,validate}.py
backup\cardC-before\src\dzbrain\tests\{test_pivot,test_planner,test_route}.py
backup\cardC-before\tools\{plan_route,route_check}.py
```

sha256 自校验（`_scratch_auto_plan/planner/evidence.py` §6）：

```
改前副本 sha256 自校验失败的条目 = 0 （0）
当前文件 == 改前 sha256 的文件（本卡未改）: validate.py, test_pivot.py
当前文件 != 改前 sha256 的文件（本卡改动的 7 个）: planner.py parser.py model.py plan_route.py route_check.py test_planner.py test_route.py
与任务卡 A 基线交叉校验（同一份文件的两个独立备份）：
  src/dzbrain/dzbrain/route/planner.py   A=1b7d38071fde… C=1b7d38071fde… 一致
  src/dzbrain/dzbrain/route/parser.py    A=ecc95bbc609a… C=ecc95bbc609a… 一致
  tools/plan_route.py                    A=b6c36c6a217e… C=b6c36c6a217e… 一致
```

---

## 2. `:PIVOT` 进 token 语法

### 2.1 语法与回写顺序

```
R: 前缀 → 格名 → *  → :SCAN  → :PIVOT → ,x|,y     ← 后缀唯一合法顺序（严格规范序）
```

* `parser.PIVOT_SUFFIX = ":PIVOT"`；`normalize_token` 用**一次正则** `_CANON_TAIL_RE` 匹配全部后缀：
  只接受上述规范序，**乱序写法仍然报错**（`F2:SCAN*` / `F2:PIVOT:SCAN` → `ValueError`）。
* `parse_token` 返回体新增 `'pivot'` 布尔键（`SHOOT@` 指令返回 `False`）。
* `Waypoint.pivot`（frozen dataclass，默认 `False`）；`Route.pivot_points()` / `pivot_blocks()`。
* `:PIVOT` **不是停站**：`Waypoint.is_stop_point` 仍只看 `shoot/scan`（有断言）。
* `parse_reset` 拒收 `:PIVOT`（复位串不允许带任何后缀）。
* **不改变循迹层**：全仓 `pursuit/**`、`tasks/**`、`main.py`、`core/**` 中
  `\.pivot\b` 与 `:PIVOT` **0 命中**（`pursuit` 只读 `reverse/shoot/scan/yaw`）；
  正因如此 `:PIVOT` 对 `Path`/`corner_turn_deg`/TrackerTask 是**不可见的声明**。

### 2.2 既有形态逐字节回归（原始输出）

`_scratch_auto_plan/planner/probe_grammar.py`：把改前的 `parser.py`（`backup/cardC-before/`）
当独立模块加载，**旧/新对拍**。

```
A. 既有形态逐字节回归（解析 + 回写）
  'F2:SCAN'        norm  old='F2:SCAN'        new='F2:SCAN'         SAME
  'F2:SCAN_STOP'   norm  old='F2:SCAN'        new='F2:SCAN'         SAME
  'F2*'            norm  old='F2*'            new='F2*'             SAME
  'R:F2'           norm  old='R:F2'           new='R:F2'            SAME
  'F2,x'           norm  old='F2,x'           new='F2,x'            SAME
  'F2:SHOOT_STOP'  norm  old='F2*'            new='F2*'             SAME
  'G10*' / 'R:B5*' / 'N5,Y' / 'N5,x' / 'k9 ' / '  k 9  ' / 'r:b5*' / 'G10:SHOOT_STOP' / 'K9' / 'A1' / 'N10,y'
     —— normalize 与 parse_token 全部 SAME
  ⇒ 既有形态 DIFF 数 = 0
B. 全组合对拍（前缀 × 格名 × 后缀序 × 朝向）
  对拍 3432 个 token，normalize_token 差异 1200 个
  --- 差异分类（旧结果类型 → 新结果类型）：
      ValueError   -> ValueError    150 个   例: '*,x' '*,y' '*,X' '*,Y'      （两边都报错，仅文案不同）
      ValueError   -> ok            1029 个  例: 'K9*,x' 'K9*,y' 'K9:PIVOT' …（见下"放宽"）
      ok           -> ValueError    21 个    例: 'K9:SCAN*' 'F2:SCAN*' 'N10:SCAN*' …（见下"收紧"）
  旧 ok 的差异类别 = [('ok', 'ValueError')]
C. 新后缀 :PIVOT 的解析/回写
  'F8:PIVOT'       -> norm='F8:PIVOT'       pivot=True 其余 False
  'f8:pivot'       -> norm='F8:PIVOT'
  'R:B5:PIVOT'     -> reverse=True pivot=True
  'G10*:PIVOT'     -> shoot=True   pivot=True
  'F2:SCAN:PIVOT'  -> scan=True    pivot=True
  'F8:PIVOT,y'     -> pivot=True   heading='y'
  'K9:FOO'         -> ValueError: 无法解析的 token: 'K9:FOO'
  'F2:PIVOT_STOP'  -> ValueError
  'F2:SCAN*'       -> ValueError        （乱序仍报错）
```

**两处刻意的行为差异（如实列出，均有对拍数据）**

1. **放宽 1029 例**：旧实现对"规范序的组合"误报错 —— `K9*,x` / `K9:SCAN,y` / `K9:PIVOT` 这类
   （`*`、`:SCAN` 之后还能接 `,x|,y` 或 `:PIVOT`）。新实现按规范序接受并正确回写。
2. **收紧 21 例**：旧实现把乱序的 `K9:SCAN*` 静默"修正"成 `K9*`（**丢掉 scan 标记**）；
   新实现按规范序**报错**。全仓 `grep 'SCAN\*|\*:SCAN|SCAN_STOP\*'` → **0 命中**（除本注释），
   故无存量数据受影响。

---

## 3. 现场地图结果（`config/map.yaml` + `config/robot.yaml`）

### 3.1 三种模式的完整 token 串与证据

**默认（无倒车，折返惩罚 0）** —— `python tools\plan_route.py --emit tokens --quiet`：

```
N10 N9 M9 L9 K9 I9 I10 H10* F10 F8:PIVOT F9 F10 F11 E11 E10 C10 C9 C8 C7 C6 D6:PIVOT C6 B6 A6 A5 C5 C4 C2 D2 E2 F2 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x
exit = 0  stdout 字节 = 153  sha256 = d9fef04a313e04e26f3582e12e747fead41acd6ba0511c321447eb57218e273f
R: 段数 = 0   :PIVOT 个数 = 2   token 数 = 43
```

**`--json`（折返点数量与清单）**：

```
exit = 0（stdout 前 3 行是既有横幅；机器解析请用 `--emit json --quiet`，其 exit 亦为 0）
  allow_reverse      = false
  pivot_penalty      = 0.0
  tour_cost          = 110.4
  pivot_count        = 2
  pivots             = [{"index": 9, "block": "F8", "deg": 180.0},
                        {"index": 20, "block": "D6", "deg": 180.0}]
  reverse_segments   = 0
  polylength_m       = 17.15
  token_count        = 43
```

**折返惩罚打开** —— `--pivot-penalty 5.0`：

```
N10 N9 M9 L9 K9 J9 I9 H9 H10* I10 I8 H8 G8 F8 F9 F10 F11 E11 E10 C10 C9 C8 C7 C6 C5 C4 A4 A6 B6 C6 D6 D5 D4 D3 D2 E2 F2 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x
token 数 = 49  R: 段 = 0  :PIVOT = 0
```

**允许倒车（旧行为 / 卡 A 基线）** —— `--allow-reverse`：见 §7（逐字节一致）。

### 3.2 折返点统计表（格名 / 转角 / 是否 `:PIVOT`）

| # | 顶点 | 折返角（行进方向夹角） | 相邻段是否倒车 | 车头需转角（`corner_turn_deg`） | `:PIVOT` | 说明 |
|---|---|---|---|---|---|---|
| 9 | `F8` | **180.0°** | 都不是 | 180° | ✅ `F8:PIVOT` | `F10→F8→F9` 死格压雷（F8 是雷），必须原地掉头 |
| 20 | `D6` | **180.0°** | 都不是 | 180° | ✅ `D6:PIVOT` | `C6→D6→C6` 死格压雷（D6 是雷） |
| — | 其余 18 处 >45° 急转弯 | 90.0° | — | ≤90° | ❌（不声明） | 正常直角，`corner_turn_deg` < 135° ⇒ 不触发 PIVOT |

* **判据与循迹层一致**：用 `pursuit.path.Path` 复算
  `corner_turn_deg(i-1) ≥ 135°` 的点集 = `{9, 20}`，与 `:PIVOT` 声明点集**完全相同**（有断言）。
* `--allow-reverse` 模式下 `E11`（#10）与 `D6`（#19）同样是 180°，但**相邻段是倒车段**
  ⇒ 车头一度都不用转 ⇒ **不算折返点、不声明**（这也是"允许倒车模式 0 个 `:PIVOT`"的原因）。

### 3.3 `--check` 六层原始输出（`tools/plan_route.py --check`，ANSI 色码已去）

```
路线[K4] 43 token / 17.15 m | 穿洞 下=K9 上=K4 | 射击停站 H10 到靶 0.350 m（H11）
  A* 代价 114.10 / 展开 7672 次 / 耗时 17.7 ms | 倒车段 0（折返改写 0） | 无倒车模式 是
  折返点 2 处（:PIVOT 声明）：F8 180° D6 180° | 顺序代价 110.40（折返惩罚 0.0/处）
  N10 N9 M9 L9 K9 I9 I10 H10* F10 F8:PIVOT F9 F10 F11 E11 E10 C10 C9 C8 C7 C6 D6:PIVOT C6 B6 A6 A5 C5 C4 C2 D2 E2 F2 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x
  route_check: ✅ 0 error
    ⚠️  无法导入 path_executor（ModuleNotFoundError），改用内置副本校验
    ✅ 全部 token 合法                    普通 43 | 倒车 0 | 射击 1 | 折返 2
    ✅ 解析出 43 个航点
    ✅ 地图边界                           全部 43 点在 [0.17,3.67]x[0.17,4.72] 内
    ✅ 挡板穿越 1 次                       均在 C 列，最小余量 0.350 m
    ✅ 障碍最近距离                         0.350 m（航段#8 距 障G9）> 0.30 m
    ⚠️  急转弯 20 处                       最大 180° @航点#9（其中 2 处已声明 :PIVOT）
    ✅ 桥洞水平直穿 4 次                     K 列  段#3(行9) 段#4(行9) 段#37(行4) 段#38(行4)
    ✅ 雷覆盖 8/8
    ✅ 靶覆盖 2/2（射击点可达）                 G11<-射击点#7(0.49m) H11<-射击点#7(0.35m)
    ✅ 射击点距离                          1 个射击点都能服务于某个靶（0.20~0.45 m 内）
    ✅ 桥洞保护点覆盖                        K4 K9
    ✅ 折返点声明一致 2 处                    #9 180° @F8 #20 180° @D6
    ⚠️  全部通过，但有 1 项提醒

✅ 规划 + 出车前校验全部通过（route_check 0 error）
exit = 0
```

`route_check.py` 的六层标题（原始）：

```
六层标题： ['【1/6 语法层】', '【2/6 解析层】', '【3/6 几何层】', '【4/6 任务层】', '【5/6 安全层】', '【6/6 折返层】']
route_check 直接调用 exit = 0
```

补充：`python tools\plan_route.py --validate` → `validate_route: ✅ 0 error / 0 warn`（R001~R014 对含 `:PIVOT` 的路线全过）。

---

## 4. 折返惩罚（5.0 等效值/处）

**实现**（都在 `planner.py`）：

* `PIVOT_PENALTY = 5.0`；`_Tour(grid, deadline, pivot_penalty=…)`。
* `_Tour.build()` 预建"每对点最短腿的**起步朝向 / 末步朝向**"（`_record_leg_dirs`）。
* `_Tour.reversals(seq)`：`a→b→c` 且"进 b 的末步朝向"与"出 b 的起步朝向"**相反** ⇒ 记 1 次折返。
* `seq_cost()` / `route_cost()` 加 `pivot_penalty × reversals`；`_insert_shoot` 也加"插入引起的折返增量"。
* `plan_route(..., pivot_penalty=None)` ⇒ **0.0（默认不加惩罚）**；显式给 `PIVOT_PENALTY` 即打开。
  默认关闭的理由：卡 A 的 43 token 形状已在真车链路验证过，惩罚版换成一个 49 token 的新形状，
  **需要现场验证后再切换**（改一个参数即可）。`--pivot-penalty` 是一等 CLI 开关。

**加惩罚前 vs 后（现场地图，`allow_reverse=False`）**：

| 配置 | 折返点数 | 总里程(m) | token 数 | 顺序代价 | route_check |
|---|---|---|---|---|---|
| 惩罚 `0.0`（默认） | **2**（`F8` 180°、`D6` 180°） | 17.15 | 43 | 110.40 | exit 0 |
| 惩罚 `5.0` | **0** | 18.55 | 49 | 116.05 | exit 0 |

**单元验证（不依赖规划结果，手工序列）**：

```
C6->D6->B6 的折返次数 = 1（应为 1：进 D6 朝东、出 D6 朝西）
惩罚项 = 5.00（应为 5.0）
penalty=0 时惩罚项 = 0.00（应为 0.0）
同一序列 seq_cost 差 = 12.4500 - 7.4500 = 5.0000
```

⇒ 惩罚真的生效：它把"进死格再原路退出来"从"顺路"改成"贵 5 格"，现场图因此**完全消除**了需要原地
掉头的顶点，代价是多绕 1.40 m。

---

## 5. 合法性自检与负向控制

**自检接线**：`plan_route(..., allow_reverse=False)` 末尾调用 `assert_legal_no_reverse(route)`。
spy 实测（`test_planner.py` §9）：

```
[自检接线] plan_route(allow_reverse=False) 内部调用 assert_legal_no_reverse   [{'no_reverse': 1, 'reverse': 1}]
[自检接线] plan_route(allow_reverse=True) 内部调用 assert_legal                [{'no_reverse': 1, 'reverse': 1}]
```

`assert_legal_no_reverse` 的四条不变量：① token 可解析且 `block/reverse/pivot` 与模型一致；
② 无相邻重复点；③ **0 段 `R:`**；④ **折返点声明完整且不虚报**。

**负向控制原始输出**（`_scratch_auto_plan/planner/probe_pivot.py`）：

```
④ 负向控制 A：拿真实规划结果删掉 :PIVOT -> assert_legal_no_reverse 必须抛错
  被删 :PIVOT 的 token: 43 个；路线折返点=0
  ✅ PlannerError: 折返点缺少 :PIVOT 声明：#9 F8 180°; #20 D6 180°（折返角 ≥ 135° 的顶点必须显式声明
     —— 该处车头要转 180°，无倒车模式下车只能原地掉头）
  对照组：带 :PIVOT 的同一路线应通过
  ✅ 通过（0 error）
⑤ 负向控制 B：含 R: 的路线 -> assert_legal_no_reverse 必须抛错
  ✅ PlannerError: 无倒车模式却出现倒车段：第 1 个航点 'R:N9'（真车底盘不支持倒车，负速度方向位被 STM32 无视）
```

第三条（虚报）在测试里也有（`test_planner.py` §9）：

```
PASS  [负向 C] 虚报 :PIVOT（给 90° 顶点加声明）也必须抛 PlannerError
      [PlannerError: 虚报 :PIVOT：#5 I9（这些顶点的折返角 < 135°，或相邻段是倒车段 —— 车头一度都不用转）]
```

**"声明无害"的实测证据**（同一路线的 token 去掉 `:PIVOT` 后编译成 `pursuit.path.Path`）：

```
PASS  [声明无害] 折线几何逐段一致（去掉 :PIVOT 后 segments 完全相同）   [42 段]
PASS  [声明无害] 每段倒车标记一致   [全一致]
PASS  [声明无害] 每个顶点车头转角一致（corner_turn_deg 逐点相同）   [全一致]
PASS  [声明无害] 停站索引一致（:PIVOT 不产生停站）   [(7, 42)]
PASS  [声明无害] 总长一致   [17.150000 m]
```

---

## 6. 3 张随机扰动地图（`_scratch_auto_plan/planner/maps/`，**未写入 `config/`**）

生成脚本 `_scratch_auto_plan/planner/make_maps.py`（`random.Random(2026100k)` 采样雷/障/桥洞/靶，
逐张验证可解；无解候选被拒后继续采样 —— 本轮的拒绝数见下）。

```
被拒（无解）候选总数 = 2

图     雷/障      桥洞        靶          token  R: 段  折返点  里程(m)  route_check
#1    8/8       K7/K1     K11+L11     43     0      4      15.75   exit=0
#2    8/8       K10/K4    G2+H2       41     0      2      15.75   exit=0
#3    8/7       K8/K5     A9+B9       45     0      3      19.25   exit=0

折返点清单：
  #1 L11 180° J8 180° A5 180° G1 180°
  #2 I7 180° E1 180°
  #3 I7 180° B1 180° I6 180°
声明一致性（token 里 :PIVOT 下标 vs PlanResult 折返点下标）：
  #1 一致  [2, 9, 20, 32] vs [2, 9, 20, 32]  OK
  #2 一致  [7, 25] vs [7, 25]  OK
  #3 一致  [5, 25, 38] vs [5, 25, 38]  OK
```

三张图的完整 token 串：

```
#1  N10 L10* L11:PIVOT L10 L9 L8 L7 K7 J7 J8:PIVOT J7 I7 H7 G7 F7 E7 D7 C7 C6 C5 A5:PIVOT B5 C5 D5 E5 F5 G5 H5 H4 H3 H2 H1 G1:PIVOT H1 I1 J1 K1 M1 M2 M3 M4 M5 N5,y
#2  N10 M10 L10 K10 I10 I9 I8 I7:PIVOT I8 H8 G8 F8 E8 D8 C8 C7 C6 C5 C3 C2 A2 A1 B1 C1 D1 E1:PIVOT D1 D3 E3 F3 G3* G2 H2 I2 J2 J4 K4 L4 M4 N4 N5,x
#3  N10 L10 L8 K8 I8 I7:PIVOT I8 I9 I10 I11 H11 G11 F11 E11 E10 D10 D8 B8* B7 B6 B5 B4 B3 C3 C1 B1:PIVOT D1 D2 E2 F2 G2 H2 H1 I1 I2 I3 I4 I5 I6:PIVOT I5 K5 L5 L3 N3 N5,x
```

> 踩坑记录（已修）：第一版 `check_map()` 没给 `--route`，于是 `route_check.py` 去校验了
> `robot.yaml` 里**另一张图的**手写 `mission_blocks`，三张图全红。加上 `--route` 后三张全 exit 0。

---

## 7. `--allow-reverse` 与任务卡 A 基线逐字节一致

```
$ python tools\plan_route.py --allow-reverse --emit tokens --quiet
exit = 0  stdout 字节 = 152  sha256 = 6789cf082cc4596852f81e7551d77ae031b03a5e05f099a629e310d958407aef
去掉行尾换行后 == 卡 A 基线 : True
（卡 A 基线正文 sha256 = 70cab73b7f4dac24538e65d12714fd10ec765a87333e429a8075196c5ef7c728，151 字节）
```

卡 A 基线原文（`docs/车上自动规划_A_基线验收.md` §1）：

```
N10 N9 M9 L9 K9 I9 I10 H10* R:H11 G11 R:E11 R:F11 F10 F9 F8 E8 D8 C8 C6 R:D6 R:C6 B6 A6 A5 C5 C4 D4 E4 F4 G4 R:G3 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x
```

**结构性保证**：`mark_pivots` 只在 `not allow_reverse` 时调用（`planner.py:1596-1597`），
且 `pivot_penalty` 缺省 0.0 ⇒ 允许倒车模式的搜索代价与输出**一字节都没变**。

---

## 8. 全套测试套件（21/21 exit 0）

命令：`Get-ChildItem src\dzbrain\tests\test_*.py | ForEach-Object { python $_.FullName; if ($LASTEXITCODE -ne 0) { throw "SUITE FAIL" } }`

| 套件 | exit | 结果 | 套件 | exit | 结果 |
|---|---|---|---|---|---|
| test_aim | 0 | 122/122 | test_pivot | 0 | 41/41 |
| **test_autoplan**（卡 B 新增） | 0 | 88/88 | **test_planner** | 0 | **83 项**（基线 53，+30） |
| test_contract | 0 | 56/56 | test_pursuit | 0 | ALL PASS |
| test_core | 0 | 50/50 | test_referee | 0 | 81/81 |
| test_deploy | 0 | 76/76 | **test_route** | 0 | **154/154**（基线 125，+29） |
| test_drone | 0 | 66/66 | test_safety | 0 | 162/162 |
| test_fix | 0 | 74/74 | test_sim | 0 | 82/82 |
| test_geometry | 0 | 38/38 | test_sim_guard | 0 | 104/104 |
| test_localization | 0 | 89/89 | test_telemetry | 0 | 79/79 |
| test_mission | 0 | 97/97 | test_tracker | 0 | 74/74 |
| test_path_overlap | 0 | 32/32 | | | |

`失败套件 = 0`。原始日志逐套件落盘：`_scratch_auto_plan/planner/logs/test_*.txt`。
套件数从 20 变 21 是因为并行卡 B 新增了 `test_autoplan.py`（不是本卡产物）。

确定性：`--emit tokens --quiet` 连跑两次输出完全相同（默认模式与 `--pivot-penalty 5` 模式均如此）。

---

## 9. outOfScope 归属（`git status --porcelain`）

```
 M config/robot.yaml
 M src/dzbrain/dzbrain/main.py
 M src/dzbrain/dzbrain/route/model.py        ← 本卡
 M src/dzbrain/dzbrain/route/parser.py       ← 本卡
 M src/dzbrain/dzbrain/route/planner.py      ← 本卡
 M src/dzbrain/dzbrain/tasks/mission.py
 M src/dzbrain/tests/test_planner.py         ← 本卡
 M src/dzbrain/tests/test_route.py           ← 本卡
 M tools/plan_route.py                       ← 本卡
 M tools/route_check.py                      ← 本卡
?? .agent-teams/
?? docs/车上自动规划_A_基线验收.md            （卡 A）
?? src/dzbrain/dzbrain/__main__.py           （卡 B 新增）
?? src/dzbrain/tests/test_autoplan.py        （卡 B 新增）

$ git diff --stat -- src/dzbrain/dzbrain/pursuit/ src/dzbrain/dzbrain/__main__.py
（无输出 —— pursuit/ 零改动）
```

**如实说明**：outOfScope 里的 `config/`、`tasks/mission.py`、`main.py`、`__main__.py` **确实有改动**，
但**全部来自并行任务卡 B（`mission-autoplan`）**，证据是改动内容本身就是卡 B 的接线：

```
$ git diff -U0 -- config/robot.yaml（新增行）
+    # ---- ★车上自动规划（任务卡 B）：启动时自己读地图算路线 ----
+    planner:
+      auto_plan_enabled: true
+      map_path: "config/map.yaml"
+      fallback_to_tokens: true
+      allow_reverse: false
+      max_seconds: 2.0
$ git diff -U0 -- src/dzbrain/dzbrain/main.py（新增行）
+    plan_on = bool(cfg.get("planner.auto_plan_enabled", False))
+    ... # ★ 自动规划（任务卡 B）…
```

* 本卡对这四条路径**零写入**：本卡的工具只写 `backup/cardC-before/`、`_scratch_auto_plan/planner/`、
  `docs/车上自动规划_C_规划器无倒车.md` 与 §1.1 里列的 7 个 inScope 文件。
* `pursuit/` **零改动**（`git diff` 无输出；`pursuit/path.py` 的 mtime 是 2026-10-09 18:18，两天前），
  符合"不得改变循迹层行为"的硬约束。
* mtime 旁证（采集时刻 2026-10-11 08:03:39）：

  | 路径 | LastWriteTime | 归属 |
  |---|---|---|
  | `config/robot.yaml` | 2026-10-10 19:49:08 | 卡 B（`planner.*` 段） |
  | `src/dzbrain/dzbrain/tasks/mission.py` | 2026-10-10 20:25:23 | 卡 B |
  | `src/dzbrain/dzbrain/__main__.py` | 2026-10-10 19:49:08 | 卡 B（新增） |
  | `src/dzbrain/dzbrain/main.py` | **2026-10-11 07:54:51** | 卡 B（同一时段并行编辑中） |
  | `src/dzbrain/dzbrain/pursuit/path.py` | 2026-10-09 18:18:42 | 无人改动 |
  | `src/dzbrain/dzbrain/route/planner.py` | 2026-10-11 07:56:32 | **本卡** |

  ⚠️ `main.py` 的时间戳落在本卡工作时段内 —— 但它是卡 B 的产物（diff 内容即其
  `planner.auto_plan_enabled` 接线），**归属判据以内容为准**，mtime 仅作旁证。

---

## 10. verify 命令原始输出汇总

| 契约命令 | exit | 关键原始输出 |
|---|---|---|
| `python tools\plan_route.py --emit tokens --quiet` | 0 | 43 token，`R:` 段 **0**，`:PIVOT` **2**（§3.1） |
| `python tools\plan_route.py --check` | 0 | `route_check: ✅ 0 error`；六层 `【1/6…6/6 折返层】` 全过（§3.3） |
| `python tools\plan_route.py --json` | 0 | `pivot_count: 2` + `pivots[]`（§3.1） |
| `Get-ChildItem src\dzbrain\tests\test_*.py \| ForEach-Object { python $_.FullName; … }` | 0 | 21/21 套件全绿（§8） |

全部原始文本（含本报告引用的每一段）落盘在：
`_scratch_auto_plan/planner/logs/cardC_evidence.txt`、
`_scratch_auto_plan/planner/logs/check_default_clean.txt`、
`_scratch_auto_plan/planner/logs/test_*.txt`。

---

## 11. 未确认 / 遗留 / 给后续卡的发现

1. **`plan_route` 的库默认仍是 `allow_reverse=True`**（签名向后兼容），
   无倒车默认只在 **CLI**（`--no-reverse`）。契约的验收判据就是 CLI，故满足；
   卡 B 已在 `config/robot.yaml` 写 `planner.allow_reverse: false`，两者语义一致。
2. **折返惩罚默认 0.0（关闭）**：惩罚版路线（49 token / 0 折返 / 18.55 m）**未上真车验证**，
   建议现场验证后再把 `--pivot-penalty 5` 设为默认（一行开关）。收益估算：
   省掉 2 次原地掉头（真车实测每次 ~10.8 s、横向扫出 ~0.54 m），代价是多跑 1.40 m
   —— 未实测，**未确认**。
3. **手写回退路线缺 `:PIVOT` 声明（给卡 D / 配置所有者）**：`robot.yaml` 的
   `path_executor.mission_blocks`（34 点，也是 `route.tokens`）里有 3 处**前进-前进 180° 折返**
   且**没有** `R:` 段，route_check 第 ⑥ 层如实报（warn，exit 仍 0）：

   ```
   python tools\route_check.py   →   ⚠️ 折返点缺 :PIVOT 3 处   #12 180° @E11 #18 180° @A6 #20 180° @D6
   ```

   与真车结论一致（`docs/上下文快照.md:22`：PIVOT 真车实测 2 次完成 —— E11 +14.6°、A6 −14.9°）：
   **这三处真车一定会自动 PIVOT**，但 token 里没写明。建议在回退路线 token 上补 `:PIVOT`
   （`config/` 不在本卡 inScope，未动）。
4. **`route_check.py` 层号从 `1/5…5/5` 变成 `1/6…6/6`**（新增第 ⑥ 层）。
   全仓 grep `1/5|5/5 安全层` 已确认无外部依赖（唯一命中就是该文件自己）。
5. **`--json` 会先打印人类横幅再输出 JSON**（既有行为，未改）；机器解析请用 `--emit json --quiet`。
6. **未执行**：真车 / `install/` 空间运行、`python -m dzbrain`、MissionTask 对 `:PIVOT` 的消费链路
   （卡 B 的 `_load_startup_route`）、跨机对账（车 `10.226.211.188` 与无人机网段本轮全部不可达，
   **BLOCKED**，未伪造任何数据）。
7. `:PIVOT` 目前**没有任何模块读取**（`pursuit/**`、`tasks/**`、`core/**` 中 `\.pivot\b` 0 命中）：
   它是纯声明 + 自检。若将来要"按声明强制/抑制 PIVOT"，那属于循迹层改动，不在本卡范围。
