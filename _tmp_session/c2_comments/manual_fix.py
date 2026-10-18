"""c2 插件侧注释精简 —— 第 2 步：人工复核后的精修（只改注释行）。

每条 (file, old, new) 都按整行精确匹配，并断言在文件中唯一；多行 docstring 用整块替换。
"""
import pathlib, sys

PLUGIN = pathlib.Path(r'D:\User\Desktop\比赛存档\v1\dzrobot_v2\dsh-plugin-dzrobot-ssh')

FIXES = [
    # ── sim.html ─────────────────────────────────────────────────────────────
    ('sim.html',
     '      <!-- （规格 §D.1）：无人机协同开关 —— 默认不勾；勾上后车路线硬避让无人机起降格',
     '      <!-- 无人机协同开关（规格 §D.1）—— 默认不勾；勾上后车路线硬避让无人机起降格'),
    ('sim.html',
     '           收口：起降格**不写死在文案里** —— 由 loadField()/refreshFieldHints() 从',
     '           起降格**不写死在文案里** —— 由 loadField()/refreshFieldHints() 从'),
    ('sim.html',
     '  /* 命中层（t6 落格）：装饰层不吃指针事件，elementFromPoint 才能落到带 data-cell 的基格 rect */',
     '  /* 命中层（落格）：装饰层不吃指针事件，elementFromPoint 才能落到带 data-cell 的基格 rect */'),
    ('sim.html',
     '        pointer-events:none ⇒ 不吃指针事件：t6 落格命中（elementFromPoint）与地图点击不受影响。 */',
     '        pointer-events:none ⇒ 不吃指针事件：落格命中（elementFromPoint）与地图点击不受影响。 */'),
    ('sim.html',
     '    <!-- ── 64 图压测面板 —— 只走 t7 的 3 个 /sim/stress 端点，后端零改动 ── -->',
     '    <!-- ── 64 图压测面板 —— 只走 3 个 /sim/stress 端点，后端零改动 ── -->'),
    ('sim.html',
     '    <!-- ── 改点位（t25 → t6）：棋盘拖拽为主编辑通道；只落运行目录，config/** 零写入 ── -->',
     '    <!-- ── 改点位：棋盘拖拽为主编辑通道；只落运行目录，config/** 零写入 ── -->'),
    ('sim.html',
     '      <!-- 卡3+：空地协同拟真地图 —— 底图 + 无人机航线（橙）/扫描大虚线圆/3×3 覆盖 + 小车路线（绿）/起终点位姿 -->',
     '      <!-- 空地协同拟真地图 —— 底图 + 无人机航线（橙）/扫描大虚线圆/3×3 覆盖 + 小车路线（绿）/起终点位姿 -->'),
    ('sim.html',
     '  // （规格 §D.1）：无人机协同 —— 仅勾选时带 drone:true（带键才挂：不勾时请求体无该键，',
     '  // 无人机协同（规格 §D.1）—— 仅勾选时带 drone:true（带键才挂：不勾时请求体无该键，'),
    ('sim.html',
     '  //   （规格增补「靶板蓝红两色**沿长边对半**、色值固定不随机」）：',
     '  //   规格增补「靶板蓝红两色**沿长边对半**、色值固定不随机」：'),
    ('sim.html',
     '  //   **废除 t14 的「点击旋转即翻面（蓝↔红）」交互** —— 旋转只改朝向 h↔v，不改分色；',
     '  //   **废除「点击旋转即翻面（蓝↔红）」交互** —— 旋转只改朝向 h↔v，不改分色；'),
    ('sim.html',
     '  //   装饰层（line + pointer-events:none）不吃命中 ⇒ t6 落格/棋子点击不受影响。',
     '  //   装饰层（line + pointer-events:none）不吃命中 ⇒ 落格/棋子点击不受影响。'),
    ('sim.html',
     '  // 收口：无人机协同开关文案里的起降格取 /sim/map 真源（FIELD.drone），不写死 N8/M2 ——',
     '  // 无人机协同开关文案里的起降格取 /sim/map 真源（FIELD.drone），不写死 N8/M2 ——'),
    ('sim.html',
     '   真源，只是到顶后该类拖放硬拒（落格不生效 + 「已满 N 枚」提示），取代 t6 时代的',
     '   真源，只是到顶后该类拖放硬拒（落格不生效 + 「已满 N 枚」提示），取代此前的'),
    ('sim.html',
     ' *  废除 t14 的「旋转即翻面」：这里**没有** face 状态 —— 分色只由几何决定，',
     ' *  「旋转即翻面」已废除：这里**没有** face 状态 —— 分色只由几何决定，'),
    ('sim.html',
     ' *  renderMap 只重写 #map.innerHTML（t10 有意不碰 #anim，以免打断回放），所以旧层不会',
     ' *  renderMap 只重写 #map.innerHTML（有意不碰 #anim，以免打断回放），所以旧层不会'),
    ('sim.html',
     '  //   取代 t6 的数量软警告（「能点出 10 个」的根因就是软警告不拦）。',
     '  //   取代此前的数量软警告（「能点出 10 个」的根因就是软警告不拦）。'),
    ('sim.html',
     '  //   旋转只改朝向（h↔v），不改哪半是蓝（旧 t14 在此处 TARGET_FACE 蓝↔红取反，已废除）。',
     '  //   旋转只改朝向（h↔v），不改哪半是蓝（此前在此处 TARGET_FACE 蓝↔红取反，已废除）。'),
    ('sim.html',
     '/* ── （卡4）：运行仿真动画 —— 车比例方块起点循迹 + 渐进路径线（trail）─────────',
     '/* ── 运行仿真动画 —— 车比例方块起点循迹 + 渐进路径线（trail）─────────'),
    ('sim.html',
     '     sim.html t28 注释「尺寸 = 现场真值车长 0.28 m × 车宽 0.135 m」）按 GRID.blockM=0.35 m/格',
     '     sim.html 车身真值注释「尺寸 = 现场真值车长 0.28 m × 车宽 0.135 m」）按 GRID.blockM=0.35 m/格'),
    ('sim.html',
     '     不打断回放；#anim pointer-events:none ⇒ t6 落格命中与 t8 时间线面板互不干扰；',
     '     不打断回放；#anim pointer-events:none ⇒ 落格命中与空地协同时间线面板互不干扰；'),
    ('sim.html',
     '   · 404（例如 t25 的 `plan-failed` 跑没有产物）→ modal 里给具体中文文案 + HTTP 状态，不留白屏；',
     '   · 404（例如 `plan-failed` 的跑没有产物）→ modal 里给具体中文文案 + HTTP 状态，不留白屏；'),
    ('sim.html',
     '   · 横置渲染**复用 t25 的同一套**：CSS `.rot90` + `rotate(-90deg)` + `fitRot()`（本文件 t25 段），',
     '   · 横置渲染**复用同一套**：CSS `.rot90` + `rotate(-90deg)` + `fitRot()`（本文件横置段），'),
    ('sim.html',
     ' *   ④ startReplay() 拉本局 traj.csv 做轨迹回放（t10；每次「运行仿真」成功后一次，',
     ' *   ④ startReplay() 拉本局 traj.csv 做轨迹回放（每次「运行仿真」成功后一次，'),
    ('sim.html',
     '      // T3：具体中文文案 + HTTP 状态；不白屏、不静默吞',
     '      // 具体中文文案 + HTTP 状态；不白屏、不静默吞'),
    ('sim.html',
     '/** 画出图：横置（默认，复用 t25 的 .rot90 + fitRot）或竖版（.rotbox）。 */',
     '/** 画出图：横置（默认，复用 .rot90 + fitRot）或竖版（.rotbox）。 */'),
    ('sim.html',
     '/* ── （规格增补 §D.1/§D.3 收口）：无人机协同「本局复核」──────────────────────',
     '/* ── 无人机协同「本局复核」（规格增补 §D.1/§D.3 收口）──────────────────────'),
    ('sim.html',
     '   旧实现（t3）只有一条 token 扫描，且三种必须区分的情况被压成二分：场地没加载时',
     '   旧实现只有一条 token 扫描，且三种必须区分的情况被压成二分：场地没加载时'),
    ('sim.html',
     '  // （t27 修）：**非法格名不得静默丢弃**。任一类"非空输入但规范化后 0 格"时**不发请求**，',
     '  // **非法格名不得静默丢弃**。任一类"非空输入但规范化后 0 格"时**不发请求**，'),
    ('sim.html',
     '      // （§D.2）页面校验层复核（双层避让的第二层）+ 收口：三态见 droneRunAudit ——',
     '      // 页面校验层复核（双层避让的第二层，§D.2）+ 三态见 droneRunAudit ——'),
    ('sim.html',
     '  // 靶板分色已无 state（蓝/红沿长边固定）⇒ 整图恢复不再需要"还原面色"（旧 t14 在此复位 TARGET_FACE）',
     '  // 靶板分色已无 state（蓝/红沿长边固定）⇒ 整图恢复不再需要"还原面色"（此前在此复位 TARGET_FACE）'),
    ('sim.html',
     '// （卡4）：回放按钮（重播当前回放数据）—— 只写 #anim 覆盖层，不碰 #map/时间线面板',
     '// 回放按钮（重播当前回放数据）—— 只写 #anim 覆盖层，不碰 #map/时间线面板'),
    ('sim.html',
     '// （卡3）：空地协同时间线 —— 双轨并行推进（无人机逐点扫描 vs 小车先验行驶）。',
     '// 空地协同时间线 —— 双轨并行推进（无人机逐点扫描 vs 小车先验行驶）。'),
    ('sim.html',
     '/* ── 卡3+：空地协同拟真地图 —— 与主地图同一套底图（棋盘/隔板/起终点/雷障洞靶），',
     '/* ── 空地协同拟真地图 —— 与主地图同一套底图（棋盘/隔板/起终点/雷障洞靶），'),
    ('sim.html',
     '   纯 SVG、只写 #droneMap 覆盖层，不碰 #map（t6 落格命中层不受影响）。 */',
     '   纯 SVG、只写 #droneMap 覆盖层，不碰 #map（落格命中层不受影响）。 */'),
    ('sim.html',
     '/* ── （卡5）：50 图压测面板 —— POST /sim/stress（background 202）→ status 轮询 →',
     '/* ── 50 图压测面板 —— POST /sim/stress（background 202）→ status 轮询 →'),
    ('sim.html',
     '     result 取回渲染。后端零改动，只消费 t7 已交付的 3 个端点。',
     '     result 取回渲染。后端零改动，只消费 3 个 /sim/stress 端点。'),
    ('sim.html',
     '   · 统计口径 = t7 stressStats 的**显式分母**字段：counts.<项> 即各均值分母原样、',
     '   · 统计口径 = stressStats 的**显式分母**字段：counts.<项> 即各均值分母原样、'),
    ('sim.html',
     '  // 把两个均值的**分母**也写出来（t7 显式分母口径；缺统计字段时退回 records 自算）',
     '  // 把两个均值的**分母**也写出来（显式分母口径；缺统计字段时退回 records 自算）'),

    # ── index.js ─────────────────────────────────────────────────────────────
    ('index.js',
     '          // 拟真（卡3+）：透传无人机航线 + 情报命中，前端据此在场地地图上画飞行/扫描/采纳。',
     '          // 拟真：透传无人机航线 + 情报命中，前端据此在场地地图上画飞行/扫描/采纳。'),

    # ── src/simbench.js ──────────────────────────────────────────────────────
    ('src/simbench.js',
     '/** 无人机起/降点：map.yaml 的 t28 识图注释里写着；读不到时用这两个（蓝 + H 图标）。',
     '/** 无人机起/降点：map.yaml 的识图注释里写着；读不到时用这两个（蓝 + H 图标）。'),
    ('src/simbench.js',
     '/** 现场 YAML 是 CRLF：解析前先把行尾归一，否则 `$` 行尾锚点/`- item` 匹配全废（t25 实测踩过）。 */',
     '/** 现场 YAML 是 CRLF：解析前先把行尾归一，否则 `$` 行尾锚点/`- item` 匹配全废（实测踩过）。 */'),
    ('src/simbench.js',
     ' *  --engine"，于是又退回 `plan_route.py` 的默认 `tour` —— 也就是 t73「乱算」原样复发，',
     ' *  --engine"，于是又退回 `plan_route.py` 的默认 `tour` —— 也就是「乱算」原样复发，'),
    ('src/simbench.js',
     '        // 去 YAML 引号（`engine: "tour"` 与 `engine: tour` 同义；带引号时旧实现会误判未知）',
     '        // 去 YAML 引号（`engine: "tour"` 与 `engine: tour` 同义；带引号时会误判未知）'),
    ('src/simbench.js',
     ' *  config 时同口径）。`0.0` 是**合法值**（= 关冗余，回到 t4 之前的行为），只有**取不到值**',
     ' *  config 时同口径）。`0.0` 是**合法值**（= 关冗余），只有**取不到值**'),
    ('src/simbench.js',
     ' *    车端 t9 的 planner 入参，按 `appendIncrementalArgs` 白名单化后追加到 `--engine` 之后；',
     ' *    车端 planner 入参，按 `appendIncrementalArgs` 白名单化后追加到 `--engine` 之后；'),
    ('src/simbench.js',
     ' *   理由：`targetYaw`（t28 靶板朝向）从表单到台账的链条里，只有这一段是**纯函数**，',
     ' *   理由：`targetYaw`（靶板朝向）从表单到台账的链条里，只有这一段是**纯函数**，'),
    ('src/simbench.js',
     '  //   键表里（t28 先例，scripts/test-sim-map-render.mjs E 组钉死 sp({}) 恰 7 键），',
     '  //   键表里（先例：scripts/test-sim-map-render.mjs E 组钉死 sp({}) 恰 7 键），'),
    ('src/simbench.js',
     '  // （规格 §D）：无人机协同开关 —— 仅 `true` 通过（false/字符串/其它一律 null 丢弃），',
     '  // 无人机协同开关（规格 §D）—— 仅 `true` 通过（false/字符串/其它一律 null 丢弃），'),
    ('src/simbench.js',
     '  //   sp({}) 键表保持 t28 的 7 键（守卫 E 组钉死）、台账形状与 OFF 现状逐字一致（§D.1）。',
     '  //   sp({}) 键表保持 7 键（守卫 E 组钉死）、台账形状与 OFF 现状逐字一致（§D.1）。'),
    ('src/simbench.js',
     '  // 带键才挂 —— sp({}) 的键表保持 t28 的 7 键不变（守卫 E 组钉死），不破 61/61。',
     '  // 带键才挂 —— sp({}) 的键表保持 7 键不变（守卫 E 组钉死），不破 61/61。'),
    ('src/simbench.js',
     ' * 这两条是**照着 sim_cli 的真实排版**钉的（t21 实测：不这么写会抓错行）：',
     ' * 这两条是**照着 sim_cli 的真实排版**钉的（实测：不这么写会抓错行）：'),
    ('src/simbench.js',
     '    // （§D.2）：无人机协同也走 override 路径 —— drone 开 ⇒ 起降格注入 override 图',
     '    // 无人机协同也走 override 路径（§D.2）—— drone 开 ⇒ 起降格注入 override 图'),
    ('src/simbench.js',
     '      // （§D.3）：drone 台账字段 —— **规划成败都落**（422 plan-failed 也在，证约束在场）。',
     '      // drone 台账字段（§D.3）—— **规划成败都落**（422 plan-failed 也在，证约束在场）。'),
    ('src/simbench.js',
     '      //   与带后缀**逐字节相同**，t25 实测），所以给 `--route` 的那份剥掉后缀，',
     '      //   与带后缀**逐字节相同**），所以给 `--route` 的那份剥掉后缀，'),
    ('src/simbench.js',
     ' *    见本卡交付的回归输出：44 ↔ 44 逐 token 相等。',
     ' *    验收口径：页面与车端 44 ↔ 44 逐 token 相等（见回归输出）。'),
    ('src/simbench.js',
     ' *    用来在车端还没接线时钉住"声明后真的会传、产物 == 车端 planner"这条 E2E。 */',
     ' *    用来钉住"声明后真的会传、产物 == 车端 planner"这条 E2E。 */'),
    ('src/simbench.js',
     ' *  `droneHistory()` 原先自己重抄一遍「读文件→逐行 parse→按 kind 过滤→slice(-50)」，\n'
     ' *    现统一走这里（`listHistory` 不带 kind、`droneHistory` 带 `kind:\'drone\'`）。 */',
     ' *  统一入口：`listHistory` 不带 kind、`droneHistory` 带 `kind:\'drone\'`，\n'
     ' *    避免各写一份「读文件→逐行 parse→按 kind 过滤→slice(-50)」的过滤逻辑。 */'),
    ('src/simbench.js',
     ' * 文件里的形状是 `route:` 段下的 `tokens: [...]`（展平后才叫 `route.tokens`），\n'
     ' *   所以这里找的是 `tokens:`，旧版 `mission_blocks:` 作兜底。',
     ' * 文件里的形状是 `route:` 段下的 `tokens: [...]`（展平后才叫 `route.tokens`）；\n'
     ' *   更早的 `mission_blocks:` 块序列形状仍作兼容兜底。'),
    ('src/simbench.js',
     ' *  形状：`planner:` 块（与 route:/mission: 同级）下的 `engine: tracker-safe`，允许行内注释与\n'
     ' *  YAML 引号（`engine: "tracker-safe"` 也认）。\n'
     ' *  找不到 / 值不在 plan_route.py 的 choices 里 → 返回 null（不传 --engine，保持其历史\n'
     ' *  默认 tour，与改动前逐字节一致）。导出仅供离线断言（同 sanitizeParams 的先例）。\n'
     ' *\n'
     ' *  ✅ 已闭环（2026-10-17 队长直改）：`src/dronesim.js` 的 `truthPlanEvidence()` 已复用\n'
     ' *    本函数拼 `--engine`（同引擎取证，无人机协同时间线与车端口径一致）；t12 起它的\n'
     ' *    `from_cell` / `already_covered` 也走同一条白名单链路（`appendIncrementalArgs()`）。 */',
     ' *  形状：`planner:` 块（与 route:/mission: 同级）下的 `engine: tracker-safe`，允许行内注释与\n'
     ' *  YAML 引号（`engine: "tracker-safe"` 也认）。\n'
     ' *  找不到 / 值不在 plan_route.py 的 choices 里 → 返回 null（不传 --engine，保持其历史\n'
     ' *  默认 tour，与改动前逐字节一致）。导出仅供离线断言（同 sanitizeParams 的先例）。\n'
     ' *\n'
     ' *  ✅ 已闭环：`src/dronesim.js` 的 `truthPlanEvidence()` 已复用本函数拼 `--engine`\n'
     ' *  （同引擎取证，无人机协同时间线与车端口径一致）；`from_cell` / `already_covered`\n'
     ' *  也走同一条白名单链路（`appendIncrementalArgs()`）。 */'),
    ('src/simbench.js',
     ' *  ★ 修「乱算」：页面重规划此前不传 --engine ⇒ plan_route.py 默认 tour，而车端 robot.yaml\n'
     ' *    配的是 tracker-safe —— 同一张图页面和车算出**两条不同的路线**。实测两次快照\n'
     ' *    （`--map config/map.yaml` + `--check`，robot.yaml md5 见括注）：\n'
     ' *      · md5 `4FFC53D6…`（t3 改射击距离之前）：tracker-safe **44 token**\n'
     ' *        `N10 M10 M9 L9 K9 … F3* … N5,y`（与 `mission_blocks` 逐 token 相等）\n'
     ' *        ｜不传 --engine（改前页面行为）**40 token** `N10 N9 M9 … C7 B7 A7 … H5 …`\n'
     ' *      · md5 `23DBE758…`（t3 改完 shoot_distance 1.00 之后）：tracker-safe **50 token**\n'
     ' *        ｜不传 --engine **49 token，且带 2 处 `:PIVOT` 折返**（`A2:PIVOT`/`B1:PIVOT`）\n'
     ' *    ⇒ 结论与 token 数无关：**只有跟车端同引擎才逐 token 相等**。',
     ' *  为什么必须传 --engine：页面重规划此前不传 ⇒ plan_route.py 默认 tour，而车端配置的是\n'
     ' *    tracker-safe —— 同一张图页面和车算出**两条不同的路线**（实测 token 数与路线形状都\n'
     ' *    不同，tour 产物还可能带 `:PIVOT` 折返，而仿真不建模 PIVOT）。\n'
     ' *    ⇒ **只有跟车端同引擎才逐 token 相等**。'),
    ('src/simbench.js',
     ' * 车端 t9 在 **planner 层**加了这两个入参（`plan_route_tracker_safe(..., from_cell=, already_covered=)`）；\n'
     ' * 仿真台要透传上去，口径与 `--engine` **完全同链路**：',
     ' * 车端 planner 层支持这两个入参（`plan_route_tracker_safe(..., from_cell=, already_covered=)`），\n'
     ' * 仿真台要透传上去，口径与 `--engine` **完全同链路**：'),
    ('src/simbench.js',
     ' * ⚠ 状态史（2026-10-17）：t12 开工时 `tools/plan_route.py` 里**还没有**这两条 add_argument\n'
     ' *   （t9 的交付只到 `planner.py` + `tests/test_incremental_plan.py`），当时透传按 ② 如实降级为\n'
     ' *   "不传"、台账 `incremental.dropped` 记诉求（不静默吞），E2E 由守卫的影子 CLI 钉住。\n'
     ' *   t12 收口时队长已给车端 CLI 接线：现役 CLI `supported = 2/2`，守卫 E 段自动转实跑。\n'
     ' *   若未来接线被回退，`plannerIncrementalSupport()` 自动变空，降级路径原样生效。',
     ' * 现役 CLI `supported = 2/2`，守卫 E 段实跑；若未来接线被回退，`plannerIncrementalSupport()`\n'
     ' * 自动变空，降级为"不传"、台账 `incremental.dropped` 记诉求（不静默吞）。'),

    # ── src/dronesim.js ──────────────────────────────────────────────────────
    ('src/dronesim.js',
     ' *  （规格 §D）：drone 开时取证图同样注入起降格硬避让（与最终 runSim 实跑的同一约束）——',
     ' *  无人机协同（规格 §D）：drone 开时取证图同样注入起降格硬避让（与最终 runSim 实跑的同一约束）——'),
    ('src/dronesim.js',
     ' *  增量规划透传口 —— `fromCell` / `alreadyCovered`（车端 t9 的 planner 入参）与',
     ' *  增量规划透传口 —— `fromCell` / `alreadyCovered`（车端 planner 入参）与'),
    ('src/dronesim.js',
     ' *    ⚠ 现役 CLI 尚未声明这两条开关（t9 只改到 planner 层）⇒ 如实降级为"不传"，',
     ' *    ⚠ CLI 若未声明这两条开关 ⇒ 如实降级为"不传"，'),
    ('src/dronesim.js',
     ' *    诉求记在返回的 `incremental.dropped` 里（不静默吞）；车端接线后自动生效。 */',
     ' *    诉求记在返回的 `incremental.dropped` 里（不静默吞）。 */'),
    ('src/dronesim.js',
     '    // 与页面/车端同引擎（t1 根因修复的收尾）：truth 取证也拼 --engine，',
     '    // 与页面/车端同引擎（根因修复的收尾）：truth 取证也拼 --engine，'),
    ('src/dronesim.js',
     '  // （规格 §D / 修订后验收 4）：drone 开时把开关**转发给 runSim**（带键才挂——runSimJob',
     '  // 无人机协同开关转发（规格 §D / 验收 4）：drone 开时把开关**转发给 runSim**（带键才挂——runSimJob'),
    ('src/dronesim.js',
     '      // 拟真（卡3+）：给前端画航线/扫描覆盖的完整格序列（含每点 3×3 覆盖）。',
     '      // 拟真：给前端画航线/扫描覆盖的完整格序列（含每点 3×3 覆盖）。'),
    # 第三轮补漏（复核 grep 后追加）
    ('sim.html',
     '  //   故恒发 TARGET_FACE_LEDGER（不再随旋转变化，t14 的"翻面"交互已废除）。',
     '  //   故恒发 TARGET_FACE_LEDGER（不再随旋转变化，"翻面"交互已废除）。'),
    ('sim.html',
     '      renderDroneMap(payload);                     // ★ 卡3+：无人机拟真地图（航线/覆盖/情报命中）',
     '      renderDroneMap(payload);                     // ★ 无人机拟真地图（航线/覆盖/情报命中）'),
    ('src/dronesim.js',
     ' * dronesim.js — 卡3：无人机协同仿真（规格 §C；报文契约 = 空地指挥链路手册 §2.1，降级语义 = §7）。',
     ' * dronesim.js — 无人机协同仿真（规格 §C；报文契约 = 空地指挥链路手册 §2.1，降级语义 = §7）。'),
    # 第四轮补漏（语序整理）
    ('src/simbench.js',
     '/** （规格 §D.2）：无人机协同 —— 把无人机起降格注入 override 图的 obstacles 列表。',
     '/** 无人机协同（规格 §D.2）—— 把无人机起降格注入 override 图的 obstacles 列表。'),
    ('src/simbench.js',
     '/** （规格 §D.3）：无人机扫描**走廊行**（上半场 top / 下半场 bottom，1-based 行号）。',
     '/** 无人机扫描**走廊行**（上半场 top / 下半场 bottom，1-based 行号；规格 §D.3）。'),
]


def main():
    files = {}
    errors = []
    for rel in {f for f, _, _ in FIXES}:
        files[rel] = (PLUGIN / rel).read_text(encoding='utf-8')
    for rel, old, new in FIXES:
        text = files[rel]
        count = text.count(old)
        if count != 1:
            errors.append(f'{rel}: match={count} -> {old[:70]}...')
            continue
        files[rel] = text.replace(old, new, 1)
    if errors:
        print('ABORT: ' + '\n'.join(errors))
        sys.exit(1)
    for rel, text in files.items():
        (PLUGIN / rel).write_text(text, encoding='utf-8')
    print(f'ok: {len(FIXES)} fixes')


if __name__ == '__main__':
    main()
