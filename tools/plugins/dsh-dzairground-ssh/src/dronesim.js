/**
 * dronesim.js — 无人机协同仿真（规格 §C；报文契约 = 空地指挥链路手册 §2.1，降级语义 = §7）。
 *
 * 决策已拍板（2026-10-17，docs/无人机拟真与动态规划_方案.md §7）：
 *   1. 全量重规划：车随情报用**整张 truth override 图**重算整条路线（雷障靶差异也触发），
 *      非只换穿洞三连。落地 = runSim 的 field override 通道（runSimJob 内部 plan_route 全量重规划）。
 *   2. 重规划上限 2：对应 lower/upper 两个桥洞 checkpoint；land 封口不规划。
 *   3. 情报先导：车不再 t=0 先验出发，而是等 first checkpoint（lower 桥洞）情报到达才规划+出发。
 *   4. 航线 = 桥洞优先短程（3 直线段 / 2 拐弯，≤4 段）；雷点「顺路覆盖」不单独拐弯。
 *   5. 先仿真台全绿，再动车端 mission.py 滚动重规划（本轮只动仿真台侧）。
 *
 * 有意边界：
 *   a. 三关采纳（K 列 → 半场行段 → ∈robot.yaml cross_*_candidates）是超集、结论不放宽（手册 §7:237）。
 *   b. 不建模打靶：target_zone 只进 field_diff，无射击动作/耗时/得分（规格 §C.6）。
 *   c. 情报缺失/被拒 → bridges 回落先验默认完赛（铁律「宁可漏打不可卡死」不破）。
 */
import fs from 'node:fs'
import path from 'node:path'
import { execFile } from 'node:child_process'
import { appendIncrementalArgs, appendMeta, applyFieldToMapText, COL_LETTERS, DRONE_FALLBACK, fieldSpec, injectDroneAvoidToMapText, normalizeCells, plannerEngineOf, readMeta, runSim, SIM_ROOT } from './simbench.js'

/** 扫描节拍：每点 TICK_S 秒（时间线与并行窗的最小刻度）。 */
export const TICK_S = 0.5
/** 起降格兜底的 `DRONE_FALLBACK` 与列字母 `COL_LETTERS` 一律从 simbench 取（原先各抄一份）。 */
const LETTERS = COL_LETTERS.split('')

const colIdx = cell => LETTERS.indexOf(String(cell)[0].toUpperCase())
const rowOf = cell => Number(String(cell).slice(1))
const sameCell = (a, b) => String(a) === String(b)

/** 3×3 视场（裁剪进 14×11），按列→行确定序。 */
function coversOf(cell) {
  const c0 = colIdx(cell), r0 = rowOf(cell)
  const out = []
  if (c0 < 0 || !Number.isFinite(r0)) return out
  for (let dc = -1; dc <= 1; dc += 1) {
    for (let dr = -1; dr <= 1; dr += 1) {
      const c = c0 + dc, r = r0 + dr
      if (c >= 0 && c < LETTERS.length && r >= 1 && r <= 11) out.push(LETTERS[c] + r)
    }
  }
  return out
}

/** 读 robot.yaml 的列表/标量键（只读正则；找不到用规格给的兜底值）。 */
function robotConf() {
  const file = path.join(SIM_ROOT, 'config', 'robot.yaml')
  let text = ''
  try { text = fs.readFileSync(file, 'utf8') } catch { text = '' }
  const list = (key, fallback) => {
    const m = text.match(new RegExp(`^\\s*${key}:\\s*\\[([^\\]]*)\\]`, 'm'))
    if (!m) return fallback
    const vals = m[1].split(',').map(s => s.replace(/['"\s]/g, '').toUpperCase()).filter(Boolean)
    return vals.length ? vals : fallback
  }
  const scalar = (key, fallback) => {
    const m = text.match(new RegExp(`^\\s*${key}:\\s*["']?([A-N]\\d{1,2})["']?`, 'm'))
    return m ? m[1].toUpperCase() : fallback
  }
  return {
    lowerCandidates: list('cross_lower_candidates', ['K9']),
    upperCandidates: list('cross_upper_candidates', ['K5']),
    lowerDefault: scalar('cross_lower_default', 'K9'),
    upperDefault: scalar('cross_upper_default', 'K5'),
    source: file,
  }
}

/**
 * 桥洞优先短程俯瞰航线（决策 #4：≤4 直线段、雷点拐弯区分）。三原则：最短 / 拐弯最少 / 信息最快。
 *
 * 三段正交（横→竖→横，共 2 个 90° 拐弯，远少于旧 U 形 6 段）：
 *   ① 起飞行横飞穿 K 列：3×3 视场在接近 K 列时即覆盖 lower 桥洞（信息最快——桥洞第 1 段就拿到）；
 *   ② 沿 K 列竖飞：途经 upper 桥洞行，顺路覆盖 upper 桥洞；
 *   ③ 降落行横飞：封口。
 * 雷/障/靶：只在三段视场内「顺路覆盖」，**不为雷点单独拐弯**（决策 #4 的「雷点拐弯区分」——
 *   桥洞段的拐弯必须做、雷点段的拐弯能省则省）；覆盖不到的雷点 = missing，车端按 map.yaml 静态默认走。
 *
 * checkpoint：每个桥洞 = 累积视场「首次」覆盖到它的那个航点（信息最快，宁可早 0.5s 不晚）。
 * 返回 {takeoff, land, points:[{cell, covers, checkpoint?}], checkpoints:[{id,target,cell,index,t}]}（纯函数）。
 */
export function scanPlan({ field, drone } = {}) {
  const takeoff = (drone && drone.takeoff) || (field && field.drone && field.drone.takeoff) || DRONE_FALLBACK.takeoff
  const land = (drone && drone.land) || (field && field.drone && field.drone.land) || DRONE_FALLBACK.land
  const bridges = normalizeCells((field && field.bridges) || [])
  const lowerBridge = bridges.find(c => c[0] === 'K' && rowOf(c) >= 7)
  const upperBridge = bridges.find(c => c[0] === 'K' && rowOf(c) <= 6)
  const tCol = colIdx(takeoff), tRow = rowOf(takeoff)
  const lCol = colIdx(land), lRow = rowOf(land)
  const kCol = LETTERS.indexOf('K')

  const cells = []
  const push = cell => { if (cell && cells[cells.length - 1] !== cell) cells.push(cell) }
  push(takeoff)
  if (tCol < 0 || !Number.isFinite(tRow)) return { takeoff, land, points: [], checkpoints: [] }   // 起飞点不合法 ⇒ 空计划
  const walk = (from, to, make) => {                                            // 含端点的方向安全步进
    const step = from <= to ? 1 : -1
    for (let v = from + step; step > 0 ? v <= to : v >= to; v += step) push(make(v))
  }
  walk(tCol, kCol, c => LETTERS[c] + tRow)        // ① 起飞行横飞穿 K 列（3×3 视场覆盖 lower 桥洞）
  walk(tRow, lRow, r => LETTERS[kCol] + r)        // ② 沿 K 列竖飞（途经 upper 桥洞行）
  walk(kCol, lCol, c => LETTERS[c] + lRow)        // ③ 降落行横飞封口
  const points = cells.map(cell => ({ cell, covers: coversOf(cell) }))

  // checkpoint：桥洞 = 累积视场首次覆盖它的航点；降落 = 最后一点（封口）。
  const firstCoverIndex = target => {
    for (let i = 0; i < points.length; i++) if ((points[i].covers || []).includes(target)) return i
    return -1
  }
  const checkpoints = []
  const mark = (id, target) => {
    if (!target) return
    const idx = firstCoverIndex(target)
    if (idx < 0) return
    checkpoints.push({ id, target, cell: points[idx].cell, index: idx, t: Math.round(idx * TICK_S * 10) / 10 })
  }
  mark('lower', lowerBridge)
  mark('upper', upperBridge)
  checkpoints.push({ id: 'land', target: land, cell: land, index: points.length - 1, t: Math.round((points.length - 1) * TICK_S * 10) / 10 })
  return { takeoff, land, points, checkpoints }
}

/**
 * 三路情报（手册 §2.1 同构报文 + 三关采纳判定）。覆盖外的半场**不产报文 = missing**。
 *   · bridge_passable：truth 桥格归属半场（行 ≥7 lower / ≤6 upper），且 ∈ 该半场 K 穿越点视场才发；
 *   · target_zone / field_verify：扫描覆盖并集过滤，非空才发（车侧行为=field_diff，只报差异）。
 *
 * 增量交付（决策 #3 情报先导）：每条 `bridge_passable` 带 `checkpoint` + `t` ——
 *   = 该桥洞首次被累积视场覆盖到的 checkpoint（来自 `plan.checkpoints`）与时刻，
 *     让车端能「按批次」滚动重规划，而不是一次拿全。
 * 返回报文数组；每条附 decision/reason/default/scanAt（手册明示：新增字段安全、新增类型不安全）。
 */
export function intelAt({ plan, truthBridges, field } = {}) {
  const conf = robotConf()
  const points = (plan && plan.points) || []
  const truth = normalizeCells(
    Array.isArray(truthBridges) && truthBridges.length ? truthBridges : (field && field.bridges) || [])
  const union = new Set()
  points.forEach(p => (p.covers || []).forEach(c => union.add(c)))
  const ts = Math.round(Date.now() / 100) / 10
  let seq = 0
  const next = () => { seq += 1; return seq }

  // checkpoint 反查：half → {id, t}（scanPlan 已标 lower/upper/land；找不到则不标）
  const cps = (plan && plan.checkpoints) || []
  const cpOfHalf = half => cps.find(cp => cp.id === half) || null

  const out = []
  const crossPoints = points.filter(p => p.cell[0] === 'K')
  for (const block of truth) {
    const row = rowOf(block)
    const half = row >= 7 ? 'lower' : 'upper'
    const crossing = crossPoints.find(p => (rowOf(p.cell) >= 7) === (half === 'lower'))
    // 视场校验：truth 格必须落在该半场 K 穿越点的 3×3 内（K7/K2 等覆盖外 = missing，不产报文）
    if (!crossing || !(crossing.covers || []).includes(block)) continue
    // 三关：① K 列 ② 半场行段（upper 1-6 / lower 7-11）③ ∈ robot.yaml 候选集
    const candidates = half === 'lower' ? conf.lowerCandidates : conf.upperCandidates
    const halfDefault = half === 'lower' ? conf.lowerDefault : conf.upperDefault
    let decision = 'adopted'
    let reason = `三关全过（K 列 ✓ 行段 ✓ 候选 ${JSON.stringify(candidates)} ✓）`
    if (block[0].toUpperCase() !== 'K') {
      decision = 'rejected'; reason = `第①关失败：非 K 列（${block}）— 手册 §7 rejected`
    } else if (half === 'upper' ? (row < 1 || row > 6) : (row < 7 || row > 11)) {
      decision = 'rejected'; reason = `第②关失败：半场行段不符（${half} 要求 ${half === 'upper' ? '行 1-6' : '行 7-11'}，实为行 ${row}）`
    } else if (!candidates.includes(block)) {
      decision = 'rejected'; reason = `第③关失败：不在候选集合（${JSON.stringify(candidates)}）— 手册 §7:237`
    }
    const cp = cpOfHalf(half)
    out.push({
      type: 'bridge_passable', half, block,
      confidence: 0.9, ts, seq: next(),
      decision, reason, default: halfDefault, scanAt: crossing.cell,
      // 增量交付：首次覆盖到它的 checkpoint（决策 #3 情报先导）
      ...(cp ? { checkpoint: cp.id, t: cp.t } : {}),
    })
  }
  const coveredTargets = normalizeCells((field && field.targets) || []).filter(c => union.has(c))
  if (coveredTargets.length) {
    out.push({
      type: 'target_zone', blocks: coveredTargets, ts, seq: next(),
      decision: 'diff-only',
      reason: '覆盖并集过滤；车侧行为=与本地 targets 差集进 field_diff，只报差异不改路线（手册 §2.1），仿真不建模打靶',
      scanAt: `${points.length} 点覆盖`,
    })
  }
  const coveredMines = normalizeCells((field && field.mines) || []).filter(c => union.has(c))
  const coveredObstacles = normalizeCells((field && field.obstacles) || []).filter(c => union.has(c))
  if (coveredMines.length || coveredObstacles.length) {
    out.push({
      type: 'field_verify', mines: coveredMines, obstacles: coveredObstacles, ts, seq: next(),
      decision: 'diff-only',
      reason: '覆盖并集过滤；车侧行为=与本地 map 差集进 field_diff（手册 §2.1）',
      scanAt: `${points.length} 点覆盖`,
    })
  }
  return out
}

/** 真值 override 图上跑 plan_route --check（重规划取证；失败由调用方降级走默认、不计数）。
 *  无人机协同（规格 §D）：drone 开时取证图同样注入起降格硬避让（与最终 runSim 实跑的同一约束）——
 *    否则时间线里的 token 数/穿洞来自「可穿 N8/M2 的图」，与车实际跑的避让路线不同口径。
 *  增量规划透传口 —— `fromCell` / `alreadyCovered`（车端 planner 入参）与
 *    `--engine` **同链路**：由 `appendIncrementalArgs()` 按 `tools/plan_route.py` 的 argparse
 *    真源白名单化后追加。缺省 `null/[]` ⇒ argv 与改动前逐字节相同（决策 #1 仍是全量重规划）。
 *    ⚠ CLI 若未声明这两条开关 ⇒ 如实降级为"不传"，
 *    诉求记在返回的 `incremental.dropped` 里（不静默吞）。 */
export function truthPlanEvidence({ board, truth, id, label, drone, fromCell = null, alreadyCovered = [] }) {
  const dir = path.join(SIM_ROOT, 'artifacts', 'sim', 'drone', id, `truth_override_${label ?? 'x'}`)
  const out = { dir, exitCode: null, tokenCount: null, crossing: null, checkExit: null, stderrTail: '', tokens: [], argv: [], incremental: null }
  try {
    fs.mkdirSync(dir, { recursive: true })
    const spec = fieldSpec()
    const mapPath = path.join(dir, 'map.yaml')
    const robotPath = path.join(dir, 'robot.yaml')
    const baseText = applyFieldToMapText(fs.readFileSync(spec.source, 'utf8'),
      { mines: board.mines, obstacles: board.obstacles, targets: board.targets, bridges: truth })
    fs.writeFileSync(mapPath, drone === true
      ? injectDroneAvoidToMapText(baseText, [spec.drone?.takeoff, spec.drone?.land].filter(Boolean))
      : baseText, 'utf8')
    fs.copyFileSync(spec.robot.path, robotPath)
  } catch (error) {
    out.stderrTail = `写 truth override 失败：${error?.message ?? error}`
    return Promise.resolve(out)
  }
  return new Promise(resolve => {
    // 与页面/车端同引擎（根因修复的收尾）：truth 取证也拼 --engine，
    //   否则时间线用 plan_route 默认 tour、车实跑 tracker-safe，token/穿洞口径分裂。
    const argv = ['tools/plan_route.py', '--config', path.join(dir, 'robot.yaml'),
      '--map', path.join(dir, 'map.yaml'), '--emit', 'json', '--check', '--quiet']
    const engine = plannerEngineOf(path.join(dir, 'robot.yaml'))
    if (engine) argv.push('--engine', engine)
    // 增量参数同链路透传（白名单化；未声明 ⇒ 不加，诉求进 dropped）
    out.incremental = appendIncrementalArgs(argv, { fromCell, alreadyCovered }).applied
    out.argv = argv.slice()
    execFile('python', argv, {
      cwd: SIM_ROOT, timeout: 300 * 1000, encoding: 'utf8',
      env: { ...process.env, PYTHONPATH: path.join(SIM_ROOT, 'src', 'dzbrain'), PYTHONIOENCODING: 'utf-8', PYTHONUTF8: '1' },
    }, (error, stdout, stderr) => {
      out.exitCode = error ? (typeof error.code === 'number' ? error.code : 1) : 0
      const open = String(stdout ?? '').indexOf('{')
      if (open >= 0) {
        try {
          // --emit json 的真源形状：{config,map,plans:[{tokens,crossing…}],failures,route_check:[{exit}]}
          const payload = JSON.parse(String(stdout).slice(open))
          const plan0 = payload.plans?.[0] ?? null
          out.tokenCount = Array.isArray(plan0?.tokens) ? plan0.tokens.length : null
          out.tokens = Array.isArray(plan0?.tokens) ? plan0.tokens : []
          out.crossing = plan0?.crossing ?? null
          out.checkExit = payload.route_check?.[0]?.exit ?? null
          out.failures = payload.failures ?? []
        } catch { /* 解析失败保持 null，如实进 timeline 明细 */ }
      }
      out.stderrTail = String(stderr ?? '').split('\n').filter(l => l.trim() !== '').slice(-4).join('\n')
      resolve(out)
    })
  })
}

/**
 * 一次空地协同（同步单跑，规格 §D 签名，决策 #1 全量重规划 + #3 情报先导）：
 * 无人机沿桥洞优先短程航线扫描 → 每个 checkpoint（lower/upper）交付情报批次
 * → 车随情报**真重规划**（plan_route 用 truth override 图算整条路线，全量重规划非只换洞）
 * → 最终路线 = 情报封口后最后一次全量规划的路线，车 record 用最终路线跑。
 *
 * 时序语义（决策 #3）：车不再 t=0 先验出发，而是等 first checkpoint（lower 桥洞）情报到达才规划+出发。
 * 上限：全量重规划 ≤2 次（决策 #2，对应 lower/upper 两个 checkpoint；land 封口不规划）。
 */
export async function runDroneSim({ field, seed, truthBridges, carDefaultBridges, drone, label = 'drone' } = {}) {
  const spec = fieldSpec()
  if (!spec.ok) return { ok: false, code: 400, error: spec.error || '读不到场地' }
  const board = field && Array.isArray(field.mines) ? { ...spec, ...field } : spec   // 页面棋盘 field 可覆盖真源四键
  const conf = robotConf()
  const priorRaw = (Array.isArray(carDefaultBridges) && carDefaultBridges.length ? carDefaultBridges : [conf.lowerDefault, conf.upperDefault])
  const prior = { lower: normalizeCells([priorRaw[0]])[0] || conf.lowerDefault, upper: normalizeCells([priorRaw[1]])[0] || conf.upperDefault }

  const id = `${Date.now()}_${label}_drone`
  const plan = scanPlan({ field: board })
  const truth = normalizeCells(
    Array.isArray(truthBridges) && truthBridges.length ? truthBridges : board.bridges || [])
  const intel = intelAt({ plan, truthBridges: truth, field: board })
  const bridgeMsgs = intel.filter(m => m.type === 'bridge_passable')
  const checkpoints = plan.checkpoints || []

  const timeline = [
    { t: 0, actor: 'drone', event: 'takeoff', cell: plan.takeoff, detail: `桥洞优先短程航线 ${plan.points.length} 点 / ${checkpoints.length} checkpoint，步长 1 格 3×3 视场` },
  ]
  plan.points.forEach((p, i) => timeline.push({ t: Math.round(i * TICK_S * 10) / 10, actor: 'drone', event: 'scan', cell: p.cell }))

  // ── 决策 #1/#3：按 checkpoint 顺序滚动全量重规划 ──
  // lower 桥洞 = 首次规划（情报先导，车此时才出发）；upper 桥洞 = 第二次全量重规划；land 封口。
  let reroutes = 0
  let confirmed = 0
  const perHalf = { lower: 0, upper: 0 }
  const plans = {}          // half → evidence
  let finalTokens = null   // 情报封口后最后一次全量规划的 token

  const planIdx = (target) => plan.points.findIndex(p => sameCell(p.cell, target))

  for (const half of ['lower', 'upper']) {
    const msg = bridgeMsgs.find(m => m.half === half)
    const cp = checkpoints.find(c => c.id === half)
    // 情报到达时刻 = checkpoint t（首次覆盖），不是 scanAt
    const tInt = cp ? cp.t : (msg ? Math.round((planIdx(msg.scanAt) + 1) * TICK_S * 10) / 10 : Math.round((plan.points.length - 1) * TICK_S * 10) / 10)
    if (!msg) {
      const priorCell = prior[half]
      timeline.push({ t: tInt, actor: 'car', event: 'no-intel', cell: priorCell, detail: `${half} 半场 missing（覆盖外不产报文）→ 走先验默认 ${priorCell}` })
      continue
    }
    if (msg.decision === 'rejected') {
      timeline.push({ t: tInt, actor: 'car', event: 'rejected', cell: msg.block, detail: `${msg.reason} → 走默认 ${msg.default}` })
      continue
    }
    if (perHalf[half] >= 1 || reroutes >= 2) {                 // 决策 #2：总 ≤2
      timeline.push({ t: tInt, actor: 'car', event: 'rejected', cell: msg.block, detail: `已达全量重规划上限 2 → 维持当前路线` })
      continue
    }
    // 决策 #1：全量重规划 —— 整张 truth override 图重算整条路线（不是只换穿洞三连）
    const evidence = await truthPlanEvidence({ board, truth, id, label: half, drone })
    if (evidence.exitCode !== 0 || !evidence.tokenCount) {     // 取证失败：降级走默认，不计数
      const priorCell = prior[half]
      timeline.push({ t: tInt, actor: 'car', event: 'reroute-fallback', cell: priorCell, detail: `全量重规划取证失败（exit=${evidence.exitCode}）→ 走默认 ${priorCell}` })
      continue
    }
    perHalf[half] += 1
    reroutes += 1
    plans[half] = evidence
    finalTokens = evidence.tokens
    timeline.push({
      t: tInt, actor: 'car', event: (half === 'lower' ? 'first-plan' : 'reroute'), cell: msg.block,
      detail: `${half === 'lower' ? '首批情报到达，首次规划' : '第二批情报到达，全量重规划'}（${msg.block}）→ 整图重算 ${evidence.tokenCount} token / 穿洞 ${JSON.stringify(evidence.crossing)}`,
    })
  }

  // 决策 #3：车等「情报封口」后才出发。最终路线 = 用 truth 图全量重规划的路线。
  // 落地方式 = runSim 的 field override 通道：传 field（四键 + bridges=truth）⇒
  //   runSimJob 内部生成 override 图 + plan_route **全量重规划** + 用新路线跑 ——
  //   这正是决策 #1「全量重规划」（雷障靶差异也重算整条路线，非只换桥洞）的正确语义。
  //   情报全部缺失/被拒（无采纳）时，bridges 回落先验默认，行为 = 走默认完赛（铁律兜底不破）。
  // 无人机协同开关转发（规格 §D / 验收 4）：drone 开时把开关**转发给 runSim**（带键才挂——runSimJob
  //   里起降格经同一 override 图 obstacles 注入 + record.plan.drone 台账 + 路线硬避让取证），
  //   参数贯通 = 页面 → /sim/drone/run → runDroneSim → runSim(runSimJob) 全链一致。
  const finalBridges = truth.length ? truth : [prior.lower, prior.upper]
  const finalField = {
    mines: board.mines, obstacles: board.obstacles, targets: board.targets,
    bridges: finalBridges,
  }
  const runArgs = { label, field: finalField }
  if (drone === true) runArgs.drone = true
  if (Number.isFinite(Number(seed))) runArgs.seed = Number(seed)
  const carResult = await runSim(runArgs)
  if (!carResult.ok) return { ok: false, code: carResult.code ?? 400, error: carResult.error }

  const finishT = Number(carResult.record?.metrics?.time_s) || 0
  const landT = plan.points.length > 1 ? Math.round((plan.points.length - 1) * TICK_S * 10) / 10 : 0
  timeline.push({ t: landT, actor: 'drone', event: 'land', cell: plan.land })
  timeline.push({ t: finishT, actor: 'car', event: 'finish', cell: carResult.record.status, detail: `完赛=${carResult.record?.metrics?.finished === true}` })
  timeline.sort((a, b) => a.t - b.t)

  const meta = {
    id, kind: 'drone', at: new Date().toISOString(), label,
    seed: Number.isFinite(Number(seed)) ? Number(seed) : null,
    drone: drone === true,                       // 无人机台账留痕（车台账在 record.plan.drone）
    status: carResult.record.status, carRecordId: carResult.record.id,
    prior, truth, reroutes, confirmed,
    intel: bridgeMsgs.map(m => ({ half: m.half, block: m.block, decision: m.decision, checkpoint: m.checkpoint })),
    planPoints: plan.points.length, parallelS: Math.round(Math.min(finishT || landT, landT || finishT) * 10) / 10,
  }
  try { appendMeta(meta) } catch { /* 台账失败不吞主结果，timeline/record 仍返回 */ }
  return {
    ok: true, timeline, reroutes, confirmed, record: carResult.record,
    scan: {
      takeoff: plan.takeoff, land: plan.land, points: plan.points.length,
      // 拟真：给前端画航线/扫描覆盖的完整格序列（含每点 3×3 覆盖）。
      //   index.js 原样透传；此前只回 points 计数，前端没有画图的素材。
      path: plan.points.map(p => p.cell),
      cells: plan.points.map(p => ({ cell: p.cell, covers: p.covers })),
      // 增量交付（决策 #3）：checkpoint 序列，前端画「情报交付点 + 车改道时刻」。
      checkpoints,
    },
    intel, plans, id,
  }
}

/** 无人机协同台账：台账读/过滤统一走 `simbench.readMeta({ kind:'drone' })`（不再自拼一遍）。 */
export function droneHistory() {
  return { ok: true, runs: readMeta({ kind: 'drone' }) }
}
