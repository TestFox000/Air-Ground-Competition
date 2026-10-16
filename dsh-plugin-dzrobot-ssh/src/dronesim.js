/**
 * dronesim.js — 卡3：无人机协同仿真（规格 §C；报文契约 = 空地指挥链路手册 §2.1，降级语义 = §7）。
 *
 * ★ 有意边界（读结论前先知道）：
 *   1. sim_cli 不支持中途换 token ⇒ 本仿真的「小车 record」= 先验默认路线单跑；
 *      途中的「重规划」只产出 plan_route --check 的真值 override 取证（artifacts/sim/drone/<id>/），
 *      不回灌正在跑的仿真。与真车「出发前换洞」的手册 L153 差异见规格 §C.4 标注。
 *   2. 采纳校验走**三关**（K 列 → 半场行段 → ∈robot.yaml cross_*_candidates）；
 *      车侧 mission 是两关（_intel_cell_ok），三关是超集、结论不放宽 —— 出处手册 §7:237
 *      （该行把「非 K 列 / 半场行段不符 / 不在候选」并列为 rejected）。任一不过 → rejected 走默认。
 *   3. 不建模打靶：target_zone 只进 field_diff 显示，无射击动作/耗时/得分（规格 §C.6）。
 *   4. 小车 t=0 双发不等情报（规格 §C.5，用户明确的并行语义）；等不到/被拒走先验默认完赛。
 */
import fs from 'node:fs'
import path from 'node:path'
import { execFile } from 'node:child_process'
import { appendMeta, applyFieldToMapText, fieldSpec, normalizeCells, runSim, SIM_ROOT } from './simbench.js'

/** 扫描节拍：每点 TICK_S 秒（时间线与并行窗的最小刻度）。 */
export const TICK_S = 0.5
/** 起降点兜底（fieldSpec().drone 读不到 map 注释时用，与 simbench 内建兜底同值）。 */
const DRONE_FALLBACK = { takeoff: 'N8', land: 'M2' }
const LETTERS = 'ABCDEFGHIJKLMN'.split('')

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
 * U 形俯瞰航线（规格 §C.2：步长 1 格、每点 3×3 覆盖）。
 * 六段正交：起飞列下到通口行 → 该行西扫到 A → A 列北上越隔板（隔板只封 D~N，A 列可越 6/7）
 * → 上通口行东扫到末列 → 末列降到落点行 → 横移落点。走廊行由 field.bridges 推。
 * 返回 {takeoff, land, points:[{cell, covers:[…]}]}（纯函数，不读写文件）。
 */
export function scanPlan({ field, drone } = {}) {
  const takeoff = (drone && drone.takeoff) || (field && field.drone && field.drone.takeoff) || DRONE_FALLBACK.takeoff
  const land = (drone && drone.land) || (field && field.drone && field.drone.land) || DRONE_FALLBACK.land
  const bridges = normalizeCells((field && field.bridges) || [])
  const lowerBridge = bridges.find(c => c[0] === 'K' && rowOf(c) >= 7)
  const upperBridge = bridges.find(c => c[0] === 'K' && rowOf(c) <= 6)
  const lowerRow = lowerBridge ? rowOf(lowerBridge) : 9           // 下半场走廊行，缺省 9
  const upperRow = upperBridge ? rowOf(upperBridge) : 5           // 上半场走廊行，缺省 5
  const tCol = colIdx(takeoff), tRow = rowOf(takeoff)
  const lCol = colIdx(land), lRow = rowOf(land)
  const lastCol = LETTERS.length - 1

  const cells = []
  const push = cell => { if (cell && cells[cells.length - 1] !== cell) cells.push(cell) }
  push(takeoff)
  if (tCol < 0 || !Number.isFinite(tRow)) return { takeoff, land, points: [] }   // 起飞点不合法 ⇒ 空计划
  const walk = (from, to, make) => {                                            // 含端点的方向安全步进
    const step = from <= to ? 1 : -1
    for (let v = from + step; step > 0 ? v <= to : v >= to; v += step) push(make(v))
  }
  walk(tRow, lowerRow, r => LETTERS[tCol] + r)        // ① 起飞列竖直到下通口行
  walk(tCol, 0, c => LETTERS[c] + lowerRow)           // ② 该行西扫到 A 列
  walk(lowerRow, upperRow, r => LETTERS[0] + r)       // ③ 沿 A 列北上越隔板
  walk(0, lastCol, c => LETTERS[c] + upperRow)        // ④ 上通口行东扫到末列
  walk(upperRow, lRow, r => LETTERS[lastCol] + r)     // ⑤ 末列降到落点行
  walk(lastCol, lCol, c => LETTERS[c] + lRow)         // ⑥ 横移落点
  return { takeoff, land, points: cells.map(cell => ({ cell, covers: coversOf(cell) })) }
}

/**
 * 三路情报（手册 §2.1 同构报文 + 三关采纳判定）。覆盖外的半场**不产报文 = missing**。
 *   · bridge_passable：truth 桥格归属半场（行 ≥7 lower / ≤6 upper），且 ∈ 该半场 K 穿越点视场才发；
 *   · target_zone / field_verify：扫描覆盖并集过滤，非空才发（车侧行为=field_diff，只报差异）。
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
    out.push({
      type: 'bridge_passable', half, block,
      confidence: 0.9, ts, seq: next(),
      decision, reason, default: halfDefault, scanAt: crossing.cell,
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

/** 真值 override 图上跑 plan_route --check（重规划取证；失败由调用方降级走默认、不计数）。 */
function truthPlanEvidence({ board, truth, id }) {
  const dir = path.join(SIM_ROOT, 'artifacts', 'sim', 'drone', id, 'truth_override')
  const out = { dir, exitCode: null, tokenCount: null, crossing: null, checkExit: null, stderrTail: '' }
  try {
    fs.mkdirSync(dir, { recursive: true })
    const spec = fieldSpec()
    const mapPath = path.join(dir, 'map.yaml')
    const robotPath = path.join(dir, 'robot.yaml')
    fs.writeFileSync(mapPath, applyFieldToMapText(fs.readFileSync(spec.source, 'utf8'),
      { ...board, bridges: truth }), 'utf8')
    fs.copyFileSync(spec.robot.path, robotPath)
  } catch (error) {
    out.stderrTail = `写 truth override 失败：${error?.message ?? error}`
    return Promise.resolve(out)
  }
  return new Promise(resolve => {
    execFile('python', ['tools/plan_route.py', '--config', path.join(dir, 'robot.yaml'),
      '--map', path.join(dir, 'map.yaml'), '--emit', 'json', '--check', '--quiet'], {
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
 * 一次空地协同（同步单跑，规格 §D 签名）：
 * t=0 双发（车按先验默认出发不等情报）→ 无人机逐点扫描 → 情报三关判定
 * → 采纳且≠先验 ⇒ 每半场 1 次、总 ≤2 的重规划取证；相同 ⇒ confirmed；rejected/missing ⇒ 走默认。
 * 小车 record = 先验路线单跑（见文件头有意边界 1）。
 */
export async function runDroneSim({ field, seed, truthBridges, carDefaultBridges, label = 'drone' } = {}) {
  const spec = fieldSpec()
  if (!spec.ok) return { ok: false, code: 400, error: spec.error || '读不到场地' }
  const board = field && Array.isArray(field.mines) ? { ...spec, ...field } : spec   // 页面棋盘 field 可覆盖真源四键
  const conf = robotConf()
  const priorRaw = (Array.isArray(carDefaultBridges) && carDefaultBridges.length ? carDefaultBridges : [conf.lowerDefault, conf.upperDefault])
  const prior = { lower: normalizeCells([priorRaw[0]])[0] || conf.lowerDefault, upper: normalizeCells([priorRaw[1]])[0] || conf.upperDefault }

  // 小车 t=0 出发：先验默认图单跑；409（上跑/压测中）与 422（规划不过）原样透传作闸
  const runArgs = { label, field: { mines: board.mines, obstacles: board.obstacles, targets: board.targets, bridges: [prior.lower, prior.upper] } }
  if (Number.isFinite(Number(seed))) runArgs.seed = Number(seed)
  const carResult = await runSim(runArgs)
  if (!carResult.ok) return { ok: false, code: carResult.code ?? 400, error: carResult.error }

  const id = `${Date.now()}_${label}_drone`
  const plan = scanPlan({ field: board })
  const truth = normalizeCells(
    Array.isArray(truthBridges) && truthBridges.length ? truthBridges : board.bridges || [])
  const intel = intelAt({ plan, truthBridges: truth, field: board })
  const bridgeMsgs = intel.filter(m => m.type === 'bridge_passable')

  const timeline = [
    { t: 0, actor: 'car', event: 'depart', cell: `${prior.lower}/${prior.upper}`, detail: '按先验默认出发，不等情报（规格 §C.5 并行语义）' },
    { t: 0, actor: 'drone', event: 'takeoff', cell: plan.takeoff, detail: `U 形航线 ${plan.points.length} 点，步长 1 格 3×3 视场` },
  ]
  plan.points.forEach((p, i) => timeline.push({ t: Math.round(i * TICK_S * 10) / 10, actor: 'drone', event: 'scan', cell: p.cell }))

  let reroutes = 0
  let confirmed = 0
  const perHalf = { lower: 0, upper: 0 }
  const plans = {}
  for (const half of ['lower', 'upper']) {
    const priorCell = prior[half]
    const msg = bridgeMsgs.find(m => m.half === half)
    const idx = msg ? plan.points.findIndex(p => sameCell(p.cell, msg.scanAt)) : -1
    const tInt = Math.round(((idx >= 0 ? idx : plan.points.length - 1) + 1) * TICK_S * 10) / 10
    if (!msg) {
      timeline.push({ t: tInt, actor: 'car', event: 'no-intel', cell: priorCell, detail: `${half} 半场 missing（覆盖外不产报文）→ 走先验默认 ${priorCell}` })
      continue
    }
    if (msg.decision === 'rejected') {
      timeline.push({ t: tInt, actor: 'car', event: 'rejected', cell: msg.block, detail: `${msg.reason} → 走默认 ${msg.default}` })
      continue
    }
    if (msg.block === priorCell) {
      confirmed += 1
      timeline.push({ t: tInt, actor: 'car', event: 'confirmed', cell: msg.block, detail: '情报与先验相同 → 不重跑规划（规格 §C.4）' })
      continue
    }
    if (perHalf[half] >= 1 || reroutes >= 2) {                 // 两口径更严者：每半场 1 次、总 ≤2
      timeline.push({ t: tInt, actor: 'car', event: 'rejected', cell: msg.block, detail: `已达重规划上限 → 走默认 ${priorCell}` })
      continue
    }
    const evidence = await truthPlanEvidence({ board, truth, id })
    if (evidence.exitCode !== 0 || !evidence.tokenCount) {     // 取证失败：降级走默认，不计数
      timeline.push({ t: tInt, actor: 'car', event: 'reroute-fallback', cell: priorCell, detail: `真值重规划取证失败（exit=${evidence.exitCode}）→ 走默认 ${priorCell}` })
      continue
    }
    perHalf[half] += 1
    reroutes += 1
    plans[half] = evidence
    timeline.push({
      t: tInt, actor: 'car', event: 'reroute', cell: msg.block,
      detail: `采纳 ${msg.block}（≠先验 ${priorCell}）→ 真值图重规划取证 exit=0 / ${evidence.tokenCount} token / 穿洞 ${JSON.stringify(evidence.crossing)}（取证目录 ${evidence.dir}）`,
    })
  }
  const finishT = Number(carResult.record?.metrics?.time_s) || 0
  const landT = plan.points.length > 1 ? Math.round((plan.points.length - 1) * TICK_S * 10) / 10 : 0
  timeline.push({ t: landT, actor: 'drone', event: 'land', cell: plan.land })
  timeline.push({ t: finishT, actor: 'car', event: 'finish', cell: carResult.record.status, detail: `完赛=${carResult.record?.metrics?.finished === true}` })
  timeline.sort((a, b) => a.t - b.t)

  const meta = {
    id, kind: 'drone', at: new Date().toISOString(), label,
    seed: Number.isFinite(Number(seed)) ? Number(seed) : null,
    status: carResult.record.status, carRecordId: carResult.record.id,
    prior, truth, reroutes, confirmed,
    intel: bridgeMsgs.map(m => ({ half: m.half, block: m.block, decision: m.decision })),
    planPoints: plan.points.length, parallelS: Math.round(Math.min(finishT || landT, landT || finishT) * 10) / 10,
  }
  try { appendMeta(meta) } catch { /* 台账失败不吞主结果，timeline/record 仍返回 */ }
  return { ok: true, timeline, reroutes, confirmed, record: carResult.record, scan: { takeoff: plan.takeoff, land: plan.land, points: plan.points.length }, intel, plans, id }
}

/** 无人机协同台账（自拼 run_meta.jsonl 过滤 kind==='drone'；simbench 未导出 readMeta）。 */
export function droneHistory() {
  const file = path.join(SIM_ROOT, 'artifacts', 'sim', 'run_meta.jsonl')
  try {
    const runs = fs.readFileSync(file, 'utf8').split('\n').filter(line => line.trim() !== '')
      .map(line => { try { return JSON.parse(line) } catch { return null } })
      .filter(record => record && record.kind === 'drone').slice(-50)
    return { ok: true, runs }
  } catch { return { ok: true, runs: [] } }
}
