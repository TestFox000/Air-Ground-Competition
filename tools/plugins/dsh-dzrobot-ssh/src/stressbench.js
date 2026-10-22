/**
 * stressbench — 50 图压测宿主半边（卡2，规格 `docs/sim_v2_设计规格.md` §B）。
 *
 * 职责边界（与 §D 模块契约一致）：
 *   · 随机地图：自带 PRNG `mulberry32(seed+i)` 生成，**禁用 Math.random**（seed 是验收项）；
 *   · 并发池：规格 §B.3 池闸 ≤4 —— 逐图跑 `runSimJob(body,{lock:false})`，手动 runSim 的
 *     running 409 语义原样保留（压测另持自己的 stressBusy 闸，见 simbench.setStressBusy）；
 *   · 两级判据：①生成器静态自检（§A.2，不过重掷至多 3 次）②plan_route --check（由
 *     runSimJob 内部完成，不过则回 status='plan-failed' 的 422 记录，照实进台账/分母）；
 *   · 产物只落 `artifacts/sim/stress/<batchId>/`（§B.8），台账复用 run_meta.jsonl。
 *
 * @module src/stressbench.js
 */

import fs from 'node:fs'
import path from 'node:path'
import {
  appendMeta, CELL_RE, checkPlan, COL_LETTERS, fieldSpec, normalizeCells,
  plannerEngineChoices, runSimJob, setStressBusy, SIM_ROOT,
} from './simbench.js'

/** 压测产物根（§B.8：地图/统计只落这里，config/**、src/dzbrain/** 零写入）。 */
const STRESS_DIR = path.join(SIM_ROOT, 'artifacts', 'sim', 'stress')

/** 规格 §B.1：雷 8 / 障 8 / 靶 2（相邻对）/ 通 2（K 列每半场 1）。 */
const DRAW_COUNT = { mines: 8, obstacles: 8, targets: 2, bridges: 2 }
/** 合同/规格硬边界：count 1..200；并发池规格 §B.3 ≤4（>4 收益递减且同毫秒撞 id 概率上升）。 */
const LIMITS = { count: [1, 200], concurrency: [1, 4] }

const clampInt = (raw, [min, max], fallback) => {
  const n = Number(raw)
  return Number.isInteger(n) ? Math.min(max, Math.max(min, n)) : fallback
}

/**
 * mulberry32：32 位确定性 PRNG（§B.1 明令禁用 Math.random —— 平台随机不可复现）。
 * ponytail: 只求"同 seed ⇒ 同字节流"，不求密码学强度；换 PCG 也行但现验收用不到。
 */
function mulberry32(seed) {
  let a = seed >>> 0
  return function next() {
    a = (a + 0x6d2b79f5) >>> 0
    let t = a
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

/** 从池中不放回抽 n 个（顺序消耗 rng ⇒ 同 seed 逐字节可复现）。 */
function take(rng, pool, n) {
  const out = []
  for (let k = 0; k < n && pool.length > 0; k += 1) {
    out.push(pool.splice(Math.floor(rng() * pool.length), 1)[0])
  }
  return out
}

/** 同半场竖向相邻对候选（行 6→7 跨隔板两侧，禁；靶不能骑缝在隔板上 —— §A.2）。 */
function verticalPairs(cells) {
  const set = new Set(cells)
  const out = []
  for (const cell of cells) {
    const row = Number(cell.slice(1))
    const next = `${cell[0]}${row + 1}`
    if (!set.has(next)) continue
    if ((row >= 1 && row <= 5) || (row >= 7 && row <= 10)) out.push([cell, next])
  }
  return out
}

/** 同行横向相邻对候选（同 ⇒ 行天然同侧）。 */
function horizontalPairs(cells) {
  const set = new Set(cells)
  const out = []
  for (const cell of cells) {
    const idx = COL_LETTERS.indexOf(cell[0])
    const next = idx >= 0 && idx + 1 < COL_LETTERS.length ? `${COL_LETTERS[idx + 1]}${cell.slice(1)}` : ''
    if (next && set.has(next)) out.push([cell, next])
  }
  return out
}

/**
 * 静态自检（§B.2① = §A.2 落格规则 + 数量口径）。返回 {ok, error}。
 * 权威判据仍是 plan_route --check（§B.2②），这里只拦"生成器自己写坏"的图。
 */
function checkField(field, forbidden) {
  const classes = ['mines', 'obstacles', 'targets', 'bridges']
  const seen = new Map()
  const fail = error => ({ ok: false, error })
  for (const key of classes) {
    const cells = field?.[key]
    if (!Array.isArray(cells) || cells.length !== DRAW_COUNT[key]) {
      return fail(`${key} 数量 ${Array.isArray(cells) ? cells.length : '非数组'} ≠ ${DRAW_COUNT[key]}`)
    }
    for (const cell of cells) {
      if (!CELL_RE.test(cell)) return fail(`${key} 含非法格名 ${cell}`)
      if (forbidden.has(cell)) return fail(`${key} 落到禁落格 ${cell}（起/终/无人机点）`)
      if (seen.has(cell)) return fail(`四类互斥被破坏：${cell} 同属 ${seen.get(cell)} 与 ${key}`)
      seen.set(cell, key)
    }
  }
  // 通：K 列 + 上半场（1~6）/下半场（7~11）各 1
  const halves = field.bridges.map(cell => (Number(cell.slice(1)) <= 6 ? 'upper' : 'lower'))
  if (!field.bridges.every(cell => cell[0] === 'K') || halves[0] === halves[1]) {
    return fail(`通须 K 列上/下半场各 1（得 ${field.bridges.join(',')}）`)
  }
  // 靶：恰 2 格横/竖相邻且同侧（不骑缝在隔板行 6/7 之间）
  const [a, b] = field.targets
  const adjacent = a[0] === b[0]
    ? Math.abs(Number(a.slice(1)) - Number(b.slice(1))) === 1
      && !((Number(a.slice(1)) === 6 && Number(b.slice(1)) === 7)
        || (Number(a.slice(1)) === 7 && Number(b.slice(1)) === 6))
    : COL_LETTERS.indexOf(a[0]) >= 0 && COL_LETTERS.indexOf(b[0]) >= 0
      && a.slice(1) === b.slice(1) && Math.abs(COL_LETTERS.indexOf(a[0]) - COL_LETTERS.indexOf(b[0])) === 1
  if (!adjacent) return fail(`靶须横/竖相邻且同侧（得 ${a}+${b}）`)
  return { ok: true, error: '' }
}

/** 抽一张图的四类点位（一次尝试；rng 顺序消耗由调用方跨重掷延续 —— 同 seed 仍确定）。 */
function drawOnce(rng, forbidden) {
  // 可落格池：先排禁落格（§B.1）；K 列整列是桥、R009 禁列内行驶 ⇒ 非穿洞行的 K 格
  // 不可压不可停，雷/障/靶一律不落 K（通口才配 K 格）。理由：否则该图必 plan-failed，
  // 属结构性不可行而非"难度"，会把完赛率污染成与 seed 无关的常数。
  const land = []
  for (const col of COL_LETTERS) {
    for (let row = 1; row <= 11; row += 1) {
      const cell = `${col}${row}`
      if (!forbidden.has(cell) && cell[0] !== 'K') land.push(cell)
    }
  }
  const mines = take(rng, land, DRAW_COUNT.mines)
  const obstacles = take(rng, land, DRAW_COUNT.obstacles)
  const used = new Set([...mines, ...obstacles])
  const rest = land.filter(cell => !used.has(cell))

  // 靶：横/竖各半概率（§B.1），再从合法相邻对里等概率抽一对
  const pairs = (rng() < 0.5 ? horizontalPairs(rest) : verticalPairs(rest))
  const targets = pairs.length > 0 ? pairs[Math.floor(rng() * pairs.length)] : null

  // 通：K 列上/下半场各抽 1（§B.1；上=行 1~6，下=行 7~11，与 validate.py 行段口径一致）
  const kUpper = []; const kLower = []
  for (let row = 1; row <= 11; row += 1) (row <= 6 ? kUpper : kLower).push(`K${row}`)
  const bridges = [kUpper[Math.floor(rng() * kUpper.length)], kLower[Math.floor(rng() * kLower.length)]]

  if (!targets) return { mines, obstacles, targets: [], bridges }
  return { mines, obstacles, targets: [...targets], bridges }
}

/**
 * 确定性生成第 index 张图（§B.1）→ `{ok, field}`；纯函数（除只读 fieldSpec 取禁落格真源）。
 * 静态自检不过 ⇒ 用同一条 rng 流重掷，至多 3 次重掷（4 次尝试内消耗仍确定 ⇒ 可复现）。
 */
export function generateMap({ seed = 0, index = 0 } = {}) {
  const spec = fieldSpec()                      // 真源：禁落格取 start/end + map 注释 N8/M2
  const forbidden = new Set()
  for (const cell of [spec.start, spec.end, spec.drone?.takeoff, spec.drone?.land]) {
    if (cell && CELL_RE.test(cell)) forbidden.add(cell)
  }
  const s = Number.isInteger(seed) ? seed : 0
  const i = Number.isInteger(index) ? index : 0
  const rng = mulberry32((s + i) >>> 0)         // 第 i 图 = mulberry32(seed+i)
  let field = null
  let error = '静态自检未跑'
  for (let attempt = 0; attempt < 4; attempt += 1) {
    field = drawOnce(rng, forbidden)
    for (const key of Object.keys(DRAW_COUNT)) field[key] = normalizeCells(field[key])
    const verdict = checkField(field, forbidden)
    if (verdict.ok) return { ok: true, field, attempts: attempt + 1 }
    error = verdict.error
  }
  return { ok: false, field, error: `静态自检 4 次未过：${error}` }
}

/**
 * 聚合五项统计（§B.6 公式，**分母与缺失口径全部显式**）：
 *   · finishedRate = count(finished) / n（n = 记录总数 = 本批图数）；
 *   · avgFinishS   = Σ time_s(finished) / count(finished 且有 time_s) —— 失败图不计入；
 *   · avgPlanMs    = Σ plan.ms / count(有 plan.ms)；
 *   · avgWallS     = Σ seconds  / count(有 seconds)；
 *   · avgLengthRatio = Σ length_ratio / count(有值)；
 *   缺失/未进该均值的 run 记在 `missing`（= n − counts.<项>：无该指标，或平均完赛时间里的
 *   未完赛图），不参与均值；`counts` 即各均值的分母原样。
 *   status 分布照实给：实现里 plan 校验失败的原词是 `plan-failed`（simbench.js 原样，
 *   规格 §B 记的 `plan_failed` 是同一语义的下划线写法 —— sim.html 不消费该字符串）。
 */
export function stressStats(records) {
  const list = Array.isArray(records) ? records : []
  const n = list.length
  const statusCounts = {}
  const sum = { finishS: 0, planMs: 0, wallS: 0, ratio: 0 }
  const cnt = { finishS: 0, planMs: 0, wallS: 0, ratio: 0 }
  let finished = 0
  let planned = 0
  let planFailed = 0
  let genUnsolved = 0
  const tokens = []
  const engines = {}
  const failReasons = {}
  for (const record of list) {
    const status = String(record?.status ?? 'unknown')
    statusCounts[status] = (statusCounts[status] ?? 0) + 1
    if (status === 'planned') planned += 1
    if (status === 'plan-failed') planFailed += 1
    if (status === 'generator-unsolvable') genUnsolved += 1
    if (record?.metrics?.finished === true) {
      finished += 1
      if (Number.isFinite(record.metrics.time_s)) { sum.finishS += record.metrics.time_s; cnt.finishS += 1 }
    }
    if (Number.isFinite(record?.plan?.ms)) { sum.planMs += record.plan.ms; cnt.planMs += 1 }
    if (Number.isFinite(record?.plan?.tokenCount) && record.plan.tokenCount > 0) tokens.push(record.plan.tokenCount)
    if (record?.plan?.engine) engines[record.plan.engine] = (engines[record.plan.engine] ?? 0) + 1
    // 失败主因：plan-failed / generator-unsolvable / invalid / error 的末行（页面顶部「失败主因」同源）
    if (status !== 'planned' && status !== 'done') {
      const raw = String(record?.plan?.stderrTail || record?.error || '')
      const last = raw.split('\n').filter(line => line.trim() !== '').pop() || ''
      const reason = last.replace(/^[\s·•-]+/, '').trim().slice(0, 120)
      if (reason) failReasons[reason] = (failReasons[reason] ?? 0) + 1
    }
    if (Number.isFinite(record?.seconds)) { sum.wallS += record.seconds; cnt.wallS += 1 }
    if (Number.isFinite(record?.metrics?.length_ratio)) { sum.ratio += record.metrics.length_ratio; cnt.ratio += 1 }
  }
  const avg = (value, count, digits) => (count > 0 ? Math.round((value / count) * (10 ** digits)) / (10 ** digits) : null)
  const tokenStats = tokens.length > 0
    ? { avg: Math.round((tokens.reduce((a, b) => a + b, 0) / tokens.length) * 10) / 10,
        min: Math.min(...tokens), max: Math.max(...tokens) }
    : { avg: null, min: null, max: null }
  const topReasons = Object.entries(failReasons).sort((a, b) => b[1] - a[1])
    .slice(0, 5).map(([reason, count]) => ({ reason, count }))
  return {
    n,
    statusCounts,
    finishedRate: n > 0 ? Math.round((finished / n) * 10000) / 10000 : null,
    avgFinishS: avg(sum.finishS, cnt.finishS, 2),
    avgPlanMs: avg(sum.planMs, cnt.planMs, 1),
    avgWallS: avg(sum.wallS, cnt.wallS, 2),
    avgLengthRatio: avg(sum.ratio, cnt.ratio, 4),
    // 规划压测口径（跟踪压测下 ok=0 属正常：track 没有 planned 状态）
    plan: {
      ok: planned,
      fail: planFailed,
      okRate: n > 0 ? Math.round((planned / n) * 10000) / 10000 : null,
      genUnsolved,
      avgMs: avg(sum.planMs, cnt.planMs, 1),
      tokens: tokenStats,
      engines,
      topReasons,
    },
    // 分母显式（§B.6：只给均值不给分母，50 张挂 20 张也看不出来）
    counts: { total: n, finished, finishS: cnt.finishS, planMs: cnt.planMs, wallS: cnt.wallS,
              lengthRatio: cnt.ratio, planned, planFailed },
    missing: { time_s: n - cnt.finishS, planMs: n - cnt.planMs, wallS: n - cnt.wallS, lengthRatio: n - cnt.ratio },
  }
}

/** 批次状态（供 GET /sim/stress/status）。 */
export function stressStatus() {
  return { running: state.running, batchId: state.batchId, done: state.done, total: state.total }
}

/** 批次结果回读（供 GET /sim/stress/result?id=）：summary.json 里存 {batch, stats, records}。 */
export function stressResult(id) {
  const clean = String(id ?? '')
  // label 来自调用方（cleanLabel 允许下划线）⇒ batchId 可能含多个下划线，不能只认一个
  if (!/^[0-9]+_[0-9A-Za-z_一-龥-]+$/.test(clean)) return { ok: false, code: 404, error: '非法 batchId' }
  try {
    const summary = JSON.parse(fs.readFileSync(path.join(STRESS_DIR, clean, 'summary.json'), 'utf8'))
    return { ok: true, batch: summary.batch, stats: summary.stats, records: summary.records }
  } catch (error) {
    return { ok: false, code: 404, error: `读不到压测结果 ${clean}（${error?.message ?? error}）` }
  }
}

/** 模块级批次状态（仅单进程内有意义：压测与手动单跑同宿主进程）。 */
const state = { running: false, batchId: null, done: 0, total: 0 }

/**
 * 跑一批压测（§B）→ `{ok, batchId, records, stats}`；批次忙 → `{ok:false, code:409}`。
 *   · `background=true`：同步段（参数校验 + 50 图生成 + 状态置位）跑完立即返回
 *     `{ok:true, batchId, total}`，整批在后台继续 —— 供 index.js 发规格 D 表的 202；
 *     默认（false）仍 await 全批返回完整结果，合同签名不变。
 *   · 仿真 seed = `(seed*1000+i) mod 2^31`（§B.7，与地图生成同源可复现）。
 */
export async function runStress({
  count = 64, seed, concurrency = 4, label = 'stress', background = false,
  mode = 'track', engine = null, resample = false, maxAttempts = 8,
} = {}) {
  if (state.running) return { ok: false, code: 409, error: '压测批次正在进行，稍候再试' }
  if (!fs.existsSync(path.join(SIM_ROOT, 'src', 'dzbrain'))) {
    return { ok: false, code: 503, error: '工作区不可用（' + SIM_ROOT + ' 下没有 src/dzbrain）' }
  }
  const runMode = mode === 'plan' ? 'plan' : 'track'
  const total = clampInt(count, LIMITS.count, 64)
  const pool = clampInt(concurrency, LIMITS.concurrency, 4)
  const cleanLabel = String(label ?? '').replace(/[^\w一-龥-]/g, '').slice(0, 40) || 'stress'
  const parsedSeed = Number(seed)
  const useSeed = Number.isInteger(parsedSeed) && parsedSeed >= 0 ? parsedSeed : Date.now() % 2147483647
  const engineWanted = engine == null || String(engine).trim() === '' ? null : String(engine).trim()
  if (engineWanted && !plannerEngineChoices().includes(engineWanted)) {
    return { ok: false, code: 400,
      error: '不认识的规划引擎：' + engineWanted + '（plan_route.py 只认 ' + plannerEngineChoices().join('/') + '）' }
  }
  // 补图只给「跟踪压测」用：规划压测若也补图，plan-failed 会被选择偏差吃掉，成功率永远≈100%。
  const resampleOn = runMode === 'track' && resample === true
  const maxTry = resampleOn ? clampInt(maxAttempts, [1, 20], 8) : 1
  const batchId = String(Date.now()) + '_' + cleanLabel

  state.running = true
  state.batchId = batchId
  state.done = 0
  state.total = total
  setStressBusy(true)                       // 压测期间手动单跑 409（§B.4，文案见 runSim）

  const job = (async () => {
    const records = new Array(total)
    let cursor = 0
    const runOne = async seq => {
      const simSeed = (useSeed * 1000 + seq) % 2147483648     // §B.7 mod 2^31
      // §B.5 防同毫秒互踩 + §B.8 产物只落 stress/<batchId>/：outDir = artifacts/sim/<runId>，
      // 所以 runId = 'stress/<batchId>/<规格字面 ts_batchId_seq_label>' —— 两个口径同时满足。
      const runId = 'stress/' + batchId + '/' + Date.now() + '_' + batchId + '_' + seq + '_' + cleanLabel
      let last = null
      for (let attempt = 0; attempt < maxTry; attempt += 1) {
        // attempt 0 = 老口径 mulberry32(seed+seq)；重试用固定大步长偏移，同 seed 仍可复现。
        const map = generateMap({ seed: (useSeed + attempt * 1000003) >>> 0, index: seq })
        if (!map.ok) { last = { kind: 'invalid', map }; continue }

        // 规划压测：每张图只跑一次 plan_route，成败即测量结果（不进入仿真）。
        if (runMode === 'plan') {
          const check = await checkPlan({ field: map.field, engine: engineWanted })
          const record = {
            id: runId, at: new Date().toISOString(), label: cleanLabel,
            batchId, seq, seed: simSeed, mode: 'plan',
            genAttempts: attempt + 1, planRejects: 0,
            field: map.field,
            status: check.ok ? 'planned' : 'plan-failed',
            plan: {
              exitCode: check.exitCode, timedOut: check.timedOut,
              engine: check.engine, engineRequested: check.engineRequested,
              engineSource: check.engineSource, ms: check.ms,
              tokenCount: check.tokens.length,
              crossing: check.plan?.crossing ?? null,
              polyline_m: check.plan?.polylength_m ?? null,
              reverseSegments: check.plan?.reverse_segments ?? null,
              checkExit: check.check?.exit ?? null,
              failures: check.failures,
              stderrTail: check.stderrTail,
            },
            ...(check.ok ? { route: check.tokens } : {}),
          }
          // 规划压测没有仿真产物，不进 run_meta.jsonl（否则 64 条无轨迹记录淹没「运行历史」）；
          // 批次级审计看 artifacts/sim/stress/<batchId>/summary.json（含全部 records）。
          return record
        }

        // 跟踪压测（补图时先轻量预规划；预规划不过 => 换下一张图，不浪费整趟仿真）
        let planRejects = 0
        if (resampleOn) {
          const pre = await checkPlan({ field: map.field, engine: engineWanted })
          if (!pre.ok) { planRejects += 1; last = { kind: 'plan', pre, map }; continue }
        }
        try {
          const body = { label: cleanLabel, seed: simSeed, field: map.field }
          if (engineWanted) body.engine = engineWanted
          const result = await runSimJob(body, { lock: false, runId, batch: { batchId, seq } })
          const record = result.record ?? {
            id: runId, batchId, seq, seed: simSeed, status: 'error', error: result.error ?? 'unknown',
          }
          record.mode = 'track'
          record.genAttempts = attempt + 1
          record.planRejects = planRejects
          return record
        } catch (error) {
          const record = {
            id: runId, batchId, seq, seed: simSeed, status: 'error',
            mode: 'track', genAttempts: attempt + 1, planRejects,
            error: String(error?.message ?? error),
          }
          appendMeta(record)
          return record
        }
      }
      // 生成器/预筛耗尽：单列 generator-unsolvable（不混进规划器的 plan-failed 失败率）
      const plan = last?.kind === 'plan' ? last.pre : null
      const record = {
        id: runId, at: new Date().toISOString(), label: cleanLabel, batchId, seq, seed: simSeed,
        mode: runMode, genAttempts: maxTry, planRejects: plan ? maxTry : 0,
        status: plan ? 'generator-unsolvable' : 'invalid',
        field: last?.map?.field ?? null,
        error: plan ? (plan.error || '规划器无解') : (last?.map?.error || '生成器静态自检未过'),
        ...(plan ? {
          plan: {
            exitCode: plan.exitCode, timedOut: plan.timedOut,
            engine: plan.engine, engineRequested: plan.engineRequested,
            engineSource: plan.engineSource, ms: plan.ms,
            tokenCount: plan.tokens.length,
            failures: plan.failures, stderrTail: plan.stderrTail,
          },
        } : {}),
      }
      appendMeta(record)
      return record
    }
    const workers = []
    for (let w = 0; w < Math.min(pool, total); w += 1) {
      workers.push((async () => {
        while (cursor < total) {
          const seq = cursor
          cursor += 1
          records[seq] = await runOne(seq)   // 按 seq 定位 ⇒ 完成顺序乱序也保 records 有序
          state.done += 1
        }
      })())
    }
    await Promise.all(workers)

    const stats = stressStats(records)
    const summary = {
      batch: { batchId, seed: useSeed, count: total, concurrency: pool, label: cleanLabel,
               mode: runMode, engine: engineWanted, resample: resampleOn, maxAttempts: maxTry,
               at: new Date().toISOString() },
      stats,
      records,
    }
    fs.mkdirSync(path.join(STRESS_DIR, batchId), { recursive: true })
    fs.writeFileSync(path.join(STRESS_DIR, batchId, 'summary.json'), JSON.stringify(summary, null, 2) + '\n', 'utf8')
    return { ok: true, batchId, records, stats }
  })()

  const settle = job.catch(error => ({ ok: false, code: 500, error: String(error?.message ?? error) }))
    .finally(() => {
      state.running = false
      state.batchId = null
      setStressBusy(false)
    })

  if (background === true) {
    void settle
    return { ok: true, batchId, total }
  }
  return settle
}
