/**
 * scoutsim.js — 仿真台「步进侦察 / 十批步进成果」宿主半边（T28/T29 + T34）。
 *
 * 这两块都只做**只读消费**，不写 config、不连车：
 *   1. runScoutSim：调 `tools/scout_sim.py --json --no-check`（T28 机端侦察模型），
 *      产出帧位/布局 v2 序列/心跳计划/覆盖率对照；页面渲染布局时间线（version/known/target）。
 *   2. stepLoopResults / stepLoopArtifact：只读 T34 的十批步进产物目录
 *      （默认 `_tmp_session/step_loop/batch10`，可用 dir 指定其下其它目录），
 *      返回 metrics/batches/analysis 与可下载的 gantt/traj 产物。
 *
 * 边界：
 *   - scout_sim.py 用仓库 tools/ 下的现有实现（不复制、不内联），参数白名单；
 *   - step-loop 目录必须在 `<root>/_tmp_session/step_loop` 之内（拒绝 ../ 越界）；
 *   - mapPath 必须在工作区内（只读路径，不允许指到系统目录）。
 */
import fs from 'node:fs'
import path from 'node:path'
import { runChild, SIM_ROOT } from './simbench.js'

const SCOUT_CLI = path.join(SIM_ROOT, 'tools', 'scout_sim.py')
const STEP_ROOT = path.join(SIM_ROOT, '_tmp_session', 'step_loop')

/** p 是否在 root 之内（root 本身不算；大小写按平台归一）。 */
function within(root, p) {
  const rel = path.relative(path.resolve(root), path.resolve(p))
  return rel !== '' && !rel.startsWith('..') && !path.isAbsolute(rel)
}

function numOrNull(value) {
  if (value === undefined || value === null || value === '') return null
  const n = Number(value)
  return Number.isFinite(n) ? n : null
}

/** 车格名白名单（与地图 A~N × 1~11 一致；非法直接 400，不传给 python 猜）。 */
const CAR_BLOCK_RE = /^[A-N](?:[1-9]|1[01])$/

/**
 * 跑一次机端侦察仿真。
 * @param {{startT?:number, frameHz?:number, heartbeatS?:number, carBlock?:string, mapPath?:string}} body
 * @returns {Promise<{ok:boolean, [key:string]:any}>}
 */
export async function runScoutSim(body = {}) {
  if (!fs.existsSync(SCOUT_CLI)) {
    return { ok: false, code: 404, error: `scout_sim.py 不存在：${SCOUT_CLI}` }
  }
  const argv = [SCOUT_CLI, '--json', '--no-check']
  const startT = numOrNull(body.startT)
  if (startT !== null) argv.push('--start-t', String(startT))
  const frameHz = numOrNull(body.frameHz)
  if (frameHz !== null && frameHz > 0) argv.push('--frame-hz', String(frameHz))
  const heartbeatS = numOrNull(body.heartbeatS)
  if (heartbeatS !== null && heartbeatS > 0) argv.push('--heartbeat-s', String(heartbeatS))
  if (body.carBlock) {
    const block = String(body.carBlock).trim().toUpperCase()
    if (!CAR_BLOCK_RE.test(block)) return { ok: false, code: 400, error: `carBlock 非法：${body.carBlock}` }
    argv.push('--car-block', block)
  }
  if (body.mapPath) {
    const abs = path.isAbsolute(String(body.mapPath))
      ? path.resolve(String(body.mapPath))
      : path.resolve(SIM_ROOT, String(body.mapPath))
    if (!within(SIM_ROOT, abs)) return { ok: false, code: 400, error: 'mapPath 必须位于工作区内' }
    if (!fs.existsSync(abs)) return { ok: false, code: 400, error: `mapPath 不存在：${abs}` }
    argv.push('--map', abs)
  }
  const started = Date.now()
  const { exitCode, stdout, stderr, timedOut } = await runChild('python', argv, {
    cwd: SIM_ROOT, timeoutMs: 120000,
  })
  const elapsedMs = Date.now() - started
  if (timedOut) return { ok: false, code: 504, error: '侦察仿真超时（>120 s）' }
  if (exitCode !== 0) {
    return { ok: false, code: 500, error: `scout_sim exit=${exitCode}`, stderrTail: String(stderr).slice(-6000) }
  }
  let data
  try {
    data = JSON.parse(stdout)
  } catch (error) {
    return { ok: false, code: 500, error: 'scout_sim 输出不是合法 JSON', stdoutTail: String(stdout).slice(0, 2000) }
  }
  const layouts = Array.isArray(data.layouts) ? data.layouts.map(item => ({
    version: item.version,
    ts: item.ts,
    knownCount: Array.isArray(item.known) ? item.known.length : 0,
    targetCell: item.target_cell || '',
    mineCount: Array.isArray(item.mines) ? item.mines.length : 0,
    obstacleCount: Array.isArray(item.obstacles) ? item.obstacles.length : 0,
    targetCount: Array.isArray(item.targets) ? item.targets.length : 0,
  })) : []
  return {
    ok: true,
    elapsedMs,
    truth: data.truth,
    stats: data.stats,
    comparison: data.comparison,
    layouts,
    knownCount: data.known_count,
    heartbeatCount: data.heartbeat_count,
    carBlock: data.car_block,
    source: SCOUT_CLI,
  }
}

/** 逐行 CSV → 对象数组（T34 的 batches.csv 是简单逗号表，无引号嵌套）。 */
function parseCsv(text) {
  const lines = String(text || '').split(/\r?\n/).filter(line => line.trim() !== '')
  if (lines.length < 2) return []
  const head = lines[0].split(',').map(item => item.trim())
  return lines.slice(1).map(line => {
    const cells = line.split(',')
    const row = {}
    head.forEach((key, index) => { row[key] = (cells[index] ?? '').trim() })
    return row
  })
}

/** 把 dir 解析到 `_tmp_session/step_loop` 之内；非法返回 {error}。 */
function resolveStepDir(dir) {
  const raw = (dir === undefined || dir === null || String(dir).trim() === '') ? 'batch10' : String(dir)
  const abs = path.isAbsolute(raw) ? path.resolve(raw) : path.resolve(STEP_ROOT, raw)
  if (!within(STEP_ROOT, abs)) return { error: 'dir 必须位于 _tmp_session/step_loop 下' }
  return { abs }
}

/**
 * 读 T34 十批步进的产物（只读）。
 * @param {{dir?:string}} [opts]
 */
export function stepLoopResults({ dir } = {}) {
  const resolved = resolveStepDir(dir)
  if (resolved.error) return { ok: false, code: 400, error: resolved.error }
  const base = resolved.abs
  let metrics
  try {
    metrics = JSON.parse(fs.readFileSync(path.join(base, 'metrics.json'), 'utf8'))
  } catch (error) {
    return { ok: false, code: 404, error: `未找到 ${path.relative(SIM_ROOT, base)}/metrics.json` }
  }
  let batches = []
  try { batches = parseCsv(fs.readFileSync(path.join(base, 'batches.csv'), 'utf8')) } catch { /* 可选 */ }
  let analysis = ''
  try { analysis = fs.readFileSync(path.join(base, 'analysis.md'), 'utf8') } catch { /* 可选 */ }
  const whitelist = ['metrics.json', 'batches.csv', 'analysis.md', 'task_metrics.json',
    'gantt.svg', 'gantt.png', 'traj_batch.svg', 'traj_batch.png', 'timeline.csv',
    'full_route_check.txt', 'boundary_tests.json']
  const files = whitelist.filter(name => fs.existsSync(path.join(base, name)))
  const dirs = fs.existsSync(STEP_ROOT)
    ? fs.readdirSync(STEP_ROOT, { withFileTypes: true }).filter(item => item.isDirectory()).map(item => item.name)
    : []
  return {
    ok: true,
    dir: path.relative(SIM_ROOT, base).replace(/\\/g, '/'),
    dirs,
    metrics,
    batches,
    analysis: analysis.slice(0, 20000),
    files,
  }
}

/**
 * 解析一个可下载的 T34 产物（只允许列表内文件 + 目录内）。
 * @param {{dir?:string, name?:string}} [opts]
 * @returns {{file:string, type:string}|null}
 */
export function stepLoopArtifact({ dir, name } = {}) {
  const resolved = resolveStepDir(dir)
  if (resolved.error || !name) return null
  const safe = String(name)
  if (!/^[A-Za-z0-9_.-]+$/.test(safe)) return null
  const file = path.join(resolved.abs, safe)
  if (!within(resolved.abs, file) || !fs.existsSync(file)) return null
  const types = {
    '.svg': 'image/svg+xml; charset=utf-8',
    '.png': 'image/png',
    '.csv': 'text/csv; charset=utf-8',
    '.md': 'text/markdown; charset=utf-8',
    '.json': 'application/json; charset=utf-8',
    '.txt': 'text/plain; charset=utf-8',
  }
  return { file, type: types[path.extname(file).toLowerCase()] || 'application/octet-stream' }
}
