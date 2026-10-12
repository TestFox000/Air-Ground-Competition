/**
 * simbench — 本地仿真台（宿主半边）。
 *
 * 在 Windows 工作区里跑 dzbrain 的离线仿真（`python -m dzbrain.tools.sim_cli`），
 * 不碰小车、不 import rclpy：一次点击 = 一次仿真，省掉每次让智能体代跑的开销。
 *
 * 铁律（与车侧门禁同一口径）：
 *   · 只读 config/robot.yaml 与 route —— 参数一律走 `--set KEY=VALUE`；
 *   · 每次运行把「参数快照 + config md5 + 产物路径」写进 run_meta.jsonl，
 *     保证任何一张图都能说清"它是哪一版跑出来的"；
 *   · 输出只落 artifacts/sim/（gitignore 内），工作区其余零写入。
 *
 * @module src/simbench.js
 */

import { spawn } from 'node:child_process'
import crypto from 'node:crypto'
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

/** 工作区根 = 插件目录的上一级（dzrobot_v2/dsh-plugin-dzrobot-ssh/ → dzrobot_v2/）。 */
const PLUGIN_DIR = fileURLToPath(new URL('..', import.meta.url))

/**
 * 工作区指针文件（**部署时**写的，内容 `{"root":"<绝对路径>"}`）。
 *
 * 为什么需要它：插件被"拷贝安装"到 `~/.dsh/profiles/<profile>/node_modules/dsh-dzrobot-ssh`
 * 时，它的上一级是 `node_modules` 而不是 dzbrain 工作区 —— 那时 `PLUGIN_DIR/..` 既没有
 * `src/dzbrain` 也没有 `config/robot.yaml`，仿真会"点一下就失败"，而且失败得很难看懂。
 * 指针文件让安装副本仍能指回真正的工作区；源码树里的副本不需要它（上一级就是工作区，
 * 优先级更高，所以工作区副本永远以自己为准）。
 */
const ROOT_POINTER_FILE = 'sim-root.json'

/** 某个目录是否像一个 dzbrain 工作区（判据：`src/dzbrain` 存在）。 */
function looksLikeWorkspace(dir) {
  try {
    return typeof dir === 'string' && dir !== '' && fs.existsSync(path.join(dir, 'src', 'dzbrain'))
  } catch { return false }
}

/** 读指针文件（不存在/坏 JSON/字段缺失都返回 ''）。 */
function pointerRoot() {
  try {
    const raw = JSON.parse(fs.readFileSync(path.join(PLUGIN_DIR, ROOT_POINTER_FILE), 'utf8'))
    const root = typeof raw === 'string' ? raw : (raw?.root ?? raw?.simRoot ?? '')
    return String(root ?? '')
  } catch { return '' }
}

/**
 * 工作区根的解析顺序（**只读**，永不抛）：
 *   1. `DZROBOT_SIM_ROOT` 环境变量（部署/调试可直接压过）；
 *   2. `PLUGIN_DIR/..`（源码树：插件就在工作区里）；
 *   3. `sim-root.json` 指针（拷贝安装：指回真正的工作区）。
 * 三条都不成立时返回 `PLUGIN_DIR/..`，让 `/sim/root` 如实报 `ok:false` 而不是悄悄跑错树。
 */
function resolveSimRoot() {
  const candidates = [
    process.env.DZROBOT_SIM_ROOT ?? '',
    path.resolve(PLUGIN_DIR, '..'),
    pointerRoot(),
  ]
  for (const candidate of candidates) if (looksLikeWorkspace(candidate)) return path.resolve(candidate)
  return path.resolve(PLUGIN_DIR, '..')
}

export const SIM_ROOT = resolveSimRoot()

/** 解析出来的根来自哪里（台账/诊断用，便于现场回答"它到底跑的是哪棵树"）。 */
export const SIM_ROOT_SOURCE = process.env.DZROBOT_SIM_ROOT && SIM_ROOT === path.resolve(process.env.DZROBOT_SIM_ROOT)
  ? 'env'
  : (SIM_ROOT === path.resolve(PLUGIN_DIR, '..') ? 'plugin-parent' : 'pointer')

const ARTIFACT_DIR = path.join(SIM_ROOT, 'artifacts', 'sim')
const META_FILE = path.join(ARTIFACT_DIR, 'run_meta.jsonl')
const ARTIFACT_EXTS = new Set(['.svg', '.csv', '.json', '.txt'])
const ARTIFACT_MAX_BYTES = 8 * 1024 * 1024

// ── 场地（★ t25：全部**只读**；`config/**` 与 `src/dzbrain/**` 零写入）─────────
/** 现场地图真源（只读）。 */
const MAP_FILE = path.join(SIM_ROOT, 'config', 'map.yaml')
/** 现场配置真源（只读）——改点位的运行时用它的**运行目录副本**，不动这份。 */
const ROBOT_FILE = path.join(SIM_ROOT, 'config', 'robot.yaml')
/** 规划器 CLI：只读调用，`--map` 指 override 地图 ⇒ 路线随新点位重规划。 */
const PLAN_CLI = path.join(SIM_ROOT, 'tools', 'plan_route.py')
/** 场地网格（`core/geometry.py:27-30`：COL_LETTERS / MAP_COLS=14 / MAP_ROWS=11 / BLOCK_SIZE=0.35）。 */
const GRID = { cols: 14, rows: 11, blockM: 0.35, colLetters: 'ABCDEFGHIJKLMN' }
/** 页面允许编辑的三类点位（其余要素保持现场值）。 */
const FIELD_KEYS = ['mines', 'obstacles', 'bridges']
/** 无人机起/降点：map.yaml 的 t28 识图注释里写着；读不到时用这两个（蓝 + H 图标）。 */
const DRONE_FALLBACK = { takeoff: 'N8', land: 'M2' }
const CELL_RE = /^[A-N](?:[1-9]|1[01])$/
const FIELD_MAX_CELLS = 40

/** 一次只跑一个仿真：并发会互相踩 artifacts/sim 的输出。 */
let running = false

/** 允许注入的误差参数（与 sim_cli 参数一一对应）。 */
const INJECT_KEYS = {
  speed_scale_err: { flag: '--speed-scale-err', type: Number },
  yaw_drift: { flag: '--yaw-drift', type: Number },
  pos_noise: { flag: '--pos-noise', type: Number },
  latency: { flag: '--latency', type: Number },
  anchor: { flag: null, type: String }, // 'on' | 'off'
}

function md5Of(file) {
  try { return crypto.createHash('md5').update(fs.readFileSync(file)).digest('hex') } catch { return '' }
}

// ── 场地读取 / 改点位 override（★ t25；只读现场真源，产物只落运行目录）──────────

/** 去掉 YAML 标量的引号与行尾注释。 */
function unquote(raw) {
  return String(raw ?? '').replace(/\s+#.*$/, '').trim().replace(/^['"]|['"]$/g, '')
}

/** 取 `key:` 的单行标量（map.yaml 的 name/start/end/barrier_* 都是这个形状）。 */
function scalarOf(text, key) {
  const found = normalizeNewlines(text).match(new RegExp(`^[ \\t]*${key}:[ \\t]*(.*)$`, 'm'))
  return found ? unquote(found[1]) : ''
}

/** 现场 YAML 是 CRLF：解析前先把行尾归一，否则 `$` 行尾锚点/`- item` 匹配全废（t25 实测踩过）。 */
function normalizeNewlines(text) {
  return String(text ?? '').replace(/\r\n?/g, '\n')
}

/** 取 `key:` 的列表，**两种形状都认**：
 *   · flow：`mines: ["B1", "G2"]`（允许跨行 ⇒ 扫描到配对的 `]`）；
 *   · 块序列：`mission_blocks:` 之后若干 `- N10` 行（robot.yaml 的路线就是这个形状）。
 */
function listOf(rawText, key) {
  const text = normalizeNewlines(rawText)
  const at = text.search(new RegExp(`^[ \\t]*${key}:[ \\t]*(.*)$`, 'm'))
  if (at < 0) return []
  const lineEnd = text.indexOf('\n', at)
  const head = text.slice(at, lineEnd < 0 ? text.length : lineEnd)
  const inline = head.slice(head.indexOf(':') + 1)

  if (inline.includes('[')) {
    const open = text.indexOf('[', at)
    let depth = 0
    let end = -1
    let quote = ''
    for (let i = open; i < text.length; i += 1) {
      const ch = text[i]
      if (quote) { if (ch === quote) quote = ''; continue }
      if (ch === '"' || ch === "'") { quote = ch; continue }
      if (ch === '[') depth += 1
      else if (ch === ']') { depth -= 1; if (depth === 0) { end = i; break } }
    }
    if (end < 0) return []
    return text.slice(open + 1, end).split(/[\s,]+/)
      .map(value => value.replace(/^['"]|['"]$/g, '')).filter(Boolean)
  }

  const out = []
  const rest = text.slice(lineEnd < 0 ? text.length : lineEnd + 1).split('\n')
  for (const line of rest) {
    if (/^[ \t]*#/.test(line)) continue
    const item = line.match(/^[ \t]*-[ \t]*(.+?)[ \t]*$/)
    if (item) {
      out.push(item[1].replace(/\s+#.*$/, '').replace(/^['"]|['"]$/g, '').trim())
      continue
    }
    if (line.trim() === '') continue
    break                                   // 下一个键 ⇒ 块结束
  }
  return out.filter(Boolean)
}

/** 单个格名归一化（不合法 → 空串；只认 A~N 列、1~11 行）。 */
const normalizeCell = value => {
  const text = String(value ?? '').trim().toUpperCase()
  return CELL_RE.test(text) ? text : ''
}

/** 点位数组：大写、只留合法格名、去重、按 A→N/1→11 排序、限长。 */
export function normalizeCells(raw, { max = FIELD_MAX_CELLS } = {}) {
  const list = Array.isArray(raw) ? raw : String(raw ?? '').split(/[\s,;|]+/)
  const seen = new Set()
  for (const item of list) {
    const cell = normalizeCell(item)
    if (cell) seen.add(cell)
  }
  return [...seen].sort((a, b) => {
    const colA = a[0]; const colB = b[0]
    return colA === colB ? Number(a.slice(1)) - Number(b.slice(1)) : colA.localeCompare(colB)
  }).slice(0, max)
}

/** 只读场地（给页面画横置地图）：解析失败返回 `ok:false`，绝不抛。 */
export function fieldSpec() {
  let text = ''
  try { text = fs.readFileSync(MAP_FILE, 'utf8') } catch (error) {
    return { ok: false, root: SIM_ROOT, error: `读不到地图 ${MAP_FILE}（${error?.message ?? error}）` }
  }
  const droneLine = (text.match(/^#.*无人机点.*$/m) ?? [''])[0]
  const droneCells = (droneLine.match(/[A-N]\d{1,2}/g) ?? []).map(normalizeCell).filter(Boolean)
  const colFrom = scalarOf(text, 'barrier_col_from').toUpperCase().slice(0, 1)
  return {
    ok: true,
    root: SIM_ROOT,
    source: MAP_FILE,
    md5: md5Of(MAP_FILE),
    robot: { path: ROBOT_FILE, md5: md5Of(ROBOT_FILE) },
    grid: GRID,
    name: scalarOf(text, 'name'),
    start: normalizeCell(scalarOf(text, 'start')),
    end: normalizeCell(scalarOf(text, 'end')),
    mines: normalizeCells(listOf(text, 'mines')),
    obstacles: normalizeCells(listOf(text, 'obstacles')),
    targets: normalizeCells(listOf(text, 'targets')),
    bridges: normalizeCells(listOf(text, 'bridges')),
    barrier: {
      colFrom: /^[A-N]$/.test(colFrom) ? colFrom : 'D',
      rowHi: Number(scalarOf(text, 'barrier_row_hi')) || 6,
      rowLo: Number(scalarOf(text, 'barrier_row_lo')) || 7,
    },
    drone: {
      takeoff: droneCells[0] || DRONE_FALLBACK.takeoff,
      land: droneCells[1] || DRONE_FALLBACK.land,
      source: droneCells.length >= 2 ? 'map-comment' : 'builtin-fallback',
    },
    editable: FIELD_KEYS,
  }
}

/** 只读路线 token（给页面画参考路线）。
 *
 * ★ 文件里的形状是 `route:` 段下的 `tokens: [...]`（展平后才叫 `route.tokens`），
 *   所以这里找的是 `tokens:`，旧版 `mission_blocks:` 作兜底。
 */
export function routeTokens() {
  try {
    const text = fs.readFileSync(ROBOT_FILE, 'utf8')
    const tokens = listOf(text, 'tokens')
    return tokens.length ? tokens : listOf(text, 'mission_blocks')
  } catch { return [] }
}

/**
 * 把编辑后的点位**只替换那三行**写进地图文本（其余字节原样保留）。
 * 三类键必须本来就在 map.yaml 里：找不到就抛 —— 往文件尾部瞎追加只会写出坏地图。
 */
export function applyFieldToMapText(text, field) {
  let out = String(text ?? '')
  for (const key of FIELD_KEYS) {
    const cells = normalizeCells(field?.[key])
    if (cells.length === 0) continue
    const line = `${key}: ["${cells.join('", "')}"]`
    const pattern = new RegExp(`^([ \\t]*)${key}:[ \\t]*\\[[^\\]]*\\][ \\t]*$`, 'm')
    if (!pattern.test(out)) throw new Error(`地图里没有 ${key}: [...] 这一行，拒绝改（避免写出坏地图）`)
    out = out.replace(pattern, `$1${line}`)
  }
  return out
}

/** 调 `tools/plan_route.py`（只读）→ 重新规划出的 token。 */
async function planRoute({ configPath, mapPath, timeoutMs = 300 * 1000 }) {
  const argv = ['tools/plan_route.py', '--config', configPath, '--map', mapPath,
                '--emit', 'json', '--check', '--quiet']
  const run = await runChild('python', argv, { cwd: SIM_ROOT, timeoutMs })
  let payload = null
  const open = run.stdout.indexOf('{')
  if (open >= 0) {
    try { payload = JSON.parse(run.stdout.slice(open)) } catch { payload = null }
  }
  const plan = payload?.plans?.[0] ?? null
  return {
    exitCode: run.exitCode,
    timedOut: run.timedOut,
    tokens: Array.isArray(plan?.tokens) ? plan.tokens : [],
    plan,
    check: payload?.route_check?.[0] ?? null,
    failures: payload?.failures ?? [],
    stderrTail: String(run.stderr).split('\n').filter(line => line.trim() !== '').slice(-8).join('\n'),
    stdoutTail: String(run.stdout).split('\n').filter(line => line.trim() !== '').slice(-8).join('\n'),
  }
}

/**
 * 参数快照（用于台账，与 run 记录一起存）。
 */
function sanitizeParams(body = {}) {
  const overrides = {}
  if (body.overrides && typeof body.overrides === 'object') {
    for (const [key, raw] of Object.entries(body.overrides)) {
      // `--set` 只接受 k=v 字符串；拒绝任何含空白的值，防止注入额外参数。
      const value = String(raw)
      if (!/^[A-Za-z0-9_.\-/]+$/.test(value)) continue
      if (/^[A-Za-z0-9_.]+$/.test(key)) overrides[key] = value
    }
  }
  const inject = {}
  for (const [key, spec] of Object.entries(INJECT_KEYS)) {
    if (body.inject?.[key] === undefined || body.inject[key] === null || body.inject[key] === '') continue
    if (spec.flag === null) { if (body.inject[key] === 'on' || body.inject[key] === true) inject[key] = true; continue }
    const value = spec.type(body.inject[key])
    if (Number.isFinite(value)) inject[key] = value
  }
  const label = String(body.label ?? 'run').replace(/[^\w\u4e00-\u9fa5\-]/g, '').slice(0, 40) || 'run'
  const maxSteps = Number.isInteger(body.maxSteps) && body.maxSteps > 0 ? body.maxSteps : 0
  // ★ t25：seed 留空 = sim_cli 的 SimConfig 默认（不传 `--seed`，与改动前逐字一致）。
  const seed = Number.isInteger(body.seed) && body.seed >= 0 ? body.seed : null
  // ★ t25：三类点位若要改，就在运行目录里生成 override（config/** 零写入）。
  const field = {}
  let fieldTouched = false
  if (body.field && typeof body.field === 'object') {
    for (const key of FIELD_KEYS) {
      if (body.field[key] === undefined || body.field[key] === null) continue
      const cells = normalizeCells(body.field[key])
      if (cells.length === 0) continue
      field[key] = cells
      fieldTouched = true
    }
  }
  return { overrides, inject, label, maxSteps, seed, field: fieldTouched ? field : null }
}

/** 把参数拼成 sim_cli 的 argv（数组，不经 shell）。 */
function buildSimArgv({ overrides, inject, maxSteps, seed, id, configPath, routePath }) {
  const argv = ['-m', 'dzbrain.tools.sim_cli']
  for (const [key, value] of Object.entries(overrides)) argv.push('--set', `${key}=${value}`)
  if (inject.speed_scale_err !== undefined) argv.push('--speed-scale-err', String(inject.speed_scale_err))
  if (inject.yaw_drift !== undefined) argv.push('--yaw-drift', String(inject.yaw_drift))
  if (inject.pos_noise !== undefined) argv.push('--pos-noise', String(inject.pos_noise))
  if (inject.latency !== undefined) argv.push('--latency', String(inject.latency))
  if (inject.anchor === true) argv.push('--anchor')
  if (maxSteps > 0) argv.push('--max-steps', String(maxSteps))
  if (Number.isInteger(seed)) argv.push('--seed', String(seed))
  // 默认与改动前逐字一致：不给 --config/--route（sim_cli 自己取工作区 config/robot.yaml）。
  if (configPath) argv.push('--config', configPath)
  if (routePath) argv.push('--route', routePath)
  argv.push('--out-dir', path.join(ARTIFACT_DIR, id))
  return argv
}

/**
 * 从 sim 的 stdout 里抓 zh/en 双语的指标数字。
 *
 * ★ 这两条是**照着 sim_cli 的真实排版**钉的（t21 实测：不这么写会抓错行）：
 *   · 横向 RMS：`sim_cli` 会打好几行 —— `横向RMS(真值)`、`横向RMS(信念)`、
 *     `信念↔真值RMS`、`横向RMS 直段/倒车段/拐角`。旧正则 `(?:横向RMS|RMS)[：:\s]*(数字)`
 *     因为 `(真值)` 挡在中间而**匹配不到真值那行**，最后抓到 `信念↔真值RMS` 的 6 位小数 ——
 *     指标表里那个"横向RMS"不是真值横向误差（错的数比没有数更糟）。
 *   · 越界：标签是 `越界拍数  0`（"越界"后面还有"拍数"），旧正则要求"越界"后直接跟数字 ⇒ 恒不匹配。
 */
const METRIC_LABELS = [
  ['finished', /(?:跑完全程|finished)[：:\s]*([是是否]|true|false|yes|no)/i],
  ['time_s', /(?:总时长|total time)[：:\s]*([0-9]+(?:\.[0-9]+)?)/i],
  ['rms_m', /横向\s*RMS(?:\s*[（(]\s*真值\s*[)）])?[：:\s]*([0-9]+(?:\.[0-9]+)?)/],
  ['out_of_bounds', /(?:越界(?:拍数)?|out.?of.?bounds)[：:\s]*([0-9]+)/i],
  ['length_ratio', /(?:路径长度比|length ratio)[：:\s]*([0-9]+(?:\.[0-9]+)?)/i],
]
function parseMetrics(text) {
  const metrics = {}
  for (const [key, pattern] of METRIC_LABELS) {
    const match = text.match(pattern)
    if (match) {
      const value = match[1]
      if (key === 'finished') metrics.finished = /是|true|yes/i.test(value)
      else {
        const number = Number(value)
        if (Number.isFinite(number)) metrics[key] = number
      }
    }
  }
  return metrics
}

/** 跑一个子进程，返回 {exitCode, stdout, stderr, timedOut}。 */
function runChild(command, argv, { cwd, timeoutMs, env = {} }) {
  return new Promise(resolve => {
    const child = spawn(command, argv, {
      cwd,
      env: {
        ...process.env,
        PYTHONPATH: path.join(SIM_ROOT, 'src', 'dzbrain'),
        PYTHONIOENCODING: 'utf-8',
        PYTHONUNBUFFERED: '1',
        ...env,
      },
      windowsHide: true,
    })
    let stdout = ''
    let stderr = ''
    let timedOut = false
    child.stdout.on('data', chunk => { stdout += chunk })
    child.stderr.on('data', chunk => { stderr += chunk })
    const timer = setTimeout(() => {
      timedOut = true
      try { child.kill() } catch { /* already gone */ }
    }, timeoutMs)
    child.on('error', error => { clearTimeout(timer); resolve({ exitCode: -1, stdout, stderr: String(error?.message ?? error), timedOut }) })
    child.on('close', exitCode => {
      clearTimeout(timer)
      // Windows 上被 kill 的进程通常 settle 为 exit 1；timedOut 标记才是权威判据。
      resolve({ exitCode: exitCode ?? 1, stdout, stderr, timedOut })
    })
  })
}

function appendMeta(record) {
  fs.mkdirSync(ARTIFACT_DIR, { recursive: true })
  fs.appendFileSync(META_FILE, `${JSON.stringify(record)}\n`, 'utf8')
}

function readMeta() {
  try {
    return fs.readFileSync(META_FILE, 'utf8').split('\n').filter(line => line.trim() !== '')
      .map(line => { try { return JSON.parse(line) } catch { return null } }).filter(Boolean).slice(-50)
  } catch { return [] }
}

/** 主接口：跑一次仿真。 */
export async function runSim(body) {
  if (running) return { ok: false, code: 409, error: '上一次仿真还在跑，稍候再试' }
  if (!looksLikeWorkspace(SIM_ROOT)) {
    return {
      ok: false, code: 409,
      error: `工作区不可用（${SIM_ROOT} 下没有 src/dzbrain）：安装副本需要在插件目录放一份 `
        + `${ROOT_POINTER_FILE}（内容 {"root":"<dzrobot_v2 绝对路径>"}）`,
    }
  }
  const params = sanitizeParams(body)
  const id = `${Date.now()}_${params.label}`
  const outDir = path.join(ARTIFACT_DIR, id)
  const record = {
    id,
    at: new Date().toISOString(),
    label: params.label,
    overrides: params.overrides,
    inject: params.inject,
    maxSteps: params.maxSteps,
    seed: params.seed,
    simRoot: SIM_ROOT,
    simRootSource: SIM_ROOT_SOURCE,
    configMd5: md5Of(ROBOT_FILE),
    mapMd5: md5Of(MAP_FILE),
    status: 'running',
  }
  running = true
  try {
    const started = Date.now()
    // ---- ★ t25：改点位 → 运行目录里生成 override（地图 + 配置副本 + 重规划路线）----
    let configPath = null
    let routePath = null
    let env = {}
    if (params.field) {
      const overrideDir = path.join(outDir, 'map_override')
      fs.mkdirSync(overrideDir, { recursive: true })
      const mapText = fs.readFileSync(MAP_FILE, 'utf8')
      const overrideMap = path.join(overrideDir, 'map.yaml')
      const overrideRobot = path.join(overrideDir, 'robot.yaml')
      fs.writeFileSync(overrideMap, applyFieldToMapText(mapText, params.field), 'utf8')
      // 配置**逐字复制**（只读真源的副本）：`load_field_spec(cfg_source=…)` 会优先取同目录的
      // map.yaml，所以这一对文件放在同一目录就是"这次跑的是哪张图"的完整证据。
      fs.copyFileSync(ROBOT_FILE, overrideRobot)
      const planned = await planRoute({ configPath: overrideRobot, mapPath: overrideMap })
      record.field = params.field
      record.override = { dir: overrideDir, map: overrideMap, robot: overrideRobot }
      record.plan = {
        exitCode: planned.exitCode, timedOut: planned.timedOut,
        tokenCount: planned.tokens.length,
        crossing: planned.plan?.crossing ?? null,
        polyline_m: planned.plan?.polylength_m ?? null,
        reverseSegments: planned.plan?.reverse_segments ?? null,
        checkExit: planned.check?.exit ?? null,
        failures: planned.failures,
        stderrTail: planned.stderrTail,
      }
      if (planned.exitCode !== 0 || planned.tokens.length === 0) {
        record.status = 'plan-failed'
        record.stderrTail = planned.stderrTail
        appendMeta(record)
        return {
          ok: false, code: 422, record,
          error: `改点位后规划/校验未过（plan_route.py exit=${planned.exitCode}）—— 路线没换，仿真未跑。`,
          planTail: planned.stdoutTail || planned.stderrTail,
        }
      }
      routePath = path.join(overrideDir, 'route.tokens.txt')
      fs.writeFileSync(routePath, `${planned.tokens.join('\n')}\n`, 'utf8')
      record.route = planned.tokens
      record.routeSource = 'planned(map_override)'
      configPath = overrideRobot
      // `ROBOT_MAP`：让任何按环境变量找地图的路径（load_field_spec）也用这张 override 图。
      env = { ROBOT_MAP: overrideMap }
    } else {
      record.route = routeTokens()
      record.routeSource = 'config/robot.yaml'
    }
    const { exitCode, stdout, stderr, timedOut } = await runChild('python',
      buildSimArgv({ ...params, id, configPath, routePath }),
      { cwd: SIM_ROOT, timeoutMs: 180 * 1000, env })
    record.configUsed = configPath ?? ROBOT_FILE
    const files = fs.existsSync(outDir)
      ? fs.readdirSync(outDir).filter(name => name !== 'run_meta.jsonl' && name !== 'map_override')
        .map(name => ({ name, size: fs.statSync(path.join(outDir, name)).size ?? 0 }))
      : []
    record.status = timedOut ? 'timeout' : (exitCode === 0 ? 'done' : `exit:${exitCode}`)
    record.exitCode = exitCode
    record.seconds = Math.round((Date.now() - started) / 100) / 10
    record.metrics = parseMetrics(stdout)
    record.files = files
    record.stderrTail = String(stderr).split('\n').filter(line => line.trim() !== '').slice(-8).join('\n')
    appendMeta(record)
    return { ok: true, record, stdoutTail: String(stdout).split('\n').filter(line => line.trim() !== '').slice(-40).join('\n') }
  } finally {
    running = false
  }
}

export function listHistory() {
  return { ok: true, runs: readMeta() }
}

/** 服务 artifacts/sim 下的产物文件（白名单扩展名，路径校验防逃逸）。 */
export function serveArtifact(relative, res) {
  const clean = String(relative ?? '').replace(/\\/g, '/')
  if (clean.includes('..') || clean.startsWith('/')) return 403
  const target = path.resolve(ARTIFACT_DIR, clean)
  if (!target.startsWith(ARTIFACT_DIR)) return 403
  if (!ARTIFACT_EXTS.has(path.extname(target).toLowerCase())) return 404
  let stat
  try { stat = fs.statSync(target) } catch { return 404 }
  if (!stat.isFile() || stat.size > ARTIFACT_MAX_BYTES) return 404
  res.writeHead(200, {
    'content-type': path.extname(target).toLowerCase() === '.svg' ? 'image/svg+xml; charset=utf-8' : 'text/plain; charset=utf-8',
    'content-length': String(stat.size),
    'cache-control': 'no-store',
  })
  fs.createReadStream(target).pipe(res)
  return undefined
}

/** 三个只读预检工具（config_selfcheck + route_check），合成一个按钮。 */
const CHECKS = [
  { id: 'config_selfcheck', name: 'config_selfcheck（配置自检）', argv: ['tools/config_selfcheck.py'] },
  { id: 'route_check', name: 'route_check（路线六层校验）', argv: ['tools/route_check.py', '--config', 'config/robot.yaml'] },
]

export async function runChecks() {
  const results = []
  for (const check of CHECKS) {
    const { exitCode, stdout, stderr, timedOut } = await runChild('python', check.argv, {
      cwd: SIM_ROOT,
      timeoutMs: 60 * 1000,
    })
    results.push({
      id: check.id,
      name: check.name,
      exitCode,
      timedOut,
      tail: String(stdout + stderr).split('\n').filter(line => line.trim() !== '').slice(-30).join('\n'),
    })
  }
  return { ok: true, results }
}

