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
import os from 'node:os'
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

// ── 场地（全部**只读**；`config/**` 与 `src/dzbrain/**` 零写入）─────────
/** 现场地图真源（只读）。 */
const MAP_FILE = path.join(SIM_ROOT, 'config', 'map.yaml')
/** 现场配置真源（只读）——改点位的运行时用它的**运行目录副本**，不动这份。 */
const ROBOT_FILE = path.join(SIM_ROOT, 'config', 'robot.yaml')
/** 规划器 CLI：只读调用，`--map` 指 override 地图 ⇒ 路线随新点位重规划。 */
const PLAN_CLI = path.join(SIM_ROOT, 'tools', 'plan_route.py')
/** 工作区根（= SIM_ROOT 的上一级，即比赛存档 v1）：只读源覆盖只允许在这个根之内。
 *  为什么需要它：仿真台要"切到实车现图"（如 v1/_tmp_session/field7），而 /sim/run 的
 *  override 传统上只认 config/ 下两张文件；把可读范围钉在工作区内，避免一次请求就能读
 *  盘上任意 yaml。**只读**：源文件只被复制进运行目录，原文件零写入。 */
const WORKSPACE_ROOT = path.resolve(SIM_ROOT, '..')
const SOURCE_MAX_BYTES = 4 * 1024 * 1024

/** 路径是否在工作区内（含工作区本身；防 `..` 逃逸）。 */
function insideWorkspace(abs) {
  const rel = path.relative(WORKSPACE_ROOT, path.resolve(abs))
  return rel === '' || (!rel.startsWith('..') && !path.isAbsolute(rel))
}

/** 校验一个只读 YAML 源：存在、是文件、扩展名 yaml/yml、大小有上限、位于工作区内。 */
function readableYaml(raw) {
  const text = String(raw ?? '').trim()
  if (!text) return ''
  const abs = path.isAbsolute(text) ? path.resolve(text) : path.resolve(SIM_ROOT, text)
  if (!/\.ya?ml$/i.test(abs) || !insideWorkspace(abs)) return ''
  try {
    const stat = fs.statSync(abs)
    if (!stat.isFile() || stat.size > SOURCE_MAX_BYTES) return ''
  } catch { return '' }
  return abs
}

/**
 * 解析"场地来源覆盖"（页面「切到第七张图」/ 离线脚本用）。
 *
 * 形状：`{ dir }`（目录下必须同时有 robot.yaml + map.yaml）或 `{ configPath, mapPath }`。
 * 解析失败返回 `{ error }`（调用方显式 400/422，**绝不静默回落 config/** —— 静默回落
 * 会让操作员以为在跑第七张图、实际跑的是仓库第六张图，是最坏的一种"看起来成功"）。
 */
export function resolveFieldSource(raw) {
  if (!raw || typeof raw !== 'object') return null
  const dir = String(raw.dir ?? '').trim()
  let configPath = ''
  let mapPath = ''
  if (dir) {
    const absDir = path.isAbsolute(dir) ? path.resolve(dir) : path.resolve(SIM_ROOT, dir)
    if (!insideWorkspace(absDir)) return { error: '场地来源目录不在工作区内：' + absDir }
    configPath = path.join(absDir, 'robot.yaml')
    mapPath = path.join(absDir, 'map.yaml')
  } else {
    configPath = readableYaml(raw.configPath)
    mapPath = readableYaml(raw.mapPath)
    if (!configPath || !mapPath) {
      return { error: '场地来源需要 dir（含 robot.yaml+map.yaml）或两个合法 yaml 路径（工作区内、≤4MB）' }
    }
  }
  if (!readableYaml(configPath) || !readableYaml(mapPath)) {
    return { error: '场地来源缺文件：' + configPath + ' / ' + mapPath + '（需同时存在 robot.yaml 与 map.yaml）' }
  }
  // 可选的 route.tokens.txt：车端 robot.yaml 尚未部署手工路线时（第七张图 41 token 就属这种），
  // 由源目录随图带一条**手动路线**。有它 ⇒ /sim/map 画它、/sim/run（未改点位/未开无人机）
  // 直接按它跑；没有 ⇒ 仍读 robot.yaml 的 route.tokens / mission_blocks。
  let routePath = ''
  try {
    const candidate = path.join(path.dirname(mapPath), 'route.tokens.txt')
    const stat = fs.statSync(candidate)
    if (stat.isFile() && stat.size > 0 && stat.size <= 1024 * 1024) routePath = candidate
  } catch { /* 没有 route.tokens.txt：保持原行为 */ }
  return { dir: dir ? path.resolve(dir) : path.dirname(configPath), configPath, mapPath, routePath }
}
/** 场地列字母（`core/geometry.py:27-30` 的 COL_LETTERS）—— **插件进程内唯一字面量**。
 *  dronesim 的 `LETTERS`、stressbench 的 `COLS` 此前各抄一份 `'ABCDEFGHIJKLMN'`；
 *    现统一从这里取（页面侧不 import 模块，走 `/sim/map` 的 `grid.colLetters`，跨进程不共享）。 */
export const COL_LETTERS = 'ABCDEFGHIJKLMN'
/** 场地网格（COL_LETTERS / MAP_COLS=14 / MAP_ROWS=11 / BLOCK_SIZE=0.35）。 */
const GRID = { cols: 14, rows: 11, blockM: 0.35, colLetters: COL_LETTERS }
/** 页面允许编辑的四类点位（+targets —— 其余要素保持现场值）。 */
const FIELD_KEYS = ['mines', 'obstacles', 'bridges', 'targets']
/** 无人机起/降点：map.yaml 的识图注释里写着；读不到时用这两个（蓝 + H 图标）。
 *  dronesim 此前另抄一份同值兜底（注释还写着"与 simbench 内建兜底同值"）——
 *    现由本常量单点供给，改兜底只改这里。 */
export const DRONE_FALLBACK = { takeoff: 'N8', land: 'M2' }
/** 单格名正则（A~N 列 × 1~11 行），`normalizeCell` 与 stressbench 的图生成共用一份。 */
export const CELL_RE = /^[A-N](?:[1-9]|1[01])$/
const FIELD_MAX_CELLS = 40

/** `fieldSpec()` 单次调用内解析出的靶格（`targetBoard()` 从它判横竖）。 */
const TARGET_CACHE = { cells: [] }

/** 一次只跑一个仿真：并发会互相踩 artifacts/sim 的输出。 */
let running = false

/**
 * 压测批次忙标志（规格 §B.4）：压测期间手动单跑 `/sim/run` 仍要 409，文案含「压测进行中」。
 * 置位入口放本模块（`setStressBusy`）是为了 **simbench 不 import stressbench** —— 反向会成环。
 */
let stressBusy = false

/** 压测方（src/stressbench.js）在批次开/结束时调用：true=压测中，false=空闲。 */
export function setStressBusy(on) {
  stressBusy = on === true
}

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

// ── 场地读取 / 改点位 override（只读现场真源，产物只落运行目录）──────────

/** 去掉 YAML 标量的引号与行尾注释。 */
function unquote(raw) {
  return String(raw ?? '').replace(/\s+#.*$/, '').trim().replace(/^['"]|['"]$/g, '')
}

/** 取 `key:` 的单行标量（map.yaml 的 name/start/end/barrier_* 都是这个形状）。 */
function scalarOf(text, key) {
  const found = normalizeNewlines(text).match(new RegExp(`^[ \\t]*${key}:[ \\t]*(.*)$`, 'm'))
  return found ? unquote(found[1]) : ''
}

/** 现场 YAML 是 CRLF：解析前先把行尾归一，否则 `$` 行尾锚点/`- item` 匹配全废（实测踩过）。 */
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

/**
 * 从地图注释里认无人机起/降格（`# … 无人机起点 N8、终点 M2`）。
 *
 * 旧实现只匹配「无人机点」四字连续的注释行；第六张图注释写成「无人机起点 / 终点 M2」，
 * 于是恒落内建兜底 —— 值恰好相同，但 `drone.source` 会谎报 `builtin-fallback`，
 * 违反「起降格取真源」的口径（页面/台账据此判断是否已对齐现场）。
 * 现按两种注释形状取：① 「无人机起点/终点」标签后紧跟的格名；② 「无人机」之后的前两格。
 * 顺序恒为 [起点, 终点]；取不齐 ⇒ 调用方落内建兜底并如实标 source。
 */
function droneCellsFromComment(text) {
  const comments = []
  for (const line of normalizeNewlines(text).split('\n')) {
    const hit = line.match(/^[ \t]*#(.*)$/)
    if (hit) comments.push(hit[1])
  }
  const tagged = { takeoff: '', land: '' }
  for (const body of comments) {
    // 标签后必须紧跟格名（至多 4 个冒号/空白/括号字符），避免跨过「）、M2」抓到别人的格。
    if (!tagged.takeoff) {
      const hit = body.match(/无人机起点[：:\s（(]{0,4}([A-N](?:1[01]|[1-9]))/)
      if (hit) tagged.takeoff = normalizeCell(hit[1])
    }
    if (!tagged.land) {
      const hit = body.match(/无人机终点[：:\s（(]{0,4}([A-N](?:1[01]|[1-9]))/)
      if (hit) tagged.land = normalizeCell(hit[1])
    }
  }
  if (tagged.takeoff && tagged.land) return { cells: [tagged.takeoff, tagged.land], source: 'map-comment' }
  // 兼容「无人机点：N8、M2」这类没有“起点/终点”字样的注释：取「无人机」之后的前两个格名。
  const loose = []
  for (const body of comments) {
    const at = body.indexOf('无人机')
    if (at < 0) continue
    for (const raw of (body.slice(at).match(/[A-N](?:1[01]|[1-9])/g) ?? [])) {
      const cell = normalizeCell(raw)
      if (cell && !loose.includes(cell)) loose.push(cell)
      if (loose.length === 2) break
    }
    if (loose.length === 2) break
  }
  return loose.length === 2 ? { cells: loose, source: 'map-comment' } : { cells: [], source: 'builtin-fallback' }
}

/** 只读场地（给页面画横置地图）：解析失败返回 `ok:false`，绝不抛。
 *
 * options（可空，缺省 = 仓库 config/ 真源，行为与改动前逐字节一致）：
 *   · mapPath    — 指定 map.yaml（只读，如 v1/_tmp_session/field7/map.yaml）
 *   · configPath — 指定 robot.yaml（只读；解析 planner/bridge 参数用）
 * 场地来源覆盖走 resolveFieldSource() 的同一套校验；本函数只负责"读这张图"。
 */
export function fieldSpec(options = {}) {
  const mapPath = options && options.mapPath ? path.resolve(options.mapPath) : MAP_FILE
  const robotPath = options && options.configPath ? path.resolve(options.configPath) : ROBOT_FILE
  let text = ''
  try { text = fs.readFileSync(mapPath, 'utf8') } catch (error) {
    return { ok: false, root: SIM_ROOT, error: `读不到地图 ${mapPath}（${error?.message ?? error}）` }
  }
  const droneParsed = droneCellsFromComment(text)
  const colFrom = scalarOf(text, 'barrier_col_from').toUpperCase().slice(0, 1)
  TARGET_CACHE.cells = normalizeCells(listOf(text, 'targets'))
  return {
    ok: true,
    root: SIM_ROOT,
    source: mapPath,
    md5: md5Of(mapPath),
    robot: { path: robotPath, md5: md5Of(robotPath) },
    // 桥洞进洞侧冗余墙长度（米）——真源 config/robot.yaml `field.bridge_entry_redundancy_m`
    //   （bridgeEntryRedundancyOf 只读解析 dzbrain 段；缺键/读不到兜底 0.0875，与
    //   planner.BRIDGE_ENTRY_REDUNDANCY_M / physics.bridge_entry_walls 同源链）。
    //   /sim/map 透传给 sim.html 画 K 列桥洞格进洞侧两段短墙——页面端零第二份硬编码。
    bridgeEntryRedundancyM: bridgeEntryRedundancyOf(robotPath),
    grid: GRID,
    name: scalarOf(text, 'name'),
    start: normalizeCell(scalarOf(text, 'start')),
    end: normalizeCell(scalarOf(text, 'end')),
    mines: normalizeCells(listOf(text, 'mines')),
    obstacles: normalizeCells(listOf(text, 'obstacles')),
    targets: TARGET_CACHE.cells.slice(),
    bridges: normalizeCells(listOf(text, 'bridges')),
    barrier: {
      colFrom: /^[A-N]$/.test(colFrom) ? colFrom : 'D',
      rowHi: Number(scalarOf(text, 'barrier_row_hi')) || 6,
      rowLo: Number(scalarOf(text, 'barrier_row_lo')) || 7,
    },
    drone: {
      takeoff: droneParsed.cells[0] || DRONE_FALLBACK.takeoff,
      land: droneParsed.cells[1] || DRONE_FALLBACK.land,
      source: droneParsed.source,
    },
    // 车端规划口径（只读透传）：页面在「改点位」旁如实显示「本图路线是车端自动规划还是手动
    //   route.tokens」—— 第六张图 planner.auto_plan_enabled=false（tracker-safe 本图无解），
    //   改点位后的重规划因此会 422；不把这个语义显式告诉操作员，页面看起来就像「改点位坏了」。
    //   engine=null 表示读不到/不在 CLI 白名单（页面不硬编码引擎名）。
    planner: {
      engine: plannerEngineOf(robotPath),
      autoPlanEnabled: plannerFlagOf(robotPath, 'auto_plan_enabled'),
    },
    // 靶板由 targets 的相邻两格并成一块；按两格几何关系自动判横/竖（页面可覆盖）。
    target: targetBoard(),
    editable: FIELD_KEYS,
  }
}

/**
 * 靶板几何：把 `map.targets` 的格名按**屏幕坐标**判横竖。
 *
 * 横置屏幕坐标：列 → x、行 → y。两块靶格若列号不同 = 横向（长边沿列），
 * 行号不同 = 竖向（长边沿行）。只认两块；多于两块时按首两块判并如实报数量。
 */
function targetBoard() {
  return { yaw: targetYawOfCells(TARGET_CACHE.cells), cells: TARGET_CACHE.cells }
}

/** 由格名列表判横/竖（'h' | 'v' | ''，空 = 无法判定）。 */
function targetYawOfCells(cells) {
  const list = (Array.isArray(cells) ? cells : []).map(normalizeCell).filter(Boolean)
  if (list.length < 2) return ''
  const colIdx = c => GRID.colLetters.indexOf(String(c)[0].toUpperCase())
  const rowNum = c => Number(String(c).slice(1))
  const a = list[0]; const b = list[1]
  if (colIdx(a) < 0 || colIdx(b) < 0) return ''
  return colIdx(a) !== colIdx(b) ? 'h' : 'v'
}

/** 只读路线 token（给页面画参考路线）。
 *
 * 文件里的形状是 `route:` 段下的 `tokens: [...]`（展平后才叫 `route.tokens`）；
 *   更早的 `mission_blocks:` 块序列形状仍作兼容兜底。
 */
export function routeTokens() {
  return routeTokensOf(ROBOT_FILE)
}

/** 读指定 robot.yaml 的路线 token（字段来源覆盖 / 离线断言用；只读，缺文件返空）。 */
export function routeTokensOf(configPath) {
  try {
    const text = fs.readFileSync(configPath, 'utf8')
    const tokens = listOf(text, 'tokens')
    return tokens.length ? tokens : listOf(text, 'mission_blocks')
  } catch { return [] }
}

/** 读一份**只含 token** 的路线文件（源目录 route.tokens.txt / 手工路线）；非法即空。
 *  与 routeTokensOf 的区别：这是"路线即 tokens"文件，不解析 YAML；末点 ,x 保留
 *  （写 sim 的 --route 文件时由 runSimJob 单独剥离）。 */
export function routeTokensFromFile(filePath) {
  try {
    const raw = fs.readFileSync(filePath, 'utf8').replace(/^\uFEFF/, '')
    const normalized = normalizeRouteTokens(raw.trim().split(/\s+/).filter(Boolean))
    return normalized.rejected.length === 0 ? normalized.tokens : []
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

/** 无人机协同（规格 §D.2）—— 把无人机起降格注入 override 图的 obstacles 列表。
 *
 * 为什么注 obstacles：dzbrain 地图格式只有 mines/obstacles/targets/bridges 四键，没有专门的
 * 「避让」键；规划器禁行集 = obstacles|mines（`route/planner.py` 的 blocked）⇒ obstacles 是
 * 插件层实现**硬禁行**的唯一途径。已知取舍：该格在仿真物理里会同时当「障碍」（遮挡视线/
 * 碰撞体）—— 无人机起降格本就不该有车经过，接受这个更强的语义。
 *
 * 为什么不做 planner `--drone` 参数 / route_check 同口径检查 / 走廊行软约束代价：三者都要写
 * dzrobot_v2 的 src/ 与 tools/（2026-10-17 只读新规禁写）⇒ 推迟到车侧改版；台账的
 * corridorRow 字段（corridorRows）先把软约束的走廊行推导同源落档。
 *
 * 形状：对 applyFieldToMapText 的**产物**做二次变换（不改 applyFieldToMapText 本体、不碰真源
 * config/map.yaml）—— 与棋盘改点的 field 变换叠加进**同一张** override 图。合并走
 * normalizeCells（去重/排序与 applyFieldToMapText 同一口径 ⇒ 同一张图无论谁先谁后结果一致）；
 * obstacles 行形状意外（不在）⇒ 抛错拒绝写坏地图（与 applyFieldToMapText 同款护栏）。 */
export function injectDroneAvoidToMapText(text, cells) {
  const raw = String(text ?? '')
  const add = normalizeCells(cells)
  if (add.length === 0) return raw
  const merged = normalizeCells([...listOf(raw, 'obstacles'), ...add])
  const line = `obstacles: ["${merged.join('", "')}"]`
  // `m` 标志不能少：obstacles 行在 map.yaml 中部，^$ 不按行锚就会整串匹配失败（误抛）。
  const pattern = /^([ \t]*)obstacles:[ \t]*\[[^\]]*\][ \t]*$/m
  if (!pattern.test(raw)) throw new Error('地图里没有 obstacles: [...] 这一行，拒绝注入（避免写出坏地图）')
  return raw.replace(pattern, `$1${line}`)
}

/** 无人机扫描**走廊行**（上半场 top / 下半场 bottom，1-based 行号；规格 §D.3）。
 *  推导从 src/dronesim.js 的 scanPlan **原样平移**（行为逐点不变；dronesim 改为 import 本函数，
 *  全仓恰一份 —— 规格「边界与已知取舍」明示走廊行推导不得有第二真源）：
 *    bridges 里的 K 列格按行分半场（行 ≥7 下半场 / ≤6 上半场），各取排序后的**首个**命中行
 *    （normalizeCells 已按 A→N / 1→11 排序，与 dronesim 旧推导同一输入同一选法）；
 *    缺省 top=5 / bottom=9（§D.2 软约束缺省，与旧推导同值）。
 *  语义 = **软约束**：走廊行本身可通行，只是无人机在扫、绕行更优 —— 台账
 *  record.plan.drone.corridorRow 落档用，规划代价加权推迟（见 injectDroneAvoidToMapText 注释）。 */
export function corridorRows(field) {
  const bridges = normalizeCells((field && field.bridges) || [])
  const rowOf = cell => Number(String(cell).slice(1))
  const lowerBridge = bridges.find(c => c[0] === 'K' && rowOf(c) >= 7)
  const upperBridge = bridges.find(c => c[0] === 'K' && rowOf(c) <= 6)
  return {
    top: upperBridge ? rowOf(upperBridge) : 5,
    bottom: lowerBridge ? rowOf(lowerBridge) : 9,
  }
}

/** `tools/plan_route.py --engine` 的合法取值 —— **从 CLI 自己的 `choices=(…)` 读**。
 *
 *  为什么不再在插件里写死 `['tour','tracker-safe']`：写死等于维护**第二份引擎白名单**，
 *  车侧一旦加/改引擎名（例如 `tracker-safe-v2`），页面这边会静默判成"不认识 ⇒ 不传
 *  --engine"，于是又退回 `plan_route.py` 的默认 `tour` —— 也就是「乱算」原样复发，
 *  而且**不报错、看不出来**。真源只有一个：CLI 的 argparse 定义。
 *  读不到 CLI / 形状认不出（改写成变量拼装等）→ 兜底用历史上的两项，行为与改动前逐字节
 *  一致（既不会少传也不会给 argparse 塞未知值）。进程内缓存一次（CLI 是只读文件，运行期
 *  不变；真改了要重启宿主，与此前 `PLAN_CLI` 常量同口径）。 */
const ENGINE_FALLBACK_CHOICES = ['tour', 'tracker-safe']
let engineChoicesCache = null
export function plannerEngineChoices() {
  if (engineChoicesCache) return engineChoicesCache
  let choices = ENGINE_FALLBACK_CHOICES
  try {
    const text = fs.readFileSync(PLAN_CLI, 'utf8')
    const hit = text.match(/add_argument\(\s*["']--engine["']\s*,\s*choices\s*=\s*\(([^)]*)\)/)
    if (hit) {
      const quoted = hit[1].match(/["'][^"']+["']/g) ?? []
      const found = quoted.map(item => item.slice(1, -1)).filter(Boolean)
      if (found.length > 0) choices = found
    }
  } catch { /* CLI 读不到 → 兜底清单（与改动前同一行为） */ }
  engineChoicesCache = choices
  return choices
}

/** 读 robot.yaml 里 planner 块下的 engine（车上自动规划用的引擎）。
 *  为什么必须传 --engine：页面重规划此前不传 ⇒ plan_route.py 默认 tour，而车端配置的是
 *    tracker-safe —— 同一张图页面和车算出**两条不同的路线**（实测 token 数与路线形状都
 *    不同，tour 产物还可能带 `:PIVOT` 折返，而仿真不建模 PIVOT）。
 *    ⇒ **只有跟车端同引擎才逐 token 相等**。
 *    这里以车端配置为唯一真源：配什么引擎，页面就传什么。
 *  形状：`planner:` 块（与 route:/mission: 同级）下的 `engine: tracker-safe`，允许行内注释与
 *  YAML 引号（`engine: "tracker-safe"` 也认）。
 *  找不到 / 值不在 plan_route.py 的 choices 里 → 返回 null（不传 --engine，保持其历史
 *  默认 tour，与改动前逐字节一致）。导出仅供离线断言（同 sanitizeParams 的先例）。
 *
 *  ✅ 已闭环：`src/dronesim.js` 的 `truthPlanEvidence()` 已复用本函数拼 `--engine`
 *  （同引擎取证，无人机协同时间线与车端口径一致）；`from_cell` / `already_covered`
 *  也走同一条白名单链路（`appendIncrementalArgs()`）。 */
export function plannerEngineOf(configPath) {
  try {
    const lines = fs.readFileSync(configPath, 'utf8').split(/\r?\n/)
    let blockIndent = -1
    for (const line of lines) {
      if (!line.trim() || /^\s*#/.test(line)) continue
      const indent = line.match(/^[ \t]*/)[0].length
      if (blockIndent < 0) {
        if (/^[ \t]*planner:[ \t]*(?:#.*)?$/.test(line)) blockIndent = indent
        continue
      }
      if (indent <= blockIndent) break                       // planner 块结束
      const hit = line.match(/^[ \t]*engine:[ \t]*([^\s#]+)/)
      if (hit) {
        // 去 YAML 引号（`engine: "tour"` 与 `engine: tour` 同义；带引号时会误判未知）
        const value = hit[1].replace(/^['"]|['"]$/g, '')
        return plannerEngineChoices().includes(value) ? value : null
      }
    }
  } catch { /* 配置读不到 → 走 plan_route 默认 */ }
  return null
}

/** 读 robot.yaml `planner:` 块下的布尔开关（只读；缺键/值非 true|false ⇒ null，不猜默认）。
 *
 *  当前唯一消费者 = `auto_plan_enabled`：车端为 false 时路线来自手动 `route.tokens`，
 *  页面仍会按车端引擎重规划改点位后的路线（这是仿真台的 what-if 能力）—— 页面把该状态
 *  透传显示，让「改点位 → 422 规划无解」有据可查，而不是看起来像改了没生效。
 *  导出仅供离线断言（同 plannerEngineOf 的先例）。 */
export function plannerFlagOf(configPath, key) {
  try {
    const lines = fs.readFileSync(configPath, 'utf8').split(/\r?\n/)
    let blockIndent = -1
    for (const line of lines) {
      if (!line.trim() || /^\s*#/.test(line)) continue
      const indent = line.match(/^[ \t]*/)[0].length
      if (blockIndent < 0) {
        if (/^[ \t]*planner:[ \t]*(?:#.*)?$/.test(line)) blockIndent = indent
        continue
      }
      if (indent <= blockIndent) break                       // planner 块结束
      const hit = line.match(new RegExp('^[ \\t]*' + key + ':[ \\t]*([^\\s#]+)'))
      if (hit) {
        const raw = hit[1].replace(/^['"]|['"]$/g, '').toLowerCase()
        return raw === 'true' ? true : (raw === 'false' ? false : null)
      }
    }
  } catch { /* 配置读不到 ⇒ null（不猜默认值） */ }
  return null
}

/* ── 增量规划透传（`from_cell` / `already_covered`）────────────────────
 *
 * 车端 planner 层支持这两个入参（`plan_route_tracker_safe(..., from_cell=, already_covered=)`），
 * 仿真台要透传上去，口径与 `--engine` **完全同链路**：
 *   ① 真源 = `tools/plan_route.py` 的 argparse（`plannerIncrementalSupport()` 现场读，不抄第二份）；
 *   ② **声明了才传**，一个字节不多传 —— 给 argparse 塞未知参数会 exit 2，页面侧只表现为
 *      "改点位后规划失败（422）"，比不传更难查；
 *   ③ 缺省不传 ⇒ argv 与改动前**逐字节相同**（整线规划语义不变）。
 *
 * 现役 CLI `supported = 2/2`，守卫 E 段实跑；若未来接线被回退，`plannerIncrementalSupport()`
 * 自动变空，降级为"不传"、台账 `incremental.dropped` 记诉求（不静默吞）。
 */
const INCREMENTAL_OPTIONS = [
  { key: 'fromCell', option: '--from-cell', aliases: ['--from_cell'], kind: 'cell' },
  { key: 'alreadyCovered', option: '--already-covered', aliases: ['--already_covered'], kind: 'cells' },
]
/** 按 CLI 路径缓存（影子 CLI 也要能查到自己的那份，故不用单值缓存）。 */
const incrementalCache = new Map()

/** 本 CLI 声明了哪几条增量开关（真源唯一 = argparse）。
 *  返回 `{ supported: [{key, option, kind}], missing: [key...], cli }`；读不到 ⇒ 全 missing。 */
export function plannerIncrementalSupport(cliPath = PLAN_CLI) {
  const cacheKey = String(cliPath)
  if (incrementalCache.has(cacheKey)) return incrementalCache.get(cacheKey)
  let text = ''
  try { text = fs.readFileSync(cliPath, 'utf8') } catch { /* 读不到 → 全 missing（不传，保持现状） */ }
  const supported = []
  const missing = []
  for (const spec of INCREMENTAL_OPTIONS) {
    const declared = [spec.option, ...spec.aliases].find(opt => (
      text.includes(`add_argument("${opt}"`) || text.includes(`add_argument('${opt}'`)
    ))
    if (declared) supported.push({ key: spec.key, option: declared, kind: spec.kind })
    else missing.push(spec.key)
  }
  const result = { supported, missing, cli: cliPath }
  incrementalCache.set(cacheKey, result)
  return result
}

/** 单个格名归一（非法 → null；复用 `normalizeCell`，与四类点位同一口径）。 */
export function normalizeFromCell(raw) {
  if (raw == null) return null
  return normalizeCell(raw) || null
}

/** 把增量参数按 CLI 真源**白名单化**追加进 argv（就地 push；返回值带台账字段）。
 *
 *  · 声明了才传；未声明 ⇒ 一个字节都不加，但把诉求如实记进 `dropped`（不静默吞）；
 *  · 非法格名同样进 `dropped`（reason=`invalid-cell`），不让 argparse 去报一句用户看不懂的参数错；
 *  · `alreadyCovered` 走 `normalizeCells`（去重 + 排序），与页面四类点位的归一同一口径。
 *  导出仅供离线断言 + dronesim 复用（与 `plannerEngineOf` / `sanitizeParams` 的先例一致）。 */
export function appendIncrementalArgs(argv, { fromCell = null, alreadyCovered = [] } = {}, { cliPath = PLAN_CLI } = {}) {
  const support = plannerIncrementalSupport(cliPath)
  const byKey = new Map(support.supported.map(item => [item.key, item]))
  const wantedFromCell = normalizeFromCell(fromCell)
  const wantedCovered = normalizeCells(alreadyCovered)
  const applied = {
    fromCell: null, alreadyCovered: [], flags: [],
    dropped: [], supported: support.supported.map(item => item.option), cli: String(cliPath),
  }
  const askedFromCell = fromCell != null && String(fromCell).trim() !== ''
  if (wantedFromCell && byKey.has('fromCell')) {
    argv.push(byKey.get('fromCell').option, wantedFromCell)
    applied.fromCell = wantedFromCell
    applied.flags.push(byKey.get('fromCell').option)
  } else if (askedFromCell) {
    applied.dropped.push({ key: 'fromCell', value: String(fromCell), reason: wantedFromCell ? 'cli-unsupported' : 'invalid-cell' })
  }
  const askedCovered = Array.isArray(alreadyCovered) ? alreadyCovered.length > 0 : String(alreadyCovered ?? '').trim() !== ''
  if (wantedCovered.length && byKey.has('alreadyCovered')) {
    argv.push(byKey.get('alreadyCovered').option, wantedCovered.join(','))
    applied.alreadyCovered = wantedCovered
    applied.flags.push(byKey.get('alreadyCovered').option)
  } else if (askedCovered) {
    applied.dropped.push({ key: 'alreadyCovered', value: wantedCovered, reason: wantedCovered.length ? 'cli-unsupported' : 'invalid-cell' })
  }
  return { argv, applied }
}

/** `field.bridge_entry_redundancy_m` 的缺省兜底（米）。
 *  与 `route/planner.py` 的兜底常量 `BRIDGE_ENTRY_REDUNDANCY_M` **同源同值**
 *  （0.0875 = 0.35 × 1/4 格，用户 2026-10-16 设计规格）；真源是 config/robot.yaml。 */
const BRIDGE_ENTRY_REDUNDANCY_FALLBACK_M = 0.0875

/** robot.yaml 里 dzbrain 段 `field:` 块下的 `bridge_entry_redundancy_m`
 *  （桥洞**进洞侧冗余墙**长度，米）——只读解析，绝不写 config。
 *
 *  同源链（单一真源，页面端禁止再抄第二份 0.0875）：
 *    `route/physics.py: bridge_entry_walls()`（几何）⇐ `planner.BRIDGE_ENTRY_REDUNDANCY_M`
 *    （兜底常量）⇐ **config/robot.yaml `field.bridge_entry_redundancy_m`（真源）** ⇐ 本函数
 *    ⇐ `fieldSpec()` 透传 `bridgeEntryRedundancyM` ⇐ GET /sim/map ⇐ sim.html 渲染进洞侧短墙。
 *
 *  为什么**必须限定 dzbrain 段**（比 plannerEngineOf 多扫一层）：robot.yaml 里有**两个**
 *  `field:` 块 —— `/dzbrain:` 段下的（L571，展平成裸键 `field.*`，planner/sim 读的就是它）
 *  与 `/dzactuator:` 段下的（L808，`core/config.py: SECTION_PREFIX` 把它展平成
 *  `dzactuator.field.*`，与本键无关）。只扫"第一个 field: 块"会在这两块之间踩错；
 *  两层扫描 = 顶层先定位 `dzbrain:`/`/dzbrain:` 段（SECTION_PREFIX 里映射为空前缀的段），
 *  段内再用 plannerEngineOf 同款块扫描找 `field:` 块。已用真加载器取证：
 *  `Config.load('config/robot.yaml').get('field.bridge_entry_redundancy_m') = 0.0875`，
 *  `dzactuator.field.bridge_entry_redundancy_m` 不存在。
 *
 *  读不到文件 / 没有 dzbrain 段 / 段内缺键 / 值非数值 → 兜底 0.0875（与 planner 缺
 *  config 时同口径）。`0.0` 是**合法值**（= 关冗余），只有**取不到值**
 *  才兜底；负值原样返回，由消费方按 physics `max(0.0, ·)` 口径钳 0。
 *  导出仅供离线断言（同 sanitizeParams/plannerEngineOf 的先例；index.js 不引用它）。 */
export function bridgeEntryRedundancyOf(configPath) {
  try {
    const lines = fs.readFileSync(configPath, 'utf8').split(/\r?\n/)
    // 第一层：顶层 `dzbrain:` / `/dzbrain:` 段（0 缩进；SECTION_PREFIX 映射为空前缀）
    let sectionIndent = -1
    let idx = 0
    for (; idx < lines.length; idx += 1) {
      const line = lines[idx]
      if (!line.trim() || /^\s*#/.test(line)) continue
      if (line.match(/^[ \t]*/)[0].length > 0) continue          // 只认顶层段名
      if (/^\/?dzbrain:[ \t]*(?:#.*)?$/.test(line)) {
        sectionIndent = line.match(/^[ \t]*/)[0].length
        idx += 1
        break
      }
    }
    if (sectionIndent < 0) return BRIDGE_ENTRY_REDUNDANCY_FALLBACK_M
    // 第二层：dzbrain 段内找 `field:` 块（块扫描手法与 plannerEngineOf 同款）
    let blockIndent = -1
    for (; idx < lines.length; idx += 1) {
      const line = lines[idx]
      if (!line.trim() || /^\s*#/.test(line)) continue
      const indent = line.match(/^[ \t]*/)[0].length
      if (indent <= sectionIndent) break                         // dzbrain 段结束
      if (blockIndent < 0) {
        if (/^[ \t]*field:[ \t]*(?:#.*)?$/.test(line)) blockIndent = indent
        continue
      }
      if (indent <= blockIndent) break                           // field 块结束
      const hit = line.match(/^[ \t]*bridge_entry_redundancy_m:[ \t]*([^\s#]+)/)
      if (hit) {
        const value = Number(hit[1].replace(/^['"]|['"]$/g, ''))
        return Number.isFinite(value) ? value : BRIDGE_ENTRY_REDUNDANCY_FALLBACK_M
      }
    }
  } catch { /* robot.yaml 读不到 → 兜底（与 planner 缺 config 时同口径） */ }
  return BRIDGE_ENTRY_REDUNDANCY_FALLBACK_M
}

/** 调 `tools/plan_route.py`（只读）→ 重新规划出的 token。
 *
 *  ★ 导出**仅供离线断言/取证**（同 `sanitizeParams` / `plannerEngineOf` 的先例；
 *    `index.js` 不引用它，`runSimJob` 是唯一真实调用方）。为什么要给断言留口子：
 *    「页面重规划 == 车端路线」这条验收，只有走**这一份** argv 拼装才算证据 ——
 *    在测试里自己复制一份 `--engine` 拼装，测的是复制品而不是线上代码。
 *    跑法（工作区根目录，node ≥ 20）：
 *      node --input-type=module -e "import {planRoute, routeTokens} from './dsh-plugin-dzrobot-ssh/src/simbench.js'; ..."
 *    验收口径：页面与车端 44 ↔ 44 逐 token 相等（见回归输出）。
 *
 *  入参：`fromCell`（增量续算起点）/ `alreadyCovered`（已压格，只对雷生效）——
 *    车端 planner 入参，按 `appendIncrementalArgs` 白名单化后追加到 `--engine` 之后；
 *    缺省 `null/[]` ⇒ argv 与改动前**逐字节相同**（整线规划语义不变）。
 *    `cliPath` 缺省 = 工作区 `tools/plan_route.py`；仅在离线断言里指向**影子 CLI**（%TEMP% 副本），
 *    用来钉住"声明后真的会传、产物 == 车端 planner"这条 E2E。 */
export async function planRoute({ configPath, mapPath, timeoutMs = 300 * 1000, fromCell = null, alreadyCovered = [], cliPath = PLAN_CLI, engine: engineWanted = null }) {
  const argv = [cliPath === PLAN_CLI ? 'tools/plan_route.py' : cliPath,
                '--config', configPath, '--map', mapPath,
                '--emit', 'json', '--check', '--quiet']
  // 修「乱算」：默认与车端同引擎（robot.yaml planner.engine；读不到就不传，保持历史默认）。
  // R1 显式引擎：只有调用方显式给出且在白名单里才覆盖；**非法值不回落**（返回 engineError），
  //   避免"选了 tour 却静默按 tracker-safe 跑"这种最难查的假成功。
  const requested = engineWanted == null || String(engineWanted).trim() === '' ? '' : String(engineWanted).trim()
  const explicit = requested && plannerEngineChoices().includes(requested) ? requested : null
  const engineError = requested && !explicit ? '不认识的规划引擎：' + requested : ''
  const engine = explicit || plannerEngineOf(configPath)
  if (engine) argv.push('--engine', engine)
  // 增量规划透传（from_cell / already_covered）——**声明了才传**，缺省逐字节不变。
  //   与 --engine 同链路：先引擎、后增量，白名单来自 CLI 真源，诉求/降级都进台账。
  // 增量规划（--from-cell / --already-covered）仅 tracker-safe 引擎支持；tour 下不传以避免
  // CLI argparse 硬拒（tools/plan_route.py:282-283）。
  const incremental = engine === 'tracker-safe'
    ? appendIncrementalArgs(argv, { fromCell, alreadyCovered }, { cliPath }).applied
    : { flags: [], dropped: [], requested: [], supported: [] }
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
    engine,
    // 引擎来源（R1 台账）：explicit=页面显式选的非默认引擎；config=跟随车端；
    //   engineError 非空 ⇒ 请求值非法（调用方必须显式失败，不准把 config 值当成功交付）。
    engineRequested: requested || null,
    engineSource: explicit ? 'explicit' : 'config',
    engineError,
    // 取证用（离线断言/台账都要能回答"这一趟到底带了哪些参数"）
    argv: argv.slice(),
    incremental,
    tokens: Array.isArray(plan?.tokens) ? plan.tokens : [],
    plan,
    check: payload?.route_check?.[0] ?? null,
    failures: payload?.failures ?? [],
    stderrTail: String(run.stderr).split('\n').filter(line => line.trim() !== '').slice(-8).join('\n'),
    stdoutTail: String(run.stdout).split('\n').filter(line => line.trim() !== '').slice(-8).join('\n'),
  }
}

/**
 * 轻量规划检查（不跑仿真、不落运行目录）：给「规划压测」和「补图预筛」用。
 *
 * 与 runSimJob 的区别：只把本次 field 写进**系统临时目录**（用完即删），
 *   config/ 与 artifacts/sim/ 都零写入；返回结构足够做成功率/token/耗时统计。
 * 为什么不在 artifacts 里建 run 目录：64 张图 × 预筛会制造大量只含 map_override
 *   的「僵尸 run」，污染历史列表；规划结果本身只需要摘要。
 */
export async function checkPlan({ field = null, engine = null } = {}) {
  const started = Date.now()
  let tmp = null
  try {
    let configPath = ROBOT_FILE
    let mapPath = MAP_FILE
    if (field) {
      tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'dzsim-plan-'))
      const overrideMap = path.join(tmp, 'map.yaml')
      const overrideRobot = path.join(tmp, 'robot.yaml')
      fs.writeFileSync(overrideMap, applyFieldToMapText(fs.readFileSync(MAP_FILE, 'utf8'), field), 'utf8')
      fs.copyFileSync(ROBOT_FILE, overrideRobot)
      configPath = overrideRobot
      mapPath = overrideMap
    }
    const planned = await planRoute({ configPath, mapPath, engine })
    const ok = !planned.engineError && planned.exitCode === 0 && !planned.timedOut
      && planned.tokens.length > 0
    const firstFailure = Array.isArray(planned.failures) && planned.failures.length > 0
      ? String(planned.failures[0]?.message ?? planned.failures[0] ?? '')
      : ''
    return {
      ok,
      ms: Date.now() - started,
      tokens: planned.tokens,
      plan: planned.plan ?? null,
      check: planned.check ?? null,
      failures: planned.failures ?? [],
      exitCode: planned.exitCode,
      timedOut: planned.timedOut,
      engine: planned.engine,
      engineRequested: planned.engineRequested,
      engineSource: planned.engineSource,
      engineError: planned.engineError,
      stderrTail: planned.stderrTail,
      stdoutTail: planned.stdoutTail,
      error: planned.engineError || (ok ? '' : (firstFailure
        || planned.stderrTail || ('plan_route exit=' + planned.exitCode))),
    }
  } finally {
    if (tmp) {
      try { fs.rmSync(tmp, { recursive: true, force: true }) } catch { /* 清理失败不改变结论 */ }
    }
  }
}

/**
 * 参数快照（用于台账，与 run 记录一起存）。
 *
 * 导出**仅供离线断言**使用（`scripts/test-sim-map-render.mjs` 的「E 透传链②」一组）：
 *   它不是插件的对外 API，`index.js` 不引用它，`runSim()` 是唯一的真实调用方。
 *   理由：`targetYaw`（靶板朝向）从表单到台账的链条里，只有这一段是**纯函数**，
 *   能不启服务、不跑 python 就被断言钉住；「record 落盘」那一段由真跑取证覆盖
 *   （5 跑实测：run_meta.jsonl 落 `"targetYaw":"v"/"h"`、非法值落 null，listHistory 原样回读）。
 */
/** 手工路线 token 的**轻量**白名单：只挡"显然不是格名/后缀"的输入。
 *
 * 为什么不在宿主里做完整解析：路线语法（R: 前缀 / * 后缀 / :SCAN / :PIVOT / ,x）的唯一真源是车侧
 *   `route/parser.py`；宿主只做形状校验，语义错误交给 sim_cli 自己报（错误码/文案同车侧）。
 * 为什么仍要挡：`--route` 文件按空白/逗号切词，像 `N5,x` 这种末点朝向会切成两个 token；
 *   宿主在写文件时剥掉**末点**后缀（见 runSimJob），但中间点带朝向、空 token、乱码必须显式拒绝。
 */
export function normalizeRouteTokens(raw) {
  const list = Array.isArray(raw) ? raw : String(raw ?? '').split(/\s+/)
  const tokens = []
  const rejected = []
  for (const item of list) {
    const token = String(item ?? '').trim()
    if (!token) continue
    const upper = token.toUpperCase()
    if (!/^(?:R:)?[A-N](?:[1-9]|1[01])(?::SCAN|:PIVOT|\*|,[XY])?$/.test(upper)) {
      rejected.push(token)
      continue
    }
    tokens.push(upper.replace(/,X$/, ',x').replace(/,Y$/, ',y'))
  }
  return { tokens, rejected }
}

export function sanitizeParams(body = {}) {
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
  // seed 留空 = sim_cli 的 SimConfig 默认（不传 `--seed`，与改动前逐字一致）。
  const seed = Number.isInteger(body.seed) && body.seed >= 0 ? body.seed : null
  // 靶板朝向（'h' 横向 / 'v' 竖向 / '' 跟随地图）——**只用于渲染与台账**，
  //   不写 config、不进 sim_cli 参数（仿真物理不感知靶板朝向）。
  const targetYaw = body.targetYaw === 'h' || body.targetYaw === 'v' ? body.targetYaw : null
  // 靶板蓝红面（'blue' | 'red'）——仿 targetYaw 先例：只用于渲染与台账，
  //   不写 config、不进 sim_cli 参数。与 targetYaw 的差别只有一处：targetYaw 恒在返回
  //   键表里（先例：scripts/test-sim-map-render.mjs E 组钉死 sp({}) 恰 7 键），
  //   targetFace 仅在请求体**带了该键**时才挂进返回对象 —— 页面 readForm 恒带 ⇒ 真跑
  //   台账恒有该字段；压测/无人机体不带 ⇒ 记录形状与改动前逐字一致。
  const targetFace = body.targetFace === 'blue' || body.targetFace === 'red' ? body.targetFace : null
  // 无人机协同开关（规格 §D）—— 仅 `true` 通过（false/字符串/其它一律 null 丢弃），
  //   **带键才挂**（targetFace 同款先例）：页面不勾时请求体不带 drone 键 ⇒ params 无该键，
  //   sp({}) 键表保持 7 键（守卫 E 组钉死）、台账形状与 OFF 现状逐字一致（§D.1）。
  //   开启语义 = 车路线硬避让无人机起降格：runSimJob 里经 override 图 obstacles 注入 +
  //   planRoute 重规划（插件层唯一硬禁行键），不进 sim_cli 参数（仿真物理不感知）。
  const drone = body.drone === true ? true : null
  // 三类点位若要改，就在运行目录里生成 override（config/** 零写入）。
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
  // 场地来源覆盖（页面「切到第七张图」）——带键才挂，且**校验失败不静默回落**：
  //   回落 config/ 会让操作员以为在跑现场图、实际跑仓库第六张图，是最坏的一种"看起来成功"。
  let source = null
  let sourceError = ''
  if (body.source !== undefined) {
    const resolved = resolveFieldSource(body.source)
    if (!resolved) sourceError = '场地来源格式不对（需 {dir} 或 {configPath,mapPath}）'
    else if (resolved.error) sourceError = resolved.error
    else source = resolved
  }
  // 手工路线（路线即 tokens；只在仿真台离线脚本/what-if 用）——同样带键才挂。
  let route = null
  let routeError = ''
  if (body.route !== undefined) {
    const raw = body.route && typeof body.route === 'object' && !Array.isArray(body.route)
      ? body.route.tokens
      : body.route
    const normalized = normalizeRouteTokens(raw)
    if (normalized.rejected.length) routeError = '手工路线含非法 token：' + normalized.rejected.slice(0, 8).join(' ')
    else if (normalized.tokens.length === 0) routeError = '手工路线为空'
    else route = { tokens: normalized.tokens, source: String(body.route?.source ?? 'manual').slice(0, 40) }
  }
  // 显式规划引擎（R1）——默认 null = 跟随 robot.yaml planner.engine（车端口径不漂移）；
  //   只有显式传值且值在 plan_route.py 的 choices 白名单里才生效，否则**报错不回落**。
  let engine = null
  let engineError = ''
  if (body.engine !== undefined) {
    const wanted = String(body.engine ?? '').trim()
    if (plannerEngineChoices().includes(wanted)) engine = wanted
    else engineError = '不认识的规划引擎：' + wanted + '（plan_route.py 只认 ' + plannerEngineChoices().join('/') + '）'
  }
  const params = { overrides, inject, label, maxSteps, seed, targetYaw, field: fieldTouched ? field : null }
  // 带键才挂 —— sp({}) 的键表保持 7 键不变（守卫 E 组钉死），不破 61/61。
  if (body.targetFace !== undefined) params.targetFace = targetFace
  // 同款带键才挂 —— 不带 drone 键的请求（压测/无人机体/OFF）params 形状不变。
  if (body.drone !== undefined) params.drone = drone
  // 同上：只有请求体显式带这些键才挂，普通跑/压测记录形状逐字不变。
  if (body.source !== undefined) { params.source = source; if (sourceError) params.sourceError = sourceError }
  if (body.route !== undefined) { params.route = route; if (routeError) params.routeError = routeError }
  if (body.engine !== undefined) { params.engine = engine; if (engineError) params.engineError = engineError }
  return params
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
 * 这两条是**照着 sim_cli 的真实排版**钉的（实测：不这么写会抓错行）：
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
export function runChild(command, argv, { cwd, timeoutMs, env = {} }) {
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

/** 追加一条台账记录（导出给 stressbench：压测自产的 invalid/error 记录进同一 run_meta.jsonl）。 */
export function appendMeta(record) {
  fs.mkdirSync(ARTIFACT_DIR, { recursive: true })
  fs.appendFileSync(META_FILE, `${JSON.stringify(record)}\n`, 'utf8')
}

/** 台账尾部记录（`run_meta.jsonl`）：`kind` 给了就只留该类的最近 50 条。
 *  统一入口：`listHistory` 不带 kind、`droneHistory` 带 `kind:'drone'`，
 *    避免各写一份「读文件→逐行 parse→按 kind 过滤→slice(-50)」的过滤逻辑。 */
export function readMeta({ kind = null } = {}) {
  try {
    const records = fs.readFileSync(META_FILE, 'utf8').split('\n').filter(line => line.trim() !== '')
      .map(line => { try { return JSON.parse(line) } catch { return null } }).filter(Boolean)
    return (kind ? records.filter(record => record.kind === kind) : records).slice(-50)
  } catch { return [] }
}

/** 主接口：跑一次仿真（锁检查 + runSimJob；对外行为逐字不变 —— 规格 §B.4 的拆分只挪不改）。 */
export async function runSim(body) {
  if (running) return { ok: false, code: 409, error: '上一次仿真还在跑，稍候再试' }
  if (stressBusy) return { ok: false, code: 409, error: '压测进行中，单跑请稍候再试' }
  if (!looksLikeWorkspace(SIM_ROOT)) {
    return {
      ok: false, code: 409,
      error: `工作区不可用（${SIM_ROOT} 下没有 src/dzbrain）：安装副本需要在插件目录放一份 `
        + `${ROOT_POINTER_FILE}（内容 {"root":"<dzrobot_v2 绝对路径>"}）`,
    }
  }
  return runSimJob(body, { lock: true })
}

/**
 * 仿真执行体（规格 §B.4 拆分产物，供压测走内部通道）：
 *   · `lock=true`（手动单跑）：持/放模块级 `running`，409 互斥语义原样保留；
 *   · `lock=false`（压测子跑）：完全绕开 `running` —— 池互斥由 stressbench 自管，
 *     run id 带 seq 保证同图不同批也不写同一个 outDir；
 *   · `runId` 覆盖默认 `${Date.now()}_${label}`：压测用规格字面
 *     `${Date.now()}_${batchId}_${seq}_${label}`（并发同毫秒互踩防护，§B.5）；
 *   · `batch` 非空才往 record 写 `batchId/seq`（手动记录形状不变）。
 */
export async function runSimJob(body, { lock = true, runId = null, batch = null } = {}) {
  const params = sanitizeParams(body)
  const id = runId ?? `${Date.now()}_${params.label}`
  const outDir = path.join(ARTIFACT_DIR, id)
  const record = {
    id,
    at: new Date().toISOString(),
    label: params.label,
    ...(batch && batch.batchId !== undefined && batch.batchId !== null
      ? { batchId: String(batch.batchId), seq: Number(batch.seq) } : {}),
    overrides: params.overrides,
    inject: params.inject,
    maxSteps: params.maxSteps,
    seed: params.seed,
    targetYaw: params.targetYaw,
    // 靶板蓝红面（页面恒带 'blue'/'red'；压测/无人机体不带时 undefined ⇒ JSON 序列化自然省略）
    targetFace: params.targetFace,
    simRoot: SIM_ROOT,
    simRootSource: SIM_ROOT_SOURCE,
    configMd5: md5Of(ROBOT_FILE),
    mapMd5: md5Of(MAP_FILE),
    status: 'running',
  }
  if (lock) running = true
  try {
    const started = Date.now()
    // ---- 源覆盖 / 改点位 / 手工路线 / 无人机协同 → 运行目录 override ----
    // 所有会改变输入的分支都只写本次 run 的 map_override/：仓库 config/ 零写入。
    //   · source：把指定 robot.yaml+map.yaml 只读复制（仿真台"切到实车现图"）
    //   · field ：改四类点位（棋盘拖拽）
    //   · drone ：起降格注入 obstacles 后重规划（§D.2）
    //   · route ：手工 token 路线（不再规划；路线即 tokens）
    if (params.sourceError) return { ok: false, code: 400, error: params.sourceError }
    if (params.routeError) return { ok: false, code: 400, error: params.routeError }
    if (params.engineError) return { ok: false, code: 400, error: params.engineError }
    const source = params.source
    // 源目录自带的 route.tokens.txt = 手动路线（仅在未改点位/未开无人机时生效；改点位必须重规划）。
    const sourceRoute = source && source.routePath && !params.field && params.drone !== true
      ? { tokens: routeTokensFromFile(source.routePath), source: 'source/route.tokens.txt' }
      : null
    if (sourceRoute && sourceRoute.tokens.length === 0) {
      return { ok: false, code: 400, error: '场地来源的 route.tokens.txt 存在但没有合法 token（不静默回落重规划）' }
    }
    const manual = params.route || sourceRoute
    let configPath = null
    let routePath = null
    let env = {}
    if (source || params.field || params.drone === true || manual) {
      const overrideDir = path.join(outDir, 'map_override')
      fs.mkdirSync(overrideDir, { recursive: true })
      const baseMap = source ? source.mapPath : MAP_FILE
      const baseConfig = source ? source.configPath : ROBOT_FILE
      const mapText = fs.readFileSync(baseMap, 'utf8')
      const overrideMap = path.join(overrideDir, 'map.yaml')
      const overrideRobot = path.join(overrideDir, 'robot.yaml')
      // field 变换先做，drone 避让注入叠在其产物上（同一张图）。起降格取**本次源图**的
      //   fieldSpec().drone（真源 + fallback，禁硬编码字面量）；source 时也必须读源图，
      //   否则会把仓库第六张图的起降格注入到第七张图。
      const specNow = (params.drone === true || source)
        ? fieldSpec({ mapPath: baseMap, configPath: baseConfig })
        : null
      const droneAvoid = params.drone === true && specNow
        ? [specNow.drone?.takeoff, specNow.drone?.land].filter(Boolean)
        : []
      let overrideText = applyFieldToMapText(mapText, params.field)
      if (droneAvoid.length) overrideText = injectDroneAvoidToMapText(overrideText, droneAvoid)
      fs.writeFileSync(overrideMap, overrideText, 'utf8')
      // 配置逐字复制：load_field_spec(cfg_source=…) 会优先取同目录 map.yaml，
      //   这一对文件就是"这次跑的是哪张图"的完整证据；仓库 config/ 不写。
      fs.copyFileSync(baseConfig, overrideRobot)
      record.override = { dir: overrideDir, map: overrideMap, robot: overrideRobot }
      if (params.field) record.field = params.field
      if (source) {
        // 源与副本逐字节核对（复制期被并发改/复制坏 ⇒ 显式失败，不拿假图跑）。
        record.source = {
          dir: source.dir, configPath: source.configPath, mapPath: source.mapPath,
          configMd5: md5Of(source.configPath), mapMd5: md5Of(source.mapPath),
          routePath: source.routePath || null,
          routeMd5: source.routePath ? md5Of(source.routePath) : '',
        }
        record.source.copyConfigMd5 = md5Of(overrideRobot)
        // overrideMap 在改点位/drone 注入后**本来就应**与源图不同 ⇒ 只有"未变换"时才要求
        // 逐字节相等；变换过就记 mapEdited=true，并把源图 md5 与本次 override md5 都留档。
        const mapEdited = Boolean(params.field) || droneAvoid.length > 0
        record.source.mapEdited = mapEdited
        record.source.overrideMapMd5 = md5Of(overrideMap)
        if (record.source.copyConfigMd5 !== record.source.configMd5
          || (!mapEdited && record.source.overrideMapMd5 !== record.source.mapMd5)) {
          record.status = 'source-copy-mismatch'
          appendMeta(record)
          return { ok: false, code: 409, record, error: '场地来源副本与源文件 md5 不一致（复制期间源被改动？）' }
        }
        // 台账的 configMd5/mapMd5 = 本局**实际使用**的源（不是仓库 config/）。
        record.configMd5 = record.source.configMd5
        record.mapMd5 = record.source.mapMd5
      }
      if (manual) {
        // 手工路线：不规划；写 --route 文件。末点 ,x/,y 会被 sim_cli 的按逗号切词切成
        //   独立 token（实测 'y' 非法），所以只对**末点**剥后缀；原样 token 另存备查。
        const routeArgv = manual.tokens.slice()
        let routeNote = ''
        const last = routeArgv.length - 1
        if (last >= 0 && /,[xy]$/i.test(routeArgv[last])) {
          routeArgv[last] = routeArgv[last].replace(/,[xy]$/i, '')
          routeNote = '末点朝向后缀已从本文件剥离（--route 文件解析按逗号切词会把它切成独立 token）；'
            + '原样 token 见 route.tokens.manual.source.txt，仿真结果不受影响'
        }
        routePath = path.join(overrideDir, 'route.tokens.manual.txt')
        fs.writeFileSync(routePath, routeArgv.join('\n') + '\n', 'utf8')
        fs.writeFileSync(path.join(overrideDir, 'route.tokens.manual.source.txt'),
          manual.tokens.join(' ') + '\n', 'utf8')
        record.route = manual.tokens.slice()
        record.routeSource = source ? 'manual(field-source)' : 'manual'
        record.plan = {
          manual: true, source: manual.source, engine: null, engineSource: 'manual',
          tokenCount: manual.tokens.length, routeFile: routePath, routeFileNote: routeNote,
          crossing: null, polyline_m: null,
          reverseSegments: manual.tokens.filter(token => /^R:/i.test(token)).length,
          checkExit: null, failures: [],
        }
        configPath = overrideRobot
        env = { ROBOT_MAP: overrideMap }
      } else {
        // 规格 §B.6：路径规划计算时间（用户点名的统计项）—— Date.now 包住 plan_route。
        const planStartedAt = Date.now()
        const planned = await planRoute({ configPath: overrideRobot, mapPath: overrideMap, engine: params.engine })
        record.plan = {
          exitCode: planned.exitCode, timedOut: planned.timedOut,
          engine: planned.engine,
          engineRequested: planned.engineRequested,
          engineSource: planned.engineSource,
          ms: Date.now() - planStartedAt,
          tokenCount: planned.tokens.length,
          crossing: planned.plan?.crossing ?? null,
          polyline_m: planned.plan?.polylength_m ?? null,
          reverseSegments: planned.plan?.reverse_segments ?? null,
          checkExit: planned.check?.exit ?? null,
          failures: planned.failures,
          stderrTail: planned.stderrTail,
        }
        // drone 台账字段（§D.3）—— 规划成败都落（422 plan-failed 也在，证约束在场）。
        if (params.drone === true) {
          const boardNow = params.field ? { ...specNow, ...params.field } : specNow
          record.plan.drone = {
            enabled: true,
            hardAvoid: droneAvoid.slice(),
            corridorRow: corridorRows(boardNow),
          }
        }
        if (planned.exitCode !== 0 || planned.tokens.length === 0) {
          record.status = 'plan-failed'
          record.stderrTail = planned.stderrTail
          appendMeta(record)
          return {
            ok: false, code: 422, record,
            error: (params.field ? '改点位后' : '') + '规划/校验未过（plan_route.py exit=' + planned.exitCode + '）—— 路线没换，仿真未跑。',
            planTail: planned.stdoutTail || planned.stderrTail,
          }
        }
        routePath = path.join(overrideDir, 'route.tokens.txt')
        // 规划器 token 的末点朝向同样剥离（理由同手工路线）。
        const routeArgv = planned.tokens.slice()
        let routeNote = ''
        const last = routeArgv.length - 1
        if (last >= 0 && /,[xy]$/i.test(routeArgv[last])) {
          routeArgv[last] = routeArgv[last].replace(/,[xy]$/i, '')
          routeNote = '末点朝向后缀已从本文件剥离（--route 的文件解析按空格/逗号切词会把它切成独立 token）；'
            + '原样 token 见 route.tokens.planner.txt，仿真结果不受影响'
        }
        fs.writeFileSync(routePath, routeArgv.join('\n') + '\n', 'utf8')
        fs.writeFileSync(path.join(overrideDir, 'route.tokens.planner.txt'),
          planned.tokens.join(' ') + '\n', 'utf8')
        record.route = planned.tokens
        record.routeSource = 'planned(map_override)'
        record.plan.routeFile = routePath
        record.plan.routeFileNote = routeNote
        configPath = overrideRobot
        env = { ROBOT_MAP: overrideMap }
      }
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
    if (lock) running = false
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

