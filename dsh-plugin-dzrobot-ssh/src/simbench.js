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
/** 场地列字母（`core/geometry.py:27-30` 的 COL_LETTERS）—— **插件进程内唯一字面量**。
 *  ★ t17：dronesim 的 `LETTERS`、stressbench 的 `COLS` 此前各抄一份 `'ABCDEFGHIJKLMN'`；
 *    现统一从这里取（页面侧不 import 模块，走 `/sim/map` 的 `grid.colLetters`，跨进程不共享）。 */
export const COL_LETTERS = 'ABCDEFGHIJKLMN'
/** 场地网格（COL_LETTERS / MAP_COLS=14 / MAP_ROWS=11 / BLOCK_SIZE=0.35）。 */
const GRID = { cols: 14, rows: 11, blockM: 0.35, colLetters: COL_LETTERS }
/** 页面允许编辑的四类点位（t6：+targets —— 其余要素保持现场值）。 */
const FIELD_KEYS = ['mines', 'obstacles', 'bridges', 'targets']
/** 无人机起/降点：map.yaml 的 t28 识图注释里写着；读不到时用这两个（蓝 + H 图标）。
 *  ★ t17：dronesim 此前另抄一份同值兜底（注释还写着"与 simbench 内建兜底同值"）——
 *    现由本常量单点供给，改兜底只改这里。 */
export const DRONE_FALLBACK = { takeoff: 'N8', land: 'M2' }
/** 单格名正则（A~N 列 × 1~11 行），`normalizeCell` 与 stressbench 的图生成共用一份。 */
export const CELL_RE = /^[A-N](?:[1-9]|1[01])$/
const FIELD_MAX_CELLS = 40

/** ★ t28：`fieldSpec()` 单次调用内解析出的靶格（`targetBoard()` 从它判横竖）。 */
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
  TARGET_CACHE.cells = normalizeCells(listOf(text, 'targets'))
  return {
    ok: true,
    root: SIM_ROOT,
    source: MAP_FILE,
    md5: md5Of(MAP_FILE),
    robot: { path: ROBOT_FILE, md5: md5Of(ROBOT_FILE) },
    // ★ t15：桥洞进洞侧冗余墙长度（米）——真源 config/robot.yaml `field.bridge_entry_redundancy_m`
    //   （bridgeEntryRedundancyOf 只读解析 dzbrain 段；缺键/读不到兜底 0.0875，与
    //   planner.BRIDGE_ENTRY_REDUNDANCY_M / physics.bridge_entry_walls 同源链）。
    //   /sim/map 透传给 sim.html 画 K 列桥洞格进洞侧两段短墙——页面端零第二份硬编码。
    bridgeEntryRedundancyM: bridgeEntryRedundancyOf(ROBOT_FILE),
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
      takeoff: droneCells[0] || DRONE_FALLBACK.takeoff,
      land: droneCells[1] || DRONE_FALLBACK.land,
      source: droneCells.length >= 2 ? 'map-comment' : 'builtin-fallback',
    },
    // ★ t28：靶板由 targets 的相邻两格并成一块；按两格几何关系自动判横/竖（页面可覆盖）。
    target: targetBoard(),
    editable: FIELD_KEYS,
  }
}

/**
 * 靶板几何（t28）：把 `map.targets` 的格名按**屏幕坐标**判横竖。
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

/** ★ t3（规格 §D.2）：无人机协同 —— 把无人机起降格注入 override 图的 obstacles 列表。
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
  // ★ `m` 标志不能少：obstacles 行在 map.yaml 中部，^$ 不按行锚就会整串匹配失败（误抛）。
  const pattern = /^([ \t]*)obstacles:[ \t]*\[[^\]]*\][ \t]*$/m
  if (!pattern.test(raw)) throw new Error('地图里没有 obstacles: [...] 这一行，拒绝注入（避免写出坏地图）')
  return raw.replace(pattern, `$1${line}`)
}

/** ★ t3（规格 §D.3）：无人机扫描**走廊行**（上半场 top / 下半场 bottom，1-based 行号）。
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
 *  --engine"，于是又退回 `plan_route.py` 的默认 `tour` —— 也就是 t73「乱算」原样复发，
 *  而且**不报错、看不出来**。真源只有一个：CLI 的 argparse 定义。
 *  读不到 CLI / 形状认不出（改写成变量拼装等）→ 兜底用历史上的两项，行为与改动前逐字节
 *  一致（既不会少传也不会给 argparse 塞未知值）。进程内缓存一次（CLI 是只读文件，运行期
 *  不变；真改了要重启宿主，与此前 `PLAN_CLI` 常量同口径）。 */
const ENGINE_FALLBACK_CHOICES = ['tour', 'tracker-safe']
let engineChoicesCache = null
function plannerEngineChoices() {
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
 *  ★ 修「乱算」：页面重规划此前不传 --engine ⇒ plan_route.py 默认 tour，而车端 robot.yaml
 *    配的是 tracker-safe —— 同一张图页面和车算出**两条不同的路线**。实测两次快照
 *    （`--map config/map.yaml` + `--check`，robot.yaml md5 见括注）：
 *      · md5 `4FFC53D6…`（t3 改射击距离之前）：tracker-safe **44 token**
 *        `N10 M10 M9 L9 K9 … F3* … N5,y`（与 `mission_blocks` 逐 token 相等）
 *        ｜不传 --engine（改前页面行为）**40 token** `N10 N9 M9 … C7 B7 A7 … H5 …`
 *      · md5 `23DBE758…`（t3 改完 shoot_distance 1.00 之后）：tracker-safe **50 token**
 *        ｜不传 --engine **49 token，且带 2 处 `:PIVOT` 折返**（`A2:PIVOT`/`B1:PIVOT`）
 *    ⇒ 结论与 token 数无关：**只有跟车端同引擎才逐 token 相等**。
 *    这里以车端配置为唯一真源：配什么引擎，页面就传什么。
 *  形状：`planner:` 块（与 route:/mission: 同级）下的 `engine: tracker-safe`，允许行内注释与
 *  YAML 引号（`engine: "tracker-safe"` 也认）。
 *  找不到 / 值不在 plan_route.py 的 choices 里 → 返回 null（不传 --engine，保持其历史
 *  默认 tour，与改动前逐字节一致）。导出仅供离线断言（同 sanitizeParams 的先例）。
 *
 *  ✅ 已闭环（2026-10-17 队长直改）：`src/dronesim.js` 的 `truthPlanEvidence()` 已复用
 *    本函数拼 `--engine`（同引擎取证，无人机协同时间线与车端口径一致）；t12 起它的
 *    `from_cell` / `already_covered` 也走同一条白名单链路（`appendIncrementalArgs()`）。 */
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
        // 去 YAML 引号（`engine: "tour"` 与 `engine: tour` 同义；带引号时旧实现会误判未知）
        const value = hit[1].replace(/^['"]|['"]$/g, '')
        return plannerEngineChoices().includes(value) ? value : null
      }
    }
  } catch { /* 配置读不到 → 走 plan_route 默认 */ }
  return null
}

/* ── ★ t12：增量规划透传（`from_cell` / `already_covered`）────────────────────
 *
 * 车端 t9 在 **planner 层**加了这两个入参（`plan_route_tracker_safe(..., from_cell=, already_covered=)`）；
 * 仿真台要透传上去，口径与 `--engine` **完全同链路**：
 *   ① 真源 = `tools/plan_route.py` 的 argparse（`plannerIncrementalSupport()` 现场读，不抄第二份）；
 *   ② **声明了才传**，一个字节不多传 —— 给 argparse 塞未知参数会 exit 2，页面侧只表现为
 *      "改点位后规划失败（422）"，比不传更难查；
 *   ③ 缺省不传 ⇒ argv 与改动前**逐字节相同**（整线规划语义不变）。
 *
 * ⚠ 状态史（2026-10-17）：t12 开工时 `tools/plan_route.py` 里**还没有**这两条 add_argument
 *   （t9 的交付只到 `planner.py` + `tests/test_incremental_plan.py`），当时透传按 ② 如实降级为
 *   "不传"、台账 `incremental.dropped` 记诉求（不静默吞），E2E 由守卫的影子 CLI 钉住。
 *   t12 收口时队长已给车端 CLI 接线：现役 CLI `supported = 2/2`，守卫 E 段自动转实跑。
 *   若未来接线被回退，`plannerIncrementalSupport()` 自动变空，降级路径原样生效。
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

/** ★ t15：`field.bridge_entry_redundancy_m` 的缺省兜底（米）。
 *  与 `route/planner.py` 的兜底常量 `BRIDGE_ENTRY_REDUNDANCY_M` **同源同值**
 *  （0.0875 = 0.35 × 1/4 格，用户 2026-10-16 设计规格）；真源是 config/robot.yaml。 */
const BRIDGE_ENTRY_REDUNDANCY_FALLBACK_M = 0.0875

/** ★ t15：robot.yaml 里 dzbrain 段 `field:` 块下的 `bridge_entry_redundancy_m`
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
 *  config 时同口径）。`0.0` 是**合法值**（= 关冗余，回到 t4 之前的行为），只有**取不到值**
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
 *    见本卡交付的回归输出：44 ↔ 44 逐 token 相等。
 *
 *  ★ t12 入参：`fromCell`（增量续算起点）/ `alreadyCovered`（已压格，只对雷生效）——
 *    车端 t9 的 planner 入参，按 `appendIncrementalArgs` 白名单化后追加到 `--engine` 之后；
 *    缺省 `null/[]` ⇒ argv 与改动前**逐字节相同**（整线规划语义不变）。
 *    `cliPath` 缺省 = 工作区 `tools/plan_route.py`；仅在离线断言里指向**影子 CLI**（%TEMP% 副本），
 *    用来在车端还没接线时钉住"声明后真的会传、产物 == 车端 planner"这条 E2E。 */
export async function planRoute({ configPath, mapPath, timeoutMs = 300 * 1000, fromCell = null, alreadyCovered = [], cliPath = PLAN_CLI }) {
  const argv = [cliPath === PLAN_CLI ? 'tools/plan_route.py' : cliPath,
                '--config', configPath, '--map', mapPath,
                '--emit', 'json', '--check', '--quiet']
  // ★ 修「乱算」：与车端同引擎（robot.yaml planner.engine；读不到就不传，保持历史默认）。
  const engine = plannerEngineOf(configPath)
  if (engine) argv.push('--engine', engine)
  // ★ t12：增量规划透传（from_cell / already_covered）——**声明了才传**，缺省逐字节不变。
  //   与 --engine 同链路：先引擎、后增量，白名单来自 CLI 真源，诉求/降级都进台账。
  const incremental = appendIncrementalArgs(argv, { fromCell, alreadyCovered }, { cliPath }).applied
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
    // ★ t12：取证用（离线断言/台账都要能回答"这一趟到底带了哪些参数"）
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
 * 参数快照（用于台账，与 run 记录一起存）。
 *
 * ★ 导出**仅供离线断言**使用（`scripts/test-sim-map-render.mjs` 的「E 透传链②」一组）：
 *   它不是插件的对外 API，`index.js` 不引用它，`runSim()` 是唯一的真实调用方。
 *   理由：`targetYaw`（t28 靶板朝向）从表单到台账的链条里，只有这一段是**纯函数**，
 *   能不启服务、不跑 python 就被断言钉住；「record 落盘」那一段由真跑取证覆盖
 *   （5 跑实测：run_meta.jsonl 落 `"targetYaw":"v"/"h"`、非法值落 null，listHistory 原样回读）。
 */
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
  // ★ t25：seed 留空 = sim_cli 的 SimConfig 默认（不传 `--seed`，与改动前逐字一致）。
  const seed = Number.isInteger(body.seed) && body.seed >= 0 ? body.seed : null
  // ★ t28：靶板朝向（'h' 横向 / 'v' 竖向 / '' 跟随地图）——**只用于渲染与台账**，
  //   不写 config、不进 sim_cli 参数（仿真物理不感知靶板朝向）。
  const targetYaw = body.targetYaw === 'h' || body.targetYaw === 'v' ? body.targetYaw : null
  // ★ t14：靶板蓝红面（'blue' | 'red'）——仿 targetYaw 先例：只用于渲染与台账，
  //   不写 config、不进 sim_cli 参数。与 targetYaw 的差别只有一处：targetYaw 恒在返回
  //   键表里（t28 先例，scripts/test-sim-map-render.mjs E 组钉死 sp({}) 恰 7 键），
  //   targetFace 仅在请求体**带了该键**时才挂进返回对象 —— 页面 readForm 恒带 ⇒ 真跑
  //   台账恒有该字段；压测/无人机体不带 ⇒ 记录形状与改动前逐字一致。
  const targetFace = body.targetFace === 'blue' || body.targetFace === 'red' ? body.targetFace : null
  // ★ t3（规格 §D）：无人机协同开关 —— 仅 `true` 通过（false/字符串/其它一律 null 丢弃），
  //   **带键才挂**（targetFace 同款先例）：页面不勾时请求体不带 drone 键 ⇒ params 无该键，
  //   sp({}) 键表保持 t28 的 7 键（守卫 E 组钉死）、台账形状与 OFF 现状逐字一致（§D.1）。
  //   开启语义 = 车路线硬避让无人机起降格：runSimJob 里经 override 图 obstacles 注入 +
  //   planRoute 重规划（插件层唯一硬禁行键），不进 sim_cli 参数（仿真物理不感知）。
  const drone = body.drone === true ? true : null
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
  const params = { overrides, inject, label, maxSteps, seed, targetYaw, field: fieldTouched ? field : null }
  // ★ t14：带键才挂 —— sp({}) 的键表保持 t28 的 7 键不变（守卫 E 组钉死），不破 61/61。
  if (body.targetFace !== undefined) params.targetFace = targetFace
  // ★ t3：同款带键才挂 —— 不带 drone 键的请求（压测/无人机体/OFF）params 形状不变。
  if (body.drone !== undefined) params.drone = drone
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

/** 追加一条台账记录（导出给 stressbench：压测自产的 invalid/error 记录进同一 run_meta.jsonl）。 */
export function appendMeta(record) {
  fs.mkdirSync(ARTIFACT_DIR, { recursive: true })
  fs.appendFileSync(META_FILE, `${JSON.stringify(record)}\n`, 'utf8')
}

/** 台账尾部记录（`run_meta.jsonl`）：`kind` 给了就只留该类的最近 50 条。
 *  ★ t17：`droneHistory()` 原先自己重抄一遍「读文件→逐行 parse→按 kind 过滤→slice(-50)」，
 *    现统一走这里（`listHistory` 不带 kind、`droneHistory` 带 `kind:'drone'`）。 */
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
    // ★ t14：靶板蓝红面（页面恒带 'blue'/'red'；压测/无人机体不带时 undefined ⇒ JSON 序列化自然省略）
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
    // ---- ★ t25：改点位 → 运行目录里生成 override（地图 + 配置副本 + 重规划路线）----
    // ★ t3（§D.2）：无人机协同也走 override 路径 —— drone 开 ⇒ 起降格注入 override 图
    //   obstacles 后重规划（规划层硬避让），哪怕棋盘没改。OFF（无 drone 键且无 field）
    //   ⇒ 分支不进，record 形状/argv/行为与现状逐字一致（§D.1）。
    let configPath = null
    let routePath = null
    let env = {}
    if (params.field || params.drone === true) {
      const overrideDir = path.join(outDir, 'map_override')
      fs.mkdirSync(overrideDir, { recursive: true })
      const mapText = fs.readFileSync(MAP_FILE, 'utf8')
      const overrideMap = path.join(overrideDir, 'map.yaml')
      const overrideRobot = path.join(overrideDir, 'robot.yaml')
      // ★ t3：field 变换（棋盘改点）先做，drone 避让注入**叠在其产物上**（同一张图）。
      //   起降格取 fieldSpec().drone（真源 + fallback，禁硬编码字面量）；仅 drone 开才多读
      //   一次 fieldSpec（field-only 跑与 OFF 现状零额外开销）。顺序保持 [takeoff, land] ——
      //   规格增补 §D.3 的台账形状 hardAvoid: ["N8","M2"]（起在前落在后；两格必异，无需排序去重）。
      const specNow = params.drone === true ? fieldSpec() : null
      const droneAvoid = specNow
        ? [specNow.drone?.takeoff, specNow.drone?.land].filter(Boolean)
        : []
      let overrideText = applyFieldToMapText(mapText, params.field)
      if (droneAvoid.length) overrideText = injectDroneAvoidToMapText(overrideText, droneAvoid)
      fs.writeFileSync(overrideMap, overrideText, 'utf8')
      // 配置**逐字复制**（只读真源的副本）：`load_field_spec(cfg_source=…)` 会优先取同目录的
      // map.yaml，所以这一对文件放在同一目录就是"这次跑的是哪张图"的完整证据。
      fs.copyFileSync(ROBOT_FILE, overrideRobot)
      // ★ 规格 §B.6：路径规划计算时间（用户点名的统计项）—— Date.now 包住 plan_route，
      //   手动带 field 的跑同样落 record.plan.ms（压测均值分母 = 有该字段的记录数）。
      const planStartedAt = Date.now()
      const planned = await planRoute({ configPath: overrideRobot, mapPath: overrideMap })
      if (params.field) record.field = params.field        // ★ t3：drone-only 跑不落 null field 键
      record.override = { dir: overrideDir, map: overrideMap, robot: overrideRobot }
      record.plan = {
        exitCode: planned.exitCode, timedOut: planned.timedOut,
        engine: planned.engine,                     // ★ 本局用的规划引擎（与车端同源的取证字段）
        ms: Date.now() - planStartedAt,
        tokenCount: planned.tokens.length,
        crossing: planned.plan?.crossing ?? null,
        polyline_m: planned.plan?.polylength_m ?? null,
        reverseSegments: planned.plan?.reverse_segments ?? null,
        checkExit: planned.check?.exit ?? null,
        failures: planned.failures,
        stderrTail: planned.stderrTail,
      }
      // ★ t3（§D.3）：drone 台账字段 —— **规划成败都落**（422 plan-failed 也在，证约束在场）。
      //   hardAvoid = 实际注入 override 图 obstacles 的格（[takeoff, land] 保序）；corridorRow =
      //   本局有效棋盘（真源 + 用户改点，与 dronesim.runDroneSim 的 board 同构）经 corridorRows
      //   推导 —— **全仓恰一份**（外来基线重写后 scanPlan 已不推导走廊行，本函数是唯一真源；
      //   规格「边界与已知取舍」：走廊规则变 ⇒ 只改这里，台账随之）。
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
          error: `改点位后规划/校验未过（plan_route.py exit=${planned.exitCode}）—— 路线没换，仿真未跑。`,
          planTail: planned.stdoutTail || planned.stderrTail,
        }
      }
      routePath = path.join(overrideDir, 'route.tokens.txt')
      // ★ `--route` 的文件解析按空格/逗号切词，末点的朝向后缀 `,x`/`,y` 会被切成独立 token
      //   （实测 `❌ 无法解析的 token: 'y'`）。仿真对末点朝向不敏感（剥掉后缀后 traj.csv
      //   与带后缀**逐字节相同**，t25 实测），所以给 `--route` 的那份剥掉后缀，
      //   规划器原样 token 另存一份备查。
      const routeArgv = planned.tokens.slice()
      let routeNote = ''
      const last = routeArgv.length - 1
      if (last >= 0 && /,[xy]$/i.test(routeArgv[last])) {
        routeArgv[last] = routeArgv[last].replace(/,[xy]$/i, '')
        routeNote = '末点朝向后缀已从本文件剥离（--route 的文件解析按空格/逗号切词会把它切成独立 token）；'
          + '原样 token 见 route.tokens.planner.txt，仿真结果不受影响'
      }
      fs.writeFileSync(routePath, `${routeArgv.join('\n')}\n`, 'utf8')
      fs.writeFileSync(path.join(overrideDir, 'route.tokens.planner.txt'),
        `${planned.tokens.join(' ')}\n`, 'utf8')
      record.route = planned.tokens
      record.routeSource = 'planned(map_override)'
      record.plan.routeFile = routePath
      record.plan.routeFileNote = routeNote
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

