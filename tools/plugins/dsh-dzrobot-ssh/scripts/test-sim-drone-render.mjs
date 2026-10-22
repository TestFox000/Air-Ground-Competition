#!/usr/bin/env node
/**
 * test-sim-drone-render.mjs — 无人机拟真地图（卡3+）的离线核对。
 *
 * 测什么：
 *   ① 后端契约：/sim/drone/run 响应必须透传 `scan`（含 path=航线格序列、cells=每点 3×3 覆盖）
 *     与 `intel`（bridge_passable 报文数组）—— 此前 index.js 只回 timeline/reroutes/confirmed/record，
 *      前端没有画无人机航线的素材（拟真缺失的根因）。
 *   ② dronesim.scanPlan / intelAt：U 形航线点带 3×3 覆盖、情报三关采纳/拒收判定。
 *   ③ sim.html 的 renderDroneMap：给定 scan+intel，产出的 SVG 含
 *     橙航线折线 / 淡蓝扫描覆盖 / 起降标注 / 采纳绿圈 / 拒收橙圈。
 *
 * 跑法：node scripts/test-sim-drone-render.mjs → 末行「结果：N/N」，全过 exit 0。
 */
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { fieldSpec } from '../src/simbench.js'
import { scanPlan, intelAt, truthPlanEvidence } from '../src/dronesim.js'
import { FIXTURE } from './fixtures/solvable-map.mjs'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PLUGIN = path.resolve(HERE, '..')
const html = fs.readFileSync(path.join(PLUGIN, 'sim.html'), 'utf8')

let pass = 0
let fail = 0
const check = (name, ok, note = '') => {
  if (ok) pass += 1
  else fail += 1
  console.log(`  ${ok ? 'PASS' : 'FAIL'} ${name}${note ? ' —— ' + note : ''}`)
}

/** 按花括号配平提取一个顶层函数声明（函数体内的花括号都配平，depth 计数可靠）。 */
function extractFn(src, name) {
  const start = src.indexOf(`function ${name}(`)
  if (start < 0) return ''
  const open = src.indexOf('{', start)
  let depth = 0
  for (let i = open; i < src.length; i++) {
    if (src[i] === '{') depth++
    else if (src[i] === '}') { depth--; if (depth === 0) return src.slice(start, i + 1) }
  }
  return ''
}

// ── ① 后端契约：index.js / dronesim.js 透传 scan/intel ─────────────
{
  const idx = fs.readFileSync(path.join(PLUGIN, 'index.js'), 'utf8')
  check('①-1 index.js 透传 scan 字段（拟真素材）', /scan:\s*result\.scan/.test(idx))
  check('①-2 index.js 透传 intel 字段', /intel:\s*result\.intel/.test(idx))
  const drone = fs.readFileSync(path.join(PLUGIN, 'src', 'dronesim.js'), 'utf8')
  check('①-3 dronesim 返回 scan.path（航线格序列）', /path:\s*plan\.points\.map\(p => p\.cell\)/.test(drone))
  check('①-4 dronesim 返回 scan.cells（每点 3×3 覆盖）', /cells:\s*plan\.points\.map\(p => \(\{ cell: p\.cell, covers: p\.covers \}\)\)/.test(drone))
}

// ── ② dronesim.scanPlan / intelAt 纯函数 ─────────────────────────────
{
  const field = fieldSpec()
  check('②-1 读到场地真源（config/map.yaml）', field.ok, field.error || '')
  const plan = scanPlan({ field })
  check('②-2 航线非空（U 形 ≥ 起降两点）', Array.isArray(plan.points) && plan.points.length >= 2,
    `${plan.points.length} 点`)
  check('②-3 起降点 == 场地 drone 真源', plan.takeoff === field.drone.takeoff && plan.land === field.drone.land,
    `${plan.takeoff}→${plan.land}`)
  check('②-4 每点带 3×3 覆盖（1..9 格，边缘裁剪）',
    plan.points.every(p => Array.isArray(p.covers) && p.covers.length >= 1 && p.covers.length <= 9),
    '覆盖 1..9 格')
  // 情报：truth = 场地 bridges（K5/K9），应三关全过 = adopted
  const intel = intelAt({ plan, truthBridges: field.bridges, field })
  const bridgeMsgs = intel.filter(m => m.type === 'bridge_passable')
  check('②-5 桥洞情报按半场产出（上/下各 ≥1）',
    bridgeMsgs.filter(m => m.half === 'lower').length >= 1 && bridgeMsgs.filter(m => m.half === 'upper').length >= 1,
    JSON.stringify(bridgeMsgs.map(m => m.half + ':' + m.block)))
  check('②-6 场地真值桥洞 = adopted（三关全过）',
    bridgeMsgs.every(m => m.decision === 'adopted'),
    JSON.stringify(bridgeMsgs.map(m => m.decision)))
  // 拒收反例①（第①关：非 K 列）：truth=J9 —— 落在 K9 穿越点的 3×3 覆盖内，但 J 非 K 列 ⇒ rejected
  const bad1 = intelAt({ plan, truthBridges: ['J9'], field })
  const bad1Bridge = bad1.filter(m => m.type === 'bridge_passable')
  check('②-7 非 K 列桥洞情报被拒（第①关：J9 在覆盖内但非 K 列）',
    bad1Bridge.some(m => m.decision === 'rejected' && /非 K 列/.test(m.reason)),
    JSON.stringify(bad1Bridge[0] || null))
  // 拒收反例②（第③关：不在候选集）：truth=K8 —— K 列、lower 半场，但候选只 [K9] ⇒ rejected
  const bad2 = intelAt({ plan, truthBridges: ['K8'], field })
  const bad2Bridge = bad2.filter(m => m.type === 'bridge_passable')
  check('②-8 候选集外桥洞被拒（第③关：K8 ∉ [K9]）',
    bad2Bridge.some(m => m.decision === 'rejected' && /候选/.test(m.reason)),
    JSON.stringify(bad2Bridge[0] || null))
}

// ── ③ sim.html renderDroneMap 渲染 ──────────────────────────────────
{
  const field = fieldSpec()
  const plan = scanPlan({ field })
  const scan = {
    takeoff: plan.takeoff, land: plan.land,
    path: plan.points.map(p => p.cell),
    cells: plan.points.map(p => ({ cell: p.cell, covers: p.covers })),
  }
  const intel = intelAt({ plan, truthBridges: field.bridges, field })
  const carRecord = { route: ['N10', 'M10', 'L10', 'L8', 'M8', 'N8'] }

  const script = html.match(/<script>([\s\S]*?)<\/script>/)[1]
  // renderDroneMap 现在依赖底图/折线/格名等函数 —— 一并提取进同一 with 环境
  const depNames = ['droneBoardSvg', 'routePolyline', 'tokenCell', 'targetBoardRect', 'colIdxOf', 'renderDroneMap']
  const fnSrc = depNames.map(n => extractFn(script, n)).filter(Boolean).join('\n')

  // DOM 桩 + with/Proxy：只让 renderDroneMap 需要的自由变量落地（不执行整段脚本）
  const els = {}
  const el = id => els[id] || (els[id] = { id, value: '', checked: false, innerHTML: '', style: {}, textContent: '' })
  const cellCenter = cell => {
    const g = field.grid
    const col = g.colLetters.indexOf(String(cell)[0].toUpperCase())
    const row = Number(String(cell).slice(1))
    if (col < 0 || !(row >= 1 && row <= g.rows)) return null
    return [col * 34 + 17, (row - 1) * 34 + 17]
  }
  const state = {
    // with+Proxy 的 has 恒真会把内置对象也劫持成 undefined —— 显式播种（Array.isArray 会被调用）
    Array, JSON, Math, String, Number, Boolean, isFinite, Object,
    FIELD: field, MAP_S: 34, FIELD_GRID_BLOCK: 0.35, $: el,
    FIELD_CELL_RE: /^[A-N](?:[1-9]|1[01])$/,
    esc: v => String(v == null ? '' : v).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])),
    cellCenter,
  }
  check('③-1 renderDroneMap 及其依赖函数可提取（函数源码非空）', fnSrc.length > 800, fnSrc.length + ' B')
  const ctx = new Proxy(state, { has: () => true, get: (t, k) => t[k], set: (t, k, v) => { t[k] = v; return true } })
  const renderDroneMap = new Function('ctx', `with (ctx) { ${fnSrc}\n; return renderDroneMap; }`)(ctx)
  check('③-2 renderDroneMap 是函数', typeof renderDroneMap === 'function')

  renderDroneMap({ scan, intel, record: carRecord })
  const svg = el('droneMap').innerHTML
  check('③-3 产出 SVG（非空）', /<svg/.test(svg) && svg.length > 400, svg.length + ' B')
  check('③-4 航线橙折线（虚线 stroke=#f08c00）', /stroke="#f08c00"/.test(svg) && /stroke-dasharray/.test(svg))
  check('③-5 扫描覆盖（淡蓝半透明 rect）', /fill="#4d8fd6" fill-opacity="0.12"/.test(svg))
  check('③-6 无人机起降标注（「机」+「H」）', svg.includes('>机<') && svg.includes('>H<'))
  check('③-7 采纳绿圈（#38c172 + 「采纳」）', /#38c172/.test(svg) && svg.includes('采纳'))
  check('③-8 扫描大虚线圆（circle + stroke-dasharray + #4d8fd6）',
    /<circle[^>]*stroke-dasharray="5 4"/.test(svg) && /stroke="#4d8fd6"/.test(svg))
  check('③-9 小车路线绿折线（stroke=#2f9e44）', /stroke="#2f9e44"/.test(svg))
  check('③-10 底图棋盘格（14×11 rect + 隔板标注）',
    (svg.match(/stroke="#c9ced8"/g) || []).length === field.grid.cols * field.grid.rows
    && svg.includes('隔板'))

  // 拒收反例走一遍：确认「拒收」字与橙圈真的会被画（truth=J9 → 非 K 列拒收）
  const badIntel = intelAt({ plan, truthBridges: ['J9'], field })
  renderDroneMap({ scan, intel: badIntel, record: carRecord })
  const svgBad = el('droneMap').innerHTML
  check('③-11 拒收情报产出「拒收」标注 + 橙圈', svgBad.includes('拒收') && /#f0b429/.test(svgBad))
}

// ── ④ checkpoint 增量交付（决策 #1/#3/#4 的新语义守卫）─────────────────
{
  const field = fieldSpec()
  const plan = scanPlan({ field })
  // 4-1 桥洞优先短程航线：≤4 直线段（= 方向改变 ≤3 次，实际 2 次：横→竖→横）
  // 拐弯 = 相邻两段方向不同（正交步进下，从横飞转竖飞 / 竖飞转横飞各算 1 次）
  let turns = 0
  for (let i = 1; i < plan.points.length - 1; i++) {
    const a = plan.points[i - 1].cell, b = plan.points[i].cell, c = plan.points[i + 1].cell
    const dir = (x, y) => (x[0] === y[0] ? 'v' : 'h')   // 同列=竖飞，同行=横飞
    if (dir(a, b) !== dir(b, c)) turns += 1
  }
  check('④-1 桥洞优先短程：方向改变 ≤3（≤4 直线段）', turns <= 3, `${turns} 拐弯 / ${turns + 1} 段`)
  // 4-2 checkpoint 含 lower/upper/land 三交付点
  const cps = plan.checkpoints || []
  const ids = cps.map(c => c.id)
  check('④-2 checkpoint 含 lower + upper + land', ids.includes('lower') && ids.includes('upper') && ids.includes('land'), JSON.stringify(ids))
  // 4-3 桥洞优先：lower 桥洞在起飞后最早交付（其 checkpoint 索引 < upper）
  const lowerCp = cps.find(c => c.id === 'lower')
  const upperCp = cps.find(c => c.id === 'upper')
  check('④-3 信息最快：lower 先于 upper 交付', lowerCp && upperCp && lowerCp.index < upperCp.index,
    `lower@${lowerCp && lowerCp.index} upper@${upperCp && upperCp.index}`)
  // 4-4 桥洞在第一个 K 列段（起飞后直达 K 列）就被覆盖：lower 的 index 小（≤ 起飞到 K 列的距离）
  const kColIdx = 'ABCDEFGHIJKLMN'.indexOf('K')
  const takeoffColIdx = 'ABCDEFGHIJKLMN'.indexOf(field.drone.takeoff[0])
  const seg1Len = Math.abs(kColIdx - takeoffColIdx)
  check('④-4 桥洞段①内拿到 lower 情报（≤ 段①长度）', lowerCp && lowerCp.index <= seg1Len,
    `lower@${lowerCp && lowerCp.index} ≤ 段①${seg1Len}`)
  // 4-5 intel 带增量字段（checkpoint + t）
  const intel = intelAt({ plan, truthBridges: field.bridges, field })
  const bridgeMsgs = intel.filter(m => m.type === 'bridge_passable')
  check('④-5 桥洞情报带 checkpoint + t（增量交付）', bridgeMsgs.every(m => m.checkpoint && typeof m.t === 'number'),
    JSON.stringify(bridgeMsgs.map(m => m.half + '@' + m.checkpoint + ':' + m.t)))
}

// ── ⑤ truthPlanEvidence 运行期守卫（t6 崩因：robotPath 块级作用域越界 → ReferenceError）───
//   ★ 2026-10-17 t6 实跑发现：plannerEngineOf(robotPath) 引用 try{} 内的 const ⇒ 每次取证必崩，
//     而此前守卫只做静态断言不跑该分支。这里真跑一次取证：ReferenceError / exit≠0 / 无 token 全红。
{
  const field = fieldSpec()
  check('⑤-0 前置：场地可读', field.ok, field.error || '')
  // 第六张图：车端 tracker-safe 无解（planner.auto_plan_enabled=false 已记录）⇒ 用引擎可解夹具的
  //   四类点位覆盖 board —— 本段守卫的是"truthPlanEvidence 运行期不崩 + 真产出 token"，
  //   换图只换"这张图能不能解"，argv/引擎链路不变。现场图的显式失败口径由 parity 2.x 守卫。
  const board = field.planner && field.planner.autoPlanEnabled === false
    ? { ...field, ...FIXTURE } : field
  if (board !== field) console.log('  INFO 第六张图 tracker-safe 无解 ⇒ truthPlanEvidence 用夹具 board（同一条 argv 链路）')
  const evidence = await truthPlanEvidence({
    board, truth: board.bridges, id: `t6_guard_${Date.now()}`, label: 'guard', drone: false,
  })
  check('⑤-1 truth 取证运行期 exit 0（无 ReferenceError）',
    evidence.exitCode === 0 && !/ReferenceError/.test(evidence.stderrTail),
    `exit=${evidence.exitCode} stderr=${JSON.stringify(evidence.stderrTail.slice(0, 80))}`)
  check('⑤-2 truth 取证产出非空 token（同引擎 plan_route 实跑）',
    Array.isArray(evidence.tokens) && evidence.tokens.length > 0, `${evidence.tokenCount} tokens`)
}

console.log('')
console.log(`结果：${pass}/${pass + fail} ${fail === 0 ? '全部通过 ✅' : '存在失败 ❌'}`)
process.exit(fail === 0 ? 0 : 1)
