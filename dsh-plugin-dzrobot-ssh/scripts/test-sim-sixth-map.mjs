#!/usr/bin/env node
/* scripts/test-sim-sixth-map.mjs —— 「本轮实车地图（第六张图）」与仿真台的逐格对齐守卫。
 *
 * 为什么需要：仿真台是"读 config/map.yaml 就画"的数据驱动页面 —— 换图后页面不会自己报错，
 *   只会画成上一张图/漏点位。本脚本把**本轮第六张图的要素**钉成期望值，并同时核对：
 *     ① fieldSpec（/sim/map 的真源）逐格 == 本轮地图（雷8/障8/靶 D5+D6 纵置/桥洞 K5+K7）；
 *     ② renderMap 的 SVG **按坐标**画出这 20 个点位（不是只数个数）、隔板/起终点/无人机点；
 *     ③ 车端手动 route.tokens（本轮 auto_plan_enabled=false）的物理口径：
 *        不压障/不穿靶板/不横穿隔板（跨行 6↔7 只允许 A/B/C 列）/8 颗雷全覆盖/先 K7 后 K5；
 *     ④ 页面如实显示"车端手动路线 + 规划引擎"（第六张图 tracker-safe 无解，改点位会 422，
 *        操作员必须能从页面看出这个状态，而不是以为"改了没生效"）；
 *     ⑤ 无 GPU 假开关（仿真链是 CPU 确定性计算）。
 *
 * ★ 快照纪律：本文件只对 md5 == 第六张图快照生效；map.yaml 换代后自动降级为
 *   「通用不变量」（渲染按 fieldSpec 动态对账 / 路线物理口径），并打印 SKIP 提示，
 *   不会因为换图产生假红 —— 换图时应把 SIXTH 常量与期望点位一起更新。
 *
 * 跑法：node scripts/test-sim-sixth-map.mjs
 *   退出码：0 = 全绿；1 = 断言失败。
 * 写域：只读（config/map.yaml、config/robot.yaml、sim.html、src/simbench.js）；不跑 python、不连车。
 */
import crypto from 'node:crypto'
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PLUGIN = path.resolve(HERE, '..')
const HTML = path.join(PLUGIN, 'sim.html')

const pass = { ok: 0, fail: 0, skip: 0 }
const check = (name, ok, note = '') => {
  if (ok) pass.ok += 1
  else pass.fail += 1
  console.log(`  ${ok ? 'PASS' : 'FAIL'} ${name}${note ? ' —— ' + note : ''}`)
}
const skip = (name, why) => { pass.skip += 1; console.log(`  SKIP ${name}${why ? ' —— ' + why : ''}`) }
const h = (t) => console.log(`\n\u001b[36m${t}\u001b[0m`)

const sb = await import('file:///' + path.join(PLUGIN, 'src', 'simbench.js').replace(/\\/g, '/'))
const { fieldSpec, routeTokens } = sb
const field = fieldSpec()
if (!field.ok) {
  console.log('FAIL 读不到场地（config/map.yaml）：' + field.error)
  process.exit(1)
}
const md5 = crypto.createHash('md5').update(fs.readFileSync(field.source)).digest('hex')
const SIXTH = '2d2f658daceedc7fb04e080fa5458918'
const isSixth = md5 === SIXTH
const rawHtml = fs.readFileSync(HTML, 'utf8')
const body = rawHtml.match(/<script>([\s\S]*?)<\/script>/)[1]

/* 第六张图期望值（用户 2026-10-17 现场图；config/map.yaml 头注释同源） */
const EXPECT = {
  mines: ['A2', 'B10', 'D11', 'E8', 'F4', 'G5', 'H4', 'I11'],
  obstacles: ['A8', 'B1', 'C9', 'D1', 'E9', 'F3', 'H7', 'I6'],
  targets: ['D5', 'D6'],
  bridges: ['K5', 'K7'],
  start: 'N10',
  end: 'N5',
  barrier: { colFrom: 'D', rowHi: 6, rowLo: 7 },
  drone: { takeoff: 'N8', land: 'M2' },
}
const sortCells = list => list.slice().sort((a, b) => (a[0] === b[0] ? Number(a.slice(1)) - Number(b.slice(1)) : a[0].localeCompare(b[0])))
const sameSet = (a, b) => JSON.stringify(sortCells(a)) === JSON.stringify(sortCells(b))

/* ── ① 真源逐格 ─────────────────────────────────────────────────── */
h('① fieldSpec（/sim/map 真源）逐格对齐')
console.log('    map.yaml md5 =', md5, isSixth ? '（本轮第六张图快照）' : '（不是第六张图快照 ⇒ 只跑通用不变量）')
check('①-0 场地可读', field.ok === true, field.error || '')
if (isSixth) {
  check('①-1 雷 8 格 == 本轮地图', sameSet(field.mines, EXPECT.mines), field.mines.join(' '))
  check('①-2 障 8 格 == 本轮地图', sameSet(field.obstacles, EXPECT.obstacles), field.obstacles.join(' '))
  check('①-3 靶 = D5+D6（纵向双格一块板）', sameSet(field.targets, EXPECT.targets) && field.target.yaw === 'v',
    `${field.targets.join('+')} yaw=${field.target.yaw}`)
  check('①-4 桥洞 = K5（上）+ K7（下），恰 2 个且都在 K 列',
    sameSet(field.bridges, EXPECT.bridges) && field.bridges.every(c => c[0] === 'K'), field.bridges.join(' '))
  check('①-5 起点 N10 / 终点 N5 / 隔板 D 列·行 6-7',
    field.start === EXPECT.start && field.end === EXPECT.end
    && field.barrier.colFrom === EXPECT.barrier.colFrom && field.barrier.rowHi === 6 && field.barrier.rowLo === 7,
    `${field.start}→${field.end} barrier ${field.barrier.colFrom}/${field.barrier.rowHi}-${field.barrier.rowLo}`)
  check('①-6 无人机起 N8 / 降 M2，来源 = map.yaml 注释真源（非内建兜底）',
    field.drone.takeoff === EXPECT.drone.takeoff && field.drone.land === EXPECT.drone.land
    && field.drone.source === 'map-comment', JSON.stringify(field.drone))
  check('①-7 车端规划口径：auto_plan_enabled=false（本图手动路线态，引擎跟随 robot.yaml）',
    field.planner && field.planner.autoPlanEnabled === false,
    JSON.stringify(field.planner))
} else {
  skip('①-1~①-7 第六张图逐格期望', 'map.yaml 已换代（md5 不符）—— 换图时同步更新 EXPECT 与 SIXTH')
}

/* ── ② 渲染层：按坐标逐个点位（不是只数个数）────────────────────── */
h('② renderMap 渲染层逐坐标核对')
const els = {}
function el(id) {
  if (!els[id]) els[id] = {
    id, value: '', checked: id === 'showRoute', innerHTML: '', textContent: '', className: '',
    style: {}, parentNode: null, addEventListener() {},
    getElementsByTagName() { return [{ innerHTML: '' }] },
    querySelectorAll() { return [] },
    getAttribute() { return '' },
  }
  return els[id]
}
const documentStub = { getElementById: el, querySelectorAll() { return [] }, addEventListener() {} }
const windowStub = { addEventListener() {} }
const fetchStub = () => Promise.resolve({ json: () => Promise.resolve({}), text: () => Promise.resolve('') })
const api = new Function('document', 'window', 'fetch',
  'return (function(){' + body + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
  + ' cellCenter: cellCenter }; })();'
)(documentStub, windowStub, fetchStub)
for (const key of ['mines', 'obstacles', 'bridges', 'targets']) el(key).value = (field[key] || []).join(' ')
field.route = routeTokens()
api.setField(field)
api.renderMap()
const svg = el('map').innerHTML
const MAP_S = 34
const center = cell => api.cellCenter(cell)
const circleAt = (cell, fill) => {
  const p = center(cell)
  return p ? svg.includes(`<circle cx="${p[0]}" cy="${p[1]}" r="11.56" fill="${fill}"`) : false
}
const bridgeAt = (cell) => {
  const p = center(cell)
  return p ? svg.includes(`<circle cx="${p[0]}" cy="${p[1]}" r="12.24" fill="none" stroke="#0b6b73"`) : false
}
check('②-1 SVG 覆盖 14×11 格（viewBox 477×375）', /viewBox="0 0 477 375"/.test(svg), (svg.match(/viewBox="[^"]+"/) || [''])[0])
check('②-2 154 个基格 + 154 个格名（A1…N11）',
  (svg.match(/stroke="#c9ced8"/g) || []).length === 154 && (svg.match(/font-size="8.5"/g) || []).length >= 154,
  `cells=${(svg.match(/stroke="#c9ced8"/g) || []).length} labels=${(svg.match(/font-size="8.5"/g) || []).length}`)
check('②-3 每颗雷都画在它的格心上（红圈 + 「雷」字）',
  field.mines.every(cell => circleAt(cell, '#e5484d')) && (svg.match(/>雷</g) || []).length === field.mines.length,
  `${field.mines.length} 雷`)
check('②-4 每个障都画在它的格心上（灰块 + 「障」字）',
  field.obstacles.every(cell => circleAt(cell, '#4a4a4a')) && (svg.match(/>障</g) || []).length === field.obstacles.length,
  `${field.obstacles.length} 障`)
check('②-5 每个桥洞都画在它的格心上（青圈 + 「洞」字）',
  field.bridges.every(bridgeAt) && (svg.match(/>洞</g) || []).length === field.bridges.length,
  field.bridges.join(' '))
check('②-6 K 列：开放格青底 / 其余 9 格灰底（封死）',
  (svg.match(/fill="#7fe3ea"/g) || []).length === field.bridges.length
  && (svg.match(/fill="#d9d9d9"/g) || []).length === 11 - field.bridges.length,
  `cyan=${(svg.match(/fill="#7fe3ea"/g) || []).length} gray=${(svg.match(/fill="#d9d9d9"/g) || []).length}`)
const tx = center(field.targets[0]); const ty = center(field.targets[field.targets.length - 1])
const targetBoard = tx && ty && Math.abs(tx[0] - ty[0]) < 1 ? [Math.min(tx[0], ty[0]) - 17, Math.min(tx[1], ty[1]) - 17, 34, 68] : null
check('②-7 靶板 = 一块 34×68 竖向并板，画在 D5+D6 上',
  Boolean(targetBoard) && svg.includes(`<rect x="${targetBoard[0]}" y="${targetBoard[1]}" width="34" height="68" fill="#e8a0c8"`)
  && (svg.match(/>靶板</g) || []).length === 1 && /竖向 D5\+D6/.test(svg),
  targetBoard ? JSON.stringify(targetBoard) : 'no vertical board')
const bx = 3 * MAP_S
check('②-8 隔板：行 6/7 之间、D 列起到 N 列的黑条 + 文案',
  svg.includes(`<rect x="${bx}" y="${6 * MAP_S - 3}" width="${14 * MAP_S - bx}" height="6" fill="#111111"`)
  && svg.includes('隔板 D~N'),
  `x=${bx} y=${6 * MAP_S - 3}`)
check('②-9 起点 N10 绿底 + 「起」 / 终点 N5 金底 + 「终」',
  svg.includes('<rect x="442" y="306" width="34" height="34" fill="#b2f2bb"') && svg.includes('>起<')
  && svg.includes('<rect x="442" y="136" width="34" height="34" fill="#ffe066"') && svg.includes('>终<'), 'start/end')
check('②-10 无人机起 N8 蓝「机」/ 落 M2 蓝「H」',
  svg.includes('<rect x="442" y="238" width="34" height="34" fill="#cfe2f7"') && svg.includes('>机<')
  && svg.includes('<rect x="408" y="34" width="34" height="34" fill="#cfe2f7"') && svg.includes('>H<'), 'drone')
check('②-11 页面如实显示「车端手动路线 + 规划引擎」（改点位 422 时操作员有据可查）',
  String(el('mapMeta').textContent).includes(field.planner.engine || 'tour') || String(el('mapMeta').textContent).includes('规划引擎 ' + (field.planner.engine || 'tour'))
  && String(el('mapMeta').textContent).includes('车端手动路线'),
  String(el('mapMeta').textContent))

/* ── ③ 车端手动路线（本轮 auto_plan_enabled=false）的物理口径 ────────── */
h('③ 车端 route.tokens 物理口径（不压障/不穿靶/隔板只走 A/B/C/雷全覆盖/先 K7 后 K5）')
const tokens = routeTokens()
const tokenCell = t => String(t).trim().replace(/^R:/i, '').replace(/[,:].*$/, '').replace(/\*$/, '').trim().toUpperCase()
const cells = tokens.map(tokenCell)
const rowOf = c => Number(String(c).slice(1))
const colOf = c => String(c)[0]
check('③-1 车端路线非空且首末 = N10 / N5',
  cells.length > 0 && cells[0] === field.start && cells[cells.length - 1] === field.end,
  `${tokens.length} token ${cells[0]}→${cells[cells.length - 1]}`)
check('③-2 路线不压障碍格', cells.every(c => !field.obstacles.includes(c)),
  cells.filter(c => field.obstacles.includes(c)).join(' ') || 'ok')
check('③-3 路线不穿靶板格（D5/D6 是实体跨格板，只能旁边停站）',
  cells.every(c => !field.targets.includes(c)), cells.filter(c => field.targets.includes(c)).join(' ') || 'ok')
const badCross = []
for (let i = 1; i < cells.length; i += 1) {
  const a = cells[i - 1]; const b = cells[i]
  const crossesBarrier = (rowOf(a) <= 6 && rowOf(b) >= 7) || (rowOf(a) >= 7 && rowOf(b) <= 6)
  if (crossesBarrier && !['A', 'B', 'C'].includes(colOf(a))) badCross.push(a + '→' + b)
}
check('③-4 跨隔板（行 6↔7）只发生在 A/B/C 列（D~N 封死）',
  badCross.length === 0, badCross.join(' ') || 'ok')
check('③-5 8 颗雷全部被路线压过（雷格必须出现在 token 序列里）',
  field.mines.every(c => cells.includes(c)),
  field.mines.filter(c => !cells.includes(c)).join(' ') || 'all 8')
const iK7 = cells.indexOf('K7'); const iK5 = cells.indexOf('K5')
check('③-6 先穿下半场 K7、后穿上半场 K5', iK7 >= 0 && iK5 >= 0 && iK7 < iK5, `K7@${iK7} K5@${iK5}`)
check('③-7 K 列只允许出现在 K5/K7（其余 K 格封死）',
  cells.filter(c => c[0] === 'K').every(c => field.bridges.includes(c)),
  cells.filter(c => c[0] === 'K').join(' ') || 'ok')

/* ── ④ 无 GPU 假开关（仿真链是 CPU 确定性计算）────────────────────── */
h('④ 无 GPU 假开关')
check('④-1 页面明示「纯 CPU 的确定性离线计算」且无 GPU 开关/加速字段',
  rawHtml.includes('纯 CPU 的确定性离线计算') && !/(id="gpu"|name="gpu"|GPU\s*加速|gpu_enabled)/i.test(rawHtml),
  (rawHtml.match(/GPU[^<]{0,20}/g) || ['(none)']).slice(0, 3).join(' | '))
const indexSrc = fs.readFileSync(path.join(PLUGIN, 'index.js'), 'utf8')
check('④-2 422 失败原因可诊断：宿主回 record/planTail，页面优先显示 record.plan.stderrTail',
  /result\.planTail/.test(indexSrc) && /problem\.body\.record/.test(rawHtml)
  && /problem\.body\.planTail/.test(rawHtml) && /plan\.stderrTail/.test(rawHtml))

/* ── 汇总 ───────────────────────────────────────────────────────── */
const total = pass.ok + pass.fail
console.log('')
console.log(`结果：${pass.ok}/${total} 通过${pass.skip ? `，${pass.skip} 跳过` : ''}`
  + `${pass.fail === 0 ? ' —— 全部通过 ✅' : `，${pass.fail} 失败 ❌`}`)
process.exit(pass.fail === 0 ? 0 : 1)
