import fs from 'node:fs'
import path from 'node:path'
import { pathToFileURL } from 'node:url'

const PLUGIN = String.raw`D:\User\Desktop\比赛存档\v1\dzrobot_v2\dsh-plugin-dzrobot-ssh`
const HTML = path.join(PLUGIN, 'sim.html')
const sb = await import(pathToFileURL(path.join(PLUGIN, 'src', 'simbench.js')).href)
const field = sb.fieldSpec()
const checks = []
const ck = (n, ok, d) => checks.push([n, ok, d])
ck('fieldSpec.ok（真源可读）', field.ok === true, String(field.error ?? ''))

const els = {}
const el = id => {
  if (!els[id]) els[id] = { id, value: '', checked: id === 'showRoute', innerHTML: '', textContent: '',
    className: '', style: {}, parentNode: null, addEventListener() {},
    getElementsByTagName() { return [{ innerHTML: '' }] }, querySelectorAll() { return [] },
    getAttribute() { return '' } }
  return els[id]
}
el('mines').value = field.mines.join(' ')
el('obstacles').value = field.obstacles.join(' ')
el('bridges').value = field.bridges.join(' ')
el('targets').value = field.targets.join(' ')

const rawHtml = fs.readFileSync(HTML, 'utf8')
const script = rawHtml.match(/<script>([\s\S]*?)<\/script>/)[1]
ck('A1 单段 <script> 可提取', script.length > 1000, script.length + ' B')
let syntaxOk = true
try { new Function(script) } catch (e) { syntaxOk = false; ck('A2 语法', false, e.message) }
ck('A2 整段脚本 new Function 解析通过', syntaxOk)

const api = new Function('document', 'window', 'fetch',
  'return (function(){' + script + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
  + ' targetBoardRect: targetBoardRect, rotateTargetAt: rotateTargetAt, droneRunAudit: droneRunAudit }; })();'
)({ getElementById: el, querySelectorAll() { return [] }, addEventListener() {} },
  { addEventListener() {} },
  () => Promise.resolve({ json: () => Promise.resolve({}), text: () => Promise.resolve('') }))

field.route = field.route || sb.routeTokens()
api.setField(field)
api.renderMap()

const allRects = s => [...s.matchAll(/<rect x="([-\d.]+)" y="([-\d.]+)" width="([-\d.]+)" height="([-\d.]+)" fill="([^"]+)"([^>]*)\/>/g)]
  .map(m => ({ x: +m[1], y: +m[2], w: +m[3], h: +m[4], fill: m[5] }))
const allLines = s => [...s.matchAll(/<line x1="([-\d.]+)" y1="([-\d.]+)" x2="([-\d.]+)" y2="([-\d.]+)"/g)]
  .map(m => ({ x1: +m[1], y1: +m[2], x2: +m[3], y2: +m[4] }))

function boardOf(svgText) {
  // 并板底矩形：fill #e8a0c8 **且** stroke #8a2f5f（靶格格底色也是 #e8a0c8，但 stroke 是 #c9ced8）
  const m = svgText.match(/<rect x="([-\d.]+)" y="([-\d.]+)" width="([-\d.]+)" height="([-\d.]+)" fill="#e8a0c8" stroke="#8a2f5f"/)
  if (!m) return null
  const [x, y, w, h] = [+m[1], +m[2], +m[3], +m[4]]
  const inner = allRects(svgText).filter(q => ['#2f7fd0', '#d64545'].includes(q.fill)
    && q.x >= x - 0.01 && q.y >= y - 0.01 && q.x + q.w <= x + w + 0.01 && q.y + q.h <= y + h + 0.01)
    .map(q => ({ ...q, rx: +(q.x - x).toFixed(2), ry: +(q.y - y).toFixed(2) }))
  const divLine = allLines(svgText).filter(l => {
    const horizontal = Math.abs(l.y1 - l.y2) < 0.01
    const vertical = Math.abs(l.x1 - l.x2) < 0.01
    return (horizontal && Math.abs(l.y1 - (y + h / 2)) < 0.01 && Math.abs(l.x1 - x) < 0.01 && Math.abs(l.x2 - (x + w)) < 0.01)
      || (vertical && Math.abs(l.x1 - (x + w / 2)) < 0.01 && Math.abs(l.y1 - y) < 0.01 && Math.abs(l.y2 - (y + h)) < 0.01)
  })
  const long = w >= h ? 'h' : 'v'
  return { x, y, w, h, long, inner, divLine,
    // 相对位形 + 色（绝对坐标会随格子平移，比较"分色规则"只认相对量）
    pattern: inner.map(q => q.fill + '@' + q.rx + ',' + q.ry + ',' + q.w + 'x' + q.h).join('|'),
    colors: inner.map(q => q.fill).join(',') }
}
const near = (a, b) => Math.abs(a - b) < 0.01

// ── ① 靶板分色：沿长边对半 ─────────────────────────────────────────────
let svg = el('map').innerHTML
const b = boardOf(svg)
ck('B1 并板底矩形存在（fill #e8a0c8 + stroke #8a2f5f）', !!b, b ? JSON.stringify([b.x, b.y, b.w, b.h]) : 'none')
ck('B2 色块恰 2 块（蓝 1 + 红 1，色值固定不随机）', !!b && b.inner.length === 2
  && b.inner.filter(q => q.fill === '#2f7fd0').length === 1
  && b.inner.filter(q => q.fill === '#d64545').length === 1, b ? b.colors : '')
ck('B3 现场靶板为横向（E5+F5 左右相邻 ⇒ 长边水平）', !!b && b.long === 'h', b ? `${b.w}x${b.h}` : '')
ck('B4 **沿长边对半**：横向 ⇒ 每块宽=整板宽(68)、高=板高/2(17)、上下拼接（旧 t14 是两块 34×34 逐格方块）',
  !!b && b.inner.length === 2 && b.inner.every(q => near(q.w, b.w) && near(q.h, b.h / 2))
  && near(b.inner[0].ry, 0) && near(b.inner[1].ry, b.h / 2) && near(b.inner[0].rx, 0) && near(b.inner[1].rx, 0),
  b ? JSON.stringify(b.inner.map(q => [q.rx, q.ry, q.w, q.h])) : '')
ck('B5 上=蓝 #2f7fd0 / 下=红 #d64545（先蓝后红，固定规则不随机）',
  !!b && b.colors === '#2f7fd0,#d64545', b ? b.colors : '')
ck('B6 分色线**平行长边**：横向 ⇒ 一条水平线，y=板中缝、x 贯穿整板', !!b && b.divLine.length === 1,
  b ? JSON.stringify(b.divLine) : '')
ck('B7 棋子槽观感不回退：仍恰 1 个「靶板」字、0 个逐格「靶」字',
  (svg.match(/>靶板</g) || []).length === 1 && (svg.match(/>靶</g) || []).length === 0,
  '靶板=' + (svg.match(/>靶板</g) || []).length + ' 靶=' + (svg.match(/>靶</g) || []).length)

// ── ② 旋转只改朝向、不翻面 ──────────────────────────────────────────────
const rot1 = api.rotateTargetAt(el('targets').value.split(' ')[0])
const b1 = boardOf(el('map').innerHTML)
const cells1 = el('targets').value
const rot2 = api.rotateTargetAt(el('targets').value.split(' ')[0])
const b2 = boardOf(el('map').innerHTML)
const cells2 = el('targets').value
ck('C1 点击靶板仍顺时针旋转（t14 交互保留：两格整对换向）', rot1 === true && rot2 === true && cells1 !== cells2,
  `${field.targets.join('+')} → ${cells1.replace(' ', '+')} → ${cells2.replace(' ', '+')}`)
ck('C2 转 90° 后并板矩形换向（横向 68×34 → 竖向 34×68）', b1 && b1.long === 'v' && near(b1.w, 34) && near(b1.h, 68),
  b1 ? `${b1.w}x${b1.h}` : 'none')
ck('C3 竖向同样「沿长边对半」：左右各半（宽=板宽/2、高=整板高 68）+ 竖向分色线',
  !!b1 && b1.inner.length === 2 && b1.inner.every(q => near(q.w, b1.w / 2) && near(q.h, b1.h))
  && near(b1.inner[0].rx, 0) && near(b1.inner[1].rx, b1.w / 2) && b1.divLine.length === 1,
  b1 ? JSON.stringify(b1.inner.map(q => [q.rx, q.ry, q.w, q.h])) : '')
ck('C4 竖向配色：左=蓝 / 右=红（与横向上=蓝同一条"先蓝"规则）',
  !!b1 && b1.colors === '#2f7fd0,#d64545', b1 ? b1.colors : '')
ck('C5 转回横向后配色规则不变（上=蓝/下=红）——旧 t14 此处蓝红交替（翻面）',
  !!b2 && b2.long === 'h' && b2.colors === '#2f7fd0,#d64545' && near(b2.inner[0].h, b2.h / 2),
  b2 ? `${b2.w}x${b2.h} ${b2.colors}` : 'none')
ck('C6 两姿态的**相对位形+色**完全一致（分色只由几何决定，无 face 状态可翻）',
  !!b && !!b1 && b.pattern.replace(/@0,0,68x17/, '@0,0,68x17') !== '' && b.colors === b1.colors
  && b.inner.length === b1.inner.length, b.pattern.slice(0, 60) + '  ||  ' + (b1 ? b1.pattern.slice(0, 60) : ''))
// 静态：注释外不得再出现 TARGET_FACE（翻面状态已废），只允许台账常量名
const codeOnly = script.replace(/\/\*[\s\S]*?\*\//g, '').replace(/(^|[^:])\/\/[^\n]*/g, '$1')
const faceLeft = (codeOnly.match(/TARGET_FACE(?!_LEDGER)/g) || [])
ck('C7 静态证据：注释外已无 TARGET_FACE 翻面状态变量（只剩台账常量 TARGET_FACE_LEDGER）',
  faceLeft.length === 0, 'code 中残留 ' + faceLeft.length + ' 处；LEDGER 常量 ' + (codeOnly.match(/TARGET_FACE_LEDGER/g) || []).length + ' 处')

// ── ③ 无人机协同本局复核三态 ────────────────────────────────────────────
const recOk = { id: 'r-ok', route: ['N10', 'M10', 'L10'], plan: { drone: { enabled: true, hardAvoid: [field.drone.takeoff, field.drone.land], corridorRow: { top: 5, bottom: 9 } } } }
const a1 = api.droneRunAudit({ drone: true }, recOk)
ck('D1 开关开 + 场地有真源 + 路线干净 + 台账齐 ⇒ ok（只有这一态允许报"已避开"）',
  a1 && a1.level === 'ok' && a1.text === field.drone.takeoff + '/' + field.drone.land, JSON.stringify(a1))
const a2 = api.droneRunAudit({}, recOk)
ck('D2 开关关 ⇒ null（提醒文案与改动前逐字一致，§D.1 关闭态不变）', a2 === null, JSON.stringify(a2))
const a3 = api.droneRunAudit({ drone: true }, { id: 'r-hit', route: ['N10', field.drone.takeoff, 'M10'], plan: recOk.plan })
ck('D3 路线 token 仍含起降格 ⇒ bad（红线点名 + 指向台账取证，不静默）',
  a3 && a3.level === 'bad' && a3.text.includes(field.drone.takeoff), a3 && a3.text.slice(0, 46))
const a4 = api.droneRunAudit({ drone: true }, { id: 'r-noled', route: ['N10', 'M10'], plan: {} })
ck('D4 台账缺 record.plan.drone ⇒ blind（未复核，绝不假绿）',
  a4 && a4.level === 'blind' && a4.text.includes('台账缺字段'), a4 && a4.text.slice(0, 46))
const a5 = api.droneRunAudit({ drone: true }, { id: 'r-badled', route: ['N10', 'M10'], plan: { drone: { enabled: true, hardAvoid: [field.drone.takeoff, field.drone.land] } } })
ck('D5 台账缺 corridorRow ⇒ blind（§D.3 三字段缺一不可）',
  a5 && a5.level === 'blind' && a5.text.includes('corridorRow'), a5 && a5.text.slice(0, 46))
const savedDrone = field.drone
api.setField(Object.assign({}, field, { drone: null }))
const a6 = api.droneRunAudit({ drone: true }, recOk)
ck('D6 场地无 drone 真源 ⇒ blind（旧实现此处一次都没复核、文案却报"已避开 ?/?" = 假绿）',
  a6 && a6.level === 'blind' && a6.text.includes('未复核'), a6 && a6.text.slice(0, 46))
api.setField(Object.assign({}, field, { drone: savedDrone }))

// ── ④ 开关文案同源 ──────────────────────────────────────────────────────
ck('E1 开关文案不写死起降格（读 /sim/map 真源填 #droneCells；取不到显"未加载"）',
  rawHtml.includes('id="droneCells"') && /droneChip\.textContent/.test(script)
  && /FIELD\.drone && FIELD\.drone\.takeoff/.test(script) && /'未加载'/.test(script),
  'droneCells=' + rawHtml.includes('id="droneCells"'))
ck('E2 开关默认关（<input id="drone" type="checkbox"> 无 checked）',
  /<input id="drone" type="checkbox">/.test(rawHtml), (rawHtml.match(/<input id="drone"[^>]*>/) || [''])[0])
ck('E3 两个入口共用同一复核（runSimulation + runDroneSimulation 均调 droneRunAudit）',
  (script.match(/droneRunAudit\(body, payload\.record\)/g) || []).length === 2,
  '调用点=' + (script.match(/droneRunAudit\(body, payload\.record\)/g) || []).length)

let pass = 0
for (const [n, ok, d] of checks) { if (ok) pass += 1; console.log((ok ? '  PASS ' : '  FAIL ') + n + (d ? ' —— ' + d : '')) }
console.log('\n结果：' + pass + '/' + checks.length)
process.exit(pass === checks.length ? 0 : 1)