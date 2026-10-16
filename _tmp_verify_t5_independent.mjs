/**
 * t5（sim-bench2 / verifier）独立取证 —— 不采信 t1/t2 自报结论，全部自己重算。
 *
 * 手法：自己搭 DOM 桩 → 跑 sim.html 的内联脚本（真 renderMap()）→ 直接量 SVG 与函数输出。
 * 真源：
 *   · 官方几何 src/dzbrain/dzbrain/core/geometry.py:69-80（x=(10.5-row)*0.35, y=(13.5-col)*0.35）
 *   · 真 traj.csv（artifacts/sim/<run>/traj.csv）
 * 只读；不改仓库任何被跟踪文件。
 */
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { execFileSync } from 'node:child_process'

const HERE = path.dirname(fileURLToPath(import.meta.url))              // dzrobot_v2/
const PLUGIN = path.join(HERE, 'dsh-plugin-dzrobot-ssh')
// ★ 并发漂移护栏：可用 T5_HTML 指到冻结快照上跑（受审对象必须是固定 revision，不是"此刻的文件"）
const HTML = process.env.T5_HTML ? path.resolve(process.env.T5_HTML) : path.join(PLUGIN, 'sim.html')
console.log(`[受审 sim.html] ${HTML}  md5=${(await import('node:crypto')).createHash('md5').update(fs.readFileSync(HTML)).digest('hex').slice(0, 10)}`)
const body = fs.readFileSync(HTML, 'utf8').match(/<script>([\s\S]*?)<\/script>/)[1]

const out = []
const say = (...a) => { const line = a.join(' '); out.push(line); console.log(line) }
const results = []
const chk = (id, ok, detail) => { results.push([id, ok]); say(`${ok ? 'OK  ' : 'BAD '} ${id}${detail ? '  ' + detail : ''}`) }

// ── DOM 桩（自建；含事件捕获与 data-reset 按钮） ──────────────────────────────
const RESET_KEYS = ['mines', 'obstacles', 'bridges', 'targetYaw']
const els = {}
const listeners = {}                     // id -> [[type, fn], ...]
const buttons = RESET_KEYS.map(key => ({
  key, listeners: [], getAttribute: n => (n === 'data-reset' ? key : ''),
  addEventListener(t, f) { this.listeners.push([t, f]) },
}))
const msgs = []
function el(id) {
  if (!els[id]) {
    els[id] = {
      id, value: '', checked: id === 'showRoute', innerHTML: '', textContent: '', className: '',
      style: {}, parentNode: null,
      addEventListener(type, fn) { (listeners[id] = listeners[id] || []).push([type, fn]) },
      getElementsByTagName() { return [{ innerHTML: '' }] },
      querySelectorAll() { return [] },
      getAttribute() { return '' },
    }
  }
  return els[id]
}
let renderCalls = 0
const documentStub = {
  getElementById: el,
  querySelectorAll(sel) { return sel === 'button[data-reset]' ? buttons : [] },
  addEventListener() {},
}
const windowStub = { addEventListener() {} }
const fetchStub = () => Promise.resolve({ json: () => Promise.resolve({}), text: () => Promise.resolve('') })
const api = new Function('document', 'window', 'fetch', 'location',
  'return (function(){' + body + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
  + ' cellCenter: cellCenter, codeToScreen: codeToScreen, routePolyline: routePolyline,'
  + ' carPolygon: carPolygon, targetBoardRect: targetBoardRect, headVec: headVec,'
  + ' targetYawMode: targetYawMode, refreshFieldHints: function(){ return refreshFieldHints(); } }; })();'
)(documentStub, windowStub, fetchStub, { hash: '' })

// 计数 renderMap 调用（包一层）
const rawRender = api.renderMap
api.renderMap = function (...a) { renderCalls += 1; return rawRender.apply(null, a) }
// 把内联脚本里注册的 map 再包一层：直接把 FIELD 塞进去后调 api.renderMap 即可
const { fieldSpec, routeTokens } = await import(pathToFileURLCompat(path.join(PLUGIN, 'src', 'simbench.js')))
function pathToFileURLCompat(p) { return new URL('file:///' + p.replace(/\\/g, '/').replace(/^\//, '')) }
const field = fieldSpec()
field.route = routeTokens()
el('mines').value = field.mines.join(' ')
el('obstacles').value = field.obstacles.join(' ')
el('bridges').value = field.bridges.join(' ')
api.setField(field)
rawRender()

// ── F1：靶板残留 ─────────────────────────────────────────────────────────────
const svg = el('map').innerHTML
const n = re => (svg.match(re) || []).length
say('')
say('── F1 靶板渲染（真 renderMap 的 SVG 实测） ──')
chk('F1-1 >靶< 出现次数 = 0（per-cell 散字已消失）', n(/>靶</g) === 0, `靶=${n(/>靶</g)}`)
chk('F1-2 >靶板< 出现次数 = 1（t28 并板）', n(/>靶板</g) === 1, `靶板=${n(/>靶板</g)}`)
chk('F1-3 并板描边 stroke="#8a2f5f" 在位', n(/stroke="#8a2f5f"/g) === 1, `n=${n(/stroke="#8a2f5f"/g)}`)
// #e8a0c8 在 SVG 里出现 3 次 = E5/F5 两格的"靶粉底"（sim.html:714 仍按设计保留）+ 1 块并板（:748）
chk('F1-4 靶粉底计数 = 2 格 + 1 并板 = 3（散字没了、粉底按设计保留）',
  n(/fill="#e8a0c8"/g) === field.targets.length + 1, `n=${n(/fill="#e8a0c8"/g)} targets=${field.targets.length}`)
say(`     （参考：雷=${n(/>雷</g)} 障=${n(/>障</g)} 洞=${n(/>洞</g)} 起=${n(/>起</g)} 终=${n(/>终</g)} 机=${n(/>机</g)} 靶板朝向标注=${n(/8\.5/g)}）`)

// ── F3：select#targetYaw 事件绑定 + reset 分流 ───────────────────────────────
say('')
say('── F3 targetYaw 事件绑定与 reset 分流 ──')
const yawL = (listeners.targetYaw || []).map(x => x[0])
chk('F3-1 #targetYaw 绑了 change 监听', yawL.includes('change'), `types=${JSON.stringify(yawL)}`)
chk('F3-2 #targetYaw 没有绑 input（select 用 change）', !yawL.includes('input'), `types=${JSON.stringify(yawL)}`)
// 触发 change → 必须重绘（把 map 的 innerHTML 换成哨兵，看它是否被真 renderMap 覆盖）
el('map').innerHTML = '<sentinel/>'
const before = renderCalls
const changeFn = (listeners.targetYaw || []).find(x => x[0] === 'change')[1]
changeFn()
chk('F3-3 触发 change 后地图真的重绘（哨兵被真 renderMap 覆盖，含 <svg viewBox）',
  !el('map').innerHTML.includes('sentinel') && /<svg viewBox=/.test(el('map').innerHTML),
  `sentinel_gone=${!el('map').innerHTML.includes('sentinel')} hasSvg=${/<svg viewBox=/.test(el('map').innerHTML)}`)
// 三个 textarea 仍是 input（对照组）
const inputTypes = ['mines', 'obstacles', 'bridges'].map(k => ((listeners[k] || []).map(x => x[0]).join('+')))
chk('F3-4 对照：mines/obstacles/bridges 仍是 input 监听', inputTypes.every(t => t === 'input'), JSON.stringify(inputTypes))
// reset 按钮：select 走默认项（''），textarea 仍走 join(' ')
const yawBtn = buttons.find(b => b.key === 'targetYaw')
const minesBtn = buttons.find(b => b.key === 'mines')
chk('F3-5 四个 data-reset 按钮都注册了 click 监听',
  buttons.every(b => b.listeners.some(x => x[0] === 'click')),
  buttons.map(b => b.key + ':' + b.listeners.length).join(' '))
el('targetYaw').value = 'v'
msgs.length = 0
el('runMsg').textContent = ''
el('runMsg').className = ''
yawBtn.listeners.find(x => x[0] === 'click')[1]()
chk('F3-6 targetYaw 的 reset → 恢复默认选中项 ""（跟随地图），不是 join(" ")',
  el('targetYaw').value === '', `value=${JSON.stringify(el('targetYaw').value)}`)
// message() 把文案写到 $('runMsg') 上（内联函数，不是我的桩）：直接读元素
chk('F3-7 targetYaw 的 reset 走的是专门分支（runMsg 收到「靶板朝向已恢复」而不是「已恢复现场默认值」）',
  /靶板朝向已恢复/.test(el('runMsg').textContent) && el('runMsg').className.includes('ok'),
  `runMsg=${JSON.stringify(el('runMsg').textContent)} class=${el('runMsg').className}`)
el('mines').value = 'ZZZ'
el('runMsg').textContent = ''
minesBtn.listeners.find(x => x[0] === 'click')[1]()
chk('F3-8 对照：mines 的 reset 仍走 join(" ")（textarea 路径没被改坏）',
  el('mines').value === (field.mines || []).join(' ') && /现场默认值/.test(el('runMsg').textContent),
  `value=${el('mines').value.slice(0, 40)}… runMsg=${JSON.stringify(el('runMsg').textContent)}`)
// select 的默认项确实是 value=""
const htmlText = fs.readFileSync(HTML, 'utf8')
const selBlock = htmlText.match(/<select id="targetYaw">([\s\S]*?)<\/select>/)[1]
const firstOpt = selBlock.match(/<option value="([^"]*)"/)[1]
chk('F3-9 <select id="targetYaw"> 的首个 option 就是 value=""（所以 value="" 等于恢复默认项）',
  firstOpt === '', `firstOptionValue=${JSON.stringify(firstOpt)}`)

// ── 三项改进基准行为（独立复算，对照官方几何真源） ───────────────────────────
say('')
say('── 基准行为独立复算（对照 src/dzbrain/core/geometry.py 真源） ──')
const eq = (a, b) => JSON.stringify(a) === JSON.stringify(b)
const cells = []
for (let c = 0; c < field.grid.cols; c += 1) for (let r = 1; r <= field.grid.rows; r += 1) cells.push(field.grid.colLetters[c] + r)
// 官方 block_to_meters 真值（python 现算，不用 t1 的数字）
const pyCode = 'from dzbrain.core.geometry import block_to_meters as b;import json,sys;'
  + 'cells=json.loads(sys.argv[1]);print(json.dumps({c:b(c) for c in cells},ensure_ascii=False))'
const truth = JSON.parse(execFileSync('python', ['-c', pyCode, JSON.stringify(cells)], {
  env: { ...process.env, PYTHONPATH: path.join(HERE, 'src', 'dzbrain'), PYTHONIOENCODING: 'utf-8' },
}).toString())
let maxDevPx = 0, worst = ''
let maxDevOldPx = 0
for (const c of cells) {
  const [x, y] = truth[c]
  const got = api.codeToScreen(x, y)
  const want = api.cellCenter(c)
  const dev = Math.hypot(got[0] - want[0], got[1] - want[1])
  if (dev > maxDevPx) { maxDevPx = dev; worst = c }
  const old = [(13.5 - y / 0.35) * 34, (10.5 - x / 0.35) * 34]     // t1 修前的旧式
  maxDevOldPx = Math.max(maxDevOldPx, Math.hypot(old[0] - want[0], old[1] - want[1]))
}
chk(`基准-1 codeToScreen(block_to_meters(cell)) == cellCenter(cell)，154 格最大偏差 ${maxDevPx.toFixed(2)} px`,
  maxDevPx < 0.01, `worst=${worst}`)
chk(`基准-2 对照：t1 修前旧式在同 154 格上最大偏差 ${maxDevOldPx.toFixed(2)} px = 半格（17√2）`,
  Math.abs(maxDevOldPx - Math.hypot(17, 17)) < 0.01, `old=${maxDevOldPx.toFixed(2)}`)
const pA1D1 = api.routePolyline(['A1', 'D1'])
chk('基准-3 routePolyline(A1→D1) = [[17,17],[51,17],[85,17],[119,17]]（4 点，补 2 格心）',
  eq(pA1D1, [[17, 17], [51, 17], [85, 17], [119, 17]]), JSON.stringify(pA1D1))
const pN11N8 = api.routePolyline(['N11', 'N8'])
chk('基准-4 routePolyline(N11→N8) 4 点且 x 恒 459', eq(pN11N8, [[459, 357], [459, 323], [459, 289], [459, 255]]), JSON.stringify(pN11N8))
const pN10N7 = api.routePolyline(['N10', 'N7'])
chk('基准-5 routePolyline(N10→N7) = [[459,323],[459,289],[459,255],[459,221]]（跨 3 行补 2 点）',
  eq(pN10N7, [[459, 323], [459, 289], [459, 255], [459, 221]]), JSON.stringify(pN10N7))
const routeCells = field.route.map(t => String(t).trim().replace(/^R:/i, '').replace(/[,:].*$/, '').replace(/\*$/, '').trim().toUpperCase()).filter(Boolean)
const pRoute = api.routePolyline(routeCells)
chk(`基准-6 真路线 ${field.route.length} token（全正交）→ 折线 ${pRoute.length} 点（=token 数，逐段应补 0）`,
  pRoute.length === routeCells.length, `pts=${pRoute.length}`)
const bb = p => [Math.max(...p.map(q => q[0])) - Math.min(...p.map(q => q[0])), Math.max(...p.map(q => q[1])) - Math.min(...p.map(q => q[1]))]
const car0 = api.carPolygon(459, 323, 0)
const car90 = api.carPolygon(459, 323, Math.PI / 2)
chk('基准-7 carPolygon 尺寸 = 0.28m 长 × 0.135m 宽 → 27.2 × 13.11 px，yaw 转 90° 长宽互换',
  Math.abs(bb(car0)[0] - 13.1143) < 0.01 && Math.abs(bb(car0)[1] - 27.2) < 0.01
  && Math.abs(bb(car90)[0] - 27.2) < 0.01 && Math.abs(bb(car90)[1] - 13.1143) < 0.01,
  `car0=${bb(car0).map(v => v.toFixed(2))} car90=${bb(car90).map(v => v.toFixed(2))}`)
const rectHF = api.targetBoardRect(['E5', 'F5'], '')
const rectHFv = api.targetBoardRect(['E5', 'F5'], 'v')
const rectVE = api.targetBoardRect(['E5', 'E6'], '')
chk('基准-8 targetBoardRect: E5+F5 auto=68×34(横向)、E5+E6 auto=34×68(竖向)',
  eq(rectHF, [136, 136, 68, 34]) && eq(rectVE, [136, 136, 34, 68]), `${JSON.stringify(rectHF)} ${JSON.stringify(rectVE)}`)
chk('基准-9 targetBoardRect 的 mode 参数**不影响返回**（显式 v 与 auto 同值）——死参数，见 finding F-A2',
  eq(rectHFv, rectHF), `${JSON.stringify(rectHFv)} vs ${JSON.stringify(rectHF)}`)

// ── 车头方向：用真 traj.csv 的位移方向做模型无关交叉验证 ────────────────────
say('')
say('── 车头方向（独立：真 traj.csv 位移 vs headVec） ──')
const runsDir = path.join(HERE, 'artifacts', 'sim')
const cands = fs.readdirSync(runsDir).filter(d => !/drift|noise|lat|anchor|bad|seed/i.test(d))
  .map(d => path.join(runsDir, d, 'traj.csv'))
  .filter(f => fs.existsSync(f))
  .map(f => ({ f, head: fs.readFileSync(f, 'utf8').split('\n')[0] }))
  .filter(x => /truth_x/.test(x.head) && /truth_y/.test(x.head) && /truth_yaw/.test(x.head))
  .sort((a, b) => fs.statSync(b.f).size - fs.statSync(a.f).size)
const runFile = cands[0].f
const csv = fs.readFileSync(runFile, 'utf8').trim().split('\n')
const head = csv[0].split(',')
const ix = head.indexOf('truth_x'), iy = head.indexOf('truth_y'), iw = head.indexOf('truth_yaw')
const iv = head.indexOf('v'), irev = head.indexOf('reverse')
const rows = csv.slice(1).map(l => l.split(',').map(Number))
// 只取「严格直行」小窗：10 拍内 yaw 基本不变 ⇒ 位移方向 = 车头方向，与 true 方向无关的模型无关判据
const STEP = 10
const angNew = [], angOld = []
for (let i = 0; i + STEP < rows.length; i += STEP) {
  const a = rows[i], b = rows[i + STEP]
  if (!a || !b) continue
  const yaw = a[iw], yaw1 = b[iw], v = iv >= 0 ? a[iv] : 1, rev = irev >= 0 ? a[irev] : 0
  if (!isFinite(yaw) || !isFinite(yaw1) || !isFinite(v) || v <= 0.05 || rev === 1) continue
  if (Math.abs(yaw1 - yaw) > 0.02) continue                    // ≈1.15°：严格直行段
  const s0 = api.codeToScreen(a[ix], a[iy]), s1 = api.codeToScreen(b[ix], b[iy])
  const dx = s1[0] - s0[0], dy = s1[1] - s0[1]
  const len = Math.hypot(dx, dy)
  if (!(len > 1.5)) continue
  const dir = [dx / len, dy / len]
  const th = yaw + Math.PI / 2
  const angle = (h) => Math.acos(Math.max(-1, Math.min(1, h[0] * dir[0] + h[1] * dir[1]))) * 180 / Math.PI
  angNew.push(angle(api.headVec(yaw)))
  angOld.push(angle([Math.cos(th), Math.sin(th)]))             // t1 修前旧式
}
const med = a => { const s = [...a].sort((p, q) => p - q); return s.length ? s[Math.floor(s.length / 2)] : NaN }
const p95 = a => { const s = [...a].sort((p, q) => p - q); return s.length ? s[Math.floor(s.length * 0.95)] : NaN }
const maxOf = a => (a.length ? Math.max(...a) : NaN)
say(`     数据：${path.basename(path.dirname(runFile))}/traj.csv（${rows.length} 拍）  严格直行样本 ${angNew.length} 段 / 每段 ${STEP} 拍`)
const fracBack = a => (a.length ? a.filter(x => x > 90).length / a.length : NaN)
say(`     修后 headVec 与前进位移夹角：中位数 ${med(angNew).toFixed(2)}° / P95 ${p95(angNew).toFixed(2)}° / 最大 ${maxOf(angNew).toFixed(2)}° / 反向(>90°)占比 ${(fracBack(angNew) * 100).toFixed(1)}%`)
say(`     修前旧式   与前进位移夹角：中位数 ${med(angOld).toFixed(2)}° / P95 ${p95(angOld).toFixed(2)}° / 最大 ${maxOf(angOld).toFixed(2)}° / 反向(>90°)占比 ${(fracBack(angOld) * 100).toFixed(1)}%`)
chk('基准-10 修后 headVec 与前进位移同向（直行段中位数 < 5°、P95 < 15°、零反向）',
  angNew.length >= 30 && med(angNew) < 5 && p95(angNew) < 15 && fracBack(angNew) === 0,
  `n=${angNew.length} med=${med(angNew).toFixed(2)} p95=${p95(angNew).toFixed(2)} back=${(fracBack(angNew) * 100).toFixed(1)}%`)
chk('基准-11 修前旧式在同数据上成片反向（P95 > 90°）⇒ 该修复是真修复、不是等价改写',
  fracBack(angOld) > 0.1 && p95(angOld) > 90,
  `old p95=${p95(angOld).toFixed(2)} back=${(fracBack(angOld) * 100).toFixed(1)}%`)
// 方向符号的"断言可见性"探针：镜像后包围盒/顶点集合是否同值
const mirror = api.carPolygon(459, 323, 0).map(p => [p[0], 2 * 323 - p[1]])
const setEq = (a, b) => JSON.stringify(a.map(p => p.map(v => +v.toFixed(6))).sort()) === JSON.stringify(b.map(p => p.map(v => +v.toFixed(6))).sort())
say(`     探针：把 yaw=0 的车身四角做 y 镜像后，与原件「顶点集合」相同？ ${setEq(mirror, car0)}（所以包围盒/顶点集类断言对镜像天然盲）`)
chk('基准-12 镜像探针成立（车矩形在 y 镜像下角点集合不变）⇒ B 组只有尺寸/转动有牙，方向符号无牙',
  setEq(mirror, car0), '')

say('')
const bad = results.filter(r => !r[1]).length
say(`独立取证：${results.length - bad}/${results.length} 通过`)
fs.writeFileSync(path.join(HERE, '_tmp_verify_t5_independent.txt'), out.join('\n') + '\n', 'utf8')
process.exit(0)
