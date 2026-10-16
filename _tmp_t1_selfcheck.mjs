// t1 自验（只读、不写仓库）：用真实 config/map.yaml + 真实 route token 串，
// 在 DOM 桩里跑 sim.html 的 renderMap()，核验三项改进 + 靶字残留。
// 与官方 scripts/test-sim-map-render.mjs 的区别：这里另外注入 TRACE（折线/位姿路径）。
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { fieldSpec, routeTokens } from '../src/simbench.js'

const HERE = fileURLToPath(new URL('.', import.meta.url))
const HTML = path.join(HERE, '..', 'sim.html')

const els = {}
function el(id) {
  if (!els[id]) {
    els[id] = {
      id, value: '', checked: id === 'showRoute', innerHTML: '', textContent: '', className: '',
      style: {}, parentNode: null, addEventListener() {},
      getElementsByTagName() { return [{ innerHTML: '' }] },
      querySelectorAll() { return [] },
      getAttribute() { return '' },
    }
  }
  return els[id]
}

const field = fieldSpec()
if (!field.ok) { console.log('FAIL 读不到场地：' + field.error); process.exit(1) }
el('mines').value = field.mines.join(' ')
el('obstacles').value = field.obstacles.join(' ')
el('bridges').value = field.bridges.join(' ')

const body = fs.readFileSync(HTML, 'utf8').match(/<script>([\s\S]*?)<\/script>/)[1]
const documentStub = { getElementById: el, querySelectorAll() { return [] }, addEventListener() {} }
const windowStub = { addEventListener() {} }
const fetchStub = () => Promise.resolve({ json: () => Promise.resolve({}), text: () => Promise.resolve('') })
const api = new Function('document', 'window', 'fetch',
  'return (function(){' + body + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
  + ' setTrace: function(t){ TRACE = t; }, cellCenter: cellCenter, codeToScreen: codeToScreen,'
  + ' routePolyline: routePolyline, carPolygon: carPolygon, targetBoardRect: targetBoardRect,'
  + ' targetYawMode: function(){ return targetYawMode(); }, fieldFromForm: fieldFromForm }; })();'
  )(documentStub, windowStub, fetchStub)

field.route = routeTokens()
api.setField(field)
api.renderMap()
const svg = el('map').innerHTML
const count = re => (svg.match(re) || []).length

let bad = 0
const chk = (name, ok, got) => { if (!ok) bad += 1; console.log((ok ? '  PASS ' : '  FAIL ') + name + (ok ? '' : '   → ' + got)) }

// ---- 0. 靶字残留（本卡 F1）----
chk('SVG 中 >靶< 出现次数（删循环后应为 1）', count(/>靶</g) === 1, '>靶< ×' + count(/>靶</g))
chk('SVG 中 >靶板< 出现次数 = 1', count(/>靶板</g) === 1, '>靶板< ×' + count(/>靶板</g))
chk('靶板并板矩形描边 #8a2f5f = 1', /stroke="#8a2f5f"/.test(svg), 'stroke count=' + count(/stroke="#8a2f5f"/g))
chk('靶板朝向标注（横向/竖向）存在', /横向|竖向/.test(svg), 'none')

// ---- ① 路径折线 ----
const toks = field.route
chk('route token 数 > 0（config/robot.yaml）', toks.length > 0, 'tokens=' + toks.length)
const cells = toks.map(t => String(t).trim().replace(/^R:/i, '').replace(/[,:].*$/, '').replace(/\*$/, '').trim().toUpperCase())
  .filter(c => /^[A-N](?:[1-9]|1[01])$/.test(c))
const pts = api.routePolyline(cells)
// 逐段统计：相邻两格间的补点数
let multi = 0, diag = 0
for (let i = 1; i < cells.length; i += 1) {
  const a = api.cellCenter(cells[i - 1]), b = api.cellCenter(cells[i])
  const dc = Math.abs(b[0] - a[0]) / 34, dr = Math.abs(b[1] - a[1]) / 34
  if (dc > 1 || dr > 1) multi += 1
  if (dc >= 1 && dr >= 1) diag += 1
}
console.log('     routePolyline 点数 = ' + pts.length + '（格心 ' + cells.length + ' 个；跨多格/对角段 ' + multi + '/' + (cells.length - 1) + '，对角 ' + diag + '）')
chk('routePolyline 点数列入 SVG polyline（stroke #2f7fd0）', /<polyline[^>]+stroke="#2f7fd0"/.test(svg), 'none')
// 折线点数必须 > 格心数（说明补了中间点）
chk('routePolyline 补点生效（pts > cells）', pts.length > cells.length, pts.length + ' vs ' + cells.length)
// 每个 polyline 点都必须是某个格心（补点落在格心网格上）
const allGrid = pts.every(p => Math.abs((p[0] % 34) - 17) < 1e-6 && Math.abs((p[1] % 34) - 17) < 1e-6)
chk('所有折线点落在格心网格（x%34=17 且 y%34=17）', allGrid, 'off-grid point: ' + JSON.stringify(pts.find(p => Math.abs((p[0] % 34) - 17) > 1e-6 || Math.abs((p[1] % 34) - 17) > 1e-6)))
// 专项：跨多格段（A1→D1 横向 3 格；N11→N8 纵向 4 格）逐格补点
const pH = api.routePolyline(['A1', 'D1'])
chk('横向跨 3 格 A1→D1 = 4 点（补 B1/C1 格心）', pH.length === 4 && pH[1][0] === 51 && pH[2][0] === 85, JSON.stringify(pH))
const pV = api.routePolyline(['N11', 'N8'])
chk('纵向跨 4 格 N11→N8 = 4 点（补 N10/N9 格心）', pV.length === 4 && pV[1][1] === 323 && pV[2][1] === 289, JSON.stringify(pV))
chk('纵向补点列不变（x 恒 = 459）', pV.every(q => q[0] === 459), JSON.stringify(pV))

// ---- ② 位姿标注 ----
// codeToScreen 与 cellCenter 互逆（口径一致）
const inv = c => { const p = api.cellCenter(c); const b = api.codeToScreen((10.5 - p[1] / 34) * 0.35, (13.5 - p[0] / 34) * 0.35); return Math.abs(b[0] - p[0]) < 1e-9 && Math.abs(b[1] - p[1]) < 1e-9 }
chk('codeToScreen ∘ cellCenter 互逆（N10/A1/K7/E5）',
  ['N10', 'A1', 'K7', 'E5'].every(inv), 'not inverse')
// N10 起点：truth(0.525, 0.175) → 屏幕 (459, 323) = N10 格心
const s10 = api.codeToScreen(0.525, 0.175)
chk('N10 起点 codeToScreen(0.525,0.175) = (459,323) = N10 格心',
  s10[0] === 459 && s10[1] === 323, JSON.stringify(s10))
// 车长 0.28 m / 车宽 0.135 m → 27.2 px / 13.114 px
const poly = api.carPolygon(459, 323, 0)
const d01 = Math.hypot(poly[1][0] - poly[0][0], poly[1][1] - poly[0][1])
const d12 = Math.hypot(poly[2][0] - poly[1][0], poly[2][1] - poly[1][1])
chk('carPolygon 车宽 ≈ 13.11px、车长 ≈ 27.2px', Math.abs(d01 - 13.114285714285714) < 1e-6 && Math.abs(d12 - 27.2) < 1e-6,
  'd01=' + d01 + ' d12=' + d12)
// yaw=+90° 应朝 -x（图左）：官方 S() 变换 θ_s = θ+90°，代码 yaw=90° → 屏幕 180° → 朝左
const p90 = api.carPolygon(459, 323, Math.PI / 2)
const headX = (p90[0][0] + p90[1][0]) / 2 - (p90[2][0] + p90[3][0]) / 2   // 车头中心 - 车尾中心
chk('carPolygon yaw=+90° 车头朝 -x（图左）', headX < -20, 'headX=' + headX)
// TRACE.poses 注入 → 车身矩形 + 箭头 + 「起点位姿」标注
api.setTrace({
  route: field.route,
  points: [[459, 323], [425, 323], [391, 323]],
  poses: [[459, 323, 0], [425, 323, 0], [391, 323, 0]],
})
api.renderMap()
const svg2 = el('map').innerHTML
chk('TRACE.poses → 车身矩形 path（fill #1f2933）×3',
  count2(svg2, /fill="#1f2933"/g) === 3, '×' + count2(svg2, /fill="#1f2933"/g))
chk('TRACE.poses → 车头箭头 line（stroke #0b1727）×3',
  count2(svg2, /stroke="#0b1727"/g) === 3, '×' + count2(svg2, /stroke="#0b1727"/g))
chk('「起点位姿」标注存在', svg2.includes('>起点位姿<'), 'none')
chk('TRACE.points → 仿真轨迹 polyline（stroke #d64545）', /<polyline[^>]+stroke="#d64545"/.test(svg2), 'none')

// ---- ③ 靶板朝向 ----
const rAuto = api.targetBoardRect(field.targets, '')
chk('targetBoardRect(E5+F5) auto = 68×34 横向', rAuto[2] === 68 && rAuto[3] === 34, JSON.stringify(rAuto))
const rV = api.targetBoardRect(['E5', 'E6'], '')
chk('targetBoardRect(E5+E6) auto = 34×68 竖向', rV[2] === 34 && rV[3] === 68, JSON.stringify(rV))
const rForceV = api.targetBoardRect(field.targets, 'v')
chk('指定 v 不改矩形尺寸（仍 68×34）', rForceV[2] === 68 && rForceV[3] === 34, JSON.stringify(rForceV))
// select 控件选项
chk('select#targetYaw 有 h/v 两个可选值', /value="h"/.test(body) && /value="v"/.test(body), 'missing options')
// 切换 select → 朝向标注文字变化
function count2(s, re) { return (s.match(re) || []).length }
el('targetYaw').value = 'v'
api.renderMap()
const svg3 = el('map').innerHTML
chk('select=v → 图上出现「竖向」标注', svg3.includes('>竖向'), 'none')
chk('select=v → 图上不再出现「横向」标注', !svg3.includes('>横向'), 'still horizontal')
chk('select=v 时并板矩形尺寸不变（仍 stroke #8a2f5f 1 个）', count2(svg3, /stroke="#8a2f5f"/g) === 1, '×' + count2(svg3, /stroke="#8a2f5f"/g))
el('targetYaw').value = ''

console.log('t1 自验：' + (bad === 0 ? '全部通过' : (countChecks() - bad) + '/' + countChecks() + ' 通过'))
function countChecks() { return 0 }
process.exit(bad ? 1 : 0)
