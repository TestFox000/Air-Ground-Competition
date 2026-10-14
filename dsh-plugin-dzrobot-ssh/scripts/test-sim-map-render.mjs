/**
 * test-sim-map-render.mjs — 仿真台「横置地图视图」的离线核对（t25）。
 *
 * 为什么需要它：地图视图是**页面里的纯前端渲染**，没有浏览器就看不见；而"14 列水平 × 11 行垂直、
 * 每格标格名、雷红/障灰/洞青/靶粉/隔板/起终/无人机蓝"这些正是验收项。这里把 `sim.html` 的内联脚本
 * 放进一个 DOM 桩里跑，喂真实的 `config/map.yaml` 解析结果（`src/simbench.js` 的 `fieldSpec()`），
 * 然后逐条断言产出的 SVG 字符串 —— 不需要浏览器、不需要起服务。
 *
 *   node scripts/test-sim-map-render.mjs      # 全过 exit 0
 */
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
if (!field.ok) {
  console.log('FAIL 读不到场地（config/map.yaml）：' + field.error)
  process.exit(1)
}
el('mines').value = field.mines.join(' ')
el('obstacles').value = field.obstacles.join(' ')
el('bridges').value = field.bridges.join(' ')

const body = fs.readFileSync(HTML, 'utf8').match(/<script>([\s\S]*?)<\/script>/)[1]
const documentStub = { getElementById: el, querySelectorAll() { return [] }, addEventListener() {} }
const windowStub = { addEventListener() {} }
const fetchStub = () => Promise.resolve({ json: () => Promise.resolve({}), text: () => Promise.resolve('') })
const api = new Function('document', 'window', 'fetch',
  'return (function(){' + body + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
  + ' cellCenter: cellCenter, fieldFromForm: fieldFromForm }; })();')(documentStub, windowStub, fetchStub)
field.route = routeTokens()          // 页面里的 FIELD 由 /sim/map 带上参考路线（index.js 里拼的）
api.setField(field)
api.renderMap()
const svg = el('map').innerHTML

const count = re => (svg.match(re) || []).length
const checks = []
const check = (name, ok, detail) => checks.push([name, ok, detail])
const CELLS = field.grid.cols * field.grid.rows

check('产出 SVG 且 viewBox 覆盖 14×11 格（477×375）',
  /viewBox="0 0 477 375"/.test(svg), (svg.match(/viewBox="[^"]+"/) || [''])[0])
check(`格背景 rect = 14×11 = ${CELLS} 个`, count(/stroke="#c9ced8"/g) === CELLS, 'rects=' + count(/stroke="#c9ced8"/g))
// ★ t28：靶板改成"一块并板"（横/竖朝向可选），多出一个朝向标注（font-size 8.5，与格名同号）
check(`每格格名标签 ${CELLS} 个（A1…N11）+ 1 个靶板朝向标注`,
  count(/font-size="8.5"/g) === CELLS + 1, 'labels=' + count(/font-size="8.5"/g))
check('四角格名都在（A1 / N1 / A11 / N11）',
  ['>A1<', '>N1<', '>A11<', '>N11<'].every(s => svg.includes(s)), 'missing 角标')
check('横置方向：A 在左（x=0）、N 在第 14 列（x=442）',
  svg.includes('<rect x="0" y="0" width="34" height="34"') && svg.includes('<rect x="442" y="0" width="34" height="34"'), 'A/N')
check('行方向：1 行在上（y=0）、11 行在下（y=340）', svg.includes('<rect x="0" y="340" width="34" height="34"'), 'row11')
check(`雷：${field.mines.length} 个红圈 + 「雷」字`,
  count(/fill="#e5484d"/g) === field.mines.length && count(/>雷</g) === field.mines.length, 'mines')
check(`障：${field.obstacles.length} 个灰块 + 「障」字`,
  count(/fill="#4a4a4a"/g) === field.obstacles.length && count(/>障</g) === field.obstacles.length, 'obstacles')
check(`桥洞：青色 + 「洞」字 ×${field.bridges.length}（= map.bridges）`,
  count(/>洞</g) === field.bridges.length && svg.includes('#7fe3ea'), 'bridges')
check('封闭桥室：K 列非开放格浅灰', svg.includes('#d9d9d9'), 'closed bridge rooms')
// ★ t28：靶板并成一块（粉色描边矩形 + 「靶板」字 + 朝向标注），不再是每格一个「靶」字
check(`靶板：一块并板（${field.targets.join('+')}）+「靶板」字`,
  count(/>靶板</g) === 1 && count(/>靶</g) === 1 && /stroke="#8a2f5f"/.test(svg), 'target board')
check('靶板朝向标注（横向/竖向）', /横向|竖向/.test(svg), 'target yaw label')
check('隔板：第 6/7 行之间 D~N 黑条 + 标注', svg.includes('隔板 D~N') && svg.includes('fill="#111111"'), 'barrier')
check('起点 N10：绿底 + 「起」', svg.includes('#b2f2bb') && svg.includes('>起<'), 'start')
check('终点 N5：金底 + 「终」', svg.includes('#ffe066') && svg.includes('>终<'), 'end')
check(`无人机起 ${field.drone.takeoff}：蓝 + 「机」`, svg.includes('>机<') && svg.includes('#4d8fd6'), 'takeoff')
check(`无人机降 ${field.drone.land}：蓝 + 「H」图标`, svg.includes('>H<'), 'land')
check('参考路线折线已画（60 token）', /<polyline[^>]+stroke="#2f7fd0"/.test(svg), 'route polyline')
check('坐标映射：N10 → x=459, y=323（右下）；A1 → (17,17)；N11 → (459,357)',
  JSON.stringify(api.cellCenter('N10')) === '[459,323]'
  && JSON.stringify(api.cellCenter('A1')) === '[17,17]'
  && JSON.stringify(api.cellCenter('N11')) === '[459,357]', 'cellCenter')
check('未改点位 → fieldFromForm() = null（请求体不带 field）', api.fieldFromForm() === null, 'null')
el('obstacles').value = field.obstacles.join(' ') + ' H8'
check('改了障碍 → fieldFromForm() 只报 obstacles', Boolean((api.fieldFromForm() || {}).obstacles), JSON.stringify(api.fieldFromForm()))

let bad = 0
for (const [name, ok, detail] of checks) {
  if (!ok) bad += 1
  console.log((ok ? '  PASS ' : '  FAIL ') + name + (ok ? '' : '   → ' + detail))
}
console.log('横置地图渲染核对：' + (checks.length - bad) + '/' + checks.length + ' 通过')
process.exit(bad ? 1 : 0)
