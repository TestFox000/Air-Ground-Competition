// 独立门禁复算：直接跑 sim.html 的 renderMap()，统计 SVG 里「靶」「靶板」出现次数。
// 复用 scripts/test-sim-map-render.mjs 的同一个 DOM 桩（口径一致），只改断言目标。
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath, pathToFileURL } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const ROOT = 'D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh'
const HTML = path.join(ROOT, 'sim.html')
const SIMBENCH = path.join(ROOT, 'src/simbench.js')
const { fieldSpec, routeTokens } = await import(pathToFileURL(SIMBENCH).href)
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
field.route = routeTokens()
api.setField(field)
api.renderMap()
const svg = el('map').innerHTML
const count = re => (svg.match(re) || []).length
let bad = 0
const chk = (n, ok, got) => { if (!ok) bad += 1; console.log((ok ? '  PASS ' : '  FAIL ') + n + (ok ? '' : '   got ' + got)) }

chk('D1 SVG 中 >靶板< 恰好 1 次', count(/>靶板</g) === 1, count(/>靶板</g))
chk('D2 SVG 中 >靶< 恰好 2 次（>靶板< 内含的 >靶< 计 1，加上 per-cell「靶」字 1）', count(/>靶</g) === 2, count(/>靶</g))
chk('D2b per-cell「靶」字循环已删（718-721 行不再有 per-cell 靶 label）',
  !/text\(p\[0\], p\[1\] \+ 4, '靶'/.test(body), 'legacy per-cell target label still present')
chk('D2c SVG 里出现「靶板」而不是每格一个「靶」', svg.includes('>靶板<') && count(/>靶</g) === 2, count(/>靶</g))
chk('D3 并板矩形 stroke="#8a2f5f" 恰好 1 处', count(/stroke="#8a2f5f"/g) === 1, count(/stroke="#8a2f5f"/g))
chk('D4 「横向/竖向」朝向标注恰好 1 处', count(/横向|竖向/g) === 1, count(/横向|竖向/g))
chk('D5 field.targets = ' + field.targets.join('+'), field.targets.length === 2, field.targets.join('+'))
console.log('靶板重叠 bug 核验：' + (5 - bad) + '/5 通过')
process.exit(bad ? 1 : 0)
