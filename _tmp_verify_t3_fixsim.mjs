// 独立复验：模拟「删 sim.html:718-721 的 per-cell 靶循环」后，官方测试第 79 行断言是否仍成立。
// 不碰仓库：只在内存里替换 body 字符串。
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

const raw = fs.readFileSync(HTML, 'utf8')
const LEGACY = `  FIELD.targets.forEach(function (cell) {
    var p = cellCenter(cell); if (!p) return;
    svg.push(text(p[0], p[1] + 4, '靶', 11, '#8a2f5f', 'bold', 'middle'));
  });
`
console.log('源码中可定位到 per-cell 靶循环块：' + raw.includes(LEGACY))

const field = fieldSpec()
el('mines').value = field.mines.join(' ')
el('obstacles').value = field.obstacles.join(' ')
el('bridges').value = field.bridges.join(' ')

function render(bodySrc) {
  const body = bodySrc.match(/<script>([\s\S]*?)<\/script>/)[1]
  const documentStub = { getElementById: el, querySelectorAll() { return [] }, addEventListener() {} }
  const windowStub = { addEventListener() {} }
  const fetchStub = () => Promise.resolve({ json: () => Promise.resolve({}), text: () => Promise.resolve('') })
  const api = new Function('document', 'window', 'fetch',
    'return (function(){' + body + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
    + ' cellCenter: cellCenter, fieldFromForm: fieldFromForm }; })();')(documentStub, windowStub, fetchStub)
  field.route = routeTokens()
  api.setField(field)
  api.renderMap()
  return el('map').innerHTML
}
const count = (s, re) => (s.match(re) || []).length

for (const [label, src] of [['现状（未删循环）', raw], ['模拟修复（删 718-721）', raw.replace(LEGACY, '')]]) {
  const svg = render(src)
  console.log('\n=== ' + label + ' ===')
  console.log('  >靶板<      = ' + count(svg, />靶板</g) + '   (官方测试要求 === 1)')
  console.log('  >靶<        = ' + count(svg, />靶</g) + '   (官方测试要求 === 1)')
  console.log('  stroke="#8a2f5f" = ' + count(svg, /stroke="#8a2f5f"/g))
  console.log('  横向|竖向    = ' + count(svg, /横向|竖向/g))
  const line79 = count(svg, />靶板</g) === 1 && count(svg, />靶</g) === 1 && /stroke="#8a2f5f"/.test(svg)
  console.log('  → 官方测试 L79 断言：' + (line79 ? 'PASS' : 'FAIL'))
}
