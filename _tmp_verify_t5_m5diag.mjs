// M5 未抓住的原因定位：变异是否真生效？targetBoardRect(['E5','E6'],'') 的真实输出是什么？
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
  if (!els[id]) els[id] = {
    id, value: '', checked: id === 'showRoute', innerHTML: '', textContent: '', className: '',
    style: {}, parentNode: null, addEventListener() {},
    getElementsByTagName() { return [{ innerHTML: '' }] },
    querySelectorAll() { return [] }, getAttribute() { return '' },
  }
  return els[id]
}
const field = fieldSpec()
el('mines').value = field.mines.join(' ')
el('obstacles').value = field.obstacles.join(' ')
el('bridges').value = field.bridges.join(' ')
const raw = fs.readFileSync(HTML, 'utf8')

function buildApi(bodySrc) {
  const body = bodySrc.match(/<script>([\s\S]*?)<\/script>/)[1]
  const documentStub = { getElementById: el, querySelectorAll() { return [] }, addEventListener() {} }
  const windowStub = { addEventListener() {} }
  const fetchStub = () => Promise.resolve({ json: () => Promise.resolve({}), text: () => Promise.resolve('') })
  return new Function('document', 'window', 'fetch',
    'return (function(){' + body + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
    + ' cellCenter: cellCenter, targetBoardRect: targetBoardRect }; })();'
  )(documentStub, windowStub, fetchStub)
}

const FROM = "  if (!mode) mode = (x1 - x0) >= (y1 - y0) ? 'h' : 'v';"
const TO = "  if (!mode) mode = 'h';"
console.log('补丁锚点命中：' + raw.includes(FROM))

for (const [label, src] of [['未变异', raw], ['变异后', raw.replace(FROM, TO)]]) {
  const api = buildApi(src)
  field.route = routeTokens()
  api.setField(field)
  console.log('\n=== ' + label + ' ===')
  for (const cells of [['E5', 'F5'], ['E5', 'E6'], ['E6', 'E5']]) {
    console.log('  targetBoardRect(' + JSON.stringify(cells) + ", '') = " + JSON.stringify(api.targetBoardRect(cells, '')))
  }
  console.log("  targetBoardRect(['E5','F5'], 'h') = " + JSON.stringify(api.targetBoardRect(['E5', 'F5'], 'h')))
  console.log("  targetBoardRect(['E5','F5'], 'v') = " + JSON.stringify(api.targetBoardRect(['E5', 'F5'], 'v')))
}
