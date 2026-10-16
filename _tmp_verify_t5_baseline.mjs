// t5 预核：不采信队长自报，直接用真实 sim.html 里的函数重算基准数据。
// 做法与官方测试同一个 DOM 桩，但把 routePolyline/carPolygon/targetBoardRect 也 return 出来。
// 只读，零仓库写入。
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
  + ' cellCenter: cellCenter, fieldFromForm: fieldFromForm, routePolyline: routePolyline,'
  + ' carPolygon: carPolygon, targetBoardRect: targetBoardRect, targetYawMode: targetYawMode,'
  + ' CAR_LEN_PX: CAR_LEN_PX, CAR_WID_PX: CAR_WID_PX, MAP_S: MAP_S }; })();'
)(documentStub, windowStub, fetchStub)
field.route = routeTokens()
api.setField(field)

let bad = 0
const eq = (name, got, want) => {
  const ok = JSON.stringify(got) === JSON.stringify(want)
  if (!ok) bad += 1
  console.log((ok ? '  MATCH ' : '  DIFF  ') + name + (ok ? '' : '\n          got  ' + JSON.stringify(got) + '\n          want ' + JSON.stringify(want)))
}
const J = v => JSON.stringify(v)

console.log('=== 常量（真实代码） ===')
console.log('  MAP_S=' + api.MAP_S + '  CAR_LEN_PX=' + api.CAR_LEN_PX + '  CAR_WID_PX=' + api.CAR_WID_PX)
eq('CAR_LEN_PX = 0.28*34/0.35', api.CAR_LEN_PX, 0.28 * 34 / 0.35)
eq('CAR_WID_PX = 0.135*34/0.35', api.CAR_WID_PX, 0.135 * 34 / 0.35)

console.log('\n=== 基础格心 ===')
for (const c of ['A1', 'E5', 'F5', 'E6', 'N10', 'N11', 'N8', 'D1', 'B1', 'C1', 'N9']) {
  console.log('  cellCenter(' + c + ') = ' + J(api.cellCenter(c)))
}
eq('A1', api.cellCenter('A1'), [17, 17])
eq('E5', api.cellCenter('E5'), [153, 153])
eq('F5', api.cellCenter('F5'), [187, 153])
eq('E6', api.cellCenter('E6'), [153, 187])
eq('N10', api.cellCenter('N10'), [459, 323])
eq('N11', api.cellCenter('N11'), [459, 357])
eq('N8', api.cellCenter('N8'), [459, 255])

console.log('\n=== routePolyline（队长基准） ===')
const p1 = api.routePolyline(['A1', 'D1'])
const p2 = api.routePolyline(['N11', 'N8'])
console.log('  A1→D1  = ' + J(p1))
console.log('  N11→N8 = ' + J(p2))
eq('routePolyline([A1,D1])', p1, [[17, 17], [51, 17], [85, 17], [119, 17]])
eq('routePolyline([N11,N8])', p2, [[459, 357], [459, 323], [459, 289], [459, 255]])
eq('A1→D1 中间点即 B1/C1 格心', [api.cellCenter('B1'), api.cellCenter('C1')], [[51, 17], [85, 17]])
eq('N11→N8 中间点即 N10/N9 格心', [api.cellCenter('N10'), api.cellCenter('N9')], [[459, 323], [459, 289]])
// 折线真的补了中间点（不是直线连首末）：点数 > 2
const polyA1D1 = p1.length > 2
const polyN11N8 = p2.length > 2
eq('A1→D1 补了点（点数>2 ⇒ 非直线连首末）', polyA1D1, true)
eq('N11→N8 补了点（点数>2 ⇒ 非直线连首末）', polyN11N8, true)

console.log('\n=== carPolygon（队长基准：包围盒 ≈ 27.2 × 13.11） ===')
const cp = api.carPolygon(459, 323, 0)
console.log('  carPolygon(459,323,0) = ' + J(cp.map(p => p.map(v => Math.round(v * 1000) / 1000))))
const xs = cp.map(p => p[0]), ys = cp.map(p => p[1])
const bw = (Math.max(...xs) - Math.min(...xs)), bh = (Math.max(...ys) - Math.min(...ys))
console.log('  包围盒 宽=' + bw.toFixed(4) + ' 高=' + bh.toFixed(4) + '  ⇒ 27.2 × 13.11 两种排列都覆盖')
eq('包围盒宽 = CAR_WID_PX', Math.abs(bw - api.CAR_WID_PX) < 1e-9, true)
eq('包围盒高 = CAR_LEN_PX', Math.abs(bh - api.CAR_LEN_PX) < 1e-9, true)
eq('yaw=0 时车头朝屏幕下方（长轴竖直）', bh > bw, true)

console.log('\n=== targetBoardRect（队长基准） ===')
const r1 = api.targetBoardRect(['E5', 'F5'], '')
const r2 = api.targetBoardRect(['E5', 'E6'], '')
const r3 = api.targetBoardRect(['E5', 'F5'], 'v')
const r4 = api.targetBoardRect(['E5', 'F5'], 'h')
console.log("  (['E5','F5'],'')  = " + J(r1) + '  (横向 68×34)')
console.log("  (['E5','E6'],'')  = " + J(r2) + '  (竖向 34×68)')
console.log("  (['E5','F5'],'v') = " + J(r3))
console.log("  (['E5','F5'],'h') = " + J(r4))
eq("targetBoardRect(['E5','F5'],'')", r1, [136, 136, 68, 34])
eq("targetBoardRect(['E5','E6'],'')", r2, [136, 136, 34, 68])
eq("targetBoardRect(['E5','F5'],'v') 矩形不变", r3, [136, 136, 68, 34])
eq("targetBoardRect(['E5','F5'],'h') 矩形不变", r4, [136, 136, 68, 34])
eq("单格退回整格 34×34", api.targetBoardRect(['E5'], ''), [136, 136, 34, 34])

console.log('\n队长基准数据独立重算：' + (bad === 0 ? '全部一致 ✓' : bad + ' 处不一致 ✗'))
process.exit(bad ? 1 : 0)
