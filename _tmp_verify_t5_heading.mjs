/**
 * t5 取证：车头方向交叉验证的**样本选择**探针。
 * 目的：先用多个真 traj.csv 对照，确认「修后/修前」的差异是系统的，而不是某个注入 run 的伪影。
 * 只读。
 */
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PLUGIN = path.join(HERE, 'dsh-plugin-dzrobot-ssh')
const body = fs.readFileSync(path.join(PLUGIN, 'sim.html'), 'utf8').match(/<script>([\s\S]*?)<\/script>/)[1]
const els = {}
const el = id => (els[id] = els[id] || {
  id, value: '', checked: false, innerHTML: '', textContent: '', className: '', style: {},
  addEventListener() {}, getElementsByTagName() { return [{ innerHTML: '' }] }, querySelectorAll() { return [] }, getAttribute() { return '' },
})
const api = new Function('document', 'window', 'fetch', 'location',
  'return (function(){' + body + '\n; return { codeToScreen: codeToScreen, headVec: headVec }; })();'
)({ getElementById: el, querySelectorAll() { return [] }, addEventListener() {} }, { addEventListener() {} },
  () => Promise.resolve({ json: () => Promise.resolve({}), text: () => Promise.resolve('') }), { hash: '' })

const runsDir = path.join(HERE, 'artifacts', 'sim')
const med = a => { const s = [...a].sort((p, q) => p - q); return s.length ? s[Math.floor(s.length / 2)] : NaN }
const rowsOut = []
for (const d of fs.readdirSync(runsDir)) {
  const f = path.join(runsDir, d, 'traj.csv')
  if (!fs.existsSync(f)) continue
  const lines = fs.readFileSync(f, 'utf8').trim().split('\n')
  const head = lines[0].split(',')
  const ix = head.indexOf('truth_x'), iy = head.indexOf('truth_y'), iw = head.indexOf('truth_yaw')
  const iv = head.indexOf('v'), irev = head.indexOf('reverse')
  if (ix < 0 || iy < 0 || iw < 0) continue
  const rows = lines.slice(1).map(l => l.split(',').map(Number))
  const an = [], ao = []
  const STEP = 125
  for (let i = 0; i + STEP < rows.length; i += STEP) {
    const a = rows[i], b = rows[i + STEP]
    const yaw = a[iw], v = iv >= 0 ? a[iv] : 1, rev = irev >= 0 ? a[irev] : 0
    if (!isFinite(yaw) || !isFinite(v) || v <= 0.05 || rev === 1) continue
    const s0 = api.codeToScreen(a[ix], a[iy]), s1 = api.codeToScreen(b[ix], b[iy])
    const dx = s1[0] - s0[0], dy = s1[1] - s0[1], len = Math.hypot(dx, dy)
    if (!(len > 10)) continue
    const dir = [dx / len, dy / len], th = yaw + Math.PI / 2
    const ang = h => Math.acos(Math.max(-1, Math.min(1, h[0] * dir[0] + h[1] * dir[1]))) * 180 / Math.PI
    an.push(ang(api.headVec(yaw))); ao.push(ang([Math.cos(th), Math.sin(th)]))
  }
  rowsOut.push({ d, rows: rows.length, n: an.length, medNew: med(an), medOld: med(ao) })
}
rowsOut.sort((a, b) => b.n - a.n)
console.log('dir                            rows    n   medNew   medOld')
for (const r of rowsOut.slice(0, 14)) {
  console.log(`${r.d.padEnd(30)} ${String(r.rows).padStart(6)} ${String(r.n).padStart(4)}  ${r.medNew.toFixed(2).padStart(6)}  ${r.medOld.toFixed(2).padStart(6)}`)
}
