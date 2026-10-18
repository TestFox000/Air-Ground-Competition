#!/usr/bin/env node
/* t7 只读复核：页面 planRoute（线上 argv）↔ 车端 mission_blocks 逐 token 对账（现役快照）
 * 不锚死 44；锚死「同一状态下 page == car」。
 */
import fs from 'node:fs'
import path from 'node:path'
import { pathToFileURL } from 'node:url'

const ROOT = 'D:/User/Desktop/比赛存档/v1/dzrobot_v2'
const TDIR = 'D:/User/Desktop/比赛存档/v1/_tmp_session/t7_review'
const sb = await import(pathToFileURL(path.join(ROOT, 'dsh-plugin-dzrobot-ssh/src/simbench.js')).href)
const { planRoute } = sb
const car = JSON.parse(fs.readFileSync(path.join(TDIR, '40_car_tokens.json'), 'utf8')).car
const MAP = path.join(ROOT, 'config/map.yaml')

const modes = [
  ['live-config (engine 由配置决定)', path.join(ROOT, 'config/robot.yaml')],
  ['cruise 0.15', path.join(TDIR, 'robot_015.yaml')],
  ['cruise 0.35', path.join(TDIR, 'robot_035.yaml')],
  ['cruise 0.40', path.join(TDIR, 'robot_040.yaml')],
]

const out = []
for (const [label, cfg] of modes) {
  const r = await planRoute({ configPath: cfg, mapPath: MAP })
  const tok = r.tokens || []
  let firstDiff = -1
  for (let i = 0; i < Math.max(tok.length, car.length); i += 1) if (tok[i] !== car[i]) { firstDiff = i; break }
  const rec = { label, engine: r.engine, exitCode: r.exitCode, checkExit: r.check?.exit ?? null, page: tok.length, car: car.length, firstDiff, equal: firstDiff < 0 && tok.length === car.length }
  out.push(rec)
  console.log(`${rec.equal ? 'PASS' : 'FAIL'} ${label} | engine=${rec.engine} exit=${rec.exitCode} check=${rec.checkExit} page=${rec.page} car=${rec.car} firstDiff=${rec.firstDiff}`)
  if (!rec.equal) console.log('  page tokens: ' + tok.join(' '))
}
fs.writeFileSync(path.join(TDIR, '41_parity_live.json'), JSON.stringify({ car, out }, null, 1), 'utf8')
const bad = out.filter(o => !o.equal)
console.log(`\n结果：${out.length - bad.length}/${out.length} 逐 token 相等`)
process.exit(bad.length ? 1 : 0)
