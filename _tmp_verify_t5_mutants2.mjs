/**
 * t5（sim-bench2 / verifier）退化注入取证 —— 用「唯一能替换被测实现的一方」的手法做变异。
 *
 * 为什么不在测试文件里内置变异：交接清单 §4 已记档「断言不在被自证的闭包里」。
 * 这里做的等价外部手法：把插件三件套（sim.html / src/simbench.js / scripts 测试 / package.json）
 * 拷到 `_tmp_verify_t5_mutant/`（父目录仍是 dzrobot_v2 ⇒ SIM_ROOT 解析正确）→ 只改副本里的
 * sim.html → 跑**官方那份断言**（逐字未改）→ 看哪几条从 PASS 变 FAIL。跑完删除副本。
 *
 * 只读原仓库；证书输出落 _tmp_verify_t5_mutants2.txt。
 */
import fs from 'node:fs'
import path from 'node:path'
import { execFileSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const SRC = path.join(HERE, 'dsh-plugin-dzrobot-ssh')
const MUT = path.join(HERE, '_tmp_verify_t5_mutant')
const origHtml = fs.readFileSync(path.join(SRC, 'sim.html'), 'utf8')
const log = []
const say = (...a) => { const l = a.join(' '); log.push(l); console.log(l) }
const results = []

function prepareCopy() {
  fs.rmSync(MUT, { recursive: true, force: true })
  fs.mkdirSync(path.join(MUT, 'src'), { recursive: true })
  fs.mkdirSync(path.join(MUT, 'scripts'), { recursive: true })
  for (const rel of ['package.json', 'src/simbench.js', 'scripts/test-sim-map-render.mjs']) {
    fs.copyFileSync(path.join(SRC, rel), path.join(MUT, rel))
  }
  fs.writeFileSync(path.join(MUT, 'sim.html'), origHtml, 'utf8')
}
function runOfficial() {
  try {
    const outText = execFileSync('node', [path.join(MUT, 'scripts', 'test-sim-map-render.mjs')],
      { encoding: 'utf8', cwd: MUT, env: { ...process.env } }).toString()
    const total = (outText.match(/核对：(\d+)\/(\d+)/) || [])[1]
    return { code: 0, fails: [], total: (outText.match(/核对：(\d+\/\d+)/) || [])[1] }
  } catch (e) {
    const outText = String(e.stdout ?? '') + String(e.stderr ?? '')
    const fails = outText.split('\n').filter(l => l.trim().startsWith('FAIL ')).map(l => l.trim().slice(5))
    const total = (outText.match(/核对：(\d+\/\d+)/) || [])[1]
    return { code: e.status ?? 1, fails, total }
  }
}

const CASES = [
  {
    id: 'M-A',
    group: 'A 路径折线',
    desc: '删掉 routePolyline 的补点分支（跨格不再补中间格心）',
    apply: t => t.replace(/    if \(Math\.abs\(dc\) > 1 && Math\.abs\(dr\) > 1\) \{[\s\S]*?\n    \}\n    out\.push\(p\);/, '    out.push(p);'),
    needle: 'out.push(p);',
  },
  {
    id: 'M-B',
    group: 'B 位姿',
    desc: '把 carPolygon 的 yaw 项固定（车身不再随 yaw 转动）',
    apply: t => t.replace('  var h = headVec(yawCode);', '  var h = headVec(0);'),
    needle: 'var h = headVec(0);',
  },
  {
    id: 'M-C',
    group: 'C 靶朝向',
    desc: '把并板朝向标注的自动判定反过来（h/v 反转）',
    apply: t => t.replace("var mode = targetYawMode() || ((r[2] >= r[3]) ? 'h' : 'v');",
      "var mode = targetYawMode() || ((r[2] >= r[3]) ? 'v' : 'h');"),
    needle: "? 'v' : 'h');",
  },
  {
    id: 'M-D',
    group: 'B 位姿（探针·非合同要求）',
    desc: '把 headVec 的 -sin 去掉 = 回退 t1 本轮修掉的车头上下镜像 bug',
    apply: t => t.replace('  return [Math.cos(th), -Math.sin(th)];', '  return [Math.cos(th), Math.sin(th)];'),
    needle: 'return [Math.cos(th), Math.sin(th)];',
  },
  {
    id: 'M-E',
    group: 'B 位姿（探针）',
    desc: '把 codeToScreen 回退成 t1 修前的半格口径（轨迹/车身整体偏半格）',
    apply: t => t.replace(/  var col = cols - y \/ FIELD_GRID_BLOCK;[\s\S]*?\n  return \[col \* MAP_S, row \* MAP_S\];/,
      '  var col = 13.5 - y / FIELD_GRID_BLOCK;\n  var row = 10.5 - x / FIELD_GRID_BLOCK;\n  return [col * MAP_S, row * MAP_S];'),
    needle: '13.5 - y / FIELD_GRID_BLOCK',
  },
  {
    id: 'M-F',
    group: 'B 位姿（R1/R2 相位）',
    desc: '车身抽样步长 125 → 500（相位退化回每 10 s 一个）',
    apply: t => t.replace('for (var j = 1; j < lines.length; j += 125)', 'for (var j = 1; j < lines.length; j += 500)'),
    needle: 'j += 500',
  },
  {
    id: 'M-G',
    group: 'R5 靶格底色',
    desc: '删掉靶格粉底填充（靶格静默退化成白底）',
    apply: t => t.replace("    if (targetSet[cell]) fill = '#e8a0c8';", '    if (false) fill = \'#e8a0c8\';'),
    needle: "if (false) fill = '#e8a0c8';",
  },
  {
    id: 'M-H',
    group: 'F3 事件绑定（合同未要求有牙）',
    desc: '删掉 #targetYaw 的 change 监听（F3 回归）',
    apply: t => t.replace(/^\$\('targetYaw'\)\.addEventListener\('change'[^\n]*\n/gm, ''),
    needle: null,
  },
  {
    id: 'M-I',
    group: 'F1 靶板残留（回归）',
    desc: '把 per-cell「靶」字循环加回去（F1 回归）',
    apply: t => t.replace('  // ★ t28：靶板按"一块并板"画',
      "  FIELD.targets.forEach(function (cell) {\n    var p = cellCenter(cell); if (!p) return;\n    svg.push(text(p[0], p[1] + 4, '靶', 11, '#8a2f5f', 'bold', 'middle'));\n  });\n  // ★ t28：靶板按\"一块并板\"画"),
    needle: "text(p[0], p[1] + 4, '靶'",
  },
]

say('基线（未变异副本）——确认副本本身能跑通官方断言：')
prepareCopy()
const base = runOfficial()
say(`  副本官方测试：exit=${base.code}  ${base.total ?? '?'}  失败条数=${base.fails.length}`)
if (base.code !== 0) { say('  ⚠ 副本基线不绿，无法做变异对照'); process.exit(1) }
results.push(['副本基线与原仓库一致（exit 0）', base.code === 0])
say('')

for (const c of CASES) {
  const mutated = c.apply(origHtml)
  const applied = mutated !== origHtml && (c.needle === null || mutated.includes(c.needle))
  if (!applied) { say(`${c.id} 变异未生效（文本没匹配上）——取证脚本自身问题`); results.push([`${c.id} 变异生效`, false]); continue }
  fs.writeFileSync(path.join(MUT, 'sim.html'), mutated, 'utf8')
  const r = runOfficial()
  const caught = r.code !== 0 && r.fails.length > 0
  say(`${c.id} [${c.group}] ${c.desc}`)
  say(`    变异已生效=${applied} → 官方测试 exit=${r.code} ${r.total ?? '?'} ${caught ? '变红条数=' + r.fails.length : '**没有任何断言变红**'}`)
  if (r.fails.length) r.fails.slice(0, 4).forEach(f => say(`      · ${f}`))
  results.push([`${c.id} ${c.group}`, caught])
  say('')
  fs.writeFileSync(path.join(MUT, 'sim.html'), origHtml, 'utf8')      // 还原，供下一个用例
}

// 结论：合同要求的三组（A/B/C 各 ≥1 个真实退化）必须都变红
const contractCases = results.filter(r => /^M-[ABC] /.test(r[0]))
const allTooth = contractCases.every(r => r[1])
say(`合同要求：A/B/C 三组各注入 1 个真实退化 → ${allTooth ? '三组全部有牙（断言真的变红）' : '存在「注入退化后断言仍绿」的组'}`)
for (const id of ['M-D', 'M-E', 'M-F', 'M-G', 'M-H', 'M-I']) {
  const r = results.find(x => x[0].startsWith(id))
  if (r) say(`探针 ${id}（${r[0].split(' ').slice(1).join(' ')}）：被官方断言抓住=${r[1]}`)
}

fs.rmSync(MUT, { recursive: true, force: true })
fs.writeFileSync(path.join(HERE, '_tmp_verify_t5_mutants2.txt'), log.join('\n') + '\n', 'utf8')
console.log(`\nmutant dir removed=${!fs.existsSync(MUT)}`)
process.exit(allTooth ? 0 : 1)
