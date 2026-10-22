/**
 * W4 — CSS namespace isolation check (S6).
 *
 * Two plugin stylesheets live in the SAME main-page DOM, so cross-package
 * contamination needs BOTH of these at once:
 *   (a) a class token used by package X's DOM or selectors, and
 *   (b) a FREE-STANDING `.<token>` rule in package Y (a rule that matches any
 *       element carrying the class, without being gated by X-own classes/tags).
 * A compound modifier like `.dzag_row2.dir` cannot hit the other package's DOM
 * (its gate class `dzag_row2` is never emitted there), so compounds are safe
 * unless the OTHER side also uses the bare token — which is exactly what the
 * symmetric check below catches.
 *
 * Per package we compute:
 *   free  — class tokens in free-standing selector position (not glued to a
 *           preceding selector atom, so `.a .b` / `.a>.b` count, `.a.b`
 *           and `div.b` do not); CSS comments are stripped first.
 *   any   — every class token appearing in any selector position.
 *   emit  — bare (non-prefixed) class tokens the client emits through
 *           `className:` string literals and template-literal modifiers.
 *
 * Hazard = (free_A ∩ any_B) ∪ (any_A ∩ free_B). Empty ⇒ the two packages can
 * coexist. Exit 0 / exit 1. Usage: node scripts/check-css-namespace.mjs [old-client.js]
 *
 * Scope note: only the two client.js stylesheets matter — they are the only
 * CSS that lands in the shared main-page DOM. float.html / sim.html /
 * vnc-*.html are standalone documents on the plugin's own http origin.
 * @keyframes names are checked as a second global CSS namespace.
 */
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
const newClient = path.join(here, '..', 'client.js')
// Sibling package inside tools/plugins/ (the trees moved there on 2026-10-22;
// the old literal re-joined 'tools/plugins' below the plugin root and ENOENT'd).
const oldClient = process.argv[2] ?? path.join(here, '..', '..', 'dsh-dzrobot-ssh', 'client.js')

function extractCss(file) {
  const text = fs.readFileSync(file, 'utf8')
  const anchor = text.indexOf('installStyles')
  if (anchor < 0) throw new Error(`${file}: no installStyles()`)
  const close = text.lastIndexOf('`', anchor)
  const open = text.lastIndexOf('`', close - 1)
  return text.slice(open + 1, close).replace(/\/\*[\s\S]*?\*\//g, '')
}

function analyze(file) {
  const text = fs.readFileSync(file, 'utf8')
  const css = extractCss(file)
  const free = new Set()
  const any = new Set()
  for (const m of css.matchAll(/(^|[^\w.-])\.([A-Za-z_][A-Za-z0-9_-]*)/g)) free.add(m[2])
  for (const m of css.matchAll(/\.([A-Za-z_][A-Za-z0-9_-]*)/g)) any.add(m[1])
  const keyframes = new Set([...css.matchAll(/@keyframes\s+([\w-]+)/g)].map(m => m[1]))
  // Bare class tokens emitted through className: 'x y' strings and through
  // bare modifiers inside className template literals (`dzag_x${cond ? ' y' : ''}`).
  const emit = new Set()
  for (const m of text.matchAll(/className:\s*([`'"])([^`'"]*)\1/g)) {
    for (const tok of m[2].split(/\s+/)) if (/^[a-z][a-z0-9-]*$/.test(tok)) emit.add(tok)
  }
  for (const m of text.matchAll(/className:\s*`[^`]*`/gs)) {
    for (const q of m[0].matchAll(/['"]\s*([a-z][a-z0-9-]*)\s*['"]/g)) emit.add(q[1])
  }
  for (const tok of [...emit]) if (/^(dzag|dzr|dsw)-/.test(tok)) emit.delete(tok)
  return { free, any, emit, keyframes }
}

const A = analyze(newClient)
const B = analyze(oldClient)
const hazard = [...new Set([...A.free].filter(x => B.any.has(x) || B.emit.has(x))
  .concat([...B.free].filter(x => A.any.has(x) || A.emit.has(x))))]
const kfHazard = [...A.keyframes].filter(x => B.keyframes.has(x))

console.log(`new  dsh-dzairground-ssh : free=${A.free.size} any=${A.any.size} emit={${[...A.emit].join(', ')||'-'}} keyframes=${[...A.keyframes].join(', ')}`)
console.log(`old  dsh-plugin-dzrobot-ssh : free=${B.free.size} any=${B.any.size} emit={${[...B.emit].join(', ')||'-'}} keyframes=${[...B.keyframes].join(', ')}`)
console.log(`hazard set (${hazard.length}): ${hazard.join(', ') || '(empty)'}`)
console.log(`keyframes hazard (${kfHazard.length}): ${kfHazard.join(', ') || '(empty)'}`)
if (hazard.length > 0 || kfHazard.length > 0) {
  console.log('FAIL: shared main-page CSS identifiers between the two packages')
  process.exit(1)
}
console.log('OK: no free-standing class of either package is used by the other — coexistence safe')
