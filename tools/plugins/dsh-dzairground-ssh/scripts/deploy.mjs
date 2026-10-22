/**
 * Sync the workspace plugin tree into the DSH profile install.
 *
 * Copy-only, never delete from the source, and it *reports* what the source tree
 * no longer has so a human decides — that is the deletion that bit us once
 * (package.json silently missing from the install, which reads as "the plugin
 * was deleted" with no error anywhere).
 *
 * Run: node scripts/deploy.mjs
 */
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const root = path.join(path.dirname(fileURLToPath(import.meta.url)), '..')
const dshHome = process.env.DSH_HOME ?? path.join(process.env.USERPROFILE ?? '', '.dsh')
const profile = process.env.DZR_PROFILE ?? 'desktop'
const installed = path.join(dshHome, 'profiles', profile, 'node_modules', 'dsh-dzairground-ssh')

if (!fs.existsSync(path.join(root, 'package.json'))) {
  console.error('source tree has no package.json — refusing to sync an incomplete tree')
  process.exit(1)
}

const walk = (dir, base = dir, acc = []) => {
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    const full = path.join(dir, entry.name)
    if (entry.isDirectory()) walk(full, base, acc)
    else acc.push(path.relative(base, full).replace(/\\/g, '/'))
  }
  return acc
}

const files = walk(root)
let copied = 0, unchanged = 0
for (const rel of files) {
  const from = path.join(root, rel), to = path.join(installed, rel)
  if (fs.existsSync(to) && fs.readFileSync(from).equals(fs.readFileSync(to))) { unchanged++; continue }
  fs.mkdirSync(path.dirname(to), { recursive: true })
  fs.copyFileSync(from, to)
  copied++
}
console.log(`synced: copied=${copied} unchanged=${unchanged} total=${files.length}`)

// Report — never act on — anything installed that the source no longer has.
const extra = walk(installed).filter(rel => !files.includes(rel))
if (extra.length > 0) {
  console.log(`\nstale in install (${extra.length}) — review and delete manually if obsolete:`)
  for (const rel of extra) console.log(`  ${rel}`)
}

// The one file whose absence makes the plugin look deleted. Loud, always.
for (const must of ['package.json', 'index.js', 'client.js']) {
  if (!fs.existsSync(path.join(installed, must))) {
    console.error(`\nFATAL ${must} is absent from the install — the plugin cannot register`)
    process.exit(1)
  }
}
console.log('\nmanifest present: package.json OK')