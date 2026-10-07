/**
 * Copy this package into the profile install that actually runs.
 *
 * The profile declares this package as a `file:` dependency
 * (`"dsh-dzrobot-ssh": "file:D:/..."`), and npm materializes that as a **packed
 * copy** — not a junction, despite what the README used to claim. So the running
 * plugin loads from `~/.dsh/profiles/<profile>/node_modules/<name>`, and editing
 * the working tree changes nothing until the files land there.
 *
 * `POST /reload` does not help either: it re-imports from that same installed
 * directory, so it faithfully reloads the *stale* copy. That combination —
 * believing edits were live while the browser loaded an old packed copy — is what
 * hid a missing `novnc/` and `vnc-app.html` for a whole debugging session.
 *
 * Only the paths `package.json#files` declares are copied, so this stays in step
 * with what a real install would produce, and it never deletes anything.
 *
 * Usage: node scripts/deploy.mjs [--profile desktop] [--dry-run]
 */

import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const argv = process.argv.slice(2)
const dryRun = argv.includes('--dry-run')
const profileIndex = argv.indexOf('--profile')
const profile = profileIndex === -1 ? (process.env.DZR_PROFILE ?? 'desktop') : argv[profileIndex + 1]

const here = path.dirname(fileURLToPath(import.meta.url))
const root = path.join(here, '..')
const pkg = JSON.parse(fs.readFileSync(path.join(root, 'package.json'), 'utf8'))
const declared = Array.isArray(pkg.files) ? pkg.files : []

const dshHome = process.env.DSH_HOME ?? path.join(os.homedir(), '.dsh')
const target = path.join(dshHome, 'profiles', profile, 'node_modules', pkg.name)

if (!fs.existsSync(path.join(target, 'package.json'))) {
  console.error(`no installed copy at ${target}`)
  console.error('install the bundle into this profile first, then re-run this script')
  process.exit(1)
}

// Walk a declared entry, returning every file it ships (entries may be a file or
// a directory).
const filesUnder = entry => {
  const absolute = path.join(root, entry)
  if (!fs.existsSync(absolute)) {
    console.warn(`  warn: "files" declares ${entry}, which does not exist — skipped`)
    return []
  }
  const stat = fs.statSync(absolute)
  if (stat.isFile()) return [entry]
  const found = []
  const walk = relative => {
    for (const child of fs.readdirSync(path.join(root, relative), { withFileTypes: true })) {
      const next = `${relative}/${child.name}`
      if (child.isDirectory()) walk(next)
      else if (child.isFile()) found.push(next)
    }
  }
  walk(entry)
  return found
}

const relFiles = declared.flatMap(filesUnder)
let copied = 0
let identical = 0

for (const relative of relFiles) {
  const from = path.join(root, relative)
  const to = path.join(target, relative)
  const fromBytes = fs.readFileSync(from)
  const same = fs.existsSync(to) && fs.readFileSync(to).equals(fromBytes)
  if (same) {
    identical += 1
    continue
  }
  if (!dryRun) {
    fs.mkdirSync(path.dirname(to), { recursive: true })
    fs.writeFileSync(to, fromBytes)
  }
  copied += 1
  console.log(`  ${dryRun ? 'would copy' : 'copied'} ${relative}`)
}

console.log(`${dryRun ? '[dry run] ' : ''}target  : ${target}`)
console.log(`${dryRun ? '[dry run] ' : ''}declared: ${relFiles.length} file(s) from ${declared.length} entr(ies)`)
console.log(`${dryRun ? '[dry run] ' : ''}updated : ${copied}, already current: ${identical}`)
if (copied > 0 && !dryRun) {
  console.log('now POST /reload for the host half, and refresh the page for the browser half')
}
