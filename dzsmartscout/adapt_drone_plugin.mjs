// Adapt the copied car plugin into the drone plugin.
// Ordered literal/regex replacements across all text files (novnc/ untouched),
// then prints any leftover car-side identifiers for manual review.
import fs from 'node:fs'
import path from 'node:path'

const ROOT = 'D:/User/Desktop/比赛存档/v1/tools/plugins/dsh-dzdrone-ssh'
const SKIP_DIRS = new Set(['novnc', 'node_modules', '__pycache__'])
const EXTS = new Set(['.js', '.mjs', '.html', '.yml', '.yaml', '.md', '.json', '.sh', '.txt'])

/** Applied in order; earlier rules feed later ones. */
const RULES = [
  // 1-3: package / plugin / data-dir identity
  ['dsh-dzrobot-ssh', 'dsh-dzdrone-ssh'],
  ['dzrobot-ssh', 'dzdrone-ssh'],
  ['dzrobot', 'dzdrone'],
  // 4: remote login marker
  ['DZROBOT', 'DZDRONE'],
  // 5: test env flag
  ['DZR_TEST_CAR_VNC', 'DZD_TEST_DRONE_VNC'],
  // 6: user-facing Chinese
  ['小车', '无人机'],
  // 7: camelCase identifiers (before word-boundary rules)
  ['describeCar', 'describeDrone'],
  ['withCar', 'withDrone'],
  ['CarGlyph', 'DroneGlyph'],
  ['makeCarPanel', 'makeDronePanel'],
  ['CarPanel', 'DronePanel'],
  ['carUser', 'droneUser'],
  ['carHost', 'droneHost'],
  ['carKernel', 'droneKernel'],
  ['carUp', 'droneUp'],
  // 8/9: English words (identifier keys, comments, copy)
  [/\bCar\b/g, 'Drone'],
  [/\bcar\b/g, 'drone'],
  // 10: css prefix + tab kinds (avoid colliding with the car plugin's styles)
  ['dzr_', 'dzd_'],
  ['dzr-', 'dzd-'],
  // 11: panel id
  ['dzdrone-car', 'dzdrone-air'],
]

const files = []
;(function walk(dir) {
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    if (entry.isDirectory()) {
      if (!SKIP_DIRS.has(entry.name)) walk(path.join(dir, entry.name))
    } else if (EXTS.has(path.extname(entry.name).toLowerCase())) {
      files.push(path.join(dir, entry.name))
    }
  }
})(ROOT)

let changed = 0
for (const file of files) {
  const before = fs.readFileSync(file, 'utf8')
  let after = before
  for (const [from, to] of RULES) {
    after = typeof from === 'string' ? after.split(from).join(to) : after.replace(from, to)
  }
  if (after !== before) {
    fs.writeFileSync(file, after, 'utf8')
    changed += 1
    console.log(`adapted: ${path.relative(ROOT, file)}`)
  }
}
console.log(`\n${changed}/${files.length} files changed`)

// Leftovers: anything car-side that the rules did not cover.
const leftover = /(dzrobot|DZROBOT|dzr_|dzr-|\bcar\b|\bCar\b|小车)/
for (const file of files) {
  const text = fs.readFileSync(file, 'utf8')
  text.split('\n').forEach((line, i) => {
    if (leftover.test(line)) console.log(`LEFTOVER ${path.relative(ROOT, file)}:${i + 1}: ${line.trim().slice(0, 140)}`)
  })
}
