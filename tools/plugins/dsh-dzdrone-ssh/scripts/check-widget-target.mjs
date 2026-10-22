// Where does the status float get its numbers?
//
// Replays the exact data path the widget uses — settings.json → collectMetrics()
// → ssh — and prints the readings beside the target identity, so "the float
// shows the drone" is a checked claim rather than a plausible one.
import { readFile } from 'node:fs/promises'
import { homedir } from 'node:os'
import { join } from 'node:path'
import { collectMetrics } from '../src/metrics.js'

// Same location index.js writes to: ~/.dsh/dzdrone-ssh/settings.json.
const settingsPath = join(homedir(), '.dsh', 'dzdrone-ssh', 'settings.json')
const settings = JSON.parse(await readFile(settingsPath, 'utf8'))

console.log('settings target:', `${settings.user}@${settings.host}:${settings.port}`)

const started = Date.now()
const metrics = await collectMetrics(settings, {})
console.log(`collectMetrics ${Date.now() - started}ms:`, JSON.stringify(metrics, null, 2))

if (metrics.ok !== true) {
  console.error('FLOAT TARGET UNREACHABLE')
  process.exit(1)
}
console.log('float would show:',
  `${metrics.temp}°C (${metrics.zones} zones, peak ${metrics.tempAt})`,
  `cpu ${metrics.cpuPercent}%`,
  `up ${Math.floor(metrics.uptimeSec / 3600)}h${Math.floor((metrics.uptimeSec % 3600) / 60)}m`)
