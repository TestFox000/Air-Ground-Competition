/**
 * The metrics probe: one SSH round trip that answers "how is the car doing?".
 *
 * The status float needs three numbers — battery level, CPU load and uptime —
 * and a browser cannot read any of them directly: it has no path to the car,
 * only to this plugin's own HTTP surface. So the Host half runs one remote
 * command per request and parses a machine-readable answer, exactly like the
 * file layer does (see `src/files.js` for why that shape won).
 *
 * One round trip, deliberately:
 *
 *   - CPU usage is a *delta*, not a level: `/proc/stat` is read twice on the
 *     car with a `sleep 1` between, and the host computes `1 - Δidle/Δtotal`.
 *     A single sample can only say "since boot", which is not what a live
 *     gauge means.
 *   - uptime and load average come from `/proc/uptime` and `/proc/loadavg`,
 *     which never fail on Linux and need no privileges. `/proc/uptime`'s
 *     first field *is* the car's own answer to "how long have I been up",
 *     so the gauge points at a kernel parameter rather than at anything this
 *     plugin invents. `btime` from `/proc/stat` rides along as the car's own
 *     boot instant (epoch seconds on the car's clock), so "开机于" shows the
 *     car's claim instead of a PC-clock back-calculation.
 *   - the battery is the one number that lives *behind ROS*: only the chassis
 *     node (`dzactuator`) reads the VCU over serial, filters the voltage and
 *     publishes `Battery_Percentage` (0..1) plus `PowerVoltage` (volts, with
 *     the pack's own calibration 12 V = full, 10 V = empty). Nothing on the
 *     car exposes it otherwise — no sysfs file, no service. So the probe
 *     subscribes to both topics with `ros2 topic echo --once`, in parallel and
 *     under `timeout`, inside a `bash -c` that sources `/opt/ros/humble` first
 *     (sourcing that file in `sh`/dash aborts the shell — measured, not
 *     assumed). When the stack is not running the topics do not exist, the
 *     echoes come back empty, and the reading carries
 *     `batteryNote=no_stack` instead of a fabricated 0 %: an honest "the car
 *     is not publishing" beats a plausible-looking lie.
 *
 * The markers (`DZH_BEGIN` / `DZH_END`) bound the payload: anything the remote
 * shell puts on stdout besides our own output — MOTD noise, a `~/.bashrc`
 * echo — is outside them and therefore ignored instead of being parsed.
 *
 * Concurrent callers share one in-flight round trip: the float window and its
 * standalone popup can poll at the same moment, and one ssh connection is a
 * better answer for both than two.
 *
 * @module src/metrics.js
 */

import { refusalFor, run, splitOptions } from './ssh.js'

/** Bounds the payload so remote shell noise outside them is never parsed. */
const MARK_BEGIN = 'DZH_BEGIN'
const MARK_END = 'DZH_END'

/**
 * The battery sub-probe, run by the remote shell inside `bash -c '…'`.
 *
 * Single quotes in the surrounding command make this program literal for the
 * remote shell, so it must contain none of its own — which is why every string
 * here is double-quoted. Both topic reads run in parallel behind `timeout 6`:
 * with the stack up they answer in ~1 s (discovery is the slow part), with the
 * stack down `ros2 topic echo` fails fast on an unknown topic, and the worst
 * case (a topic that exists but never publishes) is bounded at 6 s.
 *
 * The `sed` accepts both shapes `--field data` can print: `0.86` bare, or the
 * `data: 0.86` line of the full message. Only a line of digits and dots is
 * believed; anything else is dropped rather than passed on.
 */
const BATTERY_PROBE = [
  '. /opt/ros/humble/setup.bash >/dev/null 2>&1 || true',
  'BP="$1"; BV="$2"',
  '( timeout 6 ros2 topic echo --once --field data /Battery_Percentage 2>/dev/null | sed -n -e "s/^data: *//" -e "/^[0-9.][0-9.]*$/p" | head -n1 > "$BP" ) &',
  '( timeout 6 ros2 topic echo --once --field data /PowerVoltage 2>/dev/null | sed -n -e "s/^data: *//" -e "/^[0-9.][0-9.]*$/p" | head -n1 > "$BV" ) &',
  'wait',
].join('\n')

/**
 * The remote command, as one string handed to `ssh` (no local shell involved).
 *
 * Written for the car's own login shell: single quotes keep `$1`/`$2` and the
 * awk programs away from the local shell, and every output line is `key=value`
 * so the parser splits on the first `=` only.
 */
export const METRICS_COMMAND = [
  'echo DZH_BEGIN',
  // Two CPU samples, one second apart: usage is the delta between them.
  "awk '/^cpu /{print $2,$3,$4,$5,$6,$7,$8,$9,$10; exit}' /proc/stat",
  'sleep 1',
  "awk '/^cpu /{print $2,$3,$4,$5,$6,$7,$8,$9,$10; exit}' /proc/stat",
  "awk '{printf \"uptimeSec=%s\\n\", $1}' /proc/uptime",
  // The car's own boot instant (epoch seconds): one number that makes
  // "开机于" the car's statement, not the host's subtraction.
  "awk '/^btime/{printf \"bootEpoch=%s\\n\", $2}' /proc/stat",
  "awk '{printf \"load1=%s load5=%s load15=%s\\n\", $1, $2, $3}' /proc/loadavg",
  // The battery block: temp files with this shell's pid so parallel probes
  // (or two concurrent float pollers) can never write into each other.
  'BP=/tmp/dzb_$$_pct',
  'BV=/tmp/dzb_$$_volt',
  `bash -c '\n${BATTERY_PROBE}\n' _ "$BP" "$BV"`,
  'p=$(cat "$BP" 2>/dev/null)',
  'v=$(cat "$BV" 2>/dev/null)',
  'rm -f "$BP" "$BV"',
  // Only digits and dots survive to the wire; an empty or garbled read emits
  // nothing at all (the parser then reports "no stack", never a fake 0 %).
  'case $p in *[!0-9.]*|"") ;; *) echo "batteryPct=$p";; esac',
  'case $v in *[!0-9.]*|"") ;; *) echo "batteryVolt=$v";; esac',
  'if [ -z "$p" ] && [ -z "$v" ]; then echo "batteryNote=no_stack"; fi',
  'echo DZH_END',
].join('\n')

/** The one in-flight reading, shared by simultaneous pollers. */
let inFlight = null

/**
 * Collect the car's current battery, CPU usage and uptime.
 *
 * @param settings - the plugin settings (host/user/port/alias/options).
 * @returns a result that always carries `ok`; a refusal carries the same
 *   `state`/`detail` vocabulary as the reachability probe. `timeoutMs` is 12 s
 *   (not the drone's 9): the ROS CLI adds ~1.5 s of Python startup on top of
 *   DDS discovery, and the battery echoes may legitimately use their full 6 s.
 */
export function collectMetrics(settings, { logger, timeoutMs = 12000 } = {}) {
  if (inFlight !== null) return inFlight
  inFlight = read(settings, { logger, timeoutMs }).finally(() => { inFlight = null })
  return inFlight
}

async function read(settings, { logger, timeoutMs } = {}) {
  const started = Date.now()
  const target = `${settings.user}@${settings.host}`
  const alias = String(settings.sshAlias ?? '').trim()
  const argv = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', '-o', 'StrictHostKeyChecking=accept-new']
  if (Number.isInteger(settings.port) && settings.port !== 22) argv.push('-p', String(settings.port))
  for (const option of splitOptions(settings.sshOptions)) argv.push(option)
  argv.push(alias !== '' ? alias : target)
  argv.push(METRICS_COMMAND)

  const result = await run(argv, { timeoutMs, maxStdoutBytes: 64 * 1024 })
  const ms = Date.now() - started
  const at = Date.now()

  if (result.code !== 0) {
    const refusal = refusalFor(result, { ms, target, logger })
    logger?.warn?.(`dzairground-ssh: metrics failed (${refusal.state}: ${refusal.detail})`)
    return { ok: false, ...refusal, at }
  }

  const parsed = parseMetrics(result.stdout)
  if (parsed === undefined) {
    logger?.warn?.(`dzairground-ssh: metrics output was not parsable (${result.stdout.slice(0, 120)})`)
    return { ok: false, state: 'error', detail: '小车返回的数据无法解析', latencyMs: ms, target, at }
  }
  logger?.info?.(`dzairground-ssh: metrics ${target} -> cpu ${parsed.cpuPercent}% battery ${parsed.battery ?? 'n/a'}% (${ms}ms)`)
  return { ok: true, ...parsed, latencyMs: ms, at }
}

/**
 * Parse the bounded payload into the gauge's numbers.
 *
 * @returns the reading, or `undefined` when the payload is not the shape this
 *   parser expects — an unparseable answer is a failed reading, never a
 *   plausible-looking zero.
 */
export function parseMetrics(stdout) {
  const text = String(stdout ?? '')
  const start = text.indexOf(MARK_BEGIN)
  const end = text.indexOf(MARK_END)
  if (start < 0 || end < start) return undefined

  const samples = []
  let uptimeSec
  let bootEpoch
  let load
  let battery = null
  let batteryVolt = null
  let batteryNote = null
  // bug#3: a `batteryPct=` line that normalizeBattery rejects below is a data
  // problem of its own (数据异常). Without this flag it fell through to
  // "no stack"/derived and the UI told the user the ROS stack was down when
  // the chassis had actually answered with an out-of-range number.
  let sawBatteryPct = false

  for (const raw of text.slice(start + MARK_BEGIN.length, end).split('\n')) {
    const line = raw.trim()
    if (line === '') continue

    if (line.startsWith('batteryPct=')) {
      sawBatteryPct = true
      battery = normalizeBattery(line.slice('batteryPct='.length))
      continue
    }
    if (line.startsWith('batteryVolt=')) {
      const value = Number(line.slice('batteryVolt='.length))
      // A plausible pack voltage only: the VCU's own calibration is 10..12 V,
      // so anything outside 5..60 V is a parse accident, not a battery.
      if (Number.isFinite(value) && value >= 5 && value <= 60) batteryVolt = Math.round(value * 100) / 100
      continue
    }
    if (line.startsWith('batteryNote=')) {
      batteryNote = line.slice('batteryNote='.length) || null
      continue
    }
    if (line.startsWith('uptimeSec=')) {
      const value = Number(line.slice('uptimeSec='.length))
      if (Number.isFinite(value) && value >= 0) uptimeSec = value
      continue
    }
    if (line.startsWith('bootEpoch=')) {
      // Optional: absent on an older answer is fine, absurd is not — a boot
      // instant in the future or before 2000 would paint a nonsense "since".
      const value = Number(line.slice('bootEpoch='.length))
      if (Number.isFinite(value) && value > 946684800 && value < 4102444800) bootEpoch = value
      continue
    }
    if (line.startsWith('load1=')) {
      const match = /^load1=([\d.]+)\s+load5=([\d.]+)\s+load15=([\d.]+)$/.exec(line)
      if (match !== null) load = [Number(match[1]), Number(match[2]), Number(match[3])]
      continue
    }
    if (/^[\d\s]+$/.test(line)) samples.push(line.split(/\s+/).filter(value => value !== '').map(Number))
  }

  if (samples.length < 2 || !Number.isFinite(uptimeSec)) return undefined
  const cpuPercent = cpuUsage(samples[0], samples[1])
  if (cpuPercent === undefined) return undefined

  // bug#3 first: a percentage that *arrived* but failed normalizeBattery is
  // 数据异常 (bad_value) — the chassis spoke, so "no stack" would be a lie
  // about the link and "derived" would paper over a chassis bug.
  if (sawBatteryPct && battery === null) {
    batteryNote = 'bad_value'
  }
  // The chassis node was not publishing (stack down) but the voltage topic
  // answered: the pack's own calibration turns volts into a level —
  // 12 V = 100 %, 10 V = 0 %, clamped (see the history log, §5.3).
  // Only reachable with no rejected percentage: that case keeps bad_value.
  else if (battery === null && batteryVolt !== null) {
    battery = Math.max(0, Math.min(100, Math.round(((batteryVolt - 10) / 2) * 1000) / 10))
    batteryNote = 'derived'
  }

  return {
    cpuPercent,
    uptimeSec,
    // undefined when the car's answer didn't carry it: the float falls back
    // to host-clock arithmetic rather than printing a fabricated boot time.
    ...(bootEpoch === undefined ? {} : { bootEpoch }),
    load,
    // The headline number: the battery level in percent, or null with
    // `batteryNote` saying why (no_stack) — never a made-up zero.
    battery,
    batteryVolt,
    batteryNote,
  }
}

/**
 * The chassis publishes `Battery_Percentage` as 0..1 (its own docs: 0 = empty,
 * 1 = full). Accept an already-percent answer as well — a topic that one day
 * switches to 0..100 must not read as "1 % for ever" — and reject anything
 * outside both scales instead of clamping a parse accident into the gauge.
 *
 * @returns percent rounded to 0.1, or null when the value is not a battery.
 */
export function normalizeBattery(raw) {
  const value = Number(String(raw).trim())
  if (!Number.isFinite(value) || value < 0) return null
  const percent = value <= 1.0001 ? value * 100 : value
  if (percent > 100.0001) return null
  return Math.round(percent * 10) / 10
}

/**
 * Usage between two `/proc/stat` `cpu` samples, as a percentage.
 *
 * Fields are user nice system idle iowait irq softirq steal; `idle + iowait`
 * is the idle half. A clock that did not advance (two identical samples) is
 * 0%, not a division by zero.
 */
export function cpuUsage(before, after) {
  const sum = row => row.reduce((total, value) => total + (Number.isFinite(value) ? value : 0), 0)
  const idle = row => (Number.isFinite(row[3]) ? row[3] : 0) + (Number.isFinite(row[4]) ? row[4] : 0)
  if (!Array.isArray(before) || !Array.isArray(after)) return undefined
  const total = sum(after) - sum(before)
  const idleDelta = idle(after) - idle(before)
  if (!(total > 0)) return 0
  const percent = (1 - idleDelta / total) * 100
  return Math.max(0, Math.min(100, Math.round(percent * 10) / 10))
}
