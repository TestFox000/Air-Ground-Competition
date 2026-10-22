/**
 * The metrics probe: one SSH round trip that answers "how is the drone doing?".
 *
 * The status float needs three numbers — temperature, CPU load and uptime —
 * and a browser cannot read any of them directly: it has no path to the drone,
 * only to this plugin's own HTTP surface. So the Host half runs one remote
 * command per request and parses a machine-readable answer, exactly like the
 * file layer does (see `src/files.js` for why that shape won).
 *
 * One round trip, deliberately:
 *
 *   - CPU usage is a *delta*, not a level: `/proc/stat` is read twice on the
 *     drone with a `sleep 1` between, and the host computes
 *     `1 - Δidle/Δtotal`. A single sample can only say "since boot", which is
 *     not what a live gauge means.
 *   - the temperature is read per thermal zone, one `type`/`temp` pair each.
 *     Jetson reports millidegrees and some zones answer `EBUSY` for a moment,
 *     so each read is retried once and an unreadable zone is dropped rather
 *     than failing the whole reading.
 *   - uptime and load average come from `/proc/uptime` and `/proc/loadavg`,
 *     which never fail on Linux and need no privileges. `/proc/uptime`'s
 *     first field *is* the drone's own answer to "how long have I been up"
 *     (cross-checked on this Jetson against PID 1's etime: 666.97s vs 11:06),
 *     so the gauge points at a kernel parameter rather than at anything this
 *     plugin invents. `btime` from `/proc/stat` rides along as the drone's
 *     own boot instant (epoch seconds on the drone's clock), so "开机于"
 *     shows the drone's claim instead of a PC-clock back-calculation; the
 *     derivation remains the fallback when an older answer lacks it.
 *   - the battery lives in the PX4 flight controller, surfaced as
 *     `/fmu/out/battery_status` (uXRCE-DDS). It needs both the FC's serial
 *     device *and* the uXRCE-DDS agent, so the reading distinguishes the two
 *     ways of having no number: `battery` is absent for "no flight controller"
 *     and `batteryNote` is set for "the board is there, the topic is not" —
 *     never a fabricated 0%, and never 无飞控 about a plugged-in board.
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
 * The remote command, as one string handed to `ssh` (no local shell involved).
 *
 * Written for the drone's own `/bin/sh` (dash): single quotes keep `$1`/`$2`
 * away from the remote shell, and every output line is `key=value` so the
 * parser splits on the first `=` only.
 */
export const METRICS_COMMAND = [
  'echo DZH_BEGIN',
  // Two CPU samples, one second apart: usage is the delta between them.
  "awk '/^cpu /{print $2,$3,$4,$5,$6,$7,$8,$9,$10; exit}' /proc/stat",
  'sleep 1',
  "awk '/^cpu /{print $2,$3,$4,$5,$6,$7,$8,$9,$10; exit}' /proc/stat",
  // One line per thermal zone: `temp_<type>=<millidegrees>`. A zone that
  // answers EBUSY (seen on this Jetson) is retried once, then skipped —
  // six of nine beats none of nine.
  'for z in /sys/class/thermal/thermal_zone*; do n=$(cat "$z/type" 2>/dev/null) || continue; t=$(cat "$z/temp" 2>/dev/null) || t=$(cat "$z/temp" 2>/dev/null) || continue; printf "temp_%s=%s\\n" "$n" "$t"; done',
  "awk '{printf \"uptimeSec=%s\\n\", $1}' /proc/uptime",
  // The drone's own boot instant (epoch seconds): one number that makes
  // "开机于" the drone's statement, not the host's subtraction.
  "awk '/^btime/{printf \"bootEpoch=%s\\n\", $2}' /proc/stat",
  "awk '{printf \"load1=%s load5=%s load15=%s\\n\", $1, $2, $3}' /proc/loadavg",
  // The drone's battery lives in the PX4 flight controller, surfaced over
  // uXRCE-DDS as /fmu/out/battery_status — which needs *two* things, not one:
  // the FC board's USB-CDC serial device, and the uXRCE-DDS agent bridging
  // /fmu/* into ROS 2 on the drone. Gate on the device first so a board with no
  // flight controller pays no DDS discovery wait; then say *which* of the two
  // is missing, because "the serial device is here but the topic is silent" is
  // a different fact from "there is no flight controller" and the card must not
  // label a plugged-in board 无飞控. When both halves answer, one bounded echo
  // yields `remaining` (0..1) and `voltage_v`; otherwise the reading carries a
  // note (`dds_silent` / `no_ros`) and never a fabricated percentage.
  // setup.bash is a bash script, so it is sourced under bash, not remote dash.
  "if ls /dev/ttyACM* /dev/ttyUSB* >/dev/null 2>&1; then printf 'batteryFc=present\\n'; if [ -f /opt/ros/humble/setup.bash ]; then o=$(bash -c '. /opt/ros/humble/setup.bash 2>/dev/null; timeout 3 ros2 topic echo /fmu/out/battery_status --once 2>/dev/null' | awk '/^[[:space:]]*remaining:/{r=$2+0} /^[[:space:]]*voltage_v:/{v=$2+0} END{if(r>=0 && r<=1 && v>0) printf \"batteryPercent=%s batteryVoltage=%s\\n\", r*100, v}'); if [ -n \"$o\" ]; then printf '%s\\n' \"$o\"; else printf 'batteryNote=dds_silent\\n'; fi; else printf 'batteryNote=no_ros\\n'; fi; else printf 'batteryFc=absent\\n'; fi",
  'echo DZH_END',
].join('\n')

/** The one in-flight reading, shared by simultaneous pollers. */
let inFlight = null

/**
 * Collect the drone's current temperature, CPU usage and uptime.
 *
 * @param settings - the plugin settings (host/user/port/alias/options).
 * @returns a result that always carries `ok`; a refusal carries the same
 *   `state`/`detail` vocabulary as the reachability probe.
 */
export function collectMetrics(settings, { logger, timeoutMs = 9000 } = {}) {
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
    logger?.warn?.(`dzdrone-ssh: metrics failed (${refusal.state}: ${refusal.detail})`)
    return { ok: false, ...refusal, at }
  }

  const parsed = parseMetrics(result.stdout)
  if (parsed === undefined) {
    logger?.warn?.(`dzdrone-ssh: metrics output was not parsable (${result.stdout.slice(0, 120)})`)
    return { ok: false, state: 'error', detail: '无人机返回的数据无法解析', latencyMs: ms, target, at }
  }
  logger?.info?.(`dzdrone-ssh: metrics ${target} -> cpu ${parsed.cpuPercent}% temp ${parsed.temp}°C (${ms}ms)`)
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
  const temps = []
  let uptimeSec
  let bootEpoch
  let load
  let batteryPercent
  let batteryVoltage
  let batteryFc
  let batteryNote

  for (const raw of text.slice(start + MARK_BEGIN.length, end).split('\n')) {
    const line = raw.trim()
    if (line === '') continue

    if (line.startsWith('temp_')) {
      const equals = line.indexOf('=', 5)
      if (equals <= 5) continue
      const id = line.slice(5, equals)
      const rawValue = line.slice(equals + 1)
      // `cat` on a busy zone can answer empty: Number('') is 0, which would
      // paint a fake 0°C. Require a real integer before believing it.
      if (!/^-?\d+$/.test(rawValue)) continue
      temps.push({ id, c: Math.round(Number(rawValue) / 100) / 10 })
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
    if (line.startsWith('batteryPercent=')) {
      // The FC's own `remaining` (0..1) scaled to percent. Both halves must be
      // present and sane before we claim a battery: a lone percentage with no
      // voltage, or a 0 V pack, is not a battery — it is the absent FC board
      // masquerading as one.
      const percent = Number(line.slice('batteryPercent='.length))
      if (Number.isFinite(percent) && percent >= 0 && percent <= 100) batteryPercent = percent
      continue
    }
    if (line.startsWith('batteryVoltage=')) {
      const voltage = Number(line.slice('batteryVoltage='.length))
      if (Number.isFinite(voltage) && voltage > 0 && voltage < 100) batteryVoltage = voltage
      continue
    }
    // Why there is no number, when there is none. `present` means the FC's
    // serial device answered `ls`; the note then says what went wrong after
    // that — the DDS echo produced nothing, or ROS 2 is not installed where we
    // looked. Both are "the FC is plugged in but unreadable", which is not the
    // same claim as 无飞控 and must not be rendered as one.
    if (line === 'batteryFc=present' || line === 'batteryFc=absent') {
      batteryFc = line.slice('batteryFc='.length)
      continue
    }
    if (line.startsWith('batteryNote=')) {
      const note = line.slice('batteryNote='.length)
      if (note === 'dds_silent' || note === 'no_ros') batteryNote = note
      continue
    }
    if (/^[\d\s]+$/.test(line)) samples.push(line.split(/\s+/).filter(value => value !== '').map(Number))
  }

  if (samples.length < 2 || !Number.isFinite(uptimeSec)) return undefined
  const cpuPercent = cpuUsage(samples[0], samples[1])
  if (cpuPercent === undefined) return undefined

  temps.sort((left, right) => right.c - left.c)
  return {
    cpuPercent,
    uptimeSec,
    // undefined when the drone's answer didn't carry it: the float falls back
    // to host-clock arithmetic rather than printing a fabricated boot time.
    ...(bootEpoch === undefined ? {} : { bootEpoch }),
    load,
    temps,
    // The headline number: the hottest zone on the board, with its name so the
    // float can say *which* sensor is at that temperature.
    temp: temps.length > 0 ? temps[0].c : null,
    tempAt: temps.length > 0 ? temps[0].id : null,
    zones: temps.length,
    // The drone's battery, when the FC board answered: `undefined` (not a
    // fabricated 0%) while the flight controller is unplugged or the DDS echo
    // timed out. Both numbers ride together or not at all. A plugged-in board
    // whose topic stayed silent is *not* silence: it rides as `batteryNote`, so
    // the card can say 飞控无数据 instead of the false 无飞控.
    ...(Number.isFinite(batteryPercent) && Number.isFinite(batteryVoltage)
      ? { battery: { percent: Math.round(batteryPercent * 10) / 10, voltage: Math.round(batteryVoltage * 100) / 100 } }
      : batteryFc === 'present'
        ? { batteryNote: batteryNote ?? 'dds_silent' }
        : {}),
  }
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
