/**
 * The unified gauge contract (`dzag` capability cells) — the single place a
 * metrics reading becomes the cells a UI draws, per
 * `docs/merge/coordination-design.md` §2.2/§2.4.
 *
 * Why this exists: the two frozen plugins answered "where is the number" in
 * two incompatible shapes — the car keeps `battery: number|null` with
 * `batteryNote` riding alongside, the drone omits `battery` entirely and only
 * then sets `batteryNote`. A UI handed those raw shapes has to guess, and the
 * guesses are the three bugs this merge fixes (fake green, fake 无飞控, fake
 * "stack down"). So every gauge here is an object that always exists for a
 * capability the node declares, carrying its own `quality` and — when absent —
 * the *reason* it is absent. The UI has nothing left to guess.
 *
 * Host-side consumers: tests (W3/W4). The browser half re-implements the same
 * mapping inside `client.js` because the two halves share no module system —
 * the contract tests in `scripts/test-s2.mjs` pin both sides to the same
 * answers so they cannot drift.
 *
 * @module src/gauges.js
 */

import { nodeSpec } from './nodes.js'

/**
 * The unified absence vocabulary — every word is a different on-site action,
 * which is the whole reason there are ten and not three (design §2.4).
 */
export const REASONS = {
  unreachable: { label: '不可达', hint: '查上电 / WiFi' },
  needs_password: { label: '需要密码', hint: '修复免密登录' },
  host_key: { label: '主机密钥不符', hint: '确认后清 known_hosts' },
  no_stack: { label: 'ROS 栈未运行', hint: '跑 start_all.sh' },
  no_ros: { label: '机上无 ROS 2', hint: '装 / 核对 setup.bash 路径' },
  no_fc: { label: '无飞控', hint: '插板 / 查线' },
  dds_silent: { label: '飞控无数据', hint: '查 uXRCE-DDS agent' },
  bad_value: { label: '数据异常', hint: '数值越界被拒，看话题语义是否变了' },
  not_supported: { label: '不支持', hint: '该节点无此项能力' },
  no_sample: { label: '本次未采到', hint: '看 detail' },
}

/** A transport refusal state (from `ssh.js`) mapped into the vocabulary. */
const REFUSAL_REASON = {
  unreachable: 'unreachable',
  'needs-password': 'needs_password',
  'host-key': 'host_key',
  error: 'no_sample',
  ready: 'no_sample',
}

/** One gauge cell: present-and-good, derived, or absent-with-a-reason. */
function gauge(key, value, { unit, quality = 'measured', reason = null, source = null, extra } = {}) {
  return {
    key,
    v: value,
    unit,
    quality,
    // `reason` is filled whenever quality is 'absent' — an absence without a
    // reason is exactly the guessing this module exists to kill.
    reason: quality === 'absent' ? (reason ?? 'no_sample') : reason,
    ...(source === null ? {} : { source }),
    ...(extra === undefined ? {} : { extra }),
  }
}

/** The car's battery cell: measured / derived / no_stack / bad_value. */
function carBattery(reading) {
  const volt = Number.isFinite(reading.batteryVolt) ? reading.batteryVolt : null
  if (Number.isFinite(reading.battery)) {
    const quality = reading.batteryNote === 'derived' ? 'derived' : 'measured'
    return gauge('batteryPercent', reading.battery, {
      unit: '%',
      quality,
      source: quality === 'derived' ? 'volts×12V满/10V空' : 'ros:/Battery_Percentage',
      extra: volt === null ? undefined : { voltage: volt },
    })
  }
  const reason = reading.batteryNote === 'bad_value' || reading.batteryNote === 'no_stack'
    ? reading.batteryNote
    : 'no_sample'
  return gauge('batteryPercent', null, {
    unit: '%',
    quality: 'absent',
    reason,
    source: 'ros:/Battery_Percentage',
    extra: volt === null ? undefined : { voltage: volt },
  })
}

/** The drone's FC battery cell: measured / no_fc / dds_silent / no_ros / bad_value. */
function droneBattery(reading) {
  const battery = reading.battery
  if (battery !== null && battery !== undefined && Number.isFinite(battery.percent)) {
    return gauge('batteryPercent', battery.percent, {
      unit: '%',
      source: 'px4:/fmu/out/battery_status',
      extra: Number.isFinite(battery.voltage) ? { voltage: battery.voltage } : undefined,
    })
  }
  const note = reading.batteryNote
  const reason = note === 'dds_silent' || note === 'no_ros' || note === 'bad_value' ? note : 'no_fc'
  return gauge('batteryPercent', null, { unit: '%', quality: 'absent', reason, source: 'px4:/fmu/out/battery_status' })
}

/**
 * One node's capability cells from one metrics reading (or refusal).
 *
 * @param role - 'car' | 'drone' — decides which gauges exist at all.
 * @param reading - the `/metrics`-shaped answer (ok:true) or a refusal
 *   (ok:false with `state`/`detail`). `now` only stamps the view.
 * @returns `{ ok, at, reason, detail, gauges }` where `gauges` always has one
 *   entry per `caps.gauges` of the role — offline simply marks every cell
 *   absent with the transport reason, so a UI never special-cases the
 *   offline shape.
 */
export function gaugeView(role, reading, now = Date.now()) {
  const spec = nodeSpec(role)
  const at = Number.isFinite(reading?.at) ? reading.at : now
  if (spec === undefined) return { ok: false, at, reason: 'not_supported', detail: `unknown role ${role}`, gauges: [] }

  const order = spec.caps.gauges
  if (reading?.ok !== true) {
    const reason = REFUSAL_REASON[reading?.state] ?? 'no_sample'
    return {
      ok: false,
      at,
      reason,
      detail: String(reading?.detail ?? reading?.state ?? 'no reading'),
      gauges: order.map(key => gauge(key, null, { quality: 'absent', reason, unit: key === 'temp' ? '°C' : key === 'uptimeSec' ? 's' : '%' })),
    }
  }

  const cells = []
  for (const key of order) {
    if (key === 'batteryPercent') {
      cells.push(role === 'car' ? carBattery(reading) : droneBattery(reading))
      continue
    }
    if (key === 'temp') {
      const temp = Number.isFinite(reading.temp) ? reading.temp : null
      cells.push(temp === null
        ? gauge('temp', null, { unit: '°C', quality: 'absent', reason: 'no_sample', source: 'thermal_zone:*' })
        : gauge('temp', temp, {
          unit: '°C',
          source: `thermal_zone:${reading.tempAt ?? '*'}`,
          extra: { zones: reading.zones, tempAt: reading.tempAt },
        }))
      continue
    }
    if (key === 'cpuPercent') {
      const cpu = Number.isFinite(reading.cpuPercent) ? reading.cpuPercent : null
      cells.push(gauge('cpuPercent', cpu, { unit: '%', source: '/proc/stat', ...(cpu === null ? { quality: 'absent', reason: 'no_sample' } : {}) }))
      continue
    }
    if (key === 'uptimeSec') {
      const up = Number.isFinite(reading.uptimeSec) ? reading.uptimeSec : null
      cells.push(gauge('uptimeSec', up, {
        unit: 's',
        source: '/proc/uptime',
        extra: Number.isFinite(reading.bootEpoch) ? { bootEpoch: reading.bootEpoch } : undefined,
        ...(up === null ? { quality: 'absent', reason: 'no_sample' } : {}),
      }))
      continue
    }
    cells.push(gauge(key, null, { quality: 'absent', reason: 'not_supported' }))
  }
  return { ok: true, at, reason: null, detail: null, gauges: cells }
}
