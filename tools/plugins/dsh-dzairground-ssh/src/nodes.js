/**
 * The unified node model — one registry for every machine this plugin can
 * reach, per `docs/merge/coordination-design.md` §1.2 (NodeSnapshot).
 *
 * Before S2 the whole Host half was written against one implicit node (the
 * car): one settings file, one status file, one metrics probe. S2 turns that
 * implicit node into a registry entry so `/car/*` and `/drone/*` run the *same*
 * handler chain against *different* per-node state, and so the caps a UI needs
 * (which gauges to draw, whether a sim bench exists) are declared data instead
 * of role checks scattered through the client.
 *
 * What this module deliberately does not own: the per-node JsonStores and the
 * metrics collectors — those are wired in `index.js`, which knows the data
 * directory and the logger. This file is the pure model: roles, defaults,
 * caps, and the snapshot shape both nodes answer with.
 *
 * @module src/nodes.js
 */

import { SETTINGS_INITIAL } from './store.js'

/**
 * The drone node's settings defaults.
 *
 * Source: the drone plugin's own persisted Host config
 * (`~/.dsh/dzdrone-ssh/settings.json`, read-only at S2 time), which carries
 * host=`dzsmart.local`, user=`duzhong`, vncPort=`5901`,
 * filesRoot=`/home/duzhong`. The values are baked in rather than read from
 * that file at runtime because the old data directory is machine-specific,
 * frozen, and about to be retired — a deployed plugin must not depend on a
 * sibling install's leftovers. New writes land under `~/.dsh/dzairground-ssh/`.
 *
 * `vncPassword` is deliberately `''` (the drone's old config had one): a
 * credential does not belong in shipped defaults — the drift audit flagged
 * exactly this (merge must not bake the drone's password into a shared
 * default). Leaving it empty makes noVNC prompt on connect, the same behaviour
 * a fresh car install gets.
 */
export const DRONE_SETTINGS_INITIAL = {
  version: 1,
  host: 'dzsmart.local',
  port: 22,
  user: 'duzhong',
  /** Empty means "use the user's own ssh config / key", i.e. plain `ssh <target>`. */
  sshAlias: '',
  probeTimeoutMs: 2500,
  sshOptions: '-o ServerAliveInterval=30 -o ServerAliveCountMax=6',
  vncHost: '',
  vncPort: 5901,
  vncMode: 'direct',
  vncPassword: '',
  filesRoot: '/home/duzhong',
  floatOpacity: 100,
  lastConnectedAt: 0,
}

/**
 * The registry. `caps.gauges` is decision ① (capability-driven metrics):
 * the car draws battery/CPU/uptime, the drone temperature/CPU/uptime/FC
 * battery — each node only ever names gauges it can really measure, and a UI
 * that renders from `caps` never has to hard-code the split. `kind` is the
 * coordination layer's ground/air distinction.
 */
export const NODES = {
  car: {
    role: 'car',
    kind: 'ground',
    label: '小车',
    settingsInitial: SETTINGS_INITIAL,
    caps: {
      gauges: ['batteryPercent', 'cpuPercent', 'uptimeSec'],
      hasSim: true,
      hasTerminal: true,
      hasFiles: true,
      hasVnc: true,
    },
  },
  drone: {
    role: 'drone',
    kind: 'air',
    label: '无人机',
    settingsInitial: DRONE_SETTINGS_INITIAL,
    caps: {
      gauges: ['temp', 'cpuPercent', 'uptimeSec', 'batteryPercent'],
      hasSim: false,
      hasTerminal: true,
      hasFiles: true,
      hasVnc: true,
    },
  },
}

/** Every role the router accepts, in registry order. */
export const ROLES = Object.keys(NODES)

/** The registry entry for one role; `undefined` for a role the router rejects. */
export function nodeSpec(role) {
  return Object.prototype.hasOwnProperty.call(NODES, role) ? NODES[role] : undefined
}

/**
 * The SSH target a node is reached at — alias first, else user@host. The same
 * rule `src/ssh.js` applies when dialling, so a snapshot's `target` is what a
 * connection would actually use, not a re-derivation that could drift.
 */
export function nodeTarget(settings) {
  const alias = String(settings?.sshAlias ?? '').trim()
  return alias !== '' ? alias : `${settings?.user ?? ''}@${settings?.host ?? ''}`
}

/**
 * One node's slice of a `/status`-shaped answer — the NodeSnapshot.
 *
 * `probe` keeps its full shape plus two fields bug#1 needs: `at` (when the
 * probe was actually taken) and `source` (`live` for a probe that just ran,
 * `cached` for the persisted snapshot `/status` reads). A UI may only paint
 * green from a live probe younger than its staleness bound; everything else is
 * `stale` + "上次探测 N 前". The snapshot never fabricates a green.
 *
 * `desc` rides through as-is: an absent description stays absent on the wire
 * (JSON.stringify drops an undefined key), which keeps `/status` and
 * `/car/status` key-for-key identical with the pre-merge single-node shape.
 *
 * @returns `{ id, role, kind, label, target, probe, desc, caps }`.
 */
export function nodeSnapshot(role, { settings, probe, desc } = {}) {
  const spec = nodeSpec(role)
  if (spec === undefined) return undefined
  return {
    id: spec.role,
    role: spec.role,
    kind: spec.kind,
    label: spec.label,
    target: nodeTarget(settings),
    probe: probe === undefined
      ? undefined
      : { ...probe, source: 'cached' },
    desc,
    caps: spec.caps,
  }
}
