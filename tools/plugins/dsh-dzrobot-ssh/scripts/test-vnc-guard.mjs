/**
 * The cached-address guard.
 *
 * `dialOrdered` remembers the address that answered, so an mDNS flake degrades to
 * "use what worked" instead of "give up". This car roams between networks
 * (observed: 192.168.43.17 and 10.230.128.188 on one day), so a cached address
 * can name a *different* host once the car moves. The bridge pumps raw desktop
 * bytes, so it must not forward a single one to a peer that fails the RFB banner
 * check — and a failed check must retire the cache entry.
 *
 * The cache is keyed by host and private to `src/vnc.js`, so every case primes
 * it the only way the product can: a real successful dial first, then a dial
 * against a name that cannot resolve (`*.invalid`), which is what sends
 * `dialOrdered` down the cache branch.
 *
 * Run: node scripts/test-vnc-guard.mjs
 */

import net from 'node:net'
import assert from 'node:assert/strict'

const { VncBridge, __primeAddress } = await import('../src/vnc.js')

const failures = []
const check = (label, run) => {
  try {
    run()
    console.log(`OK    ${label}`)
  } catch (error) {
    failures.push(`${label}: ${error?.message ?? error}`)
    console.log(`FAIL  ${label}\n      ${error?.message ?? error}`)
  }
}

/** A server that greets with `line`, then goes quiet. */
const greet = line => new Promise(resolve => {
  const open = new Set()
  const server = net.createServer(socket => {
    // The bridge tears its side down abruptly; an unhandled reset here would
    // crash the test run instead of reporting a verdict.
    socket.on('error', () => {})
    open.add(socket)
    socket.once('close', () => open.delete(socket))
    if (line !== undefined) socket.write(line)
  })
  // Tracked connections are dropped on close: a held socket would keep
  // server.close() pending for the whole run and look like a hang.
  server.teardown = () => { for (const socket of open) socket.destroy() }
  server.listen(0, '127.0.0.1', () => resolve(server))
})

/** A server that accepts and never says anything. */
const silent = () => greet(undefined)

/** Close a server without waiting for its connections to drain. */
const close = server => new Promise(resolve => {
  server.teardown?.()
  server.close(resolve)
})

/**
 * Seed the cache so the next dial must go through the cached-address path.
 *
 * `name` is `.invalid`, so `dns.lookup` fails for it — which is exactly what
 * makes `dialOrdered` fall back to the remembered address. A test cannot prime
 * that by dialling, because the cache is only written by a dial that resolved.
 */
const primeCache = (name, address, port) => {
  __primeAddress(name, address)
  return new VncBridge({ host: name, vncHost: name, vncPort: port })
}

// ── 1. a cached address that still speaks RFB is bridged, banner included ──────
await (async () => {
  const server = await greet('RFB 003.008\n')
  const { port } = server.address()
  const bridge = primeCache('dzrobot-guard-ok.invalid', '127.0.0.1', port)
  // The same order the Host uses: the sink is attached before open() is called,
  // so the banner released by the guard has somewhere to go.
  const received = []
  bridge.onData(chunk => received.push(chunk.toString('latin1')))
  await bridge.open()
  check('cached RFB peer is bridged and its banner replayed', () => {
    assert.deepEqual(received, ['RFB 003.008\n'])
  })
  bridge.close()
  await close(server)
})()

// ── 2. a cached address that is NOT VNC is refused, and nothing is forwarded ──
// This is the security property: the car has moved, and a stranger now owns the
// old address. No desktop bytes may reach the browser, and the entry is retired.
await (async () => {
  const server = await greet('HTTP/1.1 200 OK\r\n\r\n')
  const { port } = server.address()
  const name = 'dzrobot-guard-wrong.invalid'
  const bridge = primeCache(name, '127.0.0.1', port)
  let opened = false
  let reason = ''
  try {
    await bridge.open()
    opened = true
  } catch (error) {
    reason = String(error?.message ?? error)
  }
  const received = []
  bridge.onData(chunk => received.push(chunk.toString()))

  console.log(`     refused with: ${reason}`)
  check('non-VNC peer at a cached address is refused', () => {
    assert.equal(opened, false, 'bridge opened against a non-VNC peer')
  })
  check('no desktop bytes reach the browser', () => {
    assert.deepEqual(received, [], `forwarded ${JSON.stringify(received)}`)
  })
  bridge.close()
  await close(server)
})()

// ── 3. a cached address that accepts but never greets is refused ──────────────
await (async () => {
  const server = await silent()
  const { port } = server.address()
  const bridge = primeCache('dzrobot-guard-silent.invalid', '127.0.0.1', port)
  let opened = false
  try {
    await bridge.open()
    opened = true
  } catch { /* expected */ }
  check('silent peer at a cached address is refused', () => {
    assert.equal(opened, false, 'bridge opened against a peer that never greets')
  })
  bridge.close()
  await close(server)
})()

// ── 4. a failed check retires the cache, so the next dial re-resolves ─────────
// Otherwise one bad verdict keeps poisoning every later attempt.
await (async () => {
  const impostor = await greet('nope\n')
  const impostorPort = impostor.address().port
  const name = 'dzrobot-guard-poison.invalid'

  // First attempt: the impostor holds the cached address and is refused, which
  // must delete the cache entry.
  const first = primeCache(name, '127.0.0.1', impostorPort)
  let refused = false
  try { await first.open() } catch { refused = true }
  check('impostor at a cached address is refused', () => {
    assert.equal(refused, true, 'bridge opened against an impostor')
  })
  first.close()

  // Second attempt with no re-priming: the name still cannot resolve and the
  // cache is now empty, so the dial must fail outright rather than fall back to
  // the impostor again.
  const second = new VncBridge({ host: name, vncHost: name, vncPort: impostorPort })
  let reconnected = false
  try {
    await second.open()
    reconnected = true
    second.close()
  } catch { /* expected */ }
  check('a retired cache is not reused on the next attempt', () => {
    assert.equal(reconnected, false, 'dial fell back to a retired cache entry')
  })
  await close(impostor)
})()

if (failures.length > 0) {
  console.log('\nFAILURES:')
  for (const failure of failures) console.log(' -', failure)
  process.exitCode = 1
} else {
  console.log('\nVNC GUARD TEST PASSED')
}