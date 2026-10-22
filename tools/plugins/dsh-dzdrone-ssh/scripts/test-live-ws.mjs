/**
 * Diagnose the live VNC WebSocket endpoint: open it the way noVNC does, report
 * the open/close events with code and reason, then send the RFB version line and
 * wait to see whether any bytes come back before the socket dies.
 * Run: node scripts/test-live-ws.mjs
 */

const url = process.argv[2] ?? 'ws://127.0.0.1:19387/api/dzdrone-ssh/vnc'
const started = Date.now()
const at = () => `+${Date.now() - started}ms`

console.log('connecting', url)
const ws = new WebSocket(url, 'binary')
ws.binaryType = 'arraybuffer'

let sawBytes = false

ws.addEventListener('open', () => {
  console.log(at(), 'OPEN — protocol =', JSON.stringify(ws.protocol))
  ws.send(new TextEncoder().encode('RFB 003.008\n'))
})

ws.addEventListener('message', event => {
  sawBytes = true
  const bytes = Buffer.from(event.data)
  console.log(at(), `MESSAGE ${bytes.length}B:`, JSON.stringify(bytes.toString('latin1').slice(0, 40)))
  ws.close()
})

ws.addEventListener('error', event => {
  console.log(at(), 'ERROR:', event?.message ?? event?.error?.message ?? JSON.stringify(event))
})

ws.addEventListener('close', event => {
  console.log(at(), `CLOSE code=${event.code} clean=${event.wasClean} reason=${JSON.stringify(event.reason)}`)
  console.log(sawBytes ? 'RESULT: bytes arrived — bridge works' : 'RESULT: no bytes arrived before close')
  process.exit(0)
})

setTimeout(() => {
  console.log(at(), 'TIMEOUT — socket still open after 12s?')
  try { ws.close() } catch { /* gone */ }
  process.exit(sawBytes ? 0 : 1)
}, 12000)
