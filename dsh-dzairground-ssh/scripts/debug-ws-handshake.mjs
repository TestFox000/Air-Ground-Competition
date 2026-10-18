/**
 * Print the raw response to a browser-shaped WebSocket upgrade.
 *
 * `test-live-ws.mjs` uses Node's client, which omits the headers Chrome always
 * sends (`Origin`, `Sec-WebSocket-Extensions`, `Accept-Encoding`, a real
 * `User-Agent`). A 101 that Node accepts can still be rejected by a browser,
 * and a browser rejection surfaces only as `error` + close code 1006 with an
 * empty reason — the least diagnosable failure there is. This prints what the
 * server actually wrote, byte for byte, plus a verdict on the parts a browser
 * enforces.
 *
 * Usage: node scripts/debug-ws-handshake.mjs [port]
 */

import net from 'node:net'
import crypto from 'node:crypto'

const port = Number(process.argv[2] ?? 19387)
const host = '127.0.0.1'

/** Exactly what Chrome puts on the wire for `new WebSocket(..., ['binary'])`. */
const browserRequest = key => [
  'GET /api/dzairground-ssh/vnc HTTP/1.1',
  `Host: ${host}:${port}`,
  'Connection: Upgrade',
  'Pragma: no-cache',
  'Cache-Control: no-cache',
  'User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/141.0.0.0 Safari/537.36',
  'Upgrade: websocket',
  'Origin: http://127.0.0.1:19387',
  'Sec-WebSocket-Version: 13',
  'Accept-Encoding: gzip, deflate, br, zstd',
  'Accept-Language: zh-CN,zh;q=0.9',
  `Sec-WebSocket-Key: ${key}`,
  'Sec-WebSocket-Extensions: permessage-deflate; client_max_window_bits',
  'Sec-WebSocket-Protocol: binary',
].join('\r\n') + '\r\n\r\n'

const expectedAccept = key => crypto.createHash('sha1')
  .update(key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest('base64')

const request = (label, raw, key) => new Promise(resolve => {
  const socket = net.connect({ host, port })
  let response = ''
  const done = () => {
    socket.destroy()
    console.log(`\n=== ${label} ===`)
    if (response === '') {
      console.log('  (no response at all — the socket closed silently)')
    } else {
      for (const line of response.split('\r\n')) console.log(`  ${line}`)
    }
    if (response.startsWith('HTTP/1.1 101')) {
      const want = expectedAccept(key)
      const got = /sec-websocket-accept:\s*(\S+)/i.exec(response)?.[1]
      const proto = /sec-websocket-protocol:\s*(\S+)/i.exec(response)?.[1]
      const ext = /sec-websocket-extensions:\s*([^\r\n]+)/i.exec(response)?.[1]
      console.log(`  -- verdict --`)
      console.log(`  accept key matches : ${got === want}${got === want ? '' : ` (want ${want}, got ${got})`}`)
      console.log(`  sub-protocol       : ${proto ?? '(none)'} ${proto === undefined ? '(legal: client may proceed with none)' : proto === 'binary' ? '(legal: was offered)' : '(ILLEGAL: not offered)'}`)
      console.log(`  extensions         : ${ext ?? '(none)'} ${ext === undefined ? '(legal: server declined permessage-deflate)' : '(CHECK: only legal if it is an offered extension)'}`)
    } else if (response !== '') {
      console.log(`  -- verdict: not a 101, so the browser reports 1006 --`)
    }
    resolve()
  }
  socket.setTimeout(6000)
  socket.on('connect', () => socket.write(raw))
  socket.on('data', chunk => { response += chunk.toString('latin1') })
  socket.on('timeout', done)
  socket.on('close', done)
  socket.on('error', error => { response += `\n[socket error] ${error.message}`; done() })
})

const key = crypto.randomBytes(16).toString('base64')
console.log(`probing ws://${host}:${port}/api/dzairground-ssh/vnc`)
await request('full browser request', browserRequest(key), key)

// Same request minus the extension offer: separates "the extension header
// confuses the server" from "something else does".
const noExtKey = crypto.randomBytes(16).toString('base64')
await request('browser request, no permessage-deflate', browserRequest(noExtKey)
  .replace('Sec-WebSocket-Extensions: permessage-deflate; client_max_window_bits\r\n', ''), noExtKey)
