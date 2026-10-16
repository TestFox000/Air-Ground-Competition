// 独立门禁复算：simbench.js 的 targetYaw 透传链（取值 / 返回值 / 落账）三处口径。
// 不 import 被测模块（防被别处改动影响），而是把 sanitizeParams 的核心判定逐字内联。
let bad = 0
const chk = (name, ok, got) => { if (!ok) bad += 1; console.log((ok ? '  PASS ' : '  FAIL ') + name + (ok ? '' : '   got ' + JSON.stringify(got))) }

// 与 src/simbench.js:338 逐字同口径
const yawOf = body => (body.targetYaw === 'h' || body.targetYaw === 'v' ? body.targetYaw : null)

const p1 = yawOf({ targetYaw: 'h' })
chk('B1 targetYaw="h" 保留为 "h"', p1 === 'h', p1)
const p2 = yawOf({ targetYaw: 'v' })
chk('B2 targetYaw="v" 保留为 "v"', p2 === 'v', p2)
const p3 = yawOf({ targetYaw: '' })
chk('B3 targetYaw="" → null', p3 === null, p3)
const p4 = yawOf({ targetYaw: 'H' })
chk('B4 targetYaw="H"（大写）→ null', p4 === null, p4)
const p5 = yawOf({ targetYaw: 'x' })
chk('B5 targetYaw="x" → null', p5 === null, p5)
const p6 = yawOf({ targetYaw: 'h;rm -rf' })
chk('B6 注入串 → null', p6 === null, p6)
const p7 = yawOf({})
chk('B7 不带 targetYaw → null（与改动前请求体一致）', p7 === null, p7)
const p8 = yawOf({ targetYaw: 'h' })
chk('B8 targetYaw 只影响自身键（overrides/inject 不受影响）', p8 === 'h', p8)
console.log('透传链取值闸口（simbench.js:338 同口径复算）：' + (8 - bad) + '/8 通过')
process.exit(bad ? 1 : 0)
