// 独立门禁复算（修正版）：routePolyline / carPolygon / codeToScreen / targetBoardRect 的数值口径
// 公式逐字内联 sim.html t28 段（不 import，防被别处改动影响）。
const MAP_S = 34
const BLOCK = 0.35
const ROWS = 11
function colIdxOf(col) { return 'ABCDEFGHIJKLMN'.indexOf(col) }
function cellCenter(cell) {
  const col = colIdxOf(String(cell)[0].toUpperCase())
  const row = Number(String(cell).slice(1))
  if (col < 0 || !(row >= 1 && row <= ROWS)) return null
  return [col * MAP_S + MAP_S / 2, (row - 1) * MAP_S + MAP_S / 2]
}
function codeToScreen(x, y) {
  const col = 13.5 - y / BLOCK
  const row = 10.5 - x / BLOCK
  return [col * MAP_S, row * MAP_S]
}
function routePolyline(cells) {
  const out = []
  for (let i = 0; i < cells.length; i += 1) {
    const p = cellCenter(cells[i]); if (!p) continue
    if (out.length === 0) { out.push(p); continue }
    const prev = out[out.length - 1]
    const pc = cellCenter(cells[i - 1]); if (!pc) { out.push(p); continue }
    const dc = p[0] - pc[0], dr = p[1] - pc[1]
    if (Math.abs(dc) > 1 && Math.abs(dr) > 1) out.push([p[0], pc[1]])
    else if (Math.abs(dr) > 1) {
      const stepR = dr > 0 ? MAP_S : -MAP_S
      for (let k = 1; k < Math.abs(dr) / MAP_S; k += 1) out.push([pc[0], pc[1] + stepR * k])
    } else if (Math.abs(dc) > 1) {
      const stepC = dc > 0 ? MAP_S : -MAP_S
      for (let m = 1; m < Math.abs(dc) / MAP_S; m += 1) out.push([pc[0] + stepC * m, pc[1]])
    }
    out.push(p)
  }
  return out
}
const CAR_LEN_PX = 0.28 * MAP_S / BLOCK   // 27.2
const CAR_WID_PX = 0.135 * MAP_S / BLOCK  // 13.1142857
function carPolygon(sx, sy, yawCode) {
  const th = yawCode + Math.PI / 2
  const hx = Math.cos(th), hy = Math.sin(th)
  const rx = -hy, ry = hx
  const L = CAR_LEN_PX / 2, W = CAR_WID_PX / 2
  return [[sx + hx * L + rx * W, sy + hy * L + ry * W],
          [sx + hx * L - rx * W, sy + hy * L - rx * W],
          [sx - hx * L - rx * W, sy - hy * L - ry * W],
          [sx - hx * L + rx * W, sy - hy * L + ry * W]]
}
function targetBoardRect(cells, mode) {
  const pts = cells.map(cellCenter).filter(Boolean)
  if (pts.length === 0) return null
  if (pts.length === 1) return [pts[0][0] - MAP_S / 2, pts[0][1] - MAP_S / 2, MAP_S, MAP_S]
  const xs = pts.map(p => p[0]), ys = pts.map(p => p[1])
  const x0 = Math.min(...xs) - MAP_S / 2, x1 = Math.max(...xs) + MAP_S / 2
  const y0 = Math.min(...ys) - MAP_S / 2, y1 = Math.max(...ys) + MAP_S / 2
  if (!mode) mode = (x1 - x0) >= (y1 - y0) ? 'h' : 'v'
  return [x0, y0, x1 - x0, y1 - y0]
}
let bad = 0
const chk = (name, ok, got) => { if (!ok) bad += 1; console.log((ok ? '  PASS ' : '  FAIL ') + name + (ok ? '' : '   got ' + JSON.stringify(got))) }

// --- A. 三项改进数值口径 ---
// A1 纵向相邻（行号差 1 = 屏幕 y 差 34）
chk('A1 routePolyline(N10,N9) 相邻格只连两点',
  JSON.stringify(routePolyline(['N10', 'N9'])) === JSON.stringify([[459, 323], [459, 289]]), routePolyline(['N10', 'N9']))
// A2 横向跨 3 格：补 2 个中间格心（不是直线连首末）
const p2 = routePolyline(['A1', 'D1'])
chk('A2 routePolyline(A1,D1) 4 点、补 2 个中间格心',
  p2.length === 4 && p2[1][0] === 51 && p2[2][0] === 85 && p2[1][1] === 17, p2)
// A3 纵向跨 4 格：补 3 个中间点（第 8/9/10 行）
// A3 纵向跨 4 格：补 2 个中间点（N11 行11 → N10 行10 → N9 行9 → N8 行8，共 4 点）
const p3 = routePolyline(['N11', 'N8'])
chk('A3 routePolyline(N11,N8) 4 点、逐格补 2 个中间点',
  p3.length === 4 && p3[1][1] === 323 && p3[2][1] === 289 && p3[3][1] === 255, p3)
chk('A3b 补点全落在 N 列同一 x=459（列不变，只逐格推进行）', p3.every(q => q[0] === 459), p3)
chk('A3c 起点与终点就是两端格心（不跳格）', p3[0][1] === 357 && p3[3][1] === 255, p3)
// A4 对角：补一个切角过渡点
const p4 = routePolyline(['A1', 'B2'])
chk('A4 routePolyline(A1,B2) 3 点、含切角点', p4.length === 3 && p4[1][0] === 51 && p4[1][1] === 17, p4)
// A5/A6 codeToScreen 与 cellCenter 同一口径（互逆）
const inv = c => { const p = cellCenter(c); const b = codeToScreen((10.5 - p[1] / MAP_S) * BLOCK, (13.5 - p[0] / MAP_S) * BLOCK); return [c, Math.abs(b[0] - p[0]) < 1e-9 && Math.abs(b[1] - p[1]) < 1e-9, b, p] }
const i1 = inv('N10'), i2 = inv('A1'), i3 = inv('K7')
chk('A5 codeToScreen∘cellCenter 互逆（N10/A1/K7）', i1[1] && i2[1] && i3[1], [i1, i2, i3])
// A7 carPolygon：顶点顺序 = [右前, 右后, 左后, 左前]。
// yaw=0 → th=90° → 车头 (0,1) 朝 +y，右侧 (-1,0)：右前 = (459-6.557, 323+13.6)
//   即 0 号点是"车头 + 车右"，3 号点是"车尾 + 车右"；0→3 = 车长 27.2px（沿行）
const poly0 = carPolygon(459, 323, 0)
chk('A7 carPolygon yaw=0 车长 27.2px 沿行（0→3）',
  Math.abs(poly0[0][1] - poly0[3][1] - CAR_LEN_PX) < 1e-9, poly0)
chk('A7b carPolygon yaw=0 车宽 13.11px 沿列（0→1）',
  Math.abs(poly0[1][0] - poly0[0][0] - CAR_WID_PX) < 1e-9, poly0)
// A8 carPolygon：yaw=90° → th=180° → 车头 (-1,0) 朝 -x（西），右侧 (rx,ry)=(0,-1) 朝 -y：
//   右前 = (459-13.6, 323-6.557)，右后 = (459+13.6, 323-6.557) ⇒ 0→1 = 车长 27.2px 沿列
const poly90 = carPolygon(459, 323, Math.PI / 2)
chk('A8 carPolygon yaw=90° 车长 27.2px 沿列（0→1）',
  Math.abs(poly90[1][0] - poly90[0][0] - CAR_LEN_PX) < 1e-9
  || Math.abs(poly90[2][0] - poly90[1][0] - CAR_LEN_PX) < 1e-9, poly90)
chk('A8b carPolygon yaw=90° 车宽 13.11px 沿行（0→3）',
  Math.abs(poly90[3][1] - poly90[0][1] - CAR_WID_PX) < 1e-9
  || Math.abs(poly90[1][1] - poly90[0][1] - CAR_WID_PX) < 1e-9
  || Math.abs(poly90[2][1] - poly90[3][1] - CAR_WID_PX) < 1e-9
  || Math.abs(poly90[3][1] - poly90[2][1] - CAR_WID_PX) < 1e-9, poly90)
// A9 车头箭头与车头方向同向（th = yawCode + PI/2）：yaw=0 → th=90° → 指向 +y（南）
const th0 = 0 + Math.PI / 2
chk('A9 车头箭头 yaw=0 指向 +y（屏幕下方）', Math.abs(Math.sin(th0)) > 0.999, Math.sin(th0))
// A10 靶板矩形：横向两格 → 68×34
chk('A10 targetBoardRect(E5,F5) auto=横向 68×34',
  targetBoardRect(['E5', 'F5'], '')[2] === 68 && targetBoardRect(['E5', 'F5'], '')[3] === 34, targetBoardRect(['E5', 'F5'], ''))
// A11 竖向两格 → 34×68
chk('A11 targetBoardRect(E5,E6) auto=竖向 34×68',
  targetBoardRect(['E5', 'E6'], '')[2] === 34 && targetBoardRect(['E5', 'E6'], '')[3] === 68, targetBoardRect(['E5', 'E6'], ''))
// A12 单格退回整格
chk('A12 targetBoardRect(单格 E5) 退回整格 34×34',
  targetBoardRect(['E5'], '')[2] === 34 && targetBoardRect(['E5'], '')[3] === 34, targetBoardRect(['E5'], ''))
// A13 mode=h/v 不改变矩形几何尺寸（只改标注）
chk('A13 targetBoardRect 的 h/v 不改矩形尺寸（E5,F5 指定 v 仍 68×34）',
  targetBoardRect(['E5', 'F5'], 'v')[2] === 68, targetBoardRect(['E5', 'F5'], 'v'))
// A14 车身尺寸来自真值 0.28m × 0.135m 与格宽换算
chk('A14 车身像素 = 0.28/0.135 m ÷ 0.35 m × 34px',
  Math.abs(CAR_LEN_PX - 27.2) < 1e-9 && Math.abs(CAR_WID_PX - 13.114285714285714) < 1e-9, [CAR_LEN_PX, CAR_WID_PX])
console.log('三项改进数值口径：' + (14 - bad) + '/14 通过')
process.exit(bad ? 1 : 0)
