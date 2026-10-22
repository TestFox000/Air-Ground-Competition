/**
 * test-sim-map-render.mjs — 仿真台「横置地图视图」的离线核对（t25 → t28 三组新增断言）。
 *
 * 为什么需要它：地图视图是**页面里的纯前端渲染**，没有浏览器就看不见；而"14 列水平 × 11 行垂直、
 * 每格标格名、雷红/障灰/洞青/靶粉/隔板/起终/无人机蓝"这些正是验收项。这里把 `sim.html` 的内联脚本
 * 放进一个 DOM 桩里跑，喂真实的 `config/map.yaml` 解析结果（`src/simbench.js` 的 `fieldSpec()`），
 * 然后逐条断言产出的 SVG 字符串 —— 不需要浏览器、不需要起服务。
 *
 * ★ t28 补测（本卡）：三项改进各一组**净增**断言，都是直接单测内联函数，不是"看一眼 SVG"：
 *   A 路径折线 `routePolyline` —— 相邻格之间补中间格心（多段），不是首末两点连直线；
 *   B 小车位姿 `carPolygon` —— 车身尺寸 = 真值 0.28 m × 0.135 m（≈27.2 × 13.11 px），且随 yaw 转动；
 *   C 靶板朝向 `targetBoardRect` —— h/v/auto 三态都正确作用到并板矩形上。
 *
 * ★ t2 补测（sim-bench2）：靶朝向**透传链**逐段钉住 + 两处收紧（原断言只查存在性）：
 *   A 折线：跨格路径点数 **>** token 数、相邻点恒单格（真实路线全逐格正交时点数 = token 数 —— t1 起期望值从
 *     field.route 动态推导，不再写死「54 token」）；
 *   B 位姿：车身/箭头条数 **= 抽样拍数**（不再是 "> 0"），只有首拍带「起点位姿」；
 *   D 透传链①：select#targetYaw → `readForm()` → `body.targetYaw`（空/非法值**不带该键**）；
 *   E 透传链②：宿主 `sanitizeParams()`（`src/simbench.js`）取值与返回值；③ 落盘见文件内注释（真跑取证）。
 * ★ t7 补测/返工（R1-R7）：
 *   A 折线：加**接线级**断言（FIELD.route=A1→C1→C4 ⇒ SVG 折线 6 点，见 R4）；
 *   B 位姿：车身抽样**独立成趟**（每 125 数据行 = 2.5 s @50 Hz，首行必画）⇒ 期望值来自规格
 *     `1 + floor((数据行数-1)/125)` 且相位恒隔 125 行（R1+R2）；补**绝对朝向**断言（R3）；
 *   R5：靶格底色 #e8a0c8 + 图例文案一致性；R6/R7：破坏实验说明与文件尾部记档改正。
 *   破坏实验的**可运行**实现见文件尾部「变异自证段」（成对谓词：真实现 true / 退化副本 false），
 *   接线级退化另有 R4 那条断言 + 独立验证方的内存替换复核；三种旧死写法的教训见 :302 起的记档。
 * ★ t1 补强（sim-fixer，闭合 t8 审查三缺口 T8-1 / T8-2 / G1）：
 *   T8-1：车头**箭头**方向守卫 —— 箭头与 carPolygon 必须共用 headVec 同口径（自造 4 分块 yaw 轨迹，
 *     因既有合成轨迹 yaw 恒 π/2，镜像分量 sin(π)≈0 ⇒ 对上下镜像全盲）；
 *   T8-2：解除「54 token 全单格」写死 —— 期望点数从 field.route 动态推导（先算 token 数再算格数）
 *     + ⇔ 性质断言 + 渲染层点数复核 + 跨格构造 A1→A11→N11「点数 > token 数」；
 *   G1：F3 事件守卫 —— #targetYaw change 注册 / 触发重绘 / targetYaw reset 专属分流（value 回 ''
 *     + 专属文案，message() 把 kind 写进 className ⇒ 可查 'msg ok'）/ mines reset 对照，
 *     用独立 DOM 桩记录 addEventListener 调用。
 *   外部变异取证（M-A/M-B/M-C1/M-C2/M-D/M-E）在 %TEMP%\t1_mut\ 跑：复制插件目录 + DZROBOT_SIM_ROOT 指回工作区。
 *
 *   node scripts/test-sim-map-render.mjs      # 全过 exit 0
 */
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { fieldSpec, routeTokens, sanitizeParams } from '../src/simbench.js'

const HERE = fileURLToPath(new URL('.', import.meta.url))
const HTML = path.join(HERE, '..', 'sim.html')

const els = {}
function el(id) {
  if (!els[id]) {
    els[id] = {
      id, value: '', checked: id === 'showRoute', innerHTML: '', textContent: '', className: '',
      style: {}, parentNode: null, addEventListener() {},
      getElementsByTagName() { return [{ innerHTML: '' }] },
      querySelectorAll() { return [] },
      getAttribute() { return '' },
    }
  }
  return els[id]
}

const field = fieldSpec()
if (!field.ok) {
  console.log('FAIL 读不到场地（config/map.yaml）：' + field.error)
  process.exit(1)
}
el('mines').value = field.mines.join(' ')
el('obstacles').value = field.obstacles.join(' ')
el('bridges').value = field.bridges.join(' ')

const body = fs.readFileSync(HTML, 'utf8').match(/<script>([\s\S]*?)<\/script>/)[1]
const documentStub = { getElementById: el, querySelectorAll() { return [] }, addEventListener() {} }
const windowStub = { addEventListener() {} }
const fetchStub = () => Promise.resolve({ json: () => Promise.resolve({}), text: () => Promise.resolve('') })
const api = new Function('document', 'window', 'fetch',
  'return (function(){' + body + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
  + ' cellCenter: cellCenter, fieldFromForm: fieldFromForm,'
  // ★ t28：单测这三条链，必须把函数暴露出来（sim.html 里是 function 声明，此处直接引用即可）
  + ' routePolyline: routePolyline, carPolygon: carPolygon, targetBoardRect: targetBoardRect,'
  // ★ t2：透传链①（表单 → 请求体）要断言 readForm() 真把 targetYaw 塞进 body
  + ' readForm: readForm,'
  // ★ t7（R3）：绝对朝向与轨迹坐标口径要能直接问被测函数
  + ' headVec: headVec, codeToScreen: codeToScreen }; })();'
)(documentStub, windowStub, fetchStub)
field.route = routeTokens()          // 页面里的 FIELD 由 /sim/map 带上参考路线（index.js 里拼的）
api.setField(field)
api.renderMap()
const svg = el('map').innerHTML

const count = re => (svg.match(re) || []).length
const checks = []
const check = (name, ok, detail) => checks.push([name, ok, detail])
const CELLS = field.grid.cols * field.grid.rows
// ★ t7：**规格**坐标换算（不与被测实现共享代码）——官方 tools/sim_map_render.py S(x,y)=(4.9-y, x)
//   + 本页 SVG y 轴向下 ⇒ 屏幕 = ((14 - y/0.35)·34, (11 - x/0.35)·34)。期望值一律用它算。
const SPEC_BLOCK = 0.35, SPEC_S = 34
const specScreen = (x, y) => [(field.grid.cols - y / SPEC_BLOCK) * SPEC_S, (field.grid.rows - x / SPEC_BLOCK) * SPEC_S]
const rawHtml = fs.readFileSync(HTML, 'utf8')          // ★ t7（R5）：图例文案/色值一致性

check('产出 SVG 且 viewBox 覆盖 14×11 格（477×375）',
  /viewBox="0 0 477 375"/.test(svg), (svg.match(/viewBox="[^"]+"/) || [''])[0])
check(`格背景 rect = 14×11 = ${CELLS} 个`, count(/stroke="#c9ced8"/g) === CELLS, 'rects=' + count(/stroke="#c9ced8"/g))
// ★ t28：靶板改成"一块并板"（横/竖朝向可选），多出一个朝向标注（font-size 8.5，与格名同号）
check(`每格格名标签 ${CELLS} 个（A1…N11）+ 1 个靶板朝向标注`,
  count(/font-size="8.5"/g) === CELLS + 1, 'labels=' + count(/font-size="8.5"/g))
check('四角格名都在（A1 / N1 / A11 / N11）',
  ['>A1<', '>N1<', '>A11<', '>N11<'].every(s => svg.includes(s)), 'missing 角标')
check('横置方向：A 在左（x=0）、N 在第 14 列（x=442）',
  svg.includes('<rect x="0" y="0" width="34" height="34"') && svg.includes('<rect x="442" y="0" width="34" height="34"'), 'A/N')
check('行方向：1 行在上（y=0）、11 行在下（y=340）', svg.includes('<rect x="0" y="340" width="34" height="34"'), 'row11')
check(`雷：${field.mines.length} 个红圈 + 「雷」字`,
  count(/fill="#e5484d"/g) === field.mines.length && count(/>雷</g) === field.mines.length, 'mines')
check(`障：${field.obstacles.length} 个灰块 + 「障」字`,
  count(/fill="#4a4a4a"/g) === field.obstacles.length && count(/>障</g) === field.obstacles.length, 'obstacles')
check(`桥洞：青色 + 「洞」字 ×${field.bridges.length}（= map.bridges）`,
  count(/>洞</g) === field.bridges.length && svg.includes('#7fe3ea'), 'bridges')
check('封闭桥室：K 列非开放格浅灰', svg.includes('#d9d9d9'), 'closed bridge rooms')
// ★ t28：靶板并成一块（粉色描边矩形 + 「靶板」字 + 朝向标注），不再每格一个「靶」字
//   期望 `>靶<` = 0：sim.html:722 注释明写"不再每格一个『靶』字"，且 text() 产出的是
//   `>靶板</text>`（"靶"后面跟"板"而不是"<"），所以旧的 / >靶< / 计数恒为 0 —— 这是忠实表达设计意图。
check(`靶板：一块并板（${field.targets.join('+')}）+「靶板」字；t28 起不再每格出「靶」字`,
  count(/>靶板</g) === 1 && count(/>靶</g) === 0 && /stroke="#8a2f5f"/.test(svg),
  '靶板=' + count(/>靶板</g) + ' 靶=' + count(/>靶</g))
check('靶板朝向标注（横向/竖向）', /横向|竖向/.test(svg), 'target yaw label')
check('隔板：第 6/7 行之间 D~N 黑条 + 标注', svg.includes('隔板 D~N') && svg.includes('fill="#111111"'), 'barrier')
check('起点 N10：绿底 + 「起」', svg.includes('#b2f2bb') && svg.includes('>起<'), 'start')
check('终点 N5：金底 + 「终」', svg.includes('#ffe066') && svg.includes('>终<'), 'end')
check(`无人机起 ${field.drone.takeoff}：蓝 + 「机」`, svg.includes('>机<') && svg.includes('#4d8fd6'), 'takeoff')
check(`无人机降 ${field.drone.land}：蓝 + 「H」图标`, svg.includes('>H<'), 'land')
check(`参考路线折线已画（${field.route.length} token，动态读自 field.route）`, /<polyline[^>]+stroke="#2f7fd0"/.test(svg), 'route polyline')
check('坐标映射：N10 → x=459, y=323（右下）；A1 → (17,17)；N11 → (459,357)',
  JSON.stringify(api.cellCenter('N10')) === '[459,323]'
  && JSON.stringify(api.cellCenter('A1')) === '[17,17]'
  && JSON.stringify(api.cellCenter('N11')) === '[459,357]', 'cellCenter')
check('未改点位 → fieldFromForm() = null（请求体不带 field）', api.fieldFromForm() === null, 'null')
el('obstacles').value = field.obstacles.join(' ') + ' H8'
check('改了障碍 → fieldFromForm() 只报 obstacles', Boolean((api.fieldFromForm() || {}).obstacles), JSON.stringify(api.fieldFromForm()))

// ── ★ t28 三组净增断言（A 折线 / B 位姿 / C 靶朝向）──────────────────────────
// 口径来源：MAP_S=34 px/格、FIELD_GRID_BLOCK=0.35 m/格（由 cellCenter('A1')===[17,17] 反证）。

// A. 真实路径折线：多格路径必须补**中间格心**（多段），不是首末两点连直线
//    A1→D1：4 点，中间两点 = B1 格心 (51,17) 与 C1 格心 (85,17)
const polyA1D1 = api.routePolyline(['A1', 'D1'])
check('A 折线：A1→D1 补 2 个中间格心，共 4 点（多段折线，非首末直线）',
  polyA1D1.length === 4
  && JSON.stringify(polyA1D1) === JSON.stringify([[17, 17], [51, 17], [85, 17], [119, 17]]),
  JSON.stringify(polyA1D1))
//    N11→N8：4 点且 x 恒为 459（中间两点 = N10/N9 格心，纵向逐格补）
const polyN11N8 = api.routePolyline(['N11', 'N8'])
check('A 折线：N11→N8 纵向逐格补点，4 点且 x 恒为 459',
  polyN11N8.length === 4 && polyN11N8.every(p => p[0] === 459)
  && JSON.stringify(polyN11N8) === JSON.stringify([[459, 357], [459, 323], [459, 289], [459, 255]]),
  JSON.stringify(polyN11N8))
//    ★ t1（T8-2）：解除「54 token 全单格步」写死 —— 期望点数从 field.route **动态推导**：
//      先解析 token 数得到格序列，再逐段算跨格补点（对角含单格对角 +1 切角点、纵横多格 +(n-1)）。
//      口径 = sim.html:595-618 的声明意图（逐格补中间格心 + 对角补切角过渡点），在测试侧按格数重实现。
//      当前 config 的路线全正交 ⇒ 推导出「点数 = token 数」是**算出来的**，不是写死的 54。
const routeCells = field.route.map(t => String(t).trim().replace(/^R:/i, '').replace(/[,:].*$/, '')
  .replace(/\*$/, '').trim().toUpperCase()).filter(Boolean)
const crossStepPtsOf = cells => cells.slice(1).reduce((n, cell, i) => {
  const prev = cells[i]
  const dc = Math.abs(prev.charCodeAt(0) - cell.charCodeAt(0))
  const dr = Math.abs(Number(prev.slice(1)) - Number(cell.slice(1)))
  if (dc > 0 && dr > 0) return n + 1                    // 对角（含单格对角）：补 1 个切角过渡点
  if (dr > 1) return n + (dr - 1)                       // 纵向多格：逐格补点
  if (dc > 1) return n + (dc - 1)                       // 横向多格：逐格补点
  return n
}, 0)
const expectedPtsOf = cells => cells.length + crossStepPtsOf(cells)
const polyRoute = api.routePolyline(routeCells)
const expectedRoutePts = expectedPtsOf(routeCells)
check(`A 折线：真实路线（${field.route.length} token）折线点数 = token 数 + 动态跨格补点数 = ${expectedRoutePts}（先算 token 数再算格数，不写死 54）`,
  polyRoute.length === expectedRoutePts,
  'pts=' + polyRoute.length + ' expected=' + expectedRoutePts + ' tokens=' + field.route.length)
//    ★ t2 补测（本卡 C 项「折线多段：点数应显著多于 token 数，不能等于 token 数」）：
//    卡面这句对**真实路线不成立**——本卡实测 54 个 token 的每一步 dc+dr 恒等于 1
//    （跨格步 0 处，由下面 `routeGridSteps` 复核），routePolyline 的插值分支在它上面 0 次触发，
//    所以点数恒 = token 数；照卡面写 ">" 会永远 FAIL（假红，不是发现的真 bug）。
//    因此用**跨格合成路径**把"多段而非首末直连"这条性质钉死，并对真实路线保留上面
//    "点数 = token 数 + 逐段可复算"两条强比对（它们对 0 处跨格的真实路线仍然有牙）。
const polyMulti = api.routePolyline(['A1', 'C1', 'C4'])          // 3 token：A1→C1 跨 2 列、C1→C4 跨 3 行
const multiSteps = polyMulti.slice(1).map((q, i) =>
  Math.abs(q[0] - polyMulti[i][0]) + Math.abs(q[1] - polyMulti[i][1]))
check('A 折线：跨格路径补中间格心 ⇒ 点数 > token 数（A1→C1→C4：3 token → 6 点），相邻点恒为单格 34 px（不是首末直连）',
  polyMulti.length === 6 && polyMulti.length > 3 && multiSteps.every(d => d === 34)
  && JSON.stringify(polyMulti) === JSON.stringify([[17, 17], [51, 17], [85, 17], [85, 51], [85, 85], [85, 119]]),
  'pts=' + polyMulti.length + ' tokens=3 dists=' + JSON.stringify(multiSteps))
//    ★ t1（T8-2）：渲染层点数复核 —— 上面问的是 routePolyline 的返回值，这里问 renderMap 画出来的 SVG
//      （观测面从函数返回值换成渲染产物；renderMap 若绕过 routePolyline，这里会先红）。
const svgRoutePts = (((svg.match(/<polyline[^>]*points="([^"]*)"/) || [, ''])[1]) || '').trim().split(/\s+/).filter(Boolean).length
check(`A 折线（渲染层）：SVG 参考路线折线点数 = 动态期望 ${expectedRoutePts} 点（renderMap 没绕过 routePolyline）`,
  svgRoutePts === expectedRoutePts, 'svgPts=' + svgRoutePts + ' expected=' + expectedRoutePts)
//    ★ t1（T8-2）：跨格构造 —— 用 field.grid 拼一条跨格路线，把「点数 > token 数」在真渲染链上钉死。
//      期望值来自规格几何：A1→A11 纵跨 11 格、A11→N11 横跨 14 格，A11 拐角共享一次
//      ⇒ 点数 = rows + cols − 1 = 11 + 14 − 1 = 24 > 3 token。
const CROSS_ROUTE = ['A1', 'A11', 'N11']
const polyCross = api.routePolyline(CROSS_ROUTE)
const crossDists = polyCross.slice(1).map((q, i) => Math.abs(q[0] - polyCross[i][0]) + Math.abs(q[1] - polyCross[i][1]))
const crossExpectedPts = field.grid.rows + field.grid.cols - 1
check(`A 折线（跨格构造）：A1→A11→N11 = ${crossExpectedPts} 点 > 3 token（点数>token 数在跨格路线上成立），相邻点恒 1 格 ${SPEC_S} px`,
  polyCross.length === crossExpectedPts && polyCross.length > CROSS_ROUTE.length
  && crossDists.every(d => d === SPEC_S),
  'pts=' + polyCross.length + ' expected=' + crossExpectedPts + ' tokens=' + CROSS_ROUTE.length
  + ' dists=' + JSON.stringify(crossDists))
//    ★ t1（T8-2）：⇔ 性质断言 —— 解耦「54 全单格」写死的通用判据：点数 > token 数 **当且仅当** 存在跨格步。
//      对当前全正交路线两侧同为 false（这也是被验证的性质，不是假红）；对上面的跨格构造两侧同为 true。
check('A 折线：点数 > token 数 ⇔ 存在跨格步（通用性质；当前全正交路线两侧 false、跨格构造两侧 true）',
  (polyRoute.length > field.route.length) === (crossStepPtsOf(routeCells) > 0)
  && (polyCross.length > CROSS_ROUTE.length) === (crossStepPtsOf(CROSS_ROUTE) > 0),
  '真实: pts=' + polyRoute.length + ' tokens=' + field.route.length + ' 跨格补点=' + crossStepPtsOf(routeCells)
  + '；跨格: pts=' + polyCross.length + ' tokens=' + CROSS_ROUTE.length + ' 跨格补点=' + crossStepPtsOf(CROSS_ROUTE))

// B. 小车位姿：车身 = 真值 0.28 m × 0.135 m → 27.2 × 13.114… px，且随 yaw 转动
//   ★ t7（R3）更正：旧注释把方向写反了。codeToScreen 的 row = 11 - x/block ⇒
//     代码 +x（行号变小）在屏幕上是**向上**，所以 yawCode=0 的车头朝屏幕**上方**，
//     长边落在 y 轴：carPolygon(459,323,0) 的包围盒 = 宽 13.11 × 高 27.20。
//   ★ 换算口径：官方 sim_map_render.py 是 matplotlib（y 轴向上）⇒ 屏幕方向 (cos θs, sin θs)；
//     本页 SVG 的 y 轴向下、场地朝向与官方图逐格相同 ⇒ 屏幕方向必须是 (cos θs, -sin θs)，
//     即 y 分量取反。官方自检锚点「代码 yaw=+90°（面朝 A 列）画出来朝左」两式都满足
//     （水平分量同号），所以只有补**非水平**朝向断言才抓得住镜像。
const CAR_LEN_PX = 0.28 * 34 / 0.35                     // 27.2（车长）
const CAR_WID_PX = 0.135 * 34 / 0.35                    // 13.114…（车宽）
const bbox = pts => [Math.max(...pts.map(p => p[0])) - Math.min(...pts.map(p => p[0])),
                     Math.max(...pts.map(p => p[1])) - Math.min(...pts.map(p => p[1]))]
const car0 = api.carPolygon(459, 323, 0)
check(`B 位姿：车身包围盒 = ${CAR_WID_PX.toFixed(2)} × ${CAR_LEN_PX.toFixed(2)} px（宽 0.135 m × 高 0.28 m；yaw=0 车头朝屏幕**上**，长边在 y 轴）`,
  car0.length === 4
  && Math.abs(bbox(car0)[0] - CAR_WID_PX) < 0.01 && Math.abs(bbox(car0)[1] - CAR_LEN_PX) < 0.01,
  JSON.stringify(car0.map(p => p.map(v => v.toFixed(2)))))
const car90 = api.carPolygon(459, 323, Math.PI / 2)
check('B 位姿：yaw 转 90° 后包围盒长宽互换（13.11×27.2 → 27.2×13.11），顶点真的动了',
  Math.abs(bbox(car90)[0] - CAR_LEN_PX) < 0.01 && Math.abs(bbox(car90)[1] - CAR_WID_PX) < 0.01,
  JSON.stringify(car90.map(p => p.map(v => v.toFixed(2)))))
check('B 位姿：yaw=0 与 yaw=π/2 的顶点集合互不相同（防"尺寸对了但其实没转"假绿）',
  JSON.stringify(car0.map(p => p.map(v => v.toFixed(2))))
  !== JSON.stringify(car90.map(p => p.map(v => v.toFixed(2)))), '顶点集合相同')
//    ★ t7（R3）：**绝对朝向**断言 —— 上面三条只测尺寸/是否转动，对"上下镜像"全盲
//    （实测：镜像版同样 PASS）。规格值：headVec(0)=(0,-1) 屏幕向上；headVec(π/2)=(-1,0)
//    朝左（= 官方锚点「代码 yaw=+90° 面朝 A 列画出来朝左」）。
const hv0 = api.headVec(0)
const hv90 = api.headVec(Math.PI / 2)
const near = (a, b) => Math.abs(a - b) < 1e-9
check('B 位姿（R3）：绝对朝向 headVec(0) = (0, -1)（y 分量为负 = 屏幕向上）、headVec(π/2) = (-1, 0)（朝左 = 面朝 A 列）',
  near(hv0[0], 0) && near(hv0[1], -1) && near(hv90[0], -1) && near(hv90[1], 0),
  'headVec(0)=' + JSON.stringify(hv0.map(v => Number(v.toFixed(6))))
  + ' headVec(π/2)=' + JSON.stringify(hv90.map(v => Number(v.toFixed(6)))))
const car0FrontMid = [(car0[0][0] + car0[1][0]) / 2, (car0[0][1] + car0[1][1]) / 2]
check('B 位姿（R3）：carPolygon(459,323,0) 的车头中点 y < 车心 y（车身画在真值朝向这一侧，不是镜像那一侧）',
  car0FrontMid[1] < 323 - CAR_LEN_PX / 2 + 0.01 && near(car0FrontMid[0], 459),
  'frontMid=' + JSON.stringify(car0FrontMid.map(v => Number(v.toFixed(2)))))
//    位姿标注整体落地：画一条合成轨迹后，SVG 里必须有车身 path + 车头线 + 「起点位姿」文字
const traceRows = []
for (let i = 0; i < 400; i += 1) {
  const y = 0.35 + (i / 399) * 3.0
  traceRows.push([i + 1, y.toFixed(4), '4.9', '1.5708', '0.30'].join(','))
}
const CSV = 'step,truth_x,truth_y,truth_yaw,truth_v\n' + traceRows.join('\n')
const traceEls = {}
const elTrace = id => (traceEls[id] = traceEls[id] || el(id))
const traceFetch = url => String(url || '').includes('traj.csv')
  ? Promise.resolve({ ok: true, text: () => Promise.resolve(CSV) })
  : fetchStub()
const traceApi = new Function('document', 'window', 'fetch', 'location',
  'return (function(){' + body + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
  // ★ t7：车身抽样要按**规格**核对相位 ⇒ 直接读 TRACE.poses；朝向/坐标口径也要暴露
  + ' loadTrace: loadTrace, getPoses: function(){ return TRACE.poses; },'
  + ' headVec: headVec, codeToScreen: codeToScreen }; })();'
)({ getElementById: elTrace, querySelectorAll() { return [] }, addEventListener() {} },
  windowStub, traceFetch, { hash: '' })
traceApi.setField(field)
await traceApi.loadTrace({ id: 'synthetic-trace', route: field.route })
const traceSvg = el('map').innerHTML
const traceCount = re => (traceSvg.match(re) || []).length
//    ★ t7（R1+R2）：期望值必须来自**规格**，不能照抄实现里的抽样写法（旧断言就是这么写的：
//    `i += 4` 与 `i % 125 === 0` 共用一个 index ⇒ 期望 2 条，把「相位错、每 10 s 才一个车身」
//    这个缺陷一起钉成了"正确"）。规格：车身独立成趟、每 125 个数据行画一个（2.5 s @50 Hz）、
//    首行必画 ⇒ 车身数 = 1 + floor((数据行数 - 1) / 125)。本合成 400 行 ⇒ 4 条（行 0/125/250/375）。
const expectedPoses = 1 + Math.floor((traceRows.length - 1) / 125)
check(`B 位姿（R2）：车身 path = 车头箭头 = 规格式车身数 ${expectedPoses}（1 + floor((${traceRows.length} - 1)/125)），且只有首拍带「起点位姿」`,
  traceCount(/<path[^>]*fill="#1f2933"/g) === expectedPoses
  && traceCount(/<line[^>]*stroke="#0b1727"/g) === expectedPoses
  && traceCount(/>起点位姿</g) === 1,
  'bodies=' + traceCount(/<path[^>]*fill="#1f2933"/g) + ' arrows=' + traceCount(/<line[^>]*stroke="#0b1727"/g)
  + ' expected=' + expectedPoses + ' 起点位姿=' + traceCount(/>起点位姿</g))
//    ★ t7（R2）：相位断言 —— 相邻车身必须**恒隔 125 个数据行**（不是"数量对了就算过"）。
//    期望位置用 specScreen() 规格换算（不调用被测实现）；合成 CSV 的 truth_x = 0.35 + i/399*3.0。
const poseRows = []
for (let i = 0; i < traceRows.length; i += 125) poseRows.push(i)
const expectedPosePos = poseRows.map(i => specScreen(0.35 + (i / (traceRows.length - 1)) * 3.0, 4.9))
const poses = traceApi.getPoses()
check('B 位姿（R2）：相邻车身恒隔 125 个数据行（相位恰好 = 行 0/125/250/375，首行必画）',
  poses.length === poseRows.length
  && poses.every((p, k) => Math.abs(p[0] - expectedPosePos[k][0]) < 0.01
    && Math.abs(p[1] - expectedPosePos[k][1]) < 0.01 && Math.abs(p[2] - 1.5708) < 1e-6),
  'rows=' + JSON.stringify(poseRows) + ' poses=' + JSON.stringify(poses.map(p => p.map(v => Number(v.toFixed(2))))))
//    ★ t7（R3）：轨迹坐标 —— 真 traj.csv 首拍 truth=(0.525, 0.175) = geometry.block_to_meters('N10')
//    （实测 artifacts/sim/traj.csv 首行）必须落在 cellCenter('N10')，即与格心同口径。
check('B 位姿（R3）：轨迹坐标口径 —— codeToScreen(0.525, 0.175) = cellCenter(N10) = [459, 323] = 规格值',
  JSON.stringify(traceApi.codeToScreen(0.525, 0.175)) === '[459,323]'
  && JSON.stringify(api.cellCenter('N10')) === '[459,323]'
  && JSON.stringify(specScreen(0.525, 0.175)) === '[459,323]',
  'codeToScreen=' + JSON.stringify(traceApi.codeToScreen(0.525, 0.175)) + ' cellCenter=' + JSON.stringify(api.cellCenter('N10')))
{
  let worst = 0, worstCell = ''
  for (let c = 0; c < field.grid.colLetters.length; c += 1) {
    for (let r = 1; r <= field.grid.rows; r += 1) {
      const cell = field.grid.colLetters[c] + r
      const bx = (10.5 - (r - 1)) * SPEC_BLOCK, by = (13.5 - c) * SPEC_BLOCK     // geometry.block_to_meters
      const got = traceApi.codeToScreen(bx, by), want = api.cellCenter(cell), spec = specScreen(bx, by)
      const d = Math.max(Math.abs(got[0] - want[0]), Math.abs(got[1] - want[1]), Math.abs(got[0] - spec[0]), Math.abs(got[1] - spec[1]))
      if (d > worst) { worst = d; worstCell = cell }
    }
  }
  const worstTxt = worst < 1e-6 ? '0（浮点残差 ' + worst.toExponential(1) + '）' : worst.toFixed(2)
  check(`B 位姿（R3）：154 格全比 codeToScreen = cellCenter = 规格值（最大偏差 ${worstTxt} px；旧式少半格 = 24.04 px）`,
    worst < 1e-6, 'worst=' + worst.toExponential(2) + ' cell=' + worstCell)
}

// ── ★ t1（T8-1）：车头**箭头**方向守卫 —— 箭头与 carPolygon 必须共用 headVec 同口径 ──────
// 规格锚点：官方 tools/sim_map_render.py 的声明意图「yaw=0 车头朝上、+90° 朝左」+ 本页 SVG y 轴向下
//   ⇒ 箭头单位方向 = (cos(yaw+90°), −sin(yaw+90°))，长度 = CAR_LEN_PX×0.62（sim.html:816-820 的画法声明）。
// 为什么不用上面的合成轨迹：它 yaw 恒为 π/2，镜像分量 sin(π)≈0 ⇒ 对「上下镜像」全盲（t8 的缺口）。
// 这里自造 500 行、yaw 分块 [0, π/2, π/4, 3π/2]（抽样行 0/125/250/375 恰好各取一档）。
const arrowRows = []
for (let i = 0; i < 500; i += 1) {
  const yawBlock = i < 125 ? 0 : i < 250 ? Math.PI / 2 : i < 375 ? Math.PI / 4 : 3 * Math.PI / 2
  arrowRows.push([i + 1, (0.35 + (i / 499) * 3.0).toFixed(4), '4.9', yawBlock.toFixed(4), '0.30'].join(','))
}
const ARROW_CSV = 'step,truth_x,truth_y,truth_yaw,truth_v\n' + arrowRows.join('\n')
const arrowEls = {}
const elArrow = id => (arrowEls[id] = arrowEls[id] || el(id))
const arrowFetch = url => String(url || '').includes('traj.csv')
  ? Promise.resolve({ ok: true, text: () => Promise.resolve(ARROW_CSV) })
  : fetchStub()
const arrowApi = new Function('document', 'window', 'fetch', 'location',
  'return (function(){' + body + '\n; return { setField: function(f){ FIELD = f; },'
  + ' loadTrace: loadTrace, getPoses: function(){ return TRACE.poses; },'
  + ' carPolygon: carPolygon, headVec: headVec }; })();'
)({ getElementById: elArrow, querySelectorAll() { return [] }, addEventListener() {} },
  windowStub, arrowFetch, { hash: '' })
arrowApi.setField(field)
await arrowApi.loadTrace({ id: 'arrow-trace', route: field.route })
const arrowSvg = el('map').innerHTML
const arrowDirs = []
{
  const ARROW_LINE_RE = /<line x1="(-?[\d.]+)" y1="(-?[\d.]+)" x2="(-?[\d.]+)" y2="(-?[\d.]+)" stroke="#0b1727"/g
  let m
  while ((m = ARROW_LINE_RE.exec(arrowSvg)) !== null) {
    arrowDirs.push([Number(m[3]) - Number(m[1]), Number(m[4]) - Number(m[2])])
  }
}
const SAMPLED_YAWS = [0, Math.PI / 2, Math.PI / 4, 3 * Math.PI / 2]  // 数据行 0/125/250/375 的 truth_yaw
const norm2 = v => { const L = Math.hypot(v[0], v[1]); return L ? [v[0] / L, v[1] / L, L] : [NaN, NaN, 0] }
const specHead = yaw => [Math.cos(yaw + Math.PI / 2), -Math.sin(yaw + Math.PI / 2)]   // 规格：θs=yaw+90°、SVG y 向下取反
const arrowN = arrowDirs.map(norm2)
check('T8-1a：车头箭头方向 = 规格 head(yaw)（θs=yaw+90°、y 取反）× 4 分块 yaw（0/π/2/π/4/3π/2），长度 = CAR_LEN_PX×0.62',
  arrowN.length === 4 && arrowN.every((a, k) => {
    const s = specHead(SAMPLED_YAWS[k])
    // 方向容差 0.03（toFixed(1) 舍入最坏 0.05/分量 ÷ 16.86 px 臂长 ≈ 0.003）；长度容差 0.12 =
    // 两端各 toFixed(1) 最坏 0.05×2 + 舍入组合（实测 16.864→16.829/16.9）；镜像偏差 ≥1.4 仍有 >10 倍余量
    return Math.abs(a[0] - s[0]) < 0.03 && Math.abs(a[1] - s[1]) < 0.03
      && Math.abs(a[2] - CAR_LEN_PX * 0.62) < 0.12
  }),
  'dirs=' + JSON.stringify(arrowN.map(a => a.map(v => Number(v.toFixed(3))))))
const arrowPoses = arrowApi.getPoses()
const carHeadDirs = arrowPoses.map(p => {
  const poly = arrowApi.carPolygon(p[0], p[1], p[2])
  return norm2([(poly[0][0] + poly[1][0]) / 2 - p[0], (poly[0][1] + poly[1][1]) / 2 - p[1]])
})
check('T8-1b：箭头方向 = carPolygon 车头侧中点方向（两者共用 headVec 同口径，防箭头单独漂成镜像）',
  arrowN.length === 4 && carHeadDirs.length === 4
  && arrowN.every((a, k) => Math.abs(a[0] - carHeadDirs[k][0]) < 0.03 && Math.abs(a[1] - carHeadDirs[k][1]) < 0.03),
  'arrow=' + JSON.stringify(arrowN.map(a => a.slice(0, 2).map(v => Number(v.toFixed(3)))))
  + ' car=' + JSON.stringify(carHeadDirs.map(a => a.slice(0, 2).map(v => Number(v.toFixed(3))))))
api.setField(field)          // 复原 FIELD（箭头轨迹实例动过渲染面）
api.renderMap()

// C. 靶板朝向：h/v/auto 三态都正确作用到并板矩形上
//    auto：E5+F5 同格不同列 → 横向 68×34；E5+E6 同列不同行 → 竖向 34×68
const boardAutoH = api.targetBoardRect(['E5', 'F5'], '')
const boardAutoV = api.targetBoardRect(['E5', 'E6'], '')
check('C 靶朝向：auto 由两格几何自动判 —— E5+F5（左右相邻）→ 68×34 横向',
  JSON.stringify(boardAutoH) === JSON.stringify([136, 136, 68, 34]), JSON.stringify(boardAutoH))
check('C 靶朝向：auto 由两格几何自动判 —— E5+E6（上下相邻）→ 34×68 竖向',
  JSON.stringify(boardAutoV) === JSON.stringify([136, 136, 34, 68]), JSON.stringify(boardAutoV))
//    指定 h/v 只改标注不改尺寸（t28 语义：物理朝向由地图两格几何决定，select 只改渲染提示）
check('C 靶朝向：显式 h / v 覆盖 auto（尺寸仍是 68×34，标注随之切换）',
  JSON.stringify(api.targetBoardRect(['E5', 'F5'], 'h')) === JSON.stringify([136, 136, 68, 34])
  && JSON.stringify(api.targetBoardRect(['E5', 'F5'], 'v')) === JSON.stringify([136, 136, 68, 34]),
  'h=' + JSON.stringify(api.targetBoardRect(['E5', 'F5'], 'h'))
  + ' v=' + JSON.stringify(api.targetBoardRect(['E5', 'F5'], 'v')))
//    渲染层真的吃这个参数：select 设 'v' → 标注变「竖向」；设 'h' → 「横向」；非法值回落 auto
el('targetYaw').value = 'v'
api.renderMap()
const svgV = el('map').innerHTML
const labelOf = s => (s.match(/<text[^>]*>(横向|竖向)[^<]*<\/text>/) || ['', ''])[1]
check('C 靶朝向：targetYaw=「v」→ 渲染标注变「竖向」',
  labelOf(svgV) === '竖向' && !/横向/.test(svgV), 'label=' + labelOf(svgV))
el('targetYaw').value = 'h'
api.renderMap()
const svgH = el('map').innerHTML
check('C 靶朝向：targetYaw=「h」→ 渲染标注变「横向」',
  labelOf(svgH) === '横向' && !/竖向/.test(svgH), 'label=' + labelOf(svgH))
el('targetYaw').value = 'x'          // 非法值：targetYawMode() 回落 '' = 跟随地图
api.renderMap()
// ★ 2026-10-18：期望值必须从**真源两格几何**推导，不能写死第五张图的「横向」——
//   第六张图靶 = D5+D6 纵向两格（field.target.yaw='v'），写死会让换图后的守卫假红。
const boardReal = api.targetBoardRect(field.targets, '')
const autoLabel = boardReal && boardReal[2] >= boardReal[3] ? '横向' : '竖向'
check('C 靶朝向：targetYaw 非法值回落「跟随地图」→ 标注 = 自动判定（按 map.targets 几何动态推导：' + autoLabel + '）',
  labelOf(el('map').innerHTML) === autoLabel, 'label=' + labelOf(el('map').innerHTML) + ' expected=' + autoLabel)
//    宿主侧口径：地图真源（field.target + fieldSpec）与页面函数三者一致（h ⇔ 横长 / v ⇔ 竖长）
check('C 靶朝向：fieldSpec() 报的 target.yaw 与 targetBoardRect auto 一致（按真源两格几何动态推导）',
  field.target && (field.target.yaw === 'h' ? boardReal[2] >= boardReal[3]
    : field.target.yaw === 'v' ? boardReal[3] > boardReal[2] : false),
  JSON.stringify(field.target) + ' rect=' + JSON.stringify(boardReal))

// ── ★ t2 新增：靶朝向**透传链**（表单 → 请求体 → 宿主 sanitizeParams → 台账）────────────
// 三项改进里只有靶朝向带**持久化**（进 run_meta.jsonl），所以链条逐段钉住：
// 前端段＝D 组，宿主段＝E 组，落盘段见 E 组末的注释（要真跑 python 才看得见，用真跑取证）。

// D. 透传链①：select#targetYaw → readForm() → body.targetYaw（sim.html:352-353 / :558-561）
//    口径：只有 'h'/'v' 才写进请求体；'' = 跟随地图 ⇒ **不带该键**（不是写 null 或空串）。
const readFormWith = v => { el('targetYaw').value = v; return api.readForm() }
const bodyYawV = readFormWith('v')
const bodyYawH = readFormWith('h')
const bodyYawUpper = readFormWith('H')
const bodyYawAuto = readFormWith('')
const bodyYawBad = readFormWith('x')
const hasYaw = b => Object.prototype.hasOwnProperty.call(b, 'targetYaw')
check('D 透传链①（表单→请求体）：targetYaw = v / h 原样进 body，大写 H 归一为小写 h',
  bodyYawV.targetYaw === 'v' && bodyYawH.targetYaw === 'h' && bodyYawUpper.targetYaw === 'h',
  'v=' + JSON.stringify(bodyYawV.targetYaw) + ' h=' + JSON.stringify(bodyYawH.targetYaw)
  + ' H=' + JSON.stringify(bodyYawUpper.targetYaw))
check('D 透传链①（表单→请求体）：跟随地图（空）与非法值**不写该键**（区别"没传"与"传了空串"）',
  !hasYaw(bodyYawAuto) && !hasYaw(bodyYawBad),
  'auto.hasKey=' + hasYaw(bodyYawAuto) + ' bad.hasKey=' + hasYaw(bodyYawBad))
el('targetYaw').value = ''

// E. 透传链②：宿主 sanitizeParams（simbench.js:321 定义 / :344 取值 / :357 返回值）
const sp = b => sanitizeParams(b)
check('E 透传链②（宿主取值）：h / v 原样通过；非法值、大写、非字符串一律挡成 null（不做真值兜底）',
  sp({ targetYaw: 'v' }).targetYaw === 'v' && sp({ targetYaw: 'h' }).targetYaw === 'h'
  && sp({ targetYaw: 'x' }).targetYaw === null && sp({ targetYaw: '' }).targetYaw === null
  && sp({ targetYaw: 'V' }).targetYaw === null && sp({ targetYaw: true }).targetYaw === null
  && sp({ targetYaw: ['v'] }).targetYaw === null && sp({}).targetYaw === null,
  JSON.stringify([sp({ targetYaw: 'v' }).targetYaw, sp({ targetYaw: 'x' }).targetYaw,
    sp({ targetYaw: 'V' }).targetYaw, sp({ targetYaw: ['v'] }).targetYaw]))
check('E 透传链②（宿主返回值）：参数快照键表含 targetYaw（漏返回/改名会被抓），且靶朝向不进 overrides（仿真物理不感知）',
  JSON.stringify(Object.keys(sp({})))
    === JSON.stringify(['overrides', 'inject', 'label', 'maxSteps', 'seed', 'targetYaw', 'field'])
  && !Object.prototype.hasOwnProperty.call(sp({ targetYaw: 'v' }).overrides, 'targetYaw'),
  'keys=' + Object.keys(sp({})).join(','))
//    透传链③（record 落盘：simbench.js:476 → :516/:564 appendMeta → artifacts/sim/run_meta.jsonl）**不在这里断言**：
//    它要真跑一次 python sim 才看得见，属"非纯函数段"。取证方式（可复核）：
//    _tmp_session/t2_yaw/_tmp_t2_yaw_meta.mjs 起 5 次真 runSim —— run_meta.jsonl 落
//    `"targetYaw":"v"` / `"h"`，非法值落 null 且键仍在，listHistory() 原样回读。

// ── ★ t1（G1）：F3 事件守卫 —— change 监听注册 + 触发重绘 + reset 分流，独立 DOM 桩取证 ──────
// 规格锚点（sim.html 声明意图）：:1186 targetYaw 绑 change（改完地图重绘+提示刷新）；
//   :1187-1202 button[data-reset] 四键分流 —— targetYaw 走专属分支（value='' 跟随地图 + 专属文案），
//   其余键 join(' ')；message() 把 kind 写进 className（:326 `node.className = 'msg ' + kind`）⇒ 可查 'msg ok'。
//   旧缺口（t8/G1）：主套件的 documentStub.querySelectorAll 恒返 [] ⇒ 整个 F3 块从未被执行过，删掉零红。
function freshApiWithDom() {
  const reg = []                                     // addEventListener 调用记录（谁绑了什么事件）
  const gEls = {}
  const gEl = id => {
    if (!gEls[id]) {
      gEls[id] = {
        id, value: '', checked: id === 'showRoute', innerHTML: '', textContent: '', className: '',
        style: {}, parentNode: null,
        addEventListener(type, fn) { reg.push({ id: id, type: type, fn: fn }) },
        getElementsByTagName() { return [{ innerHTML: '' }] },
        querySelectorAll() { return [] },
        getAttribute() { return '' },
      }
    }
    return gEls[id]
  }
  const buttons = ['mines', 'obstacles', 'bridges', 'targetYaw'].map(key => ({
    innerHTML: '', className: '',
    addEventListener(type, fn) { reg.push({ id: 'button[data-reset=' + key + ']', type: type, fn: fn }) },
    getAttribute(name) { return name === 'data-reset' ? key : '' },
  }))
  const a = new Function('document', 'window', 'fetch', 'location',
    'return (function(){' + body + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; } }; })();'
  )({ getElementById: gEl,
      querySelectorAll(sel) { return /button\[data-reset\]/.test(String(sel)) ? buttons : [] },
      addEventListener() {} }, windowStub, fetchStub, { hash: '' })
  a.setField(field)                                  // FIELD.ok = true ⇒ reset 处理器可执行
  return { api: a, reg: reg, els: gEls, el: gEl }
}
const g1 = freshApiWithDom()
const g1Change = g1.reg.find(r => r.id === 'targetYaw' && r.type === 'change')
const g1ResetYaw = g1.reg.find(r => r.id === 'button[data-reset=targetYaw]' && r.type === 'click')
check('G1-1：F3 监听已注册 —— #targetYaw 绑 change、button[data-reset=targetYaw] 绑 click（删掉注册即红）',
  Boolean(g1Change) && Boolean(g1ResetYaw),
  'reg=' + JSON.stringify(g1.reg.map(r => r.id + ':' + r.type)))
{
  g1.el('map').innerHTML = '<sentinel/>'
  g1.el('targetYaw').value = 'v'
  if (g1Change) g1Change.fn()                        // 真触发：refreshFieldHints() + renderMap()
  check('G1-2：触发 targetYaw change → renderMap 重绘（哨兵被真 SVG 覆盖、viewBox 复位）',
    !g1.el('map').innerHTML.includes('sentinel') && /viewBox="0 0 477 375"/.test(g1.el('map').innerHTML),
    'html=' + g1.el('map').innerHTML.slice(0, 60))
}
const g3 = freshApiWithDom()
const g3Reset = g3.reg.find(r => r.id === 'button[data-reset=targetYaw]' && r.type === 'click')
g3.el('targetYaw').value = 'h'                        // 先改脏（变异体删掉注册时 el 不会被创建，必须走 gEl 工厂）
if (g3Reset) g3Reset.fn()
check("G1-3：targetYaw 恢复默认 → value 回 ''（跟随地图）+ runMsg 类含 ok + 专属文案「靶板朝向已恢复」",
  g3.el('targetYaw').value === '' && g3.el('runMsg').className.includes('ok')
  && g3.el('runMsg').textContent === '靶板朝向已恢复「跟随地图」',
  'value=' + JSON.stringify(g3.el('targetYaw').value) + ' class=' + g3.el('runMsg').className
  + ' msg=' + g3.el('runMsg').textContent)
const g4 = freshApiWithDom()
const g4Reset = g4.reg.find(r => r.id === 'button[data-reset=mines]' && r.type === 'click')
if (g4Reset) g4Reset.fn()
check("G1-4（对照）：mines 恢复默认仍走 join(' ') 文案（专属分流只给 targetYaw，不是全体静默空串）",
  g4.el('mines').value === field.mines.join(' ') && g4.el('runMsg').textContent === 'mines 已恢复现场默认值',
  'value=' + JSON.stringify(g4.el('mines').value) + ' msg=' + g4.el('runMsg').textContent)

// ── ★ t7（R4）：**接线级**断言 —— 前面单测的是 routePolyline 本体，这里测 renderMap 的调用点 ──
//    规格：FIELD.route = 跨格合成路线 A1→C1→C4（3 token：跨 2 列 + 跨 3 行）时必须逐格补点
//    ⇒ SVG 折线 **6** 个点；若 renderMap 绕过 routePolyline 直接连格心，只会是 3 个点。
//    （真实 54 token 全逐格正交，点数 = token 数，区分不出这种退化 ⇒ 必须用合成路线。）
{
  const savedRoute = field.route.slice()
  api.setField(Object.assign({}, field, { route: ['A1', 'C1', 'C4'] }))
  api.renderMap()
  const wirePts = (((el('map').innerHTML.match(/<polyline[^>]*points="([^"]*)"/) || [, ''])[1]) || '')
    .trim().split(/\s+/).filter(Boolean)
  check('R4 接线级：FIELD.route = A1→C1→C4 时 renderMap 画 6 个折线点（绕过 routePolyline 直连只有 3）',
    wirePts.length === 6 && wirePts[0] === '17.0,17.0' && wirePts[5] === '85.0,119.0',
    'pts=' + wirePts.length + ' ' + JSON.stringify(wirePts))
  api.setField(Object.assign({}, field, { route: savedRoute }))     // 复原 FIELD（后续断言仍用真路线）
  api.renderMap()
}

// ── ★ t7（R5）：靶格**底色**断言（原来的覆盖缺口：删掉 `if (targetSet[cell]) fill = '#e8a0c8';`
//    没有任何断言会红 —— 靶格会静默退化成白底）──
const targetCellRects = (svg.match(/<rect x="\d+" y="\d+" width="34" height="34" fill="#e8a0c8"/g) || []).length
check(`R5 底色：靶格 rect fill = #e8a0c8 恰 ${field.targets.length} 个 + 并板 1 块（删掉靶格填色那行会退化成白底）`,
  count(/fill="#e8a0c8"/g) === field.targets.length + 1 && targetCellRects === field.targets.length,
  'g8a0c8=' + count(/fill="#e8a0c8"/g) + ' 靶格rect=' + targetCellRects)
check('R5 图例：legend「跨格板」文案 + 色值与并板画法一致（图例说跨格板、SVG 画一块「靶板」）',
  rawHtml.includes('<i style="background:#e8a0c8"></i>靶（跨格板）') && svg.includes('>靶板<'),
  'legend/targetboard')

// ── 关于「破坏实验」：三种**死写法**的教训 + 现在怎么做（t7 重写；旧版「无法自证」的说法过强）──
// 曾三次尝试内置"改坏输入 → 断言必须 FAIL"的变异段、三次都假绿。复盘结论：那三次都是
// **脚手架实现 bug**，不是原理不可能 —— 所以「断言无法自证」不能当跳过的理由：
//
//  ① `mustFail(name, fn)` + `threw !== null || fn.length === 0`
//     **判据写错**：箭头函数无参时 length 恒 0 ⇒ "没抛错"也判 OK ⇒ 六条变异全假绿。
//  ② 变异输入与断言输入不一致（断言测 A1→D1，变异喂 A1→B1）
//     相邻格正确输出本就是 2 点，"if (length===4) throw" 永不成立 ⇒ 逻辑上不可能 FAIL。
//  ③ `expectFail(name, fn)` 把谓词写成"正确行为应满足的条件"（如 length < 4）
//     实现正确时该谓词恒为 false ⇒ 永远"未捕获" ⇒ 九条全 WARN。
//
// ⇒ 可行写法 = **成对谓词**：同一个谓词 + 同一输入，分别在 ①真实现 ②内存里用字符串锚点
//   替换出来的退化实现 上求值，要求「真实现 true 且 退化 false」；再加「锚点命中数必须
//   恰为 1」的护栏。恒真谓词会在退化侧失败、恒假谓词会在真实现侧失败 ⇒ 两侧都堵死。
//   **本文件下面的「变异自证段」就是按这个写法落地的**（4 组：函数级 / 接线级 / 朝向 / R1 相位）。
//
// 仍然需要外部/内存替换复核的只剩一层：**不在本文件观测面内**的调用链（独立验证员只读
//   sim.html → 内存替换 body → 跑真 renderMap，_tmp_verify_t3_fixsim.mjs 那一路）以及
//   真跑 python sim 才看得见的落盘段。即：外部手法不是因为"无法自证"，而是因为那一层
//   本文件管不着。

// ── 变异自证段（t7/R7）────────────────────────────────────────────────────────
// 写法来源：t4 审查员实测过的参考实现（%TEMP% 副本 45/45 全绿、wiring 变异体 exit 1）。
// 三原则：① 同一谓词 + 同一输入，两侧都真跑；② 必须 pristine===true 且 mutant===false
//        （恒真谓词在退化侧失败、恒假谓词在真实现侧失败 ⇒ 都不可能假绿）；
//        ③ 变异锚点命中数必须恰为 1，否则判 FAIL（重构时喊出来，不静默通过）。
function apiFromBody(srcBody) {
  const a = new Function('document', 'window', 'fetch',
    'return (function(){' + srcBody + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
    + ' cellCenter: cellCenter, routePolyline: routePolyline, headVec: headVec }; })();'
  )({ getElementById: el, querySelectorAll() { return [] }, addEventListener() {} }, windowStub, fetchStub)
  a.setField(field)                        // ★ 必须：否则 FIELD 为 null，cellCenter 会抛错
  return a
}
function mutantOf(find, repl) {
  const hits = body.split(find).length - 1
  if (hits !== 1) return null              // 锚点失配 ⇒ 由下面的 check 判 FAIL，不静默通过
  return apiFromBody(body.replace(find, repl))
}

// ① 函数级退化（routePolyline 退化成"首末两点直连"）
const PRED_POLY = a => JSON.stringify(a.routePolyline(['A1', 'D1'])) === JSON.stringify([[17, 17], [51, 17], [85, 17], [119, 17]])
const mPoly = mutantOf('function routePolyline(cells) {\n  var out = [];\n',
  'function routePolyline(cells) {\n  var out = [];\n  var __c = cells.map(cellCenter).filter(Boolean);\n  if (__c.length) return [__c[0], __c[__c.length - 1]];\n')
check('变异自证①（函数级）：routePolyline 退化成首末两点 ⇒ A1→D1 谓词由真变假',
  mPoly !== null && PRED_POLY(apiFromBody(body)) === true && PRED_POLY(mPoly) === false, 'mutantBuilt=' + (mPoly !== null))

// ② 接线级退化（renderMap 绕过 routePolyline 直接连格心）—— 这条正是"外部手法"要覆盖的那层
const wirePointCount = a => {
  a.setField(Object.assign({}, field, { route: ['A1', 'C1', 'C4'] }))
  a.renderMap()
  const m = el('map').innerHTML.match(/<polyline[^>]*points="([^"]*)"/)
  return m ? m[1].trim().split(/\s+/).length : 0
}
const mWire = mutantOf('    var pts = routePolyline(cells);', '    var pts = cells.map(cellCenter).filter(Boolean);')
check('变异自证②（接线级）：renderMap 绕过 routePolyline ⇒ 跨格折线点数由 6 降为 3',
  mWire !== null && wirePointCount(apiFromBody(body)) === 6 && wirePointCount(mWire) === 3, 'mutantBuilt=' + (mWire !== null))

// ③ 朝向类退化（headVec 的 y 分量镜像回按官方 matplotlib 的写法）
const PRED_HEAD = a => a.headVec(0)[1] < 0
const mHead = mutantOf('return [Math.cos(th), -Math.sin(th)];', 'return [Math.cos(th), Math.sin(th)];')
check('变异自证③（朝向）：headVec 的 y 分量镜像 ⇒ 朝向谓词由真变假',
  mHead !== null && PRED_HEAD(apiFromBody(body)) === true && PRED_HEAD(mHead) === false, 'mutantBuilt=' + (mHead !== null))

// ④ R1 的守卫：车身抽样相位退化（步长 125 → 500）
function traceApiFromBody(srcBody) {
  return new Function('document', 'window', 'fetch', 'location',
    'return (function(){' + srcBody + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
    + ' loadTrace: loadTrace, getPoses: function(){ return TRACE.poses; } }; })();'
  )({ getElementById: elTrace, querySelectorAll() { return [] }, addEventListener() {} },
    windowStub, traceFetch, { hash: '' })
}
const poseCountOf = async a => { a.setField(field); await a.loadTrace({ id: 'mutant-trace', route: field.route }); return a.getPoses().length }
const mPhase = body.split('j += 125').length - 1 === 1 ? traceApiFromBody(body.replace('j += 125', 'j += 500')) : null
check('变异自证④（R1 相位）：车身抽样步长 125 → 500 ⇒ 车身数由 ' + expectedPoses + ' 降为 1',
  mPhase !== null && (await poseCountOf(traceApiFromBody(body))) === expectedPoses && (await poseCountOf(mPhase)) === 1,
  'mutantBuilt=' + (mPhase !== null))
api.setField(field)          // 复原 FIELD（变异段里动过 FIELD.route）
api.renderMap()

let bad = 0
for (const [name, ok, detail] of checks) {
  if (!ok) bad += 1
  console.log((ok ? '  PASS ' : '  FAIL ') + name + (ok ? '' : '   → ' + detail))
}
console.log('横置地图渲染核对：' + (checks.length - bad) + '/' + checks.length + ' 通过')
process.exit(bad ? 1 : 0)
