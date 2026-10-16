// 独立门禁复算：select#targetYaw 的 change 事件是否接通（不采信页面肉眼）。
// 判据：document.querySelectorAll('button[data-reset="targetYaw"]') 之外，
//       必须存在对 $('targetYaw') 的 addEventListener('change') 或 'input'。
import fs from 'node:fs'
const html = fs.readFileSync('D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh/sim.html', 'utf8')
let bad = 0
const chk = (n, ok, got) => { if (!ok) bad += 1; console.log((ok ? '  PASS ' : '  FAIL ') + n + (ok ? '' : '   got ' + got)) }
const hasChange = /targetYaw'\)\.addEventListener\('(change|input)'/.test(html)
chk('C1 select#targetYaw 绑定了 change/input 监听', hasChange, 'no listener')
chk('C2 reset 按钮（data-reset=targetYaw）存在', /data-reset="targetYaw"/.test(html), 'no reset')
console.log('select#targetYaw 事件接通：' + (2 - bad) + '/2 通过')
process.exit(bad ? 1 : 0)
