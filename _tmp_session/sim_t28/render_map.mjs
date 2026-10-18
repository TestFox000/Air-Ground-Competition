import { pathToFileURL } from 'node:url';
import fs from 'node:fs';
import path from 'node:path';

const PLUGIN = "D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh";
const ROOT = "D:/User/Desktop/比赛存档/v1/dzrobot_v2";
const OUT = "D:/User/Desktop/比赛存档/v1/_tmp_session/sim_t28";
const runMeta = JSON.parse(fs.readFileSync(path.join(OUT, 'sim_run_sixth.json'), 'utf8'));
const runId = runMeta.id;
const csvText = fs.readFileSync(path.join(ROOT, 'artifacts', 'sim', runId, 'traj.csv'), 'utf8');
const html = fs.readFileSync(path.join(PLUGIN, 'sim.html'), 'utf8');
const body = html.match(/<script>([\s\S]*?)<\/script>/)[1];

const els = {};
function el(id) {
  if (!els[id]) els[id] = {
    id, value: '', checked: id === 'showRoute', innerHTML: '', textContent: '', className: '',
    style: {}, parentNode: null, addEventListener() {},
    getElementsByTagName() { return [{ innerHTML: '' }]; },
    querySelectorAll() { return []; },
    getAttribute() { return ''; },
  };
  return els[id];
}
const documentStub = { getElementById: el, querySelectorAll() { return []; }, addEventListener() {} };
const windowStub = { addEventListener() {} };
const fetchStub = url => String(url || '').includes('traj.csv')
  ? Promise.resolve({ ok: true, text: () => Promise.resolve(csvText), json: () => Promise.resolve({}) })
  : Promise.resolve({ ok: true, json: () => Promise.resolve({}), text: () => Promise.resolve('') });
const api = new Function('document', 'window', 'fetch', 'location',
  'return (function(){' + body + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
  + ' loadTrace: loadTrace, setTrace: function(t){ TRACE = t; }, getTrace: function(){ return TRACE; } }; })();'
)(documentStub, windowStub, fetchStub, { hash: '' });
const sb = await import(pathToFileURL(path.join(PLUGIN, 'src', 'simbench.js')).href);
const field = sb.fieldSpec();
field.route = sb.routeTokens();
api.setField(field);
for (const key of ['mines', 'obstacles', 'bridges', 'targets']) el(key).value = (field[key] || []).join(' ');

function wrap(svg, subtitle, legend) {
  return '<!doctype html><html><head><meta charset="utf-8"><style>'
    + 'body{margin:0;background:#fff;font-family:"Microsoft YaHei",system-ui,sans-serif;color:#111}'
    + '.head{padding:18px 24px 6px}.head h1{font-size:22px;margin:0 0 4px}'
    + '.head .sub{font-size:13px;color:#555;line-height:1.7}'
    + '.map{margin:6px 24px 0;width:1150px}.map svg{width:100%;height:auto;display:block}'
    + '.legend{margin:10px 24px 0;font-size:13px;color:#333;display:flex;flex-wrap:wrap;gap:6px 18px}'
    + '.legend i{display:inline-block;width:12px;height:12px;border-radius:3px;margin-right:6px;vertical-align:-1px;border:1px solid rgba(0,0,0,.35)}'
    + '.legend .ln{display:inline-block;width:18px;height:0;border-top:3px solid #111;margin:0 6px -3px 0}'
    + '.legend .ln.b{border-top-color:#2f7fd0}.legend .ln.r{border-top-color:#2f9e44}'
    + '.foot{margin:10px 24px 18px;font-size:12px;color:#777}'
    + '</style></head><body>'
    + '<div class="head"><h1>第六张图 · 仿真台横置场地视图</h1><div class="sub">' + subtitle + '</div></div>'
    + '<div class="map">' + svg + '</div>'
    + '<div class="legend">' + legend + '</div>'
    + '<div class="foot">蓝色折线 = 参考路线（route.tokens）；绿色折线 = 本局真值轨迹（traj.csv）；深灰车身 = 每 2.5 s 位姿抽样（0.28 m × 0.135 m 等比）。</div>'
    + '</body></html>';
}
const legend = '<span><i style="background:#e5484d"></i>雷（8）</span>'
  + '<span><i style="background:#4a4a4a"></i>障（8）</span>'
  + '<span><i style="background:#7fe3ea"></i>桥洞（K5/K7）</span>'
  + '<span><i style="background:#d9d9d9"></i>封闭桥室</span>'
  + '<span><i style="background:#e8a0c8"></i>靶板 D5+D6（竖向双格）</span>'
  + '<span><span class="ln"></span>隔板（第 6/7 行之间 D~N）</span>'
  + '<span><i style="background:#b2f2bb"></i>起点 N10</span>'
  + '<span><i style="background:#ffe066"></i>终点 N5</span>'
  + '<span><i style="background:#4d8fd6"></i>无人机 N8 起 / M2 降</span>'
  + '<span><span class="ln b"></span>参考路线</span><span><span class="ln r"></span>本局轨迹</span>';

// ① 仅参考路线（不含本局轨迹）
el('targetYaw').value = '';
api.setTrace(null);
api.renderMap();
const svgRoute = el('map').innerHTML;
const htmlRoute = wrap(svgRoute,
  '14 列 A~N 水平 × 11 行 1~11 垂直 · 每格 0.35 m · 起 N10 → 终 N5 · 隔板行 6/7 之间 D~N 封死 · 桥洞仅 K5/K7',
  legend);
fs.writeFileSync(path.join(OUT, 'map_view_route.html'), htmlRoute, 'utf8');
fs.writeFileSync(path.join(OUT, 'map_view_route.svg'), svgRoute, 'utf8');

// ② 叠加本局真值轨迹与位姿
await api.loadTrace({ id: runId, route: field.route });
const trace = api.getTrace();
const svgSim = el('map').innerHTML;
const htmlSim = wrap(svgSim,
  'run ' + runId + ' · 68 token / 24.85 m / 166.68 s · 跑完全程=是 · 越界 0 拍 · 横向RMS 0.1173 m'
  + '<br>路线来源 config/robot.yaml（车端手动路线；planner.auto_plan_enabled=' + (field.planner && field.planner.autoPlanEnabled) + '）',
  legend);
fs.writeFileSync(path.join(OUT, 'map_view_sim.html'), htmlSim, 'utf8');
fs.writeFileSync(path.join(OUT, 'map_view_sim.svg'), svgSim, 'utf8');
console.log(JSON.stringify({ runId, svgBytes: svgSim.length, tracePoints: trace.points.length, poses: trace.poses.length, routeTokens: field.route.length }, null, 2));
