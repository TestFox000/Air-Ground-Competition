import { pathToFileURL } from 'node:url';
import crypto from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
const PLUGIN = "D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh";
const ROOT = "D:/User/Desktop/比赛存档/v1/dzrobot_v2";
const sb = await import(pathToFileURL(PLUGIN + '/src/simbench.js').href);
const md5 = f => crypto.createHash('md5').update(fs.readFileSync(f)).digest('hex');
const before = { map: md5(ROOT + '/config/map.yaml'), robot: md5(ROOT + '/config/robot.yaml') };
const field = sb.fieldSpec();
const res = await sb.runSim({ label: 'sixth_map_edit_422', overrides: { 'pursuit.cruise_speed_mps': '0.15' },
  field: { mines: field.mines.filter(c => c !== 'A2').concat('A1') } });
const after = { map: md5(ROOT + '/config/map.yaml'), robot: md5(ROOT + '/config/robot.yaml') };
const rec = res.record || {};
const overrideDir = rec.override ? rec.override.dir : '';
const overrideFiles = overrideDir && fs.existsSync(overrideDir) ? fs.readdirSync(overrideDir) : [];
const r = {
  ok: res.ok, code: res.code, error: res.error,
  status: rec.status, plan: rec.plan, field: rec.field,
  planTail: res.planTail ? String(res.planTail).split('\n').filter(Boolean).slice(-2) : [],
  md5Before: before, md5After: after, configUnchanged: before.map === after.map && before.robot === after.robot,
  overrideDir, overrideFiles,
};
fs.writeFileSync("D:/User/Desktop/比赛存档/v1/_tmp_session/sim_t28/sim_edit_422.json", JSON.stringify(r, null, 2));
console.log(JSON.stringify(r, null, 2));
