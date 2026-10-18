import { pathToFileURL } from 'node:url';
import fs from 'node:fs';
const mod = await import(pathToFileURL("D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh"+'/src/simbench.js').href);
const res = await mod.runSim({ label: 'sixth_map_sim_t28', overrides: { 'pursuit.cruise_speed_mps': '0.15' }, inject: {} });
const rec = res.record || {};
const out = {
  ok: res.ok,
  error: res.error || null,
  id: rec.id, status: rec.status, exitCode: rec.exitCode, seconds: rec.seconds,
  routeSource: rec.routeSource, routeCount: (rec.route||[]).length, route: rec.route,
  metrics: rec.metrics, files: rec.files, stdoutTail: res.stdoutTail, stderrTail: rec.stderrTail,
  configMd5: rec.configMd5, mapMd5: rec.mapMd5,
};
fs.writeFileSync("D:/User/Desktop/比赛存档/v1/_tmp_session/sim_t28/sim_run_sixth.json", JSON.stringify(out, null, 2));
console.log(JSON.stringify(out, null, 2));
