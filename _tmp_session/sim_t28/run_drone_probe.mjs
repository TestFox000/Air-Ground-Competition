import { pathToFileURL } from 'node:url';
import fs from 'node:fs';
const ds = await import(pathToFileURL("D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh"+'/src/dronesim.js').href);
const field = (await import(pathToFileURL("D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh"+'/src/simbench.js').href)).fieldSpec();
const res = await ds.runDroneSim({ field, seed: 20261018, label: 'sixth_drone_probe' });
const summary = { ok: res.ok, code: res.code, error: res.error,
  reroutes: res.reroutes, confirmed: res.confirmed,
  timeline: (res.timeline||[]).filter(e => ['no-intel','rejected','reroute-fallback','first-plan','reroute','finish'].includes(e.event)),
  scanPoints: res.scan ? res.scan.points : null };
fs.writeFileSync("D:/User/Desktop/比赛存档/v1/_tmp_session/sim_t28/drone_probe.json", JSON.stringify(summary, null, 2));
console.log(JSON.stringify(summary, null, 2));
