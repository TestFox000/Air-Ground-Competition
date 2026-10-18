import { pathToFileURL } from 'node:url';
const mod = await import(pathToFileURL("D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh"+'/src/simbench.js').href);
const { planRoute } = mod;
const r = await planRoute({ configPath: "D:/User/Desktop/比赛存档/v1/dzrobot_v2"+'/config/robot.yaml', mapPath: "D:/User/Desktop/比赛存档/v1/_tmp_session/sim_t28/probe/map_k9.yaml", timeoutMs: 120000 });
console.log(JSON.stringify({exitCode:r.exitCode, engine:r.engine, tokens:r.tokens.length, checkExit:r.check?.exit, stderrTail:r.stderrTail, tokensList:r.tokens.join(' ')}, null, 2));
