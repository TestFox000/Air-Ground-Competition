import { pathToFileURL } from 'node:url';
const mod = await import(pathToFileURL("D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh"+'/src/simbench.js').href);
const { planRoute, SIM_ROOT } = mod;
const r = await planRoute({ configPath: "D:/User/Desktop/比赛存档/v1/dzrobot_v2"+'/config/robot.yaml', mapPath: "D:/User/Desktop/比赛存档/v1/_tmp_session/sim_t28/probe/map_empty.yaml", timeoutMs: 120000 });
console.log(JSON.stringify({exitCode:r.exitCode, timedOut:r.timedOut, engine:r.engine, tokens:r.tokens.length, checkExit:r.check?.exit, failures:r.failures, stderrTail:r.stderrTail, stdoutTail:r.stdoutTail, tokensList:r.tokens.join(' ')}, null, 2));
