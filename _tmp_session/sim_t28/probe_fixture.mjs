import { pathToFileURL } from 'node:url';
const sb = await import(pathToFileURL("D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh"+'/src/simbench.js').href);
const { FIXTURE_MAP } = await import(pathToFileURL("D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh"+'/scripts/fixtures/solvable-map.mjs').href);
const r = await sb.planRoute({ configPath: "D:/User/Desktop/比赛存档/v1/dzrobot_v2"+'/config/robot.yaml', mapPath: FIXTURE_MAP, timeoutMs: 120000 });
console.log(JSON.stringify({exitCode:r.exitCode, engine:r.engine, tokens:r.tokens.length, checkExit:r.check?.exit, first:r.tokens[0], last:r.tokens.slice(-1), stderrTail:r.stderrTail, tokensList:r.tokens.join(' ')}, null, 2));
