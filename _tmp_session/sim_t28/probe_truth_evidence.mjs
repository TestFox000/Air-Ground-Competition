import { pathToFileURL } from 'node:url';
const d = await import(pathToFileURL("D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh"+'/src/dronesim.js').href);
const ev = await d.truthPlanEvidence({ board: {"mines":["C1","A3","E3","G4","I6","B7","D9","H10"],"obstacles":["A1","E2","H4","D8","I8","G9","B11","F11"],"targets":["E5","F5"],"bridges":["K5","K9"]}, truth: ["K5","K9"], id: 'probe_1792287615831', label: 'probe', drone: false });
console.log(JSON.stringify({exitCode: ev.exitCode, tokenCount: ev.tokenCount, checkExit: ev.checkExit, stderrTail: ev.stderrTail, tokens: ev.tokens.join(' ')}, null, 2));
