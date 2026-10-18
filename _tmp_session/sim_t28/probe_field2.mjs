import { pathToFileURL } from 'node:url';
const sb = await import(pathToFileURL("D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh"+'/src/simbench.js').href);
const f = sb.fieldSpec();
console.log(JSON.stringify({drone:f.drone, planner:f.planner, mines:f.mines.length, obstacles:f.obstacles.length, targets:f.targets, bridges:f.bridges, target:f.target}, null, 2));
