import { pathToFileURL } from 'node:url';
const mod = await import(pathToFileURL("D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh"+'/src/simbench.js').href);
const { fieldSpec, routeTokens, SIM_ROOT, SIM_ROOT_SOURCE } = mod;
const f = fieldSpec();
console.log(JSON.stringify({SIM_ROOT, SIM_ROOT_SOURCE, ok:f.ok, name:f.name, md5:f.md5, start:f.start, end:f.end, mines:f.mines, obstacles:f.obstacles, targets:f.targets, bridges:f.bridges, barrier:f.barrier, drone:f.drone, target:f.target, grid:f.grid, bridgeRed:f.bridgeEntryRedundancyM, routeCount: routeTokens().length}, null, 2));
console.log('ROUTE:', routeTokens().join(' '));
