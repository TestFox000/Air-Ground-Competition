import { pathToFileURL } from 'node:url';
const sb = await import(pathToFileURL("D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh"+'/src/simbench.js').href);
const t = sb.routeTokens();
const cells = t.map(x => String(x).trim().replace(/^R:/i,'').replace(/[,:].*$/,'').replace(/\*$/,'').trim().toUpperCase());
const stops = t.filter(x => /\*$/.test(String(x)));
const pivots = t.filter(x => /:PIVOT/i.test(String(x)));
console.log(JSON.stringify({count:t.length, tokens:t, cells, stops, pivots, k7: cells.indexOf('K7'), k5: cells.indexOf('K5'), first:cells[0], last:cells[cells.length-1]}, null, 2));
