import { pathToFileURL } from 'node:url';
const mod = await import(pathToFileURL("D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh"+'/src/simbench.js').href);
const toks = mod.routeTokens();
const cellOf = t => String(t).trim().replace(/^R:/i,'').replace(/[,:].*$/,'').replace(/\*$/,'').trim().toUpperCase();
const seen = new Map();
toks.forEach((t,i)=>{ const c = cellOf(t); const arr = seen.get(c)||[]; arr.push({i,t}); seen.set(c,arr); });
const dups = [...seen.entries()].filter(([c,arr])=>arr.length>1);
console.log('tokens', toks.length, 'uniqueCells', seen.size);
for (const [c,arr] of dups) console.log(c, JSON.stringify(arr.map(x=>x.i+':'+x.t)));
console.log('consecutive same raw:', toks.filter((t,i)=>i>0 && t===toks[i-1]).length);
console.log('pivot count', toks.filter(t=>/:PIVOT/i.test(t)).length);
// adjacency: are duplicates separated by pivot tokens?
