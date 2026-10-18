import { pathToFileURL } from 'node:url';
const mod = await import(pathToFileURL("D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh"+'/src/simbench.js').href);
const toks = mod.routeTokens();
const ngram = n => { const m = new Map(); for (let i=0;i+n<=toks.length;i++){ const k=toks.slice(i,i+n).join('>'); m.set(k,(m.get(k)||0)+1);} return [...m.entries()].filter(([k,v])=>v>1); };
console.log('bigrams repeated:', JSON.stringify(ngram(2)));
console.log('trigrams repeated:', JSON.stringify(ngram(3)));
console.log('4grams repeated:', JSON.stringify(ngram(4)));
// count duplicated cell occurrences that are adjacent in time within 3 tokens
