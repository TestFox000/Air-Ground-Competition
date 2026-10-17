// List files inside an Electron asar archive whose paths match a regex.
const fs = require('fs');

const asarPath = process.argv[2];
const pattern = new RegExp(process.argv[3] || '.', 'i');

const fd = fs.openSync(asarPath, 'r');
const prefix = Buffer.alloc(16);
fs.readSync(fd, prefix, 0, 16, 0);
const headerSize = prefix.readUInt32LE(12);
const headerBuf = Buffer.alloc(headerSize);
fs.readSync(fd, headerBuf, 0, headerSize, 16);
fs.closeSync(fd);

const header = JSON.parse(headerBuf.toString('utf8'));

const out = [];
function walk(node, path) {
  if (node.files) {
    for (const [name, child] of Object.entries(node.files)) {
      walk(child, path ? `${path}/${name}` : name);
    }
  } else {
    out.push({ path, size: node.size || 0 });
  }
}
walk(header, '');

console.log(`total files: ${out.length}`);
for (const f of out.filter((f) => pattern.test(f.path)).slice(0, 200)) {
  console.log(`${String(f.size).padStart(10)}  ${f.path}`);
}
