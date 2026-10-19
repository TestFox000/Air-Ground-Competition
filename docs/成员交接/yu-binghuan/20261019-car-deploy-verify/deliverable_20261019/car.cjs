#!/usr/bin/env node
/**
 * 车端 SSH 自动化入口（密码登录，非交互，弱链路强化）。
 *
 * 用法：
 *   node car.cjs "命令"                          # 单命令执行
 *   node car.cjs run <本地脚本路径>                # 上传本地脚本到车端 /tmp/car_run.sh 并执行
 *   node car.cjs get <远程路径> <本地路径>          # 下载文件
 *   node car.cjs put <本地路径> <远程路径>          # 上传文件
 *   node car.cjs batch                             # 从 stdin 逐行读命令，单连接顺序执行
 *
 * 环境变量：CAR_HOST(192.168.0.197) CAR_USER(duzhong) CAR_PW(duzhong)
 *           CAR_TIMEOUT_MS(120000) CAR_RETRIES(3) CAR_READY_MS(40000)
 * 退出码：0 成功；2 连接/认证失败；3 远程命令失败；4 传输失败；5 参数错误。
 */
'use strict';
const path = require('path');
const fs = require('fs');
let Client;
try {
  ({ Client } = require('ssh2'));
} catch (e) {
  ({ Client } = require('./node_modules/.pnpm/ssh2@1.17.0/node_modules/ssh2'));
}

const HOST = process.env.CAR_HOST || '192.168.0.197';
const USER = process.env.CAR_USER || 'duzhong';
const PASSWORD = process.env.CAR_PW || 'duzhong';
const TIMEOUT_MS = parseInt(process.env.CAR_TIMEOUT_MS || '120000', 10);
const RETRIES = parseInt(process.env.CAR_RETRIES || '3', 10);
const READY_TIMEOUT_MS = parseInt(process.env.CAR_READY_MS || '40000', 10);

const argv = process.argv.slice(2);
const op = argv[0];
if (!op) {
  console.error('usage: node car.cjs "<cmd>" | run <local.sh> | get <remote> <local> | put <local> <remote> | batch');
  process.exit(5);
}

const watchdog = setTimeout(() => {
  console.error(`[car] timeout after ${TIMEOUT_MS}ms (host=${HOST})`);
  process.exit(2);
}, TIMEOUT_MS);
watchdog.unref();

const connectOpts = () => ({
  host: HOST,
  port: 22,
  username: USER,
  password: PASSWORD,
  readyTimeout: READY_TIMEOUT_MS,
  keepaliveInterval: 8000,
  keepaliveCountMax: 5,
});

const conn = new Client();
let attempt = 0;
conn.on('error', (err) => {
  const retriable = /handshake|ECONNRESET|ETIMEDOUT|EHOSTUNREACH|ENETUNREACH|connection lost|Keepalive|Not connected/i.test(err.message);
  if (retriable && attempt < RETRIES) {
    attempt += 1;
    console.error(`[car] attempt ${attempt}/${RETRIES} failed: ${err.message}, retrying in 4s...`);
    setTimeout(() => conn.connect(connectOpts()), 4000);
  } else {
    console.error(`[car] ssh error: ${err.message}`);
    process.exit(2);
  }
});
conn.connect(connectOpts());

/** 在当前连接上执行一条命令，返回 Promise<{code, out, err}> */
function execOnce(cmd, opts = {}) {
  return new Promise((resolve) => {
    conn.exec(cmd, { pty: false }, (err, stream) => {
      if (err) return resolve({ code: -1, out: '', err: String(err.message) });
      let out = '', errOut = '';
      stream.on('data', (d) => { out += d.toString(); });
      stream.stderr.on('data', (d) => { errOut += d.toString(); });
      stream.on('close', (code) => resolve({ code: code ?? -1, out, err: errOut }));
    });
  });
}

function sftpOnce() {
  return new Promise((resolve, reject) => {
    conn.sftp((err, sftp) => (err ? reject(err) : resolve(sftp)));
  });
}

function readStdin() {
  return new Promise((resolve) => {
    let data = '';
    process.stdin.setEncoding('utf8');
    process.stdin.on('data', (c) => { data += c; });
    process.stdin.on('end', () => resolve(data));
    if (process.stdin.isTTY) resolve('');
  });
}

conn.on('ready', async () => {
  try {
    if (op === 'get' || op === 'put') {
      const [a1, a2] = argv.slice(1);
      if (!a1 || !a2) { console.error('get/put need two paths'); process.exit(5); }
      const sftp = await sftpOnce();
      sftp[a1 === a1 ? 'fastGet' : 'fastGet']; // noop keep linter quiet
      const finish = (e) => {
        clearTimeout(watchdog);
        if (e) { console.error(`[car] transfer failed: ${e.message}`); process.exit(4); }
        console.log(`[car] transfer ok: ${op} ${a1} -> ${a2}`);
        conn.end();
        process.exit(0);
      };
      if (op === 'get') sftp.fastGet(a1, path.resolve(a2), {}, (e) => finish(e || null));
      else sftp.fastPut(path.resolve(a1), a2, {}, (e) => finish(e || null));
      return;
    }
    if (op === 'run') {
      const [scriptPath] = argv.slice(1);
      if (!scriptPath || !fs.existsSync(scriptPath)) { console.error('run: script not found'); process.exit(5); }
      const local = fs.readFileSync(scriptPath, 'utf8');
      const remote = '/tmp/car_run.sh';
      const sftp = await sftpOnce();
      await new Promise((res, rej) => sftp.writeFile(remote, local, (e) => (e ? rej(e) : res())));
      const r = await execOnce(`bash ${remote}`);
      clearTimeout(watchdog);
      if (r.out) process.stdout.write(r.out);
      if (r.err) process.stderr.write(r.err);
      console.log(`[car] run exit=${r.code}`);
      conn.end();
      process.exit(r.code === 0 ? 0 : 3);
    }
    if (op === 'batch') {
      const data = await readStdin();
      const cmds = data.split(/\r?\n/).map((l) => l.trim()).filter((l) => l && !l.startsWith('#'));
      let hadFail = false;
      for (let i = 0; i < cmds.length; i++) {
        console.log(`\n===== [${i + 1}/${cmds.length}] ${cmds[i].slice(0, 100)}`);
        const r = await execOnce(cmds[i]);
        if (r.out) process.stdout.write(r.out);
        if (r.err) process.stderr.write(r.err);
        if (r.code !== 0) { hadFail = true; console.log(`----- exit=${r.code}`); }
      }
      clearTimeout(watchdog);
      conn.end();
      process.exit(hadFail ? 3 : 0);
    }
    // 默认：把所有参数拼成一条命令
    const r = await execOnce(argv.join(' '));
    clearTimeout(watchdog);
    if (r.out) process.stdout.write(r.out);
    if (r.err) process.stderr.write(r.err);
    conn.end();
    process.exit(r.code === 0 ? 0 : 3);
  } catch (e) {
    console.error(`[car] ${e.message}`);
    process.exit(3);
  }
});
