"""c2 插件侧注释精简 —— 第 1 步：剥掉注释里的已完结卡号标签（只动注释，不动代码）。

与 gen_candidates.py 同规则；额外做几处「括号内只剩标点」的收尾。
"""
import re, pathlib

PLUGIN = pathlib.Path(r'D:\User\Desktop\比赛存档\v1\dzrobot_v2\dsh-plugin-dzrobot-ssh')
FILES = ['sim.html', 'index.js', 'client.js', 'src/simbench.js', 'src/dronesim.js',
         'src/stressbench.js', 'src/store.js', 'src/vnc.js', 'src/ws.js',
         'src/metrics.js', 'src/files.js', 'src/reload.js', 'src/ssh.js', 'src/trust.js']

TAG = r't\d{1,3}(?:-[A-Z]\d+)?'
RULES = [
    (re.compile(r'★\s*' + TAG + r'\s*[：:]\s*'), ''),
    (re.compile(r'★\s*' + TAG + r'\s*'), ''),
    (re.compile(r'──\s*★\s*'), '── '),
    (re.compile(r'──\s*' + TAG + r'\s*[：:]\s*'), '── '),
    (re.compile(r'（\s*' + TAG + r'\s*[：:]\s*'), '（'),
    (re.compile(r'（\s*' + TAG + r'\s*[）]'), ''),
    (re.compile(r'（\s*' + TAG + r'\s*[，,]\s*'), '（'),
    (re.compile(r'' + TAG + r'（卡\d+）\s*[：:]\s*'), ''),
    (re.compile(r'' + TAG + r'\s*[：:]\s*'), ''),
    (re.compile(r'(?<=/\* ── )★\s*'), ''),
    (re.compile(r'(?<=// )★\s*'), ''),
    (re.compile(r'(?<=<!-- )★\s*'), ''),
    (re.compile(r'(?<=\* )★\s*'), ''),
    (re.compile(r'(?<=/\*\* )★\s*'), ''),
]
COMMENT_START = re.compile(r'^\s*(//|/\*|\*|<!--)')


def looks_like_comment(line: str) -> bool:
    return bool(COMMENT_START.match(line) or re.match(r'^\s*★', line))


def transform(text: str) -> str:
    out = text
    for pattern, repl in RULES:
        out = pattern.sub(repl, out)
    out = re.sub(r'（\s*[；;]\s*', '（', out)          # 括号内只剩分号
    out = re.sub(r'──\s{2,}', '── ', out)
    out = re.sub(r'──\s*$', '──', out)
    out = out.replace('定容硬顶+ ', '定容硬顶 + ')
    return out


def main():
    total = 0
    for rel in FILES:
        path = PLUGIN / rel
        lines = path.read_text(encoding='utf-8').split('\n')
        changed = 0
        for i, line in enumerate(lines):
            if not ('★' in line or re.search(r'(?<![A-Za-z0-9_])' + TAG + r'(?![0-9A-Za-z_])', line)):
                continue
            if looks_like_comment(line):
                new = transform(line)
            else:
                pos = line.find('//')
                if pos < 0 or not re.search(TAG, line[pos:]):
                    continue
                new = line[:pos + 2] + transform(line[pos + 2:])
            if new != line:
                lines[i] = new
                changed += 1
        if changed:
            path.write_text('\n'.join(lines), encoding='utf-8')
            print(f'{rel}: {changed} lines')
            total += changed
    print(f'total {total}')


if __name__ == '__main__':
    main()
