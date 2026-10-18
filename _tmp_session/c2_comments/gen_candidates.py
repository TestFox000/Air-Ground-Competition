"""c2 插件侧注释精简 —— 候选生成器（只读，不写插件树）。

用途：把「已完结卡号标签」从注释里剥掉，生成 OLD ||| NEW 对照供人工复核。
评论判定（保守）：整行注释（// /* * <!-- 或 HTML 注释续行 ★）、或代码行里 `//` 之后的注释段。
"""
import re, sys, pathlib

PLUGIN = pathlib.Path(r'D:\User\Desktop\比赛存档\v1\dzrobot_v2\dsh-plugin-dzrobot-ssh')
OUT = pathlib.Path(r'D:\User\Desktop\比赛存档\v1\_tmp_session\c2_comments\candidates.txt')

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
]

COMMENT_START = re.compile(r'^\s*(//|/\*|\*|<!--)')


def looks_like_comment(line: str) -> bool:
    if COMMENT_START.match(line):
        return True
    # HTML 注释续行（行首 ★ / 中文说明），交给规则判定后复核
    return bool(re.match(r'^\s*★', line))


def transform_comment(text: str) -> str:
    out = text
    for pattern, repl in RULES:
        out = pattern.sub(repl, out)
    # 收尾：标题横线前多出的空格、`── ` 双空格、行尾孤立的 `：`
    out = re.sub(r'──\s{2,}', '── ', out)
    out = re.sub(r'──\s*$', '──', out)
    return out


def main():
    chunks = []
    for rel in FILES:
        path = PLUGIN / rel
        lines = path.read_text(encoding='utf-8').splitlines()
        for idx, line in enumerate(lines, 1):
            if '★' not in line and not re.search(r'(?<![A-Za-z0-9_])' + TAG + r'(?![0-9A-Za-z_])', line):
                continue
            if not looks_like_comment(line):
                # 代码行：只在 `//` 之后有 tag 时处理注释段
                pos = line.find('//')
                if pos < 0 or not re.search(r'' + TAG, line[pos:]):
                    continue
                head, tail = line[:pos + 2], line[pos + 2:]
                new = head + transform_comment(tail)
            else:
                new = transform_comment(line)
            if new != line:
                chunks.append(f'### {rel}:{idx}\nOLD|{line}\nNEW|{new}\n')
    OUT.write_text('\n'.join(chunks), encoding='utf-8')
    print(f'{len(chunks)} changed lines -> {OUT}')


if __name__ == '__main__':
    main()
