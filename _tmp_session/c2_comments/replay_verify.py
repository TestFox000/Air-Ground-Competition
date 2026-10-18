"""验证：工作区 = HEAD + 本卡记录的两轮注释改写（精确重放）。

任何脚本外的手工/意外改动都会让重放结果 ≠ 工作区，从而暴露出来。
"""
import subprocess, pathlib, re, sys

REPO = pathlib.Path(r'D:\User\Desktop\比赛存档\v1\dzrobot_v2')
PLUGIN_REL = 'dsh-plugin-dzrobot-ssh'
FILES = ['sim.html', 'src/simbench.js', 'src/dronesim.js']

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


def looks_like_comment(line):
    return bool(COMMENT_START.match(line) or re.match(r'^\s*★', line))


def transform(text):
    out = text
    for pattern, repl in RULES:
        out = pattern.sub(repl, out)
    out = re.sub(r'（\s*[；;]\s*', '（', out)
    out = re.sub(r'──\s{2,}', '── ', out)
    out = re.sub(r'──\s*$', '──', out)
    out = out.replace('定容硬顶+ ', '定容硬顶 + ')
    return out


def apply_transform(text):
    lines = text.split('\n')
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
        lines[i] = new
    return '\n'.join(lines)


def load_fixes():
    spec = __import__('importlib.util').util.spec_from_file_location(
        'mf', pathlib.Path(__file__).with_name('manual_fix.py'))
    mod = __import__('importlib.util').util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.FIXES


def main():
    fixes = load_fixes()
    ok = True
    for rel in FILES:
        head = subprocess.run(['git', 'show', f'HEAD:{PLUGIN_REL}/{rel}'], cwd=REPO,
                              capture_output=True, check=True).stdout.decode('utf-8')
        head = head.replace('\r\n', '\n')
        replay = apply_transform(head)
        for frel, old, new in fixes:
            if frel != rel:
                continue
            if replay.count(old) != 1:
                print(f'{rel}: FATAL replay fix match={replay.count(old)}: {old[:60]}')
                ok = False
                continue
            replay = replay.replace(old, new, 1)
        cur = (REPO / PLUGIN_REL / rel).read_text(encoding='utf-8').replace('\r\n', '\n')
        same = replay == cur
        print(f'{rel:22} replay-equal={same}')
        if not same:
            ok = False
            a, b = replay, cur
            limit = min(len(a), len(b))
            at = next((k for k in range(limit) if a[k] != b[k]), limit)
            print('  first diff at', at)
            print('  REPLAY:', repr(a[max(0, at - 100):at + 100]))
            print('  CUR   :', repr(b[max(0, at - 100):at + 100]))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
