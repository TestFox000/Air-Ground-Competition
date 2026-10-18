import subprocess, re, collections, pathlib

REPO = pathlib.Path(r'D:\User\Desktop\比赛存档\v1\dzrobot_v2')
out = subprocess.run(['git', 'diff', '--unified=0', '--', 'dsh-plugin-dzrobot-ssh'],
                     cwd=REPO, capture_output=True, text=True, encoding='utf-8').stdout
cur = None
res = collections.OrderedDict()
for line in out.split('\n'):
    if line.startswith('+++ b/'):
        cur = line[6:]
        res[cur] = {'hunks': 0, 'ranges': [], 'del': 0, 'add': 0}
    m = re.match(r'@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@', line)
    if m and cur:
        b = int(m.group(2) or 1)
        c = int(m.group(3))
        d = int(m.group(4) or 1)
        res[cur]['hunks'] += 1
        res[cur]['del'] += b
        res[cur]['add'] += d
        res[cur]['ranges'].append((c, d))
for f, v in res.items():
    ranges = ', '.join(str(c) if d == 1 else f'{c}-{c + d - 1}' for c, d in v['ranges'])
    print(f"{f}: hunks={v['hunks']} -{v['del']}/+{v['add']} new-lines=[{ranges}]")
