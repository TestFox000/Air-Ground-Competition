"""验证「只改注释」：把 HEAD 版与工作区版都去掉注释后逐字节对比。

- JS：去 // 与 /* */（字符串/模板字面量内不当作注释）
- HTML：去 <!-- -->，并对 <script>/<style> 段分别按 JS/CSS 去注释
- 两侧行尾统一为 LF 后比较；相同 => 代码零变化（注释内容可任意变）
"""
import subprocess, pathlib, sys

REPO = pathlib.Path(r'D:\User\Desktop\比赛存档\v1\dzrobot_v2')
PLUGIN_REL = 'dsh-plugin-dzrobot-ssh'


def strip_js(s: str) -> str:
    out = []
    i, n = 0, len(s)
    while i < n:
        c = s[i]
        if c == '/' and i + 1 < n and s[i + 1] == '/':
            j = s.find('\n', i)
            i = n if j < 0 else j
            continue
        if c == '/' and i + 1 < n and s[i + 1] == '*':
            j = s.find('*/', i + 2)
            i = n if j < 0 else j + 2
            continue
        if c in ('"', "'", '`'):
            q = c
            j = i + 1
            while j < n:
                ch = s[j]
                if ch == '\\':
                    j += 2
                    continue
                if ch == q:
                    j += 1
                    break
                j += 1
            out.append(s[i:j])
            i = j
            continue
        out.append(c)
        i += 1
    return ''.join(out)


def strip_style(s: str) -> str:
    out = []
    i, n = 0, len(s)
    while i < n:
        if s.startswith('/*', i):
            j = s.find('*/', i + 2)
            i = n if j < 0 else j + 2
            continue
        out.append(s[i])
        i += 1
    return ''.join(out)


def strip_html(s: str) -> str:
    out = []
    i, n = 0, len(s)
    while i < n:
        if s.startswith('<!--', i):
            j = s.find('-->', i + 4)
            i = n if j < 0 else j + 3
            continue
        lower = s[i:i + 8].lower()
        if lower.startswith('<script') or lower.startswith('<style'):
            tag_end = s.find('>', i)
            if tag_end < 0:
                out.append(s[i:])
                break
            close = '</script' if lower.startswith('<script') else '</style'
            close_at = s.lower().find(close, tag_end)
            close_end = s.find('>', close_at) if close_at >= 0 else n
            if close_end < 0:
                close_end = n
            head = s[i:tag_end + 1]
            body = s[tag_end + 1:close_at if close_at >= 0 else n]
            tail = s[close_at if close_at >= 0 else n:close_end + 1]
            out.append(head)
            out.append(strip_js(body) if lower.startswith('<script') else strip_style(body))
            out.append(tail)
            i = close_end + 1
            continue
        out.append(s[i])
        i += 1
    return ''.join(out)


def norm(text: str) -> str:
    return text.replace('\r\n', '\n').replace('\r', '\n')


def head_of(rel: str) -> str:
    return subprocess.run(['git', 'show', f'HEAD:{PLUGIN_REL}/{rel}'], cwd=REPO,
                          capture_output=True, check=True).stdout.decode('utf-8')


def main():
    files = sys.argv[1:] or ['sim.html', 'index.js', 'src/simbench.js', 'src/dronesim.js']
    ok = True
    for rel in files:
        old = norm(head_of(rel))
        new = norm((REPO / PLUGIN_REL / rel).read_text(encoding='utf-8'))
        strip = strip_html if rel.endswith('.html') else strip_js
        a, b = strip(old), strip(new)
        same = a == b
        print(f'{rel:22} code-identical={same}')
        if not same:
            ok = False
            # 找到第一处差异，打印上下文
            limit = min(len(a), len(b))
            at = next((k for k in range(limit) if a[k] != b[k]), limit)
            print('  first diff at', at)
            print('  OLD:', repr(a[max(0, at - 120):at + 120]))
            print('  NEW:', repr(b[max(0, at - 120):at + 120]))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
