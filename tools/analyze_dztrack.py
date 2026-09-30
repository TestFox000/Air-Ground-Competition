import sys, re
sys.stdout.reconfigure(encoding="utf-8")
b = r"C:\Users\29212\Desktop\人工智能算法精英大赛-空地协同侦排挑战赛\车端香橙派ros自主导航程序\dzrobot-bisai-20260617\src\dztrack\src"
t = open(b + r"\control.cpp", encoding="utf-8", errors="replace").read()
print("control.cpp 行数", t.count("\n"))

print("===== 话题 =====")
for m in re.finditer(r'create_(subscription|publisher)\s*<([^>]*)>\s*\(\s*"([^"]+)"', t):
    print("  ", m.group(1), m.group(3), "|", m.group(2).strip()[:45])

print("===== 路径来源与参数 =====")
for i, l in enumerate(t.splitlines()):
    if re.search(r"mapping|saveMapping|naviPoint|declare|get_parameter|pursuitSpeed|savePath|amcl|odom|/tf", l):
        s = l.strip()
        if s:
            print("%5d: %s" % (i + 1, s[:130]))
