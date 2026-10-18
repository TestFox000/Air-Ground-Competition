import sys
path, a, b = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
for i, l in enumerate(open(path, encoding='utf-8').read().splitlines(), 1):
    if a <= i <= b:
        print(f"{i:5d}: {l}")
