import os
from PIL import Image
TMP = os.path.dirname(os.path.abspath(__file__))   # 证据自包含：与脚本同目录的 SVG/PNG
im = Image.open(os.path.join(TMP, "t2_target_board_compare.png")).convert("RGB")
PAD, TITLE, W = 12, 26, 888
def px(panel, sx, sy):
    x0 = PAD + panel*(W+PAD)
    return im.getpixel((x0 + int((sx-96)*6), TITLE + int((sy-96)*6)))
checks = []
def ck(name, got, want):
    ok = got == want
    checks.append(ok)
    print(("  PASS " if ok else "  FAIL ") + name + f" —— got {got} want {want}")
BLUE, RED = (47,127,208), (214,69,69)
ck("P1 新·横板上半 = 蓝", px(0,170,142), BLUE)
ck("P1 新·横板下半 = 红", px(0,170,164), RED)
ck("P2 新·竖板左半 = 蓝", px(1,145,170), BLUE)
ck("P2 新·竖板右半 = 红", px(1,160,170), RED)
ck("P2 竖板左上/左下同蓝（分色线竖直，不再按格切）", px(1,145,150), px(1,145,190))
ck("P3 旧·E5 整格蓝（上半）", px(2,145,145), BLUE)
ck("P3 旧·E5 整格蓝（下半）", px(2,145,165), BLUE)
ck("P3 旧·F5 整格红", px(2,185,145), RED)
print("\n像素断言：" + str(sum(checks)) + "/" + str(len(checks)))