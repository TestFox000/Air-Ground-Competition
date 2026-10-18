"""诊断: 用frame_00020(东列强锚点)的东列间距, 裁决frame_00250 #6/#7/#8 的读数。

若20帧东列间距与250帧#6-#7/#7-#8间距同量级(~250-350px),
则250帧#6/#7/#8=H1/I1/J1读数正确(低空东列压缩), 几何外推崩溃才是错的。
"""
import json
import numpy as np

ROOT = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\cell_recog_render\label"


def centers(frame):
    b = json.load(open(rf"{ROOT}\{frame}\boxes.json", encoding="utf-8"))["boxes"]
    return np.array([[x["cx"], x["cy"]] for x in b])


def d(c, i, j):
    return round(float(np.linalg.norm(c[i] - c[j])), 1)


c20 = centers("frame_00020")
print("== frame_00020 东列间距 (16锚点拟合med残差2.9px, 可信) ==")
print(f"G1-H1: {d(c20,2,1)}  H1-I1: {d(c20,1,0)}")
print(f"L1-M1: {d(c20,3,4)}  M1-N1: {d(c20,4,5)}")
print(f"I2-L2: {d(c20,6,7)} (跨J,K)  L2-N2: {d(c20,7,8)} (跨M)")
print(f"L3-M3: {d(c20,11,12)}  M3-N3: {d(c20,12,13)}")
print(f"L4-M4: {d(c20,14,15)}  M4-N4: {d(c20,15,16)}")
print(f"H3-I3: {d(c20,9,10)}")

c250 = centers("frame_00250")
print("\n== frame_00250 待裁决框间距 ==")
print(f"#6-#7 (读H1-I1): {d(c250,6,7)}")
print(f"#7-#8 (读I1-J1): {d(c250,7,8)}")
print(f"#5-#6 (F1-?): {d(c250,5,6)}")
print(f"#14-#15 (G2-H2): {d(c250,14,15)}")
print(f"#18-#19 (读I3-H3): {d(c250,18,19)}")

# 20帧相邻列间距均值
e20 = [d(c20,3,4), d(c20,4,5), d(c20,11,12), d(c20,12,13), d(c20,14,15), d(c20,15,16)]
print(f"\n20帧东列相邻间距: {e20}  均值={np.mean(e20):.0f}px")
