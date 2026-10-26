# -*- coding: utf-8 -*-
"""ft_v3 训练驱动：ft_v1 同配方（d13 基底/60轮/1280/batch6）。

数据 = data_14_ft_v2(hue 红面扩增) + 官方爆炸物 Set A(66 掩膜伪框近景)
     + Set B(240 泊松 copypaste 合成)；val 与 data_14 完全一致保跨版可比。

Windows spawn 语义下 DataLoader worker 会再导入本模块——训练调用必须挂在
__main__ 守卫内（ft_v2 三次炸的根因之一）。
"""
from ultralytics import YOLO


def main():
    m = YOLO(r"C:\Users\29212\Documents\Trae-CN\default\dk_train_pack_20261016\models\hbb_d13_best.pt")
    m.train(
        data=r"C:\Users\29212\Documents\Trae-CN\default\ft_v3_official_20261026\data_14_ft_v3\data_14_ft_v3.yaml",
        epochs=60, imgsz=1280, batch=6, patience=20, device=0,
        workers=4,
        project=r"C:\Users\29212\Documents\Trae-CN\default\ft_v3_official_20261026\runs",
        name="ft_v3",
    )


if __name__ == "__main__":
    main()
