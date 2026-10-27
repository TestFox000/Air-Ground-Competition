# -*- coding: utf-8 -*-
"""ft_v3b 训练驱动（二轮）：Set A 过采样数据集，其余配方同 ft_v1/ft_v3。

__main__ 守卫必须保留（spawn 二次训练坑）。
"""
from ultralytics import YOLO


def main():
    m = YOLO(r"C:\Users\29212\Documents\Trae-CN\default\dk_train_pack_20261016\models\hbb_d13_best.pt")
    m.train(
        data=r"C:\Users\29212\Documents\Trae-CN\default\ft_v3_official_20261026\data_14_ft_v3b\data_14_ft_v3b.yaml",
        epochs=60, imgsz=1280, batch=6, patience=20, device=0,
        workers=4,
        project=r"C:\Users\29212\Documents\Trae-CN\default\ft_v3_official_20261026\runs",
        name="ft_v3b",
    )


if __name__ == "__main__":
    main()
