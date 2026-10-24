#!/usr/bin/env python3
# coding=utf-8
r"""tf_compat —— numpy-only 的最小 tf_transformations 替身（AGC-16）。

## 为什么要这个文件

`transform_fusion.py` 原来的兜底是：

    try:
        import tf_transformations
    except ImportError:
        from tf2_ros import transformations as tf_transformations   # ← 这句必然炸

ROS2（humble 及以前）**没有** `tf2_ros.transformations` 这个模块（它是 ROS1 `tf.transformations` 的
历史遗留）。于是节点在 `import` 阶段就抛 `ImportError: cannot import name 'transformations' from 'tf2_ros'`
⇒ 进程立刻死 ⇒ `/localization` 永远没有发布者（这就是 D3 的一半根因：FAST-LIO 在跑、点云正常，
唯独定位话题不存在，现场观感与"一切正常"几乎无差别）。

## 范围：只实现真正用到的 4 个函数

`transform_fusion.py` 只调用这四个（grep 可核）：

| 函数 | 签名 | 返回 | 语义 |
|---|---|---|---|
| `translation_matrix` | `(vector) -> (4,4)` | 齐次平移矩阵 | `M[:3, 3] = vector[:3]` |
| `quaternion_matrix` | `([x, y, z, w]) -> (4,4)` | 齐次旋转矩阵 | 不做单位化假设：内部先归一化；模长≈0 时返回单位阵 |
| `translation_from_matrix` | `(4,4) -> (3,)` | 平移 | 直接取 `M[:3, 3]` |
| `quaternion_from_matrix` | `(4,4) -> (4,)` | 四元数 `[x, y, z, w]` | 旋转部分 → 单位四元数，规范化 `w >= 0` |

四元数一律 **`[x, y, z, w]`（w 在最后）**——与 ROS `geometry_msgs/Quaternion` 的字段序、
与 `transform_fusion.py` 里 `quat[0..3] -> rotation.x/y/z/w` 的用法一致。
矩阵一律 4×4 行主序齐次阵，`dtype=float64`。

## 与 tf_transformations 的数值关系（"不改行为"的口径）

* `translation_matrix` / `translation_from_matrix`：逐位等价（纯取值，没有算法自由度）。
* `quaternion_matrix`：与 tf 同一算法（归一化 + 外积展开），对单位四元数逐位可达
  ~1e-16 级一致（浮点求和顺序相同）。
* `quaternion_from_matrix`：与 tf 默认路径同类（由旋转阵构造对称 K 阵，取最大特征值对应的
  特征向量），数学上唯一到一个符号；本模块**统一规范化为 `w >= 0`**。`q` 与 `-q` 表示同一个
  旋转，所以车端/飞控拿到的姿态**等价**；`w == 0`（180° 旋转）时符号本身不唯一，属正常数学歧义。

## 边界（不做什么）

* 不 import rclpy / tf2_ros / scipy —— **只依赖 numpy 与 math**，任何平台可离线跑（Windows 已实测）。
* 不提供 euler/quaternion 互转、`compose_matrix`、`quaternion_slerp` 等未用到的 API：
  需要时另行加，不要在这里长成一个"迷你 transformations.py"。

复跑（离线、不部署机端）：

    python -B -X utf8 -m unittest discover -s dzsmartscout/src/FAST_LIO_LOCALIZATION_ROS2/scripts -p "test_tf_compat.py"
"""

from __future__ import annotations

import math

import numpy as np

__all__ = [
    "translation_matrix",
    "quaternion_matrix",
    "translation_from_matrix",
    "quaternion_from_matrix",
]

#: 与 tf_transformations 同口径的零判定阈值（4 * float eps）
_EPS: float = 4.0 * np.finfo(float).eps


def translation_matrix(vector) -> np.ndarray:
    """平移向量 → 4×4 齐次矩阵。

    >>> translation_matrix([1.0, 2.0, 3.0])[:3, 3].tolist()
    [1.0, 2.0, 3.0]
    """
    M = np.identity(4, dtype=np.float64)
    M[:3, 3] = np.asarray(vector, dtype=np.float64)[:3]
    return M


def translation_from_matrix(matrix) -> np.ndarray:
    """4×4 齐次矩阵 → 平移向量（长度 3 的 ndarray，copy）。

    >>> translation_from_matrix(translation_matrix([1.0, 2.0, 3.0])).tolist()
    [1.0, 2.0, 3.0]
    """
    M = np.asarray(matrix, dtype=np.float64)
    return np.array(M[:3, 3], dtype=np.float64, copy=True)


def quaternion_matrix(quaternion) -> np.ndarray:
    """四元数 [x, y, z, w] → 4×4 齐次旋转矩阵（内部归一化）。

    模长小于 _EPS 时返回单位阵（与 tf 同行为：坏四元数不炸，退化成无旋转）。

    >>> R = quaternion_matrix([0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    >>> np.allclose(R[:3, :3], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-12)
    True
    """
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)   # [x, y, z, w]
    nq = float(np.dot(q, q))
    if nq < _EPS:
        return np.identity(4, dtype=np.float64)
    # q *= sqrt(2 / |q|^2) 之后，外积 Mq[i, j] = 2 * qi * qj / |q|^2，
    # 下面每个元素就是教科书上的 2(xy-zw) 这一族，省掉重复的乘法。
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0 - q[1, 1] - q[2, 2],        q[0, 1] - q[2, 3],        q[0, 2] + q[1, 3], 0.0),
        (       q[0, 1] + q[2, 3], 1.0 - q[0, 0] - q[2, 2],        q[1, 2] - q[0, 3], 0.0),
        (       q[0, 2] - q[1, 3],        q[1, 2] + q[0, 3], 1.0 - q[0, 0] - q[1, 1], 0.0),
        (                    0.0,                     0.0,                     0.0, 1.0),
    ), dtype=np.float64)


def quaternion_from_matrix(matrix) -> np.ndarray:
    """4×4 齐次矩阵（只看旋转部分）→ 单位四元数 [x, y, z, w]，规范化为 w >= 0。

    做法与 tf 默认路径同类：由旋转阵构造对称 K 阵（只填下三角，numpy eigh 读下三角），
    取最大特征值对应的特征向量。相比"对角最大分量分支法"，它对接近 180° 的旋转更稳。

    >>> quaternion_from_matrix(np.identity(4)).tolist()
    [0.0, 0.0, 0.0, 1.0]
    """
    M = np.asarray(matrix, dtype=np.float64)[:4, :4]
    m00, m01, m02 = float(M[0, 0]), float(M[0, 1]), float(M[0, 2])
    m10, m11, m12 = float(M[1, 0]), float(M[1, 1]), float(M[1, 2])
    m20, m21, m22 = float(M[2, 0]), float(M[2, 1]), float(M[2, 2])

    K = np.array([
        [m00 - m11 - m22, 0.0, 0.0, 0.0],
        [m01 + m10, m11 - m00 - m22, 0.0, 0.0],
        [m02 + m20, m12 + m21, m22 - m00 - m11, 0.0],
        [m21 - m12, m02 - m20, m10 - m01, m00 + m11 + m22],
    ], dtype=np.float64)
    K /= 3.0
    eigvals, eigvecs = np.linalg.eigh(K)          # 升序；eigh 只用下三角（K 是上三角留 0 的对称填充）
    q = np.array(eigvecs[:, int(np.argmax(eigvals))], dtype=np.float64)

    # 归一化（特征向量本应已单位化；显式再除一次，防病态输入）
    n = float(np.linalg.norm(q))
    if not np.isfinite(n) or n < _EPS:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    q /= n
    if q[3] < 0.0:                                # 统一 w >= 0（±q 同一旋转，取规范代表）
        q = -q
    return q
