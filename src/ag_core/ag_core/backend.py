"""模型推理后端抽象：让同一套代码能在 PC（onnxruntime）与板端（RKNN Lite）运行。

后端选择由模型文件后缀自动推断：
    *.rknn -> RknnBackend（需要 from rknnlite.api import RKNNLite，仅 RK3588S 可用）
    *.onnx -> OnnxBackend（需要 onnxruntime，PC 调试用）
    其他    -> DummyBackend（返回全零，用于跑通数据流与单测）
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Union

import numpy as np

__all__ = ["BaseBackend", "DummyBackend", "OnnxBackend", "RknnBackend", "load_backend"]


class BaseBackend(ABC):
    """约定：infer() 返回模型原始输出。

    anchor-free 模型返回单个 ndarray，形状 (1, C, N)；
    anchor-based（YOLOv5 三尺度）返回 list[[(1, C, H, W), ...]]。
    """

    #: 输出是否为多尺度（YOLOv5 风格）
    multi_scale: bool = False

    @abstractmethod
    def infer(self, blob: np.ndarray) -> Union[np.ndarray, List[np.ndarray]]:
        ...

    def close(self) -> None:  # pragma: no cover - 默认空实现
        pass


class DummyBackend(BaseBackend):
    """返回全零输出，用于无模型环境下验证数据流。"""

    def __init__(self, channels: int = 9, anchors: int = 8400, multi_scale: bool = False):
        self.channels = channels
        self.anchors = anchors
        self.multi_scale = multi_scale

    def infer(self, blob: np.ndarray):
        if self.multi_scale:
            return [np.zeros((1, self.channels, s, s), dtype=np.float32) for s in (80, 40, 20)]
        return np.zeros((1, self.channels, self.anchors), dtype=np.float32)


class OnnxBackend(BaseBackend):
    def __init__(self, path: str, providers=None):
        import onnxruntime as ort  # 局部导入，PC 端才有

        providers = providers or ["CPUExecutionProvider"]
        self.sess = ort.InferenceSession(path, providers=providers)
        outs = self.sess.get_outputs()
        self.multi_scale = len(outs) > 1
        self.out_names = [o.name for o in outs]

    def infer(self, blob: np.ndarray):
        res = self.sess.run(self.out_names, {self.sess.get_inputs()[0].name: blob})
        return res if self.multi_scale else res[0]


class RknnBackend(BaseBackend):
    """RKNN Lite 后端（RK3588S / rknn-toolkit2-lite）。"""

    def __init__(self, path: str, core_mask=None):
        from rknnlite.api import RKNNLite  # 仅板端可用

        self.rk = RKNNLite()
        if self.rk.load_rknn(path) != 0:
            raise RuntimeError(f"RKNN 加载失败: {path}")
        if self.rk.init_runtime(core_mask=core_mask) != 0:
            raise RuntimeError("RKNN runtime 初始化失败")
        self.multi_scale = False

    def infer(self, blob: np.ndarray):
        return self.rk.inference(inputs=[blob])[0]

    def close(self) -> None:
        try:
            self.rk.release()
        except Exception:
            pass


def load_backend(path: str, **kw) -> BaseBackend:
    p = str(path).lower()
    if p.endswith(".rknn"):
        return RknnBackend(path, **kw)
    if p.endswith(".onnx"):
        return OnnxBackend(path, **kw)
    raise ValueError(f"无法推断后端类型: {path}")
