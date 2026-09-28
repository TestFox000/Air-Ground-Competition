import sys
import types

import numpy as np
import pytest

from ag_core.backend import DummyBackend, OnnxBackend, RknnBackend, load_backend


def test_dummy_backend_single_scale_shape():
    b = DummyBackend(channels=9, anchors=8400)
    out = b.infer(np.zeros((1, 3, 640, 640), dtype=np.float32))
    assert out.shape == (1, 9, 8400)
    assert b.multi_scale is False


def test_dummy_backend_multi_scale_shape():
    b = DummyBackend(channels=18, multi_scale=True)
    outs = b.infer(np.zeros((1, 3, 640, 640), dtype=np.float32))
    assert isinstance(outs, list) and len(outs) == 3
    assert [o.shape[2] for o in outs] == [80, 40, 20]
    assert b.multi_scale is True


def test_dummy_backend_close_is_safe():
    DummyBackend().close()


def test_unknown_suffix_raises():
    with pytest.raises(ValueError):
        load_backend("model.weights")


def test_load_backend_selects_onnx(monkeypatch):
    class _Named:
        name = "images"

    class FakeSession:
        def __init__(self, path, providers=None):
            self.path = path
            self.providers = providers

        def get_outputs(self):
            return [_Named()]

        def get_inputs(self):
            return [_Named()]

        def run(self, names, feed):
            return [np.zeros((1, 9, 8400), dtype=np.float32)]

    fake = types.ModuleType("onnxruntime")
    fake.InferenceSession = FakeSession
    monkeypatch.setitem(sys.modules, "onnxruntime", fake)

    b = load_backend("model.onnx")
    assert isinstance(b, OnnxBackend)
    assert b.multi_scale is False
    out = b.infer(np.zeros((1, 3, 640, 640), dtype=np.float32))
    assert out.shape == (1, 9, 8400)


def test_onnx_backend_multi_scale_when_two_outputs(monkeypatch):
    class _Named:
        name = "o"

    class FakeSession:
        def __init__(self, path, providers=None):
            pass

        def get_outputs(self):
            return [_Named(), _Named()]

        def get_inputs(self):
            return [_Named()]

        def run(self, names, feed):
            return [np.zeros((1, 18, 80, 80), dtype=np.float32),
                    np.zeros((1, 18, 40, 40), dtype=np.float32)]

    fake = types.ModuleType("onnxruntime")
    fake.InferenceSession = FakeSession
    monkeypatch.setitem(sys.modules, "onnxruntime", fake)

    b = OnnxBackend("m.onnx")
    assert b.multi_scale is True
    assert len(b.infer(np.zeros((1, 3, 640, 640), dtype=np.float32))) == 2


def test_load_backend_selects_rknn(monkeypatch):
    class FakeRKNNLite:
        def __init__(self):
            self.released = False

        def load_rknn(self, path):
            return 0

        def init_runtime(self, core_mask=None):
            return 0

        def inference(self, inputs=None):
            return [np.zeros((1, 9, 8400), dtype=np.float32)]

        def release(self):
            self.released = True

    api = types.ModuleType("rknnlite.api")
    api.RKNNLite = FakeRKNNLite
    rknnlite = types.ModuleType("rknnlite")
    rknnlite.api = api
    monkeypatch.setitem(sys.modules, "rknnlite", rknnlite)
    monkeypatch.setitem(sys.modules, "rknnlite.api", api)

    b = load_backend("best_new.rknn")
    assert isinstance(b, RknnBackend)
    assert b.infer(np.zeros((1, 3, 640, 640), dtype=np.float32)).shape == (1, 9, 8400)
    b.close()


def test_rknn_load_failure_raises(monkeypatch):
    class BadRKNN:
        def load_rknn(self, path):
            return -1

        def init_runtime(self, core_mask=None):
            return 0

    api = types.ModuleType("rknnlite.api")
    api.RKNNLite = BadRKNN
    rknnlite = types.ModuleType("rknnlite")
    rknnlite.api = api
    monkeypatch.setitem(sys.modules, "rknnlite", rknnlite)
    monkeypatch.setitem(sys.modules, "rknnlite.api", api)

    with pytest.raises(RuntimeError):
        load_backend("bad.rknn")


def test_rknn_close_swallows_errors(monkeypatch):
    class Boom:
        def load_rknn(self, path):
            return 0

        def init_runtime(self, core_mask=None):
            return 0

        def release(self):
            raise RuntimeError("release failed")

    api = types.ModuleType("rknnlite.api")
    api.RKNNLite = Boom
    rknnlite = types.ModuleType("rknnlite")
    rknnlite.api = api
    monkeypatch.setitem(sys.modules, "rknnlite", rknnlite)
    monkeypatch.setitem(sys.modules, "rknnlite.api", api)

    b = load_backend("m.rknn")
    b.close()  # 不应抛出
