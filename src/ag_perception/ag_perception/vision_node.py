"""视觉推理节点：订阅相机图像 -> 模型推理 -> 发布 DetectionArray。

支持的模型布局：
    anchor_free  : best_new.rknn / rest_clean.rknn（Ultralytics v8/v11 导出，[1,C,8400]）
    anchored     : light_det.rknn（官方 YOLOv5 风格三尺度输出）

⚠️ kpt_unit / cls_activation 两个参数必须先用 tools/verify_decode.py 在真机上校准一次，
   不同 Ultralytics 版本的 pose 导出行为不一致。
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from ag_core.backend import DummyBackend, load_backend
from ag_core.blackbox import EV, BlackBox
from ag_core.postprocess import (
    decode_anchor_free,
    decode_anchored,
    letterbox_params,
    scale_coords,
)
from ag_interfaces.msg import Detection, DetectionArray

try:  # OpenCV 仅用于缩放，缺失时退化为最近邻
    import cv2
except Exception:  # pragma: no cover
    cv2 = None


def image_to_bgr(msg: Image) -> np.ndarray:
    """把 sensor_msgs/Image 转成 (H,W,3) uint8 BGR，不依赖 cv_bridge。"""
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = (msg.encoding or "").lower()

    if enc in ("bgr8", "bgr;8"):
        return buf.reshape(h, w, 3)
    if enc in ("rgb8", "rgb;8"):
        return buf.reshape(h, w, 3)[:, :, ::-1]
    if enc in ("yuyv", "yuyv2", "yuv422"):
        yuyv = buf.reshape(h, w, 2)
        # 手动拆 YUYV：每 4 字节 = Y0 U Y1 V
        packed = yuyv.reshape(h, w // 2, 4)
        y0 = packed[:, :, 0].astype(np.float32)
        uu = packed[:, :, 1].astype(np.float32)
        y1 = packed[:, :, 2].astype(np.float32)
        vv = packed[:, :, 3].astype(np.float32)
        yy = np.empty((h, w), dtype=np.float32)
        yy[:, 0::2] = y0
        yy[:, 1::2] = y1
        uu = np.repeat(uu, 2, axis=1)
        vv = np.repeat(vv, 2, axis=1)
        b = yy + 1.772 * (uu - 128.0)
        g = yy - 0.344136 * (uu - 128.0) - 0.714136 * (vv - 128.0)
        r = yy + 1.402 * (vv - 128.0)
        out = np.stack([b, g, r], axis=-1)
        return np.clip(out, 0, 255).astype(np.uint8)
    if enc in ("mono8", "8uc1"):
        g = buf.reshape(h, w)
        return np.stack([g, g, g], axis=-1)
    raise ValueError(f"暂不支持的图像编码: {msg.encoding}")


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("layout", "anchor_free")      # anchor_free | anchored
        self.declare_parameter("nc", 2)
        self.declare_parameter("nkpt", 1)
        self.declare_parameter("img_size", 640)
        self.declare_parameter("conf_thres", 0.50)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("cls_activation", "sigmoid")  # sigmoid | raw
        self.declare_parameter("kpt_unit", "pixel")          # pixel | grid
        self.declare_parameter("input_layout", "nchw")       # nchw | nhwc (RKNN 常用 NHWC)
        self.declare_parameter("input_topic", "/image_raw")
        self.declare_parameter("output_topic", "/perception/detections")
        self.declare_parameter("publish_original_coords", True)
        # ---- 相机断流看门狗 ----
        # 真机踩过：官方 usb_cam 崩掉（SIGABRT）之后，det_node 只会无限刷
        # "waiting for /image_raw"，**没有一处显式报错**；现场表现就是
        # "模型在跑但什么都检测不到"，靠翻节点日志才找得到原因。
        self.declare_parameter("max_image_gap_s", 5.0)   # 超过这么久没图就报；0 = 不检测
        self.declare_parameter("watchdog_period_s", 1.0)
        # ---- 黑匣子 ----
        self.declare_parameter("log_enabled", True)
        self.declare_parameter("log_dir", "")
        self.declare_parameter("log_level", "info")

        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.model_path = p("model_path")
        self.layout = p("layout")
        self.nc = int(p("nc"))
        self.nkpt = int(p("nkpt"))
        self.img_size = int(p("img_size"))
        self.conf = float(p("conf_thres"))
        self.iou = float(p("iou_thres"))
        self.cls_activation = p("cls_activation")
        self.kpt_unit = p("kpt_unit")
        self.input_layout = p("input_layout")
        self.publish_original_coords = bool(p("publish_original_coords"))

        self.backend = self._make_backend()
        self.pub = self.create_publisher(DetectionArray, p("output_topic"), 10)
        self.sub = self.create_subscription(Image, p("input_topic"), self.cb, 5)

        self.max_gap = float(p("max_image_gap_s"))
        self._images = 0
        self._last_img_t = 0.0          # 0 = 一帧都还没收到
        self._stale = False
        self._t0 = time.monotonic()
        if self.max_gap > 0:
            self.create_timer(float(p("watchdog_period_s")), self.cb_watchdog)

        self.bb = BlackBox.from_params(self)
        # 解码参数必须进日志：`cls_activation` / `kpt_unit` 一旦配错，
        # 现象是"模型在跑但什么都检测不到"，光看 ROS 日志看不出来。
        self.bb.log(EV.VISION_MODEL, model=self.model_path or "<dummy>",
                    layout=self.layout, nc=self.nc, nkpt=self.nkpt,
                    img_size=self.img_size, conf_thres=self.conf, iou_thres=self.iou,
                    cls_activation=self.cls_activation, kpt_unit=self.kpt_unit,
                    input_layout=self.input_layout,
                    backend=type(self.backend).__name__)
        self.get_logger().info(
            f"vision_node 就绪: model={self.model_path or '<dummy>'} layout={self.layout} "
            f"nc={self.nc} nkpt={self.nkpt}"
        )

    # ---------------- 后端 ----------------
    def _make_backend(self):
        if not self.model_path:
            self.get_logger().warning("未指定 model_path，使用 DummyBackend（输出恒为 0）")
            return DummyBackend(channels=4 + self.nc + 3 * self.nkpt)
        try:
            return load_backend(self.model_path)
        except Exception as e:  # 板端缺库时不要让节点崩溃
            self.get_logger().error(f"后端加载失败({e})，回退 DummyBackend")
            return DummyBackend(channels=4 + self.nc + 3 * self.nkpt)

    # ---------------- 预处理 ----------------
    def _letterbox(self, img: np.ndarray):
        h, w = img.shape[:2]
        scale, pad_x, pad_y, nw, nh = letterbox_params(h, w, self.img_size)
        pad_x_i, pad_y_i = int(round(pad_x)), int(round(pad_y))
        if cv2 is not None:
            resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        else:  # 退化实现，仅保证形状正确
            yi = (np.arange(nh) * (h / nh)).astype(int)
            xi = (np.arange(nw) * (w / nw)).astype(int)
            resized = img[yi[:, None], xi[None, :]]
        canvas = np.full((self.img_size, self.img_size, 3), 114, dtype=np.uint8)
        canvas[pad_y_i : pad_y_i + nh, pad_x_i : pad_x_i + nw] = resized
        return canvas, scale, pad_x, pad_y

    def _to_blob(self, canvas: np.ndarray) -> np.ndarray:
        x = canvas.astype(np.float32) / 255.0
        if self.input_layout == "nhwc":
            return np.ascontiguousarray(x[None, ...])
        return np.ascontiguousarray(x.transpose(2, 0, 1)[None, ...])

    # ---------------- 相机看门狗 ----------------
    def cb_watchdog(self) -> None:
        """没有图像时必须**显式**报出来。

        官方 `usb_cam` 崩溃后整个链路是"静默"的：det_node 无限等 /image_raw，
        我们的节点也安静得不发一帧检测。现场只会看到"什么都检测不到"，
        而要确认"是不是相机挂了"得去翻节点日志 —— 太慢。这里把它变成一条事件。
        """
        now = time.monotonic()
        if self._last_img_t <= 0:
            # 启动后要先给相机一点时间，别一起来就报警
            gap = now - self._t0
            never = True
        else:
            gap = now - self._last_img_t
            never = False

        if gap > self.max_gap:
            if not self._stale:
                self._stale = True
                self.bb.log(EV.VISION_STALE, level="error",
                            images=self._images, never=never,
                            gap_s=round(gap, 1),
                            note=("从来没收到过图像" if never else "相机断流"))
                self.get_logger().error(
                    f"{'从来没收到过图像' if never else '相机断流'} "
                    f"{gap:.1f}s（阈值 {self.max_gap}s，已收到 {self._images} 帧）—— "
                    "检查 usb_cam 是否还在、`ros2 topic hz /image_raw` 有没有帧"
                )
            return
        if self._stale:
            self._stale = False
            self.bb.log(EV.VISION_STALE, level="info", images=self._images,
                        never=False, gap_s=round(gap, 1), note="图像恢复")
            self.get_logger().info(f"图像恢复（已收到 {self._images} 帧）")

    # ---------------- 回调 ----------------
    def cb(self, msg: Image) -> None:
        self._images += 1
        self._last_img_t = time.monotonic()
        try:
            img = image_to_bgr(msg)
        except Exception as e:
            self.get_logger().warning(f"图像解码失败: {e}")
            return

        canvas, scale, pad_x, pad_y = self._letterbox(img)
        blob = self._to_blob(canvas)
        raw = self.backend.infer(blob)

        if self.layout == "anchored":
            dets = decode_anchored(
                raw, conf_thres=self.conf, iou_thres=self.iou,
                img_size=self.img_size, cls_activation=self.cls_activation,
            )
        else:
            dets = decode_anchor_free(
                raw, nc=self.nc, nkpt=self.nkpt, conf_thres=self.conf, iou_thres=self.iou,
                img_size=self.img_size, cls_activation=self.cls_activation, kpt_unit=self.kpt_unit,
            )

        out = DetectionArray()
        out.header = msg.header
        for d in dets:
            m = Detection()
            m.cls = int(d.cls)
            m.score = float(d.score)
            if self.publish_original_coords:
                box = scale_coords(
                    np.array([[d.x1, d.y1, d.x2, d.y2]]), scale, pad_x, pad_y,
                    clip_to=(img.shape[1], img.shape[0]),
                )[0]
                x1, y1, x2, y2 = box
            else:
                x1, y1, x2, y2 = d.x1, d.y1, d.x2, d.y2
            m.x1, m.y1, m.x2, m.y2 = float(x1), float(y1), float(x2), float(y2)

            if d.kpts is not None and len(d.kpts) > 0:
                kx, ky = scale_coords(
                    np.array([[d.kpts[0][0], d.kpts[0][1]]]), scale, pad_x, pad_y,
                    clip_to=(img.shape[1], img.shape[0]),
                )[0]
                m.kpt_x, m.kpt_y = float(kx), float(ky)
                m.kpt_conf = float(d.kpts[0][2])
            else:
                m.kpt_x = m.kpt_y = -1.0
                m.kpt_conf = 0.0
            out.detections.append(m)

        self.pub.publish(out)

        # 相机是 ~30fps，逐帧写会把时间轴淹掉；按秒采样就够看出
        # "到底有没有检出目标、置信度大概多少"。
        if self.bb.throttled("detect", 1.0):
            self.bb.log(EV.VISION_DETECT, count=len(out.detections),
                        cls=[int(d.cls) for d in dets][:20],
                        scores=[round(float(d.score), 2) for d in dets][:20])


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.bb.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
