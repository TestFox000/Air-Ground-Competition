import onnxruntime as ort, numpy as np
from PIL import Image
from pathlib import Path

sess = ort.InferenceSession(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\models\release\cell_classifier.onnx")
letters = "ABCDEFGHIJKLMN"
crops = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\datasets\cell_labels\crops")

tests = ["A1", "B2", "C3", "D5", "E10", "F11", "G7", "H8", "N1", "N11"]
for tid in tests:
    im = Image.open(crops / f"{tid}.png").convert("RGB").resize((96, 96))
    x = np.array(im).astype(np.float32) / 255.0
    x = (x - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]
    x = x.transpose(2, 0, 1)[None].astype(np.float32)
    l, r = sess.run(None, {"images": x})
    li, ri = int(l.argmax()), int(r.argmax())
    pred = f"{letters[li]}{ri+1}"
    ok = "OK" if pred == tid else "WRONG"
    print(f"{ok:5s} {tid} -> {pred}")
