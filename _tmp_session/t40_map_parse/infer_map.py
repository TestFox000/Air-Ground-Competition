
import onnxruntime as ort, numpy as np, cv2, sys, os
mp = r"D:\User\Text\xwechat_files\wxid_np5618xp9ee422_e3bb\msg\file\2026-10\无人机上机包_20261017\无人机上机包_20261017\02_D13与v4d模型链\models\hbb_d13.onnx"
names = ["paper","cell_label","car","target_red","target_blue","obstacle","bucket_red","bucket_green","bucket_cyan","car_start","car_end","uav_start","uav_end"]
sess = ort.InferenceSession(mp, providers=["CPUExecutionProvider"])

def infer(path):
    img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        print("MISSING", path); return
    h, w = img.shape[:2]
    s = min(1280.0/w, 1280.0/h)
    r = cv2.resize(img, (int(round(w*s)), int(round(h*s))))
    canvas = np.full((1280,1280,3), 114, np.uint8)
    canvas[:r.shape[0], :r.shape[1]] = r
    blob = canvas[:, :, ::-1].astype(np.float32)/255.0
    blob = blob.transpose(2,0,1)[None]
    out = sess.run(None, {"images": blob})[0][0]
    boxes, scores, classes = [], [], []
    for i in range(out.shape[1]):
        cls_scores = out[4:, i]
        c = int(np.argmax(cls_scores)); sc = float(cls_scores[c])
        if sc < 0.25: continue
        x, y, bw, bh = out[0,i], out[1,i], out[2,i], out[3,i]
        boxes.append([(x-bw/2)/s, (y-bh/2)/s, bw/s, bh/s]); scores.append(sc); classes.append(c)
    keep = []
    for c in range(13):
        idx = [i for i in range(len(classes)) if classes[i]==c]
        rects = [list(map(int, boxes[i])) for i in idx]
        scs = [scores[i] for i in idx]
        if rects:
            k = cv2.dnn.NMSBoxes(rects, scs, 0.25, 0.45)
            for j in np.array(k).flatten(): keep.append(idx[j])
    print(f"== {os.path.basename(path)}  ({w}x{h})  dets={len(keep)}")
    for k in sorted(keep, key=lambda i: -scores[i]):
        x,y,bw,bh = boxes[k]
        print(f"  {names[classes[k]]:12s} {scores[k]:.3f}  box=({x:.0f},{y:.0f},{bw:.0f},{bh:.0f})")

base = r"D:\User\Desktop\比赛存档\v1\_tmp_session"
for p in [base + r"\t39_map_official\map_official.png", base + r"\t40_map_parse\map_new.png"]:
    infer(p)
