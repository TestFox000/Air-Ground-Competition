
import onnxruntime as ort, numpy as np, cv2, os, collections
mp = r"D:\User\Text\xwechat_files\wxid_np5618xp9ee422_e3bb\msg\file\2026-10\无人机上机包_20261017\无人机上机包_20261017\02_D13与v4d模型链\models\hbb_d13.onnx"
names = ["paper","cell_label","car","target_red","target_blue","obstacle","bucket_red","bucket_green","bucket_cyan","car_start","car_end","uav_start","uav_end"]
sess = ort.InferenceSession(mp, providers=["CPUExecutionProvider"])

def run(path):
    img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None: return None
    h, w = img.shape[:2]
    s = min(1280.0/w, 1280.0/h)
    r = cv2.resize(img, (int(round(w*s)), int(round(h*s))))
    canvas = np.full((1280,1280,3), 114, np.uint8); canvas[:r.shape[0], :r.shape[1]] = r
    blob = canvas[:, :, ::-1].astype(np.float32)/255.0; blob = blob.transpose(2,0,1)[None]
    out = sess.run(None, {"images": blob})[0][0]
    dets = []
    for i in range(out.shape[1]):
        cs = out[4:, i]; c = int(np.argmax(cs)); sc = float(cs[c])
        if sc < 0.35: continue
        x, y, bw, bh = out[0,i], out[1,i], out[2,i], out[3,i]
        dets.append((names[c], sc, (x-bw/2)/s, (y-bh/2)/s, bw/s, bh/s))
    cnt = collections.Counter(d[0] for d in dets)
    print(f"== {os.path.basename(path)} {w}x{h}: {dict(cnt)}")
    return img, dets

base = r"D:\User\Desktop\比赛存档\v1\_tmp_session"
for p in [base + r"\t39_map_official\map_official.png", base + r"\t40_map_parse\map_new.png"]:
    res = run(p)
