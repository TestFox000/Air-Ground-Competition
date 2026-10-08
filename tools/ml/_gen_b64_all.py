import base64, json, os
from PIL import Image
import io

review_dir = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\mine_official\review"
labels_dir = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\mine_official\labels"
out_json = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\mine_official\_b64_all.json"

items = []
files = sorted([f for f in os.listdir(review_dir) if f.endswith(".jpg")])
for fn in files:
    stem = fn[:-4]
    img = Image.open(os.path.join(review_dir, fn))
    w, h = img.size
    # thumbnail
    img.thumbnail((160, 160))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=55)
    b64 = base64.b64encode(buf.getvalue()).decode()
    # count boxes
    label_path = os.path.join(labels_dir, stem + ".txt")
    nbox = 0
    if os.path.exists(label_path):
        with open(label_path) as f:
            nbox = sum(1 for line in f if line.strip())
    items.append({"name": stem, "b64": b64, "nbox": nbox, "w": w, "h": h})

with open(out_json, "w") as f:
    json.dump(items, f)
print(f"saved {len(items)} items to {out_json}")
print(f"total b64 size: {sum(len(x['b64']) for x in items)} chars")
