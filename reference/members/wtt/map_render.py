# -*- coding: utf-8 -*-
"""实时地图渲染(可选模块): 用 Pillow 把当前二维矩阵 + 路径画成 PNG/GIF。
由 mission_planner.py 的 --map-out / --map-dir / --gif-out 调用;
未安装 Pillow 时自动跳过。支持高亮"刚填充的扫描块", 用于展示逐块建图过程。
"""


def _font(size):
    try:
        from PIL import ImageFont
    except Exception:
        return None
    for fp in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\msyhbd.ttc",
               r"C:\Windows\Fonts\simhei.ttf", r"C:\Windows\Fonts\simsun.ttc"):
        try:
            return ImageFont.truetype(fp, size)
        except Exception:
            continue
    try:
        return ImageFont.load_default()
    except Exception:
        return None


def render_map_image(field, route=None, roles=None, car=None, title="",
                     highlight=(), scan_blocks=(), arrows=None):
    """渲染当前地图 + 路径, 返回 PIL Image; highlight=刚填充的块(黄色高亮框);
    scan_blocks=无人机 S 型扫描路线块序列(蓝色虚线+箭头);
    arrows=桥洞可通行通道 {rc: True}(顶部通行状态标识, 绿色标记)"""
    try:
        from PIL import Image, ImageDraw
    except Exception:
        return None
    import mission_planner as M

    CS = 46
    MX, MY = 46, 78
    W = MX * 2 + M.COLS * CS
    H = MY + M.ROWS * CS + 46
    img = Image.new("RGB", (W, H), (245, 246, 250))
    d = ImageDraw.Draw(img)
    f_t = _font(16)
    f_b = _font(12)
    f_c = _font(10)
    arrows = arrows or {}

    def center(rc):
        return (MX + rc[1] * CS + CS // 2, MY + rc[0] * CS + CS // 2)

    # 无人机 S 型扫描路线(蓝色虚线 + 前进箭头)
    if scan_blocks:
        pts = []
        for b in scan_blocks:
            rr = sum(rc[0] for rc in b) / len(b)
            cc = sum(rc[1] for rc in b) / len(b)
            pts.append((MX + cc * CS, MY + rr * CS))
        for i in range(len(pts) - 1):
            x0, y0 = pts[i]
            x1, y1 = pts[i + 1]
            seg = 8
            dist = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
            n = max(1, int(dist / (seg * 2)))
            for k in range(n):
                t0, t1 = k * 2 * seg / dist, (k * 2 + 1) * seg / dist
                d.line([(x0 + (x1 - x0) * min(t0, 1), y0 + (y1 - y0) * min(t0, 1)),
                        (x0 + (x1 - x0) * min(t1, 1), y0 + (y1 - y0) * min(t1, 1))],
                       fill=(66, 135, 245), width=3)
            dx, dy = x1 - x0, y1 - y0
            L = max((dx * dx + dy * dy) ** 0.5, 1e-6)
            ux, uy = dx / L, dy / L
            d.polygon([(x1, y1), (x1 - 8 * ux + 4 * uy, y1 - 8 * uy - 4 * ux),
                       (x1 - 8 * ux - 4 * uy, y1 - 8 * uy + 4 * ux)],
                      fill=(66, 135, 245))

    FILL = {M.UNKNOWN: (185, 185, 185), M.EMPTY: (255, 255, 255),
            M.BUCKET: (211, 92, 92), M.EXPLOSIVE: (255, 206, 84),
            M.TARGET: (232, 118, 118), M.FRIEND: (186, 140, 220),
            M.LINE: (96, 178, 96)}
    route_set = set(route or [])
    roles = roles or {}

    for r in range(M.ROWS):
        for c in range(M.COLS):
            rc = (r, c)
            x, y = MX + c * CS, MY + r * CS
            st = field.state(rc)
            color = FILL.get(st, (200, 200, 200))
            if field.is_bridge(rc) and st == M.EMPTY:
                color = (206, 206, 206)
            d.rectangle([x, y, x + CS, y + CS], fill=color,
                        outline=(214, 214, 214))
            if st == M.TARGET and field.blue(rc):
                dr, dc = field.blue(rc)
                if dc == 1:
                    d.line([x + CS - 2, y + 3, x + CS - 2, y + CS - 3],
                           fill=(25, 120, 220), width=5)
                elif dc == -1:
                    d.line([x + 2, y + 3, x + 2, y + CS - 3],
                           fill=(25, 120, 220), width=5)
                elif dr == 1:
                    d.line([x + 3, y + CS - 2, x + CS - 3, y + CS - 2],
                           fill=(25, 120, 220), width=5)
                else:
                    d.line([x + 3, y + 2, x + CS - 3, y + 2],
                           fill=(25, 120, 220), width=5)
            if st == M.EXPLOSIVE:          # 爆炸物位于格心
                cx, cy = x + CS // 2, y + CS // 2
                d.ellipse([cx - 6, cy - 6, cx + 6, cy + 6], fill=(30, 30, 30))

    # 官方横向挡板 D7~N7(第 6/7 行边界, 黄色带); E 版坐标系转置: 第6/7列(索引5↔6)
    # 之间, D~N 行严禁跨越; A/B/C 行无挡板
    lo_row = M.ROW_LETTERS.index(M.BARRIER_LO_ROW)
    hi_row = M.ROW_LETTERS.index(M.BARRIER_HI_ROW)
    b_col = max(M.BARRIER_COLS) - 1    # 挡板线画在禁跨列界上(与 barrier_blocks_cross 同一条线)
    bx = MX + b_col * CS
    d.rectangle([bx - 4, MY + lo_row * CS, bx + 4, MY + (hi_row + 1) * CS],
                fill=(255, 214, 64))
    d.text((bx - 8, MY + (lo_row + hi_row + 1) * CS // 2), "挡板",
           font=f_c, fill=(120, 90, 0))

    # 刚填充的扫描块: 黄色高亮框
    for rc in highlight:
        x, y = MX + rc[1] * CS, MY + rc[0] * CS
        d.rectangle([x - 1, y - 1, x + CS + 1, y + CS + 1],
                    outline=(255, 170, 0), width=4)

    # 桥洞可通行通道(顶部通行状态标识, 绿色)
    for rc in arrows:
        x, y = MX + rc[1] * CS, MY + rc[0] * CS
        d.rectangle([x + 3, y + 3, x + CS - 3, y + CS - 3],
                    fill=(20, 150, 40))
        d.text((x + 9, y + 10), "通", font=f_b, fill=(255, 255, 255))

    # 路径(格心连线 + 拐弯箭头)
    for i in range(len(route) - 1):
        d.line([center(route[i]), center(route[i + 1])], fill=(150, 0, 200), width=5)
    for i in range(1, len(route) - 1):
        din = M.sub(route[i], route[i - 1])
        dout = M.sub(route[i + 1], route[i])
        if din != dout:
            cx, cy = center(route[i])
            dr, dc = dout
            d.polygon([(cx, cy), (cx - 6 * dr, cy - 6 * dc),
                       (cx - 3 * dr, cy - 3 * dc)], fill=(100, 0, 150))
    # 小车
    if car is not None:
        cx, cy = center(car)
        d.ellipse([cx - 9, cy - 9, cx + 9, cy + 9], fill=(46, 160, 67))
    # 角色标记
    for rc, role in roles.items():
        if role == "stop_face":
            cx, cy = center(rc)
            d.ellipse([cx - 9, cy - 9, cx + 9, cy + 9], outline=(25, 120, 220), width=3)
        elif role == "hit":
            d.text((center(rc)[0] - 12, center(rc)[1] - 26), "敌", font=f_b,
                   fill=(180, 20, 20))
        elif role == "return":
            cx, cy = center(rc)
            d.ellipse([cx - 9, cy - 9, cx + 9, cy + 9], outline=(30, 30, 30), width=3)

    # 行列标号
    for r in range(M.ROWS):
        d.text((MX - 22, MY + r * CS + CS // 2 - 6), M.ROW_LETTERS[r],
               font=f_b, fill=(60, 60, 60))
    for c in range(M.COLS):
        d.text((MX + c * CS + CS // 2 - 5, MY - 18), str(c + 1),
               font=f_b, fill=(60, 60, 60))
    # 标题
    if f_t is not None:
        d.text((MX, 12), title, font=f_t, fill=(30, 30, 30))
    # 图例
    ly = MY + M.ROWS * CS + 16
    leg = [((255, 214, 64), "挡板线(禁跨)"), ((211, 92, 92), "障碍"),
           ((255, 206, 84), "地雷"), ((232, 118, 118), "敌(蓝面)"),
           ((206, 206, 206), "桥洞"), ((20, 150, 40), "可通行通道"),
           ((150, 0, 200), "路径"), ((46, 160, 67), "小车"),
           ((255, 170, 0), "刚填充块"), ((66, 135, 245), "S型扫描线")]
    lx = MX
    for col, name in leg:
        d.rectangle([lx, ly, lx + 14, ly + 14], fill=col, outline=(150, 150, 150))
        d.text((lx + 18, ly - 1), name, font=f_c, fill=(60, 60, 60))
        lx += 16 + (d.textlength(name, font=f_c) if f_c else 10) + 18
    return img


def render_map_png(field, route=None, roles=None, car=None, title="",
                   out_path="实时地图.png", highlight=(), arrows=None):
    """渲染并保存 PNG; 成功返回 True"""
    img = render_map_image(field, route, roles, car, title, highlight, arrows=arrows)
    if img is None:
        return False
    img.save(out_path)
    return True


def save_gif(frames, out_path, durations=None, default_ms=200):
    """把帧序列保存为循环播放的 GIF; 成功返回 True"""
    if not frames:
        return False
    dur = durations or [default_ms] * len(frames)
    try:
        frames[0].save(out_path, save_all=True, append_images=frames[1:],
                       duration=dur, loop=0, disposal=2)
        return True
    except Exception:
        return False
